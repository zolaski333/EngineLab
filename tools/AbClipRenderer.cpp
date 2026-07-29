// Blind A/B listening-clip renderer.
//
// This tool does NOT change the audio engine. It renders the real realtime path
// (RealtimeEngineAudio, default voicing) for a listenable RPM trajectory --
// stable low idle, rev-up into the rev limiter, then overrun decel -- and writes
// a 48 kHz 32-bit float WAV per engine. Clips are loudness-matched with an
// ITU-R BS.1770 integrated-LUFS measurement (mandatory: otherwise the louder
// clip is judged "better"), then laid out as neutrally named A/B pairs with a
// randomised assignment. The answer key is written OUTSIDE the listening folder.
//
// If a royalty-free reference recording is supplied per engine (--ref
// name=path) or through --reference-manifest, it is decoded, resampled to
// 48 kHz, duration-matched, loudness-matched and placed on the opposite side of
// the pair. Without one, the reference slot is left as a documented placeholder
// (see docs/audio-ab-listening.md); es2d could not be built locally (empty
// submodules), so no es2d clip is produced.

#include <enginelab/audio/RealtimeEngineAudio.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <juce_audio_basics/juce_audio_basics.h>
#include <juce_audio_formats/juce_audio_formats.h>
#include <juce_cryptography/juce_cryptography.h>
#include <juce_dsp/juce_dsp.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
using namespace enginelab;
using Json = nlohmann::json;

struct Clip final {
    std::vector<float> left, right;
    double sampleRate { 48'000.0 };
};

struct ReferenceSpec final {
    std::string key;
    std::filesystem::path file;
    std::string creator { "UNVERIFIED" };
    std::string license { "UNVERIFIED" };
    std::string licenseUrl;
    std::string distribution { "evaluation-only" };
    std::string sourceUrl;
    std::string previewUrl;
    std::string sha256;
    std::string sourceKind { "unknown" };
    std::string matchQuality { "unverified" };
    std::string notes;
    Json sourceEngine { Json::object() };
    Json operatingConditions { Json::array() };
    Json rpm { Json::object() };
    Json microphone { Json::object() };
    Json audioQuality { Json::object() };
    double clipStartSeconds { 0.0 };
};

struct ReferenceLoadInfo final {
    double originalSampleRate { 0.0 };
    int originalChannels { 0 };
    double originalDurationSeconds { 0.0 };
    bool resampled { false };
};

// --- 32-bit float WAV I/O ----------------------------------------------------

void writeWavFloat(const std::filesystem::path& path, const Clip& clip) {
    std::ofstream out(path, std::ios::binary);
    const auto frames = std::min(clip.left.size(), clip.right.size());
    const auto channels = std::uint16_t { 2 };
    const auto bits = std::uint16_t { 32 };
    const auto rate = static_cast<std::uint32_t>(clip.sampleRate);
    const auto blockAlign = static_cast<std::uint16_t>(channels * bits / 8);
    const auto dataBytes = static_cast<std::uint32_t>(frames * blockAlign);
    const auto put32 = [&](std::uint32_t v) { for (int i = 0; i < 4; ++i) out.put(static_cast<char>((v >> (8 * i)) & 0xffU)); };
    const auto put16 = [&](std::uint16_t v) { for (int i = 0; i < 2; ++i) out.put(static_cast<char>((v >> (8 * i)) & 0xffU)); };
    out.write("RIFF", 4); put32(36U + dataBytes); out.write("WAVE", 4);
    out.write("fmt ", 4); put32(16U); put16(3U /*IEEE float*/); put16(channels);
    put32(rate); put32(rate * blockAlign); put16(blockAlign); put16(bits);
    out.write("data", 4); put32(dataBytes);
    const auto putFloat = [&](float f) { std::uint32_t bitsU; std::memcpy(&bitsU, &f, 4); put32(bitsU); };
    for (std::size_t i = 0; i < frames; ++i) { putFloat(clip.left[i]); putFloat(clip.right[i]); }
}

bool loadReferenceAudio(
    const std::filesystem::path& path,
    double targetSampleRate,
    Clip& out,
    ReferenceLoadInfo& info,
    std::string& error) {
    const juce::File file(path.string());
    if (!file.existsAsFile()) {
        error = "file not found";
        return false;
    }

    juce::AudioFormatManager formats;
    formats.registerBasicFormats();
    auto reader = std::unique_ptr<juce::AudioFormatReader>(formats.createReaderFor(file));
    if (reader == nullptr) {
        error = "unsupported or corrupt audio file";
        return false;
    }
    if (!(reader->sampleRate > 0.0) || reader->numChannels == 0 || reader->lengthInSamples <= 0) {
        error = "invalid audio metadata";
        return false;
    }
    if (reader->lengthInSamples > static_cast<juce::int64>(30 * 60 * reader->sampleRate)) {
        error = "reference longer than 30 minutes";
        return false;
    }
    if (reader->lengthInSamples > static_cast<juce::int64>(std::numeric_limits<int>::max() - 16)) {
        error = "reference too large for offline decoder";
        return false;
    }

    const auto sourceFrames = static_cast<int>(reader->lengthInSamples);
    const auto sourceChannels = static_cast<int>(reader->numChannels);
    juce::AudioBuffer<float> decoded(std::max(1, std::min(2, sourceChannels)), sourceFrames);
    if (!reader->read(&decoded, 0, sourceFrames, 0, true, sourceChannels > 1)) {
        error = "audio decode failed";
        return false;
    }

    info.originalSampleRate = reader->sampleRate;
    info.originalChannels = sourceChannels;
    info.originalDurationSeconds = static_cast<double>(sourceFrames) / reader->sampleRate;
    info.resampled = std::abs(reader->sampleRate - targetSampleRate) > 0.5;
    out.sampleRate = targetSampleRate;

    const auto destinationFrames = static_cast<std::size_t>(std::max(
        1.0, std::round(static_cast<double>(sourceFrames) * targetSampleRate / reader->sampleRate)));
    const auto copyOrResample = [&](int channel, std::vector<float>& destination) {
        const auto sourceChannel = std::min(channel, decoded.getNumChannels() - 1);
        const auto* source = decoded.getReadPointer(sourceChannel);
        destination.resize(destinationFrames);
        if (!info.resampled) {
            std::copy_n(source, std::min<std::size_t>(destinationFrames, static_cast<std::size_t>(sourceFrames)),
                destination.begin());
            return;
        }

        // Lagrange interpolation is deterministic and avoids requiring users to
        // pre-convert 44.1/96 kHz field recordings before a listening run.
        std::vector<float> padded(static_cast<std::size_t>(sourceFrames) + 8U, 0.0F);
        std::copy_n(source, sourceFrames, padded.begin());
        juce::LagrangeInterpolator interpolator;
        const auto speedRatio = reader->sampleRate / targetSampleRate;
        (void) interpolator.process(
            speedRatio, padded.data(), destination.data(), static_cast<int>(destinationFrames));
    };
    copyOrResample(0, out.left);
    copyOrResample(1, out.right);
    return !out.left.empty() && out.left.size() == out.right.size();
}

bool loadReferenceManifest(
    const std::filesystem::path& path,
    std::map<std::string, ReferenceSpec>& references,
    std::string& error) {
    std::ifstream input(path);
    if (!input) {
        error = "cannot open manifest: " + path.string();
        return false;
    }
    try {
        const auto manifest = Json::parse(input);
        if (manifest.value("schema_version", 0) != 2 || !manifest.contains("references")
            || !manifest.at("references").is_array()) {
            error = "unsupported reference manifest schema";
            return false;
        }
        const auto base = std::filesystem::absolute(path).parent_path();
        const auto requiredText = [](const Json& object, const char* field) {
            if (!object.contains(field) || !object.at(field).is_string()) {
                throw std::runtime_error(std::string("missing text field '") + field + "'");
            }
            auto value = object.at(field).get<std::string>();
            if (value.empty()) {
                throw std::runtime_error(std::string("empty text field '") + field + "'");
            }
            return value;
        };
        const auto isOneOf = [](const std::string& value, std::initializer_list<const char*> allowed) {
            return std::any_of(allowed.begin(), allowed.end(),
                [&](const char* candidate) { return value == candidate; });
        };
        const auto validateMetadataObject = [&](const Json& item, const char* field) -> const Json& {
            if (!item.contains(field) || !item.at(field).is_object()) {
                throw std::runtime_error(std::string("missing object field '") + field + "'");
            }
            return item.at(field);
        };
        for (const auto& item : manifest.at("references")) {
            if (!item.is_object()) {
                throw std::runtime_error("reference entry is not an object");
            }
            ReferenceSpec spec;
            spec.key = requiredText(item, "key");
            spec.creator = requiredText(item, "creator");
            spec.license = requiredText(item, "license");
            spec.licenseUrl = requiredText(item, "license_url");
            spec.distribution = requiredText(item, "distribution");
            spec.sourceUrl = requiredText(item, "source_url");
            spec.sourceKind = requiredText(item, "source_kind");
            spec.matchQuality = requiredText(item, "match_quality");
            spec.notes = requiredText(item, "notes");
            spec.previewUrl = item.value("preview_url", std::string {});
            spec.sha256 = item.value("sha256", std::string {});

            if (!isOneOf(spec.distribution, { "redistributable", "evaluation-only" })) {
                throw std::runtime_error("unsupported distribution for '" + spec.key + "'");
            }
            if (!isOneOf(spec.sourceKind,
                    { "field-recording", "dyno-recording", "static-test-recording", "archival-recording" })) {
                throw std::runtime_error("unsupported source_kind for '" + spec.key + "'");
            }
            if (!isOneOf(spec.matchQuality,
                    { "exact-platform", "family-proxy", "platform-proxy", "architecture-proxy" })) {
                throw std::runtime_error("unsupported match_quality for '" + spec.key + "'");
            }
            if (spec.licenseUrl.rfind("https://", 0) != 0 || spec.sourceUrl.rfind("https://", 0) != 0) {
                throw std::runtime_error("license_url and source_url must use HTTPS for '" + spec.key + "'");
            }

            const auto relativeFileText = item.value("file", std::string {});
            if (spec.distribution == "redistributable") {
                if (relativeFileText.empty() || spec.previewUrl.rfind("https://", 0) != 0) {
                    throw std::runtime_error(
                        "redistributable reference needs file and HTTPS preview_url for '" + spec.key + "'");
                }
                const auto validSha = spec.sha256.size() == 64
                    && std::all_of(spec.sha256.begin(), spec.sha256.end(), [](unsigned char character) {
                        return std::isdigit(character) != 0 || (character >= 'a' && character <= 'f');
                    });
                if (!validSha) {
                    throw std::runtime_error(
                        "redistributable reference needs lowercase SHA-256 for '" + spec.key + "'");
                }
            }
            if (!relativeFileText.empty()) {
                const auto relativeFile = std::filesystem::path(relativeFileText).lexically_normal();
                if (relativeFile.is_absolute()
                    || std::any_of(relativeFile.begin(), relativeFile.end(), [](const auto& component) {
                        return component == std::filesystem::path("..");
                    })) {
                    throw std::runtime_error("reference file escapes manifest directory for '" + spec.key + "'");
                }
                spec.file = (base / relativeFile).lexically_normal();
            }

            if (!item.contains("clip_start_seconds") || !item.at("clip_start_seconds").is_number()) {
                throw std::runtime_error("missing numeric clip_start_seconds for '" + spec.key + "'");
            }
            spec.clipStartSeconds = item.at("clip_start_seconds").get<double>();
            if (!std::isfinite(spec.clipStartSeconds) || spec.clipStartSeconds < 0.0) {
                throw std::runtime_error("invalid clip_start_seconds for '" + spec.key + "'");
            }

            spec.sourceEngine = validateMetadataObject(item, "source_engine");
            for (const auto* field : { "identity", "platform", "architecture", "aspiration" }) {
                (void) requiredText(spec.sourceEngine, field);
            }
            if (!spec.sourceEngine.contains("cylinders")
                || (!spec.sourceEngine.at("cylinders").is_null()
                    && (!spec.sourceEngine.at("cylinders").is_number_integer()
                        || spec.sourceEngine.at("cylinders").get<int>() < 1
                        || spec.sourceEngine.at("cylinders").get<int>() > 24))) {
                throw std::runtime_error("invalid source_engine.cylinders for '" + spec.key + "'");
            }

            if (!item.contains("operating_conditions") || !item.at("operating_conditions").is_array()
                || item.at("operating_conditions").empty()) {
                throw std::runtime_error("missing operating_conditions for '" + spec.key + "'");
            }
            spec.operatingConditions = item.at("operating_conditions");
            if (!std::all_of(spec.operatingConditions.begin(), spec.operatingConditions.end(),
                    [](const Json& condition) { return condition.is_string() && !condition.get<std::string>().empty(); })) {
                throw std::runtime_error("invalid operating_conditions for '" + spec.key + "'");
            }

            spec.rpm = validateMetadataObject(item, "rpm");
            const auto rpmStatus = requiredText(spec.rpm, "status");
            if (!isOneOf(rpmStatus, { "documented", "instrumented", "estimated", "unknown" })) {
                throw std::runtime_error("invalid rpm.status for '" + spec.key + "'");
            }
            const auto optionalRpm = [&](const char* field) -> std::optional<double> {
                if (!spec.rpm.contains(field) || spec.rpm.at(field).is_null()) return std::nullopt;
                if (!spec.rpm.at(field).is_number()) {
                    throw std::runtime_error(std::string("invalid rpm.") + field + " for '" + spec.key + "'");
                }
                const auto value = spec.rpm.at(field).get<double>();
                if (!std::isfinite(value) || value < 0.0) {
                    throw std::runtime_error(std::string("invalid rpm.") + field + " for '" + spec.key + "'");
                }
                return value;
            };
            const auto minimumRpm = optionalRpm("minimum");
            const auto maximumRpm = optionalRpm("maximum");
            if (minimumRpm.has_value() && maximumRpm.has_value() && *minimumRpm > *maximumRpm) {
                throw std::runtime_error("rpm minimum exceeds maximum for '" + spec.key + "'");
            }

            spec.microphone = validateMetadataObject(item, "microphone");
            for (const auto* field : { "placement", "model", "calibration" }) {
                (void) requiredText(spec.microphone, field);
            }

            spec.audioQuality = validateMetadataObject(item, "audio_quality");
            for (const auto* field : { "asset", "original_format" }) {
                (void) requiredText(spec.audioQuality, field);
            }
            if (!spec.audioQuality.contains("original_sample_rate_hz")
                || !spec.audioQuality.at("original_sample_rate_hz").is_number()
                || spec.audioQuality.at("original_sample_rate_hz").get<double>() <= 0.0
                || !spec.audioQuality.contains("original_channels")
                || !spec.audioQuality.at("original_channels").is_number_integer()
                || spec.audioQuality.at("original_channels").get<int>() < 1
                || spec.audioQuality.at("original_channels").get<int>() > 8
                || !spec.audioQuality.contains("original_bit_depth")
                || (!spec.audioQuality.at("original_bit_depth").is_null()
                    && (!spec.audioQuality.at("original_bit_depth").is_number_integer()
                        || spec.audioQuality.at("original_bit_depth").get<int>() < 1))) {
                throw std::runtime_error("invalid audio_quality for '" + spec.key + "'");
            }

            if (spec.key.empty() || references.contains(spec.key)) {
                error = "empty or duplicate reference key in manifest";
                return false;
            }
            references.emplace(spec.key, std::move(spec));
        }
    } catch (const std::exception& e) {
        error = std::string("invalid reference manifest: ") + e.what();
        return false;
    }
    return true;
}

bool verifyReferenceSha256(const ReferenceSpec& spec, std::string& error) {
    if (spec.sha256.empty()) return true;
    const juce::File file(spec.file.string());
    if (!file.existsAsFile()) {
        error = "file not found";
        return false;
    }
    const auto actual = juce::SHA256(file).toHexString().toStdString();
    if (actual != spec.sha256) {
        error = "SHA-256 mismatch: expected " + spec.sha256 + ", got " + actual;
        return false;
    }
    return true;
}

// --- ITU-R BS.1770 integrated loudness (48 kHz K-weighting) ------------------

struct Biquad final {
    double b0, b1, b2, a1, a2;
    double x1 { 0 }, x2 { 0 }, y1 { 0 }, y2 { 0 };
    double process(double x) {
        const auto y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
        x2 = x1; x1 = x; y2 = y1; y1 = y;
        return y;
    }
};

// Two-stage K-weighting from BS.1770 (coefficients specified at 48 kHz).
Biquad makeShelf() { return { 1.53512485958697, -2.69169618940638, 1.19839281085285, -1.69065929318241, 0.73248077421585 }; }
Biquad makeHighPass() { return { 1.0, -2.0, 1.0, -1.99004745483398, 0.99007225036621 }; }

// Integrated loudness in LUFS with the absolute (-70) and relative (-10) gates.
double integratedLufs(const Clip& clip) {
    const auto n = std::min(clip.left.size(), clip.right.size());
    if (n == 0) return -70.0;
    std::vector<double> kl(n), kr(n);
    auto sL = makeShelf(); auto hL = makeHighPass();
    auto sR = makeShelf(); auto hR = makeHighPass();
    for (std::size_t i = 0; i < n; ++i) {
        kl[i] = hL.process(sL.process(static_cast<double>(clip.left[i])));
        kr[i] = hR.process(sR.process(static_cast<double>(clip.right[i])));
    }
    const auto rate = clip.sampleRate;
    const auto blockSamples = static_cast<std::size_t>(0.400 * rate);
    const auto stepSamples = static_cast<std::size_t>(0.100 * rate); // 75% overlap
    if (n < blockSamples) return -70.0;
    std::vector<double> blockMeanSquare;
    for (std::size_t start = 0; start + blockSamples <= n; start += stepSamples) {
        double sum = 0.0;
        for (std::size_t i = start; i < start + blockSamples; ++i) sum += kl[i] * kl[i] + kr[i] * kr[i];
        blockMeanSquare.push_back(sum / static_cast<double>(blockSamples));
    }
    const auto loudnessOf = [](double meanSquare) { return -0.691 + 10.0 * std::log10(std::max(meanSquare, 1e-12)); };
    // Absolute gate at -70 LUFS.
    double gatedSum = 0.0; std::size_t gatedCount = 0;
    for (const auto ms : blockMeanSquare)
        if (loudnessOf(ms) >= -70.0) { gatedSum += ms; ++gatedCount; }
    if (gatedCount == 0) return -70.0;
    const auto absoluteGatedLoudness = loudnessOf(gatedSum / static_cast<double>(gatedCount));
    // Relative gate at integrated-over-absolute-gated minus 10 LU.
    const auto relativeThreshold = absoluteGatedLoudness - 10.0;
    double relSum = 0.0; std::size_t relCount = 0;
    for (const auto ms : blockMeanSquare)
        if (loudnessOf(ms) >= -70.0 && loudnessOf(ms) >= relativeThreshold) { relSum += ms; ++relCount; }
    if (relCount == 0) return absoluteGatedLoudness;
    return loudnessOf(relSum / static_cast<double>(relCount));
}

double peakOf(const Clip& c) {
    double p = 0.0;
    for (const auto v : c.left) p = std::max(p, std::abs(static_cast<double>(v)));
    for (const auto v : c.right) p = std::max(p, std::abs(static_cast<double>(v)));
    return p;
}

void applyGain(Clip& c, double gain) {
    for (auto& v : c.left) v = static_cast<float>(v * gain);
    for (auto& v : c.right) v = static_cast<float>(v * gain);
}

void selectReferenceWindow(Clip& clip, double startSeconds, std::size_t maximumFrames) {
    const auto available = std::min(clip.left.size(), clip.right.size());
    const auto requestedStart = static_cast<std::size_t>(
        std::max(0.0, std::round(startSeconds * clip.sampleRate)));
    const auto start = std::min(requestedStart, available);
    const auto frames = std::min(maximumFrames, available - start);
    std::vector<float> left(frames), right(frames);
    std::copy_n(clip.left.begin() + static_cast<std::ptrdiff_t>(start), frames, left.begin());
    std::copy_n(clip.right.begin() + static_cast<std::ptrdiff_t>(start), frames, right.begin());
    clip.left = std::move(left);
    clip.right = std::move(right);
}

void resizeClip(Clip& clip, std::size_t frames) {
    clip.left.resize(std::min(frames, clip.left.size()));
    clip.right.resize(std::min(frames, clip.right.size()));
}

void applySymmetricFade(Clip& clip, double seconds) {
    const auto frames = std::min(clip.left.size(), clip.right.size());
    const auto fadeFrames = std::min(
        frames / 2U, static_cast<std::size_t>(std::max(0.0, std::round(seconds * clip.sampleRate))));
    if (fadeFrames == 0) return;
    for (std::size_t i = 0; i < fadeFrames; ++i) {
        const auto gain = static_cast<float>(static_cast<double>(i) / static_cast<double>(fadeFrames));
        clip.left[i] *= gain;
        clip.right[i] *= gain;
        clip.left[frames - 1U - i] *= gain;
        clip.right[frames - 1U - i] *= gain;
    }
}

// --- render the listening trajectory -----------------------------------------

// idle hold -> rev-up into the limiter -> throttle-off overrun decel.
// The offline path has no self-idle loop, so a brake governor shapes the RPM:
// it holds a low idle target, releases as the target ramps to the limiter under
// full throttle (the ECU rev limiter then bounces at redline), and brakes the
// overrun back down with the throttle shut.
Clip renderTrajectory(const EngineConfig& baseConfig, double sampleRate) {
    auto config = baseConfig;
    normaliseEngineConfig(config);
    SimpleEcuModel ecu; SimplifiedGasolinePhysics physics; FourStrokeEventGenerator events;
    auto exhaust = ExhaustGraph::makeForEngine(config);
    auto simulatorPtr = std::make_unique<EngineSimulator>(config, ecu, physics, events, exhaust);
    auto& simulator = *simulatorPtr;
    simulator.setPressureSamplingEnabled(true);

    auto eventQueuePtr = std::make_unique<FiringEventQueue>();
    auto pressureQueuePtr = std::make_unique<CylinderPressureQueue>();
    auto& eventQueue = *eventQueuePtr;
    auto& pressureQueue = *pressureQueuePtr;
    auto audioConfiguration = std::make_unique<EngineRuntime>(config);
    auto& audioState = audioConfiguration->audioState();  // convolution stays default (0.45)

    constexpr double dt = 1.0 / 240.0;
    const auto samplesPerStep = static_cast<int>(std::lround(sampleRate / 240.0));
    auto rendererPtr = std::make_unique<RealtimeEngineAudio>(
        eventQueue, audioState, &pressureQueue, &audioConfiguration->exhaustGraph(),
        &audioConfiguration->engineConfig());
    auto& renderer = *rendererPtr;
    renderer.prepare(sampleRate, samplesPerStep);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    const auto redline = config.redlineRpm;
    // Crank and idle run UNGOVERNED with a shut throttle: the brake governor
    // below can only add load, never throttle, so it cannot produce an idle --
    // only a lug against a brake. The engine's own ECU idle control does it.
    //
    // This used to hold max(idleRpm * 1.35, redline * 0.22), whose floor won on
    // 12 of 14 catalogue engines and put the "idle" at up to 2.14x the real one,
    // while cranking against a 20% throttle free-revved past 6700 rpm. The first
    // listening pass reported the idle as unrecognisable; it was never an idle.
    // See `makeDefaultOfflineAudioScenario` for the measured settle figures.
    const auto idleRpm = std::max(300.0, config.idleRpm);

    // Phase timing (seconds). The idle hold is 7 s because that is what the
    // after-start flare needs to decay to the catalogue idle.
    constexpr double crank = 1.5, idleHold = 7.0, revUp = 3.2, limiterHold = 1.0, decel = 2.6;
    const auto total = crank + idleHold + revUp + limiterHold + decel;

    Clip clip; clip.sampleRate = sampleRate;
    clip.left.reserve(static_cast<std::size_t>(total * sampleRate));
    clip.right.reserve(static_cast<std::size_t>(total * sampleRate));
    juce::AudioBuffer<float> block(2, samplesPerStep);
    double realtime = 0.0, loadIntegral = 0.0;
    const auto steps = static_cast<std::size_t>(total / dt);
    for (std::size_t step = 0; step < steps; ++step) {
        const auto t = static_cast<double>(step) * dt;
        double throttle = 0.0, target = idleRpm; bool governed = true;
        if (t < crank + idleHold) { throttle = 0.0; governed = false; }
        else if (t < crank + idleHold + revUp) {
            const auto u = (t - (crank + idleHold)) / revUp;   // 0..1
            throttle = 0.98;
            target = idleRpm + (redline * 1.03 - idleRpm) * u; // ramp into the limiter
        } else if (t < crank + idleHold + revUp + limiterHold) {
            throttle = 0.99; target = redline * 1.03;                // sit on the limiter
        } else {
            const auto u = (t - (crank + idleHold + revUp + limiterHold)) / decel;
            throttle = 0.0; target = idleRpm + (redline * 0.9 - idleRpm) * (1.0 - std::min(1.0, u));
        }

        EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = t < crank;
        controls.throttle = throttle;
        controls.load = 0.0;
        if (governed) {
            const auto speedError = (simulator.state().rpm - target) / std::max(1.0, target);
            loadIntegral = std::clamp(loadIntegral + speedError * dt * 1.20, 0.0, 0.95);
            controls.load = std::clamp(loadIntegral + speedError * 0.70, 0.0, 1.0);
        } else {
            loadIntegral = 0.0;
        }

        auto frame = simulator.step(dt, controls);
        const auto simStart = frame.state.simulationTimeSeconds - dt;
        for (std::size_t i = 0; i < frame.firingEventCount; ++i) {
            const auto fraction = std::clamp((frame.firingEvents[i].timeSeconds - simStart) / dt, 0.0, 1.0);
            frame.firingEvents[i].timeSeconds = realtime + fraction * dt;
            (void) eventQueue.tryPush(frame.firingEvents[i]);
        }
        CylinderPressureSample ps;
        while (simulator.tryPopCylinderPressureSample(ps)) {
            const auto fraction = std::clamp((ps.timeSeconds - simStart) / dt, 0.0, 1.0);
            ps.timeSeconds = realtime + fraction * dt;
            (void) pressureQueue.tryPush(ps);
        }
        publishAudioFrame(audioState, frame.state, { false, controls.starterEngaged, controls.load, 1.0 });
        audioState.producerTimeNanoseconds.store(
            static_cast<std::uint64_t>(std::max(0.0, realtime + dt) * 1.0e9), std::memory_order_release);
        block.clear();
        renderer.render(block, 0, samplesPerStep);
        for (int s = 0; s < samplesPerStep; ++s) { clip.left.push_back(block.getSample(0, s)); clip.right.push_back(block.getSample(1, s)); }
        realtime += dt;
    }
    return clip;
}

} // namespace

int main(int argc, char** argv) {
    std::cout << std::unitbuf;
    std::filesystem::path root = ENGINELAB_CATALOG_ROOT;
    std::filesystem::path outRoot = "listening-test";
    std::filesystem::path manifestPath;
    double targetLufs = -20.0;
    unsigned seed = 20260718U;
    bool requireReferences = false;
    bool validateManifestOnly = false;
    std::map<std::string, std::filesystem::path> refs; // engine name substring -> reference audio
    std::vector<std::string> requestedEngines; // name substrings; empty -> default trio
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--output" && i + 1 < argc) outRoot = argv[++i];
        else if (a == "--root" && i + 1 < argc) root = argv[++i];
        else if (a == "--reference-manifest" && i + 1 < argc) manifestPath = argv[++i];
        else if (a == "--require-references") requireReferences = true;
        else if (a == "--validate-manifest") validateManifestOnly = true;
        else if (a == "--target-lufs" && i + 1 < argc) targetLufs = std::stod(argv[++i]);
        else if (a == "--seed" && i + 1 < argc) seed = static_cast<unsigned>(std::stoul(argv[++i]));
        else if (a == "--engines" && i + 1 < argc) {
            // Comma-separated catalogue-name substrings, e.g. "LS3,Merlin,Twin".
            std::string list = argv[++i];
            for (std::size_t pos = 0; pos <= list.size();) {
                const auto comma = list.find(',', pos);
                const auto token = list.substr(pos, comma == std::string::npos ? std::string::npos : comma - pos);
                if (!token.empty()) requestedEngines.push_back(token);
                if (comma == std::string::npos) break;
                pos = comma + 1;
            }
        }
        else if (a == "--ref" && i + 1 < argc) {
            const std::string kv = argv[++i]; const auto eq = kv.find('=');
            if (eq != std::string::npos) refs[kv.substr(0, eq)] = kv.substr(eq + 1);
        }
        else if (a == "--help") {
            std::cout
                << "EngineLabAbClipRenderer options:\n"
                << "  --output <dir>\n"
                << "  --engines <comma-separated catalogue substrings>\n"
                << "  --reference-manifest <manifest.json>\n"
                << "  --require-references\n"
                << "  --validate-manifest\n"
                << "  --ref <engine-substring>=<audio-path>\n"
                << "  --target-lufs <value>\n"
                << "  --seed <integer>\n";
            return 0;
        } else {
            std::cerr << "Unknown or incomplete option: " << a << '\n';
            return 1;
        }
    }
    if (!(targetLufs >= -40.0 && targetLufs <= -10.0)) {
        std::cerr << "Target loudness must be between -40 and -10 LUFS.\n";
        return 1;
    }
    constexpr double sampleRate = 48'000.0;
    constexpr double peakCeiling = 0.98;

    std::map<std::string, ReferenceSpec> manifestReferences;
    if (!manifestPath.empty()) {
        std::string error;
        if (!loadReferenceManifest(manifestPath, manifestReferences, error)) {
            std::cerr << error << '\n';
            return 1;
        }
    }
    if (validateManifestOnly) {
        if (manifestPath.empty()) {
            std::cerr << "--validate-manifest requires --reference-manifest.\n";
            return 1;
        }
        int unavailable = 0;
        std::cout << "Manifest metadata valid: " << manifestReferences.size() << " references\n";
        for (const auto& [key, spec] : manifestReferences) {
            if (spec.file.empty()) {
                std::cout << "  " << key << ": metadata only (" << spec.distribution << ")\n";
                continue;
            }
            std::string error;
            if (!verifyReferenceSha256(spec, error)) {
                std::cerr << "  " << key << ": " << error << '\n';
                ++unavailable;
                continue;
            }
            Clip clip;
            ReferenceLoadInfo info;
            if (!loadReferenceAudio(spec.file, sampleRate, clip, info, error)) {
                std::cerr << "  " << key << ": " << error << '\n';
                ++unavailable;
                continue;
            }
            std::cout << "  " << key << ": SHA-256 and decode OK, "
                      << info.originalSampleRate << " Hz, " << info.originalChannels << " ch, "
                      << info.originalDurationSeconds << " s\n";
        }
        if (unavailable != 0) {
            std::cerr << "Manifest metadata is valid, but " << unavailable
                      << " referenced asset(s) failed integrity or decode checks.\n";
            return 2;
        }
        std::cout << "All referenced assets passed integrity and decode checks.\n";
        return 0;
    }

    const auto listeningDir = outRoot / "clips";

    auto catalog = loadEngineCatalog(root);
    if (catalog.entries.empty()) { std::cerr << "No catalogue engines.\n"; return 1; }

    // Default: the three near-equivalent archetypes present in both catalogues.
    std::vector<std::string> wanted { "2JZ", "LS3", "Hayabusa" };
    if (!requestedEngines.empty()) wanted = requestedEngines;
    struct Selected { std::string label; const EngineConfig* config; };
    std::vector<Selected> selected;
    for (const auto& key : wanted)
        for (const auto& e : catalog.entries)
            if (e.config.name.find(key) != std::string::npos) { selected.push_back({ key, &e.config }); break; }
    if (selected.size() != wanted.size()) { std::cerr << "Could not resolve all requested engines.\n"; return 1; }

    struct PreparedReference final {
        bool present { false };
        ReferenceSpec spec;
        Clip clip;
        ReferenceLoadInfo loadInfo;
        std::string error;
    };
    std::vector<PreparedReference> prepared(selected.size());
    bool missingRequiredReference = false;
    for (std::size_t p = 0; p < selected.size(); ++p) {
        const auto& sel = selected[p];
        auto& destination = prepared[p];
        if (const auto manual = refs.find(sel.label); manual != refs.end()) {
            destination.spec.key = sel.label;
            destination.spec.file = manual->second;
            destination.spec.creator = "MANUAL";
            destination.spec.license = "UNVERIFIED";
            destination.spec.distribution = "evaluation-only";
            destination.spec.sourceKind = "unknown";
            destination.spec.matchQuality = "manual-unverified";
            destination.spec.notes = "Supplied with --ref; verify provenance before publishing results.";
        } else if (const auto manifest = manifestReferences.find(sel.label);
                   manifest != manifestReferences.end()) {
            destination.spec = manifest->second;
        } else {
            destination.error = "no reference configured";
        }

        if (!destination.spec.file.empty()) {
            destination.present = verifyReferenceSha256(destination.spec, destination.error);
            if (destination.present) {
                destination.present = loadReferenceAudio(
                    destination.spec.file, sampleRate, destination.clip, destination.loadInfo, destination.error);
            }
        }
        if (!destination.present) {
            std::cerr << "Reference preflight [" << sel.label << "]: " << destination.error;
            if (!destination.spec.file.empty()) std::cerr << " (" << destination.spec.file.string() << ')';
            std::cerr << '\n';
            missingRequiredReference = true;
        } else {
            std::cout << "Reference preflight [" << sel.label << "]: "
                      << destination.loadInfo.originalSampleRate << " Hz, "
                      << destination.loadInfo.originalChannels << " ch, "
                      << destination.loadInfo.originalDurationSeconds << " s"
                      << (destination.loadInfo.resampled ? " -> resample 48000 Hz" : "") << '\n';
        }
    }
    if (requireReferences && missingRequiredReference) {
        std::cerr << "Required reference corpus is incomplete; no listening clips were rendered.\n";
        return 2;
    }
    if ((std::filesystem::exists(listeningDir) && !std::filesystem::is_empty(listeningDir))
        || std::filesystem::exists(outRoot / "listening-key.json")) {
        std::cerr << "Output already contains listening results; choose a new --output directory.\n";
        return 1;
    }
    std::filesystem::create_directories(listeningDir);

    std::mt19937 rng(seed);
    Json key {
        { "schema_version", 2 },
        { "target_lufs", targetLufs },
        { "peak_ceiling", peakCeiling },
        { "seed", seed },
        { "profile", "idle-hold -> rev-up into limiter -> throttle-off decel" },
        { "sample_rate_hz", sampleRate },
        { "reference_manifest", manifestPath.empty() ? "" : std::filesystem::absolute(manifestPath).string() },
        { "pairs", Json::array() },
    };

    // Pre-draw the A/B side per pair, re-rolling if every EngineLab clip landed on
    // the same side (a valid but poor blind batch: guessing one would reveal all).
    std::vector<bool> elIsASide(selected.size());
    for (int attempt = 0; attempt < 16; ++attempt) {
        for (std::size_t p = 0; p < selected.size(); ++p)
            elIsASide[p] = std::uniform_int_distribution<int>(0, 1)(rng) == 0;
        const bool allSame = std::all_of(elIsASide.begin(), elIsASide.end(), [&](bool b) { return b == elIsASide[0]; });
        if (!allSame || selected.size() < 2) break;
    }

    std::cout << "Target loudness: " << targetLufs << " LUFS  (BS.1770 integrated)\n\n";
    for (std::size_t p = 0; p < selected.size(); ++p) {
        const auto& sel = selected[p];
        std::cout << "=== pair " << (p + 1) << ": " << sel.config->name << " ===\n";

        auto elClip = renderTrajectory(*sel.config, sampleRate);
        auto refClip = prepared[p].clip;
        auto haveRef = prepared[p].present;
        if (haveRef) {
            selectReferenceWindow(refClip, prepared[p].spec.clipStartSeconds, elClip.left.size());
            const auto pairFrames = std::min({
                elClip.left.size(), elClip.right.size(), refClip.left.size(), refClip.right.size() });
            if (pairFrames < static_cast<std::size_t>(2.0 * sampleRate)) {
                std::cerr << "Reference [" << sel.label
                          << "] has less than two usable seconds after its configured start offset.\n";
                if (requireReferences) return 2;
                haveRef = false;
                refClip = {};
            } else {
                resizeClip(elClip, pairFrames);
                resizeClip(refClip, pairFrames);
            }
        }
        applySymmetricFade(elClip, 0.020);
        if (haveRef) applySymmetricFade(refClip, 0.020);

        const auto elLufsBefore = integratedLufs(elClip);
        applyGain(elClip, std::pow(10.0, (targetLufs - elLufsBefore) / 20.0));
        double refLufsBefore = 0.0;
        if (haveRef) {
            refLufsBefore = integratedLufs(refClip);
            applyGain(refClip, std::pow(10.0, (targetLufs - refLufsBefore) / 20.0));
        }

        // Apply the same extra attenuation to both sides if either would exceed
        // the peak ceiling. This preserves equal loudness without clipping.
        auto pairPeak = peakOf(elClip);
        if (haveRef) pairPeak = std::max(pairPeak, peakOf(refClip));
        if (pairPeak > peakCeiling) {
            const auto pairGain = peakCeiling / pairPeak;
            applyGain(elClip, pairGain);
            if (haveRef) applyGain(refClip, pairGain);
        }

        const auto elLufsAfter = integratedLufs(elClip);
        const auto elPeak = peakOf(elClip);
        const auto refLufsAfter = haveRef ? integratedLufs(refClip) : 0.0;
        const auto refPeak = haveRef ? peakOf(refClip) : 0.0;
        std::cout << "  EngineLab: " << elLufsBefore << " -> " << elLufsAfter
                  << " LUFS, peak " << std::fixed << std::setprecision(3) << elPeak << '\n';
        if (haveRef) {
            std::cout << "  Reference: " << refLufsBefore << " -> " << refLufsAfter
                      << " LUFS, peak " << refPeak << "  ("
                      << prepared[p].spec.file.string() << ")\n";
        } else {
            std::cout << "  Reference: none supplied -> slot left as placeholder\n";
        }

        // Randomised side (A/B) for EngineLab, drawn above.
        const bool elIsA = elIsASide[p];
        const auto sideEl = elIsA ? "A" : "B";
        const auto sideRef = elIsA ? "B" : "A";
        const auto nameA = listeningDir / ("pair_" + std::to_string(p + 1) + "_A.wav");
        const auto nameB = listeningDir / ("pair_" + std::to_string(p + 1) + "_B.wav");
        writeWavFloat(elIsA ? nameA : nameB, elClip);
        if (haveRef) writeWavFloat(elIsA ? nameB : nameA, refClip);

        Json pair {
            { "pair", p + 1 },
            { "engine", sel.config->name },
            { "duration_seconds", static_cast<double>(elClip.left.size()) / sampleRate },
            { "enginelab_side", sideEl },
            { "enginelab_lufs_before", elLufsBefore },
            { "enginelab_lufs", elLufsAfter },
            { "enginelab_peak", elPeak },
            { "reference_side", sideRef },
            { "reference_present", haveRef },
        };
        if (haveRef) {
            pair["reference"] = {
                { "file", std::filesystem::absolute(prepared[p].spec.file).string() },
                { "creator", prepared[p].spec.creator },
                { "license", prepared[p].spec.license },
                { "license_url", prepared[p].spec.licenseUrl },
                { "distribution", prepared[p].spec.distribution },
                { "source_url", prepared[p].spec.sourceUrl },
                { "preview_url", prepared[p].spec.previewUrl },
                { "sha256", prepared[p].spec.sha256 },
                { "source_kind", prepared[p].spec.sourceKind },
                { "match_quality", prepared[p].spec.matchQuality },
                { "source_engine", prepared[p].spec.sourceEngine },
                { "operating_conditions", prepared[p].spec.operatingConditions },
                { "rpm", prepared[p].spec.rpm },
                { "microphone", prepared[p].spec.microphone },
                { "audio_quality", prepared[p].spec.audioQuality },
                { "notes", prepared[p].spec.notes },
                { "clip_start_seconds", prepared[p].spec.clipStartSeconds },
                { "original_sample_rate_hz", prepared[p].loadInfo.originalSampleRate },
                { "original_channels", prepared[p].loadInfo.originalChannels },
                { "resampled", prepared[p].loadInfo.resampled },
                { "lufs_before", refLufsBefore },
                { "lufs", refLufsAfter },
                { "peak", refPeak },
            };
        } else {
            pair["reference_error"] = prepared[p].error.empty()
                ? "reference window unusable" : prepared[p].error;
        }
        key["pairs"].push_back(std::move(pair));
    }

    std::ofstream keyFile(outRoot / "listening-key.json"); // deliberately OUTSIDE clips/
    if (!keyFile) {
        std::cerr << "Could not write listening key.\n";
        return 1;
    }
    keyFile << key.dump(2) << '\n';
    std::cout << "\nWrote clips to " << listeningDir.string()
              << " and key to " << (outRoot / "listening-key.json").string() << '\n';
    std::cout << "The key is outside the clips/ folder: keep it away from listeners.\n";
    return 0;
}
