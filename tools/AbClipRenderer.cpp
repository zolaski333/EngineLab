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
// name=path.wav), it is loudness-matched the same way and placed on the opposite
// side of the pair. Without one, the reference slot is left as a documented
// placeholder (see docs/audio-ab-listening.md); es2d could not be built locally
// (empty submodules), so no es2d clip is produced.

#include <enginelab/audio/RealtimeEngineAudio.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <juce_audio_basics/juce_audio_basics.h>
#include <juce_dsp/juce_dsp.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <random>
#include <string>
#include <thread>
#include <vector>

namespace {
using namespace enginelab;

struct Clip final {
    std::vector<float> left, right;
    double sampleRate { 48'000.0 };
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

std::uint32_t readU32(const unsigned char* p) {
    return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8)
        | (static_cast<std::uint32_t>(p[2]) << 16) | (static_cast<std::uint32_t>(p[3]) << 24);
}
std::uint16_t readU16(const unsigned char* p) {
    return static_cast<std::uint16_t>(static_cast<std::uint16_t>(p[0]) | (static_cast<std::uint16_t>(p[1]) << 8));
}

// Reference-recording loader (PCM16 or float WAV, mono or stereo) -> stereo 48k.
bool loadWavAny(const std::filesystem::path& path, Clip& out) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;
    std::vector<unsigned char> b((std::istreambuf_iterator<char>(in)), {});
    if (b.size() < 44 || readU32(b.data()) != 0x46464952U) return false;
    std::size_t off = 12; std::uint16_t ch = 1, bits = 16, fmt = 1; std::uint32_t rate = 48'000;
    while (off + 8 <= b.size()) {
        const auto id = readU32(&b[off]); const auto sz = readU32(&b[off + 4]); const auto body = off + 8;
        if (id == 0x20746d66U && body + 16 <= b.size()) {
            fmt = readU16(&b[body]); ch = std::max<std::uint16_t>(1, readU16(&b[body + 2]));
            rate = readU32(&b[body + 4]); bits = readU16(&b[body + 14]);
        } else if (id == 0x61746164U) {
            const auto bytes = std::min<std::size_t>(sz, b.size() - body);
            const auto bytesPerSample = bits / 8U;
            if (bytesPerSample == 0) return false;
            const auto total = bytes / bytesPerSample;
            for (std::size_t i = 0; i + ch <= total; i += ch) {
                const auto sample = [&](std::size_t k) -> float {
                    const auto p = &b[body + (i + k) * bytesPerSample];
                    if (fmt == 3 && bits == 32) { float f; std::memcpy(&f, p, 4); return f; }
                    if (bits == 16) return static_cast<float>(static_cast<std::int16_t>(readU16(p))) / 32768.0F;
                    return 0.0F;
                };
                const auto l = sample(0);
                const auto r = ch > 1 ? sample(1) : l;
                out.left.push_back(l); out.right.push_back(r);
            }
            out.sampleRate = rate;
            return !out.left.empty();
        }
        off = body + sz + (sz & 1U);
    }
    return false;
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
    auto rendererPtr = std::make_unique<RealtimeEngineAudio>(eventQueue, audioState, &pressureQueue);
    auto& renderer = *rendererPtr;
    renderer.prepare(sampleRate, samplesPerStep);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    const auto idle = config.idleRpm;
    const auto redline = config.redlineRpm;
    const auto idleTarget = std::max(idle * 1.35, redline * 0.22);

    // Phase timing (seconds).
    constexpr double crank = 1.1, idleHold = 2.4, revUp = 3.2, limiterHold = 1.0, decel = 2.6;
    const auto total = crank + idleHold + revUp + limiterHold + decel;

    Clip clip; clip.sampleRate = sampleRate;
    clip.left.reserve(static_cast<std::size_t>(total * sampleRate));
    clip.right.reserve(static_cast<std::size_t>(total * sampleRate));
    juce::AudioBuffer<float> block(2, samplesPerStep);
    double realtime = 0.0, loadIntegral = 0.0;
    const auto steps = static_cast<std::size_t>(total / dt);
    for (std::size_t step = 0; step < steps; ++step) {
        const auto t = static_cast<double>(step) * dt;
        double throttle = 0.2, target = idleTarget; bool governed = true;
        if (t < crank) { throttle = 0.2; governed = false; }
        else if (t < crank + idleHold) { throttle = 0.12; target = idleTarget; }
        else if (t < crank + idleHold + revUp) {
            const auto u = (t - (crank + idleHold)) / revUp;   // 0..1
            throttle = 0.98;
            target = idleTarget + (redline * 1.03 - idleTarget) * u; // ramp into the limiter
        } else if (t < crank + idleHold + revUp + limiterHold) {
            throttle = 0.99; target = redline * 1.03;                // sit on the limiter
        } else {
            const auto u = (t - (crank + idleHold + revUp + limiterHold)) / decel;
            throttle = 0.0; target = idleTarget + (redline * 0.9 - idleTarget) * (1.0 - std::min(1.0, u));
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

std::string jsonEscape(const std::string& s) {
    std::string o; for (const auto c : s) { if (c == '\\' || c == '"') o += '\\'; o += c; } return o;
}
} // namespace

int main(int argc, char** argv) {
    std::cout << std::unitbuf;
    std::filesystem::path root = ENGINELAB_CATALOG_ROOT;
    std::filesystem::path outRoot = "listening-test";
    double targetLufs = -20.0;
    unsigned seed = 20260718U;
    std::map<std::string, std::filesystem::path> refs; // engine name substring -> reference wav
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--output" && i + 1 < argc) outRoot = argv[++i];
        else if (a == "--root" && i + 1 < argc) root = argv[++i];
        else if (a == "--target-lufs" && i + 1 < argc) targetLufs = std::stod(argv[++i]);
        else if (a == "--seed" && i + 1 < argc) seed = static_cast<unsigned>(std::stoul(argv[++i]));
        else if (a == "--ref" && i + 1 < argc) {
            const std::string kv = argv[++i]; const auto eq = kv.find('=');
            if (eq != std::string::npos) refs[kv.substr(0, eq)] = kv.substr(eq + 1);
        }
    }
    const auto listeningDir = outRoot / "clips";
    std::filesystem::create_directories(listeningDir);

    auto catalog = loadEngineCatalog(root);
    if (catalog.entries.empty()) { std::cerr << "No catalogue engines.\n"; return 1; }

    // The three near-equivalent archetypes present in both catalogues.
    const std::array<std::string, 3> wanted { "2JZ", "LS3", "Hayabusa" };
    struct Selected { std::string label; const EngineConfig* config; };
    std::vector<Selected> selected;
    for (const auto& key : wanted)
        for (const auto& e : catalog.entries)
            if (e.config.name.find(key) != std::string::npos) { selected.push_back({ key, &e.config }); break; }
    if (selected.size() != wanted.size()) { std::cerr << "Could not resolve all three archetypes.\n"; return 1; }

    constexpr double sampleRate = 48'000.0;
    std::mt19937 rng(seed);
    std::ofstream key(outRoot / "listening-key.json"); // deliberately OUTSIDE clips/
    key << "{\n  \"target_lufs\": " << targetLufs << ",\n  \"seed\": " << seed
        << ",\n  \"profile\": \"idle-hold -> rev-up into limiter -> throttle-off decel\",\n  \"pairs\": [\n";

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

        // EngineLab side.
        auto elClip = renderTrajectory(*sel.config, sampleRate);
        const auto elLufsBefore = integratedLufs(elClip);
        applyGain(elClip, std::pow(10.0, (targetLufs - elLufsBefore) / 20.0));
        const auto elLufsAfter = integratedLufs(elClip);
        const auto elPeak = peakOf(elClip);
        std::cout << "  EngineLab: " << elLufsBefore << " -> " << elLufsAfter
                  << " LUFS, peak " << std::fixed << std::setprecision(3) << elPeak << '\n';

        // Reference side (optional).
        bool haveRef = false; Clip refClip; double refLufsAfter = 0.0; double refPeak = 0.0;
        std::string refSource = "PLACEHOLDER";
        if (const auto it = refs.find(sel.label); it != refs.end()) {
            if (loadWavAny(it->second, refClip)) {
                haveRef = true; refSource = it->second.string();
                if (std::abs(refClip.sampleRate - sampleRate) > 1.0)
                    std::cerr << "  WARNING: reference is " << refClip.sampleRate
                              << " Hz, not 48000; resample it to 48 kHz first (LUFS K-weighting"
                                 " and playback both assume 48 kHz).\n";
                const auto before = integratedLufs(refClip);
                applyGain(refClip, std::pow(10.0, (targetLufs - before) / 20.0));
                refLufsAfter = integratedLufs(refClip); refPeak = peakOf(refClip);
                std::cout << "  Reference: " << before << " -> " << refLufsAfter
                          << " LUFS, peak " << refPeak << "  (" << refSource << ")\n";
            } else std::cout << "  Reference: FAILED to load " << it->second.string() << '\n';
        }
        if (!haveRef) std::cout << "  Reference: none supplied -> slot left as placeholder\n";

        // Randomised side (A/B) for EngineLab, drawn above.
        const bool elIsA = elIsASide[p];
        const auto sideEl = elIsA ? "A" : "B";
        const auto sideRef = elIsA ? "B" : "A";
        const auto nameA = listeningDir / ("pair_" + std::to_string(p + 1) + "_A.wav");
        const auto nameB = listeningDir / ("pair_" + std::to_string(p + 1) + "_B.wav");
        writeWavFloat(elIsA ? nameA : nameB, elClip);
        if (haveRef) writeWavFloat(elIsA ? nameB : nameA, refClip);

        key << "    {\n      \"pair\": " << (p + 1) << ",\n      \"engine\": \"" << jsonEscape(sel.config->name) << "\",\n"
            << "      \"enginelab_side\": \"" << sideEl << "\",\n"
            << "      \"enginelab_lufs\": " << elLufsAfter << ",\n"
            << "      \"reference_side\": \"" << sideRef << "\",\n"
            << "      \"reference_source\": \"" << jsonEscape(refSource) << "\",\n"
            << "      \"reference_present\": " << (haveRef ? "true" : "false")
            << (haveRef ? (",\n      \"reference_lufs\": " + std::to_string(refLufsAfter)) : "") << "\n    }"
            << (p + 1 < selected.size() ? ",\n" : "\n");
    }
    key << "  ]\n}\n";
    std::cout << "\nWrote clips to " << listeningDir.string()
              << " and key to " << (outRoot / "listening-key.json").string() << '\n';
    std::cout << "The key is outside the clips/ folder: keep it away from listeners.\n";
    return 0;
}
