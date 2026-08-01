#include <enginelab/audio/OfflineAudioExporter.hpp>

#include <enginelab/audio/ImpulseResponseLoader.hpp>
#include <enginelab/audio/RealtimeEngineAudio.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/runtime/EngineRuntime.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <juce_audio_basics/juce_audio_basics.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <numbers>
#include <numeric>
#include <stdexcept>
#include <thread>
#include <utility>

namespace enginelab {
namespace {
using Json = nlohmann::json;

constexpr double simulationRateHz = 240.0;
constexpr double maximumScenarioSeconds = 120.0;
constexpr std::size_t maximumScenarioStages = 128;
constexpr std::array<std::uint32_t, 3> supportedSampleRates {
    48'000U, 96'000U, 192'000U
};
constexpr std::array<const char*, 6> stemNames {
    "combustion", "exhaust_dry", "exhaust_ir",
    "intake", "forced_induction", "mechanical"
};
constexpr const char* premasterName = "premaster.wav";
constexpr const char* processingDeltaName = "master_processing_delta.wav";

class EngineOrderMap final {
public:
    EngineOrderMap(std::uint32_t sampleRate, std::ostream& output)
        : sampleRate_(static_cast<double>(sampleRate)), output_(output),
          windowSize_(sampleRate / 12U), hopSize_(windowSize_ / 2U) {
        mono_.reserve(windowSize_ + hopSize_);
        rpm_.reserve(windowSize_ + hopSize_);
        window_.resize(windowSize_);
        for (std::size_t sample = 0; sample < windowSize_; ++sample)
            window_[sample] = 0.5 - 0.5 * std::cos(
                2.0 * std::numbers::pi * static_cast<double>(sample)
                / static_cast<double>(windowSize_ - 1U));
        windowSum_ = std::accumulate(window_.begin(), window_.end(), 0.0);
        output_ << "time_s,rpm,order,frequency_hz,level_dbfs\n";
    }

    void push(const juce::AudioBuffer<float>& audio, int sampleCount,
              double rpm) {
        const auto* left = audio.getReadPointer(0);
        const auto* right = audio.getReadPointer(1);
        for (int sample = 0; sample < sampleCount; ++sample) {
            mono_.push_back(0.5F * (left[sample] + right[sample]));
            rpm_.push_back(static_cast<float>(rpm));
        }
        while (mono_.size() >= windowSize_) {
            analyse();
            mono_.erase(mono_.begin(), mono_.begin()
                + static_cast<std::ptrdiff_t>(hopSize_));
            rpm_.erase(rpm_.begin(), rpm_.begin()
                + static_cast<std::ptrdiff_t>(hopSize_));
            consumedFrames_ += hopSize_;
        }
    }

private:
    void analyse() {
        const auto rpmSum = std::accumulate(
            rpm_.begin(), rpm_.begin() + static_cast<std::ptrdiff_t>(windowSize_),
            0.0);
        const auto meanRpm = rpmSum / static_cast<double>(windowSize_);
        const auto centreSeconds = (static_cast<double>(consumedFrames_)
            + 0.5 * static_cast<double>(windowSize_)) / sampleRate_;
        for (int halfOrder = 1; halfOrder <= 48; ++halfOrder) {
            const auto order = 0.5 * static_cast<double>(halfOrder);
            const auto frequencyHz = meanRpm / 60.0 * order;
            if (frequencyHz <= 0.0 || frequencyHz >= sampleRate_ * 0.48)
                continue;
            const auto phaseStep = 2.0 * std::numbers::pi
                * frequencyHz / sampleRate_;
            const auto cosine = std::cos(phaseStep);
            const auto sine = std::sin(phaseStep);
            const auto coefficient = 2.0 * cosine;
            double previous = 0.0;
            double beforePrevious = 0.0;
            for (std::size_t sample = 0; sample < windowSize_; ++sample) {
                const auto value = static_cast<double>(mono_[sample])
                    * window_[sample];
                const auto current = value + coefficient * previous
                    - beforePrevious;
                beforePrevious = previous;
                previous = current;
            }
            const auto real = previous - beforePrevious * cosine;
            const auto imaginary = beforePrevious * sine;
            const auto amplitude = 2.0 * std::hypot(real, imaginary)
                / std::max(1.0, windowSum_);
            const auto levelDbfs = 20.0 * std::log10(
                std::max(1.0e-12, amplitude));
            output_ << centreSeconds << ',' << meanRpm << ',' << order
                    << ',' << frequencyHz << ',' << levelDbfs << '\n';
        }
    }

    double sampleRate_;
    std::ostream& output_;
    std::size_t windowSize_;
    std::size_t hopSize_;
    std::uint64_t consumedFrames_ { 0 };
    std::vector<float> mono_;
    std::vector<float> rpm_;
    std::vector<double> window_;
    double windowSum_ { 1.0 };
};

[[nodiscard]] bool finiteInRange(
    double value, double minimum, double maximum) noexcept {
    return std::isfinite(value) && value >= minimum && value <= maximum;
}

[[nodiscard]] bool validateScenario(
    const OfflineAudioScenario& scenario, std::string& error) {
    if (scenario.name.empty()) {
        error = "Scenario name must not be empty.";
        return false;
    }
    if (scenario.stages.empty()) {
        error = "Scenario must contain at least one stage.";
        return false;
    }
    if (scenario.stages.size() > maximumScenarioStages) {
        error = "Scenario contains more than 128 stages.";
        return false;
    }

    double totalSeconds = 0.0;
    for (std::size_t index = 0; index < scenario.stages.size(); ++index) {
        const auto& stage = scenario.stages[index];
        const auto prefix = "Stage " + std::to_string(index + 1) + ": ";
        if (stage.name.empty()) {
            error = prefix + "name must not be empty.";
            return false;
        }
        if (!finiteInRange(stage.durationSeconds, 1.0 / simulationRateHz,
                           maximumScenarioSeconds)) {
            error = prefix + "duration_seconds must be finite and at least 1/240 s.";
            return false;
        }
        if (!finiteInRange(stage.throttleStart, 0.0, 1.0)
            || !finiteInRange(stage.throttleEnd, 0.0, 1.0)
            || !finiteInRange(stage.loadStart, 0.0, 1.0)
            || !finiteInRange(stage.loadEnd, 0.0, 1.0)) {
            error = prefix + "throttle and load values must be in [0, 1].";
            return false;
        }
        if (!finiteInRange(stage.targetRpmStart, 0.0, 30'000.0)
            || !finiteInRange(stage.targetRpmEnd, 0.0, 30'000.0)
            || (stage.governed
                && (stage.targetRpmStart < 100.0 || stage.targetRpmEnd < 100.0))) {
            error = prefix
                + "governed target RPM must be finite and in [100, 30000].";
            return false;
        }
        totalSeconds += stage.durationSeconds;
        if (!std::isfinite(totalSeconds)
            || totalSeconds > maximumScenarioSeconds) {
            error = "Scenario duration exceeds the 120 second RIFF/WAV safety limit.";
            return false;
        }
    }
    return true;
}

[[nodiscard]] Json stageToJson(const OfflineAudioStage& stage) {
    return {
        { "name", stage.name },
        { "duration_seconds", stage.durationSeconds },
        { "ignition", stage.ignitionEnabled },
        { "starter", stage.starterEngaged },
        { "governed", stage.governed },
        { "throttle_start", stage.throttleStart },
        { "throttle_end", stage.throttleEnd },
        { "load_start", stage.loadStart },
        { "load_end", stage.loadEnd },
        { "target_rpm_start", stage.targetRpmStart },
        { "target_rpm_end", stage.targetRpmEnd },
    };
}

[[nodiscard]] Json scenarioToJson(const OfflineAudioScenario& scenario) {
    Json stages = Json::array();
    for (const auto& stage : scenario.stages)
        stages.push_back(stageToJson(stage));
    return {
        { "schema_version", 1 },
        { "name", scenario.name },
        { "stages", std::move(stages) },
    };
}

template <typename Value>
bool readRequired(
    const Json& source, const char* key, Value& destination,
    std::string& error, const std::string& prefix) {
    const auto found = source.find(key);
    if (found == source.end()) {
        error = prefix + "missing `" + key + "`.";
        return false;
    }
    try {
        destination = found->get<Value>();
    } catch (const std::exception&) {
        error = prefix + "`" + key + "` has the wrong type.";
        return false;
    }
    return true;
}

class WaveStreamWriter final {
public:
    WaveStreamWriter(
        std::filesystem::path path, std::uint32_t sampleRate,
        OfflineWaveFormat format)
        : path_(std::move(path)), sampleRate_(sampleRate), format_(format) {
        stream_.open(path_, std::ios::binary | std::ios::trunc);
        if (!stream_)
            throw std::runtime_error(
                "Could not create " + path_.string());
        writeHeader(0);
    }

    WaveStreamWriter(const WaveStreamWriter&) = delete;
    WaveStreamWriter& operator=(const WaveStreamWriter&) = delete;

    ~WaveStreamWriter() {
        if (active_) {
            try {
                close();
            } catch (...) {
            }
        }
    }

    void write(const juce::AudioBuffer<float>& audio, int sampleCount) {
        if (!active_ || sampleCount < 0
            || audio.getNumChannels() < 2
            || sampleCount > audio.getNumSamples()) {
            throw std::runtime_error("Invalid WAV block.");
        }
        const auto bytesPerFrame =
            format_ == OfflineWaveFormat::pcm24 ? 6ULL : 8ULL;
        if ((frames_ + static_cast<std::uint64_t>(sampleCount))
                * bytesPerFrame
            > std::numeric_limits<std::uint32_t>::max()) {
            throw std::runtime_error(
                "Render exceeds the 32-bit RIFF/WAV data limit.");
        }

        const auto* left = audio.getReadPointer(0);
        const auto* right = audio.getReadPointer(1);
        for (int sample = 0; sample < sampleCount; ++sample) {
            if (format_ == OfflineWaveFormat::pcm24) {
                writePcm24(left[sample]);
                writePcm24(right[sample]);
            } else {
                writeFloat32(left[sample]);
                writeFloat32(right[sample]);
            }
        }
        frames_ += static_cast<std::uint64_t>(sampleCount);
        if (!stream_)
            throw std::runtime_error(
                "Failed while writing " + path_.string());
    }

    void close() {
        if (!active_) return;
        const auto bytesPerFrame =
            format_ == OfflineWaveFormat::pcm24 ? 6ULL : 8ULL;
        const auto dataBytes = static_cast<std::uint32_t>(
            frames_ * bytesPerFrame);
        stream_.seekp(0, std::ios::beg);
        writeHeader(dataBytes);
        stream_.flush();
        if (!stream_)
            throw std::runtime_error(
                "Could not finalise " + path_.string());
        stream_.close();
        active_ = false;
    }

    void abort() noexcept {
        if (stream_.is_open()) stream_.close();
        active_ = false;
    }

private:
    void put16(std::uint16_t value) {
        for (int byte = 0; byte < 2; ++byte)
            stream_.put(static_cast<char>(
                (value >> (8 * byte)) & 0xffU));
    }

    void put32(std::uint32_t value) {
        for (int byte = 0; byte < 4; ++byte)
            stream_.put(static_cast<char>(
                (value >> (8 * byte)) & 0xffU));
    }

    void writeHeader(std::uint32_t dataBytes) {
        constexpr std::uint16_t channels = 2;
        const auto bits = static_cast<std::uint16_t>(
            format_ == OfflineWaveFormat::pcm24 ? 24 : 32);
        const auto formatCode = static_cast<std::uint16_t>(
            format_ == OfflineWaveFormat::pcm24 ? 1 : 3);
        const auto blockAlign = static_cast<std::uint16_t>(
            channels * bits / 8);
        stream_.write("RIFF", 4);
        put32(36U + dataBytes);
        stream_.write("WAVE", 4);
        stream_.write("fmt ", 4);
        put32(16U);
        put16(formatCode);
        put16(channels);
        put32(sampleRate_);
        put32(sampleRate_ * blockAlign);
        put16(blockAlign);
        put16(bits);
        stream_.write("data", 4);
        put32(dataBytes);
    }

    void writePcm24(float sample) {
        const auto finite = std::isfinite(sample) ? sample : 0.0F;
        const auto clamped = std::clamp(finite, -1.0F, 1.0F);
        const auto quantised = static_cast<std::int32_t>(
            std::llround(static_cast<double>(clamped) * 8'388'607.0));
        const auto bits = static_cast<std::uint32_t>(quantised);
        stream_.put(static_cast<char>(bits & 0xffU));
        stream_.put(static_cast<char>((bits >> 8U) & 0xffU));
        stream_.put(static_cast<char>((bits >> 16U) & 0xffU));
    }

    void writeFloat32(float sample) {
        const auto finite = std::isfinite(sample) ? sample : 0.0F;
        std::uint32_t bits {};
        static_assert(sizeof(bits) == sizeof(finite));
        std::memcpy(&bits, &finite, sizeof(bits));
        put32(bits);
    }

    std::filesystem::path path_;
    std::ofstream stream_;
    std::uint32_t sampleRate_ {};
    OfflineWaveFormat format_ { OfflineWaveFormat::pcm24 };
    std::uint64_t frames_ {};
    bool active_ { true };
};

[[nodiscard]] std::string impulseResponseErrorName(
    ImpulseResponseLoadError error) {
    switch (error) {
    case ImpulseResponseLoadError::none:
        return "unspecified failure";
    case ImpulseResponseLoadError::missingFile:
        return "file not found";
    case ImpulseResponseLoadError::unsupportedOrCorrupt:
        return "unsupported or corrupt";
    case ImpulseResponseLoadError::invalidMetadata:
        return "invalid metadata";
    case ImpulseResponseLoadError::empty:
        return "empty";
    case ImpulseResponseLoadError::readFailure:
        return "read failure";
    }
    return "unknown failure";
}

void applyFade(
    juce::AudioBuffer<float>& buffer, int sampleCount,
    std::uint64_t firstFrame, std::uint64_t totalFrames,
    std::uint64_t fadeFrames) noexcept {
    if (fadeFrames == 0) return;
    for (int sample = 0; sample < sampleCount; ++sample) {
        const auto frame = firstFrame
            + static_cast<std::uint64_t>(sample);
        const auto fadeIn = std::min(
            1.0, static_cast<double>(frame + 1) /
                     static_cast<double>(fadeFrames));
        const auto remaining = totalFrames - frame;
        const auto fadeOut = std::min(
            1.0, static_cast<double>(remaining) /
                     static_cast<double>(fadeFrames));
        const auto gain = static_cast<float>(
            std::min(fadeIn, fadeOut));
        buffer.setSample(0, sample, buffer.getSample(0, sample) * gain);
        buffer.setSample(1, sample, buffer.getSample(1, sample) * gain);
    }
}

void removePartialFiles(
    const std::vector<std::filesystem::path>& files) noexcept {
    std::error_code ignored;
    for (const auto& file : files)
        std::filesystem::remove(file, ignored);
}

[[nodiscard]] std::filesystem::path partialPathFor(
    const std::filesystem::path& finalPath) {
    auto partial = finalPath;
    partial += ".partial";
    return partial;
}

} // namespace

const char* offlineWaveFormatName(
    OfflineWaveFormat format) noexcept {
    switch (format) {
    case OfflineWaveFormat::pcm24:
        return "pcm24";
    case OfflineWaveFormat::float32:
        return "float32";
    }
    return "unknown";
}

OfflineAudioScenario makeDefaultOfflineAudioScenario(
    const EngineConfig& engine) {
    // Crank and idle are UNGOVERNED with a shut throttle, so the engine's own
    // ECU idle control starts it and holds it.
    //
    // They used to be a braked hold at max(idleRpm * 1.35, redlineRpm * 0.22),
    // and that floor decided the answer on 12 of the 14 catalogue engines: the
    // "idle" stage sat at 1.35-2.14x the real idle (the flat-six held 1672 rpm
    // against a catalogue 780), while cranking against a 20% throttle with no
    // brake free-revved to 6723 rpm on the K20. A listener therefore never
    // heard an idle at all, which is exactly what the first listening pass
    // reported. The governor here can only apply load, never throttle, so it
    // cannot produce an idle -- only a lug against a brake.
    //
    // Measured with `stage_speeds` after the change (settled rpm / catalogue
    // idle): K20 1.04x, LS3 1.00x, Big Twin 1.06x, Merlin 0.98x. The 7 s hold
    // is what the after-start flare needs to decay; at 3.5 s the same engines
    // still read 1.12-1.38x, and the residual is a real post-start flare rather
    // than an error.
    const auto idleRpm = std::max(300.0, engine.idleRpm);
    const auto limiterTarget = engine.redlineRpm * 1.03;
    const auto overrunStart = engine.redlineRpm * 0.90;
    return {
        "showcase",
        {
            { .name = "crank", .durationSeconds = 1.5,
              .ignitionEnabled = true, .starterEngaged = true,
              .governed = false,
              .throttleStart = 0.0, .throttleEnd = 0.0,
              .loadStart = 0.0, .loadEnd = 0.0,
              .targetRpmStart = 0.0, .targetRpmEnd = 0.0 },
            { .name = "idle", .durationSeconds = 7.0,
              .ignitionEnabled = true, .starterEngaged = false,
              .governed = false,
              .throttleStart = 0.0, .throttleEnd = 0.0,
              .loadStart = 0.0, .loadEnd = 0.0,
              .targetRpmStart = 0.0, .targetRpmEnd = 0.0 },
            { .name = "rev_up", .durationSeconds = 3.2,
              .ignitionEnabled = true, .starterEngaged = false,
              .governed = true,
              .throttleStart = 0.98, .throttleEnd = 0.98,
              .loadStart = 0.0, .loadEnd = 0.0,
              .targetRpmStart = idleRpm, .targetRpmEnd = limiterTarget },
            { .name = "limiter", .durationSeconds = 1.0,
              .ignitionEnabled = true, .starterEngaged = false,
              .governed = true,
              .throttleStart = 0.99, .throttleEnd = 0.99,
              .loadStart = 0.0, .loadEnd = 0.0,
              .targetRpmStart = limiterTarget, .targetRpmEnd = limiterTarget },
            { .name = "overrun", .durationSeconds = 2.6,
              .ignitionEnabled = true, .starterEngaged = false,
              .governed = true,
              .throttleStart = 0.0, .throttleEnd = 0.0,
              .loadStart = 0.0, .loadEnd = 0.0,
              .targetRpmStart = overrunStart, .targetRpmEnd = idleRpm },
        }
    };
}

bool loadOfflineAudioScenario(
    const std::filesystem::path& file,
    OfflineAudioScenario& destination,
    std::string& error) {
    try {
        std::ifstream input(file);
        if (!input) {
            error = "Could not open scenario: " + file.string();
            return false;
        }
        const auto root = Json::parse(input);
        if (!root.is_object()) {
            error = "Scenario root must be a JSON object.";
            return false;
        }
        int schemaVersion {};
        OfflineAudioScenario parsed;
        if (!readRequired(
                root, "schema_version", schemaVersion, error, "Scenario: ")
            || schemaVersion != 1) {
            if (error.empty())
                error = "Scenario: schema_version must be 1.";
            return false;
        }
        if (!readRequired(
                root, "name", parsed.name, error, "Scenario: "))
            return false;
        const auto stages = root.find("stages");
        if (stages == root.end() || !stages->is_array()) {
            error = "Scenario: `stages` must be an array.";
            return false;
        }
        for (std::size_t index = 0; index < stages->size(); ++index) {
            const auto& source = (*stages)[index];
            const auto prefix =
                "Stage " + std::to_string(index + 1) + ": ";
            if (!source.is_object()) {
                error = prefix + "stage must be an object.";
                return false;
            }
            OfflineAudioStage stage;
            if (!readRequired(source, "name", stage.name, error, prefix)
                || !readRequired(
                    source, "duration_seconds",
                    stage.durationSeconds, error, prefix)
                || !readRequired(
                    source, "ignition",
                    stage.ignitionEnabled, error, prefix)
                || !readRequired(
                    source, "starter",
                    stage.starterEngaged, error, prefix)
                || !readRequired(
                    source, "governed",
                    stage.governed, error, prefix)
                || !readRequired(
                    source, "throttle_start",
                    stage.throttleStart, error, prefix)
                || !readRequired(
                    source, "throttle_end",
                    stage.throttleEnd, error, prefix)
                || !readRequired(
                    source, "load_start",
                    stage.loadStart, error, prefix)
                || !readRequired(
                    source, "load_end",
                    stage.loadEnd, error, prefix)
                || !readRequired(
                    source, "target_rpm_start",
                    stage.targetRpmStart, error, prefix)
                || !readRequired(
                    source, "target_rpm_end",
                    stage.targetRpmEnd, error, prefix)) {
                return false;
            }
            parsed.stages.push_back(std::move(stage));
        }
        if (!validateScenario(parsed, error)) return false;
        destination = std::move(parsed);
        return true;
    } catch (const std::exception& exception) {
        error = "Could not parse scenario JSON: "
            + std::string(exception.what());
        return false;
    }
}

bool saveOfflineAudioScenario(
    const std::filesystem::path& file,
    const OfflineAudioScenario& scenario,
    std::string& error) {
    if (!validateScenario(scenario, error)) return false;
    try {
        std::ofstream output(file);
        if (!output) {
            error = "Could not create scenario: " + file.string();
            return false;
        }
        output << scenarioToJson(scenario).dump(2) << '\n';
        if (!output) {
            error = "Could not write scenario: " + file.string();
            return false;
        }
        return true;
    } catch (const std::exception& exception) {
        error = "Could not save scenario: "
            + std::string(exception.what());
        return false;
    }
}

OfflineAudioExportResult exportOfflineAudio(
    const OfflineAudioExportRequest& request,
    const OfflineAudioProgress& progress) {
    OfflineAudioExportResult result;
    std::vector<std::filesystem::path> partialFiles;
    std::vector<std::unique_ptr<WaveStreamWriter>> writers;
    std::unique_ptr<std::ofstream> orderMapStream;
    std::unique_ptr<EngineOrderMap> orderMap;

    const auto fail = [&](std::string message) {
        orderMap.reset();
        if (orderMapStream) orderMapStream->close();
        orderMapStream.reset();
        for (auto& writer : writers)
            if (writer) writer->abort();
        removePartialFiles(partialFiles);
        result.error = std::move(message);
        return result;
    };

    try {
        if (request.outputDirectory.empty())
            return fail("Output directory must not be empty.");
        if (std::find(
                supportedSampleRates.begin(), supportedSampleRates.end(),
                request.sampleRateHz)
            == supportedSampleRates.end()) {
            return fail("Sample rate must be 48000, 96000 or 192000 Hz.");
        }
        std::string validationError;
        if (!validateScenario(request.scenario, validationError))
            return fail(validationError);

        auto config = request.engine;
        normaliseEngineConfig(config);
        if (const auto configError = validateEngineConfig(config))
            return fail("Invalid engine configuration: " + *configError);

        if (progress && !progress(0.0, "preflight")) {
            result.cancelled = true;
            return result;
        }

        std::filesystem::create_directories(request.outputDirectory);
        const auto masterPath =
            request.outputDirectory / "master.wav";
        const auto scenarioPath =
            request.outputDirectory / "scenario.json";
        const auto manifestPath =
            request.outputDirectory / "render-manifest.json";
        const auto orderMapPath =
            request.outputDirectory / "engine-order-map.csv";

        std::vector<std::filesystem::path> finalWavePaths { masterPath };
        if (request.writeStems) {
            for (const auto* name : stemNames)
                finalWavePaths.push_back(
                    request.outputDirectory
                    / (std::string("stem_") + name + ".wav"));
            finalWavePaths.push_back(request.outputDirectory / premasterName);
            finalWavePaths.push_back(
                request.outputDirectory / processingDeltaName);
        }
        std::vector<std::filesystem::path> finalFiles = finalWavePaths;
        if (request.writeStems) finalFiles.push_back(orderMapPath);
        finalFiles.push_back(scenarioPath);
        finalFiles.push_back(manifestPath);
        for (const auto& finalFile : finalFiles) {
            const auto partial = partialPathFor(finalFile);
            if (std::filesystem::exists(partial))
                return fail(
                    "Stale partial export exists: " + partial.string());
            if (!request.overwriteExistingFiles
                && std::filesystem::exists(finalFile)) {
                return fail(
                    "Output already exists: " + finalFile.string());
            }
            partialFiles.push_back(partial);
        }

        double totalSeconds = 0.0;
        for (const auto& stage : request.scenario.stages)
            totalSeconds += stage.durationSeconds;
        const auto totalFrames = static_cast<std::uint64_t>(
            std::llround(
                totalSeconds
                * static_cast<double>(request.sampleRateHz)));
        const auto samplesPerStep = static_cast<int>(
            request.sampleRateHz
            / static_cast<std::uint32_t>(simulationRateHz));
        if (totalFrames == 0 || samplesPerStep <= 0)
            return fail("Scenario produces no audio frames.");

        SimpleEcuModel ecu;
        SimplifiedGasolinePhysics physics;
        FourStrokeEventGenerator events;
        auto exhaust = ExhaustGraph::makeForEngine(config);
        auto simulator = std::make_unique<EngineSimulator>(
            config, ecu, physics, events, exhaust);
        simulator->setPressureSamplingEnabled(true);

        auto eventQueue = std::make_unique<FiringEventQueue>();
        auto pressureQueue = std::make_unique<CylinderPressureQueue>();
        auto audioConfiguration =
            std::make_unique<EngineRuntime>(config);
        audioConfiguration->setAudioVolume(request.mix.volume);
        audioConfiguration->setAudioConvolution(
            request.mix.convolution);
        audioConfiguration->setHighFrequencyGain(
            request.mix.highFrequencyGain);
        audioConfiguration->setLowFrequencyGain(
            request.mix.lowFrequencyGain);
        audioConfiguration->setLowFrequencyNoise(
            request.mix.lowFrequencyNoise);
        audioConfiguration->setHighFrequencyNoise(
            request.mix.highFrequencyNoise);
        audioConfiguration->setCombustionGain(
            request.mix.combustionGain);
        audioConfiguration->setExhaustGain(
            request.mix.exhaustGain);
        audioConfiguration->setIntakeGain(
            request.mix.intakeGain);
        audioConfiguration->setMechanicalGain(
            request.mix.mechanicalGain);
        audioConfiguration->setStereoWidth(
            request.mix.stereoWidth);
        audioConfiguration->setOutletJetGain(
            request.mix.outletJetGain);
        audioConfiguration->setSaturationDrive(
            request.mix.saturationDrive);
        audioConfiguration->setSaturationPlacement(
            request.mix.saturationPlacement);

        auto renderer = std::make_unique<RealtimeEngineAudio>(
            *eventQueue, audioConfiguration->audioState(),
            pressureQueue.get(), &audioConfiguration->exhaustGraph(),
            &audioConfiguration->engineConfig());

        if (request.loadAuthoredImpulseResponses) {
            const auto pathCount = config.exhaustPaths.size();
            for (std::size_t pathIndex = 0;
                 pathIndex < pathCount; ++pathIndex) {
                const auto& configured =
                    config.exhaustPaths[pathIndex]
                        .impulseResponsePath;
                if (configured.empty()) continue;
                ++result.authoredImpulseResponses;
                if (pathIndex
                    >= RealtimeConvolutionBank::maximumPaths) {
                    result.warnings.push_back(
                        "Impulse response path "
                        + std::to_string(pathIndex + 1)
                        + " exceeds the eight-path renderer limit.");
                    continue;
                }
                auto fullPath = std::filesystem::path(configured);
                if (fullPath.is_relative())
                    fullPath = request.assetRoot / fullPath;
                auto decoded = loadImpulseResponseFile(
                    juce::File(fullPath.string()), 262'144);
                if (!decoded.ok()) {
                    result.warnings.push_back(
                        "Impulse response not loaded ("
                        + impulseResponseErrorName(decoded.error)
                        + "): " + fullPath.string());
                    continue;
                }
                if (decoded.truncated) {
                    result.warnings.push_back(
                        "Impulse response truncated to 262144 samples: "
                        + fullPath.string());
                }
                renderer->setImpulseResponse(
                    std::move(decoded.samples),
                    decoded.sampleRateHz, pathIndex);
                ++result.loadedImpulseResponses;
            }
        }

        renderer->prepare(
            static_cast<double>(request.sampleRateHz),
            samplesPerStep);
        if (result.loadedImpulseResponses != 0)
            std::this_thread::sleep_for(
                std::chrono::milliseconds(300));

        for (const auto& finalWave : finalWavePaths) {
            const auto partial = partialPathFor(finalWave);
            writers.push_back(std::make_unique<WaveStreamWriter>(
                partial, request.sampleRateHz, request.format));
        }
        if (request.writeStems) {
            orderMapStream = std::make_unique<std::ofstream>(
                partialPathFor(orderMapPath));
            if (!*orderMapStream)
                return fail("Could not create engine-order-map.csv.");
            orderMap = std::make_unique<EngineOrderMap>(
                request.sampleRateHz, *orderMapStream);
        }

        juce::AudioBuffer<float> master(2, samplesPerStep);
        juce::AudioBuffer<float> premaster(2, samplesPerStep);
        juce::AudioBuffer<float> processingDelta(2, samplesPerStep);
        std::array<juce::AudioBuffer<float>, stemNames.size()>
            stemBlocks;
        RealtimeAudioStemBuffers stemViews;
        if (request.writeStems) {
            for (auto& stem : stemBlocks)
                stem.setSize(2, samplesPerStep);
            stemViews = {
                &stemBlocks[0], &stemBlocks[1],
                &stemBlocks[2], &stemBlocks[3],
                &stemBlocks[4], &stemBlocks[5],
            };
        }

        const auto fadeFrames = std::min<std::uint64_t>(
            totalFrames / 2,
            static_cast<std::uint64_t>(
                std::llround(
                    0.020
                    * static_cast<double>(request.sampleRateHz))));
        constexpr double dt = 1.0 / simulationRateHz;
        std::size_t stageIndex = 0;
        // Per-stage speed telemetry. `tail*` accumulate only the last quarter
        // of a stage, which is what the stage settled to rather than what it
        // passed through on the way.
        struct StageSpeedAccumulator final {
            double sum { 0.0 };
            double minimum { std::numeric_limits<double>::infinity() };
            double maximum { -std::numeric_limits<double>::infinity() };
            std::uint64_t count { 0 };
            double tailSum { 0.0 };
            std::uint64_t tailCount { 0 };
        };
        std::vector<StageSpeedAccumulator> stageSpeeds(
            request.scenario.stages.size());
        double stageStartSeconds = 0.0;
        double loadIntegral = 0.0;
        double realtimeSeconds = 0.0;
        long double masterSquareSum = 0.0;
        std::uint64_t masterSampleCount = 0;
        std::uint64_t writtenFrames = 0;
        std::uint64_t step = 0;

        while (writtenFrames < totalFrames) {
            while (stageIndex + 1
                       < request.scenario.stages.size()
                   && realtimeSeconds
                       >= stageStartSeconds
                           + request.scenario
                                 .stages[stageIndex]
                                 .durationSeconds
                           - 1.0e-9) {
                stageStartSeconds +=
                    request.scenario.stages[stageIndex]
                        .durationSeconds;
                ++stageIndex;
            }
            const auto& stage =
                request.scenario.stages[stageIndex];
            const auto stageFraction = std::clamp(
                (realtimeSeconds - stageStartSeconds)
                    / stage.durationSeconds,
                0.0, 1.0);
            const auto interpolate =
                [stageFraction](double start, double end) {
                    return start + (end - start) * stageFraction;
                };

            EngineControls controls;
            controls.ignitionEnabled =
                stage.ignitionEnabled;
            controls.starterEngaged =
                stage.starterEngaged;
            controls.throttle = interpolate(
                stage.throttleStart, stage.throttleEnd);
            if (stage.governed) {
                const auto targetRpm = interpolate(
                    stage.targetRpmStart, stage.targetRpmEnd);
                const auto speedError =
                    (simulator->state().rpm - targetRpm)
                    / std::max(1.0, targetRpm);
                loadIntegral = std::clamp(
                    loadIntegral + speedError * dt * 1.20,
                    0.0, 0.95);
                controls.load = std::clamp(
                    loadIntegral + speedError * 0.70,
                    0.0, 1.0);
            } else {
                loadIntegral = 0.0;
                controls.load = interpolate(
                    stage.loadStart, stage.loadEnd);
            }

            auto frame = simulator->step(dt, controls);
            if (stageIndex < stageSpeeds.size()) {
                auto& speed = stageSpeeds[stageIndex];
                const auto rpm = frame.state.rpm;
                speed.sum += rpm;
                speed.minimum = std::min(speed.minimum, rpm);
                speed.maximum = std::max(speed.maximum, rpm);
                ++speed.count;
                if (stageFraction >= 0.75) {
                    speed.tailSum += rpm;
                    ++speed.tailCount;
                }
            }
            const auto simulationStart =
                frame.state.simulationTimeSeconds - dt;
            for (std::size_t eventIndex = 0;
                 eventIndex < frame.firingEventCount;
                 ++eventIndex) {
                auto event = frame.firingEvents[eventIndex];
                const auto fraction = std::clamp(
                    (event.timeSeconds - simulationStart)
                        / dt,
                    0.0, 1.0);
                event.timeSeconds =
                    realtimeSeconds + fraction * dt;
                if (!eventQueue->tryPush(event))
                    ++result.droppedFiringEvents;
            }
            CylinderPressureSample pressureSample;
            while (simulator->tryPopCylinderPressureSample(
                pressureSample)) {
                const auto fraction = std::clamp(
                    (pressureSample.timeSeconds
                        - simulationStart)
                        / dt,
                    0.0, 1.0);
                pressureSample.timeSeconds =
                    realtimeSeconds + fraction * dt;
                if (!pressureQueue->tryPush(pressureSample))
                    ++result.droppedPressureSamples;
            }

            publishAudioFrame(
                audioConfiguration->audioState(), frame.state,
                { false, controls.starterEngaged,
                  controls.load, 1.0 });
            audioConfiguration->audioState()
                .producerTimeNanoseconds.store(
                    static_cast<std::uint64_t>(
                        std::max(0.0, realtimeSeconds + dt)
                        * 1.0e9),
                    std::memory_order_release);

            master.clear();
            if (request.writeStems) {
                for (auto& stem : stemBlocks)
                    stem.clear();
                renderer->renderWithStems(
                    master, 0, samplesPerStep, stemViews);
            } else {
                renderer->render(
                    master, 0, samplesPerStep);
            }

            const auto remaining =
                totalFrames - writtenFrames;
            const auto framesThisStep = static_cast<int>(
                std::min<std::uint64_t>(
                    remaining,
                    static_cast<std::uint64_t>(
                        samplesPerStep)));
            applyFade(
                master, framesThisStep, writtenFrames,
                totalFrames, fadeFrames);
            if (request.writeStems) {
                for (auto& stem : stemBlocks)
                    applyFade(
                        stem, framesThisStep, writtenFrames,
                        totalFrames, fadeFrames);

                premaster.clear();
                processingDelta.clear();
                for (int channel = 0; channel < 2; ++channel) {
                    for (int sample = 0; sample < framesThisStep; ++sample) {
                        float stemSum = 0.0F;
                        for (const auto& stem : stemBlocks)
                            stemSum += stem.getSample(channel, sample);
                        const auto masterSample = master.getSample(channel, sample);
                        const auto delta = masterSample - stemSum;
                        premaster.setSample(channel, sample, stemSum);
                        processingDelta.setSample(channel, sample, delta);
                        float rebuiltStemSum = 0.0F;
                        for (const auto& stem : stemBlocks)
                            rebuiltStemSum += stem.getSample(channel, sample);
                        result.stemPremasterMaxError = std::max(
                            result.stemPremasterMaxError,
                            std::abs(static_cast<double>(rebuiltStemSum - stemSum)));
                        result.masterReconstructionMaxError = std::max(
                            result.masterReconstructionMaxError,
                            std::abs(static_cast<double>(
                                (stemSum + delta) - masterSample)));
                    }
                }
            }

            const auto* left = master.getReadPointer(0);
            const auto* right = master.getReadPointer(1);
            for (int sample = 0;
                 sample < framesThisStep; ++sample) {
                const auto l = std::isfinite(left[sample])
                    ? static_cast<double>(left[sample]) : 0.0;
                const auto r = std::isfinite(right[sample])
                    ? static_cast<double>(right[sample]) : 0.0;
                result.masterPeak = std::max(
                    result.masterPeak,
                    std::max(std::abs(l), std::abs(r)));
                masterSquareSum += l * l + r * r;
                masterSampleCount += 2;
            }

            writers[0]->write(master, framesThisStep);
            if (request.writeStems) {
                for (std::size_t stemIndex = 0;
                     stemIndex < stemBlocks.size();
                     ++stemIndex) {
                    writers[stemIndex + 1]->write(
                        stemBlocks[stemIndex],
                        framesThisStep);
                }
                writers[stemBlocks.size() + 1]->write(
                    premaster, framesThisStep);
                writers[stemBlocks.size() + 2]->write(
                    processingDelta, framesThisStep);
                orderMap->push(master, framesThisStep, frame.state.rpm);
            }
            writtenFrames +=
                static_cast<std::uint64_t>(framesThisStep);
            realtimeSeconds += dt;
            ++step;

            if (progress && (step % 8U == 0U
                             || writtenFrames == totalFrames)) {
                const auto fraction =
                    static_cast<double>(writtenFrames)
                    / static_cast<double>(totalFrames);
                if (!progress(fraction, stage.name)) {
                    for (auto& writer : writers)
                        writer->abort();
                    removePartialFiles(partialFiles);
                    result.cancelled = true;
                    result.error.clear();
                    return result;
                }
            }
        }

        for (auto& writer : writers)
            writer->close();
        writers.clear();
        orderMap.reset();
        if (orderMapStream) {
            orderMapStream->flush();
            if (!*orderMapStream)
                return fail("Could not write engine-order-map.csv.");
            orderMapStream->close();
            orderMapStream.reset();
        }

        result.renderedFrames = writtenFrames;
        result.durationSeconds =
            static_cast<double>(writtenFrames)
            / static_cast<double>(request.sampleRateHz);
        result.masterRms = masterSampleCount == 0
            ? 0.0
            : std::sqrt(
                static_cast<double>(
                    masterSquareSum
                    / static_cast<long double>(
                        masterSampleCount)));
        result.stageSpeeds.reserve(stageSpeeds.size());
        for (std::size_t index = 0; index < stageSpeeds.size(); ++index) {
            const auto& accumulator = stageSpeeds[index];
            const auto& stage = request.scenario.stages[index];
            OfflineAudioStageSpeed speed;
            speed.name = stage.name;
            speed.governed = stage.governed;
            speed.requestedRpmStart = stage.targetRpmStart;
            speed.requestedRpmEnd = stage.targetRpmEnd;
            if (accumulator.count > 0) {
                speed.meanRpm = accumulator.sum
                    / static_cast<double>(accumulator.count);
                speed.minimumRpm = accumulator.minimum;
                speed.maximumRpm = accumulator.maximum;
            }
            speed.settledRpm = accumulator.tailCount > 0
                ? accumulator.tailSum
                    / static_cast<double>(accumulator.tailCount)
                : speed.meanRpm;
            result.stageSpeeds.push_back(std::move(speed));
        }
        result.physicalExhaustActive =
            renderer->physicalExhaustActive();
        result.compiledExhaustTopologyActive =
            renderer->compiledExhaustTopologyActive();
        result.compiledIntakeTopologyActive =
            renderer->compiledIntakeTopologyActive();
        result.forcedInductionAcousticsActive =
            renderer->forcedInductionAcousticsActive();
        result.delayTruncationCount =
            renderer->delayTruncationCount();
        result.invalidBoundarySampleCount =
            renderer->invalidBoundarySampleCount();
        result.legacyPathSampleCount =
            renderer->legacyPathSampleCount();

        const auto scenarioPartial =
            partialPathFor(scenarioPath);
        if (!saveOfflineAudioScenario(
                scenarioPartial, request.scenario,
                validationError)) {
            return fail(validationError);
        }

        Json fileList = Json::array();
        for (const auto& path : finalWavePaths)
            fileList.push_back(path.filename().string());
        if (request.writeStems)
            fileList.push_back(orderMapPath.filename().string());
        Json warnings = Json::array();
        for (const auto& warning : result.warnings)
            warnings.push_back(warning);
        const Json manifest {
            { "schema_version", 2 },
            { "engine", config.name },
            { "scenario", request.scenario.name },
            { "render_path",
              "EngineSimulator -> publishAudioFrame -> RealtimeEngineAudio" },
            { "sample_rate_hz", request.sampleRateHz },
            { "wave_format",
              offlineWaveFormatName(request.format) },
            { "wave_format_code",
              request.format == OfflineWaveFormat::pcm24
                  ? 1 : 3 },
            { "bits_per_sample",
              request.format == OfflineWaveFormat::pcm24
                  ? 24 : 32 },
            { "channels", 2 },
            { "frames", result.renderedFrames },
            { "duration_seconds", result.durationSeconds },
            { "master_peak", result.masterPeak },
            { "master_rms", result.masterRms },
            { "stems_written", request.writeStems },
            { "stem_reconstruction", {
                { "premaster", premasterName },
                { "master_processing_delta", processingDeltaName },
                { "stem_sum_to_premaster_max_abs_error",
                  result.stemPremasterMaxError },
                { "premaster_plus_delta_to_master_max_abs_error",
                  result.masterReconstructionMaxError },
                { "domain", "float_before_wave_encoding" },
            } },
            { "engine_order_map",
              request.writeStems ? orderMapPath.filename().string() : "" },
            { "stage_speeds", [&result] {
                auto speeds = Json::array();
                for (const auto& speed : result.stageSpeeds) {
                    speeds.push_back({
                        { "name", speed.name },
                        { "governed", speed.governed },
                        { "requested_rpm_start", speed.requestedRpmStart },
                        { "requested_rpm_end", speed.requestedRpmEnd },
                        { "mean_rpm", speed.meanRpm },
                        { "minimum_rpm", speed.minimumRpm },
                        { "maximum_rpm", speed.maximumRpm },
                        { "settled_rpm", speed.settledRpm },
                    });
                }
                return speeds;
            }() },
            { "files", std::move(fileList) },
            { "impulse_responses", {
                { "authored",
                  result.authoredImpulseResponses },
                { "loaded",
                  result.loadedImpulseResponses },
            } },
            { "path_diagnostics", {
                { "physical_exhaust_active",
                  result.physicalExhaustActive },
                { "compiled_exhaust_topology_active",
                  result.compiledExhaustTopologyActive },
                { "compiled_intake_topology_active",
                  result.compiledIntakeTopologyActive },
                { "forced_induction_acoustics_active",
                  result.forcedInductionAcousticsActive },
                { "delay_truncations",
                  result.delayTruncationCount },
                { "invalid_boundary_samples",
                  result.invalidBoundarySampleCount },
                { "legacy_path_samples",
                  result.legacyPathSampleCount },
                { "dropped_firing_events",
                  result.droppedFiringEvents },
                { "dropped_pressure_samples",
                  result.droppedPressureSamples },
            } },
            { "mix", {
                { "volume", request.mix.volume },
                { "convolution", request.mix.convolution },
                { "high_frequency_gain",
                  request.mix.highFrequencyGain },
                { "low_frequency_gain",
                  request.mix.lowFrequencyGain },
                { "low_frequency_noise",
                  request.mix.lowFrequencyNoise },
                { "high_frequency_noise",
                  request.mix.highFrequencyNoise },
                { "combustion_gain",
                  request.mix.combustionGain },
                { "exhaust_gain",
                  request.mix.exhaustGain },
                { "intake_gain",
                  request.mix.intakeGain },
                { "mechanical_gain",
                  request.mix.mechanicalGain },
                { "stereo_width", request.mix.stereoWidth },
                { "outlet_jet_gain", request.mix.outletJetGain },
                { "saturation_drive", request.mix.saturationDrive },
                { "saturation_placement",
                  request.mix.saturationPlacement
                        == AudioSaturationPlacement::preShelf
                    ? "pre_shelf" : "post_shelf" },
            } },
            { "warnings", std::move(warnings) },
        };
        const auto manifestPartial =
            partialPathFor(manifestPath);
        {
            std::ofstream output(manifestPartial);
            if (!output)
                return fail(
                    "Could not create render manifest.");
            output << manifest.dump(2) << '\n';
            if (!output)
                return fail(
                    "Could not write render manifest.");
        }

        for (const auto& finalFile : finalFiles) {
            const auto partial = partialPathFor(finalFile);
            if (request.overwriteExistingFiles) {
                std::error_code removeError;
                std::filesystem::remove(
                    finalFile, removeError);
                if (removeError)
                    return fail(
                        "Could not replace "
                        + finalFile.string() + ": "
                        + removeError.message());
            }
            std::filesystem::rename(partial, finalFile);
        }
        result.files = std::move(finalFiles);
        result.success = true;
        return result;
    } catch (const std::exception& exception) {
        return fail(exception.what());
    }
}

} // namespace enginelab
