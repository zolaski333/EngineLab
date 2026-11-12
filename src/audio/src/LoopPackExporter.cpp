#include <enginelab/audio/LoopPackExporter.hpp>

#include "OfflineRenderChain.hpp"

#include <enginelab/simulation/DynoAbsorberController.hpp>

#include <juce_audio_formats/juce_audio_formats.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <utility>

namespace enginelab {
namespace {

using Json = nlohmann::json;
constexpr double simulationRateHz = offline::simulationRateHz;
constexpr double frameSeconds = 1.0 / simulationRateHz;

/** Off-load hold: a motor that drives the shaft at the requested speed with
 *  the throttle shut, as the wheels drive an engine on the overrun.
 *
 * Feed-forward is the engine's own cycle-averaged drag (its brake torque is
 * negative while it is motored); a PI loop trims the rest. The proportional
 * gain is the shaft's inertia times 6 rad/s, well inside the half-inertia-a-
 * frame stiffness a frame-delayed actuator tolerates, and slow beside the
 * 8 rad/s speed filter it reads through. */
class MotoringController final {
public:
    explicit MotoringController(const EngineConfig& config) noexcept {
        const auto inertia = std::max(
            1.0e-4, effectiveRotatingInertiaKgM2(config));
        proportionalGain_ = 6.0 * inertia;
        integralGain_ = 1.5 * proportionalGain_;
        maximumTorqueNm_ = std::max(
            50.0, 10'000'000.0 * breathingDisplacementLitres(config) * 0.001
                / (4.0 * std::numbers::pi));
    }

    void reset(double rpm) noexcept {
        filteredRpm_ = std::isfinite(rpm) ? std::max(0.0, rpm) : 0.0;
        feedForwardNm_ = 0.0;
        integralNm_ = 0.0;
    }

    [[nodiscard]] double advance(
        double dt, double targetRpm, const EngineState& state) noexcept {
        const auto observed = state.frameAveragedRpm > 0.0
            ? state.frameAveragedRpm : state.rpm;
        if (std::isfinite(observed))
            filteredRpm_ += (std::max(0.0, observed) - filteredRpm_)
                * (1.0 - std::exp(-dt * 8.0));
        const auto drag = std::isfinite(state.cycleAveragedTorqueNm)
            ? std::max(0.0, -state.cycleAveragedTorqueNm) : 0.0;
        feedForwardNm_ += (drag - feedForwardNm_)
            * (1.0 - std::exp(-dt * 8.0));
        const auto errorRadPerSecond = (targetRpm - filteredRpm_)
            * std::numbers::pi / 30.0;
        const auto candidate = std::clamp(
            integralNm_ + errorRadPerSecond * integralGain_ * dt,
            -maximumTorqueNm_, maximumTorqueNm_);
        const auto unclamped = feedForwardNm_ + candidate
            + errorRadPerSecond * proportionalGain_;
        const auto torque = std::clamp(unclamped, 0.0, maximumTorqueNm_);
        if (torque == unclamped
            || (unclamped < 0.0 && errorRadPerSecond > 0.0)
            || (unclamped > maximumTorqueNm_ && errorRadPerSecond < 0.0))
            integralNm_ = candidate;
        return std::isfinite(torque) ? torque : 0.0;
    }

private:
    double proportionalGain_ { 1.0 };
    double integralGain_ { 1.0 };
    double maximumTorqueNm_ { 50.0 };
    double filteredRpm_ { 0.0 };
    double feedForwardNm_ { 0.0 };
    double integralNm_ { 0.0 };
};

/** Mean of `values` over [first, last). */
[[nodiscard]] double meanOf(
    const std::vector<double>& values, std::size_t first,
    std::size_t last) noexcept {
    if (last <= first) return 0.0;
    double sum = 0.0;
    for (auto index = first; index < last; ++index) sum += values[index];
    return sum / static_cast<double>(last - first);
}

struct HoldTask final {
    double targetRpm { 0.0 };
    bool onLoad { true };
    /** Off load at the idle itself: the engine idles on its own, the motor
     *  only a floor under it (a two-stroke's idle can wander off and stall). */
    bool freeIdle { false };
};

struct RecordedLoop final {
    LoopPackLoop summary;
    std::vector<float> engine;
    std::vector<float> exhaust;
};

[[nodiscard]] std::string rpmLabel(double rpm) {
    std::array<char, 16> buffer {};
    std::snprintf(buffer.data(), buffer.size(), "%05d",
        static_cast<int>(std::lround(std::max(0.0, rpm))));
    return buffer.data();
}

[[nodiscard]] std::filesystem::path partialPathFor(
    const std::filesystem::path& finalPath) {
    auto partial = finalPath;
    partial += ".partial";
    return partial;
}

void removeFiles(const std::vector<std::filesystem::path>& files) noexcept {
    std::error_code ignored;
    for (const auto& file : files) std::filesystem::remove(file, ignored);
}

/** Write one mono loop: a WAV whose `smpl` chunk marks the whole file as one
 *  forward loop, or an Ogg Vorbis file. */
[[nodiscard]] bool writeLoopFile(
    const std::filesystem::path& file, const std::vector<float>& samples,
    std::uint32_t sampleRateHz, bool ogg, std::string& error) {
    std::error_code ignored;
    std::filesystem::remove(file, ignored);
    const auto encoded = file.u8string();
    auto stream = std::make_unique<juce::FileOutputStream>(
        juce::File(juce::String::fromUTF8(
            reinterpret_cast<const char*>(encoded.data()),
            static_cast<int>(encoded.size()))));
    if (!stream->openedOk()) {
        error = "Could not create " + file.string();
        return false;
    }
    std::unique_ptr<juce::OutputStream> output = std::move(stream);
    auto options = juce::AudioFormatWriterOptions {}
        .withSampleRate(static_cast<double>(sampleRateHz))
        .withNumChannels(1);
    std::unique_ptr<juce::AudioFormatWriter> writer;
    if (ogg) {
        juce::OggVorbisAudioFormat format;
        writer = format.createWriterFor(output,
            options.withBitsPerSample(16).withQualityOptionIndex(8));
    } else {
        const auto lastFrame = samples.empty() ? juce::uint64 { 0 }
            : static_cast<juce::uint64>(samples.size() - 1U);
        const std::unordered_map<juce::String, juce::String> metadata {
            { "NumSampleLoops", "1" },
            { "Loop0Identifier", "0" },
            { "Loop0Type", "0" },
            { "Loop0Start", "0" },
            { "Loop0End", juce::String(lastFrame) },
            { "Loop0Fraction", "0" },
            { "Loop0PlayCount", "0" },
            { "MidiUnityNote", "60" },
            { "SamplePeriod", juce::String(static_cast<juce::int64>(
                std::llround(1.0e9 / static_cast<double>(sampleRateHz)))) },
        };
        juce::WavAudioFormat format;
        writer = format.createWriterFor(output,
            options.withBitsPerSample(24).withMetadataValues(metadata));
    }
    if (writer == nullptr) {
        error = "Could not encode " + file.string();
        return false;
    }
    const float* channels[] { samples.data() };
    const auto written = writer->writeFromFloatArrays(
        channels, 1, static_cast<int>(samples.size()));
    writer.reset();
    if (!written) {
        error = "Could not write " + file.string();
        return false;
    }
    return true;
}

[[nodiscard]] double decibels(double gain) noexcept {
    return 20.0 * std::log10(std::max(1.0e-12, gain));
}

} // namespace

std::uint32_t equivalentFourStrokeCylinders(
    const EngineConfig& engine) noexcept {
    switch (engine.cycle) {
    case EngineCycle::twoStroke:
        return static_cast<std::uint32_t>(2U * engine.cylinders.size());
    case EngineCycle::rotary:
        return 2U * std::max<std::uint32_t>(1U, engine.rotary.rotorCount);
    case EngineCycle::fourStroke:
        break;
    }
    return static_cast<std::uint32_t>(engine.cylinders.size());
}

std::vector<double> loopPackPoints(
    const EngineConfig& engine, double stepRpm, double lowestRpm,
    double highestRpm) {
    std::vector<double> points;
    if (!(stepRpm > 0.0) || !std::isfinite(stepRpm)) return points;
    const auto idle = std::max(300.0, engine.idleRpm);
    const auto ceiling = 0.95 * std::min(
        engine.redlineRpm > 0.0 ? engine.redlineRpm : engine.ignition.revLimitRpm,
        engine.ignition.revLimitRpm > 0.0 ? engine.ignition.revLimitRpm : engine.redlineRpm);
    const auto first = lowestRpm > 0.0 && std::isfinite(lowestRpm)
        ? std::max(100.0, lowestRpm) : idle;
    const auto last = highestRpm > 0.0 && std::isfinite(highestRpm)
        ? highestRpm : ceiling;
    points.push_back(first);
    if (!(last > first)) return points;
    for (auto rpm = std::ceil((first + 0.5 * stepRpm) / stepRpm) * stepRpm;
         rpm < last - 0.25 * stepRpm; rpm += stepRpm)
        points.push_back(rpm);
    if (last > first + 0.25 * stepRpm) points.push_back(last);
    return points;
}

LoopPackResult exportLoopPack(
    const LoopPackRequest& request, const OfflineAudioProgress& progress) {
    LoopPackResult result;
    std::vector<std::filesystem::path> createdFiles;
    const auto fail = [&](std::string message) {
        removeFiles(createdFiles);
        result.error = std::move(message);
        result.success = false;
        return result;
    };

    try {
        if (request.outputDirectory.empty())
            return fail("Output directory must not be empty.");
        if (request.sampleRateHz != 48'000U && request.sampleRateHz != 96'000U)
            return fail("A loop pack is rendered at 48000 or 96000 Hz.");
        if (!(request.stepRpm >= 50.0) || !std::isfinite(request.stepRpm))
            return fail("The rpm step must be at least 50 rpm.");
        if (!(request.loopSeconds >= 0.25) || !std::isfinite(request.loopSeconds))
            return fail("A loop must last at least 0.25 s.");
        if (!request.onLoad && !request.offLoad)
            return fail("Choose on load, off load or both.");
        if (!std::isfinite(request.packPeakDbfs) || request.packPeakDbfs > 0.0)
            return fail("The pack peak must be at most 0 dBFS.");

        auto config = request.engine;
        normaliseEngineConfig(config);
        if (const auto configError = validateEngineConfig(config))
            return fail("Invalid engine configuration: " + *configError);

        const auto points = loopPackPoints(
            config, request.stepRpm, request.lowestRpm, request.highestRpm);
        if (points.empty()) return fail("No rpm point to hold.");
        const auto idle = std::max(300.0, config.idleRpm);
        std::vector<HoldTask> tasks;
        for (const auto rpm : points) {
            if (request.offLoad)
                tasks.push_back({ rpm, false, rpm <= idle * 1.02 });
            if (request.onLoad) tasks.push_back({ rpm, true, false });
        }

        result.cycleDegrees = cycleDegreesOf(config.cycle);
        result.equivalentFourStrokeCylinders =
            equivalentFourStrokeCylinders(config);

        // Every file the pack will hold, checked before any work is done.
        std::filesystem::create_directories(request.outputDirectory);
        const auto summaryPath = request.outputDirectory / "loop-pack.json";
        std::vector<std::filesystem::path> finalFiles;
        const auto fileFor = [&request](const HoldTask& task,
                                        const char* layer, bool ogg) {
            return request.outputDirectory
                / (std::string(task.onLoad ? "onload_" : "offload_")
                   + rpmLabel(task.targetRpm) + "rpm_" + layer
                   + (ogg ? ".ogg" : ".wav"));
        };
        for (const auto& task : tasks) {
            for (const auto* layer : { "engine", "exhaust" }) {
                finalFiles.push_back(fileFor(task, layer, false));
                if (request.writeOgg)
                    finalFiles.push_back(fileFor(task, layer, true));
            }
        }
        finalFiles.push_back(summaryPath);
        for (const auto& file : finalFiles) {
            if (std::filesystem::exists(partialPathFor(file)))
                return fail("Stale partial export exists: "
                    + partialPathFor(file).string());
            if (!request.overwriteExistingFiles && std::filesystem::exists(file))
                return fail("Output already exists: " + file.string());
        }

        if (progress && !progress(0.0, "preflight")) {
            result.cancelled = true;
            return result;
        }

        auto chain = std::make_unique<offline::RenderChain>(config, request.mix);
        auto& simulator = chain->simulator();
        auto& renderer = chain->renderer();
        std::size_t authoredResponses = 0;
        std::size_t loadedResponses = 0;
        if (request.loadAuthoredImpulseResponses)
            chain->loadAuthoredImpulseResponses(
                config, request.assetRoot, result.warnings,
                authoredResponses, loadedResponses);
        const auto samplesPerStep = static_cast<int>(
            request.sampleRateHz / static_cast<std::uint32_t>(simulationRateHz));
        renderer.prepare(static_cast<double>(request.sampleRateHz), samplesPerStep);
        if (loadedResponses != 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(300));

        juce::AudioBuffer<float> master(2, samplesPerStep);
        std::array<juce::AudioBuffer<float>, 8> stems;
        for (auto& stem : stems) stem.setSize(2, samplesPerStep);
        const RealtimeAudioStemBuffers stemViews {
            &stems[0], &stems[1], &stems[2], &stems[3],
            &stems[4], &stems[5], &stems[6], &stems[7],
        };
        // The engine layer is what radiates from the engine itself, the
        // exhaust layer what leaves the tailpipe; together they are the
        // premaster (docs/audio-diagnostics.md).
        constexpr std::array<std::size_t, 4> engineStems { 0, 3, 4, 5 };
        constexpr std::array<std::size_t, 2> exhaustStems { 1, 2 };
        std::vector<float> engineBlock(static_cast<std::size_t>(samplesPerStep));
        std::vector<float> exhaustBlock(static_cast<std::size_t>(samplesPerStep));

        offline::DroppedTelemetry dropped;
        double realtimeSeconds = 0.0;
        std::uint64_t step = 0;
        const auto clutchScale = std::max(20.0, config.transmission.maxClutchTorqueNm);
        const auto advanceFrame = [&](const EngineControls& controls,
                                      double appliedTorqueNm) {
            const auto frame = simulator.step(frameSeconds, controls);
            chain->forward(frame, controls.starterEngaged,
                std::abs(appliedTorqueNm) / clutchScale,
                realtimeSeconds, frameSeconds, dropped);
            master.clear();
            for (auto& stem : stems) stem.clear();
            renderer.renderWithStems(master, 0, samplesPerStep, stemViews);
            for (int sample = 0; sample < samplesPerStep; ++sample) {
                float engine = 0.0F;
                for (const auto stem : engineStems)
                    engine += stems[stem].getSample(0, sample)
                        + stems[stem].getSample(1, sample);
                float exhaust = 0.0F;
                for (const auto stem : exhaustStems)
                    exhaust += stems[stem].getSample(0, sample)
                        + stems[stem].getSample(1, sample);
                engineBlock[static_cast<std::size_t>(sample)] =
                    std::isfinite(engine) ? 0.5F * engine : 0.0F;
                exhaustBlock[static_cast<std::size_t>(sample)] =
                    std::isfinite(exhaust) ? 0.5F * exhaust : 0.0F;
            }
            realtimeSeconds += frameSeconds;
            ++step;
            return frame;
        };
        bool cancelled = false;
        const auto report = [&](double fraction, const std::string& stage) {
            if (!progress || cancelled) return;
            if (step % 8U != 0U) return;
            if (!progress(std::clamp(fraction, 0.0, 1.0), stage))
                cancelled = true;
        };

        // Start as the audio harness starts every catalogue engine: 1.1 s
        // of starter at a fifth of throttle, then shut. A rotary does not
        // catch on a shut plate before the starter's cut-out speed, and a
        // starter released at that speed left it coasting down. Running
        // means above half the idle for the last half second of 3 s.
        {
            constexpr double starterSeconds = 1.1;
            constexpr double startSeconds = 3.0;
            double elapsed = 0.0;
            double runningSeconds = 0.0;
            while (elapsed < startSeconds) {
                EngineControls controls;
                controls.ignitionEnabled = true;
                controls.starterEngaged = elapsed < starterSeconds;
                controls.throttle = elapsed < starterSeconds - 0.2 ? 0.2 : 0.0;
                const auto frame = advanceFrame(controls, 0.0);
                elapsed += frameSeconds;
                runningSeconds = frame.state.rpm > 0.5 * config.idleRpm
                    ? runningSeconds + frameSeconds : 0.0;
                report(0.0, "start");
                if (cancelled) {
                    result.cancelled = true;
                    return result;
                }
            }
            if (runningSeconds < 0.5)
                return fail("The engine does not start, so it has no loop to give.");
        }

        std::vector<RecordedLoop> recorded;
        recorded.reserve(tasks.size());
        const auto settleWindow = static_cast<std::size_t>(simulationRateHz / 2.0);
        for (std::size_t taskIndex = 0; taskIndex < tasks.size(); ++taskIndex) {
            const auto& task = tasks[taskIndex];
            const auto stage = std::to_string(static_cast<int>(
                    std::lround(task.targetRpm)))
                + (task.onLoad ? " rpm on load" : " rpm off load");
            DynoAbsorberController absorber(config);
            absorber.reset(simulator.state().rpm, simulator.state().torqueNm);
            MotoringController motor(config);
            motor.reset(simulator.state().rpm);
            const auto controlsFor = [&](const EngineState& state,
                                         double& appliedTorqueNm) {
                EngineControls controls;
                controls.ignitionEnabled = true;
                if (task.onLoad) {
                    // As the dyno does: the starter catches a stalled shaft.
                    controls.starterEngaged =
                        state.rpm < dynoStarterCutoutRpm(config);
                    controls.throttle = 1.0;
                    controls.dynamometerTorqueNm = absorber.advance(
                        frameSeconds, task.targetRpm, state).brakeTorqueNm;
                    appliedTorqueNm = controls.dynamometerTorqueNm;
                } else {
                    controls.throttle = 0.0;
                    controls.externalTorqueNm = motor.advance(frameSeconds,
                        task.freeIdle ? 0.9 * task.targetRpm : task.targetRpm,
                        state);
                    appliedTorqueNm = controls.externalTorqueNm;
                }
                return controls;
            };

            // Hold until the speed and the manifold have settled.
            std::vector<double> rpmHistory;
            std::vector<double> mapHistory;
            const auto minimumHold = task.onLoad ? 1.5 : 1.0;
            constexpr double maximumHold = 12.0;
            double held = 0.0;
            bool settled = false;
            while (held < maximumHold) {
                double applied = 0.0;
                const auto controls = controlsFor(simulator.state(), applied);
                const auto frame = advanceFrame(controls, applied);
                held += frameSeconds;
                rpmHistory.push_back(frame.state.frameAveragedRpm > 0.0
                    ? frame.state.frameAveragedRpm : frame.state.rpm);
                mapHistory.push_back(frame.state.manifoldPressureKpa);
                report((static_cast<double>(taskIndex)
                        + 0.5 * held / maximumHold)
                        / static_cast<double>(tasks.size()), stage);
                if (cancelled) {
                    result.cancelled = true;
                    return result;
                }
                if (held < minimumHold || rpmHistory.size() < 2 * settleWindow)
                    continue;
                const auto count = rpmHistory.size();
                const auto lastRpm = meanOf(rpmHistory, count - settleWindow, count);
                const auto previousRpm = meanOf(
                    rpmHistory, count - 2 * settleWindow, count - settleWindow);
                const auto lastMap = meanOf(mapHistory, count - settleWindow, count);
                const auto previousMap = meanOf(
                    mapHistory, count - 2 * settleWindow, count - settleWindow);
                const auto onTarget = task.freeIdle
                    || std::abs(lastRpm - task.targetRpm) <= 0.015 * task.targetRpm;
                const auto steady = std::abs(lastRpm - previousRpm)
                        <= 0.004 * std::max(task.targetRpm, 500.0)
                    && std::abs(lastMap - previousMap) <= 1.0;
                if (onTarget && steady) {
                    settled = true;
                    break;
                }
            }

            // Record whole cycles, plus the cycles the seam fades over.
            const auto startRpm = std::max(1.0, rpmHistory.empty()
                ? simulator.state().rpm
                : meanOf(rpmHistory,
                      rpmHistory.size() - std::min(rpmHistory.size(), settleWindow),
                      rpmHistory.size()));
            const auto cycleSeconds = 60.0 / startRpm * result.cycleDegrees / 360.0;
            const auto loopCycles = static_cast<std::uint32_t>(std::max(
                4.0, std::round(request.loopSeconds / cycleSeconds)));
            const auto fadeCycles = static_cast<std::uint32_t>(std::clamp(
                std::ceil(0.050 / cycleSeconds), 1.0,
                std::max(1.0, std::floor(0.5 * loopCycles))));
            const auto loopDegrees = static_cast<double>(loopCycles) * result.cycleDegrees;
            const auto fadeDegrees = static_cast<double>(fadeCycles) * result.cycleDegrees;

            RecordedLoop loop;
            std::vector<float> engineRaw;
            std::vector<float> exhaustRaw;
            std::vector<double> frameStartDegrees;
            std::vector<double> frameDegrees;
            double travelled = 0.0;
            double rpmSum = 0.0;
            double mapSum = 0.0;
            std::size_t loopFrames = 0;
            loop.summary.minimumRpm = std::numeric_limits<double>::infinity();
            loop.summary.maximumRpm = 0.0;
            const auto spf = static_cast<std::size_t>(samplesPerStep);
            const auto maximumRecordSeconds = 4.0 * (request.loopSeconds + 1.0)
                + 2.0 * cycleSeconds * static_cast<double>(loopCycles + fadeCycles);
            double recordedSeconds = 0.0;
            // Half a fade more than needed: the seam reads that far past the
            // loop's end, and the shaft's speed is not exactly constant.
            while (travelled < loopDegrees + 1.5 * fadeDegrees) {
                double applied = 0.0;
                const auto controls = controlsFor(simulator.state(), applied);
                const auto frame = advanceFrame(controls, applied);
                const auto rpm = frame.state.frameAveragedRpm > 0.0
                    ? frame.state.frameAveragedRpm : frame.state.rpm;
                const auto degrees = std::max(0.0, rpm) * 6.0 * frameSeconds;
                frameStartDegrees.push_back(travelled);
                frameDegrees.push_back(degrees);
                if (travelled < loopDegrees) {
                    rpmSum += rpm;
                    mapSum += frame.state.manifoldPressureKpa;
                    ++loopFrames;
                    loop.summary.minimumRpm = std::min(loop.summary.minimumRpm, rpm);
                    loop.summary.maximumRpm = std::max(loop.summary.maximumRpm, rpm);
                }
                travelled += degrees;
                engineRaw.insert(engineRaw.end(), engineBlock.begin(), engineBlock.end());
                exhaustRaw.insert(exhaustRaw.end(), exhaustBlock.begin(), exhaustBlock.end());
                recordedSeconds += frameSeconds;
                report((static_cast<double>(taskIndex) + 0.5
                        + 0.5 * std::min(1.0, travelled / (loopDegrees + fadeDegrees)))
                        / static_cast<double>(tasks.size()), stage);
                if (cancelled) {
                    result.cancelled = true;
                    return result;
                }
                if (recordedSeconds > maximumRecordSeconds)
                    return fail("The shaft barely turned at " + stage
                        + ": no whole cycle to loop.");
            }
            // The sample where the crank has travelled `target` degrees.
            const auto sampleAt = [&](double target) {
                for (std::size_t frame = 0; frame < frameDegrees.size(); ++frame) {
                    const auto end = frameStartDegrees[frame] + frameDegrees[frame];
                    if (end + 1.0e-12 < target || frameDegrees[frame] <= 0.0) continue;
                    const auto within = std::clamp(
                        (target - frameStartDegrees[frame]) / frameDegrees[frame],
                        0.0, 1.0);
                    return frame * spf + static_cast<std::size_t>(
                        std::llround(within * static_cast<double>(spf)));
                }
                return frameDegrees.size() * spf;
            };
            const auto loopLength = std::min(sampleAt(loopDegrees), engineRaw.size());
            const auto fadeLength = std::min(
                sampleAt(fadeDegrees), engineRaw.size() - loopLength);
            if (loopLength < 16 || fadeLength == 0)
                return fail("The loop at " + stage + " is too short to cut.");

            // The head fades in under the tail that follows the loop's end:
            // the last sample runs straight into the first, which is the
            // sample that came after it.
            const auto seam = [&](const std::vector<float>& raw) {
                std::vector<float> out(raw.begin(),
                    raw.begin() + static_cast<std::ptrdiff_t>(loopLength));
                for (std::size_t index = 0; index < fadeLength; ++index) {
                    const auto head = 0.5 - 0.5 * std::cos(std::numbers::pi
                        * static_cast<double>(index)
                        / static_cast<double>(fadeLength));
                    out[index] = static_cast<float>(
                        static_cast<double>(raw[index]) * head
                        + static_cast<double>(raw[loopLength + index]) * (1.0 - head));
                }
                // Over whole cycles the mean is the loop's DC: remove it.
                double sum = 0.0;
                for (const auto value : out) sum += value;
                const auto mean = static_cast<float>(sum / static_cast<double>(out.size()));
                for (auto& value : out) value -= mean;
                return out;
            };
            loop.engine = seam(engineRaw);
            loop.exhaust = seam(exhaustRaw);
            loop.summary.onLoad = task.onLoad;
            loop.summary.requestedRpm = task.targetRpm;
            loop.summary.meanRpm = loopFrames > 0
                ? rpmSum / static_cast<double>(loopFrames) : 0.0;
            loop.summary.meanManifoldPressureKpa = loopFrames > 0
                ? mapSum / static_cast<double>(loopFrames) : 0.0;
            loop.summary.settled = settled;
            loop.summary.settleSeconds = held;
            loop.summary.cycles = loopCycles;
            loop.summary.frames = loopLength;
            {
                // Degrees at the cut sample, read back from the crank record.
                const auto frame = std::min(loopLength / spf, frameDegrees.size() - 1);
                const auto within = static_cast<double>(loopLength - frame * spf)
                    / static_cast<double>(spf);
                loop.summary.crankDegrees = frameStartDegrees[frame]
                    + within * frameDegrees[frame];
            }
            if (!settled)
                result.warnings.push_back(stage + " had not settled after "
                    + std::to_string(static_cast<int>(maximumHold))
                    + " s; recorded anyway.");
            recorded.push_back(std::move(loop));
        }

        // One gain for the whole pack: its loudest file peaks at the target.
        double peak = 0.0;
        for (const auto& loop : recorded) {
            for (const auto* layer : { &loop.engine, &loop.exhaust })
                for (const auto value : *layer)
                    peak = std::max(peak, static_cast<double>(std::abs(value)));
        }
        if (!(peak > 0.0) || !std::isfinite(peak))
            return fail("The pack is silent: nothing to normalise.");
        const auto gain = std::pow(10.0, request.packPeakDbfs / 20.0) / peak;
        result.packGainDb = decibels(gain);
        if (progress && !progress(1.0, "writing")) {
            result.cancelled = true;
            return result;
        }

        Json loops = Json::array();
        for (std::size_t index = 0; index < recorded.size(); ++index) {
            auto& loop = recorded[index];
            const auto& task = tasks[index];
            double enginePeak = 0.0, exhaustPeak = 0.0;
            double engineSquares = 0.0, exhaustSquares = 0.0;
            for (auto& value : loop.engine) {
                value = static_cast<float>(static_cast<double>(value) * gain);
                enginePeak = std::max(enginePeak, static_cast<double>(std::abs(value)));
                engineSquares += static_cast<double>(value) * value;
            }
            for (auto& value : loop.exhaust) {
                value = static_cast<float>(static_cast<double>(value) * gain);
                exhaustPeak = std::max(exhaustPeak, static_cast<double>(std::abs(value)));
                exhaustSquares += static_cast<double>(value) * value;
            }
            result.packPeak = std::max({ result.packPeak, enginePeak, exhaustPeak });
            loop.summary.enginePeak = enginePeak;
            loop.summary.exhaustPeak = exhaustPeak;
            loop.summary.engineRms = std::sqrt(
                engineSquares / static_cast<double>(loop.engine.size()));
            loop.summary.exhaustRms = std::sqrt(
                exhaustSquares / static_cast<double>(loop.exhaust.size()));
            loop.summary.engineFile = fileFor(task, "engine", false);
            loop.summary.exhaustFile = fileFor(task, "exhaust", false);

            for (const auto ogg : { false, true }) {
                if (ogg && !request.writeOgg) continue;
                for (const auto* layer : { "engine", "exhaust" }) {
                    const auto file = fileFor(task, layer, ogg);
                    const auto partial = partialPathFor(file);
                    createdFiles.push_back(partial);
                    std::string error;
                    const auto& samples = std::string_view(layer) == "engine"
                        ? loop.engine : loop.exhaust;
                    if (!writeLoopFile(partial, samples, request.sampleRateHz, ogg, error))
                        return fail(error);
                }
            }
            const auto& summary = loop.summary;
            loops.push_back({
                { "load", summary.onLoad ? "on" : "off" },
                { "requested_rpm", summary.requestedRpm },
                { "mean_rpm", summary.meanRpm },
                { "minimum_rpm", summary.minimumRpm },
                { "maximum_rpm", summary.maximumRpm },
                { "mean_manifold_pressure_kpa", summary.meanManifoldPressureKpa },
                { "settled", summary.settled },
                { "settle_seconds", summary.settleSeconds },
                { "cycles", summary.cycles },
                { "frames", summary.frames },
                { "duration_seconds", static_cast<double>(summary.frames)
                    / static_cast<double>(request.sampleRateHz) },
                { "crank_degrees", summary.crankDegrees },
                { "firing_frequency_hz", summary.meanRpm / 120.0
                    * static_cast<double>(result.equivalentFourStrokeCylinders) },
                { "engine", {
                    { "file", summary.engineFile.filename().string() },
                    { "peak_dbfs", decibels(summary.enginePeak) },
                    { "rms_dbfs", decibels(summary.engineRms) },
                } },
                { "exhaust", {
                    { "file", summary.exhaustFile.filename().string() },
                    { "peak_dbfs", decibels(summary.exhaustPeak) },
                    { "rms_dbfs", decibels(summary.exhaustRms) },
                } },
            });
            result.loops.push_back(summary);
        }
        result.oggWritten = request.writeOgg;
        result.packGainDb = decibels(gain);

        Json warnings = Json::array();
        for (const auto& warning : result.warnings) warnings.push_back(warning);
        const Json summary {
            { "schema_version", 1 },
            { "engine", config.name },
            { "kind", "loop_pack" },
            { "sample_rate_hz", request.sampleRateHz },
            { "channels", 1 },
            { "bits_per_sample", 24 },
            { "ogg_vorbis", request.writeOgg },
            { "cycle_degrees", result.cycleDegrees },
            { "equivalent_four_stroke_cylinders", result.equivalentFourStrokeCylinders },
            { "firing_frequency_rule",
              "firing_frequency_hz = rpm / 120 * equivalent_four_stroke_cylinders" },
            { "idle_rpm", config.idleRpm },
            { "redline_rpm", config.redlineRpm },
            { "rev_limit_rpm", config.ignition.revLimitRpm },
            { "step_rpm", request.stepRpm },
            { "pack_gain_db", result.packGainDb },
            { "pack_peak_dbfs", decibels(result.packPeak) },
            { "layers", {
                { "engine", "combustion + intake + forced induction + mechanical: "
                            "what radiates from the engine (BeamNG soundConfig)" },
                { "exhaust", "exhaust dry + exhaust IR return: what leaves the "
                             "tailpipe (BeamNG soundConfigExhaust)" },
            } },
            { "listening_chain",
              "sources before the tone shelf, saturation, AGC and limiter; one "
              "gain for every file, the mean of each loop removed" },
            { "loop", "whole engine cycles; the end cross-fades into the start "
                      "over whole cycles; WAV smpl chunk marks the whole file" },
            { "impulse_responses", {
                { "authored", authoredResponses }, { "loaded", loadedResponses } } },
            { "dropped_telemetry", {
                { "firing_events", dropped.firingEvents },
                { "pressure_samples", dropped.pressureSamples },
                { "exhaust_acoustic_samples", dropped.exhaustAcousticSamples },
            } },
            { "loops", std::move(loops) },
            { "warnings", std::move(warnings) },
        };
        {
            const auto partial = partialPathFor(summaryPath);
            createdFiles.push_back(partial);
            std::ofstream output(partial);
            if (!output) return fail("Could not create loop-pack.json.");
            output << summary.dump(2) << '\n';
            if (!output) return fail("Could not write loop-pack.json.");
        }

        for (const auto& file : finalFiles) {
            if (request.overwriteExistingFiles) {
                std::error_code removeError;
                std::filesystem::remove(file, removeError);
                if (removeError)
                    return fail("Could not replace " + file.string() + ": "
                        + removeError.message());
            }
            std::filesystem::rename(partialPathFor(file), file);
        }
        createdFiles.clear();
        result.files = std::move(finalFiles);
        result.success = true;
        return result;
    } catch (const std::exception& exception) {
        return fail(exception.what());
    }
}

} // namespace enginelab
