// Afterfire instrument for the "retour de flamme non fonctionnel" complaint.
//
// The report is that the afterfire does nothing, and that what is there is a
// poor implementation. Both halves need separating before anything is changed,
// because they have different causes and only one of them is a model defect.
//
//  * Most catalogue engines intentionally use clean DFCO. The Audio Physics Lab
//    engine authors a discrete strategy, so its exact authored calibration must
//    remain observable. Hidden harness defaults used to replace that calibration
//    on every run; the default path now preserves it and --force-demo is the
//    explicit legacy laboratory override.
//
//  * Whether what is there is a POOR model is a question about the shape of the
//    heat release in time, and that is what this harness measures. A real
//    afterfire is a bang: an inventory accumulates and then goes off. The
//    implementation is an exponential relaxation with a fixed time constant
//    applied to every hot cell at once, which is the shape of a smear. The two
//    are trivially distinguishable if you actually plot the heat release, and
//    indistinguishable if you argue about the code.
//
// So this drives a real engine through the transient that is supposed to
// produce pops -- accelerate under load, lift off to closed throttle, hold the
// overrun, then tip back in -- and reports the heat release as a TIME SERIES
// plus the statistics that separate a bang from a smear:
//
//   events        discrete excursions above a running baseline
//   crest         peak / mean heat release over the overrun window
//   rise          10-90 % rise time of the largest excursion
//   duty          fraction of the overrun window with any reaction at all
//
// A smear has crest near 1, no events, and duty near 1. A bang has a high
// crest, a countable number of events, a short rise, and a low duty.
//
// With --audio it additionally drives the production RealtimeEngineAudio path
// through the real event/pressure/geometry contracts and reports whether the
// heat release reaches the observer at all -- which is a separate question,
// because the exhaust acoustic chain is excited by the cylinder-pressure
// telemetry and the afterfire releases its heat into the 1-D exhaust network.
// The connection between the two is the port state carried in
// CylinderPressureSample, so the answer is neither obviously yes nor obviously
// no, and it must be measured rather than reasoned about.
//
// Deterministic: no threads, no wall clock in the physics, the driveline
// coupling replicated in the same order EngineRuntime::run uses.
//
//   EngineLabAfterfireHarness [--engines NAME] [--trace] [--audio DIR]
//                             [--force-demo] [--fuel-fraction F]
//                             [--ignition-k K]
//                             [--reaction-ms MS] [--overrun-seconds S]
//                             [--pulse-timing-variation FRACTION]
//                             [--no-reaction-acoustics]
//                             [--list]

#include <enginelab/audio/ImpulseResponseLoader.hpp>
#include <enginelab/audio/RealtimeEngineAudio.hpp>
#include <enginelab/audio/ThermoacousticHeatReleaseSource.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/runtime/DrivelineModel.hpp>
#include <enginelab/runtime/EngineRuntime.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <juce_audio_basics/juce_audio_basics.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {
constexpr double stepSeconds = 1.0 / 240.0;
constexpr double audioSampleRate = 48'000.0;
constexpr int audioSamplesPerStep = 200;

struct Sample final {
    double t { 0.0 };
    double rpm { 0.0 };
    double throttle { 0.0 };
    double heatKw { 0.0 };
    double fuelBurnMgPerSecond { 0.0 };
    double meteredFuelGrams { 0.0 };
    double deliveredFuelMgPerSecond { 0.0 };
    double exhaustPortPeakKpa { 0.0 };
    double exhaustGasC { 0.0 };
    double exhaustWallC { 0.0 };
    double wallIgnitedFraction { 0.0 };
    double inductionProgress { 0.0 };
    double minimumInductionDelayMs { 0.0 };
    double maximumInductionDelayMs { 0.0 };
    bool overrunActive { false };
};

struct AudioContractCounters final {
    std::uint64_t droppedFiringEvents {};
    std::uint64_t droppedCylinderPressureSamples {};
    std::uint64_t droppedExhaustAcousticSamples {};
    std::uint64_t droppedReactionEvents {};
    std::uint64_t reactionPressureLimitedSamples {};
    std::uint64_t lateEvents {};
    std::uint64_t droppedPendingEvents {};
    std::uint64_t stolenVoices {};
    std::uint64_t delayTruncations {};
    std::uint64_t legacyPathSamples {};
    std::uint64_t invalidBoundarySamples {};
    std::uint64_t levelLimitedSamples {};
};

struct ReactionAcousticDiagnostics final {
    std::uint64_t eventCount {};
    double releasedEnergyJoules {};
    double maximumPowerW {};
    double maximumCompactPressureJumpPa {};
    double energyWeightedAxial {};
};

struct RealtimeTiming final {
    std::uint64_t blockCount {};
    std::uint64_t overBudgetBlockCount {};
    double meanBudgetFraction {};
    double p99BudgetFraction {};
    double maximumBudgetFraction {};
};

[[nodiscard]] RealtimeTiming summariseRealtimeTiming(
    std::vector<double> durationsSeconds, double budgetSeconds) {
    RealtimeTiming result;
    result.blockCount = durationsSeconds.size();
    if (durationsSeconds.empty() || !(budgetSeconds > 0.0)) return result;
    std::sort(durationsSeconds.begin(), durationsSeconds.end());
    const auto sumSeconds = std::accumulate(
        durationsSeconds.begin(), durationsSeconds.end(), 0.0);
    result.meanBudgetFraction = sumSeconds
        / (budgetSeconds * static_cast<double>(durationsSeconds.size()));
    const auto p99Index = std::min(
        durationsSeconds.size() - 1,
        static_cast<std::size_t>(std::floor(
            0.99 * static_cast<double>(durationsSeconds.size() - 1))));
    result.p99BudgetFraction = durationsSeconds[p99Index] / budgetSeconds;
    result.maximumBudgetFraction = durationsSeconds.back() / budgetSeconds;
    result.overBudgetBlockCount = static_cast<std::uint64_t>(
        std::count_if(durationsSeconds.begin(), durationsSeconds.end(),
            [budgetSeconds](double seconds) {
                return seconds > budgetSeconds;
            }));
    return result;
}

struct Metrics final {
    std::string name;
    bool ran { false };
    double liftOffTime { 0.0 };
    double tipInTime { 0.0 };
    double rpmAtLiftOff { 0.0 };
    double wallTemperatureAtLiftOffC { 0.0 };
    double wallTemperatureMinDuringOverrunC { 0.0 };
    double peakWallIgnitedFraction { 0.0 };
    double meanWallIgnitedFraction { 0.0 };
    double peakInductionProgress { 0.0 };
    double minimumInductionDelayMs { 0.0 };
    double maximumInductionDelayMs { 0.0 };
    // Heat release over the overrun window.
    double peakHeatKw { 0.0 };
    double troughHeatKw { 0.0 };
    double modulationDepth { 0.0 };
    double meanHeatKw { 0.0 };
    double crest { 0.0 };
    double dutyCycle { 0.0 };
    int eventCount { 0 };
    double largestEventRiseMs { 0.0 };
    double largestEventPeakKw { 0.0 };
    double minimumEventIntervalMs { 0.0 };
    double maximumEventIntervalMs { 0.0 };
    double eventIntervalStdDevMs { 0.0 };
    double burnedFuelMg { 0.0 };
    double meteredFuelMg { 0.0 };
    double deliveredFuelMg { 0.0 };
    int overrunActiveSteps { 0 };
    // Port pressure, the only quantity that can carry the bang to the audio.
    double portPeakDuringOverrunKpa { 0.0 };
    double portBaselineKpa { 0.0 };
    // Audio, only when requested.
    bool audioMeasured { false };
    bool audioFinite { true };
    double audioPeak { 0.0 };
    double audioOverrunPeak { 0.0 };
    double audioBaselineP999 { 0.0 };
    double audioOverrunP999 { 0.0 };
    // Peak over RMS across the overrun. A level metric cannot tell a pop from a
    // slightly louder steady note; a crest factor is exactly "does something
    // stick out of the background", which is what a pop IS.
    double audioOverrunCrest { 0.0 };
    double audioBaselineCrest { 0.0 };
    bool physicalExhaustActive { false };
    bool compiledExhaustTopologyActive { false };
    AudioContractCounters audioContract {};
    ReactionAcousticDiagnostics preLiftReactionAcoustics {};
    bool preLiftReactionWindowMeasured { false };
    ReactionAcousticDiagnostics reactionAcoustics {};
    bool reactionWindowCaptured { false };
    RealtimeTiming reactionWindowSimulationTiming {};
    RealtimeTiming reactionWindowRenderTiming {};
    enginelab::ExhaustAfterfireConfig effectiveCalibration {};
    bool calibrationOverridden { false };
    bool reactionAcousticsEnabled { true };
};

std::string safeFileStem(std::string name) {
    for (auto& character : name) {
        const auto byte = static_cast<unsigned char>(character);
        if (!std::isalnum(byte) && character != '-' && character != '_')
            character = '_';
    }
    return name;
}

// 16-bit PCM by hand rather than through juce_audio_formats: this target only
// links juce_audio_basics, and the proof WAV is for listening, not analysis.
void writeWav(const std::filesystem::path& path,
              const std::vector<float>& left,
              const std::vector<float>& right) {
    if (!path.parent_path().empty())
        std::filesystem::create_directories(path.parent_path());
    std::ofstream out(path, std::ios::binary);
    if (!out)
        throw std::runtime_error("could not create " + path.string());
    const auto frames = std::min(left.size(), right.size());
    const auto dataBytes = static_cast<std::uint32_t>(
        frames * 2 * sizeof(std::int16_t));
    const auto put32 = [&out](std::uint32_t value) {
        for (int index = 0; index < 4; ++index)
            out.put(static_cast<char>((value >> (8 * index)) & 0xffU));
    };
    const auto put16 = [&out](std::uint16_t value) {
        for (int index = 0; index < 2; ++index)
            out.put(static_cast<char>((value >> (8 * index)) & 0xffU));
    };
    out.write("RIFF", 4);
    put32(36U + dataBytes);
    out.write("WAVEfmt ", 8);
    put32(16U);
    put16(1U);
    put16(2U);
    put32(static_cast<std::uint32_t>(audioSampleRate));
    put32(static_cast<std::uint32_t>(audioSampleRate * 4.0));
    put16(4U);
    put16(16U);
    out.write("data", 4);
    put32(dataBytes);
    const auto encode = [&put16](float sample) {
        put16(static_cast<std::uint16_t>(static_cast<std::int16_t>(
            std::lrint(std::clamp(sample, -1.0F, 1.0F) * 32767.0F))));
    };
    for (std::size_t index = 0; index < frames; ++index) {
        encode(left[index]);
        encode(right[index]);
    }
    out.flush();
    if (!out)
        throw std::runtime_error("could not finish " + path.string());
}

class AudioCapture final {
public:
    AudioCapture(const enginelab::EngineConfig& config,
                 const std::filesystem::path& impulseResponsePath,
                 bool reactionAcousticsEnabled)
        : runtime_(std::make_unique<enginelab::EngineRuntime>(config)),
          reactionAcousticsEnabled_(reactionAcousticsEnabled) {
        renderer_ = std::make_unique<enginelab::RealtimeEngineAudio>(
            runtime_->audioEvents(), runtime_->audioState(),
            &runtime_->cylinderPressureSamples(), &runtime_->exhaustGraph(),
            &runtime_->engineConfig(), &runtime_->exhaustAcousticSamples());
        if (!impulseResponsePath.empty()) {
            auto impulse = enginelab::loadImpulseResponseFile(
                juce::File(impulseResponsePath.string()));
            if (!impulse.ok())
                throw std::runtime_error("cannot decode IR: "
                    + impulseResponsePath.string());
            renderer_->setImpulseResponse(
                std::move(impulse.samples), impulse.sampleRateHz, 0);
        }
        renderer_->prepare(audioSampleRate, audioSamplesPerStep);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        left_.reserve(static_cast<std::size_t>(30.0 * audioSampleRate));
        right_.reserve(left_.capacity());
    }

    void renderFrame(enginelab::SimulationFrame& frame,
                     enginelab::EngineSimulator& simulator,
                     bool starterEngaged, double drivelineLoad) {
        counters_.droppedFiringEvents += frame.droppedFiringEventCount;
        counters_.droppedCylinderPressureSamples +=
            frame.droppedCylinderPressureSampleCount;
        counters_.droppedExhaustAcousticSamples +=
            frame.droppedExhaustAcousticSampleCount;
        const auto simulationStart =
            frame.state.simulationTimeSeconds - stepSeconds;
        for (std::size_t index = 0; index < frame.firingEventCount; ++index) {
            auto event = frame.firingEvents[index];
            const auto fraction = std::clamp(
                (event.timeSeconds - simulationStart) / stepSeconds, 0.0, 1.0);
            event.timeSeconds = realtimeSeconds_ + fraction * stepSeconds;
            if (!runtime_->audioEvents().tryPush(event))
                ++counters_.droppedFiringEvents;
        }
        enginelab::CylinderPressureSample pressureSample;
        while (simulator.tryPopCylinderPressureSample(pressureSample)) {
            const auto fraction = std::clamp(
                (pressureSample.timeSeconds - simulationStart) / stepSeconds,
                0.0, 1.0);
            pressureSample.timeSeconds =
                realtimeSeconds_ + fraction * stepSeconds;
            if (!runtime_->cylinderPressureSamples().tryPush(pressureSample))
                ++counters_.droppedCylinderPressureSamples;
        }
        enginelab::ExhaustAcousticSample acousticSample;
        while (simulator.tryPopExhaustAcousticSample(acousticSample)) {
            counters_.droppedReactionEvents +=
                acousticSample.droppedReactionEventCount;
            const auto fraction = std::clamp(
                (acousticSample.timeSeconds - simulationStart) / stepSeconds,
                0.0, 1.0);
            acousticSample.timeSeconds =
                realtimeSeconds_ + fraction * stepSeconds;
            for (std::size_t eventIndex = 0;
                 eventIndex < acousticSample.reactionEventCount;
                 ++eventIndex) {
                const auto& observed =
                    acousticSample.reactionEvents[eventIndex];
                ++reactionDiagnostics_.eventCount;
                reactionDiagnostics_.releasedEnergyJoules +=
                    observed.releasedEnergyJoules;
                reactionDiagnostics_.energyWeightedAxial +=
                    observed.releasedEnergyJoules
                        * observed.axialPosition;
                const auto powerW = observed.durationSeconds > 0.0F
                    ? static_cast<double>(observed.releasedEnergyJoules)
                        / observed.durationSeconds
                    : 0.0;
                reactionDiagnostics_.maximumPowerW = std::max(
                    reactionDiagnostics_.maximumPowerW, powerW);
                reactionDiagnostics_.maximumCompactPressureJumpPa = std::max(
                    reactionDiagnostics_.maximumCompactPressureJumpPa,
                    enginelab::ThermoacousticHeatReleaseSource::
                        compactPressureJumpPa(
                            powerW, observed.flowAreaM2,
                            observed.speedOfSoundMps));
                const auto eventFraction = std::clamp(
                    (acousticSample.reactionEvents[eventIndex].timeSeconds
                        - simulationStart) / stepSeconds,
                    0.0, 1.0);
                acousticSample.reactionEvents[eventIndex].timeSeconds =
                    realtimeSeconds_ + eventFraction * stepSeconds;
            }
            // Instrumental A/B: remove only the copied acoustic observations.
            // The finite-volume chemistry, heat release, wall state and engine
            // trajectory have already advanced and remain bit-identical.
            if (!reactionAcousticsEnabled_)
                acousticSample.reactionEventCount = 0;
            if (!runtime_->exhaustAcousticSamples().tryPush(acousticSample))
                ++counters_.droppedExhaustAcousticSamples;
        }
        enginelab::publishAudioFrame(runtime_->audioState(), frame.state,
            { false, starterEngaged, drivelineLoad, 1.0 });
        runtime_->audioState().producerTimeNanoseconds.store(
            static_cast<std::uint64_t>(
                (realtimeSeconds_ + stepSeconds) * 1.0e9),
            std::memory_order_release);
        block_.clear();
        const auto renderStart = std::chrono::steady_clock::now();
        renderer_->render(block_, 0, audioSamplesPerStep);
        const auto renderEnd = std::chrono::steady_clock::now();
        renderDurationsSeconds_.push_back(
            std::chrono::duration<double>(renderEnd - renderStart).count());
        for (int sample = 0; sample < audioSamplesPerStep; ++sample) {
            left_.push_back(block_.getSample(0, sample));
            right_.push_back(block_.getSample(1, sample));
        }
        realtimeSeconds_ += stepSeconds;
    }

    [[nodiscard]] std::size_t sampleCount() const noexcept {
        return left_.size();
    }
    [[nodiscard]] const std::vector<float>& left() const noexcept {
        return left_;
    }
    [[nodiscard]] const std::vector<float>& right() const noexcept {
        return right_;
    }
    [[nodiscard]] AudioContractCounters audioContractCounters() const noexcept {
        auto result = counters_;
        result.lateEvents = renderer_->lateEventCount();
        result.droppedPendingEvents = renderer_->droppedPendingEventCount();
        result.stolenVoices = renderer_->stolenVoiceCount();
        result.reactionPressureLimitedSamples =
            renderer_->reactionPressureLimitedSampleCount();
        result.delayTruncations = renderer_->delayTruncationCount();
        result.legacyPathSamples = renderer_->legacyPathSampleCount();
        result.invalidBoundarySamples = renderer_->invalidBoundarySampleCount();
        result.levelLimitedSamples = renderer_->levelLimitedSampleCount();
        return result;
    }
    [[nodiscard]] ReactionAcousticDiagnostics reactionDiagnostics() const noexcept {
        auto result = reactionDiagnostics_;
        if (result.releasedEnergyJoules > 0.0)
            result.energyWeightedAxial /= result.releasedEnergyJoules;
        return result;
    }
    void resetReactionDiagnostics() noexcept {
        reactionDiagnostics_ = {};
    }
    [[nodiscard]] RealtimeTiming renderTiming() const {
        const auto budgetSeconds = static_cast<double>(audioSamplesPerStep)
            / audioSampleRate;
        return summariseRealtimeTiming(renderDurationsSeconds_, budgetSeconds);
    }
    void resetRenderTiming() noexcept {
        renderDurationsSeconds_.clear();
    }
    [[nodiscard]] bool physicalExhaustActive() const noexcept {
        return renderer_->physicalExhaustActive();
    }
    [[nodiscard]] bool compiledExhaustTopologyActive() const noexcept {
        return renderer_->compiledExhaustTopologyActive();
    }

private:
    std::unique_ptr<enginelab::EngineRuntime> runtime_;
    std::unique_ptr<enginelab::RealtimeEngineAudio> renderer_;
    juce::AudioBuffer<float> block_ { 2, audioSamplesPerStep };
    std::vector<float> left_;
    std::vector<float> right_;
    double realtimeSeconds_ {};
    AudioContractCounters counters_ {};
    ReactionAcousticDiagnostics reactionDiagnostics_ {};
    std::vector<double> renderDurationsSeconds_ {};
    bool reactionAcousticsEnabled_ { true };
};

struct TickResult final {
    enginelab::DrivelineOutput drive {};
    enginelab::SimulationFrame frame {};
    double engineThrottle { 0.0 };
};

TickResult coupledStep(enginelab::EngineSimulator& simulator,
                       enginelab::DrivelineModel& driveline, double throttleCmd,
                       double clutchPedal, bool starter,
                       double brakePressure = 0.0) {
    // Same ordering as EngineRuntime::run: the driveline advances on the
    // PREVIOUS engine state, then the simulator steps with this tick's reaction.
    TickResult result;
    result.drive = driveline.advance(
        stepSeconds, simulator.state(), 0.0, clutchPedal, brakePressure);
    enginelab::EngineControls controls;
    controls.ignitionEnabled = true;
    controls.starterEngaged = starter;
    result.engineThrottle =
        std::clamp(throttleCmd, 0.0, 1.0) * result.drive.torqueCutMultiplier;
    controls.throttle = result.engineThrottle;
    controls.externalTorqueNm = result.drive.engineCouplingTorqueNm;
    controls.externalRotatingInertiaKgM2 =
        result.drive.reflectedRotatingInertiaKgM2;
    result.frame = simulator.step(stepSeconds, controls);
    return result;
}

[[nodiscard]] double percentile(std::vector<double> values, double fraction) {
    if (values.empty()) return 0.0;
    const auto index = std::min(values.size() - 1,
        static_cast<std::size_t>(std::floor(
            fraction * static_cast<double>(values.size() - 1))));
    std::nth_element(values.begin(),
        values.begin() + static_cast<std::ptrdiff_t>(index), values.end());
    return values[index];
}

struct Options final {
    std::string engineFilter;
    bool trace { false };
    bool list { false };
    std::filesystem::path audioDirectory;
    std::filesystem::path impulseResponsePath;
    // Default means "use the selected engine exactly as authored". Individual
    // optionals are explicit overrides; --force-demo recreates the historical
    // 0.18 / 900 K / 10 ms continuous laboratory scenario.
    bool forceDemo { false };
    std::optional<double> fuelFraction;
    std::optional<double> ignitionTemperatureK;
    std::optional<double> reactionMilliseconds;
    double overrunSeconds { 3.0 };
    // Long enough for 1.5 mm of steel to arrive; see phase C.
    double warmupSeconds { 30.0 };
    // Zero keeps the historical continuous strategy (anti-lag).
    std::optional<double> pulseHz;
    std::optional<double> pulseDuty;
    std::optional<double> pulseTimingVariation;
    /** Same-binary null for the schema-8 induction correlation. */
    bool flatInductionControl { false };
    bool reactionAcousticsEnabled { true };
    // Speed the measured overrun opens at. Must be controlled: left to the
    // warm-up it lands on the rev limiter, where engine pumping buries the
    // afterfire and the audio verdict is about the wrong thing entirely.
    double liftOffRpm { 4'000.0 };
};

[[nodiscard]] const char* afterfireStrategyName(
    enginelab::ExhaustAfterfireStrategy strategy) noexcept {
    switch (strategy) {
    case enginelab::ExhaustAfterfireStrategy::cleanDfco:
        return "clean_dfco";
    case enginelab::ExhaustAfterfireStrategy::continuousAntiLag:
        return "continuous_anti_lag";
    case enginelab::ExhaustAfterfireStrategy::discreteAfterfire:
        return "discrete_afterfire";
    }
    return "unknown";
}

void applyCalibrationOverrides(enginelab::EngineConfig& config,
                               const Options& options) {
    auto& afterfire = config.exhaustAfterfire;
    if (options.forceDemo) {
        afterfire.overrunFuelFraction = 0.18;
        afterfire.ignitionTemperatureK = 900.0;
        afterfire.reactionTimeConstantSeconds = 0.010;
        afterfire.overrunPulseHz = 0.0;
        afterfire.overrunPulseDutyCycle = 0.35;
        afterfire.overrunPulseTimingVariation = 0.25;
        afterfire.inductionActivationTemperatureK = 0.0;
        afterfire.inductionPressureExponent = 0.0;
        afterfire.inductionEquivalenceRatioExponent = 0.0;
        afterfire.inductionDecayTimeSeconds =
            2.0 * afterfire.inductionTimeSeconds;
        afterfire.overrunMinimumRpm =
            std::clamp(config.idleRpm * 1.8, 500.0, 20'000.0);
    }
    if (options.fuelFraction)
        afterfire.overrunFuelFraction = *options.fuelFraction;
    if (options.ignitionTemperatureK)
        afterfire.ignitionTemperatureK = *options.ignitionTemperatureK;
    if (options.reactionMilliseconds)
        afterfire.reactionTimeConstantSeconds =
            *options.reactionMilliseconds * 0.001;
    if (options.pulseHz)
        afterfire.overrunPulseHz = *options.pulseHz;
    if (options.pulseDuty)
        afterfire.overrunPulseDutyCycle = *options.pulseDuty;
    if (options.pulseTimingVariation)
        afterfire.overrunPulseTimingVariation =
            *options.pulseTimingVariation;
    if (options.flatInductionControl) {
        afterfire.inductionActivationTemperatureK = 0.0;
        afterfire.inductionPressureExponent = 0.0;
        afterfire.inductionEquivalenceRatioExponent = 0.0;
        afterfire.inductionDecayTimeSeconds =
            2.0 * afterfire.inductionTimeSeconds;
    }

    // Strategy is only inferred when the caller explicitly changes delivery.
    // Merely overriding a chemistry parameter must not rewrite ECU intent.
    if (options.forceDemo || options.fuelFraction || options.pulseHz) {
        afterfire.strategy = afterfire.overrunFuelFraction <= 0.0
            ? enginelab::ExhaustAfterfireStrategy::cleanDfco
            : (afterfire.overrunPulseHz > 0.0
                ? enginelab::ExhaustAfterfireStrategy::discreteAfterfire
                : enginelab::ExhaustAfterfireStrategy::continuousAntiLag);
        afterfire.enabled = enginelab::afterfireRetainsFuel(afterfire.strategy);
    }
}

Metrics measureAfterfire(const enginelab::EngineConfig& baseConfig,
                         const Options& options) {
    auto config = baseConfig;
    applyCalibrationOverrides(config, options);

    enginelab::SimpleEcuModel ecu;
    enginelab::SimplifiedGasolinePhysics physics;
    enginelab::FourStrokeEventGenerator events;
    auto exhaust = enginelab::ExhaustGraph::makeForEngine(config);
    enginelab::EngineSimulator simulator(config, ecu, physics, events, exhaust);
    enginelab::DrivelineModel driveline(config);
    std::unique_ptr<AudioCapture> audio;
    if (!options.audioDirectory.empty()) {
        simulator.setPressureSamplingEnabled(true);
        audio = std::make_unique<AudioCapture>(
            config, options.impulseResponsePath,
            options.reactionAcousticsEnabled);
    }

    Metrics metrics;
    metrics.name = config.name;
    metrics.effectiveCalibration = config.exhaustAfterfire;
    metrics.calibrationOverridden = options.forceDemo
        || options.fuelFraction || options.ignitionTemperatureK
        || options.reactionMilliseconds || options.pulseHz
        || options.pulseDuty || options.pulseTimingVariation
        || options.flatInductionControl;
    metrics.reactionAcousticsEnabled = options.reactionAcousticsEnabled;
    const auto captureAudioContract = [&metrics, &audio]() {
        if (!audio) return;
        metrics.audioMeasured = true;
        metrics.audioContract = audio->audioContractCounters();
        if (!metrics.reactionWindowCaptured)
            metrics.reactionAcoustics = audio->reactionDiagnostics();
        if (!metrics.reactionWindowCaptured)
            metrics.reactionWindowRenderTiming = audio->renderTiming();
        metrics.physicalExhaustActive = audio->physicalExhaustActive();
        metrics.compiledExhaustTopologyActive =
            audio->compiledExhaustTopologyActive();
    };

    std::vector<Sample> samples;
    samples.reserve(8192);
    auto t = 0.0;
    const auto record = [&](const TickResult& tick, double throttleCmd) {
        const auto& s = tick.frame.state;
        Sample sample;
        sample.t = t;
        sample.rpm = s.rpm;
        sample.throttle = throttleCmd;
        sample.heatKw = s.exhaustAfterfireHeatReleaseKw;
        sample.fuelBurnMgPerSecond = s.exhaustAfterfireFuelBurnMgPerSecond;
        sample.meteredFuelGrams = s.fuelConsumedGrams;
        sample.deliveredFuelMgPerSecond = s.deliveredFuelMgPerCycle
            * std::max(0.0, s.rpm) / 120.0;
        sample.overrunActive = s.exhaustAfterfireOverrunActive;
        sample.exhaustPortPeakKpa = s.exhaustPressureKpa;
        sample.exhaustGasC = s.exhaustTemperatureC;
        sample.exhaustWallC = s.exhaustWallTemperatureC;
        sample.wallIgnitedFraction = s.exhaustAfterfireWallIgnitedFraction;
        sample.inductionProgress =
            s.exhaustAfterfireInductionProgress;
        sample.minimumInductionDelayMs =
            s.exhaustAfterfireMinimumInductionDelayMs;
        sample.maximumInductionDelayMs =
            s.exhaustAfterfireMaximumInductionDelayMs;
        samples.push_back(sample);
    };

    // Phase A -- crank to idle in neutral.
    driveline.requestGear(-1);
    for (int step = 0; step < static_cast<int>(4.0 / stepSeconds);
         ++step, t += stepSeconds) {
        const auto starter = simulator.state().rpm < config.idleRpm * 0.85
            && t < 3.0;
        auto tick = coupledStep(simulator, driveline, 0.0, 0.0, starter);
        if (audio)
            audio->renderFrame(tick.frame, simulator, starter,
                tick.drive.requestedLoad);
        record(tick, 0.0);
    }
    // Phase B -- launch in first, clutch fed in over 0.9 s.
    driveline.requestGear(0);
    for (int step = 0; step < static_cast<int>(0.9 / stepSeconds);
         ++step, t += stepSeconds) {
        const auto clutch = std::clamp(
            static_cast<double>(step) * stepSeconds / 0.9, 0.0, 1.0);
        auto tick = coupledStep(simulator, driveline, 1.0, clutch, false);
        if (audio)
            audio->renderFrame(tick.frame, simulator, false,
                tick.drive.requestedLoad);
        record(tick, 1.0);
    }

    // Phase C -- HEAT THE PIPE, then reach the arming speed.
    //
    // This phase exists because of a measurement, not for realism's sake. The
    // afterfire ignition source is the pipe wall, and 1.5 mm of steel is about
    // 5.9 kJ/m2K against an exhaust-side film of a few hundred W/m2K -- a time
    // constant of tens of seconds. An engine that has just been started has a
    // wall at ambient and CANNOT pop, correctly. Lifting off six seconds after
    // cranking therefore measures a cold exhaust and reports a null that says
    // nothing about the model: the first version of this harness did exactly
    // that and the wall-ignition path it was written to test came out
    // bit-identical to no path at all.
    //
    // So hold the engine at load long enough for the wall to arrive, shifting
    // up as needed so the run does not end against the rev limiter.
    const auto armRpm = std::max(
        config.exhaustAfterfire.overrunMinimumRpm * 1.35,
        config.idleRpm * 2.6);
    auto reachedArm = false;
    const auto warmupSteps =
        static_cast<int>(options.warmupSeconds / stepSeconds);
    const auto maximumSteps = warmupSteps + static_cast<int>(14.0 / stepSeconds);
    for (int step = 0; step < maximumSteps; ++step, t += stepSeconds) {
        auto tick = coupledStep(simulator, driveline, 1.0, 1.0, false);
        if (audio)
            audio->renderFrame(tick.frame, simulator, false,
                tick.drive.requestedLoad);
        record(tick, 1.0);
        // Keep pulling without hitting the limiter: shift up near the top and
        // let it climb again, which is also how a driver warms an exhaust.
        if (!tick.drive.shiftInProgress
            && tick.frame.state.rpm > 0.88 * config.redlineRpm
            && tick.drive.engagedGear >= 0
            && tick.drive.engagedGear
                < static_cast<int>(config.transmission.gearRatios.size()) - 1)
            driveline.shiftUp();
        if (step >= warmupSteps && tick.frame.state.rpm >= armRpm) {
            reachedArm = true;
            break;
        }
    }
    if (!reachedArm) {
        captureAudioContract();
        return metrics;
    }

    // Phase C2 -- BRAKE DOWN under positive throttle to the speed where the
    // real lift-off is to be measured.
    //
    // Without this the recorded window starts wherever the warm-up left the
    // engine, which is the rev limiter: phase C shifts up and holds WOT for
    // however long the wall needs, so by the time the arming test is reached
    // the engine is at redline and lifts off from 9,999 rpm. That is the wrong
    // operating point for this question and it is not a small error -- at
    // redline the pumping and blowdown of the engine itself dominate the port
    // signal, so a few kPa of afterfire sits under it and any audio verdict is
    // a verdict about pumping noise. A real trailing-throttle pop happens at
    // moderate speed. Same class of mistake as reading `--trace <low rpm>` as
    // an idle when the dyno controller is holding WOT.
    //
    // The old harness closed the throttle here, so the ECU began afterfire for
    // several seconds before the timestamp it labelled "lift-off". It measured
    // an arbitrary late slice of a coast and hid the onset users actually hear.
    // Vehicle braking with 28% throttle keeps the strategy armed and the pipe
    // loaded while removing road speed; Phase D below is now the unique falling
    // edge of the pedal command.
    for (int step = 0; step < static_cast<int>(20.0 / stepSeconds)
             && simulator.state().rpm > options.liftOffRpm;
         ++step, t += stepSeconds) {
        auto tick = coupledStep(
            simulator, driveline, 0.28, 1.0, false, 1.0);
        if (audio)
            audio->renderFrame(tick.frame, simulator, false,
                tick.drive.requestedLoad);
        record(tick, 0.28);
    }

    // Phase C3 -- a fixed loaded proof immediately before lift-off. The first
    // 250 ms flushes any wet-limiter state left by the warm-up; the final 500 ms
    // must contain no reaction event. This makes the operating-state gate a
    // falsifiable harness contract instead of relying on a particular coast
    // duration in Phase C2.
    constexpr auto loadedProofSeconds = 0.75;
    constexpr auto loadedProofFlushSeconds = 0.25;
    if (audio) audio->resetReactionDiagnostics();
    for (int step = 0;
         step < static_cast<int>(loadedProofSeconds / stepSeconds);
         ++step, t += stepSeconds) {
        auto tick = coupledStep(
            simulator, driveline, 0.28, 1.0, false);
        if (audio)
            audio->renderFrame(tick.frame, simulator, false,
                tick.drive.requestedLoad);
        record(tick, 0.28);
        if (audio
            && step == static_cast<int>(
                loadedProofFlushSeconds / stepSeconds)) {
            audio->resetReactionDiagnostics();
            metrics.preLiftReactionWindowMeasured = true;
        }
    }
    metrics.wallTemperatureAtLiftOffC = simulator.state().exhaustWallTemperatureC;
    metrics.liftOffTime = t;
    metrics.rpmAtLiftOff = simulator.state().rpm;
    const auto liftOffIndex = samples.size();
    if (audio) {
        if (metrics.preLiftReactionWindowMeasured)
            metrics.preLiftReactionAcoustics =
                audio->reactionDiagnostics();
        audio->resetReactionDiagnostics();
        audio->resetRenderTiming();
    }

    // Phase D -- lift off. Closed throttle, clutch home, engine driven by the
    // car. This is the overrun the pops are supposed to live in.
    std::vector<double> overrunSimulationDurationsSeconds;
    overrunSimulationDurationsSeconds.reserve(
        static_cast<std::size_t>(options.overrunSeconds / stepSeconds));
    for (int step = 0;
         step < static_cast<int>(options.overrunSeconds / stepSeconds);
         ++step, t += stepSeconds) {
        const auto simulationStart = std::chrono::steady_clock::now();
        auto tick = coupledStep(simulator, driveline, 0.0, 1.0, false);
        const auto simulationEnd = std::chrono::steady_clock::now();
        overrunSimulationDurationsSeconds.push_back(
            std::chrono::duration<double>(
                simulationEnd - simulationStart).count());
        if (audio)
            audio->renderFrame(tick.frame, simulator, false,
                tick.drive.requestedLoad);
        record(tick, 0.0);
    }
    metrics.reactionWindowSimulationTiming = summariseRealtimeTiming(
        std::move(overrunSimulationDurationsSeconds), stepSeconds);
    metrics.tipInTime = t;
    const auto tipInIndex = samples.size();
    if (audio) {
        metrics.reactionAcoustics = audio->reactionDiagnostics();
        metrics.reactionWindowRenderTiming = audio->renderTiming();
        metrics.reactionWindowCaptured = true;
    }

    // Phase E -- tip back in. A model that only accumulates and never releases
    // during the overrun still has to dump somewhere, and this is where it
    // would show up.
    for (int step = 0; step < static_cast<int>(1.5 / stepSeconds);
         ++step, t += stepSeconds) {
        auto tick = coupledStep(simulator, driveline, 1.0, 1.0, false);
        if (audio)
            audio->renderFrame(tick.frame, simulator, false,
                tick.drive.requestedLoad);
        record(tick, 1.0);
    }

    metrics.ran = true;

    // --- heat-release statistics over the SETTLED overrun ------------------
    //
    // The first half second after lift-off is the throttle-closing transient,
    // and it is several times larger than anything the overrun itself does.
    // Including it does not merely add noise, it BREAKS the burst statistics:
    // the event threshold is a fraction of the window peak, so a 16.9 kW
    // transient puts the floor at 0.85 kW while the bursts swing 0.12-2.57 kW,
    // and every one of them is then reported as a single continuous event at
    // 100 % duty. That is exactly the trap this project documents about any
    // aggregate taken over a window spanning widely different levels -- it
    // reports the loudest part -- and it nearly refuted a correct hypothesis
    // here. Measure the settled overrun on its own.
    const auto settleSteps = std::min<std::size_t>(
        static_cast<std::size_t>(0.5 / stepSeconds),
        (tipInIndex - liftOffIndex) / 2);
    const auto steadyBegin = liftOffIndex + settleSteps;
    auto heatSum = 0.0;
    auto wallIgnitedSum = 0.0;
    auto activeSteps = 0;
    std::vector<double> overrunHeat;
    overrunHeat.reserve(tipInIndex - steadyBegin);
    for (auto index = steadyBegin; index < tipInIndex; ++index) {
        const auto& sample = samples[index];
        overrunHeat.push_back(sample.heatKw);
        heatSum += sample.heatKw;
        metrics.peakHeatKw = std::max(metrics.peakHeatKw, sample.heatKw);
        metrics.burnedFuelMg += sample.fuelBurnMgPerSecond * stepSeconds;
        metrics.deliveredFuelMg +=
            sample.deliveredFuelMgPerSecond * stepSeconds;
        if (sample.overrunActive) ++metrics.overrunActiveSteps;
        // Ignore sub-milliwatt numerical residue. A clean-DFCO run can carry a
        // positive denormal-sized heat value even though no fuel is consumed;
        // treating that as 100 % duty and one burst makes the diagnostic lie.
        if (sample.heatKw > 1.0e-6) ++activeSteps;
        metrics.portPeakDuringOverrunKpa = std::max(
            metrics.portPeakDuringOverrunKpa, sample.exhaustPortPeakKpa);
        metrics.wallTemperatureMinDuringOverrunC =
            index == steadyBegin ? sample.exhaustWallC
                : std::min(metrics.wallTemperatureMinDuringOverrunC,
                    sample.exhaustWallC);
        metrics.peakWallIgnitedFraction = std::max(
            metrics.peakWallIgnitedFraction, sample.wallIgnitedFraction);
        metrics.peakInductionProgress = std::max(
            metrics.peakInductionProgress, sample.inductionProgress);
        if (sample.minimumInductionDelayMs > 0.0) {
            metrics.minimumInductionDelayMs =
                metrics.minimumInductionDelayMs > 0.0
                ? std::min(metrics.minimumInductionDelayMs,
                    sample.minimumInductionDelayMs)
                : sample.minimumInductionDelayMs;
        }
        metrics.maximumInductionDelayMs = std::max(
            metrics.maximumInductionDelayMs,
            sample.maximumInductionDelayMs);
        wallIgnitedSum += sample.wallIgnitedFraction;
        metrics.troughHeatKw = index == steadyBegin
            ? sample.heatKw : std::min(metrics.troughHeatKw, sample.heatKw);
    }
    metrics.meanWallIgnitedFraction =
        wallIgnitedSum / static_cast<double>(
            std::max<std::size_t>(1, tipInIndex - steadyBegin));
    if (tipInIndex > steadyBegin)
        metrics.meteredFuelMg = std::max(0.0,
            (samples[tipInIndex - 1].meteredFuelGrams
                - samples[steadyBegin].meteredFuelGrams) * 1'000.0);
    // The direct "is it bursty" reading, and the one that needs no threshold
    // at all: peak over trough across the settled overrun.
    metrics.modulationDepth = metrics.peakHeatKw
        / std::max(1.0e-6, metrics.troughHeatKw);
    const auto overrunSteps = std::max<std::size_t>(1, overrunHeat.size());
    metrics.meanHeatKw = heatSum / static_cast<double>(overrunSteps);
    metrics.crest = metrics.peakHeatKw
        / std::max(1.0e-9, metrics.meanHeatKw);
    metrics.dutyCycle = static_cast<double>(activeSteps)
        / static_cast<double>(overrunSteps);

    // Baseline port pressure from the second before lift-off, so the overrun
    // peak has something to be a peak ABOVE.
    {
        auto sum = 0.0;
        auto count = 0;
        const auto begin = liftOffIndex > 240 ? liftOffIndex - 240
                                              : std::size_t { 0 };
        for (auto index = begin; index < liftOffIndex; ++index) {
            sum += samples[index].exhaustPortPeakKpa;
            ++count;
        }
        metrics.portBaselineKpa = count > 0
            ? sum / static_cast<double>(count) : 0.0;
    }

    // Discrete events: an excursion is a contiguous run above a threshold set
    // from the window's own upper quartile, which makes it scale-free rather
    // than a tuned kW number that would only suit one displacement. A smear
    // produces one enormous "event" covering the whole window, which the duty
    // cycle then exposes; a bang produces several short ones.
    if (metrics.peakHeatKw > 1.0e-6) {
        // Threshold from the settled window's OWN median, halfway between
        // trough and peak in the log sense. Anchoring it to a fraction of the
        // peak is what hid a 21x modulation behind "one continuous event".
        const auto threshold = std::max(
            std::sqrt(std::max(1.0e-9, metrics.troughHeatKw)
                * metrics.peakHeatKw),
            0.5 * percentile(overrunHeat, 0.50));
        auto inEvent = false;
        std::size_t eventStart = 0;
        std::vector<double> eventStartsSeconds;
        for (std::size_t index = 0; index < overrunHeat.size(); ++index) {
            const auto above = overrunHeat[index] > threshold;
            if (above && !inEvent) { inEvent = true; eventStart = index; }
            if ((!above || index + 1 == overrunHeat.size()) && inEvent) {
                inEvent = false;
                ++metrics.eventCount;
                eventStartsSeconds.push_back(
                    static_cast<double>(eventStart) * stepSeconds);
                auto peak = 0.0;
                std::size_t peakIndex = eventStart;
                for (auto scan = eventStart; scan <= index; ++scan) {
                    if (overrunHeat[scan] > peak) {
                        peak = overrunHeat[scan];
                        peakIndex = scan;
                    }
                }
                if (peak > metrics.largestEventPeakKw) {
                    metrics.largestEventPeakKw = peak;
                    // 10-90 % rise measured backwards from the peak.
                    auto tenIndex = peakIndex;
                    auto ninetyIndex = peakIndex;
                    for (auto scan = peakIndex; scan > eventStart; --scan) {
                        if (overrunHeat[scan] >= 0.9 * peak) ninetyIndex = scan;
                        if (overrunHeat[scan] >= 0.1 * peak) tenIndex = scan;
                    }
                    metrics.largestEventRiseMs =
                        static_cast<double>(ninetyIndex - tenIndex)
                        * stepSeconds * 1'000.0;
                }
            }
        }
        if (eventStartsSeconds.size() > 1) {
            auto intervalSum = 0.0;
            std::vector<double> intervals;
            intervals.reserve(eventStartsSeconds.size() - 1);
            for (std::size_t index = 1;
                 index < eventStartsSeconds.size(); ++index) {
                const auto interval = eventStartsSeconds[index]
                    - eventStartsSeconds[index - 1];
                intervals.push_back(interval);
                intervalSum += interval;
            }
            const auto mean = intervalSum
                / static_cast<double>(intervals.size());
            auto varianceSum = 0.0;
            metrics.minimumEventIntervalMs = intervals.front() * 1'000.0;
            metrics.maximumEventIntervalMs = intervals.front() * 1'000.0;
            for (const auto interval : intervals) {
                const auto delta = interval - mean;
                varianceSum += delta * delta;
                metrics.minimumEventIntervalMs = std::min(
                    metrics.minimumEventIntervalMs, interval * 1'000.0);
                metrics.maximumEventIntervalMs = std::max(
                    metrics.maximumEventIntervalMs, interval * 1'000.0);
            }
            metrics.eventIntervalStdDevMs = std::sqrt(
                varianceSum / static_cast<double>(intervals.size())) * 1'000.0;
        }
    }

    // --- audio ------------------------------------------------------------
    if (audio) {
        captureAudioContract();
        const auto& left = audio->left();
        for (const auto sample : left) {
            if (!std::isfinite(sample)) { metrics.audioFinite = false; continue; }
            metrics.audioPeak = std::max(metrics.audioPeak,
                std::abs(static_cast<double>(sample)));
        }
        const auto samplesPerStep =
            static_cast<std::size_t>(audioSamplesPerStep);
        const auto overrunBegin = std::min(left.size(),
            liftOffIndex * samplesPerStep);
        const auto overrunEnd = std::min(left.size(),
            tipInIndex * samplesPerStep);
        const auto baselineBegin = overrunBegin
            > static_cast<std::size_t>(audioSampleRate)
            ? overrunBegin - static_cast<std::size_t>(audioSampleRate)
            : std::size_t { 0 };
        std::vector<double> baseline;
        std::vector<double> overrun;
        baseline.reserve(overrunBegin - baselineBegin);
        overrun.reserve(overrunEnd - overrunBegin);
        auto baselineSquareSum = 0.0;
        auto baselinePeak = 0.0;
        auto overrunSquareSum = 0.0;
        for (auto index = baselineBegin; index < overrunBegin; ++index) {
            const auto magnitude = std::abs(static_cast<double>(left[index]));
            baseline.push_back(magnitude);
            baselineSquareSum += magnitude * magnitude;
            baselinePeak = std::max(baselinePeak, magnitude);
        }
        for (auto index = overrunBegin; index < overrunEnd; ++index) {
            const auto magnitude = std::abs(static_cast<double>(left[index]));
            overrun.push_back(magnitude);
            overrunSquareSum += magnitude * magnitude;
            metrics.audioOverrunPeak =
                std::max(metrics.audioOverrunPeak, magnitude);
        }
        metrics.audioBaselineP999 = percentile(baseline, 0.999);
        metrics.audioOverrunP999 = percentile(overrun, 0.999);
        if (!overrun.empty()) {
            const auto rms = std::sqrt(overrunSquareSum
                / static_cast<double>(overrun.size()));
            metrics.audioOverrunCrest =
                metrics.audioOverrunPeak / std::max(1.0e-9, rms);
        }
        if (!baseline.empty()) {
            const auto rms = std::sqrt(baselineSquareSum
                / static_cast<double>(baseline.size()));
            metrics.audioBaselineCrest = baselinePeak / std::max(1.0e-9, rms);
        }
        if (!options.audioDirectory.empty()) {
            std::filesystem::create_directories(options.audioDirectory);
            writeWav(options.audioDirectory
                    / (safeFileStem(config.name) + "_afterfire.wav"),
                audio->left(), audio->right());
        }
    }

    if (options.trace) {
        std::printf("# %s\n", config.name.c_str());
        std::printf("t_s,rpm,throttle,heat_kw,fuel_mg_s,port_kpa,egt_c,"
                    "wall_c,induction_i,induction_min_ms,"
                    "induction_max_ms,armed\n");
        const auto traceBegin = liftOffIndex > 240 ? liftOffIndex - 240
                                                   : std::size_t { 0 };
        for (std::size_t index = traceBegin; index < samples.size(); ++index) {
            const auto& sample = samples[index];
            std::printf("%.4f,%.1f,%.2f,%.4f,%.3f,%.1f,%.1f,%.1f,"
                        "%.4f,%.4f,%.4f,%d\n",
                sample.t, sample.rpm, sample.throttle, sample.heatKw,
                sample.fuelBurnMgPerSecond, sample.exhaustPortPeakKpa,
                sample.exhaustGasC, sample.exhaustWallC,
                sample.inductionProgress, sample.minimumInductionDelayMs,
                sample.maximumInductionDelayMs,
                sample.overrunActive ? 1 : 0);
        }
    }
    return metrics;
}

[[nodiscard]] bool violatesAudioContract(const Metrics& metrics) noexcept {
    if (!metrics.audioMeasured) return false;
    const auto& counters = metrics.audioContract;
    return !metrics.audioFinite
        || !metrics.physicalExhaustActive
        || !metrics.compiledExhaustTopologyActive
        || (metrics.preLiftReactionWindowMeasured
            && metrics.preLiftReactionAcoustics.eventCount > 0)
        || metrics.reactionWindowRenderTiming.overBudgetBlockCount > 0
        || counters.droppedFiringEvents > 0
        || counters.droppedCylinderPressureSamples > 0
        || counters.droppedExhaustAcousticSamples > 0
        || counters.droppedReactionEvents > 0
        || counters.reactionPressureLimitedSamples > 0
        || counters.lateEvents > 0
        || counters.droppedPendingEvents > 0
        || counters.stolenVoices > 0
        || counters.delayTruncations > 0
        || counters.legacyPathSamples > 0
        || counters.invalidBoundarySamples > 0
        || counters.levelLimitedSamples > 0;
}

[[nodiscard]] bool violatesSimulationBudget(const Metrics& metrics) noexcept {
    return metrics.reactionWindowSimulationTiming.overBudgetBlockCount > 0;
}

[[nodiscard]] bool violatesPhysicalAfterfireContract(
    const Metrics& metrics) noexcept {
    if (!metrics.ran) return true;
    const auto expectsAfterfire = enginelab::afterfireRetainsFuel(
            metrics.effectiveCalibration.strategy)
        && metrics.effectiveCalibration.overrunFuelFraction > 0.0;
    if (!expectsAfterfire) return false;
    return metrics.overrunActiveSteps == 0
        || !(metrics.deliveredFuelMg > 0.0)
        || !(metrics.burnedFuelMg > 0.0)
        || metrics.eventCount == 0
        || metrics.peakInductionProgress < 1.0
        || (metrics.audioMeasured
            && metrics.reactionAcoustics.eventCount == 0);
}

void reportAudioContract(const Metrics& metrics) {
    if (!metrics.audioMeasured) return;
    const auto& counters = metrics.audioContract;
    std::printf("    audio contract physical=%s compiled=%s"
                " dropF=%llu dropP=%llu dropA=%llu dropR=%llu"
                " reactLimit=%llu late=%llu pending=%llu stolen=%llu trunc=%llu"
                " boundary=%llu legacy=%llu leveler=%llu  %s\n",
        metrics.physicalExhaustActive ? "yes" : "NO",
        metrics.compiledExhaustTopologyActive ? "yes" : "NO",
        static_cast<unsigned long long>(counters.droppedFiringEvents),
        static_cast<unsigned long long>(
            counters.droppedCylinderPressureSamples),
        static_cast<unsigned long long>(
            counters.droppedExhaustAcousticSamples),
        static_cast<unsigned long long>(counters.droppedReactionEvents),
        static_cast<unsigned long long>(
            counters.reactionPressureLimitedSamples),
        static_cast<unsigned long long>(counters.lateEvents),
        static_cast<unsigned long long>(counters.droppedPendingEvents),
        static_cast<unsigned long long>(counters.stolenVoices),
        static_cast<unsigned long long>(counters.delayTruncations),
        static_cast<unsigned long long>(counters.invalidBoundarySamples),
        static_cast<unsigned long long>(counters.legacyPathSamples),
        static_cast<unsigned long long>(counters.levelLimitedSamples),
        violatesAudioContract(metrics) ? "INVALID" : "valid");
}

void report(const Metrics& metrics) {
    const auto& calibration = metrics.effectiveCalibration;
    std::printf("    calibration %s  strategy=%s enabled=%s fuel=%.3f"
                " ignition=%.1f K reaction=%.2f ms induction=%.2f ms\n",
        metrics.calibrationOverridden ? "OVERRIDDEN" : "authored",
        afterfireStrategyName(calibration.strategy),
        calibration.enabled ? "yes" : "no",
        calibration.overrunFuelFraction,
        calibration.ignitionTemperatureK,
        calibration.reactionTimeConstantSeconds * 1'000.0,
        calibration.inductionTimeSeconds * 1'000.0);
    std::printf("    induction reference=%.3f kPa Ea/R=%.1f K"
                " p_exp=%.3f phi_exp=%.3f decay=%.2f ms\n",
        calibration.inductionReferencePressureKpa,
        calibration.inductionActivationTemperatureK,
        calibration.inductionPressureExponent,
        calibration.inductionEquivalenceRatioExponent,
        calibration.inductionDecayTimeSeconds * 1'000.0);
    std::printf("    delivery pulse=%.3f Hz duty=%.3f variation=%.3f"
                " min_rpm=%.0f max_throttle=%.3f quench=%.1f K"
                " reaction_audio=%s\n",
        calibration.overrunPulseHz,
        calibration.overrunPulseDutyCycle,
        calibration.overrunPulseTimingVariation,
        calibration.overrunMinimumRpm,
        calibration.overrunMaximumThrottle,
        calibration.quenchTemperatureK,
        metrics.reactionAcousticsEnabled ? "on" : "OFF (instrumental)"
    );
    if (!metrics.ran) {
        std::printf("%-44s  NOT RUN (never reached the arming speed)\n",
            metrics.name.c_str());
        reportAudioContract(metrics);
        return;
    }
    std::printf("%-44s  rpm@lift %6.0f  armed %4d steps\n",
        metrics.name.c_str(), metrics.rpmAtLiftOff,
        metrics.overrunActiveSteps);
    std::printf("    wall  %7.1f degC at lift-off, %7.1f min over the overrun"
                "   wall-ignited peak %5.1f %%  mean %5.1f %%\n",
        metrics.wallTemperatureAtLiftOffC,
        metrics.wallTemperatureMinDuringOverrunC,
        metrics.peakWallIgnitedFraction * 100.0,
        metrics.meanWallIgnitedFraction * 100.0);
    std::printf("    induction Imax=%.3f local_delay=%.3f..%.3f ms\n",
        metrics.peakInductionProgress,
        metrics.minimumInductionDelayMs,
        metrics.maximumInductionDelayMs);
    std::printf("    heat  peak %8.3f kW  trough %8.4f kW  modulation %8.1fx"
                "  mean %8.4f kW  duty %5.1f %%\n",
        metrics.peakHeatKw, metrics.troughHeatKw, metrics.modulationDepth,
        metrics.meanHeatKw, metrics.dutyCycle * 100.0);
    // The event count only means something once the signal actually modulates.
    // Below about 2x the threshold sits inside the firing-rate ripple and the
    // counter crosses it repeatedly: a steady burn measured 1.3x modulation and
    // "60 events" over 2.5 s, which is the ripple, not bursts. Modulation depth
    // needs no threshold and is the reading to trust.
    std::printf("    burst events %3d%s  largest %8.3f kW  rise %6.1f ms"
                "  burned %8.3f / delivered %8.3f / metered %8.3f mg"
                " (%5.1f %% delivered burned)\n",
        metrics.eventCount,
        metrics.modulationDepth < 2.0 ? " (ripple, not bursts)" : "",
        metrics.largestEventPeakKw,
        metrics.largestEventRiseMs, metrics.burnedFuelMg,
        metrics.deliveredFuelMg,
        metrics.meteredFuelMg,
        100.0 * metrics.burnedFuelMg
            / std::max(1.0e-9, metrics.deliveredFuelMg));
    if (metrics.eventCount > 1)
        std::printf("    burst spacing min %6.1f ms  max %6.1f ms"
                    "  stddev %6.1f ms\n",
            metrics.minimumEventIntervalMs, metrics.maximumEventIntervalMs,
            metrics.eventIntervalStdDevMs);
    std::printf("    port  overrun peak %7.1f kPa  pre-lift mean %7.1f kPa\n",
        metrics.portPeakDuringOverrunKpa, metrics.portBaselineKpa);
    std::printf("    simulation step %llu x %.3f ms: mean=%5.1f %%"
                " p99=%5.1f %% max=%5.1f %% over-budget=%llu  %s\n",
        static_cast<unsigned long long>(
            metrics.reactionWindowSimulationTiming.blockCount),
        stepSeconds * 1'000.0,
        metrics.reactionWindowSimulationTiming.meanBudgetFraction * 100.0,
        metrics.reactionWindowSimulationTiming.p99BudgetFraction * 100.0,
        metrics.reactionWindowSimulationTiming.maximumBudgetFraction * 100.0,
        static_cast<unsigned long long>(
            metrics.reactionWindowSimulationTiming.overBudgetBlockCount),
        violatesSimulationBudget(metrics) ? "INVALID" : "valid");
    const auto expectsAfterfire = enginelab::afterfireRetainsFuel(
            calibration.strategy)
        && calibration.overrunFuelFraction > 0.0;
    std::printf("    physical afterfire expected=%s induction=%s"
                " heat_events=%d source_events=%llu  %s\n",
        expectsAfterfire ? "yes" : "no",
        metrics.peakInductionProgress >= 1.0 ? "yes" : "no",
        metrics.eventCount,
        static_cast<unsigned long long>(
            metrics.reactionAcoustics.eventCount),
        violatesPhysicalAfterfireContract(metrics) ? "INVALID" : "valid");
    if (metrics.audioMeasured) {
        if (metrics.preLiftReactionWindowMeasured)
            std::printf("    loaded pre-lift reaction events=%llu  %s\n",
                static_cast<unsigned long long>(
                    metrics.preLiftReactionAcoustics.eventCount),
                metrics.preLiftReactionAcoustics.eventCount == 0
                    ? "valid" : "INVALID");
        std::printf("    reaction source events=%llu energy=%8.3f J"
                    " max_power=%8.3f kW jump=%7.3f kPa axial=%5.3f\n",
            static_cast<unsigned long long>(
                metrics.reactionAcoustics.eventCount),
            metrics.reactionAcoustics.releasedEnergyJoules,
            metrics.reactionAcoustics.maximumPowerW * 0.001,
            metrics.reactionAcoustics.maximumCompactPressureJumpPa * 0.001,
            metrics.reactionAcoustics.energyWeightedAxial);
        std::printf("    audio render %llu x %d samples: mean=%5.1f %%"
                    " p99=%5.1f %% max=%5.1f %% over-budget=%llu  %s\n",
            static_cast<unsigned long long>(
                metrics.reactionWindowRenderTiming.blockCount),
            audioSamplesPerStep,
            metrics.reactionWindowRenderTiming.meanBudgetFraction * 100.0,
            metrics.reactionWindowRenderTiming.p99BudgetFraction * 100.0,
            metrics.reactionWindowRenderTiming.maximumBudgetFraction * 100.0,
            static_cast<unsigned long long>(
                metrics.reactionWindowRenderTiming.overBudgetBlockCount),
            metrics.reactionWindowRenderTiming.overBudgetBlockCount == 0
                ? "valid" : "INVALID");
        std::printf("    audio overrun peak %8.5f  p999 %8.5f  crest %6.2f"
                    "   (pre-lift p999 %8.5f crest %6.2f)%s\n",
            metrics.audioOverrunPeak, metrics.audioOverrunP999,
            metrics.audioOverrunCrest,
            metrics.audioBaselineP999, metrics.audioBaselineCrest,
            metrics.audioFinite ? "" : "  NON-FINITE");
        reportAudioContract(metrics);
    }
}

void printUsage() {
    std::puts(
        "EngineLabAfterfireHarness [options]\n"
        "  --engines NAME                 select one catalogue engine\n"
        "  --audio DIR                    render the production audio path\n"
        "  --ir FILE                      optional listening impulse response\n"
        "  --trace                        print the 240 Hz heat-release trace\n"
        "  --list                         list catalogue engines\n"
        "  --force-demo                   apply the historical hidden lab defaults\n"
        "  --fuel-fraction F              override retained fuel fraction\n"
        "  --ignition-k K                 override ignition threshold\n"
        "  --reaction-ms MS               override reaction time constant\n"
        "  --pulse-hz HZ                  override fuel-slug frequency\n"
        "  --pulse-duty FRACTION          override fuel-slug duty cycle\n"
        "  --pulse-timing-variation F     override deterministic timing variation\n"
        "  --flat-induction               same-binary schema-7 timer control\n"
        "  --warmup-seconds S             loaded exhaust warm-up duration\n"
        "  --overrun-seconds S            closed-throttle capture duration\n"
        "  --liftoff-rpm RPM              controlled lift-off speed\n"
        "  --no-reaction-acoustics        suppress only copied reaction events\n"
        "\nWithout calibration overrides, the selected engine is measured exactly as authored."
    );
}
} // namespace

int main(int argc, char** argv) {
    Options options;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        const auto next = [&]() -> std::string {
            return index + 1 < argc ? argv[++index] : std::string {};
        };
        if (argument == "--engines" || argument == "--filter")
            options.engineFilter = next();
        else if (argument == "--help" || argument == "-h") {
            printUsage();
            return 0;
        }
        else if (argument == "--trace") options.trace = true;
        else if (argument == "--list") options.list = true;
        else if (argument == "--audio") options.audioDirectory = next();
        else if (argument == "--ir") options.impulseResponsePath = next();
        else if (argument == "--force-demo") options.forceDemo = true;
        else if (argument == "--no-reaction-acoustics")
            options.reactionAcousticsEnabled = false;
        else if (argument == "--fuel-fraction")
            options.fuelFraction = std::stod(next());
        else if (argument == "--ignition-k")
            options.ignitionTemperatureK = std::stod(next());
        else if (argument == "--reaction-ms")
            options.reactionMilliseconds = std::stod(next());
        else if (argument == "--overrun-seconds")
            options.overrunSeconds = std::stod(next());
        else if (argument == "--warmup-seconds")
            options.warmupSeconds = std::stod(next());
        else if (argument == "--liftoff-rpm")
            options.liftOffRpm = std::stod(next());
        else if (argument == "--pulse-hz") options.pulseHz = std::stod(next());
        else if (argument == "--pulse-duty") options.pulseDuty = std::stod(next());
        else if (argument == "--pulse-timing-variation")
            options.pulseTimingVariation = std::stod(next());
        else if (argument == "--flat-induction")
            options.flatInductionControl = true;
        else {
            std::fprintf(stderr, "unknown argument: %s\n", argument.c_str());
            return 2;
        }
    }

    const auto catalog = enginelab::loadEngineCatalog(
        std::filesystem::path(ENGINELAB_CATALOG_ROOT));
    if (!catalog.errors.empty()) {
        std::fprintf(stderr, "could not load the engine catalogue: %s\n",
            catalog.errors.front().c_str());
        return 1;
    }
    if (options.list) {
        for (const auto& entry : catalog.entries)
            std::printf("%s\n", entry.config.name.c_str());
        return 0;
    }

    std::vector<enginelab::EngineConfig> selected;
    if (options.engineFilter.empty()) {
        selected.reserve(catalog.entries.size());
        for (const auto& entry : catalog.entries) {
            auto config = entry.config;
            enginelab::normaliseEngineConfig(config);
            selected.push_back(std::move(config));
        }
    } else {
        const auto resolved = enginelab::selectSingleEngineCatalogEntry(
            catalog.entries, options.engineFilter);
        if (!resolved) {
            std::fprintf(stderr, "%s engine selector \"%s\"\n",
                resolved.status == enginelab::EngineCatalogSelectionStatus::ambiguous
                    ? "ambiguous" : "unmatched",
                options.engineFilter.c_str());
            for (const auto* match : resolved.matches)
                std::fprintf(stderr, "  %s  %s\n",
                    match->config.audioVoicingKey.c_str(),
                    match->config.name.c_str());
            return 1;
        }
        auto config = resolved.entry->config;
        enginelab::normaliseEngineConfig(config);
        selected.push_back(std::move(config));
    }

    auto invalidRealtimeMeasurements = 0;
    for (const auto& engine : selected) {
        const auto metrics = measureAfterfire(engine, options);
        report(metrics);
        if (violatesPhysicalAfterfireContract(metrics)
            || violatesSimulationBudget(metrics)
            || violatesAudioContract(metrics)) {
            std::fprintf(stderr,
                "INVALID: %s: physical or real-time afterfire contract"
                " was violated\n",
                metrics.name.c_str());
            ++invalidRealtimeMeasurements;
        }
    }
    return invalidRealtimeMeasurements == 0 ? 0 : 1;
}
