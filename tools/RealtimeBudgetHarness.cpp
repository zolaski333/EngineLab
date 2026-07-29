/**
 * Realtime budget instrument.
 *
 * Every other harness in this tree measures the simulator offline, where a step
 * may take as long as it likes. That cannot see the defect this tool exists to
 * measure: `EngineRuntime::run` advances a FIXED `1/240 s` of simulated time per
 * iteration and then sleeps until a wall-clock deadline. When an iteration costs
 * more wall time than it advances, simulated time falls behind wall time and
 * never catches up -- the loop has no accumulator, and past four steps of
 * lateness it resets its own deadline to `now`, discarding the debt.
 *
 * The observable consequence is not a dropped frame. It is that the whole
 * simulation runs in slow motion: controls respond late, and the cylinder
 * pressure telemetry -- which is the ONLY excitation of the exhaust acoustic
 * chain -- is produced more slowly than the audio thread consumes it.
 *
 * So the number that matters is neither CPU time per step nor the overrun count
 * (both already reported elsewhere); it is the ratio
 *
 *     realtime factor = simulated seconds advanced / wall seconds elapsed
 *
 * which is 1.0 on a healthy engine and falls below it exactly when the physics
 * thread is saturated. It is measured here on the real `EngineRuntime` thread,
 * at real thread priority, because that is the thing the user hears and feels.
 */

#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/audio/ImpulseResponseLoader.hpp>
#include <enginelab/audio/RealtimeConvolutionBank.hpp>
#include <enginelab/audio/RealtimeEngineAudio.hpp>
#include <enginelab/diagnostics/EngineDiagnostics.hpp>
#include <enginelab/foundation/EngineTypes.hpp>
#include <enginelab/runtime/EngineRuntime.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;

constexpr std::size_t audioUtilisationHistogramBins = 201;

[[nodiscard]] std::string lowercase(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

void sleepSeconds(double seconds) {
    std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
}

struct AudioProbeSnapshot final {
    std::uint64_t callbacks {};
    std::uint64_t renderNanoseconds {};
    std::uint64_t deadlineMisses {};
    std::uint64_t renderOverBudget {};
    std::uint64_t nonFiniteSamples {};
    std::array<std::uint64_t, audioUtilisationHistogramBins> utilisationHistogram {};
    std::uint64_t lateEvents {};
    std::uint64_t stolenVoices {};
    std::uint64_t droppedPendingEvents {};
    std::uint64_t legacyPathSamples {};
    std::uint64_t invalidBoundarySamples {};
    std::uint64_t levelLimitedSamples {};
};

struct AudioProbeDelta final {
    std::uint64_t callbacks {};
    double utilisationMeanPercent {};
    std::uint64_t deadlineMisses {};
    std::uint64_t renderOverBudget {};
    std::uint64_t nonFiniteSamples {};
    double utilisationP99Percent {};
    std::uint64_t lateEvents {};
    std::uint64_t stolenVoices {};
    std::uint64_t droppedPendingEvents {};
    std::uint64_t legacyPathSamples {};
    std::uint64_t invalidBoundarySamples {};
    std::uint64_t levelLimitedSamples {};
};

[[nodiscard]] std::uint64_t counterDelta(
    std::uint64_t end, std::uint64_t begin) noexcept {
    return end >= begin ? end - begin : 0;
}

[[nodiscard]] AudioProbeDelta audioProbeDelta(
    const AudioProbeSnapshot& begin, const AudioProbeSnapshot& end,
    double sampleRate, int blockSize) {
    AudioProbeDelta delta;
    delta.callbacks = counterDelta(end.callbacks, begin.callbacks);
    const auto renderNanoseconds = counterDelta(
        end.renderNanoseconds, begin.renderNanoseconds);
    if (delta.callbacks > 0 && sampleRate > 0.0 && blockSize > 0) {
        const auto meanRenderSeconds =
            static_cast<double>(renderNanoseconds) * 1.0e-9
            / static_cast<double>(delta.callbacks);
        delta.utilisationMeanPercent = meanRenderSeconds * sampleRate
            / static_cast<double>(blockSize) * 100.0;
    }
    delta.deadlineMisses =
        counterDelta(end.deadlineMisses, begin.deadlineMisses);
    delta.renderOverBudget =
        counterDelta(end.renderOverBudget, begin.renderOverBudget);
    delta.nonFiniteSamples =
        counterDelta(end.nonFiniteSamples, begin.nonFiniteSamples);
    delta.lateEvents = counterDelta(end.lateEvents, begin.lateEvents);
    delta.stolenVoices = counterDelta(end.stolenVoices, begin.stolenVoices);
    delta.droppedPendingEvents =
        counterDelta(end.droppedPendingEvents, begin.droppedPendingEvents);
    delta.legacyPathSamples =
        counterDelta(end.legacyPathSamples, begin.legacyPathSamples);
    delta.invalidBoundarySamples =
        counterDelta(end.invalidBoundarySamples, begin.invalidBoundarySamples);
    delta.levelLimitedSamples =
        counterDelta(end.levelLimitedSamples, begin.levelLimitedSamples);

    const auto rank = delta.callbacks > 0
        ? std::max<std::uint64_t>(
            1, static_cast<std::uint64_t>(
                std::ceil(static_cast<double>(delta.callbacks) * 0.99)))
        : 0;
    auto accumulated = std::uint64_t { 0 };
    for (std::size_t bin = 0;
         bin < audioUtilisationHistogramBins; ++bin) {
        accumulated += counterDelta(
            end.utilisationHistogram[bin],
            begin.utilisationHistogram[bin]);
        if (rank > 0 && accumulated >= rank) {
            delta.utilisationP99Percent = static_cast<double>(bin);
            break;
        }
    }
    return delta;
}

/**
 * Real-wall-clock consumer for the same renderer and SPSC queues used by the
 * application. This is intentionally not an audio-device wrapper: a hardware
 * callback would make the catalogue gate depend on whichever Windows device is
 * selected. The fixed-period thread gives every machine the same reproducible
 * consumer while still competing with EngineRuntime for real CPU time.
 */
class RealtimeAudioProbe final {
public:
    RealtimeAudioProbe(enginelab::EngineRuntime& runtime,
                       const std::filesystem::path& catalogRoot,
                       double sampleRate, int blockSize,
                       bool accelerated,
                       bool outletJetNoiseEnabled)
        : runtime_(runtime),
          sampleRate_(sampleRate),
          blockSize_(blockSize),
          accelerated_(accelerated),
          renderer_(runtime.audioEvents(), runtime.audioState(),
                    &runtime.cylinderPressureSamples(), &runtime.exhaustGraph(),
                    &runtime.engineConfig()),
          block_(2, blockSize) {
        if (!(sampleRate_ > 0.0) || blockSize_ <= 0)
            throw std::invalid_argument("invalid audio probe format");
        for (std::size_t pathIndex = 0;
             pathIndex < runtime.engineConfig().exhaustPaths.size();
             ++pathIndex) {
            const auto& configured =
                runtime.engineConfig().exhaustPaths[pathIndex].impulseResponsePath;
            if (configured.empty()) continue;
            if (pathIndex >= enginelab::RealtimeConvolutionBank::maximumPaths)
                throw std::runtime_error(
                    "audio probe exceeds the eight-path convolution limit");
            auto resolved = std::filesystem::path(configured);
            if (!resolved.is_absolute()) resolved = catalogRoot / resolved;
            const auto pathText = resolved.string();
            auto impulse = enginelab::loadImpulseResponseFile(
                juce::File(juce::String::fromUTF8(pathText.c_str())));
            if (!impulse.ok())
                throw std::runtime_error(
                    "audio probe could not load IR: " + resolved.string());
            renderer_.setImpulseResponse(
                std::move(impulse.samples), impulse.sampleRateHz, pathIndex);
        }
        renderer_.setOutletJetNoiseEnabled(outletJetNoiseEnabled);
        renderer_.prepare(sampleRate_, blockSize_);
    }

    ~RealtimeAudioProbe() {
        stop();
    }

    RealtimeAudioProbe(const RealtimeAudioProbe&) = delete;
    RealtimeAudioProbe& operator=(const RealtimeAudioProbe&) = delete;

    void start() {
        if (thread_.joinable()) return;
        thread_ = std::jthread(
            [this](std::stop_token stopToken) { run(stopToken); });
    }

    void stop() noexcept {
        if (thread_.joinable()) {
            thread_.request_stop();
            thread_.join();
        }
        renderer_.release();
    }

    [[nodiscard]] AudioProbeSnapshot snapshot() const noexcept {
        AudioProbeSnapshot result;
        result.callbacks = callbacks_.load(std::memory_order_relaxed);
        result.renderNanoseconds =
            renderNanoseconds_.load(std::memory_order_relaxed);
        result.deadlineMisses =
            deadlineMisses_.load(std::memory_order_relaxed);
        result.renderOverBudget =
            renderOverBudget_.load(std::memory_order_relaxed);
        result.nonFiniteSamples =
            nonFiniteSamples_.load(std::memory_order_relaxed);
        for (std::size_t bin = 0;
             bin < audioUtilisationHistogramBins; ++bin) {
            result.utilisationHistogram[bin] =
                utilisationHistogram_[bin].load(std::memory_order_relaxed);
        }
        result.lateEvents = renderer_.lateEventCount();
        result.stolenVoices = renderer_.stolenVoiceCount();
        result.droppedPendingEvents = renderer_.droppedPendingEventCount();
        result.legacyPathSamples = renderer_.legacyPathSampleCount();
        result.invalidBoundarySamples = renderer_.invalidBoundarySampleCount();
        result.levelLimitedSamples = renderer_.levelLimitedSampleCount();
        return result;
    }

    [[nodiscard]] bool physicalExhaustActive() const noexcept {
        return renderer_.physicalExhaustActive();
    }

    [[nodiscard]] bool compiledExhaustTopologyActive() const noexcept {
        return renderer_.compiledExhaustTopologyActive();
    }

    [[nodiscard]] float minimumLevelGain() const noexcept {
        return renderer_.minObservedLevelGain();
    }

    [[nodiscard]] float maximumPreLimiterMagnitude() const noexcept {
        return renderer_.maxPreLimiterMagnitude();
    }

private:
    void run(std::stop_token stopToken) noexcept {
        const auto period = std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(
                static_cast<double>(blockSize_) / sampleRate_));
        const auto blockSeconds =
            static_cast<double>(blockSize_) / sampleRate_;
        auto renderedSimulationSeconds = 0.0;
        auto scheduledStart = Clock::now();
        while (!stopToken.stop_requested()) {
            if (accelerated_) {
                // EngineRuntime deliberately publishes wall-clock timestamps
                // even when its instrumentation throttle is disabled. In
                // free-run mode, the snapshot's simulation clock is therefore
                // the only pace that advances with the accelerated pressure
                // windows. Polling it here keeps this consumer aligned with
                // simulated audio time instead of letting the queue overflow
                // while wall time advances more slowly.
                const auto producerSeconds =
                    runtime_.snapshot().simulationTimeSeconds;
                if (producerSeconds + 1.0e-9
                    < renderedSimulationSeconds + blockSeconds) {
                    std::this_thread::yield();
                    continue;
                }
            } else {
                std::this_thread::sleep_until(scheduledStart);
            }
            if (stopToken.stop_requested()) break;
            const auto renderStart = Clock::now();
            const auto deadline = scheduledStart + period;
            block_.clear();
            renderer_.render(block_, 0, blockSize_);
            const auto renderEnd = Clock::now();
            const auto elapsed =
                std::chrono::duration<double>(renderEnd - renderStart).count();
            renderNanoseconds_.fetch_add(
                static_cast<std::uint64_t>(
                    std::max(0.0, elapsed) * 1.0e9),
                std::memory_order_relaxed);
            const auto utilisation = elapsed * sampleRate_
                / static_cast<double>(blockSize_);
            const auto bin = std::min<std::size_t>(
                audioUtilisationHistogramBins - 1,
                static_cast<std::size_t>(
                    std::ceil(std::max(0.0, utilisation) * 100.0)));
            utilisationHistogram_[bin].fetch_add(
                1, std::memory_order_relaxed);
            callbacks_.fetch_add(1, std::memory_order_relaxed);
            if ((!accelerated_ && renderEnd > deadline)
                || (accelerated_ && elapsed > blockSeconds))
                deadlineMisses_.fetch_add(1, std::memory_order_relaxed);
            if (elapsed > blockSeconds)
                renderOverBudget_.fetch_add(1, std::memory_order_relaxed);
            for (int channel = 0; channel < block_.getNumChannels(); ++channel) {
                const auto* samples = block_.getReadPointer(channel);
                for (int sample = 0; sample < blockSize_; ++sample) {
                    if (!std::isfinite(samples[sample]))
                        nonFiniteSamples_.fetch_add(
                            1, std::memory_order_relaxed);
                }
            }
            if (accelerated_) {
                renderedSimulationSeconds += blockSeconds;
            } else {
                scheduledStart += period;
                if (renderEnd > scheduledStart + period * 4)
                    scheduledStart = renderEnd;
            }
        }
    }

    enginelab::EngineRuntime& runtime_;
    double sampleRate_ {};
    int blockSize_ {};
    bool accelerated_ {};
    enginelab::RealtimeEngineAudio renderer_;
    juce::AudioBuffer<float> block_;
    std::jthread thread_;
    std::atomic<std::uint64_t> callbacks_ {};
    std::atomic<std::uint64_t> renderNanoseconds_ {};
    std::atomic<std::uint64_t> deadlineMisses_ {};
    std::atomic<std::uint64_t> renderOverBudget_ {};
    std::atomic<std::uint64_t> nonFiniteSamples_ {};
    std::array<std::atomic<std::uint64_t>,
               audioUtilisationHistogramBins> utilisationHistogram_ {};
};

struct Measurement final {
    bool valid { false };
    std::string invalidReason;
    double targetRpm {};
    double realtimeFactor {};
    double wallSeconds {};
    double simulatedSeconds {};
    double meanRpm {};
    double minimumRpm {};
    double maximumRpm {};
    std::size_t intakeWorkers {};
    std::uint64_t overruns {};
    std::uint64_t iterations {};
    double maximumLatenessMs {};
    // What the GUI diagnostics panel would show at this operating point. Both
    // exhaust pressures are reported side by side because the panel's
    // back-pressure warning used to read the peak envelope and compare it to a
    // mean-sized allowance, which made it a permanent false alarm.
    double exhaustPeakKpa {};
    double exhaustMeanKpa {};
    bool backPressureWarning {};
    bool tractionLimited {};
    bool audioEnabled {};
    bool physicalExhaustActive {};
    bool compiledExhaustTopologyActive {};
    AudioProbeDelta audio {};
    std::uint64_t droppedEvents {};
    std::uint64_t droppedPressureSamples {};
    float minimumLevelGain { 1.0F };
    float maximumPreLimiterMagnitude {};
};
// Deliberately NOT reported: the dropped cylinder-pressure count. This harness
// runs no audio thread, so nothing drains the telemetry queue and it overflows
// on every engine regardless of load -- the number would look alarming and mean
// nothing. Read that one from the AudioRender harness, which has a consumer.

/**
 * Runs one engine on the real runtime thread and reports how much simulated
 * time it managed to produce per second of wall time.
 *
 * The engine is held at a commanded speed by the dyno absorber rather than left
 * free, because the cost of a step scales with speed (more substeps per second
 * of simulated time) and comparing engines at whatever speed each happens to
 * settle at would confound engine cost with operating point.
 */
[[nodiscard]] Measurement measureEngine(const enginelab::EngineConfig& config,
                                        double holdRpm,
                                        double warmupSeconds,
                                        double measureSeconds,
                                        bool freeRun,
                                        std::optional<std::size_t> intakeWorkers,
                                        std::optional<std::size_t> intakeMaximumCells,
                                        std::optional<std::size_t> intakeStaircaseRounds,
                                        std::optional<double> intakeCouplingSeconds,
                                        std::optional<double> intakeTargetCellLengthM,
                                        std::optional<double> exhaustCouplingSeconds,
                                        std::optional<bool> intakeFirstOrderTimeIntegration,
                                        std::optional<double> intakeWallHeatUpdateSeconds,
                                        bool useWellMixedExhaustJunctions,
                                        bool withAudio,
                                        bool outletJetNoiseEnabled,
                                        const std::filesystem::path& catalogRoot,
                                        double audioSampleRate,
                                        int audioBlockSize) {
    enginelab::EngineSimulatorOptions simulatorOptions;
    simulatorOptions.intakeWorkerCount = intakeWorkers;
    simulatorOptions.intakeMaximumCellCount = intakeMaximumCells;
    simulatorOptions.intakeStaircaseRounds = intakeStaircaseRounds;
    simulatorOptions.intakeCouplingIntervalSeconds = intakeCouplingSeconds;
    simulatorOptions.intakeTargetCellLengthM = intakeTargetCellLengthM;
    simulatorOptions.maximumLowSpeedExhaustCouplingSeconds =
        exhaustCouplingSeconds;
    simulatorOptions.intakeFirstOrderTimeIntegration =
        intakeFirstOrderTimeIntegration;
    simulatorOptions.intakeWallHeatUpdateIntervalSeconds =
        intakeWallHeatUpdateSeconds;
    simulatorOptions.evolveExhaustJunctionAxialMomentum =
        !useWellMixedExhaustJunctions;
    auto runtime = std::make_unique<enginelab::EngineRuntime>(
        config, nullptr, simulatorOptions);
    std::unique_ptr<RealtimeAudioProbe> audioProbe;
    Measurement result;
    result.intakeWorkers = runtime->intakeWorkerCount();
    result.audioEnabled = withAudio;
    if (withAudio) {
        try {
            audioProbe = std::make_unique<RealtimeAudioProbe>(
                *runtime, catalogRoot, audioSampleRate, audioBlockSize,
                freeRun, outletJetNoiseEnabled);
            audioProbe->start();
        } catch (const std::exception& error) {
            result.invalidReason = error.what();
            return result;
        }
    }
    // Throttled, the loop sleeps to its wall deadline, so the factor saturates
    // at 1.0 and an engine at 3x reads the same as one exactly breaking even.
    // Free-running, the same ratio is the capacity headroom.
    runtime->setRealtimeThrottleEnabled(!freeRun);
    runtime->setDynoMaximumDurationSeconds(120.0);
    runtime->setIgnitionEnabled(true);
    runtime->setStarterEngaged(true);
    runtime->setDynoHoldEnabled(true);
    // This must be an absolute setter. Before it existed the harness subtracted
    // the not-yet-published snapshot value (zero) from the requested speed and
    // added that delta to the runtime's internal 2,500 rpm default. Most points
    // therefore hit the limiter while the heading claimed a common setpoint.
    runtime->setDynoHoldRpm(holdRpm);
    result.targetRpm = runtime->dynoHoldRpm();
    runtime->start();
    runtime->startDyno();

    // Do not begin a sample merely because a wall-clock warm-up expired. The
    // catalogue spans twins to a V12, and in free-run mode each engine advances
    // simulation time at a different rate. Use the same complete-window
    // stability contract as the validated dyno reference gate: mean within 2%,
    // excursion <= 4%, drift <= 1%, and standard deviation <= 1.5%. The old
    // gate reset a timer whenever one polled value crossed 2%; K20, LS3,
    // Merlin and the flat-six could sit on the requested mean indefinitely yet
    // never accumulate 1.5 uninterrupted seconds because the poll sampled
    // normal firing ripple. A statistical window rejects a sweep but does not
    // confuse cycle ripple with loss of control.
    const auto holdWallDeadline = Clock::now() + std::chrono::seconds(90);
    auto holdState = runtime->snapshot();
    const auto holdStartSimulationTime = holdState.simulationTimeSeconds;
    auto previousSimulationTime = holdState.simulationTimeSeconds;
    auto filteredRpm = holdState.rpm;
    struct HoldSample final {
        double timeSeconds {};
        double filteredRpm {};
    };
    std::deque<HoldSample> holdWindow;
    auto holdReached = false;
    while (Clock::now() < holdWallDeadline) {
        holdState = runtime->snapshot();
        const auto dt = std::max(
            0.0, holdState.simulationTimeSeconds - previousSimulationTime);
        previousSimulationTime = holdState.simulationTimeSeconds;
        if (dt > 0.0) {
            filteredRpm += (holdState.rpm - filteredRpm)
                * (1.0 - std::exp(-dt * 3.0));
            holdWindow.push_back(
                { holdState.simulationTimeSeconds, filteredRpm });
            while (!holdWindow.empty()
                && holdState.simulationTimeSeconds
                    - holdWindow.front().timeSeconds > 1.5) {
                holdWindow.pop_front();
            }
        }
        const auto toleranceRpm = std::max(60.0, result.targetRpm * 0.02);
        if (holdWindow.size() >= 4
            && holdWindow.back().timeSeconds
                - holdWindow.front().timeSeconds >= 1.45) {
            auto sum = 0.0;
            auto squareSum = 0.0;
            auto minimum = std::numeric_limits<double>::infinity();
            auto maximum = 0.0;
            auto firstHalfSum = 0.0;
            auto secondHalfSum = 0.0;
            auto firstHalfCount = std::size_t { 0 };
            auto secondHalfCount = std::size_t { 0 };
            const auto midpoint = holdWindow.front().timeSeconds
                + 0.5 * (holdWindow.back().timeSeconds
                    - holdWindow.front().timeSeconds);
            for (const auto& sample : holdWindow) {
                sum += sample.filteredRpm;
                squareSum += sample.filteredRpm * sample.filteredRpm;
                minimum = std::min(minimum, sample.filteredRpm);
                maximum = std::max(maximum, sample.filteredRpm);
                if (sample.timeSeconds < midpoint) {
                    firstHalfSum += sample.filteredRpm;
                    ++firstHalfCount;
                } else {
                    secondHalfSum += sample.filteredRpm;
                    ++secondHalfCount;
                }
            }
            const auto count = static_cast<double>(holdWindow.size());
            const auto mean = sum / count;
            const auto standardDeviation = std::sqrt(std::max(
                0.0, squareSum / count - mean * mean));
            const auto drift =
                secondHalfSum / static_cast<double>(
                    std::max<std::size_t>(1, secondHalfCount))
                - firstHalfSum / static_cast<double>(
                    std::max<std::size_t>(1, firstHalfCount));
            if (std::abs(mean - result.targetRpm) <= toleranceRpm
                && maximum - minimum <= result.targetRpm * 0.04
                && std::abs(drift) <= result.targetRpm * 0.01
                && standardDeviation <= result.targetRpm * 0.015) {
                holdReached = true;
                break;
            }
        }
        if (!runtime->dynoRunning()
            && holdState.simulationTimeSeconds > holdStartSimulationTime + 0.25) {
            result.invalidReason = "dyno stopped before hold";
            break;
        }
        if (holdState.simulationTimeSeconds - holdStartSimulationTime >= 60.0) {
            result.invalidReason = "hold not stable within 60 sim s";
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    if (!holdReached) {
        if (result.invalidReason.empty()) result.invalidReason = "hold wall timeout";
        result.meanRpm = holdState.rpm;
        result.minimumRpm = holdState.rpm;
        result.maximumRpm = holdState.rpm;
        runtime->stop();
        return result;
    }

    // Warm up by SIMULATED time, not wall time. This gives every engine the
    // same number of thermodynamic seconds after its speed has settled.
    const auto warmupStart = runtime->snapshot().simulationTimeSeconds;
    const auto warmupWallDeadline = Clock::now()
        + std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(std::max(30.0, warmupSeconds * 20.0)));
    while (runtime->snapshot().simulationTimeSeconds - warmupStart < warmupSeconds) {
        if (!runtime->dynoRunning()) {
            result.invalidReason = "dyno stopped during warmup";
            result.meanRpm = runtime->snapshot().rpm;
            result.minimumRpm = result.meanRpm;
            result.maximumRpm = result.meanRpm;
            runtime->stop();
            return result;
        }
        if (Clock::now() >= warmupWallDeadline) {
            result.invalidReason = "warmup wall timeout";
            result.meanRpm = runtime->snapshot().rpm;
            result.minimumRpm = result.meanRpm;
            result.maximumRpm = result.meanRpm;
            runtime->stop();
            return result;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    // Both clocks are read as close together as possible at each end of the
    // window: the quantity is a ratio of two intervals, so any skew between the
    // reads is a direct error on the result.
    const auto wallStart = Clock::now();
    const auto simulatedStart = runtime->snapshot().simulationTimeSeconds;
    const auto overrunsStart = runtime->timingOverrunCount();
    const auto droppedEventsStart = runtime->droppedEventCount();
    const auto droppedPressureStart = runtime->droppedPressureSampleCount();
    const auto audioStart = audioProbe
        ? audioProbe->snapshot() : AudioProbeSnapshot {};
    const auto measurementDeadline = wallStart
        + std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(measureSeconds));
    auto rpmSum = 0.0;
    auto rpmSamples = std::uint64_t { 0 };
    result.minimumRpm = std::numeric_limits<double>::infinity();
    result.maximumRpm = 0.0;
    while (Clock::now() < measurementDeadline) {
        const auto state = runtime->snapshot();
        rpmSum += state.rpm;
        ++rpmSamples;
        result.minimumRpm = std::min(result.minimumRpm, state.rpm);
        result.maximumRpm = std::max(result.maximumRpm, state.rpm);
        if (!runtime->dynoRunning()) {
            result.invalidReason = "dyno stopped during measurement";
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const auto simulatedEnd = runtime->snapshot().simulationTimeSeconds;
    const auto wallEnd = Clock::now();
    const auto overrunsEnd = runtime->timingOverrunCount();
    const auto droppedEventsEnd = runtime->droppedEventCount();
    const auto droppedPressureEnd = runtime->droppedPressureSampleCount();
    const auto audioEnd = audioProbe
        ? audioProbe->snapshot() : AudioProbeSnapshot {};

    result.wallSeconds = std::chrono::duration<double>(wallEnd - wallStart).count();
    result.simulatedSeconds = simulatedEnd - simulatedStart;
    result.realtimeFactor = result.wallSeconds > 0.0
        ? result.simulatedSeconds / result.wallSeconds : 0.0;
    result.meanRpm = rpmSamples > 0 ? rpmSum / static_cast<double>(rpmSamples)
                                   : runtime->snapshot().rpm;
    if (!std::isfinite(result.minimumRpm)) result.minimumRpm = result.meanRpm;
    result.overruns = overrunsEnd - overrunsStart;
    result.droppedEvents =
        counterDelta(droppedEventsEnd, droppedEventsStart);
    result.droppedPressureSamples =
        counterDelta(droppedPressureEnd, droppedPressureStart);
    if (audioProbe) {
        result.audio = audioProbeDelta(
            audioStart, audioEnd, audioSampleRate, audioBlockSize);
        result.physicalExhaustActive =
            audioProbe->physicalExhaustActive();
        result.compiledExhaustTopologyActive =
            audioProbe->compiledExhaustTopologyActive();
        result.minimumLevelGain = audioProbe->minimumLevelGain();
        result.maximumPreLimiterMagnitude =
            audioProbe->maximumPreLimiterMagnitude();
    }
    result.iterations = static_cast<std::uint64_t>(result.simulatedSeconds * 240.0);
    result.maximumLatenessMs = runtime->maximumTimingLatenessSeconds() * 1.0e3;
    const auto finalState = runtime->snapshot();
    result.exhaustPeakKpa = finalState.exhaustPressureKpa;
    result.exhaustMeanKpa = finalState.exhaustBackPressureKpa;
    result.tractionLimited = finalState.tractionLimited;
    for (const auto& diagnostic : enginelab::EngineDiagnostics {}.evaluate(config, finalState))
        if (diagnostic.code == "exhaust.back_pressure") result.backPressureWarning = true;
    const auto meanToleranceRpm = std::max(60.0, result.targetRpm * 0.02);
    if (result.invalidReason.empty()
        && std::abs(result.meanRpm - result.targetRpm) > meanToleranceRpm) {
        std::ostringstream reason;
        reason << "mean rpm outside 2% (" << std::fixed << std::setprecision(0)
               << result.meanRpm << " vs " << result.targetRpm << ')';
        result.invalidReason = reason.str();
    }
    if (result.invalidReason.empty() && withAudio
        && (!result.physicalExhaustActive
            || !result.compiledExhaustTopologyActive)) {
        result.invalidReason = "physical audio topology inactive";
    }
    if (result.invalidReason.empty() && withAudio
        && (result.droppedEvents > 0
            || result.droppedPressureSamples > 0
            || result.audio.nonFiniteSamples > 0
            || result.audio.renderOverBudget > 0
            || result.audio.legacyPathSamples > 0
            || result.audio.invalidBoundarySamples > 0
            || result.audio.levelLimitedSamples > 0)) {
        result.invalidReason = "audio realtime contract violation";
    }
    result.valid = result.invalidReason.empty();
    runtime->stop();
    return result;
}
} // namespace

int main(int argc, char** argv) {
    std::filesystem::path catalogRoot = std::filesystem::current_path();
    std::string filter;
    double holdRpm = 5'000.0;
    double warmupSeconds = 3.0;
    double measureSeconds = 5.0;
    // A realtime loop that produces less simulated time than wall time is
    // running the whole simulation in slow motion. 0.97 leaves room for
    // scheduler noise on a loaded desktop without admitting a real deficit.
    double failBelow = 0.0;
    bool freeRun = false;
    std::optional<double> relativeRpm;
    std::optional<std::size_t> intakeWorkers;
    std::optional<std::size_t> intakeMaximumCells;
    std::optional<std::size_t> intakeStaircaseRounds;
    std::optional<double> intakeCouplingSeconds;
    std::optional<double> intakeTargetCellLengthM;
    std::optional<double> exhaustCouplingSeconds;
    std::optional<bool> intakeFirstOrderTimeIntegration;
    std::optional<double> intakeWallHeatUpdateSeconds;
    bool useWellMixedExhaustJunctions = false;
    bool withAudio = false;
    bool outletJetNoiseEnabled = true;
    double audioSampleRate = 48'000.0;
    int audioBlockSize = 256;

    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--catalog-root" && index + 1 < argc) catalogRoot = argv[++index];
        else if (argument == "--filter" && index + 1 < argc) filter = lowercase(argv[++index]);
        else if (argument == "--rpm" && index + 1 < argc) holdRpm = std::stod(argv[++index]);
        else if (argument == "--relative-rpm" && index + 1 < argc)
            relativeRpm = std::stod(argv[++index]);
        else if (argument == "--intake-workers" && index + 1 < argc)
            intakeWorkers = static_cast<std::size_t>(std::stoull(argv[++index]));
        else if (argument == "--intake-max-cells" && index + 1 < argc)
            intakeMaximumCells = static_cast<std::size_t>(std::stoull(argv[++index]));
        else if (argument == "--intake-staircase-rounds" && index + 1 < argc)
            intakeStaircaseRounds = static_cast<std::size_t>(std::stoull(argv[++index]));
        else if (argument == "--intake-coupling-us" && index + 1 < argc)
            intakeCouplingSeconds = std::stod(argv[++index]) * 1.0e-6;
        else if (argument == "--intake-cell-mm" && index + 1 < argc)
            intakeTargetCellLengthM = std::stod(argv[++index]) * 1.0e-3;
        else if (argument == "--exhaust-coupling-us" && index + 1 < argc)
            exhaustCouplingSeconds = std::stod(argv[++index]) * 1.0e-6;
        else if (argument == "--intake-euler")
            intakeFirstOrderTimeIntegration = true;
        else if (argument == "--intake-rk2")
            intakeFirstOrderTimeIntegration = false;
        else if (argument == "--intake-wall-us" && index + 1 < argc)
            intakeWallHeatUpdateSeconds = std::stod(argv[++index]) * 1.0e-6;
        else if (argument == "--warmup" && index + 1 < argc) warmupSeconds = std::stod(argv[++index]);
        else if (argument == "--seconds" && index + 1 < argc) measureSeconds = std::stod(argv[++index]);
        else if (argument == "--enforce" && index + 1 < argc) failBelow = std::stod(argv[++index]);
        else if (argument == "--free-run") freeRun = true;
        else if (argument == "--with-audio") withAudio = true;
        else if (argument == "--disable-exhaust-jet-noise")
            outletJetNoiseEnabled = false;
        else if (argument == "--audio-rate" && index + 1 < argc)
            audioSampleRate = std::stod(argv[++index]);
        else if (argument == "--audio-block" && index + 1 < argc)
            audioBlockSize = std::stoi(argv[++index]);
        else if (argument == "--well-mixed-junctions")
            useWellMixedExhaustJunctions = true;
        else if (argument == "--help") {
            std::cout << "usage: EngineLabRealtimeBudgetHarness [--catalog-root DIR] "
                         "[--filter NAME] [--rpm N] [--warmup S] [--seconds S] "
                         "[--relative-rpm FRACTION] [--intake-workers N] "
                         "[--intake-max-cells N] [--intake-staircase-rounds N] "
                         "[--intake-coupling-us N] "
                         "[--intake-cell-mm N] "
                         "[--exhaust-coupling-us N] "
                         "[--intake-euler|--intake-rk2] "
                         "[--intake-wall-us N] "
                         "[--well-mixed-junctions] "
                         "[--enforce FACTOR] [--free-run] "
                         "[--with-audio] [--disable-exhaust-jet-noise] "
                         "[--audio-rate HZ] [--audio-block N]\n"
                         "  --free-run  remove the loop's wall-clock sleep, so the factor\n"
                         "              reads capacity instead of saturating at 1.0.\n"
                         "  --with-audio  run the production renderer on a fixed-period\n"
                         "                consumer thread and enforce its realtime contract.\n"
                         "  --disable-exhaust-jet-noise  same-binary null control for\n"
                         "                               outlet-noise CPU measurements.\n"
                         "  --relative-rpm  hold each engine at this fraction of redline,\n"
                         "                  capped at 95% to stay below the limiter.\n"
                         "  --intake-workers  override background intake workers; zero is\n"
                         "                    the serial null control.\n"
                         "  --well-mixed-junctions  select the legacy zero-momentum exhaust\n"
                         "                          collector for same-machine A/B evidence.\n";
            return 0;
        }
    }
    if (relativeRpm.has_value()
        && (!std::isfinite(*relativeRpm) || *relativeRpm <= 0.0 || *relativeRpm > 1.0)) {
        std::cerr << "FAIL: --relative-rpm must be in (0, 1]\n";
        return 2;
    }
    if (!std::isfinite(audioSampleRate) || audioSampleRate < 8'000.0
        || audioSampleRate > 384'000.0 || audioBlockSize < 16
        || audioBlockSize > 8'192) {
        std::cerr << "FAIL: invalid audio format\n";
        return 2;
    }

    const auto catalog = enginelab::loadEngineCatalog(catalogRoot);
    if (catalog.entries.empty()) {
        std::cerr << "FAIL: no engines found under " << catalogRoot << '\n';
        return 1;
    }

    std::cout << "Realtime budget: simulated seconds produced per wall second by the\n"
                 "240 Hz EngineRuntime thread, "
              << (relativeRpm.has_value()
                  ? "each engine held at " + std::to_string(*relativeRpm * 100.0)
                      + "% of redline.\n"
                  : "held at an absolute requested speed of "
                      + std::to_string(static_cast<int>(holdRpm)) + " rpm "
                        "(clamped to 95% of each engine's valid range).\n");
    std::cout << (freeRun
        ? "FREE-RUN: the wall-clock sleep is removed, so the factor is CAPACITY.\n"
          "1.0 is exactly break-even and leaves no margin for scheduler jitter.\n\n"
        : "A factor below 1.0 means the simulation is running in slow motion.\n"
          "It saturates at 1.0; use --free-run to see the headroom above it.\n\n");
    if (withAudio)
        std::cout << "AUDIO: production RealtimeEngineAudio consumer at "
                  << static_cast<int>(audioSampleRate) << " Hz / "
                  << audioBlockSize << " samples"
                  << (outletJetNoiseEnabled
                      ? ", outlet jet noise ON"
                      : ", outlet jet noise OFF")
                  << (freeRun
                      ? ", paced by accelerated simulated time.\n"
                      : ", paced by wall-clock deadlines.\n")
                  << "A valid point requires no "
                     "over-budget render, queue loss, invalid boundary, "
                     "legacy sample, leveler activity or non-finite output.\n\n";
    std::cout << std::left << std::setw(26) << "engine"
              << std::right << std::setw(5) << "cyl"
              << std::setw(9) << "target"
              << std::setw(9) << "meanRpm"
              << std::setw(6) << "wrk"
              << std::setw(8) << "held"
              << std::setw(9) << "factor"
              << std::setw(11) << "sim/wall"
              << std::setw(12) << "overruns"
              << std::setw(10) << "maxLate"
              << std::setw(11) << "exhPeak"
              << std::setw(11) << "exhMean"
              << std::setw(8) << "bpWarn"
              << std::setw(9) << "tyreLim";
    if (withAudio)
        std::cout << std::setw(9) << "audMean"
                  << std::setw(9) << "audP99"
                  << std::setw(8) << "miss"
                  << std::setw(8) << "dropP"
                  << std::setw(8) << "late"
                  << std::setw(8) << "legacy"
                  << std::setw(8) << "lvl";
    std::cout << '\n';

    auto worst = 1.0e30;
    std::string worstEngine;
    auto failures = 0;
    auto invalidMeasurements = 0;
    for (const auto& entry : catalog.entries) {
        if (!filter.empty() && lowercase(entry.config.name).find(filter) == std::string::npos)
            continue;
        const auto revLimitRpm = std::min(
            entry.config.redlineRpm,
            entry.config.ignition.revLimitRpm);
        // A latched rev limiter is not a steady operating point: missing sparks
        // contaminate pressure, torque and timing cost. Every other WOT/science
        // harness already stops at 95%; apply the same ceiling here so the
        // documented absolute 7,000 rpm catalogue command cannot silently ask
        // low-redline engines to hold on the limiter.
        const auto maximumMeasurementRpm = 0.95 * revLimitRpm;
        const auto requestedRpm = std::min(maximumMeasurementRpm,
            relativeRpm.has_value()
                ? revLimitRpm * *relativeRpm : holdRpm);
        const auto measurement = measureEngine(
            entry.config, requestedRpm, warmupSeconds, measureSeconds, freeRun,
            intakeWorkers, intakeMaximumCells, intakeStaircaseRounds,
            intakeCouplingSeconds, intakeTargetCellLengthM,
            exhaustCouplingSeconds, intakeFirstOrderTimeIntegration,
            intakeWallHeatUpdateSeconds,
            useWellMixedExhaustJunctions, withAudio, outletJetNoiseEnabled,
            catalogRoot,
            audioSampleRate, audioBlockSize);
        const auto cylinders = static_cast<int>(entry.config.cylinders.size());
        std::cout << std::left << std::setw(26) << entry.config.name
                  << std::right << std::setw(5) << cylinders
                  << std::setw(9) << std::fixed << std::setprecision(0) << measurement.targetRpm
                  << std::setw(9) << measurement.meanRpm
                  << std::setw(6) << measurement.intakeWorkers
                  << std::setw(8) << (measurement.valid ? "yes" : "INVALID")
                  << std::setw(9) << (measurement.valid
                      ? [&measurement] {
                            std::ostringstream value;
                            value << std::fixed << std::setprecision(3)
                                  << measurement.realtimeFactor;
                            return value.str();
                        }()
                      : "--")
                  << std::setw(11) << (std::to_string(static_cast<int>(measurement.simulatedSeconds * 100.0) / 100)
                                       + "/" + std::to_string(static_cast<int>(measurement.wallSeconds * 100.0) / 100))
                  << std::setw(12) << measurement.overruns
                  << std::setw(8) << std::setprecision(1) << measurement.maximumLatenessMs << "ms"
                  << std::setw(11) << measurement.exhaustPeakKpa
                  << std::setw(11) << measurement.exhaustMeanKpa
                  << std::setw(8) << (measurement.backPressureWarning ? "YES" : "no")
                  << std::setw(9) << (measurement.tractionLimited ? "YES" : "no");
        if (withAudio) {
            std::ostringstream audioMean;
            audioMean << std::fixed << std::setprecision(1)
                      << measurement.audio.utilisationMeanPercent << '%';
            std::ostringstream audioP99;
            audioP99 << std::fixed << std::setprecision(0)
                     << measurement.audio.utilisationP99Percent << '%';
            std::cout << std::setw(9) << audioMean.str()
                      << std::setw(9) << audioP99.str()
                      << std::setw(8) << measurement.audio.deadlineMisses
                      << std::setw(8) << measurement.droppedPressureSamples
                      << std::setw(8) << measurement.audio.lateEvents
                      << std::setw(8) << measurement.audio.legacyPathSamples
                      << std::setw(8) << measurement.audio.levelLimitedSamples;
        }
        std::cout << '\n';
        if (!measurement.valid) {
            std::cerr << "INVALID: " << entry.config.name << ": "
                      << measurement.invalidReason << '\n';
            ++invalidMeasurements;
        } else if (measurement.realtimeFactor < worst) {
            worst = measurement.realtimeFactor;
            worstEngine = entry.config.name;
        }
        if (measurement.valid && failBelow > 0.0
            && measurement.realtimeFactor < failBelow) {
            std::cerr << "FAIL: " << entry.config.name << " produced only "
                      << std::setprecision(3) << measurement.realtimeFactor
                      << " simulated seconds per wall second (floor " << failBelow << ")\n";
            ++failures;
        }
    }

    if (worstEngine.empty()) std::cout << "\nworst: n/a (no valid points)\n";
    else
        std::cout << "\nworst: " << worstEngine << " at "
                  << std::setprecision(3) << worst << '\n';
    if (invalidMeasurements > 0)
        std::cerr << invalidMeasurements << " invalid measurement(s); no comparison is allowed\n";
    if (failures > 0 || invalidMeasurements > 0) {
        std::cerr << failures << " engine(s) below the realtime floor\n";
        return 1;
    }
    return 0;
}
