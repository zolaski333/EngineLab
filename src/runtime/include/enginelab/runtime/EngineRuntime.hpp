#pragma once
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FiringEvent.hpp>
#include <enginelab/events/CylinderPressureSample.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/foundation/SpscQueue.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>
#include <enginelab/runtime/DrivelineModel.hpp>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

namespace enginelab {
using FiringEventQueue = SpscQueue<FiringEvent, 2'048>;
// The runtime publishes at most 96 kHz of pressure telemetry. 8,191 usable
// slots retain more than 85 ms, covering the adaptive look-ahead plus a full
// 2,048-sample callback at every supported sample rate.
using CylinderPressureQueue = SpscQueue<CylinderPressureSample, 8'192>;
enum class AudioExhaustPreset : int { street = 0, openHeaders = 1, turboMuffled = 2, longTube = 3, motorcycle = 4 };

struct RealtimeAudioState final {
    std::atomic<float> rpm { 0.0F };
    std::atomic<float> throttle { 0.0F };
    std::atomic<float> load { 0.0F };
    std::atomic<float> mechanicalStress { 0.0F };
    std::atomic<float> starter { 0.0F };
    std::atomic<float> timeScale { 1.0F };
    std::atomic<float> cylinderCount { 4.0F };
    std::atomic<float> redlineRpm { 7'200.0F };
    std::atomic<float> displacementLitres { 2.0F };
    std::atomic<float> cylinderDisplacementLitres { 0.5F };
    std::atomic<float> boreStrokeRatio { 1.0F };
    std::atomic<float> bankSeparation { 0.0F };
    std::array<std::atomic<float>, 32> cylinderPan {};
    std::atomic<float> exhaustOpenness { 0.5F };
    std::atomic<float> manifoldPressureKpa { 101.325F };
    std::atomic<float> exhaustPressureKpa { 101.325F };
    std::atomic<float> exhaustFlowGramsPerSecond { 0.0F };
    std::atomic<float> exhaustTemperatureC { 20.0F };
    std::atomic<float> exhaustReferenceSoundSpeedMps { 520.0F };
    std::atomic<float> boostPressureRatio { 1.0F };
    std::atomic<float> exhaustReflectionSeconds { 0.006F };
    std::atomic<float> ambientPressureKpa { 101.325F };
    std::atomic<std::uint64_t> producerTimeNanoseconds { 0 };
    std::atomic<std::uint32_t> exhaustPathCount { 1 };
    std::array<std::atomic<float>, 8> exhaustPathOpenness {
        0.5F, 0.5F, 0.5F, 0.5F, 0.5F, 0.5F, 0.5F, 0.5F
    };
    std::array<std::atomic<float>, 8> exhaustPathReflectionSeconds {
        0.006F, 0.006F, 0.006F, 0.006F, 0.006F, 0.006F, 0.006F, 0.006F
    };
    std::array<std::atomic<float>, 8> exhaustPathGain {
        1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F
    };
    // Expansion-chamber silencer on each collector-to-outlet duct, published as
    // bare geometry: the expansion ratio (chamber area / duct area) and the
    // one-way traversal time. A zero on either means "no chamber", which the
    // audio element renders as an exact through-connection -- the correct model
    // for an open stack, and what every engine gets until it configures one.
    std::array<std::atomic<float>, 8> exhaustPathMufflerExpansionRatio {};
    std::array<std::atomic<float>, 8> exhaustPathMufflerTraversalSeconds {};
    std::array<std::atomic<std::uint32_t>, 32> cylinderExhaustPathIndex {};
    /** Stable IDs let a compiled acoustic graph map its port order back to the
     * configuration-order pressure telemetry without assuming either order. */
    std::array<std::atomic<std::uint32_t>, 32> cylinderId {};
    // Per-cylinder graph transmission. The pressure renderer applies it before
    // the runner waveguide; firing events carry the same metric in their own
    // payload, so neither source is multiplied again at the path output.
    std::array<std::atomic<float>, 32> cylinderExhaustGain {
        1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F,
        1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F,
        1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F,
        1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F
    };
    std::atomic<float> volume { 1.0F };
    std::atomic<float> convolution { 0.45F };
    std::atomic<float> highFrequencyGain { 1.0F };
    std::atomic<float> lowFrequencyNoise { 0.35F };
    std::atomic<float> highFrequencyNoise { 0.35F };
    std::atomic<float> combustionGain { 1.0F };
    std::atomic<float> exhaustGain { 1.0F };
    std::atomic<float> intakeGain { 0.85F };
    std::atomic<float> mechanicalGain { 0.70F };
    // Microphone/preamp calibration for SI exhaust pressure. dBFS has no
    // intrinsic pressure unit; keeping the capture-chain headroom explicit
    // avoids disguising a fixed voicing gain as acoustics.
    // Derived from what the model actually radiates at the observer, not chosen
    // by ear. Must track AcousticMonitorCalibration::defaultFullScaleSplDb,
    // which documents the measurement it comes from; the constant is duplicated
    // rather than included because audio depends on runtime, not the reverse.
    // RealtimeRegressionTests asserts the two stay equal.
    std::atomic<float> acousticFullScaleSplDb { 134.0F };
    std::atomic<int> exhaustPreset { static_cast<int>(AudioExhaustPreset::street) };
    // Extended physical telemetry for the intake/forced-induction/mechanical and
    // waveguide audio layers (populated once at construction or per sim frame).
    std::atomic<float> intakeRunnerResonanceHz { 0.0F };
    std::atomic<float> intakeRunnerAmplitudeKpa { 0.0F };
    std::atomic<float> forcedInductionShaftRpm { 0.0F };
    std::atomic<float> wastegateOpening { 0.0F };
    std::atomic<int> forcedInductionKind { 0 };  // 0 none, 1 turbocharger, 2 supercharger
    std::atomic<float> meanBoreMm { 84.0F };
    std::atomic<float> peakPistonAccelerationG { 0.0F };
    std::array<std::atomic<float>, 32> runnerDelaySeconds {};
    // Acoustic scattering uses cross-sectional admittance instead of assuming
    // that every primary and outlet has the same impedance.
    std::array<std::atomic<float>, 32> cylinderExhaustAreaM2 {};
    std::array<std::atomic<float>, 8> exhaustPathOutletAreaM2 {};
};
static_assert(std::atomic<float>::is_always_lock_free, "Realtime audio telemetry requires lock-free float atomics");
static_assert(std::atomic<std::uint64_t>::is_always_lock_free,
              "Realtime audio clock publication requires a lock-free 64-bit atomic");

/** Per-frame audio context that EngineState does not itself carry. */
struct AudioFramePublication final {
    bool paused { false };
    bool starterEngaged { false };
    /// Driveline clutch load; the published load is the max of this and EngineState::load.
    double drivelineLoad { 0.0 };
    double timeScale { 1.0 };
};

/**
 * Map one simulation frame onto the realtime audio telemetry.
 *
 * This is the only definition of the per-frame simulation-to-audio mapping.
 * EngineRuntime calls it from its simulation thread and the offline render
 * harness calls it with the frames it steps itself, so a regression run
 * exercises the mapping the application ships rather than a parallel copy.
 * Static, configuration-derived telemetry (geometry, pan, path areas, runner
 * delays) is published once by the EngineRuntime constructor instead.
 */
void publishAudioFrame(RealtimeAudioState& state, const EngineState& engineState,
                       const AudioFramePublication& context) noexcept;

/** Owns the fixed-rate simulation thread and the simulation-to-audio event queue. */
class EngineRuntime final {
public:
    explicit EngineRuntime(EngineConfig,
                           std::shared_ptr<calibration::CalibrationStore> calibrations = {});
    ~EngineRuntime();
    EngineRuntime(const EngineRuntime&) = delete;
    EngineRuntime& operator=(const EngineRuntime&) = delete;
    void start();
    void stop();
    void setIgnitionEnabled(bool value) noexcept { ignition_.store(value); }
    void setStarterEngaged(bool value) noexcept { starter_.store(value); }
    void setThrottle(double value) noexcept { throttle_.store(std::isfinite(value) ? std::clamp(value, 0.0, 1.0) : 0.0); }
    void setLoad(double value) noexcept { load_.store(std::isfinite(value) ? std::clamp(value, 0.0, 1.0) : 0.0); }
    void setClutchPressure(double value) noexcept { clutchPressure_.store(std::isfinite(value) ? std::clamp(value, 0.0, 1.0) : 1.0); }
    void setBrakePressure(double value) noexcept { brakePressure_.store(std::isfinite(value) ? std::clamp(value, 0.0, 1.0) : 0.0); }
    void shiftUp() noexcept;
    void shiftDown() noexcept;
    void setGear(int gear) noexcept;
    void setDynoHoldEnabled(bool value) noexcept { dynoHoldEnabled_.store(value); }
    void adjustDynoHoldRpm(double delta) noexcept;
    void setAudioVolume(double value) noexcept { audioState_.volume.store(static_cast<float>(std::clamp(value, 0.0, 2.0))); }
    void setAudioConvolution(double value) noexcept { audioState_.convolution.store(static_cast<float>(std::clamp(value, 0.0, 1.0))); }
    void setHighFrequencyGain(double value) noexcept { audioState_.highFrequencyGain.store(static_cast<float>(std::clamp(value, 0.2, 2.5))); }
    void setLowFrequencyNoise(double value) noexcept { audioState_.lowFrequencyNoise.store(static_cast<float>(std::clamp(value, 0.0, 1.5))); }
    void setHighFrequencyNoise(double value) noexcept { audioState_.highFrequencyNoise.store(static_cast<float>(std::clamp(value, 0.0, 1.5))); }
    void setCombustionGain(double value) noexcept { audioState_.combustionGain.store(static_cast<float>(std::clamp(value, 0.0, 2.0))); }
    void setExhaustGain(double value) noexcept { audioState_.exhaustGain.store(static_cast<float>(std::clamp(value, 0.0, 2.0))); }
    void setIntakeGain(double value) noexcept { audioState_.intakeGain.store(static_cast<float>(std::clamp(value, 0.0, 2.0))); }
    void setMechanicalGain(double value) noexcept { audioState_.mechanicalGain.store(static_cast<float>(std::clamp(value, 0.0, 2.0))); }
    void setAcousticFullScaleSplDb(double value) noexcept {
        audioState_.acousticFullScaleSplDb.store(
            static_cast<float>(std::clamp(value, 100.0, 180.0)));
    }
    void setExhaustPreset(AudioExhaustPreset value) noexcept { audioState_.exhaustPreset.store(static_cast<int>(value), std::memory_order_relaxed); }
    [[nodiscard]] bool dynoHoldEnabled() const noexcept { return dynoHoldEnabled_.load(); }
    void setAirFuelRatioTrim(double value) noexcept { ecu_.setAirFuelRatioTrim(value); }
    void setTargetAirFuelRatio(double value) noexcept { ecu_.setTargetAirFuelRatio(value); }
    void setIgnitionTrimDegrees(double value) noexcept { ecu_.setIgnitionTrimDegrees(value); }
    void setIgnitionAdvanceDegrees(double value) noexcept { setIgnitionTrimDegrees(value); }
    void setPaused(bool value) noexcept { paused_.store(value); }
    void setTimeScale(double value) noexcept { timeScale_.store(std::isfinite(value) ? std::clamp(value, 0.25, 4.0) : 1.0); }
    [[nodiscard]] bool paused() const noexcept { return paused_.load(); }
    [[nodiscard]] double timeScale() const noexcept { return timeScale_.load(); }
    void startDyno();
    void stopDyno();
    [[nodiscard]] bool dynoRunning() const noexcept {
        return dynoRequestedRunning_.load(std::memory_order_acquire);
    }
    [[nodiscard]] DynoRun currentDynoRun() const;
    [[nodiscard]] std::vector<DynoRun> dynoHistory() const;
    void deleteDynoRun(std::uint64_t id);
    [[nodiscard]] EngineState snapshot() const;
    [[nodiscard]] FiringEventQueue& audioEvents() noexcept { return eventQueue_; }
    [[nodiscard]] CylinderPressureQueue& cylinderPressureSamples() noexcept { return *pressureQueue_; }
    [[nodiscard]] RealtimeAudioState& audioState() noexcept { return audioState_; }
    [[nodiscard]] const EngineConfig& engineConfig() const noexcept { return config_; }
    [[nodiscard]] const ExhaustGraph& exhaustGraph() const noexcept { return exhaust_; }
    [[nodiscard]] std::shared_ptr<calibration::CalibrationStore> calibrationStore() const noexcept {
        return ecu_.calibrationStore();
    }
    [[nodiscard]] std::uint64_t droppedEventCount() const noexcept { return droppedEvents_.load(); }
    [[nodiscard]] std::uint64_t droppedPressureSampleCount() const noexcept { return droppedPressureSamples_.load(); }
    [[nodiscard]] std::uint64_t timingOverrunCount() const noexcept { return timingOverruns_.load(); }
    [[nodiscard]] double maximumTimingLatenessSeconds() const noexcept {
        return maximumTimingLatenessSeconds_.load();
    }
private:
    void run(std::stop_token stopToken);
    void beginDynoSession();
    void finishDynoSession();
    void updateDriveline(double dtSeconds, const EngineState& engineState,
                         double requestedLoad) noexcept;
    EngineConfig config_;
    SimpleEcuModel ecu_;
    SimplifiedGasolinePhysics physics_;
    FourStrokeEventGenerator eventGenerator_;
    ExhaustGraph exhaust_;
    EngineSimulator simulator_;
    DrivelineModel driveline_;
    DrivelineOutput drivelineOutput_;
    FiringEventQueue eventQueue_;
    // Large enough for adaptive audio look-ahead; heap storage keeps
    // EngineRuntime safe to instantiate in stack-based tools/tests.
    std::unique_ptr<CylinderPressureQueue> pressureQueue_;
    RealtimeAudioState audioState_;
    mutable std::mutex snapshotMutex_;
    EngineState snapshot_;
    std::atomic<bool> ignition_ { false };
    std::atomic<bool> starter_ { false };
    std::atomic<double> throttle_ { 0.0 };
    std::atomic<double> load_ { 0.08 };
    std::atomic<double> clutchPressure_ { 1.0 };
    std::atomic<double> brakePressure_ { 0.0 };
    std::atomic<int> gear_ { -1 };
    std::atomic<std::uint64_t> gearCommandGeneration_ { 0 };
    std::atomic<bool> dynoHoldEnabled_ { false };
    std::atomic<double> dynoHoldRpm_ { 2'500.0 };
    std::atomic<std::uint64_t> droppedEvents_ { 0 };
    std::atomic<std::uint64_t> droppedPressureSamples_ { 0 };
    std::atomic<std::uint64_t> timingOverruns_ { 0 };
    std::atomic<double> maximumTimingLatenessSeconds_ { 0.0 };
    std::atomic<bool> paused_ { false };
    std::atomic<double> timeScale_ { 1.0 };
    // UI writes only the desired state. The simulation thread owns all mutable
    // session fields below and reconciles this mailbox once per tick.
    std::atomic<bool> dynoRequestedRunning_ { false };
    bool dynoActive_ { false };
    mutable std::mutex dynoMutex_;
    DynoRun currentRun_;
    std::vector<DynoRun> dynoHistory_;
    std::uint64_t nextDynoId_ { 1 };
    double dynoElapsed_ { 0.0 };
    double dynoStartupElapsed_ { 0.0 };
    double nextSampleRpm_ { 0.0 };
    double dynoTargetRpm_ { 0.0 };
    double dynoStableElapsed_ { 0.0 };
    double dynoBrakeTorqueNm_ { 0.0 };
    double dynoControllerIntegralNm_ { 0.0 };
    double dynoFeedForwardTorqueNm_ { 0.0 };
    double dynoFilteredRpm_ { 0.0 };
    double dynoFilteredAccelerationRpmPerSecond_ { 0.0 };
    double dynoTorqueAccumulator_ { 0.0 };
    double dynoPowerAccumulator_ { 0.0 };
    std::uint32_t dynoSampleCount_ { 0 };
    bool savedIgnition_ { false };
    bool savedStarter_ { false };
    double savedThrottle_ { 0.0 };
    double savedLoad_ { 0.08 };
    double vehicleSpeedMps_ { 0.0 };
    double vehicleDistanceM_ { 0.0 };
    double wheelTorqueNm_ { 0.0 };
    double drivelineLoadTorqueNm_ { 0.0 };
    double engineClutchTorqueNm_ { 0.0 };
    double clutchSlipRpm_ { 0.0 };
    double effectiveClutchPressure_ { 0.0 };
    int engagedGear_ { -1 };
    double shiftProgress_ { 0.0 };
    bool shiftInProgress_ { false };
    std::atomic<bool> dynoSweeping_ { false };
    std::atomic<bool> dynoCompleted_ { false };
    std::jthread thread_;
};
} // namespace enginelab
