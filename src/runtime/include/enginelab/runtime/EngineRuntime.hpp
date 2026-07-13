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
using CylinderPressureQueue = SpscQueue<CylinderPressureSample, 2'048>;
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
    std::atomic<float> boostPressureRatio { 1.0F };
    std::atomic<float> exhaustReflectionSeconds { 0.006F };
    std::atomic<float> volume { 1.0F };
    std::atomic<float> convolution { 0.45F };
    std::atomic<float> highFrequencyGain { 1.0F };
    std::atomic<float> lowFrequencyNoise { 0.35F };
    std::atomic<float> highFrequencyNoise { 0.35F };
    std::atomic<float> combustionGain { 1.0F };
    std::atomic<float> exhaustGain { 1.0F };
    std::atomic<float> intakeGain { 0.85F };
    std::atomic<float> mechanicalGain { 0.70F };
    std::atomic<int> exhaustPreset { static_cast<int>(AudioExhaustPreset::street) };
};
static_assert(std::atomic<float>::is_always_lock_free, "Realtime audio telemetry requires lock-free float atomics");

/** Owns the fixed-rate simulation thread and the simulation-to-audio event queue. */
class EngineRuntime final {
public:
    explicit EngineRuntime(EngineConfig);
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
    void setExhaustPreset(AudioExhaustPreset value) noexcept { audioState_.exhaustPreset.store(static_cast<int>(value), std::memory_order_relaxed); }
    [[nodiscard]] bool dynoHoldEnabled() const noexcept { return dynoHoldEnabled_.load(); }
    void setTargetAirFuelRatio(double value) noexcept { ecu_.setTargetAirFuelRatio(value); }
    void setIgnitionTrimDegrees(double value) noexcept { ecu_.setIgnitionTrimDegrees(value); }
    void setIgnitionAdvanceDegrees(double value) noexcept { setIgnitionTrimDegrees(value); }
    void setPaused(bool value) noexcept { paused_.store(value); }
    void setTimeScale(double value) noexcept { timeScale_.store(std::isfinite(value) ? std::clamp(value, 0.25, 4.0) : 1.0); }
    [[nodiscard]] bool paused() const noexcept { return paused_.load(); }
    [[nodiscard]] double timeScale() const noexcept { return timeScale_.load(); }
    void startDyno();
    void stopDyno();
    [[nodiscard]] bool dynoRunning() const noexcept { return dynoRunning_.load(); }
    [[nodiscard]] DynoRun currentDynoRun() const;
    [[nodiscard]] std::vector<DynoRun> dynoHistory() const;
    void deleteDynoRun(std::uint64_t id);
    [[nodiscard]] EngineState snapshot() const;
    [[nodiscard]] FiringEventQueue& audioEvents() noexcept { return eventQueue_; }
    [[nodiscard]] CylinderPressureQueue& cylinderPressureSamples() noexcept { return pressureQueue_; }
    [[nodiscard]] RealtimeAudioState& audioState() noexcept { return audioState_; }
    [[nodiscard]] std::uint64_t droppedEventCount() const noexcept { return droppedEvents_.load(); }
    [[nodiscard]] std::uint64_t droppedPressureSampleCount() const noexcept { return droppedPressureSamples_.load(); }
    [[nodiscard]] std::uint64_t timingOverrunCount() const noexcept { return timingOverruns_.load(); }
private:
    void run(std::stop_token stopToken);
    [[nodiscard]] double updateDriveline(double dtSeconds, const EngineState& engineState, double requestedLoad) noexcept;
    EngineConfig config_;
    SimpleEcuModel ecu_;
    SimplifiedGasolinePhysics physics_;
    FourStrokeEventGenerator eventGenerator_;
    ExhaustGraph exhaust_;
    EngineSimulator simulator_;
    DrivelineModel driveline_;
    DrivelineOutput drivelineOutput_;
    FiringEventQueue eventQueue_;
    CylinderPressureQueue pressureQueue_;
    RealtimeAudioState audioState_;
    mutable std::mutex snapshotMutex_;
    EngineState snapshot_;
    std::atomic<bool> ignition_ { false };
    std::atomic<bool> starter_ { false };
    std::atomic<double> throttle_ { 0.12 };
    std::atomic<double> load_ { 0.08 };
    std::atomic<double> clutchPressure_ { 1.0 };
    std::atomic<double> brakePressure_ { 0.0 };
    std::atomic<int> gear_ { -1 };
    std::atomic<bool> dynoHoldEnabled_ { false };
    std::atomic<double> dynoHoldRpm_ { 2'500.0 };
    std::atomic<std::uint64_t> droppedEvents_ { 0 };
    std::atomic<std::uint64_t> droppedPressureSamples_ { 0 };
    std::atomic<std::uint64_t> timingOverruns_ { 0 };
    std::atomic<bool> paused_ { false };
    std::atomic<double> timeScale_ { 1.0 };
    std::atomic<bool> dynoRunning_ { false };
    mutable std::mutex dynoMutex_;
    DynoRun currentRun_;
    std::vector<DynoRun> dynoHistory_;
    std::uint64_t nextDynoId_ { 1 };
    double dynoElapsed_ { 0.0 };
    double nextSampleRpm_ { 0.0 };
    double dynoTargetRpm_ { 0.0 };
    double dynoStableElapsed_ { 0.0 };
    double dynoLoadCommand_ { 0.34 };
    double dynoFilteredRpm_ { 0.0 };
    double dynoTorqueAccumulator_ { 0.0 };
    double dynoPowerAccumulator_ { 0.0 };
    std::uint32_t dynoSampleCount_ { 0 };
    bool savedIgnition_ { false };
    bool savedStarter_ { false };
    double savedThrottle_ { 0.12 };
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
