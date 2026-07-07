#pragma once
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FiringEvent.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/foundation/SpscQueue.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

namespace enginelab {
using FiringEventQueue = SpscQueue<FiringEvent, 2'048>;
struct RealtimeAudioState final {
    std::atomic<float> rpm { 0.0F };
    std::atomic<float> throttle { 0.0F };
    std::atomic<float> load { 0.0F };
    std::atomic<float> mechanicalStress { 0.0F };
    std::atomic<float> starter { 0.0F };
    std::atomic<float> timeScale { 1.0F };
    std::atomic<float> cylinderCount { 4.0F };
    std::atomic<float> exhaustReflectionSeconds { 0.006F };
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
    void setTargetAirFuelRatio(double value) noexcept { ecu_.setTargetAirFuelRatio(value); }
    void setIgnitionAdvanceDegrees(double value) noexcept { ecu_.setIgnitionAdvanceDegrees(value); }
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
    [[nodiscard]] RealtimeAudioState& audioState() noexcept { return audioState_; }
    [[nodiscard]] std::uint64_t droppedEventCount() const noexcept { return droppedEvents_.load(); }
    [[nodiscard]] std::uint64_t timingOverrunCount() const noexcept { return timingOverruns_.load(); }
private:
    void run(std::stop_token stopToken);
    EngineConfig config_;
    SimpleEcuModel ecu_;
    SimplifiedGasolinePhysics physics_;
    FourStrokeEventGenerator eventGenerator_;
    ExhaustGraph exhaust_;
    EngineSimulator simulator_;
    FiringEventQueue eventQueue_;
    RealtimeAudioState audioState_;
    mutable std::mutex snapshotMutex_;
    EngineState snapshot_;
    std::atomic<bool> ignition_ { false };
    std::atomic<bool> starter_ { false };
    std::atomic<double> throttle_ { 0.12 };
    std::atomic<double> load_ { 0.08 };
    std::atomic<std::uint64_t> droppedEvents_ { 0 };
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
    double dynoTorqueAccumulator_ { 0.0 };
    double dynoPowerAccumulator_ { 0.0 };
    std::uint32_t dynoSampleCount_ { 0 };
    bool savedIgnition_ { false };
    bool savedStarter_ { false };
    double savedThrottle_ { 0.12 };
    double savedLoad_ { 0.08 };
    std::atomic<bool> dynoSweeping_ { false };
    std::atomic<bool> dynoCompleted_ { false };
    std::jthread thread_;
};
} // namespace enginelab
