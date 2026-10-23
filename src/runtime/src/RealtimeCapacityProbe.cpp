#include <enginelab/runtime/RealtimeCapacityProbe.hpp>

#include <enginelab/runtime/EngineRuntime.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <memory>
#include <thread>

namespace enginelab {

double realtimeCapacityHoldRpm(const EngineConfig& config) noexcept {
    const auto limiterRpm = std::min(config.redlineRpm, config.ignition.revLimitRpm);
    return std::min(5'000.0, 0.95 * limiterRpm);
}

RealtimeHoldJudge::RealtimeHoldJudge(double holdRpm) noexcept
    : holdRpm_(holdRpm), toleranceRpm_(std::max(60.0, 0.03 * holdRpm)) {}

bool RealtimeHoldJudge::add(double simulationTimeSeconds, double rpm) noexcept {
    if (!started_) {
        started_ = true;
        meanRpm_ = rpm;
        settleStartSeconds_ = simulationTimeSeconds;
        settleStartRpm_ = rpm;
    } else {
        const auto dt = simulationTimeSeconds - lastTimeSeconds_;
        if (dt <= 0.0) return held_;
        // The budget harness's filter: a 1/3 s time constant.
        meanRpm_ += (rpm - meanRpm_) * (1.0 - std::exp(-3.0 * dt));
    }
    lastTimeSeconds_ = simulationTimeSeconds;
    if (std::abs(meanRpm_ - holdRpm_) <= toleranceRpm_) {
        if (withinSince_ < 0.0) withinSince_ = simulationTimeSeconds;
        held_ = simulationTimeSeconds - withinSince_ >= 0.5;
    } else {
        withinSince_ = -1.0;
        held_ = false;
    }
    // Settled short of the target: the dyno cannot drag a full-load engine
    // up to it (the Merlin and the radial stop 5 % short). The cost there is
    // as good a reading.
    if (!held_ && simulationTimeSeconds - settleStartSeconds_ >= 1.0) {
        held_ = std::abs(meanRpm_ - settleStartRpm_) <= 0.005 * holdRpm_
            && std::abs(meanRpm_ - holdRpm_) <= 0.15 * holdRpm_;
        settleStartSeconds_ = simulationTimeSeconds;
        settleStartRpm_ = meanRpm_;
    }
    return held_;
}

RealtimeCapacity measureRealtimeCapacity(const EngineConfig& config, double measureWallSeconds,
                                         double wallSecondsLimit, const std::atomic<bool>* cancel) {
    using Clock = std::chrono::steady_clock;
    RealtimeCapacity result;
    result.holdRpm = realtimeCapacityHoldRpm(config);
    const auto cancelled = [cancel] { return cancel != nullptr && cancel->load(); };
    const auto deadline = Clock::now()
        + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(wallSecondsLimit));
    try {
        // About 7.4 MB: on the heap, never on the stack.
        auto runtime = std::make_unique<EngineRuntime>(config);
        runtime->setRealtimeThrottleEnabled(false);
        runtime->setDynoMaximumDurationSeconds(120.0);
        runtime->setIgnitionEnabled(true);
        runtime->setStarterEngaged(true);
        runtime->setDynoHoldEnabled(true);
        runtime->setDynoHoldRpm(result.holdRpm);
        runtime->start();
        // Cranked with the throttle shut until it runs, as a carburetted
        // engine is: at full throttle and cranking speed its venturi draws
        // nearly nothing and its pilot is dead, so it never catches.
        while (!cancelled() && Clock::now() < deadline) {
            const auto state = runtime->snapshot();
            if (state.rpm > 0.8 * config.idleRpm || state.simulationTimeSeconds > 5.0) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        runtime->startDyno();

        RealtimeHoldJudge judge(result.holdRpm);
        auto held = false;
        while (!held) {
            if (cancelled()) {
                result.error = "cancelled";
                break;
            }
            if (Clock::now() >= deadline) {
                result.error = "did not hold " + std::to_string(static_cast<int>(result.holdRpm))
                    + " rpm in time; mean speed " + std::to_string(static_cast<int>(std::lround(judge.meanRpm())))
                    + " rpm";
                break;
            }
            const auto state = runtime->snapshot();
            held = judge.add(state.simulationTimeSeconds, state.rpm);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        if (held) {
            result.measuredRpm = judge.meanRpm();
            const auto wallStart = Clock::now();
            const auto simulatedStart = runtime->snapshot().simulationTimeSeconds;
            const auto measureEnd = wallStart
                + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(measureWallSeconds));
            while (Clock::now() < measureEnd && !cancelled())
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            const auto simulatedEnd = runtime->snapshot().simulationTimeSeconds;
            const auto wallEnd = Clock::now();
            if (cancelled()) {
                result.error = "cancelled";
            } else {
                const auto wall = std::chrono::duration<double>(wallEnd - wallStart).count();
                result.factor = wall > 0.0 ? (simulatedEnd - simulatedStart) / wall : 0.0;
            }
        }
        runtime->stop();
    } catch (const std::exception& error) {
        result.error = error.what();
    }
    return result;
}

} // namespace enginelab
