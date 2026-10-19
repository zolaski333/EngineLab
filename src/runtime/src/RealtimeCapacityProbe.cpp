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
        runtime->startDyno();

        // Held: within 3 % (60 rpm at least) for half a simulated second.
        const auto tolerance = std::max(60.0, 0.03 * result.holdRpm);
        auto heldSince = -1.0;
        auto held = false;
        while (!held) {
            if (cancelled()) {
                result.error = "cancelled";
                break;
            }
            if (Clock::now() >= deadline) {
                result.error = "did not hold " + std::to_string(static_cast<int>(result.holdRpm)) + " rpm in time";
                break;
            }
            const auto state = runtime->snapshot();
            if (std::abs(state.rpm - result.holdRpm) <= tolerance) {
                if (heldSince < 0.0) heldSince = state.simulationTimeSeconds;
                held = state.simulationTimeSeconds - heldSince >= 0.5;
            } else {
                heldSince = -1.0;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        if (held) {
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
