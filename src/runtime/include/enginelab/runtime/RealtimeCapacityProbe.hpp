#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

#include <atomic>
#include <string>

namespace enginelab {

/**
 * How fast this computer runs an engine's physics: simulated seconds per wall
 * second with the real-time sleep removed, held on the dyno at
 * realtimeCapacityHoldRpm(). The same quantity and speed as
 * `EngineLabRealtimeBudgetHarness --free-run`, measured in a few seconds so
 * the engine designer can check a design before it is built.
 *
 * Physics only: the audio renderer and the 3-D view's gas requests come on
 * top. Measured on 19 designs (journal 2026-10-19), the whole chain runs at
 * 0.88-0.97 of the physics-only factor, hence realtimeCapacityMargin.
 */
struct RealtimeCapacity final {
    double factor { 0.0 };
    double holdRpm { 0.0 };
    /** The mean speed the factor was measured at: holdRpm, or where a
     *  full-load engine settled short of it. */
    double measuredRpm { 0.0 };
    /** Empty when the factor was measured; otherwise why it was not. */
    std::string error;
};

/** Physics-only factor below which the whole chain may fall under real time. */
inline constexpr double realtimeCapacityMargin = 1.14;

/** The catalogue's measuring speed: 5,000 rpm, at most 95 % of the limiter. */
[[nodiscard]] double realtimeCapacityHoldRpm(const EngineConfig& config) noexcept;

/**
 * Whether the dyno holds a speed, from speeds polled at any crank angle: held
 * once the mean speed (filtered over 1/3 s of simulated time) has stayed within
 * 3 % (60 rpm at least) for half a simulated second, or has settled (moved
 * under 0.5 % in a simulated second) within 15 % of it. On one sample a single
 * at 5,000 rpm swings +4 / -6 % within each cycle and never counted as held
 * (journal 2026-10-19).
 */
class RealtimeHoldJudge final {
public:
    explicit RealtimeHoldJudge(double holdRpm) noexcept;
    /** Adds a polled sample; returns whether the speed is now held. */
    bool add(double simulationTimeSeconds, double rpm) noexcept;
    [[nodiscard]] double meanRpm() const noexcept { return meanRpm_; }

private:
    double holdRpm_;
    double toleranceRpm_;
    double meanRpm_ { 0.0 };
    double lastTimeSeconds_ { 0.0 };
    double withinSince_ { -1.0 };
    double settleStartSeconds_ { 0.0 };
    double settleStartRpm_ { 0.0 };
    bool started_ { false };
    bool held_ { false };
};

/**
 * Runs the engine for at most `wallSecondsLimit` (reaching the hold speed,
 * then `measureWallSeconds` of measurement). `cancel` may be null; when it is
 * set the probe stops and reports "cancelled".
 */
[[nodiscard]] RealtimeCapacity measureRealtimeCapacity(const EngineConfig& config,
                                                       double measureWallSeconds = 1.5,
                                                       double wallSecondsLimit = 30.0,
                                                       const std::atomic<bool>* cancel = nullptr);

} // namespace enginelab
