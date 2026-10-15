#pragma once

#include <span>

namespace enginelab {

/** What an automatic gearbox reads to choose its gear. */
struct ShiftScheduleInput final {
    double engineRpm { 0.0 };
    /** The driver's pedal (0-1), not the engine's throttle: a blip or a
     * torque cut must not move the shift points. */
    double driverThrottle { 0.0 };
    int engagedGear { -1 };
    /** Overall ratio (gear x final drive) of each forward gear. */
    std::span<const double> overallRatios;
    /** The driven wheel's speed: where the engine would land in each gear. */
    double wheelRadPerSecond { 0.0 };
    double idleRpm { 800.0 };
    /** The active rev limit (the ECU tuner's, else the authored one). */
    double revLimitRpm { 6'500.0 };
    /** Full-throttle upshift speed; 0 derives it from the rev limit. */
    double fullThrottleUpshiftRpm { 0.0 };
    /** Foot-off downshift speed; 0 derives it from the idle. */
    double coastDownshiftRpm { 0.0 };
};

/** Shift points of an automatic gearbox, from the pedal: a light foot shifts
 * up early and a heavy one late, near the limiter; pressing the pedal hard
 * at a low engine speed shifts down (kickdown), several gears at once if
 * they all land under the upshift point.
 *
 * Hysteresis keeps it from hunting: an upshift must land above the
 * downshift point with a margin, a downshift below the upshift point.
 * The caller adds the time hysteresis (no new choice right after a shift)
 * and keeps it out of a launch and of the driver's clutch. */
class ShiftSchedule final {
public:
    [[nodiscard]] static double upshiftRpm(const ShiftScheduleInput&) noexcept;
    [[nodiscard]] static double downshiftRpm(const ShiftScheduleInput&) noexcept;
    /** The gear to be in: the engaged one, one higher, or one or more lower. */
    [[nodiscard]] static int chooseGear(const ShiftScheduleInput&) noexcept;
};

} // namespace enginelab
