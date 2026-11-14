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
    /** Speed under which the engaged gear lugs and the gearbox changes down
     *  whatever the pedal; 0 derives it from the idle. */
    double coastDownshiftRpm { 0.0 };
    /** The driver's brake pedal (0-1): braking changes down early, for the
     *  engine's braking, and never changes up. */
    double brake { 0.0 };
};

/** Shift points of an automatic gearbox, from the pedal: a light foot shifts
 * up early and a heavy one late, near the limiter. It changes down when the
 * road speed falls a quarter under the speed it would change up at with
 * that pedal: pressing harder raises that line, so the pedal asks for a
 * lower gear (several at once, kickdown, as long as they land under the
 * upshift point), and braking raises it further, for the engine's braking.
 * Under the lugging speed it changes down whatever the pedal.
 *
 * Hysteresis keeps it from hunting: the downshift line is a quarter of the
 * road speed under the upshift line, an upshift must land clear of both, and
 * it never changes up while braking. The caller adds the time hysteresis (no
 * new choice right after a shift) and keeps it out of a launch and of the
 * driver's clutch. */
class ShiftSchedule final {
public:
    [[nodiscard]] static double upshiftRpm(const ShiftScheduleInput&) noexcept;
    /** The highest speed the next gear down may land at for the gearbox to
     *  change down to it: the downshift line, read in the lower gear. */
    [[nodiscard]] static double downshiftLandingRpm(const ShiftScheduleInput&) noexcept;
    /** Under this speed the engaged gear lugs: it changes down whatever the
     *  pedal. */
    [[nodiscard]] static double luggingRpm(const ShiftScheduleInput&) noexcept;
    /** The gear to be in: the engaged one, one higher, or one or more lower. */
    [[nodiscard]] static int chooseGear(const ShiftScheduleInput&) noexcept;
};

} // namespace enginelab
