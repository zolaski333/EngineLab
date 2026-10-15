#include <enginelab/runtime/ShiftSchedule.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace enginelab {

namespace {
// How far the pedal moves the shift points: a power above 1 keeps part
// throttle near the economical end, as production schedules do (their
// curves are not published per car; the shape is a choice, not a fit).
double pedalShape(double throttle) noexcept {
    const auto t = std::isfinite(throttle) ? std::clamp(throttle, 0.0, 1.0) : 0.0;
    return t * std::sqrt(t);
}

double limitOf(const ShiftScheduleInput& input) noexcept {
    return input.revLimitRpm > 0.0 && std::isfinite(input.revLimitRpm) ? input.revLimitRpm : 6'500.0;
}

double idleOf(const ShiftScheduleInput& input) noexcept {
    return std::isfinite(input.idleRpm) ? std::max(300.0, input.idleRpm) : 800.0;
}

double fullThrottleUpshift(const ShiftScheduleInput& input) noexcept {
    // The engine keeps climbing during the shift: change just under the
    // limiter rather than on it.
    const auto ceiling = 0.97 * limitOf(input);
    return input.fullThrottleUpshiftRpm > 0.0 ? std::min(input.fullThrottleUpshiftRpm, ceiling)
                                              : 0.94 * limitOf(input);
}

double landingRpm(const ShiftScheduleInput& input, int gear) noexcept {
    const auto index = static_cast<std::size_t>(gear);
    return std::abs(input.wheelRadPerSecond * input.overallRatios[index]) * 60.0 / (2.0 * std::numbers::pi);
}
} // namespace

double ShiftSchedule::upshiftRpm(const ShiftScheduleInput& input) noexcept {
    const auto full = fullThrottleUpshift(input);
    // A light foot changes up at a little over twice the idle speed.
    const auto light = std::min(full, std::max(2.2 * idleOf(input), 0.3 * limitOf(input)));
    return light + (full - light) * pedalShape(input.driverThrottle);
}

double ShiftSchedule::downshiftRpm(const ShiftScheduleInput& input) noexcept {
    const auto coast = input.coastDownshiftRpm > 0.0 ? input.coastDownshiftRpm : 1.3 * idleOf(input);
    // Flat out, anything under about half the upshift speed asks for a
    // lower gear: that is the kickdown.
    const auto kickdown = std::max(coast, 0.55 * fullThrottleUpshift(input));
    return coast + (kickdown - coast) * pedalShape(input.driverThrottle);
}

int ShiftSchedule::chooseGear(const ShiftScheduleInput& input) noexcept {
    const auto gearCount = static_cast<int>(input.overallRatios.size());
    const auto engaged = input.engagedGear;
    if (engaged < 0 || engaged >= gearCount || !std::isfinite(input.engineRpm)
        || !std::isfinite(input.wheelRadPerSecond))
        return engaged;
    const auto up = upshiftRpm(input);
    const auto down = downshiftRpm(input);
    // An upshift must land clear of the downshift point, or the next frame
    // would change back down.
    constexpr double upshiftLandingMargin = 1.15;
    if (input.engineRpm >= up && engaged + 1 < gearCount
        && landingRpm(input, engaged + 1) >= down * upshiftLandingMargin)
        return engaged + 1;
    // A downshift must land under the upshift point, with the same kind of
    // margin; kickdown carries on down while the gear reached still lugs.
    constexpr double downshiftLandingMargin = 0.9;
    auto gear = engaged;
    auto rpm = input.engineRpm;
    while (gear > 0 && rpm <= down && landingRpm(input, gear - 1) <= downshiftLandingMargin * up) {
        --gear;
        rpm = landingRpm(input, gear);
    }
    return gear;
}

} // namespace enginelab
