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

double ShiftSchedule::luggingRpm(const ShiftScheduleInput& input) noexcept {
    return input.coastDownshiftRpm > 0.0 ? input.coastDownshiftRpm : 1.3 * idleOf(input);
}

double ShiftSchedule::downshiftLandingRpm(const ShiftScheduleInput& input) noexcept {
    // A road speed a quarter under the upshift line at the same pedal, as
    // production schedules space their lines (their 3-2 line near 0.7 x the
    // 2-3 one at part throttle; not fitted to one car). Read in the lower
    // gear, that speed is 0.72 x the upshift point.
    constexpr double downshiftLine = 0.72;
    const auto demand = downshiftLine * upshiftRpm(input);
    // Braking: from that line up to half the full-throttle upshift speed at
    // half the brake pedal, where the engine brakes the car well without
    // nearing its limit.
    const auto brake = std::isfinite(input.brake) ? std::clamp(input.brake / 0.5, 0.0, 1.0) : 0.0;
    const auto braking = demand + (0.5 * fullThrottleUpshift(input) - demand) * brake;
    return std::max(demand, braking);
}

int ShiftSchedule::chooseGear(const ShiftScheduleInput& input) noexcept {
    const auto gearCount = static_cast<int>(input.overallRatios.size());
    const auto engaged = input.engagedGear;
    if (engaged < 0 || engaged >= gearCount || !std::isfinite(input.engineRpm)
        || !std::isfinite(input.wheelRadPerSecond))
        return engaged;
    const auto up = upshiftRpm(input);
    const auto landingCeiling = downshiftLandingRpm(input);
    const auto lugging = luggingRpm(input);
    const auto braking = std::isfinite(input.brake) && input.brake > 0.02;
    // An upshift lands clear of the lugging speed, and leaves the engine
    // above the downshift line (or the next frame would change back down);
    // braking never changes up.
    constexpr double upshiftLandingMargin = 1.15;
    if (!braking && input.engineRpm >= up && engaged + 1 < gearCount
        && input.engineRpm > landingCeiling * 1.05
        && landingRpm(input, engaged + 1) >= lugging * upshiftLandingMargin)
        return engaged + 1;
    // A downshift lands under the upshift point with a margin (under the
    // full-throttle one while braking, which never changes up); it carries
    // on down while the next gear also lands under the line, or while the
    // gear reached still lugs.
    constexpr double downshiftLandingMargin = 0.9;
    const auto overRev = downshiftLandingMargin * (braking ? fullThrottleUpshift(input) : up);
    auto gear = engaged;
    auto rpm = input.engineRpm;
    while (gear > 0) {
        const auto lower = landingRpm(input, gear - 1);
        if (lower > overRev || (lower > landingCeiling && rpm > lugging)) break;
        --gear;
        rpm = lower;
    }
    return gear;
}

} // namespace enginelab
