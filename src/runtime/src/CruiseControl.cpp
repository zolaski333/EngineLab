#include <enginelab/runtime/CruiseControl.hpp>

#include <algorithm>
#include <cmath>

namespace enginelab {
namespace {
// Throttle per m/s of speed error, and its integral per second. A car needs
// some 10-25 % throttle to hold 100 km/h on the flat, a motorcycle less: the
// integral finds it, the proportional term answers a disturbance within a
// second or two without hunting.
constexpr double proportionalGain = 0.08;
constexpr double integralGain = 0.03;
// While the speed returns to the set one faster than this, the proportional
// term is bringing it back and the integral holds.
constexpr double returningAccelerationMps2 = 0.1;
// Cancels when the road speed falls this far under the set speed (a climb the
// engine cannot hold in this gear): a real cruise control gives up too.
constexpr double cancelBelowFraction = 0.6;
// A set pressed while accelerating hard starts the integral at that pull's
// throttle, far above the one that holds the speed (the PI alone overshot
// 15-19 km/h). Until the vehicle stops gaining (or losing), the integral
// moves against the acceleration, in proportion to it; then the PI holds
// alone. This used to act whenever the speed was 0.3 km/h off and the raw
// frame-to-frame acceleration pointed away, at a fixed 0.5 a second: the
// firing ripple crossed that all the time, and the throttle swung 0-20 %
// every 2.5 s (owner report 2026-11-05: a K20A at 100 km/h, +-0.9 km/h,
// +-45 rpm; a CP2 and a Hayabusa +-3-5 km/h once it acted beyond 1 km/h).
constexpr double settlingPerAcceleration = 0.6;
// Below this the vehicle is no longer gaining or losing.
constexpr double settledAccelerationMps2 = 0.1;
// The acceleration is filtered over this time: one frame's speed step is
// mostly the crank's firing ripple.
constexpr double accelerationFilterSeconds = 0.3;
} // namespace

bool CruiseControl::engage(double speedMps, double currentThrottle) noexcept {
    if (!std::isfinite(speedMps) || speedMps < minimumSpeedMps) {
        cancel();
        return false;
    }
    engaged_ = true;
    setSpeedMps_ = speedMps;
    integral_ = std::isfinite(currentThrottle) ? std::clamp(currentThrottle, 0.0, 1.0) : 0.0;
    previousSpeedMps_ = speedMps;
    filteredAccelerationMps2_ = 0.0;
    settlingSeconds_ = 0.0;
    settling_ = true;
    settlingSign_ = 0.0;
    return true;
}

void CruiseControl::cancel() noexcept {
    engaged_ = false;
    integral_ = 0.0;
}

DriverInputs CruiseControl::apply(double dt, double speedMps, DriverInputs driver, bool inGear) noexcept {
    if (!engaged_) return driver;
    const auto clutchPressed = driver.clutchPedal < 0.5;
    if (driver.brake > 0.05 || clutchPressed || !inGear || !std::isfinite(speedMps)
        || speedMps < cancelBelowFraction * setSpeedMps_) {
        cancel();
        return driver;
    }
    const auto error = setSpeedMps_ - speedMps;
    if (dt > 0.0) {
        const auto acceleration = (speedMps - previousSpeedMps_) / dt;
        filteredAccelerationMps2_ += (acceleration - filteredAccelerationMps2_)
            * std::min(1.0, dt / accelerationFilterSeconds);
    }
    previousSpeedMps_ = speedMps;
    const auto acceleration = filteredAccelerationMps2_;
    if (settling_ && dt > 0.0) {
        settlingSeconds_ += dt;
        // The filter has seen the pull once its time has passed: the
        // settling ends when the acceleration has died or turned.
        if (settlingSeconds_ >= accelerationFilterSeconds) {
            if (settlingSign_ == 0.0)
                settlingSign_ = std::abs(acceleration) < settledAccelerationMps2 ? 0.0 : std::copysign(1.0, acceleration);
            if (settlingSign_ == 0.0 || acceleration * settlingSign_ < settledAccelerationMps2) settling_ = false;
        }
        // The driver's pedal overtaking ends it: that acceleration is the
        // driver's, and the integral holds for the release.
        if (driver.throttle > integral_) settling_ = false;
        if (settling_)
            integral_ = std::clamp(integral_ - settlingPerAcceleration * acceleration * dt, 0.0, 1.0);
    }
    const auto proportional = proportionalGain * error;
    const auto command = std::clamp(proportional + integral_, 0.0, 1.0);
    // The driver's pedal overtakes: hold the integral meanwhile, so letting
    // go returns to the throttle that held the speed. Otherwise integrate
    // only while that does not push further into saturation.
    const auto overtaken = driver.throttle > command;
    const auto saturatedHigh = proportional + integral_ >= 1.0 && error > 0.0;
    const auto saturatedLow = proportional + integral_ <= 0.0 && error < 0.0;
    // Already on the way back (the driver let go of the pedal, the top of a
    // climb): the integral that held the speed is right, and winding it on
    // during the return carries the speed past the set one (2.4 km/h under
    // after a pedal overtake, then a slow tail).
    const auto returning = error * acceleration > 0.0 && std::abs(acceleration) > returningAccelerationMps2;
    if (!overtaken && !saturatedHigh && !saturatedLow && !returning && dt > 0.0)
        integral_ = std::clamp(integral_ + integralGain * error * dt, 0.0, 1.0);
    driver.throttle = std::max(driver.throttle, command);
    return driver;
}

} // namespace enginelab
