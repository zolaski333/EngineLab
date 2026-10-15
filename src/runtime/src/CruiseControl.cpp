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
// Cancels when the road speed falls this far under the set speed (a climb the
// engine cannot hold in this gear): a real cruise control gives up too.
constexpr double cancelBelowFraction = 0.6;
// Past the set speed and still gaining (or under it and still losing), the
// throttle is surely wrong: it moves this fast per second (full range in 2 s),
// whatever the integral says: a set pressed while accelerating hard needs it.
constexpr double correctionRatePerSecond = 0.5;
constexpr double correctionDeadbandMps = 0.3 / 3.6;
constexpr double correctionAccelerationMps2 = 0.05;
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
    const auto acceleration = dt > 0.0 ? (speedMps - previousSpeedMps_) / dt : 0.0;
    previousSpeedMps_ = speedMps;
    if (dt > 0.0 && driver.throttle <= integral_) {
        if (error < -correctionDeadbandMps && acceleration > correctionAccelerationMps2)
            integral_ = std::max(0.0, integral_ - correctionRatePerSecond * dt);
        else if (error > correctionDeadbandMps && acceleration < -correctionAccelerationMps2)
            integral_ = std::min(1.0, integral_ + correctionRatePerSecond * dt);
    }
    const auto proportional = proportionalGain * error;
    const auto command = std::clamp(proportional + integral_, 0.0, 1.0);
    // The driver's pedal overtakes: hold the integral meanwhile, so letting
    // go returns to the throttle that held the speed. Otherwise integrate
    // only while that does not push further into saturation.
    const auto overtaken = driver.throttle > command;
    const auto saturatedHigh = proportional + integral_ >= 1.0 && error > 0.0;
    const auto saturatedLow = proportional + integral_ <= 0.0 && error < 0.0;
    if (!overtaken && !saturatedHigh && !saturatedLow && dt > 0.0)
        integral_ = std::clamp(integral_ + integralGain * error * dt, 0.0, 1.0);
    driver.throttle = std::max(driver.throttle, command);
    return driver;
}

} // namespace enginelab
