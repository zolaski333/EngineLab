#include <enginelab/render/CrankClock.hpp>

#include <algorithm>
#include <cmath>

namespace enginelab::render {
namespace {
/** Shortest signed difference on the 720-degree cycle, in [-360, 360). */
[[nodiscard]] double signedCycleDifference(double degrees) noexcept {
    return wrapCycleDegrees(degrees + 360.0) - 360.0;
}

/** Time constant of the pull towards the simulator's angle. Short enough to
    absorb a speed change within a few frames, long enough to hide the 30 Hz
    sampling. */
constexpr double lockTimeConstantSeconds = 0.08;
/** Beyond this error the clock jumps instead of sweeping the crank round. */
constexpr double snapThresholdDegrees = 90.0;
} // namespace

double wrapCycleDegrees(double degrees) noexcept {
    const auto wrapped = std::fmod(degrees, 720.0);
    return wrapped < 0.0 ? wrapped + 720.0 : wrapped;
}

void CrankClock::observe(double wallSeconds, double crankAngleDegrees, double rpm, double rate) noexcept {
    // Extrapolating with the speed alone lags a hard rev by a third of a
    // sample period; the measured acceleration removes that lag.
    const auto interval = wallSeconds - sampleWall_;
    rpmPerSecond_ = hasSample_ && interval > 1.0e-3 && interval < 0.5
        ? std::clamp((std::max(0.0, rpm) - rpm_) / interval, -50'000.0, 50'000.0) : 0.0;
    hasSample_ = true;
    sampleWall_ = wallSeconds;
    sampleAngle_ = wrapCycleDegrees(crankAngleDegrees);
    rpm_ = std::max(0.0, rpm);
    rate_ = std::max(0.0, rate);
}

void CrankClock::setPlaybackFactor(double factor) noexcept {
    factor_ = std::clamp(factor, 0.0, 1.0);
}

double CrankClock::rpmAt(double wallSeconds) const noexcept {
    // Never extrapolate the acceleration further than one sample period.
    const auto elapsed = std::clamp(wallSeconds - sampleWall_, 0.0, 0.05);
    return std::max(0.0, rpm_ + rpmPerSecond_ * elapsed);
}

double CrankClock::displayDegreesPerSecond() const noexcept {
    return rpmAt(displayWall_) * 6.0 * rate_ * (realTime() ? 1.0 : factor_);
}

double CrankClock::advance(double wallSeconds) noexcept {
    if (!hasSample_) return displayAngle_;
    const auto elapsed = std::clamp(wallSeconds - sampleWall_, 0.0, 0.05);
    const auto target = sampleAngle_ + 6.0 * rate_ * ((rpm_ + 0.5 * rpmPerSecond_ * elapsed) * elapsed
        + rpmAt(wallSeconds) * std::max(0.0, wallSeconds - sampleWall_ - elapsed));
    if (!hasDisplay_) {
        hasDisplay_ = true;
        displayWall_ = wallSeconds;
        displayAngle_ = wrapCycleDegrees(target);
        return displayAngle_;
    }
    // A long stall (window minimised, debugger) must not fling the crank.
    const auto dt = std::clamp(wallSeconds - displayWall_, 0.0, 0.25);
    // Mean speed over the step (trapezoid), so an accelerating crank is not
    // drawn one step behind.
    const auto startRate = displayDegreesPerSecond();
    displayWall_ = wallSeconds;
    auto angle = displayAngle_ + 0.5 * (startRate + displayDegreesPerSecond()) * dt;
    if (realTime()) {
        const auto error = signedCycleDifference(target - angle);
        angle = std::abs(error) > snapThresholdDegrees
            ? target : angle + error * (1.0 - std::exp(-dt / lockTimeConstantSeconds));
    }
    displayAngle_ = wrapCycleDegrees(angle);
    return displayAngle_;
}

void CrankClock::reset() noexcept {
    *this = CrankClock {};
}

} // namespace enginelab::render
