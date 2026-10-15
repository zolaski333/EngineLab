#include <enginelab/runtime/VariatorControl.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace enginelab {

namespace {
// The full ratio range in this time, either way. Production belt CVTs change
// ratio in about a second or two; the figure is not measured.
constexpr double fullRangeSeconds = 1.5;

double finiteOr(double value, double fallback) noexcept { return std::isfinite(value) ? value : fallback; }
} // namespace

double VariatorControl::targetEngineRpm(const VariatorInput& input) noexcept {
    const auto idle = std::max(300.0, finiteOr(input.idleRpm, 800.0));
    const auto limit = input.revLimitRpm > 0.0 && std::isfinite(input.revLimitRpm) ? input.revLimitRpm : 6'500.0;
    // Foot-light it holds the engine just off idle (a car's CVT cruises at
    // 1,500-2,000 rpm), never under a centrifugal clutch's lock speed; flat
    // out near peak power, under the limiter. Along pedal^1.5 between, as
    // the automatic gearbox's schedule (a choice, not a fit).
    const auto lock = std::max(0.0, finiteOr(input.centrifugalLockRpm, 0.0));
    const auto floor = std::max({ 1.5 * idle, 0.25 * limit, 1.05 * lock });
    const auto ceiling = std::max(floor, 0.85 * limit);
    const auto pedal = std::clamp(finiteOr(input.driverThrottle, 0.0), 0.0, 1.0);
    return floor + (ceiling - floor) * pedal * std::sqrt(pedal);
}

double VariatorControl::nextRatio(const VariatorInput& input, double dtSeconds) noexcept {
    const auto low = std::max(input.lowestRatio, input.tallestRatio);
    const auto tall = std::min(input.lowestRatio, input.tallestRatio);
    if (!(tall > 0.0) || !std::isfinite(low)) return std::isfinite(low) && low > 0.0 ? low : 1.0;
    const auto current = std::clamp(finiteOr(input.currentRatio, low), tall, low);
    if (low / tall < 1.0 + 1.0e-9) return low;
    auto desired = low;
    const auto output = finiteOr(input.outputRadPerSecond, 0.0);
    if (input.coupled && output > 1.0e-3) {
        const auto targetOmega = targetEngineRpm(input) * 2.0 * std::numbers::pi / 60.0;
        desired = std::clamp(targetOmega / output, tall, low);
    }
    const auto dt = std::clamp(finiteOr(dtSeconds, 0.0), 0.0, 0.05);
    const auto maximumLogStep = std::log(low / tall) / fullRangeSeconds * dt;
    const auto logStep = std::clamp(std::log(desired / current), -maximumLogStep, maximumLogStep);
    return std::clamp(current * std::exp(logStep), tall, low);
}

double VariatorControl::centrifugalEngageRpm(double engageRpm, double idleRpm) noexcept {
    // Scooter clutches bite well above idle so the engine idles free: some
    // 3,000 rpm on a 1,700 rpm idle.
    const auto idle = std::max(300.0, finiteOr(idleRpm, 800.0));
    return engageRpm > 0.0 && std::isfinite(engageRpm) ? std::max(engageRpm, 1.05 * idle) : 1.7 * idle;
}

double VariatorControl::centrifugalLockRpm(double engageRpm, double idleRpm) noexcept {
    return 1.35 * centrifugalEngageRpm(engageRpm, idleRpm);
}

double VariatorControl::centrifugalClutchPressure(double engineRpm, double engageRpm, double idleRpm) noexcept {
    const auto engage = centrifugalEngageRpm(engageRpm, idleRpm);
    const auto lock = centrifugalLockRpm(engageRpm, idleRpm);
    // The shoes' pressure grows with the square of the speed past the
    // spring's preload; a smooth step between bite and lock stands for it.
    const auto x = std::clamp((finiteOr(engineRpm, 0.0) - engage) / (lock - engage), 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
}

} // namespace enginelab
