#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <algorithm>
#include <array>
#include <cmath>
namespace enginelab {
namespace {
constexpr std::array<double, 4> rpmAxis { 0.0, 2'000.0, 4'500.0, 8'000.0 };
constexpr std::array<double, 3> loadAxis { 0.0, 0.5, 1.0 };
using CalibrationTable = std::array<std::array<double, loadAxis.size()>, rpmAxis.size()>;

double interpolate(const CalibrationTable& table, double rpm, double load) noexcept {
    const auto upperRpm = std::upper_bound(rpmAxis.begin(), rpmAxis.end(), rpm);
    const auto upperLoad = std::upper_bound(loadAxis.begin(), loadAxis.end(), load);
    const auto r1 = static_cast<std::size_t>(std::clamp<std::ptrdiff_t>(upperRpm - rpmAxis.begin(), 1, rpmAxis.size() - 1));
    const auto l1 = static_cast<std::size_t>(std::clamp<std::ptrdiff_t>(upperLoad - loadAxis.begin(), 1, loadAxis.size() - 1));
    const auto r0 = r1 - 1; const auto l0 = l1 - 1;
    const auto rt = std::clamp((rpm - rpmAxis[r0]) / (rpmAxis[r1] - rpmAxis[r0]), 0.0, 1.0);
    const auto lt = std::clamp((load - loadAxis[l0]) / (loadAxis[l1] - loadAxis[l0]), 0.0, 1.0);
    const auto low = std::lerp(table[r0][l0], table[r0][l1], lt);
    const auto high = std::lerp(table[r1][l0], table[r1][l1], lt);
    return std::lerp(low, high, rt);
}

double interpolateTimingCurve(const std::vector<IgnitionMapSample>& curve, double rpm) noexcept {
    if (curve.empty()) return 18.0;
    if (curve.size() == 1 || rpm <= curve.front().rpm) return curve.front().advanceDegrees;
    for (std::size_t index = 1; index < curve.size(); ++index) {
        if (rpm <= curve[index].rpm) {
            const auto span = std::max(1.0, curve[index].rpm - curve[index - 1].rpm);
            const auto t = std::clamp((rpm - curve[index - 1].rpm) / span, 0.0, 1.0);
            return std::lerp(curve[index - 1].advanceDegrees, curve[index].advanceDegrees, t);
        }
    }
    return curve.back().advanceDegrees;
}
}

EcuCommand SimpleEcuModel::evaluate(const EngineConfig& config, const EngineState& state,
                                    const EngineControls& controls) const noexcept {
    const auto revLimit = std::clamp(config.ignition.revLimitRpm, config.idleRpm + 100.0, 25'000.0);
    auto limiterActive = limiterLatched_.load(std::memory_order_relaxed);
    if (state.rpm >= revLimit) {
        limiterActive = true;
        limiterReleaseTime_.store(state.simulationTimeSeconds + config.ignition.limiterDurationSeconds,
                                  std::memory_order_relaxed);
    } else if (state.rpm < revLimit - 180.0
               && state.simulationTimeSeconds >= limiterReleaseTime_.load(std::memory_order_relaxed)) {
        limiterActive = false;
    }
    limiterLatched_.store(limiterActive, std::memory_order_relaxed);

    const auto idleError = std::max(0.0, config.idleRpm - state.rpm);
    const auto idleThrottle = state.rpm >= 250.0
        ? std::clamp(0.055 + idleError / std::max(1'500.0, config.idleRpm * 3.0), 0.0, 0.28)
        : 0.0;
    const auto effectiveThrottle = std::max(std::clamp(controls.throttle, 0.0, 1.0), idleThrottle);
    const auto warmupCorrection = std::clamp(1.0 + (70.0 - state.coolantTemperatureC) * 0.0025, 1.0, 1.12);
    const auto crankingCorrection = controls.starterEngaged
        ? 1.0 + std::clamp((700.0 - state.rpm) / 700.0, 0.0, 1.0) * 0.38 : 1.0;
    const auto normalizedLoad = std::clamp(state.manifoldPressureKpa / std::max(1.0, config.ambientPressureKpa), 0.0, 1.0);
    constexpr CalibrationTable afrCorrection {{
        {{ 0.0, 0.0, -0.3 }}, {{ 0.7, 0.1, -1.0 }},
        {{ 0.9, 0.0, -1.5 }}, {{ 0.6, -0.2, -1.7 }} }};
    constexpr CalibrationTable advanceCorrection {{
        {{ -5.0, -5.0, -6.0 }}, {{ 5.0, 2.0, -1.0 }},
        {{ 9.0, 5.0, 1.0 }}, {{ 11.0, 6.0, 0.0 }} }};
    auto mappedAfr = std::clamp(targetAfr_.load(std::memory_order_relaxed)
        + interpolate(afrCorrection, state.rpm, normalizedLoad), 10.5, 18.0);
    const auto configuredAdvance = interpolateTimingCurve(config.ignition.timingCurve, state.rpm);
    const auto liveTrim = ignitionTrimDegrees_.load(std::memory_order_relaxed);
    auto mappedAdvance = std::clamp(configuredAdvance + liveTrim
        + interpolate(advanceCorrection, state.rpm, normalizedLoad), -10.0, 55.0);
    const auto previousThrottle = previousThrottle_.exchange(effectiveThrottle, std::memory_order_relaxed);
    const auto accelerationEnrichment = std::clamp(effectiveThrottle - previousThrottle, 0.0, 0.35);
    mappedAfr = std::clamp(mappedAfr - accelerationEnrichment * 2.2
        - std::max(0.0, state.coolantTemperatureC - 108.0) * 0.025, 10.5, 18.0);
    mappedAdvance = std::clamp(mappedAdvance - state.knockLevel * 12.0
        - std::max(0.0, state.coolantTemperatureC - 108.0) * 0.20, -10.0, 55.0);
    const auto softLimit = state.rpm > revLimit - 220.0;
    const auto alternatingCut = softLimit && (static_cast<std::uint64_t>(state.simulationTimeSeconds * 120.0) & 1U) != 0U;
    const auto enabled = controls.ignitionEnabled && !limiterActive;
    return { mappedAfr, mappedAdvance,
             effectiveThrottle, warmupCorrection * crankingCorrection, enabled,
             enabled && !alternatingCut };
}
} // namespace enginelab
