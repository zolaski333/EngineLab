#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/calibration/EcuCalibration.hpp>
#include <enginelab/calibration/EcuCalibrationKeys.hpp>
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

SimpleEcuModel::SimpleEcuModel()
    : SimpleEcuModel(std::make_shared<calibration::CalibrationStore>()) {}

SimpleEcuModel::SimpleEcuModel(
    std::shared_ptr<calibration::CalibrationStore> calibrations)
    : calibrations_(calibrations ? std::move(calibrations)
                                 : std::make_shared<calibration::CalibrationStore>()),
      calibrationReader_(calibrations_->registerReader()),
      frameCalibration_(calibrations_->snapshot()) {}

void SimpleEcuModel::initialiseCalibration(const EngineConfig& config) {
    if (calibrations_->snapshot()->revision() == 0) {
        (void)calibrations_->publish(calibration::makeDefaultEcuCalibration(config),
                                     { 0, "engine-default" });
    }
    beginFrame();
}

void SimpleEcuModel::beginFrame() noexcept {
    auto next = calibrations_->snapshot();
    frameCalibration_ = std::move(next);
    calibrationReader_->acknowledge(frameCalibration_->revision());
}

EcuCommand SimpleEcuModel::evaluate(const EngineConfig& config, const EngineState& state,
                                    const EngineControls& controls) const noexcept {
    const auto& calibrationSnapshot = frameCalibration_;
    const calibration::AxisCoordinate rpmCoordinate {
        calibration::AxisQuantity::engineSpeed,
        calibration::CalibrationUnit::revolutionsPerMinute,
        state.rpm
    };
    const auto normalizedLoad = std::clamp(
        state.manifoldPressureKpa / std::max(1.0, config.ambientPressureKpa),
        calibration::ecuLimits::minimumNormalizedLoad,
        calibration::ecuLimits::maximumNormalizedLoad);
    const calibration::AxisCoordinate loadCoordinate {
        calibration::AxisQuantity::normalizedLoad,
        calibration::CalibrationUnit::ratio,
        normalizedLoad
    };
    const auto calibratedRevLimit = calibrationSnapshot->scalarOr(
        calibration::keys::revLimit, config.ignition.revLimitRpm);
    const auto revLimit = std::clamp(calibratedRevLimit,
        calibration::ecuLimits::minimumRevLimitRpm,
        calibration::ecuLimits::maximumRevLimitRpm);
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

    // The driver's throttle and the idle-air actuator are separate physical
    // controls.  The former must be able to close completely; using a minimum
    // throttle opening as an idle controller made every engine idle high and
    // made a released keyboard throttle remain physically open.
    const auto effectiveThrottle = std::clamp(controls.throttle, 0.0, 1.0);
    const auto previousIdleTime = previousIdleEvaluationTime_.exchange(
        state.simulationTimeSeconds, std::memory_order_relaxed);
    const auto idleDt = previousIdleTime > 0.0
        ? std::clamp(state.simulationTimeSeconds - previousIdleTime, 0.0, 0.02) : 0.0;
    auto idleIntegral = idleIntegral_.load(std::memory_order_relaxed);
    auto idleAirOpening = 0.0;
    if (controls.ignitionEnabled || controls.starterEngaged) {
        const auto targetRpm = std::max(300.0, config.idleRpm);
        const auto normalizedError = (targetRpm - state.rpm) / targetRpm;
        constexpr double feedForward = 0.42;
        constexpr double proportionalGain = 0.90;
        constexpr double integralGain = 0.55;
        const auto proposedIntegral = std::clamp(
            idleIntegral + normalizedError * integralGain * idleDt, -0.42, 0.58);
        const auto proposedCommand = feedForward
            + proportionalGain * normalizedError + proposedIntegral;
        // Conditional integration prevents wind-up when the actuator is on a
        // stop and the error would push it farther into saturation.
        if ((proposedCommand > 0.0 && proposedCommand < 1.0)
            || (proposedCommand <= 0.0 && normalizedError > 0.0)
            || (proposedCommand >= 1.0 && normalizedError < 0.0))
            idleIntegral = proposedIntegral;

        const auto crankingAir = state.rpm < 350.0 ? 0.88 : 0.0;
        const auto driverOverride = std::clamp(1.0 - effectiveThrottle * 12.0, 0.0, 1.0);
        idleAirOpening = std::max(crankingAir,
            std::clamp(feedForward + proportionalGain * normalizedError + idleIntegral,
                       0.0, 1.0) * driverOverride);
    } else {
        idleIntegral *= std::exp(-idleDt * 5.0);
    }
    idleIntegral_.store(idleIntegral, std::memory_order_relaxed);
    const auto warmupCorrection = std::clamp(1.0 + (70.0 - state.coolantTemperatureC) * 0.0025, 1.0, 1.12);
    const auto crankingCorrection = controls.starterEngaged
        ? 1.0 + std::clamp((700.0 - state.rpm) / 700.0, 0.0, 1.0) * 0.38 : 1.0;
    constexpr CalibrationTable afrCorrection {{
        {{ 0.0, 0.0, -0.3 }}, {{ 0.7, 0.1, -1.0 }},
        {{ 0.9, 0.0, -1.5 }}, {{ 0.6, -0.2, -1.7 }} }};
    constexpr CalibrationTable advanceCorrection {{
        {{ -5.0, -5.0, -6.0 }}, {{ 5.0, 2.0, -1.0 }},
        {{ 9.0, 5.0, 1.0 }}, {{ 11.0, 6.0, 0.0 }} }};
    const auto legacyAfr = calibration::ecuLimits::referenceAirFuelRatio
        + afrTrim_.load(std::memory_order_relaxed)
        + interpolate(afrCorrection, state.rpm, normalizedLoad);
    auto calibratedAfr = calibrationSnapshot->sampleTable(
        calibration::keys::targetAirFuelRatio, rpmCoordinate, loadCoordinate);
    if (!calibratedAfr)
        calibratedAfr = calibrationSnapshot->sampleCurve(
            calibration::keys::targetAirFuelRatio, rpmCoordinate);
    // The lightweight live control is an explicit trim around the active map.
    const auto afrCommand = calibratedAfr
        ? *calibratedAfr + afrTrim_.load(std::memory_order_relaxed)
        : legacyAfr;
    auto mappedAfr = std::clamp(afrCommand,
        calibration::ecuLimits::minimumAirFuelRatio,
        calibration::ecuLimits::maximumAirFuelRatio);
    const auto configuredAdvance = interpolateTimingCurve(config.ignition.timingCurve, state.rpm);
    const auto liveTrim = ignitionTrimDegrees_.load(std::memory_order_relaxed);
    const auto legacyAdvance = configuredAdvance
        + interpolate(advanceCorrection, state.rpm, normalizedLoad);
    auto calibratedAdvance = calibrationSnapshot->sampleTable(
        calibration::keys::ignitionAdvance, rpmCoordinate, loadCoordinate);
    if (!calibratedAdvance)
        calibratedAdvance = calibrationSnapshot->sampleCurve(
            calibration::keys::ignitionAdvance, rpmCoordinate);
    auto mappedAdvance = std::clamp(calibratedAdvance.value_or(legacyAdvance) + liveTrim,
        calibration::ecuLimits::minimumIgnitionAdvanceDegrees,
        calibration::ecuLimits::maximumIgnitionAdvanceDegrees);
    const auto previousThrottle = previousThrottle_.exchange(effectiveThrottle, std::memory_order_relaxed);
    const auto accelerationEnrichment = std::clamp(effectiveThrottle - previousThrottle, 0.0, 0.35);
    mappedAfr = std::clamp(mappedAfr - accelerationEnrichment * 2.2
        - std::max(0.0, state.coolantTemperatureC - 108.0) * 0.025,
        calibration::ecuLimits::minimumAirFuelRatio,
        calibration::ecuLimits::maximumAirFuelRatio);
    mappedAdvance = std::clamp(mappedAdvance - state.knockLevel * 12.0
        - std::max(0.0, state.coolantTemperatureC - 108.0) * 0.20,
        calibration::ecuLimits::minimumIgnitionAdvanceDegrees,
        calibration::ecuLimits::maximumIgnitionAdvanceDegrees);
    const auto softLimit = state.rpm > revLimit - 220.0;
    const auto alternatingCut = softLimit && (static_cast<std::uint64_t>(state.simulationTimeSeconds * 120.0) & 1U) != 0U;
    const auto enabled = controls.ignitionEnabled && !limiterActive;
    return { mappedAfr, mappedAdvance,
             effectiveThrottle, idleAirOpening,
             warmupCorrection * crankingCorrection, enabled,
             enabled && !alternatingCut };
}
} // namespace enginelab
