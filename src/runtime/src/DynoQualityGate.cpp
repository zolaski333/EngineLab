#include <enginelab/runtime/DynoQualityGate.hpp>

#include <algorithm>
#include <cmath>

namespace enginelab {
namespace {

double finitePositiveOr(double value, double fallback) noexcept {
    return std::isfinite(value) && value > 0.0 ? value : fallback;
}

} // namespace

DynoQualityGate::DynoQualityGate(DynoQualityGateConfig config) noexcept {
    config_.minimumSteadyContactFraction = std::clamp(
        std::isfinite(config.minimumSteadyContactFraction)
            ? config.minimumSteadyContactFraction : 0.95,
        0.0, 1.0);
    config_.minimumRampContactFraction = std::clamp(
        std::isfinite(config.minimumRampContactFraction)
            ? config.minimumRampContactFraction : 0.0,
        0.0, 1.0);
    config_.minimumEngineTorqueNm = finitePositiveOr(
        config.minimumEngineTorqueNm, 1.4);
    config_.steadyMaximumSpeedErrorRpm = finitePositiveOr(
        config.steadyMaximumSpeedErrorRpm, 60.0);
    config_.rampMaximumSpeedErrorRpm = finitePositiveOr(
        config.rampMaximumSpeedErrorRpm, 150.0);
    config_.steadyMaximumAccelerationRpmPerSecond = finitePositiveOr(
        config.steadyMaximumAccelerationRpmPerSecond, 120.0);
    config_.rampAccelerationMultiplier = finitePositiveOr(
        config.rampAccelerationMultiplier, 3.0);
}

DynoQualityGateResult DynoQualityGate::evaluate(
    const DynoQualityGateInput& input,
    const EngineState& engineState,
    const DynoAbsorberOutput& absorber) const noexcept {
    DynoQualityGateResult result;
    const auto ramp = input.mode == DynoAcquisitionMode::continuousRamp;
    result.maximumAllowedSpeedErrorRpm = ramp
        ? config_.rampMaximumSpeedErrorRpm
        : config_.steadyMaximumSpeedErrorRpm;
    result.maximumAllowedAccelerationRpmPerSecond = ramp
        ? std::max(config_.steadyMaximumAccelerationRpmPerSecond,
            config_.rampAccelerationMultiplier
                * std::abs(input.rampRateRpmPerSecond))
        : config_.steadyMaximumAccelerationRpmPerSecond;

    const auto numericalInputsFinite =
        std::isfinite(input.targetRpm)
        && std::isfinite(input.measuredRpm)
        && std::isfinite(input.measuredCycleTorqueNm)
        && (!ramp || std::isfinite(input.rampRateRpmPerSecond))
        && std::isfinite(engineState.rpm)
        && std::isfinite(absorber.brakeTorqueNm)
        && std::isfinite(absorber.filteredRpm)
        && std::isfinite(absorber.filteredAccelerationRpmPerSecond)
        && std::isfinite(absorber.contactFraction)
        && std::isfinite(absorber.unclampedBrakeTorqueNm);
    if (!numericalInputsFinite) {
        result.reasons |= DynoQualityReason::nonFinite;
    } else {
        result.speedErrorRpm = input.measuredRpm - input.targetRpm;
        const auto minimumContactFraction = ramp
            ? config_.minimumRampContactFraction
            : config_.minimumSteadyContactFraction;
        if (absorber.contactFraction + 1.0e-12
            < minimumContactFraction)
            result.reasons |= DynoQualityReason::noBrakeContact;
        if (!(input.measuredCycleTorqueNm
              > config_.minimumEngineTorqueNm))
            result.reasons |= DynoQualityReason::insufficientEngineTorque;
        if (std::abs(result.speedErrorRpm)
            > result.maximumAllowedSpeedErrorRpm + 1.0e-12)
            result.reasons |= DynoQualityReason::speedTrackingError;
        if (std::abs(absorber.filteredAccelerationRpmPerSecond)
            > result.maximumAllowedAccelerationRpmPerSecond + 1.0e-12)
            result.reasons |= DynoQualityReason::accelerationOutOfBounds;
    }

    if (!input.prepared)
        result.reasons |= DynoQualityReason::notPrepared;
    if (!input.protocolReady)
        result.reasons |= DynoQualityReason::protocolNotReady;
    if (input.recoveryActive)
        result.reasons |= DynoQualityReason::recoveryActive;
    if (!input.cycleContinuous)
        result.reasons |= DynoQualityReason::discontinuousCycle;
    if (absorber.saturatedHigh)
        result.reasons |= DynoQualityReason::absorberCapacityLimited;
    if (engineState.ecuSoftRevLimiterActive
        || engineState.ecuHardRevLimiterActive
        || engineState.ecuAlternatingSparkCutActive)
        result.reasons |= DynoQualityReason::revLimiterActive;

    // Low saturation means a passive absorber would have to motor the engine.
    // It is not a capacity overrun: a ramp may still be valid while the engine
    // accelerates under its own torque, provided tracking and acceleration are
    // within their explicit limits. High saturation is the metrological fault.
    return result;
}

} // namespace enginelab
