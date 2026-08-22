#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace enginelab {

struct TurboExhaustFlowSplit final {
    double turbineKgPerSecond {};
    double wastegateKgPerSecond {};
};

/** One explicit energy step of the reduced turbocharger shaft model.
 *
 * `compressorPressureRatio` is the head available at the END of the step from
 * the centrifugal similarity law. `compressorPowerWatts` is the work actually
 * removed during the step at the measured compressor outlet pressure. Keeping
 * those two quantities separate preserves causality without hiding a block of
 * shaft energy in a pressure clamp.
 */
struct TurboShaftAdvance final {
    double angularSpeedRadPerSecond {};
    double compressorPressureRatio { 1.0 };
    double compressorPowerWatts {};
    double bearingPowerWatts {};
    double netPowerWatts {};
    double energyBeforeJoules {};
    double energyAfterJoules {};
};

inline constexpr double turboAirCpJPerKgK = 1'005.0;
inline constexpr double turboIsentropicExponent = 0.285714285714;

/** Centrifugal-compressor head from the affinity law.
 *
 * This is deliberately an extrapolatable reduced-order law, not a hidden
 * compressor map. At the authored design shaft speed it returns the authored
 * design pressure ratio; shaft-speed ratio remains separately observable so a
 * caller can identify operation beyond that design point.
 */
[[nodiscard]] inline double compressorPressureRatioFromSpeed(
    const ForcedInductionConfig& config,
    double angularSpeedRadPerSecond) noexcept {
    const auto designOmega = config.designShaftSpeedRpm
        * 2.0 * std::numbers::pi / 60.0;
    const auto speedRatio = std::max(0.0, angularSpeedRadPerSecond)
        / std::max(1.0, designOmega);
    return 1.0 + std::max(0.0, config.pressureRatio - 1.0)
        * speedRatio * speedRatio;
}

/** Isentropic compressor work at the measured through-flow and pressure. */
[[nodiscard]] inline double compressorPowerForPressureRatioWatts(
    const ForcedInductionConfig& config,
    double massFlowKgPerSecond,
    double inletTemperatureK,
    double pressureRatio) noexcept {
    return std::max(0.0, massFlowKgPerSecond) * turboAirCpJPerKgK
        * std::max(1.0, inletTemperatureK)
        * (std::pow(std::max(1.0, pressureRatio),
             turboIsentropicExponent) - 1.0)
        / std::max(0.35, config.compressorEfficiency);
}

/** Advance kinetic shaft energy without an undocumented speed sink.
 *
 * The update is exact in energy for zero-order-held powers:
 *
 *   E[n+1] = max(0, E[n] + (P_t - P_c - P_b) dt)
 *
 * A previous implementation clamped speed after this equation and silently
 * discarded the excess energy. Here every watt is either compressor work,
 * authored bearing loss, or retained shaft kinetic energy. Wastegate control
 * remains the physical means of reducing turbine input power.
 */
[[nodiscard]] inline TurboShaftAdvance advanceTurboShaft(
    const ForcedInductionConfig& config,
    double angularSpeedRadPerSecond,
    double turbinePowerWatts,
    double compressorMassFlowKgPerSecond,
    double inletTemperatureK,
    double measuredPressureRatio,
    double dtSeconds) noexcept {
    TurboShaftAdvance result;
    const auto omega = std::max(0.0, angularSpeedRadPerSecond);
    const auto inertia = std::max(1.0e-12, config.shaftInertiaKgM2);
    const auto designOmega = config.designShaftSpeedRpm
        * 2.0 * std::numbers::pi / 60.0;
    const auto speedRatio = omega / std::max(1.0, designOmega);
    const auto requestedCompressorPowerWatts =
        compressorPowerForPressureRatioWatts(
            config, compressorMassFlowKgPerSecond, inletTemperatureK,
            measuredPressureRatio);
    const auto requestedBearingPowerWatts = std::max(0.0,
        config.bearingFrictionPowerWatts) * speedRatio * speedRatio;
    result.energyBeforeJoules = 0.5 * inertia * omega * omega;
    const auto turbinePower = std::max(0.0, turbinePowerWatts);
    const auto dt = std::max(0.0, dtSeconds);
    auto deliveredLossScale = 1.0;
    const auto requestedLossPower = requestedCompressorPowerWatts
        + requestedBearingPowerWatts;
    if (dt > 0.0 && requestedLossPower > 0.0) {
        // A stopped shaft cannot deliver more work than the turbine supplies
        // plus the kinetic energy available during this step. Scale both
        // zero-order-held loss channels together if the requested work would
        // otherwise drive kinetic energy negative.
        const auto availableLossPower = turbinePower
            + result.energyBeforeJoules / dt;
        deliveredLossScale = std::min(
            1.0, availableLossPower / requestedLossPower);
    }
    result.compressorPowerWatts = requestedCompressorPowerWatts
        * deliveredLossScale;
    result.bearingPowerWatts = requestedBearingPowerWatts
        * deliveredLossScale;
    result.netPowerWatts = turbinePower - result.compressorPowerWatts
        - result.bearingPowerWatts;
    result.energyAfterJoules = result.energyBeforeJoules
        + result.netPowerWatts * dt;
    // Only floating-point roundoff may put a conservative stopped shaft a few
    // ulps below zero; there is no positive-energy ceiling here.
    result.energyAfterJoules = std::max(0.0, result.energyAfterJoules);
    result.angularSpeedRadPerSecond = std::sqrt(
        2.0 * result.energyAfterJoules / inertia);
    result.compressorPressureRatio = compressorPressureRatioFromSpeed(
        config, result.angularSpeedRadPerSecond);
    return result;
}

/** Conservatively partitions the measured exhaust flow through the parallel
 *  effective turbine and wastegate throats. The same split drives shaft power
 *  and aeroacoustics so neither subsystem can create or discard bypass flow. */
[[nodiscard]] inline TurboExhaustFlowSplit partitionTurboExhaustFlow(
    const ForcedInductionConfig& config, double totalKgPerSecond,
    double wastegateOpening) noexcept {
    const auto total = std::max(0.0, totalKgPerSecond);
    const auto turbineArea = std::max(0.0, config.turbineFlowAreaMm2);
    const auto wastegateArea = std::max(0.0, config.wastegateFlowAreaMm2)
        * std::clamp(wastegateOpening, 0.0, 1.0);
    const auto effectiveArea = turbineArea + wastegateArea;
    if (!(effectiveArea > 0.0)) return { total, 0.0 };
    const auto bypass = total * wastegateArea / effectiveArea;
    return { total - bypass, bypass };
}

} // namespace enginelab
