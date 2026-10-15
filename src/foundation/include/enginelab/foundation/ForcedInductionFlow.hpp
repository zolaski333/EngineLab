#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
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

/** Where a centrifugal compressor runs on its map. */
struct CompressorOperatingPoint final {
    double pressureRatio { 1.0 };
    double efficiency { 0.0 };
    /** Corrected shaft speed over the design speed. */
    double speedRatio { 0.0 };
    /** Flow coefficient over the design one: corrected flow over speed. */
    double flowCoefficientRatio { 0.0 };
    /** Left of the surge line: the real flow would reverse in bursts. */
    bool surge { false };
    /** Past the choke line: the wheel passes no more flow and adds no head. */
    bool choke { false };
};

inline constexpr double compressorReferenceTemperatureK = 298.15;
inline constexpr double compressorReferencePressureKpa = 101.325;
/** Choke and surge, as flow coefficients over the design one. */
inline constexpr double compressorChokeFlowCoefficientRatio = 1.5;
inline constexpr double compressorSurgeFlowCoefficientRatio = 0.4;

/** A generic compressor map scaled to the authored design point.
 *
 * A compressor family's speed lines collapse onto one curve of the head
 * coefficient against the flow coefficient (Moraal and Kolmanovsky, SAE
 * 1999-01-0908). Both are taken relative to the design point here: the head
 * ratio is (PR^k - 1) / ((PR_d^k - 1) N^2) and the flow ratio is the corrected
 * flow over the design flow, divided by the corrected speed ratio N. The curve
 * is psi = psi0 (1 - (phi / phi_c)^4): flat towards surge, falling steeply to
 * zero head at choke (phi_c = 1.5), through psi = 1 at the design point. At
 * the design flow and speed the map returns the design pressure ratio, as the
 * affinity law did; it now also falls with flow, as a real wheel's does.
 *
 * Efficiency is an island around the design point, the authored peak at its
 * centre: 6 points lost at half or one and a half the design flow ratio, 4 at
 * 60 % or 140 % of the design speed.
 */
[[nodiscard]] inline CompressorOperatingPoint compressorOperatingPoint(
    const ForcedInductionConfig& config, double designMassFlowKgPerSecond,
    double angularSpeedRadPerSecond, double massFlowKgPerSecond,
    double inletTemperatureK, double inletPressureKpa) noexcept {
    CompressorOperatingPoint point;
    const auto designOmega = std::max(1.0, config.designShaftSpeedRpm * 2.0 * std::numbers::pi / 60.0);
    const auto temperatureK = std::max(150.0, inletTemperatureK);
    const auto pressureKpa = std::max(1.0, inletPressureKpa);
    point.speedRatio = std::max(0.0, angularSpeedRadPerSecond) / designOmega
        * std::sqrt(compressorReferenceTemperatureK / temperatureK);
    const auto correctedFlowRatio = std::max(0.0, massFlowKgPerSecond)
        * std::sqrt(temperatureK / compressorReferenceTemperatureK)
        / (pressureKpa / compressorReferencePressureKpa)
        / std::max(1.0e-6, designMassFlowKgPerSecond);
    const auto peak = std::clamp(config.compressorEfficiency, 0.35, 0.90);
    if (point.speedRatio < 1.0e-6) {
        point.flowCoefficientRatio = correctedFlowRatio > 0.0 ? compressorChokeFlowCoefficientRatio : 0.0;
        point.choke = correctedFlowRatio > 0.0;
        point.efficiency = 0.35;
        return point;
    }
    const auto phi = correctedFlowRatio / point.speedRatio;
    point.flowCoefficientRatio = phi;
    constexpr auto chokeFourth = compressorChokeFlowCoefficientRatio * compressorChokeFlowCoefficientRatio
        * compressorChokeFlowCoefficientRatio * compressorChokeFlowCoefficientRatio;
    const auto psi0 = 1.0 / (1.0 - 1.0 / chokeFourth);
    const auto psi = std::max(0.0, psi0 * (1.0 - phi * phi * phi * phi / chokeFourth));
    const auto designHead = std::pow(std::max(1.0, config.pressureRatio), turboIsentropicExponent) - 1.0;
    point.pressureRatio = std::pow(1.0 + psi * designHead * point.speedRatio * point.speedRatio,
        1.0 / turboIsentropicExponent);
    point.choke = phi >= compressorChokeFlowCoefficientRatio;
    point.surge = phi < compressorSurgeFlowCoefficientRatio;
    const auto flowOffset = phi - 1.0;
    const auto speedOffset = point.speedRatio - 1.0;
    point.efficiency = std::clamp(peak - 0.24 * flowOffset * flowOffset - 0.25 * speedOffset * speedOffset,
        0.35, peak);
    return point;
}

/** Axial speed through the inducer at the centre of a typical compressor map. */
inline constexpr double compressorInducerDesignVelocityMps = 100.0;

/** One unit's corrected flow at the centre of its map: the authored one, else
 *  100 m/s through an authored inducer, else the engine's air flow at 70 % of
 *  the redline at the wastegate boost (volumetric efficiency 0.9), shared by
 *  the units. */
[[nodiscard]] inline double compressorDesignMassFlowKgPerSecond(const EngineConfig& config) noexcept {
    const auto& turbo = config.forcedInduction;
    if (turbo.compressorDesignMassFlowKgPerSecond > 0.0) return turbo.compressorDesignMassFlowKgPerSecond;
    constexpr double referenceDensityKgPerM3 = compressorReferencePressureKpa * 1'000.0
        / (287.05 * compressorReferenceTemperatureK);
    if (turbo.compressorInducerDiameterMm > 0.0) {
        const auto radiusM = turbo.compressorInducerDiameterMm * 0.0005;
        return referenceDensityKgPerM3 * std::numbers::pi * radiusM * radiusM
            * compressorInducerDesignVelocityMps;
    }
    const auto ratio = std::max(1.0, turbo.wastegatePressureRatio);
    const auto chargeTemperatureRatio = 1.0 + (std::pow(ratio, turboIsentropicExponent) - 1.0)
        / std::clamp(turbo.compressorEfficiency, 0.35, 0.90);
    const auto flow = referenceDensityKgPerM3 * engineDisplacementLitres(config) * 0.001
        * 0.7 * config.redlineRpm / 120.0 * 0.9 * ratio / chargeTemperatureRatio;
    return std::max(0.005, flow / static_cast<double>(std::max(1U, turbo.turbochargerCount)));
}

/** The pressure ratio parallel compressors hold together, and each one's
 *  share of the flow.
 *
 * Compressors feeding one charge sit at one outlet pressure; each passes the
 * flow its own map gives at that pressure and its own speed. The ratio is
 * the one at which those flows add up to the flow drawn, found by bisection
 * (the total falls with the ratio). A compressor that cannot reach the ratio
 * at its speed passes nothing (it would surge). One compressor needs no
 * search: the map gives the ratio from the flow directly.
 */
template <std::size_t N>
[[nodiscard]] inline double parallelCompressorPressureRatio(
    const ForcedInductionConfig& config, double designMassFlowKgPerSecond,
    const std::array<double, N>& angularSpeedRadPerSecond, std::size_t count,
    double totalMassFlowKgPerSecond, double inletTemperatureK, double inletPressureKpa,
    std::array<double, N>& massFlowKgPerSecond) noexcept {
    count = std::clamp<std::size_t>(count, 1, N);
    massFlowKgPerSecond.fill(0.0);
    const auto total = std::max(0.0, totalMassFlowKgPerSecond);
    if (count == 1) {
        massFlowKgPerSecond[0] = total;
        return compressorOperatingPoint(config, designMassFlowKgPerSecond, angularSpeedRadPerSecond[0],
            total, inletTemperatureK, inletPressureKpa).pressureRatio;
    }
    const auto designOmega = std::max(1.0, config.designShaftSpeedRpm * 2.0 * std::numbers::pi / 60.0);
    const auto temperatureK = std::max(150.0, inletTemperatureK);
    const auto flowScale = designMassFlowKgPerSecond * (std::max(1.0, inletPressureKpa)
        / compressorReferencePressureKpa) / std::sqrt(temperatureK / compressorReferenceTemperatureK);
    const auto designHead = std::pow(std::max(1.0, config.pressureRatio), turboIsentropicExponent) - 1.0;
    constexpr auto chokeFourth = compressorChokeFlowCoefficientRatio * compressorChokeFlowCoefficientRatio
        * compressorChokeFlowCoefficientRatio * compressorChokeFlowCoefficientRatio;
    const auto psi0 = 1.0 / (1.0 - 1.0 / chokeFourth);
    // Each compressor's flow at a ratio: the map read backwards.
    const auto flowAt = [&](std::size_t index, double ratio) {
        const auto speed = std::max(0.0, angularSpeedRadPerSecond[index]) / designOmega
            * std::sqrt(compressorReferenceTemperatureK / temperatureK);
        if (speed < 1.0e-6) return ratio <= 1.0 ? total / static_cast<double>(count) : 0.0;
        const auto psi = (std::pow(std::max(1.0, ratio), turboIsentropicExponent) - 1.0)
            / (designHead * speed * speed);
        if (psi >= psi0) return 0.0;
        const auto phi = compressorChokeFlowCoefficientRatio * std::pow(1.0 - psi / psi0, 0.25);
        return phi * speed * flowScale;
    };
    auto highest = 1.0;
    for (std::size_t index = 0; index < count; ++index)
        highest = std::max(highest, compressorOperatingPoint(config, designMassFlowKgPerSecond,
            angularSpeedRadPerSecond[index], 0.0, inletTemperatureK, inletPressureKpa).pressureRatio);
    auto low = 1.0;
    auto high = highest;
    const auto sumAt = [&](double ratio) {
        auto sum = 0.0;
        for (std::size_t index = 0; index < count; ++index) sum += flowAt(index, ratio);
        return sum;
    };
    if (sumAt(low) <= total) {
        // Even at no pressure rise they cannot pass the flow: choked.
        high = low;
    } else {
        for (int iteration = 0; iteration < 40; ++iteration) {
            const auto middle = 0.5 * (low + high);
            (sumAt(middle) > total ? low : high) = middle;
        }
    }
    const auto ratio = 0.5 * (low + high);
    auto sum = 0.0;
    for (std::size_t index = 0; index < count; ++index) {
        massFlowKgPerSecond[index] = flowAt(index, ratio);
        sum += massFlowKgPerSecond[index];
    }
    // Share the drawn flow in the proportions the maps give.
    for (std::size_t index = 0; index < count; ++index)
        massFlowKgPerSecond[index] = sum > 0.0 ? total * massFlowKgPerSecond[index] / sum
                                               : total / static_cast<double>(count);
    return ratio;
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
