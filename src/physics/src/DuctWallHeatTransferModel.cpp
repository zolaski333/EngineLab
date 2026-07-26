#include <enginelab/physics/DuctWallHeatTransferModel.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace enginelab {
namespace {
[[nodiscard]] bool finitePositive(double value) noexcept {
    return std::isfinite(value) && value > 0.0;
}
}

bool DuctWallHeatTransferGeometry::valid() const noexcept {
    return finitePositive(innerDiameterM)
        && finitePositive(wallHeatCapacityJPerK)
        && finitePositive(innerAreaM2)
        && std::isfinite(externalConductanceWPerK)
        && externalConductanceWPerK >= 0.0;
}

DuctWallHeatTransferGeometry DuctWallHeatTransferModel::prepareGeometry(
    double innerDiameterM,
    double lengthM,
    double wallThicknessM,
    double wallDensityKgPerM3,
    double wallSpecificHeatJPerKgK,
    double externalHeatTransferWPerM2K) noexcept {
    if (!finitePositive(innerDiameterM)
        || !finitePositive(lengthM)
        || !finitePositive(wallThicknessM)
        || !finitePositive(wallDensityKgPerM3)
        || !finitePositive(wallSpecificHeatJPerKgK)
        || !(externalHeatTransferWPerM2K >= 0.0)
        || !std::isfinite(externalHeatTransferWPerM2K))
        return {};
    const auto innerRadiusM = innerDiameterM * 0.5;
    const auto outerRadiusM = innerRadiusM + wallThicknessM;
    const auto wallVolumeM3 = std::numbers::pi * lengthM
        * (outerRadiusM * outerRadiusM - innerRadiusM * innerRadiusM);
    const auto wallHeatCapacityJPerK = wallVolumeM3 * wallDensityKgPerM3
        * wallSpecificHeatJPerKgK;
    const auto innerAreaM2 = std::numbers::pi * innerDiameterM * lengthM;
    const auto outerAreaM2 =
        2.0 * std::numbers::pi * outerRadiusM * lengthM;
    DuctWallHeatTransferGeometry result {
        innerDiameterM,
        wallHeatCapacityJPerK,
        innerAreaM2,
        externalHeatTransferWPerM2K * outerAreaM2,
    };
    return result.valid() ? result : DuctWallHeatTransferGeometry {};
}

DuctWallHeatTransferResult DuctWallHeatTransferModel::advancePrepared(
    DuctWallThermalState& state,
    const DuctWallHeatTransferGeometry& geometry,
    const DuctWallHeatTransferConditions& conditions) noexcept {
    DuctWallHeatTransferResult result;
    if (!geometry.valid()
        || !finitePositive(conditions.surroundingsTemperatureK)
        || !finitePositive(conditions.gasDensityKgPerM3)
        || !std::isfinite(conditions.gasVelocityMps)
        || !finitePositive(conditions.gasSpecificHeatCpJPerKgK)
        || !finitePositive(conditions.gasHeatCapacityJPerK)
        || !finitePositive(conditions.gasTemperatureK)
        || !finitePositive(conditions.durationSeconds))
        return result;

    if (!finitePositive(state.temperatureK))
        state.temperatureK = conditions.surroundingsTemperatureK;

    result.wallHeatCapacityJPerK = geometry.wallHeatCapacityJPerK;

    constexpr double referenceViscosityPaS = 1.716e-5;
    constexpr double referenceTemperatureK = 273.15;
    constexpr double sutherlandTemperatureK = 110.4;
    constexpr double prandtlNumber = 0.71;
    // Gas properties at the film temperature: Sutherland viscosity and the
    // Eucken identity k = mu*Cp/Pr keep conductivity consistent with the
    // caller's mixture heat capacity instead of assuming room-temperature air.
    const auto filmTemperatureK = 0.5
        * (conditions.gasTemperatureK + state.temperatureK);
    const auto temperatureRatio = filmTemperatureK / referenceTemperatureK;
    const auto dynamicViscosityPaS = referenceViscosityPaS
        * temperatureRatio * std::sqrt(temperatureRatio)
        * (referenceTemperatureK + sutherlandTemperatureK)
        / (filmTemperatureK + sutherlandTemperatureK);
    result.reynoldsNumber = conditions.gasDensityKgPerM3
        * std::abs(conditions.gasVelocityMps) * geometry.innerDiameterM
        / dynamicViscosityPaS;

    // Fully developed laminar circular-tube limit below transition; the
    // turbulent branch is the Gnielinski correlation with its smooth-pipe
    // friction factor. Blend only across the transitional Reynolds interval.
    constexpr double laminarNusselt = 3.66;
    const auto turbulentReynolds = std::max(3'000.0, result.reynoldsNumber);
    const auto frictionFactor = 1.0 / std::pow(
        0.79 * std::log(turbulentReynolds) - 1.64, 2.0);
    const auto turbulentNusselt = (frictionFactor / 8.0)
        * (turbulentReynolds - 1'000.0) * prandtlNumber
        / (1.0 + 12.7 * std::sqrt(frictionFactor / 8.0)
            * (std::pow(prandtlNumber, 2.0 / 3.0) - 1.0));
    const auto turbulentBlend = std::clamp(
        (result.reynoldsNumber - 2'300.0) / (4'000.0 - 2'300.0), 0.0, 1.0);
    result.nusseltNumber = std::lerp(
        laminarNusselt, turbulentNusselt, turbulentBlend);

    const auto gasThermalConductivityWPerMK = dynamicViscosityPaS
        * conditions.gasSpecificHeatCpJPerKgK / prandtlNumber;
    result.internalCoefficientWPerM2K = result.nusseltNumber
        * gasThermalConductivityWPerMK / geometry.innerDiameterM;
    const auto internalConductanceWPerK = result.internalCoefficientWPerM2K
        * geometry.innerAreaM2;

    // Exact exchange between two finite thermal capacities. Unlike a clipped
    // Euler source, this conserves gas+wall energy and cannot overshoot their
    // common equilibrium temperature at any solver step size.
    const auto reducedHeatCapacityJPerK =
        conditions.gasHeatCapacityJPerK * result.wallHeatCapacityJPerK
        / (conditions.gasHeatCapacityJPerK + result.wallHeatCapacityJPerK);
    const auto exchangeRatePerSecond = internalConductanceWPerK
        * (1.0 / conditions.gasHeatCapacityJPerK
            + 1.0 / result.wallHeatCapacityJPerK);
    const auto exchangedFraction = -std::expm1(
        -exchangeRatePerSecond * conditions.durationSeconds);
    result.heatToGasJ = reducedHeatCapacityJPerK
        * (state.temperatureK - conditions.gasTemperatureK)
        * exchangedFraction;
    state.temperatureK -= result.heatToGasJ / result.wallHeatCapacityJPerK;

    const auto wallTemperatureBeforeExternalK = state.temperatureK;
    state.temperatureK = conditions.surroundingsTemperatureK
        + (state.temperatureK - conditions.surroundingsTemperatureK)
            * std::exp(-geometry.externalConductanceWPerK * conditions.durationSeconds
                / result.wallHeatCapacityJPerK);
    result.heatRejectedJ = result.wallHeatCapacityJPerK
        * (wallTemperatureBeforeExternalK - state.temperatureK);
    return result;
}

DuctWallHeatTransferResult DuctWallHeatTransferModel::advance(
    DuctWallThermalState& state,
    const DuctWallHeatTransferConditions& conditions) noexcept {
    const auto geometry = prepareGeometry(
        conditions.innerDiameterM,
        conditions.lengthM,
        conditions.wallThicknessM,
        conditions.wallDensityKgPerM3,
        conditions.wallSpecificHeatJPerKgK,
        conditions.externalHeatTransferWPerM2K);
    return advancePrepared(state, geometry, conditions);
}

} // namespace enginelab
