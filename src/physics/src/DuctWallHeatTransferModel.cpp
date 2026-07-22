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

DuctWallHeatTransferResult DuctWallHeatTransferModel::advance(
    DuctWallThermalState& state,
    const DuctWallHeatTransferConditions& conditions) noexcept {
    DuctWallHeatTransferResult result;
    if (!finitePositive(conditions.innerDiameterM)
        || !finitePositive(conditions.lengthM)
        || !finitePositive(conditions.wallThicknessM)
        || !finitePositive(conditions.wallDensityKgPerM3)
        || !finitePositive(conditions.wallSpecificHeatJPerKgK)
        || !(conditions.externalHeatTransferWPerM2K >= 0.0)
        || !std::isfinite(conditions.externalHeatTransferWPerM2K)
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

    const auto innerRadiusM = conditions.innerDiameterM * 0.5;
    const auto outerRadiusM = innerRadiusM + conditions.wallThicknessM;
    const auto wallVolumeM3 = std::numbers::pi * conditions.lengthM
        * (outerRadiusM * outerRadiusM - innerRadiusM * innerRadiusM);
    result.wallHeatCapacityJPerK = wallVolumeM3 * conditions.wallDensityKgPerM3
        * conditions.wallSpecificHeatJPerKgK;
    if (!finitePositive(result.wallHeatCapacityJPerK)) return {};

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
        * std::abs(conditions.gasVelocityMps) * conditions.innerDiameterM
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
        * gasThermalConductivityWPerMK / conditions.innerDiameterM;
    const auto innerAreaM2 = std::numbers::pi * conditions.innerDiameterM
        * conditions.lengthM;
    const auto internalConductanceWPerK = result.internalCoefficientWPerM2K
        * innerAreaM2;

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

    const auto outerAreaM2 = 2.0 * std::numbers::pi * outerRadiusM
        * conditions.lengthM;
    const auto externalConductanceWPerK =
        conditions.externalHeatTransferWPerM2K * outerAreaM2;
    const auto wallTemperatureBeforeExternalK = state.temperatureK;
    state.temperatureK = conditions.surroundingsTemperatureK
        + (state.temperatureK - conditions.surroundingsTemperatureK)
            * std::exp(-externalConductanceWPerK * conditions.durationSeconds
                / result.wallHeatCapacityJPerK);
    result.heatRejectedJ = result.wallHeatCapacityJPerK
        * (wallTemperatureBeforeExternalK - state.temperatureK);
    return result;
}

} // namespace enginelab
