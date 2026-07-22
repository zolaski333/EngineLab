#include <enginelab/physics/CylinderHeatTransferModel.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace enginelab {

CylinderHeatTransferResult CylinderHeatTransferModel::evaluate(
    const CylinderHeatTransferConditions& conditions) noexcept {
    CylinderHeatTransferResult result;
    if (!(conditions.boreM > 0.0)
        || !(conditions.pistonTravelM >= 0.0)
        || !(conditions.meanPistonSpeedMps > 0.0)
        || !(conditions.pressureKpa > 0.0)
        || !(conditions.gasTemperatureK > 0.0)
        || !(conditions.wallTemperatureK > 0.0)
        || !(conditions.gasHeatCapacityJPerK > 0.0)
        || !(conditions.fixedConductanceWPerK >= 0.0)
        || !(conditions.durationSeconds > 0.0)
        || !std::isfinite(conditions.boreM)
        || !std::isfinite(conditions.pistonTravelM)
        || !std::isfinite(conditions.meanPistonSpeedMps)
        || !std::isfinite(conditions.pressureKpa)
        || !std::isfinite(conditions.gasTemperatureK)
        || !std::isfinite(conditions.wallTemperatureK)
        || !std::isfinite(conditions.gasHeatCapacityJPerK)
        || !std::isfinite(conditions.fixedConductanceWPerK)
        || !std::isfinite(conditions.durationSeconds))
        return result;

    // Woschni (SAE 670931), with B in m, p in kPa, T in K and w in m/s:
    // h = 3.26 B^-0.2 p^0.8 T^-0.55 w^0.8 [W/(m^2 K)].
    // C1 is 6.18 during gas exchange and 2.28 for closed-valve motoring.
    const auto gasVelocityMps = (conditions.gasExchangeStroke ? 6.18 : 2.28)
        * conditions.meanPistonSpeedMps;
    result.coefficientWPerM2K = 3.26
        * std::pow(conditions.boreM, -0.2)
        * std::pow(conditions.pressureKpa, 0.8)
        * std::pow(conditions.gasTemperatureK, -0.55)
        * std::pow(gasVelocityMps, 0.8);

    const auto radiusM = conditions.boreM * 0.5;
    const auto crownAndHeadAreaM2 = 2.0 * std::numbers::pi
        * radiusM * radiusM;
    const auto exposedLinerAreaM2 = std::numbers::pi * conditions.boreM
        * conditions.pistonTravelM;
    result.exposedAreaM2 = crownAndHeadAreaM2 + exposedLinerAreaM2;
    result.conductanceWPerK = conditions.fixedConductanceWPerK
        + result.coefficientWPerM2K * result.exposedAreaM2;

    const auto exponent = -result.conductanceWPerK
        * conditions.durationSeconds / conditions.gasHeatCapacityJPerK;
    const auto exchangedFraction = -std::expm1(exponent);
    result.heatToGasJ = conditions.gasHeatCapacityJPerK
        * (conditions.wallTemperatureK - conditions.gasTemperatureK)
        * exchangedFraction;
    return result;
}

} // namespace enginelab
