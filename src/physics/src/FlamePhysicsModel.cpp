#include <enginelab/physics/FlamePhysicsModel.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace enginelab {

double FlamePhysicsModel::laminarFlameSpeedMps(const FuelConfig& fuel,
                                                double equivalenceRatio,
                                                double temperatureK,
                                                double pressurePa) noexcept {
    // Metghalchi-Keck gasoline correlation. B values are expressed in m/s.
    const auto phi = std::clamp(equivalenceRatio, 0.45, 1.75);
    constexpr double phiMaximum = 1.21;
    constexpr double bMaximum = 0.305;
    constexpr double bPhi = -0.549;
    const auto alpha = 2.4 - 0.271 * std::pow(phi, 3.51);
    const auto beta = -0.357 + 0.14 * std::pow(phi, 2.77);
    const auto referenceSpeed = std::max(0.015,
        bMaximum + bPhi * (phi - phiMaximum) * (phi - phiMaximum));
    const auto temperatureRatio = std::clamp(temperatureK / 298.0, 0.45, 8.0);
    const auto pressureRatio = std::clamp(pressurePa / 101'325.0, 0.08, 80.0);
    const auto fuelCalibration = std::clamp(fuel.laminarFlameSpeedMps / bMaximum, 0.25, 3.0);
    return std::clamp(referenceSpeed * std::pow(temperatureRatio, alpha)
        * std::pow(pressureRatio, beta) * fuelCalibration, 0.01, 8.0);
}

double FlamePhysicsModel::turbulentFlameSpeedMps(const FuelConfig& fuel,
                                                  const FlameConditions& conditions) noexcept {
    const auto laminar = laminarFlameSpeedMps(fuel, conditions.equivalenceRatio,
                                               conditions.temperatureK, conditions.pressurePa);
    const auto turbulence = std::max(0.0, conditions.meanPistonSpeedMps)
        * std::clamp(conditions.chamberTurbulenceIntensityRatio, 0.1, 4.0)
        * std::clamp(fuel.turbulenceFlameSpeedGain, 0.0, 8.0)
        * (0.28 + 0.72 * std::clamp(conditions.load, 0.0, 1.5));
    const auto dilutionAttenuation = std::clamp(1.0 - conditions.residualDilutionSensitivity
        * std::clamp(conditions.burnedGasFraction, 0.0, 0.85), 0.25, 1.0);
    // In a running SI engine the integral turbulent flame speed scales with
    // turbulence intensity (u'), not with sqrt(u'/S_L).  The previous square-
    // root closure limited high-speed flames to a few m/s and left most fuel
    // unburned at EVO. The coefficient represents the fraction of the bulk
    // piston-driven turbulence effective at wrinkling the flame front.
    const auto turbulentContribution = turbulence * 1.12;
    // Dilution must attenuate the laminar component too.  Using S_L as the
    // lower clamp silently cancelled the residual-gas term at low turbulence.
    return std::clamp((laminar + turbulentContribution) * dilutionAttenuation,
                      laminar * 0.25, 42.0);
}

double FlamePhysicsModel::ignitionDelaySeconds(const CombustionCalibrationConfig& calibration,
                                                const FlameConditions& conditions) noexcept {
    const auto temperatureRatio = std::clamp(700.0 / std::max(250.0, conditions.temperatureK), 0.25, 3.0);
    const auto pressureRatio = std::clamp(101'325.0 / std::max(20'000.0, conditions.pressurePa), 0.05, 5.0);
    const auto mixturePenalty = 1.0 + std::pow(std::abs(conditions.equivalenceRatio - 1.05), 1.35) * 1.8;
    const auto residualPenalty = 1.0 + std::clamp(conditions.burnedGasFraction, 0.0, 0.9)
        * calibration.residualDilutionSensitivity;
    return std::clamp(calibration.baseIgnitionDelaySeconds
        * std::pow(temperatureRatio, calibration.ignitionDelayTemperatureExponent)
        * std::pow(pressureRatio, calibration.ignitionDelayPressureExponent)
        * mixturePenalty * residualPenalty, 0.0, 0.02);
}

double FlamePhysicsModel::combustionEfficiency(const FlameConditions& conditions) noexcept {
    const auto phiError = conditions.equivalenceRatio - 1.08;
    const auto mixtureEfficiency = std::clamp(1.0 - 0.72 * phiError * phiError, 0.18, 1.0);
    const auto dilutionEfficiency = std::clamp(1.0 - 1.15
        * std::clamp(conditions.burnedGasFraction, 0.0, 0.75), 0.20, 1.0);
    const auto turbulenceEfficiency = std::clamp(0.82
        + std::max(0.0, conditions.meanPistonSpeedMps) * 0.018, 0.82, 1.0);
    return std::clamp(0.94 * mixtureEfficiency * dilutionEfficiency * turbulenceEfficiency,
                      0.12, 0.94);
}

void FlamePhysicsModel::ignite(FlameEvent& event, const FuelConfig& fuel,
                               const FlameConditions& conditions,
                               double burnableFuelMoles) const noexcept {
    event = {};
    if (burnableFuelMoles <= 1.0e-15 || conditions.chamberVolumeM3 <= 1.0e-10) return;
    constexpr double kernelRadiusM = 0.00045;
    event.radialTravelM = kernelRadiusM;
    event.axialTravelM = kernelRadiusM;
    event.lastChamberVolumeM3 = conditions.chamberVolumeM3;
    event.flameSpeedMps = turbulentFlameSpeedMps(fuel, conditions);
    event.efficiency = combustionEfficiency(conditions);
    event.initialBurnableFuelMoles = burnableFuelMoles;
    event.active = true;
}

FlameStepResult FlamePhysicsModel::advance(FlameEvent& event, const FuelConfig& fuel,
                                           const FlameConditions& conditions,
                                           double dtSeconds) const noexcept {
    FlameStepResult result { event.burnedFraction, 0.0, event.flameSpeedMps,
                             event.efficiency, !event.active };
    if (!event.active || dtSeconds <= 0.0 || !std::isfinite(dtSeconds)) return result;

    const auto pistonAreaM2 = std::numbers::pi * conditions.boreM * conditions.boreM * 0.25;
    const auto radialLimit = std::max(0.0005, conditions.boreM * 0.5);
    const auto axialLimit = std::max(0.0005, conditions.chamberVolumeM3 / pistonAreaM2);
    if (event.lastChamberVolumeM3 > 1.0e-12) {
        const auto expansion = conditions.chamberVolumeM3 / event.lastChamberVolumeM3;
        event.axialTravelM = std::clamp(event.axialTravelM * expansion, 0.0, axialLimit);
    }

    event.flameSpeedMps = turbulentFlameSpeedMps(fuel, conditions);
    const auto travel = event.flameSpeedMps * dtSeconds;
    event.radialTravelM = std::min(radialLimit, event.radialTravelM + travel);
    event.axialTravelM = std::min(axialLimit, event.axialTravelM + travel);
    event.elapsedSeconds += dtSeconds;
    event.lastChamberVolumeM3 = conditions.chamberVolumeM3;

    // The propagation coordinates span the chamber radius and its instantaneous
    // axial height.  pi*r^2*h therefore reaches exactly the chamber volume at
    // both geometric limits; the previous 4/3 factor completed combustion
    // before the front had traversed the chamber.
    const auto kernelCount = static_cast<double>(
        std::clamp<std::uint32_t>(conditions.ignitionSiteCount, 1, 4));
    const auto burnedVolumeM3 = kernelCount * std::numbers::pi
        * event.radialTravelM * event.radialTravelM * event.axialTravelM;
    const auto geometricFraction = std::clamp(burnedVolumeM3
        / std::max(1.0e-12, conditions.chamberVolumeM3), 0.0, 1.0);
    const auto previousFraction = event.burnedFraction;
    event.burnedFraction = std::max(previousFraction, geometricFraction);
    const auto complete = event.burnedFraction >= 0.999
        || (event.radialTravelM >= radialLimit && event.axialTravelM >= axialLimit);
    if (complete) {
        event.burnedFraction = 1.0;
        event.active = false;
    }

    result = { event.burnedFraction, std::max(0.0, event.burnedFraction - previousFraction),
               event.flameSpeedMps, event.efficiency, !event.active };
    return result;
}

} // namespace enginelab
