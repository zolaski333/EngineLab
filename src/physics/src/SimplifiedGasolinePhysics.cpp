#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <numeric>

namespace enginelab {
namespace {
double baseVolumetricEfficiency(double rpmRatio, double load) noexcept {
    constexpr std::array<double, 5> rpmAxis { 0.0, 0.25, 0.55, 0.82, 1.10 };
    constexpr std::array<double, 3> loadAxis { 0.25, 0.60, 1.0 };
    constexpr std::array<std::array<double, 3>, 5> table {{
        {{ 0.42, 0.55, 0.62 }}, {{ 0.58, 0.73, 0.82 }}, {{ 0.67, 0.86, 0.96 }},
        {{ 0.62, 0.82, 0.92 }}, {{ 0.49, 0.68, 0.78 }} }};
    const auto rpmUpper = std::upper_bound(rpmAxis.begin(), rpmAxis.end(), rpmRatio);
    const auto loadUpper = std::upper_bound(loadAxis.begin(), loadAxis.end(), load);
    const auto r1 = static_cast<std::size_t>(std::clamp<std::ptrdiff_t>(rpmUpper - rpmAxis.begin(), 1, rpmAxis.size() - 1));
    const auto l1 = static_cast<std::size_t>(std::clamp<std::ptrdiff_t>(loadUpper - loadAxis.begin(), 1, loadAxis.size() - 1));
    const auto r0 = r1 - 1; const auto l0 = l1 - 1;
    const auto rt = std::clamp((rpmRatio - rpmAxis[r0]) / (rpmAxis[r1] - rpmAxis[r0]), 0.0, 1.0);
    const auto lt = std::clamp((load - loadAxis[l0]) / (loadAxis[l1] - loadAxis[l0]), 0.0, 1.0);
    return std::lerp(std::lerp(table[r0][l0], table[r0][l1], lt),
                     std::lerp(table[r1][l0], table[r1][l1], lt), rt);
}

[[nodiscard]] double effectiveLiftArea(double durationDegrees, double maximumLiftMm,
                                       const std::vector<ValveLiftSample>& profile) noexcept {
    if (profile.size() < 2) return durationDegrees * maximumLiftMm * 0.5;
    double area = 0.0;
    for (std::size_t index = 1; index < profile.size(); ++index) {
        const auto width = std::max(0.0, profile[index].angleDegrees - profile[index - 1].angleDegrees);
        const auto lift = (profile[index].liftMm + profile[index - 1].liftMm) * 0.5;
        area += width * std::clamp(lift, 0.0, maximumLiftMm);
    }
    return area;
}
}

CombustionResult SimplifiedGasolinePhysics::evaluateCombustion(
    const EngineConfig& config, const EngineState& state, const EngineControls& controls,
    const EcuCommand& ecu, double exhaustBackPressureKpa) const noexcept {
    if (config.cylinders.empty()) return {};
    const auto combustionEnabled = ecu.fuelEnabled && ecu.sparkEnabled && state.rpm >= 220.0 && state.damage < 1.0;

    const auto compressionSum = std::accumulate(config.cylinders.begin(), config.cylinders.end(), 0.0,
        [](double sum, const CylinderConfig& cylinder) { return sum + cylinder.compressionRatio; });
    const auto compression = compressionSum / static_cast<double>(config.cylinders.size());
    const auto rpmRatio = std::clamp(state.rpm / config.redlineRpm, 0.0, 1.35);
    // The cosine lift law has a mean lift of half its peak. Using its integrated
    // curtain-area proxy keeps duration and lift meaningful without pretending
    // to be a 1D gas solver.
    const auto intakeAreaRatio = effectiveLiftArea(config.camshafts.intakeDurationDegrees, config.camshafts.intakeLiftMm,
                                                   config.camshafts.intakeLiftProfile)
        / (248.0 * 10.2 * 0.5);
    const auto exhaustAreaRatio = effectiveLiftArea(config.camshafts.exhaustDurationDegrees, config.camshafts.exhaustLiftMm,
                                                    config.camshafts.exhaustLiftProfile)
        / (244.0 * 9.8 * 0.5);
    const auto intakeCenter = 360.0 + config.camshafts.intakeCenterlineDegrees;
    const auto exhaustCenter = 720.0 - config.camshafts.exhaustCenterlineDegrees;
    const auto centerSeparation = std::abs(exhaustCenter - intakeCenter);
    const auto overlapProxy = std::max(0.0, (config.camshafts.intakeDurationDegrees
        + config.camshafts.exhaustDurationDegrees) * 0.5 - centerSeparation);
    constexpr double baselineOverlap = 108.0;
    const auto highRpmCamGain = std::clamp((config.camshafts.intakeDurationDegrees - 248.0) / 180.0, -0.25, 0.35)
        * std::clamp((rpmRatio - 0.35) / 0.65, 0.0, 1.0);
    const auto lowRpmOverlapLoss = std::max(0.0, overlapProxy - baselineOverlap) / 240.0
        * (1.0 - std::clamp(rpmRatio / 0.55, 0.0, 1.0));
    const auto camCenterEffect = 1.0 + (110.0 - config.camshafts.intakeCenterlineDegrees) * 0.0012
                               + (112.0 - config.camshafts.exhaustCenterlineDegrees) * 0.0008
                               + highRpmCamGain - lowRpmOverlapLoss;
    const auto normalizedLoad = std::clamp(state.manifoldPressureKpa / std::max(1.0, config.ambientPressureKpa), 0.25, 1.0);
    const auto mappedVe = baseVolumetricEfficiency(rpmRatio - (intakeAreaRatio - 1.0) * 0.12, normalizedLoad);
    const auto backPressureLoss = std::clamp((exhaustBackPressureKpa - config.ambientPressureKpa) / 90.0, 0.0, 0.32);
    const auto intakeLiftFactor = std::clamp(config.camshafts.intakeLiftMm / 1.5, 0.0, 1.0);
    const auto exhaustLiftFactor = std::clamp(config.camshafts.exhaustLiftMm / 1.5, 0.0, 1.0);
    const auto liftFactor = std::min(intakeLiftFactor, exhaustLiftFactor)
        * std::sqrt(config.camshafts.intakeFlowCoefficient * config.camshafts.exhaustFlowCoefficient) / 0.62;
    const auto volumetricEfficiency = std::clamp(mappedVe
        * std::clamp(std::sqrt(std::max(0.1, intakeAreaRatio * exhaustAreaRatio)) * camCenterEffect, 0.72, 1.22)
        * (1.0 - backPressureLoss), 0.30, 1.18) * liftFactor;

    const auto temperatureK = std::max(240.0, config.ambientTemperatureC + 273.15);
    const auto manifoldDensityKgM3 = 1.204 * (state.manifoldPressureKpa / 101.325) * (293.15 / temperatureK);
    const auto displacedVolumeM3 = engineDisplacementLitres(config) * 0.001;
    const auto airMassMg = displacedVolumeM3 * volumetricEfficiency * manifoldDensityKgM3 * 1'000'000.0;
    const auto afr = std::clamp(ecu.targetAirFuelRatio, 8.0, 30.0);
    const auto fuelMassMg = ecu.fuelEnabled ? airMassMg / afr * ecu.fuelCorrection : 0.0;
    const auto actualAfr = fuelMassMg > 1.0e-9 ? airMassMg / fuelMassMg : afr;
    constexpr double gasolineEnergyJPerKg = 43'000'000.0;

    const auto afrEfficiency = std::clamp(1.0 - std::abs(actualAfr - 12.8) * 0.045, 0.35, 1.0);
    const auto load = std::clamp(0.25 + state.manifoldPressureKpa / std::max(1.0, config.ambientPressureKpa) * 0.75, 0.0, 1.0);
    const auto optimumAdvance = 8.0 + rpmRatio * 20.0 + (1.0 - load) * 12.0;
    const auto timingError = ecu.ignitionAdvanceDegrees - optimumAdvance;
    const auto timingEfficiency = std::clamp(std::exp(-timingError * timingError / 450.0), 0.45, 1.0);
    const auto chamberVolumeM3 = std::max(1.0e-6, displacedVolumeM3
        / (static_cast<double>(config.cylinders.size()) * std::max(1.0, compression - 1.0)));
    constexpr double universalGasConstant = 8.314462618;
    constexpr double airMolarMassKg = 0.02897;
    constexpr double fuelMolarMassKg = 0.11423; // octane proxy
    const auto airMoles = airMassMg * 1.0e-6 / airMolarMassKg;
    const auto fuelMoles = fuelMassMg * 1.0e-6 / fuelMolarMassKg;
    const auto oxygenMoles = airMoles * 0.2095;
    const auto inertMoles = airMoles * 0.7905;
    const auto stoichFuelMoles = oxygenMoles / 12.5;
    const auto burnedFuelMoles = std::min(fuelMoles, stoichFuelMoles);
    const auto burnCompleteness = fuelMoles > 1.0e-12 ? burnedFuelMoles / fuelMoles : 0.0;
    const auto turbulence = std::clamp(state.meanPistonSpeedMps / 22.0, 0.0, 1.8);
    const auto pressureRatio = std::max(0.35, state.manifoldPressureKpa / std::max(1.0, config.ambientPressureKpa));
    const auto flameSpeedMps = std::clamp(0.38 * (1.0 + turbulence * 1.55)
        * std::clamp(1.0 - std::abs(actualAfr - 12.8) * 0.035, 0.25, 1.0)
        * std::sqrt(std::max(0.65, temperatureK / 293.15)) / std::pow(pressureRatio, 0.18), 0.08, 5.8);
    const auto boreMeanM = std::accumulate(config.cylinders.begin(), config.cylinders.end(), 0.0,
        [](double sum, const CylinderConfig& cylinder) { return sum + cylinder.boreMm * 0.001; })
        / static_cast<double>(config.cylinders.size());
    const auto flameRadiusM = flameSpeedMps * std::clamp(0.0025 + state.rpm / 600'000.0, 0.0025, 0.013);
    const auto flameFraction = std::clamp(std::pow(flameRadiusM / std::max(0.005, boreMeanM * 0.5), 3.0), 0.0, 1.0);
    const auto burnedFraction = std::clamp(flameFraction * burnCompleteness, 0.0, 1.0);
    const auto releasedTemperatureK = temperatureK + burnedFraction * fuelMassMg * 1.0e-6
        * gasolineEnergyJPerKg / std::max(0.015, airMassMg * 1.0e-6 * 930.0);
    const auto wallLoss = std::clamp((state.coolantTemperatureC + 273.15) / std::max(300.0, releasedTemperatureK), 0.55, 0.98);
    const auto burnedGasMoles = inertMoles + std::max(0.0, oxygenMoles - burnedFuelMoles * 12.5)
        + burnedFuelMoles * 8.0 + burnedFuelMoles * 9.0;
    const auto idealPressureBar = burnedGasMoles * universalGasConstant * releasedTemperatureK * wallLoss
        / chamberVolumeM3 / 100'000.0;
    const auto boostPressureRatio = std::clamp(state.manifoldPressureKpa / std::max(1.0, config.ambientPressureKpa), 0.35, 3.5);
    const auto effectiveCompression = compression * std::sqrt(boostPressureRatio);
    const auto compressionEfficiency = std::clamp(0.25 + (effectiveCompression - 8.0) * 0.018, 0.22, 0.42);
    const auto damageEfficiency = std::clamp(1.0 - state.damage * 0.82 - state.wear * 0.18, 0.0, 1.0);
    const auto thermalEfficiency = compressionEfficiency * afrEfficiency * timingEfficiency * damageEfficiency
        * std::clamp(0.72 + burnedFraction * 0.28, 0.60, 1.0);
    const auto workPerCycleJ = fuelMassMg * 1.0e-6 * gasolineEnergyJPerKg * thermalEfficiency;
    const auto indicatedTorque = combustionEnabled ? workPerCycleJ / (4.0 * std::numbers::pi) : 0.0;

    const auto octaneKnockResistance = 9.0 + (config.octaneRating - 87.0) * 0.055;
    const auto compressionKnock = std::max(0.0, compression - octaneKnockResistance) * 0.13;
    const auto timingKnock = std::max(0.0, timingError) / 13.0;
    const auto thermalKnock = std::max(0.0, state.coolantTemperatureC - 105.0) / 35.0;
    const auto knock = combustionEnabled ? std::clamp((compressionKnock + timingKnock + thermalKnock) * load, 0.0, 1.0) : 0.0;
    const auto mixtureMisfire = std::max(0.0, std::abs(actualAfr - 14.0) - 2.6) * 0.055;
    const auto lowSpeedMisfire = std::max(0.0, 420.0 - state.rpm) / 1'500.0;
    const auto misfire = combustionEnabled ? std::clamp(mixtureMisfire + lowSpeedMisfire + knock * 0.07, 0.0, 0.85) : 0.0;
    const auto displacementM3 = std::max(1.0e-6, displacedVolumeM3);
    const auto imepBar = indicatedTorque * 4.0 * std::numbers::pi / displacementM3 / 100'000.0;
    const auto heatOutput = combustionEnabled
        ? std::clamp(fuelMassMg / std::max(1.0, displacedVolumeM3 * 100'000.0), 0.0, 1.5) : 0.0;
    const auto cyclesPerSecond = state.rpm / 120.0;
    const auto fuelPowerKw = combustionEnabled ? fuelMassMg * 1.0e-6 * cyclesPerSecond * gasolineEnergyJPerKg / 1'000.0 : 0.0;
    const auto heatPowerKw = fuelPowerKw * (1.0 - thermalEfficiency);
    (void)controls;
    return { indicatedTorque, combustionEnabled ? std::clamp(std::max(imepBar * 5.5, idealPressureBar), 2.0, 160.0) : 0.0,
             combustionEnabled ? afrEfficiency * timingEfficiency : 0.0,
             heatOutput, knock, misfire, volumetricEfficiency, airMassMg, fuelMassMg, thermalEfficiency,
             actualAfr, heatPowerKw, combustionEnabled };
}
} // namespace enginelab
