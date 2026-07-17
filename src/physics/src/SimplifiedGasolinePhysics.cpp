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

struct CamMetrics final {
    double intakeAreaRatio { 1.0 };
    double exhaustAreaRatio { 1.0 };
    double centerEffect { 1.0 };
    double liftFactor { 1.0 };
};

[[nodiscard]] const CamshaftConfig& camshaftForCylinder(const EngineConfig& config,
                                                        const CylinderConfig& cylinder) noexcept {
    const auto bank = std::find_if(config.banks.begin(), config.banks.end(),
        [&cylinder](const CylinderBankConfig& item) {
            return item.id == cylinder.bankId
                || std::find(item.cylinderIds.begin(), item.cylinderIds.end(), cylinder.id)
                    != item.cylinderIds.end();
        });
    return bank != config.banks.end() ? bank->camshafts : config.camshafts;
}

[[nodiscard]] CamMetrics aggregateCamMetrics(const EngineConfig& config, double rpmRatio,
                                              double rpm, double throttle) noexcept {
    CamMetrics total { 0.0, 0.0, 0.0, 0.0 };
    for (const auto& cylinder : config.cylinders) {
        const auto& cam = camshaftForCylinder(config, cylinder);
        const auto high = cam.variableProfileEnabled && rpm >= cam.switchRpm && throttle >= cam.switchThrottle;
        const auto intakeDuration = high ? cam.highIntakeDurationDegrees : cam.intakeDurationDegrees;
        const auto exhaustDuration = high ? cam.highExhaustDurationDegrees : cam.exhaustDurationDegrees;
        const auto intakeLift = high ? cam.highIntakeLiftMm : cam.intakeLiftMm;
        const auto exhaustLift = high ? cam.highExhaustLiftMm : cam.exhaustLiftMm;
        const auto& intakeProfile = high && !cam.highIntakeLiftProfile.empty()
            ? cam.highIntakeLiftProfile : cam.intakeLiftProfile;
        const auto& exhaustProfile = high && !cam.highExhaustLiftProfile.empty()
            ? cam.highExhaustLiftProfile : cam.exhaustLiftProfile;
        const auto intakeAreaRatio = effectiveLiftArea(intakeDuration, intakeLift, intakeProfile)
            / (248.0 * 10.2 * 0.5);
        const auto exhaustAreaRatio = effectiveLiftArea(exhaustDuration, exhaustLift, exhaustProfile)
            / (244.0 * 9.8 * 0.5);
        const auto intakeCenter = 360.0 + cam.intakeCenterlineDegrees;
        const auto exhaustCenter = 360.0 - cam.exhaustCenterlineDegrees;
        const auto overlapProxy = std::max(0.0, (intakeDuration + exhaustDuration) * 0.5
            - std::abs(exhaustCenter - intakeCenter));
        constexpr double baselineOverlap = 108.0;
        const auto highRpmCamGain = std::clamp((intakeDuration - 248.0) / 180.0, -0.25, 0.35)
            * std::clamp((rpmRatio - 0.35) / 0.65, 0.0, 1.0);
        const auto lowRpmOverlapLoss = std::max(0.0, overlapProxy - baselineOverlap) / 240.0
            * (1.0 - std::clamp(rpmRatio / 0.55, 0.0, 1.0));
        total.intakeAreaRatio += intakeAreaRatio;
        total.exhaustAreaRatio += exhaustAreaRatio;
        total.centerEffect += 1.0 + (110.0 - cam.intakeCenterlineDegrees) * 0.0012
            + (112.0 - cam.exhaustCenterlineDegrees) * 0.0008 + highRpmCamGain - lowRpmOverlapLoss;
        total.liftFactor += std::min(std::clamp(intakeLift / 1.5, 0.0, 1.0),
            std::clamp(exhaustLift / 1.5, 0.0, 1.0))
            * std::sqrt(cam.intakeFlowCoefficient * cam.exhaustFlowCoefficient) / 0.62;
    }
    const auto count = static_cast<double>(std::max<std::size_t>(1, config.cylinders.size()));
    total.intakeAreaRatio /= count;
    total.exhaustAreaRatio /= count;
    total.centerEffect /= count;
    total.liftFactor /= count;
    return total;
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
    // Aggregate the active profile of every cylinder/bank. This keeps the
    // low-frequency calibration model aligned with the valves used by the gas
    // solver instead of silently falling back to the global camshaft.
    const auto cam = aggregateCamMetrics(config, rpmRatio, state.rpm, state.throttle);
    const auto normalizedLoad = std::clamp(state.manifoldPressureKpa / std::max(1.0, config.ambientPressureKpa), 0.25, 1.0);
    const auto mappedVe = baseVolumetricEfficiency(rpmRatio - (cam.intakeAreaRatio - 1.0) * 0.12, normalizedLoad);
    const auto backPressureLoss = std::clamp((exhaustBackPressureKpa - config.ambientPressureKpa) / 90.0, 0.0, 0.32);
    const auto calibratedVolumetricEfficiency = std::clamp(mappedVe
        * std::clamp(std::sqrt(std::max(0.1, cam.intakeAreaRatio * cam.exhaustAreaRatio)) * cam.centerEffect, 0.72, 1.22)
        * (1.0 - backPressureLoss), 0.30, 1.18) * cam.liftFactor;
    const auto hasResolvedCharge = state.airMassMgPerCycle > 1.0
        && state.volumetricEfficiency > 0.0;
    const auto volumetricEfficiency = hasResolvedCharge
        ? std::clamp(state.volumetricEfficiency, 0.0, 2.5)
        : calibratedVolumetricEfficiency;

    const auto temperatureK = std::max(240.0, config.ambientTemperatureC + 273.15);
    const auto manifoldDensityKgM3 = 1.204 * (state.manifoldPressureKpa / 101.325) * (293.15 / temperatureK);
    const auto displacedVolumeM3 = engineDisplacementLitres(config) * 0.001;
    const auto airMassMg = hasResolvedCharge ? state.airMassMgPerCycle
        : displacedVolumeM3 * volumetricEfficiency * manifoldDensityKgM3 * 1'000'000.0;
    const auto afr = std::clamp(ecu.targetAirFuelRatio, 5.0, 30.0);
    const auto fuelMassMg = ecu.fuelEnabled ? airMassMg / afr * ecu.fuelCorrection : 0.0;
    const auto actualAfr = fuelMassMg > 1.0e-9 ? airMassMg / fuelMassMg : afr;
    const auto fuelEnergyJPerKg = config.fuelProperties.lowerHeatingValueMjPerKg * 1'000'000.0;
    const auto bestPowerAfr = config.fuelProperties.stoichiometricAirFuelRatio * 0.87;

    const auto afrEfficiency = std::clamp(1.0 - std::abs(actualAfr - bestPowerAfr)
        / config.fuelProperties.stoichiometricAirFuelRatio * 0.66, 0.35, 1.0);
    const auto load = std::clamp(0.25 + state.manifoldPressureKpa / std::max(1.0, config.ambientPressureKpa) * 0.75, 0.0, 1.0);
    const auto optimumAdvance = 8.0 + rpmRatio * 20.0 + (1.0 - load) * 12.0;
    const auto timingError = ecu.ignitionAdvanceDegrees - optimumAdvance;
    const auto timingEfficiency = std::clamp(std::exp(-timingError * timingError / 450.0), 0.45, 1.0);
    const auto chamberVolumeM3 = std::max(1.0e-6, displacedVolumeM3
        / (static_cast<double>(config.cylinders.size()) * std::max(1.0, compression - 1.0)));
    constexpr double universalGasConstant = 8.314462618;
    constexpr double airMolarMassKg = 0.02897;
    const auto fuelMolarMassKg = config.fuelProperties.molarMassGramsPerMole * 0.001;
    const auto airMoles = airMassMg * 1.0e-6 / airMolarMassKg;
    const auto fuelMoles = fuelMassMg * 1.0e-6 / fuelMolarMassKg;
    const auto oxygenMoles = airMoles * 0.2095;
    const auto inertMoles = airMoles * 0.7905;
    const auto stoichFuelMoles = oxygenMoles / config.fuelProperties.oxygenMolesPerFuelMole;
    const auto burnedFuelMoles = std::min(fuelMoles, stoichFuelMoles);
    const auto burnCompleteness = fuelMoles > 1.0e-12 ? burnedFuelMoles / fuelMoles : 0.0;
    const auto turbulence = std::clamp(state.meanPistonSpeedMps / 22.0, 0.0, 1.8);
    const auto pressureRatio = std::max(0.35, state.manifoldPressureKpa / std::max(1.0, config.ambientPressureKpa));
    const auto flameSpeedMps = std::clamp(config.fuelProperties.laminarFlameSpeedMps
        * (1.0 + turbulence * config.fuelProperties.turbulenceFlameSpeedGain)
        * std::clamp(1.0 - std::abs(actualAfr - bestPowerAfr)
            / config.fuelProperties.stoichiometricAirFuelRatio * 0.52, 0.25, 1.0)
        * std::sqrt(std::max(0.65, temperatureK / 293.15)) / std::pow(pressureRatio, 0.18), 0.08, 5.8);
    const auto boreMeanM = std::accumulate(config.cylinders.begin(), config.cylinders.end(), 0.0,
        [](double sum, const CylinderConfig& cylinder) { return sum + cylinder.boreMm * 0.001; })
        / static_cast<double>(config.cylinders.size());
    const auto flameRadiusM = flameSpeedMps * std::clamp(0.0025 + state.rpm / 600'000.0, 0.0025, 0.013);
    const auto flameFraction = std::clamp(std::pow(flameRadiusM / std::max(0.005, boreMeanM * 0.5), 3.0), 0.0, 1.0);
    const auto burnedFraction = std::clamp(flameFraction * burnCompleteness, 0.0, 1.0);
    const auto releasedTemperatureK = temperatureK + burnedFraction * fuelMassMg * 1.0e-6
        * fuelEnergyJPerKg / std::max(0.015, airMassMg * 1.0e-6 * 930.0);
    const auto wallLoss = std::clamp((state.coolantTemperatureC + 273.15) / std::max(300.0, releasedTemperatureK), 0.55, 0.98);
    const auto burnedGasMoles = inertMoles + std::max(0.0, oxygenMoles
        - burnedFuelMoles * config.fuelProperties.oxygenMolesPerFuelMole)
        + burnedFuelMoles * config.fuelProperties.productMolesPerFuelMole;
    const auto idealPressureBar = burnedGasMoles * universalGasConstant * releasedTemperatureK * wallLoss
        / chamberVolumeM3 / 100'000.0;
    const auto boostPressureRatio = std::clamp(state.manifoldPressureKpa / std::max(1.0, config.ambientPressureKpa), 0.35, 3.5);
    const auto effectiveCompression = compression * std::sqrt(boostPressureRatio);
    const auto compressionEfficiency = std::clamp(0.25 + (effectiveCompression - 8.0) * 0.018, 0.22, 0.42);
    const auto damageEfficiency = std::clamp(1.0 - state.damage * 0.82 - state.wear * 0.18, 0.0, 1.0);
    const auto thermalEfficiency = compressionEfficiency * afrEfficiency * timingEfficiency * damageEfficiency
        * std::clamp(0.72 + burnedFraction * 0.28, 0.60, 1.0);
    const auto workPerCycleJ = fuelMassMg * 1.0e-6 * fuelEnergyJPerKg * thermalEfficiency;
    const auto indicatedTorque = combustionEnabled ? workPerCycleJ / (4.0 * std::numbers::pi) : 0.0;

    // Knock is resolved in EngineSimulator by the Livengood-Wu end-gas model
    // from each cylinder's actual pressure and temperature. The mean-value
    // policy must not create an independent empirical knock signal.
    constexpr double knock = 0.0;
    const auto mixtureMisfire = std::max(0.0, std::abs(actualAfr - 14.0) - 2.6) * 0.055;
    const auto lowSpeedMisfire = std::max(0.0, 420.0 - state.rpm) / 1'500.0;
    const auto misfire = combustionEnabled ? std::clamp(mixtureMisfire + lowSpeedMisfire + knock * 0.07, 0.0, 0.85) : 0.0;
    const auto displacementM3 = std::max(1.0e-6, displacedVolumeM3);
    const auto imepBar = indicatedTorque * 4.0 * std::numbers::pi / displacementM3 / 100'000.0;
    const auto heatOutput = combustionEnabled
        ? std::clamp(fuelMassMg / std::max(1.0, displacedVolumeM3 * 100'000.0), 0.0, 1.5) : 0.0;
    const auto cyclesPerSecond = state.rpm / 120.0;
    const auto fuelPowerKw = combustionEnabled
        ? fuelMassMg * 1.0e-6 * cyclesPerSecond * fuelEnergyJPerKg / 1'000.0 : 0.0;
    const auto heatPowerKw = fuelPowerKw * (1.0 - thermalEfficiency);
    (void)controls;
    // Telemetry, not the crank-driving model. Each field is annotated with its
    // sole downstream reader (measured in Phase 6; see CombustionResult in
    // EngineTypes.hpp and docs/physics-audit.md). Designated init keeps the wiring
    // legible; order still matches the struct declaration (C++20 requirement).
    return CombustionResult {
        .indicatedTorqueNm = indicatedTorque,                                   // -> meanWorkTorqueNm display
        .pressureEstimateBar = combustionEnabled
            ? std::clamp(std::max(imepBar * 5.5, idealPressureBar), 2.0, 160.0)
            : 0.0,                                                              // -> event-gen pressure fallback
        .combustionQuality = combustionEnabled ? afrEfficiency * timingEfficiency : 0.0, // -> audio amplitude
        .heatOutput = heatOutput,                                               // -> cylinder wall temperature
        .knockLevel = knock,                                                    // unread (knock resolved per-cylinder)
        .misfireProbability = misfire,                                          // -> event-gen misfire fallback
        .volumetricEfficiency = volumetricEfficiency,                           // unread (telemetry)
        .airMassMgPerCycle = airMassMg,                                         // -> breathingQuality display
        .fuelMassMgPerCycle = fuelMassMg,                                       // unread (telemetry)
        .thermalEfficiency = thermalEfficiency,                                 // unread (telemetry)
        .actualAirFuelRatio = actualAfr,                                        // read only by EngineLab.Core
        .heatPowerKw = heatPowerKw,                                             // read only by EngineLab.Core
        .combustionEnabled = combustionEnabled,                                 // gates injection/combustion/event-gen
    };
}
} // namespace enginelab
