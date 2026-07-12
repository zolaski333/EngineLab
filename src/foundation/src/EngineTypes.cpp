#include <enginelab/foundation/EngineTypes.hpp>
#include <algorithm>
#include <cmath>
#include <numbers>
#include <unordered_set>
#include <limits>

namespace enginelab {

namespace {
EngineConfig makeEngine(std::string name, EngineLayout layout, std::uint32_t count,
                        std::vector<std::uint32_t> order, double bore, double stroke,
                        double idle, double redline, double inertia, CamshaftConfig cams) {
    EngineConfig config;
    config.name = std::move(name);
    config.layout = layout;
    config.idleRpm = idle;
    config.redlineRpm = redline;
    config.rotatingInertiaKgM2 = inertia;
    config.plenumVolumeLitres = std::max(0.6, static_cast<double>(count) * 0.70);
    config.throttleDiameterMm = std::clamp(36.0 + static_cast<double>(count) * 4.5, 42.0, 82.0);
    config.bankAngleDegrees = layout == EngineLayout::vLayout ? 60.0
        : (layout == EngineLayout::flat ? 180.0 : 0.0);
    config.camshafts = cams;
    config.intake.plenumVolumeLitres = config.plenumVolumeLitres;
    config.intake.throttleDiameterMm = config.throttleDiameterMm;
    config.ignition.revLimitRpm = redline;
    config.cylinders.reserve(count);
    const auto spacing = 720.0 / static_cast<double>(count);
    for (std::uint32_t id = 1; id <= count; ++id) {
        CylinderConfig cylinder;
        cylinder.id = id;
        cylinder.boreMm = bore;
        cylinder.strokeMm = stroke;
        cylinder.efficiencyOffset = (static_cast<double>(id) - (static_cast<double>(count) + 1.0) * 0.5) * 0.003;
        const auto it = std::find(order.begin(), order.end(), id);
        if (it != order.end()) {
            const auto orderIndex = static_cast<double>(std::distance(order.begin(), it));
            cylinder.crankOffsetDegrees = orderIndex * spacing;
        } else {
            cylinder.crankOffsetDegrees = 0.0;
        }
        cylinder.crankJournalId = id;
        if (layout == EngineLayout::vLayout) {
            cylinder.bankOffsetDegrees = id % 2U == 0U ? config.bankAngleDegrees * 0.5 : -config.bankAngleDegrees * 0.5;
        } else if (layout == EngineLayout::flat) {
            cylinder.bankOffsetDegrees = id % 2U == 0U ? 90.0 : -90.0;
        }
        config.crankJournals.push_back({ id, cylinder.crankOffsetDegrees, stroke * 0.5 });
        config.cylinders.push_back(cylinder);
    }
    config.firingOrder = std::move(order);
    if (layout == EngineLayout::vLayout || layout == EngineLayout::flat) {
        CylinderBankConfig left { 1, -config.bankAngleDegrees * 0.5, {}, cams, 0, 1 };
        CylinderBankConfig right { 2, config.bankAngleDegrees * 0.5, {}, cams, 0, 1 };
        for (const auto& cylinder : config.cylinders)
            (cylinder.id % 2U == 0U ? right.cylinderIds : left.cylinderIds).push_back(cylinder.id);
        config.banks = { std::move(left), std::move(right) };
    } else {
        CylinderBankConfig bank { 1, 0.0, {}, cams, 0, 1 };
        for (const auto& cylinder : config.cylinders) bank.cylinderIds.push_back(cylinder.id);
        config.banks.push_back(std::move(bank));
    }
    ExhaustPathConfig path;
    path.id = 1;
    path.geometry = config.exhaust;
    for (const auto& cylinder : config.cylinders) path.cylinderIds.push_back(cylinder.id);
    config.exhaustPaths.push_back(std::move(path));
    return config;
}
}

EngineConfig makeDefaultInlineFour() {
    return makeEngine("EL-20 I4", EngineLayout::inlineLayout, 4, { 1, 3, 4, 2 },
                      86.0, 86.0, 850.0, 7'200.0, 0.24, {});
}

EngineConfig makeDefaultInlineTwo() {
    return makeEngine("EL-09 I2", EngineLayout::inlineLayout, 2, { 1, 2 },
                      84.0, 81.0, 1'050.0, 8'200.0, 0.13, { 262.0, 256.0, 10.8, 10.3, 108.0, 110.0 });
}

EngineConfig makeDefaultInlineFive() {
    return makeEngine("EL-25 I5", EngineLayout::inlineLayout, 5, { 1, 2, 4, 5, 3 },
                      82.5, 92.8, 780.0, 6'800.0, 0.29, { 252.0, 248.0, 10.5, 10.0, 110.0, 112.0 });
}

EngineConfig makeDefaultV6() {
    return makeEngine("EL-30 V6", EngineLayout::vLayout, 6, { 1, 4, 2, 5, 3, 6 },
                      86.0, 86.0, 750.0, 7'000.0, 0.32, { 256.0, 252.0, 11.0, 10.5, 109.0, 111.0 });
}

EngineConfig makeDefaultV8() {
    auto config = makeEngine("EL-50 V8", EngineLayout::vLayout, 8, { 1, 5, 4, 8, 6, 3, 7, 2 },
                             94.0, 90.0, 720.0, 6'600.0, 0.46, { 268.0, 264.0, 12.0, 11.5, 108.0, 110.0 });
    config.frictionCoefficient = 0.15;
    config.bankAngleDegrees = 90.0;
    for (auto& cylinder : config.cylinders)
        cylinder.bankOffsetDegrees = cylinder.id % 2U == 0U ? 45.0 : -45.0;
    if (config.banks.size() == 2) {
        config.banks[0].angleDegrees = -45.0;
        config.banks[1].angleDegrees = 45.0;
    }
    return config;
}

EngineConfig makeDefaultFlatSix() {
    auto config = makeEngine("EL-36 F6", EngineLayout::flat, 6, { 1, 6, 2, 4, 3, 5 },
                             97.0, 81.5, 780.0, 7'600.0, 0.33,
                             { 270.0, 266.0, 12.3, 11.9, 106.0, 109.0 });
    config.frictionCoefficient = 0.13;
    config.plenumVolumeLitres = 4.2;
    config.throttleDiameterMm = 74.0;
    config.exhaust = { 720.0, 42.0, 66.0, 0.22, 74.0 };
    config.transmission = { { 3.82, 2.20, 1.52, 1.22, 1.02, 0.84 }, 3.44, 980.0 };
    config.vehicle = { 1'430.0, 0.30, 2.04, 0.325, 0.013 };
    return config;
}

EngineConfig makeDefaultRadialFive() {
    EngineConfig config;
    config.name = "EL-R5 Radial";
    config.layout = EngineLayout::radial;
    config.firingOrder = { 1, 3, 5, 2, 4 };
    config.idleRpm = 640.0;
    config.redlineRpm = 3'200.0;
    config.rotatingInertiaKgM2 = 0.86;
    config.frictionCoefficient = 0.19;
    config.octaneRating = 92.0;
    config.coolingEfficiency = 2.6;
    config.plenumVolumeLitres = 6.4;
    config.throttleDiameterMm = 82.0;
    config.bankAngleDegrees = 72.0;
    config.camshafts = { 246.0, 250.0, 11.6, 11.2, 106.0, 110.0 };
    config.exhaust = { 920.0, 54.0, 96.0, 0.08, 110.0 };
    config.transmission = { { 1.0 }, 1.0, 6'800.0 };
    config.vehicle = { 1'000.0, 0.30, 2.0, 0.320, 0.014 };
    config.thermal = { 820.0, 430.0, 0.24, 0.16, 0.92, 0.46 };
    config.crankJournals.push_back({ 1, 0.0, 63.5 });
    constexpr std::uint32_t count = 5;
    for (std::uint32_t id = 1; id <= count; ++id) {
        CylinderConfig cylinder;
        cylinder.id = id;
        cylinder.boreMm = 114.0;
        cylinder.strokeMm = 127.0;
        cylinder.connectingRodMm = 228.0;
        cylinder.pistonMassGrams = 1'050.0;
        cylinder.compressionRatio = 7.0;
        cylinder.efficiencyOffset = (static_cast<double>(id) - 3.0) * 0.004;
        const auto order = std::find(config.firingOrder.begin(), config.firingOrder.end(), id);
        cylinder.crankOffsetDegrees = order == config.firingOrder.end() ? 0.0
            : static_cast<double>(std::distance(config.firingOrder.begin(), order)) * (720.0 / static_cast<double>(count));
        cylinder.crankJournalId = 1;
        cylinder.bankOffsetDegrees = static_cast<double>(id - 1U) * (360.0 / static_cast<double>(count));
        config.cylinders.push_back(cylinder);
    }
    return config;
}

std::vector<EngineConfig> makeBaseEnginePresets() {
    return { makeDefaultInlineTwo(), makeDefaultInlineFour(), makeDefaultInlineFive(), makeDefaultV6(),
             makeDefaultV8(), makeDefaultFlatSix(), makeDefaultRadialFive() };
}

double engineDisplacementLitres(const EngineConfig& config) noexcept {
    double cubicMillimetres = 0.0;
    for (const auto& cylinder : config.cylinders)
        cubicMillimetres += std::numbers::pi * cylinder.boreMm * cylinder.boreMm * 0.25 * cylinder.strokeMm;
    return cubicMillimetres / 1'000'000.0;
}

double valveLiftMm(double crankAngleDegrees, double centerlineDegrees,
                   double durationDegrees, double maximumLiftMm) noexcept {
    const auto wrapped = std::remainder(crankAngleDegrees - centerlineDegrees, 720.0);
    const auto halfDuration = std::max(1.0, durationDegrees * 0.5);
    if (std::abs(wrapped) >= halfDuration) return 0.0;
    const auto phase = (wrapped + halfDuration) / durationDegrees;
    return std::max(0.0, maximumLiftMm * 0.5 * (1.0 - std::cos(phase * 2.0 * std::numbers::pi)));
}

double profiledValveLiftMm(double crankAngleDegrees, double centerlineDegrees,
                           double durationDegrees, double maximumLiftMm,
                           const std::vector<ValveLiftSample>& samples) noexcept {
    if (samples.size() < 2) return valveLiftMm(crankAngleDegrees, centerlineDegrees, durationDegrees, maximumLiftMm);
    const auto relativeAngle = std::remainder(crankAngleDegrees - centerlineDegrees, 720.0);
    const auto halfDuration = std::max(1.0, durationDegrees * 0.5);
    if (std::abs(relativeAngle) >= halfDuration) return 0.0;
    const auto firstAngle = samples.front().angleDegrees;
    const auto lastAngle = samples.back().angleDegrees;
    if (relativeAngle <= firstAngle) return std::clamp(samples.front().liftMm, 0.0, maximumLiftMm);
    if (relativeAngle >= lastAngle) return std::clamp(samples.back().liftMm, 0.0, maximumLiftMm);
    for (std::size_t index = 1; index < samples.size(); ++index) {
        const auto& left = samples[index - 1];
        const auto& right = samples[index];
        if (relativeAngle <= right.angleDegrees) {
            const auto span = std::max(1.0e-9, right.angleDegrees - left.angleDegrees);
            const auto t = std::clamp((relativeAngle - left.angleDegrees) / span, 0.0, 1.0);
            return std::clamp(std::lerp(left.liftMm, right.liftMm, t), 0.0, maximumLiftMm);
        }
    }
    return 0.0;
}

double combustionPulse(double cylinderPhaseDegrees, double ignitionAdvanceDegrees,
                       double ignitionOffsetDegrees) noexcept {
    const auto pressurePhase = std::fmod(cylinderPhaseDegrees + ignitionAdvanceDegrees * 0.35
        - ignitionOffsetDegrees + 720.0, 720.0);
    return pressurePhase < 180.0
        ? std::pow(std::sin(pressurePhase * std::numbers::pi / 180.0), 2.0) : 0.0;
}

std::optional<std::string> validateEngineConfig(const EngineConfig& config) {
    const auto inRange = [](double value, double minimum, double maximum) {
        return std::isfinite(value) && value >= minimum && value <= maximum;
    };
    if (config.schemaVersion != 1) return "Unsupported engine schema version";
    if (config.name.empty() || config.name.size() > 128) return "Engine name must contain between 1 and 128 bytes";
    if (config.cycle != EngineCycle::fourStroke || config.fuel != FuelType::gasoline)
        return "Only four-stroke gasoline engines are currently supported";
    if (config.fuelProperties.name.empty() || config.fuelProperties.name.size() > 128
        || !inRange(config.fuelProperties.lowerHeatingValueMjPerKg, 10.0, 60.0)
        || !inRange(config.fuelProperties.densityKgPerL, 0.30, 1.50)
        || !inRange(config.fuelProperties.stoichiometricAirFuelRatio, 5.0, 25.0)
        || !inRange(config.fuelProperties.molarMassGramsPerMole, 20.0, 300.0)
        || !inRange(config.fuelProperties.oxygenMolesPerFuelMole, 1.0, 40.0)
        || !inRange(config.fuelProperties.productMolesPerFuelMole, 1.0, 60.0)
        || !inRange(config.fuelProperties.laminarFlameSpeedMps, 0.05, 2.0)
        || !inRange(config.fuelProperties.turbulenceFlameSpeedGain, 0.0, 10.0))
        return "Fuel properties must define finite gasoline chemistry and thermodynamic values";
    const auto chemistryStoichiometricAfr = config.fuelProperties.oxygenMolesPerFuelMole * 31.9988
        / config.fuelProperties.molarMassGramsPerMole / 0.232;
    if (std::abs(config.fuelProperties.stoichiometricAirFuelRatio / chemistryStoichiometricAfr - 1.0) > 0.25)
        return "Fuel stoichiometric AFR is inconsistent with its molar oxygen requirement";
    if (config.cylinders.empty() || config.cylinders.size() > 32 || config.firingOrder.size() != config.cylinders.size())
        return "Firing order must reference between 1 and 32 cylinders";
    if (!inRange(config.idleRpm, 200.0, 3'000.0)
        || !inRange(config.redlineRpm, config.idleRpm + 100.0, 20'000.0)
        || !inRange(config.rotatingInertiaKgM2, 0.01, 5.0)
        || !inRange(config.frictionCoefficient, 0.0, 1.0)
        || !inRange(config.octaneRating, 70.0, 130.0)
        || !inRange(config.ambientPressureKpa, 50.0, 120.0)
        || !inRange(config.ambientTemperatureC, -50.0, 60.0)
        || !inRange(config.coolingEfficiency, 0.1, 5.0)
        || !inRange(config.plenumVolumeLitres, 0.1, 50.0)
        || !inRange(config.throttleDiameterMm, 15.0, 150.0)
        || !inRange(config.bankAngleDegrees, 0.0, 180.0)
        || !inRange(config.forcedInduction.pressureRatio, 1.0, 3.5)
        || !inRange(config.forcedInduction.fullBoostRpm, 200.0, 20'000.0)
        || !inRange(config.forcedInduction.compressorEfficiency, 0.35, 0.95)
        || !inRange(config.forcedInduction.chargeTemperatureRiseC, 0.0, 160.0)
        || !inRange(config.thermal.coolantMassKjPerC, 20.0, 5'000.0)
        || !inRange(config.thermal.oilMassKjPerC, 10.0, 2'500.0)
        || !inRange(config.thermal.coolantHeatShare, 0.0, 1.0)
        || !inRange(config.thermal.oilHeatShare, 0.0, 1.0)
        || !inRange(config.thermal.coolingPowerKwPerC, 0.02, 20.0)
        || !inRange(config.thermal.oilCoolingPowerKwPerC, 0.01, 10.0)
        || !inRange(config.camshafts.intakeDurationDegrees, 1.0, 720.0)
        || !inRange(config.camshafts.exhaustDurationDegrees, 1.0, 720.0)
        || !inRange(config.camshafts.intakeLiftMm, 0.0, 30.0)
        || !inRange(config.camshafts.exhaustLiftMm, 0.0, 30.0)
        || !inRange(config.camshafts.intakeCenterlineDegrees, 0.0, 720.0)
        || !inRange(config.camshafts.exhaustCenterlineDegrees, 0.0, 720.0)
        || !inRange(config.camshafts.intakeFlowCoefficient, 0.05, 1.50)
        || !inRange(config.camshafts.exhaustFlowCoefficient, 0.05, 1.50)
        || !inRange(config.exhaust.primaryLengthMm, 100.0, 3'000.0)
        || !inRange(config.exhaust.primaryDiameterMm, 15.0, 200.0)
        || !inRange(config.exhaust.collectorDiameterMm, 15.0, 300.0)
        || !inRange(config.exhaust.outletDiameterMm, 15.0, 300.0)
        || !inRange(config.exhaust.mufflerRestriction, 0.0, 1.0)
        || !inRange(config.exhaust.collectorVolumeLitres, 0.05, 200.0)
        || !inRange(config.exhaust.outletDischargeCoefficient, 0.02, 1.5)
        || !inRange(config.transmission.finalDriveRatio, 1.0, 8.0)
        || !inRange(config.transmission.maxClutchTorqueNm, 10.0, 10'000.0)
        || !inRange(config.vehicle.massKg, 50.0, 20'000.0)
        || !inRange(config.vehicle.dragCoefficient, 0.05, 2.0)
        || !inRange(config.vehicle.frontalAreaM2, 0.1, 20.0)
        || !inRange(config.vehicle.tireRadiusM, 0.05, 2.0)
        || !inRange(config.vehicle.rollingResistanceCoefficient, 0.0, 0.20)
        || !inRange(config.intake.plenumVolumeLitres, 0.1, 50.0)
        || !inRange(config.intake.throttleDiameterMm, 15.0, 150.0)
        || !inRange(config.intake.throttleDischargeCoefficient, 0.05, 1.5)
        || !inRange(config.intake.runnerLengthMm, 20.0, 2'000.0)
        || !inRange(config.intake.runnerDiameterMm, 10.0, 150.0)
        || !inRange(config.intake.idleBypassAreaMm2, 0.0, 1'000.0)
        || !inRange(config.intake.throttleGamma, 0.2, 5.0)
        || !inRange(config.ignition.revLimitRpm, config.idleRpm + 100.0, 25'000.0)
        || !inRange(config.ignition.limiterDurationSeconds, 0.005, 2.0)
        || !inRange(config.injection.startAngleDegrees, 0.0, 720.0)
        || !inRange(config.injection.endAngleDegrees, 0.0, 720.0)
        || std::abs(config.injection.startAngleDegrees - config.injection.endAngleDegrees) < 1.0
        || !inRange(config.injection.injectorFlowMgPerSecond, 10.0, 500'000.0)
        || !inRange(config.injection.fuelTemperatureC, -50.0, 180.0)
        || !inRange(config.injection.railPressureBar, 1.2, 3'000.0)
        || !inRange(config.injection.referencePressureBar, 1.2, 3'000.0)
        || !inRange(config.injection.wallFilmFraction, 0.0, 0.98)
        || !inRange(config.injection.vaporisationTimeConstantSeconds, 0.001, 2.0)
        || !inRange(config.injection.latentHeatKjPerKg, 10.0, 1'000.0)
        || !inRange(config.injection.directChargeCoolingEfficiency, 0.0, 1.0)
        || !inRange(config.injection.portChargeCoolingEfficiency, 0.0, 1.0)
        || !inRange(config.solver.mechanicalFrequencyHz, 240.0, 50'000.0)
        || !inRange(config.solver.maximumMechanicalFrequencyHz, config.solver.mechanicalFrequencyHz, 100'000.0)
        || !inRange(config.solver.maximumCrankDegreesPerStep, 0.1, 30.0)
        || config.solver.gasSubsteps < 1 || config.solver.gasSubsteps > 32)
        return "Engine configuration contains non-finite or physically invalid values";
    const auto maximumCalibratedRpm = std::max(config.redlineRpm, config.ignition.revLimitRpm);
    const auto requiredAngleFrequency = maximumCalibratedRpm * 6.0
        / config.solver.maximumCrankDegreesPerStep;
    const auto requiredSolverFrequency = std::max(requiredAngleFrequency,
        config.solver.mechanicalFrequencyHz * static_cast<double>(config.solver.gasSubsteps));
    if (config.solver.maximumMechanicalFrequencyHz + 1.0e-9 < requiredSolverFrequency)
        return "Solver maximum frequency cannot satisfy the configured crank-angle resolution at the rev limit";
    if (config.transmission.gearRatios.empty() || config.transmission.gearRatios.size() > 12)
        return "Transmission must contain between 1 and 12 forward gears";
    for (const auto gearRatio : config.transmission.gearRatios)
        if (!inRange(gearRatio, 0.05, 10.0)) return "Transmission gear ratios must be finite and positive";
    if (config.crankJournals.size() > 64) return "Crank journal table must contain at most 64 entries";
    std::unordered_set<std::uint32_t> journalIds;
    for (const auto& journal : config.crankJournals) {
        if (journal.id == 0 || !inRange(journal.angleDegrees, -360.0, 720.0)
            || !inRange(journal.throwMm, 1.0, 120.0))
            return "Crank journals must have finite IDs, angles and throws";
        journalIds.insert(journal.id);
    }
    if (journalIds.size() != config.crankJournals.size()) return "Crank journal IDs must be unique";
    const auto validateLiftProfile = [&inRange](const std::vector<ValveLiftSample>& profile, double maxLift) {
        if (profile.size() > 64) return false;
        double previous = -std::numeric_limits<double>::infinity();
        for (const auto& sample : profile) {
            if (!inRange(sample.angleDegrees, -360.0, 360.0) || !inRange(sample.liftMm, 0.0, maxLift)) return false;
            if (sample.angleDegrees <= previous) return false;
            previous = sample.angleDegrees;
        }
        return true;
    };
    const auto validateCamshaft = [&inRange, &validateLiftProfile](const CamshaftConfig& cam) {
        return inRange(cam.intakeDurationDegrees, 1.0, 720.0)
            && inRange(cam.exhaustDurationDegrees, 1.0, 720.0)
            && inRange(cam.intakeLiftMm, 0.0, 30.0)
            && inRange(cam.exhaustLiftMm, 0.0, 30.0)
            && inRange(cam.intakeCenterlineDegrees, 0.0, 720.0)
            && inRange(cam.exhaustCenterlineDegrees, 0.0, 720.0)
            && inRange(cam.intakeFlowCoefficient, 0.05, 1.50)
            && inRange(cam.exhaustFlowCoefficient, 0.05, 1.50)
            && inRange(cam.switchRpm, 200.0, 25'000.0)
            && inRange(cam.switchThrottle, 0.0, 1.0)
            && inRange(cam.highIntakeDurationDegrees, 1.0, 720.0)
            && inRange(cam.highExhaustDurationDegrees, 1.0, 720.0)
            && inRange(cam.highIntakeLiftMm, 0.0, 30.0)
            && inRange(cam.highExhaustLiftMm, 0.0, 30.0)
            && validateLiftProfile(cam.intakeLiftProfile, cam.intakeLiftMm)
            && validateLiftProfile(cam.exhaustLiftProfile, cam.exhaustLiftMm)
            && validateLiftProfile(cam.highIntakeLiftProfile, cam.highIntakeLiftMm)
            && validateLiftProfile(cam.highExhaustLiftProfile, cam.highExhaustLiftMm);
    };
    if (!validateCamshaft(config.camshafts))
        return "Valve lift profiles must be sorted finite tables within configured max lift";
    if (config.ignition.timingCurve.empty() || config.ignition.timingCurve.size() > 128)
        return "Ignition timing curve must contain between 1 and 128 samples";
    double previousIgnitionRpm = -1.0;
    for (const auto& sample : config.ignition.timingCurve) {
        if (!inRange(sample.rpm, 0.0, 25'000.0) || !inRange(sample.advanceDegrees, -30.0, 80.0)
            || sample.rpm <= previousIgnitionRpm)
            return "Ignition timing curve must be finite and strictly ordered by RPM";
        previousIgnitionRpm = sample.rpm;
    }
    std::unordered_set<std::uint32_t> cylinderIds;
    constexpr std::uint32_t mergeId = std::numeric_limits<std::uint32_t>::max() - 2U;
    for (const auto& cylinder : config.cylinders) {
        if (cylinder.id == 0 || cylinder.id >= mergeId || !inRange(cylinder.boreMm, 20.0, 200.0)
            || !inRange(cylinder.strokeMm, 20.0, 200.0)
            || !inRange(cylinder.connectingRodMm, cylinder.strokeMm * 0.5 + 0.01, 500.0)
            || !inRange(cylinder.pistonMassGrams, 50.0, 2'500.0)
            || !inRange(cylinder.compressionRatio, 5.0, 20.0)
            || !inRange(cylinder.ignitionOffsetDegrees, -30.0, 30.0)
            || !inRange(cylinder.efficiencyOffset, -0.5, 0.5)
            || !inRange(cylinder.crankOffsetDegrees, -360.0, 720.0)
            || !inRange(cylinder.bankOffsetDegrees, -360.0, 360.0)
            || !inRange(cylinder.intakeRunnerLengthMm, 20.0, 2'000.0)
            || !inRange(cylinder.intakeRunnerDiameterMm, 10.0, 150.0)
            || !inRange(cylinder.exhaustPrimaryLengthMm, 0.0, 3'000.0)
            || !inRange(cylinder.soundAttenuation, 0.0, 4.0)
            || !inRange(cylinder.blowByCoefficient, 0.0, 0.1)
            || !inRange(cylinder.connectingRodMassGrams, 20.0, 5'000.0)
            || !inRange(cylinder.pistonFrictionCoefficient, 0.0, 0.5)
            || !inRange(cylinder.pistonBreakawayForceN, 0.0, 5'000.0)
            || !inRange(cylinder.pistonBreakawayVelocityMps, 0.001, 5.0)
            || !inRange(cylinder.pistonViscousFrictionNsPerM, 0.0, 2'000.0))
            return "Cylinder IDs and dimensions must be finite and physically valid";
        if (cylinder.crankJournalId != 0 && !journalIds.empty() && !journalIds.contains(cylinder.crankJournalId))
            return "Cylinder crankJournalId must reference a configured crank journal";
        cylinderIds.insert(cylinder.id);
    }
    const std::unordered_set<std::uint32_t> firingIds(config.firingOrder.begin(), config.firingOrder.end());
    if (cylinderIds.size() != config.cylinders.size()) return "Cylinder IDs must be unique";
    if (firingIds != cylinderIds) return "Firing order must contain every cylinder ID exactly once";
    if (config.banks.size() > 32) return "Bank topology may contain at most 32 banks";
    std::unordered_set<std::uint32_t> assignedBankCylinders;
    std::unordered_set<std::uint32_t> bankIds;
    for (const auto& bank : config.banks) {
        if (bank.id == 0 || bank.cylinderIds.empty() || !bankIds.insert(bank.id).second || !inRange(bank.angleDegrees, -180.0, 180.0)
            || !validateCamshaft(bank.camshafts)) return "Bank IDs, angles and camshafts must be valid";
        for (const auto cylinderId : bank.cylinderIds) {
            if (!cylinderIds.contains(cylinderId) || !assignedBankCylinders.insert(cylinderId).second)
                return "Bank cylinder references must be unique and valid";
        }
    }
    if (!config.banks.empty() && assignedBankCylinders != cylinderIds)
        return "Configured banks must assign every cylinder exactly once";
    if (config.exhaustPaths.size() > 32) return "Exhaust topology may contain at most 32 paths";
    std::unordered_set<std::uint32_t> exhaustPathIds;
    std::unordered_set<std::uint32_t> assignedExhaustCylinders;
    for (const auto& path : config.exhaustPaths) {
        const auto& exhaust = path.geometry;
        if (path.id == 0 || path.cylinderIds.empty() || !exhaustPathIds.insert(path.id).second || !inRange(path.audioVolume, 0.0, 8.0)
            || path.impulseResponsePath.size() > 1'024
            || !inRange(exhaust.primaryLengthMm, 100.0, 3'000.0)
            || !inRange(exhaust.primaryDiameterMm, 15.0, 200.0)
            || !inRange(exhaust.collectorDiameterMm, 15.0, 300.0)
            || !inRange(exhaust.outletDiameterMm, 15.0, 300.0)
            || !inRange(exhaust.mufflerRestriction, 0.0, 1.0)
            || !inRange(exhaust.collectorVolumeLitres, 0.05, 200.0)
            || !inRange(exhaust.outletDischargeCoefficient, 0.02, 1.5))
            return "Exhaust path IDs and audio volumes must be valid";
        for (const auto cylinderId : path.cylinderIds)
            if (!cylinderIds.contains(cylinderId) || !assignedExhaustCylinders.insert(cylinderId).second)
                return "Exhaust paths must reference every cylinder at most once";
    }
    if (!config.exhaustPaths.empty() && assignedExhaustCylinders != cylinderIds)
        return "Configured exhaust paths must assign every cylinder exactly once";
    for (const auto& bank : config.banks)
        if (!config.exhaustPaths.empty() && bank.exhaustPathId != 0 && !exhaustPathIds.contains(bank.exhaustPathId))
            return "Bank exhaustPathId must reference a configured exhaust path";
    const auto displacement = engineDisplacementLitres(config);
    if (!inRange(displacement, 0.05, 20.0)) return "Total displacement must be between 0.05 and 20 litres";
    return std::nullopt;
}

} // namespace enginelab
