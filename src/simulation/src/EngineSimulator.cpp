#include <enginelab/simulation/EngineSimulator.hpp>
#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace enginelab {
namespace {
[[nodiscard]] double smooth(double current, double target, double dt, double rate) noexcept {
    return current + (target - current) * (1.0 - std::exp(-dt * rate));
}

[[nodiscard]] double valveAreaMm2(double boreMm, double liftMm) noexcept {
    const auto valveDiameterMm = std::clamp(boreMm * 0.34, 16.0, 58.0);
    return std::numbers::pi * valveDiameterMm * std::max(0.0, liftMm);
}

[[nodiscard]] double chamberVolumeLitres(const CylinderConfig& cylinder, double phaseDegrees) noexcept {
    const auto boreM = cylinder.boreMm * 0.001;
    const auto strokeM = cylinder.strokeMm * 0.001;
    const auto crankRadiusM = strokeM * 0.5;
    const auto rodM = cylinder.connectingRodMm * 0.001;
    const auto theta = phaseDegrees * std::numbers::pi / 180.0;
    const auto rodRoot = std::sqrt(std::max(0.0, rodM * rodM
        - crankRadiusM * crankRadiusM * std::sin(theta) * std::sin(theta)));
    const auto pistonTravelM = crankRadiusM * (1.0 - std::cos(theta)) + rodM - rodRoot;
    const auto sweptM3 = std::numbers::pi * boreM * boreM * 0.25 * strokeM;
    const auto clearanceM3 = sweptM3 / std::max(1.0, cylinder.compressionRatio - 1.0);
    return (clearanceM3 + std::numbers::pi * boreM * boreM * 0.25 * pistonTravelM) * 1'000.0;
}

[[nodiscard]] double crankThrowMmFor(const EngineConfig& config, const CylinderConfig& cylinder) noexcept {
    if (cylinder.crankJournalId != 0) {
        const auto journal = std::find_if(config.crankJournals.begin(), config.crankJournals.end(),
            [&cylinder](const CrankJournalConfig& item) { return item.id == cylinder.crankJournalId; });
        if (journal != config.crankJournals.end()) return journal->throwMm;
    }
    return cylinder.strokeMm * 0.5;
}

[[nodiscard]] double crankOffsetDegreesFor(const EngineConfig& config, const CylinderConfig& cylinder) noexcept {
    (void)config;
    return cylinder.crankOffsetDegrees;
}

struct ActiveCamshaft final {
    const CamshaftConfig* config {};
    bool highProfile { false };
    [[nodiscard]] double intakeDuration() const noexcept { return highProfile ? config->highIntakeDurationDegrees : config->intakeDurationDegrees; }
    [[nodiscard]] double exhaustDuration() const noexcept { return highProfile ? config->highExhaustDurationDegrees : config->exhaustDurationDegrees; }
    [[nodiscard]] double intakeLift() const noexcept { return highProfile ? config->highIntakeLiftMm : config->intakeLiftMm; }
    [[nodiscard]] double exhaustLift() const noexcept { return highProfile ? config->highExhaustLiftMm : config->exhaustLiftMm; }
    [[nodiscard]] const std::vector<ValveLiftSample>& intakeProfile() const noexcept {
        return highProfile && !config->highIntakeLiftProfile.empty() ? config->highIntakeLiftProfile : config->intakeLiftProfile;
    }
    [[nodiscard]] const std::vector<ValveLiftSample>& exhaustProfile() const noexcept {
        return highProfile && !config->highExhaustLiftProfile.empty() ? config->highExhaustLiftProfile : config->exhaustLiftProfile;
    }
};

[[nodiscard]] ActiveCamshaft activeCamshaft(const EngineConfig& config, const CylinderConfig& cylinder,
                                             double rpm, double throttle) noexcept {
    const CamshaftConfig* selected = &config.camshafts;
    const auto bank = std::find_if(config.banks.begin(), config.banks.end(), [&cylinder](const CylinderBankConfig& item) {
        return item.id == cylinder.bankId
            || std::find(item.cylinderIds.begin(), item.cylinderIds.end(), cylinder.id) != item.cylinderIds.end();
    });
    if (bank != config.banks.end()) selected = &bank->camshafts;
    const auto high = selected->variableProfileEnabled && rpm >= selected->switchRpm
        && throttle >= selected->switchThrottle;
    return { selected, high };
}
}

EngineSimulator::EngineSimulator(EngineConfig config, IEcuModel& ecu, IPhysicsModel& physics,
                                 IFiringEventGenerator& events, IExhaustModel& exhaust)
    : config_(std::move(config)), ecu_(ecu), physics_(physics), eventGenerator_(events), exhaust_(exhaust) {
    if (const auto error = validateEngineConfig(config_)) throw std::invalid_argument(*error);
    reset();
}

SimulationFrame EngineSimulator::step(double dtSeconds, const EngineControls& controls) noexcept {
    const auto dt = std::isfinite(dtSeconds) ? std::clamp(dtSeconds, 0.0, 0.05) : 0.0;
    if (dt <= 0.0) {
        return { state_ };
    }
    auto safeControls = controls;
    safeControls.throttle = std::isfinite(controls.throttle) ? std::clamp(controls.throttle, 0.0, 1.0) : 0.0;
    safeControls.load = std::isfinite(controls.load) ? std::clamp(controls.load, 0.0, 1.0) : 0.0;
    safeControls.externalTorqueNm = std::isfinite(controls.externalTorqueNm)
        ? std::clamp(controls.externalTorqueNm, -5'000.0, 5'000.0) : 0.0;
    const auto stepStartTime = state_.simulationTimeSeconds;
    const auto previousAngle = state_.crankAngleDegrees;

    // Gas pressure and crank loading need much finer resolution than UI/runtime
    // updates. Resolution increases with crank speed and remains bounded by the
    // engine definition so slow machines fail observably instead of diverging.
    const auto angleFrequency = std::abs(state_.rpm) * 6.0
        / std::max(0.1, config_.solver.maximumCrankDegreesPerStep);
    const auto solverFrequency = std::clamp(std::max(config_.solver.mechanicalFrequencyHz, angleFrequency)
        * static_cast<double>(config_.solver.gasSubsteps), config_.solver.mechanicalFrequencyHz,
        config_.solver.maximumMechanicalFrequencyHz);
    const auto maxSubStepDt = 1.0 / solverFrequency;
    const auto subStepCount = std::max(std::size_t { 1 }, static_cast<std::size_t>(std::ceil(dt / maxSubStepDt)));
    const auto subDt = dt / static_cast<double>(subStepCount);
    state_.solverFrequencyHz = solverFrequency;
    state_.solverSubsteps = static_cast<std::uint32_t>(std::min<std::size_t>(subStepCount,
        std::numeric_limits<std::uint32_t>::max()));
    double totalTravelled = 0.0;

    for (std::size_t subStep = 0; subStep < subStepCount; ++subStep) {
        const auto subPreviousRpm = state_.rpm;
        const auto subPreviousAngle = state_.crankAngleDegrees;

        state_.load = smooth(state_.load, safeControls.load, subDt, 12.0);
        const auto ecuCommand = ecu_.evaluate(config_, state_, safeControls);
        state_.throttle = smooth(state_.throttle, ecuCommand.effectiveThrottle, subDt, 10.0);
        const auto stationary = state_.rpm < 20.0 && !safeControls.starterEngaged;
        if (stationary) {
            state_.manifoldPressureKpa = smooth(state_.manifoldPressureKpa, config_.ambientPressureKpa, subDt, 13.0);
            manifoldGas_.reset(config_.ambientPressureKpa, config_.ambientTemperatureC + 273.15);
        } else {
            const auto boostRange = std::max(250.0, config_.forcedInduction.fullBoostRpm - config_.idleRpm);
            const auto boostBlend = config_.forcedInduction.enabled
                ? std::clamp((state_.rpm - config_.idleRpm * 0.75) / boostRange, 0.0, 1.0)
                    * std::clamp(std::pow(state_.throttle, 0.72), 0.0, 1.0)
                : 0.0;
            const auto pressureRatioTarget = 1.0 + (config_.forcedInduction.pressureRatio - 1.0) * boostBlend;
            const auto exhaustDrive = std::clamp(state_.exhaustFlowGramsPerSecond / 120.0, 0.0, 1.0);
            const auto spoolRate = 0.65 + exhaustDrive * 5.5 + state_.throttle * 1.8;
            state_.boostPressureRatio = smooth(state_.boostPressureRatio,
                config_.forcedInduction.enabled ? pressureRatioTarget : 1.0, subDt, spoolRate);
            const auto intakeSourcePressureKpa = config_.ambientPressureKpa * state_.boostPressureRatio;
            const auto chargeTemperatureC = config_.ambientTemperatureC
                + config_.forcedInduction.chargeTemperatureRiseC * boostBlend
                    / std::max(0.35, config_.forcedInduction.compressorEfficiency);
            const auto temperatureK = std::max(240.0, chargeTemperatureC + 273.15);

            const auto throttleRadiusM = config_.intake.throttleDiameterMm * 0.0005;
            const auto throttlePlateAreaM2 = std::numbers::pi * throttleRadiusM * throttleRadiusM;
            const auto throttleAreaM2 = throttlePlateAreaM2
                * (0.006 + 0.994 * std::pow(state_.throttle, config_.intake.throttleGamma));
            (void)ConservativeGasSystem::flowFromBoundary(manifoldGas_, intakeSourcePressureKpa,
                temperatureK, throttleAreaM2,
                config_.intake.throttleDischargeCoefficient, subDt);
        }
        state_.manifoldPressureKpa = manifoldGas_.pressureKpa();

        const auto backPressure = exhaust_.backPressureKpa(state_);
        const auto combustion = physics_.evaluateCombustion(config_, state_, safeControls, ecuCommand, backPressure);
        const auto cranking = safeControls.starterEngaged && state_.rpm < 620.0 && state_.damage < 1.0;
        const auto displacement = engineDisplacementLitres(config_);
        double pistonSpeedSum = 0.0;
        double reciprocatingTorque = 0.0;
        double peakPistonAcceleration = 0.0;
        double combustionPulseSum = 0.0;
        double gasIndicatedTorque = 0.0;
        double intakeRunnerPressureSum = 0.0;
        double exhaustRunnerPressureSum = 0.0;
        double exhaustFlowMgSum = 0.0;

        for (std::size_t cylinderIndex = 0; cylinderIndex < config_.cylinders.size(); ++cylinderIndex) {
            const auto& cylinder = config_.cylinders[cylinderIndex];
            const auto crankOffset = crankOffsetDegreesFor(config_, cylinder);
            const auto theta = (state_.crankAngleDegrees - crankOffset) * std::numbers::pi / 180.0;
            const auto cyclePhase = std::fmod(state_.crankAngleDegrees - crankOffset + 1'440.0, 720.0);
            const auto wrappedCycle = cyclePhase < previousCylinderPhases_[cylinderIndex];
            const auto reachedIgnitionWindow = !wrappedCycle && previousCylinderPhases_[cylinderIndex] < 600.0
                && cyclePhase >= 600.0;
            if (!combustion.combustionEnabled) {
                cylinderMisfires_[cylinderIndex] = false;
                cylinderMisfirePrepared_[cylinderIndex] = false;
            } else if (reachedIgnitionWindow || (wrappedCycle && !cylinderMisfirePrepared_[cylinderIndex])) {
                randomState_ ^= randomState_ << 13U;
                randomState_ ^= randomState_ >> 17U;
                randomState_ ^= randomState_ << 5U;
                const auto randomUnit = static_cast<double>(randomState_) / static_cast<double>(0xffffffffU);
                cylinderMisfires_[cylinderIndex] = randomUnit < combustion.misfireProbability;
                cylinderMisfirePrepared_[cylinderIndex] = true;
            }
            if (wrappedCycle) cylinderMisfirePrepared_[cylinderIndex] = false;
            previousCylinderPhases_[cylinderIndex] = cyclePhase;
            const auto pulse = combustion.combustionEnabled && !cylinderMisfires_[cylinderIndex]
                ? enginelab::combustionPulse(cyclePhase, ecuCommand.ignitionAdvanceDegrees,
                                             cylinder.ignitionOffsetDegrees) : 0.0;
            const auto bankedPhase = cyclePhase + cylinder.bankOffsetDegrees;
            const auto cams = activeCamshaft(config_, cylinder, state_.rpm, state_.throttle);
            const auto intakeLift = profiledValveLiftMm(bankedPhase, 360.0 + cams.config->intakeCenterlineDegrees,
                                                        cams.intakeDuration(), cams.intakeLift(), cams.intakeProfile());
            const auto exhaustLift = profiledValveLiftMm(bankedPhase, 720.0 - cams.config->exhaustCenterlineDegrees,
                                                         cams.exhaustDuration(), cams.exhaustLift(), cams.exhaustProfile());
            const auto chamberVolume = chamberVolumeLitres(cylinder, cyclePhase);
            cylinderGas_[cylinderIndex].setVolumeAdiabatic(chamberVolume);
            const auto profile = pulse;
            if (wrappedCycle) {
                lastCombustionProfile_[cylinderIndex] = 0.0;
                injectedFuelMolesThisCycle_[cylinderIndex] = 0.0;
            }
            if (combustion.combustionEnabled && !cylinderMisfires_[cylinderIndex]
                && injectedFuelMolesThisCycle_[cylinderIndex] <= 0.0
                && (wrappedCycle || cyclePhase > 570.0)) {
                constexpr double gasolineMolarMassKg = 0.11423;
                const auto fuelMoles = combustion.fuelMassMgPerCycle * 1.0e-6
                    / gasolineMolarMassKg / static_cast<double>(config_.cylinders.size());
                cylinderGas_[cylinderIndex].injectFuelMoles(fuelMoles,
                    std::max(293.15, manifoldGas_.temperatureK()));
                injectedFuelMolesThisCycle_[cylinderIndex] = fuelMoles;
            }
            const auto profileAdvance = std::max(0.0, profile - lastCombustionProfile_[cylinderIndex]);
            if (profileAdvance > 0.0 && combustion.combustionEnabled && !cylinderMisfires_[cylinderIndex])
                (void)ConservativeGasSystem::reactGasoline(cylinderGas_[cylinderIndex], profileAdvance,
                    std::clamp(combustion.thermalEfficiency * 2.25, 0.25, 0.88));
            lastCombustionProfile_[cylinderIndex] = profile;

            const auto chamberKpa = cylinderGas_[cylinderIndex].pressureKpa();
            chamberPressureBar_[cylinderIndex] = chamberKpa / 100.0;
            const auto intakeArea = valveAreaMm2(cylinder.boreMm, intakeLift);
            const auto exhaustArea = valveAreaMm2(cylinder.boreMm, exhaustLift);
            const auto runnerDiameterMm = cylinder.intakeRunnerDiameterMm > 0.0
                ? cylinder.intakeRunnerDiameterMm : config_.intake.runnerDiameterMm;
            const auto runnerAreaM2 = std::numbers::pi * std::pow(runnerDiameterMm * 0.0005, 2.0);
            const auto primaryAreaM2 = std::numbers::pi * std::pow(config_.exhaust.primaryDiameterMm * 0.0005, 2.0);
            (void)ConservativeGasSystem::flow(manifoldGas_, intakeRunnerGas_[cylinderIndex],
                runnerAreaM2, 0.78, subDt);
            const auto intakeTransfer = ConservativeGasSystem::flow(intakeRunnerGas_[cylinderIndex],
                cylinderGas_[cylinderIndex], intakeArea * 1.0e-6,
                cams.config->intakeFlowCoefficient, subDt);
            const auto exhaustTransfer = ConservativeGasSystem::flow(cylinderGas_[cylinderIndex],
                exhaustRunnerGas_[cylinderIndex], exhaustArea * 1.0e-6,
                cams.config->exhaustFlowCoefficient, subDt);
            (void)ConservativeGasSystem::flow(exhaustRunnerGas_[cylinderIndex], collectorGas_,
                primaryAreaM2, 0.74, subDt);
            intakeRunnerGas_[cylinderIndex].dissipateMomentum(
                std::max(0.004, cylinder.intakeRunnerLengthMm / 180'000.0), subDt);
            exhaustRunnerGas_[cylinderIndex].dissipateMomentum(
                std::max(0.003, (cylinder.exhaustPrimaryLengthMm > 0.0 ? cylinder.exhaustPrimaryLengthMm
                    : config_.exhaust.primaryLengthMm) / 240'000.0), subDt);
            const auto wallHeatTransfer = std::clamp((cylinderWallTemperatureC_[cylinderIndex] + 273.15
                - cylinderGas_[cylinderIndex].temperatureK()) * 0.42 * subDt, -120.0, 35.0);
            cylinderGas_[cylinderIndex].addHeatJoules(wallHeatTransfer);
            if (cylinder.blowByCoefficient > 0.0)
                (void)ConservativeGasSystem::flowFromBoundary(cylinderGas_[cylinderIndex],
                    config_.ambientPressureKpa, config_.ambientTemperatureC + 273.15,
                    std::numbers::pi * std::pow(cylinder.boreMm * 0.0005, 2.0) * cylinder.blowByCoefficient,
                    0.65, subDt);
            const auto intakeIn = std::max(0.0, intakeTransfer.transferredMassKg * 1.0e6);
            const auto intakeReversion = std::max(0.0, -intakeTransfer.transferredMassKg * 1.0e6);
            const auto exhaustOut = std::max(0.0, exhaustTransfer.transferredMassKg * 1.0e6);
            const auto exhaustReversion = std::max(0.0, -exhaustTransfer.transferredMassKg * 1.0e6);
            intakeFlowMgPerCycle_[cylinderIndex] = smooth(intakeFlowMgPerCycle_[cylinderIndex],
                std::max(0.0, intakeIn - intakeReversion * 0.45), subDt, 52.0);
            exhaustFlowMgPerCycle_[cylinderIndex] = smooth(exhaustFlowMgPerCycle_[cylinderIndex],
                std::max(0.0, exhaustOut - exhaustReversion * 0.35), subDt, 58.0);
            const auto runnerPulse = std::sin(cyclePhase * std::numbers::pi / 180.0);
            intakeRunnerPressureKpa_[cylinderIndex] = intakeRunnerGas_[cylinderIndex].pressureKpa();
            exhaustRunnerPressureKpa_[cylinderIndex] = exhaustRunnerGas_[cylinderIndex].pressureKpa();
            (void)runnerPulse;
            cylinderWallTemperatureC_[cylinderIndex] = smooth(cylinderWallTemperatureC_[cylinderIndex],
                config_.ambientTemperatureC + combustion.heatOutput * 175.0 + pulse * 95.0, subDt, 0.22);
            intakeRunnerPressureSum += intakeRunnerPressureKpa_[cylinderIndex];
            exhaustRunnerPressureSum += exhaustRunnerPressureKpa_[cylinderIndex];
            exhaustFlowMgSum += exhaustFlowMgPerCycle_[cylinderIndex];
            const auto breathingQuality = std::clamp(intakeFlowMgPerCycle_[cylinderIndex]
                / std::max(1.0, combustion.airMassMgPerCycle / static_cast<double>(config_.cylinders.size())),
                0.45, 1.35);
            combustionPulseSum += pulse * std::clamp(1.0 + cylinder.efficiencyOffset, 0.8, 1.2) * breathingQuality;
            const auto radiusM = crankThrowMmFor(config_, cylinder) * 0.001;
            const auto rodM = cylinder.connectingRodMm * 0.001;
            const auto massKg = cylinder.pistonMassGrams * 0.001;
            const auto ratio = radiusM / rodM;

            // Exact kinematic acceleration formula
            const auto cosTheta = std::cos(theta);
            const auto sinTheta = std::sin(theta);
            const auto sin2 = sinTheta * sinTheta;
            const auto ratio2 = ratio * ratio;
            const auto denom = 1.0 - ratio2 * sin2;
            const auto exactTerm = (ratio * std::cos(2.0 * theta) + ratio2 * ratio * sin2 * sin2) / (denom * std::sqrt(denom));
            const auto acceleration = radiusM * state_.angularVelocityRadPerSecond * state_.angularVelocityRadPerSecond
                * (cosTheta + exactTerm);

            const auto inertiaForce = massKg * acceleration;
            const auto leverArm = radiusM * (std::sin(theta) + (ratio * std::sin(2.0 * theta)) / (2.0 * std::sqrt(denom)));
            reciprocatingTorque -= inertiaForce * leverArm;
            const auto pistonAreaM2 = std::numbers::pi * std::pow(cylinder.boreMm * 0.001, 2.0) * 0.25;
            gasIndicatedTorque += (chamberKpa - config_.ambientPressureKpa) * 1'000.0
                * pistonAreaM2 * leverArm;
            peakPistonAcceleration = std::max(peakPistonAcceleration, std::abs(acceleration));
            pistonSpeedSum += 2.0 * cylinder.strokeMm * 0.001 * state_.rpm / 60.0;
        }
        const auto meanPistonSpeed = pistonSpeedSum / static_cast<double>(config_.cylinders.size());
        const auto pumpingTorque = state_.rpm > 30.0
            ? std::max(0.0, config_.ambientPressureKpa - state_.manifoldPressureKpa) * displacement * 0.055 : 0.0;
        const auto mechanicalFriction = state_.rpm > 1.0
            ? displacement * (2.2 + config_.frictionCoefficient * 18.0 + state_.rpm * 0.00075
                              + state_.rpm * state_.rpm * 0.000000045 + meanPistonSpeed * 0.12) : 0.0;
        const auto seizureTorque = state_.damage >= 1.0 ? displacement * 250.0
            + state_.angularVelocityRadPerSecond * 0.22 : 0.0;
        const auto frictionTorque = mechanicalFriction + pumpingTorque + seizureTorque;
        const auto loadTorque = state_.rpm > 40.0 && !safeControls.starterEngaged
            ? state_.load * displacement * 105.0 : 0.0;
        const auto starterPeakTorque = 24.0 + displacement * 17.0;
        const auto starterTorque = cranking ? starterPeakTorque * std::clamp(1.0 - state_.rpm / 760.0, 0.18, 1.0) : 0.0;
        const auto normalizedCombustionPulse = std::clamp(combustionPulseSum * 8.0
            / static_cast<double>(config_.cylinders.size()), 0.0, 2.8);
        const auto modeledIndicatedTorque = combustion.indicatedTorqueNm * normalizedCombustionPulse;
        const auto boundedGasTorque = std::clamp(gasIndicatedTorque,
            -std::max(80.0, modeledIndicatedTorque * 1.5),
            std::max(120.0, modeledIndicatedTorque * 2.8));
        // Keep the proven mean-work model as a stabilising low-frequency term
        // while cylinder pressure contributes the resolved cyclic component.
        // This avoids inventing energy at idle and permits calibration against
        // reference pressure traces without regressing starter behaviour.
        const auto pressureBlend = std::clamp((state_.rpm - 180.0) / 1'200.0, 0.08, 0.28);
        const auto indicatedTorque = std::isfinite(boundedGasTorque)
            ? modeledIndicatedTorque * (1.0 - pressureBlend) + boundedGasTorque * pressureBlend
            : modeledIndicatedTorque;
        const auto brakeTorque = indicatedTorque - frictionTorque;
        const auto boundedReciprocatingTorque = std::clamp(reciprocatingTorque,
            -std::max(20.0, indicatedTorque * 0.5), std::max(20.0, indicatedTorque * 0.5));
        const auto netTorque = brakeTorque + starterTorque - loadTorque
            + safeControls.externalTorqueNm + boundedReciprocatingTorque;

        auto omega = state_.angularVelocityRadPerSecond;
        omega = std::max(0.0, omega + netTorque / config_.rotatingInertiaKgM2 * subDt);
        state_.angularVelocityRadPerSecond = omega;
        state_.rpm = omega * 60.0 / (2.0 * std::numbers::pi);
        if (state_.rpm < 0.5 && !cranking && indicatedTorque <= 0.0) {
            state_.rpm = 0.0;
            state_.angularVelocityRadPerSecond = 0.0;
        }
        const auto subTravelled = (subPreviousRpm + state_.rpm) * 0.5 * 6.0 * subDt;
        totalTravelled += subTravelled;
        state_.crankAngleDegrees = std::fmod(subPreviousAngle + subTravelled, eventGenerator_.cycleDegrees());
        state_.simulationTimeSeconds += subDt;

        state_.indicatedTorqueNm = indicatedTorque;
        state_.frictionTorqueNm = frictionTorque;
        state_.reciprocatingTorqueNm = boundedReciprocatingTorque;
        state_.meanPistonSpeedMps = meanPistonSpeed;
        state_.loadTorqueNm = loadTorque;
        state_.starterTorqueNm = starterTorque;
        state_.netTorqueNm = netTorque;
        state_.torqueNm = brakeTorque;
        state_.cycleAveragedTorqueNm = smooth(state_.cycleAveragedTorqueNm, brakeTorque, subDt, 8.0);
        state_.knockLevel = smooth(state_.knockLevel, combustion.knockLevel, subDt, 10.0);
        state_.misfireRate = smooth(state_.misfireRate, combustion.misfireProbability, subDt, 6.0);
        state_.airFuelRatio = combustion.actualAirFuelRatio;
        state_.targetAirFuelRatio = ecuCommand.targetAirFuelRatio;
        state_.ignitionAdvanceDegrees = ecuCommand.ignitionAdvanceDegrees;
        state_.volumetricEfficiency = state_.rpm > 20.0 ? combustion.volumetricEfficiency : 0.0;
        state_.airMassMgPerCycle = state_.rpm > 20.0 ? combustion.airMassMgPerCycle : 0.0;
        state_.injectedFuelMgPerCycle = ecuCommand.fuelEnabled ? combustion.fuelMassMgPerCycle : 0.0;
        state_.fuelFlowGramsPerSecond = state_.injectedFuelMgPerCycle * (state_.rpm / 120.0) / 1'000.0;
        state_.airFlowGramsPerSecond = state_.airMassMgPerCycle * (state_.rpm / 120.0) / 1'000.0;
        state_.lambda = state_.airFuelRatio / 14.7;
        state_.exhaustPressureKpa = backPressure;
        state_.intakeRunnerPressureKpa = intakeRunnerPressureSum / static_cast<double>(config_.cylinders.size());
        state_.exhaustRunnerPressureKpa = exhaustRunnerPressureSum / static_cast<double>(config_.cylinders.size());
        state_.exhaustFlowGramsPerSecond = exhaustFlowMgSum * (state_.rpm / 120.0) / 1'000.0;
        state_.manifoldGasMassGrams = manifoldGas_.massKg() * 1'000.0;
        state_.cylinderGasMassGrams = 0.0;
        state_.gasInternalEnergyJoules = manifoldGas_.internalEnergyJoules() + collectorGas_.internalEnergyJoules();
        for (std::size_t index = 0; index < config_.cylinders.size(); ++index) {
            state_.cylinderGasMassGrams += cylinderGas_[index].massKg() * 1'000.0;
            state_.gasInternalEnergyJoules += cylinderGas_[index].internalEnergyJoules()
                + intakeRunnerGas_[index].internalEnergyJoules() + exhaustRunnerGas_[index].internalEnergyJoules();
        }
        state_.powerKw = state_.torqueNm * state_.angularVelocityRadPerSecond / 1'000.0;
        state_.cycleAveragedPowerKw = state_.cycleAveragedTorqueNm * state_.angularVelocityRadPerSecond / 1'000.0;
        state_.brakeSpecificFuelConsumptionGPerKwh = state_.cycleAveragedPowerKw > 1.0
            ? state_.fuelFlowGramsPerSecond * 3'600.0 / state_.cycleAveragedPowerKw : 0.0;
        state_.mechanicalEfficiency = indicatedTorque > 1.0 ? std::clamp(brakeTorque / indicatedTorque, 0.0, 1.0) : 0.0;
        state_.oilPressureKpa = state_.rpm > 0.0
            ? std::clamp(80.0 + state_.rpm * 0.055 - std::max(0.0, state_.oilTemperatureC - 105.0) * 2.0, 0.0, 520.0) : 0.0;
        state_.peakPistonAccelerationG = peakPistonAcceleration / 9.80665;

        const auto thermostatOpening = std::clamp((state_.coolantTemperatureC - 82.0) / 12.0, 0.08, 1.0);
        const auto coolingAirflow = config_.coolingEfficiency * (0.18 + thermostatOpening
            * (0.42 + state_.rpm / std::max(1.0, config_.redlineRpm) * 0.8));
        const auto coolantHeatKw = combustion.heatPowerKw * config_.thermal.coolantHeatShare;
        const auto oilHeatKw = combustion.heatPowerKw * config_.thermal.oilHeatShare
            + mechanicalFriction * state_.angularVelocityRadPerSecond / 1'000.0 * 0.18;
        const auto coolantRejectedKw = std::max(0.0, state_.coolantTemperatureC - config_.ambientTemperatureC)
            * config_.thermal.coolingPowerKwPerC * coolingAirflow;
        const auto oilRejectedKw = std::max(0.0, state_.oilTemperatureC - config_.ambientTemperatureC)
            * config_.thermal.oilCoolingPowerKwPerC * std::max(0.35, coolingAirflow * 0.72);
        const auto exhaustTarget = config_.ambientTemperatureC + combustion.heatOutput * 760.0;
        state_.coolantTemperatureC = std::clamp(state_.coolantTemperatureC
            + (coolantHeatKw - coolantRejectedKw) / config_.thermal.coolantMassKjPerC * subDt,
            config_.ambientTemperatureC - 5.0, 150.0);
        state_.oilTemperatureC = std::clamp(state_.oilTemperatureC
            + (oilHeatKw - oilRejectedKw) / config_.thermal.oilMassKjPerC * subDt,
            config_.ambientTemperatureC - 5.0, 170.0);
        state_.exhaustTemperatureC = smooth(state_.exhaustTemperatureC, exhaustTarget, subDt, 0.18);
        const auto overrev = std::max(0.0, state_.rpm / config_.redlineRpm - 1.0);
        const auto oilStarvation = state_.rpm > 800.0 && state_.oilPressureKpa < 90.0 ? 1.0 : 0.0;
        const auto excessivePistonAcceleration = std::max(0.0, state_.peakPistonAccelerationG - 5'500.0) / 5'500.0;
        const auto damageRate = state_.knockLevel * state_.throttle * 0.0012 + overrev * overrev * 0.022
                              + std::max(0.0, state_.coolantTemperatureC - 115.0) * 0.000025 + oilStarvation * 0.002
                              + excessivePistonAcceleration * excessivePistonAcceleration * 0.002;
        state_.damage = std::clamp(state_.damage + damageRate * subDt, 0.0, 1.0);
        state_.wear = std::clamp(state_.wear + (state_.rpm / config_.redlineRpm * 0.000002 + damageRate * 0.12) * subDt, 0.0, 1.0);
        state_.runningState = determineRunningState(safeControls);
        state_.cylinderStateCount = std::min(config_.cylinders.size(), state_.cylinderStates.size());
        for (std::size_t index = 0; index < state_.cylinderStateCount; ++index) {
            const auto& cylinder = config_.cylinders[index];
            const auto phase = std::fmod(state_.crankAngleDegrees - crankOffsetDegreesFor(config_, cylinder) + 1'440.0, 720.0);
            const auto active = combustion.combustionEnabled && !cylinderMisfires_[index];
            const auto pulse = active ? enginelab::combustionPulse(phase, ecuCommand.ignitionAdvanceDegrees,
                                                                   cylinder.ignitionOffsetDegrees) : 0.0;
            const auto bankedPhase = phase + cylinder.bankOffsetDegrees;
            const auto cams = activeCamshaft(config_, cylinder, state_.rpm, state_.throttle);
            const auto intakeLift = profiledValveLiftMm(bankedPhase, 360.0 + cams.config->intakeCenterlineDegrees,
                                                        cams.intakeDuration(), cams.intakeLift(), cams.intakeProfile());
            const auto exhaustLift = profiledValveLiftMm(bankedPhase, 720.0 - cams.config->exhaustCenterlineDegrees,
                                                         cams.exhaustDuration(), cams.exhaustLift(), cams.exhaustProfile());
            const auto& gas = cylinderGas_[index];
            state_.cylinderStates[index] = { cylinder.id, phase,
                chamberPressureBar_[index],
                pulse, combustion.misfireProbability,
                intakeLift, exhaustLift, intakeFlowMgPerCycle_[index], exhaustFlowMgPerCycle_[index],
                exhaustRunnerPressureKpa_[index],
                config_.ambientTemperatureC + (state_.exhaustTemperatureC - config_.ambientTemperatureC)
                    * std::clamp(1.0 + cylinder.efficiencyOffset, 0.8, 1.2)
                    + (cylinderWallTemperatureC_[index] - config_.ambientTemperatureC) * 0.18,
                gas.temperatureK() - 273.15, gas.massKg() * 1.0e6,
                gas.mixture().oxygenMoles, gas.mixture().fuelMoles, gas.mixture().burnedMoles,
                intakeRunnerGas_[index].bulkVelocityMps(), exhaustRunnerGas_[index].bulkVelocityMps(),
                active, combustion.combustionEnabled && cylinderMisfires_[index] };
        }
    }

    SimulationFrame frame;
    frame.state = state_;
    const auto finalEcuCommand = ecu_.evaluate(config_, state_, safeControls);
    const auto finalBackPressure = exhaust_.backPressureKpa(state_);
    const auto finalCombustion = physics_.evaluateCombustion(config_, state_, safeControls, finalEcuCommand, finalBackPressure);
    if (finalEcuCommand.fuelEnabled && finalEcuCommand.sparkEnabled && totalTravelled > 0.0) {
        frame.firingEventCount = eventGenerator_.generate(config_, state_, finalEcuCommand, finalCombustion,
            stepStartTime, previousAngle, totalTravelled, dt, frame.firingEvents);
        frame.droppedFiringEventCount = eventGenerator_.droppedEventCountLastGenerate();
        for (std::size_t index = 0; index < frame.firingEventCount; ++index) exhaust_.process(frame.firingEvents[index]);
    }
    return frame;
}

void EngineSimulator::reset() noexcept {
    ecu_.reset();
    eventGenerator_.reset();
    state_ = {};
    state_.coolantTemperatureC = config_.ambientTemperatureC;
    state_.oilTemperatureC = config_.ambientTemperatureC;
    state_.exhaustTemperatureC = config_.ambientTemperatureC;
    state_.manifoldPressureKpa = config_.ambientPressureKpa;
    state_.boostPressureRatio = 1.0;
    state_.exhaustPressureKpa = config_.ambientPressureKpa;
    state_.intakeRunnerPressureKpa = config_.ambientPressureKpa;
    state_.exhaustRunnerPressureKpa = config_.ambientPressureKpa;
    randomState_ = 0x6d2b79f5U;
    cylinderMisfires_.fill(false);
    cylinderMisfirePrepared_.fill(false);
    intakeFlowMgPerCycle_.fill(0.0);
    exhaustFlowMgPerCycle_.fill(0.0);
    lastCombustionProfile_.fill(0.0);
    injectedFuelMolesThisCycle_.fill(0.0);
    manifoldGas_.initialise(config_.ambientPressureKpa, config_.plenumVolumeLitres,
                            config_.ambientTemperatureC + 273.15);
    collectorGas_.initialise(config_.ambientPressureKpa,
        std::max(0.6, static_cast<double>(config_.cylinders.size()) * 0.22),
        config_.ambientTemperatureC + 273.15);
    for (std::size_t index = 0; index < config_.cylinders.size(); ++index) {
        previousCylinderPhases_[index] = std::fmod(state_.crankAngleDegrees
            - crankOffsetDegreesFor(config_, config_.cylinders[index]) + 1'440.0, 720.0);
        intakeRunnerPressureKpa_[index] = config_.ambientPressureKpa;
        exhaustRunnerPressureKpa_[index] = config_.ambientPressureKpa;
        chamberPressureBar_[index] = config_.ambientPressureKpa / 100.0;
        cylinderWallTemperatureC_[index] = config_.ambientTemperatureC;
        intakeRunnerGas_[index].initialise(config_.ambientPressureKpa, 0.18,
                                           config_.ambientTemperatureC + 273.15);
        cylinderGas_[index].initialise(config_.ambientPressureKpa,
            chamberVolumeLitres(config_.cylinders[index], previousCylinderPhases_[index]),
            config_.ambientTemperatureC + 273.15);
        exhaustRunnerGas_[index].initialise(config_.ambientPressureKpa,
            std::max(0.10, config_.exhaust.primaryLengthMm * std::numbers::pi
                * std::pow(config_.exhaust.primaryDiameterMm * 0.5, 2.0) / 1.0e6),
            config_.ambientTemperatureC + 273.15);
    }
}

RunningState EngineSimulator::determineRunningState(const EngineControls& controls) const noexcept {
    if (state_.damage >= 1.0) return RunningState::destroyed;
    if (state_.damage > 0.5) return RunningState::damaged;
    if (state_.coolantTemperatureC > 118.0) return RunningState::overheating;
    if (state_.knockLevel > 0.45) return RunningState::knocking;
    if (controls.starterEngaged && state_.rpm < 620.0) return RunningState::cranking;
    if (state_.rpm < 250.0) return RunningState::stopped;
    if (state_.rpm < config_.idleRpm * 1.2) return state_.misfireRate > 0.18 ? RunningState::unstable : RunningState::idling;
    return RunningState::running;
}
} // namespace enginelab
