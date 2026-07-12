#include <enginelab/simulation/EngineSimulator.hpp>
#include <algorithm>
#include <cmath>
#include <numbers>
#include <numeric>
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

[[nodiscard]] double mechanicalCrankOffsetDegreesFor(const EngineConfig& config,
                                                      const CylinderConfig& cylinder) noexcept {
    // A radial cylinder reaches geometric TDC according to its spatial bank
    // direction while its firing TDC remains a separate 720-degree event.
    if (config.layout == EngineLayout::radial) return cylinder.bankOffsetDegrees;
    if (cylinder.crankJournalId != 0) {
        const auto journal = std::find_if(config.crankJournals.begin(), config.crankJournals.end(),
            [&cylinder](const CrankJournalConfig& item) { return item.id == cylinder.crankJournalId; });
        if (journal != config.crankJournals.end()) return journal->angleDegrees;
    }
    return cylinder.crankOffsetDegrees;
}

[[nodiscard]] const ExhaustConfig& exhaustGeometryFor(const EngineConfig& config,
                                                       const CylinderConfig& cylinder) noexcept {
    const auto path = std::find_if(config.exhaustPaths.begin(), config.exhaustPaths.end(),
        [&cylinder](const ExhaustPathConfig& item) {
            return std::find(item.cylinderIds.begin(), item.cylinderIds.end(), cylinder.id)
                != item.cylinderIds.end();
        });
    return path != config.exhaustPaths.end() ? path->geometry : config.exhaust;
}

[[nodiscard]] std::size_t exhaustPathIndexFor(const EngineConfig& config,
                                              const CylinderConfig& cylinder) noexcept {
    const auto path = std::find_if(config.exhaustPaths.begin(), config.exhaustPaths.end(),
        [&cylinder](const ExhaustPathConfig& item) {
            return std::find(item.cylinderIds.begin(), item.cylinderIds.end(), cylinder.id)
                != item.cylinderIds.end();
        });
    return path != config.exhaustPaths.end()
        ? static_cast<std::size_t>(std::distance(config.exhaustPaths.begin(), path)) : 0U;
}

[[nodiscard]] const ExhaustConfig& exhaustGeometryAt(const EngineConfig& config,
                                                      std::size_t pathIndex) noexcept {
    return pathIndex < config.exhaustPaths.size() ? config.exhaustPaths[pathIndex].geometry : config.exhaust;
}

[[nodiscard]] double forwardPhaseDegrees(double previous, double current) noexcept {
    return std::fmod(current - previous + 720.0, 720.0);
}

[[nodiscard]] bool crossedPhase(double previous, double current, double target) noexcept {
    const auto travel = forwardPhaseDegrees(previous, current);
    if (travel <= 1.0e-9) return false;
    const auto distance = std::fmod(target - previous + 720.0, 720.0);
    return distance <= travel;
}

[[nodiscard]] bool phaseInsideWindow(double phase, double start, double end) noexcept {
    if (start < end) return phase >= start && phase <= end;
    return phase >= start || phase <= end;
}

[[nodiscard]] double airFuelRatioForCell(const GasCell& cell, const FuelConfig& fuel) noexcept {
    const auto fuelMass = cell.mixture().fuelMoles * fuel.molarMassGramsPerMole * 0.001;
    if (fuelMass <= 1.0e-15) return 100.0;
    const auto airMass = cell.mixture().oxygenMoles * GasCell::oxygenMolarMassKg
        + cell.mixture().inertMoles * GasCell::inertMolarMassKg;
    return airMass / fuelMass;
}

[[nodiscard]] double stribeckFrictionForce(const CylinderConfig& cylinder,
                                            double pistonSpeedMps,
                                            double cylinderWallForceN) noexcept {
    const auto velocity = std::abs(pistonSpeedMps);
    const auto coulomb = cylinder.pistonFrictionCoefficient * std::abs(cylinderWallForceN);
    const auto stribeckVelocity = cylinder.pistonBreakawayVelocityMps * std::numbers::sqrt2;
    const auto coulombVelocity = cylinder.pistonBreakawayVelocityMps * 0.1;
    const auto transition = std::numbers::sqrt2 * std::numbers::e_v<double>
        * (cylinder.pistonBreakawayForceN - coulomb)
        * (velocity / stribeckVelocity)
        * std::exp(-std::pow(velocity / stribeckVelocity, 2.0));
    return std::max(0.0, transition + coulomb * std::tanh(velocity / coulombVelocity)
        + cylinder.pistonViscousFrictionNsPerM * velocity);
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

void EngineSimulator::setPressureSamplingEnabled(bool enabled) {
    if (enabled && !pressureSamples_)
        pressureSamples_ = std::make_unique<SpscQueue<CylinderPressureSample, 1'024>>();
    else if (!enabled)
        pressureSamples_.reset();
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
    // gasSubsteps raises the minimum integration cadence; it must not multiply
    // the speed-dependent demand and then hide an under-resolved crank step
    // behind the configured frequency cap.
    const auto minimumGasFrequency = config_.solver.mechanicalFrequencyHz
        * static_cast<double>(config_.solver.gasSubsteps);
    const auto requestedSolverFrequency = std::max(minimumGasFrequency, angleFrequency);
    const auto solverFrequency = std::min(requestedSolverFrequency,
        config_.solver.maximumMechanicalFrequencyHz);
    const auto maxSubStepDt = 1.0 / solverFrequency;
    const auto subStepCount = std::max(std::size_t { 1 }, static_cast<std::size_t>(std::ceil(dt / maxSubStepDt)));
    const auto subDt = dt / static_cast<double>(subStepCount);
    state_.solverFrequencyHz = solverFrequency;
    state_.crankDegreesPerSolverStep = solverFrequency > 0.0
        ? std::abs(state_.rpm) * 6.0 / solverFrequency : 0.0;
    state_.solverResolutionLimited = requestedSolverFrequency
        > config_.solver.maximumMechanicalFrequencyHz + 1.0e-9;
    state_.solverSubsteps = static_cast<std::uint32_t>(std::min<std::size_t>(subStepCount,
        std::numeric_limits<std::uint32_t>::max()));
    double totalTravelled = 0.0;
    double maximumIntegratedCrankStep = 0.0;
    SimulationFrame frame;

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
            const auto idleBypassAreaM2 = config_.intake.idleBypassAreaMm2 * 1.0e-6;
            const auto throttleAreaM2 = idleBypassAreaM2
                + throttlePlateAreaM2 * std::pow(state_.throttle, config_.intake.throttleGamma);
            (void)ConservativeGasSystem::flowFromBoundary(manifoldGas_, intakeSourcePressureKpa,
                temperatureK, throttleAreaM2,
                config_.intake.throttleDischargeCoefficient, subDt);
        }
        state_.manifoldPressureKpa = manifoldGas_.pressureKpa();

        // The collector is a real control volume. Its pressure is the engine's
        // downstream boundary; the exhaust policy adds only the restriction
        // associated with pipes/mufflers which are not spatially discretised.
        double collectorPressureKpa = config_.ambientPressureKpa;
        for (std::size_t pathIndex = 0; pathIndex < exhaustCollectorCount_; ++pathIndex)
            collectorPressureKpa = std::max(collectorPressureKpa, exhaustCollectorGas_[pathIndex].pressureKpa());
        state_.exhaustPressureKpa = collectorPressureKpa;
        const auto backPressure = std::max(state_.exhaustPressureKpa, exhaust_.backPressureKpa(state_));
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
        double pistonFrictionPowerW = 0.0;
        double pistonBreakawayTorqueNm = 0.0;
        double physicalEndGasKnockLevel = 0.0;

        for (std::size_t cylinderIndex = 0; cylinderIndex < config_.cylinders.size(); ++cylinderIndex) {
            const auto& cylinder = config_.cylinders[cylinderIndex];
            const auto crankOffset = crankOffsetDegreesFor(config_, cylinder);
            const auto mechanicalOffset = mechanicalCrankOffsetDegreesFor(config_, cylinder);
            const auto mechanicalPhase = std::fmod(state_.crankAngleDegrees - mechanicalOffset + 720.0, 360.0);
            const auto theta = mechanicalPhase * std::numbers::pi / 180.0;
            const auto cyclePhase = std::fmod(state_.crankAngleDegrees - crankOffset + 1'440.0, 720.0);
            const auto cams = activeCamshaft(config_, cylinder, state_.rpm, state_.throttle);
            const auto chamberVolume = chamberVolumeLitres(cylinder, mechanicalPhase);
            cylinderGas_[cylinderIndex].setVolumeAdiabatic(chamberVolume);
            const auto previousPhase = previousCylinderPhases_[cylinderIndex];
            const auto sparkPhase = std::fmod(720.0 - ecuCommand.ignitionAdvanceDegrees
                + cylinder.ignitionOffsetDegrees + 720.0, 720.0);
            const auto sparkCrossed = crossedPhase(previousPhase, cyclePhase, sparkPhase);
            const auto injectionStartCrossed = crossedPhase(previousPhase, cyclePhase,
                config_.injection.startAngleDegrees);
            if (injectionStartCrossed) {
                injectedFuelMolesThisCycle_[cylinderIndex] = 0.0;
                requestedFuelMolesThisCycle_[cylinderIndex] = 0.0;
            }
            const auto fuelMolarMassKg = config_.fuelProperties.molarMassGramsPerMole * 0.001;
            const auto oxygenEquivalentAirMassMg = cylinderGas_[cylinderIndex].mixture().oxygenMoles
                / 0.21 * GasCell::airMolarMassKg * 1.0e6;
            const auto measuredChargeMassMg = config_.injection.mode == InjectionMode::direct
                ? oxygenEquivalentAirMassMg
                : std::max(oxygenEquivalentAirMassMg, trappedAirMassMgLastCycle_[cylinderIndex]);
            const auto physicalFuelTargetMoles = measuredChargeMassMg
                / std::clamp(ecuCommand.targetAirFuelRatio, 5.0, 30.0)
                * ecuCommand.fuelCorrection * 1.0e-6 / fuelMolarMassKg;
            requestedFuelMolesThisCycle_[cylinderIndex] = std::max(
                requestedFuelMolesThisCycle_[cylinderIndex], physicalFuelTargetMoles);
            const auto requestedFuelMoles = requestedFuelMolesThisCycle_[cylinderIndex];
            auto& injectionTarget = config_.injection.mode == InjectionMode::port
                ? intakeRunnerGas_[cylinderIndex] : cylinderGas_[cylinderIndex];
            double commandedFuelMoles = 0.0;
            if (combustion.combustionEnabled && phaseInsideWindow(cyclePhase,
                    config_.injection.startAngleDegrees, config_.injection.endAngleDegrees)) {
                // Meter to the requested physical inventory, accounting for
                // vapour already present and port-wall film. Counting only the
                // current-cycle injector pulse over-fuels residual-rich cells.
                const auto existingFuelInventory = injectionTarget.mixture().fuelMoles
                    + (config_.injection.mode == InjectionMode::port
                        ? injectionStates_[cylinderIndex].liquidFilmMoles : 0.0);
                commandedFuelMoles = std::max(0.0,
                    requestedFuelMoles - existingFuelInventory);
            }
            const auto injectionResult = FuelInjectionModel::deliver(config_.injection,
                config_.fuelProperties, injectionStates_[cylinderIndex], injectionTarget,
                commandedFuelMoles, subDt);
            injectedFuelMolesThisCycle_[cylinderIndex] += injectionResult.meteredMoles;
            if (!combustion.combustionEnabled) {
                cylinderMisfires_[cylinderIndex] = false;
                flameEvents_[cylinderIndex] = {};
            } else if (sparkCrossed) {
                endGasKnockStates_[cylinderIndex] = {};
                deliveredFuelMolesLastCycle_[cylinderIndex] = injectedFuelMolesThisCycle_[cylinderIndex];
                const auto chamberFuelMoles = cylinderGas_[cylinderIndex].mixture().fuelMoles;
                fuelDeliveryRatio_[cylinderIndex] = requestedFuelMoles > 1.0e-15
                    ? std::clamp(chamberFuelMoles / requestedFuelMoles, 0.0, 1.0) : 0.0;
                const auto mixtureAfr = airFuelRatioForCell(cylinderGas_[cylinderIndex], config_.fuelProperties);
                const auto mixtureError = std::abs(mixtureAfr - ecuCommand.targetAirFuelRatio)
                    / std::max(5.0, ecuCommand.targetAirFuelRatio);
                const auto stoichiometricAfr = config_.fuelProperties.stoichiometricAirFuelRatio;
                const auto flammabilityPenalty = std::max(0.0,
                    mixtureAfr - stoichiometricAfr * 1.12) / (stoichiometricAfr * 0.35)
                    + std::max(0.0, stoichiometricAfr * 0.68 - mixtureAfr)
                        / (stoichiometricAfr * 0.28);
                const auto actualMisfireProbability = std::clamp(
                    std::max(0.0, mixtureError - 0.20) * 0.95
                    + flammabilityPenalty * 0.72
                    + std::max(0.0, 380.0 - state_.rpm) / 1'400.0
                    + (requestedFuelMoles > 1.0e-15
                        ? std::max(0.0, 0.55 - fuelDeliveryRatio_[cylinderIndex]) * 0.65 : 0.0),
                    0.0, 0.92);
                randomState_ ^= randomState_ << 13U;
                randomState_ ^= randomState_ >> 17U;
                randomState_ ^= randomState_ << 5U;
                const auto randomUnit = static_cast<double>(randomState_) / static_cast<double>(0xffffffffU);
                cylinderMisfires_[cylinderIndex] = randomUnit < actualMisfireProbability;
                const auto totalMoles = cylinderGas_[cylinderIndex].totalMoles();
                const FlameConditions flameConditions {
                    cylinder.boreMm * 0.001, chamberVolume * 0.001,
                    cylinderGas_[cylinderIndex].temperatureK(),
                    cylinderGas_[cylinderIndex].pressureKpa() * 1'000.0,
                    config_.fuelProperties.stoichiometricAirFuelRatio / std::max(1.0, mixtureAfr),
                    totalMoles > 1.0e-15
                        ? cylinderGas_[cylinderIndex].mixture().burnedMoles / totalMoles : 0.0,
                    2.0 * cylinder.strokeMm * 0.001 * state_.rpm / 60.0,
                    state_.load };
                const auto burnableFuelMoles = std::min(chamberFuelMoles,
                    cylinderGas_[cylinderIndex].mixture().oxygenMoles
                        / config_.fuelProperties.oxygenMolesPerFuelMole);
                if (!cylinderMisfires_[cylinderIndex])
                    flamePhysics_.ignite(flameEvents_[cylinderIndex], config_.fuelProperties,
                                         flameConditions, burnableFuelMoles);
                else flameEvents_[cylinderIndex] = {};
            }
            previousCylinderPhases_[cylinderIndex] = cyclePhase;
            const auto phaseTravel = forwardPhaseDegrees(previousPhase, cyclePhase);
            const auto mixtureAfr = airFuelRatioForCell(cylinderGas_[cylinderIndex], config_.fuelProperties);
            const auto totalMoles = cylinderGas_[cylinderIndex].totalMoles();
            const FlameConditions flameConditions {
                cylinder.boreMm * 0.001, chamberVolume * 0.001,
                cylinderGas_[cylinderIndex].temperatureK(),
                cylinderGas_[cylinderIndex].pressureKpa() * 1'000.0,
                config_.fuelProperties.stoichiometricAirFuelRatio / std::max(1.0, mixtureAfr),
                totalMoles > 1.0e-15
                    ? cylinderGas_[cylinderIndex].mixture().burnedMoles / totalMoles : 0.0,
                2.0 * cylinder.strokeMm * 0.001 * state_.rpm / 60.0,
                state_.load };
            const auto flameStep = flamePhysics_.advance(flameEvents_[cylinderIndex],
                config_.fuelProperties, flameConditions, subDt);
            const auto burnAdvance = flameStep.burnedFractionAdvance;
            const auto pulse = phaseTravel > 1.0e-6
                ? std::clamp(burnAdvance / phaseTravel * 90.0, 0.0, 3.5) : 0.0;
            instantaneousCombustionPulse_[cylinderIndex] = pulse * fuelDeliveryRatio_[cylinderIndex];
            // Bank angle is spatial geometry, not cam timing. Valve events are
            // referenced to the cylinder's 720-degree thermodynamic phase.
            const auto intakeLift = profiledValveLiftMm(cyclePhase, 360.0 + cams.config->intakeCenterlineDegrees,
                                                        cams.intakeDuration(), cams.intakeLift(), cams.intakeProfile());
            const auto exhaustLift = profiledValveLiftMm(cyclePhase, 360.0 - cams.config->exhaustCenterlineDegrees,
                                                         cams.exhaustDuration(), cams.exhaustLift(), cams.exhaustProfile());
            if (burnAdvance > 0.0 && combustion.combustionEnabled && !cylinderMisfires_[cylinderIndex])
                (void)ConservativeGasSystem::reactFuelMoles(cylinderGas_[cylinderIndex],
                    flameEvents_[cylinderIndex].initialBurnableFuelMoles * burnAdvance,
                    flameStep.efficiency,
                    config_.fuelProperties.lowerHeatingValueMjPerKg * 1'000'000.0);

            const auto endGas = EndGasKnockModel::advance(endGasKnockStates_[cylinderIndex], {
                cylinderGas_[cylinderIndex].pressureKpa() / 100.0,
                cylinderGas_[cylinderIndex].temperatureK(),
                flameConditions.equivalenceRatio, flameEvents_[cylinderIndex].burnedFraction,
                config_.octaneRating,
                combustion.combustionEnabled && flameEvents_[cylinderIndex].active
                    && !cylinderMisfires_[cylinderIndex] }, subDt);
            if (endGas.autoIgnited && endGas.autoIgnitedFuelFraction > 0.0) {
                (void)ConservativeGasSystem::reactFuelMoles(cylinderGas_[cylinderIndex],
                    flameEvents_[cylinderIndex].initialBurnableFuelMoles
                        * endGas.autoIgnitedFuelFraction,
                    std::max(0.72, flameStep.efficiency),
                    config_.fuelProperties.lowerHeatingValueMjPerKg * 1'000'000.0);
                flameEvents_[cylinderIndex].burnedFraction = std::clamp(
                    flameEvents_[cylinderIndex].burnedFraction + endGas.autoIgnitedFuelFraction,
                    0.0, 1.0);
                if (flameEvents_[cylinderIndex].burnedFraction >= 0.999)
                    flameEvents_[cylinderIndex].active = false;
            }
            physicalEndGasKnockLevel = std::max(physicalEndGasKnockLevel, endGas.level);

            auto chamberKpa = cylinderGas_[cylinderIndex].pressureKpa();
            chamberPressureBar_[cylinderIndex] = chamberKpa / 100.0;
            const auto intakeArea = valveAreaMm2(cylinder.boreMm, intakeLift);
            const auto exhaustArea = valveAreaMm2(cylinder.boreMm, exhaustLift);
            const auto runnerDiameterMm = cylinder.intakeRunnerDiameterMm > 0.0
                ? cylinder.intakeRunnerDiameterMm : config_.intake.runnerDiameterMm;
            const auto runnerAreaM2 = std::numbers::pi * std::pow(runnerDiameterMm * 0.0005, 2.0);
            const auto& cylinderExhaust = exhaustGeometryFor(config_, cylinder);
            const auto primaryAreaM2 = std::numbers::pi * std::pow(cylinderExhaust.primaryDiameterMm * 0.0005, 2.0);
            const auto pistonAreaM2 = std::numbers::pi * std::pow(cylinder.boreMm * 0.0005, 2.0);
            const auto exhaustPathIndex = exhaustPathIndexFor(config_, cylinder);

            // ----------------------------------------------------------------
            // Gas flow — full physics variant with dynamic pressure and jet
            // momentum.  Direction vectors follow the physical path from
            // atmosphere through the intake to the cylinder and out through
            // the exhaust to the collector.
            // ----------------------------------------------------------------

            // (1) Manifold → intake runner
            (void)ConservativeGasSystem::flow({ &manifoldGas_, &intakeRunnerGas_[cylinderIndex],
                runnerAreaM2, 0.78, subDt,
                /*dirX=*/0.0, /*dirY=*/1.0,   // downward into runner
                /*csArea0=*/0.0, /*csArea1=*/runnerAreaM2 });

            // (2) Intake runner → cylinder (intake valve)
            const FlowParameters intakeValveFlow {
                &intakeRunnerGas_[cylinderIndex], &cylinderGas_[cylinderIndex],
                intakeArea * 1.0e-6, cams.config->intakeFlowCoefficient, subDt,
                /*dirX=*/0.0, /*dirY=*/1.0,   // downward through valve
                /*csArea0=*/runnerAreaM2, /*csArea1=*/pistonAreaM2 };

            // (3) Cylinder → exhaust runner (exhaust valve)
            const FlowParameters exhaustValveFlow {
                &cylinderGas_[cylinderIndex], &exhaustRunnerGas_[cylinderIndex],
                exhaustArea * 1.0e-6, cams.config->exhaustFlowCoefficient, subDt,
                /*dirX=*/0.0, /*dirY=*/-1.0,  // upward out of cylinder
                /*csArea0=*/pistonAreaM2, /*csArea1=*/primaryAreaM2 };
            const auto valveTransfers = ConservativeGasSystem::flowSimultaneous(
                intakeValveFlow, exhaustValveFlow);
            const auto& intakeTransfer = valveTransfers.first;
            const auto& exhaustTransfer = valveTransfers.second;

            // (4) Exhaust runner → collector
            (void)ConservativeGasSystem::flow({
                &exhaustRunnerGas_[cylinderIndex], &exhaustCollectorGas_[exhaustPathIndex],
                primaryAreaM2, 0.74, subDt,
                /*dirX=*/1.0, /*dirY=*/0.0,   // outward to collector
                /*csArea0=*/primaryAreaM2, /*csArea1=*/0.0 });

            intakeRunnerGas_[cylinderIndex].dissipateMomentum(
                std::max(0.004, cylinder.intakeRunnerLengthMm / 180'000.0), subDt);
            exhaustRunnerGas_[cylinderIndex].dissipateMomentum(
                std::max(0.003, (cylinder.exhaustPrimaryLengthMm > 0.0 ? cylinder.exhaustPrimaryLengthMm
                    : cylinderExhaust.primaryLengthMm) / 240'000.0), subDt);
            const auto wallHeatTransfer = std::clamp((cylinderWallTemperatureC_[cylinderIndex] + 273.15
                - cylinderGas_[cylinderIndex].temperatureK()) * 0.42 * subDt, -120.0, 35.0);
            cylinderGas_[cylinderIndex].addHeatJoules(wallHeatTransfer);
            if (cylinder.blowByCoefficient > 0.0)
                (void)ConservativeGasSystem::flowFromBoundary(cylinderGas_[cylinderIndex],
                    config_.ambientPressureKpa, config_.ambientTemperatureC + 273.15,
                    std::numbers::pi * std::pow(cylinder.boreMm * 0.0005, 2.0) * cylinder.blowByCoefficient,
                    0.65, subDt);
            chamberKpa = cylinderGas_[cylinderIndex].pressureKpa();
            chamberPressureBar_[cylinderIndex] = chamberKpa / 100.0;
            const auto intakeClosePhase = std::fmod(360.0 + cams.config->intakeCenterlineDegrees
                + cams.intakeDuration() * 0.5 + 720.0, 720.0);
            if (crossedPhase(previousPhase, cyclePhase, intakeClosePhase))
                trappedAirMassMgLastCycle_[cylinderIndex] = cylinderGas_[cylinderIndex]
                    .mixture().oxygenMoles / 0.21 * GasCell::airMolarMassKg * 1.0e6;
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
            const auto breathingQuality = std::clamp(intakeFlowMgPerCycle_[cylinderIndex]
                / std::max(1.0, combustion.airMassMgPerCycle / static_cast<double>(config_.cylinders.size())),
                0.45, 1.35);
            combustionPulseSum += pulse * std::clamp(1.0 + cylinder.efficiencyOffset, 0.8, 1.2)
                * breathingQuality * fuelDeliveryRatio_[cylinderIndex];
            const auto radiusM = crankThrowMmFor(config_, cylinder) * 0.001;
            const auto rodM = cylinder.connectingRodMm * 0.001;
            // One third of the connecting rod mass is a standard equivalent
            // reciprocating-mass approximation; the remainder contributes to
            // crank rotational inertia represented by EngineConfig.
            const auto massKg = (cylinder.pistonMassGrams
                + cylinder.connectingRodMassGrams / 3.0) * 0.001;
            const auto ratio = radiusM / rodM;

            // Exact kinematic acceleration formula
            const auto cosTheta = std::cos(theta);
            const auto sinTheta = std::sin(theta);
            const auto sin2 = sinTheta * sinTheta;
            const auto ratio2 = ratio * ratio;
            const auto denom = 1.0 - ratio2 * sin2;
            const auto exactTerm = (ratio * std::cos(2.0 * theta) + ratio2 * ratio * sin2 * sin2) / (denom * std::sqrt(denom));
            const auto predictedOmega = std::max(0.0, state_.angularVelocityRadPerSecond
                + state_.netTorqueNm / config_.rotatingInertiaKgM2 * subDt);
            const auto meanOmega = 0.5 * (state_.angularVelocityRadPerSecond + predictedOmega);
            const auto acceleration = radiusM * meanOmega * meanOmega
                * (cosTheta + exactTerm);

            const auto inertiaForce = massKg * acceleration;
            const auto leverArm = radiusM * (std::sin(theta) + (ratio * std::sin(2.0 * theta)) / (2.0 * std::sqrt(denom)));
            reciprocatingTorque -= inertiaForce * leverArm;
            // pistonAreaM2 is already defined above for the gas flow calls (same scope).
            const auto gasForceN = (chamberKpa - config_.ambientPressureKpa) * 1'000.0 * pistonAreaM2;
            gasIndicatedTorque += gasForceN * leverArm;
            const auto rodSideRatio = std::abs(radiusM * sinTheta
                / std::max(1.0e-6, rodM * std::sqrt(denom)));
            const auto cylinderWallForceN = std::abs(gasForceN - inertiaForce) * rodSideRatio;
            const auto pistonVelocityMps = radiusM * meanOmega
                * (sinTheta + ratio * std::sin(2.0 * theta) / (2.0 * std::sqrt(denom)));
            const auto pistonFrictionForceN = stribeckFrictionForce(cylinder, pistonVelocityMps,
                                                                    cylinderWallForceN);
            pistonFrictionPowerW += pistonFrictionForceN * std::abs(pistonVelocityMps);
            pistonBreakawayTorqueNm += pistonFrictionForceN * radiusM;
            peakPistonAcceleration = std::max(peakPistonAcceleration, std::abs(acceleration));
            pistonSpeedSum += 2.0 * cylinder.strokeMm * 0.001 * state_.rpm / 60.0;
        }
        // Discharge the shared collector exactly once per gas sub-step. This
        // closes the open-system mass/energy balance and makes outlet diameter
        // and muffler restriction affect the physical collector pressure.
        double outletMassKg = 0.0;
        collectorPressureKpa = config_.ambientPressureKpa;
        for (std::size_t pathIndex = 0; pathIndex < exhaustCollectorCount_; ++pathIndex) {
            const auto& geometry = exhaustGeometryAt(config_, pathIndex);
            const auto outletRadiusM = geometry.outletDiameterMm * 0.0005;
            const auto outletAreaM2 = std::numbers::pi * outletRadiusM * outletRadiusM;
            const auto outletCoefficient = std::clamp(geometry.outletDischargeCoefficient
                * (1.0 - geometry.mufflerRestriction * 0.72), 0.02, 1.2);
            const auto outletFlow = ConservativeGasSystem::flowFromBoundary(exhaustCollectorGas_[pathIndex],
                config_.ambientPressureKpa, config_.ambientTemperatureC + 273.15,
                outletAreaM2, outletCoefficient, subDt);
            outletMassKg += std::max(0.0, outletFlow.transferredMassKg);
            exhaustCollectorGas_[pathIndex].dissipateMomentum(
                std::max(0.002, geometry.primaryLengthMm / 300'000.0), subDt);
            collectorPressureKpa = std::max(collectorPressureKpa,
                exhaustCollectorGas_[pathIndex].pressureKpa());
        }

        const auto meanPistonSpeed = pistonSpeedSum / static_cast<double>(config_.cylinders.size());
        const auto pumpingTorque = state_.rpm > 30.0
            ? std::max(0.0, config_.ambientPressureKpa - state_.manifoldPressureKpa) * displacement * 0.055 : 0.0;
        const auto bearingFriction = state_.rpm > 1.0
            ? displacement * (1.4 + config_.frictionCoefficient * 10.0 + state_.rpm * 0.00055
                              + state_.rpm * state_.rpm * 0.000000035) : 0.0;
        const auto pistonFriction = state_.angularVelocityRadPerSecond > 1.0
            ? pistonFrictionPowerW / state_.angularVelocityRadPerSecond
            : pistonBreakawayTorqueNm;
        const auto seizureTorque = state_.damage >= 1.0 ? displacement * 250.0
            + state_.angularVelocityRadPerSecond * 0.22 : 0.0;
        const auto mechanicalFriction = bearingFriction + pistonFriction;
        const auto frictionTorque = mechanicalFriction + pumpingTorque + seizureTorque;
        const auto loadTorque = state_.rpm > 40.0 && !safeControls.starterEngaged
            ? state_.load * displacement * 105.0 : 0.0;
        // Automotive starters deliver high reduction torque at cranking speed;
        // this must overcome resolved compression peaks, not only mean friction.
        const auto starterPeakTorque = 45.0 + displacement * 52.0;
        const auto starterTorque = cranking ? starterPeakTorque * std::clamp(1.0 - state_.rpm / 760.0, 0.18, 1.0) : 0.0;
        const auto normalizedCombustionPulse = std::clamp(combustionPulseSum * 8.0
            / static_cast<double>(config_.cylinders.size()), 0.0, 2.8);
        const auto modeledIndicatedTorque = combustion.indicatedTorqueNm * normalizedCombustionPulse;
        // The conservative chamber-pressure solution is authoritative from the
        // first substep. Cranking torque fills/compresses the real control
        // volumes; no elapsed-time switch can fabricate startup work.
        const auto indicatedTorque = std::isfinite(gasIndicatedTorque) ? gasIndicatedTorque : 0.0;
        constexpr double pressureBlend = 1.0;
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
        maximumIntegratedCrankStep = std::max(maximumIntegratedCrankStep, std::abs(subTravelled));
        totalTravelled += subTravelled;
        state_.crankAngleDegrees = std::fmod(subPreviousAngle + subTravelled, eventGenerator_.cycleDegrees());
        state_.simulationTimeSeconds += subDt;

        state_.indicatedTorqueNm = indicatedTorque;
        state_.meanWorkTorqueNm = modeledIndicatedTorque;
        state_.cylinderPressureTorqueNm = gasIndicatedTorque;
        state_.cylinderPressureTorqueBlend = pressureBlend;
        state_.frictionTorqueNm = frictionTorque;
        state_.reciprocatingTorqueNm = boundedReciprocatingTorque;
        state_.meanPistonSpeedMps = meanPistonSpeed;
        state_.loadTorqueNm = loadTorque;
        state_.starterTorqueNm = starterTorque;
        state_.netTorqueNm = netTorque;
        state_.torqueNm = brakeTorque;
        state_.cycleAveragedTorqueNm = smooth(state_.cycleAveragedTorqueNm, brakeTorque, subDt, 8.0);
        state_.knockLevel = smooth(state_.knockLevel, physicalEndGasKnockLevel, subDt, 65.0);
        const auto misfiringCylinderCount = static_cast<double>(std::count(
            cylinderMisfires_.begin(), cylinderMisfires_.begin()
                + static_cast<std::ptrdiff_t>(config_.cylinders.size()), true));
        state_.misfireRate = smooth(state_.misfireRate,
            misfiringCylinderCount / static_cast<double>(config_.cylinders.size()), subDt, 12.0);
        state_.targetAirFuelRatio = ecuCommand.targetAirFuelRatio;
        state_.ignitionAdvanceDegrees = ecuCommand.ignitionAdvanceDegrees;
        const auto physicalAirMassMg = std::accumulate(trappedAirMassMgLastCycle_.begin(),
            trappedAirMassMgLastCycle_.begin()
                + static_cast<std::ptrdiff_t>(config_.cylinders.size()), 0.0);
        const auto ambientTemperatureK = config_.ambientTemperatureC + 273.15;
        const auto referenceAirMassMg = config_.ambientPressureKpa * 1'000.0
            * engineDisplacementLitres(config_) * 0.001
            / (GasCell::universalGasConstant / GasCell::airMolarMassKg * ambientTemperatureK)
            * 1.0e6;
        state_.airMassMgPerCycle = state_.rpm > 20.0 ? physicalAirMassMg : 0.0;
        state_.volumetricEfficiency = state_.rpm > 20.0
            ? std::clamp(physicalAirMassMg / std::max(1.0, referenceAirMassMg), 0.0, 2.5) : 0.0;
        const auto deliveredFuelMoles = std::accumulate(deliveredFuelMolesLastCycle_.begin(),
            deliveredFuelMolesLastCycle_.begin() + static_cast<std::ptrdiff_t>(config_.cylinders.size()), 0.0);
        state_.injectedFuelMgPerCycle = ecuCommand.fuelEnabled
            ? deliveredFuelMoles * config_.fuelProperties.molarMassGramsPerMole * 1'000.0 : 0.0;
        state_.airFuelRatio = state_.injectedFuelMgPerCycle > 1.0e-9
            ? state_.airMassMgPerCycle / state_.injectedFuelMgPerCycle
            : ecuCommand.targetAirFuelRatio;
        state_.fuelFlowGramsPerSecond = state_.injectedFuelMgPerCycle * (state_.rpm / 120.0) / 1'000.0;
        state_.fuelConsumedGrams += state_.fuelFlowGramsPerSecond * subDt;
        state_.fuelConsumedLitres = state_.fuelConsumedGrams * 0.001
            / config_.fuelProperties.densityKgPerL;
        state_.airFlowGramsPerSecond = state_.airMassMgPerCycle * (state_.rpm / 120.0) / 1'000.0;
        state_.lambda = state_.airFuelRatio / config_.fuelProperties.stoichiometricAirFuelRatio;
        state_.exhaustPressureKpa = collectorPressureKpa;
        state_.intakeRunnerPressureKpa = intakeRunnerPressureSum / static_cast<double>(config_.cylinders.size());
        state_.exhaustRunnerPressureKpa = exhaustRunnerPressureSum / static_cast<double>(config_.cylinders.size());
        const auto physicalOutletFlowGramsPerSecond = outletMassKg / subDt * 1'000.0;
        state_.exhaustFlowGramsPerSecond = smooth(state_.exhaustFlowGramsPerSecond,
            physicalOutletFlowGramsPerSecond, subDt, 35.0);
        state_.manifoldGasMassGrams = manifoldGas_.massKg() * 1'000.0;
        state_.cylinderGasMassGrams = 0.0;
        state_.gasInternalEnergyJoules = manifoldGas_.internalEnergyJoules();
        for (std::size_t pathIndex = 0; pathIndex < exhaustCollectorCount_; ++pathIndex)
            state_.gasInternalEnergyJoules += exhaustCollectorGas_[pathIndex].internalEnergyJoules();
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
            const auto active = (flameEvents_[index].active
                || instantaneousCombustionPulse_[index] > 1.0e-9) && !cylinderMisfires_[index];
            const auto pulse = active ? instantaneousCombustionPulse_[index] : 0.0;
            const auto cams = activeCamshaft(config_, cylinder, state_.rpm, state_.throttle);
            const auto intakeLift = profiledValveLiftMm(phase, 360.0 + cams.config->intakeCenterlineDegrees,
                                                        cams.intakeDuration(), cams.intakeLift(), cams.intakeProfile());
            const auto exhaustLift = profiledValveLiftMm(phase, 360.0 - cams.config->exhaustCenterlineDegrees,
                                                         cams.exhaustDuration(), cams.exhaustLift(), cams.exhaustProfile());
            const auto& gas = cylinderGas_[index];
            state_.cylinderStates[index] = { cylinder.id, phase,
                chamberPressureBar_[index],
                pulse, cylinderMisfires_[index] ? 1.0 : 0.0,
                intakeLift, exhaustLift, intakeFlowMgPerCycle_[index], exhaustFlowMgPerCycle_[index],
                exhaustRunnerPressureKpa_[index],
                config_.ambientTemperatureC + (state_.exhaustTemperatureC - config_.ambientTemperatureC)
                    * std::clamp(1.0 + cylinder.efficiencyOffset, 0.8, 1.2)
                    + (cylinderWallTemperatureC_[index] - config_.ambientTemperatureC) * 0.18,
                gas.temperatureK() - 273.15, gas.massKg() * 1.0e6,
                gas.mixture().oxygenMoles, gas.mixture().fuelMoles, gas.mixture().burnedMoles,
                intakeRunnerGas_[index].bulkVelocityMps(), exhaustRunnerGas_[index].bulkVelocityMps(),
                fuelDeliveryRatio_[index],
                flameEvents_[index].flameSpeedMps, flameEvents_[index].burnedFraction,
                flameEvents_[index].efficiency,
                endGasKnockStates_[index].filteredLevel,
                active, combustion.combustionEnabled && cylinderMisfires_[index] };
        }
        if (pressureSamples_) {
            CylinderPressureSample pressureSample;
            pressureSample.timeSeconds = state_.simulationTimeSeconds;
            pressureSample.cylinderCount = config_.cylinders.size();
            for (std::size_t index = 0; index < config_.cylinders.size(); ++index)
                pressureSample.pressureBar[index] = static_cast<float>(chamberPressureBar_[index]);
            if (pressureSamples_->tryPush(pressureSample))
                ++frame.cylinderPressureSampleCount;
            else
                ++frame.droppedCylinderPressureSampleCount;
        }
    }

    state_.crankDegreesPerSolverStep = maximumIntegratedCrankStep;
    state_.solverResolutionLimited = state_.solverResolutionLimited
        || maximumIntegratedCrankStep > config_.solver.maximumCrankDegreesPerStep + 1.0e-9;
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
    intakeFlowMgPerCycle_.fill(0.0);
    exhaustFlowMgPerCycle_.fill(0.0);
    instantaneousCombustionPulse_.fill(0.0);
    injectedFuelMolesThisCycle_.fill(0.0);
    deliveredFuelMolesLastCycle_.fill(0.0);
    requestedFuelMolesThisCycle_.fill(0.0);
    trappedAirMassMgLastCycle_.fill(0.0);
    fuelDeliveryRatio_.fill(0.0);
    flameEvents_.fill({});
    injectionStates_.fill({});
    endGasKnockStates_.fill({});
    const auto configureFuel = [this](GasCell& cell) {
        cell.configureFuelChemistry(config_.fuelProperties.molarMassGramsPerMole * 0.001,
            config_.fuelProperties.oxygenMolesPerFuelMole,
            config_.fuelProperties.productMolesPerFuelMole);
    };
    configureFuel(manifoldGas_);
    manifoldGas_.initialise(config_.ambientPressureKpa, config_.plenumVolumeLitres,
                            config_.ambientTemperatureC + 273.15);
    // Manifold: large plenum, no strong directional bias — neutral orientation.
    {
        const auto throttleRadius = config_.intake.throttleDiameterMm * 0.0005;
        const auto throttleArea = std::numbers::pi * throttleRadius * throttleRadius;
        manifoldGas_.setGeometry(throttleArea, 0.0, 1.0);
    }

    exhaustCollectorCount_ = std::max<std::size_t>(1,
        std::min<std::size_t>(config_.exhaustPaths.size(), exhaustCollectorGas_.size()));
    for (std::size_t pathIndex = 0; pathIndex < exhaustCollectorCount_; ++pathIndex) {
        const auto& geometry = exhaustGeometryAt(config_, pathIndex);
        configureFuel(exhaustCollectorGas_[pathIndex]);
        exhaustCollectorGas_[pathIndex].initialise(config_.ambientPressureKpa,
            geometry.collectorVolumeLitres, config_.ambientTemperatureC + 273.15);
        // Collector flows toward the outlet (positive x direction by convention).
        const auto collectorRadiusM = geometry.collectorDiameterMm * 0.0005;
        exhaustCollectorGas_[pathIndex].setGeometry(
            std::numbers::pi * collectorRadiusM * collectorRadiusM, 1.0, 0.0);
    }
    for (std::size_t index = 0; index < config_.cylinders.size(); ++index) {
        previousCylinderPhases_[index] = std::fmod(state_.crankAngleDegrees
            - crankOffsetDegreesFor(config_, config_.cylinders[index]) + 1'440.0, 720.0);
        intakeRunnerPressureKpa_[index] = config_.ambientPressureKpa;
        exhaustRunnerPressureKpa_[index] = config_.ambientPressureKpa;
        chamberPressureBar_[index] = config_.ambientPressureKpa / 100.0;
        cylinderWallTemperatureC_[index] = config_.ambientTemperatureC;
        configureFuel(intakeRunnerGas_[index]);
        configureFuel(cylinderGas_[index]);
        configureFuel(exhaustRunnerGas_[index]);

        const auto& cyl = config_.cylinders[index];
        const auto runnerDiamMm = cyl.intakeRunnerDiameterMm > 0.0
            ? cyl.intakeRunnerDiameterMm : config_.intake.runnerDiameterMm;
        const auto runnerAreaM2 = std::numbers::pi * std::pow(runnerDiamMm * 0.0005, 2.0);
        const auto pistonAreaM2 = std::numbers::pi * std::pow(cyl.boreMm * 0.0005, 2.0);
        const auto& cylExhaust = exhaustGeometryFor(config_, cyl);
        const auto primaryAreaM2 = std::numbers::pi * std::pow(cylExhaust.primaryDiameterMm * 0.0005, 2.0);

        // Geometry: positive momentum means gas flowing downward into the cylinder
        // for intake, and upward out of the cylinder for exhaust.
        intakeRunnerGas_[index].setGeometry(runnerAreaM2, 0.0,  1.0); // downward
        cylinderGas_[index].setGeometry(pistonAreaM2,    0.0, -1.0); // upward toward head
        exhaustRunnerGas_[index].setGeometry(primaryAreaM2, 0.0, -1.0); // upward/outward

        intakeRunnerGas_[index].initialise(config_.ambientPressureKpa, 0.18,
                                           config_.ambientTemperatureC + 273.15);
        cylinderGas_[index].initialise(config_.ambientPressureKpa,
            chamberVolumeLitres(config_.cylinders[index], std::fmod(state_.crankAngleDegrees
                - mechanicalCrankOffsetDegreesFor(config_, config_.cylinders[index]) + 720.0, 360.0)),
            config_.ambientTemperatureC + 273.15);
        const auto& cylinderExhaust = exhaustGeometryFor(config_, config_.cylinders[index]);
        const auto primaryLengthMm = config_.cylinders[index].exhaustPrimaryLengthMm > 0.0
            ? config_.cylinders[index].exhaustPrimaryLengthMm : cylinderExhaust.primaryLengthMm;
        exhaustRunnerGas_[index].initialise(config_.ambientPressureKpa,
            std::max(0.10, primaryLengthMm * std::numbers::pi
                * std::pow(cylinderExhaust.primaryDiameterMm * 0.5, 2.0) / 1.0e6),
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
