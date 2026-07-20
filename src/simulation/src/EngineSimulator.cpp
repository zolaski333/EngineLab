#include <enginelab/simulation/EngineSimulator.hpp>
#include <enginelab/simulation/SubstepParallel.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/MechanicalKinematics.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <numbers>
#include <numeric>
#include <optional>
#include <span>
#include <stdexcept>

namespace enginelab {
namespace {
[[nodiscard]] double smooth(double current, double target, double dt, double rate) noexcept {
    return current + (target - current) * (1.0 - std::exp(-dt * rate));
}

[[nodiscard]] gasdynamics::ThermodynamicModel exhaustThermodynamicsFor(
    const EngineConfig& config) noexcept {
    auto model = gasdynamics::ThermodynamicModel::standardCombustionGas();
    const auto fuelIndex = static_cast<std::size_t>(gasdynamics::GasSpecies::fuel);
    const auto burnedIndex = static_cast<std::size_t>(gasdynamics::GasSpecies::burned);
    const auto fuelMolarMassKg = config.fuelProperties.molarMassGramsPerMole * 0.001;
    model.species[fuelIndex].molarMassKgPerMol = fuelMolarMassKg;
    model.species[burnedIndex].molarMassKgPerMol =
        (fuelMolarMassKg
         + config.fuelProperties.oxygenMolesPerFuelMole * GasCell::oxygenMolarMassKg)
        / config.fuelProperties.productMolesPerFuelMole;
    return model;
}

[[nodiscard]] gasdynamics::ConservativeState networkStateForGasCell(
    const GasCell& cell) noexcept {
    gasdynamics::ConservativeState result;
    const auto inverseVolume = 1.0 / cell.volumeM3();
    result.speciesMassDensityKgPerM3[
        static_cast<std::size_t>(gasdynamics::GasSpecies::oxygen)] =
        cell.mixture().oxygenMoles * GasCell::oxygenMolarMassKg * inverseVolume;
    result.speciesMassDensityKgPerM3[
        static_cast<std::size_t>(gasdynamics::GasSpecies::inert)] =
        cell.mixture().inertMoles * GasCell::inertMolarMassKg * inverseVolume;
    result.speciesMassDensityKgPerM3[
        static_cast<std::size_t>(gasdynamics::GasSpecies::fuel)] =
        cell.mixture().fuelMoles * cell.configuredFuelMolarMassKg() * inverseVolume;
    result.speciesMassDensityKgPerM3[
        static_cast<std::size_t>(gasdynamics::GasSpecies::burned)] =
        cell.mixture().burnedMoles * cell.configuredBurnedGasMolarMassKg() * inverseVolume;
    // The chamber is a zero-dimensional reservoir. Its resolved bulk momentum
    // has no unique exhaust-axis projection; valve-wall reactions carry it.
    result.momentumDensityKgPerM2S = 0.0;
    result.totalEnergyDensityJPerM3 = cell.internalEnergyJoules() * inverseVolume;
    return result;
}

[[nodiscard]] GasInventoryDelta cylinderDeltaForExchange(
    const gasdynamics::CylinderGasExchange& exchange,
    const GasCell& cell) noexcept {
    const auto oxygen = static_cast<std::size_t>(gasdynamics::GasSpecies::oxygen);
    const auto inert = static_cast<std::size_t>(gasdynamics::GasSpecies::inert);
    const auto fuel = static_cast<std::size_t>(gasdynamics::GasSpecies::fuel);
    const auto burned = static_cast<std::size_t>(gasdynamics::GasSpecies::burned);
    return {
        {
            -exchange.speciesMassKg[oxygen] / GasCell::oxygenMolarMassKg,
            -exchange.speciesMassKg[inert] / GasCell::inertMolarMassKg,
            -exchange.speciesMassKg[fuel] / cell.configuredFuelMolarMassKg(),
            -exchange.speciesMassKg[burned] / cell.configuredBurnedGasMolarMassKg(),
        },
        -exchange.totalEnergyJ,
        0.0,
        0.0,
    };
}

// Below this cylinder count the per-cylinder loop runs serially: the fork-join
// dispatch latency outweighs the work, and the serial path stays bit-identical.
constexpr std::size_t parallelCylinderThreshold = 8;

// Private per-cylinder scratch for one gas sub-step. Every cross-cylinder result
// the loop used to accumulate into a shared scalar (or into a shared plenum /
// collector) is written here per cylinder instead, so the per-cylinder body has
// no shared write and can run in any order. The fixed-order reductions after the
// loop reproduce the original serial sums bit-for-bit.
struct CylinderSubstepScratch final {
    double reciprocatingTorque {};
    double combustionPulse {};
    double gasIndicatedTorque {};
    double intakeRunnerPressure {};
    double exhaustRunnerPressure {};
    double exhaustTemperatureK {};
    double pistonFrictionPowerW {};
    double pistonBreakawayTorqueNm {};
    double pistonSpeed {};
    double peakPistonAcceleration {};
    double endGasKnockLevel {};
    double releasedEnergyJoules {};
    double meteredFuelMassKg {};
    // Jacobi branch against the frozen shared intake plenum.
    JacobiGasFlowBranch plenumFlow {};
    std::size_t intakePath {};
};

struct ParallelSubstepState final {
    std::array<GasCell, 32> frozenPlenum {};
    std::array<CylinderSubstepScratch, 32> scratch {};
};

[[nodiscard]] double valveAreaMm2(double boreMm, double liftMm,
                                  std::uint32_t valveCount,
                                  double configuredDiameterMm,
                                  double multiValveBoreRatio,
                                  double singleValveBoreRatio) noexcept {
    const auto count = static_cast<double>(std::clamp<std::uint32_t>(valveCount, 1, 4));
    const auto derivedRatio = valveCount == 1 ? singleValveBoreRatio : multiValveBoreRatio;
    const auto diameterMm = configuredDiameterMm > 0.0
        ? configuredDiameterMm : std::clamp(boreMm * derivedRatio, 12.0, 80.0);
    const auto curtainAreaMm2 = count * std::numbers::pi * diameterMm
        * std::max(0.0, liftMm);
    // Curtain flow cannot exceed the valve-seat throat even at very high lift.
    constexpr double seatThroatRatio = 0.88;
    const auto throatDiameterMm = diameterMm * seatThroatRatio;
    const auto throatAreaMm2 = count * std::numbers::pi * throatDiameterMm
        * throatDiameterMm * 0.25;
    return std::min(curtainAreaMm2, throatAreaMm2);
}

[[nodiscard]] double crankOffsetDegreesFor(const EngineConfig& config, const CylinderConfig& cylinder) noexcept {
    (void)config;
    return cylinder.crankOffsetDegrees;
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

[[nodiscard]] std::size_t intakePathIndexFor(const EngineConfig& config,
                                             const CylinderConfig& cylinder) noexcept {
    const auto path = std::find_if(config.intakePaths.begin(), config.intakePaths.end(),
        [&cylinder](const IntakePathConfig& item) {
            return std::find(item.cylinderIds.begin(), item.cylinderIds.end(), cylinder.id)
                != item.cylinderIds.end();
        });
    return path != config.intakePaths.end()
        ? static_cast<std::size_t>(std::distance(config.intakePaths.begin(), path)) : 0U;
}

[[nodiscard]] const IntakeConfig& intakeGeometryAt(const EngineConfig& config,
                                                   std::size_t pathIndex) noexcept {
    return pathIndex < config.intakePaths.size() ? config.intakePaths[pathIndex].geometry : config.intake;
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
    const auto fuelMoles = cell.mixture().fuelMoles;
    if (fuelMoles <= 1.0e-15) return 100.0;
    // Residual nitrogen remains in the cylinder after combustion and cannot be
    // counted as fresh intake air.  Mixture strength is therefore derived from
    // the oxygen actually available to the fuel, then converted back to the
    // calibrated mass AFR for display and ECU feedback.
    const auto lambda = cell.mixture().oxygenMoles
        / std::max(1.0e-15, fuelMoles * fuel.oxygenMolesPerFuelMole);
    return std::clamp(lambda * fuel.stoichiometricAirFuelRatio, 0.0, 100.0);
}

[[nodiscard]] double stribeckFrictionForce(const CylinderConfig& cylinder,
                                            double pistonSpeedMps,
                                            double cylinderWallForceN) noexcept {
    const auto velocity = std::abs(pistonSpeedMps);
    const auto coulomb = cylinder.pistonFrictionCoefficient * std::abs(cylinderWallForceN);
    const auto stribeckVelocity = std::max(1.0e-6, cylinder.pistonBreakawayVelocityMps);
    const auto staticFriction = std::max(coulomb, cylinder.pistonBreakawayForceN);
    const auto stribeck = coulomb + (staticFriction - coulomb)
        * std::exp(-std::pow(velocity / stribeckVelocity, 2.0));
    return std::max(0.0, stribeck + cylinder.pistonViscousFrictionNsPerM * velocity);
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
    normaliseEngineConfig(config_);
    if (const auto error = validateEngineConfig(config_)) throw std::invalid_argument(*error);
    ecu_.initialise(config_);
    kinematicsReference_ = buildEngineKinematicsReference(config_);
    configurePhysicalExhaustNetwork();
    reset();
}

void EngineSimulator::configurePhysicalExhaustNetwork() {
    const auto physicalGraph = ExhaustGraph::makeForEngine(config_);
    gasdynamics::ExhaustNetworkDiscretisation feedbackMesh;
    // The realtime FV mesh owns nonlinear mean-flow/back-pressure feedback,
    // not audio-band propagation. 75 mm retains at least six cells per
    // wavelength through ~650 Hz at the cold-gas sound speed, covering the
    // firing fundamentals of the shipped engines. The characteristic audio
    // network transports the remaining band without asking every exhaust cell
    // to run at more than 100 kHz.
    feedbackMesh.targetCellLengthM = 0.075;
    feedbackMesh.maximumCellsPerDuct = 64;
    feedbackMesh.maximumTotalCells = 1'024;
    const auto layout = gasdynamics::ExhaustNetworkLayout::compile(
        physicalGraph, feedbackMesh);
    auto network = std::make_unique<gasdynamics::ExhaustGasNetwork>(
        exhaustThermodynamicsFor(config_));
    gasdynamics::ExhaustGasNetworkConfig networkConfig;
    networkConfig.initialPressurePa = config_.ambientPressureKpa * 1'000.0;
    networkConfig.initialTemperatureK = config_.ambientTemperatureC + 273.15;
    networkConfig.absoluteRoughnessM = 4.5e-5;
    networkConfig.maximumCourantNumber = 0.8;
    // Wall thermal inertia belongs to a separate conserved subsystem. Until it
    // exists, adiabatic walls are more honest than a fixed fabricated wall
    // temperature which would create or destroy untracked energy.
    networkConfig.wallHeatTransferWPerM2K = 0.0;
    if (!network->configure(layout, networkConfig))
        throw std::runtime_error("failed to configure conservative exhaust network");
    const auto ambientState = network->mixtureModel().conservativeFromPressureTemperature(
        config_.ambientPressureKpa * 1'000.0,
        config_.ambientTemperatureC + 273.15);
    if (!ambientState)
        throw std::runtime_error("failed to construct physical exhaust ambient state");
    physicalExhaustAmbientState_ = *ambientState;
    for (std::size_t portIndex = 0;
         portIndex < layout.cylinderPorts().size(); ++portIndex) {
        const auto cylinderId = layout.cylinderPorts()[portIndex].cylinderId;
        const auto cylinder = std::find_if(config_.cylinders.begin(), config_.cylinders.end(),
            [cylinderId](const CylinderConfig& candidate) {
                return candidate.id == cylinderId;
            });
        if (cylinder == config_.cylinders.end())
            throw std::runtime_error("exhaust network references an unknown cylinder");
        exhaustNetworkCylinderIndex_[portIndex] = static_cast<std::size_t>(
            std::distance(config_.cylinders.begin(), cylinder));
    }
    physicalExhaustNetwork_ = std::move(network);
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
    ecu_.beginFrame();
    safeControls.throttle = std::isfinite(controls.throttle) ? std::clamp(controls.throttle, 0.0, 1.0) : 0.0;
    safeControls.load = std::isfinite(controls.load) ? std::clamp(controls.load, 0.0, 1.0) : 0.0;
    safeControls.externalTorqueNm = std::isfinite(controls.externalTorqueNm)
        ? std::clamp(controls.externalTorqueNm, -5'000.0, 5'000.0) : 0.0;
    safeControls.dynamometerTorqueNm = std::isfinite(controls.dynamometerTorqueNm)
        ? std::clamp(controls.dynamometerTorqueNm, 0.0, 10'000.0) : 0.0;
    // Gas pressure and crank loading need much finer resolution than UI/runtime
    // updates. Resolution increases with crank speed and remains bounded by the
    // engine definition so slow machines fail observably instead of diverging.
    // Reserve crank-angle headroom for acceleration during the outer runtime
    // step.  Sizing only from the entry RPM can otherwise exceed the declared
    // angular resolution even though the nominal frequency is sufficient.
    const auto guardedCrankDegrees = std::max(0.1,
        config_.solver.maximumCrankDegreesPerStep * 0.94);
    const auto angleFrequency = std::abs(state_.rpm) * 6.0 / guardedCrankDegrees;
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
    double maximumIntegratedCrankStep = 0.0;
    SimulationFrame frame;

    for (std::size_t subStep = 0; subStep < subStepCount; ++subStep) {
        const auto subStepStartTime = state_.simulationTimeSeconds;
        const auto subPreviousRpm = state_.rpm;
        const auto subPreviousAngle = state_.crankAngleDegrees;

        // Engine load is a thermodynamic state (approximately MAP / ambient
        // for a naturally aspirated SI engine), not the operator's brake
        // command.  Coupling flame turbulence and VVT to the dyno slider made
        // a wide-open, unloaded engine look like a zero-load combustion event.
        const auto thermodynamicLoad = std::clamp(state_.manifoldPressureKpa
            / std::max(1.0, config_.ambientPressureKpa), 0.0, 1.5);
        state_.load = smooth(state_.load, thermodynamicLoad, subDt, 12.0);
        const auto ecuCommand = ecu_.evaluate(config_, state_, safeControls);
        state_.throttle = smooth(state_.throttle, ecuCommand.effectiveThrottle, subDt, 10.0);
        const auto stationary = state_.rpm < 20.0 && !safeControls.starterEngaged;
        if (stationary) {
            state_.boostPressureRatio = 1.0;
            state_.forcedInductionShaftSpeedRpm = 0.0;
            state_.wastegateOpening = 0.0;
            state_.compressorPowerKw = 0.0;
            state_.turbinePowerKw = 0.0;
            for (std::size_t pathIndex = 0; pathIndex < intakePlenumCount_; ++pathIndex) {
                const auto& intake = intakeGeometryAt(config_, pathIndex);
                const auto throttleRadiusM = intake.throttleDiameterMm * 0.0005;
                const auto throttlePlateAreaM2 = static_cast<double>(intake.throttleCount)
                    * std::numbers::pi * throttleRadiusM * throttleRadiusM;
                const auto throttleAreaM2 = intake.idleBypassAreaMm2
                        * ecuCommand.idleAirOpening * 1.0e-6
                    + throttlePlateAreaM2 * std::pow(state_.throttle, intake.throttleGamma);
                (void)ConservativeGasSystem::flowFromBoundary(intakePlenumGas_[pathIndex],
                    config_.ambientPressureKpa, config_.ambientTemperatureC + 273.15,
                    throttleAreaM2, intake.throttleDischargeCoefficient, subDt,
                    0.0, -1.0); // plenum -> upstream atmosphere
            }
        } else {
            constexpr double airCpJPerKgK = 1'005.0;
            constexpr double exhaustCpJPerKgK = 1'120.0;
            constexpr double compressorExponent = 0.285714285714;
            const auto ambientTemperatureK = config_.ambientTemperatureC + 273.15;
            const auto airMassFlowKgPerSecond = std::max(0.0, state_.airFlowGramsPerSecond) * 0.001;
            const auto exhaustMassFlowKgPerSecond = std::max(0.0, state_.exhaustFlowGramsPerSecond) * 0.001;
            auto pressureRatioTarget = 1.0;
            auto desiredCompressorPowerW = 0.0;
            auto turbinePowerW = 0.0;
            const auto exhaustTemperatureK = std::max(ambientTemperatureK, state_.exhaustTemperatureC + 273.15);
            if (config_.forcedInduction.enabled
                    && config_.forcedInduction.type == ForcedInductionType::supercharger) {
                const auto speedRatio = std::clamp(state_.rpm
                    / std::max(1.0, config_.forcedInduction.fullBoostRpm), 0.0, 1.0);
                const auto drive = speedRatio * std::clamp(std::pow(state_.throttle, 0.72), 0.0, 1.0);
                pressureRatioTarget = 1.0
                    + (config_.forcedInduction.pressureRatio - 1.0) * drive;
                state_.forcedInductionShaftSpeedRpm = 0.0;
                state_.wastegateOpening = 0.0;
            } else if (config_.forcedInduction.enabled) {
                const auto designOmega = config_.forcedInduction.designShaftSpeedRpm
                    * 2.0 * std::numbers::pi / 60.0;
                auto shaftOmega = state_.forcedInductionShaftSpeedRpm
                    * 2.0 * std::numbers::pi / 60.0;
                const auto turbineInletPressureKpa = std::max(state_.exhaustPressureKpa,
                                                               state_.exhaustRunnerPressureKpa);
                const auto turbineExpansionRatio = std::max(1.0,
                    turbineInletPressureKpa / std::max(1.0, config_.ambientPressureKpa));
                turbinePowerW = exhaustMassFlowKgPerSecond * exhaustCpJPerKgK * exhaustTemperatureK
                    * (1.0 - std::pow(turbineExpansionRatio, -compressorExponent))
                    * config_.forcedInduction.turbineEfficiency;
                desiredCompressorPowerW = airMassFlowKgPerSecond * airCpJPerKgK * ambientTemperatureK
                    * (std::pow(std::max(1.0, state_.boostPressureRatio), compressorExponent) - 1.0)
                    / std::max(0.35, config_.forcedInduction.compressorEfficiency);
                state_.wastegateOpening = std::clamp((state_.boostPressureRatio
                    - config_.forcedInduction.wastegatePressureRatio + 0.02) / 0.08, 0.0, 1.0);
                const auto speedRatio = shaftOmega / std::max(1.0, designOmega);
                const auto bearingPowerW = config_.forcedInduction.bearingFrictionPowerWatts
                    * speedRatio * speedRatio;
                const auto shaftPowerW = turbinePowerW * (1.0 - state_.wastegateOpening * 0.94)
                    - desiredCompressorPowerW - bearingPowerW;
                auto shaftEnergyJ = 0.5 * config_.forcedInduction.shaftInertiaKgM2
                    * shaftOmega * shaftOmega;
                shaftEnergyJ = std::max(0.0, shaftEnergyJ + shaftPowerW * subDt);
                shaftOmega = std::min(std::sqrt(2.0 * shaftEnergyJ
                    / config_.forcedInduction.shaftInertiaKgM2), designOmega * 1.16);
                state_.forcedInductionShaftSpeedRpm = shaftOmega * 60.0
                    / (2.0 * std::numbers::pi);
                const auto compressorSpeedRatio = std::clamp(shaftOmega
                    / std::max(1.0, designOmega), 0.0, 1.12);
                pressureRatioTarget = 1.0 + (config_.forcedInduction.pressureRatio - 1.0)
                    * compressorSpeedRatio * compressorSpeedRatio
                    * std::clamp(std::pow(state_.throttle, 0.38), 0.0, 1.0);
                pressureRatioTarget = std::min(pressureRatioTarget,
                    config_.forcedInduction.wastegatePressureRatio + 0.04);
            }
            desiredCompressorPowerW = airMassFlowKgPerSecond * airCpJPerKgK * ambientTemperatureK
                * (std::pow(std::max(1.0, pressureRatioTarget), compressorExponent) - 1.0)
                / std::max(0.35, config_.forcedInduction.compressorEfficiency);
            state_.compressorPowerKw = desiredCompressorPowerW * 0.001;
            state_.turbinePowerKw = turbinePowerW * 0.001;
            state_.boostPressureRatio = smooth(state_.boostPressureRatio,
                config_.forcedInduction.enabled ? pressureRatioTarget : 1.0, subDt, 14.0);
            if (!config_.forcedInduction.enabled) {
                state_.forcedInductionShaftSpeedRpm = 0.0;
                state_.wastegateOpening = 0.0;
            }
            const auto boostBlend = std::clamp((state_.boostPressureRatio - 1.0)
                / std::max(0.01, config_.forcedInduction.pressureRatio - 1.0), 0.0, 1.0);
            const auto intakeSourcePressureKpa = config_.ambientPressureKpa * state_.boostPressureRatio;
            const auto isentropicRiseC = ambientTemperatureK
                * (std::pow(std::max(1.0, state_.boostPressureRatio), compressorExponent) - 1.0)
                / std::max(0.35, config_.forcedInduction.compressorEfficiency);
            const auto chargeTemperatureC = config_.ambientTemperatureC + isentropicRiseC
                + config_.forcedInduction.chargeTemperatureRiseC * 0.15 * boostBlend;
            const auto temperatureK = std::max(240.0, chargeTemperatureC + 273.15);

            for (std::size_t pathIndex = 0; pathIndex < intakePlenumCount_; ++pathIndex) {
                const auto& intake = intakeGeometryAt(config_, pathIndex);
                const auto throttleRadiusM = intake.throttleDiameterMm * 0.0005;
                const auto throttlePlateAreaM2 = static_cast<double>(intake.throttleCount)
                    * std::numbers::pi * throttleRadiusM * throttleRadiusM;
                const auto idleBypassAreaM2 = intake.idleBypassAreaMm2
                    * ecuCommand.idleAirOpening * 1.0e-6;
                const auto throttleAreaM2 = idleBypassAreaM2
                    + throttlePlateAreaM2 * std::pow(state_.throttle, intake.throttleGamma);
                (void)ConservativeGasSystem::flowFromBoundary(intakePlenumGas_[pathIndex], intakeSourcePressureKpa,
                    temperatureK, throttleAreaM2, intake.throttleDischargeCoefficient, subDt,
                    0.0, -1.0); // plenum -> upstream compressor/atmosphere
            }
        }
        // Resolve plenum jet mixing as a sudden expansion loss. The equivalent
        // spherical diameter supplies its characteristic length; no numerical
        // decay time is introduced.
        for (std::size_t pathIndex = 0; pathIndex < intakePlenumCount_; ++pathIndex) {
            const auto volumeM3 = intakePlenumGas_[pathIndex].volumeM3();
            const auto hydraulicDiameterM = std::cbrt(6.0 * volumeM3 / std::numbers::pi);
            const auto plenumAreaM2 = std::numbers::pi * hydraulicDiameterM
                * hydraulicDiameterM * 0.25;
            const auto inletAreaRatio = std::clamp(
                intakePlenumGas_[pathIndex].characteristicAreaM2()
                    / std::max(1.0e-9, plenumAreaM2), 0.0, 1.0);
            const auto suddenExpansionLoss = std::pow(1.0 - inletAreaRatio, 2.0);
            intakePlenumGas_[pathIndex].applyFlowResistance(
                hydraulicDiameterM, hydraulicDiameterM, 1.5e-6,
                suddenExpansionLoss, subDt);
        }
        state_.manifoldPressureKpa = 0.0;
        for (std::size_t pathIndex = 0; pathIndex < intakePlenumCount_; ++pathIndex)
            state_.manifoldPressureKpa += intakePlenumGas_[pathIndex].pressureKpa();
        state_.manifoldPressureKpa /= static_cast<double>(intakePlenumCount_);

        // The nonlinear network is mandatory and is compiled by the
        // constructor. There is intentionally no lumped-exhaust fallback.
        auto& physicalExhaustNetwork = *physicalExhaustNetwork_;
        double collectorPressureKpa = config_.ambientPressureKpa;
        for (const auto& exchange : physicalExhaustNetwork.cylinderExchanges())
            collectorPressureKpa = std::max(
                collectorPressureKpa, exchange.networkPressurePa * 0.001);
        state_.exhaustPressureKpa = collectorPressureKpa;
        const auto backPressure = state_.exhaustPressureKpa;
        const auto combustion = physics_.evaluateCombustion(config_, state_, safeControls, ecuCommand, backPressure);
        const auto sparkCombustionAvailable = ecuCommand.sparkEnabled
            && state_.rpm >= 220.0 && state_.damage < 1.0;
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
        double releasedEnergyJoules = 0.0;
        double meteredFuelMassKg = 0.0;
        double exhaustTemperatureSumK = 0.0;
        std::array<double, 32> exhaustValveAreaM2 {};
        std::array<double, 32> exhaustValveDischargeCoefficient {};
        std::array<double, 32> intakeValveAreaM2 {};
        std::array<double, 32> intakeValveDischargeCoefficient {};
        std::array<double, 32> intakeRunnerAreaM2ForSplit {};
        std::array<double, 32> pistonAreaM2ForSplit {};
        std::array<double, 32> chamberVolumeLitresForWork {};
        std::array<double, 32> chamberPressureBeforeNetworkKpa {};
        std::array<double, 32> gasTorqueLeverArmM {};
        std::array<double, 32> exhaustMassFlowKgPerSecond {};
        std::array<double, 32> exhaustPortDensityKgPerM3 {};
        std::array<double, 32> exhaustPortSpeedOfSoundMps {};
        std::array<std::uint8_t, 32> thermoacousticBoundaryValid {};
        std::array<bool, 32> cycleBoundaryForWork {};
        std::array<bool, 32> intakeCloseForTrappedAir {};

        // Engines below the parallel threshold retain the original serial
        // Gauss-Seidel shared-volume updates and shared PRNG sequence. Larger
        // engines use a Jacobi snapshot even if this machine has only one usable
        // participant, so their physics and PRNG sequence do not depend on the
        // host's hardware concurrency.
        const auto decoupleSharedVolumes =
            config_.cylinders.size() >= parallelCylinderThreshold;
        // Freeze both shared control volumes at the start of the sub-step so that
        // every parallel cylinder evaluates the same upstream/downstream state.
        // ConservativeGasSystem commits the fixed-order N-way branch transaction
        // after the loop, including overdraw scaling and kinetic-energy correction.
        std::optional<ParallelSubstepState> parallelState;
        if (decoupleSharedVolumes) {
            parallelState.emplace();
            for (std::size_t path = 0; path < intakePlenumCount_; ++path)
                parallelState->frozenPlenum[path] = intakePlenumGas_[path];
        }
        const auto totalCellEnergy = [](const GasCell& cell) noexcept {
            return cell.internalEnergyJoules() + cell.bulkKineticEnergyJoules();
        };
        const auto accumulateContribution = [&](const CylinderSubstepScratch& contribution) noexcept {
            reciprocatingTorque += contribution.reciprocatingTorque;
            combustionPulseSum += contribution.combustionPulse;
            gasIndicatedTorque += contribution.gasIndicatedTorque;
            intakeRunnerPressureSum += contribution.intakeRunnerPressure;
            exhaustRunnerPressureSum += contribution.exhaustRunnerPressure;
            exhaustTemperatureSumK += contribution.exhaustTemperatureK;
            pistonFrictionPowerW += contribution.pistonFrictionPowerW;
            pistonBreakawayTorqueNm += contribution.pistonBreakawayTorqueNm;
            pistonSpeedSum += contribution.pistonSpeed;
            peakPistonAcceleration = std::max(
                peakPistonAcceleration, contribution.peakPistonAcceleration);
            physicalEndGasKnockLevel = std::max(
                physicalEndGasKnockLevel, contribution.endGasKnockLevel);
            releasedEnergyJoules += contribution.releasedEnergyJoules;
            meteredFuelMassKg += contribution.meteredFuelMassKg;
        };

        const auto processCylinder = [&](std::size_t cylinderIndex) {
            CylinderSubstepScratch serialScratch {};
            auto& contribution = decoupleSharedVolumes
                ? parallelState->scratch[cylinderIndex] : serialScratch;
            const auto& cylinder = config_.cylinders[cylinderIndex];
            const auto crankOffset = crankOffsetDegreesFor(config_, cylinder);
            const auto cyclePhase = std::fmod(state_.crankAngleDegrees - crankOffset + 1'440.0, 720.0);
            const auto cams = activeCamshaft(config_, cylinder, state_.rpm, state_.throttle);
            const auto previousPhase = previousCylinderPhases_[cylinderIndex];
            const auto cycleBoundaryCrossed = crossedPhase(previousPhase, cyclePhase, 0.0);
            if (cycleBoundaryCrossed) {
                if (cylinderFlowCycleStarted_[cylinderIndex]) {
                    intakeFlowMgPerCycle_[cylinderIndex] = std::max(0.0,
                        intakeFlowMgThisCycle_[cylinderIndex]);
                    exhaustFlowMgPerCycle_[cylinderIndex] = std::max(0.0,
                        exhaustFlowMgThisCycle_[cylinderIndex]);
                } else {
                    cylinderFlowCycleStarted_[cylinderIndex] = true;
                }
                intakeFlowMgThisCycle_[cylinderIndex] = 0.0;
                exhaustFlowMgThisCycle_[cylinderIndex] = 0.0;
            }
            const auto predictedAngularAcceleration = state_.netTorqueNm / effectiveRotatingInertiaKgM2(config_);
            const auto kinematics = evaluateCylinderKinematics(config_, kinematicsReference_, cylinderIndex,
                state_.crankAngleDegrees, state_.angularVelocityRadPerSecond,
                predictedAngularAcceleration);
            const auto chamberVolume = kinematics.chamberVolumeLitres;
            cylinderGas_[cylinderIndex].setVolumeAdiabatic(chamberVolume);
            valveTrainResults_[cylinderIndex] = ValveTrainModel::evaluate(*cams.config, cams.highProfile,
                valveTrainStates_[cylinderIndex], cyclePhase, state_.rpm, state_.load, subDt);
            const auto& valveTrain = valveTrainResults_[cylinderIndex];
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
            const auto physicalFuelTargetMoles = ecuCommand.fuelEnabled
                ? measuredChargeMassMg
                    / std::clamp(ecuCommand.targetAirFuelRatio, 5.0, 30.0)
                    * ecuCommand.fuelCorrection * closedLoopFuelTrim_[cylinderIndex]
                    * 1.0e-6 / fuelMolarMassKg
                : 0.0;
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
                // Meter against the complete fuel inventory that can reach the
                // next trapped charge.  Port injection previously ignored fuel
                // already resident in the cylinder, so every incomplete cycle
                // added another full pulse and progressively flooded large or
                // slow-running engines.
                // Fuel in a closed cylinder belongs to the charge that is
                // currently being compressed or burned; it cannot satisfy the
                // following port-injection request. It becomes part of the
                // reachable inventory only while the intake valve actually
                // connects the chamber and runner (including reversion).
                const auto trappedCylinderFuel = config_.injection.mode == InjectionMode::port
                    && valveTrain.intakeLiftMm > 0.01
                    ? cylinderGas_[cylinderIndex].mixture().fuelMoles : 0.0;
                const auto crankDegreesToSpark = std::fmod(sparkPhase - cyclePhase + 720.0, 720.0);
                const auto secondsToSpark = state_.rpm > 20.0
                    ? crankDegreesToSpark / (state_.rpm * 6.0) : 0.25;
                const auto filmTemperatureFactor = std::clamp(
                    (injectionTarget.temperatureK() - 240.0) / 120.0, 0.08, 2.0);
                const auto filmAvailableFraction = 1.0 - std::exp(-secondsToSpark
                    * filmTemperatureFactor
                    / std::max(1.0e-4, config_.injection.vaporisationTimeConstantSeconds));
                const auto existingFuelInventory = injectionTarget.mixture().fuelMoles
                    + (config_.injection.mode == InjectionMode::port
                        ? injectionStates_[cylinderIndex].liquidFilmMoles * filmAvailableFraction
                            + trappedCylinderFuel
                        : 0.0);
                commandedFuelMoles = std::max(0.0,
                    requestedFuelMoles - existingFuelInventory);
            }
            const auto injectionResult = FuelInjectionModel::deliver(config_.injection,
                config_.fuelProperties, injectionStates_[cylinderIndex], injectionTarget,
                commandedFuelMoles, subDt);
            injectedFuelMolesThisCycle_[cylinderIndex] += injectionResult.meteredMoles;
            contribution.meteredFuelMassKg += injectionResult.meteredMoles * fuelMolarMassKg;
            if (!sparkCombustionAvailable) {
                cylinderMisfires_[cylinderIndex] = false;
                flameEvents_[cylinderIndex] = {};
                ignitionPending_[cylinderIndex] = false;
                ignitionDelayRemainingSeconds_[cylinderIndex] = 0.0;
            } else if (sparkCrossed) {
                endGasKnockStates_[cylinderIndex] = {};
                meteredFuelMolesLastCycle_[cylinderIndex] = injectedFuelMolesThisCycle_[cylinderIndex];
                const auto chamberFuelMoles = cylinderGas_[cylinderIndex].mixture().fuelMoles;
                deliveredFuelMolesLastCycle_[cylinderIndex] = chamberFuelMoles;
                fuelDeliveryRatio_[cylinderIndex] = requestedFuelMoles > 1.0e-15
                    ? std::clamp(chamberFuelMoles / requestedFuelMoles, 0.0, 1.0) : 0.0;
                const auto mixtureAfr = airFuelRatioForCell(cylinderGas_[cylinderIndex], config_.fuelProperties);
                actualAfrLastCycle_[cylinderIndex] = mixtureAfr;
                // Closed-loop lambda correction is based on the mixture that
                // actually reached the chamber, one cycle after injection.
                // It compensates port-film and runner transport losses without
                // fabricating fuel or altering the pressure-derived work.
                if (state_.rpm > 450.0 && mixtureAfr > 4.0 && mixtureAfr < 40.0) {
                    closedLoopFuelTrim_[cylinderIndex] = FuelInjectionModel::updateClosedLoopTrim(
                        config_.injection, state_.rpm, mixtureAfr,
                        std::clamp(ecuCommand.targetAirFuelRatio, 5.0, 30.0),
                        closedLoopFuelTrim_[cylinderIndex]);
                }
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
                auto& rng = decoupleSharedVolumes
                    ? cylinderRandomState_[cylinderIndex] : randomState_;
                rng ^= rng << 13U;
                rng ^= rng >> 17U;
                rng ^= rng << 5U;
                const auto randomUnit = static_cast<double>(rng) / static_cast<double>(0xffffffffU);
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
                    state_.load, config_.combustionCalibration.residualDilutionSensitivity };
                ignitionPending_[cylinderIndex] = !cylinderMisfires_[cylinderIndex];
                ignitionDelayRemainingSeconds_[cylinderIndex] = ignitionPending_[cylinderIndex]
                    ? FlamePhysicsModel::ignitionDelaySeconds(config_.combustionCalibration, flameConditions) : 0.0;
                flameEvents_[cylinderIndex] = {};
            }
            previousCylinderPhases_[cylinderIndex] = cyclePhase;
            const auto phaseTravel = forwardPhaseDegrees(previousPhase, cyclePhase);
            // Bank angle is spatial geometry, not cam timing. Valve events are
            // referenced to the cylinder's 720-degree thermodynamic phase.
            const auto intakeLift = valveTrain.intakeLiftMm;
            const auto exhaustLift = valveTrain.exhaustLiftMm;
            const auto exhaustCenterPhase = std::fmod(360.0 - cams.config->exhaustCenterlineDegrees
                - valveTrain.exhaustAdvanceDegrees + 720.0, 720.0);
            const auto exhaustOpenPhase = std::fmod(exhaustCenterPhase
                - cams.exhaustDuration() * 0.5 + 720.0, 720.0);
            const auto distanceToExhaustOpen = std::fmod(
                exhaustOpenPhase - previousPhase + 720.0, 720.0);
            const auto exhaustOpenedThisStep = phaseTravel > 1.0e-9
                && distanceToExhaustOpen <= phaseTravel;
            const auto combustionDt = exhaustOpenedThisStep
                ? subDt * std::clamp(distanceToExhaustOpen / phaseTravel, 0.0, 1.0)
                : subDt;
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
                state_.load, config_.combustionCalibration.residualDilutionSensitivity };
            if (ignitionPending_[cylinderIndex]) {
                ignitionDelayRemainingSeconds_[cylinderIndex] -= combustionDt;
                if (ignitionDelayRemainingSeconds_[cylinderIndex] <= 0.0) {
                    const auto burnableFuelMoles = std::min(cylinderGas_[cylinderIndex].mixture().fuelMoles,
                        cylinderGas_[cylinderIndex].mixture().oxygenMoles
                            / config_.fuelProperties.oxygenMolesPerFuelMole);
                    flamePhysics_.ignite(flameEvents_[cylinderIndex], config_.fuelProperties,
                                         flameConditions, burnableFuelMoles);
                    ignitionPending_[cylinderIndex] = false;
                }
            }
            const auto flameStep = flamePhysics_.advance(flameEvents_[cylinderIndex],
                config_.fuelProperties, flameConditions, combustionDt);
            const auto burnAdvance = flameStep.burnedFractionAdvance;
            const auto pulse = phaseTravel > 1.0e-6
                ? std::clamp(burnAdvance / phaseTravel * 90.0, 0.0, 3.5) : 0.0;
            instantaneousCombustionPulse_[cylinderIndex] = pulse * fuelDeliveryRatio_[cylinderIndex];
            if (burnAdvance > 0.0 && sparkCombustionAvailable && !cylinderMisfires_[cylinderIndex]) {
                const auto reaction = ConservativeGasSystem::reactFuelMoles(cylinderGas_[cylinderIndex],
                    flameEvents_[cylinderIndex].initialBurnableFuelMoles * burnAdvance,
                    flameStep.efficiency,
                    config_.fuelProperties.lowerHeatingValueMjPerKg * 1'000'000.0);
                contribution.releasedEnergyJoules += reaction.releasedEnergyJoules;
            }

            const auto endGas = EndGasKnockModel::advance(endGasKnockStates_[cylinderIndex], {
                cylinderGas_[cylinderIndex].pressureKpa() / 100.0,
                cylinderGas_[cylinderIndex].temperatureK(),
                flameConditions.equivalenceRatio, flameEvents_[cylinderIndex].burnedFraction,
                config_.octaneRating,
                sparkCombustionAvailable && flameEvents_[cylinderIndex].active
                    && !cylinderMisfires_[cylinderIndex] }, combustionDt);
            if (endGas.autoIgnited && endGas.autoIgnitedFuelFraction > 0.0) {
                const auto reaction = ConservativeGasSystem::reactFuelMoles(cylinderGas_[cylinderIndex],
                    flameEvents_[cylinderIndex].initialBurnableFuelMoles
                        * endGas.autoIgnitedFuelFraction,
                    std::max(0.72, flameStep.efficiency),
                    config_.fuelProperties.lowerHeatingValueMjPerKg * 1'000'000.0);
                contribution.releasedEnergyJoules += reaction.releasedEnergyJoules;
                flameEvents_[cylinderIndex].burnedFraction = std::clamp(
                    flameEvents_[cylinderIndex].burnedFraction + endGas.autoIgnitedFuelFraction,
                    0.0, 1.0);
                if (flameEvents_[cylinderIndex].burnedFraction >= 0.999)
                    flameEvents_[cylinderIndex].active = false;
            }
            contribution.endGasKnockLevel = std::max(contribution.endGasKnockLevel, endGas.level);
            if (exhaustOpenedThisStep) {
                // A premixed flame cannot propagate into the following
                // gas-exchange strokes. Only the part of this integration
                // interval preceding EVO was allowed to release heat above.
                flameEvents_[cylinderIndex].active = false;
                ignitionPending_[cylinderIndex] = false;
                ignitionDelayRemainingSeconds_[cylinderIndex] = 0.0;
            }

            auto chamberKpa = cylinderGas_[cylinderIndex].pressureKpa();
            chamberPressureBar_[cylinderIndex] = chamberKpa / 100.0;
            const auto intakeArea = valveAreaMm2(cylinder.boreMm, intakeLift,
                cylinder.intakeValveCount, cylinder.intakeValveDiameterMm, 0.40, 0.50);
            const auto exhaustArea = valveAreaMm2(cylinder.boreMm, exhaustLift,
                cylinder.exhaustValveCount, cylinder.exhaustValveDiameterMm, 0.34, 0.41);
            const auto intakePathIndex = intakePathIndexFor(config_, cylinder);
            const auto& cylinderIntake = intakeGeometryAt(config_, intakePathIndex);
            const auto runnerDiameterMm = cylinder.intakeRunnerDiameterMm > 0.0
                ? cylinder.intakeRunnerDiameterMm : cylinderIntake.runnerDiameterMm;
            const auto runnerAreaM2 = std::numbers::pi * std::pow(runnerDiameterMm * 0.0005, 2.0);
            const auto pistonAreaM2 = std::numbers::pi * std::pow(cylinder.boreMm * 0.0005, 2.0);
            exhaustValveAreaM2[cylinderIndex] = exhaustArea * 1.0e-6;
            exhaustValveDischargeCoefficient[cylinderIndex] =
                valveTrain.exhaustDischargeCoefficient;
            intakeValveAreaM2[cylinderIndex] = intakeArea * 1.0e-6;
            intakeValveDischargeCoefficient[cylinderIndex] =
                valveTrain.intakeDischargeCoefficient;
            intakeRunnerAreaM2ForSplit[cylinderIndex] = runnerAreaM2;
            pistonAreaM2ForSplit[cylinderIndex] = pistonAreaM2;
            chamberVolumeLitresForWork[cylinderIndex] = chamberVolume;
            cycleBoundaryForWork[cylinderIndex] = cycleBoundaryCrossed;
            const auto plenumPressureKpa = decoupleSharedVolumes
                ? parallelState->frozenPlenum[intakePathIndex].pressureKpa()
                : intakePlenumGas_[intakePathIndex].pressureKpa();
            runnerAcousticResults_[cylinderIndex] = HelmholtzRunnerModel::advance(config_.runnerAcoustics,
                runnerAcousticStates_[cylinderIndex], cylinder, cylinderIntake,
                intakeRunnerGas_[cylinderIndex].temperatureK(),
                plenumPressureKpa, intakeRunnerGas_[cylinderIndex].pressureKpa(), subDt);

            // ----------------------------------------------------------------
            // Gas flow — full physics variant with dynamic pressure and jet
            // momentum.  Direction vectors follow the physical path from
            // atmosphere through the intake to the cylinder and out through
            // the exhaust to the collector.
            // ----------------------------------------------------------------

            // (1) Manifold → intake runner. Large engines evaluate against the
            // frozen plenum; small engines update the real plenum serially.
            GasCell plenumWork;
            auto* plenumFlowCell = &intakePlenumGas_[intakePathIndex];
            GasCell intakeRunnerBefore;
            if (decoupleSharedVolumes) {
                plenumWork = parallelState->frozenPlenum[intakePathIndex];
                plenumFlowCell = &plenumWork;
                intakeRunnerBefore = intakeRunnerGas_[cylinderIndex];
            }
            (void)ConservativeGasSystem::flow({ plenumFlowCell, &intakeRunnerGas_[cylinderIndex],
                runnerAreaM2, 0.78 * runnerAcousticResults_[cylinderIndex].flowAdmittance, subDt,
                /*dirX=*/0.0, /*dirY=*/1.0,   // downward into runner
                /*csArea0=*/0.0, /*csArea1=*/runnerAreaM2 });
            if (decoupleSharedVolumes) {
                auto& branch = contribution.plenumFlow;
                branch.counterpart = &intakeRunnerGas_[cylinderIndex];
                branch.sharedDelta = ConservativeGasSystem::inventoryDelta(
                    plenumWork, parallelState->frozenPlenum[intakePathIndex]);
                branch.counterpartDelta = ConservativeGasSystem::inventoryDelta(
                    intakeRunnerGas_[cylinderIndex], intakeRunnerBefore);
                branch.sharedTotalEnergyDeltaJ = totalCellEnergy(plenumWork)
                    - totalCellEnergy(parallelState->frozenPlenum[intakePathIndex]);
                contribution.intakePath = intakePathIndex;
            }

            // (2) Intake runner → cylinder (intake valve)
            const FlowParameters intakeValveFlow {
                &intakeRunnerGas_[cylinderIndex], &cylinderGas_[cylinderIndex],
                intakeArea * 1.0e-6, valveTrain.intakeDischargeCoefficient,
                subDt * 0.5,
                /*dirX=*/0.0, /*dirY=*/1.0,   // downward through valve
                /*csArea0=*/runnerAreaM2, /*csArea1=*/pistonAreaM2 };

            // (3) Cylinder → exhaust runner (exhaust valve)
            const auto intakeTransfer = ConservativeGasSystem::flow(intakeValveFlow);

            // (4) Exhaust runner → collector, with the same thresholded Jacobi
            // treatment as the intake side.
            // Exhaust transfer is solved simultaneously for all cylinders by
            // the conservative 1D network after this private-state pass.

            const auto intakeRunnerLengthM = (cylinder.intakeRunnerLengthMm > 0.0
                ? cylinder.intakeRunnerLengthMm : cylinderIntake.runnerLengthMm) * 0.001;
            intakeRunnerGas_[cylinderIndex].applyFlowResistance(
                intakeRunnerLengthM, runnerDiameterMm * 0.001, 1.5e-6, 0.0, subDt);
            const auto wallHeatTransfer = std::clamp((cylinderWallTemperatureC_[cylinderIndex] + 273.15
                - cylinderGas_[cylinderIndex].temperatureK())
                * config_.combustionCalibration.wallHeatTransferCoefficientWPerK * subDt, -120.0, 35.0);
            cylinderGas_[cylinderIndex].addHeatJoules(wallHeatTransfer);
            if (cylinder.blowByCoefficient > 0.0)
                (void)ConservativeGasSystem::flowFromBoundary(cylinderGas_[cylinderIndex],
                    config_.ambientPressureKpa, config_.ambientTemperatureC + 273.15,
                    std::numbers::pi * std::pow(cylinder.boreMm * 0.0005, 2.0) * cylinder.blowByCoefficient,
                    0.65, subDt);
            chamberKpa = cylinderGas_[cylinderIndex].pressureKpa();
            chamberPressureBar_[cylinderIndex] = chamberKpa / 100.0;
            const auto intakeClosePhase = std::fmod(360.0 + cams.config->intakeCenterlineDegrees
                - valveTrain.intakeAdvanceDegrees
                + cams.intakeDuration() * 0.5 + 720.0, 720.0);
            intakeCloseForTrappedAir[cylinderIndex] = crossedPhase(
                previousPhase, cyclePhase, intakeClosePhase);
            intakeFlowMgThisCycle_[cylinderIndex] += intakeTransfer.transferredMassKg * 1.0e6;
            const auto runnerPulse = std::sin(cyclePhase * std::numbers::pi / 180.0);
            intakeRunnerPressureKpa_[cylinderIndex] = intakeRunnerGas_[cylinderIndex].pressureKpa();
            (void)runnerPulse;
            cylinderWallTemperatureC_[cylinderIndex] = smooth(cylinderWallTemperatureC_[cylinderIndex],
                config_.ambientTemperatureC + combustion.heatOutput * 175.0 + pulse * 95.0, subDt, 0.22);
            contribution.intakeRunnerPressure += intakeRunnerPressureKpa_[cylinderIndex];
            const auto breathingQuality = std::clamp(intakeFlowMgPerCycle_[cylinderIndex]
                / std::max(1.0, combustion.airMassMgPerCycle / static_cast<double>(config_.cylinders.size())),
                0.45, 1.35);
            contribution.combustionPulse += pulse * std::clamp(1.0 + cylinder.efficiencyOffset, 0.8, 1.2)
                * breathingQuality * fuelDeliveryRatio_[cylinderIndex];
            // One third of the connecting rod mass is a standard equivalent
            // reciprocating-mass approximation; the remainder contributes to
            // crank rotational inertia represented by EngineConfig.
            const auto massKg = (cylinder.pistonMassGrams
                + cylinder.connectingRodMassGrams / 3.0) * 0.001;
            const auto inertiaForce = massKg * kinematics.pistonAccelerationMps2;
            const auto leverArm = kinematics.displacementDerivativeMPerRadian;
            contribution.reciprocatingTorque += inertiaForce * leverArm;
            // pistonAreaM2 is already defined above for the gas flow calls (same scope).
            const auto gasForceN = (chamberKpa - config_.ambientPressureKpa) * 1'000.0 * pistonAreaM2;
            chamberPressureBeforeNetworkKpa[cylinderIndex] = chamberKpa;
            gasTorqueLeverArmM[cylinderIndex] = leverArm;
            contribution.gasIndicatedTorque += gasForceN * leverArm;
            // The side thrust is governed by the acute angle between the rod
            // and the cylinder axis.  Treating an aligned rod as 90 degrees
            // multiplies skirt/ring friction by tan(77 deg) and consumes most
            // of the indicated work.  MechanicalKinematics owns the geometry,
            // so use its vector-derived obliquity directly.
            const auto rodObliquityRadians = std::clamp(
                kinematics.connectingRodObliquityDegrees * std::numbers::pi / 180.0,
                0.0, 1.35);
            const auto rodSideRatio = std::abs(std::tan(rodObliquityRadians));
            const auto cylinderWallForceN = std::abs(gasForceN - inertiaForce) * rodSideRatio;
            const auto pistonVelocityMps = kinematics.pistonVelocityMps;
            const auto pistonFrictionForceN = stribeckFrictionForce(cylinder, pistonVelocityMps,
                                                                    cylinderWallForceN);
            contribution.pistonFrictionPowerW += pistonFrictionForceN * std::abs(pistonVelocityMps);
            contribution.pistonBreakawayTorqueNm += pistonFrictionForceN * std::abs(leverArm);
            contribution.peakPistonAcceleration = std::max(
                contribution.peakPistonAcceleration, std::abs(kinematics.pistonAccelerationMps2));
            contribution.pistonSpeed += 2.0 * cylinder.strokeMm * 0.001 * state_.rpm / 60.0;
            if (!decoupleSharedVolumes) accumulateContribution(contribution);
        };
        // The per-cylinder body writes only cylinder-private state (per-index
        // arrays plus its own scratch entry), so it is safe to run across threads.
        // A stopped engine stays inline so an idle application neither constructs
        // nor repeatedly wakes the pool. The Jacobi/RNG choice above remains tied
        // only to cylinder count, not to whether this particular sub-step threaded.
        auto ranParallel = false;
        if (decoupleSharedVolumes && !stationary) {
            auto& executor = SubstepParallelExecutor::shared();
            if (executor.participantCount() > 1) {
                executor.parallelFor(config_.cylinders.size(), processCylinder,
                    subStep + 1 < subStepCount);
                ranParallel = true;
            }
        }
        if (!ranParallel) {
            for (std::size_t cylinderIndex = 0; cylinderIndex < config_.cylinders.size(); ++cylinderIndex)
                processCylinder(cylinderIndex);
        }

        double outletMassKg = 0.0;
        {
            auto exhaustNetworkCompleted = false;
            std::array<gasdynamics::CylinderValveBoundary, 32> cylinderBoundaries {};
            for (std::size_t index = 0; index < config_.cylinders.size(); ++index) {
                cylinderBoundaries[index] = {
                    config_.cylinders[index].id,
                    networkStateForGasCell(cylinderGas_[index]),
                    cylinderGas_[index].volumeM3(),
                    exhaustValveAreaM2[index],
                    exhaustValveDischargeCoefficient[index],
                };
            }
            auto outletOpeningScale = 1.0;
            if (config_.forcedInduction.enabled
                && config_.forcedInduction.type == ForcedInductionType::turbocharger) {
                const auto turbineAreaM2 = (config_.forcedInduction.turbineFlowAreaMm2
                    + state_.wastegateOpening
                        * config_.forcedInduction.wastegateFlowAreaMm2) * 1.0e-6;
                auto configuredOutletConductanceM2 = 0.0;
                for (const auto& outlet : physicalExhaustNetwork.layout().outlets())
                    configuredOutletConductanceM2 +=
                        outlet.openingAreaM2 * outlet.dischargeCoefficient;
                outletOpeningScale = std::clamp(turbineAreaM2
                    / std::max(1.0e-12, configuredOutletConductanceM2), 0.0, 1.0);
            }
            const gasdynamics::ExhaustAmbientBoundary ambient {
                physicalExhaustAmbientState_, outletOpeningScale
            };
            const auto networkAdvance = physicalExhaustNetwork.advance(
                subDt,
                std::span<const gasdynamics::CylinderValveBoundary>(
                    cylinderBoundaries.data(), config_.cylinders.size()),
                ambient);
            exhaustNetworkCompleted = networkAdvance.completed;
            if (!networkAdvance.completed)
                state_.solverResolutionLimited = true;

            collectorPressureKpa = config_.ambientPressureKpa;
            const auto cylinderExchanges = physicalExhaustNetwork.cylinderExchanges();
            for (std::size_t exchangeIndex = 0;
                 exchangeIndex < cylinderExchanges.size(); ++exchangeIndex) {
                const auto& exchange = cylinderExchanges[exchangeIndex];
                const auto index = exhaustNetworkCylinderIndex_[exchangeIndex];
                const auto applied = cylinderGas_[index].tryApplyInventoryDelta(
                    cylinderDeltaForExchange(exchange, cylinderGas_[index]));
                if (!applied) state_.solverResolutionLimited = true;
                exhaustFlowMgThisCycle_[index] += exchange.totalMassKg() * 1.0e6;
                exhaustRunnerPressureKpa_[index] = exchange.networkPressurePa * 0.001;
                exhaustRunnerVelocityMps_[index] = exchange.networkVelocityMps;
                exhaustMassFlowKgPerSecond[index] = exchange.totalMassKg() / subDt;
                exhaustPortDensityKgPerM3[index] = exchange.networkDensityKgPerM3;
                exhaustPortSpeedOfSoundMps[index] = exchange.networkSpeedOfSoundMps;
                thermoacousticBoundaryValid[index] = static_cast<std::uint8_t>(
                    exhaustNetworkCompleted && applied
                    && std::isfinite(exhaustMassFlowKgPerSecond[index])
                    && exhaustPortDensityKgPerM3[index] > 0.0
                    && exhaustPortSpeedOfSoundMps[index] > 0.0);
                collectorPressureKpa = std::max(
                    collectorPressureKpa, exhaustRunnerPressureKpa_[index]);

                // Symmetric operator split around the exhaust-network solve:
                // the second intake half-step sees the pressure returned by
                // the exhaust valve and removes first-order overlap bias.
                const FlowParameters secondIntakeHalfStep {
                    &intakeRunnerGas_[index], &cylinderGas_[index],
                    intakeValveAreaM2[index], intakeValveDischargeCoefficient[index],
                    subDt * 0.5,
                    0.0, 1.0,
                    intakeRunnerAreaM2ForSplit[index], pistonAreaM2ForSplit[index],
                };
                const auto secondIntakeTransfer = ConservativeGasSystem::flow(
                    secondIntakeHalfStep);
                intakeFlowMgThisCycle_[index] +=
                    secondIntakeTransfer.transferredMassKg * 1.0e6;
                if (intakeCloseForTrappedAir[index]) {
                    trappedAirMassMgLastCycle_[index] = cylinderGas_[index]
                        .mixture().oxygenMoles / 0.21 * GasCell::airMolarMassKg * 1.0e6;
                }

                const auto finalChamberPressureKpa = cylinderGas_[index].pressureKpa();
                chamberPressureBar_[index] = finalChamberPressureKpa / 100.0;
                IndicatedWorkModel::advance(indicatedWorkStates_[index],
                    finalChamberPressureKpa, chamberVolumeLitresForWork[index],
                    config_.ambientPressureKpa, cycleBoundaryForWork[index]);
                const auto torqueCorrection = (finalChamberPressureKpa
                        - chamberPressureBeforeNetworkKpa[index]) * 1'000.0
                    * pistonAreaM2ForSplit[index] * gasTorqueLeverArmM[index];
                if (decoupleSharedVolumes) {
                    parallelState->scratch[index].gasIndicatedTorque += torqueCorrection;
                    parallelState->scratch[index].exhaustRunnerPressure +=
                        exhaustRunnerPressureKpa_[index];
                    parallelState->scratch[index].exhaustTemperatureK +=
                        exchange.networkTemperatureK;
                } else {
                    gasIndicatedTorque += torqueCorrection;
                    exhaustRunnerPressureSum += exhaustRunnerPressureKpa_[index];
                    exhaustTemperatureSumK += exchange.networkTemperatureK;
                }
            }
            for (const auto& outlet : physicalExhaustNetwork.outletSamples()) {
                auto transferredMassKg = 0.0;
                for (const auto speciesMassKg : outlet.speciesMassKg)
                    transferredMassKg += speciesMassKg;
                outletMassKg += std::max(0.0, transferredMassKg);
            }
        }
        // Fixed-order reduction of the per-cylinder scratch. Small engines retain
        // their original serial sum; large-engine output is independent of worker
        // scheduling because the same cylinder order is always used here.
        if (decoupleSharedVolumes) {
            for (std::size_t index = 0; index < config_.cylinders.size(); ++index)
                accumulateContribution(parallelState->scratch[index]);
            // Gather and commit each shared path in cylinder order. This order is
            // independent of worker scheduling and therefore fixes every floating-
            // point reduction, including the N-way conservation correction.
            std::array<JacobiGasFlowBranch, 32> pathBranches {};
            for (std::size_t path = 0; path < intakePlenumCount_; ++path) {
                auto branchCount = std::size_t { 0 };
                for (std::size_t index = 0; index < config_.cylinders.size(); ++index)
                    if (parallelState->scratch[index].intakePath == path)
                        pathBranches[branchCount++] = parallelState->scratch[index].plenumFlow;
                ConservativeGasSystem::commitJacobiFlows(intakePlenumGas_[path],
                    std::span<const JacobiGasFlowBranch>(pathBranches.data(), branchCount));
            }
        }

        const auto meanPistonSpeed = pistonSpeedSum / static_cast<double>(config_.cylinders.size());
        const auto displacementM3 = engineDisplacementLitres(config_) * 0.001;
        // Lumped non-piston friction MEP (bearings + valvetrain + accessories),
        // a Chen-Flynn-style polynomial in displacement and speed. Piston-skirt
        // and ring friction are NOT in here: they are resolved separately below
        // via stribeckFrictionForce. The two are complementary; their sum tracks
        // the gasoline FMEP band (~0.6 bar idle -> ~2.1 bar redline), verified by
        // the friction-MEP sweep in CoreTests.
        const auto bearingFriction = state_.rpm > 1.0
            ? displacement * (1.4 + config_.frictionCoefficient * 10.0 + state_.rpm * 0.00055
                              + state_.rpm * state_.rpm * 0.000000035) : 0.0;
        const auto pistonFriction = state_.angularVelocityRadPerSecond > 1.0
            ? pistonFrictionPowerW / state_.angularVelocityRadPerSecond
            : pistonBreakawayTorqueNm;
        const auto seizureTorque = state_.damage >= 1.0 ? displacement * 250.0
            + state_.angularVelocityRadPerSecond * 0.22 : 0.0;
        double crankshaftFriction = 0.0;
        for (const auto& crankshaft : config_.crankshafts)
            crankshaftFriction += crankshaft.frictionTorqueNm * std::abs(crankshaft.rotationRatio);
        const auto superchargerTorque = config_.forcedInduction.enabled
            && config_.forcedInduction.type == ForcedInductionType::supercharger
            && state_.angularVelocityRadPerSecond > 1.0
            ? state_.compressorPowerKw * 1'000.0 / state_.angularVelocityRadPerSecond : 0.0;
        const auto mechanicalFriction = bearingFriction + pistonFriction + crankshaftFriction + superchargerTorque;
        // Gas-pressure torque already contains the complete intake/exhaust
        // pumping loop. Adding a MAP-derived pumping loss here counted the
        // same work twice, especially at closed throttle.
        const auto frictionTorque = mechanicalFriction + seizureTorque;
        constexpr double maximumBrakeMeanEffectivePressurePa = 2'500'000.0;
        const auto loadTorque = state_.rpm > 40.0 && !safeControls.starterEngaged
            ? (safeControls.dynamometerTorqueNm > 0.0
                ? safeControls.dynamometerTorqueNm
                : safeControls.load * maximumBrakeMeanEffectivePressurePa * displacementM3
                    / (4.0 * std::numbers::pi))
            : 0.0;
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
        const auto netTorque = brakeTorque + starterTorque - loadTorque
            + safeControls.externalTorqueNm + reciprocatingTorque;

        auto omega = state_.angularVelocityRadPerSecond;
        omega = std::max(0.0, omega + netTorque / effectiveRotatingInertiaKgM2(config_) * subDt);
        state_.angularVelocityRadPerSecond = omega;
        state_.rpm = omega * 60.0 / (2.0 * std::numbers::pi);
        if (state_.rpm < 0.5 && !cranking && indicatedTorque <= 0.0) {
            state_.rpm = 0.0;
            state_.angularVelocityRadPerSecond = 0.0;
        }
        const auto subTravelled = (subPreviousRpm + state_.rpm) * 0.5 * 6.0 * subDt;
        maximumIntegratedCrankStep = std::max(maximumIntegratedCrankStep, std::abs(subTravelled));
        accumulateCycleTelemetry(subPreviousAngle, subTravelled, subDt,
                                 indicatedTorque, brakeTorque);
        state_.crankAngleDegrees = std::fmod(subPreviousAngle + subTravelled, eventGenerator_.cycleDegrees());
        state_.simulationTimeSeconds += subDt;

        state_.indicatedTorqueNm = indicatedTorque;
        state_.meanWorkTorqueNm = modeledIndicatedTorque;
        state_.cylinderPressureTorqueNm = gasIndicatedTorque;
        state_.cylinderPressureTorqueBlend = pressureBlend;
        state_.frictionTorqueNm = frictionTorque;
        state_.frictionMeanEffectivePressureBar = displacementM3 > 0.0
            ? frictionTorque * 4.0 * std::numbers::pi / displacementM3 / 100'000.0 : 0.0;
        state_.reciprocatingTorqueNm = reciprocatingTorque;
        state_.meanPistonSpeedMps = meanPistonSpeed;
        state_.loadTorqueNm = loadTorque;
        state_.starterTorqueNm = starterTorque;
        state_.netTorqueNm = netTorque;
        state_.torqueNm = brakeTorque;
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
        const auto meteredFuelMoles = std::accumulate(meteredFuelMolesLastCycle_.begin(),
            meteredFuelMolesLastCycle_.begin()
                + static_cast<std::ptrdiff_t>(config_.cylinders.size()), 0.0);
        state_.injectedFuelMgPerCycle = ecuCommand.fuelEnabled
            ? meteredFuelMoles * config_.fuelProperties.molarMassGramsPerMole * 1'000.0 : 0.0;
        const auto deliveredFuelMoles = std::accumulate(deliveredFuelMolesLastCycle_.begin(),
            deliveredFuelMolesLastCycle_.begin()
                + static_cast<std::ptrdiff_t>(config_.cylinders.size()), 0.0);
        state_.deliveredFuelMgPerCycle = ecuCommand.fuelEnabled
            ? deliveredFuelMoles * config_.fuelProperties.molarMassGramsPerMole * 1'000.0 : 0.0;
        const auto actualAfrSum = std::accumulate(actualAfrLastCycle_.begin(),
            actualAfrLastCycle_.begin() + static_cast<std::ptrdiff_t>(config_.cylinders.size()), 0.0);
        state_.airFuelRatio = actualAfrSum / static_cast<double>(config_.cylinders.size());
        const auto meteredFuelFlowGramsPerSecond = meteredFuelMassKg / subDt * 1'000.0;
        state_.fuelFlowGramsPerSecond = smooth(state_.fuelFlowGramsPerSecond,
            meteredFuelFlowGramsPerSecond, subDt, 22.0);
        state_.fuelConsumedGrams += meteredFuelMassKg * 1'000.0;
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
        state_.manifoldGasMassGrams = 0.0;
        state_.cylinderGasMassGrams = 0.0;
        state_.gasInternalEnergyJoules = 0.0;
        for (std::size_t pathIndex = 0; pathIndex < intakePlenumCount_; ++pathIndex) {
            state_.manifoldGasMassGrams += intakePlenumGas_[pathIndex].massKg() * 1'000.0;
            state_.gasInternalEnergyJoules += intakePlenumGas_[pathIndex].internalEnergyJoules();
        }
        state_.gasInternalEnergyJoules += physicalExhaustNetwork.inventory().totalEnergyJ;
        for (std::size_t index = 0; index < config_.cylinders.size(); ++index) {
            state_.cylinderGasMassGrams += cylinderGas_[index].massKg() * 1'000.0;
            state_.gasInternalEnergyJoules += cylinderGas_[index].internalEnergyJoules()
                + intakeRunnerGas_[index].internalEnergyJoules();
        }
        state_.powerKw = state_.torqueNm * state_.angularVelocityRadPerSecond / 1'000.0;
        state_.brakeSpecificFuelConsumptionGPerKwh = state_.cycleAveragedPowerKw > 1.0
            ? state_.fuelFlowGramsPerSecond * 3'600.0 / state_.cycleAveragedPowerKw : 0.0;
        state_.mechanicalEfficiency = indicatedTorque > 1.0 ? std::clamp(brakeTorque / indicatedTorque, 0.0, 1.0) : 0.0;
        state_.oilPressureKpa = state_.rpm > 0.0
            ? std::clamp(80.0 + state_.rpm * 0.055 - std::max(0.0, state_.oilTemperatureC - 105.0) * 2.0, 0.0, 520.0) : 0.0;
        state_.peakPistonAccelerationG = peakPistonAcceleration / 9.80665;

        const auto thermostatOpening = std::clamp((state_.coolantTemperatureC - 82.0) / 12.0, 0.08, 1.0);
        const auto coolingAirflow = config_.coolingEfficiency * (0.18 + thermostatOpening
            * (0.42 + state_.rpm / std::max(1.0, config_.redlineRpm) * 0.8));
        state_.resolvedHeatReleaseKw = releasedEnergyJoules / subDt / 1'000.0;
        const auto coolantHeatKw = state_.resolvedHeatReleaseKw * config_.thermal.coolantHeatShare;
        const auto oilHeatKw = state_.resolvedHeatReleaseKw * config_.thermal.oilHeatShare
            + mechanicalFriction * state_.angularVelocityRadPerSecond / 1'000.0 * 0.18;
        const auto coolantRejectedKw = std::max(0.0, state_.coolantTemperatureC - config_.ambientTemperatureC)
            * config_.thermal.coolingPowerKwPerC * coolingAirflow;
        const auto oilRejectedKw = std::max(0.0, state_.oilTemperatureC - config_.ambientTemperatureC)
            * config_.thermal.oilCoolingPowerKwPerC * std::max(0.35, coolingAirflow * 0.72);
        const auto exhaustTarget = exhaustTemperatureSumK
            / static_cast<double>(config_.cylinders.size()) - 273.15;
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
            const auto& valveTrain = valveTrainResults_[index];
            const auto intakeLift = valveTrain.intakeLiftMm;
            const auto exhaustLift = valveTrain.exhaustLiftMm;
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
                intakeRunnerGas_[index].bulkVelocityMps(),
                exhaustRunnerVelocityMps_[index],
                fuelDeliveryRatio_[index],
                flameEvents_[index].flameSpeedMps, flameEvents_[index].burnedFraction,
                flameEvents_[index].efficiency,
                endGasKnockStates_[index].filteredLevel,
                active, combustion.combustionEnabled && cylinderMisfires_[index] };
            const auto kinematics = evaluateCylinderKinematics(config_, kinematicsReference_, index,
                state_.crankAngleDegrees, state_.angularVelocityRadPerSecond,
                state_.netTorqueNm / effectiveRotatingInertiaKgM2(config_));
            auto& cylinderState = state_.cylinderStates[index];
            cylinderState.airFuelRatio = actualAfrLastCycle_[index];
            cylinderState.requestedFuelMgPerCycle = requestedFuelMolesThisCycle_[index]
                * config_.fuelProperties.molarMassGramsPerMole * 1'000.0;
            cylinderState.deliveredFuelMgPerCycle = deliveredFuelMolesLastCycle_[index]
                * config_.fuelProperties.molarMassGramsPerMole * 1'000.0;
            cylinderState.closedLoopFuelTrim = closedLoopFuelTrim_[index];
            cylinderState.pistonTravelMm = kinematics.pistonTravelMm;
            cylinderState.pistonPositionMm = kinematics.pistonPositionMm;
            cylinderState.pistonVelocityMps = kinematics.pistonVelocityMps;
            cylinderState.pistonAccelerationMps2 = kinematics.pistonAccelerationMps2;
            cylinderState.connectingRodAngleDegrees = kinematics.connectingRodAngleDegrees;
            cylinderState.crankPinXMm = kinematics.crankPinXMm;
            cylinderState.crankPinYMm = kinematics.crankPinYMm;
            cylinderState.wristPinXMm = kinematics.wristPinXMm;
            cylinderState.wristPinYMm = kinematics.wristPinYMm;
            cylinderState.mechanicalReactionTorqueNm = kinematics.displacementDerivativeMPerRadian
                * (gas.pressureKpa() - config_.ambientPressureKpa) * 1'000.0
                * std::numbers::pi * std::pow(cylinder.boreMm * 0.0005, 2.0);
            cylinderState.crankshaftId = kinematics.crankshaftId;
            cylinderState.crankJournalId = kinematics.crankJournalId;
            cylinderState.indicatedWorkJoulesPerCycle = indicatedWorkStates_[index].completedCycleJoules;
            const auto totalMoles = gas.totalMoles();
            cylinderState.residualGasFraction = totalMoles > 1.0e-15
                ? gas.mixture().burnedMoles / totalMoles : 0.0;
            cylinderState.intakeValveAdvanceDegrees = valveTrain.intakeAdvanceDegrees;
            cylinderState.exhaustValveAdvanceDegrees = valveTrain.exhaustAdvanceDegrees;
            cylinderState.valveLiftMultiplier = valveTrain.liftMultiplier;
            cylinderState.intakeResonancePressureKpa = runnerAcousticResults_[index].pressureAmplitudeKpa;
            cylinderState.intakeResonanceFrequencyHz = runnerAcousticResults_[index].resonanceFrequencyHz;
        }
        if (pressureSamples_) {
            CylinderPressureSample pressureSample;
            pressureSample.timeSeconds = state_.simulationTimeSeconds;
            pressureSample.cylinderCount = config_.cylinders.size();
            for (std::size_t index = 0; index < config_.cylinders.size(); ++index) {
                pressureSample.pressureBar[index] = static_cast<float>(chamberPressureBar_[index]);
                pressureSample.exhaustRunnerPressureKpa[index] = static_cast<float>(exhaustRunnerPressureKpa_[index]);
                pressureSample.exhaustMassFlowKgPerSecond[index] =
                    static_cast<float>(exhaustMassFlowKgPerSecond[index]);
                pressureSample.exhaustPortDensityKgPerM3[index] =
                    static_cast<float>(exhaustPortDensityKgPerM3[index]);
                pressureSample.exhaustPortSpeedOfSoundMps[index] =
                    static_cast<float>(exhaustPortSpeedOfSoundMps[index]);
                pressureSample.exhaustValveConductanceAreaM2[index] = static_cast<float>(
                    exhaustValveAreaM2[index] * exhaustValveDischargeCoefficient[index]);
                pressureSample.exhaustFlowMgPerCycle[index] = static_cast<float>(exhaustFlowMgPerCycle_[index]);
                const auto cams = activeCamshaft(config_, config_.cylinders[index],
                                                  state_.rpm, state_.throttle);
                const auto maximumExhaustLiftMm = std::max(0.1, cams.exhaustLift());
                pressureSample.exhaustValveOpening[index] = static_cast<float>(std::clamp(
                    valveTrainResults_[index].exhaustLiftMm / maximumExhaustLiftMm, 0.0, 1.0));
                pressureSample.exhaustPathIndex[index] = static_cast<std::uint8_t>(
                    exhaustPathIndexFor(config_, config_.cylinders[index]));
                pressureSample.thermoacousticBoundaryValid[index] =
                    thermoacousticBoundaryValid[index];
            }
            if (pressureSamples_->tryPush(pressureSample))
                ++frame.cylinderPressureSampleCount;
            else
                ++frame.droppedCylinderPressureSampleCount;
        }

        // Combustion decisions above resolve the crank interval ending at
        // `subPreviousAngle`. Generate its event now with the same ECU command
        // and actual per-cylinder misfire state. This one-substep-delayed
        // publication is bounded by the mechanical solver cadence and avoids
        // reconstructing a whole outer frame from its final limiter state.
        const auto eventTravelled = forwardPhaseDegrees(eventEvaluationAngleDegrees_,
                                                         subPreviousAngle);
        const auto eventDt = std::max(0.0, subStepStartTime - eventEvaluationTimeSeconds_);
        if (combustion.combustionEnabled && ecuCommand.fuelEnabled && ecuCommand.sparkEnabled
                && eventTravelled > 0.0) {
            const auto firstEvent = frame.firingEventCount;
            auto available = std::span<FiringEvent>(frame.firingEvents).subspan(firstEvent);
            const auto generated = eventGenerator_.generate(config_, state_, ecuCommand, combustion,
                eventEvaluationTimeSeconds_, eventEvaluationAngleDegrees_, eventTravelled,
                eventDt, available);
            frame.firingEventCount += generated;
            frame.droppedFiringEventCount += eventGenerator_.droppedEventCountLastGenerate();
            for (std::size_t index = firstEvent; index < frame.firingEventCount; ++index)
                exhaust_.process(frame.firingEvents[index]);
        }
        eventEvaluationAngleDegrees_ = subPreviousAngle;
        eventEvaluationTimeSeconds_ = subStepStartTime;
    }

    state_.crankDegreesPerSolverStep = maximumIntegratedCrankStep;
    state_.solverResolutionLimited = state_.solverResolutionLimited
        || maximumIntegratedCrankStep > config_.solver.maximumCrankDegreesPerStep + 1.0e-9;
    frame.state = state_;
    return frame;
}

void EngineSimulator::accumulateCycleTelemetry(double previousAngleDegrees,
                                               double travelledDegrees,
                                               double dtSeconds,
                                               double indicatedTorqueNm,
                                               double brakeTorqueNm) noexcept {
    if (!(travelledDegrees > 0.0) || !(dtSeconds > 0.0)) return;
    const auto cycleDegrees = eventGenerator_.cycleDegrees();
    if (!(cycleDegrees > 0.0)) return;
    const auto cycleRadians = cycleDegrees * std::numbers::pi / 180.0;
    auto angle = std::fmod(previousAngleDegrees + cycleDegrees, cycleDegrees);
    auto remainingDegrees = travelledDegrees;
    int guard = 0;
    while (remainingDegrees > 1.0e-12 && guard++ < 16) {
        const auto degreesToBoundary = cycleDegrees - angle;
        const auto segmentDegrees = std::min(remainingDegrees, degreesToBoundary);
        const auto segmentFraction = segmentDegrees / travelledDegrees;
        if (cycleTelemetryStarted_) {
            const auto segmentRadians = segmentDegrees * std::numbers::pi / 180.0;
            indicatedWorkThisCycleJoules_ += indicatedTorqueNm * segmentRadians;
            brakeWorkThisCycleJoules_ += brakeTorqueNm * segmentRadians;
            cycleElapsedSeconds_ += dtSeconds * segmentFraction;
        }
        remainingDegrees -= segmentDegrees;
        const auto reachedBoundary = segmentDegrees >= degreesToBoundary - 1.0e-12;
        if (!reachedBoundary) {
            angle += segmentDegrees;
            continue;
        }

        if (cycleTelemetryStarted_ && cycleElapsedSeconds_ > 1.0e-12) {
            state_.indicatedWorkJoulesPerCycle = indicatedWorkThisCycleJoules_;
            const auto displacementM3 = engineDisplacementLitres(config_) * 0.001;
            state_.indicatedMeanEffectivePressureBar = displacementM3 > 0.0
                ? indicatedWorkThisCycleJoules_ / displacementM3 / 100'000.0 : 0.0;
            state_.indicatedPowerKw = indicatedWorkThisCycleJoules_
                / cycleElapsedSeconds_ / 1'000.0;
            state_.pdvTorqueNm = indicatedWorkThisCycleJoules_ / cycleRadians;
            state_.cycleAveragedTorqueNm = brakeWorkThisCycleJoules_ / cycleRadians;
            state_.cycleAveragedPowerKw = brakeWorkThisCycleJoules_
                / cycleElapsedSeconds_ / 1'000.0;
        }
        cycleTelemetryStarted_ = true;
        indicatedWorkThisCycleJoules_ = 0.0;
        brakeWorkThisCycleJoules_ = 0.0;
        cycleElapsedSeconds_ = 0.0;
        angle = 0.0;
    }
}

void EngineSimulator::reset() noexcept {
    ecu_.reset();
    eventGenerator_.reset();
    if (pressureSamples_) {
        CylinderPressureSample discarded;
        while (pressureSamples_->tryPop(discarded)) {}
    }
    state_ = {};
    state_.coolantTemperatureC = config_.ambientTemperatureC;
    state_.oilTemperatureC = config_.ambientTemperatureC;
    state_.exhaustTemperatureC = config_.ambientTemperatureC;
    state_.manifoldPressureKpa = config_.ambientPressureKpa;
    state_.boostPressureRatio = 1.0;
    state_.forcedInductionShaftSpeedRpm = 0.0;
    state_.wastegateOpening = 0.0;
    state_.exhaustPressureKpa = config_.ambientPressureKpa;
    state_.intakeRunnerPressureKpa = config_.ambientPressureKpa;
    state_.exhaustRunnerPressureKpa = config_.ambientPressureKpa;
    randomState_ = 0x6d2b79f5U;
    for (std::size_t index = 0; index < cylinderRandomState_.size(); ++index)
        cylinderRandomState_[index] = 0x6d2b79f5U
            + 0x9e3779b9U * static_cast<std::uint32_t>(index + 1);
    cylinderMisfires_.fill(false);
    intakeFlowMgPerCycle_.fill(0.0);
    exhaustFlowMgPerCycle_.fill(0.0);
    intakeFlowMgThisCycle_.fill(0.0);
    exhaustFlowMgThisCycle_.fill(0.0);
    exhaustRunnerVelocityMps_.fill(0.0);
    cylinderFlowCycleStarted_.fill(false);
    instantaneousCombustionPulse_.fill(0.0);
    injectedFuelMolesThisCycle_.fill(0.0);
    meteredFuelMolesLastCycle_.fill(0.0);
    deliveredFuelMolesLastCycle_.fill(0.0);
    requestedFuelMolesThisCycle_.fill(0.0);
    trappedAirMassMgLastCycle_.fill(0.0);
    actualAfrLastCycle_.fill(config_.fuelProperties.stoichiometricAirFuelRatio);
    fuelDeliveryRatio_.fill(0.0);
    closedLoopFuelTrim_.fill(1.0);
    flameEvents_.fill({});
    injectionStates_.fill({});
    endGasKnockStates_.fill({});
    indicatedWorkStates_.fill({});
    valveTrainStates_.fill({});
    valveTrainResults_.fill({});
    runnerAcousticStates_.fill({});
    runnerAcousticResults_.fill({});
    ignitionDelayRemainingSeconds_.fill(0.0);
    ignitionPending_.fill(false);
    eventEvaluationAngleDegrees_ = state_.crankAngleDegrees;
    eventEvaluationTimeSeconds_ = state_.simulationTimeSeconds;
    indicatedWorkThisCycleJoules_ = 0.0;
    brakeWorkThisCycleJoules_ = 0.0;
    cycleElapsedSeconds_ = 0.0;
    cycleTelemetryStarted_ = false;
    const auto configureFuel = [this](GasCell& cell) {
        cell.configureFuelChemistry(config_.fuelProperties.molarMassGramsPerMole * 0.001,
            config_.fuelProperties.oxygenMolesPerFuelMole,
            config_.fuelProperties.productMolesPerFuelMole);
    };
    intakePlenumCount_ = std::max<std::size_t>(1,
        std::min<std::size_t>(config_.intakePaths.size(), intakePlenumGas_.size()));
    // Manifold: large plenum, no strong directional bias — neutral orientation.
    for (std::size_t pathIndex = 0; pathIndex < intakePlenumCount_; ++pathIndex) {
        const auto& intake = intakeGeometryAt(config_, pathIndex);
        configureFuel(intakePlenumGas_[pathIndex]);
        intakePlenumGas_[pathIndex].initialise(config_.ambientPressureKpa, intake.plenumVolumeLitres,
                                               config_.ambientTemperatureC + 273.15);
        const auto throttleRadius = intake.throttleDiameterMm * 0.0005;
        const auto throttleArea = std::numbers::pi * throttleRadius * throttleRadius;
        intakePlenumGas_[pathIndex].setGeometry(throttleArea, 0.0, 1.0);
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

        const auto& cyl = config_.cylinders[index];
        const auto intakePathIndex = intakePathIndexFor(config_, cyl);
        const auto& cylinderIntake = intakeGeometryAt(config_, intakePathIndex);
        const auto runnerDiamMm = cyl.intakeRunnerDiameterMm > 0.0
            ? cyl.intakeRunnerDiameterMm : cylinderIntake.runnerDiameterMm;
        const auto runnerAreaM2 = std::numbers::pi * std::pow(runnerDiamMm * 0.0005, 2.0);
        const auto pistonAreaM2 = std::numbers::pi * std::pow(cyl.boreMm * 0.0005, 2.0);

        // Geometry: positive momentum means gas flowing downward into the cylinder
        // for intake, and upward out of the cylinder for exhaust.
        intakeRunnerGas_[index].setGeometry(runnerAreaM2, 0.0,  1.0); // downward
        cylinderGas_[index].setGeometry(pistonAreaM2,    0.0, -1.0); // upward toward head

        intakeRunnerGas_[index].initialise(config_.ambientPressureKpa,
                                           intakeRunnerVolumeLitres(cyl, cylinderIntake),
                                           config_.ambientTemperatureC + 273.15);
        cylinderGas_[index].initialise(config_.ambientPressureKpa,
            evaluateCylinderKinematics(config_, kinematicsReference_, index,
                state_.crankAngleDegrees, 0.0).chamberVolumeLitres,
            config_.ambientTemperatureC + 273.15);
    }

    // Construction guarantees this invariant. reset() cannot report failure
    // through IEngineSimulation and must not silently revive the old lumped
    // exhaust, so an invariant violation is deliberately fatal.
    if (!physicalExhaustNetwork_
        || !physicalExhaustNetwork_->reset(config_.ambientPressureKpa * 1'000.0,
                                            config_.ambientTemperatureC + 273.15))
        std::terminate();
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
