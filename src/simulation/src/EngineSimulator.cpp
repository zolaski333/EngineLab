#include <enginelab/simulation/EngineSimulator.hpp>
#include <enginelab/foundation/ForcedInductionFlow.hpp>
#include <enginelab/simulation/TransientChargeEstimator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/MechanicalKinematics.hpp>
#include <enginelab/physics/CylinderHeatTransferModel.hpp>
#include <enginelab/physics/CombustionCycleVariation.hpp>
#include <enginelab/physics/DuctWallHeatTransferModel.hpp>
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

/** Inventory delta a runner-mouth sample applies to the plenum cell.
 *
 * The runner mouth is compiled as the network's "outlet" whose ambient
 * reservoir is the plenum, so a mouth sample is positive from the runner INTO
 * the plenum: the plenum's inventory moves WITH the sample (the network's own
 * inventory already moved against it). During a normal intake stroke every
 * component is therefore negative — the plenum is being drawn down.
 */
[[nodiscard]] GasInventoryDelta plenumDeltaForMouthSample(
    const gasdynamics::ExhaustOutletFlowSample& sample,
    const GasCell& cell) noexcept {
    const auto oxygen = static_cast<std::size_t>(gasdynamics::GasSpecies::oxygen);
    const auto inert = static_cast<std::size_t>(gasdynamics::GasSpecies::inert);
    const auto fuel = static_cast<std::size_t>(gasdynamics::GasSpecies::fuel);
    const auto burned = static_cast<std::size_t>(gasdynamics::GasSpecies::burned);
    return {
        {
            sample.speciesMassKg[oxygen] / GasCell::oxygenMolarMassKg,
            sample.speciesMassKg[inert] / GasCell::inertMolarMassKg,
            sample.speciesMassKg[fuel] / cell.configuredFuelMolarMassKg(),
            sample.speciesMassKg[burned] / cell.configuredBurnedGasMolarMassKg(),
        },
        sample.transferredEnergyJ,
        0.0,
        0.0,
    };
}

// From this cylinder count up, the shared intake plenum is frozen at the start of
// each gas sub-step and every cylinder reads that snapshot (Jacobi) instead of
// seeing the previous cylinder's writes (Gauss-Seidel).
//
// The threshold originally existed to make a per-cylinder fork-join safe, and was
// named for it. That fork-join has been REMOVED, and its removal is not an
// optimisation to be reversed on a faster machine: the dispatch rate is one
// barrier per gas sub-step, which at 5,940 rpm on a V8 is ~19,000/s, and no
// condition-variable barrier can amortise a body that small. Measured on a
// 16-thread laptop, realtime factor (simulated seconds produced per wall second,
// EngineLabRealtimeBudgetHarness), LS3 V8 / Merlin V12:
//
//   fork-join, broadcast wake + 2048-yield spin   0.266 / 0.286
//   fork-join, per-worker targeted wake + spin    0.249 / 0.256
//   fork-join, per-worker wake, no spin           0.282 / 0.336
//   inline (shipped)                              0.343 / 0.396
//
// Every threaded variant lost, and the spin actively stole cycles from the very
// thread the barrier was waiting on. A real parallelisation here would have to
// keep the worker threads resident ACROSS sub-steps, synchronising only where the
// shared plenum is touched -- a different architecture, not a tuning of this one.
// The removal was verified bit-identical on the LS3 (torque, IMEP, VE and
// air_mg agree to every printed digit).
constexpr std::size_t decoupledSharedVolumeCylinderThreshold = 8;

// Representative metal defaults for thermal walls. They describe
// geometry/material, never a target gas temperature: both wall temperatures
// emerge from conserved exchange with the simulated gas.
constexpr double aluminiumRunnerWallThicknessM = 0.003;
constexpr double aluminiumDensityKgPerM3 = 2'700.0;
constexpr double aluminiumSpecificHeatJPerKgK = 900.0;
constexpr double runnerExternalHeatTransferWPerM2K = 12.0;
// The duct wall exchange is sub-rated to this interval instead of running on
// every solver sub-step. A runner cell moves about 0.04% of the gas-wall
// equilibrium gap per sub-step -- a ~69 ms time constant integrated every
// ~26 us -- so the exchange itself is resolved some three orders of magnitude
// finer than it needs. What sets the floor is not that time constant but the
// sampling of the heat-transfer coefficient, which follows the flow: the
// binding scale is the cell residence time L/u, about 600 us at 50 m/s
// through a 30 mm cell. 150 us keeps four samples inside the fastest
// residence time and about thirty across an intake valve event at 7,000 rpm,
// while cutting the wall work by four.
constexpr double ductWallHeatUpdateIntervalSeconds = 150.0e-6;
constexpr double stainlessExhaustWallThicknessM = 0.0015;
constexpr double stainlessDensityKgPerM3 = 7'900.0;
constexpr double stainlessSpecificHeatJPerKgK = 500.0;
constexpr double exhaustExternalHeatTransferWPerM2K = 18.0;

/**
 * Flow area an intake still presents with the throttle plate fully closed.
 *
 * Scaled by the number of bores, exactly as the plate area is: leakage is a
 * property of each plate-in-bore fit, so a four-throttle intake leaks four
 * times as much as a single-throttle one of the same bore size. Clamped
 * non-negative so an authored configuration cannot subtract flow area.
 *
 * See IntakeConfig::closedThrottleLeakageAreaMm2 for why this belongs to the
 * hardware rather than to the ECU's idle command.
 */
[[nodiscard]] double closedThrottleLeakageAreaM2(const IntakeConfig& intake) noexcept {
    const auto perBoreMm2 = std::isfinite(intake.closedThrottleLeakageAreaMm2)
        ? std::max(0.0, intake.closedThrottleLeakageAreaMm2) : 0.0;
    return perBoreMm2 * static_cast<double>(std::max<std::uint32_t>(1, intake.throttleCount))
        * 1.0e-6;
}

/** Isentropic ideal-gas orifice flow from an upstream reservoir.
 *
 * Used for the compressor bypass valve, whose upstream charge pipe is a
 * boundary reservoir in the current zero-dimensional boost model. This is the
 * same choked/subcritical law used by the conservative gas solver, exposed
 * here without fabricating a control-volume inventory that the schema does not
 * yet provide.
 */
[[nodiscard]] double reservoirOrificeMassFlowKgPerSecond(
    double upstreamPressureKpa, double downstreamPressureKpa,
    double temperatureK, double effectiveAreaM2,
    double dischargeCoefficient) noexcept {
    if (!(upstreamPressureKpa > downstreamPressureKpa)
        || !(downstreamPressureKpa > 0.0) || !(temperatureK > 0.0)
        || !(effectiveAreaM2 > 0.0) || !(dischargeCoefficient > 0.0))
        return 0.0;
    constexpr double gamma = 1.4;
    constexpr double specificGasConstant = 287.05;
    const auto pressureRatio = downstreamPressureKpa / upstreamPressureKpa;
    const auto criticalRatio = std::pow(2.0 / (gamma + 1.0),
        gamma / (gamma - 1.0));
    const auto upstreamPressurePa = upstreamPressureKpa * 1'000.0;
    if (pressureRatio <= criticalRatio) {
        return dischargeCoefficient * effectiveAreaM2 * upstreamPressurePa
            * std::sqrt(gamma / (specificGasConstant * temperatureK))
            * std::pow(2.0 / (gamma + 1.0),
                (gamma + 1.0) / (2.0 * (gamma - 1.0)));
    }
    const auto pressureTerm = 2.0 * gamma / (gamma - 1.0)
        * (std::pow(pressureRatio, 2.0 / gamma)
            - std::pow(pressureRatio, (gamma + 1.0) / gamma));
    return dischargeCoefficient * effectiveAreaM2 * upstreamPressurePa
        / std::sqrt(specificGasConstant * temperatureK)
        * std::sqrt(std::max(0.0, pressureTerm));
}

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
};

struct ParallelSubstepState final {
    std::array<GasCell, 32> frozenPlenum {};
    std::array<CylinderSubstepScratch, 32> scratch {};
};

/**
 * Effective (Cd-weighted) valve flow area in mm^2.
 *
 * Two regimes, and the discharge coefficient means different things in each.
 * At low lift the jet stays attached to the seat and the flow is genuinely
 * curtain-limited, so `dischargeCoefficient` is the curtain-referenced Cd the
 * camshaft authors (0.6-0.7 for a real port, which is what the authored flow
 * curves carry). Above roughly L/D 0.2 the jet separates and the effective area
 * stops growing with lift: what it saturates at is a property of the PORT, and
 * the literature reports it as a fraction of the valve head area pi*D^2/4 --
 * 0.55-0.65 for a good production intake port, 0.50-0.55 for an exhaust one
 * (Heywood, Internal Combustion Engine Fundamentals, ch. 6).
 *
 * This used to cap the GEOMETRIC curtain at a geometric seat throat (0.88*D)
 * and let the caller multiply by Cd afterwards, which applied the seat
 * contraction twice: the delivered plateau came out at 0.62 * 0.88^2 = 0.48 of
 * head area, below any real production port. Measured on the CP3, restoring the
 * plateau to the literature band is worth ~12 % of peak power on its own. The
 * cap is therefore applied to the EFFECTIVE area, after Cd, and stated in the
 * units the literature reports it in. Low lift is untouched, which matters:
 * that regime governs idle bypass flow and overlap backflow.
 *
 * The returned area already carries Cd, so callers pass a discharge coefficient
 * of 1.0 alongside it.
 */
[[nodiscard]] double effectiveValveAreaMm2(double boreMm, double liftMm,
                                           std::uint32_t valveCount,
                                           double configuredDiameterMm,
                                           double dischargeCoefficient,
                                           double multiValveBoreRatio,
                                           double singleValveBoreRatio,
                                           double maximumHeadAreaFraction) noexcept {
    const auto count = static_cast<double>(std::clamp<std::uint32_t>(valveCount, 1, 4));
    // A multi-valve head packs smaller valves: literature per-valve D/B is
    // 0.44-0.48 intake and 0.38-0.42 exhaust for a two-valve head, against
    // 0.37-0.40 and 0.30-0.34 for a four-valve one. The ordering is what buys
    // a four-valve head its extra total area despite the smaller valves.
    const auto derivedRatio = valveCount == 1 ? singleValveBoreRatio : multiValveBoreRatio;
    const auto diameterMm = configuredDiameterMm > 0.0
        ? configuredDiameterMm : std::clamp(boreMm * derivedRatio, 12.0, 80.0);
    const auto coefficient = std::max(0.0, dischargeCoefficient);
    const auto curtainAreaMm2 = count * std::numbers::pi * diameterMm
        * std::max(0.0, liftMm);
    const auto headAreaMm2 = count * std::numbers::pi * diameterMm * diameterMm * 0.25;
    return std::min(coefficient * curtainAreaMm2,
                    maximumHeadAreaFraction * headAreaMm2);
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

[[nodiscard]] double fullLoadFuelLimitMg(
    const InjectionConfig& injection, double rpm) noexcept {
    const auto& curve = injection.fullLoadFuelLimit;
    if (curve.empty()) return std::numeric_limits<double>::infinity();
    if (curve.size() == 1 || rpm <= curve.front().rpm)
        return curve.front().milligramsPerCycle;
    for (std::size_t index = 1; index < curve.size(); ++index) {
        if (rpm <= curve[index].rpm) {
            const auto span = std::max(
                1.0, curve[index].rpm - curve[index - 1].rpm);
            const auto fraction = std::clamp(
                (rpm - curve[index - 1].rpm) / span, 0.0, 1.0);
            return std::lerp(
                curve[index - 1].milligramsPerCycle,
                curve[index].milligramsPerCycle, fraction);
        }
    }
    return curve.back().milligramsPerCycle;
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

[[nodiscard]] ActiveCamshaft activeCamshaft(
    const EngineConfig& config, const CylinderConfig& cylinder,
    double rpm, double throttle) noexcept {
    const CamshaftConfig* selected = &config.camshafts;
    const auto bank = std::find_if(
        config.banks.begin(), config.banks.end(),
        [&cylinder](const CylinderBankConfig& item) {
            return item.id == cylinder.bankId
                || std::find(item.cylinderIds.begin(),
                    item.cylinderIds.end(), cylinder.id)
                    != item.cylinderIds.end();
        });
    if (bank != config.banks.end()) selected = &bank->camshafts;
    const auto high = selected->variableProfileEnabled
        && rpm >= selected->switchRpm
        && throttle >= selected->switchThrottle;
    return { selected, high };
}

}

EngineSimulator::EngineSimulator(EngineConfig config, IEcuModel& ecu, IPhysicsModel& physics,
                                 IFiringEventGenerator& events, IExhaustModel& exhaust,
                                 EngineSimulatorOptions options)
    : config_(std::move(config)), options_(std::move(options)), ecu_(ecu), physics_(physics),
      eventGenerator_(events), exhaust_(exhaust) {
    normaliseEngineConfig(config_);
    if (const auto error = validateEngineConfig(config_)) throw std::invalid_argument(*error);
    setIntakeWallHeatUpdateIntervalSeconds(
        options_.intakeWallHeatUpdateIntervalSeconds.value_or(
            ductWallHeatUpdateIntervalSeconds));
    for (std::size_t index = 0; index < config_.cylinders.size(); ++index) {
        const auto& cylinder = config_.cylinders[index];
        intakePathIndexByCylinder_[index] =
            intakePathIndexFor(config_, cylinder);
        exhaustPathIndexByCylinder_[index] =
            exhaustPathIndexFor(config_, cylinder);
    }
    ecu_.initialise(config_);
    kinematicsReference_ = buildEngineKinematicsReference(config_);
    configurePhysicalExhaustNetwork();
    configurePhysicalIntakeNetworks();
    configureIntakeWorkerPool();
    reset();
}

void EngineSimulator::configureIntakeWorkerPool() {
    // Only the 1-D runner advance is dispatched, and only its per-cylinder
    // half. The 95 mm production mesh makes each item too short to amortise one
    // participant per cylinder: on the 12-thread reference machine a same-hour,
    // counterbalanced free-run sweep found two workers best for the V8 and
    // three for the V12. Four and five workers lost to barrier/atomic traffic;
    // an undispatched twin tracked machine drift as the null control.
    //
    // The work-cardinality cap below preserves those measured optima. The
    // hardware cap is still lower when necessary, and half the reported
    // concurrency remains reserved for the audio callback, UI and OS. Explicit
    // harness overrides deliberately bypass this production policy.
    const auto cylinderCount = config_.cylinders.size();
    if (options_.intakeStaircaseRounds.has_value())
        intakeStaircaseRounds_ = std::clamp<std::size_t>(
            *options_.intakeStaircaseRounds, 0, 4);
    if (cylinderCount < 3) return;
    if (options_.intakeWorkerCount.has_value()) {
        const auto workerCount = std::min(*options_.intakeWorkerCount, cylinderCount - 1);
        if (workerCount == 0) return;
        intakeWorkerPool_ = std::make_unique<CylinderWorkerPool>(workerCount);
        intakePredictionGroupCount_ = 1;
        return;
    }
    const auto reportedConcurrency = std::thread::hardware_concurrency();
    if (reportedConcurrency < 4) return;
    const auto usableThreads = std::max<std::size_t>(1, reportedConcurrency / 2);
    const std::size_t workloadWorkerCap = cylinderCount >= 10 ? 3 : 2;
    const auto workerCount = std::min(
        { cylinderCount - 1, usableThreads - 1, workloadWorkerCap });
    if (workerCount == 0) return;
    intakeWorkerPool_ = std::make_unique<CylinderWorkerPool>(workerCount);
    // Only a group with more than one cylinder in it needs the plenum staircase
    // reconstructed. Without a pool the groups stay singletons and the runner
    // pass is the original serial Gauss-Seidel, exactly, at the original cost.
    intakePredictionGroupCount_ = 1;
}

void EngineSimulator::configurePhysicalIntakeNetworks() {
    for (std::size_t index = 0; index < config_.cylinders.size(); ++index) {
        const auto& cylinder = config_.cylinders[index];
        const auto intakePathIndex = intakePathIndexByCylinder_[index];
        const auto& intake = intakeGeometryAt(config_, intakePathIndex);
        const auto lengthM = (cylinder.intakeRunnerLengthMm > 0.0
            ? cylinder.intakeRunnerLengthMm : intake.runnerLengthMm) * 0.001;
        const auto inletDiameterM = (cylinder.intakeRunnerDiameterMm > 0.0
            ? cylinder.intakeRunnerDiameterMm : intake.runnerDiameterMm) * 0.001;
        const auto outletDiameterM = (intake.runnerPlenumDiameterMm > 0.0
            ? intake.runnerPlenumDiameterMm : inletDiameterM * 1'000.0) * 0.001;
        const auto inletAreaM2 =
            std::numbers::pi * inletDiameterM * inletDiameterM * 0.25;
        const auto outletAreaM2 =
            std::numbers::pi * outletDiameterM * outletDiameterM * 0.25;
        const auto meanAreaM2 = (inletAreaM2
            + std::sqrt(inletAreaM2 * outletAreaM2) + outletAreaM2) / 3.0;

        gasdynamics::CompiledExhaustDuct runner;
        runner.nodeId = cylinder.id;
        runner.sourceComponentId = cylinder.id;
        runner.pathIndex = static_cast<std::uint32_t>(intakePathIndex);
        runner.lengthM = std::max(0.03, lengthM);
        runner.flowAreaM2 = meanAreaM2;
        runner.inletFlowAreaM2 = inletAreaM2;
        runner.outletFlowAreaM2 = outletAreaM2;
        runner.connectionAreaM2 = meanAreaM2;
        runner.inletConnectionAreaM2 = inletAreaM2;
        runner.outletConnectionAreaM2 = outletAreaM2;
        runner.hydraulicDiameterM =
            2.0 * std::sqrt(meanAreaM2 / std::numbers::pi);
        runner.volumeM3 = meanAreaM2 * runner.lengthM;
        // The realtime mesh targets 95 mm while retaining at least three
        // volumes: a quarter-wave fundamental (lambda ~= 4L) therefore has at
        // least twelve cells per wavelength under the unchanged MUSCL spatial
        // reconstruction. The 30 mm / RK2 / every-substep variant remains
        // reachable through EngineSimulatorOptions as the offline oracle.
        const auto targetCellLengthM = std::clamp(
            options_.intakeTargetCellLengthM.value_or(0.095),
            0.020, 0.150);
        constexpr auto minimumCellCount = 3U;
        runner.cellCount = std::clamp<std::size_t>(
            static_cast<std::size_t>(
                std::llround(runner.lengthM / targetCellLengthM)),
            minimumCellCount, 12);
        if (options_.intakeMaximumCellCount.has_value())
            runner.cellCount = std::min(
                runner.cellCount,
                std::clamp<std::size_t>(*options_.intakeMaximumCellCount, 3, 12));

        gasdynamics::CompiledCylinderPort port;
        port.cylinderId = cylinder.id;
        port.pathIndex = runner.pathIndex;
        port.networkEndpoint = {
            gasdynamics::ExhaustEndpointType::ductInlet, 0, cylinder.id };
        port.runnerConnectionAreaM2 = inletAreaM2;
        port.dischargeCoefficient = 1.0; // the valve Cd arrives per advance
        gasdynamics::CompiledExhaustOutlet mouth;
        mouth.outletNodeId = cylinder.id;
        mouth.pathIndex = runner.pathIndex;
        mouth.networkEndpoint = {
            gasdynamics::ExhaustEndpointType::ductOutlet, 0, cylinder.id };
        mouth.openingAreaM2 = outletAreaM2;
        mouth.dischargeCoefficient = 1.0; // a runner mouth is a bellmouth

        const auto layout = gasdynamics::ExhaustNetworkLayout::assemble(
            { runner }, {}, {}, { port }, { mouth });
        if (!layout.valid())
            throw std::runtime_error("failed to assemble intake runner layout");
        auto network = std::make_unique<gasdynamics::ExhaustGasNetwork>(
            exhaustThermodynamicsFor(config_));
        gasdynamics::ExhaustGasNetworkConfig networkConfig;
        networkConfig.initialPressurePa = config_.ambientPressureKpa * 1'000.0;
        networkConfig.initialTemperatureK = config_.ambientTemperatureC + 273.15;
        networkConfig.absoluteRoughnessM = 1.5e-6; // smooth aluminium/plastic
        networkConfig.maximumCourantNumber = 0.8;
        networkConfig.wallHeatTransferWPerM2K = 0.0;
        networkConfig.wallTemperatureK = config_.ambientTemperatureC + 273.15;
        networkConfig.dynamicWallHeatTransferEnabled = true;
        networkConfig.wallThicknessM = aluminiumRunnerWallThicknessM;
        networkConfig.wallDensityKgPerM3 = aluminiumDensityKgPerM3;
        networkConfig.wallSpecificHeatJPerKgK = aluminiumSpecificHeatJPerKgK;
        networkConfig.externalWallHeatTransferWPerM2K =
            runnerExternalHeatTransferWPerM2K;
        networkConfig.externalTemperatureK = config_.ambientTemperatureC + 273.15;
        // The runner networks are advanced concurrently, so the cadence is
        // driven from `advanceIntakeRunners` on the second half-step, where
        // every cylinder is dispatched together. A self-timed network would
        // burst on a different dispatch from its siblings and the barrier
        // would pay every burst.
        networkConfig.wallHeatUpdateExternallyTriggered = true;
        networkConfig.firstOrderTimeIntegration =
            options_.intakeFirstOrderTimeIntegration.value_or(true);
        if (!network->configure(layout, networkConfig))
            throw std::runtime_error("failed to configure intake runner network");
        intakeRunnerNetworks_[index] = std::move(network);
    }
}

void EngineSimulator::configurePhysicalExhaustNetwork() {
    const auto physicalGraph = ExhaustGraph::makeForEngine(config_);
    gasdynamics::ExhaustNetworkDiscretisation feedbackMesh;
    // The realtime FV mesh owns nonlinear mean-flow/back-pressure feedback,
    // not audio-band propagation. A 360 mm maximum cell length gives five
    // control volumes per 1.8 m wavelength (roughly 330-360 Hz in hot exhaust),
    // covering the resolved firing fundamentals of the shipped engines. The
    // characteristic audio network transports the remaining band without asking
    // every exhaust cell to run at more than 100 kHz. This is an explicit
    // physical scale separation: no
    // authored component, volume, area or loss is removed from either model.
    feedbackMesh.targetCellLengthM = std::clamp(
        options_.exhaustTargetCellLengthM.value_or(0.360), 0.025, 0.600);
    // Components shorter than the feedback scale remain one conservative
    // finite volume with their exact volume, ports and loss. Their propagation
    // delay is owned by the characteristic network, so duplicating a second FV
    // cell would add cost but no resolved mean-flow information.
    feedbackMesh.minimumCellsPerDuct = 1;
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
    // Exhaust-wall temperature is a finite-capacity state. Internal exchange
    // conserves gas plus metal energy; only the explicit outside convection
    // below rejects heat from that combined subsystem to the environment.
    networkConfig.wallHeatTransferWPerM2K = 0.0;
    networkConfig.wallTemperatureK = config_.ambientTemperatureC + 273.15;
    networkConfig.dynamicWallHeatTransferEnabled = true;
    networkConfig.wallThicknessM = stainlessExhaustWallThicknessM;
    networkConfig.wallDensityKgPerM3 = stainlessDensityKgPerM3;
    networkConfig.wallSpecificHeatJPerKgK = stainlessSpecificHeatJPerKgK;
    networkConfig.externalWallHeatTransferWPerM2K =
        exhaustExternalHeatTransferWPerM2K;
    networkConfig.externalTemperatureK = config_.ambientTemperatureC + 273.15;
    networkConfig.wallHeatUpdateIntervalSeconds =
        ductWallHeatUpdateIntervalSeconds;
    networkConfig.evolveJunctionAxialMomentum =
        options_.evolveExhaustJunctionAxialMomentum.value_or(true);
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
    if (enabled && !pressureSamples_) {
        pressureSamples_ = std::make_unique<SpscQueue<CylinderPressureSample, 1'024>>();
        exhaustAcousticSamples_ =
            std::make_unique<SpscQueue<ExhaustAcousticSample, 1'024>>();
    } else if (!enabled) {
        pressureSamples_.reset();
        exhaustAcousticSamples_.reset();
    }
}

void EngineSimulator::applyAudioPhysicsCalibration(
    const AudioPhysicsCalibration& calibration) noexcept {
    config_.combustionCalibration.cycleVariationCoefficientOfVariation =
        calibration.cycleVariationCoefficientOfVariation;
    config_.combustionCalibration.cycleVariationCorrelation =
        calibration.cycleVariationCorrelation;
    config_.ignition.limiterKeepsFuel = calibration.limiterKeepsFuel;
    config_.exhaustAfterfire = calibration.exhaustAfterfire;
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
    safeControls.externalRotatingInertiaKgM2 =
        std::isfinite(controls.externalRotatingInertiaKgM2)
        ? std::clamp(controls.externalRotatingInertiaKgM2, 0.0, 100.0)
        : 0.0;
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

    // Couple the low-band nonlinear network at no fewer than sixteen samples per
    // four-stroke firing period. This is a physical multirate integration rate,
    // not a frame skip: valve CdA is integrated over every mechanical substep,
    // while the instantaneous Riemann boundary remains sampled at that faster
    // cadence for audio. The absolute cap also resolves cranking transients where
    // firing frequency alone approaches zero.
    //
    // The cap sets the audio bandwidth of the exhaust boundary wherever the
    // per-firing-period rule is slower than it -- i.e. at idle and low rpm,
    // and across the whole range of low-cylinder-count engines. At 500 us the
    // published coupling Nyquist was ~1 kHz there, which listeners reported as
    // a muffled, undifferentiated exhaust: every engine's blowdown edge was
    // smeared by the same reconstruction cutoff. The former 250 us production
    // cap doubled that headroom to ~2 kHz; the measured 125 us cap used here
    // doubles it again to at least 4 kHz.
    // The extra cost is per-flush overhead only (the network's internal CFL
    // substep count per second is unchanged), and the low-speed regime where
    // this cap binds is far below the redline load case that sets the budget.
    // Measured before committing: see docs/thermoacoustic-architecture.md.
    const auto maximumLowSpeedCouplingSeconds = std::clamp(
        options_.maximumLowSpeedExhaustCouplingSeconds.value_or(125.0e-6),
        25.0e-6, 500.0e-6);
    constexpr auto couplingSamplesPerFiringPeriod = 16.0;
    const auto firingFrequencyHz = std::abs(state_.rpm)
        * static_cast<double>(config_.cylinders.size()) / 120.0;
    const auto productionExhaustCouplingSeconds = firingFrequencyHz > 1.0e-9
        ? std::min(maximumLowSpeedCouplingSeconds,
            1.0 / (couplingSamplesPerFiringPeriod * firingFrequencyHz))
        : maximumLowSpeedCouplingSeconds;
    const auto maximumExhaustCouplingSeconds = exhaustCouplingEverySubstep_
        ? subDt : productionExhaustCouplingSeconds;
    // Telemetry: the duration-based flush lands on the substep nearest the
    // target interval, so the actual knot cadence rounds to the substep grid.
    const auto couplingSubsteps = std::max<long long>(1,
        std::llround(maximumExhaustCouplingSeconds / subDt));
    state_.exhaustCouplingFrequencyHz = subDt > 0.0
        ? 1.0 / (subDt * static_cast<double>(couplingSubsteps)) : 0.0;
    // Coupling accumulators are members carried across frames; see the header.
    // Diagnostic only: how much acoustic bandwidth the network resolves against
    // how much the audio boundary is allowed to observe. See EngineState.
    std::uint64_t exhaustNetworkAcceptedSubsteps = 0;
    double exhaustNetworkAdvancedSeconds = 0.0;

    for (std::size_t subStep = 0; subStep < subStepCount; ++subStep) {
        const auto subStepStartTime = state_.simulationTimeSeconds;
        const auto subPreviousRpm = state_.rpm;
        const auto subPreviousAngle = state_.crankAngleDegrees;
        std::array<double, 8> intakeThrottleConductanceAreaM2 {};

        // Engine load is a thermodynamic state (approximately MAP / ambient
        // for a naturally aspirated SI engine), not the operator's brake
        // command.  Coupling flame turbulence and VVT to the dyno slider made
        // a wide-open, unloaded engine look like a zero-load combustion event.
        const auto compressionIgnitionEngine =
            config_.fuel == FuelType::diesel;
        const auto thermodynamicLoad = compressionIgnitionEngine
            ? std::clamp(std::max(safeControls.throttle,
                (safeControls.ignitionEnabled || safeControls.starterEngaged)
                    ? 0.08 : 0.0), 0.0, 1.0)
            : std::clamp(state_.manifoldPressureKpa
                / std::max(1.0, config_.ambientPressureKpa), 0.0, 1.5);
        state_.load = smooth(state_.load, thermodynamicLoad, subDt, 12.0);
        const auto ecuCommand = ecu_.evaluate(config_, state_, safeControls);
        state_.exhaustAfterfireOverrunActive =
            ecuCommand.overrunAfterfireActive;
        state_.exhaustAfterfireBlockers = ecuCommand.overrunAfterfireBlockers;
        state_.ecuFuelCorrection = ecuCommand.fuelCorrection;
        state_.ecuDieselFuelQuantityLimitMgPerCycle =
            ecuCommand.dieselFuelQuantityLimitMgPerCycle;
        state_.ecuFuelEnabled = ecuCommand.fuelEnabled;
        state_.ecuSparkEnabled = ecuCommand.sparkEnabled;
        state_.ecuSoftRevLimiterActive = ecuCommand.softRevLimiterActive;
        state_.ecuHardRevLimiterActive = ecuCommand.hardRevLimiterActive;
        state_.ecuAlternatingSparkCutActive =
            ecuCommand.alternatingSparkCutActive;
        state_.ecuDecelerationFuelCutActive =
            ecuCommand.decelerationFuelCutActive;
        state_.throttle = smooth(state_.throttle, ecuCommand.effectiveThrottle, subDt, 10.0);
        if (compressionIgnitionEngine) {
            // A conventional diesel has no load-controlling throttle plate:
            // pedal and idle governor command fuel while the intake remains
            // open. `state_.load` therefore follows fuel demand rather than MAP.
            const auto idleFuelDemand =
                (safeControls.ignitionEnabled || safeControls.starterEngaged)
                ? ecuCommand.idleAirOpening * 0.20 : 0.0;
            state_.load = smooth(state_.load,
                std::clamp(std::max(state_.throttle, idleFuelDemand), 0.0, 1.0),
                subDt, 18.0);
        }
        const auto stationary = state_.rpm < 20.0 && !safeControls.starterEngaged;
        if (stationary) {
            state_.boostPressureRatio = 1.0;
            state_.forcedInductionShaftSpeedRpm = 0.0;
            state_.wastegateOpening = 0.0;
            state_.blowOffMassFlowKgPerSecond = 0.0;
            state_.compressorPowerKw = 0.0;
            state_.turbinePowerKw = 0.0;
            for (std::size_t pathIndex = 0; pathIndex < intakePlenumCount_; ++pathIndex) {
                const auto& intake = intakeGeometryAt(config_, pathIndex);
                const auto throttleRadiusM = intake.throttleDiameterMm * 0.0005;
                const auto throttlePlateAreaM2 = static_cast<double>(intake.throttleCount)
                    * std::numbers::pi * throttleRadiusM * throttleRadiusM;
                const auto throttleAreaM2 = compressionIgnitionEngine
                    ? throttlePlateAreaM2
                    : closedThrottleLeakageAreaM2(intake)
                        + intake.idleBypassAreaMm2 * ecuCommand.idleAirOpening * 1.0e-6
                        + throttlePlateAreaM2 * std::pow(
                            state_.throttle, intake.throttleGamma);
                intakeThrottleConductanceAreaM2[pathIndex] = throttleAreaM2
                    * intake.throttleDischargeCoefficient;
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
            const auto compressorMassFlowKgPerSecond = airMassFlowKgPerSecond
                + std::max(0.0, state_.blowOffMassFlowKgPerSecond);
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
                state_.forcedInductionShaftSpeedRpm = state_.rpm
                    * config_.forcedInduction.superchargerDriveRatio;
                state_.wastegateOpening = 0.0;
            } else if (config_.forcedInduction.enabled) {
                const auto exhaustFlowSplit = partitionTurboExhaustFlow(
                    config_.forcedInduction, exhaustMassFlowKgPerSecond,
                    state_.wastegateOpening);
                const auto designOmega = config_.forcedInduction.designShaftSpeedRpm
                    * 2.0 * std::numbers::pi / 60.0;
                auto shaftOmega = state_.forcedInductionShaftSpeedRpm
                    * 2.0 * std::numbers::pi / 60.0;
                const auto turbineInletPressureKpa = std::max(state_.exhaustPressureKpa,
                                                               state_.exhaustRunnerPressureKpa);
                // A turbine expands to ITS OWN OUTLET, not to atmosphere.
                //
                // Referencing the expansion to ambient hard-wired away the one
                // thing a downpipe and a silencer actually set. The only
                // remaining path from exhaust geometry to a turbo's output was
                // then the INLET term, and that one has the wrong sign: larger
                // primaries expand the blowdown pulse into more volume and
                // lower the peak the turbine is charged with. So the model
                // could only ever answer "bigger exhaust, less boost", which is
                // what a user rebuilding a 2JZ exhaust in 80 mm reported --
                // more inertia, less response, and no improvement anywhere.
                //
                // The downstream system is not a separate control volume here,
                // so its back pressure is taken quasi-steadily from the flow it
                // is actually passing: the dynamic head an orifice of the
                // configured downstream conductance needs to pass this mass
                // flow. It is bounded well below the inlet pressure -- a
                // turbine outlet above its own inlet is not a turbine -- and it
                // reduces exactly to the previous behaviour as the downstream
                // conductance grows, so a genuinely free downpipe still reads
                // ambient.
                const auto downstreamConductanceM2 =
                    std::max(1.0e-6, exhaustOutletConductanceM2_);
                const auto downstreamDensityKgPerM3 = std::max(0.05,
                    config_.ambientPressureKpa * 1'000.0
                        / (287.0 * std::max(250.0, exhaustTemperatureK)));
                const auto downstreamVelocityMps =
                    std::max(0.0, exhaustMassFlowKgPerSecond)
                    / (downstreamDensityKgPerM3 * downstreamConductanceM2);
                const auto downstreamLossKpa = std::min(
                    0.5 * downstreamDensityKgPerM3 * downstreamVelocityMps
                        * downstreamVelocityMps * 0.001,
                    0.60 * turbineInletPressureKpa);
                const auto turbineOutletPressureKpa = std::max(1.0,
                    config_.ambientPressureKpa + downstreamLossKpa);
                state_.turbineOutletPressureKpa = turbineOutletPressureKpa;
                const auto turbineExpansionRatio = std::max(1.0,
                    turbineInletPressureKpa / turbineOutletPressureKpa);
                turbinePowerW = exhaustFlowSplit.turbineKgPerSecond
                    * exhaustCpJPerKgK * exhaustTemperatureK
                    * (1.0 - std::pow(turbineExpansionRatio, -compressorExponent))
                    * config_.forcedInduction.turbineEfficiency;
                desiredCompressorPowerW = compressorMassFlowKgPerSecond * airCpJPerKgK * ambientTemperatureK
                    * (std::pow(std::max(1.0, state_.boostPressureRatio), compressorExponent) - 1.0)
                    / std::max(0.35, config_.forcedInduction.compressorEfficiency);
                state_.wastegateOpening = std::clamp((state_.boostPressureRatio
                    - config_.forcedInduction.wastegatePressureRatio + 0.02) / 0.08, 0.0, 1.0);
                const auto speedRatio = shaftOmega / std::max(1.0, designOmega);
                const auto bearingPowerW = config_.forcedInduction.bearingFrictionPowerWatts
                    * speedRatio * speedRatio;
                const auto shaftPowerW = turbinePowerW
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
            desiredCompressorPowerW = compressorMassFlowKgPerSecond * airCpJPerKgK * ambientTemperatureK
                * (std::pow(std::max(1.0, pressureRatioTarget), compressorExponent) - 1.0)
                / std::max(0.35, config_.forcedInduction.compressorEfficiency);
            state_.compressorPowerKw = desiredCompressorPowerW * 0.001;
            state_.turbinePowerKw = turbinePowerW * 0.001;
            state_.boostPressureRatio = smooth(state_.boostPressureRatio,
                config_.forcedInduction.enabled ? pressureRatioTarget : 1.0, subDt, 14.0);
            if (!config_.forcedInduction.enabled) {
                state_.forcedInductionShaftSpeedRpm = 0.0;
                state_.wastegateOpening = 0.0;
                state_.blowOffMassFlowKgPerSecond = 0.0;
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
            if (config_.forcedInduction.enabled
                    && config_.forcedInduction.blowOffValveFlowAreaMm2 > 0.0) {
                const auto referencePressureRatio = intakeSourcePressureKpa
                    / std::max(1.0, state_.manifoldPressureKpa);
                const auto opening = std::clamp((referencePressureRatio
                    - config_.forcedInduction.blowOffValveOpeningPressureRatio)
                    / 0.08, 0.0, 1.0);
                state_.blowOffMassFlowKgPerSecond =
                    reservoirOrificeMassFlowKgPerSecond(
                        intakeSourcePressureKpa, config_.ambientPressureKpa,
                        temperatureK,
                        config_.forcedInduction.blowOffValveFlowAreaMm2
                            * opening * 1.0e-6,
                        config_.forcedInduction.blowOffValveDischargeCoefficient);
            } else {
                state_.blowOffMassFlowKgPerSecond = 0.0;
            }

            for (std::size_t pathIndex = 0; pathIndex < intakePlenumCount_; ++pathIndex) {
                const auto& intake = intakeGeometryAt(config_, pathIndex);
                const auto throttleRadiusM = intake.throttleDiameterMm * 0.0005;
                const auto throttlePlateAreaM2 = static_cast<double>(intake.throttleCount)
                    * std::numbers::pi * throttleRadiusM * throttleRadiusM;
                const auto idleBypassAreaM2 = intake.idleBypassAreaMm2
                    * ecuCommand.idleAirOpening * 1.0e-6;
                const auto throttleAreaM2 = compressionIgnitionEngine
                    ? throttlePlateAreaM2
                    : closedThrottleLeakageAreaM2(intake)
                        + idleBypassAreaM2
                        + throttlePlateAreaM2 * std::pow(
                            state_.throttle, intake.throttleGamma);
                intakeThrottleConductanceAreaM2[pathIndex] = throttleAreaM2
                    * intake.throttleDischargeCoefficient;
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
        // Spatial mean over the ports, taken in the same sweep as the peak. The
        // peak is what audio and blowdown telemetry need; the mean is what back
        // pressure means, and conflating them made a free-flowing engine look
        // restricted. See EngineState::exhaustPressureKpa.
        double portPressureSumKpa = 0.0;
        std::size_t portPressureCount = 0;
        for (const auto& exchange : physicalExhaustNetwork.cylinderExchanges()) {
            const auto portKpa = exchange.networkPressurePa * 0.001;
            collectorPressureKpa = std::max(collectorPressureKpa, portKpa);
            portPressureSumKpa += portKpa;
            ++portPressureCount;
        }
        state_.exhaustPressureKpa = collectorPressureKpa;
        // Damp the spatial mean over ~3 firing periods so the published figure is
        // a gauge reading rather than a snapshot of wherever in the blowdown the
        // sub-step happened to land. Floored so a cranking or stalled engine still
        // converges instead of freezing at its initial value.
        if (portPressureCount > 0) {
            const auto instantaneousMeanKpa = portPressureSumKpa
                / static_cast<double>(portPressureCount);
            state_.exhaustBackPressureKpa = smooth(state_.exhaustBackPressureKpa,
                instantaneousMeanKpa, subDt, std::max(4.0, firingFrequencyHz / 3.0));
        }
        const auto backPressure = state_.exhaustPressureKpa;
        const auto combustion = physics_.evaluateCombustion(config_, state_, safeControls, ecuCommand, backPressure);
        const auto sparkCombustionAvailable =
            config_.fuel == FuelType::gasoline && ecuCommand.sparkEnabled
            && state_.rpm >= 220.0 && state_.damage < 1.0;
        const auto compressionIgnitionAvailable =
            config_.fuel == FuelType::diesel && ecuCommand.fuelEnabled
            && state_.rpm >= 220.0 && state_.damage < 1.0;
        const auto configuredDieselFullLoadFuelLimitMg =
            compressionIgnitionEngine && ecuCommand.fuelEnabled
            ? fullLoadFuelLimitMg(config_.injection, state_.rpm)
            : std::numeric_limits<double>::infinity();
        const auto dieselFullLoadFuelLimitMg =
            compressionIgnitionEngine && ecuCommand.fuelEnabled
                && ecuCommand.dieselFuelQuantityLimitMgPerCycle > 0.0
                && std::isfinite(ecuCommand.dieselFuelQuantityLimitMgPerCycle)
            ? ecuCommand.dieselFuelQuantityLimitMgPerCycle
            : configuredDieselFullLoadFuelLimitMg;
        const auto cranking = safeControls.starterEngaged && state_.rpm < 620.0 && state_.damage < 1.0;
        const auto displacement = engineDisplacementLitres(config_);
        const auto rotatingInertia =
            effectiveRotatingInertiaKgM2(config_)
            + safeControls.externalRotatingInertiaKgM2;
        const auto fuelMolarMassKg =
            config_.fuelProperties.molarMassGramsPerMole * 0.001;
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
        std::array<double, 32> pistonAreaM2ForSplit {};
        std::array<double, 32> chamberVolumeLitresForWork {};
        std::array<double, 32> chamberPressureBeforeNetworkKpa {};
        std::array<double, 32> gasTorqueLeverArmM {};
        std::array<double, 32> structuralInertiaForceN {};
        std::array<double, 32> structuralSideRatio {};
        std::array<double, 32> instantaneousIntakeTransferredMassKg {};
        std::array<std::uint8_t, 32> intakeInjectionRefused {};
        std::array<double, 32> exhaustMassFlowKgPerSecond {};
        std::array<double, 32> exhaustAcousticMassFlowKgPerSecond {};
        std::array<double, 32> exhaustPortDensityKgPerM3 {};
        std::array<double, 32> exhaustPortSpeedOfSoundMps {};
        std::array<std::uint8_t, 32> thermoacousticBoundaryValid {};
        std::array<bool, 32> cycleBoundaryForWork {};
        // Phase 0 is firing TDC, so [180, 540) is exhaust stroke + intake
        // stroke: the pumping loop. Captured in the first per-cylinder pass and
        // consumed by the indicated-work split in the second.
        std::array<bool, 32> gasExchangeStrokeForWork {};
        // And [180, 360) alone is the exhaust stroke, which splits the pumping
        // loop into back pressure and intake depression.
        std::array<bool, 32> exhaustStrokeForWork {};
        std::array<bool, 32> intakeCloseForTrappedAir {};

        // Small engines retain Gauss-Seidel shared-volume updates and a shared
        // PRNG sequence; from this cylinder count up, the sub-step switches to a
        // Jacobi snapshot instead.
        //
        // This is purely a PHYSICS choice and is keyed only to cylinder count, so
        // that an engine integrates identically on every machine. It used to also
        // gate a per-cylinder fork-join, which has been removed: see the note on
        // `decoupledSharedVolumeCylinderThreshold` for the measurements. Do not
        // re-couple the two -- changing this threshold changes the shared-volume
        // solver for real, and would move every large engine's calibration.
        const auto decoupleSharedVolumes =
            config_.cylinders.size() >= decoupledSharedVolumeCylinderThreshold;
        // Freeze both shared control volumes at the start of the sub-step so that
        // every cylinder evaluates the same upstream/downstream state.
        // The plenum snapshot keeps every cylinder in the sub-step reading the
        // same upstream state; writes to the plenum happen only in the
        // intake-network advances, so no commit transaction is needed.
        std::optional<ParallelSubstepState> parallelState;
        if (decoupleSharedVolumes) {
            parallelState.emplace();
            for (std::size_t path = 0; path < intakePlenumCount_; ++path)
                parallelState->frozenPlenum[path] = intakePlenumGas_[path];
        }
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
            const auto intakePathIndex = intakePathIndexByCylinder_[cylinderIndex];
            const auto crankOffset = crankOffsetDegreesFor(config_, cylinder);
            const auto cyclePhase = std::fmod(state_.crankAngleDegrees - crankOffset + 1'440.0, 720.0);
            const auto cams = activeCamshaft(
                config_, cylinder, state_.rpm, state_.throttle);
            const auto previousPhase = previousCylinderPhases_[cylinderIndex];
            const auto cycleBoundaryCrossed = crossedPhase(previousPhase, cyclePhase, 0.0);
            if (cycleBoundaryCrossed) {
                commandedSparkEventsLastCycle_[cylinderIndex] =
                    commandedSparkEventsThisCycle_[cylinderIndex];
                completedIgnitionEventsLastCycle_[cylinderIndex] =
                    completedIgnitionEventsThisCycle_[cylinderIndex];
                commandedSparkPhaseLastCycle_[cylinderIndex] =
                    commandedSparkPhaseThisCycle_[cylinderIndex];
                completedIgnitionPhaseLastCycle_[cylinderIndex] =
                    completedIgnitionPhaseThisCycle_[cylinderIndex];
                commandedSparkEventsThisCycle_[cylinderIndex] = 0;
                completedIgnitionEventsThisCycle_[cylinderIndex] = 0;
                commandedSparkPhaseThisCycle_[cylinderIndex] = -1.0;
                completedIgnitionPhaseThisCycle_[cylinderIndex] = -1.0;
                // A schedule deliberately SURVIVES this boundary. It used to be
                // discarded here so a prevented event could not leak into the
                // next cycle, but the cycle boundary is not the end of a spark
                // schedule: a retarded event (negative commanded advance) sits
                // just after it, and discarding it there is what turned a
                // retard into a misfire. The schedule is now a bounded
                // countdown, so it expires inside its own power stroke by
                // construction and cannot reach the next compression stroke.
                if (cylinderFlowCycleStarted_[cylinderIndex]) {
                    intakeFlowMgPerCycle_[cylinderIndex] = std::max(0.0,
                        intakeFlowMgThisCycle_[cylinderIndex]);
                    exhaustFlowMgPerCycle_[cylinderIndex] = std::max(0.0,
                        exhaustFlowMgThisCycle_[cylinderIndex]);
                    // Not clamped: a cylinder that reverts more fresh air than
                    // it draws has a genuinely negative delivery, and hiding
                    // that behind a zero would hide the failure this measures.
                    deliveredAirMgPerCycle_[cylinderIndex] =
                        deliveredAirMgThisCycle_[cylinderIndex];
                } else {
                    cylinderFlowCycleStarted_[cylinderIndex] = true;
                }
                intakeFlowMgThisCycle_[cylinderIndex] = 0.0;
                exhaustFlowMgThisCycle_[cylinderIndex] = 0.0;
                deliveredAirMgThisCycle_[cylinderIndex] = 0.0;
            }
            const auto predictedAngularAcceleration =
                state_.netTorqueNm / rotatingInertia;
            const auto kinematics = evaluateCylinderKinematics(config_, kinematicsReference_, cylinderIndex,
                state_.crankAngleDegrees, state_.angularVelocityRadPerSecond,
                predictedAngularAcceleration);
            const auto chamberVolume = kinematics.chamberVolumeLitres;
            cylinderGas_[cylinderIndex].setVolumeAdiabatic(chamberVolume);
            valveTrainResults_[cylinderIndex] = ValveTrainModel::evaluate(*cams.config, cams.highProfile,
                valveTrainStates_[cylinderIndex], cyclePhase, state_.rpm, state_.load, subDt);
            const auto& valveTrain = valveTrainResults_[cylinderIndex];
            const auto requestedSparkPhase = std::fmod(720.0 - ecuCommand.ignitionAdvanceDegrees
                + cylinder.ignitionOffsetDegrees + 720.0, 720.0);
            // A real ECU commits a spark event ahead of the target tooth. The
            // former code moved the crossing target at every solver substep;
            // under a loaded acceleration, a small advance change could move
            // that target across the crank between two evaluations, producing
            // no event for the entire cylinder cycle. Latch it at the start of
            // compression (phase 540) and consume it once near firing TDC.
            //
            // The schedule is carried as a REMAINING CRANK DISTANCE, not as an
            // absolute phase to be crossed again. A phase target cannot express
            // a spark that falls after the cycle boundary, and the ECU can
            // legitimately command one: `minimumIgnitionAdvanceDegrees` is
            // -10 deg, which heavy knock retard (`knockLevel * 12`) or an
            // over-temperature pull (`coolant > 108 C`) reaches. Such an event
            // lands at phase [0, 10) -- outside the (540, 720] arc the latch
            // covers -- and the boundary reset below then discarded it, turning
            // a retarded spark into a complete misfire. A countdown has no arc.
            //
            // It also cannot leak into the following cycle, which is what the
            // boundary reset existed to prevent: the reachable distance is
            // bounded by the validated ranges (advance [-10, 55], per-cylinder
            // ignition offset [-30, 30]) to [95, 220] deg, and the clamp below
            // keeps even a malformed configuration inside the same power
            // stroke.
            constexpr double sparkScheduleMaximumTravelDegrees = 260.0;
            if (crossedPhase(previousPhase, cyclePhase, 540.0)) {
                scheduledSparkPhaseDegrees_[cylinderIndex] =
                    requestedSparkPhase;
                // Measured from the CURRENT phase, not from 540: the crank has
                // already moved past 540 inside this substep, and counting from
                // the trigger angle would spend that travel twice.
                sparkScheduleTravelRemainingDegrees_[cylinderIndex] =
                    std::clamp(std::fmod(requestedSparkPhase - cyclePhase
                            + 720.0, 720.0),
                        0.0, sparkScheduleMaximumTravelDegrees);
                sparkScheduleArmed_[cylinderIndex] = true;
            } else if (sparkScheduleArmed_[cylinderIndex]) {
                // `forwardPhaseDegrees` wraps, so a momentarily REVERSING crank
                // -- start kickback is real here -- reports ~720 deg of forward
                // travel for a small backward step. A crossing test shrugs that
                // off; an accumulator would not, and would fire the schedule at
                // once. One substep can never legitimately advance half a
                // cycle, so anything that large is the wrap artefact.
                constexpr double maximumCreditedSubstepTravelDegrees = 180.0;
                const auto travel =
                    forwardPhaseDegrees(previousPhase, cyclePhase);
                if (travel <= maximumCreditedSubstepTravelDegrees)
                    sparkScheduleTravelRemainingDegrees_[cylinderIndex] -=
                        travel;
            }
            const auto sparkPhase =
                scheduledSparkPhaseDegrees_[cylinderIndex];
            const auto sparkCrossed = sparkScheduleArmed_[cylinderIndex]
                && sparkScheduleTravelRemainingDegrees_[cylinderIndex] <= 0.0;
            const auto injectionStartCrossed = crossedPhase(previousPhase, cyclePhase,
                config_.injection.startAngleDegrees);
            if (injectionStartCrossed) {
                if (compressionIgnitionEngine) {
                    combustionCycleMultiplier_[cylinderIndex] =
                        CombustionCycleVariation::advance(
                            config_.combustionCalibration
                                .cycleVariationCoefficientOfVariation,
                            config_.combustionCalibration
                                .cycleVariationCorrelation,
                            combustionVariationRandomState_[cylinderIndex],
                            combustionVariationNormalisedState_[cylinderIndex]);
                    meteredFuelMolesLastCycle_[cylinderIndex] =
                        injectedFuelMolesThisCycle_[cylinderIndex];
                    deliveredFuelMolesLastCycle_[cylinderIndex] =
                        entrainedFuelMolesThisCycle_[cylinderIndex];
                    injectorCapacityRatio_[cylinderIndex] =
                        injectorDutyHeadroom(cylinderIndex);
                    CompressionIgnitionModel::beginCycle(
                        compressionIgnitionStates_[cylinderIndex]);
                    compressionIgnitionResults_[cylinderIndex] = {};
                    cylinderMisfires_[cylinderIndex] = false;
                }
                injectedFuelMolesThisCycle_[cylinderIndex] = 0.0;
                entrainedFuelMolesThisCycle_[cylinderIndex] = 0.0;
                requestedFuelMolesThisCycle_[cylinderIndex] = 0.0;
                commandedFuelMolesMaxThisCycle_[cylinderIndex] = 0.0;
                injectorOpenSubsteps_[cylinderIndex] = 0.0;
                injectorWindowSubsteps_[cylinderIndex] = 0.0;
            }
            const auto oxygenEquivalentAirMassMg = cylinderGas_[cylinderIndex].mixture().oxygenMoles
                / 0.21 * GasCell::airMolarMassKg * 1.0e6;
            const auto& chargeSource = decoupleSharedVolumes
                ? parallelState->frozenPlenum[intakePathIndex]
                : intakePlenumGas_[intakePathIndex];
            const auto predictedPortChargeMassMg = TransientChargeEstimator::estimateFreshAirMassMg(
                oxygenEquivalentAirMassMg,
                {
                    trappedAirMassMgLastCycle_[cylinderIndex],
                    trappedAirSourcePressureKpaLastCycle_[cylinderIndex],
                    trappedAirSourceTemperatureKLastCycle_[cylinderIndex],
                },
                chargeSource.pressureKpa(), chargeSource.temperatureK());
            const auto measuredChargeMassMg = config_.injection.mode == InjectionMode::direct
                ? oxygenEquivalentAirMassMg
                : predictedPortChargeMassMg;
            // Fuel adaptation belongs to an operating region, not to an entire
            // cylinder. Blend two learned cells using the smoothed physical
            // throttle so the command remains continuous through a tip-in/out.
            if (state_.throttle > 0.10
                    && !highLoadClosedLoopFuelTrimSeeded_[cylinderIndex]) {
                highLoadClosedLoopFuelTrim_[cylinderIndex] =
                    closedLoopFuelTrim_[cylinderIndex];
                highLoadClosedLoopFuelTrimSeeded_[cylinderIndex] = true;
            }
            const auto highLoadTrimBlend = std::clamp(
                (state_.throttle - 0.10) / 0.15, 0.0, 1.0);
            const auto activeClosedLoopFuelTrim = std::lerp(
                closedLoopFuelTrim_[cylinderIndex],
                highLoadClosedLoopFuelTrim_[cylinderIndex],
                highLoadTrimBlend);
            const auto dieselFuelDemand = compressionIgnitionEngine
                ? std::clamp(std::max(state_.throttle,
                    ecuCommand.idleAirOpening * 0.20), 0.0, 1.0)
                : 1.0;
            const auto targetAirFuelRatio = compressionIgnitionEngine
                ? std::max(config_.fuelProperties.stoichiometricAirFuelRatio
                        * 1.16,
                    ecuCommand.targetAirFuelRatio)
                : std::clamp(ecuCommand.targetAirFuelRatio, 5.0, 30.0);
            auto physicalFuelTargetMoles = ecuCommand.fuelEnabled
                ? measuredChargeMassMg / targetAirFuelRatio
                    * dieselFuelDemand
                    * ecuCommand.fuelCorrection
                    * (compressionIgnitionEngine
                        ? 1.0 : activeClosedLoopFuelTrim)
                    * 1.0e-6 / fuelMolarMassKg
                : 0.0;
            if (compressionIgnitionEngine && ecuCommand.fuelEnabled) {
                // The AFR map is a rich-side smoke boundary. Transient fuel
                // correction and the quantity map may request less fuel, never
                // more than this oxygen-derived ceiling.
                const auto smokeLimitMoles = measuredChargeMassMg
                    / targetAirFuelRatio * 1.0e-6 / fuelMolarMassKg;
                const auto quantityLimitMoles =
                    dieselFullLoadFuelLimitMg
                    * dieselFuelDemand * 1.0e-6 / fuelMolarMassKg;
                physicalFuelTargetMoles = std::min(
                    physicalFuelTargetMoles,
                    std::min(smokeLimitMoles, quantityLimitMoles));
            }
            requestedFuelMolesThisCycle_[cylinderIndex] = std::max(
                requestedFuelMolesThisCycle_[cylinderIndex], physicalFuelTargetMoles);
            const auto requestedFuelMoles = requestedFuelMolesThisCycle_[cylinderIndex];
            // Port injection sprays at the valve, which is now the inlet cell
            // of the 1-D runner duct. FuelInjectionModel keeps its GasCell
            // interface, so it meters against a scratch cell synchronised to
            // the port state (it only reads pressure and temperature from the
            // target); the vaporised fuel and its charge cooling are then
            // deposited into the duct cell as a conservative species source.
            const auto portInjection = config_.injection.mode == InjectionMode::port;
            GasCell portInjectionScratch;
            auto portFuelMoles = 0.0;
            if (portInjection) {
                const auto& runnerDuct = intakeRunnerNetworks_[cylinderIndex]->ducts().front();
                const auto portPrimitives = runnerDuct.cellPrimitives();
                const auto portPressureKpa = portPrimitives.empty()
                    ? intakeRunnerPressureKpa_[cylinderIndex]
                    : portPrimitives.front().pressurePa * 0.001;
                const auto portTemperatureK = portPrimitives.empty()
                    ? intakePortTemperatureK_[cylinderIndex]
                    : portPrimitives.front().temperatureK;
                portInjectionScratch.initialise(portPressureKpa, 0.1, portTemperatureK);
                portFuelMoles = runnerDuct.inventory().speciesMassKg[
                        static_cast<std::size_t>(gasdynamics::GasSpecies::fuel)]
                    / fuelMolarMassKg;
            }
            auto& injectionTarget = portInjection
                ? portInjectionScratch : cylinderGas_[cylinderIndex];
            double commandedFuelMoles = 0.0;
            // An injector fires on crank-synchronised pulses; a stopped crank
            // freezes the cycle phase, and if it freezes INSIDE the injection
            // window the phase test alone models a stuck-open injector
            // dribbling into a dead engine. Measured on the zero-lift Core
            // case: the dribble's vapour and charge cooling kept the resolved
            // runner ringing against the plenum at 0 rpm (port temperature
            // swinging 14-39 degC), holding the manifold off ambient. No crank
            // signal, no pulse.
            // Preserve the established combustion gate except for the one
            // deliberate wet strategies: authored spark-cut overrun or the
            // opt-in wet limiter. Treating every fuel-enabled/spark-disabled
            // cycle as injectable also wets ordinary cranking cuts; the
            // large-inertia Big Twin then floods before the dyno start can
            // catch. Explicit ECU bits keep both audio strategies physical
            // without changing start behaviour elsewhere.
            if ((combustion.combustionEnabled
                    || ecuCommand.overrunAfterfireActive
                    || ecuCommand.wetSparkCutActive)
                && state_.rpm > 20.0
                && phaseInsideWindow(cyclePhase,
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
                // connects the chamber and runner (including reversion) — OR
                // when the last cycle MISFIRED. A misfired chamber keeps its
                // unburned air and its unburned fuel together, and the charge
                // estimator's resolved-oxygen floor deliberately counts that
                // retained air into the next request (it is a real anti-stall
                // enrichment after a lean cycle — see TransientChargeEstimator).
                // Counting the air while hiding the fuel is the asymmetry that
                // flooded the V12 at idle catch into an AFR-3 misfire lock-in;
                // after a misfire the chamber fuel must count symmetrically.
                const auto trappedCylinderFuel = portInjection
                    && (valveTrain.intakeLiftMm > 0.01 || cylinderMisfires_[cylinderIndex])
                    ? cylinderGas_[cylinderIndex].mixture().fuelMoles : 0.0;
                const auto crankDegreesToSpark = std::fmod(sparkPhase - cyclePhase + 720.0, 720.0);
                const auto secondsToSpark = state_.rpm > 20.0
                    ? crankDegreesToSpark / (state_.rpm * 6.0) : 0.25;
                const auto filmTemperatureFactor = std::clamp(
                    (injectionTarget.temperatureK() - 240.0) / 120.0, 0.08, 2.0);
                const auto filmAvailableFraction = 1.0 - std::exp(-secondsToSpark
                    * filmTemperatureFactor
                    / std::max(1.0e-4, config_.injection.vaporisationTimeConstantSeconds));
                if (compressionIgnitionEngine) {
                    // Diesel injection is a metered mass per cycle. Fuel which
                    // starts burning while the pulse is still open must not be
                    // replaced, otherwise combustion rate feeds back into
                    // injected quantity and creates fuel from controller logic.
                    commandedFuelMoles = std::max(0.0,
                        requestedFuelMoles
                            - injectedFuelMolesThisCycle_[cylinderIndex]);
                } else {
                    const auto existingFuelInventory = (portInjection
                            ? portFuelMoles : injectionTarget.mixture().fuelMoles)
                        + (portInjection
                            ? injectionStates_[cylinderIndex].liquidFilmMoles
                                * filmAvailableFraction
                                + trappedCylinderFuel
                            : 0.0);
                    const auto fuelInventoryDeficit = std::max(
                        0.0, requestedFuelMoles - existingFuelInventory);
                    if (portInjection) {
                        // `requestedFuelMoles` is the fuel required in the
                        // trapped charge, while the injector meters liquid.
                        // A fresh port-injection pulse is not fully available:
                        // (1-X) vaporises immediately and only the fraction
                        // below of X can leave the wall film before spark.
                        //
                        // The old code commanded the charge deficit itself.
                        // After DFCO had emptied the film this guaranteed a
                        // lean first cycle by exactly the unavailable wetting
                        // fraction. Steady closed-loop trim could hide it, but
                        // no feedback controller can repair that first pulse.
                        const auto wallFilmFraction = std::clamp(
                            config_.injection.wallFilmFraction, 0.0, 0.98);
                        const auto newPulseAvailableFraction = std::max(
                            0.02, (1.0 - wallFilmFraction)
                                + wallFilmFraction * filmAvailableFraction);
                        commandedFuelMoles =
                            fuelInventoryDeficit / newPulseAvailableFraction;
                    } else {
                        commandedFuelMoles = fuelInventoryDeficit;
                    }
                }
            }
            // The largest single-substep command is the whole new pulse the
            // injector must place before the window closes; delivery is metered
            // per substep at the injector's flow limit, so a healthy injector
            // reaches it and a saturated one does not.
            commandedFuelMolesMaxThisCycle_[cylinderIndex] = std::max(
                commandedFuelMolesMaxThisCycle_[cylinderIndex], commandedFuelMoles);
            const auto injectionResult = FuelInjectionModel::deliver(config_.injection,
                config_.fuelProperties, injectionStates_[cylinderIndex], injectionTarget,
                commandedFuelMoles, subDt);
            // Duty accounting. Every sub-step reaching here is inside the
            // angular window. Preserve the fractional final opening instead of
            // rounding every non-zero pulse up to one complete sub-step.
            injectorWindowSubsteps_[cylinderIndex] += 1.0;
            injectorOpenSubsteps_[cylinderIndex] += injectionResult.openFraction;
            if (portInjection && injectionResult.vaporisedMoles > 0.0) {
                // Each cylinder owns its runner network, so this write is as
                // cylinder-private as the old runner-cell injection was.
                if (!intakeRunnerNetworks_[cylinderIndex]->injectSpeciesAtPort(0,
                        gasdynamics::GasSpecies::fuel,
                        injectionResult.vaporisedMoles * fuelMolarMassKg,
                        config_.injection.fuelTemperatureC + 273.15,
                        -injectionResult.chargeCoolingJoules))
                    intakeInjectionRefused[cylinderIndex] = 1;
            }
            injectedFuelMolesThisCycle_[cylinderIndex] += injectionResult.meteredMoles;
            entrainedFuelMolesThisCycle_[cylinderIndex] +=
                injectionResult.entrainedMoles;
            contribution.meteredFuelMassKg += injectionResult.meteredMoles * fuelMolarMassKg;
            if (compressionIgnitionEngine) {
                flameEvents_[cylinderIndex] = {};
                ignitionPending_[cylinderIndex] = false;
                ignitionDelayRemainingSeconds_[cylinderIndex] = 0.0;
                fuelDeliveryRatio_[cylinderIndex] =
                    requestedFuelMoles > 1.0e-15
                    ? std::clamp(entrainedFuelMolesThisCycle_[cylinderIndex]
                        / requestedFuelMoles, 0.0, 1.0)
                    : 0.0;
                injectorCapacityRatio_[cylinderIndex] =
                    injectorDutyHeadroom(cylinderIndex);
                const auto deliveredFuelMassMg =
                    injectedFuelMolesThisCycle_[cylinderIndex]
                    * fuelMolarMassKg * 1.0e6;
                const auto cycleFreshAirMassMg = std::max(
                    trappedAirMassMgLastCycle_[cylinderIndex],
                    oxygenEquivalentAirMassMg);
                actualAfrLastCycle_[cylinderIndex] =
                    deliveredFuelMassMg > 1.0e-12
                    ? cycleFreshAirMassMg / deliveredFuelMassMg
                    : 100.0;
            } else if (!sparkCombustionAvailable) {
                cylinderMisfires_[cylinderIndex] = false;
                flameEvents_[cylinderIndex] = {};
                ignitionPending_[cylinderIndex] = false;
                ignitionDelayRemainingSeconds_[cylinderIndex] = 0.0;
            } else if (sparkCrossed) {
                ++commandedSparkEventsThisCycle_[cylinderIndex];
                commandedSparkPhaseThisCycle_[cylinderIndex] = sparkPhase;
                combustionCycleMultiplier_[cylinderIndex] =
                    CombustionCycleVariation::advance(
                        config_.combustionCalibration
                            .cycleVariationCoefficientOfVariation,
                        config_.combustionCalibration
                            .cycleVariationCorrelation,
                        combustionVariationRandomState_[cylinderIndex],
                        combustionVariationNormalisedState_[cylinderIndex]);
                endGasKnockStates_[cylinderIndex] = {};
                meteredFuelMolesLastCycle_[cylinderIndex] = injectedFuelMolesThisCycle_[cylinderIndex];
                const auto chamberFuelMoles = cylinderGas_[cylinderIndex].mixture().fuelMoles;
                deliveredFuelMolesLastCycle_[cylinderIndex] = chamberFuelMoles;
                fuelDeliveryRatio_[cylinderIndex] = requestedFuelMoles > 1.0e-15
                    ? std::clamp(chamberFuelMoles / requestedFuelMoles, 0.0, 1.0) : 0.0;
                // Fraction of the requested charge fuel the injector DID
                // place before its window shut. Unlike `fuelDeliveryRatio` it
                // does not carry the closed-loop trim in its reference -- the
                // trim scales the request and the delivery alike -- so a
                // well-regulated engine reads ~1.0 and only a genuinely
                // undersized injector, or a pulse wider than its window, falls
                // short. See `injectorOpenSubsteps_` for why the two earlier
                // formulations measured neither.
                injectorCapacityRatio_[cylinderIndex] =
                    injectorDutyHeadroom(cylinderIndex);
                const auto mixtureAfr = airFuelRatioForCell(cylinderGas_[cylinderIndex], config_.fuelProperties);
                actualAfrLastCycle_[cylinderIndex] = mixtureAfr;
                // Closed-loop lambda correction is based on the mixture that
                // actually reached the chamber, one cycle after injection.
                // It compensates port-film and runner transport losses without
                // fabricating fuel or altering the pressure-derived work.
                //
                // The observation is NOT valid during deceleration fuel cut.
                // The request is then zero but residual runner/chamber fuel
                // keeps producing an AFR; integrating that value drove the
                // learned trim from 0.80 to its 0.55 lower stop before the
                // injectors resumed. Production ECUs suspend lambda learning
                // in DFCO for the same observability reason. Transient-fuel
                // operation remains learnable here because the current start
                // model relies on the measured chamber mixture to converge its
                // initially dry port film; only a zero request is unobservable.
                const auto observableFuelCommand = ecuCommand.fuelEnabled
                    && requestedFuelMoles > 1.0e-15;
                if (observableFuelCommand && state_.rpm > 450.0
                    && mixtureAfr > 4.0 && mixtureAfr < 40.0) {
                    const auto targetAfr = std::clamp(
                        ecuCommand.targetAirFuelRatio, 5.0, 30.0);
                    // Attribute the observation to each cell in proportion to
                    // the authority it actually held over the command that
                    // produced it -- the same blend the command was built
                    // from, a few lines above. This must be the SAME weight:
                    // a cell that sets the fuel must answer for the mixture.
                    //
                    // The previous rule ("an observation in the interpolation
                    // band cannot identify which cell owns its error, so
                    // retain both") left a learning hole exactly over the
                    // authority ramp: the blend hands the high-load cell 0 to
                    // 100 % of the command between throttle 0.10 and 0.25,
                    // while neither cell learned anywhere inside that band.
                    // Worse, the ramp REACHES full authority at the same 0.25
                    // where learning was supposed to begin, and `state_.throttle`
                    // is the smoothed physical opening, so a command held at
                    // 0.25 approaches the test from below and can sit there
                    // indefinitely: full authority, zero adaptation.
                    //
                    // Measured on the Flat-6 free-revving at a fixed 0.25
                    // opening, before this change: trim pinned at 0.586 (its
                    // clamp floor is 0.55) while the engine ran AFR 20.9
                    // against a commanded 13.4 -- the controller removing 41 %
                    // of the fuel from an engine already starving, 81 % misfire,
                    // and net torque swinging -190 to +250 Nm. That torque
                    // spike is the audible "bop" reported from the application,
                    // and 0.25 is not a coincidence in the report: it is this
                    // constant. The diesel was the only catalogue engine free
                    // of it because compression ignition bypasses this trim
                    // entirely (see the `dieselFuelDemand` branch above).
                    //
                    // Proportional attribution is the textbook answer to the
                    // credit-assignment objection, not a threshold nudge: it
                    // is a normalised least-mean-squares update, it converges
                    // to the same fixed point, and it is identical to the old
                    // behaviour at both extremes (blend 0 updates only the low
                    // cell at full rate, blend 1 only the high cell).
                    const auto trimAuthorityBlend = std::clamp(
                        (state_.throttle - 0.10) / 0.15, 0.0, 1.0);
                    if (trimAuthorityBlend < 1.0) {
                        closedLoopFuelTrim_[cylinderIndex] = std::lerp(
                            closedLoopFuelTrim_[cylinderIndex],
                            FuelInjectionModel::updateClosedLoopTrim(
                                config_.injection, state_.rpm, mixtureAfr,
                                targetAfr, closedLoopFuelTrim_[cylinderIndex]),
                            1.0 - trimAuthorityBlend);
                    }
                    if (trimAuthorityBlend > 0.0) {
                        highLoadClosedLoopFuelTrim_[cylinderIndex] = std::lerp(
                            highLoadClosedLoopFuelTrim_[cylinderIndex],
                            FuelInjectionModel::updateClosedLoopTrim(
                                config_.injection, state_.rpm, mixtureAfr,
                                targetAfr,
                                highLoadClosedLoopFuelTrim_[cylinderIndex]),
                            trimAuthorityBlend);
                    }
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
                    + std::max(0.0, 380.0 - state_.rpm) / 1'400.0,
                    0.0, 0.92);
                // Do not add fuelDeliveryRatio as a second mixture penalty.
                // It compares chamber inventory with the current command and
                // legitimately sits below one on a port-injected transient;
                // the mixture above is the fuel and oxygen the flame actually
                // sees. On the boosted I5 it read ~0.48 while every cylinder
                // was at AFR 12.0-13.2 and the injector retained ~62% duty
                // headroom, yet the duplicate term invented simultaneous WOT
                // misfires. Genuine under-delivery already raises mixtureAfr
                // and therefore both mixtureError and flammabilityPenalty.
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
                    state_.load, config_.combustionCalibration.residualDilutionSensitivity,
                    config_.combustionCalibration.chamberTurbulenceIntensityRatio
                        * combustionCycleMultiplier_[cylinderIndex],
                    config_.combustionCalibration.ignitionSiteCount };
                ignitionPending_[cylinderIndex] = !cylinderMisfires_[cylinderIndex];
                ignitionDelayRemainingSeconds_[cylinderIndex] = ignitionPending_[cylinderIndex]
                    ? FlamePhysicsModel::ignitionDelaySeconds(config_.combustionCalibration, flameConditions) : 0.0;
                flameEvents_[cylinderIndex] = {};
            }
            if (sparkCrossed)
                sparkScheduleArmed_[cylinderIndex] = false;
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
                state_.load, config_.combustionCalibration.residualDilutionSensitivity,
                config_.combustionCalibration.chamberTurbulenceIntensityRatio
                    * combustionCycleMultiplier_[cylinderIndex],
                config_.combustionCalibration.ignitionSiteCount };
            if (compressionIgnitionEngine) {
                const auto wasAutoIgnited =
                    compressionIgnitionStates_[cylinderIndex].autoIgnited;
                auto& compressionResult =
                    compressionIgnitionResults_[cylinderIndex];
                auto cycleCalibration = config_.combustionCalibration;
                cycleCalibration.compressionIgnitionMixingTimeSeconds /=
                    std::max(0.55,
                        combustionCycleMultiplier_[cylinderIndex]);
                compressionResult = CompressionIgnitionModel::advance(
                    compressionIgnitionStates_[cylinderIndex],
                    cylinderGas_[cylinderIndex], config_.fuelProperties,
                    cycleCalibration,
                    { cyclePhase,
                      2.0 * cylinder.strokeMm * 0.001 * state_.rpm / 60.0,
                      compressionIgnitionAvailable },
                    combustionDt);
                if (!wasAutoIgnited && compressionResult.autoIgnited) {
                    residualGasFractionAtSpark_[cylinderIndex] =
                        flameConditions.burnedGasFraction;
                    equivalenceRatioAtSpark_[cylinderIndex] =
                        flameConditions.equivalenceRatio;
                }
                const auto burnAdvance =
                    compressionIgnitionStates_[cylinderIndex]
                            .cycleFuelReferenceMoles > 1.0e-15
                    ? compressionResult.reaction.burnedFuelMoles
                        / compressionIgnitionStates_[cylinderIndex]
                            .cycleFuelReferenceMoles
                    : 0.0;
                instantaneousCombustionPulse_[cylinderIndex] =
                    phaseTravel > 1.0e-6
                    ? std::clamp(burnAdvance / phaseTravel * 90.0,
                                 0.0, 4.5)
                    : 0.0;
                contribution.releasedEnergyJoules +=
                    compressionResult.reaction.releasedEnergyJoules;
                endGasKnockStates_[cylinderIndex] = {};
            } else {
                if (ignitionPending_[cylinderIndex]) {
                    ignitionDelayRemainingSeconds_[cylinderIndex] -= combustionDt;
                    if (ignitionDelayRemainingSeconds_[cylinderIndex] <= 0.0) {
                        const auto burnableFuelMoles = std::min(cylinderGas_[cylinderIndex].mixture().fuelMoles,
                            cylinderGas_[cylinderIndex].mixture().oxygenMoles
                                / config_.fuelProperties.oxygenMolesPerFuelMole);
                        flamePhysics_.ignite(flameEvents_[cylinderIndex], config_.fuelProperties,
                                             flameConditions, burnableFuelMoles);
                        ++completedIgnitionEventsThisCycle_[cylinderIndex];
                        completedIgnitionPhaseThisCycle_[cylinderIndex] =
                            cyclePhase;
                        residualGasFractionAtSpark_[cylinderIndex] =
                            flameConditions.burnedGasFraction;
                        equivalenceRatioAtSpark_[cylinderIndex] =
                            flameConditions.equivalenceRatio;
                        ignitionPending_[cylinderIndex] = false;
                    }
                }
                const auto flameStep = flamePhysics_.advance(
                    flameEvents_[cylinderIndex], config_.fuelProperties,
                    flameConditions, combustionDt);
                const auto burnAdvance = flameStep.burnedFractionAdvance;
                const auto pulse = phaseTravel > 1.0e-6
                    ? std::clamp(burnAdvance / phaseTravel * 90.0,
                                 0.0, 3.5)
                    : 0.0;
                instantaneousCombustionPulse_[cylinderIndex] =
                    pulse * fuelDeliveryRatio_[cylinderIndex];
                if (burnAdvance > 0.0 && sparkCombustionAvailable
                        && !cylinderMisfires_[cylinderIndex]) {
                    const auto reaction =
                        ConservativeGasSystem::reactFuelMoles(
                            cylinderGas_[cylinderIndex],
                            flameEvents_[cylinderIndex]
                                .initialBurnableFuelMoles * burnAdvance,
                            flameStep.efficiency,
                            config_.fuelProperties.lowerHeatingValueMjPerKg
                                * 1'000'000.0);
                    contribution.releasedEnergyJoules +=
                        reaction.releasedEnergyJoules;
                }

                const auto endGas = EndGasKnockModel::advance(
                    endGasKnockStates_[cylinderIndex], {
                        cylinderGas_[cylinderIndex].pressureKpa() / 100.0,
                        cylinderGas_[cylinderIndex].temperatureK(),
                        flameConditions.equivalenceRatio,
                        flameEvents_[cylinderIndex].burnedFraction,
                        config_.octaneRating,
                        sparkCombustionAvailable
                            && flameEvents_[cylinderIndex].active
                            && !cylinderMisfires_[cylinderIndex] },
                    combustionDt);
                if (endGas.autoIgnited
                        && endGas.autoIgnitedFuelFraction > 0.0) {
                    const auto reaction =
                        ConservativeGasSystem::reactFuelMoles(
                            cylinderGas_[cylinderIndex],
                            flameEvents_[cylinderIndex]
                                    .initialBurnableFuelMoles
                                * endGas.autoIgnitedFuelFraction,
                            std::max(0.72, flameStep.efficiency),
                            config_.fuelProperties.lowerHeatingValueMjPerKg
                                * 1'000'000.0);
                    contribution.releasedEnergyJoules +=
                        reaction.releasedEnergyJoules;
                    flameEvents_[cylinderIndex].burnedFraction = std::clamp(
                        flameEvents_[cylinderIndex].burnedFraction
                            + endGas.autoIgnitedFuelFraction,
                        0.0, 1.0);
                    if (flameEvents_[cylinderIndex].burnedFraction >= 0.999)
                        flameEvents_[cylinderIndex].active = false;
                }
                contribution.endGasKnockLevel = std::max(
                    contribution.endGasKnockLevel, endGas.level);
            }
            if (exhaustOpenedThisStep) {
                // A premixed flame cannot propagate into the following
                // gas-exchange strokes. Only the part of this integration
                // interval preceding EVO was allowed to release heat above.
                flameEvents_[cylinderIndex].active = false;
                ignitionPending_[cylinderIndex] = false;
                ignitionDelayRemainingSeconds_[cylinderIndex] = 0.0;
                if (compressionIgnitionEngine) {
                    cylinderMisfires_[cylinderIndex] =
                        injectedFuelMolesThisCycle_[cylinderIndex] > 1.0e-15
                        && !compressionIgnitionStates_[cylinderIndex]
                            .autoIgnited;
                    compressionIgnitionStates_[cylinderIndex].active = false;
                    compressionIgnitionResults_[cylinderIndex].active = false;
                }
            }

            auto chamberKpa = cylinderGas_[cylinderIndex].pressureKpa();
            chamberPressureBar_[cylinderIndex] = chamberKpa / 100.0;
            // Port plateaus: 0.58 and 0.51 of valve head area are the middle of
            // the production band in the literature cited on the helper. They
            // are deliberately not race-port figures -- a catalogue engine that
            // wants more says so through its camshaft flow curve.
            const auto intakeArea = effectiveValveAreaMm2(cylinder.boreMm, intakeLift,
                cylinder.intakeValveCount, cylinder.intakeValveDiameterMm,
                valveTrain.intakeDischargeCoefficient, 0.40, 0.50, 0.58);
            const auto exhaustArea = effectiveValveAreaMm2(cylinder.boreMm, exhaustLift,
                cylinder.exhaustValveCount, cylinder.exhaustValveDiameterMm,
                valveTrain.exhaustDischargeCoefficient, 0.34, 0.41,
                std::clamp(options_.exhaustMaximumHeadAreaFraction.value_or(0.51),
                    0.05, 1.5));
            const auto& cylinderIntake = intakeGeometryAt(config_, intakePathIndex);
            const auto pistonAreaM2 = std::numbers::pi * std::pow(cylinder.boreMm * 0.0005, 2.0);
            // Both areas already carry their discharge coefficient, so the
            // coefficient that travels with them is unity. Everything
            // downstream multiplies the pair, so the product is what matters.
            exhaustValveAreaM2[cylinderIndex] = exhaustArea * 1.0e-6
                * std::clamp(options_.exhaustValveAreaMultiplier.value_or(1.0),
                    0.05, 8.0);
            exhaustValveDischargeCoefficient[cylinderIndex] = 1.0;
            intakeValveAreaM2[cylinderIndex] = intakeArea * 1.0e-6
                * std::clamp(options_.intakeValveAreaMultiplier.value_or(1.0),
                    0.05, 8.0);
            intakeValveDischargeCoefficient[cylinderIndex] = 1.0;
            pistonAreaM2ForSplit[cylinderIndex] = pistonAreaM2;
            chamberVolumeLitresForWork[cylinderIndex] = chamberVolume;
            cycleBoundaryForWork[cylinderIndex] = cycleBoundaryCrossed;
            gasExchangeStrokeForWork[cylinderIndex] =
                cyclePhase >= 180.0 && cyclePhase < 540.0;
            exhaustStrokeForWork[cylinderIndex] =
                cyclePhase >= 180.0 && cyclePhase < 360.0;
            const auto plenumPressureKpa = decoupleSharedVolumes
                ? parallelState->frozenPlenum[intakePathIndex].pressureKpa()
                : intakePlenumGas_[intakePathIndex].pressureKpa();
            // Telemetry only. The resonator frequency feeds the UI and the
            // acoustic path; its former flow role — an admittance modulating a
            // plenum->runner orifice — is superseded by the resolved 1-D
            // runner, which owns the wave physics the admittance approximated.
            runnerAcousticResults_[cylinderIndex] = HelmholtzRunnerModel::advance(config_.runnerAcoustics,
                runnerAcousticStates_[cylinderIndex], cylinder, cylinderIntake,
                intakePortTemperatureK_[cylinderIndex],
                plenumPressureKpa, intakeRunnerPressureKpa_[cylinderIndex], subDt);

            // ----------------------------------------------------------------
            // Gas flow. The intake side is a resolved 1-D finite-volume runner
            // per cylinder — plenum reservoir -> duct -> valve Riemann port —
            // advanced OUTSIDE this per-cylinder pass in a symmetric split
            // around the exhaust coupling (see advanceIntakeRunners). Wall
            // friction and the aluminium wall heat exchange are owned by the
            // duct solver. This replaced a lumped runner cell plus
            // quasi-steady valve orifice whose equilibrium clamp capped the
            // fill at static manifold density: intake tuning is a WAVE
            // phenomenon, and four algebraic bias formulations on the lumped
            // topology were measured and refuted before this
            // (docs/physics-audit.md, "L'inertance de runner" and "Le
            // diagnostic architectural"). Do not re-add a pressure bias at the
            // valve: the duct carries the column's momentum as state, which is
            // the thing every bias was trying to counterfeit.
            //
            // Exhaust transfer is likewise solved by the conservative 1-D
            // network after this private-state pass.
            // ----------------------------------------------------------------
            const auto cylinderGasTemperatureK = cylinderGas_[cylinderIndex].temperatureK();
            const auto cylinderGasHeatCapacityJPerK = cylinderGas_[cylinderIndex].totalMoles()
                * cylinderGas_[cylinderIndex].molarHeatCapacityCvEffective();
            const auto wallHeatTransfer = CylinderHeatTransferModel::evaluate({
                cylinder.boreMm * 0.001,
                kinematics.pistonTravelMm * 0.001,
                2.0 * cylinder.strokeMm * 0.001 * state_.rpm / 60.0,
                cylinderGas_[cylinderIndex].pressureKpa(),
                cylinderGasTemperatureK,
                cylinderWallTemperatureC_[cylinderIndex] + 273.15,
                cylinderGasHeatCapacityJPerK,
                config_.combustionCalibration.wallHeatTransferCoefficientWPerK,
                subDt,
                intakeLift > 0.01 || exhaustLift > 0.01,
            });
            cylinderGas_[cylinderIndex].addHeatJoules(wallHeatTransfer.heatToGasJ);
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
            // Intake mass accounting and the published runner state now come
            // from the network advances outside this pass.
            const auto resolvedPulse =
                instantaneousCombustionPulse_[cylinderIndex];
            cylinderWallTemperatureC_[cylinderIndex] = smooth(cylinderWallTemperatureC_[cylinderIndex],
                config_.ambientTemperatureC + combustion.heatOutput * 175.0
                    + resolvedPulse * 95.0,
                subDt, 0.22);
            contribution.intakeRunnerPressure += intakeRunnerPressureKpa_[cylinderIndex];
            const auto breathingQuality = std::clamp(intakeFlowMgPerCycle_[cylinderIndex]
                / std::max(1.0, combustion.airMassMgPerCycle / static_cast<double>(config_.cylinders.size())),
                0.45, 1.35);
            contribution.combustionPulse += resolvedPulse
                * std::clamp(1.0 + cylinder.efficiencyOffset, 0.8, 1.2)
                * breathingQuality * fuelDeliveryRatio_[cylinderIndex];
            // One third of the connecting rod mass is a standard equivalent
            // reciprocating-mass approximation; the remainder contributes to
            // crank rotational inertia represented by EngineConfig.
            const auto massKg = (cylinder.pistonMassGrams
                + cylinder.connectingRodMassGrams / 3.0) * 0.001;
            const auto inertiaForce = massKg * kinematics.pistonAccelerationMps2;
            structuralInertiaForceN[cylinderIndex] = inertiaForce;
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
            const auto sideDirection = kinematics.crankPinXMm >= kinematics.wristPinXMm
                ? 1.0 : -1.0;
            structuralSideRatio[cylinderIndex] = rodSideRatio * sideDirection;
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
        // Run inline. The body writes only cylinder-private state, so it *could*
        // be threaded; it was, and the fork-join measured slower than this loop on
        // every engine in the catalogue. See
        // `decoupledSharedVolumeCylinderThreshold` above for the four measured
        // variants before reaching for a thread pool here again.
        for (std::size_t cylinderIndex = 0; cylinderIndex < config_.cylinders.size(); ++cylinderIndex)
            processCylinder(cylinderIndex);

        const auto requestedIntakeCouplingSeconds = std::clamp(
            options_.intakeCouplingIntervalSeconds.value_or(400.0e-6),
            0.0, 500.0e-6);
        const auto multirateIntake = requestedIntakeCouplingSeconds
            > subDt * 1.5;
        std::array<gasdynamics::ConservativeState, 32>
            intakeAveragedBoundaryState {};
        std::array<double, 32> intakeAveragedBoundaryVolumeM3 {};
        std::array<double, 32> intakeAveragedValveConductanceM2 {};
        if (multirateIntake) {
            intakeCouplingDurationSeconds_ += subDt;
            for (std::size_t index = 0; index < config_.cylinders.size(); ++index) {
                const auto boundaryState = networkStateForGasCell(cylinderGas_[index]);
                auto& stateIntegral = intakeBoundaryStateTimeIntegral_[index];
                for (std::size_t species = 0;
                     species < gasdynamics::gasSpeciesCount; ++species) {
                    stateIntegral.speciesMassDensityKgPerM3[species] +=
                        boundaryState.speciesMassDensityKgPerM3[species] * subDt;
                }
                stateIntegral.momentumDensityKgPerM2S +=
                    boundaryState.momentumDensityKgPerM2S * subDt;
                stateIntegral.totalEnergyDensityJPerM3 +=
                    boundaryState.totalEnergyDensityJPerM3 * subDt;
                intakeBoundaryVolumeTimeIntegralM3S_[index] +=
                    cylinderGas_[index].volumeM3() * subDt;
                intakeValveConductanceTimeIntegralM2S_[index] +=
                    intakeValveAreaM2[index]
                    * intakeValveDischargeCoefficient[index] * subDt;
            }
        }
        const auto intakeValveClosing = std::any_of(
            intakeCloseForTrappedAir.begin(),
            intakeCloseForTrappedAir.begin()
                + static_cast<std::ptrdiff_t>(config_.cylinders.size()),
            [](bool closing) { return closing; });
        const auto flushIntakeNetworks = !multirateIntake
            || intakeValveClosing
            || intakeCouplingDurationSeconds_
                >= requestedIntakeCouplingSeconds - 0.5 * subDt;
        const auto intakeAdvanceDurationSeconds = multirateIntake
            ? intakeCouplingDurationSeconds_ : subDt;
        if (flushIntakeNetworks) {
            for (std::size_t index = 0; index < config_.cylinders.size(); ++index) {
                if (multirateIntake) {
                    auto averagedState = intakeBoundaryStateTimeIntegral_[index];
                    for (auto& speciesDensity :
                         averagedState.speciesMassDensityKgPerM3) {
                        speciesDensity /= intakeCouplingDurationSeconds_;
                    }
                    averagedState.momentumDensityKgPerM2S /=
                        intakeCouplingDurationSeconds_;
                    averagedState.totalEnergyDensityJPerM3 /=
                        intakeCouplingDurationSeconds_;
                    intakeAveragedBoundaryState[index] = averagedState;
                    intakeAveragedBoundaryVolumeM3[index] =
                        intakeBoundaryVolumeTimeIntegralM3S_[index]
                        / intakeCouplingDurationSeconds_;
                    intakeAveragedValveConductanceM2[index] =
                        intakeValveConductanceTimeIntegralM2S_[index]
                        / intakeCouplingDurationSeconds_;
                } else {
                    intakeAveragedBoundaryState[index] =
                        networkStateForGasCell(cylinderGas_[index]);
                    intakeAveragedBoundaryVolumeM3[index] =
                        cylinderGas_[index].volumeM3();
                    intakeAveragedValveConductanceM2[index] =
                        intakeValveAreaM2[index]
                        * intakeValveDischargeCoefficient[index];
                }
            }
        }

        // The 1-D intake runners advance in a symmetric split around the
        // exhaust coupling, exactly like the lumped valve orifice they
        // replaced: half the substep before the exhaust network sees the
        // cylinders, half after. Each cylinder owns one single-duct network
        // whose ambient reservoir is its path's plenum, so cylinders still
        // interact only through the shared plenum cell, and the serial
        // cylinder order keeps every floating-point reduction fixed.
        //
        // The split is taken ONLY while a cylinder's intake valve is open. That
        // is where it earns its cost: the valve is the stiff, fast-moving
        // boundary term, and halving the step there is what keeps volumetric
        // efficiency converged. With the valve shut the runner is an isolated
        // column ringing at its own acoustic rate, well inside the mesh CFL
        // limit, and the second half-step buys almost nothing.
        //
        // Measured, LS3: collapsing the split for ALL cylinders is worth 41% of
        // the sub-step but costs 2.3% of VE and 2.6% of torque at 3,628 rpm --
        // refused, see docs/physics-audit.md. Gating it on the valve keeps the
        // resolution where that error lives.
        //
        // `openIntakeValve` reads the same `intakeValveAreaM2` array in both
        // passes, and nothing writes that array between them, so every cylinder
        // advances exactly `subDt` per sub-step whichever branch it takes. That
        // is a conservation requirement, not a nicety: a cylinder that took the
        // first pass and then failed the predicate would silently lose half a
        // sub-step of mass and energy transfer.
        //
        // `advanceIntakeRunnerFor` below is the concurrent phase: it must touch
        // nothing but state private to its own cylinder. Everything shared --
        // the plenum -- is decided before it and committed after it, in
        // cylinder order, so the result cannot depend on the schedule. The
        // scheme that picks the plenum state each cylinder reads, and the three
        // that were measured and rejected first, are described on
        // `advanceIntakeRunners` further down.
        //
        // `plenumDeltaForMouthSample` reads the plenum cell only for its
        // CONFIGURED molar masses, never for its dynamic state, so holding the
        // delta back changes nothing about the delta itself; the boundary state
        // is the entire difference.
        std::array<gasdynamics::ConservativeState, 32> plenumBoundaryState {};
        std::array<gasdynamics::ExhaustOutletFlowSample, 32> predictedPlenumDraw {};
        std::array<std::uint8_t, 32> predictedPlenumDrawValid {};
        std::array<GasInventoryDelta, 32> pendingPlenumDelta {};
        std::array<std::uint8_t, 32> intakePhaseFailed {};
        // A cylinder whose intake valve is shut on the first half takes no
        // advance at all, and must not then have a zero delta pushed into the
        // plenum: `tryApplyInventoryDelta` is not required to be a no-op on a
        // zero argument, and the previous code simply did not call it.
        std::array<std::uint8_t, 32> intakePhaseProducedDelta {};
        const auto advanceIntakeRunnerFor = [&](std::size_t index,
                                                double halfStepSeconds,
                                                bool firstHalf) noexcept {
            {
                const auto openIntakeValve = multirateIntake
                    ? intakeAveragedValveConductanceM2[index] > 0.0
                    : intakeValveAreaM2[index] > 0.0;
                const auto durationSeconds = openIntakeValve
                    ? halfStepSeconds
                    : (firstHalf ? 0.0 : halfStepSeconds * 2.0);
                if (!(durationSeconds > 0.0)) return;
                auto& network = *intakeRunnerNetworks_[index];
                const auto intakePathIndex =
                    intakePathIndexByCylinder_[index];
                const gasdynamics::CylinderValveBoundary boundary {
                    config_.cylinders[index].id,
                    multirateIntake
                        ? intakeAveragedBoundaryState[index]
                        : networkStateForGasCell(cylinderGas_[index]),
                    multirateIntake
                        ? intakeAveragedBoundaryVolumeM3[index]
                        : cylinderGas_[index].volumeM3(),
                    multirateIntake
                        ? intakeAveragedValveConductanceM2[index]
                        : intakeValveAreaM2[index],
                    multirateIntake
                        ? 1.0 : intakeValveDischargeCoefficient[index],
                };
                const gasdynamics::ExhaustAmbientBoundary plenumReservoir {
                    plenumBoundaryState[index], 1.0 };
                const auto advance = network.advance(durationSeconds,
                    std::span<const gasdynamics::CylinderValveBoundary>(&boundary, 1),
                    plenumReservoir);
                if (!advance.completed) intakePhaseFailed[index] = 1;
                const auto& exchange = network.cylinderExchanges().front();
                if (!cylinderGas_[index].tryApplyInventoryDelta(
                        cylinderDeltaForExchange(exchange, cylinderGas_[index])))
                    intakePhaseFailed[index] = 1;
                // The exchange is positive from the cylinder into the runner,
                // so the intake mass the cylinder gained is its negation.
                instantaneousIntakeTransferredMassKg[index] += -exchange.totalMassKg();
                intakeFlowMgThisCycle_[index] += -exchange.totalMassKg() * 1.0e6;
                // Fresh air on the same oxygen basis the trapped figure uses, so
                // the two divide into a trapping efficiency. At stoichiometry the
                // residual carries no oxygen, so oxygen crossing the valve is
                // fresh air in whichever direction it goes.
                deliveredAirMgThisCycle_[index] +=
                    -exchange.speciesMassKg[static_cast<std::size_t>(
                         gasdynamics::GasSpecies::oxygen)]
                    / GasCell::oxygenMolarMassKg / 0.21 * GasCell::airMolarMassKg * 1.0e6;
                // Held for the serial phase below rather than applied here: the
                // shared plenum is the one thing in this body that is not
                // cylinder-private.
                pendingPlenumDelta[index] = plenumDeltaForMouthSample(
                    network.outletSamples().front(),
                    intakePlenumGas_[intakePathIndex]);
                intakePhaseProducedDelta[index] = 1;
                // Port-side runner state for telemetry, injection metering, the
                // Helmholtz telemetry model and the acoustic intake excitation.
                intakeRunnerPressureKpa_[index] = exchange.networkPressurePa * 0.001;
                intakePortTemperatureK_[index] = exchange.networkTemperatureK;
                intakePortDensityKgPerM3_[index] = exchange.networkDensityKgPerM3;
                intakePortSpeedOfSoundMps_[index] = exchange.networkSpeedOfSoundMps;
                // The port sits at the duct inlet, where positive axial
                // velocity points from the valve toward the plenum; the column
                // velocity TOWARD the cylinder is therefore its negation.
                intakeValveColumnVelocityMps_[index] = -exchange.networkVelocityMps;
            }
        };
        // Runs `body(i)` for every cylinder, possibly concurrently. The body
        // must write only state private to `i`; see CylinderWorkerPool.
        const auto runIntakePhaseOverRange =
            [&](std::size_t begin, std::size_t end, const auto& body) noexcept {
                using BodyType = std::decay_t<decltype(body)>;
                if (!intakeWorkerPool_) {
                    for (std::size_t index = begin; index < end; ++index) body(index);
                    return;
                }
                intakeWorkerPool_->runIndexRange(begin, end,
                    [](void* context, std::size_t index) noexcept {
                        (*static_cast<const BodyType*>(context))(index);
                    },
                    const_cast<void*>(static_cast<const void*>(&body)));
            };
        // Every cylinder in one concurrent group must read ONE plenum state,
        // and choosing it is the whole difficulty of running these advances in
        // parallel. The serial loop this replaces was Gauss-Seidel: cylinder 0
        // saw the plenum untouched, cylinder 7 saw it after seven draws.
        //
        // Simply freezing the plenum -- plain Jacobi -- is not an acceptable
        // substitute. Measured, it settles the stationary manifold pressure
        // 0.62 kPa BELOW ambient and stays there; twenty times the settling
        // time recovers 0.06 kPa of it, so it is a wrong fixed point and not
        // slow relaxation. `EngineLab.Core`'s stationary-pressure invariant
        // catches it, as does `EngineLab.CatalogPhysics`.
        //
        // What is done instead: the drawdown staircase is reconstructed from a
        // SAME-INSTANT prediction of each cylinder's draw
        // (`predictOutletTransfer`). Deviations from ambient on that invariant,
        // against +0.0004 kPa for the serial reference it stands in for:
        //
        //   frozen plenum (plain Jacobi)              -0.62    kPa
        //   staircase from the PREVIOUS pass's draws   0.17    kPa, oscillating
        //   every cylinder at the midpoint draw       -0.095   kPa
        //   same-instant staircase, one-stage          -0.0109 kPa
        //   same-instant staircase, Heun, two rounds   +0.0011 kPa   shipped
        //
        // The lag result is the one worth remembering. The correction is only
        // ~0.04% of plenum mass, and lagging it by one half-sub-step still
        // installs a permanent 0.17 kPa limit cycle where the serial reference
        // is flat to 0.002 kPa -- because the terminal runner cell's acoustic
        // response time, volume/(mouth area * c), is about 89 us against a
        // 26 us lag. Nothing this pass reads may be stale.
        //
        // `intakePredictionGroupCount_` is the escape hatch if a future engine
        // needs more accuracy than the iteration gives: a group reads a plenum
        // already carrying the ACTUAL transfers of every group before it and
        // predicts only within itself, so one group per cylinder is the old
        // serial scheme exactly, at the cost of all the concurrency.
        // Applies one group's plenum transfers, serially and in cylinder order,
        // so the plenum's floating-point reduction is fixed however the phase
        // that produced them was scheduled.
        const auto commitGroup = [&](std::size_t begin, std::size_t end) noexcept {
            for (std::size_t index = begin; index < end; ++index) {
                if (intakePhaseFailed[index] != 0)
                    state_.solverResolutionLimited = true;
                if (intakePhaseProducedDelta[index] == 0) continue;
                const auto path = intakePathIndexByCylinder_[index];
                if (!intakePlenumGas_[path].tryApplyInventoryDelta(
                        pendingPlenumDelta[index]))
                    state_.solverResolutionLimited = true;
            }
        };
        const auto advanceIntakeRunners = [&](double halfStepSeconds,
                                              bool firstHalf) noexcept {
            const auto cylinderCount = config_.cylinders.size();
            for (std::size_t index = 0; index < cylinderCount; ++index) {
                intakePhaseFailed[index] = 0;
                intakePhaseProducedDelta[index] = 0;
            }
            // Sub-rated duct wall exchange, triggered on the SECOND half-step
            // because that is the only pass where every cylinder is
            // dispatched -- the first half advances only the runners whose
            // intake valve is open. Firing it here keeps the burst on one
            // shared dispatch instead of scattering it across cylinders,
            // which is what a network timing itself would do.
            if (!firstHalf) {
                intakeWallHeatPendingSeconds_ += halfStepSeconds * 2.0;
                if (intakeWallHeatPendingSeconds_
                        >= intakeWallHeatUpdateIntervalSeconds_) {
                    intakeWallHeatPendingSeconds_ = 0.0;
                    for (auto& network : intakeRunnerNetworks_)
                        if (network) network->requestWallHeatUpdate();
                }
            }
            const auto groupCount = std::min(cylinderCount,
                std::max<std::size_t>(1, intakePredictionGroupCount_));
            const auto groupStride = (cylinderCount + groupCount - 1) / groupCount;
            for (std::size_t begin = 0; begin < cylinderCount; begin += groupStride) {
                const auto end = std::min(cylinderCount, begin + groupStride);
                // A group of one has no staircase to reconstruct: it reads the
                // committed plenum, which IS what the serial Gauss-Seidel loop
                // gave it. Engines too small for the worker pool fall entirely
                // into this branch and keep the original scheme exactly, at the
                // original cost -- predicting for them would be pure overhead,
                // and it measured as one (the CP2 twin lost 14% of its capacity
                // to a staircase it has no use for).
                if (end - begin <= 1) {
                    plenumBoundaryState[begin] = networkStateForGasCell(
                        intakePlenumGas_[intakePathIndexByCylinder_[begin]]);
                    advanceIntakeRunnerFor(begin, halfStepSeconds, firstHalf);
                    commitGroup(begin, end);
                    continue;
                }
                // Round 0 starts every prediction from the committed plenum.
                for (std::size_t index = begin; index < end; ++index)
                    plenumBoundaryState[index] =
                        networkStateForGasCell(
                            intakePlenumGas_[intakePathIndexByCylinder_[index]]);
                // The staircase is a fixed point: cylinder i should read the
                // plenum after the cylinders before it drew, but how much they
                // draw depends on what THEY read. Iterating converges on it
                // fast -- each round cut the stationary-manifold deviation by
                // about an order of magnitude (-0.056 kPa after one round,
                // against 0.0004 for the serial reference).
                //
                // Phase A of each round is concurrent because a prediction
                // touches only its own duct; phase B is serial but arithmetic
                // only. Doing phase A serially instead was measured and
                // rejected: it handed Amdahl a serial term of the same order as
                // the parallel advance it exists to order, and cost the V8 a
                // third of the parallelism it had just gained.
                for (std::size_t round = 0; round < intakeStaircaseRounds_; ++round) {
                    runIntakePhaseOverRange(begin, end, [&](std::size_t index) noexcept {
                        predictedPlenumDrawValid[index] = 0;
                        const auto openIntakeValve = multirateIntake
                            ? intakeAveragedValveConductanceM2[index] > 0.0
                            : intakeValveAreaM2[index] > 0.0;
                        const auto durationSeconds = openIntakeValve
                            ? halfStepSeconds
                            : (firstHalf ? 0.0 : halfStepSeconds * 2.0);
                        if (!(durationSeconds > 0.0)) return;
                        const auto predicted =
                            intakeRunnerNetworks_[index]->predictOutletTransfer(0,
                                { plenumBoundaryState[index], 1.0 }, durationSeconds);
                        if (!predicted) return;
                        predictedPlenumDraw[index] = *predicted;
                        predictedPlenumDrawValid[index] = 1;
                    });
                    // Phase B, serial and cheap. The scratch starts from the
                    // real plenum, which already carries every earlier group's
                    // committed transfer.
                    for (std::size_t path = 0; path < intakePlenumCount_; ++path)
                        plenumStaircaseScratch_[path] = intakePlenumGas_[path];
                    for (std::size_t index = begin; index < end; ++index) {
                        const auto path = intakePathIndexByCylinder_[index];
                        plenumBoundaryState[index] =
                            networkStateForGasCell(plenumStaircaseScratch_[path]);
                        // A missing prediction just leaves the staircase where
                        // it is. It cannot break conservation: nothing outside
                        // this pre-pass reads the scratch cell, and the real
                        // plenum only ever receives real transfers.
                        if (predictedPlenumDrawValid[index] == 0) continue;
                        (void)plenumStaircaseScratch_[path].tryApplyInventoryDelta(
                            plenumDeltaForMouthSample(predictedPlenumDraw[index],
                                plenumStaircaseScratch_[path]));
                    }
                }
                runIntakePhaseOverRange(begin, end,
                    [&](std::size_t index) noexcept {
                        advanceIntakeRunnerFor(index, halfStepSeconds, firstHalf);
                    });
                commitGroup(begin, end);
            }
        };
        if (flushIntakeNetworks)
            advanceIntakeRunners(intakeAdvanceDurationSeconds * 0.5, true);

        auto configuredOutletConductanceM2 = 0.0;
        for (const auto& outlet : physicalExhaustNetwork.layout().outlets())
            configuredOutletConductanceM2 +=
                outlet.openingAreaM2 * outlet.dischargeCoefficient;
        // Published so the forced-induction block can charge the turbine its
        // real downstream back pressure instead of assuming atmosphere.
        exhaustOutletConductanceM2_ = configuredOutletConductanceM2;
        auto instantaneousOutletOpeningScale = 1.0;
        if (config_.forcedInduction.enabled
            && config_.forcedInduction.type == ForcedInductionType::turbocharger) {
            const auto turbineAreaM2 = (config_.forcedInduction.turbineFlowAreaMm2
                + state_.wastegateOpening
                    * config_.forcedInduction.wastegateFlowAreaMm2) * 1.0e-6;
            // Turbine throat and downstream system are two restrictions IN
            // SERIES, so their losses add and the effective area is the
            // reciprocal-square-sum, not the smaller of the two. The old `min`
            // made the larger one perfectly invisible. On the shipped geometry
            // the two forms differ by well under one per cent, which is the
            // honest answer: a 700 mm2 turbine throat really does dominate a
            // 4000 mm2 tailpipe, and no amount of tailpipe changes that. What
            // the downstream system does change is the pressure the turbine
            // expands INTO, which is handled with the shaft power above.
            const auto downstream = std::max(1.0e-12,
                configuredOutletConductanceM2);
            const auto seriesAreaM2 = 1.0 / std::sqrt(
                1.0 / (turbineAreaM2 * turbineAreaM2)
                + 1.0 / (downstream * downstream));
            instantaneousOutletOpeningScale =
                std::clamp(seriesAreaM2 / downstream, 0.0, 1.0);
        }
        exhaustCouplingDurationSeconds_ += subDt;
        outletOpeningScaleTimeIntegralSeconds_ +=
            instantaneousOutletOpeningScale * subDt;
        for (std::size_t index = 0; index < config_.cylinders.size(); ++index) {
            exhaustValveConductanceTimeIntegralM2S_[index] +=
                exhaustValveAreaM2[index] * exhaustValveDischargeCoefficient[index] * subDt;
            const auto boundaryState = networkStateForGasCell(cylinderGas_[index]);
            auto& stateIntegral = exhaustBoundaryStateTimeIntegral_[index];
            for (std::size_t species = 0; species < gasdynamics::gasSpeciesCount; ++species) {
                stateIntegral.speciesMassDensityKgPerM3[species] +=
                    boundaryState.speciesMassDensityKgPerM3[species] * subDt;
            }
            stateIntegral.momentumDensityKgPerM2S +=
                boundaryState.momentumDensityKgPerM2S * subDt;
            stateIntegral.totalEnergyDensityJPerM3 +=
                boundaryState.totalEnergyDensityJPerM3 * subDt;
            exhaustBoundaryVolumeTimeIntegralM3S_[index] +=
                cylinderGas_[index].volumeM3() * subDt;
        }

        // Duration-based flush, deliberately NOT forced at the end of the
        // frame: see the accumulator members in the header. The half-substep
        // bias flushes on the nearest substep to the target interval, so the
        // network's integration grid is uniform in time and free-running with
        // respect to both the frame grid and the substep grid.
        const auto flushExhaustNetwork = exhaustCouplingDurationSeconds_
            >= maximumExhaustCouplingSeconds - 0.5 * subDt;
        auto exhaustNetworkCompleted = true;
        auto exhaustAdvanceDurationSeconds = 0.0;
        auto outletMassKg = 0.0;
        auto publishExhaustAcousticState = false;
        exhaustFuelReactionScratch_ = {};
        auto& exhaustFuelReaction = exhaustFuelReactionScratch_;
        std::array<std::uint8_t, 32> exhaustExchangeApplied {};
        exhaustExchangeApplied.fill(1);
        if (flushExhaustNetwork) {
            if (options_.resetExhaustToAmbientEachCoupling.value_or(false)) {
                const auto reset = physicalExhaustNetwork.reset(
                    config_.ambientPressureKpa * 1'000.0,
                    config_.ambientTemperatureC + 273.15);
                if (!reset) state_.solverResolutionLimited = true;
            }
            std::array<gasdynamics::CylinderValveBoundary, 32> averagedBoundaries {};
            const auto cylinderPorts = physicalExhaustNetwork.layout().cylinderPorts();
            for (std::size_t portIndex = 0; portIndex < cylinderPorts.size(); ++portIndex) {
                const auto index = exhaustNetworkCylinderIndex_[portIndex];
                auto averagedState = exhaustBoundaryStateTimeIntegral_[index];
                for (auto& speciesDensity : averagedState.speciesMassDensityKgPerM3)
                    speciesDensity /= exhaustCouplingDurationSeconds_;
                averagedState.momentumDensityKgPerM2S /= exhaustCouplingDurationSeconds_;
                averagedState.totalEnergyDensityJPerM3 /= exhaustCouplingDurationSeconds_;
                averagedBoundaries[portIndex] = {
                    cylinderPorts[portIndex].cylinderId,
                    averagedState,
                    exhaustBoundaryVolumeTimeIntegralM3S_[index]
                        / exhaustCouplingDurationSeconds_,
                    exhaustValveConductanceTimeIntegralM2S_[index]
                        / exhaustCouplingDurationSeconds_,
                    1.0,
                };
            }
            const gasdynamics::ExhaustAmbientBoundary ambient {
                physicalExhaustAmbientState_,
                outletOpeningScaleTimeIntegralSeconds_ / exhaustCouplingDurationSeconds_,
            };
            const auto networkAdvance = physicalExhaustNetwork.advance(
                exhaustCouplingDurationSeconds_,
                std::span<const gasdynamics::CylinderValveBoundary>(
                    averagedBoundaries.data(), cylinderPorts.size()),
                ambient);
            exhaustNetworkCompleted = networkAdvance.completed;
            exhaustAdvanceDurationSeconds = networkAdvance.advancedTimeSeconds;
            exhaustNetworkAcceptedSubsteps += networkAdvance.acceptedSubsteps;
            exhaustNetworkAdvancedSeconds += networkAdvance.advancedTimeSeconds;
            if (!networkAdvance.completed) state_.solverResolutionLimited = true;
            // Chemistry is an operating state, not a permanent property of an
            // engine that happens to author an afterfire map. The old condition
            // ran it throughout every loaded warm-up whenever either feature
            // existed, oxidising normal trace HC and sending compact reaction
            // sources as high as the 100 kPa safety bound before the driver had
            // lifted. Closed-throttle DFCO remains true across the pulse's OFF
            // windows, so a transported slug continues reacting after injection
            // closes; a wet hard limiter likewise stays enabled across its
            // alternating spark phases.
            const auto exhaustReactionOperatingState =
                (afterfireRetainsFuel(config_.exhaustAfterfire.strategy)
                    && ecuCommand.decelerationFuelCutActive)
                || (config_.ignition.limiterKeepsFuel
                    && (ecuCommand.wetSparkCutActive
                        || ecuCommand.hardRevLimiterActive));
            if (networkAdvance.completed
                && exhaustReactionOperatingState) {
                exhaustFuelReaction = physicalExhaustNetwork.reactUnburnedFuel(
                    networkAdvance.advancedTimeSeconds,
                    {
                        config_.exhaustAfterfire.ignitionTemperatureK,
                        config_.exhaustAfterfire.reactionTimeConstantSeconds,
                        config_.exhaustAfterfire.reactionEfficiency,
                        config_.fuelProperties.oxygenMolesPerFuelMole,
                        config_.fuelProperties.molarMassGramsPerMole * 0.001,
                        config_.fuelProperties.lowerHeatingValueMjPerKg
                            * 1'000'000.0,
                        config_.exhaustAfterfire.inductionTimeSeconds,
                        config_.exhaustAfterfire.minimumEquivalenceRatio,
                        config_.exhaustAfterfire.maximumEquivalenceRatio,
                        config_.exhaustAfterfire.quenchTemperatureK,
                    });
            }

            // Boundary/media states are consumed only at the renderer's block
            // boundary (187.5 Hz at 48 kHz/256). Publishing them at every
            // ~8 kHz solver coupling rebuilt and copied the complete duct array
            // dozens of times before any consumer could observe it. Reaction
            // sources remain losslessly coupling-rate: any non-empty source
            // forces an immediate packet.
            constexpr double acousticStatePublishIntervalSeconds = 1.0 / 240.0;
            const auto projectedTimeSeconds =
                state_.simulationTimeSeconds + subDt;
            publishExhaustAcousticState = exhaustAcousticSamples_
                && (exhaustFuelReaction.sourceCount > 0
                    || lastExhaustAcousticStatePublishSeconds_ < 0.0
                    || projectedTimeSeconds
                        - lastExhaustAcousticStatePublishSeconds_
                        >= acousticStatePublishIntervalSeconds
                            - 0.5 * subDt);

            const auto cylinderExchanges = physicalExhaustNetwork.cylinderExchanges();
            for (std::size_t exchangeIndex = 0;
                 exchangeIndex < cylinderExchanges.size(); ++exchangeIndex) {
                const auto& exchange = cylinderExchanges[exchangeIndex];
                const auto index = exhaustNetworkCylinderIndex_[exchangeIndex];
                const auto applied = cylinderGas_[index].tryApplyInventoryDelta(
                    cylinderDeltaForExchange(exchange, cylinderGas_[index]));
                exhaustExchangeApplied[index] = static_cast<std::uint8_t>(applied);
                if (!applied) state_.solverResolutionLimited = true;
                exhaustFlowMgThisCycle_[index] += exchange.totalMassKg() * 1.0e6;
            }
            for (const auto& outlet : physicalExhaustNetwork.outletSamples()) {
                auto transferredMassKg = 0.0;
                for (const auto speciesMassKg : outlet.speciesMassKg)
                    transferredMassKg += speciesMassKg;
                outletMassKg += std::max(0.0, transferredMassKg);
            }
            // Publish the acoustic medium of each duct, not just of the valve.
            // The waveguide's delays, wall/liner losses and radiation state all
            // read the local gas state, and it varies by hundreds of kelvin
            // between the port and the tailpipe.
            if (publishExhaustAcousticState) {
                const auto networkDucts = physicalExhaustNetwork.ducts();
                exhaustDuctMediumCount_ = std::min(networkDucts.size(),
                    ExhaustAcousticSample::maximumDucts);
                for (std::size_t index = 0;
                     index < exhaustDuctMediumCount_; ++index) {
                    auto densityKgPerM3 = 0.0;
                    auto speedOfSoundMps = 0.0;
                    if (!networkDucts[index].meanAcousticMedium(
                            densityKgPerM3, speedOfSoundMps)) {
                        // Leave the previous sample in place rather than publish
                        // a zero the renderer would have to special-case.
                        continue;
                    }
                    exhaustDuctDensityKgPerM3_[index] =
                        static_cast<float>(densityKgPerM3);
                    exhaustDuctSpeedOfSoundMps_[index] =
                        static_cast<float>(speedOfSoundMps);
                }
            }

            exhaustValveConductanceTimeIntegralM2S_.fill(0.0);
            exhaustBoundaryStateTimeIntegral_.fill({});
            exhaustBoundaryVolumeTimeIntegralM3S_.fill(0.0);
            exhaustCouplingDurationSeconds_ = 0.0;
            outletOpeningScaleTimeIntegralSeconds_ = 0.0;
        }

        // Symmetric intake split around the exhaust coupling. This remains at
        // mechanical cadence even on substeps where the slower network state is
        // held, so valve overlap and trapped charge are never decimated.
        if (flushIntakeNetworks) {
            advanceIntakeRunners(intakeAdvanceDurationSeconds * 0.5, false);
            if (multirateIntake) {
                intakeBoundaryStateTimeIntegral_.fill({});
                intakeBoundaryVolumeTimeIntegralM3S_.fill(0.0);
                intakeValveConductanceTimeIntegralM2S_.fill(0.0);
                intakeCouplingDurationSeconds_ = 0.0;
            }
        }
        for (std::size_t index = 0; index < config_.cylinders.size(); ++index) {
            if (intakeInjectionRefused[index] != 0)
                state_.solverResolutionLimited = true;
            // The stagnation head the arriving column carries, published as a
            // diagnostic. The velocity is now the RESOLVED port state of the
            // 1-D runner — the duct carries the column's momentum, so the ram
            // this head used to approximate is delivered by the solver itself,
            // and IVC trapping is irreversible for the physical reason: the
            // wave is still arriving when the valve shuts.
            // Only a column travelling toward the cylinder carries a charging
            // head; reversion has its own (adverse) direction.
            const auto towardCylinderMps =
                std::max(0.0, intakeValveColumnVelocityMps_[index]);
            intakePortRamKpa_[index] = std::min(
                0.5 * intakePortDensityKgPerM3_[index]
                    * towardCylinderMps * towardCylinderMps * 0.001,
                config_.runnerAcoustics.maximumPressureAmplitudeKpa);
            if (intakeCloseForTrappedAir[index]) {
                trappedAirMassMgLastCycle_[index] = cylinderGas_[index]
                    .mixture().oxygenMoles / 0.21 * GasCell::airMolarMassKg * 1.0e6;
                const auto intakePathIndex =
                    intakePathIndexByCylinder_[index];
                trappedAirSourcePressureKpaLastCycle_[index] =
                    intakePlenumGas_[intakePathIndex].pressureKpa();
                trappedAirSourceTemperatureKLastCycle_[index] =
                    intakePlenumGas_[intakePathIndex].temperatureK();
            }
            const auto finalChamberPressureKpa = cylinderGas_[index].pressureKpa();
            chamberPressureBar_[index] = finalChamberPressureKpa / 100.0;
            IndicatedWorkModel::advance(indicatedWorkStates_[index],
                finalChamberPressureKpa, chamberVolumeLitresForWork[index],
                config_.ambientPressureKpa, cycleBoundaryForWork[index],
                gasExchangeStrokeForWork[index], exhaustStrokeForWork[index]);
            const auto torqueCorrection = (finalChamberPressureKpa
                    - chamberPressureBeforeNetworkKpa[index]) * 1'000.0
                * pistonAreaM2ForSplit[index] * gasTorqueLeverArmM[index];
            if (decoupleSharedVolumes)
                parallelState->scratch[index].gasIndicatedTorque += torqueCorrection;
            else
                gasIndicatedTorque += torqueCorrection;
        }

        // Publish the instantaneous valve-plane Riemann flow after both split
        // operators. This preserves signed high-rate thermoacoustic forcing even
        // though conservative low-band state exchange is deliberately multirate.
        std::array<gasdynamics::CylinderValveBoundary, 32> instantaneousBoundaries {};
        const auto cylinderPorts = physicalExhaustNetwork.layout().cylinderPorts();
        for (std::size_t portIndex = 0; portIndex < cylinderPorts.size(); ++portIndex) {
            const auto index = exhaustNetworkCylinderIndex_[portIndex];
            instantaneousBoundaries[portIndex] = {
                cylinderPorts[portIndex].cylinderId,
                networkStateForGasCell(cylinderGas_[index]),
                cylinderGas_[index].volumeM3(),
                exhaustValveAreaM2[index],
                exhaustValveDischargeCoefficient[index],
            };
        }
        std::array<gasdynamics::CylinderBoundaryFlowSample, 32> boundarySamples {};
        const auto sampled = physicalExhaustNetwork.sampleCylinderBoundaries(
            std::span<const gasdynamics::CylinderValveBoundary>(
                instantaneousBoundaries.data(), cylinderPorts.size()),
            std::span<gasdynamics::CylinderBoundaryFlowSample>(
                boundarySamples.data(), physicalExhaustNetwork.layout().cylinderPorts().size()));
        collectorPressureKpa = config_.ambientPressureKpa;
        if (!sampled) state_.solverResolutionLimited = true;
        for (std::size_t portIndex = 0;
             portIndex < physicalExhaustNetwork.layout().cylinderPorts().size(); ++portIndex) {
            const auto index = exhaustNetworkCylinderIndex_[portIndex];
            const auto& sample = boundarySamples[portIndex];
            if (sampled) {
                // The network state behind this sample is only new on a flush
                // substep. Record it as a knot of the boundary's time history
                // rather than publishing it directly: publishing it on every
                // substep would hold it flat between flushes, which is a
                // zero-order hold clocked at the coupling rate and lands in the
                // audible band. See ExhaustNetworkBoundary.
                if (flushExhaustNetwork) {
                    exhaustBoundaryKnots_[index][exhaustBoundaryKnotWrite_] = {
                        sample.networkPressurePa * 0.001,
                        sample.networkVelocityMps,
                        sample.networkTemperatureK,
                        sample.massFlowKgPerSecond,
                        sample.networkDensityKgPerM3,
                        sample.networkSpeedOfSoundMps,
                        state_.simulationTimeSeconds,
                        true,
                    };
                }
                // Reconstruct the boundary at one constant delay behind now,
                // interpolated between whichever knots bracket that instant.
                // See exhaustBoundaryKnots_ for why the delay must not follow
                // the current coupling interval. The delay exceeds the longest
                // interval (the 250 us low-speed cap plus slack), so a
                // bracketing pair exists whenever the ring holds history;
                // until then the reconstruction holds the nearest knot, which
                // only occurs while the network itself is still priming.
                const auto targetTimeSeconds = state_.simulationTimeSeconds
                    - exhaustBoundaryReconstructionDelaySeconds;
                const auto& ring = exhaustBoundaryKnots_[index];
                const auto newestSlot = flushExhaustNetwork
                    ? exhaustBoundaryKnotWrite_
                    : (exhaustBoundaryKnotWrite_ + exhaustBoundaryKnotCount - 1)
                        % exhaustBoundaryKnotCount;
                const auto* before = &ring[newestSlot];
                const auto* after = &ring[newestSlot];
                for (std::size_t age = 0; age < exhaustBoundaryKnotCount; ++age) {
                    const auto slot = (newestSlot + exhaustBoundaryKnotCount - age)
                        % exhaustBoundaryKnotCount;
                    if (!ring[slot].valid) break;
                    before = &ring[slot];
                    if (ring[slot].timeSeconds <= targetTimeSeconds) break;
                    after = &ring[slot];
                }
                const auto knotInterval = after->timeSeconds - before->timeSeconds;
                const auto phase = knotInterval > 0.0
                    ? std::clamp((targetTimeSeconds - before->timeSeconds)
                        / knotInterval, 0.0, 1.0)
                    : 1.0;
                const auto& from = *before;
                const auto& to = *after;
                const auto blend = [phase](double a, double b) {
                    return a + (b - a) * phase;
                };
                exhaustRunnerPressureKpa_[index] = blend(from.pressureKpa, to.pressureKpa);
                exhaustRunnerVelocityMps_[index] = blend(from.velocityMps, to.velocityMps);
                exhaustRunnerTemperatureK_[index] = blend(from.temperatureK, to.temperatureK);
                exhaustPortDensityKgPerM3[index] =
                    blend(from.densityKgPerM3, to.densityKgPerM3);
                exhaustPortSpeedOfSoundMps[index] =
                    blend(from.speedOfSoundMps, to.speedOfSoundMps);
                // Two mass flows, deliberately, because two consumers need
                // different things and one field cannot honestly serve both.
                //
                // The acoustic one is reconstructed with the same phase as the
                // pressure, density and sound speed above. The audio path splits
                // these into characteristics as 0.5*(p' +- Zc*U'), which only
                // means anything if p' and U' describe the same instant of the
                // same field. Blending pressure between two network knots while
                // taking flow from just the latest one leaves a residual of
                // (1 - phase) * (to.p - from.p): a sawtooth clocked at the
                // coupling rate, whose harmonics reach well above the coupling
                // Nyquist and are audible there as images.
                exhaustAcousticMassFlowKgPerSecond[index] =
                    blend(from.massFlowKgPerSecond, to.massFlowKgPerSecond);
                // The instantaneous one is recomputed from the live cylinder
                // state every substep and keeps valve-plane detail the network
                // state does not carry. Mass accounting wants exactly that, and
                // smoothing it would degrade the flow balance, so it stays raw.
                exhaustMassFlowKgPerSecond[index] = sample.massFlowKgPerSecond;
            }
            thermoacousticBoundaryValid[index] = static_cast<std::uint8_t>(sampled
                && sample.valid && exhaustNetworkCompleted
                && exhaustBoundaryKnots_[index][flushExhaustNetwork
                    ? exhaustBoundaryKnotWrite_
                    : (exhaustBoundaryKnotWrite_ + exhaustBoundaryKnotCount - 1)
                        % exhaustBoundaryKnotCount].valid
                && exhaustExchangeApplied[index] != 0);
            collectorPressureKpa = std::max(
                collectorPressureKpa, exhaustRunnerPressureKpa_[index]);
        }
        // The write cursor advances only after every port has recorded its knot
        // for this flush, so all cylinders share one time base.
        if (sampled && flushExhaustNetwork)
            exhaustBoundaryKnotWrite_ = (exhaustBoundaryKnotWrite_ + 1)
                % exhaustBoundaryKnotCount;
        for (std::size_t index = 0; index < config_.cylinders.size(); ++index) {
            if (decoupleSharedVolumes) {
                parallelState->scratch[index].exhaustRunnerPressure +=
                    exhaustRunnerPressureKpa_[index];
                parallelState->scratch[index].exhaustTemperatureK +=
                    exhaustRunnerTemperatureK_[index];
            } else {
                exhaustRunnerPressureSum += exhaustRunnerPressureKpa_[index];
                exhaustTemperatureSumK += exhaustRunnerTemperatureK_[index];
            }
        }
        // Fixed-order reduction of the per-cylinder scratch. Small engines retain
        // their original serial sum; large-engine output is independent of worker
        // scheduling because the same cylinder order is always used here.
        if (decoupleSharedVolumes) {
            for (std::size_t index = 0; index < config_.cylinders.size(); ++index)
                accumulateContribution(parallelState->scratch[index]);
            // The per-cylinder pass no longer touches the shared plenum: the
            // plenum <-> runner exchange is owned by the serial intake network
            // advances, applied in cylinder order, so the N-way Jacobi commit
            // that used to reconcile parallel plenum draws is gone with the
            // lumped orifice it reconciled.
        }

        const auto meanPistonSpeed = pistonSpeedSum / static_cast<double>(config_.cylinders.size());
        const auto displacementM3 = displacement * 0.001;
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
        // A real starter is sized against the PEAK single-cylinder compression
        // torque, which scales with per-cylinder displacement, not with total
        // displacement: a 6.5 L five-cylinder radial fights 1.3 L compression
        // peaks spaced 144 deg apart, a far harder duty than a 6.2 L V8's
        // 0.78 L peaks every 90 deg. The old total-displacement sizing was
        // implicitly calibrated against the lumped intake's under-filled
        // cranking charges; with the resolved runners filling properly, the
        // radial's first full compression pinned a 383 Nm starter at zero for
        // 3.4 s (measured: -1087 Nm at the crank, decaying only through wall
        // heat loss because the no-reverse clamp forbids the rock-back a real
        // crank uses to get over compression).
        const auto perCylinderDisplacement = displacement
            / static_cast<double>(std::max<std::size_t>(1, config_.cylinders.size()));
        const auto starterPeakTorque = 45.0 + displacement * 52.0
            + perCylinderDisplacement * 350.0;
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
        omega = std::max(0.0, omega
            + netTorque / rotatingInertia * subDt);
        state_.angularVelocityRadPerSecond = omega;
        state_.rpm = omega * 60.0 / (2.0 * std::numbers::pi);
        if (state_.rpm < 0.5 && !cranking && indicatedTorque <= 0.0) {
            state_.rpm = 0.0;
            state_.angularVelocityRadPerSecond = 0.0;
        }
        const auto subTravelled = (subPreviousRpm + state_.rpm) * 0.5 * 6.0 * subDt;
        maximumIntegratedCrankStep = std::max(maximumIntegratedCrankStep, std::abs(subTravelled));
        accumulateCycleTelemetry(subPreviousAngle, subTravelled, subDt,
                                 indicatedTorque, brakeTorque, frame);
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
        state_.targetAirFuelRatio = compressionIgnitionEngine
            ? std::max(config_.fuelProperties.stoichiometricAirFuelRatio
                    * 1.16,
                ecuCommand.targetAirFuelRatio)
            : ecuCommand.targetAirFuelRatio;
        state_.ignitionAdvanceDegrees = ecuCommand.ignitionAdvanceDegrees;
        const auto physicalAirMassMg = std::accumulate(trappedAirMassMgLastCycle_.begin(),
            trappedAirMassMgLastCycle_.begin()
                + static_cast<std::ptrdiff_t>(config_.cylinders.size()), 0.0);
        const auto ambientTemperatureK = config_.ambientTemperatureC + 273.15;
        const auto referenceAirMassMg = config_.ambientPressureKpa * 1'000.0
            * displacement * 0.001
            / (GasCell::universalGasConstant / GasCell::airMolarMassKg * ambientTemperatureK)
            * 1.0e6;
        state_.airMassMgPerCycle = state_.rpm > 20.0 ? physicalAirMassMg : 0.0;
        state_.inductedChargeMassMgPerCycle = std::accumulate(
            intakeFlowMgPerCycle_.begin(),
            intakeFlowMgPerCycle_.begin()
                + static_cast<std::ptrdiff_t>(config_.cylinders.size()), 0.0);
        state_.volumetricEfficiency = state_.rpm > 20.0
            ? std::clamp(physicalAirMassMg / std::max(1.0, referenceAirMassMg), 0.0, 2.5) : 0.0;
        const auto deliveredAirMassMg = std::accumulate(deliveredAirMgPerCycle_.begin(),
            deliveredAirMgPerCycle_.begin()
                + static_cast<std::ptrdiff_t>(config_.cylinders.size()), 0.0);
        state_.deliveredAirMassMgPerCycle = state_.rpm > 20.0 ? deliveredAirMassMg : 0.0;
        state_.deliveredVolumetricEfficiency = state_.rpm > 20.0
            ? std::clamp(deliveredAirMassMg / std::max(1.0, referenceAirMassMg), 0.0, 2.5) : 0.0;
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
        if (exhaustAdvanceDurationSeconds > 0.0) {
            const auto physicalOutletFlowGramsPerSecond = outletMassKg
                / exhaustAdvanceDurationSeconds * 1'000.0;
            state_.exhaustFlowGramsPerSecond = smooth(state_.exhaustFlowGramsPerSecond,
                physicalOutletFlowGramsPerSecond,
                exhaustAdvanceDurationSeconds, 35.0);
            state_.exhaustAfterfireHeatReleaseKw = smooth(
                state_.exhaustAfterfireHeatReleaseKw,
                exhaustFuelReaction.releasedEnergyJoules
                    / exhaustAdvanceDurationSeconds / 1'000.0,
                exhaustAdvanceDurationSeconds, 80.0);
            state_.exhaustAfterfireFuelBurnMgPerSecond = smooth(
                state_.exhaustAfterfireFuelBurnMgPerSecond,
                exhaustFuelReaction.burnedFuelMassKg
                    / exhaustAdvanceDurationSeconds * 1.0e6,
                exhaustAdvanceDurationSeconds, 80.0);
            // Unsmoothed: the wall moves on a tens-of-seconds time constant, so
            // there is nothing here for a filter to remove.
            //
            // This is a max over every exhaust cell and it sits in the SUB-STEP
            // loop, which on this project's critical thread deserves an answer
            // rather than a shrug: ~40 cells against ~160 sub-steps per frame is
            // ~6 us of a 4166 us budget, i.e. 0.15%, and the `inventory()`
            // traversal a few lines below is already more expensive. Sampling a
            // 55 s time constant at 240 Hz would be plenty, so if this block
            // ever moves somewhere cheaper, take this with it.
            state_.exhaustWallTemperatureC =
                physicalExhaustNetwork.peakWallTemperatureK() - 273.15;
        }
        // Zero when nothing reacts, rather than holding the last value. Holding
        // it reads as "the hot-surface path is carrying almost everything"
        // exactly when the answer is "nothing is burning at all", which is the
        // most misleading moment for it to say that -- a null-control run with
        // no retained fuel reported 99.5 % beside a heat release of 0.000 kW.
        state_.exhaustAfterfireWallIgnitedFraction =
            exhaustFuelReaction.reactingControlVolumes > 0
                ? static_cast<double>(
                      exhaustFuelReaction.wallIgnitedControlVolumes)
                    / static_cast<double>(
                        exhaustFuelReaction.reactingControlVolumes)
                : 0.0;
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
                + intakeRunnerNetworks_[index]->inventory().totalEnergyJ;
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
            const auto compressionIgnition =
                config_.fuel == FuelType::diesel;
            const auto active = ((compressionIgnition
                    ? compressionIgnitionStates_[index].active
                    : flameEvents_[index].active)
                || instantaneousCombustionPulse_[index] > 1.0e-9)
                && !cylinderMisfires_[index];
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
                intakeValveColumnVelocityMps_[index],
                exhaustRunnerVelocityMps_[index],
                fuelDeliveryRatio_[index],
                injectorCapacityRatio_[index],
                compressionIgnition ? 0.0 : flameEvents_[index].flameSpeedMps,
                compressionIgnition
                    ? compressionIgnitionResults_[index].burnedFraction
                    : flameEvents_[index].burnedFraction,
                compressionIgnition
                    ? compressionIgnitionResults_[index].efficiency
                    : flameEvents_[index].efficiency,
                compressionIgnition
                    ? 0.0 : endGasKnockStates_[index].filteredLevel,
                active, combustion.combustionEnabled && cylinderMisfires_[index] };
            const auto kinematics = evaluateCylinderKinematics(config_, kinematicsReference_, index,
                state_.crankAngleDegrees, state_.angularVelocityRadPerSecond,
                state_.netTorqueNm / rotatingInertia);
            auto& cylinderState = state_.cylinderStates[index];
            cylinderState.exhaustValveConductanceAreaM2 =
                exhaustValveAreaM2[index] * exhaustValveDischargeCoefficient[index];
            cylinderState.exhaustMassFlowKgPerSecond =
                exhaustMassFlowKgPerSecond[index];
            cylinderState.trappedFreshAirMassMg =
                trappedAirMassMgLastCycle_[index];
            cylinderState.deliveredFreshAirMassMgPerCycle =
                deliveredAirMgPerCycle_[index];
            cylinderState.intakeRunnerTemperatureC =
                intakePortTemperatureK_[index] - 273.15;
            cylinderState.intakeRunnerChargePressureKpa = intakeRunnerPressureKpa_[index];
            cylinderState.airFuelRatio = actualAfrLastCycle_[index];
            cylinderState.requestedFuelMgPerCycle = requestedFuelMolesThisCycle_[index]
                * config_.fuelProperties.molarMassGramsPerMole * 1'000.0;
            cylinderState.deliveredFuelMgPerCycle = deliveredFuelMolesLastCycle_[index]
                * config_.fuelProperties.molarMassGramsPerMole * 1'000.0;
            const auto highLoadTrimBlend = std::clamp(
                (state_.throttle - 0.10) / 0.15, 0.0, 1.0);
            cylinderState.closedLoopFuelTrim = std::lerp(
                closedLoopFuelTrim_[index],
                highLoadClosedLoopFuelTrim_[index],
                highLoadTrimBlend);
            cylinderState.residualGasFractionAtSpark = residualGasFractionAtSpark_[index];
            cylinderState.equivalenceRatioAtSpark = equivalenceRatioAtSpark_[index];
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
            cylinderState.intakePortRamPressureKpa = intakePortRamKpa_[index];
            cylinderState.intakePortColumnVelocityMps = intakeValveColumnVelocityMps_[index];
            cylinderState.compressionIgnition =
                compressionIgnition
                && compressionIgnitionResults_[index].autoIgnited;
            cylinderState.combustionStartPhaseDegrees =
                compressionIgnition
                ? compressionIgnitionResults_[index].startPhaseDegrees
                : -state_.ignitionAdvanceDegrees
                    + cylinder.ignitionOffsetDegrees;
            cylinderState.combustionDurationMs =
                compressionIgnition
                ? compressionIgnitionResults_[index].durationSeconds * 1'000.0
                : (flameEvents_[index].flameSpeedMps > 0.01
                    ? std::clamp(cylinder.boreMm * 0.5
                        / flameEvents_[index].flameSpeedMps, 1.0, 45.0)
                    : 0.0);
            cylinderState.combustionSharpness =
                compressionIgnition
                ? compressionIgnitionResults_[index].sharpness : 0.0;
            cylinderState.directLiquidSprayFuelMg =
                injectionStates_[index].directLiquidSprayMoles
                * config_.fuelProperties.molarMassGramsPerMole * 1'000.0;
            cylinderState.directDispersingFuelMg =
                injectionStates_[index].directDispersingVapourMoles
                * config_.fuelProperties.molarMassGramsPerMole * 1'000.0;
            cylinderState.combustionCycleMultiplier =
                combustionCycleMultiplier_[index];
            cylinderState.commandedSparkEventsLastCycle =
                commandedSparkEventsLastCycle_[index];
            cylinderState.completedIgnitionEventsLastCycle =
                completedIgnitionEventsLastCycle_[index];
            cylinderState.commandedSparkPhaseLastCycle =
                commandedSparkPhaseLastCycle_[index];
            cylinderState.completedIgnitionPhaseLastCycle =
                completedIgnitionPhaseLastCycle_[index];
        }
        if (publishExhaustAcousticState && flushExhaustNetwork
            && exhaustNetworkCompleted) {
            ExhaustAcousticSample acousticSample;
            acousticSample.timeSeconds = state_.simulationTimeSeconds;
            acousticSample.couplingFrequencyHz =
                state_.exhaustCouplingFrequencyHz;
            const auto copiedReactionSourceCount = std::min(
                exhaustFuelReaction.sourceCount,
                acousticSample.reactionEvents.size());
            acousticSample.reactionEventCount =
                copiedReactionSourceCount;
            acousticSample.droppedReactionEventCount =
                exhaustFuelReaction.droppedSourceCount
                + exhaustFuelReaction.sourceCount
                    - copiedReactionSourceCount;
            for (std::size_t index = 0;
                index < copiedReactionSourceCount; ++index) {
                const auto& source = exhaustFuelReaction.sources[index];
                auto& event = acousticSample.reactionEvents[index];
                event.timeSeconds = state_.simulationTimeSeconds
                    - 0.5 * source.durationSeconds;
                event.nodeId = source.nodeId;
                event.sourceComponentId = source.sourceComponentId;
                event.pathIndex = source.pathIndex;
                event.axialPosition = static_cast<float>(
                    source.axialPosition);
                event.releasedEnergyJoules = static_cast<float>(
                    source.releasedEnergyJoules);
                event.burnedFuelMassKg = static_cast<float>(
                    source.burnedFuelMassKg);
                event.durationSeconds = static_cast<float>(
                    source.durationSeconds);
                event.densityKgPerM3 = static_cast<float>(
                    source.densityKgPerM3);
                event.speedOfSoundMps = static_cast<float>(
                    source.speedOfSoundMps);
                event.flowAreaM2 = static_cast<float>(
                    source.flowAreaM2);
            }
            const auto outletSamples = physicalExhaustNetwork.outletSamples();
            acousticSample.outletCount = std::min(
                outletSamples.size(),
                acousticSample.outletMassFlowKgPerSecond.size());
            for (std::size_t index = 0;
                 index < acousticSample.outletCount; ++index) {
                const auto& outlet = outletSamples[index];
                acousticSample.outletNodeId[index] =
                    outlet.outletNodeId;
                acousticSample.outletPathIndex[index] =
                    static_cast<std::uint8_t>(outlet.pathIndex);
                acousticSample.outletMassFlowKgPerSecond[index] =
                    static_cast<float>(outlet.massFlowKgPerS);
                acousticSample.outletDensityKgPerM3[index] =
                    static_cast<float>(outlet.densityKgPerM3);
                acousticSample.outletSpeedOfSoundMps[index] =
                    static_cast<float>(outlet.speedOfSoundMps);
                acousticSample.outletAreaM2[index] =
                    static_cast<float>(outlet.openingAreaM2);
            }
            acousticSample.ductCount = exhaustDuctMediumCount_;
            for (std::size_t index = 0;
                 index < exhaustDuctMediumCount_; ++index) {
                acousticSample.ductDensityKgPerM3[index] =
                    exhaustDuctDensityKgPerM3_[index];
                acousticSample.ductSpeedOfSoundMps[index] =
                    exhaustDuctSpeedOfSoundMps_[index];
            }
            if (exhaustAcousticSamples_->tryPush(acousticSample)) {
                ++frame.exhaustAcousticSampleCount;
                lastExhaustAcousticStatePublishSeconds_ =
                    state_.simulationTimeSeconds;
            } else {
                ++frame.droppedExhaustAcousticSampleCount;
            }
        }
        if (pressureSamples_) {
            CylinderPressureSample pressureSample;
            pressureSample.timeSeconds = state_.simulationTimeSeconds;
            pressureSample.cylinderCount = config_.cylinders.size();
            pressureSample.structural.cylinderCount = config_.cylinders.size();
            pressureSample.intakePathCount = intakePlenumCount_;
            for (std::size_t path = 0; path < intakePlenumCount_; ++path) {
                pressureSample.intakeThrottleConductanceAreaM2[path] =
                    static_cast<float>(intakeThrottleConductanceAreaM2[path]);
            }
            for (std::size_t index = 0; index < config_.cylinders.size(); ++index) {
                pressureSample.pressureBar[index] = static_cast<float>(chamberPressureBar_[index]);
                const auto pistonRadiusM = config_.cylinders[index].boreMm * 0.0005;
                const auto pistonAreaM2 = std::numbers::pi * pistonRadiusM * pistonRadiusM;
                const auto gasForceN = (chamberPressureBar_[index] * 100.0
                    - config_.ambientPressureKpa) * 1'000.0 * pistonAreaM2;
                const auto inertiaForceN = structuralInertiaForceN[index];
                const auto bearingReactionForceN = gasForceN - inertiaForceN;
                pressureSample.structural.gasForceN[index] = static_cast<float>(gasForceN);
                pressureSample.structural.inertiaForceN[index] = static_cast<float>(inertiaForceN);
                pressureSample.structural.bearingReactionForceN[index] =
                    static_cast<float>(bearingReactionForceN);
                pressureSample.structural.sideThrustForceN[index] = static_cast<float>(
                    bearingReactionForceN * structuralSideRatio[index]);
                pressureSample.structural.crankReactionTorqueNm[index] = static_cast<float>(
                    bearingReactionForceN * gasTorqueLeverArmM[index]);
                // Port-end (valve-side) state of the 1-D runner, refreshed by
                // the intake network advances earlier in this substep.
                pressureSample.intakeMassFlowKgPerSecond[index] = static_cast<float>(
                    instantaneousIntakeTransferredMassKg[index] / subDt);
                pressureSample.intakeRunnerPressureKpa[index] = static_cast<float>(
                    intakeRunnerPressureKpa_[index]);
                pressureSample.intakeRunnerDensityKgPerM3[index] = static_cast<float>(
                    intakePortDensityKgPerM3_[index]);
                pressureSample.intakeRunnerSpeedOfSoundMps[index] = static_cast<float>(
                    intakePortSpeedOfSoundMps_[index]);
                pressureSample.intakeValveConductanceAreaM2[index] = static_cast<float>(
                    intakeValveAreaM2[index] * intakeValveDischargeCoefficient[index]);
                pressureSample.intakePathIndex[index] =
                    static_cast<std::uint8_t>(
                        intakePathIndexByCylinder_[index]);
                pressureSample.exhaustRunnerPressureKpa[index] = static_cast<float>(exhaustRunnerPressureKpa_[index]);
                pressureSample.exhaustMassFlowKgPerSecond[index] =
                    static_cast<float>(exhaustMassFlowKgPerSecond[index]);
                pressureSample.exhaustAcousticMassFlowKgPerSecond[index] =
                    static_cast<float>(exhaustAcousticMassFlowKgPerSecond[index]);
                pressureSample.exhaustPortDensityKgPerM3[index] =
                    static_cast<float>(exhaustPortDensityKgPerM3[index]);
                pressureSample.exhaustPortSpeedOfSoundMps[index] =
                    static_cast<float>(exhaustPortSpeedOfSoundMps[index]);
                pressureSample.exhaustValveConductanceAreaM2[index] = static_cast<float>(
                    exhaustValveAreaM2[index] * exhaustValveDischargeCoefficient[index]);
                pressureSample.exhaustFlowMgPerCycle[index] = static_cast<float>(exhaustFlowMgPerCycle_[index]);
                const auto cams = activeCamshaft(
                    config_, config_.cylinders[index],
                    state_.rpm, state_.throttle);
                const auto maximumExhaustLiftMm = std::max(0.1, cams.exhaustLift());
                pressureSample.exhaustValveOpening[index] = static_cast<float>(std::clamp(
                    valveTrainResults_[index].exhaustLiftMm / maximumExhaustLiftMm, 0.0, 1.0));
                pressureSample.exhaustPathIndex[index] =
                    static_cast<std::uint8_t>(
                        exhaustPathIndexByCylinder_[index]);
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
        if (combustion.combustionEnabled && ecuCommand.fuelEnabled
                && (ecuCommand.sparkEnabled
                    || config_.fuel == FuelType::diesel)
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
    // Rate the network resolved at, over the network time it actually advanced.
    // Dividing by frame dt instead would dilute it on frames where the coupling
    // stride left the network held, and understate the resolved bandwidth.
    state_.exhaustNetworkAcceptedSubsteps = exhaustNetworkAcceptedSubsteps;
    state_.exhaustNetworkSubstepFrequencyHz = exhaustNetworkAdvancedSeconds > 0.0
        ? static_cast<double>(exhaustNetworkAcceptedSubsteps) / exhaustNetworkAdvancedSeconds
        : 0.0;
    frame.state = state_;
    return frame;
}

void EngineSimulator::accumulateCycleTelemetry(double previousAngleDegrees,
                                               double travelledDegrees,
                                               double dtSeconds,
                                               double indicatedTorqueNm,
                                               double brakeTorqueNm,
                                               SimulationFrame& frame) noexcept {
    if (!(travelledDegrees > 0.0) || !(dtSeconds > 0.0)) return;
    const auto cycleDegrees = eventGenerator_.cycleDegrees();
    if (!(cycleDegrees > 0.0)) return;
    const auto cycleRadians = cycleDegrees * std::numbers::pi / 180.0;
    auto angle = std::fmod(previousAngleDegrees + cycleDegrees, cycleDegrees);
    auto remainingDegrees = travelledDegrees;
    auto elapsedSubstepSeconds = 0.0;
    // A 50 ms recovery step at the configuration ceiling of 20,000 rpm spans
    // only 8.4 four-stroke cycles. Sixteen therefore covers the declared input
    // envelope with margin and preserves a small hard realtime bound if a
    // future event generator returns a nonsensically tiny cycle angle.
    constexpr std::size_t maximumCycleSegmentsPerSubstep = 16;
    std::size_t segmentCount = 0;
    while (remainingDegrees > 1.0e-12
           && segmentCount++ < maximumCycleSegmentsPerSubstep) {
        const auto degreesToBoundary = cycleDegrees - angle;
        const auto segmentDegrees = std::min(remainingDegrees, degreesToBoundary);
        const auto segmentFraction = segmentDegrees / travelledDegrees;
        const auto segmentSeconds = dtSeconds * segmentFraction;
        const auto segmentRadians = segmentDegrees * std::numbers::pi / 180.0;
        if (cycleTelemetryStarted_) {
            indicatedWorkThisCycleJoules_ += indicatedTorqueNm * segmentRadians;
            brakeWorkThisCycleJoules_ += brakeTorqueNm * segmentRadians;
            integratedCrankRadiansThisCycle_ += segmentRadians;
            cycleElapsedSeconds_ += segmentSeconds;
        }
        remainingDegrees -= segmentDegrees;
        elapsedSubstepSeconds += segmentSeconds;
        const auto reachedBoundary = segmentDegrees >= degreesToBoundary - 1.0e-12;
        if (!reachedBoundary) {
            angle += segmentDegrees;
            continue;
        }

        if (cycleTelemetryStarted_ && cycleElapsedSeconds_ > 1.0e-12) {
            CompletedBrakeCycleSample sample;
            sample.cycleId = nextCompletedBrakeCycleId_++;
            sample.startTimeSeconds = cycleStartTimeSeconds_;
            sample.endTimeSeconds = state_.simulationTimeSeconds
                + std::min(dtSeconds, elapsedSubstepSeconds);
            sample.durationSeconds = sample.endTimeSeconds
                - sample.startTimeSeconds;
            sample.integratedCrankRadians = integratedCrankRadiansThisCycle_;
            sample.indicatedWorkJoules = indicatedWorkThisCycleJoules_;
            sample.brakeWorkJoules = brakeWorkThisCycleJoules_;
            const auto positiveDenominators = sample.durationSeconds > 1.0e-12
                && sample.integratedCrankRadians > 1.0e-12;
            if (positiveDenominators) {
                sample.meanRpm = sample.integratedCrankRadians
                    / sample.durationSeconds * 60.0
                    / (2.0 * std::numbers::pi);
                sample.meanTorqueNm = sample.brakeWorkJoules
                    / sample.integratedCrankRadians;
                sample.meanPowerKw = sample.brakeWorkJoules
                    / sample.durationSeconds / 1'000.0;
            }
            const auto timeTolerance = 1.0e-9
                * std::max({ 1.0, std::abs(sample.durationSeconds),
                             std::abs(cycleElapsedSeconds_) });
            const auto angleTolerance = 1.0e-9
                * std::max(1.0, std::abs(cycleRadians));
            sample.numericallyValid = positiveDenominators
                && std::isfinite(sample.startTimeSeconds)
                && std::isfinite(sample.endTimeSeconds)
                && std::isfinite(sample.durationSeconds)
                && std::isfinite(sample.integratedCrankRadians)
                && std::isfinite(sample.indicatedWorkJoules)
                && std::isfinite(sample.brakeWorkJoules)
                && std::isfinite(sample.meanRpm)
                && std::isfinite(sample.meanTorqueNm)
                && std::isfinite(sample.meanPowerKw)
                && std::abs(sample.durationSeconds - cycleElapsedSeconds_)
                    <= timeTolerance
                && std::abs(sample.integratedCrankRadians - cycleRadians)
                    <= angleTolerance;

            if (frame.completedBrakeCycleSampleCount
                < frame.completedBrakeCycleSamples.size()) {
                frame.completedBrakeCycleSamples[
                    frame.completedBrakeCycleSampleCount++] = sample;
            } else {
                ++frame.droppedCompletedBrakeCycleSampleCount;
            }

            // Preserve the existing EngineState contract as a latch of the
            // latest complete, valid cycle. Frames without a boundary leave it
            // untouched; the event array above is never replayed.
            const auto displacementM3 =
                engineDisplacementLitres(config_) * 0.001;
            if (sample.numericallyValid) {
                state_.indicatedWorkJoulesPerCycle = sample.indicatedWorkJoules;
                state_.indicatedMeanEffectivePressureBar = displacementM3 > 0.0
                    ? sample.indicatedWorkJoules / displacementM3 / 100'000.0
                    : 0.0;
                state_.indicatedPowerKw = sample.indicatedWorkJoules
                    / sample.durationSeconds / 1'000.0;
                state_.pdvTorqueNm = sample.indicatedWorkJoules
                    / sample.integratedCrankRadians;
                state_.cycleAveragedTorqueNm = sample.meanTorqueNm;
                state_.cycleAveragedPowerKw = sample.meanPowerKw;
            }
            // The gas-exchange split comes from the per-cylinder p-dV integrals,
            // not from the engine-level torque accumulation above: at any crank
            // angle the cylinders are in different strokes, so only a
            // per-cylinder loop can be attributed to one stroke or the other.
            // Each cylinder contributes its most recently completed cycle,
            // exactly as CylinderState::indicatedWorkJoulesPerCycle already does.
            double pumpingJoules = 0.0;
            double exhaustStrokeJoules = 0.0;
            double totalJoules = 0.0;
            for (std::size_t index = 0; index < config_.cylinders.size(); ++index) {
                pumpingJoules += indicatedWorkStates_[index].completedPumpingCycleJoules;
                exhaustStrokeJoules +=
                    indicatedWorkStates_[index].completedExhaustStrokeCycleJoules;
                totalJoules += indicatedWorkStates_[index].completedCycleJoules;
            }
            state_.pumpingMeanEffectivePressureBar = displacementM3 > 0.0
                ? pumpingJoules / displacementM3 / 100'000.0 : 0.0;
            state_.exhaustStrokeMeanEffectivePressureBar = displacementM3 > 0.0
                ? exhaustStrokeJoules / displacementM3 / 100'000.0 : 0.0;
            state_.grossIndicatedMeanEffectivePressureBar = displacementM3 > 0.0
                ? (totalJoules - pumpingJoules) / displacementM3 / 100'000.0 : 0.0;
        }
        cycleTelemetryStarted_ = true;
        cycleStartTimeSeconds_ = state_.simulationTimeSeconds
            + std::min(dtSeconds, elapsedSubstepSeconds);
        indicatedWorkThisCycleJoules_ = 0.0;
        brakeWorkThisCycleJoules_ = 0.0;
        integratedCrankRadiansThisCycle_ = 0.0;
        cycleElapsedSeconds_ = 0.0;
        angle = 0.0;
    }
}

void EngineSimulator::reset() noexcept {
    ecu_.reset();
    eventGenerator_.reset();
    intakeBoundaryStateTimeIntegral_.fill({});
    intakeBoundaryVolumeTimeIntegralM3S_.fill(0.0);
    intakeValveConductanceTimeIntegralM2S_.fill(0.0);
    intakeCouplingDurationSeconds_ = 0.0;
    exhaustBoundaryKnotWrite_ = 0;
    exhaustValveConductanceTimeIntegralM2S_.fill(0.0);
    exhaustBoundaryStateTimeIntegral_.fill({});
    exhaustBoundaryVolumeTimeIntegralM3S_.fill(0.0);
    exhaustCouplingDurationSeconds_ = 0.0;
    outletOpeningScaleTimeIntegralSeconds_ = 0.0;
    lastExhaustAcousticStatePublishSeconds_ = -1.0;
    if (pressureSamples_) {
        CylinderPressureSample discarded;
        while (pressureSamples_->tryPop(discarded)) {}
    }
    if (exhaustAcousticSamples_) {
        ExhaustAcousticSample discarded;
        while (exhaustAcousticSamples_->tryPop(discarded)) {}
    }
    state_ = {};
    state_.coolantTemperatureC = config_.ambientTemperatureC;
    state_.oilTemperatureC = config_.ambientTemperatureC;
    state_.exhaustTemperatureC = config_.ambientTemperatureC;
    state_.manifoldPressureKpa = config_.ambientPressureKpa;
    state_.boostPressureRatio = 1.0;
    state_.forcedInductionShaftSpeedRpm = 0.0;
    state_.wastegateOpening = 0.0;
    state_.blowOffMassFlowKgPerSecond = 0.0;
    state_.exhaustPressureKpa = config_.ambientPressureKpa;
    state_.exhaustBackPressureKpa = config_.ambientPressureKpa;
    state_.intakeRunnerPressureKpa = config_.ambientPressureKpa;
    state_.exhaustRunnerPressureKpa = config_.ambientPressureKpa;
    randomState_ = 0x6d2b79f5U;
    for (std::size_t index = 0; index < cylinderRandomState_.size(); ++index)
        cylinderRandomState_[index] = 0x6d2b79f5U
            + 0x9e3779b9U * static_cast<std::uint32_t>(index + 1);
    for (std::size_t index = 0;
            index < combustionVariationRandomState_.size(); ++index)
        combustionVariationRandomState_[index] = 0xa341316cU
            + 0x7f4a7c15U * static_cast<std::uint32_t>(index + 1);
    combustionVariationNormalisedState_.fill(0.0);
    combustionCycleMultiplier_.fill(1.0);
    cylinderMisfires_.fill(false);
    intakeFlowMgPerCycle_.fill(0.0);
    exhaustFlowMgPerCycle_.fill(0.0);
    intakeFlowMgThisCycle_.fill(0.0);
    exhaustFlowMgThisCycle_.fill(0.0);
    deliveredAirMgPerCycle_.fill(0.0);
    deliveredAirMgThisCycle_.fill(0.0);
    exhaustRunnerVelocityMps_.fill(0.0);
    exhaustRunnerTemperatureK_.fill(config_.ambientTemperatureC + 273.15);
    cylinderFlowCycleStarted_.fill(false);
    instantaneousCombustionPulse_.fill(0.0);
    injectedFuelMolesThisCycle_.fill(0.0);
    entrainedFuelMolesThisCycle_.fill(0.0);
    meteredFuelMolesLastCycle_.fill(0.0);
    deliveredFuelMolesLastCycle_.fill(0.0);
    requestedFuelMolesThisCycle_.fill(0.0);
    trappedAirMassMgLastCycle_.fill(0.0);
    trappedAirSourcePressureKpaLastCycle_.fill(config_.ambientPressureKpa);
    trappedAirSourceTemperatureKLastCycle_.fill(config_.ambientTemperatureC + 273.15);
    actualAfrLastCycle_.fill(config_.fuelProperties.stoichiometricAirFuelRatio);
    fuelDeliveryRatio_.fill(0.0);
    commandedFuelMolesMaxThisCycle_.fill(0.0);
    injectorOpenSubsteps_.fill(0.0);
    injectorWindowSubsteps_.fill(0.0);
    injectorCapacityRatio_.fill(1.0);
    closedLoopFuelTrim_.fill(1.0);
    highLoadClosedLoopFuelTrim_.fill(1.0);
    highLoadClosedLoopFuelTrimSeeded_.fill(false);
    flameEvents_.fill({});
    compressionIgnitionStates_.fill({});
    compressionIgnitionResults_.fill({});
    injectionStates_.fill({});
    endGasKnockStates_.fill({});
    indicatedWorkStates_.fill({});
    valveTrainStates_.fill({});
    valveTrainResults_.fill({});
    runnerAcousticStates_.fill({});
    runnerAcousticResults_.fill({});
    ignitionDelayRemainingSeconds_.fill(0.0);
    ignitionPending_.fill(false);
    sparkScheduleArmed_.fill(false);
    scheduledSparkPhaseDegrees_.fill(0.0);
    sparkScheduleTravelRemainingDegrees_.fill(0.0);
    commandedSparkEventsThisCycle_.fill(0);
    commandedSparkEventsLastCycle_.fill(0);
    completedIgnitionEventsThisCycle_.fill(0);
    completedIgnitionEventsLastCycle_.fill(0);
    commandedSparkPhaseThisCycle_.fill(-1.0);
    commandedSparkPhaseLastCycle_.fill(-1.0);
    completedIgnitionPhaseThisCycle_.fill(-1.0);
    completedIgnitionPhaseLastCycle_.fill(-1.0);
    eventEvaluationAngleDegrees_ = state_.crankAngleDegrees;
    eventEvaluationTimeSeconds_ = state_.simulationTimeSeconds;
    indicatedWorkThisCycleJoules_ = 0.0;
    brakeWorkThisCycleJoules_ = 0.0;
    integratedCrankRadiansThisCycle_ = 0.0;
    cycleElapsedSeconds_ = 0.0;
    cycleStartTimeSeconds_ = 0.0;
    nextCompletedBrakeCycleId_ = 1;
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
        intakePortTemperatureK_[index] = config_.ambientTemperatureC + 273.15;
        intakePortDensityKgPerM3_[index] = config_.ambientPressureKpa * 1'000.0
            / (GasCell::universalGasConstant / GasCell::airMolarMassKg
               * (config_.ambientTemperatureC + 273.15));
        intakePortSpeedOfSoundMps_[index] = std::sqrt(1.4
            * GasCell::universalGasConstant / GasCell::airMolarMassKg
            * (config_.ambientTemperatureC + 273.15));
        intakeValveColumnVelocityMps_[index] = 0.0;
        intakePortRamKpa_[index] = 0.0;
        exhaustRunnerPressureKpa_[index] = config_.ambientPressureKpa;
        exhaustBoundaryKnots_[index].fill({});
        chamberPressureBar_[index] = config_.ambientPressureKpa / 100.0;
        cylinderWallTemperatureC_[index] = config_.ambientTemperatureC;
        configureFuel(cylinderGas_[index]);

        const auto& cyl = config_.cylinders[index];
        const auto pistonAreaM2 = std::numbers::pi * std::pow(cyl.boreMm * 0.0005, 2.0);

        // Geometry: positive momentum means gas flowing upward out of the
        // cylinder for exhaust. The intake runner is a 1-D duct network with
        // its own conservative state, reset below.
        cylinderGas_[index].setGeometry(pistonAreaM2, 0.0, -1.0); // upward toward head

        cylinderGas_[index].initialise(config_.ambientPressureKpa,
            evaluateCylinderKinematics(config_, kinematicsReference_, index,
                state_.crankAngleDegrees, 0.0).chamberVolumeLitres,
            config_.ambientTemperatureC + 273.15);
    }

    // Construction guarantees these invariants. reset() cannot report failure
    // through IEngineSimulation and must not silently revive the old lumped
    // exhaust or intake, so an invariant violation is deliberately fatal.
    if (!physicalExhaustNetwork_
        || !physicalExhaustNetwork_->reset(config_.ambientPressureKpa * 1'000.0,
                                            config_.ambientTemperatureC + 273.15))
        std::terminate();
    for (std::size_t index = 0; index < config_.cylinders.size(); ++index) {
        if (!intakeRunnerNetworks_[index]
            || !intakeRunnerNetworks_[index]->reset(config_.ambientPressureKpa * 1'000.0,
                                                     config_.ambientTemperatureC + 273.15))
            std::terminate();
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
