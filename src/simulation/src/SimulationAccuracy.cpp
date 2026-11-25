#include <enginelab/simulation/SimulationAccuracy.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace enginelab {

std::string_view accuracyProfileName(AccuracyProfile profile) noexcept {
    switch (profile) {
    case AccuracyProfile::realtime: return "realtime";
    case AccuracyProfile::balanced: return "balanced";
    case AccuracyProfile::engineering: return "engineering";
    case AccuracyProfile::highPrecision: return "high-precision";
    }
    return "realtime";
}

std::optional<AccuracyProfile> accuracyProfileFromName(std::string_view name) noexcept {
    for (const auto profile : { AccuracyProfile::realtime, AccuracyProfile::balanced,
                                AccuracyProfile::engineering, AccuracyProfile::highPrecision })
        if (name == accuracyProfileName(profile)) return profile;
    return std::nullopt;
}

AccuracyResolution accuracyResolution(AccuracyProfile profile, const EngineConfig& config) noexcept {
    AccuracyResolution result;
    result.profile = profile;
    result.refinement = profile == AccuracyProfile::realtime ? 1.0
        : profile == AccuracyProfile::balanced ? 2.0
        : profile == AccuracyProfile::engineering ? 4.0 : 8.0;
    const auto k = result.refinement;
    const auto ceilingOf = [k](std::size_t production) noexcept {
        return static_cast<std::size_t>(static_cast<double>(production) * k);
    };
    result.exhaustCellLengthM = std::max(EngineSimulator::finestExhaustCellLengthM,
        EngineSimulator::productionExhaustCellLengthM(config) / k);
    result.exhaustCouplingSeconds = productionExhaustCouplingSeconds / k;
    result.exhaustCouplingEverySubstep = profile == AccuracyProfile::highPrecision;
    result.exhaustMaximumCellsPerDuct = ceilingOf(64);
    result.exhaustMaximumTotalCells = ceilingOf(1'024);
    result.intakeCellLengthM = productionIntakeCellLengthM / k;
    result.intakeCouplingSeconds = profile == AccuracyProfile::highPrecision
        ? 0.0 : productionIntakeCouplingSeconds / k;
    result.intakeMaximumCellsPerRunner = ceilingOf(12);
    const auto courant = profile == AccuracyProfile::highPrecision
        ? productionCourantNumber * 0.5 : productionCourantNumber;
    result.exhaustCourantNumber = courant;
    result.intakeCourantNumber = courant;

    // The crank step: production's divided by k, as long as the substep rate
    // at the rev limit stays under the solver's ceiling.
    const auto maximumRpm = std::max(config.redlineRpm, config.ignition.revLimitRpm);
    const auto asked = config.solver.maximumCrankDegreesPerStep / k;
    const auto finestAllowed = maximumRpm * 6.0 / solverFrequencyCeilingHz;
    result.crankStepHeldBySolverCeiling = asked < finestAllowed;
    result.crankDegreesPerStep = std::max(asked, finestAllowed);
    result.solverMaximumFrequencyHz = profile == AccuracyProfile::realtime
        ? config.solver.maximumMechanicalFrequencyHz
        : std::clamp(maximumRpm * 6.0 / result.crankDegreesPerStep,
                     config.solver.maximumMechanicalFrequencyHz, solverFrequencyCeilingHz);
    return result;
}

AccuracyResolution applyAccuracyProfile(AccuracyProfile profile, EngineConfig& config,
                                        EngineSimulatorOptions& options) {
    const auto resolution = accuracyResolution(profile, config);
    if (profile == AccuracyProfile::realtime) return resolution;
    options.exhaustTargetCellLengthM = resolution.exhaustCellLengthM;
    options.exhaustMaximumCellsPerDuct = resolution.exhaustMaximumCellsPerDuct;
    options.exhaustMaximumTotalCells = resolution.exhaustMaximumTotalCells;
    options.maximumLowSpeedExhaustCouplingSeconds = resolution.exhaustCouplingSeconds;
    options.exhaustCouplingEverySubstep = resolution.exhaustCouplingEverySubstep;
    options.exhaustMaximumCourantNumber = resolution.exhaustCourantNumber;
    options.intakeTargetCellLengthM = resolution.intakeCellLengthM;
    options.intakeMaximumCellsPerRunner = resolution.intakeMaximumCellsPerRunner;
    options.intakeCouplingIntervalSeconds = resolution.intakeCouplingSeconds;
    options.intakeMaximumCourantNumber = resolution.intakeCourantNumber;
    config.solver.maximumCrankDegreesPerStep = resolution.crankDegreesPerStep;
    config.solver.maximumMechanicalFrequencyHz = resolution.solverMaximumFrequencyHz;
    return resolution;
}

std::string accuracyReport(const AccuracyResolution& asked, const EngineSimulatorOptions& options,
                           const EngineConfig& config,
                           const gasdynamics::ExhaustNetworkLayout& layout) {
    const auto askedCellM = options.exhaustTargetCellLengthM.value_or(
        EngineSimulator::productionExhaustCellLengthM(config));
    std::ostringstream line;
    line << std::setprecision(4) << "accuracy " << accuracyProfileName(asked.profile)
         << ": exhaust " << layout.totalCellCount() << " cells at "
         << layout.effectiveTargetCellLengthM() * 1'000.0 << " mm (asked "
         << askedCellM * 1'000.0 << ')';
    if (layout.effectiveTargetCellLengthM() > askedCellM * (1.0 + 1.0e-9))
        line << " COARSENED by the network's cell ceiling";
    if (layout.ductCellCapReached()) line << ", a duct held at its cell ceiling";
    if (options.exhaustTaperedDuctRefinement.value_or(1.0) > 1.0)
        line << ", cones " << *options.exhaustTaperedDuctRefinement << " x finer";
    if (options.exhaustMinimumCellsPerDuct)
        line << ", at least " << *options.exhaustMinimumCellsPerDuct << " cells a duct";
    line << ", shortest " << layout.minimumCellLengthM() * 1'000.0 << " mm; coupling ";
    if (options.exhaustCouplingEverySubstep.value_or(false)) line << "every substep";
    else
        line << "at most " << std::max(25.0e-6, options.maximumLowSpeedExhaustCouplingSeconds.value_or(
                                  productionExhaustCouplingSeconds)) * 1.0e6 << " us";
    line << ", Courant " << options.exhaustMaximumCourantNumber.value_or(productionCourantNumber)
         << "; crank step " << config.solver.maximumCrankDegreesPerStep << " deg";
    if (asked.crankStepHeldBySolverCeiling) line << " (held by the 100 kHz solver ceiling)";
    line << "; intake cell " << std::clamp(options.intakeTargetCellLengthM.value_or(
                                    productionIntakeCellLengthM), 0.010, 0.150) * 1'000.0
         << " mm, coupling ";
    const auto intakeCoupling = options.intakeCouplingIntervalSeconds.value_or(
        productionIntakeCouplingSeconds);
    if (intakeCoupling <= 0.0) line << "every substep";
    else line << "at most " << intakeCoupling * 1.0e6 << " us";
    return line.str();
}

} // namespace enginelab
