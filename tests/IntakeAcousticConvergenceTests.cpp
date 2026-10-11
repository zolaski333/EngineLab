// A passive intake runner cannot manufacture acoustic energy. This experiment
// changes only the time integrator within each paired run: grid, initial mode,
// prescribed time step, closed boundaries and physical losses are identical.
// Total gas energy conservation alone cannot detect spurious conversion of
// internal energy into an acoustic oscillation, so measure the perturbation.
#include <enginelab/gasdynamics/ExhaustGasNetwork.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <string_view>

namespace {
using namespace enginelab::gasdynamics;

constexpr double runnerLengthM = 0.285;
constexpr double runnerDiameterM = 0.038;
constexpr double referencePressurePa = 101'325.0;
constexpr double referenceTemperatureK = 300.0;
constexpr double pressureAmplitudePa = 100.0;
constexpr double prescribedCourantNumber = 0.70;
constexpr double traversalCount = 100.0;
constexpr double maximumPassiveAmplitudeGain = 1.05;
// Losses are passive, but no analytic decay rate is assumed for the viscous
// runner. Require a reduction without calibrating it onto a solver output.
constexpr double maximumFinalEnergyFraction = 1.0;

void require(bool condition, std::string_view message) {
    if (condition) return;
    std::cerr << "FAILED: " << message << '\n';
    std::exit(EXIT_FAILURE);
}

[[nodiscard]] ExhaustNetworkLayout runnerLayout(std::size_t cellCount) {
    const auto areaM2 = std::numbers::pi * runnerDiameterM
        * runnerDiameterM * 0.25;
    CompiledExhaustDuct duct;
    duct.nodeId = 100;
    duct.lengthM = runnerLengthM;
    duct.flowAreaM2 = areaM2;
    duct.inletFlowAreaM2 = areaM2;
    duct.outletFlowAreaM2 = areaM2;
    duct.connectionAreaM2 = areaM2;
    duct.inletConnectionAreaM2 = areaM2;
    duct.outletConnectionAreaM2 = areaM2;
    duct.hydraulicDiameterM = runnerDiameterM;
    duct.volumeM3 = areaM2 * runnerLengthM;
    duct.cellCount = cellCount;
    CompiledCylinderPort valve;
    valve.cylinderId = 1;
    valve.networkEndpoint = { ExhaustEndpointType::ductInlet, 0, 100 };
    valve.runnerConnectionAreaM2 = areaM2;
    CompiledExhaustOutlet mouth;
    mouth.outletNodeId = 200;
    mouth.networkEndpoint = { ExhaustEndpointType::ductOutlet, 0, 100 };
    mouth.openingAreaM2 = areaM2;
    return ExhaustNetworkLayout::assemble(
        { duct }, {}, {}, { valve }, { mouth });
}

struct AcousticMeasurement final {
    double energyJ { 0.0 };
    double pressureRmsPa { 0.0 };
    double velocityRmsMps { 0.0 };
};

[[nodiscard]] AcousticMeasurement acousticMeasurement(
    const ExhaustGasNetwork& network, double referenceDensityKgPerM3,
    double referenceSoundSpeedMps) {
    const auto& duct = network.ducts().front();
    const auto cells = duct.cells();
    auto pressureMeanPa = 0.0;
    for (const auto& cell : cells) {
        const auto primitive = network.mixtureModel().primitiveFromConservative(cell);
        require(primitive.has_value(), "a passive runner state must remain physical");
        pressureMeanPa += primitive->pressurePa;
    }
    pressureMeanPa /= static_cast<double>(cells.size());
    auto pressureSquaresPa2 = 0.0;
    auto velocitySquaresM2PerS2 = 0.0;
    for (const auto& cell : cells) {
        const auto primitive = network.mixtureModel().primitiveFromConservative(cell);
        require(primitive.has_value(), "a passive runner state must remain physical");
        const auto pressurePerturbationPa = primitive->pressurePa - pressureMeanPa;
        pressureSquaresPa2 += pressurePerturbationPa * pressurePerturbationPa;
        velocitySquaresM2PerS2 += primitive->velocityMps * primitive->velocityMps;
    }
    const auto cellCount = static_cast<double>(cells.size());
    const auto volumeM3 = network.layout().ducts().front().volumeM3;
    AcousticMeasurement result;
    result.pressureRmsPa = std::sqrt(pressureSquaresPa2 / cellCount);
    result.velocityRmsMps = std::sqrt(velocitySquaresM2PerS2 / cellCount);
    result.energyJ = volumeM3 * 0.5 / cellCount
        * (pressureSquaresPa2 / (referenceDensityKgPerM3
                * referenceSoundSpeedMps * referenceSoundSpeedMps)
           + referenceDensityKgPerM3 * velocitySquaresM2PerS2);
    require(std::isfinite(result.energyJ), "acoustic energy must remain finite");
    return result;
}

struct PassiveResult final {
    double stepSeconds { 0.0 };
    double initialEnergyJ { 0.0 };
    double maximumEnergyRatio { 1.0 };
    double finalEnergyRatio { 0.0 };
    double maximumPressureRmsPa { 0.0 };
    double maximumVelocityRmsMps { 0.0 };
    double relativeConservationError { 0.0 };
    double maximumAcceptedStepRatio { 0.0 };
    double maximumCourantCurrent { 0.0 };
    double maximumCourantPredictor { 0.0 };
    std::size_t acceptedSubsteps { 0 };
    std::size_t rejectedSubsteps { 0 };
    std::size_t prescribedSteps { 0 };
};

[[nodiscard]] PassiveResult passiveRun(std::size_t cellCount, bool euler) {
    ExhaustGasNetworkConfig config;
    config.initialPressurePa = referencePressurePa;
    config.initialTemperatureK = referenceTemperatureK;
    config.absoluteRoughnessM = 1.5e-6;
    config.maximumCourantNumber = 0.80;
    config.firstOrderTimeIntegration = euler;
    ExhaustGasNetwork network;
    require(network.configure(runnerLayout(cellCount), config),
        "the passive intake runner must configure");
    const auto base = network.mixtureModel().conservativeFromPressureTemperature(
        referencePressurePa, referenceTemperatureK);
    require(base.has_value(), "the ambient fixture must be physical");
    const auto basePrimitive = network.mixtureModel().primitiveFromConservative(*base);
    require(basePrimitive.has_value(), "the ambient fixture must recover");
    const auto densityKgPerM3 = basePrimitive->densityKgPerM3;
    const auto soundSpeedMps = basePrimitive->speedOfSoundMps;
    // Closed-end fundamental at maximum compression: u=0, p'=A*cos(pi*x/L).
    // Isentropic density avoids adding a thermal/entropy perturbation.
    auto& duct = network.ducts().front();
    auto cells = duct.cells();
    for (std::size_t index = 0; index < cells.size(); ++index) {
        const auto positionFraction = (static_cast<double>(index) + 0.5)
            / static_cast<double>(cells.size());
        const auto pressurePa = referencePressurePa + pressureAmplitudePa
            * std::cos(std::numbers::pi * positionFraction);
        const auto density = densityKgPerM3 * std::pow(
            pressurePa / referencePressurePa, 1.0 / basePrimitive->heatCapacityRatio);
        const auto state = network.mixtureModel().conservativeFromPrimitive(
            density, 0.0, pressurePa, GasComposition::dryAir());
        require(state.has_value(), "the small acoustic mode must be physical");
        cells[index] = *state;
    }
    const auto before = network.inventory();
    const auto initial = acousticMeasurement(network, densityKgPerM3, soundSpeedMps);
    require(initial.energyJ > 0.0, "the passive fixture must contain an acoustic mode");
    PassiveResult result;
    result.initialEnergyJ = initial.energyJ;
    result.maximumPressureRmsPa = initial.pressureRmsPa;
    result.stepSeconds = prescribedCourantNumber * runnerLengthM
        / static_cast<double>(cellCount) / soundSpeedMps;
    result.prescribedSteps = static_cast<std::size_t>(std::ceil(
        traversalCount * runnerLengthM / soundSpeedMps / result.stepSeconds));
    // Missing cylinder boundary closes the valve; openingScale=0 closes the
    // mouth. Both ends reflect and no boundary can supply mass or energy.
    const ExhaustAmbientBoundary sealedMouth { *base, 0.0 };
    for (std::size_t step = 0; step < result.prescribedSteps; ++step) {
        const auto advance = network.advance(result.stepSeconds, {}, sealedMouth);
        require(advance.completed, "the passive runner must complete every prescribed step");
        require(std::abs(advance.advancedTimeSeconds - result.stepSeconds)
                    <= result.stepSeconds * 1.0e-12,
            "the integrator must advance the complete prescribed duration");
        result.acceptedSubsteps += advance.acceptedSubsteps;
        result.rejectedSubsteps += advance.rejectedSubsteps;
        result.maximumAcceptedStepRatio = std::max(result.maximumAcceptedStepRatio,
            advance.maximumAcceptedStableStepRatio);
        result.maximumCourantCurrent = std::max(result.maximumCourantCurrent,
            advance.maximumAcceptedDuctCourantCurrent);
        result.maximumCourantPredictor = std::max(result.maximumCourantPredictor,
            advance.maximumAcceptedDuctCourantPredictor);
        const auto measured = acousticMeasurement(network, densityKgPerM3, soundSpeedMps);
        const auto ratio = measured.energyJ / initial.energyJ;
        result.maximumEnergyRatio = std::max(result.maximumEnergyRatio, ratio);
        result.finalEnergyRatio = ratio;
        result.maximumPressureRmsPa = std::max(
            result.maximumPressureRmsPa, measured.pressureRmsPa);
        result.maximumVelocityRmsMps = std::max(
            result.maximumVelocityRmsMps, measured.velocityRmsMps);
    }
    const auto after = network.inventory();
    result.relativeConservationError = std::abs(after.totalEnergyJ - before.totalEnergyJ)
        / before.totalEnergyJ;
    for (std::size_t species = 0; species < gasSpeciesCount; ++species) {
        require(std::abs(after.speciesMassKg[species] - before.speciesMassKg[species])
                    <= std::max(1.0e-14, std::abs(before.speciesMassKg[species]) * 1.0e-10),
            "a sealed passive runner must conserve every gas species");
    }
    require(result.relativeConservationError < 1.0e-10,
        "a sealed passive runner must conserve total gas energy");
    return result;
}

void printResult(std::size_t cells, bool euler, const PassiveResult& result) {
    std::cout << (euler ? "Euler" : "RK2") << " cells=" << cells
              << " dt_us=" << result.stepSeconds * 1.0e6
              << " initial_acoustic_J=" << result.initialEnergyJ
              << " max_amplitude_gain=" << std::sqrt(result.maximumEnergyRatio)
              << " final_amplitude_gain=" << std::sqrt(result.finalEnergyRatio)
              << " max_pressure_rms_Pa=" << result.maximumPressureRmsPa
              << " max_velocity_rms_mps=" << result.maximumVelocityRmsMps
              << " total_energy_relative_error=" << result.relativeConservationError
              << " prescribed_steps=" << result.prescribedSteps
              << " accepted_substeps=" << result.acceptedSubsteps
              << " rejected_substeps=" << result.rejectedSubsteps
              << " accepted_step_ratio=" << result.maximumAcceptedStepRatio
              << " courant_current=" << result.maximumCourantCurrent
              << " courant_predictor=" << result.maximumCourantPredictor << '\n';
}
} // namespace

int main(int argc, char** argv) {
    auto diagnosticOnly = false;
    for (int index = 1; index < argc; ++index) {
        require(std::string_view(argv[index]) == "--diagnostic-only",
            "the only supported option is --diagnostic-only");
        diagnosticOnly = true;
    }
    std::cout << std::setprecision(10);
    const auto productionForwardEuler = enginelab::EngineSimulatorOptions {}
        .intakeUsesForwardEuler();
    std::cout << "Production intake integrator: "
              << (productionForwardEuler ? "Euler" : "RK2") << '\n';
    auto allPassive = true;
    for (const auto cells : std::array<std::size_t, 3> { 3, 6, 12 }) {
        const auto euler = passiveRun(cells, true);
        const auto rk2 = passiveRun(cells, false);
        printResult(cells, true, euler);
        printResult(cells, false, rk2);
        require(euler.stepSeconds == rk2.stepSeconds
                    && euler.prescribedSteps == rk2.prescribedSteps,
            "each Euler/RK2 pair must share the prescribed grid and time steps");
        // The alternate integrator remains a diagnostic control. The gate
        // follows the exact policy used by EngineSimulator, with unchanged
        // physical passivity and decay requirements for production.
        const auto& production = productionForwardEuler ? euler : rk2;
        allPassive = allPassive && std::sqrt(production.maximumEnergyRatio)
            <= maximumPassiveAmplitudeGain
            && production.finalEnergyRatio < maximumFinalEnergyFraction;
    }
    if (!diagnosticOnly)
        require(allPassive,
            "a passive runner must not amplify its initial acoustic mode and must decay");
    return EXIT_SUCCESS;
}
