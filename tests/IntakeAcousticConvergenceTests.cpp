// A passive intake runner cannot manufacture acoustic energy. This experiment
// changes only the time integrator within each paired run: grid, initial mode,
// prescribed time step, closed boundaries and physical losses are identical.
// Total gas energy conservation alone cannot detect spurious conversion of
// internal energy into an acoustic oscillation, so measure the perturbation.
// The single-stage MUSCL-Hancock scheme the two-stroke exhaust uses must stay
// passive where plain forward Euler on the same reconstruction does not.
#include <enginelab/gasdynamics/ExhaustGasNetwork.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
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

enum class Integrator { euler, rk2, hancock };

[[nodiscard]] const char* integratorName(Integrator integrator) noexcept {
    switch (integrator) {
    case Integrator::euler: return "Euler";
    case Integrator::rk2: return "RK2";
    case Integrator::hancock: return "Hancock";
    }
    return "?";
}

[[nodiscard]] PassiveResult passiveRun(std::size_t cellCount, Integrator integrator) {
    ExhaustGasNetworkConfig config;
    config.initialPressurePa = referencePressurePa;
    config.initialTemperatureK = referenceTemperatureK;
    config.absoluteRoughnessM = 1.5e-6;
    config.maximumCourantNumber = 0.80;
    config.firstOrderTimeIntegration = integrator != Integrator::rk2;
    config.hancockPredictor = integrator == Integrator::hancock;
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

// Two sealed ducts joined by an open area step (38 -> 76 mm): three quarters
// of the wide duct's first face are wall beside an open aperture, the face a
// stinger meets its silencer on.
[[nodiscard]] double steppedMaximumAmplitudeGain(std::size_t cellsPerDuct,
                                                 Integrator integrator,
                                                 double& finalAmplitudeGain) {
    const auto makeDuct = [&](std::uint32_t nodeId, double diameterM) {
        const auto areaM2 = std::numbers::pi * diameterM * diameterM * 0.25;
        CompiledExhaustDuct duct;
        duct.nodeId = nodeId;
        duct.lengthM = runnerLengthM;
        duct.flowAreaM2 = areaM2;
        duct.inletFlowAreaM2 = areaM2;
        duct.outletFlowAreaM2 = areaM2;
        duct.connectionAreaM2 = areaM2;
        duct.inletConnectionAreaM2 = areaM2;
        duct.outletConnectionAreaM2 = areaM2;
        duct.hydraulicDiameterM = diameterM;
        duct.volumeM3 = areaM2 * runnerLengthM;
        duct.cellCount = cellsPerDuct;
        return duct;
    };
    CompiledExhaustInterface areaStep;
    areaStep.upstream = { ExhaustEndpointType::ductOutlet, 0, 100 };
    areaStep.downstream = { ExhaustEndpointType::ductInlet, 1, 101 };
    CompiledCylinderPort valve;
    valve.cylinderId = 1;
    valve.networkEndpoint = { ExhaustEndpointType::ductInlet, 0, 100 };
    valve.runnerConnectionAreaM2 = std::numbers::pi * runnerDiameterM * runnerDiameterM * 0.25;
    CompiledExhaustOutlet mouth;
    mouth.outletNodeId = 200;
    mouth.networkEndpoint = { ExhaustEndpointType::ductOutlet, 1, 101 };
    mouth.openingAreaM2 = valve.runnerConnectionAreaM2;
    const auto layout = ExhaustNetworkLayout::assemble(
        { makeDuct(100, runnerDiameterM), makeDuct(101, 2.0 * runnerDiameterM) },
        {}, { areaStep }, { valve }, { mouth });
    ExhaustGasNetworkConfig config;
    config.initialPressurePa = referencePressurePa;
    config.initialTemperatureK = referenceTemperatureK;
    config.absoluteRoughnessM = 1.5e-6;
    config.maximumCourantNumber = 0.80;
    config.firstOrderTimeIntegration = integrator != Integrator::rk2;
    config.hancockPredictor = integrator == Integrator::hancock;
    ExhaustGasNetwork network;
    require(network.configure(layout, config), "the stepped fixture must configure");
    const auto base = network.mixtureModel().conservativeFromPressureTemperature(
        referencePressurePa, referenceTemperatureK);
    require(base.has_value(), "the ambient fixture must be physical");
    const auto basePrimitive = network.mixtureModel().primitiveFromConservative(*base);
    require(basePrimitive.has_value(), "the ambient fixture must recover");
    const auto densityKgPerM3 = basePrimitive->densityKgPerM3;
    const auto soundSpeedMps = basePrimitive->speedOfSoundMps;
    // The closed-closed fundamental of the whole length, isentropic.
    for (std::size_t ductIndex = 0; ductIndex < 2; ++ductIndex) {
        auto cells = network.ducts()[ductIndex].cells();
        for (std::size_t index = 0; index < cells.size(); ++index) {
            const auto positionFraction = (static_cast<double>(ductIndex * cellsPerDuct + index)
                + 0.5) / static_cast<double>(2 * cellsPerDuct);
            const auto pressurePa = referencePressurePa + pressureAmplitudePa
                * std::cos(std::numbers::pi * positionFraction);
            const auto density = densityKgPerM3 * std::pow(
                pressurePa / referencePressurePa, 1.0 / basePrimitive->heatCapacityRatio);
            const auto state = network.mixtureModel().conservativeFromPrimitive(
                density, 0.0, pressurePa, GasComposition::dryAir());
            require(state.has_value(), "the stepped mode must be physical");
            cells[index] = *state;
        }
    }
    const auto energy = [&]() {
        auto volumeM3 = 0.0;
        auto pressureVolume = 0.0;
        for (std::size_t ductIndex = 0; ductIndex < 2; ++ductIndex) {
            const auto cellVolumeM3 = network.layout().ducts()[ductIndex].volumeM3
                / static_cast<double>(cellsPerDuct);
            for (const auto& cell : network.ducts()[ductIndex].cells()) {
                const auto primitive = network.mixtureModel().primitiveFromConservative(cell);
                require(primitive.has_value(), "a stepped state must remain physical");
                pressureVolume += primitive->pressurePa * cellVolumeM3;
                volumeM3 += cellVolumeM3;
            }
        }
        const auto meanPressurePa = pressureVolume / volumeM3;
        auto energyJ = 0.0;
        for (std::size_t ductIndex = 0; ductIndex < 2; ++ductIndex) {
            const auto cellVolumeM3 = network.layout().ducts()[ductIndex].volumeM3
                / static_cast<double>(cellsPerDuct);
            for (const auto& cell : network.ducts()[ductIndex].cells()) {
                const auto primitive = network.mixtureModel().primitiveFromConservative(cell);
                const auto pressurePa = primitive->pressurePa - meanPressurePa;
                energyJ += cellVolumeM3 * 0.5 * (pressurePa * pressurePa
                        / (densityKgPerM3 * soundSpeedMps * soundSpeedMps)
                    + densityKgPerM3 * primitive->velocityMps * primitive->velocityMps);
            }
        }
        require(std::isfinite(energyJ), "stepped acoustic energy must remain finite");
        return energyJ;
    };
    const auto initialEnergyJ = energy();
    const auto stepSeconds = prescribedCourantNumber * runnerLengthM
        / static_cast<double>(cellsPerDuct) / soundSpeedMps;
    const auto steps = static_cast<std::size_t>(std::ceil(
        traversalCount * 2.0 * runnerLengthM / soundSpeedMps / stepSeconds));
    const ExhaustAmbientBoundary sealedMouth { *base, 0.0 };
    auto maximumRatio = 1.0;
    auto ratio = 1.0;
    for (std::size_t step = 0; step < steps; ++step) {
        require(network.advance(stepSeconds, {}, sealedMouth).completed,
            "the stepped fixture must complete every prescribed step");
        ratio = energy() / initialEnergyJ;
        maximumRatio = std::max(maximumRatio, ratio);
    }
    finalAmplitudeGain = std::sqrt(ratio);
    return std::sqrt(maximumRatio);
}

void printResult(std::size_t cells, Integrator integrator, const PassiveResult& result) {
    std::cout << integratorName(integrator) << " cells=" << cells
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
    auto hancockPassive = true;
    const auto passive = [](const PassiveResult& result) noexcept {
        return std::sqrt(result.maximumEnergyRatio) <= maximumPassiveAmplitudeGain
            && result.finalEnergyRatio < maximumFinalEnergyFraction;
    };
    for (const auto cells : std::array<std::size_t, 3> { 3, 6, 12 }) {
        const auto euler = passiveRun(cells, Integrator::euler);
        const auto rk2 = passiveRun(cells, Integrator::rk2);
        const auto hancock = passiveRun(cells, Integrator::hancock);
        printResult(cells, Integrator::euler, euler);
        printResult(cells, Integrator::rk2, rk2);
        printResult(cells, Integrator::hancock, hancock);
        require(euler.stepSeconds == rk2.stepSeconds
                    && euler.prescribedSteps == rk2.prescribedSteps
                    && hancock.prescribedSteps == rk2.prescribedSteps,
            "each Euler/RK2/Hancock set must share the prescribed grid and time steps");
        hancockPassive = hancockPassive && passive(hancock);
        for (const auto integrator : { Integrator::rk2, Integrator::hancock }) {
            auto finalGain = 0.0;
            const auto gain = steppedMaximumAmplitudeGain(cells, integrator, finalGain);
            std::cout << integratorName(integrator) << " stepped cells=" << cells
                      << " max_amplitude_gain=" << gain
                      << " final_amplitude_gain=" << finalGain << '\n';
            if (integrator == Integrator::hancock)
                hancockPassive = hancockPassive
                    && gain <= maximumPassiveAmplitudeGain && finalGain < 1.0;
        }
        // The alternate integrator remains a diagnostic control. The gate
        // follows the exact policy used by EngineSimulator, with unchanged
        // physical passivity and decay requirements for production.
        const auto& production = productionForwardEuler ? euler : rk2;
        allPassive = allPassive && passive(production);
    }
    if (!diagnosticOnly) {
        require(allPassive,
            "a passive runner must not amplify its initial acoustic mode and must decay");
        require(hancockPassive,
            "the Hancock scheme must not amplify the closed acoustic mode and must decay");
    }
    return EXIT_SUCCESS;
}
