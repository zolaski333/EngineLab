// Intake-shaped use of the finite-volume gas network.
//
// The 1-D intake runner network reuses ExhaustGasNetwork verbatim: one duct
// per cylinder assembled programmatically (ExhaustNetworkLayout::assemble),
// the cylinder port at the duct inlet (the valve), the outlet at the duct
// outlet (the runner mouth), and the plenum passed as the ambient reservoir.
// The network then runs predominantly in sustained INFLOW through its outlet
// -- the regime the open-end characteristic boundary handles as "the duct is
// drawing from a reservoir" -- with a cold medium, which no exhaust layout
// exercises continuously. These tests pin that usage: assembly invariants,
// conservation while drawing, and the port species source used by port fuel
// injection.

#include <enginelab/gasdynamics/ExhaustGasNetwork.hpp>

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string_view>
#include <vector>

namespace {

using namespace enginelab::gasdynamics;

void requireIntake(bool condition, std::string_view message) {
    if (condition) return;
    std::cerr << "FAILED: " << message << '\n';
    std::exit(EXIT_FAILURE);
}

[[nodiscard]] double sumSpeciesMass(
    const std::array<double, gasSpeciesCount>& species) noexcept {
    double result = 0.0;
    for (const auto value : species) result += value;
    return result;
}

// One 250 mm x 38 mm runner duct with 8 cells: valve port at the inlet,
// runner mouth at the outlet. This mirrors what the simulator assembles.
[[nodiscard]] ExhaustNetworkLayout makeRunnerLayout(
    std::size_t cellCount = 8, double outletRadiusM = 0.019) {
    CompiledExhaustDuct runner;
    runner.nodeId = 100;
    runner.lengthM = 0.25;
    const auto radiusM = 0.019;
    runner.inletFlowAreaM2 =
        3.14159265358979323846 * radiusM * radiusM;
    runner.outletFlowAreaM2 =
        3.14159265358979323846 * outletRadiusM * outletRadiusM;
    runner.flowAreaM2 = (runner.inletFlowAreaM2
        + std::sqrt(runner.inletFlowAreaM2 * runner.outletFlowAreaM2)
        + runner.outletFlowAreaM2) / 3.0;
    runner.inletConnectionAreaM2 = runner.inletFlowAreaM2;
    runner.outletConnectionAreaM2 = runner.outletFlowAreaM2;
    runner.connectionAreaM2 = runner.flowAreaM2;
    runner.hydraulicDiameterM =
        2.0 * std::sqrt(runner.flowAreaM2 / 3.14159265358979323846);
    runner.volumeM3 = runner.flowAreaM2 * runner.lengthM;
    runner.cellCount = cellCount;
    CompiledCylinderPort port;
    port.cylinderId = 1;
    port.networkEndpoint = { ExhaustEndpointType::ductInlet, 0, 100 };
    port.runnerConnectionAreaM2 = runner.inletConnectionAreaM2;
    port.dischargeCoefficient = 1.0;
    CompiledExhaustOutlet mouth;
    mouth.outletNodeId = 200;
    mouth.networkEndpoint = { ExhaustEndpointType::ductOutlet, 0, 100 };
    mouth.openingAreaM2 = runner.outletConnectionAreaM2;
    mouth.dischargeCoefficient = 1.0;
    return ExhaustNetworkLayout::assemble({ runner }, {}, {}, { port }, { mouth });
}

void testAssembleAcceptsRunnerShapeAndRejectsBadOrientation() {
    const auto layout = makeRunnerLayout();
    requireIntake(layout.valid(), "a runner-shaped layout must assemble as valid");
    requireIntake(layout.ducts().size() == 1 && layout.cylinderPorts().size() == 1
                      && layout.outlets().size() == 1 && layout.totalCellCount() == 8,
                  "assembled layout must carry exactly the supplied elements");

    // A port on the outlet face uses a flux orientation evaluateStage never
    // applies with the matching sign; assemble must refuse it outright.
    CompiledExhaustDuct runner;
    runner.nodeId = 100;
    runner.lengthM = 0.25;
    runner.flowAreaM2 = 1.1e-3;
    runner.connectionAreaM2 = 1.1e-3;
    runner.hydraulicDiameterM = 0.038;
    runner.cellCount = 8;
    CompiledCylinderPort wrongEndPort;
    wrongEndPort.cylinderId = 1;
    wrongEndPort.networkEndpoint = { ExhaustEndpointType::ductOutlet, 0, 100 };
    wrongEndPort.runnerConnectionAreaM2 = 1.1e-3;
    CompiledExhaustOutlet wrongEndMouth;
    wrongEndMouth.outletNodeId = 200;
    wrongEndMouth.networkEndpoint = { ExhaustEndpointType::ductInlet, 0, 100 };
    wrongEndMouth.openingAreaM2 = 1.1e-3;
    const auto flipped = ExhaustNetworkLayout::assemble(
        { runner }, {}, {}, { wrongEndPort }, { wrongEndMouth });
    requireIntake(!flipped.valid(),
                  "a port at a duct outlet face must be rejected at assembly");

    // Double coverage of one face must be rejected, not first-come-first-served.
    CompiledCylinderPort port;
    port.cylinderId = 1;
    port.networkEndpoint = { ExhaustEndpointType::ductInlet, 0, 100 };
    port.runnerConnectionAreaM2 = 1.1e-3;
    CompiledExhaustOutlet mouth;
    mouth.outletNodeId = 200;
    mouth.networkEndpoint = { ExhaustEndpointType::ductOutlet, 0, 100 };
    mouth.openingAreaM2 = 1.1e-3;
    const auto doubled = ExhaustNetworkLayout::assemble(
        { runner }, {}, {}, { port, port }, { mouth });
    requireIntake(!doubled.valid(),
                  "two ports claiming one duct face must be rejected at assembly");

    // An uncovered face would make every advance fail; reject at assembly.
    const auto uncovered = ExhaustNetworkLayout::assemble(
        { runner }, {}, {}, { port }, {});
    requireIntake(!uncovered.valid(),
                  "a layout with an uncovered duct face must be rejected");
}

void testDrawingFromThePlenumReservoirConserves() {
    ExhaustGasNetworkConfig configuration;
    configuration.initialPressurePa = 101'325.0;
    configuration.initialTemperatureK = 300.0;
    ExhaustGasNetwork network;
    requireIntake(network.configure(makeRunnerLayout(), configuration),
                  "runner network must configure");

    // Intake stroke: the cylinder sits well below the plenum, the valve is
    // half-open, and the plenum (ambient reservoir) feeds the runner mouth.
    const auto cylinderState = network.mixtureModel().conservativeFromPressureTemperature(
        55'000.0, 330.0);
    requireIntake(cylinderState.has_value(), "cylinder fixture must be physical");
    // A large reservoir volume keeps the draw sustained over the whole window
    // (with the corrected reservoir-inflow boundary, a chamber-sized volume
    // equalises with the plenum before the window ends).
    const CylinderValveBoundary drawing { 1, *cylinderState, 0.05, 5.0e-4, 0.62 };
    const auto plenumState = network.mixtureModel().conservativeFromPressureTemperature(
        98'000.0, 305.0);
    requireIntake(plenumState.has_value(), "plenum fixture must be physical");
    const ExhaustAmbientBoundary plenum { *plenumState, 1.0 };

    const auto before = network.inventory();
    const auto advance = network.advance(0.004,
        std::span<const CylinderValveBoundary>(&drawing, 1), plenum);
    requireIntake(advance.completed && advance.acceptedSubsteps > 0,
                  "a drawing intake runner must advance to completion");
    const auto after = network.inventory();

    const auto& exchange = network.cylinderExchanges().front();
    const auto& mouth = network.outletSamples().front();
    // Signs: exchange is positive cylinder -> network; the mouth sample is
    // positive network -> plenum. An intake stroke therefore reports both
    // negative: the cylinder GAINS mass and the plenum LOSES it.
    requireIntake(exchange.totalMassKg() < 0.0,
                  "an intake stroke must move mass into the cylinder");
    requireIntake(sumSpeciesMass(mouth.speciesMassKg) < 0.0,
                  "an intake stroke must draw mass out of the plenum reservoir");
    for (std::size_t species = 0; species < gasSpeciesCount; ++species) {
        const auto expected = exchange.speciesMassKg[species]
            - mouth.speciesMassKg[species];
        const auto actual = after.speciesMassKg[species]
            - before.speciesMassKg[species];
        requireIntake(std::abs(actual - expected)
                          <= std::max(1.0e-13, std::abs(before.speciesMassKg[species]) * 1.0e-9),
                      "drawing must close the species balance across both boundaries");
    }
    const auto expectedEnergy = exchange.totalEnergyJ - mouth.transferredEnergyJ;
    const auto actualEnergy = after.totalEnergyJ - before.totalEnergyJ;
    requireIntake(std::abs(actualEnergy - expectedEnergy)
                      <= std::max(1.0e-8, std::abs(before.totalEnergyJ) * 1.0e-9),
                  "drawing must close the energy balance across both boundaries");

    // The column must actually be moving toward the valve. Port at the duct
    // inlet means charging flow is NEGATIVE axial velocity in this layout.
    requireIntake(exchange.networkVelocityMps < -1.0,
                  "the runner column must run toward the valve while drawing");
}

// At steady draw the valve is the binding restriction (its effective area is
// well below the duct bore), so the settled mass flow must approach the
// isentropic nozzle flow of the applied pressure ratio. This pins the whole
// boundary chain — mouth reservoir inflow, duct transport, valve nozzle —
// against an independent analytic reference: a systematic deficit here means
// a lossy boundary, not physics.
void testSteadyDrawMatchesIsentropicValveFlow() {
    ExhaustGasNetworkConfig configuration;
    configuration.initialPressurePa = 101'325.0;
    configuration.initialTemperatureK = 300.0;
    ExhaustGasNetwork network;
    requireIntake(network.configure(makeRunnerLayout(), configuration),
                  "runner network must configure for steady draw");

    const auto cylinderState = network.mixtureModel().conservativeFromPressureTemperature(
        55'000.0, 300.0);
    requireIntake(cylinderState.has_value(), "cylinder fixture must be physical");
    // A large frozen reservoir approximates a held cylinder pressure.
    const CylinderValveBoundary drawing { 1, *cylinderState, 1.0, 2.5e-4, 0.62 };
    const auto plenumState = network.mixtureModel().conservativeFromPressureTemperature(
        101'325.0, 300.0);
    requireIntake(plenumState.has_value(), "plenum fixture must be physical");
    const ExhaustAmbientBoundary plenum { *plenumState, 1.0 };

    // Let transients ring down (~20 acoustic transits), then measure.
    for (int window = 0; window < 10; ++window) {
        const auto settle = network.advance(0.003,
            std::span<const CylinderValveBoundary>(&drawing, 1), plenum);
        requireIntake(settle.completed, "steady draw must advance to completion");
    }
    const auto measure = network.advance(0.005,
        std::span<const CylinderValveBoundary>(&drawing, 1), plenum);
    requireIntake(measure.completed, "measurement window must complete");
    const auto measuredKgPerS =
        -network.cylinderExchanges().front().totalMassKg() / 0.005;

    // Isentropic nozzle reference for the same pressure ratio and area. The
    // duct entry and friction cost a few percent; a deficit beyond ~12 %
    // means the boundary chain itself is throttling the draw.
    const auto gamma = 1.4;
    const auto gasConstant = 287.05;
    const auto pressureRatio = 55'000.0 / 101'325.0;
    const auto flowTerm = std::sqrt(std::max(0.0, 2.0 * gamma / (gamma - 1.0)
        * (std::pow(pressureRatio, 2.0 / gamma)
           - std::pow(pressureRatio, (gamma + 1.0) / gamma))));
    const auto referenceKgPerS = 0.62 * 2.5e-4 * 101'325.0
        / std::sqrt(gasConstant * 300.0) * flowTerm;
    std::cout << "  steady draw: measured " << measuredKgPerS
              << " kg/s vs isentropic " << referenceKgPerS
              << " kg/s (ratio " << measuredKgPerS / referenceKgPerS << ")\n";
    // Both measured failure modes stay excluded: the resting-reservoir ghost
    // metered this draw at 0.868 (acoustic-impedance throttling), and the
    // full-face nozzle stress pumped it to 1.064 (free-jet momentum ramming
    // the interior above the reservoir). The corrected boundary reads 0.963,
    // and the shortfall from 1.0 is physical entry/friction loss.
    requireIntake(measuredKgPerS > 0.90 * referenceKgPerS
                      && measuredKgPerS < 1.05 * referenceKgPerS,
                  "steady draw through a wide-open valve must match the "
                  "isentropic nozzle flow of its pressure ratio");
}

void testPortSpeciesInjectionIsConservativeAndGuarded() {
    ExhaustGasNetworkConfig configuration;
    ExhaustGasNetwork network;
    requireIntake(network.configure(makeRunnerLayout(8, 0.027), configuration),
                  "tapered runner network must configure for injection");

    const auto before = network.inventory();
    const auto fuelKg = 2.5e-6;
    const auto fuelTemperatureK = 320.0;
    const auto coolingJ = -0.9;
    requireIntake(network.injectSpeciesAtPort(0, GasSpecies::fuel, fuelKg,
                                              fuelTemperatureK, coolingJ),
                  "a physical port fuel pulse must be accepted");
    const auto after = network.inventory();
    const auto fuelIndex = static_cast<std::size_t>(GasSpecies::fuel);
    requireIntake(std::abs(after.speciesMassKg[fuelIndex]
                               - before.speciesMassKg[fuelIndex] - fuelKg) < 1.0e-15,
                  "injected fuel mass must appear exactly once in the inventory");
    const auto fuelCv = network.mixtureModel().thermodynamics()
        .species[fuelIndex].molarHeatCapacityCvJPerMolK
        / network.mixtureModel().thermodynamics().species[fuelIndex].molarMassKgPerMol;
    const auto expectedEnergyJ = fuelKg * fuelCv * fuelTemperatureK + coolingJ;
    requireIntake(std::abs(after.totalEnergyJ - before.totalEnergyJ - expectedEnergyJ)
                      < std::max(1.0e-10, std::abs(expectedEnergyJ) * 1.0e-9),
                  "injected fuel must carry its sensible energy plus the heat term");

    // The source must not destabilise the following advance.
    const auto quiescent = network.mixtureModel().conservativeFromPressureTemperature(
        101'325.0, 300.0);
    requireIntake(quiescent.has_value(), "quiescent fixture must be physical");
    const CylinderValveBoundary closed { 1, *quiescent, 4.0e-4, 0.0, 0.62 };
    const ExhaustAmbientBoundary plenum { *quiescent, 1.0 };
    const auto advance = network.advance(0.002,
        std::span<const CylinderValveBoundary>(&closed, 1), plenum);
    requireIntake(advance.completed,
                  "the network must advance normally after a port injection");

    // Guards: an unphysical command must be refused without touching state.
    const auto guardBefore = network.inventory();
    requireIntake(!network.injectSpeciesAtPort(0, GasSpecies::fuel, 1.0e-6, 300.0,
                                               -1.0e9),
                  "a heat command that empties the cell must be refused");
    requireIntake(!network.injectSpeciesAtPort(0, GasSpecies::fuel, -1.0e-9, 300.0, 0.0),
                  "negative injected mass must be refused");
    requireIntake(!network.injectSpeciesAtPort(7, GasSpecies::fuel, 1.0e-9, 300.0, 0.0),
                  "an out-of-range port index must be refused");
    const auto guardAfter = network.inventory();
    requireIntake(guardAfter.totalEnergyJ == guardBefore.totalEnergyJ
                      && sumSpeciesMass(guardAfter.speciesMassKg)
                             == sumSpeciesMass(guardBefore.speciesMassKg),
                  "refused injections must leave the state untouched");
}

} // namespace

void runIntakeNetworkTests() {
    testAssembleAcceptsRunnerShapeAndRejectsBadOrientation();
    testDrawingFromThePlenumReservoirConserves();
    testSteadyDrawMatchesIsentropicValveFlow();
    testPortSpeciesInjectionIsConservativeAndGuarded();
}
