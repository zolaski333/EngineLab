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
#include <bit>
#include <cmath>
#include <cstdint>
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
    auto fuelBearingCells = std::size_t { 0 };
    for (const auto& cell : network.ducts().front().cells()) {
        if (cell.speciesMassDensityKgPerM3[fuelIndex] > 0.0)
            ++fuelBearingCells;
    }
    requireIntake(fuelBearingCells == 2,
                  "a 31 mm mesh must resolve the 60 mm port-injector footprint "
                  "with two cells, not a fixed cell count");
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

    // The realtime runner mesh is deliberately coarse (~83-95 mm). Its first
    // cell already covers the whole physical injector footprint; spreading to
    // three cells would move fuel most of the way to the plenum and make
    // transient delivery depend on performance settings.
    ExhaustGasNetwork coarseNetwork;
    requireIntake(coarseNetwork.configure(makeRunnerLayout(3), configuration),
                  "coarse realtime runner must configure for injection");
    requireIntake(coarseNetwork.injectSpeciesAtPort(0, GasSpecies::fuel, fuelKg,
                                                    fuelTemperatureK, coolingJ),
                  "coarse realtime runner must accept a port fuel pulse");
    fuelBearingCells = 0;
    for (const auto& cell : coarseNetwork.ducts().front().cells()) {
        if (cell.speciesMassDensityKgPerM3[fuelIndex] > 0.0)
            ++fuelBearingCells;
    }
    requireIntake(fuelBearingCells == 1,
                  "a coarse runner cell must not expand the injector footprint "
                  "to most of the runner");
}

[[nodiscard]] bool sameBits(double left, double right) noexcept {
    return std::bit_cast<std::uint64_t>(left) == std::bit_cast<std::uint64_t>(right);
}

[[nodiscard]] bool sameBits(const ConservativeState& left,
                            const ConservativeState& right) noexcept {
    for (std::size_t species = 0; species < gasSpeciesCount; ++species)
        if (!sameBits(left.speciesMassDensityKgPerM3[species],
                      right.speciesMassDensityKgPerM3[species]))
            return false;
    return sameBits(left.momentumDensityKgPerM2S, right.momentumDensityKgPerM2S)
        && sameBits(left.totalEnergyDensityJPerM3, right.totalEnergyDensityJPerM3);
}

// Runs a parallel-for's indices last to first on the calling thread: any
// order-dependent write inside a per-duct pass then changes the result.
struct ReversedExecutor final {
    std::size_t dispatches { 0 };
    static void run(void* executor, std::size_t count,
                    ExhaustGasNetwork::ParallelFor::Body body, void* context) noexcept {
        ++static_cast<ReversedExecutor*>(executor)->dispatches;
        for (auto index = count; index > 0; --index) body(context, index - 1);
    }
};

// A joint manifold (six runners sharing one finite plenum, characteristic
// valve faces, one valve opening and one closing) must give bit-identical
// states and transfers whatever order the per-duct pass runs in.
void testPerDuctPassIsOrderIndependent() {
    std::vector<CompiledExhaustDuct> runners;
    std::vector<CompiledCylinderPort> ports;
    std::vector<CompiledExhaustOutlet> mouths;
    const auto single = makeRunnerLayout(5);
    constexpr std::size_t runnerCount = 6;
    for (std::uint32_t index = 0; index < runnerCount; ++index) {
        auto runner = single.ducts().front();
        runner.nodeId = 100 + index;
        runner.lengthM = 0.20 + 0.05 * index;
        runner.volumeM3 = runner.flowAreaM2 * runner.lengthM;
        runners.push_back(runner);
        auto port = single.cylinderPorts().front();
        port.cylinderId = index + 1;
        port.networkEndpoint = { ExhaustEndpointType::ductInlet, index, runner.nodeId };
        ports.push_back(port);
        auto mouth = single.outlets().front();
        mouth.outletNodeId = 200 + index;
        mouth.networkEndpoint = { ExhaustEndpointType::ductOutlet, index, runner.nodeId };
        mouths.push_back(mouth);
    }
    const auto layout = ExhaustNetworkLayout::assemble(runners, {}, {}, ports, mouths);
    requireIntake(layout.valid(), "a six-runner manifold must assemble");
    ExhaustGasNetworkConfig configuration;
    configuration.characteristicValveBoundary = true;
    ExhaustGasNetwork serial;
    ExhaustGasNetwork reversed;
    requireIntake(serial.configure(layout, configuration)
                      && reversed.configure(layout, configuration),
                  "the manifold networks must configure");
    ReversedExecutor executor;
    reversed.setParallelFor({ &ReversedExecutor::run, &executor });

    const auto& model = serial.mixtureModel();
    auto plenum = model.conservativeFromPressureTemperature(97'000.0, 305.0);
    requireIntake(plenum.has_value(), "plenum fixture must be physical");
    for (int window = 0; window < 40; ++window) {
        std::array<CylinderValveBoundary, runnerCount> boundaries {};
        for (std::size_t index = 0; index < runnerCount; ++index) {
            const auto state = model.conservativeFromPressureTemperature(
                50'000.0 + 15'000.0 * index + 500.0 * window, 320.0 + 20.0 * index);
            requireIntake(state.has_value(), "cylinder fixture must be physical");
            boundaries[index] = { static_cast<std::uint32_t>(index + 1), *state,
                4.0e-4, 1.5e-4 * static_cast<double>(index % 3), 0.62 };
        }
        boundaries[1].effectiveValveAreaRateM2PerS = -0.4;
        boundaries[1].valveClosesAfterSeconds = 2.0e-4;
        const ExhaustAmbientBoundary reservoir { *plenum, 1.0, 1.5e-3 };
        const auto serialAdvance = serial.advance(3.3e-4, boundaries, reservoir);
        const auto reversedAdvance = reversed.advance(3.3e-4, boundaries, reservoir);
        requireIntake(serialAdvance.completed && reversedAdvance.completed
                          && serialAdvance.acceptedSubsteps
                              == reversedAdvance.acceptedSubsteps,
                      "both manifolds must advance through the same substeps");
        for (std::size_t duct = 0; duct < runnerCount; ++duct) {
            const auto serialCells = serial.ducts()[duct].cells();
            const auto reversedCells = reversed.ducts()[duct].cells();
            for (std::size_t cell = 0; cell < serialCells.size(); ++cell)
                requireIntake(sameBits(serialCells[cell], reversedCells[cell]),
                              "a reordered per-duct pass must leave every cell "
                              "bit-identical");
        }
        for (std::size_t index = 0; index < runnerCount; ++index) {
            requireIntake(sameBits(serial.cylinderExchanges()[index].totalEnergyJ,
                                   reversed.cylinderExchanges()[index].totalEnergyJ)
                              && sameBits(serial.outletSamples()[index].transferredEnergyJ,
                                          reversed.outletSamples()[index].transferredEnergyJ),
                          "a reordered per-duct pass must leave every transfer "
                          "bit-identical");
        }
    }
    requireIntake(executor.dispatches > 0,
                  "the reversed network must actually dispatch through its executor");
}

} // namespace

void runIntakeNetworkTests() {
    testAssembleAcceptsRunnerShapeAndRejectsBadOrientation();
    testDrawingFromThePlenumReservoirConserves();
    testSteadyDrawMatchesIsentropicValveFlow();
    testPortSpeciesInjectionIsConservativeAndGuarded();
    testPerDuctPassIsOrderIndependent();
}
