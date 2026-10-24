// An exhaust of two unconnected pipes (PartitionedExhaustNetwork), each fed by
// its own held tank: pipe A is two ducts in line, pipe B a duct, a junction and
// a duct, and the compiled order interleaves them (ducts A0 B0 A1 B1, the
// ports and outlets B before A). Partitioned, each pipe must evolve exactly as
// a network built of that pipe alone, whatever order an executor runs them
// in, and every transfer must come back at its compiled index. Unpartitioned,
// it must be exactly the whole network. A cylinder ported into both pipes
// joins them, and a finite ambient reservoir, which the parts cannot share,
// is refused.
#include <enginelab/gasdynamics/PartitionedExhaustNetwork.hpp>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <numbers>
#include <string_view>
#include <vector>

namespace {
using namespace enginelab::gasdynamics;
constexpr double ambientPressurePa = 100'000.0;
constexpr double stepSeconds = 100.0e-6;
constexpr std::size_t steps = 300;

void require(bool condition, std::string_view message) {
    if (condition) return;
    std::cerr << "FAILED: " << message << '\n';
    std::exit(EXIT_FAILURE);
}

[[nodiscard]] CompiledExhaustDuct duct(std::uint32_t id, double lengthM, double boreM,
                                       std::size_t cells) {
    const auto areaM2 = std::numbers::pi * boreM * boreM * 0.25;
    CompiledExhaustDuct result;
    result.nodeId = id;
    result.lengthM = lengthM;
    result.flowAreaM2 = areaM2;
    result.connectionAreaM2 = areaM2;
    result.hydraulicDiameterM = boreM;
    result.volumeM3 = lengthM * areaM2;
    result.cellCount = cells;
    return result;
}

[[nodiscard]] CompiledCylinderPort port(std::uint32_t cylinderId, std::size_t ductIndex,
                                        std::uint32_t nodeId, double boreM) {
    CompiledCylinderPort result;
    result.cylinderId = cylinderId;
    result.networkEndpoint = { ExhaustEndpointType::ductInlet, ductIndex, nodeId };
    result.runnerConnectionAreaM2 = std::numbers::pi * boreM * boreM * 0.25;
    return result;
}

[[nodiscard]] CompiledExhaustOutlet outlet(std::uint32_t id, std::size_t ductIndex,
                                           std::uint32_t nodeId, double boreM) {
    CompiledExhaustOutlet result;
    result.outletNodeId = id;
    result.networkEndpoint = { ExhaustEndpointType::ductOutlet, ductIndex, nodeId };
    result.openingAreaM2 = std::numbers::pi * boreM * boreM * 0.25;
    return result;
}

[[nodiscard]] CompiledExhaustJunction junction() {
    CompiledExhaustJunction result;
    result.nodeId = 15;
    result.volumeM3 = 0.3e-3;
    result.characteristicDiameterM = 0.050;
    return result;
}

[[nodiscard]] CompiledExhaustInterface face(ExhaustEndpoint upstream, ExhaustEndpoint downstream) {
    CompiledExhaustInterface result;
    result.upstream = upstream;
    result.downstream = downstream;
    return result;
}

// Pipe A: ducts 1 and 2 from cylinder 1 to outlet 3. Pipe B: duct 11, the
// junction, duct 12, from cylinder 2 to outlet 13.
[[nodiscard]] CompiledExhaustDuct a0() { return duct(1, 0.20, 0.040, 4); }
[[nodiscard]] CompiledExhaustDuct a1() { return duct(2, 0.45, 0.040, 6); }
[[nodiscard]] CompiledExhaustDuct b0() { return duct(11, 0.30, 0.032, 3); }
[[nodiscard]] CompiledExhaustDuct b1() { return duct(12, 0.50, 0.045, 5); }

[[nodiscard]] ExhaustNetworkLayout wholeLayout(bool sharedCylinder) {
    std::vector<CompiledCylinderPort> ports { port(2, 1, 11, 0.032), port(1, 0, 1, 0.040) };
    // Cylinder 1 also exhausts into pipe B, through a third duct of its own
    // that joins B's junction.
    std::vector<CompiledExhaustDuct> ducts { a0(), b0(), a1(), b1() };
    std::vector<CompiledExhaustInterface> faces {
        face({ ExhaustEndpointType::ductOutlet, 0, 1 }, { ExhaustEndpointType::ductInlet, 2, 2 }),
        face({ ExhaustEndpointType::ductOutlet, 1, 11 }, { ExhaustEndpointType::junction, 0, 15 }),
        face({ ExhaustEndpointType::junction, 0, 15 }, { ExhaustEndpointType::ductInlet, 3, 12 }),
    };
    if (sharedCylinder) {
        ducts.push_back(duct(21, 0.25, 0.030, 3));
        ports.push_back(port(1, 4, 21, 0.030));
        faces.push_back(face({ ExhaustEndpointType::ductOutlet, 4, 21 },
                             { ExhaustEndpointType::junction, 0, 15 }));
    }
    return ExhaustNetworkLayout::assemble(std::move(ducts), { junction() }, std::move(faces),
        std::move(ports), { outlet(13, 3, 12, 0.045), outlet(3, 2, 2, 0.040) });
}

[[nodiscard]] ExhaustNetworkLayout pipeA() {
    return ExhaustNetworkLayout::assemble({ a0(), a1() }, {},
        { face({ ExhaustEndpointType::ductOutlet, 0, 1 }, { ExhaustEndpointType::ductInlet, 1, 2 }) },
        { port(1, 0, 1, 0.040) }, { outlet(3, 1, 2, 0.040) });
}

[[nodiscard]] ExhaustNetworkLayout pipeB() {
    return ExhaustNetworkLayout::assemble({ b0(), b1() }, { junction() },
        { face({ ExhaustEndpointType::ductOutlet, 0, 11 }, { ExhaustEndpointType::junction, 0, 15 }),
          face({ ExhaustEndpointType::junction, 0, 15 }, { ExhaustEndpointType::ductInlet, 1, 12 }) },
        { port(2, 0, 11, 0.032) }, { outlet(13, 1, 12, 0.045) });
}

[[nodiscard]] ExhaustGasNetworkConfig rigConfig() {
    ExhaustGasNetworkConfig config;
    config.initialPressurePa = ambientPressurePa;
    config.initialTemperatureK = 300.0;
    config.dynamicWallHeatTransferEnabled = true;
    config.wallTemperatureK = 300.0;
    config.externalWallHeatTransferWPerM2K = 10.0;
    config.reconstructAcrossInterfaces = true;
    return config;
}

struct Tanks final {
    CylinderValveBoundary one {};
    CylinderValveBoundary two {};
    ConservativeState air {};
};

[[nodiscard]] Tanks tanks(const EulerMixtureModel& model) {
    const auto hot = model.conservativeFromPressureTemperature(160'000.0, 900.0);
    const auto warm = model.conservativeFromPressureTemperature(135'000.0, 700.0);
    const auto air = model.conservativeFromPressureTemperature(ambientPressurePa, 300.0);
    require(hot && warm && air, "the reference states must recover");
    const auto areaM2 = std::numbers::pi * 0.030 * 0.030 * 0.25;
    return { { 1, *hot, 0.5, areaM2, 0.8 }, { 2, *warm, 0.5, areaM2, 0.8 }, *air };
}

[[nodiscard]] bool sameCells(const FiniteVolumeDuct& first, const FiniteVolumeDuct& second) {
    const auto a = first.cells();
    const auto b = second.cells();
    if (a.size() != b.size()) return false;
    for (std::size_t cell = 0; cell < a.size(); ++cell) {
        if (a[cell].momentumDensityKgPerM2S != b[cell].momentumDensityKgPerM2S
            || a[cell].totalEnergyDensityJPerM3 != b[cell].totalEnergyDensityJPerM3
            || a[cell].speciesMassDensityKgPerM3 != b[cell].speciesMassDensityKgPerM3)
            return false;
    }
    return true;
}

[[nodiscard]] double massKg(const ExhaustNetworkInventory& inventory) {
    auto total = 0.0;
    for (const auto species : inventory.speciesMassKg) total += species;
    return total;
}

// Runs indices last to first, as a pool's threads might claim them.
void runBackwards(void*, std::size_t count, ExhaustGasNetwork::ParallelFor::Body body,
                  void* context) noexcept {
    for (std::size_t index = count; index-- > 0;) body(context, index);
}

void partsEvolveAsTheirOwnNetworks(bool throughExecutor) {
    PartitionedExhaustNetwork partitioned;
    require(partitioned.configure(wholeLayout(false), rigConfig(), true), "the pipes must configure");
    require(partitioned.partCount() == 2, "two unconnected pipes are two parts");
    if (throughExecutor) {
        ExhaustGasNetwork::ParallelFor parallelFor;
        parallelFor.run = runBackwards;
        parallelFor.threads = 2;
        partitioned.setParallelFor(parallelFor);
    }
    ExhaustGasNetwork alone[2];
    require(alone[0].configure(pipeA(), rigConfig()) && alone[1].configure(pipeB(), rigConfig()),
        "each pipe must configure alone");
    const auto rig = tanks(partitioned.mixtureModel());
    // Compiled port order: cylinder 2, then cylinder 1.
    const CylinderValveBoundary both[] { rig.two, rig.one };
    const ExhaustAmbientBoundary ambient { rig.air, 1.0, 0.0 };
    auto suppliedKg = 0.0;
    auto releasedKg = 0.0;
    const auto initialKg = massKg(partitioned.inventory());
    for (std::size_t step = 0; step < steps; ++step) {
        const auto whole = partitioned.advance(stepSeconds, both, ambient);
        const auto first = alone[0].advance(stepSeconds, std::span(&rig.one, 1), ambient);
        const auto second = alone[1].advance(stepSeconds, std::span(&rig.two, 1), ambient);
        require(whole.completed && first.completed && second.completed, "every advance must complete");
        require(whole.acceptedSubsteps == std::max(first.acceptedSubsteps, second.acceptedSubsteps),
            "the substeps are the busiest part's");
        const auto exchanges = partitioned.cylinderExchanges();
        require(exchanges.size() == 2 && exchanges[0].cylinderId == 2 && exchanges[1].cylinderId == 1,
            "the exchanges come back in compiled port order");
        require(exchanges[1].totalMassKg() == alone[0].cylinderExchanges()[0].totalMassKg()
                && exchanges[0].totalMassKg() == alone[1].cylinderExchanges()[0].totalMassKg(),
            "each exchange is its pipe's");
        const auto outlets = partitioned.outletSamples();
        require(outlets.size() == 2 && outlets[0].outletNodeId == 13 && outlets[1].outletNodeId == 3
                && outlets[1].massFlowKgPerS == alone[0].outletSamples()[0].massFlowKgPerS
                && outlets[0].massFlowKgPerS == alone[1].outletSamples()[0].massFlowKgPerS,
            "the outlets come back in compiled order, each its pipe's");
        for (const auto& exchange : exchanges) suppliedKg += exchange.totalMassKg();
        for (const auto& sample : outlets)
            for (const auto species : sample.speciesMassKg) releasedKg += species;
    }
    require(sameCells(partitioned.duct(0), alone[0].ducts()[0])
            && sameCells(partitioned.duct(2), alone[0].ducts()[1])
            && sameCells(partitioned.duct(1), alone[1].ducts()[0])
            && sameCells(partitioned.duct(3), alone[1].ducts()[1]),
        "each pipe evolves exactly as a network of its own");
    require(partitioned.junctionState(0).totalEnergyDensityJPerM3
            == alone[1].junctionStates()[0].totalEnergyDensityJPerM3,
        "the junction is pipe B's");
    const auto errorKg = massKg(partitioned.inventory()) - initialKg - suppliedKg + releasedKg;
    require(std::abs(errorKg) < 1.0e-9 * initialKg, "the parts together conserve mass");
    require(suppliedKg > 0.0 && releasedKg > 0.0, "the tanks blow down through the pipes");

    CylinderBoundaryFlowSample samples[2] {};
    CylinderBoundaryFlowSample sampleA {};
    CylinderBoundaryFlowSample sampleB {};
    require(partitioned.sampleCylinderBoundaries(both, samples)
            && alone[0].sampleCylinderBoundaries(std::span(&rig.one, 1), std::span(&sampleA, 1))
            && alone[1].sampleCylinderBoundaries(std::span(&rig.two, 1), std::span(&sampleB, 1)),
        "the valve planes must sample");
    require(samples[1].massFlowKgPerSecond == sampleA.massFlowKgPerSecond
            && samples[0].massFlowKgPerSecond == sampleB.massFlowKgPerSecond,
        "each valve-plane sample is its pipe's, at its compiled index");

    const ExhaustAmbientBoundary finite { rig.air, 1.0, 0.01 };
    require(!partitioned.advance(stepSeconds, both, finite).completed,
        "the parts cannot share a finite reservoir");
}

void unpartitionedIsTheWholeNetwork() {
    PartitionedExhaustNetwork single;
    require(single.configure(wholeLayout(false), rigConfig(), false), "the pipes must configure");
    require(single.partCount() == 1, "unpartitioned is one network");
    ExhaustGasNetwork whole;
    require(whole.configure(wholeLayout(false), rigConfig()), "the whole network must configure");
    const auto rig = tanks(whole.mixtureModel());
    const CylinderValveBoundary both[] { rig.two, rig.one };
    const ExhaustAmbientBoundary ambient { rig.air, 1.0, 0.0 };
    for (std::size_t step = 0; step < steps; ++step) {
        require(single.advance(stepSeconds, both, ambient).completed
                && whole.advance(stepSeconds, both, ambient).completed,
            "every advance must complete");
    }
    for (std::size_t index = 0; index < whole.ducts().size(); ++index)
        require(sameCells(single.duct(index), whole.ducts()[index]),
            "unpartitioned is bit-identical to the whole network");
}

void aSharedCylinderJoinsThePipes() {
    PartitionedExhaustNetwork joined;
    require(joined.configure(wholeLayout(true), rigConfig(), true), "the joined pipes must configure");
    require(joined.partCount() == 1, "a cylinder ported into both pipes makes them one part");
}

} // namespace

int main() {
    partsEvolveAsTheirOwnNetworks(false);
    partsEvolveAsTheirOwnNetworks(true);
    unpartitionedIsTheWholeNetwork();
    aSharedCylinderJoinsThePipes();
    std::cout << "PartitionedExhaustNetwork: all checks passed\n";
    return EXIT_SUCCESS;
}
