// A closed four-port X crossover between four pipes. The junction's explicit
// step bound must follow the same convention as the duct cells around it, so
// a compact crossover does not set the step of the whole network, and the
// closed system must stay passive at that step.
#include <enginelab/gasdynamics/ExhaustGasNetwork.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <string_view>

namespace {
using namespace enginelab::gasdynamics;

constexpr double pipeLengthM = 0.32;
constexpr double pipeDiameterM = 0.08;
constexpr double referencePressurePa = 101'325.0;
constexpr double referenceTemperatureK = 600.0;
constexpr double pressureAmplitudePa = 200.0;
constexpr double traversalCount = 300.0;

void require(bool condition, std::string_view message) {
    if (condition) return;
    std::cerr << "FAILED: " << message << '\n';
    std::exit(EXIT_FAILURE);
}

// Two pipes run from closed cylinder ports into the crossover and two run out
// of it to outlets that the ambient boundary keeps shut.
[[nodiscard]] ExhaustNetworkLayout crossoverLayout(std::size_t cellsPerPipe) {
    const auto areaM2 = std::numbers::pi * pipeDiameterM * pipeDiameterM * 0.25;
    std::vector<CompiledExhaustDuct> ducts;
    for (std::uint32_t index = 0; index < 4; ++index) {
        CompiledExhaustDuct duct;
        duct.nodeId = 100 + index;
        duct.lengthM = pipeLengthM;
        duct.flowAreaM2 = areaM2;
        duct.inletFlowAreaM2 = areaM2;
        duct.outletFlowAreaM2 = areaM2;
        duct.connectionAreaM2 = areaM2;
        duct.inletConnectionAreaM2 = areaM2;
        duct.outletConnectionAreaM2 = areaM2;
        duct.hydraulicDiameterM = pipeDiameterM;
        duct.volumeM3 = areaM2 * pipeLengthM;
        duct.cellCount = cellsPerPipe;
        ducts.push_back(duct);
    }
    CompiledExhaustJunction crossover;
    crossover.nodeId = 300;
    crossover.sourceType = enginelab::ExhaustNodeType::crossover;
    // The derived compact volume of a crossover: two duct sections, 2 A d.
    crossover.volumeM3 = 2.0 * areaM2 * pipeDiameterM;
    crossover.characteristicDiameterM = pipeDiameterM;
    crossover.volumeWasDerived = true;
    std::vector<CompiledExhaustInterface> interfaces {
        { { ExhaustEndpointType::ductOutlet, 0, 100 }, { ExhaustEndpointType::junction, 0, 300 } },
        { { ExhaustEndpointType::ductOutlet, 1, 101 }, { ExhaustEndpointType::junction, 0, 300 } },
        { { ExhaustEndpointType::junction, 0, 300 }, { ExhaustEndpointType::ductInlet, 2, 102 } },
        { { ExhaustEndpointType::junction, 0, 300 }, { ExhaustEndpointType::ductInlet, 3, 103 } },
    };
    std::vector<CompiledCylinderPort> ports;
    for (std::uint32_t index = 0; index < 2; ++index) {
        CompiledCylinderPort port;
        port.cylinderId = index + 1;
        port.networkEndpoint = { ExhaustEndpointType::ductInlet, index, 100 + index };
        port.runnerConnectionAreaM2 = areaM2;
        ports.push_back(port);
    }
    std::vector<CompiledExhaustOutlet> outlets;
    for (std::uint32_t index = 2; index < 4; ++index) {
        CompiledExhaustOutlet outlet;
        outlet.outletNodeId = 200 + index;
        outlet.networkEndpoint = { ExhaustEndpointType::ductOutlet, index, 100 + index };
        outlet.openingAreaM2 = areaM2;
        outlets.push_back(outlet);
    }
    return ExhaustNetworkLayout::assemble(std::move(ducts), { crossover },
        std::move(interfaces), std::move(ports), std::move(outlets));
}

// Acoustic energy about the volume-weighted mean pressure, junction included.
[[nodiscard]] double acousticEnergyJ(const ExhaustGasNetwork& network,
                                     double densityKgPerM3, double soundSpeedMps) {
    const auto& model = network.mixtureModel();
    struct Sample { double volumeM3; double pressurePa; double velocityMps; };
    std::vector<Sample> samples;
    for (std::size_t duct = 0; duct < network.ducts().size(); ++duct) {
        const auto cells = network.ducts()[duct].cells();
        const auto cellVolumeM3 = network.layout().ducts()[duct].volumeM3
            / static_cast<double>(cells.size());
        for (const auto& cell : cells) {
            const auto primitive = model.primitiveFromConservative(cell);
            require(primitive.has_value(), "every pipe cell must remain physical");
            samples.push_back({ cellVolumeM3, primitive->pressurePa, primitive->velocityMps });
        }
    }
    const auto junction = model.primitiveFromConservative(network.junctionStates().front());
    require(junction.has_value(), "the crossover must remain physical");
    samples.push_back({ network.layout().junctions().front().volumeM3, junction->pressurePa, 0.0 });
    auto volumeM3 = 0.0;
    auto pressureVolume = 0.0;
    for (const auto& sample : samples) {
        volumeM3 += sample.volumeM3;
        pressureVolume += sample.volumeM3 * sample.pressurePa;
    }
    const auto meanPa = pressureVolume / volumeM3;
    const auto stiffness = densityKgPerM3 * soundSpeedMps * soundSpeedMps;
    auto energyJ = 0.0;
    for (const auto& sample : samples) {
        const auto perturbation = sample.pressurePa - meanPa;
        energyJ += 0.5 * sample.volumeM3 * (perturbation * perturbation / stiffness
            + densityKgPerM3 * sample.velocityMps * sample.velocityMps);
    }
    require(std::isfinite(energyJ), "the acoustic energy must remain finite");
    return energyJ;
}

struct Result final {
    double meanStepSeconds { 0.0 };
    double ductStepSeconds { 0.0 };
    double junctionStepSeconds { 0.0 };
    double maximumEnergyRatio { 1.0 };
    double finalEnergyRatio { 1.0 };
};

[[nodiscard]] Result run(std::size_t cellsPerPipe) {
    ExhaustGasNetworkConfig config;
    config.initialPressurePa = referencePressurePa;
    config.initialTemperatureK = referenceTemperatureK;
    config.wallHeatTransferWPerM2K = 0.0;
    config.dynamicWallHeatTransferEnabled = false;
    config.externalWallHeatTransferWPerM2K = 0.0;
    ExhaustGasNetwork network;
    require(network.configure(crossoverLayout(cellsPerPipe), config), "the crossover must configure");
    const auto& model = network.mixtureModel();
    const auto base = model.conservativeFromPressureTemperature(
        referencePressurePa, referenceTemperatureK);
    require(base.has_value(), "the reference state must be physical");
    const auto basePrimitive = model.primitiveFromConservative(*base);
    require(basePrimitive.has_value(), "the reference state must recover");

    // A half-cosine pressure mode in the first inbound pipe.
    auto cells = network.ducts().front().cells();
    for (std::size_t index = 0; index < cells.size(); ++index) {
        const auto fraction = (static_cast<double>(index) + 0.5)
            / static_cast<double>(cells.size());
        const auto pressurePa = referencePressurePa
            + pressureAmplitudePa * std::cos(std::numbers::pi * fraction);
        const auto density = basePrimitive->densityKgPerM3 * std::pow(
            pressurePa / referencePressurePa, 1.0 / basePrimitive->heatCapacityRatio);
        const auto state = model.conservativeFromPrimitive(
            density, 0.0, pressurePa, GasComposition::dryAir());
        require(state.has_value(), "the perturbation must be physical");
        cells[index] = *state;
    }
    const auto initialEnergyJ = acousticEnergyJ(network,
        basePrimitive->densityKgPerM3, basePrimitive->speedOfSoundMps);
    require(initialEnergyJ > 0.0, "the fixture must carry a mode");

    const ExhaustAmbientBoundary closed { *base, 0.0 };
    const auto cellLengthM = pipeLengthM / static_cast<double>(cellsPerPipe);
    const auto ductStepSeconds = config.maximumCourantNumber * cellLengthM
        / basePrimitive->speedOfSoundMps;
    const auto totalSeconds = traversalCount * pipeLengthM / basePrimitive->speedOfSoundMps;
    constexpr double callSeconds = 1.0e-3;
    std::size_t substeps = 0;
    auto maximumRatio = 1.0;
    for (auto elapsed = 0.0; elapsed < totalSeconds; elapsed += callSeconds) {
        const auto advance = network.advance(callSeconds, {}, closed);
        require(advance.completed, "every advance must complete");
        substeps += advance.acceptedSubsteps;
        maximumRatio = std::max(maximumRatio, acousticEnergyJ(network,
            basePrimitive->densityKgPerM3, basePrimitive->speedOfSoundMps) / initialEnergyJ);
    }
    const auto calls = std::ceil(totalSeconds / callSeconds);
    Result result;
    result.meanStepSeconds = calls * callSeconds / static_cast<double>(substeps);
    result.ductStepSeconds = ductStepSeconds;
    // Four ports of the pipe's area around 2 A d: 2 C V / (4 A c) = C d / c.
    result.junctionStepSeconds = config.maximumCourantNumber * pipeDiameterM
        / basePrimitive->speedOfSoundMps;
    result.maximumEnergyRatio = maximumRatio;
    result.finalEnergyRatio = acousticEnergyJ(network,
        basePrimitive->densityKgPerM3, basePrimitive->speedOfSoundMps) / initialEnergyJ;
    std::cout << std::setprecision(4) << cellsPerPipe << " cells per pipe: mean step "
              << result.meanStepSeconds * 1.0e6 << " us, duct bound "
              << result.ductStepSeconds * 1.0e6 << " us, crossover bound "
              << result.junctionStepSeconds * 1.0e6 << " us, energy ratio maximum "
              << result.maximumEnergyRatio << " final " << result.finalEnergyRatio << '\n';
    return result;
}

} // namespace

int main() {
    // 80 mm cells, one pipe diameter: the crossover is no stiffer than they
    // are. It used to take half their step and set the whole network's.
    const auto matched = run(4);
    require(matched.meanStepSeconds > 0.9 * matched.ductStepSeconds,
        "the crossover must not set a shorter step than the pipes around it");
    require(matched.maximumEnergyRatio < 1.02, "the closed crossover must stay passive");
    // 160 mm cells: now the crossover binds, and must stay passive at its
    // own bound.
    const auto bound = run(2);
    require(bound.meanStepSeconds < 1.01 * bound.junctionStepSeconds
            && bound.meanStepSeconds > 0.9 * bound.junctionStepSeconds,
        "the crossover must set the step when its pipes' cells are longer than it");
    require(bound.maximumEnergyRatio < 1.02,
        "the closed crossover must stay passive at its own step bound");
    std::cout << "exhaust junction step: passed\n";
    return 0;
}
