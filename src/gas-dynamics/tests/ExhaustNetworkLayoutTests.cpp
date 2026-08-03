#include <enginelab/foundation/EngineTypes.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/gasdynamics/ExhaustNetworkLayout.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <numbers>
#include <numeric>
#include <string_view>

namespace {

using namespace enginelab;
using namespace enginelab::gasdynamics;

void requireLayout(bool condition, std::string_view message) {
    if (condition) return;
    std::cerr << "FAILED: " << message << '\n';
    std::exit(EXIT_FAILURE);
}

[[nodiscard]] ExhaustComponentConfig component(std::uint32_t id,
                                                ExhaustComponentType type,
                                                double lengthMm,
                                                double diameterMm) {
    ExhaustComponentConfig result;
    result.id = id;
    result.type = type;
    result.lengthMm = lengthMm;
    result.diameterMm = diameterMm;
    return result;
}

[[nodiscard]] EngineConfig customNetworkConfig(double audioGain = 1.0) {
    auto config = makeDefaultInlineFour();
    auto& path = config.exhaustPaths.front();
    path.audioVolume = audioGain;
    ExhaustNetworkConfig network;
    for (std::size_t index = 0; index < config.cylinders.size(); ++index) {
        const auto id = static_cast<std::uint32_t>(101 + index);
        auto primary = component(id, ExhaustComponentType::pipe,
                                 440.0 + 10.0 * static_cast<double>(index), 41.0);
        primary.restriction = 0.015;
        primary.dischargeCoefficient = 0.81;
        network.components.push_back(primary);
        network.cylinderConnections.push_back({ config.cylinders[index].id, id });
        network.connections.push_back({ id, 200 });
    }
    auto merge = component(200, ExhaustComponentType::merge, 0.0, 60.0);
    merge.volumeLitres = 0.42;
    network.components.push_back(merge);
    auto muffler = component(300, ExhaustComponentType::muffler, 500.0, 62.0);
    muffler.volumeLitres = 8.0;
    muffler.restriction = 0.24;
    network.components.push_back(muffler);
    auto outlet = component(400, ExhaustComponentType::outlet, 160.0, 70.0);
    outlet.dischargeCoefficient = 0.76;
    network.components.push_back(outlet);
    network.connections.push_back({ 200, 300 });
    network.connections.push_back({ 300, 400 });
    path.network = std::move(network);
    normaliseEngineConfig(config);
    return config;
}

void testAuthoredTopologyCompilation() {
    const auto config = customNetworkConfig();
    requireLayout(!validateEngineConfig(config), "layout fixture must be a valid authored graph");
    const auto graph = ExhaustGraph::makeForEngine(config);
    const auto layout = ExhaustNetworkLayout::compile(graph);
    requireLayout(layout.valid(), "valid authored graph must compile to a physical layout");
    requireLayout(layout.ducts().size() == 6,
                  "four primaries, chamber and outlet must remain distinct ducts");
    requireLayout(layout.junctions().size() == 1,
                  "only the authored merge may become a well-mixed junction");
    requireLayout(layout.interfaces().size() == 6,
                  "all non-cylinder graph edges must survive as physical interfaces");
    requireLayout(layout.cylinderPorts().size() == config.cylinders.size(),
                  "each cylinder must own one valve-to-network boundary");
    requireLayout(layout.outlets().size() == 1,
                  "the authored terminal component must become one physical outlet");

    const auto muffler = std::find_if(layout.ducts().begin(), layout.ducts().end(),
        [](const CompiledExhaustDuct& duct) { return duct.sourceComponentId == 300; });
    requireLayout(muffler != layout.ducts().end(), "muffler duct is missing");
    requireLayout(std::abs(muffler->volumeM3 - 0.008) < 1.0e-12
            && std::abs(muffler->flowAreaM2 - 0.016) < 1.0e-12
            && muffler->areaWasDerivedFromVolume,
        "an explicit chamber volume must determine resolved flow area exactly");
    requireLayout(std::abs(muffler->lossCoefficient - 0.24) < 1.0e-12,
        "the physical solver must receive authored local K without legacy geometric reduction");

    const auto merge = layout.junctions().front();
    requireLayout(std::abs(merge.volumeM3 - 0.00042) < 1.0e-12
            && !merge.volumeWasDerived,
        "an authored junction volume must remain authoritative");
    const auto outlet = layout.outlets().front();
    requireLayout(outlet.networkEndpoint.type == ExhaustEndpointType::ductOutlet
            && std::abs(outlet.dischargeCoefficient - 0.76) < 1.0e-12,
        "outlet area and discharge must be exposed at the terminal duct face");
    requireLayout(std::all_of(layout.cylinderPorts().begin(), layout.cylinderPorts().end(),
        [](const CompiledCylinderPort& port) {
            return port.networkEndpoint.type == ExhaustEndpointType::ductInlet
                && port.runnerConnectionAreaM2 > 0.0
                && std::abs(port.dischargeCoefficient - 0.81) < 1.0e-12;
        }), "authored primary throats must be retained for every cylinder");
    const auto summedCells = std::accumulate(layout.ducts().begin(), layout.ducts().end(),
        std::size_t { 0 }, [](std::size_t sum, const CompiledExhaustDuct& duct) {
            return sum + duct.cellCount;
        });
    requireLayout(summedCells == layout.totalCellCount() && summedCells > 40,
        "reported network cell count must equal the preallocated duct meshes");
}

void testAudioFieldsCannotChangePhysicalLayout() {
    auto quietConfig = customNetworkConfig(0.0);
    auto loudConfig = customNetworkConfig(8.0);
    for (auto& componentConfig : quietConfig.exhaustPaths.front().network->components)
        componentConfig.acousticGain = 0.0;
    for (auto& componentConfig : loudConfig.exhaustPaths.front().network->components)
        componentConfig.acousticGain = 8.0;
    const auto quiet = ExhaustNetworkLayout::compile(ExhaustGraph::makeForEngine(quietConfig));
    const auto loud = ExhaustNetworkLayout::compile(ExhaustGraph::makeForEngine(loudConfig));
    requireLayout(quiet.valid() && loud.valid() && quiet.ducts().size() == loud.ducts().size(),
        "audio-independence fixtures must compile equally");
    for (std::size_t index = 0; index < quiet.ducts().size(); ++index) {
        const auto& first = quiet.ducts()[index];
        const auto& second = loud.ducts()[index];
        requireLayout(first.nodeId == second.nodeId
                && first.lengthM == second.lengthM
                && first.flowAreaM2 == second.flowAreaM2
                && first.lossCoefficient == second.lossCoefficient
                && first.cellCount == second.cellCount,
            "audio gain and path volume must not leak into gas geometry");
    }
}

void testLegacyTopologyCompilation() {
    const auto config = makeDefaultInlineFour();
    const auto graph = ExhaustGraph::makeForEngine(config);
    const auto layout = ExhaustNetworkLayout::compile(graph);
    requireLayout(layout.valid(), "legacy geometry must compile without an authored DAG");
    requireLayout(layout.cylinderPorts().size() == config.cylinders.size(),
        "legacy root pipes must expose implicit cylinder boundaries");
    requireLayout(layout.ducts().size() == config.cylinders.size() + 2
            && layout.junctions().size() == 1 && layout.outlets().size() == 1,
        "legacy geometry must become primaries, merge, chamber and outlet");
    requireLayout(std::all_of(layout.cylinderPorts().begin(), layout.cylinderPorts().end(),
        [](const CompiledCylinderPort& port) {
            return port.networkEndpoint.type == ExhaustEndpointType::ductInlet;
        }), "legacy cylinder IDs must map to primary inlets, not artificial plenums");
}

void testAuthoredTaperSurvivesTopologyCompilation() {
    auto config = customNetworkConfig();
    auto& taperedPrimary =
        config.exhaustPaths.front().network->components.front();
    taperedPrimary.outletDiameterMm = 55.0;
    requireLayout(!validateEngineConfig(config),
        "an authored outlet diameter must be valid exhaust geometry");
    const auto layout =
        ExhaustNetworkLayout::compile(ExhaustGraph::makeForEngine(config));
    const auto compiled = std::find_if(
        layout.ducts().begin(), layout.ducts().end(),
        [&taperedPrimary](const CompiledExhaustDuct& duct) {
            return duct.sourceComponentId == taperedPrimary.id;
        });
    const auto circularArea = [](double diameterMm) {
        return std::numbers::pi
            * std::pow(diameterMm * 0.0005, 2.0);
    };
    const auto inletAreaM2 = circularArea(41.0);
    const auto outletAreaM2 = circularArea(55.0);
    requireLayout(layout.valid() && compiled != layout.ducts().end(),
        "an authored tapered component must compile to one physical duct");
    requireLayout(std::abs(compiled->inletFlowAreaM2 - inletAreaM2) < 1.0e-14
            && std::abs(compiled->outletFlowAreaM2 - outletAreaM2) < 1.0e-14
            && std::abs(compiled->inletConnectionAreaM2 - inletAreaM2) < 1.0e-14
            && std::abs(compiled->outletConnectionAreaM2 - outletAreaM2) < 1.0e-14
            && std::abs(compiled->volumeM3
                - (inletAreaM2 + std::sqrt(inletAreaM2 * outletAreaM2)
                    + outletAreaM2) / 3.0 * 0.44) < 1.0e-14
            && !compiled->areaWasDerivedFromVolume,
        "topology compilation must preserve both taper faces and exact frustum volume");
}

void testZeroLengthResolutionAndHardCellBudget() {
    auto config = customNetworkConfig();
    auto& outlet = config.exhaustPaths.front().network->components.back();
    outlet.lengthMm = 0.0;
    outlet.volumeLitres = 0.0;
    const auto graph = ExhaustGraph::makeForEngine(config);
    const auto resolved = ExhaustNetworkLayout::compile(graph);
    requireLayout(resolved.valid(),
        "a zero-length terminal restriction must receive an explicit, diagnosed thickness");
    requireLayout(std::any_of(resolved.diagnostics().begin(), resolved.diagnostics().end(),
        [](const ExhaustNetworkLayoutDiagnostic& diagnostic) {
            return diagnostic.issue == ExhaustNetworkLayoutIssue::zeroLengthComponentResolved;
        }), "a derived component length must never be silent");

    ExhaustNetworkDiscretisation impossible;
    impossible.maximumTotalCells = 4;
    const auto rejected = ExhaustNetworkLayout::compile(graph, impossible);
    requireLayout(!rejected.valid()
            && std::any_of(rejected.diagnostics().begin(), rejected.diagnostics().end(),
                [](const ExhaustNetworkLayoutDiagnostic& diagnostic) {
                    return diagnostic.issue == ExhaustNetworkLayoutIssue::cellBudgetExceeded;
                }),
        "a hard cell budget must reject an impossible topology instead of dropping ducts");
}

void testShortestCellIsReportedAndTracksOneShortElement() {
    // The shortest cell anywhere sets the explicit time step for the WHOLE
    // network, so it is the number that says what an authored geometry will
    // cost the solver. `cellBudgetExceeded` does not cover this: it guards the
    // TOTAL cell count, and the coarsening loop in `compile` only reacts to
    // that total, never to one duct being far shorter than the target.
    auto config = customNetworkConfig();
    const auto baseline = ExhaustNetworkLayout::compile(
        ExhaustGraph::makeForEngine(config));
    requireLayout(baseline.valid(), "the reference network must compile");

    const auto shortestM = baseline.minimumCellLengthM();
    requireLayout(shortestM > 0.0,
        "a compiled network must report a positive shortest cell");
    auto expected = 0.0;
    for (const auto& duct : baseline.ducts()) {
        if (duct.cellCount == 0 || !(duct.lengthM > 0.0)) continue;
        const auto cellM = duct.lengthM / static_cast<double>(duct.cellCount);
        if (expected == 0.0 || cellM < expected) expected = cellM;
    }
    requireLayout(shortestM == expected,
        "the reported shortest cell must be the minimum over the compiled ducts");

    // Now shorten ONE pipe far below the meshing target. Everything else is
    // untouched, so any change is attributable to that element alone. A pipe is
    // used rather than a chamber deliberately: ExhaustGraph floors a chamber at
    // its own plane-wave resolution limit, and a pipe has no such floor, so
    // this is the case that survives that fix and still needs to be visible.
    auto& components = config.exhaustPaths.front().network->components;
    auto* pipe = static_cast<ExhaustComponentConfig*>(nullptr);
    for (auto& candidate : components) {
        if (candidate.type == ExhaustComponentType::pipe) { pipe = &candidate; break; }
    }
    requireLayout(pipe != nullptr, "the reference network must contain a pipe");
    pipe->lengthMm = 8.0;
    const auto degraded = ExhaustNetworkLayout::compile(
        ExhaustGraph::makeForEngine(config));
    requireLayout(degraded.valid(), "a short pipe must still compile");
    const auto degradedM = degraded.minimumCellLengthM();
    requireLayout(degradedM > 0.0 && degradedM < shortestM,
        "one short element must lower the network's shortest cell");
    // The substep rate is proportional to 1/dx for the same gas, so this ratio
    // is exactly the factor by which the whole exhaust solver gets slower.
    const auto costRatio = shortestM / degradedM;
    requireLayout(costRatio > 2.0,
        "an 8 mm element must show up as a large, visible solver-cost ratio");
    std::cout << "shortest cell: " << shortestM * 1'000.0 << " mm -> "
              << degradedM * 1'000.0 << " mm, solver cost x"
              << costRatio << '\n';
}

} // namespace

void runExhaustNetworkLayoutTests() {
    testAuthoredTopologyCompilation();
    testAudioFieldsCannotChangePhysicalLayout();
    testLegacyTopologyCompilation();
    testAuthoredTaperSurvivesTopologyCompilation();
    testZeroLengthResolutionAndHardCellBudget();
    testShortestCellIsReportedAndTracksOneShortElement();
}
