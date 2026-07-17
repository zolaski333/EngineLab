#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/serialization/JsonEngineSerializer.hpp>
#include <enginelab/serialization/YamlEngineSerializer.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using namespace enginelab;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

[[nodiscard]] ExhaustComponentConfig component(std::uint32_t id, ExhaustComponentType type,
                                                double lengthMm, double diameterMm,
                                                double restriction = 0.0) {
    ExhaustComponentConfig value;
    value.id = id;
    value.type = type;
    value.lengthMm = lengthMm;
    value.diameterMm = diameterMm;
    value.restriction = restriction;
    return value;
}

[[nodiscard]] EngineConfig makeCustomExhaust() {
    auto config = makeDefaultInlineFour();
    auto& path = config.exhaustPaths.front();
    ExhaustNetworkConfig network;
    for (std::size_t index = 0; index < config.cylinders.size(); ++index) {
        const auto componentId = static_cast<std::uint32_t>(101U + index);
        network.components.push_back(component(componentId, ExhaustComponentType::pipe,
            430.0 + static_cast<double>(index) * 12.0, 42.0));
        network.cylinderConnections.push_back({ config.cylinders[index].id, componentId });
        network.connections.push_back({ componentId, 200 });
    }
    network.components.push_back(component(200, ExhaustComponentType::merge, 0.0, 60.0));
    network.components.push_back(component(300, ExhaustComponentType::splitter, 0.0, 60.0));
    network.components.push_back(component(400, ExhaustComponentType::muffler, 480.0, 62.0, 0.20));
    network.components.push_back(component(401, ExhaustComponentType::catalyst, 180.0, 58.0, 0.08));
    network.components.push_back(component(500, ExhaustComponentType::outlet, 160.0, 70.0));
    network.components.push_back(component(501, ExhaustComponentType::outlet, 210.0, 58.0));
    network.connections.push_back({ 200, 300 });
    network.connections.push_back({ 300, 400 });
    network.connections.push_back({ 300, 401 });
    network.connections.push_back({ 400, 500 });
    network.connections.push_back({ 401, 501 });
    path.network = std::move(network);
    normaliseEngineConfig(config);
    return config;
}

void testValidationAndRouting() {
    const auto config = makeCustomExhaust();
    require(!validateEngineConfig(config).has_value(), "valid split custom exhaust rejected");
    const auto graph = ExhaustGraph::makeForEngine(config);
    require(graph.nodes().size() == 14, "explicit graph did not preserve ports and components");
    require(graph.edges().size() == 13, "explicit graph connections were not compiled");
    require(graph.routes().size() == 8, "splitter must create two routes per cylinder");
    for (const auto& cylinder : config.cylinders) {
        const auto routeCount = std::count_if(graph.routes().begin(), graph.routes().end(),
            [&cylinder](const ExhaustRoute& route) { return route.cylinderId == cylinder.id; });
        require(routeCount == 2, "each cylinder must reach both custom outlets");
    }
    require(std::all_of(graph.routes().begin(), graph.routes().end(), [](const ExhaustRoute& route) {
        return route.lengthMm > 500.0 && route.delaySeconds > 0.001
            && route.restriction > 0.0 && route.audioGain > 0.0;
    }), "route metrics were not compiled");
    const auto pathFlow = graph.pathFlowProperties(0);
    require(pathFlow.authoredNetwork && pathFlow.collectorVolumeLitres > 0.0
            && pathFlow.effectiveOutletAreaM2 > 0.0
            && pathFlow.equivalentRestriction > 0.0,
        "custom DAG did not compile finite physical collector/outlet properties");
    for (const auto& cylinder : config.cylinders) {
        const auto cylinderFlow = graph.cylinderFlowProperties(cylinder.id);
        require(cylinderFlow.authoredNetwork && cylinderFlow.cylinderId == cylinder.id
                && cylinderFlow.pathIndex == 0 && cylinderFlow.inletAreaM2 > 0.0
                && cylinderFlow.inletDischargeCoefficient > 0.0,
            "custom DAG did not compile the cylinder inlet throat");
    }

    FiringEvent event;
    event.cylinderId = config.cylinders.front().id;
    event.exhaustPortId = event.cylinderId;
    event.intensity = 1.0F;
    const auto metrics = graph.acousticsForCylinder(event.cylinderId);
    graph.process(event);
    require(event.exhaustDelaySeconds > 0.001F, "event did not receive graph propagation delay");
    require(event.exhaustResonanceHz > 0.0F, "event did not receive route resonance");
    require(event.intensity == 1.0F,
        "exhaust topology must not alter the direct combustion event intensity");
    require(metrics.routeCount == 2
            && std::abs(event.exhaustTransmissionGain - metrics.transmissionGain) < 1.0e-6,
        "event and continuous-pressure graph metrics must share one transmission gain");
    require(event.exhaustTransmissionGain > 0.0F && event.exhaustTransmissionGain <= 8.0F,
        "event route gain or attenuation is invalid");
    require(event.exhaustComponentCount >= 2,
        "split exhaust routes and modes must survive into the realtime event payload");
    double retainedComponentEnergy = 0.0;
    for (std::size_t index = 0; index < event.exhaustComponentCount; ++index) {
        retainedComponentEnergy += event.exhaustComponentGain[index]
            * event.exhaustComponentGain[index];
        require(event.exhaustComponentDelaySeconds[index] > 0.0F
                && event.exhaustComponentResonanceHz[index] > 0.0F,
            "every retained exhaust component must have causal timing and a physical mode");
    }
    require(std::abs(retainedComponentEnergy
                - event.exhaustTransmissionGain * event.exhaustTransmissionGain) < 1.0e-5,
        "bounded route/mode retention must preserve the compiled acoustic energy");
}

void testAcousticGainEnergyAccounting() {
    auto config = makeCustomExhaust();
    auto& path = config.exhaustPaths.front();
    path.audioVolume = 0.5;
    config.cylinders.front().soundAttenuation = 0.8;
    normaliseEngineConfig(config);
    const auto graph = ExhaustGraph::makeForEngine(config);
    const auto metrics = graph.acousticsForCylinder(config.cylinders.front().id);
    require(metrics.routeCount == 2, "split acoustic fixture must expose two outlets");
    const auto restrictionTransmission = 1.0
        / std::sqrt(1.0 + metrics.equivalentRestriction);
    // A splitter distributes energy by downstream acoustic admittance. Its
    // two unity-gain branches still preserve the incoming total energy:
    // 0.5 path * 0.8 cylinder = 0.4.
    require(std::abs(metrics.transmissionGain - 0.4 * restrictionTransmission) < 1.0e-9,
        "splitter, path and cylinder gains were not energy-combined exactly once");

    FiringEvent event;
    event.cylinderId = config.cylinders.front().id;
    event.intensity = 0.73F;
    graph.process(event);
    require(std::abs(event.intensity - 0.73F) < 1.0e-7F
            && std::abs(event.exhaustTransmissionGain
                - static_cast<float>(metrics.transmissionGain)) < 1.0e-6F,
        "graph processing must isolate combustion intensity from exhaust transmission");
}

void testModalRoutesAndAdmittanceWeightedBranches() {
    const auto config = makeCustomExhaust();
    const auto graph = ExhaustGraph::makeForEngine(config);
    const auto cylinderId = config.cylinders.front().id;
    const auto metrics = graph.acousticsForCylinder(cylinderId);
    require(metrics.routeCount == 2 && metrics.modeCount >= 2,
        "split topology must retain route and combined acoustic modes");

    std::vector<const ExhaustRoute*> cylinderRoutes;
    for (const auto& route : graph.routes())
        if (route.cylinderId == cylinderId) cylinderRoutes.push_back(&route);
    require(cylinderRoutes.size() == 2, "modal test did not find both cylinder routes");

    double routeEnergy = 0.0;
    const ExhaustRoute* dominantRoute = nullptr;
    for (const auto* route : cylinderRoutes) {
        routeEnergy += route->audioGain * route->audioGain;
        require(route->modeCount >= 3 && route->modeCount <= maximumExhaustAcousticModes,
            "a complete route must retain bounded quarter-wave modes");
        const auto modeEnergy = std::accumulate(route->modes.begin(),
            route->modes.begin() + route->modeCount, 0.0,
            [](double sum, const ExhaustAcousticMode& mode) {
                return sum + mode.relativeEnergy;
            });
        require(std::abs(modeEnergy - 1.0) < 1.0e-12,
            "route modal energy must be normalised");

        const auto outlet = std::find_if(graph.nodes().begin(), graph.nodes().end(),
            [route](const ExhaustNode& node) { return node.id == route->outletNodeId; });
        require(outlet != graph.nodes().end(), "compiled route references a missing outlet");
        const auto fundamentalHz = graph.referenceWaveSpeedMps() * 1'000.0
            / (4.0 * (route->lengthMm + 0.3 * outlet->diameterMm));
        require(std::any_of(route->modes.begin(), route->modes.begin() + route->modeCount,
            [fundamentalHz](const ExhaustAcousticMode& mode) {
                return std::abs(mode.frequencyHz - fundamentalHz)
                    <= fundamentalHz * 1.0e-10;
            }), "route-length fundamental was discarded by component aggregation");

        if (dominantRoute == nullptr
            || route->audioGain > dominantRoute->audioGain
            || (route->audioGain == dominantRoute->audioGain
                && route->delaySeconds < dominantRoute->delaySeconds)) {
            dominantRoute = route;
        }
    }
    require(std::abs(routeEnergy - 1.0) < 1.0e-12,
        "admittance-weighted splitter must conserve unity-gain route energy");
    require(std::abs(cylinderRoutes[0]->audioGain - cylinderRoutes[1]->audioGain) > 0.01,
        "unequal downstream diameters/restrictions must not receive equal branch energy");
    require(dominantRoute != nullptr
            && std::abs(metrics.delaySeconds - dominantRoute->delaySeconds) < 1.0e-15,
        "scalar delay must follow the dominant route instead of averaging arrivals");
    require(metrics.firstArrivalDelaySeconds < metrics.lastArrivalDelaySeconds
            && metrics.meanDelaySeconds >= metrics.firstArrivalDelaySeconds
            && metrics.meanDelaySeconds <= metrics.lastArrivalDelaySeconds
            && metrics.rmsDelaySpreadSeconds > 0.0,
        "multipath timing statistics were not retained");
    require(std::abs(metrics.resonanceHz - metrics.modes.front().frequencyHz) < 1.0e-12,
        "scalar resonance must expose the strongest retained mode");
}

void testTemperatureAwareWaveSpeed() {
    const auto config = makeCustomExhaust();
    const auto cold = ExhaustGraph::makeForEngine(config, 350.0);
    const auto hot = ExhaustGraph::makeForEngine(config, 950.0);
    require(hot.referenceWaveSpeedMps() > cold.referenceWaveSpeedMps(),
        "hotter exhaust gas must propagate pressure waves faster");

    const auto cylinderId = config.cylinders.front().id;
    const auto coldRoute = std::find_if(cold.routes().begin(), cold.routes().end(),
        [cylinderId](const ExhaustRoute& route) { return route.cylinderId == cylinderId; });
    require(coldRoute != cold.routes().end(), "cold graph route missing");
    const auto hotRoute = std::find_if(hot.routes().begin(), hot.routes().end(),
        [coldRoute](const ExhaustRoute& route) {
            return route.cylinderId == coldRoute->cylinderId
                && route.outletNodeId == coldRoute->outletNodeId;
        });
    require(hotRoute != hot.routes().end(), "hot graph route missing");
    const auto speedRatio = hot.referenceWaveSpeedMps() / cold.referenceWaveSpeedMps();
    require(std::abs(coldRoute->delaySeconds / hotRoute->delaySeconds - speedRatio) < 1.0e-12,
        "route delay does not scale inversely with temperature-aware wave speed");
    require(std::abs(hotRoute->resonanceHz / coldRoute->resonanceHz - speedRatio) < 1.0e-12,
        "derived route resonance does not scale with temperature-aware wave speed");
}

void testCompiledGainBounds() {
    auto config = makeCustomExhaust();
    config.exhaustPaths.front().audioVolume = 8.0;
    config.cylinders.front().soundAttenuation = 4.0;
    for (auto& component : config.exhaustPaths.front().network->components)
        component.acousticGain = 8.0;
    require(!validateEngineConfig(config).has_value(), "high but valid acoustic gain fixture rejected");

    const auto graph = ExhaustGraph::makeForEngine(config);
    const auto cylinderId = config.cylinders.front().id;
    require(std::all_of(graph.routes().begin(), graph.routes().end(),
        [cylinderId](const ExhaustRoute& route) {
            return route.cylinderId != cylinderId
                || (std::isfinite(route.audioGain) && route.audioGain >= 0.0
                    && route.audioGain <= 8.0);
        }), "compiled route gain must remain finite and bounded");
    const auto metrics = graph.acousticsForCylinder(cylinderId);
    require(std::isfinite(metrics.transmissionGain)
            && metrics.transmissionGain >= 0.0 && metrics.transmissionGain <= 8.0,
        "combined topology gain must remain finite and bounded");
}

void testInvalidGraphs() {
    auto duplicateMapping = makeCustomExhaust();
    duplicateMapping.exhaustPaths.front().network->cylinderConnections.push_back(
        duplicateMapping.exhaustPaths.front().network->cylinderConnections.front());
    require(validateEngineConfig(duplicateMapping).has_value(),
        "duplicate cylinder connection must be rejected");
    const auto duplicateSafeGraph = ExhaustGraph::makeForEngine(duplicateMapping);
    require(duplicateSafeGraph.acousticsForCylinder(
                duplicateMapping.cylinders.front().id).routeCount == 2,
        "runtime compiler must not duplicate acoustic energy from an unvalidated mapping");

    auto unknownReference = makeCustomExhaust();
    unknownReference.exhaustPaths.front().network->connections.front().toComponentId = 999'999;
    require(validateEngineConfig(unknownReference).has_value(),
        "connection to an unknown component must be rejected");

    auto cycle = makeCustomExhaust();
    const auto cycleEdge = std::find_if(cycle.exhaustPaths.front().network->connections.begin(),
        cycle.exhaustPaths.front().network->connections.end(), [](const ExhaustComponentConnectionConfig& edge) {
            return edge.fromComponentId == 400 && edge.toComponentId == 500;
        });
    require(cycleEdge != cycle.exhaustPaths.front().network->connections.end(), "cycle test edge missing");
    cycleEdge->toComponentId = 300;
    require(validateEngineConfig(cycle).has_value(), "cyclic exhaust graph must be rejected");
    const auto unsafeInputGraph = ExhaustGraph::makeForEngine(cycle);
    EngineState state;
    state.rpm = 4'000.0;
    state.exhaustFlowGramsPerSecond = 100.0;
    require(!unsafeInputGraph.nodes().empty() && std::isfinite(unsafeInputGraph.backPressureKpa(state)),
        "runtime graph compiler must safely contain an unvalidated cycle");

    auto deadEnd = makeCustomExhaust();
    deadEnd.exhaustPaths.front().network->connections.erase(
        deadEnd.exhaustPaths.front().network->connections.end() - 1);
    require(validateEngineConfig(deadEnd).has_value(), "route that cannot reach an outlet must be rejected");

    auto nonFiniteGain = makeCustomExhaust();
    nonFiniteGain.exhaustPaths.front().network->components.front().acousticGain =
        std::numeric_limits<double>::quiet_NaN();
    require(validateEngineConfig(nonFiniteGain).has_value(),
        "non-finite acoustic component gain must be rejected");
}

template <typename Serializer>
void testRoundTrip(const char* formatName) {
    const auto original = makeCustomExhaust();
    const Serializer serializer;
    const auto document = serializer.encode(original);
    const auto decoded = serializer.decode(document);
    require(static_cast<bool>(decoded), std::string(formatName) + " custom exhaust decode failed: " + decoded.error);
    const auto& path = decoded.config->exhaustPaths.front();
    require(path.network.has_value(), std::string(formatName) + " lost custom exhaust graph");
    require(path.network->components.size() == original.exhaustPaths.front().network->components.size(),
        std::string(formatName) + " lost exhaust components");
    require(path.network->connections.size() == original.exhaustPaths.front().network->connections.size(),
        std::string(formatName) + " lost exhaust connections");
    const auto catalyst = std::find_if(path.network->components.begin(), path.network->components.end(),
        [](const ExhaustComponentConfig& value) { return value.id == 401; });
    require(catalyst != path.network->components.end()
            && catalyst->type == ExhaustComponentType::catalyst
            && std::abs(catalyst->restriction - 0.08) < 1.0e-12,
        std::string(formatName) + " changed typed component data");
}

void testLegacyAndBackPressure() {
    auto legacy = makeDefaultInlineFour();
    legacy.exhaustPaths.front().network.reset();
    const auto legacyGraph = ExhaustGraph::makeForEngine(legacy);
    require(legacyGraph.nodes().size() == legacy.cylinders.size() + 3,
        "legacy geometry was not auto-compiled");
    require(legacyGraph.routes().size() == legacy.cylinders.size(),
        "legacy graph must expose one route per cylinder");
    require(!legacyGraph.pathFlowProperties(0).authoredNetwork
            && !legacyGraph.cylinderFlowProperties(legacy.cylinders.front().id).authoredNetwork,
        "legacy geometry must remain on the simulator's compatibility flow path");

    auto open = makeCustomExhaust();
    auto restricted = open;
    for (auto& item : restricted.exhaustPaths.front().network->components)
        if (item.type == ExhaustComponentType::muffler || item.type == ExhaustComponentType::catalyst)
            item.restriction += 2.0;
    const auto openGraph = ExhaustGraph::makeForEngine(open);
    const auto restrictedGraph = ExhaustGraph::makeForEngine(restricted);
    require(restrictedGraph.effectiveRestriction() > openGraph.effectiveRestriction(),
        "route restriction did not affect the compiled graph");
    require(restrictedGraph.pathFlowProperties(0).effectiveOutletAreaM2
            < openGraph.pathFlowProperties(0).effectiveOutletAreaM2,
        "route restriction did not reduce physical outlet conductance");

    auto sharedRestriction = makeCustomExhaust();
    auto& sharedNetwork = *sharedRestriction.exhaustPaths.front().network;
    sharedNetwork.components.push_back(component(250, ExhaustComponentType::pipe, 300.0, 55.0, 2.0));
    const auto mergeToSplitter = std::find_if(sharedNetwork.connections.begin(), sharedNetwork.connections.end(),
        [](const ExhaustComponentConnectionConfig& edge) {
            return edge.fromComponentId == 200 && edge.toComponentId == 300;
        });
    require(mergeToSplitter != sharedNetwork.connections.end(), "shared restriction test edge missing");
    mergeToSplitter->toComponentId = 250;
    sharedNetwork.connections.push_back({ 250, 300 });
    require(!validateEngineConfig(sharedRestriction).has_value(), "shared series pipe graph rejected");

    auto branchRestriction = sharedRestriction;
    for (auto& item : branchRestriction.exhaustPaths.front().network->components) {
        if (item.id == 250) item.restriction = 0.0;
        if (item.id == 400 || item.id == 401) item.restriction += 2.0;
    }
    const auto sharedGraph = ExhaustGraph::makeForEngine(sharedRestriction);
    const auto branchGraph = ExhaustGraph::makeForEngine(branchRestriction);
    require(sharedGraph.effectiveRestriction() >= 2.0,
        "a common pipe's restriction was incorrectly parallelised with its branches");
    require(sharedGraph.effectiveRestriction() > branchGraph.effectiveRestriction() + 0.5,
        "series-before-split and per-branch restriction must not be equivalent");
}

void testMixedLegacyAndCustomCylinderLookup() {
    auto config = makeCustomExhaust();
    auto customPath = config.exhaustPaths.front();
    const auto firstCustomCylinder = config.cylinders[2].id;
    const auto secondCustomCylinder = config.cylinders[3].id;
    customPath.id = 2;
    customPath.cylinderIds = { firstCustomCylinder, secondCustomCylinder };
    auto& network = *customPath.network;
    std::erase_if(network.cylinderConnections, [firstCustomCylinder, secondCustomCylinder](const auto& item) {
        return item.cylinderId != firstCustomCylinder && item.cylinderId != secondCustomCylinder;
    });
    std::erase_if(network.components, [](const auto& item) {
        return item.id == 101 || item.id == 102;
    });
    std::erase_if(network.connections, [](const auto& item) {
        return item.fromComponentId == 101 || item.fromComponentId == 102;
    });

    ExhaustPathConfig legacyPath;
    legacyPath.id = 1;
    legacyPath.cylinderIds = { config.cylinders[0].id, config.cylinders[1].id };
    legacyPath.geometry = config.exhaust;
    config.exhaustPaths = { std::move(legacyPath), std::move(customPath) };
    normaliseEngineConfig(config);
    require(!validateEngineConfig(config).has_value(), "mixed legacy/custom fixture rejected");

    const auto graph = ExhaustGraph::makeForEngine(config);
    require(!graph.cylinderFlowProperties(config.cylinders[0].id).authoredNetwork,
        "legacy cylinder incorrectly received another path's custom inlet");
    const auto custom = graph.cylinderFlowProperties(firstCustomCylinder);
    require(custom.authoredNetwork && custom.cylinderId == firstCustomCylinder
            && custom.pathIndex == 1,
        "custom cylinder lookup must be keyed by cylinder ID in mixed topologies");
}

void testLegacyGeometryInheritance() {
    auto preset = makeDefaultInlineFour();
    require(preset.exhaustPaths.front().inheritsGlobalGeometry,
        "preset exhaust path must retain generated legacy provenance");
    preset.exhaust.primaryLengthMm = 777.0;
    preset.exhaust.collectorDiameterMm = 71.0;
    normaliseEngineConfig(preset);
    require(std::abs(preset.exhaustPaths.front().geometry.primaryLengthMm - 777.0) < 1.0e-12
            && std::abs(preset.exhaustPaths.front().geometry.collectorDiameterMm - 71.0) < 1.0e-12,
        "editing preset global exhaust geometry did not update its generated path");

    auto explicitPath = preset;
    explicitPath.exhaustPaths.front().inheritsGlobalGeometry = false;
    explicitPath.exhaustPaths.front().geometry.primaryLengthMm = 612.0;
    explicitPath.exhaust.primaryLengthMm = 999.0;
    normaliseEngineConfig(explicitPath);
    require(std::abs(explicitPath.exhaust.primaryLengthMm - 612.0) < 1.0e-12
            && std::abs(explicitPath.exhaustPaths.front().geometry.primaryLengthMm - 612.0) < 1.0e-12,
        "an explicit single exhaust path must be authoritative over the legacy global geometry");

    const JsonEngineSerializer serializer;
    const auto decoded = serializer.decode(serializer.encode(preset));
    require(static_cast<bool>(decoded) && !decoded.config->exhaustPaths.front().inheritsGlobalGeometry,
        "runtime-only exhaust geometry provenance must not leak into schema v1");
}
[[nodiscard]] bool reports(const ExhaustGraph& graph, ExhaustCompileIssue issue) {
    const auto& diagnostics = graph.diagnostics();
    return std::any_of(diagnostics.begin(), diagnostics.end(),
        [issue](const ExhaustCompileDiagnostic& item) { return item.issue == issue; });
}

// A non-finite authored field must never make the exhaust louder or quieter than
// the same graph with the field absent: both used to pick the worst extreme.
void testNonFiniteAuthoredFieldsFailSafe() {
    const auto transmissionFor = [](auto&& mutate) {
        auto config = makeCustomExhaust();
        mutate(config.exhaustPaths.front().network->components);
        const auto graph = ExhaustGraph::makeForEngine(config);
        return graph.acousticsForCylinder(config.cylinders.front().id).transmissionGain;
    };
    const auto reference = transmissionFor([](auto&) {});
    require(reference > 0.0, "reference custom exhaust must transmit sound");

    const auto nanRestriction = transmissionFor([](auto& components) {
        components.front().restriction = std::numeric_limits<double>::quiet_NaN();
    });
    require(std::abs(nanRestriction - reference) < 1.0e-9,
        "a non-finite restriction must fall back to no restriction, not to the maximum");

    const auto nanGain = transmissionFor([](auto& components) {
        for (auto& item : components) item.acousticGain = std::numeric_limits<double>::quiet_NaN();
    });
    require(nanGain == 0.0, "a non-finite acoustic gain must be silent, not maximum gain");
}

// Replacing an authored topology with a generated fallback is a real loss of the
// user's design, so it has to be reported rather than silently substituted.
void testTopologyRejectionIsReported() {
    auto valid = makeCustomExhaust();
    const auto validGraph = ExhaustGraph::makeForEngine(valid);
    require(validGraph.diagnostics().empty(),
        "a well-formed topology must compile without diagnostics");

    auto duplicated = makeDefaultInlineFour();
    // The same cylinder claimed by two paths: coverage is no longer exactly one.
    duplicated.exhaustPaths.front().cylinderIds.push_back(
        duplicated.exhaustPaths.front().cylinderIds.front());
    const auto rejected = ExhaustGraph::makeForEngine(duplicated);
    require(reports(rejected, ExhaustCompileIssue::topologyRejected),
        "a cylinder covered twice must report topologyRejected");
    require(rejected.routes().size() > 0,
        "a rejected topology must still compile a usable fallback graph");
}
} // namespace

int main() {
    try {
        testNonFiniteAuthoredFieldsFailSafe();
        testTopologyRejectionIsReported();
        testValidationAndRouting();
        testAcousticGainEnergyAccounting();
        testModalRoutesAndAdmittanceWeightedBranches();
        testTemperatureAwareWaveSpeed();
        testCompiledGainBounds();
        testInvalidGraphs();
        testRoundTrip<enginelab::JsonEngineSerializer>("JSON");
        testRoundTrip<enginelab::YamlEngineSerializer>("YAML");
        testLegacyAndBackPressure();
        testMixedLegacyAndCustomCylinderLookup();
        testLegacyGeometryInheritance();
        std::cout << "EngineLab exhaust graph tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "EngineLab exhaust graph tests failed: " << error.what() << '\n';
        return 1;
    }
}
