#include <enginelab/exhaust/ExhaustPathTopologyEditor.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unordered_set>

namespace {
using namespace enginelab;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

[[nodiscard]] ExhaustComponentConfig component(std::uint32_t id, ExhaustComponentType type,
                                                double lengthMm, double diameterMm) {
    ExhaustComponentConfig value;
    value.id = id;
    value.type = type;
    value.lengthMm = lengthMm;
    value.diameterMm = diameterMm;
    return value;
}

void installGeneratedNetwork(ExhaustPathConfig& path) {
    ExhaustNetworkConfig network;
    std::uint32_t nextId = 1;
    std::vector<std::uint32_t> primaries;
    for (const auto cylinderId : path.cylinderIds) {
        const auto id = nextId++;
        network.components.push_back(component(id, ExhaustComponentType::pipe,
            path.geometry.primaryLengthMm, path.geometry.primaryDiameterMm));
        network.cylinderConnections.push_back({ cylinderId, id });
        primaries.push_back(id);
    }

    std::uint32_t previous = primaries.front();
    if (primaries.size() > 1U) {
        const auto mergeId = nextId++;
        network.components.push_back(component(
            mergeId, ExhaustComponentType::merge, 0.0, path.geometry.collectorDiameterMm));
        for (const auto primaryId : primaries)
            network.connections.push_back({ primaryId, mergeId });
        previous = mergeId;
    }
    const auto mufflerId = nextId++;
    auto muffler = component(mufflerId, ExhaustComponentType::muffler,
        480.0, path.geometry.collectorDiameterMm);
    muffler.restriction = path.geometry.mufflerRestriction;
    network.components.push_back(muffler);
    network.connections.push_back({ previous, mufflerId });
    const auto outletId = nextId;
    network.components.push_back(component(
        outletId, ExhaustComponentType::outlet, 0.0, path.geometry.outletDiameterMm));
    network.connections.push_back({ mufflerId, outletId });
    path.network = std::move(network);
}

void requireUniqueCompleteOwnership(const EngineConfig& config) {
    std::unordered_set<std::uint32_t> assigned;
    for (const auto& path : config.exhaustPaths) {
        require(!path.cylinderIds.empty(), "path ownership edit created an empty path");
        for (const auto cylinderId : path.cylinderIds)
            require(assigned.insert(cylinderId).second, "a cylinder belongs to multiple paths");
    }
    require(assigned.size() == config.cylinders.size(), "a cylinder is missing from exhaust paths");
    for (const auto& cylinder : config.cylinders)
        require(assigned.contains(cylinder.id), "unknown/missing cylinder ownership");
    require(!validateEngineConfig(config).has_value(), "edited engine configuration must remain valid");
}

[[nodiscard]] const ExhaustPathConfig& pathById(const EngineConfig& config, std::uint32_t id) {
    const auto found = std::find_if(config.exhaustPaths.begin(), config.exhaustPaths.end(),
        [id](const ExhaustPathConfig& path) { return path.id == id; });
    if (found == config.exhaustPaths.end()) throw std::runtime_error("expected exhaust path missing");
    return *found;
}

void basicPathLifecycle() {
    auto config = makeDefaultInlineFour();
    requireUniqueCompleteOwnership(config);
    const auto originalPathId = config.exhaustPaths.front().id;
    const auto splitCylinder = config.cylinders.back().id;
    const auto created = ExhaustPathTopologyEditor::createPathFromCylinder(
        config, originalPathId, splitCylinder);
    require(static_cast<bool>(created), "creating a second path failed: " + created.error);
    require(config.exhaustPaths.size() == 2U && created.selectedPathId != originalPathId,
        "new path ID or count is invalid");
    requireUniqueCompleteOwnership(config);
    require(pathById(config, created.selectedPathId).cylinderIds == std::vector<std::uint32_t> { splitCylinder },
        "new path did not receive the selected cylinder");

    const auto movedCylinder = config.cylinders[2].id;
    const auto moved = ExhaustPathTopologyEditor::moveCylinder(
        config, movedCylinder, originalPathId, created.selectedPathId);
    require(static_cast<bool>(moved), "moving a cylinder failed: " + moved.error);
    require(pathById(config, created.selectedPathId).cylinderIds.size() == 2U,
        "destination path did not receive the moved cylinder");
    requireUniqueCompleteOwnership(config);

    const auto beforeRejectedMove = config;
    const auto onlyCylinder = pathById(config, originalPathId).cylinderIds.front();
    const auto secondMove = ExhaustPathTopologyEditor::moveCylinder(
        config, onlyCylinder, originalPathId, created.selectedPathId);
    require(static_cast<bool>(secondMove), "moving the penultimate source cylinder should succeed");
    const auto rejected = ExhaustPathTopologyEditor::moveCylinder(
        config, pathById(config, originalPathId).cylinderIds.front(),
        originalPathId, created.selectedPathId);
    require(!static_cast<bool>(rejected), "moving the final path cylinder must be rejected");
    require(pathById(config, originalPathId).cylinderIds.size() == 1U,
        "rejected move was not transactional");
    (void)beforeRejectedMove;

    for (auto& bank : config.banks) bank.exhaustPathId = created.selectedPathId;
    const auto removed = ExhaustPathTopologyEditor::removePath(
        config, created.selectedPathId, originalPathId);
    require(static_cast<bool>(removed), "deleting/merging a path failed: " + removed.error);
    require(config.exhaustPaths.size() == 1U, "deleted path remains in the configuration");
    requireUniqueCompleteOwnership(config);
    require(std::all_of(config.banks.begin(), config.banks.end(), [originalPathId](const auto& bank) {
        return bank.exhaustPathId == originalPathId;
    }), "bank path hints were not repaired after deletion");
}

void customDagOwnershipEdits() {
    auto config = makeDefaultInlineFour();
    const auto originalPathId = config.exhaustPaths.front().id;
    installGeneratedNetwork(config.exhaustPaths.front());
    requireUniqueCompleteOwnership(config);
    const auto originalComponentCount = config.exhaustPaths.front().network->components.size();
    const auto splitCylinder = config.cylinders.back().id;
    const auto splitRouteBefore = ExhaustGraph::makeForEngine(config)
        .acousticsForCylinder(splitCylinder);

    const auto created = ExhaustPathTopologyEditor::createPathFromCylinder(
        config, originalPathId, splitCylinder);
    require(static_cast<bool>(created), "custom DAG split failed: " + created.error);
    requireUniqueCompleteOwnership(config);
    require(pathById(config, originalPathId).network->components.size() + 1U == originalComponentCount,
        "moving a cylinder did not prune its exclusive primary branch");

    auto& newPath = *std::find_if(config.exhaustPaths.begin(), config.exhaustPaths.end(),
        [&created](const ExhaustPathConfig& path) { return path.id == created.selectedPathId; });
    require(newPath.network.has_value(),
        "creating a path discarded the selected cylinder's custom downstream branch");
    require(std::any_of(newPath.network->components.begin(), newPath.network->components.end(),
        [](const ExhaustComponentConfig& item) { return item.type == ExhaustComponentType::muffler; })
        && std::any_of(newPath.network->components.begin(), newPath.network->components.end(),
        [](const ExhaustComponentConfig& item) { return item.type == ExhaustComponentType::outlet; }),
        "transferred custom branch lost its muffler or outlet");
    const auto splitRouteAfter = ExhaustGraph::makeForEngine(config)
        .acousticsForCylinder(splitCylinder);
    require(std::abs(splitRouteAfter.meanLengthMm - splitRouteBefore.meanLengthMm) < 1.0e-9,
        "creating a path changed the transferred cylinder's tuned route length");

    // Replace the transferred branch with a known single-cylinder chain for
    // the independent destination-expansion scenario below.
    installGeneratedNetwork(newPath); // valid single-cylinder chain, deliberately without a merge
    requireUniqueCompleteOwnership(config);
    const auto existingDestinationCylinder = config.cylinders.back().id;
    const auto routeBeforeMove = ExhaustGraph::makeForEngine(config)
        .acousticsForCylinder(existingDestinationCylinder);

    const auto moveIntoSingleChain = ExhaustPathTopologyEditor::moveCylinder(
        config, config.cylinders[2].id, originalPathId, created.selectedPathId);
    require(static_cast<bool>(moveIntoSingleChain),
        "moving into a single-cylinder custom chain failed: " + moveIntoSingleChain.error);
    requireUniqueCompleteOwnership(config);
    const auto& expanded = pathById(config, created.selectedPathId);
    require(std::count_if(expanded.network->components.begin(), expanded.network->components.end(),
        [](const ExhaustComponentConfig& item) { return item.type == ExhaustComponentType::merge; }) == 1,
        "moving into a custom chain did not insert one shared merge");
    const auto expandedGraph = ExhaustGraph::makeForEngine(config);
    const auto oldCylinderRoute = expandedGraph.acousticsForCylinder(existingDestinationCylinder);
    const auto newCylinderRoute = expandedGraph.acousticsForCylinder(config.cylinders[2].id);
    require(std::abs(oldCylinderRoute.meanLengthMm - routeBeforeMove.meanLengthMm) < 1.0e-9,
        "inserting a cylinder changed the existing cylinder's tuned primary length");
    require(std::abs(newCylinderRoute.meanLengthMm - routeBeforeMove.meanLengthMm) < 1.0e-9,
        "new cylinder did not receive an equivalent independent primary route");
    require(std::abs(oldCylinderRoute.equivalentRestriction
            - newCylinderRoute.equivalentRestriction) < 1.0e-9,
        "two equivalent primary branches received different compiled restrictions");

    const auto mergedBack = ExhaustPathTopologyEditor::removePath(
        config, created.selectedPathId, originalPathId);
    require(static_cast<bool>(mergedBack), "merging custom paths failed: " + mergedBack.error);
    requireUniqueCompleteOwnership(config);
    require(config.exhaustPaths.size() == 1U && config.exhaustPaths.front().network.has_value(),
        "custom destination graph was discarded during path deletion");
}

// A V8's dual 4-into-1 as the designer builds it: per bank four primaries into
// a collector (200), a mid pipe (500), a silencer (600) and a tail (700).
constexpr double dualPrimaryLengthMm = 760.0;
constexpr double dualPrimaryDiameterMm = 38.0;
constexpr double dualMidLengthMm = 1'500.0;

[[nodiscard]] EngineConfig dualFourIntoOneV8() {
    auto config = makeDefaultV8();
    require(config.banks.size() == 2U, "the default V8 has two banks");
    const auto geometry = config.exhaustPaths.front().geometry;
    config.exhaustPaths.clear();
    for (std::size_t side = 0; side < 2U; ++side) {
        ExhaustPathConfig path;
        path.id = static_cast<std::uint32_t>(side + 1U);
        path.geometry = geometry;
        path.inheritsGlobalGeometry = false;
        path.cylinderIds = config.banks[side].cylinderIds;
        ExhaustNetworkConfig network;
        auto collector = component(200, ExhaustComponentType::merge, 0.0, 54.0);
        collector.volumeLitres = 0.5;
        network.components.push_back(collector);
        for (const auto cylinderId : path.cylinderIds) {
            network.components.push_back(component(1'000 + cylinderId, ExhaustComponentType::pipe,
                dualPrimaryLengthMm, dualPrimaryDiameterMm));
            network.cylinderConnections.push_back({ cylinderId, 1'000 + cylinderId });
            network.connections.push_back({ 1'000 + cylinderId, 200 });
        }
        network.components.push_back(component(500, ExhaustComponentType::pipe, dualMidLengthMm, 54.0));
        auto silencer = component(600, ExhaustComponentType::muffler, 480.0, 62.0);
        silencer.volumeLitres = 8.0;
        silencer.restriction = 0.28;
        network.components.push_back(silencer);
        network.components.push_back(component(700, ExhaustComponentType::outlet, 300.0, 54.0));
        network.connections.push_back({ 200, 500 });
        network.connections.push_back({ 500, 600 });
        network.connections.push_back({ 600, 700 });
        path.network = std::move(network);
        config.exhaustPaths.push_back(std::move(path));
    }
    for (auto& bank : config.banks) bank.exhaustPathId = 0;
    requireUniqueCompleteOwnership(config);
    return config;
}

[[nodiscard]] std::size_t countType(const ExhaustNetworkConfig& network, ExhaustComponentType type) {
    return static_cast<std::size_t>(std::count_if(network.components.begin(), network.components.end(),
        [type](const ExhaustComponentConfig& item) { return item.type == type; }));
}

[[nodiscard]] std::size_t inputsOf(const ExhaustNetworkConfig& network, std::uint32_t id) {
    return static_cast<std::size_t>(std::count_if(network.connections.begin(), network.connections.end(),
        [id](const ExhaustComponentConnectionConfig& edge) { return edge.toComponentId == id; }));
}

/** Every cylinder of the path has its own primary like the designer's, flowing
 *  straight into a merge. */
void requireOwnPrimaries(const ExhaustPathConfig& path, const std::string& label) {
    const auto& network = *path.network;
    std::unordered_set<std::uint32_t> roots;
    for (const auto& mapping : network.cylinderConnections) {
        require(roots.insert(mapping.componentId).second, label + ": two cylinders share a primary");
        const auto root = std::find_if(network.components.begin(), network.components.end(),
            [&mapping](const ExhaustComponentConfig& item) { return item.id == mapping.componentId; });
        require(root != network.components.end() && root->type == ExhaustComponentType::pipe
                && root->lengthMm == dualPrimaryLengthMm && root->diameterMm == dualPrimaryDiameterMm,
            label + ": a cylinder lost its primary's dimensions");
        const auto out = std::find_if(network.connections.begin(), network.connections.end(),
            [&mapping](const ExhaustComponentConnectionConfig& edge) { return edge.fromComponentId == mapping.componentId; });
        require(out != network.connections.end(), label + ": a primary leads nowhere");
        const auto next = std::find_if(network.components.begin(), network.components.end(),
            [&out](const ExhaustComponentConfig& item) { return item.id == out->toComponentId; });
        require(next != network.components.end() && next->type == ExhaustComponentType::merge,
            label + ": a primary does not flow into a collector");
    }
}

// Owner report 2026-11-11: removing one path of a V8's dual 4-into-1 into the
// other made a chain of merges on one primary, the cylinders "wired anyhow".
void removingAPathRebuildsOneCollector() {
    auto config = dualFourIntoOneV8();
    const auto removed = ExhaustPathTopologyEditor::removePath(config, 2, 1);
    require(static_cast<bool>(removed), "rebuilding the dual system failed: " + removed.error);
    requireUniqueCompleteOwnership(config);
    require(config.exhaustPaths.size() == 1U && removed.note.empty(), "the rebuild fell back or kept the path");
    const auto& path = config.exhaustPaths.front();
    require(path.cylinderIds.size() == 8U, "the rebuilt path does not own the eight cylinders");
    require(countType(*path.network, ExhaustComponentType::merge) == 1U,
        "the rebuild left " + std::to_string(countType(*path.network, ExhaustComponentType::merge))
            + " merges, not one 8-into-1 collector");
    require(inputsOf(*path.network, 200) == 8U, "the destination collector does not take all eight primaries");
    requireOwnPrimaries(path, "rebuild");
    require(countType(*path.network, ExhaustComponentType::muffler) == 1U
            && countType(*path.network, ExhaustComponentType::outlet) == 1U,
        "the rebuild did not keep the destination's own silencer and tail");
}

void removingAPathWithAYKeepsBothCollectors() {
    auto config = dualFourIntoOneV8();
    const auto routeBefore = ExhaustGraph::makeForEngine(config).acousticsForCylinder(config.banks[1].cylinderIds.front());
    const auto removed = ExhaustPathTopologyEditor::removePath(config, 2, 1, ExhaustPathJoin::yJunction);
    require(static_cast<bool>(removed), "joining the dual system with a Y failed: " + removed.error);
    requireUniqueCompleteOwnership(config);
    require(config.exhaustPaths.size() == 1U && removed.note.empty(), "the Y join fell back or kept the path");
    const auto& path = config.exhaustPaths.front();
    const auto& network = *path.network;
    require(countType(network, ExhaustComponentType::merge) == 3U,
        "a Y join is two collectors and one Y, not "
            + std::to_string(countType(network, ExhaustComponentType::merge)) + " merges");
    require(countType(network, ExhaustComponentType::muffler) == 1U
            && countType(network, ExhaustComponentType::outlet) == 1U,
        "the removed path's pipes past its collector were kept");
    requireOwnPrimaries(path, "Y");
    const auto mid = std::find_if(network.components.begin(), network.components.end(),
        [](const ExhaustComponentConfig& item) { return item.id == 500; });
    require(mid != network.components.end() && std::abs(mid->lengthMm - (dualMidLengthMm - 300.0)) < 1.0e-9,
        "the mid pipe did not give the Y's legs their length");
    const auto graph = ExhaustGraph::makeForEngine(config);
    for (const auto bank : { 0U, 1U }) {
        const auto route = graph.acousticsForCylinder(config.banks[bank].cylinderIds.front());
        require(std::abs(route.meanLengthMm - routeBefore.meanLengthMm) < 1.0e-9,
            "the Y changed a cylinder's length to the silencer");
    }
}

void legacyPathsJoinBothWays() {
    auto config = dualFourIntoOneV8();
    for (auto& path : config.exhaustPaths) path.network.reset();
    auto yJoined = config;
    const auto y = ExhaustPathTopologyEditor::removePath(yJoined, 2, 1, ExhaustPathJoin::yJunction);
    require(static_cast<bool>(y), "a Y join of two scalar paths failed: " + y.error);
    requireUniqueCompleteOwnership(yJoined);
    require(yJoined.exhaustPaths.front().network
            && countType(*yJoined.exhaustPaths.front().network, ExhaustComponentType::merge) == 3U,
        "two scalar 4-into-1 paths did not become two collectors and a Y");
    const auto rebuilt = ExhaustPathTopologyEditor::removePath(config, 2, 1);
    require(static_cast<bool>(rebuilt) && !config.exhaustPaths.front().network
            && config.exhaustPaths.front().cylinderIds.size() == 8U,
        "rebuilding two scalar paths is not one scalar 8-into-1");
}

void cylindersThatNeverMeetFallBack() {
    auto config = dualFourIntoOneV8();
    // Path 1 becomes four separate pipes, each to its own tail: no collector.
    auto& network = *config.exhaustPaths.front().network;
    network.components.clear();
    network.connections.clear();
    for (const auto& mapping : network.cylinderConnections) {
        network.components.push_back(component(mapping.componentId, ExhaustComponentType::pipe,
            dualPrimaryLengthMm, dualPrimaryDiameterMm));
        network.components.push_back(component(mapping.componentId + 2'000, ExhaustComponentType::outlet, 300.0, 40.0));
        network.connections.push_back({ mapping.componentId, mapping.componentId + 2'000 });
    }
    requireUniqueCompleteOwnership(config);
    const auto removed = ExhaustPathTopologyEditor::removePath(config, 2, 1);
    require(static_cast<bool>(removed), "removing into separate pipes failed: " + removed.error);
    requireUniqueCompleteOwnership(config);
    require(!removed.note.empty(), "a fallback join did not say so");
}

void movedCylinderJoinsItsNeighboursCollector() {
    auto config = dualFourIntoOneV8();
    const auto moved = config.banks[1].cylinderIds.front();
    const auto result = ExhaustPathTopologyEditor::moveCylinder(config, moved, 2, 1);
    require(static_cast<bool>(result), "moving a cylinder between banks failed: " + result.error);
    requireUniqueCompleteOwnership(config);
    const auto& path = pathById(config, 1);
    require(countType(*path.network, ExhaustComponentType::merge) == 1U && inputsOf(*path.network, 200) == 5U,
        "a moved cylinder did not make the bank's 4-into-1 a 5-into-1");
    requireOwnPrimaries(path, "move");
}

void cylinderFeedsAnyPart() {
    auto config = dualFourIntoOneV8();
    auto& network = *config.exhaustPaths.front().network;
    const auto cylinderId = config.exhaustPaths.front().cylinderIds.front();
    require(ExhaustNetworkEditor::firstCommonJunction(network) == std::optional<std::uint32_t> { 200 },
        "the dual system's collector is not where its cylinders meet");
    // Owner report 2026-11-11: delete the cylinder's default pipe, put
    // something else in its place, feed it from the cylinder.
    const auto oldPrimary = 1'000 + cylinderId;
    std::erase_if(network.components, [oldPrimary](const auto& item) { return item.id == oldPrimary; });
    std::erase_if(network.connections, [oldPrimary](const auto& edge) { return edge.fromComponentId == oldPrimary; });
    std::erase_if(network.cylinderConnections, [oldPrimary](const auto& mapping) { return mapping.componentId == oldPrimary; });
    const auto id = ExhaustNetworkEditor::nextComponentId(network);
    network.components.push_back(ExhaustNetworkEditor::defaultComponent(ExhaustComponentType::resonator, id));
    require(ExhaustNetworkEditor::connectComponents(network, id, 200).empty(), "a part could not feed the collector");
    require(!ExhaustNetworkEditor::connectComponents(network, id, 200).empty(), "a duplicate connection was accepted");
    require(!ExhaustNetworkEditor::connectComponents(network, id, id).empty(), "a self connection was accepted");
    require(ExhaustNetworkEditor::connectCylinder(network, cylinderId, id).empty(), "a cylinder could not feed a part");
    requireUniqueCompleteOwnership(config);
    require(ExhaustNetworkEditor::disconnectCylinder(network, cylinderId), "the cylinder fed nothing");
    require(validateEngineConfig(config).has_value(), "a cylinder feeding nothing must not validate");
    require(ExhaustNetworkEditor::connectCylinder(network, cylinderId, id).empty(), "reconnecting failed");

    // Right-click "Replace with": the silencer becomes a catalyst, same place.
    require(ExhaustNetworkEditor::replaceComponent(network, 600, ExhaustComponentType::catalyst).empty(),
        "replacing the silencer failed");
    const auto replaced = std::find_if(network.components.begin(), network.components.end(),
        [](const ExhaustComponentConfig& item) { return item.id == 600; });
    require(replaced != network.components.end() && replaced->type == ExhaustComponentType::catalyst
            && replaced->lengthMm == 480.0 && replaced->diameterMm == 62.0 && replaced->volumeLitres == 0.0,
        "a replaced part did not keep its place, length and bore with the new type's values");
    require(inputsOf(network, 600) == 1U && std::any_of(network.connections.begin(), network.connections.end(),
        [](const auto& edge) { return edge.fromComponentId == 600 && edge.toComponentId == 700; }),
        "a replaced part lost its connections");
    requireUniqueCompleteOwnership(config);

    // An X-pipe takes numbered ports; a cylinder cannot feed one.
    require(ExhaustNetworkEditor::replaceComponent(network, 500, ExhaustComponentType::crossover).empty(),
        "replacing the mid pipe by an X failed");
    const auto ports = std::count_if(network.connections.begin(), network.connections.end(),
        [](const auto& edge) { return (edge.toComponentId == 500 && edge.toPort == 0U)
                                   || (edge.fromComponentId == 500 && edge.fromPort == 0U); });
    require(ports == 2 && !ExhaustNetworkEditor::connectCylinder(network, cylinderId, 500).empty(),
        "an X-pipe's ports were not numbered or a cylinder fed it");
}

// Audit EXHAUST-01: a turbocharger chosen by path index (-1) moved with the
// paths after the one removed, so the other two paths swapped turbines.
void removingAPathKeepsEachTurbine() {
    auto config = makeDefaultInlineFour();
    const auto first = config.exhaustPaths.front().id;
    const auto second = ExhaustPathTopologyEditor::createPathFromCylinder(
        config, first, config.cylinders[3].id).selectedPathId;
    const auto third = ExhaustPathTopologyEditor::createPathFromCylinder(
        config, first, config.cylinders[2].id).selectedPathId;
    require(config.exhaustPaths.size() == 3U && second != 0U && third != 0U, "three paths for the twin-turbo fixture");
    config.forcedInduction.enabled = true;
    config.forcedInduction.type = ForcedInductionType::turbocharger;
    config.forcedInduction.turbochargerCount = 2U;
    for (auto& path : config.exhaustPaths) path.turbocharger = -1;
    if (const auto error = validateEngineConfig(config)) require(false, "the twin-turbo fixture is valid: " + *error);
    const auto turbineOf = [](const EngineConfig& engine, std::uint32_t pathId) {
        for (std::size_t index = 0; index < engine.exhaustPaths.size(); ++index) {
            const auto& path = engine.exhaustPaths[index];
            if (path.id != pathId) continue;
            return path.turbocharger >= 0 ? static_cast<std::size_t>(path.turbocharger)
                                          : index % engine.forcedInduction.turbochargerCount;
        }
        return std::size_t { 99 };
    };
    const auto secondTurbine = turbineOf(config, second);
    const auto thirdTurbine = turbineOf(config, third);
    const auto removed = ExhaustPathTopologyEditor::removePath(config, first, third);
    require(static_cast<bool>(removed), "removing the first path failed: " + removed.error);
    require(turbineOf(config, second) == secondTurbine && turbineOf(config, third) == thirdTurbine,
        "each remaining path keeps the turbine it drove");
}

[[nodiscard]] const ExhaustComponentConfig* componentById(const ExhaustNetworkConfig& network, std::uint32_t id) {
    const auto found = std::find_if(network.components.begin(), network.components.end(),
        [id](const ExhaustComponentConfig& item) { return item.id == id; });
    return found != network.components.end() ? &*found : nullptr;
}

/** The pipes past the collectors other than `except`, by length. */
[[nodiscard]] std::vector<double> midPipeLengths(const ExhaustNetworkConfig& network, std::uint32_t except) {
    std::vector<double> lengths;
    for (const auto& part : network.components) {
        if (part.type != ExhaustComponentType::pipe || part.id == except) continue;
        if (std::any_of(network.cylinderConnections.begin(), network.cylinderConnections.end(),
                [&part](const auto& mapping) { return mapping.componentId == part.id; }))
            continue;
        lengths.push_back(part.lengthMm);
    }
    std::sort(lengths.begin(), lengths.end());
    return lengths;
}

/** One click: an H-pipe between a dual exhaust drawn as two paths, graph or
 *  scalar. The two paths become one, each exhaust whole; each mid pipe is
 *  cut a third of the way and a 250 mm balance pipe of its bore joins the
 *  cuts on a splitter's and a merge's side ports, as the designer's H. By
 *  hand it took three parts, two deletions, six drags and edits, and no drag
 *  could make a side branch at all (owner 2026-11-19). */
void oneClickHPipe() {
    for (const auto scalar : { false, true }) {
        auto config = dualFourIntoOneV8();
        if (scalar)
            for (auto& path : config.exhaustPaths) path.network.reset();
        const auto label = std::string(scalar ? "scalar" : "graph") + " dual V8: ";
        const auto result = ExhaustPathTopologyEditor::addCrossover(config, 1, ExhaustCrossoverKind::hPipe);
        require(static_cast<bool>(result), label + "the H was refused: " + result.error);
        requireUniqueCompleteOwnership(config);
        require(config.exhaustPaths.size() == 1U && config.exhaustPaths.front().cylinderIds.size() == 8U
                && result.selectedPathId == 1U && result.joinedPathId == 2U,
            label + "the two paths are not one carrying all eight cylinders");
        const auto& network = *config.exhaustPaths.front().network;
        require(countType(network, ExhaustComponentType::outlet) == 2U
                && countType(network, ExhaustComponentType::splitter) == 1U,
            label + "both exhausts are not kept whole beside the H's splitter");
        const auto* balance = componentById(network, result.componentId);
        require(balance != nullptr && balance->type == ExhaustComponentType::pipe
                && std::abs(balance->lengthMm - 250.0) < 1.0e-9,
            label + "no 250 mm balance pipe");
        std::size_t side = 0;
        for (const auto& edge : network.connections) {
            if (edge.fromPort != exhaustSideBranchPort && edge.toPort != exhaustSideBranchPort) continue;
            ++side;
            const auto* junction = componentById(network,
                edge.fromPort == exhaustSideBranchPort ? edge.fromComponentId : edge.toComponentId);
            require((edge.fromComponentId == balance->id || edge.toComponentId == balance->id) && junction != nullptr
                    && junction->volumeLitres == 0.0 && junction->lengthMm == 0.0
                    && (junction->type == ExhaustComponentType::splitter
                        || junction->type == ExhaustComponentType::merge),
                label + "the balance pipe does not meet a compact junction on its side port");
        }
        require(side == 2U, label + "the balance pipe is not a side branch at each end");
        if (!scalar) {
            // Each side's way from its collector to its silencer is as long as before.
            const auto lengths = midPipeLengths(network, balance->id);
            require(lengths.size() == 4U && std::abs(lengths[0] - 0.35 * dualMidLengthMm) < 1.0e-9
                    && std::abs(lengths[1] - 0.35 * dualMidLengthMm) < 1.0e-9
                    && std::abs(lengths[2] - 0.65 * dualMidLengthMm) < 1.0e-9
                    && std::abs(lengths[3] - 0.65 * dualMidLengthMm) < 1.0e-9
                    && std::abs(balance->diameterMm - 54.0) < 1.0e-9,
                label + "the mid pipes are not cut a third of the way, or the balance pipe lost their bore");
        }

        // No limit: a second one goes in beside it.
        const auto again = ExhaustPathTopologyEditor::addCrossover(config, 1, ExhaustCrossoverKind::hPipe);
        require(static_cast<bool>(again) && again.joinedPathId == 0U
                && countType(*config.exhaustPaths.front().network, ExhaustComponentType::splitter) == 2U,
            label + "a second H-pipe in the same exhaust was refused: " + again.error);
    }
}

/** One click: an X-pipe, the first path's pipe through port 0 each way. */
void oneClickXPipe() {
    auto config = dualFourIntoOneV8();
    const auto result = ExhaustPathTopologyEditor::addCrossover(config, 2, ExhaustCrossoverKind::xPipe);
    require(static_cast<bool>(result), "the X was refused: " + result.error);
    requireUniqueCompleteOwnership(config);
    require(config.exhaustPaths.size() == 1U && result.selectedPathId == 2U, "the X's two paths are not one");
    const auto& network = *config.exhaustPaths.front().network;
    const auto* x = componentById(network, result.componentId);
    require(x != nullptr && x->type == ExhaustComponentType::crossover && x->lengthMm == 0.0
            && std::abs(x->crossoverCoupling - 0.30) < 1.0e-9 && std::abs(x->restriction - 0.06) < 1.0e-9
            && std::abs(x->diameterMm - 54.0) < 1.0e-9,
        "no compact X of the pipes' bore, k 0.30 and K 0.06");
    std::array<int, 4> ports {};
    for (const auto& edge : network.connections) {
        if (edge.toComponentId == x->id && edge.toPort <= 1U) ++ports[edge.toPort];
        if (edge.fromComponentId == x->id && edge.fromPort <= 1U) ++ports[2U + edge.fromPort];
    }
    require(ports == std::array<int, 4> { 1, 1, 1, 1 }, "each of the X's four ports does not carry one pipe");
    require(midPipeLengths(network, 0).size() == 4U, "each pipe is not cut in two around the X");
}

/** What has no two pipes to join says so and changes nothing; two paths on
 *  their own turbines are not joined ahead of them; a one-bank exhaust split
 *  into two tailpipes takes one inside its path. */
void crossoverRefusalsSayWhy() {
    auto single = makeDefaultInlineFour();
    installGeneratedNetwork(single.exhaustPaths.front());
    const auto parts = single.exhaustPaths.front().network->components.size();
    const auto refused = ExhaustPathTopologyEditor::addCrossover(single, single.exhaustPaths.front().id,
                                                                 ExhaustCrossoverKind::hPipe);
    require(!refused && refused.error.find("one pipe") != std::string::npos
            && single.exhaustPaths.front().network->components.size() == parts,
        "an H on a single pipe is refused, saying why, and nothing changes: " + refused.error);

    auto turbo = dualFourIntoOneV8();
    turbo.forcedInduction.enabled = true;
    turbo.forcedInduction.type = ForcedInductionType::turbocharger;
    turbo.forcedInduction.turbochargerCount = 2U;
    if (const auto error = validateEngineConfig(turbo)) require(false, "the twin-turbo V8 fixture is valid: " + *error);
    const auto blocked = ExhaustPathTopologyEditor::addCrossover(turbo, 1, ExhaustCrossoverKind::xPipe);
    require(!blocked && blocked.error.find("turbine") != std::string::npos && turbo.exhaustPaths.size() == 2U,
        "an X joining two turbocharged paths is refused, saying why: " + blocked.error);

    auto split = makeDefaultInlineFour();
    installGeneratedNetwork(split.exhaustPaths.front());
    auto& network = *split.exhaustPaths.front().network;
    const auto idOf = [&network](ExhaustComponentType type) {
        return std::find_if(network.components.begin(), network.components.end(),
            [type](const ExhaustComponentConfig& part) { return part.type == type; })->id;
    };
    const auto merge = idOf(ExhaustComponentType::merge);
    const auto muffler = idOf(ExhaustComponentType::muffler);
    std::erase_if(network.connections, [muffler](const ExhaustComponentConnectionConfig& edge) {
        return edge.toComponentId == muffler;
    });
    network.components.push_back(component(90, ExhaustComponentType::splitter, 0.0, 50.0));
    network.components.push_back(component(91, ExhaustComponentType::pipe, 900.0, 45.0));
    network.components.push_back(component(92, ExhaustComponentType::pipe, 900.0, 45.0));
    network.components.push_back(component(93, ExhaustComponentType::outlet, 0.0, 45.0));
    network.connections.push_back({ merge, 90 });
    network.connections.push_back({ 90, 91 });
    network.connections.push_back({ 90, 92 });
    network.connections.push_back({ 91, muffler });
    network.connections.push_back({ 92, 93 });
    if (const auto error = validateEngineConfig(split)) require(false, "the split fixture is valid: " + *error);
    const auto inside = ExhaustPathTopologyEditor::addCrossover(split, split.exhaustPaths.front().id,
                                                                ExhaustCrossoverKind::hPipe);
    require(static_cast<bool>(inside) && inside.joinedPathId == 0U
            && std::abs(componentById(*split.exhaustPaths.front().network, inside.componentId)->diameterMm - 45.0)
                < 1.0e-9,
        "a split one-bank exhaust takes an H between its two pipes: " + inside.error);
}

void invalidWorkIsNotMutated() {
    auto config = makeDefaultInlineFour();
    installGeneratedNetwork(config.exhaustPaths.front());
    config.exhaustPaths.front().network->components.push_back(
        component(999, ExhaustComponentType::pipe, 300.0, 42.0)); // orphan: deliberately invalid
    const auto pathCount = config.exhaustPaths.size();
    const auto ownership = config.exhaustPaths.front().cylinderIds;
    const auto result = ExhaustPathTopologyEditor::createPathFromCylinder(
        config, config.exhaustPaths.front().id, config.cylinders.back().id);
    require(!static_cast<bool>(result), "path edit must reject an already-invalid working graph");
    require(config.exhaustPaths.size() == pathCount
            && config.exhaustPaths.front().cylinderIds == ownership,
        "failed path edit mutated the caller's configuration");
}
} // namespace

int main() {
    try {
        basicPathLifecycle();
        customDagOwnershipEdits();
        invalidWorkIsNotMutated();
        removingAPathRebuildsOneCollector();
        movedCylinderJoinsItsNeighboursCollector();
        removingAPathWithAYKeepsBothCollectors();
        legacyPathsJoinBothWays();
        cylindersThatNeverMeetFallBack();
        cylinderFeedsAnyPart();
        removingAPathKeepsEachTurbine();
        oneClickHPipe();
        oneClickXPipe();
        crossoverRefusalsSayWhy();
        std::cout << "EngineLab exhaust path topology editor tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "EngineLab exhaust path topology editor tests failed: " << error.what() << '\n';
        return 1;
    }
}
