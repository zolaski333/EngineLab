#include <enginelab/exhaust/ExhaustPathTopologyEditor.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>

#include <algorithm>
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
        std::cout << "EngineLab exhaust path topology editor tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "EngineLab exhaust path topology editor tests failed: " << error.what() << '\n';
        return 1;
    }
}
