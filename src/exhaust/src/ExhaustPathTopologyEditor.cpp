#include <enginelab/exhaust/ExhaustPathTopologyEditor.hpp>

#include <algorithm>
#include <limits>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace enginelab {
namespace {
constexpr std::size_t maximumPaths = 8;
constexpr std::size_t maximumComponents = 256;
constexpr std::size_t maximumConnections = 1'024;

using PathIterator = std::vector<ExhaustPathConfig>::iterator;

[[nodiscard]] PathIterator findPath(EngineConfig& config, std::uint32_t id) {
    return std::find_if(config.exhaustPaths.begin(), config.exhaustPaths.end(),
        [id](const ExhaustPathConfig& path) { return path.id == id; });
}

[[nodiscard]] bool containsCylinder(const ExhaustPathConfig& path,
                                    std::uint32_t cylinderId) noexcept {
    return std::find(path.cylinderIds.begin(), path.cylinderIds.end(), cylinderId)
        != path.cylinderIds.end();
}

[[nodiscard]] std::uint32_t nextPathId(const EngineConfig& config) {
    std::unordered_set<std::uint32_t> used;
    used.reserve(config.exhaustPaths.size());
    for (const auto& path : config.exhaustPaths) used.insert(path.id);
    for (std::uint64_t candidate = 1;
         candidate <= std::numeric_limits<std::uint32_t>::max(); ++candidate) {
        if (!used.contains(static_cast<std::uint32_t>(candidate)))
            return static_cast<std::uint32_t>(candidate);
    }
    return 0;
}

[[nodiscard]] std::uint32_t nextComponentId(const ExhaustNetworkConfig& network) {
    std::unordered_set<std::uint32_t> used;
    used.reserve(network.components.size());
    for (const auto& component : network.components) used.insert(component.id);
    for (std::uint64_t candidate = 1;
         candidate <= std::numeric_limits<std::uint32_t>::max(); ++candidate) {
        if (!used.contains(static_cast<std::uint32_t>(candidate)))
            return static_cast<std::uint32_t>(candidate);
    }
    return 0;
}

[[nodiscard]] ExhaustComponentConfig primaryFor(const ExhaustPathConfig& path,
                                                 std::uint32_t id) noexcept {
    ExhaustComponentConfig primary;
    primary.id = id;
    primary.type = ExhaustComponentType::pipe;
    primary.lengthMm = path.geometry.primaryLengthMm;
    primary.diameterMm = path.geometry.primaryDiameterMm;
    return primary;
}

[[nodiscard]] ExhaustComponentConfig mergeFor(const ExhaustPathConfig& path,
                                               std::uint32_t id) noexcept {
    ExhaustComponentConfig merge;
    merge.id = id;
    merge.type = ExhaustComponentType::merge;
    merge.lengthMm = 0.0;
    merge.diameterMm = path.geometry.collectorDiameterMm;
    merge.volumeLitres = path.geometry.collectorVolumeLitres;
    return merge;
}

void eraseComponent(ExhaustNetworkConfig& network, std::uint32_t id) {
    std::erase_if(network.components,
        [id](const ExhaustComponentConfig& component) { return component.id == id; });
    std::erase_if(network.connections, [id](const ExhaustComponentConnectionConfig& edge) {
        return edge.fromComponentId == id || edge.toComponentId == id;
    });
    std::erase_if(network.cylinderConnections, [id](const ExhaustCylinderConnectionConfig& mapping) {
        return mapping.componentId == id;
    });
}

/** Prunes the removed cylinder's now-unreachable root and collapses 1-input merges. */
void repairAfterCylinderRemoval(ExhaustNetworkConfig& network) {
    bool changed = true;
    while (changed) {
        changed = false;
        std::unordered_map<std::uint32_t, std::size_t> componentIncoming;
        std::unordered_map<std::uint32_t, std::size_t> cylinderIncoming;
        for (const auto& component : network.components) {
            componentIncoming.emplace(component.id, 0U);
            cylinderIncoming.emplace(component.id, 0U);
        }
        for (const auto& edge : network.connections) {
            ++componentIncoming[edge.toComponentId];
        }
        for (const auto& mapping : network.cylinderConnections)
            ++cylinderIncoming[mapping.componentId];

        const auto orphan = std::find_if(network.components.begin(), network.components.end(),
            [&componentIncoming, &cylinderIncoming](const ExhaustComponentConfig& component) {
                return componentIncoming[component.id] + cylinderIncoming[component.id] == 0U;
            });
        if (orphan != network.components.end()) {
            const auto id = orphan->id;
            eraseComponent(network, id);
            changed = true;
            continue;
        }

        const auto unaryMerge = std::find_if(network.components.begin(), network.components.end(),
            [&componentIncoming, &cylinderIncoming](const ExhaustComponentConfig& component) {
                return component.type == ExhaustComponentType::merge
                    && componentIncoming[component.id] + cylinderIncoming[component.id] == 1U;
            });
        if (unaryMerge == network.components.end()) continue;

        const auto mergeId = unaryMerge->id;
        const auto output = std::find_if(network.connections.begin(), network.connections.end(),
            [mergeId](const ExhaustComponentConnectionConfig& edge) {
                return edge.fromComponentId == mergeId;
            });
        if (output == network.connections.end()) {
            eraseComponent(network, mergeId);
            changed = true;
            continue;
        }
        const auto outputId = output->toComponentId;
        const auto cylinderInput = std::find_if(network.cylinderConnections.begin(),
            network.cylinderConnections.end(), [mergeId](const ExhaustCylinderConnectionConfig& mapping) {
                return mapping.componentId == mergeId;
            });
        if (cylinderInput != network.cylinderConnections.end()) {
            cylinderInput->componentId = outputId;
        } else {
            const auto input = std::find_if(network.connections.begin(), network.connections.end(),
                [mergeId](const ExhaustComponentConnectionConfig& edge) {
                    return edge.toComponentId == mergeId;
                });
            if (input != network.connections.end()) input->toComponentId = outputId;
        }
        eraseComponent(network, mergeId);
        changed = true;
    }
}

[[nodiscard]] std::optional<ExhaustNetworkConfig> networkForCylinder(
    const ExhaustPathConfig& path, std::uint32_t cylinderId) {
    if (!path.network) return std::nullopt;
    const auto& source = *path.network;
    const auto mapping = std::find_if(source.cylinderConnections.begin(),
        source.cylinderConnections.end(), [cylinderId](const ExhaustCylinderConnectionConfig& item) {
            return item.cylinderId == cylinderId;
        });
    if (mapping == source.cylinderConnections.end()) return std::nullopt;

    std::unordered_map<std::uint32_t, std::vector<std::uint32_t>> outgoing;
    for (const auto& component : source.components) outgoing.try_emplace(component.id);
    for (const auto& edge : source.connections)
        outgoing[edge.fromComponentId].push_back(edge.toComponentId);
    std::unordered_set<std::uint32_t> reachable;
    std::vector<std::uint32_t> pending { mapping->componentId };
    while (!pending.empty()) {
        const auto id = pending.back();
        pending.pop_back();
        if (!reachable.insert(id).second) continue;
        if (const auto next = outgoing.find(id); next != outgoing.end())
            pending.insert(pending.end(), next->second.begin(), next->second.end());
    }

    ExhaustNetworkConfig result;
    result.components.reserve(reachable.size());
    for (const auto& component : source.components)
        if (reachable.contains(component.id)) result.components.push_back(component);
    for (const auto& edge : source.connections) {
        if (reachable.contains(edge.fromComponentId) && reachable.contains(edge.toComponentId))
            result.connections.push_back(edge);
    }
    result.cylinderConnections.push_back({ cylinderId, mapping->componentId });
    // Shared downstream merges become unary once the other cylinders are no
    // longer part of this path. Collapse them while retaining every tuned pipe,
    // resonator, catalyst, muffler, splitter and outlet on the selected routes.
    repairAfterCylinderRemoval(result);
    return result;
}

[[nodiscard]] std::optional<std::string> removeCylinderFromPath(
    ExhaustPathConfig& path, std::uint32_t cylinderId) {
    const auto cylinder = std::find(path.cylinderIds.begin(), path.cylinderIds.end(), cylinderId);
    if (cylinder == path.cylinderIds.end()) return "Cylinder is not assigned to the source exhaust path";
    if (path.cylinderIds.size() <= 1U) return "The source exhaust path must retain at least one cylinder";

    path.cylinderIds.erase(cylinder);
    if (!path.network) return std::nullopt;
    auto& network = *path.network;
    const auto mapping = std::find_if(network.cylinderConnections.begin(),
        network.cylinderConnections.end(), [cylinderId](const ExhaustCylinderConnectionConfig& item) {
            return item.cylinderId == cylinderId;
        });
    if (mapping == network.cylinderConnections.end())
        return "The source custom graph does not map the selected cylinder";
    network.cylinderConnections.erase(mapping);
    repairAfterCylinderRemoval(network);
    return std::nullopt;
}

[[nodiscard]] std::optional<std::string> addCylinderToPath(
    ExhaustPathConfig& path, std::uint32_t cylinderId) {
    if (containsCylinder(path, cylinderId))
        return "Cylinder is already assigned to the destination exhaust path";
    path.cylinderIds.push_back(cylinderId);
    if (!path.network) return std::nullopt;

    auto& network = *path.network;
    if (network.cylinderConnections.empty())
        return "Destination custom graph has no existing cylinder root";
    if (network.components.size() + 2U > maximumComponents)
        return "Destination custom graph has no component capacity for another cylinder";
    if (network.connections.size() + 2U > maximumConnections)
        return "Destination custom graph has no connection capacity for another cylinder";

    // Extend one explicit cylinder branch instead of attaching to an arbitrary
    // merge in a potentially multi-collector DAG. This preserves every existing
    // route and gives the added cylinder its own primary.
    const auto existingMappingIndex = std::size_t { 0 };
    const auto existingRootId = network.cylinderConnections[existingMappingIndex].componentId;
    std::vector<std::size_t> rootOutputIndices;
    for (std::size_t index = 0; index < network.connections.size(); ++index) {
        if (network.connections[index].fromComponentId == existingRootId)
            rootOutputIndices.push_back(index);
    }
    const auto primaryId = nextComponentId(network);
    if (primaryId == 0) return "No component ID is available in the destination path";
    network.components.push_back(primaryFor(path, primaryId));
    const auto mergeId = nextComponentId(network);
    if (mergeId == 0) return "No merge component ID is available in the destination path";
    network.components.push_back(mergeFor(path, mergeId));
    network.cylinderConnections.push_back({ cylinderId, primaryId });
    network.connections.push_back({ primaryId, mergeId });
    if (rootOutputIndices.size() == 1U) {
        // Common case: old cylinder -> old primary -> downstream. Splice the
        // new merge after the old primary, keeping one primary per cylinder.
        const auto downstreamId = network.connections[rootOutputIndices.front()].toComponentId;
        network.connections[rootOutputIndices.front()].toComponentId = mergeId;
        network.connections.push_back({ mergeId, downstreamId });
    } else {
        // A root outlet (zero outputs) or root splitter (multiple outputs) has
        // no single downstream edge to splice. Insert the merge before it;
        // the existing route stays intact after that zero-length junction.
        network.cylinderConnections[existingMappingIndex].componentId = mergeId;
        network.connections.push_back({ mergeId, existingRootId });
    }
    return std::nullopt;
}

void refreshBankPathHints(EngineConfig& config) {
    for (auto& bank : config.banks) {
        std::uint32_t commonPathId = 0;
        bool first = true;
        bool split = false;
        for (const auto cylinderId : bank.cylinderIds) {
            const auto path = std::find_if(config.exhaustPaths.begin(), config.exhaustPaths.end(),
                [cylinderId](const ExhaustPathConfig& item) {
                    return containsCylinder(item, cylinderId);
                });
            if (path == config.exhaustPaths.end()) {
                split = true;
                break;
            }
            if (first) {
                commonPathId = path->id;
                first = false;
            } else if (commonPathId != path->id) {
                split = true;
                break;
            }
        }
        bank.exhaustPathId = split || first ? 0U : commonPathId;
    }
}

[[nodiscard]] ExhaustPathEditResult reject(std::string error) {
    return { 0, std::move(error) };
}

[[nodiscard]] std::optional<std::string> validateStartingPoint(const EngineConfig& config) {
    if (const auto error = validateEngineConfig(config))
        return "Finish or repair the current configuration before changing path ownership: " + *error;
    return std::nullopt;
}

[[nodiscard]] ExhaustPathEditResult commitIfValid(
    EngineConfig& destination, EngineConfig candidate, std::uint32_t selectedPathId) {
    refreshBankPathHints(candidate);
    if (const auto error = validateEngineConfig(candidate))
        return reject("Path edit would make the engine invalid: " + *error);
    destination = std::move(candidate);
    return { selectedPathId, {} };
}
} // namespace

ExhaustPathEditResult ExhaustPathTopologyEditor::createPathFromCylinder(
    EngineConfig& config, std::uint32_t sourcePathId, std::uint32_t cylinderId) {
    if (const auto error = validateStartingPoint(config)) return reject(*error);
    if (config.exhaustPaths.size() >= maximumPaths)
        return reject("Audio supports at most eight exhaust paths");

    auto candidate = config;
    const auto source = findPath(candidate, sourcePathId);
    if (source == candidate.exhaustPaths.end()) return reject("Source exhaust path does not exist");
    const auto newId = nextPathId(candidate);
    if (newId == 0) return reject("No exhaust path ID is available");
    const auto transferredNetwork = networkForCylinder(*source, cylinderId);
    ExhaustPathConfig newPath;
    newPath.id = newId;
    newPath.cylinderIds = { cylinderId };
    newPath.geometry = source->geometry;
    newPath.impulseResponsePath = source->impulseResponsePath;
    newPath.audioVolume = source->audioVolume;
    newPath.network = transferredNetwork;
    newPath.inheritsGlobalGeometry = false;
    if (const auto error = removeCylinderFromPath(*source, cylinderId)) return reject(*error);
    candidate.exhaustPaths.push_back(std::move(newPath));
    return commitIfValid(config, std::move(candidate), newId);
}

ExhaustPathEditResult ExhaustPathTopologyEditor::moveCylinder(
    EngineConfig& config, std::uint32_t cylinderId,
    std::uint32_t sourcePathId, std::uint32_t destinationPathId) {
    if (sourcePathId == destinationPathId) return reject("Source and destination paths are identical");
    if (const auto error = validateStartingPoint(config)) return reject(*error);

    auto candidate = config;
    auto source = findPath(candidate, sourcePathId);
    auto destination = findPath(candidate, destinationPathId);
    if (source == candidate.exhaustPaths.end() || destination == candidate.exhaustPaths.end())
        return reject("Source or destination exhaust path does not exist");
    // Mutating a path does not invalidate vector iterators, so both remain valid.
    if (const auto error = removeCylinderFromPath(*source, cylinderId)) return reject(*error);
    if (const auto error = addCylinderToPath(*destination, cylinderId)) return reject(*error);
    return commitIfValid(config, std::move(candidate), destinationPathId);
}

ExhaustPathEditResult ExhaustPathTopologyEditor::removePath(
    EngineConfig& config, std::uint32_t pathId, std::uint32_t destinationPathId) {
    if (pathId == destinationPathId) return reject("A path cannot be removed into itself");
    if (const auto error = validateStartingPoint(config)) return reject(*error);
    if (config.exhaustPaths.size() <= 1U) return reject("The engine must retain at least one exhaust path");

    auto candidate = config;
    const auto source = findPath(candidate, pathId);
    const auto destination = findPath(candidate, destinationPathId);
    if (source == candidate.exhaustPaths.end() || destination == candidate.exhaustPaths.end())
        return reject("Path to remove or destination path does not exist");
    const auto cylinders = source->cylinderIds;
    for (const auto cylinderId : cylinders) {
        if (const auto error = addCylinderToPath(*destination, cylinderId)) return reject(*error);
    }
    candidate.exhaustPaths.erase(findPath(candidate, pathId));
    return commitIfValid(config, std::move(candidate), destinationPathId);
}

} // namespace enginelab
