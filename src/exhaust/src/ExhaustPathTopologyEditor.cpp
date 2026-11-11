#include <enginelab/exhaust/ExhaustPathTopologyEditor.hpp>
#include <enginelab/exhaust/LegacyExhaustNetwork.hpp>

#include <algorithm>
#include <array>
#include <cstdlib>
#include <limits>
#include <numbers>
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
    // A junction inserted into existing routes: zero length, so no tuned
    // primary changes. Without an authored body it holds what a collector
    // trunk of the same bore would sweep, not an empty point.
    merge.lengthMm = 0.0;
    merge.diameterMm = path.geometry.collectorDiameterMm;
    const auto radiusM = path.geometry.collectorDiameterMm * 0.0005;
    merge.volumeLitres = path.geometry.collectorVolumeLitres > 0.0 ? path.geometry.collectorVolumeLitres
        : std::numbers::pi * radiusM * radiusM * legacyCollectorTrunkLengthMm * 0.001 * 1'000.0;
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

[[nodiscard]] ExhaustComponentConfig* findComponent(ExhaustNetworkConfig& network, std::uint32_t id) {
    const auto found = std::find_if(network.components.begin(), network.components.end(),
        [id](const ExhaustComponentConfig& component) { return component.id == id; });
    return found != network.components.end() ? &*found : nullptr;
}

[[nodiscard]] const ExhaustComponentConfig* findComponent(const ExhaustNetworkConfig& network, std::uint32_t id) {
    const auto found = std::find_if(network.components.begin(), network.components.end(),
        [id](const ExhaustComponentConfig& component) { return component.id == id; });
    return found != network.components.end() ? &*found : nullptr;
}

/** A resonator with no outlet: a closed acoustic branch, not a flow route. */
[[nodiscard]] bool isTerminalBranch(const ExhaustNetworkConfig& network, std::uint32_t id) {
    const auto* component = findComponent(network, id);
    return component != nullptr && component->type == ExhaustComponentType::resonator
        && std::none_of(network.connections.begin(), network.connections.end(),
               [id](const ExhaustComponentConnectionConfig& edge) { return edge.fromComponentId == id; });
}

[[nodiscard]] bool isSideBranch(const ExhaustComponentConnectionConfig& edge) noexcept {
    return edge.fromPort == exhaustSideBranchPort || edge.toPort == exhaustSideBranchPort;
}

/** The indices of the edges the flow leaves `id` by (no closed branch, no
 *  balance pipe). */
[[nodiscard]] std::vector<std::size_t> flowOutputs(const ExhaustNetworkConfig& network, std::uint32_t id) {
    std::vector<std::size_t> result;
    for (std::size_t index = 0; index < network.connections.size(); ++index) {
        const auto& edge = network.connections[index];
        if (edge.fromComponentId == id && !isSideBranch(edge) && !isTerminalBranch(network, edge.toComponentId))
            result.push_back(index);
    }
    return result;
}

/** Every component `id` is reached from, `id` excluded. */
[[nodiscard]] std::unordered_set<std::uint32_t> ancestorsOf(const ExhaustNetworkConfig& network, std::uint32_t id) {
    std::unordered_set<std::uint32_t> result;
    std::vector<std::uint32_t> pending { id };
    while (!pending.empty()) {
        const auto current = pending.back();
        pending.pop_back();
        for (const auto& edge : network.connections)
            if (edge.toComponentId == current && result.insert(edge.fromComponentId).second)
                pending.push_back(edge.fromComponentId);
    }
    result.erase(id);
    return result;
}

/** Drops what nothing feeds (no cylinder, no component), repeatedly. */
void pruneOrphans(ExhaustNetworkConfig& network) {
    for (auto changed = true; changed;) {
        changed = false;
        for (const auto& component : network.components) {
            const auto id = component.id;
            const auto fed = std::any_of(network.connections.begin(), network.connections.end(),
                                 [id](const auto& edge) { return edge.toComponentId == id; })
                || std::any_of(network.cylinderConnections.begin(), network.cylinderConnections.end(),
                       [id](const auto& mapping) { return mapping.componentId == id; });
            if (fed) continue;
            eraseComponent(network, id);
            changed = true;
            break;
        }
    }
}

/** A neighbour's primary pipe and the collector it feeds, for `cylinderId`
 *  to copy: the mapped cylinder nearest by id whose first component is a
 *  pipe flowing straight into a merge. */
[[nodiscard]] std::optional<std::pair<ExhaustComponentConfig, std::uint32_t>> neighbourPrimary(
    const ExhaustNetworkConfig& network, std::uint32_t cylinderId) {
    std::optional<std::pair<ExhaustComponentConfig, std::uint32_t>> best;
    auto bestDistance = std::numeric_limits<long long>::max();
    for (const auto& mapping : network.cylinderConnections) {
        const auto* root = findComponent(network, mapping.componentId);
        if (root == nullptr || root->type != ExhaustComponentType::pipe) continue;
        const auto outputs = flowOutputs(network, root->id);
        if (outputs.size() != 1U) continue;
        const auto* collector = findComponent(network, network.connections[outputs.front()].toComponentId);
        if (collector == nullptr || collector->type != ExhaustComponentType::merge) continue;
        const auto distance = std::llabs(static_cast<long long>(mapping.cylinderId) - static_cast<long long>(cylinderId));
        if (distance < bestDistance) {
            bestDistance = distance;
            best = std::pair { *root, collector->id };
        }
    }
    return best;
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

    // Where the neighbours' primaries meet in a collector, the cylinder joins
    // them there with a primary like theirs: moving one cylinder of a V8's
    // bank into the other's 4-into-1 makes a 5-into-1, not a chain of merges
    // on one pipe (owner report 2026-11-11).
    if (const auto neighbour = neighbourPrimary(network, cylinderId)) {
        auto primary = neighbour->first;
        primary.id = nextComponentId(network);
        if (primary.id == 0) return "No component ID is available in the destination path";
        network.components.push_back(primary);
        network.cylinderConnections.push_back({ cylinderId, primary.id });
        network.connections.push_back({ primary.id, neighbour->second });
        return std::nullopt;
    }

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
    return { selectedPathId, {}, {} };
}

/** `component` added under a fresh id of `network`; that id (0 if none). */
[[nodiscard]] std::uint32_t addCopy(ExhaustNetworkConfig& network, ExhaustComponentConfig component) {
    component.id = nextComponentId(network);
    if (component.id != 0) network.components.push_back(component);
    return component.id;
}

[[nodiscard]] ExhaustComponentConfig pipeOf(std::uint32_t id, double lengthMm, double diameterMm) {
    ExhaustComponentConfig pipe;
    pipe.id = id;
    pipe.type = ExhaustComponentType::pipe;
    pipe.lengthMm = lengthMm;
    pipe.diameterMm = diameterMm;
    return pipe;
}

/** The removed path's cylinders, up to and with its collector, grafted into
 *  the destination; a leg from each collector into a new Y, which takes the
 *  destination collector's way on. Empty, or why it cannot. */
[[nodiscard]] std::optional<std::string> joinWithY(ExhaustPathConfig& destination, const ExhaustPathConfig& source) {
    auto& network = *destination.network;
    const auto& removed = *source.network;
    const auto ownJunction = ExhaustNetworkEditor::firstCommonJunction(network);
    const auto removedJunction = ExhaustNetworkEditor::firstCommonJunction(removed);
    if (!ownJunction || !removedJunction) return "a path's cylinders never meet in one collector";
    const auto outputs = flowOutputs(network, *ownJunction);
    if (outputs.size() != 1U) return "the destination's collector has no single way on";

    // What is kept of the removed path: its cylinders' pipes up to and with
    // the collector, and the closed branches hanging off them.
    auto kept = ancestorsOf(removed, *removedJunction);
    kept.insert(*removedJunction);
    for (const auto& edge : removed.connections)
        if (kept.contains(edge.fromComponentId) && isTerminalBranch(removed, edge.toComponentId))
            kept.insert(edge.toComponentId);
    if (network.components.size() + kept.size() + 3U > maximumComponents)
        return "the joined graph would pass 256 components";
    std::unordered_map<std::uint32_t, std::uint32_t> renamed;
    for (const auto& component : removed.components) {
        if (!kept.contains(component.id)) continue;
        const auto id = addCopy(network, component);
        if (id == 0) return "no component id is left";
        renamed[component.id] = id;
    }
    for (const auto& edge : removed.connections)
        if (renamed.contains(edge.fromComponentId) && renamed.contains(edge.toComponentId))
            network.connections.push_back({ renamed[edge.fromComponentId], renamed[edge.toComponentId],
                                            edge.fromPort, edge.toPort });
    for (const auto& mapping : removed.cylinderConnections) {
        const auto root = renamed.find(mapping.componentId);
        if (root == renamed.end()) return "a removed cylinder does not reach its collector";
        network.cylinderConnections.push_back({ mapping.cylinderId, root->second });
    }

    // The Y: as the designer's (its legs a fifth of the pipe on, 150 to
    // 400 mm), the pipe on giving up the legs' length when it has it.
    const auto edge = network.connections[outputs.front()];
    auto* onward = findComponent(network, edge.toComponentId);
    const auto* own = findComponent(network, *ownJunction);
    const auto* theirs = findComponent(network, renamed[*removedJunction]);
    if (onward == nullptr || own == nullptr || theirs == nullptr) return "the collectors were not found";
    const auto onwardIsPipe = onward->type == ExhaustComponentType::pipe;
    const auto legMm = onwardIsPipe ? std::clamp(0.2 * onward->lengthMm, 150.0, 400.0) : 250.0;
    if (onwardIsPipe && onward->lengthMm - legMm >= 100.0) onward->lengthMm -= legMm;
    const auto yDiameter = onwardIsPipe ? onward->diameterMm : own->diameterMm;
    // Read before the legs are added: adding moves the components.
    const auto ownDiameter = own->diameterMm;
    const auto theirDiameter = theirs->diameterMm;
    const auto ownLeg = addCopy(network, pipeOf(0, legMm, ownDiameter));
    const auto theirLeg = addCopy(network, pipeOf(0, legMm, theirDiameter));
    auto y = pipeOf(0, 0.0, yDiameter);
    y.type = ExhaustComponentType::merge;
    const auto yId = addCopy(network, y);
    if (ownLeg == 0 || theirLeg == 0 || yId == 0) return "no component id is left";
    network.connections[outputs.front()] = { *ownJunction, ownLeg, edge.fromPort, unspecifiedExhaustComponentPort };
    network.connections.push_back({ ownLeg, yId });
    network.connections.push_back({ renamed[*removedJunction], theirLeg });
    network.connections.push_back({ theirLeg, yId });
    network.connections.push_back({ yId, edge.toComponentId, unspecifiedExhaustComponentPort, edge.toPort });
    for (const auto cylinderId : source.cylinderIds) destination.cylinderIds.push_back(cylinderId);
    return std::nullopt;
}

/** Every cylinder of the destination and of `added`, each with a primary
 *  like the destination's first, all into its collector (or, for a lone
 *  cylinder's pipe, a new collector where that pipe was). Empty, or why it
 *  cannot. */
[[nodiscard]] std::optional<std::string> rebuildInto(ExhaustPathConfig& destination,
                                                     const std::vector<std::uint32_t>& added) {
    auto cylinders = destination.cylinderIds;
    cylinders.insert(cylinders.end(), added.begin(), added.end());
    if (!destination.network) {
        // The engine builds a scalar path's N-into-1 itself.
        destination.cylinderIds = std::move(cylinders);
        return std::nullopt;
    }
    auto& network = *destination.network;
    const auto junction = ExhaustNetworkEditor::firstCommonJunction(network);
    if (!junction) return "the destination's cylinders never meet in one collector";
    if (network.components.size() + cylinders.size() + 1U > maximumComponents)
        return "the rebuilt graph would pass 256 components";
    auto primary = primaryFor(destination, 0);
    for (const auto& mapping : network.cylinderConnections)
        if (const auto* root = findComponent(network, mapping.componentId);
                root != nullptr && root->type == ExhaustComponentType::pipe) {
            primary = *root;
            break;
        }

    auto collector = *junction;
    const auto* found = findComponent(network, collector);
    if (found == nullptr) return "the collector was not found";
    if (found->type != ExhaustComponentType::merge) {
        // A lone cylinder: its pipe makes way for a collector, which takes
        // its way on; anything else first in line stays, after the collector.
        auto merge = mergeFor(destination, 0);
        if (found->type == ExhaustComponentType::pipe) {
            const auto outputs = flowOutputs(network, collector);
            if (outputs.size() != 1U) return "the lone pipe has no single way on";
            const auto edge = network.connections[outputs.front()];
            merge.diameterMm = std::max(merge.diameterMm, found->diameterMm);
            const auto mergeId = addCopy(network, merge);
            if (mergeId == 0) return "no component id is left";
            for (const auto ancestor : ancestorsOf(network, collector)) eraseComponent(network, ancestor);
            eraseComponent(network, collector);
            network.connections.push_back({ mergeId, edge.toComponentId, unspecifiedExhaustComponentPort, edge.toPort });
            collector = mergeId;
        } else {
            merge.diameterMm = std::max(merge.diameterMm, found->diameterMm);
            const auto mergeId = addCopy(network, merge);
            if (mergeId == 0) return "no component id is left";
            for (const auto ancestor : ancestorsOf(network, collector)) eraseComponent(network, ancestor);
            network.cylinderConnections.clear();
            network.connections.push_back({ mergeId, collector });
            collector = mergeId;
        }
    } else {
        for (const auto ancestor : ancestorsOf(network, collector)) eraseComponent(network, ancestor);
    }
    network.cylinderConnections.clear();
    for (const auto cylinderId : cylinders) {
        const auto id = addCopy(network, primary);
        if (id == 0) return "no component id is left";
        network.cylinderConnections.push_back({ cylinderId, id });
        network.connections.push_back({ id, collector });
    }
    // Closed branches that hung off the old primaries.
    pruneOrphans(network);
    destination.cylinderIds = std::move(cylinders);
    return std::nullopt;
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
    EngineConfig& config, std::uint32_t pathId, std::uint32_t destinationPathId, ExhaustPathJoin join) {
    if (pathId == destinationPathId) return reject("A path cannot be removed into itself");
    if (const auto error = validateStartingPoint(config)) return reject(*error);
    if (config.exhaustPaths.size() <= 1U) return reject("The engine must retain at least one exhaust path");

    auto candidate = config;
    const auto source = findPath(candidate, pathId);
    const auto destination = findPath(candidate, destinationPathId);
    if (source == candidate.exhaustPaths.end() || destination == candidate.exhaustPaths.end())
        return reject("Path to remove or destination path does not exist");
    const auto cylinders = source->cylinderIds;
    std::optional<std::string> why;
    if (join == ExhaustPathJoin::yJunction) {
        auto removed = *source;
        if (!removed.network) removed.network = makeEditableExhaustNetwork(removed);
        if (!destination->network) destination->network = makeEditableExhaustNetwork(*destination);
        why = joinWithY(*destination, removed);
    } else {
        why = rebuildInto(*destination, cylinders);
    }
    if (!why) {
        candidate.exhaustPaths.erase(findPath(candidate, pathId));
        refreshBankPathHints(candidate);
        const auto error = validateEngineConfig(candidate);
        if (!error) {
            config = std::move(candidate);
            return { destinationPathId, {}, {} };
        }
        why = "the joined graph is not valid: " + *error;
    }

    // What is left when the cylinders do not meet in one junction: one at a
    // time, each onto the collector its nearest neighbour feeds.
    auto fallback = config;
    const auto target = findPath(fallback, destinationPathId);
    for (const auto cylinderId : cylinders)
        if (const auto error = addCylinderToPath(*target, cylinderId)) return reject(*error);
    fallback.exhaustPaths.erase(findPath(fallback, pathId));
    auto result = commitIfValid(config, std::move(fallback), destinationPathId);
    if (result) result.note = "Each cylinder joined the collector its nearest neighbour feeds (" + *why + ").";
    return result;
}

ExhaustComponentConfig ExhaustNetworkEditor::defaultComponent(ExhaustComponentType type,
                                                              std::uint32_t id) noexcept {
    ExhaustComponentConfig result;
    result.id = id;
    result.type = type;
    switch (type) {
    case ExhaustComponentType::pipe:
        result.lengthMm = 450.0;
        result.diameterMm = 42.0;
        break;
    case ExhaustComponentType::merge:
        result.lengthMm = 0.0;
        result.diameterMm = 58.0;
        result.volumeLitres = 1.5;
        break;
    case ExhaustComponentType::splitter:
        result.lengthMm = 0.0;
        result.diameterMm = 58.0;
        break;
    case ExhaustComponentType::resonator:
        // The most useful neutral starting point is a closed quarter-wave
        // branch. One incoming connection and no output gives it that meaning;
        // adding an output keeps the historical inline-duct behaviour.
        result.lengthMm = 535.0;
        result.diameterMm = 28.0;
        break;
    case ExhaustComponentType::muffler:
        result.lengthMm = 480.0;
        result.diameterMm = 62.0;
        result.volumeLitres = 8.0;
        result.restriction = 0.28;
        result.acousticGain = 0.72;
        break;
    case ExhaustComponentType::catalyst:
        result.lengthMm = 180.0;
        result.diameterMm = 56.0;
        result.restriction = 0.18;
        result.dischargeCoefficient = 0.68;
        result.catalystCellDensityCpsi = 400.0;
        result.catalystOpenAreaRatio = 0.80;
        result.catalystSubstrateVolumetricHeatCapacityJPerM3K = 2'000'000.0;
        break;
    case ExhaustComponentType::outlet:
        // A tailpipe is a pipe. Zero here used to mean "the system ends", but
        // the layout has to mesh it anyway, and a length it has to invent is
        // the length that then sets the CFL limit for the whole network. Give
        // it the real thing instead; the user can still shorten it.
        result.lengthMm = 300.0;
        result.diameterMm = 65.0;
        break;
    case ExhaustComponentType::crossover:
        result.lengthMm = 0.0;
        result.diameterMm = 80.0;
        result.restriction = 0.06;
        // Estimated neutral starting point, not a measured calibration.
        result.crossoverCoupling = 0.30;
        break;
    }
    return result;
}

std::uint32_t ExhaustNetworkEditor::nextComponentId(const ExhaustNetworkConfig& network) {
    return enginelab::nextComponentId(network);
}

std::optional<std::uint32_t> ExhaustNetworkEditor::firstCommonJunction(const ExhaustNetworkConfig& network) {
    std::unordered_set<std::uint32_t> mapped;
    for (const auto& mapping : network.cylinderConnections) mapped.insert(mapping.cylinderId);
    if (mapped.empty()) return std::nullopt;
    // The cylinders reaching each component, relaxed along the flow (not
    // across a balance pipe: an H's two sides do not meet in it).
    std::unordered_map<std::uint32_t, std::unordered_set<std::uint32_t>> reaching;
    for (const auto& component : network.components) reaching[component.id];
    for (const auto& mapping : network.cylinderConnections)
        if (const auto found = reaching.find(mapping.componentId); found != reaching.end())
            found->second.insert(mapping.cylinderId);
    for (std::size_t pass = 0; pass <= network.components.size(); ++pass) {
        auto changed = false;
        for (const auto& edge : network.connections) {
            if (isSideBranch(edge)) continue;
            const auto from = reaching.find(edge.fromComponentId);
            const auto to = reaching.find(edge.toComponentId);
            if (from == reaching.end() || to == reaching.end()) continue;
            for (const auto cylinderId : from->second) changed = to->second.insert(cylinderId).second || changed;
        }
        if (!changed) break;
    }
    const auto full = [&](std::uint32_t id) {
        const auto found = reaching.find(id);
        return found != reaching.end() && found->second.size() == mapped.size();
    };
    for (const auto& component : network.components) {
        if (!full(component.id)) continue;
        const auto fedFull = std::any_of(network.connections.begin(), network.connections.end(),
            [&](const ExhaustComponentConnectionConfig& edge) {
                return edge.toComponentId == component.id && !isSideBranch(edge) && full(edge.fromComponentId);
            });
        if (fedFull) continue;
        // A splitter, an X or an outlet is where routes part or end, not
        // where they meet into one way on.
        if (component.type == ExhaustComponentType::splitter || component.type == ExhaustComponentType::crossover
            || component.type == ExhaustComponentType::outlet)
            return std::nullopt;
        return component.id;
    }
    return std::nullopt;
}

std::string ExhaustNetworkEditor::connectCylinder(ExhaustNetworkConfig& network, std::uint32_t cylinderId,
                                                  std::uint32_t componentId) {
    const auto* component = findComponent(network, componentId);
    if (component == nullptr) return "That component is not in this path.";
    if (component->type == ExhaustComponentType::crossover)
        return "A cylinder cannot feed an X-pipe directly: put a pipe before it.";
    const auto existing = std::find_if(network.cylinderConnections.begin(), network.cylinderConnections.end(),
        [cylinderId](const ExhaustCylinderConnectionConfig& mapping) { return mapping.cylinderId == cylinderId; });
    if (existing != network.cylinderConnections.end()) existing->componentId = componentId;
    else network.cylinderConnections.push_back({ cylinderId, componentId });
    return {};
}

bool ExhaustNetworkEditor::disconnectCylinder(ExhaustNetworkConfig& network, std::uint32_t cylinderId) {
    return std::erase_if(network.cylinderConnections, [cylinderId](const ExhaustCylinderConnectionConfig& mapping) {
        return mapping.cylinderId == cylinderId;
    }) > 0U;
}

std::string ExhaustNetworkEditor::connectComponents(ExhaustNetworkConfig& network, std::uint32_t from,
                                                    std::uint32_t to) {
    if (from == to) return "A component cannot be connected to itself.";
    const auto* source = findComponent(network, from);
    const auto* target = findComponent(network, to);
    if (source == nullptr || target == nullptr) return "Both components must be in this path.";
    if (network.connections.size() >= maximumConnections) return "Limit reached: 1024 connections per path.";
    if (std::any_of(network.connections.begin(), network.connections.end(),
            [from, to](const ExhaustComponentConnectionConfig& edge) {
                return edge.fromComponentId == from && edge.toComponentId == to;
            }))
        return "This connection already exists.";
    // An X-pipe's ports are explicit: the next free one on each side.
    const auto nextPort = [&network](std::uint32_t id, bool output) -> std::optional<std::uint8_t> {
        std::array<bool, 2> used {};
        for (const auto& edge : network.connections) {
            if (output && edge.fromComponentId == id && edge.fromPort <= 1U) used[edge.fromPort] = true;
            if (!output && edge.toComponentId == id && edge.toPort <= 1U) used[edge.toPort] = true;
        }
        for (std::uint8_t port = 0; port <= 1U; ++port)
            if (!used[port]) return port;
        return std::nullopt;
    };
    auto fromPort = unspecifiedExhaustComponentPort;
    auto toPort = unspecifiedExhaustComponentPort;
    if (source->type == ExhaustComponentType::crossover) {
        const auto port = nextPort(from, true);
        if (!port) return "Both outlet ports of the X-pipe are already in use.";
        fromPort = *port;
    }
    if (target->type == ExhaustComponentType::crossover) {
        const auto port = nextPort(to, false);
        if (!port) return "Both inlet ports of the X-pipe are already in use.";
        toPort = *port;
    }
    network.connections.push_back({ from, to, fromPort, toPort });
    return {};
}

std::string ExhaustNetworkEditor::replaceComponent(ExhaustNetworkConfig& network, std::uint32_t componentId,
                                                   ExhaustComponentType type) {
    auto* component = findComponent(network, componentId);
    if (component == nullptr) return "That component is not in this path.";
    if (component->type == type) return {};
    const auto lengthy = [](ExhaustComponentType kind) {
        return kind == ExhaustComponentType::pipe || kind == ExhaustComponentType::resonator
            || kind == ExhaustComponentType::muffler || kind == ExhaustComponentType::catalyst
            || kind == ExhaustComponentType::outlet;
    };
    auto replacement = defaultComponent(type, componentId);
    if (component->diameterMm >= 4.0) replacement.diameterMm = component->diameterMm;
    if (lengthy(type) && lengthy(component->type) && component->lengthMm >= 1.0)
        replacement.lengthMm = component->lengthMm;
    if (type == ExhaustComponentType::outlet) {
        replacement.acousticPositionM = component->acousticPositionM;
        replacement.acousticAxis = component->acousticAxis;
    }
    *component = replacement;
    // The connections stay; their ports follow the new type: an X-pipe's
    // are numbered, a merge's or a splitter's balance-pipe port stays, any
    // other is unspecified.
    const auto junction = type == ExhaustComponentType::merge || type == ExhaustComponentType::splitter;
    auto nextInput = std::uint8_t { 0 };
    auto nextOutput = std::uint8_t { 0 };
    const auto port = [&](std::uint8_t current, std::uint8_t& next) -> std::uint8_t {
        if (type == ExhaustComponentType::crossover) return next <= 1U ? next++ : unspecifiedExhaustComponentPort;
        return junction && current == exhaustSideBranchPort ? current : unspecifiedExhaustComponentPort;
    };
    for (auto& edge : network.connections) {
        if (edge.toComponentId == componentId) edge.toPort = port(edge.toPort, nextInput);
        if (edge.fromComponentId == componentId) edge.fromPort = port(edge.fromPort, nextOutput);
    }
    return {};
}

} // namespace enginelab
