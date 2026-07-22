#include <enginelab/gasdynamics/ExhaustNetworkLayout.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <optional>
#include <unordered_map>
#include <unordered_set>

namespace enginelab::gasdynamics {
namespace {

[[nodiscard]] bool finite(double value) noexcept {
    return std::isfinite(value);
}

[[nodiscard]] double circularArea(double diameterM) noexcept {
    const auto radiusM = 0.5 * diameterM;
    return std::numbers::pi * radiusM * radiusM;
}

[[nodiscard]] bool isJunctionType(ExhaustNodeType type) noexcept {
    return type == ExhaustNodeType::merge || type == ExhaustNodeType::splitter;
}

struct ElementReference final {
    bool junction { false };
    std::size_t index { 0 };
};

[[nodiscard]] std::size_t cellsForLength(double lengthM,
                                         double targetCellLengthM,
                                         const ExhaustNetworkDiscretisation& policy) noexcept {
    const auto requested = static_cast<std::size_t>(std::ceil(lengthM / targetCellLengthM));
    return std::clamp(requested, policy.minimumCellsPerDuct,
                      policy.maximumCellsPerDuct);
}

} // namespace

bool ExhaustNetworkDiscretisation::valid() const noexcept {
    return finite(targetCellLengthM) && targetCellLengthM > 0.0
        && finite(minimumResolvedLengthM) && minimumResolvedLengthM > 0.0
        && minimumCellsPerDuct >= 1
        && maximumCellsPerDuct >= minimumCellsPerDuct
        && maximumTotalCells >= minimumCellsPerDuct;
}

ExhaustNetworkLayout ExhaustNetworkLayout::compile(
    const ExhaustGraph& graph,
    ExhaustNetworkDiscretisation discretisation) {
    ExhaustNetworkLayout layout;
    layout.discretisation_ = discretisation;
    if (!discretisation.valid()) {
        layout.diagnostics_.push_back(
            { ExhaustNetworkLayoutIssue::invalidDiscretisation, 0 });
        return layout;
    }

    std::unordered_map<std::uint32_t, const ExhaustNode*> graphNodes;
    graphNodes.reserve(graph.nodes().size());
    for (const auto& node : graph.nodes()) graphNodes.try_emplace(node.id, &node);

    std::unordered_map<std::uint32_t, ElementReference> elements;
    elements.reserve(graph.nodes().size());
    for (const auto& node : graph.nodes()) {
        if (node.type == ExhaustNodeType::port) continue;
        const auto diameterM = std::clamp(node.diameterMm * 0.001, 0.005, 0.5);
        const auto connectionAreaM2 = circularArea(diameterM);
        if (isJunctionType(node.type)) {
            auto volumeM3 = finite(node.volumeLitres) && node.volumeLitres > 0.0
                ? node.volumeLitres * 0.001 : 0.0;
            auto derived = false;
            if (!(volumeM3 > 0.0)) {
                // A branch junction has finite physical extent even when the
                // author did not draw a plenum. One hydraulic diameter is the
                // smallest geometry-based volume that does not invent a tuning
                // frequency or depend on the numerical mesh length.
                volumeM3 = connectionAreaM2 * diameterM;
                derived = true;
            }
            const auto index = layout.junctions_.size();
            layout.junctions_.push_back({
                node.id,
                node.sourceComponentId,
                node.pathIndex,
                node.type,
                volumeM3,
                diameterM,
                std::max(0.0, node.localLossCoefficient),
                derived,
            });
            elements.emplace(node.id, ElementReference { true, index });
            continue;
        }

        auto lengthM = finite(node.lengthMm) ? node.lengthMm * 0.001 : 0.0;
        const auto requestedVolumeM3 = finite(node.volumeLitres) && node.volumeLitres > 0.0
            ? node.volumeLitres * 0.001 : 0.0;
        auto lengthWasDerived = false;
        if (!(lengthM > 0.0)) {
            lengthM = requestedVolumeM3 > 0.0
                ? requestedVolumeM3 / connectionAreaM2
                : discretisation.minimumResolvedLengthM;
            lengthM = std::max(lengthM, discretisation.minimumResolvedLengthM);
            lengthWasDerived = true;
            layout.diagnostics_.push_back(
                { ExhaustNetworkLayoutIssue::zeroLengthComponentResolved, node.id });
        }
        auto flowAreaM2 = connectionAreaM2;
        auto areaWasDerivedFromVolume = false;
        if (requestedVolumeM3 > connectionAreaM2 * lengthM * (1.0 + 1.0e-9)) {
            flowAreaM2 = requestedVolumeM3 / lengthM;
            areaWasDerivedFromVolume = true;
        }
        const auto volumeM3 = flowAreaM2 * lengthM;
        const auto hydraulicDiameterM = areaWasDerivedFromVolume
            ? 2.0 * std::sqrt(flowAreaM2 / std::numbers::pi)
            : diameterM;
        const auto index = layout.ducts_.size();
        layout.ducts_.push_back({
            node.id,
            node.sourceComponentId,
            node.pathIndex,
            node.type,
            lengthM,
            flowAreaM2,
            connectionAreaM2,
            hydraulicDiameterM,
            volumeM3,
            std::max(0.0, node.localLossCoefficient),
            std::clamp(node.dischargeCoefficient, 0.02, 1.5),
            0,
            lengthWasDerived,
            areaWasDerivedFromVolume,
        });
        elements.emplace(node.id, ElementReference { false, index });
    }

    const auto assignCellCounts = [&layout, &discretisation](double targetLengthM) noexcept {
        auto total = std::size_t { 0 };
        for (auto& duct : layout.ducts_) {
            duct.cellCount = cellsForLength(duct.lengthM, targetLengthM, discretisation);
            total += duct.cellCount;
        }
        return total;
    };
    layout.totalCellCount_ = assignCellCounts(discretisation.targetCellLengthM);
    if (layout.totalCellCount_ > discretisation.maximumTotalCells) {
        auto coarsenedTarget = discretisation.targetCellLengthM;
        for (std::size_t iteration = 0;
             iteration < 32 && layout.totalCellCount_ > discretisation.maximumTotalCells;
             ++iteration) {
            coarsenedTarget *= 1.35;
            layout.totalCellCount_ = assignCellCounts(coarsenedTarget);
        }
    }
    if (layout.totalCellCount_ > discretisation.maximumTotalCells) {
        layout.diagnostics_.push_back(
            { ExhaustNetworkLayoutIssue::cellBudgetExceeded, 0 });
    }

    const auto endpointFor = [&elements](std::uint32_t nodeId,
                                         bool downstreamEnd) -> std::optional<ExhaustEndpoint> {
        const auto found = elements.find(nodeId);
        if (found == elements.end()) return std::nullopt;
        if (found->second.junction)
            return ExhaustEndpoint {
                ExhaustEndpointType::junction, found->second.index, nodeId };
        return ExhaustEndpoint {
            downstreamEnd ? ExhaustEndpointType::ductOutlet
                          : ExhaustEndpointType::ductInlet,
            found->second.index,
            nodeId,
        };
    };

    std::unordered_map<std::uint32_t, std::vector<std::uint32_t>> outgoing;
    outgoing.reserve(graph.nodes().size());
    for (const auto& edge : graph.edges()) outgoing[edge.from].push_back(edge.to);

    for (const auto& edge : graph.edges()) {
        const auto fromNode = graphNodes.find(edge.from);
        const auto toNode = graphNodes.find(edge.to);
        if (fromNode == graphNodes.end() || toNode == graphNodes.end()) {
            layout.diagnostics_.push_back(
                { ExhaustNetworkLayoutIssue::missingGraphNode,
                  fromNode == graphNodes.end() ? edge.from : edge.to });
            continue;
        }
        if (fromNode->second->type == ExhaustNodeType::port) continue;
        const auto upstream = endpointFor(edge.from, true);
        const auto downstream = endpointFor(edge.to, false);
        if (!upstream || !downstream) {
            layout.diagnostics_.push_back(
                { ExhaustNetworkLayoutIssue::unresolvedEndpoint,
                  !upstream ? edge.from : edge.to });
            continue;
        }
        layout.interfaces_.push_back({ *upstream, *downstream });
    }

    std::vector<std::uint32_t> cylinderIds;
    std::unordered_set<std::uint32_t> seenCylinderIds;
    for (const auto& route : graph.routes()) {
        if (seenCylinderIds.insert(route.cylinderId).second)
            cylinderIds.push_back(route.cylinderId);
    }
    for (const auto cylinderId : cylinderIds) {
        const auto node = graphNodes.find(cylinderId);
        if (node == graphNodes.end()) {
            layout.diagnostics_.push_back(
                { ExhaustNetworkLayoutIssue::missingCylinderRoute, cylinderId });
            continue;
        }
        auto connectedNodeId = cylinderId;
        if (node->second->type == ExhaustNodeType::port) {
            const auto connections = outgoing.find(cylinderId);
            if (connections == outgoing.end() || connections->second.size() != 1) {
                layout.diagnostics_.push_back(
                    { ExhaustNetworkLayoutIssue::unresolvedEndpoint, cylinderId });
                continue;
            }
            connectedNodeId = connections->second.front();
        }
        const auto endpoint = endpointFor(connectedNodeId, false);
        if (!endpoint) {
            layout.diagnostics_.push_back(
                { ExhaustNetworkLayoutIssue::unresolvedEndpoint, connectedNodeId });
            continue;
        }
        const auto flow = graph.cylinderFlowProperties(cylinderId);
        auto areaM2 = 0.0;
        auto dischargeCoefficient = 1.0;
        auto pathIndex = node->second->pathIndex;
        if (flow.authoredNetwork) {
            areaM2 = flow.inletAreaM2;
            dischargeCoefficient = flow.inletDischargeCoefficient;
            pathIndex = flow.pathIndex;
        } else if (endpoint->type == ExhaustEndpointType::junction) {
            areaM2 = circularArea(
                layout.junctions_[endpoint->elementIndex].characteristicDiameterM);
        } else {
            areaM2 = layout.ducts_[endpoint->elementIndex].connectionAreaM2;
        }
        layout.cylinderPorts_.push_back({
            cylinderId,
            pathIndex,
            *endpoint,
            std::max(1.0e-10, areaM2),
            std::clamp(dischargeCoefficient, 0.02, 1.5),
        });
    }

    for (const auto& node : graph.nodes()) {
        if (node.type != ExhaustNodeType::outlet) continue;
        const auto endpoint = endpointFor(node.id, true);
        if (!endpoint) {
            layout.diagnostics_.push_back(
                { ExhaustNetworkLayoutIssue::missingOutlet, node.id });
            continue;
        }
        auto openingAreaM2 = circularArea(std::clamp(node.diameterMm * 0.001, 0.005, 0.5));
        if (endpoint->type != ExhaustEndpointType::junction)
            openingAreaM2 = layout.ducts_[endpoint->elementIndex].connectionAreaM2;
        layout.outlets_.push_back({
            node.id,
            node.pathIndex,
            *endpoint,
            openingAreaM2,
            std::clamp(node.dischargeCoefficient, 0.02, 1.5),
            node.acousticPositionM,
            node.acousticAxis,
            node.acousticTermination,
        });
    }

    const auto fatalDiagnostic = std::any_of(
        layout.diagnostics_.begin(), layout.diagnostics_.end(),
        [](const ExhaustNetworkLayoutDiagnostic& diagnostic) {
            return diagnostic.issue != ExhaustNetworkLayoutIssue::zeroLengthComponentResolved;
        });
    layout.valid_ = !fatalDiagnostic && !layout.ducts_.empty()
        && !layout.cylinderPorts_.empty() && !layout.outlets_.empty()
        && layout.totalCellCount_ <= discretisation.maximumTotalCells;
    return layout;
}

} // namespace enginelab::gasdynamics
