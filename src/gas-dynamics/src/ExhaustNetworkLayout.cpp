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
        const auto outletDiameterM = node.outletDiameterMm > 0.0
            ? std::clamp(node.outletDiameterMm * 0.001, 0.005, 0.5)
            : diameterM;
        const auto inletConnectionAreaM2 = circularArea(diameterM);
        const auto outletConnectionAreaM2 = circularArea(outletDiameterM);
        const auto connectionAreaM2 = (inletConnectionAreaM2
            + std::sqrt(inletConnectionAreaM2 * outletConnectionAreaM2)
            + outletConnectionAreaM2) / 3.0;
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
                finite(node.lengthMm) && node.lengthMm > 0.0
                    ? node.lengthMm * 0.001 : 0.0,
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
            // `minimumResolvedLengthM` is a numerical guard against a degenerate
            // zero, NOT a physical scale, and it must not become the length of a
            // real element: the explicit time step is the minimum over cells of
            // dx/(c+|u|), so one 5 mm cell makes EVERY duct in the network
            // substep at its rate.
            //
            // A 1-D element carries resolvable plane-wave physics only down to
            // L ~= 0.853 d, where its half-wave c/(2L) meets its own plane-mode
            // cutoff f_c = 1.8412 c/(2 pi a) -- the same limit, and the same
            // constant, that floors a short silencer body in ExhaustGraph.cpp.
            // Below it the length carries nothing the network can use, so
            // flooring a DERIVED length there is not an approximation.
            //
            // This is the difference between a usable exhaust designer and an
            // unusable one. `defaultComponent` gives an outlet and a splitter no
            // length and no volume, and every network has an outlet, so before
            // this floor EVERY user-authored network fell to 5 mm: measured on
            // the 2JZ and the LS3, 180 mm shipped against 5 mm authored, i.e.
            // x36 solver cost for a geometry the user did not choose.
            constexpr double planeWaveResonantLengthRatio = 0.853;
            lengthM = requestedVolumeM3 > 0.0
                ? requestedVolumeM3 / connectionAreaM2
                : planeWaveResonantLengthRatio * diameterM;
            lengthM = std::max({ lengthM, discretisation.minimumResolvedLengthM,
                                 planeWaveResonantLengthRatio * diameterM });
            lengthWasDerived = true;
            layout.diagnostics_.push_back(
                { ExhaustNetworkLayoutIssue::zeroLengthComponentResolved, node.id });
        }
        auto inletFlowAreaM2 = inletConnectionAreaM2;
        auto outletFlowAreaM2 = outletConnectionAreaM2;
        auto flowAreaM2 = (inletFlowAreaM2
            + std::sqrt(inletFlowAreaM2 * outletFlowAreaM2)
            + outletFlowAreaM2) / 3.0;
        auto areaWasDerivedFromVolume = false;
        if (requestedVolumeM3 > connectionAreaM2 * lengthM * (1.0 + 1.0e-9)) {
            flowAreaM2 = requestedVolumeM3 / lengthM;
            // An explicit chamber volume describes a large internal control
            // section behind its two real connection apertures, not a smooth
            // taper between the apertures.
            inletFlowAreaM2 = flowAreaM2;
            outletFlowAreaM2 = flowAreaM2;
            areaWasDerivedFromVolume = true;
        }
        const auto volumeM3 = flowAreaM2 * lengthM;
        const auto hydraulicDiameterM =
            2.0 * std::sqrt(flowAreaM2 / std::numbers::pi);
        const auto index = layout.ducts_.size();
        layout.ducts_.push_back({
            node.id,
            node.sourceComponentId,
            node.pathIndex,
            node.type,
            lengthM,
            flowAreaM2,
            inletFlowAreaM2,
            outletFlowAreaM2,
            connectionAreaM2,
            inletConnectionAreaM2,
            outletConnectionAreaM2,
            hydraulicDiameterM,
            volumeM3,
            std::max(0.0, node.localLossCoefficient),
            std::clamp(node.dischargeCoefficient, 0.02, 1.5),
            0,
            lengthWasDerived,
            areaWasDerivedFromVolume,
            std::max(0.0, node.packingFlowResistivityPaSPerM2),
            std::max(0.0, node.packingThicknessMm) * 0.001,
            std::clamp(node.perforatedOpenAreaRatio, 0.0, 1.0),
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
            const auto& duct = layout.ducts_[endpoint->elementIndex];
            areaM2 = endpoint->type == ExhaustEndpointType::ductInlet
                ? duct.inletConnectionAreaM2
                : duct.outletConnectionAreaM2;
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
        const auto openingDiameterMm = node.outletDiameterMm > 0.0
            ? node.outletDiameterMm : node.diameterMm;
        auto openingAreaM2 = circularArea(
            std::clamp(openingDiameterMm * 0.001, 0.005, 0.5));
        if (endpoint->type != ExhaustEndpointType::junction) {
            const auto& duct = layout.ducts_[endpoint->elementIndex];
            openingAreaM2 = endpoint->type == ExhaustEndpointType::ductInlet
                ? duct.inletConnectionAreaM2
                : duct.outletConnectionAreaM2;
        }
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

ExhaustNetworkLayout ExhaustNetworkLayout::assemble(
    std::vector<CompiledExhaustDuct> ducts,
    std::vector<CompiledExhaustJunction> junctions,
    std::vector<CompiledExhaustInterface> interfaces,
    std::vector<CompiledCylinderPort> cylinderPorts,
    std::vector<CompiledExhaustOutlet> outlets,
    ExhaustNetworkDiscretisation discretisation) {
    ExhaustNetworkLayout layout;
    layout.discretisation_ = discretisation;
    if (!discretisation.valid()) {
        layout.diagnostics_.push_back(
            { ExhaustNetworkLayoutIssue::invalidDiscretisation, 0 });
        return layout;
    }
    layout.ducts_ = std::move(ducts);
    layout.junctions_ = std::move(junctions);
    layout.interfaces_ = std::move(interfaces);
    layout.cylinderPorts_ = std::move(cylinderPorts);
    layout.outlets_ = std::move(outlets);

    const auto finitePositive = [](double value) {
        return std::isfinite(value) && value > 0.0;
    };
    for (auto& duct : layout.ducts_) {
        // Preserve programmatically assembled legacy layouts. Prior callers
        // populated only the mean flow/connection areas.
        if (!(duct.inletFlowAreaM2 > 0.0))
            duct.inletFlowAreaM2 = duct.flowAreaM2;
        if (!(duct.outletFlowAreaM2 > 0.0))
            duct.outletFlowAreaM2 = duct.flowAreaM2;
        if (!(duct.inletConnectionAreaM2 > 0.0))
            duct.inletConnectionAreaM2 = duct.connectionAreaM2;
        if (!(duct.outletConnectionAreaM2 > 0.0))
            duct.outletConnectionAreaM2 = duct.connectionAreaM2;
        if (!finitePositive(duct.lengthM) || !finitePositive(duct.flowAreaM2)
            || !finitePositive(duct.inletFlowAreaM2)
            || !finitePositive(duct.outletFlowAreaM2)
            || !finitePositive(duct.connectionAreaM2)
            || !finitePositive(duct.inletConnectionAreaM2)
            || !finitePositive(duct.outletConnectionAreaM2)
            || !finitePositive(duct.hydraulicDiameterM)
            || !(duct.lossCoefficient >= 0.0) || !std::isfinite(duct.lossCoefficient)
            || duct.cellCount < 1
            || duct.cellCount > discretisation.maximumCellsPerDuct) {
            layout.diagnostics_.push_back(
                { ExhaustNetworkLayoutIssue::unresolvedEndpoint, duct.nodeId });
        }
        layout.totalCellCount_ += duct.cellCount;
    }
    for (const auto& junction : layout.junctions_) {
        if (!finitePositive(junction.volumeM3)
            || !finitePositive(junction.characteristicDiameterM)) {
            layout.diagnostics_.push_back(
                { ExhaustNetworkLayoutIssue::unresolvedEndpoint, junction.nodeId });
        }
    }
    if (layout.totalCellCount_ > discretisation.maximumTotalCells) {
        layout.diagnostics_.push_back(
            { ExhaustNetworkLayoutIssue::cellBudgetExceeded, 0 });
    }

    // Exact face coverage: every duct face is claimed exactly once, and every
    // element sits at the face orientation whose flux sign `evaluateStage`
    // actually applies (see the header comment).
    std::vector<std::uint8_t> inletClaims(layout.ducts_.size(), 0);
    std::vector<std::uint8_t> outletClaims(layout.ducts_.size(), 0);
    const auto endpointInRange = [&layout](const ExhaustEndpoint& endpoint) {
        return endpoint.type == ExhaustEndpointType::junction
            ? endpoint.elementIndex < layout.junctions_.size()
            : endpoint.elementIndex < layout.ducts_.size();
    };
    const auto claim = [&](const ExhaustEndpoint& endpoint) {
        if (endpoint.type == ExhaustEndpointType::junction) return true;
        auto& claims = endpoint.type == ExhaustEndpointType::ductInlet
            ? inletClaims[endpoint.elementIndex]
            : outletClaims[endpoint.elementIndex];
        ++claims;
        return claims == 1;
    };
    const auto reject = [&layout](std::uint32_t nodeId) {
        layout.diagnostics_.push_back(
            { ExhaustNetworkLayoutIssue::unresolvedEndpoint, nodeId });
    };
    for (const auto& connection : layout.interfaces_) {
        if (!endpointInRange(connection.upstream)
            || !endpointInRange(connection.downstream)
            || connection.upstream.type == ExhaustEndpointType::ductInlet
            || connection.downstream.type == ExhaustEndpointType::ductOutlet
            || !claim(connection.upstream) || !claim(connection.downstream))
            reject(connection.upstream.nodeId);
    }
    for (const auto& port : layout.cylinderPorts_) {
        if (!endpointInRange(port.networkEndpoint)
            || port.networkEndpoint.type == ExhaustEndpointType::ductOutlet
            || !finitePositive(port.runnerConnectionAreaM2)
            || !(port.dischargeCoefficient > 0.0)
            || !claim(port.networkEndpoint))
            reject(port.cylinderId);
    }
    for (const auto& outlet : layout.outlets_) {
        if (!endpointInRange(outlet.networkEndpoint)
            || outlet.networkEndpoint.type == ExhaustEndpointType::ductInlet
            || !finitePositive(outlet.openingAreaM2)
            || !(outlet.dischargeCoefficient > 0.0)
            || !claim(outlet.networkEndpoint))
            reject(outlet.outletNodeId);
    }
    for (std::size_t index = 0; index < layout.ducts_.size(); ++index) {
        if (inletClaims[index] != 1 || outletClaims[index] != 1)
            reject(layout.ducts_[index].nodeId);
    }

    layout.valid_ = layout.diagnostics_.empty() && !layout.ducts_.empty()
        && !layout.cylinderPorts_.empty() && !layout.outlets_.empty();
    return layout;
}

double ExhaustNetworkLayout::minimumCellLengthM() const noexcept {
    auto shortest = 0.0;
    for (const auto& duct : ducts_) {
        if (duct.cellCount == 0 || !(duct.lengthM > 0.0)) continue;
        const auto cellLengthM =
            duct.lengthM / static_cast<double>(duct.cellCount);
        if (shortest == 0.0 || cellLengthM < shortest) shortest = cellLengthM;
    }
    return shortest;
}

} // namespace enginelab::gasdynamics
