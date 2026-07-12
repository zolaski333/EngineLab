#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <limits>

namespace enginelab {
ExhaustGraph ExhaustGraph::makeForEngine(const EngineConfig& config) {
    ExhaustGraph graph;
    // Internal IDs live at the top of the uint32 range so valid, arbitrary
    // cylinder IDs can never alias graph plumbing.
    std::vector<ExhaustPathConfig> paths = config.exhaustPaths;
    const auto topologyCoversCylinders = std::all_of(config.cylinders.begin(), config.cylinders.end(),
        [&paths](const CylinderConfig& cylinder) {
            return std::any_of(paths.begin(), paths.end(), [&cylinder](const ExhaustPathConfig& path) {
                return std::find(path.cylinderIds.begin(), path.cylinderIds.end(), cylinder.id) != path.cylinderIds.end();
            });
        });
    if (paths.empty() || !topologyCoversCylinders) {
        paths.clear();
        ExhaustPathConfig fallback;
        fallback.id = 1;
        fallback.geometry = config.exhaust;
        for (const auto& cylinder : config.cylinders) fallback.cylinderIds.push_back(cylinder.id);
        paths.push_back(std::move(fallback));
    }
    double weightedRestriction = 0.0;
    double restrictionWeight = 0.0;
    for (std::size_t pathIndex = 0; pathIndex < paths.size(); ++pathIndex) {
        const auto& path = paths[pathIndex];
        const auto legacySinglePath = paths.size() == 1 && path.cylinderIds.size() == config.cylinders.size()
            && path.impulseResponsePath.empty() && std::abs(path.audioVolume - 1.0) < 1.0e-9;
        const auto& geometry = legacySinglePath ? config.exhaust : path.geometry;
        const auto mergeId = std::numeric_limits<std::uint32_t>::max() - static_cast<std::uint32_t>(pathIndex * 3U + 2U);
        const auto mufflerId = mergeId + 1U;
        const auto outletId = mergeId + 2U;
        const auto primaryDiameter = std::max(20.0, geometry.primaryDiameterMm);
        const auto collectorDiameter = std::max(20.0, geometry.collectorDiameterMm);
        const auto outletDiameter = std::max(20.0, geometry.outletDiameterMm);
        const auto collectorRestriction = 0.055 * std::pow(58.0 / collectorDiameter, 4.0);
        const auto outletRestriction = 0.01 * std::pow(65.0 / outletDiameter, 4.0);
        double primaryRestrictionSum = 0.0;
        std::size_t connectedCylinders = 0;
        for (const auto cylinderId : path.cylinderIds) {
            const auto cylinder = std::find_if(config.cylinders.begin(), config.cylinders.end(),
                [cylinderId](const CylinderConfig& item) { return item.id == cylinderId; });
            if (cylinder == config.cylinders.end()) continue;
            const auto length = cylinder->exhaustPrimaryLengthMm > 0.0
                ? cylinder->exhaustPrimaryLengthMm : geometry.primaryLengthMm;
            const auto restriction = 0.04 * (length / 480.0) * std::pow(42.0 / primaryDiameter, 4.0);
            primaryRestrictionSum += restriction;
            ++connectedCylinders;
            graph.nodes_.push_back({ cylinder->id, ExhaustNodeType::pipe, length, primaryDiameter,
                                     restriction, 520'000.0 / std::max(200.0, 4.0 * length),
                                     path.audioVolume * cylinder->soundAttenuation,
                                     static_cast<std::uint32_t>(pathIndex) });
            graph.edges_.push_back({ cylinder->id, mergeId });
        }
        const auto meanPrimaryRestriction = primaryRestrictionSum / static_cast<double>(std::max<std::size_t>(1, connectedCylinders));
        const auto primaryResonance = 520'000.0 / std::max(200.0, 4.0 * geometry.primaryLengthMm);
        graph.nodes_.push_back({ mergeId, ExhaustNodeType::merge, 120.0, collectorDiameter, collectorRestriction, primaryResonance, path.audioVolume });
        graph.nodes_.push_back({ mufflerId, ExhaustNodeType::muffler, 450.0, collectorDiameter,
                                 std::clamp(geometry.mufflerRestriction, 0.0, 1.0), 82.0, path.audioVolume });
        graph.nodes_.push_back({ outletId, ExhaustNodeType::outlet, 180.0, outletDiameter, outletRestriction, 0.0, path.audioVolume });
        graph.edges_.push_back({ mergeId, mufflerId });
        graph.edges_.push_back({ mufflerId, outletId });
        const auto pathRestriction = meanPrimaryRestriction + collectorRestriction
            + geometry.mufflerRestriction * 0.62 + outletRestriction;
        weightedRestriction += pathRestriction * static_cast<double>(std::max<std::size_t>(1, connectedCylinders));
        restrictionWeight += static_cast<double>(std::max<std::size_t>(1, connectedCylinders));
    }
    graph.effectiveRestriction_ = weightedRestriction / std::max(1.0, restrictionWeight);
    graph.ambientPressureKpa_ = config.ambientPressureKpa;
    return graph;
}

double ExhaustGraph::backPressureKpa(const EngineState& state) const noexcept {
    const auto massFlowGramsPerSecond = std::max(state.airFlowGramsPerSecond + state.fuelFlowGramsPerSecond,
                                                 state.exhaustFlowGramsPerSecond);
    const auto flowFactor = std::max(0.0, massFlowGramsPerSecond * 0.0042 + state.rpm / 18'000.0);
    const auto runnerPulse = std::max(0.0, state.exhaustRunnerPressureKpa - ambientPressureKpa_) * 0.045;
    return ambientPressureKpa_ + effectiveRestriction_ * flowFactor * flowFactor * 48.0 + runnerPulse;
}

void ExhaustGraph::process(FiringEvent& event) const noexcept {
    event.intensity *= static_cast<float>(std::clamp(1.0 - effectiveRestriction_ * 0.18, 0.45, 1.0));
    double pathLengthMm = 0.0;
    double resonanceHz = 0.0;
    auto currentId = event.exhaustPortId;
    double audioGain = 1.0;
    for (std::size_t hop = 0; hop < nodes_.size(); ++hop) {
        const auto node = std::find_if(nodes_.begin(), nodes_.end(), [currentId](const ExhaustNode& item) {
            return item.id == currentId;
        });
        if (node == nodes_.end()) break;
        if (hop == 0) event.exhaustPathIndex = node->pathIndex;
        pathLengthMm += node->lengthMm;
        if (hop == 0) audioGain = node->audioGain;
        resonanceHz = std::max(resonanceHz, node->resonanceHz);
        const auto edge = std::find_if(edges_.begin(), edges_.end(), [currentId](const ExhaustEdge& item) {
            return item.from == currentId;
        });
        if (edge == edges_.end()) break;
        currentId = edge->to;
    }
    event.exhaustDelaySeconds = static_cast<float>(pathLengthMm / 520'000.0);
    event.exhaustResonanceHz = static_cast<float>(resonanceHz);
    event.intensity *= static_cast<float>(std::clamp(audioGain, 0.0, 8.0));
}
} // namespace enginelab
