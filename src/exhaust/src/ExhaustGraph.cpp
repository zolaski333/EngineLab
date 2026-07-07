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
    const auto mergeId = std::numeric_limits<std::uint32_t>::max() - 2U;
    const auto mufflerId = std::numeric_limits<std::uint32_t>::max() - 1U;
    const auto outletId = std::numeric_limits<std::uint32_t>::max();
    const auto primaryDiameter = std::max(20.0, config.exhaust.primaryDiameterMm);
    const auto primaryRestriction = 0.04 * (config.exhaust.primaryLengthMm / 480.0)
        * std::pow(42.0 / primaryDiameter, 4.0);
    for (const auto& cylinder : config.cylinders)
        graph.nodes_.push_back({ cylinder.id, ExhaustNodeType::pipe, config.exhaust.primaryLengthMm,
                                 primaryDiameter, primaryRestriction, 0.0 });
    const auto collectorDiameter = std::max(20.0, config.exhaust.collectorDiameterMm);
    const auto outletDiameter = std::max(20.0, config.exhaust.outletDiameterMm);
    const auto collectorRestriction = 0.055 * std::pow(58.0 / collectorDiameter, 4.0);
    const auto outletRestriction = 0.01 * std::pow(65.0 / outletDiameter, 4.0);
    const auto waveSpeedMmPerSecond = 520'000.0;
    const auto primaryResonance = waveSpeedMmPerSecond / std::max(200.0, 4.0 * config.exhaust.primaryLengthMm);
    graph.nodes_.push_back({ mergeId, ExhaustNodeType::merge, 120.0, collectorDiameter, collectorRestriction, primaryResonance });
    graph.nodes_.push_back({ mufflerId, ExhaustNodeType::muffler, 450.0, collectorDiameter,
                             std::clamp(config.exhaust.mufflerRestriction, 0.0, 1.0), 82.0 });
    graph.nodes_.push_back({ outletId, ExhaustNodeType::outlet, 180.0,
                             outletDiameter, outletRestriction, 0.0 });
    for (const auto& cylinder : config.cylinders) graph.edges_.push_back({ cylinder.id, mergeId });
    graph.edges_.push_back({ mergeId, mufflerId });
    graph.edges_.push_back({ mufflerId, outletId });
    graph.effectiveRestriction_ = primaryRestriction + collectorRestriction
        + config.exhaust.mufflerRestriction + outletRestriction;
    graph.ambientPressureKpa_ = config.ambientPressureKpa;
    return graph;
}

double ExhaustGraph::backPressureKpa(const EngineState& state) const noexcept {
    const auto massFlowGramsPerSecond = state.airFlowGramsPerSecond + state.fuelFlowGramsPerSecond;
    const auto flowFactor = std::max(0.0, massFlowGramsPerSecond * 0.0070 + state.rpm / 12'000.0);
    return ambientPressureKpa_ + effectiveRestriction_ * flowFactor * flowFactor * 48.0;
}

void ExhaustGraph::process(FiringEvent& event) const noexcept {
    event.intensity *= static_cast<float>(std::clamp(1.0 - effectiveRestriction_ * 0.18, 0.45, 1.0));
    double pathLengthMm = 0.0;
    double resonanceHz = 0.0;
    auto currentId = event.exhaustPortId;
    for (std::size_t hop = 0; hop < nodes_.size(); ++hop) {
        const auto node = std::find_if(nodes_.begin(), nodes_.end(), [currentId](const ExhaustNode& item) {
            return item.id == currentId;
        });
        if (node == nodes_.end()) break;
        pathLengthMm += node->lengthMm;
        resonanceHz = std::max(resonanceHz, node->resonanceHz);
        const auto edge = std::find_if(edges_.begin(), edges_.end(), [currentId](const ExhaustEdge& item) {
            return item.from == currentId;
        });
        if (edge == edges_.end()) break;
        currentId = edge->to;
    }
    event.exhaustDelaySeconds = static_cast<float>(pathLengthMm / 520'000.0);
    event.exhaustResonanceHz = static_cast<float>(resonanceHz);
}
} // namespace enginelab
