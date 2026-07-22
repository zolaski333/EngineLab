#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/foundation/ExhaustGasAcoustics.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <numbers>
#include <numeric>
#include <unordered_map>
#include <unordered_set>

namespace enginelab {
namespace {
constexpr std::size_t maximumCompiledRoutes = 4'096;
constexpr std::size_t maximumTraversalDepth = 512;
// A static topology needs a representative gas temperature. Retain the old
// 520 m/s behaviour near standard ambient conditions while making the model
// temperature-aware and allowing callers to supply a measured design value.
constexpr double nominalExhaustTemperatureRiseC = 405.0;
constexpr double maximumCompiledAmplitudeGain = 8.0;
constexpr double modeMergeRatio = 1.03;

[[nodiscard]] double finiteClamped(double value, double minimum, double maximum,
                                   double fallback) noexcept {
    return std::isfinite(value) ? std::clamp(value, minimum, maximum) : fallback;
}

[[nodiscard]] double exhaustWaveSpeedMmPerSecond(double temperatureC) noexcept {
    // Shared exhaust-gas acoustics: identical wave speed to the realtime audio
    // delay lines, expressed here in mm/s for the compiler's length units.
    return exhaustSpeedOfSoundMps(temperatureC) * 1'000.0;
}

struct ModeSet final {
    std::size_t count { 0 };
    std::array<ExhaustAcousticMode, maximumExhaustAcousticModes> values {};

    void add(double frequencyHz, double energy) noexcept {
        if (!std::isfinite(frequencyHz) || frequencyHz <= 0.0 || frequencyHz > 20'000.0
            || !std::isfinite(energy) || energy <= 0.0) return;

        for (std::size_t index = 0; index < count; ++index) {
            const auto lower = values[index].frequencyHz / modeMergeRatio;
            const auto upper = values[index].frequencyHz * modeMergeRatio;
            if (frequencyHz < lower || frequencyHz > upper) continue;
            const auto combinedEnergy = values[index].relativeEnergy + energy;
            values[index].frequencyHz =
                (values[index].frequencyHz * values[index].relativeEnergy + frequencyHz * energy)
                / combinedEnergy;
            values[index].relativeEnergy = combinedEnergy;
            return;
        }

        if (count < values.size()) {
            values[count++] = { frequencyHz, energy };
            return;
        }

        const auto weakest = std::min_element(values.begin(), values.end(),
            [](const auto& left, const auto& right) {
                return left.relativeEnergy < right.relativeEnergy;
            });
        if (weakest != values.end() && energy > weakest->relativeEnergy)
            *weakest = { frequencyHz, energy };
    }

    void normaliseAndSort() noexcept {
        const auto energy = std::accumulate(values.begin(), values.begin() + count, 0.0,
            [](double sum, const ExhaustAcousticMode& mode) {
                return sum + mode.relativeEnergy;
            });
        if (energy > 0.0 && std::isfinite(energy)) {
            for (std::size_t index = 0; index < count; ++index)
                values[index].relativeEnergy /= energy;
        }
        std::sort(values.begin(), values.begin() + count,
            [](const auto& left, const auto& right) {
                if (left.relativeEnergy != right.relativeEnergy)
                    return left.relativeEnergy > right.relativeEnergy;
                return left.frequencyHz < right.frequencyHz;
            });
    }
};

struct ComponentResonance final {
    double frequencyHz { 0.0 };
    double strength { 0.0 };
};

[[nodiscard]] ExhaustNodeType nodeType(ExhaustComponentType type) noexcept {
    switch (type) {
    case ExhaustComponentType::pipe: return ExhaustNodeType::pipe;
    case ExhaustComponentType::merge: return ExhaustNodeType::merge;
    case ExhaustComponentType::splitter: return ExhaustNodeType::splitter;
    case ExhaustComponentType::resonator: return ExhaustNodeType::resonator;
    case ExhaustComponentType::muffler: return ExhaustNodeType::muffler;
    case ExhaustComponentType::catalyst: return ExhaustNodeType::catalyst;
    case ExhaustComponentType::outlet: return ExhaustNodeType::outlet;
    }
    return ExhaustNodeType::pipe;
}

[[nodiscard]] double geometricRestriction(const ExhaustComponentConfig& component) noexcept {
    const auto diameterMm = finiteClamped(component.diameterMm, 5.0, 500.0, 42.0);
    const auto diameterRatio42 = 42.0 / diameterMm;
    const auto diameterRatio58 = 58.0 / diameterMm;
    const auto lengthRatio = finiteClamped(component.lengthMm, 0.0, 10'000.0, 0.0) / 480.0;
    double loss = 0.0;
    switch (component.type) {
    case ExhaustComponentType::pipe:
        loss = 0.04 * lengthRatio * std::pow(diameterRatio42, 4.0);
        break;
    case ExhaustComponentType::merge:
        loss = 0.055 * std::pow(diameterRatio58, 4.0);
        break;
    case ExhaustComponentType::splitter:
        loss = 0.065 * std::pow(diameterRatio58, 4.0);
        break;
    case ExhaustComponentType::resonator:
        loss = 0.015 * lengthRatio * std::pow(diameterRatio42, 4.0);
        break;
    case ExhaustComponentType::muffler:
        loss = 0.025 * std::max(0.25, lengthRatio) * std::pow(diameterRatio58, 4.0);
        break;
    case ExhaustComponentType::catalyst:
        loss = 0.10 * std::pow(diameterRatio42, 4.0);
        break;
    case ExhaustComponentType::outlet:
        // The discharge coefficient is a flow-contraction term. It is already
        // applied once as the outlet's effective area (A*Cd) when the authored
        // network's outlet conductance is summed, so it must not reappear here
        // as a 1/Cd^2 pressure-loss factor: that scaled the same effective area
        // by sqrt(1+K) a second time and understated outlet flow. Only the
        // diameter-derived geometric loss belongs to the restriction.
        loss = 0.01 * std::pow(65.0 / diameterMm, 4.0);
        break;
    }
    // A non-finite authored restriction must not silently choke the path: the
    // maximum is the worst possible guess, and it mutes the exhaust without a
    // single symptom. Fall back to no additional restriction and let the
    // geometric loss above stand alone.
    return loss + finiteClamped(component.restriction, 0.0, 100.0, 0.0);
}

[[nodiscard]] ComponentResonance componentResonance(
    const ExhaustComponentConfig& component, double waveSpeedMmPerSecond) noexcept {
    if (std::isfinite(component.resonanceHz) && component.resonanceHz > 0.0)
        return { std::min(20'000.0, component.resonanceHz), 1.5 };
    if (component.type == ExhaustComponentType::resonator && component.volumeLitres > 0.0
        && component.lengthMm > 0.0 && std::isfinite(component.volumeLitres)
        && std::isfinite(component.lengthMm)) {
        const auto diameterM = finiteClamped(component.diameterMm, 5.0, 500.0, 42.0) * 0.001;
        const auto neckAreaM2 = std::numbers::pi * diameterM * diameterM * 0.25;
        const auto volumeM3 = component.volumeLitres * 0.001;
        const auto neckLengthM = finiteClamped(component.lengthMm, 1.0, 10'000.0, 300.0)
            * 0.001 + 0.85 * diameterM;
        return { waveSpeedMmPerSecond * 0.001 / (2.0 * std::numbers::pi)
                * std::sqrt(neckAreaM2 / std::max(1.0e-12, volumeM3 * neckLengthM)),
            1.2 };
    }
    if ((component.type == ExhaustComponentType::resonator
            || component.type == ExhaustComponentType::muffler)
        && component.lengthMm > 0.0 && std::isfinite(component.lengthMm))
        return { waveSpeedMmPerSecond
                / (4.0 * finiteClamped(component.lengthMm, 1.0, 10'000.0, 300.0)),
            component.type == ExhaustComponentType::resonator ? 0.9 : 0.55 };
    // Plain pipe quarter-wave modes are compiled from the complete route.
    // Treating every short segment as an independent resonator biases the
    // result toward the highest (and usually least relevant) frequency.
    return {};
}

[[nodiscard]] double componentAreaM2(const ExhaustComponentConfig& component) noexcept {
    const auto radiusM = finiteClamped(component.diameterMm, 5.0, 500.0, 42.0) * 0.0005;
    return std::numbers::pi * radiusM * radiusM;
}

[[nodiscard]] double componentVolumeLitres(const ExhaustComponentConfig& component) noexcept {
    if (std::isfinite(component.volumeLitres) && component.volumeLitres > 0.0)
        return std::clamp(component.volumeLitres, 0.001, 1'000.0);
    const auto areaMm2 = std::numbers::pi
        * std::pow(finiteClamped(component.diameterMm, 5.0, 500.0, 42.0) * 0.5, 2.0);
    return areaMm2 * finiteClamped(component.lengthMm, 0.0, 10'000.0, 0.0) / 1.0e6;
}

[[nodiscard]] const CylinderConfig* findCylinder(const EngineConfig& config,
                                                  std::uint32_t cylinderId) noexcept {
    const auto cylinder = std::find_if(config.cylinders.begin(), config.cylinders.end(),
        [cylinderId](const CylinderConfig& item) { return item.id == cylinderId; });
    return cylinder == config.cylinders.end() ? nullptr : &*cylinder;
}

[[nodiscard]] double amplitudeFromLog(double logAmplitude) noexcept {
    if (logAmplitude == -std::numeric_limits<double>::infinity()) return 0.0;
    // A deliberate mute is the -inf above. Anything else non-finite is a NaN
    // that reached us through an authored gain: silence is the only safe
    // reading. Returning the maximum gain instead made a corrupt field the
    // loudest route in the graph.
    if (!std::isfinite(logAmplitude)) return 0.0;
    const auto maximumLog = std::log(maximumCompiledAmplitudeGain);
    if (logAmplitude >= maximumLog) return maximumCompiledAmplitudeGain;
    if (logAmplitude <= std::log(std::numeric_limits<double>::min())) return 0.0;
    return std::exp(logAmplitude);
}

[[nodiscard]] double restrictionTransmission(double restriction) noexcept {
    // K relates dynamic pressure to dissipated pressure. Mapping it to an
    // amplitude with 1/sqrt(1+K) is monotonic, energy-safe and avoids the old
    // arbitrary non-zero attenuation floor.
    return 1.0 / std::sqrt(1.0 + std::clamp(restriction, 0.0, 100.0));
}
} // namespace

ExhaustGraph ExhaustGraph::makeForEngine(const EngineConfig& config) {
    const auto ambientC = finiteClamped(config.ambientTemperatureC, -50.0, 60.0, 22.0);
    return makeForEngine(config, ambientC + nominalExhaustTemperatureRiseC);
}

ExhaustGraph ExhaustGraph::makeForEngine(
    const EngineConfig& config, double referenceExhaustTemperatureC) {
    ExhaustGraph graph;
    graph.acousticObserver_ = config.acousticObserver;
    const auto fallbackTemperatureC = finiteClamped(
        config.ambientTemperatureC, -50.0, 60.0, 22.0) + nominalExhaustTemperatureRiseC;
    graph.waveSpeedMmPerSecond_ = exhaustWaveSpeedMmPerSecond(finiteClamped(
        referenceExhaustTemperatureC, -50.0, 2'226.85, fallbackTemperatureC));
    std::vector<ExhaustPathConfig> paths = config.exhaustPaths;
    const auto occurrencesOf = [&paths](std::uint32_t cylinderId) {
        std::size_t occurrences = 0;
        for (const auto& path : paths)
            occurrences += static_cast<std::size_t>(std::count(
                path.cylinderIds.begin(), path.cylinderIds.end(), cylinderId));
        return occurrences;
    };
    const auto topologyCoversCylinders = std::all_of(config.cylinders.begin(), config.cylinders.end(),
        [&occurrencesOf](const CylinderConfig& cylinder) {
            return occurrencesOf(cylinder.id) == 1;
        });
    if (paths.empty() || !topologyCoversCylinders) {
        // Replacing the authored topology is a real loss of user intent, so name
        // every cylinder responsible rather than substituting a default in silence.
        if (!topologyCoversCylinders)
            for (const auto& cylinder : config.cylinders)
                if (occurrencesOf(cylinder.id) != 1)
                    graph.diagnostics_.push_back(
                        { ExhaustCompileIssue::topologyRejected, cylinder.id });
        paths.clear();
        ExhaustPathConfig fallback;
        fallback.id = 1;
        fallback.geometry = config.exhaust;
        for (const auto& cylinder : config.cylinders) fallback.cylinderIds.push_back(cylinder.id);
        paths.push_back(std::move(fallback));
    }
    graph.pathFlowProperties_.resize(paths.size());

    struct Root final {
        std::uint32_t nodeId;
        std::uint32_t cylinderId;
        std::uint32_t pathIndex;
    };
    std::vector<Root> roots;
    std::unordered_set<std::uint32_t> usedNodeIds;
    for (const auto& cylinder : config.cylinders) usedNodeIds.insert(cylinder.id);
    auto nextGeneratedId = std::numeric_limits<std::uint32_t>::max();
    auto nodeIdSpaceExhausted = false;
    const auto allocateNodeId = [&usedNodeIds, &nextGeneratedId, &nodeIdSpaceExhausted, &graph]() {
        while (nextGeneratedId != 0 && usedNodeIds.contains(nextGeneratedId)) --nextGeneratedId;
        // Reaching 0 used to hand out the same ID forever, silently fusing
        // unrelated nodes into one. Report it once instead of corrupting the graph.
        if (nextGeneratedId == 0 && !nodeIdSpaceExhausted) {
            nodeIdSpaceExhausted = true;
            graph.diagnostics_.push_back({ ExhaustCompileIssue::nodeIdSpaceExhausted, 0 });
        }
        const auto result = nextGeneratedId;
        usedNodeIds.insert(result);
        if (nextGeneratedId != 0) --nextGeneratedId;
        return result;
    };

    for (std::size_t pathIndex = 0; pathIndex < paths.size(); ++pathIndex) {
        const auto& path = paths[pathIndex];
        const auto runtimePathIndex = static_cast<std::uint32_t>(pathIndex);
        if (path.network && !path.network->components.empty()) {
            auto& pathFlow = graph.pathFlowProperties_[pathIndex];
            pathFlow.authoredNetwork = true;
            double outletConductanceM2 = 0.0;
            std::unordered_set<std::uint32_t> summarisedComponentIds;
            for (const auto& component : path.network->components) {
                if (component.id == 0 || !summarisedComponentIds.insert(component.id).second)
                    continue;
                pathFlow.collectorVolumeLitres += componentVolumeLitres(component);
                if (component.type == ExhaustComponentType::outlet) {
                    outletConductanceM2 += componentAreaM2(component)
                        * finiteClamped(component.dischargeCoefficient, 0.02, 1.5, 0.72);
                }
            }
            if (outletConductanceM2 <= 0.0) {
                const auto outletRadiusM = finiteClamped(path.geometry.outletDiameterMm,
                    5.0, 500.0, 65.0) * 0.0005;
                outletConductanceM2 = std::numbers::pi * outletRadiusM * outletRadiusM
                    * finiteClamped(path.geometry.outletDischargeCoefficient, 0.02, 1.5, 0.72);
            }
            pathFlow.collectorVolumeLitres = std::max(0.01, pathFlow.collectorVolumeLitres);
            pathFlow.effectiveOutletAreaM2 = outletConductanceM2;
            pathFlow.outletDischargeCoefficient = 1.0;
            std::unordered_map<std::uint32_t, std::uint32_t> componentNodeIds;
            componentNodeIds.reserve(path.network->components.size());
            for (const auto& component : path.network->components) {
                if (component.id == 0 || componentNodeIds.contains(component.id)) continue;
                const auto runtimeId = allocateNodeId();
                componentNodeIds.emplace(component.id, runtimeId);
                const auto resonance = componentResonance(
                    component, graph.waveSpeedMmPerSecond_);
                graph.nodes_.push_back({ runtimeId, nodeType(component.type),
                    finiteClamped(component.lengthMm, 0.0, 10'000.0, 0.0),
                    finiteClamped(component.diameterMm, 5.0, 500.0, 42.0),
                    geometricRestriction(component), resonance.frequencyHz,
                    finiteClamped(component.acousticGain, 0.0, 8.0, 0.0),
                    runtimePathIndex, component.id, resonance.strength,
                    componentVolumeLitres(component),
                    finiteClamped(component.dischargeCoefficient, 0.02, 1.5, 0.72),
                    finiteClamped(component.restriction, 0.0, 100.0, 0.0),
                    component.acousticPositionM, component.acousticAxis,
                    component.acousticTermination });
            }
            std::unordered_set<std::uint64_t> compiledConnections;
            for (const auto& connection : path.network->connections) {
                const auto from = componentNodeIds.find(connection.fromComponentId);
                const auto to = componentNodeIds.find(connection.toComponentId);
                const auto key = (static_cast<std::uint64_t>(connection.fromComponentId) << 32U)
                    | connection.toComponentId;
                if (from != componentNodeIds.end() && to != componentNodeIds.end()
                    && from != to && compiledConnections.insert(key).second)
                    graph.edges_.push_back({ from->second, to->second });
            }
            std::unordered_set<std::uint32_t> compiledCylinderConnections;
            for (const auto& connection : path.network->cylinderConnections) {
                const auto component = componentNodeIds.find(connection.componentId);
                const auto cylinder = findCylinder(config, connection.cylinderId);
                if (component == componentNodeIds.end() || cylinder == nullptr
                    || !compiledCylinderConnections.insert(connection.cylinderId).second) continue;
                graph.nodes_.push_back({ connection.cylinderId, ExhaustNodeType::port, 0.0,
                    finiteClamped(path.geometry.primaryDiameterMm, 5.0, 500.0, 42.0), 0.0, 0.0,
                    finiteClamped(path.audioVolume, 0.0, 8.0, 0.0)
                        * finiteClamped(cylinder->soundAttenuation, 0.0, 4.0, 0.0),
                    runtimePathIndex, 0 });
                graph.edges_.push_back({ connection.cylinderId, component->second });
                roots.push_back({ connection.cylinderId, connection.cylinderId, runtimePathIndex });
                const auto sourceComponent = std::find_if(path.network->components.begin(),
                    path.network->components.end(), [&connection](const auto& item) {
                        return item.id == connection.componentId;
                    });
                if (sourceComponent != path.network->components.end()) {
                    graph.cylinderFlowProperties_.push_back({ true, connection.cylinderId,
                        runtimePathIndex,
                        componentAreaM2(*sourceComponent),
                        finiteClamped(sourceComponent->dischargeCoefficient, 0.02, 1.5, 0.72),
                        std::max(0.001, componentVolumeLitres(*sourceComponent)),
                        finiteClamped(sourceComponent->lengthMm, 0.0, 10'000.0, 0.0) });
                }
            }
            continue;
        }

        // Backward-compatible compiler for schema-v1 geometry-only paths.
        const auto legacySinglePath = paths.size() == 1 && path.cylinderIds.size() == config.cylinders.size()
            && path.impulseResponsePath.empty() && std::abs(path.audioVolume - 1.0) < 1.0e-9;
        const auto& geometry = legacySinglePath ? config.exhaust : path.geometry;
        const auto mergeId = allocateNodeId();
        const auto mufflerId = allocateNodeId();
        const auto outletId = allocateNodeId();
        const auto primaryDiameter = finiteClamped(geometry.primaryDiameterMm, 20.0, 500.0, 42.0);
        const auto collectorDiameter = finiteClamped(geometry.collectorDiameterMm, 20.0, 500.0, 58.0);
        const auto outletDiameter = finiteClamped(geometry.outletDiameterMm, 20.0, 500.0, 65.0);
        const auto collectorRestriction = 0.055 * std::pow(58.0 / collectorDiameter, 4.0);
        const auto collectorVolumeLitres = finiteClamped(
            geometry.collectorVolumeLitres, 0.01, 1'000.0, 2.5);
        const auto outletDischargeCoefficient = finiteClamped(
            geometry.outletDischargeCoefficient, 0.02, 1.5, 0.72);
        const auto outletRestriction = 0.01 * std::pow(65.0 / outletDiameter, 4.0)
            / std::pow(outletDischargeCoefficient, 2.0);
        for (const auto cylinderId : path.cylinderIds) {
            const auto cylinder = findCylinder(config, cylinderId);
            if (cylinder == nullptr) continue;
            const auto configuredLength = cylinder->exhaustPrimaryLengthMm > 0.0
                ? cylinder->exhaustPrimaryLengthMm : geometry.primaryLengthMm;
            const auto length = finiteClamped(configuredLength, 1.0, 10'000.0, 480.0);
            const auto restriction = 0.04 * (length / 480.0) * std::pow(42.0 / primaryDiameter, 4.0);
            graph.nodes_.push_back({ cylinder->id, ExhaustNodeType::pipe, length, primaryDiameter,
                restriction, 0.0,
                finiteClamped(path.audioVolume, 0.0, 8.0, 0.0)
                    * finiteClamped(cylinder->soundAttenuation, 0.0, 4.0, 0.0),
                runtimePathIndex, 0, 0.0,
                std::numbers::pi * std::pow(primaryDiameter * 0.0005, 2.0)
                    * (length * 0.001) * 1'000.0,
                1.0, 0.0 });
            graph.edges_.push_back({ cylinder->id, mergeId });
            roots.push_back({ cylinder->id, cylinder->id, runtimePathIndex });
        }
        graph.nodes_.push_back({ mergeId, ExhaustNodeType::merge, 120.0, collectorDiameter,
            collectorRestriction, 0.0, 1.0, runtimePathIndex, 0, 0.0,
            collectorVolumeLitres, 1.0, 0.0 });
        const auto chamberConfigured = geometry.mufflerChamberDiameterMm > 1.0
            && geometry.mufflerChamberLengthMm > 1.0;
        const auto mufflerLengthMm = chamberConfigured
            ? finiteClamped(geometry.mufflerChamberLengthMm, 1.0, 10'000.0, 450.0)
            : 450.0;
        const auto mufflerFlowDiameterMm = chamberConfigured
            ? finiteClamped(geometry.mufflerChamberDiameterMm,
                collectorDiameter, 1'000.0, collectorDiameter)
            : collectorDiameter;
        const auto mufflerVolumeLitres = std::numbers::pi
            * std::pow(mufflerFlowDiameterMm * 0.0005, 2.0)
            * (mufflerLengthMm * 0.001) * 1'000.0;
        // Keep the connection diameter at the collector throat and publish the
        // larger body through volume. ExhaustNetworkLayout then derives the
        // chamber's internal flow area while retaining the two real area steps.
        graph.nodes_.push_back({ mufflerId, ExhaustNodeType::muffler,
            mufflerLengthMm, collectorDiameter,
            finiteClamped(geometry.mufflerRestriction, 0.0, 1.0, 1.0) * 0.62,
            82.0, 1.0, runtimePathIndex, 0, 0.55,
            mufflerVolumeLitres,
            1.0, finiteClamped(geometry.mufflerRestriction, 0.0, 1.0, 1.0) });
        graph.nodes_.push_back({ outletId, ExhaustNodeType::outlet, 180.0, outletDiameter,
            outletRestriction, 0.0, 1.0, runtimePathIndex, 0, 0.0,
            std::numbers::pi * std::pow(outletDiameter * 0.0005, 2.0)
                * 0.180 * 1'000.0,
            outletDischargeCoefficient, 0.0 });
        graph.nodes_.back().acousticPositionM = path.acousticPositionM;
        graph.nodes_.back().acousticAxis = path.acousticAxis;
        graph.nodes_.back().acousticTermination = path.acousticTermination;
        graph.edges_.push_back({ mergeId, mufflerId });
        graph.edges_.push_back({ mufflerId, outletId });
    }

    // Indices, not pointers: graph.nodes_ is a vector, and a pointer map would
    // turn any later push_back into silent use-after-free.
    std::unordered_map<std::uint32_t, std::size_t> nodesById;
    std::unordered_map<std::uint32_t, std::vector<std::uint32_t>> outgoing;
    nodesById.reserve(graph.nodes_.size());
    outgoing.reserve(graph.nodes_.size());
    for (std::size_t index = 0; index < graph.nodes_.size(); ++index) {
        nodesById.emplace(graph.nodes_[index].id, index);
        outgoing.try_emplace(graph.nodes_[index].id);
    }
    for (const auto& edge : graph.edges_) outgoing[edge.from].push_back(edge.to);

    // For pressure loss K, common segments are in series and only downstream
    // branches are parallel: K_eq = K_node + 1/(sum(1/sqrt(K_branch)))^2.
    // The active set and depth bound also make this compiler safe when it is
    // handed an invalid graph before application-level validation runs.
    // A result reached through a cycle or a depth cut-off depends on which root
    // the traversal started from, so it must not be cached: a node found invalid
    // because of a back edge on one branch would stay invalid for every other.
    struct RestrictionResult final { double value { 0.0 }; bool pathDependent { false }; };
    std::unordered_map<std::uint32_t, double> equivalentMemo;
    std::unordered_set<std::uint32_t> equivalentActive;
    const auto invalidRestriction = std::numeric_limits<double>::infinity();
    std::function<RestrictionResult(std::uint32_t, std::size_t)> equivalentRestriction =
        [&](std::uint32_t nodeId, std::size_t depth) -> RestrictionResult {
            if (depth > maximumTraversalDepth) return { invalidRestriction, true };
            if (const auto memo = equivalentMemo.find(nodeId); memo != equivalentMemo.end())
                return { memo->second, false };
            const auto foundNode = nodesById.find(nodeId);
            if (foundNode == nodesById.end()) return { invalidRestriction, false };
            if (!equivalentActive.insert(nodeId).second) return { invalidRestriction, true };
            const auto& node = graph.nodes_[foundNode->second];
            double result = node.restriction;
            auto pathDependent = false;
            if (!std::isfinite(result) || result < 0.0) {
                result = invalidRestriction;
            } else if (node.type != ExhaustNodeType::outlet) {
                const auto foundOutputs = outgoing.find(nodeId);
                if (foundOutputs == outgoing.end() || foundOutputs->second.empty()) {
                    result = invalidRestriction;
                } else {
                    double downstreamConductance = 0.0;
                    for (const auto nextNodeId : foundOutputs->second) {
                        const auto branch = equivalentRestriction(nextNodeId, depth + 1);
                        pathDependent = pathDependent || branch.pathDependent;
                        if (!std::isfinite(branch.value) || branch.value < 0.0) {
                            downstreamConductance = 0.0;
                            result = invalidRestriction;
                            break;
                        }
                        downstreamConductance += 1.0
                            / std::sqrt(std::max(1.0e-9, branch.value));
                    }
                    if (std::isfinite(result)) {
                        if (downstreamConductance <= 0.0) result = invalidRestriction;
                        else result += 1.0 / (downstreamConductance * downstreamConductance);
                    }
                }
            }
            equivalentActive.erase(nodeId);
            if (!pathDependent) equivalentMemo.emplace(nodeId, result);
            return { result, pathDependent };
        };

    double totalRestriction = 0.0;
    for (const auto& root : roots) {
        const auto compiled = equivalentRestriction(root.nodeId, 0).value;
        // A malformed topology is rejected by validation immediately after
        // construction. Until then, keep all transient state finite and safe --
        // but say so, because charging a route the maximum restriction is
        // indistinguishable by ear from a deliberately restrictive exhaust.
        if (!std::isfinite(compiled))
            graph.diagnostics_.push_back(
                { ExhaustCompileIssue::unresolvedRestriction, root.cylinderId });
        const auto safeRestriction = std::isfinite(compiled)
            ? std::clamp(compiled, 0.0, 100.0) : 100.0;
        graph.cylinderRestrictions_.push_back({ root.cylinderId, root.pathIndex, safeRestriction });
        totalRestriction += safeRestriction;
    }
    graph.effectiveRestriction_ = totalRestriction
        / static_cast<double>(std::max<std::size_t>(1, roots.size()));

    const auto acousticAdmittance = [&](std::uint32_t nodeId) noexcept {
        const auto foundNode = nodesById.find(nodeId);
        if (foundNode == nodesById.end()) return 0.0;
        const auto restriction = equivalentRestriction(nodeId, 0).value;
        if (!std::isfinite(restriction) || restriction < 0.0) return 0.0;
        const auto diameterMm = finiteClamped(
            graph.nodes_[foundNode->second].diameterMm, 5.0, 500.0, 42.0);
        const auto areaMm2 = std::numbers::pi * diameterMm * diameterMm * 0.25;
        // Characteristic admittance is proportional to area/(rho*c). rho and
        // c are common at a junction; the equivalent K supplies a bounded
        // approximation of the downstream resistive load.
        return areaMm2 / std::sqrt(1.0 + restriction);
    };

    auto routeLimitReported = false;
    for (const auto& root : roots) {
        std::unordered_set<std::uint32_t> activeNodes;
        // ModeSet carries a fixed mode array; taking it by const reference and
        // copying once per node keeps the per-branch copy the algorithm needs
        // without also copying it on every call.
        std::function<void(std::uint32_t, double, double, double, const ModeSet&)> visit =
            [&](std::uint32_t nodeId, double lengthMm, double restriction,
                double logAmplitudeGain, const ModeSet& inheritedModes) {
                if (graph.routes_.size() >= maximumCompiledRoutes) {
                    if (!routeLimitReported) {
                        routeLimitReported = true;
                        graph.diagnostics_.push_back(
                            { ExhaustCompileIssue::routeLimitReached, root.cylinderId });
                    }
                    return;
                }
                const auto foundNode = nodesById.find(nodeId);
                if (foundNode == nodesById.end()) return;
                if (activeNodes.size() >= maximumTraversalDepth
                    || !activeNodes.insert(nodeId).second) return;
                const auto& node = graph.nodes_[foundNode->second];
                auto modes = inheritedModes;
                lengthMm += node.lengthMm;
                restriction += node.restriction;
                if (node.resonanceStrength > 0.0)
                    modes.add(node.resonanceHz,
                        node.resonanceStrength * node.resonanceStrength);
                if (node.audioGain <= 0.0) {
                    logAmplitudeGain = -std::numeric_limits<double>::infinity();
                } else if (logAmplitudeGain != -std::numeric_limits<double>::infinity()) {
                    logAmplitudeGain += std::log(node.audioGain);
                }
                if (node.type == ExhaustNodeType::outlet) {
                    // A complete cylinder-to-open-outlet route behaves as a
                    // quarter-wave resonator. Retain its first odd modes and
                    // any explicitly authored/local component modes instead
                    // of selecting the largest component frequency.
                    const auto effectiveLengthMm = lengthMm + 0.3 * node.diameterMm;
                    if (effectiveLengthMm > 0.0) {
                        const auto fundamentalHz = graph.waveSpeedMmPerSecond_
                            / (4.0 * effectiveLengthMm);
                        for (const auto order : { 1.0, 3.0, 5.0 })
                            modes.add(fundamentalHz * order, 1.0 / (order * order));
                    }
                    modes.normaliseAndSort();

                    ExhaustRoute route;
                    route.cylinderId = root.cylinderId;
                    route.pathIndex = root.pathIndex;
                    route.outletNodeId = node.id;
                    route.lengthMm = lengthMm;
                    route.delaySeconds = lengthMm / graph.waveSpeedMmPerSecond_;
                    route.restriction = restriction;
                    route.audioGain = amplitudeFromLog(logAmplitudeGain);
                    route.modeCount = modes.count;
                    route.modes = modes.values;
                    route.resonanceHz = route.modeCount > 0
                        ? route.modes.front().frequencyHz : 0.0;
                    graph.routes_.push_back(route);
                    activeNodes.erase(nodeId);
                    return;
                }
                const auto foundOutputs = outgoing.find(nodeId);
                if (foundOutputs == outgoing.end() || foundOutputs->second.empty()) {
                    activeNodes.erase(nodeId);
                    return;
                }
                double totalAdmittance = 0.0;
                for (const auto nextNodeId : foundOutputs->second)
                    totalAdmittance += acousticAdmittance(nextNodeId);
                const auto equalEnergyShare = 1.0
                    / static_cast<double>(foundOutputs->second.size());
                for (const auto nextNodeId : foundOutputs->second) {
                    const auto branchAdmittance = acousticAdmittance(nextNodeId);
                    const auto energyShare = totalAdmittance > 0.0
                        ? branchAdmittance / totalAdmittance : equalEnergyShare;
                    const auto branchLogGain = energyShare > 0.0
                        && logAmplitudeGain != -std::numeric_limits<double>::infinity()
                        ? logAmplitudeGain + 0.5 * std::log(energyShare)
                        : -std::numeric_limits<double>::infinity();
                    visit(nextNodeId, lengthMm, restriction, branchLogGain, modes);
                }
                activeNodes.erase(nodeId);
            };
        visit(root.nodeId, 0.0, 0.0, 0.0, {});
    }

    std::vector<double> restrictionSums(paths.size(), 0.0);
    std::vector<std::size_t> restrictionCounts(paths.size(), 0);
    double legacyRestrictionSum = 0.0;
    std::size_t legacyRestrictionCount = 0;
    for (const auto& restriction : graph.cylinderRestrictions_) {
        const auto pathIndex = static_cast<std::size_t>(restriction.pathIndex);
        if (pathIndex >= graph.pathFlowProperties_.size()) continue;
        restrictionSums[pathIndex] += restriction.equivalentRestriction;
        ++restrictionCounts[pathIndex];
        if (!graph.pathFlowProperties_[pathIndex].authoredNetwork) {
            legacyRestrictionSum += restriction.equivalentRestriction;
            ++legacyRestrictionCount;
        }
    }
    std::vector<double> routeLengthSums(paths.size(), 0.0);
    std::vector<std::size_t> routeLengthCounts(paths.size(), 0);
    for (const auto& route : graph.routes_) {
        const auto pathIndex = static_cast<std::size_t>(route.pathIndex);
        if (pathIndex >= routeLengthSums.size()) continue;
        routeLengthSums[pathIndex] += route.lengthMm;
        ++routeLengthCounts[pathIndex];
    }
    for (std::size_t pathIndex = 0; pathIndex < graph.pathFlowProperties_.size(); ++pathIndex) {
        auto& flow = graph.pathFlowProperties_[pathIndex];
        if (restrictionCounts[pathIndex] > 0) {
            flow.equivalentRestriction = restrictionSums[pathIndex]
                / static_cast<double>(restrictionCounts[pathIndex]);
        }
        if (routeLengthCounts[pathIndex] > 0) {
            flow.meanFlowLengthMm = routeLengthSums[pathIndex]
                / static_cast<double>(routeLengthCounts[pathIndex]);
        }
        if (flow.authoredNetwork) {
            // Parallel outlet conductances were summed as A*Cd above. Apply
            // the network's equivalent K exactly once and expose Cd=1 to the
            // conservative solver.
            flow.effectiveOutletAreaM2 /= std::sqrt(1.0 + flow.equivalentRestriction);
            flow.effectiveOutletAreaM2 = std::max(1.0e-9, flow.effectiveOutletAreaM2);
        }
    }
    graph.legacyEffectiveRestriction_ = legacyRestrictionCount > 0
        ? legacyRestrictionSum / static_cast<double>(legacyRestrictionCount) : 0.0;
    graph.ambientPressureKpa_ = finiteClamped(config.ambientPressureKpa, 50.0, 120.0, 101.325);
    return graph;
}

double ExhaustGraph::backPressureKpa(const EngineState& state) const noexcept {
    const auto massFlowGramsPerSecond = std::max(state.airFlowGramsPerSecond + state.fuelFlowGramsPerSecond,
                                                 state.exhaustFlowGramsPerSecond);
    const auto flowFactor = std::max(0.0, massFlowGramsPerSecond * 0.0042 + state.rpm / 18'000.0);
    const auto runnerPulse = std::max(0.0, state.exhaustRunnerPressureKpa - ambientPressureKpa_) * 0.045;
    // Authored networks are already closed through their compiled area/K in
    // EngineSimulator. Keep the analytical fallback only for legacy paths so
    // custom restrictions are not charged twice.
    return ambientPressureKpa_ + legacyEffectiveRestriction_ * flowFactor * flowFactor * 48.0
        + runnerPulse;
}

ExhaustCylinderAcoustics ExhaustGraph::acousticsForCylinder(
    std::uint32_t cylinderId) const noexcept {
    ExhaustCylinderAcoustics acoustics;
    acoustics.cylinderId = cylinderId;
    double energySum = 0.0;
    double weightedLength = 0.0;
    double weightedDelay = 0.0;
    double weightedDelaySquared = 0.0;
    double unweightedLength = 0.0;
    double unweightedDelay = 0.0;
    double dominantRouteEnergy = -1.0;
    double dominantRouteDelay = std::numeric_limits<double>::infinity();
    double firstAudibleDelay = std::numeric_limits<double>::infinity();
    double lastAudibleDelay = 0.0;
    double firstGeometricDelay = std::numeric_limits<double>::infinity();
    double lastGeometricDelay = 0.0;
    ModeSet combinedModes;
    for (const auto& route : routes_) {
        if (route.cylinderId != cylinderId) continue;
        if (acoustics.routeCount == 0) acoustics.pathIndex = route.pathIndex;
        ++acoustics.routeCount;
        unweightedLength += route.lengthMm;
        unweightedDelay += route.delaySeconds;
        firstGeometricDelay = std::min(firstGeometricDelay, route.delaySeconds);
        lastGeometricDelay = std::max(lastGeometricDelay, route.delaySeconds);

        const auto routeEnergy = std::isfinite(route.audioGain)
            ? std::max(0.0, route.audioGain * route.audioGain) : 0.0;
        if (routeEnergy <= 0.0) continue;
        energySum += routeEnergy;
        weightedLength += route.lengthMm * routeEnergy;
        weightedDelay += route.delaySeconds * routeEnergy;
        weightedDelaySquared += route.delaySeconds * route.delaySeconds * routeEnergy;
        firstAudibleDelay = std::min(firstAudibleDelay, route.delaySeconds);
        lastAudibleDelay = std::max(lastAudibleDelay, route.delaySeconds);
        if (routeEnergy > dominantRouteEnergy
            || (routeEnergy == dominantRouteEnergy && route.delaySeconds < dominantRouteDelay)) {
            dominantRouteEnergy = routeEnergy;
            dominantRouteDelay = route.delaySeconds;
        }
        if (route.modeCount > 0) {
            for (std::size_t index = 0; index < route.modeCount; ++index)
                combinedModes.add(route.modes[index].frequencyHz,
                    routeEnergy * route.modes[index].relativeEnergy);
        } else {
            combinedModes.add(route.resonanceHz, routeEnergy);
        }
    }
    const auto cylinderRestriction = std::find_if(cylinderRestrictions_.begin(), cylinderRestrictions_.end(),
        [cylinderId](const CylinderRestriction& item) { return item.cylinderId == cylinderId; });
    const auto routeRestriction = cylinderRestriction != cylinderRestrictions_.end()
        ? cylinderRestriction->equivalentRestriction : effectiveRestriction_;
    acoustics.equivalentRestriction = std::clamp(routeRestriction, 0.0, 100.0);
    if (acoustics.routeCount == 0) {
        // Unknown ports must not unexpectedly attenuate an otherwise valid
        // event. Real configured cylinders always have at least one route.
        acoustics.transmissionGain = 1.0;
        return acoustics;
    }

    if (energySum > 0.0) {
        acoustics.meanLengthMm = weightedLength / energySum;
        acoustics.meanDelaySeconds = weightedDelay / energySum;
        acoustics.delaySeconds = dominantRouteDelay;
        acoustics.firstArrivalDelaySeconds = firstAudibleDelay;
        acoustics.lastArrivalDelaySeconds = lastAudibleDelay;
        const auto delayVariance = std::max(0.0,
            weightedDelaySquared / energySum
                - acoustics.meanDelaySeconds * acoustics.meanDelaySeconds);
        acoustics.rmsDelaySpreadSeconds = std::sqrt(delayVariance);
        acoustics.transmissionGain = restrictionTransmission(acoustics.equivalentRestriction)
            * std::min(maximumCompiledAmplitudeGain, std::sqrt(energySum));
        combinedModes.normaliseAndSort();
        acoustics.modeCount = combinedModes.count;
        acoustics.modes = combinedModes.values;
        acoustics.resonanceHz = acoustics.modeCount > 0
            ? acoustics.modes.front().frequencyHz : 0.0;
    } else {
        // A deliberately muted graph still publishes finite geometric timing
        // for editors and diagnostics, while correctly transmitting no sound.
        const auto routeCount = static_cast<double>(acoustics.routeCount);
        acoustics.meanLengthMm = unweightedLength / routeCount;
        acoustics.meanDelaySeconds = unweightedDelay / routeCount;
        acoustics.delaySeconds = firstGeometricDelay;
        acoustics.firstArrivalDelaySeconds = firstGeometricDelay;
        acoustics.lastArrivalDelaySeconds = lastGeometricDelay;
        acoustics.transmissionGain = 0.0;
    }
    return acoustics;
}

ExhaustPathFlowProperties ExhaustGraph::pathFlowProperties(
    std::size_t pathIndex) const noexcept {
    return pathIndex < pathFlowProperties_.size()
        ? pathFlowProperties_[pathIndex] : ExhaustPathFlowProperties {};
}

ExhaustCylinderFlowProperties ExhaustGraph::cylinderFlowProperties(
    std::uint32_t cylinderId) const noexcept {
    const auto found = std::find_if(cylinderFlowProperties_.begin(),
        cylinderFlowProperties_.end(), [cylinderId](const auto& item) {
            return item.authoredNetwork && item.cylinderId == cylinderId;
        });
    if (found != cylinderFlowProperties_.end()) return *found;
    return {};
}

void ExhaustGraph::process(FiringEvent& event) const noexcept {
    const auto cylinderId = event.exhaustPortId != 0 ? event.exhaustPortId : event.cylinderId;
    const auto acoustics = acousticsForCylinder(cylinderId);
    const auto baseDelaySeconds = event.exhaustDelaySeconds;
    const auto baseTransmissionGain = event.exhaustTransmissionGain;
    if (acoustics.routeCount > 0) {
        event.exhaustPathIndex = acoustics.pathIndex;
        event.exhaustDelaySeconds += static_cast<float>(acoustics.delaySeconds);
        event.exhaustResonanceHz = static_cast<float>(acoustics.resonanceHz);
    }
    event.exhaustTransmissionGain *= static_cast<float>(acoustics.transmissionGain);

    struct Component final {
        double energy { 0.0 };
        double delaySeconds { 0.0 };
        double amplitude { 0.0 };
        double resonanceHz { 0.0 };
        std::uint32_t pathIndex { 0 };
    };
    std::array<Component, maximumExhaustEventComponents> strongest {};
    std::size_t strongestCount = 0;
    const auto retain = [&strongest, &strongestCount](Component candidate) noexcept {
        if (!(candidate.energy > 0.0) || !std::isfinite(candidate.energy)) return;
        if (strongestCount < strongest.size()) {
            strongest[strongestCount++] = candidate;
            return;
        }
        const auto weakest = std::min_element(strongest.begin(), strongest.end(),
            [](const auto& left, const auto& right) { return left.energy < right.energy; });
        if (candidate.energy > weakest->energy) *weakest = candidate;
    };
    for (const auto& route : routes_) {
        if (route.cylinderId != cylinderId || !(route.audioGain > 0.0)) continue;
        if (route.modeCount == 0) {
            retain({ route.audioGain * route.audioGain, route.delaySeconds,
                route.audioGain, route.resonanceHz, route.pathIndex });
            continue;
        }
        for (std::size_t index = 0; index < route.modeCount; ++index) {
            const auto modeAmplitude = route.audioGain
                * std::sqrt(std::max(0.0, route.modes[index].relativeEnergy));
            retain({ modeAmplitude * modeAmplitude, route.delaySeconds,
                modeAmplitude, route.modes[index].frequencyHz, route.pathIndex });
        }
    }
    std::sort(strongest.begin(), strongest.begin() + static_cast<std::ptrdiff_t>(strongestCount),
        [](const auto& left, const auto& right) {
            if (left.delaySeconds != right.delaySeconds)
                return left.delaySeconds < right.delaySeconds;
            return left.energy > right.energy;
        });
    double retainedEnergy = 0.0;
    for (std::size_t index = 0; index < strongestCount; ++index)
        retainedEnergy += strongest[index].energy;
    const auto componentScale = retainedEnergy > 0.0
        ? static_cast<double>(baseTransmissionGain) * acoustics.transmissionGain
            / std::sqrt(retainedEnergy)
        : 0.0;
    event.exhaustComponentCount = static_cast<std::uint8_t>(strongestCount);
    for (std::size_t index = 0; index < strongestCount; ++index) {
        event.exhaustComponentDelaySeconds[index] = baseDelaySeconds
            + static_cast<float>(strongest[index].delaySeconds);
        event.exhaustComponentGain[index] = static_cast<float>(
            strongest[index].amplitude * componentScale);
        event.exhaustComponentResonanceHz[index] = static_cast<float>(
            strongest[index].resonanceHz);
        event.exhaustComponentPathIndex[index] = static_cast<std::uint8_t>(
            std::min<std::uint32_t>(strongest[index].pathIndex, 255U));
    }
}
} // namespace enginelab
