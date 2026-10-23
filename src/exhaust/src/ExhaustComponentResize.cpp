#include <enginelab/exhaust/ExhaustComponentResize.hpp>

#include <enginelab/exhaust/LegacyExhaustNetwork.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

namespace enginelab {
namespace {

constexpr double minimumLengthMm = 10.0;
constexpr double maximumLengthMm = 5'000.0;
constexpr double minimumDiameterMm = 10.0;
constexpr double maximumDiameterMm = 400.0;

template <typename Config>
[[nodiscard]] auto* findPath(Config& config, std::uint32_t pathId) noexcept {
    const auto found = std::find_if(config.exhaustPaths.begin(), config.exhaustPaths.end(),
        [pathId](const ExhaustPathConfig& path) { return path.id == pathId; });
    return found != config.exhaustPaths.end() ? &*found : nullptr;
}

[[nodiscard]] const ExhaustComponentConfig* findComponent(const ExhaustNetworkConfig& network,
                                                          std::uint32_t componentId) noexcept {
    const auto found = std::find_if(network.components.begin(), network.components.end(),
        [componentId](const ExhaustComponentConfig& component) { return component.id == componentId; });
    return found != network.components.end() ? &*found : nullptr;
}

[[nodiscard]] bool hasLength(ExhaustComponentType type) noexcept {
    switch (type) {
    case ExhaustComponentType::pipe:
    case ExhaustComponentType::muffler:
    case ExhaustComponentType::resonator:
    case ExhaustComponentType::catalyst:
    case ExhaustComponentType::crossover: return true;
    case ExhaustComponentType::merge:
    case ExhaustComponentType::splitter:
    case ExhaustComponentType::outlet: return false;
    }
    return false;
}

[[nodiscard]] bool isPrimary(const ExhaustNetworkConfig& network, std::uint32_t componentId) noexcept {
    return std::any_of(network.cylinderConnections.begin(), network.cylinderConnections.end(),
        [componentId](const ExhaustCylinderConnectionConfig& connection) {
            return connection.componentId == componentId;
        });
}

[[nodiscard]] std::vector<ExhaustComponentType> typesOf(const ExhaustNetworkConfig& network) {
    std::vector<ExhaustComponentType> types;
    types.reserve(network.components.size());
    for (const auto& component : network.components) types.push_back(component.type);
    return types;
}

} // namespace

std::optional<ExhaustComponentSize> exhaustComponentSize(const EngineConfig& config, std::uint32_t pathId,
                                                         std::uint32_t componentId) {
    const auto* path = findPath(config, pathId);
    if (path == nullptr) return std::nullopt;
    const auto scalar = !path->network || path->network->components.empty();
    const auto network = scalar ? makeEditableExhaustNetwork(*path) : *path->network;
    const auto* component = findComponent(network, componentId);
    if (component == nullptr) return std::nullopt;
    ExhaustComponentSize size;
    size.lengthMm = component->lengthMm;
    size.diameterMm = component->diameterMm;
    if (scalar) {
        // The scalar fields: a silencer is sized by its body, which the
        // network shows as a volume over a core passage.
        size.lengthEditable = component->type == ExhaustComponentType::pipe
            || component->type == ExhaustComponentType::muffler;
        if (component->type == ExhaustComponentType::muffler)
            size.diameterMm = path->geometry.mufflerChamberDiameterMm;
        size.sharedByPrimaries = component->type == ExhaustComponentType::pipe && path->cylinderIds.size() > 1U
            && isPrimary(network, componentId);
    } else {
        size.lengthEditable = hasLength(component->type) || component->lengthMm > 0.0;
    }
    return size;
}

std::string resizeExhaustComponent(EngineConfig& config, std::uint32_t pathId, std::uint32_t componentId,
                                   double lengthMm, double diameterMm) {
    const auto size = exhaustComponentSize(config, pathId, componentId);
    if (!size) return "No such exhaust component.";
    if (!(diameterMm >= minimumDiameterMm && diameterMm <= maximumDiameterMm))
        return "The diameter must be between 10 and 400 mm.";
    const auto lengthChanged = std::abs(lengthMm - size->lengthMm) > 1.0e-9;
    if (lengthChanged && !size->lengthEditable) return "This component has no length to edit.";
    if (lengthChanged && !(lengthMm >= minimumLengthMm && lengthMm <= maximumLengthMm))
        return "The length must be between 10 and 5,000 mm.";

    auto edited = config;
    auto& path = *findPath(edited, pathId);
    const auto scalar = !path.network || path.network->components.empty();
    if (scalar) {
        const auto before = makeEditableExhaustNetwork(path);
        auto& geometry = path.geometry;
        switch (findComponent(before, componentId)->type) {
        case ExhaustComponentType::pipe:
            if (isPrimary(before, componentId)) {
                geometry.primaryLengthMm = lengthMm;
                geometry.primaryDiameterMm = diameterMm;
            } else {
                geometry.midpipeLengthMm = lengthMm;
                geometry.midpipeDiameterMm = diameterMm;
            }
            break;
        case ExhaustComponentType::merge: geometry.collectorDiameterMm = diameterMm; break;
        case ExhaustComponentType::muffler:
            geometry.mufflerChamberLengthMm = lengthMm;
            geometry.mufflerChamberDiameterMm = diameterMm;
            break;
        case ExhaustComponentType::outlet: geometry.outletDiameterMm = diameterMm; break;
        case ExhaustComponentType::splitter:
        case ExhaustComponentType::resonator:
        case ExhaustComponentType::catalyst:
        case ExhaustComponentType::crossover: return "This component cannot be resized here.";
        }
        if (typesOf(makeEditableExhaustNetwork(path)) != typesOf(before))
            return "The silencer must stay wider than the pipes on either side of it.";
        // A single path mirrors the global geometry: normalisation copies
        // one over the other, and the solver reads the global one.
        if (edited.exhaustPaths.size() == 1U) edited.exhaust = path.geometry;
    } else {
        auto& component = *std::find_if(path.network->components.begin(), path.network->components.end(),
            [componentId](const ExhaustComponentConfig& item) { return item.id == componentId; });
        // An authored collector body follows its geometry: a shorter or
        // narrower collector holds less gas. Left as it was, the volume stayed
        // lumped at the branch however short the trunk was made.
        if (component.type == ExhaustComponentType::merge && component.volumeLitres > 0.0
                && component.lengthMm > 0.0 && component.diameterMm > 0.0)
            component.volumeLitres *= (lengthMm * diameterMm * diameterMm)
                / (component.lengthMm * component.diameterMm * component.diameterMm);
        component.lengthMm = lengthMm;
        component.diameterMm = diameterMm;
    }
    config = std::move(edited);
    return {};
}

std::string joinExhaustPaths(EngineConfig& config, std::uint32_t keptId, std::uint32_t joinedId) {
    if (keptId == joinedId) return "A path cannot be joined to itself.";
    const auto* keptPath = findPath(config, keptId);
    const auto* joinedPath = findPath(config, joinedId);
    if (keptPath == nullptr || joinedPath == nullptr) return "The engine has no such exhaust paths.";
    if (config.forcedInduction.enabled && config.forcedInduction.type == ForcedInductionType::turbocharger
        && config.forcedInduction.turbochargerCount + 1U > config.exhaustPaths.size())
        return "Each turbocharger needs an exhaust path of its own.";
    auto network = keptPath->network ? *keptPath->network : makeEditableExhaustNetwork(*keptPath);
    if (network.cylinderConnections.empty()) return "The exhaust path has no primary to copy.";

    std::unordered_map<std::uint32_t, std::size_t> incoming;
    std::unordered_map<std::uint32_t, std::vector<const ExhaustComponentConnectionConfig*>> outgoing;
    for (const auto& connection : network.cylinderConnections) ++incoming[connection.componentId];
    for (const auto& connection : network.connections) {
        ++incoming[connection.toComponentId];
        outgoing[connection.fromComponentId].push_back(&connection);
    }
    // The first cylinder's own run: each component fed by the one before
    // alone, up to the junction several pipes meet at.
    // Copied out: the network grows below.
    std::vector<std::uint32_t> run { network.cylinderConnections.front().componentId };
    std::vector<ExhaustComponentConnectionConfig> runLinks;
    ExhaustComponentConnectionConfig intoJunction;
    while (true) {
        const auto& next = outgoing[run.back()];
        if (next.empty()) return "The primary meets no other pipe before its outlet.";
        if (next.size() != 1U) return "The primary splits before it meets the others.";
        if (incoming[next.front()->toComponentId] > 1U) {
            intoJunction = *next.front();
            break;
        }
        if (run.size() >= network.components.size()) return "The exhaust graph loops.";
        runLinks.push_back(*next.front());
        run.push_back(next.front()->toComponentId);
    }

    auto nextId = std::uint32_t { 1 };
    for (const auto& component : network.components) nextId = std::max(nextId, component.id + 1U);
    for (const auto cylinderId : joinedPath->cylinderIds) {
        std::uint32_t previous = 0;
        for (std::size_t index = 0; index < run.size(); ++index) {
            const auto* source = findComponent(network, run[index]);
            if (source == nullptr) return "The exhaust graph names a missing component.";
            auto copy = *source;
            copy.id = nextId++;
            if (index == 0) {
                network.cylinderConnections.push_back({ cylinderId, copy.id });
            } else {
                // The same ports as the run it copies.
                auto link = runLinks[index - 1];
                link.fromComponentId = previous;
                link.toComponentId = copy.id;
                network.connections.push_back(link);
            }
            previous = copy.id;
            network.components.push_back(std::move(copy));
        }
        auto link = intoJunction;
        link.fromComponentId = previous;
        network.connections.push_back(link);
    }

    auto edited = config;
    auto* kept = findPath(edited, keptId);
    kept->cylinderIds.insert(kept->cylinderIds.end(), joinedPath->cylinderIds.begin(), joinedPath->cylinderIds.end());
    kept->network = std::move(network);
    kept->inheritsGlobalGeometry = false;
    std::erase_if(edited.exhaustPaths, [joinedId](const ExhaustPathConfig& path) { return path.id == joinedId; });
    for (auto& bank : edited.banks)
        if (bank.exhaustPathId == joinedId) bank.exhaustPathId = keptId;
    // A turbocharger chosen by path index moves with the paths after the one
    // removed: name the units the paths drove.
    for (std::size_t index = 0; index < edited.exhaustPaths.size(); ++index) {
        auto& path = edited.exhaustPaths[index];
        if (path.turbocharger >= 0 || config.forcedInduction.turbochargerCount < 2U) continue;
        const auto was = std::find_if(config.exhaustPaths.begin(), config.exhaustPaths.end(),
            [&path](const ExhaustPathConfig& item) { return item.id == path.id; });
        path.turbocharger = static_cast<std::int32_t>(static_cast<std::size_t>(was - config.exhaustPaths.begin())
                                                      % config.forcedInduction.turbochargerCount);
    }
    if (config.forcedInduction.enabled && config.forcedInduction.type == ForcedInductionType::turbocharger)
        for (std::uint32_t unit = 0; unit < config.forcedInduction.turbochargerCount; ++unit) {
            const auto driven = std::any_of(edited.exhaustPaths.begin(), edited.exhaustPaths.end(),
                [&](const ExhaustPathConfig& candidate) {
                    const auto index = static_cast<std::size_t>(&candidate - edited.exhaustPaths.data());
                    const auto drives = candidate.turbocharger >= 0 ? static_cast<std::uint32_t>(candidate.turbocharger)
                        : static_cast<std::uint32_t>(index % config.forcedInduction.turbochargerCount);
                    return drives == unit;
                });
            if (!driven) return "Turbocharger " + std::to_string(unit + 1U) + " would have no exhaust path.";
        }
    if (edited.exhaustPaths.size() == 1U) edited.exhaust = edited.exhaustPaths.front().geometry;
    config = std::move(edited);
    return {};
}

} // namespace enginelab
