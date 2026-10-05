#include <enginelab/exhaust/ExhaustComponentResize.hpp>

#include <enginelab/exhaust/LegacyExhaustNetwork.hpp>

#include <algorithm>
#include <cmath>
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
        size.sharedByPrimaries = component->type == ExhaustComponentType::pipe && path->cylinderIds.size() > 1U;
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
            geometry.primaryLengthMm = lengthMm;
            geometry.primaryDiameterMm = diameterMm;
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
        component.lengthMm = lengthMm;
        component.diameterMm = diameterMm;
    }
    config = std::move(edited);
    return {};
}

} // namespace enginelab
