#include <enginelab/render/GasFieldView.hpp>

#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/exhaust/LegacyExhaustNetwork.hpp>

#include <algorithm>
#include <cmath>
#include <optional>

namespace enginelab::render {
namespace {

[[nodiscard]] ExhaustNodeType nodeTypeOf(ExhaustComponentType type) noexcept {
    switch (type) {
    case ExhaustComponentType::pipe: return ExhaustNodeType::pipe;
    case ExhaustComponentType::merge: return ExhaustNodeType::merge;
    case ExhaustComponentType::splitter: return ExhaustNodeType::splitter;
    case ExhaustComponentType::resonator: return ExhaustNodeType::resonator;
    case ExhaustComponentType::muffler: return ExhaustNodeType::muffler;
    case ExhaustComponentType::catalyst: return ExhaustNodeType::catalyst;
    case ExhaustComponentType::outlet: return ExhaustNodeType::outlet;
    case ExhaustComponentType::crossover: return ExhaustNodeType::crossover;
    }
    return ExhaustNodeType::pipe;
}

template <typename Paths>
[[nodiscard]] std::optional<std::uint32_t> indexOfPath(const Paths& paths, std::uint32_t id) {
    for (std::size_t i = 0; i < paths.size(); ++i)
        if (paths[i].id == id) return static_cast<std::uint32_t>(i);
    return std::nullopt;
}

[[nodiscard]] bool isExhaust(GasFieldElementKind kind) noexcept {
    return kind == GasFieldElementKind::exhaustDuct || kind == GasFieldElementKind::exhaustJunction;
}

[[nodiscard]] Vec3 srgb(std::uint32_t hex) noexcept {
    return { static_cast<float>((hex >> 16U) & 0xffU) / 255.0F, static_cast<float>((hex >> 8U) & 0xffU) / 255.0F,
             static_cast<float>(hex & 0xffU) / 255.0F };
}
} // namespace

std::vector<int> bindGasField(const EngineModel3D& model, const GasFieldSnapshot& field) {
    const auto& config = model.config();
    const auto& elements = field.elements;
    std::vector<int> binding(model.ducts().size(), -1);
    const auto find = [&elements](auto&& matches) {
        for (std::size_t i = 0; i < elements.size(); ++i)
            if (matches(elements[i])) return static_cast<int>(i);
        return -1;
    };

    for (std::size_t d = 0; d < model.ducts().size(); ++d) {
        const auto& duct = model.ducts()[d];
        switch (duct.kind) {
        case DuctKind::exhaustComponent:
        case DuctKind::exhaustFeeder: {
            const auto pathIndex = indexOfPath(config.exhaustPaths, duct.pathId);
            if (!pathIndex) break;
            const auto& path = config.exhaustPaths[*pathIndex];
            if (path.network && !path.network->components.empty()) {
                binding[d] = find([&](const GasFieldElement& e) {
                    return isExhaust(e.kind) && e.pathIndex == *pathIndex && e.componentId == duct.elementId;
                });
                break;
            }
            // A geometry-only path: the solver compiles it without component
            // ids, a primary per cylinder (node id = cylinder id) and one node
            // of each other type.
            const auto type = static_cast<std::uint8_t>(nodeTypeOf(duct.componentType));
            std::optional<std::uint32_t> cylinderId;
            if (duct.componentType == ExhaustComponentType::pipe) {
                for (const auto& connection : makeEditableExhaustNetwork(path).cylinderConnections)
                    if (connection.componentId == duct.elementId) cylinderId = connection.cylinderId;
            }
            binding[d] = find([&](const GasFieldElement& e) {
                return isExhaust(e.kind) && e.pathIndex == *pathIndex && e.nodeType == type
                    && (!cylinderId || e.nodeId == *cylinderId);
            });
            break;
        }
        case DuctKind::intakeRunner:
            binding[d] = find([&](const GasFieldElement& e) {
                return e.kind == GasFieldElementKind::intakeRunner && e.cylinderId == duct.elementId;
            });
            break;
        case DuctKind::intakePlenum: {
            const auto pathIndex = indexOfPath(config.intakePaths, duct.pathId);
            binding[d] = find([&](const GasFieldElement& e) {
                return e.kind == GasFieldElementKind::intakePlenum && e.pathIndex == pathIndex.value_or(0U);
            });
            break;
        }
        case DuctKind::intakeThrottle:
        case DuctKind::intakeAirbox:
        case DuctKind::intakeInletDuct: break;
        }
    }
    return binding;
}

Vec3 pressureColour(float normalised) noexcept {
    // Violet for rarefaction so it never reads as intake air (cyan).
    static const auto neutral = srgb(0x4a5451);
    static const auto positive = srgb(0xff6a2b);
    static const auto hot = srgb(0xffe6b0);
    static const auto negative = srgb(0x9a7bff);
    // Compressed (power 0.75): weak reflections downstream stay visible next
    // to the strong blowdown pulses.
    const auto magnitude = std::pow(std::min(1.0F, std::abs(normalised)), 0.75F);
    Vec3 colour = neutral;
    if (!(normalised < 0.0F)) {
        colour = lerp(neutral, positive, std::min(1.0F, magnitude / 0.7F));
        if (magnitude > 0.7F) colour = lerp(colour, hot, (magnitude - 0.7F) / 0.3F);
    } else {
        colour = lerp(neutral, negative, magnitude);
    }
    return { std::pow(colour.x, 2.2F), std::pow(colour.y, 2.2F), std::pow(colour.z, 2.2F) };
}

void GasFieldView::clear() {
    model_ = nullptr;
    boundElements_ = 0;
    binding_.clear();
    parts_.clear();
    meanPa_.clear();
    meanSeconds_ = 0.0;
    scalePa_ = 0.0F;
    afterfire_ = 0.0F;
}

void GasFieldView::update(const EngineModel3D& model, const GasFieldSnapshot& field) {
    if (model_ != &model || boundElements_ != field.elements.size()) {
        clear();
        model_ = &model;
        boundElements_ = field.elements.size();
        binding_ = bindGasField(model, field);
    }

    // The running mean, over simulated time: 0.5 s is three cycles at
    // 750 rpm. A clock that goes back (a restart) starts it again; a frozen
    // view leaves it alone.
    const auto elapsed = field.simulationTimeSeconds - meanSeconds_;
    const auto restart = meanPa_.size() != field.elements.size() || elapsed < 0.0;
    if (restart) meanPa_.assign(field.elements.size(), {});
    const auto weight = restart ? 1.0F : static_cast<float>(1.0 - std::exp(-elapsed / 0.5));
    meanSeconds_ = field.simulationTimeSeconds;
    float peak = 0.0F;
    for (std::size_t e = 0; e < field.elements.size(); ++e) {
        const auto& element = field.elements[e];
        for (std::size_t s = 0; s < element.sampleCount; ++s) {
            auto& mean = meanPa_[e][s];
            mean += weight * (element.pressurePa[s] - mean);
            peak = std::max(peak, std::abs(element.pressurePa[s] - mean));
        }
    }
    // A slowly decaying peak, never under 2 kPa: a quiet engine is not
    // amplified into noise.
    scalePa_ = std::max({ 2'000.0F, peak, 0.94F * scalePa_ });
    afterfire_ = static_cast<float>(1.0 - std::exp(-std::max(0.0, field.afterfireHeatReleaseKw) / 10.0));

    parts_.clear();
    for (std::size_t d = 0; d < binding_.size(); ++d) {
        const auto& duct = model.ducts()[d];
        if (binding_[d] < 0) continue;
        const auto& element = field.elements[static_cast<std::size_t>(binding_[d])];
        const auto& mean = meanPa_[static_cast<std::size_t>(binding_[d])];
        if (element.sampleCount == 0) continue;
        PartField part;
        part.part = duct.part;
        part.count = element.sampleCount;
        for (std::size_t s = 0; s < element.sampleCount; ++s) {
            const auto colour = pressureColour((element.pressurePa[s] - mean[s]) / scalePa_);
            part.samples[4 * s + 0] = colour.x;
            part.samples[4 * s + 1] = colour.y;
            part.samples[4 * s + 2] = colour.z;
            part.samples[4 * s + 3] = element.wallTemperatureK[s];
        }
        parts_.push_back(part);
    }
}

} // namespace enginelab::render
