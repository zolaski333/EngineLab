#include <enginelab/render/ListenerFrame.hpp>

#include <enginelab/render/EngineModel3D.hpp>

#include <cmath>
#include <optional>

namespace enginelab::render {

ListenerFrame ListenerFrame::of(const EngineModel3D& scene) noexcept {
    // The drawn tailpipe's horizontal direction, in the sound's axes (the
    // view's x, then its z as the sound's y). A vertical one gives none.
    double drawnX = 0.0;
    double drawnY = 1.0;
    for (const auto& duct : scene.ducts()) {
        if (duct.kind != DuctKind::exhaustComponent || duct.componentType != ExhaustComponentType::outlet
            || duct.centreline.size() < 2U)
            continue;
        const auto axis = duct.centreline.back() - duct.centreline[duct.centreline.size() - 2U];
        const auto horizontal = std::hypot(static_cast<double>(axis.x), static_cast<double>(axis.z));
        if (horizontal > 0.2 * static_cast<double>(length(axis))) {
            drawnX = axis.x / horizontal;
            drawnY = axis.z / horizontal;
        }
        break;
    }
    // The first outlet's acoustic axis: its component's in a network, as the
    // exhaust graph takes it, else its path's.
    const auto& paths = scene.config().exhaustPaths;
    auto wanted = paths.empty() ? AcousticPoint3M { 0.0, 1.0, 0.0 } : paths.front().acousticAxis;
    if (!paths.empty() && paths.front().network) {
        for (const auto& component : paths.front().network->components)
            if (component.type == ExhaustComponentType::outlet) {
                wanted = component.acousticAxis;
                break;
            }
    }
    auto wantedX = wanted.x;
    auto wantedY = wanted.y;
    const auto horizontal = std::hypot(wantedX, wantedY);
    if (horizontal > 1.0e-6) {
        wantedX /= horizontal;
        wantedY /= horizontal;
    } else {
        wantedX = 0.0;
        wantedY = 1.0;
    }
    // The turn taking the drawn direction onto the wanted one.
    ListenerFrame frame;
    frame.cosine = drawnX * wantedX + drawnY * wantedY;
    frame.sine = drawnX * wantedY - drawnY * wantedX;
    return frame;
}

AcousticPoint3M ListenerFrame::acoustic(Vec3 pointMm) const noexcept {
    const auto x = 0.001 * static_cast<double>(pointMm.x);
    const auto y = 0.001 * static_cast<double>(pointMm.z);
    return { cosine * x - sine * y, sine * x + cosine * y, 0.001 * static_cast<double>(pointMm.y) };
}

AcousticPoint3M ListenerFrame::direction(Vec3 axis) const noexcept {
    const auto x = static_cast<double>(axis.x);
    const auto y = static_cast<double>(axis.z);
    const auto z = static_cast<double>(axis.y);
    const auto norm = std::sqrt(x * x + y * y + z * z);
    if (!(norm > 1.0e-9)) return { 0.0, 1.0, 0.0 };
    return { (cosine * x - sine * y) / norm, (sine * x + cosine * y) / norm, z / norm };
}

AcousticSourcePlacements ListenerFrame::sources(const EngineModel3D& scene) const {
    const auto& config = scene.config();
    const auto indexOf = [](const auto& paths, std::uint32_t id) -> std::optional<std::uint32_t> {
        for (std::size_t index = 0; index < paths.size(); ++index)
            if (paths[index].id == id) return static_cast<std::uint32_t>(index);
        return std::nullopt;
    };
    AcousticSourcePlacements result;
    for (const auto& duct : scene.ducts()) {
        if (!duct.opensToAtmosphere) continue;
        AcousticSourcePlacement placement;
        placement.positionM = acoustic(duct.openEnd);
        placement.axis = direction(duct.openAxis);
        if (duct.kind == DuctKind::exhaustComponent) {
            const auto path = indexOf(config.exhaustPaths, duct.pathId);
            if (!path) continue;
            placement.pathIndex = *path;
            // A path without a network compiles its outlet with no component id.
            placement.componentId = config.exhaustPaths[*path].network ? duct.elementId : 0U;
            result.exhaustOutlets.push_back(placement);
        } else {
            // An engine without intake paths sounds its one legacy path.
            const auto path = config.intakePaths.empty() ? std::optional<std::uint32_t> { 0U }
                                                         : indexOf(config.intakePaths, duct.pathId);
            if (!path) continue;
            placement.pathIndex = *path;
            result.intakeMouths.push_back(placement);
        }
    }
    if (scene.turbo().placed)
        result.forcedInductionM = acoustic(scene.turbo().compressorCentre);
    else if (scene.supercharger().placed)
        result.forcedInductionM = acoustic(scene.supercharger().centre);
    return result;
}

std::array<AcousticPoint3M, 2> ListenerFrame::microphones(Vec3 eyeMm, Vec3 right, double spacingM) const noexcept {
    const auto half = right * static_cast<float>(500.0 * spacingM);
    return { acoustic(eyeMm - half), acoustic(eyeMm + half) };
}

} // namespace enginelab::render
