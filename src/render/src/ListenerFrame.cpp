#include <enginelab/render/ListenerFrame.hpp>

#include <enginelab/render/EngineModel3D.hpp>

#include <cmath>

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

std::array<AcousticPoint3M, 2> ListenerFrame::microphones(Vec3 eyeMm, Vec3 right, double spacingM) const noexcept {
    const auto half = right * static_cast<float>(500.0 * spacingM);
    return { acoustic(eyeMm - half), acoustic(eyeMm + half) };
}

} // namespace enginelab::render
