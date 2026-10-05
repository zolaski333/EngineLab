#include <enginelab/render/DuctLayout3D.hpp>

#include <cmath>
#include <numbers>

namespace enginelab::render {
namespace {
constexpr float pi = std::numbers::pi_v<float>;
} // namespace

void appendTurboHousings(Mesh& turbine, Mesh& compressor, const TurboPlacement& turbo) {
    if (!turbo.placed || turbo.scroll.size() < 2U) return;
    const auto axis = turbo.axis;
    const auto rt = turbo.turbineWheelRadius;
    const auto rk = turbo.compressorWheelRadius;
    const auto wt = turbo.turbineWidth;
    const auto wk = turbo.compressorWidth;

    // Turbine: the volute closes on itself round the wheel.
    appendTube(turbine, turbo.scroll, turbo.scrollRadii, 22, false, true);
    // The shell round the wheel, closed on the bearing side.
    appendCylinder(turbine, Mat4::frameAlongY(turbo.turbineCentre - axis * (0.5F * wt), axis), rt + 5.0F, rt + 5.0F, wt,
                   28, true, false);
    // The exducer, then a cone down to the pipe that follows.
    const auto exducer = turbo.turbineCentre + axis * (0.5F * wt);
    appendCylinder(turbine, Mat4::frameAlongY(exducer, axis), rt + 5.0F, turbo.outletRadius + 1.5F,
                   length(turbo.outlet - exducer), 28, false, false);
    // Bearing housing, between the two backs.
    const auto bearingEnd = turbo.compressorCentre + axis * (0.5F * wk);
    appendCylinder(turbine, Mat4::frameAlongY(bearingEnd, axis), 0.6F * rt + 6.0F, 0.6F * rt + 6.0F,
                   length(exducer - axis * wt - bearingEnd), 20);

    // Compressor: a volute ring, a back plate, a cone to the inducer, and the
    // inducer open to the front so the wheel shows.
    const auto ring = 1.45F * rk;
    const auto tube = turbo.compressorOuterRadius - ring;
    appendTorus(compressor, Mat4::frameAlongY(turbo.compressorCentre, axis), ring, tube, 36, 14);
    appendCylinder(compressor, Mat4::frameAlongY(turbo.compressorCentre, axis), ring, ring, 0.5F * wk, 36, false, true);
    const auto front = turbo.compressorCentre - axis * (0.5F * wk);
    appendCylinder(compressor, Mat4::frameAlongY(turbo.compressorCentre, -axis), ring, rk + 3.0F, 0.5F * wk, 36, false,
                   false);
    appendCylinder(compressor, Mat4::frameAlongY(front, -axis), rk + 3.0F, rk + 5.0F, 0.9F * rk, 36, false, false);
    // The outlet leaves the volute tangentially, upwards where it can.
    auto up = Vec3 { 0.0F, 1.0F, 0.0F } - axis * dot(Vec3 { 0.0F, 1.0F, 0.0F }, axis);
    up = length(up) > 0.3F ? normalise(up) : turbo.radial;
    const auto tangent = normalise(cross(axis, up));
    appendCylinder(compressor, Mat4::frameAlongY(turbo.compressorCentre + up * ring - tangent * tube, tangent), 0.9F * tube,
                   0.9F * tube, 1.4F * rk + tube, 20, false, true);
}

void appendSuperchargerHousing(Mesh& mesh, const SuperchargerPlacement& supercharger) {
    if (!supercharger.placed) return;
    const auto axis = supercharger.axis;
    const auto wheel = supercharger.wheelRadius;
    const auto width = supercharger.width;
    // A volute ring, a back plate towards the throttle, a shroud closing in
    // to the inducer, and the inducer's mouth open so the impeller shows.
    const auto ring = 1.45F * wheel;
    const auto tube = supercharger.outerRadius - ring;
    appendTorus(mesh, Mat4::frameAlongY(supercharger.centre, axis), ring, tube, 40, 14);
    appendCylinder(mesh, Mat4::frameAlongY(supercharger.centre, -axis), ring, ring, 0.5F * width, 40, false, true);
    appendCylinder(mesh, Mat4::frameAlongY(supercharger.centre, axis), ring, wheel + 3.0F, 0.5F * width, 40, false,
                   false);
    const auto front = supercharger.centre + axis * (0.5F * width);
    appendCylinder(mesh, Mat4::frameAlongY(front, axis), wheel + 3.0F, supercharger.eyeRadius,
                   length(supercharger.eye - front), 40, false, false);
    // The charge pipe, from the back of the volute to the throttle.
    appendTube(mesh, { supercharger.chargeOutlet, supercharger.throttleInlet },
               { supercharger.chargeRadius, supercharger.chargeRadius }, 24);
}

void appendTurboWheel(Mesh& mesh, float radius, float width, int blades, float noseSign) {
    const auto nose = noseSign < 0.0F ? -1.0F : 1.0F;
    const Vec3 y { 0.0F, nose, 0.0F };
    // Hub: wide at the back, narrow at the nose; a back plate behind the blades.
    appendCylinder(mesh, Mat4::frameAlongY(y * (-0.5F * width), y), 0.42F * radius, 0.2F * radius, width, 20);
    appendCylinder(mesh, Mat4::frameAlongY(y * (-0.5F * width), y), 0.95F * radius, 0.95F * radius, 0.08F * width, 28);
    // Blades: full radius at the back, 85 % at the nose, leaning round the
    // axis so the rotation reads.
    const std::vector<Vec2> outline { { 0.25F * radius, -0.5F * width }, { radius, -0.5F * width },
                                      { radius, -0.2F * width }, { 0.85F * radius, 0.5F * width },
                                      { 0.25F * radius, 0.5F * width } };
    const auto count = std::max(blades, 3);
    constexpr float lean = 0.45F;
    for (int b = 0; b < count; ++b) {
        const auto angle = 2.0F * pi * static_cast<float>(b) / static_cast<float>(count);
        const Vec3 radial { std::cos(angle), 0.0F, std::sin(angle) };
        const auto around = cross(y, radial);
        const auto along = y * std::cos(lean) + around * std::sin(lean);
        appendConvexExtrusion(mesh, Mat4::basis(radial, along, cross(radial, along), {}), outline,
                              std::max(1.2F, 0.035F * radius));
    }
}

} // namespace enginelab::render
