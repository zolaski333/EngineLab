#pragma once

#include <enginelab/foundation/AcousticSourcePlacement.hpp>
#include <enginelab/foundation/EngineTypes.hpp>
#include <enginelab/render/Math3D.hpp>

#include <array>

namespace enginelab::render {

class EngineModel3D;

/**
 * From the 3-D view's frame (millimetres, +Y up, +Z along the crankshaft) to
 * the sound's (AcousticObserverConfig: metres, +Z up), both about the
 * engine's origin. The sound places its sources schematically (outlets near
 * the origin, radiating along their acoustic axis, the listener 3.5 to 5 m
 * along it), so the frame is turned about the vertical until the drawn
 * tailpipe points along the first exhaust path's acoustic axis: a camera
 * behind the drawn tailpipe hears that outlet on its axis. Distances from the
 * origin are kept.
 */
struct ListenerFrame final {
    double cosine { 1.0 };
    double sine { 0.0 };

    [[nodiscard]] static ListenerFrame of(const EngineModel3D& scene) noexcept;
    [[nodiscard]] AcousticPoint3M acoustic(Vec3 pointMm) const noexcept;
    /** A unit direction of the view, turned into the sound's axes. */
    [[nodiscard]] AcousticPoint3M direction(Vec3 axis) const noexcept;
    /** Where `scene` draws its openings, in the sound's frame: each exhaust
     * outlet's tip and each intake path's mouth with the direction out of it,
     * and the turbo's compressor or the supercharger. */
    [[nodiscard]] AcousticSourcePlacements sources(const EngineModel3D& scene) const;
    /** Two microphones `spacingM` apart along `right` (the camera's right,
     * a unit vector), centred on `eyeMm`: left first. */
    [[nodiscard]] std::array<AcousticPoint3M, 2> microphones(Vec3 eyeMm, Vec3 right,
                                                             double spacingM) const noexcept;
};

} // namespace enginelab::render
