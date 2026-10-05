#pragma once

#include <enginelab/render/Mesh.hpp>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace enginelab::render {

/** A solid of the engine that ducts stay out of: an oriented box, or a
    cylinder whose axis is `axes[1]`. */
struct EngineSolid final {
    enum class Shape : std::uint8_t { box, cylinder };
    Shape shape { Shape::box };
    Vec3 centre;
    /** Orthonormal local axes. */
    Vec3 axes[3] { { 1.0F, 0.0F, 0.0F }, { 0.0F, 1.0F, 0.0F }, { 0.0F, 0.0F, 1.0F } };
    /** Box: half sizes along the axes. Cylinder: radius, half length, unused. */
    Vec3 half;
};

/** Distance from `point` to the surface of `solid`: negative inside. */
[[nodiscard]] float signedDistance(const EngineSolid& solid, Vec3 point) noexcept;

/** An axis-aligned box solid spanning `minimum` to `maximum`. */
[[nodiscard]] EngineSolid boxBetween(Vec3 minimum, Vec3 maximum) noexcept;

/** A tube: a centreline with the outer radius at each point. Its ends are
    open and flat. */
struct RouteTube final {
    std::vector<Vec3> points;
    std::vector<float> radii;
};

/** How deep a sphere sits in a tube, and the way out. */
struct TubeContact final {
    /** Positive when they overlap (mm). */
    float depth;
    /** Unit direction that frees the sphere. */
    Vec3 normal;
};

/**
 * Overlap of the sphere of radius `reach` at `q` with the tube through
 * `points`: radius `radii[k]` at each point, or `radius` when `radii` is null.
 * The tube's ends are open and flat: behind an end plane it is only its end
 * disc. Segments [skipFrom, skipTo) are left out.
 */
[[nodiscard]] TubeContact tubeContact(Vec3 q, float reach, const std::vector<Vec3>& points,
                                      const std::vector<float>* radii, float radius = 0.0F,
                                      std::size_t skipFrom = 0, std::size_t skipTo = 0);

/** A pipe whose path the router chooses. */
struct RoutedPipe final {
    /** In: a first guess from the start to the end. Out: the route, in
        segments of equal length. */
    std::vector<Vec3> points;
    /** Largest outer radius along the pipe. */
    float radius {};
    /** Centreline length to keep; raised to what the span needs when the pipe
        is too short to join its ends. */
    float lengthMm {};
    /** Unit directions it leaves the start along and arrives at the end along. */
    Vec3 startDirection;
    Vec3 endDirection;
    /** Straight runs held fixed at each end (a port spigot, a collector slot). */
    float startLead {};
    float endLead {};
};

struct RouteSettings final {
    /** Tightest centreline bend radius, in pipe diameters. */
    float minimumBendDiameters { 1.25F };
    /** Clearance kept between tube walls and from the engine. */
    float gapMm { 3.0F };
    /** Contact passes, at most: the relaxation stops once nothing moves. */
    int iterations { 200 };
    /** Bend and length passes after each contact pass. */
    int shapePasses { 40 };
};

/**
 * Bends every pipe of `pipes` so that, together, none passes through another,
 * through itself, through a tube of `obstacles` or through a solid, none bends
 * tighter than the minimum radius, and each keeps its length and its straight
 * ends. A pipe too short to span its ends is left as given.
 *
 * Position-based relaxation: each pipe is a chain of equal segments; contacts,
 * then bends and segment lengths, are projected in turn until nothing moves or
 * the pass budget runs out, in a fixed order, so the result is deterministic.
 * There is no stiffness: where every constraint already holds, a pipe stays
 * where its guess put it (measured: a pull towards straight acts as tension
 * and gathers the curvature at the fixed ends).
 */
void relaxRoutes(std::vector<RoutedPipe>& pipes, const std::vector<RouteTube>& obstacles,
                 const std::vector<EngineSolid>& solids, const RouteSettings& settings = {});

} // namespace enginelab::render
