#pragma once

#include <enginelab/render/EngineModel3D.hpp>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace enginelab::render {

enum class RouteIssueKind : std::uint8_t {
    /** Two ducts pass through each other. */
    ductClash,
    /** A duct passes through itself (a loop tighter than its own diameter). */
    selfClash,
    /** A duct passes through the block, a head, the crankcase, the sump, a
        liner, the pulley or the flywheel. */
    engineClash,
    /** A centreline bent tighter than one diameter: no tube bends that way. */
    tightBend,
    /** A pipe drawn more than 3 % longer than authored. */
    stretched,
};

struct RouteIssue final {
    RouteIssueKind kind {};
    /** Index in EngineModel3D::ducts(). */
    int duct { -1 };
    /** The other duct of a duct clash; -1 otherwise. */
    int other { -1 };
    Vec3 where;
    /** Clash: deepest overlap (mm). Bend: tightest centreline radius over the
        diameter. Stretch: drawn over authored length. */
    float value {};
};

struct RouteReport final {
    std::vector<RouteIssue> issues;
    [[nodiscard]] std::size_t count(RouteIssueKind kind) const noexcept;
};

/**
 * What a physical exhaust and intake could not do, measured on the laid-out
 * ducts of `model`. Each duct is a tube of its drawn radius along its
 * centreline, with flat open ends.
 *
 * Joints are not clashes: near an end that plugs into another duct, a port or
 * a box (plenum, airbox), within two radii of that end, overlap is allowed.
 * Siblings converging on one collector are checked like any other pair beyond
 * that, so a mouth too small for its primaries shows. One issue per duct pair
 * (the deepest overlap), per duct and solid, and per tight bend.
 */
[[nodiscard]] RouteReport checkRoutes(const EngineModel3D& model);

/**
 * Whether the exhaust is placed as one is fitted to a vehicle, not only clear
 * of itself (owner 2026-11-18: an X16's four tailpipes 1.2 m out to the
 * sides, a V8's cross-bank primary coiled, both with no route issue). Judged
 * in the vehicle's frame (EngineModel3D::rear(): rearwards, up, across) on
 * the engine's solids (block, heads, crankcase, sump, flywheel, gearbox):
 *
 * - beside: how far an exhaust centreline goes out past the engine's widest
 *   point, beyond a header's 150 mm;
 * - below: under the engine's lowest point, beyond 250 mm (the ground);
 * - above: over the highest exhaust port, beyond two port diameters;
 * - ahead: in front of the engine (or of a motorcycle's forward-facing
 *   ports, three port diameters out), beyond 100 mm;
 * - coiled: exhaust pipes whose centreline turns through more than 450
 *   degrees in all (a header that drops and runs back turns 180-270);
 * - astray: outlets not pointing rearwards (within 45 degrees);
 * - asymmetry: where the exhaust ports are mirror images across the crank's
 *   vertical plane (a V, a flat, an X), on twin pipes (one type and length,
 *   starting at each other's images: the primaries of mirror ports, what
 *   follows two mirrored collectors), the 90th percentile distance from a
 *   mirrored point of one to the nearest point of the other; 0 otherwise. A
 *   single turbo on one side is the design's, not measured.
 */
struct PlausibilityReport final {
    float besideMm {};
    float belowMm {};
    float aboveMm {};
    float aheadMm {};
    std::size_t coiled {};
    std::size_t astray {};
    float asymmetryMm {};
    /** Whether the ports are mirror images, so `asymmetryMm` was measured. */
    bool mirrored {};
    /** Exhaust pipes drawn shorter than authored (DuctPiece::foldedMm). */
    std::size_t folded {};
    /** The exhaust ducts concerned (indices in EngineModel3D::ducts()). */
    std::vector<int> offenders;
    [[nodiscard]] bool plausible(float asymmetryAllowanceMm = 40.0F) const noexcept {
        return besideMm <= 0.0F && belowMm <= 0.0F && aboveMm <= 0.0F && aheadMm <= 0.0F && coiled == 0U
            && astray == 0U && asymmetryMm <= asymmetryAllowanceMm;
    }
};

[[nodiscard]] PlausibilityReport checkPlausibility(const EngineModel3D& model);

} // namespace enginelab::render
