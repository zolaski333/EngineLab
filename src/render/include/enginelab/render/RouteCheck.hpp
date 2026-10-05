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

} // namespace enginelab::render
