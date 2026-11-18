#pragma once

#include <enginelab/render/RouteSolver.hpp>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace enginelab::render {

/** Half sizes of the axis-aligned box holding `solid`. */
[[nodiscard]] Vec3 solidHalfExtent(const EngineSolid& solid) noexcept;

/** Points well inside `solid` (its centre, and halfway and nine tenths of the
    way to each corner or rim point): where it sits in another solid. */
[[nodiscard]] std::vector<Vec3> solidSamples(const EngineSolid& solid);

/**
 * The room round the engine, as a grid of cubic cells, for a duct that has
 * to go round it: a cell is blocked when its centre lies within the
 * clearance of a solid or of an obstacle tube (the clearance is the duct's
 * radius plus a gap, so a free cell is somewhere the duct's centreline may
 * pass).
 *
 * The relaxation that bends ducts together (relaxRoutes) only moves a guess
 * locally: a guess through the crankcase stays there. This finds a guess
 * that goes round it, and the shortest distance a duct needs from one place
 * to another, which is what decides where a collector can sit (owner
 * 2026-11-18: an X12's primaries ran through its block to a collector in
 * its gearbox).
 */
class FreeSpace final {
public:
    FreeSpace(Vec3 minimum, Vec3 maximum, float cellMm, const std::vector<EngineSolid>& solids,
              const std::vector<RouteTube>& tubes, float clearanceMm);

    [[nodiscard]] bool blocked(Vec3 point) const noexcept;
    /** Every point of the segment, sampled at half a cell, is free. */
    [[nodiscard]] bool clear(Vec3 a, Vec3 b) const noexcept;

    /** Shortest free distances from one place to every cell. */
    struct Field final {
        std::vector<float> distance;
        std::vector<std::int32_t> previous;
        /** Where the walk starts: the first free point from the origin along
            the way out, and the straight run to it. */
        Vec3 start;
        float startRun {};
        bool valid {};
    };
    /** From `origin`, leaving along the unit `leave` until the first free
        cell (at most `maximumRunMm`): a port sits in its head. */
    [[nodiscard]] Field reach(Vec3 origin, Vec3 leave, float maximumRunMm) const;
    /** Free distance from the field's origin to `target`, arriving along the
        unit `arrive` (the target may be blocked: it is reached from the first
        free point behind it), straight runs included; infinite when there is
        no way. */
    [[nodiscard]] float distance(const Field& field, Vec3 target, Vec3 arrive, float maximumRunMm) const;
    /** The way itself, from the origin to `target`, shortened to the
        straightest free line through the cells. Empty when there is none. */
    [[nodiscard]] std::vector<Vec3> path(const Field& field, Vec3 origin, Vec3 target, Vec3 arrive,
                                         float maximumRunMm) const;

private:
    [[nodiscard]] std::int32_t cellOf(Vec3 point) const noexcept;
    [[nodiscard]] Vec3 centreOf(std::int32_t cell) const noexcept;
    /** The first free point from `origin` along `direction`, and how far. */
    [[nodiscard]] bool firstFree(Vec3 origin, Vec3 direction, float maximumRunMm, Vec3& point, float& run) const;

    Vec3 minimum_;
    float cell_ {};
    int nx_ {}, ny_ {}, nz_ {};
    std::vector<std::uint8_t> blocked_;
};

} // namespace enginelab::render
