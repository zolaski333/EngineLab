#pragma once

#include <enginelab/foundation/EngineTypes.hpp>
#include <enginelab/render/Mesh.hpp>

#include <cstdint>
#include <vector>

namespace enginelab::render {

/** Where a cylinder's port leaves the head, for routing intake and exhaust ducts. */
struct PortAnchor final {
    std::uint32_t cylinderId {};
    Vec3 position;
    Vec3 direction;
    float diameterMm {};
    /** Inside the head, where the port starts behind the valves. */
    Vec3 inner;
};

/** What a laid-out duct stands for, so it can later be coloured from the solver. */
enum class DuctKind : std::uint8_t {
    /** One component of an exhaust path's graph. */
    exhaustComponent,
    /** A port joined straight to a junction: no authored length. */
    exhaustFeeder,
    intakeRunner,
    intakePlenum,
    intakeThrottle,
    intakeAirbox,
    intakeInletDuct,
};

struct DuctPiece final {
    DuctKind kind { DuctKind::exhaustComponent };
    /** Exhaust or intake path id. */
    std::uint32_t pathId {};
    /** Exhaust component id (for a feeder, the junction it joins); cylinder
        id for a runner. */
    std::uint32_t elementId {};
    ExhaustComponentType componentType { ExhaustComponentType::pipe };
    Mesh mesh;
    /** Centreline as drawn, from inlet to outlet (gas-flow order). Empty for
        boxes (plenum, airbox). */
    std::vector<Vec3> centreline;
    /** Authored length; 0 when the configuration gives none. */
    float authoredLengthMm {};
    /** Framed by the engine camera views (ports, runners, plenum, primaries). */
    bool nearEngine {};
};

/**
 * Lays out every exhaust path of `config` from its component graph (or from
 * the scalar geometry when the path has none), with the authored lengths and
 * diameters: primaries leave the ports and sag until their centreline has the
 * real length, junctions and bodies sit on a trunk running along +Z (towards
 * the flywheel), bodies get the diameter of their authored volume.
 *
 * A pipe that must span more than its authored length is drawn straight and
 * therefore longer; tests measure how often that happens.
 */
[[nodiscard]] std::vector<DuctPiece> layoutExhaust(const EngineConfig& config,
                                                   const std::vector<PortAnchor>& ports,
                                                   float boreMm);

/**
 * Lays out every intake path: runners of their real length and taper from
 * each port to a plenum of the authored volume, then the throttle bores, the
 * airbox volume, the inlet duct and its bellmouth, in the order the gas
 * crosses them. `radial` gathers the runners on a drum around the crank axis.
 */
[[nodiscard]] std::vector<DuctPiece> layoutIntake(const EngineConfig& config,
                                                  const std::vector<PortAnchor>& ports,
                                                  float boreMm, bool radial);

} // namespace enginelab::render
