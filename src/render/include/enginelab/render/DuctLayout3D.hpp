#pragma once

#include <enginelab/foundation/EngineTypes.hpp>
#include <enginelab/render/Mesh.hpp>
#include <enginelab/render/RouteSolver.hpp>

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
    /** Outer radius at each centreline point; empty for boxes. */
    std::vector<float> radii;
    /** Authored length; 0 when the configuration gives none. */
    float authoredLengthMm {};
    /** Framed by the engine camera views (ports, runners, plenum, primaries). */
    bool nearEngine {};
    /** Where the gas meets the atmosphere: an exhaust outlet's tip, or the
        mouth of an intake path's outermost piece (bellmouth, airbox inlet,
        throttle mouths or supercharger eye). The sound of that opening
        radiates from `openEnd` along the unit `openAxis`, pointing out. */
    bool opensToAtmosphere {};
    Vec3 openEnd;
    Vec3 openAxis { 0.0F, 1.0F, 0.0F };
};

/**
 * Where a turbocharger sits on the exhaust. The simulator has no turbine in
 * its gas network (it applies the turbine as a restriction on every outlet),
 * so the picture puts it where a real one sits: bolted to the collector of
 * the first path, the trunk resuming at its turbine outlet.
 *
 * The shaft runs along the crankshaft (+Z). Gas leaves the collector, bends
 * outwards and enters the turbine volute tangentially; it leaves through the
 * exducer along +Z. The compressor sits on the shaft on the other side.
 */
struct TurboPlacement final {
    bool placed {};
    std::uint32_t pathId {};
    /** The collector it is bolted to. */
    std::uint32_t collectorId {};
    /** Gas path from the collector's outlet round the volute: centreline and
        outer radius, in gas-flow order. */
    std::vector<Vec3> scroll;
    std::vector<float> scrollRadii;
    /** Unit shaft axis (+Z), and the unit radial pointing from the shaft to
        where the gas enters the volute. */
    Vec3 axis { 0.0F, 0.0F, 1.0F };
    Vec3 radial { 1.0F, 0.0F, 0.0F };
    Vec3 turbineCentre;
    Vec3 compressorCentre;
    float turbineWheelRadius {};
    float compressorWheelRadius {};
    /** Axial widths of the turbine housing's centre and of the compressor's. */
    float turbineWidth {};
    float compressorWidth {};
    /** Outer radius of the compressor volute. */
    float compressorOuterRadius {};
    /** Where the exhaust resumes, past the exducer, and its radius there. */
    Vec3 outlet;
    float outletRadius {};
};

/**
 * Where a geared supercharger sits on the intake. The simulator applies it as
 * a pressure ratio on the intake, with no compressor in its gas network, so
 * the picture puts a centrifugal one where the air passes it: the charge
 * leaves the back of its volute through a straight pipe to the throttle, and
 * the airbox or inlet duct feeds its eye.
 *
 * The impeller turns about `axis`, the direction the air comes from at the
 * throttle, offset away from the engine by the volute's radius so that the
 * charge pipe runs straight along the throttle's axis.
 */
struct SuperchargerPlacement final {
    bool placed {};
    std::uint32_t pathId {};
    /** Unit axis, from the impeller's back plate towards its eye. */
    Vec3 axis { 0.0F, 0.0F, -1.0F };
    /** Unit radial from the impeller's axis towards the charge outlet. */
    Vec3 radial { 1.0F, 0.0F, 0.0F };
    Vec3 centre;
    float wheelRadius {};
    /** Axial width of the impeller's housing, and its outer radius. */
    float width {};
    float outerRadius {};
    /** The charge pipe, from the back of the volute to the throttle. */
    Vec3 chargeOutlet;
    Vec3 throttleInlet;
    float chargeRadius {};
    /** The inducer's mouth, where the airbox or inlet duct joins. */
    Vec3 eye;
    float eyeRadius {};
};

/** The solid a placed supercharger occupies, for the router and the route
    check. */
[[nodiscard]] std::vector<EngineSolid> superchargerSolids(const SuperchargerPlacement& supercharger);

/** The supercharger housing (volute, back plate, inducer) and its charge
    pipe, in world coordinates. */
void appendSuperchargerHousing(Mesh& mesh, const SuperchargerPlacement& supercharger);

/**
 * Lays out every exhaust path of `config` from its component graph (or from
 * the scalar geometry when the path has none), with the authored lengths and
 * diameters: junctions and bodies sit on a trunk running along +Z (towards
 * the flywheel), bodies get the diameter of their authored volume, and a
 * collector's mouth is wide enough for all its inputs side by side.
 *
 * Pipes between ports and junctions are then bent together by relaxRoutes:
 * each keeps its length, enters its junction along the trunk, and stays clear
 * of the other pipes, of `avoid` (the intake) and of `solids` (the engine).
 * A pipe that must span more than its authored length is drawn straight and
 * therefore longer; tests measure how often that happens.
 *
 * With `turbo` given and a turbocharger configured, the turbo is placed after
 * the first path's collector (see TurboPlacement) and the pipes are routed
 * clear of it.
 */
[[nodiscard]] std::vector<DuctPiece> layoutExhaust(const EngineConfig& config,
                                                   const std::vector<PortAnchor>& ports,
                                                   float boreMm,
                                                   const std::vector<RouteTube>& avoid = {},
                                                   const std::vector<EngineSolid>& solids = {},
                                                   TurboPlacement* turbo = nullptr);

/** The solids a placed turbo occupies (turbine housing centre, bearing
    housing, compressor), for the router and the route check. */
[[nodiscard]] std::vector<EngineSolid> turboSolids(const TurboPlacement& turbo);

/** The turbine housing (inlet bend, volute, exducer and bearing housing) and
    the compressor housing (volute, open inducer, outlet stub), in world
    coordinates. */
void appendTurboHousings(Mesh& turbine, Mesh& compressor, const TurboPlacement& turbo);

/** A bladed wheel round local +Y, centred on the origin: full radius at the
    back, its nose (the turbine's exducer, the compressor's inducer) towards
    `noseSign` × Y. */
void appendTurboWheel(Mesh& mesh, float radius, float width, int blades, float noseSign);

/**
 * Lays out every intake path: runners of their real length and taper from
 * each port to a plenum of the authored volume, then the throttle bores, the
 * airbox volume, the inlet duct and its bellmouth, in the order the gas
 * crosses them. `radial` gathers the runners on a drum around the crank axis.
 * The runners are bent together by relaxRoutes, clear of one another, of the
 * plenums and of `solids`.
 *
 * With `supercharger` given and a supercharger configured, the first path
 * with a single throttle gets it between its throttle and its airbox (see
 * SuperchargerPlacement).
 */
[[nodiscard]] std::vector<DuctPiece> layoutIntake(const EngineConfig& config,
                                                  const std::vector<PortAnchor>& ports,
                                                  float boreMm, bool radial,
                                                  const std::vector<EngineSolid>& solids = {},
                                                  SuperchargerPlacement* supercharger = nullptr);

} // namespace enginelab::render
