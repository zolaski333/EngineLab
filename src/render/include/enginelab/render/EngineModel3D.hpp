#pragma once

#include <enginelab/foundation/EngineTypes.hpp>
#include <enginelab/physics/MechanicalKinematics.hpp>
#include <enginelab/render/DuctLayout3D.hpp>
#include <enginelab/render/Mesh.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

namespace enginelab::render {

/** Surface families the renderer knows how to shade. */
enum class SceneMaterial : std::uint8_t {
    steel,
    forged,
    piston,
    intakeValve,
    exhaustValve,
    blockShell,
    headShell,
    intakeFlow,
    exhaustPipe,
    flame,
    /** Turbine and compressor housings: translucent in X-ray, so the wheels show. */
    turboHousing,
    /** An injector's fuel spray, the nitrous kit's: translucent. */
    spray,
};
inline constexpr std::size_t sceneMaterialCount = 12;

/** Bits for the view layers (All / Combustion / Mechanical / Gas flow). */
enum SceneLayer : std::uint8_t {
    layerStructure = 1U << 0U,
    layerMechanism = 1U << 1U,
    layerValves = 1U << 2U,
    layerCombustion = 1U << 3U,
    layerIntake = 1U << 4U,
    layerExhaust = 1U << 5U,
};

/** One draw: a mesh with a material, drawn once per frame with a transform. */
struct ScenePart final {
    Mesh mesh;
    SceneMaterial material { SceneMaterial::steel };
    std::uint8_t layers { layerMechanism };
    /** For a duct, where each vertex sits along its centreline: 0 at the
        inlet, 1 at the outlet. Empty for every other part. */
    std::vector<float> stations {};
};

struct SceneInstance final {
    std::uint16_t part {};
    Mat4 transform;
    /** Flame strength or flow glow, 0..1; 1 for plain parts. */
    float intensity { 1.0F };
};

/** 32 ones: the default of a per-cylinder multiplier. */
[[nodiscard]] constexpr std::array<float, 32> unitPerCylinder() noexcept {
    std::array<float, 32> ones {};
    for (auto& one : ones) one = 1.0F;
    return ones;
}

struct ScenePoseInput final {
    /** Primary crank angle over the 720-degree domain, as the simulator reports it. */
    double crankAngleDegrees {};
    /** Strength of each cylinder's current burn, 0..1 (0 = misfire or cut). */
    std::array<float, 32> combustion {};
    /** Engine speed and throttle. */
    double rpm {};
    double throttle {};
    /** VTEC: bit i set while cylinder i runs its high cam (EngineState). */
    std::uint32_t vtecHighCylinderMask {};
    /** Strength of the flame at the exhaust outlets, 0..1 (afterfire). */
    float afterfire {};
    /** Turbocharger shaft angle, degrees. */
    double turboShaftDegrees {};
    /** Gearbox input shaft angle, degrees, never wrapped: each output gear
        turns at its own ratio of it. */
    double gearboxInputDegrees {};
    /** The engaged gear: 0 for first, -1 for neutral, -2 for reverse. */
    int engagedGear { -1 };
    /** A two-stroke's power valve, 0 shut to 1 open. */
    double powerValveOpening { 1.0 };
    /** How many cylinders the engine brake holds (EngineState): the first
        ones, their exhaust valve cracked open around firing TDC. */
    std::uint32_t engineBrakeCylinders { 0 };
    /** The picture is strobed (slowed down or frozen): what happens once a
        cycle (the injector's spray, the spark) is drawn only then, so
        nothing flickers in the live view. */
    bool cycleDetail { false };
    /** The ECU's spark advance, degrees before firing top dead centre
        (EngineState): where each cylinder's flame kernel starts (with its
        ignition offset) and, strobed, where its plug sparks. For a diesel,
        the start of injection. */
    double ignitionAdvanceDegrees { 15.0 };
    /** The ECU fires the plugs (spark enabled, no cut). */
    bool sparkEnabled { false };
    /** Each cylinder's injector open time over its cycle (CylinderState). */
    std::array<float, 32> injectorDuty {};
    /** Each cylinder's variable cam timing (CylinderState): crank degrees its
        intake and exhaust lobes are advanced, and its lift multiplier. */
    std::array<float, 32> intakeCamAdvanceDegrees {};
    std::array<float, 32> exhaustCamAdvanceDegrees {};
    std::array<float, 32> valveLiftMultiplier { unitPerCylinder() };
    /** The turbochargers' common wastegate, 0 shut to 1 open, and the flow
        through their blow-off valve, kg/s (EngineState). */
    double wastegateOpening {};
    double blowOffMassFlowKgPerSecond {};
    /** The active exhaust valves' butterfly stroke, 0 shut to 1 open. */
    double exhaustValveStroke { 1.0 };
    /** The nitrous kit's flow, kg/s of liquid nitrous (0 while it does not spray). */
    double nitrousMassFlowKgPerSecond {};
};

struct SceneBounds final {
    Vec3 minimum;
    Vec3 maximum;

    [[nodiscard]] Vec3 centre() const noexcept { return (minimum + maximum) * 0.5F; }
    [[nodiscard]] float diagonal() const noexcept { return length(maximum - minimum); }
};

/** A laid-out intake or exhaust duct and the part that draws it. */
struct SceneDuct final {
    DuctKind kind { DuctKind::exhaustComponent };
    std::uint32_t pathId {};
    std::uint32_t elementId {};
    ExhaustComponentType componentType { ExhaustComponentType::pipe };
    std::uint16_t part {};
    std::vector<Vec3> centreline;
    float authoredLengthMm {};
    /** Outer radius at each centreline point; empty for boxes. */
    std::vector<float> radii;
    /** DuctPiece::opensToAtmosphere, openEnd and openAxis. */
    bool opensToAtmosphere {};
    Vec3 openEnd;
    Vec3 openAxis { 0.0F, 1.0F, 0.0F };
    /** DuctPiece::foldedMm: how much shorter than authored it is drawn. */
    float foldedMm {};
    /** DuctPiece::bores: a throttle body's plates or slides. */
    std::vector<ThrottleBore> bores {};
};

/** What a part is, for the part inspector. */
enum class PartRole : std::uint8_t {
    block,
    head,
    liner,
    crankshaft,
    piston,
    rod,
    intakeValve,
    exhaustValve,
    combustion,
    intakePorts,
    exhaustPorts,
    duct,
    afterfire,
    turbo,
    supercharger,
    /** A cylinder's fuel injector: in its intake port, or in the head for
        direct injection. */
    injector,
    /** The gearbox: case, shafts, gear pairs and selector. */
    gearbox,
    /** A two-stroke's power valve, the blade in its exhaust port's roof. */
    powerValve,
    /** A camshaft with its lobes and sprocket, the timing chain, a VTEC pin. */
    camshaft,
    /** The spark plugs. */
    sparkPlug,
    /** A throttle plate, or a slide carburettor's slide and needle. */
    throttlePlate,
    /** The nitrous kit: bottle, line, plate and its spray. */
    nitrous,
};

struct PartIdentity final {
    PartRole role { PartRole::block };
    /** Index in the configuration's cylinders; -1 when not one cylinder's. */
    int cylinder { -1 };
    /** Index in EngineModel3D::ducts(); -1 when not a duct. */
    int duct { -1 };
    /** Index in EngineModel3D::turbos(); -1 when not a turbocharger. */
    int turbo { -1 };
    /** A camshaft's: true for the exhaust cam. */
    bool exhaust {};
};

/**
 * 3-D engine geometry built from an EngineConfig, in millimetres.
 *
 * World axes: +Y up, +Z along the crankshaft. The simulator resolves the crank
 * mechanism in its own XY plane with Y pointing down the cylinder; the world
 * plane is that plane turned by 180 degrees, so the geometry is not mirrored.
 * Every moving part is placed with evaluateCylinderKinematics(), the same
 * function the simulator uses, so the picture matches the physics for every
 * layout (inline, V, flat, radial, articulated rods).
 */
class EngineModel3D final {
public:
    explicit EngineModel3D(EngineConfig config);

    [[nodiscard]] const std::vector<ScenePart>& parts() const noexcept { return parts_; }
    /** The engine with its ports, runners, plenum and primaries: what the
        engine camera views frame. */
    [[nodiscard]] const SceneBounds& bounds() const noexcept { return bounds_; }
    /** Everything, down to the last exhaust outlet and the air intake mouth. */
    [[nodiscard]] const SceneBounds& systemBounds() const noexcept { return systemBounds_; }
    [[nodiscard]] const std::vector<SceneDuct>& ducts() const noexcept { return ducts_; }
    [[nodiscard]] std::size_t cylinderCount() const noexcept { return cylinders_.size(); }
    [[nodiscard]] const EngineConfig& config() const noexcept { return config_; }
    [[nodiscard]] const std::vector<PortAnchor>& intakePorts() const noexcept { return intakePorts_; }
    [[nodiscard]] const std::vector<PortAnchor>& exhaustPorts() const noexcept { return exhaustPorts_; }
    /** Block, heads, crankcase, sump, liners, pulley, flywheel and gearbox, as simple
        solids the ducts must stay out of. */
    [[nodiscard]] const std::vector<EngineSolid>& solids() const noexcept { return solids_; }
    /** How many of solids(), from the first, are the engine's and its
        gearbox's: the turbochargers' and the supercharger's follow. */
    [[nodiscard]] std::size_t engineSolidCount() const noexcept { return engineSolids_; }
    /** The turbochargers as drawn, one per unit an exhaust path feeds. Their
        solids are at the end of solids(). */
    [[nodiscard]] const std::vector<TurboPlacement>& turbos() const noexcept { return turbos_; }
    /** The first turbocharger as drawn; `placed` is false without one. */
    [[nodiscard]] const TurboPlacement& turbo() const noexcept {
        static const TurboPlacement none {};
        return turbos_.empty() ? none : turbos_.front();
    }
    /** Cylinder `index`'s injector: its part, nozzle tip and spray direction. */
    /** Cylinder `index`'s power valve blade; 0xFFFF without one. */
    [[nodiscard]] std::uint16_t powerValvePart(std::size_t index) const noexcept {
        return index < cylinders_.size() ? cylinders_[index].powerValvePart : std::uint16_t { 0xFFFFU };
    }
    /** Cylinder `index`'s combustion chamber (its flame); 0xFFFF past the end. */
    [[nodiscard]] std::uint16_t flamePart(std::size_t index) const noexcept {
        return index < cylinders_.size() ? cylinders_[index].flamePart : std::uint16_t { 0xFFFFU };
    }
    /** Cylinder `index`'s exhaust valve; 0xFFFF past the end. */
    [[nodiscard]] std::uint16_t exhaustValvePart(std::size_t index) const noexcept {
        return index < cylinders_.size() ? cylinders_[index].exhaustValvePart : std::uint16_t { 0xFFFFU };
    }
    [[nodiscard]] std::uint16_t injectorPart(std::size_t index) const noexcept {
        return index < cylinders_.size() ? cylinders_[index].injectorPart : std::uint16_t { 0xFFFFU };
    }
    [[nodiscard]] Vec3 injectorTip(std::size_t index) const noexcept {
        return index < cylinders_.size() ? cylinders_[index].injectorTip : Vec3 {};
    }
    [[nodiscard]] Vec3 injectorAim(std::size_t index) const noexcept {
        return index < cylinders_.size() ? cylinders_[index].injectorAim : Vec3 {};
    }
    /** The camshafts as drawn: an intake and an exhaust per bank of a
        poppet-valve four-stroke (none on a radial, a two-stroke or a rotary). */
    struct CamshaftPlacement final {
        std::uint16_t part {};
        /** A point on its axis (it runs along +Z), z = 0. */
        Vec3 centre;
        bool exhaust {};
        /** The bank's first cylinder: its cam timing turns the shaft. */
        std::size_t cylinder {};
    };
    [[nodiscard]] const std::vector<CamshaftPlacement>& camshafts() const noexcept { return camshafts_; }
    /** A cam's turn about its axis from where it is drawn, radians about +Z,
        at a crank angle and a crank-degree advance of its lobes: half the
        crank's turn, the same way round. */
    [[nodiscard]] float camshaftRadians(double crankAngleDegrees, double advanceDegrees) const noexcept;
    /** Cylinder `index`'s spark (a point at its plug's electrode) and its
        injector's spray; 0xFFFF without one. */
    [[nodiscard]] std::uint16_t sparkPart(std::size_t index) const noexcept {
        return index < cylinders_.size() ? cylinders_[index].sparkPart : std::uint16_t { 0xFFFFU };
    }
    [[nodiscard]] std::uint16_t sprayPart(std::size_t index) const noexcept {
        return index < cylinders_.size() ? cylinders_[index].sprayPart : std::uint16_t { 0xFFFFU };
    }
    /** Cylinder `index`'s VTEC lock pin; 0xFFFF without one. */
    [[nodiscard]] std::uint16_t vtecPinPart(std::size_t index) const noexcept {
        return index < cylinders_.size() ? cylinders_[index].vtecPinPart : std::uint16_t { 0xFFFFU };
    }
    /** A part one control turns or slides: a throttle plate or slide, an
        active exhaust valve's butterfly, a wastegate flap, a blow-off
        valve's piston, the nitrous spray. */
    struct ControlPart final {
        enum class Kind : std::uint8_t { throttlePlate, throttleSlide, exhaustValve, wastegate, blowOff, nitrousSpray };
        Kind kind { Kind::throttlePlate };
        std::uint16_t part {};
        /** Index in ducts() (a throttle body, a runner, an exhaust part); -1
            otherwise. */
        int duct { -1 };
        /** Index in turbos() for a wastegate or a blow-off valve; -1 otherwise. */
        int turbo { -1 };
        /** A plate: its centre, the unit flow direction and its spindle.
            A flap: its hinge, the face's normal and the hinge axis, `size`
            its radius. A slide, a piston or a spray: where it sits shut
            (`centre`) and the unit way it opens (`normal`), `size` its travel
            or reach. */
        Vec3 centre;
        Vec3 normal { 0.0F, 1.0F, 0.0F };
        Vec3 hinge { 1.0F, 0.0F, 0.0F };
        float size {};
        /** The simulator holds it wide open (an individual-throttle engine's
            bores upstream of its runners' own plates). */
        bool heldOpen {};
    };
    [[nodiscard]] const std::vector<ControlPart>& controlParts() const noexcept { return controlParts_; }
    /** A throttle plate's angle off shut, radians, at a throttle opening 0-1:
        a few degrees at rest (a plate is set off square so it cannot jam in
        its bore), 90 wide open. */
    [[nodiscard]] static float throttlePlateRadians(double throttle) noexcept;
    /** The supercharger as drawn; `placed` is false without one. Its solid is
        in solids(), after the engine's. */
    [[nodiscard]] const SuperchargerPlacement& supercharger() const noexcept { return supercharger_; }
    /** The impeller's angle about its axis at a crank angle: the crank's
        times the drive ratio, degrees. */
    [[nodiscard]] double superchargerImpellerDegrees(double crankAngleDegrees) const noexcept;

    /** The gearbox as drawn; `placed` is false with fewer than two gears. */
    struct GearboxPlacement final {
        bool placed {};
        /** A point on each shaft axis (both run along +Z), z = 0. */
        Vec3 inputAxis;
        Vec3 outputAxis;
        /** Teeth of each pair, input then output, and where the pair sits on Z. */
        std::vector<int> inputTeeth;
        std::vector<int> outputTeeth;
        std::vector<float> pairZ;
    };
    [[nodiscard]] const GearboxPlacement& gearbox() const noexcept { return gearbox_; }
    /** The way the vehicle's rear lies from the engine, horizontal and unit:
        a car's in-line engine has its gearbox and its exhaust's run along +Z;
        a motorcycle's crank lies across the frame, the rear the side its
        gearbox stands on (+X or -X). The exhaust is laid out towards it. */
    [[nodiscard]] Vec3 rear() const noexcept { return rear_; }
    /** Output gear `pair`'s angle about its shaft, radians, for an input shaft
        angle: it turns the other way at the pair's tooth ratio. */
    [[nodiscard]] float outputGearRadians(std::size_t pair, double inputDegrees) const noexcept;

    /** Cycle phase of one cylinder: 0 = firing TDC, 360 = overlap TDC. */
    [[nodiscard]] double cyclePhaseDegrees(std::size_t cylinder, double crankAngleDegrees) const noexcept;

    /** Writes one instance per part. `out` is cleared, never shrunk, so a
        reserved vector does not allocate. */
    void pose(const ScenePoseInput&, std::vector<SceneInstance>& out) const;

    /** Positions used by tests: the wrist pin, crank pin and deck centre of a
        cylinder at a crank angle, in world millimetres. */
    struct CylinderProbe final {
        Vec3 wristPin;
        Vec3 bigEnd;
        Vec3 crownCentre;
        Vec3 deckCentre;
        Vec3 axis;
        float rodLengthMm {};
    };
    [[nodiscard]] CylinderProbe probe(std::size_t cylinder, double crankAngleDegrees) const noexcept;
    /** A main journal drawn on a crankshaft, along +Z (world mm): tests check
        that no rod's big end swings through one. */
    struct JournalSpan final {
        float zStart {};
        float zEnd {};
        float radius {};
    };
    [[nodiscard]] const std::vector<JournalSpan>& mainJournals() const noexcept { return mainJournals_; }

    [[nodiscard]] PartIdentity identify(std::uint16_t part) const noexcept;
    /** The camshafts that drive a cylinder's valves (its bank's, or the engine's). */
    [[nodiscard]] const CamshaftConfig& camshaftsOf(std::size_t cylinder) const noexcept {
        return *cylinders_[std::min(cylinder, cylinders_.size() - 1U)].cams;
    }

private:
    struct Cylinder final {
        Vec3 axis;
        Vec3 exhaustSide;
        float z {};
        float bore {};
        float compressionHeight {};
        Vec3 deckCentre;
        Vec3 chamberCentre;
        double crankOffsetDegrees {};
        const CamshaftConfig* cams {};
        Vec3 intakeValveAxis;
        Vec3 exhaustValveAxis;
        std::uint16_t pistonPart {};
        std::uint16_t rodPart {};
        std::uint16_t intakeValvePart {};
        std::uint16_t exhaustValvePart {};
        std::uint16_t flamePart {};
        std::uint16_t linerPart { 0xFFFFU };
        std::uint16_t runnerPart { 0xFFFFU };
        std::uint16_t injectorPart { 0xFFFFU };
        std::uint16_t sparkPart { 0xFFFFU };
        std::uint16_t sprayPart { 0xFFFFU };
        /** A VTEC lock pin, and how far it slides to lock the high lobe in. */
        std::uint16_t vtecPinPart { 0xFFFFU };
        Vec3 vtecPinSlide;
        /** Where the spark jumps: the plug's electrode, just inside the head. */
        Vec3 sparkPoint;
        /** The spray's reach from the injector tip with the injector open for
            its whole window. */
        float sprayReach {};
        /** A two-stroke's power valve blade, and how far it drops to shut. */
        std::uint16_t powerValvePart { 0xFFFFU };
        Vec3 powerValveDrop;
        /** A two-stroke's: its crank's centre, level with it, and the round
            crankcase, barrel and port windows the structure is drawn round. */
        Vec3 crankCentre;
        Vec3 exhaustWindowCentre;
        /** A two-stroke: the exhaust port's roof, on the cylinder axis. */
        Vec3 exhaustPortRoof;
        Vec3 intakeWindowCentre;
        Vec3 transferWindowCentre;
        Vec3 transferFacing;
        float barrelRadius {};
        float barrelBottom {};
        /** Where the injector sprays from, and the unit direction it sprays. */
        Vec3 injectorTip;
        Vec3 injectorAim;
    };
    struct Throw final {
        Vec3 origin;
        std::size_t referenceCylinder {};
        std::uint16_t part {};
    };

    [[nodiscard]] Vec3 world(double simX, double simY, float z) const noexcept;
    [[nodiscard]] float throwRotation(const Throw&, double crankAngleDegrees) const noexcept;
    std::uint16_t addPart(Mesh, SceneMaterial, std::uint8_t layers);
    void buildCylinders();
    /** A rotary's housings, rotors and eccentric shaft, in place of the
        cylinders, crank and block. */
    void buildRotary();
    void poseRotary(const ScenePoseInput&, std::vector<SceneInstance>&) const;
    void buildCrankshaft();
    void buildStructure();
    void buildTwoStrokeStructure();
    void buildDucts();
    void buildTurbo();
    void buildSupercharger();
    /** Without `draw`, only its solids (before the ducts); with it, its parts. */
    void buildGearbox(bool draw);
    /** What the cycle or a control moves, drawn after the layout (no solid
        of theirs moves a duct): cams and their drive, plugs and sparks,
        sprays, throttle plates, exhaust valves, turbo valves, nitrous. */
    void buildValvetrain();
    void buildIgnitionAndInjection();
    void buildControls();
    void buildNitrous();
    void poseCycleParts(const ScenePoseInput&, std::vector<SceneInstance>&) const;
    void poseControls(const ScenePoseInput&, std::vector<SceneInstance>&) const;
    void computeBounds();

    EngineConfig config_;
    EngineKinematicsReference reference_ {};
    std::vector<ScenePart> parts_;
    std::vector<Cylinder> cylinders_;
    std::vector<Throw> throws_;
    std::vector<EngineSolid> solids_;
    std::size_t engineSolids_ {};
    std::vector<std::uint16_t> rotatingParts_;
    std::vector<std::size_t> rotatingThrow_;
    std::vector<PortAnchor> intakePorts_;
    std::vector<PortAnchor> exhaustPorts_;
    std::vector<SceneDuct> ducts_;
    std::vector<TurboPlacement> turbos_;
    /** Per turbo: turbine housing, compressor housing, turbine wheel,
        compressor wheel. */
    std::vector<std::array<std::uint16_t, 4>> turboParts_;
    SuperchargerPlacement supercharger_;
    /** Housing (with the charge pipe) and impeller. */
    std::array<std::uint16_t, 2> superchargerParts_ { 0xFFFFU, 0xFFFFU };
    GearboxPlacement gearbox_;
    Vec3 rear_ { 0.0F, 0.0F, 1.0F };
    /** Case, input shaft with its gears, output shaft, selector collar. */
    std::array<std::uint16_t, 4> gearboxParts_ { 0xFFFFU, 0xFFFFU, 0xFFFFU, 0xFFFFU };
    std::vector<std::uint16_t> outputGearParts_;
    /** Output gear phase that meshes its teeth with the input gear's. */
    std::vector<float> outputGearPhase_;
    /** Z of the collar per gear, then in neutral. */
    std::vector<float> collarZ_;
    /** +1 when the input shaft turns positively about +Z in the world. */
    float inputSense_ { 1.0F };
    /** Where the flywheel ends along +Z, its radius; set by buildCrankshaft(). */
    float flywheelEndZ_ { -1.0e9F };
    std::vector<JournalSpan> mainJournals_;
    /** A two-stroke's crankcase: round, just clear of the crank's webs. */
    float crankcaseRadius_ {};
    float flywheelRadius_ {};
    /** Flames at the exhaust outlets, lit by an afterfire. */
    std::vector<std::uint16_t> outletFlameParts_;
    /** Parts left out of the engine framing (exhaust trunk, airbox, inlet duct). */
    std::vector<bool> farParts_;
    SceneBounds bounds_ {};
    SceneBounds systemBounds_ {};
    float pitch_ {};
    bool radialLike_ {};
    /** A rotary's: each rotor's part, its station along the shaft and its
        eccentric's offset (shaft degrees, its first face's top dead centre). */
    struct Rotor final {
        std::uint16_t part {};
        float z {};
        double offsetDegrees {};
    };
    std::vector<Rotor> rotors_;
    std::uint16_t rotaryShaftPart_ { 0xFFFFU };
    std::vector<CamshaftPlacement> camshafts_;
    /** The timing chain and the crank's sprocket: camshaft roles. */
    std::vector<std::uint16_t> camDriveParts_;
    std::uint16_t sparkPlugPart_ { 0xFFFFU };
    std::vector<ControlPart> controlParts_;
    /** The nitrous bottle, its line and its plate. */
    std::uint16_t nitrousPart_ { 0xFFFFU };
    /** +1 when the crank turns positively about +Z in the world. */
    float crankSense_ { 1.0F };
};

struct PartHit final {
    std::uint16_t part {};
    /** Distance along the ray, millimetres. */
    float distance {};
};

/** The nearest part a ray meets, among the posed instances `pickable` accepts.
 *  A see-through intake duct (the gas flow's material) does not hide what it
 *  holds: a solid part the ray meets between the duct's near and far walls
 *  (a port injector in its runner) is picked rather than the duct. */
[[nodiscard]] std::optional<PartHit> pickPart(const EngineModel3D&, const std::vector<SceneInstance>&, Vec3 origin,
                                              Vec3 direction, const std::function<bool(std::uint16_t)>& pickable);

} // namespace enginelab::render
