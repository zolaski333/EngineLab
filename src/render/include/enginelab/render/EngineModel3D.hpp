#pragma once

#include <enginelab/foundation/EngineTypes.hpp>
#include <enginelab/physics/MechanicalKinematics.hpp>
#include <enginelab/render/Mesh.hpp>

#include <array>
#include <cstdint>
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
};
inline constexpr std::size_t sceneMaterialCount = 10;

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
};

struct SceneInstance final {
    std::uint16_t part {};
    Mat4 transform;
    /** Flame strength or flow glow, 0..1; 1 for plain parts. */
    float intensity { 1.0F };
};

struct ScenePoseInput final {
    /** Primary crank angle over the 720-degree cycle, as the simulator reports it. */
    double crankAngleDegrees {};
    /** Strength of each cylinder's current burn, 0..1 (0 = misfire or cut). */
    std::array<float, 32> combustion {};
    /** Engine speed and throttle, for a variable cam profile switch. */
    double rpm {};
    double throttle {};
};

struct SceneBounds final {
    Vec3 minimum;
    Vec3 maximum;

    [[nodiscard]] Vec3 centre() const noexcept { return (minimum + maximum) * 0.5F; }
    [[nodiscard]] float diagonal() const noexcept { return length(maximum - minimum); }
};

/** Where a cylinder's port leaves the head, for routing intake and exhaust ducts. */
struct PortAnchor final {
    std::uint32_t cylinderId {};
    Vec3 position;
    Vec3 direction;
    float diameterMm {};
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
    [[nodiscard]] const SceneBounds& bounds() const noexcept { return bounds_; }
    [[nodiscard]] std::size_t cylinderCount() const noexcept { return cylinders_.size(); }
    [[nodiscard]] const EngineConfig& config() const noexcept { return config_; }
    [[nodiscard]] const std::vector<PortAnchor>& intakePorts() const noexcept { return intakePorts_; }
    [[nodiscard]] const std::vector<PortAnchor>& exhaustPorts() const noexcept { return exhaustPorts_; }

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
        std::uint16_t runnerPart {};
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
    void buildCrankshaft();
    void buildStructure();
    void buildDucts();
    void computeBounds();

    EngineConfig config_;
    EngineKinematicsReference reference_ {};
    std::vector<ScenePart> parts_;
    std::vector<Cylinder> cylinders_;
    std::vector<Throw> throws_;
    std::vector<std::uint16_t> rotatingParts_;
    std::vector<std::size_t> rotatingThrow_;
    std::vector<PortAnchor> intakePorts_;
    std::vector<PortAnchor> exhaustPorts_;
    SceneBounds bounds_ {};
    int plenumPart_ { -1 };
    float pitch_ {};
    bool radialLike_ {};
};

} // namespace enginelab::render
