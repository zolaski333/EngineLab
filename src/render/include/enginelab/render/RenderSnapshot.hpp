#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace enginelab {

/**
 * Renderer-neutral 3D vector expressed in the engine model's millimetre space.
 *
 * EngineLab's solver currently resolves the crank mechanism in XY.  The render
 * adapter places cylinder stations along Z so a future GPU renderer never has
 * to infer layout from presentation-specific pixels.
 */
struct RenderVector3 final {
    double x {};
    double y {};
    double z {};
};

/** Euler orientation in degrees. Meshes are expected to point along local +Y. */
struct RenderRotation3 final {
    double xDegrees {};
    double yDegrees {};
    double zDegrees {};
};

struct RenderTransform final {
    RenderVector3 positionMm;
    RenderRotation3 rotation;
    RenderVector3 scale { 1.0, 1.0, 1.0 };
};

enum class RenderPartKind : std::uint8_t {
    crankshaft,
    crankJournal,
    cylinder,
    piston,
    connectingRod,
    intakeValve,
    exhaustValve
};

/** Stable key and transform for one renderable engine component. */
struct RenderPartState final {
    RenderPartKind kind { RenderPartKind::cylinder };
    std::uint32_t id {};
    std::uint32_t cylinderId {};
    std::uint32_t parentId {};
    RenderTransform transform;
    double activity {};
    double temperatureC { 22.0 };
};

struct RenderLayout final {
    double cylinderSpacingMm { 105.0 };
    double nominalCylinderLengthMm { 180.0 };
    double nominalValveLengthMm { 28.0 };
};

/**
 * Immutable-by-convention frame consumed by a renderer.
 *
 * A fixed array keeps publication deterministic and allocation-free.  The
 * current 32-cylinder configuration limit requires at most 32 * 6 parts plus
 * crankshafts and journals, leaving comfortable room for future accessories.
 */
struct RenderSnapshot final {
    static constexpr std::size_t maximumParts = 256;

    double simulationTimeSeconds {};
    double crankAngleDegrees {};
    double rpm {};
    RenderVector3 boundsMinimumMm;
    RenderVector3 boundsMaximumMm;
    std::array<RenderPartState, maximumParts> parts {};
    std::size_t partCount {};
};

/** Converts physical configuration/state into a renderer-neutral scene frame. */
class RenderSnapshotBuilder final {
public:
    explicit RenderSnapshotBuilder(EngineConfig config, RenderLayout layout = {});

    [[nodiscard]] const EngineConfig& config() const noexcept { return config_; }
    [[nodiscard]] const RenderLayout& layout() const noexcept { return layout_; }
    [[nodiscard]] RenderSnapshot build(const EngineState& state) const noexcept;

private:
    [[nodiscard]] double cylinderStationZ(std::size_t cylinderIndex) const noexcept;

    EngineConfig config_;
    RenderLayout layout_;
    std::vector<double> cylinderStationsZ_;
};

/** Two-frame interpolation buffer for a 60/120 Hz renderer fed by a 240 Hz sim. */
class RenderSnapshotInterpolator final {
public:
    void reset() noexcept;
    void push(RenderSnapshot snapshot) noexcept;
    [[nodiscard]] bool ready() const noexcept { return current_.has_value(); }
    [[nodiscard]] RenderSnapshot sample(double simulationTimeSeconds) const noexcept;

private:
    std::optional<RenderSnapshot> previous_;
    std::optional<RenderSnapshot> current_;
};

} // namespace enginelab
