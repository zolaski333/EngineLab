#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

#include <cstdint>
#include <optional>
#include <string>

namespace enginelab {

/**
 * The length and diameter of one exhaust component as the 3-D view draws it,
 * and which of them an edit may change.
 *
 * Component ids are those of the path's network or, on a path authored as
 * scalar geometry, of makeEditableExhaustNetwork(). There the primaries share
 * one length and diameter, a collector and an outlet have no length, and a
 * silencer's diameter is its body's.
 */
struct ExhaustComponentSize final {
    double lengthMm { 0.0 };
    double diameterMm { 0.0 };
    bool lengthEditable { false };
    /** A scalar-geometry primary: resizing it resizes every primary of the path. */
    bool sharedByPrimaries { false };
};

[[nodiscard]] std::optional<ExhaustComponentSize> exhaustComponentSize(
    const EngineConfig& config, std::uint32_t pathId, std::uint32_t componentId);

/**
 * Resizes one component in place. Returns an empty string on success, or why
 * the edit was refused (the configuration is then unchanged): a length on a
 * component without one, a size out of range, or an edit that would change
 * which components the path has (a silencer no wider than its pipes stops
 * being one). On the only path of a scalar exhaust, the global geometry
 * (`EngineConfig::exhaust`) changes with it. The caller still validates the
 * whole configuration.
 */
[[nodiscard]] std::string resizeExhaustComponent(
    EngineConfig& config, std::uint32_t pathId, std::uint32_t componentId,
    double lengthMm, double diameterMm);

} // namespace enginelab
