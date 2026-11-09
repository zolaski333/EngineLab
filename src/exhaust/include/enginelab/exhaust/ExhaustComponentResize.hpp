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
 * one length and diameter and a silencer's diameter is its body's. Every
 * component's length is editable; a collector, splitter, crossover or outlet
 * at 0 has none (a lumped junction, an open end).
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
 * the edit was refused (the configuration is then unchanged): a size out of
 * range (0 is a length only for a component that may have none), or an edit
 * that would change which components the path has (a silencer no wider than
 * its pipes stops being one). On the only path of a scalar exhaust, the
 * global geometry (`EngineConfig::exhaust`) changes with it; a length the
 * scalar fields cannot hold (a collector's, an outlet's) makes the path its
 * authored network first. The caller still validates the whole configuration.
 */
[[nodiscard]] std::string resizeExhaustComponent(
    EngineConfig& config, std::uint32_t pathId, std::uint32_t componentId,
    double lengthMm, double diameterMm);

/**
 * Joins exhaust path `joinedId` into path `keptId`, in place: a boxer six's
 * two 3-into-1 banks become one 6-into-1. Every cylinder of the joined path
 * gets a copy of the kept path's first primary run (its components up to the
 * first junction several pipes meet at) feeding that junction; the joined
 * path's own pipes, silencers and outlet go. Banks that fed the joined path
 * feed the kept one. Returns an empty string on success, or why the join was
 * refused (the configuration is then unchanged): no such paths, the same
 * path twice, a kept path whose primary reaches its outlet without meeting
 * another (nothing to join into), or a turbocharger left without a path. The
 * collector keeps its size. The caller still validates the whole
 * configuration.
 */
[[nodiscard]] std::string joinExhaustPaths(EngineConfig& config, std::uint32_t keptId, std::uint32_t joinedId);

} // namespace enginelab
