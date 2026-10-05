#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

#include <optional>
#include <string>

namespace enginelab {

/** The bore and stroke every cylinder shares. */
struct CylinderSize final {
    double boreMm { 0.0 };
    double strokeMm { 0.0 };
};

/** The engine's bore and stroke, or nothing when the cylinders differ. */
[[nodiscard]] std::optional<CylinderSize> cylinderSize(const EngineConfig& config);

/**
 * Gives every cylinder `boreMm` and `strokeMm` in place, with the crank
 * journals' throws at half the stroke. Returns an empty string on success, or
 * why the edit was refused (the configuration is then unchanged): cylinders
 * that do not share one size, a size out of range, or a connecting rod too
 * short for the stroke.
 *
 * An engine authored by compression ratio keeps it: the clearance volume
 * follows the swept volume. An engine authored by chamber geometry keeps its
 * piston-to-deck clearance (the deck rises by half the added stroke) and its
 * compression ratio follows, as on a real stroker or overbore. The caller
 * still validates the whole configuration.
 */
[[nodiscard]] std::string resizeCylinders(EngineConfig& config, double boreMm, double strokeMm);

} // namespace enginelab
