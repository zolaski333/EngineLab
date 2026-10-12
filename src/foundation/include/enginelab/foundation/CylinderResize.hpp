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

/** The connecting rod length every cylinder shares, or nothing when the rods
 *  differ or one is a radial's master or articulated rod. */
[[nodiscard]] std::optional<double> connectingRodLength(const EngineConfig& config);

/**
 * Gives every cylinder a connecting rod `rodMm` long, in place. Returns an
 * empty string on success, or why the edit was refused (the configuration is
 * then unchanged): rods that differ or are articulated, or a rod shorter
 * than 1.2 strokes (the angularity no real engine runs) or over 500 mm.
 *
 * The compression ratio stays: the piston reaches the same height at TDC.
 * An engine authored by chamber geometry keeps its piston-to-deck clearance,
 * so its deck rises by the added length, as with a taller block. The caller
 * still validates the whole configuration.
 */
[[nodiscard]] std::string resizeConnectingRods(EngineConfig& config, double rodMm);

} // namespace enginelab
