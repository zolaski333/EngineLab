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

/**
 * Gives every cylinder the compression ratio `ratio`, in place. Returns an
 * empty string on success, or why the edit was refused (the configuration is
 * then unchanged): a ratio out of [5, 20], or a chamber geometry that cannot
 * reach it.
 *
 * An engine authored by compression ratio takes it as is: the clearance
 * volume follows. An engine authored by chamber geometry keeps its head
 * chamber, gasket and piston, and its deck moves (a decked or shimmed block):
 * refused when the piston crown would have to rise past the gasket. The
 * caller still validates the whole configuration.
 */
[[nodiscard]] std::string setCompressionRatio(EngineConfig& config, double ratio);

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

/** The share of the exhaust valves' head area a production port's effective
 *  area saturates at (Heywood ch. 6), what a camshaft's
 *  `exhaustPortHeadAreaFraction` of 0 means. EngineSimulator uses the same
 *  0.51. */
inline constexpr double defaultExhaustPortHeadAreaFraction = 0.51;

/** The exhaust valve diameter a cylinder runs: its own, else the bore's share
 *  (0.41 for one valve, 0.34 for more, 12 to 80 mm), as EngineSimulator
 *  derives it. */
[[nodiscard]] double exhaustValveDiameterMm(const CylinderConfig& cylinder) noexcept;

/**
 * The exhaust port's diameter, per valve: the round hole whose area is the
 * port's effective flow area at full lift (the camshaft's port fraction of
 * the valve head's area, Cd included, as a flow bench reads it). Nothing on a
 * two-stroke (its port is a window) or a rotary, or when the cylinders'
 * exhaust valves or their camshafts' ports differ.
 */
[[nodiscard]] std::optional<double> exhaustPortDiameterMm(const EngineConfig& config);

/**
 * Gives every cylinder's exhaust port the effective diameter `diameterMm` per
 * valve, in place: the port fraction of every camshaft (each bank's too)
 * becomes (diameterMm / valve diameter)^2. Refused, the configuration then
 * unchanged, where exhaustPortDiameterMm reads nothing or the fraction would
 * leave 0.2-1.2 (what validation accepts). Owner 2026-11-27: the exhaust
 * port's diameter could not be set in the app.
 */
[[nodiscard]] std::string setExhaustPortDiameterMm(EngineConfig& config, double diameterMm);

} // namespace enginelab
