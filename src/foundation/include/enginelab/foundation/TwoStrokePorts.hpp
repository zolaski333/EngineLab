#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace enginelab {

/** How far the piston crown is below top dead centre, mm, at `crankDegrees`
 *  after it (slider-crank, no wrist-pin offset). */
[[nodiscard]] inline double pistonTravelMm(double strokeMm, double rodMm, double crankDegrees) noexcept {
    const auto radius = 0.5 * strokeMm;
    const auto angle = crankDegrees * std::numbers::pi / 180.0;
    const auto sine = std::sin(angle);
    const auto rod = std::max(rodMm, radius * 1.01);
    return radius * (1.0 - std::cos(angle)) + rod - std::sqrt(rod * rod - radius * radius * sine * sine);
}

/** A port's total width: the authored one, else its share of the bore. */
[[nodiscard]] inline double exhaustPortWidthMm(const TwoStrokeConfig& ports, const CylinderConfig& cylinder) noexcept {
    return ports.exhaustPortWidthMm > 0.0 ? ports.exhaustPortWidthMm : 0.65 * cylinder.boreMm;
}
[[nodiscard]] inline double transferPortWidthMm(const TwoStrokeConfig& ports, const CylinderConfig& cylinder) noexcept {
    return ports.transferPortWidthMm > 0.0 ? ports.transferPortWidthMm : 1.30 * cylinder.boreMm;
}

/**
 * The effective open area, mm^2 (discharge coefficient included), of a port
 * whose top edge the piston uncovers `openDegrees` after top dead centre, at
 * cycle phase `phaseDegrees` (0-360, 0 = top dead centre). The port's bottom
 * is at bottom dead centre: fully open there.
 */
[[nodiscard]] inline double portOpenAreaMm2(const CylinderConfig& cylinder, double openDegrees, double widthMm,
                                            double dischargeCoefficient, double phaseDegrees) noexcept {
    const auto top = pistonTravelMm(cylinder.strokeMm, cylinder.connectingRodMm, openDegrees);
    const auto now = pistonTravelMm(cylinder.strokeMm, cylinder.connectingRodMm, phaseDegrees);
    const auto height = std::clamp(now - top, 0.0, std::max(0.0, cylinder.strokeMm - top));
    return widthMm * height * dischargeCoefficient;
}

/** Crank degrees a port stays open, from the angle at which the piston crown
 *  uncovers its top edge.
 *
 *  `portOpenAreaMm2` puts the port's bottom at bottom dead centre, so a port
 *  that opens `openDegrees` after top dead centre closes the same distance the
 *  other side of it. The duration is NOT `openDegrees`: on the stock
 *  catalogue twin the transfer ports open at 118 deg and are open for 124, and
 *  the exhaust port opens at 92 and is open for 176. A larger opening angle is
 *  a shorter window, which is why the exhaust port -- the smaller number --
 *  opens first and closes last, as it must.
 */
[[nodiscard]] inline double portOpenDurationDegrees(double openDegrees) noexcept {
    return 2.0 * std::max(0.0, 180.0 - openDegrees);
}

/** The crankcase volume, litres, at `phaseDegrees`: largest with the piston
 *  at top dead centre, `crankcaseCompressionRatio` times its volume at
 *  bottom dead centre. */
[[nodiscard]] inline double crankcaseVolumeLitres(const CylinderConfig& cylinder, double crankcaseCompressionRatio,
                                                  double phaseDegrees) noexcept {
    const auto areaMm2 = std::numbers::pi * 0.25 * cylinder.boreMm * cylinder.boreMm;
    const auto sweptMm3 = areaMm2 * cylinder.strokeMm;
    const auto atBottomMm3 = sweptMm3 / std::max(0.01, crankcaseCompressionRatio - 1.0);
    const auto travel = pistonTravelMm(cylinder.strokeMm, cylinder.connectingRodMm, phaseDegrees);
    return (atBottomMm3 + areaMm2 * (cylinder.strokeMm - travel)) * 1.0e-6;
}

} // namespace enginelab
