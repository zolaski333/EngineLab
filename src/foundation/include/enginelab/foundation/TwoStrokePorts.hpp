#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <optional>
#include <string_view>
#include <vector>

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

[[nodiscard]] inline double portWindowsWidthMm(const std::vector<TwoStrokePortWindow>& windows) noexcept {
    auto sum = 0.0;
    for (const auto& window : windows) sum += window.widthMm;
    return sum;
}

/** A port's total width: its windows', else the authored one, else its
 *  share of the bore. */
[[nodiscard]] inline double exhaustPortWidthMm(const TwoStrokeConfig& ports, const CylinderConfig& cylinder) noexcept {
    if (!ports.exhaustWindows.empty()) return portWindowsWidthMm(ports.exhaustWindows);
    return ports.exhaustPortWidthMm > 0.0 ? ports.exhaustPortWidthMm : 0.65 * cylinder.boreMm;
}
[[nodiscard]] inline double transferPortWidthMm(const TwoStrokeConfig& ports, const CylinderConfig& cylinder) noexcept {
    if (!ports.transferWindows.empty()) return portWindowsWidthMm(ports.transferWindows);
    return ports.transferPortWidthMm > 0.0 ? ports.transferPortWidthMm : 1.30 * cylinder.boreMm;
}

/** Sets each group's opening angle to its earliest window's, as validation
 *  asks; a group without windows keeps its own. */
inline void synchronisePortWindowTimings(TwoStrokeConfig& ports) noexcept {
    const auto earliest = [](const std::vector<TwoStrokePortWindow>& windows, double fallback) noexcept {
        if (windows.empty()) return fallback;
        auto angle = windows.front().openDegrees;
        for (const auto& window : windows) angle = std::min(angle, window.openDegrees);
        return angle;
    };
    ports.exhaustPortOpenDegrees = earliest(ports.exhaustWindows, ports.exhaustPortOpenDegrees);
    ports.transferPortOpenDegrees = earliest(ports.transferWindows, ports.transferPortOpenDegrees);
}

[[nodiscard]] inline bool hasPowerValve(const TwoStrokeConfig& ports) noexcept {
    return ports.powerValveClosedExhaustPortOpenDegrees > ports.exhaustPortOpenDegrees
        && ports.powerValveFullyOpenRpm > ports.powerValveOpeningStartRpm;
}

/** Where a power valve's servo is told to be at `rpm`, 0 shut to 1 open:
 *  a smooth lift between its start and fully-open speeds; 1 without one. */
[[nodiscard]] inline double powerValveTargetOpening(const TwoStrokeConfig& ports, double rpm) noexcept {
    if (!hasPowerValve(ports)) return 1.0;
    const auto t = std::clamp((rpm - ports.powerValveOpeningStartRpm)
        / (ports.powerValveFullyOpenRpm - ports.powerValveOpeningStartRpm), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

/** The exhaust port's opening angle at a valve opening, 0 shut to 1 open. */
[[nodiscard]] inline double exhaustPortOpenDegreesAt(const TwoStrokeConfig& ports, double valveOpening) noexcept {
    if (!hasPowerValve(ports)) return ports.exhaustPortOpenDegrees;
    return ports.powerValveClosedExhaustPortOpenDegrees + std::clamp(valveOpening, 0.0, 1.0)
        * (ports.exhaustPortOpenDegrees - ports.powerValveClosedExhaustPortOpenDegrees);
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

/** The area, mm^2, of a window `widthMm` wide and `heightMm` high whose
 *  top corners are rounded to `topRadiusMm` and bottom ones to
 *  `bottomRadiusMm`, uncovered to `depthMm` below its top edge. Exact: each
 *  quarter circle's deficit against the rectangle is integrated in closed
 *  form, so the fully open window is w h - (1 - pi/4)(2 r_top^2 +
 *  2 r_bottom^2). The radii are held within the window. */
[[nodiscard]] inline double roundedWindowAreaMm2(double widthMm, double heightMm, double topRadiusMm,
                                                 double bottomRadiusMm, double depthMm) noexcept {
    const auto height = std::max(0.0, heightMm);
    const auto depth = std::clamp(depthMm, 0.0, height);
    const auto halfWidth = 0.5 * std::max(0.0, widthMm);
    auto top = std::clamp(topRadiusMm, 0.0, halfWidth);
    auto bottom = std::clamp(bottomRadiusMm, 0.0, halfWidth);
    if (top + bottom > height) {
        const auto scale = height / (top + bottom);
        top *= scale;
        bottom *= scale;
    }
    // F(u) = integral of sqrt(r^2 - u^2) du from 0 to u.
    const auto circle = [](double u, double r) noexcept {
        if (!(r > 0.0)) return 0.0;
        const auto x = std::clamp(u / r, -1.0, 1.0);
        return 0.5 * (u * std::sqrt(std::max(0.0, r * r - u * u)) + r * r * std::asin(x));
    };
    const auto topDepth = std::min(depth, top);
    const auto topDeficit = top > 0.0
        ? 2.0 * (top * topDepth - 0.25 * std::numbers::pi * top * top + circle(top - topDepth, top)) : 0.0;
    const auto bottomDepth = std::clamp(depth - (height - bottom), 0.0, bottom);
    const auto bottomDeficit = bottom > 0.0
        ? 2.0 * (bottom * bottomDepth - circle(bottomDepth, bottom)) : 0.0;
    return std::max(0.0, 2.0 * halfWidth * depth - topDeficit - bottomDeficit);
}

/** Which way gas crosses a port, and the static pressure ratio across it
 *  (upstream over downstream, 1 or more). Forward is the way the port is
 *  built for. */
struct PortFlowCondition final {
    double pressureRatio { 1.0 };
    bool forward { true };
};

/** Bilinear interpolation of a map's coefficients, held at its edges. */
[[nodiscard]] inline double portDischargeMapCd(const PortDischargeMap& map, double areaRatio,
                                               PortFlowCondition flow) noexcept {
    const auto& table = !flow.forward && !map.reverseCd.empty() ? map.reverseCd : map.forwardCd;
    const auto rows = map.areaRatios.size();
    const auto columns = map.pressureRatios.size();
    if (rows == 0 || columns == 0 || table.size() != rows * columns) return 0.0;
    const auto bracket = [](const std::vector<double>& axis, double x, std::size_t& lower) noexcept {
        if (axis.size() == 1 || x <= axis.front()) { lower = 0; return 0.0; }
        if (x >= axis.back()) { lower = axis.size() - 2; return 1.0; }
        lower = static_cast<std::size_t>(std::upper_bound(axis.begin(), axis.end(), x) - axis.begin()) - 1;
        return (x - axis[lower]) / (axis[lower + 1] - axis[lower]);
    };
    std::size_t row = 0;
    std::size_t column = 0;
    const auto u = bracket(map.areaRatios, areaRatio, row);
    const auto v = bracket(map.pressureRatios, flow.pressureRatio, column);
    const auto at = [&](std::size_t r, std::size_t c) noexcept {
        return table[std::min(r, rows - 1) * columns + std::min(c, columns - 1)];
    };
    const auto low = at(row, column) * (1.0 - v) + at(row, column + 1) * v;
    const auto high = at(row + 1, column) * (1.0 - v) + at(row + 1, column + 1) * v;
    return low * (1.0 - u) + high * u;
}

/** The effective open area, mm^2, of one window whose top edge the crown
 *  uncovers `topOpenDegrees` after top dead centre, at `phaseDegrees`: its
 *  uncovered area times its discharge coefficient, the map's at that area
 *  ratio and flow when it has one. */
[[nodiscard]] inline double portWindowEffectiveAreaMm2(const CylinderConfig& cylinder,
                                                       const TwoStrokePortWindow& window, double topOpenDegrees,
                                                       double defaultDischargeCoefficient, double phaseDegrees,
                                                       PortFlowCondition flow) noexcept {
    const auto top = pistonTravelMm(cylinder.strokeMm, cylinder.connectingRodMm, topOpenDegrees);
    const auto now = pistonTravelMm(cylinder.strokeMm, cylinder.connectingRodMm, phaseDegrees);
    const auto height = std::max(0.0, cylinder.strokeMm - top);
    const auto open = roundedWindowAreaMm2(window.widthMm, height, window.topCornerRadiusMm,
        window.bottomCornerRadiusMm, now - top);
    if (!(open > 0.0)) return 0.0;
    if (!window.dischargeMap.empty()) {
        const auto full = roundedWindowAreaMm2(window.widthMm, height, window.topCornerRadiusMm,
            window.bottomCornerRadiusMm, height);
        return open * portDischargeMapCd(window.dischargeMap, full > 0.0 ? open / full : 1.0, flow);
    }
    return open * (window.dischargeCoefficient > 0.0 ? window.dischargeCoefficient : defaultDischargeCoefficient);
}

/** The transfer ports' effective open area, mm^2, at `phaseDegrees`: the
 *  sum of their windows', or the one rectangle of `portOpenAreaMm2`. */
[[nodiscard]] inline double transferPortsAreaMm2(const TwoStrokeConfig& ports, const CylinderConfig& cylinder,
                                                 double phaseDegrees, PortFlowCondition flow = {}) noexcept {
    if (ports.transferWindows.empty())
        return portOpenAreaMm2(cylinder, ports.transferPortOpenDegrees, transferPortWidthMm(ports, cylinder),
            ports.portDischargeCoefficient, phaseDegrees);
    auto sum = 0.0;
    for (const auto& window : ports.transferWindows)
        sum += portWindowEffectiveAreaMm2(cylinder, window, window.openDegrees, ports.portDischargeCoefficient,
            phaseDegrees, flow);
    return sum;
}

/** The exhaust ports' effective open area, mm^2, at `phaseDegrees`, the
 *  power valve's roof at `exhaustOpenDegrees` (`exhaustPortOpenDegreesAt`):
 *  a window under the valve opens at the later of its own edge and the
 *  roof. */
[[nodiscard]] inline double exhaustPortsAreaMm2(const TwoStrokeConfig& ports, const CylinderConfig& cylinder,
                                                double exhaustOpenDegrees, double phaseDegrees,
                                                PortFlowCondition flow = {}) noexcept {
    if (ports.exhaustWindows.empty())
        return portOpenAreaMm2(cylinder, exhaustOpenDegrees, exhaustPortWidthMm(ports, cylinder),
            ports.portDischargeCoefficient, phaseDegrees);
    auto sum = 0.0;
    for (const auto& window : ports.exhaustWindows)
        sum += portWindowEffectiveAreaMm2(cylinder, window,
            window.powerValve ? std::max(window.openDegrees, exhaustOpenDegrees) : window.openDegrees,
            ports.portDischargeCoefficient, phaseDegrees, flow);
    return sum;
}

/** A two-stroke intake's name in engine files. */
[[nodiscard]] inline const char* twoStrokeIntakeName(TwoStrokeIntake intake) noexcept {
    return intake == TwoStrokeIntake::cylinderPort ? "cylinder_port" : "crankcase_reed";
}
[[nodiscard]] inline std::optional<TwoStrokeIntake> parseTwoStrokeIntake(std::string_view name) noexcept {
    if (name == "crankcase_reed") return TwoStrokeIntake::crankcaseReed;
    if (name == "cylinder_port") return TwoStrokeIntake::cylinderPort;
    return std::nullopt;
}

[[nodiscard]] inline double intakePortWidthMm(const TwoStrokeConfig& ports, const CylinderConfig& cylinder) noexcept {
    return ports.intakePortWidthMm > 0.0 ? ports.intakePortWidthMm : 0.70 * cylinder.boreMm;
}

/** The geometric open area, mm^2, of a cylinder intake port (TwoStrokeIntake::
 *  cylinderPort) at cycle phase `phaseDegrees`. The skirt's lower edge
 *  passes the port's lower edge `intakePortOpenDegreesBtdc` before top dead
 *  centre and travels with the piston, so the uncovered height is how far the
 *  piston is above where it stood then; the port's top edge is where the skirt
 *  reaches at top dead centre. Zero for a crankcase reed. */
[[nodiscard]] inline double intakePortOpenAreaMm2(const TwoStrokeConfig& ports, const CylinderConfig& cylinder,
                                                  double phaseDegrees) noexcept {
    if (ports.intake != TwoStrokeIntake::cylinderPort) return 0.0;
    const auto opening = pistonTravelMm(cylinder.strokeMm, cylinder.connectingRodMm, ports.intakePortOpenDegreesBtdc);
    const auto now = pistonTravelMm(cylinder.strokeMm, cylinder.connectingRodMm, phaseDegrees);
    return intakePortWidthMm(ports, cylinder) * std::clamp(opening - now, 0.0, opening);
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
