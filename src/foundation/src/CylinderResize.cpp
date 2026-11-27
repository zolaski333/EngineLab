#include <enginelab/foundation/CylinderResize.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <string>

namespace enginelab {

std::optional<CylinderSize> cylinderSize(const EngineConfig& config) {
    if (config.cylinders.empty()) return std::nullopt;
    const auto& first = config.cylinders.front();
    for (const auto& cylinder : config.cylinders)
        if (cylinder.boreMm != first.boreMm || cylinder.strokeMm != first.strokeMm) return std::nullopt;
    return CylinderSize { first.boreMm, first.strokeMm };
}

std::string resizeCylinders(EngineConfig& config, double boreMm, double strokeMm) {
    if (config.cycle == EngineCycle::rotary)
        return "A rotary's faces take their size from the rotor: design it again to resize it.";
    if (!cylinderSize(config)) return "The cylinders do not share one bore and stroke.";
    if (!std::isfinite(boreMm) || boreMm < 20.0 || boreMm > 200.0)
        return "The bore must be between 20 and 200 mm.";
    if (!std::isfinite(strokeMm) || strokeMm < 20.0 || strokeMm > 200.0)
        return "The stroke must be between 20 and 200 mm.";
    for (const auto& cylinder : config.cylinders)
        if (cylinder.connectingRodMm <= strokeMm * 0.5 + 0.01)
            return "The connecting rod is too short for this stroke.";
    auto edited = config;
    for (auto& cylinder : edited.cylinders) {
        const auto addedStrokeMm = strokeMm - cylinder.strokeMm;
        cylinder.boreMm = boreMm;
        cylinder.strokeMm = strokeMm;
        if (cylinder.headChamberVolumeCc <= 0.0) continue;
        // Same piston-to-deck clearance at TDC: the crank radius grew by half
        // the added stroke, so the deck rises by as much.
        cylinder.deckHeightMm += addedStrokeMm * 0.5;
        const auto areaMm2 = std::numbers::pi * boreMm * boreMm * 0.25;
        const auto deckClearanceMm = cylinder.deckHeightMm - strokeMm * 0.5
            - cylinder.connectingRodMm - cylinder.compressionHeightMm;
        const auto clearanceCc = cylinder.headChamberVolumeCc - cylinder.pistonCrownVolumeCc
            + areaMm2 * (cylinder.headGasketThicknessMm + deckClearanceMm) / 1'000.0;
        if (clearanceCc <= 0.1) return "The piston would reach the head.";
        cylinder.compressionRatio = 1.0 + areaMm2 * strokeMm / 1'000.0 / clearanceCc;
    }
    for (auto& journal : edited.crankJournals) journal.throwMm = strokeMm * 0.5;
    config = std::move(edited);
    return {};
}

std::string setCompressionRatio(EngineConfig& config, double ratio) {
    if (!std::isfinite(ratio) || ratio < 5.0 || ratio > 20.0)
        return "The compression ratio must be between 5 and 20.";
    auto edited = config;
    for (auto& cylinder : edited.cylinders) {
        cylinder.compressionRatio = ratio;
        if (cylinder.headChamberVolumeCc <= 0.0) continue;
        // The clearance this ratio needs, taken from the piston-to-deck gap.
        const auto areaMm2 = std::numbers::pi * cylinder.boreMm * cylinder.boreMm * 0.25;
        const auto clearanceCc = areaMm2 * cylinder.strokeMm / 1'000.0 / (ratio - 1.0);
        const auto deckClearanceMm = (clearanceCc - cylinder.headChamberVolumeCc + cylinder.pistonCrownVolumeCc)
                * 1'000.0 / areaMm2
            - cylinder.headGasketThicknessMm;
        if (deckClearanceMm + cylinder.headGasketThicknessMm < 0.0)
            return "The piston would rise past the head gasket: this chamber cannot reach that ratio.";
        cylinder.deckHeightMm = deckClearanceMm + cylinder.strokeMm * 0.5 + cylinder.connectingRodMm
            + cylinder.compressionHeightMm;
    }
    config = std::move(edited);
    return {};
}

std::optional<double> connectingRodLength(const EngineConfig& config) {
    if (config.cylinders.empty()) return std::nullopt;
    const auto& first = config.cylinders.front();
    for (const auto& cylinder : config.cylinders)
        if (cylinder.connectingRodType != ConnectingRodType::conventional
            || cylinder.connectingRodMm != first.connectingRodMm)
            return std::nullopt;
    return first.connectingRodMm;
}

std::string resizeConnectingRods(EngineConfig& config, double rodMm) {
    if (config.cycle == EngineCycle::rotary) return "A rotary has no connecting rods.";
    if (!connectingRodLength(config))
        return "The cylinders do not share one conventional connecting rod.";
    if (!std::isfinite(rodMm) || rodMm > 500.0) return "The connecting rod must be at most 500 mm.";
    for (const auto& cylinder : config.cylinders)
        if (rodMm < 1.2 * cylinder.strokeMm)
            return "The connecting rod must be at least 1.2 times the stroke.";
    for (auto& cylinder : config.cylinders) {
        // Same piston-to-deck clearance at TDC, so the same chamber.
        if (cylinder.headChamberVolumeCc > 0.0)
            cylinder.deckHeightMm += rodMm - cylinder.connectingRodMm;
        cylinder.connectingRodMm = rodMm;
    }
    return {};
}

double exhaustValveDiameterMm(const CylinderConfig& cylinder) noexcept {
    if (cylinder.exhaustValveDiameterMm > 0.0) return cylinder.exhaustValveDiameterMm;
    return std::clamp(cylinder.boreMm * (cylinder.exhaustValveCount == 1U ? 0.41 : 0.34), 12.0, 80.0);
}

namespace {
[[nodiscard]] double portFraction(const CamshaftConfig& cams) noexcept {
    return cams.exhaustPortHeadAreaFraction > 0.0 ? cams.exhaustPortHeadAreaFraction
                                                  : defaultExhaustPortHeadAreaFraction;
}
} // namespace

std::optional<double> exhaustPortDiameterMm(const EngineConfig& config) {
    if (config.cycle != EngineCycle::fourStroke || config.cylinders.empty()) return std::nullopt;
    const auto& first = config.cylinders.front();
    const auto valve = exhaustValveDiameterMm(first);
    const auto fraction = portFraction(camshaftOfCylinder(config, first));
    for (const auto& cylinder : config.cylinders)
        if (exhaustValveDiameterMm(cylinder) != valve || portFraction(camshaftOfCylinder(config, cylinder)) != fraction)
            return std::nullopt;
    return valve * std::sqrt(fraction);
}

std::string setExhaustPortDiameterMm(EngineConfig& config, double diameterMm) {
    if (config.cycle == EngineCycle::twoStroke)
        return "A two-stroke's exhaust port is a window in the liner: set its width and timing on the port.";
    if (config.cycle == EngineCycle::rotary) return "A rotary's exhaust port is in its housing, not behind a valve.";
    if (!exhaustPortDiameterMm(config))
        return "The cylinders' exhaust valves or ports differ: give them one valve diameter first.";
    const auto valve = exhaustValveDiameterMm(config.cylinders.front());
    const auto fraction = std::isfinite(diameterMm) && diameterMm > 0.0 ? std::pow(diameterMm / valve, 2.0) : 0.0;
    if (!(fraction >= 0.2 && fraction <= 1.2))
        return "With " + std::to_string(static_cast<int>(std::lround(valve))) + " mm valves the port is "
            + std::to_string(static_cast<int>(std::ceil(valve * std::sqrt(0.2)))) + " to "
            + std::to_string(static_cast<int>(std::floor(valve * std::sqrt(1.2)))) + " mm.";
    config.camshafts.exhaustPortHeadAreaFraction = fraction;
    for (auto& bank : config.banks) bank.camshafts.exhaustPortHeadAreaFraction = fraction;
    return {};
}

} // namespace enginelab
