#include <enginelab/foundation/CylinderResize.hpp>

#include <cmath>
#include <numbers>

namespace enginelab {

std::optional<CylinderSize> cylinderSize(const EngineConfig& config) {
    if (config.cylinders.empty()) return std::nullopt;
    const auto& first = config.cylinders.front();
    for (const auto& cylinder : config.cylinders)
        if (cylinder.boreMm != first.boreMm || cylinder.strokeMm != first.strokeMm) return std::nullopt;
    return CylinderSize { first.boreMm, first.strokeMm };
}

std::string resizeCylinders(EngineConfig& config, double boreMm, double strokeMm) {
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

} // namespace enginelab
