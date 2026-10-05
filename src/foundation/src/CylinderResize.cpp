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

} // namespace enginelab
