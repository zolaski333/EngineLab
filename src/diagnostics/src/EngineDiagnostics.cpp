#include <enginelab/diagnostics/EngineDiagnostics.hpp>
#include <algorithm>

namespace enginelab {
std::vector<Diagnostic> EngineDiagnostics::evaluate(const EngineConfig& config, const EngineState& state) const {
    std::vector<Diagnostic> result;
    if (state.knockLevel > 0.45) result.push_back({ DiagnosticSeverity::critical, "combustion.knock", "Cliquetis severe : reduisez compression, avance ou charge." });
    if (state.coolantTemperatureC > 112.0) result.push_back({ DiagnosticSeverity::warning, "thermal.coolant", "Temperature de refroidissement excessive." });
    if (state.misfireRate > 0.2) result.push_back({ DiagnosticSeverity::warning, "combustion.misfire", "Combustion instable : verifiez la richesse et l'allumage." });
    if (state.rpm > config.redlineRpm) result.push_back({ DiagnosticSeverity::critical, "mechanical.overrev", "Regime superieur a la limite mecanique estimee." });
    if (state.rpm > 900.0 && state.oilPressureKpa < 100.0) result.push_back({ DiagnosticSeverity::critical, "lubrication.pressure", "Pression d'huile insuffisante sous regime." });
    if (state.exhaustTemperatureC > 920.0) result.push_back({ DiagnosticSeverity::warning, "thermal.exhaust", "Temperature d'echappement excessive." });
    if (state.oilTemperatureC > 135.0) result.push_back({ DiagnosticSeverity::critical, "thermal.oil", "Temperature d'huile critique." });
    if (state.load > 0.55 && state.airFuelRatio > 15.2)
        result.push_back({ DiagnosticSeverity::critical, "combustion.lean", "Melange trop pauvre sous charge." });
    if (state.airFuelRatio < 10.8)
        result.push_back({ DiagnosticSeverity::warning, "combustion.rich", "Melange excessivement riche : dilution d'huile possible." });
    const auto exhaustDeltaKpa = state.exhaustPressureKpa - config.ambientPressureKpa;
    const auto flowAllowance = 28.0 + engineDisplacementLitres(config) * 1.4
        + config.exhaust.collectorDiameterMm * 0.10 + config.exhaust.outletDiameterMm * 0.06;
    const auto stableHighLoad = state.load > 0.45 && state.rpm > config.idleRpm * 1.25;
    if (stableHighLoad && exhaustDeltaKpa > flowAllowance
        && state.exhaustPressureKpa / std::max(1.0, config.ambientPressureKpa) > 1.32)
        result.push_back({ DiagnosticSeverity::warning, "exhaust.back_pressure", "Contre-pression d'echappement excessive." });
    if (state.damage > 0.5) result.push_back({ DiagnosticSeverity::critical, "mechanical.damage", "Dommages mecaniques importants : puissance et fiabilite degradees." });
    else if (state.wear > 0.35) result.push_back({ DiagnosticSeverity::warning, "mechanical.wear", "Usure mecanique mesurable." });
    if (state.meanPistonSpeedMps > 25.0) result.push_back({ DiagnosticSeverity::warning, "mechanical.piston_speed", "Vitesse moyenne des pistons elevee." });
    if (state.peakPistonAccelerationG > 5'500.0)
        result.push_back({ DiagnosticSeverity::critical, "mechanical.piston_acceleration", "Acceleration piston critique pour l'equipage mobile." });
    return result;
}
} // namespace enginelab
