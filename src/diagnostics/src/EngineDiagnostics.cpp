#include <enginelab/diagnostics/EngineDiagnostics.hpp>
namespace enginelab {
std::vector<Diagnostic> EngineDiagnostics::evaluate(const EngineConfig& config, const EngineState& state) const {
    std::vector<Diagnostic> result;
    if (state.knockLevel > 0.45) result.push_back({ DiagnosticSeverity::critical, "combustion.knock", "Cliquetis sévère : réduisez compression, avance ou charge." });
    if (state.coolantTemperatureC > 112.0) result.push_back({ DiagnosticSeverity::warning, "thermal.coolant", "Température de refroidissement excessive." });
    if (state.misfireRate > 0.2) result.push_back({ DiagnosticSeverity::warning, "combustion.misfire", "Combustion instable : vérifiez la richesse et l'allumage." });
    if (state.rpm > config.redlineRpm) result.push_back({ DiagnosticSeverity::critical, "mechanical.overrev", "Régime supérieur à la limite mécanique estimée." });
    if (state.rpm > 900.0 && state.oilPressureKpa < 100.0) result.push_back({ DiagnosticSeverity::critical, "lubrication.pressure", "Pression d'huile insuffisante sous régime." });
    if (state.exhaustTemperatureC > 920.0) result.push_back({ DiagnosticSeverity::warning, "thermal.exhaust", "Température d'échappement excessive." });
    if (state.oilTemperatureC > 135.0) result.push_back({ DiagnosticSeverity::critical, "thermal.oil", "Température d'huile critique." });
    if (state.load > 0.55 && state.airFuelRatio > 15.2)
        result.push_back({ DiagnosticSeverity::critical, "combustion.lean", "Mélange trop pauvre sous charge." });
    if (state.airFuelRatio < 10.8)
        result.push_back({ DiagnosticSeverity::warning, "combustion.rich", "Mélange excessivement riche : dilution d'huile possible." });
    if (state.exhaustPressureKpa - config.ambientPressureKpa > 35.0)
        result.push_back({ DiagnosticSeverity::warning, "exhaust.back_pressure", "Contre-pression d'échappement excessive." });
    if (state.damage > 0.5) result.push_back({ DiagnosticSeverity::critical, "mechanical.damage", "Dommages mécaniques importants : puissance et fiabilité dégradées." });
    else if (state.wear > 0.35) result.push_back({ DiagnosticSeverity::warning, "mechanical.wear", "Usure mécanique mesurable." });
    if (state.meanPistonSpeedMps > 25.0) result.push_back({ DiagnosticSeverity::warning, "mechanical.piston_speed", "Vitesse moyenne des pistons élevée." });
    if (state.peakPistonAccelerationG > 5'500.0)
        result.push_back({ DiagnosticSeverity::critical, "mechanical.piston_acceleration", "Accélération piston critique pour l'équipage mobile." });
    return result;
}
} // namespace enginelab
