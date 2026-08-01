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
    if (state.load > 0.55 && state.lambda > 1.03)
        result.push_back({ DiagnosticSeverity::critical, "combustion.lean", "Melange trop pauvre sous charge." });
    if (state.lambda < 0.74)
        result.push_back({ DiagnosticSeverity::warning, "combustion.rich", "Melange excessivement riche : dilution d'huile possible." });
    if (state.solverResolutionLimited)
        result.push_back({ DiagnosticSeverity::critical, "solver.resolution", "Resolution angulaire du solveur insuffisante au regime actuel." });
    if (state.load > 0.40 && state.rpm > config.idleRpm * 1.2 && state.cylinderStateCount > 0) {
        // `injectorCapacityRatio` is the headroom left on the injector's DUTY
        // CYCLE (1 = shut, 0 = open for the whole 720 deg cycle), so the test is
        // against a duty limit and the threshold comes from production practice:
        // a port injector is sized to stay under roughly 85-90 % duty at rated
        // power, and above that it cannot meter a larger pulse whatever the ECU
        // commands. 0.10 of headroom is therefore the point at which the
        // complaint is true.
        //
        // It used to compare the same field against 0.90, which was correct for
        // what that field USED to hold and is why this warning was permanently
        // on: measured across the catalogue at wide-open throttle the duty runs
        // 7 % (Hayabusa at 3,000 rpm) to 39 % (Merlin), so no shipped engine is
        // injector-limited at all and the message was pure false alarm. Do not
        // raise this threshold to make the warning appear again -- if it never
        // fires on this catalogue that is a measurement, not a broken test.
        // One saturated cylinder is enough to run that cylinder lean. An
        // average can hide it behind seven healthy injectors on a V8, so the
        // diagnostic follows the worst (smallest-headroom) cylinder.
        double injectorDutyHeadroom = 1.0;
        for (std::size_t index = 0; index < state.cylinderStateCount; ++index)
            injectorDutyHeadroom = std::min(injectorDutyHeadroom,
                state.cylinderStates[index].injectorCapacityRatio);
        if (injectorDutyHeadroom < 0.10)
            result.push_back({ DiagnosticSeverity::warning, "injection.capacity", "Injecteurs satures : rapport cyclique au-dela de 90 % sous charge." });
    }
    // Back pressure is a MEAN. This test used to read `exhaustPressureKpa`,
    // which is the max over cylinders of the instantaneous runner pressure -- a
    // blowdown peak envelope -- and compare it against an allowance sized for a
    // mean. A healthy LS3 at 5,940 rpm peaks at 172 kPa against ~101 ambient
    // while discharging freely, so the warning latched on and never cleared: it
    // was a false positive by construction, not a reading. It now uses the
    // damped port mean.
    //
    // Threshold from engine literature rather than from this simulator's own
    // output: a production naturally aspirated exhaust runs roughly 15-30 kPa of
    // mean back pressure at rated power, and a turbocharged one more because the
    // turbine is a deliberate restriction. 40 kPa over ambient is therefore
    // genuinely excessive for an NA engine; a turbo engine is allowed its turbine
    // pressure ratio before the same complaint applies.
    const auto backPressureDeltaKpa =
        state.exhaustBackPressureKpa - config.ambientPressureKpa;
    // A turbine is a deliberate restriction, and the pressure it needs upstream
    // tracks the boost it is producing: for a matched turbo at comparable
    // stage efficiencies the expansion ratio is of the same order as the
    // compressor pressure ratio. So the allowance scales with delivered boost
    // rather than sitting at some flat number that is simultaneously too tight
    // at full boost and too loose off boost.
    const auto turbocharged = config.forcedInduction.enabled
        && config.forcedInduction.type == ForcedInductionType::turbocharger;
    const auto boostAboveAmbientKpa = turbocharged
        ? std::max(0.0, (state.boostPressureRatio - 1.0) * config.ambientPressureKpa)
        : 0.0;
    const auto allowanceKpa = 40.0 + boostAboveAmbientKpa;
    const auto stableHighLoad = state.load > 0.45 && state.rpm > config.idleRpm * 1.25;
    if (stableHighLoad && backPressureDeltaKpa > allowanceKpa)
        result.push_back({ DiagnosticSeverity::warning, "exhaust.back_pressure", "Contre-pression d'echappement excessive." });
    if (state.damage > 0.5) result.push_back({ DiagnosticSeverity::critical, "mechanical.damage", "Dommages mecaniques importants : puissance et fiabilite degradees." });
    else if (state.wear > 0.35) result.push_back({ DiagnosticSeverity::warning, "mechanical.wear", "Usure mecanique mesurable." });
    if (state.meanPistonSpeedMps > 25.0) result.push_back({ DiagnosticSeverity::warning, "mechanical.piston_speed", "Vitesse moyenne des pistons elevee." });
    if (state.peakPistonAccelerationG > 5'500.0)
        result.push_back({ DiagnosticSeverity::critical, "mechanical.piston_acceleration", "Acceleration piston critique pour l'equipage mobile." });
    if (state.clutchTemperatureC >= config.transmission.clutchFailureTemperatureC)
        result.push_back({ DiagnosticSeverity::critical, "driveline.clutch_failure", "Embrayage en surchauffe critique : capacite de couple perdue." });
    else if (state.clutchTemperatureC >= config.transmission.clutchFadeStartTemperatureC)
        result.push_back({ DiagnosticSeverity::warning, "driveline.clutch_fade", "Embrayage surchauffe : la capacite de couple diminue." });
    if (state.tractionLimited && state.throttle > 0.2)
        result.push_back({ DiagnosticSeverity::information, "vehicle.traction_limit", "Force longitudinale limitee par l'adherence disponible." });
    return result;
}
} // namespace enginelab
