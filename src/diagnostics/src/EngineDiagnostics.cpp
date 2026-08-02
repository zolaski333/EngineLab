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
    if (config.fuel == FuelType::diesel) {
        // A quality-governed diesel is deliberately lean: lambda 1.4-3 under
        // load is normal and is not the gasoline lean-fault this diagnostic
        // historically reported. Its AFR command is instead a rich-side smoke
        // floor. Allow a small cycle/telemetry lag before reporting a breach.
        if (state.load > 0.55 && state.airFuelRatio > 1.0
                && state.airFuelRatio < state.targetAirFuelRatio * 0.97) {
            result.push_back({ DiagnosticSeverity::warning,
                "combustion.diesel_smoke_limit",
                "Richesse Diesel au-dela de la limite fumee : reduisez la quantite injectee." });
        }
    } else {
        if (state.load > 0.55 && state.lambda > 1.03)
            result.push_back({ DiagnosticSeverity::critical, "combustion.lean", "Melange trop pauvre sous charge." });
        if (state.lambda < 0.74)
            result.push_back({ DiagnosticSeverity::warning, "combustion.rich", "Melange excessivement riche : dilution d'huile possible." });
    }
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
    // A naturally aspirated engine can be judged against ambient pressure. A
    // turbo cannot: the turbine is deliberately upstream of the cat-back and
    // needs drive pressure to make boost. Comparing its manifold to ambient made
    // every healthy boosted engine look obstructed and suggested enlarging the
    // wrong part of the exhaust.
    const auto backPressureDeltaKpa =
        state.exhaustBackPressureKpa - config.ambientPressureKpa;
    const auto turbocharged = config.forcedInduction.enabled
        && config.forcedInduction.type == ForcedInductionType::turbocharger;
    const auto stableHighLoad = state.load > 0.45 && state.rpm > config.idleRpm * 1.25;
    if (turbocharged) {
        // Diagnose a turbo by drive-pressure ratio once it is in the post-spool
        // operating region. The catalogue's same-hour loaded runs sit between
        // 1.1 and 1.4; 2:1 is kept as the conservative fault boundary. Transient
        // pressure before spool is intentionally ignored.
        const auto postSpool = state.rpm >= config.forcedInduction.fullBoostRpm * 0.85;
        const auto intakeReferenceKpa = std::max(config.ambientPressureKpa,
            state.manifoldPressureKpa);
        const auto drivePressureRatio = state.exhaustBackPressureKpa
            / std::max(1.0, intakeReferenceKpa);
        if (stableHighLoad && postSpool && drivePressureRatio > 2.0)
            result.push_back({ DiagnosticSeverity::warning,
                "exhaust.back_pressure",
                "Pression motrice turbo excessive : verifiez turbine et wastegate." });
    } else if (stableHighLoad && backPressureDeltaKpa > 40.0) {
        result.push_back({ DiagnosticSeverity::warning,
            "exhaust.back_pressure", "Contre-pression d'echappement excessive." });
    }
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
