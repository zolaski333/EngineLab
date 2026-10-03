#include <enginelab/diagnostics/EngineDiagnostics.hpp>
#include <algorithm>

namespace enginelab {
std::vector<Diagnostic> EngineDiagnostics::evaluate(const EngineConfig& config, const EngineState& state) const {
    std::vector<Diagnostic> result;
    if (state.knockLevel > 0.45) result.push_back({ DiagnosticSeverity::critical, "combustion.knock", "Severe knock: reduce compression, spark advance or load." });
    if (state.coolantTemperatureC > 112.0) result.push_back({ DiagnosticSeverity::warning, "thermal.coolant", "Coolant temperature too high." });
    if (state.misfireRate > 0.2) result.push_back({ DiagnosticSeverity::warning, "combustion.misfire", "Unstable combustion: check mixture and ignition." });
    if (state.rpm > config.redlineRpm) result.push_back({ DiagnosticSeverity::critical, "mechanical.overrev", "Engine speed above the estimated mechanical limit." });
    if (state.rpm > 900.0 && state.oilPressureKpa < 100.0) result.push_back({ DiagnosticSeverity::critical, "lubrication.pressure", "Oil pressure too low under load." });
    if (state.exhaustTemperatureC > 920.0) result.push_back({ DiagnosticSeverity::warning, "thermal.exhaust", "Exhaust temperature too high." });
    if (state.oilTemperatureC > 135.0) result.push_back({ DiagnosticSeverity::critical, "thermal.oil", "Critical oil temperature." });
    if (config.fuel == FuelType::diesel) {
        // A quality-governed diesel is deliberately lean: lambda 1.4-3 under
        // load is normal and is not the gasoline lean-fault this diagnostic
        // historically reported. Its AFR command is instead a rich-side smoke
        // floor. Allow a small cycle/telemetry lag before reporting a breach.
        if (state.airFuelRatioValid && state.load > 0.55
                && state.airFuelRatio > 1.0
                && state.airFuelRatio < state.targetAirFuelRatio * 0.97) {
            result.push_back({ DiagnosticSeverity::warning,
                "combustion.diesel_smoke_limit",
                "Diesel mixture past the smoke limit: reduce the injected quantity." });
        }
    } else {
        if (state.airFuelRatioValid && state.load > 0.55
                && state.lambda > 1.03)
            result.push_back({ DiagnosticSeverity::critical, "combustion.lean", "Mixture too lean under load." });
        if (state.airFuelRatioValid && state.lambda < 0.74)
            result.push_back({ DiagnosticSeverity::warning, "combustion.rich", "Mixture far too rich: oil dilution possible." });
    }
    if (state.solverResolutionLimited)
        result.push_back({ DiagnosticSeverity::critical, "solver.resolution", "Solver crank-angle resolution too coarse at the current engine speed." });
    if (state.load > 0.40 && state.rpm > config.idleRpm * 1.2 && state.cylinderStateCount > 0) {
        // Capacity is unused authored-window fraction; duty is open time over
        // the complete 720-degree cycle. Less than 10 % window headroom or
        // >90 % total-cycle duty means actuator authority is nearly exhausted.
        // Follow the worst cylinder because an average can hide one lean hole.
        double minimumCapacity = 1.0;
        double maximumDuty = 0.0;
        for (std::size_t index = 0; index < state.cylinderStateCount; ++index) {
            minimumCapacity = std::min(minimumCapacity,
                state.cylinderStates[index].injectorCapacityRatio);
            maximumDuty = std::max(maximumDuty,
                state.cylinderStates[index].injectorDutyCycle);
        }
        if (minimumCapacity < 0.10 || maximumDuty > 0.90)
            result.push_back({ DiagnosticSeverity::warning, "injection.capacity", "Injectors saturated: the commanded mass can no longer be fully injected." });
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
                "Turbine drive pressure too high: check turbine and wastegate." });
    } else if (stableHighLoad && backPressureDeltaKpa > 40.0) {
        result.push_back({ DiagnosticSeverity::warning,
            "exhaust.back_pressure", "Exhaust back pressure too high." });
    }
    if (state.damage > 0.5) result.push_back({ DiagnosticSeverity::critical, "mechanical.damage", "Significant mechanical damage: power and reliability degraded." });
    else if (state.wear > 0.35) result.push_back({ DiagnosticSeverity::warning, "mechanical.wear", "Measurable mechanical wear." });
    if (state.meanPistonSpeedMps > 25.0) result.push_back({ DiagnosticSeverity::warning, "mechanical.piston_speed", "High mean piston speed." });
    if (state.peakPistonAccelerationG > 5'500.0)
        result.push_back({ DiagnosticSeverity::critical, "mechanical.piston_acceleration", "Critical piston acceleration for the reciprocating assembly." });
    if (state.clutchTemperatureC >= config.transmission.clutchFailureTemperatureC)
        result.push_back({ DiagnosticSeverity::critical, "driveline.clutch_failure", "Clutch critically overheated: torque capacity lost." });
    else if (state.clutchTemperatureC >= config.transmission.clutchFadeStartTemperatureC)
        result.push_back({ DiagnosticSeverity::warning, "driveline.clutch_fade", "Clutch overheating: torque capacity dropping." });
    if (state.tractionLimited && state.throttle > 0.2)
        result.push_back({ DiagnosticSeverity::information, "vehicle.traction_limit", "Longitudinal force limited by available grip." });
    return result;
}
} // namespace enginelab
