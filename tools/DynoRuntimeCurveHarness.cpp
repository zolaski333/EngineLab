/**
 * Diagnostic capture of the application's EngineRuntime dynamometer path.
 *
 * The point file contains the recorded DynoRun. The trace contains sampled,
 * distinct public snapshots and the current live aggregate (including hold
 * aggregates which the run intentionally replaces). Polling gaps are exported;
 * this is not a claim to have observed every internal gate evaluation.
 */

#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/runtime/EngineRuntime.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;

struct Options final {
    std::filesystem::path catalogRoot = ENGINELAB_CATALOG_ROOT;
    std::filesystem::path output;
    std::string filter;
    enginelab::DynoSessionConfig session;
    double durationSeconds { 90.0 };
    double wallTimeoutSeconds { 180.0 };
    double startRpm { 0.0 };
};

[[nodiscard]] std::string csvString(const std::string& value) {
    std::string result = "\"";
    for (const auto character : value) {
        if (character == '"') result += '"';
        result += character;
    }
    return result + '"';
}

[[nodiscard]] std::filesystem::path companionPath(
    const std::filesystem::path& output, const char* suffix) {
    auto result = output;
    result.replace_filename(output.stem().string() + suffix + ".csv");
    return result;
}

void pointHeader(std::ostream& output, bool prefix, bool live = false) {
    if (prefix) output << "engine,engine_key,run_id,point_index,";
    constexpr const char* fields[] = {
        "rpm", "bin_rpm", "torque_nm", "power_kw", "corrected_torque_nm",
        "corrected_power_kw", "atmospheric_correction_factor", "valid",
        "quality_reasons", "window_mean_rpm", "window_min_rpm", "window_max_rpm",
        "window_duration_s", "torque_variance_nm2", "first_cycle_id",
        "last_cycle_id", "accepted_cycle_count", "ve", "afr", "afr_valid",
        "target_afr", "lambda", "map_kpa", "exhaust_peak_kpa", "coolant_c",
        "oil_c", "oil_pressure_kpa", "exhaust_c", "ignition_degrees",
        "fuel_g_s", "air_g_s", "bsfc_g_kwh"
    };
    auto first = true;
    for (const auto* field : fields) {
        if (!first) output << ',';
        if (live) output << "live_";
        output << field;
        first = false;
    }
}

void writePoint(std::ostream& output, const enginelab::DynoPoint& point) {
    output << point.rpm << ',' << point.binRpm << ',' << point.torqueNm
           << ',' << point.powerKw << ',' << point.correctedTorqueNm
           << ',' << point.correctedPowerKw
           << ',' << point.atmosphericCorrectionFactor << ',' << point.valid
           << ',' << point.qualityReasons << ',' << point.windowMeanRpm
           << ',' << point.windowMinimumRpm << ',' << point.windowMaximumRpm
           << ',' << point.windowDurationSeconds
           << ',' << point.torqueVarianceNm2 << ',' << point.firstCycleId
           << ',' << point.lastCycleId << ',' << point.acceptedCycleCount
           << ',' << point.volumetricEfficiency << ',' << point.airFuelRatio
           << ',' << point.airFuelRatioValid << ',' << point.targetAirFuelRatio
           << ',' << point.lambda << ',' << point.manifoldPressureKpa
           << ',' << point.exhaustPressureKpa << ',' << point.coolantTemperatureC
           << ',' << point.oilTemperatureC << ',' << point.oilPressureKpa
           << ',' << point.exhaustTemperatureC << ',' << point.ignitionAdvanceDegrees
           << ',' << point.fuelFlowGramsPerSecond << ',' << point.airFlowGramsPerSecond
           << ',' << point.brakeSpecificFuelConsumptionGPerKwh;
}

void traceHeader(std::ostream& output) {
    output << "engine,engine_key,run_id,sample_index,snapshot_time_s,"
              "snapshot_gap_s,relative_dyno_time_s,active,status,mode,phase,"
              "preparing,recovery_count,quality_reasons,measurement_ready,"
              "target_rpm,controller_target_rpm,rpm,acceleration_rpm_s,"
              "brake_torque_nm,contact_fraction,saturated_low,saturated_high,"
              "progress,window_cycles,window_duration_s,window_mean_rpm,"
              "window_min_rpm,window_max_rpm,solver_resolution_limited,"
              "intake_cylinder_transfer_failures,intake_plenum_transfer_failures,"
              "intake_rejected_substeps,intake_accepted_substeps,"
              "intake_maximum_accepted_step_ratio,intake_maximum_courant_current,"
              "intake_maximum_courant_predictor,exhaust_maximum_accepted_step_ratio,"
              "exhaust_maximum_courant_current,exhaust_maximum_courant_predictor,"
              "state_cycle_torque_nm,"
              "state_torque_nm,state_power_kw,gas_work_torque_nm,friction_nm,"
              "net_imep_bar,pumping_mep_bar,ve,delivered_ve,afr,target_afr,lambda,"
              "map_kpa,boost_ratio,exhaust_back_pressure_kpa,throttle,coolant_c,"
              "oil_c,exhaust_c,fuel_g_s,air_g_s,soft_limiter,hard_limiter,"
              "alternating_spark_cut,commanded_sparks,misfires,live_point_present,";
    pointHeader(output, false, true);
    output << '\n';
}

void writeTrace(std::ostream& output,
                const enginelab::EngineConfig& config,
                const enginelab::EngineState& state,
                const enginelab::DynoRun& run, std::uint64_t sampleIndex,
                double previousTime, double dynoStartTime) {
    output << csvString(config.name) << ',' << csvString(config.audioVoicingKey)
           << ',' << run.id << ',' << sampleIndex
           << ',' << state.simulationTimeSeconds
           << ',' << (sampleIndex == 0 ? 0.0
                        : state.simulationTimeSeconds - previousTime)
           << ',' << state.simulationTimeSeconds - dynoStartTime
           << ',' << state.dynoActive << ',' << static_cast<int>(state.dynoRunStatus)
           << ',' << static_cast<int>(state.dynoMode)
           << ',' << static_cast<int>(state.dynoPhase)
           << ',' << state.dynoPreparing << ',' << state.dynoRecoveryCount
           << ',' << state.dynoQualityReasons << ',' << state.dynoMeasurementReady
           << ',' << state.dynoTargetRpm << ',' << state.dynoControllerTargetRpm
           << ',' << state.rpm << ',' << state.dynoFilteredAccelerationRpmPerSecond
           << ',' << state.dynoBrakeTorqueNm << ',' << state.dynoBrakeContactFraction
           << ',' << state.dynoAbsorberSaturatedLow
           << ',' << state.dynoAbsorberSaturatedHigh << ',' << state.dynoProgress
           << ',' << state.dynoAcceptedCycleCount << ',' << state.dynoWindowDurationSeconds
           << ',' << state.dynoWindowMeanRpm << ',' << state.dynoWindowMinimumRpm
           << ',' << state.dynoWindowMaximumRpm << ',' << state.solverResolutionLimited
           << ',' << state.intakeCylinderTransferFailures
           << ',' << state.intakePlenumTransferFailures
           << ',' << state.intakeNetworkRejectedSubsteps
           << ',' << state.intakeNetworkAcceptedSubsteps
           << ',' << state.intakeNetworkMaximumAcceptedStepRatio
           << ',' << state.intakeNetworkMaximumCourantCurrent
           << ',' << state.intakeNetworkMaximumCourantPredictor
           << ',' << state.exhaustNetworkMaximumAcceptedStepRatio
           << ',' << state.exhaustNetworkMaximumCourantCurrent
           << ',' << state.exhaustNetworkMaximumCourantPredictor
           << ',' << state.cycleAveragedTorqueNm
           << ',' << state.torqueNm << ',' << state.powerKw << ',' << state.meanWorkTorqueNm
           << ',' << state.frictionTorqueNm << ',' << state.indicatedMeanEffectivePressureBar
           << ',' << state.pumpingMeanEffectivePressureBar << ',' << state.volumetricEfficiency
           << ',' << state.deliveredVolumetricEfficiency << ',' << state.airFuelRatio
           << ',' << state.targetAirFuelRatio << ',' << state.lambda
           << ',' << state.manifoldPressureKpa << ',' << state.boostPressureRatio
           << ',' << state.exhaustBackPressureKpa << ',' << state.throttle
           << ',' << state.coolantTemperatureC << ',' << state.oilTemperatureC
           << ',' << state.exhaustTemperatureC << ',' << state.fuelFlowGramsPerSecond
           << ',' << state.airFlowGramsPerSecond << ',' << state.ecuSoftRevLimiterActive
           << ',' << state.ecuHardRevLimiterActive << ',' << state.ecuAlternatingSparkCutActive
           << ',' << state.commandedSparkEventCount << ',' << state.misfireEventCount
           << ',' << !run.points.empty() << ',';
    // These defaults carry no observation when live_point_present is false.
    enginelab::DynoPoint absent;
    absent.valid = false;
    if (run.points.empty()) writePoint(output, absent);
    else writePoint(output, run.points.back());
    output << '\n';
}

void metadataHeader(std::ostream& output) {
    output << "engine,engine_key,run_id,status,stop_reason,calibration_revision,"
              "started_sim_time_s,ended_sim_time_s,mode,accepted_entry_rpm,"
              "accepted_ceiling_rpm,accepted_hold_rpm,accepted_ramp_rpm_s,"
              "accepted_bin_width_rpm,accepted_window_s,accepted_maximum_duration_s,"
              "requested_start_rpm,actual_start_rpm,final_rpm,sampled_rows,"
              "maximum_snapshot_gap_s,point_count,capture_result\n";
}

[[nodiscard]] bool captureOne(const enginelab::EngineConfig& config,
                              const Options& options, std::ostream& points,
                              std::ostream& trace, std::ostream& metadata) {
    const auto physicalLimitRpm = std::min(config.redlineRpm, config.ignition.revLimitRpm);
    const auto ceilingRpm = std::max(500.0,
        std::min(0.95 * physicalLimitRpm, config.ignition.revLimitRpm - 300.0));
    const auto entryRpm = std::min(ceilingRpm,
        std::max({ 1'000.0, config.idleRpm + 400.0,
                   config.cylinders.size() <= 2U ? 1'800.0 : 0.0 }));
    const auto startRpm = options.startRpm > 0.0 ? options.startRpm
        : std::min(ceilingRpm - 300.0, std::max(entryRpm + 900.0, entryRpm * 1.75));

    auto runtime = std::make_unique<enginelab::EngineRuntime>(config);
    runtime->setRealtimeThrottleEnabled(false);
    runtime->setRealtimeLoadProtectionEnabled(false);
    runtime->setDynoMode(options.session.mode);
    // Hold's wheel-control mailbox is live even in an explicitly configured
    // session. Set it as the UI does, so it cannot replace the requested hold
    // with the runtime's default 2,500 rpm on the following tick.
    runtime->setDynoHoldRpm(std::clamp(options.session.holdRpm,
        std::max(500.0, config.idleRpm * 0.6), ceilingRpm));
    runtime->setIgnitionEnabled(true);
    runtime->setStarterEngaged(true);
    runtime->setThrottle(1.0);
    runtime->start();
    auto state = runtime->snapshot();
    const auto wallDeadline = Clock::now()
        + std::chrono::duration<double>(options.wallTimeoutSeconds);
    const auto runUpStartTime = state.simulationTimeSeconds;
    while (state.rpm < startRpm
           && state.simulationTimeSeconds - runUpStartTime < 14.0
           && Clock::now() < wallDeadline) {
        if (state.rpm >= std::max(650.0, config.idleRpm * 0.82))
            runtime->setStarterEngaged(false);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        state = runtime->snapshot();
    }
    const auto actualStartRpm = state.rpm;
    if (state.rpm < startRpm) {
        runtime->stop();
        std::cerr << config.name << ": did not reach start rpm=" << startRpm
                  << "; actual=" << actualStartRpm << '\n';
        return false;
    }
    runtime->setStarterEngaged(false);
    const auto dynoStartTime = state.simulationTimeSeconds;
    runtime->startDyno(options.session);
    auto run = runtime->currentDynoRun();
    double previousTime = -1.0;
    double maximumGap = 0.0;
    std::uint64_t sampledRows = 0;
    while (runtime->dynoRunning()
           && state.simulationTimeSeconds - dynoStartTime < options.durationSeconds
           && Clock::now() < wallDeadline) {
        state = runtime->snapshot();
        if (state.simulationTimeSeconds != previousTime) {
            run = runtime->currentDynoRun();
            writeTrace(trace, config, state, run, sampledRows,
                       previousTime, dynoStartTime);
            if (sampledRows > 0)
                maximumGap = std::max(maximumGap,
                    state.simulationTimeSeconds - previousTime);
            previousTime = state.simulationTimeSeconds;
            ++sampledRows;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    const auto wallTimedOut = Clock::now() >= wallDeadline;
    if (runtime->dynoRunning()) runtime->stopDyno();
    // Operator stop is a mailbox request. Keep the simulation running until it
    // has actually archived the run, rather than exporting its stale precursor.
    const auto stopDeadline = Clock::now() + std::chrono::seconds(10);
    state = runtime->snapshot();
    while ((runtime->dynoRunning() || state.dynoActive) && Clock::now() < stopDeadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        state = runtime->snapshot();
    }
    if (sampledRows > 0)
        maximumGap = std::max(maximumGap, state.simulationTimeSeconds - previousTime);
    const auto history = runtime->dynoHistory();
    if (!history.empty()) run = history.back();
    else run = runtime->currentDynoRun();
    writeTrace(trace, config, state, run, sampledRows,
               previousTime, dynoStartTime);
    runtime->stop();
    for (std::size_t index = 0; index < run.points.size(); ++index) {
        points << csvString(config.name) << ',' << csvString(config.audioVoicingKey)
               << ',' << run.id << ',' << index << ',';
        writePoint(points, run.points[index]);
        points << '\n';
    }
    const auto& accepted = run.sessionConfig;
    const auto expectedStopReason = options.session.mode == enginelab::DynoMode::hold
        ? enginelab::DynoStopReason::operatorFinished
        : enginelab::DynoStopReason::sweepCeilingReached;
    const auto ok = !wallTimedOut && !run.points.empty()
        && run.status == enginelab::DynoRunStatus::completed
        && run.stopReason == expectedStopReason;
    metadata << csvString(config.name) << ',' << csvString(config.audioVoicingKey)
             << ',' << run.id << ',' << static_cast<int>(run.status)
             << ',' << static_cast<int>(run.stopReason) << ',' << run.calibrationRevision
             << ',' << run.startedAtSimulationSeconds << ',' << run.endedAtSimulationSeconds
             << ',' << static_cast<int>(accepted.mode) << ',' << accepted.sweepEntryRpm
             << ',' << accepted.sweepCeilingRpm << ',' << accepted.holdRpm
             << ',' << accepted.rampRateRpmPerSecond << ',' << accepted.binWidthRpm
             << ',' << accepted.rollingWindowSeconds << ',' << accepted.maximumDurationSeconds
             << ',' << startRpm << ',' << actualStartRpm << ',' << state.rpm
             << ',' << sampledRows + 1U << ',' << maximumGap << ',' << run.points.size()
             << ',' << (wallTimedOut ? "wall_timeout" : ok ? "captured" : "failed") << '\n';
    std::cout << config.name << ": " << run.points.size() << " points, status="
              << static_cast<int>(run.status) << ", reason=" << static_cast<int>(run.stopReason)
              << ", snapshots=" << sampledRows + 1U << ", max_gap=" << maximumGap
              << " s\n";
    return ok;
}

[[nodiscard]] double number(const char* input) {
    std::size_t consumed = 0;
    const std::string text = input;
    const auto value = std::stod(text, &consumed);
    if (consumed != text.size() || !std::isfinite(value))
        throw std::invalid_argument("expected a finite number: " + text);
    return value;
}

} // namespace

int main(int argc, char** argv) {
    try {
        Options options;
        for (int index = 1; index < argc; ++index) {
            const std::string argument = argv[index];
            if (argument == "--help") {
                std::cout << "Usage: EngineLabDynoRuntimeCurveHarness --output FILE.csv "
                    "[--filter NAME] [--mode ramp|hold|stepped] [--hold-rpm RPM] "
                    "[--entry-rpm RPM] [--ceiling-rpm RPM] [--ramp-rate RPM_S] "
                    "[--window-seconds S] [--duration-seconds S] "
                    "[--maximum-duration-seconds S] [--wall-timeout-seconds S] "
                    "[--start-rpm RPM] [--catalog-root PATH]\n";
                return 0;
            }
            if (index + 1 >= argc)
                throw std::invalid_argument("missing value for " + argument);
            const auto* value = argv[++index];
            if (argument == "--output") options.output = value;
            else if (argument == "--filter") options.filter = value;
            else if (argument == "--catalog-root") options.catalogRoot = value;
            else if (argument == "--mode") {
                const std::string mode = value;
                if (mode == "ramp") options.session.mode = enginelab::DynoMode::continuousRamp;
                else if (mode == "hold") options.session.mode = enginelab::DynoMode::hold;
                else if (mode == "stepped") options.session.mode = enginelab::DynoMode::steppedCalibration;
                else throw std::invalid_argument("unknown mode: " + mode);
            } else if (argument == "--hold-rpm") options.session.holdRpm = number(value);
            else if (argument == "--entry-rpm") options.session.sweepEntryRpm = number(value);
            else if (argument == "--ceiling-rpm") options.session.sweepCeilingRpm = number(value);
            else if (argument == "--ramp-rate") options.session.rampRateRpmPerSecond = number(value);
            else if (argument == "--window-seconds") options.session.rollingWindowSeconds = number(value);
            else if (argument == "--duration-seconds") options.durationSeconds = number(value);
            else if (argument == "--maximum-duration-seconds") options.session.maximumDurationSeconds = number(value);
            else if (argument == "--wall-timeout-seconds") options.wallTimeoutSeconds = number(value);
            else if (argument == "--start-rpm") options.startRpm = number(value);
            else throw std::invalid_argument("unknown argument: " + argument);
        }
        if (options.output.empty()) throw std::invalid_argument("--output is required");
        if (!(options.durationSeconds > 0.0) || !(options.wallTimeoutSeconds > 0.0))
            throw std::invalid_argument("duration and wall timeout must be positive");
        if (options.session.rampRateRpmPerSecond < 50.0
            || options.session.rampRateRpmPerSecond > 2'000.0
            || options.session.rollingWindowSeconds < 0.10
            || options.session.rollingWindowSeconds > 1.0
            || options.session.maximumDurationSeconds < 30.0
            || options.session.maximumDurationSeconds > 300.0)
            throw std::invalid_argument("protocol outside runtime limits: ramp 50..2000 rpm/s, "
                "window 0.1..1 s, maximum duration 30..300 s");
        if (options.startRpm < 0.0 || options.session.sweepEntryRpm < 0.0
            || options.session.sweepCeilingRpm < 0.0 || options.session.holdRpm <= 0.0)
            throw std::invalid_argument("rpm values must be nonnegative; hold rpm must be positive");
        const auto catalog = enginelab::loadEngineCatalog(options.catalogRoot);
        if (!catalog.errors.empty()) {
            for (const auto& error : catalog.errors) std::cerr << "catalog: " << error << '\n';
            return 2;
        }
        std::vector<const enginelab::EngineCatalogEntry*> entries;
        if (options.filter.empty()) {
            for (const auto& entry : catalog.entries) entries.push_back(&entry);
        } else {
            const auto selected = enginelab::selectSingleEngineCatalogEntry(catalog.entries, options.filter);
            if (!selected) {
                std::cerr << "Engine selector did not identify exactly one entry\n";
                for (const auto* entry : selected.matches)
                    std::cerr << entry->config.audioVoicingKey << "  " << entry->config.name << '\n';
                return 2;
            }
            entries.push_back(selected.entry);
        }
        if (!options.output.parent_path().empty())
            std::filesystem::create_directories(options.output.parent_path());
        std::ofstream points(options.output);
        std::ofstream trace(companionPath(options.output, ".trace"));
        std::ofstream metadata(companionPath(options.output, ".meta"));
        if (!points || !trace || !metadata) throw std::runtime_error("could not open CSV output files");
        points << std::setprecision(17);
        trace << std::setprecision(17);
        metadata << std::setprecision(17);
        pointHeader(points, true);
        points << '\n';
        traceHeader(trace);
        metadataHeader(metadata);
        std::size_t passed = 0;
        for (const auto* entry : entries)
            if (captureOne(entry->config, options, points, trace, metadata)) ++passed;
        return passed == entries.size() && !entries.empty() ? 0 : 1;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 2;
    }
}
