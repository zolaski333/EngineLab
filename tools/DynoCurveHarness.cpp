// Established-point diagnostic acquisition. Work is integrated by the real
// simulator between complete 720-degree boundaries; display telemetry is
// sampled separately at outer-frame cadence and never substitutes for work.
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/simulation/DynoAbsorberController.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {
constexpr auto nan = std::numeric_limits<double>::quiet_NaN();
struct Settings final {
    std::filesystem::path catalogRoot { ENGINELAB_CATALOG_ROOT };
    std::filesystem::path outputPath;
    std::filesystem::path cyclesPath;
    std::filesystem::path provenancePath;
    std::string filter;
    std::string direction { "ascending" };
    std::vector<double> rpmList;
    double startRpm { 0.0 }, endRpm { 0.0 }, stepRpm { 100.0 };
    double settleMin { 3.0 }, settleMax { 30.0 }, sampleSeconds { 2.0 };
    double preheatSeconds { 0.0 };
    double stableWindow { 0.5 }, stableRelative { 0.01 };
    double thermalDriftCPerSecond { 0.5 };
    double outerHz { 240.0 };
    std::size_t minimumCycles { 12 }, stableWindows { 3 };
    std::optional<double> mechanicalMinHz, mechanicalCapHz, crankStepDegrees;
    bool exhaustEverySubstep { false }, schemaOnly { false };
    enginelab::EngineSimulatorOptions simulatorOptions;
};

// Keeping the initial schema identical permits comparisons against the older
// catalogue sweeps. New diagnostics are appended rather than renamed.
constexpr std::string_view legacyHeader =
    "engine,cylinders,target_rpm,actual_rpm,torque_nm,power_kw,power_ps,ve,lambda,"
    "imep_bar,peak_cyl_bar,egt_c,map_kpa,air_mg,fuel_g_s,pmep_bar,gross_imep_bar,"
    "exh_kpa,delivered_ve,exhaust_stroke_mep_bar,intake_stroke_mep_bar,"
    "rpm_min,rpm_max,rpm_stddev,rpm_drift,held_rpm_min,held_rpm_max,"
    "held_rpm_stddev,held_rpm_drift,brake_nm_mean,brake_nm_min,brake_nm_max";
constexpr std::string_view diagnosticHeader =
    ",status,settle_s,sample_s,complete_cycles,invalid_cycles,dropped_cycles,"
    "torque_cycle_stddev,torque_cycle_cv,torque_drift_fraction,ve_drift_fraction,"
    "lambda_drift_fraction,map_drift_fraction,boost_drift_fraction,boost_ratio,"
    "coolant_c,oil_c,intake_runner_c,exhaust_wall_c,coolant_drift_c_s,"
    "oil_drift_c_s,intake_runner_drift_c_s,exhaust_wall_drift_c_s,"
    "injected_fuel_mg,delivered_fuel_mg,fuel_trim,port_vapour_mg,port_film_mg,"
    "fuel_request_mg,metered_fuel_mg,injector_duty,phi_spark,residual_spark,"
    "combustion_efficiency,friction_nm,mean_work_nm,indicated_work_j,brake_work_j,"
    "exhaust_back_kpa,soft_limiter_frames,hard_limiter_frames,spark_cut_frames,"
    "misfire_events,solver_resolution_limited_frames,actual_mechanical_hz_mean,"
    "actual_mechanical_hz_min,actual_mechanical_hz_max,crank_step_deg_max,"
    "exhaust_coupling_hz_mean,exhaust_internal_hz_mean,stable_windows,"
    "sample_start_s,sample_end_s,frame_rpm_mean,intake_accepted_substeps,"
    "intake_rejected_substeps,intake_advanced_s,intake_coupling_flushes,"
    "intake_coupling_advanced_s,intake_coupling_interval_min_s,"
    "intake_coupling_interval_max_s,intake_cylinder_transfer_failures,"
    "intake_plenum_transfer_failures,actual_intake_internal_hz,"
    "actual_intake_coupling_hz,intake_max_accepted_step_ratio,"
    "intake_max_courant_current,intake_max_courant_predictor,"
    "exhaust_max_accepted_step_ratio,exhaust_max_courant_current,"
    "exhaust_max_courant_predictor,ignition_advance_deg,target_afr";

enum Metric : std::size_t {
    rpm, heldRpm, brakeNm, ve, deliveredVe, lambda, imep, peakCyl, egt,
    map, airMg, fuelFlow, pmep, grossImep, exhaustPeak, exhaustStrokeMep,
    boost, coolant, oil, intakeRunnerC, exhaustWallC, injectedFuel,
    deliveredFuel, fuelTrim, portVapour, portFilm, requestedFuel, meteredFuel,
    injectorDuty, phiSpark, residualSpark, combustionEfficiency, friction,
    meanWork, exhaustBack, actualMechanicalHz, crankStep, exhaustCouplingHz,
    exhaustInternalHz, ignitionAdvance, targetAfr, metricCount
};

using Values = std::array<double, metricCount>;
Values readValues(const enginelab::EngineState& s, double held, double brake,
                  double dt) {
    Values v {};
    v[rpm] = s.rpm; v[heldRpm] = held; v[brakeNm] = brake;
    v[ve] = s.volumetricEfficiency;
    v[deliveredVe] = s.deliveredVolumetricEfficiency;
    v[lambda] = s.lambda; v[imep] = s.indicatedMeanEffectivePressureBar;
    v[egt] = s.exhaustTemperatureC; v[map] = s.manifoldPressureKpa;
    v[airMg] = s.airMassMgPerCycle; v[fuelFlow] = s.fuelFlowGramsPerSecond;
    v[pmep] = s.pumpingMeanEffectivePressureBar;
    v[grossImep] = s.grossIndicatedMeanEffectivePressureBar;
    v[exhaustPeak] = s.exhaustPressureKpa;
    v[exhaustStrokeMep] = s.exhaustStrokeMeanEffectivePressureBar;
    v[boost] = s.boostPressureRatio;
    v[coolant] = s.coolantTemperatureC; v[oil] = s.oilTemperatureC;
    v[exhaustWallC] = s.exhaustWallTemperatureC;
    v[injectedFuel] = s.injectedFuelMgPerCycle;
    v[deliveredFuel] = s.deliveredFuelMgPerCycle;
    v[friction] = s.frictionTorqueNm; v[meanWork] = s.meanWorkTorqueNm;
    v[exhaustBack] = s.exhaustBackPressureKpa;
    // Count accepted mechanical advances over the actual outer-frame duration.
    v[actualMechanicalHz] = static_cast<double>(s.solverSubsteps) / dt;
    v[crankStep] = s.crankDegreesPerSolverStep;
    v[exhaustCouplingHz] = s.exhaustCouplingFrequencyHz;
    v[exhaustInternalHz] = static_cast<double>(s.exhaustNetworkAcceptedSubsteps) / dt;
    v[ignitionAdvance] = s.ignitionAdvanceDegrees;
    v[targetAfr] = s.targetAirFuelRatio;
    for (std::size_t index = 0; index < s.cylinderStateCount; ++index) {
        const auto& c = s.cylinderStates[index];
        v[peakCyl] = std::max(v[peakCyl], c.pressureEstimateBar);
        v[intakeRunnerC] += c.intakeRunnerTemperatureC;
        v[fuelTrim] += c.closedLoopFuelTrim;
        v[portVapour] += c.portFuelVapourInventoryMg;
        v[portFilm] += c.portLiquidFilmFuelMg;
        v[requestedFuel] += c.requestedFuelMgPerCycle;
        v[meteredFuel] += c.meteredFuelMgPerCycle;
        v[injectorDuty] += c.injectorDutyCycle;
        v[phiSpark] += c.equivalenceRatioAtSpark;
        v[residualSpark] += c.residualGasFractionAtSpark;
        v[combustionEfficiency] += c.combustionEfficiency;
    }
    const auto count = static_cast<double>(std::max(std::size_t { 1 }, s.cylinderStateCount));
    for (const auto metric : { intakeRunnerC, fuelTrim, portVapour, portFilm,
                              requestedFuel, meteredFuel, injectorDuty, phiSpark,
                              residualSpark, combustionEfficiency })
        v[metric] /= count;
    return v;
}

struct Aggregate final {
    Values sum {}, squares {}, minimum {}, maximum {}, first {}, last {};
    std::vector<Values> frames;
    std::vector<double> cycleTorques;
    double work {}, indicatedWork {}, radians {}, duration {}, start {}, end {};
    std::size_t cycles {}, invalid {}, dropped {}, softLimiter {}, hardLimiter {},
        sparkCut {}, resolutionLimited {};
    std::uint64_t firstMisfires {}, lastMisfires {};
    std::uint64_t intakeAccepted {}, intakeRejected {}, intakeFlushes {},
        intakeCylinderFailures {}, intakePlenumFailures {};
    double intakeAdvancedSeconds {}, intakeCouplingAdvancedSeconds {},
        intakeIntervalMinimum { std::numeric_limits<double>::infinity() },
        intakeIntervalMaximum {};
    std::array<double, 6> maximumCflDiagnostics {};

    explicit Aggregate(double startTime = 0.0) : start(startTime), end(startTime) {
        minimum.fill(std::numeric_limits<double>::infinity());
        maximum.fill(-std::numeric_limits<double>::infinity());
    }
    void addFrame(const enginelab::SimulationFrame& frame, double held, double brake, double dt) {
        const auto values = readValues(frame.state, held, brake, dt);
        if (frames.empty()) { first = values; firstMisfires = frame.state.misfireEventCount; }
        last = values; lastMisfires = frame.state.misfireEventCount;
        for (std::size_t i = 0; i < metricCount; ++i) {
            sum[i] += values[i]; squares[i] += values[i] * values[i];
            minimum[i] = std::min(minimum[i], values[i]);
            maximum[i] = std::max(maximum[i], values[i]);
        }
        frames.push_back(values); end = frame.state.simulationTimeSeconds;
        softLimiter += frame.state.ecuSoftRevLimiterActive ? 1U : 0U;
        hardLimiter += frame.state.ecuHardRevLimiterActive ? 1U : 0U;
        sparkCut += frame.state.ecuAlternatingSparkCutActive ? 1U : 0U;
        resolutionLimited += frame.state.solverResolutionLimited ? 1U : 0U;
        intakeAccepted += frame.state.intakeNetworkAcceptedSubsteps;
        intakeRejected += frame.state.intakeNetworkRejectedSubsteps;
        intakeAdvancedSeconds += frame.state.intakeNetworkAdvancedSeconds;
        intakeFlushes += frame.state.intakeCouplingFlushCount;
        intakeCouplingAdvancedSeconds += frame.state.intakeCouplingAdvancedSeconds;
        if (frame.state.intakeCouplingFlushCount > 0) {
            intakeIntervalMinimum = std::min(intakeIntervalMinimum,
                frame.state.intakeCouplingMinimumIntervalSeconds);
            intakeIntervalMaximum = std::max(intakeIntervalMaximum,
                frame.state.intakeCouplingMaximumIntervalSeconds);
        }
        intakeCylinderFailures += frame.state.intakeCylinderTransferFailures;
        intakePlenumFailures += frame.state.intakePlenumTransferFailures;
        const std::array<double, 6> cflDiagnostics {
            frame.state.intakeNetworkMaximumAcceptedStepRatio,
            frame.state.intakeNetworkMaximumCourantCurrent,
            frame.state.intakeNetworkMaximumCourantPredictor,
            frame.state.exhaustNetworkMaximumAcceptedStepRatio,
            frame.state.exhaustNetworkMaximumCourantCurrent,
            frame.state.exhaustNetworkMaximumCourantPredictor,
        };
        for (std::size_t index = 0; index < cflDiagnostics.size(); ++index)
            maximumCflDiagnostics[index] = std::max(maximumCflDiagnostics[index], cflDiagnostics[index]);
        dropped += frame.droppedCompletedBrakeCycleSampleCount;
        for (std::size_t i = 0; i < frame.completedBrakeCycleSampleCount; ++i) {
            const auto& c = frame.completedBrakeCycleSamples[i];
            if (c.startTimeSeconds + 1.0e-12 < start) continue;
            if (!c.numericallyValid) { ++invalid; continue; }
            ++cycles; work += c.brakeWorkJoules; indicatedWork += c.indicatedWorkJoules;
            radians += c.integratedCrankRadians; duration += c.durationSeconds;
            cycleTorques.push_back(c.meanTorqueNm);
        }
    }
    [[nodiscard]] double mean(Metric i) const {
        return frames.empty() ? nan : sum[i] / static_cast<double>(frames.size());
    }
    [[nodiscard]] double stddev(Metric i) const {
        return std::sqrt(std::max(0.0, squares[i] / static_cast<double>(std::max(
            std::size_t { 1 }, frames.size())) - mean(i) * mean(i)));
    }
    [[nodiscard]] double halfDrift(Metric i) const {
        if (frames.size() < 2) return nan;
        const auto mid = frames.size() / 2;
        double a = 0.0, b = 0.0;
        for (std::size_t j = 0; j < mid; ++j) a += frames[j][i];
        for (std::size_t j = mid; j < frames.size(); ++j) b += frames[j][i];
        return b / static_cast<double>(frames.size() - mid) - a / static_cast<double>(mid);
    }
    [[nodiscard]] double relativeDrift(Metric i) const {
        return halfDrift(i) / std::max(std::abs(mean(i)), 1.0e-9);
    }
    [[nodiscard]] double thermalRate(Metric i) const {
        if (frames.size() < 2 || end <= start) return nan;
        // Means in each half are separated by half of the acquisition duration.
        return halfDrift(i) / ((end - start) * 0.5);
    }
    [[nodiscard]] double torque() const { return radians > 0.0 ? work / radians : nan; }
    [[nodiscard]] double power() const { return duration > 0.0 ? work / duration / 1000.0 : nan; }
    [[nodiscard]] double actualRpm() const {
        return duration > 0.0 ? radians / duration * 60.0 / (2.0 * std::numbers::pi) : nan;
    }
    [[nodiscard]] double torqueHalfDrift() const {
        if (cycleTorques.size() < 2) return nan;
        const auto mid = cycleTorques.size() / 2;
        double a = 0.0, b = 0.0;
        for (std::size_t i = 0; i < mid; ++i) a += cycleTorques[i];
        for (std::size_t i = mid; i < cycleTorques.size(); ++i) b += cycleTorques[i];
        return (b / static_cast<double>(cycleTorques.size() - mid)
            - a / static_cast<double>(mid)) / std::max(std::abs(torque()), 1.0e-9);
    }
    [[nodiscard]] double torqueStddev() const {
        if (cycleTorques.empty()) return nan;
        double sumTorque = 0.0, sumSquare = 0.0;
        for (const auto t : cycleTorques) { sumTorque += t; sumSquare += t * t; }
        const auto count = static_cast<double>(cycleTorques.size());
        return std::sqrt(std::max(0.0, sumSquare / count - std::pow(sumTorque / count, 2.0)));
    }
};

bool stable(const Aggregate& a, const Aggregate* previous, double target, const Settings& settings) {
    if (a.cycles < 3 || a.invalid != 0 || a.dropped != 0 || a.frames.size() < 2
        || !std::isfinite(a.torque()) || a.softLimiter != 0 || a.hardLimiter != 0
        || a.sparkCut != 0 || a.resolutionLimited != 0 || a.lastMisfires != a.firstMisfires
        || a.intakeCylinderFailures != 0 || a.intakePlenumFailures != 0)
        return false;
    if (std::abs(a.mean(heldRpm) - target) > target * 0.01
        || a.maximum[heldRpm] - a.minimum[heldRpm] > target * 0.02
        || std::abs(a.halfDrift(heldRpm)) > target * settings.stableRelative
        || std::abs(a.torqueHalfDrift()) > settings.stableRelative)
        return false;
    for (const auto i : { ve, lambda, map, boost }) {
        if (!std::isfinite(a.mean(i)) || std::abs(a.relativeDrift(i)) > settings.stableRelative)
            return false;
        if (previous && std::abs(a.mean(i) - previous->mean(i))
            > settings.stableRelative * std::max(std::abs(a.mean(i)), 1.0e-9))
            return false;
    }
    if (previous && std::abs(a.torque() - previous->torque())
        > settings.stableRelative * std::max(std::abs(a.torque()), 1.0e-9))
        return false;
    for (const auto i : { coolant, oil, intakeRunnerC, exhaustWallC })
        if (!std::isfinite(a.thermalRate(i))
            || std::abs(a.thermalRate(i)) > settings.thermalDriftCPerSecond)
            return false;
    return true;
}

std::uint64_t hashFile(const std::filesystem::path& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file) throw std::runtime_error("cannot hash " + path.string());
    auto hash = std::uint64_t { 14695981039346656037ULL };
    char c = 0;
    while (file.get(c)) { hash ^= static_cast<unsigned char>(c); hash *= 1099511628211ULL; }
    return hash;
}

std::string hexHash(std::uint64_t value) {
    std::ostringstream output; output << std::hex << std::setw(16) << std::setfill('0') << value;
    return output.str();
}

void prepareParent(const std::filesystem::path& path) {
    if (!path.parent_path().empty()) std::filesystem::create_directories(path.parent_path());
}

void writeSummary(std::ostream& output, const enginelab::EngineConfig& config,
                  double target, const Aggregate& a, std::string_view status,
                  double settleSeconds, std::size_t stableCount) {
    output << std::quoted(config.name) << ',' << config.cylinders.size() << ',' << target;
    const auto emit = [&output](double x) { output << ',' << x; };
    emit(a.actualRpm()); emit(a.torque()); emit(a.power()); emit(a.power() / 0.735499);
    for (const auto i : { ve, lambda, imep }) emit(a.mean(i));
    emit(a.maximum[peakCyl]);
    for (const auto i : { egt, map, airMg, fuelFlow, pmep, grossImep, exhaustPeak,
                         deliveredVe, exhaustStrokeMep }) emit(a.mean(i));
    emit(a.mean(pmep) - a.mean(exhaustStrokeMep));
    for (const auto i : { rpm, heldRpm }) {
        emit(a.minimum[i]); emit(a.maximum[i]); emit(a.stddev(i)); emit(a.halfDrift(i));
    }
    emit(a.mean(brakeNm)); emit(a.minimum[brakeNm]); emit(a.maximum[brakeNm]);
    output << ',' << status << ',' << settleSeconds << ',' << a.end - a.start
        << ',' << a.cycles << ',' << a.invalid << ',' << a.dropped;
    emit(a.torqueStddev()); emit(a.torqueStddev() / std::max(std::abs(a.torque()), 1.0e-9));
    emit(a.torqueHalfDrift());
    for (const auto i : { ve, lambda, map, boost }) emit(a.relativeDrift(i));
    emit(a.mean(boost));
    for (const auto i : { coolant, oil, intakeRunnerC, exhaustWallC }) emit(a.mean(i));
    for (const auto i : { coolant, oil, intakeRunnerC, exhaustWallC }) emit(a.thermalRate(i));
    for (const auto i : { injectedFuel, deliveredFuel, fuelTrim, portVapour, portFilm,
                         requestedFuel, meteredFuel, injectorDuty, phiSpark, residualSpark,
                         combustionEfficiency, friction, meanWork }) emit(a.mean(i));
    emit(a.indicatedWork); emit(a.work); emit(a.mean(exhaustBack));
    output << ',' << a.softLimiter << ',' << a.hardLimiter << ',' << a.sparkCut
        << ',' << a.lastMisfires - a.firstMisfires << ',' << a.resolutionLimited;
    emit(a.mean(actualMechanicalHz)); emit(a.minimum[actualMechanicalHz]);
    emit(a.maximum[actualMechanicalHz]); emit(a.maximum[crankStep]);
    emit(a.mean(exhaustCouplingHz)); emit(a.mean(exhaustInternalHz));
    output << ',' << stableCount; emit(a.start); emit(a.end); emit(a.mean(rpm));
    output << ',' << a.intakeAccepted << ',' << a.intakeRejected;
    emit(a.intakeAdvancedSeconds); output << ',' << a.intakeFlushes;
    emit(a.intakeCouplingAdvancedSeconds);
    emit(std::isfinite(a.intakeIntervalMinimum) ? a.intakeIntervalMinimum : nan);
    emit(a.intakeIntervalMaximum);
    output << ',' << a.intakeCylinderFailures << ',' << a.intakePlenumFailures;
    emit(a.intakeAdvancedSeconds > 0.0
        ? static_cast<double>(a.intakeAccepted) / a.intakeAdvancedSeconds : nan);
    emit(a.intakeCouplingAdvancedSeconds > 0.0
        ? static_cast<double>(a.intakeFlushes) / a.intakeCouplingAdvancedSeconds : nan);
    for (const auto value : a.maximumCflDiagnostics) emit(value);
    emit(a.mean(ignitionAdvance)); emit(a.mean(targetAfr));
    output << '\n' << std::flush;
}

void writeCycleHeader(std::ostream& out) {
    out << "engine,target_rpm,stage,cycle_id,start_s,end_s,duration_s,crank_rad,"
        "indicated_work_j,brake_work_j,actual_rpm,torque_nm,power_kw,valid,"
        "telemetry_time_s,ve,delivered_ve,lambda,imep_bar,pmep_bar,gross_imep_bar,"
        "exhaust_stroke_mep_bar,map_kpa,boost_ratio,egt_c,coolant_c,oil_c,"
        "intake_runner_c,exhaust_wall_c,injected_fuel_mg,delivered_fuel_mg,"
        "fuel_trim,port_vapour_mg,port_film_mg,phi_spark,residual_spark,"
        "combustion_efficiency,soft_limiter,hard_limiter,spark_cut,misfire_count,"
        "frame_raw_rpm,frame_held_rpm,actual_mechanical_hz,exhaust_coupling_hz,"
        "exhaust_internal_hz,intake_accepted_substeps,intake_rejected_substeps,"
        "intake_advanced_s,intake_coupling_flushes,intake_coupling_advanced_s,"
        "intake_coupling_interval_min_s,intake_coupling_interval_max_s,"
        "intake_cylinder_transfer_failures,intake_plenum_transfer_failures,"
        "intake_max_accepted_step_ratio,intake_max_courant_current,"
        "intake_max_courant_predictor,exhaust_max_accepted_step_ratio,"
        "exhaust_max_courant_current,exhaust_max_courant_predictor,"
        "frame_ignition_advance_deg,frame_target_afr,frame_crank_step_deg,"
        "frame_solver_resolution_limited\n";
}

void writeCycles(std::ostream& out, const enginelab::EngineConfig& config, double target,
                 std::string_view stage, const enginelab::SimulationFrame& frame,
                 double held, double brake, double dt) {
    const auto v = readValues(frame.state, held, brake, dt);
    for (std::size_t i = 0; i < frame.completedBrakeCycleSampleCount; ++i) {
        const auto& c = frame.completedBrakeCycleSamples[i];
        out << std::quoted(config.name) << ',' << target << ',' << stage << ',' << c.cycleId
            << ',' << c.startTimeSeconds << ',' << c.endTimeSeconds << ',' << c.durationSeconds
            << ',' << c.integratedCrankRadians << ',' << c.indicatedWorkJoules
            << ',' << c.brakeWorkJoules << ',' << c.meanRpm << ',' << c.meanTorqueNm
            << ',' << c.meanPowerKw << ',' << c.numericallyValid
            << ',' << frame.state.simulationTimeSeconds;
        for (const auto j : { ve, deliveredVe, lambda, imep, pmep, grossImep, exhaustStrokeMep,
                              map, boost, egt, coolant, oil, intakeRunnerC, exhaustWallC,
                              injectedFuel, deliveredFuel, fuelTrim, portVapour, portFilm,
                              phiSpark, residualSpark, combustionEfficiency }) out << ',' << v[j];
        out << ',' << frame.state.ecuSoftRevLimiterActive
            << ',' << frame.state.ecuHardRevLimiterActive
            << ',' << frame.state.ecuAlternatingSparkCutActive
            << ',' << frame.state.misfireEventCount;
        for (const auto j : { rpm, heldRpm, actualMechanicalHz, exhaustCouplingHz,
                              exhaustInternalHz }) out << ',' << v[j];
        out << ',' << frame.state.intakeNetworkAcceptedSubsteps
            << ',' << frame.state.intakeNetworkRejectedSubsteps
            << ',' << frame.state.intakeNetworkAdvancedSeconds
            << ',' << frame.state.intakeCouplingFlushCount
            << ',' << frame.state.intakeCouplingAdvancedSeconds
            << ',' << frame.state.intakeCouplingMinimumIntervalSeconds
            << ',' << frame.state.intakeCouplingMaximumIntervalSeconds
            << ',' << frame.state.intakeCylinderTransferFailures
            << ',' << frame.state.intakePlenumTransferFailures
            << ',' << frame.state.intakeNetworkMaximumAcceptedStepRatio
            << ',' << frame.state.intakeNetworkMaximumCourantCurrent
            << ',' << frame.state.intakeNetworkMaximumCourantPredictor
            << ',' << frame.state.exhaustNetworkMaximumAcceptedStepRatio
            << ',' << frame.state.exhaustNetworkMaximumCourantCurrent
            << ',' << frame.state.exhaustNetworkMaximumCourantPredictor
            << ',' << v[ignitionAdvance] << ',' << v[targetAfr]
            << ',' << frame.state.crankDegreesPerSolverStep
            << ',' << frame.state.solverResolutionLimited;
        out << '\n';
    }
    out << std::flush;
}

void preheat(enginelab::EngineSimulator& simulator, double target,
             const Settings& settings, std::ostream& cyclesOut) {
    if (settings.preheatSeconds <= 0.0) return;
    const auto& config = simulator.config();
    const auto dt = 1.0 / settings.outerHz;
    const auto startTime = simulator.state().simulationTimeSeconds;
    enginelab::DynoAbsorberController absorber(config);
    absorber.reset(simulator.state().rpm, simulator.state().torqueNm);
    while (simulator.state().simulationTimeSeconds - startTime + 1.0e-12 < settings.preheatSeconds) {
        const auto brake = absorber.advance(dt, target, simulator.state());
        enginelab::EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = simulator.state().rpm < 550.0;
        controls.throttle = 1.0;
        controls.dynamometerTorqueNm = brake.brakeTorqueNm;
        const auto frame = simulator.step(dt, controls);
        writeCycles(cyclesOut, config, target, "preheat", frame,
            brake.filteredRpm, brake.brakeTorqueNm, dt);
    }
    std::cerr << config.audioVoicingKey << " preheat at " << target << " rpm: "
        << simulator.state().simulationTimeSeconds - startTime << " s, exhaust wall "
        << simulator.state().exhaustWallTemperatureC << " C\n";
}

void measurePoint(enginelab::EngineSimulator& simulator, double target,
                  const Settings& settings, std::ostream& output, std::ostream& cyclesOut) {
    const auto& config = simulator.config();
    const auto dt = 1.0 / settings.outerHz;
    const auto start = simulator.state().simulationTimeSeconds;
    enginelab::DynoAbsorberController absorber(config);
    absorber.reset(simulator.state().rpm, simulator.state().torqueNm);
    Aggregate window(start);
    std::optional<Aggregate> previous;
    std::size_t stableCount = 0;
    bool established = false;
    double elapsed = 0.0;
    const auto advance = [&](std::string_view stage, Aggregate& destination) {
        const auto brake = absorber.advance(dt, target, simulator.state());
        enginelab::EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = simulator.state().rpm < 550.0;
        controls.throttle = 1.0; controls.dynamometerTorqueNm = brake.brakeTorqueNm;
        const auto frame = simulator.step(dt, controls);
        destination.addFrame(frame, brake.filteredRpm, brake.brakeTorqueNm, dt);
        writeCycles(cyclesOut, config, target, stage, frame,
            brake.filteredRpm, brake.brakeTorqueNm, dt);
        elapsed = simulator.state().simulationTimeSeconds - start;
    };
    while (elapsed + 1.0e-12 < settings.settleMax) {
        advance("settle", window);
        if (window.end - window.start + 1.0e-12 < settings.stableWindow || window.cycles < 3)
            continue;
        const auto settledWindow = elapsed >= settings.settleMin
            && stable(window, previous ? &*previous : nullptr, target, settings);
        stableCount = settledWindow ? stableCount + 1 : 0;
        previous = std::move(window);
        window = Aggregate(simulator.state().simulationTimeSeconds);
        if (stableCount >= settings.stableWindows) { established = true; break; }
    }
    const auto settleSeconds = elapsed;
    Aggregate sample(simulator.state().simulationTimeSeconds);
    // A stalled or otherwise unmeasurable point must not hang the instrument.
    // The bounded extension covers the requested minimum complete cycles.
    const auto acquisitionLimit = std::max(settings.sampleSeconds,
        static_cast<double>(settings.minimumCycles + 2) * 120.0 / std::max(target, 200.0)) + 5.0;
    while (sample.end - sample.start + 1.0e-12 < acquisitionLimit) {
        advance("sample", sample);
        if (sample.end - sample.start + 1.0e-12 >= settings.sampleSeconds
            && sample.cycles >= settings.minimumCycles) break;
    }
    std::string_view status = "ESTABLISHED";
    if (!established) status = "UNSETTLED_TIMEOUT";
    else if (sample.cycles < settings.minimumCycles) status = "INSUFFICIENT_CYCLES";
    else if (!stable(sample, previous ? &*previous : nullptr, target, settings))
        status = "UNSETTLED_ACQUISITION";
    if (sample.invalid != 0 || sample.dropped != 0) status = "INVALID_CYCLES";
    if (sample.softLimiter != 0 || sample.hardLimiter != 0 || sample.sparkCut != 0)
        status = "LIMITER_CONTAMINATED";
    if (sample.lastMisfires != sample.firstMisfires) status = "MISFIRE_CONTAMINATED";
    if (sample.intakeCylinderFailures != 0 || sample.intakePlenumFailures != 0)
        status = "INTAKE_TRANSFER_FAILURE";
    writeSummary(output, config, target, sample, status, settleSeconds, stableCount);
    std::cerr << config.audioVoicingKey << ' ' << target << " rpm: " << status
        << ", settle " << settleSeconds << " s, " << sample.cycles << " complete cycles\n";
}

void usage() {
    std::cerr << "usage: EngineLabDynoCurveHarness [--catalog-root dir] [--filter exact-key]"
        " [--rpm-list 4800,4850] [--start rpm --end rpm --step rpm]"
        " [--direction ascending|descending|independent] [--preheat-seconds s]"
        " [--settle-min s --settle-max s]"
        " [--sample-seconds s --min-cycles n] [--stable-window s --stable-windows n]"
        " [--stable-relative fraction --thermal-drift-c-per-s rate]"
        " [--intake-cell-mm mm --intake-rk2|--intake-euler --intake-coupling-us us]"
        " [--intake-cfl number --intake-workers integer --intake-joint-manifold 0|1]"
        " [--exhaust-every-substep --exhaust-coupling-us us]"
        " [--mechanical-min-hz hz --mechanical-cap-hz hz --crank-step-deg deg --outer-hz hz]"
        " [--exhaust-cell-mm mm]"
        " [--output csv --cycles csv --provenance json] [--schema-only]\n";
}

Settings parse(int argc, char** argv) {
    Settings s;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto next = [&]() -> std::string {
            if (i + 1 >= argc) throw std::runtime_error("missing value for " + arg);
            return argv[++i];
        };
        const auto number = [&]() { return std::stod(next()); };
        if (arg == "--catalog-root") s.catalogRoot = next();
        else if (arg == "--filter") s.filter = next();
        else if (arg == "--rpm-list") {
            std::istringstream list(next()); std::string value;
            while (std::getline(list, value, ',')) s.rpmList.push_back(std::stod(value));
        }
        else if (arg == "--start") s.startRpm = number();
        else if (arg == "--end") s.endRpm = number();
        else if (arg == "--step") s.stepRpm = number();
        else if (arg == "--direction") s.direction = next();
        else if (arg == "--settle-min") s.settleMin = number();
        else if (arg == "--settle-max") s.settleMax = number();
        else if (arg == "--preheat-seconds") s.preheatSeconds = number();
        else if (arg == "--sample-seconds") s.sampleSeconds = number();
        else if (arg == "--min-cycles") s.minimumCycles = std::stoull(next());
        else if (arg == "--stable-window") s.stableWindow = number();
        else if (arg == "--stable-windows") s.stableWindows = std::stoull(next());
        else if (arg == "--stable-relative") s.stableRelative = number();
        else if (arg == "--thermal-drift-c-per-s") s.thermalDriftCPerSecond = number();
        else if (arg == "--intake-cell-mm") s.simulatorOptions.intakeTargetCellLengthM = number() / 1000.0;
        else if (arg == "--intake-rk2") s.simulatorOptions.intakeFirstOrderTimeIntegration = false;
        else if (arg == "--intake-euler") s.simulatorOptions.intakeFirstOrderTimeIntegration = true;
        else if (arg == "--intake-coupling-us") s.simulatorOptions.intakeCouplingIntervalSeconds = number() * 1.0e-6;
        else if (arg == "--intake-cfl") s.simulatorOptions.intakeMaximumCourantNumber = number();
        else if (arg == "--intake-joint-manifold") {
            const auto value = next();
            if (value != "0" && value != "1")
                throw std::runtime_error("intake joint manifold must be 0 or 1");
            s.simulatorOptions.intakeJointManifold = value == "1";
        }
        else if (arg == "--intake-workers") {
            const auto raw = next();
            if (raw.empty() || !std::all_of(raw.begin(), raw.end(), [](char digit) {
                return digit >= '0' && digit <= '9';
            })) throw std::runtime_error("intake workers must be a nonnegative integer");
            s.simulatorOptions.intakeWorkerCount = static_cast<std::size_t>(std::stoull(raw));
        }
        else if (arg == "--exhaust-every-substep") s.exhaustEverySubstep = true;
        else if (arg == "--exhaust-coupling-us") s.simulatorOptions.maximumLowSpeedExhaustCouplingSeconds = number() * 1.0e-6;
        else if (arg == "--mechanical-min-hz") s.mechanicalMinHz = number();
        else if (arg == "--mechanical-cap-hz") s.mechanicalCapHz = number();
        else if (arg == "--exhaust-cell-mm") s.simulatorOptions.exhaustTargetCellLengthM = number() / 1000.0;
        else if (arg == "--crank-step-deg") s.crankStepDegrees = number();
        else if (arg == "--outer-hz") s.outerHz = number();
        else if (arg == "--output") s.outputPath = next();
        else if (arg == "--cycles") s.cyclesPath = next();
        else if (arg == "--provenance") s.provenancePath = next();
        else if (arg == "--schema-only") s.schemaOnly = true;
        else throw std::runtime_error("unknown argument " + arg);
    }
    const auto finitePositive = [](double x) { return std::isfinite(x) && x > 0.0; };
    if (!finitePositive(s.stepRpm) || !finitePositive(s.outerHz)
        || !finitePositive(s.stableWindow) || !finitePositive(s.stableRelative)
        || !finitePositive(s.sampleSeconds) || !finitePositive(s.settleMax)
        || !std::isfinite(s.settleMin) || s.settleMin < 0.0 || s.settleMax < s.settleMin
        || !std::isfinite(s.preheatSeconds) || s.preheatSeconds < 0.0
        || !finitePositive(s.thermalDriftCPerSecond) || s.minimumCycles < 2
        || s.stableWindows < 3)
        throw std::runtime_error("invalid acquisition settings");
    if (s.direction != "ascending" && s.direction != "descending" && s.direction != "independent")
        throw std::runtime_error("invalid direction");
    if (s.mechanicalMinHz && !finitePositive(*s.mechanicalMinHz))
        throw std::runtime_error("invalid mechanical frequency");
    if (s.mechanicalCapHz && !finitePositive(*s.mechanicalCapHz))
        throw std::runtime_error("invalid mechanical frequency cap");
    if (s.crankStepDegrees && !finitePositive(*s.crankStepDegrees))
        throw std::runtime_error("invalid crank step");
    if (s.simulatorOptions.intakeTargetCellLengthM
        && !finitePositive(*s.simulatorOptions.intakeTargetCellLengthM))
        throw std::runtime_error("invalid intake cell length");
    if (s.simulatorOptions.exhaustTargetCellLengthM
        && !finitePositive(*s.simulatorOptions.exhaustTargetCellLengthM))
        throw std::runtime_error("invalid exhaust cell length");
    if (s.simulatorOptions.intakeMaximumCourantNumber
        && (!finitePositive(*s.simulatorOptions.intakeMaximumCourantNumber)
            || *s.simulatorOptions.intakeMaximumCourantNumber > 1.0))
        throw std::runtime_error("intake CFL must be finite and in (0,1]");
    if (s.simulatorOptions.intakeCouplingIntervalSeconds
        && (!std::isfinite(*s.simulatorOptions.intakeCouplingIntervalSeconds)
            || *s.simulatorOptions.intakeCouplingIntervalSeconds < 0.0))
        throw std::runtime_error("invalid intake coupling interval");
    if (s.simulatorOptions.maximumLowSpeedExhaustCouplingSeconds
        && (*s.simulatorOptions.maximumLowSpeedExhaustCouplingSeconds < 25.0e-6
            || *s.simulatorOptions.maximumLowSpeedExhaustCouplingSeconds > 500.0e-6))
        throw std::runtime_error("exhaust coupling interval must be 25..500 us");
    for (const auto x : s.rpmList)
        if (!finitePositive(x)) throw std::runtime_error("invalid rpm list");
    const auto stem = s.outputPath.empty() ? std::filesystem::path("dyno-curve") : s.outputPath;
    if (s.cyclesPath.empty()) s.cyclesPath = stem.string() + ".cycles.csv";
    if (s.provenancePath.empty()) s.provenancePath = stem.string() + ".provenance.json";
    return s;
}

std::vector<double> targets(const Settings& settings, const enginelab::EngineConfig& config) {
    auto result = settings.rpmList;
    const auto ceiling = std::min(0.95 * std::min(config.redlineRpm, config.ignition.revLimitRpm),
        config.ignition.revLimitRpm - 300.0);
    if (result.empty()) {
        const auto start = settings.startRpm > 0.0 ? settings.startRpm
            : std::ceil(config.idleRpm * 1.5 / settings.stepRpm) * settings.stepRpm;
        const auto end = settings.endRpm > 0.0 ? settings.endRpm : ceiling;
        if (end < start) throw std::runtime_error("end rpm precedes start rpm");
        for (auto x = start; x <= end + 1.0e-9; x += settings.stepRpm) result.push_back(x);
    }
    for (const auto x : result)
        if (x > ceiling + 1.0e-9)
            throw std::runtime_error("target " + std::to_string(x) + " rpm exceeds safe ceiling "
                + std::to_string(ceiling) + " for " + config.name);
    std::sort(result.begin(), result.end());
    result.erase(std::unique(result.begin(), result.end()), result.end());
    if (settings.direction == "descending") std::reverse(result.begin(), result.end());
    return result;
}

nlohmann::json intakeGridProvenance(enginelab::EngineConfig config, const Settings& settings) {
    enginelab::normaliseEngineConfig(config);
    // This mirrors buildIntakeRunnerNetwork. The explicit diagnostic lower
    // bound must change here together with that constructor; production95mm
    // and the3..12-cell policy remain separately visible in the evidence.
    constexpr auto minimumTargetM = 0.010;
    const auto requestedTargetM = settings.simulatorOptions.intakeTargetCellLengthM.value_or(0.095);
    const auto effectiveTargetM = std::clamp(requestedTargetM, minimumTargetM, 0.150);
    nlohmann::json result = {
        { "basis", "Mirrored normalized EngineSimulator constructor; all cylinder/path overrides resolved before rounding." },
        { "requested_target_mm", requestedTargetM * 1000.0 },
        { "clamped_target_mm", effectiveTargetM * 1000.0 },
        { "minimum_target_mm", minimumTargetM * 1000.0 },
        { "minimum_cells", 3 }, { "maximum_cells", 12 },
        { "runners", nlohmann::json::array() }
    };
    for (const auto& cylinder : config.cylinders) {
        const auto path = std::find_if(config.intakePaths.begin(), config.intakePaths.end(),
            [&cylinder](const enginelab::IntakePathConfig& candidate) {
                return std::find(candidate.cylinderIds.begin(), candidate.cylinderIds.end(), cylinder.id)
                    != candidate.cylinderIds.end();
            });
        const auto pathIndex = path != config.intakePaths.end()
            ? static_cast<std::size_t>(std::distance(config.intakePaths.begin(), path)) : 0U;
        const auto& intake = pathIndex < config.intakePaths.size()
            ? config.intakePaths[pathIndex].geometry : config.intake;
        const auto lengthM = std::max(0.03, (cylinder.intakeRunnerLengthMm > 0.0
            ? cylinder.intakeRunnerLengthMm : intake.runnerLengthMm) * 0.001);
        auto cells = std::clamp<std::size_t>(
            static_cast<std::size_t>(std::llround(lengthM / effectiveTargetM)), 3, 12);
        if (settings.simulatorOptions.intakeMaximumCellCount)
            cells = std::min(cells, std::clamp<std::size_t>(
                *settings.simulatorOptions.intakeMaximumCellCount, 3, 12));
        result["runners"].push_back({ { "cylinder_id", cylinder.id }, { "path_index", pathIndex },
            { "length_mm", lengthM * 1000.0 }, { "cell_count", cells },
            { "actual_cell_length_mm", lengthM * 1000.0 / static_cast<double>(cells) } });
    }
    return result;
}

std::size_t runEngine(const enginelab::EngineCatalogEntry& entry, const Settings& settings,
               std::ostream& output, std::ostream& cyclesOut) {
    auto config = entry.config;
    enginelab::normaliseEngineConfig(config);
    if (settings.mechanicalMinHz) config.solver.mechanicalFrequencyHz = *settings.mechanicalMinHz;
    if (settings.mechanicalCapHz) config.solver.maximumMechanicalFrequencyHz = *settings.mechanicalCapHz;
    if (settings.crankStepDegrees) config.solver.maximumCrankDegreesPerStep = *settings.crankStepDegrees;
    if (config.solver.mechanicalFrequencyHz * static_cast<double>(config.solver.gasSubsteps)
        > config.solver.maximumMechanicalFrequencyHz)
        throw std::runtime_error("mechanical minimum exceeds configured frequency cap");
    enginelab::SimpleEcuModel ecu;
    enginelab::SimplifiedGasolinePhysics physics;
    enginelab::FourStrokeEventGenerator events;
    auto exhaust = enginelab::ExhaustGraph::makeForEngine(config);
    auto simulator = std::make_unique<enginelab::EngineSimulator>(
        config, ecu, physics, events, exhaust, settings.simulatorOptions);
    simulator->setExhaustCouplingEverySubstep(settings.exhaustEverySubstep);
    const auto coldStart = [&]() {
        const auto dt = 1.0 / settings.outerHz;
        for (auto time = 0.0; time < 2.0; time += dt)
            (void)simulator->step(dt, { true, time < 1.5, 0.55, 0.0 });
    };
    const auto rpmTargets = targets(settings, config);
    if (settings.direction != "independent") {
        coldStart();
        preheat(*simulator, rpmTargets.front(), settings, cyclesOut);
    }
    for (const auto target : rpmTargets) {
        if (settings.direction == "independent") {
            simulator->reset(); coldStart();
            preheat(*simulator, target, settings, cyclesOut);
        }
        measurePoint(*simulator, target, settings, output, cyclesOut);
    }
    return simulator->intakeWorkerCount();
}
} // namespace

int main(int argc, char** argv) {
    try {
        const auto settings = parse(argc, argv);
        std::cout << std::fixed << std::setprecision(6);
        if (settings.schemaOnly) {
            std::cout << legacyHeader << diagnosticHeader << '\n'; return EXIT_SUCCESS;
        }
        auto catalog = enginelab::loadEngineCatalog(settings.catalogRoot);
        if (!catalog.errors.empty()) throw std::runtime_error(catalog.errors.front());
        if (catalog.entries.empty()) throw std::runtime_error("empty catalogue");
        std::vector<const enginelab::EngineCatalogEntry*> selected;
        if (settings.filter.empty()) {
            for (const auto& entry : catalog.entries) selected.push_back(&entry);
        } else {
            const auto selection = enginelab::selectSingleEngineCatalogEntry(catalog.entries, settings.filter);
            if (!selection || !selection.exactMatch)
                throw std::runtime_error("filter must be an exact catalogue key, source stem, or complete engine name");
            selected.push_back(selection.entry);
        }
        nlohmann::json provenance;
        provenance["schema"] = "EngineLabDynoCurveHarness/1";
        provenance["source_revision_at_configure"] = ENGINELAB_CURVE_SOURCE_REVISION;
        provenance["build_identifier"] = std::string(__DATE__) + " " + __TIME__;
        const auto executable = std::filesystem::absolute(argv[0]);
        provenance["executable"] = executable.string();
        provenance["executable_fnv1a64"] = hexHash(hashFile(executable));
        provenance["arguments"] = nlohmann::json::array();
        for (int i = 0; i < argc; ++i) provenance["arguments"].push_back(argv[i]);
        provenance["catalog_root"] = std::filesystem::absolute(settings.catalogRoot).string();
        provenance["acquisition"] = {
            { "settle_min_s", settings.settleMin }, { "settle_max_s", settings.settleMax },
            { "stable_window_s", settings.stableWindow }, { "stable_windows", settings.stableWindows },
            { "relative_drift_threshold", settings.stableRelative },
            { "thermal_drift_c_per_s_threshold", settings.thermalDriftCPerSecond },
            { "sample_seconds", settings.sampleSeconds }, { "minimum_cycles", settings.minimumCycles },
            { "preheat_seconds", settings.preheatSeconds },
            { "outer_hz", settings.outerHz }, { "direction", settings.direction }
        };
        provenance["telemetry_basis"] = {
            { "work", "Exact completed 720-degree EngineSimulator work/angle/time records; each used once." },
            { "summary_state", "Outer-frame sampled EngineState; mean of frames, including latched cylinder-cycle fields." },
            { "cycle_state", "Outer-frame EngineState snapshot when the exact cycle record is delivered; not a substep cycle average." },
            { "crank_step_deg_max", "Maximum reported EngineState crankDegreesPerSolverStep; actual solver violation separately flagged." },
            { "intake_internal_cadence", "Actual accepted/rejected FV steps and advanced simulation time, summed once per advanced intake network (one per path in joint mode, one per cylinder in historical mode); coupling flush count/advanced time and interval extrema." },
            { "ignition_and_afr", "Outer-frame applied ECU commands; cycle rows retain the delivery frame snapshot, not a cycle average or a decomposition of thermal corrections." },
            { "thermal_gate", "Bounded thermal drift, not complete heat-soak equilibrium; rates are reported explicitly." }
        };
        provenance["preheat_policy"] = "WOT absorber at the first target before ascending/descending sweeps; independent resets repeat WOT preheat at each target. Logged as preheat cycles, excluded from settling/sample work.";
        provenance["intake_plenum_evolution"] = "finite_reservoir_conservative_states_at_internal_substeps_and_RK_stages";
        provenance["solver_recipe"] = {
            { "intake_plenum_state_basis", "Actual plenum GasCell/0D conservative mass and energy with its actual volume; evolved at internal finite-volume substeps and Runge-Kutta stages." },
            { "intake_plenum_momentum", "Zero reservoir momentum; no inferred per-path volume or geometry values are declared by this metadata." },
            { "intake_time_integration", settings.simulatorOptions.intakeFirstOrderTimeIntegration.value_or(false) ? "Euler" : "SSP_RK2" },
            { "intake_joint_manifold", settings.simulatorOptions.intakeJointManifold.value_or(true) },
            { "intake_coupling_interval_s", settings.simulatorOptions.intakeCouplingIntervalSeconds.value_or(
                settings.simulatorOptions.intakeJointManifold.value_or(true) ? 0.0 : 0.0004) }
        };
        provenance["requested_solver_overrides"] = nlohmann::json::object();
        auto& overrides = provenance["requested_solver_overrides"];
        const auto& opt = settings.simulatorOptions;
        if (opt.intakeTargetCellLengthM) overrides["intake_cell_m"] = *opt.intakeTargetCellLengthM;
        if (opt.intakeFirstOrderTimeIntegration) overrides["intake_euler"] = *opt.intakeFirstOrderTimeIntegration;
        if (opt.intakeJointManifold) overrides["intake_joint_manifold"] = *opt.intakeJointManifold;
        if (opt.intakeCouplingIntervalSeconds) overrides["intake_coupling_s"] = *opt.intakeCouplingIntervalSeconds;
        if (opt.intakeMaximumCourantNumber) overrides["intake_maximum_courant_number"] = *opt.intakeMaximumCourantNumber;
        if (opt.intakeWorkerCount) overrides["intake_workers"] = *opt.intakeWorkerCount;
        if (opt.maximumLowSpeedExhaustCouplingSeconds) overrides["exhaust_coupling_cap_s"] = *opt.maximumLowSpeedExhaustCouplingSeconds;
        if (opt.exhaustTargetCellLengthM) overrides["exhaust_cell_m"] = *opt.exhaustTargetCellLengthM;
        overrides["exhaust_every_substep"] = settings.exhaustEverySubstep;
        if (settings.mechanicalMinHz) overrides["mechanical_min_hz"] = *settings.mechanicalMinHz;
        if (settings.mechanicalCapHz) overrides["mechanical_cap_hz"] = *settings.mechanicalCapHz;
        if (settings.crankStepDegrees) overrides["crank_step_deg"] = *settings.crankStepDegrees;
        provenance["engines"] = nlohmann::json::array();
        for (const auto* entry : selected) {
            // Validate the entire experiment before creating acquisition files.
            const auto rpmTargets = targets(settings, entry->config);
            provenance["engines"].push_back({ { "name", entry->config.name },
                { "key", entry->config.audioVoicingKey },
                { "config_path", std::filesystem::absolute(entry->sourcePath).string() },
                { "config_fnv1a64", hexHash(hashFile(entry->sourcePath)) },
                { "rpm_targets", rpmTargets },
                { "authored_mechanical_min_hz", entry->config.solver.mechanicalFrequencyHz },
                { "authored_gas_substeps", entry->config.solver.gasSubsteps },
                { "mechanical_cap_hz", entry->config.solver.maximumMechanicalFrequencyHz },
                { "authored_crank_step_deg", entry->config.solver.maximumCrankDegreesPerStep },
                { "effective_mechanical_cap_hz", settings.mechanicalCapHz.value_or(
                    entry->config.solver.maximumMechanicalFrequencyHz) },
                { "effective_mechanical_min_hz", settings.mechanicalMinHz.value_or(
                    entry->config.solver.mechanicalFrequencyHz) },
                { "effective_crank_step_deg", settings.crankStepDegrees.value_or(
                    entry->config.solver.maximumCrankDegreesPerStep) },
                { "effective_intake_grid", intakeGridProvenance(entry->config, settings) },
                { "effective_intake_maximum_courant_number", opt.intakeMaximumCourantNumber.value_or(0.8) },
                { "actual_intake_worker_count", nullptr },
                { "safe_ceiling_rpm", std::min(0.95 * std::min(
                    entry->config.redlineRpm, entry->config.ignition.revLimitRpm),
                    entry->config.ignition.revLimitRpm - 300.0) }
            });
        }
        prepareParent(settings.provenancePath);
        std::ofstream provenanceOut(settings.provenancePath);
        if (!provenanceOut) throw std::runtime_error("cannot open provenance output");
        provenanceOut << provenance.dump(2) << '\n'; provenanceOut.close();
        prepareParent(settings.cyclesPath);
        std::ofstream cyclesOut(settings.cyclesPath);
        if (!cyclesOut) throw std::runtime_error("cannot open cycle output");
        cyclesOut << std::fixed << std::setprecision(9); writeCycleHeader(cyclesOut);
        std::ofstream outputFile;
        std::ostream* output = &std::cout;
        if (!settings.outputPath.empty()) {
            prepareParent(settings.outputPath); outputFile.open(settings.outputPath);
            if (!outputFile) throw std::runtime_error("cannot open summary output");
            outputFile << std::fixed << std::setprecision(6); output = &outputFile;
        }
        *output << legacyHeader << diagnosticHeader << '\n' << std::flush;
        for (std::size_t index = 0; index < selected.size(); ++index) {
            const auto workers = runEngine(*selected[index], settings, *output, cyclesOut);
            // The public getter reports the constructed pool, including serial
            // engines and explicit overrides clamped to cylinder cardinality.
            provenance["engines"][index]["actual_intake_worker_count"] = workers;
            std::ofstream updatedProvenanceOut(settings.provenancePath);
            if (!updatedProvenanceOut) throw std::runtime_error("cannot update provenance output");
            updatedProvenanceOut << provenance.dump(2) << '\n';
        }
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n'; usage(); return EXIT_FAILURE;
    }
}
