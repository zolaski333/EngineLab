// Deterministic fixed-gear WOT acceleration instrument.
//
// This reproduces the product coupling order (driveline first, engine second)
// while holding one requested gear and disabling automatic shifts. It exists
// to attribute the reported high-rpm torque collapses without confusing them
// with a gear-change torque cut or with the optional tyre-grip model.

#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/runtime/DrivelineModel.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr double stepSeconds = 1.0 / 240.0;

struct TickResult final {
    enginelab::SimulationFrame frame;
    enginelab::DrivelineOutput drive;
};

struct CyclePoint final {
    double timeSeconds {};
    double rpm {};
    double torqueNm {};
    double afr {};
    double targetAfr {};
    double deliveredFuelRatio {};
    double misfireRate {};
    bool clutchUnlocked {};
    bool fuelEnabled {};
    bool sparkEnabled {};
    bool softLimiter {};
    bool hardLimiter {};
    bool alternatingSparkCut {};
    bool solverLimited {};
};

struct SurgeEvent final {
    std::size_t cycleIndex {};
    double rpm {};
    double baselineTorqueNm {};
    double lowTorqueNm {};
    double torqueRatio { 1.0 };
    std::string attribution;
};

struct RunMetrics final {
    std::string name;
    int requestedGearNumber {};
    bool started {};
    bool gearEngaged {};
    bool scanWindowReached {};
    bool reachedLimiter {};
    double minimumScanRpm {};
    double maximumRpm {};
    double minimumTorqueRatio { 1.0 };
    double firstSurgeRpm {};
    double maximumAfrError {};
    double minimumFuelDeliveryRatio { 1.0 };
    double maximumMisfireRate {};
    double maximumExhaustBackPressureKpa {};
    std::size_t preLimiterFuelCutFrames {};
    std::size_t preLimiterSparkCutFrames {};
    std::size_t preLimiterClutchSlipFrames {};
    std::size_t solverLimitedFrames {};
    std::vector<SurgeEvent> surges;
};

TickResult coupledStep(enginelab::EngineSimulator& simulator,
                       enginelab::DrivelineModel& driveline,
                       double throttle, double clutchPressure,
                       bool starter) {
    TickResult result;
    result.drive = driveline.advance(
        stepSeconds, simulator.state(), 0.0, clutchPressure, 0.0);
    enginelab::EngineControls controls;
    controls.ignitionEnabled = true;
    controls.starterEngaged = starter;
    controls.throttle = std::clamp(throttle, 0.0, 1.0)
        * result.drive.torqueCutMultiplier;
    controls.externalTorqueNm = result.drive.engineCouplingTorqueNm;
    controls.externalRotatingInertiaKgM2 =
        result.drive.reflectedRotatingInertiaKgM2;
    result.frame = simulator.step(stepSeconds, controls);
    return result;
}

bool containsCaseInsensitive(const std::string& haystack,
                             const std::string& needle) {
    if (needle.empty()) return true;
    return std::search(
        haystack.begin(), haystack.end(), needle.begin(), needle.end(),
        [](char left, char right) {
            return std::tolower(static_cast<unsigned char>(left))
                == std::tolower(static_cast<unsigned char>(right));
        }) != haystack.end();
}

std::string safeFileStem(std::string name) {
    for (auto& character : name) {
        const auto byte = static_cast<unsigned char>(character);
        if (!std::isalnum(byte) && character != '-' && character != '_')
            character = '_';
    }
    return name;
}

double median(std::vector<double> values) {
    if (values.empty()) return 0.0;
    const auto middle = values.begin()
        + static_cast<std::ptrdiff_t>(values.size() / 2);
    std::nth_element(values.begin(), middle, values.end());
    if ((values.size() & 1U) != 0U) return *middle;
    const auto lower = std::max_element(values.begin(), middle);
    return (*lower + *middle) * 0.5;
}

std::string attributionFor(const CyclePoint& point) {
    if (point.hardLimiter) return "hard_limiter";
    if (point.alternatingSparkCut || point.softLimiter)
        return "soft_limiter";
    if (!point.fuelEnabled) return "fuel_cut";
    if (!point.sparkEnabled) return "spark_cut";
    if (point.misfireRate > 0.20) return "misfire";
    if (point.deliveredFuelRatio < 0.80) return "fuel_delivery";
    if (point.clutchUnlocked) return "clutch_slip";
    if (point.solverLimited) return "solver_resolution";
    return "unattributed";
}

void detectSurges(const std::vector<CyclePoint>& cycles,
                  RunMetrics& metrics) {
    // A user-visible collapse is a one-cycle torque value below 55% of the
    // preceding local median which recovers above 80% within four cycles.
    // Progressive high-rpm torque fall-off therefore cannot trip this gate.
    constexpr std::size_t historyCycles = 5;
    constexpr std::size_t recoveryCycles = 4;
    for (std::size_t index = historyCycles;
         index + 1 < cycles.size(); ++index) {
        if (cycles[index].softLimiter || cycles[index].hardLimiter) continue;
        std::vector<double> history;
        history.reserve(historyCycles);
        for (std::size_t back = historyCycles; back > 0; --back)
            history.push_back(cycles[index - back].torqueNm);
        const auto baseline = median(std::move(history));
        if (baseline <= 5.0) continue;
        const auto ratio = cycles[index].torqueNm / baseline;
        metrics.minimumTorqueRatio = std::min(metrics.minimumTorqueRatio, ratio);
        if (ratio >= 0.55) continue;
        const auto recoveryEnd = std::min(
            cycles.size(), index + recoveryCycles + 1);
        auto recovered = false;
        for (auto candidate = index + 1;
             candidate < recoveryEnd; ++candidate) {
            if (cycles[candidate].torqueNm >= baseline * 0.80) {
                recovered = true;
                break;
            }
        }
        if (!recovered) continue;
        SurgeEvent event;
        event.cycleIndex = index;
        event.rpm = cycles[index].rpm;
        event.baselineTorqueNm = baseline;
        event.lowTorqueNm = cycles[index].torqueNm;
        event.torqueRatio = ratio;
        event.attribution = attributionFor(cycles[index]);
        metrics.surges.push_back(std::move(event));
        if (metrics.firstSurgeRpm <= 0.0)
            metrics.firstSurgeRpm = cycles[index].rpm;
    }
}

RunMetrics measureAcceleration(
    enginelab::EngineConfig config, int gearNumber,
    const std::optional<std::filesystem::path>& csvDirectory,
    bool traceEvents) {
    config.transmission.automaticShifting = false;
    // The product currently keeps the optional grip limiter disabled so the
    // user can load engines without a traction constraint. The diagnostic must
    // reproduce that state and must never silently opt it back in.
    config.vehicle.tyreGripLimitEnabled = false;
    enginelab::normaliseEngineConfig(config);

    enginelab::SimpleEcuModel ecu;
    enginelab::SimplifiedGasolinePhysics physics;
    enginelab::FourStrokeEventGenerator events;
    auto exhaust = enginelab::ExhaustGraph::makeForEngine(config);
    enginelab::EngineSimulator simulator(config, ecu, physics, events, exhaust);
    enginelab::DrivelineModel driveline(config);

    RunMetrics metrics;
    metrics.name = config.name;
    metrics.requestedGearNumber = gearNumber;
    const auto gearIndex = gearNumber - 1;
    if (gearIndex < 0
        || gearIndex >= static_cast<int>(config.transmission.gearRatios.size()))
        return metrics;

    double timeSeconds = 0.0;
    driveline.requestGear(-1);
    for (int tick = 0;
         tick < static_cast<int>(5.0 / stepSeconds);
         ++tick, timeSeconds += stepSeconds) {
        const auto starter = simulator.state().rpm < config.idleRpm * 0.85
            && timeSeconds < 4.0;
        (void)coupledStep(simulator, driveline, 0.0, 0.0, starter);
    }
    metrics.started = simulator.state().rpm >= config.idleRpm * 0.55;
    if (!metrics.started) return metrics;

    // Engage the requested gear directly with the clutch open, then feed it in
    // over two seconds at WOT. This creates a rolling fixed-gear pull without
    // any preceding upshift torque-cut state contaminating the scan.
    driveline.requestGear(gearIndex);
    for (int tick = 0;
         tick < static_cast<int>(2.0 / stepSeconds);
         ++tick, timeSeconds += stepSeconds) {
        const auto elapsed = static_cast<double>(tick) * stepSeconds;
        const auto clutch = std::clamp((elapsed - 0.35) / 1.65, 0.0, 1.0);
        (void)coupledStep(simulator, driveline, 1.0, clutch, false);
    }

    std::ofstream csv;
    if (csvDirectory) {
        std::filesystem::create_directories(*csvDirectory);
        const auto path = *csvDirectory
            / (safeFileStem(config.name) + "-gear-"
                + std::to_string(gearNumber) + ".csv");
        csv.open(path);
        if (!csv)
            throw std::runtime_error("cannot create trace: " + path.string());
        csv << "time_s,rpm,gear,vehicle_mps,cycle_torque_nm,instant_torque_nm,"
               "net_torque_nm,driveline_reaction_nm,coupling_torque_nm,"
               "reflected_inertia_kg_m2,clutch_torque_nm,"
               "clutch_slip_rpm,torque_cut,throttle,map_kpa,boost_pr,"
               "exhaust_backpressure_kpa,afr,target_afr,lambda,"
               "injected_fuel_mg,delivered_fuel_mg,fuel_delivery_ratio,"
               "fuel_correction,misfire_rate,fuel_enabled,spark_enabled,"
               "soft_limiter,hard_limiter,alternating_spark_cut,dfco,"
               "solver_limited,traction_limited";
        for (int cylinder = 0; cylinder < 8; ++cylinder) {
            csv << ",c" << cylinder << "_work_j"
                << ",c" << cylinder << "_efficiency"
                << ",c" << cylinder << "_residual"
                << ",c" << cylinder << "_phi_at_spark"
                << ",c" << cylinder << "_injector_capacity"
                << ",c" << cylinder << "_fuel_delivery_ratio"
                << ",c" << cylinder << "_afr"
                << ",c" << cylinder << "_misfire"
                << ",c" << cylinder << "_spark_events"
                << ",c" << cylinder << "_ignition_events"
                << ",c" << cylinder << "_spark_phase"
                << ",c" << cylinder << "_ignition_phase";
        }
        csv << '\n';
    }

    std::vector<CyclePoint> cycles;
    cycles.reserve(256);
    auto previousAngle = simulator.state().crankAngleDegrees;
    auto limiterSeconds = 0.0;
    const auto scanStartRpm = std::max(
        config.idleRpm * 1.5, config.ignition.revLimitRpm - 3'500.0);
    const auto lockBandRpm = std::max(
        2.0, config.transmission.clutchLockSpeedRpm * 2.0);
    for (int tick = 0;
         tick < static_cast<int>(20.0 / stepSeconds);
         ++tick, timeSeconds += stepSeconds) {
        const auto result = coupledStep(
            simulator, driveline, 1.0, 1.0, false);
        const auto& state = result.frame.state;
        const auto deliveryRatio = state.injectedFuelMgPerCycle > 1.0e-9
            ? state.deliveredFuelMgPerCycle / state.injectedFuelMgPerCycle
            : 1.0;
        const auto afrError = std::abs(state.airFuelRatio
            - state.targetAirFuelRatio)
            / std::max(1.0, state.targetAirFuelRatio);
        const auto lockedInRequestedGear =
            result.drive.engagedGear == gearIndex
            && !result.drive.shiftInProgress
            && std::abs(result.drive.clutchSlipRpm) <= lockBandRpm;
        metrics.gearEngaged = metrics.gearEngaged || lockedInRequestedGear;
        metrics.maximumRpm = std::max(metrics.maximumRpm, state.rpm);

        const auto inScan = result.drive.engagedGear == gearIndex
            && !result.drive.shiftInProgress
            && state.rpm >= scanStartRpm;
        if (inScan) {
            if (!metrics.scanWindowReached)
                metrics.minimumScanRpm = state.rpm;
            metrics.scanWindowReached = true;
            metrics.maximumAfrError = std::max(
                metrics.maximumAfrError, afrError);
            metrics.minimumFuelDeliveryRatio = std::min(
                metrics.minimumFuelDeliveryRatio, deliveryRatio);
            metrics.maximumMisfireRate = std::max(
                metrics.maximumMisfireRate, state.misfireRate);
            metrics.maximumExhaustBackPressureKpa = std::max(
                metrics.maximumExhaustBackPressureKpa,
                state.exhaustBackPressureKpa);
            if (!state.ecuFuelEnabled && !state.ecuSoftRevLimiterActive)
                ++metrics.preLimiterFuelCutFrames;
            if (!state.ecuSparkEnabled && !state.ecuSoftRevLimiterActive)
                ++metrics.preLimiterSparkCutFrames;
            if (std::abs(result.drive.clutchSlipRpm) > lockBandRpm)
                ++metrics.preLimiterClutchSlipFrames;
            if (state.solverResolutionLimited)
                ++metrics.solverLimitedFrames;
        }

        if (csv) {
            csv << timeSeconds << ',' << state.rpm << ','
                << result.drive.engagedGear + 1 << ','
                << result.drive.vehicleSpeedMps << ','
                << state.cycleAveragedTorqueNm << ',' << state.torqueNm << ','
                << state.netTorqueNm << ','
                << result.drive.engineReactionTorqueNm << ','
                << result.drive.engineCouplingTorqueNm << ','
                << result.drive.reflectedRotatingInertiaKgM2 << ','
                << result.drive.clutchTorqueNm << ','
                << result.drive.clutchSlipRpm << ','
                << result.drive.torqueCutMultiplier << ','
                << state.throttle << ',' << state.manifoldPressureKpa << ','
                << state.boostPressureRatio << ','
                << state.exhaustBackPressureKpa << ','
                << state.airFuelRatio << ',' << state.targetAirFuelRatio << ','
                << state.lambda << ',' << state.injectedFuelMgPerCycle << ','
                << state.deliveredFuelMgPerCycle << ',' << deliveryRatio << ','
                << state.ecuFuelCorrection << ',' << state.misfireRate << ','
                << state.ecuFuelEnabled << ',' << state.ecuSparkEnabled << ','
                << state.ecuSoftRevLimiterActive << ','
                << state.ecuHardRevLimiterActive << ','
                << state.ecuAlternatingSparkCutActive << ','
                << state.ecuDecelerationFuelCutActive << ','
                << state.solverResolutionLimited << ','
                << result.drive.tractionLimited;
            for (std::size_t cylinder = 0; cylinder < 8; ++cylinder) {
                if (cylinder < state.cylinderStateCount) {
                    const auto& cylinderState = state.cylinderStates[cylinder];
                    csv << ',' << cylinderState.indicatedWorkJoulesPerCycle
                        << ',' << cylinderState.combustionEfficiency
                        << ',' << cylinderState.residualGasFractionAtSpark
                        << ',' << cylinderState.equivalenceRatioAtSpark
                        << ',' << cylinderState.injectorCapacityRatio
                        << ',' << cylinderState.fuelDeliveryRatio
                        << ',' << cylinderState.airFuelRatio
                        << ',' << cylinderState.misfiring
                        << ',' << cylinderState.commandedSparkEventsLastCycle
                        << ',' << cylinderState.completedIgnitionEventsLastCycle
                        << ',' << cylinderState.commandedSparkPhaseLastCycle
                        << ',' << cylinderState.completedIgnitionPhaseLastCycle;
                } else {
                    csv << ",,,,,,,,,,,,";
                }
            }
            csv << '\n';
        }

        const auto cycleWrapped = state.crankAngleDegrees < previousAngle;
        previousAngle = state.crankAngleDegrees;
        // Keep the regression window outside the limiter's own deliberately
        // discontinuous cycle. A cycle which entered the 220 rpm soft-cut band
        // can complete just below its threshold after losing work, so a 300 rpm
        // guard is required instead of consulting only the final frame flag.
        if (inScan && cycleWrapped
            && state.rpm < config.ignition.revLimitRpm - 300.0) {
            CyclePoint point;
            point.timeSeconds = timeSeconds;
            point.rpm = state.rpm;
            point.torqueNm = state.cycleAveragedTorqueNm;
            point.afr = state.airFuelRatio;
            point.targetAfr = state.targetAirFuelRatio;
            point.deliveredFuelRatio = deliveryRatio;
            point.misfireRate = state.misfireRate;
            point.clutchUnlocked =
                std::abs(result.drive.clutchSlipRpm) > lockBandRpm;
            point.fuelEnabled = state.ecuFuelEnabled;
            point.sparkEnabled = state.ecuSparkEnabled;
            point.softLimiter = state.ecuSoftRevLimiterActive;
            point.hardLimiter = state.ecuHardRevLimiterActive;
            point.alternatingSparkCut =
                state.ecuAlternatingSparkCutActive;
            point.solverLimited = state.solverResolutionLimited;
            cycles.push_back(point);
        }

        if (state.ecuSoftRevLimiterActive
            || state.ecuHardRevLimiterActive) {
            metrics.reachedLimiter = true;
            limiterSeconds += stepSeconds;
        }
        if (limiterSeconds >= 0.6) break;
        if (metrics.scanWindowReached
            && state.rpm < metrics.minimumScanRpm * 0.70) break;
    }

    detectSurges(cycles, metrics);
    if (traceEvents) {
        for (const auto& surge : metrics.surges) {
            std::printf(
                "    surge @ %.0f rpm: %.1f -> %.1f Nm (%.1f%%), %s\n",
                surge.rpm, surge.baselineTorqueNm, surge.lowTorqueNm,
                surge.torqueRatio * 100.0, surge.attribution.c_str());
        }
    }
    return metrics;
}
} // namespace

int main(int argc, char** argv) {
    std::string filter;
    int gearNumber = 2;
    bool traceEvents = false;
    std::optional<std::filesystem::path> csvDirectory;
    std::filesystem::path catalogRoot = ENGINELAB_CATALOG_ROOT;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--filter" && index + 1 < argc)
            filter = argv[++index];
        else if (argument == "--gear" && index + 1 < argc)
            gearNumber = std::stoi(argv[++index]);
        else if (argument == "--csv-dir" && index + 1 < argc)
            csvDirectory = std::filesystem::path(argv[++index]);
        else if (argument == "--catalog-root" && index + 1 < argc)
            catalogRoot = argv[++index];
        else if (argument == "--trace") traceEvents = true;
        else {
            std::cerr
                << "usage: EngineLabLoadedAccelerationHarness"
                   " [--filter NAME] [--gear 2|3] [--csv-dir DIR]"
                   " [--catalog-root DIR] [--trace]\n";
            return EXIT_FAILURE;
        }
    }
    if (gearNumber < 1) {
        std::cerr << "FAILED: gear must be a positive forward gear\n";
        return EXIT_FAILURE;
    }

    const auto catalog = enginelab::loadEngineCatalog(catalogRoot);
    if (!catalog.errors.empty()) {
        std::cerr << "FAILED: catalogue load: "
                  << catalog.errors.front() << '\n';
        return EXIT_FAILURE;
    }

    std::printf(
        "fixed-gear WOT acceleration, gear %d (automatic shifts OFF, grip limiter OFF)\n",
        gearNumber);
    std::printf(
        "  %-32s start gear scan limiter maxRpm surges firstRpm minTq%% afrErr%% fuel%% misfire%% fuelCut sparkCut clutchSlip solver\n",
        "engine");
    auto matched = false;
    auto allRunnable = true;
    for (const auto& entry : catalog.entries) {
        if (!containsCaseInsensitive(entry.config.name, filter)) continue;
        matched = true;
        if (gearNumber
            > static_cast<int>(entry.config.transmission.gearRatios.size())) {
            std::printf("  %-32s   n/a (gear unavailable)\n",
                entry.config.name.c_str());
            continue;
        }
        try {
            const auto metrics = measureAcceleration(
                entry.config, gearNumber, csvDirectory, traceEvents);
            std::printf(
                "  %-32s %5s %4s %4s %7s %6.0f %6zu %8.0f %6.1f %7.1f %5.1f %8.1f %7zu %8zu %10zu %6zu\n",
                metrics.name.c_str(), metrics.started ? "yes" : "NO",
                metrics.gearEngaged ? "yes" : "NO",
                metrics.scanWindowReached ? "yes" : "NO",
                metrics.reachedLimiter ? "yes" : "no",
                metrics.maximumRpm, metrics.surges.size(),
                metrics.firstSurgeRpm,
                metrics.minimumTorqueRatio * 100.0,
                metrics.maximumAfrError * 100.0,
                metrics.minimumFuelDeliveryRatio * 100.0,
                metrics.maximumMisfireRate * 100.0,
                metrics.preLimiterFuelCutFrames,
                metrics.preLimiterSparkCutFrames,
                metrics.preLimiterClutchSlipFrames,
                metrics.solverLimitedFrames);
            allRunnable = allRunnable && metrics.started
                && metrics.gearEngaged && metrics.scanWindowReached
                && metrics.surges.empty();
        } catch (const std::exception& exception) {
            std::cerr << "FAILED: " << entry.config.name << ": "
                      << exception.what() << '\n';
            allRunnable = false;
        }
    }
    if (!matched) {
        std::cerr << "FAILED: no catalogue engine matched filter\n";
        return EXIT_FAILURE;
    }
    return allRunnable ? EXIT_SUCCESS : EXIT_FAILURE;
}
