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
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
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
    bool afrValid {};
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
    double maximumHighRpmAfrError {};
    double afrErrorSum {};
    double minimumObservedAfr { std::numeric_limits<double>::infinity() };
    double minimumFuelDeliveryRatio { 1.0 };
    double minimumInjectorCapacityRatio { 1.0 };
    double maximumMisfireRate {};
    double misfireRateSum {};
    std::uint64_t preLimiterMisfireEvents {};
    std::size_t validAfrFrames {};
    std::size_t invalidAfrFrames {};
    double maximumExhaustBackPressureKpa {};
    std::size_t preLimiterFuelCutFrames {};
    std::size_t preLimiterSparkCutFrames {};
    std::size_t preLimiterClutchSlipFrames {};
    std::size_t solverLimitedFrames {};
    bool dfcoTipInRequested {};
    bool dfcoObserved {};
    bool postDfcoAfrRecovered {};
    double preDfcoWarmIdleRpm {};
    double preDfcoLiftRpm {};
    double postDfcoLiftRpm {};
    double postDfcoTimeToValidSeconds {};
    double postDfcoMaximumAfrError {};
    double postDfcoMinimumAfr { std::numeric_limits<double>::infinity() };
    double postDfcoMaximumMisfireRate {};
    std::uint64_t postDfcoCommandedSparkEvents {};
    std::uint64_t postDfcoMisfireEvents {};
    double postDfcoLastMisfireAfr {};
    double postDfcoLastMisfireTimeSeconds {};
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

struct StandingStartMetrics final {
    std::string name;
    bool started {};
    double idleRpm {};
    double launchRpm {};
    double minimumRpm { std::numeric_limits<double>::infinity() };
    double finalRpm {};
    double minimumAfr { std::numeric_limits<double>::infinity() };
    std::uint64_t misfireEvents {};
};

/** Standing start from idle: requested gear, WOT, clutch fed in over 0.9 s.
 *
 * This is the launch the afterfire harness and a user at the keyboard make. It
 * is harsher than the rolling pull above (0.35 s delay, 1.65 s ramp), and it
 * is the one d70e8b8 broke: with the high-load fuel cell starting at unity, the
 * CP2 twins and the LS3 flooded to AFR 5-7 within 0.2 s and stalled, while no
 * existing test launched from idle this quickly.
 */
StandingStartMetrics measureStandingStart(
    enginelab::EngineConfig config, int gearNumber, bool traceEvents,
    double launchAtSeconds) {
    config.transmission.automaticShifting = false;
    config.vehicle.tyreGripLimitEnabled = false;
    enginelab::normaliseEngineConfig(config);

    enginelab::SimpleEcuModel ecu;
    enginelab::SimplifiedGasolinePhysics physics;
    enginelab::FourStrokeEventGenerator events;
    auto exhaust = enginelab::ExhaustGraph::makeForEngine(config);
    enginelab::EngineSimulator simulator(config, ecu, physics, events, exhaust);
    enginelab::DrivelineModel driveline(config);

    StandingStartMetrics metrics;
    metrics.name = config.name;
    metrics.idleRpm = config.idleRpm;
    // Same crank as the afterfire harness: starter until 3 s, launch at 4 s by
    // default, while the post-start state is still decaying.
    driveline.requestGear(-1);
    auto timeSeconds = 0.0;
    for (int tick = 0; tick < static_cast<int>(launchAtSeconds / stepSeconds);
         ++tick, timeSeconds += stepSeconds) {
        const auto starter = simulator.state().rpm < config.idleRpm * 0.85
            && timeSeconds < 3.0;
        (void)coupledStep(simulator, driveline, 0.0, 0.0, starter);
    }
    metrics.started = simulator.state().rpm >= config.idleRpm * 0.55;
    metrics.launchRpm = simulator.state().rpm;
    if (!metrics.started) return metrics;

    driveline.requestGear(gearNumber - 1);
    const auto misfiresBefore = simulator.state().misfireEventCount;
    for (int tick = 0; tick < static_cast<int>(2.4 / stepSeconds);
         ++tick, timeSeconds += stepSeconds) {
        const auto clutch = std::clamp(
            static_cast<double>(tick) * stepSeconds / 0.9, 0.0, 1.0);
        const auto result = coupledStep(simulator, driveline, 1.0, clutch, false);
        const auto& state = result.frame.state;
        metrics.minimumRpm = std::min(metrics.minimumRpm, state.rpm);
        if (state.airFuelRatio > 0.0)
            metrics.minimumAfr = std::min(metrics.minimumAfr, state.airFuelRatio);
        if (traceEvents && tick % 12 == 0)
            std::printf("    t=%.3f rpm=%.0f clutch=%.2f afr=%.2f misfires=%llu"
                        " | c0 air=%.1f req=%.2f met=%.2f del=%.2f portV=%.2f"
                        " film=%.2f trim=%.3f\n",
                timeSeconds, state.rpm, clutch, state.airFuelRatio,
                static_cast<unsigned long long>(state.misfireEventCount),
                state.cylinderStates[0].trappedFreshAirMassMg,
                state.cylinderStates[0].requestedFuelMgPerCycle,
                state.cylinderStates[0].meteredFuelMgPerCycle,
                state.cylinderStates[0].deliveredFuelMgPerCycle,
                state.cylinderStates[0].portFuelVapourInventoryMg,
                state.cylinderStates[0].portLiquidFilmFuelMg,
                state.cylinderStates[0].closedLoopFuelTrim);
    }
    metrics.finalRpm = simulator.state().rpm;
    metrics.misfireEvents = simulator.state().misfireEventCount - misfiresBefore;
    return metrics;
}

RunMetrics measureAcceleration(
    enginelab::EngineConfig config, int gearNumber,
    const std::optional<std::filesystem::path>& csvDirectory,
    bool traceEvents, bool dfcoTipIn) {
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
    metrics.dfcoTipInRequested = dfcoTipIn;
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

    if (dfcoTipIn) {
        // Test an actual running-engine overrun, not the ECU's protected
        // after-start flare. Launch first while the car is rolling, then hold a
        // moderate loaded speed for twice the longest cold after-start decay.
        // Waiting in neutral made this a second-gear standing-start test and
        // stalled the 2JZ/EJ25 before DFCO could even be exercised.
        const auto warmTargetRpm = std::min(
            config.ignition.revLimitRpm - 2'000.0,
            std::max(config.idleRpm * 3.0,
                config.forcedInduction.fullBoostRpm - 500.0));
        for (int tick = 0;
             tick < static_cast<int>(12.0 / stepSeconds);
             ++tick, timeSeconds += stepSeconds) {
            const auto throttle = std::clamp(
                0.18 + (warmTargetRpm - simulator.state().rpm) / 2'000.0,
                0.0, 0.60);
            (void)coupledStep(simulator, driveline, throttle, 1.0, false);
        }
        metrics.preDfcoWarmIdleRpm = simulator.state().rpm;
        const auto targetRpm = std::min(
            config.ignition.revLimitRpm - 1'200.0,
            std::max(config.forcedInduction.fullBoostRpm + 500.0,
                config.idleRpm * 2.5));
        for (int tick = 0;
             tick < static_cast<int>(6.0 / stepSeconds)
                && simulator.state().rpm < targetRpm;
             ++tick, timeSeconds += stepSeconds) {
            (void)coupledStep(simulator, driveline, 1.0, 1.0, false);
        }
        metrics.preDfcoLiftRpm = simulator.state().rpm;
        for (int tick = 0;
             tick < static_cast<int>(1.2 / stepSeconds);
             ++tick, timeSeconds += stepSeconds) {
            const auto result = coupledStep(
                simulator, driveline, 0.0, 1.0, false);
            metrics.dfcoObserved = metrics.dfcoObserved
                || result.frame.state.ecuDecelerationFuelCutActive
                || !result.frame.state.ecuFuelEnabled;
        }
        metrics.postDfcoLiftRpm = simulator.state().rpm;
        auto observableCommandedSparkEventsAtStart = std::uint64_t { 0 };
        auto observableMisfireEventsAtStart = std::uint64_t { 0 };
        auto lastObservedMisfireEvents = std::uint64_t { 0 };
        auto validStart = -1.0;
        for (int tick = 0;
             tick < static_cast<int>(2.0 / stepSeconds);
             ++tick, timeSeconds += stepSeconds) {
            const auto result = coupledStep(
                simulator, driveline, 1.0, 1.0, false);
            const auto& state = result.frame.state;
            const auto elapsed = static_cast<double>(tick) * stepSeconds;
            if (validStart >= 0.0
                    && (!state.airFuelRatioValid
                        || !state.ecuFuelEnabled || !state.ecuSparkEnabled
                        || state.ecuSoftRevLimiterActive
                        || state.ecuHardRevLimiterActive))
                break;
            if (traceEvents && !state.airFuelRatioValid && tick % 2 == 0) {
                std::printf(
                    "    tip-in t=%.3f AFR=invalid lastMisfireAFR=%.2f "
                    "misfireEvents=%llu\n",
                    elapsed, state.lastMisfireAirFuelRatio,
                    static_cast<unsigned long long>(state.misfireEventCount));
                for (std::size_t cylinder = 0;
                     cylinder < state.cylinderStateCount; ++cylinder) {
                    const auto& cylinderState = state.cylinderStates[cylinder];
                    std::printf(
                        "      c%zu phase=%6.1f command=%s AFR=%s%6.2f "
                        "request=%6.2fmg delivered=%6.2fmg\n",
                        cylinder, cylinderState.cyclePhaseDegrees,
                        cylinderState.combustionCommandAvailable ? "yes" : "cut",
                        cylinderState.airFuelRatioValid ? "" : "invalid/",
                        cylinderState.airFuelRatio,
                        cylinderState.requestedFuelMgPerCycle,
                        cylinderState.deliveredFuelMgPerCycle);
                }
            }
            if (!state.airFuelRatioValid) continue;
            if (validStart < 0.0) {
                validStart = elapsed;
                metrics.postDfcoTimeToValidSeconds = elapsed;
                observableCommandedSparkEventsAtStart =
                    state.commandedSparkEventCount;
                observableMisfireEventsAtStart = state.misfireEventCount;
                lastObservedMisfireEvents = state.misfireEventCount;
            }
            const auto error = std::abs(state.airFuelRatio
                - state.targetAirFuelRatio)
                / std::max(1.0, state.targetAirFuelRatio);
            metrics.postDfcoMaximumAfrError = std::max(
                metrics.postDfcoMaximumAfrError, error);
            metrics.postDfcoMinimumAfr = std::min(
                metrics.postDfcoMinimumAfr, state.airFuelRatio);
            metrics.postDfcoMaximumMisfireRate = std::max(
                metrics.postDfcoMaximumMisfireRate,
                state.misfireRate);
            metrics.postDfcoCommandedSparkEvents =
                state.commandedSparkEventCount
                    - observableCommandedSparkEventsAtStart;
            metrics.postDfcoMisfireEvents = state.misfireEventCount
                - observableMisfireEventsAtStart;
            if (traceEvents
                    && state.misfireEventCount > lastObservedMisfireEvents) {
                std::printf(
                    "    NEW post-DFCO misfire t=%.3f rpm=%.0f AFR=%.3f "
                    "target=%.3f lastCylinderAFR=%.3f events=%llu (+%llu)\n",
                    elapsed, state.rpm, state.airFuelRatio,
                    state.targetAirFuelRatio, state.lastMisfireAirFuelRatio,
                    static_cast<unsigned long long>(metrics.postDfcoMisfireEvents),
                    static_cast<unsigned long long>(state.misfireEventCount
                        - lastObservedMisfireEvents));
                for (std::size_t cylinder = 0;
                     cylinder < state.cylinderStateCount; ++cylinder) {
                    const auto& cylinderState = state.cylinderStates[cylinder];
                    std::printf(
                        "      c%zu%s phase=%6.1f air=%6.1fmg request=%6.2fmg "
                        "metered=%6.2fmg delivered=%6.2fmg portV=%5.2fmg "
                        "local=%5.2f/%5.2fmg film=%5.2fmg AFR=%s%6.2f "
                        "phi=%5.2f trim=%5.3f\n",
                        cylinder, cylinderState.misfiring ? " MISFIRE" : "",
                        cylinderState.cyclePhaseDegrees,
                        cylinderState.trappedFreshAirMassMg,
                        cylinderState.requestedFuelMgPerCycle,
                        cylinderState.meteredFuelMgPerCycle,
                        cylinderState.deliveredFuelMgPerCycle,
                        cylinderState.portFuelVapourInventoryMg,
                        cylinderState.portInjectorFootprintFuelMg,
                        cylinderState.portInjectorFootprintTargetFuelMg,
                        cylinderState.portLiquidFilmFuelMg,
                        cylinderState.airFuelRatioValid ? "" : "invalid/",
                        cylinderState.airFuelRatio,
                        cylinderState.equivalenceRatioAtSpark,
                        cylinderState.closedLoopFuelTrim);
                }
            }
            lastObservedMisfireEvents = state.misfireEventCount;
            const auto traceStride = elapsed <= 0.25 ? 6 : 24;
            if (traceEvents && tick % traceStride == 0) {
                std::printf(
                    "    tip-in t=%.3f rpm=%.0f MAP=%.1fkPa AFR=%s%.3f target=%.3f "
                    "fuel=%s dfco=%s misfire=%.1f%% events=%llu\n",
                    elapsed, state.rpm, state.manifoldPressureKpa,
                    state.airFuelRatioValid ? "" : "invalid/",
                    state.airFuelRatio,
                    state.targetAirFuelRatio,
                    state.ecuFuelEnabled ? "on" : "OFF",
                    state.ecuDecelerationFuelCutActive ? "on" : "off",
                    state.misfireRate * 100.0,
                    static_cast<unsigned long long>(state.misfireEventCount
                        - observableMisfireEventsAtStart));
                if (elapsed <= 0.25 || elapsed >= 1.89) {
                    for (std::size_t cylinder = 0;
                         cylinder < state.cylinderStateCount; ++cylinder) {
                        const auto& cylinderState =
                            state.cylinderStates[cylinder];
                        std::printf(
                            "      c%zu phase=%6.1f air=%6.1fmg request=%6.2fmg "
                            "metered=%6.2fmg delivered=%6.2fmg "
                            "portV=%5.2fmg local=%5.2f/%5.2fmg film=%5.2fmg AFR=%s%6.2f "
                            "phi=%5.2f trim=%5.3f duty=%5.1f%%\n",
                            cylinder,
                            cylinderState.cyclePhaseDegrees,
                            cylinderState.trappedFreshAirMassMg,
                            cylinderState.requestedFuelMgPerCycle,
                            cylinderState.meteredFuelMgPerCycle,
                            cylinderState.deliveredFuelMgPerCycle,
                            cylinderState.portFuelVapourInventoryMg,
                            cylinderState.portInjectorFootprintFuelMg,
                            cylinderState.portInjectorFootprintTargetFuelMg,
                            cylinderState.portLiquidFilmFuelMg,
                            cylinderState.airFuelRatioValid ? "" : "invalid/",
                            cylinderState.airFuelRatio,
                            cylinderState.equivalenceRatioAtSpark,
                            cylinderState.closedLoopFuelTrim,
                            cylinderState.injectorDutyCycle * 100.0);
                    }
                }
            }
        }
        metrics.postDfcoLastMisfireAfr = simulator.state().lastMisfireAirFuelRatio;
        metrics.postDfcoLastMisfireTimeSeconds =
            simulator.state().lastMisfireTimeSeconds;
        metrics.postDfcoAfrRecovered = validStart >= 0.0;
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
               "exhaust_backpressure_kpa,afr,afr_valid,target_afr,lambda,"
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
                << ",c" << cylinder << "_injector_duty"
                << ",c" << cylinder << "_fuel_delivery_ratio"
                << ",c" << cylinder << "_afr"
                << ",c" << cylinder << "_afr_valid"
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
    auto scanMisfireEventsAtStart = std::uint64_t { 0 };
    auto lastObservedMisfireEvents = simulator.state().misfireEventCount;
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
        if (traceEvents
                && state.misfireEventCount != lastObservedMisfireEvents) {
            std::printf(
                "    NEW physical misfire t=%.3f rpm=%.0f MAP=%.1fkPa "
                "engineAFR=%s%.3f target=%.3f events=%llu (+%llu)\n",
                timeSeconds, state.rpm, state.manifoldPressureKpa,
                state.airFuelRatioValid ? "" : "invalid/",
                state.airFuelRatio, state.targetAirFuelRatio,
                static_cast<unsigned long long>(state.misfireEventCount),
                static_cast<unsigned long long>(state.misfireEventCount
                    - lastObservedMisfireEvents));
            for (std::size_t cylinder = 0;
                 cylinder < state.cylinderStateCount; ++cylinder) {
                const auto& cylinderState = state.cylinderStates[cylinder];
                if (!cylinderState.misfiring) continue;
                std::printf(
                    "      c%zu phase=%6.1f trappedAir=%6.1fmg request=%6.2fmg "
                    "metered=%6.2fmg delivered=%6.2fmg "
                    "portV=%5.2fmg local=%5.2f/%5.2fmg film=%5.2fmg "
                    "AFR=%s%6.2f lastPhi=%5.2f trim=%5.3f "
                    "duty=%5.1f%% capacity=%5.1f%%\n",
                    cylinder, cylinderState.cyclePhaseDegrees,
                    cylinderState.trappedFreshAirMassMg,
                    cylinderState.requestedFuelMgPerCycle,
                    cylinderState.meteredFuelMgPerCycle,
                    cylinderState.deliveredFuelMgPerCycle,
                    cylinderState.portFuelVapourInventoryMg,
                    cylinderState.portInjectorFootprintFuelMg,
                    cylinderState.portInjectorFootprintTargetFuelMg,
                    cylinderState.portLiquidFilmFuelMg,
                    cylinderState.airFuelRatioValid ? "" : "invalid/",
                    cylinderState.airFuelRatio,
                    cylinderState.equivalenceRatioAtSpark,
                    cylinderState.closedLoopFuelTrim,
                    cylinderState.injectorDutyCycle * 100.0,
                    cylinderState.injectorCapacityRatio * 100.0);
            }
            lastObservedMisfireEvents = state.misfireEventCount;
        }
        const auto deliveryRatio = state.injectedFuelMgPerCycle > 1.0e-9
            ? state.deliveredFuelMgPerCycle / state.injectedFuelMgPerCycle
            : 1.0;
        const auto afrError = state.airFuelRatioValid
            ? std::abs(state.airFuelRatio - state.targetAirFuelRatio)
                / std::max(1.0, state.targetAirFuelRatio)
            : 0.0;
        const auto lockedInRequestedGear =
            result.drive.engagedGear == gearIndex
            && !result.drive.shiftInProgress
            && std::abs(result.drive.clutchSlipRpm) <= lockBandRpm;
        metrics.gearEngaged = metrics.gearEngaged || lockedInRequestedGear;
        metrics.maximumRpm = std::max(metrics.maximumRpm, state.rpm);

        const auto inScan = result.drive.engagedGear == gearIndex
            && !result.drive.shiftInProgress
            && state.rpm >= scanStartRpm;
        const auto preLimiterScan = inScan
            && state.rpm < config.ignition.revLimitRpm - 300.0
            && !state.ecuSoftRevLimiterActive
            && !state.ecuHardRevLimiterActive;
        if (inScan) {
            if (!metrics.scanWindowReached) {
                metrics.minimumScanRpm = state.rpm;
                scanMisfireEventsAtStart = state.misfireEventCount;
            }
            metrics.scanWindowReached = true;
            if (preLimiterScan) {
                if (state.airFuelRatioValid) {
                    ++metrics.validAfrFrames;
                    metrics.minimumObservedAfr = std::min(
                        metrics.minimumObservedAfr, state.airFuelRatio);
                    metrics.maximumAfrError = std::max(
                        metrics.maximumAfrError, afrError);
                    metrics.afrErrorSum += afrError;
                    const auto highRpmThreshold = std::max(
                        config.forcedInduction.fullBoostRpm + 500.0,
                        config.ignition.revLimitRpm - 1'800.0);
                    if (state.rpm >= highRpmThreshold) {
                        metrics.maximumHighRpmAfrError = std::max(
                            metrics.maximumHighRpmAfrError, afrError);
                    }
                } else {
                    ++metrics.invalidAfrFrames;
                }
                metrics.minimumFuelDeliveryRatio = std::min(
                    metrics.minimumFuelDeliveryRatio, deliveryRatio);
                for (std::size_t cylinder = 0;
                     cylinder < state.cylinderStateCount; ++cylinder) {
                    metrics.minimumInjectorCapacityRatio = std::min(
                        metrics.minimumInjectorCapacityRatio,
                        state.cylinderStates[cylinder]
                            .injectorCapacityRatio);
                }
                metrics.maximumMisfireRate = std::max(
                    metrics.maximumMisfireRate, state.misfireRate);
                metrics.misfireRateSum += state.misfireRate;
                metrics.preLimiterMisfireEvents = state.misfireEventCount
                    - scanMisfireEventsAtStart;
            }
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
                << state.airFuelRatio << ',' << state.airFuelRatioValid << ','
                << state.targetAirFuelRatio << ','
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
                        << ',' << cylinderState.injectorDutyCycle
                        << ',' << cylinderState.fuelDeliveryRatio
                        << ',' << cylinderState.airFuelRatio
                        << ',' << cylinderState.airFuelRatioValid
                        << ',' << cylinderState.misfiring
                        << ',' << cylinderState.commandedSparkEventsLastCycle
                        << ',' << cylinderState.completedIgnitionEventsLastCycle
                        << ',' << cylinderState.commandedSparkPhaseLastCycle
                        << ',' << cylinderState.completedIgnitionPhaseLastCycle;
                } else {
                    csv << ",,,,,,,,,,,,,,";
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
            point.afrValid = state.airFuelRatioValid;
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
        std::printf("    pre-limiter physical misfire events=%llu\n",
            static_cast<unsigned long long>(
                metrics.preLimiterMisfireEvents));
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
    bool dfcoTipIn = false;
    bool standingStart = false;
    double launchAtSeconds = 4.0;
    std::vector<std::string> skipped;
    std::vector<std::string> knownStalls;
    std::vector<std::string> knownResumeMisfires;
    double resumeAfrEnvelope = 0.18;
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
        else if (argument == "--dfco-tip-in") dfcoTipIn = true;
        else if (argument == "--standing-start") standingStart = true;
        else if (argument == "--known-stall" && index + 1 < argc)
            knownStalls.emplace_back(argv[++index]);
        else if (argument == "--known-resume-misfire" && index + 1 < argc)
            knownResumeMisfires.emplace_back(argv[++index]);
        else if (argument == "--resume-afr-envelope" && index + 1 < argc)
            resumeAfrEnvelope = std::stod(argv[++index]);
        else if (argument == "--launch-at" && index + 1 < argc)
            launchAtSeconds = std::max(3.5, std::stod(argv[++index]));
        else if (argument == "--skip" && index + 1 < argc)
            skipped.emplace_back(argv[++index]);
        else {
            std::cerr
                << "usage: EngineLabLoadedAccelerationHarness"
                   " [--filter NAME] [--gear 2|3] [--csv-dir DIR]"
                   " [--catalog-root DIR] [--trace] [--dfco-tip-in]"
                   " [--standing-start] [--launch-at S] [--skip NAME]... [--known-stall NAME]...\n"
                   "  [--resume-afr-envelope X] [--known-resume-misfire NAME]...\n";
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

    std::vector<const enginelab::EngineCatalogEntry*> selectedEntries;
    if (filter.empty()) {
        selectedEntries.reserve(catalog.entries.size());
        for (const auto& entry : catalog.entries)
            selectedEntries.push_back(&entry);
    } else {
        const auto selected = enginelab::selectSingleEngineCatalogEntry(
            catalog.entries, filter);
        if (!selected) {
            std::cerr << (selected.status
                    == enginelab::EngineCatalogSelectionStatus::ambiguous
                    ? "FAILED: ambiguous engine selector; use an exact catalogue key:\n"
                    : "FAILED: no catalogue engine matched selector\n");
            for (const auto* match : selected.matches)
                std::cerr << "  " << match->config.audioVoicingKey
                          << "  " << match->config.name << '\n';
            return EXIT_FAILURE;
        }
        selectedEntries.push_back(selected.entry);
    }
    std::erase_if(selectedEntries, [&skipped](const auto* entry) {
        return std::any_of(skipped.begin(), skipped.end(),
            [entry](const std::string& name) {
                return entry->config.name.find(name) != std::string::npos;
            });
    });

    if (standingStart) {
        // Stalled: the crank fell below 220 rpm, where EngineSimulator stops
        // offering spark combustion and nothing can recover it, or the engine
        // has not regained three quarters of idle 1.5 s after the clutch is
        // home. A healthy launch dips and then pulls.
        constexpr double sparkCombustionFloorRpm = 220.0;
        std::printf("standing start, gear %d, WOT, clutch over 0.9 s\n",
            gearNumber);
        std::printf("  %-32s %6s %7s %7s %7s %7s %7s %s\n", "engine", "idle",
            "launch", "minRpm", "final", "minAFR", "misfire", "verdict");
        auto allLaunched = true;
        for (const auto* entry : selectedEntries) {
            if (gearNumber
                > static_cast<int>(entry->config.transmission.gearRatios.size()))
                continue;
            const auto metrics = measureStandingStart(
                entry->config, gearNumber, traceEvents, launchAtSeconds);
            const auto launched = metrics.started
                && metrics.minimumRpm >= sparkCombustionFloorRpm
                && metrics.finalRpm >= 0.75 * metrics.idleRpm;
            // A declared, separately tracked stall still prints STALLED but
            // does not fail the run; any other engine stalling does.
            const auto known = std::any_of(knownStalls.begin(),
                knownStalls.end(), [&](const std::string& token) {
                    return metrics.name.find(token) != std::string::npos;
                });
            allLaunched = allLaunched && (launched || known);
            std::printf("  %-32s %6.0f %7.0f %7.0f %7.0f %7.2f %7llu %s\n",
                metrics.name.c_str(), metrics.idleRpm, metrics.launchRpm,
                std::isfinite(metrics.minimumRpm) ? metrics.minimumRpm : 0.0,
                metrics.finalRpm,
                std::isfinite(metrics.minimumAfr) ? metrics.minimumAfr : 0.0,
                static_cast<unsigned long long>(metrics.misfireEvents),
                !metrics.started ? "NOT STARTED"
                    : launched ? (known ? "ok (declared stall fixed?)" : "ok")
                    : known ? "STALLED (known)" : "STALLED");
        }
        if (!allLaunched) {
            std::cerr << "FAILED: at least one engine stalled on a standing start\n";
            return EXIT_FAILURE;
        }
        return EXIT_SUCCESS;
    }

    std::printf(
        "fixed-gear WOT acceleration, gear %d (automatic shifts OFF, grip limiter OFF)\n",
        gearNumber);
    std::printf(
        "  %-32s start gear scan limiter maxRpm surges firstRpm minTq%% afrPeak%% afrHigh%% fuel%% injHead%% misfire%% invalidAFR fuelCut sparkCut clutchSlip solver\n",
        "engine");
    auto allRunnable = true;
    for (const auto* entry : selectedEntries) {
        if (gearNumber
            > static_cast<int>(entry->config.transmission.gearRatios.size())) {
            std::printf("  %-32s   n/a (gear unavailable)\n",
                entry->config.name.c_str());
            continue;
        }
        try {
            const auto metrics = measureAcceleration(
                entry->config, gearNumber, csvDirectory, traceEvents,
                dfcoTipIn);
            std::printf(
                "  %-32s %5s %4s %4s %7s %6.0f %6zu %8.0f %6.1f %8.1f %8.1f %5.1f %8.1f %8.1f %10zu %7zu %8zu %10zu %6zu\n",
                metrics.name.c_str(), metrics.started ? "yes" : "NO",
                metrics.gearEngaged ? "yes" : "NO",
                metrics.scanWindowReached ? "yes" : "NO",
                metrics.reachedLimiter ? "yes" : "no",
                metrics.maximumRpm, metrics.surges.size(),
                metrics.firstSurgeRpm,
                metrics.minimumTorqueRatio * 100.0,
                metrics.maximumAfrError * 100.0,
                metrics.maximumHighRpmAfrError * 100.0,
                metrics.minimumFuelDeliveryRatio * 100.0,
                metrics.minimumInjectorCapacityRatio * 100.0,
                metrics.maximumMisfireRate * 100.0,
                metrics.invalidAfrFrames,
                metrics.preLimiterFuelCutFrames,
                metrics.preLimiterSparkCutFrames,
                metrics.preLimiterClutchSlipFrames,
                metrics.solverLimitedFrames);
            if (dfcoTipIn) {
                std::printf(
                    "    DFCO->WOT: warmIdle=%.0f preLift=%.0f postLift=%.0f "
                    "cut=%s valid=%s tValid=%.3fs afrPeak=%.1f%% "
                    "misfirePeak=%.1f%% events=%llu/%llu lastMisfireAFR=%.2f at=%.3fs\n",
                    metrics.preDfcoWarmIdleRpm,
                    metrics.preDfcoLiftRpm,
                    metrics.postDfcoLiftRpm,
                    metrics.dfcoObserved ? "yes" : "NO",
                    metrics.postDfcoAfrRecovered ? "yes" : "NO",
                    metrics.postDfcoTimeToValidSeconds,
                    metrics.postDfcoMaximumAfrError * 100.0,
                    metrics.postDfcoMaximumMisfireRate * 100.0,
                    static_cast<unsigned long long>(metrics.postDfcoMisfireEvents),
                    static_cast<unsigned long long>(
                        metrics.postDfcoCommandedSparkEvents),
                    metrics.postDfcoLastMisfireAfr,
                    metrics.postDfcoLastMisfireTimeSeconds);
            }
            if (metrics.dfcoTipInRequested) {
                // This mode owns a running overrun/resume scenario. It can
                // reach the limiter before the later generic scan window (the
                // 2JZ does), so requiring that unrelated window makes a valid
                // DFCO result fail with zero-filled steady-pull metrics.
                const auto postDfcoAfrWithinEnvelope =
                    entry->config.fuel == enginelab::FuelType::diesel
                    // Compression ignition controls torque with fuel quantity;
                    // its healthy resume is intentionally lean and does not
                    // track the gasoline enrichment map. Guard its rich smoke
                    // side independently instead of applying a symmetric SI
                    // target-error band that it can never meaningfully pass.
                    ? metrics.postDfcoMinimumAfr
                        >= entry->config.fuelProperties
                            .stoichiometricAirFuelRatio * 1.05
                    : metrics.postDfcoMaximumAfrError <= resumeAfrEnvelope;
                // A declared engine may misfire on the resume; it is still
                // printed, and tracked in docs/journal.md.
                const auto resumeMisfireKnown = std::any_of(
                    knownResumeMisfires.begin(), knownResumeMisfires.end(),
                    [&](const std::string& token) {
                        return entry->config.name.find(token) != std::string::npos;
                    });
                allRunnable = allRunnable && metrics.started
                    && metrics.gearEngaged
                    && metrics.dfcoObserved
                    && metrics.postDfcoAfrRecovered
                    && metrics.postDfcoTimeToValidSeconds <= 0.25
                        // The WOT target is deliberately richer than
                        // stoichiometric. An 18 % target-relative envelope is
                        // still only about lambda 1.04 for the catalogue's
                        // 12.9:1 resume target; zero physical misfire events is
                        // required independently below. This rejects the old
                        // AFR 20-30 lean excursion without pretending a brief
                        // AFR 15.1 transport transient is a flammability loss.
                    && postDfcoAfrWithinEnvelope
                    && (metrics.postDfcoMisfireEvents == 0 || resumeMisfireKnown);
            } else {
                const auto afrTrackingValid =
                    entry->config.fuel == enginelab::FuelType::diesel
                    ? metrics.minimumObservedAfr
                        >= entry->config.fuelProperties.stoichiometricAirFuelRatio
                            * 1.10
                    : metrics.maximumHighRpmAfrError <= 0.12
                        && metrics.afrErrorSum
                            / static_cast<double>(metrics.validAfrFrames) <= 0.08;
                allRunnable = allRunnable && metrics.started
                    && metrics.gearEngaged && metrics.scanWindowReached
                    && metrics.surges.empty()
                    && metrics.validAfrFrames > 0
                    && metrics.invalidAfrFrames == 0
                    && afrTrackingValid
                    && metrics.minimumInjectorCapacityRatio >= 0.10
                    && metrics.preLimiterMisfireEvents == 0
                    && metrics.maximumMisfireRate <= 0.10
                    && metrics.misfireRateSum
                        / static_cast<double>(metrics.validAfrFrames) <= 0.02;
            }
        } catch (const std::exception& exception) {
            std::cerr << "FAILED: " << entry->config.name << ": "
                      << exception.what() << '\n';
            allRunnable = false;
        }
    }
    return allRunnable ? EXIT_SUCCESS : EXIT_FAILURE;
}
