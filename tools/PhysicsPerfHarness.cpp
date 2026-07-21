#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/foundation/EngineTypes.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace {
struct StepMetrics final {
    double actualRpm {};
    double meanMicroseconds {};
    double p50Microseconds {};
    double p95Microseconds {};
    double maximumMicroseconds {};
    double meanSubsteps {};
    double meanExhaustTemperatureC {};
    double meanImepBar {};
    double peakCylinderPressureBar {};
    double meanLambda {};
    double meanTorqueNm {};
    double meanPowerKw {};
    double meanVolumetricEfficiency {};
    double meanManifoldKpa {};
    double meanExhaustKpa {};
    double meanIntakeRunnerKpa {};
    double meanIntakeMgPerCycle {};
    double meanTrappedMg {};
    double meanResidualFraction {};
    double meanAirMgPerCycle {};
};

bool containsCaseInsensitive(const std::string& text, const std::string& filter) {
    const auto lower = [](unsigned char value) { return static_cast<char>(std::tolower(value)); };
    std::string loweredText(text.size(), '\0');
    std::string loweredFilter(filter.size(), '\0');
    std::transform(text.begin(), text.end(), loweredText.begin(), lower);
    std::transform(filter.begin(), filter.end(), loweredFilter.begin(), lower);
    return loweredText.find(loweredFilter) != std::string::npos;
}

StepMetrics measurePoint(enginelab::EngineSimulator& simulator, double targetRpm) {
    constexpr double dt = 1.0 / 240.0;
    constexpr int settleSteps = static_cast<int>(3.0 / dt);
    constexpr int sampleSteps = static_cast<int>(1.25 / dt);
    auto dynoIntegral = 0.0;
    std::vector<double> timings;
    timings.reserve(sampleSteps);
    auto rpmSum = 0.0;
    auto substepSum = 0.0;
    auto exhaustSum = 0.0;
    auto imepSum = 0.0;
    auto lambdaSum = 0.0;
    auto peakCylinderBar = 0.0;
    auto torqueSum = 0.0;
    auto powerSum = 0.0;
    auto veSum = 0.0;
    auto mapSum = 0.0, exhSum = 0.0, runnerSum = 0.0;
    auto intakeMgSum = 0.0, trappedSum = 0.0, residualSum = 0.0, airMgSum = 0.0;
    auto cylSamples = 0.0;
    for (int step = 0; step < settleSteps + sampleSteps; ++step) {
        const auto speedError = (simulator.state().rpm - targetRpm) / std::max(1.0, targetRpm);
        dynoIntegral = std::clamp(dynoIntegral + speedError * dt * 1.20, 0.0, 0.95);
        enginelab::EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = simulator.state().rpm < 550.0;
        controls.throttle = 1.0;
        controls.load = std::clamp(dynoIntegral + speedError * 0.70, 0.0, 1.0);
        const auto begin = std::chrono::steady_clock::now();
        const auto frame = simulator.step(dt, controls);
        const auto end = std::chrono::steady_clock::now();
        if (step >= settleSteps) {
            timings.push_back(std::chrono::duration<double, std::micro>(end - begin).count());
            rpmSum += frame.state.rpm;
            substepSum += static_cast<double>(frame.state.solverSubsteps);
            exhaustSum += frame.state.exhaustTemperatureC;
            imepSum += frame.state.indicatedMeanEffectivePressureBar;
            lambdaSum += frame.state.lambda;
            torqueSum += frame.state.cycleAveragedTorqueNm;
            powerSum += frame.state.cycleAveragedPowerKw;
            veSum += frame.state.volumetricEfficiency;
            mapSum += frame.state.manifoldPressureKpa;
            exhSum += frame.state.exhaustPressureKpa;
            runnerSum += frame.state.intakeRunnerPressureKpa;
            airMgSum += frame.state.airMassMgPerCycle;
            for (std::size_t c = 0; c < frame.state.cylinderStateCount; ++c) {
                intakeMgSum += frame.state.cylinderStates[c].intakeFlowMgPerCycle;
                trappedSum += frame.state.cylinderStates[c].trappedMassMg;
                residualSum += frame.state.cylinderStates[c].residualGasFraction;
                cylSamples += 1.0;
            }
            for (std::size_t c = 0; c < frame.state.cylinderStateCount; ++c)
                peakCylinderBar = std::max(peakCylinderBar,
                    frame.state.cylinderStates[c].pressureEstimateBar);
        }
    }
    std::sort(timings.begin(), timings.end());
    auto mean = 0.0;
    for (const auto timing : timings) mean += timing;
    mean /= static_cast<double>(timings.size());
    const auto percentile = [&timings](double fraction) {
        const auto index = static_cast<std::size_t>(fraction * static_cast<double>(timings.size() - 1));
        return timings[index];
    };
    return {
        rpmSum / static_cast<double>(timings.size()),
        mean,
        percentile(0.50),
        percentile(0.95),
        timings.back(),
        substepSum / static_cast<double>(timings.size()),
        exhaustSum / static_cast<double>(timings.size()),
        imepSum / static_cast<double>(timings.size()),
        peakCylinderBar,
        lambdaSum / static_cast<double>(timings.size()),
        torqueSum / static_cast<double>(timings.size()),
        powerSum / static_cast<double>(timings.size()),
        veSum / static_cast<double>(timings.size()),
        mapSum / static_cast<double>(timings.size()),
        exhSum / static_cast<double>(timings.size()),
        runnerSum / static_cast<double>(timings.size()),
        intakeMgSum / std::max(1.0, cylSamples),
        trappedSum / std::max(1.0, cylSamples),
        residualSum / std::max(1.0, cylSamples),
        airMgSum / static_cast<double>(timings.size())
    };
}


/**
 * Dump one cylinder's gas-exchange cycle at crank resolution (`--trace rpm`).
 *
 * The swept CSV above reports cycle averages, and a cycle average cannot tell a
 * healthy engine from one that fills with hot gas: it reports what went in, not
 * what state it was in. This mode settles the engine at a speed under the same
 * wide-open dyno as measurePoint(), then continues at 1/48 of the frame step so
 * consecutive rows are ~2 deg of crank apart, and prints the charge state on
 * both sides of the intake valve.
 *
 * Reading it: the columns that matter are `irt_c` (intake runner charge
 * temperature) against `cyl_mass_mg` (trapped mass). Filling failures show up
 * there as a density problem long before they show up as a pressure one -- the
 * runner sitting at 333 degC while the manifold reads a healthy 96 kPa is what
 * exposed the jet-momentum defect in docs/physics-audit.md.
 *
 * Note the fine step is a diagnostic instrument, not the delivered cadence: it
 * changes the solver substep structure, so read trends and ratios from it and
 * take absolute figures from the swept CSV.
 */
void traceEngine(const enginelab::EngineConfig& baseConfig, double targetRpm) {
    auto config = baseConfig;
    enginelab::normaliseEngineConfig(config);
    enginelab::SimpleEcuModel ecu;
    enginelab::SimplifiedGasolinePhysics physics;
    enginelab::FourStrokeEventGenerator events;
    auto exhaust = enginelab::ExhaustGraph::makeForEngine(config);
    enginelab::EngineSimulator simulator(config, ecu, physics, events, exhaust);
    constexpr double dt = 1.0 / 240.0;
    auto dynoIntegral = 0.0;
    const auto drive = [&](double stepDt) {
        const auto speedError = (simulator.state().rpm - targetRpm) / std::max(1.0, targetRpm);
        dynoIntegral = std::clamp(dynoIntegral + speedError * stepDt * 1.20, 0.0, 0.95);
        enginelab::EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = simulator.state().rpm < 550.0;
        controls.throttle = 1.0;
        controls.load = std::clamp(dynoIntegral + speedError * 0.70, 0.0, 1.0);
        return simulator.step(stepDt, controls);
    };
    for (int step = 0; step < static_cast<int>(4.0 / dt); ++step) (void)drive(dt);
    std::cout << "phase,rpm,cyl_p_bar,cyl_t_c,cyl_mass_mg,runner_kpa,lift_mm,"
                 "intake_v_mps,intake_mg,residual,exh_gps,net_hz,limited,irt_c,irp_kpa\n";
    const auto fineDt = dt / 48.0;
    const auto cycleSeconds = 120.0 / std::max(1.0, simulator.state().rpm);
    const auto fineSteps = static_cast<int>(1.2 * cycleSeconds / fineDt);
    for (int step = 0; step < fineSteps; ++step) {
        const auto frame = drive(fineDt);
        const auto& c = frame.state.cylinderStates[0];
        std::cout << c.cyclePhaseDegrees << ',' << frame.state.rpm << ',' << c.pressureEstimateBar
                  << ',' << c.gasTemperatureC << ',' << c.trappedMassMg
                  << ',' << c.runnerPressureKpa << ',' << c.intakeValveLiftMm
                  << ',' << c.intakeVelocityMps << ',' << c.intakeFlowMgPerCycle
                  << ',' << c.residualGasFraction
                  << ',' << frame.state.exhaustFlowGramsPerSecond
                  << ',' << frame.state.exhaustNetworkSubstepFrequencyHz
                  << ',' << (frame.state.solverResolutionLimited ? 1 : 0)
                  << ',' << c.intakeRunnerTemperatureC
                  << ',' << c.intakeRunnerChargePressureKpa << '\n';
    }
}

void measureEngine(const enginelab::EngineConfig& baseConfig, int run) {
    auto config = baseConfig;
    enginelab::normaliseEngineConfig(config);
    enginelab::SimpleEcuModel ecu;
    enginelab::SimplifiedGasolinePhysics physics;
    enginelab::FourStrokeEventGenerator events;
    auto exhaust = enginelab::ExhaustGraph::makeForEngine(config);
    enginelab::EngineSimulator simulator(config, ecu, physics, events, exhaust);
    constexpr double dt = 1.0 / 240.0;
    for (int step = 0; step < static_cast<int>(2.0 / dt); ++step) {
        const auto time = static_cast<double>(step) * dt;
        (void)simulator.step(dt, { true, time < 1.5, 0.72, 0.0 });
    }
    const auto maximumRpm = std::min(config.redlineRpm, config.ignition.revLimitRpm);
    // Near-idle is measured explicitly: it is where the exhaust-coupling
    // low-speed cap binds (firing frequency below 1/(16*cap)), so its cost
    // moves independently of the load points whenever that cap changes.
    const std::array targets {
        config.idleRpm * 1.1,
        std::max(config.idleRpm * 1.5, maximumRpm * 0.55),
        std::max(config.idleRpm * 1.5, maximumRpm * 0.90)
    };
    for (const auto target : targets) {
        const auto result = measurePoint(simulator, target);
        std::cout << std::quoted(config.name) << ',' << config.cylinders.size() << ',' << run
                  << ',' << target << ',' << result.actualRpm << ',' << result.meanMicroseconds
                  << ',' << result.p50Microseconds << ',' << result.p95Microseconds
                  << ',' << result.maximumMicroseconds << ',' << result.meanSubsteps
                  << ',' << result.meanExhaustTemperatureC << ',' << result.meanImepBar
                  << ',' << result.peakCylinderPressureBar << ',' << result.meanLambda
                  << ',' << result.meanTorqueNm << ',' << result.meanPowerKw
                  << ',' << result.meanVolumetricEfficiency
                  << ',' << result.meanManifoldKpa << ',' << result.meanExhaustKpa
                  << ',' << result.meanIntakeRunnerKpa << ',' << result.meanIntakeMgPerCycle
                  << ',' << result.meanTrappedMg << ',' << result.meanResidualFraction
                  << ',' << result.meanAirMgPerCycle << '\n';
    }
}
}

int main(int argc, char** argv) {
    std::filesystem::path catalogRoot = std::filesystem::current_path();
    std::string filter;
    auto runs = 3;
    auto traceRpm = 0.0;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--catalog-root" && index + 1 < argc) catalogRoot = argv[++index];
        else if (argument == "--filter" && index + 1 < argc) filter = argv[++index];
        else if (argument == "--trace" && index + 1 < argc) traceRpm = std::stod(argv[++index]);
        else if (argument == "--runs" && index + 1 < argc) runs = std::max(1, std::stoi(argv[++index]));
        else {
            std::cerr << "usage: EngineLabPhysicsPerfHarness [--catalog-root dir]"
                         " [--filter name-fragment] [--runs count] [--trace rpm]\n";
            return EXIT_FAILURE;
        }
    }

    std::vector<enginelab::EngineConfig> engines {
        enginelab::makeDefaultInlineTwo(), enginelab::makeDefaultV8()
    };
    const auto catalog = enginelab::loadEngineCatalog(catalogRoot);
    for (const auto& entry : catalog.entries)
        if (containsCaseInsensitive(entry.config.name, "LS3")
                || containsCaseInsensitive(entry.config.name, "Merlin"))
            engines.push_back(entry.config);
    if (!filter.empty())
        std::erase_if(engines, [&filter](const auto& config) {
            return !containsCaseInsensitive(config.name, filter);
        });
    if (engines.empty()) {
        std::cerr << "no matching engine\n";
        return EXIT_FAILURE;
    }

    std::cout << std::fixed << std::setprecision(3);
    // --trace is a single-engine instrument: pair it with --filter.
    if (traceRpm > 0.0) {
        traceEngine(engines.front(), traceRpm);
        return EXIT_SUCCESS;
    }
    std::cout << "engine,cylinders,run,target_rpm,actual_rpm,mean_us,p50_us,p95_us,max_us,mean_substeps,"
                 "exhaust_c,imep_bar,peak_cyl_bar,lambda,torque_nm,power_kw,ve,"
                 "map_kpa,exh_kpa,runner_kpa,intake_mg,trapped_mg,residual,air_mg\n";
    for (int run = 1; run <= runs; ++run)
        for (const auto& engine : engines) measureEngine(engine, run);
    return EXIT_SUCCESS;
}
