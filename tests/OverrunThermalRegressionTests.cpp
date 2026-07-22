#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string_view>

namespace {
constexpr double stepSeconds = 1.0 / 240.0;

void require(bool condition, const char* message) {
    if (condition) return;
    std::cerr << "FAILED: " << message << '\n';
    std::exit(EXIT_FAILURE);
}

enginelab::EngineConfig loadAircooledFlatSix() {
    const auto catalog = enginelab::loadEngineCatalog(
        std::filesystem::path(ENGINELAB_CATALOG_ROOT));
    if (!catalog.errors.empty())
        throw std::runtime_error(catalog.errors.front());
    constexpr std::string_view engineName = "Aircooled-like 3.6 Flat-6";
    const auto engine = std::find_if(catalog.entries.begin(), catalog.entries.end(),
        [](const enginelab::EngineCatalogEntry& entry) {
            return entry.config.name == engineName;
        });
    if (engine == catalog.entries.end())
        throw std::runtime_error("air-cooled flat-six regression fixture is missing");
    return engine->config;
}

void settleAtWideOpenThrottle(enginelab::EngineSimulator& simulator,
                              double targetRpm) {
    for (int step = 0; step < static_cast<int>(2.0 / stepSeconds); ++step) {
        const auto time = static_cast<double>(step) * stepSeconds;
        (void)simulator.step(stepSeconds, { true, time < 1.5, 0.72, 0.0 });
    }

    auto dynoIntegral = 0.0;
    for (int step = 0; step < static_cast<int>(4.25 / stepSeconds); ++step) {
        const auto normalisedSpeedError =
            (simulator.state().rpm - targetRpm) / std::max(1.0, targetRpm);
        dynoIntegral = std::clamp(
            dynoIntegral + normalisedSpeedError * stepSeconds * 1.20, 0.0, 0.95);
        enginelab::EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = simulator.state().rpm < 550.0;
        controls.throttle = 1.0;
        controls.load = std::clamp(
            dynoIntegral + normalisedSpeedError * 0.70, 0.0, 1.0);
        (void)simulator.step(stepSeconds, controls);
    }
}
}

int main() {
    try {
        auto config = loadAircooledFlatSix();
        enginelab::normaliseEngineConfig(config);
        enginelab::SimpleEcuModel ecu;
        enginelab::SimplifiedGasolinePhysics physics;
        enginelab::FourStrokeEventGenerator events;
        auto exhaust = enginelab::ExhaustGraph::makeForEngine(config);
        enginelab::EngineSimulator simulator(config, ecu, physics, events, exhaust);

        const auto hotRpm = std::min(config.redlineRpm,
            config.ignition.revLimitRpm) * 0.90;
        settleAtWideOpenThrottle(simulator, hotRpm);
        const auto liftOffTemperatureC = simulator.state().exhaustTemperatureC;

        constexpr double overrunDurationSeconds = 8.0;
        const auto finalRpm = std::max(config.idleRpm * 2.0, hotRpm * 0.45);
        auto motoringTorqueNm = 0.0;
        auto peakExhaustTemperatureC = liftOffTemperatureC;
        auto observedFuelCut = false;
        auto observedMotoredOverrun = false;
        for (int step = 0;
             step < static_cast<int>(overrunDurationSeconds / stepSeconds); ++step) {
            const auto time = static_cast<double>(step) * stepSeconds;
            const auto phase = time / overrunDurationSeconds;
            const auto targetRpm = std::lerp(hotRpm, finalRpm, phase);
            const auto speedErrorRpm = targetRpm - simulator.state().rpm;
            const auto requiredTorqueNm = std::max(0.0,
                -simulator.state().cycleAveragedTorqueNm + speedErrorRpm * 0.20);
            motoringTorqueNm += (requiredTorqueNm - motoringTorqueNm)
                * (1.0 - std::exp(-stepSeconds * 18.0));

            enginelab::EngineControls controls;
            controls.ignitionEnabled = true;
            controls.throttle = 0.0;
            controls.externalTorqueNm = motoringTorqueNm;
            const auto frame = simulator.step(stepSeconds, controls);
            peakExhaustTemperatureC = std::max(
                peakExhaustTemperatureC, frame.state.exhaustTemperatureC);
            if (time > 0.5) {
                observedFuelCut = observedFuelCut
                    || frame.state.fuelFlowGramsPerSecond < 1.0e-9;
                observedMotoredOverrun = observedMotoredOverrun
                    || (motoringTorqueNm > 1.0 && frame.state.rpm > config.idleRpm * 1.5);
            }
        }

        if (peakExhaustTemperatureC >= 1'200.0)
            std::cerr << "overrun thermal diagnostic: lift_off_c="
                      << liftOffTemperatureC << " peak_c="
                      << peakExhaustTemperatureC << " final_rpm="
                      << simulator.state().rpm << '\n';
        require(observedFuelCut,
            "closed-throttle high-rpm overrun must enter deceleration fuel cut");
        require(observedMotoredOverrun,
            "the regression fixture must keep the unfuelled engine driven through the crankshaft");
        require(std::isfinite(peakExhaustTemperatureC),
            "overrun exhaust temperature must remain finite");
        require(peakExhaustTemperatureC < 1'200.0,
            "an unfuelled motored engine must not create combustion-range exhaust temperature");
    } catch (const std::exception& error) {
        std::cerr << "FAILED: " << error.what() << '\n';
        return EXIT_FAILURE;
    }

    std::cout << "Overrun thermal regression tests passed\n";
    return EXIT_SUCCESS;
}
