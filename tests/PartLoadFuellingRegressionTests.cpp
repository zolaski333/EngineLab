/**
 * The ECU must actually deliver the mixture it commands at PART throttle.
 *
 * This gate exists because nothing else covered the throttle band between the
 * two learned fuel-adaptation cells. The command is a blend of a low-load and a
 * high-load cell, ramped by the physical throttle opening from 0.10 to 0.25;
 * learning, however, was permitted only at the two ends. The high-load cell
 * therefore held up to 100 % of the authority over a region in which neither
 * cell could correct anything -- and worse, the ramp reaches full authority at
 * the same 0.25 where its learning was gated on, while the opening that drives
 * it is a smoothed physical value that approaches such a test from below and
 * can sit there indefinitely.
 *
 * The observable consequence was not subtle: free-revving a Flat-6 at a fixed
 * quarter throttle, the adaptation sat pinned at 0.586 against a clamp floor of
 * 0.55 -- removing 41 % of the fuel from an engine already starving -- for a
 * measured AFR of 20.9 against a commanded 13.4, 81 % misfire, and net torque
 * swinging between -190 and +250 Nm. That torque spike is audible, and it was
 * reported from the application as an intermittent "bop" at "exactly 25 %
 * throttle", which is this constant and not a coincidence. The catalogue's only
 * compression-ignition engine was the only one unaffected, because it bypasses
 * this adaptation entirely.
 *
 * Both tolerances are engineering practice, NOT this simulator's output:
 *  * a production closed-loop fuel system holds lambda within a few per cent of
 *    its target at a settled operating point, so 5 % is already generous;
 *  * a healthy engine held at a steady part throttle does not misfire at all,
 *    so 5 % of firing opportunities is likewise generous.
 * Do not widen either to accommodate a future change.
 */
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/foundation/EngineTypes.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/simulation/DynoAbsorberController.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>

namespace {

void require(bool condition, const std::string& message) {
    if (condition) return;
    std::cerr << "FAILED: " << message << '\n';
    std::exit(EXIT_FAILURE);
}

[[nodiscard]] bool containsCaseInsensitive(std::string_view haystack,
                                           std::string_view needle) {
    const auto lower = [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    };
    return std::search(haystack.begin(), haystack.end(), needle.begin(), needle.end(),
        [&lower](char a, char b) { return lower(static_cast<unsigned char>(a))
            == lower(static_cast<unsigned char>(b)); }) != haystack.end();
}

struct BandResult final {
    std::size_t samplesInBand { 0 };
    double meanLambdaError { 0.0 };   // |lambda - target| / target
    double meanMisfireRate { 0.0 };
    double meanThrottle { 0.0 };
};

/** Hold the engine at a moderate speed while COMMANDING a part throttle.
 *
 * A dyno hold is used rather than a free rev because a free rev at any
 * appreciable opening ends on the rev limiter, where combustion telemetry is an
 * artefact. The absorber answers a small throttle request with a small brake
 * torque, but the commanded opening -- which is what selects the adaptation
 * cells -- stays where it was put, and that is the quantity under test.
 */
[[nodiscard]] BandResult measureBand(const enginelab::EngineConfig& baseConfig,
                                     double throttleCommand, double holdRpm) {
    auto config = baseConfig;
    enginelab::normaliseEngineConfig(config);
    enginelab::SimpleEcuModel ecu;
    enginelab::SimplifiedGasolinePhysics physics;
    enginelab::FourStrokeEventGenerator events;
    auto exhaust = enginelab::ExhaustGraph::makeForEngine(config);
    enginelab::EngineSimulator simulator(config, ecu, physics, events, exhaust);
    constexpr double dt = 1.0 / 240.0;
    enginelab::DynoAbsorberController absorber(config);
    absorber.reset(simulator.state().rpm, simulator.state().torqueNm);

    const auto step = [&](double throttle) {
        enginelab::EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = simulator.state().rpm < 550.0;
        controls.throttle = throttle;
        controls.dynamometerTorqueNm =
            absorber.advance(dt, holdRpm, simulator.state()).brakeTorqueNm;
        return simulator.step(dt, controls);
    };

    // Settle: reach the hold speed, then give the adaptation ample time at the
    // operating point. The defect this gate covers is a fixed point, not a
    // transient -- it does not heal with time -- so a generous settle cannot
    // hide it, while a short one would only measure the approach.
    for (int i = 0; i < static_cast<int>(10.0 / dt); ++i) (void)step(throttleCommand);

    BandResult result;
    auto lambdaErrorSum = 0.0;
    auto misfireSum = 0.0;
    auto throttleSum = 0.0;
    for (int i = 0; i < static_cast<int>(4.0 / dt); ++i) {
        const auto frame = step(throttleCommand);
        const auto& state = frame.state;
        // Only samples genuinely inside the interpolation band count. An
        // assertion on a quantity the engine never reached proves nothing --
        // the non-vacuity guard below turns an empty band into a failure.
        if (state.throttle <= 0.10 || state.throttle >= 0.25) continue;
        if (!(state.targetAirFuelRatio > 1.0) || !(state.airFuelRatio > 1.0)) continue;
        lambdaErrorSum += std::abs(state.airFuelRatio - state.targetAirFuelRatio)
            / state.targetAirFuelRatio;
        misfireSum += state.misfireRate;
        throttleSum += state.throttle;
        ++result.samplesInBand;
    }
    if (result.samplesInBand > 0) {
        const auto divisor = static_cast<double>(result.samplesInBand);
        result.meanLambdaError = lambdaErrorSum / divisor;
        result.meanMisfireRate = misfireSum / divisor;
        result.meanThrottle = throttleSum / divisor;
    }
    return result;
}

} // namespace

int main() {
    const auto catalog = enginelab::loadEngineCatalog(
        std::filesystem::path(ENGINELAB_CATALOG_ROOT));
    require(catalog.errors.empty(),
        "catalogue load: " + (catalog.errors.empty() ? std::string {} : catalog.errors.front()));

    // The engine the defect was reported on, plus one unrelated architecture so
    // a single engine's calibration cannot carry the gate.
    const char* const engineFilters[] = { "Flat-6", "LS3" };
    const double throttleCommands[] = { 0.14, 0.18, 0.22 };
    constexpr double holdRpm = 3'000.0;
    constexpr double maximumLambdaError = 0.05;
    constexpr double maximumMisfireRate = 0.05;
    constexpr std::size_t minimumSamplesInBand = 240;  // one second at 240 Hz

    auto matched = 0;
    for (const auto* filter : engineFilters) {
        const auto entry = std::find_if(catalog.entries.begin(), catalog.entries.end(),
            [filter](const auto& candidate) {
                return containsCaseInsensitive(candidate.config.name, filter);
            });
        require(entry != catalog.entries.end(),
            std::string { "no catalogue engine matched " } + filter);
        ++matched;
        for (const auto throttle : throttleCommands) {
            const auto measured = measureBand(entry->config, throttle, holdRpm);
            const auto label = entry->config.name + " @ throttle "
                + std::to_string(throttle);
            require(measured.samplesInBand >= minimumSamplesInBand,
                label + ": the engine never held the 0.10-0.25 adaptation band, so "
                        "this measurement proves nothing (samples="
                        + std::to_string(measured.samplesInBand) + ")");
            std::cout << "  " << label
                      << "  throttle=" << measured.meanThrottle
                      << "  lambdaError=" << measured.meanLambdaError
                      << "  misfire=" << measured.meanMisfireRate << '\n';
            require(measured.meanLambdaError <= maximumLambdaError,
                label + ": commanded mixture not delivered at part load "
                        "(mean |lambda-target|/target = "
                        + std::to_string(measured.meanLambdaError) + ")");
            require(measured.meanMisfireRate <= maximumMisfireRate,
                label + ": engine misfires while held at a steady part throttle "
                        "(mean rate = " + std::to_string(measured.meanMisfireRate) + ")");
        }
    }
    require(matched == 2, "expected both reference engines to be measured");
    std::cout << "Part-load fuelling regression: PASS\n";
    return EXIT_SUCCESS;
}
