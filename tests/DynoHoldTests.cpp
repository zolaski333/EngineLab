// The dyno absorber holds a light shaft at a steady point.
//
// The absorber's acceleration feedback is a virtual inertia. Scaled with
// displacement alone it reached 26-56 times the shaft's own on light-shaft
// catalogue engines, and behind the speed filters and a frame's delay it
// swung the brake into a limit cycle: stepped from 2,000 to 3,000 rpm, the
// Hayabusa and the 13B scattered 53 and 45 rpm, and a CP2 the runtime dyno caught at
// 1,800 rpm fell into a stall-recovery loop (journal 2026-10-30). A held point
// must scatter by a few rpm, as every other catalogue engine does.

#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/simulation/DynoAbsorberController.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <string_view>

namespace {
using namespace enginelab;

void require(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

EngineConfig findEngine(const EngineCatalogLoadResult& catalog, std::string_view fragment) {
    const auto found = std::find_if(catalog.entries.begin(), catalog.entries.end(),
        [&](const auto& entry) { return entry.config.name.find(fragment) != std::string::npos; });
    require(found != catalog.entries.end(), std::string(fragment) + " must exist in the catalogue");
    auto config = found->config;
    normaliseEngineConfig(config);
    return config;
}

/** Wide-open throttle held at `rpm` by a fresh absorber, as the dyno-sweep
 *  harness holds each point: the standard deviation of the absorber's
 *  filtered speed over `sampleSeconds` after `settleSeconds`, its
 *  `held_rpm_stddev`. */
double heldRpmStandardDeviation(EngineSimulator& simulator, const EngineConfig& config,
    double rpm, double settleSeconds, double sampleSeconds) {
    constexpr double dt = 1.0 / 240.0;
    DynoAbsorberController absorber(config);
    absorber.reset(simulator.state().rpm, simulator.state().torqueNm);
    const auto settle = static_cast<int>(settleSeconds / dt), sample = static_cast<int>(sampleSeconds / dt);
    auto sum = 0.0, sumSquares = 0.0;
    for (int step = 0; step < settle + sample; ++step) {
        const auto output = absorber.advance(dt, rpm, simulator.state());
        EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = simulator.state().rpm < dynoStarterCutoutRpm(config);
        controls.throttle = 1.0;
        controls.dynamometerTorqueNm = output.brakeTorqueNm;
        (void)simulator.step(dt, controls);
        if (step < settle) continue;
        sum += output.filteredRpm;
        sumSquares += output.filteredRpm * output.filteredRpm;
    }
    const auto mean = sum / sample;
    require(std::abs(mean - rpm) < 50.0,
        config.name + ": the held speed is the target (" + std::to_string(mean) + " rpm)");
    return std::sqrt(std::max(0.0, sumSquares / sample - mean * mean));
}
} // namespace

int main() {
    const auto catalog = loadEngineCatalog(ENGINELAB_CATALOG_ROOT);
    require(catalog.errors.empty() && !catalog.entries.empty(), "the shipped catalogue loads");

    const struct { std::string_view engine; double rpm; } points[] {
        { "Hayabusa", 3'000.0 },
        { "13B", 3'000.0 },
    };
    for (const auto& point : points) {
        // The limit cycle starts on a step between points: held from cold at
        // its target, the Hayabusa scattered 3.9 rpm uncapped. The sweep's
        // own path is a 2,000 rpm hold, then a fresh absorber 1,000 rpm up.
        const auto config = findEngine(catalog, point.engine);
        SimpleEcuModel ecu;
        SimplifiedGasolinePhysics physics;
        FourStrokeEventGenerator events;
        ExhaustGraph exhaust = ExhaustGraph::makeForEngine(config);
        auto simulator = std::make_unique<EngineSimulator>(config, ecu, physics, events, exhaust);
        (void)heldRpmStandardDeviation(*simulator, config, point.rpm - 1'000.0, 5.0, 0.5);
        const auto deviation = heldRpmStandardDeviation(*simulator, config, point.rpm, 2.0, 1.0);
        std::cout << config.name << " held at " << point.rpm << " rpm: deviation "
                  << deviation << " rpm\n";
        require(deviation < 15.0,
            config.name + ": a held point scatters by a few rpm, not a brake limit cycle");
    }
    std::cout << "PASS: the dyno absorber holds light shafts\n";
    return EXIT_SUCCESS;
}
