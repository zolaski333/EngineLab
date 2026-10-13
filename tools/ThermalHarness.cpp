// Does the engine's temperature drift over a dyno session?
//
// Reproduces what a user does in the application: start, idle, then several
// dyno passes in a row (wide-open throttle, the absorber ramping the engine
// from the application's sweep entry to its ceiling at 500 rpm/s), idling
// between passes, then a long idle to see how fast the engine cools down.
// It prints, per pass, the coolant and oil temperatures and the peak brake
// power, so a pass-to-pass power loss can be tied to the temperature.
//
// Deterministic: the simulator is stepped directly, 240 frames a second, as
// the application's runtime does.
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/foundation/EngineTypes.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/simulation/DynoAbsorberController.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numbers>
#include <string>

namespace {

using namespace enginelab;

constexpr double frameSeconds = 1.0 / 240.0;

/** The application's sweep endpoints (EngineRuntime.cpp). */
[[nodiscard]] double sweepCeilingRpm(const EngineConfig& config) {
    const auto physicalLimitRpm = std::min(config.redlineRpm, config.ignition.revLimitRpm);
    return std::max(500.0, std::min(0.95 * physicalLimitRpm, config.ignition.revLimitRpm - 300.0));
}

[[nodiscard]] double sweepEntryRpm(const EngineConfig& config) {
    const auto lowCylinderEntry = config.cylinders.size() <= 2U ? 1'800.0 : 0.0;
    return std::min(sweepCeilingRpm(config), std::max({ 1'000.0, config.idleRpm + 400.0, lowCylinderEntry }));
}

struct Rig final {
    explicit Rig(EngineConfig engine)
        : config(std::move(engine)), exhaust(ExhaustGraph::makeForEngine(config)),
          simulator(std::make_unique<EngineSimulator>(config, ecu, physics, events, exhaust)) {}
    EngineConfig config;
    SimpleEcuModel ecu;
    SimplifiedGasolinePhysics physics;
    FourStrokeEventGenerator events;
    ExhaustGraph exhaust;
    std::unique_ptr<EngineSimulator> simulator;
};

void idle(Rig& rig, double seconds) {
    EngineControls controls;
    controls.ignitionEnabled = true;
    const auto frames = static_cast<int>(seconds / frameSeconds);
    for (auto frame = 0; frame < frames; ++frame) {
        controls.starterEngaged = rig.simulator->state().rpm < 300.0;
        (void) rig.simulator->step(frameSeconds, controls);
    }
}

struct Pass final {
    double peakPowerKw { 0.0 };
    double peakPowerRpm { 0.0 };
    double minimumRpm { 1.0e9 };
};

/** Two seconds held at the entry, then the ramp to the ceiling. */
[[nodiscard]] Pass dynoPass(Rig& rig) {
    const auto entry = sweepEntryRpm(rig.config);
    const auto ceiling = sweepCeilingRpm(rig.config);
    DynoAbsorberController absorber(rig.config);
    absorber.reset(rig.simulator->state().rpm, rig.simulator->state().torqueNm);
    Pass pass;
    constexpr double holdSeconds = 2.0;
    constexpr double rampRpmPerSecond = 500.0;
    const auto rampSeconds = (ceiling - entry) / rampRpmPerSecond;
    const auto frames = static_cast<int>((holdSeconds + rampSeconds) / frameSeconds);
    for (auto frame = 0; frame < frames; ++frame) {
        const auto t = frame * frameSeconds;
        const auto target = entry + rampRpmPerSecond * std::max(0.0, t - holdSeconds);
        const auto output = absorber.advance(frameSeconds, std::min(target, ceiling), rig.simulator->state());
        EngineControls controls;
        controls.ignitionEnabled = true;
        controls.throttle = 1.0;
        controls.dynamometerTorqueNm = output.brakeTorqueNm;
        const auto state = rig.simulator->step(frameSeconds, controls).state;
        pass.minimumRpm = std::min(pass.minimumRpm, state.rpm);
        if (t < holdSeconds) continue;
        const auto powerKw = output.brakeTorqueNm * state.angularVelocityRadPerSecond / 1'000.0;
        if (powerKw > pass.peakPowerKw) {
            pass.peakPowerKw = powerKw;
            pass.peakPowerRpm = state.rpm;
        }
    }
    return pass;
}

void printTemperatures(const EngineState& state) {
    std::cout << "coolant " << std::setw(6) << state.coolantTemperatureC << " degC, oil " << std::setw(6)
              << state.oilTemperatureC << " degC, damage " << std::setprecision(3) << state.damage << ", wear "
              << state.wear << std::setprecision(1);
}

} // namespace

int main(int argc, char** argv) {
    std::filesystem::path catalogRoot = ENGINELAB_CATALOG_ROOT;
    std::string filter = "Yamaha CP2";
    int passes = 5;
    double idleSeconds = 20.0;
    double cooldownSeconds = 120.0;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--catalog-root" && index + 1 < argc) catalogRoot = argv[++index];
        else if (argument == "--filter" && index + 1 < argc) filter = argv[++index];
        else if (argument == "--passes" && index + 1 < argc) passes = std::stoi(argv[++index]);
        else if (argument == "--idle-seconds" && index + 1 < argc) idleSeconds = std::stod(argv[++index]);
        else if (argument == "--cooldown-seconds" && index + 1 < argc) cooldownSeconds = std::stod(argv[++index]);
        else {
            std::cout << "usage: " << argv[0]
                      << " [--filter name] [--passes N] [--idle-seconds S] [--cooldown-seconds S]"
                         " [--catalog-root path]\n";
            return argument == "--help" ? 0 : 2;
        }
    }
    const auto catalog = loadEngineCatalog(catalogRoot);
    const auto selected = selectSingleEngineCatalogEntry(catalog.entries, filter);
    if (!selected) {
        std::cerr << "\"" << filter << "\" does not select exactly one engine\n";
        return 2;
    }
    auto config = selected.entry->config;
    normaliseEngineConfig(config);
    auto rig = std::make_unique<Rig>(config);

    std::cout << std::fixed << std::setprecision(1) << config.name << ": " << passes << " dyno passes "
              << sweepEntryRpm(config) << " -> " << sweepCeilingRpm(config) << " rpm at 500 rpm/s, "
              << idleSeconds << " s of idle between them\n";
    idle(*rig, 10.0);
    std::cout << "  after 10 s of idle   ";
    printTemperatures(rig->simulator->state());
    std::cout << '\n';
    double firstPeakKw = 0.0;
    for (auto index = 0; index < passes; ++index) {
        const auto pass = dynoPass(*rig);
        if (index == 0) firstPeakKw = pass.peakPowerKw;
        std::cout << "  pass " << index + 1 << "  peak " << std::setw(6) << pass.peakPowerKw << " kW at "
                  << std::setw(6) << pass.peakPowerRpm << " rpm (" << std::showpos
                  << 100.0 * (pass.peakPowerKw / std::max(1.0e-9, firstPeakKw) - 1.0) << std::noshowpos
                  << " %), lowest " << std::setw(6) << pass.minimumRpm << " rpm; at its end ";
        printTemperatures(rig->simulator->state());
        std::cout << '\n';
        idle(*rig, idleSeconds);
    }
    const auto hot = rig->simulator->state().coolantTemperatureC;
    idle(*rig, cooldownSeconds);
    std::cout << "  after " << cooldownSeconds << " s more of idle ";
    printTemperatures(rig->simulator->state());
    std::cout << "; coolant " << std::showpos << rig->simulator->state().coolantTemperatureC - hot
              << std::noshowpos << " degC, rpm " << rig->simulator->state().rpm << '\n';
    return 0;
}
