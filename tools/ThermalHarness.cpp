// Does the engine's temperature drift over a dyno session?
//
// Reproduces what a user does in the application: start, idle, then several
// dyno passes in a row (wide-open throttle, the absorber ramping the engine
// from the application's sweep entry to its ceiling at 500 rpm/s), idling
// between passes, then a long idle to see how fast the engine cools down.
// It prints, per pass, the coolant and oil temperatures and the peak brake
// power, so a pass-to-pass power loss can be tied to the temperature.
//
// --realistic lets the temperatures free (EngineThermalModel); --air-speed
// is the air at the radiator during idles and passes (the application's dyno
// cell fan: 12 m/s), --warmup-minutes idles from cold first, --ride-seconds
// ends with a ride: the engine held at half its redline, a quarter throttle,
// in the air of --ride-air-speed.
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
double airSpeedMps = 0.0;

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
    controls.coolingAirSpeedMps = airSpeedMps;
    const auto frames = static_cast<int>(seconds / frameSeconds);
    for (auto frame = 0; frame < frames; ++frame) {
        controls.starterEngaged = rig.simulator->state().rpm < 300.0;
        (void) rig.simulator->step(frameSeconds, controls);
    }
}

/** Held at half the redline by a proportional brake, a quarter throttle. */
void ride(Rig& rig, double seconds, double rideAirSpeedMps) {
    EngineControls controls;
    controls.ignitionEnabled = true;
    controls.throttle = 0.25;
    controls.coolingAirSpeedMps = rideAirSpeedMps;
    const auto frames = static_cast<int>(seconds / frameSeconds);
    for (auto frame = 0; frame < frames; ++frame) {
        controls.dynamometerTorqueNm = std::max(0.0, 2.0 * (rig.simulator->state().rpm - 0.5 * rig.config.redlineRpm));
        (void) rig.simulator->step(frameSeconds, controls);
    }
}

struct Pass final {
    double peakPowerKw { 0.0 };
    double peakPowerRpm { 0.0 };
    double minimumRpm { 1.0e9 };
    double meanKnock { 0.0 };
    double peakKnock { 0.0 };
    /** Share of the pass above the simulator's "knocking" state (0.45). */
    double heavyKnockShare { 0.0 };
    double damageGain { 0.0 };
    /** Means over the ramp within 150 rpm of 80 % of the way to the ceiling. */
    struct Probe final {
        double samples { 0.0 };
        double torqueNm { 0.0 };
        double boostRatio { 0.0 };
        double afr { 0.0 };
        double advanceDeg { 0.0 };
        double volumetricEfficiency { 0.0 };
        double exhaustWallC { 0.0 };
        double knock { 0.0 };
    } probe;
};

/** Two seconds held at the entry, then the ramp to the ceiling. */
[[nodiscard]] Pass dynoPass(Rig& rig) {
    const auto entry = sweepEntryRpm(rig.config);
    const auto ceiling = sweepCeilingRpm(rig.config);
    DynoAbsorberController absorber(rig.config);
    absorber.reset(rig.simulator->state().rpm, rig.simulator->state().torqueNm);
    Pass pass;
    const auto damageBefore = rig.simulator->state().damage;
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
        controls.coolingAirSpeedMps = airSpeedMps;
        const auto state = rig.simulator->step(frameSeconds, controls).state;
        pass.minimumRpm = std::min(pass.minimumRpm, state.rpm);
        pass.meanKnock += state.knockLevel / frames;
        pass.peakKnock = std::max(pass.peakKnock, state.knockLevel);
        if (state.knockLevel > 0.45) pass.heavyKnockShare += 1.0 / frames;
        if (t < holdSeconds) continue;
        if (std::abs(state.rpm - (entry + 0.8 * (ceiling - entry))) < 150.0) {
            auto& probe = pass.probe;
            probe.samples += 1.0;
            probe.torqueNm += output.brakeTorqueNm;
            probe.boostRatio += state.boostPressureRatio;
            probe.afr += state.airFuelRatio;
            probe.advanceDeg += state.ignitionAdvanceDegrees;
            probe.volumetricEfficiency += state.volumetricEfficiency;
            probe.exhaustWallC += state.exhaustWallTemperatureC;
            probe.knock += state.knockLevel;
        }
        const auto powerKw = output.brakeTorqueNm * state.angularVelocityRadPerSecond / 1'000.0;
        if (powerKw > pass.peakPowerKw) {
            pass.peakPowerKw = powerKw;
            pass.peakPowerRpm = state.rpm;
        }
    }
    pass.damageGain = rig.simulator->state().damage - damageBefore;
    return pass;
}

void printTemperatures(const EngineState& state) {
    std::cout << "coolant " << std::setw(6) << state.coolantTemperatureC << " degC, oil " << std::setw(6)
              << state.oilTemperatureC << " degC" << (state.coolingFanOn ? " (fan on)" : "") << ", damage "
              << std::setprecision(3) << state.damage << ", wear " << state.wear << std::setprecision(1);
}

} // namespace

int main(int argc, char** argv) {
    std::filesystem::path catalogRoot = ENGINELAB_CATALOG_ROOT;
    std::string filter = "Yamaha CP2";
    int passes = 5;
    double idleSeconds = 20.0;
    double cooldownSeconds = 120.0;
    bool realistic = false;
    double warmupMinutes = 0.0;
    double rideSeconds = 0.0;
    double rideAirSpeedMps = 25.0;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--catalog-root" && index + 1 < argc) catalogRoot = argv[++index];
        else if (argument == "--filter" && index + 1 < argc) filter = argv[++index];
        else if (argument == "--passes" && index + 1 < argc) passes = std::stoi(argv[++index]);
        else if (argument == "--idle-seconds" && index + 1 < argc) idleSeconds = std::stod(argv[++index]);
        else if (argument == "--cooldown-seconds" && index + 1 < argc) cooldownSeconds = std::stod(argv[++index]);
        else if (argument == "--realistic") realistic = true;
        else if (argument == "--air-speed" && index + 1 < argc) airSpeedMps = std::stod(argv[++index]);
        else if (argument == "--warmup-minutes" && index + 1 < argc) warmupMinutes = std::stod(argv[++index]);
        else if (argument == "--ride-seconds" && index + 1 < argc) rideSeconds = std::stod(argv[++index]);
        else if (argument == "--ride-air-speed" && index + 1 < argc) rideAirSpeedMps = std::stod(argv[++index]);
        else {
            std::cout << "usage: " << argv[0]
                      << " [--filter name] [--passes N] [--idle-seconds S] [--cooldown-seconds S]"
                         " [--realistic] [--air-speed M/S] [--warmup-minutes M] [--ride-seconds S]"
                         " [--ride-air-speed M/S] [--catalog-root path]\n";
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
    if (realistic) config.thermal.regulated = false;
    normaliseEngineConfig(config);
    auto rig = std::make_unique<Rig>(config);

    std::cout << std::fixed << std::setprecision(1) << config.name << ": " << passes << " dyno passes "
              << sweepEntryRpm(config) << " -> " << sweepCeilingRpm(config) << " rpm at 500 rpm/s, "
              << idleSeconds << " s of idle between them\n";
    idle(*rig, 10.0);
    std::cout << "  after 10 s of idle   ";
    printTemperatures(rig->simulator->state());
    std::cout << '\n';
    for (auto minute = 1; minute <= static_cast<int>(warmupMinutes); ++minute) {
        idle(*rig, 60.0);
        std::cout << "  idle minute " << std::setw(3) << minute << "      ";
        printTemperatures(rig->simulator->state());
        std::cout << '\n';
    }
    double firstPeakKw = 0.0;
    for (auto index = 0; index < passes; ++index) {
        const auto pass = dynoPass(*rig);
        if (index == 0) firstPeakKw = pass.peakPowerKw;
        std::cout << "  pass " << index + 1 << "  peak " << std::setw(6) << pass.peakPowerKw << " kW at "
                  << std::setw(6) << pass.peakPowerRpm << " rpm (" << std::showpos
                  << 100.0 * (pass.peakPowerKw / std::max(1.0e-9, firstPeakKw) - 1.0) << std::noshowpos
                  << " %), lowest " << std::setw(6) << pass.minimumRpm << " rpm; knock mean " << std::setprecision(3)
                  << pass.meanKnock << " peak " << pass.peakKnock << ", heavy " << std::setprecision(1)
                  << 100.0 * pass.heavyKnockShare << " % of the pass, damage +" << std::setprecision(4)
                  << pass.damageGain << std::setprecision(1) << "; at its end ";
        printTemperatures(rig->simulator->state());
        std::cout << '\n';
        if (const auto& probe = pass.probe; probe.samples > 0.0) {
            const auto n = probe.samples;
            std::cout << "    at 80 % of the ramp: torque " << std::setprecision(2) << probe.torqueNm / n
                      << " Nm, boost " << std::setprecision(4) << probe.boostRatio / n << ", AFR "
                      << probe.afr / n << ", advance " << probe.advanceDeg / n << " deg, VE "
                      << probe.volumetricEfficiency / n << ", exhaust wall " << std::setprecision(1)
                      << probe.exhaustWallC / n << " degC, knock " << std::setprecision(3) << probe.knock / n
                      << std::setprecision(1) << '\n';
        }
        idle(*rig, idleSeconds);
    }
    const auto hot = rig->simulator->state().coolantTemperatureC;
    idle(*rig, cooldownSeconds);
    std::cout << "  after " << cooldownSeconds << " s more of idle ";
    printTemperatures(rig->simulator->state());
    std::cout << "; coolant " << std::showpos << rig->simulator->state().coolantTemperatureC - hot
              << std::noshowpos << " degC, rpm " << rig->simulator->state().rpm << '\n';
    for (auto elapsed = 0.0; elapsed + 1.0e-9 < rideSeconds; elapsed += 10.0) {
        ride(*rig, std::min(10.0, rideSeconds - elapsed), rideAirSpeedMps);
        std::cout << "  ride " << std::setw(5) << elapsed + 10.0 << " s at " << rideAirSpeedMps << " m/s, "
                  << std::setw(6) << rig->simulator->state().rpm << " rpm ";
        printTemperatures(rig->simulator->state());
        std::cout << '\n';
    }
    return 0;
}
