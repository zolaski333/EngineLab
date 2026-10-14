// Deterministic driving instrument: a timeline of driver inputs (pedals, gear
// lever, starter) played against one catalogue engine and its driveline,
// through the same arbitration EngineRuntime uses (advanceVehicle). It writes
// one CSV row per public frame and prints a summary: the engine-to-road ratio
// measured in each gear, 0-100 km/h, top speed and who held the throttle.
//
// It exists so a driving complaint ("the quickshifter changes the gear
// spacing", "the blipper blips with the clutch pulled") is reproduced by a
// file anyone can rerun, not by memory of a session in the app.
//
// Usage:
//   EngineLabDriveScenarioHarness --engine "CP2 MT-07" --builtin pull
//   EngineLabDriveScenarioHarness --scenario my.json --csv out.csv
//   options: --shift-style manual|sequential|dual_clutch --quickshifter 0|1
//            --blipper 0|1 --automatic 0|1 --duration <s> --list
//
// Scenario file (every event key optional except "t"):
//   { "engine": "CP2 MT-07", "duration_s": 30,
//     "events": [
//       { "t": 0, "start": true },                 starter until it idles
//       { "t": 2, "gear": 1 },                     0 = neutral, "R" = reverse
//       { "t": 2, "throttle": 1, "clutch": 1, "ramp_s": 0.9 },
//       { "t": 2, "shift_up_at_rpm": 8500 },       the driver upshifts there
//       { "t": 9, "shift": "down" },               or "up"
//       { "t": 9, "brake": 1, "starter": false, "ignition": true } ] }
// Pedal targets are reached linearly over "ramp_s" (0 = at once).

#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/runtime/DriverCommandArbiter.hpp>
#include <enginelab/runtime/DrivelineModel.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr double stepSeconds = 1.0 / 240.0;
using Json = nlohmann::json;
/** `ActiveLimit` bits, in bit order. */
constexpr std::array<const char*, 7> activeLimitNames {
    "rev_limiter", "knock_retard", "coolant_protection", "diesel_smoke",
    "diesel_quantity", "boost_at_target", "injector_saturated" };

struct ScenarioEvent final {
    double timeSeconds { 0.0 };
    std::optional<double> throttle;
    std::optional<double> clutch;
    std::optional<double> brake;
    double rampSeconds { 0.0 };
    /** Human numbering: 0 neutral, 1..N, -1 reverse. */
    std::optional<int> gear;
    int shift { 0 };
    std::optional<bool> starter;
    std::optional<bool> ignition;
    bool start { false };
    std::optional<double> shiftUpAtRpm;
};

struct Scenario final {
    std::string engine;
    double durationSeconds { 30.0 };
    std::vector<ScenarioEvent> events;
};

struct Options final {
    std::string engine;
    std::filesystem::path scenarioPath;
    std::string builtin;
    std::filesystem::path csvPath;
    std::optional<enginelab::ShiftStyle> shiftStyle;
    std::optional<bool> quickshifter;
    std::optional<bool> blipper;
    std::optional<bool> automatic;
    std::optional<double> durationSeconds;
    bool list { false };
};

[[noreturn]] void fail(const std::string& message) { throw std::runtime_error(message); }

bool parseFlag(const std::string& value) {
    if (value == "1" || value == "true" || value == "on") return true;
    if (value == "0" || value == "false" || value == "off") return false;
    fail("expected 0 or 1, got '" + value + "'");
}

Options parseOptions(int argc, char** argv) {
    Options options;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        const auto next = [&]() -> std::string {
            if (index + 1 >= argc) fail(argument + " needs a value");
            return argv[++index];
        };
        if (argument == "--engine") options.engine = next();
        else if (argument == "--scenario") options.scenarioPath = next();
        else if (argument == "--builtin") options.builtin = next();
        else if (argument == "--csv") options.csvPath = next();
        else if (argument == "--shift-style") {
            const auto name = next();
            options.shiftStyle = enginelab::parseShiftStyle(name);
            if (!options.shiftStyle) fail("unknown shift style '" + name + "'");
        } else if (argument == "--quickshifter") options.quickshifter = parseFlag(next());
        else if (argument == "--blipper") options.blipper = parseFlag(next());
        else if (argument == "--automatic") options.automatic = parseFlag(next());
        else if (argument == "--duration") options.durationSeconds = std::stod(next());
        else if (argument == "--list") options.list = true;
        else fail("unknown argument '" + argument + "'");
    }
    return options;
}

ScenarioEvent parseEvent(const Json& json) {
    ScenarioEvent event;
    event.timeSeconds = json.at("t").get<double>();
    if (json.contains("throttle")) event.throttle = json["throttle"].get<double>();
    if (json.contains("clutch")) event.clutch = json["clutch"].get<double>();
    if (json.contains("brake")) event.brake = json["brake"].get<double>();
    event.rampSeconds = json.value("ramp_s", 0.0);
    if (json.contains("gear")) {
        const auto& gear = json["gear"];
        if (gear.is_string()) {
            const auto name = gear.get<std::string>();
            if (name == "R") event.gear = -1;
            else if (name == "N") event.gear = 0;
            else fail("gear must be a number, \"N\" or \"R\"");
        } else {
            event.gear = gear.get<int>();
        }
    }
    if (json.contains("shift")) {
        const auto direction = json["shift"].get<std::string>();
        if (direction == "up") event.shift = 1;
        else if (direction == "down") event.shift = -1;
        else fail("shift must be \"up\" or \"down\"");
    }
    if (json.contains("starter")) event.starter = json["starter"].get<bool>();
    if (json.contains("ignition")) event.ignition = json["ignition"].get<bool>();
    event.start = json.value("start", false);
    if (json.contains("shift_up_at_rpm")) event.shiftUpAtRpm = json["shift_up_at_rpm"].get<double>();
    return event;
}

Scenario parseScenario(const Json& json) {
    Scenario scenario;
    scenario.engine = json.value("engine", std::string {});
    scenario.durationSeconds = json.value("duration_s", 30.0);
    for (const auto& event : json.at("events")) scenario.events.push_back(parseEvent(event));
    std::stable_sort(scenario.events.begin(), scenario.events.end(),
        [](const auto& left, const auto& right) { return left.timeSeconds < right.timeSeconds; });
    return scenario;
}

/** The engine's own ceiling: its limiter, or the redline below it. */
double ceilingRpm(const enginelab::EngineConfig& config) {
    return std::min(config.redlineRpm, config.ignition.revLimitRpm);
}

/** Built-in scenarios, written against the engine so they need no tuning. */
Scenario builtinScenario(const std::string& name, const enginelab::EngineConfig& config) {
    const auto shiftRpm = 0.9 * ceilingRpm(config);
    Json events;
    if (name == "pull") {
        // Idle, launch in first at full throttle, upshift at 90 % of the
        // ceiling through every gear.
        events = Json::array({
            { { "t", 0.0 }, { "start", true }, { "clutch", 0.0 } },
            { { "t", 3.0 }, { "gear", 1 } },
            { { "t", 3.5 }, { "throttle", 1.0 } },
            { { "t", 3.5 }, { "clutch", 1.0 }, { "ramp_s", 0.9 },
              { "shift_up_at_rpm", shiftRpm } } });
        return parseScenario(Json { { "duration_s", 45.0 }, { "events", events } });
    }
    if (name == "downshift-clutch") {
        // Up to third, lift, pull the clutch and drop two gears with it
        // pulled, then let it out: what a blipper must (not) do.
        events = Json::array({
            { { "t", 0.0 }, { "start", true }, { "clutch", 0.0 } },
            { { "t", 3.0 }, { "gear", 1 } },
            { { "t", 3.5 }, { "throttle", 1.0 } },
            { { "t", 3.5 }, { "clutch", 1.0 }, { "ramp_s", 0.9 },
              { "shift_up_at_rpm", 0.7 * ceilingRpm(config) } },
            { { "t", 12.0 }, { "shift_up_at_rpm", 0.0 }, { "throttle", 0.0 } },
            { { "t", 12.5 }, { "clutch", 0.0 }, { "ramp_s", 0.1 } },
            { { "t", 13.0 }, { "shift", "down" } },
            { { "t", 13.8 }, { "shift", "down" } },
            { { "t", 15.0 }, { "clutch", 1.0 }, { "ramp_s", 0.8 } } });
        return parseScenario(Json { { "duration_s", 20.0 }, { "events", events } });
    }
    fail("unknown builtin scenario '" + name + "' (pull, downshift-clutch)");
}

const enginelab::EngineConfig& selectEngine(const enginelab::EngineCatalogLoadResult& catalog,
                                            const std::string& selector) {
    if (selector.empty()) fail("no engine: pass --engine or set \"engine\" in the scenario");
    std::vector<const enginelab::EngineConfig*> matches;
    for (const auto& entry : catalog.entries) {
        if (entry.config.name == selector) return entry.config;
        if (entry.config.name.find(selector) != std::string::npos) matches.push_back(&entry.config);
    }
    if (matches.size() == 1) return *matches.front();
    std::string message = matches.empty() ? "no engine matches '" + selector + "'"
                                          : "'" + selector + "' matches several engines:";
    for (const auto* match : matches) message += "\n  " + match->name;
    fail(message);
}

/** A pedal moving linearly to its target. */
class Pedal final {
public:
    explicit Pedal(double value) : from_(value), to_(value) {}
    void moveTo(double target, double now, double rampSeconds) {
        from_ = value(now);
        to_ = std::clamp(target, 0.0, 1.0);
        start_ = now;
        ramp_ = std::max(0.0, rampSeconds);
    }
    [[nodiscard]] double value(double now) const {
        if (ramp_ <= 0.0) return to_;
        return from_ + (to_ - from_) * std::clamp((now - start_) / ramp_, 0.0, 1.0);
    }
private:
    double from_;
    double to_;
    double start_ { 0.0 };
    double ramp_ { 0.0 };
};

/** One engine, ECU and driveline, stepped as EngineRuntime steps them. */
struct Vehicle final {
    explicit Vehicle(const enginelab::EngineConfig& config)
        : exhaust(enginelab::ExhaustGraph::makeForEngine(config)),
          simulator(config, ecu, physics, events, exhaust), driveline(config) {}
    enginelab::SimpleEcuModel ecu;
    enginelab::SimplifiedGasolinePhysics physics;
    enginelab::FourStrokeEventGenerator events;
    enginelab::ExhaustGraph exhaust;
    enginelab::EngineSimulator simulator;
    enginelab::DrivelineModel driveline;
};

const char* authorityName(enginelab::ThrottleAuthority authority) {
    switch (authority) {
    case enginelab::ThrottleAuthority::driver: return "driver";
    case enginelab::ThrottleAuthority::shiftTorqueCut: return "torque_cut";
    case enginelab::ThrottleAuthority::blipper: return "blipper";
    }
    return "driver";
}

/** Human gear number from the driveline's index (-1 neutral, -2 reverse). */
int humanGear(int index) { return index == -2 ? -1 : index + 1; }

struct GearRatioSample final {
    std::vector<double> ratios;
};

double median(std::vector<double> values) {
    if (values.empty()) return 0.0;
    const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
    std::nth_element(values.begin(), middle, values.end());
    return *middle;
}

int run(const Options& options) {
    const auto catalog = enginelab::loadEngineCatalog(std::filesystem::path(ENGINELAB_CATALOG_ROOT));
    if (!catalog.errors.empty()) fail("the catalogue does not load");
    if (options.list) {
        for (const auto& entry : catalog.entries) std::printf("%s\n", entry.config.name.c_str());
        return EXIT_SUCCESS;
    }

    std::optional<Json> scenarioJson;
    if (!options.scenarioPath.empty()) {
        std::ifstream input(options.scenarioPath);
        if (!input) fail("cannot read " + options.scenarioPath.string());
        scenarioJson = Json::parse(input);
    }
    auto selector = options.engine;
    if (selector.empty() && scenarioJson) selector = scenarioJson->value("engine", std::string {});
    auto config = selectEngine(catalog, selector);
    if (options.shiftStyle) config.transmission.shiftStyle = *options.shiftStyle;
    if (options.quickshifter) config.transmission.quickshifter = *options.quickshifter;
    if (options.blipper) config.transmission.autoBlipper = *options.blipper;
    if (options.automatic) config.transmission.automaticShifting = *options.automatic;

    Scenario scenario;
    if (scenarioJson) scenario = parseScenario(*scenarioJson);
    else if (!options.builtin.empty()) scenario = builtinScenario(options.builtin, config);
    else fail("pass --scenario <file.json> or --builtin pull|downshift-clutch");
    if (options.durationSeconds) scenario.durationSeconds = *options.durationSeconds;

    const auto vehicle = std::make_unique<Vehicle>(config);
    const auto gearCount = static_cast<int>(config.transmission.gearRatios.size());
    const auto tireRadius = config.vehicle.tireRadiusM;
    const auto lockBandRpm = 2.0 * config.transmission.clutchLockSpeedRpm;

    std::ofstream csv;
    if (!options.csvPath.empty()) {
        csv.open(options.csvPath);
        if (!csv) fail("cannot write " + options.csvPath.string());
        csv << "time_s,rpm,speed_kmh,requested_gear,engaged_gear,driver_throttle,engine_throttle,"
               "throttle_authority,clutch_pedal,clutch_pressure,clutch_slip_rpm,engine_torque_nm,"
               "wheel_torque_nm,shift_in_progress,ignition_cut,blip_throttle,brake,knock_level,"
               "boost_ratio,fuel_g_s,soft_limiter,hard_limiter,traction_limited,"
               "active_rev_limit_rpm,active_limits\n";
    }

    Pedal throttle(0.0);
    Pedal clutch(1.0);
    Pedal brake(0.0);
    bool ignition = true;
    bool starter = false;
    double autoStartSince = -1.0;
    double shiftUpAtRpm = 0.0;
    std::size_t nextEvent = 0;

    std::vector<GearRatioSample> gearSamples(static_cast<std::size_t>(std::max(gearCount, 0)));
    std::array<std::size_t, 3> authorityTicks {};
    std::size_t ignitionCutTicks = 0;
    std::size_t softLimiterTicks = 0;
    std::size_t hardLimiterTicks = 0;
    std::array<std::size_t, activeLimitNames.size()> activeLimitTicks {};
    double motionStart = -1.0;
    double time100Kmh = -1.0;
    double time60Mph = -1.0;
    double topSpeedMps = 0.0;
    double maximumRpm = 0.0;

    const auto steps = static_cast<long long>(std::ceil(scenario.durationSeconds / stepSeconds));
    for (long long index = 0; index < steps; ++index) {
        const auto now = static_cast<double>(index) * stepSeconds;
        while (nextEvent < scenario.events.size() && scenario.events[nextEvent].timeSeconds <= now + 1.0e-12) {
            const auto& event = scenario.events[nextEvent++];
            if (event.throttle) throttle.moveTo(*event.throttle, now, event.rampSeconds);
            if (event.clutch) clutch.moveTo(*event.clutch, now, event.rampSeconds);
            if (event.brake) brake.moveTo(*event.brake, now, event.rampSeconds);
            if (event.gear) {
                const auto human = std::clamp(*event.gear, -1, gearCount);
                vehicle->driveline.requestGear(human == -1 ? -2 : human - 1);
            }
            if (event.shift > 0) vehicle->driveline.shiftUp();
            if (event.shift < 0) vehicle->driveline.shiftDown();
            if (event.starter) starter = *event.starter;
            if (event.ignition) ignition = *event.ignition;
            if (event.start) { ignition = true; autoStartSince = now; }
            if (event.shiftUpAtRpm) shiftUpAtRpm = *event.shiftUpAtRpm;
        }
        const auto& previous = vehicle->simulator.state();
        if (autoStartSince >= 0.0) {
            const auto cranking = previous.rpm < config.idleRpm * 0.85 && now - autoStartSince < 3.0;
            starter = cranking;
            if (!cranking) autoStartSince = -1.0;
        }

        const enginelab::DriverInputs inputs { throttle.value(now), clutch.value(now), brake.value(now) };
        const auto step = enginelab::advanceVehicle(vehicle->driveline, stepSeconds, previous, 0.0, inputs);
        enginelab::EngineControls controls;
        controls.ignitionEnabled = ignition;
        controls.starterEngaged = starter;
        enginelab::applyVehicleStep(controls, step);
        const auto frame = vehicle->simulator.step(stepSeconds, controls);
        const auto& state = frame.state;
        const auto& drive = step.drive;

        // The driver's own upshift at a set speed, once the last shift is done.
        const auto requested = vehicle->driveline.requestedGear();
        if (shiftUpAtRpm > 0.0 && state.rpm >= shiftUpAtRpm && !drive.shiftInProgress
                && requested >= 0 && requested < gearCount - 1 && drive.engagedGear == requested)
            vehicle->driveline.shiftUp();

        const auto speed = drive.vehicleSpeedMps;
        topSpeedMps = std::max(topSpeedMps, speed);
        maximumRpm = std::max(maximumRpm, state.rpm);
        if (motionStart < 0.0 && speed > 0.1) motionStart = now;
        if (motionStart >= 0.0 && time100Kmh < 0.0 && speed >= 100.0 / 3.6) time100Kmh = now - motionStart;
        if (motionStart >= 0.0 && time60Mph < 0.0 && speed >= 60.0 * 0.44704) time60Mph = now - motionStart;
        ++authorityTicks[static_cast<std::size_t>(step.throttleAuthority)];
        if (step.shiftIgnitionCut) ++ignitionCutTicks;
        if (state.ecuSoftRevLimiterActive) ++softLimiterTicks;
        if (state.ecuHardRevLimiterActive) ++hardLimiterTicks;
        for (std::size_t bit = 0; bit < activeLimitNames.size(); ++bit)
            if ((state.activeLimits & (1U << bit)) != 0U) ++activeLimitTicks[bit];

        const auto locked = drive.engagedGear >= 0 && drive.engagedGear < gearCount
            && !drive.shiftInProgress && !drive.tractionLimited
            && std::abs(drive.clutchSlipRpm) < lockBandRpm && speed > 2.0 && inputs.clutchPedal > 0.99;
        if (locked) {
            const auto wheelRadPerSecond = speed / tireRadius;
            gearSamples[static_cast<std::size_t>(drive.engagedGear)].ratios.push_back(
                state.angularVelocityRadPerSecond / wheelRadPerSecond);
        }

        if (csv.is_open()) {
            char row[640];
            std::snprintf(row, sizeof(row),
                "%.5f,%.2f,%.3f,%d,%d,%.4f,%.4f,%s,%.4f,%.4f,%.2f,%.3f,%.3f,%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%d,%d,%.0f,%u\n",
                now, state.rpm, speed * 3.6, humanGear(requested), humanGear(drive.engagedGear),
                inputs.throttle, step.engineThrottle, authorityName(step.throttleAuthority),
                inputs.clutchPedal, drive.clutchPressure, drive.clutchSlipRpm, state.torqueNm,
                drive.wheelTorqueNm, drive.shiftInProgress ? 1 : 0, step.shiftIgnitionCut ? 1 : 0,
                drive.blipThrottle, drive.brakePressure, state.knockLevel, state.boostPressureRatio,
                state.fuelFlowGramsPerSecond, state.ecuSoftRevLimiterActive ? 1 : 0,
                state.ecuHardRevLimiterActive ? 1 : 0, drive.tractionLimited ? 1 : 0,
                state.activeRevLimitRpm, static_cast<unsigned>(state.activeLimits));
            csv << row;
        }
    }

    std::printf("engine: %s\n", config.name.c_str());
    std::printf("gearbox: %s, quickshifter %s, auto-blipper %s, automatic %s\n",
        enginelab::shiftStyleName(config.transmission.shiftStyle),
        config.transmission.quickshifter ? "on" : "off",
        config.transmission.autoBlipper ? "on" : "off",
        config.transmission.automaticShifting ? "on" : "off");
    std::printf("gear  authored_ratio  measured_ratio  deviation_%%  samples\n");
    for (int gear = 0; gear < gearCount; ++gear) {
        const auto authored = config.transmission.gearRatios[static_cast<std::size_t>(gear)]
            * config.transmission.finalDriveRatio;
        const auto& samples = gearSamples[static_cast<std::size_t>(gear)].ratios;
        if (samples.empty()) {
            std::printf("%4d  %14.4f  %14s  %11s  %7d\n", gear + 1, authored, "-", "-", 0);
            continue;
        }
        const auto measured = median(samples);
        std::printf("%4d  %14.4f  %14.4f  %11.2f  %7zu\n", gear + 1, authored, measured,
            100.0 * (measured / authored - 1.0), samples.size());
    }
    const auto printTime = [](const char* label, double seconds) {
        if (seconds < 0.0) std::printf("%s: not reached\n", label);
        else std::printf("%s: %.2f s\n", label, seconds);
    };
    printTime("0-100 km/h from first motion", time100Kmh);
    printTime("0-60 mph from first motion", time60Mph);
    std::printf("top speed: %.1f km/h, maximum engine speed: %.0f rpm (ceiling %.0f)\n",
        topSpeedMps * 3.6, maximumRpm, ceilingRpm(config));
    std::printf("throttle authority ticks: driver %zu, torque cut %zu, blipper %zu\n",
        authorityTicks[0], authorityTicks[1], authorityTicks[2]);
    std::printf("ignition-cut ticks %zu, soft-limiter ticks %zu, hard-limiter ticks %zu\n",
        ignitionCutTicks, softLimiterTicks, hardLimiterTicks);
    std::printf("active-limit seconds:");
    for (std::size_t bit = 0; bit < activeLimitNames.size(); ++bit)
        std::printf(" %s %.2f", activeLimitNames[bit], static_cast<double>(activeLimitTicks[bit]) * stepSeconds);
    std::printf("\n");
    return EXIT_SUCCESS;
}
} // namespace

int main(int argc, char** argv) {
    try {
        return run(parseOptions(argc, argv));
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
