// Active limits: the rev limit the ECU applies now (its calibration's, not the
// authored one) and the bitmask of what holds the engine back this frame. The
// dyno's sweep and the over-rev damage will read the first; the app and the
// drive-scenario harness show the second.

#include <enginelab/calibration/EcuCalibration.hpp>
#include <enginelab/calibration/EcuCalibrationKeys.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/runtime/EngineRuntime.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <string_view>
#include <thread>
#include <variant>

namespace {
using enginelab::ActiveLimit;
using enginelab::hasActiveLimit;
constexpr double stepSeconds = 1.0 / 240.0;

void require(bool condition, std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

void publishRevLimit(enginelab::SimpleEcuModel& ecu, double rpm) {
    auto draft = enginelab::calibration::makeDraft(*ecu.calibrationStore()->snapshot());
    auto limiter = *draft.find(enginelab::calibration::keys::revLimit);
    std::get<enginelab::calibration::ScalarCalibration>(limiter).value = rpm;
    draft.set(std::move(limiter));
    require(ecu.calibrationStore()->publish(draft).published, "a rev limit must publish");
    ecu.beginFrame();
}

void checkEcu() {
    const auto config = enginelab::makeDefaultInlineFour();
    enginelab::SimpleEcuModel ecu;
    ecu.initialiseCalibration(config);
    ecu.beginFrame();
    require(enginelab::activeRevLimitRpm(config, *ecu.calibrationStore()->snapshot())
                == config.ignition.revLimitRpm,
        "without a tune the active rev limit is the authored one");

    publishRevLimit(ecu, 2'500.0);
    require(enginelab::activeRevLimitRpm(config, *ecu.calibrationStore()->snapshot()) == 2'500.0,
        "a tuned rev limit replaces the authored one");

    enginelab::EngineState state;
    state.rpm = 1'800.0;
    state.manifoldPressureKpa = config.ambientPressureKpa * 0.6;
    state.coolantTemperatureC = 90.0;
    state.simulationTimeSeconds = 1.0;
    enginelab::EngineControls controls;
    controls.ignitionEnabled = true;
    controls.throttle = 0.3;
    const auto quiet = ecu.evaluate(config, state, controls);
    require(quiet.revLimitRpm == 2'500.0, "the command carries the rev limit it applied");
    require(quiet.activeLimits == 0U, "a warm engine below its limit, without knock, is not limited");

    state.rpm = 2'600.0;
    const auto limited = ecu.evaluate(config, state, controls);
    require(!limited.sparkEnabled && hasActiveLimit(limited.activeLimits, ActiveLimit::revLimiter),
        "at the tuned limit the ECU cuts and says the rev limiter holds the engine");

    state.rpm = 1'800.0;
    state.simulationTimeSeconds = 5.0;
    state.knockLevel = 0.2;
    (void)ecu.evaluate(config, state, controls);
    const auto knocking = ecu.evaluate(config, state, controls);
    require(hasActiveLimit(knocking.activeLimits, ActiveLimit::knockRetard),
        "knock control pulling spark is reported");
    state.knockLevel = 0.01;
    require(!hasActiveLimit(ecu.evaluate(config, state, controls).activeLimits, ActiveLimit::knockRetard),
        "a retard below half a degree is not reported");

    state.knockLevel = 0.0;
    state.coolantTemperatureC = 115.0;
    require(hasActiveLimit(ecu.evaluate(config, state, controls).activeLimits, ActiveLimit::coolantProtection),
        "a hot engine's protection retard is reported");
}

struct Engine final {
    explicit Engine(const enginelab::EngineConfig& config)
        : exhaust(enginelab::ExhaustGraph::makeForEngine(config)),
          simulator(config, ecu, physics, events, exhaust) {}
    enginelab::SimpleEcuModel ecu;
    enginelab::SimplifiedGasolinePhysics physics;
    enginelab::FourStrokeEventGenerator events;
    enginelab::ExhaustGraph exhaust;
    enginelab::EngineSimulator simulator;
};

const enginelab::EngineConfig& findEngine(const enginelab::EngineCatalogLoadResult& catalog,
                                          std::string_view fragment) {
    const auto found = std::find_if(catalog.entries.begin(), catalog.entries.end(),
        [&](const auto& entry) { return entry.config.name.find(fragment) != std::string::npos; });
    require(found != catalog.entries.end(), "the engine must exist in the catalogue");
    return found->config;
}

/** Start, then run at full throttle, optionally held near a speed by a
 * proportional brake. Returns every limit bit seen once at full throttle. */
std::uint32_t fullThrottleLimits(Engine& engine, const enginelab::EngineConfig& config,
                                 double holdRpm, double seconds, double& activeRevLimit,
                                 bool& wastegateMatchesBit) {
    enginelab::EngineControls controls;
    controls.ignitionEnabled = true;
    for (int index = 0; index < static_cast<int>(3.0 / stepSeconds); ++index) {
        controls.starterEngaged = engine.simulator.state().rpm < config.idleRpm * 0.85 && index * stepSeconds < 2.0;
        (void)engine.simulator.step(stepSeconds, controls);
    }
    controls.starterEngaged = false;
    controls.throttle = 1.0;
    std::uint32_t seen = 0U;
    wastegateMatchesBit = true;
    for (int index = 0; index < static_cast<int>(seconds / stepSeconds); ++index) {
        const auto rpm = engine.simulator.state().rpm;
        controls.dynamometerTorqueNm = holdRpm > 0.0 ? std::clamp(4.0 * (rpm - holdRpm), 0.0, 2'000.0) : 0.0;
        const auto frame = engine.simulator.step(stepSeconds, controls);
        const auto& state = frame.state;
        seen |= state.activeLimits;
        activeRevLimit = state.activeRevLimitRpm;
        wastegateMatchesBit = wastegateMatchesBit
            && (state.wastegateOpening > 0.02) == hasActiveLimit(state.activeLimits, ActiveLimit::boostAtTarget);
    }
    return seen;
}

void checkSimulator(const enginelab::EngineCatalogLoadResult& catalog) {
    // A petrol engine revving free at full throttle meets its limiter.
    const auto& cp2 = findEngine(catalog, "CP2 MT-07");
    {
        auto engine = std::make_unique<Engine>(cp2);
        double activeRevLimit = 0.0;
        bool wastegateMatches = true;
        const auto seen = fullThrottleLimits(*engine, cp2, 0.0, 4.0, activeRevLimit, wastegateMatches);
        require(hasActiveLimit(seen, ActiveLimit::revLimiter), "a free-revving CP2 is held by its rev limiter");
        require(activeRevLimit == cp2.ignition.revLimitRpm, "the state publishes the authored limit untuned");
        require(!hasActiveLimit(seen, ActiveLimit::dieselSmokeLimit)
                && !hasActiveLimit(seen, ActiveLimit::dieselQuantityLimit)
                && !hasActiveLimit(seen, ActiveLimit::boostAtTarget),
            "a petrol, naturally aspirated engine reports no diesel or boost limit");
    }
    {
        auto engine = std::make_unique<Engine>(cp2);
        engine->ecu.initialiseCalibration(cp2);
        publishRevLimit(engine->ecu, 6'000.0);
        double activeRevLimit = 0.0;
        bool wastegateMatches = true;
        (void)fullThrottleLimits(*engine, cp2, 0.0, 4.0, activeRevLimit, wastegateMatches);
        require(activeRevLimit == 6'000.0, "the state publishes the tuned limit");
        require(engine->simulator.state().rpm < 6'400.0, "the tuned limiter holds the engine");
    }

    // A turbo diesel held at full load: one of its two fuel ceilings caps it
    // and its wastegate regulates the boost.
    const auto& tdi = findEngine(catalog, "2.0 TDI");
    require(tdi.fuel == enginelab::FuelType::diesel, "the TDI is a diesel");
    auto engine = std::make_unique<Engine>(tdi);
    double activeRevLimit = 0.0;
    bool wastegateMatches = true;
    const auto seen = fullThrottleLimits(*engine, tdi, 2'500.0, 8.0, activeRevLimit, wastegateMatches);
    require(hasActiveLimit(seen, ActiveLimit::dieselSmokeLimit)
            || hasActiveLimit(seen, ActiveLimit::dieselQuantityLimit),
        "a diesel at full load is capped by its smoke or quantity limit");
    require(wastegateMatches, "the boost-at-target bit follows the wastegate");
    require(hasActiveLimit(seen, ActiveLimit::boostAtTarget), "the TDI's wastegate regulates at full load");
}
/** Damage from over-revving a CP2 motored at `targetRpm` for 3 s: the
 * engine is driven, as by a missed downshift. */
double motoredDamage(const enginelab::EngineConfig& config, double revLimitRpm, double targetRpm) {
    auto engine = std::make_unique<Engine>(config);
    if (revLimitRpm > 0.0) {
        engine->ecu.initialiseCalibration(config);
        publishRevLimit(engine->ecu, revLimitRpm);
    }
    enginelab::EngineControls controls;
    controls.ignitionEnabled = true;
    for (int index = 0; index < static_cast<int>(2.0 / stepSeconds); ++index) {
        controls.starterEngaged = engine->simulator.state().rpm < config.idleRpm * 0.85 && index * stepSeconds < 1.5;
        (void)engine->simulator.step(stepSeconds, controls);
    }
    controls.starterEngaged = false;
    double heldRpm = 0.0;
    for (int index = 0; index < static_cast<int>(3.0 / stepSeconds); ++index) {
        const auto rpm = engine->simulator.state().rpm;
        // Positive external torque drives the crank.
        controls.externalTorqueNm = std::clamp(0.2 * (targetRpm - rpm), -400.0, 400.0);
        heldRpm = engine->simulator.step(stepSeconds, controls).state.rpm;
    }
    const auto damage = engine->simulator.state().damage;
    std::printf("  motored at %.0f rpm (held %.0f), limit %.0f: damage %.3g\n",
        targetRpm, heldRpm, revLimitRpm > 0.0 ? revLimitRpm : config.ignition.revLimitRpm, damage);
    require(std::abs(heldRpm - targetRpm) < 0.05 * targetRpm, "the motoring brake must hold its speed");
    return damage;
}

void checkOverrevDamage(const enginelab::EngineConfig& cp2) {
    require(cp2.redlineRpm < cp2.ignition.revLimitRpm, "the CP2's limiter sits above its redline");
    // Between the redline and the limiter: not an over-rev any more.
    require(motoredDamage(cp2, 0.0, 0.5 * (cp2.redlineRpm + cp2.ignition.revLimitRpm)) == 0.0,
        "running between the redline and the limiter does not damage the engine");
    // Past a lowered limiter, well under the redline: an over-rev.
    require(motoredDamage(cp2, 6'000.0, 7'000.0) > 0.0,
        "driven past a lowered limiter, the engine is damaged");
}

/** The ceiling of a dyno run the runtime accepts, with an optional tune. */
double dynoCeiling(const enginelab::EngineConfig& config, double revLimitRpm) {
    auto runtime = std::make_unique<enginelab::EngineRuntime>(config);
    if (revLimitRpm > 0.0) {
        auto draft = enginelab::calibration::makeDraft(*runtime->calibrationStore()->snapshot());
        auto limiter = *draft.find(enginelab::calibration::keys::revLimit);
        std::get<enginelab::calibration::ScalarCalibration>(limiter).value = revLimitRpm;
        draft.set(std::move(limiter));
        require(runtime->calibrationStore()->publish(draft).published, "a rev limit must publish");
    }
    runtime->setIgnitionEnabled(true);
    runtime->start();
    runtime->startDyno();
    auto run = runtime->currentDynoRun();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (run.engineName.empty() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        run = runtime->currentDynoRun();
    }
    runtime->stopDyno();
    runtime->stop();
    require(!run.engineName.empty(), "the dyno run must start");
    std::printf("  dyno, limit %.0f: sweep ceiling %.0f rpm\n",
        revLimitRpm > 0.0 ? revLimitRpm : config.ignition.revLimitRpm, run.sessionConfig.sweepCeilingRpm);
    return run.sessionConfig.sweepCeilingRpm;
}

void checkDynoCeiling(const enginelab::EngineConfig& cp2) {
    const auto authored = std::min(0.95 * std::min(cp2.redlineRpm, cp2.ignition.revLimitRpm),
                                   cp2.ignition.revLimitRpm - 300.0);
    require(std::abs(dynoCeiling(cp2, 0.0) - authored) < 1.0e-6,
        "untuned, the dyno keeps its validated ceiling");
    require(std::abs(dynoCeiling(cp2, 7'000.0) - 0.95 * 7'000.0) < 1.0e-6,
        "a lowered limiter lowers the dyno's ceiling");
    require(std::abs(dynoCeiling(cp2, 12'000.0) - 0.95 * 12'000.0) < 1.0e-6,
        "a raised limiter raises the dyno's ceiling past the redline");
}
} // namespace

int main() {
    try {
        checkEcu();
        const auto catalog = enginelab::loadEngineCatalog(std::filesystem::path(ENGINELAB_CATALOG_ROOT));
        require(catalog.errors.empty(), "the catalogue must load");
        checkSimulator(catalog);
        const auto& cp2 = findEngine(catalog, "CP2 MT-07");
        std::printf("over-rev damage against the active limiter\n");
        checkOverrevDamage(cp2);
        std::printf("dyno ceiling from the active limiter\n");
        checkDynoCeiling(cp2);
        std::printf("active limits: all checks passed\n");
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
