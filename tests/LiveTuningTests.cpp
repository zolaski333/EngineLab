// Live tuning: the running engine takes a new flywheel between two steps,
// without a restart. These tests drive the real simulator the way
// EngineRuntime does, deterministically.
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/runtime/EngineRuntime.hpp>
#include <enginelab/serialization/EngineEditScope.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace enginelab::tests {
namespace {

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

constexpr double frameSeconds = 1.0 / 240.0;

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

[[nodiscard]] EngineConfig catalogueEngine(std::string_view selector) {
    static const auto catalogue = loadEngineCatalog(ENGINELAB_CATALOG_ROOT);
    const auto selected = selectSingleEngineCatalogEntry(catalogue.entries, selector);
    require(static_cast<bool>(selected), "catalogue engine " + std::string(selector));
    auto config = selected.entry->config;
    normaliseEngineConfig(config);
    return config;
}

/** Every crankshaft, and so the flywheel, `factor` times heavier. */
[[nodiscard]] EngineConfig flywheelTimes(EngineConfig config, double factor) {
    for (auto& crankshaft : config.crankshafts) crankshaft.momentOfInertiaKgM2 *= factor;
    config.rotatingInertiaKgM2 *= factor;
    normaliseEngineConfig(config);
    return config;
}

/** Cranked to 1.5 s, idling to 4 s, then full throttle with no load. */
[[nodiscard]] EngineControls freeRev(double timeSeconds) {
    EngineControls controls;
    controls.ignitionEnabled = true;
    controls.starterEngaged = timeSeconds < 1.5;
    controls.throttle = timeSeconds < 4.0 ? 0.0 : 1.0;
    return controls;
}

/** A flywheel taken at 3 s, at idle: the free rev from 4 s climbs like an
 * engine built with that flywheel. */
void flywheelRevsLikeARestart() {
    for (const auto* name : { "Yamaha CP2", "2JZ" }) {
        const auto original = catalogueEngine(name);
        const auto edited = flywheelTimes(original, 2.0);
        require(EngineSimulator::settingsReplaceable(original, edited), std::string(name) + ": replaceable");
        Rig built(edited);
        Rig swapped(original);
        Rig untouched(original);
        const auto target = 0.75 * original.redlineRpm;
        std::array<double, 3> reached { -1.0, -1.0, -1.0 };
        double minimumRpm = 1.0e9;
        for (auto frame = 0; frame < 8 * 240; ++frame) {
            const auto t = frame * frameSeconds;
            if (frame == 3 * 240) {
                auto incoming = edited;
                require(swapped.simulator->replaceSettings(incoming), std::string(name) + ": the flywheel is taken");
            }
            const auto controls = freeRev(t);
            std::size_t index = 0;
            for (auto* rig : { &built, &swapped, &untouched }) {
                const auto& state = rig->simulator->step(frameSeconds, controls).state;
                if (t >= 4.0 && reached[index] < 0.0 && state.rpm >= target) reached[index] = t - 4.0;
                if (index == 1 && t >= 3.0 && t < 4.0) minimumRpm = std::min(minimumRpm, state.rpm);
                ++index;
            }
        }
        std::cout << "  " << name << ", flywheel x2: free rev to " << target << " rpm in built "
                  << reached[0] << " s, swapped " << reached[1] << " s, untouched " << reached[2]
                  << " s; lowest idle after the change " << minimumRpm << " rpm\n";
        require(reached[0] > 0.0 && reached[1] > 0.0 && reached[2] > 0.0,
                std::string(name) + ": every engine reaches the target");
        require(reached[0] > 1.3 * reached[2],
                std::string(name) + ": the heavier flywheel slows the rev (else this proves nothing)");
        require(minimumRpm > 0.5 * original.idleRpm, std::string(name) + ": the engine keeps idling");
        require(std::abs(reached[1] - reached[0]) <= 0.15 * (reached[0] - reached[2]),
                std::string(name) + ": the changed engine revs like one built with the flywheel");
    }
}

/** Another crankshaft layout is refused; the runtime and the edit scope
 * treat a flywheel as a setting. */
void flywheelIsASetting() {
    const auto engine = catalogueEngine("2JZ");
    const auto heavier = flywheelTimes(engine, 1.5);
    const auto scope = engineEditScope(engine, heavier);
    require(scope.settings && !scope.other && !scope.cylinders && !scope.exhaust && !scope.intake,
            "a flywheel edit is a settings edit only");
    auto turned = engine;
    turned.crankshafts.front().rotationRatio = 2.0;
    require(!EngineSimulator::settingsReplaceable(engine, turned), "another crankshaft ratio is refused");
    auto runtime = std::make_unique<EngineRuntime>(engine);
    require(runtime->applyLiveSettings(heavier), "the runtime takes a heavier flywheel");
    require(std::abs(runtime->engineConfig().rotatingInertiaKgM2 - heavier.rotatingInertiaKgM2) < 1.0e-12,
            "the runtime's configuration carries the new flywheel");
    require(!runtime->applyLiveSettings(turned), "the runtime refuses another crankshaft");
}

} // namespace

void liveTuningRegression() {
    flywheelIsASetting();
    flywheelRevsLikeARestart();
}

} // namespace enginelab::tests
