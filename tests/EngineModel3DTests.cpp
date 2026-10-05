#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/exhaust/LegacyExhaustNetwork.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/render/CrankClock.hpp>
#include <enginelab/render/EngineModel3D.hpp>
#include <enginelab/render/GasFieldView.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <numbers>
#include <string>

namespace {
using namespace enginelab;
using namespace enginelab::render;

void require(bool condition, const std::string& message) {
    if (condition) return;
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
}

[[nodiscard]] bool finite(const Mat4& m) {
    for (const auto value : m.m)
        if (!std::isfinite(value)) return false;
    return true;
}

/** Volume enclosed by a closed triangle mesh (divergence theorem). */
[[nodiscard]] double enclosedVolume(const Mesh& mesh) {
    double volume = 0.0;
    for (std::size_t i = 0; i + 2U < mesh.indices.size(); i += 3U) {
        const auto a = mesh.vertices[mesh.indices[i]].position;
        const auto b = mesh.vertices[mesh.indices[i + 1U]].position;
        const auto c = mesh.vertices[mesh.indices[i + 2U]].position;
        volume += static_cast<double>(dot(a, cross(b, c))) / 6.0;
    }
    return std::abs(volume);
}

int stretchedPipes = 0;
int checkedPipes = 0;

/** A simulator with the models it borrows. */
struct Bench final {
    explicit Bench(const EngineConfig& config)
        : exhaust(ExhaustGraph::makeForEngine(config)),
          simulator(std::make_unique<EngineSimulator>(config, ecu, physics, events, exhaust)) {}
    SimpleEcuModel ecu;
    SimplifiedGasolinePhysics physics;
    FourStrokeEventGenerator events;
    ExhaustGraph exhaust;
    std::unique_ptr<EngineSimulator> simulator;
};

/** Every drawn exhaust component, runner and plenum shows the solver's own gas. */
void checkGasFieldBinding(const EngineModel3D& model) {
    const auto& config = model.config();
    Bench bench(config);
    bench.simulator->captureGasFieldNow();
    const auto& field = bench.simulator->gasField();
    require(field.sequence == 1, config.name + ": a capture is taken on request");
    const auto binding = bindGasField(model, field);
    for (std::size_t d = 0; d < model.ducts().size(); ++d) {
        const auto& duct = model.ducts()[d];
        const auto label = config.name + " path " + std::to_string(duct.pathId) + " element "
                         + std::to_string(duct.elementId);
        if (duct.kind == DuctKind::exhaustComponent) {
            // A resonator with no outlet is an acoustic side branch the gas
            // solver does not carry.
            const auto& path = *std::find_if(config.exhaustPaths.begin(), config.exhaustPaths.end(),
                [&duct](const ExhaustPathConfig& item) { return item.id == duct.pathId; });
            const auto network = path.network ? *path.network : makeEditableExhaustNetwork(path);
            const auto sideBranch = duct.componentType == ExhaustComponentType::resonator
                && std::none_of(network.connections.begin(), network.connections.end(),
                       [&duct](const ExhaustComponentConnectionConfig& c) { return c.fromComponentId == duct.elementId; });
            require(sideBranch || binding[d] >= 0, label + ": an exhaust component shows its solver duct");
        }
        if (duct.kind == DuctKind::intakeRunner || duct.kind == DuctKind::intakePlenum)
            require(binding[d] >= 0, label + ": an intake runner or plenum shows its solver gas");
        if (binding[d] < 0) continue;
        const auto& element = field.elements[static_cast<std::size_t>(binding[d])];
        require(element.sampleCount > 0, label + ": a bound element has samples");
        for (std::size_t s = 0; s < element.sampleCount; ++s)
            require(element.pressurePa[s] > 1'000.0F && element.gasTemperatureK[s] > 100.0F,
                    label + ": the captured gas is physical");
    }
    // Two components never share one solver element.
    for (std::size_t a = 0; a < binding.size(); ++a)
        for (std::size_t b = a + 1; b < binding.size(); ++b)
            require(binding[a] < 0 || binding[a] != binding[b] || model.ducts()[a].kind != DuctKind::exhaustComponent
                        || model.ducts()[b].kind != DuctKind::exhaustComponent,
                    config.name + ": two exhaust components are bound to one solver element");
}

/** The colours show the wave, not the mean: an exhaust held 200 kPa above
    ambient (a turbocharged one) still shows rarefaction and compression. */
void checkWaveColours(const EngineModel3D& model) {
    const auto& config = model.config();
    Bench bench(config);
    bench.simulator->captureGasFieldNow();
    auto field = bench.simulator->gasField();
    GasFieldView view;
    for (int k = 0; k < 200; ++k) {
        ++field.sequence;
        field.simulationTimeSeconds += 0.01;
        for (std::size_t e = 0; e < field.elements.size(); ++e) {
            auto& element = field.elements[e];
            for (std::size_t s = 0; s < element.sampleCount; ++s)
                element.pressurePa[s] = static_cast<float>(field.ambientPressurePa + 200'000.0
                    + 10'000.0 * std::sin(2.0 * std::numbers::pi * k / 16.0 + static_cast<double>(s + e)));
        }
        view.update(model, field);
    }
    require(view.valid(), config.name + ": the exhaust has a field to draw");
    require(view.pressureScalePa() > 5'000.0F && view.pressureScalePa() < 20'000.0F,
            config.name + ": the scale follows the wave, not the back pressure ("
                + std::to_string(view.pressureScalePa()) + " Pa)");
    bool below = false;
    bool above = false;
    for (const auto& part : view.parts())
        for (std::size_t s = 0; s < part.count; ++s) {
            below = below || part.samples[4 * s + 2] > part.samples[4 * s + 0];
            above = above || part.samples[4 * s + 0] > part.samples[4 * s + 2];
        }
    require(below && above, config.name + ": a wave over a high mean shows both violet and orange");
}

/** The field is captured at the angle asked for, and capturing it changes nothing. */
void checkGasFieldCapture(const EngineConfig& config) {
    Bench watched(config);
    Bench control(config);
    const auto controls = [](int frame) {
        const auto time = frame / 240.0;
        EngineControls value;
        value.ignitionEnabled = true;
        value.starterEngaged = time < 1.0;
        value.throttle = 0.35;
        value.load = 0.2;
        return value;
    };
    for (int frame = 0; frame < 360; ++frame) {
        (void)watched.simulator->step(1.0 / 240.0, controls(frame));
        (void)control.simulator->step(1.0 / 240.0, controls(frame));
    }
    require(watched.simulator->state().rpm > 400.0, config.name + ": the engine runs before the capture is checked");
    const auto cycle = watched.events.cycleDegrees();
    const auto forward = [cycle](double degrees) { return std::fmod(std::fmod(degrees, cycle) + cycle, cycle); };
    int captures = 0;
    for (int frame = 360; frame < 480; ++frame) {
        const auto target = forward(37.0 * frame);
        watched.simulator->trackGasFieldAngle(target);
        const auto before = watched.simulator->gasField().sequence;
        (void)watched.simulator->step(1.0 / 240.0, controls(frame));
        (void)control.simulator->step(1.0 / 240.0, controls(frame));
        const auto& field = watched.simulator->gasField();
        if (field.sequence == before) continue;
        ++captures;
        const auto late = forward(field.crankAngleDegrees - target);
        require(late <= watched.simulator->state().crankDegreesPerSolverStep + 1.0e-6,
                config.name + ": the field is captured within one sub-step after the angle asked for (late by "
                    + std::to_string(late) + " deg)");
        if (frame % 7 == 0) watched.simulator->captureGasFieldNow();
    }
    require(captures > 20, config.name + ": a tracked angle is captured once per crossing (" + std::to_string(captures) + ")");
    watched.simulator->stopGasFieldTracking();
    require(watched.simulator->state().rpm == control.simulator->state().rpm
                && watched.simulator->state().crankAngleDegrees == control.simulator->state().crankAngleDegrees,
            config.name + ": capturing the gas field does not change the simulation");
}

void checkStations(const EngineModel3D& model) {
    for (const auto& duct : model.ducts()) {
        const auto& part = model.parts()[duct.part];
        if (duct.centreline.size() < 2U) {
            require(part.stations.empty(), model.config().name + ": a box has no stations");
            continue;
        }
        require(part.stations.size() == part.mesh.vertices.size(), model.config().name + ": one station per duct vertex");
        const auto [low, high] = std::minmax_element(part.stations.begin(), part.stations.end());
        require(*low >= 0.0F && *high <= 1.0F && *low < 0.05F && *high > 0.95F,
                model.config().name + ": duct stations run from inlet to outlet");
    }
}

/** The intake and exhaust drawn from the engine's own configuration. */
void checkDucts(const EngineModel3D& model) {
    const auto& config = model.config();
    const auto& name = config.name;
    require(model.systemBounds().diagonal() >= model.bounds().diagonal(),
            name + ": the whole system frames at least the engine");
    for (const auto& duct : model.ducts()) {
        require(duct.part < model.parts().size(), name + ": every duct has a part");
        for (const auto& point : duct.centreline)
            require(std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z),
                    name + ": duct centrelines are finite");
    }

    // Every component of every exhaust graph is drawn once.
    for (const auto& path : config.exhaustPaths) {
        const auto network = path.network ? *path.network : makeEditableExhaustNetwork(path);
        for (const auto& component : network.components) {
            const auto drawn = std::count_if(model.ducts().begin(), model.ducts().end(), [&](const SceneDuct& duct) {
                return duct.kind == DuctKind::exhaustComponent && duct.pathId == path.id
                    && duct.elementId == component.id;
            });
            require(drawn == 1, name + ": exhaust component " + std::to_string(component.id) + " of path "
                                    + std::to_string(path.id) + " is drawn once");
            if (component.type != ExhaustComponentType::outlet || drawn != 1) continue;
            // An outlet is the open end of its pipe: no length of its own
            // unless one is authored.
            const auto& outlet = *std::find_if(model.ducts().begin(), model.ducts().end(), [&](const SceneDuct& duct) {
                return duct.kind == DuctKind::exhaustComponent && duct.pathId == path.id && duct.elementId == component.id;
            });
            const auto outletLength = polylineLength(outlet.centreline);
            require(component.lengthMm > 1.0 ? outletLength > 0.99 * component.lengthMm
                                             : outletLength <= 0.5 * component.diameterMm,
                    name + ": outlet " + std::to_string(component.id) + " is drawn as an open end ("
                        + std::to_string(outletLength) + " mm)");
        }
    }

    for (const auto& duct : model.ducts()) {
        const auto drawn = polylineLength(duct.centreline);
        const auto label = name + " path " + std::to_string(duct.pathId) + " element " + std::to_string(duct.elementId);
        if (duct.kind == DuctKind::exhaustComponent && duct.componentType == ExhaustComponentType::pipe
            && duct.authoredLengthMm > 1.0F) {
            // A pipe is never drawn shorter than it is; it is drawn longer
            // only when it must span more than its length.
            require(drawn > 0.99F * duct.authoredLengthMm, label + ": a pipe is never drawn shorter than authored ("
                        + std::to_string(drawn) + " for " + std::to_string(duct.authoredLengthMm) + " mm)");
            ++checkedPipes;
            if (drawn > 1.03F * duct.authoredLengthMm) ++stretchedPipes;
        }
        if (duct.kind == DuctKind::intakeRunner) {
            require(std::abs(drawn - duct.authoredLengthMm) < 0.03F * duct.authoredLengthMm,
                    label + ": a runner keeps its authored length (" + std::to_string(drawn) + " for "
                        + std::to_string(duct.authoredLengthMm) + " mm)");
        }
        if (duct.kind == DuctKind::intakePlenum || duct.kind == DuctKind::intakeAirbox) {
            const auto& path = *std::find_if(config.intakePaths.begin(), config.intakePaths.end(),
                [&duct](const IntakePathConfig& item) { return item.id == duct.pathId; });
            const auto authored = 1.0e6 * (duct.kind == DuctKind::intakePlenum ? path.geometry.plenumVolumeLitres
                                                                                : path.geometry.airboxVolumeLitres);
            const auto volume = enclosedVolume(model.parts()[duct.part].mesh);
            require(volume > 0.97 * authored, label + ": an intake volume is drawn at least as large as authored ("
                        + std::to_string(volume * 1.0e-6) + " L)");
            // Only a plenum too thin to cover its runners is drawn larger.
            require(duct.kind == DuctKind::intakePlenum || volume < 1.03 * authored,
                    label + ": an airbox is drawn with its authored volume");
        }
    }
}

void checkEngine(const EngineConfig& config) {
    const EngineModel3D model(config);
    const auto& name = config.name;
    require(model.cylinderCount() == config.cylinders.size(), name + ": one cylinder layout per configured cylinder");

    for (const auto& part : model.parts()) {
        require(!part.mesh.empty() && part.mesh.indices.size() % 3U == 0U, name + ": every part is a triangle mesh");
        for (const auto index : part.mesh.indices)
            require(index < part.mesh.vertices.size(), name + ": mesh indices stay inside the vertex array");
    }
    const auto& bounds = model.bounds();
    require(std::isfinite(bounds.diagonal()) && bounds.diagonal() > 100.0F && bounds.diagonal() < 20'000.0F,
            name + ": the engine bounds are finite and engine-sized");

    checkDucts(model);
    checkStations(model);
    checkGasFieldBinding(model);
    checkWaveColours(model);

    std::vector<SceneInstance> instances;
    for (int step = 0; step < 72; ++step) {
        const auto angle = 10.0 * step;
        ScenePoseInput input;
        input.crankAngleDegrees = angle;
        input.combustion.fill(1.0F);
        model.pose(input, instances);
        require(instances.size() == model.parts().size(), name + ": the pose places every part");
        for (const auto& instance : instances)
            require(finite(instance.transform) && std::isfinite(instance.intensity), name + ": pose transforms are finite");

        for (std::size_t c = 0; c < model.cylinderCount(); ++c) {
            const auto probe = model.probe(c, angle);
            const auto label = name + " cylinder " + std::to_string(c + 1) + " at " + std::to_string(angle) + " deg";
            // The rod drawn between the two pins must keep its real length:
            // the picture uses the simulator's own kinematics.
            require(std::abs(length(probe.wristPin - probe.bigEnd) - probe.rodLengthMm) < 0.5F,
                    label + ": the connecting rod keeps its length");
            // The piston slides on the cylinder axis and never meets the head.
            const auto offset = probe.wristPin - probe.deckCentre;
            const auto lateral = offset - probe.axis * dot(offset, probe.axis);
            require(length(lateral) < 0.5F + static_cast<float>(std::abs(config.cylinders[c].wristPinOffsetMm)),
                    label + ": the wrist pin stays on the cylinder axis");
            require(dot(probe.deckCentre - probe.crownCentre, probe.axis) > 0.5F,
                    label + ": the crown stays below the deck");
        }
    }
}

void checkCrankClock() {
    // Constant 3,000 rpm sampled at 30 Hz, displayed at 144 Hz.
    CrankClock clock;
    const auto rpm = 3'000.0;
    const auto exact = [rpm](double t) { return wrapCycleDegrees(123.0 + rpm * 6.0 * t); };
    double lastSample = -1.0;
    double previous = 0.0;
    for (int frame = 0; frame < 144 * 2; ++frame) {
        const auto t = frame / 144.0;
        if (t - lastSample >= 1.0 / 30.0) {
            clock.observe(t, exact(t), rpm, 1.0);
            lastSample = t;
        }
        const auto angle = clock.advance(t);
        if (frame > 0) {
            const auto step = wrapCycleDegrees(angle - previous);
            require(std::abs(step - rpm * 6.0 / 144.0) < 1.0,
                    "a steady engine turns by the same angle every frame");
        }
        if (t > 0.5) {
            const auto error = std::abs(wrapCycleDegrees(angle - exact(t) + 360.0) - 360.0);
            require(error < 0.5, "the drawn crank angle matches the simulated one in real time");
        }
        previous = angle;
    }

    // A speed change between samples is absorbed without a jump.
    CrankClock ramp;
    double simAngle = 0.0, simRpm = 1'000.0;
    lastSample = -1.0;
    previous = 0.0;
    for (int frame = 0; frame < 144 * 3; ++frame) {
        const auto t = frame / 144.0;
        // Exact integral of a linear ramp, so the reported rpm is the true speed.
        const auto previousRpm = simRpm;
        simRpm = 1'000.0 + 2'000.0 * t;
        simAngle = wrapCycleDegrees(simAngle + 0.5 * (previousRpm + simRpm) * 6.0 / 144.0);
        if (t - lastSample >= 1.0 / 30.0) {
            ramp.observe(t, simAngle, simRpm, 1.0);
            lastSample = t;
        }
        const auto angle = ramp.advance(t);
        if (t > 0.5) {
            const auto error = std::abs(wrapCycleDegrees(angle - simAngle + 360.0) - 360.0);
            require(error < 1.0, "the clock follows an accelerating engine (error " + std::to_string(error) + " deg)");
            require(wrapCycleDegrees(angle - previous) < 2.0 * simRpm * 6.0 / 144.0,
                    "an accelerating engine never jumps ahead");
        }
        previous = angle;
    }

    // Slow motion replays the current speed, freeze holds, pause stops.
    CrankClock slow;
    slow.observe(0.0, 0.0, 4'000.0, 1.0);
    (void)slow.advance(0.0);
    slow.setPlaybackFactor(1.0 / 250.0);
    const auto start = slow.advance(0.0);
    const auto end = slow.advance(0.1);
    require(std::abs(wrapCycleDegrees(end - start) - 4'000.0 * 6.0 / 250.0 * 0.1) < 1.0e-6,
            "1:250 turns the crank 250 times slower than the engine");
    slow.setPlaybackFactor(0.0);
    require(slow.advance(0.2) == slow.advance(0.3), "freeze holds the crank still");
    CrankClock paused;
    paused.observe(0.0, 90.0, 4'000.0, 0.0);
    require(paused.advance(0.0) == paused.advance(0.2), "a paused simulation does not turn the crank");
}
} // namespace

int main() {
    const auto catalogue = loadEngineCatalog(ENGINELAB_CATALOG_ROOT);
    require(catalogue.errors.empty() && !catalogue.entries.empty(), "the shipped catalogue loads");
    for (const auto& entry : catalogue.entries) checkEngine(entry.config);
    // Measured 2026-10-04: the four 140 mm pipes of the LS3's X, which must
    // cross between the banks, and the end primaries of the 2JZ and the I5.
    require(stretchedPipes <= 6, "no more exhaust pipes are drawn longer than authored than when measured ("
                                     + std::to_string(stretchedPipes) + ")");
    checkCrankClock();
    checkGasFieldCapture(catalogue.entries.front().config);
    std::cout << "EngineModel3D: " << catalogue.entries.size() << " engines checked; " << stretchedPipes << " of "
              << checkedPipes << " exhaust pipes drawn more than 3% longer than authored\n";
    return 0;
}
