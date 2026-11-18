#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/catalog/EngineDesign.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustComponentResize.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/exhaust/LegacyExhaustNetwork.hpp>
#include <enginelab/foundation/TurboSizing.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/render/CrankClock.hpp>
#include <enginelab/render/EngineModel3D.hpp>
#include <enginelab/render/GasFieldView.hpp>
#include <enginelab/render/ListenerFrame.hpp>
#include <enginelab/render/RouteCheck.hpp>
#include <enginelab/render/RouteSolver.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <string>
#include <unordered_map>

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
std::array<std::size_t, 5> routeIssues {};

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
        // A geometry-only path's mid pipe is its own solver node, never a
        // cylinder's primary (audit RENDER-01).
        if (duct.kind == DuctKind::exhaustComponent && duct.componentType == ExhaustComponentType::pipe) {
            const auto& path = *std::find_if(config.exhaustPaths.begin(), config.exhaustPaths.end(),
                [&duct](const ExhaustPathConfig& item) { return item.id == duct.pathId; });
            if ((!path.network || path.network->components.empty()) && duct.elementId == editableMidpipeId(path))
                require(std::none_of(config.cylinders.begin(), config.cylinders.end(),
                            [&element](const auto& cylinder) { return cylinder.id == element.nodeId; }),
                        label + ": the mid pipe shows its own gas, not a primary's");
        }
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
    ambient (a turbocharged one) still shows rarefaction and compression, and
    so does the intake, below ambient, on the same scale. */
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
            const auto intake = element.kind == GasFieldElementKind::intakeRunner
                             || element.kind == GasFieldElementKind::intakePlenum;
            for (std::size_t s = 0; s < element.sampleCount; ++s)
                element.pressurePa[s] = static_cast<float>(field.ambientPressurePa + (intake ? -20'000.0 : 200'000.0)
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

    std::vector<bool> intakePart(model.parts().size(), false);
    for (const auto& duct : model.ducts())
        intakePart[duct.part] = duct.kind == DuctKind::intakeRunner || duct.kind == DuctKind::intakePlenum;
    std::size_t intakeShown = 0;
    bool intakeBelow = false;
    bool intakeAbove = false;
    for (const auto& part : view.parts()) {
        if (!intakePart[part.part]) continue;
        ++intakeShown;
        for (std::size_t s = 0; s < part.count; ++s) {
            intakeBelow = intakeBelow || part.samples[4 * s + 2] > part.samples[4 * s + 0];
            intakeAbove = intakeAbove || part.samples[4 * s + 0] > part.samples[4 * s + 2];
        }
    }
    const auto drawnIntakes = static_cast<std::size_t>(std::count(intakePart.begin(), intakePart.end(), true));
    require(intakeShown == drawnIntakes, config.name + ": every intake runner and plenum shows its wave");
    require(intakeBelow && intakeAbove, config.name + ": an intake wave below ambient shows both colours");
}

/** Largest departure of an intake runner or plenum sample from `referencePa`. */
double intakeDeparturePa(const GasFieldSnapshot& field, double referencePa) {
    double worst = 0.0;
    for (const auto& element : field.elements) {
        if (element.kind != GasFieldElementKind::intakeRunner && element.kind != GasFieldElementKind::intakePlenum)
            continue;
        for (std::size_t s = 0; s < element.sampleCount; ++s)
            worst = std::max(worst, std::abs(element.pressurePa[s] - referencePa));
    }
    return worst;
}

/** A stopped engine's intake holds still. Measured 2026-10-06 before the fix:
 * from a uniform ambient start, every catalogue engine rang its runners to
 * ±14 to ±111 kPa within a second, valves open or shut. */
void checkStoppedIntakeAtRest(const EngineConfig& config, bool shutDown) {
    Bench bench(config);
    for (int frame = 0; frame < 240; ++frame) {
        (void)bench.simulator->step(1.0 / 240.0, EngineControls {});
        if (frame % 24 != 23) continue;
        bench.simulator->captureGasFieldNow();
        const auto departure = intakeDeparturePa(bench.simulator->gasField(), config.ambientPressureKpa * 1'000.0);
        require(departure < 1.0, config.name + ": a never-started intake stays at ambient pressure (off by "
                                     + std::to_string(departure) + " Pa)");
    }
    if (!shutDown) return;
    // Run, then switch off: once the crank stops, the runners stand at their
    // plenum's pressure, which refills through the shut throttle.
    for (int frame = 0; frame < 6 * 240; ++frame) {
        EngineControls controls;
        controls.ignitionEnabled = frame < 4 * 240;
        controls.starterEngaged = frame < 360;
        (void)bench.simulator->step(1.0 / 240.0, controls);
        if (frame == 4 * 240 - 1)
            require(bench.simulator->state().rpm > 400.0, config.name + ": the engine runs before it is switched off");
    }
    require(bench.simulator->state().rpm == 0.0, config.name + ": the engine has stopped");
    for (int frame = 0; frame < 240; ++frame) {
        (void)bench.simulator->step(1.0 / 240.0, EngineControls {});
        if (frame % 24 != 23) continue;
        bench.simulator->captureGasFieldNow();
        const auto& field = bench.simulator->gasField();
        double runnerSpread = 0.0;
        for (const auto& element : field.elements) {
            if (element.kind != GasFieldElementKind::intakeRunner) continue;
            const auto plenumPa = std::find_if(field.elements.begin(), field.elements.end(), [&](const auto& e) {
                return e.kind == GasFieldElementKind::intakePlenum && e.pathIndex == element.pathIndex;
            })->pressurePa[0];
            for (std::size_t s = 0; s < element.sampleCount; ++s)
                runnerSpread = std::max(runnerSpread, static_cast<double>(std::abs(element.pressurePa[s] - plenumPa)));
        }
        require(runnerSpread < 1.0, config.name + ": a stopped engine's runners stand at their plenum's pressure (off by "
                                        + std::to_string(runnerSpread) + " Pa)");
    }
}

/** The pulsation strength holds still while the wave moves: over a quarter
    of a pulse every wave colour changes and no strength colour does; a
    pulsing duct is warm, a quiet one grey. */
void checkStrengthColours(const EngineModel3D& model) {
    const auto& config = model.config();
    Bench bench(config);
    bench.simulator->captureGasFieldNow();
    const auto start = bench.simulator->gasField();
    const auto run = [&](GasFieldView::Colouring colouring, double amplitudePa, int frames) {
        auto field = start;
        GasFieldView view;
        view.setColouring(colouring);
        std::vector<std::vector<float>> history;
        for (int k = 0; k < frames; ++k) {
            ++field.sequence;
            field.simulationTimeSeconds += 0.01;
            for (std::size_t e = 0; e < field.elements.size(); ++e)
                for (std::size_t s = 0; s < field.elements[e].sampleCount; ++s)
                    field.elements[e].pressurePa[s] = static_cast<float>(field.ambientPressurePa + 50'000.0
                        + amplitudePa * std::sin(2.0 * std::numbers::pi * k / 16.0 + static_cast<double>(s + e)));
            view.update(model, field);
            std::vector<float> colours;
            for (const auto& part : view.parts())
                colours.insert(colours.end(), part.samples.begin(), part.samples.begin() + 4 * part.count);
            history.push_back(std::move(colours));
        }
        return history;
    };
    const auto change = [](const std::vector<std::vector<float>>& history) {
        // The largest colour change over the last quarter pulse.
        float worst = 0.0F;
        const auto& last = history.back();
        for (std::size_t back = 1; back <= 4; ++back) {
            const auto& earlier = history[history.size() - 1 - back];
            for (std::size_t i = 0; i < last.size(); ++i)
                if (i % 4 != 3) worst = std::max(worst, std::abs(last[i] - earlier[i]));
        }
        return worst;
    };
    const auto wave = run(GasFieldView::Colouring::wave, 10'000.0, 400);
    const auto strength = run(GasFieldView::Colouring::strength, 10'000.0, 400);
    require(change(wave) > 0.1F, config.name + ": the live wave colours move");
    require(change(strength) < 0.02F, config.name + ": the strength colours hold still ("
                                          + std::to_string(change(strength)) + ")");
    bool warm = !strength.back().empty();
    for (std::size_t i = 0; i + 2 < strength.back().size(); i += 4)
        warm = warm && strength.back()[i] > 1.5F * strength.back()[i + 2];
    require(warm, config.name + ": every pulsing duct shows a warm strength colour");
    const auto still = run(GasFieldView::Colouring::strength, 0.0, 40);
    bool grey = !still.back().empty();
    for (std::size_t i = 0; i + 2 < still.back().size(); i += 4)
        grey = grey && still.back()[i] < still.back()[i + 2];
    require(grey, config.name + ": a duct that does not pulse stays grey");
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

/** The probe records one cell at every solver step, reads the cell the gas
    field shows, and changes nothing. */
void checkGasProbe(const EngineConfig& config) {
    Bench watched(config);
    Bench control(config);
    const auto controls = [](int frame) {
        EngineControls value;
        value.ignitionEnabled = true;
        value.starterEngaged = frame < 240;
        value.throttle = 0.35;
        value.load = 0.2;
        return value;
    };
    const auto step = [&](int frame) {
        (void)watched.simulator->step(1.0 / 240.0, controls(frame));
        (void)control.simulator->step(1.0 / 240.0, controls(frame));
    };
    int frame = 0;
    for (; frame < 360; ++frame) step(frame);
    watched.simulator->captureGasFieldNow();
    const auto elements = watched.simulator->gasField().elements;
    require(!elements.empty(), config.name + ": the gas field has elements to probe");

    // A whole cycle on the first element fills every crank bin.
    watched.simulator->probeGasField(0, 0);
    for (const auto end = frame + 120; frame < end; ++frame) step(frame);
    const auto& trace = watched.simulator->gasProbe();
    const auto filled = std::count_if(trace.pressurePa.begin(), trace.pressurePa.end(), [](float p) { return p > 0.0F; });
    require(filled == static_cast<std::ptrdiff_t>(GasProbeTrace::bins),
            config.name + ": a probed cycle fills every crank bin (" + std::to_string(filled) + ")");
    require(trace.sequence > 0 && trace.latestBin >= 0, config.name + ": the probe publishes its trace");

    // Every element and sample reads the same cell a capture shows.
    for (std::size_t e = 0; e < elements.size(); ++e) {
        const auto count = std::max<std::size_t>(1U, elements[e].sampleCount);
        const auto sample = static_cast<std::uint8_t>(e % count);
        watched.simulator->probeGasField(static_cast<std::int32_t>(e), sample);
        step(frame++);
        watched.simulator->captureGasFieldNow();
        const auto& now = watched.simulator->gasProbe();
        require(now.pressurePa[static_cast<std::size_t>(now.latestBin)]
                    == watched.simulator->gasField().elements[e].pressurePa[sample],
                config.name + ": the probe reads the cell the gas field shows (element " + std::to_string(e) + ")");
    }
    watched.simulator->stopGasProbe();
    require(watched.simulator->state().rpm == control.simulator->state().rpm
                && watched.simulator->state().crankAngleDegrees == control.simulator->state().crankAngleDegrees,
            config.name + ": probing the gas field does not change the simulation");
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

int turbosDrawn = 0;

/** Points on the surface of a cylinder solid. */
[[nodiscard]] std::vector<Vec3> surfaceOf(const EngineSolid& solid) {
    std::vector<Vec3> points;
    for (int a = 0; a < 24; ++a) {
        const auto angle = 2.0F * std::numbers::pi_v<float> * static_cast<float>(a) / 24.0F;
        const auto radial = solid.axes[0] * std::cos(angle) + solid.axes[2] * std::sin(angle);
        for (int k = 0; k <= 4; ++k) {
            const auto along = solid.half.y * (static_cast<float>(k) / 2.0F - 1.0F);
            points.push_back(solid.centre + solid.axes[1] * along + radial * solid.half.x);
        }
    }
    return points;
}

void checkTurbo(const EngineModel3D& model) {
    const auto& config = model.config();
    const auto& name = config.name;
    const auto& turbo = model.turbo();
    const auto wanted = config.forcedInduction.enabled && config.forcedInduction.type == ForcedInductionType::turbocharger;
    require(turbo.placed == wanted, name + ": a turbocharger is drawn when one is configured, and only then");
    if (!turbo.placed) return;
    ++turbosDrawn;

    // One turbo is drawn, so it is fed by every cylinder: a second path would
    // bypass it.
    const auto turboPath = std::find_if(config.exhaustPaths.begin(), config.exhaustPaths.end(),
        [&turbo](const ExhaustPathConfig& path) { return path.id == turbo.pathId; });
    require(turboPath != config.exhaustPaths.end() && turboPath->cylinderIds.size() == config.cylinders.size(),
            name + ": every cylinder's exhaust reaches the turbo");
    // The solver's turbine sits on that collector too, not at the outlets.
    const Bench bench(config);
    require(bench.simulator->exhaustTurbineFaceCount() == 1,
            name + ": the solver's turbine sits on the collector (" + std::to_string(
                bench.simulator->exhaustTurbineFaceCount()) + " faces)");

    // It is bolted to its collector, and the path resumes at its outlet.
    bool fed = false, resumes = false;
    for (const auto& duct : model.ducts()) {
        if (duct.pathId != turbo.pathId || duct.kind != DuctKind::exhaustComponent || duct.centreline.empty()) continue;
        if (duct.elementId == turbo.collectorId && length(duct.centreline.back() - turbo.scroll.front()) < 0.5F) fed = true;
        if (length(duct.centreline.front() - turbo.outlet) < 0.5F) resumes = true;
    }
    require(fed, name + ": the turbine volute starts at its collector's outlet");
    require(resumes, name + ": the exhaust resumes at the turbine outlet");
    require(std::abs(dot(turbo.outlet - turbo.turbineCentre, turbo.axis)) > 0.5F * turbo.turbineWidth
                && dot(turbo.turbineCentre - turbo.compressorCentre, turbo.axis) > 0.0F,
            name + ": gas leaves the turbine along the shaft, on the side away from the compressor");

    // Clear of the engine: the turbo's solids against the engine's.
    const auto ownSolids = turboSolids(turbo);
    const auto& solids = model.solids();
    require(solids.size() >= ownSolids.size(), name + ": the turbo's solids are listed with the engine's");
    float deepest = 0.0F;
    for (std::size_t s = 0; s + ownSolids.size() < solids.size(); ++s)
        for (const auto& own : ownSolids)
            for (const auto& point : surfaceOf(own)) deepest = std::max(deepest, -signedDistance(solids[s], point));
    require(deepest < 1.0F, name + ": the turbo stays out of the engine (" + std::to_string(deepest) + " mm in)");

    // The wheels turn about the shaft, and only they move.
    std::vector<SceneInstance> still, turned;
    ScenePoseInput input;
    model.pose(input, still);
    input.turboShaftDegrees = 90.0;
    model.pose(input, turned);
    int moved = 0, turbines = 0;
    for (std::size_t part = 0; part < still.size(); ++part) {
        const auto& a = still[part].transform;
        const auto& b = turned[part].transform;
        if (model.identify(static_cast<std::uint16_t>(part)).role == PartRole::turbo) ++turbines;
        if (a.m == b.m) continue;
        ++moved;
        require(model.identify(static_cast<std::uint16_t>(part)).role == PartRole::turbo,
                name + ": the shaft angle moves only the turbo's wheels");
        const auto origin = b.transformPoint({});
        const auto axis = b.transformDirection({ 0.0F, 1.0F, 0.0F });
        require((length(origin - turbo.turbineCentre) < 1.0e-3F || length(origin - turbo.compressorCentre) < 1.0e-3F)
                    && dot(axis, turbo.axis) > 0.999F,
                name + ": a wheel turns about the shaft");
        const auto spun = b.transformDirection({ 1.0F, 0.0F, 0.0F });
        const auto rest = a.transformDirection({ 1.0F, 0.0F, 0.0F });
        require(std::abs(dot(spun, rest)) < 1.0e-3F, name + ": a quarter turn of the shaft turns the wheel a quarter");
    }
    require(moved == 2 && turbines == 4, name + ": two wheels turn, in two housings");
}

int gearboxesDrawn = 0;

/** A gearbox is drawn wherever there are gears to change: one pair per
    gear at its ratio, the pairs in mesh at every input angle, the collar
    against the engaged gear (between first and second in neutral) and the
    output shaft turning with the engaged gear. */
void checkGearbox(const EngineModel3D& model) {
    const auto& config = model.config();
    const auto& name = config.name;
    const auto& box = model.gearbox();
    const auto& ratios = config.transmission.gearRatios;
    require(box.placed == (ratios.size() >= 2U), name + ": a gearbox is drawn when there are gears to change");
    if (!box.placed) return;
    ++gearboxesDrawn;
    const auto pairs = ratios.size();
    require(box.pairZ.size() == pairs && box.inputTeeth.size() == pairs && box.outputTeeth.size() == pairs,
            name + ": one gear pair per gear");
    for (std::size_t k = 0; k < pairs; ++k) {
        const auto drawn = static_cast<double>(box.outputTeeth[k]) / static_cast<double>(box.inputTeeth[k]);
        require(std::abs(drawn / ratios[k] - 1.0) < 0.06,
                name + ": gear " + std::to_string(k + 1) + "'s teeth give its ratio within 6 %");
    }
    const auto centre = length(box.outputAxis - box.inputAxis);
    const auto spacing = box.pairZ[1] - box.pairZ[0];
    const auto near = [](const Mat4& transform, Vec3 axis) {
        return std::abs(transform.at(0, 3) - axis.x) < 1.0e-3F && std::abs(transform.at(1, 3) - axis.y) < 1.0e-3F;
    };
    const auto angleOf = [](const Mat4& transform) { return std::atan2(transform.at(1, 0), transform.at(0, 0)); };
    std::vector<SceneInstance> instances;
    for (const auto engaged : { -1, 0, static_cast<int>(pairs) - 1 })
        for (const auto inputDegrees : { 0.0, 37.0, 1'234.5, 98'765.4 }) {
            ScenePoseInput input;
            input.gearboxInputDegrees = inputDegrees;
            input.engagedGear = engaged;
            model.pose(input, instances);
            std::optional<Mat4> inputShaft, outputShaft, collar;
            std::vector<Mat4> outputGears;
            for (const auto& instance : instances) {
                if (model.identify(instance.part).role != PartRole::gearbox) continue;
                const auto material = model.parts()[instance.part].material;
                if (material == SceneMaterial::forged && near(instance.transform, box.inputAxis)) inputShaft = instance.transform;
                else if (material == SceneMaterial::forged) outputGears.push_back(instance.transform);
                else if (material == SceneMaterial::steel) outputShaft = instance.transform;
                else if (material == SceneMaterial::intakeValve) collar = instance.transform;
            }
            const auto label = name + " in gear " + std::to_string(engaged) + " at " + std::to_string(inputDegrees) + " deg";
            require(inputShaft && outputShaft && collar && outputGears.size() == pairs,
                    label + ": the input shaft, output shaft, collar and every output gear are drawn");
            for (const auto& gear : outputGears)
                require(near(gear, box.outputAxis), label + ": the output gears turn about the output shaft");
            // Tooth centres of a pair never meet: they interleave half a pitch apart.
            for (std::size_t k = 0; k < pairs; ++k) {
                const auto teethIn = box.inputTeeth[k], teethOut = box.outputTeeth[k];
                const auto toothModule = 2.0F * centre / static_cast<float>(teethIn + teethOut);
                const auto teeth = [&](const Mat4& transform, int count) {
                    std::vector<Vec3> points;
                    const auto radius = 0.5F * toothModule * static_cast<float>(count) - 0.05F * toothModule;
                    for (int t = 0; t < count; ++t) {
                        const auto a = 2.0F * std::numbers::pi_v<float> * static_cast<float>(t) / static_cast<float>(count);
                        points.push_back(transform.transformPoint({ -radius * std::sin(a), radius * std::cos(a), box.pairZ[k] }));
                    }
                    return points;
                };
                float closest = std::numeric_limits<float>::max();
                for (const auto& a : teeth(*inputShaft, teethIn))
                    for (const auto& b : teeth(outputGears[k], teethOut)) closest = std::min(closest, length(a - b));
                require(closest > 1.2F * toothModule,
                        label + ": the teeth of pair " + std::to_string(k + 1) + " mesh ("
                            + std::to_string(closest / toothModule) + " modules apart)");
            }
            const auto collarZ = collar->at(2, 3);
            if (engaged >= 0) {
                const auto target = box.pairZ[static_cast<std::size_t>(engaged)];
                require(std::abs(collarZ - target) < 0.5F * spacing, label + ": the collar sits against the engaged gear");
                require(std::abs(std::remainder(angleOf(*outputShaft) - angleOf(outputGears[static_cast<std::size_t>(engaged)]),
                                                2.0F * std::numbers::pi_v<float>)) < 1.0e-3F,
                        label + ": the output shaft turns with the engaged gear");
            } else {
                require(collarZ > box.pairZ[0] + 0.3F * spacing && collarZ < box.pairZ[1] - 0.3F * spacing,
                        label + ": in neutral the collar sits between first and second");
            }
        }
}

/** Least distance, across the crank, from the drawn gearbox case to a barrel
    (from above the crank's sweep to the deck, a bore wide), mm; negative is
    the case inside a cylinder. Infinite without a gearbox. */
float gearboxBarrelClearance(const EngineModel3D& model) {
    const auto& box = model.gearbox();
    if (!box.placed) return std::numeric_limits<float>::infinity();
    std::vector<SceneInstance> instances;
    model.pose({}, instances);
    const auto along = normalise(Vec3 { box.outputAxis.x - box.inputAxis.x, box.outputAxis.y - box.inputAxis.y, 0.0F });
    const Vec3 across { -along.y, along.x, 0.0F };
    float lowAlong = 1.0e9F, highAlong = -1.0e9F, lowAcross = 1.0e9F, highAcross = -1.0e9F;
    for (const auto& instance : instances) {
        if (model.identify(instance.part).role != PartRole::gearbox
            || model.parts()[instance.part].material != SceneMaterial::blockShell)
            continue;
        for (const auto& vertex : model.parts()[instance.part].mesh.vertices) {
            const auto p = instance.transform.transformPoint(vertex.position) - box.inputAxis;
            lowAlong = std::min(lowAlong, dot(p, along));
            highAlong = std::max(highAlong, dot(p, along));
            lowAcross = std::min(lowAcross, dot(p, across));
            highAcross = std::max(highAcross, dot(p, across));
        }
    }
    const auto& config = model.config();
    float least = std::numeric_limits<float>::infinity();
    for (std::size_t i = 0; i < config.cylinders.size(); ++i) {
        const auto& cylinder = config.cylinders[i];
        const auto top = model.probe(i, 0.0), bottom = model.probe(i, 180.0);
        const auto crank = (top.bigEnd + bottom.bigEnd) * 0.5F;
        const auto axis = normalise(Vec3 { top.axis.x, top.axis.y, 0.0F });
        const auto bore = static_cast<float>(cylinder.boreMm);
        const auto start = Vec3 { crank.x, crank.y, 0.0F }
            + axis * static_cast<float>(0.5 * cylinder.strokeMm + 0.25 * cylinder.boreMm);
        const Vec3 end { top.deckCentre.x, top.deckCentre.y, 0.0F };
        for (int k = 0; k <= 32; ++k) {
            const auto p = start + (end - start) * (static_cast<float>(k) / 32.0F)
                - Vec3 { box.inputAxis.x, box.inputAxis.y, 0.0F };
            const auto a = dot(p, along), c = dot(p, across);
            const auto u = std::max({ 0.0F, lowAlong - a, a - highAlong });
            const auto v = std::max({ 0.0F, lowAcross - c, c - highAcross });
            least = std::min(least, std::sqrt(u * u + v * v) - 0.5F * bore);
        }
    }
    return least;
}

int superchargersDrawn = 0;

/** A supercharger is drawn where one is configured: fed by the airbox or the
    inlet duct, feeding the throttle through its charge pipe, clear of the
    engine, its impeller geared to the crank. */
void checkSupercharger(const EngineModel3D& model) {
    const auto& config = model.config();
    const auto& name = config.name;
    const auto& supercharger = model.supercharger();
    const auto& forced = config.forcedInduction;
    const auto wanted = forced.enabled && forced.type == ForcedInductionType::supercharger;
    require(supercharger.placed == wanted, name + ": a supercharger is drawn when one is configured, and only then");
    if (!supercharger.placed) return;
    ++superchargersDrawn;

    bool fed = false, feeds = false;
    for (const auto& duct : model.ducts()) {
        if (duct.pathId != supercharger.pathId) continue;
        if ((duct.kind == DuctKind::intakeAirbox || duct.kind == DuctKind::intakeInletDuct) && !duct.centreline.empty()
            && (length(duct.centreline.front() - supercharger.eye) < 0.5F
                || length(duct.centreline.back() - supercharger.eye) < 0.5F))
            fed = true;
        if (duct.kind == DuctKind::intakeThrottle) {
            // The charge pipe ends on the throttle body's mouth.
            float nearest = 1.0e9F;
            for (const auto& vertex : model.parts()[duct.part].mesh.vertices)
                nearest = std::min(nearest, length(vertex.position - supercharger.throttleInlet));
            feeds = nearest < 1.1F * supercharger.chargeRadius;
        }
    }
    require(fed, name + ": the airbox or the inlet duct feeds the supercharger's eye");
    require(feeds, name + ": the charge pipe ends at the throttle");
    require(dot(supercharger.eye - supercharger.centre, supercharger.axis) > 0.5F * supercharger.width
                && dot(supercharger.centre - supercharger.chargeOutlet, supercharger.axis) > 0.0F,
            name + ": air enters the eye on one side and leaves for the throttle on the other");

    const auto ownSolids = superchargerSolids(supercharger);
    const auto& solids = model.solids();
    float deepest = 0.0F;
    for (const auto& solid : solids) {
        if (length(solid.centre - ownSolids.front().centre) < 1.0e-3F) continue;
        for (const auto& point : surfaceOf(ownSolids.front()))
            deepest = std::max(deepest, -signedDistance(solid, point));
    }
    require(deepest < 1.0F, name + ": the supercharger stays out of the engine (" + std::to_string(deepest) + " mm in)");

    // Geared to the crank: 10 degrees of crank turn the impeller by the
    // drive ratio times as much, about its axis.
    std::vector<SceneInstance> still, turned;
    ScenePoseInput input;
    model.pose(input, still);
    input.crankAngleDegrees = 10.0;
    model.pose(input, turned);
    int impellers = 0;
    for (std::size_t part = 0; part < still.size(); ++part) {
        if (model.identify(static_cast<std::uint16_t>(part)).role != PartRole::supercharger) continue;
        const auto& a = still[part].transform;
        const auto& b = turned[part].transform;
        if (a.m == b.m) continue;
        ++impellers;
        require(length(b.transformPoint({}) - supercharger.centre) < 1.0e-3F
                    && dot(b.transformDirection({ 0.0F, 1.0F, 0.0F }), supercharger.axis) > 0.999F,
                name + ": the impeller turns about its axis");
        const auto expected = std::cos(10.0 * forced.superchargerDriveRatio * std::numbers::pi / 180.0);
        const auto turnedBy = dot(a.transformDirection({ 1.0F, 0.0F, 0.0F }), b.transformDirection({ 1.0F, 0.0F, 0.0F }));
        require(std::abs(turnedBy - static_cast<float>(expected)) < 1.0e-3F,
                name + ": the impeller turns at the drive ratio times the crank");
    }
    require(impellers == 1, name + ": one impeller turns with the crank");
}

int silencersKept = 0;
int lengthsGiven = 0;
int injectorsBehindDucts = 0;

/** Resizing a drawn exhaust component from the 3-D view changes that
    component, and only that, in the configuration the scene is drawn from. */
void checkExhaustResize(const EngineModel3D& model) {
    const auto& config = model.config();
    const auto& name = config.name;
    const SceneDuct* target = nullptr;
    std::optional<ExhaustComponentSize> size;
    for (const auto& duct : model.ducts()) {
        if (duct.kind != DuctKind::exhaustComponent) continue;
        const auto found = exhaustComponentSize(config, duct.pathId, duct.elementId);
        require(found.has_value(), name + ": every drawn exhaust component has a size to edit");
        if (found->lengthEditable)
            require(std::abs(found->lengthMm - duct.authoredLengthMm) < 0.5,
                    name + ": the editor shows the length the scene draws");
        // A pipe if there is one (the Merlin has only stacks).
        const auto pipe = duct.componentType == ExhaustComponentType::pipe;
        if (found->lengthEditable && (target == nullptr || (pipe && target->componentType != ExhaustComponentType::pipe))) {
            target = &duct;
            size = found;
        }
        require(found->lengthEditable, name + ": every exhaust component has a length to edit from the view");
        // A collector or an outlet with no length takes one from the view (it
        // could only be given one in the exhaust designer before), is drawn
        // at it, and goes back to none at 0.
        // A rotary's port passage is drawn as the port through its housing,
        // whatever its length: not checked here.
        if (found->lengthMm == 0.0 && config.cycle != EngineCycle::rotary
            && (duct.componentType == ExhaustComponentType::merge || duct.componentType == ExhaustComponentType::outlet)) {
            auto lengthened = config;
            const auto error = resizeExhaustComponent(lengthened, duct.pathId, duct.elementId, 120.0, found->diameterMm);
            require(error.empty(), name + ": a collector or an outlet with no length takes one (" + error + ")");
            const auto invalid = validateEngineConfig(lengthened);
            require(!invalid, name + ": the lengthened exhaust is valid (" + invalid.value_or("") + ")");
            const auto after = exhaustComponentSize(lengthened, duct.pathId, duct.elementId);
            require(after && std::abs(after->lengthMm - 120.0) < 1.0e-9, name + ": the length is the one asked");
            const EngineModel3D redrawn(lengthened);
            const auto drawn = std::any_of(redrawn.ducts().begin(), redrawn.ducts().end(), [&duct](const SceneDuct& item) {
                return item.kind == DuctKind::exhaustComponent && item.pathId == duct.pathId
                    && item.elementId == duct.elementId && std::abs(item.authoredLengthMm - 120.0F) < 0.5F;
            });
            require(drawn, name + ": the new length is drawn");
            auto none = lengthened;
            require(resizeExhaustComponent(none, duct.pathId, duct.elementId, 0.0, found->diameterMm).empty()
                        && exhaustComponentSize(none, duct.pathId, duct.elementId)->lengthMm == 0.0,
                    name + ": a length of 0 is none again");
            auto tooShort = lengthened;
            require(!resizeExhaustComponent(tooShort, duct.pathId, duct.elementId, 5.0, found->diameterMm).empty(),
                    name + ": a length under 10 mm other than 0 is refused");
            ++lengthsGiven;
        }
        // A silencer narrowed to its pipes would stop being one: refused.
        if (duct.componentType == ExhaustComponentType::muffler) {
            const auto& path = *std::find_if(config.exhaustPaths.begin(), config.exhaustPaths.end(),
                [&duct](const ExhaustPathConfig& item) { return item.id == duct.pathId; });
            if (!path.network) {
                auto narrowed = config;
                require(!resizeExhaustComponent(narrowed, duct.pathId, duct.elementId, found->lengthMm,
                                                path.geometry.outletDiameterMm).empty(),
                        name + ": a silencer no wider than its outlet is refused");
                ++silencersKept;
            }
        }
    }
    require(target != nullptr, name + ": the exhaust has a component with a length to resize");

    auto edited = config;
    auto outOfRange = config;
    require(!resizeExhaustComponent(outOfRange, target->pathId, target->elementId, size->lengthMm, 3.0).empty(),
            name + ": a 3 mm diameter is refused");
    const auto error = resizeExhaustComponent(edited, target->pathId, target->elementId, size->lengthMm + 80.0,
                                              size->diameterMm + 4.0);
    require(error.empty(), name + ": 80 mm longer and 4 mm wider is accepted (" + error + ")");
    const auto invalid = validateEngineConfig(edited);
    require(!invalid, name + ": the resized configuration is valid (" + invalid.value_or("") + ")");

    const EngineModel3D redrawn(edited);
    require(redrawn.ducts().size() == model.ducts().size(), name + ": resizing keeps every drawn duct");
    for (std::size_t d = 0; d < model.ducts().size(); ++d) {
        const auto& before = model.ducts()[d];
        const auto& after = redrawn.ducts()[d];
        if (before.kind != DuctKind::exhaustComponent) continue;
        const auto resized = (before.pathId == target->pathId && before.elementId == target->elementId)
            || (size->sharedByPrimaries && before.pathId == target->pathId
                && before.componentType == ExhaustComponentType::pipe);
        if (!resized) {
            require(after.authoredLengthMm == before.authoredLengthMm,
                    name + ": resizing one component leaves the others' lengths alone");
            continue;
        }
        require(std::abs(after.authoredLengthMm - (before.authoredLengthMm + 80.0F)) < 0.5F,
                name + ": the resized component is drawn 80 mm longer");
        if (before.componentType == ExhaustComponentType::pipe)
            require(!before.radii.empty() && std::abs(after.radii.front() - before.radii.front() - 2.0F) < 0.25F,
                    name + ": the resized pipe is drawn 4 mm wider");
    }
}

int portInjectorsDrawn = 0;
int directInjectorsDrawn = 0;

/** One injector per cylinder, named by a click, at its place: a port injector
    in the runner out of the head spraying at the intake valves, a direct one
    in the chamber's roof on the intake side spraying into the cylinder. */
void checkInjectors(const EngineModel3D& model) {
    const auto& config = model.config();
    const auto& name = config.name;
    const auto direct = config.injection.mode == InjectionMode::direct;
    // A rotor's three faces share its ports and its injector: one a rotor.
    const auto rotary = config.cycle == EngineCycle::rotary;
    const auto step = rotary ? std::size_t { 3 } : std::size_t { 1 };
    for (std::size_t c = 0; c < model.cylinderCount(); c += step) {
        const auto label = name + " cylinder " + std::to_string(c + 1) + ": ";
        const auto part = model.injectorPart(c);
        require(part < model.parts().size(), label + "an injector is drawn");
        const auto identity = model.identify(part);
        require(identity.role == PartRole::injector && identity.cylinder == static_cast<int>(c),
                label + "a click on the injector names it and its cylinder");
        const auto tip = model.injectorTip(c);
        const auto aim = model.injectorAim(c);
        require(std::abs(length(aim) - 1.0F) < 1.0e-3F, label + "the spray direction is a unit vector");
        // The nozzle ends at the tip: no vertex of the injector lies beyond it.
        float furthest = -std::numeric_limits<float>::max();
        for (const auto& vertex : model.parts()[part].mesh.vertices) furthest = std::max(furthest, dot(vertex.position, aim));
        require(std::abs(furthest - dot(tip, aim)) < 0.5F, label + "the injector's nozzle ends at its tip");
        const auto probe = model.probe(c, 0.0);
        const auto bore = static_cast<float>(config.cylinders[c].boreMm);
        const auto& intake = model.intakePorts()[c / step];
        const auto& exhaust = model.exhaustPorts()[c / step];
        require(length(tip - intake.inner) < length(tip - exhaust.inner), label + "the injector is on the intake side");
        if (direct) {
            // In the roof of the chamber, within the bore, spraying down into it.
            const auto offset = tip - probe.deckCentre;
            const auto height = dot(offset, probe.axis);
            require(height > 0.0F && height < 0.3F * bore, label + "a direct injector sits in the chamber's roof");
            require(length(offset - probe.axis * height) < 0.5F * bore, label + "a direct injector sits within the bore");
            require(dot(aim, probe.axis) < -0.5F, label + "a direct injector sprays into the cylinder");
            ++directInjectorsDrawn;
        } else if (config.cycle == EngineCycle::twoStroke && !intake.transferDuct.empty()) {
            // Out of the barrel, on the reed manifold, spraying through the reeds.
            const auto offset = tip - probe.deckCentre;
            require(length(offset - probe.axis * dot(offset, probe.axis)) > 0.5F * bore,
                    label + "a two-stroke's port injector sits out of the barrel");
            require(dot(aim, normalise(intake.position - tip)) > 0.99F,
                    label + "a two-stroke's port injector sprays through the reeds");
            ++portInjectorsDrawn;
        } else {
            // Out of the head (above the port's inner end) and spraying at the valves.
            require(dot(tip - probe.deckCentre, probe.axis) > 0.3F * bore, label + "a port injector sits out of the head");
            require(dot(aim, normalise(intake.inner - tip)) > 0.99F, label + "a port injector sprays at the intake valves");
            ++portInjectorsDrawn;
        }
        // A click on the injector picks it, even through the see-through
        // runner it sits in: aimed at its body from six sides, with the
        // intake's ducts in the way (the head's shell is not pickable in the
        // X-ray view).
        std::vector<SceneInstance> instances;
        model.pose({}, instances);
        Vec3 centre {};
        const auto& vertices = model.parts()[part].mesh.vertices;
        for (const auto& vertex : vertices) centre = centre + vertex.position;
        centre = centre * (1.0F / static_cast<float>(std::max<std::size_t>(1U, vertices.size())));
        // The see-through duct it sits in: its cylinder's runner.
        const auto cylinderId = config.cylinders[c].id;
        const auto flow = [&model, cylinderId](std::uint16_t p) {
            return std::any_of(model.ducts().begin(), model.ducts().end(), [p, cylinderId](const SceneDuct& duct) {
                return duct.part == p && duct.kind == DuctKind::intakeRunner && duct.elementId == cylinderId;
            }) && model.parts()[p].material == SceneMaterial::intakeFlow;
        };
        for (const auto direction : { Vec3 { 1, 0, 0 }, Vec3 { -1, 0, 0 }, Vec3 { 0, 1, 0 }, Vec3 { 0, -1, 0 },
                                      Vec3 { 0, 0, 1 }, Vec3 { 0, 0, -1 } }) {
            const auto origin = centre - direction * 5'000.0F;
            const auto hit = pickPart(model, instances, origin, direction,
                [&](std::uint16_t p) { return p == part || flow(p); });
            const auto duct = pickPart(model, instances, origin, direction, flow);
            const auto injector = pickPart(model, instances, origin, direction, [part](std::uint16_t p) { return p == part; });
            if (!injector) continue;
            // The runner's far wall, from the other side.
            const auto back = pickPart(model, instances, centre + direction * 5'000.0F, direction * -1.0F, flow);
            const auto inside = duct && back && duct->distance < injector->distance
                && 10'000.0F - back->distance > injector->distance;
            if (!duct || inside) {
                if (inside) ++injectorsBehindDucts;
                require(hit && hit->part == part, label + "a click on the injector picks it, not the runner around it");
            }
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
    const auto routes = checkRoutes(model);
    for (std::size_t k = 0; k < routeIssues.size(); ++k) routeIssues[k] += routes.count(static_cast<RouteIssueKind>(k));
    checkStations(model);
    checkTurbo(model);
    checkSupercharger(model);
    checkGearbox(model);
    checkGasFieldBinding(model);
    checkWaveColours(model);
    checkStrengthColours(model);
    checkExhaustResize(model);
    checkInjectors(model);

    // No rod's big end swings through a main journal: across a throw only its
    // pin lies between the webs (owner 2026-11-14: every rod went through the
    // crank, the I5's included). The big end drawn is 0.22 bore wide.
    if (config.cycle != EngineCycle::rotary)
        for (std::size_t c = 0; c < model.cylinderCount(); ++c) {
            if (config.cylinders[c].connectingRodType == ConnectingRodType::articulated) continue;
            const auto z = model.probe(c, 0.0).bigEnd.z;
            const auto half = 0.11F * static_cast<float>(config.cylinders[c].boreMm);
            for (const auto& journal : model.mainJournals())
                require(journal.zEnd <= z - half || journal.zStart >= z + half,
                        name + " cylinder " + std::to_string(c + 1) + ": its big end swings clear of the main journals");
        }

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

        // A rotor has no rod, pin or crown to check: its pose is above.
        for (std::size_t c = 0; c < model.cylinderCount() && config.cycle != EngineCycle::rotary; ++c) {
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

/** Tightest centreline bend, as the radius of the circle through points half
    a diameter apart along the arc, over the diameter. */
[[nodiscard]] float tightestBend(const std::vector<Vec3>& p, float diameter) {
    std::vector<float> arc(p.size(), 0.0F);
    for (std::size_t i = 1; i < p.size(); ++i) arc[i] = arc[i - 1U] + length(p[i] - p[i - 1U]);
    float tightest = std::numeric_limits<float>::max();
    for (std::size_t i = 1; i + 1U < p.size(); ++i) {
        std::size_t before = i, after = i;
        while (before > 0 && arc[i] - arc[before] < 0.5F * diameter) --before;
        while (after + 1U < p.size() && arc[after] - arc[i] < 0.5F * diameter) ++after;
        const auto ab = p[i] - p[before], bc = p[after] - p[i], ca = p[before] - p[after];
        const auto area2 = length(cross(ab, bc));
        if (area2 > 1.0e-6F) tightest = std::min(tightest, length(ab) * length(bc) * length(ca) / (2.0F * area2) / diameter);
    }
    return tightest;
}

void checkTubeContact() {
    // A wide body along +Z from its inlet plane at z = 0.
    const std::vector<Vec3> body { { 0, 0, 0 }, { 0, 0, 20 }, { 0, 0, 40 }, { 0, 0, 60 }, { 0, 0, 80 } };
    const std::vector<float> radii(body.size(), 60.0F);
    // Its open end is flat: just in front of the inlet, inside the rim, a
    // ball clear of the plane does not touch it, whatever the radius behind.
    require(tubeContact({ 40, 0, -10 }, 8.0F, body, &radii).depth < 0.0F,
            "a tube does not bulge out past its open end");
    const auto face = tubeContact({ 40, 0, -5 }, 8.0F, body, &radii);
    require(std::abs(face.depth - 3.0F) < 1.0e-3F && face.normal.z < -0.99F,
            "a ball pressing on an open end is pushed back out along the axis");
    const auto wall = tubeContact({ 70, 0, 40 }, 15.0F, body, &radii);
    require(std::abs(wall.depth - 5.0F) < 1.0e-3F && wall.normal.x > 0.99F, "a ball against the wall is pushed out radially");
}

void checkRouter() {
    // Two pipes whose first guesses pass through each other, the first also
    // through a box: relaxed, they part and clear the box, keep their length,
    // and bend no tighter than one diameter.
    std::vector<RoutedPipe> pipes(3);
    pipes[0].points = { { -300, 0, 0 }, { 0, 0, 10 }, { 300, 0, 0 } };
    pipes[0].startDirection = pipes[0].endDirection = { 1, 0, 0 };
    pipes[1].points = { { 0, -300, 0 }, { 0, 0, -10 }, { 0, 300, 0 } };
    pipes[1].startDirection = pipes[1].endDirection = { 0, 1, 0 };
    // A third, well clear and already smooth, stays where it was put.
    for (int k = 0; k <= 200; ++k) {
        const auto angle = std::numbers::pi_v<float> * static_cast<float>(k) / 200.0F;
        pipes[2].points.push_back({ -150.0F * std::cos(angle), 600.0F, 150.0F * std::sin(angle) });
    }
    pipes[2].startDirection = { 0, 0, 1 };
    pipes[2].endDirection = { 0, 0, -1 };
    for (std::size_t k = 0; k < pipes.size(); ++k) {
        pipes[k].radius = 20.0F;
        pipes[k].lengthMm = k == 2U ? polylineLength(pipes[k].points) : 720.0F;
        pipes[k].startLead = pipes[k].endLead = k == 2U ? 0.0F : 30.0F;
    }
    const auto clear = pipes[2].points;
    const std::vector<EngineSolid> solids { boxBetween({ -140, -20, -20 }, { -100, 20, 20 }) };
    relaxRoutes(pipes, {}, solids);
    for (std::size_t k = 0; k < 2U; ++k) {
        const auto& pipe = pipes[k];
        const auto label = "router pipe " + std::to_string(k);
        require(std::abs(polylineLength(pipe.points) - 720.0F) < 0.01F * 720.0F, label + " keeps its length");
        require(tightestBend(pipe.points, 40.0F) >= 1.0F, label + " bends no tighter than one diameter ("
                                                              + std::to_string(tightestBend(pipe.points, 40.0F)) + ")");
        for (const auto& point : pipe.points) {
            require(tubeContact(point, 20.0F, pipes[1U - k].points, nullptr, 20.0F).depth < 0.5F,
                    label + " no longer passes through the other");
            require(signedDistance(solids.front(), point) > 19.5F, label + " stays out of the box");
        }
    }
    require(length(pipes[0].points.front() - Vec3 { -300, 0, 0 }) < 1.0e-3F
                && length(pipes[0].points.back() - Vec3 { 300, 0, 0 }) < 1.0e-3F,
            "the router keeps the ends of a pipe");
    float drift = 0.0F;
    for (const auto& point : pipes[2].points) {
        float nearest = std::numeric_limits<float>::max();
        for (std::size_t i = 0; i + 1U < clear.size(); ++i) {
            const auto d = clear[i + 1U] - clear[i];
            const auto t = std::clamp(dot(point - clear[i], d) / dot(d, d), 0.0F, 1.0F);
            nearest = std::min(nearest, length(point - (clear[i] + d * t)));
        }
        drift = std::max(drift, nearest);
    }
    require(drift < 0.5F, "a pipe that already fits stays where it was put (" + std::to_string(drift) + " mm)");
}

int listenersChecked = 0;

/** A camera 4 m behind the drawn tailpipe, along it, hears the first outlet
 * on its acoustic axis; the frame keeps distances from the origin. */
void checkListenerFrame(const EngineConfig& config) {
    const EngineModel3D scene(config);
    const auto frame = ListenerFrame::of(scene);
    const Vec3 probe { 312.0F, -147.0F, 905.0F };
    const auto mapped = frame.acoustic(probe);
    require(std::abs(std::hypot(mapped.x, mapped.y, mapped.z) - 0.001 * length(probe)) < 1.0e-6,
            config.name + ": the listener frame keeps distances");
    require(std::abs(mapped.z - 0.001 * probe.y) < 1.0e-9, config.name + ": up stays up");
    const SceneDuct* outlet = nullptr;
    for (const auto& duct : scene.ducts())
        if (duct.kind == DuctKind::exhaustComponent && duct.componentType == ExhaustComponentType::outlet
            && duct.centreline.size() >= 2U) {
            outlet = &duct;
            break;
        }
    if (outlet == nullptr) return;
    const auto tip = outlet->centreline.back();
    auto along = tip - outlet->centreline[outlet->centreline.size() - 2U];
    along.y = 0.0F;
    if (length(along) < 1.0e-3F) return;
    const auto eye = tip + normalise(along) * 4'000.0F;
    const auto heard = frame.acoustic(eye);
    const auto& path = config.exhaustPaths.front();
    auto source = path.acousticPositionM;
    auto axis = path.acousticAxis;
    if (path.network)
        for (const auto& component : path.network->components)
            if (component.type == ExhaustComponentType::outlet) {
                source = component.acousticPositionM;
                axis = component.acousticAxis;
                break;
            }
    const auto dx = heard.x - source.x;
    const auto dy = heard.y - source.y;
    const auto cosine = (dx * axis.x + dy * axis.y)
        / std::max(1.0e-9, std::hypot(dx, dy) * std::hypot(axis.x, axis.y));
    std::cout << "  " << config.name << ": camera behind the drawn tailpipe at " << std::hypot(dx, dy)
              << " m from the outlet, cosine to its acoustic axis " << cosine << "\n";
    require(cosine > 0.9, config.name + ": a camera behind the drawn tailpipe is on the outlet's acoustic axis");
    ++listenersChecked;
}

int sourcesChecked = 0;
int inletDuctsChecked = 0;

/** Every outlet the exhaust compiles finds the tip drawn for it, every intake
 * path its drawn mouth, and a drawn turbo its place: the sound radiates from
 * where the picture shows the opening. */
void checkSoundSources(const EngineConfig& config) {
    const EngineModel3D scene(config);
    const auto frame = ListenerFrame::of(scene);
    const auto sources = frame.sources(scene);
    const auto graph = ExhaustGraph::makeForEngine(config);
    std::size_t outlets = 0;
    for (const auto& node : graph.nodes()) {
        if (node.type != ExhaustNodeType::outlet) continue;
        ++outlets;
        const auto placed = std::find_if(sources.exhaustOutlets.begin(), sources.exhaustOutlets.end(),
                                         [&node](const auto& placement) {
                                             return placement.pathIndex == node.pathIndex
                                                 && placement.componentId == node.sourceComponentId;
                                         });
        require(placed != sources.exhaustOutlets.end(),
                config.name + ": outlet " + std::to_string(node.sourceComponentId) + " of path "
                    + std::to_string(node.pathIndex) + " has a drawn place");
        // Its drawn tip, and the direction out of it.
        const auto drawn = std::find_if(scene.ducts().begin(), scene.ducts().end(), [&](const SceneDuct& duct) {
            return duct.kind == DuctKind::exhaustComponent && duct.componentType == ExhaustComponentType::outlet
                && duct.pathId == config.exhaustPaths[node.pathIndex].id
                && (node.sourceComponentId == 0U || duct.elementId == node.sourceComponentId);
        });
        require(drawn != scene.ducts().end() && drawn->centreline.size() >= 2U, config.name + ": the outlet is drawn");
        const auto tip = frame.acoustic(drawn->centreline.back());
        const auto before = frame.acoustic(drawn->centreline[drawn->centreline.size() - 2U]);
        const AcousticPoint3M out { tip.x - before.x, tip.y - before.y, tip.z - before.z };
        require(std::hypot(placed->positionM.x - tip.x, placed->positionM.y - tip.y, placed->positionM.z - tip.z) < 1.0e-6,
                config.name + ": an outlet sounds from its drawn tip");
        require((placed->axis.x * out.x + placed->axis.y * out.y + placed->axis.z * out.z)
                        / std::hypot(out.x, out.y, out.z) > 0.999,
                config.name + ": an outlet sounds along its drawn axis");
        require(std::hypot(tip.x, tip.y, tip.z) > 0.2, config.name + ": a drawn tailpipe is away from the origin");
    }
    require(outlets > 0 && sources.exhaustOutlets.size() == outlets,
            config.name + ": one drawn place per compiled outlet");
    const auto intakePaths = std::max<std::size_t>(1U, config.intakePaths.size());
    for (std::uint32_t path = 0; path < intakePaths; ++path) {
        const auto mouths = std::count_if(sources.intakeMouths.begin(), sources.intakeMouths.end(),
                                          [path](const auto& placement) { return placement.pathIndex == path; });
        require(mouths == 1, config.name + ": intake path " + std::to_string(path) + " has one drawn mouth");
    }
    for (const auto& mouth : sources.intakeMouths) {
        require(std::hypot(mouth.positionM.x, mouth.positionM.y, mouth.positionM.z) > 0.05,
                config.name + ": a drawn intake mouth is away from the origin");
        // With an open inlet duct, the mouth is its open end, the air coming in
        // along it.
        const auto pathId = config.intakePaths.empty() ? 1U : config.intakePaths[mouth.pathIndex].id;
        for (const auto& duct : scene.ducts()) {
            if (duct.kind != DuctKind::intakeInletDuct || !duct.opensToAtmosphere || duct.pathId != pathId
                || duct.centreline.size() < 2U)
                continue;
            const auto open = frame.acoustic(duct.centreline.front());
            const auto inward = frame.acoustic(duct.centreline[1]);
            const AcousticPoint3M out { open.x - inward.x, open.y - inward.y, open.z - inward.z };
            require(std::hypot(mouth.positionM.x - open.x, mouth.positionM.y - open.y, mouth.positionM.z - open.z) < 1.0e-6,
                    config.name + ": an inlet duct sounds from its open end");
            require((mouth.axis.x * out.x + mouth.axis.y * out.y + mouth.axis.z * out.z) / std::hypot(out.x, out.y, out.z)
                        > 0.999,
                    config.name + ": an inlet duct sounds out of its open end");
            ++inletDuctsChecked;
        }
    }
    require(sources.forcedInductionM.has_value() == (scene.turbo().placed || scene.supercharger().placed),
            config.name + ": a drawn turbo or supercharger sounds from its place");
    ++sourcesChecked;
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

/** A two-stroke is drawn as one is built (owner's report 2026-10-24: its
    exhaust left from the head, its carburettor faced the barrel and a pipe ran
    from it to the crankcase). The exhaust leaves from its window in the
    barrel; a reed on the crankcase feeds it below the barrel, a cylinder port
    through the barrel's back; the transfers rise in the flanks; a power valve
    is a blade that drops into the port's roof as it shuts. */
void checkTwoStrokeDrawing(EngineConfig config) {
    const auto label = config.name + ": ";
    config.twoStroke.powerValveClosedExhaustPortOpenDegrees = config.twoStroke.exhaustPortOpenDegrees + 10.0;
    config.twoStroke.powerValveOpeningStartRpm = 5'000.0;
    config.twoStroke.powerValveFullyOpenRpm = 8'000.0;
    for (const auto intake : { TwoStrokeIntake::crankcaseReed, TwoStrokeIntake::cylinderPort }) {
        config.twoStroke.intake = intake;
        const EngineModel3D model(config);
        for (std::size_t c = 0; c < model.cylinderCount(); ++c) {
            const auto bore = static_cast<float>(config.cylinders[c].boreMm);
            const auto stroke = static_cast<float>(config.cylinders[c].strokeMm);
            const auto probe = model.probe(c, 0.0);
            const auto crank = (probe.bigEnd + model.probe(c, 180.0).bigEnd) * 0.5F;
            const auto axis = probe.axis;
            const auto deck = dot(probe.deckCentre - crank, axis);
            const auto& exhaust = model.exhaustPorts()[c];
            const auto& port = model.intakePorts()[c];
            const auto side = normalise(exhaust.inner - crank - axis * dot(exhaust.inner - crank, axis));
            require(dot(exhaust.inner - probe.deckCentre, axis) < -0.25F * stroke,
                    label + "the exhaust leaves from its window in the barrel, not from the head");
            const auto window = port.inner - crank - axis * dot(port.inner - crank, axis);
            require(std::abs(dot(normalise(window), side)) < 0.5F, label + "the transfers open on the bore's flanks");
            const auto height = dot(port.position - crank, axis);
            if (intake == TwoStrokeIntake::crankcaseReed) {
                require(height < 0.5F * deck && dot(port.position - crank, side) < 0.0F,
                        label + "the reed is on the crankcase's back, under the barrel");
            } else {
                require(height > 0.5F * deck && dot(port.position - crank, side) < -0.6F * bore,
                        label + "a cylinder port's carburettor is on the barrel's back");
            }
            const auto blade = model.powerValvePart(c);
            require(blade < model.parts().size() && model.identify(blade).role == PartRole::powerValve,
                    label + "a fitted power valve is drawn and named by a click");
            std::vector<SceneInstance> open, shut;
            ScenePoseInput pose;
            model.pose(pose, open);
            pose.powerValveOpening = 0.0;
            model.pose(pose, shut);
            const auto& a = open[blade].transform.m;
            const auto& b = shut[blade].transform.m;
            const auto travel = dot(Vec3 { b[12] - a[12], b[13] - a[13], b[14] - a[14] }, axis);
            require(travel < -1.0F, label + "shut, the power valve's blade drops into the port's roof");
            // Its housing is cast with the barrel (owner 2026-11-14: a grey
            // box floated over the pipe): every solid the valve adds reaches
            // into the barrel's wall (0.70 bore from the axis).
            auto bare = config;
            bare.twoStroke.powerValveClosedExhaustPortOpenDegrees = 0.0;
            const EngineModel3D unvalved(bare);
            int added = 0;
            for (const auto& solid : model.solids()) {
                const auto known = std::any_of(unvalved.solids().begin(), unvalved.solids().end(),
                    [&solid](const EngineSolid& other) { return length(other.centre - solid.centre) < 0.01F; });
                if (known || solid.shape != EngineSolid::Shape::box) continue;
                const auto offset = solid.centre - crank;
                const auto radial = offset - axis * dot(offset, axis);
                if (length(radial) < 1.0e-3F) continue;
                const auto outward = normalise(radial);
                auto innermost = length(radial);
                for (int k = 0; k < 3; ++k)
                    innermost -= std::abs(dot(solid.axes[static_cast<std::size_t>(k)], outward))
                        * (k == 0 ? solid.half.x : k == 1 ? solid.half.y : solid.half.z);
                // Only this cylinder's: a twin's other barrel stands 1.5 bores
                // along the crank, a V's other bank elsewhere.
                const auto along = normalise(cross(axis, side));
                if (length(radial) > 2.0F * bore || std::abs(dot(offset, axis)) > 3.0F * bore
                    || std::abs(dot(offset, along)) > 0.6F * bore)
                    continue;
                ++added;
                require(innermost < 0.70F * bore, label + "the power valve's housing is cast with the barrel ("
                        + std::to_string(innermost / bore) + " bore from the axis)");
            }
            require(added > 0, label + "the power valve's housing is drawn");
        }
    }
    config.twoStroke.powerValveClosedExhaustPortOpenDegrees = 0.0;
    const EngineModel3D without(config);
    require(without.powerValvePart(0) == 0xFFFFU, label + "without a power valve no blade is drawn");
}

/** An H-pipe is drawn as an H: each side's pipe keeps its lane through its
 *  junction, and the balance pipe crosses between them. Its side branch was
 *  laid out as a fork and a collector: the left pipe stepped aside and the
 *  right one ran down the middle (owner report 2026-10-30). */
/** A rotor's faces open onto one port: its pipe leaves from that port, with
    no junction out in the air holding a feeder (owner report 2026-10-30). */
void checkRotaryExhaust(const EngineConfig& config) {
    const EngineModel3D model(config);
    const auto& name = config.name;
    checkDucts(model);
    for (const auto& duct : model.ducts())
        require(duct.kind != DuctKind::exhaustFeeder, name + ": no rotor port is fed into a junction");
    for (std::size_t rotor = 0; rotor < config.cylinders.size() / 3U; ++rotor) {
        const auto face = config.cylinders[3U * rotor].id;
        const auto port = std::find_if(model.exhaustPorts().begin(), model.exhaustPorts().end(),
            [face](const PortAnchor& anchor) { return anchor.cylinderId == face; });
        require(port != model.exhaustPorts().end(), name + ": each rotor has an exhaust port");
        const auto leaves = std::any_of(model.ducts().begin(), model.ducts().end(), [&](const SceneDuct& duct) {
            return duct.kind == DuctKind::exhaustComponent && duct.componentType == ExhaustComponentType::pipe
                && !duct.centreline.empty() && length(duct.centreline.front() - port->position) < port->diameterMm;
        });
        require(leaves, name + ": rotor " + std::to_string(rotor + 1) + "'s pipe leaves from its port");
    }
}

void checkRotaryExhausts(const EngineCatalogLoadResult& catalogue) {
    for (const auto& entry : catalogue.entries)
        if (entry.config.cycle == EngineCycle::rotary) checkRotaryExhaust(entry.config);
    EngineSpec spec;
    spec.name = "Designed three-rotor";
    spec.cycle = EngineCycle::rotary;
    spec.use = EngineUse::sportCar;
    spec.cylinderCount = 3;
    spec.displacementLitres = 1.962;
    const auto designed = designCatalogueEngine(spec, ENGINELAB_CATALOG_ROOT);
    require(designed.config.has_value(), "a three-rotor designs: " + designed.error);
    checkRotaryExhaust(*designed.config);
}

/** A single has no collector: its turbo is drawn where the one primary ends,
    where the simulator puts its turbine, and clicks as a turbocharger (it was
    not drawn at all, so its settings could not be reached). */
void checkSingleCylinderTurbo() {
    EngineSpec spec;
    spec.name = "Designed turbo single";
    spec.use = EngineUse::roadMotorcycle;
    spec.cylinderCount = 1;
    spec.displacementLitres = 0.5;
    spec.induction = Induction::turbocharged;
    const auto designed = designCatalogueEngine(spec, ENGINELAB_CATALOG_ROOT);
    require(designed.config.has_value(), "a turbo single designs: " + designed.error);
    const EngineModel3D model(*designed.config);
    require(model.turbos().size() == 1U && model.turbo().placed, "a single's turbo is drawn");
    const auto& turbo = model.turbo();
    // The primary, the duct its port opens into, ends on the turbine's inlet.
    const auto primary = std::find_if(model.ducts().begin(), model.ducts().end(), [&](const SceneDuct& duct) {
        return duct.kind == DuctKind::exhaustComponent && duct.componentType == ExhaustComponentType::pipe
            && duct.centreline.size() >= 2U && length(duct.centreline.front() - model.exhaustPorts().front().position) < 1.0F;
    });
    require(primary != model.ducts().end(), "the single's primary is drawn from its port");
    require(!turbo.scroll.empty() && length(primary->centreline.back() - turbo.scroll.front()) < 2.0F,
            "the single's primary ends on the turbine");
    auto clickable = 0;
    for (std::uint16_t part = 0; part < model.parts().size(); ++part)
        if (model.identify(part).role == PartRole::turbo) ++clickable;
    require(clickable > 0, "a click on the single's turbo names it");
    const auto invalid = validateEngineConfig(*designed.config);
    require(!invalid, "the turbo single is valid (" + invalid.value_or("") + ")");
}

void checkHPipeDrawing() {
    EngineSpec spec;
    spec.name = "Designed H-pipe V8";
    spec.use = EngineUse::sportCar;
    spec.layout = EngineLayout::vLayout;
    spec.cylinderCount = 8;
    spec.displacementLitres = 5.0;
    spec.exhaustSystem = ExhaustSystem::dualHPipe;
    const auto designed = designCatalogueEngine(spec, ENGINELAB_CATALOG_ROOT);
    require(designed.config.has_value(), "an H-pipe V8 designs: " + designed.error);
    const EngineModel3D model(*designed.config);
    const auto duct = [&](std::uint32_t id) -> const SceneDuct& {
        const auto found = std::find_if(model.ducts().begin(), model.ducts().end(), [id](const SceneDuct& candidate) {
            return candidate.kind == DuctKind::exhaustComponent && candidate.elementId == id
                && candidate.centreline.size() >= 2U;
        });
        require(found != model.ducts().end(), "the H's pipe " + std::to_string(id) + " is drawn");
        return *found;
    };
    const auto across = [](Vec3 a, Vec3 b) { return std::hypot(a.x - b.x, a.y - b.y); };
    const auto& leftFront = duct(400);
    const auto& rightFront = duct(401);
    const auto& leftRear = duct(500);
    const auto& rightRear = duct(501);
    const auto& balance = duct(460);
    const auto lanes = across(leftFront.centreline.back(), rightFront.centreline.back());
    const auto bore = 2.0F * rightRear.radii.front();
    const auto leftStep = across(leftFront.centreline.back(), leftRear.centreline.front());
    const auto rightStep = across(rightFront.centreline.back(), rightRear.centreline.front());
    const auto crossing = across(balance.centreline.front(), balance.centreline.back());
    std::cout << "  H-pipe: lanes " << lanes << " mm apart, rear pipes step " << leftStep << " / " << rightStep
              << " mm, balance pipe spans " << crossing << " mm\n";
    require(lanes > 2.0F * bore, "the two sides run in lanes of their own");
    require(leftStep < 0.25F * bore && rightStep < 0.25F * bore, "each side's pipe keeps its lane through the H");
    require(crossing > lanes - 2.0F * bore, "the balance pipe crosses from one side to the other");
}

/** A truck's engine brake cracks the drawn exhaust valve open around firing
    TDC on the cylinders it holds (owner's report 2026-11-09: the valves did
    not move), and leaves the others on their cam. */
void checkEngineBrakeValves() {
    EngineSpec spec;
    spec.name = "Designed truck six";
    spec.fuel = FuelType::diesel;
    spec.use = EngineUse::truck;
    spec.cylinderCount = 6;
    spec.boreMm = 131.0;
    spec.strokeMm = 158.0;
    spec.induction = Induction::turbocharged;
    const auto designed = designCatalogueEngine(spec, ENGINELAB_CATALOG_ROOT);
    require(designed.config.has_value() && designed.config->engineBrake.fitted,
            "a truck six designs with an engine brake: " + designed.error);
    const EngineModel3D model(*designed.config);
    const auto exhaustValveOffset = [&model](const ScenePoseInput& pose, std::size_t cylinder) {
        std::vector<SceneInstance> instances;
        model.pose(pose, instances);
        const auto part = model.exhaustValvePart(cylinder);
        for (const auto& instance : instances)
            if (instance.part == part) return length(instance.transform.transformPoint({}));
        return -1.0F;
    };
    // The most the brake moves each cylinder's valve over a cycle.
    std::array<float, 6> moved {};
    for (int degree = 0; degree < 720; ++degree) {
        ScenePoseInput plain;
        plain.crankAngleDegrees = degree;
        auto braking = plain;
        braking.engineBrakeCylinders = 2;
        for (std::size_t c = 0; c < 6; ++c)
            moved[c] = std::max(moved[c], exhaustValveOffset(braking, c) - exhaustValveOffset(plain, c));
    }
    std::cout << "  engine brake: exhaust valve opened " << moved[0] << " / " << moved[1] << " mm more on the two "
              << "braking cylinders, " << moved[2] << " on the third\n";
    const auto lift = static_cast<float>(designed.config->engineBrake.valveLiftMm);
    require(std::abs(moved[0] - lift) < 0.05F && std::abs(moved[1] - lift) < 0.05F,
            "a braking cylinder's drawn exhaust valve opens by the brake's lift");
    for (std::size_t c = 2; c < 6; ++c) require(moved[c] < 1.0e-4F, "the others stay on their cam");
}

/**
 * No pipe, collector, turbo or intake box is drawn through the engine or its
 * gearbox, whatever the layout and the exhaust: an X12's 12-into-1 ran its
 * primaries through the block and its collector into the gearbox (owner
 * 2026-11-18). Designed engines over the layouts and the designer's exhaust
 * systems, with and without a turbo; a pipe too short to go round is drawn
 * longer (a stretch), never through the engine.
 */
void checkEngineEnvelope() {
    struct Shape final {
        const char* name;
        EngineLayout layout;
        std::uint32_t cylinders;
        double bankAngle;
        std::vector<double> banks;
        EngineUse use;
        double bore;
        double stroke;
    };
    const std::vector<Shape> shapes {
        { "I4", EngineLayout::inlineLayout, 4, 0.0, {}, EngineUse::roadCar, 86.0, 86.0 },
        { "I6", EngineLayout::inlineLayout, 6, 0.0, {}, EngineUse::sportCar, 86.0, 86.0 },
        { "V8", EngineLayout::vLayout, 8, 90.0, {}, EngineUse::sportCar, 100.0, 92.0 },
        { "V12", EngineLayout::vLayout, 12, 60.0, {}, EngineUse::sportCar, 89.0, 80.0 },
        { "flat-6", EngineLayout::flat, 6, 0.0, {}, EngineUse::sportCar, 102.0, 76.0 },
        { "flat-12", EngineLayout::flat, 12, 0.0, {}, EngineUse::sportCar, 80.0, 74.0 },
        { "X8", EngineLayout::custom, 8, 0.0, { -135.0, -45.0, 45.0, 135.0 }, EngineUse::sportCar, 86.0, 86.0 },
        { "X12", EngineLayout::custom, 12, 0.0, { -135.0, -45.0, 45.0, 135.0 }, EngineUse::sportCar, 86.0, 86.0 },
        { "X16", EngineLayout::custom, 16, 0.0, { -135.0, -45.0, 45.0, 135.0 }, EngineUse::sportCar, 86.0, 86.0 },
        { "W8", EngineLayout::custom, 8, 0.0, { -52.5, -37.5, 37.5, 52.5 }, EngineUse::sportCar, 84.0, 90.0 },
        { "W12", EngineLayout::custom, 12, 0.0, { -60.0, 0.0, 60.0 }, EngineUse::sportCar, 84.0, 90.0 },
        { "V4 motorcycle", EngineLayout::vLayout, 4, 65.0, {}, EngineUse::sportMotorcycle, 81.0, 53.5 },
    };
    const std::array systems { ExhaustSystem::automatic, ExhaustSystem::single, ExhaustSystem::dual,
                               ExhaustSystem::dualHPipe, ExhaustSystem::dualXPipe };
    std::size_t drawn = 0, stretched = 0, failing = 0;
    for (const auto& shape : shapes)
        for (const auto system : systems)
            for (const auto induction : { Induction::natural, Induction::turbocharged }) {
                // A turbo on the one pipe and on the family's (the owner's X12-2: 12-into-1, one turbo).
                if (induction == Induction::turbocharged && system != ExhaustSystem::single
                    && system != ExhaustSystem::automatic)
                    continue;
                EngineSpec spec;
                spec.name = std::string("Designed ") + shape.name;
                spec.use = shape.use;
                spec.layout = shape.layout;
                spec.cylinderCount = shape.cylinders;
                spec.bankAngleDegrees = shape.bankAngle;
                spec.bankAnglesDegrees = shape.banks;
                spec.boreMm = shape.bore;
                spec.strokeMm = shape.stroke;
                spec.induction = induction;
                spec.exhaustSystem = system;
                const auto designed = designCatalogueEngine(spec, ENGINELAB_CATALOG_ROOT);
                // A system a layout has no use for (two pipes on one bank's motorcycle) is refused.
                if (!designed.config) continue;
                const EngineModel3D model(*designed.config);
                const auto routes = checkRoutes(model);
                ++drawn;
                stretched += routes.count(RouteIssueKind::stretched);
                const auto inside = routes.count(RouteIssueKind::engineClash);
                if (inside == 0U) continue;
                ++failing;
                std::cout << "EngineModel3D: " << shape.name << " exhaust " << static_cast<int>(system)
                          << (induction == Induction::turbocharged ? " turbo" : "") << ": " << inside
                          << " pieces through the engine\n";
                for (const auto& issue : routes.issues) {
                    if (issue.kind != RouteIssueKind::engineClash) continue;
                    std::cout << "    ";
                    if (issue.duct >= 0) {
                        const auto& duct = model.ducts()[static_cast<std::size_t>(issue.duct)];
                        std::cout << "duct kind " << static_cast<int>(duct.kind) << " path " << duct.pathId
                                  << " element " << duct.elementId;
                    } else {
                        std::cout << "a body";
                    }
                    std::cout << " in solid " << issue.other << " by " << issue.value << " mm at (" << issue.where.x
                              << ", " << issue.where.y << ", " << issue.where.z << ")\n";
                }
            }
    std::cout << "EngineModel3D: envelope: " << drawn << " designed engines, " << failing << " with a pipe through the "
              << "engine, " << stretched << " pipes drawn longer\n";
    require(drawn >= 60U, "most layouts design with most exhaust systems (" + std::to_string(drawn) + ")");
    require(failing == 0U, "no designed engine has a pipe, collector or turbo through its engine or gearbox ("
                               + std::to_string(failing) + " do)");
}

int main() {
    checkEngineBrakeValves();
    checkHPipeDrawing();
    checkSingleCylinderTurbo();
    checkTubeContact();
    checkRouter();
    const auto catalogue = loadEngineCatalog(ENGINELAB_CATALOG_ROOT);
    require(catalogue.errors.empty() && !catalogue.entries.empty(), "the shipped catalogue loads");
    for (const auto& entry : catalogue.entries) checkEngine(entry.config);
    {
        // A geometry-only exhaust with a mid pipe: the binding the catalogue's
        // authored networks never reach (audit RENDER-01).
        auto scalar = catalogue.entries.front().config;
        scalar.exhaust.midpipeLengthMm = 200.0;
        for (auto& path : scalar.exhaustPaths) {
            path.network.reset();
            path.geometry.midpipeLengthMm = 200.0;
        }
        const EngineModel3D drawn(scalar);
        const auto midpipeDrawn = std::any_of(drawn.ducts().begin(), drawn.ducts().end(), [&](const SceneDuct& duct) {
            return duct.kind == DuctKind::exhaustComponent && duct.componentType == ExhaustComponentType::pipe
                && duct.elementId == editableMidpipeId(scalar.exhaustPaths.front());
        });
        require(midpipeDrawn, scalar.name + ": a geometry-only path draws its mid pipe");
        checkGasFieldBinding(drawn);
    }
    // The compression ratio moves the head: above the crown at top dead
    // centre stands the clearance volume over the bore's area (owner
    // 2026-11-14: a new ratio changed nothing in the picture).
    for (const auto ratio : { 8.0, 12.0 }) {
        auto config = catalogue.entries.front().config;
        for (auto& cylinder : config.cylinders) cylinder.compressionRatio = ratio;
        const EngineModel3D model(config);
        const auto& cylinder = config.cylinders.front();
        double topDeadCentre = 0.0;
        float lowest = std::numeric_limits<float>::max();
        for (int step = 0; step < 7200; ++step) {
            const auto probe = model.probe(0, 0.1 * step);
            const auto gap = dot(probe.deckCentre - probe.crownCentre, probe.axis);
            if (gap < lowest) { lowest = gap; topDeadCentre = 0.1 * step; }
        }
        const auto expected = static_cast<float>(cylinder.strokeMm / (ratio - 1.0));
        std::cout << "EngineModel3D: compression " << ratio << ":1, crown to head at TDC " << lowest << " mm (flat "
                  << expected << " mm, at " << topDeadCentre << " deg)\n";
        require(std::abs(lowest - expected) < 0.02F * expected + 0.1F,
                "the head stands the clearance volume's height above the crown at top dead centre");
    }
    // A designed V8 with one pin per row: the two rods side by side on it put
    // its banks one rod width apart, not a whole throw's (owner 2026-11-14:
    // "one per row (shared)", the left bank stood off the right one). With a
    // pin per cylinder (a 90 deg V6's, split to fire evenly), a throw's width.
    for (const auto pins : { CrankPins::shared, CrankPins::separate }) {
        EngineSpec spec;
        spec.name = "Designed V";
        spec.cycle = EngineCycle::fourStroke;
        spec.use = EngineUse::roadCar;
        spec.layout = EngineLayout::vLayout;
        spec.cylinderCount = pins == CrankPins::shared ? 8U : 6U;
        spec.bankAngleDegrees = 90.0;
        spec.boreMm = 100.0;
        spec.strokeMm = 92.0;
        spec.crankPins = pins;
        const auto designed = designCatalogueEngine(spec, ENGINELAB_CATALOG_ROOT);
        require(designed.config.has_value(), "a V8 designs: " + designed.error);
        const EngineModel3D model(*designed.config);
        const auto bore = static_cast<float>(designed.config->cylinders.front().boreMm);
        float widest = 0.0F, narrowest = std::numeric_limits<float>::max();
        for (std::size_t a = 0; a < model.cylinderCount(); ++a)
            for (std::size_t b = a + 1U; b < model.cylinderCount(); ++b) {
                const auto apart = std::abs(model.probe(a, 0.0).bigEnd.z - model.probe(b, 0.0).bigEnd.z);
                // Neighbours of one row: well under a cylinder pitch apart.
                if (apart < 0.6F * bore) {
                    widest = std::max(widest, apart);
                    narrowest = std::min(narrowest, apart);
                }
            }
        const auto shared = pins == CrankPins::shared;
        std::cout << "EngineModel3D: V" << model.cylinderCount() << " " << (shared ? "shared" : "separate") << " pins, banks stand "
                  << widest / bore << " bore apart\n";
        require(shared ? widest > 0.0F && widest < 0.3F * bore && narrowest > 0.22F * bore
                       : narrowest > 0.45F * bore,
                shared ? "a V8's banks stand one rod width apart on their shared pins"
                       : "a V6's banks stand a throw apart on pins of their own");
    }
    // A designed V4 two-stroke: each chamber has its own path, two to a side,
    // and they ran through each other along their whole length, parted only
    // by their header cones' width (owner 2026-11-14). Past the ports, where
    // the headers cross, nothing touches.
    for (const auto use : { EngineUse::sportMotorcycle, EngineUse::roadMotorcycle }) {
        EngineSpec spec;
        spec.name = "Designed V4 two-stroke";
        spec.cycle = EngineCycle::twoStroke;
        spec.use = use;
        spec.layout = EngineLayout::vLayout;
        spec.cylinderCount = 4U;
        spec.bankAngleDegrees = 90.0;
        spec.boreMm = 54.0;
        spec.strokeMm = 54.0;
        const auto designed = designCatalogueEngine(spec, ENGINELAB_CATALOG_ROOT);
        require(designed.config.has_value(), "a V4 two-stroke designs: " + designed.error);
        const EngineModel3D model(*designed.config);
        const auto routes = checkRoutes(model);
        float portsEnd = -std::numeric_limits<float>::max();
        for (std::size_t c = 0; c < model.cylinderCount(); ++c)
            portsEnd = std::max(portsEnd, model.probe(c, 0.0).bigEnd.z);
        std::size_t chambers = 0;
        for (const auto& issue : routes.issues)
            if (issue.kind == RouteIssueKind::ductClash && issue.where.z > portsEnd + 2.0F * spec.boreMm) ++chambers;
        std::cout << "EngineModel3D: V4 two-stroke (" << (use == EngineUse::sportMotorcycle ? "sport" : "road")
                  << "): " << chambers << " clashes past the ports, " << routes.count(RouteIssueKind::ductClash)
                  << " in all\n";
        require(chambers == 0, "a V4 two-stroke's expansion chambers clear each other");
    }
    checkEngineEnvelope();
    checkRotaryExhausts(catalogue);
    for (const auto& entry : catalogue.entries)
        if (entry.config.cycle == EngineCycle::twoStroke) checkTwoStrokeDrawing(entry.config);
    // Both kinds of injection are checked, on a direct-injection copy of the first engine if the catalogue has none.
    if (directInjectorsDrawn == 0 || portInjectorsDrawn == 0) {
        auto other = catalogue.entries.front().config;
        other.injection.mode = portInjectorsDrawn == 0 ? InjectionMode::port : InjectionMode::direct;
        checkInjectors(EngineModel3D(other));
    }
    require(portInjectorsDrawn > 0 && directInjectorsDrawn > 0, "port and direct injectors are both checked");
    require(injectorsBehindDucts > 0, "an injector behind a runner wall was clicked (else this proves nothing)");
    // A motorcycle's gearbox stands behind the crank, clear of every barrel:
    // behind a V's front bank stands its rear one (owner's report 2026-10-24,
    // a designed V's box inside its cylinders).
    {
        std::vector<EngineConfig> motorcycles;
        for (const auto& entry : catalogue.entries)
            if (entry.config.vehicle.massKg < 500.0) motorcycles.push_back(entry.config);
        const auto design = [](EngineCycle cycle, EngineLayout layout, std::uint32_t cylinders, double bankAngle,
                               double bore, double stroke) {
            EngineSpec spec;
            spec.name = "Designed " + std::to_string(cylinders) + "-cylinder at " + std::to_string(bankAngle) + " deg";
            spec.cycle = cycle;
            spec.use = EngineUse::sportMotorcycle;
            spec.layout = layout;
            spec.cylinderCount = cylinders;
            spec.bankAngleDegrees = bankAngle;
            spec.boreMm = bore;
            spec.strokeMm = stroke;
            const auto designed = designCatalogueEngine(spec, ENGINELAB_CATALOG_ROOT);
            require(designed.config.has_value(), "a motorcycle V designs: " + designed.error);
            return *designed.config;
        };
        motorcycles.push_back(design(EngineCycle::fourStroke, EngineLayout::vLayout, 2, 90.0, 88.0, 66.0));
        motorcycles.push_back(design(EngineCycle::fourStroke, EngineLayout::vLayout, 2, 45.0, 98.0, 96.0));
        motorcycles.push_back(design(EngineCycle::fourStroke, EngineLayout::vLayout, 4, 65.0, 81.0, 53.5));
        motorcycles.push_back(design(EngineCycle::fourStroke, EngineLayout::inlineLayout, 4, 0.0, 76.0, 55.0));
        motorcycles.push_back(design(EngineCycle::twoStroke, EngineLayout::vLayout, 4, 90.0, 54.0, 54.0));
        for (const auto& motorcycle : motorcycles) {
            const EngineModel3D model(motorcycle);
            const auto clearance = gearboxBarrelClearance(model);
            std::cout << "EngineModel3D: " << motorcycle.name << " gearbox clears its barrels by " << clearance
                      << " mm\n";
            require(clearance >= 0.0F, motorcycle.name + ": the gearbox case clears every barrel ("
                                           + std::to_string(clearance) + " mm)");
        }
    }
    for (const auto& entry : catalogue.entries) checkListenerFrame(entry.config);
    for (const auto& entry : catalogue.entries) checkSoundSources(entry.config);
    require(listenersChecked >= 12, "most catalogue engines have a drawn tailpipe to listen behind ("
                                        + std::to_string(listenersChecked) + ")");
    // 2JZ, EJ25, I5 and TDI; the Merlin's supercharger is not a turbo.
    require(turbosDrawn == 4, "the four catalogue turbochargers are drawn (" + std::to_string(turbosDrawn) + ")");
    require(superchargersDrawn == 1, "the Merlin's supercharger is drawn (" + std::to_string(superchargersDrawn) + ")");
    // A turbo fitted to an engine without forced induction is drawn the same
    // way where one exhaust path takes every cylinder through a collector.
    // Measured 2026-10-13: the Aircooled's two banks get a turbine each in the
    // simulation and the view draws the first (both since 2026-10-23, with two
    // units: below); the LS3's X-pipe and the Big
    // Twin's two pipes have no collector, so the turbine sits at the outlets
    // and is not drawn.
    auto turbosFitted = 0, turbosFittedDrawn = 0;
    for (const auto& entry : catalogue.entries) {
        // A two-stroke refuses forced induction (validation; TurboFit).
        if (entry.config.forcedInduction.enabled || entry.config.cycle == EngineCycle::twoStroke) continue;
        const EngineModel3D fitted(withTurbocharger(entry.config));
        ++turbosFitted;
        if (!fitted.turbo().placed) {
            std::cout << "EngineModel3D: a fitted turbo is not drawn on " << entry.config.name << '\n';
            continue;
        }
        ++turbosFittedDrawn;
        if (entry.config.exhaustPaths.size() == 1U) checkTurbo(fitted);
    }
    require(turbosFitted >= 10 && turbosFittedDrawn >= turbosFitted - 2,
            "a fitted turbo is drawn on all but two of the engines without one (" + std::to_string(turbosFittedDrawn)
                + " of " + std::to_string(turbosFitted) + ")");
    // Two turbochargers on the Aircooled's two banks: each unit sits on its
    // own bank's collector, where the simulator puts its turbine, and both
    // are drawn and clickable. With one unit, one turbo.
    {
        const auto selected = selectSingleEngineCatalogEntry(catalogue.entries, "Aircooled");
        require(static_cast<bool>(selected), "the catalogue has the Aircooled flat six");
        const auto& flatSix = selected.entry->config;
        const EngineModel3D single(withTurbocharger(flatSix));
        const EngineModel3D twin(withTurbochargerCount(withTurbocharger(flatSix), 2U));
        const auto& turbos = twin.turbos();
        std::cout << "EngineModel3D: Aircooled with two turbochargers draws " << turbos.size() << '\n';
        require(single.turbos().size() == 1U, "one turbocharger is drawn once");
        require(turbos.size() == 2U && turbos[0].placed && turbos[1].placed
                    && turbos[0].unit != turbos[1].unit && turbos[0].pathId != turbos[1].pathId
                    && length(turbos[0].turbineCentre - turbos[1].turbineCentre) > 50.0F,
            "two turbochargers on two banks are both drawn, each on its own bank");
        auto clickable = 0;
        for (std::uint16_t part = 0; part < twin.parts().size(); ++part)
            if (const auto identity = twin.identify(part); identity.role == PartRole::turbo && identity.turbo == 1)
                ++clickable;
        require(clickable == 4, "the second turbocharger's housings and wheels identify as it");
    }
    // The Aircooled's two 3-into-1 banks joined into one 6-into-1: each
    // cylinder of the second bank gets a copy of the first's primary into the
    // first's collector, the second path goes, and the engine is drawn, bound
    // to its solver and idles. Pipes that meet no other, or a turbocharger
    // that would lose its path, are refused and change nothing.
    {
        const auto selected = selectSingleEngineCatalogEntry(catalogue.entries, "Aircooled");
        require(static_cast<bool>(selected), "the catalogue has the Aircooled flat six");
        const auto& flatSix = selected.entry->config;
        require(flatSix.exhaustPaths.size() == 2U, "the Aircooled has one exhaust path per bank");
        const auto keptId = flatSix.exhaustPaths[0].id;
        const auto joinedId = flatSix.exhaustPaths[1].id;
        auto joined = flatSix;
        const auto error = joinExhaustPaths(joined, keptId, joinedId);
        require(error.empty(), "the Aircooled's banks join: " + error);
        const auto invalid = validateEngineConfig(joined);
        require(!invalid, "the joined Aircooled is valid: " + invalid.value_or(""));
        require(joined.exhaustPaths.size() == 1U && joined.exhaustPaths.front().cylinderIds.size() == 6U
                    && joined.exhaustPaths.front().network
                    && joined.exhaustPaths.front().network->cylinderConnections.size() == 6U,
                "one path with six primaries");
        std::size_t intoJunction = 0;
        const auto& network = *joined.exhaustPaths.front().network;
        const auto& original = *flatSix.exhaustPaths.front().network;
        std::unordered_map<std::uint32_t, std::size_t> incoming;
        for (const auto& connection : network.connections) ++incoming[connection.toComponentId];
        for (const auto& [id, count] : incoming) intoJunction = std::max(intoJunction, count);
        require(intoJunction == 6U && network.components.size() > original.components.size(),
                "six pipes meet at one collector");
        const EngineModel3D drawn(joined);
        auto exhaustPieces = 0;
        for (const auto& duct : drawn.ducts())
            if (duct.kind == DuctKind::exhaustComponent || duct.kind == DuctKind::exhaustFeeder) {
                require(duct.pathId == keptId, "every drawn exhaust piece is the joined path's");
                ++exhaustPieces;
            }
        require(exhaustPieces > 0, "the joined exhaust is drawn");
        checkGasFieldBinding(drawn);
        Bench bench(joined);
        EngineState state;
        for (auto frame = 0; frame < 4 * 240; ++frame) {
            EngineControls controls;
            controls.ignitionEnabled = true;
            controls.starterEngaged = frame < 360;
            state = bench.simulator->step(1.0 / 240.0, controls).state;
        }
        std::cout << "EngineModel3D: Aircooled 6-into-1 idles at " << state.rpm << " rpm (idle "
                  << joined.idleRpm << ")\n";
        require(state.rpm > 0.5 * joined.idleRpm, "the 6-into-1 Aircooled idles");

        auto same = flatSix;
        require(!joinExhaustPaths(same, keptId, keptId).empty(), "a path is not joined to itself");
        auto twinTurbo = withTurbochargerCount(withTurbocharger(flatSix), 2U);
        const auto before = twinTurbo.exhaustPaths.size();
        require(!joinExhaustPaths(twinTurbo, keptId, joinedId).empty() && twinTurbo.exhaustPaths.size() == before,
                "two turbochargers keep their two paths");
        auto singleTurbo = withTurbocharger(flatSix);
        require(joinExhaustPaths(singleTurbo, keptId, joinedId).empty() && !validateEngineConfig(singleTurbo),
                "one turbocharger takes the joined path");
        const auto bigTwin = selectSingleEngineCatalogEntry(catalogue.entries, "Big Twin");
        require(static_cast<bool>(bigTwin) && bigTwin.entry->config.exhaustPaths.size() == 2U,
                "the catalogue has the Big Twin and its two pipes");
        auto pipes = bigTwin.entry->config;
        require(!joinExhaustPaths(pipes, pipes.exhaustPaths[0].id, pipes.exhaustPaths[1].id).empty()
                    && pipes.exhaustPaths.size() == 2U,
                "a pipe that meets no other has no collector to join into");
    }
    const auto withGears = std::count_if(catalogue.entries.begin(), catalogue.entries.end(),
        [](const auto& entry) { return entry.config.transmission.gearRatios.size() >= 2U; });
    require(gearboxesDrawn == static_cast<int>(withGears) && gearboxesDrawn >= 10,
            "every catalogue engine with gears has its gearbox drawn (" + std::to_string(gearboxesDrawn) + ")");
    {
        // A ratio the configuration accepts (down to 0.05) is drawn as itself
        // (audit RENDER-02: 0.1 was drawn 9/41).
        auto overdrive = catalogue.entries.front().config;
        overdrive.transmission.gearRatios = { 1.0, 0.1 };
        checkGearbox(EngineModel3D(overdrive));
    }
    // The catalogue compiles every exhaust path into a network at load; a
    // path authored as scalar geometry (an imported file) on a copy.
    auto scalar = catalogue.entries.front().config;
    for (auto& path : scalar.exhaustPaths) path.network.reset();
    checkExhaustResize(EngineModel3D(scalar));
    checkSoundSources(scalar);
    require(sourcesChecked == static_cast<int>(catalogue.entries.size()) + 1,
            "every catalogue engine sounds from its drawn openings");
    require(inletDuctsChecked > 0, "an inlet duct's mouth was checked (else this proves nothing)");
    require(silencersKept > 0, "a scalar silencer narrowed to its outlet is refused");
    require(lengthsGiven > 0, "a collector or an outlet with no length was given one");
    // Measured 2026-10-05: the four 140 mm pipes of the X of the LS3, which
    // must cross between the banks, the end primary of the I5, and the two
    // end primaries of the 2JZ, whose 430 mm cannot reach a collector they
    // must enter along its axis. 2026-11-14: 6, the I5's collector turned to
    // face its primaries.
    require(stretchedPipes <= 6, "no more exhaust pipes are drawn longer than authored than when measured ("
                                     + std::to_string(stretchedPipes) + ")");
    // Route check over the catalogue, measured 2026-10-05 with the router
    // (before it: 107 clashes, 51 self-clashes, 0, 259 tight bends, 6).
    // 2026-10-07: 43 tight bends once a branch leaving an X turns away at
    // once (the LS3's 1,070 mm pipes ran through each other otherwise).
    // 2026-10-15: the rest of the catalogue at 5 clashes and 42 tight bends;
    // the RD350LC-like adds 4 and 6, its two 110 mm expansion-chamber
    // bellies laid side by side at the cylinder pitch (the real bike spreads
    // them apart under the frame).
    // 2026-10-24: its barrels 1.5 bores apart for the transfers in their
    // flanks, its ports leaving from their windows: those bellies touch at 6
    // places, and it has 2 tight bends instead of 6.
    // 2026-10-29: the 13B-like adds 2 and 9: each rotor's port feeds its
    // passage (a merge drawn beside the housing, not inside it) through the
    // passage's own body, and the rear rotor's primary winds to its length.
    // 2026-10-30: the passage is the port inside the housing and the
    // collector stands back: the 13B-like at 0 clashes and 7 tight bends.
    // 2026-11-14: 4, 4, 0, 46, 6. The heads stand on their real combustion
    // chambers (2JZ +2 clashes, +1 bend); the I5's and the TDI's collectors
    // turn to face their primaries (I5 2, 4, 1 -> 0; TDI 1 clash, 1 bend
    // -> 0); the RD350's chambers part by their bellies' width (6 -> 0).
    // 2026-11-17 (audit R01-R03): 4, 4, 0, 21, 6. A plenum's face stands out
    // as far as one arc of three runner diameters leaves room for (the CP2's,
    // CP3's, CP4's and LS3's runners, 21 -> 0); a merge sits at its routed
    // inputs' mean target within the earliest's reach (the 4-2-1s' later
    // secondary no longer coils through the other: Hayabusa 21 -> 4.5 mm,
    // CP4 37 -> 14 mm deep, 4 bends fewer).
    const std::array<std::size_t, 5> measured { 4, 4, 0, 21, 6 };
    const std::array<const char*, 5> names { "duct clashes", "self-clashes", "engine clashes", "tight bends",
                                             "stretched pipes" };
    for (std::size_t k = 0; k < measured.size(); ++k)
        require(routeIssues[k] <= measured[k], std::string("no more ") + names[k] + " than when measured ("
                                                   + std::to_string(routeIssues[k]) + ")");
    checkCrankClock();
    checkGasFieldCapture(catalogue.entries.front().config);
    checkGasProbe(catalogue.entries.front().config);
    for (const auto& entry : catalogue.entries)
        checkStoppedIntakeAtRest(entry.config, entry.config.name.find("CP2") != std::string::npos
                                                   || entry.config.name.find("LS3") != std::string::npos);
    std::cout << "EngineModel3D: routes: " << routeIssues[0] << " clashes, " << routeIssues[1] << " self-clashes, "
              << routeIssues[2] << " engine clashes, " << routeIssues[3] << " tight bends, " << routeIssues[4]
              << " stretched\n";
    std::cout << "EngineModel3D: " << catalogue.entries.size() << " engines checked; " << stretchedPipes << " of "
              << checkedPipes << " exhaust pipes drawn more than 3% longer than authored\n";
    return 0;
}
