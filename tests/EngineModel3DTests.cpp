#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/exhaust/LegacyExhaustNetwork.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/render/CrankClock.hpp>
#include <enginelab/render/EngineModel3D.hpp>
#include <enginelab/render/GasFieldView.hpp>
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
    checkTubeContact();
    checkRouter();
    const auto catalogue = loadEngineCatalog(ENGINELAB_CATALOG_ROOT);
    require(catalogue.errors.empty() && !catalogue.entries.empty(), "the shipped catalogue loads");
    for (const auto& entry : catalogue.entries) checkEngine(entry.config);
    // 2JZ, EJ25, I5 and TDI; the Merlin's supercharger is not a turbo.
    require(turbosDrawn == 4, "the four catalogue turbochargers are drawn (" + std::to_string(turbosDrawn) + ")");
    // Measured 2026-10-05: the four 140 mm pipes of the X of the LS3, which
    // must cross between the banks, the end primary of the I5, and the two
    // end primaries of the 2JZ, whose 430 mm cannot reach a collector they
    // must enter along its axis.
    require(stretchedPipes <= 7, "no more exhaust pipes are drawn longer than authored than when measured ("
                                     + std::to_string(stretchedPipes) + ")");
    // Route check over the catalogue, measured 2026-10-05 with the router
    // (before it: 107 clashes, 51 self-clashes, 0, 259 tight bends, 6).
    const std::array<std::size_t, 5> measured { 7, 4, 0, 48, 7 };
    const std::array<const char*, 5> names { "duct clashes", "self-clashes", "engine clashes", "tight bends",
                                             "stretched pipes" };
    for (std::size_t k = 0; k < measured.size(); ++k)
        require(routeIssues[k] <= measured[k], std::string("no more ") + names[k] + " than when measured ("
                                                   + std::to_string(routeIssues[k]) + ")");
    checkCrankClock();
    checkGasFieldCapture(catalogue.entries.front().config);
    checkGasProbe(catalogue.entries.front().config);
    std::cout << "EngineModel3D: routes: " << routeIssues[0] << " clashes, " << routeIssues[1] << " self-clashes, "
              << routeIssues[2] << " engine clashes, " << routeIssues[3] << " tight bends, " << routeIssues[4]
              << " stretched\n";
    std::cout << "EngineModel3D: " << catalogue.entries.size() << " engines checked; " << stretchedPipes << " of "
              << checkedPipes << " exhaust pipes drawn more than 3% longer than authored\n";
    return 0;
}
