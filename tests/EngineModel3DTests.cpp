#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/render/CrankClock.hpp>
#include <enginelab/render/EngineModel3D.hpp>

#include <cmath>
#include <cstdlib>
#include <iostream>
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
    checkCrankClock();
    std::cout << "EngineModel3D: " << catalogue.entries.size() << " engines checked\n";
    return 0;
}
