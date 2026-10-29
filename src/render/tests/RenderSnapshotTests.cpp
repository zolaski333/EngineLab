#include <enginelab/render/DuctLayout3D.hpp>
#include <enginelab/render/RenderSnapshot.hpp>

#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <numbers>

namespace {
void require(bool condition, const char* message) {
    if (condition) return;
    std::cerr << "RenderSnapshot test failure: " << message << '\n';
    std::exit(1);
}
}

int main() {
    auto config = enginelab::makeDefaultV8();
    enginelab::normaliseEngineConfig(config);
    enginelab::EngineState state;
    state.simulationTimeSeconds = 1.0;
    state.crankAngleDegrees = 359.0;
    state.rpm = 2'400.0;
    state.cylinderStateCount = config.cylinders.size();
    for (std::size_t index = 0; index < state.cylinderStateCount; ++index) {
        auto& cylinder = state.cylinderStates[index];
        cylinder.id = config.cylinders[index].id;
        cylinder.crankshaftId = 1;
        cylinder.crankJournalId = config.cylinders[index].crankJournalId;
        cylinder.crankPinXMm = static_cast<double>(index) * 2.0;
        cylinder.crankPinYMm = 35.0;
        cylinder.wristPinXMm = static_cast<double>(index) * 2.0 + 12.0;
        cylinder.wristPinYMm = -105.0;
        cylinder.pistonTravelMm = 20.0;
        cylinder.gasTemperatureC = 420.0;
        cylinder.exhaustTemperatureC = 640.0;
    }

    enginelab::RenderSnapshotBuilder builder(config);
    const auto first = builder.build(state);
    require(first.partCount == 1U + config.cylinders.size() * 6U,
            "builder must emit crankshaft plus six stable parts per cylinder");
    require(std::isfinite(first.boundsMinimumMm.x) && std::isfinite(first.boundsMaximumMm.z),
            "scene bounds must be finite");

    // The first cylinder of each V bank occupies the same longitudinal station.
    const auto firstCylinderZ = first.parts[1].transform.positionMm.z;
    const auto opposingCylinderZ = first.parts[7].transform.positionMm.z;
    require(std::abs(firstCylinderZ - opposingCylinderZ) < 1.0e-9,
            "opposing V-bank cylinders must share their longitudinal station");

    auto secondState = state;
    secondState.simulationTimeSeconds = 2.0;
    secondState.crankAngleDegrees = 1.0;
    const auto bankRadians = config.banks[0].angleDegrees * std::numbers::pi / 180.0;
    const auto axisX = std::sin(bankRadians);
    const auto axisY = -std::cos(bankRadians);
    secondState.cylinderStates[0].wristPinXMm -= axisX * 10.0;
    secondState.cylinderStates[0].wristPinYMm -= axisY * 10.0;
    secondState.cylinderStates[0].pistonTravelMm += 10.0;
    const auto second = builder.build(secondState);
    require(std::abs(first.parts[1].transform.positionMm.x
        - second.parts[1].transform.positionMm.x) < 1.0e-9
        && std::abs(first.parts[1].transform.positionMm.y
            - second.parts[1].transform.positionMm.y) < 1.0e-9,
        "cylinder block must remain fixed when the piston moves");
    require(std::abs(first.parts[5].transform.positionMm.x
        - second.parts[5].transform.positionMm.x) < 1.0e-9
        && std::abs(first.parts[5].transform.positionMm.y
            - second.parts[5].transform.positionMm.y) < 1.0e-9,
        "closed valve position must remain fixed when the piston moves");
    enginelab::RenderSnapshotInterpolator interpolation;
    interpolation.push(first);
    interpolation.push(second);
    const auto middle = interpolation.sample(1.5);
    require(std::abs(std::remainder(middle.crankAngleDegrees, 360.0)) < 1.0e-9,
            "crank interpolation must take the short path across 360 degrees");
    require(std::abs(middle.parts[4].transform.positionMm.x
        - 0.5 * (first.parts[4].transform.positionMm.x
            + second.parts[4].transform.positionMm.x)) < 1.0e-9,
        "moving parts must interpolate linearly");

    interpolation.reset();
    require(!interpolation.ready(), "reset must clear interpolation history");

    // The throttle's mouth is drawn as the intake is built: bare, a pod filter
    // on it, or a velocity stack.
    const auto throttleVertices = [](bool pod, double bellmouthMm) {
        auto engine = enginelab::makeDefaultInlineFour();
        engine.intake.airboxVolumeLitres = 0.0;
        engine.intake.inletDuctLengthMm = 0.0;
        engine.intake.podFilter = pod;
        engine.intake.bellmouthDiameterMm = bellmouthMm;
        for (auto& path : engine.intakePaths) path.geometry = engine.intake;
        enginelab::normaliseEngineConfig(engine);
        std::vector<enginelab::render::PortAnchor> ports;
        for (std::size_t index = 0; index < engine.cylinders.size(); ++index) {
            enginelab::render::PortAnchor port;
            port.cylinderId = engine.cylinders[index].id;
            port.position = { static_cast<float>(index) * 90.0F, 150.0F, 40.0F };
            port.direction = { 0.0F, 0.3F, 1.0F };
            port.inner = { static_cast<float>(index) * 90.0F, 120.0F, 20.0F };
            port.diameterMm = 32.0F;
            ports.push_back(port);
        }
        std::size_t vertices = 0;
        for (const auto& piece : enginelab::render::layoutIntake(engine, ports, 86.0F, false))
            if (piece.kind == enginelab::render::DuctKind::intakeThrottle) vertices += piece.mesh.vertices.size();
        return vertices;
    };
    const auto bare = throttleVertices(false, 0.0);
    require(throttleVertices(true, 0.0) > bare, "a pod filter is drawn on the throttle's mouth");
    require(throttleVertices(false, 2.0 * enginelab::IntakeConfig {}.throttleDiameterMm) > bare, "an open trumpet is drawn on the mouth");
    return 0;
}
