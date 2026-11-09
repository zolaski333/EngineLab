// A truck's compression-release engine brake (EngineBrakeConfig).
//
// - Only a truck design gets one (and its 12-speed automated gearbox and its
//   40 t tractor); the same engine designed for a road car has none.
// - Motored at 2,100 rpm, foot off the pedal, the brake on every cylinder
//   raises the drag on the shaft by about what production brakes give: a
//   standard brake 416 kW on a 13 L six at 2,100 rpm (Jacobs, against its
//   high-power one's 611 kW at 2,500), about 32 kW/L. The lift and timing are
//   estimates within published ranges (1-2 mm near TDC), so the check is
//   broad: 0.5-1.5 x that.
// - The stages brake 2, 4 and 6 cylinders; their fuel is cut.
// - It lets go with the pedal pressed, below its minimum speed, with the
//   clutch out or a shift under way, and on an engine without one.

#include <enginelab/catalog/EngineDesign.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/runtime/DriverCommandArbiter.hpp>
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

void require(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

EngineSpec truckSix(EngineUse use) {
    EngineSpec spec;
    spec.name = "Designed truck six";
    spec.fuel = FuelType::diesel;
    spec.use = use;
    spec.cylinderCount = 6;
    // A 13 L six's bore and stroke (a Volvo D13's 131 x 158).
    spec.boreMm = 131.0;
    spec.strokeMm = 158.0;
    spec.induction = Induction::turbocharged;
    return spec;
}

struct Motored final {
    double driveTorqueNm {};
    double rpm {};
    double fuelGramsPerSecond {};
    std::uint32_t brakingCylinders {};
};

/** The engine motored at `rpm` through its shaft, pedal at `throttle`, the
 * engine brake's switch at `stage`: the drive torque that holds it, averaged
 * over 1 s after 5 s. The drive acts once a frame on the frame's speed, well
 * under half the shaft's inertia a frame (CLAUDE.md, the absorber trap). */
Motored motor(const EngineConfig& config, double rpm, std::uint32_t stage, double throttle = 0.0) {
    SimpleEcuModel ecu;
    SimplifiedGasolinePhysics physics;
    FourStrokeEventGenerator events;
    ExhaustGraph exhaust = ExhaustGraph::makeForEngine(config);
    auto simulator = std::make_unique<EngineSimulator>(config, ecu, physics, events, exhaust);
    constexpr double dt = 1.0 / 240.0;
    const auto target = rpm * 2.0 * std::numbers::pi / 60.0;
    const auto inertia = effectiveRotatingInertiaKgM2(config);
    // N m per rad/s, and per rad: a 0.5 s integral.
    const auto proportional = 0.15 * inertia / dt;
    const auto integralGain = 2.0 * proportional;
    double integral = 0.0;
    Motored result;
    const auto settle = static_cast<int>(5.0 / dt), sample = static_cast<int>(1.0 / dt);
    for (int step = 0; step < settle + sample; ++step) {
        // Spun up from rest, then held; the integral runs once near speed.
        const auto error = target - simulator->state().angularVelocityRadPerSecond;
        if (std::abs(error) < 20.0) integral += error * dt;
        EngineControls controls;
        controls.ignitionEnabled = true;
        controls.throttle = throttle;
        controls.engineBrakeStage = stage;
        controls.externalTorqueNm = std::max(0.0, proportional * error + integralGain * integral);
        const auto frame = simulator->step(dt, controls);
        if (step < settle) continue;
        result.driveTorqueNm += controls.externalTorqueNm / sample;
        result.rpm += frame.state.rpm / sample;
        result.fuelGramsPerSecond += frame.state.fuelFlowGramsPerSecond / sample;
        result.brakingCylinders = std::max(result.brakingCylinders, frame.state.engineBrakeCylinders);
    }
    return result;
}
} // namespace

int main() {
    const auto designed = designCatalogueEngine(truckSix(EngineUse::truck), ENGINELAB_CATALOG_ROOT);
    require(designed.config.has_value(), "a truck six designs: " + designed.error);
    const auto& truck = *designed.config;
    require(truck.engineBrake.fitted && truck.engineBrake.stageCount == 3, "a truck gets a three-stage engine brake");
    require(truck.transmission.shiftStyle == ShiftStyle::automatedManual
                && truck.transmission.gearRatios.size() == 12,
            "a truck gets its 12-speed automated gearbox");
    require(truck.vehicle.massKg > 30'000.0, "a truck pulls a 40 t trailer");
    const auto road = designCatalogueEngine(truckSix(EngineUse::roadCar), ENGINELAB_CATALOG_ROOT);
    require(road.config.has_value(), "the same six designs for a road car: " + road.error);
    require(!road.config->engineBrake.fitted, "only a truck gets an engine brake");

    const auto litres = engineDisplacementLitres(truck);
    const auto off = motor(truck, 2'100.0, 0);
    const auto on = motor(truck, 2'100.0, 3);
    const auto brakeKw = (on.driveTorqueNm - off.driveTorqueNm) * 2'100.0 * 2.0 * std::numbers::pi / 60.0 * 1.0e-3;
    std::cout << "EngineBrake: " << litres << " L six at " << on.rpm << " rpm: drag " << off.driveTorqueNm
              << " N m off, " << on.driveTorqueNm << " N m on: brake " << brakeKw << " kW ("
              << brakeKw / litres << " kW/L; a production brake about 32), " << on.brakingCylinders << " cylinders\n";
    require(std::abs(on.rpm - 2'100.0) < 50.0 && std::abs(off.rpm - 2'100.0) < 50.0, "the shaft is held at 2,100 rpm");
    require(on.brakingCylinders == 6, "the third stage brakes every cylinder");
    // The fuel cut, where the engine takes fuel without the brake: a pedal
    // just under the brake's switch point.
    const auto pedal = 0.5 * engineBrakeMaximumThrottle;
    const auto fuelledOff = motor(truck, 1'400.0, 0, pedal);
    const auto fuelledOn = motor(truck, 1'400.0, 3, pedal);
    std::cout << "EngineBrake: fuel at 1,400 rpm, pedal " << pedal << ": " << fuelledOff.fuelGramsPerSecond
              << " g/s off, " << fuelledOn.fuelGramsPerSecond << " g/s on\n";
    require(fuelledOff.fuelGramsPerSecond > 0.05, "the engine takes fuel there without the brake");
    require(fuelledOn.fuelGramsPerSecond < 0.02 * fuelledOff.fuelGramsPerSecond, "a braking cylinder takes no fuel");
    require(brakeKw / litres > 0.5 * 32.0 && brakeKw / litres < 1.5 * 32.0,
            "the brake takes about a production brake's power (" + std::to_string(brakeKw / litres) + " kW/L)");

    const auto first = motor(truck, 2'100.0, 1);
    require(first.brakingCylinders == 2, "the first stage brakes two cylinders of six");
    require(first.driveTorqueNm > off.driveTorqueNm && first.driveTorqueNm < on.driveTorqueNm,
            "the first stage brakes less than the third");
    require(motor(truck, 2'100.0, 3, 0.3).brakingCylinders == 0, "the pedal releases it");
    require(motor(truck, 0.8 * truck.engineBrake.minimumRpm, 3).brakingCylinders == 0,
            "it lets go below its minimum speed");
    require(motor(*road.config, 2'100.0, 3).brakingCylinders == 0, "an engine without one ignores the switch");

    // The runtime's clutch switch and the shift's release.
    DriverInputs driver;
    driver.engineBrakeStage = 2;
    VehicleStep step;
    step.drive.clutchPressure = 1.0;
    DriverCommandArbiter::arbitrate(driver, step);
    require(step.engineBrakeStage == 2, "the switch passes with the clutch in");
    step.drive.clutchPressure = 0.3;
    DriverCommandArbiter::arbitrate(driver, step);
    require(step.engineBrakeStage == 0, "the clutch out releases it");
    step.drive.clutchPressure = 1.0;
    step.drive.shiftInProgress = true;
    DriverCommandArbiter::arbitrate(driver, step);
    require(step.engineBrakeStage == 0, "a shift under way releases it");
    return EXIT_SUCCESS;
}
