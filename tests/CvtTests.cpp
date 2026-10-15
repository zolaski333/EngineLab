// A continuously variable transmission on a toy engine whose torque is known:
// the variator holds the engine at the speed it schedules from the pedal and
// moves its ratio smoothly while the vehicle speeds up; a car's wet start
// clutch creeps and launches like a dual clutch's, a scooter's centrifugal one
// idles free and bites well above idle; coasting to a stop never stalls the
// engine; and the gear selector only knows drive, neutral and reverse.
// EngineLabDriveScenarioHarness --shift-style cvt drives a real engine.

#include <enginelab/runtime/DriverCommandArbiter.hpp>
#include <enginelab/runtime/DrivelineModel.hpp>
#include <enginelab/runtime/VariatorControl.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <numbers>
#include <string_view>

namespace {
using enginelab::DriverInputs;

void require(bool condition, std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

constexpr double stepSeconds = 1.0 / 240.0;
constexpr double radPerSecondPerRpm = 2.0 * std::numbers::pi / 60.0;

/** A 2-litre-like engine: 200 Nm at 4,500 rpm flat out, friction rising with
    speed, an idle governor and a hard limiter. */
struct ToyEngine final {
    double inertiaKgM2 { 0.2 };
    double idleRpm { 850.0 };
    double limitRpm { 7'200.0 };
    enginelab::EngineState state;

    ToyEngine() {
        state.rpm = idleRpm;
        state.angularVelocityRadPerSecond = idleRpm * radPerSecondPerRpm;
        state.simulationTimeSeconds = 1.0;
        state.activeRevLimitRpm = limitRpm;
    }

    void step(double throttle, double couplingTorqueNm, double reflectedInertiaKgM2) {
        const auto rpm = state.rpm;
        const auto shape = std::max(0.2, 1.0 - std::pow((rpm - 4'500.0) / 5'000.0, 2.0));
        const auto combustion = rpm > limitRpm ? 0.0 : 200.0 * shape * throttle;
        const auto governor = std::clamp((idleRpm - rpm) * 0.15, 0.0, 40.0);
        const auto friction = rpm > 1.0 ? 8.0 + 0.002 * rpm : 0.0;
        const auto torque = combustion + governor - friction;
        state.throttle = throttle;
        state.torqueNm = torque;
        state.cycleAveragedTorqueNm = torque;
        auto omega = state.angularVelocityRadPerSecond
            + (torque + couplingTorqueNm) / (inertiaKgM2 + reflectedInertiaKgM2) * stepSeconds;
        omega = std::max(0.0, omega);
        state.angularVelocityRadPerSecond = omega;
        state.rpm = omega / radPerSecondPerRpm;
        state.simulationTimeSeconds += stepSeconds;
    }
};

enginelab::EngineConfig cvtConfig(bool centrifugal) {
    enginelab::EngineConfig config;
    config.idleRpm = 850.0;
    config.redlineRpm = 7'200.0;
    config.ignition.revLimitRpm = 7'200.0;
    config.transmission.shiftStyle = enginelab::ShiftStyle::cvt;
    config.transmission.gearRatios = { 2.5, 0.4 };
    config.transmission.finalDriveRatio = 5.0;
    config.transmission.centrifugalClutch = centrifugal;
    config.vehicle.massKg = 1'300.0;
    return config;
}

struct Rig final {
    explicit Rig(const enginelab::EngineConfig& config) : config(config), driveline(config) {
        driveline.setEngineInertia(engine.inertiaKgM2);
    }
    enginelab::DrivelineOutput step(double pedal, double brake = 0.0) {
        const auto vehicle = enginelab::advanceVehicle(driveline, stepSeconds, engine.state, 0.0,
            DriverInputs { pedal, 1.0, brake });
        engine.step(vehicle.engineThrottle, vehicle.drive.engineCouplingTorqueNm,
            vehicle.drive.reflectedRotatingInertiaKgM2);
        return vehicle.drive;
    }
    enginelab::EngineConfig config;
    enginelab::DrivelineModel driveline;
    ToyEngine engine;
};

double targetRpm(const enginelab::EngineConfig& config, double pedal) {
    enginelab::VariatorInput input;
    input.driverThrottle = pedal;
    input.idleRpm = config.idleRpm;
    input.revLimitRpm = config.ignition.revLimitRpm;
    if (config.transmission.centrifugalClutch)
        input.centrifugalLockRpm = enginelab::VariatorControl::centrifugalLockRpm(
            config.transmission.centrifugalClutchEngageRpm, config.idleRpm);
    return enginelab::VariatorControl::targetEngineRpm(input);
}

void checkPureFunctions() {
    enginelab::VariatorInput input;
    input.idleRpm = 850.0;
    input.revLimitRpm = 7'200.0;
    input.driverThrottle = 0.0;
    const auto light = enginelab::VariatorControl::targetEngineRpm(input);
    input.driverThrottle = 1.0;
    const auto full = enginelab::VariatorControl::targetEngineRpm(input);
    require(light >= 1.5 * 850.0 && light < 2'000.0, "foot-light the variator holds the engine just off idle");
    require(full > 0.8 * 7'200.0 && full < 7'200.0, "flat out it holds the engine near peak power, under the limit");

    input.coupled = true;
    input.currentRatio = 2.5;
    input.outputRadPerSecond = 10.0;
    const auto next = enginelab::VariatorControl::nextRatio(input, stepSeconds);
    const auto maximumStep = std::log(2.5 / 0.4) / 1.5 * stepSeconds;
    require(std::abs(std::log(next / 2.5)) <= maximumStep + 1.0e-12, "the ratio moves at a bounded rate");
    input.coupled = false;
    input.currentRatio = 0.4;
    for (int index = 0; index < static_cast<int>(2.0 / stepSeconds); ++index)
        input.currentRatio = enginelab::VariatorControl::nextRatio(input, stepSeconds);
    require(std::abs(input.currentRatio - 2.5) < 1.0e-9, "uncoupled, it returns to its lowest ratio");

    const auto engage = enginelab::VariatorControl::centrifugalEngageRpm(0.0, 1'700.0);
    require(std::abs(engage - 2'890.0) < 1.0, "a centrifugal clutch bites at 1.7 x idle by default");
    require(enginelab::VariatorControl::centrifugalClutchPressure(1'700.0, 0.0, 1'700.0) == 0.0,
        "a centrifugal clutch is open at idle");
    require(enginelab::VariatorControl::centrifugalClutchPressure(1.35 * engage, 0.0, 1'700.0) == 1.0,
        "and fully shut at its lock speed");
}

/** Launch at a steady pedal and drive until the vehicle passes `speedKmh`. */
void checkLaunchAndHold(bool centrifugal, double pedal, double speedKmh) {
    Rig rig(cvtConfig(centrifugal));
    rig.driveline.requestGear(0);
    for (int index = 0; index < static_cast<int>(1.0 / stepSeconds); ++index) rig.step(0.0, 1.0);
    const auto target = targetRpm(rig.config, pedal);
    double heldSeconds = 0.0;
    double worstHeldError = 0.0;
    double largestRpmStep = 0.0;
    double lowestRpm = rig.engine.state.rpm;
    double previousRpm = rig.engine.state.rpm;
    double ratioAtStart = 0.0;
    double ratio = 0.0;
    auto seconds = 0.0;
    enginelab::DrivelineOutput drive;
    while (drive.vehicleSpeedMps * 3.6 < speedKmh && seconds < 60.0) {
        drive = rig.step(pedal);
        seconds += stepSeconds;
        const auto rpm = rig.engine.state.rpm;
        lowestRpm = std::min(lowestRpm, rpm);
        if (ratioAtStart == 0.0) ratioAtStart = drive.gearboxRatio;
        ratio = drive.gearboxRatio;
        // Once the clutch holds and the variator has left its lowest ratio,
        // the engine sits at the scheduled speed and moves smoothly.
        if (drive.clutchStickFraction > 0.99 && ratio < 0.98 * 2.5) {
            heldSeconds += stepSeconds;
            if (heldSeconds > 1.0) {
                worstHeldError = std::max(worstHeldError, std::abs(rpm - target) / target);
                largestRpmStep = std::max(largestRpmStep, std::abs(rpm - previousRpm));
            }
        }
        previousRpm = rpm;
    }
    std::printf("  %-11s pedal %3.0f %%: 0-%3.0f km/h in %5.2f s, held %5.2f s at %5.0f rpm (worst %4.1f %%,"
                " largest step %4.1f rpm/frame), ratio %4.2f -> %4.2f, lowest %4.0f rpm\n",
        centrifugal ? "centrifugal" : "wet clutch", pedal * 100.0, speedKmh, seconds, heldSeconds, target,
        worstHeldError * 100.0, largestRpmStep, ratioAtStart, ratio, lowestRpm);
    require(drive.vehicleSpeedMps * 3.6 >= speedKmh, "the vehicle reaches the speed");
    require(heldSeconds > 2.0, "the variator holds the engine at its scheduled speed while the ratio climbs");
    require(worstHeldError < 0.05, "within 5 % of the scheduled speed");
    require(largestRpmStep < 20.0, "and moves smoothly: no clutch chatter from the moving ratio");
    require(lowestRpm > 0.75 * rig.config.idleRpm, "the launch never stalls the engine");
    require(ratio < ratioAtStart, "the ratio has moved toward its tallest");
}

void checkCreepAndIdle() {
    Rig wet(cvtConfig(false));
    wet.driveline.requestGear(0);
    for (int index = 0; index < static_cast<int>(6.0 / stepSeconds); ++index) wet.step(0.0);
    const auto creep = wet.driveline.advance(0.0, wet.engine.state, 0.0, 1.0, 0.0).vehicleSpeedMps;
    require(creep > 0.3, "a car's CVT creeps at idle, foot off");

    Rig scooter(cvtConfig(true));
    scooter.driveline.requestGear(0);
    double moved = 0.0;
    for (int index = 0; index < static_cast<int>(6.0 / stepSeconds); ++index)
        moved = std::max(moved, std::abs(scooter.step(0.0).vehicleSpeedMps));
    require(moved < 0.01, "a scooter's centrifugal clutch idles free");
    require(std::abs(scooter.engine.state.rpm - scooter.config.idleRpm) < 100.0, "and the engine idles");
}

void checkCoastToStop(bool centrifugal) {
    Rig rig(cvtConfig(centrifugal));
    rig.driveline.requestGear(0);
    enginelab::DrivelineOutput drive;
    for (int index = 0; index < static_cast<int>(12.0 / stepSeconds); ++index) drive = rig.step(0.6);
    require(drive.vehicleSpeedMps > 15.0, "the vehicle got going");
    double lowestRpm = rig.engine.state.rpm;
    for (int index = 0; index < static_cast<int>(20.0 / stepSeconds); ++index) {
        drive = rig.step(0.0, 0.6);
        lowestRpm = std::min(lowestRpm, rig.engine.state.rpm);
    }
    std::printf("  %-11s braked to a stop: lowest %4.0f rpm, ratio %4.2f\n",
        centrifugal ? "centrifugal" : "wet clutch", lowestRpm, drive.gearboxRatio);
    require(std::abs(drive.vehicleSpeedMps) < 0.05, "the vehicle stops");
    require(lowestRpm > 0.7 * rig.config.idleRpm, "braking to a stop never stalls the engine");
    require(drive.gearboxRatio > 0.98 * 2.5, "stopped, the variator is back at its lowest ratio");
}

void checkSelector() {
    Rig rig(cvtConfig(false));
    rig.driveline.requestGear(0);
    rig.driveline.shiftUp();
    require(rig.driveline.requestedGear() == 0, "a CVT has one forward position: shifting up stays in drive");
    rig.driveline.requestGear(4);
    require(rig.driveline.requestedGear() == 0, "a gear number past drive selects drive");
    rig.driveline.shiftDown();
    require(rig.driveline.requestedGear() == -1, "shifting down from drive selects neutral");
    require(!rig.driveline.driverClutchConnected(), "the clutch pedal does not reach a CVT's start clutch");
}
} // namespace

int main() {
    checkPureFunctions();
    checkSelector();
    std::printf("launches\n");
    checkLaunchAndHold(false, 0.3, 55.0);
    checkLaunchAndHold(false, 1.0, 120.0);
    checkLaunchAndHold(true, 0.3, 50.0);
    checkLaunchAndHold(true, 1.0, 100.0);
    checkCreepAndIdle();
    std::printf("stops\n");
    checkCoastToStop(false);
    checkCoastToStop(true);
    std::cout << "CVT tests passed\n";
    return EXIT_SUCCESS;
}
