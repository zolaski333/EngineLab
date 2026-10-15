// Automatic shift schedule (ShiftSchedule, DrivelineModel::advance).
//
// The schedule is checked on its properties, not on numbers read back from
// it: the upshift point rises with the pedal and stays under the limiter,
// every upshift lands where the next frame would not change back down (no
// hunting), a kickdown lands under the upshift point. Then a dual-clutch 2JZ
// is driven through the product's coupling: flat out it changes up near the
// limiter, at a light pedal well below half of it, and it comes back down to
// first on its own when it brakes to a stop, without stalling.

#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/runtime/DriverCommandArbiter.hpp>
#include <enginelab/runtime/DrivelineModel.hpp>
#include <enginelab/runtime/ShiftSchedule.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <numbers>
#include <string>
#include <string_view>
#include <vector>

namespace {
constexpr double stepSeconds = 1.0 / 240.0;

void require(bool condition, std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

double omegaFor(double rpm, double overallRatio) {
    return rpm * 2.0 * std::numbers::pi / 60.0 / overallRatio;
}

void scheduleProperties() {
    // A 2JZ-like car: six gears, idle 760, limiter 7,000 rpm.
    const std::vector<double> gears { 3.25, 1.96, 1.42, 1.00, 0.84, 0.70 };
    std::vector<double> overall;
    for (const auto gear : gears) overall.push_back(gear * 3.27);
    enginelab::ShiftScheduleInput input;
    input.overallRatios = overall;
    input.idleRpm = 760.0;
    input.revLimitRpm = 7'000.0;

    double previousUp = 0.0;
    for (int step = 0; step <= 20; ++step) {
        input.driverThrottle = step / 20.0;
        const auto up = enginelab::ShiftSchedule::upshiftRpm(input);
        const auto down = enginelab::ShiftSchedule::downshiftRpm(input);
        require(up >= previousUp, "the upshift point must rise with the pedal");
        require(up < 7'000.0, "the upshift point must stay under the limiter");
        require(down < 0.6 * up, "the downshift point must stay well under the upshift point");
        previousUp = up;
    }
    input.driverThrottle = 1.0;
    require(enginelab::ShiftSchedule::upshiftRpm(input) > 0.9 * 7'000.0,
            "flat out, the gearbox must change up near the limiter");
    input.driverThrottle = 0.2;
    require(enginelab::ShiftSchedule::upshiftRpm(input) < 0.5 * 7'000.0,
            "at a light pedal, the gearbox must change up below half the limiter");

    // No hunting: wherever the schedule changes gear, the gear it lands in
    // must not send it back the other way at the same pedal and road speed.
    int changes = 0;
    for (int pedal = 0; pedal <= 10; ++pedal) {
        input.driverThrottle = pedal / 10.0;
        for (int gear = 0; gear < static_cast<int>(overall.size()); ++gear) {
            for (double rpm = 600.0; rpm <= 7'200.0; rpm += 50.0) {
                input.engagedGear = gear;
                input.engineRpm = rpm;
                input.wheelRadPerSecond = omegaFor(rpm, overall[static_cast<std::size_t>(gear)]);
                const auto chosen = enginelab::ShiftSchedule::chooseGear(input);
                if (chosen == gear) continue;
                ++changes;
                auto landed = input;
                landed.engagedGear = chosen;
                landed.engineRpm = rpm * overall[static_cast<std::size_t>(chosen)]
                    / overall[static_cast<std::size_t>(gear)];
                // Lifting off high in a low gear may change up again (each
                // after the caller's wait); changing back is hunting.
                const auto next = enginelab::ShiftSchedule::chooseGear(landed);
                require(chosen > gear ? next >= chosen : next <= chosen,
                        "a gear the schedule lands in must not send it back the other way (hunting)");
                if (chosen < gear)
                    require(landed.engineRpm < enginelab::ShiftSchedule::upshiftRpm(input),
                            "a downshift must land under the upshift point");
            }
        }
    }
    require(changes > 100, "the sweep must exercise the schedule");

    // Kickdown: flat out in sixth at 1,800 rpm, it drops several gears.
    input.driverThrottle = 1.0;
    input.engagedGear = 5;
    input.engineRpm = 1'800.0;
    input.wheelRadPerSecond = omegaFor(1'800.0, overall[5]);
    require(enginelab::ShiftSchedule::chooseGear(input) <= 3,
            "flat out at a low speed in top gear, the gearbox must kick down at least two gears");
    // Cruising at a light pedal in sixth at 2,000 rpm: it stays.
    input.driverThrottle = 0.15;
    input.engineRpm = 2'000.0;
    input.wheelRadPerSecond = omegaFor(2'000.0, overall[5]);
    require(enginelab::ShiftSchedule::chooseGear(input) == 5,
            "cruising at a light pedal, the gearbox must hold top gear");
}

class Rig final {
public:
    explicit Rig(const enginelab::EngineConfig& config)
        : config_(config), exhaust_(enginelab::ExhaustGraph::makeForEngine(config)),
          simulator_(config, ecu_, physics_, events_, exhaust_), driveline_(config) {}

    struct Tick final {
        enginelab::DrivelineOutput drive;
        enginelab::EngineState state;
    };

    Tick step(double throttle, double brake = 0.0, bool starter = false) {
        Tick tick;
        const auto vehicle = enginelab::advanceVehicle(driveline_, stepSeconds, simulator_.state(), 0.0,
            enginelab::DriverInputs { throttle, 1.0, brake });
        tick.drive = vehicle.drive;
        enginelab::EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = starter;
        enginelab::applyVehicleStep(controls, vehicle);
        tick.state = simulator_.step(stepSeconds, controls).state;
        return tick;
    }

    void startAndIdle() {
        driveline_.requestGear(-1);
        for (int index = 0; index < static_cast<int>(6.0 / stepSeconds); ++index)
            step(0.0, 1.0, simulator_.state().rpm < config_.idleRpm * 0.85 && index * stepSeconds < 3.0);
        driveline_.requestGear(0);
    }

private:
    enginelab::EngineConfig config_;
    enginelab::SimpleEcuModel ecu_;
    enginelab::SimplifiedGasolinePhysics physics_;
    enginelab::FourStrokeEventGenerator events_;
    enginelab::ExhaustGraph exhaust_;
    enginelab::EngineSimulator simulator_;
    enginelab::DrivelineModel driveline_;
};

struct DriveResult final {
    std::vector<double> upshiftRpm;
    int highestGear { 0 };
};

/** Pedal held from a standstill for `seconds`: the engine speed at each
 * upshift (the frame before the gear changes). */
DriveResult drive(const enginelab::EngineConfig& config, double pedal, double seconds) {
    auto rig = std::make_unique<Rig>(config);
    rig->startAndIdle();
    DriveResult result;
    int gear = 0;
    double previousRpm = 0.0;
    for (int index = 0; index < static_cast<int>(seconds / stepSeconds); ++index) {
        const auto tick = rig->step(pedal);
        const auto target = tick.drive.engagedGear;
        if (target > gear) result.upshiftRpm.push_back(previousRpm);
        gear = std::max(gear, target);
        previousRpm = tick.state.rpm;
    }
    result.highestGear = gear;
    return result;
}

void drivenTwoJz(const enginelab::EngineConfig& base) {
    auto config = base;
    config.transmission.shiftStyle = enginelab::ShiftStyle::dualClutch;
    config.transmission.automaticShifting = true;
    config.vehicle.tyreGripLimitEnabled = false;
    enginelab::normaliseEngineConfig(config);
    const auto limit = config.ignition.revLimitRpm;

    const auto flatOut = drive(config, 1.0, 22.0);
    require(flatOut.highestGear >= 3, "flat out for 22 s, the 2JZ must reach fourth");
    require(!flatOut.upshiftRpm.empty(), "flat out, the 2JZ must change up");
    for (const auto rpm : flatOut.upshiftRpm) {
        std::cout << "flat out: upshift at " << rpm << " rpm (limit " << limit << ")\n";
        require(rpm > 0.85 * limit && rpm < 1.01 * limit,
                "flat out, every upshift must happen near the limiter");
    }

    const auto light = drive(config, 0.25, 22.0);
    require(light.highestGear >= 2, "at a light pedal for 22 s, the 2JZ must reach third");
    for (const auto rpm : light.upshiftRpm) {
        std::cout << "light pedal: upshift at " << rpm << " rpm\n";
        require(rpm < 0.55 * limit, "at a light pedal, every upshift must happen well below the limiter");
    }

    // Then the brake to a stop: it comes back to first on its own and idles.
    auto rig = std::make_unique<Rig>(config);
    rig->startAndIdle();
    for (int index = 0; index < static_cast<int>(16.0 / stepSeconds); ++index) rig->step(0.6);
    Rig::Tick tick;
    int gearAtSpeed = 0;
    for (int index = 0; index < static_cast<int>(2.0 / stepSeconds); ++index) tick = rig->step(0.0);
    gearAtSpeed = tick.drive.engagedGear;
    double lowestRpm = 1.0e9;
    for (int index = 0; index < static_cast<int>(25.0 / stepSeconds); ++index) {
        tick = rig->step(0.0, 0.5);
        lowestRpm = std::min(lowestRpm, tick.state.rpm);
    }
    std::cout << "braked from gear " << gearAtSpeed + 1 << ": ended in gear " << tick.drive.engagedGear + 1
              << " at " << tick.drive.vehicleSpeedMps << " m/s, lowest " << lowestRpm << " rpm\n";
    require(gearAtSpeed >= 2, "after 16 s at 60 % pedal the 2JZ must be in third or higher");
    require(std::abs(tick.drive.vehicleSpeedMps) < 0.2, "the brake must stop the car");
    require(tick.drive.engagedGear == 0, "stopped, an automatic gearbox must be back in first");
    require(lowestRpm > 0.6 * config.idleRpm, "braking to a stop must not stall the engine");
}
} // namespace

int main() {
    scheduleProperties();
    const auto catalog = enginelab::loadEngineCatalog(std::filesystem::path(ENGINELAB_CATALOG_ROOT));
    const auto found = std::find_if(catalog.entries.begin(), catalog.entries.end(),
        [](const auto& entry) { return entry.config.name.find("2JZ") != std::string::npos; });
    require(found != catalog.entries.end(), "the catalogue must hold the 2JZ");
    drivenTwoJz(found->config);
    std::cout << "EngineLab shift schedule tests passed\n";
    return EXIT_SUCCESS;
}
