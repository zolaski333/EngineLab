// A designed truck driven away flat out through its automated manual gearbox,
// shifting by itself (owner's reports 2026-11-09: the gearbox closed its
// clutch with the throttle still open and hit the limiter shift after shift;
// it changed up far too early).
//
// - Each upshift is made with the engine's torque taken away while the
//   clutch is open: the engine never reaches its limiter during a shift, and
//   comes down towards the new gear's speed instead of flaring.
// - Flat out it changes up near the engine's peak power, not at its torque
//   plateau's start.

#include <enginelab/catalog/EngineDesign.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/runtime/DriverCommandArbiter.hpp>
#include <enginelab/runtime/DrivelineModel.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace {
using namespace enginelab;
constexpr double stepSeconds = 1.0 / 240.0;

void require(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

struct Shift final {
    int from {};
    int to {};
    double startRpm {};
    double peakRpm {};
    double landedRpm {};
    double limiterSeconds {};
    double openRpm { -1.0 };
    double openPeakRpm {};
};

struct Drive final {
    std::vector<Shift> shifts;
    double limiterSecondsOutsideShifts {};
    double finalSpeedKmh {};
};

/** Starts the engine in neutral, selects first and holds `throttle` for
 * `seconds`, the gearbox shifting by itself. */
Drive driveAway(const EngineConfig& config, double throttle, double seconds) {
    SimpleEcuModel ecu;
    SimplifiedGasolinePhysics physics;
    FourStrokeEventGenerator events;
    ExhaustGraph exhaust = ExhaustGraph::makeForEngine(config);
    auto simulator = std::make_unique<EngineSimulator>(config, ecu, physics, events, exhaust);
    DrivelineModel driveline(config);
    const auto run = [&](double pedal, bool starter) {
        const auto vehicle = advanceVehicle(driveline, stepSeconds, simulator->state(), 0.0,
                                            DriverInputs { pedal, 1.0, 0.0 });
        EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = starter;
        applyVehicleStep(controls, vehicle);
        return std::pair { vehicle, simulator->step(stepSeconds, controls).state };
    };
    driveline.requestGear(-1);
    for (int index = 0; index < static_cast<int>(6.0 / stepSeconds); ++index)
        run(0.0, simulator->state().rpm < 0.85 * config.idleRpm && index * stepSeconds < 3.0);
    require(simulator->state().rpm > 0.8 * config.idleRpm, config.name + " starts and idles");
    driveline.requestGear(0);
    Drive drive;
    std::optional<Shift> current;
    auto lastGear = -1;
    for (int index = 0; index < static_cast<int>(seconds / stepSeconds); ++index) {
        const auto before = simulator->state().rpm;
        const auto [vehicle, state] = run(throttle, false);
        const auto limiting = (state.activeLimits & static_cast<std::uint32_t>(ActiveLimit::revLimiter)) != 0U;
        if (vehicle.drive.shiftInProgress) {
            if (!current) current = Shift { lastGear, -1, before, before, 0.0, 0.0 };
            current->peakRpm = std::max(current->peakRpm, state.rpm);
            // With the clutch open the engine is free: what it gains there is
            // the flare, not the vehicle's acceleration.
            if (vehicle.drive.clutchPressure < 0.02) {
                if (current->openRpm < 0.0) current->openRpm = state.rpm;
                current->openPeakRpm = std::max(current->openPeakRpm, state.rpm);
            }
            if (limiting) current->limiterSeconds += stepSeconds;
        } else {
            if (current) {
                current->to = vehicle.drive.engagedGear;
                current->landedRpm = state.rpm;
                if (current->from >= 0 && current->to > current->from) drive.shifts.push_back(*current);
                current.reset();
            }
            if (limiting) drive.limiterSecondsOutsideShifts += stepSeconds;
            lastGear = vehicle.drive.engagedGear;
        }
        drive.finalSpeedKmh = vehicle.drive.vehicleSpeedMps * 3.6;
    }
    return drive;
}

EngineSpec truckSix() {
    EngineSpec spec;
    spec.name = "Designed truck six";
    spec.fuel = FuelType::diesel;
    spec.use = EngineUse::truck;
    spec.cylinderCount = 6;
    spec.boreMm = 131.0;
    spec.strokeMm = 158.0;
    spec.induction = Induction::turbocharged;
    return spec;
}
} // namespace

int main() {
    const auto designed = designCatalogueEngine(truckSix(), ENGINELAB_CATALOG_ROOT);
    require(designed.config.has_value(), "a truck six designs: " + designed.error);
    const auto& truck = *designed.config;
    require(truck.transmission.shiftStyle == ShiftStyle::automatedManual && truck.transmission.automaticShifting,
            "a truck shifts its automated manual by itself");
    for (const auto throttle : { 1.0, 0.5 }) {
        const auto drive = driveAway(truck, throttle, 40.0);
        std::cout << "TruckDrive: pedal " << throttle << ", " << drive.shifts.size() << " upshifts to "
                  << drive.finalSpeedKmh << " km/h in 40 s; limiter outside shifts " << drive.limiterSecondsOutsideShifts
                  << " s\n";
        for (const auto& shift : drive.shifts)
            std::cout << "  " << shift.from + 1 << "->" << shift.to + 1 << ": from " << shift.startRpm << " rpm, peak "
                      << shift.peakRpm << ", landed " << shift.landedRpm << ", limiter " << shift.limiterSeconds
                      << " s; clutch open from " << shift.openRpm << " rpm, peak there " << shift.openPeakRpm << '\n';
        require(drive.shifts.size() >= 3, "the truck changes up as it gathers speed");
        const auto limit = truck.ignition.revLimitRpm;
        for (const auto& shift : drive.shifts) {
            const auto label = "pedal " + std::to_string(throttle) + ", " + std::to_string(shift.from + 1) + " to "
                + std::to_string(shift.to + 1) + ": ";
            require(shift.limiterSeconds == 0.0, label + "the engine never reaches its limiter during a shift");
            require(shift.openRpm > 0.0, label + "the clutch opens for the shift");
            // Before the change it flared 360-400 rpm here (0.18 x limit).
            require(shift.openPeakRpm < shift.openRpm + 0.02 * limit,
                    label + "the engine does not flare while the clutch is open");
            require(shift.peakRpm < 0.95 * limit, label + "the engine stays clear of its limiter");
            // Where the clutch opens is where it changes: the controller starts
            // early enough to get there (it was 0.73 x limit before).
            if (throttle == 1.0)
                require(shift.openRpm > 0.75 * limit, label + "flat out it changes up near its peak power (redline "
                        + std::to_string(limit) + ", the clutch opened at " + std::to_string(shift.openRpm) + ")");
        }
    }
    return EXIT_SUCCESS;
}
