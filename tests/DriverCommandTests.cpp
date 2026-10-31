// The driver-command arbiter: the one place where the driver's pedals and the
// driveline's own devices (a clutch shift's torque cut, a quickshifter's
// ignition cut, an auto-blipper) become engine commands. EngineRuntime, the
// shift-style tests and the harnesses all step the vehicle through it.

#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/runtime/DriverCommandArbiter.hpp>
#include <enginelab/runtime/DrivelineModel.hpp>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>
#include <limits>
#include <string>
#include <string_view>

namespace {
using enginelab::ThrottleAuthority;

void require(bool condition, std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

enginelab::VehicleStep arbitrated(double driver, double torqueCut, double blip, bool ignitionCut = false) {
    enginelab::VehicleStep step;
    step.drive.torqueCutMultiplier = torqueCut;
    step.drive.blipThrottle = blip;
    step.drive.ignitionCut = ignitionCut;
    enginelab::DriverCommandArbiter::arbitrate(enginelab::DriverInputs { driver, 1.0, 0.0 }, step);
    return step;
}

void checkThrottleRules() {
    const auto alone = arbitrated(0.6, 1.0, 0.0);
    require(alone.engineThrottle == 0.6 && alone.throttleAuthority == ThrottleAuthority::driver,
        "without a device the engine gets the driver's throttle");

    const auto cut = arbitrated(0.6, 0.25, 0.0);
    require(cut.engineThrottle == 0.6 * 0.25 && cut.throttleAuthority == ThrottleAuthority::shiftTorqueCut,
        "a clutch shift lifts the driver's throttle by its torque cut");

    const auto blipped = arbitrated(0.2, 1.0, 0.5);
    require(blipped.engineThrottle == 0.5 && blipped.throttleAuthority == ThrottleAuthority::blipper,
        "an auto-blipper opens the throttle past the driver's");

    const auto underFoot = arbitrated(0.6, 1.0, 0.1);
    require(underFoot.engineThrottle == 0.6 && underFoot.throttleAuthority == ThrottleAuthority::driver,
        "an auto-blipper never closes the driver's throttle");

    const auto closed = arbitrated(0.0, 0.0, 0.0);
    require(closed.engineThrottle == 0.0 && closed.throttleAuthority == ThrottleAuthority::driver,
        "a closed pedal under a full cut is still the driver's");

    require(arbitrated(std::numeric_limits<double>::quiet_NaN(), 1.0, 0.0).engineThrottle == 0.0,
        "a non-finite pedal closes the throttle");
    require(arbitrated(1.7, 1.0, 0.0).engineThrottle == 1.0 && arbitrated(-0.3, 1.0, 0.0).engineThrottle == 0.0,
        "the pedal is clamped to [0, 1]");

    require(arbitrated(0.6, 1.0, 0.0, true).shiftIgnitionCut && !arbitrated(0.6, 1.0, 0.0).shiftIgnitionCut,
        "a quickshifter's ignition cut passes through to the engine");
}

void checkControls() {
    auto step = arbitrated(0.4, 1.0, 0.0, true);
    step.drive.engineCouplingTorqueNm = -37.5;
    step.drive.reflectedRotatingInertiaKgM2 = 0.42;
    step.drive.vehicleSpeedMps = -3.0;
    step.drive.brakePressure = 0.7;
    step.drive.engagedGear = 1;
    enginelab::EngineControls controls;
    controls.ignitionEnabled = true;
    enginelab::applyVehicleStep(controls, step);
    require(controls.engagedGear == 1 && controls.vehicleSpeedMps == 3.0,
        "the ECU is told the engaged gear and the road speed (launch control reads them)");
    require(controls.throttle == 0.4 && controls.shiftIgnitionCut && controls.brake == 0.7,
        "the controls carry the arbitrated throttle, the ignition cut and the brake");
    require(controls.externalTorqueNm == -37.5 && controls.externalRotatingInertiaKgM2 == 0.42,
        "the controls carry the driveline's coupling torque and reflected inertia");
    require(controls.coolingAirSpeedMps == 3.0, "reversing still cools the engine");
    require(controls.ignitionEnabled && controls.load == 0.0 && controls.dynamometerTorqueNm == 0.0,
        "the vehicle step leaves the ignition and the dyno commands alone");
}

/** advanceVehicle() gives the driveline exactly what a direct advance() gets:
 * the arbiter changes no driveline behaviour by itself. */
void checkDrivelineUnchanged(const enginelab::EngineConfig& config) {
    enginelab::DrivelineModel direct(config);
    enginelab::DrivelineModel arbitrated(config);
    direct.requestGear(1);
    arbitrated.requestGear(1);
    enginelab::EngineState state;
    state.rpm = config.idleRpm;
    state.angularVelocityRadPerSecond = config.idleRpm * 2.0 * 3.141592653589793 / 60.0;
    for (int tick = 0; tick < 2400; ++tick) {
        const auto seconds = tick / 240.0;
        const auto clutch = std::clamp(seconds - 1.0, 0.0, 1.0);
        const auto brake = seconds > 8.0 ? 0.5 : 0.0;
        const auto expected = direct.advance(1.0 / 240.0, state, 0.0, clutch, brake);
        const auto step = enginelab::advanceVehicle(arbitrated, 1.0 / 240.0, state, 0.0,
            enginelab::DriverInputs { 0.3, clutch, brake });
        require(step.drive.engineCouplingTorqueNm == expected.engineCouplingTorqueNm
                && step.drive.vehicleSpeedMps == expected.vehicleSpeedMps
                && step.drive.clutchSlipRpm == expected.clutchSlipRpm
                && step.drive.engagedGear == expected.engagedGear,
            "advanceVehicle must drive the driveline exactly as a direct advance");
    }
}
} // namespace

int main() {
    try {
        checkThrottleRules();
        checkControls();
        const auto catalog = enginelab::loadEngineCatalog(std::filesystem::path(ENGINELAB_CATALOG_ROOT));
        require(catalog.errors.empty(), "the catalogue must load");
        const auto found = std::find_if(catalog.entries.begin(), catalog.entries.end(),
            [](const auto& entry) { return entry.config.name.find("CP2 MT-07") != std::string::npos; });
        require(found != catalog.entries.end(), "the CP2 must exist in the catalogue");
        checkDrivelineUnchanged(found->config);
        std::printf("driver-command arbitration: all checks passed\n");
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
