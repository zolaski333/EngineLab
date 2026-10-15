// Launch control (two-step): standing in gear under throttle, the ECU's limiter
// cuts at the calibration's launch speed; rolling, or with it at zero, the
// normal rev limit applies.

#include <enginelab/calibration/EcuCalibration.hpp>
#include <enginelab/calibration/EcuCalibrationKeys.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>

#include <cstdlib>
#include <iostream>
#include <string_view>
#include <utility>
#include <variant>

namespace {
using namespace enginelab;

void require(bool condition, std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

void publishLaunchRpm(SimpleEcuModel& ecu, double rpm) {
    auto draft = calibration::makeDraft(*ecu.calibrationStore()->snapshot());
    const auto* found = draft.find(calibration::keys::launchLimit);
    require(found != nullptr, "every engine's tables carry launch control");
    auto entry = *found;
    std::get<calibration::ScalarCalibration>(entry).value = rpm;
    draft.set(std::move(entry));
    require(ecu.calibrationStore()->publish(draft).published, "a launch speed must publish");
    ecu.beginFrame();
}

} // namespace

int main() {
    const auto config = makeDefaultInlineFour();
    SimpleEcuModel ecu;
    ecu.initialiseCalibration(config);
    ecu.beginFrame();
    require(launchControlRpm(*ecu.calibrationStore()->snapshot()) == 0.0, "launch control is off by default");

    EngineState state;
    state.coolantTemperatureC = 90.0;
    state.manifoldPressureKpa = config.ambientPressureKpa;
    state.gear = 1;
    state.vehicleSpeedMps = 0.0;
    EngineControls controls;
    controls.ignitionEnabled = true;
    controls.throttle = 1.0;
    auto time = 1.0;
    // Each evaluation a little later, past the limiter's own hold time.
    const auto at = [&](double rpm) {
        state.rpm = rpm;
        time += 1.0;
        state.simulationTimeSeconds = time;
        return ecu.evaluate(config, state, controls);
    };

    require(at(4'500.0).sparkEnabled && !hasActiveLimit(at(4'500.0).activeLimits, ActiveLimit::launchControl),
        "off, the engine revs freely at 4,500 rpm in gear at a standstill");

    publishLaunchRpm(ecu, 4'000.0);
    const auto held = at(4'100.0);
    require(!held.sparkEnabled || !held.fuelEnabled, "at a standstill above the launch speed the limiter cuts");
    require(hasActiveLimit(held.activeLimits, ActiveLimit::launchControl), "and reports launch control");
    require(held.revLimitRpm == activeRevLimitRpm(config, *ecu.calibrationStore()->snapshot()),
        "the engine's rev limit itself is unchanged (over-rev damage stays relative to it)");

    // Well below the launch speed the limiter lets go.
    const auto below = at(3'000.0);
    require(below.sparkEnabled && below.fuelEnabled, "below the launch speed the engine fires");

    state.vehicleSpeedMps = 3.0;
    const auto rolling = at(4'100.0);
    require(rolling.sparkEnabled && rolling.fuelEnabled
            && !hasActiveLimit(rolling.activeLimits, ActiveLimit::launchControl),
        "rolling above 5 km/h, launch control lets go");

    state.vehicleSpeedMps = 0.0;
    controls.throttle = 0.3;
    const auto partThrottle = at(4'100.0);
    require(partThrottle.sparkEnabled && partThrottle.fuelEnabled, "at part throttle it does not hold");

    controls.throttle = 1.0;
    state.gear = 0;
    const auto neutral = at(4'100.0);
    require(neutral.sparkEnabled && neutral.fuelEnabled, "in neutral (or on the dyno) it does not hold");

    std::cout << "launch control: ok\n";
    return 0;
}
