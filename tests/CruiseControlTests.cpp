// Cruise control: holds a speed on a toy vehicle whose steady throttle is
// known, cancels on brake, clutch, neutral and a lost speed, lets the driver's
// pedal overtake it, and refuses to engage below 30 km/h. The real engine and
// driveline are driven through it by EngineLabDriveScenarioHarness --builtin
// cruise.

#include <enginelab/runtime/CruiseControl.hpp>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string_view>

namespace {
using enginelab::CruiseControl;
using enginelab::DriverInputs;

void require(bool condition, std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

/** A 1,000 kg car: thrust proportional to throttle, aerodynamic and rolling
    drag. Its steady speed at a throttle t solves 4000 t = 150 + 0.4 v^2. */
struct ToyCar final {
    double speedMps { 0.0 };
    double gradeForceN { 0.0 };
    void step(double dt, double throttle) {
        const auto thrust = 4'000.0 * throttle;
        const auto drag = 150.0 + 0.4 * speedMps * speedMps + gradeForceN;
        speedMps = std::max(0.0, speedMps + (thrust - drag) / 1'000.0 * dt);
    }
};

constexpr double dt = 1.0 / 240.0;
const DriverInputs released { 0.0, 1.0, 0.0 };

double run(CruiseControl& cruise, ToyCar& car, double seconds, DriverInputs driver = released) {
    double throttle = 0.0;
    for (int index = 0; index < static_cast<int>(seconds / dt); ++index) {
        throttle = cruise.apply(dt, car.speedMps, driver, true).throttle;
        car.step(dt, throttle);
    }
    return throttle;
}

void checkHoldsSpeed() {
    CruiseControl cruise;
    ToyCar car;
    car.speedMps = 100.0 / 3.6;
    const auto steady = (150.0 + 0.4 * car.speedMps * car.speedMps) / 4'000.0;
    require(cruise.engage(car.speedMps, steady * 0.5), "engages at 100 km/h");
    const auto throttle = run(cruise, car, 30.0);
    require(cruise.engaged(), "stays engaged on the flat");
    require(std::abs(car.speedMps - 100.0 / 3.6) < 0.3 / 3.6, "holds 100 km/h within 0.3 km/h");
    require(std::abs(throttle - steady) < 0.01, "finds the throttle that holds the speed");

    // A climb: the grade force is a fifth of the flat-road drag.
    car.gradeForceN = 120.0;
    run(cruise, car, 30.0);
    require(cruise.engaged(), "a gentle climb does not cancel it");
    require(std::abs(car.speedMps - 100.0 / 3.6) < 0.5 / 3.6, "holds the speed up a climb");
}

void checkBumpless() {
    CruiseControl cruise;
    ToyCar car;
    car.speedMps = 80.0 / 3.6;
    const auto steady = (150.0 + 0.4 * car.speedMps * car.speedMps) / 4'000.0;
    require(cruise.engage(car.speedMps, steady), "engages at the steady throttle");
    double lowest = car.speedMps;
    double highest = car.speedMps;
    for (int index = 0; index < 2'400; ++index) {
        car.step(dt, cruise.apply(dt, car.speedMps, released, true).throttle);
        lowest = std::min(lowest, car.speedMps);
        highest = std::max(highest, car.speedMps);
    }
    require(highest - lowest < 0.05 / 3.6, "engaging at the throttle that held the speed neither surges nor drops");
}

void checkCancels() {
    const auto engagedAt = [](double speedKmh) {
        CruiseControl cruise;
        require(cruise.engage(speedKmh / 3.6, 0.2), "engages");
        return cruise;
    };
    auto brake = engagedAt(90.0);
    (void)brake.apply(dt, 90.0 / 3.6, DriverInputs { 0.0, 1.0, 0.3 }, true);
    require(!brake.engaged(), "the brake cancels it");

    auto clutch = engagedAt(90.0);
    (void)clutch.apply(dt, 90.0 / 3.6, DriverInputs { 0.0, 0.2, 0.0 }, true);
    require(!clutch.engaged(), "pressing the clutch cancels it");

    auto neutral = engagedAt(90.0);
    (void)neutral.apply(dt, 90.0 / 3.6, released, false);
    require(!neutral.engaged(), "neutral cancels it");

    auto lost = engagedAt(90.0);
    (void)lost.apply(dt, 50.0 / 3.6, released, true);
    require(!lost.engaged(), "falling far under the set speed cancels it");

    auto cancelled = engagedAt(90.0);
    const auto after = cancelled.apply(dt, 90.0 / 3.6, DriverInputs { 0.1, 1.0, 0.5 }, true);
    require(after.throttle == 0.1, "a cancelled cruise control leaves the driver's throttle alone");
    require(cancelled.apply(dt, 90.0 / 3.6, DriverInputs { 0.1, 1.0, 0.0 }, true).throttle == 0.1,
        "and does not come back by itself");

    CruiseControl slow;
    require(!slow.engage(25.0 / 3.6, 0.1) && !slow.engaged(), "refuses to engage under 30 km/h");
}

void checkDriverOvertakes() {
    CruiseControl cruise;
    ToyCar car;
    car.speedMps = 100.0 / 3.6;
    const auto steady = (150.0 + 0.4 * car.speedMps * car.speedMps) / 4'000.0;
    require(cruise.engage(car.speedMps, steady), "engages");
    // Ten seconds with the pedal at 60 %: the car runs away from the set speed.
    const auto pushed = run(cruise, car, 10.0, DriverInputs { 0.6, 1.0, 0.0 });
    require(pushed == 0.6 && cruise.engaged(), "the driver's pedal overtakes and the cruise stays set");
    require(car.speedMps > 110.0 / 3.6, "the car is faster than the set speed");
    // Let go: it returns to the set speed without overshooting below it.
    double lowest = car.speedMps;
    for (int index = 0; index < static_cast<int>(40.0 / dt); ++index) {
        car.step(dt, cruise.apply(dt, car.speedMps, released, true).throttle);
        lowest = std::min(lowest, car.speedMps);
    }
    require(std::abs(car.speedMps - 100.0 / 3.6) < 0.3 / 3.6, "returns to the set speed");
    require(lowest > 98.0 / 3.6, "the integral did not wind up while overtaken");
}
/** Set while accelerating hard: the throttle it engages at is far above the
    one that holds the speed, and it must not run far past the set speed. */
void checkEngagedWhileAccelerating() {
    CruiseControl cruise;
    ToyCar car;
    car.speedMps = 60.0 / 3.6;
    while (car.speedMps < 100.0 / 3.6) car.step(dt, 0.6);
    require(cruise.engage(car.speedMps, 0.6), "engages while accelerating");
    double highest = car.speedMps;
    for (int index = 0; index < static_cast<int>(40.0 / dt); ++index) {
        car.step(dt, cruise.apply(dt, car.speedMps, released, true).throttle);
        highest = std::max(highest, car.speedMps);
    }
    std::cout << "set at 100 km/h while accelerating at 60 %: peak " << highest * 3.6 << " km/h\n";
    require(highest < 104.0 / 3.6, "set while accelerating, it stays within 4 km/h of the set speed");
    require(std::abs(car.speedMps - 100.0 / 3.6) < 0.3 / 3.6, "and settles at the set speed");
}
} // namespace

int main() {
    checkEngagedWhileAccelerating();
    checkHoldsSpeed();
    checkBumpless();
    checkCancels();
    checkDriverOvertakes();
    std::cout << "Cruise control tests passed\n";
    return EXIT_SUCCESS;
}
