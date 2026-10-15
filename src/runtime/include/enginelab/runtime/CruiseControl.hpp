#pragma once

#include <enginelab/runtime/DriverCommandArbiter.hpp>

namespace enginelab {

/** Holds a road speed with the throttle, like a car's cruise control.

    It is a driver-side device: its throttle joins the driver's pedal (the
    higher of the two wins, so pressing the pedal overtakes it) before the
    driver-command arbiter applies the driveline's own devices. Braking,
    pressing the clutch, neutral or falling far below the set speed cancel
    it; a later engage resumes nothing, it sets the current speed. */
class CruiseControl final {
public:
    /** Below this road speed the cruise control will not engage (30 km/h). */
    static constexpr double minimumSpeedMps = 30.0 / 3.6;

    /** Engages at `speedMps`, starting from `currentThrottle` so the car
        neither surges nor drops. Returns false (and stays off) below the
        minimum speed. */
    bool engage(double speedMps, double currentThrottle) noexcept;
    void cancel() noexcept;
    [[nodiscard]] bool engaged() const noexcept { return engaged_; }
    [[nodiscard]] double setSpeedMps() const noexcept { return setSpeedMps_; }

    /** Advances the controller by `dt` and returns the driver inputs with
        its throttle merged in. `inGear` is false in neutral. */
    [[nodiscard]] DriverInputs apply(double dt, double speedMps, DriverInputs driver, bool inGear) noexcept;

private:
    bool engaged_ { false };
    double setSpeedMps_ { 0.0 };
    double integral_ { 0.0 };
    double previousSpeedMps_ { 0.0 };
};

} // namespace enginelab
