#include <enginelab/runtime/DriverCommandArbiter.hpp>

#include <algorithm>
#include <cmath>

namespace enginelab {

namespace {
double unitOr(double value, double fallback) noexcept {
    return std::isfinite(value) ? std::clamp(value, 0.0, 1.0) : fallback;
}
} // namespace

double DriverCommandArbiter::drivelineClutch(const DriverInputs& inputs,
                                             const TransmissionConfig&) noexcept {
    return unitOr(inputs.clutchPedal, 1.0);
}

void DriverCommandArbiter::arbitrate(const DriverInputs& inputs, VehicleStep& step) noexcept {
    const auto driver = unitOr(inputs.throttle, 0.0);
    const auto lifted = driver * step.drive.torqueCutMultiplier;
    const auto blip = step.drive.blipThrottle;
    step.engineThrottle = std::max(lifted, blip);
    step.throttleAuthority = blip > lifted ? ThrottleAuthority::blipper
        : lifted < driver ? ThrottleAuthority::shiftTorqueCut
        : ThrottleAuthority::driver;
    step.shiftIgnitionCut = step.drive.ignitionCut;
}

VehicleStep advanceVehicle(DrivelineModel& driveline, double dtSeconds,
                           const EngineState& engineState, double requestedLoad,
                           const DriverInputs& inputs) noexcept {
    VehicleStep step;
    step.drive = driveline.advance(dtSeconds, engineState, requestedLoad,
        DriverCommandArbiter::drivelineClutch(inputs, driveline.transmission()),
        unitOr(inputs.brake, 0.0));
    DriverCommandArbiter::arbitrate(inputs, step);
    return step;
}

void applyVehicleStep(EngineControls& controls, const VehicleStep& step) noexcept {
    controls.throttle = step.engineThrottle;
    controls.brake = step.drive.brakePressure;
    controls.externalTorqueNm = step.drive.engineCouplingTorqueNm;
    controls.externalRotatingInertiaKgM2 = step.drive.reflectedRotatingInertiaKgM2;
    controls.coolingAirSpeedMps = std::abs(step.drive.vehicleSpeedMps);
    controls.shiftIgnitionCut = step.shiftIgnitionCut;
}

} // namespace enginelab
