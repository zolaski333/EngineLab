#include <enginelab/physics/ValveTrainModel.hpp>
#include <algorithm>
#include <cmath>
#include <numbers>

namespace enginelab {

ValveTrainResult ValveTrainModel::evaluate(const CamshaftConfig& cam, bool highProfile,
                                           ValveTrainState& state, double phase, double rpm,
                                           double load, double dtSeconds) noexcept {
    const auto target = interpolateValveControl(cam.continuousControl, rpm, load);
    const auto response = 1.0 - std::exp(-std::max(0.0, dtSeconds)
        * 2.0 * std::numbers::pi * cam.continuousControl.responseFrequencyHz);
    state.intakeAdvanceDegrees = std::lerp(state.intakeAdvanceDegrees,
        cam.continuousControl.enabled ? target.intakeAdvanceDegrees : 0.0, response);
    state.exhaustAdvanceDegrees = std::lerp(state.exhaustAdvanceDegrees,
        cam.continuousControl.enabled ? target.exhaustAdvanceDegrees : 0.0, response);
    state.liftMultiplier = std::lerp(state.liftMultiplier,
        cam.continuousControl.enabled ? target.liftMultiplier : 1.0, response);

    const auto intakeDuration = highProfile ? cam.highIntakeDurationDegrees : cam.intakeDurationDegrees;
    const auto exhaustDuration = highProfile ? cam.highExhaustDurationDegrees : cam.exhaustDurationDegrees;
    const auto intakeMaximumLift = highProfile ? cam.highIntakeLiftMm : cam.intakeLiftMm;
    const auto exhaustMaximumLift = highProfile ? cam.highExhaustLiftMm : cam.exhaustLiftMm;
    const auto& intakeProfile = highProfile ? cam.highIntakeLiftProfile : cam.intakeLiftProfile;
    const auto& exhaustProfile = highProfile ? cam.highExhaustLiftProfile : cam.exhaustLiftProfile;
    const auto intakeCenter = 360.0 + cam.intakeCenterlineDegrees - state.intakeAdvanceDegrees;
    const auto exhaustCenter = 360.0 - cam.exhaustCenterlineDegrees - state.exhaustAdvanceDegrees;
    const auto intakeLift = profiledValveLiftMm(phase, intakeCenter, intakeDuration,
        intakeMaximumLift, intakeProfile) * state.liftMultiplier;
    const auto exhaustLift = profiledValveLiftMm(phase, exhaustCenter, exhaustDuration,
        exhaustMaximumLift, exhaustProfile) * state.liftMultiplier;
    return { intakeLift, exhaustLift,
        valveFlowCoefficient(intakeLift, cam.intakeFlowCoefficient, cam.intakeFlowCurve),
        valveFlowCoefficient(exhaustLift, cam.exhaustFlowCoefficient, cam.exhaustFlowCurve),
        state.intakeAdvanceDegrees, state.exhaustAdvanceDegrees, state.liftMultiplier };
}

} // namespace enginelab
