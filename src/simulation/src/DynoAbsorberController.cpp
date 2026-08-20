#include <enginelab/simulation/DynoAbsorberController.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace enginelab {

DynoAbsorberController::DynoAbsorberController(
    const EngineConfig& config) noexcept {
    const auto displacementM3 =
        engineDisplacementLitres(config) * 0.001;
    // Controller scaling follows a strong naturally aspirated engine, while
    // the absorber itself has four times that capacity. Equipment capacity
    // must not be confused with a prediction of engine output.
    constexpr double controllerBrakeMeanEffectivePressurePa = 2'500'000.0;
    constexpr double absorberBrakeMeanEffectivePressurePa = 10'000'000.0;
    controllerTorqueScaleNm_ = std::max(
        1.0, controllerBrakeMeanEffectivePressurePa * displacementM3
            / (4.0 * std::numbers::pi));
    maximumBrakeTorqueNm_ = std::max(
        controllerTorqueScaleNm_,
        absorberBrakeMeanEffectivePressurePa * displacementM3
            / (4.0 * std::numbers::pi));
}

void DynoAbsorberController::reset(
    double initialRpm, double initialBrakeTorqueNm) noexcept {
    filteredRpm_ = std::isfinite(initialRpm)
        ? std::max(0.0, initialRpm) : 0.0;
    feedForwardTorqueNm_ = std::isfinite(initialBrakeTorqueNm)
        ? std::clamp(initialBrakeTorqueNm, 0.0, maximumBrakeTorqueNm_)
        : 0.0;
    filteredAccelerationRpmPerSecond_ = 0.0;
    integralTorqueNm_ = 0.0;
    brakeTorqueNm_ = 0.0;
    fullContactLatched_ = false;
    initialised_ = true;
}

DynoAbsorberOutput DynoAbsorberController::advance(
    double dtSeconds, double targetRpm,
    const EngineState& engineState,
    double requestedContactBandRpm) noexcept {
    const auto dt = std::isfinite(dtSeconds)
        ? std::clamp(dtSeconds, 0.0, 0.05) : 0.0;
    const auto target = std::isfinite(targetRpm)
        ? std::max(1.0, targetRpm) : 1.0;
    if (!initialised_)
        reset(engineState.rpm, engineState.torqueNm);

    // A non-finite observation is not a zero-speed impulse. Hold the last
    // finite observation so one corrupt frame cannot inject an artificial
    // acceleration spike into the controller.
    const auto measuredRpm = std::isfinite(engineState.rpm)
        ? std::max(0.0, engineState.rpm) : filteredRpm_;
    const auto previousFilteredRpm = filteredRpm_;
    filteredRpm_ += (measuredRpm - filteredRpm_)
        * (1.0 - std::exp(-dt * 8.0));
    const auto accelerationObservation = dt > 0.0
        ? (filteredRpm_ - previousFilteredRpm) / dt : 0.0;
    const auto rawAcceleration = std::isfinite(accelerationObservation)
        ? accelerationObservation : 0.0;
    filteredAccelerationRpmPerSecond_ +=
        (rawAcceleration - filteredAccelerationRpmPerSecond_)
        * (1.0 - std::exp(-dt * 6.0));

    // A cycle-average is the actual quantity the absorber must balance. Using
    // instantaneous gas torque makes the brake chase each firing pulse and can
    // excite the crank/flywheel mode it is supposed to hold.
    const auto cycleBrakeTorqueNm =
        std::isfinite(engineState.cycleAveragedTorqueNm)
            && engineState.cycleAveragedTorqueNm > 0.0
        ? engineState.cycleAveragedTorqueNm : 0.0;
    const auto instantaneousBrakeTorqueNm =
        std::isfinite(engineState.torqueNm)
        ? std::max(0.0, engineState.torqueNm) : 0.0;
    const auto measuredBrakeTorqueNm = cycleBrakeTorqueNm > 0.0
        ? cycleBrakeTorqueNm : instantaneousBrakeTorqueNm;
    feedForwardTorqueNm_ += (
        measuredBrakeTorqueNm - feedForwardTorqueNm_)
        * (1.0 - std::exp(-dt * 8.0));
    const auto errorRpm = filteredRpm_ - target;
    const auto proportionalGain = controllerTorqueScaleNm_ / 600.0;
    const auto integralGain = controllerTorqueScaleNm_ / 1'800.0;
    const auto accelerationGain = controllerTorqueScaleNm_ / 800.0;

    // Ramp the absorber into contact across the final 60 rpm. Switching the
    // full feed-forward torque at one threshold creates a relaxation
    // oscillator, most visibly on the high-compression diesel. A wider band
    // starts absorbing too early at the catalogue's 2,000 rpm points and can
    // leave the one-way brake wound up below its target.
    const auto contactBandRpm = std::isfinite(requestedContactBandRpm)
        ? std::clamp(requestedContactBandRpm, 20.0, 1'000.0)
        : 60.0;
    const auto contactPhase = std::clamp(
        (errorRpm + contactBandRpm) / contactBandRpm, 0.0, 1.0);
    const auto approachContactScale =
        contactPhase * contactPhase * (3.0 - 2.0 * contactPhase);
    // The smooth approach prevents a brake step. Once it carries roughly 90%
    // of the requested torque, capture the rotor and expose full contact; the
    // old memoryless scale could settle permanently around 93%, making a 95%
    // measurement gate mathematically unreachable. A full contact is released
    // only after falling an entire band below target, so ramp ripple cannot
    // chatter the coupling state.
    if (!fullContactLatched_ && approachContactScale >= 0.90)
        fullContactLatched_ = true;
    else if (fullContactLatched_ && errorRpm < -contactBandRpm)
        fullContactLatched_ = false;
    const auto contactScale = fullContactLatched_
        ? 1.0 : approachContactScale;
    const auto contactedFeedForwardTorqueNm =
        feedForwardTorqueNm_ * contactScale;
    auto unclampedBrakeTorqueNm = 0.0;
    auto saturatedLow = false;
    auto saturatedHigh = false;
    if (contactScale <= 0.0) {
        integralTorqueNm_ = 0.0;
        brakeTorqueNm_ = 0.0;
    } else {
        // Do not integrate the several-thousand-rpm pull-down from a cold
        // free-rev. That stored brake torque used to survive after the shaft
        // reached its setpoint and held the EA288 just below 2,000 rpm. Only
        // trim steady-state error inside the contact band; outside it, forget
        // any trim left by a previous sweep target.
        const auto integralCandidate =
            std::abs(errorRpm) <= contactBandRpm
            ? integralTorqueNm_ + errorRpm * integralGain * dt
            : integralTorqueNm_ * std::exp(-dt * 4.0);
        const auto unsaturated = contactedFeedForwardTorqueNm
            + integralCandidate
            + errorRpm * proportionalGain
            + filteredAccelerationRpmPerSecond_ * accelerationGain;
        const auto saturated = std::clamp(
            unsaturated, 0.0, maximumBrakeTorqueNm_);
        if (unsaturated == saturated
            || (unsaturated < 0.0 && errorRpm > 0.0)
            || (unsaturated > maximumBrakeTorqueNm_
                && errorRpm < 0.0)) {
            integralTorqueNm_ = integralCandidate;
        }
        integralTorqueNm_ = std::clamp(
            integralTorqueNm_,
            -maximumBrakeTorqueNm_, maximumBrakeTorqueNm_);
        unclampedBrakeTorqueNm = contactedFeedForwardTorqueNm
            + integralTorqueNm_
            + errorRpm * proportionalGain
            + filteredAccelerationRpmPerSecond_ * accelerationGain;
        if (!std::isfinite(unclampedBrakeTorqueNm))
            unclampedBrakeTorqueNm = 0.0;
        brakeTorqueNm_ = std::clamp(
            unclampedBrakeTorqueNm, 0.0, maximumBrakeTorqueNm_);

        // The clamp can change a value by a few ULPs at a boundary. Expose a
        // capacity event only when the rejected torque is material compared
        // with the scale of this absorber.
        const auto saturationToleranceNm =
            64.0 * std::numeric_limits<double>::epsilon()
            * std::max(1.0, maximumBrakeTorqueNm_);
        saturatedLow = unclampedBrakeTorqueNm < -saturationToleranceNm;
        saturatedHigh = unclampedBrakeTorqueNm
            > maximumBrakeTorqueNm_ + saturationToleranceNm;
    }
    return {
        brakeTorqueNm_,
        filteredRpm_,
        filteredAccelerationRpmPerSecond_,
        contactScale > 0.0,
        contactScale,
        unclampedBrakeTorqueNm,
        saturatedLow,
        saturatedHigh,
    };
}

} // namespace enginelab
