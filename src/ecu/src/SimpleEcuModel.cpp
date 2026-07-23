#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/calibration/EcuCalibration.hpp>
#include <enginelab/calibration/EcuCalibrationKeys.hpp>
#include <algorithm>
#include <array>
#include <cmath>
namespace enginelab {
namespace {
constexpr std::array<double, 4> rpmAxis { 0.0, 2'000.0, 4'500.0, 8'000.0 };
constexpr std::array<double, 3> loadAxis { 0.0, 0.5, 1.0 };
using CalibrationTable = std::array<std::array<double, loadAxis.size()>, rpmAxis.size()>;

double interpolate(const CalibrationTable& table, double rpm, double load) noexcept {
    const auto upperRpm = std::upper_bound(rpmAxis.begin(), rpmAxis.end(), rpm);
    const auto upperLoad = std::upper_bound(loadAxis.begin(), loadAxis.end(), load);
    const auto r1 = static_cast<std::size_t>(std::clamp<std::ptrdiff_t>(upperRpm - rpmAxis.begin(), 1, rpmAxis.size() - 1));
    const auto l1 = static_cast<std::size_t>(std::clamp<std::ptrdiff_t>(upperLoad - loadAxis.begin(), 1, loadAxis.size() - 1));
    const auto r0 = r1 - 1; const auto l0 = l1 - 1;
    const auto rt = std::clamp((rpm - rpmAxis[r0]) / (rpmAxis[r1] - rpmAxis[r0]), 0.0, 1.0);
    const auto lt = std::clamp((load - loadAxis[l0]) / (loadAxis[l1] - loadAxis[l0]), 0.0, 1.0);
    const auto low = std::lerp(table[r0][l0], table[r0][l1], lt);
    const auto high = std::lerp(table[r1][l0], table[r1][l1], lt);
    return std::lerp(low, high, rt);
}

double interpolateTimingCurve(const std::vector<IgnitionMapSample>& curve, double rpm) noexcept {
    if (curve.empty()) return 18.0;
    if (curve.size() == 1 || rpm <= curve.front().rpm) return curve.front().advanceDegrees;
    for (std::size_t index = 1; index < curve.size(); ++index) {
        if (rpm <= curve[index].rpm) {
            const auto span = std::max(1.0, curve[index].rpm - curve[index - 1].rpm);
            const auto t = std::clamp((rpm - curve[index - 1].rpm) / span, 0.0, 1.0);
            return std::lerp(curve[index - 1].advanceDegrees, curve[index].advanceDegrees, t);
        }
    }
    return curve.back().advanceDegrees;
}
}

SimpleEcuModel::SimpleEcuModel()
    : SimpleEcuModel(std::make_shared<calibration::CalibrationStore>()) {}

SimpleEcuModel::SimpleEcuModel(
    std::shared_ptr<calibration::CalibrationStore> calibrations)
    : calibrations_(calibrations ? std::move(calibrations)
                                 : std::make_shared<calibration::CalibrationStore>()),
      calibrationReader_(calibrations_->registerReader()),
      frameCalibration_(calibrations_->snapshot()) {}

void SimpleEcuModel::initialiseCalibration(const EngineConfig& config) {
    if (calibrations_->snapshot()->revision() == 0) {
        (void)calibrations_->publish(calibration::makeDefaultEcuCalibration(config),
                                     { 0, "engine-default" });
    }
    beginFrame();
}

void SimpleEcuModel::beginFrame() noexcept {
    auto next = calibrations_->snapshot();
    frameCalibration_ = std::move(next);
    calibrationReader_->acknowledge(frameCalibration_->revision());
}

EcuCommand SimpleEcuModel::evaluate(const EngineConfig& config, const EngineState& state,
                                    const EngineControls& controls) const noexcept {
    const auto& calibrationSnapshot = frameCalibration_;
    const calibration::AxisCoordinate rpmCoordinate {
        calibration::AxisQuantity::engineSpeed,
        calibration::CalibrationUnit::revolutionsPerMinute,
        state.rpm
    };
    const auto normalizedLoad = std::clamp(
        state.manifoldPressureKpa / std::max(1.0, config.ambientPressureKpa),
        calibration::ecuLimits::minimumNormalizedLoad,
        calibration::ecuLimits::maximumNormalizedLoad);
    const calibration::AxisCoordinate loadCoordinate {
        calibration::AxisQuantity::normalizedLoad,
        calibration::CalibrationUnit::ratio,
        normalizedLoad
    };
    const auto calibratedRevLimit = calibrationSnapshot->scalarOr(
        calibration::keys::revLimit, config.ignition.revLimitRpm);
    const auto revLimit = std::clamp(calibratedRevLimit,
        calibration::ecuLimits::minimumRevLimitRpm,
        calibration::ecuLimits::maximumRevLimitRpm);
    auto limiterActive = limiterLatched_.load(std::memory_order_relaxed);
    if (state.rpm >= revLimit) {
        limiterActive = true;
        limiterReleaseTime_.store(state.simulationTimeSeconds + config.ignition.limiterDurationSeconds,
                                  std::memory_order_relaxed);
    } else if (state.rpm < revLimit - 180.0
               && state.simulationTimeSeconds >= limiterReleaseTime_.load(std::memory_order_relaxed)) {
        limiterActive = false;
    }
    limiterLatched_.store(limiterActive, std::memory_order_relaxed);

    // The driver's throttle and the idle-air actuator are separate physical
    // controls.  The former must be able to close completely; using a minimum
    // throttle opening as an idle controller made every engine idle high and
    // made a released keyboard throttle remain physically open.
    const auto effectiveThrottle = std::clamp(controls.throttle, 0.0, 1.0);
    const auto previousIdleTime = previousIdleEvaluationTime_.exchange(
        state.simulationTimeSeconds, std::memory_order_relaxed);
    const auto idleDt = previousIdleTime > 0.0
        ? std::clamp(state.simulationTimeSeconds - previousIdleTime, 0.0, 0.02) : 0.0;
    auto idleIntegral = idleIntegral_.load(std::memory_order_relaxed);
    auto idleDashpot = idleDashpot_.load(std::memory_order_relaxed);
    auto idleAirOpening = 0.0;
    if (controls.ignitionEnabled || controls.starterEngaged) {
        const auto targetRpm = std::max(300.0, config.idleRpm);
        const auto normalizedError = (targetRpm - state.rpm) / targetRpm;
        // Cross-fade against the physical (smoothed) throttle plate, not the
        // driver's instantaneous request. Otherwise the bypass snaps shut one
        // integration step before the plate has opened and creates a real air
        // gap during tip-in.
        const auto driverOverride = std::clamp(1.0 - state.throttle * 12.0, 0.0, 1.0);
        constexpr double feedForward = 0.42;
        constexpr double proportionalGain = 0.90;
        constexpr double integralGain = 0.55;
        constexpr double minimumIntegral = -0.20;
        if (effectiveThrottle > 0.02) {
            // A real throttle/idle system does not snap from a driven opening
            // to the closed-throttle stop. Preserve a short deceleration-air
            // dashpot so manifold filling and fuel transport can settle before
            // the PI loop resumes sole authority.
            idleDashpot = std::max(idleDashpot,
                std::clamp(effectiveThrottle * 2.0, 0.0, 0.65));
        } else if (state.rpm < targetRpm * 1.30) {
            // Near idle: hand back to the governor, bleeding the dashpot out.
            idleDashpot *= std::exp(-idleDt * 1.25);
        }
        // Above ~1.3x idle at a shut throttle the dashpot is HELD, not decayed.
        // A tip-out from a rev coasts down over seconds; the former fixed ~0.8 s
        // decay expired while the engine was still far above idle, so the bypass
        // was already shut when it finally arrived. With the plate shut too, the
        // manifold had pumped down to vacuum, the first fired cycles after the
        // fuel cut released made almost no torque, and the engine sagged straight
        // through idle and stalled (measured: LS3 to 314 rpm, Merlin to a full
        // stall on a 0.5 s blip recovery). Holding the dashpot through the coast
        // keeps a live manifold charge so combustion re-establishes at idle. It
        // only ever charges from a throttle-open condition, so a pure motoring
        // overrun (throttle never opened) is unaffected.
        if (driverOverride < 0.999) {
            // The driver's pedal is physically closing the bypass downstream of
            // this PI controller.  Integrating an overspeed error while the
            // actuator is overridden used to wind the state down to -0.42; on
            // lift-off the bypass then stayed shut until the engine had already
            // fallen through its combustion threshold.  Track the neutral
            // feed-forward state while overridden so returning to idle is
            // bumpless even after a long rev.
            idleIntegral *= std::exp(-idleDt * 3.0);
        } else {
            const auto proposedIntegral = std::clamp(
                idleIntegral + normalizedError * integralGain * idleDt,
                minimumIntegral, 0.58);
            const auto proposedCommand = feedForward
                + proportionalGain * normalizedError + proposedIntegral;
            // Conditional integration prevents wind-up when the actuator is on
            // either stop and the error would push it farther into saturation.
            if ((proposedCommand > 0.0 && proposedCommand < 1.0)
                || (proposedCommand <= 0.0 && normalizedError > 0.0)
                || (proposedCommand >= 1.0 && normalizedError < 0.0))
                idleIntegral = proposedIntegral;
        }

        // Post-start air.
        //
        // Cranking air was commanded only below 350 rpm, so it vanished the
        // instant the engine caught. The idle PI then saw a speed far above its
        // target -- the post-start flare every engine has -- and closed the
        // bypass completely. With the plate also shut the engine had almost no
        // air path at all: measured on the catalogue, manifold pressure fell to
        // about 10 kPa, cycle torque went to roughly -75 Nm as the engine pumped
        // against the vacuum it had created, and it died before the PI could
        // reopen. Only the two engines whose flare stayed near their idle target
        // survived.
        //
        // Real engine management does not hand the idle valve to the governor at
        // the moment of catch. It holds an elevated opening after start and bleeds
        // it out over seconds, so the engine breathes while its speed settles and
        // while the port fuel film is still building. That schedule is what is
        // modelled here: charged during cranking, then decaying with a time
        // constant, and always a floor under the governor rather than a
        // replacement for it. The governor still has full authority to open
        // further, and once the schedule has bled away it has sole authority.
        //
        // Cold engines need it for longer, because a cold port wets more fuel and
        // a cold engine has more friction to overcome.
        const auto postStartDecaySeconds = std::clamp(
            2.5 + (70.0 - state.coolantTemperatureC) * 0.045, 2.5, 6.0);
        auto postStartAir = postStartAirOpening_.load(std::memory_order_relaxed);
        if (controls.starterEngaged || state.rpm < 350.0) {
            // Cranking: hold the schedule charged, so it starts full at catch.
            postStartAir = 0.88;
        } else {
            postStartAir *= std::exp(-idleDt / postStartDecaySeconds);
            if (postStartAir < 1.0e-4) postStartAir = 0.0;
        }
        postStartAirOpening_.store(postStartAir, std::memory_order_relaxed);

        idleAirOpening = std::max(postStartAir,
            std::max(std::clamp(feedForward + proportionalGain * normalizedError
                                    + idleIntegral, 0.0, 1.0),
                     idleDashpot) * driverOverride);
    } else {
        idleIntegral *= std::exp(-idleDt * 5.0);
        idleDashpot *= std::exp(-idleDt * 5.0);
    }
    idleIntegral_.store(idleIntegral, std::memory_order_relaxed);
    idleDashpot_.store(idleDashpot, std::memory_order_relaxed);
    const auto warmupCorrection = std::clamp(1.0 + (70.0 - state.coolantTemperatureC) * 0.0025, 1.0, 1.12);
    const auto crankingCorrection = controls.starterEngaged
        ? 1.0 + std::clamp((700.0 - state.rpm) / 700.0, 0.0, 1.0) * 0.38 : 1.0;
    constexpr CalibrationTable afrCorrection {{
        {{ 0.0, 0.0, -0.3 }}, {{ 0.7, 0.1, -1.0 }},
        {{ 0.9, 0.0, -1.5 }}, {{ 0.6, -0.2, -1.7 }} }};
    constexpr CalibrationTable advanceCorrection {{
        {{ -5.0, -5.0, -6.0 }}, {{ 5.0, 2.0, -1.0 }},
        {{ 9.0, 5.0, 1.0 }}, {{ 11.0, 6.0, 0.0 }} }};
    const auto legacyAfr = calibration::ecuLimits::referenceAirFuelRatio
        + afrTrim_.load(std::memory_order_relaxed)
        + interpolate(afrCorrection, state.rpm, normalizedLoad);
    auto calibratedAfr = calibrationSnapshot->sampleTable(
        calibration::keys::targetAirFuelRatio, rpmCoordinate, loadCoordinate);
    if (!calibratedAfr)
        calibratedAfr = calibrationSnapshot->sampleCurve(
            calibration::keys::targetAirFuelRatio, rpmCoordinate);
    // The lightweight live control is an explicit trim around the active map.
    const auto afrCommand = calibratedAfr
        ? *calibratedAfr + afrTrim_.load(std::memory_order_relaxed)
        : legacyAfr;
    auto mappedAfr = std::clamp(afrCommand,
        calibration::ecuLimits::minimumAirFuelRatio,
        calibration::ecuLimits::maximumAirFuelRatio);
    const auto configuredAdvance = interpolateTimingCurve(config.ignition.timingCurve, state.rpm);
    const auto liveTrim = ignitionTrimDegrees_.load(std::memory_order_relaxed);
    const auto legacyAdvance = configuredAdvance
        + interpolate(advanceCorrection, state.rpm, normalizedLoad);
    auto calibratedAdvance = calibrationSnapshot->sampleTable(
        calibration::keys::ignitionAdvance, rpmCoordinate, loadCoordinate);
    if (!calibratedAdvance)
        calibratedAdvance = calibrationSnapshot->sampleCurve(
            calibration::keys::ignitionAdvance, rpmCoordinate);
    auto mappedAdvance = std::clamp(calibratedAdvance.value_or(legacyAdvance) + liveTrim,
        calibration::ecuLimits::minimumIgnitionAdvanceDegrees,
        calibration::ecuLimits::maximumIgnitionAdvanceDegrees);
    const auto previousThrottle = previousThrottle_.exchange(effectiveThrottle, std::memory_order_relaxed);
    const auto throttleIncrease = std::clamp(effectiveThrottle - previousThrottle, 0.0, 0.35);
    auto accelerationFuelEnrichment = accelerationFuelEnrichment_.load(std::memory_order_relaxed);
    if (throttleIncrease > 0.0) {
        // Accumulate the complete pedal movement even though evaluate() runs at
        // the solver substep rate. The former one-substep AFR trim vanished on
        // the next substep and made tip-in behaviour depend on solver frequency.
        accelerationFuelEnrichment = std::clamp(
            accelerationFuelEnrichment + throttleIncrease * 2.0, 0.0, 0.80);
    } else {
        accelerationFuelEnrichment *= std::exp(-idleDt * 1.8);
    }
    accelerationFuelEnrichment_.store(accelerationFuelEnrichment,
                                      std::memory_order_relaxed);
    mappedAfr = std::clamp(mappedAfr - throttleIncrease * 2.2
        - std::max(0.0, state.coolantTemperatureC - 108.0) * 0.025,
        calibration::ecuLimits::minimumAirFuelRatio,
        calibration::ecuLimits::maximumAirFuelRatio);
    mappedAdvance = std::clamp(mappedAdvance - state.knockLevel * 12.0
        - std::max(0.0, state.coolantTemperatureC - 108.0) * 0.20,
        calibration::ecuLimits::minimumIgnitionAdvanceDegrees,
        calibration::ecuLimits::maximumIgnitionAdvanceDegrees);
    const auto softLimit = state.rpm > revLimit - 220.0;
    const auto alternatingCut = softLimit && (static_cast<std::uint64_t>(state.simulationTimeSeconds * 120.0) & 1U) != 0U;
    const auto enabled = controls.ignitionEnabled && !limiterActive;
    auto decelerationFuelCut = decelerationFuelCutLatched_.load(
        std::memory_order_relaxed);
    const auto idleTargetRpm = std::max(300.0, config.idleRpm);
    if (controls.starterEngaged || effectiveThrottle > 0.02
            || state.rpm < idleTargetRpm * 1.25) {
        decelerationFuelCut = false;
    } else if (state.throttle < 0.02 && state.rpm > idleTargetRpm * 1.65) {
        // Stop metering fuel on closed-throttle overrun so port film and
        // residual chamber inventory cannot flood the charge that must catch
        // the engine at idle. Hysteresis keeps the injector command stable.
        decelerationFuelCut = true;
    }
    decelerationFuelCutLatched_.store(decelerationFuelCut,
                                      std::memory_order_relaxed);
    auto decelerationFuelResume = decelerationFuelResume_.load(
        std::memory_order_relaxed);
    if (decelerationFuelCut) {
        decelerationFuelResume = 0.0;
    } else if (controls.starterEngaged || effectiveThrottle > 0.02) {
        decelerationFuelResume = 1.0;
    } else {
        decelerationFuelResume = std::min(1.0,
            decelerationFuelResume + idleDt * 3.0);
    }
    decelerationFuelResume_.store(decelerationFuelResume,
                                  std::memory_order_relaxed);
    return { mappedAfr, mappedAdvance,
             effectiveThrottle, idleAirOpening,
             warmupCorrection * crankingCorrection
                 * (1.0 + accelerationFuelEnrichment * 1.40)
                 * decelerationFuelResume,
             enabled && !decelerationFuelCut,
             enabled && !alternatingCut };
}
} // namespace enginelab
