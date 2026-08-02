#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/calibration/EcuCalibration.hpp>
#include <enginelab/calibration/EcuCalibrationKeys.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
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
        config.fuel == FuelType::diesel
            ? std::max(controls.throttle, state.load)
            : state.manifoldPressureKpa
                / std::max(1.0, config.ambientPressureKpa),
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
            // Releasing this floor faster when the engine flares was measured
            // and REJECTED. It does fix the two engines whose flare is worst
            // (radial 0 -> 683 rpm, Big Twin sigma 57.7 -> 16.7), but the floor
            // is precisely what keeps a flaring engine breathing: bleeding it on
            // overspeed at 6.0/s took the Audi I5 from a settled 780/746 rpm
            // (sigma 9.3) to a full stall. The flare is not the destructive
            // event -- the fuel cut that the flare triggers is. See the
            // after-start DFCO inhibit below.
            postStartAir *= std::exp(-idleDt / postStartDecaySeconds);
            if (postStartAir < 1.0e-4) postStartAir = 0.0;
        }
        postStartAirOpening_.store(postStartAir, std::memory_order_relaxed);

        // The governor's own command, and the position the actuator will
        // actually take. They are NOT the same signal: the post-start floor and
        // the deceleration dashpot each override the governor from below, so
        // for seconds after a catch the valve is held open by the schedule
        // while the governor's output is ignored.
        const auto governorCommand = [&](double integral) noexcept {
            return feedForward + proportionalGain * normalizedError + integral;
        };
        const auto deliveredFor = [&](double integral) noexcept {
            return std::max(postStartAir,
                std::max(std::clamp(governorCommand(integral), 0.0, 1.0), idleDashpot)
                    * driverOverride);
        };

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
            const auto proposedCommand = governorCommand(proposedIntegral);
            // Anti-windup has to test the DELIVERED actuator position, not the
            // governor's own command, and this is the difference between an
            // idle that settles and one that hunts.
            //
            // While the post-start floor holds the valve open, the engine sits
            // ABOVE its idle target (that is what a post-start flare is), so
            // the error is negative and the integral winds down -- even though
            // the governor is not moving the actuator at all. The old guard
            // could not see this, because it compared the governor's own
            // command against 0 and 1 and that command sits mid-range.
            // Measured on the 2JZ: from t=5.0 s to t=9.25 s the delivered
            // opening equalled the floor to three decimals while the integral
            // was driven to its -0.20 stop and pinned there for two seconds.
            // The floor then bled below the governor and handed over to a
            // controller with no authority left; the engine sagged from 847 to
            // 571 rpm while the integral climbed back at 0.11/s, overshot, and
            // rang at ~0.2 Hz for the rest of the run. That ringing is what the
            // idle gate was sampling -- whether a run "passed" depended on
            // which phase of it the 4 s measurement window happened to catch.
            //
            // So: when something other than the governor owns the actuator,
            // the governor is saturated LOW and may only integrate in the
            // direction that takes it back into control (error > 0, engine
            // below target). Otherwise it holds its authority.
            const auto governorOwnsActuator =
                std::clamp(proposedCommand, 0.0, 1.0) * driverOverride
                    >= deliveredFor(proposedIntegral) - 1.0e-9;
            const auto admissible = governorOwnsActuator
                ? ((proposedCommand > 0.0 && proposedCommand < 1.0)
                    || (proposedCommand <= 0.0 && normalizedError > 0.0)
                    || (proposedCommand >= 1.0 && normalizedError < 0.0))
                : normalizedError > 0.0;
            if (admissible) idleIntegral = proposedIntegral;
        }

        idleAirOpening = deliveredFor(idleIntegral);
    } else {
        idleIntegral *= std::exp(-idleDt * 5.0);
        idleDashpot *= std::exp(-idleDt * 5.0);
    }
    idleIntegral_.store(idleIntegral, std::memory_order_relaxed);
    idleDashpot_.store(idleDashpot, std::memory_order_relaxed);
    idleAirOpening_.store(idleAirOpening, std::memory_order_relaxed);
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
    // A cranking-retard clamp (`rpm < 500 -> advance <= 4 deg`) was tried here
    // to cure the radial R5's start kickback and is deliberately NOT kept: the
    // real cause was starter sizing (see EngineSimulator), and measured with
    // that fixed the clamp is unnecessary -- the R5 starts and idles without
    // it. It is not free either. A bare rpm threshold is inside the normal
    // operating envelope of a low-idling engine, so it fires on an ordinary
    // idle dip and yanks the advance exactly when the engine needs torque to
    // recover: it alone took the 2JZ idle from 728/709 rpm (sigma 12.1) to
    // 682/559 (sigma 52.8) and the Audi I5 from 760/737 to 718/594. A real
    // start-mode retard is gated on the ECU's run/start state, not on speed.
    const auto softLimit = state.rpm > revLimit - 220.0;
    const auto alternatingCut = softLimit && (static_cast<std::uint64_t>(state.simulationTimeSeconds * 120.0) & 1U) != 0U;
    const auto fuelEnabled = controls.ignitionEnabled
        && (!limiterActive || config.ignition.limiterKeepsFuel);
    const auto sparkEnabled = controls.ignitionEnabled && !limiterActive;
    auto decelerationFuelCut = decelerationFuelCutLatched_.load(
        std::memory_order_relaxed);
    const auto idleTargetRpm = std::max(300.0, config.idleRpm);
    // The ratio scales the refill window with each engine's idle speed while
    // retaining real hysteresis against the 1.65-times-idle entry threshold.
    // An additive margin was tested here, but it changed the start-up DFCO
    // sequence of low-idle, uneven-firing engines and could destabilise their
    // learned idle mixture long after the initial coast event.
    const auto decelerationFuelResumeRpm = idleTargetRpm * 1.25;
    // Deceleration fuel cut is an OVERRUN function: it presumes a running,
    // warmed engine coasting down under a shut throttle. The post-start flare
    // satisfies its speed threshold while being the exact opposite condition --
    // the engine is accelerating away from a catch with an empty port film --
    // and cutting there is what makes a marginal idle unrecoverable. Measured
    // on the radial: fuel was cut 0.3 s after catch at 1433 rpm against a 640
    // target, cycle torque went from +209 to -70 Nm, the engine fell back
    // through the catch threshold, re-cranked, and repeated. Every production
    // ECU inhibits overrun cut through the after-start phase for this reason.
    //
    // The after-start air schedule IS that phase, already maintained above and
    // already longer on a cold engine, which is exactly when a real inhibit
    // lasts longest. Gate on it rather than adding a second timer -- the
    // comment on the start-mode retard above makes the same point: this belongs
    // on the ECU's run/start state, not on a speed threshold.
    auto overrunAfterfireArmed = overrunAfterfireArmed_.load(
        std::memory_order_relaxed);
    if (!controls.ignitionEnabled || controls.starterEngaged) {
        overrunAfterfireArmed = false;
    } else if (config.exhaustAfterfire.enabled
            && config.exhaustAfterfire.overrunFuelFraction > 0.0
            && effectiveThrottle > 0.20
            && state.rpm >= config.exhaustAfterfire.overrunMinimumRpm) {
        // The post-start air floor can remain above its normal DFCO-inhibit
        // threshold for more than ten seconds on a cold start. An authored
        // afterfire engine which the driver has deliberately taken above its
        // RPM gate is no longer in an accidental start flare: arm its next
        // lift-off without changing clean/default engines or weakening the
        // ordinary idle-stability protection.
        overrunAfterfireArmed = true;
    }
    overrunAfterfireArmed_.store(overrunAfterfireArmed,
        std::memory_order_relaxed);
    const auto afterStartPhase =
        postStartAirOpening_.load(std::memory_order_relaxed) > 0.05
        && !overrunAfterfireArmed;
    if (controls.starterEngaged || effectiveThrottle > 0.02 || afterStartPhase
            || state.rpm < decelerationFuelResumeRpm) {
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
    // This is an ECU calibration, not a pop scheduler. It only retains a small
    // authored fraction of the normal fuel request while the existing DFCO
    // state proves that the engine is in closed-throttle overrun. Spark is cut,
    // but the downstream chemistry must still find oxygen and sufficient gas
    // temperature before any heat or sound can be produced.
    const auto overrunAfterfireActive =
        config.fuel == FuelType::gasoline
        && config.exhaustAfterfire.enabled
        && config.exhaustAfterfire.overrunFuelFraction > 0.0
        && decelerationFuelCut && !limiterActive
        && controls.ignitionEnabled && !controls.starterEngaged
        && state.rpm >= config.exhaustAfterfire.overrunMinimumRpm
        && effectiveThrottle <= config.exhaustAfterfire.overrunMaximumThrottle;
    const auto fuelCorrection = overrunAfterfireActive
        // This is a bounded fraction of the normal charge request. Do not let
        // the preceding tip-in reserve or cold-start correction multiply it:
        // both can still be decaying at lift-off and would turn a conservative
        // 12% strategy into an uncontrolled rich pulse.
        ? config.exhaustAfterfire.overrunFuelFraction
        : warmupCorrection * crankingCorrection
            * (1.0 + accelerationFuelEnrichment * 1.40)
            * decelerationFuelResume;
    const auto commandedFuelEnabled = overrunAfterfireActive
        || (fuelEnabled && !decelerationFuelCut);
    const auto commandedSparkEnabled = !overrunAfterfireActive
        && sparkEnabled && !alternatingCut;
    const auto wetSparkCutActive =
        config.fuel == FuelType::gasoline
        && config.ignition.limiterKeepsFuel
        && !overrunAfterfireActive
        && commandedFuelEnabled && !commandedSparkEnabled
        && !controls.starterEngaged
        && (limiterActive || alternatingCut);
    return { mappedAfr, mappedAdvance,
             effectiveThrottle, idleAirOpening,
             fuelCorrection,
             commandedFuelEnabled,
             commandedSparkEnabled,
             overrunAfterfireActive,
             wetSparkCutActive,
             softLimit,
             limiterActive,
             alternatingCut,
             decelerationFuelCut };
}
} // namespace enginelab
