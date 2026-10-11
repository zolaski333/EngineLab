#pragma once

// Simulator-owned valve-profile demand and actuation. Physical inputs only.
#include <enginelab/physics/ValveTrainModel.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

namespace enginelab::cam_controller {

namespace detail {
inline constexpr double shaftPeriodDegrees = 720.0;
[[nodiscard]] inline bool validPhase(double phase) noexcept {
    return std::isfinite(phase) && phase >= 0.0 && phase < shaftPeriodDegrees;
}
[[nodiscard]] inline bool roundoffClose(double first, double second,
                                        double scale = shaftPeriodDegrees) noexcept {
    // A consistency check for independently converted physical angles, not a
    // control dead band. No RPM threshold is widened by this allowance.
    return std::abs(first - second) <= 32.0 * std::numeric_limits<double>::epsilon()
        * std::max({scale, std::abs(first), std::abs(second)});
}
[[nodiscard]] inline bool samePhase(double first, double second) noexcept {
    return roundoffClose(std::remainder(first - second, shaftPeriodDegrees), 0.0);
}
}

struct ShaftCycleMeasurement final {
    std::uint64_t epoch {};
    std::uint64_t generation {};
    double durationSeconds {};
    double minimumRpm {};
    double maximumRpm {};
    double meanRpm {};
};

struct ShaftAdvanceResult final {
    bool accepted {};
    std::uint64_t completedCycles {};
};

class ShaftCycleWindow final {
public:
    void reset() noexcept {
        ++epoch_;
        generation_ = 0;
        active_ = aligned_ = false;
        seconds_ = 0.0;
        minimum_ = std::numeric_limits<double>::infinity();
        maximum_ = 0.0;
        expectedBeginPhase_.reset();
        lastTravelDegrees_.reset();
        completed_.reset();
    }

    [[nodiscard]] const std::optional<ShaftCycleMeasurement>& completed() const noexcept {
        return completed_;
    }
    [[nodiscard]] std::optional<double> lastTravelDegrees() const noexcept {
        return lastTravelDegrees_;
    }

    // Begin/end are the actual normalized shaft angles from the mechanical
    // integrator. Travel is its unwrapped forward angle. RPM is linear within
    // this physical step; the crossing time is solved from that acceleration.
    // The end angle disambiguates an exact wrap rounded just below 720 degrees.
    [[nodiscard]] ShaftAdvanceResult advance(double beginPhase, double endPhase,
        double travelDegrees, double durationSeconds, double firstRpm,
        double lastRpm) noexcept {
        const auto invalid = [this]() noexcept { reset(); return ShaftAdvanceResult {}; };
        if (!detail::validPhase(beginPhase) || !detail::validPhase(endPhase)
            || !(durationSeconds > 0.0) || !std::isfinite(durationSeconds)
            || !(travelDegrees >= 0.0) || !std::isfinite(travelDegrees)
            || !(firstRpm >= 0.0) || !(lastRpm >= 0.0)
            || !std::isfinite(firstRpm) || !std::isfinite(lastRpm))
            return invalid();
        const auto integralDegrees = (firstRpm + lastRpm) * 0.5 * 6.0 * durationSeconds;
        if (!std::isfinite(integralDegrees)
            || !detail::roundoffClose(integralDegrees, travelDegrees)) return invalid();
        if (expectedBeginPhase_ && !detail::samePhase(*expectedBeginPhase_, beginPhase))
            return invalid();
        const auto turns = std::round((beginPhase + travelDegrees - endPhase)
            / detail::shaftPeriodDegrees);
        if (!(turns >= 0.0) || !std::isfinite(turns)
            || turns >= static_cast<double>(std::numeric_limits<std::uint64_t>::max())
            || !detail::roundoffClose(beginPhase + travelDegrees,
                endPhase + turns * detail::shaftPeriodDegrees)) return invalid();
        const auto crossings = static_cast<std::uint64_t>(turns);
        if (travelDegrees == 0.0) {
            // A stalled shaft supplies no new cycle. Never leave a high
            // request backed by its last pre-stall completed cycle.
            reset(); expectedBeginPhase_ = endPhase; lastTravelDegrees_ = 0.0;
            return { true, 0 };
        }
        if (!active_) {
            active_ = true;
            aligned_ = beginPhase == 0.0;
        }
        const auto rpmAt = [&](double time) noexcept {
            return std::lerp(firstRpm, lastRpm, time / durationSeconds);
        };
        const auto crossingTime = [&](double distanceDegrees) noexcept {
            const auto initialSpeed = firstRpm * 6.0;
            const auto acceleration = (lastRpm - firstRpm) * 6.0 / durationSeconds;
            const auto discriminant = std::max(0.0,
                initialSpeed * initialSpeed + 2.0 * acceleration * distanceDegrees);
            const auto denominator = initialSpeed + std::sqrt(discriminant);
            return denominator > 0.0
                ? std::clamp(2.0 * distanceDegrees / denominator, 0.0, durationSeconds)
                : durationSeconds;
        };
        auto sealed = std::uint64_t { 0 };
        if (crossings == 0) {
            segment(durationSeconds, firstRpm, lastRpm);
        } else {
            const auto firstCrossing = crossingTime(detail::shaftPeriodDegrees - beginPhase);
            segment(firstCrossing, firstRpm, rpmAt(firstCrossing));
            if (aligned_) {
                if (!publish(seconds_, minimum_, maximum_, 1)) return invalid();
                ++sealed;
            }
            aligned_ = true;
            auto lastCrossing = firstCrossing;
            if (crossings > 1) {
                // Intermediate complete cycles need no storage or loop. Only
                // the newest complete physical cycle qualifies the next call.
                const auto lastDistance = detail::shaftPeriodDegrees
                    * static_cast<double>(crossings) - beginPhase;
                const auto previousCrossing = crossingTime(lastDistance - detail::shaftPeriodDegrees);
                lastCrossing = crossingTime(lastDistance);
                const auto first = rpmAt(previousCrossing);
                const auto last = rpmAt(lastCrossing);
                if (!publish(lastCrossing - previousCrossing,
                    std::min(first, last), std::max(first, last), crossings - 1))
                    return invalid();
                sealed += crossings - 1;
            }
            seconds_ = 0.0;
            minimum_ = std::numeric_limits<double>::infinity();
            maximum_ = 0.0;
            segment(durationSeconds - lastCrossing, rpmAt(lastCrossing), lastRpm);
        }
        if (!std::isfinite(seconds_)) return invalid();
        expectedBeginPhase_ = endPhase;
        lastTravelDegrees_ = travelDegrees;
        if (lastRpm == 0.0) {
            reset(); expectedBeginPhase_ = endPhase; lastTravelDegrees_ = travelDegrees;
        }
        return { true, sealed };
    }

private:
    void segment(double time, double first, double last) noexcept {
        seconds_ += time;
        minimum_ = std::min({minimum_, first, last});
        maximum_ = std::max({maximum_, first, last});
    }
    [[nodiscard]] bool publish(double seconds, double minimum, double maximum,
                               std::uint64_t count) noexcept {
        if (!(seconds > 0.0) || !std::isfinite(seconds) || !std::isfinite(minimum)
            || !std::isfinite(maximum) || count > std::numeric_limits<std::uint64_t>::max() - generation_)
            return false;
        generation_ += count;
        completed_ = ShaftCycleMeasurement { epoch_, generation_, seconds,
            minimum, maximum, 120.0 / seconds };
        return std::isfinite(completed_->meanRpm);
    }
    std::uint64_t epoch_ {};
    std::uint64_t generation_ {};
    bool active_ {};
    bool aligned_ {};
    double seconds_ {};
    double minimum_ { std::numeric_limits<double>::infinity() };
    double maximum_ {};
    std::optional<double> expectedBeginPhase_;
    std::optional<double> lastTravelDegrees_;
    std::optional<ShaftCycleMeasurement> completed_;
};

struct BaseCircleWitness final {
    bool valid {};
    bool common {};
    bool liftEvaluated {};
    std::uint64_t camRevision {};
    double phaseDegrees {};
    ValveTrainState actualVvtState;
    std::array<double, 4> actualLiftMm {};
};

[[nodiscard]] inline BaseCircleWitness commonBaseCircle(const CamshaftConfig& cam,
    const ValveTrainState& state, double phase, std::uint64_t revision) noexcept {
    BaseCircleWitness result { false, false, false, revision, phase, state, {} };
    if (!detail::validPhase(phase) || !std::isfinite(state.intakeAdvanceDegrees)
        || !std::isfinite(state.exhaustAdvanceDegrees) || !std::isfinite(state.liftMultiplier)
        || state.liftMultiplier < 0.0) return result;
    result.valid = true;
    result.common = true;
    const auto intakeCenter = 360.0 + cam.intakeCenterlineDegrees - state.intakeAdvanceDegrees;
    const auto exhaustCenter = 360.0 - cam.exhaustCenterlineDegrees - state.exhaustAdvanceDegrees;
    for (std::size_t mode = 0; mode < 2; ++mode) {
        const auto high = mode != 0;
        const auto intakeDuration = high ? cam.highIntakeDurationDegrees : cam.intakeDurationDegrees;
        const auto exhaustDuration = high ? cam.highExhaustDurationDegrees : cam.exhaustDurationDegrees;
        if (!std::isfinite(intakeDuration) || !std::isfinite(exhaustDuration)
            || !(intakeDuration > 0.0) || !(exhaustDuration > 0.0)
            || intakeDuration > 720.0 || exhaustDuration > 720.0
            || !std::isfinite(intakeCenter) || !std::isfinite(exhaustCenter)) {
            result.valid = result.common = false;
            return result;
        }
        // The physical lift evaluator hard-gates custom samples with this
        // exact support, including the max(1,duration/2) rule. An internal
        // zero plateau is not an opportunity to create a new nominal IVC/EVO.
        const auto withinIntake = std::abs(std::remainder(phase - intakeCenter, 720.0))
            < std::max(1.0, intakeDuration * 0.5);
        const auto withinExhaust = std::abs(std::remainder(phase - exhaustCenter, 720.0))
            < std::max(1.0, exhaustDuration * 0.5);
        result.common = result.common && !withinIntake && !withinExhaust;
    }
    if (!result.common) return result;
    result.liftEvaluated = true;
    for (std::size_t mode = 0; mode < 2; ++mode) {
        const auto high = mode != 0;
        result.actualLiftMm[mode * 2] = profiledValveLiftMm(phase, intakeCenter,
            high ? cam.highIntakeDurationDegrees : cam.intakeDurationDegrees,
            high ? cam.highIntakeLiftMm : cam.intakeLiftMm,
            high ? cam.highIntakeLiftProfile : cam.intakeLiftProfile) * state.liftMultiplier;
        result.actualLiftMm[mode * 2 + 1] = profiledValveLiftMm(phase, exhaustCenter,
            high ? cam.highExhaustDurationDegrees : cam.exhaustDurationDegrees,
            high ? cam.highExhaustLiftMm : cam.exhaustLiftMm,
            high ? cam.highExhaustLiftProfile : cam.exhaustLiftProfile) * state.liftMultiplier;
        result.common = result.common
            && result.actualLiftMm[mode * 2] == 0.0
            && result.actualLiftMm[mode * 2 + 1] == 0.0;
        result.valid = result.valid && std::isfinite(result.actualLiftMm[mode * 2])
            && std::isfinite(result.actualLiftMm[mode * 2 + 1]);
    }
    result.common = result.common && result.valid;
    return result;
}

[[nodiscard]] inline bool commonBaseCircleArc(const CamshaftConfig& cam,
    const ValveTrainState& currentVvt, double previousPhase, double travel) noexcept {
    if (!detail::validPhase(previousPhase) || !std::isfinite(travel)
        || travel < 0.0 || travel >= 720.0) return false;
    const auto outside = [&](double center, double duration) noexcept {
        const auto start = std::remainder(previousPhase - center, 720.0);
        const auto end = start + travel;
        const auto half = std::max(1.0, duration * 0.5);
        for (const auto copy : {-1.0, 0.0, 1.0}) {
            const auto middle = copy * 720.0;
            // crossedPhase counts both distance==0 and distance==travel.
            // A zero-lift support endpoint can still create a new IVC/EVO.
            if (start <= middle + half && end >= middle - half) return false;
        }
        return true;
    };
    const auto intakeCenter = 360.0 + cam.intakeCenterlineDegrees - currentVvt.intakeAdvanceDegrees;
    const auto exhaustCenter = 360.0 - cam.exhaustCenterlineDegrees - currentVvt.exhaustAdvanceDegrees;
    return outside(intakeCenter, cam.intakeDurationDegrees)
        && outside(intakeCenter, cam.highIntakeDurationDegrees)
        && outside(exhaustCenter, cam.exhaustDurationDegrees)
        && outside(exhaustCenter, cam.highExhaustDurationDegrees);
}

class ValveProfileActuator final {
public:
    void reset(bool initialAppliedHigh = false) noexcept {
        applied_ = requested_ = initialAppliedHigh;
        enabled_ = false;
        armed_ = false;
        qualifiedDemand_ = false;
        previous_.reset();
        seenCycle_.reset();
        revision_.reset();
    }
    [[nodiscard]] bool appliedHigh() const noexcept { return applied_; }
    [[nodiscard]] bool requestedHigh() const noexcept { return requested_; }
    [[nodiscard]] bool pending() const noexcept { return armed_; }
    [[nodiscard]] bool qualifiedDemand() const noexcept { return qualifiedDemand_; }

    // Thresholds remain the authored thresholds. A wholly high physical shaft
    // cycle requests high; a wholly low cycle requests low; a mixed cycle holds
    // the existing request. Missing or stopped cycle data falls back to low,
    // canceling an old high request without bypassing the physical actuator.
    void updateDemand(const CamshaftConfig& cam, double throttle,
        const std::optional<ShaftCycleMeasurement>& cycle,
        std::uint64_t camRevision) noexcept {
        if (!cam.variableProfileEnabled) { reset(false); return; }
        enabled_ = true;
        if (revision_ && *revision_ != camRevision) {
            previous_.reset(); armed_ = false; seenCycle_.reset();
            setRequest(false, false);
        }
        revision_ = camRevision;
        if (!std::isfinite(throttle) || !std::isfinite(cam.switchThrottle)
            || !std::isfinite(cam.switchRpm)) {
            setRequest(false, false); return;
        }
        if (throttle < cam.switchThrottle) {
            if (cycle) seenCycle_ = std::array { cycle->epoch, cycle->generation };
            setRequest(false, true); return;
        }
        if (!cycle || cycle->generation == 0 || !(cycle->durationSeconds > 0.0)
            || !std::isfinite(cycle->durationSeconds) || !(cycle->meanRpm > 0.0)
            || !std::isfinite(cycle->meanRpm) || !std::isfinite(cycle->minimumRpm)
            || !std::isfinite(cycle->maximumRpm) || cycle->minimumRpm < 0.0
            || cycle->minimumRpm > cycle->maximumRpm) {
            seenCycle_.reset(); setRequest(false, false); return;
        }
        const auto token = std::array { cycle->epoch, cycle->generation };
        if (seenCycle_ && *seenCycle_ == token) return;
        if (seenCycle_ && (*seenCycle_)[0] != token[0]) setRequest(false, false);
        seenCycle_ = token;
        if (cycle->minimumRpm >= cam.switchRpm) setRequest(true, true);
        else if (cycle->maximumRpm < cam.switchRpm) setRequest(false, true);
        // Otherwise retain the previous request; this is not a numerical or
        // manufacturer-specific hysteresis band.
    }

    // Call after one real VVT advance using the old applied profile. Previous
    // stores its actual earlier VVT snapshot. Revalidating the entire shaft arc
    // against current VVT additionally prevents a shifted IVC/EVO target from
    // becoming newly crossed just because the discrete profile changed.
    [[nodiscard]] bool finish(const CamshaftConfig& cam, const ValveTrainState& currentVvt,
        double currentPhase, std::optional<double> actualTravelSincePrevious,
        std::uint64_t camRevision) noexcept {
        if (!enabled_ || !cam.variableProfileEnabled) return false;
        const auto validSnapshot = detail::validPhase(currentPhase)
            && std::isfinite(currentVvt.intakeAdvanceDegrees)
            && std::isfinite(currentVvt.exhaustAdvanceDegrees)
            && std::isfinite(currentVvt.liftMultiplier) && currentVvt.liftMultiplier >= 0.0;
        const auto snapshot = PhaseSnapshot { validSnapshot, camRevision, currentPhase, currentVvt };
        if (!revision_ || *revision_ != camRevision || !validSnapshot) {
            previous_.reset(); armed_ = false; setRequest(false, false);
            return applied_;
        }
        if (requested_ == applied_) {
            // No remainder, lift evaluation or transcendental work on the
            // stable-profile path. Its actual previous VVT is retained.
            armed_ = false; previous_ = snapshot; return applied_;
        }
        const auto current = commonBaseCircle(cam, currentVvt, currentPhase, camRevision);
        const auto previous = previous_ ? commonBaseCircle(cam, previous_->actualVvtState,
            previous_->phaseDegrees, previous_->camRevision) : BaseCircleWitness {};
        const auto coherent = previous_ && actualTravelSincePrevious
            && std::isfinite(*actualTravelSincePrevious) && *actualTravelSincePrevious >= 0.0
            && *actualTravelSincePrevious < 720.0
            && detail::samePhase(currentPhase,
                previous_->phaseDegrees + *actualTravelSincePrevious);
        const auto safe = coherent && previous.valid && previous.common && current.common
            && previous.camRevision == current.camRevision
            && commonBaseCircleArc(cam, currentVvt, previous_->phaseDegrees,
                *actualTravelSincePrevious);
        if (armed_ && requested_ != applied_ && safe) applied_ = requested_;
        armed_ = requested_ != applied_ && current.common;
        previous_ = snapshot;
        return applied_;
    }

private:
    struct PhaseSnapshot final {
        bool valid {};
        std::uint64_t camRevision {};
        double phaseDegrees {};
        ValveTrainState actualVvtState;
    };
    void setRequest(bool high, bool qualified) noexcept {
        if (requested_ != high) armed_ = false;
        requested_ = high;
        qualifiedDemand_ = qualified;
    }
    bool applied_ {};
    bool requested_ {};
    bool enabled_ {};
    bool armed_ {};
    bool qualifiedDemand_ {};
    std::optional<PhaseSnapshot> previous_;
    std::optional<std::array<std::uint64_t, 2>> seenCycle_;
    std::optional<std::uint64_t> revision_;
};

} // namespace enginelab::cam_controller
