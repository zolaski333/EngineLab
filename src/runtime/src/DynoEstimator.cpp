#include <enginelab/runtime/DynoEstimator.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace enginelab {
namespace {
double secondsConsistencyTolerance(double lhs, double rhs) noexcept {
    // Ten nanoseconds at ordinary simulation times: wide enough for a
    // subtraction of two accumulated doubles, but far below any useful dyno
    // interval or controller discontinuity.
    return 1.0e-8 * std::max({ 1.0, std::abs(lhs), std::abs(rhs) });
}

bool sameTime(double lhs, double rhs) noexcept {
    return std::abs(lhs - rhs) <= secondsConsistencyTolerance(lhs, rhs);
}
} // namespace

DynoEstimator::DynoEstimator(double requestedWindowDurationSeconds) noexcept
    : requestedWindowDurationSeconds_(
          std::isfinite(requestedWindowDurationSeconds)
              && requestedWindowDurationSeconds > 0.0
          ? requestedWindowDurationSeconds
          : defaultWindowDurationSeconds) {}

void DynoEstimator::reset() noexcept {
    hasObservedCycle_ = false;
    hasObservedTime_ = false;
    lastObservedCycleId_ = 0;
    lastObservedEndTimeSeconds_ = 0.0;
    clearWindow(false);
}

void DynoEstimator::breakContinuity() noexcept {
    // A protocol break is also a new sequence boundary: engine replacement and
    // simulator reset may legitimately restart the cycle counter.
    hasObservedCycle_ = false;
    hasObservedTime_ = false;
    clearWindow(false);
}

bool DynoEstimator::structurallyValid(
    const CompletedBrakeCycleSample& sample) const noexcept {
    if (!sample.numericallyValid
        || !std::isfinite(sample.startTimeSeconds)
        || !std::isfinite(sample.endTimeSeconds)
        || !std::isfinite(sample.durationSeconds)
        || !std::isfinite(sample.integratedCrankRadians)
        || !std::isfinite(sample.brakeWorkJoules)
        || !std::isfinite(sample.meanRpm)
        || !std::isfinite(sample.meanTorqueNm)
        || !std::isfinite(sample.meanPowerKw)
        || !(sample.durationSeconds > 0.0)
        || !(sample.integratedCrankRadians > 0.0)
        || !(sample.endTimeSeconds > sample.startTimeSeconds))
        return false;

    return sameTime(sample.endTimeSeconds - sample.startTimeSeconds,
                    sample.durationSeconds);
}

void DynoEstimator::clearWindow(bool continuous) noexcept {
    head_ = 0;
    cycleCount_ = 0;
    totalDurationSeconds_ = 0.0;
    totalBrakeWorkJoules_ = 0.0;
    totalCrankRadians_ = 0.0;
    torqueSquaredAngleSum_ = 0.0;
    capacityLimited_ = false;
    estimate_ = {};
    estimate_.continuous = continuous;
}

void DynoEstimator::rejectAndAdvanceSequence(
    const CompletedBrakeCycleSample& sample,
    bool advanceCycleId,
    bool advanceTime) noexcept {
    clearWindow(false);
    if (advanceCycleId
        && (!hasObservedCycle_ || sample.cycleId > lastObservedCycleId_)) {
        lastObservedCycleId_ = sample.cycleId;
        hasObservedCycle_ = true;
    }
    if (advanceTime && std::isfinite(sample.endTimeSeconds)) {
        lastObservedEndTimeSeconds_ = sample.endTimeSeconds;
        hasObservedTime_ = true;
    }
}

void DynoEstimator::removeOldest() noexcept {
    const auto& oldest = cycles_[head_];
    totalDurationSeconds_ -= oldest.durationSeconds;
    totalBrakeWorkJoules_ -= oldest.brakeWorkJoules;
    totalCrankRadians_ -= oldest.crankRadians;
    torqueSquaredAngleSum_ -=
        oldest.torqueNm * oldest.torqueNm * oldest.crankRadians;
    head_ = (head_ + 1) % capacity;
    --cycleCount_;

    // Subtraction of like-sized floating values can leave a tiny negative
    // residue. Exact zero is preferable when the ring becomes empty.
    if (cycleCount_ == 0) {
        totalDurationSeconds_ = 0.0;
        totalBrakeWorkJoules_ = 0.0;
        totalCrankRadians_ = 0.0;
        torqueSquaredAngleSum_ = 0.0;
    }
}

void DynoEstimator::refreshEstimate(bool continuous) noexcept {
    estimate_ = {};
    estimate_.continuous = continuous;
    estimate_.capacityLimited = capacityLimited_;
    if (cycleCount_ == 0) return;

    estimate_.firstCycleId = cycles_[head_].cycleId;
    estimate_.lastCycleId =
        cycles_[(head_ + cycleCount_ - 1) % capacity].cycleId;
    estimate_.cycleCount = cycleCount_;
    estimate_.durationSeconds = totalDurationSeconds_;
    estimate_.brakeWorkJoules = totalBrakeWorkJoules_;
    estimate_.integratedCrankRadians = totalCrankRadians_;
    estimate_.quality = totalDurationSeconds_ >= requestedWindowDurationSeconds_
        ? DynoEstimateQuality::ready : DynoEstimateQuality::warmingUp;

    if (!(totalDurationSeconds_ > 0.0)
        || !(totalCrankRadians_ > 0.0)) {
        estimate_.quality = DynoEstimateQuality::unavailable;
        estimate_.continuous = false;
        return;
    }

    constexpr auto radiansPerRevolution = 2.0 * std::numbers::pi;
    estimate_.meanRpm = totalCrankRadians_ / totalDurationSeconds_
        * 60.0 / radiansPerRevolution;
    estimate_.meanTorqueNm =
        totalBrakeWorkJoules_ / totalCrankRadians_;
    estimate_.meanPowerKw =
        totalBrakeWorkJoules_ / totalDurationSeconds_ / 1'000.0;

    // The conserved sums above are maintained in O(1). A bounded scan is less
    // state and less numerical risk than monotonic min/max queues here: even at
    // the hard cap it examines only 64 values per completed engine cycle.
    auto minRpm = std::numeric_limits<double>::infinity();
    auto maxRpm = -std::numeric_limits<double>::infinity();
    for (std::size_t offset = 0; offset < cycleCount_; ++offset) {
        const auto rpm = cycles_[(head_ + offset) % capacity].rpm;
        minRpm = std::min(minRpm, rpm);
        maxRpm = std::max(maxRpm, rpm);
    }
    estimate_.minRpm = minRpm;
    estimate_.maxRpm = maxRpm;

    const auto secondMoment =
        torqueSquaredAngleSum_ / totalCrankRadians_;
    estimate_.torqueVarianceNm2 = std::max(0.0,
        secondMoment - estimate_.meanTorqueNm * estimate_.meanTorqueNm);
}

DynoEstimatorUpdate DynoEstimator::push(
    const CompletedBrakeCycleSample& sample) noexcept {
    if (!structurallyValid(sample)) {
        const auto monotonic = !hasObservedCycle_
            || sample.cycleId > lastObservedCycleId_;
        rejectAndAdvanceSequence(sample, monotonic,
                                 std::isfinite(sample.endTimeSeconds));
        return { DynoCycleAcceptance::invalidSample, estimate_ };
    }

    if (hasObservedCycle_) {
        if (sample.cycleId <= lastObservedCycleId_) {
            rejectAndAdvanceSequence(sample, false, false);
            return { DynoCycleAcceptance::nonMonotonicCycleId, estimate_ };
        }
        if (sample.cycleId - lastObservedCycleId_ != 1) {
            rejectAndAdvanceSequence(sample, true, true);
            return { DynoCycleAcceptance::discontinuousCycleId, estimate_ };
        }
    }
    if (hasObservedTime_
        && !sameTime(sample.startTimeSeconds,
                     lastObservedEndTimeSeconds_)) {
        rejectAndAdvanceSequence(sample, true, true);
        return { DynoCycleAcceptance::discontinuousTime, estimate_ };
    }

    lastObservedCycleId_ = sample.cycleId;
    lastObservedEndTimeSeconds_ = sample.endTimeSeconds;
    hasObservedCycle_ = true;
    hasObservedTime_ = true;

    capacityLimited_ = false;
    if (cycleCount_ == capacity) {
        removeOldest();
        capacityLimited_ = true;
    }

    const auto insertion = (head_ + cycleCount_) % capacity;
    auto& stored = cycles_[insertion];
    stored.cycleId = sample.cycleId;
    stored.endTimeSeconds = sample.endTimeSeconds;
    stored.durationSeconds = sample.durationSeconds;
    stored.brakeWorkJoules = sample.brakeWorkJoules;
    stored.crankRadians = sample.integratedCrankRadians;
    stored.rpm = sample.integratedCrankRadians / sample.durationSeconds
        * 60.0 / (2.0 * std::numbers::pi);
    stored.torqueNm =
        sample.brakeWorkJoules / sample.integratedCrankRadians;
    ++cycleCount_;
    totalDurationSeconds_ += stored.durationSeconds;
    totalBrakeWorkJoules_ += stored.brakeWorkJoules;
    totalCrankRadians_ += stored.crankRadians;
    torqueSquaredAngleSum_ +=
        stored.torqueNm * stored.torqueNm * stored.crankRadians;

    while (cycleCount_ > 1) {
        const auto& oldest = cycles_[head_];
        if (totalDurationSeconds_ - oldest.durationSeconds
            < requestedWindowDurationSeconds_)
            break;
        removeOldest();
    }

    refreshEstimate(true);
    return { DynoCycleAcceptance::accepted, estimate_ };
}

} // namespace enginelab
