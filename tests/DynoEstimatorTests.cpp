#include <enginelab/runtime/DynoEstimator.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <numbers>
#include <string_view>

namespace {
void require(bool condition, std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

void requireNear(double actual, double expected, double tolerance,
                 std::string_view message) {
    if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance) {
        std::cerr << "FAIL: " << message << " (actual=" << actual
                  << ", expected=" << expected << ")\n";
        std::exit(1);
    }
}

enginelab::CompletedBrakeCycleSample cycle(std::uint64_t id,
                                           double startSeconds,
                                           double rpm,
                                           double torqueNm) {
    constexpr auto crankRadians = 4.0 * std::numbers::pi;
    const auto duration = crankRadians
        / (rpm * 2.0 * std::numbers::pi / 60.0);
    const auto work = torqueNm * crankRadians;
    return {
        .cycleId = id,
        .startTimeSeconds = startSeconds,
        .endTimeSeconds = startSeconds + duration,
        .durationSeconds = duration,
        .integratedCrankRadians = crankRadians,
        .brakeWorkJoules = work,
        .meanRpm = rpm,
        .meanTorqueNm = torqueNm,
        .meanPowerKw = work / duration / 1'000.0,
        .numericallyValid = true
    };
}
} // namespace

int main() {
    using enginelab::DynoCycleAcceptance;
    using enginelab::DynoEstimateQuality;

    // Conserved-integral oracle: unlike an arithmetic average of RPM or power,
    // these values remain correct when consecutive cycles have unequal times.
    {
        enginelab::DynoEstimator estimator(0.15);
        const auto first = cycle(10, 0.0, 1'000.0, 100.0);
        const auto second = cycle(11, first.endTimeSeconds, 2'000.0, 200.0);
        require(estimator.push(first).acceptance
                    == DynoCycleAcceptance::accepted,
                "the first complete cycle must be accepted");
        const auto update = estimator.push(second);
        require(update.acceptance == DynoCycleAcceptance::accepted,
                "a contiguous complete cycle must be accepted");
        const auto& value = update.estimate;
        require(value.quality == DynoEstimateQuality::ready
                    && value.continuous && value.cycleCount == 2
                    && value.firstCycleId == 10 && value.lastCycleId == 11,
                "a covered contiguous window must be ready");
        requireNear(value.meanTorqueNm, 150.0, 1.0e-10,
                    "torque must equal total work divided by total angle");
        requireNear(value.meanRpm, 4'000.0 / 3.0, 1.0e-10,
                    "RPM must equal total angle divided by total time");
        requireNear(value.meanPowerKw,
                    value.brakeWorkJoules / value.durationSeconds / 1'000.0,
                    1.0e-12,
                    "power must equal total work divided by total time");
        requireNear(value.minRpm, 1'000.0, 1.0e-10,
                    "minimum cycle RPM must be retained");
        requireNear(value.maxRpm, 2'000.0, 1.0e-10,
                    "maximum cycle RPM must be retained");
        requireNear(value.torqueVarianceNm2, 2'500.0, 1.0e-9,
                    "torque variance must be crank-angle weighted");
    }

    // The window is the shortest causal suffix which still covers the target.
    {
        enginelab::DynoEstimator estimator(0.10);
        auto sample = cycle(1, 0.0, 1'000.0, 100.0); // 0.12 s
        auto update = estimator.push(sample);
        require(update.estimate.cycleCount == 1
                    && update.estimate.quality == DynoEstimateQuality::ready,
                "one slow cycle may cover the requested time");
        sample = cycle(2, sample.endTimeSeconds, 2'000.0, 200.0); // 0.06 s
        update = estimator.push(sample);
        require(update.estimate.cycleCount == 2
                    && update.estimate.firstCycleId == 1,
                "the oldest cycle must stay while the suffix is too short");
        sample = cycle(3, sample.endTimeSeconds, 2'000.0, 300.0);
        update = estimator.push(sample);
        require(update.estimate.cycleCount == 2
                    && update.estimate.firstCycleId == 2
                    && update.estimate.lastCycleId == 3,
                "the oldest cycle must leave once the suffix covers the window");
        requireNear(update.estimate.meanTorqueNm, 250.0, 1.0e-10,
                    "evicted cycles must leave every aggregate");
    }

    // Compare every update against a deliberately simple O(N) suffix oracle.
    {
        constexpr double targetSeconds = 0.19;
        enginelab::DynoEstimator estimator(targetSeconds);
        std::array<enginelab::CompletedBrakeCycleSample, 40> samples {};
        double time = 0.0;
        for (std::size_t index = 0; index < samples.size(); ++index) {
            samples[index] = cycle(index + 100, time,
                900.0 + 43.0 * static_cast<double>(index),
                75.0 + 7.0 * static_cast<double>(index % 9));
            time = samples[index].endTimeSeconds;
            const auto update = estimator.push(samples[index]);
            require(update.acceptance == DynoCycleAcceptance::accepted,
                    "oracle sequence must remain contiguous");

            std::size_t first = index;
            double suffixDuration = samples[index].durationSeconds;
            while (first > 0
                   && suffixDuration < targetSeconds) {
                --first;
                suffixDuration += samples[first].durationSeconds;
            }
            // Match the estimator's strict minimal-suffix rule when one large
            // old cycle can be removed while the newer cycles still cover.
            while (first < index
                   && suffixDuration - samples[first].durationSeconds
                        >= targetSeconds) {
                suffixDuration -= samples[first].durationSeconds;
                ++first;
            }

            double work = 0.0;
            double angle = 0.0;
            double duration = 0.0;
            double torqueSecondMoment = 0.0;
            double minRpm = samples[first].meanRpm;
            double maxRpm = minRpm;
            for (auto oracleIndex = first;
                 oracleIndex <= index; ++oracleIndex) {
                const auto& item = samples[oracleIndex];
                work += item.brakeWorkJoules;
                angle += item.integratedCrankRadians;
                duration += item.durationSeconds;
                const auto torque = item.brakeWorkJoules
                    / item.integratedCrankRadians;
                torqueSecondMoment += torque * torque
                    * item.integratedCrankRadians;
                minRpm = std::min(minRpm, item.meanRpm);
                maxRpm = std::max(maxRpm, item.meanRpm);
            }
            const auto& value = update.estimate;
            const auto oracleTorque = work / angle;
            require(value.cycleCount == index - first + 1,
                    "ring membership must match the O(N) oracle");
            requireNear(value.durationSeconds, duration, 1.0e-12,
                        "ring duration must match the O(N) oracle");
            requireNear(value.meanTorqueNm, oracleTorque, 1.0e-10,
                        "ring torque must match the O(N) oracle");
            requireNear(value.meanRpm,
                        angle / duration * 60.0
                            / (2.0 * std::numbers::pi),
                        1.0e-9,
                        "ring RPM must match the O(N) oracle");
            requireNear(value.torqueVarianceNm2,
                        torqueSecondMoment / angle
                            - oracleTorque * oracleTorque,
                        1.0e-8,
                        "ring variance must match the O(N) oracle");
            requireNear(value.minRpm, minRpm, 1.0e-9,
                        "ring minimum must match the O(N) oracle");
            requireNear(value.maxRpm, maxRpm, 1.0e-9,
                        "ring maximum must match the O(N) oracle");
        }
    }

    // A duration larger than 64 cycles proves the hard cap and exposes that
    // the requested statistical window could not be retained in full.
    {
        enginelab::DynoEstimator estimator(100.0);
        double time = 0.0;
        enginelab::DynoEstimatorUpdate update;
        for (std::uint64_t id = 1; id <= 80; ++id) {
            const auto sample = cycle(id, time, 6'000.0, 120.0);
            time = sample.endTimeSeconds;
            update = estimator.push(sample);
        }
        require(update.estimate.cycleCount == enginelab::DynoEstimator::capacity
                    && update.estimate.firstCycleId == 17
                    && update.estimate.lastCycleId == 80
                    && update.estimate.capacityLimited
                    && update.estimate.quality == DynoEstimateQuality::warmingUp,
                "the estimator must remain bounded and report a truncated window");
    }

    // Invalid, duplicate, skipped and time-discontinuous cycles cannot leak
    // their integrals into a later estimate.
    {
        enginelab::DynoEstimator estimator(0.01);
        auto first = cycle(1, 0.0, 2'000.0, 100.0);
        require(estimator.push(first).acceptance
                    == DynoCycleAcceptance::accepted,
                "setup cycle must be accepted");

        auto invalid = cycle(2, first.endTimeSeconds, 2'000.0, 900.0);
        invalid.numericallyValid = false;
        auto update = estimator.push(invalid);
        require(update.acceptance == DynoCycleAcceptance::invalidSample
                    && update.estimate.cycleCount == 0
                    && !update.estimate.continuous,
                "an invalid cycle must break and clear the estimate");

        auto third = cycle(3, invalid.endTimeSeconds, 2'000.0, 110.0);
        update = estimator.push(third);
        require(update.acceptance == DynoCycleAcceptance::accepted
                    && update.estimate.cycleCount == 1,
                "the next cycle after a rejected invalid cycle may restart");
        requireNear(update.estimate.meanTorqueNm, 110.0, 1.0e-10,
                    "a rejected cycle must not contaminate restarted sums");

        update = estimator.push(third);
        require(update.acceptance == DynoCycleAcceptance::nonMonotonicCycleId
                    && update.estimate.cycleCount == 0,
                "a duplicate ID must be rejected and break continuity");

        auto skipped = cycle(5, third.endTimeSeconds, 2'000.0, 500.0);
        update = estimator.push(skipped);
        require(update.acceptance == DynoCycleAcceptance::discontinuousCycleId
                    && update.estimate.cycleCount == 0,
                "a skipped ID must be rejected and break continuity");

        auto sixth = cycle(6, skipped.endTimeSeconds, 2'000.0, 130.0);
        update = estimator.push(sixth);
        require(update.acceptance == DynoCycleAcceptance::accepted
                    && update.estimate.cycleCount == 1,
                "a contiguous cycle after a rejected gap may restart");

        auto wrongTime = cycle(7, sixth.endTimeSeconds + 0.01,
                               2'000.0, 700.0);
        update = estimator.push(wrongTime);
        require(update.acceptance == DynoCycleAcceptance::discontinuousTime
                    && update.estimate.cycleCount == 0,
                "an unexplained timestamp gap must be rejected");

        estimator.breakContinuity();
        auto restarted = cycle(1, 20.0, 3'000.0, 140.0);
        update = estimator.push(restarted);
        require(update.acceptance == DynoCycleAcceptance::accepted
                    && update.estimate.firstCycleId == 1,
                "an explicit break must permit a new cycle-ID epoch");
        estimator.reset();
        require(estimator.estimate().quality
                    == DynoEstimateQuality::unavailable
                    && estimator.estimate().cycleCount == 0
                    && !estimator.estimate().continuous,
                "reset must clear the estimator contract");
    }

    std::cout << "PASS: bounded cycle dyno estimator and conserved-integral oracle\n";
    return 0;
}
