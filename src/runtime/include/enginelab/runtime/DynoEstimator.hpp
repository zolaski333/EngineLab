#pragma once

#include <enginelab/simulation/IEngineSimulation.hpp>

#include <array>
#include <cstddef>
#include <cstdint>

namespace enginelab {

/** Why a completed brake cycle was or was not admitted to the dyno window. */
enum class DynoCycleAcceptance : std::uint8_t {
    accepted,
    invalidSample,
    nonMonotonicCycleId,
    discontinuousCycleId,
    discontinuousTime
};

/** Readiness of the physically aggregated rolling dyno estimate. */
enum class DynoEstimateQuality : std::uint8_t {
    unavailable,
    warmingUp,
    ready
};

/**
 * A bounded rolling estimate derived from complete 720-degree brake cycles.
 *
 * Torque, speed and power are deliberately recomputed from the conserved
 * integrals. The per-cycle mean fields are input diagnostics, not values to
 * average arithmetically.
 */
struct DynoWindowEstimate final {
    DynoEstimateQuality quality { DynoEstimateQuality::unavailable };
    bool continuous { false };
    bool capacityLimited { false };
    std::uint64_t firstCycleId { 0 };
    std::uint64_t lastCycleId { 0 };
    std::size_t cycleCount { 0 };
    double durationSeconds { 0.0 };
    double brakeWorkJoules { 0.0 };
    double integratedCrankRadians { 0.0 };
    double meanRpm { 0.0 };
    double minRpm { 0.0 };
    double maxRpm { 0.0 };
    double meanTorqueNm { 0.0 };
    double meanPowerKw { 0.0 };
    double torqueVarianceNm2 { 0.0 };
};

struct DynoEstimatorUpdate final {
    DynoCycleAcceptance acceptance { DynoCycleAcceptance::invalidSample };
    DynoWindowEstimate estimate {};
};

/**
 * Allocation-free rolling dyno estimator with a hard 64-cycle work bound.
 *
 * Once the requested duration is covered, the oldest cycle is removed only
 * when the remaining suffix still covers that duration. This retains the
 * shortest causal suffix which satisfies the measurement contract.
 */
class DynoEstimator final {
public:
    static constexpr std::size_t capacity = 64;
    static constexpr double defaultWindowDurationSeconds = 0.25;

    explicit DynoEstimator(double requestedWindowDurationSeconds =
                               defaultWindowDurationSeconds) noexcept;

    /** Clears samples and sequence history, accepting any next cycle ID. */
    void reset() noexcept;

    /**
     * Marks an external protocol/gating discontinuity. No window may bridge
     * the break, and any next valid cycle ID may begin the new sequence.
     */
    void breakContinuity() noexcept;

    [[nodiscard]] DynoEstimatorUpdate push(
        const CompletedBrakeCycleSample& sample) noexcept;

    [[nodiscard]] const DynoWindowEstimate& estimate() const noexcept {
        return estimate_;
    }
    [[nodiscard]] double requestedWindowDurationSeconds() const noexcept {
        return requestedWindowDurationSeconds_;
    }

private:
    struct StoredCycle final {
        std::uint64_t cycleId { 0 };
        double endTimeSeconds { 0.0 };
        double durationSeconds { 0.0 };
        double brakeWorkJoules { 0.0 };
        double crankRadians { 0.0 };
        double rpm { 0.0 };
        double torqueNm { 0.0 };
    };

    [[nodiscard]] bool structurallyValid(
        const CompletedBrakeCycleSample& sample) const noexcept;
    void clearWindow(bool continuous) noexcept;
    void rejectAndAdvanceSequence(
        const CompletedBrakeCycleSample& sample,
        bool advanceCycleId,
        bool advanceTime) noexcept;
    void removeOldest() noexcept;
    void refreshEstimate(bool continuous) noexcept;

    std::array<StoredCycle, capacity> cycles_ {};
    std::size_t head_ { 0 };
    std::size_t cycleCount_ { 0 };
    double totalDurationSeconds_ { 0.0 };
    double totalBrakeWorkJoules_ { 0.0 };
    double totalCrankRadians_ { 0.0 };
    double torqueSquaredAngleSum_ { 0.0 };
    double requestedWindowDurationSeconds_ { defaultWindowDurationSeconds };
    std::uint64_t lastObservedCycleId_ { 0 };
    double lastObservedEndTimeSeconds_ { 0.0 };
    bool hasObservedCycle_ { false };
    bool hasObservedTime_ { false };
    bool capacityLimited_ { false };
    DynoWindowEstimate estimate_ {};
};

} // namespace enginelab
