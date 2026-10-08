#pragma once

#include <enginelab/foundation/EngineTypes.hpp>
#include <enginelab/simulation/DynoAbsorberController.hpp>

#include <cstdint>

namespace enginelab {

using DynoAcquisitionMode = DynoMode;

/** Whether a ramp has an acquired bin from which its target can advance. */
[[nodiscard]] constexpr bool dynoRampReferenceReady(
    bool rampPrimed, bool precedingBinAcquired) noexcept {
    return rampPrimed && precedingBinAcquired;
}

enum class DynoQualityReason : std::uint32_t {
    none = 0,
    nonFinite = 1U << 0,
    notPrepared = 1U << 1,
    protocolNotReady = 1U << 2,
    noBrakeContact = 1U << 3,
    insufficientEngineTorque = 1U << 4,
    speedTrackingError = 1U << 5,
    accelerationOutOfBounds = 1U << 6,
    absorberCapacityLimited = 1U << 7,
    revLimiterActive = 1U << 8,
    recoveryActive = 1U << 9,
    discontinuousCycle = 1U << 10,
    /** An unresolved solver step or a rejected conservative inventory transfer. */
    simulationNumericalFault = 1U << 11
};

[[nodiscard]] constexpr DynoQualityReason operator|(
    DynoQualityReason lhs, DynoQualityReason rhs) noexcept {
    return static_cast<DynoQualityReason>(
        static_cast<std::uint32_t>(lhs) | static_cast<std::uint32_t>(rhs));
}

constexpr DynoQualityReason& operator|=(
    DynoQualityReason& lhs, DynoQualityReason rhs) noexcept {
    lhs = lhs | rhs;
    return lhs;
}

[[nodiscard]] constexpr bool hasDynoQualityReason(
    DynoQualityReason mask, DynoQualityReason reason) noexcept {
    return (static_cast<std::uint32_t>(mask)
        & static_cast<std::uint32_t>(reason)) != 0U;
}

struct DynoQualityGateConfig final {
    double minimumSteadyContactFraction { 0.95 };
    /** This acquisition is a brake-dyno measurement, not an inferred inertial
     * pull: material loss of absorber contact invalidates the cycle. */
    double minimumRampContactFraction { 0.95 };
    double minimumEngineTorqueNm { 1.4 };
    double steadyMaximumSpeedErrorRpm { 60.0 };
    double rampMaximumSpeedErrorRpm { 150.0 };
    double steadyMaximumAccelerationRpmPerSecond { 120.0 };
    double rampAccelerationMultiplier { 3.0 };
};

struct DynoQualityGateInput final {
    DynoAcquisitionMode mode { DynoAcquisitionMode::steppedCalibration };
    double targetRpm { 0.0 };
    /** RPM represented by the same observation as measuredCycleTorqueNm. */
    double measuredRpm { 0.0 };
    double rampRateRpmPerSecond { 500.0 };
    /** Authoritative torque from the completed brake-cycle event. */
    double measuredCycleTorqueNm { 0.0 };
    bool prepared { false };
    bool protocolReady { false };
    bool recoveryActive { false };
    bool cycleContinuous { false };
    /** A configured ramp holds its entry or recovery bin. Runtime supplies
     * false until a complete steady window has acquired that bin. */
    bool rampPrimed { true };
};

struct DynoQualityGateResult final {
    DynoQualityReason reasons { DynoQualityReason::none };
    double speedErrorRpm { 0.0 };
    double maximumAllowedSpeedErrorRpm { 0.0 };
    double maximumAllowedAccelerationRpmPerSecond { 0.0 };

    [[nodiscard]] bool accepted() const noexcept {
        return reasons == DynoQualityReason::none;
    }

    /**
     * A rejected observation invalidates the complete cycle containing it.
     * Restoring tracking at the boundary cannot repair an interior excursion.
     */
    [[nodiscard]] bool taintsCycle() const noexcept {
        return !accepted();
    }
};

/**
 * Stateless attribution gate for dyno acquisition.
 *
 * It performs no filtering, allocation, recovery or target mutation. The
 * controller decides what to do with a rejected interval; this class only
 * makes every rejection deterministic and observable.
 */
class DynoQualityGate final {
public:
    explicit DynoQualityGate(DynoQualityGateConfig config = {}) noexcept;

    [[nodiscard]] DynoQualityGateResult evaluate(
        const DynoQualityGateInput& input,
        const EngineState& engineState,
        const DynoAbsorberOutput& absorber) const noexcept;

    [[nodiscard]] const DynoQualityGateConfig& config() const noexcept {
        return config_;
    }

private:
    DynoQualityGateConfig config_ {};
};

} // namespace enginelab
