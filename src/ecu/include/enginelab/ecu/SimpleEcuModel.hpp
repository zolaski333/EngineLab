#pragma once
#include <enginelab/calibration/CalibrationStore.hpp>
#include <enginelab/calibration/EcuCalibrationKeys.hpp>
#include <enginelab/ecu/IEcuModel.hpp>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <memory>
namespace enginelab {
/** Configured ignition curve plus live AFR/advance trims, transient enrichment and latched limiter. */
class SimpleEcuModel final : public IEcuModel {
public:
    SimpleEcuModel();
    explicit SimpleEcuModel(std::shared_ptr<calibration::CalibrationStore> calibrations);

    [[nodiscard]] EcuCommand evaluate(const EngineConfig&, const EngineState&, const EngineControls&) const noexcept override;
    void beginFrame() noexcept override;
    void initialise(const EngineConfig& config) override { initialiseCalibration(config); }
    void initialiseCalibration(const EngineConfig&);
    [[nodiscard]] std::shared_ptr<calibration::CalibrationStore> calibrationStore() const noexcept {
        return calibrations_;
    }
    void reset() noexcept override {
        limiterLatched_.store(false, std::memory_order_relaxed);
        decelerationFuelCutLatched_.store(false, std::memory_order_relaxed);
        decelerationFuelResume_.store(1.0, std::memory_order_relaxed);
        limiterReleaseTime_.store(0.0, std::memory_order_relaxed);
        previousThrottle_.store(0.0, std::memory_order_relaxed);
        accelerationFuelEnrichment_.store(0.0, std::memory_order_relaxed);
        idleIntegral_.store(0.0, std::memory_order_relaxed);
        idleAirOpening_.store(0.0, std::memory_order_relaxed);
        idleFilteredRpm_.store(0.0, std::memory_order_relaxed);
        idleDashpot_.store(0.0, std::memory_order_relaxed);
        previousIdleEvaluationTime_.store(0.0, std::memory_order_relaxed);
        postStartAirOpening_.store(0.0, std::memory_order_relaxed);
        overrunAfterfireArmed_.store(false, std::memory_order_relaxed);
        afterfireOverrunWindow_.store(false, std::memory_order_relaxed);
        afterfirePulseEpochSeconds_.store(0.0, std::memory_order_relaxed);
    }
    /**
     * Read-only view of the idle governor's internal state, for diagnostics.
     *
     * The idle actuator is the sum of a PI governor and a decaying post-start
     * floor, and an idle fault is usually a handoff between the two rather than
     * either one alone. That is invisible from rpm/MAP telemetry, so these
     * expose the three states an idle trace needs. They are observations only;
     * nothing in the control path reads them back.
     */
    [[nodiscard]] double idleIntegral() const noexcept {
        return idleIntegral_.load(std::memory_order_relaxed);
    }
    [[nodiscard]] double postStartAirOpening() const noexcept {
        return postStartAirOpening_.load(std::memory_order_relaxed);
    }
    [[nodiscard]] double idleAirOpening() const noexcept {
        return idleAirOpening_.load(std::memory_order_relaxed);
    }
    [[nodiscard]] bool decelerationFuelCutActive() const noexcept {
        return decelerationFuelCutLatched_.load(std::memory_order_relaxed);
    }
    [[nodiscard]] double decelerationFuelResume() const noexcept {
        return decelerationFuelResume_.load(std::memory_order_relaxed);
    }
    void setAirFuelRatioTrim(double value) noexcept {
        afrTrim_.store(std::isfinite(value)
            ? std::clamp(value,
                calibration::ecuLimits::minimumAirFuelRatioTrim,
                calibration::ecuLimits::maximumAirFuelRatioTrim)
            : 0.0);
    }
    /** Compatibility API: converts the former absolute AFR knob into a trim. */
    void setTargetAirFuelRatio(double value) noexcept { setAirFuelRatioTrim(value - 14.2); }
    void setIgnitionTrimDegrees(double value) noexcept {
        ignitionTrimDegrees_.store(std::isfinite(value) ? std::clamp(value, -30.0, 30.0) : 0.0);
    }
    void setIgnitionAdvanceDegrees(double value) noexcept { setIgnitionTrimDegrees(value); }
private:
    std::shared_ptr<calibration::CalibrationStore> calibrations_;
    std::shared_ptr<calibration::CalibrationReaderEpoch> calibrationReader_;
    std::shared_ptr<const calibration::CalibrationSnapshot> frameCalibration_;
    std::atomic<double> afrTrim_ { 0.0 };
    std::atomic<double> ignitionTrimDegrees_ { 0.0 };
    mutable std::atomic<bool> limiterLatched_ { false };
    mutable std::atomic<bool> decelerationFuelCutLatched_ { false };
    mutable std::atomic<double> decelerationFuelResume_ { 1.0 };
    mutable std::atomic<double> limiterReleaseTime_ { 0.0 };
    mutable std::atomic<double> previousThrottle_ { 0.0 };
    mutable std::atomic<double> accelerationFuelEnrichment_ { 0.0 };
    mutable std::atomic<double> idleIntegral_ { 0.0 };
    mutable std::atomic<double> idleAirOpening_ { 0.0 };
    mutable std::atomic<double> idleFilteredRpm_ { 0.0 };
    mutable std::atomic<double> idleDashpot_ { 0.0 };
    mutable std::atomic<double> previousIdleEvaluationTime_ { 0.0 };
    /** Decaying post-start idle air opening. See evaluate(). */
    mutable std::atomic<double> postStartAirOpening_ { 0.0 };
    /** Latched only after a deliberate high-speed driver power request. */
    mutable std::atomic<bool> overrunAfterfireArmed_ { false };
    /** The pulse scheduler is anchored to the actual lift-off transition, not
     * an arbitrary absolute simulation-time grid. */
    mutable std::atomic<bool> afterfireOverrunWindow_ { false };
    mutable std::atomic<double> afterfirePulseEpochSeconds_ { 0.0 };
};
} // namespace enginelab
