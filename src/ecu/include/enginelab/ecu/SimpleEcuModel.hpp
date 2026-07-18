#pragma once
#include <enginelab/calibration/CalibrationStore.hpp>
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
        idleDashpot_.store(0.0, std::memory_order_relaxed);
        previousIdleEvaluationTime_.store(0.0, std::memory_order_relaxed);
    }
    void setAirFuelRatioTrim(double value) noexcept {
        afrTrim_.store(std::isfinite(value) ? std::clamp(value, -3.0, 3.0) : 0.0);
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
    mutable std::atomic<double> idleDashpot_ { 0.0 };
    mutable std::atomic<double> previousIdleEvaluationTime_ { 0.0 };
};
} // namespace enginelab
