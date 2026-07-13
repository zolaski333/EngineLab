#pragma once
#include <enginelab/ecu/IEcuModel.hpp>
#include <algorithm>
#include <atomic>
#include <cmath>
namespace enginelab {
/** Configured ignition curve plus live AFR/advance trims, transient enrichment and latched limiter. */
class SimpleEcuModel final : public IEcuModel {
public:
    [[nodiscard]] EcuCommand evaluate(const EngineConfig&, const EngineState&, const EngineControls&) const noexcept override;
    void reset() noexcept override {
        limiterLatched_.store(false, std::memory_order_relaxed);
        limiterReleaseTime_.store(0.0, std::memory_order_relaxed);
        previousThrottle_.store(0.0, std::memory_order_relaxed);
    }
    void setTargetAirFuelRatio(double value) noexcept {
        targetAfr_.store(std::isfinite(value) ? std::clamp(value, 8.0, 30.0) : 14.2);
    }
    void setIgnitionTrimDegrees(double value) noexcept {
        ignitionTrimDegrees_.store(std::isfinite(value) ? std::clamp(value, -30.0, 30.0) : 0.0);
    }
    void setIgnitionAdvanceDegrees(double value) noexcept { setIgnitionTrimDegrees(value); }
private:
    std::atomic<double> targetAfr_ { 14.2 };
    std::atomic<double> ignitionTrimDegrees_ { 0.0 };
    mutable std::atomic<bool> limiterLatched_ { false };
    mutable std::atomic<double> limiterReleaseTime_ { 0.0 };
    mutable std::atomic<double> previousThrottle_ { 0.0 };
};
} // namespace enginelab
