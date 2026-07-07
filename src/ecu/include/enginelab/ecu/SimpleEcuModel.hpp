#pragma once
#include <enginelab/ecu/IEcuModel.hpp>
#include <algorithm>
#include <atomic>
#include <cmath>
namespace enginelab {
/** Initial scalar ECU; its atomic parameters will later be replaced by interpolated RPM/load maps. */
class SimpleEcuModel final : public IEcuModel {
public:
    [[nodiscard]] EcuCommand evaluate(const EngineConfig&, const EngineState&, const EngineControls&) const noexcept override;
    void reset() noexcept override {
        limiterLatched_.store(false, std::memory_order_relaxed);
        previousThrottle_.store(0.0, std::memory_order_relaxed);
    }
    void setTargetAirFuelRatio(double value) noexcept {
        targetAfr_.store(std::isfinite(value) ? std::clamp(value, 8.0, 30.0) : 14.2);
    }
    void setIgnitionAdvanceDegrees(double value) noexcept {
        ignitionAdvance_.store(std::isfinite(value) ? std::clamp(value, -30.0, 80.0) : 18.0);
    }
private:
    std::atomic<double> targetAfr_ { 14.2 };
    std::atomic<double> ignitionAdvance_ { 18.0 };
    mutable std::atomic<bool> limiterLatched_ { false };
    mutable std::atomic<double> previousThrottle_ { 0.0 };
};
} // namespace enginelab
