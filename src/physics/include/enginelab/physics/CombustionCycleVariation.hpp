#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace enginelab {

/** Deterministic, correlated cycle-to-cycle turbulence dispersion.
 *
 * The authored coefficient is the standard deviation of the multiplicative
 * burn-rate perturbation. A bounded unit-variance innovation avoids impossible
 * negative burn rates, while the AR(1) state gives neighbouring cycles the
 * correlation seen in real cyclic-variability measurements. A zero coefficient
 * is an exact bypass and deliberately does not advance the random stream.
 */
class CombustionCycleVariation final {
public:
    [[nodiscard]] static double advance(
        double coefficientOfVariation, double correlation,
        std::uint32_t& randomState, double& normalisedState) noexcept {
        if (!(coefficientOfVariation > 0.0)) {
            normalisedState = 0.0;
            return 1.0;
        }
        randomState ^= randomState << 13U;
        randomState ^= randomState >> 17U;
        randomState ^= randomState << 5U;
        const auto uniform = static_cast<double>(randomState)
            / static_cast<double>(0xffffffffU);
        // Uniform(-sqrt(3), sqrt(3)) has exactly unit variance.
        const auto innovation = std::sqrt(12.0) * (uniform - 0.5);
        const auto rho = std::clamp(correlation, 0.0, 0.98);
        normalisedState = rho * normalisedState
            + std::sqrt(std::max(0.0, 1.0 - rho * rho)) * innovation;
        return std::clamp(
            1.0 + std::clamp(coefficientOfVariation, 0.0, 0.20)
                * normalisedState,
            0.55, 1.45);
    }
};

} // namespace enginelab
