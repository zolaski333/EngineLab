#pragma once

#include <algorithm>
#include <cmath>

namespace enginelab {

/**
 * One physically measured trapped-air charge and the intake-source state that
 * produced it at intake-valve closure.
 */
struct TrappedChargeReference final {
    double freshAirMassMg { 0.0 };
    double sourcePressureKpa { 0.0 };
    double sourceTemperatureK { 0.0 };
};

/**
 * Predicts the fresh-air inventory of the next port-injected charge.
 *
 * Port fuel must be metered before the next intake valve has finished filling
 * the cylinder. Waiting for oxygen already present in the chamber therefore
 * introduces one-cycle lag during manifold transients. This estimator retains
 * the last measured cylinder filling (including its real volumetric efficiency
 * and wave dynamics) and scales it only by the change in intake-source density:
 *
 *     m_next = m_last * (p_next / T_next) / (p_last / T_last)
 *
 * This is the ideal-gas speed-density relation, not a calibration multiplier.
 * The resolved chamber oxygen inventory remains a lower bound, and invalid
 * reference data falls back to that directly resolved value.
 */
class TransientChargeEstimator final {
public:
    [[nodiscard]] static double estimateFreshAirMassMg(
        double resolvedFreshAirMassMg,
        const TrappedChargeReference& previous,
        double currentSourcePressureKpa,
        double currentSourceTemperatureK) noexcept {
        const auto resolved = std::isfinite(resolvedFreshAirMassMg)
            ? std::max(0.0, resolvedFreshAirMassMg) : 0.0;
        if (!(previous.freshAirMassMg > 0.0)
            || !(previous.sourcePressureKpa > 0.0)
            || !(previous.sourceTemperatureK > 0.0)
            || !(currentSourcePressureKpa > 0.0)
            || !(currentSourceTemperatureK > 0.0)
            || !std::isfinite(previous.freshAirMassMg)
            || !std::isfinite(previous.sourcePressureKpa)
            || !std::isfinite(previous.sourceTemperatureK)
            || !std::isfinite(currentSourcePressureKpa)
            || !std::isfinite(currentSourceTemperatureK))
            return resolved;

        const auto densityRatio = currentSourcePressureKpa
            * previous.sourceTemperatureK
            / (previous.sourcePressureKpa * currentSourceTemperatureK);
        const auto predicted = previous.freshAirMassMg * densityRatio;
        // The resolved-oxygen floor is deliberately double-edged, and it must
        // stay paired with symmetric FUEL accounting at the metering site.
        // After a lean or misfired cycle the chamber keeps its unburned air;
        // flooring the request on that oxygen is a real anti-stall enrichment
        // (removing it made two turbo idles dip toward stall). But the same
        // retained charge keeps its unburned FUEL, and if the metering only
        // counts that fuel while the intake valve is open, the floor doubles
        // the request exactly when the engine is flooded — measured on the
        // V12 at idle catch: 106 mg requested for a ~500 mg charge, an AFR-3
        // misfire spiral that stalled it. The pairing lives in
        // EngineSimulator's `trappedCylinderFuel`: after a misfire the
        // chamber's fuel counts against the request even with the valve shut.
        return std::isfinite(predicted) && predicted >= 0.0
            ? std::max(resolved, predicted) : resolved;
    }
};

} // namespace enginelab
