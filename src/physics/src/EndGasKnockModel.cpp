#include <enginelab/physics/EndGasKnockModel.hpp>

#include <algorithm>
#include <cmath>

namespace enginelab {

double EndGasKnockModel::autoIgnitionDelaySeconds(const EndGasConditions& conditions) noexcept {
    const auto temperatureK = std::clamp(conditions.temperatureK, 450.0, 1'500.0);
    const auto pressureBar = std::clamp(conditions.pressureBar, 1.0, 200.0);
    const auto octane = std::clamp(conditions.octaneRating, 60.0, 130.0);
    const auto phi = std::clamp(conditions.equivalenceRatio, 0.45, 1.55);
    const auto temperatureFactor = std::exp(3'800.0 * (1.0 / temperatureK - 1.0 / 850.0));
    const auto pressureFactor = std::pow(20.0 / pressureBar, 1.2);
    const auto octaneFactor = std::pow(octane / 95.0, 2.35);
    const auto mixtureFactor = 1.0 + 2.2 * (phi - 1.0) * (phi - 1.0);
    return std::clamp(0.018 * temperatureFactor * pressureFactor * octaneFactor
        * mixtureFactor, 0.00008, 1.0);
}

EndGasKnockResult EndGasKnockModel::advance(EndGasKnockState& state,
                                             const EndGasConditions& conditions,
                                             double dtSeconds) noexcept {
    EndGasKnockResult result;
    if (!conditions.combustionActive || dtSeconds <= 0.0 || !std::isfinite(dtSeconds)) {
        state.inductionIntegral = 0.0;
        state.filteredLevel *= std::exp(-std::max(0.0, dtSeconds) * 120.0);
        state.autoIgnitedThisCycle = false;
        result.level = state.filteredLevel;
        return result;
    }

    const auto unburnedFraction = std::clamp(1.0 - conditions.burnedFraction, 0.0, 1.0);
    result.autoIgnitionDelaySeconds = autoIgnitionDelaySeconds(conditions);
    if (unburnedFraction > 0.015 && !state.autoIgnitedThisCycle)
        state.inductionIntegral += dtSeconds / result.autoIgnitionDelaySeconds;
    else
        state.inductionIntegral = std::max(0.0, state.inductionIntegral - dtSeconds * 80.0);

    if (state.inductionIntegral >= 1.0 && unburnedFraction > 0.015
            && !state.autoIgnitedThisCycle) {
        const auto thermalSeverity = std::clamp((conditions.temperatureK - 820.0) / 330.0, 0.0, 1.0);
        const auto pressureSeverity = std::clamp((conditions.pressureBar - 18.0) / 82.0, 0.0, 1.0);
        const auto severity = std::clamp(0.18 + thermalSeverity * 0.45
            + pressureSeverity * 0.37, 0.0, 1.0);
        result.autoIgnitedFuelFraction = unburnedFraction
            * std::clamp(0.22 + severity * 0.68, 0.0, 0.95);
        result.autoIgnited = true;
        state.autoIgnitedThisCycle = true;
        state.filteredLevel = std::max(state.filteredLevel,
            std::clamp(severity * std::sqrt(unburnedFraction), 0.0, 1.0));
    } else {
        state.filteredLevel *= std::exp(-dtSeconds * 45.0);
    }
    result.level = state.filteredLevel;
    return result;
}

} // namespace enginelab
