#include <enginelab/physics/EndGasKnockModel.hpp>

#include <algorithm>
#include <cmath>

namespace enginelab {

namespace {
constexpr double universalGasConstant = 8.31446261815324;
}

double EndGasKnockModel::unburnedMolarHeatCapacityCp(double temperatureK,
                                                     double fuelMoleFraction,
                                                     double burnedMoleFraction) noexcept {
    const auto t = std::clamp(temperatureK, 250.0, 1'500.0);
    const auto fuel = std::clamp(fuelMoleFraction, 0.0, 0.2);
    const auto burned = std::clamp(burnedMoleFraction, 0.0, 1.0 - fuel);
    // Straight-line fits over 300-1,200 K to the ideal-gas tables, within 1 %
    // for air (3.50 R at 300 K, 3.98 R at 1,000 K) and the stoichiometric
    // products (N2, CO2, H2O: 3.69 R at 300 K, 4.49 R at 1,200 K); a quadratic
    // for gasoline vapour, taken as iso-octane (189 J/(mol K) at 298 K, ~390
    // at 800 K), within 5 %. The simulator's own gas, linear in temperature
    // over 300-2,600 K, holds air's cv 2 % short of these at 900 K.
    const auto air = (3.305 + 6.58e-4 * t) * universalGasConstant;
    const auto products = (3.44 + 8.36e-4 * t) * universalGasConstant;
    const auto above = t - 298.0;
    const auto vapour = 188.9 + 0.48 * above - 1.5e-4 * above * above;
    return (1.0 - fuel - burned) * air + fuel * vapour + burned * products;
}

double EndGasKnockModel::autoIgnitionDelaySeconds(double pressureBar,
                                                  double unburnedTemperatureK,
                                                  double octaneRating) noexcept {
    // Douaud and Eyzat, fitted on primary reference fuels in a knocking
    // engine: tau = 17.68 ms (ON/100)^3.402 p[atm]^-1.7 exp(3800/T). Its
    // prefactor is the constant users recalibrate to their engine; here it is
    // doubled. At 17.68 ms every catalogue engine with a knock limit held 4-13
    // deg of knock retard behind its own spark map, the NA K20 and LS3
    // included, where a production map sits 0-3 deg from the limit. At 35.4
    // the maps run at 0-5 deg of retard and 10 deg over the map knocks
    // (journal 2026-10-23).
    const auto temperatureK = std::clamp(unburnedTemperatureK, 300.0, 1'500.0);
    const auto pressureAtm = std::clamp(pressureBar, 0.5, 250.0) / 1.01325;
    const auto octane = std::clamp(octaneRating, 60.0, 130.0);
    return std::clamp(35.4e-3 * std::pow(octane / 100.0, 3.402)
        * std::pow(pressureAtm, -1.7) * std::exp(3'800.0 / temperatureK), 1.0e-6, 10.0);
}

EndGasKnockResult EndGasKnockModel::advance(EndGasKnockState& state,
                                             const EndGasConditions& conditions,
                                             double dtSeconds) noexcept {
    EndGasKnockResult result;
    const auto dt = std::isfinite(dtSeconds) ? std::max(0.0, dtSeconds) : 0.0;
    const auto pressureBar = std::max(0.01, conditions.pressureBar);
    if (!conditions.sealed) {
        // An open cylinder has no end gas. The cycle's peak is kept for
        // whoever reads it while the next charge comes in.
        if (state.sealed)
            state.lastCyclePeakUnburnedTemperatureK = state.peakUnburnedTemperatureK;
        state.sealed = false;
        state.inductionIntegral = 0.0;
        state.autoIgnitedThisCycle = false;
        state.compositionFrozen = false;
        state.peakUnburnedTemperatureK = 0.0;
        state.unburnedTemperatureK = conditions.temperatureK;
        state.lastPressureBar = pressureBar;
        state.filteredLevel *= std::exp(-dt * 45.0);
        result.level = state.filteredLevel;
        result.unburnedTemperatureK = state.unburnedTemperatureK;
        return result;
    }

    if (!state.sealed) {
        // The seal: the whole charge is unburned and at the mean temperature.
        state.sealed = true;
        state.inductionIntegral = 0.0;
        state.autoIgnitedThisCycle = false;
        state.compositionFrozen = false;
        state.unburnedTemperatureK = conditions.temperatureK;
        state.lastPressureBar = pressureBar;
        state.peakUnburnedTemperatureK = conditions.temperatureK;
    } else {
        if (!state.compositionFrozen) {
            state.fuelMoleFraction = conditions.fuelMoleFraction;
            state.burnedMoleFraction = conditions.burnedMoleFraction;
            state.compositionFrozen = conditions.combustionActive;
        }
        // dT/T = (R/cp) dp/p, with cp at the zone's own temperature.
        const auto cp = unburnedMolarHeatCapacityCp(state.unburnedTemperatureK,
            state.fuelMoleFraction, state.burnedMoleFraction);
        state.unburnedTemperatureK *= std::exp(universalGasConstant / cp
            * std::log(pressureBar / state.lastPressureBar));
        state.lastPressureBar = pressureBar;
        state.peakUnburnedTemperatureK = std::max(state.peakUnburnedTemperatureK,
            state.unburnedTemperatureK);
    }
    result.unburnedTemperatureK = state.unburnedTemperatureK;
    result.autoIgnitionDelaySeconds = autoIgnitionDelaySeconds(
        pressureBar, state.unburnedTemperatureK, conditions.octaneRating);

    // The integral runs from ignition. Douaud and Eyzat fitted their delay
    // to knock onsets, in the end gas the flame compresses; read below that,
    // its 3,800 K activation credits the compression stroke (500-750 K) with
    // chemistry gasoline does not have there, two-stage as it is. From the
    // seal, the boosted catalogue knocked heavily at its makers' boost.
    if (!conditions.combustionActive)
        state.inductionIntegral = 0.0;
    else if (!state.autoIgnitedThisCycle)
        state.inductionIntegral += dt / result.autoIgnitionDelaySeconds;

    const auto unburnedFraction = std::clamp(1.0 - conditions.burnedFraction, 0.0, 1.0);
    if (state.inductionIntegral >= 1.0 && conditions.combustionActive
            && unburnedFraction > 0.015 && !state.autoIgnitedThisCycle) {
        // Knock: whatever the flame has not reached ignites at once. The
        // pressure that gas adds burning at constant volume is the scale of
        // the oscillation it starts, so it is the intensity, and it grows
        // with how early the end gas went: the unburned share at onset.
        const auto volumeM3 = std::max(1.0e-9, conditions.chamberVolumeM3);
        const auto riseBar = std::max(0.0, conditions.heatCapacityRatio - 1.0)
            * unburnedFraction * std::max(0.0, conditions.chargeFuelEnergyJoules)
            / volumeM3 * 1.0e-5;
        result.autoIgnitedFuelFraction = unburnedFraction;
        result.knockPressureRiseBar = riseBar;
        result.autoIgnited = true;
        state.autoIgnitedThisCycle = true;
        state.lastKnockPressureRiseBar = riseBar;
        state.filteredLevel = std::max(state.filteredLevel,
            std::clamp(riseBar / fullScaleKnockPressureRiseBar, 0.0, 1.0));
    } else {
        state.filteredLevel *= std::exp(-dt * 45.0);
    }
    result.level = state.filteredLevel;
    return result;
}

} // namespace enginelab
