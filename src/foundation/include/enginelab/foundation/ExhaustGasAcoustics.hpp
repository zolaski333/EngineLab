#pragma once

#include <algorithm>
#include <cmath>

namespace enginelab {

/**
 * Representative acoustic properties of hot exhaust gas.
 *
 * These drive wave-speed estimates: delay-line timing in the realtime audio
 * path and quarter-wave resonance modes in the exhaust topology compiler. They
 * are a single fixed operating point, deliberately distinct from
 * GasCell::heatCapacityRatioEffective(), which tracks per-cell composition and
 * temperature inside the thermodynamic solver.
 *
 * The ratio of specific heats is below cold air's 1.40 because the gas is hot
 * and partly triatomic (CO2/H2O). The specific gas constant is close to air's:
 * combustion products remain dominated by nitrogen, so their mean molar mass is
 * within ~1% of air. Both consumers used to carry their own copies of these two
 * numbers; keeping one definition here stops them drifting apart.
 */
inline constexpr double exhaustHeatCapacityRatio = 1.33;
inline constexpr double exhaustSpecificGasConstantJPerKgK = 287.05;

// Bounds on the gas temperature used for the wave-speed estimate. Pipe gas spans
// roughly a cold start to a sustained-load peak; chamber-peak temperatures are
// deliberately excluded because they never describe the gas filling the runners.
inline constexpr double minimumExhaustGasTemperatureK = 220.0;
inline constexpr double maximumExhaustGasTemperatureK = 2'200.0;
// Fallback used when a non-finite temperature reaches the estimate.
inline constexpr double nominalExhaustGasTemperatureK = 700.0;

/** Isentropic speed of sound of representative exhaust gas at a temperature. */
[[nodiscard]] inline double exhaustSpeedOfSoundMps(double temperatureCelsius) noexcept {
    const auto kelvin = std::isfinite(temperatureCelsius)
        ? std::clamp(temperatureCelsius + 273.15,
                     minimumExhaustGasTemperatureK, maximumExhaustGasTemperatureK)
        : nominalExhaustGasTemperatureK;
    return std::sqrt(exhaustHeatCapacityRatio * exhaustSpecificGasConstantJPerKgK * kelvin);
}

} // namespace enginelab
