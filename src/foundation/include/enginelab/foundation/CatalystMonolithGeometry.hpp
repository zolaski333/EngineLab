#pragma once

#include <algorithm>
#include <cmath>

namespace enginelab {

/** Homogenised geometry of a square-channel catalyst substrate.
 *
 * Cell density fixes the square pitch, while the open frontal-area ratio fixes
 * the channel width inside that pitch. The bundle is still represented by one
 * quasi-1D duct: open area sets its total volume-flow admittance and channel
 * width sets its hydraulic diameter. The number of real channels therefore
 * never changes the solver or callback work.
 */
struct CatalystMonolithGeometry final {
    bool active { false };
    double cellPitchM { 0.0 };
    double channelWidthM { 0.0 };
    double openAreaRatio { 1.0 };
    double hydraulicDiameterM { 0.0 };
};

[[nodiscard]] inline CatalystMonolithGeometry catalystMonolithGeometry(
    double cellDensityCpsi, double openAreaRatio) noexcept {
    CatalystMonolithGeometry result;
    if (!std::isfinite(cellDensityCpsi) || cellDensityCpsi <= 0.0
        || !std::isfinite(openAreaRatio) || openAreaRatio <= 0.0
        || openAreaRatio >= 1.0)
        return result;

    constexpr double metresPerInch = 0.0254;
    result.cellPitchM = metresPerInch / std::sqrt(cellDensityCpsi);
    result.channelWidthM = result.cellPitchM * std::sqrt(openAreaRatio);
    if (!std::isfinite(result.cellPitchM)
        || !std::isfinite(result.channelWidthM)
        || result.channelWidthM <= 0.0)
        return {};
    result.active = true;
    result.openAreaRatio = std::clamp(openAreaRatio, 0.0, 1.0);
    // A square channel's hydraulic diameter 4A/P equals its side width.
    result.hydraulicDiameterM = result.channelWidthM;
    return result;
}

} // namespace enginelab
