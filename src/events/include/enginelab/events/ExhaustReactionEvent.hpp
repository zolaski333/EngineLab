#pragma once

#include <cstdint>

namespace enginelab {

/** Spatially resolved observation of conservative exhaust heat release.
 *
 * It never owns or invents energy. `releasedEnergyJoules` is copied from the
 * finite-volume species/energy update and lets the acoustic solver reconstruct
 * only the high-band pressure complement at the same physical node.
 */
struct ExhaustReactionEvent final {
    double timeSeconds { 0.0 };
    std::uint32_t nodeId { 0 };
    std::uint32_t sourceComponentId { 0 };
    std::uint32_t pathIndex { 0 };
    float axialPosition { 0.5F };
    float releasedEnergyJoules { 0.0F };
    float burnedFuelMassKg { 0.0F };
    float durationSeconds { 0.0F };
    float densityKgPerM3 { 0.0F };
    float speedOfSoundMps { 0.0F };
    float flowAreaM2 { 0.0F };
};

} // namespace enginelab
