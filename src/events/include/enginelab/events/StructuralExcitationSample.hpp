#pragma once

#include <array>
#include <cstddef>

namespace enginelab {

/** Instantaneous forces transmitted from the cylinders to the engine structure.
 *
 * Every entry is evaluated on the mechanical solver substep. Gas and inertia
 * are kept separate so a head mode can be driven by chamber loading while a
 * block/bearing mode can use their signed resultant without reconstructing it
 * from audio-rate heuristics.
 */
struct StructuralExcitationSample final {
    std::array<float, 32> gasForceN {};
    std::array<float, 32> inertiaForceN {};
    std::array<float, 32> bearingReactionForceN {};
    std::array<float, 32> sideThrustForceN {};
    std::array<float, 32> crankReactionTorqueNm {};
    std::size_t cylinderCount { 0 };
};

} // namespace enginelab
