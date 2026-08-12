#pragma once

#include <enginelab/events/ExhaustReactionEvent.hpp>

#include <array>
#include <cstddef>
#include <cstdint>

namespace enginelab {

/** One sparse exhaust-network coupling observation for realtime acoustics.
 *
 * Duct media, terminal flow and chemical heat release only change when the
 * finite-volume network advances. Keeping them out of CylinderPressureSample
 * avoids copying several kilobytes on every thermodynamic substep merely to
 * repeat the previous coupling state.
 */
struct ExhaustAcousticSample final {
    static constexpr std::size_t maximumDucts { 64 };
    static constexpr std::size_t maximumOutlets { 32 };
    static constexpr std::size_t maximumReactionEvents { 32 };

    double timeSeconds { 0.0 };
    double couplingFrequencyHz { 0.0 };

    std::array<float, maximumDucts> ductDensityKgPerM3 {};
    std::array<float, maximumDucts> ductSpeedOfSoundMps {};
    std::size_t ductCount { 0 };

    std::array<std::uint32_t, maximumOutlets> outletNodeId {};
    std::array<std::uint8_t, maximumOutlets> outletPathIndex {};
    std::array<float, maximumOutlets> outletMassFlowKgPerSecond {};
    std::array<float, maximumOutlets> outletDensityKgPerM3 {};
    std::array<float, maximumOutlets> outletSpeedOfSoundMps {};
    std::array<float, maximumOutlets> outletAreaM2 {};
    std::size_t outletCount { 0 };

    std::array<ExhaustReactionEvent, maximumReactionEvents> reactionEvents {};
    std::size_t reactionEventCount { 0 };
    std::size_t droppedReactionEventCount { 0 };
};

} // namespace enginelab
