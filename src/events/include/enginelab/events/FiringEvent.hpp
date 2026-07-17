#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace enginelab {

inline constexpr std::size_t maximumExhaustEventComponents = 4;

/** Immutable payload crossing from simulation to realtime audio. */
struct FiringEvent final {
    double timeSeconds { 0.0 };
    std::uint32_t cylinderId { 0 };
    double crankAngleDegrees { 0.0 };
    float intensity { 0.0F };
    float pressureEstimateBar { 0.0F };
    float combustionDurationMs { 0.0F };
    float airFuelRatio { 14.7F };
    float ignitionAdvanceDegrees { 0.0F };
    float knockAmount { 0.0F };
    float stereoPosition { 0.0F };
    float exhaustFlowMgPerCycle { 0.0F };
    float exhaustRunnerPressureKpa { 101.325F };
    float exhaustDelaySeconds { 0.0F };
    float exhaustResonanceHz { 0.0F };
    bool misfire { false };
    std::uint32_t exhaustPortId { 0 };
    std::uint32_t intakePortId { 0 };
    std::uint32_t exhaustPathIndex { 0 };
    // Acoustic transmission from the exhaust topology only. Keeping this
    // separate from intensity prevents a muffler/tailpipe edit from changing
    // the direct in-cylinder combustion layer rendered from the same event.
    float exhaustTransmissionGain { 1.0F };
    // Strongest route/mode components retained by the exhaust DAG compiler.
    // Each component has its own causal arrival, energy-derived amplitude,
    // resonance and physical output path. The scalar fields above remain the
    // dominant-route compatibility view for older renderers.
    std::uint8_t exhaustComponentCount { 0 };
    std::array<float, maximumExhaustEventComponents> exhaustComponentDelaySeconds {};
    std::array<float, maximumExhaustEventComponents> exhaustComponentGain {};
    std::array<float, maximumExhaustEventComponents> exhaustComponentResonanceHz {};
    std::array<std::uint8_t, maximumExhaustEventComponents> exhaustComponentPathIndex {};
};

} // namespace enginelab
