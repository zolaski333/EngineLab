#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace enginelab {

/** One thermodynamic-substep snapshot consumed by the realtime audio path. */
struct CylinderPressureSample final {
    double timeSeconds { 0.0 };
    std::array<float, 32> pressureBar {};
    std::array<float, 32> exhaustRunnerPressureKpa {};
    std::array<float, 32> exhaustFlowMgPerCycle {};
    /** Normalised exhaust-valve opening used by the acoustic port reflection. */
    std::array<float, 32> exhaustValveOpening {};
    /** Index of the acoustic exhaust path fed by each cylinder. */
    std::array<std::uint8_t, 32> exhaustPathIndex {};
    std::size_t cylinderCount { 0 };
};

} // namespace enginelab
