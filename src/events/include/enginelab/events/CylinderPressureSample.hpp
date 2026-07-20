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
    /** Signed instantaneous valve mass flow; positive cylinder -> exhaust. */
    std::array<float, 32> exhaustMassFlowKgPerSecond {};
    /** Local runner state defining the linear acoustic characteristic. */
    std::array<float, 32> exhaustPortDensityKgPerM3 {};
    std::array<float, 32> exhaustPortSpeedOfSoundMps {};
    /** Valve curtain area multiplied by its discharge coefficient. */
    std::array<float, 32> exhaustValveConductanceAreaM2 {};
    std::array<float, 32> exhaustFlowMgPerCycle {};
    /** Normalised exhaust-valve opening used by the acoustic port reflection. */
    std::array<float, 32> exhaustValveOpening {};
    /** Index of the acoustic exhaust path fed by each cylinder. */
    std::array<std::uint8_t, 32> exhaustPathIndex {};
    /** 1 only when all SI thermoacoustic boundary fields above are valid. */
    std::array<std::uint8_t, 32> thermoacousticBoundaryValid {};
    std::size_t cylinderCount { 0 };
};

} // namespace enginelab
