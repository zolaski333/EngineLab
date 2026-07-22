#pragma once

#include <enginelab/events/StructuralExcitationSample.hpp>

#include <array>
#include <cstddef>
#include <cstdint>

namespace enginelab {

/** One thermodynamic-substep snapshot consumed by the realtime audio path. */
struct CylinderPressureSample final {
    double timeSeconds { 0.0 };
    std::array<float, 32> pressureBar {};
    std::array<float, 32> exhaustRunnerPressureKpa {};
    /** Signed instantaneous valve mass flow; positive cylinder -> exhaust.
     *
     * Recomputed from the live cylinder state on every mechanical substep, so
     * it carries valve-plane detail the network state does not. Use it for mass
     * accounting and thermodynamics. It must NOT be paired with the fields below
     * to form an acoustic characteristic: it does not share their cadence.
     */
    std::array<float, 32> exhaustMassFlowKgPerSecond {};
    /** Local runner state defining the linear acoustic characteristic.
     *
     * Every field in this group is reconstructed from the same pair of exhaust
     * network knots with the same interpolation phase, so they describe one
     * consistent instant of one field. A characteristic split p' +- Zc*U' is
     * only meaningful across quantities from this group.
     */
    std::array<float, 32> exhaustPortDensityKgPerM3 {};
    std::array<float, 32> exhaustPortSpeedOfSoundMps {};
    /** Network-consistent port mass flow, in the group above.
     *
     * This is the acoustic partner of exhaustRunnerPressureKpa. It exists
     * separately from exhaustMassFlowKgPerSecond because the two answer
     * different questions: this one is "what flow does the reconstructed network
     * state imply at this instant", which is what the characteristic split
     * needs, while the other is "what flow is the valve actually passing right
     * now", which is what mass balance needs. Pairing the instantaneous flow
     * with an interpolated pressure mixes two different network states; the
     * residual ramps across each coupling interval and resets at every flush,
     * which is a sawtooth clocked at the coupling rate whose harmonics land far
     * above the coupling Nyquist as audible images.
     */
    std::array<float, 32> exhaustAcousticMassFlowKgPerSecond {};
    /** Valve curtain area multiplied by its discharge coefficient. */
    std::array<float, 32> exhaustValveConductanceAreaM2 {};
    std::array<float, 32> exhaustFlowMgPerCycle {};
    /** Normalised exhaust-valve opening used by the acoustic port reflection. */
    std::array<float, 32> exhaustValveOpening {};
    /** Index of the acoustic exhaust path fed by each cylinder. */
    std::array<std::uint8_t, 32> exhaustPathIndex {};
    /** 1 only when all SI thermoacoustic boundary fields above are valid. */
    std::array<std::uint8_t, 32> thermoacousticBoundaryValid {};
    /** Rate at which the acoustic boundary group was actually sampled from the
     * exhaust network, Hz. The renderer's reconstruction filter tracks it to
     * remove imaging above half this rate. Zero means the boundary was authored
     * directly rather than sampled, in which case there is no imaging to remove
     * and the filter stays disabled -- exactness, not a fallback.
     */
    double exhaustCouplingFrequencyHz { 0.0 };
    /** Structure-borne excitation sampled on the same mechanical substep. */
    StructuralExcitationSample structural;
    std::size_t cylinderCount { 0 };
};

} // namespace enginelab
