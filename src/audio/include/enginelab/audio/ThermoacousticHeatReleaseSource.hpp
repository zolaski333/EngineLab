#pragma once

#include <enginelab/audio/BoundaryReconstructionFilter.hpp>

#include <cmath>
#include <numbers>

namespace enginelab {

/** Converts conservative exhaust heat release into a compact acoustic source.
 *
 * For a compact heat source in a uniform duct, integrating the linearised
 * energy equation across the source gives the characteristic-pressure jump
 *
 *     Delta p = (gamma - 1) Qdot / (A c).
 *
 * AcousticExhaustNetwork splits that jump equally into the two travelling
 * characteristics. Each outgoing wave therefore has the textbook amplitude
 * (gamma - 1) Qdot / (2 A c). No gain or recorded pop is involved.
 *
 * Qdot arrives at the finite-volume coupling cadence. The same anti-imaging
 * reconstruction used for physical port boundaries removes information above
 * that cadence's Nyquist region. A 25 Hz two-pole DC blocker then removes only
 * the quasi-steady heat/mean-flow component already owned by the finite-volume
 * solver. It leaves the audible pressure front of an 8-10 ms reaction intact,
 * instead of the former 3-6 kHz high-pass that removed virtually all of it.
 */
class ThermoacousticHeatReleaseSource final {
public:
    static constexpr double exhaustGammaMinusOne = 0.34;
    static constexpr double dcBlockFrequencyHz = 25.0;

    struct Coefficients final {
        BoundaryReconstructionFilter::Coefficients reconstruction {};
        double dcBlockPole { 0.0 };
        bool valid { false };
    };

    struct State final {
        BoundaryReconstructionFilter::State reconstruction {};
        double previousInput1 {};
        double highPass1 {};
        double previousInput2 {};
        double highPass2 {};

        void reset() noexcept { *this = State {}; }
    };

    [[nodiscard]] static Coefficients compute(
        double couplingFrequencyHz, double sampleRateHz) noexcept {
        Coefficients coefficients;
        if (!(sampleRateHz > 2.0 * dcBlockFrequencyHz)
            || !std::isfinite(sampleRateHz))
            return coefficients;
        coefficients.reconstruction = BoundaryReconstructionFilter::compute(
            couplingFrequencyHz, sampleRateHz);
        coefficients.dcBlockPole = std::exp(
            -2.0 * std::numbers::pi * dcBlockFrequencyHz / sampleRateHz);
        coefficients.valid = std::isfinite(coefficients.dcBlockPole)
            && coefficients.dcBlockPole > 0.0
            && coefficients.dcBlockPole < 1.0;
        return coefficients;
    }

    [[nodiscard]] static double compactPressureJumpPa(
        double releasedPowerW, double flowAreaM2,
        double speedOfSoundMps) noexcept {
        if (!(releasedPowerW >= 0.0) || !(flowAreaM2 > 0.0)
            || !(speedOfSoundMps > 0.0)
            || !std::isfinite(releasedPowerW)
            || !std::isfinite(flowAreaM2)
            || !std::isfinite(speedOfSoundMps))
            return 0.0;
        return exhaustGammaMinusOne * releasedPowerW
            / (flowAreaM2 * speedOfSoundMps);
    }

    [[nodiscard]] static float process(
        const Coefficients& coefficients, State& state,
        double releasedPowerW, double flowAreaM2,
        double speedOfSoundMps) noexcept {
        if (!coefficients.valid) {
            state.reset();
            return 0.0F;
        }
        const auto sourcePressurePa = compactPressureJumpPa(
            releasedPowerW, flowAreaM2, speedOfSoundMps);

        // BoundaryReconstructionFilter normally primes to the first input to
        // avoid a startup transient in an already-running boundary. A reaction
        // voice is born from silence, so zero is its correct initial history.
        if (!state.reconstruction.primed)
            state.reconstruction.primed = true;
        const auto reconstructedPressurePa =
            BoundaryReconstructionFilter::process(
                coefficients.reconstruction, state.reconstruction,
                sourcePressurePa);
        state.highPass1 = coefficients.dcBlockPole
            * (state.highPass1 + reconstructedPressurePa
                - state.previousInput1);
        state.previousInput1 = reconstructedPressurePa;
        state.highPass2 = coefficients.dcBlockPole
            * (state.highPass2 + state.highPass1 - state.previousInput2);
        state.previousInput2 = state.highPass1;
        if (!std::isfinite(state.highPass2)) {
            state.reset();
            return 0.0F;
        }
        return static_cast<float>(state.highPass2);
    }
};

} // namespace enginelab
