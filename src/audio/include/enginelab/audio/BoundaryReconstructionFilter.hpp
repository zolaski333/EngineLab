#pragma once

#include <algorithm>
#include <cmath>
#include <numbers>

namespace enginelab {

/** Reconstruction low-pass that completes the multirate boundary's sampled-data
 * chain.
 *
 * The exhaust boundary that excites the audio waveguide is produced by sampling
 * the finite-volume network at the multirate coupling cadence and linearly
 * interpolating between those knots. Linear interpolation is a first-order
 * hold: it attenuates the images of the sampled signal by only sinc^2, and its
 * reconstruction error through any curved signal concentrates in lines at
 * exact multiples of the knot rate. Measured on the catalogue, those lines are
 * the isolated 15-25 dB peaks the render harness reports at 1x and 2x each
 * engine's coupling rate -- and because the coupling rate tracks the firing
 * rate below its cap, they rise with rpm, which is audible as a whine.
 *
 * A signal sampled at the coupling rate carries no information above half that
 * rate. Everything this filter removes is therefore guaranteed to be artefact:
 * this is the anti-imaging reconstruction filter required by sampled-data
 * theory, not a voicing choice. Its passband is flat (Butterworth), so it does
 * not shape the physical band it preserves.
 *
 * Topology: two cascaded identical second-order Butterworth sections
 * (Linkwitz-Riley style), -6 dB at the cutoff, 24 dB/octave beyond. With the
 * cutoff at 0.45x the coupling rate this leaves the first image line about
 * 28 dB down and the second about 52 dB down, while the top of the physical
 * band (half the coupling rate) loses about 4 dB.
 *
 * A coupling rate of zero disables the filter and it passes the input through
 * exactly. That is correct, not a fallback: a boundary authored directly at the
 * audio rate -- as the regression fixtures do -- was never sampled, so there is
 * no imaging to remove and nothing to compensate.
 *
 * Coefficients and state are double throughout for the same reason as
 * ValvePortTermination: float recursion here showed limit-cycle-scale
 * cancellation errors near coefficient extremes.
 */
class BoundaryReconstructionFilter final {
public:
    struct Coefficients final {
        double b0 { 1.0 };
        double b1 { 0.0 };
        double b2 { 0.0 };
        double a1 { 0.0 };
        double a2 { 0.0 };
        /** False disables filtering entirely (exact passthrough). */
        bool active { false };
    };

    struct SectionState final {
        double x1 {};
        double x2 {};
        double y1 {};
        double y2 {};
    };

    struct State final {
        SectionState first {};
        SectionState second {};
        /** Primed on first active sample so enabling the filter mid-stream
         * settles to the current input instead of ringing from zero. */
        bool primed { false };

        void reset() noexcept { *this = State {}; }
    };

    /** Shared Linkwitz-Riley crossover frequency.
     *
     * The complementary valve-flow source uses the matching fourth-order high
     * pass. Keeping the frequency policy here guarantees that the two physical
     * bands meet at one crossover rather than drifting through duplicated
     * constants.
     */
    [[nodiscard]] static double crossoverFrequencyHz(
        double couplingFrequencyHz, double sampleRateHz) noexcept {
        if (!(couplingFrequencyHz > 0.0) || !(sampleRateHz > 0.0)
            || !std::isfinite(couplingFrequencyHz) || !std::isfinite(sampleRateHz))
            return 0.0;
        return std::clamp(
            0.45 * couplingFrequencyHz, 150.0, 0.40 * sampleRateHz);
    }

    /** Butterworth section for a cutoff at 0.45x the coupling rate.
     *
     * The 0.45 factor places the corner just below the coupling Nyquist: the
     * image lines start at 1.0x the coupling rate, so the stopband must be
     * established there, while the passband should reach as close to 0.5x as
     * the rolloff allows. The cutoff is clamped away from both DC and the
     * audio Nyquist so the bilinear prewarp stays well conditioned.
     */
    [[nodiscard]] static Coefficients compute(double couplingFrequencyHz,
                                              double sampleRateHz) noexcept {
        Coefficients coefficients;
        const auto cutoffHz = crossoverFrequencyHz(
            couplingFrequencyHz, sampleRateHz);
        if (!(cutoffHz > 0.0)) return coefficients;
        const auto w0 = 2.0 * std::numbers::pi * cutoffHz / sampleRateHz;
        const auto cosW0 = std::cos(w0);
        // sin(w0) / (2 Q) with Q = 1/sqrt(2): a maximally flat section.
        const auto alpha = std::sin(w0) * (0.5 * std::numbers::sqrt2);
        const auto a0 = 1.0 + alpha;
        coefficients.b0 = 0.5 * (1.0 - cosW0) / a0;
        coefficients.b1 = (1.0 - cosW0) / a0;
        coefficients.b2 = coefficients.b0;
        coefficients.a1 = -2.0 * cosW0 / a0;
        coefficients.a2 = (1.0 - alpha) / a0;
        coefficients.active = true;
        return coefficients;
    }

    [[nodiscard]] static double process(const Coefficients& coefficients,
                                        State& state, double input) noexcept {
        if (!coefficients.active) return input;
        if (!state.primed) {
            // Steady-state initialisation: both sections have unity DC gain, so
            // holding every delay element at the input reproduces the input.
            state.first = { input, input, input, input };
            state.second = { input, input, input, input };
            state.primed = true;
        }
        return processSection(coefficients, state.second,
            processSection(coefficients, state.first, input));
    }

private:
    [[nodiscard]] static double processSection(const Coefficients& c,
                                               SectionState& s, double x) noexcept {
        const auto y = c.b0 * x + c.b1 * s.x1 + c.b2 * s.x2
            - c.a1 * s.y1 - c.a2 * s.y2;
        s.x2 = s.x1;
        s.x1 = x;
        s.y2 = s.y1;
        s.y1 = y;
        return y;
    }
};

} // namespace enginelab
