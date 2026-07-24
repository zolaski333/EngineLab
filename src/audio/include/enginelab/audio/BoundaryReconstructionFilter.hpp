#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
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
 * Topology: two cascaded fourth-order Butterworth filters, which is an
 * eighth-order Linkwitz-Riley low-pass -- four biquads, -6 dB at the cutoff and
 * 48 dB/octave beyond.
 *
 * This was a fourth-order Linkwitz-Riley at 0.45x the coupling rate, which left
 * the first image line only 28 dB down. That is not enough: the render harness
 * defines an audibly metallic resonance as one standing 20 dB proud of its
 * spectral neighbourhood, and in the quiet top of the band a 28 dB image clears
 * that easily. Measured on the reference inline four it stood 30 dB proud at
 * 4107 Hz, and raising the coupling rate with the offline oracle moved it, which
 * is what identifies it as an image rather than a duct mode.
 *
 * Doubling the order and moving the corner from 0.45x to 0.47x of the coupling
 * rate is better in both directions at once, because a steeper filter can afford
 * a corner closer to the band edge:
 *
 *                        0.25x coupling   0.5x coupling   1.0x coupling
 *     LR4 at 0.45x           -0.79 dB        -8.0 dB         -28.1 dB
 *     LR8 at 0.47x           -0.06 dB        -8.4 dB         -52.5 dB
 *
 * So the resolved band is thirteen times less attenuated where the physics
 * actually lives, the band edge is unchanged to within half a dB, and the first
 * image is 24 dB further down. The cost is two more biquads per boundary in a
 * callback that measures at 15-33% of its budget.
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
    /** Quality factors of a fourth-order Butterworth, 1/(2 cos(pi(2k+1)/8)).
     *  The Linkwitz-Riley cascade runs this pair twice. */
    static constexpr std::array<double, 2> butterworthQ { 0.541196100146197,
                                                          1.306562964876377 };
    /** Four biquads: the Butterworth pair above, applied twice. */
    static constexpr std::size_t sectionCount = 2 * butterworthQ.size();

    struct Biquad final {
        double b0 { 1.0 };
        double b1 { 0.0 };
        double b2 { 0.0 };
        double a1 { 0.0 };
        double a2 { 0.0 };
    };

    struct Coefficients final {
        std::array<Biquad, butterworthQ.size()> stage {};
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
        std::array<SectionState, sectionCount> sections {};
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
            crossoverFraction * couplingFrequencyHz, 150.0, 0.40 * sampleRateHz);
    }

    /** Corner as a fraction of the coupling rate.
     *
     * The image lines start at 1.0x the coupling rate, so the stopband has to be
     * established there, while the passband should reach as close to 0.5x as the
     * rolloff allows. An eighth-order transition affords a corner nearer the
     * band edge than a fourth-order one did, which is why this is 0.47 and not
     * the 0.45 that went with the shallower filter.
     */
    static constexpr double crossoverFraction = 0.47;

    /** The four Butterworth biquads for that corner.
     *
     * The cutoff is clamped away from both DC and the audio Nyquist by
     * crossoverFrequencyHz so the bilinear prewarp stays well conditioned.
     */
    [[nodiscard]] static Coefficients compute(double couplingFrequencyHz,
                                              double sampleRateHz) noexcept {
        Coefficients coefficients;
        const auto cutoffHz = crossoverFrequencyHz(
            couplingFrequencyHz, sampleRateHz);
        if (!(cutoffHz > 0.0)) return coefficients;
        const auto w0 = 2.0 * std::numbers::pi * cutoffHz / sampleRateHz;
        const auto cosW0 = std::cos(w0);
        const auto sinW0 = std::sin(w0);
        for (std::size_t index = 0; index < butterworthQ.size(); ++index) {
            const auto alpha = sinW0 / (2.0 * butterworthQ[index]);
            const auto a0 = 1.0 + alpha;
            auto& stage = coefficients.stage[index];
            stage.b0 = 0.5 * (1.0 - cosW0) / a0;
            stage.b1 = (1.0 - cosW0) / a0;
            stage.b2 = stage.b0;
            stage.a1 = -2.0 * cosW0 / a0;
            stage.a2 = (1.0 - alpha) / a0;
        }
        coefficients.active = true;
        return coefficients;
    }

    [[nodiscard]] static double process(const Coefficients& coefficients,
                                        State& state, double input) noexcept {
        if (!coefficients.active) return input;
        if (!state.primed) {
            // Steady-state initialisation: every section has unity DC gain, so
            // holding all delay elements at the input reproduces the input.
            for (auto& section : state.sections)
                section = { input, input, input, input };
            state.primed = true;
        }
        auto signal = input;
        for (std::size_t index = 0; index < sectionCount; ++index)
            signal = processSection(coefficients.stage[index % butterworthQ.size()],
                                    state.sections[index], signal);
        return signal;
    }

private:
    [[nodiscard]] static double processSection(const Biquad& c,
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
