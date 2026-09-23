#pragma once

#include <enginelab/audio/BoundaryReconstructionFilter.hpp>
#include <enginelab/audio/ThermoacousticHeatReleaseSource.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>

namespace enginelab {

/** Generic high band of an exhaust reaction: the crack of a pop.
 *
 * Why it exists. ThermoacousticHeatReleaseSource turns the conservative heat
 * release of the exhaust chemistry into the compact pressure jump
 * (gamma - 1) Qdot / (A c), but Qdot is only known at the finite-volume
 * coupling cadence (at most 8 kHz) and the anti-imaging reconstruction keeps
 * nothing above 0.47x that rate. Measured 2026-09-23 (docs/journal.md): pop
 * energy centroid 450-790 Hz, 0.02-0.5 % of it between 4 and 8 kHz, none above
 * 8 kHz. That band is not an artefact waiting to be recovered: it was never
 * computed, because a 1-D network of 95-300 mm cells does not resolve a
 * turbulent flame.
 *
 * What it does. VISION.md's rule for this: physics decides when and how much, a
 * generic synthesis supplies the band physics cannot carry. The source runs only
 * while a reaction voice carries heat release, and its amplitude is that
 * voice's own compact pressure jump:
 *
 *     p_crack(t) = crackRatio * e(t) * n(t)
 *
 * e(t) follows the jump with a 0.1 ms attack and 0.5 ms release. n(t) is
 * unit-RMS white noise high-passed at the reconstruction crossover (fourth-order
 * Linkwitz-Riley, i.e. the complementary band) and tilted -6 dB/octave above it,
 * the slope of a step front's spectrum. The noise is a per-voice xorshift
 * sequence, so offline renders are bit-reproducible.
 *
 * What is not measured. crackRatio is one generic number, never a per-engine
 * parameter. It is NOT calibrated against a real pop, which requires a
 * recording; see defaultCrackRatio for the bracket that was measured.
 */
class ReactionCrackSynthesiser final {
public:
    /** RMS of the crack relative to the voice's compact pressure jump.
     *  Bracketed 2026-09-23 on 2JZ pops (1/3-octave power, physical band one
     *  octave below the crossover vs crack one octave above): continuing the
     *  simulator's own slope asks for 0.8, continuing an ideal step front for
     *  2.5. 1.0 sits at the conservative end: +3.9 dB in 4-8 kHz during pops,
     *  still 10 dB under the engine above 8 kHz, nothing below 4 kHz. */
    static constexpr double defaultCrackRatio = 1.0;
    static constexpr double envelopeAttackSeconds = 0.0001;
    static constexpr double envelopeReleaseSeconds = 0.0005;
    /** Impulse-response length used to normalise the noise shaping to unit
     *  RMS. The slowest pole is the tilt at >= 150 Hz (1.1 ms); 2048 samples
     *  is 5 of its time constants even at 192 kHz. Recomputed only when the
     *  coupling cadence moves by more than 1 %, like the physical source. */
    static constexpr std::size_t normalisationSamples = 2048;

    struct Biquad final {
        double b0 { 1.0 };
        double b1 { 0.0 };
        double b2 { 0.0 };
        double a1 { 0.0 };
        double a2 { 0.0 };
    };

    struct Coefficients final {
        std::array<Biquad, 2> highPass {};
        double tiltPole { 0.0 };
        double attackCoefficient { 0.0 };
        double releaseCoefficient { 0.0 };
        double noiseNormalisation { 0.0 };
        bool valid { false };
    };

    struct SectionState final {
        double x1 {};
        double x2 {};
        double y1 {};
        double y2 {};
    };

    struct State final {
        std::array<SectionState, 2> highPass {};
        double tilt {};
        double envelopePa {};
        std::uint32_t noise { 0x9e3779b9U };

        void reset(std::uint32_t seed) noexcept {
            *this = State {};
            noise = seed != 0U ? seed : 0x9e3779b9U;
        }
    };

    [[nodiscard]] static Coefficients compute(
        double couplingFrequencyHz, double sampleRateHz) noexcept {
        Coefficients coefficients;
        const auto crossoverHz = BoundaryReconstructionFilter::crossoverFrequencyHz(
            couplingFrequencyHz, sampleRateHz);
        if (!(crossoverHz > 0.0) || !(sampleRateHz > 0.0)) return coefficients;

        // Two identical second-order Butterworth high-passes: LR4.
        const auto w0 = 2.0 * std::numbers::pi * crossoverHz / sampleRateHz;
        const auto cosW0 = std::cos(w0);
        constexpr double butterworthQ = 1.0 / std::numbers::sqrt2;
        const auto alpha = std::sin(w0) / (2.0 * butterworthQ);
        const auto a0 = 1.0 + alpha;
        Biquad section;
        section.b0 = 0.5 * (1.0 + cosW0) / a0;
        section.b1 = -(1.0 + cosW0) / a0;
        section.b2 = section.b0;
        section.a1 = -2.0 * cosW0 / a0;
        section.a2 = (1.0 - alpha) / a0;
        coefficients.highPass = { section, section };
        coefficients.tiltPole = std::exp(-w0);
        coefficients.attackCoefficient = 1.0 - std::exp(
            -1.0 / (envelopeAttackSeconds * sampleRateHz));
        coefficients.releaseCoefficient = 1.0 - std::exp(
            -1.0 / (envelopeReleaseSeconds * sampleRateHz));

        // White noise of unit variance through a linear filter has variance
        // sum(h^2); measure it once so n(t) leaves the shaping at unit RMS.
        State probe;
        auto energy = 0.0;
        for (std::size_t index = 0; index < normalisationSamples; ++index) {
            const auto response = shape(
                coefficients, probe, index == 0 ? 1.0 : 0.0);
            energy += response * response;
        }
        if (!(energy > 0.0) || !std::isfinite(energy)) return coefficients;
        coefficients.noiseNormalisation = 1.0 / std::sqrt(energy);
        coefficients.valid = true;
        return coefficients;
    }

    /** One audio sample of crack pressure, Pa, for a voice currently releasing
     *  `releasedPowerW`. Returns exactly zero once the voice is silent. */
    [[nodiscard]] static float process(
        const Coefficients& coefficients, State& state,
        double releasedPowerW, double flowAreaM2, double speedOfSoundMps,
        double crackRatio) noexcept {
        if (!coefficients.valid || !(crackRatio > 0.0)) return 0.0F;
        const auto jumpPa = ThermoacousticHeatReleaseSource::compactPressureJumpPa(
            releasedPowerW, flowAreaM2, speedOfSoundMps);
        const auto rate = jumpPa > state.envelopePa
            ? coefficients.attackCoefficient
            : coefficients.releaseCoefficient;
        state.envelopePa += rate * (jumpPa - state.envelopePa);
        if (state.envelopePa < 1.0e-6) {
            state.envelopePa = 0.0;
            return 0.0F;
        }
        const auto shaped = shape(coefficients, state, nextUniform(state.noise))
            * coefficients.noiseNormalisation;
        const auto pressurePa = crackRatio * state.envelopePa * shaped;
        return std::isfinite(pressurePa) ? static_cast<float>(pressurePa) : 0.0F;
    }

private:
    /** Uniform white noise of unit variance (range +-sqrt(3)). */
    [[nodiscard]] static double nextUniform(std::uint32_t& stateWord) noexcept {
        auto x = stateWord;
        x ^= x << 13U;
        x ^= x >> 17U;
        x ^= x << 5U;
        stateWord = x;
        const auto unit = static_cast<double>(x) / 4294967296.0;
        return (2.0 * unit - 1.0) * std::numbers::sqrt3;
    }

    [[nodiscard]] static double shape(
        const Coefficients& coefficients, State& state, double input) noexcept {
        auto signal = input;
        for (std::size_t index = 0; index < state.highPass.size(); ++index) {
            const auto& c = coefficients.highPass[index];
            auto& s = state.highPass[index];
            const auto y = c.b0 * signal + c.b1 * s.x1 + c.b2 * s.x2
                - c.a1 * s.y1 - c.a2 * s.y2;
            s.x2 = s.x1;
            s.x1 = signal;
            s.y2 = s.y1;
            s.y1 = y;
            signal = y;
        }
        state.tilt = (1.0 - coefficients.tiltPole) * signal
            + coefficients.tiltPole * state.tilt;
        return state.tilt;
    }
};

} // namespace enginelab
