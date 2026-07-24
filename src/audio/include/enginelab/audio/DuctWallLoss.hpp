#pragma once

#include <algorithm>
#include <cmath>

namespace enginelab {

/**
 * Thermoviscous attenuation of a plane wave travelling one length of duct.
 *
 * The exhaust waveguides propagated sound through pure delay lines. A pure delay
 * is lossless, so the only damping anywhere in the network came from the
 * radiation load at the mouth -- and that load reflects almost perfectly at low
 * frequency. The result was a duct whose modes decayed only through the small
 * fraction of energy that escaped the outlet, giving a comb with tens of dB
 * between its peaks and valleys, standing well above the engine's own firing
 * harmonics. That is what a listener hears as metallic.
 *
 * Real ducts attenuate. Against the wall, viscosity drags the acoustic particle
 * velocity to zero and conduction drags the acoustic temperature to the wall
 * temperature; both losses live in thin boundary layers and both scale as the
 * square root of frequency. The Kirchhoff-Rayleigh result for the pressure
 * attenuation coefficient of the plane mode in a circular duct is
 *
 *     alpha(f) = sqrt(pi f nu) / (r c) * (1 + (gamma - 1) / sqrt(Pr))
 *
 * in nepers per metre, with r the duct radius, nu the kinematic viscosity, Pr
 * the Prandtl number and gamma the specific heat ratio. Over a duct of length L
 * the pressure amplitude is multiplied by exp(-alpha(f) L).
 *
 * Note that L = c * tau for a line of delay tau, so c cancels and the loss over
 * one traversal depends on the delay and the radius but not directly on the
 * speed of sound:
 *
 *     alpha(f) L = sqrt(pi f nu) * (1 + (gamma - 1)/sqrt(Pr)) * tau / r
 *
 * This is a real and dominant loss mechanism in a small-bore hot duct, and it is
 * frequency dependent, so it damps the high modes that sound metallic far harder
 * than the firing fundamental that carries the engine's character. It is not a
 * tone control: nothing here is tuned to taste, and the only free quantities are
 * the duct geometry and the gas state, both of which the network already knows.
 *
 * ---------------------------------------------------------------------------
 * Realisation
 *
 * exp(-k sqrt(f)) is not rational, so it has to be approximated. The shape to
 * hit is very gentle: over the whole audio band a catalogue duct loses between
 * 0.2 and 2.5 dB in total, and the slope of the target in dB per octave is
 * A*ln2/2 where A is the attenuation already accumulated. A one-pole lowpass
 * cannot follow that. Its slope runs away to 6 dB/octave, so matching it to the
 * exact value at a single reference frequency buys accuracy there and loses it
 * everywhere above: measured against the law it implements, a one-pole fitted at
 * 1 kHz over-attenuated the delivered LS3 duct chain by 1.0 dB at 2 kHz, 4.2 dB
 * at 4 kHz and 11.1 dB at 8 kHz, per one-way traversal, and the network is
 * bidirectional. Worse, the fitted pole landed in 0.31-0.67 for every duct in
 * the catalogue, so the top of the band was shaped by the filter's own corner
 * rather than by duct geometry -- flattening exactly the differences between
 * engines that the geometry exists to create.
 *
 * A first-order shelf -- one pole and one zero -- has the missing degree of
 * freedom: the zero arrests the roll-off, so the response can leave DC at 0 dB,
 * tilt gently across the band and arrive at a finite attenuation at Nyquist,
 * which is the shape of sqrt(f) over a bounded band. Fitting it is exact and
 * cheap because of the following identity. For
 *
 *     H(z) = g (1 - z z^-1) / (1 - p z^-1),  g = (1-p)/(1-z)  (unit DC gain),
 *
 * the squared magnitude collapses to a Moebius function of s = sin^2(w/2):
 *
 *     |H(w)|^2 = (1 + Z s) / (1 + P s),   Z = 4z/(1-z)^2,  P = 4p/(1-p)^2.
 *
 * Matching two target gains is therefore a 2x2 *linear* system in (Z, P), and
 * the warped parameters invert in closed form, z = (sqrt(1+Z) - 1)^2 / Z. No
 * iteration, no search, and the passivity condition is simply 0 <= Z <= P.
 *
 * Fitted at 2.5 kHz and 15 kHz, the shelf tracks the exact Kirchhoff law to
 * within 0.35 dB from 80 Hz to Nyquist for every duct in the catalogue, against
 * 11.9 dB worst case for the one-pole it replaces.
 *
 * Limits. The shelf is flat to second order at DC where the target has infinite
 * slope, so it under-attenuates below ~200 Hz -- bounded by 0.13 dB, because the
 * target itself is near zero there.
 *
 * The fit exists only while the duct is gentle enough for a first-order tilt to
 * span it. Past roughly 9.4 dB of loss per traversal at the lower design point
 * the sqrt(f) law is steeper than 6 dB/octave through the upper band, the solve
 * returns a zero outside the pole, and no passive first-order shelf reaches both
 * design points. fit() detects that and falls back to a pure pole matched at the
 * lower design frequency: still passive, still monotone, still exact where the
 * audible content of such a duct actually is, and approximate across a top end
 * that is already more than 17 dB down. The narrowest, longest duct in the
 * catalogue sits at 7% of that threshold, so the fallback is a guard on the
 * graph editor's clamped extremes, not a path the delivered engines take.
 *
 * The model accounts only for the boundary-layer term. A real exhaust also loses
 * high-frequency energy to wall compliance, to higher-order mode conversion
 * above the plane-mode cutoff, and to the mean flow's turbulent shear, so the
 * true attenuation is higher than this everywhere. Reproducing the Kirchhoff law
 * accurately is therefore a floor on the damping, not a ceiling.
 */
class DuctWallLoss final {
public:
    /** The two frequencies at which the shelf matches the exact Kirchhoff
     *  attenuation. They bracket the band the network can carry: the lower one
     *  sits above the coupling Nyquist of every catalogue engine, where the
     *  audible mode structure lives, and the upper one near the top of the
     *  reconstructed band. Both are clamped down for low sample rates by fit(). */
    static constexpr double lowerDesignFrequencyHz = 2'500.0;
    static constexpr double upperDesignFrequencyHz = 15'000.0;

    /** Gas properties. Defaults describe hot exhaust rather than ambient air. */
    static constexpr double prandtlNumber = 0.72;
    static constexpr double specificHeatRatio = 1.35;
    static constexpr double gasConstantJPerKgK = 287.0;

    /** Sutherland's law for dynamic viscosity, in Pa*s. */
    [[nodiscard]] static double dynamicViscosityPaS(double temperatureK) noexcept {
        if (!(std::isfinite(temperatureK)) || temperatureK <= 1.0) return 1.716e-5;
        constexpr double referenceViscosity = 1.716e-5;
        constexpr double referenceTemperature = 273.15;
        constexpr double sutherland = 110.4;
        return referenceViscosity
            * std::pow(temperatureK / referenceTemperature, 1.5)
            * (referenceTemperature + sutherland) / (temperatureK + sutherland);
    }

    /** Static temperature implied by a speed of sound, in K. */
    [[nodiscard]] static double temperatureFromSoundSpeedK(double soundSpeedMps) noexcept {
        return std::isfinite(soundSpeedMps) && soundSpeedMps > 0.0
            ? soundSpeedMps * soundSpeedMps
                / (specificHeatRatio * gasConstantJPerKgK)
            : 293.15;
    }

    /**
     * Exact Kirchhoff pressure gain over one traversal, at one frequency.
     *
     * @param frequencyHz     frequency of interest.
     * @param traversalSeconds  one-way propagation time through the duct.
     * @param radiusM         duct radius.
     * @param densityKgPerM3  gas density.
     * @param soundSpeedMps   gas speed of sound (sets the temperature).
     */
    [[nodiscard]] static double traversalGain(double frequencyHz,
                                              double traversalSeconds,
                                              double radiusM,
                                              double densityKgPerM3,
                                              double soundSpeedMps) noexcept {
        if (!(frequencyHz > 0.0) || !(traversalSeconds > 0.0) || !(radiusM > 0.0)
            || !(densityKgPerM3 > 0.0) || !(soundSpeedMps > 0.0))
            return 1.0;
        const auto temperature = temperatureFromSoundSpeedK(soundSpeedMps);
        const auto kinematicViscosity = dynamicViscosityPaS(temperature) / densityKgPerM3;
        constexpr double pi = 3.14159265358979323846;
        const auto boundaryLayerTerm = std::sqrt(pi * frequencyHz * kinematicViscosity);
        const auto thermalTerm = 1.0
            + (specificHeatRatio - 1.0) / std::sqrt(prandtlNumber);
        const auto exponent = boundaryLayerTerm * thermalTerm * traversalSeconds / radiusM;
        return std::isfinite(exponent) ? std::exp(-exponent) : 1.0;
    }

    /** Shelf state: one input and one output word per traversal direction. */
    struct State final {
        float previousInput { 0.0F };
        float previousOutput { 0.0F };
        void reset() noexcept {
            previousInput = 0.0F;
            previousOutput = 0.0F;
        }
    };

    /** y[n] = g (x[n] - z x[n-1]) + p y[n-1], with g fixed by unit DC gain. */
    struct Coefficients final {
        float pole { 0.0F };
        float zero { 0.0F };
        float gain { 1.0F };

        /** Restore g = (1-p)/(1-z) after pole or zero has been moved.
         *  Callers that slew the coefficients must call this, otherwise the
         *  duct develops a DC gain during the slew. */
        void renormalise() noexcept {
            const auto denominator = 1.0F - zero;
            gain = denominator > 1.0e-6F ? (1.0F - pole) / denominator : 1.0F;
        }
    };

    /** Largest pole or zero the shelf will place, keeping g well conditioned. */
    static constexpr double maximumCoefficient = 0.9999;

    /**
     * Solve W = 4a/(1-a)^2 for the coefficient a in [0, 1).
     *
     * Completing the square gives a = (sqrt(1+W) - 1)^2 / W directly, which is
     * also numerically the better of the two algebraic forms: it is a ratio of
     * quantities that stay well scaled as W approaches zero, where the other
     * root form cancels catastrophically.
     */
    [[nodiscard]] static double coefficientFromWarped(double warped) noexcept {
        if (!std::isfinite(warped) || warped <= 0.0) return 0.0;
        const auto root = std::sqrt(1.0 + warped) - 1.0;
        const auto coefficient = root * root / warped;
        return std::isfinite(coefficient)
            ? std::clamp(coefficient, 0.0, maximumCoefficient) : 0.0;
    }

    /**
     * Fit the shelf to the exact Kirchhoff gain at the two design frequencies.
     *
     * With s = sin^2(w/2) and the warped parameters Z, P of the class comment,
     * |H|^2 = (1 + Z s)/(1 + P s), so each design point contributes the linear
     * equation  s Z - G^2 s P = G^2 - 1  and the pair solves by Cramer's rule.
     */
    [[nodiscard]] static Coefficients fit(double traversalSeconds,
                                          double radiusM,
                                          double densityKgPerM3,
                                          double soundSpeedMps,
                                          double sampleRateHz) noexcept {
        Coefficients c {};
        if (!(sampleRateHz > 0.0)) return c;
        const auto upperHz = std::min(upperDesignFrequencyHz, sampleRateHz * 0.45);
        const auto lowerHz = std::min(lowerDesignFrequencyHz, upperHz * 0.4);
        const auto gainAt = [&](double frequencyHz) {
            return traversalGain(frequencyHz, traversalSeconds, radiusM,
                                 densityKgPerM3, soundSpeedMps);
        };
        const auto lowerGain = gainAt(lowerHz);
        const auto upperGain = gainAt(upperHz);
        // Lossless, or too close to lossless for the fit to be conditioned.
        if (!(lowerGain > 0.0) || lowerGain >= 1.0) return c;

        constexpr double pi = 3.14159265358979323846;
        const auto warpOf = [&](double frequencyHz) {
            const auto halfOmega = pi * frequencyHz / sampleRateHz;
            const auto sine = std::sin(halfOmega);
            return sine * sine;
        };
        const auto lowerS = warpOf(lowerHz);
        const auto upperS = warpOf(upperHz);
        const auto lowerG2 = lowerGain * lowerGain;
        const auto upperG2 = upperGain * upperGain;
        const auto determinant = lowerS * upperS * (lowerG2 - upperG2);
        if (determinant > 1.0e-30) {
            const auto zeroWarp = (lowerG2 * lowerS * (upperG2 - 1.0)
                - (lowerG2 - 1.0) * upperG2 * upperS) / determinant;
            const auto poleWarp = (lowerS * (upperG2 - 1.0)
                - upperS * (lowerG2 - 1.0)) / determinant;
            if (std::isfinite(zeroWarp) && std::isfinite(poleWarp)
                && zeroWarp > 0.0 && poleWarp > zeroWarp) {
                c.zero = static_cast<float>(coefficientFromWarped(zeroWarp));
                c.pole = static_cast<float>(coefficientFromWarped(poleWarp));
                // Guard the invariant the caller relies on rather than assume
                // it survived the narrowing to float.
                if (c.zero <= c.pole) {
                    c.renormalise();
                    return c;
                }
            }
        }

        // No passive first-order shelf reaches both points: the duct attenuates
        // faster than a shelf can tilt. Fall back to a pure pole matched at the
        // lower design frequency, over-attenuating above it.
        c.zero = 0.0F;
        c.pole = static_cast<float>(
            coefficientFromWarped((1.0 / lowerG2 - 1.0) / lowerS));
        c.renormalise();
        return c;
    }

    /** Apply one traversal's attenuation to one sample. */
    [[nodiscard]] static float process(const Coefficients& c, State& state,
                                       float pressurePa) noexcept {
        const auto input = std::isfinite(pressurePa) ? pressurePa : 0.0F;
        const auto output = c.gain * (input - c.zero * state.previousInput)
            + c.pole * state.previousOutput;
        state.previousInput = input;
        state.previousOutput = std::isfinite(output) ? output : 0.0F;
        return state.previousOutput;
    }

    /** Magnitude response of a fitted shelf, for tests and diagnostics. */
    [[nodiscard]] static double magnitude(const Coefficients& c,
                                          double frequencyHz,
                                          double sampleRateHz) noexcept {
        if (!(sampleRateHz > 0.0)) return 1.0;
        constexpr double pi = 3.14159265358979323846;
        const auto sine = std::sin(pi * frequencyHz / sampleRateHz);
        const auto s = sine * sine;
        const auto warp = [](double a) {
            const auto complement = 1.0 - a;
            return complement > 1.0e-12 ? 4.0 * a / (complement * complement) : 0.0;
        };
        const auto numerator = 1.0 + warp(c.zero) * s;
        const auto denominator = 1.0 + warp(c.pole) * s;
        return denominator > 0.0 ? std::sqrt(numerator / denominator) : 1.0;
    }
};

} // namespace enginelab
