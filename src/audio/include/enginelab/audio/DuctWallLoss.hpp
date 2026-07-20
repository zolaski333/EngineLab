#pragma once

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
 * Realisation and its limits
 *
 * exp(-k sqrt(f)) is not rational, so it is approximated by a one-pole lowpass
 * whose gain is matched exactly to the Kirchhoff value at a reference frequency
 * in the middle of the band that matters. The one-pole is exact at DC (no loss,
 * correct) and at the reference; between them it under-attenuates slightly and
 * above the reference it over-attenuates relative to sqrt(f).
 *
 * Over-attenuating the top octaves is the conservative direction and is also
 * physically defensible: this model accounts only for the boundary-layer term,
 * while a real exhaust additionally loses high-frequency energy to wall
 * compliance, to higher-order mode conversion above the plane-mode cutoff, and
 * to the mean flow's turbulent shear. Those are not modelled here, so the true
 * high-frequency attenuation is higher than the Kirchhoff term alone, not lower.
 * A duct model that reproduced the sqrt(f) law exactly and stopped there would
 * still be optimistic about how much the top end rings.
 */
class DuctWallLoss final {
public:
    /** Frequency at which the one-pole matches the exact Kirchhoff attenuation.
     *  Chosen in the range where the audible mode structure lives. */
    static constexpr double referenceFrequencyHz = 1'000.0;

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

    /** One-pole state: a single word per traversal direction. */
    struct State final {
        float previousOutput { 0.0F };
        void reset() noexcept { previousOutput = 0.0F; }
    };

    /** y[n] = (1-a) x[n] + a y[n-1]; `a` is the fitted pole. */
    struct Coefficients final {
        float pole { 0.0F };
    };

    /**
     * Fit the one-pole so its gain equals the exact Kirchhoff gain at
     * referenceFrequencyHz.
     *
     * Solving |(1-a)/(1 - a e^-jw)| = G for a gives a quadratic
     *     (1-G^2) a^2 - 2 (1 - G^2 cos w) a + (1 - G^2) = 0
     * whose root inside the unit circle is the stable pole.
     */
    [[nodiscard]] static Coefficients fit(double traversalSeconds,
                                          double radiusM,
                                          double densityKgPerM3,
                                          double soundSpeedMps,
                                          double sampleRateHz) noexcept {
        Coefficients c {};
        if (!(sampleRateHz > 0.0)) return c;
        const auto reference = std::min(referenceFrequencyHz, sampleRateHz * 0.45);
        const auto gain = traversalGain(reference, traversalSeconds, radiusM,
                                        densityKgPerM3, soundSpeedMps);
        if (!(gain > 0.0) || gain >= 1.0) return c; // Lossless: pass through.
        const auto gainSquared = gain * gain;
        const auto denominator = 1.0 - gainSquared;
        if (!(denominator > 1.0e-12)) return c;
        constexpr double pi = 3.14159265358979323846;
        const auto omega = 2.0 * pi * reference / sampleRateHz;
        const auto b = 1.0 - gainSquared * std::cos(omega);
        const auto discriminant = b * b - denominator * denominator;
        if (!(discriminant >= 0.0)) return c;
        const auto root = (b - std::sqrt(discriminant)) / denominator;
        if (!std::isfinite(root) || root < 0.0 || root >= 1.0) return c;
        c.pole = static_cast<float>(root);
        return c;
    }

    /** Apply one traversal's attenuation to one sample. */
    [[nodiscard]] static float process(const Coefficients& c, State& state,
                                       float pressurePa) noexcept {
        const auto input = std::isfinite(pressurePa) ? pressurePa : 0.0F;
        const auto output = (1.0F - c.pole) * input + c.pole * state.previousOutput;
        state.previousOutput = std::isfinite(output) ? output : 0.0F;
        return state.previousOutput;
    }
};

} // namespace enginelab
