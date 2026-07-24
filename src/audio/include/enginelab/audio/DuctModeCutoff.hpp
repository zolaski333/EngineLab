#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace enginelab {

/**
 * Band limit of the plane-wave model in a circular duct.
 *
 * Every duct in the acoustic network is a bidirectional delay line, which is a
 * model of *one* thing: the plane mode, the axial wave whose pressure is uniform
 * across the section. That model is not approximately right above a certain
 * frequency -- it stops being a model of the duct at all.
 *
 * A rigid circular duct of radius a supports higher-order modes whose radial
 * wavenumbers are the extrema of the Bessel functions. The first of them, the
 * (1,0) mode, begins to propagate at
 *
 *     f_c = j'_11 c / (2 pi a),   j'_11 = 1.8412,
 *
 * which is 4.0 kHz for a 50 mm duct in air and, for hot exhaust gas, 7-11 kHz in
 * a header primary and 2.1-2.9 kHz in an expansion chamber. Below f_c the plane
 * mode is the only propagating solution and the delay line is exact. Above it
 * the duct carries at least two modes with different axial phase velocities;
 * every discontinuity the wave meets -- an area step, a junction, a bend, the
 * valve port -- scatters plane-mode energy into them, and once there it no
 * longer keeps step with the plane mode, no longer forms the standing-wave
 * pattern the delay line computes, and no longer radiates from the outlet with
 * plane-wave directivity.
 *
 * From the plane-wave channel's point of view that scattering is a loss, and it
 * is the loss this network was missing. It is also the mechanism the wall-loss
 * model names as the reason it is allowed to be conservative -- see
 * DuctWallLoss. Leaving it implicit inside another model's fit error meant the
 * band limit was set by a filter's corner instead of by duct geometry, which is
 * why an expansion chamber and a header primary used to roll off alike when
 * physically they are three octaves apart.
 *
 * ---------------------------------------------------------------------------
 * What this is, and what it is not
 *
 * This is a validity band, not a tone control. The only inputs are the duct
 * radius and the gas speed of sound, both of which the network already knows;
 * there is no depth, no amount, and nothing to tune. It is the same kind of
 * statement as the anti-imaging low-pass at the coupling rate: content above the
 * limit is not signal the model computed, so the model does not carry it.
 *
 * The realisation is a fourth-order Butterworth low-pass at f_c, applied once
 * per traversal, because a traversal is exactly the interval that ends at a
 * discontinuity where the conversion happens.
 *
 * Butterworth, because a maximally flat passband is what "transparent below
 * cutoff" has to mean here: an evanescent mode below f_c stores energy
 * reactively and dissipates none, so the model must not attenuate there. That
 * requirement is sharper than it looks, because this section sits in the
 * collector-to-outlet feedback loop where a fraction of a dB per traversal
 * compounds -- the failure this project has already paid for once. Fourth order
 * is what buys the clean passband: an octave below cutoff it is 0.017 dB down,
 * where second order would be 0.264 dB and would accumulate into several dB of
 * midrange loss around a resonance. It is also the more faithful transition. The
 * first higher mode does not fade in; its axial wavenumber turns from imaginary
 * to real at f_c, so the physical onset is abrupt, and a gentle roll-off would
 * misplace loss below cutoff where there is none. The same fourth-order choice
 * is already made for the other validity band in this signal path, the
 * anti-imaging limit in BoundaryReconstructionFilter.
 *
 * The cascade over repeated traversals is what makes a trapped high mode decay,
 * and that accumulation is physical: each bounce scatters more energy out of the
 * plane mode.
 *
 * The structure is a cascade of two topology-preserving (TPT) state-variable
 * sections rather than biquads. The network slews the gas state every sample, so
 * the cutoff moves every sample; a direct-form biquad with interpolated
 * coefficients can misbehave transiently, while the TPT form is parametrised by
 * the single prewarped quantity g = tan(pi f_c / fs) and stays stable for any
 * positive g. That makes interpolating the cutoff safe by construction, and both
 * sections share the one g.
 *
 * Scope: the exhaust network applies this. The intake network runs the same
 * delay-line model and has the same validity limit, but its voicing is a
 * separate maintained calibration and is not measured by the exhaust
 * instruments, so it is left alone deliberately rather than by oversight.
 */
class DuctModeCutoff final {
public:
    /** First zero of J1', which sets the (1,0) mode cutoff in a circular duct. */
    static constexpr double firstHigherModeBesselRoot = 1.8412;

    /** Highest fraction of the sample rate the cutoff is allowed to reach.
     *  Clamping instead of switching keeps the response continuous when a duct
     *  is narrow enough that its physical cutoff runs past Nyquist; at this
     *  corner the section is transparent to better than 0.001 dB in band. */
    static constexpr double maximumCutoffFraction = 0.49;

    /** Plane-mode cutoff of a circular duct, Hz. */
    [[nodiscard]] static double cutoffFrequencyHz(double radiusM,
                                                  double soundSpeedMps) noexcept {
        if (!(radiusM > 0.0) || !(soundSpeedMps > 0.0)
            || !std::isfinite(radiusM) || !std::isfinite(soundSpeedMps))
            return 0.0;
        constexpr double pi = 3.14159265358979323846;
        return firstHigherModeBesselRoot * soundSpeedMps / (2.0 * pi * radiusM);
    }

    /** Number of cascaded second-order sections; fourth order overall. */
    static constexpr std::size_t sectionCount = 2;

    /** Butterworth section damping, 2R = 1/Q. A fourth-order Butterworth places
     *  its pole pairs at 22.5 and 67.5 degrees from the imaginary axis, giving
     *  Q = 0.5412 and Q = 1.3066. */
    static constexpr std::array<double, sectionCount> sectionTwoR {
        1.84775906502257351225, 0.76536686473017954346 };

    /** Two integrators per section, per traversal direction. */
    struct State final {
        std::array<float, sectionCount> integrator1 {};
        std::array<float, sectionCount> integrator2 {};
        void reset() noexcept {
            integrator1.fill(0.0F);
            integrator2.fill(0.0F);
        }
    };

    struct Coefficients final {
        /** Prewarped cutoff, tan(pi f_c / fs), shared by both sections. */
        float g { 0.0F };
        /** 1 / (1 + 2R g + g^2) per section, the resolved feedback denominator. */
        std::array<float, sectionCount> normalisation { 1.0F, 1.0F };

        /** Rebuild the denominators after g has been slewed. */
        void renormalise() noexcept {
            for (std::size_t section = 0; section < sectionCount; ++section) {
                const auto denominator = 1.0F
                    + static_cast<float>(sectionTwoR[section]) * g + g * g;
                normalisation[section] =
                    denominator > 1.0e-12F ? 1.0F / denominator : 1.0F;
            }
        }
    };

    /** Resolve the section for one duct's geometry and current gas state. */
    [[nodiscard]] static Coefficients fit(double radiusM,
                                          double soundSpeedMps,
                                          double sampleRateHz) noexcept {
        Coefficients c {};
        if (!(sampleRateHz > 0.0)) return c;
        const auto cutoffHz = cutoffFrequencyHz(radiusM, soundSpeedMps);
        if (!(cutoffHz > 0.0)) {
            // No usable geometry: stay transparent rather than invent a band.
            c.g = static_cast<float>(std::tan(3.14159265358979323846
                                              * maximumCutoffFraction));
            c.renormalise();
            return c;
        }
        constexpr double pi = 3.14159265358979323846;
        const auto normalised = std::min(cutoffHz / sampleRateHz, maximumCutoffFraction);
        c.g = static_cast<float>(std::tan(pi * normalised));
        c.renormalise();
        return c;
    }

    /** Advance one traversal's band limit by one sample. */
    [[nodiscard]] static float process(const Coefficients& c, State& state,
                                       float pressurePa) noexcept {
        auto signal = std::isfinite(pressurePa) ? pressurePa : 0.0F;
        for (std::size_t section = 0; section < sectionCount; ++section) {
            const auto twoR = static_cast<float>(sectionTwoR[section]);
            const auto highPass = (signal - (twoR + c.g) * state.integrator1[section]
                - state.integrator2[section]) * c.normalisation[section];
            const auto bandPass = c.g * highPass + state.integrator1[section];
            state.integrator1[section] = c.g * highPass + bandPass;
            const auto lowPass = c.g * bandPass + state.integrator2[section];
            state.integrator2[section] = c.g * bandPass + lowPass;
            if (!std::isfinite(state.integrator1[section]))
                state.integrator1[section] = 0.0F;
            if (!std::isfinite(state.integrator2[section]))
                state.integrator2[section] = 0.0F;
            signal = std::isfinite(lowPass) ? lowPass : 0.0F;
        }
        return signal;
    }

    /** Magnitude response of a resolved cascade, for tests and diagnostics.
     *
     *  Each TPT section realises its prewarped analog Butterworth section
     *  exactly, so with Omega = tan(pi f / fs) / g the cascade magnitude is the
     *  fourth-order Butterworth 1/sqrt(1 + Omega^8).
     */
    [[nodiscard]] static double magnitude(const Coefficients& c,
                                          double frequencyHz,
                                          double sampleRateHz) noexcept {
        if (!(sampleRateHz > 0.0) || !(c.g > 0.0F)) return 1.0;
        constexpr double pi = 3.14159265358979323846;
        const auto normalised = frequencyHz / sampleRateHz;
        if (!(normalised > 0.0)) return 1.0;
        if (normalised >= 0.5) return 0.0;
        const auto omega = std::tan(pi * normalised) / static_cast<double>(c.g);
        const auto omegaFourth = std::pow(omega, 4);
        return 1.0 / std::sqrt(1.0 + omegaFourth * omegaFourth);
    }
};

} // namespace enginelab
