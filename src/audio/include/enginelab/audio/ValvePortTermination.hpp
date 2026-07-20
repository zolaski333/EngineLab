#pragma once

#include <cmath>

namespace enginelab {

/**
 * Acoustic termination presented to an exhaust runner by its exhaust valve.
 *
 * The runner is a waveguide whose port end is closed by a poppet valve. What the
 * wave arriving at that end sees is the valve's acoustic impedance, and the
 * reflection coefficient follows from it:
 *
 *     R(w) = (Z(w) - Zc) / (Z(w) + Zc)
 *
 * with Zc = rho*c/A the runner's characteristic impedance. Modelling Z as a pure
 * resistance linearised about the mean flow is wrong in both limits that matter
 * here:
 *
 *  - As the mean flow goes to zero the resistance goes to zero and R goes to -1,
 *    so a nearly shut valve is modelled as a perfect pressure-release open end.
 *    The sign is backwards: a shut valve is a rigid end, R -> +1.
 *  - |R| = 1 for any purely real Z of either sign, so the port returns every
 *    joule it receives. A runner terminated that way has no loss path at all and
 *    its modes ring without bound, which is audible as a metallic tone that
 *    grows with run length.
 *
 * Two terms are restored, both standard orifice acoustics:
 *
 *  - Inertance. Accelerating gas through the opening costs momentum, giving a
 *    reactance jwM with M = rho*l_eff/(CdA). The effective length carries the
 *    classic 0.85a end correction on each face of a thin orifice, so
 *    l_eff = 1.7*sqrt(CdA/pi). As CdA -> 0, M -> infinity and R -> +1 for every
 *    frequency, which is the rigid end the previous model got backwards. This
 *    term is reactive, so it sets which frequencies resonate but dissipates
 *    nothing on its own.
 *
 *  - Nonlinear orifice resistance (Ingard & Ising). The resistance of an orifice
 *    is set by the *total* velocity through it, not by the mean flow alone: an
 *    acoustic velocity sheds vortices at the orifice edge and loses energy even
 *    with no mean flow at all. Using mean + acoustic velocity is what gives the
 *    port a finite Q at idle, where the mean-flow-only model gave it none. The
 *    resistance rises with acoustic amplitude, so it damps a growing mode harder
 *    the louder that mode gets -- which is why a real runner does not ring.
 *
 * The result is a first-order filter, not a scalar, so it is realised by
 * bilinear transform of R(s) and applied per sample. Coefficients are recomputed
 * from the instantaneous boundary state; the caller owns the two state words.
 */
class ValvePortTermination final {
public:
    /** Bilinear-transformed R(z) = (b0 + b1 z^-1) / (1 + a1 z^-1).
     *
     * Held in double deliberately. A strongly resistive port drives Rv far above
     * Zc, where b0, b1 and a1 all approach 1 and differ only in their low-order
     * bits; the response at Nyquist is the ratio of two such differences. In
     * float those differences fall below the epsilon of the operands and the
     * filter can round to a magnitude just above unity, i.e. an active
     * termination feeding a waveguide. Double keeps the differences exact enough
     * for the passivity the physics guarantees. There is one of these per runner
     * per sample, so the cost is irrelevant. */
    struct Coefficients final {
        double b0 { 1.0 };
        double b1 { -1.0 };
        double a1 { -1.0 };
    };

    /** Two-word filter memory for one port. */
    struct State final {
        double previousInput { 0.0 };
        double previousOutput { 0.0 };
        void reset() noexcept { previousInput = previousOutput = 0.0; }
    };

    /** Below this open area the valve is treated as a rigid end (R = +1). */
    static constexpr double closedConductanceAreaM2 = 1.0e-9;

    /**
     * Reflection filter for one port.
     *
     * @param conductanceAreaM2     effective open area Cd*A of the valve.
     * @param meanMassFlowKgPerS    mean mass flow through the port (either sign).
     * @param acousticVolumeVelocityM3PerS
     *                              instantaneous acoustic volume velocity at the
     *                              port; drives the nonlinear resistance.
     * @param densityKgPerM3        gas density at the port.
     * @param characteristicImpedancePaSPerM3
     *                              rho*c/A of the runner the port terminates.
     * @param sampleRateHz          render rate.
     */
    [[nodiscard]] static Coefficients compute(
        double conductanceAreaM2,
        double meanMassFlowKgPerS,
        double acousticVolumeVelocityM3PerS,
        double densityKgPerM3,
        double characteristicImpedancePaSPerM3,
        double sampleRateHz) noexcept {
        Coefficients rigid {};
        const auto finitePositive = [](double v) { return std::isfinite(v) && v > 0.0; };
        if (!finitePositive(conductanceAreaM2)
            || conductanceAreaM2 <= closedConductanceAreaM2
            || !finitePositive(densityKgPerM3)
            || !finitePositive(characteristicImpedancePaSPerM3)
            || !finitePositive(sampleRateHz))
            return rigid; // Rigid end: R(z) = 1 at every frequency.

        // Total mass flow through the opening: mean plus the acoustic
        // contribution. rho*|U_ac| converts the acoustic volume velocity to the
        // same units as the mean mass flow.
        const auto acoustic = std::isfinite(acousticVolumeVelocityM3PerS)
            ? std::abs(acousticVolumeVelocityM3PerS) : 0.0;
        const auto mean = std::isfinite(meanMassFlowKgPerS)
            ? std::abs(meanMassFlowKgPerS) : 0.0;
        const auto totalMassFlow = mean + densityKgPerM3 * acoustic;

        // Quasi-steady orifice pressure drop dp = rho/2 * (U/CdA)^2 linearised
        // about the total flow: d(dp)/dU = mdot/(CdA)^2.
        const auto resistance = totalMassFlow
            / (conductanceAreaM2 * conductanceAreaM2);

        // Orifice inertance with the 0.85a end correction on both faces.
        constexpr double pi = 3.14159265358979323846;
        const auto effectiveLengthM = 1.7 * std::sqrt(conductanceAreaM2 / pi);
        const auto inertance = densityKgPerM3 * effectiveLengthM / conductanceAreaM2;

        // R(s) = (Rv + sM - Zc) / (Rv + sM + Zc), bilinear s = 2fs(1-z)/(1+z).
        const auto k = 2.0 * sampleRateHz * inertance;
        const auto zc = characteristicImpedancePaSPerM3;
        const auto denominator0 = (resistance + zc) + k;
        if (!(std::isfinite(denominator0)) || std::abs(denominator0) < 1.0e-30)
            return rigid;

        Coefficients c {};
        c.b0 = ((resistance - zc) + k) / denominator0;
        c.b1 = ((resistance - zc) - k) / denominator0;
        c.a1 = ((resistance + zc) - k) / denominator0;
        if (!std::isfinite(c.b0) || !std::isfinite(c.b1) || !std::isfinite(c.a1))
            return rigid;
        return c;
    }

    /** Apply one sample of R(z). */
    [[nodiscard]] static float process(const Coefficients& c, State& state,
                                       float incidentPressurePa) noexcept {
        const auto input = std::isfinite(incidentPressurePa)
            ? static_cast<double>(incidentPressurePa) : 0.0;
        const auto output = c.b0 * input + c.b1 * state.previousInput
            - c.a1 * state.previousOutput;
        state.previousInput = input;
        state.previousOutput = std::isfinite(output) ? output : 0.0;
        return static_cast<float>(state.previousOutput);
    }
};

} // namespace enginelab
