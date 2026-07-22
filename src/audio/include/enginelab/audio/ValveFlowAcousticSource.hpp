#pragma once

#include <enginelab/audio/BoundaryReconstructionFilter.hpp>

#include <cmath>
#include <numbers>

namespace enginelab {

/** High-band valve source complementary to the finite-volume boundary.
 *
 * The conservative exhaust network publishes a pressure/flow characteristic
 * only up to its coupling Nyquist frequency. The mechanical solver also
 * publishes the instantaneous signed valve mass flow at its higher cadence.
 * Pairing that flow with the slower network pressure would create a false
 * characteristic, so this class never does that. Instead it extracts only the
 * complementary high band with the high-pass half of the same fourth-order
 * Linkwitz-Riley crossover used by BoundaryReconstructionFilter.
 *
 * The remaining mass flow is a Norton volume-velocity source at the valve. For
 * characteristic pressure waves, an imposed volume velocity U contributes
 * +Zc*U/2 to the outgoing wave and -Zc*U/2 to the incoming wave. Passing the
 * latter through the physical valve termination therefore produces the exact
 * source coupling Zc*(1+R)*U/2, including the termination's inertance and loss,
 * without an empirical gain.
 */
class ValveFlowAcousticSource final {
public:
    struct Coefficients final {
        double b0 {};
        double b1 {};
        double b2 {};
        double a1 {};
        double a2 {};
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
        bool primed { false };
        void reset() noexcept { *this = State {}; }
    };

    struct Waves final {
        double outgoingPressurePa {};
        double incomingPressurePa {};
        double highBandMassFlowKgPerSecond {};
    };

    [[nodiscard]] static Coefficients compute(
        double couplingFrequencyHz, double sampleRateHz) noexcept {
        Coefficients coefficients;
        const auto cutoffHz = BoundaryReconstructionFilter::crossoverFrequencyHz(
            couplingFrequencyHz, sampleRateHz);
        if (!(cutoffHz > 0.0)) return coefficients;

        const auto w0 = 2.0 * std::numbers::pi * cutoffHz / sampleRateHz;
        const auto cosW0 = std::cos(w0);
        const auto alpha = std::sin(w0) * (0.5 * std::numbers::sqrt2);
        const auto a0 = 1.0 + alpha;
        coefficients.b0 = 0.5 * (1.0 + cosW0) / a0;
        coefficients.b1 = -(1.0 + cosW0) / a0;
        coefficients.b2 = coefficients.b0;
        coefficients.a1 = -2.0 * cosW0 / a0;
        coefficients.a2 = (1.0 - alpha) / a0;
        coefficients.active = true;
        return coefficients;
    }

    [[nodiscard]] static Waves process(
        const Coefficients& coefficients,
        State& state,
        double instantaneousMassFlowKgPerSecond,
        double densityKgPerM3,
        double characteristicImpedancePaSPerM3) noexcept {
        if (!coefficients.active) return {};
        const auto input = std::isfinite(instantaneousMassFlowKgPerSecond)
            ? instantaneousMassFlowKgPerSecond : 0.0;
        if (!state.primed) {
            // A constant input is the high-pass steady state: the first section
            // remembers the input while both outputs and the second input stay
            // at zero. Enabling the source therefore cannot create a startup
            // impulse from the mean exhaust flow.
            state.first = { input, input, 0.0, 0.0 };
            state.second = {};
            state.primed = true;
        }
        const auto highBandMassFlow = processSection(coefficients, state.second,
            processSection(coefficients, state.first, input));
        if (!(densityKgPerM3 > 0.0) || !std::isfinite(densityKgPerM3)
            || !(characteristicImpedancePaSPerM3 > 0.0)
            || !std::isfinite(characteristicImpedancePaSPerM3)
            || !std::isfinite(highBandMassFlow))
            return {};
        const auto wavePressure = 0.5 * characteristicImpedancePaSPerM3
            * highBandMassFlow / densityKgPerM3;
        if (!std::isfinite(wavePressure)) return {};
        return { wavePressure, -wavePressure, highBandMassFlow };
    }

private:
    [[nodiscard]] static double processSection(
        const Coefficients& c, SectionState& s, double x) noexcept {
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
