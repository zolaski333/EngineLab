#pragma once

#include <enginelab/audio/BoundaryReconstructionFilter.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
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
    using Biquad = BoundaryReconstructionFilter::Biquad;
    using SectionState = BoundaryReconstructionFilter::SectionState;
    /** Both edges track the reconstruction filter's order and Q pair, so the two
     *  physical bands always meet with matched slopes at one crossover. */
    static constexpr auto butterworthQ = BoundaryReconstructionFilter::butterworthQ;
    static constexpr auto sectionCount = BoundaryReconstructionFilter::sectionCount;

    struct Coefficients final {
        std::array<Biquad, butterworthQ.size()> highStage {};
        std::array<Biquad, butterworthQ.size()> upperStage {};
        bool active { false };
        bool upperBandLimited { false };
    };

    struct State final {
        std::array<SectionState, sectionCount> high {};
        std::array<SectionState, sectionCount> upper {};
        bool primed { false };
        void reset() noexcept { *this = State {}; }
    };

    struct Waves final {
        double outgoingPressurePa {};
        double incomingPressurePa {};
        double highBandMassFlowKgPerSecond {};
    };

    [[nodiscard]] static Coefficients compute(
        double couplingFrequencyHz, double sampleRateHz,
        double sourceSamplingFrequencyHz = 0.0) noexcept {
        Coefficients coefficients;
        const auto cutoffHz = BoundaryReconstructionFilter::crossoverFrequencyHz(
            couplingFrequencyHz, sampleRateHz);
        if (!(cutoffHz > 0.0)) return coefficients;

        const auto w0 = 2.0 * std::numbers::pi * cutoffHz / sampleRateHz;
        const auto cosW0 = std::cos(w0);
        const auto sinW0 = std::sin(w0);
        for (std::size_t index = 0; index < butterworthQ.size(); ++index) {
            const auto alpha = sinW0 / (2.0 * butterworthQ[index]);
            const auto a0 = 1.0 + alpha;
            auto& stage = coefficients.highStage[index];
            stage.b0 = 0.5 * (1.0 + cosW0) / a0;
            stage.b1 = -(1.0 + cosW0) / a0;
            stage.b2 = stage.b0;
            stage.a1 = -2.0 * cosW0 / a0;
            stage.a2 = (1.0 - alpha) / a0;
        }
        coefficients.active = true;
        // The instantaneous source is still sampled by the mechanical solver.
        // Its complementary band ends at that producer's Nyquist frequency;
        // without this reconstruction low-pass, first-order-hold images pass
        // through the radiation derivative as isolated clicks. The upper edge is
        // the same eighth-order Linkwitz-Riley as the lower one, for the same
        // reason: at fourth order the first image of this stream sat only 30 dB
        // down, which the harness's own metallic threshold does not clear.
        // 0.42x the source rate keeps the transition below the source Nyquist
        // (0.5x) so the stop-band is established before that first image; it is
        // clamped by the audio-rate limit for very fast source streams.
        const auto upperHz = std::min(0.42 * sourceSamplingFrequencyHz,
            0.45 * sampleRateHz);
        if (upperHz > cutoffHz * 1.05 && upperHz > 0.0) {
            const auto upperW0 = 2.0 * std::numbers::pi * upperHz / sampleRateHz;
            const auto upperCos = std::cos(upperW0);
            const auto upperSin = std::sin(upperW0);
            for (std::size_t index = 0; index < butterworthQ.size(); ++index) {
                const auto alpha = upperSin / (2.0 * butterworthQ[index]);
                const auto a0 = 1.0 + alpha;
                auto& stage = coefficients.upperStage[index];
                stage.b0 = 0.5 * (1.0 - upperCos) / a0;
                stage.b1 = (1.0 - upperCos) / a0;
                stage.b2 = stage.b0;
                stage.a1 = -2.0 * upperCos / a0;
                stage.a2 = (1.0 - alpha) / a0;
            }
            coefficients.upperBandLimited = true;
        } else if (sourceSamplingFrequencyHz > 0.0) {
            // A real source rate was published but it does not clear the
            // reconstruction crossover, so the complementary band is empty (its
            // upper edge is at or below the lower edge). Leaving the high-pass
            // running with no upper edge would radiate only first-order-hold
            // image content above the source Nyquist. Disable the source
            // entirely: the reconstructed low band already carries everything
            // the producer can resolve. (When no source rate is given the band
            // stays open, the historical behaviour for rate-agnostic callers.)
            coefficients.active = false;
        }
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
            // A constant input is the high-pass steady state: only the leading
            // section remembers the input, because every section downstream of
            // it sees the zero that a settled high pass produces. Enabling the
            // source therefore cannot create a startup impulse from the mean
            // exhaust flow.
            state.high.fill({});
            state.upper.fill({});
            state.high.front() = { input, input, 0.0, 0.0 };
            state.primed = true;
        }
        auto highBandMassFlow = input;
        for (std::size_t index = 0; index < sectionCount; ++index)
            highBandMassFlow = processSection(
                coefficients.highStage[index % butterworthQ.size()],
                state.high[index], highBandMassFlow);
        if (coefficients.upperBandLimited) {
            for (std::size_t index = 0; index < sectionCount; ++index)
                highBandMassFlow = processSection(
                    coefficients.upperStage[index % butterworthQ.size()],
                    state.upper[index], highBandMassFlow);
        }
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
        const Biquad& c, SectionState& s, double x) noexcept {
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
