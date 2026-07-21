#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace enginelab {

/**
 * A reactive expansion-chamber silencer on the collector-to-outlet duct.
 *
 * ---------------------------------------------------------------------------
 * Why this exists
 *
 * Until this element, `muffler_restriction` was a pressure-loss term and
 * nothing else: the physical exhaust branch never read it, never read
 * `openness`, and never reached the muffler FDN. Measured, driving it from
 * 0.30 to 0.95 -- a sports exhaust to a nearly blocked duct -- moved the
 * delivered spectrum by under 1.4 dB in any third octave. Every engine was
 * rendered as a straight open pipe terminated by a radiation load.
 *
 * That is why the Merlin V12, the flat-six and the radial already sounded
 * right and every road engine converged on the same generic timbre: those
 * three exhaust into a near-open stack in reality, so a straight-pipe model is
 * accidentally correct for them. A road car's timbre is largely *made* by its
 * silencer, and there was no silencer.
 *
 * ---------------------------------------------------------------------------
 * The model
 *
 * The simplest silencer that is a real acoustic filter rather than a tone
 * control is the single expansion chamber: a length L of pipe whose area jumps
 * from the duct area S_p up to a chamber area S_c and back down again. It is
 * two area discontinuities separated by a duct, so in travelling-wave variables
 * it is exactly two scattering junctions and one bidirectional delay line.
 *
 * At a junction between areas a (left) and b (right), continuity of pressure
 * and of volume velocity gives, for the outgoing waves,
 *
 *     p2+ = (1 + k) p1+ - k p2-
 *     p1- =      k p1+ + (1 - k) p2-        with  k = (a - b) / (a + b)
 *
 * -- the Kelly-Lochbaum junction. It is lossless for every k in [-1, 1]: the
 * element cannot add energy, and with no absorption it removes none either. It
 * only redistributes energy between what continues to the mouth and what is
 * reflected back toward the collector, which is what a reactive silencer does.
 *
 * Cascading the inlet junction (k), a delay of L/c, and the outlet junction
 * (-k) gives, with an anechoic outlet, the transmitted pressure ratio
 *
 *     p_t / p_i = (1 - k^2) D / (1 - k^2 D^2),   D = exp(-j w L / c)
 *
 * whose transmission loss is, writing m = S_c / S_p for the expansion ratio,
 *
 *     TL = 10 log10 [ 1 + 1/4 (m - 1/m)^2 sin^2(k L) ]
 *
 * the textbook single-expansion-chamber result (Munjal, *Acoustics of Ducts
 * and Mufflers*, ch. 2). `transmissionLossDb` below evaluates it, and
 * `EngineLab.RealtimeRegression` drives this element with sines and checks the
 * realisation against it. That reference comes from the literature, not from
 * the simulator's output, so tightening the gate later cannot recalibrate the
 * test onto the behaviour it is meant to catch.
 *
 * What matters audibly is the *shape*: TL vanishes whenever kL is a multiple
 * of pi and peaks between, so the chamber stamps a periodic comb of pass- and
 * stop-bands onto the exhaust, spaced by c/(2L) and deep in proportion to the
 * expansion ratio. That periodic colouration is what makes a road exhaust
 * sound like one, and it differs per engine because L and m differ.
 *
 * ---------------------------------------------------------------------------
 * Purely reactive, deliberately
 *
 * An earlier version also dissipated inside the chamber, with a broadband gain
 * and a one-pole lowpass driven by `muffler_restriction`. That mapping was
 * invented -- the restriction is a pressure-loss coefficient, not an
 * absorption coefficient -- and measuring it showed why inventing it was a
 * mistake: this element sits inside the collector-outlet feedback loop, so a
 * per-traversal loss of a couple of dB compounds and collapses the network's
 * low-frequency resonance. The EJ25 lost 11 dB at its rev-range fundamental,
 * and once loudness normalisation raised the clip to compensate, the
 * renderer's pre-existing high-frequency floor became 40 % of the signal
 * energy. The chamber was not generating hiss; it was deleting the engine.
 *
 * So there is no absorption here. Everything this element does follows from
 * two areas and a length, and the wall loss on the duct either side is the
 * network's frequency-dependent damping. A real absorptive silencer is a
 * different component; add it as one, with its own geometry, rather than by
 * overloading a flow coefficient.
 *
 * ---------------------------------------------------------------------------
 * Limits
 *
 * Plane-wave, single-chamber, no perforate and no side branch: it will not
 * reproduce a multi-chamber production silencer's exact curve, and above the
 * chamber's own plane-mode cutoff (roughly c / (1.7 * chamber diameter)) the
 * one-dimensional assumption fails and the real TL flattens out where this
 * model keeps combing. Both errors are in the direction of *more* structure
 * than reality, so do not stack further colouration on top of it.
 */
class ExpansionChamberMuffler final {
public:
    /**
     * Munjal's transmission loss for a single expansion chamber, in dB.
     *
     * @param expansionRatio  m = chamber area / duct area, > 0.
     * @param frequencyHz     frequency of interest.
     * @param traversalSeconds  one-way chamber traversal time L / c.
     */
    [[nodiscard]] static double transmissionLossDb(double expansionRatio,
                                                   double frequencyHz,
                                                   double traversalSeconds) noexcept {
        if (!(expansionRatio > 0.0) || !std::isfinite(frequencyHz)
            || !std::isfinite(traversalSeconds))
            return 0.0;
        constexpr double pi = 3.14159265358979323846;
        const auto phase = 2.0 * pi * frequencyHz * traversalSeconds;
        const auto shape = expansionRatio - 1.0 / expansionRatio;
        const auto sine = std::sin(phase);
        return 10.0 * std::log10(1.0 + 0.25 * shape * shape * sine * sine);
    }

    /** Kelly-Lochbaum reflection coefficient of the inlet junction. */
    [[nodiscard]] static float reflectionCoefficient(double ductAreaM2,
                                                     double chamberAreaM2) noexcept {
        if (!(ductAreaM2 > 0.0) || !(chamberAreaM2 > 0.0)) return 0.0F;
        const auto sum = ductAreaM2 + chamberAreaM2;
        if (!(sum > 0.0)) return 0.0F;
        return static_cast<float>(
            std::clamp((ductAreaM2 - chamberAreaM2) / sum, -0.98, 0.98));
    }

    /** Geometry and dissipation, recomputed at control rate. */
    struct Coefficients final {
        float reflection { 0.0F };
        float delaySamples { 0.0F };
        bool enabled { false };
    };

    /** The chamber's two travelling-wave lines. */
    struct State final {
        std::vector<float> towardOutlet;
        std::vector<float> towardCollector;
        std::size_t write { 0 };
        std::size_t mask { 0 };

        /** @param lengthPowerOfTwo line length; must be a power of two. */
        void prepare(std::size_t lengthPowerOfTwo) {
            towardOutlet.assign(lengthPowerOfTwo, 0.0F);
            towardCollector.assign(lengthPowerOfTwo, 0.0F);
            mask = lengthPowerOfTwo - 1U;
            write = 0;
        }

        void reset() noexcept {
            std::fill(towardOutlet.begin(), towardOutlet.end(), 0.0F);
            std::fill(towardCollector.begin(), towardCollector.end(), 0.0F);
            write = 0;
        }
    };

    /** Waves leaving the element on each side. */
    struct Output final {
        float towardOutlet { 0.0F };
        float towardCollector { 0.0F };
    };

    /**
     * Scatter one sample through the chamber.
     *
     * @param fromCollector  wave arriving from the collector line.
     * @param fromOutlet     wave arriving back from the radiation load.
     *
     * With `enabled` false the element is an exact through-connection, so an
     * engine with no chamber configured renders bit-identically to before.
     */
    [[nodiscard]] static Output process(State& state, const Coefficients& c,
                                        float fromCollector, float fromOutlet) noexcept {
        if (!c.enabled || state.towardOutlet.empty())
            return { fromCollector, fromOutlet };

        const auto reflection = c.reflection;
        const auto arrivedAtOutletJunction = read(state.towardOutlet, state, c.delaySamples);
        const auto arrivedAtInletJunction = read(state.towardCollector, state, c.delaySamples);

        // Inlet junction: duct area -> chamber area, coefficient k.
        const auto intoChamber = (1.0F + reflection) * fromCollector
            - reflection * arrivedAtInletJunction;
        const auto backToCollector = reflection * fromCollector
            + (1.0F - reflection) * arrivedAtInletJunction;
        // Outlet junction: chamber area -> duct area, coefficient -k.
        const auto onToOutlet = (1.0F - reflection) * arrivedAtOutletJunction
            + reflection * fromOutlet;
        const auto backIntoChamber = -reflection * arrivedAtOutletJunction
            + (1.0F + reflection) * fromOutlet;

        state.towardOutlet[state.write] = finite(intoChamber);
        state.towardCollector[state.write] = finite(backIntoChamber);
        state.write = (state.write + 1U) & state.mask;
        return { finite(onToOutlet), finite(backToCollector) };
    }

private:
    [[nodiscard]] static float finite(float value) noexcept {
        return std::isfinite(value) ? value : 0.0F;
    }

    /** Fractionally delayed read of a chamber line, one traversal back. */
    [[nodiscard]] static float read(const std::vector<float>& line, const State& state,
                                    float delaySamples) noexcept {
        const auto limit = static_cast<float>(line.size() - 2U);
        const auto delay = std::clamp(delaySamples, 1.0F, std::max(1.0F, limit));
        const auto whole = static_cast<std::size_t>(delay);
        const auto fraction = delay - static_cast<float>(whole);
        const auto first = (state.write + line.size() - whole) & state.mask;
        const auto second = (state.write + line.size() - whole - 1U) & state.mask;
        return std::lerp(line[first], line[second], fraction);
    }
};

} // namespace enginelab
