#pragma once

/** Loop-scavenging quality for a crankcase-scavenged two-stroke.
 *
 * The simulator gives a two-stroke one well-mixed cylinder cell with the
 * transfer and exhaust ports as orifices onto it. That geometry *is* perfect
 * mixing: whatever leaves the exhaust port carries the cylinder's mean
 * composition, so trapping becomes a function of delivery ratio with no engine
 * speed in it anywhere. Measured on the catalogue RD350 (journal 2026-10-21),
 * trapping ran 0.448 at 2,000 rpm to 0.807 at 9,000 purely along that
 * relation, and the torque curve climbed to the rev limiter instead of falling
 * away after its peak.
 *
 * A real loop-scavenged cylinder is not mixed. The transfer ports aim the
 * incoming charge up the wall away from the exhaust port; it loops over the
 * head and comes down, pushing burned gas out ahead of it. Until that front
 * reaches the exhaust port, what leaves is burned gas and the engine is
 * *displacing*. After it arrives, fresh charge starts escaping and the engine
 * is *mixing*. Those two are the classical bounds on scavenging efficiency,
 *
 *     perfect displacement: SE = min(DR, 1)
 *     perfect mixing:       SE = 1 - exp(-DR)
 *
 * and every real engine sits between them (Benson-Brandham's two-phase model
 * is exactly this: displacement, then mixing). The simulator was pinned to the
 * lower bound.
 *
 * What puts an engine between the bounds is the race between the front's
 * travel and the port window: the front needs one loop length to reach the
 * exhaust port, and the window lasts `transferOpenDegrees / (6 * rpm)`
 * seconds. Raising engine speed shortens the window without making the jet
 * proportionally faster, so the displacement phase occupies a *larger* share
 * of a faster window -- which is why a real two-stroke scavenges better near
 * its tuned speed and is dirty and inefficient when lugged. That is the
 * engine-speed dependence the model had no mechanism for, and it falls out of
 * the geometry rather than being a fitted curve.
 *
 * The front's progress is *integrated*, not predicted from one sample. An
 * earlier version took the jet velocity once, at the instant the transfers
 * opened, and divided the loop length by it to get a transit time. That put
 * the whole model on the single jitteriest quantity available: in the
 * 5,000-6,000 rpm band the exhaust pipe's returning wave sits near the port
 * opening, so the cylinder pressure at that instant swings cycle to cycle, and
 * the loop closed -- strong jet, more trapped, more exhaust energy, stronger
 * wave, different pressure at the next opening. The RD350's brake torque then
 * failed to hold at all there: 13.3 to 37.6 N.m around a 25.6 N.m mean at
 * 5,999 rpm, against 6 % spread with the model switched off (journal
 * 2026-10-21). A front does not travel at the orifice velocity of one instant
 * anyway, so integrating is both the cheaper and the truer answer.
 *
 * The progress is a state, never a derivative of the flow the bias modulates:
 * the velocity comes from the *transfer* port's pressure difference, while the
 * bias acts on the *exhaust* port's outflow composition. A bias derived from
 * its own flow over-determines the orifice and its gain grows as the time step
 * shrinks (`CLAUDE.md`: "a flow bias must be a state, never a derivative of
 * the flow it drives").
 */

#include <algorithm>
#include <cmath>

namespace enginelab {

/** Per-cylinder scavenging state, reset when the transfer ports open.
 *
 * `frontProgress` counts loop lengths the fresh front has travelled this
 * window: 1.0 is the exhaust port. It is integrated sub-step by sub-step from
 * the transfer jet's velocity, so a window that shuts early leaves it short
 * and a window with no pressure behind the jet does not advance it at all.
 */
struct TwoStrokeScavengeState final {
    /** Loop lengths travelled since the transfer ports opened. */
    double frontProgress { 0.0 };
    /** This cylinder's loop length, held for the window. Zero disables the
     *  model and restores the perfectly mixed cylinder exactly. */
    double loopLengthM { 0.0 };
    /** False until the window has been opened at least once. */
    bool open { false };
};

/** Smooth 0->1 ramp. A hard switch between the displacement and mixing phases
 * would step the exhaust boundary's composition inside one sub-step, and a step
 * at a solver boundary rings; the classical two-phase model's corner is not
 * physical anyway, since the front arrives over a finite spread.
 *
 * The ramp's WIDTH is one further loop length of travel, not the rest of the
 * window. Once the front reaches the exhaust port the loop has broken down, and
 * a turbulent jet mixes over roughly the same timescale it needed to arrive.
 * Spreading the breakdown over the whole remaining window instead gave a lugged
 * engine a gentle ramp across 90 % of a 10.7 ms window, where a through-flowing
 * cylinder is fully mixed (or worse) for almost all of it: trapping at 2,000
 * rpm came out 0.73 against a measured 0.45, and torque 48.8 N.m against about
 * 29 (journal 2026-10-21). */
[[nodiscard]] constexpr double smoothRamp(double t) noexcept {
    if (!(t > 0.0)) return 0.0;
    if (t >= 1.0) return 1.0;
    return t * t * (3.0 - 2.0 * t);
}

/** Open a scavenging window. `loopLengthM` is the distance the front covers to
 *  reach the exhaust port: up the far wall, over the head and back down. */
inline void openScavengeWindow(
    TwoStrokeScavengeState& state, double loopLengthM) noexcept {
    state.frontProgress = 0.0;
    state.loopLengthM = std::max(0.0, loopLengthM);
    state.open = true;
}

/** Advance the front by one sub-step at the transfer jet's current velocity.
 *
 * `jetVelocityMps` is taken from the crankcase-to-cylinder pressure difference
 * -- not from the port mass flow, which this bias goes on to shape. A
 * non-positive velocity leaves the front where it is: there is nothing pushing
 * it. */
inline void advanceScavengeWindow(TwoStrokeScavengeState& state,
    double jetVelocityMps, double dtSeconds) noexcept {
    if (!state.open || !(state.loopLengthM > 0.0)) return;
    if (!(jetVelocityMps > 0.0) || !(dtSeconds > 0.0)) return;
    state.frontProgress += jetVelocityMps * dtSeconds / state.loopLengthM;
}

inline void closeScavengeWindow(TwoStrokeScavengeState& state) noexcept {
    state.open = false;
    state.frontProgress = 0.0;
}

/** How much of the exhaust outflow is drawn from the fully mixed cylinder
 * rather than from its burned gas, in [0, 1].
 *
 * 0 is perfect displacement: the port is passing burned gas only, and every
 * molecule of fresh charge admitted so far is kept. 1 is perfect mixing, the
 * behaviour the simulator had unconditionally. The window runs from 0 towards 1
 * as the front arrives and the loop breaks down.
 *
 * Returns 1 for a closed window or an unknown geometry, so an engine without a
 * loop length behaves exactly as it did before this existed.
 */
[[nodiscard]] inline double scavengeMixedFraction(
    const TwoStrokeScavengeState& state) noexcept {
    if (!state.open || !(state.loopLengthM > 0.0)) return 1.0;
    return smoothRamp(state.frontProgress - 1.0);
}

/** Scavenging efficiency of the two classical bounds, for tests and for
 * reporting. `deliveryRatio` is delivered charge over cylinder trapped volume
 * at port closure, both as mass. */
[[nodiscard]] inline double perfectDisplacementEfficiency(
    double deliveryRatio) noexcept {
    return std::clamp(deliveryRatio, 0.0, 1.0);
}

[[nodiscard]] inline double perfectMixingEfficiency(
    double deliveryRatio) noexcept {
    if (!(deliveryRatio > 0.0)) return 0.0;
    return 1.0 - std::exp(-deliveryRatio);
}

/** Densities (any consistent unit) of the four species at the exhaust port,
 * biased towards burned gas by a scavenging window's `mixedFraction`.
 *
 * Only the composition is biased, never the total: the sum of the four is
 * preserved exactly, and the caller presents the cell's own energy density
 * unchanged. That is deliberate. The pressure and temperature at the boundary
 * drive the flow; this model says what the gas leaving *is*, not how much of
 * it leaves. Biasing the energy density too would change the mass flow the
 * orifice produces, which is a different claim and one this measurement cannot
 * support.
 *
 * `fuel` and `oxygen` are the only unambiguous markers of fresh charge --
 * `inert` is nitrogen, which fresh air and burned gas both carry, so it cannot
 * be attributed to either. So the bias keeps back fresh charge in proportion to
 * `mixedFraction` and makes the missing mass up from inert and burned in the
 * ratio the cylinder already holds them.
 *
 * The guard matters for cranking and the first cycles: if the cylinder holds
 * less burned gas than the fresh charge being held back, there is nothing to
 * displace *with*, and the bias is reduced to what the burned gas can cover
 * rather than inventing mass. Without it the inventory delta would be rejected
 * and `solverResolutionLimited` would latch on every start.
 */
struct ScavengeOutflowDensities final {
    double oxygen { 0.0 };
    double inert { 0.0 };
    double fuel { 0.0 };
    double burned { 0.0 };
};

[[nodiscard]] inline ScavengeOutflowDensities biasedOutflowDensities(
    double oxygen, double inert, double fuel, double burned,
    double mixedFraction) noexcept {
    const ScavengeOutflowDensities unbiased { oxygen, inert, fuel, burned };
    const auto mixed = std::clamp(mixedFraction, 0.0, 1.0);
    if (mixed >= 1.0) return unbiased;
    const auto fresh = oxygen + fuel;
    const auto residual = inert + burned;
    if (!(fresh > 0.0) || !(residual > 0.0)) return unbiased;
    // What the fresh charge gives up, capped at what the burned gas can
    // replace: a displacement needs something to displace with.
    const auto shortfall = std::min((1.0 - mixed) * fresh, residual);
    const auto keptShare = 1.0 - shortfall / fresh;
    const auto residualScale = 1.0 + shortfall / residual;
    return {
        oxygen * keptShare,
        inert * residualScale,
        fuel * keptShare,
        burned * residualScale,
    };
}

} // namespace enginelab
