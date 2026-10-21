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
 * transit and the port window. The front needs `loopLength / jetVelocity` to
 * arrive; the window lasts `transferOpenDegrees / (6 * rpm)` seconds. Raising
 * engine speed shortens the window without changing the transit, so the
 * displacement phase occupies a *larger* share of a faster window -- which is
 * why a real two-stroke scavenges better near its tuned speed and is dirty and
 * inefficient when lugged. That is the engine-speed dependence the model had no
 * mechanism for, and it is a ratio of two times rather than a fitted curve.
 *
 * Everything here is a pure function of a per-window state. The state is
 * integrated from the flows (see `advance`); the bias is never computed from
 * the instantaneous flow it goes on to modulate, because a bias derived from
 * its own flow over-determines the orifice and its gain grows as the time step
 * shrinks (`CLAUDE.md`: "a flow bias must be a state, never a derivative of
 * the flow it drives").
 */

#include <algorithm>
#include <cmath>

namespace enginelab {

/** Per-cylinder scavenging state, reset when the transfer ports open.
 *
 * `windowSeconds` and `transitSeconds` are sampled once at window open and
 * held for the window. Sampling them every sub-step would make the bias track
 * the flow it modulates; holding them makes the window a state.
 */
struct TwoStrokeScavengeState final {
    /** Seconds since the transfer ports opened. */
    double elapsedSeconds { 0.0 };
    /** Length of this window, from engine speed at the moment it opened. */
    double windowSeconds { 0.0 };
    /** Time the fresh front needs to reach the exhaust port, this window. */
    double transitSeconds { 0.0 };
    /** False until the window has been opened at least once. */
    bool open { false };
};

/** Smooth 0->1 ramp. A hard switch between the displacement and mixing phases
 * would step the exhaust boundary's composition inside one sub-step, and a step
 * at a solver boundary rings; the classical two-phase model's corner is not
 * physical anyway, since the front arrives over a finite spread.
 *
 * The ramp's WIDTH is the transit time, not the rest of the window. Once the
 * front reaches the exhaust port the loop has broken down, and a turbulent jet
 * mixes over roughly the same loop-length/jet-velocity timescale it needed to
 * arrive. Spreading the breakdown over the whole remaining window instead gave
 * a lugged engine a gentle ramp across 90 % of a 10.7 ms window, where a
 * through-flowing cylinder is fully mixed (or worse) for almost all of it:
 * trapping at 2,000 rpm came out 0.73 against a measured 0.45, and torque
 * 48.8 N.m against about 29 (journal 2026-10-21). */
[[nodiscard]] constexpr double smoothRamp(double t) noexcept {
    if (!(t > 0.0)) return 0.0;
    if (t >= 1.0) return 1.0;
    return t * t * (3.0 - 2.0 * t);
}

/** Open a scavenging window.
 *
 * `jetVelocityMps` is taken once, here, from the crankcase-to-cylinder
 * pressure difference -- not from the port mass flow, which this bias goes on
 * to shape.
 */
inline void openScavengeWindow(TwoStrokeScavengeState& state,
    double transferOpenDegrees, double rpm, double loopLengthM,
    double jetVelocityMps) noexcept {
    state.elapsedSeconds = 0.0;
    state.open = true;
    state.windowSeconds = (rpm > 0.0 && transferOpenDegrees > 0.0)
        ? transferOpenDegrees / (6.0 * rpm)
        : 0.0;
    state.transitSeconds = (jetVelocityMps > 0.0 && loopLengthM > 0.0)
        ? loopLengthM / jetVelocityMps
        : 0.0;
}

/** Advance the open window by one sub-step. */
inline void advanceScavengeWindow(
    TwoStrokeScavengeState& state, double dtSeconds) noexcept {
    if (state.open && dtSeconds > 0.0) state.elapsedSeconds += dtSeconds;
}

inline void closeScavengeWindow(TwoStrokeScavengeState& state) noexcept {
    state.open = false;
    state.elapsedSeconds = 0.0;
}

/** How much of the exhaust outflow is drawn from the fully mixed cylinder
 * rather than from its burned gas, in [0, 1].
 *
 * 0 is perfect displacement: the port is passing burned gas only, and every
 * molecule of fresh charge admitted so far is kept. 1 is perfect mixing, the
 * behaviour the simulator had unconditionally. The window runs from 0 towards 1
 * as the front arrives and the loop breaks down.
 *
 * Returns 1 for a closed or degenerate window, so an engine whose geometry is
 * unknown behaves exactly as it did before this existed.
 */
[[nodiscard]] inline double scavengeMixedFraction(
    const TwoStrokeScavengeState& state) noexcept {
    if (!state.open || !(state.windowSeconds > 0.0)) return 1.0;
    const auto displacement =
        std::clamp(state.transitSeconds / state.windowSeconds, 0.0, 1.0);
    // A window entirely inside the transit time never breaks down: the front
    // has not arrived by the time the ports shut, which is pure displacement.
    if (displacement >= 1.0) return 0.0;
    // No loop at all -- an unknown geometry, or a jet with no pressure behind
    // it. There is nothing to displace with, so the cylinder mixes exactly as
    // it did before this model existed. Guarded rather than divided: the ramp
    // below would otherwise be 0/0 on the window's first sub-step.
    if (!(displacement > 0.0)) return 1.0;
    const auto progress = state.elapsedSeconds / state.windowSeconds;
    // Displacement until the front arrives, then mixing established over one
    // further transit time -- the jet's own mixing timescale, not the window's
    // leftovers. A short window therefore ends mid-breakdown and keeps more
    // charge, which is the engine-speed dependence the whole model is for.
    return smoothRamp((progress - displacement) / displacement);
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
