#pragma once
#include <algorithm>
#include <cmath>

namespace enginelab {

/** Finite-amplitude (nonlinear) propagation correction for duct waveguides.
 *
 * Exhaust ducts do not carry small-signal acoustics. The in-duct levels behind
 * a blowdown front are in the kilopascal range (150-175 dB SPL), where the
 * local propagation speed of a simple wave is amplitude dependent:
 *
 *     dx/dt = c + beta * u,   beta = (gamma + 1) / 2,   u = p' / (rho * c)
 *
 * Crests outrun troughs, wavefronts steepen, and the steepening pumps energy
 * up the harmonic series -- the physical origin of the "bark"/"crackle" of an
 * unmuffled exhaust, and the same mechanism as brassiness in trombones and
 * trumpets (Hirschberg et al., JASA 1996; Msallam et al., Acta Acustica 2000,
 * both model it exactly this way: a delay line whose delay is modulated by the
 * signal's own amplitude). A linear delay line, whatever its excitation,
 * cannot produce this cascade; without it every engine's pulse train keeps
 * the smooth low-passed shape the band-limited boundary telemetry gave it,
 * which listeners describe as muffled and "all engines sound the same".
 *
 * Implementation: a wave sample that has just traversed a line of nominal
 * delay D arrives instead at D * delayScale(p'), where
 *
 *     delayScale = 1 / (1 + beta * p' / (rho * c^2))
 *
 * since p'/(rho*c) is the acoustic particle velocity of a simple wave and the
 * transit time scales inversely with propagation speed. This is a pure time
 * warp: it moves samples in time and adds no energy, so it is passive by
 * construction and exactly transparent as p' -> 0. Shock capture is implicit:
 * once crests try to overtake the sampling grid, the fractional-delay
 * interpolation dissipates the overturning region, which is the discrete
 * analogue of weak-shock entropy loss.
 *
 * The caller supplies rho*c^2 (== gamma * p0) of the duct medium, which the
 * physical exhaust path already tracks per runner and per collector. The
 * scale is clamped: beyond +-30% transit-time modulation the first-order
 * simple-wave relation is no longer trustworthy and the clamp keeps reads
 * inside the allocated line.
 */
namespace NonlinearDuctAcoustics {

/** beta = (gamma + 1) / 2 for hot exhaust gas (gamma ~= 1.35). */
inline constexpr float coefficientOfNonlinearity = 1.175F;

/** Multiplier on the nominal transit delay for a wave sample of acoustic
 *  pressure acousticPressurePa in a medium of stiffness rhoC2 = rho * c^2.
 *  Returns 1 exactly for zero amplitude or an invalid medium. */
[[nodiscard]] inline float delayScale(float acousticPressurePa, float rhoC2) noexcept {
    if (!(rhoC2 > 1.0F) || !std::isfinite(acousticPressurePa)) return 1.0F;
    const auto machNumber = std::clamp(
        coefficientOfNonlinearity * acousticPressurePa / rhoC2, -0.30F, 0.30F);
    return 1.0F / (1.0F + machNumber);
}

/** Finite-amplitude read from a delay line: probe the wave at the nominal
 *  (linear) delay, then re-read at the amplitude-corrected arrival time. One
 *  probe iteration of the implicit arrival relation is accurate to O(Mach^2),
 *  which the clamp in delayScale already bounds. readDelayed(delaySamples) is
 *  the caller's interpolated read on its own line storage; it is invoked once
 *  for the probe and once for the corrected read, so an unsteepened medium
 *  (rhoC2 <= 1) reproduces the linear read bit-exactly. */
template <typename ReadDelayed>
[[nodiscard]] inline float steepenedRead(ReadDelayed&& readDelayed, float delaySamples,
                                         float delayLimit, float stiffnessRhoC2) noexcept {
    const auto probe = readDelayed(delaySamples);
    const auto warped = std::clamp(
        delaySamples * delayScale(probe, stiffnessRhoC2), 1.0F, delayLimit);
    return readDelayed(warped);
}

} // namespace NonlinearDuctAcoustics
} // namespace enginelab
