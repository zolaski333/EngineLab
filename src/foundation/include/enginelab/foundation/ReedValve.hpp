#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace enginelab {

/**
 * A reed valve's petals, as the user measured or designed them. With no
 * petal (`petalCount` 0) the reed is the perfect check valve the simulator
 * always had: no lift, no inertia, no backflow, bit for bit.
 *
 * What the model is, and where each part comes from:
 *
 * - One degree of freedom per reed block, the petals' tip lift: a mass, a
 *   spring and a damper. Dave et al. (SAE 2004-01-1455) found this
 *   representation adequate for an engine simulation though it is not a
 *   bending beam; Fleck, Cartwright & Thornhill (SAE 972738) and Ortenzi &
 *   Bossaglia (Energies 16, 4947, 2023) use a beam's first modes instead.
 *   Their measured lifts are not openly published, so this model has not been
 *   checked against a measured lift here (a blocker, docs/simulation-model.md).
 * - Stiffness and mass from Euler-Bernoulli cantilever theory (validated
 *   structural physics): the spring is the tip stiffness under a uniform
 *   pressure, 8 E I / L^3 for the force Δp b L, so the static lift is exact;
 *   the mass is that stiffness over the first bending mode's squared
 *   frequency, (1.8751^2)^2 E I / (rho A L^4), so the free vibration is too.
 *   Matching those two together is this model's choice (an assumption).
 * - The damping ratio is an empirical input (structural and gas damping
 *   together); no value is derived.
 * - The pressure force is the drop across the reed over the petal's face,
 *   quasi-static: no jet momentum, no pressure distribution along the petal,
 *   no preload (assumptions).
 * - Seat and stop plate are inelastic contacts: the petal stops dead
 *   (assumption; no restitution, no stop-plate curvature).
 * - Flow area: the curtain the lift opens, the tip edge b x plus the two
 *   sides' triangles L x / 2 each (a straight petal, an approximation of the
 *   bent one), capped by the reed block's window, times an authored discharge
 *   coefficient. Gas flows both ways while the petal is off its seat.
 */
struct ReedValveConfig final {
    /** Petals on the block, all alike; 0: the perfect check valve. */
    int petalCount { 0 };
    /** One petal's free (bending) length from its clamp to its tip. */
    double freeLengthMm { 0.0 };
    double widthMm { 0.0 };
    double thicknessMm { 0.0 };
    /** The petal material's Young's modulus (spring steel about 200 GPa,
     *  glass or carbon fibre composites 20-60 GPa) and density. */
    double youngsModulusGpa { 0.0 };
    double densityKgPerM3 { 0.0 };
    /** Damping over critical, 0 to below 1 (empirical). */
    double dampingRatio { 0.0 };
    /** The stop plate's lift at the tip. */
    double stopLiftMm { 0.0 };
    /** The block's window area under all its petals; 0: the petals' faces. */
    double windowAreaMm2 { 0.0 };
    double dischargeCoefficient { 0.0 };
};

[[nodiscard]] inline bool hasDynamicReed(const ReedValveConfig& reed) noexcept {
    return reed.petalCount > 0;
}

/** Why an authored reed cannot run, or nullptr. */
[[nodiscard]] inline const char* reedValveConfigError(const ReedValveConfig& reed) noexcept {
    if (reed.petalCount < 0 || reed.petalCount > 16) return "reed petal count must be 0-16";
    if (!hasDynamicReed(reed)) return nullptr;
    const auto positive = [](double value) { return std::isfinite(value) && value > 0.0; };
    if (!positive(reed.freeLengthMm) || !positive(reed.widthMm) || !positive(reed.thicknessMm))
        return "reed petal length, width and thickness must be positive";
    if (!(reed.thicknessMm < reed.freeLengthMm) || !(reed.thicknessMm < reed.widthMm))
        return "a reed petal must be thinner than it is long and wide";
    if (!positive(reed.youngsModulusGpa) || !positive(reed.densityKgPerM3))
        return "reed material modulus and density must be positive";
    if (!std::isfinite(reed.dampingRatio) || reed.dampingRatio < 0.0 || !(reed.dampingRatio < 1.0))
        return "reed damping ratio must be 0 to below 1";
    if (!positive(reed.stopLiftMm)) return "reed stop-plate lift must be positive";
    if (!std::isfinite(reed.windowAreaMm2) || reed.windowAreaMm2 < 0.0)
        return "reed window area must be 0 or positive";
    if (!positive(reed.dischargeCoefficient) || reed.dischargeCoefficient > 1.0)
        return "reed discharge coefficient must be above 0 and at most 1";
    return nullptr;
}

/** The single-degree-of-freedom petal the config describes, SI units. */
struct ReedValveDynamics final {
    double stiffnessNPerM { 0.0 };
    double massKg { 0.0 };
    double dampingNsPerM { 0.0 };
    double naturalFrequencyHz { 0.0 };
    double faceAreaM2 { 0.0 };
    double stopLiftM { 0.0 };
    double windowAreaM2 { 0.0 };
    /** Curtain area per metre of tip lift, all petals. */
    double curtainPerLiftM { 0.0 };
    double dischargeCoefficient { 0.0 };
};

/** The first cantilever mode's eigenvalue, beta1 L. */
inline constexpr double cantileverFirstModeBetaL = 1.875104068711961;

[[nodiscard]] inline ReedValveDynamics reedValveDynamics(const ReedValveConfig& reed) noexcept {
    ReedValveDynamics result;
    if (!hasDynamicReed(reed)) return result;
    const auto petals = static_cast<double>(reed.petalCount);
    const auto length = reed.freeLengthMm * 1.0e-3;
    const auto width = reed.widthMm * 1.0e-3;
    const auto thickness = reed.thicknessMm * 1.0e-3;
    const auto modulus = reed.youngsModulusGpa * 1.0e9;
    const auto inertia = width * thickness * thickness * thickness / 12.0;
    const auto betaSquared = cantileverFirstModeBetaL * cantileverFirstModeBetaL;
    const auto omega = betaSquared * std::sqrt(modulus * inertia
        / (reed.densityKgPerM3 * width * thickness * length * length * length * length));
    // The block's petals act together: their stiffnesses, masses and faces add.
    result.stiffnessNPerM = petals * 8.0 * modulus * inertia / (length * length * length);
    result.massKg = result.stiffnessNPerM / (omega * omega);
    result.dampingNsPerM = 2.0 * reed.dampingRatio * std::sqrt(result.stiffnessNPerM * result.massKg);
    result.naturalFrequencyHz = omega / (2.0 * 3.14159265358979323846);
    result.faceAreaM2 = petals * width * length;
    result.stopLiftM = reed.stopLiftMm * 1.0e-3;
    result.windowAreaM2 = reed.windowAreaMm2 > 0.0 ? reed.windowAreaMm2 * 1.0e-6 : result.faceAreaM2;
    result.curtainPerLiftM = petals * (width + length);
    result.dischargeCoefficient = reed.dischargeCoefficient;
    return result;
}

/** The petals' tip lift and its rate. */
struct ReedValveState final {
    double liftM { 0.0 };
    double velocityMPerS { 0.0 };
};

/** Geometric flow area (the discharge coefficient not applied). */
[[nodiscard]] inline double reedValveFlowAreaM2(const ReedValveDynamics& reed,
                                                const ReedValveState& state) noexcept {
    return std::min(reed.windowAreaM2, reed.curtainPerLiftM * std::max(0.0, state.liftM));
}

/**
 * Advances the petal over `dtSeconds` under a pressure drop held constant
 * over the step (`dropPa`, positive towards opening). The linear oscillator
 * is integrated exactly for a constant force (underdamped, so stable at any
 * step), then the seat and the stop plate stop it dead where it reached them.
 * Returns the new state.
 */
[[nodiscard]] inline ReedValveState advanceReedValve(const ReedValveDynamics& reed, ReedValveState state,
                                                     double dropPa, double dtSeconds) noexcept {
    if (!(reed.massKg > 0.0) || !(dtSeconds > 0.0)) return state;
    // Seated and pushed onto the seat, or at the stop and pushed onto it:
    // the contact holds and nothing moves.
    const auto force = dropPa * reed.faceAreaM2;
    if (state.liftM <= 0.0 && force <= 0.0) return {};
    if (state.liftM >= reed.stopLiftM && force >= reed.stiffnessNPerM * reed.stopLiftM)
        return { reed.stopLiftM, 0.0 };
    const auto omega = std::sqrt(reed.stiffnessNPerM / reed.massKg);
    const auto zeta = reed.dampingNsPerM / (2.0 * std::sqrt(reed.stiffnessNPerM * reed.massKg));
    const auto omegaD = omega * std::sqrt(std::max(1.0e-12, 1.0 - zeta * zeta));
    const auto equilibrium = force / reed.stiffnessNPerM;
    const auto x0 = state.liftM - equilibrium;
    const auto v0 = state.velocityMPerS;
    const auto decay = std::exp(-zeta * omega * dtSeconds);
    const auto c = std::cos(omegaD * dtSeconds);
    const auto s = std::sin(omegaD * dtSeconds);
    const auto b = (v0 + zeta * omega * x0) / omegaD;
    ReedValveState next;
    next.liftM = equilibrium + decay * (x0 * c + b * s);
    // d/dt [e^{-a t}(x0 cos + b sin)] = e^{-a t}(-a (x0 cos + b sin) - x0 wd sin + b wd cos).
    next.velocityMPerS = decay * (-zeta * omega * (x0 * c + b * s) - x0 * omegaD * s + b * omegaD * c);
    if (next.liftM <= 0.0) return {};
    if (next.liftM >= reed.stopLiftM) return { reed.stopLiftM, 0.0 };
    return next;
}

/**
 * The share of a pressure drop across a carburettor and a reed in series
 * that the reed sees: quasi-steady incompressible orifices carry one flow,
 * so each drop goes as 1 / (Cd A)^2 (an approximation: the gas between
 * them has no volume). A shut reed holds the whole drop.
 */
[[nodiscard]] inline double reedShareOfSeriesDrop(double carburettorEffectiveAreaM2,
                                                  double reedEffectiveAreaM2) noexcept {
    if (!(reedEffectiveAreaM2 > 0.0)) return 1.0;
    if (!(carburettorEffectiveAreaM2 > 0.0)) return 0.0;
    const auto carb2 = carburettorEffectiveAreaM2 * carburettorEffectiveAreaM2;
    const auto reed2 = reedEffectiveAreaM2 * reedEffectiveAreaM2;
    return carb2 / (carb2 + reed2);
}

} // namespace enginelab
