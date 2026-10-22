#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

namespace enginelab {

struct CarburettorState final {
    /** Fuel wetting the bore and the crankcase walls, waiting to evaporate. */
    double wallFilmMoles { 0.0 };
};

struct CarburettorMetering final {
    /** Fuel that left the jets this step. */
    double meteredMoles { 0.0 };
    /** Fuel that reached the charge this step: what the jets passed less what
     * the walls took, plus what the film gave back. */
    double deliveredMoles { 0.0 };
    double mainMoles { 0.0 };
    double pilotMoles { 0.0 };
    double venturiDepressionKpa { 0.0 };
    double venturiVelocityMps { 0.0 };
};

/** Fuel metered by the air itself, the way a slide carburettor does it.
 *
 * Nothing here is commanded. The jets are orifices into a float bowl held at
 * the pressure outside the engine, and the mixture is whatever depression the
 * air happens to draw through them. That is the whole point of modelling it:
 * an injector asked for the right fuel and got it, so a carburetted engine's
 * real mixture excursions -- lean off idle, rich as the slide comes up, and a
 * stumble on a fast tip-in while the bore is still wet -- cannot come out of
 * an injector model no matter how it is calibrated.
 *
 * The two circuits never need blending, because each is driven by a
 * depression the other does not see. The main circuit is driven by the
 * venturi's own depression, which goes as the square of the throat velocity
 * and is worth tens of pascals at idle -- less than the lift of the jet above
 * the bowl, so it passes nothing at all. The pilot circuit is driven by the
 * drop from the bowl to the crankcase, which is largest with the slide shut
 * and collapses at full throttle, where a two-stroke's crankcase pumps at
 * about ambient. Each fades where the other takes over, with no crossover
 * term written anywhere.
 *
 * Two things a real carburettor has are deliberately absent, and both would
 * show up at the top of the range rather than at idle:
 *
 * - **No emulsion tube or air corrector.** A real main circuit bleeds air
 *   into the fuel passage once the depression uncovers the tube's holes, which
 *   is what keeps the mixture flat at high flow; a plain jet richens instead.
 *   The flattening is a two-phase effect in the passage, and no single
 *   algebraic factor stands in for it honestly, so the model has the
 *   plain-jet characteristic and this is a measured gap, not a calibration.
 * - **No throat density drop.** The air density used is the one handed in, so
 *   the compressible fall across the venturi (a few per cent at full throttle)
 *   is not there. It is the other half of why a plain jet richens.
 */
class CarburettorModel final {
public:
    /** Throat area in square metres; zero configured diameter takes its
     *  fraction of the throttle bore. */
    [[nodiscard]] static double venturiAreaM2(const CarburettorConfig& carburettor,
                                              double throttleDiameterMm) noexcept;

    /** Orifice area of a jet stamped with a flow number.
     *
     * A jet's number is cubic centimetres per minute of test fluid under one
     * metre of head. The velocity that head produces is sqrt(2 g h), which is
     * 4.43 m/s and carries no fluid density at all, so the area a number
     * implies is independent of what the jet was calibrated with.
     */
    [[nodiscard]] static double jetAreaM2(double flowNumber,
                                          double dischargeCoefficient) noexcept;

    /** How far the needle has withdrawn from the needle jet, 0 to 1.
     *
     * On a slide carburettor the needle rides with the slide, so over roughly
     * the first three quarters of the opening it is the needle taper and not
     * the main jet that meters. The clip position shifts the whole curve: a
     * lower clip holds the needle higher and richens everything below full
     * throttle without touching the top.
     */
    [[nodiscard]] static double needleOpenFraction(const CarburettorConfig& carburettor,
                                                   double throttleFraction) noexcept;

    /** Meter the fuel for one solver sub-step.
     *
     * `airMassFlowKgPerSecond` is the air actually crossing the bore, and the
     * fuel follows from the depression that moved it, so this is an amount per
     * step and never a gain on a flow it goes on to change.
     */
    [[nodiscard]] static CarburettorMetering meter(const CarburettorConfig& carburettor,
                                                   const FuelConfig& fuel,
                                                   CarburettorState& state,
                                                   double airMassFlowKgPerSecond,
                                                   double airDensityKgPerM3,
                                                   double bowlPressureKpa,
                                                   double crankcasePressureKpa,
                                                   double throttleDiameterMm,
                                                   double throttleFraction,
                                                   double dtSeconds) noexcept;
};

} // namespace enginelab
