#include <enginelab/physics/CarburettorModel.hpp>

#include <algorithm>
#include <cmath>

namespace enginelab {
namespace {

constexpr double gravityMPerS2 = 9.80665;
/** Velocity a one-metre head produces, sqrt(2 g h), which is what a jet's
 *  stamped flow number is measured at. */
const double jetReferenceVelocityMps = std::sqrt(2.0 * gravityMPerS2);
/** Nominal air-screw setting the pilot's flow number is taken to belong to. */
constexpr double nominalAirScrewTurns = 1.5;
/** Opening by which the needle has fully withdrawn and the main jet meters. */
constexpr double needleFullyLiftedThrottle = 0.75;
/** Needle-jet area still open with the slide shut. */
constexpr double needleShutFraction = 0.18;
/** Throttle shift per clip position away from the middle one. */
constexpr double throttleShiftPerClip = 0.06;

[[nodiscard]] double smoothStep(double t) noexcept {
    if (!(t > 0.0)) return 0.0;
    if (t >= 1.0) return 1.0;
    return t * t * (3.0 - 2.0 * t);
}

[[nodiscard]] double orificeMassFlowKgPerSecond(double areaM2,
                                                double dischargeCoefficient,
                                                double densityKgPerM3,
                                                double pressureDropPa) noexcept {
    if (!(areaM2 > 0.0) || !(densityKgPerM3 > 0.0) || !(pressureDropPa > 0.0))
        return 0.0;
    return dischargeCoefficient * areaM2
        * std::sqrt(2.0 * densityKgPerM3 * pressureDropPa);
}

} // namespace

double CarburettorModel::venturiAreaM2(const CarburettorConfig& carburettor,
                                       double throttleDiameterMm) noexcept {
    auto diameterMm = carburettor.venturiDiameterMm;
    if (!(diameterMm > 0.0)) diameterMm = 0.80 * std::max(0.0, throttleDiameterMm);
    if (!(diameterMm > 0.0)) return 0.0;
    const auto diameterM = diameterMm * 0.001;
    return 0.25 * 3.14159265358979323846 * diameterM * diameterM;
}

double CarburettorModel::jetAreaM2(double flowNumber,
                                   double dischargeCoefficient) noexcept {
    if (!(flowNumber > 0.0)) return 0.0;
    const auto coefficient = std::clamp(dischargeCoefficient, 0.05, 1.0);
    // Flow number is cc/min; the reference velocity carries no density.
    const auto referenceFlowM3PerSecond = flowNumber * 1.0e-6 / 60.0;
    return referenceFlowM3PerSecond / (coefficient * jetReferenceVelocityMps);
}

double CarburettorModel::needleOpenFraction(const CarburettorConfig& carburettor,
                                            double throttleFraction) noexcept {
    const auto clip = std::clamp(carburettor.needleClipPosition, 1.0, 5.0);
    // A lower clip number holds the needle higher, which is the same thing as
    // the slide already being further open as far as the taper is concerned.
    const auto shifted = std::clamp(throttleFraction, 0.0, 1.0)
        + (3.0 - clip) * throttleShiftPerClip;
    const auto open = smoothStep(std::clamp(shifted, 0.0, 1.0)
        / needleFullyLiftedThrottle);
    return needleShutFraction + (1.0 - needleShutFraction) * open;
}

CarburettorMetering CarburettorModel::meter(const CarburettorConfig& carburettor,
                                             const FuelConfig& fuel,
                                             CarburettorState& state,
                                             double airMassFlowKgPerSecond,
                                             double airDensityKgPerM3,
                                             double bowlPressureKpa,
                                             double crankcasePressureKpa,
                                             double throttleDiameterMm,
                                             double throttleFraction,
                                             double dtSeconds,
                                             double idleCircuitOpening,
                                             double starterCircuitOpening) noexcept {
    CarburettorMetering result;
    if (!(dtSeconds > 0.0) || !std::isfinite(dtSeconds)) return result;

    const auto fuelDensityKgPerM3 = std::max(1.0, fuel.densityKgPerL * 1'000.0);
    const auto fuelMolarMassKg = std::max(1.0e-9, fuel.molarMassGramsPerMole * 0.001);
    const auto coefficient = std::clamp(carburettor.jetDischargeCoefficient, 0.05, 1.0);

    // The lift of the jet above the bowl is a real head the depression has to
    // beat before anything leaves the jet at all. It is only tens of pascals,
    // which is exactly why it decides the idle and nothing else.
    const auto liftPressurePa = fuelDensityKgPerM3 * gravityMPerS2
        * std::max(0.0, carburettor.jetLiftMm) * 0.001;

    // Main circuit: the venturi's own depression, from the velocity the air
    // crossing the throat must have.
    const auto throatAreaM2 = venturiAreaM2(carburettor, throttleDiameterMm);
    if (throatAreaM2 > 0.0 && airDensityKgPerM3 > 0.0
        && airMassFlowKgPerSecond > 0.0) {
        result.venturiVelocityMps =
            airMassFlowKgPerSecond / (airDensityKgPerM3 * throatAreaM2);
        const auto depressionPa = 0.5 * airDensityKgPerM3
            * result.venturiVelocityMps * result.venturiVelocityMps;
        result.venturiDepressionKpa = depressionPa * 0.001;
        const auto mainAreaM2 = jetAreaM2(carburettor.mainJetFlowNumber, coefficient)
            * (carburettor.fixedVenturi ? 1.0 : needleOpenFraction(carburettor, throttleFraction));
        const auto mainFlowKgPerSecond = orificeMassFlowKgPerSecond(
            mainAreaM2, coefficient, fuelDensityKgPerM3,
            depressionPa - liftPressurePa);
        result.mainMoles = mainFlowKgPerSecond * dtSeconds / fuelMolarMassKg;
    }

    // Pilot circuit: the drop from the bowl to the crankcase, whatever the
    // slide is doing. No taper is applied as the slide opens, because none is
    // needed: a two-stroke's crankcase pumps at about ambient at full
    // throttle, so this depression puts the circuit out of business by itself.
    const auto pilotDropPa = (bowlPressureKpa - crankcasePressureKpa) * 1'000.0;
    const auto airScrewTurns = std::clamp(carburettor.airScrewTurns, 0.25, 4.0);
    // Bleeding air into the pilot passage leans it; area falls as the square
    // root so that a doubled bleed is a halved head rather than a halved flow.
    const auto airScrewFactor = std::sqrt(nominalAirScrewTurns / airScrewTurns);
    const auto pilotAreaM2 = jetAreaM2(carburettor.pilotJetFlowNumber, coefficient)
        * std::max(0.0, carburettor.pilotCircuitFactor) * airScrewFactor
        * (std::isfinite(idleCircuitOpening) ? std::clamp(idleCircuitOpening, 0.0, 4.0) : 1.0);
    const auto pilotFlowKgPerSecond = orificeMassFlowKgPerSecond(
        pilotAreaM2, coefficient, fuelDensityKgPerM3, pilotDropPa - liftPressurePa);
    result.pilotMoles = pilotFlowKgPerSecond * dtSeconds / fuelMolarMassKg;

    // Starter circuit: a plain jet on the same drop, open as far as the
    // start aid's extra air is.
    if (carburettor.starterJetFlowNumber > 0.0 && starterCircuitOpening > 0.0) {
        const auto starterAreaM2 = jetAreaM2(carburettor.starterJetFlowNumber, coefficient)
            * std::min(starterCircuitOpening, 4.0);
        result.starterMoles = orificeMassFlowKgPerSecond(starterAreaM2, coefficient,
            fuelDensityKgPerM3, pilotDropPa - liftPressurePa) * dtSeconds / fuelMolarMassKg;
    }

    // Accelerator pump: the throttle's rise strokes a piston that displaces
    // its share of the pump's volume, and the nozzle delivers that over its
    // own time. A closing throttle refills it and delivers nothing.
    const auto throttle = std::clamp(throttleFraction, 0.0, 1.0);
    if (carburettor.acceleratorPumpStrokeCc > 0.0) {
        const auto rise = std::max(0.0, throttle - state.previousThrottle);
        state.pumpShotMoles += rise * carburettor.acceleratorPumpStrokeCc * 1.0e-6
            * fuelDensityKgPerM3 / fuelMolarMassKg;
        result.pumpMoles = state.pumpShotMoles * (1.0 - std::exp(-dtSeconds
            / std::max(1.0e-3, carburettor.acceleratorPumpTimeConstantSeconds)));
        state.pumpShotMoles = std::max(0.0, state.pumpShotMoles - result.pumpMoles);
    }
    state.previousThrottle = throttle;

    result.meteredMoles = result.mainMoles + result.pilotMoles + result.pumpMoles
        + result.starterMoles;

    // What the walls take, and what they give back. A carburettor wets its
    // bore and then the crankcase, and the charge gets the difference: this is
    // the stumble on a fast tip-in and the reason an accelerator pump exists.
    const auto filmFraction = std::clamp(carburettor.wallFilmFraction, 0.0, 0.98);
    state.wallFilmMoles += result.meteredMoles * filmFraction;
    const auto timeConstant = std::max(1.0e-4,
        carburettor.wallFilmTimeConstantSeconds);
    const auto returnedMoles = state.wallFilmMoles
        * (1.0 - std::exp(-dtSeconds / timeConstant));
    state.wallFilmMoles = std::max(0.0, state.wallFilmMoles - returnedMoles);
    result.deliveredMoles =
        result.meteredMoles * (1.0 - filmFraction) + returnedMoles;
    return result;
}

} // namespace enginelab
