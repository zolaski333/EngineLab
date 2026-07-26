#include <enginelab/physics/FuelInjectionModel.hpp>

#include <algorithm>
#include <cmath>

namespace enginelab {

FuelInjectionResult FuelInjectionModel::deliver(const InjectionConfig& injection,
                                                 const FuelConfig& fuel,
                                                 FuelInjectionState& state,
                                                 GasCell& target,
                                                 double commandedMoles,
                                                 double dtSeconds) noexcept {
    FuelInjectionResult result;
    if (dtSeconds <= 0.0 || !std::isfinite(dtSeconds)) return result;
    const auto fuelMolarMassKg = fuel.molarMassGramsPerMole * 0.001;
    const auto railPressureKpa = injection.railPressureBar * 100.0;
    // Port-fuel rails are manifold referenced: their configured pressure is the
    // regulated pressure drop across the injector, not an absolute rail
    // pressure. Direct-injection rails remain absolute because the injector
    // discharges into a cylinder whose pressure changes throughout the cycle.
    const auto referenceDeltaKpa = std::max(10.0, injection.referencePressureBar * 100.0);
    const auto actualDeltaKpa = injection.mode == InjectionMode::port
        ? std::max(0.0, railPressureKpa)
        : std::max(0.0, railPressureKpa - target.pressureKpa());
    const auto pressureFlowRatio = std::sqrt(actualDeltaKpa / referenceDeltaKpa);
    const auto capacityMoles = injection.injectorFlowMgPerSecond * pressureFlowRatio
        * dtSeconds * 1.0e-6 / std::max(1.0e-9, fuelMolarMassKg);
    result.meteredMoles = std::min(std::max(0.0, commandedMoles), capacityMoles);

    double newlyVaporisedMoles = 0.0;
    double newlyEntrainedMoles = 0.0;
    if (injection.mode == InjectionMode::port) {
        const auto filmFraction = std::clamp(injection.wallFilmFraction, 0.0, 0.98);
        state.liquidFilmMoles += result.meteredMoles * filmFraction;
        newlyVaporisedMoles += result.meteredMoles * (1.0 - filmFraction);
        const auto timeConstant = std::max(0.001, injection.vaporisationTimeConstantSeconds);
        const auto temperatureFactor = std::clamp((target.temperatureK() - 240.0) / 120.0, 0.08, 2.0);
        const auto evaporatedFilm = state.liquidFilmMoles
            * (1.0 - std::exp(-dtSeconds * temperatureFactor / timeConstant));
        state.liquidFilmMoles = std::max(0.0, state.liquidFilmMoles - evaporatedFilm);
        newlyVaporisedMoles += evaporatedFilm;
        newlyEntrainedMoles = newlyVaporisedMoles;
    } else {
        state.directLiquidSprayMoles += result.meteredMoles;
        const auto vaporisationTime =
            injection.directSprayVaporisationTimeConstantSeconds;
        if (vaporisationTime <= 0.0) {
            newlyVaporisedMoles = state.directLiquidSprayMoles;
        } else {
            // Droplet heating and vaporisation accelerate strongly as the
            // compressed charge warms. The bounded factor retains finite cold
            // operation while avoiding a solver-rate-dependent threshold.
            const auto temperatureFactor = std::clamp(
                (target.temperatureK() - 240.0) / 560.0, 0.04, 2.5);
            newlyVaporisedMoles = state.directLiquidSprayMoles
                * (1.0 - std::exp(-dtSeconds * temperatureFactor
                    / vaporisationTime));
        }
        newlyVaporisedMoles = std::clamp(newlyVaporisedMoles, 0.0,
                                          state.directLiquidSprayMoles);
        state.directLiquidSprayMoles -= newlyVaporisedMoles;
        state.directDispersingVapourMoles += newlyVaporisedMoles;

        const auto entrainmentTime =
            injection.directSprayEntrainmentTimeConstantSeconds;
        newlyEntrainedMoles = entrainmentTime <= 0.0
            ? state.directDispersingVapourMoles
            : state.directDispersingVapourMoles
                * (1.0 - std::exp(-dtSeconds / entrainmentTime));
        newlyEntrainedMoles = std::clamp(newlyEntrainedMoles, 0.0,
                                          state.directDispersingVapourMoles);
        state.directDispersingVapourMoles -= newlyEntrainedMoles;
    }

    if (newlyEntrainedMoles > 0.0) {
        target.injectFuelMoles(newlyEntrainedMoles,
                               injection.fuelTemperatureC + 273.15);
    }
    if (newlyVaporisedMoles > 0.0) {
        const auto latentHeatJ = newlyVaporisedMoles * fuelMolarMassKg
            * injection.latentHeatKjPerKg * 1'000.0;
        const auto coolingEfficiency = injection.mode == InjectionMode::direct
            ? injection.directChargeCoolingEfficiency : injection.portChargeCoolingEfficiency;
        result.chargeCoolingJoules = latentHeatJ * std::clamp(coolingEfficiency, 0.0, 1.0);
        target.addHeatJoules(-result.chargeCoolingJoules);
    }
    result.vaporisedMoles = newlyVaporisedMoles;
    result.entrainedMoles = newlyEntrainedMoles;
    result.liquidSprayMoles = state.directLiquidSprayMoles;
    result.dispersingVapourMoles = state.directDispersingVapourMoles;
    return result;
}

double FuelInjectionModel::updateClosedLoopTrim(const InjectionConfig& injection,
                                                double rpm,
                                                double measuredAirFuelRatio,
                                                double targetAirFuelRatio,
                                                double currentTrim) noexcept {
    if (!std::isfinite(currentTrim)) return 1.0;
    if (!std::isfinite(rpm) || !std::isfinite(measuredAirFuelRatio)
            || !std::isfinite(targetAirFuelRatio)
            || rpm <= 0.0 || measuredAirFuelRatio <= 0.0
            || targetAirFuelRatio <= 0.0 || currentTrim <= 0.0) {
        return std::clamp(currentTrim, 0.55, 2.20);
    }

    const auto cycleDurationSeconds = 120.0 / std::max(450.0, rpm);
    const auto transportDelaySeconds = injection.mode == InjectionMode::port
        ? std::max(0.0, injection.vaporisationTimeConstantSeconds) : 0.0;
    const auto loopTimeConstantSeconds = injection.mode == InjectionMode::port
        ? std::max(0.45, transportDelaySeconds * 8.0)
        : 0.25;
    const auto integralGain = std::clamp(
        cycleDurationSeconds / loopTimeConstantSeconds, 0.015, 0.15);
    const auto logarithmicError = std::clamp(
        std::log(measuredAirFuelRatio / targetAirFuelRatio), -0.70, 0.70);
    const auto correction = std::clamp(logarithmicError * integralGain, -0.05, 0.05);
    return std::clamp(currentTrim * std::exp(correction), 0.55, 2.20);
}

} // namespace enginelab
