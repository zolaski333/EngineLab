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
    const auto referenceDeltaKpa = std::max(10.0, injection.referencePressureBar * 100.0 - 101.325);
    const auto actualDeltaKpa = std::max(0.0, railPressureKpa - target.pressureKpa());
    const auto pressureFlowRatio = std::sqrt(actualDeltaKpa / referenceDeltaKpa);
    const auto capacityMoles = injection.injectorFlowMgPerSecond * pressureFlowRatio
        * dtSeconds * 1.0e-6 / std::max(1.0e-9, fuelMolarMassKg);
    result.meteredMoles = std::min(std::max(0.0, commandedMoles), capacityMoles);

    double newlyVaporisedMoles = 0.0;
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
    } else {
        newlyVaporisedMoles = result.meteredMoles;
    }

    if (newlyVaporisedMoles > 0.0) {
        target.injectFuelMoles(newlyVaporisedMoles, injection.fuelTemperatureC + 273.15);
        const auto latentHeatJ = newlyVaporisedMoles * fuelMolarMassKg
            * injection.latentHeatKjPerKg * 1'000.0;
        const auto coolingEfficiency = injection.mode == InjectionMode::direct
            ? injection.directChargeCoolingEfficiency : injection.portChargeCoolingEfficiency;
        result.chargeCoolingJoules = latentHeatJ * std::clamp(coolingEfficiency, 0.0, 1.0);
        target.addHeatJoules(-result.chargeCoolingJoules);
    }
    result.vaporisedMoles = newlyVaporisedMoles;
    return result;
}

} // namespace enginelab
