#include <enginelab/simulation/EngineThermalModel.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace enginelab {

namespace {

constexpr double designAirSpeedMps = 30.0;
constexpr double designTemperatureRiseC = 65.0;
/** Brake efficiency at rated power, to turn power into released heat. */
constexpr double ratedBrakeEfficiency = 0.30;
/** Share of the air in front of a car that crosses its radiator core. */
constexpr double radiatorRamFraction = 0.35;
/** Core air speed of an electric radiator fan. */
constexpr double fanCoreSpeedMps = 2.5;
/** Air an engine-driven blower moves across the fins at redline. */
constexpr double blowerAirSpeedAtRedlineMps = 25.0;
/** A closed thermostat still bleeds a little coolant through the radiator. */
constexpr double thermostatBleed = 0.03;
/** Natural convection through a radiator in still air: about 1.5 % of its
 * design conductance. */
constexpr double stillRadiatorAirFraction = 0.005;
/** An air-cooled engine's fins in still air: the skin term carries it; the
 * temperature stands for the oil's, which climbs at a standstill. */
constexpr double stillFinAirFraction = 0.001;

} // namespace

EngineThermalModel::EngineThermalModel(const EngineConfig& config) noexcept {
    const auto& thermal = config.thermal;
    double displacementLitres = 0.0;
    for (const auto& cylinder : config.cylinders)
        displacementLitres += std::numbers::pi * 0.25 * cylinder.boreMm * cylinder.boreMm * cylinder.strokeMm * 1.0e-6;
    displacementLitres = std::max(0.05, displacementLitres);
    const auto surfaceScale = std::cbrt(displacementLitres * displacementLitres);
    const auto boost = config.forcedInduction.enabled
        ? 0.5 * (1.0 + std::max(1.0, config.forcedInduction.wastegatePressureRatio)) : 1.0;
    redlineRpm_ = std::max(1.0, config.redlineRpm);
    ratedPowerKw_ = 9.0 * displacementLitres * redlineRpm_ / 1'000.0 * boost;
    coolantHeatShare_ = thermal.coolantHeatShare;
    oilHeatShare_ = thermal.oilHeatShare;
    // What a closed thermostat circulates: the water jacket's coolant (about
    // 1.5 L per litre) and the head and cylinder metal around it (about 10 kg
    // of aluminium per litre); the radiator's coolant and the outer cases lag.
    coolantCapacityKjPerC_ = thermal.coolantMassKjPerC > 0.0
        ? thermal.coolantMassKjPerC : 14.0 * displacementLitres + 3.0;
    // About a litre of oil per litre, with the crank and rods it soaks.
    oilCapacityKjPerC_ = thermal.oilMassKjPerC > 0.0
        ? thermal.oilMassKjPerC : 4.0 * displacementLitres + 2.0;
    // An authored conductance keeps the meaning it had in the former model,
    // whose full airflow factor was 1.4. A sized radiator also takes the
    // friction heat, about 15 % of the rated power.
    radiatorKwPerC_ = thermal.coolingPowerKwPerC > 0.0
        ? thermal.coolingPowerKwPerC * 1.4
        : ((coolantHeatShare_ + oilHeatShare_) / ratedBrakeEfficiency + 0.15) * ratedPowerKw_
            / designTemperatureRiseC;
    // Natural convection and radiation, about 12 W/m2K, from about 0.75 m2
    // per litre^(2/3) of outer surface.
    skinKwPerC_ = 0.009 * surfaceScale;
    oilToAirKwPerC_ = thermal.oilCoolingPowerKwPerC > 0.0
        ? thermal.oilCoolingPowerKwPerC : 0.002 * surfaceScale;
    oilToCoolantKwPerC_ = 0.5 * displacementLitres + 0.1;
    airflowScale_ = config.coolingEfficiency;
    liquidCooled_ = thermal.liquidCooled;
    engineDrivenFan_ = thermal.engineDrivenFan;
}

void EngineThermalModel::advance(State& state, const Inputs& inputs, double dtSeconds) const noexcept {
    if (!(dtSeconds > 0.0)) return;
    if (liquidCooled_) {
        if (state.coolantTemperatureC >= fanOnC) state.fanOn = true;
        else if (state.coolantTemperatureC <= fanOffC) state.fanOn = false;
    } else {
        state.fanOn = false;
    }
    const auto airSpeed = std::max(0.0, inputs.coolingAirSpeedMps);
    double coreAirFraction = 0.0;
    double opening = 1.0;
    if (liquidCooled_) {
        const auto coreSpeed = radiatorRamFraction * airSpeed + (state.fanOn ? fanCoreSpeedMps : 0.0);
        coreAirFraction = std::max(stillRadiatorAirFraction, coreSpeed / (radiatorRamFraction * designAirSpeedMps));
        opening = std::clamp((state.coolantTemperatureC - thermostatStartC)
                / (thermostatFullC - thermostatStartC), thermostatBleed, 1.0);
    } else {
        const auto blower = engineDrivenFan_
            ? blowerAirSpeedAtRedlineMps * std::max(0.0, inputs.rpm) / redlineRpm_ : 0.0;
        coreAirFraction = std::max(stillFinAirFraction, (airSpeed + blower) / designAirSpeedMps);
    }
    const auto radiator = radiatorKwPerC_ * airflowScale_ * opening * std::pow(coreAirFraction, 0.8);
    const auto skinAir = 1.0 + 0.1 * airSpeed;
    const auto coolantExcess = state.coolantTemperatureC - inputs.ambientTemperatureC;
    const auto oilExcess = state.oilTemperatureC - inputs.ambientTemperatureC;
    const auto oilToCoolantKw = oilToCoolantKwPerC_ * (state.oilTemperatureC - state.coolantTemperatureC);
    const auto heat = std::max(0.0, inputs.heatReleaseKw);
    const auto coolantNetKw = heat * coolantHeatShare_ + oilToCoolantKw
        - (radiator + skinKwPerC_ * skinAir) * coolantExcess;
    const auto oilNetKw = heat * oilHeatShare_ + std::max(0.0, inputs.frictionHeatToOilKw)
        - oilToCoolantKw - oilToAirKwPerC_ * skinAir * oilExcess;
    state.coolantTemperatureC = std::clamp(state.coolantTemperatureC
        + coolantNetKw / coolantCapacityKjPerC_ * dtSeconds,
        inputs.ambientTemperatureC - 5.0, 150.0);
    state.oilTemperatureC = std::clamp(state.oilTemperatureC
        + oilNetKw / oilCapacityKjPerC_ * dtSeconds,
        inputs.ambientTemperatureC - 5.0, 170.0);
}

} // namespace enginelab
