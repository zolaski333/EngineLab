#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

namespace enginelab {

/** Coolant and oil temperatures of an engine left free to warm up and cool
 * down (`ThermalConfig::regulated` off): the realistic mode.
 *
 * Two lumped capacities. The coolant one stands for the coolant and the part
 * of the block and head that follows it; the oil exchanges heat with it
 * through the block. A liquid-cooled engine sheds heat through a radiator
 * that its thermostat opens (82 -> 94 degC), cooled by the air of its own
 * speed and by an electric fan (on at 100 degC, off at 95 degC); an air-cooled
 * one through its fins, by the air of its speed and, if it has one, by an
 * engine-driven blower. Both lose a little through their skin.
 *
 * Unauthored sizes (zero in the configuration) follow the displacement: the
 * capacities scale with it, and the radiator rejects the engine's heat at its
 * estimated rated power at 30 m/s and 65 K above the air. They are sizes for a
 * plausible engine of that size, not measured ones. */
class EngineThermalModel final {
public:
    struct Inputs final {
        /** Heat released by combustion, kW. */
        double heatReleaseKw { 0.0 };
        /** Friction power, kW: it all ends up as heat in the oil. */
        double frictionHeatToOilKw { 0.0 };
        /** Air speed at the radiator or fins from motion or a dyno cell fan. */
        double coolingAirSpeedMps { 0.0 };
        double rpm { 0.0 };
        double ambientTemperatureC { 22.0 };
    };

    struct State final {
        double coolantTemperatureC { 22.0 };
        double oilTemperatureC { 22.0 };
        bool fanOn { false };
    };

    explicit EngineThermalModel(const EngineConfig& config) noexcept;

    void advance(State& state, const Inputs& inputs, double dtSeconds) const noexcept;

    [[nodiscard]] double coolantCapacityKjPerC() const noexcept { return coolantCapacityKjPerC_; }
    [[nodiscard]] double oilCapacityKjPerC() const noexcept { return oilCapacityKjPerC_; }
    /** Radiator (or fins) conductance at the 30 m/s design air speed, kW/K. */
    [[nodiscard]] double radiatorKwPerC() const noexcept { return radiatorKwPerC_; }
    [[nodiscard]] double estimatedRatedPowerKw() const noexcept { return ratedPowerKw_; }

    static constexpr double thermostatStartC = 82.0;
    static constexpr double thermostatFullC = 94.0;
    static constexpr double fanOnC = 100.0;
    static constexpr double fanOffC = 95.0;

private:
    double coolantCapacityKjPerC_ { 0.0 };
    double oilCapacityKjPerC_ { 0.0 };
    double radiatorKwPerC_ { 0.0 };
    double skinKwPerC_ { 0.0 };
    double oilToAirKwPerC_ { 0.0 };
    double oilToCoolantKwPerC_ { 0.0 };
    double ratedPowerKw_ { 0.0 };
    double coolantHeatShare_ { 0.0 };
    double oilHeatShare_ { 0.0 };
    double airflowScale_ { 1.0 };
    double redlineRpm_ { 1.0 };
    bool liquidCooled_ { true };
    bool engineDrivenFan_ { false };
};

} // namespace enginelab
