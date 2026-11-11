#pragma once

#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/foundation/EngineTypes.hpp>

#include <string>
#include <vector>

/** The fuel's editable properties, shared by the Fuel window and the 3-D
    view's combustion chamber so both edit it the same way. */
namespace enginelab::fuel_editing {

struct Field final {
    const char* label;
    const char* unit;
    int decimals;
    /** A step of the field's buttons and a fine (modifier) step. */
    double step;
    double fineStep;
    double minimum;
    double maximum;
};

/** The fields of an engine's fuel, in order: octane or cetane, heating value,
    stoichiometric ratio, latent heat, and a petrol's laminar flame speed. */
[[nodiscard]] std::vector<Field> fields(bool diesel);
/** Kilograms of oxygen a kilogram of the fuel burns (stoichiometric). */
[[nodiscard]] double oxygenMassPerFuelMass(const FuelConfig&);
/** The heat the fuel releases per kilogram of the air's oxygen it burns,
    MJ/kg: about 13 for every real fuel (Thornton's rule: petrol 12.6,
    ethanol 12.8, methanol 13.3, diesel 12.7). Above it the engine makes more
    power from the same air than any fuel that takes its oxygen from the air
    can (nitromethane carries its own). */
[[nodiscard]] double energyPerOxygenMjPerKg(const FuelConfig&);
/** The heating value that keeps `fuel`'s energy per kilogram of oxygen at
    another stoichiometric ratio (within the heating value's field). */
[[nodiscard]] double followingHeatingValue(const FuelConfig& fuel, double stoichiometricAirFuelRatio);
/** The engine's values of `fields`, in the same order. */
[[nodiscard]] std::vector<double> values(const EngineConfig&);
/** Writes `fields`' values into the engine. A listed fuel whose values change
    is renamed "... (edited)". Another stoichiometric ratio moves the oxygen
    the fuel needs with it, and its heating value too when that value is
    left as it was (followingHeatingValue). Returns an error, or an empty
    string. */
[[nodiscard]] std::string apply(EngineConfig&, const std::vector<double>& values);
/** Puts a listed fuel in the tank. */
void applyPreset(EngineConfig&, const FuelPreset&);
/** Refits the fuel metering for the fuel now on `config`, as a tuner does
    for E85 or methanol: the same mixture takes `before`'s stoichiometric
    ratio over the new one times the fuel mass, so the injectors flow that
    much more mass and the carburettor jets pass it (an orifice's mass flow
    goes as its area times the square root of the fuel's density). A spark
    engine only; nothing changes when the ratio does not. */
void refitFuelMetering(EngineConfig& config, const FuelConfig& before);
/** The listed fuels an engine of this kind can burn. */
[[nodiscard]] std::vector<FuelPreset> presetsFor(const std::vector<FuelPreset>&, bool diesel);
/** What the fields change, in a sentence or three. */
[[nodiscard]] const char* note(bool diesel);

} // namespace enginelab::fuel_editing
