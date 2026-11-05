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
/** The engine's values of `fields`, in the same order. */
[[nodiscard]] std::vector<double> values(const EngineConfig&);
/** Writes `fields`' values into the engine. A listed fuel whose values change
    is renamed "... (edited)". Returns an error, or an empty string. */
[[nodiscard]] std::string apply(EngineConfig&, const std::vector<double>& values);
/** Puts a listed fuel in the tank. */
void applyPreset(EngineConfig&, const FuelPreset&);
/** The listed fuels an engine of this kind can burn. */
[[nodiscard]] std::vector<FuelPreset> presetsFor(const std::vector<FuelPreset>&, bool diesel);
/** What the fields change, in a sentence or three. */
[[nodiscard]] const char* note(bool diesel);

} // namespace enginelab::fuel_editing
