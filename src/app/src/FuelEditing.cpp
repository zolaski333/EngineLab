#include <enginelab/app/FuelEditing.hpp>

#include <string_view>

namespace enginelab::fuel_editing {

std::vector<Field> fields(bool diesel) {
    std::vector<Field> result;
    if (diesel)
        result.push_back({ "Cetane number", "", 0, 1.0, 0.5, 30.0, 80.0 });
    else
        result.push_back({ "Octane (RON)", "", 1, 1.0, 0.5, 70.0, 130.0 });
    result.push_back({ "Heating value", " MJ/kg", 1, 0.5, 0.1, 10.0, 60.0 });
    result.push_back({ "Stoichiometric AFR", "", 2, 0.1, 0.01, 5.0, 25.0 });
    result.push_back({ "Latent heat", " kJ/kg", 0, 10.0, 1.0, 10.0, 1'200.0 });
    if (!diesel) result.push_back({ "Laminar flame speed", " m/s", 3, 0.01, 0.002, 0.05, 2.0 });
    return result;
}

std::vector<double> values(const EngineConfig& config) {
    const auto diesel = config.fuel == FuelType::diesel;
    const auto& fuel = config.fuelProperties;
    std::vector<double> result { diesel ? fuel.cetaneNumber : config.octaneRating, fuel.lowerHeatingValueMjPerKg,
        fuel.stoichiometricAirFuelRatio, config.injection.latentHeatKjPerKg };
    if (!diesel) result.push_back(fuel.laminarFlameSpeedMps);
    return result;
}

std::string apply(EngineConfig& edited, const std::vector<double>& values) {
    const auto diesel = edited.fuel == FuelType::diesel;
    const auto count = diesel ? 4U : 5U;
    if (values.size() < count) return "The fuel changed meanwhile.";
    auto& fuel = edited.fuelProperties;
    const auto before = fuel;
    const auto octaneBefore = edited.octaneRating;
    const auto latentBefore = edited.injection.latentHeatKjPerKg;
    if (diesel) fuel.cetaneNumber = values[0];
    else edited.octaneRating = values[0];
    fuel.lowerHeatingValueMjPerKg = values[1];
    // The oxygen a mole of fuel needs is what sets its stoichiometric
    // ratio: they move together, or the engine refuses the fuel.
    if (values[2] != fuel.stoichiometricAirFuelRatio && fuel.stoichiometricAirFuelRatio > 0.0)
        fuel.oxygenMolesPerFuelMole *= values[2] / fuel.stoichiometricAirFuelRatio;
    fuel.stoichiometricAirFuelRatio = values[2];
    edited.injection.latentHeatKjPerKg = values[3];
    if (!diesel) fuel.laminarFlameSpeedMps = values[4];
    const auto changed = fuel.cetaneNumber != before.cetaneNumber
        || fuel.lowerHeatingValueMjPerKg != before.lowerHeatingValueMjPerKg
        || fuel.stoichiometricAirFuelRatio != before.stoichiometricAirFuelRatio
        || fuel.laminarFlameSpeedMps != before.laminarFlameSpeedMps
        || edited.octaneRating != octaneBefore || edited.injection.latentHeatKjPerKg != latentBefore;
    // A listed fuel edited is no longer that fuel.
    constexpr std::string_view editedSuffix { " (edited)" };
    if (changed && !fuel.name.ends_with(editedSuffix) && fuel.name.size() + editedSuffix.size() <= 128U)
        fuel.name += editedSuffix;
    return {};
}

void applyPreset(EngineConfig& edited, const FuelPreset& preset) {
    const auto diesel = edited.fuel == FuelType::diesel;
    edited.fuelProperties = preset.properties;
    if (!diesel && preset.octaneRating > 0.0) edited.octaneRating = preset.octaneRating;
    if (preset.latentHeatKjPerKg > 0.0) edited.injection.latentHeatKjPerKg = preset.latentHeatKjPerKg;
}

std::vector<FuelPreset> presetsFor(const std::vector<FuelPreset>& presets, bool diesel) {
    std::vector<FuelPreset> offered;
    for (const auto& preset : presets)
        if (preset.diesel() == diesel) offered.push_back(preset);
    return offered;
}

const char* note(bool diesel) {
    return diesel
        ? "A higher cetane number lights sooner after injection, a softer premixed burn. The ECU's smoke limit "
          "follows the stoichiometric ratio. The engine restarts to take them."
        : "The knock model reads the octane: lower knocks sooner, and the knock control pulls the spark back. "
          "The ECU's mixture map is in lambda, so another stoichiometric ratio (E85, methanol) is metered "
          "right; the oxygen the fuel needs follows it. The engine restarts to take them.";
}

} // namespace enginelab::fuel_editing
