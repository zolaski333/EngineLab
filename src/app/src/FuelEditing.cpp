#include <enginelab/app/FuelEditing.hpp>

#include <algorithm>
#include <cmath>
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

void refitFuelMetering(EngineConfig& config, const FuelConfig& before) {
    if (config.fuel == FuelType::diesel) return;
    const auto& after = config.fuelProperties;
    if (!(before.stoichiometricAirFuelRatio > 0.0) || !(after.stoichiometricAirFuelRatio > 0.0)) return;
    const auto massRatio = before.stoichiometricAirFuelRatio / after.stoichiometricAirFuelRatio;
    if (!std::isfinite(massRatio) || massRatio == 1.0) return;
    // The validation's ranges bound the refit.
    config.injection.injectorFlowMgPerSecond = std::clamp(
        config.injection.injectorFlowMgPerSecond * massRatio, 10.0, 500'000.0);
    const auto densityRatio = before.densityKgPerL > 0.0 && after.densityKgPerL > 0.0
        ? before.densityKgPerL / after.densityKgPerL : 1.0;
    const auto jetScale = massRatio * std::sqrt(densityRatio);
    const auto rejet = [&](CarburettorConfig& carburettor) {
        if (!carburettor.enabled) return;
        carburettor.mainJetFlowNumber = std::clamp(carburettor.mainJetFlowNumber * jetScale, 5.0, 5'000.0);
        carburettor.pilotJetFlowNumber = std::clamp(carburettor.pilotJetFlowNumber * jetScale, 1.0, 2'000.0);
        carburettor.starterJetFlowNumber = std::clamp(carburettor.starterJetFlowNumber * jetScale, 0.0, 20'000.0);
        // The pump displaces a volume: the same mass shot takes this much.
        carburettor.acceleratorPumpStrokeCc = std::clamp(
            carburettor.acceleratorPumpStrokeCc * massRatio * densityRatio, 0.0, 100.0);
    };
    rejet(config.intake.carburettor);
    for (auto& path : config.intakePaths) rejet(path.geometry.carburettor);
}

std::string apply(EngineConfig& edited, const std::vector<double>& values) {
    const auto diesel = edited.fuel == FuelType::diesel;
    const auto count = diesel ? 4U : 5U;
    if (values.size() < count) return "The fuel changed meanwhile.";
    auto& fuel = edited.fuelProperties;
    const auto before = fuel;
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
    refitFuelMetering(edited, before);
    if (!diesel) fuel.laminarFlameSpeedMps = values[4];
    // The octane is the engine's (an engine file's octane_rating), not the
    // listed fuel's: a new octane alone keeps the fuel's name, and the
    // running engine takes it without a restart.
    const auto changed = fuel.cetaneNumber != before.cetaneNumber
        || fuel.lowerHeatingValueMjPerKg != before.lowerHeatingValueMjPerKg
        || fuel.stoichiometricAirFuelRatio != before.stoichiometricAirFuelRatio
        || fuel.laminarFlameSpeedMps != before.laminarFlameSpeedMps
        || edited.injection.latentHeatKjPerKg != latentBefore;
    // A listed fuel edited is no longer that fuel.
    constexpr std::string_view editedSuffix { " (edited)" };
    if (changed && !fuel.name.ends_with(editedSuffix) && fuel.name.size() + editedSuffix.size() <= 128U)
        fuel.name += editedSuffix;
    return {};
}

void applyPreset(EngineConfig& edited, const FuelPreset& preset) {
    const auto diesel = edited.fuel == FuelType::diesel;
    const auto before = edited.fuelProperties;
    edited.fuelProperties = preset.properties;
    refitFuelMetering(edited, before);
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
          "A new octane is taken at once and changes no spark map: the advance is yours to tune (ECU). "
          "The ECU's mixture map is in lambda, so another stoichiometric ratio (E85, methanol) is metered "
          "right; the oxygen the fuel needs follows it, and the injectors and carburettor jets are resized "
          "to deliver it (methanol takes 2.3 times the fuel mass). The other values restart the engine.";
}

} // namespace enginelab::fuel_editing
