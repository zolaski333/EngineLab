#include <enginelab/serialization/JsonEngineSerializer.hpp>
#include <nlohmann/json.hpp>

namespace enginelab {
namespace {
using Json = nlohmann::json;
[[nodiscard]] std::string cycleName(EngineCycle value) { return value == EngineCycle::fourStroke ? "four_stroke" : "two_stroke"; }
[[nodiscard]] std::string fuelName(FuelType value) { return value == FuelType::gasoline ? "gasoline" : "diesel"; }
[[nodiscard]] std::string layoutName(EngineLayout value) {
    switch (value) {
    case EngineLayout::inlineLayout: return "inline";
    case EngineLayout::vLayout: return "v";
    case EngineLayout::flat: return "flat";
    case EngineLayout::custom: return "custom";
    }
    return "custom";
}
}

std::string JsonEngineSerializer::encode(const EngineConfig& config) const {
    Json cylinders = Json::array();
    for (const auto& cylinder : config.cylinders) cylinders.push_back({
        {"id", cylinder.id}, {"bore_mm", cylinder.boreMm}, {"stroke_mm", cylinder.strokeMm},
        {"connecting_rod_mm", cylinder.connectingRodMm}, {"piston_mass_g", cylinder.pistonMassGrams},
        {"compression_ratio", cylinder.compressionRatio}, {"ignition_offset_deg", cylinder.ignitionOffsetDegrees},
        {"efficiency_offset", cylinder.efficiencyOffset}, {"crank_offset_deg", cylinder.crankOffsetDegrees} });
    Json document = { {"schema_version", config.schemaVersion}, {"engine", {
        {"name", config.name}, {"cycle", cycleName(config.cycle)}, {"fuel", fuelName(config.fuel)},
        {"layout", layoutName(config.layout)},
        {"cylinders", cylinders}, {"firing_order", config.firingOrder}, {"idle_rpm", config.idleRpm},
        {"redline_rpm", config.redlineRpm}, {"rotating_inertia_kg_m2", config.rotatingInertiaKgM2},
        {"friction_coefficient", config.frictionCoefficient}, {"octane_rating", config.octaneRating},
        {"ambient_pressure_kpa", config.ambientPressureKpa}, {"ambient_temperature_c", config.ambientTemperatureC},
        {"cooling_efficiency", config.coolingEfficiency},
        {"plenum_volume_l", config.plenumVolumeLitres},
        {"throttle_diameter_mm", config.throttleDiameterMm},
        {"bank_angle_deg", config.bankAngleDegrees},
        {"camshafts", {{"intake_duration_deg", config.camshafts.intakeDurationDegrees},
                        {"exhaust_duration_deg", config.camshafts.exhaustDurationDegrees},
                        {"intake_lift_mm", config.camshafts.intakeLiftMm}, {"exhaust_lift_mm", config.camshafts.exhaustLiftMm},
                        {"intake_centerline_deg", config.camshafts.intakeCenterlineDegrees},
                        {"exhaust_centerline_deg", config.camshafts.exhaustCenterlineDegrees}}},
        {"exhaust", {{"primary_length_mm", config.exhaust.primaryLengthMm},
                      {"primary_diameter_mm", config.exhaust.primaryDiameterMm},
                      {"collector_diameter_mm", config.exhaust.collectorDiameterMm},
                      {"muffler_restriction", config.exhaust.mufflerRestriction},
                      {"outlet_diameter_mm", config.exhaust.outletDiameterMm}}} }} };
    return document.dump(2);
}

EngineDecodeResult JsonEngineSerializer::decode(std::string_view text) const noexcept {
    try {
        const auto document = Json::parse(text);
        const auto& engine = document.at("engine");
        EngineConfig config;
        config.schemaVersion = document.at("schema_version").get<std::uint32_t>();
        if (config.schemaVersion != 1) return { std::nullopt, "Unsupported schema version" };
        config.name = engine.at("name").get<std::string>();
        const auto cycle = engine.at("cycle").get<std::string>();
        if (cycle == "four_stroke") config.cycle = EngineCycle::fourStroke;
        else if (cycle == "two_stroke") config.cycle = EngineCycle::twoStroke;
        else return { std::nullopt, "Unknown engine cycle: " + cycle };
        const auto fuel = engine.at("fuel").get<std::string>();
        if (fuel == "gasoline") config.fuel = FuelType::gasoline;
        else if (fuel == "diesel") config.fuel = FuelType::diesel;
        else return { std::nullopt, "Unknown fuel type: " + fuel };
        const auto layout = engine.value("layout", "inline");
        if (layout == "inline") config.layout = EngineLayout::inlineLayout;
        else if (layout == "v") config.layout = EngineLayout::vLayout;
        else if (layout == "flat") config.layout = EngineLayout::flat;
        else if (layout == "custom") config.layout = EngineLayout::custom;
        else return { std::nullopt, "Unknown engine layout: " + layout };
        config.firingOrder = engine.at("firing_order").get<std::vector<std::uint32_t>>();
        config.idleRpm = engine.at("idle_rpm"); config.redlineRpm = engine.at("redline_rpm");
        config.rotatingInertiaKgM2 = engine.at("rotating_inertia_kg_m2");
        config.frictionCoefficient = engine.at("friction_coefficient"); config.octaneRating = engine.at("octane_rating");
        config.ambientPressureKpa = engine.value("ambient_pressure_kpa", 101.325);
        config.ambientTemperatureC = engine.value("ambient_temperature_c", 22.0);
        config.coolingEfficiency = engine.value("cooling_efficiency", 1.0);
        config.plenumVolumeLitres = engine.value("plenum_volume_l", 3.0);
        config.throttleDiameterMm = engine.value("throttle_diameter_mm", 60.0);
        config.bankAngleDegrees = engine.value("bank_angle_deg", config.layout == EngineLayout::vLayout ? 60.0 : 0.0);
        if (engine.contains("camshafts")) {
            const auto& cams = engine.at("camshafts");
            config.camshafts = { cams.value("intake_duration_deg", 248.0), cams.value("exhaust_duration_deg", 244.0),
                cams.value("intake_lift_mm", 10.2), cams.value("exhaust_lift_mm", 9.8),
                cams.value("intake_centerline_deg", 110.0), cams.value("exhaust_centerline_deg", 112.0) };
        }
        if (engine.contains("exhaust")) {
            const auto& exhaust = engine.at("exhaust");
            config.exhaust = { exhaust.value("primary_length_mm", 480.0), exhaust.value("primary_diameter_mm", 42.0),
                exhaust.value("collector_diameter_mm", 58.0), exhaust.value("muffler_restriction", 0.28),
                exhaust.value("outlet_diameter_mm", 65.0) };
        }
        for (const auto& item : engine.at("cylinders")) {
            const std::uint32_t cylinderId = item.at("id");
            double crankOffset = 0.0;
            if (item.contains("crank_offset_deg")) {
                crankOffset = item.at("crank_offset_deg").get<double>();
            } else {
                const auto it = std::find(config.firingOrder.begin(), config.firingOrder.end(), cylinderId);
                if (it != config.firingOrder.end()) {
                    const auto orderIndex = static_cast<double>(std::distance(config.firingOrder.begin(), it));
                    crankOffset = orderIndex * (720.0 / static_cast<double>(config.firingOrder.size()));
                }
            }
            config.cylinders.push_back({
                cylinderId, item.at("bore_mm"), item.at("stroke_mm"), item.at("connecting_rod_mm"),
                item.at("piston_mass_g"), item.at("compression_ratio"), item.at("ignition_offset_deg"),
                item.at("efficiency_offset"), crankOffset });
        }
        if (const auto error = validateEngineConfig(config)) return { std::nullopt, *error };
        return { std::move(config), {} };
    } catch (const std::exception& error) { return { std::nullopt, error.what() }; }
}
} // namespace enginelab
