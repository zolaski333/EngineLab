#include <enginelab/serialization/YamlEngineSerializer.hpp>
#include <yaml-cpp/yaml.h>

namespace enginelab {
namespace {
[[nodiscard]] const char* layoutName(EngineLayout value) noexcept {
    switch (value) {
    case EngineLayout::inlineLayout: return "inline";
    case EngineLayout::vLayout: return "v";
    case EngineLayout::flat: return "flat";
    case EngineLayout::custom: return "custom";
    }
    return "custom";
}
}

std::string YamlEngineSerializer::encode(const EngineConfig& config) const {
    YAML::Emitter out;
    out << YAML::BeginMap << YAML::Key << "schema_version" << YAML::Value << config.schemaVersion
        << YAML::Key << "engine" << YAML::Value << YAML::BeginMap
        << YAML::Key << "name" << YAML::Value << config.name
        << YAML::Key << "cycle" << YAML::Value << (config.cycle == EngineCycle::fourStroke ? "four_stroke" : "two_stroke")
        << YAML::Key << "fuel" << YAML::Value << (config.fuel == FuelType::gasoline ? "gasoline" : "diesel")
        << YAML::Key << "layout" << YAML::Value << layoutName(config.layout)
        << YAML::Key << "idle_rpm" << YAML::Value << config.idleRpm
        << YAML::Key << "redline_rpm" << YAML::Value << config.redlineRpm
        << YAML::Key << "rotating_inertia_kg_m2" << YAML::Value << config.rotatingInertiaKgM2
        << YAML::Key << "friction_coefficient" << YAML::Value << config.frictionCoefficient
        << YAML::Key << "octane_rating" << YAML::Value << config.octaneRating
        << YAML::Key << "ambient_pressure_kpa" << YAML::Value << config.ambientPressureKpa
        << YAML::Key << "ambient_temperature_c" << YAML::Value << config.ambientTemperatureC
        << YAML::Key << "cooling_efficiency" << YAML::Value << config.coolingEfficiency
        << YAML::Key << "plenum_volume_l" << YAML::Value << config.plenumVolumeLitres
        << YAML::Key << "throttle_diameter_mm" << YAML::Value << config.throttleDiameterMm
        << YAML::Key << "bank_angle_deg" << YAML::Value << config.bankAngleDegrees
        << YAML::Key << "camshafts" << YAML::Value << YAML::BeginMap
        << YAML::Key << "intake_duration_deg" << YAML::Value << config.camshafts.intakeDurationDegrees
        << YAML::Key << "exhaust_duration_deg" << YAML::Value << config.camshafts.exhaustDurationDegrees
        << YAML::Key << "intake_lift_mm" << YAML::Value << config.camshafts.intakeLiftMm
        << YAML::Key << "exhaust_lift_mm" << YAML::Value << config.camshafts.exhaustLiftMm
        << YAML::Key << "intake_centerline_deg" << YAML::Value << config.camshafts.intakeCenterlineDegrees
        << YAML::Key << "exhaust_centerline_deg" << YAML::Value << config.camshafts.exhaustCenterlineDegrees << YAML::EndMap
        << YAML::Key << "exhaust" << YAML::Value << YAML::BeginMap
        << YAML::Key << "primary_length_mm" << YAML::Value << config.exhaust.primaryLengthMm
        << YAML::Key << "primary_diameter_mm" << YAML::Value << config.exhaust.primaryDiameterMm
        << YAML::Key << "collector_diameter_mm" << YAML::Value << config.exhaust.collectorDiameterMm
        << YAML::Key << "muffler_restriction" << YAML::Value << config.exhaust.mufflerRestriction
        << YAML::Key << "outlet_diameter_mm" << YAML::Value << config.exhaust.outletDiameterMm << YAML::EndMap
        << YAML::Key << "firing_order" << YAML::Value << YAML::Flow << config.firingOrder
        << YAML::Key << "cylinders" << YAML::Value << YAML::BeginSeq;
    for (const auto& cylinder : config.cylinders) out << YAML::BeginMap
        << YAML::Key << "id" << YAML::Value << cylinder.id << YAML::Key << "bore_mm" << YAML::Value << cylinder.boreMm
        << YAML::Key << "stroke_mm" << YAML::Value << cylinder.strokeMm << YAML::Key << "connecting_rod_mm" << YAML::Value << cylinder.connectingRodMm
        << YAML::Key << "piston_mass_g" << YAML::Value << cylinder.pistonMassGrams << YAML::Key << "compression_ratio" << YAML::Value << cylinder.compressionRatio
        << YAML::Key << "ignition_offset_deg" << YAML::Value << cylinder.ignitionOffsetDegrees << YAML::Key << "efficiency_offset" << YAML::Value << cylinder.efficiencyOffset
        << YAML::Key << "crank_offset_deg" << YAML::Value << cylinder.crankOffsetDegrees
        << YAML::EndMap;
    out << YAML::EndSeq << YAML::EndMap << YAML::EndMap;
    return out.c_str();
}

EngineDecodeResult YamlEngineSerializer::decode(std::string_view text) const noexcept {
    try {
        const auto document = YAML::Load(std::string(text));
        EngineConfig config;
        config.schemaVersion = document["schema_version"].as<std::uint32_t>();
        if (config.schemaVersion != 1) return { std::nullopt, "Unsupported schema version" };
        const auto engine = document["engine"];
        config.name = engine["name"].as<std::string>();
        const auto cycle = engine["cycle"].as<std::string>();
        if (cycle == "four_stroke") config.cycle = EngineCycle::fourStroke;
        else if (cycle == "two_stroke") config.cycle = EngineCycle::twoStroke;
        else return { std::nullopt, "Unknown engine cycle: " + cycle };
        const auto fuel = engine["fuel"].as<std::string>();
        if (fuel == "gasoline") config.fuel = FuelType::gasoline;
        else if (fuel == "diesel") config.fuel = FuelType::diesel;
        else return { std::nullopt, "Unknown fuel type: " + fuel };
        const auto layout = engine["layout"] ? engine["layout"].as<std::string>() : "inline";
        if (layout == "inline") config.layout = EngineLayout::inlineLayout;
        else if (layout == "v") config.layout = EngineLayout::vLayout;
        else if (layout == "flat") config.layout = EngineLayout::flat;
        else if (layout == "custom") config.layout = EngineLayout::custom;
        else return { std::nullopt, "Unknown engine layout: " + layout };
        config.idleRpm = engine["idle_rpm"].as<double>(); config.redlineRpm = engine["redline_rpm"].as<double>();
        config.rotatingInertiaKgM2 = engine["rotating_inertia_kg_m2"].as<double>();
        config.frictionCoefficient = engine["friction_coefficient"].as<double>(); config.octaneRating = engine["octane_rating"].as<double>();
        if (engine["ambient_pressure_kpa"]) config.ambientPressureKpa = engine["ambient_pressure_kpa"].as<double>();
        if (engine["ambient_temperature_c"]) config.ambientTemperatureC = engine["ambient_temperature_c"].as<double>();
        if (engine["cooling_efficiency"]) config.coolingEfficiency = engine["cooling_efficiency"].as<double>();
        if (engine["plenum_volume_l"]) config.plenumVolumeLitres = engine["plenum_volume_l"].as<double>();
        if (engine["throttle_diameter_mm"]) config.throttleDiameterMm = engine["throttle_diameter_mm"].as<double>();
        if (engine["bank_angle_deg"]) config.bankAngleDegrees = engine["bank_angle_deg"].as<double>();
        else config.bankAngleDegrees = config.layout == EngineLayout::vLayout ? 60.0 : 0.0;
        if (const auto cams = engine["camshafts"]) config.camshafts = {
            cams["intake_duration_deg"].as<double>(248.0), cams["exhaust_duration_deg"].as<double>(244.0),
            cams["intake_lift_mm"].as<double>(10.2), cams["exhaust_lift_mm"].as<double>(9.8),
            cams["intake_centerline_deg"].as<double>(110.0), cams["exhaust_centerline_deg"].as<double>(112.0) };
        if (const auto exhaust = engine["exhaust"]) config.exhaust = {
            exhaust["primary_length_mm"].as<double>(480.0), exhaust["primary_diameter_mm"].as<double>(42.0),
            exhaust["collector_diameter_mm"].as<double>(58.0), exhaust["muffler_restriction"].as<double>(0.28),
            exhaust["outlet_diameter_mm"].as<double>(65.0) };
        config.firingOrder = engine["firing_order"].as<std::vector<std::uint32_t>>();
        for (const auto& item : engine["cylinders"]) {
            const std::uint32_t cylinderId = item["id"].as<std::uint32_t>();
            double crankOffset = 0.0;
            if (item["crank_offset_deg"]) {
                crankOffset = item["crank_offset_deg"].as<double>();
            } else {
                const auto it = std::find(config.firingOrder.begin(), config.firingOrder.end(), cylinderId);
                if (it != config.firingOrder.end()) {
                    const auto orderIndex = static_cast<double>(std::distance(config.firingOrder.begin(), it));
                    crankOffset = orderIndex * (720.0 / static_cast<double>(config.firingOrder.size()));
                }
            }
            config.cylinders.push_back({
                cylinderId, item["bore_mm"].as<double>(), item["stroke_mm"].as<double>(),
                item["connecting_rod_mm"].as<double>(), item["piston_mass_g"].as<double>(), item["compression_ratio"].as<double>(),
                item["ignition_offset_deg"].as<double>(), item["efficiency_offset"].as<double>(), crankOffset });
        }
        if (const auto error = validateEngineConfig(config)) return { std::nullopt, *error };
        return { std::move(config), {} };
    } catch (const std::exception& error) { return { std::nullopt, error.what() }; }
}
} // namespace enginelab
