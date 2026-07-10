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
    case EngineLayout::radial: return "radial";
    case EngineLayout::custom: return "custom";
    }
    return "custom";
}

[[nodiscard]] Json liftProfileJson(const std::vector<ValveLiftSample>& profile) {
    Json output = Json::array();
    for (const auto& sample : profile)
        output.push_back({ { "angle_deg", sample.angleDegrees }, { "lift_mm", sample.liftMm } });
    return output;
}

[[nodiscard]] std::vector<ValveLiftSample> decodeLiftProfile(const Json& values) {
    std::vector<ValveLiftSample> profile;
    profile.reserve(values.size());
    for (const auto& sample : values)
        profile.push_back({ sample.at("angle_deg").get<double>(), sample.at("lift_mm").get<double>() });
    return profile;
}
}

std::string JsonEngineSerializer::encode(const EngineConfig& config) const {
    Json cylinders = Json::array();
    for (const auto& cylinder : config.cylinders) cylinders.push_back({
        {"id", cylinder.id}, {"bore_mm", cylinder.boreMm}, {"stroke_mm", cylinder.strokeMm},
        {"connecting_rod_mm", cylinder.connectingRodMm}, {"piston_mass_g", cylinder.pistonMassGrams},
        {"compression_ratio", cylinder.compressionRatio}, {"ignition_offset_deg", cylinder.ignitionOffsetDegrees},
        {"efficiency_offset", cylinder.efficiencyOffset}, {"crank_offset_deg", cylinder.crankOffsetDegrees},
        {"crank_journal_id", cylinder.crankJournalId}, {"bank_offset_deg", cylinder.bankOffsetDegrees} });
    Json crankJournals = Json::array();
    for (const auto& journal : config.crankJournals) crankJournals.push_back({
        {"id", journal.id}, {"angle_deg", journal.angleDegrees}, {"throw_mm", journal.throwMm} });
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
        {"forced_induction", {{"enabled", config.forcedInduction.enabled},
                               {"pressure_ratio", config.forcedInduction.pressureRatio},
                               {"full_boost_rpm", config.forcedInduction.fullBoostRpm},
                               {"compressor_efficiency", config.forcedInduction.compressorEfficiency},
                               {"charge_temperature_rise_c", config.forcedInduction.chargeTemperatureRiseC}}},
        {"thermal", {{"coolant_mass_kj_per_c", config.thermal.coolantMassKjPerC},
                      {"oil_mass_kj_per_c", config.thermal.oilMassKjPerC},
                      {"coolant_heat_share", config.thermal.coolantHeatShare},
                      {"oil_heat_share", config.thermal.oilHeatShare},
                      {"cooling_power_kw_per_c", config.thermal.coolingPowerKwPerC},
                      {"oil_cooling_power_kw_per_c", config.thermal.oilCoolingPowerKwPerC}}},
        {"camshafts", {{"intake_duration_deg", config.camshafts.intakeDurationDegrees},
                        {"exhaust_duration_deg", config.camshafts.exhaustDurationDegrees},
                        {"intake_lift_mm", config.camshafts.intakeLiftMm}, {"exhaust_lift_mm", config.camshafts.exhaustLiftMm},
                        {"intake_centerline_deg", config.camshafts.intakeCenterlineDegrees},
                        {"exhaust_centerline_deg", config.camshafts.exhaustCenterlineDegrees},
                        {"intake_flow_coefficient", config.camshafts.intakeFlowCoefficient},
                        {"exhaust_flow_coefficient", config.camshafts.exhaustFlowCoefficient},
                        {"intake_lift_profile", liftProfileJson(config.camshafts.intakeLiftProfile)},
                        {"exhaust_lift_profile", liftProfileJson(config.camshafts.exhaustLiftProfile)}}},
        {"crank_journals", crankJournals},
        {"exhaust", {{"primary_length_mm", config.exhaust.primaryLengthMm},
                      {"primary_diameter_mm", config.exhaust.primaryDiameterMm},
                      {"collector_diameter_mm", config.exhaust.collectorDiameterMm},
                      {"muffler_restriction", config.exhaust.mufflerRestriction},
                      {"outlet_diameter_mm", config.exhaust.outletDiameterMm}}},
        {"transmission", {{"gear_ratios", config.transmission.gearRatios},
                           {"final_drive_ratio", config.transmission.finalDriveRatio},
                           {"max_clutch_torque_nm", config.transmission.maxClutchTorqueNm}}},
        {"vehicle", {{"mass_kg", config.vehicle.massKg},
                      {"drag_coefficient", config.vehicle.dragCoefficient},
                      {"frontal_area_m2", config.vehicle.frontalAreaM2},
                      {"tire_radius_m", config.vehicle.tireRadiusM},
                      {"rolling_resistance_coefficient", config.vehicle.rollingResistanceCoefficient}}} }} };
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
        else if (layout == "radial") config.layout = EngineLayout::radial;
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
        config.bankAngleDegrees = engine.value("bank_angle_deg", config.layout == EngineLayout::vLayout ? 60.0
            : (config.layout == EngineLayout::flat ? 180.0 : (config.layout == EngineLayout::radial ? 72.0 : 0.0)));
        if (engine.contains("forced_induction")) {
            const auto& forced = engine.at("forced_induction");
            config.forcedInduction.enabled = forced.value("enabled", false);
            config.forcedInduction.pressureRatio = forced.value("pressure_ratio", 1.0);
            config.forcedInduction.fullBoostRpm = forced.value("full_boost_rpm", 3'500.0);
            config.forcedInduction.compressorEfficiency = forced.value("compressor_efficiency", 0.68);
            config.forcedInduction.chargeTemperatureRiseC = forced.value("charge_temperature_rise_c", 35.0);
        }
        if (engine.contains("thermal")) {
            const auto& thermal = engine.at("thermal");
            config.thermal.coolantMassKjPerC = thermal.value("coolant_mass_kj_per_c", config.thermal.coolantMassKjPerC);
            config.thermal.oilMassKjPerC = thermal.value("oil_mass_kj_per_c", config.thermal.oilMassKjPerC);
            config.thermal.coolantHeatShare = thermal.value("coolant_heat_share", config.thermal.coolantHeatShare);
            config.thermal.oilHeatShare = thermal.value("oil_heat_share", config.thermal.oilHeatShare);
            config.thermal.coolingPowerKwPerC = thermal.value("cooling_power_kw_per_c", config.thermal.coolingPowerKwPerC);
            config.thermal.oilCoolingPowerKwPerC = thermal.value("oil_cooling_power_kw_per_c", config.thermal.oilCoolingPowerKwPerC);
        }
        if (engine.contains("camshafts")) {
            const auto& cams = engine.at("camshafts");
            config.camshafts = { cams.value("intake_duration_deg", 248.0), cams.value("exhaust_duration_deg", 244.0),
                cams.value("intake_lift_mm", 10.2), cams.value("exhaust_lift_mm", 9.8),
                cams.value("intake_centerline_deg", 110.0), cams.value("exhaust_centerline_deg", 112.0) };
            config.camshafts.intakeFlowCoefficient = cams.value("intake_flow_coefficient", 0.62);
            config.camshafts.exhaustFlowCoefficient = cams.value("exhaust_flow_coefficient", 0.62);
            if (cams.contains("intake_lift_profile")) config.camshafts.intakeLiftProfile = decodeLiftProfile(cams.at("intake_lift_profile"));
            if (cams.contains("exhaust_lift_profile")) config.camshafts.exhaustLiftProfile = decodeLiftProfile(cams.at("exhaust_lift_profile"));
        }
        if (engine.contains("crank_journals")) {
            for (const auto& journal : engine.at("crank_journals"))
                config.crankJournals.push_back({ journal.at("id").get<std::uint32_t>(),
                    journal.at("angle_deg").get<double>(), journal.at("throw_mm").get<double>() });
        }
        if (engine.contains("exhaust")) {
            const auto& exhaust = engine.at("exhaust");
            config.exhaust = { exhaust.value("primary_length_mm", 480.0), exhaust.value("primary_diameter_mm", 42.0),
                exhaust.value("collector_diameter_mm", 58.0), exhaust.value("muffler_restriction", 0.28),
                exhaust.value("outlet_diameter_mm", 65.0) };
        }
        if (engine.contains("transmission")) {
            const auto& transmission = engine.at("transmission");
            config.transmission.gearRatios = transmission.value("gear_ratios", config.transmission.gearRatios);
            config.transmission.finalDriveRatio = transmission.value("final_drive_ratio", config.transmission.finalDriveRatio);
            config.transmission.maxClutchTorqueNm = transmission.value("max_clutch_torque_nm", config.transmission.maxClutchTorqueNm);
        }
        if (engine.contains("vehicle")) {
            const auto& vehicle = engine.at("vehicle");
            config.vehicle.massKg = vehicle.value("mass_kg", config.vehicle.massKg);
            config.vehicle.dragCoefficient = vehicle.value("drag_coefficient", config.vehicle.dragCoefficient);
            config.vehicle.frontalAreaM2 = vehicle.value("frontal_area_m2", config.vehicle.frontalAreaM2);
            config.vehicle.tireRadiusM = vehicle.value("tire_radius_m", config.vehicle.tireRadiusM);
            config.vehicle.rollingResistanceCoefficient = vehicle.value("rolling_resistance_coefficient",
                                                                        config.vehicle.rollingResistanceCoefficient);
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
            CylinderConfig cylinder {
                cylinderId, item.at("bore_mm"), item.at("stroke_mm"), item.at("connecting_rod_mm"),
                item.at("piston_mass_g"), item.at("compression_ratio"), item.at("ignition_offset_deg"),
                item.at("efficiency_offset"), crankOffset };
            cylinder.crankJournalId = item.value("crank_journal_id", std::uint32_t { 0 });
            cylinder.bankOffsetDegrees = item.value("bank_offset_deg", 0.0);
            config.cylinders.push_back(cylinder);
        }
        if (const auto error = validateEngineConfig(config)) return { std::nullopt, *error };
        return { std::move(config), {} };
    } catch (const std::exception& error) { return { std::nullopt, error.what() }; }
}
} // namespace enginelab
