#include <enginelab/serialization/YamlEngineSerializer.hpp>
#include <yaml-cpp/yaml.h>

namespace enginelab {
namespace {
[[nodiscard]] const char* layoutName(EngineLayout value) noexcept {
    switch (value) {
    case EngineLayout::inlineLayout: return "inline";
    case EngineLayout::vLayout: return "v";
    case EngineLayout::flat: return "flat";
    case EngineLayout::radial: return "radial";
    case EngineLayout::custom: return "custom";
    }
    return "custom";
}

void emitLiftProfile(YAML::Emitter& out, const char* key, const std::vector<ValveLiftSample>& profile) {
    out << YAML::Key << key << YAML::Value << YAML::BeginSeq;
    for (const auto& sample : profile)
        out << YAML::BeginMap << YAML::Key << "angle_deg" << YAML::Value << sample.angleDegrees
            << YAML::Key << "lift_mm" << YAML::Value << sample.liftMm << YAML::EndMap;
    out << YAML::EndSeq;
}

[[nodiscard]] std::vector<ValveLiftSample> decodeLiftProfile(const YAML::Node& values) {
    std::vector<ValveLiftSample> profile;
    profile.reserve(values.size());
    for (const auto& sample : values)
        profile.push_back({ sample["angle_deg"].as<double>(), sample["lift_mm"].as<double>() });
    return profile;
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
        << YAML::Key << "forced_induction" << YAML::Value << YAML::BeginMap
        << YAML::Key << "enabled" << YAML::Value << config.forcedInduction.enabled
        << YAML::Key << "pressure_ratio" << YAML::Value << config.forcedInduction.pressureRatio
        << YAML::Key << "full_boost_rpm" << YAML::Value << config.forcedInduction.fullBoostRpm
        << YAML::Key << "compressor_efficiency" << YAML::Value << config.forcedInduction.compressorEfficiency
        << YAML::Key << "charge_temperature_rise_c" << YAML::Value << config.forcedInduction.chargeTemperatureRiseC
        << YAML::EndMap
        << YAML::Key << "thermal" << YAML::Value << YAML::BeginMap
        << YAML::Key << "coolant_mass_kj_per_c" << YAML::Value << config.thermal.coolantMassKjPerC
        << YAML::Key << "oil_mass_kj_per_c" << YAML::Value << config.thermal.oilMassKjPerC
        << YAML::Key << "coolant_heat_share" << YAML::Value << config.thermal.coolantHeatShare
        << YAML::Key << "oil_heat_share" << YAML::Value << config.thermal.oilHeatShare
        << YAML::Key << "cooling_power_kw_per_c" << YAML::Value << config.thermal.coolingPowerKwPerC
        << YAML::Key << "oil_cooling_power_kw_per_c" << YAML::Value << config.thermal.oilCoolingPowerKwPerC
        << YAML::EndMap
        << YAML::Key << "camshafts" << YAML::Value << YAML::BeginMap
        << YAML::Key << "intake_duration_deg" << YAML::Value << config.camshafts.intakeDurationDegrees
        << YAML::Key << "exhaust_duration_deg" << YAML::Value << config.camshafts.exhaustDurationDegrees
        << YAML::Key << "intake_lift_mm" << YAML::Value << config.camshafts.intakeLiftMm
        << YAML::Key << "exhaust_lift_mm" << YAML::Value << config.camshafts.exhaustLiftMm
        << YAML::Key << "intake_centerline_deg" << YAML::Value << config.camshafts.intakeCenterlineDegrees
        << YAML::Key << "exhaust_centerline_deg" << YAML::Value << config.camshafts.exhaustCenterlineDegrees
        << YAML::Key << "intake_flow_coefficient" << YAML::Value << config.camshafts.intakeFlowCoefficient
        << YAML::Key << "exhaust_flow_coefficient" << YAML::Value << config.camshafts.exhaustFlowCoefficient;
    emitLiftProfile(out, "intake_lift_profile", config.camshafts.intakeLiftProfile);
    emitLiftProfile(out, "exhaust_lift_profile", config.camshafts.exhaustLiftProfile);
    out << YAML::EndMap
        << YAML::Key << "crank_journals" << YAML::Value << YAML::BeginSeq;
    for (const auto& journal : config.crankJournals)
        out << YAML::BeginMap << YAML::Key << "id" << YAML::Value << journal.id
            << YAML::Key << "angle_deg" << YAML::Value << journal.angleDegrees
            << YAML::Key << "throw_mm" << YAML::Value << journal.throwMm << YAML::EndMap;
    out << YAML::EndSeq
        << YAML::Key << "exhaust" << YAML::Value << YAML::BeginMap
        << YAML::Key << "primary_length_mm" << YAML::Value << config.exhaust.primaryLengthMm
        << YAML::Key << "primary_diameter_mm" << YAML::Value << config.exhaust.primaryDiameterMm
        << YAML::Key << "collector_diameter_mm" << YAML::Value << config.exhaust.collectorDiameterMm
        << YAML::Key << "muffler_restriction" << YAML::Value << config.exhaust.mufflerRestriction
        << YAML::Key << "outlet_diameter_mm" << YAML::Value << config.exhaust.outletDiameterMm << YAML::EndMap
        << YAML::Key << "transmission" << YAML::Value << YAML::BeginMap
        << YAML::Key << "gear_ratios" << YAML::Value << YAML::Flow << config.transmission.gearRatios
        << YAML::Key << "final_drive_ratio" << YAML::Value << config.transmission.finalDriveRatio
        << YAML::Key << "max_clutch_torque_nm" << YAML::Value << config.transmission.maxClutchTorqueNm << YAML::EndMap
        << YAML::Key << "vehicle" << YAML::Value << YAML::BeginMap
        << YAML::Key << "mass_kg" << YAML::Value << config.vehicle.massKg
        << YAML::Key << "drag_coefficient" << YAML::Value << config.vehicle.dragCoefficient
        << YAML::Key << "frontal_area_m2" << YAML::Value << config.vehicle.frontalAreaM2
        << YAML::Key << "tire_radius_m" << YAML::Value << config.vehicle.tireRadiusM
        << YAML::Key << "rolling_resistance_coefficient" << YAML::Value << config.vehicle.rollingResistanceCoefficient << YAML::EndMap
        << YAML::Key << "firing_order" << YAML::Value << YAML::Flow << config.firingOrder
        << YAML::Key << "cylinders" << YAML::Value << YAML::BeginSeq;
    for (const auto& cylinder : config.cylinders) out << YAML::BeginMap
        << YAML::Key << "id" << YAML::Value << cylinder.id << YAML::Key << "bore_mm" << YAML::Value << cylinder.boreMm
        << YAML::Key << "stroke_mm" << YAML::Value << cylinder.strokeMm << YAML::Key << "connecting_rod_mm" << YAML::Value << cylinder.connectingRodMm
        << YAML::Key << "piston_mass_g" << YAML::Value << cylinder.pistonMassGrams << YAML::Key << "compression_ratio" << YAML::Value << cylinder.compressionRatio
        << YAML::Key << "ignition_offset_deg" << YAML::Value << cylinder.ignitionOffsetDegrees << YAML::Key << "efficiency_offset" << YAML::Value << cylinder.efficiencyOffset
        << YAML::Key << "crank_offset_deg" << YAML::Value << cylinder.crankOffsetDegrees
        << YAML::Key << "crank_journal_id" << YAML::Value << cylinder.crankJournalId
        << YAML::Key << "bank_offset_deg" << YAML::Value << cylinder.bankOffsetDegrees
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
        else if (layout == "radial") config.layout = EngineLayout::radial;
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
        else config.bankAngleDegrees = config.layout == EngineLayout::vLayout ? 60.0
            : (config.layout == EngineLayout::flat ? 180.0 : (config.layout == EngineLayout::radial ? 72.0 : 0.0));
        if (const auto forced = engine["forced_induction"]) {
            config.forcedInduction.enabled = forced["enabled"].as<bool>(false);
            config.forcedInduction.pressureRatio = forced["pressure_ratio"].as<double>(1.0);
            config.forcedInduction.fullBoostRpm = forced["full_boost_rpm"].as<double>(3'500.0);
            config.forcedInduction.compressorEfficiency = forced["compressor_efficiency"].as<double>(0.68);
            config.forcedInduction.chargeTemperatureRiseC = forced["charge_temperature_rise_c"].as<double>(35.0);
        }
        if (const auto thermal = engine["thermal"]) {
            config.thermal.coolantMassKjPerC = thermal["coolant_mass_kj_per_c"].as<double>(config.thermal.coolantMassKjPerC);
            config.thermal.oilMassKjPerC = thermal["oil_mass_kj_per_c"].as<double>(config.thermal.oilMassKjPerC);
            config.thermal.coolantHeatShare = thermal["coolant_heat_share"].as<double>(config.thermal.coolantHeatShare);
            config.thermal.oilHeatShare = thermal["oil_heat_share"].as<double>(config.thermal.oilHeatShare);
            config.thermal.coolingPowerKwPerC = thermal["cooling_power_kw_per_c"].as<double>(config.thermal.coolingPowerKwPerC);
            config.thermal.oilCoolingPowerKwPerC = thermal["oil_cooling_power_kw_per_c"].as<double>(config.thermal.oilCoolingPowerKwPerC);
        }
        if (const auto cams = engine["camshafts"]) {
            config.camshafts = { cams["intake_duration_deg"].as<double>(248.0),
                cams["exhaust_duration_deg"].as<double>(244.0), cams["intake_lift_mm"].as<double>(10.2),
                cams["exhaust_lift_mm"].as<double>(9.8), cams["intake_centerline_deg"].as<double>(110.0),
                cams["exhaust_centerline_deg"].as<double>(112.0) };
            config.camshafts.intakeFlowCoefficient = cams["intake_flow_coefficient"].as<double>(0.62);
            config.camshafts.exhaustFlowCoefficient = cams["exhaust_flow_coefficient"].as<double>(0.62);
            if (cams["intake_lift_profile"]) config.camshafts.intakeLiftProfile = decodeLiftProfile(cams["intake_lift_profile"]);
            if (cams["exhaust_lift_profile"]) config.camshafts.exhaustLiftProfile = decodeLiftProfile(cams["exhaust_lift_profile"]);
        }
        if (const auto crankJournals = engine["crank_journals"]) {
            for (const auto& journal : crankJournals)
                config.crankJournals.push_back({ journal["id"].as<std::uint32_t>(),
                    journal["angle_deg"].as<double>(), journal["throw_mm"].as<double>() });
        }
        if (const auto exhaust = engine["exhaust"]) config.exhaust = {
            exhaust["primary_length_mm"].as<double>(480.0), exhaust["primary_diameter_mm"].as<double>(42.0),
            exhaust["collector_diameter_mm"].as<double>(58.0), exhaust["muffler_restriction"].as<double>(0.28),
            exhaust["outlet_diameter_mm"].as<double>(65.0) };
        if (const auto transmission = engine["transmission"]) {
            if (transmission["gear_ratios"]) config.transmission.gearRatios = transmission["gear_ratios"].as<std::vector<double>>();
            if (transmission["final_drive_ratio"]) config.transmission.finalDriveRatio = transmission["final_drive_ratio"].as<double>();
            if (transmission["max_clutch_torque_nm"]) config.transmission.maxClutchTorqueNm = transmission["max_clutch_torque_nm"].as<double>();
        }
        if (const auto vehicle = engine["vehicle"]) {
            if (vehicle["mass_kg"]) config.vehicle.massKg = vehicle["mass_kg"].as<double>();
            if (vehicle["drag_coefficient"]) config.vehicle.dragCoefficient = vehicle["drag_coefficient"].as<double>();
            if (vehicle["frontal_area_m2"]) config.vehicle.frontalAreaM2 = vehicle["frontal_area_m2"].as<double>();
            if (vehicle["tire_radius_m"]) config.vehicle.tireRadiusM = vehicle["tire_radius_m"].as<double>();
            if (vehicle["rolling_resistance_coefficient"])
                config.vehicle.rollingResistanceCoefficient = vehicle["rolling_resistance_coefficient"].as<double>();
        }
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
            CylinderConfig cylinder {
                cylinderId, item["bore_mm"].as<double>(), item["stroke_mm"].as<double>(),
                item["connecting_rod_mm"].as<double>(), item["piston_mass_g"].as<double>(), item["compression_ratio"].as<double>(),
                item["ignition_offset_deg"].as<double>(), item["efficiency_offset"].as<double>(), crankOffset };
            cylinder.crankJournalId = item["crank_journal_id"].as<std::uint32_t>(0);
            cylinder.bankOffsetDegrees = item["bank_offset_deg"].as<double>(0.0);
            config.cylinders.push_back(cylinder);
        }
        if (const auto error = validateEngineConfig(config)) return { std::nullopt, *error };
        return { std::move(config), {} };
    } catch (const std::exception& error) { return { std::nullopt, error.what() }; }
}
} // namespace enginelab
