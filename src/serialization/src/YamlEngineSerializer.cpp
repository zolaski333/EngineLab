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
        << YAML::Key << "exhaust_flow_coefficient" << YAML::Value << config.camshafts.exhaustFlowCoefficient
        << YAML::Key << "variable_profile_enabled" << YAML::Value << config.camshafts.variableProfileEnabled
        << YAML::Key << "switch_rpm" << YAML::Value << config.camshafts.switchRpm
        << YAML::Key << "switch_throttle" << YAML::Value << config.camshafts.switchThrottle
        << YAML::Key << "high_intake_duration_deg" << YAML::Value << config.camshafts.highIntakeDurationDegrees
        << YAML::Key << "high_exhaust_duration_deg" << YAML::Value << config.camshafts.highExhaustDurationDegrees
        << YAML::Key << "high_intake_lift_mm" << YAML::Value << config.camshafts.highIntakeLiftMm
        << YAML::Key << "high_exhaust_lift_mm" << YAML::Value << config.camshafts.highExhaustLiftMm;
    emitLiftProfile(out, "intake_lift_profile", config.camshafts.intakeLiftProfile);
    emitLiftProfile(out, "exhaust_lift_profile", config.camshafts.exhaustLiftProfile);
    emitLiftProfile(out, "high_intake_lift_profile", config.camshafts.highIntakeLiftProfile);
    emitLiftProfile(out, "high_exhaust_lift_profile", config.camshafts.highExhaustLiftProfile);
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
        << YAML::Key << "intake" << YAML::Value << YAML::BeginMap
        << YAML::Key << "plenum_volume_l" << YAML::Value << config.intake.plenumVolumeLitres
        << YAML::Key << "throttle_diameter_mm" << YAML::Value << config.intake.throttleDiameterMm
        << YAML::Key << "throttle_discharge_coefficient" << YAML::Value << config.intake.throttleDischargeCoefficient
        << YAML::Key << "runner_length_mm" << YAML::Value << config.intake.runnerLengthMm
        << YAML::Key << "runner_diameter_mm" << YAML::Value << config.intake.runnerDiameterMm
        << YAML::Key << "idle_bypass_area_mm2" << YAML::Value << config.intake.idleBypassAreaMm2
        << YAML::Key << "throttle_gamma" << YAML::Value << config.intake.throttleGamma << YAML::EndMap
        << YAML::Key << "ignition" << YAML::Value << YAML::BeginMap
        << YAML::Key << "rev_limit_rpm" << YAML::Value << config.ignition.revLimitRpm
        << YAML::Key << "limiter_duration_s" << YAML::Value << config.ignition.limiterDurationSeconds
        << YAML::Key << "timing_curve" << YAML::Value << YAML::BeginSeq;
    for (const auto& sample : config.ignition.timingCurve)
        out << YAML::BeginMap << YAML::Key << "rpm" << YAML::Value << sample.rpm
            << YAML::Key << "advance_deg" << YAML::Value << sample.advanceDegrees << YAML::EndMap;
    out << YAML::EndSeq << YAML::EndMap
        << YAML::Key << "solver" << YAML::Value << YAML::BeginMap
        << YAML::Key << "mechanical_frequency_hz" << YAML::Value << config.solver.mechanicalFrequencyHz
        << YAML::Key << "maximum_frequency_hz" << YAML::Value << config.solver.maximumMechanicalFrequencyHz
        << YAML::Key << "maximum_crank_deg_per_step" << YAML::Value << config.solver.maximumCrankDegreesPerStep
        << YAML::Key << "gas_substeps" << YAML::Value << config.solver.gasSubsteps << YAML::EndMap
        << YAML::Key << "banks" << YAML::Value << YAML::BeginSeq;
    for (const auto& bank : config.banks)
        out << YAML::BeginMap << YAML::Key << "id" << YAML::Value << bank.id
            << YAML::Key << "angle_deg" << YAML::Value << bank.angleDegrees
            << YAML::Key << "cylinder_ids" << YAML::Value << YAML::Flow << bank.cylinderIds
            << YAML::Key << "intake_id" << YAML::Value << bank.intakeId
            << YAML::Key << "exhaust_path_id" << YAML::Value << bank.exhaustPathId
            << YAML::Key << "camshafts" << YAML::Value << YAML::BeginMap
            << YAML::Key << "intake_duration_deg" << YAML::Value << bank.camshafts.intakeDurationDegrees
            << YAML::Key << "exhaust_duration_deg" << YAML::Value << bank.camshafts.exhaustDurationDegrees
            << YAML::Key << "intake_lift_mm" << YAML::Value << bank.camshafts.intakeLiftMm
            << YAML::Key << "exhaust_lift_mm" << YAML::Value << bank.camshafts.exhaustLiftMm
            << YAML::Key << "intake_centerline_deg" << YAML::Value << bank.camshafts.intakeCenterlineDegrees
            << YAML::Key << "exhaust_centerline_deg" << YAML::Value << bank.camshafts.exhaustCenterlineDegrees
            << YAML::Key << "intake_flow_coefficient" << YAML::Value << bank.camshafts.intakeFlowCoefficient
            << YAML::Key << "exhaust_flow_coefficient" << YAML::Value << bank.camshafts.exhaustFlowCoefficient
            << YAML::Key << "variable_profile_enabled" << YAML::Value << bank.camshafts.variableProfileEnabled
            << YAML::Key << "switch_rpm" << YAML::Value << bank.camshafts.switchRpm
            << YAML::Key << "switch_throttle" << YAML::Value << bank.camshafts.switchThrottle
            << YAML::Key << "high_intake_duration_deg" << YAML::Value << bank.camshafts.highIntakeDurationDegrees
            << YAML::Key << "high_exhaust_duration_deg" << YAML::Value << bank.camshafts.highExhaustDurationDegrees
            << YAML::Key << "high_intake_lift_mm" << YAML::Value << bank.camshafts.highIntakeLiftMm
            << YAML::Key << "high_exhaust_lift_mm" << YAML::Value << bank.camshafts.highExhaustLiftMm
            << YAML::EndMap << YAML::EndMap;
    out << YAML::EndSeq << YAML::Key << "exhaust_paths" << YAML::Value << YAML::BeginSeq;
    for (const auto& path : config.exhaustPaths)
        out << YAML::BeginMap << YAML::Key << "id" << YAML::Value << path.id
            << YAML::Key << "cylinder_ids" << YAML::Value << YAML::Flow << path.cylinderIds
            << YAML::Key << "impulse_response" << YAML::Value << path.impulseResponsePath
            << YAML::Key << "audio_volume" << YAML::Value << path.audioVolume
            << YAML::Key << "geometry" << YAML::Value << YAML::BeginMap
            << YAML::Key << "primary_length_mm" << YAML::Value << path.geometry.primaryLengthMm
            << YAML::Key << "primary_diameter_mm" << YAML::Value << path.geometry.primaryDiameterMm
            << YAML::Key << "collector_diameter_mm" << YAML::Value << path.geometry.collectorDiameterMm
            << YAML::Key << "muffler_restriction" << YAML::Value << path.geometry.mufflerRestriction
            << YAML::Key << "outlet_diameter_mm" << YAML::Value << path.geometry.outletDiameterMm
            << YAML::EndMap << YAML::EndMap;
    out << YAML::EndSeq
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
        << YAML::Key << "bank_id" << YAML::Value << cylinder.bankId
        << YAML::Key << "intake_runner_length_mm" << YAML::Value << cylinder.intakeRunnerLengthMm
        << YAML::Key << "intake_runner_diameter_mm" << YAML::Value << cylinder.intakeRunnerDiameterMm
        << YAML::Key << "exhaust_primary_length_mm" << YAML::Value << cylinder.exhaustPrimaryLengthMm
        << YAML::Key << "sound_attenuation" << YAML::Value << cylinder.soundAttenuation
        << YAML::Key << "blow_by_coefficient" << YAML::Value << cylinder.blowByCoefficient
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
            config.camshafts.variableProfileEnabled = cams["variable_profile_enabled"].as<bool>(false);
            config.camshafts.switchRpm = cams["switch_rpm"].as<double>(config.camshafts.switchRpm);
            config.camshafts.switchThrottle = cams["switch_throttle"].as<double>(config.camshafts.switchThrottle);
            config.camshafts.highIntakeDurationDegrees = cams["high_intake_duration_deg"].as<double>(config.camshafts.highIntakeDurationDegrees);
            config.camshafts.highExhaustDurationDegrees = cams["high_exhaust_duration_deg"].as<double>(config.camshafts.highExhaustDurationDegrees);
            config.camshafts.highIntakeLiftMm = cams["high_intake_lift_mm"].as<double>(config.camshafts.highIntakeLiftMm);
            config.camshafts.highExhaustLiftMm = cams["high_exhaust_lift_mm"].as<double>(config.camshafts.highExhaustLiftMm);
            if (cams["intake_lift_profile"]) config.camshafts.intakeLiftProfile = decodeLiftProfile(cams["intake_lift_profile"]);
            if (cams["exhaust_lift_profile"]) config.camshafts.exhaustLiftProfile = decodeLiftProfile(cams["exhaust_lift_profile"]);
            if (cams["high_intake_lift_profile"]) config.camshafts.highIntakeLiftProfile = decodeLiftProfile(cams["high_intake_lift_profile"]);
            if (cams["high_exhaust_lift_profile"]) config.camshafts.highExhaustLiftProfile = decodeLiftProfile(cams["high_exhaust_lift_profile"]);
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
        config.intake.plenumVolumeLitres = config.plenumVolumeLitres;
        config.intake.throttleDiameterMm = config.throttleDiameterMm;
        if (const auto intake = engine["intake"]) {
            config.intake.plenumVolumeLitres = intake["plenum_volume_l"].as<double>(config.intake.plenumVolumeLitres);
            config.intake.throttleDiameterMm = intake["throttle_diameter_mm"].as<double>(config.intake.throttleDiameterMm);
            config.intake.throttleDischargeCoefficient = intake["throttle_discharge_coefficient"].as<double>(config.intake.throttleDischargeCoefficient);
            config.intake.runnerLengthMm = intake["runner_length_mm"].as<double>(config.intake.runnerLengthMm);
            config.intake.runnerDiameterMm = intake["runner_diameter_mm"].as<double>(config.intake.runnerDiameterMm);
            config.intake.idleBypassAreaMm2 = intake["idle_bypass_area_mm2"].as<double>(config.intake.idleBypassAreaMm2);
            config.intake.throttleGamma = intake["throttle_gamma"].as<double>(config.intake.throttleGamma);
        }
        config.ignition.revLimitRpm = config.redlineRpm;
        if (const auto ignition = engine["ignition"]) {
            config.ignition.revLimitRpm = ignition["rev_limit_rpm"].as<double>(config.redlineRpm);
            config.ignition.limiterDurationSeconds = ignition["limiter_duration_s"].as<double>(config.ignition.limiterDurationSeconds);
            if (ignition["timing_curve"]) {
                config.ignition.timingCurve.clear();
                for (const auto& sample : ignition["timing_curve"])
                    config.ignition.timingCurve.push_back({ sample["rpm"].as<double>(), sample["advance_deg"].as<double>() });
            }
        }
        if (const auto solver = engine["solver"]) {
            config.solver.mechanicalFrequencyHz = solver["mechanical_frequency_hz"].as<double>(config.solver.mechanicalFrequencyHz);
            config.solver.maximumMechanicalFrequencyHz = solver["maximum_frequency_hz"].as<double>(config.solver.maximumMechanicalFrequencyHz);
            config.solver.maximumCrankDegreesPerStep = solver["maximum_crank_deg_per_step"].as<double>(config.solver.maximumCrankDegreesPerStep);
            config.solver.gasSubsteps = solver["gas_substeps"].as<std::uint32_t>(config.solver.gasSubsteps);
        }
        if (const auto banks = engine["banks"]) {
            for (const auto& item : banks) {
                CylinderBankConfig bank;
                bank.id = item["id"].as<std::uint32_t>();
                bank.angleDegrees = item["angle_deg"].as<double>(0.0);
                bank.cylinderIds = item["cylinder_ids"].as<std::vector<std::uint32_t>>();
                bank.intakeId = item["intake_id"].as<std::uint32_t>(0);
                bank.exhaustPathId = item["exhaust_path_id"].as<std::uint32_t>(0);
                if (const auto cams = item["camshafts"]) {
                    bank.camshafts.intakeDurationDegrees = cams["intake_duration_deg"].as<double>(bank.camshafts.intakeDurationDegrees);
                    bank.camshafts.exhaustDurationDegrees = cams["exhaust_duration_deg"].as<double>(bank.camshafts.exhaustDurationDegrees);
                    bank.camshafts.intakeLiftMm = cams["intake_lift_mm"].as<double>(bank.camshafts.intakeLiftMm);
                    bank.camshafts.exhaustLiftMm = cams["exhaust_lift_mm"].as<double>(bank.camshafts.exhaustLiftMm);
                    bank.camshafts.intakeCenterlineDegrees = cams["intake_centerline_deg"].as<double>(bank.camshafts.intakeCenterlineDegrees);
                    bank.camshafts.exhaustCenterlineDegrees = cams["exhaust_centerline_deg"].as<double>(bank.camshafts.exhaustCenterlineDegrees);
                    bank.camshafts.intakeFlowCoefficient = cams["intake_flow_coefficient"].as<double>(bank.camshafts.intakeFlowCoefficient);
                    bank.camshafts.exhaustFlowCoefficient = cams["exhaust_flow_coefficient"].as<double>(bank.camshafts.exhaustFlowCoefficient);
                    bank.camshafts.variableProfileEnabled = cams["variable_profile_enabled"].as<bool>(false);
                    bank.camshafts.switchRpm = cams["switch_rpm"].as<double>(bank.camshafts.switchRpm);
                    bank.camshafts.switchThrottle = cams["switch_throttle"].as<double>(bank.camshafts.switchThrottle);
                    bank.camshafts.highIntakeDurationDegrees = cams["high_intake_duration_deg"].as<double>(bank.camshafts.highIntakeDurationDegrees);
                    bank.camshafts.highExhaustDurationDegrees = cams["high_exhaust_duration_deg"].as<double>(bank.camshafts.highExhaustDurationDegrees);
                    bank.camshafts.highIntakeLiftMm = cams["high_intake_lift_mm"].as<double>(bank.camshafts.highIntakeLiftMm);
                    bank.camshafts.highExhaustLiftMm = cams["high_exhaust_lift_mm"].as<double>(bank.camshafts.highExhaustLiftMm);
                } else bank.camshafts = config.camshafts;
                config.banks.push_back(std::move(bank));
            }
        }
        if (const auto paths = engine["exhaust_paths"]) {
            for (const auto& item : paths) {
                ExhaustPathConfig path;
                path.id = item["id"].as<std::uint32_t>();
                path.cylinderIds = item["cylinder_ids"].as<std::vector<std::uint32_t>>();
                path.impulseResponsePath = item["impulse_response"].as<std::string>("");
                path.audioVolume = item["audio_volume"].as<double>(1.0);
                if (const auto geometry = item["geometry"]) path.geometry = {
                    geometry["primary_length_mm"].as<double>(480.0), geometry["primary_diameter_mm"].as<double>(42.0),
                    geometry["collector_diameter_mm"].as<double>(58.0), geometry["muffler_restriction"].as<double>(0.28),
                    geometry["outlet_diameter_mm"].as<double>(65.0) };
                config.exhaustPaths.push_back(std::move(path));
            }
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
            cylinder.bankId = item["bank_id"].as<std::uint32_t>(0);
            cylinder.intakeRunnerLengthMm = item["intake_runner_length_mm"].as<double>(cylinder.intakeRunnerLengthMm);
            cylinder.intakeRunnerDiameterMm = item["intake_runner_diameter_mm"].as<double>(cylinder.intakeRunnerDiameterMm);
            cylinder.exhaustPrimaryLengthMm = item["exhaust_primary_length_mm"].as<double>(cylinder.exhaustPrimaryLengthMm);
            cylinder.soundAttenuation = item["sound_attenuation"].as<double>(cylinder.soundAttenuation);
            cylinder.blowByCoefficient = item["blow_by_coefficient"].as<double>(cylinder.blowByCoefficient);
            config.cylinders.push_back(cylinder);
        }
        if (const auto error = validateEngineConfig(config)) return { std::nullopt, *error };
        return { std::move(config), {} };
    } catch (const std::exception& error) { return { std::nullopt, error.what() }; }
}
} // namespace enginelab
