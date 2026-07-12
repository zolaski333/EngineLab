#include <enginelab/serialization/JsonEngineSerializer.hpp>
#include <nlohmann/json.hpp>

namespace enginelab {
namespace {
using Json = nlohmann::json;
[[nodiscard]] std::string cycleName(EngineCycle value) { return value == EngineCycle::fourStroke ? "four_stroke" : "two_stroke"; }
[[nodiscard]] std::string fuelName(FuelType value) { return value == FuelType::gasoline ? "gasoline" : "diesel"; }
[[nodiscard]] std::string injectionModeName(InjectionMode value) { return value == InjectionMode::port ? "port" : "direct"; }
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

[[nodiscard]] Json camshaftJson(const CamshaftConfig& cams) {
    return { {"intake_duration_deg", cams.intakeDurationDegrees}, {"exhaust_duration_deg", cams.exhaustDurationDegrees},
        {"intake_lift_mm", cams.intakeLiftMm}, {"exhaust_lift_mm", cams.exhaustLiftMm},
        {"intake_centerline_deg", cams.intakeCenterlineDegrees}, {"exhaust_centerline_deg", cams.exhaustCenterlineDegrees},
        {"intake_flow_coefficient", cams.intakeFlowCoefficient}, {"exhaust_flow_coefficient", cams.exhaustFlowCoefficient},
        {"intake_lift_profile", liftProfileJson(cams.intakeLiftProfile)},
        {"exhaust_lift_profile", liftProfileJson(cams.exhaustLiftProfile)},
        {"variable_profile_enabled", cams.variableProfileEnabled}, {"switch_rpm", cams.switchRpm},
        {"switch_throttle", cams.switchThrottle}, {"high_intake_duration_deg", cams.highIntakeDurationDegrees},
        {"high_exhaust_duration_deg", cams.highExhaustDurationDegrees}, {"high_intake_lift_mm", cams.highIntakeLiftMm},
        {"high_exhaust_lift_mm", cams.highExhaustLiftMm},
        {"high_intake_lift_profile", liftProfileJson(cams.highIntakeLiftProfile)},
        {"high_exhaust_lift_profile", liftProfileJson(cams.highExhaustLiftProfile)} };
}

[[nodiscard]] CamshaftConfig decodeCamshaft(const Json& cams) {
    CamshaftConfig value;
    value.intakeDurationDegrees = cams.value("intake_duration_deg", value.intakeDurationDegrees);
    value.exhaustDurationDegrees = cams.value("exhaust_duration_deg", value.exhaustDurationDegrees);
    value.intakeLiftMm = cams.value("intake_lift_mm", value.intakeLiftMm);
    value.exhaustLiftMm = cams.value("exhaust_lift_mm", value.exhaustLiftMm);
    value.intakeCenterlineDegrees = cams.value("intake_centerline_deg", value.intakeCenterlineDegrees);
    value.exhaustCenterlineDegrees = cams.value("exhaust_centerline_deg", value.exhaustCenterlineDegrees);
    value.intakeFlowCoefficient = cams.value("intake_flow_coefficient", value.intakeFlowCoefficient);
    value.exhaustFlowCoefficient = cams.value("exhaust_flow_coefficient", value.exhaustFlowCoefficient);
    value.variableProfileEnabled = cams.value("variable_profile_enabled", false);
    value.switchRpm = cams.value("switch_rpm", value.switchRpm);
    value.switchThrottle = cams.value("switch_throttle", value.switchThrottle);
    value.highIntakeDurationDegrees = cams.value("high_intake_duration_deg", value.highIntakeDurationDegrees);
    value.highExhaustDurationDegrees = cams.value("high_exhaust_duration_deg", value.highExhaustDurationDegrees);
    value.highIntakeLiftMm = cams.value("high_intake_lift_mm", value.highIntakeLiftMm);
    value.highExhaustLiftMm = cams.value("high_exhaust_lift_mm", value.highExhaustLiftMm);
    if (cams.contains("intake_lift_profile")) value.intakeLiftProfile = decodeLiftProfile(cams.at("intake_lift_profile"));
    if (cams.contains("exhaust_lift_profile")) value.exhaustLiftProfile = decodeLiftProfile(cams.at("exhaust_lift_profile"));
    if (cams.contains("high_intake_lift_profile")) value.highIntakeLiftProfile = decodeLiftProfile(cams.at("high_intake_lift_profile"));
    if (cams.contains("high_exhaust_lift_profile")) value.highExhaustLiftProfile = decodeLiftProfile(cams.at("high_exhaust_lift_profile"));
    return value;
}
}

std::string JsonEngineSerializer::encode(const EngineConfig& config) const {
    Json cylinders = Json::array();
    for (const auto& cylinder : config.cylinders) cylinders.push_back({
        {"id", cylinder.id}, {"bore_mm", cylinder.boreMm}, {"stroke_mm", cylinder.strokeMm},
        {"connecting_rod_mm", cylinder.connectingRodMm}, {"piston_mass_g", cylinder.pistonMassGrams},
        {"connecting_rod_mass_g", cylinder.connectingRodMassGrams},
        {"compression_ratio", cylinder.compressionRatio}, {"ignition_offset_deg", cylinder.ignitionOffsetDegrees},
        {"efficiency_offset", cylinder.efficiencyOffset}, {"crank_offset_deg", cylinder.crankOffsetDegrees},
        {"crank_journal_id", cylinder.crankJournalId}, {"bank_offset_deg", cylinder.bankOffsetDegrees},
        {"bank_id", cylinder.bankId}, {"intake_runner_length_mm", cylinder.intakeRunnerLengthMm},
        {"intake_runner_diameter_mm", cylinder.intakeRunnerDiameterMm},
        {"exhaust_primary_length_mm", cylinder.exhaustPrimaryLengthMm},
        {"sound_attenuation", cylinder.soundAttenuation}, {"blow_by_coefficient", cylinder.blowByCoefficient},
        {"piston_friction_coefficient", cylinder.pistonFrictionCoefficient},
        {"piston_breakaway_force_n", cylinder.pistonBreakawayForceN},
        {"piston_breakaway_velocity_mps", cylinder.pistonBreakawayVelocityMps},
        {"piston_viscous_friction_ns_per_m", cylinder.pistonViscousFrictionNsPerM} });
    Json crankJournals = Json::array();
    for (const auto& journal : config.crankJournals) crankJournals.push_back({
        {"id", journal.id}, {"angle_deg", journal.angleDegrees}, {"throw_mm", journal.throwMm} });
    Json banks = Json::array();
    for (const auto& bank : config.banks) banks.push_back({ {"id", bank.id}, {"angle_deg", bank.angleDegrees},
        {"cylinder_ids", bank.cylinderIds}, {"intake_id", bank.intakeId}, {"exhaust_path_id", bank.exhaustPathId},
        {"camshafts", camshaftJson(bank.camshafts)} });
    Json exhaustPaths = Json::array();
    for (const auto& path : config.exhaustPaths) exhaustPaths.push_back({ {"id", path.id},
        {"cylinder_ids", path.cylinderIds}, {"impulse_response", path.impulseResponsePath},
        {"audio_volume", path.audioVolume}, {"geometry", {{"primary_length_mm", path.geometry.primaryLengthMm},
            {"primary_diameter_mm", path.geometry.primaryDiameterMm}, {"collector_diameter_mm", path.geometry.collectorDiameterMm},
            {"muffler_restriction", path.geometry.mufflerRestriction}, {"outlet_diameter_mm", path.geometry.outletDiameterMm},
            {"collector_volume_l", path.geometry.collectorVolumeLitres},
            {"outlet_discharge_coefficient", path.geometry.outletDischargeCoefficient}}} });
    Json timingCurve = Json::array();
    for (const auto& sample : config.ignition.timingCurve)
        timingCurve.push_back({ {"rpm", sample.rpm}, {"advance_deg", sample.advanceDegrees} });
    Json document = { {"schema_version", config.schemaVersion}, {"engine", {
        {"name", config.name}, {"cycle", cycleName(config.cycle)}, {"fuel", fuelName(config.fuel)},
        {"fuel_properties", {{"name", config.fuelProperties.name},
                             {"lower_heating_value_mj_per_kg", config.fuelProperties.lowerHeatingValueMjPerKg},
                             {"density_kg_per_l", config.fuelProperties.densityKgPerL},
                             {"stoichiometric_afr", config.fuelProperties.stoichiometricAirFuelRatio},
                             {"molar_mass_g_per_mol", config.fuelProperties.molarMassGramsPerMole},
                             {"oxygen_moles_per_fuel_mole", config.fuelProperties.oxygenMolesPerFuelMole},
                             {"product_moles_per_fuel_mole", config.fuelProperties.productMolesPerFuelMole},
                             {"laminar_flame_speed_mps", config.fuelProperties.laminarFlameSpeedMps},
                             {"turbulence_flame_speed_gain", config.fuelProperties.turbulenceFlameSpeedGain}}},
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
        {"camshafts", camshaftJson(config.camshafts)},
        {"intake", {{"plenum_volume_l", config.intake.plenumVolumeLitres},
                     {"throttle_diameter_mm", config.intake.throttleDiameterMm},
                     {"throttle_discharge_coefficient", config.intake.throttleDischargeCoefficient},
                     {"runner_length_mm", config.intake.runnerLengthMm}, {"runner_diameter_mm", config.intake.runnerDiameterMm},
                     {"idle_bypass_area_mm2", config.intake.idleBypassAreaMm2}, {"throttle_gamma", config.intake.throttleGamma}}},
        {"banks", banks}, {"exhaust_paths", exhaustPaths},
        {"ignition", {{"rev_limit_rpm", config.ignition.revLimitRpm},
                        {"limiter_duration_s", config.ignition.limiterDurationSeconds}, {"timing_curve", timingCurve}}},
        {"injection", {{"mode", injectionModeName(config.injection.mode)},
                        {"start_angle_deg", config.injection.startAngleDegrees},
                        {"end_angle_deg", config.injection.endAngleDegrees},
                        {"injector_flow_mg_s", config.injection.injectorFlowMgPerSecond},
                        {"fuel_temperature_c", config.injection.fuelTemperatureC},
                        {"rail_pressure_bar", config.injection.railPressureBar},
                        {"reference_pressure_bar", config.injection.referencePressureBar},
                        {"wall_film_fraction", config.injection.wallFilmFraction},
                        {"vaporisation_time_constant_s", config.injection.vaporisationTimeConstantSeconds},
                        {"latent_heat_kj_per_kg", config.injection.latentHeatKjPerKg},
                        {"direct_charge_cooling_efficiency", config.injection.directChargeCoolingEfficiency},
                        {"port_charge_cooling_efficiency", config.injection.portChargeCoolingEfficiency}}},
        {"solver", {{"mechanical_frequency_hz", config.solver.mechanicalFrequencyHz},
                     {"maximum_frequency_hz", config.solver.maximumMechanicalFrequencyHz},
                     {"maximum_crank_deg_per_step", config.solver.maximumCrankDegreesPerStep},
                     {"gas_substeps", config.solver.gasSubsteps}}},
        {"crank_journals", crankJournals},
        {"exhaust", {{"primary_length_mm", config.exhaust.primaryLengthMm},
                      {"primary_diameter_mm", config.exhaust.primaryDiameterMm},
                      {"collector_diameter_mm", config.exhaust.collectorDiameterMm},
                      {"muffler_restriction", config.exhaust.mufflerRestriction},
                      {"outlet_diameter_mm", config.exhaust.outletDiameterMm},
                      {"collector_volume_l", config.exhaust.collectorVolumeLitres},
                      {"outlet_discharge_coefficient", config.exhaust.outletDischargeCoefficient}}},
        {"transmission", {{"gear_ratios", config.transmission.gearRatios},
                           {"final_drive_ratio", config.transmission.finalDriveRatio},
                           {"max_clutch_torque_nm", config.transmission.maxClutchTorqueNm},
                           {"driveline_efficiency", config.transmission.drivelineEfficiency},
                           {"driven_wheel_inertia_kg_m2", config.transmission.drivenWheelInertiaKgM2},
                           {"clutch_slip_stiffness_nm_per_rpm", config.transmission.clutchSlipStiffnessNmPerRpm},
                           {"clutch_lock_speed_rpm", config.transmission.clutchLockSpeedRpm},
                           {"shift_duration_s", config.transmission.shiftDurationSeconds},
                           {"automatic_shifting", config.transmission.automaticShifting},
                           {"automatic_upshift_rpm", config.transmission.automaticUpshiftRpm},
                           {"automatic_downshift_rpm", config.transmission.automaticDownshiftRpm}}},
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
        if (engine.contains("fuel_properties")) {
            const auto& properties = engine.at("fuel_properties");
            config.fuelProperties.name = properties.value("name", config.fuelProperties.name);
            config.fuelProperties.lowerHeatingValueMjPerKg = properties.value(
                "lower_heating_value_mj_per_kg", config.fuelProperties.lowerHeatingValueMjPerKg);
            config.fuelProperties.densityKgPerL = properties.value(
                "density_kg_per_l", config.fuelProperties.densityKgPerL);
            config.fuelProperties.stoichiometricAirFuelRatio = properties.value(
                "stoichiometric_afr", config.fuelProperties.stoichiometricAirFuelRatio);
            config.fuelProperties.molarMassGramsPerMole = properties.value(
                "molar_mass_g_per_mol", config.fuelProperties.molarMassGramsPerMole);
            config.fuelProperties.oxygenMolesPerFuelMole = properties.value(
                "oxygen_moles_per_fuel_mole", config.fuelProperties.oxygenMolesPerFuelMole);
            config.fuelProperties.productMolesPerFuelMole = properties.value(
                "product_moles_per_fuel_mole", config.fuelProperties.productMolesPerFuelMole);
            config.fuelProperties.laminarFlameSpeedMps = properties.value(
                "laminar_flame_speed_mps", config.fuelProperties.laminarFlameSpeedMps);
            config.fuelProperties.turbulenceFlameSpeedGain = properties.value(
                "turbulence_flame_speed_gain", config.fuelProperties.turbulenceFlameSpeedGain);
        }
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
            config.camshafts = decodeCamshaft(cams);
        }
        config.intake.plenumVolumeLitres = config.plenumVolumeLitres;
        config.intake.throttleDiameterMm = config.throttleDiameterMm;
        if (engine.contains("intake")) {
            const auto& intake = engine.at("intake");
            config.intake.plenumVolumeLitres = intake.value("plenum_volume_l", config.intake.plenumVolumeLitres);
            config.intake.throttleDiameterMm = intake.value("throttle_diameter_mm", config.intake.throttleDiameterMm);
            config.intake.throttleDischargeCoefficient = intake.value("throttle_discharge_coefficient", config.intake.throttleDischargeCoefficient);
            config.intake.runnerLengthMm = intake.value("runner_length_mm", config.intake.runnerLengthMm);
            config.intake.runnerDiameterMm = intake.value("runner_diameter_mm", config.intake.runnerDiameterMm);
            config.intake.idleBypassAreaMm2 = intake.value("idle_bypass_area_mm2", config.intake.idleBypassAreaMm2);
            config.intake.throttleGamma = intake.value("throttle_gamma", config.intake.throttleGamma);
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
                exhaust.value("outlet_diameter_mm", 65.0), exhaust.value("collector_volume_l", 2.0),
                exhaust.value("outlet_discharge_coefficient", 0.72) };
        }
        if (engine.contains("transmission")) {
            const auto& transmission = engine.at("transmission");
            config.transmission.gearRatios = transmission.value("gear_ratios", config.transmission.gearRatios);
            config.transmission.finalDriveRatio = transmission.value("final_drive_ratio", config.transmission.finalDriveRatio);
            config.transmission.maxClutchTorqueNm = transmission.value("max_clutch_torque_nm", config.transmission.maxClutchTorqueNm);
            config.transmission.drivelineEfficiency = transmission.value("driveline_efficiency", config.transmission.drivelineEfficiency);
            config.transmission.drivenWheelInertiaKgM2 = transmission.value("driven_wheel_inertia_kg_m2", config.transmission.drivenWheelInertiaKgM2);
            config.transmission.clutchSlipStiffnessNmPerRpm = transmission.value("clutch_slip_stiffness_nm_per_rpm", config.transmission.clutchSlipStiffnessNmPerRpm);
            config.transmission.clutchLockSpeedRpm = transmission.value("clutch_lock_speed_rpm", config.transmission.clutchLockSpeedRpm);
            config.transmission.shiftDurationSeconds = transmission.value("shift_duration_s", config.transmission.shiftDurationSeconds);
            config.transmission.automaticShifting = transmission.value("automatic_shifting", config.transmission.automaticShifting);
            config.transmission.automaticUpshiftRpm = transmission.value("automatic_upshift_rpm", config.transmission.automaticUpshiftRpm);
            config.transmission.automaticDownshiftRpm = transmission.value("automatic_downshift_rpm", config.transmission.automaticDownshiftRpm);
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
        if (engine.contains("ignition")) {
            const auto& ignition = engine.at("ignition");
            config.ignition.revLimitRpm = ignition.value("rev_limit_rpm", config.redlineRpm);
            config.ignition.limiterDurationSeconds = ignition.value("limiter_duration_s", config.ignition.limiterDurationSeconds);
            if (ignition.contains("timing_curve")) {
                config.ignition.timingCurve.clear();
                for (const auto& sample : ignition.at("timing_curve"))
                    config.ignition.timingCurve.push_back({ sample.at("rpm"), sample.at("advance_deg") });
            }
        } else config.ignition.revLimitRpm = config.redlineRpm;
        if (engine.contains("injection")) {
            const auto& injection = engine.at("injection");
            const auto mode = injection.value("mode", std::string { "direct" });
            if (mode == "port") {
                config.injection.mode = InjectionMode::port;
                config.injection.startAngleDegrees = 250.0;
                config.injection.endAngleDegrees = 620.0;
                config.injection.injectorFlowMgPerSecond = 5'000.0;
                config.injection.railPressureBar = 4.0;
                config.injection.referencePressureBar = 4.0;
                config.injection.wallFilmFraction = 0.22;
                config.injection.vaporisationTimeConstantSeconds = 0.040;
            }
            else if (mode == "direct") config.injection.mode = InjectionMode::direct;
            else return { std::nullopt, "Unknown injection mode: " + mode };
            config.injection.startAngleDegrees = injection.value("start_angle_deg", config.injection.startAngleDegrees);
            config.injection.endAngleDegrees = injection.value("end_angle_deg", config.injection.endAngleDegrees);
            config.injection.injectorFlowMgPerSecond = injection.value("injector_flow_mg_s", config.injection.injectorFlowMgPerSecond);
            config.injection.fuelTemperatureC = injection.value("fuel_temperature_c", config.injection.fuelTemperatureC);
            config.injection.railPressureBar = injection.value("rail_pressure_bar", config.injection.railPressureBar);
            config.injection.referencePressureBar = injection.value("reference_pressure_bar", config.injection.referencePressureBar);
            config.injection.wallFilmFraction = injection.value("wall_film_fraction", config.injection.wallFilmFraction);
            config.injection.vaporisationTimeConstantSeconds = injection.value("vaporisation_time_constant_s", config.injection.vaporisationTimeConstantSeconds);
            config.injection.latentHeatKjPerKg = injection.value("latent_heat_kj_per_kg", config.injection.latentHeatKjPerKg);
            config.injection.directChargeCoolingEfficiency = injection.value("direct_charge_cooling_efficiency", config.injection.directChargeCoolingEfficiency);
            config.injection.portChargeCoolingEfficiency = injection.value("port_charge_cooling_efficiency", config.injection.portChargeCoolingEfficiency);
        }
        if (engine.contains("solver")) {
            const auto& solver = engine.at("solver");
            config.solver.mechanicalFrequencyHz = solver.value("mechanical_frequency_hz", config.solver.mechanicalFrequencyHz);
            config.solver.maximumMechanicalFrequencyHz = solver.value("maximum_frequency_hz", config.solver.maximumMechanicalFrequencyHz);
            config.solver.maximumCrankDegreesPerStep = solver.value("maximum_crank_deg_per_step", config.solver.maximumCrankDegreesPerStep);
            config.solver.gasSubsteps = solver.value("gas_substeps", config.solver.gasSubsteps);
        }
        if (engine.contains("banks")) {
            for (const auto& item : engine.at("banks")) {
                CylinderBankConfig bank;
                bank.id = item.at("id");
                bank.angleDegrees = item.value("angle_deg", 0.0);
                bank.cylinderIds = item.value("cylinder_ids", std::vector<std::uint32_t> {});
                bank.intakeId = item.value("intake_id", std::uint32_t { 0 });
                bank.exhaustPathId = item.value("exhaust_path_id", std::uint32_t { 0 });
                bank.camshafts = item.contains("camshafts") ? decodeCamshaft(item.at("camshafts")) : config.camshafts;
                config.banks.push_back(std::move(bank));
            }
        }
        if (engine.contains("exhaust_paths")) {
            for (const auto& item : engine.at("exhaust_paths")) {
                ExhaustPathConfig path;
                path.id = item.at("id");
                path.cylinderIds = item.value("cylinder_ids", std::vector<std::uint32_t> {});
                path.impulseResponsePath = item.value("impulse_response", std::string {});
                path.audioVolume = item.value("audio_volume", 1.0);
                if (item.contains("geometry")) {
                    const auto& geometry = item.at("geometry");
                    path.geometry.primaryLengthMm = geometry.value("primary_length_mm", path.geometry.primaryLengthMm);
                    path.geometry.primaryDiameterMm = geometry.value("primary_diameter_mm", path.geometry.primaryDiameterMm);
                    path.geometry.collectorDiameterMm = geometry.value("collector_diameter_mm", path.geometry.collectorDiameterMm);
                    path.geometry.mufflerRestriction = geometry.value("muffler_restriction", path.geometry.mufflerRestriction);
                    path.geometry.outletDiameterMm = geometry.value("outlet_diameter_mm", path.geometry.outletDiameterMm);
                    path.geometry.collectorVolumeLitres = geometry.value("collector_volume_l", path.geometry.collectorVolumeLitres);
                    path.geometry.outletDischargeCoefficient = geometry.value("outlet_discharge_coefficient", path.geometry.outletDischargeCoefficient);
                }
                config.exhaustPaths.push_back(std::move(path));
            }
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
            cylinder.connectingRodMassGrams = item.value("connecting_rod_mass_g", cylinder.connectingRodMassGrams);
            cylinder.bankOffsetDegrees = item.value("bank_offset_deg", 0.0);
            cylinder.bankId = item.value("bank_id", std::uint32_t { 0 });
            cylinder.intakeRunnerLengthMm = item.value("intake_runner_length_mm", cylinder.intakeRunnerLengthMm);
            cylinder.intakeRunnerDiameterMm = item.value("intake_runner_diameter_mm", cylinder.intakeRunnerDiameterMm);
            cylinder.exhaustPrimaryLengthMm = item.value("exhaust_primary_length_mm", cylinder.exhaustPrimaryLengthMm);
            cylinder.soundAttenuation = item.value("sound_attenuation", cylinder.soundAttenuation);
            cylinder.blowByCoefficient = item.value("blow_by_coefficient", cylinder.blowByCoefficient);
            cylinder.pistonFrictionCoefficient = item.value("piston_friction_coefficient", cylinder.pistonFrictionCoefficient);
            cylinder.pistonBreakawayForceN = item.value("piston_breakaway_force_n", cylinder.pistonBreakawayForceN);
            cylinder.pistonBreakawayVelocityMps = item.value("piston_breakaway_velocity_mps", cylinder.pistonBreakawayVelocityMps);
            cylinder.pistonViscousFrictionNsPerM = item.value("piston_viscous_friction_ns_per_m", cylinder.pistonViscousFrictionNsPerM);
            config.cylinders.push_back(cylinder);
        }
        if (const auto error = validateEngineConfig(config)) return { std::nullopt, *error };
        return { std::move(config), {} };
    } catch (const std::exception& error) { return { std::nullopt, error.what() }; }
}
} // namespace enginelab
