#include <enginelab/serialization/JsonEngineSerializer.hpp>
#include <nlohmann/json.hpp>
#include <stdexcept>

namespace enginelab {
namespace {
using Json = nlohmann::json;
[[nodiscard]] std::string cycleName(EngineCycle value) { return value == EngineCycle::fourStroke ? "four_stroke" : "two_stroke"; }
[[nodiscard]] std::string fuelName(FuelType value) { return value == FuelType::gasoline ? "gasoline" : "diesel"; }
[[nodiscard]] std::string injectionModeName(InjectionMode value) { return value == InjectionMode::port ? "port" : "direct"; }
[[nodiscard]] std::string rodTypeName(ConnectingRodType value) {
    if (value == ConnectingRodType::master) return "master";
    if (value == ConnectingRodType::articulated) return "articulated";
    return "conventional";
}
[[nodiscard]] std::string forcedInductionTypeName(ForcedInductionType value) {
    return value == ForcedInductionType::supercharger ? "supercharger" : "turbocharger";
}
[[nodiscard]] const char* drivenAxleLayoutName(
    DrivenAxleLayout value) noexcept {
    switch (value) {
    case DrivenAxleLayout::front: return "front";
    case DrivenAxleLayout::rear: return "rear";
    case DrivenAxleLayout::all: return "all";
    }
    return "rear";
}
[[nodiscard]] DrivenAxleLayout decodeDrivenAxleLayout(
    std::string_view value) {
    if (value == "front") return DrivenAxleLayout::front;
    if (value == "rear") return DrivenAxleLayout::rear;
    if (value == "all") return DrivenAxleLayout::all;
    throw std::invalid_argument(
        "Unknown driven axle layout: " + std::string(value));
}
[[nodiscard]] const char* structuralNvhProvenanceName(
    StructuralNvhProvenance value) noexcept {
    switch (value) {
    case StructuralNvhProvenance::estimatedFamily:
        return "estimated_family";
    case StructuralNvhProvenance::calculatedGeometry:
        return "calculated_geometry";
    case StructuralNvhProvenance::measured: return "measured";
    }
    return "estimated_family";
}
[[nodiscard]] StructuralNvhProvenance decodeStructuralNvhProvenance(
    std::string_view value) {
    if (value == "estimated_family")
        return StructuralNvhProvenance::estimatedFamily;
    if (value == "calculated_geometry")
        return StructuralNvhProvenance::calculatedGeometry;
    if (value == "measured")
        return StructuralNvhProvenance::measured;
    throw std::invalid_argument(
        "Unknown structural NVH provenance: " + std::string(value));
}
[[nodiscard]] const char* structuralModeDriveName(
    StructuralModeDrive value) noexcept {
    switch (value) {
    case StructuralModeDrive::headGas: return "head_gas";
    case StructuralModeDrive::bearingAxial: return "bearing_axial";
    case StructuralModeDrive::bearingLateral: return "bearing_lateral";
    case StructuralModeDrive::torsion: return "torsion";
    }
    return "head_gas";
}
[[nodiscard]] StructuralModeDrive decodeStructuralModeDrive(
    std::string_view value) {
    if (value == "head_gas") return StructuralModeDrive::headGas;
    if (value == "bearing_axial")
        return StructuralModeDrive::bearingAxial;
    if (value == "bearing_lateral")
        return StructuralModeDrive::bearingLateral;
    if (value == "torsion") return StructuralModeDrive::torsion;
    throw std::invalid_argument(
        "Unknown structural mode drive: " + std::string(value));
}
[[nodiscard]] Json structuralNvhJson(
    const StructuralNvhConfig& config) {
    auto modes = Json::array();
    for (const auto& mode : config.modes)
        modes.push_back({
            { "name", mode.name },
            { "drive", structuralModeDriveName(mode.drive) },
            { "frequency_hz", mode.frequencyHz },
            { "damping_ratio", mode.dampingRatio },
            { "modal_mass_kg", mode.modalMassKg },
            { "radiating_area_m2", mode.radiatingAreaM2 },
            { "radiation_efficiency", mode.radiationEfficiency },
            { "surface_velocity_rms_scale",
                mode.surfaceVelocityRmsScale },
            { "torque_radius_m", mode.torqueRadiusM },
            { "cylinder_participation",
                mode.cylinderParticipation },
        });
    return {
        { "provenance",
            structuralNvhProvenanceName(config.provenance) },
        { "source", config.source },
        { "modes", std::move(modes) },
    };
}
[[nodiscard]] StructuralNvhConfig decodeStructuralNvh(
    const Json& encoded) {
    StructuralNvhConfig config;
    config.provenance = decodeStructuralNvhProvenance(
        encoded.value("provenance",
            std::string { "estimated_family" }));
    config.source = encoded.value("source", std::string {});
    if (!encoded.contains("modes")) return config;
    for (const auto& item : encoded.at("modes")) {
        StructuralModeConfig mode;
        mode.name = item.at("name").get<std::string>();
        mode.drive = decodeStructuralModeDrive(
            item.at("drive").get<std::string>());
        mode.frequencyHz = item.at("frequency_hz").get<double>();
        mode.dampingRatio =
            item.at("damping_ratio").get<double>();
        mode.modalMassKg = item.at("modal_mass_kg").get<double>();
        mode.radiatingAreaM2 =
            item.at("radiating_area_m2").get<double>();
        mode.radiationEfficiency =
            item.at("radiation_efficiency").get<double>();
        mode.surfaceVelocityRmsScale =
            item.at("surface_velocity_rms_scale").get<double>();
        mode.torqueRadiusM =
            item.at("torque_radius_m").get<double>();
        mode.cylinderParticipation =
            item.at("cylinder_participation")
                .get<std::vector<double>>();
        config.modes.push_back(std::move(mode));
    }
    return config;
}
[[nodiscard]] const char* exhaustComponentTypeName(ExhaustComponentType value) noexcept {
    switch (value) {
    case ExhaustComponentType::pipe: return "pipe";
    case ExhaustComponentType::merge: return "merge";
    case ExhaustComponentType::splitter: return "splitter";
    case ExhaustComponentType::resonator: return "resonator";
    case ExhaustComponentType::muffler: return "muffler";
    case ExhaustComponentType::catalyst: return "catalyst";
    case ExhaustComponentType::outlet: return "outlet";
    }
    return "pipe";
}
[[nodiscard]] std::optional<ExhaustComponentType> decodeExhaustComponentType(std::string_view value) noexcept {
    if (value == "pipe") return ExhaustComponentType::pipe;
    if (value == "merge") return ExhaustComponentType::merge;
    if (value == "splitter") return ExhaustComponentType::splitter;
    if (value == "resonator") return ExhaustComponentType::resonator;
    if (value == "muffler") return ExhaustComponentType::muffler;
    if (value == "catalyst") return ExhaustComponentType::catalyst;
    if (value == "outlet") return ExhaustComponentType::outlet;
    return std::nullopt;
}
[[nodiscard]] const char* terminationName(AcousticTerminationType value) noexcept {
    return value == AcousticTerminationType::flanged ? "flanged" : "unflanged";
}
[[nodiscard]] AcousticTerminationType decodeTermination(std::string_view value) noexcept {
    return value == "flanged" ? AcousticTerminationType::flanged
                              : AcousticTerminationType::unflanged;
}
[[nodiscard]] Json pointJson(const AcousticPoint3M& point) {
    return { { "x", point.x }, { "y", point.y }, { "z", point.z } };
}
[[nodiscard]] AcousticPoint3M decodePoint(
    const Json& value, AcousticPoint3M fallback = {}) {
    return { value.value("x", fallback.x), value.value("y", fallback.y),
        value.value("z", fallback.z) };
}
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
    Json intakeFlowCurve = Json::array();
    for (const auto& sample : cams.intakeFlowCurve)
        intakeFlowCurve.push_back({ { "lift_mm", sample.liftMm }, { "discharge_coefficient", sample.dischargeCoefficient } });
    Json exhaustFlowCurve = Json::array();
    for (const auto& sample : cams.exhaustFlowCurve)
        exhaustFlowCurve.push_back({ { "lift_mm", sample.liftMm }, { "discharge_coefficient", sample.dischargeCoefficient } });
    Json continuousSamples = Json::array();
    for (const auto& sample : cams.continuousControl.samples)
        continuousSamples.push_back({ { "rpm", sample.rpm }, { "load", sample.load },
            { "intake_advance_deg", sample.intakeAdvanceDegrees },
            { "exhaust_advance_deg", sample.exhaustAdvanceDegrees }, { "lift_multiplier", sample.liftMultiplier } });
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
        {"high_exhaust_lift_profile", liftProfileJson(cams.highExhaustLiftProfile)},
        {"intake_flow_curve", intakeFlowCurve}, {"exhaust_flow_curve", exhaustFlowCurve},
        {"continuous_control", {{"enabled", cams.continuousControl.enabled},
            {"response_frequency_hz", cams.continuousControl.responseFrequencyHz}, {"samples", continuousSamples}}} };
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
    if (cams.contains("intake_flow_curve")) for (const auto& sample : cams.at("intake_flow_curve"))
        value.intakeFlowCurve.push_back({ sample.at("lift_mm"), sample.at("discharge_coefficient") });
    if (cams.contains("exhaust_flow_curve")) for (const auto& sample : cams.at("exhaust_flow_curve"))
        value.exhaustFlowCurve.push_back({ sample.at("lift_mm"), sample.at("discharge_coefficient") });
    if (cams.contains("continuous_control")) {
        const auto& control = cams.at("continuous_control");
        value.continuousControl.enabled = control.value("enabled", false);
        value.continuousControl.responseFrequencyHz = control.value("response_frequency_hz", value.continuousControl.responseFrequencyHz);
        if (control.contains("samples")) for (const auto& sample : control.at("samples"))
            value.continuousControl.samples.push_back({ sample.at("rpm"), sample.at("load"),
                sample.at("intake_advance_deg"), sample.at("exhaust_advance_deg"), sample.at("lift_multiplier") });
    }
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
        {"piston_viscous_friction_ns_per_m", cylinder.pistonViscousFrictionNsPerM},
        {"connecting_rod_type", rodTypeName(cylinder.connectingRodType)},
        {"master_cylinder_id", cylinder.masterCylinderId},
        {"articulated_journal_radius_mm", cylinder.articulatedJournalRadiusMm},
        {"articulated_journal_angle_deg", cylinder.articulatedJournalAngleDegrees},
        {"deck_height_mm", cylinder.deckHeightMm}, {"compression_height_mm", cylinder.compressionHeightMm},
        {"wrist_pin_offset_mm", cylinder.wristPinOffsetMm}, {"piston_crown_volume_cc", cylinder.pistonCrownVolumeCc},
        {"head_chamber_volume_cc", cylinder.headChamberVolumeCc},
        {"head_gasket_thickness_mm", cylinder.headGasketThicknessMm},
        {"connecting_rod_inertia_kg_m2", cylinder.connectingRodMomentOfInertiaKgM2},
        {"intake_valve_count", cylinder.intakeValveCount},
        {"exhaust_valve_count", cylinder.exhaustValveCount},
        {"intake_valve_diameter_mm", cylinder.intakeValveDiameterMm},
        {"exhaust_valve_diameter_mm", cylinder.exhaustValveDiameterMm} });
    Json crankJournals = Json::array();
    for (const auto& journal : config.crankJournals) crankJournals.push_back({
        {"id", journal.id}, {"angle_deg", journal.angleDegrees}, {"throw_mm", journal.throwMm},
        {"crankshaft_id", journal.crankshaftId} });
    Json crankshafts = Json::array();
    for (const auto& crankshaft : config.crankshafts) crankshafts.push_back({
        {"id", crankshaft.id}, {"position_x_mm", crankshaft.positionXMm}, {"position_y_mm", crankshaft.positionYMm},
        {"phase_offset_deg", crankshaft.phaseOffsetDegrees}, {"rotation_ratio", crankshaft.rotationRatio},
        {"mass_kg", crankshaft.massKg}, {"flywheel_mass_kg", crankshaft.flywheelMassKg},
        {"moment_of_inertia_kg_m2", crankshaft.momentOfInertiaKgM2},
        {"friction_torque_nm", crankshaft.frictionTorqueNm} });
    Json intakePaths = Json::array();
    for (const auto& path : config.intakePaths) intakePaths.push_back({ {"id", path.id},
        {"cylinder_ids", path.cylinderIds}, {"geometry", {{"plenum_volume_l", path.geometry.plenumVolumeLitres},
            {"throttle_diameter_mm", path.geometry.throttleDiameterMm},
            {"throttle_count", path.geometry.throttleCount},
            {"throttle_discharge_coefficient", path.geometry.throttleDischargeCoefficient},
            {"runner_length_mm", path.geometry.runnerLengthMm}, {"runner_diameter_mm", path.geometry.runnerDiameterMm},
            {"runner_plenum_diameter_mm", path.geometry.runnerPlenumDiameterMm},
            {"airbox_volume_l", path.geometry.airboxVolumeLitres},
            {"inlet_duct_length_mm", path.geometry.inletDuctLengthMm},
            {"inlet_duct_diameter_mm", path.geometry.inletDuctDiameterMm},
            {"bellmouth_diameter_mm", path.geometry.bellmouthDiameterMm},
            {"idle_bypass_area_mm2", path.geometry.idleBypassAreaMm2}, {"throttle_gamma", path.geometry.throttleGamma}}} });
    Json banks = Json::array();
    for (const auto& bank : config.banks) banks.push_back({ {"id", bank.id}, {"angle_deg", bank.angleDegrees},
        {"cylinder_ids", bank.cylinderIds}, {"intake_id", bank.intakeId}, {"exhaust_path_id", bank.exhaustPathId},
        {"camshafts", camshaftJson(bank.camshafts)} });
    Json exhaustPaths = Json::array();
    for (const auto& path : config.exhaustPaths) {
        Json encodedPath = { {"id", path.id}, {"cylinder_ids", path.cylinderIds},
            {"impulse_response", path.impulseResponsePath}, {"audio_volume", path.audioVolume},
            {"acoustic_position_m", pointJson(path.acousticPositionM)},
            {"acoustic_axis", pointJson(path.acousticAxis)},
            {"acoustic_termination", terminationName(path.acousticTermination)},
            {"geometry", {{"primary_length_mm", path.geometry.primaryLengthMm},
                {"primary_diameter_mm", path.geometry.primaryDiameterMm},
                {"collector_diameter_mm", path.geometry.collectorDiameterMm},
                {"muffler_restriction", path.geometry.mufflerRestriction},
                {"outlet_diameter_mm", path.geometry.outletDiameterMm},
                {"collector_volume_l", path.geometry.collectorVolumeLitres},
                {"outlet_discharge_coefficient", path.geometry.outletDischargeCoefficient},
                {"muffler_chamber_diameter_mm", path.geometry.mufflerChamberDiameterMm},
                {"muffler_chamber_length_mm", path.geometry.mufflerChamberLengthMm},
                {"muffler_packing_flow_resistivity_pa_s_m2", path.geometry.mufflerPackingFlowResistivityPaSPerM2},
                {"muffler_packing_thickness_mm", path.geometry.mufflerPackingThicknessMm},
                {"muffler_perforated_open_area_ratio", path.geometry.mufflerPerforatedOpenAreaRatio}}} };
        if (path.network) {
            Json components = Json::array();
            for (const auto& component : path.network->components)
                components.push_back({ {"id", component.id}, {"type", exhaustComponentTypeName(component.type)},
                    {"length_mm", component.lengthMm}, {"diameter_mm", component.diameterMm},
                    {"outlet_diameter_mm", component.outletDiameterMm},
                    {"volume_l", component.volumeLitres}, {"restriction", component.restriction},
                    {"resonance_hz", component.resonanceHz}, {"acoustic_gain", component.acousticGain},
                    {"discharge_coefficient", component.dischargeCoefficient},
                    {"packing_flow_resistivity_pa_s_m2", component.packingFlowResistivityPaSPerM2},
                    {"packing_thickness_mm", component.packingThicknessMm},
                    {"perforated_open_area_ratio", component.perforatedOpenAreaRatio},
                    {"acoustic_position_m", pointJson(component.acousticPositionM)},
                    {"acoustic_axis", pointJson(component.acousticAxis)},
                    {"acoustic_termination", terminationName(component.acousticTermination)} });
            Json cylinderConnections = Json::array();
            for (const auto& connection : path.network->cylinderConnections)
                cylinderConnections.push_back({ {"cylinder_id", connection.cylinderId},
                    {"to_component_id", connection.componentId} });
            Json connections = Json::array();
            for (const auto& connection : path.network->connections)
                connections.push_back({ {"from_component_id", connection.fromComponentId},
                    {"to_component_id", connection.toComponentId} });
            encodedPath["graph"] = { {"components", std::move(components)},
                {"cylinder_connections", std::move(cylinderConnections)},
                {"connections", std::move(connections)} };
        }
        exhaustPaths.push_back(std::move(encodedPath));
    }
    Json timingCurve = Json::array();
    for (const auto& sample : config.ignition.timingCurve)
        timingCurve.push_back({ {"rpm", sample.rpm}, {"advance_deg", sample.advanceDegrees} });
    Json fullLoadFuelLimit = Json::array();
    for (const auto& sample : config.injection.fullLoadFuelLimit)
        fullLoadFuelLimit.push_back({
            { "rpm", sample.rpm },
            { "mg_per_cycle", sample.milligramsPerCycle } });
    Json document = { {"schema_version", currentEngineSchemaVersion}, {"engine", {
        {"name", config.name}, {"cycle", cycleName(config.cycle)}, {"fuel", fuelName(config.fuel)},
        {"fuel_properties", {{"name", config.fuelProperties.name},
                             {"lower_heating_value_mj_per_kg", config.fuelProperties.lowerHeatingValueMjPerKg},
                             {"density_kg_per_l", config.fuelProperties.densityKgPerL},
                             {"stoichiometric_afr", config.fuelProperties.stoichiometricAirFuelRatio},
                             {"molar_mass_g_per_mol", config.fuelProperties.molarMassGramsPerMole},
                             {"oxygen_moles_per_fuel_mole", config.fuelProperties.oxygenMolesPerFuelMole},
                             {"product_moles_per_fuel_mole", config.fuelProperties.productMolesPerFuelMole},
                             {"laminar_flame_speed_mps", config.fuelProperties.laminarFlameSpeedMps},
                             {"turbulence_flame_speed_gain", config.fuelProperties.turbulenceFlameSpeedGain},
                             {"cetane_number", config.fuelProperties.cetaneNumber}}},
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
                               {"type", forcedInductionTypeName(config.forcedInduction.type)},
                               {"pressure_ratio", config.forcedInduction.pressureRatio},
                               {"full_boost_rpm", config.forcedInduction.fullBoostRpm},
                               {"compressor_efficiency", config.forcedInduction.compressorEfficiency},
                               {"charge_temperature_rise_c", config.forcedInduction.chargeTemperatureRiseC},
                               {"turbine_efficiency", config.forcedInduction.turbineEfficiency},
                               {"shaft_inertia_kg_m2", config.forcedInduction.shaftInertiaKgM2},
                               {"wastegate_pressure_ratio", config.forcedInduction.wastegatePressureRatio},
                               {"design_shaft_speed_rpm", config.forcedInduction.designShaftSpeedRpm},
                               {"bearing_friction_power_w", config.forcedInduction.bearingFrictionPowerWatts},
                               {"turbine_flow_area_mm2", config.forcedInduction.turbineFlowAreaMm2},
                               {"wastegate_flow_area_mm2", config.forcedInduction.wastegateFlowAreaMm2},
                               {"compressor_blade_count", config.forcedInduction.compressorBladeCount},
                               {"turbine_blade_count", config.forcedInduction.turbineBladeCount},
                               {"supercharger_lobe_count", config.forcedInduction.superchargerLobeCount},
                               {"supercharger_drive_ratio", config.forcedInduction.superchargerDriveRatio},
                               {"compressor_inducer_diameter_mm", config.forcedInduction.compressorInducerDiameterMm},
                               {"turbine_exducer_diameter_mm", config.forcedInduction.turbineExducerDiameterMm},
                               {"blow_off_valve_flow_area_mm2", config.forcedInduction.blowOffValveFlowAreaMm2},
                               {"blow_off_valve_opening_pressure_ratio", config.forcedInduction.blowOffValveOpeningPressureRatio},
                               {"blow_off_valve_discharge_coefficient", config.forcedInduction.blowOffValveDischargeCoefficient},
                               {"tonal_acoustic_efficiency", config.forcedInduction.tonalAcousticEfficiency},
                               {"turbulent_jet_noise_coefficient", config.forcedInduction.turbulentJetNoiseCoefficient}}},
        {"thermal", {{"coolant_mass_kj_per_c", config.thermal.coolantMassKjPerC},
                      {"oil_mass_kj_per_c", config.thermal.oilMassKjPerC},
                      {"coolant_heat_share", config.thermal.coolantHeatShare},
                      {"oil_heat_share", config.thermal.oilHeatShare},
                      {"cooling_power_kw_per_c", config.thermal.coolingPowerKwPerC},
                      {"oil_cooling_power_kw_per_c", config.thermal.oilCoolingPowerKwPerC}}},
        {"combustion_calibration", {{"base_ignition_delay_s", config.combustionCalibration.baseIgnitionDelaySeconds},
                      {"ignition_delay_temperature_exponent", config.combustionCalibration.ignitionDelayTemperatureExponent},
                      {"ignition_delay_pressure_exponent", config.combustionCalibration.ignitionDelayPressureExponent},
                      {"wall_heat_transfer_w_per_k", config.combustionCalibration.wallHeatTransferCoefficientWPerK},
                      {"residual_dilution_sensitivity", config.combustionCalibration.residualDilutionSensitivity},
                      {"chamber_turbulence_intensity_ratio", config.combustionCalibration.chamberTurbulenceIntensityRatio},
                      {"ignition_site_count", config.combustionCalibration.ignitionSiteCount},
                      {"compression_ignition_delay_scale", config.combustionCalibration.compressionIgnitionDelayScale},
                      {"compression_ignition_mixing_time_s", config.combustionCalibration.compressionIgnitionMixingTimeSeconds},
                      {"compression_ignition_premixed_fraction", config.combustionCalibration.compressionIgnitionPremixedFraction},
                      {"cycle_variation_cov", config.combustionCalibration.cycleVariationCoefficientOfVariation},
                      {"cycle_variation_correlation", config.combustionCalibration.cycleVariationCorrelation}}},
        {"exhaust_afterfire", {{"enabled", config.exhaustAfterfire.enabled},
                      {"ignition_temperature_k", config.exhaustAfterfire.ignitionTemperatureK},
                      {"reaction_time_constant_s", config.exhaustAfterfire.reactionTimeConstantSeconds},
                      {"reaction_efficiency", config.exhaustAfterfire.reactionEfficiency},
                      {"overrun_fuel_fraction", config.exhaustAfterfire.overrunFuelFraction},
                      {"overrun_minimum_rpm", config.exhaustAfterfire.overrunMinimumRpm},
                      {"overrun_maximum_throttle", config.exhaustAfterfire.overrunMaximumThrottle}}},
        {"runner_acoustics", {{"enabled", config.runnerAcoustics.enabled},
                      {"damping_ratio", config.runnerAcoustics.dampingRatio},
                      {"coupling_gain", config.runnerAcoustics.couplingGain},
                      {"maximum_pressure_amplitude_kpa", config.runnerAcoustics.maximumPressureAmplitudeKpa}}},
        {"structural_nvh", structuralNvhJson(config.structuralNvh)},
        {"camshafts", camshaftJson(config.camshafts)},
        {"intake", {{"plenum_volume_l", config.intake.plenumVolumeLitres},
                     {"throttle_diameter_mm", config.intake.throttleDiameterMm},
                     {"throttle_count", config.intake.throttleCount},
                     {"throttle_discharge_coefficient", config.intake.throttleDischargeCoefficient},
                     {"runner_length_mm", config.intake.runnerLengthMm}, {"runner_diameter_mm", config.intake.runnerDiameterMm},
                     {"runner_plenum_diameter_mm", config.intake.runnerPlenumDiameterMm},
                     {"airbox_volume_l", config.intake.airboxVolumeLitres},
                     {"inlet_duct_length_mm", config.intake.inletDuctLengthMm},
                     {"inlet_duct_diameter_mm", config.intake.inletDuctDiameterMm},
                     {"bellmouth_diameter_mm", config.intake.bellmouthDiameterMm},
                     {"idle_bypass_area_mm2", config.intake.idleBypassAreaMm2}, {"throttle_gamma", config.intake.throttleGamma}}},
        {"intake_paths", intakePaths}, {"banks", banks}, {"exhaust_paths", exhaustPaths},
        {"acoustic_observer", {
            {"left_microphone_m", pointJson(config.acousticObserver.leftMicrophoneM)},
            {"right_microphone_m", pointJson(config.acousticObserver.rightMicrophoneM)},
            {"sound_speed_mps", config.acousticObserver.soundSpeedMps}}},
        {"ignition", {{"rev_limit_rpm", config.ignition.revLimitRpm},
                        {"limiter_duration_s", config.ignition.limiterDurationSeconds},
                        {"limiter_keeps_fuel", config.ignition.limiterKeepsFuel},
                        {"timing_curve", timingCurve}}},
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
                        {"port_charge_cooling_efficiency", config.injection.portChargeCoolingEfficiency},
                        {"direct_spray_vaporisation_time_constant_s", config.injection.directSprayVaporisationTimeConstantSeconds},
                        {"direct_spray_entrainment_time_constant_s", config.injection.directSprayEntrainmentTimeConstantSeconds},
                        {"full_load_fuel_limit", fullLoadFuelLimit}}},
        {"solver", {{"mechanical_frequency_hz", config.solver.mechanicalFrequencyHz},
                     {"maximum_frequency_hz", config.solver.maximumMechanicalFrequencyHz},
                     {"maximum_crank_deg_per_step", config.solver.maximumCrankDegreesPerStep},
                     {"gas_substeps", config.solver.gasSubsteps}}},
        {"crankshafts", crankshafts}, {"crank_journals", crankJournals},
        {"exhaust", {{"primary_length_mm", config.exhaust.primaryLengthMm},
                      {"primary_diameter_mm", config.exhaust.primaryDiameterMm},
                      {"collector_diameter_mm", config.exhaust.collectorDiameterMm},
                      {"muffler_restriction", config.exhaust.mufflerRestriction},
                      {"outlet_diameter_mm", config.exhaust.outletDiameterMm},
                      {"collector_volume_l", config.exhaust.collectorVolumeLitres},
                      {"outlet_discharge_coefficient", config.exhaust.outletDischargeCoefficient},
                      {"muffler_chamber_diameter_mm", config.exhaust.mufflerChamberDiameterMm},
                      {"muffler_chamber_length_mm", config.exhaust.mufflerChamberLengthMm},
                      {"muffler_packing_flow_resistivity_pa_s_m2", config.exhaust.mufflerPackingFlowResistivityPaSPerM2},
                      {"muffler_packing_thickness_mm", config.exhaust.mufflerPackingThicknessMm},
                      {"muffler_perforated_open_area_ratio", config.exhaust.mufflerPerforatedOpenAreaRatio}}},
        {"transmission", {{"gear_ratios", config.transmission.gearRatios},
                           {"final_drive_ratio", config.transmission.finalDriveRatio},
                           {"max_clutch_torque_nm", config.transmission.maxClutchTorqueNm},
                           {"driveline_efficiency", config.transmission.drivelineEfficiency},
                           {"driven_wheel_inertia_kg_m2", config.transmission.drivenWheelInertiaKgM2},
                           {"clutch_lock_speed_rpm", config.transmission.clutchLockSpeedRpm},
                           {"shift_duration_s", config.transmission.shiftDurationSeconds},
                           {"automatic_shifting", config.transmission.automaticShifting},
                           {"automatic_upshift_rpm", config.transmission.automaticUpshiftRpm},
                           {"automatic_downshift_rpm", config.transmission.automaticDownshiftRpm},
                           {"reverse_ratio", config.transmission.reverseRatio},
                           {"gearbox_input_inertia_kg_m2", config.transmission.gearboxInputInertiaKgM2},
                           {"differential_inertia_kg_m2", config.transmission.differentialInertiaKgM2},
                           {"clutch_thermal_capacity_j_per_c", config.transmission.clutchThermalCapacityJPerC},
                           {"clutch_cooling_w_per_c", config.transmission.clutchCoolingWPerC},
                           {"clutch_fade_start_temperature_c", config.transmission.clutchFadeStartTemperatureC},
                           {"clutch_failure_temperature_c", config.transmission.clutchFailureTemperatureC},
                           {"shift_torque_cut_fraction", config.transmission.shiftTorqueCutFraction}}},
        {"vehicle", {{"mass_kg", config.vehicle.massKg},
                      {"drag_coefficient", config.vehicle.dragCoefficient},
                      {"frontal_area_m2", config.vehicle.frontalAreaM2},
                      {"tire_radius_m", config.vehicle.tireRadiusM},
                      {"rolling_resistance_coefficient", config.vehicle.rollingResistanceCoefficient},
                      {"tire_friction_coefficient", config.vehicle.tireFrictionCoefficient},
                      {"tyre_grip_limit_enabled", config.vehicle.tyreGripLimitEnabled},
                      {"driven_axle_layout", drivenAxleLayoutName(config.vehicle.drivenAxleLayout)},
                      {"driven_axle_weight_fraction", config.vehicle.drivenAxleWeightFraction},
                      {"wheelbase_m", config.vehicle.wheelbaseM},
                      {"center_of_gravity_height_m", config.vehicle.centerOfGravityHeightM},
                      {"maximum_brake_force_n", config.vehicle.maximumBrakeForceN}}} }} };
    return document.dump(2);
}

EngineDecodeResult JsonEngineSerializer::decode(std::string_view text) const noexcept {
    try {
        const auto document = Json::parse(text);
        const auto& engine = document.at("engine");
        EngineConfig config;
        config.schemaVersion = document.at("schema_version").get<std::uint32_t>();
        if (config.schemaVersion < minimumSupportedEngineSchemaVersion
                || config.schemaVersion > currentEngineSchemaVersion)
            return { std::nullopt, "Unsupported schema version" };
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
            config.fuelProperties.cetaneNumber = properties.value(
                "cetane_number", config.fuelProperties.cetaneNumber);
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
            const auto type = forced.value("type", std::string { "turbocharger" });
            if (type == "turbocharger") config.forcedInduction.type = ForcedInductionType::turbocharger;
            else if (type == "supercharger") config.forcedInduction.type = ForcedInductionType::supercharger;
            else return { std::nullopt, "Unknown forced-induction type: " + type };
            config.forcedInduction.pressureRatio = forced.value("pressure_ratio", 1.0);
            config.forcedInduction.fullBoostRpm = forced.value("full_boost_rpm", 3'500.0);
            config.forcedInduction.compressorEfficiency = forced.value("compressor_efficiency", 0.68);
            config.forcedInduction.chargeTemperatureRiseC = forced.value("charge_temperature_rise_c", 35.0);
            config.forcedInduction.turbineEfficiency = forced.value("turbine_efficiency", config.forcedInduction.turbineEfficiency);
            config.forcedInduction.shaftInertiaKgM2 = forced.value("shaft_inertia_kg_m2", config.forcedInduction.shaftInertiaKgM2);
            config.forcedInduction.wastegatePressureRatio = forced.value("wastegate_pressure_ratio", config.forcedInduction.wastegatePressureRatio);
            config.forcedInduction.designShaftSpeedRpm = forced.value("design_shaft_speed_rpm", config.forcedInduction.designShaftSpeedRpm);
            config.forcedInduction.bearingFrictionPowerWatts = forced.value("bearing_friction_power_w", config.forcedInduction.bearingFrictionPowerWatts);
            config.forcedInduction.turbineFlowAreaMm2 = forced.value("turbine_flow_area_mm2", config.forcedInduction.turbineFlowAreaMm2);
            config.forcedInduction.wastegateFlowAreaMm2 = forced.value("wastegate_flow_area_mm2", config.forcedInduction.wastegateFlowAreaMm2);
            config.forcedInduction.compressorBladeCount = forced.value("compressor_blade_count", config.forcedInduction.compressorBladeCount);
            config.forcedInduction.turbineBladeCount = forced.value("turbine_blade_count", config.forcedInduction.turbineBladeCount);
            config.forcedInduction.superchargerLobeCount = forced.value("supercharger_lobe_count", config.forcedInduction.superchargerLobeCount);
            config.forcedInduction.superchargerDriveRatio = forced.value("supercharger_drive_ratio", config.forcedInduction.superchargerDriveRatio);
            config.forcedInduction.compressorInducerDiameterMm = forced.value("compressor_inducer_diameter_mm", config.forcedInduction.compressorInducerDiameterMm);
            config.forcedInduction.turbineExducerDiameterMm = forced.value("turbine_exducer_diameter_mm", config.forcedInduction.turbineExducerDiameterMm);
            config.forcedInduction.blowOffValveFlowAreaMm2 = forced.value("blow_off_valve_flow_area_mm2", config.forcedInduction.blowOffValveFlowAreaMm2);
            config.forcedInduction.blowOffValveOpeningPressureRatio = forced.value("blow_off_valve_opening_pressure_ratio", config.forcedInduction.blowOffValveOpeningPressureRatio);
            config.forcedInduction.blowOffValveDischargeCoefficient = forced.value("blow_off_valve_discharge_coefficient", config.forcedInduction.blowOffValveDischargeCoefficient);
            config.forcedInduction.tonalAcousticEfficiency = forced.value("tonal_acoustic_efficiency", config.forcedInduction.tonalAcousticEfficiency);
            config.forcedInduction.turbulentJetNoiseCoefficient = forced.value("turbulent_jet_noise_coefficient", config.forcedInduction.turbulentJetNoiseCoefficient);
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
        if (engine.contains("combustion_calibration")) {
            const auto& calibration = engine.at("combustion_calibration");
            config.combustionCalibration.baseIgnitionDelaySeconds = calibration.value("base_ignition_delay_s", config.combustionCalibration.baseIgnitionDelaySeconds);
            config.combustionCalibration.ignitionDelayTemperatureExponent = calibration.value("ignition_delay_temperature_exponent", config.combustionCalibration.ignitionDelayTemperatureExponent);
            config.combustionCalibration.ignitionDelayPressureExponent = calibration.value("ignition_delay_pressure_exponent", config.combustionCalibration.ignitionDelayPressureExponent);
            config.combustionCalibration.wallHeatTransferCoefficientWPerK = calibration.value("wall_heat_transfer_w_per_k", config.combustionCalibration.wallHeatTransferCoefficientWPerK);
            config.combustionCalibration.residualDilutionSensitivity = calibration.value("residual_dilution_sensitivity", config.combustionCalibration.residualDilutionSensitivity);
            config.combustionCalibration.chamberTurbulenceIntensityRatio = calibration.value("chamber_turbulence_intensity_ratio", config.combustionCalibration.chamberTurbulenceIntensityRatio);
            config.combustionCalibration.ignitionSiteCount = calibration.value("ignition_site_count", config.combustionCalibration.ignitionSiteCount);
            config.combustionCalibration.compressionIgnitionDelayScale = calibration.value("compression_ignition_delay_scale", config.combustionCalibration.compressionIgnitionDelayScale);
            config.combustionCalibration.compressionIgnitionMixingTimeSeconds = calibration.value("compression_ignition_mixing_time_s", config.combustionCalibration.compressionIgnitionMixingTimeSeconds);
            config.combustionCalibration.compressionIgnitionPremixedFraction = calibration.value("compression_ignition_premixed_fraction", config.combustionCalibration.compressionIgnitionPremixedFraction);
            config.combustionCalibration.cycleVariationCoefficientOfVariation = calibration.value("cycle_variation_cov", config.combustionCalibration.cycleVariationCoefficientOfVariation);
            config.combustionCalibration.cycleVariationCorrelation = calibration.value("cycle_variation_correlation", config.combustionCalibration.cycleVariationCorrelation);
        }
        if (engine.contains("exhaust_afterfire")) {
            const auto& afterfire = engine.at("exhaust_afterfire");
            config.exhaustAfterfire.enabled = afterfire.value("enabled", config.exhaustAfterfire.enabled);
            config.exhaustAfterfire.ignitionTemperatureK = afterfire.value("ignition_temperature_k", config.exhaustAfterfire.ignitionTemperatureK);
            config.exhaustAfterfire.reactionTimeConstantSeconds = afterfire.value("reaction_time_constant_s", config.exhaustAfterfire.reactionTimeConstantSeconds);
            config.exhaustAfterfire.reactionEfficiency = afterfire.value("reaction_efficiency", config.exhaustAfterfire.reactionEfficiency);
            config.exhaustAfterfire.overrunFuelFraction = afterfire.value("overrun_fuel_fraction", config.exhaustAfterfire.overrunFuelFraction);
            config.exhaustAfterfire.overrunMinimumRpm = afterfire.value("overrun_minimum_rpm", config.exhaustAfterfire.overrunMinimumRpm);
            config.exhaustAfterfire.overrunMaximumThrottle = afterfire.value("overrun_maximum_throttle", config.exhaustAfterfire.overrunMaximumThrottle);
        }
        if (engine.contains("runner_acoustics")) {
            const auto& acoustics = engine.at("runner_acoustics");
            config.runnerAcoustics.enabled = acoustics.value("enabled", config.runnerAcoustics.enabled);
            config.runnerAcoustics.dampingRatio = acoustics.value("damping_ratio", config.runnerAcoustics.dampingRatio);
            config.runnerAcoustics.couplingGain = acoustics.value("coupling_gain", config.runnerAcoustics.couplingGain);
            config.runnerAcoustics.maximumPressureAmplitudeKpa = acoustics.value("maximum_pressure_amplitude_kpa", config.runnerAcoustics.maximumPressureAmplitudeKpa);
        }
        if (engine.contains("structural_nvh"))
            config.structuralNvh =
                decodeStructuralNvh(engine.at("structural_nvh"));
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
            config.intake.throttleCount = intake.value("throttle_count", config.intake.throttleCount);
            config.intake.throttleDischargeCoefficient = intake.value("throttle_discharge_coefficient", config.intake.throttleDischargeCoefficient);
            config.intake.runnerLengthMm = intake.value("runner_length_mm", config.intake.runnerLengthMm);
            config.intake.runnerDiameterMm = intake.value("runner_diameter_mm", config.intake.runnerDiameterMm);
            config.intake.runnerPlenumDiameterMm = intake.value("runner_plenum_diameter_mm", config.intake.runnerPlenumDiameterMm);
            config.intake.airboxVolumeLitres = intake.value("airbox_volume_l", config.intake.airboxVolumeLitres);
            config.intake.inletDuctLengthMm = intake.value("inlet_duct_length_mm", config.intake.inletDuctLengthMm);
            config.intake.inletDuctDiameterMm = intake.value("inlet_duct_diameter_mm", config.intake.inletDuctDiameterMm);
            config.intake.bellmouthDiameterMm = intake.value("bellmouth_diameter_mm", config.intake.bellmouthDiameterMm);
            config.intake.idleBypassAreaMm2 = intake.value("idle_bypass_area_mm2", config.intake.idleBypassAreaMm2);
            config.intake.throttleGamma = intake.value("throttle_gamma", config.intake.throttleGamma);
        }
        if (engine.contains("crankshafts")) {
            for (const auto& item : engine.at("crankshafts")) {
                CrankshaftConfig crankshaft;
                crankshaft.id = item.at("id");
                crankshaft.positionXMm = item.value("position_x_mm", 0.0);
                crankshaft.positionYMm = item.value("position_y_mm", 0.0);
                crankshaft.phaseOffsetDegrees = item.value("phase_offset_deg", 0.0);
                crankshaft.rotationRatio = item.value("rotation_ratio", 1.0);
                crankshaft.massKg = item.value("mass_kg", crankshaft.massKg);
                crankshaft.flywheelMassKg = item.value("flywheel_mass_kg", crankshaft.flywheelMassKg);
                crankshaft.momentOfInertiaKgM2 = item.value("moment_of_inertia_kg_m2", config.rotatingInertiaKgM2);
                crankshaft.frictionTorqueNm = item.value("friction_torque_nm", 0.0);
                config.crankshafts.push_back(crankshaft);
            }
        }
        if (engine.contains("crank_journals")) {
            for (const auto& journal : engine.at("crank_journals"))
                config.crankJournals.push_back({ journal.at("id").get<std::uint32_t>(),
                    journal.at("angle_deg").get<double>(), journal.at("throw_mm").get<double>(),
                    journal.value("crankshaft_id", std::uint32_t { 1 }) });
        }
        if (engine.contains("exhaust")) {
            const auto& exhaust = engine.at("exhaust");
            config.exhaust = { exhaust.value("primary_length_mm", 480.0), exhaust.value("primary_diameter_mm", 42.0),
                exhaust.value("collector_diameter_mm", 58.0), exhaust.value("muffler_restriction", 0.28),
                exhaust.value("outlet_diameter_mm", 65.0), exhaust.value("collector_volume_l", 2.0),
                exhaust.value("outlet_discharge_coefficient", 0.72),
                exhaust.value("muffler_chamber_diameter_mm", 0.0),
                exhaust.value("muffler_chamber_length_mm", 0.0),
                exhaust.value("muffler_packing_flow_resistivity_pa_s_m2", 0.0),
                exhaust.value("muffler_packing_thickness_mm", 0.0),
                exhaust.value("muffler_perforated_open_area_ratio", 0.0) };
        }
        if (engine.contains("transmission")) {
            const auto& transmission = engine.at("transmission");
            config.transmission.gearRatios = transmission.value("gear_ratios", config.transmission.gearRatios);
            config.transmission.finalDriveRatio = transmission.value("final_drive_ratio", config.transmission.finalDriveRatio);
            config.transmission.maxClutchTorqueNm = transmission.value("max_clutch_torque_nm", config.transmission.maxClutchTorqueNm);
            config.transmission.drivelineEfficiency = transmission.value("driveline_efficiency", config.transmission.drivelineEfficiency);
            config.transmission.drivenWheelInertiaKgM2 = transmission.value("driven_wheel_inertia_kg_m2", config.transmission.drivenWheelInertiaKgM2);
            config.transmission.clutchLockSpeedRpm = transmission.value("clutch_lock_speed_rpm", config.transmission.clutchLockSpeedRpm);
            config.transmission.shiftDurationSeconds = transmission.value("shift_duration_s", config.transmission.shiftDurationSeconds);
            config.transmission.automaticShifting = transmission.value("automatic_shifting", config.transmission.automaticShifting);
            config.transmission.automaticUpshiftRpm = transmission.value("automatic_upshift_rpm", config.transmission.automaticUpshiftRpm);
            config.transmission.automaticDownshiftRpm = transmission.value("automatic_downshift_rpm", config.transmission.automaticDownshiftRpm);
            config.transmission.reverseRatio = transmission.value("reverse_ratio", config.transmission.reverseRatio);
            config.transmission.gearboxInputInertiaKgM2 = transmission.value("gearbox_input_inertia_kg_m2", config.transmission.gearboxInputInertiaKgM2);
            config.transmission.differentialInertiaKgM2 = transmission.value("differential_inertia_kg_m2", config.transmission.differentialInertiaKgM2);
            config.transmission.clutchThermalCapacityJPerC = transmission.value("clutch_thermal_capacity_j_per_c", config.transmission.clutchThermalCapacityJPerC);
            config.transmission.clutchCoolingWPerC = transmission.value("clutch_cooling_w_per_c", config.transmission.clutchCoolingWPerC);
            config.transmission.clutchFadeStartTemperatureC = transmission.value("clutch_fade_start_temperature_c", config.transmission.clutchFadeStartTemperatureC);
            config.transmission.clutchFailureTemperatureC = transmission.value("clutch_failure_temperature_c", config.transmission.clutchFailureTemperatureC);
            config.transmission.shiftTorqueCutFraction = transmission.value("shift_torque_cut_fraction", config.transmission.shiftTorqueCutFraction);
        }
        if (engine.contains("vehicle")) {
            const auto& vehicle = engine.at("vehicle");
            config.vehicle.massKg = vehicle.value("mass_kg", config.vehicle.massKg);
            config.vehicle.dragCoefficient = vehicle.value("drag_coefficient", config.vehicle.dragCoefficient);
            config.vehicle.frontalAreaM2 = vehicle.value("frontal_area_m2", config.vehicle.frontalAreaM2);
            config.vehicle.tireRadiusM = vehicle.value("tire_radius_m", config.vehicle.tireRadiusM);
            config.vehicle.rollingResistanceCoefficient = vehicle.value("rolling_resistance_coefficient",
                                                                        config.vehicle.rollingResistanceCoefficient);
            config.vehicle.tireFrictionCoefficient = vehicle.value("tire_friction_coefficient", config.vehicle.tireFrictionCoefficient);
            config.vehicle.tyreGripLimitEnabled = vehicle.value("tyre_grip_limit_enabled", config.vehicle.tyreGripLimitEnabled);
            config.vehicle.drivenAxleLayout = decodeDrivenAxleLayout(
                vehicle.value("driven_axle_layout",
                    std::string { drivenAxleLayoutName(
                        config.vehicle.drivenAxleLayout) }));
            config.vehicle.drivenAxleWeightFraction = vehicle.value("driven_axle_weight_fraction", config.vehicle.drivenAxleWeightFraction);
            config.vehicle.wheelbaseM =
                vehicle.value("wheelbase_m", config.vehicle.wheelbaseM);
            config.vehicle.centerOfGravityHeightM =
                vehicle.value("center_of_gravity_height_m",
                    config.vehicle.centerOfGravityHeightM);
            config.vehicle.maximumBrakeForceN = vehicle.value("maximum_brake_force_n", config.vehicle.maximumBrakeForceN);
        }
        if (engine.contains("ignition")) {
            const auto& ignition = engine.at("ignition");
            config.ignition.revLimitRpm = ignition.value("rev_limit_rpm", config.redlineRpm);
            config.ignition.limiterDurationSeconds = ignition.value("limiter_duration_s", config.ignition.limiterDurationSeconds);
            config.ignition.limiterKeepsFuel = ignition.value("limiter_keeps_fuel", config.ignition.limiterKeepsFuel);
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
            config.injection.directSprayVaporisationTimeConstantSeconds = injection.value("direct_spray_vaporisation_time_constant_s", config.injection.directSprayVaporisationTimeConstantSeconds);
            config.injection.directSprayEntrainmentTimeConstantSeconds = injection.value("direct_spray_entrainment_time_constant_s", config.injection.directSprayEntrainmentTimeConstantSeconds);
            if (injection.contains("full_load_fuel_limit")) {
                config.injection.fullLoadFuelLimit.clear();
                for (const auto& sample : injection.at("full_load_fuel_limit"))
                    config.injection.fullLoadFuelLimit.push_back({
                        sample.at("rpm").get<double>(),
                        sample.at("mg_per_cycle").get<double>() });
            }
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
        if (engine.contains("intake_paths")) {
            for (const auto& item : engine.at("intake_paths")) {
                IntakePathConfig path;
                path.id = item.at("id");
                path.cylinderIds = item.value("cylinder_ids", std::vector<std::uint32_t> {});
                if (item.contains("geometry")) {
                    const auto& geometry = item.at("geometry");
                    path.geometry.plenumVolumeLitres = geometry.value("plenum_volume_l", path.geometry.plenumVolumeLitres);
                    path.geometry.throttleDiameterMm = geometry.value("throttle_diameter_mm", path.geometry.throttleDiameterMm);
                    path.geometry.throttleCount = geometry.value("throttle_count", path.geometry.throttleCount);
                    path.geometry.throttleDischargeCoefficient = geometry.value("throttle_discharge_coefficient", path.geometry.throttleDischargeCoefficient);
                    path.geometry.runnerLengthMm = geometry.value("runner_length_mm", path.geometry.runnerLengthMm);
                    path.geometry.runnerDiameterMm = geometry.value("runner_diameter_mm", path.geometry.runnerDiameterMm);
                    path.geometry.runnerPlenumDiameterMm = geometry.value("runner_plenum_diameter_mm", path.geometry.runnerPlenumDiameterMm);
                    path.geometry.airboxVolumeLitres = geometry.value("airbox_volume_l", path.geometry.airboxVolumeLitres);
                    path.geometry.inletDuctLengthMm = geometry.value("inlet_duct_length_mm", path.geometry.inletDuctLengthMm);
                    path.geometry.inletDuctDiameterMm = geometry.value("inlet_duct_diameter_mm", path.geometry.inletDuctDiameterMm);
                    path.geometry.bellmouthDiameterMm = geometry.value("bellmouth_diameter_mm", path.geometry.bellmouthDiameterMm);
                    path.geometry.idleBypassAreaMm2 = geometry.value("idle_bypass_area_mm2", path.geometry.idleBypassAreaMm2);
                    path.geometry.throttleGamma = geometry.value("throttle_gamma", path.geometry.throttleGamma);
                }
                config.intakePaths.push_back(std::move(path));
            }
        }
        if (engine.contains("acoustic_observer")) {
            const auto& observer = engine.at("acoustic_observer");
            if (observer.contains("left_microphone_m"))
                config.acousticObserver.leftMicrophoneM = decodePoint(
                    observer.at("left_microphone_m"),
                    config.acousticObserver.leftMicrophoneM);
            if (observer.contains("right_microphone_m"))
                config.acousticObserver.rightMicrophoneM = decodePoint(
                    observer.at("right_microphone_m"),
                    config.acousticObserver.rightMicrophoneM);
            config.acousticObserver.soundSpeedMps = observer.value(
                "sound_speed_mps", config.acousticObserver.soundSpeedMps);
        }
        if (engine.contains("exhaust_paths")) {
            for (const auto& item : engine.at("exhaust_paths")) {
                ExhaustPathConfig path;
                path.id = item.at("id");
                path.cylinderIds = item.value("cylinder_ids", std::vector<std::uint32_t> {});
                path.impulseResponsePath = item.value("impulse_response", std::string {});
                path.audioVolume = item.value("audio_volume", 1.0);
                if (item.contains("acoustic_position_m"))
                    path.acousticPositionM = decodePoint(item.at("acoustic_position_m"));
                if (item.contains("acoustic_axis"))
                    path.acousticAxis = decodePoint(
                        item.at("acoustic_axis"), path.acousticAxis);
                path.acousticTermination = decodeTermination(item.value(
                    "acoustic_termination", std::string("unflanged")));
                if (item.contains("geometry")) {
                    const auto& geometry = item.at("geometry");
                    path.geometry.primaryLengthMm = geometry.value("primary_length_mm", path.geometry.primaryLengthMm);
                    path.geometry.primaryDiameterMm = geometry.value("primary_diameter_mm", path.geometry.primaryDiameterMm);
                    path.geometry.collectorDiameterMm = geometry.value("collector_diameter_mm", path.geometry.collectorDiameterMm);
                    path.geometry.mufflerRestriction = geometry.value("muffler_restriction", path.geometry.mufflerRestriction);
                    path.geometry.outletDiameterMm = geometry.value("outlet_diameter_mm", path.geometry.outletDiameterMm);
                    path.geometry.collectorVolumeLitres = geometry.value("collector_volume_l", path.geometry.collectorVolumeLitres);
                    path.geometry.outletDischargeCoefficient = geometry.value("outlet_discharge_coefficient", path.geometry.outletDischargeCoefficient);
                    path.geometry.mufflerChamberDiameterMm = geometry.value("muffler_chamber_diameter_mm", path.geometry.mufflerChamberDiameterMm);
                    path.geometry.mufflerChamberLengthMm = geometry.value("muffler_chamber_length_mm", path.geometry.mufflerChamberLengthMm);
                    path.geometry.mufflerPackingFlowResistivityPaSPerM2 = geometry.value(
                        "muffler_packing_flow_resistivity_pa_s_m2", path.geometry.mufflerPackingFlowResistivityPaSPerM2);
                    path.geometry.mufflerPackingThicknessMm = geometry.value(
                        "muffler_packing_thickness_mm", path.geometry.mufflerPackingThicknessMm);
                    path.geometry.mufflerPerforatedOpenAreaRatio = geometry.value(
                        "muffler_perforated_open_area_ratio", path.geometry.mufflerPerforatedOpenAreaRatio);
                }
                if (item.contains("graph") && !item.at("graph").is_null()) {
                    const auto& encodedGraph = item.at("graph");
                    ExhaustNetworkConfig network;
                    for (const auto& encodedComponent : encodedGraph.at("components")) {
                        const auto typeName = encodedComponent.at("type").get<std::string>();
                        const auto type = decodeExhaustComponentType(typeName);
                        if (!type) return { std::nullopt, "Unknown exhaust component type: " + typeName };
                        ExhaustComponentConfig component;
                        component.id = encodedComponent.at("id");
                        component.type = *type;
                        component.lengthMm = encodedComponent.value("length_mm", component.lengthMm);
                        component.diameterMm = encodedComponent.value("diameter_mm", component.diameterMm);
                        component.outletDiameterMm = encodedComponent.value(
                            "outlet_diameter_mm", component.outletDiameterMm);
                        component.volumeLitres = encodedComponent.value("volume_l", component.volumeLitres);
                        component.restriction = encodedComponent.value("restriction", component.restriction);
                        component.resonanceHz = encodedComponent.value("resonance_hz", component.resonanceHz);
                        component.acousticGain = encodedComponent.value("acoustic_gain", component.acousticGain);
                        component.dischargeCoefficient = encodedComponent.value(
                            "discharge_coefficient", component.dischargeCoefficient);
                        component.packingFlowResistivityPaSPerM2 = encodedComponent.value(
                            "packing_flow_resistivity_pa_s_m2",
                            component.packingFlowResistivityPaSPerM2);
                        component.packingThicknessMm = encodedComponent.value(
                            "packing_thickness_mm", component.packingThicknessMm);
                        component.perforatedOpenAreaRatio = encodedComponent.value(
                            "perforated_open_area_ratio",
                            component.perforatedOpenAreaRatio);
                        if (encodedComponent.contains("acoustic_position_m"))
                            component.acousticPositionM = decodePoint(
                                encodedComponent.at("acoustic_position_m"));
                        if (encodedComponent.contains("acoustic_axis"))
                            component.acousticAxis = decodePoint(
                                encodedComponent.at("acoustic_axis"),
                                component.acousticAxis);
                        component.acousticTermination = decodeTermination(
                            encodedComponent.value("acoustic_termination",
                                std::string("unflanged")));
                        network.components.push_back(component);
                    }
                    for (const auto& encodedConnection : encodedGraph.at("cylinder_connections"))
                        network.cylinderConnections.push_back({ encodedConnection.at("cylinder_id"),
                            encodedConnection.at("to_component_id") });
                    for (const auto& encodedConnection : encodedGraph.at("connections"))
                        network.connections.push_back({ encodedConnection.at("from_component_id"),
                            encodedConnection.at("to_component_id") });
                    path.network = std::move(network);
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
            const auto rodType = item.value("connecting_rod_type", std::string { "conventional" });
            if (rodType == "conventional") cylinder.connectingRodType = ConnectingRodType::conventional;
            else if (rodType == "master") cylinder.connectingRodType = ConnectingRodType::master;
            else if (rodType == "articulated") cylinder.connectingRodType = ConnectingRodType::articulated;
            else return { std::nullopt, "Unknown connecting-rod type: " + rodType };
            cylinder.masterCylinderId = item.value("master_cylinder_id", std::uint32_t { 0 });
            cylinder.articulatedJournalRadiusMm = item.value("articulated_journal_radius_mm", 0.0);
            cylinder.articulatedJournalAngleDegrees = item.value("articulated_journal_angle_deg", 0.0);
            cylinder.deckHeightMm = item.value("deck_height_mm", 0.0);
            cylinder.compressionHeightMm = item.value("compression_height_mm", 0.0);
            cylinder.wristPinOffsetMm = item.value("wrist_pin_offset_mm", 0.0);
            cylinder.pistonCrownVolumeCc = item.value("piston_crown_volume_cc", 0.0);
            cylinder.headChamberVolumeCc = item.value("head_chamber_volume_cc", 0.0);
            cylinder.headGasketThicknessMm = item.value("head_gasket_thickness_mm", 0.0);
            cylinder.connectingRodMomentOfInertiaKgM2 = item.value("connecting_rod_inertia_kg_m2", 0.0);
            cylinder.intakeValveCount = item.value("intake_valve_count", cylinder.intakeValveCount);
            cylinder.exhaustValveCount = item.value("exhaust_valve_count", cylinder.exhaustValveCount);
            cylinder.intakeValveDiameterMm = item.value("intake_valve_diameter_mm", 0.0);
            cylinder.exhaustValveDiameterMm = item.value("exhaust_valve_diameter_mm", 0.0);
            config.cylinders.push_back(cylinder);
        }
        normaliseEngineConfig(config);
        if (const auto error = validateEngineConfig(config)) return { std::nullopt, *error };
        return { std::move(config), {} };
    } catch (const std::exception& error) { return { std::nullopt, error.what() }; }
}
} // namespace enginelab
