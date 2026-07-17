#include <enginelab/catalog/EngineCatalog.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <fstream>
#include <map>
#include <optional>
#include <sstream>

namespace enginelab {
namespace {
template <typename T>
void assignIfPresent(const YAML::Node& node, const char* key, T& value) {
    if (node && node[key]) value = node[key].as<T>();
}

[[nodiscard]] EngineLayout parseLayout(const std::string& value) {
    if (value == "inline") return EngineLayout::inlineLayout;
    if (value == "v") return EngineLayout::vLayout;
    if (value == "flat") return EngineLayout::flat;
    if (value == "radial") return EngineLayout::radial;
    if (value == "custom") return EngineLayout::custom;
    throw std::runtime_error("Unknown engine layout: " + value);
}

template <typename T>
[[nodiscard]] std::map<std::string, T> loadPartMap(const std::filesystem::path& file,
                                                   T (*decode)(const YAML::Node&)) {
    std::map<std::string, T> parts;
    if (!std::filesystem::exists(file)) return parts;
    const auto document = YAML::LoadFile(file.string());
    for (const auto& item : document) parts.emplace(item.first.as<std::string>(), decode(item.second));
    return parts;
}

[[nodiscard]] CamshaftConfig decodeCamshafts(const YAML::Node& node) {
    CamshaftConfig value;
    assignIfPresent(node, "intake_duration_deg", value.intakeDurationDegrees);
    assignIfPresent(node, "exhaust_duration_deg", value.exhaustDurationDegrees);
    assignIfPresent(node, "intake_lift_mm", value.intakeLiftMm);
    assignIfPresent(node, "exhaust_lift_mm", value.exhaustLiftMm);
    assignIfPresent(node, "intake_centerline_deg", value.intakeCenterlineDegrees);
    assignIfPresent(node, "exhaust_centerline_deg", value.exhaustCenterlineDegrees);
    assignIfPresent(node, "intake_flow_coefficient", value.intakeFlowCoefficient);
    assignIfPresent(node, "exhaust_flow_coefficient", value.exhaustFlowCoefficient);
    assignIfPresent(node, "variable_profile_enabled", value.variableProfileEnabled);
    assignIfPresent(node, "switch_rpm", value.switchRpm);
    assignIfPresent(node, "switch_throttle", value.switchThrottle);
    assignIfPresent(node, "high_intake_duration_deg", value.highIntakeDurationDegrees);
    assignIfPresent(node, "high_exhaust_duration_deg", value.highExhaustDurationDegrees);
    assignIfPresent(node, "high_intake_lift_mm", value.highIntakeLiftMm);
    assignIfPresent(node, "high_exhaust_lift_mm", value.highExhaustLiftMm);
    if (node["intake_lift_profile"]) {
        for (const auto& sample : node["intake_lift_profile"])
            value.intakeLiftProfile.push_back({ sample["angle_deg"].as<double>(), sample["lift_mm"].as<double>() });
    }
    if (node["exhaust_lift_profile"]) {
        for (const auto& sample : node["exhaust_lift_profile"])
            value.exhaustLiftProfile.push_back({ sample["angle_deg"].as<double>(), sample["lift_mm"].as<double>() });
    }
    if (node["high_intake_lift_profile"]) for (const auto& sample : node["high_intake_lift_profile"])
        value.highIntakeLiftProfile.push_back({ sample["angle_deg"].as<double>(), sample["lift_mm"].as<double>() });
    if (node["high_exhaust_lift_profile"]) for (const auto& sample : node["high_exhaust_lift_profile"])
        value.highExhaustLiftProfile.push_back({ sample["angle_deg"].as<double>(), sample["lift_mm"].as<double>() });
    if (node["intake_flow_curve"]) for (const auto& sample : node["intake_flow_curve"])
        value.intakeFlowCurve.push_back({ sample["lift_mm"].as<double>(), sample["discharge_coefficient"].as<double>() });
    if (node["exhaust_flow_curve"]) for (const auto& sample : node["exhaust_flow_curve"])
        value.exhaustFlowCurve.push_back({ sample["lift_mm"].as<double>(), sample["discharge_coefficient"].as<double>() });
    if (const auto control = node["continuous_control"]) {
        assignIfPresent(control, "enabled", value.continuousControl.enabled);
        assignIfPresent(control, "response_frequency_hz", value.continuousControl.responseFrequencyHz);
        if (control["samples"]) for (const auto& sample : control["samples"])
            value.continuousControl.samples.push_back({ sample["rpm"].as<double>(), sample["load"].as<double>(),
                sample["intake_advance_deg"].as<double>(), sample["exhaust_advance_deg"].as<double>(),
                sample["lift_multiplier"].as<double>() });
    }
    return value;
}

[[nodiscard]] ExhaustConfig decodeExhaust(const YAML::Node& node) {
    ExhaustConfig value;
    assignIfPresent(node, "primary_length_mm", value.primaryLengthMm);
    assignIfPresent(node, "primary_diameter_mm", value.primaryDiameterMm);
    assignIfPresent(node, "collector_diameter_mm", value.collectorDiameterMm);
    assignIfPresent(node, "muffler_restriction", value.mufflerRestriction);
    assignIfPresent(node, "outlet_diameter_mm", value.outletDiameterMm);
    assignIfPresent(node, "collector_volume_l", value.collectorVolumeLitres);
    assignIfPresent(node, "outlet_discharge_coefficient", value.outletDischargeCoefficient);
    return value;
}

[[nodiscard]] ExhaustComponentType parseExhaustComponentType(const std::string& value) {
    if (value == "pipe") return ExhaustComponentType::pipe;
    if (value == "merge") return ExhaustComponentType::merge;
    if (value == "splitter") return ExhaustComponentType::splitter;
    if (value == "resonator") return ExhaustComponentType::resonator;
    if (value == "muffler") return ExhaustComponentType::muffler;
    if (value == "catalyst") return ExhaustComponentType::catalyst;
    if (value == "outlet") return ExhaustComponentType::outlet;
    throw std::runtime_error("Unknown exhaust component type: " + value);
}

[[nodiscard]] ExhaustNetworkConfig decodeExhaustNetwork(const YAML::Node& node) {
    ExhaustNetworkConfig network;
    if (const auto components = node["components"]) {
        for (const auto& encoded : components) {
            ExhaustComponentConfig component;
            component.id = encoded["id"].as<std::uint32_t>();
            component.type = parseExhaustComponentType(encoded["type"].as<std::string>());
            assignIfPresent(encoded, "length_mm", component.lengthMm);
            assignIfPresent(encoded, "diameter_mm", component.diameterMm);
            assignIfPresent(encoded, "volume_l", component.volumeLitres);
            assignIfPresent(encoded, "restriction", component.restriction);
            assignIfPresent(encoded, "resonance_hz", component.resonanceHz);
            assignIfPresent(encoded, "acoustic_gain", component.acousticGain);
            assignIfPresent(encoded, "discharge_coefficient", component.dischargeCoefficient);
            network.components.push_back(component);
        }
    }
    if (const auto cylinderConnections = node["cylinder_connections"]) {
        for (const auto& encoded : cylinderConnections)
            network.cylinderConnections.push_back({ encoded["cylinder_id"].as<std::uint32_t>(),
                encoded["to_component_id"].as<std::uint32_t>() });
    }
    if (const auto connections = node["connections"]) {
        for (const auto& encoded : connections)
            network.connections.push_back({ encoded["from_component_id"].as<std::uint32_t>(),
                encoded["to_component_id"].as<std::uint32_t>() });
    }
    return network;
}

[[nodiscard]] TransmissionConfig decodeTransmission(const YAML::Node& node) {
    TransmissionConfig value;
    assignIfPresent(node, "gear_ratios", value.gearRatios);
    assignIfPresent(node, "final_drive_ratio", value.finalDriveRatio);
    assignIfPresent(node, "max_clutch_torque_nm", value.maxClutchTorqueNm);
    assignIfPresent(node, "driveline_efficiency", value.drivelineEfficiency);
    assignIfPresent(node, "driven_wheel_inertia_kg_m2", value.drivenWheelInertiaKgM2);
    assignIfPresent(node, "clutch_slip_stiffness_nm_per_rpm", value.clutchSlipStiffnessNmPerRpm);
    assignIfPresent(node, "clutch_lock_speed_rpm", value.clutchLockSpeedRpm);
    assignIfPresent(node, "shift_duration_s", value.shiftDurationSeconds);
    assignIfPresent(node, "automatic_shifting", value.automaticShifting);
    assignIfPresent(node, "automatic_upshift_rpm", value.automaticUpshiftRpm);
    assignIfPresent(node, "automatic_downshift_rpm", value.automaticDownshiftRpm);
    assignIfPresent(node, "reverse_ratio", value.reverseRatio);
    assignIfPresent(node, "gearbox_input_inertia_kg_m2", value.gearboxInputInertiaKgM2);
    assignIfPresent(node, "differential_inertia_kg_m2", value.differentialInertiaKgM2);
    assignIfPresent(node, "clutch_thermal_capacity_j_per_c", value.clutchThermalCapacityJPerC);
    assignIfPresent(node, "clutch_cooling_w_per_c", value.clutchCoolingWPerC);
    assignIfPresent(node, "clutch_fade_start_temperature_c", value.clutchFadeStartTemperatureC);
    assignIfPresent(node, "clutch_failure_temperature_c", value.clutchFailureTemperatureC);
    assignIfPresent(node, "shift_torque_cut_fraction", value.shiftTorqueCutFraction);
    return value;
}

[[nodiscard]] VehicleConfig decodeVehicle(const YAML::Node& node) {
    VehicleConfig value;
    assignIfPresent(node, "mass_kg", value.massKg);
    assignIfPresent(node, "drag_coefficient", value.dragCoefficient);
    assignIfPresent(node, "frontal_area_m2", value.frontalAreaM2);
    assignIfPresent(node, "tire_radius_m", value.tireRadiusM);
    assignIfPresent(node, "rolling_resistance_coefficient", value.rollingResistanceCoefficient);
    assignIfPresent(node, "tire_friction_coefficient", value.tireFrictionCoefficient);
    assignIfPresent(node, "driven_axle_weight_fraction", value.drivenAxleWeightFraction);
    assignIfPresent(node, "maximum_brake_force_n", value.maximumBrakeForceN);
    return value;
}

[[nodiscard]] ForcedInductionConfig decodeForcedInduction(const YAML::Node& node) {
    ForcedInductionConfig value;
    assignIfPresent(node, "enabled", value.enabled);
    const auto type = node["type"].as<std::string>("turbocharger");
    if (type == "supercharger") value.type = ForcedInductionType::supercharger;
    else if (type != "turbocharger") throw std::runtime_error("Unknown forced-induction type: " + type);
    assignIfPresent(node, "pressure_ratio", value.pressureRatio);
    assignIfPresent(node, "full_boost_rpm", value.fullBoostRpm);
    assignIfPresent(node, "compressor_efficiency", value.compressorEfficiency);
    assignIfPresent(node, "charge_temperature_rise_c", value.chargeTemperatureRiseC);
    assignIfPresent(node, "turbine_efficiency", value.turbineEfficiency);
    assignIfPresent(node, "shaft_inertia_kg_m2", value.shaftInertiaKgM2);
    assignIfPresent(node, "wastegate_pressure_ratio", value.wastegatePressureRatio);
    assignIfPresent(node, "design_shaft_speed_rpm", value.designShaftSpeedRpm);
    assignIfPresent(node, "bearing_friction_power_w", value.bearingFrictionPowerWatts);
    assignIfPresent(node, "turbine_flow_area_mm2", value.turbineFlowAreaMm2);
    assignIfPresent(node, "wastegate_flow_area_mm2", value.wastegateFlowAreaMm2);
    return value;
}

[[nodiscard]] ThermalConfig decodeThermal(const YAML::Node& node) {
    ThermalConfig value;
    assignIfPresent(node, "coolant_mass_kj_per_c", value.coolantMassKjPerC);
    assignIfPresent(node, "oil_mass_kj_per_c", value.oilMassKjPerC);
    assignIfPresent(node, "coolant_heat_share", value.coolantHeatShare);
    assignIfPresent(node, "oil_heat_share", value.oilHeatShare);
    assignIfPresent(node, "cooling_power_kw_per_c", value.coolingPowerKwPerC);
    assignIfPresent(node, "oil_cooling_power_kw_per_c", value.oilCoolingPowerKwPerC);
    return value;
}

[[nodiscard]] FuelConfig decodeFuel(const YAML::Node& node) {
    FuelConfig value;
    assignIfPresent(node, "name", value.name);
    assignIfPresent(node, "lower_heating_value_mj_per_kg", value.lowerHeatingValueMjPerKg);
    assignIfPresent(node, "density_kg_per_l", value.densityKgPerL);
    assignIfPresent(node, "stoichiometric_afr", value.stoichiometricAirFuelRatio);
    assignIfPresent(node, "molar_mass_g_per_mol", value.molarMassGramsPerMole);
    assignIfPresent(node, "oxygen_moles_per_fuel_mole", value.oxygenMolesPerFuelMole);
    assignIfPresent(node, "product_moles_per_fuel_mole", value.productMolesPerFuelMole);
    assignIfPresent(node, "laminar_flame_speed_mps", value.laminarFlameSpeedMps);
    assignIfPresent(node, "turbulence_flame_speed_gain", value.turbulenceFlameSpeedGain);
    return value;
}

[[nodiscard]] InjectionConfig decodeInjection(const YAML::Node& node) {
    InjectionConfig value;
    const auto mode = node["mode"].as<std::string>("direct");
    if (mode == "port") {
        value.mode = InjectionMode::port;
        value.startAngleDegrees = 250.0;
        value.endAngleDegrees = 620.0;
        value.injectorFlowMgPerSecond = 5'000.0;
        value.railPressureBar = 4.0;
        value.referencePressureBar = 4.0;
        value.wallFilmFraction = 0.22;
        value.vaporisationTimeConstantSeconds = 0.040;
    }
    else if (mode == "direct") value.mode = InjectionMode::direct;
    else throw std::runtime_error("Unknown injection mode: " + mode);
    assignIfPresent(node, "start_angle_deg", value.startAngleDegrees);
    assignIfPresent(node, "end_angle_deg", value.endAngleDegrees);
    assignIfPresent(node, "injector_flow_mg_s", value.injectorFlowMgPerSecond);
    assignIfPresent(node, "fuel_temperature_c", value.fuelTemperatureC);
    assignIfPresent(node, "rail_pressure_bar", value.railPressureBar);
    assignIfPresent(node, "reference_pressure_bar", value.referencePressureBar);
    assignIfPresent(node, "wall_film_fraction", value.wallFilmFraction);
    assignIfPresent(node, "vaporisation_time_constant_s", value.vaporisationTimeConstantSeconds);
    assignIfPresent(node, "latent_heat_kj_per_kg", value.latentHeatKjPerKg);
    assignIfPresent(node, "direct_charge_cooling_efficiency", value.directChargeCoolingEfficiency);
    assignIfPresent(node, "port_charge_cooling_efficiency", value.portChargeCoolingEfficiency);
    return value;
}

struct PartsLibrary final {
    std::map<std::string, FuelConfig> fuels;
    std::map<std::string, InjectionConfig> injections;
    std::map<std::string, CamshaftConfig> camshafts;
    std::map<std::string, ExhaustConfig> exhausts;
    std::map<std::string, TransmissionConfig> transmissions;
    std::map<std::string, VehicleConfig> vehicles;
};

template <typename T>
void applyPart(const std::map<std::string, T>& parts, const YAML::Node& uses, const char* key, T& target) {
    if (!uses || !uses[key]) return;
    const auto name = uses[key].as<std::string>();
    const auto found = parts.find(name);
    if (found == parts.end()) throw std::runtime_error(std::string("Unknown part reference: ") + key + "." + name);
    target = found->second;
}

[[nodiscard]] std::vector<CylinderConfig> decodeCylinders(const YAML::Node& engine) {
    if (engine["cylinders"]) {
        std::vector<CylinderConfig> cylinders;
        for (const auto& item : engine["cylinders"]) {
            CylinderConfig cylinder;
            cylinder.id = item["id"].as<std::uint32_t>();
            cylinder.boreMm = item["bore_mm"].as<double>();
            cylinder.strokeMm = item["stroke_mm"].as<double>();
            cylinder.connectingRodMm = item["connecting_rod_mm"].as<double>();
            cylinder.pistonMassGrams = item["piston_mass_g"].as<double>();
            assignIfPresent(item, "connecting_rod_mass_g", cylinder.connectingRodMassGrams);
            cylinder.compressionRatio = item["compression_ratio"].as<double>();
            assignIfPresent(item, "ignition_offset_deg", cylinder.ignitionOffsetDegrees);
            assignIfPresent(item, "efficiency_offset", cylinder.efficiencyOffset);
            assignIfPresent(item, "crank_offset_deg", cylinder.crankOffsetDegrees);
            assignIfPresent(item, "crank_journal_id", cylinder.crankJournalId);
            assignIfPresent(item, "bank_offset_deg", cylinder.bankOffsetDegrees);
            assignIfPresent(item, "bank_id", cylinder.bankId);
            assignIfPresent(item, "intake_runner_length_mm", cylinder.intakeRunnerLengthMm);
            assignIfPresent(item, "intake_runner_diameter_mm", cylinder.intakeRunnerDiameterMm);
            assignIfPresent(item, "exhaust_primary_length_mm", cylinder.exhaustPrimaryLengthMm);
            assignIfPresent(item, "sound_attenuation", cylinder.soundAttenuation);
            assignIfPresent(item, "blow_by_coefficient", cylinder.blowByCoefficient);
            assignIfPresent(item, "piston_friction_coefficient", cylinder.pistonFrictionCoefficient);
            assignIfPresent(item, "piston_breakaway_force_n", cylinder.pistonBreakawayForceN);
            assignIfPresent(item, "piston_breakaway_velocity_mps", cylinder.pistonBreakawayVelocityMps);
            assignIfPresent(item, "piston_viscous_friction_ns_per_m", cylinder.pistonViscousFrictionNsPerM);
            const auto rodType = item["connecting_rod_type"].as<std::string>("conventional");
            if (rodType == "master") cylinder.connectingRodType = ConnectingRodType::master;
            else if (rodType == "articulated") cylinder.connectingRodType = ConnectingRodType::articulated;
            else if (rodType != "conventional") throw std::runtime_error("Unknown connecting-rod type: " + rodType);
            assignIfPresent(item, "master_cylinder_id", cylinder.masterCylinderId);
            assignIfPresent(item, "articulated_journal_radius_mm", cylinder.articulatedJournalRadiusMm);
            assignIfPresent(item, "articulated_journal_angle_deg", cylinder.articulatedJournalAngleDegrees);
            assignIfPresent(item, "deck_height_mm", cylinder.deckHeightMm);
            assignIfPresent(item, "compression_height_mm", cylinder.compressionHeightMm);
            assignIfPresent(item, "wrist_pin_offset_mm", cylinder.wristPinOffsetMm);
            assignIfPresent(item, "piston_crown_volume_cc", cylinder.pistonCrownVolumeCc);
            assignIfPresent(item, "head_chamber_volume_cc", cylinder.headChamberVolumeCc);
            assignIfPresent(item, "head_gasket_thickness_mm", cylinder.headGasketThicknessMm);
            assignIfPresent(item, "connecting_rod_inertia_kg_m2", cylinder.connectingRodMomentOfInertiaKgM2);
            assignIfPresent(item, "intake_valve_count", cylinder.intakeValveCount);
            assignIfPresent(item, "exhaust_valve_count", cylinder.exhaustValveCount);
            assignIfPresent(item, "intake_valve_diameter_mm", cylinder.intakeValveDiameterMm);
            assignIfPresent(item, "exhaust_valve_diameter_mm", cylinder.exhaustValveDiameterMm);
            cylinders.push_back(cylinder);
        }
        return cylinders;
    }

    const auto count = engine["cylinder_count"].as<std::uint32_t>();
    const auto cylinderNode = engine["cylinder_template"];
    std::vector<CylinderConfig> cylinders;
    cylinders.reserve(count);
    for (std::uint32_t index = 1; index <= count; ++index) {
        CylinderConfig cylinder;
        cylinder.id = index;
        cylinder.boreMm = cylinderNode["bore_mm"].as<double>();
        cylinder.strokeMm = cylinderNode["stroke_mm"].as<double>();
        cylinder.connectingRodMm = cylinderNode["connecting_rod_mm"].as<double>();
        cylinder.pistonMassGrams = cylinderNode["piston_mass_g"].as<double>();
        assignIfPresent(cylinderNode, "connecting_rod_mass_g", cylinder.connectingRodMassGrams);
        cylinder.compressionRatio = cylinderNode["compression_ratio"].as<double>();
        assignIfPresent(cylinderNode, "ignition_offset_deg", cylinder.ignitionOffsetDegrees);
        assignIfPresent(cylinderNode, "efficiency_offset", cylinder.efficiencyOffset);
        assignIfPresent(cylinderNode, "crank_journal_id", cylinder.crankJournalId);
        assignIfPresent(cylinderNode, "bank_offset_deg", cylinder.bankOffsetDegrees);
        assignIfPresent(cylinderNode, "bank_id", cylinder.bankId);
        assignIfPresent(cylinderNode, "intake_runner_length_mm", cylinder.intakeRunnerLengthMm);
        assignIfPresent(cylinderNode, "intake_runner_diameter_mm", cylinder.intakeRunnerDiameterMm);
        assignIfPresent(cylinderNode, "exhaust_primary_length_mm", cylinder.exhaustPrimaryLengthMm);
        assignIfPresent(cylinderNode, "sound_attenuation", cylinder.soundAttenuation);
        assignIfPresent(cylinderNode, "blow_by_coefficient", cylinder.blowByCoefficient);
        assignIfPresent(cylinderNode, "piston_friction_coefficient", cylinder.pistonFrictionCoefficient);
        assignIfPresent(cylinderNode, "piston_breakaway_force_n", cylinder.pistonBreakawayForceN);
        assignIfPresent(cylinderNode, "piston_breakaway_velocity_mps", cylinder.pistonBreakawayVelocityMps);
        assignIfPresent(cylinderNode, "piston_viscous_friction_ns_per_m", cylinder.pistonViscousFrictionNsPerM);
        assignIfPresent(cylinderNode, "connecting_rod_inertia_kg_m2", cylinder.connectingRodMomentOfInertiaKgM2);
        assignIfPresent(cylinderNode, "intake_valve_count", cylinder.intakeValveCount);
        assignIfPresent(cylinderNode, "exhaust_valve_count", cylinder.exhaustValveCount);
        assignIfPresent(cylinderNode, "intake_valve_diameter_mm", cylinder.intakeValveDiameterMm);
        assignIfPresent(cylinderNode, "exhaust_valve_diameter_mm", cylinder.exhaustValveDiameterMm);
        assignIfPresent(cylinderNode, "deck_height_mm", cylinder.deckHeightMm);
        assignIfPresent(cylinderNode, "compression_height_mm", cylinder.compressionHeightMm);
        assignIfPresent(cylinderNode, "wrist_pin_offset_mm", cylinder.wristPinOffsetMm);
        assignIfPresent(cylinderNode, "piston_crown_volume_cc", cylinder.pistonCrownVolumeCc);
        assignIfPresent(cylinderNode, "head_chamber_volume_cc", cylinder.headChamberVolumeCc);
        assignIfPresent(cylinderNode, "head_gasket_thickness_mm", cylinder.headGasketThicknessMm);
        cylinders.push_back(cylinder);
    }
    return cylinders;
}

void applyCrankOffsets(EngineConfig& config) {
    const auto spacing = 720.0 / static_cast<double>(config.firingOrder.size());
    for (std::size_t index = 0; index < config.cylinders.size(); ++index) {
        auto& cylinder = config.cylinders[index];
        const auto found = std::find(config.firingOrder.begin(), config.firingOrder.end(), cylinder.id);
        if (found == config.firingOrder.end()) continue;
        cylinder.crankOffsetDegrees = static_cast<double>(std::distance(config.firingOrder.begin(), found)) * spacing;
        if (config.layout == EngineLayout::vLayout) {
            cylinder.bankOffsetDegrees = cylinder.id % 2U == 0U ? config.bankAngleDegrees * 0.5 : -config.bankAngleDegrees * 0.5;
        } else if (config.layout == EngineLayout::flat) {
            cylinder.bankOffsetDegrees = cylinder.id % 2U == 0U ? 90.0 : -90.0;
        } else if (config.layout == EngineLayout::radial) {
            cylinder.bankOffsetDegrees = static_cast<double>(index) * (360.0 / static_cast<double>(config.cylinders.size()));
        }
    }
}

[[nodiscard]] EngineConfig decodeEngineFile(const std::filesystem::path& path, const PartsLibrary& parts) {
    const auto document = YAML::LoadFile(path.string());
    const auto engine = document["engine"];
    if (!engine) throw std::runtime_error("Missing engine node");

    EngineConfig config;
    config.schemaVersion = document["schema_version"].as<std::uint32_t>(1);
    config.name = engine["name"].as<std::string>();
    config.layout = parseLayout(engine["layout"].as<std::string>("inline"));
    config.firingOrder = engine["firing_order"].as<std::vector<std::uint32_t>>();
    const auto hasExplicitCylinders = static_cast<bool>(engine["cylinders"]);
    config.cylinders = decodeCylinders(engine);

    assignIfPresent(engine, "idle_rpm", config.idleRpm);
    assignIfPresent(engine, "redline_rpm", config.redlineRpm);
    assignIfPresent(engine, "rotating_inertia_kg_m2", config.rotatingInertiaKgM2);
    assignIfPresent(engine, "friction_coefficient", config.frictionCoefficient);
    assignIfPresent(engine, "octane_rating", config.octaneRating);
    assignIfPresent(engine, "ambient_pressure_kpa", config.ambientPressureKpa);
    assignIfPresent(engine, "ambient_temperature_c", config.ambientTemperatureC);
    assignIfPresent(engine, "cooling_efficiency", config.coolingEfficiency);
    assignIfPresent(engine, "plenum_volume_l", config.plenumVolumeLitres);
    assignIfPresent(engine, "throttle_diameter_mm", config.throttleDiameterMm);
    assignIfPresent(engine, "bank_angle_deg", config.bankAngleDegrees);
    config.intake.plenumVolumeLitres = config.plenumVolumeLitres;
    config.intake.throttleDiameterMm = config.throttleDiameterMm;
    if (const auto intake = engine["intake"]) {
        assignIfPresent(intake, "plenum_volume_l", config.intake.plenumVolumeLitres);
        assignIfPresent(intake, "throttle_diameter_mm", config.intake.throttleDiameterMm);
        assignIfPresent(intake, "throttle_count", config.intake.throttleCount);
        assignIfPresent(intake, "throttle_discharge_coefficient", config.intake.throttleDischargeCoefficient);
        assignIfPresent(intake, "runner_length_mm", config.intake.runnerLengthMm);
        assignIfPresent(intake, "runner_diameter_mm", config.intake.runnerDiameterMm);
        assignIfPresent(intake, "idle_bypass_area_mm2", config.intake.idleBypassAreaMm2);
        assignIfPresent(intake, "throttle_gamma", config.intake.throttleGamma);
        config.plenumVolumeLitres = config.intake.plenumVolumeLitres;
        config.throttleDiameterMm = config.intake.throttleDiameterMm;
    }
    config.ignition.revLimitRpm = config.redlineRpm;
    if (const auto ignition = engine["ignition"]) {
        assignIfPresent(ignition, "rev_limit_rpm", config.ignition.revLimitRpm);
        assignIfPresent(ignition, "limiter_duration_s", config.ignition.limiterDurationSeconds);
        if (ignition["timing_curve"]) {
            config.ignition.timingCurve.clear();
            for (const auto& sample : ignition["timing_curve"])
                config.ignition.timingCurve.push_back({ sample["rpm"].as<double>(), sample["advance_deg"].as<double>() });
        }
    }
    if (const auto injection = engine["injection"]) {
        config.injection = decodeInjection(injection);
    }
    if (const auto solver = engine["solver"]) {
        assignIfPresent(solver, "mechanical_frequency_hz", config.solver.mechanicalFrequencyHz);
        assignIfPresent(solver, "maximum_frequency_hz", config.solver.maximumMechanicalFrequencyHz);
        assignIfPresent(solver, "maximum_crank_deg_per_step", config.solver.maximumCrankDegreesPerStep);
        assignIfPresent(solver, "gas_substeps", config.solver.gasSubsteps);
    }
    if (!hasExplicitCylinders) applyCrankOffsets(config);

    const auto uses = engine["uses"];
    applyPart(parts.fuels, uses, "fuel", config.fuelProperties);
    applyPart(parts.injections, uses, "injection", config.injection);
    applyPart(parts.camshafts, uses, "camshafts", config.camshafts);
    applyPart(parts.exhausts, uses, "exhaust", config.exhaust);
    applyPart(parts.transmissions, uses, "transmission", config.transmission);
    applyPart(parts.vehicles, uses, "vehicle", config.vehicle);
    if (engine["injection"]) config.injection = decodeInjection(engine["injection"]);
    if (engine["camshafts"]) config.camshafts = decodeCamshafts(engine["camshafts"]);
    if (engine["exhaust"]) config.exhaust = decodeExhaust(engine["exhaust"]);
    if (engine["transmission"]) config.transmission = decodeTransmission(engine["transmission"]);
    if (engine["vehicle"]) config.vehicle = decodeVehicle(engine["vehicle"]);
    if (engine["forced_induction"]) config.forcedInduction = decodeForcedInduction(engine["forced_induction"]);
    if (engine["thermal"]) config.thermal = decodeThermal(engine["thermal"]);
    if (const auto calibration = engine["combustion_calibration"]) {
        assignIfPresent(calibration, "base_ignition_delay_s", config.combustionCalibration.baseIgnitionDelaySeconds);
        assignIfPresent(calibration, "ignition_delay_temperature_exponent", config.combustionCalibration.ignitionDelayTemperatureExponent);
        assignIfPresent(calibration, "ignition_delay_pressure_exponent", config.combustionCalibration.ignitionDelayPressureExponent);
        assignIfPresent(calibration, "wall_heat_transfer_w_per_k", config.combustionCalibration.wallHeatTransferCoefficientWPerK);
        assignIfPresent(calibration, "residual_dilution_sensitivity", config.combustionCalibration.residualDilutionSensitivity);
    }
    if (const auto acoustics = engine["runner_acoustics"]) {
        assignIfPresent(acoustics, "enabled", config.runnerAcoustics.enabled);
        assignIfPresent(acoustics, "damping_ratio", config.runnerAcoustics.dampingRatio);
        assignIfPresent(acoustics, "coupling_gain", config.runnerAcoustics.couplingGain);
        assignIfPresent(acoustics, "maximum_pressure_amplitude_kpa", config.runnerAcoustics.maximumPressureAmplitudeKpa);
    }
    if (engine["fuel_properties"]) config.fuelProperties = decodeFuel(engine["fuel_properties"]);
    if (const auto crankJournals = engine["crank_journals"]) {
        config.crankJournals.clear();
        for (const auto& journal : crankJournals)
            config.crankJournals.push_back({ journal["id"].as<std::uint32_t>(),
                journal["angle_deg"].as<double>(), journal["throw_mm"].as<double>(),
                journal["crankshaft_id"].as<std::uint32_t>(1) });
    }
    if (const auto crankshafts = engine["crankshafts"]) {
        config.crankshafts.clear();
        for (const auto& item : crankshafts) {
            CrankshaftConfig crankshaft;
            crankshaft.id = item["id"].as<std::uint32_t>();
            assignIfPresent(item, "position_x_mm", crankshaft.positionXMm);
            assignIfPresent(item, "position_y_mm", crankshaft.positionYMm);
            assignIfPresent(item, "phase_offset_deg", crankshaft.phaseOffsetDegrees);
            assignIfPresent(item, "rotation_ratio", crankshaft.rotationRatio);
            assignIfPresent(item, "mass_kg", crankshaft.massKg);
            assignIfPresent(item, "flywheel_mass_kg", crankshaft.flywheelMassKg);
            assignIfPresent(item, "moment_of_inertia_kg_m2", crankshaft.momentOfInertiaKgM2);
            assignIfPresent(item, "friction_torque_nm", crankshaft.frictionTorqueNm);
            config.crankshafts.push_back(crankshaft);
        }
    }
    if (const auto paths = engine["intake_paths"]) {
        config.intakePaths.clear();
        for (const auto& item : paths) {
            IntakePathConfig pathConfig;
            pathConfig.id = item["id"].as<std::uint32_t>();
            pathConfig.cylinderIds = item["cylinder_ids"].as<std::vector<std::uint32_t>>();
            if (const auto geometry = item["geometry"]) {
                assignIfPresent(geometry, "plenum_volume_l", pathConfig.geometry.plenumVolumeLitres);
                assignIfPresent(geometry, "throttle_diameter_mm", pathConfig.geometry.throttleDiameterMm);
                assignIfPresent(geometry, "throttle_count", pathConfig.geometry.throttleCount);
                assignIfPresent(geometry, "throttle_discharge_coefficient", pathConfig.geometry.throttleDischargeCoefficient);
                assignIfPresent(geometry, "runner_length_mm", pathConfig.geometry.runnerLengthMm);
                assignIfPresent(geometry, "runner_diameter_mm", pathConfig.geometry.runnerDiameterMm);
                assignIfPresent(geometry, "idle_bypass_area_mm2", pathConfig.geometry.idleBypassAreaMm2);
                assignIfPresent(geometry, "throttle_gamma", pathConfig.geometry.throttleGamma);
            }
            config.intakePaths.push_back(std::move(pathConfig));
        }
    }
    if (const auto banks = engine["banks"]) {
        config.banks.clear();
        for (const auto& bankNode : banks) {
            CylinderBankConfig bank;
            bank.id = bankNode["id"].as<std::uint32_t>();
            bank.angleDegrees = bankNode["angle_deg"].as<double>(0.0);
            bank.cylinderIds = bankNode["cylinder_ids"].as<std::vector<std::uint32_t>>();
            bank.intakeId = bankNode["intake_id"].as<std::uint32_t>(0);
            bank.exhaustPathId = bankNode["exhaust_path_id"].as<std::uint32_t>(0);
            bank.camshafts = bankNode["camshafts"] ? decodeCamshafts(bankNode["camshafts"]) : config.camshafts;
            config.banks.push_back(std::move(bank));
        }
    }
    if (const auto paths = engine["exhaust_paths"]) {
        config.exhaustPaths.clear();
        for (const auto& pathNode : paths) {
            ExhaustPathConfig pathConfig;
            pathConfig.id = pathNode["id"].as<std::uint32_t>();
            pathConfig.cylinderIds = pathNode["cylinder_ids"].as<std::vector<std::uint32_t>>();
            pathConfig.geometry = pathNode["geometry"] ? decodeExhaust(pathNode["geometry"]) : config.exhaust;
            pathConfig.impulseResponsePath = pathNode["impulse_response"].as<std::string>("");
            pathConfig.audioVolume = pathNode["audio_volume"].as<double>(1.0);
            if (const auto graph = pathNode["graph"])
                pathConfig.network = decodeExhaustNetwork(graph);
            config.exhaustPaths.push_back(std::move(pathConfig));
        }
    }

    if (config.banks.empty()) {
        if (config.layout == EngineLayout::vLayout || config.layout == EngineLayout::flat) {
            CylinderBankConfig left { 1, -config.bankAngleDegrees * 0.5, {}, config.camshafts, 0, 1 };
            CylinderBankConfig right { 2, config.bankAngleDegrees * 0.5, {}, config.camshafts, 0, 1 };
            for (auto& cylinder : config.cylinders) {
                auto& bank = cylinder.id % 2U == 0U ? right : left;
                bank.cylinderIds.push_back(cylinder.id);
                cylinder.bankId = bank.id;
            }
            config.banks = { std::move(left), std::move(right) };
        } else {
            CylinderBankConfig bank { 1, 0.0, {}, config.camshafts, 0, 1 };
            for (auto& cylinder : config.cylinders) { bank.cylinderIds.push_back(cylinder.id); cylinder.bankId = 1; }
            config.banks.push_back(std::move(bank));
        }
    }
    if (config.exhaustPaths.empty()) {
        ExhaustPathConfig pathConfig;
        pathConfig.id = 1;
        pathConfig.geometry = config.exhaust;
        pathConfig.inheritsGlobalGeometry = true;
        for (const auto& cylinder : config.cylinders) pathConfig.cylinderIds.push_back(cylinder.id);
        config.exhaustPaths.push_back(std::move(pathConfig));
    }

    normaliseEngineConfig(config);
    if (const auto error = validateEngineConfig(config)) throw std::runtime_error(*error);
    return config;
}

[[nodiscard]] PartsLibrary loadParts(const std::filesystem::path& root) {
    const auto partsRoot = root / "parts";
    return {
        loadPartMap<FuelConfig>(partsRoot / "fuels.yaml", decodeFuel),
        loadPartMap<InjectionConfig>(partsRoot / "injections.yaml", decodeInjection),
        loadPartMap<CamshaftConfig>(partsRoot / "camshafts.yaml", decodeCamshafts),
        loadPartMap<ExhaustConfig>(partsRoot / "exhausts.yaml", decodeExhaust),
        loadPartMap<TransmissionConfig>(partsRoot / "transmissions.yaml", decodeTransmission),
        loadPartMap<VehicleConfig>(partsRoot / "vehicles.yaml", decodeVehicle)
    };
}
} // namespace

EngineCatalogLoadResult loadEngineCatalog(const std::filesystem::path& rootDirectory) {
    EngineCatalogLoadResult result;
    const auto enginesRoot = rootDirectory / "engines";
    if (!std::filesystem::exists(enginesRoot)) {
        result.errors.push_back("Engine catalog not found: " + enginesRoot.string());
        return result;
    }

    PartsLibrary parts;
    try {
        parts = loadParts(rootDirectory);
    } catch (const std::exception& error) {
        result.errors.push_back(std::string("Failed to load parts library: ") + error.what());
        return result;
    }

    std::vector<std::filesystem::path> files;
    for (const auto& entry : std::filesystem::directory_iterator(enginesRoot)) {
        if (entry.is_regular_file() && entry.path().extension() == ".yaml") files.push_back(entry.path());
    }
    std::sort(files.begin(), files.end());
    for (const auto& file : files) {
        try {
            EngineCatalogEntry entry;
            entry.config = decodeEngineFile(file, parts);
            entry.sourcePath = file;
            if (const auto document = YAML::LoadFile(file.string()); document["family"])
                entry.family = document["family"].as<std::string>();
            result.entries.push_back(std::move(entry));
        } catch (const std::exception& error) {
            result.errors.push_back(file.filename().string() + ": " + error.what());
        }
    }
    return result;
}

std::vector<EngineConfig> makeCatalogOrBasePresets(const std::filesystem::path& rootDirectory) {
    auto loaded = loadEngineCatalog(rootDirectory);
    if (loaded.entries.empty()) return makeBaseEnginePresets();
    std::vector<EngineConfig> configs;
    configs.reserve(loaded.entries.size());
    for (auto& entry : loaded.entries) configs.push_back(std::move(entry.config));
    return configs;
}

} // namespace enginelab
