#include <enginelab/catalog/EngineCatalog.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <map>
#include <optional>
#include <numbers>
#include <set>
#include <sstream>
#include <stdexcept>

namespace enginelab {
namespace {
[[nodiscard]] std::uint8_t decodeOptionalCrossoverPort(
    const YAML::Node& object, const char* key) {
    const auto encoded = object[key];
    if (!encoded) return unspecifiedExhaustComponentPort;
    const auto value = encoded.as<unsigned>();
    if (value > 1U)
        throw std::invalid_argument(
            std::string("Exhaust crossover port must be 0 or 1: ") + key);
    return static_cast<std::uint8_t>(value);
}
[[nodiscard]] ExhaustAfterfireStrategy decodeAfterfireStrategy(
    std::string_view value) {
    if (value == "clean_dfco")
        return ExhaustAfterfireStrategy::cleanDfco;
    if (value == "continuous_anti_lag")
        return ExhaustAfterfireStrategy::continuousAntiLag;
    if (value == "discrete_afterfire")
        return ExhaustAfterfireStrategy::discreteAfterfire;
    throw std::invalid_argument(
        "Unknown exhaust afterfire strategy: " + std::string(value));
}

[[nodiscard]] ExhaustComponentConfig exhaustComponent(
    std::uint32_t id, ExhaustComponentType type, double lengthMm,
    double diameterMm) {
    ExhaustComponentConfig component;
    component.id = id;
    component.type = type;
    component.lengthMm = lengthMm;
    component.diameterMm = diameterMm;
    return component;
}

/** Compile catalogue scalar hardware into a component DAG once at load time.
 * This is a canonical representation, not the old reduced audio fallback: the
 * same graph is subsequently consumed by gas dynamics, acoustics, the editor
 * and exported snapshots. */
void compileCatalogueExhaustPath(ExhaustPathConfig& path,
                                 std::string_view presetKey) {
    if (path.network || path.cylinderIds.empty()) return;
    const auto& geometry = path.geometry;
    ExhaustNetworkConfig network;
    const auto primaryLength = std::max(20.0, geometry.primaryLengthMm);
    const auto primaryDiameter = std::max(10.0, geometry.primaryDiameterMm);
    const auto collectorDiameter = std::max(
        primaryDiameter, geometry.collectorDiameterMm);
    const auto addConnection = [&network](std::uint32_t from,
                                          std::uint32_t to) {
        network.connections.push_back({ from, to });
    };

    if (presetKey == "aircraft_manifold") {
        // Short independent ejector stacks: no atmosphere-side merge, no
        // invented 450 mm muffler. All outlets may share an audio bus while
        // remaining disconnected physical terminals in the global network.
        for (std::size_t index = 0; index < path.cylinderIds.size(); ++index) {
            const auto outletId = static_cast<std::uint32_t>(500 + index);
            auto outlet = exhaustComponent(outletId,
                // An ejector stack is itself the terminal duct. Representing
                // it as a pipe followed by a second outlet duct doubled every
                // stack's cells and interfaces while adding no geometry. The
                // inlet diameter carries the tube; outletDiameterMm carries
                // the authored open lip/taper on that same 150 mm element.
                ExhaustComponentType::outlet,
                primaryLength, primaryDiameter);
            outlet.outletDiameterMm = std::max(
                primaryDiameter, geometry.outletDiameterMm);
            outlet.dischargeCoefficient =
                geometry.outletDischargeCoefficient;
            outlet.acousticPositionM = path.acousticPositionM;
            outlet.acousticPositionM.x += 0.055
                * (static_cast<double>(index)
                    - 0.5 * static_cast<double>(path.cylinderIds.size() - 1));
            outlet.acousticAxis = path.acousticAxis;
            outlet.acousticTermination = path.acousticTermination;
            network.components.push_back(outlet);
            network.cylinderConnections.push_back({
                path.cylinderIds[index], outletId });
        }
        path.network = std::move(network);
        return;
    }

    std::vector<std::uint32_t> primaryIds;
    primaryIds.reserve(path.cylinderIds.size());
    for (std::size_t index = 0; index < path.cylinderIds.size(); ++index) {
        const auto id = static_cast<std::uint32_t>(100 + index);
        primaryIds.push_back(id);
        network.components.push_back(exhaustComponent(
            id, ExhaustComponentType::pipe,
            primaryLength, primaryDiameter));
        network.cylinderConnections.push_back({ path.cylinderIds[index], id });
    }

    auto terminalComponent = std::uint32_t {};
    if (presetKey == "motorcycle_4_2_1"
        && primaryIds.size() == 4) {
        const auto secondaryDiameter = std::min(collectorDiameter,
            primaryDiameter * 1.28);
        // A conventional inline-four 4-2-1 pairs the two outside cylinders
        // (1+4) and the two inside cylinders (2+3). Pairing adjacent entries
        // made consecutive firing pulses collide in one secondary and erased
        // the alternating collector signature the layout is chosen for.
        constexpr std::array<std::array<std::size_t, 2>, 2> pairs {{
            {{ 0, 3 }}, {{ 1, 2 }}
        }};
        for (std::size_t pair = 0; pair < pairs.size(); ++pair) {
            const auto mergeId = static_cast<std::uint32_t>(200 + pair);
            auto merge = exhaustComponent(mergeId,
                ExhaustComponentType::merge, 90.0, secondaryDiameter);
            merge.volumeLitres = 0.20;
            network.components.push_back(merge);
            const auto secondaryId = static_cast<std::uint32_t>(300 + pair);
            network.components.push_back(exhaustComponent(secondaryId,
                ExhaustComponentType::pipe,
                primaryLength * 0.42, secondaryDiameter));
            addConnection(primaryIds[pairs[pair][0]], mergeId);
            addConnection(primaryIds[pairs[pair][1]], mergeId);
            addConnection(mergeId, secondaryId);
        }
        auto merge = exhaustComponent(400,
            ExhaustComponentType::merge, 130.0, collectorDiameter);
        merge.volumeLitres = std::max(0.20,
            geometry.collectorVolumeLitres);
        network.components.push_back(merge);
        addConnection(300, 400);
        addConnection(301, 400);
        terminalComponent = 400;
    } else if (primaryIds.size() == 1) {
        terminalComponent = primaryIds.front();
    } else {
        auto merge = exhaustComponent(200,
            ExhaustComponentType::merge,
            std::max(120.0, primaryLength * 0.35), collectorDiameter);
        merge.volumeLitres = std::max(0.05,
            geometry.collectorVolumeLitres);
        network.components.push_back(merge);
        for (const auto primaryId : primaryIds)
            addConnection(primaryId, 200);
        terminalComponent = 200;
    }

    const auto chamberConfigured =
        geometry.mufflerChamberDiameterMm > 1.0
        && geometry.mufflerChamberLengthMm > 1.0;
    const auto downstreamId = std::uint32_t { 600 };
    if (chamberConfigured) {
        auto muffler = exhaustComponent(downstreamId,
            ExhaustComponentType::muffler,
            geometry.mufflerChamberLengthMm, collectorDiameter);
        const auto radiusM = geometry.mufflerChamberDiameterMm * 0.0005;
        muffler.volumeLitres = std::numbers::pi * radiusM * radiusM
            * geometry.mufflerChamberLengthMm;
        muffler.restriction = geometry.mufflerRestriction;
        muffler.packingFlowResistivityPaSPerM2 =
            geometry.mufflerPackingFlowResistivityPaSPerM2;
        muffler.packingThicknessMm =
            geometry.mufflerPackingThicknessMm;
        muffler.perforatedOpenAreaRatio =
            geometry.mufflerPerforatedOpenAreaRatio;
        network.components.push_back(muffler);
    } else {
        network.components.push_back(exhaustComponent(downstreamId,
            ExhaustComponentType::pipe,
            std::max(100.0, primaryLength * 0.28), collectorDiameter));
    }
    addConnection(terminalComponent, downstreamId);

    auto outlet = exhaustComponent(700, ExhaustComponentType::outlet,
        // Boundary-only opening: the layout derives its minimum physically
        // resolvable terminal cell. An explicitly authored tailpipe remains a
        // pipe component upstream and keeps its real length.
        0.0, std::max(10.0, geometry.outletDiameterMm));
    outlet.dischargeCoefficient = geometry.outletDischargeCoefficient;
    outlet.acousticPositionM = path.acousticPositionM;
    outlet.acousticAxis = path.acousticAxis;
    outlet.acousticTermination = path.acousticTermination;
    network.components.push_back(outlet);
    addConnection(downstreamId, 700);
    path.network = std::move(network);
}

template <typename T>
void assignIfPresent(const YAML::Node& node, const char* key, T& value) {
    if (node && node[key]) value = node[key].as<T>();
}

[[nodiscard]] AcousticPoint3M decodePoint(
    const YAML::Node& node, AcousticPoint3M fallback = {}) {
    return { node["x"].as<double>(fallback.x), node["y"].as<double>(fallback.y),
        node["z"].as<double>(fallback.z) };
}

[[nodiscard]] EngineLayout parseLayout(const std::string& value) {
    if (value == "inline") return EngineLayout::inlineLayout;
    if (value == "v") return EngineLayout::vLayout;
    if (value == "flat") return EngineLayout::flat;
    if (value == "radial") return EngineLayout::radial;
    if (value == "custom") return EngineLayout::custom;
    throw std::runtime_error("Unknown engine layout: " + value);
}

[[nodiscard]] std::string voicingFileKey(std::string_view value) {
    std::string result;
    result.reserve(value.size());
    auto previousSeparator = false;
    for (const auto character : value) {
        const auto byte = static_cast<unsigned char>(character);
        if (std::isalnum(byte)) {
            result.push_back(static_cast<char>(std::tolower(byte)));
            previousSeparator = false;
        } else if (!previousSeparator && !result.empty()) {
            result.push_back('_');
            previousSeparator = true;
        }
    }
    while (!result.empty() && result.back() == '_') result.pop_back();
    return result;
}

void requireKnownKeys(const YAML::Node& node,
                      const std::set<std::string>& allowed,
                      std::string_view context) {
    if (!node || !node.IsMap())
        throw std::runtime_error(std::string(context) + " must be a map");
    for (const auto& item : node) {
        const auto key = item.first.as<std::string>();
        if (!allowed.contains(key))
            throw std::runtime_error(std::string(context)
                + ": unknown key `" + key + "`");
    }
}

void assignVoicingNumber(const YAML::Node& node, const char* key,
                         double minimum, double maximum, double& destination) {
    if (!node[key]) return;
    const auto value = node[key].as<double>();
    if (!std::isfinite(value) || value < minimum || value > maximum)
        throw std::runtime_error(std::string("voicing.") + key
            + " must be finite and in [" + std::to_string(minimum)
            + ", " + std::to_string(maximum) + "]");
    destination = value;
}

void applyVoicingFile(const std::filesystem::path& file,
                      AudioVoicingConfig& voicing) {
    const auto document = YAML::LoadFile(file.string());
    requireKnownKeys(document, { "schema_version", "voicing" },
        file.filename().string());
    if (document["schema_version"].as<int>(1) != 1)
        throw std::runtime_error(file.filename().string()
            + ": unsupported schema_version");
    const auto node = document["voicing"];
    if (!node) return;
    requireKnownKeys(node, {
        "volume", "convolution", "high_frequency_gain",
        "low_frequency_gain", "low_frequency_noise",
        "high_frequency_noise", "combustion_gain", "exhaust_gain",
        "intake_gain", "mechanical_gain", "stereo_width",
        "outlet_jet_gain", "saturation_drive", "saturation_placement",
        "monitor_mode"
    }, file.filename().string() + ".voicing");
    assignVoicingNumber(node, "volume", 0.0, 2.0, voicing.volume);
    assignVoicingNumber(node, "convolution", 0.0, 1.0, voicing.convolution);
    assignVoicingNumber(node, "high_frequency_gain", 0.2, 2.5,
        voicing.highFrequencyGain);
    assignVoicingNumber(node, "low_frequency_gain", 0.2, 2.5,
        voicing.lowFrequencyGain);
    assignVoicingNumber(node, "low_frequency_noise", 0.0, 1.5,
        voicing.lowFrequencyNoise);
    assignVoicingNumber(node, "high_frequency_noise", 0.0, 1.5,
        voicing.highFrequencyNoise);
    assignVoicingNumber(node, "combustion_gain", 0.0, 2.0,
        voicing.combustionGain);
    assignVoicingNumber(node, "exhaust_gain", 0.0, 2.0,
        voicing.exhaustGain);
    assignVoicingNumber(node, "intake_gain", 0.0, 2.0,
        voicing.intakeGain);
    assignVoicingNumber(node, "mechanical_gain", 0.0, 2.0,
        voicing.mechanicalGain);
    assignVoicingNumber(node, "stereo_width", 0.0, 2.0,
        voicing.stereoWidth);
    assignVoicingNumber(node, "outlet_jet_gain", 0.0, 2.0,
        voicing.outletJetGain);
    assignVoicingNumber(node, "saturation_drive", 0.0, 4.0,
        voicing.saturationDrive);
    if (node["saturation_placement"]) {
        const auto placement = node["saturation_placement"].as<std::string>();
        if (placement == "pre_shelf")
            voicing.saturationPlacement = AudioSaturationPlacement::preShelf;
        else if (placement == "post_shelf")
            voicing.saturationPlacement = AudioSaturationPlacement::postShelf;
        else
            throw std::runtime_error("voicing.saturation_placement must be pre_shelf or post_shelf");
    }
    if (node["monitor_mode"]) {
        const auto mode = node["monitor_mode"].as<std::string>();
        if (mode == "physical_reference")
            voicing.monitorMode = AudioMonitorMode::physicalReference;
        else if (mode == "capture_voiced")
            voicing.monitorMode = AudioMonitorMode::captureVoiced;
        else
            throw std::runtime_error(
                "voicing.monitor_mode must be physical_reference or capture_voiced");
    }
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
    assignIfPresent(node, "muffler_chamber_diameter_mm", value.mufflerChamberDiameterMm);
    assignIfPresent(node, "muffler_chamber_length_mm", value.mufflerChamberLengthMm);
    assignIfPresent(node, "muffler_packing_flow_resistivity_pa_s_m2",
        value.mufflerPackingFlowResistivityPaSPerM2);
    assignIfPresent(node, "muffler_packing_thickness_mm",
        value.mufflerPackingThicknessMm);
    assignIfPresent(node, "muffler_perforated_open_area_ratio",
        value.mufflerPerforatedOpenAreaRatio);
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
    if (value == "crossover") return ExhaustComponentType::crossover;
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
            assignIfPresent(encoded, "outlet_diameter_mm", component.outletDiameterMm);
            assignIfPresent(encoded, "volume_l", component.volumeLitres);
            assignIfPresent(encoded, "restriction", component.restriction);
            assignIfPresent(encoded, "resonance_hz", component.resonanceHz);
            assignIfPresent(encoded, "acoustic_gain", component.acousticGain);
            assignIfPresent(encoded, "discharge_coefficient", component.dischargeCoefficient);
            assignIfPresent(encoded, "packing_flow_resistivity_pa_s_m2",
                component.packingFlowResistivityPaSPerM2);
            assignIfPresent(encoded, "packing_thickness_mm",
                component.packingThicknessMm);
            assignIfPresent(encoded, "perforated_open_area_ratio",
                component.perforatedOpenAreaRatio);
            assignIfPresent(encoded, "catalyst_cell_density_cpsi",
                component.catalystCellDensityCpsi);
            assignIfPresent(encoded, "catalyst_open_area_ratio",
                component.catalystOpenAreaRatio);
            assignIfPresent(encoded,
                "catalyst_substrate_volumetric_heat_capacity_j_m3_k",
                component.catalystSubstrateVolumetricHeatCapacityJPerM3K);
            assignIfPresent(encoded, "crossover_coupling",
                component.crossoverCoupling);
            if (encoded["acoustic_position_m"])
                component.acousticPositionM = decodePoint(encoded["acoustic_position_m"]);
            if (encoded["acoustic_axis"])
                component.acousticAxis = decodePoint(
                    encoded["acoustic_axis"], component.acousticAxis);
            component.acousticTermination = encoded["acoustic_termination"]
                .as<std::string>("unflanged") == "flanged"
                ? AcousticTerminationType::flanged
                : AcousticTerminationType::unflanged;
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
            network.connections.push_back({
                encoded["from_component_id"].as<std::uint32_t>(),
                encoded["to_component_id"].as<std::uint32_t>(),
                decodeOptionalCrossoverPort(encoded, "from_port"),
                decodeOptionalCrossoverPort(encoded, "to_port") });
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
    assignIfPresent(node, "tyre_grip_limit_enabled", value.tyreGripLimitEnabled);
    const auto drivenAxle = node["driven_axle_layout"].as<std::string>("rear");
    if (drivenAxle == "front")
        value.drivenAxleLayout = DrivenAxleLayout::front;
    else if (drivenAxle == "rear")
        value.drivenAxleLayout = DrivenAxleLayout::rear;
    else if (drivenAxle == "all")
        value.drivenAxleLayout = DrivenAxleLayout::all;
    else
        throw std::invalid_argument(
            "Unknown driven axle layout: " + drivenAxle);
    assignIfPresent(node, "driven_axle_weight_fraction", value.drivenAxleWeightFraction);
    assignIfPresent(node, "wheelbase_m", value.wheelbaseM);
    assignIfPresent(
        node, "center_of_gravity_height_m",
        value.centerOfGravityHeightM);
    assignIfPresent(node, "maximum_brake_force_n", value.maximumBrakeForceN);
    return value;
}

[[nodiscard]] StructuralNvhConfig decodeStructuralNvh(
    const YAML::Node& node) {
    StructuralNvhConfig value;
    const auto provenance =
        node["provenance"].as<std::string>("estimated_family");
    if (provenance == "estimated_family")
        value.provenance =
            StructuralNvhProvenance::estimatedFamily;
    else if (provenance == "calculated_geometry")
        value.provenance =
            StructuralNvhProvenance::calculatedGeometry;
    else if (provenance == "measured")
        value.provenance = StructuralNvhProvenance::measured;
    else
        throw std::invalid_argument(
            "Unknown structural NVH provenance: " + provenance);
    value.source = node["source"].as<std::string>("");
    if (const auto modes = node["modes"]) {
        value.modes.reserve(modes.size());
        for (const auto& item : modes) {
            StructuralModeConfig mode;
            mode.name = item["name"].as<std::string>();
            const auto drive = item["drive"].as<std::string>();
            if (drive == "head_gas")
                mode.drive = StructuralModeDrive::headGas;
            else if (drive == "bearing_axial")
                mode.drive = StructuralModeDrive::bearingAxial;
            else if (drive == "bearing_lateral")
                mode.drive = StructuralModeDrive::bearingLateral;
            else if (drive == "torsion")
                mode.drive = StructuralModeDrive::torsion;
            else
                throw std::invalid_argument(
                    "Unknown structural mode drive: " + drive);
            mode.frequencyHz =
                item["frequency_hz"].as<double>();
            mode.dampingRatio =
                item["damping_ratio"].as<double>();
            mode.modalMassKg =
                item["modal_mass_kg"].as<double>();
            mode.radiatingAreaM2 =
                item["radiating_area_m2"].as<double>();
            mode.radiationEfficiency =
                item["radiation_efficiency"].as<double>();
            mode.surfaceVelocityRmsScale =
                item["surface_velocity_rms_scale"].as<double>();
            mode.torqueRadiusM =
                item["torque_radius_m"].as<double>();
            mode.cylinderParticipation =
                item["cylinder_participation"]
                    .as<std::vector<double>>();
            value.modes.push_back(std::move(mode));
        }
    }
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
    assignIfPresent(node, "compressor_blade_count", value.compressorBladeCount);
    assignIfPresent(node, "turbine_blade_count", value.turbineBladeCount);
    assignIfPresent(node, "supercharger_lobe_count", value.superchargerLobeCount);
    assignIfPresent(node, "supercharger_drive_ratio", value.superchargerDriveRatio);
    assignIfPresent(node, "compressor_inducer_diameter_mm", value.compressorInducerDiameterMm);
    assignIfPresent(node, "turbine_exducer_diameter_mm", value.turbineExducerDiameterMm);
    assignIfPresent(node, "blow_off_valve_flow_area_mm2", value.blowOffValveFlowAreaMm2);
    assignIfPresent(node, "blow_off_valve_opening_pressure_ratio", value.blowOffValveOpeningPressureRatio);
    assignIfPresent(node, "blow_off_valve_discharge_coefficient", value.blowOffValveDischargeCoefficient);
    assignIfPresent(node, "tonal_acoustic_efficiency", value.tonalAcousticEfficiency);
    assignIfPresent(node, "turbulent_jet_noise_coefficient", value.turbulentJetNoiseCoefficient);
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
    assignIfPresent(node, "cetane_number", value.cetaneNumber);
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
    assignIfPresent(node, "direct_spray_vaporisation_time_constant_s",
                    value.directSprayVaporisationTimeConstantSeconds);
    assignIfPresent(node, "direct_spray_entrainment_time_constant_s",
                    value.directSprayEntrainmentTimeConstantSeconds);
    if (const auto limit = node["full_load_fuel_limit"]) {
        value.fullLoadFuelLimit.clear();
        for (const auto& sample : limit)
            value.fullLoadFuelLimit.push_back({
                sample["rpm"].as<double>(),
                sample["mg_per_cycle"].as<double>() });
    }
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
    const auto cycle = engine["cycle"].as<std::string>("four_stroke");
    if (cycle == "four_stroke") config.cycle = EngineCycle::fourStroke;
    else if (cycle == "two_stroke") config.cycle = EngineCycle::twoStroke;
    else throw std::runtime_error("Unknown engine cycle: " + cycle);
    const auto fuelType = engine["fuel"].as<std::string>("gasoline");
    if (fuelType == "gasoline") config.fuel = FuelType::gasoline;
    else if (fuelType == "diesel") config.fuel = FuelType::diesel;
    else throw std::runtime_error("Unknown fuel type: " + fuelType);
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
        assignIfPresent(intake, "runner_plenum_diameter_mm", config.intake.runnerPlenumDiameterMm);
        assignIfPresent(intake, "airbox_volume_l", config.intake.airboxVolumeLitres);
        assignIfPresent(intake, "inlet_duct_length_mm", config.intake.inletDuctLengthMm);
        assignIfPresent(intake, "inlet_duct_diameter_mm", config.intake.inletDuctDiameterMm);
        assignIfPresent(intake, "bellmouth_diameter_mm", config.intake.bellmouthDiameterMm);
        assignIfPresent(intake, "idle_bypass_area_mm2", config.intake.idleBypassAreaMm2);
        assignIfPresent(intake, "throttle_gamma", config.intake.throttleGamma);
        config.plenumVolumeLitres = config.intake.plenumVolumeLitres;
        config.throttleDiameterMm = config.intake.throttleDiameterMm;
    }
    config.ignition.revLimitRpm = config.redlineRpm;
    if (const auto ignition = engine["ignition"]) {
        assignIfPresent(ignition, "rev_limit_rpm", config.ignition.revLimitRpm);
        assignIfPresent(ignition, "limiter_duration_s", config.ignition.limiterDurationSeconds);
        assignIfPresent(ignition, "limiter_keeps_fuel", config.ignition.limiterKeepsFuel);
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
    const auto exhaustPresetKey = uses && uses["exhaust"]
        ? uses["exhaust"].as<std::string>() : std::string {};
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
        assignIfPresent(calibration, "chamber_turbulence_intensity_ratio", config.combustionCalibration.chamberTurbulenceIntensityRatio);
        assignIfPresent(calibration, "ignition_site_count", config.combustionCalibration.ignitionSiteCount);
        assignIfPresent(calibration, "compression_ignition_delay_scale", config.combustionCalibration.compressionIgnitionDelayScale);
        assignIfPresent(calibration, "compression_ignition_mixing_time_s", config.combustionCalibration.compressionIgnitionMixingTimeSeconds);
        assignIfPresent(calibration, "compression_ignition_premixed_fraction", config.combustionCalibration.compressionIgnitionPremixedFraction);
        assignIfPresent(calibration, "cycle_variation_cov", config.combustionCalibration.cycleVariationCoefficientOfVariation);
        assignIfPresent(calibration, "cycle_variation_correlation", config.combustionCalibration.cycleVariationCorrelation);
    }
    if (const auto afterfire = engine["exhaust_afterfire"]) {
        assignIfPresent(afterfire, "enabled", config.exhaustAfterfire.enabled);
        if (afterfire["strategy"])
            config.exhaustAfterfire.strategy = decodeAfterfireStrategy(
                afterfire["strategy"].as<std::string>());
        assignIfPresent(afterfire, "ignition_temperature_k", config.exhaustAfterfire.ignitionTemperatureK);
        assignIfPresent(afterfire, "reaction_time_constant_s", config.exhaustAfterfire.reactionTimeConstantSeconds);
        assignIfPresent(afterfire, "reaction_efficiency", config.exhaustAfterfire.reactionEfficiency);
        assignIfPresent(afterfire, "overrun_fuel_fraction", config.exhaustAfterfire.overrunFuelFraction);
        assignIfPresent(afterfire, "overrun_minimum_rpm", config.exhaustAfterfire.overrunMinimumRpm);
        assignIfPresent(afterfire, "overrun_maximum_throttle", config.exhaustAfterfire.overrunMaximumThrottle);
        assignIfPresent(afterfire, "overrun_pulse_hz", config.exhaustAfterfire.overrunPulseHz);
        assignIfPresent(afterfire, "overrun_pulse_duty", config.exhaustAfterfire.overrunPulseDutyCycle);
        assignIfPresent(afterfire, "overrun_pulse_timing_variation", config.exhaustAfterfire.overrunPulseTimingVariation);
        assignIfPresent(afterfire, "induction_time_s", config.exhaustAfterfire.inductionTimeSeconds);
        assignIfPresent(afterfire, "induction_reference_pressure_kpa", config.exhaustAfterfire.inductionReferencePressureKpa);
        assignIfPresent(afterfire, "induction_activation_temperature_k", config.exhaustAfterfire.inductionActivationTemperatureK);
        assignIfPresent(afterfire, "induction_pressure_exponent", config.exhaustAfterfire.inductionPressureExponent);
        assignIfPresent(afterfire, "induction_equivalence_ratio_exponent", config.exhaustAfterfire.inductionEquivalenceRatioExponent);
        assignIfPresent(afterfire, "induction_decay_time_s", config.exhaustAfterfire.inductionDecayTimeSeconds);
        assignIfPresent(afterfire, "minimum_equivalence_ratio", config.exhaustAfterfire.minimumEquivalenceRatio);
        assignIfPresent(afterfire, "maximum_equivalence_ratio", config.exhaustAfterfire.maximumEquivalenceRatio);
        assignIfPresent(afterfire, "quench_temperature_k", config.exhaustAfterfire.quenchTemperatureK);
    }
    if (const auto acoustics = engine["runner_acoustics"]) {
        assignIfPresent(acoustics, "enabled", config.runnerAcoustics.enabled);
        assignIfPresent(acoustics, "damping_ratio", config.runnerAcoustics.dampingRatio);
        assignIfPresent(acoustics, "coupling_gain", config.runnerAcoustics.couplingGain);
        assignIfPresent(acoustics, "maximum_pressure_amplitude_kpa", config.runnerAcoustics.maximumPressureAmplitudeKpa);
    }
    if (const auto structuralNvh = engine["structural_nvh"])
        config.structuralNvh =
            decodeStructuralNvh(structuralNvh);
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
                assignIfPresent(geometry, "runner_plenum_diameter_mm", pathConfig.geometry.runnerPlenumDiameterMm);
                assignIfPresent(geometry, "airbox_volume_l", pathConfig.geometry.airboxVolumeLitres);
                assignIfPresent(geometry, "inlet_duct_length_mm", pathConfig.geometry.inletDuctLengthMm);
                assignIfPresent(geometry, "inlet_duct_diameter_mm", pathConfig.geometry.inletDuctDiameterMm);
                assignIfPresent(geometry, "bellmouth_diameter_mm", pathConfig.geometry.bellmouthDiameterMm);
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
    if (const auto observer = engine["acoustic_observer"]) {
        if (observer["left_microphone_m"])
            config.acousticObserver.leftMicrophoneM = decodePoint(
                observer["left_microphone_m"],
                config.acousticObserver.leftMicrophoneM);
        if (observer["right_microphone_m"])
            config.acousticObserver.rightMicrophoneM = decodePoint(
                observer["right_microphone_m"],
                config.acousticObserver.rightMicrophoneM);
        assignIfPresent(observer, "sound_speed_mps",
            config.acousticObserver.soundSpeedMps);
        assignIfPresent(observer, "listening_distance_m",
            config.acousticObserver.listeningDistanceM);
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
            if (pathNode["acoustic_position_m"])
                pathConfig.acousticPositionM = decodePoint(
                    pathNode["acoustic_position_m"]);
            if (pathNode["acoustic_axis"])
                pathConfig.acousticAxis = decodePoint(
                    pathNode["acoustic_axis"], pathConfig.acousticAxis);
            pathConfig.acousticTermination = pathNode["acoustic_termination"]
                .as<std::string>("unflanged") == "flanged"
                ? AcousticTerminationType::flanged
                : AcousticTerminationType::unflanged;
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
        } else if (config.layout == EngineLayout::radial) {
            // A radial's bank angle is per cylinder. Grouping every cylinder
            // into the generic zero-degree inline bank makes bankAngleFor()
            // prefer that bank over each authored bank_offset_deg; all pistons
            // then move in phase while their valve/firing phases remain spread
            // around 720 degrees. The result is not a cosmetic layout error:
            // most cylinders open their valves on the wrong piston stroke.
            config.banks.reserve(config.cylinders.size());
            for (auto& cylinder : config.cylinders) {
                CylinderBankConfig bank {
                    cylinder.id,
                    cylinder.bankOffsetDegrees,
                    { cylinder.id },
                    config.camshafts,
                    0,
                    1,
                };
                cylinder.bankId = bank.id;
                config.banks.push_back(std::move(bank));
            }
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
    for (auto& exhaustPath : config.exhaustPaths)
        compileCatalogueExhaustPath(exhaustPath, exhaustPresetKey);

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

AudioVoicingLoadResult loadAudioVoicing(
    const std::filesystem::path& rootDirectory,
    std::string_view family,
    std::string_view engineKey) {
    AudioVoicingLoadResult result;
    const auto voicingRoot = rootDirectory / "voicing";
    const std::array candidates {
        voicingRoot / "default.yaml",
        voicingRoot / "families" / (voicingFileKey(family) + ".yaml"),
        voicingRoot / "engines" / (voicingFileKey(engineKey) + ".yaml")
    };
    try {
        for (const auto& file : candidates) {
            if (file.filename() == ".yaml" || !std::filesystem::exists(file))
                continue;
            applyVoicingFile(file, result.voicing);
            result.sources.push_back(file);
        }
    } catch (const std::exception& error) {
        result.voicing = {};
        result.sources.clear();
        result.error = error.what();
    }
    return result;
}

std::filesystem::file_time_type audioVoicingRevision(
    const std::filesystem::path& rootDirectory) noexcept {
    const auto voicingRoot = rootDirectory / "voicing";
    auto revision = std::filesystem::file_time_type::min();
    std::error_code error;
    if (!std::filesystem::is_directory(voicingRoot, error)) return revision;
    for (std::filesystem::recursive_directory_iterator iterator(
             voicingRoot,
             std::filesystem::directory_options::skip_permission_denied,
             error), end;
         iterator != end && !error; iterator.increment(error)) {
        if (!iterator->is_regular_file(error)) continue;
        const auto updated = iterator->last_write_time(error);
        if (!error) revision = std::max(revision, updated);
    }
    return revision;
}

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
            entry.config.audioVoicingFamily = entry.family;
            entry.config.audioVoicingKey = file.stem().string();
            const auto voicing = loadAudioVoicing(rootDirectory, entry.family,
                entry.config.audioVoicingKey);
            if (voicing)
                entry.config.audioVoicing = voicing.voicing;
            else
                result.errors.push_back(file.filename().string()
                    + " voicing: " + voicing.error);
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

EngineCatalogSelectionResult selectSingleEngineCatalogEntry(
    const std::vector<EngineCatalogEntry>& entries,
    std::string_view selector) {
    EngineCatalogSelectionResult result;
    if (selector.empty()) return result;

    const auto lowercase = [](std::string_view text) {
        std::string lowered(text);
        std::transform(lowered.begin(), lowered.end(), lowered.begin(),
            [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
        return lowered;
    };
    const auto requested = lowercase(selector);
    const auto addIfMissing = [](auto& matches, const EngineCatalogEntry& entry) {
        if (std::find(matches.begin(), matches.end(), &entry) == matches.end())
            matches.push_back(&entry);
    };

    for (const auto& entry : entries) {
        const auto key = lowercase(entry.config.audioVoicingKey);
        const auto stem = lowercase(entry.sourcePath.stem().string());
        const auto name = lowercase(entry.config.name);
        if (requested == key || requested == stem || requested == name)
            addIfMissing(result.matches, entry);
    }

    if (!result.matches.empty()) {
        result.exactMatch = true;
    } else {
        for (const auto& entry : entries) {
            const auto key = lowercase(entry.config.audioVoicingKey);
            const auto stem = lowercase(entry.sourcePath.stem().string());
            const auto name = lowercase(entry.config.name);
            if (key.find(requested) != std::string::npos
                    || stem.find(requested) != std::string::npos
                    || name.find(requested) != std::string::npos)
                addIfMissing(result.matches, entry);
        }
    }

    if (result.matches.size() == 1) {
        result.status = EngineCatalogSelectionStatus::unique;
        result.entry = result.matches.front();
    } else if (!result.matches.empty()) {
        result.status = EngineCatalogSelectionStatus::ambiguous;
    }
    return result;
}

} // namespace enginelab
