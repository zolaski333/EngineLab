#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/foundation/EngineTypes.hpp>
#include <enginelab/runtime/DrivelineModel.hpp>
#include <enginelab/serialization/JsonEngineSerializer.hpp>
#include <enginelab/serialization/YamlEngineSerializer.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <numbers>
#include <string_view>

namespace {
void require(bool condition, std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

enginelab::EngineConfig launchConfig(enginelab::DrivenAxleLayout layout) {
    auto config = enginelab::makeDefaultInlineFour();
    config.vehicle.massKg = 1'200.0;
    config.vehicle.dragCoefficient = 0.30;
    config.vehicle.frontalAreaM2 = 2.0;
    config.vehicle.tireRadiusM = 0.30;
    config.vehicle.rollingResistanceCoefficient = 0.0;
    config.vehicle.tireFrictionCoefficient = 1.0;
    config.vehicle.drivenAxleLayout = layout;
    config.vehicle.drivenAxleWeightFraction = 0.50;
    config.vehicle.wheelbaseM = 2.50;
    config.vehicle.centerOfGravityHeightM = 0.52;
    config.transmission.gearRatios = { 3.20 };
    config.transmission.finalDriveRatio = 4.0;
    config.transmission.maxClutchTorqueNm = 2'000.0;
    config.transmission.shiftDurationSeconds = 0.01;
    config.transmission.drivelineEfficiency = 0.95;
    return config;
}

enginelab::DrivelineOutput launch(enginelab::DrivenAxleLayout layout,
                                  bool reverse = false) {
    const auto config = launchConfig(layout);
    enginelab::DrivelineModel model(config);
    model.requestGear(reverse ? -2 : 0);

    enginelab::EngineState engine;
    engine.rpm = 5'000.0;
    engine.angularVelocityRadPerSecond =
        engine.rpm * 2.0 * std::numbers::pi / 60.0;
    engine.torqueNm = 800.0;
    engine.throttle = 1.0;

    enginelab::DrivelineOutput output;
    for (int step = 0; step < 600; ++step)
        output = model.advance(0.001, engine, 0.0, 1.0, 0.0);
    return output;
}

const enginelab::EngineConfig& findEngine(
    const enginelab::EngineCatalogLoadResult& catalog,
    std::string_view nameFragment) {
    const auto found = std::find_if(catalog.entries.begin(),
        catalog.entries.end(), [&](const auto& entry) {
            return entry.config.name.find(nameFragment)
                != std::string::npos;
        });
    require(found != catalog.entries.end(),
        "expected engine must exist in the catalogue");
    return found->config;
}
} // namespace

int main() {
    constexpr double gravityMps2 = 9.80665;
    constexpr double massKg = 1'200.0;
    constexpr double staticDrivenNormalN = 0.5 * massKg * gravityMps2;

    const auto front = launch(enginelab::DrivenAxleLayout::front);
    const auto rear = launch(enginelab::DrivenAxleLayout::rear);
    const auto all = launch(enginelab::DrivenAxleLayout::all);
    const auto rearReverse =
        launch(enginelab::DrivenAxleLayout::rear, true);

    std::cout << "layout,direction,accel_mps2,normal_n,tire_n,speed_mps,limited\n";
    const auto print = [](std::string_view layout,
                          std::string_view direction,
                          const enginelab::DrivelineOutput& value) {
        std::cout << layout << ',' << direction << ','
                  << value.longitudinalAccelerationMps2 << ','
                  << value.drivenAxleNormalForceN << ','
                  << value.tireForceN << ',' << value.vehicleSpeedMps
                  << ',' << (value.tractionLimited ? "yes" : "no")
                  << '\n';
    };
    print("front", "forward", front);
    print("rear", "forward", rear);
    print("all", "forward", all);
    print("rear", "reverse", rearReverse);

    require(front.tractionLimited && rear.tractionLimited
            && all.tractionLimited && rearReverse.tractionLimited,
        "the witnesses must actually exercise the tyre friction limit");
    require(front.longitudinalAccelerationMps2 > 0.0
            && rear.longitudinalAccelerationMps2 > 0.0
            && rearReverse.longitudinalAccelerationMps2 < 0.0,
        "the launch directions must produce signed longitudinal acceleration");
    require(front.drivenAxleNormalForceN < staticDrivenNormalN,
        "forward acceleration must unload a front driven axle");
    require(rear.drivenAxleNormalForceN > staticDrivenNormalN,
        "forward acceleration must load a rear driven axle");
    require(rearReverse.drivenAxleNormalForceN < staticDrivenNormalN,
        "reverse acceleration must unload a rear driven axle");
    require(rear.tireForceN > front.tireForceN * 1.25
            && rear.vehicleSpeedMps > front.vehicleSpeedMps * 1.20,
        "load transfer must materially separate otherwise identical FWD and RWD launches");
    require(std::abs(all.drivenAxleNormalForceN
                     - massKg * gravityMps2) < 1.0,
        "all-wheel drive must retain the sum of front and rear normal loads");

    auto roundTripConfig = launchConfig(
        enginelab::DrivenAxleLayout::front);
    roundTripConfig.vehicle.wheelbaseM = 2.73;
    roundTripConfig.vehicle.centerOfGravityHeightM = 0.49;
    const enginelab::JsonEngineSerializer json;
    const enginelab::YamlEngineSerializer yaml;
    const auto jsonResult = json.decode(json.encode(roundTripConfig));
    const auto yamlResult = yaml.decode(yaml.encode(roundTripConfig));
    require(jsonResult && yamlResult
            && jsonResult.config->vehicle.drivenAxleLayout
                == enginelab::DrivenAxleLayout::front
            && yamlResult.config->vehicle.drivenAxleLayout
                == enginelab::DrivenAxleLayout::front
            && std::abs(jsonResult.config->vehicle.wheelbaseM - 2.73)
                < 1.0e-9
            && std::abs(yamlResult.config->vehicle
                .centerOfGravityHeightM - 0.49) < 1.0e-9,
        "schema 5 JSON/YAML must preserve vehicle load-transfer geometry");

    auto invalid = roundTripConfig;
    invalid.vehicle.centerOfGravityHeightM =
        invalid.vehicle.wheelbaseM;
    require(enginelab::validateEngineConfig(invalid).has_value(),
        "validation must reject a centre of gravity at or beyond the wheelbase");

    const auto catalog =
        enginelab::loadEngineCatalog(ENGINELAB_CATALOG_ROOT);
    // The exact count matters and is not redundant with `errors.empty()`: a file
    // the loader SKIPS rather than rejects produces no error, so only the count
    // catches an engine silently vanishing from the catalogue. Bump it when a
    // catalogue entry is deliberately added -- 15 covers the 14 base engines plus
    // the CP2 full-system listening variant (engines/15_cp2_full_system_like).
    require(catalog.errors.empty() && catalog.entries.size() == 15,
        "the full schema-5 catalogue and vehicle parts must load");
    require(findEngine(catalog, "K20A").vehicle.drivenAxleLayout
                == enginelab::DrivenAxleLayout::front
            && findEngine(catalog, "Audi I5").vehicle.drivenAxleLayout
                == enginelab::DrivenAxleLayout::all
            && findEngine(catalog, "Aircooled").vehicle.drivenAxleLayout
                == enginelab::DrivenAxleLayout::rear
            && findEngine(catalog, "VW EA288").vehicle.drivenAxleLayout
                == enginelab::DrivenAxleLayout::front,
        "catalogue engines must select their authored driveline layout");

    std::cout << "PASS: longitudinal load transfer, persistence and catalogue layouts\n";
    return 0;
}
