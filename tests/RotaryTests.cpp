// The Wankel rotary: a face sweeps 3 sqrt(3) R e b over 270 deg of the
// eccentric shaft, its gas torque is p dV/da, a rotor's three faces share its
// ports, and the catalogue's 13B-class twin-rotor idles and makes its maker's
// torque and power.

#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/gasdynamics/ExhaustNetworkLayout.hpp>
#include <enginelab/physics/MechanicalKinematics.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <numbers>
#include <string_view>

namespace {
using namespace enginelab;
constexpr double frameSeconds = 1.0 / 240.0;

void require(bool condition, std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

[[nodiscard]] EngineConfig thirteenB() {
    const auto catalogue = loadEngineCatalog(ENGINELAB_CATALOG_ROOT);
    for (const auto& error : catalogue.errors) {
        std::cerr << error << '\n';
    }
    require(catalogue.errors.empty(), "the catalogue loads");
    const auto found = std::find_if(catalogue.entries.begin(), catalogue.entries.end(),
        [](const auto& entry) { return entry.config.cycle == EngineCycle::rotary; });
    require(found != catalogue.entries.end(), "the catalogue has a rotary");
    return found->config;
}

void faceFollowsTheWankelGeometry(const EngineConfig& config) {
    require(!validateEngineConfig(config).has_value(), "the 13B validates");
    const auto vd = rotaryFaceDisplacementLitres(config.rotary);
    require(std::abs(vd - 0.6547) < 0.0005, "a 13B face sweeps 654.7 cc (3 sqrt(3) R e b)");
    const auto reference = buildEngineKinematicsReference(config);
    const auto& face = config.cylinders.front();
    const auto vmin = vd / (face.compressionRatio - 1.0);
    const auto volumeAt = [&](double fromTdc) {
        return evaluateCylinderKinematics(config, reference, 0, face.crankOffsetDegrees + fromTdc, 100.0)
            .chamberVolumeLitres;
    };
    require(std::abs(volumeAt(0.0) - vmin) < 1.0e-6, "the face is smallest at its firing top dead centre");
    require(std::abs(volumeAt(270.0) - (vmin + vd)) < 1.0e-6, "and largest 270 deg of the shaft later");
    require(std::abs(volumeAt(540.0) - vmin) < 1.0e-6, "and smallest again at its gas-exchange top");
    // dV/da = Vd/3 sin(2a/3): the lever is Vd/3 per radian half way down.
    const auto lever = evaluateCylinderKinematics(config, reference, 0, face.crankOffsetDegrees + 135.0, 100.0);
    const auto areaM2 = std::numbers::pi * 0.25 * face.boreMm * face.boreMm * 1.0e-6;
    require(std::abs(lever.displacementDerivativeMPerRadian * areaM2 - vd * 1.0e-3 / 3.0) < 1.0e-8,
        "the gas torque's lever is dV/da = Vd / 3 at 135 deg");
}

void validationKeepsARotorWhole(const EngineConfig& config) {
    auto diesel = config;
    diesel.fuel = FuelType::diesel;
    require(validateEngineConfig(diesel).has_value(), "a diesel rotary is refused");
    auto wrongBore = config;
    wrongBore.cylinders[1].boreMm += 1.0;
    require(validateEngineConfig(wrongBore).has_value(), "a face's bore follows its rotor");
    auto split = config;
    ExhaustPathConfig front;
    front.id = 1;
    front.geometry = config.exhaust;
    front.cylinderIds = { 1, 2, 4 };
    ExhaustPathConfig rear = front;
    rear.id = 2;
    rear.cylinderIds = { 3, 5, 6 };
    split.exhaustPaths = { front, rear };
    require(validateEngineConfig(split).has_value(), "a rotor's faces cannot leave by two exhaust paths");
}

void aRotorsFacesShareItsExhaustPort(const EngineConfig& config) {
    const auto graph = ExhaustGraph::makeForEngine(config);
    const auto layout = gasdynamics::ExhaustNetworkLayout::compile(
        graph, gasdynamics::realtimeExhaustFeedbackDiscretisation());
    require(layout.valid(), "the rotary's exhaust compiles");
    require(layout.cylinderPorts().size() == 6, "six faces, six ports");
    std::array<std::size_t, 6> junctionOf {};
    for (const auto& port : layout.cylinderPorts()) {
        require(port.networkEndpoint.type == gasdynamics::ExhaustEndpointType::junction,
            "a face opens onto its rotor's port passage");
        junctionOf[port.cylinderId - 1] = port.networkEndpoint.elementIndex;
    }
    require(junctionOf[0] == junctionOf[1] && junctionOf[1] == junctionOf[2]
            && junctionOf[3] == junctionOf[4] && junctionOf[4] == junctionOf[5]
            && junctionOf[0] != junctionOf[3],
        "faces 1-3 share the front rotor's passage, 4-6 the rear's");
}

struct Rig final {
    explicit Rig(EngineConfig engine)
        : config(std::move(engine)), exhaust(ExhaustGraph::makeForEngine(config)),
          simulator(std::make_unique<EngineSimulator>(config, ecu, physics, events, exhaust)) {}
    EngineConfig config;
    SimpleEcuModel ecu;
    SimplifiedGasolinePhysics physics;
    FourStrokeEventGenerator events;
    ExhaustGraph exhaust;
    std::unique_ptr<EngineSimulator> simulator;
};

struct Held final {
    double rpm { 0.0 };
    double torqueNm { 0.0 };
    double intakeRunnerSpreadKpa { 0.0 };
};

#if defined(_MSC_VER)
#pragma optimize("", off)
#endif
/** Cranked, then `throttle` held against a dyno at `dynoRpm` (none at zero);
 *  means over the last 2 s of 8. */
[[nodiscard]] Held held(const EngineConfig& config, double throttle, double dynoRpm) {
    auto rig = std::make_unique<Rig>(config);
    constexpr int frames = 8 * 240;
    constexpr int firstMeasured = frames - 2 * 240;
    constexpr double weight = 1.0 / static_cast<double>(frames - firstMeasured);
    Held result;
    auto speed = 0.0;
    for (auto frame = 0; frame < frames; ++frame) {
        const auto t = frame * frameSeconds;
        EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = t < 1.5;
        controls.throttle = t < 2.0 ? 0.0 : throttle;
        if (t >= 2.0 && dynoRpm > 0.0) controls.dynamometerTorqueNm = std::max(0.0, 2.0 * (speed - dynoRpm));
        const auto state = rig->simulator->step(frameSeconds, controls).state;
        speed = state.rpm;
        if (frame < firstMeasured) continue;
        result.rpm += state.rpm * weight;
        result.torqueNm += state.torqueNm * weight;
        // The three faces of a rotor read one port passage.
        const auto& faces = state.cylinderStates;
        result.intakeRunnerSpreadKpa = std::max({ result.intakeRunnerSpreadKpa,
            std::abs(faces[0].intakeRunnerChargePressureKpa - faces[1].intakeRunnerChargePressureKpa),
            std::abs(faces[0].intakeRunnerChargePressureKpa - faces[2].intakeRunnerChargePressureKpa) });
    }
    return result;
}
#if defined(_MSC_VER)
#pragma optimize("", on)
#endif

void thirteenBMakesItsMakersFigures(const EngineConfig& config) {
    const auto idle = held(config, 0.0, 0.0);
    std::cout << "13B idle " << idle.rpm << " rpm\n";
    require(idle.rpm > 0.6 * config.idleRpm && idle.rpm < 1.6 * config.idleRpm, "the 13B idles");
    // Mazda: 211 N.m at 5,500 rpm, 184 kW at 8,500 (VISION.md: within ~15 %).
    const auto torque = held(config, 1.0, 5'500.0);
    std::cout << "13B WOT " << torque.rpm << " rpm " << torque.torqueNm << " N.m\n";
    require(torque.torqueNm > 0.85 * 211.0 && torque.torqueNm < 1.15 * 211.0, "211 N.m at 5,500 rpm, within 15 %");
    require(torque.intakeRunnerSpreadKpa < 1.0e-9, "a rotor's three faces share one intake runner");
    const auto power = held(config, 1.0, 8'500.0);
    const auto kw = power.torqueNm * power.rpm * std::numbers::pi / 30.0 / 1'000.0;
    std::cout << "13B WOT " << power.rpm << " rpm " << power.torqueNm << " N.m " << kw << " kW\n";
    require(kw > 0.85 * 184.0 && kw < 1.15 * 184.0, "184 kW at 8,500 rpm, within 15 %");
}
} // namespace

int main() {
    const auto config = thirteenB();
    faceFollowsTheWankelGeometry(config);
    validationKeepsARotorWhole(config);
    aRotorsFacesShareItsExhaustPort(config);
    thirteenBMakesItsMakersFigures(config);
    std::cout << "rotary tests passed\n";
    return EXIT_SUCCESS;
}
