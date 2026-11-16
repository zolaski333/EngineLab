// The nitrous kit (NitrousConfig): fitted and not spraying it changes
// nothing, bit for bit; a 100 hp shot adds about 100 hp at full throttle on
// two different engines (a wet kit on the K20A, a dry one on the LS3); the
// jet passes 2.27 kg/min per 100 hp at 900 psi, a bottle empties at that rate
// and an unlimited one never does; the ECU's lambda loop holds its trims
// while it sprays, so a lean kit runs lean (and knocks harder) instead of
// being corrected; and the engine file keeps the kit.

#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/serialization/JsonEngineSerializer.hpp>
#include <enginelab/serialization/YamlEngineSerializer.hpp>
#include <enginelab/simulation/DynoAbsorberController.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <numbers>
#include <string>

namespace {

void require(bool condition, const std::string& message) {
    if (condition) return;
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
}

constexpr double frameSeconds = 1.0 / 240.0;

enginelab::EngineConfig catalogueEngine(const std::string& name) {
    const auto catalog = enginelab::loadEngineCatalog(ENGINELAB_CATALOG_ROOT);
    for (const auto& entry : catalog.entries)
        if (entry.config.name.find(name) != std::string::npos) return entry.config;
    require(false, "the " + name + " is in the catalogue");
    return {};
}

struct Rig final {
    explicit Rig(const enginelab::EngineConfig& engine)
        : config(engine), exhaust(enginelab::ExhaustGraph::makeForEngine(engine)),
          simulator(std::make_unique<enginelab::EngineSimulator>(
              config, ecu, physics, events, exhaust)) {}
    enginelab::EngineConfig config;
    enginelab::SimpleEcuModel ecu;
    enginelab::SimplifiedGasolinePhysics physics;
    enginelab::FourStrokeEventGenerator events;
    enginelab::ExhaustGraph exhaust;
    std::unique_ptr<enginelab::EngineSimulator> simulator;
};

struct Hold final {
    double powerKw { 0.0 };
    double torqueNm { 0.0 };
    double flowKgPerSecond { 0.0 };
    double bottleBeforeKg { 0.0 };
    double bottleAfterKg { 0.0 };
    double bottlePressureKpa { 0.0 };
    double manifoldKpa { 0.0 };
    double knock { 0.0 };
    double airFuelRatio { 0.0 };
    double trimBefore { 0.0 };
    double trimAfter { 0.0 };
    double peakBar { 0.0 };
    double fuelFlowGramsPerSecond { 0.0 };
};

/** Started, then full throttle held at `rpm` by the dyno: `settle` seconds
 *  without nitrous, then `spray` seconds armed (or not), the absorber's
 *  torque averaged over the last half of them. */
Hold holdAt(const enginelab::EngineConfig& config, double rpm, bool armed, double settle = 3.0,
            double spray = 3.0) {
    auto rig = std::make_unique<Rig>(config);
    for (int frame = 0; frame < static_cast<int>(1.5 / frameSeconds); ++frame) {
        enginelab::EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = frame * frameSeconds < 1.1;
        controls.throttle = 0.2;
        static_cast<void>(rig->simulator->step(frameSeconds, controls));
    }
    require(rig->simulator->state().rpm > 500.0, config.name + " starts");
    enginelab::DynoAbsorberController absorber(config);
    absorber.reset(rig->simulator->state().rpm, rig->simulator->state().torqueNm);
    Hold hold;
    double torque = 0.0;
    double knock = 0.0;
    double afr = 0.0;
    int samples = 0;
    const auto settleFrames = static_cast<int>(settle / frameSeconds);
    const auto sprayFrames = static_cast<int>(spray / frameSeconds);
    for (int frame = 0; frame < settleFrames + sprayFrames; ++frame) {
        enginelab::EngineControls controls;
        controls.ignitionEnabled = true;
        controls.throttle = 1.0;
        controls.nitrousArmed = armed && frame >= settleFrames;
        const auto brake = absorber.advance(frameSeconds, rpm, rig->simulator->state()).brakeTorqueNm;
        controls.dynamometerTorqueNm = brake;
        if (frame == settleFrames) {
            hold.bottleBeforeKg = rig->simulator->state().nitrousBottleMassKg;
            hold.trimBefore = rig->simulator->state().cylinderStates[0].closedLoopFuelTrim;
        }
        const auto result = rig->simulator->step(frameSeconds, controls);
        if (frame >= settleFrames + sprayFrames / 2) {
            torque += brake;
            knock += result.state.knockLevel;
            afr += result.state.airFuelRatio;
            hold.flowKgPerSecond += result.state.nitrousMassFlowKgPerSecond;
            hold.bottlePressureKpa += result.state.nitrousBottlePressureKpa;
            hold.manifoldKpa += result.state.manifoldPressureKpa;
            hold.fuelFlowGramsPerSecond += result.state.fuelFlowGramsPerSecond;
            hold.peakBar = std::max(hold.peakBar, result.state.peakCylinderPressureBar);
            ++samples;
        }
    }
    const auto& state = rig->simulator->state();
    hold.torqueNm = torque / samples;
    hold.powerKw = hold.torqueNm * state.rpm * std::numbers::pi / 30.0 / 1'000.0;
    hold.flowKgPerSecond /= samples;
    hold.bottlePressureKpa /= samples;
    hold.manifoldKpa /= samples;
    hold.fuelFlowGramsPerSecond /= samples;
    hold.knock = knock / samples;
    hold.airFuelRatio = afr / samples;
    hold.bottleAfterKg = state.nitrousBottleMassKg;
    hold.trimAfter = state.cylinderStates[0].closedLoopFuelTrim;
    require(std::abs(state.rpm - rpm) < 0.05 * rpm, config.name + " is held at its speed");
    return hold;
}

enginelab::EngineConfig withKit(enginelab::EngineConfig config, enginelab::NitrousKitType type) {
    config.nitrous.fitted = true;
    config.nitrous.type = type;
    config.nitrous.pushButton = false;
    config.nitrous.minimumRpm = 2'000.0;
    return config;
}

void checkFittedIsNothing(const enginelab::EngineConfig& plain) {
    // Fitted, unarmed: the same engine, frame for frame.
    const auto fitted = withKit(plain, enginelab::NitrousKitType::wet);
    auto a = std::make_unique<Rig>(plain);
    auto b = std::make_unique<Rig>(fitted);
    for (int frame = 0; frame < static_cast<int>(3.0 / frameSeconds); ++frame) {
        enginelab::EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = frame * frameSeconds < 1.1;
        controls.throttle = frame * frameSeconds < 2.0 ? 0.2 : 1.0;
        const auto x = a->simulator->step(frameSeconds, controls);
        const auto y = b->simulator->step(frameSeconds, controls);
        require(x.state.rpm == y.state.rpm && x.state.torqueNm == y.state.torqueNm
                && x.state.manifoldPressureKpa == y.state.manifoldPressureKpa
                && x.state.airFuelRatio == y.state.airFuelRatio,
            "a kit fitted and not armed changes nothing (frame " + std::to_string(frame) + ")");
        require(!y.state.nitrousSpraying && y.state.nitrousBottleMassKg == fitted.nitrous.bottleCapacityKg,
            "unarmed, the kit stays full");
    }
    std::cout << "fitted, not armed == no kit over 720 frames\n";
}

/** A kit that does not spray says why: every condition that holds it. */
void checkBlockers() {
    using enginelab::NitrousBlocker;
    const auto bit = [](NitrousBlocker blocker) { return static_cast<std::uint32_t>(blocker); };
    require(!enginelab::NitrousConfig {}.pushButton, "a new kit sprays armed at full throttle, no push button");
    auto config = withKit(catalogueEngine("K20"), enginelab::NitrousKitType::wet);
    config.nitrous.pushButton = true;
    config.nitrous.minimumRpm = 5'000.0; // over what 5 % throttle reaches unloaded
    auto rig = std::make_unique<Rig>(config);
    enginelab::EngineState state;
    for (int frame = 0; frame < static_cast<int>(1.5 / frameSeconds); ++frame) {
        enginelab::EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = frame * frameSeconds < 1.1;
        controls.throttle = 0.05;
        state = rig->simulator->step(frameSeconds, controls).state;
    }
    require(state.nitrousBlockers == (bit(NitrousBlocker::notArmed) | bit(NitrousBlocker::pushButton)
                                      | bit(NitrousBlocker::throttle) | bit(NitrousBlocker::belowMinimumRpm)),
            "disarmed, button up, part throttle, at idle: all four say so (" + std::to_string(state.nitrousBlockers)
                + ")");
    enginelab::EngineControls controls;
    controls.ignitionEnabled = true;
    controls.throttle = 1.0;
    controls.nitrousArmed = true;
    controls.nitrousButton = true;
    state = rig->simulator->step(frameSeconds, controls).state;
    require(state.nitrousBlockers == bit(NitrousBlocker::belowMinimumRpm),
            "armed, held, full throttle: only the engine speed holds it");
    std::cout << "a kit that does not spray says why\n";
}

void checkShot(const std::string& name, enginelab::NitrousKitType type, double rpm) {
    const auto plain = catalogueEngine(name);
    const auto kit = withKit(plain, type);
    const auto without = holdAt(kit, rpm, false);
    const auto with = holdAt(kit, rpm, true);
    const auto gain = with.powerKw - without.powerKw;
    std::cout << name << (type == enginelab::NitrousKitType::wet ? " wet" : " dry") << " 100 hp shot at " << rpm
              << " rpm: " << without.powerKw << " kW -> " << with.powerKw << " kW (+" << gain << " kW, "
              << gain / 0.7457 << " hp); flow " << with.flowKgPerSecond * 60.0 << " kg/min at "
              << with.bottlePressureKpa << " kPa into " << with.manifoldKpa << " kPa; AFR "
              << without.airFuelRatio << " -> " << with.airFuelRatio << "; knock " << without.knock << " -> "
              << with.knock << "; peak " << without.peakBar << " -> " << with.peakBar << " bar\n";
    // Tuners' and install sheets' figures spread 4-6 lb/min per 100 hp; the
    // shot is the kit's rating at the crank. Measured 2026-11-12: 77.3 hp
    // (K20, wet) and 75.6 hp (LS3, dry) at the rated flow, with the 2 deg per
    // 50 hp retard -- below the rating (journal). The bound catches a kit that
    // does nothing or burns its oxygen twice, not that gap.
    require(gain > 0.65 * 74.57 && gain < 1.25 * 74.57, name + ": a 100 hp shot adds about 100 hp");
    // The jet: 2.27 kg/min per 100 hp at 900 psi into the atmosphere, as the
    // square root of the drop.
    const auto expected = 2.27 / 60.0
        * std::sqrt(std::max(0.0, with.bottlePressureKpa - with.manifoldKpa)
                    / (enginelab::nitrousReferenceBottlePressureKpa - 101.325));
    require(std::abs(with.flowKgPerSecond / expected - 1.0) < 0.01, name + ": the jet passes its rated flow");
    require(with.bottleBeforeKg - with.bottleAfterKg > 0.9 * with.flowKgPerSecond * 3.0, name + ": the bottle empties");
}

/** A wet kit's own fuel is fuel used: it burnt uncounted, so the fuel flow
 *  and BSFC undercounted the spray (audit PG-02). A dry kit's fuel goes
 *  through the injectors, which were always counted: at the same shot the
 *  two use about the same fuel. (Against the unsprayed engine the delta
 *  is not the kit's fuel: the lambda loop leans the injectors under it.) */
void checkWetKitFuelCounted() {
    const auto plain = catalogueEngine("K20");
    const auto wet = holdAt(withKit(plain, enginelab::NitrousKitType::wet), 6'000.0, true);
    const auto dry = holdAt(withKit(plain, enginelab::NitrousKitType::dry), 6'000.0, true);
    const auto kit = withKit(plain, enginelab::NitrousKitType::wet);
    const auto& fuel = kit.fuelProperties;
    // 0.5 mol of O2 per mol of N2O, at the kit's lambda.
    const auto kitFuelGramsPerSecond = wet.flowKgPerSecond / 0.0440128 * 0.5
        / (fuel.oxygenMolesPerFuelMole * kit.nitrous.kitLambda) * fuel.molarMassGramsPerMole;
    std::cout << "K20 fuel flow sprayed: wet " << wet.fuelFlowGramsPerSecond << " g/s, dry "
              << dry.fuelFlowGramsPerSecond << " g/s (the wet kit's own " << kitFuelGramsPerSecond << " g/s)\n";
    require(std::abs(wet.fuelFlowGramsPerSecond - dry.fuelFlowGramsPerSecond) < 0.25 * kitFuelGramsPerSecond,
            "K20: the wet kit's fuel is counted in the fuel flow, as the dry kit's is");
}

void checkBottle() {
    auto config = withKit(catalogueEngine("K20"), enginelab::NitrousKitType::wet);
    config.nitrous.bottleCapacityKg = 0.5;
    const auto small = holdAt(config, 6'000.0, true, 3.0, 2.0);
    std::cout << "a 0.5 kg bottle after 2 s: " << small.bottleAfterKg << " kg\n";
    require(small.bottleAfterKg < 0.5 - 0.9 * 2.0 * 2.27 / 60.0, "a bottle runs down at the jet's flow");
    config.nitrous.unlimitedBottle = true;
    const auto unlimited = holdAt(config, 6'000.0, true, 3.0, 2.0);
    require(unlimited.bottleAfterKg == 0.5 && unlimited.flowKgPerSecond > 0.0, "an unlimited bottle never runs down");
}

void checkLeanKit() {
    // A dry kit whose ECU pulse is half what its oxygen needs: the lambda
    // loop holds, so the engine runs lean and knocks harder than with the
    // kit's proper fuel.
    auto config = withKit(catalogueEngine("K20"), enginelab::NitrousKitType::dry);
    const auto rich = holdAt(config, 6'000.0, true);
    config.nitrous.kitLambda = 2.0;
    const auto lean = holdAt(config, 6'000.0, true);
    std::cout << "dry kit at lambda 0.8 / 2.0: AFR " << rich.airFuelRatio << " / " << lean.airFuelRatio
              << ", knock " << rich.knock << " / " << lean.knock << ", trim " << lean.trimBefore << " -> "
              << lean.trimAfter << "\n";
    require(lean.airFuelRatio > rich.airFuelRatio + 1.0, "a lean kit runs lean: the ECU does not correct it");
    require(lean.trimAfter == lean.trimBefore, "the lambda loop holds its trim while the kit sprays");
}

void checkFiles() {
    auto config = withKit(catalogueEngine("K20"), enginelab::NitrousKitType::dry);
    config.nitrous.shotKw = 111.0;
    config.nitrous.unlimitedBottle = true;
    config.nitrous.kitLambda = 0.72;
    config.nitrous.maximumRpm = 7'800.0;
    config.nitrous.pushButton = true;
    const auto check = [&](const auto& serializer, const char* name) {
        const auto decoded = serializer.decode(serializer.encode(config));
        require(static_cast<bool>(decoded), std::string(name) + " reads the kit back: " + decoded.error);
        const auto& kit = decoded.config->nitrous;
        require(kit.fitted && kit.type == enginelab::NitrousKitType::dry && kit.shotKw == 111.0
                && kit.unlimitedBottle && kit.kitLambda == 0.72 && kit.maximumRpm == 7'800.0 && kit.pushButton,
            std::string(name) + " keeps the kit");
        const auto plain = serializer.encode(catalogueEngine("K20"));
        require(plain.find("nitrous") == std::string::npos, std::string(name) + ": an engine without a kit writes none");
    };
    check(enginelab::JsonEngineSerializer {}, "JSON");
    check(enginelab::YamlEngineSerializer {}, "YAML");
}

} // namespace

int main() {
    checkFittedIsNothing(catalogueEngine("K20"));
    checkBlockers();
    checkFiles();
    checkShot("K20", enginelab::NitrousKitType::wet, 6'000.0);
    checkShot("LS3", enginelab::NitrousKitType::dry, 4'500.0);
    checkBottle();
    checkWetKitFuelCounted();
    checkLeanKit();
    std::cout << "EngineLab.Nitrous PASS\n";
    return 0;
}
