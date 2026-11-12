// Anti-lag (calibration `antilag.*`): a turbocharged petrol engine's tables
// carry it, off; off, the tables with its settings run the engine bit for bit
// as the tables without them; armed, a lift-off from full boost keeps the
// throttle open, fires late and cuts a share of the sparks with their fuel
// kept, the manifold burns what the cylinders send it, and the turbo keeps
// more of its speed and boost than without it.

#include <enginelab/calibration/CalibrationTypes.hpp>
#include <enginelab/calibration/EcuCalibration.hpp>
#include <enginelab/calibration/EcuCalibrationKeys.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/simulation/DynoAbsorberController.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace {

using namespace enginelab;

void require(bool condition, const std::string& message) {
    if (condition) return;
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
}

constexpr double frameSeconds = 1.0 / 240.0;

EngineConfig catalogueEngine(const std::string& name) {
    const auto catalog = loadEngineCatalog(ENGINELAB_CATALOG_ROOT);
    for (const auto& entry : catalog.entries)
        if (entry.config.name.find(name) != std::string::npos) return entry.config;
    require(false, "the " + name + " is in the catalogue");
    return {};
}

constexpr std::string_view antiLagKeys[] = {
    calibration::keys::antiLagEnabled, calibration::keys::antiLagMinimumRpm, calibration::keys::antiLagAir,
    calibration::keys::antiLagRetard, calibration::keys::antiLagLambda, calibration::keys::antiLagCut,
};

struct Rig final {
    explicit Rig(EngineConfig engine)
        : config(std::move(engine)), exhaust(ExhaustGraph::makeForEngine(config)),
          simulator(std::make_unique<EngineSimulator>(config, ecu, physics, events, exhaust)) {
        config = simulator->config();
        ecu.initialiseCalibration(config);
    }
    EngineConfig config;
    SimpleEcuModel ecu;
    SimplifiedGasolinePhysics physics;
    FourStrokeEventGenerator events;
    ExhaustGraph exhaust;
    std::unique_ptr<EngineSimulator> simulator;

    void set(std::string_view key, double value) {
        auto draft = calibration::makeDraft(*ecu.calibrationStore()->snapshot());
        const auto* found = draft.find(key);
        require(found != nullptr, "a turbocharged petrol engine's tables carry " + std::string(key));
        auto entry = *found;
        std::get<calibration::ScalarCalibration>(entry).value = value;
        draft.set(std::move(entry));
        require(ecu.calibrationStore()->publish(draft).published, std::string(key) + " publishes within its range");
    }
    void eraseAntiLag() {
        auto draft = calibration::makeDraft(*ecu.calibrationStore()->snapshot());
        for (const auto key : antiLagKeys) require(draft.erase(key), "the anti-lag settings can be taken out");
        require(ecu.calibrationStore()->publish(draft).published, "tables without anti-lag publish");
    }
};

struct Sample final {
    double rpm { 0.0 };
    double shaftRpm { 0.0 };
    double manifoldKpa { 0.0 };
    double boostRatio { 0.0 };
    double torqueNm { 0.0 };
};

struct LiftOff final {
    std::vector<Sample> trace;
    Sample before;
    Sample after;
    double reactionKw { 0.0 };
    double misfireShare { 0.0 };
    bool activeSeen { false };
    bool armedSeen { false };
};

/** Started, full throttle held at `rpm` by the dyno for `spool` seconds,
 *  then the throttle shut and the absorber let go for `lift` seconds. */
LiftOff liftOff(Rig& rig, double rpm, double spool, double lift) {
    for (int frame = 0; frame < static_cast<int>(1.5 / frameSeconds); ++frame) {
        EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = frame * frameSeconds < 1.1;
        controls.throttle = 0.2;
        static_cast<void>(rig.simulator->step(frameSeconds, controls));
    }
    require(rig.simulator->state().rpm > 500.0, rig.config.name + " starts");
    DynoAbsorberController absorber(rig.config);
    absorber.reset(rig.simulator->state().rpm, rig.simulator->state().torqueNm);
    LiftOff result;
    const auto sample = [](const EngineState& state) {
        return Sample { state.rpm, state.forcedInductionShaftSpeedRpm, state.manifoldPressureKpa,
                        state.boostPressureRatio, state.torqueNm };
    };
    const auto spoolFrames = static_cast<int>(spool / frameSeconds);
    const auto liftFrames = static_cast<int>(lift / frameSeconds);
    std::uint64_t misfiresAtLift = 0;
    for (int frame = 0; frame < spoolFrames + liftFrames; ++frame) {
        EngineControls controls;
        controls.ignitionEnabled = true;
        const auto lifted = frame >= spoolFrames;
        controls.throttle = lifted ? 0.0 : 1.0;
        const auto brake = absorber.advance(frameSeconds, rpm, rig.simulator->state()).brakeTorqueNm;
        controls.dynamometerTorqueNm = lifted ? 0.0 : brake;
        if (frame == spoolFrames) {
            result.before = sample(rig.simulator->state());
            misfiresAtLift = rig.simulator->state().misfireEventCount;
        }
        const auto step = rig.simulator->step(frameSeconds, controls);
        result.trace.push_back(sample(step.state));
        if (lifted) {
            result.reactionKw += step.state.exhaustAfterfireHeatReleaseKw / liftFrames;
            result.activeSeen = result.activeSeen || step.state.antiLagActive;
            result.armedSeen = result.armedSeen || step.state.antiLagArmed;
        }
    }
    const auto& state = rig.simulator->state();
    result.after = sample(state);
    // Firings over the lift: a four-stroke fires every cylinder once in two
    // turns; the share of them counted as misfires.
    const auto firings = std::max(1.0, lift * 0.5 * (0.5 * (result.before.rpm + state.rpm)) / 60.0
        * static_cast<double>(rig.config.cylinders.size()));
    result.misfireShare = static_cast<double>(state.misfireEventCount - misfiresAtLift) / firings;
    return result;
}

void tablesCarryItOff() {
    const auto jz = catalogueEngine("2JZ");
    const auto tables = calibration::makeDefaultEcuCalibration(jz);
    const auto* enabled = tables.find(calibration::keys::antiLagEnabled);
    require(enabled != nullptr && std::get<calibration::ScalarCalibration>(*enabled).value == 0.0,
            "a turbocharged petrol engine's tables carry anti-lag, off");
    for (const auto key : antiLagKeys)
        require(tables.find(key) != nullptr, "and every anti-lag setting: " + std::string(key));
    const auto k20 = catalogueEngine("K20");
    require(calibration::makeDefaultEcuCalibration(k20).find(calibration::keys::antiLagEnabled) == nullptr,
            "an engine without a turbo has no anti-lag");
    // Older tables get the settings added, off.
    auto older = calibration::makeDefaultEcuCalibration(jz);
    for (const auto key : antiLagKeys) require(older.erase(key), "a setting can be taken out");
    calibration::addMissingDefaultSettings(older, jz);
    require(older.find(calibration::keys::antiLagCut) != nullptr
                && std::get<calibration::ScalarCalibration>(*older.find(calibration::keys::antiLagEnabled)).value
                    == 0.0,
            "older tables get anti-lag added, off");
    std::cout << "tables: the 2JZ carries anti-lag, off; the K20 none\n";
}

void offIsTheEngineWithoutIt() {
    const auto jz = catalogueEngine("2JZ");
    auto with = std::make_unique<Rig>(jz);
    auto without = std::make_unique<Rig>(jz);
    without->eraseAntiLag();
    const auto a = liftOff(*with, 5'000.0, 2.0, 1.0);
    const auto b = liftOff(*without, 5'000.0, 2.0, 1.0);
    require(a.trace.size() == b.trace.size(), "the same frames");
    for (std::size_t i = 0; i < a.trace.size(); ++i)
        require(a.trace[i].rpm == b.trace[i].rpm && a.trace[i].shaftRpm == b.trace[i].shaftRpm
                    && a.trace[i].manifoldKpa == b.trace[i].manifoldKpa && a.trace[i].torqueNm == b.trace[i].torqueNm,
                "anti-lag off runs the engine bit for bit as tables without it (frame " + std::to_string(i) + ")");
    require(!a.armedSeen && !a.activeSeen, "off, it is neither armed nor working");
    std::cout << "off: " << a.trace.size() << " frames bit-identical to tables without anti-lag\n";
}

void armedKeepsTheTurboSpinning() {
    const auto jz = catalogueEngine("2JZ");
    auto off = std::make_unique<Rig>(jz);
    auto armed = std::make_unique<Rig>(jz);
    armed->set(calibration::keys::antiLagEnabled, 1.0);
    // The coast falls fast with no load: work down to 2,000 rpm so the
    // whole lift-off is inside its window.
    armed->set(calibration::keys::antiLagMinimumRpm, 2'000.0);
    off->set(calibration::keys::antiLagMinimumRpm, 2'000.0);
    constexpr double lift = 1.0;
    const auto a = liftOff(*off, 5'000.0, 3.0, lift);
    const auto b = liftOff(*armed, 5'000.0, 3.0, lift);
    std::cout << "spooled at 5,000 rpm: shaft " << a.before.shaftRpm << " / " << b.before.shaftRpm
              << " rpm, boost ratio " << a.before.boostRatio << " / " << b.before.boostRatio << '\n';
    std::cout << "after " << lift << " s lifted, off: engine " << a.after.rpm << " rpm, shaft " << a.after.shaftRpm
              << " rpm, manifold " << a.after.manifoldKpa << " kPa, manifold reaction " << a.reactionKw
              << " kW, misfire share " << a.misfireShare << '\n';
    std::cout << "after " << lift << " s lifted, armed: engine " << b.after.rpm << " rpm, shaft " << b.after.shaftRpm
              << " rpm, manifold " << b.after.manifoldKpa << " kPa, manifold reaction " << b.reactionKw
              << " kW, misfire share " << b.misfireShare << '\n';
    require(std::abs(a.before.shaftRpm - b.before.shaftRpm) < 0.02 * a.before.shaftRpm,
            "at full throttle, armed anti-lag changes nothing");
    require(b.armedSeen && b.activeSeen && !a.activeSeen, "armed, it works once the foot is off");
    require(b.misfireShare > 0.2, "a share of the sparks is cut");
    require(b.reactionKw > 1.0 && b.reactionKw > 10.0 * std::max(a.reactionKw, 0.01),
            "the manifold burns what the cylinders send it");
    // Measured: 129,322 rpm against 96,303 (x1.34) and 101.6 kPa against 17.1
    // after 1 s, 11.1 kW burning in the manifold, 35 % of the sparks cut.
    require(b.after.shaftRpm > 1.2 * a.after.shaftRpm, "the turbo keeps much more of its speed");
    require(b.after.manifoldKpa > a.after.manifoldKpa + 20.0, "and the manifold more of its pressure");
}

} // namespace

int main() {
    tablesCarryItOff();
    offIsTheEngineWithoutIt();
    armedKeepsTheTurboSpinning();
    std::cout << "PASS: anti-lag off is the engine without it; armed it keeps the turbo spinning\n";
    return 0;
}
