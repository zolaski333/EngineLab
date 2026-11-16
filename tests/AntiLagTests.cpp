// Anti-lag (calibration `antilag.*`): a turbocharged petrol engine's tables
// carry it, off; off, the tables with its settings run the engine bit for bit
// as the tables without them. Armed (rally), a lift-off from full boost keeps
// the throttle open by a feedback on the engine's torque: the engine still
// slows (it never gains speed), it brakes about the share of its own drag it
// is asked to keep, and the turbo keeps more of its speed than without it.
// Its exits (a speed, a time, an exhaust temperature, neutral or the clutch)
// let it go, each said by its blocker bit; launch mode works only while
// launch control holds the engine on the line. The exhaust's fuel balance is
// printed: where the fuel it sends burns.

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
#include <functional>
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
    calibration::keys::antiLagMode, calibration::keys::antiLagBrakingKept, calibration::keys::antiLagAggressiveness,
    calibration::keys::antiLagMaximumRpm, calibration::keys::antiLagMaximumSeconds,
    calibration::keys::antiLagExhaustLimit,
};

std::uint32_t bit(AntiLagBlocker blocker) { return static_cast<std::uint32_t>(blocker); }

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
    /** Armed, working down to 2,000 rpm (an unloaded coast falls fast). */
    void arm() {
        set(calibration::keys::antiLagEnabled, 1.0);
        set(calibration::keys::antiLagMinimumRpm, 2'000.0);
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
    /** The fuel balance over the lift, mg/s: injected, sent unburnt into the
     *  exhaust, burnt there (and before the turbine), out of the tailpipe. */
    double injectedMgPerS { 0.0 };
    double exhaustInMgPerS { 0.0 };
    double burntMgPerS { 0.0 };
    double burntBeforeTurbineMgPerS { 0.0 };
    double tailpipeMgPerS { 0.0 };
    double misfireShare { 0.0 };
    /** Over the lift: the highest engine speed, the mean cycle torque from
     *  0.4 s on (the manifold emptied), the controller's target and opening,
     *  and how long it worked. */
    double peakRpm { 0.0 };
    double meanTorqueNm { 0.0 };
    double targetTorqueNm { 0.0 };
    double authority { 0.0 };
    double activeSeconds { 0.0 };
    /** The hottest readings over the lift: the cylinders' mean exhaust and
     *  the probe at the turbine inlet. */
    double peakExhaustC { 0.0 };
    double peakTurbineInletC { 0.0 };
    std::uint32_t blockersAtEnd { 0U };
    bool activeAtEnd { false };
    bool activeSeen { false };
    bool armedSeen { false };
};

struct LiftOptions final {
    double rpm { 5'000.0 };
    double spool { 3.0 };
    double lift { 2.0 };
    /** Neutral or the clutch out over the whole lift. */
    bool decoupled { false };
    /** Called with each lifted frame's state, from 0.4 s into the lift. */
    std::function<void(const EngineState&)> settled {};
    /** Called with every lifted frame's state and its time into the lift. */
    std::function<void(const EngineState&, double)> lifted {};
};

/** Started, full throttle held at `rpm` by the dyno for `spool` seconds,
 *  then the throttle shut and the absorber let go for `lift` seconds. */
LiftOff liftOff(Rig& rig, const LiftOptions& options) {
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
    const auto spoolFrames = static_cast<int>(options.spool / frameSeconds);
    const auto liftFrames = static_cast<int>(options.lift / frameSeconds);
    const auto settledFrame = spoolFrames + static_cast<int>(0.4 / frameSeconds);
    std::uint64_t misfiresAtLift = 0;
    auto settledFrames = 0;
    for (int frame = 0; frame < spoolFrames + liftFrames; ++frame) {
        EngineControls controls;
        controls.ignitionEnabled = true;
        const auto lifted = frame >= spoolFrames;
        controls.throttle = lifted ? 0.0 : 1.0;
        controls.drivelineDecoupled = lifted && options.decoupled;
        const auto brake = absorber.advance(frameSeconds, options.rpm, rig.simulator->state()).brakeTorqueNm;
        controls.dynamometerTorqueNm = lifted ? 0.0 : brake;
        if (frame == spoolFrames) {
            result.before = sample(rig.simulator->state());
            misfiresAtLift = rig.simulator->state().misfireEventCount;
        }
        const auto step = rig.simulator->step(frameSeconds, controls);
        result.trace.push_back(sample(step.state));
        if (lifted) {
            const auto& state = step.state;
            if (options.lifted) options.lifted(state, (frame - spoolFrames + 1) * frameSeconds);
            result.reactionKw += state.exhaustAfterfireHeatReleaseKw / liftFrames;
            result.injectedMgPerS += state.fuelFlowGramsPerSecond * 1'000.0 / liftFrames;
            result.exhaustInMgPerS += state.exhaustFuelInMgPerSecond / liftFrames;
            result.burntMgPerS += state.exhaustAfterfireFuelBurnMgPerSecond / liftFrames;
            result.burntBeforeTurbineMgPerS += state.exhaustFuelBurnBeforeTurbineMgPerSecond / liftFrames;
            result.tailpipeMgPerS += state.tailpipeFuelMgPerSecond / liftFrames;
            result.activeSeen = result.activeSeen || state.antiLagActive;
            result.armedSeen = result.armedSeen || state.antiLagArmed;
            result.peakRpm = std::max(result.peakRpm, state.rpm);
            result.peakExhaustC = std::max(result.peakExhaustC, state.exhaustTemperatureC);
            result.peakTurbineInletC = std::max(result.peakTurbineInletC, state.turbineInletTemperatureC);
            if (state.antiLagActive) result.activeSeconds += frameSeconds;
            if (frame >= settledFrame) {
                if (options.settled) options.settled(state);
                ++settledFrames;
                // The last whole cycle's brake torque: the instantaneous one,
                // read once a frame, aliases the cut firings (2JZ at 0 % kept:
                // +-300 N.m a sample, its mean +3.7 or -5.0 N.m by where the
                // frames land, while the speed held within 60 rpm).
                result.meanTorqueNm += state.cycleAveragedTorqueNm;
                result.targetTorqueNm += state.antiLagTargetTorqueNm;
                result.authority += state.antiLagAuthority;
            }
        }
    }
    const auto& state = rig.simulator->state();
    result.after = sample(state);
    result.blockersAtEnd = state.antiLagBlockers;
    result.activeAtEnd = state.antiLagActive;
    if (settledFrames > 0) {
        result.meanTorqueNm /= settledFrames;
        result.targetTorqueNm /= settledFrames;
        result.authority /= settledFrames;
    }
    // Firings over the lift: a four-stroke fires every cylinder once in two
    // turns; the share of them counted as misfires.
    const auto firings = std::max(1.0, options.lift * 0.5 * (0.5 * (result.before.rpm + state.rpm)) / 60.0
        * static_cast<double>(rig.config.cylinders.size()));
    result.misfireShare = static_cast<double>(state.misfireEventCount - misfiresAtLift) / firings;
    return result;
}

void print(const std::string& name, const LiftOff& run) {
    std::cout << name << ": engine " << run.before.rpm << " -> peak " << run.peakRpm << " -> " << run.after.rpm
              << " rpm, shaft " << run.before.shaftRpm << " -> " << run.after.shaftRpm << " rpm, manifold "
              << run.after.manifoldKpa << " kPa, torque " << run.meanTorqueNm << " N.m (target "
              << run.targetTorqueNm << "), opening " << run.authority << ", worked " << run.activeSeconds
              << " s, blockers " << run.blockersAtEnd << ", reaction " << run.reactionKw << " kW, misfires "
              << run.misfireShare << ", hottest exhaust " << run.peakExhaustC << " C (cylinders), "
              << run.peakTurbineInletC << " C (turbine inlet)" << std::endl;
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
    require(older.find(calibration::keys::antiLagExhaustLimit) != nullptr
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
    const auto a = liftOff(*with, { 5'000.0, 2.0, 1.0 });
    const auto b = liftOff(*without, { 5'000.0, 2.0, 1.0 });
    require(a.trace.size() == b.trace.size(), "the same frames");
    for (std::size_t i = 0; i < a.trace.size(); ++i)
        require(a.trace[i].rpm == b.trace[i].rpm && a.trace[i].shaftRpm == b.trace[i].shaftRpm
                    && a.trace[i].manifoldKpa == b.trace[i].manifoldKpa && a.trace[i].torqueNm == b.trace[i].torqueNm,
                "anti-lag off runs the engine bit for bit as tables without it (frame " + std::to_string(i) + ")");
    require(!a.armedSeen && !a.activeSeen, "off, it is neither armed nor working");
    std::cout << "off: " << a.trace.size() << " frames bit-identical to tables without anti-lag\n";
}

void rallyKeepsItsShareOfTheBraking() {
    const auto jz = catalogueEngine("2JZ");
    auto offRig = std::make_unique<Rig>(jz);
    const auto off = liftOff(*offRig, {});
    print("off", off);
    // The controller, with no exhaust limit: at the default 1,050 C the
    // 2JZ's turbine inlet reaches it within ~1 s of a lift, and each exit
    // brakes with the whole drag (exitsLetItGo tests it).
    auto defaultRig = std::make_unique<Rig>(jz);
    defaultRig->arm();
    defaultRig->set(calibration::keys::antiLagExhaustLimit, 0.0);
    const auto armed = liftOff(*defaultRig, {});
    print("armed, 50 % kept", armed);
    std::cout << "fuel over the lift, armed, mg/s: injected " << armed.injectedMgPerS << ", into the exhaust unburnt "
              << armed.exhaustInMgPerS << ", burnt there " << armed.burntMgPerS << " (before the turbine "
              << armed.burntBeforeTurbineMgPerS << "), out of the tailpipe " << armed.tailpipeMgPerS << std::endl;
    auto holdRig = std::make_unique<Rig>(jz);
    holdRig->arm();
    holdRig->set(calibration::keys::antiLagExhaustLimit, 0.0);
    holdRig->set(calibration::keys::antiLagBrakingKept, 0.0);
    holdRig->set(calibration::keys::antiLagAggressiveness, 100.0);
    const auto hold = liftOff(*holdRig, {});
    print("armed, 0 % kept, full aggressiveness", hold);
    auto brakeRig = std::make_unique<Rig>(jz);
    brakeRig->arm();
    brakeRig->set(calibration::keys::antiLagExhaustLimit, 0.0);
    brakeRig->set(calibration::keys::antiLagBrakingKept, 90.0);
    const auto brake = liftOff(*brakeRig, {});
    print("armed, 90 % kept", brake);

    require(armed.burntBeforeTurbineMgPerS <= armed.burntMgPerS * 1.001 + 1.0e-9
                && armed.exhaustInMgPerS <= armed.injectedMgPerS * 1.05,
            "the fuel balance adds up");
    require(std::abs(off.before.shaftRpm - armed.before.shaftRpm) < 0.02 * off.before.shaftRpm,
            "at full throttle, armed anti-lag changes nothing");
    require(armed.armedSeen && armed.activeSeen && !off.activeSeen, "armed, it works once the foot is off");
    require(armed.activeAtEnd && armed.blockersAtEnd == 0U, "still working 2 s into the lift, nothing holding it");
    require(armed.misfireShare > 0.2, "a share of the sparks is cut");
    // The owner's report: it made the engine gain speed and cancelled the
    // engine braking. Each kept share still slows the engine: never past
    // the peak the lift itself gives (the manifold emptying), and lower
    // after 2 s than at the lift.
    for (const auto* run : { &armed, &hold, &brake }) {
        require(run->peakRpm <= off.peakRpm + 25.0, "anti-lag never adds speed over the lift's own peak");
        require(run->after.rpm < run->before.rpm, "the engine is slower 2 s after the lift");
        // 0 % kept holds the speed: its torque is zero within 2 % of the drag.
        require(run->meanTorqueNm < 0.02 * std::abs(off.meanTorqueNm), "its torque never drives the engine");
    }
    // The braking follows the share asked for: more kept, more braking, and
    // the target is met within a fifth of the drag.
    require(brake.meanTorqueNm < armed.meanTorqueNm && armed.meanTorqueNm < hold.meanTorqueNm,
            "more braking kept brakes more");
    for (const auto* run : { &armed, &brake })
        require(std::abs(run->meanTorqueNm - run->targetTorqueNm) < 0.2 * std::abs(off.meanTorqueNm),
                "the engine's torque meets the braking asked for");
    require(armed.after.shaftRpm > 1.1 * off.after.shaftRpm, "the turbo keeps more of its speed");
    // Braking costs turbo speed where it closes the air (90 % kept: the
    // manifold near 60 kPa). At 0 and 50 % both runs end at or above their
    // loaded speed, where the wastegate holds the shaft, so their order is
    // within 1 % and flips with any change to the loaded state (the spark
    // map's boost retard removed: 129,331 / 128,483 became 131,163 / 131,448).
    require(brake.after.shaftRpm < armed.after.shaftRpm && brake.after.shaftRpm < hold.after.shaftRpm,
            "more braking kept, less turbo speed kept");
    require(hold.after.shaftRpm > 0.99 * armed.after.shaftRpm, "keeping no braking costs no turbo speed");
    // The owner's report: nothing like the real thing. A rally anti-lag
    // keeps boost with the foot off and the turbo near its loaded speed,
    // the charges it cuts burning in the manifold. Before: the manifold
    // fell to 37 kPa, the shaft to 64 %, 1 % of the fuel burnt before the
    // turbine (journal 2026-11-14).
    const auto lastSecond = [](const LiftOff& run) {
        auto sum = 0.0;
        const auto last = std::min<std::size_t>(run.trace.size(), static_cast<std::size_t>(1.0 / frameSeconds));
        for (std::size_t index = run.trace.size() - last; index < run.trace.size(); ++index)
            sum += run.trace[index].manifoldKpa;
        return last > 0 ? sum / static_cast<double>(last) : 0.0;
    };
    std::cout << "last second's manifold: off " << lastSecond(off) << " kPa, armed " << lastSecond(armed)
              << " kPa; shaft kept: off " << off.after.shaftRpm / off.before.shaftRpm << ", armed "
              << armed.after.shaftRpm / armed.before.shaftRpm << "; burnt before the turbine "
              << armed.burntBeforeTurbineMgPerS / armed.injectedMgPerS << " of the fuel" << std::endl;
    for (const auto* run : { &armed, &hold }) {
        require(lastSecond(*run) > 110.0, "the foot off, the manifold keeps boost");
        require(run->after.shaftRpm > 0.8 * run->before.shaftRpm, "the turbo keeps 80 % of its loaded speed");
    }
    require(armed.burntBeforeTurbineMgPerS > 0.1 * armed.injectedMgPerS,
            "a tenth of the fuel or more burns in the manifold, before the turbine");
}

void exitsLetItGo() {
    const auto jz = catalogueEngine("2JZ");
    {
        auto rig = std::make_unique<Rig>(jz);
        rig->arm();
        rig->set(calibration::keys::antiLagMaximumSeconds, 0.5);
        const auto run = liftOff(*rig, {});
        print("max time 0.5 s", run);
        require(!run.activeAtEnd && (run.blockersAtEnd & bit(AntiLagBlocker::timeUp)) != 0U
                    && run.activeSeconds < 0.6,
                "its maximum time lets it go, until the throttle re-arms it");
    }
    {
        auto rig = std::make_unique<Rig>(jz);
        rig->arm();
        rig->set(calibration::keys::antiLagExhaustLimit, 100.0);
        const auto run = liftOff(*rig, {});
        require(!run.activeSeen && (run.blockersAtEnd & bit(AntiLagBlocker::exhaustTooHot)) != 0U,
                "an exhaust over its limit holds it");
    }
    {
        // The limit is read at the turbine inlet, where the cut charges
        // burn: 850 C is under it after a full-boost pull, over the
        // cylinders' mean (journal 2026-11-14: ~420-570 C, which never let go).
        auto rig = std::make_unique<Rig>(jz);
        rig->arm();
        rig->set(calibration::keys::antiLagExhaustLimit, 850.0);
        // Working, then held by the heat on the next frame. Not the state
        // at the lift's end: the 50 C band cycles it on and off (on at
        // 1.1 s, off at 1.5, on at 1.6, off near 1.9-2.0 s), so the last
        // frame's phase shifts with any combustion change (journal
        // 2026-11-14).
        LiftOptions options {};
        auto wasActive = false;
        auto letGoByHeat = false;
        options.lifted = [&](const EngineState& state, double) {
            letGoByHeat = letGoByHeat
                || (wasActive && !state.antiLagActive
                    && (state.antiLagBlockers & bit(AntiLagBlocker::exhaustTooHot)) != 0U);
            wasActive = state.antiLagActive;
        };
        const auto run = liftOff(*rig, options);
        print("exhaust limit 850 C", run);
        require(run.peakExhaustC < 850.0 && run.peakTurbineInletC > 850.0,
                "the cylinders' mean stays under 850 C, the turbine inlet goes over it");
        require(run.activeSeconds < 1.9 && letGoByHeat,
                "the turbine inlet over its limit lets it go");
    }
    {
        // Letting go on boost, at the default limit with no braking kept:
        // the engine speed while it is let go, up to 0.4 s after each exit,
        // against its speed then. The plate it held closes over ~0.3 s, and the boosted
        // manifold fired normal charges until it had: the 2JZ gained 530 rpm
        // (journal 2026-11-14).
        auto rig = std::make_unique<Rig>(jz);
        rig->arm();
        rig->set(calibration::keys::antiLagBrakingKept, 0.0);
        rig->set(calibration::keys::antiLagAggressiveness, 100.0);
        auto exits = 0;
        auto wasActive = false;
        auto exitRpm = 0.0;
        auto exitSeconds = -1.0;
        auto surgeRpm = 0.0;
        LiftOptions options;
        options.lifted = [&](const EngineState& state, double seconds) {
            if (wasActive && !state.antiLagActive
                    && (state.antiLagBlockers & bit(AntiLagBlocker::exhaustTooHot)) != 0U) {
                ++exits;
                exitRpm = state.rpm;
                exitSeconds = seconds;
            }
            if (!state.antiLagActive && exitSeconds >= 0.0 && seconds - exitSeconds <= 0.4)
                surgeRpm = std::max(surgeRpm, state.rpm - exitRpm);
            wasActive = state.antiLagActive;
        };
        const auto run = liftOff(*rig, options);
        print("default exhaust limit, 0 % kept", run);
        std::cout << "default exhaust limit, 0 % kept: " << exits << " exits, the engine up at most " << surgeRpm
                  << " rpm while let go" << std::endl;
        require(exits > 0, "on boost with no braking kept, the turbine inlet reaches the default limit");
        require(surgeRpm < 50.0, "letting go adds no speed: the overrun fuel cut starts at once");
    }
    {
        auto rig = std::make_unique<Rig>(jz);
        rig->arm();
        rig->set(calibration::keys::antiLagMaximumRpm, 2'500.0);
        const auto run = liftOff(*rig, {});
        require(!run.activeSeen && (run.blockersAtEnd & bit(AntiLagBlocker::aboveMaximumRpm)) != 0U,
                "above its maximum speed it lets go");
    }
    {
        auto rig = std::make_unique<Rig>(jz);
        rig->arm();
        LiftOptions options;
        options.decoupled = true;
        const auto run = liftOff(*rig, options);
        print("neutral", run);
        require(run.activeSeen && !run.activeAtEnd && (run.blockersAtEnd & bit(AntiLagBlocker::decoupled)) != 0U
                    && run.activeSeconds < 1.1,
                "in neutral it lets go after a gear change's time");
    }
    std::cout << "exits: time, exhaust temperature, speed and neutral each let it go\n";
}

void launchBuildsBoostOnTheLine() {
    // Launch control at 4,000 rpm, first gear, standing still, foot down.
    const auto hold = [](Rig& rig, double seconds) {
        for (int frame = 0; frame < static_cast<int>(1.5 / frameSeconds); ++frame) {
            EngineControls controls;
            controls.ignitionEnabled = true;
            controls.starterEngaged = frame * frameSeconds < 1.1;
            controls.throttle = 0.2;
            static_cast<void>(rig.simulator->step(frameSeconds, controls));
        }
        auto activeSeconds = 0.0;
        for (int frame = 0; frame < static_cast<int>(seconds / frameSeconds); ++frame) {
            EngineControls controls;
            controls.ignitionEnabled = true;
            controls.throttle = 1.0;
            controls.engagedGear = 0;
            controls.vehicleSpeedMps = 0.0;
            controls.drivelineDecoupled = true;
            if (rig.simulator->step(frameSeconds, controls).state.antiLagActive) activeSeconds += frameSeconds;
        }
        return activeSeconds;
    };
    const auto jz = catalogueEngine("2JZ");
    auto plain = std::make_unique<Rig>(jz);
    plain->set(calibration::keys::launchLimit, 4'000.0);
    static_cast<void>(hold(*plain, 3.0));
    auto rally = std::make_unique<Rig>(jz);
    rally->set(calibration::keys::launchLimit, 4'000.0);
    rally->arm();
    const auto rallySeconds = hold(*rally, 3.0);
    auto launch = std::make_unique<Rig>(jz);
    launch->set(calibration::keys::launchLimit, 4'000.0);
    launch->arm();
    launch->set(calibration::keys::antiLagMode, 1.0);
    const auto launchSeconds = hold(*launch, 3.0);
    const auto& a = plain->simulator->state();
    const auto& b = launch->simulator->state();
    std::cout << "launch control at 4,000 rpm for 3 s: engine " << a.rpm << " / " << b.rpm << " rpm, shaft "
              << a.forcedInductionShaftSpeedRpm << " / " << b.forcedInductionShaftSpeedRpm << " rpm, manifold "
              << a.manifoldPressureKpa << " / " << b.manifoldPressureKpa << " kPa (launch control alone / with "
              << "anti-lag in launch mode); worked " << launchSeconds << " s" << std::endl;
    require(rallySeconds == 0.0 && (rally->simulator->state().antiLagBlockers & bit(AntiLagBlocker::throttle)) != 0U,
            "rally mode does nothing with the foot down");
    require(launchSeconds > 2.0 && b.antiLagBlockers == 0U, "launch mode works while launch control holds");
    require(b.forcedInductionShaftSpeedRpm > 1.1 * a.forcedInductionShaftSpeedRpm,
            "and the turbo spins faster on the line");
    require(b.rpm < 4'400.0, "launch control still holds the speed");
}

void cutTurnsOverTheCylinders() {
    // Its cut, as many firings as asked, comes round every cylinder: a
    // sequence over the engine's firings locked a fixed set at 90 % (2JZ
    // cylinders 5, 6 and 4 never lit, an I5 lit only cylinder 1).
    for (const auto* name : { "2JZ", "I5" }) {
        auto rig = std::make_unique<Rig>(catalogueEngine(name));
        rig->arm();
        const auto cylinders = rig->config.cylinders.size();
        std::vector<int> cycles(cylinders, 0), fired(cylinders, 0);
        std::vector<double> phase(cylinders, -1.0), misfired(cylinders, 0.0);
        LiftOptions options;
        options.settled = [&](const EngineState& state) {
            for (std::size_t i = 0; i < cylinders; ++i) {
                const auto& c = state.cylinderStates[i];
                // The frame before the cycle wraps holds this cycle's spark.
                if (phase[i] >= 0.0 && c.cyclePhaseDegrees < phase[i]) {
                    ++cycles[i];
                    if (misfired[i] < 0.5) ++fired[i];
                }
                phase[i] = c.cyclePhaseDegrees;
                misfired[i] = c.misfireProbability;
            }
        };
        const auto run = liftOff(*rig, options);
        print(std::string(name) + " cut by cylinder", run);
        auto total = 0.0;
        for (std::size_t i = 0; i < cylinders; ++i) total += static_cast<double>(fired[i]) / std::max(1, cycles[i]);
        const auto mean = total / static_cast<double>(cylinders);
        std::cout << "  lit share by cylinder:";
        for (std::size_t i = 0; i < cylinders; ++i)
            std::cout << ' ' << rig->config.cylinders[i].id << '=' << static_cast<double>(fired[i]) / std::max(1, cycles[i]);
        std::cout << std::endl;
        require(mean > 0.05, std::string(name) + ": some firings are lit");
        for (std::size_t i = 0; i < cylinders; ++i)
            require(static_cast<double>(fired[i]) / std::max(1, cycles[i]) > 0.5 * mean,
                    std::string(name) + ": every cylinder takes its turn to fire (cylinder "
                        + std::to_string(rig->config.cylinders[i].id) + ")");
    }
}

void pocketVariant() {
    // Measurement only: the same lift with the unlit charges carried as
    // pockets (the backfire profile's 40 ms) instead of mixed at once.
    auto jz = catalogueEngine("2JZ");
    jz.exhaustAfterfire.pocketMixingTimeSeconds = 0.040;
    auto armed = std::make_unique<Rig>(jz);
    armed->arm();
    const auto b = liftOff(*armed, {});
    print("pockets 40 ms", b);
    std::cout << "fuel mg/s injected " << b.injectedMgPerS << ", into the exhaust " << b.exhaustInMgPerS << ", burnt "
              << b.burntMgPerS << " (before the turbine " << b.burntBeforeTurbineMgPerS << "), tailpipe "
              << b.tailpipeMgPerS << std::endl;
}

void leverSweep() {
    // Measurement only: at 50 % braking kept, one lever at a time.
    struct Variant final {
        const char* name;
        double spark;
        double cut;
        double air;
    };
    for (const auto& variant : { Variant { "default (25 deg ATDC, 33 % cut, 40 % air)", 25.0, 33.0, 40.0 },
                                 Variant { "spark 45 deg ATDC", 45.0, 33.0, 40.0 },
                                 Variant { "cut 60 %", 25.0, 60.0, 40.0 },
                                 Variant { "air 20 %", 25.0, 33.0, 20.0 } }) {
        auto rig = std::make_unique<Rig>(catalogueEngine("2JZ"));
        rig->arm();
        rig->set(calibration::keys::antiLagRetard, variant.spark);
        rig->set(calibration::keys::antiLagCut, variant.cut);
        rig->set(calibration::keys::antiLagAir, variant.air);
        print(variant.name, liftOff(*rig, {}));
    }
}

void mixtureBalance(const std::string& engine, double coolantC) {
    // Measurement only: where anti-lag's mixture leaves its lambda. Per
    // cylinder over the settled lift: the ECU's air reference (trapped
    // oxygen), the fresh air the valve delivered, the fuel asked and
    // delivered, the trim, and the cycle's chamber AFR.
    auto config = catalogueEngine(engine);
    if (coolantC > 0.0) {
        config.thermal.regulated = true;
        config.thermal.regulatedCoolantTemperatureC = coolantC;
    }
    auto rig = std::make_unique<Rig>(config);
    rig->arm();
    const auto cylinders = rig->config.cylinders.size();
    std::vector<double> trapped(cylinders), delivered(cylinders), requested(cylinders), injected(cylinders), metered(cylinders),
        afr(cylinders), trim(cylinders);
    double target = 0.0, correction = 0.0, coolant = 0.0;
    auto frames = 0;
    LiftOptions options;
    options.settled = [&](const EngineState& state) {
        ++frames;
        target += state.targetAirFuelRatio;
        correction += state.ecuFuelCorrection;
        coolant += state.coolantTemperatureC;
        for (std::size_t i = 0; i < cylinders; ++i) {
            const auto& c = state.cylinderStates[i];
            trapped[i] += c.trappedFreshAirMassMg;
            delivered[i] += c.deliveredFreshAirMassMgPerCycle;
            requested[i] += c.requestedFuelMgPerCycle;
            injected[i] += c.deliveredFuelMgPerCycle;
            metered[i] += c.meteredFuelMgPerCycle;
            afr[i] += c.airFuelRatio;
            trim[i] += c.closedLoopFuelTrim;
        }
    };
    const auto run = liftOff(*rig, options);
    print(engine + " mixture", run);
    const auto n = static_cast<double>(std::max(1, frames));
    std::cout << "target AFR " << target / n << ", ECU fuel correction " << correction / n << ", coolant "
              << coolant / n << " C\n";
    for (std::size_t i = 0; i < cylinders; ++i)
        std::cout << "  cylinder " << rig->config.cylinders[i].id << ": trapped air " << trapped[i] / n
                  << " mg, delivered air " << delivered[i] / n << " mg, fuel asked " << requested[i] / n
                  << " mg, metered " << metered[i] / n << " mg, in the chamber " << injected[i] / n << " mg, trim " << trim[i] / n << ", AFR " << afr[i] / n
                  << ", trapped/asked " << trapped[i] / std::max(1.0e-9, requested[i]) << ", delivered/asked "
                  << delivered[i] / std::max(1.0e-9, requested[i]) << std::endl;
}

} // namespace

int main(int argc, char** argv) {
    if (argc > 1 && std::string_view(argv[1]) == "--trace") {
        // Measurement only: a lift every 50 ms, at a braking kept (argv[2]).
        auto rig = std::make_unique<Rig>(catalogueEngine("2JZ"));
        rig->arm();
        if (argc > 2) rig->set(calibration::keys::antiLagBrakingKept, std::atof(argv[2]));
        if (argc > 3) rig->set(calibration::keys::antiLagExhaustLimit, std::atof(argv[3]));
        if (argc > 4) rig->set(calibration::keys::antiLagAggressiveness, std::atof(argv[4]));
        LiftOptions options;
        auto frame = 0;
        options.lifted = [&](const EngineState& state, double seconds) {
            if (frame++ % 12 != 0) return;
            std::cout << seconds << " s: " << state.rpm << " rpm, torque " << state.torqueNm << ", manifold "
                      << state.manifoldPressureKpa << " kPa, shaft " << state.forcedInductionShaftSpeedRpm
                      << ", active " << state.antiLagActive << ", blockers " << state.antiLagBlockers
                      << ", opening " << state.antiLagAuthority << ", turbine inlet " << state.turbineInletTemperatureC
                      << " C, cylinders " << state.exhaustTemperatureC << " C, DFCO " << state.ecuFuelEnabled
                      << std::endl;
        };
        print("trace", liftOff(*rig, options));
        return 0;
    }
    if (argc > 1 && std::string_view(argv[1]) == "--cut") {
        cutTurnsOverTheCylinders();
        return 0;
    }
    if (argc > 1 && std::string_view(argv[1]) == "--mixture") {
        const auto engine = argc > 2 ? std::string(argv[2]) : std::string("2JZ");
        mixtureBalance(engine, argc > 3 ? std::atof(argv[3]) : 0.0);
        return 0;
    }
    if (argc > 1 && std::string_view(argv[1]) == "--pockets") {
        pocketVariant();
        return 0;
    }
    if (argc > 1 && std::string_view(argv[1]) == "--levers") {
        leverSweep();
        return 0;
    }
    tablesCarryItOff();
    offIsTheEngineWithoutIt();
    rallyKeepsItsShareOfTheBraking();
    cutTurnsOverTheCylinders();
    exitsLetItGo();
    launchBuildsBoostOnTheLine();
    std::cout << "PASS: anti-lag off is the engine without it; armed it keeps its share of the braking, never adds "
                 "speed, lets go at its exits and builds boost on the line in launch mode\n";
    return 0;
}
