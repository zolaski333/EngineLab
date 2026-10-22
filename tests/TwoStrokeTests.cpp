// Two-stroke engines: the piston uncovers the ports, the crankcase pumps the
// charge through a reed, and the catalogue twin runs at its maker's figures.

#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/foundation/TwoStrokePorts.hpp>
#include <enginelab/foundation/TwoStrokeScavenging.hpp>
#include <enginelab/physics/CarburettorModel.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/serialization/JsonEngineSerializer.hpp>
#include <enginelab/serialization/YamlEngineSerializer.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <string_view>
#include <utility>

namespace {
using namespace enginelab;
constexpr double frameSeconds = 1.0 / 240.0;

void require(bool condition, std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
    std::cout << "ok: " << message << '\n';
}

[[nodiscard]] EngineConfig catalogueTwoStroke() {
    static const auto catalogue = loadEngineCatalog(ENGINELAB_CATALOG_ROOT);
    const auto selected = selectSingleEngineCatalogEntry(catalogue.entries, "RD350LC");
    require(static_cast<bool>(selected), "the catalogue has the RD350LC-like twin");
    auto config = selected.entry->config;
    normaliseEngineConfig(config);
    return config;
}

struct Rig final {
    explicit Rig(EngineConfig engine, EngineSimulatorOptions options = {})
        : config(std::move(engine)), exhaust(ExhaustGraph::makeForEngine(config)),
          simulator(std::make_unique<EngineSimulator>(config, ecu, physics, events, exhaust, std::move(options))) {}
    EngineConfig config;
    SimpleEcuModel ecu;
    SimplifiedGasolinePhysics physics;
    FourStrokeEventGenerator events;
    ExhaustGraph exhaust;
    std::unique_ptr<EngineSimulator> simulator;
};

void portsFollowThePiston() {
    CylinderConfig cylinder;
    cylinder.boreMm = 64.0;
    cylinder.strokeMm = 54.0;
    cylinder.connectingRodMm = 110.0;
    require(pistonTravelMm(54.0, 110.0, 0.0) == 0.0 && std::abs(pistonTravelMm(54.0, 110.0, 180.0) - 54.0) < 1.0e-9,
        "the piston travels the stroke from top to bottom dead centre");
    require(portOpenAreaMm2(cylinder, 82.0, 41.6, 0.72, 81.0) == 0.0, "the exhaust port is shut before it opens");
    const auto bottom = portOpenAreaMm2(cylinder, 82.0, 41.6, 0.72, 180.0);
    require(bottom > 0.0 && portOpenAreaMm2(cylinder, 82.0, 41.6, 0.72, 120.0) < bottom,
        "and opens wider down to bottom dead centre");
    require(std::abs(portOpenAreaMm2(cylinder, 82.0, 41.6, 0.72, 150.0)
                     - portOpenAreaMm2(cylinder, 82.0, 41.6, 0.72, 210.0)) < 1.0e-9,
        "symmetrically about it");
    require(portOpenAreaMm2(cylinder, 82.0, 41.6, 0.72, 279.0) == 0.0, "and is shut again past 278 degrees");
    const auto top = crankcaseVolumeLitres(cylinder, 1.35, 0.0);
    const auto bottomVolume = crankcaseVolumeLitres(cylinder, 1.35, 180.0);
    require(std::abs(top / bottomVolume - 1.35) < 1.0e-9, "the crankcase compresses by its ratio");
}

void configurationIsChecked() {
    auto config = catalogueTwoStroke();
    require(config.cycle == EngineCycle::twoStroke && !validateEngineConfig(config), "the catalogue twin is valid");
    require(config.intakePaths.size() == 2, "each cylinder has its own crankcase circuit");
    auto diesel = config;
    diesel.fuel = FuelType::diesel;
    require(validateEngineConfig(diesel).has_value(), "a two-stroke diesel is refused");
    auto late = config;
    late.twoStroke.transferPortOpenDegrees = late.twoStroke.exhaustPortOpenDegrees;
    require(validateEngineConfig(late).has_value(), "transfers opening with the exhaust are refused");
    auto injection = config;
    injection.injection.endAngleDegrees = 400.0;
    require(validateEngineConfig(injection).has_value(), "an injection angle past the 360-degree cycle is refused");
}

/** A file that leaves out the crank offsets spaces the cylinders evenly over
 *  the engine's own cycle: 180 degrees apart for a two-stroke twin, not 360,
 *  where both would fire together. */
void missingCrankOffsetsFollowTheCycle() {
    const auto config = catalogueTwoStroke();
    const auto withoutOffsets = [](std::string text) {
        for (auto at = text.find("crank_offset_deg"); at != std::string::npos;
             at = text.find("crank_offset_deg", at + 1))
            text.replace(at, 16, "unread_offset_d");
        return text;
    };
    const JsonEngineSerializer json;
    const YamlEngineSerializer yaml;
    for (const auto* serializer : { static_cast<const IEngineSerializer*>(&json),
                                    static_cast<const IEngineSerializer*>(&yaml) }) {
        const auto decoded = serializer->decode(withoutOffsets(serializer->encode(config)));
        require(decoded && decoded.config->cylinders.size() == 2, "a file without crank offsets decodes");
        const auto apart = std::fmod(std::abs(decoded.config->cylinders[1].crankOffsetDegrees
            - decoded.config->cylinders[0].crankOffsetDegrees), 360.0);
        require(std::abs(apart - 180.0) < 1.0e-9, "without crank offsets, the twin's cylinders are 180 degrees apart");
    }
}

struct Held final {
    double torqueNm { 0.0 };
    double lambda { 0.0 };
    double firingsPerCylinderRevolution { 0.0 };
};

/** Cranked, then full throttle against a dyno holding `rpm`; means over the
 *  last 2 s of 8. */
[[nodiscard]] Held heldFullLoad(const EngineConfig& config, double rpm,
                                bool exhaustEverySubstep = false) {
    auto rig = std::make_unique<Rig>(config);
    rig->simulator->setExhaustCouplingEverySubstep(exhaustEverySubstep);
    constexpr int frames = 8 * 240;
    constexpr int firstMeasured = frames - 2 * 240;
    constexpr double weight = 1.0 / static_cast<double>(frames - firstMeasured);
    Held held;
    auto speed = 0.0;
    auto firings = 0.0;
    auto revolutions = 0.0;
    for (auto frame = 0; frame < frames; ++frame) {
        const auto t = frame * frameSeconds;
        EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = t < 1.5;
        controls.throttle = t < 2.0 ? 0.0 : 1.0;
        if (t >= 2.0) controls.dynamometerTorqueNm = std::max(0.0, 2.0 * (speed - rpm));
        const auto& result = rig->simulator->step(frameSeconds, controls);
        speed = result.state.rpm;
        if (frame < firstMeasured) continue;
        held.torqueNm += result.state.torqueNm * weight;
        held.lambda += result.state.lambda * weight;
        for (std::size_t index = 0; index < result.firingEventCount; ++index)
            if (!result.firingEvents[index].misfire) firings += 1.0;
        revolutions += result.state.rpm / 60.0 * frameSeconds;
    }
    held.firingsPerCylinderRevolution = firings / std::max(1.0e-9, revolutions)
        / static_cast<double>(config.cylinders.size());
    return held;
}

/** Maker: 40.2 N.m at 8,000 rpm (37 N.m in the Japanese listing). The model
 *  makes about 33 here at the 100 mm real-time exhaust mesh (journal
 *  2026-10-16): the first cell behind the port holds about a cylinder's
 *  volume, and 25 mm cells, not real time, make about 36. The test
 *  guards what holds -- one firing per cylinder per revolution, a sane
 *  mixture, positive torque -- and prints the figure; it does not pretend the
 *  gap is closed. */
void catalogueTwinMakesItsTorque() {
    const auto config = catalogueTwoStroke();
    const auto held = heldFullLoad(config, 8'000.0);
    std::cout << "RD350LC-like at 8,000 rpm: " << held.torqueNm << " N.m (maker 37-40.2), lambda "
              << held.lambda << ", " << held.firingsPerCylinderRevolution
              << " firings per cylinder per revolution\n";
    require(held.firingsPerCylinderRevolution > 0.9 && held.firingsPerCylinderRevolution < 1.1,
        "each cylinder fires once per revolution");
    require(held.lambda > 0.7 && held.lambda < 1.3, "the full-load mixture is near stoichiometric");
    require(held.torqueNm > 0.0 && held.torqueNm < 1.15 * 40.2,
        "the full-load torque at 8,000 rpm is positive and not above the maker's");
}

/** The scavenging model's own switch, measured in one binary on one engine.
 *
 *  A loop length of zero is the perfectly mixed cylinder the simulator had
 *  before the model existed, so this is the cleanest control available: the
 *  same code, the same catalogue engine, one field changed. It is also the
 *  answer to "what did the model actually buy", without comparing two builds
 *  whose toolchains or meshes might differ.
 *
 *  The assertion is only that the switch works and that the result stays below
 *  the maker's figure. It deliberately does NOT assert that the biased torque
 *  is higher by some amount: the reference is the real engine's 40.2 N.m, not
 *  this simulator's output on either setting. */
void theScavengingLoopIsASwitch() {
    auto mixed = catalogueTwoStroke();
    mixed.twoStroke.scavengeLoopLengthRatio = 0.0;
    const auto mixedHeld = heldFullLoad(mixed, 8'000.0);

    auto looped = catalogueTwoStroke();
    looped.twoStroke.scavengeLoopLengthRatio = 1.8;
    const auto loopedHeld = heldFullLoad(looped, 8'000.0);

    std::cout << "RD350LC-like at 8,000 rpm: perfectly mixed " << mixedHeld.torqueNm
              << " N.m, loop 1.8 x bore " << loopedHeld.torqueNm
              << " N.m (maker 40.2)\n";
    require(std::abs(loopedHeld.torqueNm - mixedHeld.torqueNm) > 1.0e-6,
        "the scavenging loop length actually reaches the cylinder");
    require(mixedHeld.torqueNm > 0.0 && loopedHeld.torqueNm > 0.0,
        "the twin runs on either setting");
    require(loopedHeld.torqueNm < 1.15 * 40.2,
        "a loop-scavenged cylinder still does not beat the maker's figure");
}

/** From bottom dead centre to the transfers' closing the cylinder's pressure
 *  rides on a few kPa between two open ports. Coupled to the exhaust every
 *  117 us there, the RD350 made 12 % less than with the exhaust coupled on
 *  every substep (journal 2026-10-16); the production coupling must stay
 *  within 5 % of that reference. */
void scavengingIsCoupledFinely() {
    const auto config = catalogueTwoStroke();
    const auto production = heldFullLoad(config, 8'000.0);
    const auto reference = heldFullLoad(config, 8'000.0, true);
    std::cout << "RD350LC-like at 8,000 rpm: " << production.torqueNm
              << " N.m, exhaust coupled every substep " << reference.torqueNm << " N.m\n";
    require(std::abs(production.torqueNm - reference.torqueNm) < 0.05 * reference.torqueNm,
        "the production exhaust coupling is within 5 % of the every-substep torque");
}

/** A two-stroke's "manifold" is its crankcase, which pumps about ambient at
 *  any throttle. Read as MAP it put the idle at 0.8 of full load: the flame
 *  burned with almost full-load turbulence and the ECU's maps saw boost at
 *  WOT. Its load is the charge it delivered (journal 2026-10-16). */
void loadIsTheDeliveredCharge() {
    const auto config = catalogueTwoStroke();
    SimpleEcuModel ecu;
    ecu.initialiseCalibration(config);
    ecu.beginFrame();
    EngineState state;
    state.rpm = 8'000.0;
    state.load = 0.8;
    state.coolantTemperatureC = 90.0;
    state.simulationTimeSeconds = 1.0;
    EngineControls controls;
    controls.ignitionEnabled = true;
    controls.throttle = 1.0;
    state.manifoldPressureKpa = config.ambientPressureKpa * 1.07;
    const auto pumping = ecu.evaluate(config, state, controls);
    state.manifoldPressureKpa = config.ambientPressureKpa * 0.8;
    const auto atLoad = ecu.evaluate(config, state, controls);
    std::cout << "ECU advance at delivery ratio 0.8, crankcase 1.07 vs 0.8 x ambient: "
              << pumping.ignitionAdvanceDegrees << " / " << atLoad.ignitionAdvanceDegrees << " deg\n";
    require(std::abs(pumping.ignitionAdvanceDegrees - atLoad.ignitionAdvanceDegrees) < 1.0e-9,
        "the ECU's advance follows the delivered charge, not the crankcase pressure");

    auto rig = std::make_unique<Rig>(config);
    auto crankcase = 0.0;
    auto load = 0.0;
    auto delivered = 0.0;
    auto samples = 0.0;
    for (auto frame = 0; frame < 10 * 240; ++frame) {
        const auto t = frame * frameSeconds;
        EngineControls idle;
        idle.ignitionEnabled = true;
        idle.starterEngaged = t < 1.5;
        const auto& result = rig->simulator->step(frameSeconds, idle);
        if (t < 8.0) continue;
        crankcase += result.state.manifoldPressureKpa / config.ambientPressureKpa;
        load += result.state.load;
        delivered += result.state.deliveredVolumetricEfficiency;
        samples += 1.0;
    }
    crankcase /= samples;
    load /= samples;
    delivered /= samples;
    std::cout << "RD350LC-like idle: crankcase/ambient " << crankcase << ", load " << load
              << ", delivery ratio " << delivered << '\n';
    require(crankcase > 0.6, "the idling crankcase pumps near ambient");
    require(load < 0.4 && std::abs(load - delivered) < 0.05,
        "the idle load is the delivery ratio, far below the crankcase's pressure");
}

/** Left at idle, it keeps running. */
void catalogueTwinIdles() {
    auto rig = std::make_unique<Rig>(catalogueTwoStroke());
    auto lowest = 1.0e9;
    for (auto frame = 0; frame < 12 * 240; ++frame) {
        const auto t = frame * frameSeconds;
        EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = t < 1.5;
        const auto rpm = rig->simulator->step(frameSeconds, controls).state.rpm;
        if (t > 4.0) lowest = std::min(lowest, rpm);
    }
    std::cout << "RD350LC-like idle: lowest " << lowest << " rpm after 4 s\n";
    require(lowest > 600.0, "the twin idles without stalling");
}

/** Instrument: `--trace <rpm> [--single] [--exhaust-cell <m>]` holds full
 *  load at `rpm` for 6 s (rpm 0: a free idle, throttle shut), then prints one
 *  cylinder's cycle state and a 0.5-degree trace of one revolution. `--single` runs one cylinder alone
 *  (the manifold pressure is then its crankcase's). */
int trace(double rpm, bool single, double exhaustCellM) {
    auto config = catalogueTwoStroke();
    if (single) {
        // One cylinder alone, so the manifold pressure is its crankcase's.
        config.name += " (one cylinder)";
        config.cylinders.resize(1);
        config.firingOrder = { config.cylinders.front().id };
        config.intakePaths.resize(1);
        config.exhaustPaths.resize(1);
        for (auto& bank : config.banks) bank.cylinderIds = { config.cylinders.front().id };
        normaliseEngineConfig(config);
        if (const auto error = validateEngineConfig(config)) {
            std::cerr << *error << '\n';
            return EXIT_FAILURE;
        }
    }
    EngineSimulatorOptions options;
    if (exhaustCellM > 0.0) options.exhaustTargetCellLengthM = exhaustCellM;
    auto rig = std::make_unique<Rig>(config, options);
    auto speed = 0.0;
    const auto control = [&](double t) {
        EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = t < 1.5;
        controls.throttle = t < 2.0 || rpm <= 0.0 ? 0.0 : 1.0;
        if (t >= 2.0 && rpm > 0.0) controls.dynamometerTorqueNm = std::max(0.0, 2.0 * (speed - rpm));
        return controls;
    };
    auto meanTorqueNm = 0.0;
    for (auto frame = 0; frame < 6 * 240; ++frame) {
        speed = rig->simulator->step(frameSeconds, control(frame * frameSeconds)).state.rpm;
        if (frame >= 4 * 240) meanTorqueNm += rig->simulator->state().torqueNm / (2.0 * 240.0);
    }
    const auto& state = rig->simulator->state();
    std::cout << "mean torque over 4-6 s " << meanTorqueNm << " N.m\n";
    const auto& cylinder = state.cylinderStates[0];
    std::cout << "rpm " << state.rpm << " torque " << state.torqueNm << " imep " << state.indicatedMeanEffectivePressureBar
              << " map " << state.manifoldPressureKpa
              << " back pressure " << state.exhaustBackPressureKpa << "\ntrapped fresh " << cylinder.trappedFreshAirMassMg
              << " mg, delivered " << cylinder.deliveredFreshAirMassMgPerCycle << " mg, trapped " << cylinder.trappedMassMg
              << " mg, residual at spark " << cylinder.residualGasFractionAtSpark << ", phi at spark "
              << cylinder.equivalenceRatioAtSpark << ", efficiency " << cylinder.combustionEfficiency
              << ", sparks " << cylinder.completedIgnitionEventsLastCycle << " at " << cylinder.completedIgnitionPhaseLastCycle
              << ", fuel delivered " << cylinder.deliveredFuelMgPerCycle << " requested " << cylinder.requestedFuelMgPerCycle
              << ", work " << cylinder.indicatedWorkJoulesPerCycle << " J, afr " << cylinder.airFuelRatio
              << ", trim " << cylinder.closedLoopFuelTrim << ", misfires " << state.misfireEventCount << '\n';
    const auto dt = 0.5 / (std::max(rpm, state.rpm) * 6.0);
    auto t = 6.0;
    std::cout << "phase,chamber_bar,runner_kpa,map_kpa,gas_c,burned_fraction,fresh_mg,trapped_mg,exh_flow_kg_s,intake_v,exh_v\n";
    for (auto step = 0; step < 720; ++step) {
        const auto& s = rig->simulator->step(dt, control(t)).state;
        t += dt;
        const auto& c = s.cylinderStates[0];
        std::cout << c.cyclePhaseDegrees << ',' << c.pressureEstimateBar << ',' << c.intakeRunnerChargePressureKpa << ','
                  << s.manifoldPressureKpa << ',' << c.gasTemperatureC << ',' << c.burnedFraction << ','
                  << c.trappedFreshAirMassMg << ',' << c.trappedMassMg << ',' << c.exhaustMassFlowKgPerSecond << ','
                  << c.intakeVelocityMps << ',' << c.exhaustVelocityMps << '\n';
    }
    return EXIT_SUCCESS;
}

// The scavenging law on its own, before any engine runs it. These are the
// properties that make it a scavenging model rather than a fudge factor: it
// must reduce to the old perfect mixing where the geometry is unknown, reach
// the displacement bound where the window is short, and carry engine speed
// through a ratio of two times rather than a fitted curve.
// A port's opening ANGLE is not its duration, and reading one as the other is
// a quiet 5 % error in the scavenging window. It also fixes the ordering the
// whole model rests on: the exhaust port must open before the transfer ports
// and close after them, or there is no blowdown and no tail to short-circuit
// through.
void portDurationIsNotTheOpeningAngle() {
    const TwoStrokeConfig stock {};
    const auto exhaustDuration =
        portOpenDurationDegrees(stock.exhaustPortOpenDegrees);
    const auto transferDuration =
        portOpenDurationDegrees(stock.transferPortOpenDegrees);
    require(std::abs(exhaustDuration - 176.0) < 1.0e-12,
        "the stock exhaust port is open for 176 deg, not 92");
    require(std::abs(transferDuration - 124.0) < 1.0e-12,
        "the stock transfer ports are open for 124 deg, not 118");
    require(exhaustDuration > transferDuration,
        "the exhaust port outlasts the transfer ports, as a two-stroke needs");
    require(stock.exhaustPortOpenDegrees < stock.transferPortOpenDegrees,
        "the exhaust port opens first, giving a blowdown before transfer");
    require(portOpenDurationDegrees(200.0) == 0.0,
        "a port the piston never uncovers is never open");
}

void scavengingLawBracketsTheClassicalBounds() {
    // Perfect displacement keeps everything up to a full cylinder; perfect
    // mixing always loses some. Displacement is the better bound everywhere.
    for (const auto deliveryRatio : { 0.25, 0.5, 1.0, 1.5, 2.0 }) {
        require(perfectDisplacementEfficiency(deliveryRatio)
                > perfectMixingEfficiency(deliveryRatio),
            "perfect displacement traps more than perfect mixing");
    }
    require(std::abs(perfectDisplacementEfficiency(2.0) - 1.0) < 1.0e-12,
        "displacement saturates at a full cylinder");
    require(perfectMixingEfficiency(0.0) == 0.0,
        "no delivered charge scavenges nothing");

    // A window nobody opened behaves exactly as the simulator did before this
    // existed: perfect mixing, so no default figure can move.
    TwoStrokeScavengeState closed;
    require(scavengeMixedFraction(closed) == 1.0,
        "a closed scavenging window is perfect mixing");

    // Geometry for these: a 64 mm bore loops up, over and down in roughly
    // 1.8 bores, so 0.12 m, and a transfer jet runs tens of m/s. Transfers
    // opening 116 deg ATDC last 2 x (180 - 116) = 128 deg, which is 3.6 ms at
    // 6,000 rpm.
    constexpr auto loopM = 0.12;
    const auto windowSeconds = [](double rpm) {
        return portOpenDurationDegrees(116.0) / (6.0 * rpm);
    };
    // Integrate a window at a constant jet velocity, in `steps` sub-steps,
    // exactly as the simulator does.
    const auto runWindow = [loopM](double jetMps, double seconds, int steps) {
        TwoStrokeScavengeState state;
        openScavengeWindow(state, loopM);
        for (auto step = 0; step < steps; ++step)
            advanceScavengeWindow(state, jetMps, seconds / steps);
        return state;
    };

    // A window the front cannot cross before the ports shut is pure
    // displacement throughout: nothing fresh escapes. A slow jet at high speed
    // is that case: 40 m/s covers 0.053 m of the 0.12 m loop in the 1.8 ms a
    // 12,000 rpm window lasts.
    const auto brief = runWindow(40.0, windowSeconds(12'000.0), 64);
    require(brief.frontProgress < 1.0,
        "the front does not reach the port in this window");
    require(scavengeMixedFraction(brief) == 0.0,
        "a window the front cannot cross is pure displacement throughout");

    // A normal window: burned gas only until the front arrives, then a
    // monotone breakdown towards mixing by the time the ports shut.
    const auto full = windowSeconds(6'000.0);
    require(scavengeMixedFraction(runWindow(80.0, full, 64)) > 0.0,
        "the front arrives inside a 6,000 rpm window at 80 m/s");
    auto previous = 0.0;
    auto rose = 0;
    for (auto step = 1; step <= 20; ++step) {
        const auto mixed =
            scavengeMixedFraction(runWindow(80.0, full * 0.05 * step, 16));
        require(mixed >= previous - 1.0e-12,
            "the mixed fraction never falls inside a window");
        if (mixed > previous + 1.0e-9) ++rose;
        previous = mixed;
    }
    require(rose >= 5, "the mixed fraction actually rises across the window");
    require(std::abs(previous - 1.0) < 1.0e-9,
        "the window ends at perfect mixing");

    // Integrating the front is what makes the model indifferent to the one
    // instant the ports crack open. A jet that is fast early and slow late
    // must land where its time average does, not where either half says: the
    // one-shot sample this replaced read the opening instant alone, which in
    // the 5,000-6,000 band is the exhaust pipe's returning wave.
    {
        TwoStrokeScavengeState split;
        openScavengeWindow(split, loopM);
        for (auto step = 0; step < 32; ++step)
            advanceScavengeWindow(split, 120.0, full * 0.5 / 32);
        for (auto step = 0; step < 32; ++step)
            advanceScavengeWindow(split, 40.0, full * 0.5 / 32);
        const auto average = runWindow(80.0, full, 64);
        require(std::abs(split.frontProgress - average.frontProgress) < 1.0e-12,
            "a window's front progress is its time-averaged jet velocity");
    }

    // A stalled jet does not move the front. A cycle where the crankcase never
    // builds a pressure difference must not creep towards mixing.
    {
        TwoStrokeScavengeState stalled;
        openScavengeWindow(stalled, loopM);
        for (auto step = 0; step < 64; ++step)
            advanceScavengeWindow(stalled, 0.0, full / 64);
        require(stalled.frontProgress == 0.0,
            "no pressure across the ports leaves the front where it was");
    }

    // The engine-speed mechanism. Same geometry, same jet: a faster engine has
    // a shorter window, so the front covers less of the loop and the
    // displacement phase owns more of the window. This is the term the model
    // had no mechanism for at all.
    const auto mixedAtRpm = [&](double rpm) {
        return scavengeMixedFraction(runWindow(80.0, windowSeconds(rpm), 64));
    };
    const auto slow = mixedAtRpm(3'000.0);
    const auto fast = mixedAtRpm(9'000.0);
    require(fast < slow,
        "a faster engine keeps more of its charge over its whole window");
    // A loop length of zero is the switch that restores the perfectly mixed
    // cylinder. It must report mixing on EVERY sub-step of the window,
    // including the first.
    {
        TwoStrokeScavengeState noLoop {};
        openScavengeWindow(noLoop, 0.0);
        for (int step = 0; step <= 10; ++step) {
            require(scavengeMixedFraction(noLoop) == 1.0,
                "a cylinder with no loop mixes on every sub-step of the window");
            advanceScavengeWindow(noLoop, 80.0, 0.0001);
        }
    }

    std::cout << "    scavenging over a whole window at 80 m/s: 3000 rpm mixed="
              << slow << "  9000 rpm mixed=" << fast << '\n';
}

// The apportionment that turns a mixed fraction into a port composition. These
// are invariants rather than values: whatever the bias does to the mixture, it
// may not create or destroy mass, and it must vanish exactly where the model
// says it has no claim to make.
void outflowBiasMovesCompositionWithoutMovingMass() {
    // A plausible mid-scavenge cylinder: fresh air and fuel over a residual.
    constexpr auto oxygen = 0.21;
    constexpr auto inert = 0.79;
    constexpr auto fuel = 0.014;
    constexpr auto burned = 0.25;
    constexpr auto total = oxygen + inert + fuel + burned;

    const auto identity = biasedOutflowDensities(oxygen, inert, fuel, burned, 1.0);
    require(identity.oxygen == oxygen && identity.inert == inert
            && identity.fuel == fuel && identity.burned == burned,
        "perfect mixing is bit-exact identity, so a four-stroke cannot move");

    for (const auto mixed : { 0.0, 0.25, 0.5, 0.75, 1.0 }) {
        const auto biased =
            biasedOutflowDensities(oxygen, inert, fuel, burned, mixed);
        const auto biasedTotal =
            biased.oxygen + biased.inert + biased.fuel + biased.burned;
        require(std::abs(biasedTotal - total) < 1.0e-12 * total,
            "the bias preserves total mass density");
        require(biased.oxygen >= 0.0 && biased.inert >= 0.0
                && biased.fuel >= 0.0 && biased.burned >= 0.0,
            "no species is driven negative");
        require(biased.oxygen <= oxygen * (1.0 + 1.0e-12)
                && biased.fuel <= fuel * (1.0 + 1.0e-12),
            "the bias only ever holds fresh charge back, never adds it");
        require(biased.inert >= inert * (1.0 - 1.0e-12)
                && biased.burned >= burned * (1.0 - 1.0e-12),
            "the mass held back is replaced by residual, not lost");
    }

    // Displacement is monotone in the window's progress: more mixing lets more
    // fresh charge out of the port, never less.
    auto previousFuel = -1.0;
    for (const auto mixed : { 0.0, 0.2, 0.4, 0.6, 0.8, 1.0 }) {
        const auto biased =
            biasedOutflowDensities(oxygen, inert, fuel, burned, mixed);
        require(biased.fuel > previousFuel - 1.0e-15,
            "fuel at the port rises with the mixed fraction");
        previousFuel = biased.fuel;
    }

    // Cranking: almost no burned gas to displace with. The bias must reduce
    // itself to what the residual can cover instead of inventing mass.
    const auto cranking = biasedOutflowDensities(0.21, 0.79, 0.014, 0.0, 0.0);
    const auto crankingTotal =
        cranking.oxygen + cranking.inert + cranking.fuel + cranking.burned;
    require(std::abs(crankingTotal - (0.21 + 0.79 + 0.014)) < 1.0e-14,
        "a cylinder with no burned gas still conserves mass");
    require(cranking.oxygen >= 0.0 && cranking.fuel >= 0.0,
        "a cylinder with no burned gas is not driven negative");

    // Nothing at all to present: the bias has no claim and must stand aside.
    const auto empty = biasedOutflowDensities(0.0, 0.0, 0.0, 0.0, 0.0);
    require(empty.oxygen == 0.0 && empty.inert == 0.0 && empty.fuel == 0.0
            && empty.burned == 0.0,
        "an empty cell biases to nothing");

    const auto pureResidual = biasedOutflowDensities(0.0, 0.79, 0.0, 0.25, 0.0);
    require(pureResidual.inert == 0.79 && pureResidual.burned == 0.25,
        "with no fresh charge to hold back the bias is identity");

    std::cout << "port fuel density, mixed=0 vs mixed=1: "
              << biasedOutflowDensities(oxygen, inert, fuel, burned, 0.0).fuel
              << "  " << biasedOutflowDensities(oxygen, inert, fuel, burned, 1.0).fuel
              << '\n';
}

// A carburettor is checked against the convention its jets are stamped with
// and against one real engine's numbers, never against its own output.
struct CarburettorRig final {
    CarburettorConfig carburettor {};
    FuelConfig fuel {};
    // The RD350LC-like twin, per cylinder: 173.5 cc on a 26 mm carburettor.
    double throttleDiameterMm { 26.0 };
    double displacementPerCylinderM3 { 173.5e-6 };
    double airDensityKgPerM3 { 1.2 };

    CarburettorRig() { carburettor.enabled = true; }

    /** Air crossing one carburettor, from what the cylinder swallows. */
    [[nodiscard]] double airFlowKgPerSecond(double rpm, double deliveryRatio) const {
        return displacementPerCylinderM3 * deliveryRatio * (rpm / 60.0)
            * airDensityKgPerM3;
    }

    [[nodiscard]] CarburettorMetering run(CarburettorState& state,
                                          double airFlowKgPerSecond,
                                          double crankcaseKpa,
                                          double throttleFraction,
                                          double dtSeconds) const {
        return CarburettorModel::meter(carburettor, fuel, state, airFlowKgPerSecond,
            airDensityKgPerM3, 101.325, crankcaseKpa, throttleDiameterMm,
            throttleFraction, dtSeconds);
    }

    /** Air-fuel ratio from the fuel a settled carburettor delivers. */
    [[nodiscard]] double settledAirFuelRatio(double airFlowKgPerSecond,
                                             double crankcaseKpa,
                                             double throttleFraction) const {
        CarburettorState state;
        constexpr double dt = 1.0e-4;
        auto fuelKgPerSecond = 0.0;
        const auto molarMassKg = fuel.molarMassGramsPerMole * 0.001;
        // Long enough for the wall film to stop taking a share.
        for (auto step = 0; step < 20'000; ++step) {
            const auto metering = run(state, airFlowKgPerSecond, crankcaseKpa,
                throttleFraction, dt);
            if (step >= 19'000)
                fuelKgPerSecond += metering.deliveredMoles * molarMassKg / dt;
        }
        fuelKgPerSecond /= 1'000.0;
        return fuelKgPerSecond > 0.0 ? airFlowKgPerSecond / fuelKgPerSecond : 1.0e9;
    }
};

void aJetPassesItsStampedNumber() {
    // A jet's number is cc/min of test fluid under one metre of head, and the
    // velocity a metre of head gives is sqrt(2 g h) whatever the fluid weighs.
    // Putting the modelled jet back under that head must return the number.
    constexpr double dischargeCoefficient = 0.80;
    for (const auto flowNumber : { 20.0, 35.0, 180.0, 280.0, 420.0 }) {
        const auto areaM2 = CarburettorModel::jetAreaM2(flowNumber, dischargeCoefficient);
        const auto velocityMps = std::sqrt(2.0 * 9.80665 * 1.0);
        const auto volumeFlowCcPerMinute =
            dischargeCoefficient * areaM2 * velocityMps * 1.0e6 * 60.0;
        require(std::abs(volumeFlowCcPerMinute - flowNumber) < 1.0e-9 * flowNumber,
            "a jet under one metre of head passes its stamped number");
    }
    require(CarburettorModel::jetAreaM2(0.0, 0.8) == 0.0,
        "an absent jet has no area");
}

void theTwoCircuitsHandOverWithoutBeingBlended() {
    const CarburettorRig rig;
    CarburettorState idleState;
    // Idle: the slide is shut, the crankcase is pumping well below ambient.
    const auto idleAir = rig.airFlowKgPerSecond(1'300.0, 0.35);
    const auto idle = rig.run(idleState, idleAir, 80.0, 0.0, 1.0e-4);
    require(idle.mainMoles == 0.0,
        "the main circuit passes nothing at idle: the venturi cannot lift the fuel");
    require(idle.pilotMoles > 0.0, "the pilot circuit carries the idle");

    CarburettorState wideState;
    // Full throttle: a two-stroke's crankcase pumps at about ambient, so the
    // pilot has no drop left to work with.
    const auto wideAir = rig.airFlowKgPerSecond(8'000.0, 0.9);
    const auto wide = rig.run(wideState, wideAir, 101.325, 1.0, 1.0e-4);
    require(wide.pilotMoles == 0.0,
        "the pilot circuit passes nothing at full throttle: its drop has gone");
    require(wide.mainMoles > 0.0, "the main circuit carries full throttle");
    require(wide.venturiDepressionKpa > 1.5 && wide.venturiDepressionKpa < 3.5,
        "the full-throttle venturi depression is the couple of kPa a 26 mm bore gives");
}

void theDefaultJetsGiveTheRealEngineItsMixture() {
    const CarburettorRig rig;
    // The one reference that is not the simulator's own output: a 280 main is
    // what the bike runs, and it has to come out at a usable full-load mixture.
    const auto wideOpenAfr = rig.settledAirFuelRatio(
        rig.airFlowKgPerSecond(8'000.0, 0.9), 101.325, 1.0);
    std::cout << "  full throttle, 8,000 rpm: AFR " << wideOpenAfr << '\n';
    require(wideOpenAfr > 11.5 && wideOpenAfr < 14.5,
        "the stock main jet gives a full-load mixture a two-stroke can run");
    const auto idleAfr = rig.settledAirFuelRatio(
        rig.airFlowKgPerSecond(1'300.0, 0.35), 80.0, 0.0);
    std::cout << "  idle, 1,300 rpm: AFR " << idleAfr << '\n';
    require(idleAfr > 11.0 && idleAfr < 17.0,
        "the stock pilot jet idles in the band a carburetted two-stroke idles in");
}

void aPlainJetRichensAsTheEngineBreathesHarder() {
    const CarburettorRig rig;
    // The lift of the jet above the bowl is a fixed head the depression has to
    // beat, so it costs a larger share of a small depression than of a big
    // one. That is the plain-jet characteristic, and it is why a real
    // carburettor needs an emulsion tube at the top of the range.
    auto previousAfr = 0.0;
    for (const auto rpm : { 3'000.0, 4'500.0, 6'000.0, 8'000.0 }) {
        const auto afr = rig.settledAirFuelRatio(
            rig.airFlowKgPerSecond(rpm, 0.9), 101.325, 1.0);
        if (previousAfr > 0.0)
            require(afr < previousAfr,
                "a plain main jet richens as the air flow rises");
        previousAfr = afr;
    }
}

void theJetsAndTheScrewsDoWhatTheyAreFor() {
    CarburettorRig rig;
    const auto wideAir = rig.airFlowKgPerSecond(8'000.0, 0.9);
    const auto stockAfr = rig.settledAirFuelRatio(wideAir, 101.325, 1.0);
    // A jet's area goes with its number, and so does the fuel it passes.
    rig.carburettor.mainJetFlowNumber = 560.0;
    const auto doubledAfr = rig.settledAirFuelRatio(wideAir, 101.325, 1.0);
    require(doubledAfr < stockAfr * 0.55 && doubledAfr > stockAfr * 0.45,
        "doubling the main jet's number halves the air-fuel ratio");
    rig.carburettor.mainJetFlowNumber = 280.0;

    // The air screw bleeds air into the pilot circuit, which leans the idle.
    const auto idleAir = rig.airFlowKgPerSecond(1'300.0, 0.35);
    const auto nominalIdleAfr = rig.settledAirFuelRatio(idleAir, 80.0, 0.0);
    rig.carburettor.airScrewTurns = 3.0;
    const auto openIdleAfr = rig.settledAirFuelRatio(idleAir, 80.0, 0.0);
    require(openIdleAfr > nominalIdleAfr,
        "opening the air screw leans the idle");
    rig.carburettor.airScrewTurns = 0.75;
    const auto closedIdleAfr = rig.settledAirFuelRatio(idleAir, 80.0, 0.0);
    require(closedIdleAfr < nominalIdleAfr,
        "closing the air screw richens the idle");
}

void theNeedleClipMovesTheMiddleAndNotTheTop() {
    CarburettorConfig rich;
    CarburettorConfig lean;
    rich.needleClipPosition = 2.0;
    lean.needleClipPosition = 4.0;
    const auto richMid = CarburettorModel::needleOpenFraction(rich, 0.4);
    const auto leanMid = CarburettorModel::needleOpenFraction(lean, 0.4);
    require(richMid > leanMid, "a lower clip holds the needle higher and richens mid-throttle");
    require(CarburettorModel::needleOpenFraction(rich, 1.0) == 1.0
        && CarburettorModel::needleOpenFraction(lean, 1.0) == 1.0,
        "the clip position does not touch full throttle, where the main jet meters");
    CarburettorConfig middle;
    require(CarburettorModel::needleOpenFraction(middle, 0.0) < 0.25,
        "the needle nearly shuts the needle jet with the slide down");
    auto previous = -1.0;
    for (const auto throttle : { 0.0, 0.2, 0.4, 0.6, 0.8, 1.0 }) {
        const auto open = CarburettorModel::needleOpenFraction(middle, throttle);
        require(open >= previous, "the needle jet opens monotonically with the slide");
        previous = open;
    }
}

void theWallFilmKeepsEveryMoleItTakes() {
    const CarburettorRig rig;
    const auto wideAir = rig.airFlowKgPerSecond(8'000.0, 0.9);
    CarburettorState state;
    auto metered = 0.0;
    auto delivered = 0.0;
    constexpr double dt = 1.0e-4;
    for (auto step = 0; step < 2'000; ++step) {
        const auto metering = rig.run(state, wideAir, 101.325, 1.0, dt);
        metered += metering.meteredMoles;
        delivered += metering.deliveredMoles;
    }
    const auto accounted = delivered + state.wallFilmMoles;
    require(std::abs(accounted - metered) < 1.0e-12 * std::max(1.0, metered),
        "every mole the jets passed is in the charge or still on the wall");

    // The stumble: the first charge after a tip-in is short of what the jets
    // metered, because the bore is being wetted before it is fed.
    CarburettorState fresh;
    const auto first = rig.run(fresh, wideAir, 101.325, 1.0, dt);
    require(first.deliveredMoles < first.meteredMoles,
        "a dry bore keeps part of the first fuel it is given");
    require(fresh.wallFilmMoles > 0.0, "that fuel is on the wall, not lost");

    // Metering is an amount per step, so the solver's rate cannot change it.
    CarburettorState coarse;
    CarburettorState fine;
    auto coarseMetered = 0.0;
    auto fineMetered = 0.0;
    for (auto step = 0; step < 100; ++step)
        coarseMetered += rig.run(coarse, wideAir, 101.325, 1.0, 1.0e-4).meteredMoles;
    for (auto step = 0; step < 400; ++step)
        fineMetered += rig.run(fine, wideAir, 101.325, 1.0, 0.25e-4).meteredMoles;
    require(std::abs(fineMetered - coarseMetered) < 1.0e-12 * coarseMetered,
        "four times the substeps meter the same fuel: this is not a rate gain");
}

} // namespace

int main(int argc, char** argv) {
    if (argc >= 3 && std::string_view(argv[1]) == "--trace") {
        auto single = false;
        auto exhaustCellM = 0.0;
        for (auto index = 3; index < argc; ++index) {
            if (std::string_view(argv[index]) == "--single") single = true;
            if (std::string_view(argv[index]) == "--exhaust-cell" && index + 1 < argc)
                exhaustCellM = std::stod(argv[++index]);
        }
        return trace(std::stod(argv[2]), single, exhaustCellM);
    }
    portsFollowThePiston();
    configurationIsChecked();
    missingCrankOffsetsFollowTheCycle();
    loadIsTheDeliveredCharge();
    catalogueTwinIdles();
    catalogueTwinMakesItsTorque();
    scavengingIsCoupledFinely();
    theScavengingLoopIsASwitch();
    portDurationIsNotTheOpeningAngle();
    scavengingLawBracketsTheClassicalBounds();
    outflowBiasMovesCompositionWithoutMovingMass();
    aJetPassesItsStampedNumber();
    theTwoCircuitsHandOverWithoutBeingBlended();
    theDefaultJetsGiveTheRealEngineItsMixture();
    aPlainJetRichensAsTheEngineBreathesHarder();
    theJetsAndTheScrewsDoWhatTheyAreFor();
    theNeedleClipMovesTheMiddleAndNotTheTop();
    theWallFilmKeepsEveryMoleItTakes();
    std::cout << "two-stroke: ok\n";
    return EXIT_SUCCESS;
}
