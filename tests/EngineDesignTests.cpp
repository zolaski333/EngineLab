// The engine designer: an engine from a short description, with no base
// engine. Crank balance matches the textbook, every layout gives a valid
// engine whose values say where they come from, a set value wins over its
// rule, and a designed engine runs.

#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/catalog/EngineDesign.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/foundation/CrankBalance.hpp>
#include <enginelab/gasdynamics/ExhaustNetworkLayout.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/runtime/RealtimeCapacityProbe.hpp>
#include <enginelab/serialization/YamlEngineSerializer.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <numeric>
#include <string_view>
#include <utility>
#include <vector>

namespace {
using namespace enginelab;
constexpr double frameSeconds = 1.0 / 240.0;

void require(bool condition, std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

[[nodiscard]] bool near(double a, double b, double tolerance) { return std::abs(a - b) <= tolerance * std::abs(b); }

/** An even-firing inline crank: cylinder ids in firing order, one pitch apart. */
[[nodiscard]] std::vector<BalanceCylinder> inlineCrank(const std::vector<std::uint32_t>& order, double interval) {
    std::vector<BalanceCylinder> cylinders(order.size());
    for (std::size_t position = 0; position < order.size(); ++position) {
        auto& cylinder = cylinders[order[position] - 1];
        cylinder.tdcDegrees = static_cast<double>(position) * interval;
        cylinder.axialPitches = static_cast<double>(order[position] - 1);
    }
    return cylinders;
}

[[nodiscard]] bool zero(double value) { return std::abs(value) < 1.0e-6; }

/** Textbook free forces and couples of inline cranks (rod ratio 0.3). */
void crankBalanceMatchesTheTextbook() {
    const auto four = analyseCrankBalance(inlineCrank({ 1, 3, 4, 2 }, 180.0), 720.0);
    require(zero(four.primaryForce) && zero(four.primaryCouple) && zero(four.secondaryCouple),
        "a flat-plane inline four has no primary and no secondary couple");
    require(near(four.secondaryForce, 4.0 * 0.3, 1.0e-4), "its secondary force is 4 m r w^2 x r/l");
    const auto three = analyseCrankBalance(inlineCrank({ 1, 2, 3 }, 240.0), 720.0);
    require(zero(three.primaryForce) && zero(three.secondaryForce), "an inline three has no free force");
    require(near(three.primaryCouple, std::sqrt(3.0), 1.0e-4), "its primary couple is sqrt(3) m r w^2 a");
    require(near(three.secondaryCouple, std::sqrt(3.0) * 0.3, 1.0e-4), "its secondary couple is sqrt(3) r/l");
    const auto six = analyseCrankBalance(inlineCrank({ 1, 5, 3, 6, 2, 4 }, 120.0), 720.0);
    require(zero(six.primaryForce) && zero(six.secondaryForce) && zero(six.primaryCouple)
            && zero(six.secondaryCouple), "an inline six is balanced in both orders");
    const auto five = analyseCrankBalance(inlineCrank({ 1, 2, 4, 5, 3 }, 144.0), 720.0);
    require(zero(five.primaryForce) && zero(five.secondaryForce) && five.primaryCouple > 0.1,
        "an inline five has no free force but a primary couple");

    // A 90 deg cross-plane V8: pins at 0/90/270/180 deg, one per row, each
    // shared by a cylinder of each bank (top dead centre when the pin meets
    // the bore axis).
    std::vector<BalanceCylinder> crossPlane;
    const double pins[] = { 0.0, 90.0, 270.0, 180.0 };
    for (int row = 0; row < 4; ++row) {
        for (const auto bank : { -45.0, 45.0 }) {
            BalanceCylinder cylinder;
            cylinder.bankDegrees = bank;
            cylinder.tdcDegrees = bank - pins[row];
            cylinder.axialPitches = static_cast<double>(row);
            crossPlane.push_back(cylinder);
        }
    }
    const auto v8 = analyseCrankBalance(crossPlane, 720.0);
    require(zero(v8.primaryForce) && zero(v8.secondaryForce) && zero(v8.secondaryCouple),
        "a cross-plane V8 has no free force and no secondary couple");
    require(near(v8.primaryCouple, std::sqrt(10.0), 1.0e-4), "its primary couple is sqrt(10) m r w^2 a");

    auto twin = inlineCrank({ 1, 2 }, 270.0);
    const auto cp2 = analyseCrankBalance(twin, 720.0);
    require(cp2.firingIntervalsDegrees.size() == 2 && near(cp2.firingIntervalsDegrees[0], 270.0, 1.0e-9)
            && near(cp2.firingIntervalsDegrees[1], 450.0, 1.0e-9), "a 270 deg twin fires at 270 / 450");
}

void firingOrdersAreRanked() {
    const auto six = inlineFiringOrders(6, EngineCycle::fourStroke, 0.3, 8);
    require(!six.empty() && zero(six.front().balance.primaryCouple) && zero(six.front().balance.secondaryCouple),
        "the best inline-six crank is balanced");
    const auto four = inlineFiringOrders(4, EngineCycle::fourStroke, 0.3, 8);
    require(!four.empty() && zero(four.front().balance.primaryCouple)
            && near(four.front().balance.secondaryForce, 1.2, 1.0e-4), "the best inline four is the flat plane");
    const auto seven = inlineFiringOrders(7, EngineCycle::fourStroke, 0.3, 4);
    require(seven.size() == 4 && zero(seven.front().balance.primaryForce)
            && seven.front().balance.firingIntervalsDegrees.size() == 7
            && near(seven.front().balance.firingIntervalsDegrees.front(), 720.0 / 7.0, 1.0e-9),
        "an inline seven fires every 102.9 deg with no free force");
    std::cout << "inline seven, best orders:";
    for (const auto& candidate : seven) {
        std::cout << "\n  ";
        for (const auto id : candidate.order) std::cout << id << ' ';
        std::cout << " primary couple " << candidate.balance.primaryCouple << ", secondary couple "
                  << candidate.balance.secondaryCouple << ", adjacent firings " << candidate.adjacentFirings;
    }
    std::cout << '\n';
}

[[nodiscard]] const PartsLibrary& parts() {
    static const auto library = loadPartsLibrary(ENGINELAB_CATALOG_ROOT);
    return library;
}

[[nodiscard]] const DesignValue* valueOf(const DesignedEngine& engine, std::string_view key) {
    const auto found = std::find_if(engine.values.begin(), engine.values.end(),
        [key](const DesignValue& value) { return value.key == key; });
    return found == engine.values.end() ? nullptr : &*found;
}

[[nodiscard]] EngineSpec inlineSeven() {
    EngineSpec spec;
    spec.name = "Inline Seven";
    spec.cylinderCount = 7;
    spec.boreMm = 84.0;
    spec.strokeMm = 72.0;
    return spec;
}

[[nodiscard]] DesignedEngine designed(const EngineSpec& spec) {
    auto engine = designEngine(spec, parts());
    if (!engine.config) std::cerr << "  " << spec.name << ": " << engine.error << '\n';
    require(engine.config.has_value(), "the spec designs a valid engine");
    return engine;
}

void evenlySpaced(const EngineConfig& config) {
    const auto balance = analyseCrankBalance(config);
    const auto interval = cycleDegreesOf(config.cycle) / static_cast<double>(config.cylinders.size());
    for (const auto gap : balance.firingIntervalsDegrees)
        require(std::abs(gap - interval) < 1.0e-6, "the cylinders fire evenly");
}

void inlineSevenFromNothing() {
    const auto engine = designed(inlineSeven());
    const auto& config = *engine.config;
    require(config.cylinders.size() == 7, "seven cylinders");
    require(std::abs(engineDisplacementLitres(config) - 2.793) < 0.01, "2.8 litres from bore and stroke");
    auto order = config.firingOrder;
    std::sort(order.begin(), order.end());
    std::vector<std::uint32_t> ids(7);
    std::iota(ids.begin(), ids.end(), 1U);
    require(order == ids, "the firing order names every cylinder once");
    const auto best = inlineFiringOrders(7, EngineCycle::fourStroke, 36.0 / 115.2, 1).front();
    require(config.firingOrder == best.order, "it is the best balanced order");
    evenlySpaced(config);
    const auto* redline = valueOf(engine, "redline_rpm");
    require(redline && !redline->overridden && !redline->rule.empty(), "the redline is derived, with its rule");
    require(redline->value > 7'500.0 && redline->value < 9'500.0, "a 72 mm stroke revs to 8,000-9,000 rpm");
    require(config.exhaustPaths.size() == 1 && config.exhaustPaths.front().network && config.exhaustPaths.front().network->components.size() > 2,
        "one generated exhaust");
    require(config.transmission.finalDriveRatio > 2.0 && config.transmission.finalDriveRatio < 6.0,
        "a final drive for a car");
}

void overridesWin() {
    auto spec = inlineSeven();
    spec.overrides["redline_rpm"] = 6'000.0;
    const auto engine = designed(spec);
    const auto* redline = valueOf(engine, "redline_rpm");
    require(redline && redline->overridden && redline->value == 6'000.0, "a set redline wins");
    require(std::abs(engine.config->redlineRpm - 6'000.0) < 1.0e-9, "and the engine takes it");
    const auto* peak = valueOf(engine, "peak_power_rpm");
    require(peak && std::abs(peak->value - 5'400.0) < 1.0e-6, "what derives from it follows");

    spec.overrides = { { "no_such_value", 1.0 } };
    const auto unknown = designEngine(spec, parts());
    require(!unknown.config && unknown.error.find("no_such_value") != std::string::npos,
        "an unknown value is refused, by name");

    spec = inlineSeven();
    spec.boreMm = 0.0;
    spec.strokeMm = 0.0;
    require(!designEngine(spec, parts()).config, "no size is refused");
    spec.displacementLitres = 3.5;
    const auto sized = designed(spec);
    require(std::abs(engineDisplacementLitres(*sized.config) - 3.5) < 0.01, "a displacement alone sizes it");
}

void everyLayoutDesigns() {
    EngineSpec v10;
    v10.name = "V10";
    v10.use = EngineUse::sportCar;
    v10.layout = EngineLayout::vLayout;
    v10.cylinderCount = 10;
    v10.bankAngleDegrees = 72.0;
    v10.displacementLitres = 5.0;
    const auto vee = designed(v10);
    require(vee.config->banks.size() == 2 && vee.config->exhaustPaths.size() == 2, "a V has two banks and pipes");
    evenlySpaced(*vee.config);

    EngineSpec w16;
    w16.name = "W16";
    w16.layout = EngineLayout::custom;
    w16.cylinderCount = 16;
    w16.bankCount = 4;
    w16.bankAngleDegrees = 30.0;
    w16.displacementLitres = 8.0;
    w16.induction = Induction::turbocharged;
    w16.turbochargerCount = 4;
    const auto w = designed(w16);
    require(w.config->banks.size() == 4, "a W has its four banks");
    std::vector<double> angles;
    for (const auto& cylinder : w.config->cylinders) angles.push_back(cylinder.bankOffsetDegrees);
    std::sort(angles.begin(), angles.end());
    angles.erase(std::unique(angles.begin(), angles.end()), angles.end());
    require(angles.size() == 4, "its cylinders lie on four bank angles, not one");
    w16.bankAnglesDegrees = { -52.5, -37.5, 37.5, 52.5 };
    const auto bugatti = designed(w16);
    require(bugatti.config->cylinders[1].bankOffsetDegrees == -37.5
            && bugatti.config->cylinders[3].bankOffsetDegrees == 52.5, "or at the angles given");
    evenlySpaced(*bugatti.config);
    require(w.config->forcedInduction.enabled && w.config->forcedInduction.turbochargerCount == 4U, "quad turbo");
    evenlySpaced(*w.config);
    auto noAngle = w16;
    noAngle.bankAngleDegrees = 0.0;
    noAngle.bankAnglesDegrees.clear();
    const auto spread = designed(noAngle);
    require(spread.config.has_value(), "a W with no angle given designs");
    std::vector<double> spreadAngles;
    for (const auto& bank : spread.config->banks) spreadAngles.push_back(bank.angleDegrees);
    require(spreadAngles == std::vector<double> { -60.0, -20.0, 20.0, 60.0 },
        "with its banks spread over 120 deg, not stacked at 0 deg");
    noAngle.bankAnglesDegrees = { -30.0, 10.0, 10.0, 30.0 };
    const auto stacked = designEngine(noAngle, parts());
    require(!stacked.config && stacked.error.find("same angle") != std::string::npos,
        "two banks at one angle are refused");

    EngineSpec radial;
    radial.name = "Radial Nine";
    radial.use = EngineUse::aircraft;
    radial.layout = EngineLayout::radial;
    radial.cylinderCount = 9;
    radial.boreMm = 155.6;
    radial.strokeMm = 174.6;
    require(designEngine(radial, parts()).config.has_value(), "a 30 litre radial designs (no size cap)");
    radial.boreMm = 114.0;
    radial.strokeMm = 127.0;
    const auto star = designed(radial);
    require(star.config->firingOrder == std::vector<std::uint32_t>({ 1, 3, 5, 7, 9, 2, 4, 6, 8 }),
        "a radial fires every other cylinder");
    require(star.config->cylinders[0].connectingRodType == ConnectingRodType::master
            && star.config->cylinders[4].connectingRodType == ConnectingRodType::articulated,
        "on one master rod");
    require(!star.config->thermal.liquidCooled, "air-cooled");
    evenlySpaced(*star.config);
    for (const auto& cylinder : star.config->cylinders)
        require(std::abs(std::fmod(cylinder.crankOffsetDegrees - cylinder.bankOffsetDegrees, 360.0)) < 1.0e-9,
            "each cylinder is at top dead centre where the single throw points at it");
    radial.cylinderCount = 6;
    radial.boreMm = 100.0;
    radial.strokeMm = 100.0;
    const auto six = designed(radial);
    const auto sixIntervals = analyseCrankBalance(*six.config).firingIntervalsDegrees;
    require(*std::max_element(sixIntervals.begin(), sixIntervals.end()) > 125.0,
        "an even-count radial on one throw cannot fire evenly");
    radial.firingOrder = { 1, 2, 3, 4, 5, 6 };
    require(!designEngine(radial, parts()).config, "a radial's firing order is not given");

    EngineSpec diesel;
    diesel.name = "Diesel Five";
    diesel.fuel = FuelType::diesel;
    diesel.cylinderCount = 5;
    diesel.displacementLitres = 2.5;
    diesel.induction = Induction::turbocharged;
    const auto oil = designed(diesel);
    require(oil.config->cylinders.front().compressionRatio > 14.0, "a diesel compresses hard");
    require(oil.config->intake.idleBypassAreaMm2 == 0.0, "and is unthrottled");

    EngineSpec twin;
    twin.name = "Two-stroke Twin";
    twin.cycle = EngineCycle::twoStroke;
    twin.use = EngineUse::sportMotorcycle;
    twin.cylinderCount = 2;
    twin.boreMm = 64.0;
    twin.strokeMm = 54.0;
    const auto stinger = designed(twin);
    require(stinger.config->exhaustPaths.size() == 2, "a two-stroke has one pipe per cylinder");
    require(stinger.config->twoStroke.exhaustPortOpenDegrees > 0.0, "and ports");
    require(stinger.config->intake.throttleCount == 1, "one carburettor bore feeds the crankcase circuit");
    const auto& chamber = stinger.config->exhaustPaths.front().network;
    require(chamber && chamber->components.size() == 8, "each pipe is an expansion chamber");
    const auto* tuned = valueOf(stinger, "expansion_chamber_tuned_length_mm");
    require(tuned && tuned->value > 700.0 && tuned->value < 1'100.0, "tuned to its peak power speed");
    for (const auto* unused : { "valves_per_side", "exhaust_primary_length_mm", "exhaust_primary_diameter_mm",
             "exhaust_collector_diameter_mm", "exhaust_outlet_diameter_mm", "injector_flow_mg_s" })
        require(valueOf(stinger, unused) == nullptr, std::string("a two-stroke lists no ") + unused);
    auto valved = twin;
    valved.overrides["valves_per_side"] = 2.0;
    require(!designCatalogueEngine(valved, ENGINELAB_CATALOG_ROOT).config, "and setting one is an unknown value");
    evenlySpaced(*stinger.config);

    EngineSpec crossplane;
    crossplane.name = "270 Twin";
    crossplane.use = EngineUse::roadMotorcycle;
    crossplane.cylinderCount = 2;
    crossplane.boreMm = 80.0;
    crossplane.strokeMm = 68.6;
    crossplane.crankOffsetsDegrees = { 0.0, 270.0 };
    const auto cp2 = designed(crossplane);
    const auto intervals = analyseCrankBalance(*cp2.config).firingIntervalsDegrees;
    require(intervals.size() == 2 && std::abs(intervals[0] - 270.0) < 1.0e-6, "a 270 deg crank as given");

    // Small engines: below some 0.3 L the rules gave values under the
    // simulator's floors (inertia, final drive, then plenum, piston, throttle)
    // and every one was refused with one generic line.
    for (const auto use : { EngineUse::roadMotorcycle, EngineUse::sportMotorcycle, EngineUse::cruiserMotorcycle,
             EngineUse::roadCar, EngineUse::sportCar }) {
        for (const auto litres : { 0.05, 0.08, 0.125, 0.2, 0.25 }) {
            EngineSpec small;
            small.name = "Small single";
            small.use = use;
            small.cylinderCount = 1;
            small.displacementLitres = litres;
            const auto designedSmall = designed(small);
            require(designedSmall.config->transmission.finalDriveRatio <= maximumFinalDriveRatio,
                "a small engine designs, its values held within the simulator's limits");
        }
    }
    EngineSpec tiny;
    tiny.name = "Tiny";
    tiny.use = EngineUse::sportMotorcycle;
    tiny.cylinderCount = 1;
    tiny.displacementLitres = 0.125;
    const auto tinyEngine = designed(tiny);
    const auto* inertia = valueOf(tinyEngine, "rotating_inertia_kg_m2");
    require(inertia && inertia->value == minimumRotatingInertiaKgM2 && inertia->rule.find("floor") != std::string::npos,
        "a value raised to a floor says so");
    tiny.overrides["rotating_inertia_kg_m2"] = 0.005;
    const auto tooLight = designEngine(tiny, parts());
    require(!tooLight.config && tooLight.error.find("rotatingInertiaKgM2") != std::string::npos,
        "a value set below a floor is refused by name");
    tiny.overrides.clear();
    tiny.displacementLitres = 0.025;
    require(designEngine(tiny, parts()).config.has_value(), "a 25 cc single designs");
    tiny.displacementLitres = 0.008;
    const auto tooSmall = designEngine(tiny, parts());
    require(!tooSmall.config && tooSmall.error.find("10 cc") != std::string::npos,
        "under 10 cc is refused with the simulator's limit");

    auto twice = inlineSeven();
    twice.firingOrder = { 1, 2, 2, 4, 5, 6, 7 };
    require(!designEngine(twice, parts()).config, "a cylinder named twice in the firing order is refused");

    auto uneven = inlineSeven();
    uneven.cylinderCount = 6;
    uneven.layout = EngineLayout::custom;
    uneven.bankCount = 4;
    uneven.bankAngleDegrees = 15.0;
    require(!designEngine(uneven, parts()).config, "six cylinders on four banks are refused");
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
    /** Lowest and highest speed over the measured window. */
    double minRpm { 1.0e9 };
    double maxRpm { 0.0 };
};

// MSVC 17.4 fails with C1001 compiling this loop optimised; the simulator it
// drives is compiled optimised in its library.
#if defined(_MSC_VER)
#pragma optimize("", off)
#endif
/** Cranked, then `throttle` held (against a dyno at `dynoRpm` if not zero);
 *  means over the last 2 s of 8. */
[[nodiscard]] Held held(const EngineConfig& config, double throttle, double dynoRpm, double dynoNmPerRpm = 2.0) {
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
        if (t >= 2.0 && dynoRpm > 0.0) controls.dynamometerTorqueNm = std::max(0.0, dynoNmPerRpm * (speed - dynoRpm));
        const auto state = rig->simulator->step(frameSeconds, controls).state;
        speed = state.rpm;
        if (frame < firstMeasured) continue;
        result.rpm += state.rpm * weight;
        result.torqueNm += state.torqueNm * weight;
        result.minRpm = std::min(result.minRpm, state.rpm);
        result.maxRpm = std::max(result.maxRpm, state.rpm);
    }
    return result;
}
#if defined(_MSC_VER)
#pragma optimize("", on)
#endif

void inlineSevenRuns() {
    const auto config = *designed(inlineSeven()).config;
    const auto idle = held(config, 0.0, 0.0).rpm;
    std::cout << "  designed I7 idles at " << idle << " rpm (target " << config.idleRpm << ")\n";
    require(std::abs(idle - config.idleRpm) < 0.3 * config.idleRpm, "the designed I7 idles near its target");
    const auto torque = held(config, 1.0, 0.6 * config.redlineRpm).torqueNm;
    std::cout << "  full load at " << 0.6 * config.redlineRpm << " rpm: " << torque << " N.m\n";
    const auto bmepBar = torque * 4.0 * 3.14159265 / (engineDisplacementLitres(config) * 1.0e-3) * 1.0e-5;
    require(bmepBar > 10.0 && bmepBar < 15.0, "and makes the torque of a good NA engine (BMEP 10-15 bar)");
}

/** The designer has no size cap: a 30 L radial (refused at 20 L before
 *  0.6.0) and a 25 cc single (refused under 50 cc) both build and pull at
 *  full throttle, held by an absorber that does not swing them. The single
 *  reads 7.7 bar where real 25 cc engines make 5-10, the radial 12.7. */
void enginesOfAnySizeRun() {
    EngineSpec radial;
    radial.name = "Radial Nine";
    radial.use = EngineUse::aircraft;
    radial.layout = EngineLayout::radial;
    radial.cylinderCount = 9;
    radial.boreMm = 155.6;
    radial.strokeMm = 174.6;
    EngineSpec single;
    single.name = "Small Single";
    single.use = EngineUse::roadMotorcycle;
    single.cylinderCount = 1;
    single.displacementLitres = 0.025;
    for (const auto& spec : { radial, single }) {
        const auto config = *designed(spec).config;
        const auto litres = engineDisplacementLitres(config);
        const auto dynoRpm = 0.6 * config.redlineRpm;
        // The absorber acts once a frame on the frame before's speed: stiffer
        // than half the shaft's inertia a frame, it overshoots, and on the
        // single's 0.01 kg m2 the 2 N.m/rpm it was given braked all or nothing
        // (9,404-9,626 rpm about 9,600, 16.4 bar; the test failed under GCC).
        const auto stableNmPerRpm = 0.5 * config.rotatingInertiaKgM2 / frameSeconds * 3.14159265 / 30.0;
        const auto run = held(config, 1.0, dynoRpm, std::min(2.0 * std::max(1.0, litres / 2.0), stableNmPerRpm));
        const auto bmepBar = run.torqueNm * 4.0 * 3.14159265 / (litres * 1.0e-3) * 1.0e-5;
        std::cout << "  " << spec.name << " (" << litres << " L) at " << run.rpm << " rpm: " << run.torqueNm
                  << " N.m, BMEP " << bmepBar << " bar, " << run.minRpm << "-" << run.maxRpm << " rpm\n";
        require(run.maxRpm - run.minRpm < 100.0, spec.name + " is held at its speed, not swung about it");
        require(bmepBar > 4.0 && bmepBar < 20.0, spec.name + " pulls at full throttle");
    }
}

/** A catalogue file may hold a design instead of a whole engine. */
[[nodiscard]] std::size_t pinsOf(const EngineConfig& config) {
    std::vector<std::uint32_t> ids;
    for (const auto& cylinder : config.cylinders) ids.push_back(cylinder.crankJournalId);
    std::sort(ids.begin(), ids.end());
    return static_cast<std::size_t>(std::unique(ids.begin(), ids.end()) - ids.begin());
}

/** Each cylinder reaches top dead centre where its pin points along its bore:
 *  the journal the 3-D view and the kinematics draw agrees with the phase the
 *  combustion runs on. */
void pinsMatchTheFiring(const EngineConfig& config) {
    for (const auto& cylinder : config.cylinders) {
        const auto journal = std::find_if(config.crankJournals.begin(), config.crankJournals.end(),
            [&cylinder](const auto& item) { return item.id == cylinder.crankJournalId; });
        require(journal != config.crankJournals.end(), "every cylinder is on a pin");
        const auto gap = std::fmod(journal->angleDegrees + cylinder.bankOffsetDegrees
            - cylinder.crankOffsetDegrees + 7'200.0, 360.0);
        require(gap < 1.0e-6 || gap > 360.0 - 1.0e-6, "its pin points along its bore at its top dead centre");
    }
}

[[nodiscard]] std::vector<double> sortedIntervals(const EngineConfig& config) {
    auto intervals = analyseCrankBalance(config).firingIntervalsDegrees;
    std::sort(intervals.begin(), intervals.end());
    return intervals;
}

[[nodiscard]] EngineSpec vee(std::uint32_t cylinders, double bankAngle, EngineUse use, double litres) {
    EngineSpec spec;
    spec.name = "V" + std::to_string(cylinders);
    spec.use = use;
    spec.layout = EngineLayout::vLayout;
    spec.cylinderCount = cylinders;
    spec.bankAngleDegrees = bankAngle;
    spec.displacementLitres = litres;
    return spec;
}

[[nodiscard]] const ExhaustComponentConfig* componentOf(const EngineConfig& config, std::uint32_t id) {
    for (const auto& path : config.exhaustPaths) {
        if (!path.network) continue;
        for (const auto& component : path.network->components)
            if (component.id == id) return &component;
    }
    return nullptr;
}

[[nodiscard]] std::size_t outletsOf(const EngineConfig& config) {
    std::size_t outlets = 0;
    for (const auto& path : config.exhaustPaths)
        if (path.network)
            outlets += static_cast<std::size_t>(std::count_if(path.network->components.begin(),
                path.network->components.end(),
                [](const ExhaustComponentConfig& part) { return part.type == ExhaustComponentType::outlet; }));
    return outlets;
}

/** The exhaust is set in the designer, before the engine is built: one pipe
 *  or two, an H- or an X-pipe, the mid pipe, the silencer and the tailpipe
 *  (the owner's report, 2026-10-29: every design came with a long pipe that
 *  could only be changed node by node afterwards). */
void exhaustIsDesigned() {
    auto v8 = vee(8, 90.0, EngineUse::sportCar, 6.2);
    const auto family = designed(v8);
    require(family.config->exhaustPaths.size() == 2 && outletsOf(*family.config) == 2,
        "a V8 keeps the family's pipe per bank when nothing is chosen");

    v8.exhaustSystem = ExhaustSystem::single;
    const auto single = designed(v8);
    require(single.config->exhaustPaths.size() == 1 && outletsOf(*single.config) == 1
            && componentOf(*single.config, 300) && componentOf(*single.config, 300)->type == ExhaustComponentType::merge,
        "a single system meets the banks in a Y and leaves through one tip");

    v8.exhaustSystem = ExhaustSystem::dual;
    const auto dual = designed(v8);
    require(dual.config->exhaustPaths.size() == 2 && outletsOf(*dual.config) == 2, "dual: a pipe per bank");

    v8.exhaustSystem = ExhaustSystem::dualHPipe;
    const auto h = designed(v8);
    const auto* balance = componentOf(*h.config, 460);
    require(h.config->exhaustPaths.size() == 1 && outletsOf(*h.config) == 2 && balance
            && std::abs(balance->lengthMm - 250.0) < 1.0e-9
            && componentOf(*h.config, 450)->type == ExhaustComponentType::splitter
            && componentOf(*h.config, 451)->type == ExhaustComponentType::merge,
        "an H-pipe: a balance pipe between a splitter on one pipe and a merge on the other");

    v8.exhaustSystem = ExhaustSystem::dualXPipe;
    v8.overrides["exhaust_x_pipe_coupling"] = 0.5;
    const auto x = designed(v8);
    const auto* crossing = componentOf(*x.config, 455);
    require(x.config->exhaustPaths.size() == 1 && outletsOf(*x.config) == 2 && crossing
            && crossing->type == ExhaustComponentType::crossover && std::abs(crossing->crossoverCoupling - 0.5) < 1.0e-9,
        "an X-pipe crossing the two pipes, its coupling as set");
    v8.overrides.clear();

    auto four = inlineSeven();
    four.cylinderCount = 4;
    four.exhaustSystem = ExhaustSystem::dual;
    const auto split = designed(four);
    require(outletsOf(*split.config) == 2 && componentOf(*split.config, 450)->type == ExhaustComponentType::splitter,
        "an inline engine's dual system splits its collector");
    four.exhaustSystem = ExhaustSystem::dualHPipe;
    const auto refused = designEngine(four, parts());
    require(!refused.config && refused.error.find("one bank") != std::string::npos,
        "an H-pipe on one bank is refused, saying why");
    auto turbo = vee(8, 90.0, EngineUse::sportCar, 4.4);
    turbo.induction = Induction::turbocharged;
    turbo.exhaustSystem = ExhaustSystem::dualXPipe;
    require(!designEngine(turbo, parts()).config, "an X-pipe ahead of the turbines is refused");

    four.exhaustSystem = ExhaustSystem::single;
    four.exhaustSilencer = ExhaustSilencer::none;
    four.overrides["exhaust_midpipe_length_mm"] = 800.0;
    four.overrides["exhaust_tailpipe_length_mm"] = 300.0;
    const auto shortPipe = designed(four);
    const auto* mid = componentOf(*shortPipe.config, 500);
    const auto* tail = componentOf(*shortPipe.config, 650);
    const auto mufflers = std::count_if(shortPipe.config->exhaustPaths.front().network->components.begin(),
        shortPipe.config->exhaustPaths.front().network->components.end(),
        [](const ExhaustComponentConfig& part) { return part.type == ExhaustComponentType::muffler; });
    require(mid && std::abs(mid->lengthMm - 800.0) < 1.0e-9 && tail && std::abs(tail->lengthMm - 300.0) < 1.0e-9
            && mufflers == 0,
        "the mid pipe and the tailpipe take the lengths set, and no silencer is none");
    four.exhaustSilencer = ExhaustSilencer::packedBox;
    four.overrides["exhaust_midpipe_diameter_mm"] = 60.0;
    const auto boxed = designed(four);
    const auto* box = componentOf(*boxed.config, 600);
    require(box && box->type == ExhaustComponentType::muffler && std::abs(box->diameterMm - 60.0) < 1.0e-9
            && std::abs(box->lengthMm - 450.0) < 1.0e-9,
        "a packed box sized from its pipe (7.5 x 60 mm long)");
}

/** An H-pipe's balance pipe carries flow both ways: the solvers' junctions
 *  have no preferred direction, so the side that pulses pushes into the
 *  other, whichever the graph names the splitter. */
void hPipeFlowsBothWays() {
    auto spec = vee(8, 90.0, EngineUse::sportCar, 6.2);
    spec.exhaustSystem = ExhaustSystem::dualHPipe;
    const auto config = *designed(spec).config;
    auto rig = std::make_unique<Rig>(config);
    auto speed = 0.0;
    auto most = 0.0;
    auto least = 0.0;
    for (auto frame = 0; frame < 5 * 240; ++frame) {
        const auto t = frame * frameSeconds;
        EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = t < 1.5;
        controls.throttle = t < 2.0 ? 0.0 : 1.0;
        if (t >= 2.0) controls.dynamometerTorqueNm = std::max(0.0, 4.0 * (speed - 3'000.0));
        speed = rig->simulator->step(frameSeconds, controls).state.rpm;
        if (t < 3.5) continue;
        rig->simulator->captureGasFieldNow();
        // The balance pipe is a cell or two at the production mesh: what
        // drives it is the pressure of the junction at each end.
        auto splitter = 0.0;
        auto merge = 0.0;
        for (const auto& element : rig->simulator->gasField().elements) {
            if (element.kind != GasFieldElementKind::exhaustJunction) continue;
            if (element.componentId == 450) splitter = element.pressurePa[0];
            if (element.componentId == 451) merge = element.pressurePa[0];
        }
        require(splitter > 0.0 && merge > 0.0, "the H's two junctions are in the gas field");
        most = std::max(most, splitter - merge);
        least = std::min(least, splitter - merge);
    }
    std::cout << "  H-pipe balance pipe at " << speed << " rpm: splitter minus merge " << least << " to " << most
              << " Pa\n";
    require(most > 500.0 && least < -500.0, "the balance pipe is driven from either side over the cycle");
}

[[nodiscard]] EngineSpec sportFour() {
    EngineSpec spec;
    spec.name = "Sport four";
    spec.use = EngineUse::sportCar;
    spec.cylinderCount = 4;
    spec.boreMm = 86.0;
    spec.strokeMm = 86.0;
    return spec;
}

[[nodiscard]] EngineSpec twinTwoStroke() {
    EngineSpec spec;
    spec.name = "Two-stroke twin";
    spec.cycle = EngineCycle::twoStroke;
    spec.use = EngineUse::sportMotorcycle;
    spec.cylinderCount = 2;
    spec.displacementLitres = 0.347;
    return spec;
}

/** The fuel system and the intake are chosen in the designer, on either cycle
 *  (the owner's request, 2026-10-29): port or direct injection, or a
 *  carburettor; an airbox, a pod filter or open velocity stacks. */
void fuelSystemsAreDesigned() {
    auto spec = sportFour();
    const auto port = designed(spec);
    require(!port.config->intake.carburettor.enabled && port.config->injection.mode == InjectionMode::port,
        "a four-stroke injects into its ports when nothing is chosen");

    spec.fuelSystem = FuelSystem::directInjection;
    const auto direct = designed(spec);
    require(direct.config->injection.mode == InjectionMode::direct
            && std::abs(direct.config->injection.startAngleDegrees - 420.0) < 1.0e-9
            && std::abs(direct.config->injection.endAngleDegrees - 600.0) < 1.0e-9,
        "a direct-injected petrol four injects on its intake stroke, for a homogeneous charge");

    spec.fuelSystem = FuelSystem::carburettor;
    const auto carb = designed(spec);
    const auto& carburettor = carb.config->intake.carburettor;
    require(carburettor.enabled && carburettor.fixedVenturi && carburettor.acceleratorPumpStrokeCc > 0.0
            && carburettor.starterJetFlowNumber > 0.0 && carburettor.venturiDiameterMm > 0.0,
        "a carburetted four-stroke gets a fixed venturi, an accelerator pump and a starter circuit");
    // The venturi is sized for 5.1 kPa at peak power: a 2-litre four's at
    // 0.90 of its swept volume every other turn.
    require(carburettor.venturiDiameterMm > 30.0 && carburettor.venturiDiameterMm < 45.0,
        "a 2-litre four's single venturi is 30-45 mm");

    auto diesel = spec;
    diesel.fuel = FuelType::diesel;
    const auto refused = designEngine(diesel, parts());
    require(!refused.config && refused.error.find("direct injection") != std::string::npos,
        "a carburetted diesel is refused, saying why");

    auto twin = twinTwoStroke();
    const auto twinCarb = designed(twin);
    require(twinCarb.config->intake.carburettor.enabled && !twinCarb.config->intake.carburettor.fixedVenturi
            && twinCarb.config->intake.carburettor.starterJetFlowNumber == 0.0,
        "a two-stroke keeps its slide carburettor, with no starter circuit");
    twin.fuelSystem = FuelSystem::directInjection;
    const auto twinDirect = designed(twin);
    require(!twinDirect.config->intake.carburettor.enabled
            && twinDirect.config->injection.mode == InjectionMode::direct,
        "a two-stroke can inject into its cylinders");

    spec = sportFour();
    spec.intakeStyle = IntakeStyle::podFilter;
    const auto pod = designed(spec);
    require(pod.config->intake.podFilter && pod.config->intake.airboxVolumeLitres == 0.0,
        "a pod filter has no box");
    spec.intakeStyle = IntakeStyle::velocityStacks;
    const auto stacks = designed(spec);
    require(!stacks.config->intake.podFilter && stacks.config->intake.airboxVolumeLitres == 0.0
            && stacks.config->intake.bellmouthDiameterMm > stacks.config->intake.throttleDiameterMm,
        "velocity stacks draw from the air through a bellmouth wider than the throttle");
}

/** What the designer derives survives a save: the throttle's closed leak was
 *  never written, so a designed engine reloaded ran on the 0.6 mm2 default. */
void designedValuesSurviveASave() {
    auto spec = sportFour();
    spec.fuelSystem = FuelSystem::carburettor;
    spec.intakeStyle = IntakeStyle::podFilter;
    const auto config = *designed(spec).config;
    const YamlEngineSerializer serializer;
    const auto decoded = serializer.decode(serializer.encode(config));
    require(decoded.config.has_value(), "a designed engine reloads");
    const auto& before = config.intake;
    const auto& after = decoded.config->intake;
    require(after.closedThrottleLeakageAreaMm2 == before.closedThrottleLeakageAreaMm2
            && after.podFilter == before.podFilter
            && after.carburettor.fixedVenturi == before.carburettor.fixedVenturi
            && after.carburettor.acceleratorPumpStrokeCc == before.carburettor.acceleratorPumpStrokeCc
            && after.carburettor.starterJetFlowNumber == before.carburettor.starterJetFlowNumber,
        "the closed leak, the pod filter and the carburettor's circuits are saved");
    require(std::abs(before.closedThrottleLeakageAreaMm2 - 0.6) > 1.0e-3,
        "and the designed leak is not the default the loss went unseen behind");
}

/** A carburetted four-stroke starts on its starter circuit, idles on its
 *  pilot, and pulls from idle at part throttle. A slide's needle on a car's
 *  fixed venturi held the main jet to a quarter of its area there, and the
 *  engine went to lambda 1.7-3.6 and stalled. */
void carburettedFourPullsFromIdle() {
    auto spec = sportFour();
    spec.fuelSystem = FuelSystem::carburettor;
    const auto config = *designed(spec).config;
    auto rig = std::make_unique<Rig>(config);
    auto idle = 0.0;
    auto pulled = 0.0;
    auto lambda = 0.0;
    for (auto frame = 0; frame < static_cast<int>(7.5 * 240); ++frame) {
        const auto t = frame * frameSeconds;
        EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = t < 6.0 && rig->simulator->state().rpm < 0.85 * config.idleRpm;
        controls.throttle = t < 6.0 ? 0.0 : 0.15;
        const auto state = rig->simulator->step(frameSeconds, controls).state;
        if (t > 5.0 && t < 6.0) idle = state.rpm;
        if (t > 7.0) {
            pulled = state.rpm;
            lambda = state.lambda;
        }
    }
    std::cout << "  carburetted four idles at " << idle << " rpm, at 15 % throttle " << pulled
              << " rpm, lambda " << lambda << '\n';
    require(idle > 0.6 * config.idleRpm && idle < 1.5 * config.idleRpm, "it starts and idles");
    require(pulled > 3'000.0 && lambda > 0.75 && lambda < 1.05,
        "and pulls past 3,000 rpm at 15 % throttle, neither lean nor drowned");
}

/** A Roots supercharger is a displacement geared to the crank: its boost
 *  rises with speed as its leak's share falls, and at part throttle the
 *  manifold's vacuum opens its bypass, so it neither boosts nor pumps there.
 *  A fraction of the authored ratio scaled by the throttle boosted a
 *  manifold standing at 70 kPa, and was flat from 30 % of the redline. */
void rootsSuperchargerFollowsItsDisplacement() {
    auto spec = sportFour();
    spec.use = EngineUse::roadCar;
    spec.boreMm = 77.0;
    spec.strokeMm = 85.8;
    spec.induction = Induction::supercharged;
    spec.boostPressureRatio = 1.75;
    const auto config = *designed(spec).config;
    require(config.forcedInduction.superchargerLobeCount > 0
            && config.forcedInduction.superchargerDisplacementLitres > 0.1,
        "a road car's supercharger is a Roots with a displacement");
    const auto held = [&](double rpm, double throttle) {
        auto rig = std::make_unique<Rig>(config);
        auto speed = 0.0;
        EngineState state;
        for (auto frame = 0; frame < 4 * 240; ++frame) {
            const auto t = frame * frameSeconds;
            EngineControls controls;
            controls.ignitionEnabled = true;
            controls.starterEngaged = t < 1.0;
            controls.throttle = t < 1.5 ? 0.0 : throttle;
            if (t >= 1.5) controls.dynamometerTorqueNm = std::max(0.0, 4.0 * (speed - rpm));
            state = rig->simulator->step(frameSeconds, controls).state;
            speed = state.rpm;
        }
        return state;
    };
    const auto low = held(2'000.0, 1.0);
    const auto high = held(5'500.0, 1.0);
    const auto cruise = held(3'000.0, 0.15);
    std::cout << "  Roots: WOT boost " << low.boostPressureRatio << " at " << low.rpm << " rpm, "
              << high.boostPressureRatio << " at " << high.rpm << "; 15 % throttle: map "
              << cruise.manifoldPressureKpa << " kPa, boost " << cruise.boostPressureRatio << ", "
              << cruise.compressorPowerKw << " kW\n";
    require(high.boostPressureRatio > low.boostPressureRatio + 0.1,
        "at full throttle its boost rises with the speed");
    require(cruise.manifoldPressureKpa < 90.0 && cruise.boostPressureRatio < 1.03
            && cruise.compressorPowerKw < 0.5,
        "at part throttle the bypass carries its air: no boost, next to no work");
}

/** A V's two cylinders of a row share one crank pin, as on the real engines;
 *  each had its own pin, placed for even firing (the owner's report,
 *  2026-10-19). */
void vCranksSharePins() {
    const auto ducati = designed(vee(2, 90.0, EngineUse::sportMotorcycle, 0.9));
    require(pinsOf(*ducati.config) == 1, "a V-twin's two rods share one pin");
    require(sortedIntervals(*ducati.config) == std::vector<double>({ 270.0, 450.0 }),
        "so a 90 deg twin fires 270-450");
    pinsMatchTheFiring(*ducati.config);
    const auto harley = designed(vee(2, 45.0, EngineUse::cruiserMotorcycle, 1.6));
    require(pinsOf(*harley.config) == 1 && sortedIntervals(*harley.config) == std::vector<double>({ 315.0, 405.0 }),
        "and a 45 deg twin 315-405");
    pinsMatchTheFiring(*harley.config);

    const auto v8 = designed(vee(8, 90.0, EngineUse::sportCar, 6.2));
    require(pinsOf(*v8.config) == 4, "a V8 has four pins");
    evenlySpaced(*v8.config);
    pinsMatchTheFiring(*v8.config);
    const auto v12 = designed(vee(12, 60.0, EngineUse::sportCar, 6.0));
    require(pinsOf(*v12.config) == 6, "a 60 deg V12 six");
    evenlySpaced(*v12.config);
    pinsMatchTheFiring(*v12.config);
    const auto v10 = designed(vee(10, 72.0, EngineUse::sportCar, 5.0));
    require(pinsOf(*v10.config) == 5, "a 72 deg V10 five");
    evenlySpaced(*v10.config);
    const auto v4 = designed(vee(4, 90.0, EngineUse::sportMotorcycle, 1.0));
    require(pinsOf(*v4.config) == 2, "a V4 two");
    pinsMatchTheFiring(*v4.config);

    // A shared pin cannot fire a 90 deg V6 evenly: its pins are split by the
    // least that does (30 deg, as the even-firing ones), unless asked.
    auto sixSpec = vee(6, 90.0, EngineUse::roadCar, 3.8);
    const auto split = designed(sixSpec);
    require(pinsOf(*split.config) == 6, "a 90 deg V6 splits its pins");
    evenlySpaced(*split.config);
    pinsMatchTheFiring(*split.config);
    const auto& journals = split.config->crankJournals;
    for (std::size_t row = 0; row < 3; ++row) {
        const auto lead = std::abs(std::remainder(journals[2 * row + 1].angleDegrees
            - journals[2 * row].angleDegrees, 360.0));
        require(std::abs(lead - 30.0) < 1.0e-6, "by 30 deg within each row");
    }
    const auto* pins = valueOf(split, "crank_pins");
    require(pins && pins->value == 6.0 && pins->rule.find("split") != std::string::npos,
        "and the design says so");
    sixSpec.crankPins = CrankPins::shared;
    const auto oddFire = designed(sixSpec);
    require(pinsOf(*oddFire.config) == 3 && sortedIntervals(*oddFire.config).back() > 140.0,
        "shared, it fires unevenly, as the odd-fire ones");
    pinsMatchTheFiring(*oddFire.config);
    auto separateSpec = vee(8, 90.0, EngineUse::sportCar, 6.2);
    separateSpec.crankPins = CrankPins::separate;
    const auto separate = designed(separateSpec);
    require(separate.config->firingOrder
            == evenFiringOrders(balanceCylinders(*separate.config), 720.0, 1).front().order,
        "one pin per cylinder is still the best balanced even-firing crank");
    evenlySpaced(*separate.config);

    // A firing order given is kept, on shared pins when they allow it.
    auto ordered = vee(8, 90.0, EngineUse::sportCar, 6.2);
    ordered.firingOrder = v8.config->firingOrder;
    const auto kept = designed(ordered);
    require(kept.config->firingOrder == ordered.firingOrder && pinsOf(*kept.config) == 4,
        "a firing order a shared crank allows keeps four pins");
    ordered.firingOrder = { 1, 3, 2, 4, 5, 7, 6, 8 };
    const auto apart = designed(ordered);
    require(apart.config->firingOrder == ordered.firingOrder && pinsOf(*apart.config) == 8,
        "one it does not is still kept, on a pin per cylinder");
    evenlySpaced(*apart.config);
    pinsMatchTheFiring(*apart.config);

    EngineSpec w16;
    w16.name = "W16";
    w16.layout = EngineLayout::custom;
    w16.cylinderCount = 16;
    w16.bankCount = 4;
    w16.displacementLitres = 8.0;
    const auto w = designed(w16);
    evenlySpaced(*w.config);
    pinsMatchTheFiring(*w.config);
    std::cout << "crank pins:";
    for (const auto* engine : { &ducati, &harley, &v4, &split, &oddFire, &v8, &v10, &v12, &w }) {
        const auto* value = valueOf(*engine, "crank_pins");
        require(value != nullptr, "every design says how its pins are shared");
        std::cout << "\n  " << engine->config->name << ' ' << engine->config->bankAngleDegrees << " deg: "
                  << value->value << ", " << value->rule;
    }
    std::cout << '\n';

    // Inline and flat engines keep one pin per cylinder.
    require(pinsOf(*designed(inlineSeven()).config) == 7, "an inline engine keeps a pin per cylinder");
    EngineSpec boxer;
    boxer.name = "Boxer Four";
    boxer.layout = EngineLayout::flat;
    boxer.cylinderCount = 4;
    boxer.displacementLitres = 2.0;
    require(pinsOf(*designed(boxer).config) == 4, "a boxer too");
}

void catalogueReadsDesigns() {
    const std::filesystem::path source = ENGINELAB_CATALOG_ROOT;
    const auto root = std::filesystem::temp_directory_path() / "enginelab-design-test";
    std::filesystem::remove_all(root);
    std::filesystem::create_directories(root / "engines");
    std::filesystem::create_directories(root / "voicing");
    std::filesystem::copy(source / "parts", root / "parts", std::filesystem::copy_options::recursive);
    std::filesystem::copy_file(source / "voicing" / "default.yaml", root / "voicing" / "default.yaml");
    {
        std::ofstream file(root / "engines" / "inline7.engine.yaml");
        file << "schema_version: 1\nfamily: designed\ndesign:\n  name: Inline Seven\n  cylinders: 7\n"
                "  bore_mm: 84\n  stroke_mm: 72\n  overrides: { idle_rpm: 900 }\n";
    }
    const auto catalogue = loadEngineCatalog(root);
    for (const auto& error : catalogue.errors) std::cerr << "  " << error << '\n';
    require(catalogue.errors.empty() && catalogue.entries.size() == 1, "a design file loads");
    const auto& config = catalogue.entries.front().config;
    require(config.cylinders.size() == 7 && config.idleRpm == 900.0, "as the engine it describes");
    {
        std::ofstream file(root / "engines" / "inline7.engine.yaml");
        file << "schema_version: 1\ndesign:\n  name: Inline Seven\n  cylinders: 7\n  bore_mm: 84\n"
                "  stroke_mm: 72\n  stroke: 72\n";
    }
    require(loadEngineCatalog(root).entries.empty(), "an unknown design key is refused");
    std::filesystem::remove_all(root);
}

/** The designer runs a design off-screen before building it; the probe holds
 * the catalogue's measuring speed and reports a factor, or why it has none. */
void realtimeIsChecked() {
    const auto designedEngine = designCatalogueEngine(inlineSeven(), ENGINELAB_CATALOG_ROOT);
    require(designedEngine.config.has_value(), "the I7 designs with its voicing");
    const auto& config = *designedEngine.config;
    const auto capacity = measureRealtimeCapacity(config, 1.0, 60.0);
    std::cout << "  designed I7: physics at " << capacity.factor << " x real time at " << capacity.holdRpm
              << " rpm" << (capacity.error.empty() ? "" : " (" + capacity.error + ")") << '\n';
    require(capacity.error.empty() && capacity.factor > 0.0 && std::isfinite(capacity.factor),
        "the probe measures a factor");
    require(capacity.holdRpm == std::min(5'000.0, 0.95 * std::min(config.redlineRpm, config.ignition.revLimitRpm)),
        "at the catalogue's measuring speed");
    const std::atomic<bool> cancelled { true };
    require(measureRealtimeCapacity(config, 1.0, 60.0, &cancelled).error == "cancelled", "and can be cancelled");

    // A single's speed swings some +4 / -6 % within each cycle at 5,000 rpm
    // on the dyno: the hold is judged on the mean speed, not on one sample.
    const auto heldAfter = [](auto speedAt) {
        RealtimeHoldJudge judge(5'000.0);
        for (auto t = 0.0; t < 10.0; t += 0.0117)
            if (judge.add(t, speedAt(t))) return t;
        return -1.0;
    };
    const auto rippling = heldAfter([](double t) { return 5'000.0 + 250.0 * std::sin(2.0 * 3.14159265 * 41.7 * t); });
    std::cout << "  a +-5 % rippling 5,000 rpm is held after " << rippling << " s\n";
    require(rippling > 0.0 && rippling < 1.5, "a speed rippling +-5 % about the target is held");
    require(heldAfter([](double) { return 4'700.0; }) >= 1.0, "a speed settled 6 % short is taken, after a second");
    require(heldAfter([](double) { return 3'000.0; }) < 0.0, "one 40 % short is not");
    const auto ramp = heldAfter([](double t) { return std::min(5'000.0, 1'000.0 + 2'000.0 * t); });
    require(ramp > 2.0, "nor a ramp before it arrives");
    const auto slowRamp = heldAfter([](double t) { return std::min(4'500.0, 3'000.0 + 400.0 * t); });
    std::cout << "  slow climb held after " << slowRamp << " s\n";
    require(slowRamp > 3.75, "nor a slow climb before it levels off");

    EngineSpec single;
    single.name = "Single";
    single.use = EngineUse::roadMotorcycle;
    single.cylinderCount = 1;
    single.displacementLitres = 0.5;
    const auto thumper = designCatalogueEngine(single, ENGINELAB_CATALOG_ROOT);
    require(thumper.config.has_value(), "a 500 cc single designs");
    const auto singleCapacity = measureRealtimeCapacity(*thumper.config, 1.0, 30.0);
    std::cout << "  designed single: physics at " << singleCapacity.factor << " x real time"
              << (singleCapacity.error.empty() ? "" : " (" + singleCapacity.error + ")") << '\n';
    require(singleCapacity.error.empty() && singleCapacity.factor > 0.0, "a single holds its measuring speed");

    // A carburetted two-stroke never catches cranked at full throttle (its
    // pilot is dead there and its venturi draws nothing at cranking speed):
    // the probe cranks it shut first, as a rider does.
    EngineSpec twin;
    twin.name = "Two-stroke Twin";
    twin.cycle = EngineCycle::twoStroke;
    twin.use = EngineUse::sportMotorcycle;
    twin.cylinderCount = 2;
    twin.boreMm = 64.0;
    twin.strokeMm = 54.0;
    const auto stinger = designCatalogueEngine(twin, ENGINELAB_CATALOG_ROOT);
    require(stinger.config.has_value(), "a two-stroke twin designs");
    const auto twinCapacity = measureRealtimeCapacity(*stinger.config, 1.0, 60.0);
    std::cout << "  designed two-stroke twin: physics at " << twinCapacity.factor << " x real time"
              << (twinCapacity.error.empty() ? "" : " (" + twinCapacity.error + ")") << '\n';
    require(twinCapacity.error.empty() && twinCapacity.factor > 0.0, "a carburetted two-stroke holds its measuring speed");
}

} // namespace

/** A rotary from its displacement: a 1.3 L twin-rotor gets the 13B's rotor
 *  (R 105, e 15, b 80 mm), six faces firing every 180 deg of the eccentric
 *  shaft, a passage per rotor into its primary, and pulls at full throttle.
 *  What a rotary cannot be is refused. */
void rotaryIsDesigned() {
    EngineSpec spec;
    spec.name = "Twin-Rotor";
    spec.cycle = EngineCycle::rotary;
    spec.use = EngineUse::sportCar;
    spec.cylinderCount = 2;
    spec.displacementLitres = 1.3094;
    spec.exhaustSystem = ExhaustSystem::single;
    const auto design = designed(spec);
    const auto& config = *design.config;
    require(near(config.rotary.generatingRadiusMm, 105.0, 0.002) && near(config.rotary.eccentricityMm, 15.0, 0.002)
            && near(config.rotary.rotorWidthMm, 80.0, 0.002) && config.rotary.rotorCount == 2
            && config.cylinders.size() == 6,
        "a 1.3 L twin-rotor is a 13B: R 105, e 15, b 80, six faces");
    require(config.firingOrder == std::vector<std::uint32_t> { 1, 4, 2, 5, 3, 6 },
        "its faces fire every 180 deg, a rotor's three 360 apart");
    require(std::abs(config.redlineRpm - 9'000.0) < 1.0 && std::abs(config.cylinders[0].compressionRatio - 10.0) < 1.0e-9,
        "a sport rotary revs to the RX-8's 9,000 on the Renesis' 10:1");
    const auto* passage = componentOf(config, 800);
    const auto& network = *config.exhaustPaths.front().network;
    const auto toPassage = std::count_if(network.cylinderConnections.begin(), network.cylinderConnections.end(),
        [](const auto& connection) { return connection.componentId == 800; });
    require(passage && passage->type == ExhaustComponentType::merge && toPassage == 3 && componentOf(config, 801),
        "a rotor's three faces open onto one exhaust passage");
    const auto graph = ExhaustGraph::makeForEngine(config);
    require(gasdynamics::ExhaustNetworkLayout::compile(graph, gasdynamics::realtimeExhaustFeedbackDiscretisation()).valid(),
        "and the designed rotary's exhaust compiles");
    const auto run = held(config, 1.0, 0.6 * config.redlineRpm);
    const auto bmepBar = run.torqueNm * 4.0 * 3.14159265 / (breathingDisplacementLitres(config) * 1.0e-3) * 1.0e-5;
    std::cout << "  designed twin-rotor at " << run.rpm << " rpm: " << run.torqueNm << " N.m, BMEP " << bmepBar
              << " bar on its 2.6 L swept in two turns\n";
    require(bmepBar > 7.0 && bmepBar < 15.0, "and pulls at full throttle");

    for (const auto& [change, why] : std::vector<std::pair<void (*)(EngineSpec&), const char*>> {
             { [](EngineSpec& refused) { refused.fuel = FuelType::diesel; }, "a diesel rotary" },
             { [](EngineSpec& refused) { refused.boreMm = 80.0; refused.strokeMm = 60.0; }, "a bore and a stroke" },
             { [](EngineSpec& refused) { refused.layout = EngineLayout::vLayout; }, "a V of rotors" },
             { [](EngineSpec& refused) { refused.cylinderCount = 11; }, "11 rotors (33 faces)" },
             { [](EngineSpec& refused) { refused.exhaustSystem = ExhaustSystem::dualHPipe; }, "an H-pipe on one bank" } }) {
        auto refused = spec;
        change(refused);
        require(!designEngine(refused, parts()).config, std::string("refused: ") + why);
    }
}

int main() {
    crankBalanceMatchesTheTextbook();
    firingOrdersAreRanked();
    inlineSevenFromNothing();
    overridesWin();
    everyLayoutDesigns();
    vCranksSharePins();
    exhaustIsDesigned();
    hPipeFlowsBothWays();
    fuelSystemsAreDesigned();
    designedValuesSurviveASave();
    carburettedFourPullsFromIdle();
    rootsSuperchargerFollowsItsDisplacement();
    catalogueReadsDesigns();
    inlineSevenRuns();
    enginesOfAnySizeRun();
    realtimeIsChecked();
    rotaryIsDesigned();
    std::cout << "engine design tests passed\n";
    return EXIT_SUCCESS;
}
