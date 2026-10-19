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
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
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

    EngineSpec radial;
    radial.name = "Radial Nine";
    radial.use = EngineUse::aircraft;
    radial.layout = EngineLayout::radial;
    radial.cylinderCount = 9;
    radial.boreMm = 155.6;
    radial.strokeMm = 174.6;
    const auto tooBig = designEngine(radial, parts());
    require(!tooBig.config && tooBig.error.find("20 litres") != std::string::npos,
        "a 30 litre radial is refused with the simulator's limit");
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
    const auto& chamber = stinger.config->exhaustPaths.front().network;
    require(chamber && chamber->components.size() == 8, "each pipe is an expansion chamber");
    const auto* tuned = valueOf(stinger, "expansion_chamber_tuned_length_mm");
    require(tuned && tuned->value > 700.0 && tuned->value < 1'100.0, "tuned to its peak power speed");
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
};

// MSVC 17.4 fails with C1001 compiling this loop optimised; the simulator it
// drives is compiled optimised in its library.
#if defined(_MSC_VER)
#pragma optimize("", off)
#endif
/** Cranked, then `throttle` held (against a dyno at `dynoRpm` if not zero);
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

/** A catalogue file may hold a design instead of a whole engine. */
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

} // namespace

int main() {
    crankBalanceMatchesTheTextbook();
    firingOrdersAreRanked();
    inlineSevenFromNothing();
    overridesWin();
    everyLayoutDesigns();
    catalogueReadsDesigns();
    inlineSevenRuns();
    std::cout << "engine design tests passed\n";
    return EXIT_SUCCESS;
}
