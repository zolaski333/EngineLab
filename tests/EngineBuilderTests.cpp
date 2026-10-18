// The engine builder: a recipe applied to a base engine gives a valid engine
// whose feeding follows its air, and that runs as its size says.

#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/foundation/CrankBalance.hpp>
#include <enginelab/foundation/CylinderResize.hpp>
#include <enginelab/foundation/EngineBuilder.hpp>
#include <enginelab/foundation/IntakeResize.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string_view>
#include <vector>
#include <utility>

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
    double torqueNm { 0.0 };
    double lambda { 0.0 };
    double injectorDuty { 0.0 };
};

/** Cranked, then full throttle against a dyno holding `rpm`; means over the
 *  last 2 s of 8. */
[[nodiscard]] Held heldFullLoad(const EngineConfig& config, double rpm) {
    auto rig = std::make_unique<Rig>(config);
    constexpr int frames = 8 * 240;
    constexpr int firstMeasured = frames - 2 * 240;
    constexpr double weight = 1.0 / static_cast<double>(frames - firstMeasured);
    Held held;
    auto speed = 0.0;
    for (auto frame = 0; frame < frames; ++frame) {
        const auto t = frame * frameSeconds;
        EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = t < 1.5;
        controls.throttle = t < 2.0 ? 0.0 : 1.0;
        if (t >= 2.0) controls.dynamometerTorqueNm = std::max(0.0, 2.0 * (speed - rpm));
        const auto state = rig->simulator->step(frameSeconds, controls).state;
        speed = state.rpm;
        if (frame < firstMeasured) continue;
        held.torqueNm += state.torqueNm * weight;
        held.lambda += state.lambda * weight;
        auto duty = 0.0;
        for (std::size_t index = 0; index < std::min(state.cylinderStateCount, state.cylinderStates.size()); ++index)
            duty = std::max(duty, state.cylinderStates[index].injectorDutyCycle);
        held.injectorDuty += duty * weight;
    }
    return held;
}

void recipeIsApplied() {
    const auto base = makeDefaultInlineFour();
    require(!buildEngine(base, {}).config, "a nameless engine is refused");

    EngineRecipe recipe;
    recipe.name = "Stroker";
    const auto same = buildEngine(base, recipe);
    require(same.config.has_value(), "the base with only a name builds");
    require(same.config->name == "Stroker", "and takes it");

    const auto size = cylinderSize(base);
    require(size.has_value(), "the base has one cylinder size");
    recipe.strokeMm = size->strokeMm * 1.2;
    recipe.compressionRatio = 9.5;
    recipe.redlineRpm = base.redlineRpm * 0.9;
    const auto built = buildEngine(base, recipe);
    require(built.config.has_value(), "a longer stroke, lower compression and lower redline build");
    const auto& engine = *built.config;
    require(near(engineDisplacementLitres(engine), 1.2 * engineDisplacementLitres(base), 1.0e-9),
        "the displacement follows the stroke");
    require(std::all_of(engine.cylinders.begin(), engine.cylinders.end(),
                [](const CylinderConfig& cylinder) { return cylinder.compressionRatio == 9.5; }),
        "every cylinder takes the compression ratio");
    require(near(engine.ignition.revLimitRpm - engine.redlineRpm, base.ignition.revLimitRpm - base.redlineRpm, 1.0e-9),
        "the rev limiter keeps its margin over the redline");
    require(near(engine.ignition.timingCurve.back().rpm, 0.9 * base.ignition.timingCurve.back().rpm, 1.0e-9),
        "the ignition curve stretches with the redline");
    const auto airScale = 1.2 * 0.9;
    require(near(engine.injection.injectorFlowMgPerSecond, airScale * base.injection.injectorFlowMgPerSecond, 1.0e-9),
        "the injectors follow the air flow");
    auto normalisedBase = base;
    normaliseEngineConfig(normalisedBase);
    const auto pathId = engine.intakePaths.front().id;
    const auto throttle = intakePartSize(engine, pathId, IntakePart::throttle);
    const auto baseThrottle = intakePartSize(normalisedBase, pathId, IntakePart::throttle);
    require(throttle && baseThrottle && near(throttle->first, std::sqrt(airScale) * baseThrottle->first, 1.0e-6),
        "the throttle keeps the air speed");

    recipe = {};
    recipe.name = "Turbo";
    recipe.aspiration = Aspiration::turbocharged;
    recipe.turbochargerCount = 2;
    const auto turbo = buildEngine(base, recipe);
    require(turbo.config && turbo.config->forcedInduction.enabled
            && turbo.config->forcedInduction.turbochargerCount == 2U, "a twin-turbo version builds");
    require(turbo.config->injection.injectorFlowMgPerSecond
            > 1.2 * base.injection.injectorFlowMgPerSecond, "with injectors for the boost");
    recipe.aspiration = Aspiration::natural;
    const auto natural = buildEngine(*turbo.config, recipe);
    require(natural.config && !natural.config->forcedInduction.enabled, "and back to natural aspiration");

    recipe = {};
    recipe.name = "Too big";
    recipe.boreMm = 1'000.0;
    const auto refused = buildEngine(base, recipe);
    require(!refused.config && !refused.error.empty(), "an impossible bore is refused with a reason");
}

/** A stroker runs: its torque follows its displacement and its injectors
 *  keep up. */
void strokerRuns() {
    const auto base = makeDefaultInlineFour();
    EngineRecipe recipe;
    recipe.name = "Stroker";
    recipe.strokeMm = cylinderSize(base)->strokeMm * 1.2;
    const auto stroker = buildEngine(base, recipe);
    require(stroker.config.has_value(), "the stroker builds");
    const auto rpm = 0.6 * base.redlineRpm;
    const auto before = heldFullLoad(base, rpm);
    const auto after = heldFullLoad(*stroker.config, rpm);
    std::cout << "  base " << before.torqueNm << " N.m, lambda " << before.lambda << ", duty " << before.injectorDuty
              << "; stroker x1.2 " << after.torqueNm << " N.m, lambda " << after.lambda << ", duty "
              << after.injectorDuty << '\n';
    require(before.torqueNm > 50.0, "the base makes torque (else this proves nothing)");
    require(after.torqueNm > 1.05 * before.torqueNm && after.torqueNm < 1.35 * before.torqueNm,
        "20 % more displacement makes 5-35 % more torque at the same speed");
    require(std::abs(after.lambda - before.lambda) < 0.08, "the mixture stays where the base's was");
    require(after.injectorDuty < 0.95, "the injectors are not saturated");
}

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

} // namespace

int main() {
    crankBalanceMatchesTheTextbook();
    std::cout << "crank balance: ok\n";
    firingOrdersAreRanked();
    std::cout << "firing orders: ok\n";
    recipeIsApplied();
    std::cout << "recipe: ok\n";
    strokerRuns();
    std::cout << "stroker runs: ok\n";
    return 0;
}
