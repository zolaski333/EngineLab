// The engine builder: a recipe applied to a base engine gives a valid engine
// whose feeding follows its air, and that runs as its size says.

#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
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

} // namespace

int main() {
    recipeIsApplied();
    std::cout << "recipe: ok\n";
    strokerRuns();
    std::cout << "stroker runs: ok\n";
    return 0;
}
