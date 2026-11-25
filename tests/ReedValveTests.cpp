// The reed's petals (ReedValve.hpp): the spring-mass-damper reproduces the
// cantilever's static deflection and first bending mode, the seat and stop
// plate hold, and on an engine a petal lets gas back out of the crankcase,
// which the perfect check valve it replaces never did.
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/foundation/ReedValve.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <numbers>
#include <string>
#include <string_view>
#include <utility>

namespace {
using namespace enginelab;

void require(bool condition, std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
    std::cout << "ok: " << message << '\n';
}

// An illustrative glass-fibre petal pair, not a measured one: the tests check
// the model's laws, not a maker's reed.
[[nodiscard]] ReedValveConfig illustrativePetals() {
    ReedValveConfig reed;
    reed.petalCount = 2;
    reed.freeLengthMm = 30.0;
    reed.widthMm = 25.0;
    reed.thicknessMm = 0.30;
    reed.youngsModulusGpa = 20.0;
    reed.densityKgPerM3 = 1'900.0;
    reed.dampingRatio = 0.05;
    reed.stopLiftMm = 8.0;
    reed.dischargeCoefficient = 0.6;
    return reed;
}

// Textbook Euler-Bernoulli cantilever values, written out apart from the model.
struct Beam final {
    double ei;
    double rhoA;
    double lengthM;
    double widthM;
};
[[nodiscard]] Beam beamOf(const ReedValveConfig& reed) {
    const auto b = reed.widthMm * 1.0e-3;
    const auto t = reed.thicknessMm * 1.0e-3;
    return { reed.youngsModulusGpa * 1.0e9 * b * t * t * t / 12.0, reed.densityKgPerM3 * b * t,
             reed.freeLengthMm * 1.0e-3, b };
}

void aPetalDeflectsAsAUniformlyLoadedCantilever() {
    auto reed = illustrativePetals();
    reed.stopLiftMm = 50.0;
    reed.dampingRatio = 0.5;
    const auto dynamics = reedValveDynamics(reed);
    const auto beam = beamOf(reed);
    constexpr double dropPa = 300.0;
    // Tip deflection under a uniform load q: q L^4 / (8 E I).
    const auto q = dropPa * beam.widthM;
    const auto expectedM = q * std::pow(beam.lengthM, 4.0) / (8.0 * beam.ei);
    ReedValveState state;
    for (int step = 0; step < 200'000; ++step) state = advanceReedValve(dynamics, state, dropPa, 1.0e-6);
    std::cout << "static tip lift " << state.liftM * 1.0e3 << " mm, beam theory "
              << expectedM * 1.0e3 << " mm\n";
    require(std::abs(state.liftM / expectedM - 1.0) < 1.0e-6,
        "the settled lift is the cantilever's tip deflection under that pressure");
}

void aPetalRingsAtItsFirstBendingMode() {
    auto reed = illustrativePetals();
    reed.stopLiftMm = 50.0;
    reed.dampingRatio = 0.0;
    const auto dynamics = reedValveDynamics(reed);
    const auto beam = beamOf(reed);
    const auto expectedHz = 1.875104068711961 * 1.875104068711961 / (2.0 * std::numbers::pi)
        * std::sqrt(beam.ei / (beam.rhoA * std::pow(beam.lengthM, 4.0)));
    constexpr double dropPa = 300.0;
    const auto equilibrium = dropPa * dynamics.faceAreaM2 / dynamics.stiffnessNPerM;
    // Released at 1.5 x its equilibrium it swings between 0.5 and 1.5, never
    // touching the seat; count the upward crossings of the equilibrium.
    ReedValveState state { 1.5 * equilibrium, 0.0 };
    constexpr double dt = 2.0e-6;
    double firstCrossing = -1.0;
    double lastCrossing = -1.0;
    int crossings = 0;
    auto previous = state.liftM - equilibrium;
    for (int step = 1; step <= 300'000 && crossings < 21; ++step) {
        state = advanceReedValve(dynamics, state, dropPa, dt);
        const auto now = state.liftM - equilibrium;
        if (previous < 0.0 && now >= 0.0) {
            const auto time = (step - now / (now - previous)) * dt;
            if (crossings == 0) firstCrossing = time;
            lastCrossing = time;
            ++crossings;
        }
        previous = now;
    }
    const auto measuredHz = (crossings - 1) / (lastCrossing - firstCrossing);
    std::cout << "free vibration " << measuredHz << " Hz over " << crossings - 1
              << " periods, first cantilever mode " << expectedHz << " Hz\n";
    require(crossings == 21, "the undamped petal keeps ringing");
    require(std::abs(measuredHz / expectedHz - 1.0) < 1.0e-6,
        "at the cantilever's first bending frequency");
    require(std::abs(state.liftM - equilibrium) < 0.5 * equilibrium * (1.0 + 1.0e-9),
        "and its swing does not grow (the integration is exact)");
}

void theSeatAndTheStopHold() {
    const auto dynamics = reedValveDynamics(illustrativePetals());
    ReedValveState state;
    for (int step = 0; step < 10'000; ++step) state = advanceReedValve(dynamics, state, -20'000.0, 1.0e-5);
    require(state.liftM == 0.0 && state.velocityMPerS == 0.0, "pushed onto its seat the petal stays shut");
    require(reedValveFlowAreaM2(dynamics, state) == 0.0, "and passes nothing");
    auto maximum = 0.0;
    for (int step = 0; step < 10'000; ++step) {
        state = advanceReedValve(dynamics, state, 50'000.0, 1.0e-5);
        maximum = std::max(maximum, state.liftM);
    }
    require(maximum <= dynamics.stopLiftM && state.liftM == dynamics.stopLiftM,
        "a large drop lays it on the stop plate and never beyond");
    require(reedValveFlowAreaM2(dynamics, state) <= dynamics.windowAreaM2,
        "its curtain never exceeds the block's window");
    require(reedValveConfigError(ReedValveConfig {}) == nullptr, "no petal is the check valve, valid");
    auto broken = illustrativePetals();
    broken.dampingRatio = 1.0;
    require(reedValveConfigError(broken) != nullptr, "an overdamped petal is refused, not run");
}

[[nodiscard]] EngineConfig rd350() {
    static const auto catalogue = loadEngineCatalog(ENGINELAB_CATALOG_ROOT);
    const auto selected = selectSingleEngineCatalogEntry(catalogue.entries, std::string("RD350LC"));
    require(static_cast<bool>(selected), "the catalogue has the RD350LC");
    auto config = selected.entry->config;
    normaliseEngineConfig(config);
    return config;
}

struct Run final {
    double airGps;
    double rpm;
    double backflowG;
    double maximumLiftMm;
};

[[nodiscard]] Run runEngine(EngineConfig config) {
    SimpleEcuModel ecu;
    SimplifiedGasolinePhysics physics;
    FourStrokeEventGenerator events;
    auto exhaust = ExhaustGraph::makeForEngine(config);
    auto simulator = std::make_unique<EngineSimulator>(config, ecu, physics, events, exhaust);
    Run run { 0.0, 0.0, 0.0, 0.0 };
    int samples = 0;
    constexpr int frames = 3 * 240;
    for (int frame = 0; frame < frames; ++frame) {
        const auto time = frame / 240.0;
        const auto state = simulator->step(1.0 / 240.0, { true, time < 1.5, 1.0, 0.0 }).state;
        run.maximumLiftMm = std::max(run.maximumLiftMm, state.reedTipLiftMm);
        run.backflowG = state.reedBackflowKg * 1.0e3;
        if (frame >= frames - 120) {
            run.airGps += state.airFlowGramsPerSecond;
            run.rpm += state.rpm;
            ++samples;
        }
    }
    run.airGps /= samples;
    run.rpm /= samples;
    return run;
}

void aPetalLetsGasBackWhereTheCheckValveCannot() {
    // Until 2026-11-25 every reed was a perfect check valve: shut the instant
    // the crankcase rose above the carburettor, open the instant it fell,
    // with no inertia and no backflow. A petal has a mass and a stiffness,
    // so it is still open as the piston starts to compress the crankcase.
    auto config = rd350();
    const auto checkValve = runEngine(config);
    config.twoStroke.reed = illustrativePetals();
    require(!validateEngineConfig(config).has_value(), "the RD350LC with petals validates");
    const auto petals = runEngine(config);
    std::cout << "RD350LC, 1.5 s at WOT after the start: check valve " << checkValve.airGps << " g/s at "
              << checkValve.rpm << " rpm, backflow " << checkValve.backflowG << " g; petals "
              << petals.airGps << " g/s at " << petals.rpm << " rpm, backflow " << petals.backflowG
              << " g, highest tip lift " << petals.maximumLiftMm << " mm\n";
    require(checkValve.backflowG == 0.0 && checkValve.maximumLiftMm == 0.0,
        "the check valve has no lift and lets nothing back");
    require(petals.maximumLiftMm > 0.0 && petals.maximumLiftMm <= 8.0 + 1.0e-9,
        "the petals lift, and no further than their stop plate");
    require(petals.backflowG > 0.0, "and let gas back out of the crankcase");
    require(std::abs(petals.airGps / checkValve.airGps - 1.0) > 0.01,
        "which moves the engine's air flow");
}

} // namespace

int main() {
    aPetalDeflectsAsAUniformlyLoadedCantilever();
    aPetalRingsAtItsFirstBendingMode();
    theSeatAndTheStopHold();
    aPetalLetsGasBackWhereTheCheckValveCannot();
    std::cout << "reed valve tests passed\n";
    return EXIT_SUCCESS;
}
