// Peak firing pressure: the simulator tracks each cylinder's cycle peak at
// every sub-step, and a peak above the engine's structural limit damages it.

#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/diagnostics/EngineDiagnostics.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
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

/** The default limits follow the fuel and induction; an authored one wins. */
void limitsFollowTheEngine() {
    auto config = makeDefaultInlineFour();
    config.forcedInduction.enabled = false;
    require(peakCylinderPressureLimitBar(config) == 120.0, "a naturally aspirated petrol engine: 120 bar");
    config.forcedInduction.enabled = true;
    require(peakCylinderPressureLimitBar(config) == 140.0, "a boosted petrol engine: 140 bar");
    config.fuel = FuelType::diesel;
    require(peakCylinderPressureLimitBar(config) == 200.0, "a diesel: 200 bar");
    config.peakFiringPressureLimitBar = 75.0;
    require(peakCylinderPressureLimitBar(config) == 75.0, "an authored limit wins");
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

struct HeldRun final {
    double meanPeakBar { 0.0 };
    double highestFrameSampleBar { 0.0 };
    double damage { 0.0 };
    bool peakPressureDiagnostic { false };
};

/** Cranked, then full throttle against a dyno that holds 0.6 x redline for
 *  `seconds`; peak means over the last 4 s. */
[[nodiscard]] HeldRun heldFullLoad(const EngineConfig& config, double seconds) {
    auto rig = std::make_unique<Rig>(config);
    const auto frames = static_cast<int>(seconds * 240.0);
    const auto firstMeasured = frames - 4 * 240;
    HeldRun run;
    auto rpm = 0.0;
    EngineState last;
    for (auto frame = 0; frame < frames; ++frame) {
        const auto t = frame * frameSeconds;
        EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = t < 1.5;
        controls.throttle = t < 2.0 ? 0.0 : 1.0;
        if (t >= 2.0)
            controls.dynamometerTorqueNm = std::max(0.0, 2.0 * (rpm - 0.6 * config.redlineRpm));
        last = rig->simulator->step(frameSeconds, controls).state;
        rpm = last.rpm;
        if (frame >= firstMeasured) {
            run.meanPeakBar += last.peakCylinderPressureBar / static_cast<double>(frames - firstMeasured);
            for (std::size_t c = 0; c < last.cylinderStateCount; ++c)
                run.highestFrameSampleBar = std::max(run.highestFrameSampleBar,
                    last.cylinderStates[c].pressureEstimateBar);
        }
    }
    run.damage = last.damage;
    for (const auto& diagnostic : EngineDiagnostics {}.evaluate(config, last))
        run.peakPressureDiagnostic = run.peakPressureDiagnostic || diagnostic.code == "mechanical.peak_pressure";
    return run;
}

[[nodiscard]] EngineConfig catalogueEngine(std::string_view selector) {
    static const auto catalogue = loadEngineCatalog(ENGINELAB_CATALOG_ROOT);
    const auto selected = selectSingleEngineCatalogEntry(catalogue.entries, selector);
    require(static_cast<bool>(selected), "catalogue engine");
    auto config = selected.entry->config;
    normaliseEngineConfig(config);
    return config;
}

/** The K20 at full load fires below its default limit and takes no damage;
 *  with a limit set below its firing pressure it is damaged and told so. */
void overPressureDamages() {
    const auto config = catalogueEngine("K20");
    const auto stock = heldFullLoad(config, 10.0);
    auto weak = config;
    weak.peakFiringPressureLimitBar = std::max(20.0, 0.7 * stock.meanPeakBar);
    const auto overloaded = heldFullLoad(weak, 10.0);
    std::cout << "  K20 held at full load: peak " << stock.meanPeakBar << " bar (highest per-frame sample "
              << stock.highestFrameSampleBar << "), damage " << stock.damage << "; with a "
              << weak.peakFiringPressureLimitBar << " bar limit: damage " << overloaded.damage << '\n';
    require(stock.meanPeakBar > 30.0 && stock.meanPeakBar < peakCylinderPressureLimitBar(config),
        "a stock K20 at full load fires between 30 bar and its limit");
    require(stock.meanPeakBar >= 0.95 * stock.highestFrameSampleBar,
        "the tracked cycle peak is not below what a per-frame sample sees");
    require(stock.damage < 1.0e-6 && !stock.peakPressureDiagnostic,
        "a stock engine within its limit takes no pressure damage");
    require(overloaded.damage > 0.02, "firing 40 % over the limit for 8 s damages the engine");
    require(overloaded.peakPressureDiagnostic, "the over-pressure is diagnosed");
}

/** A diesel's mixing burn follows its rail pressure: at half the rail
 *  pressure, with the injector flow raised so its capacity is unchanged, the
 *  burn is slower and the firing peak lower. */
void railPressureDrivesTheDieselBurn() {
    const auto config = catalogueEngine("TDI");
    require(config.fuel == FuelType::diesel
            && config.injection.railPressureBar == config.injection.referencePressureBar,
        "the catalogue diesel runs at its injectors' rated pressure");
    auto low = config;
    low.injection.railPressureBar = 0.5 * config.injection.railPressureBar;
    low.injection.injectorFlowMgPerSecond = config.injection.injectorFlowMgPerSecond * std::sqrt(2.0);
    const auto rated = heldFullLoad(config, 10.0);
    const auto halved = heldFullLoad(low, 10.0);
    std::cout << "  TDI held at full load: peak " << rated.meanPeakBar << " bar at "
              << config.injection.railPressureBar << " bar rail, " << halved.meanPeakBar << " bar at "
              << low.injection.railPressureBar << '\n';
    require(halved.meanPeakBar < rated.meanPeakBar - 2.0, "a slower mixing burn fires lower");
}

} // namespace

int main() {
    limitsFollowTheEngine();
    std::cout << "default limits: ok\n";
    overPressureDamages();
    std::cout << "over-pressure damage: ok\n";
    railPressureDrivesTheDieselBurn();
    std::cout << "diesel rail pressure: ok\n";
    return 0;
}
