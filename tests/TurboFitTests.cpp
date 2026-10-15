// A turbocharger fitted to an engine that has none (sizedTurbocharger).
//
// - The scaling laws hold on the four catalogue turbochargers they were fitted
//   to (whose sizes were set against their makers' torque and power): turbine
//   area within 10 %, inducer within 15 % with the boost the law picks, full
//   boost between 35 and 55 % of the redline.
// - Every catalogue engine without forced induction takes it and stays valid;
//   the boost goes down as the compression goes up.
// - On the CP2 held at 7,000 rpm at wide-open throttle, the turbo holds its
//   wastegate boost and raises the torque about as much as the denser charge
//   allows (boost over the charge's temperature rise), with the mixture kept
//   rich and no heavy knock. The reference is the charge density, not a
//   number read back from this simulator.

#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/foundation/TurboSizing.hpp>
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

namespace {
using namespace enginelab;

void require(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

const EngineConfig& findEngine(const EngineCatalogLoadResult& catalog, std::string_view fragment) {
    const auto found = std::find_if(catalog.entries.begin(), catalog.entries.end(),
        [&](const auto& entry) { return entry.config.name.find(fragment) != std::string::npos; });
    require(found != catalog.entries.end(), "expected engine must exist in the catalogue");
    return found->config;
}

struct Held final {
    double torqueNm {};
    double boost {};
    double lambda {};
    double knock {};
};

/** Wide-open throttle held at `rpm` by the dyno absorber: means over 1 s after 4 s. */
Held hold(const EngineConfig& config, double rpm) {
    SimpleEcuModel ecu;
    SimplifiedGasolinePhysics physics;
    FourStrokeEventGenerator events;
    ExhaustGraph exhaust = ExhaustGraph::makeForEngine(config);
    auto simulator = std::make_unique<EngineSimulator>(config, ecu, physics, events, exhaust);
    constexpr double dt = 1.0 / 240.0;
    DynoAbsorberController absorber(config);
    absorber.reset(simulator->state().rpm, simulator->state().torqueNm);
    Held held;
    const auto settle = static_cast<int>(4.0 / dt), sample = static_cast<int>(1.0 / dt);
    for (int step = 0; step < settle + sample; ++step) {
        const auto output = absorber.advance(dt, rpm, simulator->state());
        EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = simulator->state().rpm < 550.0;
        controls.throttle = 1.0;
        controls.dynamometerTorqueNm = output.brakeTorqueNm;
        const auto frame = simulator->step(dt, controls);
        if (step < settle) continue;
        held.torqueNm += controls.dynamometerTorqueNm / sample;
        held.boost += frame.state.boostPressureRatio / sample;
        held.lambda += frame.state.lambda / sample;
        held.knock = std::max(held.knock, frame.state.knockLevel);
    }
    return held;
}
} // namespace

int main() {
    const auto catalog = loadEngineCatalog(ENGINELAB_CATALOG_ROOT);
    require(catalog.errors.empty() && !catalog.entries.empty(), "the shipped catalogue loads");

    auto anchors = 0;
    for (const auto& entry : catalog.entries) {
        const auto& config = entry.config;
        const auto& real = config.forcedInduction;
        if (!real.enabled || real.type != ForcedInductionType::turbocharger) continue;
        ++anchors;
        const auto sized = sizedTurbocharger(config);
        const auto& name = config.name;
        require(std::abs(sized.turbineFlowAreaMm2 / real.turbineFlowAreaMm2 - 1.0) < 0.10,
                name + ": the sized turbine area is within 10 % of the real one");
        require(std::abs(sized.compressorInducerDiameterMm / real.compressorInducerDiameterMm - 1.0) < 0.15,
                name + ": the sized inducer is within 15 % of the real one");
        const auto boostShare = real.fullBoostRpm / config.redlineRpm;
        require(boostShare > 0.35 && boostShare < 0.55, name + ": full boost comes at 35-55 % of the redline");
    }
    require(anchors == 4, "the four catalogue turbochargers anchor the laws");

    auto fitted = 0;
    for (const auto& entry : catalog.entries) {
        if (entry.config.forcedInduction.enabled) continue;
        if (entry.config.cycle == EngineCycle::twoStroke) {
            require(validateEngineConfig(withTurbocharger(entry.config)).has_value(),
                entry.config.name + ": a two-stroke refuses a turbo");
            continue;
        }
        const auto turbo = withTurbocharger(entry.config);
        require(!validateEngineConfig(turbo).has_value(), entry.config.name + ": a fitted turbo leaves the engine valid");
        require(turbo.forcedInduction.enabled && turbo.forcedInduction.type == ForcedInductionType::turbocharger
                    && turbo.forcedInduction.wastegatePressureRatio < turbo.forcedInduction.pressureRatio,
                entry.config.name + ": the fitted turbo is a turbocharger with its wastegate below its target");
        require(!withoutForcedInduction(turbo).forcedInduction.enabled, entry.config.name + ": it comes off again");
        ++fitted;
    }
    require(fitted >= 10, "most catalogue engines can take a turbo");
    auto low = findEngine(catalog, "Radial");
    auto high = findEngine(catalog, "MT-07-like 689 Twin");
    require(sizedTurbocharger(low).pressureRatio > sizedTurbocharger(high).pressureRatio,
            "a lower compression ratio takes more boost");

    const auto& cp2 = findEngine(catalog, "Yamaha CP2");
    const auto natural = hold(cp2, 7'000.0);
    const auto turbo = withTurbocharger(cp2);
    const auto boosted = hold(turbo, 7'000.0);
    const auto& forced = turbo.forcedInduction;
    // Charge density over the naturally aspirated one: the boost, less the
    // charge's temperature rise over the ambient air.
    const auto ambientK = cp2.ambientTemperatureC + 273.15;
    const auto density = forced.wastegatePressureRatio * ambientK / (ambientK + forced.chargeTemperatureRiseC);
    const auto gain = boosted.torqueNm / natural.torqueNm;
    std::cout << "TurboFit: CP2 at 7,000 rpm: " << natural.torqueNm << " -> " << boosted.torqueNm << " Nm (x" << gain
              << ", charge density x" << density << "), boost " << boosted.boost << ", lambda " << boosted.lambda
              << ", peak knock " << boosted.knock << '\n';
    require(std::abs(boosted.boost - forced.wastegatePressureRatio) < 0.05, "the fitted turbo holds its wastegate boost");
    require(gain > 0.9 * density && gain < 1.35 * density,
            "the torque rises about as the charge density does (x" + std::to_string(gain) + " for x"
                + std::to_string(density) + ")");
    require(boosted.lambda > 0.78 && boosted.lambda < 0.95, "the mixture stays rich at boost");
    require(boosted.knock < 0.45, "no heavy knock at the fitted boost");
    return EXIT_SUCCESS;
}
