/**
 * Compression-ignition ECU authority regression.
 *
 * Runs the catalogue diesel at one held full-load point with four hot-map
 * quantities. A pass proves the user-editable injected-quantity curve reaches
 * the physical injector/combustion path, changes delivered fuel and torque in
 * the correct direction, and does not revive the gasoline-only lean warning.
 */

#include <enginelab/calibration/EcuCalibration.hpp>
#include <enginelab/calibration/EcuCalibrationKeys.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/diagnostics/EngineDiagnostics.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/simulation/DynoAbsorberController.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>
#include <variant>

namespace {
constexpr double targetRpm = 2'000.0;
constexpr double dt = 1.0 / 240.0;

struct Sample final {
    std::string label;
    double mapScale {};
    double commandMg {};
    double rpm {};
    double fuelMgPerCycle {};
    double afr {};
    double smokeFloorAfr {};
    double torqueNm {};
    double oxygenCeilingMg {};
    bool falseLeanDiagnostic {};
};

// Observe the actual command without evaluating the stateful ECU a second time.
class CapturingEcu final : public enginelab::IEcuModel {
public:
    explicit CapturingEcu(enginelab::SimpleEcuModel& delegate) : delegate_(delegate) {}
    void initialise(const enginelab::EngineConfig& config) override { delegate_.initialise(config); }
    void beginFrame() noexcept override { delegate_.beginFrame(); }
    void reset() noexcept override { delegate_.reset(); }
    enginelab::EcuCommand evaluate(const enginelab::EngineConfig& config,
        const enginelab::EngineState& state, const enginelab::EngineControls& controls) const noexcept override {
        last = delegate_.evaluate(config, state, controls);
        return last;
    }
    mutable enginelab::EcuCommand last {};
private:
    enginelab::SimpleEcuModel& delegate_;
};

[[nodiscard]] Sample measure(const enginelab::EngineConfig& config,
                             double mapScale, std::string label) {
    enginelab::SimpleEcuModel ecu;
    ecu.initialiseCalibration(config);
    auto draft = enginelab::calibration::makeDraft(
        *ecu.calibrationStore()->snapshot());
    auto quantity = *draft.find(
        enginelab::calibration::keys::dieselFuelQuantityMgPerCycle);
    auto& curve = std::get<enginelab::calibration::CalibrationCurve1D>(quantity);
    for (auto& value : curve.values)
        value *= mapScale;
    draft.set(std::move(quantity));
    if (!ecu.calibrationStore()->publish(draft).published)
        return { std::move(label), mapScale };
    ecu.beginFrame();

    enginelab::SimplifiedGasolinePhysics physics;
    enginelab::FourStrokeEventGenerator events;
    auto exhaust = enginelab::ExhaustGraph::makeForEngine(config);
    CapturingEcu captured(ecu);
    enginelab::EngineSimulator simulator(
        config, captured, physics, events, exhaust);
    for (int step = 0; step < static_cast<int>(2.0 / dt); ++step) {
        const auto time = static_cast<double>(step) * dt;
        (void)simulator.step(dt, { true, time < 1.5, 0.55, 0.0 });
    }

    enginelab::DynoAbsorberController absorber(config);
    absorber.reset(simulator.state().rpm, simulator.state().torqueNm);
    constexpr auto settleSteps = static_cast<int>(6.0 / dt);
    constexpr auto sampleSteps = static_cast<int>(1.5 / dt);
    Sample result { std::move(label), mapScale };
    enginelab::EngineState lastState;
    for (int step = 0; step < settleSteps + sampleSteps; ++step) {
        const auto output = absorber.advance(dt, targetRpm, simulator.state());
        enginelab::EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = simulator.state().rpm < 550.0;
        controls.throttle = 1.0;
        controls.dynamometerTorqueNm = output.brakeTorqueNm;
        const auto frame = simulator.step(dt, controls);
        lastState = frame.state;
        if (step < settleSteps)
            continue;
        result.commandMg +=
            frame.state.ecuDieselFuelQuantityLimitMgPerCycle;
        result.rpm += frame.state.rpm;
        result.fuelMgPerCycle += frame.state.injectedFuelMgPerCycle;
        result.afr += frame.state.airFuelRatio;
        const auto smokeFloor = std::max(config.fuelProperties.stoichiometricAirFuelRatio * 1.16,
            captured.last.targetAirFuelRatio);
        result.smokeFloorAfr += smokeFloor;
        result.oxygenCeilingMg += frame.state.airMassMgPerCycle / smokeFloor;
        result.torqueNm += frame.state.cycleAveragedTorqueNm;
    }
    const auto divisor = static_cast<double>(sampleSteps);
    result.commandMg /= divisor;
    result.rpm /= divisor;
    result.fuelMgPerCycle /= divisor;
    result.afr /= divisor;
    result.smokeFloorAfr /= divisor;
    result.torqueNm /= divisor;
    result.oxygenCeilingMg /= divisor;
    const auto diagnostics = enginelab::EngineDiagnostics {}.evaluate(
        config, lastState);
    result.falseLeanDiagnostic = std::any_of(
        diagnostics.begin(), diagnostics.end(), [](const auto& item) {
            return item.code == "combustion.lean";
        });
    return result;
}
}

int main() {
    const auto catalog = enginelab::loadEngineCatalog(ENGINELAB_CATALOG_ROOT);
    if (!catalog.errors.empty()) {
        for (const auto& error : catalog.errors)
            std::cerr << "catalog: " << error << '\n';
        return EXIT_FAILURE;
    }
    const auto diesel = std::find_if(
        catalog.entries.begin(), catalog.entries.end(), [](const auto& entry) {
            return entry.config.fuel == enginelab::FuelType::diesel;
        });
    if (diesel == catalog.entries.end()) {
        std::cerr << "FAIL: no diesel engine in catalogue\n";
        return EXIT_FAILURE;
    }

    const std::array<Sample, 4> samples {
        measure(diesel->config, 0.50, "lower"),
        measure(diesel->config, 0.65, "headroom"),
        measure(diesel->config, 1.00, "authored"),
        measure(diesel->config, 1.20, "smoke cap")
    };
    std::cout << "engine: " << diesel->config.name << " @ "
              << targetRpm << " rpm WOT\n";
    std::cout << std::left << std::setw(11) << "map"
              << std::right << std::setw(10) << "cmd_mg"
              << std::setw(10) << "rpm"
              << std::setw(13) << "fuel_mg/cyc"
              << std::setw(9) << "AFR"
              << std::setw(12) << "smoke_min"
              << std::setw(13) << "air_cap_mg"
              << std::setw(12) << "torque_Nm"
              << std::setw(11) << "lean_diag" << '\n';
    for (const auto& sample : samples) {
        std::cout << std::left << std::setw(11) << sample.label
                  << std::right << std::fixed << std::setprecision(2)
                  << std::setw(10) << sample.commandMg
                  << std::setw(10) << sample.rpm
                  << std::setw(13) << sample.fuelMgPerCycle
                  << std::setw(9) << sample.afr
                  << std::setw(12) << sample.smokeFloorAfr
                  << std::setw(13) << sample.oxygenCeilingMg
                  << std::setw(12) << sample.torqueNm
                  << std::setw(11)
                  << (sample.falseLeanDiagnostic ? "YES" : "NO") << '\n';
    }

    const auto& low = samples[0];
    const auto& base = samples[1];
    const auto& authored = samples[2];
    const auto& high = samples[3];
    const auto held = std::all_of(samples.begin(), samples.end(), [](const auto& sample) {
        return std::abs(sample.rpm - targetRpm) <= targetRpm * 0.03;
    });
    const auto orderedCommands = low.commandMg < base.commandMg
        && base.commandMg < authored.commandMg && authored.commandMg < high.commandMg;
    // Test map authority below the independent oxygen ceiling. The authored
    // quantity may already meet that ceiling; an upward edit must then plateau.
    const auto fuelAuthority = low.fuelMgPerCycle < base.fuelMgPerCycle * 0.90
        && authored.fuelMgPerCycle > base.fuelMgPerCycle * 1.005;
    const auto torqueAuthority = low.torqueNm < base.torqueNm * 0.90
        && authored.torqueNm > base.torqueNm * 1.005;
    const auto count = static_cast<double>(diesel->config.cylinders.size());
    const auto quantityTracking = std::all_of(samples.begin(), samples.begin() + 2,
        [count](const auto& sample) {
            const auto requested = sample.commandMg * count;
            return requested < sample.oxygenCeilingMg * 0.90
                && std::abs(sample.fuelMgPerCycle / requested - 1.0) < 0.03;
        });
    const auto oxygenTracking = std::abs(high.fuelMgPerCycle / high.oxygenCeilingMg - 1.0) < 0.03
        && high.commandMg * count > high.oxygenCeilingMg * 1.10;
    const auto upwardEditReachedSmokeCap =
        high.afr <= high.smokeFloorAfr * 1.03;
    const auto smokeProtected = std::all_of(
        samples.begin(), samples.end(), [](const auto& sample) {
            return sample.afr >= sample.smokeFloorAfr * 0.94;
        });
    const auto diagnosticCorrect = std::none_of(
        samples.begin(), samples.end(), [](const auto& sample) {
            return sample.falseLeanDiagnostic;
        });
    const auto passed = held && orderedCommands && fuelAuthority
        && torqueAuthority && upwardEditReachedSmokeCap
        && quantityTracking && oxygenTracking && smokeProtected && diagnosticCorrect;
    std::cout << "checks: hold=" << (held ? "PASS" : "FAIL")
              << " command=" << (orderedCommands ? "PASS" : "FAIL")
              << " fuel=" << (fuelAuthority ? "PASS" : "FAIL")
              << " torque=" << (torqueAuthority ? "PASS" : "FAIL")
              << " quantity_tracking=" << (quantityTracking ? "PASS" : "FAIL")
              << " oxygen_tracking=" << (oxygenTracking ? "PASS" : "FAIL")
              << " upward_smoke_cap="
              << (upwardEditReachedSmokeCap ? "PASS" : "FAIL")
              << " smoke=" << (smokeProtected ? "PASS" : "FAIL")
              << " diagnostic=" << (diagnosticCorrect ? "PASS" : "FAIL")
              << '\n';
    return passed ? EXIT_SUCCESS : EXIT_FAILURE;
}
