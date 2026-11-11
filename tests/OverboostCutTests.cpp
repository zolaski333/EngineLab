// The overboost cut: a turbocharged engine's ECU cuts the fuel while the
// manifold's gauge pressure runs its margin past the boost target, and gives
// it back at half of it. Zero, the default, turns it off; an engine without a
// turbocharger has none.

#include <enginelab/calibration/EcuCalibration.hpp>
#include <enginelab/calibration/EcuCalibrationKeys.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/foundation/TurboSizing.hpp>

#include <cstdlib>
#include <iostream>
#include <string_view>
#include <utility>
#include <variant>

namespace {
using namespace enginelab;

void require(bool condition, std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

void publishMargin(SimpleEcuModel& ecu, double kpa) {
    auto draft = calibration::makeDraft(*ecu.calibrationStore()->snapshot());
    const auto* found = draft.find(calibration::keys::overboostCut);
    require(found != nullptr, "a turbocharged engine's tables carry the overboost cut");
    auto entry = *found;
    std::get<calibration::ScalarCalibration>(entry).value = kpa;
    draft.set(std::move(entry));
    require(ecu.calibrationStore()->publish(draft).published, "an overboost margin must publish");
    ecu.beginFrame();
}

} // namespace

int main() {
    const auto config = withTurbocharger(makeDefaultInlineFour());
    SimpleEcuModel ecu;
    ecu.initialiseCalibration(config);
    ecu.beginFrame();
    require(overboostCutMarginKpa(*ecu.calibrationStore()->snapshot()) == 0.0, "the cut is off by default");

    const auto targetGaugeKpa = (config.forcedInduction.wastegatePressureRatio - 1.0) * 101.325;
    EngineState state;
    state.coolantTemperatureC = 90.0;
    state.rpm = 4'000.0;
    EngineControls controls;
    controls.ignitionEnabled = true;
    controls.throttle = 1.0;
    auto time = 1.0;
    const auto at = [&](double gaugeOverTargetKpa) {
        state.manifoldPressureKpa = config.ambientPressureKpa + targetGaugeKpa + gaugeOverTargetKpa;
        time += 0.01;
        state.simulationTimeSeconds = time;
        return ecu.evaluate(config, state, controls);
    };
    const auto cut = [](const EcuCommand& command) {
        return !command.fuelEnabled && hasActiveLimit(command.activeLimits, ActiveLimit::overboostCut);
    };

    require(at(400.0).fuelEnabled, "off, the boost may run as far as it goes");
    publishMargin(ecu, 40.0);
    require(at(0.0).fuelEnabled && at(35.0).fuelEnabled, "a boost held at its target, or 35 kPa past it, is fuelled");
    require(cut(at(45.0)), "45 kPa past the target the fuel is cut, and the cut is reported");
    require(cut(at(25.0)), "the cut holds until the boost falls back to half its margin");
    const auto back = at(15.0);
    require(back.fuelEnabled && !hasActiveLimit(back.activeLimits, ActiveLimit::overboostCut),
        "below half its margin the fuel comes back");

    // The boost table is the target: a table raised to 200 kPa is not cut at
    // 220 kPa, and is at 250.
    {
        auto draft = calibration::makeDraft(*ecu.calibrationStore()->snapshot());
        auto entry = *draft.find(calibration::keys::boostTargetGauge);
        for (auto& value : std::get<calibration::CalibrationCurve1D>(entry).values) value = 200.0;
        draft.set(std::move(entry));
        require(ecu.calibrationStore()->publish(draft).published, "a boost table must publish");
        ecu.beginFrame();
        const auto over = [&](double gaugeKpa) {
            state.manifoldPressureKpa = config.ambientPressureKpa + gaugeKpa;
            time += 0.01;
            state.simulationTimeSeconds = time;
            return ecu.evaluate(config, state, controls);
        };
        require(over(220.0).fuelEnabled, "the cut follows a raised boost table");
        require(cut(over(250.0)), "and trips past it");
        require(over(100.0).fuelEnabled, "then lets go");
    }

    publishMargin(ecu, 0.0);
    require(at(400.0).fuelEnabled, "zero turns it off again");

    const auto aspirated = makeDefaultInlineFour();
    const auto tables = calibration::makeDefaultEcuCalibration(aspirated);
    require(tables.find(calibration::keys::overboostCut) == nullptr, "an engine without a turbocharger has none");

    // Tables saved before the cut existed get it, at its default (so the
    // tuner lists it); a saved setting stays.
    auto older = calibration::makeDefaultEcuCalibration(config);
    require(older.erase(calibration::keys::overboostCut), "the default tables carry the cut");
    calibration::addMissingDefaultSettings(older, config);
    const auto* added = older.find(calibration::keys::overboostCut);
    require(added != nullptr && std::get<calibration::ScalarCalibration>(*added).value == 0.0,
        "older saved tables get the cut at its default");
    auto on = calibration::makeDefaultEcuCalibration(config);
    auto entry = *on.find(calibration::keys::overboostCut);
    std::get<calibration::ScalarCalibration>(entry).value = 40.0;
    on.set(std::move(entry));
    calibration::addMissingDefaultSettings(on, config);
    require(std::get<calibration::ScalarCalibration>(*on.find(calibration::keys::overboostCut)).value == 40.0,
        "a saved setting is kept");

    std::cout << "overboost cut: ok\n";
    return 0;
}
