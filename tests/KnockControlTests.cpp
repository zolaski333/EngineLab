// Knock control is a calibration: the ECU retards the spark by a gain per unit
// of knock intensity, `knock.retard_deg_per_intensity` (12 deg by default, the
// gain it always used). Zero turns the control off and the knock runs free.

#include <enginelab/calibration/EcuCalibration.hpp>
#include <enginelab/calibration/EcuCalibrationKeys.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>
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
#include <string>
#include <string_view>
#include <utility>
#include <variant>

namespace {
using namespace enginelab;
constexpr double frameSeconds = 1.0 / 240.0;

void require(bool condition, std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

void publishKnockGain(SimpleEcuModel& ecu, double degreesPerIntensity) {
    auto draft = calibration::makeDraft(*ecu.calibrationStore()->snapshot());
    const auto* found = draft.find(calibration::keys::knockRetardGain);
    require(found != nullptr, "a spark-ignition baseline carries the knock retard gain");
    auto gain = *found;
    std::get<calibration::ScalarCalibration>(gain).value = degreesPerIntensity;
    draft.set(std::move(gain));
    require(ecu.calibrationStore()->publish(draft).published, "a knock retard gain must publish");
    ecu.beginFrame();
}

/** The ECU alone: the retard is the gain times the knock, and the knock-retard
 *  limit is reported only when the control actually retards. */
void ecuAppliesTheGain() {
    const auto config = makeDefaultInlineFour();
    SimpleEcuModel ecu;
    ecu.initialiseCalibration(config);
    ecu.beginFrame();
    require(knockRetardGainDegrees(*ecu.calibrationStore()->snapshot())
                == calibration::ecuLimits::defaultKnockRetardGainDegrees,
        "the baseline gain is the ECU's former 12 deg");

    EngineState state;
    state.rpm = 3'000.0;
    state.manifoldPressureKpa = config.ambientPressureKpa * 0.95;
    state.coolantTemperatureC = 90.0;
    state.simulationTimeSeconds = 1.0;
    EngineControls controls;
    controls.ignitionEnabled = true;
    controls.throttle = 1.0;
    const auto advanceAt = [&](double knock) {
        state.knockLevel = knock;
        return ecu.evaluate(config, state, controls);
    };
    const auto clean = advanceAt(0.0).ignitionAdvanceDegrees;
    const auto knocking = advanceAt(0.4);
    require(std::abs(clean - knocking.ignitionAdvanceDegrees - 4.8) < 1.0e-9,
        "at the default gain, knock 0.4 retards 4.8 deg");
    require(hasActiveLimit(knocking.activeLimits, ActiveLimit::knockRetard),
        "a retarding knock control is reported");

    publishKnockGain(ecu, 0.0);
    const auto off = advanceAt(0.4);
    require(std::abs(off.ignitionAdvanceDegrees - clean) < 1.0e-9,
        "with the gain at zero the knock does not move the spark");
    require(!hasActiveLimit(off.activeLimits, ActiveLimit::knockRetard),
        "a control that is off does not report a knock retard");

    publishKnockGain(ecu, 25.0);
    require(std::abs(clean - advanceAt(0.4).ignitionAdvanceDegrees - 10.0) < 1.0e-9,
        "a gain of 25 deg retards 10 deg at knock 0.4");

    auto diesel = config;
    diesel.fuel = FuelType::diesel;
    SimpleEcuModel dieselEcu;
    dieselEcu.initialiseCalibration(diesel);
    require(dieselEcu.calibrationStore()->snapshot()->find(calibration::keys::knockRetardGain) == nullptr,
        "a diesel has no spark to retard and no knock control entry");
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
    double meanKnock { 0.0 };
    double peakKnock { 0.0 };
    double meanAdvance { 0.0 };
    double meanTorque { 0.0 };
    double damage { 0.0 };
};

/** Cranked, then full throttle against a dyno that holds 0.6 x redline; means
 *  over the last `measuredSeconds`. The gain is published at 1 s, before the
 *  throttle opens. */
[[nodiscard]] HeldRun heldFullLoad(const EngineConfig& config, double knockGain, double seconds,
                                   double measuredSeconds) {
    auto rig = std::make_unique<Rig>(config);
    const auto frames = static_cast<int>(seconds * 240.0);
    const auto firstMeasured = frames - static_cast<int>(measuredSeconds * 240.0);
    HeldRun run;
    auto rpm = 0.0;
    for (auto frame = 0; frame < frames; ++frame) {
        const auto t = frame * frameSeconds;
        if (frame == 240) publishKnockGain(rig->ecu, knockGain);
        EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = t < 1.5;
        controls.throttle = t < 2.0 ? 0.0 : 1.0;
        if (t >= 2.0)
            controls.dynamometerTorqueNm = std::max(0.0, 2.0 * (rpm - 0.6 * config.redlineRpm));
        const auto state = rig->simulator->step(frameSeconds, controls).state;
        rpm = state.rpm;
        if (frame >= firstMeasured) {
            const auto weight = 1.0 / static_cast<double>(frames - firstMeasured);
            run.meanKnock += state.knockLevel * weight;
            run.meanAdvance += state.ignitionAdvanceDegrees * weight;
            run.meanTorque += state.torqueNm * weight;
            run.peakKnock = std::max(run.peakKnock, state.knockLevel);
        }
    }
    run.damage = rig->simulator->state().damage;
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

/** The 2JZ at full load, knock control on and off. */
void knockRunsFreeWithoutControl() {
    const auto config = catalogueEngine("2JZ");
    const auto on = heldFullLoad(config, calibration::ecuLimits::defaultKnockRetardGainDegrees, 12.0, 6.0);
    const auto off = heldFullLoad(config, 0.0, 12.0, 6.0);
    for (const auto& [name, run] : { std::pair { "on ", on }, std::pair { "off", off } })
        std::cout << "  2JZ held at full load, knock control " << name << ": knock mean " << run.meanKnock
                  << ", peak " << run.peakKnock << ", advance " << run.meanAdvance << " deg, torque "
                  << run.meanTorque << " N.m, damage " << run.damage << '\n';
    require(on.meanKnock > 0.1, "the 2JZ knocks lightly at full load (else this proves nothing)");
    require(off.meanAdvance > on.meanAdvance + 2.0, "without knock control the spark stays where the map puts it");
    require(off.meanKnock > on.meanKnock + 0.05, "without knock control the knock grows");
}

} // namespace

int main() {
    ecuAppliesTheGain();
    std::cout << "ECU knock retard gain: ok\n";
    knockRunsFreeWithoutControl();
    std::cout << "knock control off, engine: ok\n";
    return 0;
}
