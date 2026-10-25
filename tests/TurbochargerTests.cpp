// Turbochargers: the compressor map, parallel compressors feeding one charge,
// and several turbochargers sharing an engine's exhaust paths.

#include <enginelab/calibration/EcuCalibration.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/calibration/EcuCalibrationKeys.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/foundation/ForcedInductionFlow.hpp>
#include <enginelab/foundation/TurboSizing.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <numbers>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
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

[[nodiscard]] double radiansPerSecond(double rpm) { return rpm * 2.0 * std::numbers::pi / 60.0; }

/** The map holds its design point and falls along a speed line. */
void mapFollowsItsDesignPoint() {
    ForcedInductionConfig turbo;
    turbo.pressureRatio = 2.0;
    turbo.designShaftSpeedRpm = 120'000.0;
    constexpr double designFlow = 0.2;
    const auto omega = radiansPerSecond(turbo.designShaftSpeedRpm);
    const auto at = [&](double speed, double flow) {
        return compressorOperatingPoint(turbo, designFlow, speed, flow,
            compressorReferenceTemperatureK, compressorReferencePressureKpa);
    };
    const auto design = at(omega, designFlow);
    require(std::abs(design.pressureRatio - 2.0) < 1.0e-9 && std::abs(design.flowCoefficientRatio - 1.0) < 1.0e-12,
        "at its design speed and flow the map gives the design pressure ratio");
    const auto low = at(omega, 0.1);
    const auto high = at(omega, 0.27);
    require(low.pressureRatio > design.pressureRatio && design.pressureRatio > high.pressureRatio,
        "along a speed line the head falls as the flow rises");
    const auto stalled = at(omega, 0.0);
    require(stalled.surge && stalled.pressureRatio > 2.0 && stalled.pressureRatio < 2.4,
        "at no flow the wheel surges, its head a little above the design one");
    const auto choked = at(omega, 0.31);
    require(choked.choke && choked.pressureRatio == 1.0, "past choke the wheel adds no head");
    const auto half = at(0.5 * omega, 0.5 * designFlow);
    const auto quarterHead = 0.25 * (std::pow(2.0, turboIsentropicExponent) - 1.0);
    require(std::abs(std::pow(half.pressureRatio, turboIsentropicExponent) - 1.0 - quarterHead) < 1.0e-9,
        "at half speed and half flow (the same flow coefficient) the head is a quarter");
}

/** Two compressors on one charge: identical ones split the flow and hold the
 *  ratio of one twice the size; a slower one passes less. */
void parallelCompressorsShareTheCharge() {
    ForcedInductionConfig turbo;
    turbo.pressureRatio = 2.0;
    turbo.designShaftSpeedRpm = 120'000.0;
    const auto omega = radiansPerSecond(110'000.0);
    std::array<double, maximumTurbochargers> speeds { omega, omega, 0.0, 0.0 };
    std::array<double, maximumTurbochargers> flows {};
    const auto ratio = parallelCompressorPressureRatio(turbo, 0.2, speeds, 2, 0.35,
        compressorReferenceTemperatureK, compressorReferencePressureKpa, flows);
    const auto single = compressorOperatingPoint(turbo, 0.4, omega, 0.35,
        compressorReferenceTemperatureK, compressorReferencePressureKpa).pressureRatio;
    require(std::abs(ratio - single) < 1.0e-6 && std::abs(flows[0] - 0.175) < 1.0e-9
            && std::abs(flows[1] - 0.175) < 1.0e-9,
        "two identical compressors hold the ratio of one of twice the flow, half the flow each");
    speeds[1] = 0.7 * omega;
    const auto uneven = parallelCompressorPressureRatio(turbo, 0.2, speeds, 2, 0.35,
        compressorReferenceTemperatureK, compressorReferencePressureKpa, flows);
    require(flows[1] < flows[0] && std::abs(flows[0] + flows[1] - 0.35) < 1.0e-9 && uneven < ratio,
        "a slower compressor passes less of the flow, and the charge holds less");
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
    double boostRatio { 0.0 };
    double boostStdDev { 0.0 };
    double torqueNm { 0.0 };
    double lambdaMin { 1.0e9 };
    double lambdaMax { 0.0 };
    std::array<double, maximumTurbochargers> shaftRpm {};
    std::array<double, maximumTurbochargers> flowShare {};
};

/** The ECU's boost table set flat at `gaugeKpa`. */
void publishBoostTable(SimpleEcuModel& ecu, double gaugeKpa) {
    auto draft = calibration::makeDraft(*ecu.calibrationStore()->snapshot());
    const auto* found = draft.find(calibration::keys::boostTargetGauge);
    require(found != nullptr, "a turbocharged engine's tables carry a boost target");
    auto entry = *found;
    for (auto& value : std::get<calibration::CalibrationCurve1D>(entry).values) value = gaugeKpa;
    draft.set(std::move(entry));
    require(ecu.calibrationStore()->publish(draft).published, "a boost table must publish");
}

/** Cranked, then full throttle against a dyno holding `heldRpm` (0.7 x
 *  redline by default); means over the last 3 s. A non-negative
 *  `boostTableGaugeKpa` replaces the ECU's boost table. */
[[nodiscard]] HeldRun heldFullLoad(const EngineConfig& config, double seconds, double boostTableGaugeKpa = -1.0,
                                   double heldRpm = 0.0) {
    if (heldRpm <= 0.0) heldRpm = 0.7 * config.redlineRpm;
    auto rig = std::make_unique<Rig>(config);
    if (boostTableGaugeKpa >= 0.0) publishBoostTable(rig->ecu, boostTableGaugeKpa);
    const auto frames = static_cast<int>(seconds * 240.0);
    const auto firstMeasured = frames - 3 * 240;
    const auto weight = 1.0 / static_cast<double>(frames - firstMeasured);
    HeldRun run;
    auto rpm = 0.0;
    for (auto frame = 0; frame < frames; ++frame) {
        const auto t = frame * frameSeconds;
        EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = t < 1.5;
        controls.throttle = t < 2.0 ? 0.0 : 1.0;
        if (t >= 2.0) controls.dynamometerTorqueNm = std::max(0.0, 2.0 * (rpm - heldRpm));
        const auto state = rig->simulator->step(frameSeconds, controls).state;
        rpm = state.rpm;
        if (frame < firstMeasured) continue;
        run.boostRatio += state.boostPressureRatio * weight;
        run.boostStdDev += state.boostPressureRatio * state.boostPressureRatio * weight;
        if (state.airFuelRatioValid) {
            run.lambdaMin = std::min(run.lambdaMin, state.lambda);
            run.lambdaMax = std::max(run.lambdaMax, state.lambda);
        }
        run.torqueNm += state.torqueNm * weight;
        auto totalFlow = 0.0;
        for (std::size_t index = 0; index < state.turbochargerCount; ++index)
            totalFlow += state.turbochargers[index].compressorMassFlowKgPerSecond;
        for (std::size_t index = 0; index < state.turbochargerCount; ++index) {
            run.shaftRpm[index] += state.turbochargers[index].shaftSpeedRpm * weight;
            run.flowShare[index] += (totalFlow > 0.0
                ? state.turbochargers[index].compressorMassFlowKgPerSecond / totalFlow : 0.0) * weight;
        }
    }
    run.boostStdDev = std::sqrt(std::max(0.0, run.boostStdDev - run.boostRatio * run.boostRatio));
    return run;
}

/** The engine with its cylinders split over scalar exhaust paths. */
[[nodiscard]] EngineConfig withPaths(EngineConfig config, const std::vector<std::vector<std::uint32_t>>& groups) {
    config.exhaustPaths.clear();
    std::uint32_t id = 1;
    for (const auto& group : groups) {
        ExhaustPathConfig path;
        path.id = id++;
        path.cylinderIds = group;
        path.geometry = config.exhaust;
        config.exhaustPaths.push_back(std::move(path));
    }
    normaliseEngineConfig(config);
    return config;
}

/** A turbocharger split in two equal halves, one per unit. */
[[nodiscard]] EngineConfig asTwinUnits(EngineConfig config, std::uint32_t count) {
    auto& turbo = config.forcedInduction;
    const auto share = 1.0 / static_cast<double>(count);
    turbo.compressorDesignMassFlowKgPerSecond = compressorDesignMassFlowKgPerSecond(config) * share;
    turbo.turbineFlowAreaMm2 *= share;
    turbo.wastegateFlowAreaMm2 *= share;
    turbo.shaftInertiaKgM2 *= share;
    turbo.bearingFrictionPowerWatts *= share;
    turbo.turbochargerCount = count;
    return config;
}

/** Changing the count keeps the set's total size: the same units as a
 *  hand split, and back again. */
void countKeepsTheSize() {
    auto single = withPaths(withTurbocharger(makeDefaultInlineFour()), { { 1, 4 }, { 2, 3 } });
    const auto designFlow = compressorDesignMassFlowKgPerSecond(single);
    const auto split = withTurbochargerCount(single, 2);
    const auto byHand = asTwinUnits(single, 2);
    const auto& a = split.forcedInduction;
    const auto& b = byHand.forcedInduction;
    require(a.turbochargerCount == 2U && !validateEngineConfig(split), "two valid units");
    require(std::abs(a.compressorDesignMassFlowKgPerSecond - b.compressorDesignMassFlowKgPerSecond) < 1.0e-12
            && a.turbineFlowAreaMm2 == b.turbineFlowAreaMm2 && a.wastegateFlowAreaMm2 == b.wastegateFlowAreaMm2
            && a.shaftInertiaKgM2 == b.shaftInertiaKgM2 && a.bearingFrictionPowerWatts == b.bearingFrictionPowerWatts,
        "each unit is half the one");
    require(a.blowOffValveFlowAreaMm2 == single.forcedInduction.blowOffValveFlowAreaMm2,
        "the blow-off valve is on the common charge");
    auto named = split;
    named.exhaustPaths[1].turbocharger = 1;
    const auto back = withTurbochargerCount(named, 1);
    require(back.forcedInduction.turbochargerCount == 1U && back.exhaustPaths[1].turbocharger == -1
            && !validateEngineConfig(back), "a path naming a unit that went goes back to the default");
    require(std::abs(back.forcedInduction.compressorDesignMassFlowKgPerSecond - designFlow) < 1.0e-12 * designFlow
            && std::abs(back.forcedInduction.turbineFlowAreaMm2 - single.forcedInduction.turbineFlowAreaMm2) < 1.0e-9,
        "and one unit is the original again");
    const auto natural = makeDefaultInlineFour();
    require(withTurbochargerCount(natural, 3).forcedInduction.turbochargerCount
            == natural.forcedInduction.turbochargerCount, "an engine without a turbo is left alone");
}

/** Two identical turbochargers, one per pair of cylinders, run as one of
 *  twice the size on both pairs. */
void twoIdenticalAreOneDouble() {
    auto single = withPaths(withTurbocharger(makeDefaultInlineFour()), { { 1, 4 }, { 2, 3 } });
    single.forcedInduction.compressorDesignMassFlowKgPerSecond = compressorDesignMassFlowKgPerSecond(single);
    const auto twin = asTwinUnits(single, 2);
    require(!validateEngineConfig(single) && !validateEngineConfig(twin), "valid fixtures");
    const auto one = heldFullLoad(single, 9.0);
    const auto two = heldFullLoad(twin, 9.0);
    std::cout << "  I4, one turbocharger: boost " << one.boostRatio << ", " << one.torqueNm << " N.m, shaft "
              << one.shaftRpm[0] << " rpm; two halves: boost " << two.boostRatio << ", " << two.torqueNm
              << " N.m, shafts " << two.shaftRpm[0] << " / " << two.shaftRpm[1] << " rpm\n";
    require(one.boostRatio > 1.2, "the fixture makes boost (else this proves nothing)");
    require(std::abs(two.boostRatio - one.boostRatio) < 0.01 * one.boostRatio
            && std::abs(two.torqueNm - one.torqueNm) < 0.01 * one.torqueNm,
        "two identical turbochargers make the boost and torque of one twice the size");
    require(std::abs(two.shaftRpm[0] - one.shaftRpm[0]) < 0.01 * one.shaftRpm[0]
            && std::abs(two.shaftRpm[1] - one.shaftRpm[0]) < 0.01 * one.shaftRpm[0],
        "each half spins as the double one does");

    // A charge-air cooler: the same boost, a denser charge.
    auto cooled = single;
    cooled.forcedInduction.intercoolerEffectiveness = 0.7;
    require(!validateEngineConfig(cooled), "valid intercooled fixture");
    const auto intercooled = heldFullLoad(cooled, 9.0);
    std::cout << "  with a 0.7 intercooler: boost " << intercooled.boostRatio << ", " << intercooled.torqueNm
              << " N.m\n";
    require(intercooled.torqueNm > 1.08 * one.torqueNm,
        "an intercooler makes more torque from the same boost");
    require(std::abs(intercooled.boostRatio - one.boostRatio) < 0.05 * one.boostRatio,
        "the wastegate still holds the boost");
}

/** A V8 with two cylinders on one turbocharger and six on the other: the
 *  small one gets a third of the gas, spins slower and passes less air. */
void aSmallBankSpinsSlower() {
    const auto engine = asTwinUnits(withPaths(withTurbocharger(makeDefaultV8()),
        { { 1, 2 }, { 3, 4, 5, 6, 7, 8 } }), 2);
    require(!validateEngineConfig(engine), "valid fixture");
    const auto run = heldFullLoad(engine, 9.0);
    std::cout << "  V8, 2 + 6 cylinders: shafts " << run.shaftRpm[0] << " / " << run.shaftRpm[1]
              << " rpm, flow shares " << run.flowShare[0] << " / " << run.flowShare[1] << ", boost "
              << run.boostRatio << '\n';
    require(run.shaftRpm[0] < 0.8 * run.shaftRpm[1], "the turbocharger on two cylinders spins slower");
    require(run.flowShare[0] < 0.4, "and its compressor passes the smaller share of the air");
}

/** The ECU's boost table is the boost: by default the configuration's
 *  wastegate ratio, and a lower table holds a lower boost. */
void boostTableIsTheTarget() {
    const auto config = withTurbocharger(makeDefaultInlineFour());
    const auto defaults = calibration::makeDefaultEcuCalibration(config);
    const auto* table = defaults.find(calibration::keys::boostTargetGauge);
    require(table != nullptr, "a turbocharged engine's default tables carry a boost target");
    const auto expectedGaugeKpa = (config.forcedInduction.wastegatePressureRatio - 1.0) * 101.325;
    for (const auto value : std::get<calibration::CalibrationCurve1D>(*table).values)
        require(std::abs(value - expectedGaugeKpa) < 1.0e-9, "the default table is the wastegate ratio, flat");
    auto aspirated = makeDefaultInlineFour();
    aspirated.forcedInduction.enabled = false;
    require(calibration::makeDefaultEcuCalibration(aspirated).find(calibration::keys::boostTargetGauge) == nullptr,
        "a naturally aspirated engine has no boost table");

    const auto stock = heldFullLoad(config, 9.0);
    const auto lower = heldFullLoad(config, 9.0, expectedGaugeKpa - 20.0);
    std::cout << "  boost table " << expectedGaugeKpa << " kPa: ratio " << stock.boostRatio << ", "
              << stock.torqueNm << " N.m; " << expectedGaugeKpa - 20.0 << " kPa: ratio " << lower.boostRatio
              << ", " << lower.torqueNm << " N.m\n";
    require(std::abs(stock.boostRatio - (1.0 + expectedGaugeKpa / 101.325)) < 0.02,
        "the default table holds the configured boost");
    require(std::abs(lower.boostRatio - (1.0 + (expectedGaugeKpa - 20.0) / 101.325)) < 0.02,
        "a table 20 kPa lower holds a boost 20 kPa lower");
    require(lower.torqueNm < stock.torqueNm, "and the engine makes less torque");
}

[[nodiscard]] EngineConfig catalogueEngine(std::string_view selector) {
    static const auto catalogue = loadEngineCatalog(ENGINELAB_CATALOG_ROOT);
    const auto selected = selectSingleEngineCatalogEntry(catalogue.entries, selector);
    require(static_cast<bool>(selected), "catalogue engine " + std::string(selector));
    auto config = selected.entry->config;
    normaliseEngineConfig(config);
    return config;
}

/** A wastegate set far past the catalogue's holds its boost: the gate's
 *  authority grows with the boost, and a fixed controller band ran a 13 Hz
 *  limit cycle from ~2.7 (2JZ) or ~3.0 (EJ25), the gate slamming shut and open
 *  and lambda swinging 0.58 <-> 0.96 (EJ25 at 3.2, journal 2026-10-25). The
 *  wastegate is moved as the engine editor moves it, the rated ratio keeping
 *  its distance. */
void highBoostHoldsSteady() {
    for (const auto* name : { "EJ25", "2JZ" }) {
        auto config = catalogueEngine(name);
        auto& turbo = config.forcedInduction;
        const auto margin = turbo.pressureRatio - turbo.wastegatePressureRatio;
        turbo.wastegatePressureRatio = 3.2;
        turbo.pressureRatio = turbo.wastegatePressureRatio + margin;
        const auto run = heldFullLoad(config, 9.0, -1.0, 5'400.0);
        std::cout << "  " << name << " wastegate 3.2 at 5,400 rpm: boost " << run.boostRatio << " +- "
                  << run.boostStdDev << ", lambda " << run.lambdaMin << " .. " << run.lambdaMax << '\n';
        require(std::abs(run.boostRatio - 3.2) < 0.05, std::string(name) + ": the boost reaches its wastegate");
        require(run.boostStdDev < 0.03, std::string(name) + ": and holds it without hunting");
        require(run.lambdaMax - run.lambdaMin < 0.08, std::string(name) + ": so the mixture holds too");
    }
}

} // namespace

int main() {
    mapFollowsItsDesignPoint();
    std::cout << "compressor map: ok\n";
    parallelCompressorsShareTheCharge();
    std::cout << "parallel compressors: ok\n";
    countKeepsTheSize();
    std::cout << "turbocharger count: ok\n";
    twoIdenticalAreOneDouble();
    std::cout << "two identical = one double: ok\n";
    aSmallBankSpinsSlower();
    std::cout << "two plus six cylinders: ok\n";
    boostTableIsTheTarget();
    std::cout << "ECU boost table: ok\n";
    highBoostHoldsSteady();
    std::cout << "high boost holds steady: ok\n";
    return 0;
}
