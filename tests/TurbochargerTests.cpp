// Turbochargers: the compressor map, parallel compressors feeding one charge,
// and several turbochargers sharing an engine's exhaust paths.

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
    double torqueNm { 0.0 };
    std::array<double, maximumTurbochargers> shaftRpm {};
    std::array<double, maximumTurbochargers> flowShare {};
};

/** Cranked, then full throttle against a dyno holding 0.7 x redline; means over
 *  the last 3 s. */
[[nodiscard]] HeldRun heldFullLoad(const EngineConfig& config, double seconds) {
    auto rig = std::make_unique<Rig>(config);
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
        if (t >= 2.0) controls.dynamometerTorqueNm = std::max(0.0, 2.0 * (rpm - 0.7 * config.redlineRpm));
        const auto state = rig->simulator->step(frameSeconds, controls).state;
        rpm = state.rpm;
        if (frame < firstMeasured) continue;
        run.boostRatio += state.boostPressureRatio * weight;
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

} // namespace

int main() {
    mapFollowsItsDesignPoint();
    std::cout << "compressor map: ok\n";
    parallelCompressorsShareTheCharge();
    std::cout << "parallel compressors: ok\n";
    twoIdenticalAreOneDouble();
    std::cout << "two identical = one double: ok\n";
    aSmallBankSpinsSlower();
    std::cout << "two plus six cylinders: ok\n";
    return 0;
}
