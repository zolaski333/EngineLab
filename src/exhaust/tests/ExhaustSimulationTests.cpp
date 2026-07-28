#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/gasdynamics/ExhaustNetworkLayout.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

using namespace enginelab;

void require(bool condition, const char* message) {
    if (condition) return;
    std::cerr << "Exhaust simulation integration failure: " << message << '\n';
    std::exit(EXIT_FAILURE);
}

ExhaustComponentConfig component(std::uint32_t id, ExhaustComponentType type,
                                 double lengthMm, double diameterMm, double restriction = 0.0) {
    ExhaustComponentConfig result;
    result.id = id;
    result.type = type;
    result.lengthMm = lengthMm;
    result.diameterMm = diameterMm;
    result.restriction = restriction;
    result.dischargeCoefficient = 0.78;
    return result;
}

EngineConfig customFixture(double mufflerRestriction) {
    auto config = makeDefaultInlineFour();
    auto& path = config.exhaustPaths.front();
    path.inheritsGlobalGeometry = false;
    ExhaustNetworkConfig network;
    for (std::size_t index = 0; index < config.cylinders.size(); ++index) {
        const auto id = static_cast<std::uint32_t>(100 + index);
        network.components.push_back(component(id, ExhaustComponentType::pipe,
                                               460.0, 42.0));
        network.cylinderConnections.push_back({ config.cylinders[index].id, id });
        network.connections.push_back({ id, 200 });
    }
    network.components.push_back(component(200, ExhaustComponentType::merge, 100.0, 58.0));
    network.components.push_back(component(300, ExhaustComponentType::muffler,
                                           480.0, 58.0, mufflerRestriction));
    network.components.push_back(component(400, ExhaustComponentType::outlet, 180.0, 65.0));
    network.connections.push_back({ 200, 300 });
    network.connections.push_back({ 300, 400 });
    path.network = std::move(network);
    normaliseEngineConfig(config);
    if (const auto error = validateEngineConfig(config)) {
        std::cerr << "Invalid custom fixture: " << *error << '\n';
        require(false, "custom fixture must validate");
    }
    return config;
}

struct AverageState final {
    double rpm {};
    double exhaustPressureKpa {};
    double exhaustFlowGramsPerSecond {};
    double airFlowGramsPerSecond {};
    double deliveredAirFlowGramsPerSecond {};
    double fuelFlowGramsPerSecond {};
    double exhaustTemperatureC {};
    double torqueNm {};
    double pumpingMepBar {};
    double exhaustStrokeMepBar {};
};

AverageState simulate(const EngineConfig& config) {
    SimpleEcuModel ecu;
    SimplifiedGasolinePhysics physics;
    FourStrokeEventGenerator events;
    auto exhaust = ExhaustGraph::makeForEngine(config);
    EngineSimulator simulator(config, ecu, physics, events, exhaust);
    AverageState average;
    constexpr int steps = 2'400;
    constexpr int averagingSteps = 480;
    constexpr double targetRpm = 3'500.0;
    double loadIntegral = 0.0;
    for (int step = 0; step < steps; ++step) {
        auto dynoLoad = 0.0;
        if (step >= 320) {
            const auto speedError = (simulator.state().rpm - targetRpm) / targetRpm;
            loadIntegral = std::clamp(loadIntegral + speedError / 240.0 * 12.0, 0.0, 1.0);
            dynoLoad = std::clamp(loadIntegral + speedError * 0.70, 0.0, 1.0);
        }
        const auto frame = simulator.step(1.0 / 240.0,
            { true, step < 320, 0.72, dynoLoad });
        if (step >= steps - averagingSteps) {
            average.rpm += frame.state.rpm;
            average.exhaustPressureKpa += frame.state.exhaustPressureKpa;
            average.exhaustFlowGramsPerSecond += frame.state.exhaustFlowGramsPerSecond;
            average.airFlowGramsPerSecond += frame.state.airFlowGramsPerSecond;
            average.deliveredAirFlowGramsPerSecond +=
                frame.state.deliveredAirMassMgPerCycle
                * (frame.state.rpm / 120.0) / 1'000.0;
            average.fuelFlowGramsPerSecond += frame.state.fuelFlowGramsPerSecond;
            average.exhaustTemperatureC += frame.state.exhaustTemperatureC;
            average.torqueNm += frame.state.cycleAveragedTorqueNm;
            average.pumpingMepBar += frame.state.pumpingMeanEffectivePressureBar;
            average.exhaustStrokeMepBar +=
                frame.state.exhaustStrokeMeanEffectivePressureBar;
        }
    }
    const auto scale = 1.0 / static_cast<double>(averagingSteps);
    average.rpm *= scale;
    average.exhaustPressureKpa *= scale;
    average.exhaustFlowGramsPerSecond *= scale;
    average.airFlowGramsPerSecond *= scale;
    average.deliveredAirFlowGramsPerSecond *= scale;
    average.fuelFlowGramsPerSecond *= scale;
    average.exhaustTemperatureC *= scale;
    average.torqueNm *= scale;
    average.pumpingMepBar *= scale;
    average.exhaustStrokeMepBar *= scale;
    return average;
}

void restrictionClosesPhysicalSolver() {
    const auto openConfig = customFixture(0.05);
    const auto restrictedConfig = customFixture(18.0);
    const auto openGraph = ExhaustGraph::makeForEngine(openConfig);
    const auto restrictedGraph = ExhaustGraph::makeForEngine(restrictedConfig);
    const auto openLayout = gasdynamics::ExhaustNetworkLayout::compile(openGraph);
    const auto restrictedLayout = gasdynamics::ExhaustNetworkLayout::compile(restrictedGraph);
    const auto mufflerLoss = [](const auto& layout) {
        const auto muffler = std::find_if(layout.ducts().begin(), layout.ducts().end(),
            [](const auto& duct) { return duct.sourceComponentId == 300; });
        return muffler != layout.ducts().end() ? muffler->lossCoefficient : -1.0;
    };
    require(openLayout.valid() && restrictedLayout.valid()
            && mufflerLoss(restrictedLayout) > mufflerLoss(openLayout) + 17.0,
        "fixture K must reach the component-resolved conservative solver");

    const auto open = simulate(openConfig);
    const auto restricted = simulate(restrictedConfig);
    std::cout << "Open/restricted exhaust: rpm=" << open.rpm << '/' << restricted.rpm
              << ", pressure=" << open.exhaustPressureKpa << '/'
              << restricted.exhaustPressureKpa << " kPa, outlet="
              << open.exhaustFlowGramsPerSecond << '/' << restricted.exhaustFlowGramsPerSecond
              << " g/s, air=" << open.airFlowGramsPerSecond << '/'
              << restricted.airFlowGramsPerSecond << " g/s, delivered_air="
              << open.deliveredAirFlowGramsPerSecond << '/'
              << restricted.deliveredAirFlowGramsPerSecond << " g/s, fuel="
              << open.fuelFlowGramsPerSecond << '/'
              << restricted.fuelFlowGramsPerSecond << " g/s, torque="
              << open.torqueNm << '/' << restricted.torqueNm << " Nm, pmep="
              << open.pumpingMepBar << '/' << restricted.pumpingMepBar
              << " bar, exhaust_mep=" << open.exhaustStrokeMepBar << '/'
              << restricted.exhaustStrokeMepBar << " bar\n";
    require(std::isfinite(open.exhaustPressureKpa) && std::isfinite(restricted.exhaustPressureKpa)
            && std::isfinite(open.torqueNm) && std::isfinite(restricted.torqueNm),
        "custom exhaust integration must remain finite");
    require(restricted.exhaustPressureKpa > open.exhaustPressureKpa + 2.0,
        "higher DAG K must raise the authoritative collector pressure");
    // At fixed speed and throttle the engine is an active gas pump: a higher K
    // need not reduce mass flow monotonically, because the dynamometer supplies
    // whatever crank work is needed to hold speed. The physically invariant
    // comparison is pressure-loss power, Delta-p times volumetric flow.
    const auto pumpingPowerW = [&openConfig](const AverageState& state) {
        constexpr double representativeExhaustGasConstantJPerKgK = 287.0;
        const auto absolutePressurePa = state.exhaustPressureKpa * 1'000.0;
        const auto densityKgPerM3 = absolutePressurePa
            / (representativeExhaustGasConstantJPerKgK
                * (state.exhaustTemperatureC + 273.15));
        const auto volumeFlowM3PerSecond = state.exhaustFlowGramsPerSecond
            * 0.001 / densityKgPerM3;
        return (absolutePressurePa - openConfig.ambientPressureKpa * 1'000.0)
            * volumeFlowM3PerSecond;
    };
    require(pumpingPowerW(restricted) > pumpingPowerW(open)
            && restricted.pumpingMepBar < open.pumpingMepBar - 0.05
            && restricted.exhaustStrokeMepBar < open.exhaustStrokeMepBar - 0.05,
        "higher DAG K must increase pressure-loss power and measured pumping loss");
    const auto massFlowIsConsistent = [](const AverageState& state) {
        const auto netInputFlowGramsPerSecond =
            state.deliveredAirFlowGramsPerSecond + state.fuelFlowGramsPerSecond;
        // The public outlet telemetry clips reverse pulses at zero and is
        // therefore gross outward flow, while delivered air + fuel is net
        // input flow. They need not be equal on an acoustically open outlet,
        // but a settled fixture must keep them in the same mass-flow band.
        return netInputFlowGramsPerSecond > 0.0
            && state.exhaustFlowGramsPerSecond > netInputFlowGramsPerSecond * 0.85
            && state.exhaustFlowGramsPerSecond < netInputFlowGramsPerSecond * 1.15;
    };
    require(massFlowIsConsistent(open) && massFlowIsConsistent(restricted),
        "established outlet flow must remain consistent with air plus fuel mass");
}

void authoredNetworkIgnoresLegacyGeometry() {
    const auto referenceConfig = customFixture(0.4);
    auto mutatedLegacy = referenceConfig;
    auto& geometry = mutatedLegacy.exhaustPaths.front().geometry;
    geometry.primaryLengthMm = 1'900.0;
    geometry.primaryDiameterMm = 25.0;
    geometry.collectorVolumeLitres = 18.0;
    geometry.collectorDiameterMm = 140.0;
    geometry.outletDiameterMm = 22.0;
    geometry.outletDischargeCoefficient = 0.10;
    geometry.mufflerRestriction = 0.98;
    normaliseEngineConfig(mutatedLegacy);
    require(!validateEngineConfig(mutatedLegacy).has_value(), "legacy mutation fixture must validate");

    const auto reference = simulate(referenceConfig);
    const auto mutated = simulate(mutatedLegacy);
    const auto same = [](double left, double right) {
        return std::abs(left - right) <= std::max(1.0, std::abs(left)) * 1.0e-10;
    };
    require(same(reference.exhaustPressureKpa, mutated.exhaustPressureKpa)
            && same(reference.exhaustFlowGramsPerSecond, mutated.exhaustFlowGramsPerSecond)
            && same(reference.airFlowGramsPerSecond, mutated.airFlowGramsPerSecond)
            && same(reference.torqueNm, mutated.torqueNm),
        "an authored DAG must be physically independent from legacy geometry fields");
}

} // namespace

int main() {
    restrictionClosesPhysicalSolver();
    authoredNetworkIgnoresLegacyGeometry();
    std::cout << "EngineLab exhaust simulation integration tests passed\n";
    return EXIT_SUCCESS;
}
