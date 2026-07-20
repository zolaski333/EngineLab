#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/foundation/EngineTypes.hpp>
#include <enginelab/gasdynamics/ExhaustGasNetwork.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string_view>
#include <vector>

namespace {

using namespace enginelab;
using namespace enginelab::gasdynamics;

void requireNetwork(bool condition, std::string_view message) {
    if (condition) return;
    std::cerr << "FAILED: " << message << '\n';
    std::exit(EXIT_FAILURE);
}

[[nodiscard]] double relativeError(double actual, double expected) {
    return std::abs(actual - expected) / std::max(1.0e-15, std::abs(expected));
}

[[nodiscard]] double totalMass(
    const std::array<double, gasSpeciesCount>& species) noexcept {
    double result = 0.0;
    for (const auto value : species) result += value;
    return result;
}

[[nodiscard]] ExhaustGasNetwork makeNetwork(const EngineConfig& config,
                                            ExhaustGasNetworkConfig networkConfig = {}) {
    const auto graph = ExhaustGraph::makeForEngine(config);
    const auto layout = ExhaustNetworkLayout::compile(graph);
    requireNetwork(layout.valid(), "test exhaust graph must compile to a valid layout");
    ExhaustGasNetwork network;
    requireNetwork(network.configure(layout, networkConfig),
                   "test physical exhaust network must configure");
    return network;
}

[[nodiscard]] ExhaustAmbientBoundary ambientFor(const ExhaustGasNetwork& network,
                                                double pressurePa,
                                                double temperatureK,
                                                double openingScale) {
    const auto state = network.mixtureModel().conservativeFromPressureTemperature(
        pressurePa, temperatureK);
    requireNetwork(state.has_value(), "ambient fixture must be physical");
    return { *state, openingScale };
}

void requireMassEnergyBalance(const ExhaustNetworkInventory& before,
                              const ExhaustNetworkInventory& after,
                              std::span<const CylinderGasExchange> cylinders,
                              std::span<const ExhaustOutletFlowSample> outlets,
                              double tolerance) {
    for (std::size_t species = 0; species < gasSpeciesCount; ++species) {
        auto expectedChange = 0.0;
        for (const auto& cylinder : cylinders)
            expectedChange += cylinder.speciesMassKg[species];
        for (const auto& outlet : outlets)
            expectedChange -= outlet.speciesMassKg[species];
        const auto actualChange = after.speciesMassKg[species] - before.speciesMassKg[species];
        requireNetwork(std::abs(actualChange - expectedChange)
                <= std::max(1.0e-13, std::abs(before.speciesMassKg[species]) * tolerance),
            "network species inventory must close against valve and outlet transfers");
    }
    auto expectedEnergyChange = 0.0;
    for (const auto& cylinder : cylinders) expectedEnergyChange += cylinder.totalEnergyJ;
    for (const auto& outlet : outlets) expectedEnergyChange -= outlet.transferredEnergyJ;
    const auto actualEnergyChange = after.totalEnergyJ - before.totalEnergyJ;
    requireNetwork(std::abs(actualEnergyChange - expectedEnergyChange)
            <= std::max(1.0e-8, std::abs(before.totalEnergyJ) * tolerance),
        "network total energy must close against valve and outlet transfers");
}

void testUniformClosedNetworkIsInvariant() {
    auto network = makeNetwork(makeDefaultInlineFour());
    const auto ambient = ambientFor(network, 101'325.0, 300.0, 0.0);
    const auto before = network.inventory();
    const auto result = network.advance(0.0015, {}, ambient);
    requireNetwork(result.completed && result.rejectedSubsteps == 0,
        "closed uniform network must advance without a positivity retry");
    const auto after = network.inventory();
    for (std::size_t species = 0; species < gasSpeciesCount; ++species) {
        requireNetwork(relativeError(after.speciesMassKg[species],
                                     before.speciesMassKg[species]) < 2.0e-12,
            "closed uniform network must conserve every species");
    }
    requireNetwork(relativeError(after.totalEnergyJ, before.totalEnergyJ) < 2.0e-12,
        "closed uniform network must conserve energy");
    requireNetwork(std::all_of(network.outletSamples().begin(), network.outletSamples().end(),
        [](const ExhaustOutletFlowSample& sample) {
            return sample.massFlowKgPerS == 0.0 && sample.transferredEnergyJ == 0.0;
        }), "closed outlet must publish exactly zero transfer");
}

void testValveExchangeIsTwoWayAndConservative() {
    ExhaustGasNetworkConfig configuration;
    configuration.initialPressurePa = 120'000.0;
    configuration.initialTemperatureK = 500.0;
    auto network = makeNetwork(makeDefaultInlineFour(), configuration);
    const auto cylinderId = network.layout().cylinderPorts().front().cylinderId;
    GasComposition burnedMixture;
    burnedMixture.massFractions = { 0.02, 0.58, 0.0, 0.40 };
    const auto highCylinderState = network.mixtureModel().conservativeFromPressureTemperature(
        360'000.0, 1'050.0, 0.0, burnedMixture);
    requireNetwork(highCylinderState.has_value(), "hot cylinder boundary must be physical");
    const CylinderValveBoundary blowingDown {
        cylinderId, *highCylinderState, 5.0e-4, 1.8e-4, 0.78
    };
    const auto closedAmbient = ambientFor(network, 101'325.0, 300.0, 0.0);
    const auto beforeBlowdown = network.inventory();
    const auto blowdown = network.advance(0.00035,
        std::span<const CylinderValveBoundary>(&blowingDown, 1), closedAmbient);
    if (!blowdown.completed) {
        std::cerr << "blowdown diagnostics: advanced=" << blowdown.advancedTimeSeconds
                  << " accepted=" << blowdown.acceptedSubsteps
                  << " rejected=" << blowdown.rejectedSubsteps << '\n';
    }
    requireNetwork(blowdown.completed, "hot cylinder blowdown must complete");
    const auto forwardExchange = std::find_if(network.cylinderExchanges().begin(),
        network.cylinderExchanges().end(), [cylinderId](const CylinderGasExchange& exchange) {
            return exchange.cylinderId == cylinderId;
        });
    requireNetwork(forwardExchange != network.cylinderExchanges().end()
            && forwardExchange->totalMassKg() > 0.0
            && forwardExchange->speciesMassKg[static_cast<std::size_t>(GasSpecies::burned)] > 0.0
            && forwardExchange->totalEnergyJ > 0.0
            && forwardExchange->cylinderPressurePaAfter < 360'000.0
            && forwardExchange->networkDensityKgPerM3 > 0.0
            && forwardExchange->networkSpeedOfSoundMps > 0.0,
        "blowdown must transport hot burned gas and its energy into the network");
    requireMassEnergyBalance(beforeBlowdown, network.inventory(),
                             network.cylinderExchanges(), network.outletSamples(), 2.0e-9);

    const auto lowCylinderState = network.mixtureModel().conservativeFromPressureTemperature(
        55'000.0, 420.0);
    requireNetwork(lowCylinderState.has_value(), "low-pressure cylinder fixture must be physical");
    const CylinderValveBoundary reversion {
        cylinderId, *lowCylinderState, 5.0e-4, 1.8e-4, 0.78
    };
    const auto beforeReversion = network.inventory();
    const auto reversed = network.advance(0.00020,
        std::span<const CylinderValveBoundary>(&reversion, 1), closedAmbient);
    requireNetwork(reversed.completed, "exhaust reversion must complete");
    const auto reverseExchange = std::find_if(network.cylinderExchanges().begin(),
        network.cylinderExchanges().end(), [cylinderId](const CylinderGasExchange& exchange) {
            return exchange.cylinderId == cylinderId;
        });
    requireNetwork(reverseExchange != network.cylinderExchanges().end()
            && reverseExchange->totalMassKg() < 0.0
            && reverseExchange->totalEnergyJ < 0.0
            && reverseExchange->cylinderPressurePaAfter > 55'000.0,
        "network backpressure must be able to return mass and energy to a cylinder");
    requireMassEnergyBalance(beforeReversion, network.inventory(),
                             network.cylinderExchanges(), network.outletSamples(), 2.0e-9);
}

void testInstantaneousBoundarySamplingIsSignedAndNonMutating() {
    ExhaustGasNetworkConfig configuration;
    configuration.initialPressurePa = 150'000.0;
    configuration.initialTemperatureK = 620.0;
    auto network = makeNetwork(makeDefaultInlineFour(), configuration);
    const auto portCount = network.layout().cylinderPorts().size();
    std::vector<CylinderBoundaryFlowSample> samples(portCount);
    const auto cylinderId = network.layout().cylinderPorts().front().cylinderId;
    const auto highState = network.mixtureModel().conservativeFromPressureTemperature(
        310'000.0, 980.0);
    const auto lowState = network.mixtureModel().conservativeFromPressureTemperature(
        70'000.0, 430.0);
    requireNetwork(highState && lowState, "sampling fixtures must be physical");
    const auto before = network.inventory();

    CylinderValveBoundary boundary {
        cylinderId, *highState, 5.0e-4, 1.4e-4, 0.78
    };
    requireNetwork(network.sampleCylinderBoundaries(
            std::span<const CylinderValveBoundary>(&boundary, 1), samples),
        "instantaneous forward sampling must succeed");
    requireNetwork(samples.front().valid && samples.front().cylinderId == cylinderId
            && samples.front().massFlowKgPerSecond > 0.0
            && samples.front().networkPressurePa > 0.0
            && samples.front().networkDensityKgPerM3 > 0.0
            && samples.front().networkSpeedOfSoundMps > 0.0,
        "higher cylinder pressure must produce a positive physical boundary flow");

    boundary.cylinderState = *lowState;
    requireNetwork(network.sampleCylinderBoundaries(
            std::span<const CylinderValveBoundary>(&boundary, 1), samples)
            && samples.front().valid && samples.front().massFlowKgPerSecond < 0.0,
        "higher network pressure must produce signed exhaust reversion");
    const auto after = network.inventory();
    requireNetwork(before.speciesMassKg == after.speciesMassKg
            && before.totalEnergyJ == after.totalEnergyJ,
        "instantaneous sampling must not mutate conservative network inventory");

    std::array<CylinderBoundaryFlowSample, 1> tooSmall {};
    requireNetwork(!network.sampleCylinderBoundaries({}, tooSmall),
        "sampling must reject an output span smaller than the compiled port count");
}

void testResetIsAllocationFreeStateReinitialisation() {
    auto network = makeNetwork(makeDefaultInlineFour());
    const auto hotState = network.mixtureModel().conservativeFromPressureTemperature(
        280'000.0, 920.0);
    requireNetwork(hotState.has_value(), "reset fixture reservoir state must be physical");
    const CylinderValveBoundary cylinder { 1, *hotState, 0.00050, 0.00030, 0.78 };
    const auto ambient = ambientFor(network, 101'325.0, 300.0, 0.0);
    const auto advance = network.advance(0.0004,
        std::span<const CylinderValveBoundary>(&cylinder, 1), ambient);
    requireNetwork(advance.completed && network.inventory().totalEnergyJ > 0.0,
        "reset fixture must first perturb the network");

    requireNetwork(network.reset(93'000.0, 305.0),
        "a configured network must accept a physical uniform reset state");
    for (const auto& duct : network.ducts()) {
        for (const auto& state : duct.cells()) {
            const auto primitive = network.mixtureModel().primitiveFromConservative(state);
            requireNetwork(primitive.has_value()
                    && std::abs(primitive->pressurePa - 93'000.0) < 1.0e-8
                    && std::abs(primitive->temperatureK - 305.0) < 1.0e-10
                    && std::abs(primitive->velocityMps) < 1.0e-14,
                "reset must restore every duct cell to the requested quiescent state");
        }
    }
    requireNetwork(std::all_of(network.cylinderExchanges().begin(),
                        network.cylinderExchanges().end(),
        [](const CylinderGasExchange& exchange) {
            return exchange.totalMassKg() == 0.0 && exchange.totalEnergyJ == 0.0;
        }), "reset must clear prior cylinder exchange integrals");
    requireNetwork(!network.reset(-1.0, 305.0),
        "reset must reject a non-physical pressure without altering topology");
}

void testOutletFlowIsPhysicalAndConservative() {
    ExhaustGasNetworkConfig configuration;
    configuration.initialPressurePa = 165'000.0;
    configuration.initialTemperatureK = 680.0;
    auto network = makeNetwork(makeDefaultInlineFour(), configuration);
    const auto ambient = ambientFor(network, 101'325.0, 300.0, 1.0);
    const auto before = network.inventory();
    const auto result = network.advance(0.0012, {}, ambient);
    requireNetwork(result.completed, "pressurised network discharge must complete");
    requireNetwork(network.outletSamples().size() == 1,
        "legacy inline-four fixture must expose one outlet sample");
    const auto& sample = network.outletSamples().front();
    requireNetwork(sample.staticPressurePa > 0.0 && sample.temperatureK > 0.0
            && sample.densityKgPerM3 > 0.0 && sample.massFlowKgPerS > 0.0
            && sample.volumeFlowM3PerS > 0.0 && sample.totalEnergyFlowW > 0.0,
        "outlet telemetry must expose finite pressure, temperature and signed flow");
    requireMassEnergyBalance(before, network.inventory(),
                             network.cylinderExchanges(), network.outletSamples(), 3.0e-9);
}

[[nodiscard]] EngineConfig directChainConfig() {
    auto config = makeDefaultInlineFour();
    auto& path = config.exhaustPaths.front();
    ExhaustNetworkConfig authored;
    for (std::size_t index = 0; index < config.cylinders.size(); ++index) {
        const auto primaryId = static_cast<std::uint32_t>(100 + index);
        const auto catalystId = static_cast<std::uint32_t>(200 + index);
        const auto outletId = static_cast<std::uint32_t>(300 + index);
        ExhaustComponentConfig primary;
        primary.id = primaryId;
        primary.type = ExhaustComponentType::pipe;
        primary.lengthMm = 200.0;
        primary.diameterMm = 42.0;
        ExhaustComponentConfig catalyst = primary;
        catalyst.id = catalystId;
        catalyst.type = ExhaustComponentType::catalyst;
        catalyst.lengthMm = 120.0;
        ExhaustComponentConfig outlet = primary;
        outlet.id = outletId;
        outlet.type = ExhaustComponentType::outlet;
        outlet.lengthMm = 100.0;
        outlet.dischargeCoefficient = 0.80;
        authored.components.push_back(primary);
        authored.components.push_back(catalyst);
        authored.components.push_back(outlet);
        authored.cylinderConnections.push_back({ config.cylinders[index].id, primaryId });
        authored.connections.push_back({ primaryId, catalystId });
        authored.connections.push_back({ catalystId, outletId });
    }
    path.network = std::move(authored);
    normaliseEngineConfig(config);
    return config;
}

void testDirectDuctInterfaceTransmitsWavesWithoutInventoryLoss() {
    auto network = makeNetwork(directChainConfig());
    const auto primaryDescriptor = std::find_if(network.layout().ducts().begin(),
        network.layout().ducts().end(), [](const CompiledExhaustDuct& duct) {
            return duct.sourceComponentId == 100;
        });
    const auto catalystDescriptor = std::find_if(network.layout().ducts().begin(),
        network.layout().ducts().end(), [](const CompiledExhaustDuct& duct) {
            return duct.sourceComponentId == 200;
        });
    requireNetwork(primaryDescriptor != network.layout().ducts().end()
            && catalystDescriptor != network.layout().ducts().end(),
        "direct-interface fixture components must be compiled");
    const auto primaryIndex = static_cast<std::size_t>(
        std::distance(network.layout().ducts().begin(), primaryDescriptor));
    const auto catalystIndex = static_cast<std::size_t>(
        std::distance(network.layout().ducts().begin(), catalystDescriptor));
    auto& primary = network.ducts()[primaryIndex];
    constexpr double basePressurePa = 101'325.0;
    constexpr double baseDensity = 1.18;
    const auto base = network.mixtureModel().conservativeFromPrimitive(
        baseDensity, 0.0, basePressurePa, GasComposition::dryAir());
    requireNetwork(base.has_value(), "direct-interface base state must be physical");
    const auto basePrimitive = network.mixtureModel().primitiveFromConservative(*base);
    requireNetwork(basePrimitive.has_value(), "direct-interface base primitive must recover");
    for (std::size_t index = 0; index < primary.cells().size(); ++index) {
        const auto x = primary.cellCentreM(index);
        const auto offset = (x - 0.145) / 0.012;
        const auto pressurePerturbation = 180.0 * std::exp(-0.5 * offset * offset);
        const auto density = baseDensity + pressurePerturbation
            / (basePrimitive->speedOfSoundMps * basePrimitive->speedOfSoundMps);
        const auto velocity = pressurePerturbation
            / (baseDensity * basePrimitive->speedOfSoundMps);
        const auto state = network.mixtureModel().conservativeFromPrimitive(
            density, velocity, basePressurePa + pressurePerturbation,
            GasComposition::dryAir());
        requireNetwork(state.has_value(), "right-running pulse state must be physical");
        primary.cells()[index] = *state;
    }
    const auto before = network.inventory();
    const auto ambient = ambientFor(network, basePressurePa, 300.0, 0.0);
    const auto result = network.advance(0.00031, {}, ambient);
    requireNetwork(result.completed, "wave must traverse a direct duct interface");
    auto peakCatalystPressure = 0.0;
    for (const auto& state : network.ducts()[catalystIndex].cells()) {
        const auto primitive = network.mixtureModel().primitiveFromConservative(state);
        requireNetwork(primitive.has_value(), "transmitted wave must remain physical");
        peakCatalystPressure = std::max(peakCatalystPressure, primitive->pressurePa);
    }
    requireNetwork(peakCatalystPressure > basePressurePa + 20.0,
        "a two-port component edge must transmit the pressure characteristic without a fake plenum");
    requireMassEnergyBalance(before, network.inventory(),
                             network.cylinderExchanges(), network.outletSamples(), 3.0e-9);
}

void testBoundaryInputOrderIsIrrelevant() {
    auto first = makeNetwork(makeDefaultInlineFour());
    auto second = makeNetwork(makeDefaultInlineFour());
    std::vector<CylinderValveBoundary> boundaries;
    for (const auto& port : first.layout().cylinderPorts()) {
        const auto pressure = 170'000.0 + 12'000.0 * static_cast<double>(boundaries.size());
        const auto state = first.mixtureModel().conservativeFromPressureTemperature(
            pressure, 720.0);
        requireNetwork(state.has_value(), "order-independence cylinder state must be physical");
        boundaries.push_back({ port.cylinderId, *state, 5.0e-4, 8.0e-5, 0.75 });
    }
    auto reversed = boundaries;
    std::reverse(reversed.begin(), reversed.end());
    const auto ambientFirst = ambientFor(first, 101'325.0, 300.0, 0.0);
    const auto ambientSecond = ambientFor(second, 101'325.0, 300.0, 0.0);
    const auto firstResult = first.advance(0.00025, boundaries, ambientFirst);
    const auto secondResult = second.advance(0.00025, reversed, ambientSecond);
    requireNetwork(firstResult.completed && secondResult.completed,
        "both boundary orders must complete");
    const auto firstInventory = first.inventory();
    const auto secondInventory = second.inventory();
    for (std::size_t species = 0; species < gasSpeciesCount; ++species) {
        requireNetwork(firstInventory.speciesMassKg[species]
                == secondInventory.speciesMassKg[species],
            "boundary array order must be bit-invariant for species inventory");
    }
    requireNetwork(firstInventory.totalEnergyJ == secondInventory.totalEnergyJ,
        "boundary array order must be bit-invariant for network energy");

    boundaries.push_back(boundaries.front());
    const auto duplicate = first.advance(0.0001, boundaries, ambientFirst);
    requireNetwork(!duplicate.completed && duplicate.advancedTimeSeconds == 0.0,
        "duplicate cylinder boundaries must be rejected before changing state");
}

} // namespace

void runExhaustGasNetworkTests() {
    testUniformClosedNetworkIsInvariant();
    testValveExchangeIsTwoWayAndConservative();
    testInstantaneousBoundarySamplingIsSignedAndNonMutating();
    testResetIsAllocationFreeStateReinitialisation();
    testOutletFlowIsPhysicalAndConservative();
    testDirectDuctInterfaceTransmitsWavesWithoutInventoryLoss();
    testBoundaryInputOrderIsIrrelevant();
}
