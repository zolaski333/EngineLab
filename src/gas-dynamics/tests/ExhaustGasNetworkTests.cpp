#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/foundation/EngineTypes.hpp>
#include <enginelab/gasdynamics/ExhaustGasNetwork.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <numbers>
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

void testMomentumCarryingJunctionRestsAtAmbientAndCarriesDirectedFlow() {
    ExhaustGasNetworkConfig directedConfiguration;
    directedConfiguration.evolveJunctionAxialMomentum = true;
    auto resting = makeNetwork(makeDefaultInlineFour(), directedConfiguration);
    const auto openAmbient = ambientFor(resting, 101'325.0, 300.0, 1.0);
    const auto beforeRest = resting.inventory();
    const auto rest = resting.advance(0.003, {}, openAmbient);
    requireNetwork(rest.completed && rest.rejectedSubsteps == 0,
        "a momentum-carrying junction must remain admissible at ambient equilibrium");
    const auto afterRest = resting.inventory();
    for (std::size_t species = 0; species < gasSpeciesCount; ++species) {
        requireNetwork(relativeError(afterRest.speciesMassKg[species],
                                     beforeRest.speciesMassKg[species]) < 2.0e-12,
            "junction momentum must not create species flow at uniform pressure");
    }
    requireNetwork(relativeError(afterRest.totalEnergyJ, beforeRest.totalEnergyJ) < 2.0e-12
            && std::abs(afterRest.resolvedAxialMomentumKgMps) < 1.0e-12,
        "junction wall-pressure balance must create neither energy nor axial momentum at rest");

    ExhaustGasNetworkConfig mixedConfiguration;
    auto mixed = makeNetwork(makeDefaultInlineFour(), mixedConfiguration);
    auto directed = makeNetwork(makeDefaultInlineFour(), directedConfiguration);
    const auto drive = directed.mixtureModel().conservativeFromPressureTemperature(
        220'000.0, 900.0);
    requireNetwork(drive.has_value(), "directed-junction drive state must be physical");
    std::vector<CylinderValveBoundary> boundaries;
    for (const auto& port : directed.layout().cylinderPorts()) {
        boundaries.push_back({ port.cylinderId, *drive, 1.0, 5.0e-4, 1.0 });
    }
    const auto mixedAmbient = ambientFor(mixed, 101'325.0, 300.0, 1.0);
    const auto directedAmbient = ambientFor(directed, 101'325.0, 300.0, 1.0);
    for (int step = 0; step < 80; ++step) {
        const auto mixedStep = mixed.advance(0.00025, boundaries, mixedAmbient);
        const auto directedStep = directed.advance(0.00025, boundaries, directedAmbient);
        requireNetwork(mixedStep.completed && directedStep.completed,
            "both junction formulations must complete the directed-flow fixture");
    }
    requireNetwork(mixed.layout().junctions().size() == 1
            && directed.layout().junctions().size() == 1,
        "directed-flow fixture must contain exactly one collector junction");
    const auto mixedJunction = mixed.mixtureModel().primitiveFromConservative(
        mixed.junctionStates().front());
    const auto directedJunction = directed.mixtureModel().primitiveFromConservative(
        directed.junctionStates().front());
    requireNetwork(mixedJunction.has_value() && directedJunction.has_value(),
        "collector junction primitives must remain recoverable");
    requireNetwork(mixedJunction->velocityMps == 0.0,
        "the legacy well-mixed junction must remain an explicit zero-momentum control");
    requireNetwork(directedJunction->velocityMps > 1.0,
        "a directed collector must retain a measurable graph-axis gas velocity");
    auto mixedOutletMassFlow = 0.0;
    auto directedOutletMassFlow = 0.0;
    for (const auto& outlet : mixed.outletSamples())
        mixedOutletMassFlow += outlet.massFlowKgPerS;
    for (const auto& outlet : directed.outletSamples())
        directedOutletMassFlow += outlet.massFlowKgPerS;
    requireNetwork(directedOutletMassFlow > mixedOutletMassFlow,
        "directed collector momentum must increase flow through the same downstream geometry");
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

void testChokedValveMatchesIsentropicNozzleFlow() {
    ExhaustGasNetworkConfig configuration;
    configuration.initialPressurePa = 120'000.0;
    configuration.initialTemperatureK = 500.0;
    auto network = makeNetwork(makeDefaultInlineFour(), configuration);
    const auto cylinderId = network.layout().cylinderPorts().front().cylinderId;
    GasComposition burnedMixture;
    burnedMixture.massFractions = { 0.02, 0.58, 0.0, 0.40 };
    const auto cylinderState = network.mixtureModel().conservativeFromPressureTemperature(
        360'000.0, 1'050.0, 0.0, burnedMixture);
    requireNetwork(cylinderState.has_value(), "choked-valve reservoir must be physical");
    const auto cylinderPrimitive = network.mixtureModel().primitiveFromConservative(
        *cylinderState);
    requireNetwork(cylinderPrimitive.has_value(),
        "choked-valve reservoir primitive must be recoverable");

    constexpr double valveAreaM2 = 1.8e-4;
    constexpr double valveDischargeCoefficient = 0.78;
    const CylinderValveBoundary boundary {
        cylinderId, *cylinderState, 5.0e-4,
        valveAreaM2, valveDischargeCoefficient
    };
    std::vector<CylinderBoundaryFlowSample> samples(
        network.layout().cylinderPorts().size());
    requireNetwork(network.sampleCylinderBoundaries(
        std::span<const CylinderValveBoundary>(&boundary, 1), samples),
        "choked-valve boundary sample must succeed");

    const auto port = network.layout().cylinderPorts().front();
    const auto effectiveAreaM2 = std::min(
        valveAreaM2 * valveDischargeCoefficient,
        port.runnerConnectionAreaM2 * port.dischargeCoefficient);
    const auto gamma = cylinderPrimitive->heatCapacityRatio;
    const auto gasConstant = cylinderPrimitive->pressurePa
        / (cylinderPrimitive->densityKgPerM3 * cylinderPrimitive->temperatureK);
    const auto criticalTemperatureRatio = 2.0 / (gamma + 1.0);
    const auto criticalPressureRatio = std::pow(
        criticalTemperatureRatio, gamma / (gamma - 1.0));
    requireNetwork(configuration.initialPressurePa / cylinderPrimitive->pressurePa
            < criticalPressureRatio,
        "analytic valve fixture must be in the choked regime");
    const auto expectedMassFlowKgPerSecond = effectiveAreaM2
        * cylinderPrimitive->pressurePa
        / std::sqrt(gasConstant * cylinderPrimitive->temperatureK)
        * std::sqrt(gamma)
        * std::pow(criticalTemperatureRatio,
            (gamma + 1.0) / (2.0 * (gamma - 1.0)));
    requireNetwork(relativeError(samples.front().massFlowKgPerSecond,
                                 expectedMassFlowKgPerSecond) < 2.0e-12,
        "valve boundary must reproduce the analytic choked-nozzle mass flow");
}

void testSubcriticalValveMatchesIsentropicNozzleFlow() {
    ExhaustGasNetworkConfig configuration;
    configuration.initialPressurePa = 300'000.0;
    configuration.initialTemperatureK = 900.0;
    auto network = makeNetwork(makeDefaultInlineFour(), configuration);
    const auto cylinderId = network.layout().cylinderPorts().front().cylinderId;
    GasComposition burnedMixture;
    burnedMixture.massFractions = { 0.02, 0.58, 0.0, 0.40 };
    const auto cylinderState = network.mixtureModel().conservativeFromPressureTemperature(
        360'000.0, 1'050.0, 0.0, burnedMixture);
    requireNetwork(cylinderState.has_value(), "subcritical reservoir must be physical");
    const auto cylinderPrimitive = network.mixtureModel().primitiveFromConservative(
        *cylinderState);
    requireNetwork(cylinderPrimitive.has_value(),
        "subcritical reservoir primitive must be recoverable");

    constexpr double valveAreaM2 = 1.8e-4;
    constexpr double valveDischargeCoefficient = 0.78;
    const CylinderValveBoundary boundary {
        cylinderId, *cylinderState, 5.0e-4,
        valveAreaM2, valveDischargeCoefficient
    };
    std::vector<CylinderBoundaryFlowSample> samples(
        network.layout().cylinderPorts().size());
    requireNetwork(network.sampleCylinderBoundaries(
        std::span<const CylinderValveBoundary>(&boundary, 1), samples),
        "subcritical boundary sample must succeed");

    const auto port = network.layout().cylinderPorts().front();
    const auto effectiveAreaM2 = std::min(
        valveAreaM2 * valveDischargeCoefficient,
        port.runnerConnectionAreaM2 * port.dischargeCoefficient);
    const auto gamma = cylinderPrimitive->heatCapacityRatio;
    const auto gasConstant = cylinderPrimitive->pressurePa
        / (cylinderPrimitive->densityKgPerM3 * cylinderPrimitive->temperatureK);
    const auto pressureRatio = configuration.initialPressurePa
        / cylinderPrimitive->pressurePa;
    const auto criticalPressureRatio = std::pow(
        2.0 / (gamma + 1.0), gamma / (gamma - 1.0));
    requireNetwork(pressureRatio > criticalPressureRatio && pressureRatio < 1.0,
        "analytic valve fixture must be in the subcritical regime");
    const auto expectedMassFlowKgPerSecond = effectiveAreaM2
        * cylinderPrimitive->pressurePa
        / std::sqrt(gasConstant * cylinderPrimitive->temperatureK)
        * std::sqrt(2.0 * gamma / (gamma - 1.0)
            * (std::pow(pressureRatio, 2.0 / gamma)
                - std::pow(pressureRatio, (gamma + 1.0) / gamma)));
    requireNetwork(relativeError(samples.front().massFlowKgPerSecond,
                                 expectedMassFlowKgPerSecond) < 2.0e-12,
        "valve boundary must reproduce analytic subcritical nozzle mass flow");
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

void testOpenEndDischargesTowardFreeExpansion() {
    // A terminal opening vents into the atmosphere, which is a reservoir and not
    // a neighbouring cell of cold dense air. The reference is Saint-Venant's
    // isentropic discharge velocity for the pressure ratio actually present --
    // a closed-form identity, so this gate cannot drift onto the simulator's own
    // output the way a recorded mass flow would.
    //
    // The bound is deliberately loose in both directions. A 0-D reservoir
    // boundary on a duct that is still filling cannot reach the ideal figure,
    // and a Riemann ghost cell deliberately stops short of it; what this catches
    // is the order-of-magnitude error the previous boundary made, which
    // discharged a 180 kPa tailpipe at 68 m/s where free expansion gives 699 and
    // left the whole exhaust 79 kPa above ambient (docs/physics-audit.md).
    ExhaustGasNetworkConfig configuration;
    configuration.initialPressurePa = 180'000.0;
    configuration.initialTemperatureK = 1'580.0;
    auto network = makeNetwork(makeDefaultInlineFour(), configuration);
    const auto ambient = ambientFor(network, 101'325.0, 295.0, 1.0);
    const auto before = network.inventory();
    const auto result = network.advance(0.0004, {}, ambient);
    requireNetwork(result.completed, "pressurised open-end discharge must complete");
    requireNetwork(network.outletSamples().size() == 1,
        "inline-four fixture must expose one outlet sample");
    const auto& sample = network.outletSamples().front();
    requireNetwork(sample.densityKgPerM3 > 0.0 && sample.temperatureK > 0.0,
        "outlet telemetry must stay physical while discharging");

    constexpr auto gamma = 1.3;
    constexpr auto gasConstant = 287.0;
    const auto pressureRatio = std::min(1.0, 101'325.0 / sample.staticPressurePa);
    const auto heatCapacityCp = gamma * gasConstant / (gamma - 1.0);
    const auto freeExpansionVelocity = std::sqrt(std::max(0.0,
        2.0 * heatCapacityCp * sample.temperatureK
            * (1.0 - std::pow(pressureRatio, (gamma - 1.0) / gamma))));
    requireNetwork(freeExpansionVelocity > 1.0,
        "the discharge fixture must actually hold a pressure ratio");
    const auto attained = sample.axialVelocityMps / freeExpansionVelocity;
    requireNetwork(attained > 0.25,
        "an open end must discharge at a substantial fraction of free expansion, "
        "not against the ambient acoustic impedance");
    requireNetwork(attained < 2.0,
        "an open-end discharge must not exceed free expansion");
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
        catalyst.catalystCellDensityCpsi = 400.0;
        catalyst.catalystOpenAreaRatio = 0.80;
        catalyst.catalystSubstrateVolumetricHeatCapacityJPerM3K =
            2'000'000.0;
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
    requireNetwork(catalystDescriptor->homogenisedCatalystMonolith
            && std::abs(catalystDescriptor->catalystOpenAreaRatio - 0.80) < 1.0e-12
            && catalystDescriptor->hydraulicDiameterM < 0.002
            && catalystDescriptor->flowAreaM2
                < catalystDescriptor->connectionAreaM2,
        "the gas solver must receive one homogenised cellular catalyst duct");
    requireNetwork(network.ducts()[static_cast<std::size_t>(std::distance(
                network.layout().ducts().begin(), catalystDescriptor))]
                .geometry().homogenisedCellularSubstrate,
        "the dynamic wall must aggregate the full cellular substrate, not one channel");
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

void testCatalystThermalStateAggregatesTheWholeSubstrate() {
    ExhaustGasNetworkConfig configuration;
    configuration.initialTemperatureK = 300.0;
    configuration.wallTemperatureK = 300.0;
    configuration.dynamicWallHeatTransferEnabled = true;
    configuration.wallThicknessM = 0.0015;
    configuration.wallDensityKgPerM3 = 7'900.0;
    configuration.wallSpecificHeatJPerKgK = 500.0;
    configuration.externalWallHeatTransferWPerM2K = 0.0;
    auto network = makeNetwork(directChainConfig(), configuration);
    const auto descriptor = std::find_if(network.layout().ducts().begin(),
        network.layout().ducts().end(), [](const CompiledExhaustDuct& duct) {
            return duct.sourceComponentId == 200;
        });
    requireNetwork(descriptor != network.layout().ducts().end()
            && descriptor->homogenisedCatalystMonolith,
        "the thermal catalyst fixture must compile its substrate");
    const auto ductIndex = static_cast<std::size_t>(std::distance(
        network.layout().ducts().begin(), descriptor));
    const auto housingAreaM2 = descriptor->flowAreaM2
        / descriptor->catalystOpenAreaRatio;
    const auto housingRadiusM = std::sqrt(housingAreaM2 / std::numbers::pi);
    const auto canOuterRadiusM = housingRadiusM + configuration.wallThicknessM;
    const auto substrateCapacityJPerK =
        (housingAreaM2 - descriptor->flowAreaM2) * descriptor->lengthM
        * descriptor->catalystSubstrateVolumetricHeatCapacityJPerM3K;
    const auto canCapacityJPerK = std::numbers::pi * descriptor->lengthM
        * (canOuterRadiusM * canOuterRadiusM
            - housingRadiusM * housingRadiusM)
        * configuration.wallDensityKgPerM3
        * configuration.wallSpecificHeatJPerKgK;
    const auto actualCapacityJPerK = network.ducts()[ductIndex]
        .wallThermalEnergyJ() / configuration.wallTemperatureK;
    requireNetwork(relativeError(actualCapacityJPerK,
                substrateCapacityJPerK + canCapacityJPerK) < 2.0e-12,
        "one wall state per axial cell must aggregate substrate solid and outer can capacity");
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
    std::vector<CylinderBoundaryFlowSample> orderedSamples(
        first.layout().cylinderPorts().size());
    std::vector<CylinderBoundaryFlowSample> reversedSamples(
        second.layout().cylinderPorts().size());
    requireNetwork(first.sampleCylinderBoundaries(boundaries, orderedSamples)
            && second.sampleCylinderBoundaries(reversed, reversedSamples),
        "ordered and generic boundary sampling paths must both succeed");
    for (std::size_t index = 0; index < orderedSamples.size(); ++index) {
        const auto& ordered = orderedSamples[index];
        const auto& generic = reversedSamples[index];
        requireNetwork(ordered.cylinderId == generic.cylinderId
                && ordered.pathIndex == generic.pathIndex
                && ordered.massFlowKgPerSecond == generic.massFlowKgPerSecond
                && ordered.networkPressurePa == generic.networkPressurePa
                && ordered.networkTemperatureK == generic.networkTemperatureK
                && ordered.networkDensityKgPerM3 == generic.networkDensityKgPerM3
                && ordered.networkVelocityMps == generic.networkVelocityMps
                && ordered.networkSpeedOfSoundMps == generic.networkSpeedOfSoundMps
                && ordered.valid == generic.valid,
            "ordered sampling must be bit-identical to ID-mapped sampling");
    }
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

[[nodiscard]] double measuredExhaustIgnitionDelaySeconds(
    double temperatureK, double pressureKpa, double equivalenceRatio,
    const ExhaustFuelReactionConfig& chemistry) {
    ExhaustGasNetworkConfig configuration;
    configuration.initialPressurePa = pressureKpa * 1'000.0;
    configuration.initialTemperatureK = temperatureK;
    configuration.wallTemperatureK = 300.0;
    const auto oxygenMassPerFuelMass = chemistry.oxygenMolesPerFuelMole
        * 0.032 / chemistry.fuelMolarMassKg;
    constexpr double oxygenMassFraction = 0.20;
    const auto fuelMassFraction = oxygenMassFraction * equivalenceRatio
        / oxygenMassPerFuelMass;
    requireNetwork(fuelMassFraction > 0.0
            && fuelMassFraction + oxygenMassFraction < 1.0,
        "induction sweep composition must be physical");
    configuration.initialComposition.massFractions = {
        oxygenMassFraction,
        1.0 - oxygenMassFraction - fuelMassFraction,
        fuelMassFraction,
        0.0,
    };
    auto network = makeNetwork(makeDefaultInlineFour(), configuration);
    constexpr double observationStepSeconds = 50.0e-6;
    constexpr double maximumObservationSeconds = 0.100;
    auto elapsedSeconds = 0.0;
    while (elapsedSeconds < maximumObservationSeconds) {
        const auto reaction = network.reactUnburnedFuel(
            observationStepSeconds, chemistry);
        elapsedSeconds += observationStepSeconds;
        if (reaction.releasedEnergyJoules > 0.0) return elapsedSeconds;
    }
    return std::numeric_limits<double>::infinity();
}

void testHotUnburnedFuelReactsConservatively() {
    ExhaustGasNetworkConfig hotConfiguration;
    hotConfiguration.initialTemperatureK = 1'150.0;
    GasComposition reactive;
    reactive.massFractions = { 0.20, 0.68, 0.08, 0.04 };
    hotConfiguration.initialComposition = reactive;
    auto hot = makeNetwork(makeDefaultInlineFour(), hotConfiguration);
    const auto before = hot.inventory();
    ExhaustFuelReactionConfig chemistry;
    chemistry.ignitionTemperatureK = 850.0;
    chemistry.reactionTimeConstantSeconds = 0.008;
    chemistry.reactionEfficiency = 0.94;
    chemistry.inductionTimeSeconds = 0.001;

    auto inductionSweepChemistry = chemistry;
    inductionSweepChemistry.ignitionTemperatureK = 900.0;
    inductionSweepChemistry.inductionTimeSeconds = 0.004;
    inductionSweepChemistry.inductionReferencePressureKpa = 101.325;
    inductionSweepChemistry.inductionActivationTemperatureK = 13'340.0;
    inductionSweepChemistry.inductionPressureExponent = 0.989;
    inductionSweepChemistry.inductionEquivalenceRatioExponent = -0.577;
    inductionSweepChemistry.inductionDecayTimeSeconds = 0.008;
    const auto referenceDelay = measuredExhaustIgnitionDelaySeconds(
        901.0, 101.325, 1.0, inductionSweepChemistry);
    const auto hotDelay = measuredExhaustIgnitionDelaySeconds(
        1'150.0, 101.325, 1.0, inductionSweepChemistry);
    const auto pressureDelay = measuredExhaustIgnitionDelaySeconds(
        901.0, 180.0, 1.0, inductionSweepChemistry);
    const auto leanDelay = measuredExhaustIgnitionDelaySeconds(
        901.0, 101.325, 0.50, inductionSweepChemistry);
    std::cout << "afterfire induction sweep: reference_ms="
              << referenceDelay * 1'000.0
              << " hot_ms=" << hotDelay * 1'000.0
              << " pressure_ms=" << pressureDelay * 1'000.0
              << " lean_ms=" << leanDelay * 1'000.0 << '\n';
    requireNetwork(std::isfinite(referenceDelay) && std::isfinite(hotDelay)
            && std::isfinite(pressureDelay) && std::isfinite(leanDelay),
        "the controlled induction sweep must ignite every flammable hot case");
    requireNetwork(referenceDelay > 0.0038 && referenceDelay < 0.0041,
        "the normalised correlation must preserve its authored reference delay");
    requireNetwork(hotDelay < referenceDelay * 0.10,
        "hotter reactive gas must accumulate induction materially faster");
    requireNetwork(pressureDelay < referenceDelay * 0.70,
        "higher local pressure must shorten the authored gasoline induction delay");
    requireNetwork(leanDelay > referenceDelay * 1.35,
        "the authored lean-mixture exponent must lengthen induction near the lean edge");

    auto flatInductionChemistry = inductionSweepChemistry;
    flatInductionChemistry.inductionActivationTemperatureK = 0.0;
    flatInductionChemistry.inductionPressureExponent = 0.0;
    flatInductionChemistry.inductionEquivalenceRatioExponent = 0.0;
    const std::array flatDelays {
        measuredExhaustIgnitionDelaySeconds(
            901.0, 101.325, 1.0, flatInductionChemistry),
        measuredExhaustIgnitionDelaySeconds(
            1'150.0, 101.325, 1.0, flatInductionChemistry),
        measuredExhaustIgnitionDelaySeconds(
            901.0, 180.0, 1.0, flatInductionChemistry),
        measuredExhaustIgnitionDelaySeconds(
            901.0, 101.325, 0.50, flatInductionChemistry),
    };
    for (const auto flatDelay : flatDelays) {
        requireNetwork(std::abs(flatDelay
                - flatInductionChemistry.inductionTimeSeconds) < 51.0e-6,
            "zero induction exponents must preserve the schema-7 flat timer");
    }
    const auto reaction = hot.reactUnburnedFuel(0.004, chemistry);
    const auto after = hot.inventory();
    const auto oxygen = static_cast<std::size_t>(GasSpecies::oxygen);
    const auto fuel = static_cast<std::size_t>(GasSpecies::fuel);
    const auto burned = static_cast<std::size_t>(GasSpecies::burned);
    requireNetwork(reaction.reactingControlVolumes > 0
            && reaction.burnedFuelMassKg > 0.0
            && reaction.releasedEnergyJoules > 0.0,
        "hot fuel and oxygen must release heat inside exhaust control volumes");
    requireNetwork(reaction.wallIgnitedControlVolumes == 0,
        "a gas-ignited kernel must not be relabelled after heat release");
    requireNetwork(after.speciesMassKg[fuel] < before.speciesMassKg[fuel]
            && after.speciesMassKg[oxygen] < before.speciesMassKg[oxygen]
            && after.speciesMassKg[burned] > before.speciesMassKg[burned],
        "exhaust reaction must consume real reactants and create burned products");
    requireNetwork(std::abs(totalMass(after.speciesMassKg)
            - totalMass(before.speciesMassKg)) < 1.0e-12,
        "exhaust reaction must conserve total species mass");
    requireNetwork(relativeError(after.totalEnergyJ - before.totalEnergyJ,
            reaction.releasedEnergyJoules) < 2.0e-12,
        "exhaust reaction telemetry must equal conservative energy increase");
    auto sourceEnergyJ = 0.0;
    auto sourceFuelKg = 0.0;
    for (std::size_t sourceIndex = 0;
         sourceIndex < reaction.sourceCount; ++sourceIndex) {
        const auto& source = reaction.sources[sourceIndex];
        sourceEnergyJ += source.releasedEnergyJoules;
        sourceFuelKg += source.burnedFuelMassKg;
        requireNetwork(source.nodeId != 0
                && source.axialPosition >= 0.0
                && source.axialPosition <= 1.0
                && source.flowAreaM2 > 0.0
                && source.speedOfSoundMps > 0.0,
            "every heat-release source must identify a usable physical injection site");
    }
    requireNetwork(reaction.sourceCount > 0
            && reaction.droppedSourceCount == 0
            && relativeError(sourceEnergyJ,
                reaction.releasedEnergyJoules) < 2.0e-12
            && relativeError(sourceFuelKg,
                reaction.burnedFuelMassKg) < 2.0e-12,
        "local acoustic sources must account for all conservative reaction energy and fuel");
    std::cout << "afterfire: volumes=" << reaction.reactingControlVolumes
              << " fuel_mg=" << reaction.burnedFuelMassKg * 1.0e6
              << " oxygen_mg=" << reaction.consumedOxygenMassKg * 1.0e6
              << " energy_j=" << reaction.releasedEnergyJoules << '\n';

    ExhaustGasNetworkConfig coldConfiguration = hotConfiguration;
    coldConfiguration.initialTemperatureK = 700.0;
    auto cold = makeNetwork(makeDefaultInlineFour(), coldConfiguration);
    const auto coldBefore = cold.inventory();
    const auto coldReaction = cold.reactUnburnedFuel(0.004, chemistry);
    const auto coldAfter = cold.inventory();
    requireNetwork(coldReaction.burnedFuelMassKg == 0.0
            && coldAfter.speciesMassKg == coldBefore.speciesMassKg
            && coldAfter.totalEnergyJ == coldBefore.totalEnergyJ,
        "sub-ignition exhaust mixture must remain an exact non-reacting state");
    requireNetwork(coldReaction.wallIgnitedControlVolumes == 0,
        "a cold pipe must not report a hot-surface ignition");

    // Induction belongs to each physical site and persists between network
    // coupling calls. Two 1.5 ms observations must not satisfy a 4 ms delay;
    // the third crosses it without inventing fuel.
    auto induced = makeNetwork(makeDefaultInlineFour(), hotConfiguration);
    auto inductionChemistry = chemistry;
    inductionChemistry.inductionTimeSeconds = 0.004;
    const auto inducedBefore = induced.inventory();
    const auto induction0 = induced.reactUnburnedFuel(
        0.0015, inductionChemistry);
    const auto induction1 = induced.reactUnburnedFuel(
        0.0015, inductionChemistry);
    const auto inducedWaiting = induced.inventory();
    const auto induction2 = induced.reactUnburnedFuel(
        0.0015, inductionChemistry);
    requireNetwork(induction0.releasedEnergyJoules == 0.0
            && induction1.releasedEnergyJoules == 0.0
            && inducedWaiting.speciesMassKg
                == inducedBefore.speciesMassKg
            && inducedWaiting.totalEnergyJ == inducedBefore.totalEnergyJ
            && induction2.releasedEnergyJoules > 0.0,
        "local induction must accumulate across coupling calls before ignition");

    // The authored induction time is defined AT the threshold. The former
    // hidden `(T-Tign)/450 K` activation made a 4 ms calibration take 1.8 s at
    // one kelvin above it and made a physically hot exhaust appear inert.
    ExhaustGasNetworkConfig thresholdConfiguration = hotConfiguration;
    thresholdConfiguration.initialTemperatureK =
        chemistry.ignitionTemperatureK + 1.0;
    auto threshold = makeNetwork(
        makeDefaultInlineFour(), thresholdConfiguration);
    auto thresholdChemistry = chemistry;
    thresholdChemistry.inductionTimeSeconds = 0.004;
    const auto threshold0 = threshold.reactUnburnedFuel(
        0.002, thresholdChemistry);
    const auto threshold1 = threshold.reactUnburnedFuel(
        0.002, thresholdChemistry);
    requireNetwork(threshold0.releasedEnergyJoules == 0.0
            && threshold1.releasedEnergyJoules > 0.0,
        "induction at the authored ignition threshold must use the authored delay");

    // Cold gas against a HOT pipe: the overrun case, and the one the gas-only
    // criterion could never serve. The mixture entering the exhaust on a
    // spark-cut overrun is pumped air, so it is below the threshold by
    // construction while the pipe it is flowing through is still glowing from
    // the preceding pull. Same 700 K gas as the run above, which does not
    // react, so anything that happens here is attributable to the wall alone.
    //
    // The wall is left static (dynamicWallHeatTransferEnabled off) precisely so
    // this asserts the ignition criterion and not the wall solver.
    ExhaustGasNetworkConfig hotWallConfiguration = coldConfiguration;
    hotWallConfiguration.wallTemperatureK = 1'000.0;
    auto hotWall = makeNetwork(makeDefaultInlineFour(), hotWallConfiguration);
    const auto hotWallBefore = hotWall.inventory();
    const auto hotWallReaction = hotWall.reactUnburnedFuel(0.004, chemistry);
    const auto hotWallAfter = hotWall.inventory();
    requireNetwork(hotWallReaction.wallIgnitedControlVolumes > 0
            && hotWallReaction.burnedFuelMassKg > 0.0
            && hotWallReaction.releasedEnergyJoules > 0.0,
        "a hot pipe must ignite an overrun mixture whose bulk gas is cold");
    requireNetwork(hotWallReaction.wallIgnitedControlVolumes
            == hotWallReaction.reactingControlVolumes,
        "with the gas below the threshold every reacting volume is wall-ignited");
    requireNetwork(std::abs(totalMass(hotWallAfter.speciesMassKg)
            - totalMass(hotWallBefore.speciesMassKg)) < 1.0e-12,
        "wall-ignited reaction must conserve total species mass");
    requireNetwork(relativeError(
            hotWallAfter.totalEnergyJ - hotWallBefore.totalEnergyJ,
            hotWallReaction.releasedEnergyJoules) < 2.0e-12,
        "wall-ignited telemetry must equal the conservative energy increase");
    std::cout << "wall afterfire: volumes=" << hotWallReaction.reactingControlVolumes
              << " wall_ignited=" << hotWallReaction.wallIgnitedControlVolumes
              << " fuel_mg=" << hotWallReaction.burnedFuelMassKg * 1.0e6
              << " energy_j=" << hotWallReaction.releasedEnergyJoules << '\n';
}

} // namespace

void runExhaustGasNetworkTests() {
    testUniformClosedNetworkIsInvariant();
    testMomentumCarryingJunctionRestsAtAmbientAndCarriesDirectedFlow();
    testValveExchangeIsTwoWayAndConservative();
    testChokedValveMatchesIsentropicNozzleFlow();
    testSubcriticalValveMatchesIsentropicNozzleFlow();
    testInstantaneousBoundarySamplingIsSignedAndNonMutating();
    testResetIsAllocationFreeStateReinitialisation();
    testOutletFlowIsPhysicalAndConservative();
    testOpenEndDischargesTowardFreeExpansion();
    testDirectDuctInterfaceTransmitsWavesWithoutInventoryLoss();
    testCatalystThermalStateAggregatesTheWholeSubstrate();
    testBoundaryInputOrderIsIrrelevant();
    testHotUnburnedFuelReactsConservatively();
}
