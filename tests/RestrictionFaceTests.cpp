// A throttle plate between two 1-D ducts (CompiledExhaustInterface::
// restriction): a held tank feeds a 40 mm bore through a full-bore valve, the
// plate sits between two 200 mm ducts and the second one opens to a fixed
// atmosphere. Checked against the isentropic orifice where the answer is
// known (choked), against the plain duct continuation where the plate must
// vanish (full bore), and for exact mass conservation.
#include <enginelab/gasdynamics/ExhaustGasNetwork.hpp>

#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <string_view>
#include <vector>

namespace {
using namespace enginelab::gasdynamics;
constexpr double boreM = 0.040;
constexpr double boreAreaM2 = std::numbers::pi * boreM * boreM * 0.25;
constexpr double ambientPressurePa = 100'000.0;
constexpr double temperatureK = 300.0;
constexpr double tankVolumeM3 = 1.0;
constexpr double stepSeconds = 100.0e-6;

void require(bool condition, std::string_view message) {
    if (condition) return;
    std::cerr << "FAILED: " << message << '\n';
    std::exit(EXIT_FAILURE);
}

[[nodiscard]] CompiledExhaustDuct duct(std::uint32_t id) {
    CompiledExhaustDuct result;
    result.nodeId = id;
    result.lengthM = 0.200;
    result.flowAreaM2 = boreAreaM2;
    result.connectionAreaM2 = boreAreaM2;
    result.hydraulicDiameterM = boreM;
    result.volumeM3 = result.lengthM * boreAreaM2;
    result.cellCount = 4;
    return result;
}

[[nodiscard]] ExhaustNetworkLayout rig(bool restriction) {
    CompiledExhaustInterface plate;
    plate.upstream = { ExhaustEndpointType::ductOutlet, 0, 1 };
    plate.downstream = { ExhaustEndpointType::ductInlet, 1, 2 };
    plate.restriction = restriction;
    CompiledCylinderPort tank;
    tank.cylinderId = 1;
    tank.networkEndpoint = { ExhaustEndpointType::ductInlet, 0, 1 };
    tank.runnerConnectionAreaM2 = boreAreaM2;
    CompiledExhaustOutlet mouth;
    mouth.outletNodeId = 3;
    mouth.networkEndpoint = { ExhaustEndpointType::ductOutlet, 1, 2 };
    mouth.openingAreaM2 = boreAreaM2;
    return ExhaustNetworkLayout::assemble(
        { duct(1), duct(2) }, {}, { plate }, { tank }, { mouth });
}

struct Run final {
    double meanMassFlowKgPerS { 0.0 };
    double maximumRelativeMassError { 0.0 };
    std::size_t rejectedSteps { 0 };
};

// Mean outflow over the last quarter of `seconds`, the tank held at its state.
[[nodiscard]] Run run(bool restriction, double openingAreaM2,
                      double tankPressurePa, double seconds) {
    ExhaustGasNetwork network;
    ExhaustGasNetworkConfig config;
    config.initialPressurePa = ambientPressurePa;
    config.initialTemperatureK = temperatureK;
    config.wallHeatTransferWPerM2K = 0.0;
    config.dynamicWallHeatTransferEnabled = false;
    config.externalWallHeatTransferWPerM2K = 0.0;
    // The production valve boundary: the quasi-steady law rings an open valve
    // held at equilibrium into a limit cycle (the feed duct sat 12 % over the tank).
    config.characteristicValveBoundary = true;
    require(network.configure(rig(restriction), config), "the rig must configure");
    if (restriction)
        require(network.setRestrictionOpeningAreaM2(0, openingAreaM2),
            "a restriction takes an opening");
    else
        require(!network.setRestrictionOpeningAreaM2(0, openingAreaM2),
            "a plain face refuses an opening");
    const auto& model = network.mixtureModel();
    const auto tank = model.conservativeFromPressureTemperature(tankPressurePa, temperatureK);
    const auto air = model.conservativeFromPressureTemperature(ambientPressurePa, temperatureK);
    require(tank.has_value() && air.has_value(), "the reference states must recover");
    const CylinderValveBoundary valve { 1, *tank, tankVolumeM3, boreAreaM2, 1.0 };

    const auto networkMass = [&network] {
        auto total = 0.0;
        for (const auto species : network.inventory().speciesMassKg) total += species;
        return total;
    };
    const auto initialMass = networkMass();
    const auto steps = static_cast<std::size_t>(std::llround(seconds / stepSeconds));
    auto suppliedKg = 0.0;
    auto releasedKg = 0.0;
    auto windowReleasedKg = 0.0;
    Run result;
    for (std::size_t step = 0; step < steps; ++step) {
        const auto advance = network.advance(stepSeconds, std::span(&valve, 1),
            { *air, 1.0, 0.0 });
        require(advance.completed, "every advance must complete");
        result.rejectedSteps += advance.rejectedSubsteps;
        suppliedKg += network.cylinderExchanges().front().totalMassKg();
        auto outKg = 0.0;
        for (const auto species : network.outletSamples().front().speciesMassKg)
            outKg += species;
        releasedKg += outKg;
        if (step >= steps * 3 / 4) windowReleasedKg += outKg;
        const auto error = std::abs(networkMass() - initialMass - suppliedKg + releasedKg)
            / initialMass;
        result.maximumRelativeMassError = std::max(result.maximumRelativeMassError, error);
    }
    result.meanMassFlowKgPerS = windowReleasedKg
        / (static_cast<double>(steps - steps * 3 / 4) * stepSeconds);
    return result;
}

[[nodiscard]] double chokedMassFlowKgPerS(double areaM2, double pressurePa) {
    constexpr double gamma = 1.4;
    constexpr double gasConstant = 287.0;
    return areaM2 * pressurePa / std::sqrt(gasConstant * temperatureK) * std::sqrt(gamma)
        * std::pow(2.0 / (gamma + 1.0), (gamma + 1.0) / (2.0 * (gamma - 1.0)));
}
} // namespace

int main() {
    std::cout << std::setprecision(6);

    // Shut: nothing reaches the atmosphere, the duct behind the plate stays
    // exactly at rest.
    const auto shut = run(true, 0.0, 150'000.0, 0.05);
    std::cout << "shut flow_kg_s=" << shut.meanMassFlowKgPerS << '\n';
    require(std::abs(shut.meanMassFlowKgPerS) < 1.0e-12, "a shut plate passes nothing");

    // Choked: 5 % of the bore at a 3:1 pressure ratio. The approach velocity
    // is ~3 % of sound speed, so the tank is the stagnation state.
    const auto smallAreaM2 = 0.05 * boreAreaM2;
    const auto choked = run(true, smallAreaM2, 300'000.0, 0.1);
    const auto expected = chokedMassFlowKgPerS(smallAreaM2, 300'000.0);
    std::cout << "choked flow_kg_s=" << choked.meanMassFlowKgPerS
              << " isentropic_kg_s=" << expected
              << " mass_error=" << choked.maximumRelativeMassError
              << " rejected=" << choked.rejectedSteps << '\n';
    require(std::abs(choked.meanMassFlowKgPerS / expected - 1.0) < 0.01,
        "a choked plate passes the isentropic orifice flow");
    require(choked.maximumRelativeMassError < 1.0e-10, "mass is conserved");

    // Full bore: the plate must vanish against the plain continuation.
    const auto plain = run(false, boreAreaM2, 110'000.0, 0.1);
    const auto open = run(true, boreAreaM2, 110'000.0, 0.1);
    std::cout << "full_bore plain_kg_s=" << plain.meanMassFlowKgPerS
              << " plate_kg_s=" << open.meanMassFlowKgPerS << '\n';
    require(plain.meanMassFlowKgPerS > 0.01, "the open rig must flow");
    require(std::abs(open.meanMassFlowKgPerS / plain.meanMassFlowKgPerS - 1.0) < 0.02,
        "a plate open to the full bore costs nothing");

    // Part open: monotone in the opening.
    auto previous = 0.0;
    for (const auto fraction : { 0.1, 0.3, 0.6, 0.9 }) {
        const auto part = run(true, fraction * boreAreaM2, 110'000.0, 0.1);
        std::cout << "opening=" << fraction << " flow_kg_s=" << part.meanMassFlowKgPerS
                  << " mass_error=" << part.maximumRelativeMassError << '\n';
        require(part.meanMassFlowKgPerS > previous, "more opening passes more flow");
        require(part.maximumRelativeMassError < 1.0e-10, "mass is conserved");
        previous = part.meanMassFlowKgPerS;
    }
    require(previous <= open.meanMassFlowKgPerS * 1.001,
        "no opening passes more than the full bore");
    return EXIT_SUCCESS;
}
