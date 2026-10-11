// One intake runner breathing into one moving-piston cylinder, advanced at a
// fixed outer interval while only the network's internal Courant number
// changes. The trapped charge at intake closing must not depend on the
// internal time step once that step is stable.
#include <enginelab/gasdynamics/ExhaustGasNetwork.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <string>
#include <string_view>

namespace {
using namespace enginelab::gasdynamics;

constexpr double runnerLengthM = 0.285;
constexpr double runnerDiameterM = 0.043;
constexpr double ambientPressurePa = 101'325.0;
constexpr double ambientTemperatureK = 300.0;
constexpr double displacementM3 = 0.5e-3;
constexpr double clearanceM3 = 0.05e-3;
inline double maximumValveAreaM2 = 1.3e-3;
inline bool characteristicBoundary = false;
// Intake valve opens 10 degrees before TDC and closes 50 after BDC.
constexpr double intakeOpenDeg = -10.0;
constexpr double intakeCloseDeg = 230.0;
constexpr double residualPressurePa = 105'000.0;
constexpr double residualTemperatureK = 700.0;

void require(bool condition, std::string_view message) {
    if (condition) return;
    std::cerr << "FAILED: " << message << '\n';
    std::exit(EXIT_FAILURE);
}

[[nodiscard]] ExhaustNetworkLayout runnerLayout(std::size_t cellCount) {
    const auto areaM2 = std::numbers::pi * runnerDiameterM * runnerDiameterM * 0.25;
    CompiledExhaustDuct duct;
    duct.nodeId = 100;
    duct.lengthM = runnerLengthM;
    duct.flowAreaM2 = areaM2;
    duct.inletFlowAreaM2 = areaM2;
    duct.outletFlowAreaM2 = areaM2;
    duct.connectionAreaM2 = areaM2;
    duct.inletConnectionAreaM2 = areaM2;
    duct.outletConnectionAreaM2 = areaM2;
    duct.hydraulicDiameterM = runnerDiameterM;
    duct.volumeM3 = areaM2 * runnerLengthM;
    duct.cellCount = cellCount;
    CompiledCylinderPort valve;
    valve.cylinderId = 1;
    valve.networkEndpoint = { ExhaustEndpointType::ductInlet, 0, 100 };
    valve.runnerConnectionAreaM2 = areaM2;
    CompiledExhaustOutlet mouth;
    mouth.outletNodeId = 200;
    mouth.networkEndpoint = { ExhaustEndpointType::ductOutlet, 0, 100 };
    mouth.openingAreaM2 = areaM2;
    return ExhaustNetworkLayout::assemble({ duct }, {}, {}, { valve }, { mouth });
}

[[nodiscard]] double cylinderVolumeM3(double crankDeg) {
    const auto angle = crankDeg * std::numbers::pi / 180.0;
    return clearanceM3 + 0.5 * displacementM3 * (1.0 - std::cos(angle));
}

[[nodiscard]] double valveAreaM2(double crankDeg) {
    const auto cycleDeg = std::fmod(crankDeg + 720.0 - intakeOpenDeg, 720.0) + intakeOpenDeg;
    if (cycleDeg <= intakeOpenDeg || cycleDeg >= intakeCloseDeg) return 0.0;
    const auto s = std::sin(std::numbers::pi * (cycleDeg - intakeOpenDeg)
        / (intakeCloseDeg - intakeOpenDeg));
    return maximumValveAreaM2 * s * s;
}

struct Result final {
    double trappedMassMg { 0.0 };
    std::size_t acceptedSubsteps { 0 };
    double courant { 0.0 };
};

[[nodiscard]] Result breathe(double courant, std::size_t cellCount, double rpm,
                             double outerSeconds, bool firstOrder) {
    ExhaustGasNetworkConfig config;
    config.initialPressurePa = ambientPressurePa;
    config.initialTemperatureK = ambientTemperatureK;
    config.maximumCourantNumber = courant;
    config.firstOrderTimeIntegration = firstOrder;
    config.characteristicValveBoundary = characteristicBoundary;
    ExhaustGasNetwork network;
    require(network.configure(runnerLayout(cellCount), config), "runner must configure");
    const auto& model = network.mixtureModel();
    const auto ambient = model.conservativeFromPressureTemperature(
        ambientPressurePa, ambientTemperatureK);
    require(ambient.has_value(), "ambient must be physical");
    const ExhaustAmbientBoundary mouth { *ambient, 1.0 };

    const auto degreesPerSecond = rpm * 6.0;
    const auto stepDeg = degreesPerSecond * outerSeconds;
    auto crankDeg = intakeOpenDeg - 1.0;
    auto volume = cylinderVolumeM3(crankDeg);
    auto state = *model.conservativeFromPressureTemperature(
        residualPressurePa, residualTemperatureK);
    // Mass and energy in the cylinder, carried as densities over `volume`.
    Result result;
    result.courant = courant;
    constexpr int cycles = 8;
    constexpr int measured = 4;
    auto trappedSum = 0.0;
    for (int cycle = 0; cycle < cycles; ++cycle) {
        // Start each cycle from the same residual, so the comparison is the
        // breathing alone.
        state = *model.conservativeFromPressureTemperature(
            residualPressurePa, residualTemperatureK);
        volume = cylinderVolumeM3(crankDeg);
        while (true) {
            const auto nextDeg = crankDeg + stepDeg;
            const auto nextVolume = cylinderVolumeM3(nextDeg);
            CylinderValveBoundary boundary;
            boundary.cylinderId = 1;
            boundary.cylinderState = state;
            boundary.cylinderVolumeM3 = volume;
            boundary.effectiveValveAreaM2 = valveAreaM2(crankDeg);
            boundary.dischargeCoefficient = 1.0;
            boundary.cylinderVolumeRateM3PerS = (nextVolume - volume) / outerSeconds;
            boundary.effectiveValveAreaRateM2PerS =
                (valveAreaM2(nextDeg) - valveAreaM2(crankDeg)) / outerSeconds;
            const auto advance = network.advance(outerSeconds, { &boundary, 1 }, mouth);
            require(advance.completed, "every outer interval must complete");
            result.acceptedSubsteps += advance.acceptedSubsteps;
            const auto& exchange = network.cylinderExchanges().front();
            // Rebuild the cylinder from its inventory: species and energy
            // move with the exchange, and the piston does p dV work at the
            // mean of the start and end pressures.
            const auto before = model.primitiveFromConservative(state);
            require(before.has_value(), "cylinder must stay physical");
            ConservativeState inventory;
            for (std::size_t species = 0; species < gasSpeciesCount; ++species)
                inventory.speciesMassDensityKgPerM3[species] =
                    state.speciesMassDensityKgPerM3[species] * volume
                    - exchange.speciesMassKg[species];
            auto energyJ = state.totalEnergyDensityJPerM3 * volume - exchange.totalEnergyJ
                - before->pressurePa * (nextVolume - volume);
            for (auto& species : inventory.speciesMassDensityKgPerM3) species /= nextVolume;
            inventory.totalEnergyDensityJPerM3 = energyJ / nextVolume;
            inventory.momentumDensityKgPerM2S = 0.0;
            state = inventory;
            volume = nextVolume;
            const auto wasOpen = valveAreaM2(crankDeg) > 0.0;
            crankDeg = nextDeg;
            if (wasOpen && valveAreaM2(crankDeg) == 0.0
                && std::fmod(crankDeg + 720.0, 720.0) > 180.0) {
                break;
            }
        }
        auto massKg = 0.0;
        for (const auto species : state.speciesMassDensityKgPerM3) massKg += species * volume;
        if (cycle >= cycles - measured) trappedSum += massKg;
        // Run the closed part of the cycle at the same outer cadence so the
        // runner keeps ringing as it would between two intake events.
        while (valveAreaM2(crankDeg + stepDeg) == 0.0) {
            const auto advance = network.advance(outerSeconds, {}, mouth);
            require(advance.completed, "closed intervals must complete");
            crankDeg += stepDeg;
        }
    }
    result.trappedMassMg = trappedSum / measured * 1.0e6;
    return result;
}

} // namespace

int main(int argc, char** argv) {
    // With arguments: an exploration table. rpm cells outer_us [euler|rk2]
    // valve_mm2 [char].
    if (argc > 1) {
        const auto rpm = std::stod(argv[1]);
        const auto cells = argc > 2 ? static_cast<std::size_t>(std::stoul(argv[2])) : 3U;
        const auto outerUs = argc > 3 ? std::stod(argv[3]) : 100.0;
        const auto firstOrder = argc > 4 && std::string_view { argv[4] } == "euler";
        if (argc > 5) maximumValveAreaM2 = std::stod(argv[5]) * 1.0e-6;
        characteristicBoundary = argc > 6 && std::string_view { argv[6] } == "char";
        std::cout << std::fixed << std::setprecision(3);
        for (const auto courant : { 0.8, 0.6, 0.4, 0.27, 0.15, 0.08 }) {
            const auto result = breathe(courant, cells, rpm, outerUs * 1.0e-6, firstOrder);
            std::cout << "courant " << courant << " trapped_mg " << result.trappedMassMg
                      << " substeps " << result.acceptedSubsteps << std::endl;
        }
        return 0;
    }

    // The quasi-steady nozzle against the first cell's average lost 6.5 % of
    // the charge at Courant 0.8 with a 1300 mm2 valve and 11 % with 2000 mm2
    // (its square-root law is stiff as the pressure difference vanishes). The
    // characteristic valve boundary must make the charge independent of the
    // solver's internal step.
    characteristicBoundary = true;
    for (const auto valveAreaMm2 : { 1300.0, 2000.0 }) {
        maximumValveAreaM2 = valveAreaMm2 * 1.0e-6;
        const auto reference = breathe(0.08, 3, 3500.0, 250.0e-6, false);
        for (const auto courant : { 0.8, 0.4 }) {
            const auto coarse = breathe(courant, 3, 3500.0, 250.0e-6, false);
            const auto error = std::abs(coarse.trappedMassMg / reference.trappedMassMg - 1.0);
            std::cout << "valve " << valveAreaMm2 << " mm2, Courant " << courant
                      << ": trapped " << coarse.trappedMassMg << " mg against "
                      << reference.trappedMassMg << " mg (" << error * 100.0 << " %)\n";
            require(error < 0.005,
                "the trapped charge must not depend on the internal Courant number");
        }
    }
    std::cout << "intake runner Courant independence: passed\n";
    return 0;
}
