// Steady-flow bench for the conservative exhaust network.
//
// Why this exists. The dyno sweep measures a mean collector pressure of 60-90
// kPa GAUGE at rated speed on every catalogue engine, and the pumping loop that
// follows from it (`pumpingMeanEffectivePressureBar`, -1.5 to -2.1 bar) is
// several times the -0.4 to -0.6 bar the literature puts on a naturally
// aspirated engine at wide-open throttle. A firing engine cannot say whether
// that is legitimate -- pulse superposition in a real 4-into-1 does raise the
// mean, and the blowdown transient is genuinely violent -- or whether the
// network is simply too restrictive for the pipe sizes it was handed.
//
// So take the engine out of it. Drive a STEADY mass flow through the authored
// network, wait for it to settle, and compare the pressure it holds against the
// closed-form Darcy-Weisbach loss for the same geometry at the same flow. A
// steady incompressible-ish duct flow has an exact answer, and the network is
// running the same solver, the same friction correlation, and the same terminal
// boundary it uses when an engine is attached.
//
// Reading the output. `K_measured` is the total loss coefficient the network
// actually imposes, referenced to the collector dynamic head:
//
//     K = (p_drive - p_ambient) / (rho_col * u_col^2 / 2)
//
// and `K_geometric` is what the authored pipe run can account for: the summed
// f*L/D of the ducts along the path, plus 1.0 for the kinetic energy that
// leaves at the tailpipe and is never recovered. A pipe run is allowed to
// exceed its friction term -- area changes, the merge, and the entry all cost
// something real -- but a healthy network sits within a small multiple. An
// order of magnitude means the loss is not coming from the geometry.
//
// The drive end is the cylinder ports, held wide open against a large fixed
// reservoir, because that is the path a firing engine actually uses; a bench
// that injected into the collector would skip the port and the primary.
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/foundation/EngineTypes.hpp>
#include <enginelab/gasdynamics/ExhaustGasNetwork.hpp>
#include <enginelab/gasdynamics/ExhaustNetworkLayout.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <string>
#include <vector>

namespace {
using namespace enginelab::gasdynamics;

bool containsCaseInsensitive(const std::string& text, const std::string& filter) {
    const auto lower = [](unsigned char value) { return static_cast<char>(std::tolower(value)); };
    std::string a(text.size(), '\0');
    std::string b(filter.size(), '\0');
    std::transform(text.begin(), text.end(), a.begin(), lower);
    std::transform(filter.begin(), filter.end(), b.begin(), lower);
    return a.find(b) != std::string::npos;
}

struct BenchPoint final {
    double drivePressureKpa { 0.0 };
    double massFlowKgPerS { 0.0 };
    double collectorGaugeKpa { 0.0 };
    double collectorVelocityMps { 0.0 };
    double collectorDensityKgPerM3 { 0.0 };
    double maxDuctGaugeKpa { 0.0 };
    double exitDuctGaugeKpa { 0.0 };
    double measuredK { 0.0 };
    /** Mass crossing the cylinder ports, on the same basis as the outlet flow.
     *  In a settled steady flow the two are equal; they are reported side by
     *  side because every number above is meaningless if they are not. */
    double portMassFlowKgPerS { 0.0 };
    /** Total-energy flux in at the ports and out at the outlets. The drive
     *  reservoir is at REST, so its static enthalpy is also its stagnation
     *  enthalpy: in a settled adiabatic network these two must be equal, and
     *  no interior state may exceed the drive's stagnation temperature. */
    double portEnergyW { 0.0 };
    double outletEnergyW { 0.0 };
    double hottestInteriorK { 0.0 };
    bool ok { false };
};

// The largest-area duct in the network stands in for "the collector": it is the
// single pipe the whole flow passes through, which is what a back-pressure
// figure is quoted against.
std::size_t widestDuctIndex(const ExhaustNetworkLayout& layout) {
    std::size_t best = 0;
    for (std::size_t index = 0; index < layout.ducts().size(); ++index)
        if (layout.ducts()[index].flowAreaM2 > layout.ducts()[best].flowAreaM2) best = index;
    return best;
}

// Summed f*L/D over every duct, weighted by nothing: an upper bound on the
// friction a single pass can accumulate, since the flow does not traverse every
// primary. Plus 1.0 for the exit velocity head. Friction factor at the fully
// rough limit for the roughness the simulator configures, which is generous --
// the real factor at these Reynolds numbers is smaller, so K_geometric errs
// toward EXCUSING the network.
double geometricLossCoefficient(const ExhaustNetworkLayout& layout) {
    auto total = 0.0;
    for (const auto& duct : layout.ducts()) {
        const auto relativeRoughness = 4.5e-5 / std::max(1.0e-6, duct.hydraulicDiameterM);
        const auto inverseRoot = -1.8 * std::log10(std::pow(relativeRoughness / 3.7, 1.11));
        const auto frictionFactor = 1.0 / (inverseRoot * inverseRoot);
        total += frictionFactor * duct.lengthM / std::max(1.0e-6, duct.hydraulicDiameterM);
    }
    return total + 1.0;
}

BenchPoint runPoint(const enginelab::EngineConfig& config,
                    const ExhaustNetworkLayout& layout,
                    double drivePressureKpa, double driveTemperatureK,
                    double settleSeconds, bool dumpProfile) {
    BenchPoint point;
    point.drivePressureKpa = drivePressureKpa;

    ExhaustGasNetwork network;
    ExhaustGasNetworkConfig networkConfig;
    networkConfig.initialPressurePa = config.ambientPressureKpa * 1'000.0;
    networkConfig.initialTemperatureK = config.ambientTemperatureC + 273.15;
    networkConfig.absoluteRoughnessM = 4.5e-5;
    networkConfig.maximumCourantNumber = 0.8;
    // Adiabatic walls. A steady bench must not have its density set by a wall
    // temperature that is itself a transient state: the loss coefficient this
    // reports would then depend on how long the bench had been running.
    networkConfig.wallHeatTransferWPerM2K = 0.0;
    networkConfig.dynamicWallHeatTransferEnabled = false;
    networkConfig.externalWallHeatTransferWPerM2K = 0.0;
    if (!network.configure(layout, networkConfig)) return point;

    const auto& mixture = network.mixtureModel();
    const auto ambient = mixture.conservativeFromPressureTemperature(
        config.ambientPressureKpa * 1'000.0, config.ambientTemperatureC + 273.15);
    const auto drive = mixture.conservativeFromPressureTemperature(
        drivePressureKpa * 1'000.0, driveTemperatureK);
    if (!ambient || !drive) return point;

    ExhaustAmbientBoundary ambientBoundary;
    ambientBoundary.reservoirState = *ambient;
    ambientBoundary.openingScale = 1.0;

    // Every port held wide open onto a reservoir big enough that the steady
    // draw cannot deplete it inside the settle window, so the drive pressure is
    // a boundary condition and not a decaying initial condition.
    std::vector<CylinderValveBoundary> boundaries;
    boundaries.reserve(layout.cylinderPorts().size());
    for (const auto& port : layout.cylinderPorts()) {
        CylinderValveBoundary boundary;
        boundary.cylinderId = port.cylinderId;
        boundary.cylinderState = *drive;
        boundary.cylinderVolumeM3 = 1.0;
        boundary.effectiveValveAreaM2 = port.runnerConnectionAreaM2;
        boundary.dischargeCoefficient = 1.0;
        boundaries.push_back(boundary);
    }

    // Settle time is a knob, not a constant, because "settled" is a claim this
    // bench has to be able to test rather than assert: if the reported loss
    // changes when the settle window is lengthened, the flow was still moving
    // and every number in the table is void.
    constexpr double dt = 1.0 / 2'000.0;
    const auto settleSteps = static_cast<int>(settleSeconds / dt);
    const auto sampleSteps = static_cast<int>(0.1 / dt);
    auto flowAcc = 0.0, collectorAcc = 0.0, velocityAcc = 0.0, densityAcc = 0.0;
    auto maxAcc = 0.0, exitAcc = 0.0, samples = 0.0, portFlowAcc = 0.0;
    auto portEnergyAcc = 0.0, outletEnergyAcc = 0.0, hottestK = 0.0;
    const auto collectorIndex = widestDuctIndex(layout);
    for (int step = 0; step < settleSteps + sampleSteps; ++step) {
        for (auto& boundary : boundaries) boundary.cylinderState = *drive;
        const auto result = network.advance(dt, boundaries, ambientBoundary);
        if (!result.completed) return point;
        if (step < settleSteps) continue;
        samples += 1.0;
        for (const auto& outlet : network.outletSamples()) {
            flowAcc += outlet.massFlowKgPerS;
            outletEnergyAcc += outlet.totalEnergyFlowW;
        }
        for (const auto& exchange : network.cylinderExchanges()) {
            portFlowAcc += exchange.totalMassKg() / dt;
            portEnergyAcc += exchange.totalEnergyJ / dt;
        }
        const auto& ducts = network.ducts();
        auto ductMax = 0.0;
        for (std::size_t index = 0; index < ducts.size(); ++index) {
            for (const auto& primitive : ducts[index].cellPrimitives()) {
                ductMax = std::max(ductMax,
                    primitive.pressurePa * 0.001 - config.ambientPressureKpa);
                hottestK = std::max(hottestK, primitive.temperatureK);
            }
        }
        for (const auto& state : network.junctionStates()) {
            const auto primitive = mixture.primitiveFromConservative(state);
            if (primitive) hottestK = std::max(hottestK, primitive->temperatureK);
        }
        maxAcc += ductMax;
        const auto& collector = ducts[collectorIndex].cellPrimitives();
        auto pressureSum = 0.0, velocitySum = 0.0, densitySum = 0.0;
        for (const auto& primitive : collector) {
            pressureSum += primitive.pressurePa * 0.001;
            velocitySum += std::abs(primitive.velocityMps);
            densitySum += primitive.densityKgPerM3;
        }
        const auto cells = static_cast<double>(std::max<std::size_t>(1, collector.size()));
        collectorAcc += pressureSum / cells - config.ambientPressureKpa;
        velocityAcc += velocitySum / cells;
        densityAcc += densitySum / cells;
        exitAcc += collector.back().pressurePa * 0.001 - config.ambientPressureKpa;
    }
    const auto d = std::max(1.0, samples);
    point.massFlowKgPerS = flowAcc / d;
    point.collectorGaugeKpa = collectorAcc / d;
    point.collectorVelocityMps = velocityAcc / d;
    point.collectorDensityKgPerM3 = densityAcc / d;
    point.maxDuctGaugeKpa = maxAcc / d;
    point.exitDuctGaugeKpa = exitAcc / d;
    point.portMassFlowKgPerS = portFlowAcc / d;
    point.portEnergyW = portEnergyAcc / d;
    point.outletEnergyW = outletEnergyAcc / d;
    point.hottestInteriorK = hottestK;
    const auto dynamicHeadKpa = 0.5 * point.collectorDensityKgPerM3
        * point.collectorVelocityMps * point.collectorVelocityMps * 0.001;
    point.measuredK = dynamicHeadKpa > 1.0e-6
        ? (drivePressureKpa - config.ambientPressureKpa) / dynamicHeadKpa : 0.0;
    point.ok = true;
    if (dumpProfile) {
        // Element by element, at the end of the sample window. A single "max
        // duct pressure" cannot say WHERE the loss is, and a cell reading above
        // the drive pressure -- impossible in a settled steady flow -- is only
        // visible against its neighbours.
        // rho*u is printed, not left to be inferred from p and T: in a settled
        // constant-area duct it must be the same in every cell, and that single
        // number is the check on whether a profile means anything.
        std::cout << "   profile at drive=" << drivePressureKpa << " kPa"
                     " (gauge kPa / m per s / K / rho*u kg per m2 s):\n";
        const auto& ducts = network.ducts();
        for (std::size_t index = 0; index < ducts.size(); ++index) {
            const auto& duct = layout.ducts()[index];
            std::cout << "     duct " << index << " L=" << duct.lengthM
                      << " D=" << duct.hydraulicDiameterM
                      << " A=" << duct.flowAreaM2 << " :";
            for (const auto& primitive : ducts[index].cellPrimitives())
                std::cout << "  " << primitive.pressurePa * 0.001
                             - config.ambientPressureKpa
                          << '/' << primitive.velocityMps
                          << '/' << primitive.temperatureK
                          << '/' << primitive.densityKgPerM3 * primitive.velocityMps;
            std::cout << '\n';
        }
        for (std::size_t index = 0; index < network.junctionStates().size(); ++index) {
            const auto primitive = mixture.primitiveFromConservative(
                network.junctionStates()[index]);
            if (primitive)
                std::cout << "     junction " << index << " : "
                          << primitive->pressurePa * 0.001 - config.ambientPressureKpa
                          << '/' << primitive->velocityMps
                          << '/' << primitive->temperatureK << '\n';
        }
    }
    return point;
}

void benchEngine(const enginelab::EngineConfig& baseConfig) {
    auto config = baseConfig;
    enginelab::normaliseEngineConfig(config);
    const auto graph = enginelab::ExhaustGraph::makeForEngine(config);
    // The same mesh the simulator compiles, so this benches the shipped
    // discretisation and not a finer one that would flatter it.
    ExhaustNetworkDiscretisation mesh;
    mesh.targetCellLengthM = 0.300;
    mesh.minimumCellsPerDuct = 1;
    mesh.maximumCellsPerDuct = 64;
    mesh.maximumTotalCells = 1'024;
    const auto layout = ExhaustNetworkLayout::compile(graph, mesh);

    const auto geometric = geometricLossCoefficient(layout);
    auto totalCells = std::size_t { 0 };
    for (const auto& duct : layout.ducts()) totalCells += duct.cellCount;
    std::cout << "== " << config.name << "\n"
              << "   ducts=" << layout.ducts().size()
              << " junctions=" << layout.junctions().size()
              << " ports=" << layout.cylinderPorts().size()
              << " cells=" << totalCells
              << " K_geometric=" << geometric << '\n';
    std::cout << "   settle_s  drive_kPa  mdot_out  mdot_port  col_gauge  col_mps"
                 "  col_rho  maxDuct  K_meas  K/K_geom  E_port_kW  E_out_kW  Tmax_K\n";
    // Two settle times per point. The pair is the measurement: matching rows
    // mean the flow is steady and the loss is real, diverging rows mean the
    // network is still ringing and nothing below may be quoted.
    for (const auto settle : { 0.5, 2.0 }) {
        for (const auto drive : { 105.0, 115.0, 135.0, 165.0, 205.0 }) {
            const auto point = runPoint(config, layout, drive, 1'050.0, settle,
                                        drive == 135.0 && settle == 2.0);
            if (!point.ok) {
                std::cout << "   " << std::setw(8) << settle
                          << std::setw(11) << drive << "  FAILED\n";
                continue;
            }
            std::cout << "   " << std::setw(8) << settle
                      << std::setw(11) << point.drivePressureKpa
                      << std::setw(10) << point.massFlowKgPerS
                      << std::setw(11) << point.portMassFlowKgPerS
                      << std::setw(11) << point.collectorGaugeKpa
                      << std::setw(9) << point.collectorVelocityMps
                      << std::setw(9) << point.collectorDensityKgPerM3
                      << std::setw(9) << point.maxDuctGaugeKpa
                      << std::setw(8) << point.measuredK
                      << std::setw(10) << point.measuredK / std::max(1.0e-9, geometric)
                      << std::setw(11) << point.portEnergyW * 0.001
                      << std::setw(10) << point.outletEnergyW * 0.001
                      << std::setw(8) << point.hottestInteriorK
                      << '\n';
        }
    }
}
}  // namespace

int main(int argc, char** argv) {
    std::filesystem::path catalogRoot = std::filesystem::current_path();
    std::string filter;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--catalog-root" && index + 1 < argc) catalogRoot = argv[++index];
        else if (argument == "--filter" && index + 1 < argc) filter = argv[++index];
        else {
            std::cerr << "usage: EngineLabExhaustFlowBench [--catalog-root dir]"
                         " [--filter name-fragment]\n";
            return EXIT_FAILURE;
        }
    }

    const auto catalog = enginelab::loadEngineCatalog(catalogRoot);
    if (catalog.entries.empty()) {
        for (const auto& error : catalog.errors) std::cerr << error << '\n';
        std::cerr << "no engines loaded\n";
        return EXIT_FAILURE;
    }
    std::cout << std::fixed << std::setprecision(3);
    for (const auto& entry : catalog.entries)
        if (filter.empty() || containsCaseInsensitive(entry.config.name, filter))
            benchEngine(entry.config);
    return EXIT_SUCCESS;
}
