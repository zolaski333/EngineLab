// Micro-instrument for the 1-D intake runner solver.
//
// The runner network is 75-84 % of the mechanical sub-step (see CLAUDE.md), so
// this is where realtime capacity is won or lost. Iterating on it through the
// full simulator costs ten minutes a round trip; this tool costs seconds, and
// it answers the two questions that matter, together:
//
//   1. what does one duct advance COST, at the cadence the simulator actually
//      calls it -- one mechanical sub-step, not one 240 Hz frame; and
//   2. is a candidate optimisation BIT-IDENTICAL?
//
// (2) is the reason this tool exists in this form. The idles in this project
// are ULP-sensitive attractors, so "the numbers look the same" is not a
// standard an optimisation can be held to. The checksum below is a bit-exact
// fingerprint of every conservative variable and every wall temperature in the
// duct after a fixed, deterministic excitation. If it is unchanged, the
// optimisation provably did not touch the physics and no catalogue re-run is
// needed to prove it. If it moves, the change is a physics change wearing an
// optimisation's clothes and has to be measured on the catalogue.
//
// The excitation is the engine's, not a quiescent duct: the valve end
// alternates between a strong WOT draw (a ~23 kPa depression ghost, at the
// harsh end of real port depressions so the cost is not flattered) and a shut
// valve, at an intake-event cadence, while the plenum end holds ambient.
// Dynamic wall heat transfer is ON, as the simulator configures it -- it is a
// quarter of the advance and leaving it out would measure a duct the simulator
// never runs.

#include <enginelab/gasdynamics/FiniteVolumeDuct.hpp>

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>

namespace {
using namespace enginelab::gasdynamics;

// Aluminium runner wall, matching EngineSimulator::configurePhysicalIntakeNetworks.
// Keep these in step with it: the point of this bench is to measure the duct the
// simulator actually advances, and the thickness and external coefficient below
// had drifted from it once already.
constexpr double aluminiumRunnerWallThicknessM = 0.003;
constexpr double aluminiumDensityKgPerM3 = 2'700.0;
constexpr double aluminiumSpecificHeatJPerKgK = 900.0;
constexpr double runnerExternalHeatTransferWPerM2K = 12.0;
constexpr double ductWallHeatUpdateIntervalSeconds = 150.0e-6;

/**
 * Order-sensitive FNV-1a over the raw bits of every double of state. Bit
 * equality is the whole point, so the doubles are hashed as bytes and never
 * compared with a tolerance.
 */
class BitChecksum final {
public:
    void add(double value) noexcept {
        std::uint64_t bits = 0;
        std::memcpy(&bits, &value, sizeof(bits));
        for (int byte = 0; byte < 8; ++byte) {
            hash_ ^= (bits >> (byte * 8)) & 0xFFull;
            hash_ *= 0x100000001B3ull;
        }
    }
    [[nodiscard]] std::uint64_t value() const noexcept { return hash_; }

private:
    std::uint64_t hash_ { 0xCBF29CE484222325ull };
};

struct BenchResult final {
    double nanosecondsPerCall { 0.0 };
    double nanosecondsPerCellSubstep { 0.0 };
    double substepsPerCall { 0.0 };
    std::uint64_t rejected { 0 };
    std::uint64_t checksum { 0 };
    bool ok { false };
};

/**
 * Advances one runner-shaped duct at the simulator's own cadence: a mechanical
 * sub-step of `maximumCrankDegreesPerStep` at `rpm`, halved, because the
 * simulator splits the advance around the exhaust coupling while the intake
 * valve is open.
 */
BenchResult benchDuct(std::size_t cellCount, double lengthM, double rpm,
                      int calls) {
    FiniteVolumeDuct duct;
    DuctGeometry geometry;
    geometry.lengthM = lengthM;
    geometry.diameterM = 0.038;
    geometry.cellCount = cellCount;
    geometry.wallFrictionEnabled = true;
    geometry.absoluteRoughnessM = 1.5e-6;
    geometry.dynamicWallHeatTransferEnabled = true;
    geometry.wallHeatTransferWPerM2K = 0.0;
    geometry.wallTemperatureK = 300.0;
    geometry.wallThicknessM = aluminiumRunnerWallThicknessM;
    geometry.wallDensityKgPerM3 = aluminiumDensityKgPerM3;
    geometry.wallSpecificHeatJPerKgK = aluminiumSpecificHeatJPerKgK;
    geometry.externalWallHeatTransferWPerM2K = runnerExternalHeatTransferWPerM2K;
    geometry.externalTemperatureK = 300.0;
    geometry.wallHeatUpdateIntervalSeconds = ductWallHeatUpdateIntervalSeconds;

    const auto& mixture = duct.mixtureModel();
    const auto ambient = mixture.conservativeFromPressureTemperature(101'325.0, 300.0);
    const auto draw = mixture.conservativeFromPressureTemperature(78'000.0, 320.0);
    if (!ambient || !draw || !duct.configure(geometry, *ambient)) return {};

    const auto plenumEnd = DuctBoundaryCondition::prescribed(*ambient);
    const auto valveOpen = DuctBoundaryCondition::prescribed(*draw);
    const auto valveShut = DuctBoundaryCondition::reflective();

    // One mechanical sub-step at 2 crank degrees, halved: what the simulator
    // asks for while the intake valve is open.
    const auto halfSubstepSeconds = 0.5 * 2.0 / (rpm * 6.0);
    // A four-stroke intake event is ~240 of 720 crank degrees, so the valve is
    // open for one call in three at this cadence.
    constexpr int valveOpenCallsPerCycle = 120;
    constexpr int cycleCalls = 360;
    constexpr int warmupCalls = 2'000;

    BenchResult result;
    result.ok = true;
    std::size_t substeps = 0;
    auto elapsedNs = std::chrono::nanoseconds::zero();
    for (int call = 0; call < warmupCalls + calls; ++call) {
        const auto& valve = (call % cycleCalls) < valveOpenCallsPerCycle
            ? valveOpen : valveShut;
        const auto start = std::chrono::steady_clock::now();
        const auto advance = duct.advance(halfSubstepSeconds, plenumEnd, valve);
        const auto stop = std::chrono::steady_clock::now();
        if (!advance.completed) { result.ok = false; break; }
        if (call < warmupCalls) continue;
        elapsedNs += std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);
        substeps += advance.acceptedSubsteps;
        result.rejected += advance.rejectedSubsteps;
    }

    BitChecksum checksum;
    for (const auto& cell : duct.cells()) {
        for (const auto density : cell.speciesMassDensityKgPerM3) checksum.add(density);
        checksum.add(cell.momentumDensityKgPerM2S);
        checksum.add(cell.totalEnergyDensityJPerM3);
    }
    for (const auto& wall : duct.wallStates()) checksum.add(wall.temperatureK);
    result.checksum = checksum.value();

    const auto n = static_cast<double>(calls);
    result.nanosecondsPerCall = static_cast<double>(elapsedNs.count()) / n;
    result.substepsPerCall = static_cast<double>(substeps) / n;
    result.nanosecondsPerCellSubstep = substeps > 0
        ? static_cast<double>(elapsedNs.count())
            / (static_cast<double>(substeps) * static_cast<double>(cellCount))
        : 0.0;
    return result;
}
}  // namespace

int main(int argc, char** argv) {
    auto calls = 20'000;
    auto rpm = 7'000.0;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--calls" && index + 1 < argc) calls = std::atoi(argv[++index]);
        else if (argument == "--rpm" && index + 1 < argc) rpm = std::atof(argv[++index]);
        else {
            std::cerr << "usage: EngineLabIntakeDuctBench [--calls n] [--rpm n]\n"
                         "  Cost of one intake-runner advance at the simulator's own\n"
                         "  cadence, plus a bit-exact checksum of the resulting state.\n"
                         "  An optimisation that leaves every checksum unchanged is\n"
                         "  provably not a physics change.\n";
            return EXIT_FAILURE;
        }
    }

    std::cout << "--- 1-D intake runner: cost per half-sub-step advance at "
              << std::fixed << std::setprecision(0) << rpm << " rpm ---\n"
              << "cells,length_m,ns_per_call,ns_per_cell_substep,substeps,"
                 "rejected,checksum\n";
    auto allOk = true;
    for (const auto cells : { std::size_t { 6 }, std::size_t { 9 }, std::size_t { 12 } }) {
        for (const auto length : { 0.18, 0.30 }) {
            const auto bench = benchDuct(cells, length, rpm, calls);
            allOk = allOk && bench.ok;
            std::cout << cells << ',' << std::setprecision(2) << length << ','
                      << std::setprecision(1) << bench.nanosecondsPerCall << ','
                      << bench.nanosecondsPerCellSubstep << ','
                      << std::setprecision(3) << bench.substepsPerCall << ','
                      << bench.rejected << ','
                      << "0x" << std::hex << bench.checksum << std::dec << '\n';
        }
    }
    if (!allOk) {
        std::cerr << "a duct advance failed to complete\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
