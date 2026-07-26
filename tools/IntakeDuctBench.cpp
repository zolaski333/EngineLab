// CPU feasibility spike for a 1-D finite-volume intake.
//
// The intake is today a lumped 0-D path, which is why intake tuning cannot
// exist (see tests/IntakeTuningTests.cpp). The candidate fix reuses
// FiniteVolumeDuct -- the same solver the exhaust network runs -- as one 1-D
// duct per intake runner. Whether that fits the realtime budget is not
// guessable from the exhaust numbers: an intake runner is much SHORTER
// (150-300 mm vs ~800 mm primaries), so at a similar cell count its cells are
// smaller and the CFL step shrinks; but the gas is COLD (a ~ 340-450 m/s vs
// ~600 hot), which relaxes the same limit. These partly cancel and the
// balance has to be measured, which is all this tool does. Nothing here is
// wired into the simulator.
//
// The duct is excited the way an engine excites it, not left quiescent: the
// valve end alternates between a strong WOT draw (a prescribed ~23 kPa
// depression ghost state -- deliberately at the harsh end of real port
// depressions, so the numbers below are not flattered) and a closed valve
// (reflective), at an intake-event cadence, while the plenum end holds
// ambient. Substep counts and per-frame wall time are meaningless for a duct
// sitting still; the whole cost IS the CFL response to the waves.
//
// Read the output against the 240 Hz physics budget (4.167 ms/frame) and
// against the measured headroom in docs (the physics thread already runs at
// ~99 % of budget on the V8 at 6500 rpm, with the exhaust FV network at
// 29-52 % of that): the go/no-go for the intake network is whether N runners
// x this per-duct cost fits the REMAINING budget, serially or with the
// per-cylinder parallelism the runner solves trivially admit.

#include <enginelab/gasdynamics/FiniteVolumeDuct.hpp>

#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>

namespace {
using namespace enginelab::gasdynamics;

struct BenchResult final {
    double millisecondsPerFrame { 0.0 };
    double substepsPerFrame { 0.0 };
    bool ok { false };
};

// Advance one intake-conditions duct for `frames` frames of 1/240 s under a
// valve open/shut cycle and return the mean wall cost of one frame.
BenchResult benchDuct(std::size_t cellCount, double lengthM, int frames) {
    FiniteVolumeDuct duct;
    DuctGeometry geometry;
    geometry.lengthM = lengthM;
    geometry.diameterM = 0.038;
    geometry.cellCount = cellCount;
    geometry.wallFrictionEnabled = true;

    const auto& mixture = duct.mixtureModel();
    const auto ambient = mixture.conservativeFromPressureTemperature(101'325.0, 300.0);
    // Cold charge at a WOT high-rpm port depression: the valve-side ghost.
    const auto draw = mixture.conservativeFromPressureTemperature(78'000.0, 320.0);
    if (!ambient || !draw || !duct.configure(geometry, *ambient)) return {};

    const auto plenumEnd = DuctBoundaryCondition::prescribed(*ambient);
    const auto valveOpen = DuctBoundaryCondition::prescribed(*draw);
    const auto valveShut = DuctBoundaryCondition::reflective();

    constexpr double frameDt = 1.0 / 240.0;
    constexpr int warmupFrames = 48;
    BenchResult result;
    result.ok = true;
    std::size_t substeps = 0;
    auto elapsedNs = std::chrono::nanoseconds::zero();
    for (int frame = 0; frame < warmupFrames + frames; ++frame) {
        // ~50 Hz intake-event cadence (one cylinder at 6000 rpm): the valve
        // spends 2 frames of every 5 open, 3 shut.
        const auto& valve = (frame % 5) < 2 ? valveOpen : valveShut;
        const auto start = std::chrono::steady_clock::now();
        const auto advance = duct.advance(frameDt, plenumEnd, valve);
        const auto stop = std::chrono::steady_clock::now();
        if (!advance.completed) { result.ok = false; break; }
        if (frame < warmupFrames) continue;
        elapsedNs += std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);
        substeps += advance.acceptedSubsteps;
    }
    const auto n = static_cast<double>(frames);
    result.millisecondsPerFrame = static_cast<double>(elapsedNs.count()) * 1.0e-6 / n;
    result.substepsPerFrame = static_cast<double>(substeps) / n;
    return result;
}
}  // namespace

int main(int argc, char** argv) {
    auto frames = 1'440;  // 6 s simulated per configuration
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--frames" && index + 1 < argc) frames = std::atoi(argv[++index]);
        else {
            std::cerr << "usage: EngineLabIntakeDuctBench [--frames n]\n";
            return EXIT_FAILURE;
        }
    }

    constexpr double budgetMs = 1'000.0 / 240.0;
    std::cout << std::fixed << std::setprecision(3)
              << "--- 1-D intake runner cost, per 240 Hz frame (budget "
              << budgetMs << " ms) ---\n"
              << "cells,length_m,ms_per_duct,substeps,x4_ms,x8_ms,x12_ms,"
                 "x4_pct,x8_pct,x12_pct\n";
    auto allOk = true;
    for (const auto cells : { std::size_t { 8 }, std::size_t { 12 } }) {
        for (const auto length : { 0.20, 0.30 }) {
            const auto bench = benchDuct(cells, length, frames);
            allOk = allOk && bench.ok;
            const auto ms = bench.millisecondsPerFrame;
            std::cout << cells << ',' << length << ',' << ms << ','
                      << bench.substepsPerFrame
                      << ',' << ms * 4.0 << ',' << ms * 8.0 << ',' << ms * 12.0
                      << ',' << ms * 4.0 / budgetMs * 100.0
                      << ',' << ms * 8.0 / budgetMs * 100.0
                      << ',' << ms * 12.0 / budgetMs * 100.0 << '\n';
        }
    }
    if (!allOk) {
        std::cerr << "a duct advance failed to complete\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
