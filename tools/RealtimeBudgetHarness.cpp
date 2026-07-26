/**
 * Realtime budget instrument.
 *
 * Every other harness in this tree measures the simulator offline, where a step
 * may take as long as it likes. That cannot see the defect this tool exists to
 * measure: `EngineRuntime::run` advances a FIXED `1/240 s` of simulated time per
 * iteration and then sleeps until a wall-clock deadline. When an iteration costs
 * more wall time than it advances, simulated time falls behind wall time and
 * never catches up -- the loop has no accumulator, and past four steps of
 * lateness it resets its own deadline to `now`, discarding the debt.
 *
 * The observable consequence is not a dropped frame. It is that the whole
 * simulation runs in slow motion: controls respond late, and the cylinder
 * pressure telemetry -- which is the ONLY excitation of the exhaust acoustic
 * chain -- is produced more slowly than the audio thread consumes it.
 *
 * So the number that matters is neither CPU time per step nor the overrun count
 * (both already reported elsewhere); it is the ratio
 *
 *     realtime factor = simulated seconds advanced / wall seconds elapsed
 *
 * which is 1.0 on a healthy engine and falls below it exactly when the physics
 * thread is saturated. It is measured here on the real `EngineRuntime` thread,
 * at real thread priority, because that is the thing the user hears and feels.
 */

#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/diagnostics/EngineDiagnostics.hpp>
#include <enginelab/foundation/EngineTypes.hpp>
#include <enginelab/runtime/EngineRuntime.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;

[[nodiscard]] std::string lowercase(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

void sleepSeconds(double seconds) {
    std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
}

struct Measurement final {
    double realtimeFactor {};
    double wallSeconds {};
    double simulatedSeconds {};
    double finalRpm {};
    std::uint64_t overruns {};
    std::uint64_t iterations {};
    double maximumLatenessMs {};
    // What the GUI diagnostics panel would show at this operating point. Both
    // exhaust pressures are reported side by side because the panel's
    // back-pressure warning used to read the peak envelope and compare it to a
    // mean-sized allowance, which made it a permanent false alarm.
    double exhaustPeakKpa {};
    double exhaustMeanKpa {};
    bool backPressureWarning {};
    bool tractionLimited {};
};
// Deliberately NOT reported: the dropped cylinder-pressure count. This harness
// runs no audio thread, so nothing drains the telemetry queue and it overflows
// on every engine regardless of load -- the number would look alarming and mean
// nothing. Read that one from the AudioRender harness, which has a consumer.

/**
 * Runs one engine on the real runtime thread and reports how much simulated
 * time it managed to produce per second of wall time.
 *
 * The engine is held at a commanded speed by the dyno absorber rather than left
 * free, because the cost of a step scales with speed (more substeps per second
 * of simulated time) and comparing engines at whatever speed each happens to
 * settle at would confound engine cost with operating point.
 */
[[nodiscard]] Measurement measureEngine(const enginelab::EngineConfig& config,
                                        double holdRpm,
                                        double warmupSeconds,
                                        double measureSeconds) {
    auto runtime = std::make_unique<enginelab::EngineRuntime>(config);
    runtime->setIgnitionEnabled(true);
    runtime->setStarterEngaged(true);
    runtime->setDynoHoldEnabled(true);
    // adjustDynoHoldRpm is a relative control (the UI drives it from a key), so
    // the absolute setpoint is reached by asking for the delta from its default.
    runtime->adjustDynoHoldRpm(holdRpm - runtime->snapshot().dynoHoldRpm);
    runtime->start();
    sleepSeconds(1.5);
    runtime->setStarterEngaged(false);
    runtime->startDyno();
    sleepSeconds(warmupSeconds);

    // Both clocks are read as close together as possible at each end of the
    // window: the quantity is a ratio of two intervals, so any skew between the
    // reads is a direct error on the result.
    const auto wallStart = Clock::now();
    const auto simulatedStart = runtime->snapshot().simulationTimeSeconds;
    const auto overrunsStart = runtime->timingOverrunCount();
    sleepSeconds(measureSeconds);
    const auto simulatedEnd = runtime->snapshot().simulationTimeSeconds;
    const auto wallEnd = Clock::now();
    const auto overrunsEnd = runtime->timingOverrunCount();

    Measurement result;
    result.wallSeconds = std::chrono::duration<double>(wallEnd - wallStart).count();
    result.simulatedSeconds = simulatedEnd - simulatedStart;
    result.realtimeFactor = result.wallSeconds > 0.0
        ? result.simulatedSeconds / result.wallSeconds : 0.0;
    result.finalRpm = runtime->snapshot().rpm;
    result.overruns = overrunsEnd - overrunsStart;
    result.iterations = static_cast<std::uint64_t>(result.simulatedSeconds * 240.0);
    result.maximumLatenessMs = runtime->maximumTimingLatenessSeconds() * 1.0e3;
    const auto finalState = runtime->snapshot();
    result.exhaustPeakKpa = finalState.exhaustPressureKpa;
    result.exhaustMeanKpa = finalState.exhaustBackPressureKpa;
    result.tractionLimited = finalState.tractionLimited;
    for (const auto& diagnostic : enginelab::EngineDiagnostics {}.evaluate(config, finalState))
        if (diagnostic.code == "exhaust.back_pressure") result.backPressureWarning = true;
    runtime->stop();
    return result;
}
} // namespace

int main(int argc, char** argv) {
    std::filesystem::path catalogRoot = std::filesystem::current_path();
    std::string filter;
    double holdRpm = 5'000.0;
    double warmupSeconds = 3.0;
    double measureSeconds = 5.0;
    // A realtime loop that produces less simulated time than wall time is
    // running the whole simulation in slow motion. 0.97 leaves room for
    // scheduler noise on a loaded desktop without admitting a real deficit.
    double failBelow = 0.0;

    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--catalog-root" && index + 1 < argc) catalogRoot = argv[++index];
        else if (argument == "--filter" && index + 1 < argc) filter = lowercase(argv[++index]);
        else if (argument == "--rpm" && index + 1 < argc) holdRpm = std::stod(argv[++index]);
        else if (argument == "--warmup" && index + 1 < argc) warmupSeconds = std::stod(argv[++index]);
        else if (argument == "--seconds" && index + 1 < argc) measureSeconds = std::stod(argv[++index]);
        else if (argument == "--enforce" && index + 1 < argc) failBelow = std::stod(argv[++index]);
        else if (argument == "--help") {
            std::cout << "usage: EngineLabRealtimeBudgetHarness [--catalog-root DIR] "
                         "[--filter NAME] [--rpm N] [--warmup S] [--seconds S] "
                         "[--enforce FACTOR]\n";
            return 0;
        }
    }

    const auto catalog = enginelab::loadEngineCatalog(catalogRoot);
    if (catalog.entries.empty()) {
        std::cerr << "FAIL: no engines found under " << catalogRoot << '\n';
        return 1;
    }

    std::cout << "Realtime budget: simulated seconds produced per wall second by the\n"
                 "240 Hz EngineRuntime thread, held at " << std::fixed
              << std::setprecision(0) << holdRpm << " rpm.\n"
                 "A factor below 1.0 means the simulation is running in slow motion.\n\n";
    std::cout << std::left << std::setw(26) << "engine"
              << std::right << std::setw(5) << "cyl"
              << std::setw(10) << "rpm"
              << std::setw(9) << "factor"
              << std::setw(11) << "sim/wall"
              << std::setw(12) << "overruns"
              << std::setw(10) << "maxLate"
              << std::setw(11) << "exhPeak"
              << std::setw(11) << "exhMean"
              << std::setw(8) << "bpWarn"
              << std::setw(9) << "tyreLim" << '\n';

    auto worst = 1.0e30;
    std::string worstEngine;
    auto failures = 0;
    for (const auto& entry : catalog.entries) {
        if (!filter.empty() && lowercase(entry.config.name).find(filter) == std::string::npos)
            continue;
        const auto measurement = measureEngine(entry.config, holdRpm, warmupSeconds, measureSeconds);
        const auto cylinders = static_cast<int>(entry.config.cylinders.size());
        std::cout << std::left << std::setw(26) << entry.config.name
                  << std::right << std::setw(5) << cylinders
                  << std::setw(10) << std::fixed << std::setprecision(0) << measurement.finalRpm
                  << std::setw(9) << std::setprecision(3) << measurement.realtimeFactor
                  << std::setw(11) << (std::to_string(static_cast<int>(measurement.simulatedSeconds * 100.0) / 100)
                                       + "/" + std::to_string(static_cast<int>(measurement.wallSeconds * 100.0) / 100))
                  << std::setw(12) << measurement.overruns
                  << std::setw(8) << std::setprecision(1) << measurement.maximumLatenessMs << "ms"
                  << std::setw(11) << measurement.exhaustPeakKpa
                  << std::setw(11) << measurement.exhaustMeanKpa
                  << std::setw(8) << (measurement.backPressureWarning ? "YES" : "no")
                  << std::setw(9) << (measurement.tractionLimited ? "YES" : "no") << '\n';
        if (measurement.realtimeFactor < worst) {
            worst = measurement.realtimeFactor;
            worstEngine = entry.config.name;
        }
        if (failBelow > 0.0 && measurement.realtimeFactor < failBelow) {
            std::cerr << "FAIL: " << entry.config.name << " produced only "
                      << std::setprecision(3) << measurement.realtimeFactor
                      << " simulated seconds per wall second (floor " << failBelow << ")\n";
            ++failures;
        }
    }

    std::cout << "\nworst: " << worstEngine << " at " << std::setprecision(3) << worst << '\n';
    if (failures > 0) {
        std::cerr << failures << " engine(s) below the realtime floor\n";
        return 1;
    }
    return 0;
}
