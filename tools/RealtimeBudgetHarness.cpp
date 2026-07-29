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
#include <limits>
#include <optional>
#include <sstream>
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
    bool valid { false };
    std::string invalidReason;
    double targetRpm {};
    double realtimeFactor {};
    double wallSeconds {};
    double simulatedSeconds {};
    double meanRpm {};
    double minimumRpm {};
    double maximumRpm {};
    std::size_t intakeWorkers {};
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
                                        double measureSeconds,
                                        bool freeRun,
                                        std::optional<std::size_t> intakeWorkers,
                                        std::optional<std::size_t> intakeMaximumCells,
                                        std::optional<std::size_t> intakeStaircaseRounds,
                                        std::optional<double> intakeCouplingSeconds,
                                        std::optional<double> intakeTargetCellLengthM,
                                        std::optional<double> exhaustCouplingSeconds,
                                        std::optional<bool> intakeFirstOrderTimeIntegration,
                                        std::optional<double> intakeWallHeatUpdateSeconds,
                                        bool useWellMixedExhaustJunctions) {
    enginelab::EngineSimulatorOptions simulatorOptions;
    simulatorOptions.intakeWorkerCount = intakeWorkers;
    simulatorOptions.intakeMaximumCellCount = intakeMaximumCells;
    simulatorOptions.intakeStaircaseRounds = intakeStaircaseRounds;
    simulatorOptions.intakeCouplingIntervalSeconds = intakeCouplingSeconds;
    simulatorOptions.intakeTargetCellLengthM = intakeTargetCellLengthM;
    simulatorOptions.maximumLowSpeedExhaustCouplingSeconds =
        exhaustCouplingSeconds;
    simulatorOptions.intakeFirstOrderTimeIntegration =
        intakeFirstOrderTimeIntegration;
    simulatorOptions.intakeWallHeatUpdateIntervalSeconds =
        intakeWallHeatUpdateSeconds;
    simulatorOptions.evolveExhaustJunctionAxialMomentum =
        !useWellMixedExhaustJunctions;
    auto runtime = std::make_unique<enginelab::EngineRuntime>(
        config, nullptr, simulatorOptions);
    Measurement result;
    result.intakeWorkers = runtime->intakeWorkerCount();
    // Throttled, the loop sleeps to its wall deadline, so the factor saturates
    // at 1.0 and an engine at 3x reads the same as one exactly breaking even.
    // Free-running, the same ratio is the capacity headroom.
    runtime->setRealtimeThrottleEnabled(!freeRun);
    runtime->setDynoMaximumDurationSeconds(120.0);
    runtime->setIgnitionEnabled(true);
    runtime->setStarterEngaged(true);
    runtime->setDynoHoldEnabled(true);
    // This must be an absolute setter. Before it existed the harness subtracted
    // the not-yet-published snapshot value (zero) from the requested speed and
    // added that delta to the runtime's internal 2,500 rpm default. Most points
    // therefore hit the limiter while the heading claimed a common setpoint.
    runtime->setDynoHoldRpm(holdRpm);
    result.targetRpm = runtime->dynoHoldRpm();
    runtime->start();
    runtime->startDyno();

    // Do not begin a sample merely because a wall-clock warm-up expired. The
    // catalogue spans twins to a V12, and in free-run mode each engine advances
    // simulation time at a different rate. Require the dyno's cycle-filtered
    // speed to remain within 2% (at least 60 rpm) for 1.5 simulated seconds.
    // The complete measurement window is checked again below, so a ramp merely
    // crossing the band cannot become a valid point. Avoid gating on an
    // instantaneous acceleration estimate here: polling aliases firing ripple
    // on the diesel even when its cycle-mean speed is stationary. A point that
    // never reaches this gate is data we
    // refuse to compare.
    const auto holdWallDeadline = Clock::now() + std::chrono::seconds(90);
    auto holdState = runtime->snapshot();
    const auto holdStartSimulationTime = holdState.simulationTimeSeconds;
    auto previousSimulationTime = holdState.simulationTimeSeconds;
    auto filteredRpm = holdState.rpm;
    auto stableSimulationSeconds = 0.0;
    auto holdReached = false;
    while (Clock::now() < holdWallDeadline) {
        holdState = runtime->snapshot();
        const auto dt = std::max(
            0.0, holdState.simulationTimeSeconds - previousSimulationTime);
        previousSimulationTime = holdState.simulationTimeSeconds;
        if (dt > 0.0) {
            filteredRpm += (holdState.rpm - filteredRpm)
                * (1.0 - std::exp(-dt * 3.0));
        }
        const auto toleranceRpm = std::max(60.0, result.targetRpm * 0.02);
        if (dt > 0.0 && std::abs(filteredRpm - result.targetRpm) <= toleranceRpm) {
            stableSimulationSeconds += dt;
            if (stableSimulationSeconds >= 1.5) {
                holdReached = true;
                break;
            }
        } else {
            stableSimulationSeconds = 0.0;
        }
        if (!runtime->dynoRunning()
            && holdState.simulationTimeSeconds > holdStartSimulationTime + 0.25) {
            result.invalidReason = "dyno stopped before hold";
            break;
        }
        if (holdState.simulationTimeSeconds - holdStartSimulationTime >= 60.0) {
            result.invalidReason = "hold not stable within 60 sim s";
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    if (!holdReached) {
        if (result.invalidReason.empty()) result.invalidReason = "hold wall timeout";
        result.meanRpm = holdState.rpm;
        result.minimumRpm = holdState.rpm;
        result.maximumRpm = holdState.rpm;
        runtime->stop();
        return result;
    }

    // Warm up by SIMULATED time, not wall time. This gives every engine the
    // same number of thermodynamic seconds after its speed has settled.
    const auto warmupStart = runtime->snapshot().simulationTimeSeconds;
    const auto warmupWallDeadline = Clock::now()
        + std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(std::max(30.0, warmupSeconds * 20.0)));
    while (runtime->snapshot().simulationTimeSeconds - warmupStart < warmupSeconds) {
        if (!runtime->dynoRunning()) {
            result.invalidReason = "dyno stopped during warmup";
            result.meanRpm = runtime->snapshot().rpm;
            result.minimumRpm = result.meanRpm;
            result.maximumRpm = result.meanRpm;
            runtime->stop();
            return result;
        }
        if (Clock::now() >= warmupWallDeadline) {
            result.invalidReason = "warmup wall timeout";
            result.meanRpm = runtime->snapshot().rpm;
            result.minimumRpm = result.meanRpm;
            result.maximumRpm = result.meanRpm;
            runtime->stop();
            return result;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    // Both clocks are read as close together as possible at each end of the
    // window: the quantity is a ratio of two intervals, so any skew between the
    // reads is a direct error on the result.
    const auto wallStart = Clock::now();
    const auto simulatedStart = runtime->snapshot().simulationTimeSeconds;
    const auto overrunsStart = runtime->timingOverrunCount();
    const auto measurementDeadline = wallStart
        + std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(measureSeconds));
    auto rpmSum = 0.0;
    auto rpmSamples = std::uint64_t { 0 };
    result.minimumRpm = std::numeric_limits<double>::infinity();
    result.maximumRpm = 0.0;
    while (Clock::now() < measurementDeadline) {
        const auto state = runtime->snapshot();
        rpmSum += state.rpm;
        ++rpmSamples;
        result.minimumRpm = std::min(result.minimumRpm, state.rpm);
        result.maximumRpm = std::max(result.maximumRpm, state.rpm);
        if (!runtime->dynoRunning()) {
            result.invalidReason = "dyno stopped during measurement";
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const auto simulatedEnd = runtime->snapshot().simulationTimeSeconds;
    const auto wallEnd = Clock::now();
    const auto overrunsEnd = runtime->timingOverrunCount();

    result.wallSeconds = std::chrono::duration<double>(wallEnd - wallStart).count();
    result.simulatedSeconds = simulatedEnd - simulatedStart;
    result.realtimeFactor = result.wallSeconds > 0.0
        ? result.simulatedSeconds / result.wallSeconds : 0.0;
    result.meanRpm = rpmSamples > 0 ? rpmSum / static_cast<double>(rpmSamples)
                                   : runtime->snapshot().rpm;
    if (!std::isfinite(result.minimumRpm)) result.minimumRpm = result.meanRpm;
    result.overruns = overrunsEnd - overrunsStart;
    result.iterations = static_cast<std::uint64_t>(result.simulatedSeconds * 240.0);
    result.maximumLatenessMs = runtime->maximumTimingLatenessSeconds() * 1.0e3;
    const auto finalState = runtime->snapshot();
    result.exhaustPeakKpa = finalState.exhaustPressureKpa;
    result.exhaustMeanKpa = finalState.exhaustBackPressureKpa;
    result.tractionLimited = finalState.tractionLimited;
    for (const auto& diagnostic : enginelab::EngineDiagnostics {}.evaluate(config, finalState))
        if (diagnostic.code == "exhaust.back_pressure") result.backPressureWarning = true;
    const auto meanToleranceRpm = std::max(60.0, result.targetRpm * 0.02);
    if (result.invalidReason.empty()
        && std::abs(result.meanRpm - result.targetRpm) > meanToleranceRpm) {
        std::ostringstream reason;
        reason << "mean rpm outside 2% (" << std::fixed << std::setprecision(0)
               << result.meanRpm << " vs " << result.targetRpm << ')';
        result.invalidReason = reason.str();
    }
    result.valid = result.invalidReason.empty();
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
    bool freeRun = false;
    std::optional<double> relativeRpm;
    std::optional<std::size_t> intakeWorkers;
    std::optional<std::size_t> intakeMaximumCells;
    std::optional<std::size_t> intakeStaircaseRounds;
    std::optional<double> intakeCouplingSeconds;
    std::optional<double> intakeTargetCellLengthM;
    std::optional<double> exhaustCouplingSeconds;
    std::optional<bool> intakeFirstOrderTimeIntegration;
    std::optional<double> intakeWallHeatUpdateSeconds;
    bool useWellMixedExhaustJunctions = false;

    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--catalog-root" && index + 1 < argc) catalogRoot = argv[++index];
        else if (argument == "--filter" && index + 1 < argc) filter = lowercase(argv[++index]);
        else if (argument == "--rpm" && index + 1 < argc) holdRpm = std::stod(argv[++index]);
        else if (argument == "--relative-rpm" && index + 1 < argc)
            relativeRpm = std::stod(argv[++index]);
        else if (argument == "--intake-workers" && index + 1 < argc)
            intakeWorkers = static_cast<std::size_t>(std::stoull(argv[++index]));
        else if (argument == "--intake-max-cells" && index + 1 < argc)
            intakeMaximumCells = static_cast<std::size_t>(std::stoull(argv[++index]));
        else if (argument == "--intake-staircase-rounds" && index + 1 < argc)
            intakeStaircaseRounds = static_cast<std::size_t>(std::stoull(argv[++index]));
        else if (argument == "--intake-coupling-us" && index + 1 < argc)
            intakeCouplingSeconds = std::stod(argv[++index]) * 1.0e-6;
        else if (argument == "--intake-cell-mm" && index + 1 < argc)
            intakeTargetCellLengthM = std::stod(argv[++index]) * 1.0e-3;
        else if (argument == "--exhaust-coupling-us" && index + 1 < argc)
            exhaustCouplingSeconds = std::stod(argv[++index]) * 1.0e-6;
        else if (argument == "--intake-euler")
            intakeFirstOrderTimeIntegration = true;
        else if (argument == "--intake-rk2")
            intakeFirstOrderTimeIntegration = false;
        else if (argument == "--intake-wall-us" && index + 1 < argc)
            intakeWallHeatUpdateSeconds = std::stod(argv[++index]) * 1.0e-6;
        else if (argument == "--warmup" && index + 1 < argc) warmupSeconds = std::stod(argv[++index]);
        else if (argument == "--seconds" && index + 1 < argc) measureSeconds = std::stod(argv[++index]);
        else if (argument == "--enforce" && index + 1 < argc) failBelow = std::stod(argv[++index]);
        else if (argument == "--free-run") freeRun = true;
        else if (argument == "--well-mixed-junctions")
            useWellMixedExhaustJunctions = true;
        else if (argument == "--help") {
            std::cout << "usage: EngineLabRealtimeBudgetHarness [--catalog-root DIR] "
                         "[--filter NAME] [--rpm N] [--warmup S] [--seconds S] "
                         "[--relative-rpm FRACTION] [--intake-workers N] "
                         "[--intake-max-cells N] [--intake-staircase-rounds N] "
                         "[--intake-coupling-us N] "
                         "[--intake-cell-mm N] "
                         "[--exhaust-coupling-us N] "
                         "[--intake-euler|--intake-rk2] "
                         "[--intake-wall-us N] "
                         "[--well-mixed-junctions] "
                         "[--enforce FACTOR] [--free-run]\n"
                         "  --free-run  remove the loop's wall-clock sleep, so the factor\n"
                         "              reads capacity instead of saturating at 1.0.\n"
                         "  --relative-rpm  hold each engine at this fraction of redline,\n"
                         "                  capped at 95% to stay below the limiter.\n"
                         "  --intake-workers  override background intake workers; zero is\n"
                         "                    the serial null control.\n"
                         "  --well-mixed-junctions  select the legacy zero-momentum exhaust\n"
                         "                          collector for same-machine A/B evidence.\n";
            return 0;
        }
    }
    if (relativeRpm.has_value()
        && (!std::isfinite(*relativeRpm) || *relativeRpm <= 0.0 || *relativeRpm > 1.0)) {
        std::cerr << "FAIL: --relative-rpm must be in (0, 1]\n";
        return 2;
    }

    const auto catalog = enginelab::loadEngineCatalog(catalogRoot);
    if (catalog.entries.empty()) {
        std::cerr << "FAIL: no engines found under " << catalogRoot << '\n';
        return 1;
    }

    std::cout << "Realtime budget: simulated seconds produced per wall second by the\n"
                 "240 Hz EngineRuntime thread, "
              << (relativeRpm.has_value()
                  ? "each engine held at " + std::to_string(*relativeRpm * 100.0)
                      + "% of redline.\n"
                  : "held at an absolute requested speed of "
                      + std::to_string(static_cast<int>(holdRpm)) + " rpm "
                        "(clamped to 95% of each engine's valid range).\n");
    std::cout << (freeRun
        ? "FREE-RUN: the wall-clock sleep is removed, so the factor is CAPACITY.\n"
          "1.0 is exactly break-even and leaves no margin for scheduler jitter.\n\n"
        : "A factor below 1.0 means the simulation is running in slow motion.\n"
          "It saturates at 1.0; use --free-run to see the headroom above it.\n\n");
    std::cout << std::left << std::setw(26) << "engine"
              << std::right << std::setw(5) << "cyl"
              << std::setw(9) << "target"
              << std::setw(9) << "meanRpm"
              << std::setw(6) << "wrk"
              << std::setw(8) << "held"
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
    auto invalidMeasurements = 0;
    for (const auto& entry : catalog.entries) {
        if (!filter.empty() && lowercase(entry.config.name).find(filter) == std::string::npos)
            continue;
        const auto revLimitRpm = std::min(
            entry.config.redlineRpm,
            entry.config.ignition.revLimitRpm);
        // A latched rev limiter is not a steady operating point: missing sparks
        // contaminate pressure, torque and timing cost. Every other WOT/science
        // harness already stops at 95%; apply the same ceiling here so the
        // documented absolute 7,000 rpm catalogue command cannot silently ask
        // low-redline engines to hold on the limiter.
        const auto maximumMeasurementRpm = 0.95 * revLimitRpm;
        const auto requestedRpm = std::min(maximumMeasurementRpm,
            relativeRpm.has_value()
                ? revLimitRpm * *relativeRpm : holdRpm);
        const auto measurement = measureEngine(
            entry.config, requestedRpm, warmupSeconds, measureSeconds, freeRun,
            intakeWorkers, intakeMaximumCells, intakeStaircaseRounds,
            intakeCouplingSeconds, intakeTargetCellLengthM,
            exhaustCouplingSeconds, intakeFirstOrderTimeIntegration,
            intakeWallHeatUpdateSeconds,
            useWellMixedExhaustJunctions);
        const auto cylinders = static_cast<int>(entry.config.cylinders.size());
        std::cout << std::left << std::setw(26) << entry.config.name
                  << std::right << std::setw(5) << cylinders
                  << std::setw(9) << std::fixed << std::setprecision(0) << measurement.targetRpm
                  << std::setw(9) << measurement.meanRpm
                  << std::setw(6) << measurement.intakeWorkers
                  << std::setw(8) << (measurement.valid ? "yes" : "INVALID")
                  << std::setw(9) << (measurement.valid
                      ? [&measurement] {
                            std::ostringstream value;
                            value << std::fixed << std::setprecision(3)
                                  << measurement.realtimeFactor;
                            return value.str();
                        }()
                      : "--")
                  << std::setw(11) << (std::to_string(static_cast<int>(measurement.simulatedSeconds * 100.0) / 100)
                                       + "/" + std::to_string(static_cast<int>(measurement.wallSeconds * 100.0) / 100))
                  << std::setw(12) << measurement.overruns
                  << std::setw(8) << std::setprecision(1) << measurement.maximumLatenessMs << "ms"
                  << std::setw(11) << measurement.exhaustPeakKpa
                  << std::setw(11) << measurement.exhaustMeanKpa
                  << std::setw(8) << (measurement.backPressureWarning ? "YES" : "no")
                  << std::setw(9) << (measurement.tractionLimited ? "YES" : "no") << '\n';
        if (!measurement.valid) {
            std::cerr << "INVALID: " << entry.config.name << ": "
                      << measurement.invalidReason << '\n';
            ++invalidMeasurements;
        } else if (measurement.realtimeFactor < worst) {
            worst = measurement.realtimeFactor;
            worstEngine = entry.config.name;
        }
        if (measurement.valid && failBelow > 0.0
            && measurement.realtimeFactor < failBelow) {
            std::cerr << "FAIL: " << entry.config.name << " produced only "
                      << std::setprecision(3) << measurement.realtimeFactor
                      << " simulated seconds per wall second (floor " << failBelow << ")\n";
            ++failures;
        }
    }

    if (worstEngine.empty()) std::cout << "\nworst: n/a (no valid points)\n";
    else
        std::cout << "\nworst: " << worstEngine << " at "
                  << std::setprecision(3) << worst << '\n';
    if (invalidMeasurements > 0)
        std::cerr << invalidMeasurements << " invalid measurement(s); no comparison is allowed\n";
    if (failures > 0 || invalidMeasurements > 0) {
        std::cerr << failures << " engine(s) below the realtime floor\n";
        return 1;
    }
    return 0;
}
