/**
 * User-facing dynamometer regression harness.
 *
 * The historical runtime test starts the dyno from rest. This instrument
 * reproduces the product failure that test could not see: press D while the
 * engine is already several thousand rpm above the first sweep point. A pass
 * requires a visible preparation phase, no stall, and three physical points
 * beginning at the configured low-speed end of the curve.
 */

#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/foundation/EngineTypes.hpp>
#include <enginelab/runtime/EngineRuntime.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <thread>

namespace {
using Clock = std::chrono::steady_clock;

[[nodiscard]] double sweepCeilingRpm(
    const enginelab::EngineConfig& config) noexcept {
    return 0.95 * std::min(
        config.redlineRpm, config.ignition.revLimitRpm);
}

[[nodiscard]] double sweepEntryRpm(
    const enginelab::EngineConfig& config) noexcept {
    const auto lowCylinderEntry = config.cylinders.size() <= 2U
        ? 1'800.0 : 0.0;
    return std::min(sweepCeilingRpm(config),
        std::max({ 1'000.0, config.idleRpm + 400.0,
                   lowCylinderEntry }));
}

struct Result final {
    std::string engine;
    bool reachedHighStart { false };
    bool observedPreparation { false };
    bool stalled { false };
    double requestedHighStartRpm {};
    double actualStartRpm {};
    double minimumDynoRpm { std::numeric_limits<double>::max() };
    double firstPointRpm {};
    double lastPointRpm {};
    double maximumProgress {};
    std::size_t pointCount {};
    std::size_t totalPointCount {};
    std::size_t invalidPointCount {};
    std::uint32_t invalidQualityReasons {};
    double lastRecordedRpm {};
    bool exactRampGrid { false };
    enginelab::DynoRunStatus runStatus { enginelab::DynoRunStatus::idle };
    enginelab::DynoStopReason stopReason { enginelab::DynoStopReason::none };
    enginelab::DynoSessionConfig sessionConfig {};
    std::uint32_t recoveryCount {};
    double finalRpm {};
    double finalTargetRpm {};
    double finalContactFraction {};
    double finalWindowMeanRpm {};
    double finalWindowDurationSeconds {};
    std::uint32_t finalWindowCycleCount {};
    std::uint32_t finalQualityReasons {};
    bool finalPreparing { false };
    std::string failure;
};

[[nodiscard]] Result runOne(
    const enginelab::EngineConfig& config, bool completeSweep) {
    Result result;
    result.engine = config.name;
    const auto entryRpm = sweepEntryRpm(config);
    const auto ceilingRpm = sweepCeilingRpm(config);
    result.requestedHighStartRpm = std::min(
        ceilingRpm - 300.0,
        std::max(entryRpm + 900.0, entryRpm * 1.75));

    enginelab::EngineRuntime runtime(config);
    runtime.setRealtimeThrottleEnabled(false);
    runtime.setRealtimeLoadProtectionEnabled(false);
    runtime.setDynoMaximumDurationSeconds(60.0);
    // This is the product/user bench regression, not the stepped catalogue
    // calibration instrument. Make the mode explicit so a default can never
    // silently turn the test back into a 250 rpm ladder.
    runtime.setDynoRampEnabled(true);
    runtime.setDynoRampRpmPerSecond(500.0);
    runtime.setIgnitionEnabled(true);
    runtime.setStarterEngaged(true);
    runtime.setThrottle(1.0);
    runtime.start();

    auto state = runtime.snapshot();
    const auto runUpStartSimulationTime = state.simulationTimeSeconds;
    const auto wallDeadline = Clock::now() + std::chrono::seconds(90);
    while (state.simulationTimeSeconds - runUpStartSimulationTime < 14.0
           && Clock::now() < wallDeadline) {
        if (state.rpm >= std::max(650.0, config.idleRpm * 0.82))
            runtime.setStarterEngaged(false);
        if (state.rpm >= result.requestedHighStartRpm) {
            result.reachedHighStart = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        state = runtime.snapshot();
    }

    result.actualStartRpm = state.rpm;
    if (!result.reachedHighStart) {
        result.failure = "engine did not reach the high-rpm start condition";
        runtime.stop();
        return result;
    }

    runtime.setStarterEngaged(false);
    runtime.startDyno();
    const auto dynoStartSimulationTime = state.simulationTimeSeconds;
    const auto minimumRunningRpm =
        std::max(300.0, config.idleRpm * 0.40);
    auto run = runtime.currentDynoRun();
    const auto validPointCount = [](const enginelab::DynoRun& candidate) {
        return static_cast<std::size_t>(std::count_if(
            candidate.points.begin(), candidate.points.end(),
            [](const enginelab::DynoPoint& point) {
                return point.valid;
            }));
    };
    while (state.simulationTimeSeconds - dynoStartSimulationTime < 80.0
           && Clock::now() < wallDeadline
           && (completeSweep ? runtime.dynoRunning()
                             : validPointCount(run) < 3)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        state = runtime.snapshot();
        run = runtime.currentDynoRun();
        if (!state.dynoActive)
            continue;
        result.observedPreparation = result.observedPreparation
            || state.dynoPreparing;
        result.minimumDynoRpm = std::min(
            result.minimumDynoRpm, state.rpm);
        result.recoveryCount = std::max(
            result.recoveryCount, state.dynoRecoveryCount);
        result.maximumProgress = std::max(
            result.maximumProgress, state.dynoProgress);
        if (state.rpm < minimumRunningRpm)
            result.stalled = true;
    }

    result.finalRpm = state.rpm;
    result.finalTargetRpm = state.dynoTargetRpm;
    result.finalContactFraction = state.dynoBrakeContactFraction;
    result.finalWindowMeanRpm = state.dynoWindowMeanRpm;
    result.finalWindowDurationSeconds = state.dynoWindowDurationSeconds;
    result.finalWindowCycleCount = state.dynoAcceptedCycleCount;
    result.finalQualityReasons = state.dynoQualityReasons;
    result.finalPreparing = state.dynoPreparing;

    if (completeSweep && !runtime.dynoRunning()) {
        const auto history = runtime.dynoHistory();
        if (!history.empty())
            run = history.back();
    }
    result.pointCount = validPointCount(run);
    result.runStatus = run.status;
    result.stopReason = run.stopReason;
    result.sessionConfig = run.sessionConfig;
    result.totalPointCount = run.points.size();
    for (const auto& point : run.points) {
        if (!point.valid) {
            ++result.invalidPointCount;
            result.invalidQualityReasons |= point.qualityReasons;
        }
    }
    if (!run.points.empty()) result.lastRecordedRpm = run.points.back().rpm;
    result.exactRampGrid = !run.points.empty()
        && std::abs(run.points.front().rpm - entryRpm) < 1.0e-6;
    for (std::size_t index = 1;
         index < run.points.size() && result.exactRampGrid; ++index) {
        result.exactRampGrid = std::abs(
            run.points[index].rpm - run.points[index - 1].rpm - 50.0)
            < 1.0e-6;
    }
    const auto firstValid = std::find_if(
        run.points.begin(), run.points.end(),
        [](const enginelab::DynoPoint& point) { return point.valid; });
    const auto lastValid = std::find_if(
        run.points.rbegin(), run.points.rend(),
        [](const enginelab::DynoPoint& point) { return point.valid; });
    if (firstValid != run.points.end()) result.firstPointRpm = firstValid->rpm;
    if (lastValid != run.points.rend()) result.lastPointRpm = lastValid->rpm;
    runtime.stopDyno();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    runtime.stop();

    if (!result.observedPreparation)
        result.failure = "progressive preparation phase was not observed";
    else if (result.stalled)
        result.failure = "engine crossed the running threshold during preparation";
    else if (result.pointCount < 3)
        result.failure = "dyno did not acquire three points; rpm="
            + std::to_string(result.finalRpm)
            + " target=" + std::to_string(result.finalTargetRpm)
            + " contact=" + std::to_string(result.finalContactFraction)
            + " window=" + std::to_string(result.finalWindowMeanRpm)
            + "/" + std::to_string(result.finalWindowDurationSeconds)
            + "s/" + std::to_string(result.finalWindowCycleCount)
            + " gate=" + std::to_string(result.finalQualityReasons)
            + " preparing=" + (result.finalPreparing ? "yes" : "no");
    else if (std::abs(result.firstPointRpm - entryRpm) > 100.0)
        result.failure = "first point was not acquired at the low-speed sweep entry";
    else if (completeSweep
             && (result.maximumProgress < 0.99
                 || result.lastPointRpm < ceilingRpm - 100.0))
        result.failure = "full sweep stopped before the high-speed endpoint; valid="
            + std::to_string(result.pointCount) + "/"
            + std::to_string(result.totalPointCount)
            + " invalidMask=" + std::to_string(result.invalidQualityReasons)
            + " lastRecorded=" + std::to_string(result.lastRecordedRpm)
            + " finalTarget=" + std::to_string(result.finalTargetRpm)
            + " finalGate=" + std::to_string(result.finalQualityReasons);
    else if (completeSweep
             && (result.invalidPointCount != 0
                 || !result.exactRampGrid))
        result.failure = "full sweep contained invalid or non-grid bins; valid="
            + std::to_string(result.pointCount) + "/"
            + std::to_string(result.totalPointCount)
            + " invalidMask=" + std::to_string(result.invalidQualityReasons)
            + " exactGrid=" + (result.exactRampGrid ? "yes" : "no");
    else if (completeSweep
             && (result.runStatus != enginelab::DynoRunStatus::completed
                 || result.stopReason
                    != enginelab::DynoStopReason::sweepCeilingReached
                 || result.sessionConfig.mode
                    != enginelab::DynoMode::continuousRamp
                 || run.calibrationRevision == 0
                 || std::abs(result.sessionConfig.rampRateRpmPerSecond
                    - 500.0) > 1.0e-9
                 || std::abs(result.sessionConfig.binWidthRpm - 50.0)
                    > 1.0e-9))
        result.failure = "completed sweep lost its terminal status or accepted protocol";
    return result;
}
}

int main(int argc, char** argv) {
    std::filesystem::path catalogRoot = ENGINELAB_CATALOG_ROOT;
    std::string filter;
    bool completeSweep = false;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--catalog-root" && index + 1 < argc)
            catalogRoot = argv[++index];
        else if (argument == "--filter" && index + 1 < argc)
            filter = argv[++index];
        else if (argument == "--complete")
            completeSweep = true;
        else if (argument == "--help") {
            std::cout << "Usage: EngineLabUserDynoHarness "
                         "[--filter NAME] [--complete] "
                         "[--catalog-root PATH]\n";
            return 0;
        } else {
            std::cerr << "Unknown argument: " << argument << '\n';
            return 2;
        }
    }

    const auto catalog = enginelab::loadEngineCatalog(catalogRoot);
    if (!catalog.errors.empty()) {
        for (const auto& error : catalog.errors)
            std::cerr << "catalog: " << error << '\n';
        return 2;
    }

    std::cout << std::left << std::setw(31) << "engine"
              << std::right << std::setw(9) << "start"
              << std::setw(9) << "minimum"
              << std::setw(9) << "first"
              << std::setw(9) << "last"
              << std::setw(8) << "points"
              << std::setw(8) << "recover"
              << std::setw(9) << "result" << '\n';

    std::vector<const enginelab::EngineCatalogEntry*> selectedEntries;
    if (filter.empty()) {
        selectedEntries.reserve(catalog.entries.size());
        for (const auto& entry : catalog.entries)
            selectedEntries.push_back(&entry);
    } else {
        const auto selected = enginelab::selectSingleEngineCatalogEntry(
            catalog.entries, filter);
        if (!selected) {
            std::cerr << (selected.status
                    == enginelab::EngineCatalogSelectionStatus::ambiguous
                    ? "FAIL: ambiguous engine selector; use an exact catalogue key:\n"
                    : "FAIL: no catalog engine matched the selector\n");
            for (const auto* match : selected.matches)
                std::cerr << "  " << match->config.audioVoicingKey
                          << "  " << match->config.name << '\n';
            return 2;
        }
        selectedEntries.push_back(selected.entry);
    }

    std::size_t matched = 0;
    std::size_t passed = 0;
    for (const auto* entry : selectedEntries) {
        ++matched;
        const auto result = runOne(entry->config, completeSweep);
        const auto pass = result.failure.empty();
        if (pass) ++passed;
        std::cout << std::left << std::setw(31) << result.engine
                  << std::right << std::fixed << std::setprecision(0)
                  << std::setw(9) << result.actualStartRpm
                  << std::setw(9) << result.minimumDynoRpm
                  << std::setw(9) << result.firstPointRpm
                  << std::setw(9) << result.lastPointRpm
                  << std::setw(8) << result.pointCount
                  << std::setw(8) << result.recoveryCount
                  << std::setw(9) << (pass ? "PASS" : "FAIL")
                  << '\n';
        if (!pass)
            std::cerr << "  " << result.failure << '\n';
    }

    std::cout << "summary: " << passed << '/' << matched << " passed\n";
    return passed == matched ? 0 : 1;
}
