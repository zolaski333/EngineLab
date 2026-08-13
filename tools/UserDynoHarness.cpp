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
    std::uint32_t recoveryCount {};
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
    while (state.simulationTimeSeconds - dynoStartSimulationTime < 80.0
           && Clock::now() < wallDeadline
           && (completeSweep ? runtime.dynoRunning()
                             : run.points.size() < 3)) {
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

    if (completeSweep && !runtime.dynoRunning()) {
        const auto history = runtime.dynoHistory();
        if (!history.empty())
            run = history.back();
    }
    result.pointCount = run.points.size();
    if (!run.points.empty())
        result.firstPointRpm = run.points.front().rpm;
    if (!run.points.empty())
        result.lastPointRpm = run.points.back().rpm;
    runtime.stopDyno();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    runtime.stop();

    if (!result.observedPreparation)
        result.failure = "progressive preparation phase was not observed";
    else if (result.stalled)
        result.failure = "engine crossed the running threshold during preparation";
    else if (result.pointCount < 3)
        result.failure = "dyno did not acquire three points";
    else if (std::abs(result.firstPointRpm - entryRpm) > 100.0)
        result.failure = "first point was not acquired at the low-speed sweep entry";
    else if (completeSweep
             && (result.maximumProgress < 0.99
                 || result.lastPointRpm < ceilingRpm - 100.0))
        result.failure = "full sweep stopped before the high-speed endpoint";
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
