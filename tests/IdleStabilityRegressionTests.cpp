// Idle-stability regression.
//
// The complaint this guards against: "no engine holds idle correctly -- it sits
// permanently on the edge of stalling." That is a governor/authority property,
// not a per-engine tuning value, so this drives EVERY catalogue engine through
// the same crank -> release -> free-idle sequence and asserts the physical
// contract of an idle: once the starter is released and the throttle is shut,
// the engine must converge to its *configured* idle speed and hold there --
// neither stalling nor hunting.
//
// The reference speed is the engine's own configured idle_rpm (from its YAML),
// never a number read back from the simulator, so tightening this test can never
// re-calibrate it onto whatever the simulator currently produces.

#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace {
constexpr double stepSeconds = 1.0 / 240.0;

struct IdleResult final {
    std::string name;
    double idleRpm { 0.0 };
    bool caught { false };        // reached a self-sustaining speed during crank
    bool steadyStalled { false }; // fell below the stall floor during free idle
    bool blipStalled { false };   // stalled while recovering from a throttle blip
    double settledMeanRpm { 0.0 };// mean over the final steady window
    double minRpm { 0.0 };        // minimum over the final steady window
    double maxRpm { 0.0 };        // maximum over the final steady window
    double stdRpm { 0.0 };        // rpm std-dev over the final steady window (hunt)
    double driftRpm { 0.0 };      // second-half mean minus first-half mean
};

IdleResult measureIdle(const enginelab::EngineConfig& config, double blipThrottle,
                       std::optional<double> exhaustTargetCellLengthM = {}) {
    IdleResult result;
    result.name = config.name;
    result.idleRpm = config.idleRpm;

    enginelab::SimpleEcuModel ecu;
    enginelab::SimplifiedGasolinePhysics physics;
    enginelab::FourStrokeEventGenerator events;
    auto exhaust = enginelab::ExhaustGraph::makeForEngine(config);
    enginelab::EngineSimulatorOptions simulatorOptions;
    simulatorOptions.exhaustTargetCellLengthM = exhaustTargetCellLengthM;
    enginelab::EngineSimulator simulator(
        config, ecu, physics, events, exhaust, simulatorOptions);

    const auto stallFloorRpm = std::max(250.0, config.idleRpm * 0.45);

    // Phase 1: start and stabilise. A real driver holds the key until the engine
    // is clearly running, so the starter re-catches the post-start flare/crash
    // whenever the speed falls back under the engine below ~idle. Runs 6 s.
    constexpr double startSeconds = 6.0;
    for (int step = 0; step < static_cast<int>(startSeconds / stepSeconds); ++step) {
        enginelab::EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = simulator.state().rpm < config.idleRpm * 0.85;
        controls.throttle = 0.0;
        (void)simulator.step(stepSeconds, controls);
        if (simulator.state().rpm > config.idleRpm * 0.60) result.caught = true;
    }

    // Phase 2: free idle -- starter released, throttle shut, no load.
    //
    // 16 s, not the 8 s this originally used. The ECU's post-start air schedule
    // decays exponentially with a 2.5-6 s time constant and is only dropped
    // below 1e-4, so its tail runs past 20 s; at 8 s the "steady" window was
    // still measuring the start transient on the largest-cylinder engines, and
    // the drift check below fails outright at 8 s (Big Twin -66.8 rpm). This is
    // a longer settle time paired with a STRICTER settled-ness assertion, not a
    // relaxation: at 16 s every catalogue engine converges onto its configured
    // target to within a few rpm.
    constexpr double idleSeconds = 16.0;
    constexpr double steadyWindowSeconds = 4.0; // measure only the settled tail
    std::vector<double> steadyRpm;
    const auto steadyStartStep =
        static_cast<int>((idleSeconds - steadyWindowSeconds) / stepSeconds);
    const auto idleSteps = static_cast<int>(idleSeconds / stepSeconds);
    for (int step = 0; step < idleSteps; ++step) {
        enginelab::EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = false;
        controls.throttle = 0.0;
        controls.load = 0.0;
        (void)simulator.step(stepSeconds, controls);
        const auto rpm = simulator.state().rpm;
        if (rpm < stallFloorRpm) result.steadyStalled = true;
        if (step >= steadyStartStep) steadyRpm.push_back(rpm);
    }

    // Phase 3: throttle blip and return to idle -- the user's real scenario
    // ("blip then return to idle risks stalling"). 0.5 s at half throttle, then
    // closed; the engine must recover to idle without stalling.
    for (int step = 0; step < static_cast<int>(0.5 / stepSeconds); ++step) {
        enginelab::EngineControls controls;
        controls.ignitionEnabled = true;
        controls.throttle = blipThrottle;
        (void)simulator.step(stepSeconds, controls);
    }
    for (int step = 0; step < static_cast<int>(4.0 / stepSeconds); ++step) {
        enginelab::EngineControls controls;
        controls.ignitionEnabled = true;
        controls.throttle = 0.0;
        (void)simulator.step(stepSeconds, controls);
        if (simulator.state().rpm < stallFloorRpm) result.blipStalled = true;
    }

    if (!steadyRpm.empty()) {
        result.minRpm = *std::min_element(steadyRpm.begin(), steadyRpm.end());
        result.maxRpm = *std::max_element(steadyRpm.begin(), steadyRpm.end());
        double sum = 0.0;
        for (const auto rpm : steadyRpm) sum += rpm;
        result.settledMeanRpm = sum / static_cast<double>(steadyRpm.size());
        double variance = 0.0;
        for (const auto rpm : steadyRpm) {
            const auto d = rpm - result.settledMeanRpm;
            variance += d * d;
        }
        result.stdRpm = std::sqrt(variance / static_cast<double>(steadyRpm.size()));
        // Drift across the window: the mean of its second half minus the mean of
        // its first. A std-dev alone cannot tell a settled idle from one that is
        // still swinging, because a slow oscillation sampled over part of a
        // period looks quiet at some phases -- which is exactly how a real
        // integral-windup fault survived this gate while every engine was
        // ringing 130-180 rpm peak-to-peak. Drift is phase-sensitive where the
        // std-dev is not, so the pair catches both.
        const auto half = steadyRpm.size() / 2;
        if (half > 0) {
            double firstSum = 0.0;
            double secondSum = 0.0;
            for (std::size_t index = 0; index < half; ++index)
                firstSum += steadyRpm[index];
            for (std::size_t index = steadyRpm.size() - half;
                 index < steadyRpm.size(); ++index)
                secondSum += steadyRpm[index];
            result.driftRpm = (secondSum - firstSum) / static_cast<double>(half);
        }
    }
    return result;
}
} // namespace

void traceIdle(const enginelab::EngineConfig& config) {
    enginelab::SimpleEcuModel ecu;
    enginelab::SimplifiedGasolinePhysics physics;
    enginelab::FourStrokeEventGenerator events;
    auto exhaust = enginelab::ExhaustGraph::makeForEngine(config);
    enginelab::EngineSimulator simulator(config, ecu, physics, events, exhaust);
    std::printf("== TRACE %s (idle target %.0f) ==\n", config.name.c_str(), config.idleRpm);
    std::printf("    t   thr     rpm     MAP  fuel_g/s  cycTq  fricTq  AFR"
                "   iac  postSt  integ cut resume  air_mg  req_mg del_mg"
                " fDel trim phiSp  misf combEff\n");
    double t = 0.0;
    // Mirrors measureIdle's schedule exactly, and is DERIVED from the same
    // constants so it cannot drift out of step with the gate again: crank
    // `startSeconds`, free idle for `idleSeconds` (the asserted window being its
    // last `steadyWindowSeconds`), then a 0.5 s blip and a 4 s recovery. These
    // were hard-coded at 14.0/14.5/18.5 and stayed there when the idle phase was
    // lengthened from 8 s to 16 s, so the trace was blipping at 14 s off an
    // unsettled idle while the gate blipped at 22 s off a settled one -- the
    // instrument was explaining a different experiment from the one that failed.
    constexpr double traceStartSeconds = 6.0;
    constexpr double traceIdleSeconds = 16.0;
    const auto blipStart = traceStartSeconds + traceIdleSeconds;
    const auto blipEnd = blipStart + 0.5;
    const auto traceEnd = blipEnd + 4.0;
    for (int step = 0; step < static_cast<int>(traceEnd / stepSeconds); ++step, t += stepSeconds) {
        enginelab::EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged =
            t < traceStartSeconds && simulator.state().rpm < config.idleRpm * 0.85;
        controls.throttle = (t >= blipStart && t < blipEnd) ? 0.5 : 0.0;
        controls.load = 0.0;
        const auto frame = simulator.step(stepSeconds, controls);
        // Fine cadence through the catch and the blip recovery, coarse between.
        const auto fine = t < 8.0 || (t >= blipStart - 0.25 && t < blipStart + 3.0);
        const auto cadence = fine ? 0.10 : 0.25;
        if (step % std::max(1, static_cast<int>(cadence / stepSeconds)) == 0) {
            const auto& s = frame.state;
            // Combustion efficiency is per cylinder, not on EngineState; the
            // mean is what an idle diagnosis needs (a partial misfire shows as a
            // dip here before it shows in rpm).
            auto combustionSum = 0.0;
            auto requestedFuelSum = 0.0;
            auto deliveredFuelSum = 0.0;
            auto fuelDeliverySum = 0.0;
            auto fuelTrimSum = 0.0;
            auto equivalenceRatioSum = 0.0;
            for (std::size_t c = 0; c < s.cylinderStateCount; ++c) {
                combustionSum += s.cylinderStates[c].combustionEfficiency;
                requestedFuelSum += s.cylinderStates[c].requestedFuelMgPerCycle;
                deliveredFuelSum += s.cylinderStates[c].deliveredFuelMgPerCycle;
                fuelDeliverySum += s.cylinderStates[c].fuelDeliveryRatio;
                fuelTrimSum += s.cylinderStates[c].closedLoopFuelTrim;
                equivalenceRatioSum += s.cylinderStates[c].equivalenceRatioAtSpark;
            }
            const auto cylinderCount = static_cast<double>(
                std::max<std::size_t>(1, s.cylinderStateCount));
            const auto meanCombustion = combustionSum / cylinderCount;
            std::printf(" %5.2f %4.2f %7.1f %6.1f %8.4f %6.1f %7.1f %5.1f"
                        " %5.3f %6.3f %6.3f %3d %6.3f %7.1f %7.1f %6.1f"
                        " %4.2f %4.2f %5.2f %5.2f %7.3f\n",
                t, controls.throttle, s.rpm, s.manifoldPressureKpa,
                s.fuelFlowGramsPerSecond, s.cycleAveragedTorqueNm,
                s.frictionTorqueNm, s.airFuelRatio,
                ecu.idleAirOpening(), ecu.postStartAirOpening(), ecu.idleIntegral(),
                ecu.decelerationFuelCutActive() ? 1 : 0,
                ecu.decelerationFuelResume(), s.airMassMgPerCycle,
                requestedFuelSum / cylinderCount,
                deliveredFuelSum / cylinderCount,
                fuelDeliverySum / cylinderCount,
                fuelTrimSum / cylinderCount,
                equivalenceRatioSum / cylinderCount,
                s.misfireRate, meanCombustion);
        }
    }
}

int main(int argc, char** argv) {
    try {
        const auto catalog = enginelab::loadEngineCatalog(
            std::filesystem::path(ENGINELAB_CATALOG_ROOT));
        if (!catalog.errors.empty()) {
            std::cerr << "FAILED: catalogue load: " << catalog.errors.front() << '\n';
            return EXIT_FAILURE;
        }

        if (argc > 1 && std::string(argv[1]) == "--trace") {
            // `--trace [name-substring ...]`; with no substring, the two engines
            // this mode was first written for.
            std::vector<std::string> wanted;
            for (int index = 2; index < argc; ++index) wanted.emplace_back(argv[index]);
            if (wanted.empty()) wanted = { "Merlin", "LS3" };
            for (const auto& entry : catalog.entries) {
                auto config = entry.config;
                enginelab::normaliseEngineConfig(config);
                const auto matches = std::any_of(wanted.begin(), wanted.end(),
                    [&](const std::string& token) {
                        return config.name.find(token) != std::string::npos;
                    });
                if (matches) traceIdle(config);
            }
            return EXIT_SUCCESS;
        }

        auto blipThrottle = 0.5;
        auto engineFilter = std::string {};
        auto exhaustTargetCellLengthM = std::optional<double> {};
        for (auto index = 1; index < argc; ++index) {
            const auto argument = std::string { argv[index] };
            if (argument == "--filter" && index + 1 < argc)
                engineFilter = argv[++index];
            else if (argument == "--exhaust-cell-mm" && index + 1 < argc)
                exhaustTargetCellLengthM = std::stod(argv[++index]) * 0.001;
            else
                blipThrottle = std::stod(argument);
        }
        std::printf("idle stability measurement (free idle; blip=%.2f)\n", blipThrottle);
        std::cout << "  engine                              target   mean    min    max"
                     "   std  drift  caught steadyStall blipStall\n";

        // A known, separately-tracked deficiency: on a throttle blip the very
        // largest / highest-motoring-drag engines relight from the decel fuel cut
        // into a low-rpm lean-out (the fuel film cannot follow the governor's air
        // as the manifold refills near idle) and sag through idle. These engines
        // still hold a STEADY idle (asserted below); only their blip recovery is
        // affected, and correcting it needs a fuel-transport change that touches
        // every engine's combustion, so it is tracked on its own. This allowlist
        // exists so that a NEW engine regressing into a blip stall still fails the
        // suite -- it is not a blanket pass.
        // Was { "Merlin" }. Emptied deliberately once the idle governor's
        // anti-windup was corrected: the V12 now recovers from the blip without
        // stalling (measured blipStall = 0), so the exemption was masking
        // nothing and keeping it would only hide a future regression.
        const std::vector<std::string> knownBlipStallAllow {};

        std::vector<std::string> failures;
        for (const auto& entry : catalog.entries) {
            auto config = entry.config;
            enginelab::normaliseEngineConfig(config);
            if (!engineFilter.empty()
                && config.name.find(engineFilter) == std::string::npos)
                continue;
            const auto r = measureIdle(
                config, blipThrottle, exhaustTargetCellLengthM);
            std::printf("  %-34s %6.0f %6.0f %6.0f %6.0f %5.1f %6.1f    %d       %d         %d\n",
                r.name.c_str(), r.idleRpm, r.settledMeanRpm, r.minRpm, r.maxRpm,
                r.stdRpm, r.driftRpm, r.caught ? 1 : 0, r.steadyStalled ? 1 : 0,
                r.blipStalled ? 1 : 0);

            // (1) Every engine must catch during start.
            if (!r.caught) failures.push_back(r.name + ": did not start");
            // (2) Every engine must hold a STEADY free idle near its CONFIGURED
            //     idle speed -- the core "holds idle" contract. Bounds are wide
            //     multiples of the configured target (never the simulator's own
            //     output) so they cannot re-calibrate onto current behaviour.
            if (r.steadyStalled)
                failures.push_back(r.name + ": stalled at steady idle");
            if (r.settledMeanRpm < 0.85 * r.idleRpm
                || r.settledMeanRpm > 1.10 * r.idleRpm)
                failures.push_back(r.name + ": steady idle far from target ("
                    + std::to_string(static_cast<int>(r.settledMeanRpm)) + " vs "
                    + std::to_string(static_cast<int>(r.idleRpm)) + ")");
            if (r.minRpm < 0.78 * r.idleRpm)
                failures.push_back(r.name + ": steady idle dips toward stall");
            if (r.stdRpm > 0.06 * r.idleRpm)
                failures.push_back(r.name + ": steady idle hunts excessively");
            // (2b) The window must be measuring a SETTLED idle. Without this the
            //      gate cannot distinguish a stable engine from one still
            //      swinging through the window, and its verdict depends on the
            //      phase it happens to sample -- which is how an idle integral
            //      pinned at its stop passed here for eleven engines at once.
            //      The bound comes from production idle-control practice
            //      (holding idle within about +/-50 rpm of target), not from
            //      anything this simulator currently produces.
            if (std::abs(r.driftRpm) > 0.05 * r.idleRpm)
                failures.push_back(r.name + ": steady idle has not settled (drift "
                    + std::to_string(static_cast<int>(r.driftRpm)) + " rpm)");
            // (3) Returning to idle from a throttle blip must not stall, except
            //     for the documented, separately-tracked outliers above.
            const auto allowed = std::any_of(knownBlipStallAllow.begin(),
                knownBlipStallAllow.end(), [&](const std::string& token) {
                    return r.name.find(token) != std::string::npos;
                });
            if (r.blipStalled && !allowed)
                failures.push_back(r.name + ": stalled recovering from a blip");
        }

        if (!failures.empty()) {
            std::cerr << "FAILED idle stability:\n";
            for (const auto& message : failures) std::cerr << "  - " << message << '\n';
            return EXIT_FAILURE;
        }
        std::cout << "Idle stability regression tests passed\n";
    } catch (const std::exception& error) {
        std::cerr << "FAILED: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
