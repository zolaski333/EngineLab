// Shift-transient instrument for the "gear crack" complaint (#4).
//
// The report: on a clutchless upshift (the user just taps the up-arrow -- no
// clutch key) at HIGH RPM and FULL throttle, there is an audible "crack" while
// the engine speed drops to the higher gear. At part throttle there is none.
//
// That last clause is the whole diagnosis. The realtime path builds the engine
// throttle as `throttle * torqueCutMultiplier` (EngineRuntime), and during a
// shift torqueCutMultiplier dips as `1 - fraction*sin(pi*progress)`. At WOT that
// is a full-scale manifold swing 1.0 -> ~0.15 -> 1.0 inside shiftDurationSeconds;
// at part throttle it is a small swing. Meanwhile the clutch re-engages over the
// back half of the shift against a large rpm slip in the new gear. The restore
// of throttle and the clutch grab OVERLAP, so the engine is making rising torque
// while the clutch drags it down to synchronous speed -- a fight that shows up as
// a sharp step in exhaust mass flow, i.e. the "crack".
//
// This harness reproduces exactly that, deterministically and without the audio
// path, by replicating EngineRuntime's one-tick-lagged driveline<->engine
// coupling (see EngineRuntime::run around the updateDriveline / simulator_.step
// pair). It launches each engine at WOT, accelerates in gear, performs one
// clutchless upshift at a chosen rpm, and reports the sharpness of the exhaust
// mass-flow transient the audio path renders. It is an instrument, not a pass/
// fail test: it exists to MEASURE a candidate change before and after, so a
// voicing change can be shown to reduce the transient without regressing the
// shift (the shift must still complete, the clutch must still re-lock, and rpm
// must still synchronise to the new ratio).

#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/runtime/DrivelineModel.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace {
constexpr double stepSeconds = 1.0 / 240.0;

// One recorded simulation tick around the shift.
struct Sample final {
    double t { 0.0 };
    double rpm { 0.0 };
    double throttleCmd { 0.0 };       // driver command (before the cut)
    double engineThrottle { 0.0 };    // command * torqueCutMultiplier -> engine
    double torqueCut { 1.0 };
    double exhaustFlowGps { 0.0 };
    double clutchTorque { 0.0 };
    double clutchPressure { 0.0 };
    double slipRpm { 0.0 };
    double shiftProgress { 0.0 };
    int engagedGear { -1 };
    bool shifting { false };
};

struct ShiftMetrics final {
    std::string name;
    bool shiftCompleted { false };    // returned to a locked, non-shifting state
    bool relocked { false };          // |slip| fell back within the lock band
    int gearBefore { -1 };
    int gearAfter { -1 };
    double rpmAtShift { 0.0 };
    double rpmAfterSync { 0.0 };      // rpm at the moment the clutch first re-locks
    double resyncMs { 0.0 };          // shift issue -> clutch re-lock, milliseconds
    double baselineFlowGps { 0.0 };   // mean exhaust flow just before the shift
    double peakFlowGps { 0.0 };       // peak during the shift window
    double peakFlowSlopeGpsPerMs { 0.0 };  // max |d(flow)/dt|, the "crack" sharpness
    double peakClutchTorqueNm { 0.0 };
    double throttleClutchOverlap { 0.0 };  // see computation below
};

// Replicate EngineRuntime's coupling for one tick and return the resulting frame
// plus the driveline output that drove it.
struct TickResult final {
    enginelab::SimulationFrame frame;
    enginelab::DrivelineOutput drive;
    double engineThrottle { 0.0 };
};

TickResult coupledStep(enginelab::EngineSimulator& simulator,
                       enginelab::DrivelineModel& driveline, double throttleCmd,
                       double clutchPedal, bool starter) {
    // updateDriveline runs on the PREVIOUS step's engine state, then the
    // simulator steps with this tick's torque-cut and clutch reaction torque --
    // exactly the ordering in EngineRuntime::run.
    TickResult result;
    result.drive = driveline.advance(stepSeconds, simulator.state(), 0.0, clutchPedal, 0.0);
    enginelab::EngineControls controls;
    controls.ignitionEnabled = true;
    controls.starterEngaged = starter;
    result.engineThrottle = std::clamp(throttleCmd, 0.0, 1.0) * result.drive.torqueCutMultiplier;
    controls.throttle = result.engineThrottle;
    controls.externalTorqueNm = result.drive.engineReactionTorqueNm;
    result.frame = simulator.step(stepSeconds, controls);
    return result;
}

ShiftMetrics measureShift(const enginelab::EngineConfig& config, double triggerRpm,
                          bool trace) {
    enginelab::SimpleEcuModel ecu;
    enginelab::SimplifiedGasolinePhysics physics;
    enginelab::FourStrokeEventGenerator events;
    auto exhaust = enginelab::ExhaustGraph::makeForEngine(config);
    enginelab::EngineSimulator simulator(config, ecu, physics, events, exhaust);
    enginelab::DrivelineModel driveline(config);

    ShiftMetrics metrics;
    metrics.name = config.name;

    double t = 0.0;
    // Phase A -- crank to a self-sustaining idle in neutral.
    driveline.requestGear(-1);
    for (int step = 0; step < static_cast<int>(4.0 / stepSeconds); ++step, t += stepSeconds) {
        const auto starter = simulator.state().rpm < config.idleRpm * 0.85 && t < 3.0;
        coupledStep(simulator, driveline, 0.0, 0.0, starter);
    }
    // Phase B -- engage first gear and launch at WOT, feeding the clutch in over
    // ~0.9 s (a slipping-clutch launch, as a driver would). This is well before
    // the recorded window; it only serves to get the car rolling in gear.
    driveline.requestGear(0);
    const auto launchSeconds = 0.9;
    for (int step = 0; step < static_cast<int>(launchSeconds / stepSeconds); ++step, t += stepSeconds) {
        const auto clutch = std::clamp(static_cast<double>(step) * stepSeconds / launchSeconds, 0.0, 1.0);
        coupledStep(simulator, driveline, 1.0, clutch, false);
    }

    // Phase C -- WOT in gear, clutch fully home, accelerating. Record every tick
    // so we keep the pre-shift context, trigger ONE clutchless upshift when rpm
    // first crosses triggerRpm from a locked state, then keep recording through
    // the shift and the settle that follows.
    std::vector<Sample> samples;
    samples.reserve(4096);
    bool shiftIssued = false;
    int shiftSampleIndex = -1;
    double settleAfterShiftSeconds = 0.0;
    const auto maxPhaseCSeconds = 12.0;
    for (int step = 0; step < static_cast<int>(maxPhaseCSeconds / stepSeconds); ++step, t += stepSeconds) {
        const auto tick = coupledStep(simulator, driveline, 1.0, 1.0, false);
        const auto& s = tick.frame.state;
        Sample sample;
        sample.t = t;
        sample.rpm = s.rpm;
        sample.throttleCmd = 1.0;
        sample.engineThrottle = tick.engineThrottle;
        sample.torqueCut = tick.drive.torqueCutMultiplier;
        sample.exhaustFlowGps = s.exhaustFlowGramsPerSecond;
        sample.clutchTorque = tick.drive.clutchTorqueNm;
        sample.clutchPressure = tick.drive.clutchPressure;
        sample.slipRpm = tick.drive.clutchSlipRpm;
        sample.shiftProgress = tick.drive.shiftProgress;
        sample.engagedGear = tick.drive.engagedGear;
        sample.shifting = tick.drive.shiftInProgress;
        samples.push_back(sample);

        if (!shiftIssued && !tick.drive.shiftInProgress
            && std::abs(tick.drive.clutchSlipRpm) < 2.0 * config.transmission.clutchLockSpeedRpm
            && s.rpm >= triggerRpm
            && tick.drive.engagedGear >= 0
            && tick.drive.engagedGear < static_cast<int>(config.transmission.gearRatios.size()) - 1) {
            driveline.shiftUp();
            shiftIssued = true;
            shiftSampleIndex = static_cast<int>(samples.size()) - 1;
            metrics.rpmAtShift = s.rpm;
            metrics.gearBefore = tick.drive.engagedGear;
        }
        if (shiftIssued) {
            // Stop ~0.6 s after the shift has fully finished.
            if (!tick.drive.shiftInProgress && tick.drive.shiftProgress <= 0.0
                && shiftSampleIndex >= 0
                && samples.size() > static_cast<std::size_t>(shiftSampleIndex) + 4) {
                settleAfterShiftSeconds += stepSeconds;
                if (settleAfterShiftSeconds > 0.6) break;
            }
        }
    }

    if (shiftSampleIndex < 0) {
        // Never reached the trigger (e.g. triggerRpm above what this gear pulls).
        metrics.shiftCompleted = false;
        return metrics;
    }

    // Baseline: mean exhaust flow over the 0.15 s window just before the shift.
    const auto baselineTicks = static_cast<int>(0.15 / stepSeconds);
    double baselineSum = 0.0;
    int baselineCount = 0;
    for (int i = std::max(0, shiftSampleIndex - baselineTicks); i < shiftSampleIndex; ++i) {
        baselineSum += samples[static_cast<std::size_t>(i)].exhaustFlowGps;
        ++baselineCount;
    }
    metrics.baselineFlowGps = baselineCount > 0 ? baselineSum / baselineCount : 0.0;

    // Transient window: from the shift to 0.3 s after it completes.
    double peakFlow = 0.0;
    double peakSlope = 0.0;
    double peakClutch = 0.0;
    double lastFlow = samples[static_cast<std::size_t>(shiftSampleIndex)].exhaustFlowGps;
    double overlap = 0.0;
    bool sawLock = false;
    double rpmAfter = samples.back().rpm;
    const auto lockBand = 2.0 * config.transmission.clutchLockSpeedRpm;
    for (std::size_t i = static_cast<std::size_t>(shiftSampleIndex); i < samples.size(); ++i) {
        const auto& s = samples[i];
        peakFlow = std::max(peakFlow, static_cast<double>(s.exhaustFlowGps));
        const auto slope = std::abs(s.exhaustFlowGps - lastFlow) / (stepSeconds * 1000.0);
        peakSlope = std::max(peakSlope, slope);
        lastFlow = s.exhaustFlowGps;
        peakClutch = std::max(peakClutch, std::abs(s.clutchTorque));
        // Overlap: engine torque restoring (throttle above the cut floor) while
        // the clutch is still slipping to synchronise -- the fight that hardens
        // the shift. Integrated over the whole event, gear engaged onward.
        if (s.engagedGear == metrics.gearBefore + 1) {
            const auto restored = std::clamp((s.engineThrottle - 0.15) / 0.85, 0.0, 1.0);
            const auto slipping = std::clamp(std::abs(s.slipRpm)
                / std::max(1.0, lockBand * 2.0), 0.0, 1.0);
            overlap += restored * slipping * stepSeconds;
        }
        // True synchronisation: the first tick, after the gear has engaged, when
        // the clutch slip falls back within the lock band. The shift's declared
        // completion (progress -> 0) happens BEFORE this on a hard clutchless
        // dump, which is the whole point.
        if (!sawLock && s.engagedGear == metrics.gearBefore + 1
            && i > static_cast<std::size_t>(shiftSampleIndex) + 2
            && std::abs(s.slipRpm) < lockBand) {
            rpmAfter = s.rpm;
            metrics.resyncMs = (s.t - metrics.rpmAtShift == 0.0 ? 0.0
                : (s.t - samples[static_cast<std::size_t>(shiftSampleIndex)].t)) * 1000.0;
            sawLock = true;
        }
    }
    metrics.peakFlowGps = peakFlow;
    metrics.peakFlowSlopeGpsPerMs = peakSlope;
    metrics.peakClutchTorqueNm = peakClutch;
    metrics.throttleClutchOverlap = overlap;
    metrics.gearAfter = samples.back().engagedGear;
    metrics.rpmAfterSync = rpmAfter;
    metrics.shiftCompleted = sawLock && samples.back().engagedGear == metrics.gearBefore + 1;
    metrics.relocked = std::abs(samples.back().slipRpm) < lockBand;

    if (trace) {
        std::printf("== TRACE %s : clutchless WOT upshift g%d->g%d at %.0f rpm ==\n",
                    config.name.c_str(), metrics.gearBefore, metrics.gearBefore + 1,
                    metrics.rpmAtShift);
        std::printf("      t     rpm  thrCmd  engThr   cut   exhF_g/s  clTq_Nm  clPress  slipRpm  prog  gear\n");
        const auto from = std::max(0, shiftSampleIndex - 6);
        for (std::size_t i = static_cast<std::size_t>(from); i < samples.size(); ++i) {
            const auto& s = samples[i];
            std::printf(" %6.3f %7.0f  %5.2f  %6.3f %5.3f %9.2f %8.1f %7.3f %8.1f %5.2f %5d%s\n",
                        s.t, s.rpm, s.throttleCmd, s.engineThrottle, s.torqueCut,
                        s.exhaustFlowGps, s.clutchTorque, s.clutchPressure, s.slipRpm,
                        s.shiftProgress, s.engagedGear,
                        static_cast<int>(i) == shiftSampleIndex ? "  <== SHIFT" : "");
        }
    }
    return metrics;
}

bool containsCaseInsensitive(const std::string& haystack, const std::string& needle) {
    if (needle.empty()) return true;
    const auto it = std::search(haystack.begin(), haystack.end(), needle.begin(), needle.end(),
        [](char a, char b) { return std::tolower(a) == std::tolower(b); });
    return it != haystack.end();
}
} // namespace

int main(int argc, char** argv) {
    std::string filter;
    double triggerRpm = 0.0;   // 0 -> derive from redline
    double triggerFraction = 0.86;
    bool trace = false;
    for (int index = 1; index < argc; ++index) {
        const std::string arg = argv[index];
        if (arg == "--filter" && index + 1 < argc) filter = argv[++index];
        else if (arg == "--rpm" && index + 1 < argc) triggerRpm = std::stod(argv[++index]);
        else if (arg == "--fraction" && index + 1 < argc) triggerFraction = std::stod(argv[++index]);
        else if (arg == "--trace") trace = true;
        else { std::cerr << "usage: ShiftTransientHarness [--filter frag] [--rpm r]"
                            " [--fraction f] [--trace]\n"; return EXIT_FAILURE; }
    }

    const auto catalog = enginelab::loadEngineCatalog(std::filesystem::path(ENGINELAB_CATALOG_ROOT));
    if (!catalog.errors.empty()) {
        std::cerr << "FAILED: catalogue load: " << catalog.errors.front() << '\n';
        return EXIT_FAILURE;
    }

    std::printf("clutchless WOT upshift transient (trigger fraction %.2f of redline)\n",
                triggerFraction);
    std::printf("  %-30s  gearShift   rpm@shift  rpm@sync  resyncMs   baseF   peakF"
                "  slope(g/s/ms)  clutchTq  overlap  done\n", "engine");
    for (const auto& entry : catalog.entries) {
        auto config = entry.config;
        enginelab::normaliseEngineConfig(config);
        if (!containsCaseInsensitive(config.name, filter)) continue;
        const auto rpm = triggerRpm > 0.0 ? triggerRpm : config.redlineRpm * triggerFraction;
        const auto m = measureShift(config, rpm, trace);
        if (m.gearBefore < 0) {
            std::printf("  %-30s  (never reached %.0f rpm in gear)\n", config.name.c_str(), rpm);
            continue;
        }
        std::printf("  %-30s   g%d->g%d    %8.0f  %8.0f  %7.1f %7.2f %7.2f     %8.3f  %8.0f %8.4f   %s\n",
                    m.name.c_str(), m.gearBefore, m.gearBefore + 1, m.rpmAtShift, m.rpmAfterSync,
                    m.resyncMs, m.baselineFlowGps, m.peakFlowGps, m.peakFlowSlopeGpsPerMs,
                    m.peakClutchTorqueNm, m.throttleClutchOverlap,
                    m.shiftCompleted && m.relocked ? "yes" : "NO");
    }
    return EXIT_SUCCESS;
}
