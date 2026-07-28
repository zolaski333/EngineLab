// Intake-tuning instrument: the volumetric-efficiency curve across the rev
// range, measured at wide-open throttle under a dyno absorber.
//
// This is the guard for intake breathing, written BEFORE the 1-D intake
// gas-dynamics work it now protects (the same design as the combustion phasing
// instrument: the reference numbers come from engine literature, never from
// this simulator's output, so the gate cannot be a re-calibration onto current
// behaviour). It sweeps a known engine at WOT, extracts the VE(rpm) curve, and
// evaluates four criteria a real naturally-aspirated multi-valve engine
// satisfies:
//
//   1. Peak VE in [0.85, 1.15]. Heywood: ordinary NA SI engines peak at
//      0.80-0.90; tuned intake/exhaust (ram + wave action) carries sport
//      engines above 1.0 at the tuned speed. Above 1.15 without boost is
//      unphysical.
//   2. The VE peak sits at or above 40 % of the rev limit. Production sport
//      engines place peak torque (~peak VE) at 55-75 % of the limiter
//      (MT-09: 65 %; R1: ~85 %); 40 % is a deliberately generous floor so
//      only the structural failure -- a curve that can only decay, pinning
//      the "peak" to the sweep floor -- trips it.
//   3. VE at ~85 % of the rev limit holds at least 0.80 x peak. Engines are
//      geared so the power peak sits near the limiter, and P ~ VE x rpm
//      requires VE to hold within ~20 % of its maximum up there.
//   4. Doubling the intake runner length moves the VE peak DOWN by at least
//      15 %. Helmholtz/quarter-wave tuned speed scales as 1/sqrt(L) to 1/L,
//      so 2x length physically shifts the tuned speed down 29-50 %; 15 %
//      catches "runner length does nothing to the delivered fill" without
//      demanding any particular tuning model. This is the decisive criterion:
//      it cannot be satisfied by scaling, only by wave dynamics.
//
// The VE definition matches what a manufacturer quotes: trapped air mass per
// cycle against displacement x AMBIENT density (EngineSimulator publishes
// exactly this), so the literature numbers compare directly.
//
// Historical finding when this was written (2026-07-25): the old lumped 0-D
// path failed criteria 2 and 4. The resolved 1-D runners subsequently promoted
// all four literature criteria to CTest gates. The 2026-07-28 realtime
// reduction adds a second, deliberately different guard under
// --compare-oracle: every production VE point must remain within 15% of the
// former 30 mm / RK2 / every-substep numerical configuration. Do not weaken
// either guard to make a change pass.
//
// Instrument notes, learned the hard way elsewhere in this repo:
//  * WOT under a dyno absorber is the only way to measure a VE curve here.
//    A part-throttle "hold" is not a lighter version of the same measurement:
//    the ECU answers the absorber's load by reopening the plate.
//  * The peak location is refined with a 3-point parabolic fit so the 500-rpm
//    sweep grid cannot alias a genuine >=15 % runner-doubling shift into a
//    sub-threshold reading.

#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/foundation/EngineTypes.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace {
void require(bool condition, const char* message) {
    if (condition) return;
    std::cerr << "FAILED: " << message << '\n';
    std::exit(EXIT_FAILURE);
}

// Literature-anchored tuning windows (documented above). Reported by default,
// gated under --enforce-tuning; promoted to always-on with the 1-D intake.
constexpr double targetPeakVeLow = 0.85;
constexpr double targetPeakVeHigh = 1.15;
constexpr double targetPeakRpmFractionOfRevLimit = 0.40;
constexpr double targetHighRpmVeFractionOfPeak = 0.80;
constexpr double targetRunnerDoublingPeakShift = 0.15;
constexpr double maximumReductionVeError = 0.15;

constexpr double sweepStepRpm = 500.0;

struct VePoint final {
    double targetRpm { 0.0 };
    double actualRpm { 0.0 };
    double ve { 0.0 };
    double mapKpa { 0.0 };
    double lambda { 0.0 };
    bool finite { true };
};

// Hold `targetRpm` under a wide-open-throttle absorber and average the steady
// state. Identical controller to the dyno-sweep harness, which produced the
// curves this instrument was designed against.
VePoint holdPoint(enginelab::EngineSimulator& simulator, double targetRpm,
                  double settleSeconds, double sampleSeconds) {
    constexpr double dt = 1.0 / 240.0;
    const auto settleSteps = static_cast<int>(settleSeconds / dt);
    const auto sampleSteps = static_cast<int>(sampleSeconds / dt);
    auto dynoIntegral = 0.0;
    VePoint result;
    result.targetRpm = targetRpm;
    auto samples = 0.0;
    auto rpmAcc = 0.0, veAcc = 0.0, mapAcc = 0.0, lambdaAcc = 0.0;
    for (int step = 0; step < settleSteps + sampleSteps; ++step) {
        const auto speedError = (simulator.state().rpm - targetRpm) / std::max(1.0, targetRpm);
        // Integral gain 12.0, not 1.20: at 1/240 s steps the old gain could not
        // wind up inside a settle window, so above ~5000 rpm the engine drifted
        // up to 7 % past the target and the top point was measured with the rev
        // limiter cutting spark. See tests/GasExchangeTests.cpp for the
        // measurement and docs/physics-audit.md.
        dynoIntegral = std::clamp(dynoIntegral + speedError * dt * 12.0, 0.0, 1.0);
        enginelab::EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = simulator.state().rpm < 550.0;
        controls.throttle = 1.0;
        controls.load = std::clamp(dynoIntegral + speedError * 0.70, 0.0, 1.0);
        const auto frame = simulator.step(dt, controls);
        if (step < settleSteps) continue;
        if (!std::isfinite(frame.state.rpm) || !std::isfinite(frame.state.volumetricEfficiency)
            || !std::isfinite(frame.state.manifoldPressureKpa) || !std::isfinite(frame.state.lambda))
            result.finite = false;
        samples += 1.0;
        rpmAcc += frame.state.rpm;
        veAcc += frame.state.volumetricEfficiency;
        mapAcc += frame.state.manifoldPressureKpa;
        lambdaAcc += frame.state.lambda;
    }
    const auto d = std::max(1.0, samples);
    result.actualRpm = rpmAcc / d;
    result.ve = veAcc / d;
    result.mapKpa = mapAcc / d;
    result.lambda = lambdaAcc / d;
    return result;
}

struct SweepResult final {
    std::vector<VePoint> points;
    double peakVe { 0.0 };
    double peakVeRpm { 0.0 };
    double highRpmVe { 0.0 };
    double highRpmTarget { 0.0 };
};

// WOT sweep, warm-started ascending like the dyno harness. The peak location
// is refined with a 3-point parabolic fit on the uniform target-rpm grid when
// the discrete maximum is interior; a maximum pinned to either end of the
// sweep is reported as-is (and at the low end is itself the diagnosis).
SweepResult sweepEngine(const enginelab::EngineConfig& config, double maxRpm,
                        const enginelab::EngineSimulatorOptions& options) {
    enginelab::SimpleEcuModel ecu;
    enginelab::SimplifiedGasolinePhysics physics;
    enginelab::FourStrokeEventGenerator events;
    auto exhaust = enginelab::ExhaustGraph::makeForEngine(config);
    enginelab::EngineSimulator simulator(
        config, ecu, physics, events, exhaust, options);

    constexpr double dt = 1.0 / 240.0;
    for (int step = 0; step < static_cast<int>(2.0 / dt); ++step) {
        const auto time = static_cast<double>(step) * dt;
        (void)simulator.step(dt, { true, time < 1.5, 0.55, 0.0 });
    }

    SweepResult result;
    const auto startRpm = std::max(2'000.0, std::round(config.idleRpm * 1.5 / sweepStepRpm) * sweepStepRpm);
    auto first = true;
    for (double target = startRpm; target <= maxRpm + 1.0; target += sweepStepRpm) {
        result.points.push_back(holdPoint(simulator, target, first ? 3.0 : 2.0, 1.0));
        first = false;
    }
    require(result.points.size() >= 4, "the sweep must cover enough points to locate a peak");

    std::size_t peakIndex = 0;
    for (std::size_t index = 0; index < result.points.size(); ++index)
        if (result.points[index].ve > result.points[peakIndex].ve) peakIndex = index;
    result.peakVe = result.points[peakIndex].ve;
    result.peakVeRpm = result.points[peakIndex].targetRpm;
    if (peakIndex > 0 && peakIndex + 1 < result.points.size()) {
        const auto left = result.points[peakIndex - 1].ve;
        const auto centre = result.points[peakIndex].ve;
        const auto right = result.points[peakIndex + 1].ve;
        const auto curvature = left - 2.0 * centre + right;
        if (curvature < -1.0e-12) {
            const auto offset = std::clamp(0.5 * (left - right) / curvature, -1.0, 1.0);
            result.peakVeRpm += offset * sweepStepRpm;
            result.peakVe = centre - 0.25 * (left - right) * offset;
        }
    }

    // VE retention: the swept point nearest 85 % of the rev limit.
    const auto highRpmGoal = 0.85 * maxRpm;
    std::size_t highIndex = 0;
    for (std::size_t index = 0; index < result.points.size(); ++index)
        if (std::abs(result.points[index].targetRpm - highRpmGoal)
            < std::abs(result.points[highIndex].targetRpm - highRpmGoal))
            highIndex = index;
    result.highRpmVe = result.points[highIndex].ve;
    result.highRpmTarget = result.points[highIndex].targetRpm;
    return result;
}
}  // namespace

int main(int argc, char** argv) {
    auto enforceTuning = false;
    auto compareOracle = false;
    enginelab::EngineSimulatorOptions options;
    for (int index = 1; index < argc; ++index) {
        const auto argument = std::string_view(argv[index]);
        if (argument == "--enforce-tuning") enforceTuning = true;
        else if (argument == "--compare-oracle") compareOracle = true;
        else if (argument == "--intake-max-cells" && index + 1 < argc)
            options.intakeMaximumCellCount =
                static_cast<std::size_t>(std::stoull(argv[++index]));
        else if (argument == "--intake-staircase-rounds" && index + 1 < argc)
            options.intakeStaircaseRounds =
                static_cast<std::size_t>(std::stoull(argv[++index]));
        else if (argument == "--intake-workers" && index + 1 < argc)
            options.intakeWorkerCount =
                static_cast<std::size_t>(std::stoull(argv[++index]));
        else if (argument == "--intake-coupling-us" && index + 1 < argc)
            options.intakeCouplingIntervalSeconds =
                std::stod(argv[++index]) * 1.0e-6;
        else if (argument == "--intake-cell-mm" && index + 1 < argc)
            options.intakeTargetCellLengthM =
                std::stod(argv[++index]) * 1.0e-3;
        else if (argument == "--intake-euler")
            options.intakeFirstOrderTimeIntegration = true;
        else if (argument == "--intake-rk2")
            options.intakeFirstOrderTimeIntegration = false;
        else if (argument == "--intake-wall-us" && index + 1 < argc)
            options.intakeWallHeatUpdateIntervalSeconds =
                std::stod(argv[++index]) * 1.0e-6;
    }

    auto baseline = enginelab::makeDefaultInlineFour();
    enginelab::normaliseEngineConfig(baseline);
    const auto maxRpm = std::min(baseline.redlineRpm, baseline.ignition.revLimitRpm);

    // Doubled-runner variant: scale every place a runner length can be
    // authored, so the comparison holds whichever one the engine uses.
    auto doubled = baseline;
    doubled.intake.runnerLengthMm *= 2.0;
    for (auto& path : doubled.intakePaths) path.geometry.runnerLengthMm *= 2.0;
    for (auto& cylinder : doubled.cylinders)
        if (cylinder.intakeRunnerLengthMm > 0.0) cylinder.intakeRunnerLengthMm *= 2.0;
    enginelab::normaliseEngineConfig(doubled);
    require(doubled.intake.runnerLengthMm > 1.9 * baseline.intake.runnerLengthMm,
            "normalisation must not clamp the doubled runner length away");

    std::cout << std::fixed << std::setprecision(3)
              << "--- Intake tuning sweep (inline4, WOT dyno absorber) ---\n";
    const auto base = sweepEngine(baseline, maxRpm, options);
    const auto twice = sweepEngine(doubled, maxRpm, options);

    // --- Always-on: the instrument's own health. A failure here means the
    // measurement is void, not that the tuning physics regressed.
    require(base.points.size() == twice.points.size(),
            "both sweeps must cover the same operating points");
    for (const auto* sweep : { &base, &twice }) {
        for (const auto& point : sweep->points) {
            std::cout << (sweep == &base ? "  base " : "  2xL  ")
                      << std::setw(5) << static_cast<int>(point.targetRpm) << " rpm -> "
                      << "rpm=" << point.actualRpm << " ve=" << point.ve
                      << " map=" << point.mapKpa << "kPa lambda=" << point.lambda << '\n';
            require(point.finite, "sweep telemetry must stay finite at every point");
            require(std::abs(point.actualRpm - point.targetRpm) <= 0.02 * point.targetRpm,
                    "the absorber must actually hold each swept operating point");
            require(point.ve > 0.2 && point.ve < 1.6,
                    "VE must stay physically bounded at WOT");
            require(point.mapKpa > 80.0,
                    "MAP must sit near ambient at WOT -- otherwise this is not a VE measurement");
            require(point.lambda > 0.6 && point.lambda < 1.4,
                    "mixture must stay in the combustible band through the sweep");
        }
        require(sweep->points.back().targetRpm >= 0.85 * maxRpm,
                "the sweep must reach the top of the rev range");
    }

    if (compareOracle) {
        auto oracleOptions = options;
        oracleOptions.intakeMaximumCellCount = 12;
        oracleOptions.intakeTargetCellLengthM = 0.030;
        oracleOptions.intakeStaircaseRounds = 2;
        oracleOptions.intakeCouplingIntervalSeconds = 0.0;
        oracleOptions.intakeFirstOrderTimeIntegration = false;

        std::cout << "--- Intake reduction A/B (production vs 30mm/RK2/substep oracle) ---\n";
        const auto oracleBase = sweepEngine(baseline, maxRpm, oracleOptions);
        const auto oracleTwice = sweepEngine(doubled, maxRpm, oracleOptions);
        require(base.points.size() == oracleBase.points.size()
                    && twice.points.size() == oracleTwice.points.size(),
                "production and oracle sweeps must cover the same operating points");

        auto maximumError = 0.0;
        const auto compareSweep = [&maximumError](
                                      const char* label,
                                      const SweepResult& production,
                                      const SweepResult& oracle) {
            for (std::size_t index = 0; index < production.points.size(); ++index) {
                const auto& measured = production.points[index];
                const auto& reference = oracle.points[index];
                require(reference.finite, "oracle telemetry must stay finite");
                require(std::abs(reference.actualRpm - reference.targetRpm)
                            <= 0.02 * reference.targetRpm,
                        "the absorber must hold every oracle operating point");
                require(std::abs(measured.targetRpm - reference.targetRpm) < 1.0,
                        "production and oracle target grids must match");
                const auto relativeError = std::abs(measured.ve - reference.ve)
                    / std::max(0.05, std::abs(reference.ve));
                maximumError = std::max(maximumError, relativeError);
                std::cout << "  " << label << ' '
                          << std::setw(5) << static_cast<int>(measured.targetRpm)
                          << " rpm production=" << measured.ve
                          << " oracle=" << reference.ve
                          << " error=" << relativeError * 100.0 << "%\n";
            }
        };
        compareSweep("base", base, oracleBase);
        compareSweep("2xL ", twice, oracleTwice);
        std::cout << "  maximum VE deviation=" << maximumError * 100.0
                  << "% (limit " << maximumReductionVeError * 100.0 << "%)\n";
        require(maximumError <= maximumReductionVeError,
                "the realtime intake reduction must stay within 15% of the numerical oracle");
    }

    // --- Literature criteria: reported always, enforced under --enforce-tuning.
    const auto peakShift = base.peakVeRpm > 0.0
        ? (base.peakVeRpm - twice.peakVeRpm) / base.peakVeRpm : 0.0;
    const auto within = [](bool ok) { return ok ? "OK" : "outside target"; };
    const auto crit1 = base.peakVe >= targetPeakVeLow && base.peakVe <= targetPeakVeHigh;
    const auto crit2 = base.peakVeRpm >= targetPeakRpmFractionOfRevLimit * maxRpm;
    const auto crit3 = base.highRpmVe >= targetHighRpmVeFractionOfPeak * base.peakVe;
    const auto crit4 = peakShift >= targetRunnerDoublingPeakShift;
    std::cout << "  crit1 peak VE = " << base.peakVe << " in [" << targetPeakVeLow << ", "
              << targetPeakVeHigh << "] (" << within(crit1) << ")\n"
              << "  crit2 peak at " << base.peakVeRpm << " rpm >= "
              << targetPeakRpmFractionOfRevLimit * maxRpm << " (" << within(crit2) << ")\n"
              << "  crit3 VE at " << base.highRpmTarget << " rpm = " << base.highRpmVe
              << " >= " << targetHighRpmVeFractionOfPeak * base.peakVe << " (" << within(crit3) << ")\n"
              << "  crit4 2x runner peak shift = " << peakShift * 100.0 << "% >= "
              << targetRunnerDoublingPeakShift * 100.0 << "% (" << within(crit4) << ")\n";
    if (enforceTuning) {
        require(crit1, "peak VE must sit in the naturally-aspirated literature band");
        require(crit2, "the VE peak must sit in the upper rev range, not at the sweep floor");
        require(crit3, "VE must hold near its peak approaching the rev limit");
        require(crit4, "doubling the runner length must move the VE peak down by >= 15%");
    } else {
        std::cout << "  (tuning criteria reported only; --enforce-tuning gates them --"
                     " promoted when the 1-D intake lands)\n";
    }

    std::cout << "Intake tuning tests passed\n";
    return EXIT_SUCCESS;
}
