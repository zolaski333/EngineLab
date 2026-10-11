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
// 30 mm target / 12-cell cap / RK2 / every-substep numerical configuration.
// Its Courant 0.1 reference was checked against 0.05 on both full safe grids:
// the largest VE difference was 0.2443%, while the production/reference gap
// stayed below 3.84%. This is an internal numerical reduction guard, not a
// manufacturer reference or a claim of spatial convergence. Do not weaken
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
#include <enginelab/simulation/DynoAbsorberController.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
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
constexpr double referenceMaximumCourant = 0.1;

constexpr double sweepStepRpm = 500.0;

// Bounded refinement diagnostics retain the test's ascending warm history.
// They do not substitute a partial sweep for any literature/CTest criterion.
constexpr std::array<std::string_view, 9> diagnosticChannelNames {
    "coolant_c", "oil_c", "runner_gas_c", "exhaust_wall_c", "ignition_deg",
    "target_afr", "fuel_trim", "display_brake_nm", "exhaust_back_kpa"
};
struct RefinementDiagnostics final {
    std::size_t frames {}, mechanicalSteps {}, intakeAccepted {}, intakeRejected {}, intakeFlushes {};
    std::size_t completeCycles {}, invalidCycles {}, droppedCycles {}, limitedFrames {}, limiterFrames {}, sparkCutFrames {};
    std::size_t cylinderFailures {}, plenumFailures {};
    std::uint64_t firstMisfires {}, lastMisfires {};
    double startSeconds {}, endSeconds {}, intakeAdvancedSeconds {}, couplingAdvancedSeconds {};
    double minimumMechanicalHz { std::numeric_limits<double>::infinity() }, maximumMechanicalHz {};
    double minimumCouplingSeconds { std::numeric_limits<double>::infinity() }, maximumCouplingSeconds {};
    double maximumCrankDegrees {}, maximumStableStepRatio {}, maximumCourantCurrent {}, maximumCourantPredictor {};
    double cycleBrakeWorkJoules {}, cycleCrankRadians {};
    std::array<double, diagnosticChannelNames.size()> sum {}, first {}, last {};
    std::array<double, diagnosticChannelNames.size()> minimum {}, maximum {};

    RefinementDiagnostics() {
        minimum.fill(std::numeric_limits<double>::infinity());
        maximum.fill(-std::numeric_limits<double>::infinity());
    }
    void observe(const enginelab::SimulationFrame& frame, double dt) {
        const auto& state = frame.state;
        auto runnerTemperature = 0.0;
        auto fuelTrim = 0.0;
        for (std::size_t index = 0; index < state.cylinderStateCount; ++index) {
            runnerTemperature += state.cylinderStates[index].intakeRunnerTemperatureC;
            fuelTrim += state.cylinderStates[index].closedLoopFuelTrim;
        }
        const auto cylinderCount = static_cast<double>(std::max<std::size_t>(1, state.cylinderStateCount));
        const std::array<double, diagnosticChannelNames.size()> channels {
            state.coolantTemperatureC, state.oilTemperatureC, runnerTemperature / cylinderCount,
            state.exhaustWallTemperatureC, state.ignitionAdvanceDegrees, state.targetAirFuelRatio,
            fuelTrim / cylinderCount, state.torqueNm, state.exhaustBackPressureKpa
        };
        if (frames == 0) {
            first = channels;
            startSeconds = state.simulationTimeSeconds - dt;
        }
        ++frames;
        last = channels;
        lastMisfires = state.misfireEventCount;
        endSeconds = state.simulationTimeSeconds;
        for (std::size_t index = 0; index < channels.size(); ++index) {
            sum[index] += channels[index];
            minimum[index] = std::min(minimum[index], channels[index]);
            maximum[index] = std::max(maximum[index], channels[index]);
        }
        mechanicalSteps += state.solverSubsteps;
        intakeAccepted += state.intakeNetworkAcceptedSubsteps;
        intakeRejected += state.intakeNetworkRejectedSubsteps;
        intakeAdvancedSeconds += state.intakeNetworkAdvancedSeconds;
        intakeFlushes += state.intakeCouplingFlushCount;
        couplingAdvancedSeconds += state.intakeCouplingAdvancedSeconds;
        minimumMechanicalHz = std::min(minimumMechanicalHz, static_cast<double>(state.solverSubsteps) / dt);
        maximumMechanicalHz = std::max(maximumMechanicalHz, static_cast<double>(state.solverSubsteps) / dt);
        minimumCouplingSeconds = std::min(minimumCouplingSeconds, state.intakeCouplingMinimumIntervalSeconds);
        maximumCouplingSeconds = std::max(maximumCouplingSeconds, state.intakeCouplingMaximumIntervalSeconds);
        maximumCrankDegrees = std::max(maximumCrankDegrees, state.crankDegreesPerSolverStep);
        maximumStableStepRatio = std::max(maximumStableStepRatio, state.intakeNetworkMaximumAcceptedStepRatio);
        maximumCourantCurrent = std::max(maximumCourantCurrent, state.intakeNetworkMaximumCourantCurrent);
        maximumCourantPredictor = std::max(maximumCourantPredictor, state.intakeNetworkMaximumCourantPredictor);
        limitedFrames += state.solverResolutionLimited ? 1U : 0U;
        limiterFrames += state.ecuSoftRevLimiterActive || state.ecuHardRevLimiterActive ? 1U : 0U;
        sparkCutFrames += state.ecuAlternatingSparkCutActive ? 1U : 0U;
        cylinderFailures += state.intakeCylinderTransferFailures;
        plenumFailures += state.intakePlenumTransferFailures;
        completeCycles += frame.completedBrakeCycleSampleCount;
        droppedCycles += frame.droppedCompletedBrakeCycleSampleCount;
        for (std::size_t cycle = 0; cycle < frame.completedBrakeCycleSampleCount; ++cycle) {
            const auto& sample = frame.completedBrakeCycleSamples[cycle];
            invalidCycles += sample.numericallyValid ? 0U : 1U;
            cycleBrakeWorkJoules += sample.brakeWorkJoules;
            cycleCrankRadians += sample.integratedCrankRadians;
        }
    }
};

struct VePoint final {
    double targetRpm { 0.0 };
    double actualRpm { 0.0 };
    double minimumRpm { 0.0 };
    double maximumRpm { 0.0 };
    double minimumHeldRpm { 0.0 };
    double maximumHeldRpm { 0.0 };
    double ve { 0.0 };
    double mapKpa { 0.0 };
    double lambda { 0.0 };
    bool finite { true };
    RefinementDiagnostics diagnostics;
};

// Hold `targetRpm` under a wide-open-throttle absorber and average the steady
// state. Identical controller to the dyno-sweep harness, which produced the
// curves this instrument was designed against.
VePoint holdPoint(
    enginelab::EngineSimulator& simulator,
    const enginelab::EngineConfig& config, double targetRpm,
    double settleSeconds, double sampleSeconds, bool diagnostics = false) {
    constexpr double dt = 1.0 / 240.0;
    const auto settleSteps = static_cast<int>(settleSeconds / dt);
    const auto sampleSteps = static_cast<int>(sampleSeconds / dt);
    enginelab::DynoAbsorberController absorber(config);
    absorber.reset(
        simulator.state().rpm, simulator.state().torqueNm);
    VePoint result;
    result.targetRpm = targetRpm;
    auto samples = 0.0;
    auto rpmAcc = 0.0, veAcc = 0.0, mapAcc = 0.0, lambdaAcc = 0.0;
    auto minimumRpm = std::numeric_limits<double>::infinity();
    auto maximumRpm = 0.0;
    auto minimumHeldRpm = std::numeric_limits<double>::infinity();
    auto maximumHeldRpm = 0.0;
    for (int step = 0; step < settleSteps + sampleSteps; ++step) {
        if (diagnostics && step == settleSteps)
            result.diagnostics.firstMisfires = simulator.state().misfireEventCount;
        const auto absorberOutput = absorber.advance(
            dt, targetRpm, simulator.state());
        enginelab::EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = simulator.state().rpm < 550.0;
        controls.throttle = 1.0;
        controls.dynamometerTorqueNm =
            absorberOutput.brakeTorqueNm;
        const auto frame = simulator.step(dt, controls);
        if (step < settleSteps) continue;
        if (diagnostics) result.diagnostics.observe(frame, dt);
        if (!std::isfinite(frame.state.rpm) || !std::isfinite(frame.state.volumetricEfficiency)
            || !std::isfinite(frame.state.manifoldPressureKpa) || !std::isfinite(frame.state.lambda))
            result.finite = false;
        samples += 1.0;
        rpmAcc += frame.state.rpm;
        minimumRpm = std::min(minimumRpm, frame.state.rpm);
        maximumRpm = std::max(maximumRpm, frame.state.rpm);
        minimumHeldRpm = std::min(
            minimumHeldRpm, absorberOutput.filteredRpm);
        maximumHeldRpm = std::max(
            maximumHeldRpm, absorberOutput.filteredRpm);
        veAcc += frame.state.volumetricEfficiency;
        mapAcc += frame.state.manifoldPressureKpa;
        lambdaAcc += frame.state.lambda;
    }
    const auto d = std::max(1.0, samples);
    result.actualRpm = rpmAcc / d;
    result.minimumRpm = minimumRpm;
    result.maximumRpm = maximumRpm;
    result.minimumHeldRpm = minimumHeldRpm;
    result.maximumHeldRpm = maximumHeldRpm;
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
                        const enginelab::EngineSimulatorOptions& options, bool diagnostics = false,
                        double maximumOperatingRpm = std::numeric_limits<double>::infinity()) {
    enginelab::SimpleEcuModel ecu;
    enginelab::SimplifiedGasolinePhysics physics;
    enginelab::FourStrokeEventGenerator events;
    auto exhaust = enginelab::ExhaustGraph::makeForEngine(config);
    enginelab::EngineSimulator simulator(
        config, ecu, physics, events, exhaust, options);
    if (diagnostics) {
        simulator.captureGasFieldNow();
        for (const auto& element : simulator.gasField().elements) {
            if (element.kind != enginelab::GasFieldElementKind::intakeRunner) continue;
            const auto cylinder = std::find_if(config.cylinders.begin(), config.cylinders.end(),
                [&element](const auto& candidate) { return candidate.id == element.cylinderId; });
            require(cylinder != config.cylinders.end(), "actual mesh must retain authored cylinder identity");
            const auto lengthMm = cylinder->intakeRunnerLengthMm > 0.0
                ? cylinder->intakeRunnerLengthMm : config.intakePaths[element.pathIndex].geometry.runnerLengthMm;
            std::cout << "mesh cylinder=" << element.cylinderId << " path=" << element.pathIndex
                << " length_mm=" << lengthMm << " actual_cells=" << static_cast<unsigned>(element.sampleCount) << '\n';
        }
    }

    constexpr double dt = 1.0 / 240.0;
    for (int step = 0; step < static_cast<int>(2.0 / dt); ++step) {
        const auto time = static_cast<double>(step) * dt;
        (void)simulator.step(dt, { true, time < 1.5, 0.55, 0.0 });
    }

    SweepResult result;
    const auto startRpm = std::max(2'000.0, std::round(config.idleRpm * 1.5 / sweepStepRpm) * sweepStepRpm);
    auto first = true;
    for (double target = startRpm; target <= std::min(maxRpm, maximumOperatingRpm) + 1.0;
         target += sweepStepRpm) {
        result.points.push_back(holdPoint(
            simulator, config, target,
            first ? 5.0 : 2.0, 1.0, diagnostics));
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
    std::optional<double> diagnosticMaximumRpm;
    std::optional<double> oracleMaximumCourant;
    enginelab::EngineSimulatorOptions options;
    for (int index = 1; index < argc; ++index) {
        const auto argument = std::string_view(argv[index]);
        if (argument == "--enforce-tuning") enforceTuning = true;
        else if (argument == "--compare-oracle") compareOracle = true;
        else if (argument == "--diagnostic-max-rpm" && index + 1 < argc)
            diagnosticMaximumRpm = std::stod(argv[++index]);
        else if (argument == "--oracle-max-courant" && index + 1 < argc) {
            const auto value = std::stod(argv[++index]);
            require(std::isfinite(value) && value > 0.0 && value <= 0.8,
                "oracle Courant must be in (0,0.8]");
            oracleMaximumCourant = value;
        }
        else if (argument == "--intake-joint-manifold" && index + 1 < argc) {
            const auto value = std::string_view(argv[++index]);
            require(value == "0" || value == "1", "--intake-joint-manifold must be0 or1");
            options.intakeJointManifold = value == "1";
        }
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
        else if (argument == "--intake-max-courant" && index + 1 < argc) {
            const auto value = std::stod(argv[++index]);
            require(std::isfinite(value) && value > 0.0 && value <= 0.8,
                "diagnostic intake Courant must be in (0,0.8]");
            options.intakeMaximumCourantNumber = value;
        }
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
    // Match the catalogue/runtime bench envelope. The ECU's absolute soft-cut
    // band starts 220 rpm below its limit; a target inside it measures commanded
    // spark cuts rather than an uncut breathing curve. Keep the physical limit
    // separately for all literature criteria, including the 85% retention point.
    const auto maximumOperatingRpm = std::min(0.95 * maxRpm,
        baseline.ignition.revLimitRpm - 300.0);
    require(!oracleMaximumCourant || compareOracle,
        "an independent oracle Courant override requires the full oracle comparison");
    if (diagnosticMaximumRpm) {
        require(!enforceTuning && !compareOracle && !oracleMaximumCourant,
            "a bounded diagnostic must not claim full peak criteria or the CTest oracle comparison");
        require(std::isfinite(*diagnosticMaximumRpm) && *diagnosticMaximumRpm >= 3'500.0
            && *diagnosticMaximumRpm <= maxRpm, "bounded diagnostic must retain at least four ascending operating points");
        std::cout << std::setprecision(17)
            << "diagnostic_only=true full_curve_criteria_evaluated=false requested_cell_mm="
            << options.intakeTargetCellLengthM.value_or(0.095) * 1'000.0
            << " maximum_courant=" << options.intakeMaximumCourantNumber.value_or(0.8)
            << " joint=" << options.intakeJointManifold.value_or(true)
            << " rk2=" << !options.intakeUsesForwardEuler() << " outer_hz=240\n";
        const auto measured = sweepEngine(baseline, *diagnosticMaximumRpm, options, true);
        for (const auto& point : measured.points) {
            const auto& d = point.diagnostics;
            const auto duration = d.endSeconds - d.startSeconds;
            require(point.finite && std::abs(point.actualRpm - point.targetRpm) <= 0.02 * point.targetRpm
                && point.maximumHeldRpm - point.minimumHeldRpm <= 0.04 * point.targetRpm,
                "a refinement diagnostic must really hold each finite operating point");
            const auto clean = d.completeCycles > 10 && d.invalidCycles == 0 && d.droppedCycles == 0
                && d.limitedFrames == 0 && d.cylinderFailures == 0 && d.plenumFailures == 0
                && d.limiterFrames == 0 && d.sparkCutFrames == 0 && d.lastMisfires == d.firstMisfires;
            std::cout << "refinement target_rpm=" << point.targetRpm << " actual_rpm=" << point.actualRpm
                << " ve=" << point.ve << " map_kpa=" << point.mapKpa << " lambda=" << point.lambda
                << " raw_rpm_min=" << point.minimumRpm << " raw_rpm_max=" << point.maximumRpm
                << " held_rpm_min=" << point.minimumHeldRpm << " held_rpm_max=" << point.maximumHeldRpm
                << " cycle_work_torque_nm=" << d.cycleBrakeWorkJoules / d.cycleCrankRadians
                << " sample_start_s=" << d.startSeconds << " sample_end_s=" << d.endSeconds
                << " frames=" << d.frames << " cycles=" << d.completeCycles << " invalid_cycles=" << d.invalidCycles
                << " dropped_cycles=" << d.droppedCycles << " limited_frames=" << d.limitedFrames
                << " cylinder_failures=" << d.cylinderFailures << " plenum_failures=" << d.plenumFailures
                << " limiter_frames=" << d.limiterFrames << " spark_cut_frames=" << d.sparkCutFrames
                << " misfires=" << d.lastMisfires - d.firstMisfires << " clean=" << clean
                << " mechanical_hz_mean=" << static_cast<double>(d.mechanicalSteps) / duration
                << " mechanical_hz_min=" << d.minimumMechanicalHz << " mechanical_hz_max=" << d.maximumMechanicalHz
                << " max_crank_deg=" << d.maximumCrankDegrees << " intake_accepted=" << d.intakeAccepted
                << " intake_rejected=" << d.intakeRejected << " intake_advanced_s=" << d.intakeAdvancedSeconds
                << " intake_internal_hz=" << static_cast<double>(d.intakeAccepted) / d.intakeAdvancedSeconds
                << " coupling_hz=" << static_cast<double>(d.intakeFlushes) / d.couplingAdvancedSeconds
                << " coupling_min_s=" << d.minimumCouplingSeconds << " coupling_max_s=" << d.maximumCouplingSeconds
                << " max_stable_step_ratio=" << d.maximumStableStepRatio
                << " max_courant_current=" << d.maximumCourantCurrent
                << " max_courant_predictor=" << d.maximumCourantPredictor;
            for (std::size_t channel = 0; channel < diagnosticChannelNames.size(); ++channel) {
                const auto name = diagnosticChannelNames[channel];
                std::cout << ' ' << name << "_mean=" << d.sum[channel] / static_cast<double>(d.frames)
                    << ' ' << name << "_min=" << d.minimum[channel] << ' ' << name << "_max=" << d.maximum[channel]
                    << ' ' << name << "_endpoint_drift_s=" << (d.last[channel] - d.first[channel]) / duration;
            }
            std::cout << '\n';
            require(clean, "a refinement diagnostic cannot infer a numerical trend from contaminated cycles");
        }
        return EXIT_SUCCESS;
    }

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

    std::cout << std::fixed << std::setprecision(compareOracle ? 17 : 3)
              << "--- Intake tuning sweep (inline4, WOT dyno absorber) ---\n";
    std::cout << "physical_rev_limit_rpm=" << maxRpm
        << " safe_measurement_ceiling_rpm=" << maximumOperatingRpm << '\n';
    if (compareOracle) std::cout << "mesh_case=production_base\n";
    const auto base = sweepEngine(baseline, maxRpm, options, compareOracle, maximumOperatingRpm);
    if (compareOracle) std::cout << "mesh_case=production_doubled\n";
    const auto twice = sweepEngine(doubled, maxRpm, options, compareOracle, maximumOperatingRpm);

    // --- Always-on: the instrument's own health. A failure here means the
    // measurement is void, not that the tuning physics regressed.
    require(base.points.size() == twice.points.size(),
            "both sweeps must cover the same operating points");
    for (const auto* sweep : { &base, &twice }) {
        for (const auto& point : sweep->points) {
            std::cout << (sweep == &base ? "  base " : "  2xL  ")
                      << std::setw(5) << static_cast<int>(point.targetRpm) << " rpm -> "
                      << "rpm=" << point.actualRpm << " ve=" << point.ve
                      << " rawRpmBand=[" << point.minimumRpm << ','
                      << point.maximumRpm << ']'
                      << " heldRpmBand=[" << point.minimumHeldRpm << ','
                      << point.maximumHeldRpm << ']'
                      << " map=" << point.mapKpa << "kPa lambda=" << point.lambda << '\n';
            require(point.finite, "sweep telemetry must stay finite at every point");
            require(std::abs(point.actualRpm - point.targetRpm) <= 0.02 * point.targetRpm,
                    "the absorber must actually hold each swept operating point");
            require(point.maximumHeldRpm - point.minimumHeldRpm
                        <= 0.04 * point.targetRpm,
                    "the absorber must settle each complete sampling window");
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
        oracleOptions.intakeMaximumCourantNumber = oracleMaximumCourant.value_or(referenceMaximumCourant);

        std::cout << "--- Intake reduction A/B (production vs 30mm-target/12-cell-cap/RK2/substep numerical reference) ---\n";
        std::cout << "oracle_maximum_courant=" << oracleOptions.intakeMaximumCourantNumber.value_or(referenceMaximumCourant) << '\n';
        std::cout << "mesh_case=oracle_base\n";
        const auto oracleBase = sweepEngine(baseline, maxRpm, oracleOptions, true, maximumOperatingRpm);
        std::cout << "mesh_case=oracle_doubled\n";
        const auto oracleTwice = sweepEngine(doubled, maxRpm, oracleOptions, true, maximumOperatingRpm);
        require(base.points.size() == oracleBase.points.size()
                    && twice.points.size() == oracleTwice.points.size(),
                "production and oracle sweeps must cover the same operating points");

        auto maximumError = 0.0;
        auto allComparisonsClean = true;
        const auto compareSweep = [&maximumError, &allComparisonsClean](
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
                for (const auto* observed : { &measured, &reference }) {
                    const auto& diagnostic = observed->diagnostics;
                    require(diagnostic.frames > 0, "every oracle operating point must expose its quality observations");
                    const auto clean = diagnostic.completeCycles > 10 && diagnostic.invalidCycles == 0
                        && diagnostic.droppedCycles == 0 && diagnostic.limitedFrames == 0
                        && diagnostic.cylinderFailures == 0 && diagnostic.plenumFailures == 0
                        && diagnostic.limiterFrames == 0 && diagnostic.sparkCutFrames == 0
                        && diagnostic.firstMisfires == diagnostic.lastMisfires;
                    allComparisonsClean = allComparisonsClean && clean;
                    const auto duration = diagnostic.endSeconds - diagnostic.startSeconds;
                    std::cout << "oracle_quality variant=" << (observed == &measured ? "production" : "oracle")
                        << " runner_scale=" << (label[0] == 'b' ? 1 : 2) << " target_rpm=" << observed->targetRpm
                        << " ve=" << observed->ve << " cycle_work_torque_nm="
                        << diagnostic.cycleBrakeWorkJoules / diagnostic.cycleCrankRadians
                        << " cycles=" << diagnostic.completeCycles << " invalid_cycles=" << diagnostic.invalidCycles
                        << " dropped_cycles=" << diagnostic.droppedCycles
                        << " limited_frames=" << diagnostic.limitedFrames << " cylinder_failures=" << diagnostic.cylinderFailures
                        << " plenum_failures=" << diagnostic.plenumFailures
                        << " limiter_frames=" << diagnostic.limiterFrames << " spark_cut_frames=" << diagnostic.sparkCutFrames
                        << " misfires=" << diagnostic.lastMisfires - diagnostic.firstMisfires << " clean=" << clean
                        << " intake_accepted=" << diagnostic.intakeAccepted
                        << " intake_rejected=" << diagnostic.intakeRejected
                        << " actual_mechanical_hz=" << static_cast<double>(diagnostic.mechanicalSteps) / duration
                        << " actual_intake_hz=" << static_cast<double>(diagnostic.intakeAccepted) / diagnostic.intakeAdvancedSeconds
                        << " actual_coupling_hz=" << static_cast<double>(diagnostic.intakeFlushes) / diagnostic.couplingAdvancedSeconds
                        << " max_crank_deg=" << diagnostic.maximumCrankDegrees
                        << " max_courant_current=" << diagnostic.maximumCourantCurrent
                        << " max_courant_predictor=" << diagnostic.maximumCourantPredictor;
                    for (std::size_t channel = 0; channel < diagnosticChannelNames.size(); ++channel)
                        std::cout << ' ' << diagnosticChannelNames[channel] << "_mean="
                            << diagnostic.sum[channel] / static_cast<double>(diagnostic.frames);
                    std::cout << '\n';
                }
            }
        };
        compareSweep("base", base, oracleBase);
        compareSweep("2xL ", twice, oracleTwice);
        std::cout << "  maximum VE deviation=" << maximumError * 100.0
                  << "% (limit " << maximumReductionVeError * 100.0 << "%)\n";
        require(allComparisonsClean,
            "every oracle comparison must retain clean numerical and combustion observations");
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
