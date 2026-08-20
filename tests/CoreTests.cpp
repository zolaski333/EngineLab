#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/foundation/SpscQueue.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/physics/ConservativeGasSystem.hpp>
#include <enginelab/physics/FlamePhysicsModel.hpp>
#include <enginelab/physics/CompressionIgnitionModel.hpp>
#include <enginelab/physics/FuelInjectionModel.hpp>
#include <enginelab/physics/EndGasKnockModel.hpp>
#include <enginelab/physics/MechanicalKinematics.hpp>
#include <enginelab/physics/IndicatedWorkModel.hpp>
#include <enginelab/physics/ValveTrainModel.hpp>
#include <enginelab/physics/HelmholtzRunnerModel.hpp>
#include <enginelab/runtime/DrivelineModel.hpp>
#include <enginelab/runtime/MonotonicPublicationTimeline.hpp>
#include <enginelab/runtime/RealtimeLoadGovernor.hpp>
#include <enginelab/serialization/JsonEngineSerializer.hpp>
#include <enginelab/serialization/YamlEngineSerializer.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>
#include <enginelab/simulation/TransientChargeEstimator.hpp>
#include <enginelab/runtime/EngineRuntime.hpp>
#include <enginelab/audio/ForcedInductionAcoustics.hpp>
#include <enginelab/audio/RealtimeEngineAudio.hpp>
#include <enginelab/audio/ImpulseResponseLoader.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>
#include <chrono>
#include <algorithm>
#include <array>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <numeric>
#include <numbers>
#include <set>
#include <cmath>
#include <string_view>
#include <tuple>
#include <stdexcept>
#include <thread>

namespace {
void require(bool condition, const char* message) {
    if (!condition) { std::cerr << "FAILED: " << message << '\n'; std::exit(EXIT_FAILURE); }
}

/** Test-only crank-cycle contract with deliberately short cycles.
 *
 * Engine thermodynamics remain the production implementation; only the cycle
 * boundary spacing is shortened so one bounded outer step exercises the fixed
 * completion buffer and its observable overflow path.
 */
class ShortCycleEventGenerator final : public enginelab::IFiringEventGenerator {
public:
    [[nodiscard]] double cycleDegrees() const noexcept override { return 10.0; }
    std::size_t generate(const enginelab::EngineConfig&,
                         const enginelab::EngineState&,
                         const enginelab::EcuCommand&,
                         const enginelab::CombustionResult&,
                         double, double, double, double,
                         std::span<enginelab::FiringEvent>) noexcept override {
        return 0;
    }
};
}

int main() {
    try {
    {
        const enginelab::TrappedChargeReference reference {
            120.0, 50.0, 300.0
        };
        const auto doubledDensityCharge =
            enginelab::TransientChargeEstimator::estimateFreshAirMassMg(
                40.0, reference, 100.0, 300.0);
        require(std::abs(doubledDensityCharge - 240.0) < 1.0e-12,
            "transient charge estimate must follow the ideal-gas density ratio");
        const auto warmerCharge =
            enginelab::TransientChargeEstimator::estimateFreshAirMassMg(
                40.0, reference, 50.0, 360.0);
        require(std::abs(warmerCharge - 100.0) < 1.0e-12,
            "transient charge estimate must account for source temperature");
        const auto resolvedLowerBound =
            enginelab::TransientChargeEstimator::estimateFreshAirMassMg(
                130.0, reference, 50.0, 360.0);
        require(std::abs(resolvedLowerBound - 130.0) < 1.0e-12,
            "resolved chamber oxygen must bound the predicted charge from below");
        const auto invalidFallback =
            enginelab::TransientChargeEstimator::estimateFreshAirMassMg(
                73.0, {}, 100.0, 300.0);
        require(std::abs(invalidFallback - 73.0) < 1.0e-12,
            "invalid charge history must fall back to resolved chamber oxygen");
    }
    {
        // Total friction mean effective pressure must track the gasoline
        // literature band across the whole rev range, not just be finite. The
        // model sums an empirical non-piston polynomial (bearings + valvetrain +
        // accessories, misleadingly named bearingFriction) with a resolved
        // Stribeck piston friction; this guards their sum against a future
        // double-count or miscalibration. Reference FMEP for a small gasoline
        // four: ~0.5-0.8 bar near idle, ~1.0-1.3 bar mid, ~1.8-2.5 bar at redline.
        // Measured baseline: 0.56 @750, 0.99 @2750, 1.11 @3250, 2.14 @7250.
        auto cfg = enginelab::makeDefaultInlineFour();
        enginelab::normaliseEngineConfig(cfg);
        enginelab::SimpleEcuModel ecu;
        enginelab::SimplifiedGasolinePhysics physics;
        enginelab::FourStrokeEventGenerator events;
        auto exhaust = enginelab::ExhaustGraph::makeForEngine(cfg);
        enginelab::EngineSimulator sim(cfg, ecu, physics, events, exhaust);
        constexpr double dt = 1.0 / 2000.0;
        std::array<double, 16> fmepSum {}; std::array<int, 16> fmepN {};
        double maximumSweepRpm = 0.0;
        for (int step = 0; step < static_cast<int>(8.0 / dt); ++step) {
            const auto time = step * dt;
            enginelab::EngineControls c;
            c.ignitionEnabled = true;
            c.starterEngaged = time < 1.2;
            c.throttle = time < 1.2 ? 0.5 : 1.0;
            // This fixture validates speed-dependent friction, not maximum-load
            // output. An unladen acceleration traverses the requested bins even
            // when the exhaust model resolves physical pumping work.
            c.load = 0.0;
            const auto f = sim.step(dt, c);
            maximumSweepRpm = std::max(maximumSweepRpm, f.state.rpm);
            const auto bin = static_cast<int>(f.state.rpm / 500.0);
            if (time > 0.3 && bin >= 0 && bin < 16) {
                fmepSum[bin] += f.state.frictionMeanEffectivePressureBar; ++fmepN[bin];
            }
        }
        const auto binFmep = [&](int b) { return fmepN[b] > 0 ? fmepSum[b] / fmepN[b] : -1.0; };
        // Every populated running bin must stay inside a physical FMEP envelope.
        for (int b = 1; b < 16; ++b) { // skip bin 0 (<500 rpm, cranking/stall)
            if (fmepN[b] == 0) continue;
            const auto fmep = binFmep(b);
            require(fmep > 0.35 && fmep < 2.7,
                    "friction MEP must stay within the gasoline literature envelope across rpm");
        }
        // Friction must rise with speed (idle < mid < high), the FMEP signature.
        const auto idleFmep = binFmep(1) > 0 ? binFmep(1) : binFmep(2);   // ~500-1000 rpm
        const auto midFmep = binFmep(6) > 0 ? binFmep(6) : binFmep(5);    // ~2750-3000 rpm
        const auto highFmep = binFmep(12) > 0 ? binFmep(12) : binFmep(11);// ~5750-6250 rpm
        if (!(idleFmep > 0.0 && midFmep > 0.0 && highFmep > 0.0))
            std::cerr << "FMEP sweep diagnostics: max_rpm=" << maximumSweepRpm
                      << " idle=" << idleFmep << " mid=" << midFmep
                      << " high=" << highFmep << '\n';
        require(idleFmep > 0.0 && midFmep > 0.0 && highFmep > 0.0,
                "FMEP sweep must populate idle, mid and high rpm bins");
        require(midFmep > idleFmep && highFmep > midFmep,
                "friction MEP must increase monotonically with engine speed");
        require(midFmep > 0.8 && midFmep < 1.4,
                "mid-range friction MEP must match the gasoline reference (~1.0-1.3 bar)");
    }
    {
        enginelab::IndicatedWorkState work;
        enginelab::IndicatedWorkModel::advance(work, 100.0, 1.0, 100.0, false, false, false);
        enginelab::IndicatedWorkModel::advance(work, 200.0, 1.0, 100.0, false, false, false);
        enginelab::IndicatedWorkModel::advance(work, 200.0, 2.0, 100.0, false, false, false);
        enginelab::IndicatedWorkModel::advance(work, 100.0, 2.0, 100.0, false, false, false);
        enginelab::IndicatedWorkModel::advance(work, 100.0, 1.0, 100.0, true, false, false);
        require(std::abs(work.completedCycleJoules - 100.0) < 1.0e-9,
                "P-dV integration must recover the signed area of a known pressure-volume loop");
        require(work.completedPumpingCycleJoules == 0.0,
                "a loop walked entirely outside the gas-exchange strokes must have no pumping share");

        // The same loop again, but with the two constant-pressure legs flagged
        // as gas exchange. Those legs are where all the volume change happens,
        // so the split must attribute the whole +100 J: the expansion leg at
        // 200 kPa gives +100 J and the return leg at 100 kPa gives 0 J against
        // a 100 kPa ambient. This is the non-vacuous half -- it fails if the
        // flag is ignored, and it fails differently if the share is integrated
        // separately rather than taken from the same increment.
        // The expansion leg is additionally flagged as the exhaust stroke, so the
        // exhaust-stroke sub-share must take the whole +100 J and the intake half
        // (pumping minus exhaust) must come out at zero.
        enginelab::IndicatedWorkState split;
        enginelab::IndicatedWorkModel::advance(split, 100.0, 1.0, 100.0, false, false, false);
        enginelab::IndicatedWorkModel::advance(split, 200.0, 1.0, 100.0, false, false, false);
        enginelab::IndicatedWorkModel::advance(split, 200.0, 2.0, 100.0, false, true, true);
        enginelab::IndicatedWorkModel::advance(split, 100.0, 2.0, 100.0, false, false, false);
        enginelab::IndicatedWorkModel::advance(split, 100.0, 1.0, 100.0, true, true, false);
        require(std::abs(split.completedCycleJoules - 100.0) < 1.0e-9,
                "flagging strokes must not change the total indicated work of the loop");
        require(std::abs(split.completedPumpingCycleJoules - 100.0) < 1.0e-9,
                "the pumping share must be the part of the same integral flagged as gas exchange");
        require(std::abs(split.completedExhaustStrokeCycleJoules - 100.0) < 1.0e-9,
                "the exhaust-stroke sub-share must be the part flagged as exhaust stroke");

        // The same loop with the exhaust flag moved to the other gas-exchange
        // leg. Without this the assertion above would also pass if the sub-share
        // simply copied the pumping accumulator and ignored its own flag.
        enginelab::IndicatedWorkState moved;
        enginelab::IndicatedWorkModel::advance(moved, 100.0, 1.0, 100.0, false, false, false);
        enginelab::IndicatedWorkModel::advance(moved, 200.0, 1.0, 100.0, false, false, false);
        enginelab::IndicatedWorkModel::advance(moved, 200.0, 2.0, 100.0, false, true, false);
        enginelab::IndicatedWorkModel::advance(moved, 100.0, 2.0, 100.0, false, false, false);
        enginelab::IndicatedWorkModel::advance(moved, 100.0, 1.0, 100.0, true, true, true);
        require(std::abs(moved.completedPumpingCycleJoules - 100.0) < 1.0e-9,
                "moving the exhaust flag must not change the pumping share");
        require(std::abs(moved.completedExhaustStrokeCycleJoules) < 1.0e-9,
                "the exhaust-stroke sub-share must follow its own flag, not the pumping one");

        enginelab::CamshaftConfig cam;
        cam.intakeFlowCurve = { { 0.0, 0.0 }, { 5.0, 0.52 }, { 10.0, 0.68 } };
        cam.exhaustFlowCurve = { { 0.0, 0.0 }, { 10.0, 0.64 } };
        cam.continuousControl.enabled = true;
        cam.continuousControl.responseFrequencyHz = 20.0;
        cam.continuousControl.samples = { { 1'000.0, 0.2, 4.0, -2.0, 0.90 },
                                          { 6'000.0, 1.0, 26.0, 12.0, 1.16 } };
        enginelab::ValveTrainState valveState;
        enginelab::ValveTrainResult valveResult;
        for (int step = 0; step < 100; ++step)
            valveResult = enginelab::ValveTrainModel::evaluate(cam, false, valveState,
                470.0, 6'000.0, 1.0, 0.001);
        require(valveResult.intakeAdvanceDegrees > 20.0 && valveResult.liftMultiplier > 1.10,
                "continuous VVT/VVL actuator must converge toward its calibrated operating point");
        require(std::abs(enginelab::valveFlowCoefficient(2.5, 0.6, cam.intakeFlowCurve) - 0.26) < 1.0e-9,
                "valve discharge coefficient must interpolate the configured lift curve");

        // A sparse phaser table is a schedule and must be followed as one. The
        // shape that matters is a peak with a fall-back above it -- every real
        // cam phaser retards again at high speed -- and it is exactly the shape
        // an isotropic distance weighting destroys, because the peak sample
        // keeps pulling on queries far above it. Asserted on the authored
        // points (which must be reproduced exactly), on the midpoints (which
        // must be the linear blend), and on the fall-back leg (which must
        // actually descend).
        enginelab::ValveControlConfig schedule;
        schedule.enabled = true;
        schedule.samples = { { 1'000.0, 0.2, 2.0, 0.0, 0.90 },
                             { 3'000.0, 1.0, 22.0, 0.0, 1.00 },
                             { 8'000.0, 1.0, 6.0, 0.0, 1.00 } };
        const auto scheduleAt = [&schedule](double rpm) {
            return enginelab::interpolateValveControl(schedule, rpm, 1.0).intakeAdvanceDegrees;
        };
        require(std::abs(scheduleAt(3'000.0) - 22.0) < 1.0e-9
                    && std::abs(scheduleAt(8'000.0) - 6.0) < 1.0e-9,
                "an authored phaser point must be reproduced exactly at its own rpm");
        require(std::abs(scheduleAt(2'000.0) - 12.0) < 1.0e-9,
                "between two authored points the phaser must interpolate linearly in rpm");
        require(std::abs(scheduleAt(5'500.0) - 14.0) < 1.0e-9,
                "the fall-back leg must descend linearly, not be dragged up by the peak sample");
        require(scheduleAt(7'000.0) < scheduleAt(5'000.0)
                    && scheduleAt(5'000.0) < scheduleAt(3'500.0),
                "phaser advance must fall monotonically above the scheduled peak");
        require(std::abs(scheduleAt(12'000.0) - 6.0) < 1.0e-9
                    && std::abs(scheduleAt(200.0) - 2.0) < 1.0e-9,
                "queries outside the table must clamp to its ends, never extrapolate");

        enginelab::RunnerAcousticsConfig acoustics;
        enginelab::IntakeConfig intake;
        enginelab::CylinderConfig shortRunner;
        shortRunner.intakeRunnerLengthMm = 180.0;
        auto longRunner = shortRunner;
        longRunner.intakeRunnerLengthMm = 720.0;
        enginelab::HelmholtzRunnerState shortState;
        enginelab::HelmholtzRunnerState longState;
        const auto shortResult = enginelab::HelmholtzRunnerModel::advance(acoustics, shortState,
            shortRunner, intake, 300.0, 100.0, 80.0, 0.001);
        const auto longResult = enginelab::HelmholtzRunnerModel::advance(acoustics, longState,
            longRunner, intake, 300.0, 100.0, 80.0, 0.001);
        require(shortResult.resonanceFrequencyHz > longResult.resonanceFrequencyHz
                && std::isfinite(shortResult.flowAdmittance),
                "Helmholtz frequency must respond physically to runner length");
    }
    {
        const auto drivelineConfig = enginelab::makeDefaultInlineFour();
        enginelab::DrivelineModel driveline(drivelineConfig);
        enginelab::EngineState engine;
        engine.rpm = 2'000.0;
        engine.angularVelocityRadPerSecond = engine.rpm * 2.0 * std::numbers::pi / 60.0;
        engine.throttle = 0.5;
        driveline.requestGear(-2);
        enginelab::DrivelineOutput output;
        for (int step = 0; step < 300; ++step)
            output = driveline.advance(0.001, engine, 0.0, step < 220 ? 0.0 : 1.0, 0.0);
        require(output.engagedGear == -2 && output.vehicleSpeedMps < 0.0
                && output.clutchDissipatedEnergyJoules > 0.0,
                "reverse gear must drive backward through the same energy-coupled clutch model");
        const auto speedBeforeBrake = std::abs(output.vehicleSpeedMps);
        for (int step = 0; step < 500; ++step)
            output = driveline.advance(0.001, engine, 0.0, 0.0, 1.0);
        require(std::abs(output.vehicleSpeedMps) < speedBeforeBrake && output.brakeForceN > 0.0,
                "wheel brake must dissipate speed without applying an artificial engine load");

        enginelab::DrivelineModel coarseStepDriveline(drivelineConfig);
        coarseStepDriveline.requestGear(0);
        for (int step = 0; step < 40; ++step)
            output = coarseStepDriveline.advance(0.05, engine, 0.0, 1.0, step > 30 ? 0.4 : 0.0);
        require(std::isfinite(output.vehicleSpeedMps) && std::abs(output.vehicleSpeedMps) < 100.0
                && std::isfinite(output.clutchTemperatureC)
                && std::isfinite(output.energyResidualJoules),
                "stiff tire and clutch coupling must remain finite at the maximum public time step");

        enginelab::DrivelineModel unloadedDriveline(drivelineConfig);
        enginelab::DrivelineModel loadedDriveline(drivelineConfig);
        unloadedDriveline.requestGear(0);
        loadedDriveline.requestGear(0);
        enginelab::DrivelineOutput unloadedOutput;
        enginelab::DrivelineOutput loadedOutput;
        for (int step = 0; step < 1'000; ++step) {
            unloadedOutput = unloadedDriveline.advance(0.001, engine, 0.0, 1.0, 0.0);
            loadedOutput = loadedDriveline.advance(0.001, engine, 0.15, 1.0, 0.0);
        }
        // With rigid rolling contact and a fixed external engine speed, both
        // drivelines must settle at the same kinematic road speed.  The manual
        // retarder is observable as additional crank reaction, not artificial
        // tyre slip or a lower speed at the same locked ratio.
        require(loadedOutput.roadLoadForceN > 0.0
                && loadedOutput.engineCouplingTorqueNm
                    < unloadedOutput.engineCouplingTorqueNm - 1.0
                && std::abs(loadedOutput.vehicleSpeedMps
                    - unloadedOutput.vehicleSpeedMps) < 0.05
                && std::isfinite(loadedOutput.energyResidualJoules),
                "manual vehicle load must reach the crank through the rigid driveline energy path");

        // A fully engaged dry clutch must lock and carry the crank's torque at a few
        // rpm of slip, not slip endlessly like a fluid coupling. Close the loop with a
        // single-DOF crank flywheel driven by a constant torque, exactly the way the
        // runtime couples the simulator to the driveline, and let it settle in gear.
        {
            const auto lockConfig = enginelab::makeDefaultInlineFour();
            enginelab::DrivelineModel lockDriveline(lockConfig);
            lockDriveline.requestGear(3);
            const auto crankInertia = enginelab::effectiveRotatingInertiaKgM2(lockConfig);
            const auto rpmToRad = [](double rpm) { return rpm * 2.0 * std::numbers::pi / 60.0; };
            constexpr double crankTorqueNm = 150.0;
            constexpr double lockDt = 1.0 / 200.0;
            double crankOmega = rpmToRad(2'600.0);
            enginelab::DrivelineOutput lockOutput;
            for (int step = 0; step < 3'000; ++step) {
                enginelab::EngineState crank;
                crank.angularVelocityRadPerSecond = crankOmega;
                crank.rpm = crankOmega * 60.0 / (2.0 * std::numbers::pi);
                crank.throttle = 1.0;
                crank.torqueNm = crankTorqueNm;
                lockOutput = lockDriveline.advance(lockDt, crank, 0.0, 1.0, 0.0);
                crankOmega += (crankTorqueNm + lockOutput.engineReactionTorqueNm) / crankInertia * lockDt;
                crankOmega = std::max(crankOmega, rpmToRad(700.0));
            }
            require(std::abs(lockOutput.clutchSlipRpm) < 2.0 * lockConfig.transmission.clutchLockSpeedRpm,
                    "an engaged clutch must lock to within its lock-speed band, not slip by hundreds of rpm");
            require(std::abs(lockOutput.clutchTorqueNm - crankTorqueNm) < 0.2 * crankTorqueNm,
                    "a locked clutch must carry the crank torque, not a fraction of it");
            require(lockOutput.clutchPowerLossKw < 1.0,
                    "a locked clutch must stop dissipating, unlike a permanently slipping coupling");
        }

        // Every catalogue engine, driven in gear with its own matched inertia,
        // clutch capacity and gearing, must settle to a locked cruise without
        // chatter at the coarsest shipping coupling cadence (timeScale 4x ->
        // 1/60 s public step, the frame rate at which the crank<->clutch reaction
        // is applied). Measured steady-state coupling ripple is <1 Nm here; a
        // lagged-coupling limit cycle would swing hundreds to thousands of Nm.
        // This guards the whole catalogue against a future coupling regression.
        {
            const auto rpmToRad2 = [](double rpm) { return rpm * 2.0 * std::numbers::pi / 60.0; };
            constexpr double crankTorqueNm = 150.0;
            constexpr double coarseDt = 1.0 / 60.0;
            for (const auto& preset : enginelab::makeBaseEnginePresets()) {
                const auto crankInertia = enginelab::effectiveRotatingInertiaKgM2(preset);
                const auto lockBandRpm = std::max(1.0, preset.transmission.clutchLockSpeedRpm);
                enginelab::DrivelineModel chatterDriveline(preset);
                chatterDriveline.requestGear(3);
                double crankOmega = rpmToRad2(3'000.0);
                enginelab::DrivelineOutput out;
                double ssMinTorque = 1.0e9, ssMaxTorque = -1.0e9;
                for (int step = 0; step < 4'000; ++step) {
                    enginelab::EngineState crank;
                    crank.angularVelocityRadPerSecond = crankOmega;
                    crank.rpm = crankOmega * 60.0 / (2.0 * std::numbers::pi);
                    crank.throttle = 1.0;
                    crank.torqueNm = crankTorqueNm;
                    out = chatterDriveline.advance(coarseDt, crank, 0.0, 1.0, 0.0);
                    crankOmega = std::max(rpmToRad2(700.0),
                        crankOmega + (crankTorqueNm + out.engineReactionTorqueNm) / crankInertia * coarseDt);
                    require(std::isfinite(out.clutchTorqueNm) && std::isfinite(crankOmega),
                            "engaged-clutch coupling must stay finite for every catalogue inertia");
                    if (step >= 3'000) {
                        ssMinTorque = std::min(ssMinTorque, out.clutchTorqueNm);
                        ssMaxTorque = std::max(ssMaxTorque, out.clutchTorqueNm);
                    }
                }
                // Fixed coupling settles to <=~240 Nm peak-to-peak; the pre-fix limit
                // cycle swung >2000 Nm (+/- capacity). 0.4*capacity discriminates with margin.
                require(ssMaxTorque - ssMinTorque < 25.0,
                        "engaged clutch must not chatter (bounded steady-state coupling torque)");
                require(std::abs(out.clutchSlipRpm) < 3.0 * lockBandRpm,
                        "engaged clutch must settle within its lock band, not slip by hundreds of rpm");
            }
        }
    }
    {
        enginelab::MonotonicPublicationTimeline timeline;
        constexpr double frameDuration = 1.0 / 240.0;
        const auto first = timeline.beginWindow(0.0, frameDuration);
        const auto late = timeline.beginWindow(0.006, frameDuration);
        const auto catchingUp = timeline.beginWindow(0.008, frameDuration);
        require(first.endSeconds() <= late.startSeconds
                && late.endSeconds() <= catchingUp.startSeconds,
                "catch-up publication windows must never overlap");
        const auto previousLastTimestamp = late.mapSimulationTime(2.0, 1.0, 1.0);
        const auto nextFirstTimestamp = catchingUp.mapSimulationTime(2.0, 2.0, 1.0);
        require(previousLastTimestamp <= nextFirstTimestamp,
                "successive realtime event timestamps must remain monotonic after a late frame");
    }
    {
        enginelab::RealtimeLoadGovernor governor;
        for (std::size_t index = 0;
             index + 1 < enginelab::RealtimeLoadGovernor::missesToEngage;
             ++index) {
            require(!governor.observe(true, 1.05) && !governor.active(),
                "isolated realtime misses must not lower simulation quality");
        }
        require(!governor.observe(false, 0.70) && !governor.active(),
            "one comfortable frame must clear the overload streak");
        for (std::size_t index = 0;
             index + 1 < enginelab::RealtimeLoadGovernor::missesToEngage;
             ++index) {
            require(!governor.observe(true, 1.05),
                "the overload guard must wait for its complete engagement window");
        }
        require(governor.observe(true, 1.05) && governor.active(),
            "a sustained deadline deficit must engage realtime protection");

        for (std::size_t index = 0;
             index + 1 < enginelab::RealtimeLoadGovernor::comfortableFramesToRecover;
             ++index) {
            require(!governor.observe(false, 0.70) && governor.active(),
                "realtime protection must not chatter during recovery");
        }
        require(!governor.observe(
                    false, std::numeric_limits<double>::quiet_NaN())
                && governor.active(),
            "invalid timing data must never count as spare realtime capacity");
        for (std::size_t index = 0;
             index + 1 < enginelab::RealtimeLoadGovernor::comfortableFramesToRecover;
             ++index) {
            require(!governor.observe(false, 0.70),
                "recovery must require a new complete comfortable window");
        }
        require(governor.observe(false, 0.70) && !governor.active(),
            "sustained spare capacity must restore the normal thermal cadence");
    }
    {
        auto topology = enginelab::makeDefaultInlineFour();
        topology.intake.plenumVolumeLitres = 4.75;
        topology.intake.throttleDiameterMm = 68.0;
        topology.intake.throttleCount = 2;
        topology.rotatingInertiaKgM2 = 0.41;
        enginelab::normaliseEngineConfig(topology);
        require(topology.plenumVolumeLitres == 4.75 && topology.throttleDiameterMm == 68.0
                && topology.intakePaths.front().geometry.plenumVolumeLitres == 4.75
                && topology.intakePaths.front().geometry.throttleCount == 2,
                "normalisation must keep one canonical intake representation");
        require(enginelab::effectiveRotatingInertiaKgM2(topology) > 0.41,
                "effective inertia must include the migrated crankshaft and rotating connecting-rod mass");
        const auto expectedRunnerLitres = std::numbers::pi
            * std::pow(topology.intake.runnerDiameterMm * 0.5, 2.0)
            * topology.intake.runnerLengthMm / 1'000'000.0;
        require(std::abs(enginelab::intakeRunnerVolumeLitres(topology.cylinders.front(), topology.intake)
            - expectedRunnerLitres) < 1.0e-12,
            "runner control-volume size must come from configured length and diameter");
        topology.intake.runnerPlenumDiameterMm = 52.0;
        const auto inletRadiusMm = topology.intake.runnerDiameterMm * 0.5;
        const auto outletRadiusMm = topology.intake.runnerPlenumDiameterMm * 0.5;
        const auto expectedTaperedRunnerLitres = std::numbers::pi
            * topology.intake.runnerLengthMm / 3.0
            * (inletRadiusMm * inletRadiusMm
                + inletRadiusMm * outletRadiusMm
                + outletRadiusMm * outletRadiusMm) / 1'000'000.0;
        require(std::abs(enginelab::intakeRunnerVolumeLitres(
                    topology.cylinders.front(), topology.intake)
                - expectedTaperedRunnerLitres) < 1.0e-12,
            "a tapered intake runner must use exact conical-frustum volume");
        const auto tdc = enginelab::evaluateCylinderKinematics(topology, 0, 0.0, 100.0);
        const auto bdc = enginelab::evaluateCylinderKinematics(topology, 0, 180.0, 100.0);
        require(std::abs(tdc.pistonTravelMm) < 0.01
                && std::abs(bdc.pistonTravelMm - topology.cylinders.front().strokeMm) < 0.05,
                "shared mechanical kinematics must resolve configured TDC and stroke");
        require(tdc.connectingRodObliquityDegrees < 0.01
                && bdc.connectingRodObliquityDegrees < 0.01,
                "an aligned connecting rod must produce zero lateral-thrust angle at dead centres");
        auto sharedPinVtwin = enginelab::makeDefaultInlineTwo();
        sharedPinVtwin.layout = enginelab::EngineLayout::vLayout;
        sharedPinVtwin.bankAngleDegrees = 45.0;
        sharedPinVtwin.crankJournals = { { 1, 0.0,
            sharedPinVtwin.cylinders.front().strokeMm * 0.5, 1 } };
        sharedPinVtwin.banks = {
            { 1, -22.5, { 1 }, sharedPinVtwin.camshafts, 1, 1 },
            { 2, 22.5, { 2 }, sharedPinVtwin.camshafts, 1, 1 } };
        sharedPinVtwin.cylinders[0].crankJournalId = 1;
        sharedPinVtwin.cylinders[0].bankId = 1;
        sharedPinVtwin.cylinders[1].crankJournalId = 1;
        sharedPinVtwin.cylinders[1].bankId = 2;
        const auto sharedLeft = enginelab::evaluateCylinderKinematics(sharedPinVtwin, 0, 123.0, 100.0);
        const auto sharedRight = enginelab::evaluateCylinderKinematics(sharedPinVtwin, 1, 123.0, 100.0);
        require(std::abs(sharedLeft.crankPinXMm - sharedRight.crankPinXMm) < 1.0e-9
                && std::abs(sharedLeft.crankPinYMm - sharedRight.crankPinYMm) < 1.0e-9,
                "cylinders sharing a V-engine journal must resolve one global crankpin position");
        auto invalidThrow = topology;
        invalidThrow.crankJournals.front().throwMm += 2.0;
        require(enginelab::validateEngineConfig(invalidThrow).has_value(),
                "journal throw inconsistent with stroke must be rejected");
        const auto radial = enginelab::makeDefaultRadialFive();
        require(!enginelab::validateEngineConfig(radial), "master/articulated radial topology must validate");
        const auto articulated = enginelab::evaluateCylinderKinematics(radial, 1, 97.0, 140.0);
        require(std::isfinite(articulated.pistonPositionMm) && articulated.crankJournalId == 1,
                "articulated-rod geometry must produce finite shared simulation/render state");
    }
    {
        enginelab::GasCell intake;
        enginelab::GasCell cylinder;
        enginelab::GasCell exhaust;
        intake.initialise(135.0, 0.35, 320.0);
        cylinder.initialise(105.0, 0.055, 520.0);
        exhaust.initialise(78.0, 0.40, 600.0);
        const auto molesBefore = intake.totalMoles() + cylinder.totalMoles() + exhaust.totalMoles();
        const auto energyBefore = intake.internalEnergyJoules() + cylinder.internalEnergyJoules()
            + exhaust.internalEnergyJoules() + intake.bulkKineticEnergyJoules()
            + cylinder.bulkKineticEnergyJoules() + exhaust.bulkKineticEnergyJoules();
        const auto overlap = enginelab::ConservativeGasSystem::flowSimultaneous(
            { &intake, &cylinder, 0.00012, 0.68, 0.0001, 0.0, 1.0 },
            { &cylinder, &exhaust, 0.00010, 0.66, 0.0001, 0.0, -1.0 });
        const auto molesAfter = intake.totalMoles() + cylinder.totalMoles() + exhaust.totalMoles();
        const auto energyAfter = intake.internalEnergyJoules() + cylinder.internalEnergyJoules()
            + exhaust.internalEnergyJoules() + intake.bulkKineticEnergyJoules()
            + cylinder.bulkKineticEnergyJoules() + exhaust.bulkKineticEnergyJoules();
        require(overlap.first.transferredMassKg > 0.0 && overlap.second.transferredMassKg > 0.0,
                "simultaneous valve transaction must resolve both overlap gradients");
        require(std::abs(molesAfter - molesBefore) < 1.0e-10
                && std::abs(energyAfter - energyBefore) < 1.0e-5,
                "simultaneous valve transaction must conserve species and total energy");
    }
    {
        enginelab::EndGasKnockState coolState;
        enginelab::EndGasKnockState hotState;
        bool hotAutoIgnited = false;
        for (int step = 0; step < 300; ++step) {
            const auto cool = enginelab::EndGasKnockModel::advance(coolState,
                { 12.0, 650.0, 1.0, 0.25, 98.0, true }, 0.00005);
            const auto hot = enginelab::EndGasKnockModel::advance(hotState,
                { 72.0, 1'060.0, 1.0, 0.25, 90.0, true }, 0.00005);
            require(!cool.autoIgnited, "cool low-pressure end gas must not knock");
            hotAutoIgnited = hotAutoIgnited || hot.autoIgnited;
        }
        require(hotAutoIgnited && hotState.filteredLevel > 0.0,
                "hot high-pressure end gas must auto-ignite through the Livengood-Wu integral");
    }
    {
        enginelab::GasCell highPressure;
        enginelab::GasCell lowPressure;
        highPressure.initialise(220.0, 1.2, 340.0);
        lowPressure.initialise(85.0, 2.4, 295.0);
        const auto initialMoles = enginelab::ConservativeGasSystem::totalMoles(highPressure, lowPressure);
        const auto initialEnergy = enginelab::ConservativeGasSystem::totalEnergy(highPressure, lowPressure);
        const auto initialMomentum = enginelab::ConservativeGasSystem::totalMomentum(highPressure, lowPressure);
        double transferred = 0.0;
        for (int step = 0; step < 2'000; ++step)
            transferred += enginelab::ConservativeGasSystem::flow(
                highPressure, lowPressure, 8.0e-6, 0.72, 1.0 / 20'000.0).transferredMoles;
        require(transferred > 0.0, "gas network must flow from high to low pressure");
        require(std::abs(enginelab::ConservativeGasSystem::totalMoles(highPressure, lowPressure) - initialMoles)
                < initialMoles * 1.0e-10, "gas transfers must conserve moles");
        require(std::abs(enginelab::ConservativeGasSystem::totalEnergy(highPressure, lowPressure) - initialEnergy)
                < std::max(1.0, initialEnergy) * 1.0e-10, "gas transfers must conserve internal energy");
        require(enginelab::ConservativeGasSystem::totalMomentum(highPressure, lowPressure) > initialMomentum,
                "the pressure gradient must accelerate the gas toward the low-pressure volume");
        require(highPressure.pressureKpa() < 220.0 && lowPressure.pressureKpa() > 85.0,
                "gas transfer must move both control volumes toward equilibrium");

        enginelab::GasCell boundaryInlet;
        boundaryInlet.initialise(80.0, 1.0, 295.0);
        const auto inletPressureBefore = boundaryInlet.pressureKpa();
        const auto boundaryInflow = enginelab::ConservativeGasSystem::flowFromBoundary(
            boundaryInlet, 101.325, 295.0, 1.0e-5, 0.72, 1.0 / 1'000.0);
        require(boundaryInflow.transferredMassKg < 0.0
                && boundaryInlet.pressureKpa() > inletPressureBefore,
                "boundary inflow must carry a negative target-to-boundary sign and raise target pressure");
        enginelab::GasCell boundaryOutlet;
        boundaryOutlet.initialise(130.0, 1.0, 320.0);
        const auto outletPressureBefore = boundaryOutlet.pressureKpa();
        const auto boundaryOutflow = enginelab::ConservativeGasSystem::flowFromBoundary(
            boundaryOutlet, 101.325, 295.0, 1.0e-5, 0.72, 1.0 / 1'000.0);
        require(boundaryOutflow.transferredMassKg > 0.0
                && boundaryOutlet.pressureKpa() < outletPressureBefore,
                "boundary outflow must carry a positive target-to-boundary sign and lower target pressure");

        const auto pressureBeforeCompression = lowPressure.pressureKpa();
        const auto energyBeforeCompression = lowPressure.internalEnergyJoules();
        lowPressure.setVolumeAdiabatic(lowPressure.volumeLitres() * 0.5);
        require(lowPressure.pressureKpa() > pressureBeforeCompression * 1.8,
                "adiabatic compression must increase pressure");
        require(lowPressure.internalEnergyJoules() > energyBeforeCompression,
                "compression work must increase gas internal energy");

        enginelab::GasCell combustible;
        combustible.initialise(100.0, 0.5, 330.0);
        combustible.injectFuelMoles(combustible.mixture().oxygenMoles / 12.5);
        const auto temperatureBefore = combustible.temperatureK();
        const auto massBeforeReaction = combustible.massKg();
        const auto reaction = enginelab::ConservativeGasSystem::reactGasoline(combustible, 0.5, 0.72);
        require(reaction.burnedFuelMoles > 0.0 && reaction.releasedEnergyJoules > 0.0,
                "gasoline reaction must consume mixture and release heat");
        require(combustible.temperatureK() > temperatureBefore,
                "combustion heat must raise cylinder temperature");
        require(std::abs(combustible.massKg() - massBeforeReaction) < massBeforeReaction * 1.0e-12,
                "global gasoline reaction must conserve reactant mass");

        enginelab::GasCell completeChemistry;
        completeChemistry.initialise(100.0, 0.5, 330.0);
        completeChemistry.injectFuelMoles(completeChemistry.mixture().oxygenMoles / 12.5);
        auto incompleteChemistry = completeChemistry;
        const auto requestedFuel = completeChemistry.mixture().fuelMoles * 0.4;
        const auto completeReaction = enginelab::ConservativeGasSystem::reactFuelMoles(
            completeChemistry, requestedFuel, 1.0, 44'000'000.0);
        const auto incompleteReaction = enginelab::ConservativeGasSystem::reactFuelMoles(
            incompleteChemistry, requestedFuel, 0.5, 44'000'000.0);
        require(std::abs(incompleteReaction.burnedFuelMoles
                    - 0.5 * completeReaction.burnedFuelMoles) < requestedFuel * 1.0e-12
                && std::abs(incompleteReaction.releasedEnergyJoules
                    - 0.5 * completeReaction.releasedEnergyJoules)
                    < completeReaction.releasedEnergyJoules * 1.0e-12
                && incompleteChemistry.mixture().fuelMoles > completeChemistry.mixture().fuelMoles,
                "combustion efficiency must leave unreacted fuel instead of deleting its chemical energy");

        enginelab::GasCell calibratedFuel;
        calibratedFuel.configureFuelChemistry(0.100, 10.0, 14.0);
        calibratedFuel.initialise(100.0, 0.5, 330.0);
        calibratedFuel.injectFuelMoles(calibratedFuel.mixture().oxygenMoles / 10.0);
        const auto calibratedMass = calibratedFuel.massKg();
        const auto calibratedReaction = enginelab::ConservativeGasSystem::reactFuel(
            calibratedFuel, 1.0, 0.72, 50'000'000.0);
        require(calibratedReaction.releasedEnergyJoules > 0.0
                && std::abs(calibratedFuel.massKg() - calibratedMass) < calibratedMass * 1.0e-12,
                "configured fuel chemistry must conserve mass and use its calibrated heating value");

        enginelab::GasCell vectorSource;
        enginelab::GasCell vectorSink;
        vectorSource.initialise(180.0, 1.0, 330.0);
        vectorSink.initialise(90.0, 1.5, 295.0);
        vectorSource.setGeometry(8.0e-5, 1.0, 0.0);
        vectorSink.setGeometry(8.0e-5, 0.0, 1.0);
        vectorSource.setBulkVelocityMps(45.0, 18.0);
        const auto pxBefore = enginelab::ConservativeGasSystem::totalMomentumX(vectorSource, vectorSink);
        const auto pyBefore = enginelab::ConservativeGasSystem::totalMomentumY(vectorSource, vectorSink);
        const auto vectorEnergyBefore = enginelab::ConservativeGasSystem::totalEnergy(vectorSource, vectorSink);
        (void)enginelab::ConservativeGasSystem::flow({ &vectorSource, &vectorSink,
            1.2e-5, 0.76, 1.0 / 10'000.0, 0.6, 0.8, 8.0e-5, 8.0e-5 });
        require(enginelab::ConservativeGasSystem::totalMomentumX(vectorSource, vectorSink) > pxBefore
                && enginelab::ConservativeGasSystem::totalMomentumY(vectorSource, vectorSink) > pyBefore,
                "duct pressure forces must accelerate both gas volumes in the configured flow direction");
        require(std::abs(enginelab::ConservativeGasSystem::totalEnergy(vectorSource, vectorSink) - vectorEnergyBefore)
                    < std::max(1.0, vectorEnergyBefore) * 1.0e-8,
                "directional gas flow must conserve internal plus bulk kinetic energy");

        enginelab::GasCell burnedProducts;
        burnedProducts.initialise(300.0, 0.5, 1'600.0, { 0.0, 0.01, 0.0, 0.09 });
        require(burnedProducts.heatCapacityRatioEffective() < 1.30,
                "burned products must not retain the cold-air heat-capacity ratio");
    }

    {
        enginelab::FuelConfig fuel;
        enginelab::FlamePhysicsModel flame;
        const auto coldSpeed = enginelab::FlamePhysicsModel::laminarFlameSpeedMps(fuel, 1.05, 320.0, 101'325.0);
        const auto hotSpeed = enginelab::FlamePhysicsModel::laminarFlameSpeedMps(fuel, 1.05, 650.0, 101'325.0);
        const auto leanSpeed = enginelab::FlamePhysicsModel::laminarFlameSpeedMps(fuel, 0.55, 650.0, 101'325.0);
        require(hotSpeed > coldSpeed && leanSpeed < hotSpeed,
                "Metghalchi-Keck flame speed must respond to temperature and mixture strength");
        enginelab::FlameConditions cleanConditions { 0.086, 0.000055, 650.0, 500'000.0,
                                                      1.05, 0.0, 0.0, 0.5 };
        auto dilutedConditions = cleanConditions;
        dilutedConditions.burnedGasFraction = 0.5;
        require(enginelab::FlamePhysicsModel::turbulentFlameSpeedMps(fuel, dilutedConditions)
                    < enginelab::FlamePhysicsModel::turbulentFlameSpeedMps(fuel, cleanConditions),
                "residual-gas dilution must attenuate flame speed even at low turbulence");
        auto tumbleConditions = cleanConditions;
        tumbleConditions.meanPistonSpeedMps = 8.0;
        tumbleConditions.chamberTurbulenceIntensityRatio = 1.6;
        require(enginelab::FlamePhysicsModel::turbulentFlameSpeedMps(
                    fuel, tumbleConditions)
                    > enginelab::FlamePhysicsModel::turbulentFlameSpeedMps(
                        fuel, cleanConditions),
                "chamber tumble must increase turbulent flame speed independently of fuel chemistry");
        auto highSpeedPentRoof = cleanConditions;
        highSpeedPentRoof.temperatureK = 650.0;
        highSpeedPentRoof.meanPistonSpeedMps = 21.0;
        highSpeedPentRoof.chamberTurbulenceIntensityRatio = 1.55;
        highSpeedPentRoof.load = 1.0;
        const auto highSpeedFlame =
            enginelab::FlamePhysicsModel::turbulentFlameSpeedMps(
                fuel, highSpeedPentRoof);
        require(highSpeedFlame > 42.0 && highSpeedFlame < 0.18
                    * std::sqrt(1.35 * 287.05
                        * highSpeedPentRoof.temperatureK) + 1.0e-12,
                "high-speed pent-roof combustion must retain calibrated tumble "
                "while remaining a subsonic deflagration");
        enginelab::FlameEvent event;
        enginelab::FlameConditions conditions { 0.086, 0.000055, 700.0, 900'000.0,
                                                 1.05, 0.04, 12.0, 0.9 };
        auto dualIgnitionConditions = conditions;
        dualIgnitionConditions.ignitionSiteCount = 2;
        enginelab::FlameEvent singleKernel;
        enginelab::FlameEvent dualKernel;
        flame.ignite(singleKernel, fuel, conditions, 1.0e-5);
        flame.ignite(dualKernel, fuel, dualIgnitionConditions, 1.0e-5);
        for (int step = 0; step < 40; ++step) {
            (void)flame.advance(singleKernel, fuel, conditions, 1.0 / 40'000.0);
            (void)flame.advance(dualKernel, fuel, dualIgnitionConditions, 1.0 / 40'000.0);
        }
        require(dualKernel.burnedFraction > singleKernel.burnedFraction,
                "two ignition kernels must consume more chamber volume at equal flame speed and time");
        flame.ignite(event, fuel, conditions, 1.0e-5);
        double accumulated = 0.0;
        for (int step = 0; step < 2'000 && event.active; ++step)
            accumulated += flame.advance(event, fuel, conditions, 1.0 / 40'000.0).burnedFractionAdvance;
        require(accumulated > 0.99 && event.burnedFraction == 1.0,
                "ellipsoidal flame propagation must monotonically consume the chamber volume");

        enginelab::GasCell directCell;
        directCell.initialise(900.0, 0.055, 650.0);
        enginelab::InjectionConfig direct;
        direct.mode = enginelab::InjectionMode::direct;
        enginelab::FuelInjectionState directState;
        const auto directTemperatureBefore = directCell.temperatureK();
        const auto directResult = enginelab::FuelInjectionModel::deliver(direct, fuel, directState,
            directCell, 2.0e-5, 0.002);
        require(directResult.meteredMoles > 0.0 && directResult.chargeCoolingJoules > 0.0
                && directResult.entrainedMoles == directResult.vaporisedMoles
                && directCell.temperatureK() < directTemperatureBefore,
                "direct injection must be pressure-limited and apply latent charge cooling");

        auto dispersedDirect = direct;
        dispersedDirect.directSprayVaporisationTimeConstantSeconds = 0.004;
        dispersedDirect.directSprayEntrainmentTimeConstantSeconds = 0.0008;
        enginelab::GasCell dispersedCell;
        dispersedCell.initialise(900.0, 0.055, 650.0);
        enginelab::FuelInjectionState dispersedState;
        const auto fuelBefore = dispersedCell.mixture().fuelMoles;
        const auto dispersed = enginelab::FuelInjectionModel::deliver(
            dispersedDirect, fuel, dispersedState, dispersedCell, 2.0e-5, 0.0001);
        const auto representedFuel = dispersedCell.mixture().fuelMoles - fuelBefore
            + dispersedState.directLiquidSprayMoles
            + dispersedState.directDispersingVapourMoles;
        require(dispersed.meteredMoles > 0.0
                && dispersed.vaporisedMoles < dispersed.meteredMoles
                && dispersed.entrainedMoles < dispersed.vaporisedMoles
                && std::abs(representedFuel - dispersed.meteredMoles) < 1.0e-15,
                "direct spray must delay homogeneous fuel availability while conserving every metered mole");

        enginelab::GasCell coldSprayCell;
        enginelab::GasCell hotSprayCell;
        coldSprayCell.initialise(900.0, 0.055, 450.0);
        hotSprayCell.initialise(900.0, 0.055, 950.0);
        enginelab::FuelInjectionState coldSprayState;
        enginelab::FuelInjectionState hotSprayState;
        const auto coldSpray = enginelab::FuelInjectionModel::deliver(
            dispersedDirect, fuel, coldSprayState, coldSprayCell, 2.0e-5, 0.0001);
        const auto hotSpray = enginelab::FuelInjectionModel::deliver(
            dispersedDirect, fuel, hotSprayState, hotSprayCell, 2.0e-5, 0.0001);
        require(hotSpray.vaporisedMoles > coldSpray.vaporisedMoles,
                "direct-spray vaporisation must accelerate in a hotter compressed charge");

        enginelab::GasCell portCell;
        portCell.initialise(80.0, 0.18, 320.0);
        auto port = direct;
        port.mode = enginelab::InjectionMode::port;
        port.railPressureBar = port.referencePressureBar = 4.0;
        port.injectorFlowMgPerSecond = 100.0;
        port.wallFilmFraction = 0.7;
        port.vaporisationTimeConstantSeconds = 0.08;
        enginelab::FuelInjectionState portState;
        const auto firstPort = enginelab::FuelInjectionModel::deliver(port, fuel, portState,
            portCell, 2.0e-5, 0.001);
        const auto filmAfterInjection = portState.liquidFilmMoles;
        const auto laterPort = enginelab::FuelInjectionModel::deliver(port, fuel, portState,
            portCell, 0.0, 0.02);
        require(firstPort.meteredMoles > firstPort.vaporisedMoles && filmAfterInjection > 0.0
                && laterPort.vaporisedMoles > 0.0 && portState.liquidFilmMoles < filmAfterInjection,
            "port injection must retain and subsequently evaporate a wall film");

        enginelab::FuelInjectionState fractionalPulseState;
        enginelab::GasCell fractionalPulseCell;
        fractionalPulseCell.initialise(80.0, 0.18, 320.0);
        const auto saturatedPulse = enginelab::FuelInjectionModel::deliver(
            port, fuel, fractionalPulseState, fractionalPulseCell, 1.0, 0.001);
        require(std::abs(saturatedPulse.openFraction - 1.0) < 1.0e-12,
            "an injector command beyond one sub-step capacity must report full opening");
        enginelab::FuelInjectionState halfPulseState;
        enginelab::GasCell halfPulseCell;
        halfPulseCell.initialise(80.0, 0.18, 320.0);
        const auto halfPulse = enginelab::FuelInjectionModel::deliver(
            port, fuel, halfPulseState, halfPulseCell,
            saturatedPulse.meteredMoles * 0.5, 0.001);
        require(std::abs(halfPulse.openFraction - 0.5) < 1.0e-12,
            "injector duty must retain a fractional final sub-step instead of rounding it up");

        enginelab::GasCell boostedPortCell;
        boostedPortCell.initialise(180.0, 0.18, 320.0);
        enginelab::FuelInjectionState boostedPortState;
        const auto boostedPort = enginelab::FuelInjectionModel::deliver(port, fuel, boostedPortState,
            boostedPortCell, 2.0e-5, 0.001);
        require(std::abs(boostedPort.meteredMoles - firstPort.meteredMoles) < 1.0e-12,
                "manifold-referenced port injection must retain its rated flow under boost");

        auto lowPressureDirect = direct;
        lowPressureDirect.railPressureBar = lowPressureDirect.referencePressureBar = 12.0;
        lowPressureDirect.injectorFlowMgPerSecond = 100.0;
        enginelab::GasCell lowCylinderPressureCell;
        lowCylinderPressureCell.initialise(100.0, 0.18, 320.0);
        enginelab::FuelInjectionState lowCylinderPressureState;
        const auto lowCylinderPressureDirect = enginelab::FuelInjectionModel::deliver(
            lowPressureDirect, fuel, lowCylinderPressureState, lowCylinderPressureCell, 1.0e-3, 0.001);
        enginelab::GasCell boostedDirectCell;
        boostedDirectCell.initialise(800.0, 0.18, 320.0);
        enginelab::FuelInjectionState lowPressureDirectState;
        const auto boostedDirect = enginelab::FuelInjectionModel::deliver(lowPressureDirect, fuel,
            lowPressureDirectState, boostedDirectCell, 1.0e-3, 0.001);
        require(boostedDirect.meteredMoles < 0.75 * lowCylinderPressureDirect.meteredMoles,
                "direct-injection flow must still fall as cylinder pressure approaches rail pressure");

        const auto delayedPortTrim = enginelab::FuelInjectionModel::updateClosedLoopTrim(
            port, 3'000.0, 18.0, 13.0, 1.0);
        const auto directTrim = enginelab::FuelInjectionModel::updateClosedLoopTrim(
            direct, 3'000.0, 18.0, 13.0, 1.0);
        require(delayedPortTrim > 1.0 && delayedPortTrim < directTrim && directTrim < 1.06,
                "closed-loop fuel control must respect port-film transport delay and bounded gain");
    }

    {
        enginelab::FuelConfig diesel;
        diesel.name = "C12H23 diesel surrogate";
        diesel.lowerHeatingValueMjPerKg = 42.6;
        diesel.molarMassGramsPerMole = 167.31;
        diesel.oxygenMolesPerFuelMole = 17.75;
        diesel.productMolesPerFuelMole = 23.5;
        diesel.stoichiometricAirFuelRatio = 14.65;
        diesel.cetaneNumber = 51.0;
        enginelab::CombustionCalibrationConfig calibration;

        const auto lowCetaneDelay =
            enginelab::CompressionIgnitionModel::ignitionDelaySeconds(
                calibration, 40.0, 45.0, 850.0, 0.75);
        const auto highCetaneDelay =
            enginelab::CompressionIgnitionModel::ignitionDelaySeconds(
                calibration, 60.0, 45.0, 850.0, 0.75);
        require(highCetaneDelay < lowCetaneDelay
                && highCetaneDelay > 20.0e-6,
                "higher cetane quality must shorten the finite compression-ignition delay");

        enginelab::GasCell hotChamber;
        hotChamber.configureFuelChemistry(
            diesel.molarMassGramsPerMole * 0.001,
            diesel.oxygenMolesPerFuelMole,
            diesel.productMolesPerFuelMole);
        hotChamber.initialise(5'000.0, 0.055, 850.0,
            { 0.002, 0.00752, 0.00008, 0.0 });
        enginelab::CompressionIgnitionState hotState;
        const auto energyBefore = hotChamber.internalEnergyJoules();
        double releasedEnergy = 0.0;
        double burnedFuel = 0.0;
        double usefulBurnDuration = 0.0;
        bool ignited = false;
        for (int step = 0; step < 400; ++step) {
            const auto result = enginelab::CompressionIgnitionModel::advance(
                hotState, hotChamber, diesel, calibration,
                { 700.0 + static_cast<double>(step) * 0.2, 9.0, true },
                25.0e-6);
            releasedEnergy += result.reaction.releasedEnergyJoules;
            burnedFuel += result.reaction.burnedFuelMoles;
            usefulBurnDuration = std::max(
                usefulBurnDuration, result.durationSeconds);
            ignited = ignited || result.autoIgnited;
        }
        require(ignited && burnedFuel > 0.0
                && std::abs((hotChamber.internalEnergyJoules() - energyBefore)
                    - releasedEnergy) < std::max(1.0, releasedEnergy) * 1.0e-12,
                "compression ignition must release exactly the chemical energy added to the conservative chamber");
        require(usefulBurnDuration > 0.0 && usefulBurnDuration < 0.008,
                "compression-ignition duration must exclude the negligible numerical burn tail");

        enginelab::GasCell coldChamber;
        coldChamber.configureFuelChemistry(
            diesel.molarMassGramsPerMole * 0.001,
            diesel.oxygenMolesPerFuelMole,
            diesel.productMolesPerFuelMole);
        coldChamber.initialise(300.0, 0.55, 360.0,
            { 0.002, 0.00752, 0.00008, 0.0 });
        enginelab::CompressionIgnitionState coldState;
        for (int step = 0; step < 400; ++step)
            (void)enginelab::CompressionIgnitionModel::advance(
                coldState, coldChamber, diesel, calibration,
                { 700.0 + static_cast<double>(step) * 0.2, 2.0, true },
                25.0e-6);
        require(!coldState.autoIgnited
                && coldChamber.mixture().fuelMoles > 0.0,
                "a cold low-pressure diesel mixture must not autoignite");

        enginelab::GasCell oxygenLimited;
        oxygenLimited.configureFuelChemistry(
            diesel.molarMassGramsPerMole * 0.001,
            diesel.oxygenMolesPerFuelMole,
            diesel.productMolesPerFuelMole);
        oxygenLimited.initialise(6'000.0, 0.055, 900.0,
            { 0.0002, 0.00075, 0.0002, 0.0 });
        const auto initialOxygen = oxygenLimited.mixture().oxygenMoles;
        enginelab::CompressionIgnitionState oxygenLimitedState;
        double oxygenLimitedBurn = 0.0;
        for (int step = 0; step < 1'000; ++step) {
            const auto result = enginelab::CompressionIgnitionModel::advance(
                oxygenLimitedState, oxygenLimited, diesel, calibration,
                { std::fmod(650.0 + static_cast<double>(step) * 0.2, 720.0),
                  10.0, true }, 25.0e-6);
            oxygenLimitedBurn += result.reaction.burnedFuelMoles;
        }
        require(oxygenLimitedBurn
                    <= initialOxygen / diesel.oxygenMolesPerFuelMole + 1.0e-15
                && oxygenLimited.mixture().oxygenMoles >= -1.0e-15,
                "compression ignition must remain strictly oxygen limited");
    }

    auto config = enginelab::makeDefaultInlineFour();

    {
        enginelab::SimpleEcuModel idleEcu;
        idleEcu.initialise(config);
        enginelab::EngineControls closedThrottle { true, false, 0.0, 0.0 };
        enginelab::EngineState belowIdle;
        belowIdle.simulationTimeSeconds = 0.1;
        belowIdle.rpm = config.idleRpm * 0.45;
        belowIdle.manifoldPressureKpa = 35.0;
        const auto recovery = idleEcu.evaluate(config, belowIdle, closedThrottle);
        require(recovery.effectiveThrottle == 0.0,
                "idle control must not hold the driver's throttle open");
        require(recovery.idleAirOpening > 0.5,
                "idle actuator must provide recovery air below the target speed");

        auto aboveIdle = belowIdle;
        aboveIdle.simulationTimeSeconds += 0.01;
        aboveIdle.rpm = config.idleRpm * 1.55;
        const auto overspeed = idleEcu.evaluate(config, aboveIdle, closedThrottle);
        require(overspeed.effectiveThrottle == 0.0 && overspeed.idleAirOpening < 0.05,
                "idle actuator must close when engine speed is above target");

        enginelab::EngineControls driverRev { true, false, 0.30, 0.0 };
        aboveIdle.throttle = driverRev.throttle;
        for (int step = 0; step < 300; ++step) {
            aboveIdle.simulationTimeSeconds += 0.01;
            (void)idleEcu.evaluate(config, aboveIdle, driverRev);
        }
        aboveIdle.simulationTimeSeconds += 0.01;
        aboveIdle.rpm = config.idleRpm;
        aboveIdle.throttle = 0.0;
        const auto liftOff = idleEcu.evaluate(config, aboveIdle, closedThrottle);
        require(liftOff.idleAirOpening > 0.35,
                "driver override must not wind the idle integral shut before lift-off");

        aboveIdle.rpm = config.idleRpm * 3.0;
        for (int step = 0; step < 600; ++step) {
            aboveIdle.simulationTimeSeconds += 0.01;
            (void)idleEcu.evaluate(config, aboveIdle, closedThrottle);
        }
        aboveIdle.simulationTimeSeconds += 0.01;
        aboveIdle.rpm = config.idleRpm;
        const auto decelerationCatch = idleEcu.evaluate(config, aboveIdle, closedThrottle);
        require(decelerationCatch.idleAirOpening > 0.20,
                "overspeed correction must preserve enough idle air to catch a falling engine");
    }

    {
        enginelab::SimpleEcuModel transientEcu;
        transientEcu.initialise(config);
        enginelab::EngineState transientState;
        transientState.simulationTimeSeconds = 1.0;
        transientState.rpm = config.idleRpm;
        transientState.coolantTemperatureC = 90.0;
        enginelab::EngineControls transientControls { true, false, 0.0, 0.0 };
        (void)transientEcu.evaluate(config, transientState, transientControls);
        transientState.simulationTimeSeconds += 0.01;
        transientControls.throttle = 0.40;
        const auto tipIn = transientEcu.evaluate(config, transientState, transientControls);
        require(tipIn.fuelCorrection > 1.8,
                "tip-in must command a solver-rate-independent transient fuel reserve");
        for (int step = 0; step < 300; ++step) {
            transientState.simulationTimeSeconds += 0.01;
            (void)transientEcu.evaluate(config, transientState, transientControls);
        }
        transientState.simulationTimeSeconds += 0.01;
        const auto settledThrottle = transientEcu.evaluate(
            config, transientState, transientControls);
        require(settledThrottle.fuelCorrection < 1.02,
                "acceleration fuel reserve must decay back to the steady-state map");

        transientState.simulationTimeSeconds += 0.01;
        transientState.rpm = config.idleRpm * 2.0;
        transientState.throttle = 0.0;
        transientControls.throttle = 0.0;
        const auto overrun = transientEcu.evaluate(
            config, transientState, transientControls);
        require(!overrun.fuelEnabled && overrun.sparkEnabled,
                "closed-throttle overrun must cut fuel without disabling ignition");

        transientState.simulationTimeSeconds += 0.01;
        transientState.rpm = config.idleRpm * 1.10;
        const auto fuelResume = transientEcu.evaluate(
            config, transientState, transientControls);
        require(fuelResume.fuelEnabled && fuelResume.fuelCorrection < 0.10,
                "deceleration fuel cut must release before the idle catch region");
        for (int step = 0; step < 40; ++step) {
            transientState.simulationTimeSeconds += 0.01;
            (void)transientEcu.evaluate(config, transientState, transientControls);
        }
        transientState.simulationTimeSeconds += 0.01;
        const auto resumedFuel = transientEcu.evaluate(
            config, transientState, transientControls);
        require(resumedFuel.fuelCorrection > 0.98,
                "post-overrun fuel ramp must return to the steady-state command");
    }

    {
        auto overrunAfterfireConfig = config;
        overrunAfterfireConfig.exhaustAfterfire.enabled = true;
        overrunAfterfireConfig.exhaustAfterfire.overrunFuelFraction = 0.12;
        overrunAfterfireConfig.exhaustAfterfire.overrunMinimumRpm = 3'000.0;
        overrunAfterfireConfig.exhaustAfterfire.overrunMaximumThrottle = 0.02;
        enginelab::SimpleEcuModel overrunAfterfireEcu;
        overrunAfterfireEcu.initialise(overrunAfterfireConfig);
        enginelab::EngineState overrunState;
        overrunState.simulationTimeSeconds = 1.0;
        overrunState.rpm = 300.0;
        overrunState.coolantTemperatureC = 22.0;
        enginelab::EngineControls cranking { true, true, 0.0, 0.0 };
        (void)overrunAfterfireEcu.evaluate(
            overrunAfterfireConfig, overrunState, cranking);

        overrunState.simulationTimeSeconds += 0.01;
        overrunState.rpm = 3'500.0;
        overrunState.throttle = 0.50;
        enginelab::EngineControls powerRequest { true, false, 0.50, 0.0 };
        (void)overrunAfterfireEcu.evaluate(
            overrunAfterfireConfig, overrunState, powerRequest);

        overrunState.simulationTimeSeconds += 0.01;
        overrunState.throttle = 0.0;
        enginelab::EngineControls closedThrottle { true, false, 0.0, 0.0 };
        const auto active = overrunAfterfireEcu.evaluate(
            overrunAfterfireConfig, overrunState, closedThrottle);
        require(active.overrunAfterfireActive && active.fuelEnabled
                && !active.sparkEnabled
                && active.fuelCorrection > 0.11
                && active.fuelCorrection < 0.13,
            "a deliberate power request must arm partial-fuel spark-cut overrun even while the post-start air floor is decaying");
        // The blocker mask has to agree with the boolean it explains. If it
        // could disagree it would be worse than useless: the whole point is
        // that a user seeing no pop can read WHY, and a mask derived
        // separately from the conjunction would eventually drift from it.
        require(active.overrunAfterfireBlockers == 0U,
            "an active overrun afterfire must report no blocker");

        overrunState.simulationTimeSeconds += 0.01;
        overrunState.rpm = 2'900.0;
        const auto belowMinimum = overrunAfterfireEcu.evaluate(
            overrunAfterfireConfig, overrunState, closedThrottle);
        require(!belowMinimum.overrunAfterfireActive
                && !belowMinimum.fuelEnabled && belowMinimum.sparkEnabled,
            "deceleration afterfire must remain a clean DFCO below its RPM gate");
        // Non-vacuous in both directions: the speed gate must be named, and
        // the conditions that are still satisfied must NOT be named. A mask
        // that simply lit every bit whenever the feature was inactive would
        // pass a one-sided assertion and tell a user nothing.
        require(enginelab::afterfireBlocked(
                    belowMinimum.overrunAfterfireBlockers,
                    enginelab::AfterfireBlocker::belowMinimumRpm),
            "the blocker mask must name the RPM gate that stopped the afterfire");
        require(!enginelab::afterfireBlocked(
                    belowMinimum.overrunAfterfireBlockers,
                    enginelab::AfterfireBlocker::notAuthored)
                && !enginelab::afterfireBlocked(
                    belowMinimum.overrunAfterfireBlockers,
                    enginelab::AfterfireBlocker::throttleOpen)
                && !enginelab::afterfireBlocked(
                    belowMinimum.overrunAfterfireBlockers,
                    enginelab::AfterfireBlocker::notArmed),
            "the blocker mask must not name conditions that are satisfied");
        require(std::string_view(enginelab::afterfireBlockerName(
                    belowMinimum.overrunAfterfireBlockers)) == "regime trop bas",
            "the blocker name must resolve to the reason a user can act on");

        // An unauthored engine is the case a reader meets first, and it must
        // say so rather than reporting the nine downstream conditions that a
        // disabled feature also fails.
        auto unauthoredConfig = overrunAfterfireConfig;
        unauthoredConfig.exhaustAfterfire.enabled = false;
        enginelab::SimpleEcuModel unauthoredEcu;
        unauthoredEcu.initialise(unauthoredConfig);
        const auto unauthored = unauthoredEcu.evaluate(
            unauthoredConfig, overrunState, closedThrottle);
        require(enginelab::afterfireBlocked(
                    unauthored.overrunAfterfireBlockers,
                    enginelab::AfterfireBlocker::notAuthored)
                && std::string_view(enginelab::afterfireBlockerName(
                    unauthored.overrunAfterfireBlockers)) == "non autorise",
            "an engine that does not author an afterfire must say exactly that");
    }

    {
        // Chopping the retained fuel is what separates a pop-and-bang map from
        // anti-lag: continuous delivery into a continuously ignited exhaust
        // burns steadily and roars, and discrete slugs are what pop. Zero must
        // stay exactly the historical every-cycle behaviour, because that is
        // what the whole catalogue authors.
        auto pulsedConfig = config;
        pulsedConfig.exhaustAfterfire.enabled = true;
        pulsedConfig.exhaustAfterfire.strategy =
            enginelab::ExhaustAfterfireStrategy::discreteAfterfire;
        pulsedConfig.exhaustAfterfire.overrunFuelFraction = 0.12;
        pulsedConfig.exhaustAfterfire.overrunMinimumRpm = 3'000.0;
        pulsedConfig.exhaustAfterfire.overrunMaximumThrottle = 0.02;
        pulsedConfig.exhaustAfterfire.overrunPulseHz = 4.0;
        pulsedConfig.exhaustAfterfire.overrunPulseDutyCycle = 0.35;

        const auto armedAt = [](const enginelab::EngineConfig& engineConfig,
                                double secondsAfterLift) {
            enginelab::SimpleEcuModel ecuModel;
            ecuModel.initialise(engineConfig);
            enginelab::EngineState state;
            state.simulationTimeSeconds = 1.0;
            state.rpm = 300.0;
            state.coolantTemperatureC = 90.0;
            enginelab::EngineControls cranking { true, true, 0.0, 0.0 };
            (void)ecuModel.evaluate(engineConfig, state, cranking);
            // Arm the strategy with a deliberate power request, then lift off.
            state.simulationTimeSeconds += 0.01;
            state.rpm = 4'000.0;
            state.throttle = 0.50;
            enginelab::EngineControls power { true, false, 0.50, 0.0 };
            (void)ecuModel.evaluate(engineConfig, state, power);
            state.simulationTimeSeconds = 1.02;
            state.throttle = 0.0;
            enginelab::EngineControls closed { true, false, 0.0, 0.0 };
            // The pulse clock is deliberately anchored at the lift-off edge,
            // rather than at an arbitrary absolute simulation-time grid.
            (void)ecuModel.evaluate(engineConfig, state, closed);
            state.simulationTimeSeconds += secondsAfterLift;
            return ecuModel.evaluate(engineConfig, state, closed)
                .overrunAfterfireActive;
        };

        // One 4 Hz period is 250 ms with a 35 % duty, so 0.05 s into a period
        // is inside the slug and 0.20 s is between slugs. Times are chosen from
        // the commanded rate, never read back from the model.
        require(armedAt(pulsedConfig, 0.05),
            "a chopped overrun must meter fuel inside its pulse");
        require(!armedAt(pulsedConfig, 0.20),
            "a chopped overrun must stop metering between its pulses");
        // Same instants, unchopped: both must stay armed, so the discriminator
        // above is the chopping and not the two timestamps.
        auto continuousConfig = pulsedConfig;
        continuousConfig.exhaustAfterfire.strategy =
            enginelab::ExhaustAfterfireStrategy::continuousAntiLag;
        continuousConfig.exhaustAfterfire.overrunPulseHz = 0.0;
        require(armedAt(continuousConfig, 0.05) && armedAt(continuousConfig, 0.20),
            "zero pulse rate must keep the historical every-cycle delivery");

        auto irregularConfig = pulsedConfig;
        irregularConfig.exhaustAfterfire.overrunPulseTimingVariation = 0.30;
        enginelab::SimpleEcuModel irregularEcu;
        irregularEcu.initialise(irregularConfig);
        enginelab::EngineState irregularState;
        irregularState.simulationTimeSeconds = 1.0;
        irregularState.rpm = 4'000.0;
        irregularState.coolantTemperatureC = 90.0;
        irregularState.throttle = 0.50;
        enginelab::EngineControls irregularPower { true, false, 0.50, 0.0 };
        (void)irregularEcu.evaluate(
            irregularConfig, irregularState, irregularPower);
        irregularState.simulationTimeSeconds += 0.001;
        irregularState.throttle = 0.0;
        enginelab::EngineControls irregularClosed { true, false, 0.0, 0.0 };
        auto previousPulseOpen = false;
        std::vector<double> irregularPulseStarts;
        constexpr auto timingSampleSeconds = 0.001;
        for (int step = 0; step < 4'000; ++step) {
            const auto command = irregularEcu.evaluate(
                irregularConfig, irregularState, irregularClosed);
            if (command.overrunAfterfireActive && !previousPulseOpen)
                irregularPulseStarts.push_back(
                    irregularState.simulationTimeSeconds);
            previousPulseOpen = command.overrunAfterfireActive;
            irregularState.simulationTimeSeconds += timingSampleSeconds;
        }
        require(irregularPulseStarts.size() >= 14,
            "irregular discrete afterfire must keep producing separate slugs");
        auto minimumInterval = 1.0;
        auto maximumInterval = 0.0;
        for (std::size_t index = 1;
             index < irregularPulseStarts.size(); ++index) {
            const auto interval = irregularPulseStarts[index]
                - irregularPulseStarts[index - 1];
            minimumInterval = std::min(minimumInterval, interval);
            maximumInterval = std::max(maximumInterval, interval);
        }
        require(minimumInterval >= 0.25 * (1.0 - 0.30) - 0.002
                && maximumInterval <= 0.25 * (1.0 + 0.30) + 0.002
                && maximumInterval - minimumInterval > 0.08,
            "timing variation must be audible but remain inside its authored bound");

        const auto retainedFuelIntegral = [](
                const enginelab::EngineConfig& engineConfig,
                double durationSeconds = 2.0) {
            enginelab::SimpleEcuModel ecuModel;
            ecuModel.initialise(engineConfig);
            enginelab::EngineState state;
            state.simulationTimeSeconds = 1.0;
            state.rpm = 4'000.0;
            state.coolantTemperatureC = 90.0;
            state.throttle = 0.50;
            enginelab::EngineControls power { true, false, 0.50, 0.0 };
            (void)ecuModel.evaluate(engineConfig, state, power);
            state.simulationTimeSeconds += 0.001;
            state.throttle = 0.0;
            enginelab::EngineControls closed { true, false, 0.0, 0.0 };
            (void)ecuModel.evaluate(engineConfig, state, closed);
            auto integral = 0.0;
            auto peak = 0.0;
            constexpr auto dt = 0.001;
            for (int step = 0;
                 step < static_cast<int>(durationSeconds / dt); ++step) {
                state.simulationTimeSeconds += dt;
                const auto command = ecuModel.evaluate(
                    engineConfig, state, closed);
                if (command.fuelEnabled)
                    integral += command.fuelCorrection * dt;
                peak = std::max(peak, command.fuelCorrection);
            }
            return std::pair { integral, peak };
        };
        const auto [pulsedIntegral, pulsedPeak] =
            retainedFuelIntegral(pulsedConfig);
        const auto [continuousIntegral, continuousPeak] =
            retainedFuelIntegral(continuousConfig);
        require(std::abs(pulsedIntegral - continuousIntegral)
                    <= continuousIntegral * 0.015
                && pulsedPeak > continuousPeak * 2.5,
            "pulse duty must concentrate, not discard, the calibrated overrun fuel mass");
        const auto [irregularIntegral, irregularPeak] =
            retainedFuelIntegral(irregularConfig, 30.0);
        const auto [longContinuousIntegral, longContinuousPeak] =
            retainedFuelIntegral(continuousConfig, 30.0);
        require(std::abs(irregularIntegral - longContinuousIntegral)
                    <= longContinuousIntegral * 0.015
                && irregularPeak > longContinuousPeak * 2.5,
            "irregular pulse timing must preserve calibrated long-run fuel mass");
    }

    {
        // A fuel-cut strategy is not safe merely because it eventually turns
        // the injectors back on.  The port film must be replenished before the
        // crank reaches idle.  Follow a representative 1,000 rpm/s coast-down
        // instead of teleporting directly into the catch region, and require
        // useful metering authority at the target. This is deliberately
        // independent of any one catalogue engine and leaves the exact refill
        // window proportional to the calibrated idle speed.
        enginelab::SimpleEcuModel coastEcu;
        coastEcu.initialise(config);
        enginelab::EngineControls closedThrottle { true, false, 0.0, 0.0 };
        enginelab::EngineState coastDown;
        coastDown.simulationTimeSeconds = 1.0;
        coastDown.rpm = config.idleRpm * 2.0;
        coastDown.throttle = 0.0;
        auto coastCommand = coastEcu.evaluate(config, coastDown, closedThrottle);
        require(!coastCommand.fuelEnabled,
                "coast-down fixture must enter deceleration fuel cut");
        constexpr double coastRateRpmPerSecond = 1'000.0;
        while (coastDown.rpm > config.idleRpm) {
            coastDown.simulationTimeSeconds += 0.01;
            coastDown.rpm = std::max(config.idleRpm,
                coastDown.rpm - coastRateRpmPerSecond * 0.01);
            coastCommand = coastEcu.evaluate(config, coastDown, closedThrottle);
        }
        require(coastCommand.fuelEnabled && coastCommand.fuelCorrection > 0.50,
                "deceleration fuel must resume early enough to refill the port film before idle");
    }

    {
        enginelab::SimpleEcuModel noStartEcu;
        enginelab::SimplifiedGasolinePhysics noStartPhysics;
        enginelab::FourStrokeEventGenerator noStartEvents;
        auto noStartExhaust = enginelab::ExhaustGraph::makeForEngine(config);
        enginelab::EngineSimulator noStart(config, noStartEcu, noStartPhysics, noStartEvents, noStartExhaust);
        for (int step = 0; step < 1'200; ++step) (void)noStart.step(1.0 / 240.0, { true, false, 0.65, 0.0 });
        require(noStart.state().rpm == 0.0, "ignition and throttle must not start a stationary engine without starter");
        require(noStart.state().volumetricEfficiency == 0.0, "stationary engine must report zero volumetric efficiency");
        require(std::abs(noStart.state().manifoldPressureKpa - config.ambientPressureKpa) < 0.01,
                "stationary manifold pressure must settle at ambient pressure");
    }
    enginelab::SimpleEcuModel ecu;
    enginelab::SimplifiedGasolinePhysics physics;
    enginelab::FourStrokeEventGenerator events;
    auto exhaust = enginelab::ExhaustGraph::makeForEngine(config);
    enginelab::EngineSimulator simulator(config, ecu, physics, events, exhaust);
    require(ecu.calibrationStore()->snapshot()->revision() > 0,
            "simulator construction must initialise the ECU maps used by offline and runtime callers");
    std::set<std::uint32_t> firedCylinders;
    bool observedCylinderTelemetry = false;
    std::uint64_t previousBrakeCycleId = 0;
    double previousBrakeCycleEndTime = 0.0;
    std::size_t completedBrakeCycleCount = 0;
    for (int step = 0; step < 2'400; ++step) {
        const enginelab::EngineControls controls { true, step < 600, 0.45, 0.05 };
        const auto frame = simulator.step(1.0 / 240.0, controls);
        for (std::size_t index = 0; index < frame.firingEventCount; ++index)
            firedCylinders.insert(frame.firingEvents[index].cylinderId);
        for (std::size_t index = 0;
             index < frame.completedBrakeCycleSampleCount; ++index) {
            const auto& sample = frame.completedBrakeCycleSamples[index];
            require(sample.numericallyValid,
                    "a completed brake cycle must satisfy its numerical contract");
            require(sample.cycleId == previousBrakeCycleId + 1,
                    "undropped completed brake cycles must have contiguous IDs");
            require(sample.endTimeSeconds > sample.startTimeSeconds
                    && std::abs(sample.durationSeconds
                        - (sample.endTimeSeconds - sample.startTimeSeconds))
                        < 1.0e-10,
                    "cycle timestamps and duration must describe exact boundaries");
            require(std::abs(sample.integratedCrankRadians
                        - 4.0 * std::numbers::pi) < 1.0e-8,
                    "a four-stroke sample must contain exactly 720 crank degrees");
            require(std::abs(sample.meanTorqueNm * sample.integratedCrankRadians
                        - sample.brakeWorkJoules)
                        < 1.0e-8 * std::max(1.0, std::abs(sample.brakeWorkJoules))
                    && std::abs(sample.meanPowerKw * sample.durationSeconds * 1'000.0
                        - sample.brakeWorkJoules)
                        < 1.0e-8 * std::max(1.0, std::abs(sample.brakeWorkJoules)),
                    "cycle work, angle, torque, time and power must be invariant");
            if (previousBrakeCycleId != 0) {
                require(std::abs(sample.startTimeSeconds
                            - previousBrakeCycleEndTime) < 1.0e-9,
                        "successive complete cycles must share one exact boundary");
            } else {
                require(sample.cycleId == 1 && sample.startTimeSeconds > 0.0,
                        "the first partial cycle after reset must not be published");
            }
            previousBrakeCycleId = sample.cycleId;
            previousBrakeCycleEndTime = sample.endTimeSeconds;
            ++completedBrakeCycleCount;
        }
        require(frame.droppedCompletedBrakeCycleSampleCount == 0,
                "ordinary four-stroke stepping must not overflow cycle samples");
        if (frame.completedBrakeCycleSampleCount > 0) {
            const auto& latest = frame.completedBrakeCycleSamples[
                frame.completedBrakeCycleSampleCount - 1];
            require(std::abs(frame.state.cycleAveragedTorqueNm
                        - latest.meanTorqueNm) < 1.0e-10
                    && std::abs(frame.state.cycleAveragedPowerKw
                        - latest.meanPowerKw) < 1.0e-10,
                    "EngineState cycle averages must latch the latest complete cycle");
        }
        observedCylinderTelemetry = observedCylinderTelemetry || std::any_of(frame.state.cylinderStates.begin(),
            frame.state.cylinderStates.begin() + static_cast<std::ptrdiff_t>(frame.state.cylinderStateCount),
            [](const enginelab::CylinderState& cylinder) {
                return cylinder.pressureEstimateBar > 1.2 && cylinder.runnerPressureKpa > 0.0
                    && (cylinder.intakeValveLiftMm > 0.0 || cylinder.exhaustValveLiftMm > 0.0);
            });
    }
    if (simulator.state().rpm <= 500.0)
        std::cerr << "startup diagnostic: rpm=" << simulator.state().rpm
                  << " work=" << simulator.state().indicatedWorkJoulesPerCycle
                  << " pressure=" << simulator.state().cylinderStates[0].pressureEstimateBar
                  << " afr=" << simulator.state().airFuelRatio
                  << " heat=" << simulator.state().resolvedHeatReleaseKw << '\n';
    require(simulator.state().rpm > 500.0, "engine should start");
    require(firedCylinders == std::set<std::uint32_t>({ 1, 2, 3, 4 }), "all cylinders should fire");
    require(simulator.state().intakeRunnerPressureKpa > 0.0 && simulator.state().exhaustRunnerPressureKpa > 0.0,
            "simulation must expose intake and exhaust runner pressures");
    require(simulator.state().exhaustFlowGramsPerSecond > 0.0,
            "running simulation must produce exhaust mass flow telemetry");
    require(simulator.state().manifoldGasMassGrams > 0.0 && simulator.state().cylinderGasMassGrams > 0.0
            && simulator.state().gasInternalEnergyJoules > 0.0,
            "running simulation must expose conserved gas mass and energy telemetry");
    require(simulator.state().fuelConsumedGrams > 0.0
            && std::abs(simulator.state().fuelConsumedLitres
                - simulator.state().fuelConsumedGrams * 0.001 / config.fuelProperties.densityKgPerL) < 1.0e-9,
            "fuel consumption must integrate mass and calibrated liquid volume over time");
    require(observedCylinderTelemetry,
            "cylinder states must expose chamber pressure, valve lift and runner pressure over a cycle");
    require(std::abs(simulator.state().torqueNm
            - (simulator.state().indicatedTorqueNm - simulator.state().frictionTorqueNm)) < 0.001,
            "brake torque must equal indicated torque minus engine losses");
    require(std::abs(simulator.state().netTorqueNm
            - (simulator.state().torqueNm + simulator.state().starterTorqueNm - simulator.state().loadTorqueNm
               + simulator.state().reciprocatingTorqueNm)) < 0.001,
            "net acceleration torque must have an explicit balance");
    require(std::isfinite(simulator.state().cycleAveragedTorqueNm)
            && std::isfinite(simulator.state().cycleAveragedPowerKw),
            "cycle-averaged output must remain finite for UI and dyno consumers");
    require(completedBrakeCycleCount >= 20,
            "the cycle-sample regression must observe many real completed cycles");
    require(simulator.state().indicatedWorkJoulesPerCycle > 0.0
            && simulator.state().indicatedMeanEffectivePressureBar > 0.0
            && simulator.state().indicatedPowerKw > 0.0,
            "running engine must expose positive cycle-integrated P-dV work, IMEP and indicated power");
    require(std::any_of(simulator.state().cylinderStates.begin(),
            simulator.state().cylinderStates.begin() + static_cast<std::ptrdiff_t>(simulator.state().cylinderStateCount),
            [](const auto& cylinder) { return cylinder.indicatedWorkJoulesPerCycle > 0.0
                && cylinder.intakeResonanceFrequencyHz > 0.0; }),
            "cylinder telemetry must expose P-dV work and runner resonance state");

    {
        auto overflowConfig = enginelab::makeDefaultInlineTwo();
        enginelab::normaliseEngineConfig(overflowConfig);
        enginelab::SimpleEcuModel overflowEcu;
        enginelab::SimplifiedGasolinePhysics overflowPhysics;
        ShortCycleEventGenerator overflowEvents;
        auto overflowExhaust = enginelab::ExhaustGraph::makeForEngine(overflowConfig);
        enginelab::EngineSimulator overflow(
            overflowConfig, overflowEcu, overflowPhysics,
            overflowEvents, overflowExhaust);
        enginelab::EngineControls drivenControls;
        drivenControls.externalTorqueNm = 5'000.0;
        const auto saturatedFrame = overflow.step(0.05, drivenControls);
        require(saturatedFrame.completedBrakeCycleSampleCount
                    == enginelab::maxCompletedBrakeCycleSamplesPerSimulationStep
                && saturatedFrame.droppedCompletedBrakeCycleSampleCount > 0,
                "cycle completion overflow must fill the fixed array and report every drop");
        require(saturatedFrame.completedBrakeCycleSamples.front().cycleId == 1
                && saturatedFrame.completedBrakeCycleSamples.back().cycleId
                    == enginelab::maxCompletedBrakeCycleSamplesPerSimulationStep,
                "retained cycle IDs must start after the ignored partial cycle");
        const auto expectedNextId = static_cast<std::uint64_t>(
            saturatedFrame.completedBrakeCycleSampleCount
            + saturatedFrame.droppedCompletedBrakeCycleSampleCount + 1);
        const auto followingFrame = overflow.step(0.001, drivenControls);
        require(followingFrame.completedBrakeCycleSampleCount > 0
                && followingFrame.completedBrakeCycleSamples.front().cycleId
                    == expectedNextId,
                "cycle IDs must advance across dropped events without replay");
        overflow.reset();
        const auto resetFrame = overflow.step(0.05, drivenControls);
        require(resetFrame.completedBrakeCycleSampleCount > 0
                && resetFrame.completedBrakeCycleSamples.front().cycleId == 1,
                "reset must begin a new cycle-ID epoch after ignoring its partial cycle");
    }

    {
        enginelab::SimpleEcuModel drivenEcu;
        enginelab::SimplifiedGasolinePhysics drivenPhysics;
        enginelab::FourStrokeEventGenerator drivenEvents;
        auto drivenExhaust = enginelab::ExhaustGraph::makeForEngine(config);
        enginelab::EngineSimulator driven(config, drivenEcu, drivenPhysics, drivenEvents, drivenExhaust);
        for (int step = 0; step < 500; ++step)
            (void)driven.step(1.0 / 500.0, { false, false, 0.0, 0.0, 90.0 });
        const auto drivenRpm = driven.state().rpm;
        require(drivenRpm > 500.0, "positive driveline torque must be able to motor the engine through the clutch");
        for (int step = 0; step < 500; ++step)
            (void)driven.step(1.0 / 500.0, { false, false, 0.0, 0.0, -90.0 });
        require(driven.state().rpm < drivenRpm * 0.55,
                "negative driveline torque must provide bidirectional engine braking");
    }

    {
        const auto crankingMap = [](double idleBypassAreaMm2) {
            auto crankingConfig = enginelab::makeDefaultInlineTwo();
            crankingConfig.intake.idleBypassAreaMm2 = idleBypassAreaMm2;
            enginelab::SimpleEcuModel crankingEcu;
            enginelab::SimplifiedGasolinePhysics crankingPhysics;
            enginelab::FourStrokeEventGenerator crankingEvents;
            auto crankingExhaust = enginelab::ExhaustGraph::makeForEngine(crankingConfig);
            enginelab::EngineSimulator crankingSimulator(crankingConfig, crankingEcu, crankingPhysics,
                                                         crankingEvents, crankingExhaust);
            for (int step = 0; step < 960; ++step)
                (void)crankingSimulator.step(1.0 / 240.0, { false, true, 0.0, 0.0 });
            return crankingSimulator.state().manifoldPressureKpa;
        };
        const auto sealedMap = crankingMap(0.0);
        const auto bypassMap = crankingMap(80.0);
        require(std::abs(bypassMap - sealedMap) > 0.2,
                "configured idle bypass area must materially affect closed-throttle manifold filling");
    }

    enginelab::SpscQueue<int, 8> queue;
    require(queue.tryPush(42), "queue push should succeed");
    int value = 0;
    require(queue.tryPop(value) && value == 42, "queue must preserve payload");

    const enginelab::JsonEngineSerializer json;
    const auto jsonRoundTrip = json.decode(json.encode(config));
    require(static_cast<bool>(jsonRoundTrip), "JSON round trip should decode");
    require(jsonRoundTrip.config->firingOrder == config.firingOrder, "JSON must preserve firing order");
    require(std::abs(jsonRoundTrip.config->exhaust.primaryLengthMm - config.exhaust.primaryLengthMm) < 0.001,
            "JSON must preserve exhaust geometry");
    require(jsonRoundTrip.config->transmission.gearRatios.size() == config.transmission.gearRatios.size()
            && std::abs(jsonRoundTrip.config->transmission.drivelineEfficiency
                - config.transmission.drivelineEfficiency) < 0.001
            && std::abs(jsonRoundTrip.config->transmission.drivenWheelInertiaKgM2
                - config.transmission.drivenWheelInertiaKgM2) < 0.001,
            "JSON must preserve transmission gear count");
    require(std::abs(jsonRoundTrip.config->vehicle.massKg - config.vehicle.massKg) < 0.001,
            "JSON must preserve vehicle mass");
    require(std::abs(jsonRoundTrip.config->plenumVolumeLitres - config.plenumVolumeLitres) < 0.001,
            "JSON must preserve intake plenum geometry");
    require(jsonRoundTrip.config->crankJournals.size() == config.crankJournals.size(),
            "JSON must preserve explicit crank journal geometry");
    require(jsonRoundTrip.config->banks.size() == config.banks.size()
            && jsonRoundTrip.config->exhaustPaths.size() == config.exhaustPaths.size(),
            "JSON must preserve bank and exhaust topology");
    require(std::abs(jsonRoundTrip.config->solver.mechanicalFrequencyHz - config.solver.mechanicalFrequencyHz) < 0.001
            && jsonRoundTrip.config->ignition.timingCurve.size() == config.ignition.timingCurve.size(),
            "JSON must preserve solver and ignition calibration");
    require(jsonRoundTrip.config->injection.mode == config.injection.mode
            && std::abs(jsonRoundTrip.config->injection.startAngleDegrees - config.injection.startAngleDegrees) < 0.001
            && std::abs(jsonRoundTrip.config->injection.endAngleDegrees - config.injection.endAngleDegrees) < 0.001
            && std::abs(jsonRoundTrip.config->injection.injectorFlowMgPerSecond
                        - config.injection.injectorFlowMgPerSecond) < 0.001,
            "JSON must preserve injection strategy and injector calibration");
    require(jsonRoundTrip.config->fuelProperties.name == config.fuelProperties.name
            && std::abs(jsonRoundTrip.config->fuelProperties.lowerHeatingValueMjPerKg
                        - config.fuelProperties.lowerHeatingValueMjPerKg) < 0.001
            && std::abs(jsonRoundTrip.config->fuelProperties.stoichiometricAirFuelRatio
                        - config.fuelProperties.stoichiometricAirFuelRatio) < 0.001,
            "JSON must preserve the calibrated fuel thermodynamics and chemistry");
    require(std::abs(jsonRoundTrip.config->exhaust.collectorVolumeLitres
                     - config.exhaust.collectorVolumeLitres) < 0.001
            && std::abs(jsonRoundTrip.config->exhaust.outletDischargeCoefficient
                        - config.exhaust.outletDischargeCoefficient) < 0.001
            && std::abs(jsonRoundTrip.config->cylinders.front().connectingRodMassGrams
                        - config.cylinders.front().connectingRodMassGrams) < 0.001,
            "JSON must preserve physical exhaust volumes, discharge and rod mass");
    auto extendedPhysicsConfig = config;
    extendedPhysicsConfig.injection.railPressureBar = 155.0;
    extendedPhysicsConfig.injection.referencePressureBar = 180.0;
    extendedPhysicsConfig.injection.wallFilmFraction = 0.13;
    extendedPhysicsConfig.injection.vaporisationTimeConstantSeconds = 0.027;
    extendedPhysicsConfig.injection.latentHeatKjPerKg = 315.0;
    extendedPhysicsConfig.injection.directChargeCoolingEfficiency = 0.74;
    extendedPhysicsConfig.injection.portChargeCoolingEfficiency = 0.31;
    extendedPhysicsConfig.injection.directSprayVaporisationTimeConstantSeconds = 0.00042;
    extendedPhysicsConfig.injection.directSprayEntrainmentTimeConstantSeconds = 0.00019;
    extendedPhysicsConfig.cylinders.front().pistonFrictionCoefficient = 0.067;
    extendedPhysicsConfig.cylinders.front().pistonBreakawayForceN = 61.0;
    extendedPhysicsConfig.combustionCalibration.baseIgnitionDelaySeconds = 0.00062;
    extendedPhysicsConfig.combustionCalibration.chamberTurbulenceIntensityRatio = 1.37;
    extendedPhysicsConfig.combustionCalibration.ignitionSiteCount = 2;
    extendedPhysicsConfig.combustionCalibration.compressionIgnitionDelayScale = 1.8;
    extendedPhysicsConfig.combustionCalibration.compressionIgnitionMixingTimeSeconds = 0.0017;
    extendedPhysicsConfig.combustionCalibration.compressionIgnitionPremixedFraction = 0.16;
    extendedPhysicsConfig.combustionCalibration.cycleVariationCoefficientOfVariation = 0.047;
    extendedPhysicsConfig.combustionCalibration.cycleVariationCorrelation = 0.62;
    extendedPhysicsConfig.ignition.limiterKeepsFuel = true;
    extendedPhysicsConfig.exhaustAfterfire = { true, 875.0, 0.012, 0.91 };
    extendedPhysicsConfig.exhaustAfterfire.overrunFuelFraction = 0.11;
    extendedPhysicsConfig.exhaustAfterfire.overrunMinimumRpm = 3'450.0;
    extendedPhysicsConfig.exhaustAfterfire.overrunMaximumThrottle = 0.015;
    extendedPhysicsConfig.exhaustAfterfire.overrunPulseHz = 6.5;
    extendedPhysicsConfig.exhaustAfterfire.overrunPulseDutyCycle = 0.42;
    extendedPhysicsConfig.exhaustAfterfire.overrunPulseTimingVariation = 0.23;
    extendedPhysicsConfig.exhaustAfterfire.strategy =
        enginelab::ExhaustAfterfireStrategy::discreteAfterfire;
    extendedPhysicsConfig.exhaustAfterfire.inductionTimeSeconds = 0.0065;
    extendedPhysicsConfig.exhaustAfterfire.minimumEquivalenceRatio = 0.51;
    extendedPhysicsConfig.exhaustAfterfire.maximumEquivalenceRatio = 1.63;
    extendedPhysicsConfig.exhaustAfterfire.quenchTemperatureK = 545.0;
    extendedPhysicsConfig.exhaust.mufflerChamberDiameterMm = 118.0;
    extendedPhysicsConfig.exhaust.mufflerChamberLengthMm = 360.0;
    extendedPhysicsConfig.exhaust.mufflerPackingFlowResistivityPaSPerM2 = 24'000.0;
    extendedPhysicsConfig.exhaust.mufflerPackingThicknessMm = 35.0;
    extendedPhysicsConfig.exhaust.mufflerPerforatedOpenAreaRatio = 0.28;
    extendedPhysicsConfig.exhaustPaths.front().geometry = extendedPhysicsConfig.exhaust;
    extendedPhysicsConfig.runnerAcoustics.dampingRatio = 0.21;
    extendedPhysicsConfig.intake.runnerPlenumDiameterMm = 49.0;
    extendedPhysicsConfig.intakePaths.front().geometry.runnerPlenumDiameterMm = 49.0;
    extendedPhysicsConfig.forcedInduction.enabled = true;
    extendedPhysicsConfig.forcedInduction.designShaftSpeedRpm = 145'000.0;
    extendedPhysicsConfig.forcedInduction.bearingFrictionPowerWatts = 360.0;
    extendedPhysicsConfig.forcedInduction.turbineFlowAreaMm2 = 640.0;
    extendedPhysicsConfig.forcedInduction.wastegateFlowAreaMm2 = 410.0;
    extendedPhysicsConfig.transmission.reverseRatio = 3.55;
    extendedPhysicsConfig.transmission.clutchThermalCapacityJPerC = 24'000.0;
    extendedPhysicsConfig.vehicle.tireFrictionCoefficient = 1.15;
    extendedPhysicsConfig.camshafts.intakeFlowCurve = { { 0.0, 0.0 }, { 5.0, 0.50 }, { 10.2, 0.68 } };
    extendedPhysicsConfig.camshafts.continuousControl.enabled = true;
    extendedPhysicsConfig.camshafts.continuousControl.samples = { { 1'000.0, 0.2, 2.0, 0.0, 0.9 },
                                                                  { 6'000.0, 1.0, 24.0, 10.0, 1.1 } };
    enginelab::ExhaustNetworkConfig taperedExhaust;
    enginelab::ExhaustComponentConfig taperedOutlet;
    taperedOutlet.id = 1;
    taperedOutlet.type = enginelab::ExhaustComponentType::outlet;
    taperedOutlet.lengthMm = 320.0;
    taperedOutlet.diameterMm = 42.0;
    taperedOutlet.outletDiameterMm = 68.0;
    taperedExhaust.components.push_back(taperedOutlet);
    for (const auto& cylinder : extendedPhysicsConfig.cylinders)
        taperedExhaust.cylinderConnections.push_back(
            { cylinder.id, taperedOutlet.id });
    extendedPhysicsConfig.exhaustPaths.front().network =
        std::move(taperedExhaust);
    const auto extendedJsonRoundTrip = json.decode(json.encode(extendedPhysicsConfig));
    require(extendedJsonRoundTrip
            && std::abs(extendedJsonRoundTrip.config->injection.railPressureBar - 155.0) < 0.001
            && std::abs(extendedJsonRoundTrip.config->injection.wallFilmFraction - 0.13) < 0.001
            && std::abs(extendedJsonRoundTrip.config->injection.latentHeatKjPerKg - 315.0) < 0.001
            && std::abs(extendedJsonRoundTrip.config->injection.directSprayVaporisationTimeConstantSeconds - 0.00042) < 1.0e-9
            && std::abs(extendedJsonRoundTrip.config->injection.directSprayEntrainmentTimeConstantSeconds - 0.00019) < 1.0e-9
            && std::abs(extendedJsonRoundTrip.config->cylinders.front().pistonFrictionCoefficient - 0.067) < 0.001
            && std::abs(extendedJsonRoundTrip.config->cylinders.front().pistonBreakawayForceN - 61.0) < 0.001,
            "JSON must preserve injection thermodynamics and per-cylinder Stribeck friction");
    require(extendedJsonRoundTrip
            && std::abs(extendedJsonRoundTrip.config->forcedInduction.designShaftSpeedRpm - 145'000.0) < 0.001
            && std::abs(extendedJsonRoundTrip.config->forcedInduction.turbineFlowAreaMm2 - 640.0) < 0.001
            && std::abs(extendedJsonRoundTrip.config->forcedInduction.wastegateFlowAreaMm2 - 410.0) < 0.001,
            "JSON must preserve turbo shaft and flow-area calibration");
    require(extendedJsonRoundTrip
            && std::abs(extendedJsonRoundTrip.config->combustionCalibration.baseIgnitionDelaySeconds - 0.00062) < 1.0e-9
            && std::abs(extendedJsonRoundTrip.config->combustionCalibration.chamberTurbulenceIntensityRatio - 1.37) < 0.001
            && extendedJsonRoundTrip.config->combustionCalibration.ignitionSiteCount == 2
            && std::abs(extendedJsonRoundTrip.config->combustionCalibration.compressionIgnitionDelayScale - 1.8) < 0.001
            && std::abs(extendedJsonRoundTrip.config->combustionCalibration.compressionIgnitionMixingTimeSeconds - 0.0017) < 1.0e-9
            && std::abs(extendedJsonRoundTrip.config->combustionCalibration.compressionIgnitionPremixedFraction - 0.16) < 0.001
            && std::abs(extendedJsonRoundTrip.config->combustionCalibration.cycleVariationCoefficientOfVariation - 0.047) < 0.001
            && std::abs(extendedJsonRoundTrip.config->combustionCalibration.cycleVariationCorrelation - 0.62) < 0.001
            && extendedJsonRoundTrip.config->ignition.limiterKeepsFuel
            && extendedJsonRoundTrip.config->exhaustAfterfire.enabled
            && std::abs(extendedJsonRoundTrip.config->exhaustAfterfire.ignitionTemperatureK - 875.0) < 0.001
            && std::abs(extendedJsonRoundTrip.config->exhaustAfterfire.reactionTimeConstantSeconds - 0.012) < 1.0e-9
            && std::abs(extendedJsonRoundTrip.config->exhaustAfterfire.reactionEfficiency - 0.91) < 0.001
            && std::abs(extendedJsonRoundTrip.config->exhaustAfterfire.overrunFuelFraction - 0.11) < 0.001
            && std::abs(extendedJsonRoundTrip.config->exhaustAfterfire.overrunMinimumRpm - 3'450.0) < 0.001
            && std::abs(extendedJsonRoundTrip.config->exhaustAfterfire.overrunMaximumThrottle - 0.015) < 0.001
            && std::abs(extendedJsonRoundTrip.config->exhaustAfterfire.overrunPulseHz - 6.5) < 0.001
            && std::abs(extendedJsonRoundTrip.config->exhaustAfterfire.overrunPulseDutyCycle - 0.42) < 0.001
            && std::abs(extendedJsonRoundTrip.config->exhaustAfterfire.overrunPulseTimingVariation - 0.23) < 0.001
            && extendedJsonRoundTrip.config->exhaustAfterfire.strategy
                == enginelab::ExhaustAfterfireStrategy::discreteAfterfire
            && std::abs(extendedJsonRoundTrip.config->exhaustAfterfire.inductionTimeSeconds - 0.0065) < 1.0e-9
            && std::abs(extendedJsonRoundTrip.config->exhaustAfterfire.minimumEquivalenceRatio - 0.51) < 0.001
            && std::abs(extendedJsonRoundTrip.config->exhaustAfterfire.maximumEquivalenceRatio - 1.63) < 0.001
            && std::abs(extendedJsonRoundTrip.config->exhaustAfterfire.quenchTemperatureK - 545.0) < 0.001
            && std::abs(extendedJsonRoundTrip.config->exhaust.mufflerPackingFlowResistivityPaSPerM2 - 24'000.0) < 0.001
            && std::abs(extendedJsonRoundTrip.config->exhaustPaths.front().geometry.mufflerPackingThicknessMm - 35.0) < 0.001
            && std::abs(extendedJsonRoundTrip.config->exhaustPaths.front().geometry.mufflerPerforatedOpenAreaRatio - 0.28) < 0.001
            && std::abs(extendedJsonRoundTrip.config->intake
                    .runnerPlenumDiameterMm - 49.0) < 0.001
            && std::abs(extendedJsonRoundTrip.config->transmission.reverseRatio - 3.55) < 0.001
            && extendedJsonRoundTrip.config->camshafts.intakeFlowCurve.size() == 3
            && extendedJsonRoundTrip.config->camshafts.continuousControl.samples.size() == 2
            && extendedJsonRoundTrip.config->exhaustPaths.front().network
            && std::abs(extendedJsonRoundTrip.config->exhaustPaths.front()
                    .network->components.front().outletDiameterMm - 68.0) < 0.001,
            "JSON must preserve Phase 3-5 combustion, driveline and valvetrain calibration");
    const auto v8JsonRoundTrip = json.decode(json.encode(enginelab::makeDefaultV8()));
    require(v8JsonRoundTrip && v8JsonRoundTrip.config->layout == enginelab::EngineLayout::vLayout,
            "JSON must preserve V engine layout");
    auto legacyJson = json.encode(config);
    const auto currentSchemaMarker = std::string("\"schema_version\": ")
        + std::to_string(enginelab::currentEngineSchemaVersion);
    const auto schemaMarker = legacyJson.find(currentSchemaMarker);
    require(schemaMarker != std::string::npos, "JSON writer must emit the current schema version");
    legacyJson.replace(schemaMarker, currentSchemaMarker.size(),
                       "\"schema_version\": 1");
    const auto migratedJson = json.decode(legacyJson);
    require(migratedJson
            && migratedJson.config->schemaVersion == enginelab::currentEngineSchemaVersion,
            "schema-v1 JSON must migrate to the current in-memory schema");
    const auto radialJsonRoundTrip = json.decode(json.encode(enginelab::makeDefaultRadialFive()));
    require(radialJsonRoundTrip && radialJsonRoundTrip.config->layout == enginelab::EngineLayout::radial
            && radialJsonRoundTrip.config->crankJournals.size() == 1
            && radialJsonRoundTrip.config->crankshafts.size() == 1
            && radialJsonRoundTrip.config->cylinders[1].connectingRodType == enginelab::ConnectingRodType::articulated
            && radialJsonRoundTrip.config->cylinders[1].masterCylinderId == 1,
            "JSON must preserve radial crankshaft and master/articulated rod topology");
    require(!json.decode(R"({"schema_version":1,"engine":{"name":"bad","cycle":"steam","fuel":"gasoline"}})"),
            "JSON must reject unknown enum values");

    const enginelab::YamlEngineSerializer yaml;
    const auto yamlRoundTrip = yaml.decode(yaml.encode(config));
    require(static_cast<bool>(yamlRoundTrip), "YAML round trip should decode");
    const auto extendedYamlRoundTrip = yaml.decode(yaml.encode(extendedPhysicsConfig));
    require(extendedYamlRoundTrip
            && std::abs(extendedYamlRoundTrip.config->injection.referencePressureBar - 180.0) < 0.001
            && std::abs(extendedYamlRoundTrip.config->injection.vaporisationTimeConstantSeconds - 0.027) < 0.001
            && std::abs(extendedYamlRoundTrip.config->injection.directChargeCoolingEfficiency - 0.74) < 0.001
            && std::abs(extendedYamlRoundTrip.config->injection.directSprayVaporisationTimeConstantSeconds - 0.00042) < 1.0e-9
            && std::abs(extendedYamlRoundTrip.config->cylinders.front().pistonFrictionCoefficient - 0.067) < 0.001,
            "YAML must preserve injection thermodynamics and per-cylinder friction");
    require(extendedYamlRoundTrip
            && std::abs(extendedYamlRoundTrip.config->forcedInduction.designShaftSpeedRpm - 145'000.0) < 0.001
            && std::abs(extendedYamlRoundTrip.config->forcedInduction.bearingFrictionPowerWatts - 360.0) < 0.001,
            "YAML must preserve turbo inertia, bearing and speed calibration");
    require(extendedYamlRoundTrip
            && std::abs(extendedYamlRoundTrip.config->runnerAcoustics.dampingRatio - 0.21) < 0.001
            && std::abs(extendedYamlRoundTrip.config->exhaustAfterfire.overrunFuelFraction - 0.11) < 0.001
            && std::abs(extendedYamlRoundTrip.config->exhaustAfterfire.overrunMinimumRpm - 3'450.0) < 0.001
            && std::abs(extendedYamlRoundTrip.config->exhaustAfterfire.overrunMaximumThrottle - 0.015) < 0.001
            && std::abs(extendedYamlRoundTrip.config->exhaustAfterfire.overrunPulseHz - 6.5) < 0.001
            && std::abs(extendedYamlRoundTrip.config->exhaustAfterfire.overrunPulseDutyCycle - 0.42) < 0.001
            && std::abs(extendedYamlRoundTrip.config->exhaustAfterfire.overrunPulseTimingVariation - 0.23) < 0.001
            && extendedYamlRoundTrip.config->exhaustAfterfire.strategy
                == enginelab::ExhaustAfterfireStrategy::discreteAfterfire
            && std::abs(extendedYamlRoundTrip.config->exhaustAfterfire.inductionTimeSeconds - 0.0065) < 1.0e-9
            && std::abs(extendedYamlRoundTrip.config->exhaustAfterfire.minimumEquivalenceRatio - 0.51) < 0.001
            && std::abs(extendedYamlRoundTrip.config->exhaustAfterfire.maximumEquivalenceRatio - 1.63) < 0.001
            && std::abs(extendedYamlRoundTrip.config->exhaustAfterfire.quenchTemperatureK - 545.0) < 0.001
            && std::abs(extendedYamlRoundTrip.config->exhaust.mufflerPackingFlowResistivityPaSPerM2 - 24'000.0) < 0.001
            && std::abs(extendedYamlRoundTrip.config->exhaustPaths.front().geometry.mufflerPackingThicknessMm - 35.0) < 0.001
            && std::abs(extendedYamlRoundTrip.config->intake
                    .runnerPlenumDiameterMm - 49.0) < 0.001
            && std::abs(extendedYamlRoundTrip.config->transmission.clutchThermalCapacityJPerC - 24'000.0) < 0.001
            && std::abs(extendedYamlRoundTrip.config->vehicle.tireFrictionCoefficient - 1.15) < 0.001
            && extendedYamlRoundTrip.config->camshafts.continuousControl.enabled
            && extendedYamlRoundTrip.config->exhaustPaths.front().network
            && std::abs(extendedYamlRoundTrip.config->exhaustPaths.front()
                    .network->components.front().outletDiameterMm - 68.0) < 0.001,
            "YAML must preserve Phase 3-5 acoustics, clutch, tire and continuous valve control");
    auto dieselConfig = config;
    dieselConfig.fuel = enginelab::FuelType::diesel;
    dieselConfig.injection.mode = enginelab::InjectionMode::direct;
    dieselConfig.fuelProperties.name = "EN 590 C12H23 surrogate";
    dieselConfig.fuelProperties.lowerHeatingValueMjPerKg = 42.6;
    dieselConfig.fuelProperties.densityKgPerL = 0.832;
    dieselConfig.fuelProperties.stoichiometricAirFuelRatio = 14.65;
    dieselConfig.fuelProperties.molarMassGramsPerMole = 167.31;
    dieselConfig.fuelProperties.oxygenMolesPerFuelMole = 17.75;
    dieselConfig.fuelProperties.productMolesPerFuelMole = 23.5;
    dieselConfig.fuelProperties.laminarFlameSpeedMps = 0.0;
    dieselConfig.fuelProperties.turbulenceFlameSpeedGain = 0.0;
    dieselConfig.fuelProperties.cetaneNumber = 51.0;
    dieselConfig.injection.fullLoadFuelLimit = {
        { 1'000.0, 55.0 }, { 4'000.0, 44.0 } };
    require(!enginelab::validateEngineConfig(dieselConfig),
            "a four-stroke direct-injected diesel configuration must validate");
    const auto dieselJsonRoundTrip = json.decode(json.encode(dieselConfig));
    const auto dieselYamlRoundTrip = yaml.decode(yaml.encode(dieselConfig));
    require(dieselJsonRoundTrip && dieselYamlRoundTrip
            && dieselJsonRoundTrip.config->fuel == enginelab::FuelType::diesel
            && dieselYamlRoundTrip.config->fuelProperties.cetaneNumber == 51.0
            && dieselJsonRoundTrip.config->injection.fullLoadFuelLimit.size() == 2
            && std::abs(dieselYamlRoundTrip.config->injection
                    .fullLoadFuelLimit.back().milligramsPerCycle - 44.0) < 0.001,
            "JSON and YAML must preserve diesel chemistry and the physical full-load fuel schedule");
    auto invalidPortDiesel = dieselConfig;
    invalidPortDiesel.injection.mode = enginelab::InjectionMode::port;
    require(enginelab::validateEngineConfig(invalidPortDiesel).has_value(),
            "compression ignition must reject port injection");
    require(yamlRoundTrip.config->cylinders.size() == 4, "YAML must preserve cylinders");
    const auto v8YamlRoundTrip = yaml.decode(yaml.encode(enginelab::makeDefaultV8()));
    require(v8YamlRoundTrip && v8YamlRoundTrip.config->layout == enginelab::EngineLayout::vLayout,
            "YAML must preserve V engine layout");
    const auto radialYamlRoundTrip = yaml.decode(yaml.encode(enginelab::makeDefaultRadialFive()));
    require(radialYamlRoundTrip && radialYamlRoundTrip.config->crankshafts.size() == 1
            && radialYamlRoundTrip.config->cylinders[0].connectingRodType == enginelab::ConnectingRodType::master
            && radialYamlRoundTrip.config->cylinders[4].connectingRodType == enginelab::ConnectingRodType::articulated,
            "YAML must preserve master/articulated rod and crankshaft topology");
    require(radialYamlRoundTrip && radialYamlRoundTrip.config->layout == enginelab::EngineLayout::radial
            && radialYamlRoundTrip.config->cylinders.front().crankJournalId == 1,
            "YAML must preserve radial layout and cylinder journal references");
    require(v8YamlRoundTrip && std::abs(v8YamlRoundTrip.config->bankAngleDegrees - 90.0) < 0.001,
            "YAML must preserve V-engine bank angle");
    require(yamlRoundTrip.config->transmission.gearRatios.size() == config.transmission.gearRatios.size()
            && std::abs(yamlRoundTrip.config->transmission.shiftDurationSeconds
                - config.transmission.shiftDurationSeconds) < 0.001,
            "YAML must preserve transmission gear count");
    require(std::abs(yamlRoundTrip.config->vehicle.tireRadiusM - config.vehicle.tireRadiusM) < 0.001,
            "YAML must preserve vehicle tire radius");
    require(yamlRoundTrip.config->crankJournals.size() == config.crankJournals.size(),
            "YAML must preserve explicit crank journal geometry");
    require(yamlRoundTrip.config->banks.size() == config.banks.size()
            && yamlRoundTrip.config->exhaustPaths.size() == config.exhaustPaths.size(),
            "YAML must preserve bank and exhaust topology");
    require(std::abs(yamlRoundTrip.config->intake.runnerLengthMm - config.intake.runnerLengthMm) < 0.001
            && yamlRoundTrip.config->ignition.timingCurve.size() == config.ignition.timingCurve.size(),
            "YAML must preserve intake and ignition calibration");
    require(yamlRoundTrip.config->injection.mode == config.injection.mode
            && std::abs(yamlRoundTrip.config->injection.injectorFlowMgPerSecond
                        - config.injection.injectorFlowMgPerSecond) < 0.001
            && std::abs(yamlRoundTrip.config->exhaust.collectorVolumeLitres
                        - config.exhaust.collectorVolumeLitres) < 0.001
            && std::abs(yamlRoundTrip.config->cylinders.front().connectingRodMassGrams
                        - config.cylinders.front().connectingRodMassGrams) < 0.001,
            "YAML must preserve injection, exhaust control volume and rod mass");
    require(yamlRoundTrip.config->fuelProperties.name == config.fuelProperties.name
            && std::abs(yamlRoundTrip.config->fuelProperties.laminarFlameSpeedMps
                        - config.fuelProperties.laminarFlameSpeedMps) < 0.001,
            "YAML must preserve fuel identity and combustion calibration");
    require(std::abs(jsonRoundTrip.config->camshafts.intakeLiftMm - config.camshafts.intakeLiftMm) < 0.001,
            "JSON must preserve camshaft lift");
    auto invalidPhysicalConfig = config;
    invalidPhysicalConfig.cylinders.front().boreMm = -1.0;
    require(!json.decode(json.encode(invalidPhysicalConfig)), "decoder must reject physically invalid engine files");
    require(!yaml.decode(yaml.encode(invalidPhysicalConfig)), "YAML decoder must reject physically invalid engine files");
    {
        const auto readText = [](const std::filesystem::path& path) {
            std::ifstream input(path, std::ios::binary);
            return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
        };
        const auto root = std::filesystem::path(ENGINELAB_CATALOG_ROOT);
        const auto jsonExample = json.decode(readText(root / "examples" / "inline-four.json"));
        const auto yamlExample = yaml.decode(readText(root / "examples" / "inline-four.yaml"));
        require(jsonExample && yamlExample
                && jsonExample.config->injection.mode == enginelab::InjectionMode::port
                && yamlExample.config->fuelProperties.name.find("98 RON") != std::string::npos,
                "shipped JSON and YAML examples must remain valid, complete physical configurations");
    }

    const auto presets = enginelab::makeBaseEnginePresets();
    require(presets.size() == 7, "I2, I4, I5, V6, V8, flat-six and radial-five presets must exist");
    require(presets[0].cylinders.size() == 2 && presets[2].cylinders.size() == 5
            && presets[3].cylinders.size() == 6 && presets[4].cylinders.size() == 8
            && presets[5].layout == enginelab::EngineLayout::flat
            && presets[6].layout == enginelab::EngineLayout::radial,
            "base presets must expose common and exotic layouts");
    require(enginelab::engineDisplacementLitres(presets[4]) > enginelab::engineDisplacementLitres(presets[0]),
            "V8 displacement should exceed I2 displacement");

    const auto catalog = enginelab::loadEngineCatalog(std::filesystem::path(ENGINELAB_CATALOG_ROOT));
    if (!catalog.errors.empty()) {
        for (const auto& error : catalog.errors) std::cerr << "catalog error: " << error << '\n';
    }
    require(catalog.errors.empty(), "engine catalog files must load without errors");
    require(catalog.entries.size() >= 10, "catalog must ship a meaningful starter library of realistic and exotic engines");
    {
        std::vector<enginelab::EngineCatalogEntry> selectionFixture(3);
        selectionFixture[0].config.name = "Big Twin-like 1.9 V2";
        selectionFixture[0].config.audioVoicingKey = "02_big_twin";
        selectionFixture[0].sourcePath = "02_big_twin.engine.yaml";
        selectionFixture[1].config.name = "Yamaha CP2 MT-07-like 689 Twin";
        selectionFixture[1].config.audioVoicingKey = "08_yamaha_cp2";
        selectionFixture[1].sourcePath = "08_yamaha_cp2.engine.yaml";
        selectionFixture[2].config.name = "MT-07-like 689 Twin Full System";
        selectionFixture[2].config.audioVoicingKey = "16_audio_physics_lab_689_twin";
        selectionFixture[2].sourcePath = "16_audio_physics_lab_689_twin.engine.yaml";

        const auto exact = enginelab::selectSingleEngineCatalogEntry(
            selectionFixture, "16_AUDIO_PHYSICS_LAB_689_TWIN");
        require(exact && exact.exactMatch
                && exact.entry == &selectionFixture[2],
                "an exact stable catalogue key must override ambiguous name substrings");
        const auto uniqueSubstring = enginelab::selectSingleEngineCatalogEntry(
            selectionFixture, "full system");
        require(uniqueSubstring && !uniqueSubstring.exactMatch
                && uniqueSubstring.entry == &selectionFixture[2],
                "a unique case-insensitive substring must remain convenient");
        const auto ambiguous = enginelab::selectSingleEngineCatalogEntry(
            selectionFixture, "Twin");
        require(ambiguous.status == enginelab::EngineCatalogSelectionStatus::ambiguous
                && ambiguous.entry == nullptr && ambiguous.matches.size() == 3,
                "an ambiguous selector must list candidates instead of choosing the first");
        const auto missing = enginelab::selectSingleEngineCatalogEntry(
            selectionFixture, "not-an-engine");
        require(missing.status == enginelab::EngineCatalogSelectionStatus::notFound
                && missing.matches.empty(),
                "a missing selector must remain distinguishable from ambiguity");
    }
    bool found2jz = false;
    bool foundV8 = false;
    bool foundMotorcycle = false;
    bool foundFlatSix = false;
    bool foundRadial = false;
    bool foundRadialBanks = false;
    bool foundCalibratedVtec = false;
    bool foundCalibratedAvgas = false;
    bool foundCompressionIgnition = false;
    bool foundGlobalCrossPlaneXPipe = false;
    bool foundMotorcycleFourTwoOne = false;
    const enginelab::EngineConfig* bigTwinConfig = nullptr;
    for (const auto& entry : catalog.entries) {
        found2jz = found2jz || entry.config.name.find("2JZ") != std::string::npos;
        foundV8 = foundV8 || (entry.config.layout == enginelab::EngineLayout::vLayout && entry.config.cylinders.size() == 8);
        foundMotorcycle = foundMotorcycle || entry.family == "motorcycle";
        foundFlatSix = foundFlatSix || (entry.config.layout == enginelab::EngineLayout::flat && entry.config.cylinders.size() == 6);
        foundRadial = foundRadial || (entry.config.layout == enginelab::EngineLayout::radial
            && entry.config.cylinders.size() == 5 && entry.config.crankJournals.size() == 1);
        if (entry.config.layout == enginelab::EngineLayout::radial) {
            foundRadialBanks = entry.config.banks.size() == entry.config.cylinders.size();
            for (const auto& cylinder : entry.config.cylinders) {
                const auto bank = std::find_if(entry.config.banks.begin(),
                    entry.config.banks.end(), [&cylinder](const auto& candidate) {
                        return candidate.id == cylinder.bankId;
                    });
                foundRadialBanks = foundRadialBanks
                    && bank != entry.config.banks.end()
                    && std::abs(bank->angleDegrees
                        - cylinder.bankOffsetDegrees) < 1.0e-9;
            }
        }
        foundCalibratedVtec = foundCalibratedVtec || (entry.config.name.find("K20A") != std::string::npos
            && entry.config.camshafts.variableProfileEnabled
            && entry.config.ignition.timingCurve.size() >= 5
            && entry.config.solver.gasSubsteps == 3
            && entry.config.intake.runnerLengthMm > 250.0);
        foundCalibratedAvgas = foundCalibratedAvgas || (entry.config.name.find("Merlin") != std::string::npos
            && entry.config.fuelProperties.name.find("100LL") != std::string::npos
            && entry.config.fuelProperties.lowerHeatingValueMjPerKg > 43.0);
        foundCompressionIgnition = foundCompressionIgnition
            || (entry.config.fuel == enginelab::FuelType::diesel
                && entry.config.fuelProperties.cetaneNumber >= 40.0
                && !entry.config.injection.fullLoadFuelLimit.empty());
        if (entry.config.name.find("Big Twin") != std::string::npos)
            bigTwinConfig = &entry.config;
        for (const auto& path : entry.config.exhaustPaths)
            require(path.network.has_value(),
                "every catalogue exhaust must compile to the explicit component DAG");
        if (entry.config.name.find("LS3") != std::string::npos
                && entry.config.exhaustPaths.size() == 1
                && entry.config.exhaustPaths.front().network) {
            const auto& components = entry.config.exhaustPaths.front()
                .network->components;
            const auto merges = std::count_if(
                components.begin(), components.end(), [](const auto& component) {
                    return component.type == enginelab::ExhaustComponentType::merge;
                });
            const auto splitters = std::count_if(
                components.begin(), components.end(), [](const auto& component) {
                    return component.type == enginelab::ExhaustComponentType::splitter;
                });
            const auto outlets = std::count_if(
                components.begin(), components.end(), [](const auto& component) {
                    return component.type == enginelab::ExhaustComponentType::outlet;
                });
            foundGlobalCrossPlaneXPipe = merges >= 3
                && splitters >= 1 && outlets == 2;
        }
        if (entry.config.name.find("Hayabusa") != std::string::npos
                && !entry.config.exhaustPaths.empty()
                && entry.config.exhaustPaths.front().network) {
            const auto& components = entry.config.exhaustPaths.front()
                .network->components;
            foundMotorcycleFourTwoOne = std::count_if(
                components.begin(), components.end(), [](const auto& component) {
                    return component.type == enginelab::ExhaustComponentType::merge;
                }) >= 3;
        }
        require(!enginelab::validateEngineConfig(entry.config), "every catalog engine must validate");
        require(!entry.sourcePath.empty(), "catalog entries must retain their source path");
    }
    require(found2jz && foundV8 && foundMotorcycle && foundFlatSix && foundRadial && foundCalibratedVtec,
            "catalog must cover iconic layouts and an explicit variable-valvetrain calibration");
    require(foundRadialBanks,
            "catalog radial cylinders must retain their independent spatial bank angles");
    require(foundCalibratedAvgas, "catalog parts must apply an explicit fuel calibration to aviation engines");
    require(foundCompressionIgnition,
            "catalog must include a cetane-calibrated compression-ignition engine");
    require(foundGlobalCrossPlaneXPipe,
            "the LS3 must expose one cross-bank 4-into-1/X-pipe/twin-outlet DAG");
    require(foundMotorcycleFourTwoOne,
            "the Hayabusa 4-2-1 must retain both pair collectors and its final merge");
    require(bigTwinConfig != nullptr,
            "catalog must retain the Big Twin start regression fixture");

    {
        // Only explicit authored afterfire or wet-limiter states may bypass
        // the normal combustion/injection gate. Letting every fuel-on,
        // spark-off phase inject wets the low-speed cranking cuts and the
        // high-inertia Big Twin reproducibly stalls at 0 rpm in the realtime
        // budget harness.
        enginelab::EngineRuntime runtime(*bigTwinConfig);
        runtime.setRealtimeThrottleEnabled(false);
        runtime.setDynoMaximumDurationSeconds(30.0);
        runtime.setIgnitionEnabled(true);
        runtime.setStarterEngaged(true);
        runtime.setDynoHoldEnabled(true);
        const auto targetRpm = std::min(
            bigTwinConfig->redlineRpm,
            bigTwinConfig->ignition.revLimitRpm) * 0.95;
        runtime.setDynoHoldRpm(targetRpm);
        runtime.start();
        runtime.startDyno();
        auto peakRpm = 0.0;
        const auto startProofDeadline =
            std::chrono::steady_clock::now() + std::chrono::seconds(15);
        while (std::chrono::steady_clock::now() < startProofDeadline) {
            const auto state = runtime.snapshot();
            peakRpm = std::max(peakRpm, state.rpm);
            if (peakRpm >= targetRpm * 0.90) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        runtime.stop();
        require(peakRpm >= targetRpm * 0.90,
            "Big Twin must catch and enter its dyno hold instead of flooding at 0 rpm");
    }

    {
        const auto simulate = [](const enginelab::EngineConfig& testConfig, double dt) {
            enginelab::SimpleEcuModel testEcu;
            enginelab::SimplifiedGasolinePhysics testPhysics;
            enginelab::FourStrokeEventGenerator testEvents;
            auto testExhaust = enginelab::ExhaustGraph::makeForEngine(testConfig);
            enginelab::EngineSimulator testSimulator(testConfig, testEcu, testPhysics, testEvents, testExhaust);
            const auto steps = static_cast<int>(6.0 / dt);
            for (int index = 0; index < steps; ++index)
                (void)testSimulator.step(dt, { true, static_cast<double>(index) * dt < 2.0, 0.52, 0.08 });
            return testSimulator.state();
        };
        const auto at120 = simulate(config, 1.0 / 120.0);
        const auto at240 = simulate(config, 1.0 / 240.0);
        const auto at480 = simulate(config, 1.0 / 480.0);
        const auto rpmSpread = std::max({ at120.rpm, at240.rpm, at480.rpm }) - std::min({ at120.rpm, at240.rpm, at480.rpm });
        if (rpmSpread / std::max(1.0, at240.rpm) >= 0.06)
            std::cerr << "step RPM: " << at120.rpm << ", " << at240.rpm << ", " << at480.rpm << '\n';
        require(rpmSpread / std::max(1.0, at240.rpm) < 0.06, "simulation result must remain stable across supported fixed steps");
        for (const auto& preset : presets) {
            const auto state = simulate(preset, 1.0 / 240.0);
            if (!(std::isfinite(state.rpm) && std::isfinite(state.torqueNm) && state.rpm > 300.0))
                std::cerr << "preset failed: " << preset.name << " rpm=" << state.rpm << " torque=" << state.torqueNm
                          << " damage=" << state.damage << " coolant=" << state.coolantTemperatureC << '\n';
            require(std::isfinite(state.rpm) && std::isfinite(state.torqueNm) && state.rpm > 300.0,
                    "every base preset must run to a finite self-sustaining state");
            require(state.rpm < preset.redlineRpm * 1.15, "preset must stay bounded by ECU limiter and mechanical losses");
        }
    }
    require(enginelab::valveLiftMm(470.0, 470.0, 248.0, 10.2) > 10.19,
            "valve must reach configured lift at cam centerline");
    require(enginelab::valveLiftMm(100.0, 470.0, 248.0, 10.2) == 0.0,
            "valve must be closed outside cam duration");
    require(enginelab::valveLiftMm(360.0 - config.camshafts.exhaustCenterlineDegrees,
                                  360.0 - config.camshafts.exhaustCenterlineDegrees,
                                  config.camshafts.exhaustDurationDegrees,
                                  config.camshafts.exhaustLiftMm) > config.camshafts.exhaustLiftMm * 0.999,
            "exhaust cam centerline must be referenced to exhaust TDC, not shifted by one crank revolution");
    require(enginelab::combustionPulse(76.0, 40.0, 0.0) > 0.999,
            "ignition advance must move the mechanical pressure peak earlier in the crank cycle");
    require(enginelab::combustionPulse(90.0, 0.0, 10.0) < 0.98,
            "per-cylinder ignition offset must shift the mechanical pressure trace");
    {
        auto bankGeometryConfig = enginelab::makeDefaultInlineTwo();
        bankGeometryConfig.layout = enginelab::EngineLayout::vLayout;
        bankGeometryConfig.bankAngleDegrees = 90.0;
        bankGeometryConfig.cylinders[0].crankOffsetDegrees = 0.0;
        bankGeometryConfig.cylinders[1].crankOffsetDegrees = 0.0;
        bankGeometryConfig.cylinders[0].bankOffsetDegrees = -45.0;
        bankGeometryConfig.cylinders[1].bankOffsetDegrees = 45.0;
        enginelab::SimpleEcuModel bankEcu;
        enginelab::SimplifiedGasolinePhysics bankPhysics;
        enginelab::FourStrokeEventGenerator bankEvents;
        auto bankExhaust = enginelab::ExhaustGraph::makeForEngine(bankGeometryConfig);
        enginelab::EngineSimulator bankSimulator(bankGeometryConfig, bankEcu, bankPhysics, bankEvents, bankExhaust);
        const auto bankFrame = bankSimulator.step(0.05, { false, false, 0.0, 0.0, 120.0 });
        require(bankFrame.state.cylinderStateCount == 2
                && std::abs(bankFrame.state.cylinderStates[0].cyclePhaseDegrees
                            - bankFrame.state.cylinderStates[1].cyclePhaseDegrees) < 1.0e-9
                && std::abs(bankFrame.state.cylinderStates[0].intakeValveLiftMm
                            - bankFrame.state.cylinderStates[1].intakeValveLiftMm) < 1.0e-9
                && std::abs(bankFrame.state.cylinderStates[0].exhaustValveLiftMm
                            - bankFrame.state.cylinderStates[1].exhaustValveLiftMm) < 1.0e-9,
                "spatial bank angle must not alter four-stroke valve phase");
    }
    {
        const auto simulateInjection = [](double injectorFlowMgPerSecond) {
            auto injectionConfig = enginelab::makeDefaultInlineTwo();
            injectionConfig.injection.injectorFlowMgPerSecond = injectorFlowMgPerSecond;
            enginelab::SimpleEcuModel injectionEcu;
            enginelab::SimplifiedGasolinePhysics injectionPhysics;
            enginelab::FourStrokeEventGenerator injectionEvents;
            auto injectionExhaust = enginelab::ExhaustGraph::makeForEngine(injectionConfig);
            enginelab::EngineSimulator injectionSimulator(injectionConfig, injectionEcu, injectionPhysics,
                                                          injectionEvents, injectionExhaust);
            bool observedPreTdcBurn = false;
            double maximumInjectedFuelMgPerCycle = 0.0;
            for (int step = 0; step < 720; ++step) {
                const auto frame = injectionSimulator.step(1.0 / 240.0,
                    { true, step < 360, 0.55, 0.04 });
                maximumInjectedFuelMgPerCycle = std::max(
                    maximumInjectedFuelMgPerCycle, frame.state.injectedFuelMgPerCycle);
                for (std::size_t index = 0; index < frame.state.cylinderStateCount; ++index) {
                    const auto& cylinder = frame.state.cylinderStates[index];
                    observedPreTdcBurn = observedPreTdcBurn
                        || (cylinder.cyclePhaseDegrees > 650.0 && cylinder.combustionPulse > 1.0e-5);
                }
            }
            return std::tuple {
                injectionSimulator.state(), observedPreTdcBurn, maximumInjectedFuelMgPerCycle
            };
        };
        const auto [fullDelivery, observedPreTdcBurn, maximumFullDelivery] = simulateInjection(20'000.0);
        const auto [flowLimited, ignoredBurnObservation, maximumFlowLimitedDelivery] = simulateInjection(10.0);
        (void)ignoredBurnObservation;
        require(observedPreTdcBurn, "resolved combustion must begin at the advanced spark angle before firing TDC");
        if (!(maximumFullDelivery > maximumFlowLimitedDelivery * 20.0))
            std::cerr << "injector diagnostic: full=" << maximumFullDelivery
                      << " limited=" << maximumFlowLimitedDelivery
                      << " full_rpm=" << fullDelivery.rpm
                      << " limited_rpm=" << flowLimited.rpm << '\n';
        require(maximumFullDelivery > maximumFlowLimitedDelivery * 20.0,
                "injector flow capacity must physically limit delivered fuel per cycle");
        require(fullDelivery.cylinderPressureTorqueBlend == 1.0
                && std::isfinite(fullDelivery.meanWorkTorqueNm)
                && std::isfinite(fullDelivery.cylinderPressureTorqueNm),
                "resolved cylinder pressure must be the operating torque source while mean-work remains telemetry");
        if (fullDelivery.solverResolutionLimited
                || fullDelivery.crankDegreesPerSolverStep
                    > config.solver.maximumCrankDegreesPerStep + 1.0e-6)
            std::cerr << "solver diagnostic: rpm=" << fullDelivery.rpm
                      << " hz=" << fullDelivery.solverFrequencyHz
                      << " crank_step=" << fullDelivery.crankDegreesPerSolverStep << '\n';
        require(!fullDelivery.solverResolutionLimited
                && fullDelivery.crankDegreesPerSolverStep
                    <= config.solver.maximumCrankDegreesPerStep + 1.0e-6,
                "validated engines must honour their advertised crank-angle solver resolution");
    }
    {
        auto variableCamConfig = config;
        variableCamConfig.banks.clear();
        variableCamConfig.camshafts.variableProfileEnabled = true;
        variableCamConfig.camshafts.switchRpm = 450.0;
        variableCamConfig.camshafts.switchThrottle = 0.2;
        variableCamConfig.camshafts.highIntakeLiftMm = 18.0;
        variableCamConfig.camshafts.highIntakeDurationDegrees = 330.0;
        enginelab::SimpleEcuModel variableEcu;
        enginelab::SimplifiedGasolinePhysics variablePhysics;
        enginelab::FourStrokeEventGenerator variableEvents;
        auto variableExhaust = enginelab::ExhaustGraph::makeForEngine(variableCamConfig);
        enginelab::EngineSimulator variableSimulator(variableCamConfig, variableEcu, variablePhysics,
                                                      variableEvents, variableExhaust);
        double observedLift = 0.0;
        for (int step = 0; step < 1'200; ++step) {
            (void)variableSimulator.step(1.0 / 480.0, { true, step < 400, 0.8, 0.02 });
            for (std::size_t index = 0; index < variableSimulator.state().cylinderStateCount; ++index)
                observedLift = std::max(observedLift, variableSimulator.state().cylinderStates[index].intakeValveLiftMm);
        }
        require(observedLift > config.camshafts.intakeLiftMm * 1.35,
                "variable valvetrain must switch to the configured high-lift profile under RPM/load conditions");
    }
    {
        std::vector<enginelab::ValveLiftSample> profile {
            { -120.0, 0.0 }, { -60.0, 2.0 }, { 0.0, 8.0 }, { 60.0, 2.0 }, { 120.0, 0.0 }
        };
        require(enginelab::profiledValveLiftMm(470.0, 470.0, 248.0, 10.2, profile) > 7.99,
                "profiled valve lift must use tabulated peak lift");
        require(enginelab::profiledValveLiftMm(350.0, 470.0, 248.0, 10.2, profile) == 0.0,
                "profiled valve lift must close at the table boundary");
    }

    {
        enginelab::SimpleEcuModel misfireEcu;
        misfireEcu.setTargetAirFuelRatio(18.0);
        auto misfireConfig = config;
        // Keep the chamber warm so cold-start enrichment does not turn this
        // deliberately lean flammability test back into a stoichiometric run.
        misfireConfig.ambientTemperatureC = 60.0;
        misfireConfig.injection.injectorFlowMgPerSecond = 20.0;
        enginelab::SimplifiedGasolinePhysics misfirePhysics;
        enginelab::FourStrokeEventGenerator misfireEvents;
        auto misfireExhaust = enginelab::ExhaustGraph::makeForEngine(misfireConfig);
        enginelab::EngineSimulator misfireSimulator(misfireConfig, misfireEcu, misfirePhysics, misfireEvents, misfireExhaust);
        bool observedMechanicalMisfire = false;
        for (int step = 0; step < 4'000; ++step) {
            (void)misfireSimulator.step(1.0 / 240.0, { true, step < 600, 0.55, 0.04 });
            const auto& state = misfireSimulator.state();
            for (std::size_t index = 0; index < state.cylinderStateCount; ++index) {
                if (state.cylinderStates[index].misfiring) {
                    observedMechanicalMisfire = true;
                    require(state.cylinderStates[index].combustionPulse == 0.0,
                            "a misfiring cylinder must remove its mechanical pressure pulse");
                }
            }
        }
        if (!observedMechanicalMisfire)
            std::cerr << "misfire diagnostic: rpm=" << misfireSimulator.state().rpm
                      << " probability=" << misfireSimulator.state().misfireRate
                      << " afr=" << misfireSimulator.state().airFuelRatio << '\n';
        require(observedMechanicalMisfire, "deterministic lean-mixture run must expose cylinder-level misfires");
    }

    {
        enginelab::FourStrokeEventGenerator timingEvents;
        enginelab::EngineState timingState;
        timingState.simulationTimeSeconds = 1.0;
        timingState.rpm = 3'000.0;
        timingState.throttle = 0.8;
        enginelab::EcuCommand command;
        command.ignitionAdvanceDegrees = 18.0;
        command.fuelEnabled = true;
        command.sparkEnabled = true;
        enginelab::CombustionResult combustion;
        combustion.combustionQuality = 1.0;
        combustion.pressureEstimateBar = 70.0;
        std::array<enginelab::FiringEvent, 64> generated {};
        const auto count = timingEvents.generate(config, timingState, command, combustion,
            0.95, 680.0, 1'500.0, 0.05, generated);
        require(count >= 8, "event generator must preserve firings across multiple crank cycles");
        for (std::size_t index = 1; index < count; ++index)
            require(generated[index - 1].timeSeconds <= generated[index].timeSeconds,
                    "firing events must be chronologically ordered and interpolated");
        require(std::any_of(generated.begin(), generated.begin() + static_cast<std::ptrdiff_t>(count), [](const auto& event) {
            return std::abs(event.crankAngleDegrees - 702.0) < 0.01;
        }), "ignition advance must shift firing angle before top dead centre");
    }

    {
        // Phase 6 characterization: lock which CombustionResult fields the event
        // generator actually reads, so a future rewrite of the mean-value model
        // cannot silently unwire the audio path. A default EngineState carries no
        // resolved per-cylinder state (cylinderStateCount == 0), forcing the
        // generator onto the CombustionResult scalar fallbacks:
        //   combustionQuality  -> firing intensity (unconditional, linear)
        //   pressureEstimateBar -> firing pressure (fallback branch)
        enginelab::EngineState fallbackState;
        fallbackState.simulationTimeSeconds = 1.0;
        fallbackState.rpm = 3'000.0;
        fallbackState.throttle = 0.6;
        fallbackState.load = 0.6;
        enginelab::EcuCommand fallbackCommand;
        fallbackCommand.fuelEnabled = true;
        fallbackCommand.sparkEnabled = true;
        const auto makeCombustion = [](double quality) {
            enginelab::CombustionResult result;
            result.combustionEnabled = true;
            result.combustionQuality = quality;
            result.misfireProbability = 0.0; // deterministic: misfire draw never trips
            result.pressureEstimateBar = 70.0;
            return result;
        };
        // Two generators freshly seeded to the same state -> identical variation
        // sequence, so the only difference between the runs is combustionQuality.
        enginelab::FourStrokeEventGenerator fullQualityEvents;
        enginelab::FourStrokeEventGenerator halfQualityEvents;
        std::array<enginelab::FiringEvent, 64> fullQualityGenerated {};
        std::array<enginelab::FiringEvent, 64> halfQualityGenerated {};
        const auto fullCount = fullQualityEvents.generate(config, fallbackState, fallbackCommand,
            makeCombustion(1.0), 0.95, 680.0, 1'500.0, 0.05, fullQualityGenerated);
        const auto halfCount = halfQualityEvents.generate(config, fallbackState, fallbackCommand,
            makeCombustion(0.5), 0.95, 680.0, 1'500.0, 0.05, halfQualityGenerated);
        require(fullCount > 0 && fullCount == halfCount,
                "characterization run must generate a matching set of firings");
        for (std::size_t index = 0; index < fullCount; ++index) {
            require(fullQualityGenerated[index].intensity > 0.01F,
                    "combustionQuality must drive a non-trivial firing intensity");
            require(std::abs(halfQualityGenerated[index].intensity
                        - 0.5F * fullQualityGenerated[index].intensity) < 1.0e-3F,
                    "firing intensity must scale linearly with combustionQuality (audio amplitude wiring)");
            require(std::abs(fullQualityGenerated[index].pressureEstimateBar - 70.0F) < 1.0e-2F,
                    "with no resolved cylinder state, firing pressure must fall back to combustion.pressureEstimateBar");
        }

        auto idleState = fallbackState;
        idleState.rpm = config.idleRpm;
        idleState.throttle = 0.0;
        idleState.load = 0.18;
        enginelab::FourStrokeEventGenerator idleEvents;
        std::array<enginelab::FiringEvent, 64> idleGenerated {};
        const auto idleCount = idleEvents.generate(config, idleState, fallbackCommand,
            makeCombustion(1.0), 0.95, 680.0, 1'500.0, 0.05, idleGenerated);
        require(idleCount > 0 && idleGenerated[0].intensity > 0.05F,
                "idle-bypass load must produce audible events with the driver throttle closed");

        auto misfireCombustion = makeCombustion(1.0);
        misfireCombustion.misfireProbability = 1.0;
        enginelab::FourStrokeEventGenerator healthyEvents;
        enginelab::FourStrokeEventGenerator misfireEvents;
        std::array<enginelab::FiringEvent, 64> healthyGenerated {};
        std::array<enginelab::FiringEvent, 64> misfireGenerated {};
        const auto healthyCount = healthyEvents.generate(config, idleState, fallbackCommand,
            makeCombustion(1.0), 0.95, 680.0, 1'500.0, 0.05, healthyGenerated);
        const auto misfireCount = misfireEvents.generate(config, idleState, fallbackCommand,
            misfireCombustion, 0.95, 680.0, 1'500.0, 0.05, misfireGenerated);
        require(healthyCount > 0 && healthyCount == misfireCount
                && !healthyGenerated[0].misfire && misfireGenerated[0].misfire
                && std::abs(healthyGenerated[0].intensity - misfireGenerated[0].intensity) < 1.0e-6F,
                "misfire intensity must retain its physical reference and be attenuated only by the renderer");
    }

    {
        auto v8Exhaust = enginelab::ExhaustGraph::makeForEngine(enginelab::makeDefaultV8());
        require(v8Exhaust.nodes().size() == 11, "exhaust graph must contain one primary per V8 cylinder");
        enginelab::FiringEvent event;
        event.exhaustPortId = 8;
        event.intensity = 1.0F;
        v8Exhaust.process(event);
        require(event.exhaustDelaySeconds > 0.0F && event.exhaustResonanceHz > 0.0F,
                "exhaust graph must propagate path delay and resonance into firing events");
    }

    {
        enginelab::EngineState healthyState;
        healthyState.rpm = 3'500.0;
        healthyState.manifoldPressureKpa = 95.0;
        enginelab::EcuCommand command;
        command.effectiveThrottle = 0.9;
        const auto healthy = physics.evaluateCombustion(config, healthyState, { true, false, 0.9, 0.2 }, command, 104.0);
        auto damagedState = healthyState;
        damagedState.damage = 0.8;
        const auto damaged = physics.evaluateCombustion(config, damagedState, { true, false, 0.9, 0.2 }, command, 104.0);
        require(healthy.indicatedTorqueNm > damaged.indicatedTorqueNm * 2.0,
                "damage must materially reduce combustion torque");
    }

    {
        auto invalidCam = config;
        invalidCam.camshafts.intakeCenterlineDegrees = std::numeric_limits<double>::quiet_NaN();
        require(enginelab::validateEngineConfig(invalidCam).has_value(),
                "non-finite cam timing must be rejected before it can poison simulation state");
        auto invalidPlenum = config;
        invalidPlenum.plenumVolumeLitres = 0.0;
        require(enginelab::validateEngineConfig(invalidPlenum).has_value(),
                "zero plenum volume must be rejected");
        auto unsupportedSchema = config;
        unsupportedSchema.schemaVersion = 99;
        require(enginelab::validateEngineConfig(unsupportedSchema).has_value(),
                "runtime construction must reject unsupported schemas too");
        auto underResolvedSolver = config;
        underResolvedSolver.solver.maximumMechanicalFrequencyHz = 2'000.0;
        require(enginelab::validateEngineConfig(underResolvedSolver).has_value(),
                "configuration must reject a solver cap that cannot honour crank resolution at the limiter");
        auto invalidFuel = config;
        invalidFuel.fuelProperties.lowerHeatingValueMjPerKg = 0.0;
        require(enginelab::validateEngineConfig(invalidFuel).has_value(),
                "configuration must reject non-physical fuel thermodynamics");
    }

    {
        auto invalidIdConfig = config;
        invalidIdConfig.cylinders.front().id = 4294967293U;
        require(enginelab::validateEngineConfig(invalidIdConfig).has_value(),
                "configuration with cylinder ID equal to or greater than reserved mergeId must be rejected");
    }

    {
        auto zeroLiftConfig = config;
        // Zero the lift everywhere the valve train can resolve it. Setting only
        // the global camshaft does not produce a zero-lift engine: activeCamshaft
        // prefers a BANK camshaft whenever the cylinder belongs to one, so the
        // banks kept their own lift and the engine went on breathing.
        const auto zeroCamLift = [](enginelab::CamshaftConfig& cam) {
            cam.intakeLiftMm = 0.0;
            cam.exhaustLiftMm = 0.0;
            cam.highIntakeLiftMm = 0.0;
            cam.highExhaustLiftMm = 0.0;
        };
        zeroCamLift(zeroLiftConfig.camshafts);
        for (auto& bank : zeroLiftConfig.banks) zeroCamLift(bank.camshafts);
        enginelab::SimpleEcuModel zeroLiftEcu;
        enginelab::SimplifiedGasolinePhysics zeroLiftPhysics;
        enginelab::FourStrokeEventGenerator zeroLiftEvents;
        auto zeroLiftExhaust = enginelab::ExhaustGraph::makeForEngine(zeroLiftConfig);
        enginelab::EngineSimulator zeroLiftSim(zeroLiftConfig, zeroLiftEcu, zeroLiftPhysics, zeroLiftEvents, zeroLiftExhaust);
        auto zeroLiftManifoldMeanKpa = 0.0;
        auto zeroLiftManifoldSamples = 0.0;
        for (int step = 0; step < 100; ++step) {
            const auto frame = zeroLiftSim.step(1.0 / 240.0, { true, step < 20, 0.5, 0.0 });
            if (step >= 76) {
                zeroLiftManifoldMeanKpa += frame.state.manifoldPressureKpa;
                zeroLiftManifoldSamples += 1.0;
            }
        }
        zeroLiftManifoldMeanKpa /= std::max(1.0, zeroLiftManifoldSamples);
        // Non-vacuity guard, and it is not hypothetical: this block used to
        // assert `volumetricEfficiency == 0` and passed for the wrong reason
        // entirely -- the starter was too weak to turn a zero-lift engine over
        // at all (measured: rpm 0.00), so the assertion was satisfied by the
        // `rpm > 20` guard rather than by anything about induction. A sealed
        // cylinder is a gas spring that returns the work put into it, so a
        // starter SHOULD spin it; once it did, the same assertion failed at
        // VE = 1.031. Requiring rotation keeps the real claim below honest.
        require(zeroLiftSim.state().rpm > 20.0,
                "zero-lift engine must still be turned by the starter, or the "
                "induction assertion below is vacuous");
        // The physical claim is that a cam with no lift cannot breathe. The
        // quantity that expresses it is what crossed the valve, not volumetric
        // efficiency: VE is computed from the oxygen TRAPPED in the chamber,
        // which for a sealed cylinder is the standing charge it was built with
        // and has never renewed. That VE definition is a real defect, tracked
        // separately -- it is deliberately not asserted here, because asserting
        // it on this quantity is what made the test misleading.
        require(zeroLiftSim.state().inductedChargeMassMgPerCycle == 0.0,
                "zero valve lift must induct no charge at all");
        require(zeroLiftSim.state().rpm < 300.0,
                "engine must not start and run with zero valve lift");
        // Guard intent: closed valves must not DRAIN (or feed) the manifold —
        // the failure this catches is kPa-scale. It is asserted on a time
        // average with a 0.25 kPa allowance because two centi-kPa effects are
        // physical, not leaks, since the intake runners became resolved 1-D
        // ducts: (1) the fuel injected during cranking equilibrates through
        // the open runner mouth, and ~10 mg of vapour in ~3 L of intake volume
        // is a real ~0.07 kPa partial pressure (the lumped runner cell used to
        // trap it, which is why an instantaneous 0.01 kPa bound ever held);
        // (2) the runner-plenum Helmholtz mode rings near-undamped at rest
        // (~±0.14 kPa on the manifold), so an instantaneous sample reads the
        // phase of a wave, not the inventory.
        std::cout << "zero-lift manifold mean: " << zeroLiftManifoldMeanKpa
                  << " kPa vs ambient " << config.ambientPressureKpa << " kPa\n";
        require(std::abs(zeroLiftManifoldMeanKpa - config.ambientPressureKpa) < 0.25,
                "zero valve lift must not drain or feed the manifold");
    }

    {
        auto customCrankConfig = config;
        customCrankConfig.crankJournals = { { 10, 90.0, 43.0 }, { 20, 270.0, 43.0 },
            { 30, 450.0, 43.0 }, { 40, 630.0, 43.0 } };
        customCrankConfig.cylinders[0].crankOffsetDegrees = 90.0;
        customCrankConfig.cylinders[0].crankJournalId = 10;
        customCrankConfig.cylinders[0].bankOffsetDegrees = -12.0;
        customCrankConfig.cylinders[0].intakeValveCount = 2;
        customCrankConfig.cylinders[0].exhaustValveCount = 2;
        customCrankConfig.cylinders[0].intakeValveDiameterMm = 34.5;
        customCrankConfig.cylinders[0].exhaustValveDiameterMm = 29.0;
        customCrankConfig.cylinders[1].crankOffsetDegrees = 270.0;
        customCrankConfig.cylinders[1].crankJournalId = 20;
        customCrankConfig.cylinders[2].crankOffsetDegrees = 450.0;
        customCrankConfig.cylinders[2].crankJournalId = 30;
        customCrankConfig.cylinders[3].crankOffsetDegrees = 630.0;
        customCrankConfig.cylinders[3].crankJournalId = 40;
        customCrankConfig.intake.throttleCount = 4;
        customCrankConfig.intakePaths.front().geometry.throttleCount = 4;
        customCrankConfig.camshafts.intakeFlowCoefficient = 0.70;
        customCrankConfig.camshafts.exhaustFlowCoefficient = 0.66;
        customCrankConfig.camshafts.intakeLiftProfile = {
            { -124.0, 0.0 }, { -60.0, 3.2 }, { 0.0, 10.2 }, { 60.0, 3.2 }, { 124.0, 0.0 }
        };
        const enginelab::JsonEngineSerializer jsonSer;
        const auto jsonRt = jsonSer.decode(jsonSer.encode(customCrankConfig));
        require(jsonRt && std::abs(jsonRt.config->cylinders[0].crankOffsetDegrees - 90.0) < 0.01,
                "JSON round trip must preserve custom crankOffsetDegrees");
        require(jsonRt && jsonRt.config->cylinders[0].crankJournalId == 10
                && std::abs(jsonRt.config->cylinders[0].bankOffsetDegrees + 12.0) < 0.01
                && jsonRt.config->cylinders[0].intakeValveCount == 2
                && jsonRt.config->intake.throttleCount == 4
                && std::abs(jsonRt.config->cylinders[0].intakeValveDiameterMm - 34.5) < 0.01
                && jsonRt.config->camshafts.intakeLiftProfile.size() == 5,
                "JSON round trip must preserve journals, banks, valve geometry and lift tables");
        const enginelab::YamlEngineSerializer yamlSer;
        const auto yamlRt = yamlSer.decode(yamlSer.encode(customCrankConfig));
        require(yamlRt && std::abs(yamlRt.config->cylinders[1].crankOffsetDegrees - 270.0) < 0.01,
                "YAML round trip must preserve custom crankOffsetDegrees");
        require(yamlRt && yamlRt.config->cylinders[1].crankJournalId == 20
                && yamlRt.config->cylinders[0].exhaustValveCount == 2
                && yamlRt.config->intake.throttleCount == 4
                && std::abs(yamlRt.config->cylinders[0].exhaustValveDiameterMm - 29.0) < 0.01
                && std::abs(yamlRt.config->camshafts.intakeFlowCoefficient - 0.70) < 0.001,
                "YAML round trip must preserve journals, valve geometry and flow coefficients");
    }

    {
        enginelab::EngineState lowSpeed;
        lowSpeed.rpm = 2'000.0;
        lowSpeed.manifoldPressureKpa = 95.0;
        lowSpeed.coolantTemperatureC = 20.0;
        auto highSpeed = lowSpeed;
        highSpeed.rpm = 6'000.0;
        enginelab::EcuCommand enriched;
        enriched.targetAirFuelRatio = 14.2;
        enriched.fuelCorrection = 1.12;
        const auto lowHeat = physics.evaluateCombustion(config, lowSpeed, { true, false, 0.9, 0.8 }, enriched, 104.0);
        const auto highHeat = physics.evaluateCombustion(config, highSpeed, { true, false, 0.9, 0.8 }, enriched, 104.0);
        require(lowHeat.actualAirFuelRatio < enriched.targetAirFuelRatio - 1.0,
                "reported AFR must include warm-up fuel correction");
        require(highHeat.heatPowerKw > lowHeat.heatPowerKw * 2.3,
                "thermal power must scale with combustion cycles per second");
        enriched.fuelEnabled = false;
        const auto disabled = physics.evaluateCombustion(config, highSpeed, {}, enriched, 104.0);
        require(!disabled.combustionEnabled && disabled.pressureEstimateBar == 0.0,
                "disabled combustion must not report fake cylinder pressure");
    }

    {
        auto openConfig = config;
        openConfig.exhaust.collectorDiameterMm = 100.0;
        openConfig.exhaust.outletDiameterMm = 100.0;
        openConfig.exhaust.mufflerRestriction = 0.0;
        auto restrictedConfig = config;
        restrictedConfig.exhaust.collectorDiameterMm = 25.0;
        restrictedConfig.exhaust.outletDiameterMm = 25.0;
        restrictedConfig.exhaust.mufflerRestriction = 1.0;
        const auto openExhaust = enginelab::ExhaustGraph::makeForEngine(openConfig);
        const auto restrictedExhaust = enginelab::ExhaustGraph::makeForEngine(restrictedConfig);
        enginelab::EngineState flowState;
        flowState.rpm = 6'000.0;
        flowState.fuelFlowGramsPerSecond = 18.0;
        require(restrictedExhaust.backPressureKpa(flowState) > openExhaust.backPressureKpa(flowState) + 20.0,
                "collector and outlet diameters must materially change back pressure");
        flowState.exhaustRunnerPressureKpa = 180.0;
        const auto pulsedBackPressure = openExhaust.backPressureKpa(flowState);
        require(pulsedBackPressure > openConfig.ambientPressureKpa + 2.0
                    && pulsedBackPressure < openConfig.ambientPressureKpa + 8.0,
                "runner pressure pulses must feed but not dominate exhaust back pressure");

        auto splitConfig = config;
        splitConfig.exhaustPaths.clear();
        auto shortPath = enginelab::ExhaustPathConfig { 1, { 1, 2 }, splitConfig.exhaust, {}, 1.0 };
        auto longPath = enginelab::ExhaustPathConfig { 2, { 3, 4 }, splitConfig.exhaust, {}, 0.7 };
        shortPath.geometry.primaryLengthMm = 320.0;
        longPath.geometry.primaryLengthMm = 980.0;
        splitConfig.exhaustPaths = { shortPath, longPath };
        auto splitGraph = enginelab::ExhaustGraph::makeForEngine(splitConfig);
        enginelab::FiringEvent shortEvent, longEvent;
        shortEvent.exhaustPortId = 1; shortEvent.intensity = 1.0F;
        longEvent.exhaustPortId = 3; longEvent.intensity = 1.0F;
        splitGraph.process(shortEvent); splitGraph.process(longEvent);
        require(longEvent.exhaustDelaySeconds > shortEvent.exhaustDelaySeconds + 0.001F
                && longEvent.exhaustTransmissionGain < shortEvent.exhaustTransmissionGain
                && longEvent.intensity == shortEvent.intensity,
                "per-path exhaust geometry and audio attenuation must affect timing and level");

        auto arbitraryIds = config;
        arbitraryIds.cylinders[0].id = 100;
        arbitraryIds.cylinders[1].id = 200;
        arbitraryIds.cylinders[2].id = 300;
        arbitraryIds.cylinders[3].id = 400;
        arbitraryIds.firingOrder = { 100, 300, 400, 200 };
        auto arbitraryGraph = enginelab::ExhaustGraph::makeForEngine(arbitraryIds);
        enginelab::FiringEvent pathEvent;
        pathEvent.exhaustPortId = 100;
        pathEvent.intensity = 1.0F;
        arbitraryGraph.process(pathEvent);
        require(pathEvent.exhaustDelaySeconds > static_cast<float>(arbitraryIds.exhaust.primaryLengthMm / 520'000.0),
                "exhaust propagation must traverse the complete path for arbitrary cylinder IDs");
    }

    {
        auto manyCylinderConfig = config;
        manyCylinderConfig.name = "32 cylinder event stress";
        manyCylinderConfig.cylinders.clear();
        manyCylinderConfig.firingOrder.clear();
        manyCylinderConfig.banks.clear();
        manyCylinderConfig.exhaustPaths.clear();
        manyCylinderConfig.crankJournals.clear();
        manyCylinderConfig.crankshafts.clear();
        manyCylinderConfig.intakePaths.clear();
        manyCylinderConfig.rotatingInertiaKgM2 = 1.5;
        for (std::uint32_t index = 0; index < 32; ++index) {
            auto cylinder = config.cylinders.front();
            cylinder.id = 100U + index;
            manyCylinderConfig.cylinders.push_back(cylinder);
            manyCylinderConfig.firingOrder.push_back(cylinder.id);
        }
        enginelab::normaliseEngineConfig(manyCylinderConfig);
        require(!enginelab::validateEngineConfig(manyCylinderConfig), "32-cylinder stress configuration must be valid");
        enginelab::FourStrokeEventGenerator stressGenerator;
        enginelab::EngineState stressState;
        stressState.rpm = 20'000.0;
        stressState.throttle = 1.0;
        stressState.cylinderStateCount = 32;
        for (std::size_t index = 0; index < 32; ++index)
            stressState.cylinderStates[index].id = manyCylinderConfig.cylinders[index].id;
        enginelab::EcuCommand stressCommand;
        enginelab::CombustionResult stressCombustion;
        stressCombustion.combustionQuality = 1.0;
        stressCombustion.actualAirFuelRatio = 13.0;
        std::array<enginelab::FiringEvent, enginelab::maxEventsPerSimulationStep> stressEvents {};
        const auto stressCount = stressGenerator.generate(manyCylinderConfig, stressState, stressCommand,
            stressCombustion, 0.0, 0.0, 6'000.0, 0.05, stressEvents);
        std::set<std::uint32_t> stressIds;
        for (std::size_t index = 0; index < stressCount; ++index) stressIds.insert(stressEvents[index].cylinderId);
        require(stressCount < stressEvents.size() && stressIds.size() == 32,
                "validated worst-case event generation must fit and preserve every cylinder");
        std::array<enginelab::FiringEvent, 4> tinyEventBuffer {};
        const auto tinyCount = stressGenerator.generate(manyCylinderConfig, stressState, stressCommand,
            stressCombustion, 0.0, 0.0, 6'000.0, 0.05, tinyEventBuffer);
        require(tinyCount == tinyEventBuffer.size() && stressGenerator.droppedEventCountLastGenerate() > 0,
                "caller-provided event buffer overflow must be explicitly observable");
        for (std::size_t index = 1; index < tinyCount; ++index)
            require(tinyEventBuffer[index - 1].timeSeconds <= tinyEventBuffer[index].timeSeconds,
                    "truncated event output must retain the earliest chronological events");
        stressGenerator.reset();
        const auto emptyCount = stressGenerator.generate(manyCylinderConfig, stressState, stressCommand,
            stressCombustion, 0.0, 0.0, 6'000.0, 0.05, std::span<enginelab::FiringEvent> {});
        require(emptyCount == 0 && stressGenerator.droppedEventCountLastGenerate() == stressCount,
                "a saturated caller must still count every subsequent firing crossing as dropped");
    }

    {
        // Compressor and turbine broadband sources are spatially distinct.
        // The former shared random sequence (with one source negated) cancelled
        // equal models sample-for-sample, making a physically active turbo
        // silently disappear. Each physical source now owns a deterministic,
        // independent noise stream.
        enginelab::ForcedInductionConfig broadbandConfig;
        broadbandConfig.enabled = true;
        broadbandConfig.type = enginelab::ForcedInductionType::turbocharger;
        broadbandConfig.compressorBladeCount = 0;
        broadbandConfig.turbineBladeCount = 0;
        broadbandConfig.compressorInducerDiameterMm = 50.0;
        broadbandConfig.turbineExducerDiameterMm = 50.0;
        broadbandConfig.wastegateFlowAreaMm2 = 0.0;
        broadbandConfig.blowOffValveFlowAreaMm2 = 0.0;
        broadbandConfig.tonalAcousticEfficiency = 0.0;
        broadbandConfig.turbulentJetNoiseCoefficient = 0.01;
        enginelab::ForcedInductionAcoustics broadband(broadbandConfig);
        require(broadband.prepare(48'000.0),
                "forced-induction broadband fixture must prepare");
        enginelab::ForcedInductionAcoustics::Input broadbandInput;
        broadbandInput.correctedAirFlowKgPerSecond = 0.08F;
        broadbandInput.exhaustMassFlowKgPerSecond = 0.08F;
        broadbandInput.densityKgPerM3 = 1.2F;
        broadbandInput.soundSpeedMps = 343.0F;
        double broadbandEnergy = 0.0;
        for (int sample = 0; sample < 8'192; ++sample) {
            const auto output = broadband.process(broadbandInput);
            broadbandEnergy += static_cast<double>(output) * output;
        }
        require(std::sqrt(broadbandEnergy / 8'192.0) > 1.0e-6,
                "equal compressor and turbine broadband sources must not cancel");

        // Telemetry arrives at the simulation-frame cadence. A parameter step
        // must be reconstructed inside the audio model; otherwise the tone
        // amplitude jumps every 200 samples and creates a 240 Hz sideband comb.
        enginelab::ForcedInductionConfig toneConfig;
        toneConfig.enabled = true;
        toneConfig.type = enginelab::ForcedInductionType::turbocharger;
        toneConfig.compressorBladeCount = 6;
        toneConfig.turbineBladeCount = 0;
        toneConfig.tonalAcousticEfficiency = 1.0e-6;
        toneConfig.turbulentJetNoiseCoefficient = 0.0;
        enginelab::ForcedInductionAcoustics tone(toneConfig);
        require(tone.prepare(48'000.0),
                "forced-induction tone fixture must prepare");
        enginelab::ForcedInductionAcoustics::Input toneInput;
        toneInput.shaftSpeedRpm = 10'000.0F; // 1 kHz at six blades
        toneInput.compressorPowerWatts = 1'000.0F;
        toneInput.densityKgPerM3 = 1.2F;
        toneInput.soundSpeedMps = 343.0F;
        std::array<float, 260> toneSamples {};
        for (std::size_t sample = 0; sample < toneSamples.size(); ++sample) {
            if (sample == 200) toneInput.compressorPowerWatts = 4'000.0F;
            toneSamples[sample] = tone.process(toneInput);
        }
        double naturalAdjacentDelta = 0.0;
        for (std::size_t sample = 150; sample < 199; ++sample)
            naturalAdjacentDelta = std::max(naturalAdjacentDelta,
                std::abs(static_cast<double>(toneSamples[sample + 1] - toneSamples[sample])));
        const auto telemetryBoundaryDelta = std::abs(
            static_cast<double>(toneSamples[200] - toneSamples[199]));
        require(telemetryBoundaryDelta <= naturalAdjacentDelta * 2.0,
                "forced-induction telemetry steps must not create an audio discontinuity");

        enginelab::ForcedInductionConfig splitConfig;
        splitConfig.enabled = true;
        splitConfig.turbineFlowAreaMm2 = 700.0;
        splitConfig.wastegateFlowAreaMm2 = 350.0;
        splitConfig.compressorBladeCount = 1;
        enginelab::ForcedInductionAcoustics flowPartition(splitConfig);
        const auto closed = flowPartition.partitionExhaustFlow(0.12, 0.0F);
        const auto open = flowPartition.partitionExhaustFlow(0.12, 1.0F);
        require(std::abs(closed.turbineKgPerSecond - 0.12) < 1.0e-12
                    && std::abs(closed.wastegateKgPerSecond) < 1.0e-12,
                "closed wastegate must route all exhaust through the turbine");
        require(std::abs(open.turbineKgPerSecond - 0.08) < 1.0e-12
                    && std::abs(open.wastegateKgPerSecond - 0.04) < 1.0e-12
                    && std::abs(open.turbineKgPerSecond
                        + open.wastegateKgPerSecond - 0.12) < 1.0e-12,
                "open turbine/wastegate flow split must follow area and conserve mass");
    }

    {
        auto audioQueue = std::make_unique<enginelab::FiringEventQueue>();
        enginelab::RealtimeAudioState audioState;
        audioState.saturationDrive.store(4.0F);
        enginelab::RealtimeEngineAudio renderer(
            *audioQueue, audioState, nullptr, nullptr, nullptr, nullptr);
        renderer.prepare(48'000.0, 256);
        enginelab::FiringEvent event;
        event.timeSeconds = 0.0;
        event.intensity = 0.8F;
        event.pressureEstimateBar = 60.0F;
        event.combustionDurationMs = 5.0F;
        event.exhaustResonanceHz = 240.0F;
        event.airFuelRatio = 12.8F;
        require(audioQueue->tryPush(event), "audio timing event must enter realtime queue");
        juce::AudioBuffer<float> buffer(2, 1'200);
        renderer.render(buffer, 0, buffer.getNumSamples());
        int firstAudible = -1;
        for (int sample = 0; sample < buffer.getNumSamples(); ++sample)
            if (std::abs(buffer.getSample(0, sample)) > 1.0e-7F) { firstAudible = sample; break; }
        require(firstAudible >= 955 && firstAudible <= 970,
                "audio event must be rendered at its sample-accurate scheduled offset");
        require(buffer.getMagnitude(0, firstAudible, buffer.getNumSamples() - firstAudible) > 1.0e-4F,
                "combustion audio must produce a sustained audible impulse tail");
        require(renderer.saturationProcessedSampleCount() == 0,
                "physical-reference monitoring must bypass authored saturation exactly");
        audioState.monitorMode.store(static_cast<int>(
            enginelab::AudioMonitorMode::captureVoiced));
        juce::AudioBuffer<float> captureBuffer(2, 256);
        renderer.render(captureBuffer, 0, captureBuffer.getNumSamples());
        require(renderer.saturationProcessedSampleCount() > 0,
                "capture monitoring must retain the explicitly requested voicing saturation");
    }

    {
        auto scaledQueue = std::make_unique<enginelab::FiringEventQueue>();
        enginelab::RealtimeAudioState scaledState;
        scaledState.rpm.store(3'000.0F);
        scaledState.timeScale.store(1.0F);
        enginelab::RealtimeEngineAudio scaledRenderer(
            *scaledQueue, scaledState, nullptr, nullptr, nullptr, nullptr);
        scaledRenderer.prepare(48'000.0, 256);
        juce::AudioBuffer<float> audible(2, 4'800);
        scaledRenderer.render(audible, 0, audible.getNumSamples());
        require(audible.getMagnitude(0, 0, audible.getNumSamples()) > 1.0e-5F,
                "mechanical audio layer must follow running RPM");

        auto pausedQueue = std::make_unique<enginelab::FiringEventQueue>();
        enginelab::RealtimeAudioState pausedState;
        pausedState.rpm.store(3'000.0F);
        pausedState.timeScale.store(0.0F);
        enginelab::RealtimeEngineAudio pausedRenderer(
            *pausedQueue, pausedState, nullptr, nullptr, nullptr, nullptr);
        pausedRenderer.prepare(48'000.0, 256);
        juce::AudioBuffer<float> silent(2, 4'800);
        pausedRenderer.render(silent, 0, silent.getNumSamples());
        require(silent.getMagnitude(0, 0, silent.getNumSamples()) < 1.0e-7F,
                "paused time scale must silence continuous mechanical layers");

        enginelab::FiringEvent overflowEvent;
        overflowEvent.intensity = 0.8F;
        overflowEvent.exhaustDelaySeconds = 0.01F;
        for (int index = 0; index < 600; ++index) {
            overflowEvent.timeSeconds = static_cast<double>(index) * 0.0001;
            require(scaledQueue->tryPush(overflowEvent), "audio overflow fixture must enter queue");
        }
        juce::AudioBuffer<float> shortBuffer(2, 64);
        scaledRenderer.render(shortBuffer, 0, shortBuffer.getNumSamples());
        require(scaledRenderer.droppedPendingEventCount() > 0,
                "pending audio saturation must be observable instead of blocking the producer queue");

        auto flowQueue = std::make_unique<enginelab::FiringEventQueue>();
        enginelab::RealtimeAudioState flowAudioState;
        flowAudioState.monitorMode.store(static_cast<int>(
            enginelab::AudioMonitorMode::captureVoiced));
        flowAudioState.combustionGain.store(0.0F);
        flowAudioState.intakeGain.store(0.0F);
        flowAudioState.mechanicalGain.store(0.0F);
        flowAudioState.exhaustFlowGramsPerSecond.store(120.0F);
        flowAudioState.exhaustPressureKpa.store(155.0F);
        enginelab::RealtimeEngineAudio flowRenderer(
            *flowQueue, flowAudioState, nullptr, nullptr, nullptr, nullptr);
        flowRenderer.prepare(48'000.0, 256);
        juce::AudioBuffer<float> physicalFlowAudio(2, 4'800);
        flowRenderer.render(physicalFlowAudio, 0, physicalFlowAudio.getNumSamples());
        require(physicalFlowAudio.getMagnitude(0, 0, physicalFlowAudio.getNumSamples()) > 1.0e-6F,
                "continuous exhaust audio must respond to simulated collector pressure and outlet mass flow");
    }

    {
        auto presetQueue = std::make_unique<enginelab::FiringEventQueue>();
        enginelab::RealtimeAudioState presetState;
        presetState.monitorMode.store(static_cast<int>(
            enginelab::AudioMonitorMode::captureVoiced));
        presetState.rpm.store(3'600.0F);
        presetState.throttle.store(0.8F);
        presetState.load.store(0.5F);
        presetState.combustionGain.store(0.0F);
        presetState.intakeGain.store(0.0F);
        presetState.mechanicalGain.store(0.0F);
        enginelab::FiringEvent event;
        event.timeSeconds = 0.0;
        event.intensity = 0.9F;
        event.pressureEstimateBar = 72.0F;
        event.combustionDurationMs = 5.8F;
        event.exhaustDelaySeconds = 0.008F;
        event.exhaustResonanceHz = 220.0F;
        event.airFuelRatio = 12.9F;

        presetState.exhaustPreset.store(static_cast<int>(enginelab::AudioExhaustPreset::openHeaders));
        enginelab::RealtimeEngineAudio openRenderer(
            *presetQueue, presetState, nullptr, nullptr, nullptr, nullptr);
        openRenderer.prepare(48'000.0, 256);
        require(presetQueue->tryPush(event), "open-header fixture event must enter queue");
        juce::AudioBuffer<float> openBuffer(2, 4'800);
        openRenderer.render(openBuffer, 0, openBuffer.getNumSamples());
        const auto openMagnitude = openBuffer.getMagnitude(0, 0, openBuffer.getNumSamples());

        auto mutedQueue = std::make_unique<enginelab::FiringEventQueue>();
        enginelab::RealtimeAudioState mutedState;
        mutedState.monitorMode.store(static_cast<int>(
            enginelab::AudioMonitorMode::captureVoiced));
        mutedState.combustionGain.store(0.0F);
        mutedState.exhaustGain.store(0.0F);
        mutedState.intakeGain.store(0.0F);
        mutedState.mechanicalGain.store(0.0F);
        enginelab::RealtimeEngineAudio mutedRenderer(
            *mutedQueue, mutedState, nullptr, nullptr, nullptr, nullptr);
        mutedRenderer.prepare(48'000.0, 256);
        require(mutedQueue->tryPush(event), "muted fixture event must enter queue");
        juce::AudioBuffer<float> mutedBuffer(2, 4'800);
        mutedRenderer.render(mutedBuffer, 0, mutedBuffer.getNumSamples());
        require(mutedBuffer.getMagnitude(0, 0, mutedBuffer.getNumSamples()) < openMagnitude * 0.05F,
                "live mixer gains must be able to mute audio layers");

        auto turboQueue = std::make_unique<enginelab::FiringEventQueue>();
        enginelab::RealtimeAudioState turboState;
        turboState.monitorMode.store(static_cast<int>(
            enginelab::AudioMonitorMode::captureVoiced));
        turboState.rpm.store(3'600.0F);
        turboState.throttle.store(0.8F);
        turboState.load.store(0.5F);
        turboState.combustionGain.store(0.0F);
        turboState.intakeGain.store(0.0F);
        turboState.mechanicalGain.store(0.0F);
        turboState.exhaustPreset.store(static_cast<int>(enginelab::AudioExhaustPreset::turboMuffled));
        enginelab::RealtimeEngineAudio turboRenderer(
            *turboQueue, turboState, nullptr, nullptr, nullptr, nullptr);
        turboRenderer.prepare(48'000.0, 256);
        require(turboQueue->tryPush(event), "turbo fixture event must enter queue");
        juce::AudioBuffer<float> turboBuffer(2, 4'800);
        turboRenderer.render(turboBuffer, 0, turboBuffer.getNumSamples());
        const auto turboMagnitude = turboBuffer.getMagnitude(0, 0, turboBuffer.getNumSamples());
        require(std::abs(openMagnitude - turboMagnitude) > 1.0e-4F,
                "exhaust presets must produce observably different impulse responses");

        auto directIrQueue = std::make_unique<enginelab::FiringEventQueue>();
        enginelab::RealtimeAudioState directIrState;
        directIrState.monitorMode.store(static_cast<int>(
            enginelab::AudioMonitorMode::captureVoiced));
        directIrState.combustionGain.store(0.0F);
        directIrState.intakeGain.store(0.0F);
        directIrState.mechanicalGain.store(0.0F);
        enginelab::RealtimeEngineAudio directIrRenderer(
            *directIrQueue, directIrState, nullptr, nullptr, nullptr, nullptr);
        const std::array<float, 1> directIr { 1.0F };
        directIrRenderer.setImpulseResponse(directIr);
        directIrRenderer.prepare(48'000.0, 256);
        require(directIrQueue->tryPush(event), "direct IR fixture event must enter queue");
        juce::AudioBuffer<float> directIrBuffer(2, 4'800);
        directIrRenderer.render(directIrBuffer, 0, directIrBuffer.getNumSamples());

        auto delayedIrQueue = std::make_unique<enginelab::FiringEventQueue>();
        enginelab::RealtimeAudioState delayedIrState;
        delayedIrState.monitorMode.store(static_cast<int>(
            enginelab::AudioMonitorMode::captureVoiced));
        delayedIrState.combustionGain.store(0.0F);
        delayedIrState.intakeGain.store(0.0F);
        delayedIrState.mechanicalGain.store(0.0F);
        enginelab::RealtimeEngineAudio delayedIrRenderer(
            *delayedIrQueue, delayedIrState, nullptr, nullptr, nullptr, nullptr);
        std::array<float, 128> delayedIr {};
        delayedIr[0] = 0.15F;
        delayedIr[96] = 0.85F;
        delayedIrRenderer.setImpulseResponse(delayedIr, 48'000.0, 1);
        delayedIrRenderer.prepare(48'000.0, 256);
        auto secondPathEvent = event;
        secondPathEvent.exhaustPathIndex = 1;
        require(delayedIrQueue->tryPush(secondPathEvent), "delayed IR fixture event must enter queue");
        juce::AudioBuffer<float> delayedIrBuffer(2, 4'800);
        delayedIrRenderer.render(delayedIrBuffer, 0, delayedIrBuffer.getNumSamples());
        double irDifference = 0.0;
        for (int sample = 0; sample < directIrBuffer.getNumSamples(); ++sample)
            irDifference += std::abs(directIrBuffer.getSample(0, sample) - delayedIrBuffer.getSample(0, sample));
        require(irDifference > 0.01,
                "a path-routed partitioned FIR response must materially change the exhaust signature");
    }

    {
        const auto validIr = enginelab::loadImpulseResponseFile(
            juce::File(juce::String(ENGINELAB_CATALOG_ROOT))
                .getChildFile("assets")
                .getChildFile("ir")
                .getChildFile("exhaust_default.wav"));
        require(validIr.ok() && validIr.sampleRateHz > 0.0
                && validIr.samples.getNumChannels() >= 1,
                "the committed downstream IR must decode before publication");

        const auto missingIr = enginelab::loadImpulseResponseFile(
            juce::File(juce::String(ENGINELAB_CATALOG_ROOT))
                .getChildFile("assets")
                .getChildFile("ir")
                .getChildFile("does-not-exist.wav"));
        require(!missingIr.ok()
                && missingIr.error == enginelab::ImpulseResponseLoadError::missingFile,
                "a missing explicit IR must remain an observable load error");

        auto corruptFile = juce::File::createTempFile(".wav");
        require(corruptFile.replaceWithText("not a wave file"),
                "the corrupt-IR fixture must be created");
        const auto corruptIr = enginelab::loadImpulseResponseFile(corruptFile);
        require(!corruptIr.ok()
                && corruptIr.error == enginelab::ImpulseResponseLoadError::unsupportedOrCorrupt,
                "a corrupt explicit IR must not be accepted as field-free audio");
        require(corruptFile.deleteFile(), "the corrupt-IR fixture must be removed");
    }

    {
        auto pressureEventQueue = std::make_unique<enginelab::FiringEventQueue>();
        enginelab::RealtimeAudioState pressureAudioState;
        auto pressureQueue = std::make_unique<enginelab::CylinderPressureQueue>();
        enginelab::CylinderPressureSample pressure0;
        pressure0.timeSeconds = 0.0;
        pressure0.cylinderCount = 4;
        pressure0.pressureBar[0] = 42.0F;
        pressure0.pressureBar[1] = 3.0F;
        pressure0.pressureBar[2] = 1.1F;
        pressure0.pressureBar[3] = 1.0F;
        auto pressure1 = pressure0;
        pressure1.timeSeconds = 0.004;
        pressure1.pressureBar[0] = 5.0F;
        pressure1.pressureBar[1] = 38.0F;
        require(pressureQueue->tryPush(pressure0) && pressureQueue->tryPush(pressure1),
                "pressure audio queue must accept thermodynamic substeps");
        enginelab::RealtimeEngineAudio pressureRenderer(
            *pressureEventQueue, pressureAudioState, pressureQueue.get(),
            nullptr, nullptr, nullptr);
        pressureRenderer.prepare(48'000.0, 512);
        juce::AudioBuffer<float> pressureBuffer(2, 512);
        pressureRenderer.render(pressureBuffer, 0, pressureBuffer.getNumSamples());
        double pressureEnergy = 0.0;
        for (int sample = 0; sample < pressureBuffer.getNumSamples(); ++sample)
            pressureEnergy += std::abs(pressureBuffer.getSample(0, sample));
        require(pressureEnergy > 0.01,
                "continuous chamber-pressure samples must produce audio without a firing event");
    }

    {
        enginelab::SpscQueue<int, 8> saturationQueue;
        for (int index = 0; index < 7; ++index) require(saturationQueue.tryPush(index), "SPSC must accept usable capacity");
        require(!saturationQueue.tryPush(8), "SPSC must report saturation without overwriting unread data");
        for (int index = 0; index < 7; ++index) {
            int item = -1;
            require(saturationQueue.tryPop(item) && item == index, "SPSC saturation must preserve FIFO ordering");
        }
    }

    const auto finiteFrame = simulator.step(std::numeric_limits<double>::quiet_NaN(),
        { true, false, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN() });
    require(std::isfinite(finiteFrame.state.rpm) && std::isfinite(finiteFrame.state.throttle)
            && std::isfinite(finiteFrame.state.load), "non-finite controls must not poison simulation state");
    const auto calibrationTimeBefore = simulator.state().simulationTimeSeconds;
    const auto calibrationRpmBefore = simulator.state().rpm;
    enginelab::AudioPhysicsCalibration liveCalibration;
    liveCalibration.cycleVariationCoefficientOfVariation = 0.035;
    liveCalibration.cycleVariationCorrelation = 0.61;
    liveCalibration.limiterKeepsFuel = true;
    liveCalibration.exhaustAfterfire = config.exhaustAfterfire;
    simulator.applyAudioPhysicsCalibration(liveCalibration);
    const auto calibrationFrame = simulator.step(
        1.0 / 240.0, { true, false, 0.25, 0.08 });
    require(calibrationFrame.state.simulationTimeSeconds
                > calibrationTimeBefore
            && std::abs(calibrationFrame.state.rpm - calibrationRpmBefore)
                < std::max(400.0, calibrationRpmBefore * 0.25),
        "live audio-physics calibration must preserve simulation time and rotating state");
    enginelab::CylinderPressureSample discardedPressureSample;
    while (simulator.tryPopCylinderPressureSample(discardedPressureSample)) {}
    simulator.setPressureSamplingEnabled(true);
    const auto pressureFrame = simulator.step(1.0 / 240.0, { true, false, 0.4, 0.1 });
    require(pressureFrame.cylinderPressureSampleCount > 0
            && pressureFrame.cylinderPressureSampleCount == pressureFrame.state.solverSubsteps,
            "simulator must publish one chamber-pressure frame per thermodynamic substep");
    bool observedPhysicalExhaustBoundary = false;
    enginelab::CylinderPressureSample physicalPressureSample;
    while (simulator.tryPopCylinderPressureSample(physicalPressureSample)) {
        for (std::size_t index = 0; index < physicalPressureSample.cylinderCount; ++index) {
            if (physicalPressureSample.thermoacousticBoundaryValid[index] == 0) continue;
            observedPhysicalExhaustBoundary = true;
            require(std::isfinite(physicalPressureSample.exhaustMassFlowKgPerSecond[index])
                    && physicalPressureSample.exhaustRunnerPressureKpa[index] > 0.0F
                    && physicalPressureSample.exhaustPortDensityKgPerM3[index] > 0.0F
                    && physicalPressureSample.exhaustPortSpeedOfSoundMps[index] > 0.0F,
                "every valid thermoacoustic boundary must contain finite SI gas state");
        }
    }
    require(observedPhysicalExhaustBoundary,
        "multirate exhaust coupling must retain a physical boundary sample at mechanical cadence");

    // Duct media/outlet states are consumed once per audio block, not at the
    // gas-network's ~8 kHz coupling rate. In a non-reacting run the simulator
    // must therefore publish them at product-frame cadence; afterfire sources
    // independently force immediate packets and are covered by the exact-node
    // reaction tests above. This guards the CPU regression where a large zeroed
    // telemetry object was rebuilt dozens of times before one consumer block.
    enginelab::ExhaustAcousticSample discardedAcousticSample;
    while (simulator.tryPopExhaustAcousticSample(discardedAcousticSample)) {}
    auto acousticSampleFrames = std::size_t { 0 };
    constexpr auto acousticCadenceProbeFrames = std::size_t { 24 };
    for (std::size_t frameIndex = 0;
         frameIndex < acousticCadenceProbeFrames; ++frameIndex) {
        const auto frame = simulator.step(
            1.0 / 240.0, { true, false, 0.4, 0.1 });
        acousticSampleFrames += frame.exhaustAcousticSampleCount;
    }
    auto poppedAcousticSamples = std::size_t { 0 };
    while (simulator.tryPopExhaustAcousticSample(discardedAcousticSample))
        ++poppedAcousticSamples;
    require(acousticSampleFrames > 0
            && acousticSampleFrames == poppedAcousticSamples
            && acousticSampleFrames <= acousticCadenceProbeFrames + 1,
        "non-reacting exhaust media telemetry must stay at product-frame cadence");

    {
        auto invalidConfig = config;
        invalidConfig.firingOrder.back() = 99;
        bool rejected = false;
        try {
            enginelab::EngineSimulator invalidSimulator(invalidConfig, ecu, physics, events, exhaust);
        } catch (const std::invalid_argument&) { rejected = true; }
        require(rejected, "simulator must reject firing orders that reference unknown cylinders");
    }

    {
        // The user-facing bench must be a continuous ramp, not a 250 rpm
        // ladder. What separates the two is not the speed of the sweep but its
        // SPACING: the stepped sweep advances the setpoint by exactly 250 rpm
        // after each settled window, so its points land on a ladder, while a
        // ramp is paced continuously while its recorder publishes an explicit
        // 50 rpm grid independently of the rolling measurement cadence.
        //
        // Both modes have to keep working. The stepped sweep stays the
        // calibration instrument -- EngineLab.CatalogReference measures its 24
        // manufacturer points with it, in steady state -- so it must not be
        // quietly replaced.
        auto rampOwner = std::make_unique<enginelab::EngineRuntime>(
            enginelab::makeDefaultInlineTwo());
        auto& rampRuntime = *rampOwner;
        rampRuntime.setRealtimeThrottleEnabled(false);
        require(rampRuntime.dynoRampEnabled(),
                "the user-facing dyno must default to a continuous ramp");
        rampRuntime.setDynoHoldEnabled(true);
        require(rampRuntime.dynoHoldEnabled()
                && !rampRuntime.dynoRampEnabled(),
                "dyno modes must be mutually exclusive");
        rampRuntime.setDynoRampEnabled(true);
        require(rampRuntime.dynoRampEnabled()
                && !rampRuntime.dynoHoldEnabled(),
                "selecting ramp must atomically replace hold mode");
        // The recorder is now independent of its rolling window: it publishes
        // one explicit valid/invalid record on an exact 50 rpm grid. Keep a
        // low rate so this regression finishes quickly without stressing the
        // controller rather than using cadence as an accidental discriminator.
        rampRuntime.setDynoRampRpmPerSecond(150.0);
        rampRuntime.setDynoMaximumDurationSeconds(45.0);
        rampRuntime.setIgnitionEnabled(true);
        rampRuntime.start();
        rampRuntime.startDyno();
        // Once start has accepted a protocol, menu/key commands are ignored.
        // They cannot relabel or retune an in-flight measurement.
        rampRuntime.setDynoHoldEnabled(true);
        rampRuntime.setDynoRampRpmPerSecond(900.0);
        require(rampRuntime.dynoRampEnabled()
                && !rampRuntime.dynoHoldEnabled()
                && std::abs(rampRuntime.dynoRampRpmPerSecond() - 150.0)
                    < 1.0e-9,
                "an active dyno session must keep an immutable mode and ramp rate");
        auto rampRun = rampRuntime.currentDynoRun();
        std::array<std::uint32_t, 11> rampQualityObservations {};
        auto minimumRampContact = 1.0;
        auto lastRampTarget = 0.0;
        auto lastRampWindowMean = 0.0;
        const auto rampDeadline =
            std::chrono::steady_clock::now() + std::chrono::seconds(40);
        auto validRampPointCount = [&rampRun] {
            return static_cast<std::size_t>(std::count_if(
                rampRun.points.begin(), rampRun.points.end(),
                [](const enginelab::DynoPoint& point) {
                    return point.valid;
                }));
        }();
        while (validRampPointCount < 5
               && std::chrono::steady_clock::now() < rampDeadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            rampRun = rampRuntime.currentDynoRun();
            validRampPointCount = static_cast<std::size_t>(std::count_if(
                rampRun.points.begin(), rampRun.points.end(),
                [](const enginelab::DynoPoint& point) {
                    return point.valid;
                }));
            const auto state = rampRuntime.snapshot();
            if (state.dynoActive && !state.dynoPreparing) {
                minimumRampContact = std::min(
                    minimumRampContact,
                    state.dynoBrakeContactFraction);
                lastRampTarget = state.dynoTargetRpm;
                lastRampWindowMean = state.dynoWindowMeanRpm;
                for (std::size_t reason = 0;
                     reason < rampQualityObservations.size(); ++reason) {
                    if ((state.dynoQualityReasons & (1U << reason)) != 0U)
                        ++rampQualityObservations[reason];
                }
            }
        }
        const auto rampPointDiagnostic = "a ramp sweep must publish five valid points; it published "
            + std::to_string(validRampPointCount) + " valid / "
            + std::to_string(rampRun.points.size()) + " total";
        require(validRampPointCount >= 5, rampPointDiagnostic.c_str());
        auto rampRose = true;
        auto widestSpacingRpm = 0.0;
        for (std::size_t index = 1; index < rampRun.points.size(); ++index) {
            const auto spacing = rampRun.points[index].rpm - rampRun.points[index - 1].rpm;
            if (!(spacing > 0.0)) rampRose = false;
            widestSpacingRpm = std::max(widestSpacingRpm, spacing);
        }
        require(rampRose, "a ramp sweep must climb");
        // Invalid intervals remain explicit holes on the same grid; they must
        // never be silently skipped or joined across by the renderer.
        const auto rampSpacingDiagnostic =
            "a ramp sweep must not land on the stepped bench's 250 rpm ladder; widest spacing was "
            + std::to_string(widestSpacingRpm) + ", points="
            + [&rampRun] {
                std::string values;
                for (const auto& point : rampRun.points) {
                    if (!values.empty()) values += '/';
                    values += std::to_string(point.rpm)
                        + (point.valid ? "V" : "I");
                }
                return values;
            }() + ", reasons="
            + [&rampQualityObservations] {
                std::string values;
                for (const auto count : rampQualityObservations) {
                    if (!values.empty()) values += '/';
                    values += std::to_string(count);
                }
                return values;
            }() + ", minContact=" + std::to_string(minimumRampContact)
            + ", target=" + std::to_string(lastRampTarget)
            + ", window=" + std::to_string(lastRampWindowMean);
        require(std::abs(widestSpacingRpm - 50.0) < 1.0e-6,
                rampSpacingDiagnostic.c_str());
        require(rampRun.status == enginelab::DynoRunStatus::running
                && rampRun.calibrationRevision > 0
                && rampRun.sessionConfig.mode
                    == enginelab::DynoMode::continuousRamp
                && std::abs(rampRun.sessionConfig.rampRateRpmPerSecond
                    - 150.0) < 1.0e-9
                && std::abs(rampRun.sessionConfig.binWidthRpm - 50.0)
                    < 1.0e-9,
                "the live run must retain the exact accepted session protocol");
        const auto liveState = rampRuntime.snapshot();
        require(liveState.dynoMode == enginelab::DynoMode::continuousRamp
                && liveState.dynoRunStatus
                    == enginelab::DynoRunStatus::running
                && liveState.dynoPhase == enginelab::DynoPhase::acquiring,
                "product state must expose the active dyno mode, status and phase");
        rampRuntime.stopDyno();
        const auto stopDeadline = std::chrono::steady_clock::now()
            + std::chrono::seconds(2);
        while (rampRuntime.dynoRunning()
               && std::chrono::steady_clock::now() < stopDeadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        const auto cancelledRuns = rampRuntime.dynoHistory();
        require(!rampRuntime.dynoRunning()
                && rampRuntime.dynoRunStatus()
                    == enginelab::DynoRunStatus::cancelled
                && cancelledRuns.size() == 1
                && cancelledRuns.front().status
                    == enginelab::DynoRunStatus::cancelled
                && cancelledRuns.front().stopReason
                    == enginelab::DynoStopReason::operatorCancelled,
                "an operator-stopped ramp must be archived explicitly as cancelled");
    }

    {
        // The realtime factor is the only reading that can see the failure this
        // project actually has, which is not a dropped frame but SLOW MOTION.
        // `EngineRuntime::run` advances a fixed 1/240 s per iteration and sleeps
        // to a wall deadline, so once a step costs more wall time than it
        // advances, simulated time falls behind permanently. Users report that
        // as late controls and a silent V8 or V12, never as a timing fault, so
        // the number has to exist and has to be honest.
        // On the heap, not the stack. An EngineRuntime embeds
        // CylinderPressureQueue -- SpscQueue<CylinderPressureSample, 8192>, some
        // 7.4 MB -- and MSVC does not reliably share frame slots between sibling
        // scopes, so a SECOND stack-allocated runtime in this function overflows
        // the 8 MB stack before main() prints anything. The symptom is exit
        // 0xC00000FD in 0.02 s, which ctest reports as SegFault.
        auto runtimeOwner =
            std::make_unique<enginelab::EngineRuntime>(enginelab::makeDefaultInlineTwo());
        auto& runtime = *runtimeOwner;
        runtime.setIgnitionEnabled(true);
        runtime.setThrottle(0.35);
        runtime.start();
        std::this_thread::sleep_for(std::chrono::milliseconds(1'200));
        const auto throttledFactor = runtime.realtimeFactor();
        require(std::isfinite(throttledFactor) && throttledFactor > 0.0,
                "realtime factor must be finite and positive while running");
        // `sleep_until` is what caps it. A throttled reading above 1 would mean
        // the loop is not pacing itself at all.
        require(throttledFactor <= 1.05,
                "a throttled runtime must not report more than realtime");
        // Non-vacuity. With the throttle removed the SAME arithmetic must read
        // capacity instead, and an inline twin has several times the margin it
        // needs. A field hard-coded to 1.0 passes the assertion above and fails
        // this one.
        runtime.setRealtimeThrottleEnabled(false);
        std::this_thread::sleep_for(std::chrono::milliseconds(1'200));
        const auto freeRunningFactor = runtime.realtimeFactor();
        require(freeRunningFactor > 1.5,
                "a free-running inline twin must report capacity above realtime");
    }

    {
        enginelab::EngineRuntime runtime(enginelab::makeDefaultInlineTwo());
        runtime.setDynoMode(enginelab::DynoMode::steppedCalibration);
        runtime.setIgnitionEnabled(false);
        runtime.setThrottle(0.31);
        runtime.setLoad(0.27);
        runtime.start(); runtime.startDyno();
        // This is an outcome guard, not a wall-clock benchmark. The smoother
        // absorber contact deliberately trades a little sweep speed for a
        // stable hold on high-compression engines, and host scheduling can add
        // further wall-time variance. Wait for the required three physical
        // points, with a bounded deadline, instead of assuming one machine's
        // 4.5 s timing.
        auto liveRun = runtime.currentDynoRun();
        auto dynoMinimumRpm = std::numeric_limits<double>::max();
        auto dynoMaximumRpm = 0.0;
        auto dynoMaximumBrakeTorqueNm = 0.0;
        std::size_t dynoBrakeContactSamples = 0;
        const auto dynoProofDeadline =
            std::chrono::steady_clock::now() + std::chrono::seconds(10);
        while (liveRun.points.size() < 3
               && std::chrono::steady_clock::now() < dynoProofDeadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            const auto dynoSnapshot = runtime.snapshot();
            dynoMinimumRpm = std::min(dynoMinimumRpm, dynoSnapshot.rpm);
            dynoMaximumRpm = std::max(dynoMaximumRpm, dynoSnapshot.rpm);
            dynoMaximumBrakeTorqueNm = std::max(
                dynoMaximumBrakeTorqueNm,
                dynoSnapshot.dynoBrakeTorqueNm);
            if (dynoSnapshot.dynoBrakeTorqueNm > 1.0)
                ++dynoBrakeContactSamples;
            liveRun = runtime.currentDynoRun();
        }
        if (liveRun.points.size() < 3) {
            const auto diagnostic = runtime.snapshot();
            std::cerr << "dyno diagnostic: points=" << liveRun.points.size()
                      << " rpm=" << diagnostic.rpm
                      << " target=" << diagnostic.dynoTargetRpm
                      << " controller_target="
                      << diagnostic.dynoControllerTargetRpm
                      << " filtered_accel="
                      << diagnostic.dynoFilteredAccelerationRpmPerSecond
                      << " brake_torque=" << diagnostic.dynoBrakeTorqueNm
                      << " preparing=" << diagnostic.dynoPreparing
                      << " recoveries=" << diagnostic.dynoRecoveryCount
                      << " load_torque=" << diagnostic.loadTorqueNm
                      << " torque=" << diagnostic.cycleAveragedTorqueNm
                      << " throttle=" << diagnostic.throttle
                      << " observed_rpm=" << dynoMinimumRpm << ".."
                      << dynoMaximumRpm
                      << " max_brake=" << dynoMaximumBrakeTorqueNm
                      << " contact_samples=" << dynoBrakeContactSamples
                      << '\n';
        }
        runtime.stopDyno();
        std::this_thread::sleep_for(std::chrono::milliseconds(750));
        const auto restored = runtime.snapshot();
        runtime.setPaused(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        const auto pausedAt = runtime.snapshot().simulationTimeSeconds;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        require(std::abs(runtime.snapshot().simulationTimeSeconds - pausedAt) < 1.0e-9,
                "runtime pause must freeze deterministic simulation time");
        runtime.stop();
        const auto runs = runtime.dynoHistory();
        require(runs.size() == 1 && runs.front().points.size() >= 3, "brake dyno must record a usable curve");
        require(runs.front().peakTorqueNm > 0.0 && runs.front().peakPowerKw > 0.0,
                "dyno must calculate torque and power");
        require(runs.front().peakCorrectedTorqueNm > 0.0 && runs.front().peakCorrectedPowerKw > 0.0,
                "dyno must publish corrected peaks explicitly");
        require(runs.front().points.front().atmosphericCorrectionFactor > 0.0
                && runs.front().points.front().correctedPowerKw > 0.0,
                 "dyno must publish atmospheric correction and corrected output");
        require(runs.front().points.front().volumetricEfficiency > 0.0
                && runs.front().points.front().manifoldPressureKpa > 0.0
                && runs.front().points.front().oilPressureKpa > 0.0
                && runs.front().points.front().airFlowGramsPerSecond > 0.0
                && runs.front().points.front().lambda > 0.0,
                "dyno points must retain the physical telemetry needed to explain a result");
        require(std::abs(restored.throttle - 0.31) < 0.01
                && std::abs(restored.requestedRoadLoad - 0.27) < 0.01
                && std::abs(restored.loadTorqueNm) < 0.5,
                "dyno must restore throttle and road load without reapplying it at the crank");
    }

    {
        enginelab::EngineRuntime runtime(enginelab::makeDefaultInlineFour());
        runtime.setIgnitionEnabled(true);
        runtime.setStarterEngaged(true);
        runtime.setThrottle(0.55);
        runtime.setGear(0);
        runtime.setClutchPressure(1.0);
        runtime.start();
        std::this_thread::sleep_for(std::chrono::milliseconds(2'200));
        runtime.setStarterEngaged(false);
        runtime.shiftUp();
        std::this_thread::sleep_for(std::chrono::milliseconds(600));
        const auto state = runtime.snapshot();
        runtime.stop();
        require(state.gear == 1 && state.gearCount >= 5, "runtime must expose es2d-style gear state");
        require(state.clutchPressure > 0.9, "runtime must expose clutch pressure");
        require(state.vehicleSpeedMps >= 0.0 && std::isfinite(state.drivelineLoadTorqueNm),
                "runtime must expose finite vehicle/driveline telemetry");
        require(std::isfinite(state.clutchSlipRpm) && std::isfinite(state.clutchTorqueNm)
                && !state.shiftInProgress,
                "physical clutch slip, transmitted torque and completed shift state must be observable");
    }
    {
        auto automaticConfig = enginelab::makeDefaultInlineTwo();
        automaticConfig.transmission.automaticShifting = true;
        automaticConfig.transmission.automaticUpshiftRpm = 500.0;
        automaticConfig.transmission.automaticDownshiftRpm = 100.0;
        auto runtime = std::make_unique<enginelab::EngineRuntime>(automaticConfig);
        runtime->setIgnitionEnabled(true);
        runtime->setStarterEngaged(true);
        runtime->setThrottle(0.5);
        runtime->setClutchPressure(0.0);
        runtime->setGear(0);
        runtime->start();
        std::this_thread::sleep_for(std::chrono::milliseconds(1'500));
        const auto freeRunning = runtime->snapshot();
        require(freeRunning.gear > 0,
                "automatic transmission must upshift from configured RPM thresholds");
        runtime->setStarterEngaged(false);
        runtime->setThrottle(0.0);
        runtime->setClutchPressure(1.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(900));
        const auto stalled = runtime->snapshot();
        runtime->stop();
        require(freeRunning.rpm > 250.0 && stalled.rpm < 250.0
                && stalled.runningState == enginelab::RunningState::stopped,
                "engaging the clutch at zero vehicle speed without throttle must be able to stall the engine");
    }
        std::cout << "EngineLab core tests passed\n";
        return EXIT_SUCCESS;
    }
    catch (const std::exception& exception) {
        std::cerr << "Unhandled core test exception: " << exception.what() << '\n';
        return EXIT_FAILURE;
    }
}
