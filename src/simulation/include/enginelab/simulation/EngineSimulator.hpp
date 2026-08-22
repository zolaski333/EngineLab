#pragma once
#include <enginelab/ecu/IEcuModel.hpp>
#include <enginelab/events/IFiringEventGenerator.hpp>
#include <enginelab/exhaust/IExhaustModel.hpp>
#include <enginelab/physics/IPhysicsModel.hpp>
#include <enginelab/physics/ConservativeGasSystem.hpp>
#include <enginelab/physics/FlamePhysicsModel.hpp>
#include <enginelab/physics/CompressionIgnitionModel.hpp>
#include <enginelab/physics/FuelInjectionModel.hpp>
#include <enginelab/physics/EndGasKnockModel.hpp>
#include <enginelab/physics/IndicatedWorkModel.hpp>
#include <enginelab/physics/ValveTrainModel.hpp>
#include <enginelab/physics/HelmholtzRunnerModel.hpp>
#include <enginelab/physics/MechanicalKinematics.hpp>
#include <enginelab/physics/DuctWallHeatTransferModel.hpp>
#include <enginelab/simulation/CylinderWorkerPool.hpp>
#include <enginelab/simulation/IEngineSimulation.hpp>
#include <enginelab/events/CylinderPressureSample.hpp>
#include <enginelab/events/ExhaustAcousticSample.hpp>
#include <enginelab/foundation/SpscQueue.hpp>
#include <enginelab/gasdynamics/ExhaustGasNetwork.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <optional>
namespace enginelab {

/** Diagnostic/runtime policy knobs that do not belong in an engine file.
 *
 * Every field defaults to the production policy. Overrides exist so a harness
 * can run a same-process A/B without editing a machine-specific constant into
 * the simulator or the catalogue.
 */
struct EngineSimulatorOptions final {
    /** Number of background intake workers. The simulation thread still
     * participates, so N workers means N+1 runner participants. `0` forces the
     * serial null control. */
    std::optional<std::size_t> intakeWorkerCount;
    /** Cap the spatial cells in each intake runner. Diagnostic A/Bs use this
     * to establish a convergence point without rewriting authored geometry. */
    std::optional<std::size_t> intakeMaximumCellCount;
    /** Diagnostic spatial target for the intake FV oracle/reduced mesh.
     * Absent selects the measured production target (95 mm). */
    std::optional<double> intakeTargetCellLengthM;
    /** Fixed-point rounds used to reconstruct the shared-plenum staircase
     * before concurrent runner advances. */
    std::optional<std::size_t> intakeStaircaseRounds;
    /** Maximum conservative intake coupling interval. Absent selects the
     * measured 400 us production interval; zero restores every-mechanical-
     * substep oracle coupling. The valve boundary is time-averaged over the
     * interval and intake-valve closing forces a flush. */
    std::optional<double> intakeCouplingIntervalSeconds;
    /** Low-speed cap for the nonlinear exhaust coupling interval. Absent uses
     * 125 us (at least 4 kHz physical-boundary Nyquist); harnesses can A/B it
     * against the historical 250 us cap and the full-substep oracle. */
    std::optional<double> maximumLowSpeedExhaustCouplingSeconds;
    /** Target cell length of the nonlinear exhaust feedback mesh. Absent keeps
     * the 360 mm production scale. This is a diagnostic/convergence knob: the
     * characteristic audio network still owns audio-band propagation. */
    std::optional<double> exhaustTargetCellLengthM;
    /** Diagnostic lower-bound experiment: reset the exhaust network to ambient
     * before every coupling advance. Never use in production; it deliberately
     * discards pulse history to reveal the valve/timing-only pumping floor. */
    std::optional<bool> resetExhaustToAmbientEachCoupling;
    /** Diagnostic effective-area multipliers used by gas-exchange ablations.
     * Production leaves both absent (exactly 1). */
    std::optional<double> intakeValveAreaMultiplier;
    std::optional<double> exhaustValveAreaMultiplier;
    /** Diagnostic port plateau, expressed as effective area / valve-head area. */
    std::optional<double> exhaustMaximumHeadAreaFraction;
    /** A/B override for directed exhaust-collector momentum. Production uses it. */
    std::optional<bool> evolveExhaustJunctionAxialMomentum;
    /** Reduced intake temporal integration. Spatial reconstruction remains
     * second-order MUSCL; only the RK2 corrector stage is omitted. */
    std::optional<bool> intakeFirstOrderTimeIntegration;
    /** Diagnostic cadence for the finite-capacity intake-wall heat exchange.
     * Gas/solid energy is accumulated between updates rather than discarded. */
    std::optional<double> intakeWallHeatUpdateIntervalSeconds;
};

/** Orchestrates policies and integrates state; owns no thread and performs no audio work. */
class EngineSimulator final : public IEngineSimulation {
public:
    EngineSimulator(EngineConfig, IEcuModel&, IPhysicsModel&, IFiringEventGenerator&,
                    IExhaustModel&, EngineSimulatorOptions = {});
    [[nodiscard]] SimulationFrame step(double dtSeconds, const EngineControls&) noexcept override;
    [[nodiscard]] const EngineState& state() const noexcept override { return state_; }
    /** Apply only live audio/combustion calibration. Called by the owning
     * simulation thread; it intentionally leaves every dynamic state and
     * compiled gas network untouched. */
    void applyAudioPhysicsCalibration(
        const AudioPhysicsCalibration& calibration) noexcept;
    [[nodiscard]] std::size_t intakeWorkerCount() const noexcept {
        return intakeWorkerPool_ ? intakeWorkerPool_->workerCount() : 0U;
    }
    void setIntakeWallHeatUpdateIntervalSeconds(double seconds) noexcept {
        if (std::isfinite(seconds))
            intakeWallHeatUpdateIntervalSeconds_ =
                std::clamp(seconds, 50.0e-6, 2.0e-3);
    }
    [[nodiscard]] double intakeWallHeatUpdateIntervalSeconds() const noexcept {
        return intakeWallHeatUpdateIntervalSeconds_;
    }
    [[nodiscard]] bool tryPopCylinderPressureSample(CylinderPressureSample& sample) noexcept {
        return pressureSamples_ && pressureSamples_->tryPop(sample);
    }
    [[nodiscard]] bool tryPopExhaustAcousticSample(ExhaustAcousticSample& sample) noexcept {
        return exhaustAcousticSamples_ && exhaustAcousticSamples_->tryPop(sample);
    }
    void setPressureSamplingEnabled(bool enabled);
    /** Diagnostic oracle: advance the nonlinear exhaust network on every
     * mechanical substep instead of using the production multirate interval.
     *
     * This is intentionally opt-in and is not realtime-safe on large engines.
     * It exists so audio-band approximations can be compared with the more
     * expensive physical solution rather than calibrated against themselves.
     */
    void setExhaustCouplingEverySubstep(bool enabled) noexcept {
        exhaustCouplingEverySubstep_ = enabled;
    }
    void reset() noexcept override;
private:
    /** Compile and allocate the mandatory nonlinear exhaust network. */
    void configurePhysicalExhaustNetwork();
    /** Assemble one 1-D finite-volume runner duct per cylinder.
     *
     * Intake tuning is a wave phenomenon: a lumped runner cell has no
     * propagation delay, so nothing can arrive at the valve in phase and the
     * fill can never exceed static manifold density. Four algebraic bias
     * formulations were measured and refuted before this (docs/physics-audit.md
     * "L'inertance de runner"); what was missing is the dimension, not a
     * coefficient. Each runner reuses the exhaust's FV network machinery with
     * the intake valve as the cylinder Riemann port and the plenum as the
     * ambient reservoir behind the runner mouth.
     */
    void configurePhysicalIntakeNetworks();
    void configureIntakeWorkerPool();
    [[nodiscard]] RunningState determineRunningState(const EngineControls&) const noexcept;
    /** Headroom left on the injector's duty cycle for one cylinder, 1 = shut,
     *  0 = open for the whole 720 deg. The open fraction is measured inside the
     *  angular window, including the fractional final sub-step, so it is
     *  scaled by that window's share of the cycle. */
    [[nodiscard]] double injectorDutyHeadroom(std::size_t cylinderIndex) const noexcept {
        const auto windowSubsteps = injectorWindowSubsteps_[cylinderIndex];
        if (windowSubsteps <= 0.0) return 1.0;
        const auto windowDegrees = std::fmod(
            config_.injection.endAngleDegrees
                - config_.injection.startAngleDegrees + 720.0, 720.0);
        const auto openFraction =
            injectorOpenSubsteps_[cylinderIndex] / windowSubsteps;
        return std::clamp(
            1.0 - openFraction * windowDegrees / 720.0, 0.0, 1.0);
    }
    void accumulateCycleTelemetry(double previousAngleDegrees, double travelledDegrees,
                                  double dtSeconds, double indicatedTorqueNm,
                                  double brakeTorqueNm,
                                  SimulationFrame& frame) noexcept;
    EngineConfig config_;
    EngineSimulatorOptions options_;
    EngineKinematicsReference kinematicsReference_;
    IEcuModel& ecu_;
    IPhysicsModel& physics_;
    IFiringEventGenerator& eventGenerator_;
    IExhaustModel& exhaust_;
    EngineState state_;
    /** Immutable authored topology compiled once. Path/bank searches used to
     * run several times per cylinder per gas substep, despite none of their
     * inputs changing during a simulation. */
    std::array<std::size_t, 32> intakePathIndexByCylinder_ {};
    std::array<std::size_t, 32> exhaustPathIndexByCylinder_ {};
    std::array<double, 32> previousCylinderPhases_ {};
    std::array<double, 32> intakeRunnerPressureKpa_ {};
    /** Intake runner column velocity at the valve plane, and the stagnation head
     *  rho*u^2/2 it carries, per cylinder. Diagnostics; neither drives flow.
     *
     * A 0-D runner cell cannot represent its own inertance: the cell's
     * `bulkVelocityMps()` is a net average that does not collapse when the valve
     * shuts, so the event that actually creates ram charging is absent from it.
     * These carry the velocity referred to the runner cross-section,
     * u = mdot_valve / (rho * A_runner), which does collapse (measured 82.5 ->
     * -2.8 m/s across IVC against 40.9 -> 40.3 for the cell average). See the
     * second intake half-step in EngineSimulator.cpp for what was measured on
     * top of them and why none of it is wired. */
    std::array<double, 32> intakeValveColumnVelocityMps_ {};
    std::array<double, 32> intakePortRamKpa_ {};
    std::array<double, 32> exhaustRunnerPressureKpa_ {};
    std::array<double, 32> exhaustRunnerVelocityMps_ {};
    std::array<double, 32> exhaustRunnerTemperatureK_ {};
    /** Network-side exhaust boundary at the two most recent network solutions.
     *
     * The exhaust network is integrated on a multirate stride, so its state is
     * only new on the substeps where it is flushed. Holding that state on the
     * substeps in between makes the published boundary a zero-order-hold
     * staircase clocked at the coupling rate, which for a running engine lands
     * between roughly 2 and 4 kHz -- squarely audible, and heard as a metallic
     * tone unrelated to the engine. Keeping the last two solutions lets the
     * published boundary be reconstructed by linear interpolation across the
     * stride instead, at the cost of one coupling interval (at most 250 us) of
     * group delay on the exhaust boundary alone.
     */
    struct ExhaustNetworkBoundary final {
        double pressureKpa {};
        double velocityMps {};
        double temperatureK {};
        double massFlowKgPerSecond {};
        double densityKgPerM3 {};
        double speedOfSoundMps {};
        /** Simulation time this knot was sampled at. */
        double timeSeconds {};
        bool valid { false };
    };
    /** Timestamped ring of boundary knots, reconstructed at a CONSTANT delay.
     *
     * Coupling intervals are not uniform: the end of every frame forces a
     * flush, so the frame's last interval is short whenever the substep count
     * is not a multiple of the stride; the stride changes with engine speed;
     * and the substep duration drifts frame to frame. Any reconstruction whose
     * delay follows the current interval -- a substep counter, or even a phase
     * over the latest pair of knot times -- therefore jumps at every interval
     * change, and the frame-rate repetition of those jumps was measured as a
     * comb at exact multiples of 240 Hz across the catalogue, engine
     * independent, with firing sidebands: the dominant metallic residue.
     *
     * Publishing the boundary at one fixed delay behind now, interpolated
     * between whichever ring knots bracket that instant, is continuous by
     * construction through every interval change. The delay must exceed the
     * longest coupling interval so a bracketing pair always exists.
     */
    static constexpr std::size_t exhaustBoundaryKnotCount = 8;
    /** Above the 250 us low-speed coupling cap with slack, and about 0.6 ms of
     * group delay on the exhaust boundary alone -- inaudible as latency. */
    static constexpr double exhaustBoundaryReconstructionDelaySeconds = 625.0e-6;
    /** Multirate coupling accumulators, carried ACROSS frames.
     *
     * The network flush used to be forced on the last substep of every frame,
     * which cut the final averaging window short whenever the frame's substep
     * count was not a multiple of the coupling stride. That repeats the same
     * irregular window pattern every frame, so the network's own solution --
     * not merely its reconstruction -- was modulated at the frame rate. It was
     * measured as a comb at exact multiples of 240 Hz that survived muting
     * every other layer, replacing the IR with a unit impulse, ramping every
     * delay, and smoothing the harness dyno. Keeping the accumulation in
     * members and flushing on accumulated duration alone decouples the
     * network's integration grid from the frame grid entirely.
     */
    std::array<double, 32> exhaustValveConductanceTimeIntegralM2S_ {};
    std::array<gasdynamics::ConservativeState, 32> exhaustBoundaryStateTimeIntegral_ {};
    std::array<double, 32> exhaustBoundaryVolumeTimeIntegralM3S_ {};
    double exhaustCouplingDurationSeconds_ { 0.0 };
    double outletOpeningScaleTimeIntegralSeconds_ { 0.0 };
    bool exhaustCouplingEverySubstep_ { false };
    std::array<std::array<ExhaustNetworkBoundary, exhaustBoundaryKnotCount>, 32>
        exhaustBoundaryKnots_ {};
    std::size_t exhaustBoundaryKnotWrite_ { 0 };
    /** Length-mean acoustic medium of every compiled exhaust duct. Reaction
     * events remain coupling-rate, while these slowly varying states are
     * refreshed at the 240 Hz product-frame cadence: the audio renderer applies
     * boundaries once per 187.5 Hz block, so copying them at ~8 kHz discarded
     * CPU without increasing the delivered temporal resolution. */
    std::array<float, ExhaustAcousticSample::maximumDucts>
        exhaustDuctDensityKgPerM3_ {};
    std::array<float, ExhaustAcousticSample::maximumDucts>
        exhaustDuctSpeedOfSoundMps_ {};
    std::size_t exhaustDuctMediumCount_ { 0 };
    double lastExhaustAcousticStatePublishSeconds_ { -1.0 };
    std::array<double, 32> chamberPressureBar_ {};
    std::array<double, 32> intakeFlowMgPerCycle_ {};
    std::array<double, 32> exhaustFlowMgPerCycle_ {};
    std::array<double, 32> intakeFlowMgThisCycle_ {};
    std::array<double, 32> exhaustFlowMgThisCycle_ {};
    /** Oxygen-equivalent fresh air across the intake valve, signed and NET, so
     *  reversion subtracts what it carries back out. Latched per cycle beside
     *  the total-charge accumulators above. */
    std::array<double, 32> deliveredAirMgPerCycle_ {};
    std::array<double, 32> deliveredAirMgThisCycle_ {};
    std::array<bool, 32> cylinderFlowCycleStarted_ {};
    std::array<double, 32> cylinderWallTemperatureC_ {};
    std::array<GasCell, 32> intakePlenumGas_ {};
    std::size_t intakePlenumCount_ { 1 };
    /** One 1-D finite-volume runner per cylinder (see
     * configurePhysicalIntakeNetworks). The plenum stays a lumped cell — a
     * plenum is physically a compliance — while the runner, which is the organ
     * pipe intake tuning lives in, remains resolved in space. Production
     * advances it at a 400 us conservative multirate cadence against a
     * time-averaged valve boundary, split symmetrically around exhaust
     * coupling. Cylinders still interact only through the shared plenum. */
    std::array<std::unique_ptr<gasdynamics::ExhaustGasNetwork>, 32> intakeRunnerNetworks_;
    /** Workers for the per-cylinder half of the runner advance, which the
     * profiler puts at 75-84% of the mechanical sub-step. Null when the engine
     * is too small or the machine too narrow for a barrier to pay for itself,
     * in which case the phase runs inline. Read CylinderWorkerPool's header
     * before touching this: an earlier per-cylinder fork-join over a much
     * smaller body was measured and removed. */
    std::unique_ptr<CylinderWorkerPool> intakeWorkerPool_;
    /** Scratch plenum cells the Gauss-Seidel staircase is walked through
     * before the runner advances run concurrently. A member rather than a
     * local so a V12 does not copy twelve gas cells onto the stack twice per
     * mechanical sub-step. Nothing outside that pre-pass reads it, and it is
     * fully overwritten at the start of every pass. */
    std::array<GasCell, 32> plenumStaircaseScratch_ {};
    /** Time integrals used only by the diagnostic/production multirate intake
     * coupling. They mirror the exhaust accumulators: no mass or energy is
     * approximated algebraically; the full conservative runner advances less
     * often against a boundary averaged over every mechanical substep. */
    std::array<gasdynamics::ConservativeState, 32>
        intakeBoundaryStateTimeIntegral_ {};
    std::array<double, 32> intakeBoundaryVolumeTimeIntegralM3S_ {};
    std::array<double, 32> intakeValveConductanceTimeIntegralM2S_ {};
    double intakeCouplingDurationSeconds_ { 0.0 };
    /** How many serial groups the concurrent runner pass is split into. One is
     * full concurrency; a group per cylinder is the old serial scheme exactly,
     * which is what an engine with no worker pool gets. Chosen from the
     * measured accuracy of the plenum staircase prediction, not from the thread
     * count -- see `configureIntakeWorkerPool`. */
    double intakeWallHeatPendingSeconds_ { 0.0 };
    double intakeWallHeatUpdateIntervalSeconds_ { 150.0e-6 };
    std::size_t intakePredictionGroupCount_ { 32 };
    /** Fixed-point rounds used to reconstruct the plenum drawdown staircase.
     * Production keeps the two converged Heun rounds: zero leaves a stationary-
     * manifold pressure bias, while one is only borderline at the invariant's
     * 0.01 kPa tolerance. Multirate coupling amortises their dispatch cost. */
    std::size_t intakeStaircaseRounds_ { 2 };
    /** Port-end (valve-side) runner state published for telemetry, the
     * Helmholtz telemetry model and the acoustic intake excitation. Refreshed
     * from each network advance's exchange record. */
    std::array<double, 32> intakePortTemperatureK_ {};
    std::array<double, 32> intakePortDensityKgPerM3_ {};
    std::array<double, 32> intakePortSpeedOfSoundMps_ {};
    std::array<GasCell, 32> cylinderGas_ {};
    std::array<double, 32> instantaneousCombustionPulse_ {};
    std::array<double, 32> injectedFuelMolesThisCycle_ {};
    std::array<double, 32> entrainedFuelMolesThisCycle_ {};
    std::array<double, 32> meteredFuelMolesLastCycle_ {};
    std::array<double, 32> deliveredFuelMolesLastCycle_ {};
    std::array<double, 32> requestedFuelMolesThisCycle_ {};
    std::array<double, 32> trappedAirMassMgLastCycle_ {};
    std::array<double, 32> trappedAirSourcePressureKpaLastCycle_ {};
    std::array<double, 32> trappedAirSourceTemperatureKLastCycle_ {};
    std::array<double, 32> actualAfrLastCycle_ {};
    std::array<double, 32> fuelDeliveryRatio_ {};
    // Largest pulse the injector was asked to deliver this cycle, and the
    // fraction of it that actually metered before the window closed. The latter
    // isolates real injector-capacity saturation from closed-loop trim: it reads
    // 1.0 whenever the injector keeps up with its command, however large the
    // trim, and only falls when the injector physically runs out of time.
    std::array<double, 32> commandedFuelMolesMaxThisCycle_ {};
    /** Sub-steps on which the injector actually flowed, and sub-steps spent
     *  inside its angular window, both per cycle. Their ratio scaled by the
     *  window's share of the 720 deg cycle is the injector's DUTY CYCLE, and
     *  `injectorCapacityRatio_` is now the headroom left on it (1 - duty).
     *  `injectorOpenSubsteps_` is a historical name: it stores the SUM of
     *  fractional openings, not an integer count.
     *
     *  The ratio above it used to be `injected sum / largest single-sub-step
     *  command`, which compares two quantities that are not commensurable: the
     *  command is a "place the whole remaining deficit now" figure recomputed
     *  every sub-step against an inventory that includes runner vapour and wall
     *  film, so it inflates whenever transport losses reopen the deficit, while
     *  the numerator is a per-cycle total. Measured, it disagreed with delivery
     *  in BOTH directions: the Flat-6 at a quarter throttle read 1.000 while
     *  only 27.9 mg of a 34.2 mg request reached the charge, and a K20A about
     *  fifteen per cent short read 0.11. So the warning missed real shortfalls
     *  and fired on healthy engines, which is what was reported from the
     *  application.
     *
     *  Deficit over request was tried next and is also wrong, for a subtler
     *  reason worth keeping: the inventory it subtracts is discounted by the
     *  vaporisation-availability model, which collapses as the window closes,
     *  so the deficit reads large at the very instant it must be sampled even
     *  though the fuel is physically present. Measured, it reported 0.46-0.56
     *  on a Flat-6 and an LS3 that were holding their commanded AFR to within
     *  0.03 with no misfire at all.
     *
     *  Duty cycle has none of those problems because it is a statement about
     *  the ACTUATOR rather than about any fuel accounting: how much of the
     *  cycle the injector had to spend open. Its fault threshold comes from
     *  production practice -- a port injector is sized to stay under roughly
     *  85-90 % duty at rated power -- and not from anything this simulator
     *  reports. */
    std::array<double, 32> injectorOpenSubsteps_ {};
    std::array<double, 32> injectorWindowSubsteps_ {};
    std::array<double, 32> injectorCapacityRatio_ {};
    std::array<double, 32> closedLoopFuelTrim_ {};
    /** Separate high-throttle adaptation cell. A single scalar let a rich WOT
     *  transient erase the low-load correction needed on return to idle. */
    std::array<double, 32> highLoadClosedLoopFuelTrim_ {};
    /** The first transition must be continuous: seed the high-load cell from
     *  the already learned low-load value, then let both regions diverge. */
    std::array<bool, 32> highLoadClosedLoopFuelTrimSeeded_ {};
    std::array<FlameEvent, 32> flameEvents_ {};
    std::array<CompressionIgnitionState, 32> compressionIgnitionStates_ {};
    std::array<CompressionIgnitionResult, 32> compressionIgnitionResults_ {};
    std::array<FuelInjectionState, 32> injectionStates_ {};
    std::array<EndGasKnockState, 32> endGasKnockStates_ {};
    std::array<IndicatedWorkState, 32> indicatedWorkStates_ {};
    std::array<ValveTrainState, 32> valveTrainStates_ {};
    std::array<ValveTrainResult, 32> valveTrainResults_ {};
    std::array<HelmholtzRunnerState, 32> runnerAcousticStates_ {};
    std::array<HelmholtzRunnerResult, 32> runnerAcousticResults_ {};
    std::array<double, 32> ignitionDelayRemainingSeconds_ {};
    /** Burned mole fraction sampled at the instant of ignition. Nothing has
     *  burned yet at that point, so it is the residual left by gas exchange --
     *  the quantity the flame model has always consumed and nothing published. */
    std::array<double, 32> residualGasFractionAtSpark_ {};
    /** Equivalence ratio the flame actually saw at ignition. Not the same as
     *  the cycle AFR the ECU reports: with a late injection window the charge
     *  can still be arriving when the spark fires. */
    std::array<double, 32> equivalenceRatioAtSpark_ {};
    std::array<bool, 32> ignitionPending_ {};
    std::array<bool, 32> sparkScheduleArmed_ {};
    std::array<double, 32> scheduledSparkPhaseDegrees_ {};
    /** Crank degrees still to travel before the latched spark event fires.
     *  A remaining DISTANCE, not an absolute phase: a commanded retard past
     *  firing TDC places the event after the cycle boundary, which no
     *  single-arc phase target can express. See the latch in step(). */
    std::array<double, 32> sparkScheduleTravelRemainingDegrees_ {};
    std::array<std::uint32_t, 32> commandedSparkEventsThisCycle_ {};
    std::array<std::uint32_t, 32> commandedSparkEventsLastCycle_ {};
    std::array<std::uint32_t, 32> completedIgnitionEventsThisCycle_ {};
    std::array<std::uint32_t, 32> completedIgnitionEventsLastCycle_ {};
    std::array<double, 32> commandedSparkPhaseThisCycle_ {};
    std::array<double, 32> commandedSparkPhaseLastCycle_ {};
    std::array<double, 32> completedIgnitionPhaseThisCycle_ {};
    std::array<double, 32> completedIgnitionPhaseLastCycle_ {};
    FlamePhysicsModel flamePhysics_ {};
    std::array<bool, 32> cylinderMisfires_ {};
    std::unique_ptr<gasdynamics::ExhaustGasNetwork> physicalExhaustNetwork_;
    /** Heap-owned scratch keeps the bounded source list out of step()'s large
     * Windows stack frame. */
    gasdynamics::ExhaustFuelReactionResult exhaustFuelReactionScratch_ {};
    gasdynamics::ConservativeState physicalExhaustAmbientState_ {};
    /** Total configured conductance of the network's terminal openings, m^2.
     *  Cached from the compiled layout so the forced-induction block can charge
     *  the turbine its real downstream back pressure. */
    double exhaustOutletConductanceM2_ { 0.0 };
    /** Network-port order -> EngineConfig cylinder order, compiled once. */
    std::array<std::size_t, 32> exhaustNetworkCylinderIndex_ {};
    std::unique_ptr<SpscQueue<CylinderPressureSample, 1'024>> pressureSamples_;
    // Coupling-rate exhaust media/reaction telemetry is intentionally separate
    // from the much denser thermodynamic pressure stream.
    std::unique_ptr<SpscQueue<ExhaustAcousticSample, 1'024>> exhaustAcousticSamples_;
    std::uint32_t randomState_ { 0x6d2b79f5U };
    // Engines below the threading threshold retain randomState_ and therefore
    // their historical misfire sequence. Thread-eligible engines use independent
    // seeded streams: a shared xorshift would race and its advance order would
    // depend on worker scheduling. This intentionally changes the exact misfire
    // sequence for 8+ cylinders while keeping it deterministic across runs and
    // across machines with different hardware concurrency.
    std::array<std::uint32_t, 32> cylinderRandomState_ {};
    /** Independent stream keeps opt-in cyclic variability from changing the
     * deterministic misfire decisions. */
    std::array<std::uint32_t, 32> combustionVariationRandomState_ {};
    std::array<double, 32> combustionVariationNormalisedState_ {};
    std::array<double, 32> combustionCycleMultiplier_ {};
    double eventEvaluationAngleDegrees_ { 719.9 };
    double eventEvaluationTimeSeconds_ { 0.0 };
    double indicatedWorkThisCycleJoules_ { 0.0 };
    double brakeWorkThisCycleJoules_ { 0.0 };
    double integratedCrankRadiansThisCycle_ { 0.0 };
    double cycleElapsedSeconds_ { 0.0 };
    double cycleStartTimeSeconds_ { 0.0 };
    std::uint64_t nextCompletedBrakeCycleId_ { 1 };
    bool cycleTelemetryStarted_ { false };
};
} // namespace enginelab
