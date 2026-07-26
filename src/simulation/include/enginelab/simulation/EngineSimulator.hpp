#pragma once
#include <enginelab/ecu/IEcuModel.hpp>
#include <enginelab/events/IFiringEventGenerator.hpp>
#include <enginelab/exhaust/IExhaustModel.hpp>
#include <enginelab/physics/IPhysicsModel.hpp>
#include <enginelab/physics/ConservativeGasSystem.hpp>
#include <enginelab/physics/FlamePhysicsModel.hpp>
#include <enginelab/physics/FuelInjectionModel.hpp>
#include <enginelab/physics/EndGasKnockModel.hpp>
#include <enginelab/physics/IndicatedWorkModel.hpp>
#include <enginelab/physics/ValveTrainModel.hpp>
#include <enginelab/physics/HelmholtzRunnerModel.hpp>
#include <enginelab/physics/MechanicalKinematics.hpp>
#include <enginelab/physics/DuctWallHeatTransferModel.hpp>
#include <enginelab/simulation/IEngineSimulation.hpp>
#include <enginelab/events/CylinderPressureSample.hpp>
#include <enginelab/foundation/SpscQueue.hpp>
#include <enginelab/gasdynamics/ExhaustGasNetwork.hpp>
#include <array>
#include <memory>
namespace enginelab {
/** Orchestrates policies and integrates state; owns no thread and performs no audio work. */
class EngineSimulator final : public IEngineSimulation {
public:
    EngineSimulator(EngineConfig, IEcuModel&, IPhysicsModel&, IFiringEventGenerator&, IExhaustModel&);
    [[nodiscard]] SimulationFrame step(double dtSeconds, const EngineControls&) noexcept override;
    [[nodiscard]] const EngineState& state() const noexcept override { return state_; }
    [[nodiscard]] bool tryPopCylinderPressureSample(CylinderPressureSample& sample) noexcept {
        return pressureSamples_ && pressureSamples_->tryPop(sample);
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
    [[nodiscard]] RunningState determineRunningState(const EngineControls&) const noexcept;
    void accumulateCycleTelemetry(double previousAngleDegrees, double travelledDegrees,
                                  double dtSeconds, double indicatedTorqueNm,
                                  double brakeTorqueNm) noexcept;
    EngineConfig config_;
    EngineKinematicsReference kinematicsReference_;
    IEcuModel& ecu_;
    IPhysicsModel& physics_;
    IFiringEventGenerator& eventGenerator_;
    IExhaustModel& exhaust_;
    EngineState state_;
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
    /** Length-mean acoustic medium of every compiled exhaust duct, refreshed on
     *  each coupling flush and republished on every substep in between. */
    std::array<float, CylinderPressureSample::maximumExhaustDucts>
        exhaustDuctDensityKgPerM3_ {};
    std::array<float, CylinderPressureSample::maximumExhaustDucts>
        exhaustDuctSpeedOfSoundMps_ {};
    std::size_t exhaustDuctMediumCount_ { 0 };
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
     * pipe intake tuning lives in, is resolved in space. Advanced twice per
     * mechanical substep (a symmetric split around the exhaust coupling, like
     * the lumped valve orifice it replaces), each network couples one cylinder
     * boundary to its own path's plenum, so cylinders only interact through
     * the shared plenum cell exactly as before. */
    std::array<std::unique_ptr<gasdynamics::ExhaustGasNetwork>, 32> intakeRunnerNetworks_;
    /** Port-end (valve-side) runner state published for telemetry, the
     * Helmholtz telemetry model and the acoustic intake excitation. Refreshed
     * from each network advance's exchange record. */
    std::array<double, 32> intakePortTemperatureK_ {};
    std::array<double, 32> intakePortDensityKgPerM3_ {};
    std::array<double, 32> intakePortSpeedOfSoundMps_ {};
    std::array<GasCell, 32> cylinderGas_ {};
    std::array<double, 32> instantaneousCombustionPulse_ {};
    std::array<double, 32> injectedFuelMolesThisCycle_ {};
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
    std::array<double, 32> injectorCapacityRatio_ {};
    std::array<double, 32> closedLoopFuelTrim_ {};
    /** Separate high-throttle adaptation cell. A single scalar let a rich WOT
     *  transient erase the low-load correction needed on return to idle. */
    std::array<double, 32> highLoadClosedLoopFuelTrim_ {};
    /** The first transition must be continuous: seed the high-load cell from
     *  the already learned low-load value, then let both regions diverge. */
    std::array<bool, 32> highLoadClosedLoopFuelTrimSeeded_ {};
    std::array<FlameEvent, 32> flameEvents_ {};
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
    FlamePhysicsModel flamePhysics_ {};
    std::array<bool, 32> cylinderMisfires_ {};
    std::unique_ptr<gasdynamics::ExhaustGasNetwork> physicalExhaustNetwork_;
    gasdynamics::ConservativeState physicalExhaustAmbientState_ {};
    /** Network-port order -> EngineConfig cylinder order, compiled once. */
    std::array<std::size_t, 32> exhaustNetworkCylinderIndex_ {};
    std::unique_ptr<SpscQueue<CylinderPressureSample, 1'024>> pressureSamples_;
    std::uint32_t randomState_ { 0x6d2b79f5U };
    // Engines below the threading threshold retain randomState_ and therefore
    // their historical misfire sequence. Thread-eligible engines use independent
    // seeded streams: a shared xorshift would race and its advance order would
    // depend on worker scheduling. This intentionally changes the exact misfire
    // sequence for 8+ cylinders while keeping it deterministic across runs and
    // across machines with different hardware concurrency.
    std::array<std::uint32_t, 32> cylinderRandomState_ {};
    double eventEvaluationAngleDegrees_ { 719.9 };
    double eventEvaluationTimeSeconds_ { 0.0 };
    double indicatedWorkThisCycleJoules_ { 0.0 };
    double brakeWorkThisCycleJoules_ { 0.0 };
    double cycleElapsedSeconds_ { 0.0 };
    bool cycleTelemetryStarted_ { false };
};
} // namespace enginelab
