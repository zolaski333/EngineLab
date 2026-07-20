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
    void reset() noexcept override;
private:
    /** Compile and allocate the mandatory nonlinear exhaust network. */
    void configurePhysicalExhaustNetwork();
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
     * stride instead, at the cost of one coupling interval (at most 500 us) of
     * group delay on the exhaust boundary alone.
     */
    struct ExhaustNetworkBoundary final {
        double pressureKpa {};
        double velocityMps {};
        double temperatureK {};
        double massFlowKgPerSecond {};
        double densityKgPerM3 {};
        double speedOfSoundMps {};
        bool valid { false };
    };
    std::array<ExhaustNetworkBoundary, 32> exhaustBoundaryAtPreviousFlush_ {};
    std::array<ExhaustNetworkBoundary, 32> exhaustBoundaryAtLastFlush_ {};
    bool exhaustBoundaryHistoryPrimed_ { false };
    /** Substeps elapsed since the last network flush, for the reconstruction. */
    std::size_t exhaustBoundarySubstepsSinceFlush_ { 0 };
    std::array<double, 32> chamberPressureBar_ {};
    std::array<double, 32> intakeFlowMgPerCycle_ {};
    std::array<double, 32> exhaustFlowMgPerCycle_ {};
    std::array<double, 32> intakeFlowMgThisCycle_ {};
    std::array<double, 32> exhaustFlowMgThisCycle_ {};
    std::array<bool, 32> cylinderFlowCycleStarted_ {};
    std::array<double, 32> cylinderWallTemperatureC_ {};
    std::array<GasCell, 32> intakePlenumGas_ {};
    std::size_t intakePlenumCount_ { 1 };
    std::array<GasCell, 32> intakeRunnerGas_ {};
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
    std::array<double, 32> closedLoopFuelTrim_ {};
    std::array<FlameEvent, 32> flameEvents_ {};
    std::array<FuelInjectionState, 32> injectionStates_ {};
    std::array<EndGasKnockState, 32> endGasKnockStates_ {};
    std::array<IndicatedWorkState, 32> indicatedWorkStates_ {};
    std::array<ValveTrainState, 32> valveTrainStates_ {};
    std::array<ValveTrainResult, 32> valveTrainResults_ {};
    std::array<HelmholtzRunnerState, 32> runnerAcousticStates_ {};
    std::array<HelmholtzRunnerResult, 32> runnerAcousticResults_ {};
    std::array<double, 32> ignitionDelayRemainingSeconds_ {};
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
