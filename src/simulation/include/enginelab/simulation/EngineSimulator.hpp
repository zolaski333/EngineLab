#pragma once
#include <enginelab/ecu/IEcuModel.hpp>
#include <enginelab/events/IFiringEventGenerator.hpp>
#include <enginelab/exhaust/IExhaustModel.hpp>
#include <enginelab/physics/IPhysicsModel.hpp>
#include <enginelab/physics/ConservativeGasSystem.hpp>
#include <enginelab/physics/FlamePhysicsModel.hpp>
#include <enginelab/physics/FuelInjectionModel.hpp>
#include <enginelab/physics/EndGasKnockModel.hpp>
#include <enginelab/simulation/IEngineSimulation.hpp>
#include <enginelab/events/CylinderPressureSample.hpp>
#include <enginelab/foundation/SpscQueue.hpp>
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
    [[nodiscard]] RunningState determineRunningState(const EngineControls&) const noexcept;
    EngineConfig config_;
    IEcuModel& ecu_;
    IPhysicsModel& physics_;
    IFiringEventGenerator& eventGenerator_;
    IExhaustModel& exhaust_;
    EngineState state_;
    std::array<double, 32> previousCylinderPhases_ {};
    std::array<double, 32> intakeRunnerPressureKpa_ {};
    std::array<double, 32> exhaustRunnerPressureKpa_ {};
    std::array<double, 32> chamberPressureBar_ {};
    std::array<double, 32> intakeFlowMgPerCycle_ {};
    std::array<double, 32> exhaustFlowMgPerCycle_ {};
    std::array<double, 32> cylinderWallTemperatureC_ {};
    GasCell manifoldGas_ {};
    std::array<GasCell, 32> exhaustCollectorGas_ {};
    std::size_t exhaustCollectorCount_ { 1 };
    std::array<GasCell, 32> intakeRunnerGas_ {};
    std::array<GasCell, 32> cylinderGas_ {};
    std::array<GasCell, 32> exhaustRunnerGas_ {};
    std::array<double, 32> instantaneousCombustionPulse_ {};
    std::array<double, 32> injectedFuelMolesThisCycle_ {};
    std::array<double, 32> deliveredFuelMolesLastCycle_ {};
    std::array<double, 32> requestedFuelMolesThisCycle_ {};
    std::array<double, 32> trappedAirMassMgLastCycle_ {};
    std::array<double, 32> fuelDeliveryRatio_ {};
    std::array<FlameEvent, 32> flameEvents_ {};
    std::array<FuelInjectionState, 32> injectionStates_ {};
    std::array<EndGasKnockState, 32> endGasKnockStates_ {};
    FlamePhysicsModel flamePhysics_ {};
    std::array<bool, 32> cylinderMisfires_ {};
    std::unique_ptr<SpscQueue<CylinderPressureSample, 1'024>> pressureSamples_;
    std::uint32_t randomState_ { 0x6d2b79f5U };
};
} // namespace enginelab
