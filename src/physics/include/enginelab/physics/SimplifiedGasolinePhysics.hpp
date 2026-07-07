#pragma once
#include <enginelab/physics/IPhysicsModel.hpp>
namespace enginelab {
/** Perceptually calibrated gasoline model; explicitly not an engineering solver. */
class SimplifiedGasolinePhysics final : public IPhysicsModel {
public:
    [[nodiscard]] CombustionResult evaluateCombustion(
        const EngineConfig&, const EngineState&, const EngineControls&,
        const EcuCommand&, double exhaustBackPressureKpa) const noexcept override;
};
} // namespace enginelab
