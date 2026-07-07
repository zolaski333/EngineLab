#pragma once
#include <enginelab/foundation/EngineTypes.hpp>
namespace enginelab {
/** Pure mechanical/combustion policy. It has no clock, thread, UI or audio dependency. */
class IPhysicsModel {
public:
    virtual ~IPhysicsModel() = default;
    [[nodiscard]] virtual CombustionResult evaluateCombustion(
        const EngineConfig&, const EngineState&, const EngineControls&,
        const EcuCommand&, double exhaustBackPressureKpa) const noexcept = 0;
};
} // namespace enginelab
