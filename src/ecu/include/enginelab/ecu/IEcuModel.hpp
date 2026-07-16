#pragma once
#include <enginelab/foundation/EngineTypes.hpp>
namespace enginelab {
class IEcuModel {
public:
    virtual ~IEcuModel() = default;
    /** Called once with the simulator's canonical, validated configuration. */
    virtual void initialise(const EngineConfig&) {}
    /** Acquire immutable live data once per outer simulation frame. */
    virtual void beginFrame() noexcept {}
    [[nodiscard]] virtual EcuCommand evaluate(const EngineConfig&, const EngineState&, const EngineControls&) const noexcept = 0;
    virtual void reset() noexcept {}
};
} // namespace enginelab
