#pragma once
#include <enginelab/foundation/EngineTypes.hpp>
namespace enginelab {
class IEcuModel {
public:
    virtual ~IEcuModel() = default;
    [[nodiscard]] virtual EcuCommand evaluate(const EngineConfig&, const EngineState&, const EngineControls&) const noexcept = 0;
    virtual void reset() noexcept {}
};
} // namespace enginelab
