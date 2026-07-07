#pragma once
#include <enginelab/events/FiringEvent.hpp>
#include <enginelab/foundation/EngineTypes.hpp>
namespace enginelab {
class IExhaustModel {
public:
    virtual ~IExhaustModel() = default;
    [[nodiscard]] virtual double backPressureKpa(const EngineState&) const noexcept = 0;
    virtual void process(FiringEvent&) const noexcept = 0;
};
} // namespace enginelab
