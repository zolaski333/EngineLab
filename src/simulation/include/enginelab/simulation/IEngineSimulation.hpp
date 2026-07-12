#pragma once
#include <enginelab/events/FiringEvent.hpp>
#include <enginelab/foundation/EngineTypes.hpp>
#include <array>
#include <cstddef>
namespace enginelab {
// The validated worst case (32 cylinders, 20,000 rpm and a 50 ms recovery
// step) produces at most 267 events. 320 keeps margin without making this
// value-returned frame large enough to exhaust an ASan-instrumented stack.
inline constexpr std::size_t maxEventsPerSimulationStep = 320;
struct SimulationFrame final {
    EngineState state;
    std::array<FiringEvent, maxEventsPerSimulationStep> firingEvents {};
    std::size_t firingEventCount { 0 };
    std::size_t droppedFiringEventCount { 0 };
    std::size_t cylinderPressureSampleCount { 0 };
    std::size_t droppedCylinderPressureSampleCount { 0 };
};
class IEngineSimulation {
public:
    virtual ~IEngineSimulation() = default;
    [[nodiscard]] virtual SimulationFrame step(double dtSeconds, const EngineControls&) noexcept = 0;
    [[nodiscard]] virtual const EngineState& state() const noexcept = 0;
    virtual void reset() noexcept = 0;
};
} // namespace enginelab
