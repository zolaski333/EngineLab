#pragma once

#include <enginelab/events/FiringEvent.hpp>
#include <enginelab/foundation/EngineTypes.hpp>

#include <span>

namespace enginelab {

/** Strategy interface allowing 4T, 2T and future rotary event timing models. */
class IFiringEventGenerator {
public:
    virtual ~IFiringEventGenerator() = default;
    [[nodiscard]] virtual double cycleDegrees() const noexcept = 0;
    virtual void reset() noexcept {}
    [[nodiscard]] virtual std::size_t droppedEventCountLastGenerate() const noexcept { return 0; }
    virtual std::size_t generate(const EngineConfig& config,
                                 const EngineState& state,
                                 const EcuCommand& ecu,
                                 const CombustionResult& combustion,
                                 double stepStartTimeSeconds,
                                 double previousAngleDegrees,
                                 double travelledDegrees,
                                 double dtSeconds,
                                 std::span<FiringEvent> output) noexcept = 0;
};

} // namespace enginelab
