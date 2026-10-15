#pragma once

#include <enginelab/events/IFiringEventGenerator.hpp>

#include <cstdint>

namespace enginelab {

/** Generates the firing events of a piston engine: one per cylinder per
 *  cylinder cycle (720 degrees, or 360 for a two-stroke). */
class FourStrokeEventGenerator final : public IFiringEventGenerator {
public:
    /** The crank angle's domain: 720 degrees for every piston engine. */
    [[nodiscard]] double cycleDegrees() const noexcept override { return 720.0; }
    void reset() noexcept override { randomState_ = 0x1a2b3c4dU; lastDroppedEventCount_ = 0; }
    [[nodiscard]] std::size_t droppedEventCountLastGenerate() const noexcept override { return lastDroppedEventCount_; }
    std::size_t generate(const EngineConfig&, const EngineState&, const EcuCommand&,
                         const CombustionResult&, double, double, double, double,
                         std::span<FiringEvent>) noexcept override;
private:
    [[nodiscard]] float randomUnit() noexcept;
    std::uint32_t randomState_ { 0x1a2b3c4dU };
    std::size_t lastDroppedEventCount_ { 0 };
};

} // namespace enginelab
