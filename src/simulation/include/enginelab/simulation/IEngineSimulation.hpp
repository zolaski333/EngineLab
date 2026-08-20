#pragma once
#include <enginelab/events/FiringEvent.hpp>
#include <enginelab/foundation/EngineTypes.hpp>
#include <array>
#include <cstddef>
#include <cstdint>
namespace enginelab {
// The validated worst case (32 cylinders, 20,000 rpm and a 50 ms recovery
// step) produces at most 267 events. 320 keeps margin without making this
// value-returned frame large enough to exhaust an ASan-instrumented stack.
inline constexpr std::size_t maxEventsPerSimulationStep = 320;
/** A complete crank cycle measured from boundary to boundary.
 *
 * Unlike EngineState's latched display values, these records are event-like:
 * each completed cycle appears in exactly one SimulationFrame.  The fixed
 * array keeps the simulation thread allocation-free while the explicit drop
 * counter makes an undersized consumer contract observable.
 */
struct CompletedBrakeCycleSample final {
    std::uint64_t cycleId { 0 };
    double startTimeSeconds { 0.0 };
    double endTimeSeconds { 0.0 };
    double durationSeconds { 0.0 };
    double integratedCrankRadians { 0.0 };
    double indicatedWorkJoules { 0.0 };
    double brakeWorkJoules { 0.0 };
    double meanRpm { 0.0 };
    double meanTorqueNm { 0.0 };
    double meanPowerKw { 0.0 };
    bool numericallyValid { false };
};
inline constexpr std::size_t maxCompletedBrakeCycleSamplesPerSimulationStep = 16;
struct SimulationFrame final {
    EngineState state;
    std::array<FiringEvent, maxEventsPerSimulationStep> firingEvents {};
    std::size_t firingEventCount { 0 };
    std::size_t droppedFiringEventCount { 0 };
    std::array<CompletedBrakeCycleSample,
               maxCompletedBrakeCycleSamplesPerSimulationStep>
        completedBrakeCycleSamples {};
    std::size_t completedBrakeCycleSampleCount { 0 };
    std::size_t droppedCompletedBrakeCycleSampleCount { 0 };
    std::size_t cylinderPressureSampleCount { 0 };
    std::size_t droppedCylinderPressureSampleCount { 0 };
    std::size_t exhaustAcousticSampleCount { 0 };
    std::size_t droppedExhaustAcousticSampleCount { 0 };
};
class IEngineSimulation {
public:
    virtual ~IEngineSimulation() = default;
    [[nodiscard]] virtual SimulationFrame step(double dtSeconds, const EngineControls&) noexcept = 0;
    [[nodiscard]] virtual const EngineState& state() const noexcept = 0;
    virtual void reset() noexcept = 0;
};
} // namespace enginelab
