// A fault in a partial cycle must survive clean outer frames until that
// cycle completes. A deliberately capped motored solver supplies real faults.
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/foundation/EngineTypes.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <numbers>

int main() {
    auto config = enginelab::makeDefaultInlineTwo();
    // Legal authored envelope; the external motor deliberately drives above
    // it with ignition off, exercising the existing resolution-fault path.
    config.redlineRpm = 1'200.0;
    config.ignition.revLimitRpm = 1'200.0;
    config.solver.mechanicalFrequencyHz = 2'000.0;
    config.solver.gasSubsteps = 1;
    config.solver.maximumMechanicalFrequencyHz = 4'000.0;
    config.solver.maximumCrankDegreesPerStep = 2.0;
    enginelab::normaliseEngineConfig(config);
    enginelab::SimpleEcuModel ecu;
    enginelab::SimplifiedGasolinePhysics physics;
    enginelab::FourStrokeEventGenerator events;
    auto exhaust = enginelab::ExhaustGraph::makeForEngine(config);
    auto simulator = std::make_unique<enginelab::EngineSimulator>(
        config, ecu, physics, events, exhaust);
    constexpr double dt = 1.0 / 240.0;
    constexpr double extraInertia = 0.35;
    double previousMotorTorque = 0.0;
    bool partialCycleHasFault = false;
    bool hasSeenCompletedCycle = false;
    std::size_t faultyFrames = 0;
    std::size_t contaminatedCycles = 0;
    std::size_t cleanFrameContaminatedCycles = 0;
    std::size_t incorrectlyValidCycles = 0;
    std::size_t cleanValidCycles = 0;
    // Repeated acceleration/deceleration moves the fault relative to 720-degree
    // boundaries. The low target is below the cap; the high one exceeds it.
    for (std::size_t step = 0; step < 1'800; ++step) {
        const auto targetRpm = step < 180 ? 900.0
            : ((step - 180) % 144 < 30 ? 1'800.0 : 900.0);
        const auto& before = simulator->state();
        const auto disturbance = before.netTorqueNm - previousMotorTorque;
        const auto motorTorque = std::clamp(
            (enginelab::effectiveRotatingInertiaKgM2(config) + extraInertia)
                * (targetRpm * 2.0 * std::numbers::pi / 60.0
                    - before.angularVelocityRadPerSecond) / dt - disturbance,
            -5'000.0, 5'000.0);
        enginelab::EngineControls controls;
        controls.ignitionEnabled = false;
        controls.throttle = 1.0;
        controls.externalTorqueNm = motorTorque;
        controls.externalRotatingInertiaKgM2 = extraInertia;
        const auto frame = simulator->step(dt, controls);
        previousMotorTorque = motorTorque;
        if (frame.state.solverResolutionLimited) {
            ++faultyFrames;
        }
        // Frames containing a fault and a boundary have uncertain fault order
        // to this public observer. Only a later CLEAN frame proves carry.
        for (std::size_t index = 0;
             index < frame.completedBrakeCycleSampleCount; ++index) {
            const auto& cycle = frame.completedBrakeCycleSamples[index];
            if (partialCycleHasFault) {
                ++contaminatedCycles;
                if (!frame.state.solverResolutionLimited) {
                    ++cleanFrameContaminatedCycles;
                    if (cycle.numericallyValid) ++incorrectlyValidCycles;
                }
            } else if (cycle.numericallyValid
                       && !frame.state.solverResolutionLimited) {
                ++cleanValidCycles;
            }
            partialCycleHasFault = false;
            hasSeenCompletedCycle = true;
        }
        // With no boundary anywhere in this fault frame, it is provably part
        // of the one pending cycle. Boundary frames cannot establish order.
        if (hasSeenCompletedCycle && frame.state.solverResolutionLimited
                && frame.completedBrakeCycleSampleCount == 0
                && frame.droppedCompletedBrakeCycleSampleCount == 0)
            partialCycleHasFault = true;
    }
    std::cout << "faulty_frames=" << faultyFrames
        << " contaminated_cycles=" << contaminatedCycles
        << " clean_frame_contaminated_cycles=" << cleanFrameContaminatedCycles
        << " incorrectly_valid_cycles=" << incorrectlyValidCycles
        << " clean_valid_cycles=" << cleanValidCycles << '\n';
    const auto exercised = faultyFrames > 20 && contaminatedCycles > 5
        && cleanFrameContaminatedCycles > 5 && cleanValidCycles > 5;
    if (!exercised || incorrectlyValidCycles != 0) {
        std::cerr << "FAILED: numerical faults must invalidate the containing "
                     "cycle even when it completes in a clean outer frame\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
