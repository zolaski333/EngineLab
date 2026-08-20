#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

namespace enginelab {

struct DynoAbsorberOutput final {
    // Applied one-way absorber torque after the physical [0, capacity] clamp.
    double brakeTorqueNm {};
    double filteredRpm {};
    double filteredAccelerationRpmPerSecond {};
    // Legacy contact flag: true for any strictly positive contact fraction.
    bool contacted { false };
    // Smooth engagement of the absorber, from disengaged (0) to full (1).
    double contactFraction {};
    // Requested torque after anti-windup, immediately before the physical
    // one-way/capacity clamp which produces brakeTorqueNm.
    double unclampedBrakeTorqueNm {};
    // True when the physical clamp materially rejected negative torque or
    // torque above the absorber capacity. Round-off at a clamp boundary is
    // deliberately ignored.
    bool saturatedLow { false };
    bool saturatedHigh { false };
};

/**
 * Unidirectional absorption-dyno controller expressed in crank torque.
 *
 * The controller filters individual firing pulses, estimates the engine's
 * brake torque as feed-forward, and closes the remaining speed error with
 * proportional, integral and acceleration feedback. Its gains scale with
 * displacement so one calibration covers the catalogue.
 */
class DynoAbsorberController final {
public:
    explicit DynoAbsorberController(const EngineConfig&) noexcept;

    void reset(double initialRpm, double initialBrakeTorqueNm = 0.0) noexcept;

    [[nodiscard]] DynoAbsorberOutput advance(
        double dtSeconds, double targetRpm,
        const EngineState& engineState,
        double contactBandRpm = 60.0) noexcept;

private:
    double controllerTorqueScaleNm_ { 1.0 };
    double maximumBrakeTorqueNm_ { 1.0 };
    double integralTorqueNm_ {};
    double feedForwardTorqueNm_ {};
    double filteredRpm_ {};
    double filteredAccelerationRpmPerSecond_ {};
    double brakeTorqueNm_ {};
    bool initialised_ { false };
};

} // namespace enginelab
