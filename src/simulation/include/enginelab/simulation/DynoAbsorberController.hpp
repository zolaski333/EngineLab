#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

namespace enginelab {

struct DynoAbsorberOutput final {
    double brakeTorqueNm {};
    double filteredRpm {};
    double filteredAccelerationRpmPerSecond {};
    bool contacted { false };
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
