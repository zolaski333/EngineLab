#pragma once

#include <enginelab/foundation/EngineTypes.hpp>
#include <cstddef>

namespace enginelab {

/** A single source of truth for simulation and rendering mechanical geometry. */
struct CylinderKinematics final {
    double pistonTravelMm { 0.0 };
    double pistonPositionMm { 0.0 };
    double pistonVelocityMps { 0.0 };
    double pistonAccelerationMps2 { 0.0 };
    double connectingRodAngleDegrees { 0.0 };
    double connectingRodObliquityDegrees { 0.0 };
    double crankPinXMm { 0.0 };
    double crankPinYMm { 0.0 };
    double wristPinXMm { 0.0 };
    double wristPinYMm { 0.0 };
    double chamberVolumeLitres { 0.0 };
    double displacementDerivativeMPerRadian { 0.0 };
    std::uint32_t crankshaftId { 1 };
    std::uint32_t crankJournalId { 0 };
};

[[nodiscard]] CylinderKinematics evaluateCylinderKinematics(
    const EngineConfig&, std::size_t cylinderIndex, double primaryCrankAngleDegrees,
    double primaryAngularVelocityRadPerSecond, double primaryAngularAccelerationRadPerSecond2 = 0.0) noexcept;

} // namespace enginelab
