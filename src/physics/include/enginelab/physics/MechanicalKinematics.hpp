#pragma once

#include <enginelab/foundation/EngineTypes.hpp>
#include <array>
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

/**
 * Immutable geometric reference shared by simulation and renderers.
 *
 * Resolving the true dead centres of master/articulated rods requires a
 * numerical extremum search. Build this object once for an engine definition
 * and reuse it for every realtime evaluation. It also provides the stable
 * geometry boundary needed by a future interpolated 3-D renderer.
 */
struct EngineKinematicsReference final {
    std::array<double, 32> topAxisPositionMm {};
    std::array<double, 32> bottomAxisPositionMm {};
    std::array<double, 32> topDeadCentreAngleDegrees {};
    std::array<double, 32> bottomDeadCentreAngleDegrees {};
    std::size_t cylinderCount { 0 };
};

[[nodiscard]] EngineKinematicsReference buildEngineKinematicsReference(
    const EngineConfig&) noexcept;

[[nodiscard]] CylinderKinematics evaluateCylinderKinematics(
    const EngineConfig&, std::size_t cylinderIndex, double primaryCrankAngleDegrees,
    double primaryAngularVelocityRadPerSecond, double primaryAngularAccelerationRadPerSecond2 = 0.0) noexcept;

[[nodiscard]] CylinderKinematics evaluateCylinderKinematics(
    const EngineConfig&, const EngineKinematicsReference&, std::size_t cylinderIndex,
    double primaryCrankAngleDegrees, double primaryAngularVelocityRadPerSecond,
    double primaryAngularAccelerationRadPerSecond2 = 0.0) noexcept;

} // namespace enginelab
