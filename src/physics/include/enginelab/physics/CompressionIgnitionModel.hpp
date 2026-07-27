#pragma once

#include <enginelab/foundation/EngineTypes.hpp>
#include <enginelab/physics/ConservativeGasSystem.hpp>

namespace enginelab {

struct CompressionIgnitionState final {
    double livengoodWuIntegral { 0.0 };
    double ignitionDelaySeconds { 0.0 };
    double premixedFuelMolesRemaining { 0.0 };
    double cumulativeBurnedFuelMoles { 0.0 };
    double cycleFuelReferenceMoles { 0.0 };
    double elapsedBurnSeconds { 0.0 };
    double startPhaseDegrees { 0.0 };
    double sharpness { 0.0 };
    bool autoIgnited { false };
    bool active { false };
};

struct CompressionIgnitionConditions final {
    double cyclePhaseDegrees { 0.0 };
    double meanPistonSpeedMps { 0.0 };
    bool enabled { false };
};

struct CompressionIgnitionResult final {
    CombustionReaction reaction {};
    double ignitionDelaySeconds { 0.0 };
    double burnedFraction { 0.0 };
    double efficiency { 0.0 };
    double burnRateFuelMolesPerSecond { 0.0 };
    double startPhaseDegrees { 0.0 };
    double durationSeconds { 0.0 };
    double sharpness { 0.0 };
    bool autoIgnited { false };
    bool active { false };
};

/**
 * Reduced-order compression-ignition heat-release model.
 *
 * Autoignition follows a Livengood-Wu induction integral using the Assanis
 * pressure/temperature/equivalence-ratio delay correlation and an explicit
 * cetane correction. Heat release then has two inventories: a rapid premixed
 * fraction accumulated during the delay, followed by a turbulence-adjusted,
 * mixing-controlled diffusion burn. The model consumes fuel and oxygen only
 * through ConservativeGasSystem, so chemical mass and released energy retain
 * the same invariants as spark combustion.
 *
 * This is a cylinder-mean (0-D) spray/combustion closure. Droplet dispersion is
 * resolved separately by FuelInjectionModel; neither class claims to resolve
 * individual jets or local soot chemistry.
 */
class CompressionIgnitionModel final {
public:
    static void beginCycle(CompressionIgnitionState& state) noexcept;

    [[nodiscard]] static double ignitionDelaySeconds(
        const CombustionCalibrationConfig& calibration,
        double cetaneNumber, double pressureBar, double temperatureK,
        double equivalenceRatio) noexcept;

    [[nodiscard]] static CompressionIgnitionResult advance(
        CompressionIgnitionState& state, GasCell& chamber,
        const FuelConfig& fuel,
        const CombustionCalibrationConfig& calibration,
        const CompressionIgnitionConditions& conditions,
        double dtSeconds) noexcept;
};

} // namespace enginelab
