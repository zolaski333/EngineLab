#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

namespace enginelab {

/** Thermodynamic and geometric inputs for one flame-propagation step. */
struct FlameConditions final {
    double boreM { 0.086 };
    double chamberVolumeM3 { 0.00005 };
    double temperatureK { 300.0 };
    double pressurePa { 101'325.0 };
    double equivalenceRatio { 1.0 };
    double burnedGasFraction { 0.0 };
    double meanPistonSpeedMps { 0.0 };
    double load { 0.0 };
    double residualDilutionSensitivity { 0.78 };
    double chamberTurbulenceIntensityRatio { 1.0 };
    std::uint32_t ignitionSiteCount { 1 };
};

/** Persistent state of the flame kernel for one cylinder and one cycle. */
struct FlameEvent final {
    double radialTravelM { 0.0 };
    double axialTravelM { 0.0 };
    double lastChamberVolumeM3 { 0.0 };
    double burnedFraction { 0.0 };
    double flameSpeedMps { 0.0 };
    double efficiency { 0.0 };
    double initialBurnableFuelMoles { 0.0 };
    double elapsedSeconds { 0.0 };
    bool active { false };
};

struct FlameStepResult final {
    double burnedFraction { 0.0 };
    double burnedFractionAdvance { 0.0 };
    double flameSpeedMps { 0.0 };
    double efficiency { 0.0 };
    bool complete { false };
};

/**
 * Metghalchi-Keck laminar flame speed augmented by piston-driven turbulence,
 * dilution and an effective cylindrical flame-front geometry (pi * r^2 * h).
 */
class FlamePhysicsModel final {
public:
    [[nodiscard]] static double laminarFlameSpeedMps(const FuelConfig& fuel,
                                                      double equivalenceRatio,
                                                      double temperatureK,
                                                      double pressurePa) noexcept;
    [[nodiscard]] static double turbulentFlameSpeedMps(const FuelConfig& fuel,
                                                        const FlameConditions& conditions) noexcept;
    [[nodiscard]] static double combustionEfficiency(const FlameConditions& conditions) noexcept;
    [[nodiscard]] static double ignitionDelaySeconds(const CombustionCalibrationConfig&,
                                                      const FlameConditions&) noexcept;

    void ignite(FlameEvent& event, const FuelConfig& fuel,
                const FlameConditions& conditions,
                double burnableFuelMoles) const noexcept;
    [[nodiscard]] FlameStepResult advance(FlameEvent& event, const FuelConfig& fuel,
                                           const FlameConditions& conditions,
                                           double dtSeconds) const noexcept;
};

} // namespace enginelab
