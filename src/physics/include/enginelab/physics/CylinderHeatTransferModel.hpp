#pragma once

namespace enginelab {

/** Inputs to the Woschni cylinder heat-transfer law. */
struct CylinderHeatTransferConditions final {
    double boreM { 0.0 };
    double pistonTravelM { 0.0 };
    double meanPistonSpeedMps { 0.0 };
    double pressureKpa { 0.0 };
    double gasTemperatureK { 0.0 };
    double wallTemperatureK { 0.0 };
    double gasHeatCapacityJPerK { 0.0 };
    double fixedConductanceWPerK { 0.0 };
    double durationSeconds { 0.0 };
    bool gasExchangeStroke { false };
    /** Woschni's combustion term: the chamber and swept volumes, and the
     *  state at the seal the motored pressure is followed from. A zero
     *  reference pressure leaves the term out. The volume also sets the
     *  exposed liner, 4V/B; zero falls back to the piston travel. */
    double volumeM3 { 0.0 };
    double sweptVolumeM3 { 0.0 };
    double referencePressureKpa { 0.0 };
    double referenceTemperatureK { 0.0 };
    double referenceVolumeM3 { 0.0 };
};

struct CylinderHeatTransferResult final {
    double coefficientWPerM2K { 0.0 };
    double exposedAreaM2 { 0.0 };
    double conductanceWPerK { 0.0 };
    /** Positive adds sensible energy to the gas; negative rejects it to the wall. */
    double heatToGasJ { 0.0 };
};

/**
 * Instantaneous cylinder-to-wall convection for a zero-dimensional gas cell.
 *
 * The coefficient follows Woschni's correlation: the piston-driven gas speed
 * (its higher coefficient while the cylinder exchanges gas) plus, sealed, the
 * speed combustion adds in proportion to the pressure above the motored one.
 * The finite-time exchange is integrated analytically against the gas heat
 * capacity, so a solver step cannot cross the prescribed wall temperature.
 */
class CylinderHeatTransferModel final {
public:
    [[nodiscard]] static CylinderHeatTransferResult evaluate(
        const CylinderHeatTransferConditions&) noexcept;
};

} // namespace enginelab
