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
    /** Woschni's combustion term, C2 Vd Tr / (pr Vr) (p - p_motored), in
     *  m/s, added to the gas velocity. The caller supplies it (zero: the
     *  motored law alone). */
    double combustionGasVelocityMps { 0.0 };
    /** The chamber's wetted area when positive (a rotary's face: flank,
     *  housing and side plates), in place of a piston's crown, head and
     *  exposed liner. */
    double exposedAreaOverrideM2 { 0.0 };
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
 * The coefficient follows the motored-flow term of Woschni's correlation;
 * open-valve gas-exchange strokes use its higher mean-gas-speed coefficient,
 * and a caller may add the combustion term's gas velocity.
 * The finite-time exchange is integrated analytically against the gas heat
 * capacity, so a solver step cannot cross the prescribed wall temperature.
 */
class CylinderHeatTransferModel final {
public:
    [[nodiscard]] static CylinderHeatTransferResult evaluate(
        const CylinderHeatTransferConditions&) noexcept;
};

} // namespace enginelab
