#pragma once

namespace enginelab {

struct DuctWallThermalState final {
    double temperatureK { 0.0 };
};

struct DuctWallHeatTransferConditions final {
    double innerDiameterM { 0.0 };
    double lengthM { 0.0 };
    double wallThicknessM { 0.0 };
    double wallDensityKgPerM3 { 0.0 };
    double wallSpecificHeatJPerKgK { 0.0 };
    double externalHeatTransferWPerM2K { 0.0 };
    double surroundingsTemperatureK { 0.0 };
    double gasDensityKgPerM3 { 0.0 };
    double gasVelocityMps { 0.0 };
    double gasSpecificHeatCpJPerKgK { 0.0 };
    double gasHeatCapacityJPerK { 0.0 };
    double gasTemperatureK { 0.0 };
    double durationSeconds { 0.0 };
};

struct DuctWallHeatTransferResult final {
    double reynoldsNumber { 0.0 };
    double nusseltNumber { 0.0 };
    double internalCoefficientWPerM2K { 0.0 };
    double wallHeatCapacityJPerK { 0.0 };
    /** Positive adds sensible energy to the gas. */
    double heatToGasJ { 0.0 };
    /** Positive rejects energy from the wall to its surroundings. */
    double heatRejectedJ { 0.0 };
};

/** Immutable circular-wall terms prepared once for a fixed duct cell. */
struct DuctWallHeatTransferGeometry final {
    double innerDiameterM { 0.0 };
    double wallHeatCapacityJPerK { 0.0 };
    double innerAreaM2 { 0.0 };
    double externalConductanceWPerK { 0.0 };

    [[nodiscard]] bool valid() const noexcept;
};

/** Conservative gas-to-solid heat exchange for a circular duct wall. */
class DuctWallHeatTransferModel final {
public:
    [[nodiscard]] static DuctWallHeatTransferGeometry prepareGeometry(
        double innerDiameterM,
        double lengthM,
        double wallThicknessM,
        double wallDensityKgPerM3,
        double wallSpecificHeatJPerKgK,
        double externalHeatTransferWPerM2K) noexcept;

    /** Advance using immutable geometry prepared outside the realtime loop. */
    [[nodiscard]] static DuctWallHeatTransferResult advancePrepared(
        DuctWallThermalState&,
        const DuctWallHeatTransferGeometry&,
        const DuctWallHeatTransferConditions&) noexcept;

    /** Compatibility entry point that prepares the geometry for this call. */
    [[nodiscard]] static DuctWallHeatTransferResult advance(
        DuctWallThermalState&, const DuctWallHeatTransferConditions&) noexcept;
};

} // namespace enginelab
