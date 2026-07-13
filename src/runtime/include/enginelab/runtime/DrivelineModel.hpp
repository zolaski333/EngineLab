#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

namespace enginelab {

struct DrivelineOutput final {
    double engineReactionTorqueNm { 0.0 };
    double requestedLoad { 0.0 };
    double torqueCutMultiplier { 1.0 };
    int engagedGear { -1 };
    double clutchPressure { 0.0 };
    double vehicleSpeedMps { 0.0 };
    double vehicleDistanceM { 0.0 };
    double wheelTorqueNm { 0.0 };
    double clutchTorqueNm { 0.0 };
    double clutchSlipRpm { 0.0 };
    double shiftProgress { 0.0 };
    bool shiftInProgress { false };
    double brakePressure { 0.0 };
    double brakeForceN { 0.0 };
    double tireForceN { 0.0 };
    bool tractionLimited { false };
    double clutchTemperatureC { 22.0 };
    double clutchDissipatedEnergyJoules { 0.0 };
    double clutchPowerLossKw { 0.0 };
    double storedEnergyJoules { 0.0 };
    double energyResidualJoules { 0.0 };
};

/** Energy-coupled clutch, gearbox, driven wheel and vehicle longitudinal model. */
class DrivelineModel final {
public:
    explicit DrivelineModel(const EngineConfig&);
    void requestGear(int gear) noexcept;
    void shiftUp() noexcept;
    void shiftDown() noexcept;
    [[nodiscard]] int requestedGear() const noexcept { return requestedGear_; }
    [[nodiscard]] DrivelineOutput advance(double dtSeconds, const EngineState&, double requestedLoad,
                                          double clutchPedalPressure, double brakePressure) noexcept;
private:
    [[nodiscard]] double selectedRatio() const noexcept;
    EngineConfig config_;
    int requestedGear_ { -1 };
    int engagedGear_ { -1 };
    int shiftTargetGear_ { -1 };
    double shiftElapsedSeconds_ { 0.0 };
    bool shiftInProgress_ { false };
    double wheelAngularVelocityRadPerSecond_ { 0.0 };
    double vehicleSpeedMps_ { 0.0 };
    double vehicleDistanceM_ { 0.0 };
    double clutchTemperatureC_ { 22.0 };
    double clutchDissipatedEnergyJoules_ { 0.0 };
    double previousStoredEnergyJoules_ { 0.0 };
    bool energyInitialised_ { false };
};

} // namespace enginelab
