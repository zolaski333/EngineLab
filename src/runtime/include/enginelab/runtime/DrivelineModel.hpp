#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

#include <algorithm>

namespace enginelab {

struct DrivelineOutput final {
    double engineReactionTorqueNm { 0.0 };
    /** Equivalent crank torque used together with
     * reflectedRotatingInertiaKgM2 by EngineSimulator. While slipping this is
     * the ordinary clutch reaction; while sticking it is the road-side load
     * reflected through the ratio, without counting coupled inertia as load. */
    double engineCouplingTorqueNm { 0.0 };
    double reflectedRotatingInertiaKgM2 { 0.0 };
    double requestedLoad { 0.0 };
    double torqueCutMultiplier { 1.0 };
    int engagedGear { -1 };
    double clutchPressure { 0.0 };
    double vehicleSpeedMps { 0.0 };
    double vehicleDistanceM { 0.0 };
    double wheelTorqueNm { 0.0 };
    double clutchTorqueNm { 0.0 };
    double clutchSlipRpm { 0.0 };
    /** Fraction of the public frame for which the Karnopp clutch law solved a
     * static-friction constraint rather than kinetic slip. */
    double clutchStickFraction { 0.0 };
    double shiftProgress { 0.0 };
    bool shiftInProgress { false };
    /** A quickshifter upshift: the ignition is cut while the dogs change. */
    bool ignitionCut { false };
    /** An auto-blipper downshift: the throttle opening it commands while the
     * dogs are free, to bring the engine to the new gear's speed. */
    double blipThrottle { 0.0 };
    double brakePressure { 0.0 };
    double brakeForceN { 0.0 };
    double roadLoadForceN { 0.0 };
    double tireForceN { 0.0 };
    double drivenAxleNormalForceN { 0.0 };
    double longitudinalAccelerationMps2 { 0.0 };
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
    /** The engine's rotating inertia after a live change of its geometry. */
    void setEngineInertia(double kgM2) noexcept { engineInertiaKgM2_ = std::max(0.001, kgM2); }
    /** A live change of the gearbox and the vehicle: read at every advance(),
     * the clutch, wheel and vehicle states carry on. False, with nothing
     * changed, for another number of gears. Copies into the existing
     * storage: no allocation. */
    [[nodiscard]] bool replaceVehicle(const TransmissionConfig& transmission, const VehicleConfig& vehicle);
    [[nodiscard]] DrivelineOutput advance(double dtSeconds, const EngineState&, double requestedLoad,
                                          double clutchPedalPressure, double brakePressure) noexcept;
private:
    /** How the shift under way is made (TransmissionConfig::shiftStyle). */
    enum class ShiftAction : std::uint8_t {
        /** The driver's clutch dip and throttle lift, over a timer. */
        timed,
        /** Ignition cut while the dogs change, clutch left in. */
        quickshift,
        /** Dogs free, throttle blipped to the new gear's speed. */
        blip,
        /** The oncoming clutch takes the drive and pulls the engine down. */
        handOver
    };
    [[nodiscard]] ShiftAction chooseShiftAction() const noexcept;
    [[nodiscard]] double ratioForGear(int gear) const noexcept;
    [[nodiscard]] double selectedRatio() const noexcept;
    EngineConfig config_;
    ShiftAction shiftAction_ { ShiftAction::timed };
    // While a blipped downshift frees the dogs, no gear carries the drive.
    bool dogsFree_ { false };
    // The slip the oncoming clutch of a hand-over must take out, captured
    // when it takes the drive.
    double handOverReferenceSlipRadPerSecond_ { 0.0 };
    // Engine + flywheel rotating inertia, cached so the locked-clutch constraint can
    // share one acceleration between the crank and the driveline.
    double engineInertiaKgM2_ { 0.24 };
    int requestedGear_ { -1 };
    int engagedGear_ { -1 };
    int shiftTargetGear_ { -1 };
    int shiftFromGear_ { -1 };
    double shiftElapsedSeconds_ { 0.0 };
    bool shiftInProgress_ { false };
    // Post-timer clutch resynchronisation on a forward upshift: the reference
    // slip captured when the sync-aware torque hold begins (0 while not holding),
    // and the clutch slip carried from the previous advance() so the hold can be
    // decided before this tick's mechanical loop recomputes it.
    double resyncReferenceSlipRpm_ { 0.0 };
    double lastClutchSlipRpm_ { 0.0 };
    double wheelAngularVelocityRadPerSecond_ { 0.0 };
    double vehicleSpeedMps_ { 0.0 };
    double vehicleDistanceM_ { 0.0 };
    double longitudinalAccelerationMps2_ { 0.0 };
    double clutchTemperatureC_ { 22.0 };
    double clutchDissipatedEnergyJoules_ { 0.0 };
    double previousStoredEnergyJoules_ { 0.0 };
    bool energyInitialised_ { false };
};

} // namespace enginelab
