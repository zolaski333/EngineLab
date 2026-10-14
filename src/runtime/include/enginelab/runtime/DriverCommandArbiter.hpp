#pragma once

#include <enginelab/foundation/EngineTypes.hpp>
#include <enginelab/runtime/DrivelineModel.hpp>

#include <cstdint>

namespace enginelab {

/** What the driver asks for this frame: pedals and keys, before any device
 * of the driveline acts on them. */
struct DriverInputs final {
    double throttle { 0.0 };
    /** The clutch the driver holds: 1 engaged (pedal up), 0 declutched. */
    double clutchPedal { 1.0 };
    double brake { 0.0 };
};

/** Which actor set the engine's throttle this frame. */
enum class ThrottleAuthority : std::uint8_t {
    /** The driver's pedal, untouched. */
    driver,
    /** The driver's pedal, lifted by a clutch shift's torque cut. */
    shiftTorqueCut,
    /** An auto-blipped downshift opened the throttle past the driver's. */
    blipper
};

/** One public frame of the vehicle: the driveline's answer and the commands
 * the engine receives for it. */
struct VehicleStep final {
    DrivelineOutput drive;
    double engineThrottle { 0.0 };
    bool shiftIgnitionCut { false };
    ThrottleAuthority throttleAuthority { ThrottleAuthority::driver };
};

/** The one place where the driver and the driveline's own devices (a clutch
 * shift's torque cut, a quickshifter's ignition cut, an auto-blipper) become
 * engine commands.
 *
 * EngineRuntime, the tests and the harnesses all step the vehicle through
 * advanceVehicle(), so they cannot drift apart, and a device added later
 * (cruise control, launch control, a dual clutch's own launch clutch) is
 * arbitrated here once instead of in every caller.
 *
 * Rules, in order:
 *  1. the driveline is given the clutch drivelineClutch() returns;
 *  2. a clutch shift scales the driver's throttle by its torque cut;
 *  3. an auto-blipper may open the throttle past that, never close it;
 *  4. a quickshifter's ignition cut passes through to the ECU.
 * The dyno bypasses all of it: the bench, not a driver, holds the engine. */
class DriverCommandArbiter final {
public:
    /** The clutch the driveline is given this frame. */
    [[nodiscard]] static double drivelineClutch(const DriverInputs& inputs,
                                                const TransmissionConfig& transmission) noexcept;
    /** The engine commands for this frame's driveline output. */
    static void arbitrate(const DriverInputs& inputs, VehicleStep& step) noexcept;
};

/** Advances the driveline one public frame on the previous frame's engine
 * state, then arbitrates the engine commands for it. */
[[nodiscard]] VehicleStep advanceVehicle(DrivelineModel& driveline, double dtSeconds,
                                         const EngineState& engineState, double requestedLoad,
                                         const DriverInputs& inputs) noexcept;

/** Writes a vehicle step into the engine controls: throttle, ignition cut,
 * brake, the driveline's coupling torque and reflected inertia, and the
 * cooling air of the vehicle's speed. The caller sets ignition and starter. */
void applyVehicleStep(EngineControls& controls, const VehicleStep& step) noexcept;

} // namespace enginelab
