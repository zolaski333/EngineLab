#pragma once

namespace enginelab {

/** What a continuously variable transmission's variator reads. */
struct VariatorInput final {
    /** The driver's pedal (0-1). */
    double driverThrottle { 0.0 };
    /** The variator's output shaft speed (wheel speed x final drive). */
    double outputRadPerSecond { 0.0 };
    /** The variator's ratio range: the lowest (largest) and the tallest. */
    double lowestRatio { 2.5 };
    double tallestRatio { 0.4 };
    /** The ratio it holds now. */
    double currentRatio { 2.5 };
    double idleRpm { 800.0 };
    /** The active rev limit (the ECU tuner's, else the authored one). */
    double revLimitRpm { 6'500.0 };
    /** The speed a centrifugal clutch is fully shut at; 0 for a wet start
     * clutch. A scooter's variator never holds the engine below it. */
    double centrifugalLockRpm { 0.0 };
    /** Whether the start clutch carries the drive without slipping: until it
     * does, the variator stays at (or returns to) its lowest ratio. */
    bool coupled { false };
};

/** A belt variator's ratio and a scooter's centrifugal clutch.
 *
 * The ratio is chosen to hold the engine at a speed scheduled from the pedal:
 * low and economical foot-light, near peak power flat out, as production
 * CVTs do. A car's is set by hydraulics, a scooter's by roller weights, a
 * spring and the rear pulley's torque cam; that mechanical balance is not
 * modelled, only the engine speed it holds. The ratio moves at a bounded
 * rate (the full range in 1.5 s, not measured). */
class VariatorControl final {
public:
    /** The engine speed the variator aims for at this pedal. */
    [[nodiscard]] static double targetEngineRpm(const VariatorInput&) noexcept;
    /** The ratio after dtSeconds. */
    [[nodiscard]] static double nextRatio(const VariatorInput&, double dtSeconds) noexcept;
    /** A centrifugal clutch's share of its capacity (0-1): its shoes touch
     * the drum at engageRpm (0: 1.7 x idle) and it is fully shut at
     * centrifugalLockRpm(). */
    [[nodiscard]] static double centrifugalClutchPressure(double engineRpm, double engageRpm,
                                                         double idleRpm) noexcept;
    [[nodiscard]] static double centrifugalEngageRpm(double engageRpm, double idleRpm) noexcept;
    [[nodiscard]] static double centrifugalLockRpm(double engageRpm, double idleRpm) noexcept;
};

} // namespace enginelab
