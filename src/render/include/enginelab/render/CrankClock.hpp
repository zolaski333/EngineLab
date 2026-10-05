#pragma once

namespace enginelab::render {

/**
 * Crank angle to draw at an arbitrary display time.
 *
 * The UI reads the simulator about 30 times a second, while the screen may
 * refresh 144 times a second. In real time the clock predicts the angle from
 * the last sample and the engine speed, then pulls the prediction back onto
 * each new sample over about a tenth of a second (a phase-locked loop), so the
 * drawn crank turns at exactly the simulated speed without stepping.
 *
 * In slow motion the picture is a slowed replay of the current engine speed:
 * the clock integrates rpm x factor itself and no longer follows the
 * simulator's angle, which would otherwise run hundreds of times ahead.
 */
class CrankClock final {
public:
    /** Records a simulator sample taken at `wallSeconds`. `rate` is the
        simulation speed multiplier (1 = real time, 0 when paused). */
    void observe(double wallSeconds, double crankAngleDegrees, double rpm, double rate) noexcept;

    /** 1 = real time, 0.02 = 1:50, 0 = freeze. */
    void setPlaybackFactor(double factor) noexcept;
    [[nodiscard]] double playbackFactor() const noexcept { return factor_; }

    /** Advances to `wallSeconds` and returns the angle to draw, in [0, 720). */
    [[nodiscard]] double advance(double wallSeconds) noexcept;

    /** Angle the crank sweeps per second of display time, in degrees. */
    [[nodiscard]] double displayDegreesPerSecond() const noexcept;

    void reset() noexcept;

private:
    [[nodiscard]] bool realTime() const noexcept { return factor_ >= 0.999; }

    [[nodiscard]] double rpmAt(double wallSeconds) const noexcept;

    bool hasSample_ { false };
    double sampleWall_ {};
    /** Engine acceleration between the last two samples, in rpm per second. */
    double rpmPerSecond_ {};
    double sampleAngle_ {};
    double rpm_ {};
    double rate_ { 1.0 };
    double factor_ { 1.0 };
    double displayWall_ {};
    double displayAngle_ {};
    bool hasDisplay_ { false };
};

/** Wraps any angle into [0, 720). */
[[nodiscard]] double wrapCycleDegrees(double degrees) noexcept;

} // namespace enginelab::render
