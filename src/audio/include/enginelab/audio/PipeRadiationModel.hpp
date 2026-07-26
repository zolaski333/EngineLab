#pragma once

#include <complex>

namespace enginelab {

/** One audio-sample result at an unflanged circular pipe termination. */
struct PipeRadiationSample final {
    /** Pressure wave travelling back into the pipe, in pascals. */
    double reflectedPressurePa { 0.0 };
    /** Net outward plane-wave volume velocity at the mouth, in m^3/s. */
    double outletVolumeVelocityM3PerS { 0.0 };
    /** Low-frequency far-field monopole pressure at the configured distance. */
    double farFieldPressurePa { 0.0 };
};

/** Passive causal radiation load for an unflanged circular pipe.
 *
 * The pressure-reflection filter is the causal (1,2) Padé approximation fitted
 * by Silva et al. to the Levine-Schwinger solution:
 * https://doi.org/10.1016/j.jsv.2008.11.008
 *
 *   R(s) = -(1 + n1 (a/c) s)
 *          / (1 + d1 (a/c) s + d2 (a/c)^2 s^2)
 *
 * with n1=0.167, d1=1.393 and d2=0.457. A bilinear transform maps that
 * continuous passive load to the audio sample rate. The radiated signal is
 * calculated from the mouth volume acceleration, not from an arbitrary EQ or
 * exhaust gain curve.
 *
 * Validity: plane mode, unflanged sharp-edged circular opening, linear
 * acoustics and negligible mean-flow correction at the termination.
 */
class UnflangedPipeRadiation final {
public:
    /** Configure geometry and observer. Clears all filter history. */
    [[nodiscard]] bool prepare(double sampleRateHz,
                               double pipeRadiusM,
                               double observerDistanceM) noexcept;

    /** Update the locally linearised gas medium without clearing history. */
    [[nodiscard]] bool setMedium(double densityKgPerM3,
                                 double speedOfSoundMps) noexcept;

    /** Add a passive amplitude-dependent series resistance at the open edge.
     *
     * The dimensionless resistance is coefficient * |u| / c, where u is mouth
     * particle velocity. Zero preserves the linear model exactly. For a
     * thin-wall unflanged edge, quasi-steady vortex shedding gives
     * coefficient = 4/(3*pi).
     */
    [[nodiscard]] bool setNonlinearLossCoefficient(double coefficient) noexcept;

    void reset() noexcept;

    /** Terminate one incident pressure-wave sample, in SI units. */
    [[nodiscard]] PipeRadiationSample process(double incidentPressurePa) noexcept;

    /** Digital reflection transfer function at a physical frequency. */
    [[nodiscard]] std::complex<double> reflectionCoefficient(
        double frequencyHz) const noexcept;

    [[nodiscard]] double characteristicImpedancePaSPerM3() const noexcept {
        return characteristicImpedancePaSPerM3_;
    }
    [[nodiscard]] double planeModeCutoffHz() const noexcept;
    [[nodiscard]] bool prepared() const noexcept { return prepared_; }

private:
    void updateCoefficients() noexcept;

    double sampleRateHz_ { 0.0 };
    double radiusM_ { 0.0 };
    double observerDistanceM_ { 0.0 };
    double densityKgPerM3_ { 1.2 };
    double speedOfSoundMps_ { 343.0 };
    double characteristicImpedancePaSPerM3_ { 0.0 };
    double b0_ { 0.0 };
    double b1_ { 0.0 };
    double b2_ { 0.0 };
    double a1_ { 0.0 };
    double a2_ { 0.0 };
    double input1_ { 0.0 };
    double input2_ { 0.0 };
    double output1_ { 0.0 };
    double output2_ { 0.0 };
    double previousVolumeVelocityM3PerS_ { 0.0 };
    double nonlinearLossCoefficient_ { 0.0 };
    bool prepared_ { false };
};

} // namespace enginelab
