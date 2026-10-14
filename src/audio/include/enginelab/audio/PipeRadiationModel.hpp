#pragma once

#include <cmath>
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

    /** Density of the medium the far field travels through. Zero (the
     * default) uses the duct medium of setMedium. Outside the mouth the field
     * is in ambient air while the volume velocity is continuous across the
     * hot/cold interface, so p = rho_ambient dQ/dt / (4 pi r); the duct value
     * scales it by rho_duct / rho_ambient. Only the far field changes: the
     * reflection and the volume velocity are untouched. Diagnostic switch. */
    [[nodiscard]] bool setFarFieldDensity(double densityKgPerM3) noexcept {
        if (!std::isfinite(densityKgPerM3) || densityKgPerM3 < 0.0) return false;
        farFieldDensityKgPerM3_ = densityKgPerM3;
        return true;
    }

    /** Replace the Levine-Schwinger load by an ideal pressure release, R = -1
     * at every frequency: the mouth returns all the energy to the pipe and the
     * far field still follows the mouth volume acceleration. Not physical above
     * ka ~ 0.3; it exists to measure what the radiation load does to the sound
     * (another simulator terminates its pipes this way). Diagnostic switch. */
    void setPressureRelease(bool enabled) noexcept {
        pressureRelease_ = enabled;
        if (prepared_) updateCoefficients();
    }

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
    double farFieldDensityKgPerM3_ { 0.0 };
    bool pressureRelease_ { false };
    bool prepared_ { false };
};

} // namespace enginelab
