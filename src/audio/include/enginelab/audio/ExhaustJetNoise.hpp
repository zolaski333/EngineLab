#pragma once

#include <cstdint>

namespace enginelab {

/** Turbulent mixing noise from one exhaust outlet.
 *
 * The source is deliberately outside the gas solver: it converts the resolved
 * mean outlet flow into far-field pressure without returning any invented wave
 * to the pipe. Acoustic power follows Lighthill's compact subsonic-jet law
 *
 *   W = K * rho0 * A * U^8 / c0^5
 *
 * with U the jet's own velocity and rho0, c0 the AMBIENT air the sound
 * radiates into, and the broadband peak follows Strouhal fD/U = 0.2. Tanna
 * (J. Sound Vib. 50, 1977) and Viswanathan (J. Fluid Mech. 516, 2004) scale
 * hot and cold jets alike on Vj/a0: at a given U, heating the jet moves the
 * power by about +-2 dB. Evaluated on the hot gas instead, rho/c^5 and the
 * radiation impedance made it (Tj/T0)^4 too quiet: 15 dB at 700 K. Their data
 * start at Vj/a0 0.5; a tailpipe runs at 0.05-0.3.
 *
 * configure() is block-rate and may evaluate transcendentals. process() is
 * allocation-free and contains no transcendentals, locks, or random services.
 */
class ExhaustJetNoise final {
public:
    /** Lighthill's coefficient on the exit area, the low-speed-jet order of
     * magnitude (acoustic efficiency 2 K M^5: 1.2e-4 at M 0.9). It is the
     * jet's alone: a silencer's flow noise, which an outlet also radiates,
     * is MufflerFlowNoise, and replaces the x100 "upstream turbulence"
     * multiplier this coefficient used to carry. */
    static constexpr double acousticPowerCoefficient { 1.0e-4 };
    /** Ambient air when no far-field density is supplied: 20 C, 101.3 kPa. */
    static constexpr double standardAmbientDensityKgPerM3 { 1.204 };
    /** Ambient sound speed (20 C): within 3.5 % from -20 to 40 C, 0.6 dB. */
    static constexpr double ambientSoundSpeedMps { 343.2 };
    static constexpr double peakStrouhalNumber { 0.2 };
    explicit ExhaustJetNoise(std::uint32_t seed = 0x6d2b79f5U) noexcept;

    [[nodiscard]] bool prepare(double sampleRateHz) noexcept;
    void reset() noexcept;
    void setSeed(std::uint32_t seed) noexcept;

    /** Set the next block's physical operating point.
     *
     * massFlowKgPerSecond is the flow assigned to this outlet. outletAreaM2,
     * density and sound speed describe the local hot jet, which sets U; the
     * ambient density is the air it radiates into. acousticTimeScale moves
     * the spectrum for slow-motion auditioning but does not alter power.
     */
    void configure(double massFlowKgPerSecond,
                   double outletAreaM2,
                   double densityKgPerM3,
                   double soundSpeedMps,
                   double ambientDensityKgPerM3,
                   double acousticTimeScale = 1.0) noexcept;

    /** Snap live coefficients to the configured target at initialisation. */
    void snapToTarget() noexcept;

    /** Return turbulent pressure at one metre, in pascals.
     *
     * The configured signed gas-network outlet flow is already the complete
     * quasi-1D flow through the mouth. The acoustic radiation volume velocity
     * describes the same wave and must not be added a second time before the
     * U^8 law; doing so made sparse engines radiate isolated broadband spikes.
     */
    [[nodiscard]] float process(float rampCoefficient) noexcept;

    [[nodiscard]] double targetCentreFrequencyHz() const noexcept {
        return targetCentreFrequencyHz_;
    }
    [[nodiscard]] double targetPressureRmsPa() const noexcept {
        return targetPressureRmsPa_;
    }

    [[nodiscard]] static double limitedJetVelocityMps(
        double massFlowKgPerSecond, double outletAreaM2,
        double densityKgPerM3, double soundSpeedMps) noexcept;
    [[nodiscard]] static double centreFrequencyHz(
        double massFlowKgPerSecond, double outletAreaM2,
        double densityKgPerM3, double soundSpeedMps,
        double acousticTimeScale = 1.0) noexcept;
    [[nodiscard]] static double acousticPowerWatts(
        double massFlowKgPerSecond, double outletAreaM2,
        double densityKgPerM3, double soundSpeedMps,
        double ambientDensityKgPerM3) noexcept;

private:
    [[nodiscard]] float nextUnitRmsWhiteNoise() noexcept;
    void computeBandTargets(double centreHz) noexcept;

    double sampleRateHz_ { 48'000.0 };
    double targetCentreFrequencyHz_ {};
    double targetPressureRmsPa_ {};
    float meanVolumeVelocityM3PerS_ {};
    float meanVolumeVelocityTargetM3PerS_ {};
    float inverseOutletAreaM2_ {};
    float inverseOutletAreaTargetM2_ {};
    float pressurePerVelocityFourth_ {};
    float pressurePerVelocityFourthTarget_ {};
    float slowCoefficient_ {};
    float slowCoefficientTarget_ {};
    float fastCoefficient_ {};
    float fastCoefficientTarget_ {};
    float bandNormalisation_ {};
    float bandNormalisationTarget_ {};
    float lowSlow_ {};
    float lowFast_ {};
    std::uint32_t initialSeed_ { 0x6d2b79f5U };
    std::uint32_t noiseState_ { 0x6d2b79f5U };
    bool prepared_ { false };
};

} // namespace enginelab
