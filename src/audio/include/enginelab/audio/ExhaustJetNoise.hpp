#pragma once

#include <cstdint>

namespace enginelab {

/** Semi-empirical turbulent mixing noise from one exhaust outlet.
 *
 * The source is deliberately outside the gas solver: it converts the resolved
 * mean outlet flow into far-field pressure without returning any invented wave
 * to the pipe. Acoustic power follows the classical compact subsonic-jet law
 *
 *   W = K * rho * A * U^8 / c^5
 *
 * and the broadband peak follows Strouhal fD/U = 0.2. The coefficient is the
 * published low-speed-jet order of magnitude, not engine-specific measured
 * data, so this renderer is explicitly semi-empirical.
 *
 * configure() is block-rate and may evaluate transcendentals. process() is
 * allocation-free and contains no transcendentals, locks, or random services.
 */
class ExhaustJetNoise final {
public:
    /** Clean free-jet reference from the classical low-speed correlation. */
    static constexpr double cleanJetAcousticPowerCoefficient { 1.0e-4 };
    /** Authored hot-engine correction for upstream turbulence.
     *
     * Low-speed engine jets are measurably louder than clean laboratory nozzles
     * because their inflow is already turbulent. Keeping the correction named
     * and separate prevents the listening calibration from being mistaken for
     * a universal fluid constant. The current +20 dB power correction keeps the
     * source below the physical blowdown bus while making it resolvable in an
     * instrumented same-binary A/B. It is not a loudness target.
     */
    static constexpr double engineOutletTurbulencePowerMultiplier { 100.0 };
    static constexpr double acousticPowerCoefficient {
        cleanJetAcousticPowerCoefficient
            * engineOutletTurbulencePowerMultiplier
    };
    static constexpr double peakStrouhalNumber { 0.2 };
    explicit ExhaustJetNoise(std::uint32_t seed = 0x6d2b79f5U) noexcept;

    [[nodiscard]] bool prepare(double sampleRateHz) noexcept;
    void reset() noexcept;
    void setSeed(std::uint32_t seed) noexcept;

    /** Set the next block's physical operating point.
     *
     * massFlowKgPerSecond is the flow assigned to this outlet. outletAreaM2,
     * density and sound speed describe the local hot jet. acousticTimeScale
     * moves the spectrum for slow-motion auditioning but does not alter power.
     */
    void configure(double massFlowKgPerSecond,
                   double outletAreaM2,
                   double densityKgPerM3,
                   double soundSpeedMps,
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
        double densityKgPerM3, double soundSpeedMps) noexcept;

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
