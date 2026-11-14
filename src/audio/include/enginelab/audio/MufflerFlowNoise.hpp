#pragma once

#include <cstdint>

namespace enginelab {

/** Flow-generated noise of a silencer element, radiated by its tailpipe.
 *
 * Mean flow through a silencer sheds turbulence at its area steps and
 * perforations. The source is monopole and dipole, so its power grows as
 * u^4-u^5 and not a free jet's u^8. Green and Smith (1988) measured it on
 * a flow bench as an efficiency eta on the duct's mechanical flow power:
 *
 *   W = eta * (rho u^3 / 2) * (pi D^2 / 4) * M^1.5
 *
 * where D is the element's duct diameter, u and rho the mean gas velocity
 * and density there, and M = u/c. Kunz and Garcia (SAE 950546) found the same
 * efficiencies with cold air and with hot exhaust gas. Its octave spectrum is
 * flat, so the tailpipe radiates it as pink noise. Both the law and eta's
 * curves, expressed as eta_w = 10 log10(eta) + 120 dB, are Onorati's account
 * of that work (Winterbone and Pearson, *Design Techniques for Engine
 * Manifolds*, 1999, 6.12, eq. 6.35-6.37, Figs 6.43-6.45). His calculated car
 * exhaust has this noise overtake the pulse noise above about 3,500 rpm
 * (Fig. 6.46).
 *
 * The renderer is allocation-free and holds no transcendental in process().
 */
class MufflerFlowNoise final {
public:
    /** eta_w (dB) of a plain expansion chamber against its length over its
     * inlet pipe's diameter: Fig. 6.43 (65 dB at L/D 0, 80.5 by 2, ~81.5 on
     * its plateau), here within ~1 dB of the curve as
     * 65 + 16.5 (1 - exp(-(L/D)/0.75)), L/D held to 0-6. */
    [[nodiscard]] static double expansionChamberEfficiencyDb(
        double lengthOverDiameter) noexcept;
    /** eta_w (dB) of a chamber bridged by a perforated duct (3 mm holes),
     * eq. 6.37 and Fig. 6.45: eta_w0(sigma) + 4 L/D, with D the perforated
     * duct's diameter. sigma (percent open area) is held to 2-27 %: Green
     * and Smith measured 4.2-27 %, and the figure's line beyond is Onorati's
     * extrapolation. L/D is held to 0-6, the figure's span. */
    [[nodiscard]] static double perforateEfficiencyDb(
        double porosityPercent, double lengthOverDiameter) noexcept;
    /** eq. 6.35 with f(M) = M^1.5: the element's flow-noise power, W. */
    [[nodiscard]] static double soundPowerWatts(double efficiencyDb,
        double massFlowKgPerSecond, double ductDiameterM,
        double densityKgPerM3, double soundSpeedMps) noexcept;

    /** The band the pink spectrum is held to: the 63 Hz to 8 kHz octaves. */
    static constexpr double lowBandEdgeHz = 44.2;
    static constexpr double highBandEdgeHz = 11'300.0;

    explicit MufflerFlowNoise(std::uint32_t seed = 0x2545f491U) noexcept;

    [[nodiscard]] bool prepare(double sampleRateHz) noexcept;
    void reset() noexcept;
    void setSeed(std::uint32_t seed) noexcept;
    /** The next block's RMS pressure at one metre, Pa. */
    void setTargetPressureRmsPa(double pressureRmsPa) noexcept;
    void snapToTarget() noexcept;
    [[nodiscard]] double targetPressureRmsPa() const noexcept {
        return targetPressureRmsPa_;
    }
    /** Pressure at one metre, Pa: band-limited pink noise at the target RMS. */
    [[nodiscard]] float process(float rampCoefficient) noexcept;

private:
    struct FilterState final {
        float b0 {}, b1 {}, b2 {}, b3 {}, b4 {}, b5 {}, b6 {};
        float highPassInput {};
        float highPassOutput {};
        float lowPassOutput {};
    };
    [[nodiscard]] float nextUnitWhiteNoise(std::uint32_t& state) const noexcept;
    [[nodiscard]] float filter(float white, FilterState& state) const noexcept;

    double sampleRateHz_ { 48'000.0 };
    double targetPressureRmsPa_ {};
    float pressureRmsPa_ {};
    float pressureRmsTargetPa_ {};
    float highPassCoefficient_ {};
    float lowPassCoefficient_ {};
    float unitRmsNormalisation_ {};
    FilterState state_ {};
    std::uint32_t initialSeed_ { 0x2545f491U };
    std::uint32_t noiseState_ { 0x2545f491U };
    bool prepared_ { false };
};

} // namespace enginelab
