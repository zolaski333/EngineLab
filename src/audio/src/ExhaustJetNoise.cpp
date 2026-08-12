#include <enginelab/audio/ExhaustJetNoise.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace enginelab {
namespace {

[[nodiscard]] bool positiveFinite(double value) noexcept {
    return std::isfinite(value) && value > 0.0;
}

[[nodiscard]] double equivalentDiameterM(double areaM2) noexcept {
    return positiveFinite(areaM2)
        ? 2.0 * std::sqrt(areaM2 / std::numbers::pi) : 0.0;
}

[[nodiscard]] float onePoleCoefficient(
    double frequencyHz, double sampleRateHz) noexcept {
    return static_cast<float>(1.0 - std::exp(
        -2.0 * std::numbers::pi * frequencyHz / sampleRateHz));
}

[[nodiscard]] float differenceFilterNormalisation(
    float fastCoefficient, float slowCoefficient) noexcept {
    const auto fast = std::clamp(
        static_cast<double>(fastCoefficient), 1.0e-7, 0.999999);
    const auto slow = std::clamp(
        static_cast<double>(slowCoefficient), 1.0e-7, fast * 0.999999);
    const auto fastVariance = fast / (2.0 - fast);
    const auto slowVariance = slow / (2.0 - slow);
    const auto covariance = fast * slow
        / (1.0 - (1.0 - fast) * (1.0 - slow));
    const auto differenceVariance = std::max(
        1.0e-12, fastVariance + slowVariance - 2.0 * covariance);
    return static_cast<float>(std::min(
        64.0, 1.0 / std::sqrt(differenceVariance)));
}

} // namespace

ExhaustJetNoise::ExhaustJetNoise(std::uint32_t seed) noexcept {
    setSeed(seed);
}

bool ExhaustJetNoise::prepare(double sampleRateHz) noexcept {
    prepared_ = std::isfinite(sampleRateHz) && sampleRateHz > 1'000.0;
    if (!prepared_) return false;
    sampleRateHz_ = sampleRateHz;
    configure(0.0, 0.001, 1.2, 343.0);
    snapToTarget();
    reset();
    return true;
}

void ExhaustJetNoise::reset() noexcept {
    lowSlow_ = 0.0F;
    lowFast_ = 0.0F;
    noiseState_ = initialSeed_;
}

void ExhaustJetNoise::setSeed(std::uint32_t seed) noexcept {
    // Xorshift32 has one forbidden all-zero state.
    initialSeed_ = seed != 0U ? seed : 0x6d2b79f5U;
    noiseState_ = initialSeed_;
}

double ExhaustJetNoise::limitedJetVelocityMps(
    double massFlowKgPerSecond, double outletAreaM2,
    double densityKgPerM3, double soundSpeedMps) noexcept {
    if (!positiveFinite(massFlowKgPerSecond)
        || !positiveFinite(outletAreaM2)
        || !positiveFinite(densityKgPerM3)
        || !positiveFinite(soundSpeedMps))
        return 0.0;
    const auto velocity = massFlowKgPerSecond
        / (densityKgPerM3 * outletAreaM2);
    // The law is subsonic. Clamp rather than extrapolating U^8 through a
    // choked/supersonic condition the reduced model cannot represent.
    return std::clamp(velocity, 0.0, 0.95 * soundSpeedMps);
}

double ExhaustJetNoise::centreFrequencyHz(
    double massFlowKgPerSecond, double outletAreaM2,
    double densityKgPerM3, double soundSpeedMps,
    double acousticTimeScale) noexcept {
    const auto velocity = limitedJetVelocityMps(
        massFlowKgPerSecond, outletAreaM2, densityKgPerM3, soundSpeedMps);
    const auto diameter = equivalentDiameterM(outletAreaM2);
    if (!(velocity > 0.0) || !(diameter > 0.0)
        || !positiveFinite(acousticTimeScale))
        return 0.0;
    return peakStrouhalNumber * velocity / diameter
        * std::clamp(acousticTimeScale, 0.25, 4.0);
}

double ExhaustJetNoise::acousticPowerWatts(
    double massFlowKgPerSecond, double outletAreaM2,
    double densityKgPerM3, double soundSpeedMps) noexcept {
    const auto velocity = limitedJetVelocityMps(
        massFlowKgPerSecond, outletAreaM2, densityKgPerM3, soundSpeedMps);
    if (!(velocity > 0.0)) return 0.0;
    const auto power = acousticPowerCoefficient * densityKgPerM3
        * outletAreaM2 * std::pow(velocity, 8.0)
        / std::pow(soundSpeedMps, 5.0);
    return std::isfinite(power) ? std::max(0.0, power) : 0.0;
}

void ExhaustJetNoise::computeBandTargets(double centreHz) noexcept {
    if (!(centreHz > 0.0) || !std::isfinite(centreHz)) return;
    const auto lowCutHz = std::clamp(
        centreHz * 0.45, 20.0, sampleRateHz_ * 0.40);
    const auto highCutHz = std::clamp(
        centreHz * 2.2, lowCutHz * 1.1, sampleRateHz_ * 0.45);
    slowCoefficientTarget_ = onePoleCoefficient(lowCutHz, sampleRateHz_);
    fastCoefficientTarget_ = onePoleCoefficient(highCutHz, sampleRateHz_);
    bandNormalisationTarget_ = differenceFilterNormalisation(
        fastCoefficientTarget_, slowCoefficientTarget_);
}

void ExhaustJetNoise::configure(
    double massFlowKgPerSecond, double outletAreaM2,
    double densityKgPerM3, double soundSpeedMps,
    double acousticTimeScale) noexcept {
    targetCentreFrequencyHz_ = centreFrequencyHz(
        massFlowKgPerSecond, outletAreaM2, densityKgPerM3,
        soundSpeedMps, acousticTimeScale);
    const auto powerWatts = acousticPowerWatts(
        massFlowKgPerSecond, outletAreaM2, densityKgPerM3, soundSpeedMps);
    // W = 4*pi*r^2*p_rms^2/(rho*c), evaluated at r=1 m.
    targetPressureRmsPa_ = powerWatts > 0.0
        ? std::sqrt(powerWatts * densityKgPerM3 * soundSpeedMps
            / (4.0 * std::numbers::pi))
        : 0.0;
    meanVolumeVelocityTargetM3PerS_ =
        positiveFinite(massFlowKgPerSecond)
            && positiveFinite(densityKgPerM3)
        ? static_cast<float>(massFlowKgPerSecond / densityKgPerM3) : 0.0F;
    inverseOutletAreaTargetM2_ = positiveFinite(outletAreaM2)
        ? static_cast<float>(1.0 / outletAreaM2) : 0.0F;
    // p_rms(1 m) = rho*sqrt(K*A/(4*pi))/c^2 * U^4.
    pressurePerVelocityFourthTarget_ =
        positiveFinite(outletAreaM2)
            && positiveFinite(densityKgPerM3)
            && positiveFinite(soundSpeedMps)
        ? static_cast<float>(densityKgPerM3 * std::sqrt(
            acousticPowerCoefficient * outletAreaM2
                / (4.0 * std::numbers::pi))
            / (soundSpeedMps * soundSpeedMps))
        : 0.0F;
    if (targetCentreFrequencyHz_ > 0.0)
        computeBandTargets(targetCentreFrequencyHz_);
}

void ExhaustJetNoise::snapToTarget() noexcept {
    meanVolumeVelocityM3PerS_ = meanVolumeVelocityTargetM3PerS_;
    inverseOutletAreaM2_ = inverseOutletAreaTargetM2_;
    pressurePerVelocityFourth_ = pressurePerVelocityFourthTarget_;
    slowCoefficient_ = slowCoefficientTarget_;
    fastCoefficient_ = fastCoefficientTarget_;
    bandNormalisation_ = bandNormalisationTarget_;
}

float ExhaustJetNoise::nextUnitRmsWhiteNoise() noexcept {
    noiseState_ ^= noiseState_ << 13U;
    noiseState_ ^= noiseState_ >> 17U;
    noiseState_ ^= noiseState_ << 5U;
    // Uniform [-1, 1] has RMS 1/sqrt(3).
    return static_cast<float>(
        static_cast<double>(noiseState_) / 2'147'483'647.5 - 1.0)
        * static_cast<float>(std::sqrt(3.0));
}

float ExhaustJetNoise::process(float rampCoefficient) noexcept {
    if (!prepared_) return 0.0F;
    const auto ramp = std::clamp(rampCoefficient, 0.0F, 1.0F);
    meanVolumeVelocityM3PerS_ += ramp
        * (meanVolumeVelocityTargetM3PerS_
            - meanVolumeVelocityM3PerS_);
    inverseOutletAreaM2_ += ramp
        * (inverseOutletAreaTargetM2_ - inverseOutletAreaM2_);
    pressurePerVelocityFourth_ += ramp
        * (pressurePerVelocityFourthTarget_
            - pressurePerVelocityFourth_);
    slowCoefficient_ += ramp
        * (slowCoefficientTarget_ - slowCoefficient_);
    fastCoefficient_ += ramp
        * (fastCoefficientTarget_ - fastCoefficient_);
    bandNormalisation_ += ramp
        * (bandNormalisationTarget_ - bandNormalisation_);
    const auto velocity = std::max(0.0,
        static_cast<double>(meanVolumeVelocityM3PerS_)
            * static_cast<double>(inverseOutletAreaM2_));
    const auto velocitySquared = velocity * velocity;
    const auto pressureRmsPa = static_cast<double>(
        pressurePerVelocityFourth_) * velocitySquared * velocitySquared;
    if (!(pressureRmsPa > 0.0)) return 0.0F;

    const auto white = nextUnitRmsWhiteNoise();
    lowSlow_ += slowCoefficient_ * (white - lowSlow_);
    lowFast_ += fastCoefficient_ * (white - lowFast_);
    const auto result = (lowFast_ - lowSlow_)
        * bandNormalisation_ * static_cast<float>(
            std::min(pressureRmsPa, 100'000.0));
    return std::isfinite(result) ? result : 0.0F;
}

} // namespace enginelab
