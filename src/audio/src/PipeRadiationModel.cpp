#include <enginelab/audio/PipeRadiationModel.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace enginelab {
namespace {

constexpr double padeN1 = 0.167;
constexpr double padeD1 = 1.393;
constexpr double padeD2 = 0.457;
constexpr double firstAxisymmetricModeRoot = 3.8317059702075125;

[[nodiscard]] bool positiveFinite(double value) noexcept {
    return std::isfinite(value) && value > 0.0;
}

} // namespace

bool UnflangedPipeRadiation::prepare(double sampleRateHz,
                                     double pipeRadiusM,
                                     double observerDistanceM) noexcept {
    prepared_ = false;
    if (!positiveFinite(sampleRateHz) || sampleRateHz < 1'000.0
        || !positiveFinite(pipeRadiusM)
        || !positiveFinite(observerDistanceM))
        return false;
    sampleRateHz_ = sampleRateHz;
    radiusM_ = pipeRadiusM;
    observerDistanceM_ = observerDistanceM;
    updateCoefficients();
    reset();
    prepared_ = true;
    return true;
}

bool UnflangedPipeRadiation::setMedium(double densityKgPerM3,
                                       double speedOfSoundMps) noexcept {
    if (!prepared_ || !positiveFinite(densityKgPerM3)
        || !positiveFinite(speedOfSoundMps))
        return false;
    densityKgPerM3_ = densityKgPerM3;
    speedOfSoundMps_ = speedOfSoundMps;
    updateCoefficients();
    return true;
}

bool UnflangedPipeRadiation::setNonlinearLossCoefficient(
    double coefficient) noexcept {
    if (!std::isfinite(coefficient) || coefficient < 0.0) return false;
    nonlinearLossCoefficient_ = coefficient;
    return true;
}

void UnflangedPipeRadiation::reset() noexcept {
    input1_ = 0.0;
    input2_ = 0.0;
    output1_ = 0.0;
    output2_ = 0.0;
    previousVolumeVelocityM3PerS_ = 0.0;
}

PipeRadiationSample UnflangedPipeRadiation::process(
    double incidentPressurePa) noexcept {
    PipeRadiationSample result;
    if (!prepared_) return result;
    const auto input = std::isfinite(incidentPressurePa) ? incidentPressurePa : 0.0;
    const auto history = b1_ * input1_ + b2_ * input2_
        - a1_ * output1_ - a2_ * output2_;

    // Cascade a zero-length positive series resistance with the linear
    // radiation impedance in wave variables. If a is the incoming wave, b the
    // returned wave, c/d the waves on the radiation side and r=R/Zc:
    //   c = (2a + rH) / (2 + r(1-b0)), d = b0*c + H, b = a-c+d.
    // H is the causal IIR history. This solves the direct feed-through exactly;
    // using the previous sample's velocity for r makes the nonlinear element
    // causal and cannot inject energy because r is always non-negative.
    const auto areaM2 = std::numbers::pi * radiusM_ * radiusM_;
    const auto previousParticleVelocity = areaM2 > 0.0
        ? previousVolumeVelocityM3PerS_ / areaM2 : 0.0;
    const auto normalisedResistance = std::clamp(
        nonlinearLossCoefficient_ * std::abs(previousParticleVelocity)
            / speedOfSoundMps_,
        0.0, 4.0);
    const auto seriesDenominator = 2.0
        + normalisedResistance * (1.0 - b0_);
    const auto radiationIncident = seriesDenominator > 1.0e-12
        ? (2.0 * input + normalisedResistance * history) / seriesDenominator
        : input;
    const auto radiationReflected = b0_ * radiationIncident + history;
    const auto reflected = input - radiationIncident + radiationReflected;
    input2_ = input1_;
    input1_ = radiationIncident;
    output2_ = output1_;
    output1_ = std::isfinite(radiationReflected) ? radiationReflected : 0.0;

    const auto volumeVelocity = characteristicImpedancePaSPerM3_ > 0.0
        ? (radiationIncident - output1_)
            / characteristicImpedancePaSPerM3_ : 0.0;
    const auto volumeAcceleration = (volumeVelocity
        - previousVolumeVelocityM3PerS_) * sampleRateHz_;
    previousVolumeVelocityM3PerS_ = volumeVelocity;

    result.reflectedPressurePa = std::isfinite(reflected) ? reflected : 0.0;
    result.outletVolumeVelocityM3PerS = volumeVelocity;
    result.farFieldPressurePa = densityKgPerM3_ * volumeAcceleration
        / (4.0 * std::numbers::pi * observerDistanceM_);
    if (!std::isfinite(result.farFieldPressurePa)) result.farFieldPressurePa = 0.0;
    return result;
}

std::complex<double> UnflangedPipeRadiation::reflectionCoefficient(
    double frequencyHz) const noexcept {
    if (!prepared_ || !std::isfinite(frequencyHz)
        || frequencyHz < 0.0 || frequencyHz > sampleRateHz_ * 0.5)
        return {};
    const auto angle = -2.0 * std::numbers::pi * frequencyHz / sampleRateHz_;
    const auto z1 = std::polar(1.0, angle);
    const auto z2 = z1 * z1;
    const auto numerator = b0_ + b1_ * z1 + b2_ * z2;
    const auto denominator = 1.0 + a1_ * z1 + a2_ * z2;
    return std::abs(denominator) > 1.0e-15 ? numerator / denominator
                                           : std::complex<double> {};
}

double UnflangedPipeRadiation::planeModeCutoffHz() const noexcept {
    if (!prepared_) return 0.0;
    return firstAxisymmetricModeRoot * speedOfSoundMps_
        / (2.0 * std::numbers::pi * radiusM_);
}

void UnflangedPipeRadiation::updateCoefficients() noexcept {
    const auto areaM2 = std::numbers::pi * radiusM_ * radiusM_;
    characteristicImpedancePaSPerM3_ = densityKgPerM3_ * speedOfSoundMps_ / areaM2;

    // Bilinear transform s = 2 fs (1-z^-1)/(1+z^-1).
    const auto tau = radiusM_ / speedOfSoundMps_;
    const auto scaledFrequency = 2.0 * sampleRateHz_ * tau;
    const auto denominator0 = 1.0 + padeD1 * scaledFrequency
        + padeD2 * scaledFrequency * scaledFrequency;
    const auto denominator1 = 2.0
        - 2.0 * padeD2 * scaledFrequency * scaledFrequency;
    const auto denominator2 = 1.0 - padeD1 * scaledFrequency
        + padeD2 * scaledFrequency * scaledFrequency;
    const auto numerator0 = -1.0 - padeN1 * scaledFrequency;
    const auto numerator1 = -2.0;
    const auto numerator2 = -1.0 + padeN1 * scaledFrequency;
    b0_ = numerator0 / denominator0;
    b1_ = numerator1 / denominator0;
    b2_ = numerator2 / denominator0;
    a1_ = denominator1 / denominator0;
    a2_ = denominator2 / denominator0;
}

} // namespace enginelab
