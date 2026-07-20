#include <enginelab/audio/PipeRadiationModel.hpp>

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
    const auto reflected = b0_ * input + b1_ * input1_ + b2_ * input2_
        - a1_ * output1_ - a2_ * output2_;
    input2_ = input1_;
    input1_ = input;
    output2_ = output1_;
    output1_ = std::isfinite(reflected) ? reflected : 0.0;

    const auto volumeVelocity = characteristicImpedancePaSPerM3_ > 0.0
        ? (input - output1_) / characteristicImpedancePaSPerM3_ : 0.0;
    const auto volumeAcceleration = (volumeVelocity
        - previousVolumeVelocityM3PerS_) * sampleRateHz_;
    previousVolumeVelocityM3PerS_ = volumeVelocity;

    result.reflectedPressurePa = output1_;
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
