#include <enginelab/audio/ForcedInductionAcoustics.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace enginelab {
namespace {

[[nodiscard]] double circularAreaM2(double diameterMm) noexcept {
    if (!(diameterMm > 0.0)) return 0.0;
    const auto radiusM = diameterMm * 0.0005;
    return std::numbers::pi * radiusM * radiusM;
}

} // namespace

ForcedInductionAcoustics::ForcedInductionAcoustics(
    const ForcedInductionConfig& config) noexcept
    : config_(config) {
    valid_ = config_.enabled && (
        (config_.type == ForcedInductionType::turbocharger
            && (config_.compressorBladeCount > 0
                || config_.turbineBladeCount > 0
                || config_.compressorInducerDiameterMm > 0.0
                || config_.wastegateFlowAreaMm2 > 0.0
                || config_.blowOffValveFlowAreaMm2 > 0.0))
        || (config_.type == ForcedInductionType::supercharger
            && (config_.superchargerLobeCount > 0
                || config_.compressorBladeCount > 0)));
}

bool ForcedInductionAcoustics::prepare(double sampleRateHz,
                                       double observerDistanceM) noexcept {
    if (!valid_ || !(sampleRateHz > 1'000.0) || !std::isfinite(sampleRateHz)
        || !(observerDistanceM > 0.0) || !std::isfinite(observerDistanceM))
        return false;
    sampleRateHz_ = sampleRateHz;
    observerDistanceM_ = observerDistanceM;
    reset();
    return true;
}

void ForcedInductionAcoustics::reset() noexcept {
    compressorPhase_ = 0.0;
    turbinePhase_ = 0.0;
    compressorNoise_ = {};
    turbineNoise_ = {};
    wastegateNoise_ = {};
    blowOffNoise_ = {};
}

float ForcedInductionAcoustics::bandNoise(
    float noise, double centreHz, BandNoiseState& state) const noexcept {
    const auto lowCutHz = std::clamp(centreHz * 0.45, 20.0, sampleRateHz_ * 0.40);
    const auto highCutHz = std::clamp(centreHz * 2.2, lowCutHz * 1.1,
                                      sampleRateHz_ * 0.45);
    const auto slow = static_cast<float>(1.0 - std::exp(
        -2.0 * std::numbers::pi * lowCutHz / sampleRateHz_));
    const auto fast = static_cast<float>(1.0 - std::exp(
        -2.0 * std::numbers::pi * highCutHz / sampleRateHz_));
    state.lowSlow += slow * (noise - state.lowSlow);
    state.lowFast += fast * (noise - state.lowFast);
    // Uniform [-1,1] has RMS 1/sqrt(3). The factor makes the unfiltered source
    // unit-RMS before the two physical corner filters.
    return (state.lowFast - state.lowSlow)
        * static_cast<float>(std::sqrt(3.0));
}

double ForcedInductionAcoustics::pressurePeakFromPower(
    double powerWatts, double densityKgPerM3,
    double soundSpeedMps) const noexcept {
    if (!(powerWatts > 0.0) || !(densityKgPerM3 > 0.0)
        || !(soundSpeedMps > 0.0))
        return 0.0;
    // Spherical spreading: W = 4*pi*r^2*p_rms^2/(rho*c).
    return std::sqrt(2.0 * powerWatts * densityKgPerM3 * soundSpeedMps
        / (4.0 * std::numbers::pi
            * observerDistanceM_ * observerDistanceM_));
}

double ForcedInductionAcoustics::jetPower(
    double massFlowKgPerSecond, double areaM2,
    double densityKgPerM3, double soundSpeedMps) const noexcept {
    if (!(massFlowKgPerSecond > 0.0) || !(areaM2 > 0.0)
        || !(densityKgPerM3 > 0.0) || !(soundSpeedMps > 0.0))
        return 0.0;
    const auto velocityMps = massFlowKgPerSecond / (densityKgPerM3 * areaM2);
    const auto limitedVelocityMps = std::min(velocityMps, soundSpeedMps);
    return config_.turbulentJetNoiseCoefficient * densityKgPerM3 * areaM2
        * std::pow(limitedVelocityMps, 8.0) / std::pow(soundSpeedMps, 5.0);
}

float ForcedInductionAcoustics::process(
    const Input& input, float whiteNoise) noexcept {
    if (!valid_ || !std::isfinite(input.shaftSpeedRpm)
        || !std::isfinite(whiteNoise))
        return 0.0F;
    const auto spectralScale = std::clamp(
        static_cast<double>(input.acousticTimeScale), 0.0, 4.0);
    if (!(spectralScale > 0.0)) return 0.0F;
    const auto shaftHz = std::max(0.0,
        static_cast<double>(input.shaftSpeedRpm)) / 60.0 * spectralScale;
    const auto advanceTone = [&](double frequencyHz, double& phase) {
        if (!(frequencyHz > 0.0) || frequencyHz >= sampleRateHz_ * 0.45)
            return 0.0F;
        phase += 2.0 * std::numbers::pi * frequencyHz / sampleRateHz_;
        if (phase >= 2.0 * std::numbers::pi)
            phase = std::fmod(phase, 2.0 * std::numbers::pi);
        return static_cast<float>(std::sin(phase));
    };
    const auto rho = std::clamp(static_cast<double>(input.densityKgPerM3), 0.1, 5.0);
    const auto c = std::clamp(static_cast<double>(input.soundSpeedMps), 250.0, 800.0);
    auto pressurePa = 0.0;

    if (config_.type == ForcedInductionType::supercharger) {
        // Positive-displacement blowers use lobe order. A mechanically driven
        // centrifugal supercharger instead uses its impeller blade order.
        const auto passingOrder = config_.superchargerLobeCount > 0
            ? config_.superchargerLobeCount : config_.compressorBladeCount;
        const auto passHz = shaftHz * static_cast<double>(
            passingOrder);
        const auto tonePowerW = std::max(0.0F, input.compressorPowerWatts)
            * config_.tonalAcousticEfficiency;
        pressurePa += pressurePeakFromPower(tonePowerW, rho, c)
            * advanceTone(passHz, compressorPhase_);
    } else {
        const auto compressorPassHz = shaftHz * static_cast<double>(
            config_.compressorBladeCount);
        const auto turbinePassHz = shaftHz * static_cast<double>(
            config_.turbineBladeCount);
        const auto compressorTonePowerW = std::max(0.0F,
            input.compressorPowerWatts) * config_.tonalAcousticEfficiency;
        const auto turbineTonePowerW = std::max(0.0F,
            input.turbinePowerWatts) * config_.tonalAcousticEfficiency;
        pressurePa += pressurePeakFromPower(compressorTonePowerW, rho, c)
            * advanceTone(compressorPassHz, compressorPhase_);
        pressurePa += pressurePeakFromPower(turbineTonePowerW, rho, c)
            * advanceTone(turbinePassHz, turbinePhase_);

        const auto compressorAreaM2 = circularAreaM2(
            config_.compressorInducerDiameterMm);
        const auto correctedFlow = std::max(0.0F,
            input.correctedAirFlowKgPerSecond);
        if (compressorAreaM2 > 0.0 && correctedFlow > 0.0) {
            const auto velocity = correctedFlow / (rho * compressorAreaM2);
            const auto diameterM = config_.compressorInducerDiameterMm * 0.001;
            const auto centreHz = 0.2 * velocity / diameterM * spectralScale;
            pressurePa += pressurePeakFromPower(jetPower(
                correctedFlow, compressorAreaM2, rho, c), rho, c)
                * bandNoise(whiteNoise, centreHz, compressorNoise_);
        }
        const auto turbineAreaM2 = circularAreaM2(
            config_.turbineExducerDiameterMm);
        const auto exhaustFlow = std::max(0.0F,
            input.exhaustMassFlowKgPerSecond);
        if (turbineAreaM2 > 0.0 && exhaustFlow > 0.0) {
            const auto velocity = exhaustFlow / (rho * turbineAreaM2);
            const auto diameterM = config_.turbineExducerDiameterMm * 0.001;
            const auto centreHz = 0.2 * velocity / diameterM * spectralScale;
            pressurePa += pressurePeakFromPower(jetPower(
                exhaustFlow, turbineAreaM2, rho, c), rho, c)
                * bandNoise(-whiteNoise, centreHz, turbineNoise_);
        }
        const auto wastegateAreaM2 = std::max(0.0,
            config_.wastegateFlowAreaMm2) * 1.0e-6;
        const auto wastegateFlow = std::max(0.0F,
            input.exhaustMassFlowKgPerSecond)
            * std::clamp(input.wastegateOpening, 0.0F, 1.0F);
        if (wastegateAreaM2 > 0.0 && wastegateFlow > 0.0) {
            const auto velocity = wastegateFlow / (rho * wastegateAreaM2);
            const auto diameterM = 2.0 * std::sqrt(
                wastegateAreaM2 / std::numbers::pi);
            const auto centreHz = 0.2 * velocity / diameterM * spectralScale;
            pressurePa += pressurePeakFromPower(jetPower(
                wastegateFlow, wastegateAreaM2, rho, c), rho, c)
                * bandNoise(whiteNoise, centreHz, wastegateNoise_);
        }
        const auto blowOffAreaM2 = std::max(0.0,
            config_.blowOffValveFlowAreaMm2) * 1.0e-6;
        const auto blowOffFlow = std::max(0.0F,
            input.blowOffMassFlowKgPerSecond);
        if (blowOffAreaM2 > 0.0 && blowOffFlow > 0.0) {
            const auto velocity = blowOffFlow / (rho * blowOffAreaM2);
            const auto diameterM = 2.0 * std::sqrt(
                blowOffAreaM2 / std::numbers::pi);
            const auto centreHz = 0.2 * velocity / diameterM * spectralScale;
            pressurePa += pressurePeakFromPower(jetPower(
                blowOffFlow, blowOffAreaM2, rho, c), rho, c)
                * bandNoise(-whiteNoise, centreHz, blowOffNoise_);
        }
    }
    if (!std::isfinite(pressurePa)) {
        reset();
        return 0.0F;
    }
    return static_cast<float>(pressurePa);
}

} // namespace enginelab
