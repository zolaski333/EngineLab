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
    const ForcedInductionConfig& config, double observerDistanceM) noexcept
    : config_(config), configuredObserverDistanceM_(
        std::isfinite(observerDistanceM) && observerDistanceM >= 0.05
            ? observerDistanceM : 1.0) {
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
    if (!(observerDistanceM > 0.0))
        observerDistanceM = configuredObserverDistanceM_;
    if (!valid_ || !(sampleRateHz > 1'000.0) || !std::isfinite(sampleRateHz)
        || !(observerDistanceM > 0.0) || !std::isfinite(observerDistanceM))
        return false;
    sampleRateHz_ = sampleRateHz;
    observerDistanceM_ = observerDistanceM;
    telemetrySmoothingCoefficient_ = static_cast<float>(1.0 - std::exp(
        -1.0 / (0.005 * sampleRateHz_)));
    transientAttackCoefficient_ = static_cast<float>(1.0 - std::exp(
        -1.0 / (0.00075 * sampleRateHz_)));
    transientReleaseCoefficient_ = static_cast<float>(1.0 - std::exp(
        -1.0 / (0.012 * sampleRateHz_)));
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
    smoothedInput_ = {};
    noiseStates_ = {
        0x9e3779b9U,
        0x243f6a88U,
        0xb7e15162U,
        0x8aed2a6bU,
    };
    telemetryInitialised_ = false;
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
    // For unit-variance white noise, a one-pole y=a*x+(1-a)*y[-1] has variance
    // a/(2-a), and two poles driven by the same noise have covariance
    // a*b/(1-(1-a)(1-b)). Normalise their difference rather than pretending
    // the bandpass retains the input RMS. The legacy switch deliberately
    // reproduces that old under-levelled result for same-binary A/B.
    auto normalisation = std::sqrt(3.0);
    if (broadbandPowerNormalisationEnabled_) {
        const auto fastVariance = static_cast<double>(fast) / (2.0 - fast);
        const auto slowVariance = static_cast<double>(slow) / (2.0 - slow);
        const auto covariance = static_cast<double>(fast) * slow
            / (1.0 - (1.0 - fast) * (1.0 - slow));
        const auto differenceVariance = std::max(
            1.0e-12,
            fastVariance + slowVariance - 2.0 * covariance);
        // nextWhiteNoise() is uniform [-1,1], whose variance is 1/3.
        normalisation = std::sqrt(3.0 / differenceVariance);
    }
    return (state.lowFast - state.lowSlow)
        * static_cast<float>(normalisation);
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

ForcedInductionAcoustics::ExhaustFlowSplit
ForcedInductionAcoustics::partitionExhaustFlow(
    double totalKgPerSecond, float wastegateOpening) const noexcept {
    const auto total = std::max(0.0, totalKgPerSecond);
    const auto turbineArea = std::max(0.0, config_.turbineFlowAreaMm2);
    const auto wastegateArea = std::max(0.0, config_.wastegateFlowAreaMm2)
        * std::clamp(static_cast<double>(wastegateOpening), 0.0, 1.0);
    const auto effectiveArea = turbineArea + wastegateArea;
    if (!(effectiveArea > 0.0)) return { total, 0.0 };
    const auto wastegate = total * wastegateArea / effectiveArea;
    return { total - wastegate, wastegate };
}

float ForcedInductionAcoustics::nextWhiteNoise(std::size_t source) noexcept {
    auto& state = noiseStates_[std::min(source, noiseStates_.size() - 1U)];
    state ^= state << 13U;
    state ^= state >> 17U;
    state ^= state << 5U;
    return static_cast<float>(
        static_cast<double>(state) / 2'147'483'647.5 - 1.0);
}

void ForcedInductionAcoustics::smoothTelemetry(const Input& input) noexcept {
    auto target = input;
    target.shaftSpeedRpm = std::max(0.0F, target.shaftSpeedRpm);
    target.correctedAirFlowKgPerSecond = std::max(
        0.0F, target.correctedAirFlowKgPerSecond);
    target.pressureRatio = std::clamp(target.pressureRatio, 1.0F, 3.5F);
    target.compressorPowerWatts = std::max(0.0F, target.compressorPowerWatts);
    target.turbinePowerWatts = std::max(0.0F, target.turbinePowerWatts);
    target.exhaustMassFlowKgPerSecond = std::max(
        0.0F, target.exhaustMassFlowKgPerSecond);
    target.wastegateOpening = std::clamp(target.wastegateOpening, 0.0F, 1.0F);
    target.blowOffMassFlowKgPerSecond = std::max(
        0.0F, target.blowOffMassFlowKgPerSecond);
    target.densityKgPerM3 = std::clamp(target.densityKgPerM3, 0.1F, 5.0F);
    target.soundSpeedMps = std::clamp(target.soundSpeedMps, 250.0F, 800.0F);
    target.acousticTimeScale = std::clamp(target.acousticTimeScale, 0.0F, 4.0F);
    if (!telemetryInitialised_) {
        smoothedInput_ = target;
        telemetryInitialised_ = true;
        return;
    }
    const auto smooth = [&](float targetValue, float& state, float coefficient) {
        state += coefficient * (targetValue - state);
    };
    smooth(target.shaftSpeedRpm, smoothedInput_.shaftSpeedRpm,
        telemetrySmoothingCoefficient_);
    smooth(target.correctedAirFlowKgPerSecond,
        smoothedInput_.correctedAirFlowKgPerSecond,
        telemetrySmoothingCoefficient_);
    smooth(target.pressureRatio, smoothedInput_.pressureRatio,
        telemetrySmoothingCoefficient_);
    smooth(target.compressorPowerWatts, smoothedInput_.compressorPowerWatts,
        telemetrySmoothingCoefficient_);
    smooth(target.turbinePowerWatts, smoothedInput_.turbinePowerWatts,
        telemetrySmoothingCoefficient_);
    smooth(target.exhaustMassFlowKgPerSecond,
        smoothedInput_.exhaustMassFlowKgPerSecond,
        telemetrySmoothingCoefficient_);
    smooth(target.wastegateOpening, smoothedInput_.wastegateOpening,
        telemetrySmoothingCoefficient_);
    const auto blowOffCoefficient = target.blowOffMassFlowKgPerSecond
            > smoothedInput_.blowOffMassFlowKgPerSecond
        ? transientAttackCoefficient_ : transientReleaseCoefficient_;
    smooth(target.blowOffMassFlowKgPerSecond,
        smoothedInput_.blowOffMassFlowKgPerSecond, blowOffCoefficient);
    smooth(target.densityKgPerM3, smoothedInput_.densityKgPerM3,
        telemetrySmoothingCoefficient_);
    smooth(target.soundSpeedMps, smoothedInput_.soundSpeedMps,
        telemetrySmoothingCoefficient_);
    // Time scale is already ramped by the renderer and must reach zero
    // immediately when the simulation is paused.
    smoothedInput_.acousticTimeScale = target.acousticTimeScale;
}

float ForcedInductionAcoustics::process(const Input& input) noexcept {
    if (!valid_ || !std::isfinite(input.shaftSpeedRpm)
        || !std::isfinite(input.correctedAirFlowKgPerSecond)
        || !std::isfinite(input.pressureRatio)
        || !std::isfinite(input.compressorPowerWatts)
        || !std::isfinite(input.turbinePowerWatts)
        || !std::isfinite(input.exhaustMassFlowKgPerSecond)
        || !std::isfinite(input.wastegateOpening)
        || !std::isfinite(input.blowOffMassFlowKgPerSecond)
        || !std::isfinite(input.densityKgPerM3)
        || !std::isfinite(input.soundSpeedMps)
        || !std::isfinite(input.acousticTimeScale))
        return 0.0F;
    smoothTelemetry(input);
    const auto& source = smoothedInput_;
    const auto spectralScale = std::clamp(
        static_cast<double>(source.acousticTimeScale), 0.0, 4.0);
    if (!(spectralScale > 0.0)) return 0.0F;
    const auto shaftHz = std::max(0.0,
        static_cast<double>(source.shaftSpeedRpm)) / 60.0 * spectralScale;
    const auto advanceTone = [&](double frequencyHz, double& phase) {
        if (!(frequencyHz > 0.0)) return 0.0F;
        // Always advance the phase so the tone stays coherent across the audible
        // limit. Freezing it (the previous behaviour) meant a blade rate that
        // dipped back below the limit resumed from a stale phase -- a click. The
        // amplitude is faded smoothly to zero over the last octave-fraction below
        // the Nyquist guard rather than hard-muted, which itself stepped.
        phase += 2.0 * std::numbers::pi * frequencyHz / sampleRateHz_;
        if (phase >= 2.0 * std::numbers::pi)
            phase = std::fmod(phase, 2.0 * std::numbers::pi);
        const auto upperHz = sampleRateHz_ * 0.45;
        if (frequencyHz >= upperHz) return 0.0F;
        const auto fadeStartHz = sampleRateHz_ * 0.40;
        const auto taper = frequencyHz > fadeStartHz
            ? std::clamp((upperHz - frequencyHz) / (upperHz - fadeStartHz), 0.0, 1.0)
            : 1.0;
        return static_cast<float>(std::sin(phase) * taper);
    };
    const auto rho = std::clamp(static_cast<double>(source.densityKgPerM3), 0.1, 5.0);
    const auto c = std::clamp(static_cast<double>(source.soundSpeedMps), 250.0, 800.0);
    auto pressurePa = 0.0;

    if (config_.type == ForcedInductionType::supercharger) {
        // Positive-displacement blowers use lobe order. A mechanically driven
        // centrifugal supercharger instead uses its impeller blade order.
        const auto passingOrder = config_.superchargerLobeCount > 0
            ? config_.superchargerLobeCount : config_.compressorBladeCount;
        const auto passHz = shaftHz * static_cast<double>(
            passingOrder);
        const auto tonePowerW = std::max(0.0F, source.compressorPowerWatts)
            * config_.tonalAcousticEfficiency;
        pressurePa += pressurePeakFromPower(tonePowerW, rho, c)
            * advanceTone(passHz, compressorPhase_);
    } else {
        const auto compressorPassHz = shaftHz * static_cast<double>(
            config_.compressorBladeCount);
        const auto turbinePassHz = shaftHz * static_cast<double>(
            config_.turbineBladeCount);
        const auto compressorTonePowerW = std::max(0.0F,
            source.compressorPowerWatts) * config_.tonalAcousticEfficiency;
        const auto turbineTonePowerW = std::max(0.0F,
            source.turbinePowerWatts) * config_.tonalAcousticEfficiency;
        pressurePa += pressurePeakFromPower(compressorTonePowerW, rho, c)
            * advanceTone(compressorPassHz, compressorPhase_);
        pressurePa += pressurePeakFromPower(turbineTonePowerW, rho, c)
            * advanceTone(turbinePassHz, turbinePhase_);

        const auto compressorAreaM2 = circularAreaM2(
            config_.compressorInducerDiameterMm);
        const auto correctedFlow = std::max(0.0F,
            source.correctedAirFlowKgPerSecond);
        if (compressorAreaM2 > 0.0 && correctedFlow > 0.0) {
            const auto velocity = correctedFlow / (rho * compressorAreaM2);
            const auto diameterM = config_.compressorInducerDiameterMm * 0.001;
            const auto centreHz = 0.2 * velocity / diameterM * spectralScale;
            const auto pressureFromPower = pressurePeakFromPower(jetPower(
                correctedFlow, compressorAreaM2, rho, c), rho, c)
                * (broadbandPowerNormalisationEnabled_
                    ? 1.0 / std::sqrt(2.0) : 1.0);
            pressurePa += pressureFromPower
                * bandNoise(nextWhiteNoise(0), centreHz, compressorNoise_);
        }
        const auto exducerAreaM2 = circularAreaM2(
            config_.turbineExducerDiameterMm);
        const auto turbineAreaM2 = exducerAreaM2 > 0.0
            ? (config_.turbineFlowAreaMm2 > 0.0
                ? config_.turbineFlowAreaMm2 * 1.0e-6 : exducerAreaM2)
            : 0.0;
        const auto flowSplit = partitionExhaustFlow(
            source.exhaustMassFlowKgPerSecond, source.wastegateOpening);
        if (turbineAreaM2 > 0.0 && flowSplit.turbineKgPerSecond > 0.0) {
            const auto velocity = flowSplit.turbineKgPerSecond / (rho * turbineAreaM2);
            const auto diameterM = 2.0 * std::sqrt(
                turbineAreaM2 / std::numbers::pi);
            const auto centreHz = 0.2 * velocity / diameterM * spectralScale;
            const auto pressureFromPower = pressurePeakFromPower(jetPower(
                flowSplit.turbineKgPerSecond, turbineAreaM2, rho, c), rho, c)
                * (broadbandPowerNormalisationEnabled_
                    ? 1.0 / std::sqrt(2.0) : 1.0);
            pressurePa += pressureFromPower
                * bandNoise(nextWhiteNoise(1), centreHz, turbineNoise_);
        }
        const auto wastegateAreaM2 = std::max(0.0,
            config_.wastegateFlowAreaMm2) * 1.0e-6;
        if (wastegateAreaM2 > 0.0 && flowSplit.wastegateKgPerSecond > 0.0) {
            const auto velocity = flowSplit.wastegateKgPerSecond
                / (rho * wastegateAreaM2);
            const auto diameterM = 2.0 * std::sqrt(
                wastegateAreaM2 / std::numbers::pi);
            const auto centreHz = 0.2 * velocity / diameterM * spectralScale;
            const auto pressureFromPower = pressurePeakFromPower(jetPower(
                flowSplit.wastegateKgPerSecond, wastegateAreaM2, rho, c), rho, c)
                * (broadbandPowerNormalisationEnabled_
                    ? 1.0 / std::sqrt(2.0) : 1.0);
            pressurePa += pressureFromPower
                * bandNoise(nextWhiteNoise(2), centreHz, wastegateNoise_);
        }
        const auto blowOffAreaM2 = std::max(0.0,
            config_.blowOffValveFlowAreaMm2) * 1.0e-6;
        const auto blowOffFlow = std::max(0.0F,
            source.blowOffMassFlowKgPerSecond);
        if (blowOffAreaM2 > 0.0 && blowOffFlow > 0.0) {
            const auto velocity = blowOffFlow / (rho * blowOffAreaM2);
            const auto diameterM = 2.0 * std::sqrt(
                blowOffAreaM2 / std::numbers::pi);
            const auto centreHz = 0.2 * velocity / diameterM * spectralScale;
            const auto pressureFromPower = pressurePeakFromPower(jetPower(
                blowOffFlow, blowOffAreaM2, rho, c), rho, c)
                * (broadbandPowerNormalisationEnabled_
                    ? 1.0 / std::sqrt(2.0) : 1.0);
            pressurePa += pressureFromPower
                * bandNoise(nextWhiteNoise(3), centreHz, blowOffNoise_);
        }
    }
    if (!std::isfinite(pressurePa)) {
        reset();
        return 0.0F;
    }
    return static_cast<float>(pressurePa);
}

} // namespace enginelab
