#include <enginelab/audio/MufflerFlowNoise.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace enginelab {

double MufflerFlowNoise::expansionChamberEfficiencyDb(
    double lengthOverDiameter) noexcept {
    const auto ratio = std::isfinite(lengthOverDiameter)
        ? std::clamp(lengthOverDiameter, 0.0, 6.0) : 0.0;
    return 65.0 + 16.5 * (1.0 - std::exp(-ratio / 0.75));
}

double MufflerFlowNoise::perforateEfficiencyDb(
    double porosityPercent, double lengthOverDiameter) noexcept {
    const auto sigma = std::isfinite(porosityPercent)
        ? std::clamp(porosityPercent, 2.0, 27.0) : 2.0;
    const auto ratio = std::isfinite(lengthOverDiameter)
        ? std::clamp(lengthOverDiameter, 0.0, 6.0) : 0.0;
    const auto base = sigma <= 5.9 ? 65.8 + 2.47 * (sigma - 4.2)
        : sigma <= 20.0 ? 70.0 + 0.34 * (sigma - 5.9)
        : 74.8 + 0.45 * (sigma - 20.0);
    return base + 4.0 * ratio;
}

double MufflerFlowNoise::soundPowerWatts(double efficiencyDb,
    double massFlowKgPerSecond, double ductDiameterM,
    double densityKgPerM3, double soundSpeedMps) noexcept {
    const auto positive = [](double value) {
        return std::isfinite(value) && value > 0.0;
    };
    if (!std::isfinite(efficiencyDb) || !positive(massFlowKgPerSecond)
        || !positive(ductDiameterM) || !positive(densityKgPerM3)
        || !positive(soundSpeedMps))
        return 0.0;
    const auto areaM2 = 0.25 * std::numbers::pi * ductDiameterM * ductDiameterM;
    // The correlation is subsonic; hold the duct to M 0.9 like the jet.
    const auto velocity = std::min(massFlowKgPerSecond
        / (densityKgPerM3 * areaM2), 0.9 * soundSpeedMps);
    const auto mach = velocity / soundSpeedMps;
    const auto efficiency = std::pow(10.0, (efficiencyDb - 120.0) / 10.0);
    const auto power = efficiency * 0.5 * densityKgPerM3 * velocity * velocity
        * velocity * areaM2 * mach * std::sqrt(mach);
    return std::isfinite(power) ? std::max(0.0, power) : 0.0;
}

MufflerFlowNoise::MufflerFlowNoise(std::uint32_t seed) noexcept {
    setSeed(seed);
}

void MufflerFlowNoise::setSeed(std::uint32_t seed) noexcept {
    initialSeed_ = seed != 0U ? seed : 0x2545f491U;
    noiseState_ = initialSeed_;
}

float MufflerFlowNoise::nextUnitWhiteNoise(std::uint32_t& state) const noexcept {
    state ^= state << 13U;
    state ^= state >> 17U;
    state ^= state << 5U;
    return static_cast<float>(
        static_cast<double>(state) / 2'147'483'647.5 - 1.0);
}

float MufflerFlowNoise::filter(float white, FilterState& s) const noexcept {
    // Paul Kellet's refined pink filter: -3 dB per octave within 0.05 dB
    // above ~10 Hz at 44.1 kHz. Its absolute gain is measured in prepare().
    s.b0 = 0.99886F * s.b0 + white * 0.0555179F;
    s.b1 = 0.99332F * s.b1 + white * 0.0750759F;
    s.b2 = 0.96900F * s.b2 + white * 0.1538520F;
    s.b3 = 0.86650F * s.b3 + white * 0.3104856F;
    s.b4 = 0.55000F * s.b4 + white * 0.5329522F;
    s.b5 = -0.7616F * s.b5 - white * 0.0168980F;
    const auto pink = s.b0 + s.b1 + s.b2 + s.b3 + s.b4 + s.b5 + s.b6
        + white * 0.5362F;
    s.b6 = white * 0.115926F;
    // One-pole high-pass and low-pass at the band edges.
    const auto highPassed = highPassCoefficient_
        * (s.highPassOutput + pink - s.highPassInput);
    s.highPassInput = pink;
    s.highPassOutput = highPassed;
    s.lowPassOutput += lowPassCoefficient_ * (highPassed - s.lowPassOutput);
    return s.lowPassOutput;
}

bool MufflerFlowNoise::prepare(double sampleRateHz) noexcept {
    prepared_ = std::isfinite(sampleRateHz) && sampleRateHz > 1'000.0;
    if (!prepared_) return false;
    sampleRateHz_ = sampleRateHz;
    const auto highEdge = std::min(highBandEdgeHz, 0.45 * sampleRateHz);
    const auto rc = 1.0 / (2.0 * std::numbers::pi * lowBandEdgeHz);
    const auto dt = 1.0 / sampleRateHz;
    highPassCoefficient_ = static_cast<float>(rc / (rc + dt));
    lowPassCoefficient_ = static_cast<float>(1.0 - std::exp(
        -2.0 * std::numbers::pi * highEdge / sampleRateHz));
    // The band's RMS for unit white noise, measured once on a private
    // sequence after the filter has settled: deterministic per sample rate.
    FilterState scratch {};
    auto scratchSeed = std::uint32_t { 0x6a09e667U };
    const auto settle = static_cast<std::size_t>(sampleRateHz * 0.25);
    const auto measure = static_cast<std::size_t>(sampleRateHz * 2.0);
    auto energy = 0.0;
    for (std::size_t sample = 0; sample < settle + measure; ++sample) {
        const auto value = filter(nextUnitWhiteNoise(scratchSeed), scratch);
        if (sample >= settle) energy += static_cast<double>(value) * value;
    }
    const auto rms = std::sqrt(energy / static_cast<double>(measure));
    unitRmsNormalisation_ = rms > 0.0 ? static_cast<float>(1.0 / rms) : 0.0F;
    setTargetPressureRmsPa(0.0);
    snapToTarget();
    reset();
    return true;
}

void MufflerFlowNoise::reset() noexcept {
    state_ = {};
    noiseState_ = initialSeed_;
}

void MufflerFlowNoise::setTargetPressureRmsPa(double pressureRmsPa) noexcept {
    targetPressureRmsPa_ = std::isfinite(pressureRmsPa)
        ? std::clamp(pressureRmsPa, 0.0, 100'000.0) : 0.0;
    pressureRmsTargetPa_ = static_cast<float>(targetPressureRmsPa_);
}

void MufflerFlowNoise::snapToTarget() noexcept {
    pressureRmsPa_ = pressureRmsTargetPa_;
}

float MufflerFlowNoise::process(float rampCoefficient) noexcept {
    if (!prepared_) return 0.0F;
    const auto ramp = std::clamp(rampCoefficient, 0.0F, 1.0F);
    pressureRmsPa_ += ramp * (pressureRmsTargetPa_ - pressureRmsPa_);
    if (!(pressureRmsPa_ > 0.0F) && !(pressureRmsTargetPa_ > 0.0F)) return 0.0F;
    const auto value = filter(nextUnitWhiteNoise(noiseState_), state_)
        * unitRmsNormalisation_ * pressureRmsPa_;
    return std::isfinite(value) ? value : 0.0F;
}

} // namespace enginelab
