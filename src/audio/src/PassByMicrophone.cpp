#include <enginelab/audio/PassByMicrophone.hpp>

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <numbers>

namespace enginelab {

bool PassByMicrophone::prepare(double sampleRateHz, const PassBySettings& settings) {
    const auto finitePositive = [](double value) { return std::isfinite(value) && value > 0.0; };
    if (!finitePositive(sampleRateHz) || !finitePositive(settings.laneDistanceM)
        || !finitePositive(settings.soundSpeedMps) || !finitePositive(settings.referenceDistanceM)
        || !finitePositive(settings.halfLengthM) || settings.halfLengthM <= 2.0 * fadeLengthM
        || !std::isfinite(settings.microphoneHeightM) || !std::isfinite(settings.sourceHeightM)
        || !std::isfinite(settings.microphoneSpacingM) || !std::isfinite(settings.groundReflection)
        || settings.microphoneHeightM < 0.0 || settings.sourceHeightM < 0.0
        || settings.groundReflection < 0.0 || settings.groundReflection > 1.0
        || !std::isfinite(settings.exhaustBehindM) || std::abs(settings.exhaustBehindM) > fadeLengthM)
        return false;
    settings_ = settings;
    sampleRateHz_ = sampleRateHz;
    std::size_t size = 1;
    while (size < static_cast<std::size_t>(delaySeconds * sampleRateHz) + 8) size <<= 1U;
    delay_.assign(size, 0.0F);
    exhaustDelay_.assign(size, 0.0F);
    mask_ = size - 1;
    // The pedal and the gearbox move the car's speed at the frame rate:
    // 20 ms smooths their steps out of the Doppler shift.
    speedCoefficient_ = 1.0 - std::exp(-1.0 / (0.020 * sampleRateHz));
    mixStep_ = static_cast<float>(1.0 / (crossFadeSeconds * sampleRateHz));
    reset();
    return true;
}

void PassByMicrophone::reset(double positionM) noexcept {
    std::fill(delay_.begin(), delay_.end(), 0.0F);
    std::fill(exhaustDelay_.begin(), exhaustDelay_.end(), 0.0F);
    write_ = 0;
    paths_ = {};
    speedMps_ = targetSpeedMps_;
    setPosition(positionM);
    mix_ = enabled_ ? 1.0F : 0.0F;
    warmup_ = 0;
}

void PassByMicrophone::setEnabled(bool enabled) noexcept {
    if (enabled && !active()) {
        // Nothing heard since it was last on: start clean, in front.
        std::fill(delay_.begin(), delay_.end(), 0.0F);
        std::fill(exhaustDelay_.begin(), exhaustDelay_.end(), 0.0F);
        paths_ = {};
        speedMps_ = targetSpeedMps_;
        positionM_ = 0.0;
        warmup_ = longestStartDelay();
    }
    enabled_ = enabled && prepared();
}

void PassByMicrophone::setVehicleSpeed(double metresPerSecond) noexcept {
    if (!std::isfinite(metresPerSecond)) return;
    const auto limit = 0.8 * settings_.soundSpeedMps;
    targetSpeedMps_ = std::clamp(metresPerSecond, -limit, limit);
}

void PassByMicrophone::setReferenceDistance(double metres) noexcept {
    if (std::isfinite(metres) && metres > 0.0) settings_.referenceDistanceM = metres;
}

void PassByMicrophone::setPosition(double positionM) noexcept {
    if (!std::isfinite(positionM)) return;
    const auto length = 2.0 * settings_.halfLengthM;
    positionM_ = positionM - length * std::floor((positionM + settings_.halfLengthM) / length);
}

double PassByMicrophone::absorptionCutoffHz(double distanceM) noexcept {
    return 58'000.0 / std::sqrt(std::max(distanceM, 1.0e-3));
}

void PassByMicrophone::retarded(double alongM, double heightDifferenceM, double offsetM,
                                double& delaySamples, double& distanceM) const noexcept {
    // Emitted tau ago at x - v tau, heard now: c tau = |(x - v tau, D)|,
    // (c^2 - v^2) tau^2 + 2 x v tau - (x^2 + D^2) = 0, the positive root.
    const auto c = settings_.soundSpeedMps;
    const auto v = speedMps_;
    const auto x = alongM - offsetM;
    const auto lateral2 = settings_.laneDistanceM * settings_.laneDistanceM
        + heightDifferenceM * heightDifferenceM;
    const auto a = c * c - v * v;
    const auto tau = (-x * v + std::sqrt(x * x * v * v + a * (x * x + lateral2))) / a;
    distanceM = c * tau;
    delaySamples = tau * sampleRateHz_;
}

std::size_t PassByMicrophone::longestStartDelay() const noexcept {
    double longest = 0.0;
    for (const auto along : { 0.0, -settings_.exhaustBehindM })
        for (const auto offset : { -0.5 * settings_.microphoneSpacingM, 0.5 * settings_.microphoneSpacingM }) {
            double delaySamples {};
            double distanceM {};
            retarded(along, settings_.microphoneHeightM + settings_.sourceHeightM, offset, delaySamples, distanceM);
            longest = std::max(longest, delaySamples);
        }
    return static_cast<std::size_t>(std::ceil(longest)) + 4;
}

float PassByMicrophone::read(const std::vector<float>& line, double delaySamples) const noexcept {
    // Four-point Hermite between the samples around the delay.
    const auto position = static_cast<double>(write_) - delaySamples;
    const auto floored = std::floor(position);
    const auto t = static_cast<float>(position - floored);
    const auto base = static_cast<std::size_t>(static_cast<long long>(floored)) & mask_;
    const auto ym1 = line[(base - 1) & mask_];
    const auto y0 = line[base];
    const auto y1 = line[(base + 1) & mask_];
    const auto y2 = line[(base + 2) & mask_];
    const auto c1 = 0.5F * (y1 - ym1);
    const auto c2 = ym1 - 2.5F * y0 + 2.0F * y1 - 0.5F * y2;
    const auto c3 = 0.5F * (y2 - ym1) + 1.5F * (y0 - y1);
    return ((c3 * t + c2) * t + c1) * t + y0;
}

float PassByMicrophone::window(double alongM) const noexcept {
    const auto fromEnd = settings_.halfLengthM - std::abs(alongM);
    if (fromEnd >= fadeLengthM) return 1.0F;
    const auto s = std::sin(0.5 * std::numbers::pi * std::max(fromEnd, 0.0) / fadeLengthM);
    return static_cast<float>(s * s);
}

StereoPressure PassByMicrophone::process(float left, float right, float exhaustLeft,
                                         float exhaustRight) noexcept {
    if (!active()) return { left, right };
    delay_[write_] = 0.5F * (left - exhaustLeft + right - exhaustRight);
    exhaustDelay_[write_] = 0.5F * (exhaustLeft + exhaustRight);

    speedMps_ += speedCoefficient_ * (targetSpeedMps_ - speedMps_);
    const auto fade = window(positionM_);
    std::array<float, 2> heard {};
    const auto direct = settings_.microphoneHeightM - settings_.sourceHeightM;
    const auto image = settings_.microphoneHeightM + settings_.sourceHeightM;
    const auto halfSpacing = 0.5 * settings_.microphoneSpacingM;
    for (std::size_t path = 0; path < paths_.size(); ++path) {
        const auto microphone = path / 4;
        const auto exhaust = (path / 2) % 2 == 1;
        const auto reflected = path % 2;
        const auto offset = microphone == 0 ? -halfSpacing : halfSpacing;
        {
            const auto gain = reflected == 0 ? 1.0 : settings_.groundReflection;
            if (gain <= 0.0) continue;
            double delaySamples {};
            double distanceM {};
            retarded(exhaust ? positionM_ - settings_.exhaustBehindM : positionM_,
                     reflected == 0 ? direct : image, offset, delaySamples, distanceM);
            auto sample = read(exhaust ? exhaustDelay_ : delay_, delaySamples)
                * static_cast<float>(gain * settings_.referenceDistanceM / distanceM);
            auto& state = paths_[path];
            if (settings_.airAbsorption) {
                const auto cutoff = std::min(absorptionCutoffHz(distanceM), 0.45 * sampleRateHz_);
                const auto coefficient = static_cast<float>(
                    1.0 - std::exp(-2.0 * std::numbers::pi * cutoff / sampleRateHz_));
                state.first += coefficient * (sample - state.first);
                state.second += coefficient * (state.first - state.second);
                sample = state.second;
            }
            heard[microphone] += sample;
        }
    }
    write_ = (write_ + 1) & mask_;
    setPosition(positionM_ + speedMps_ / sampleRateHz_);

    const auto target = enabled_ ? 1.0F : 0.0F;
    if (warmup_ != 0 && enabled_)
        --warmup_;
    else
        mix_ = mix_ < target ? std::min(target, mix_ + mixStep_) : std::max(target, mix_ - mixStep_);
    return { left + mix_ * (fade * heard[0] - left), right + mix_ * (fade * heard[1] - right) };
}

} // namespace enginelab
