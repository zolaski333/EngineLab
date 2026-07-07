#include <enginelab/audio/RealtimeEngineAudio.hpp>
#include <algorithm>
#include <cmath>
#include <numbers>

namespace enginelab {
void RealtimeEngineAudio::prepare(double sampleRate, int) noexcept {
    sampleRate_ = sampleRate > 0.0 ? sampleRate : 48'000.0;
    const auto reflectionSeconds = std::clamp(realtimeState_.exhaustReflectionSeconds.load(std::memory_order_relaxed),
                                               0.001F, 0.080F);
    reflectionDelaySamples_ = std::clamp(static_cast<std::size_t>(sampleRate_ * reflectionSeconds),
                                         std::size_t { 1 }, exhaustDelay_.size() - 1);
    lowPassCoefficient_ = static_cast<float>(1.0 - std::exp(-2.0 * std::numbers::pi * 1'500.0 / sampleRate_));
    release();
}
void RealtimeEngineAudio::release() noexcept {
    voices_.fill({}); pendingEvents_.fill({}); pendingEventCount_ = 0; audioTimeSeconds_ = 0.0;
    exhaustDelay_.fill(0.0F); delayWrite_ = 0; lowPassLeft_ = 0.0F; lowPassRight_ = 0.0F;
    mechanicalPhase_ = 0.0; valvetrainPhase_ = 0.0; starterPhase_ = 0.0; smoothedRpm_ = 0.0F; intakeFilter_ = 0.0F;
}

void RealtimeEngineAudio::render(juce::AudioBuffer<float>& output, int startSample, int sampleCount) noexcept {
    output.clear(startSample, sampleCount);
    FiringEvent event;
    std::size_t drained = 0;
    while (drained++ < 512 && queue_.tryPop(event)) {
        if (std::abs(event.timeSeconds - audioTimeSeconds_) > 0.25 && pendingEventCount_ == 0)
            audioTimeSeconds_ = event.timeSeconds;
        const auto hasExhaustPulse = event.exhaustDelaySeconds > 0.00005F;
        const auto required = hasExhaustPulse ? std::size_t { 2 } : std::size_t { 1 };
        if (pendingEventCount_ + required > pendingEvents_.size()) {
            droppedPendingEvents_.fetch_add(required, std::memory_order_relaxed);
            continue;
        }
        auto& direct = pendingEvents_[pendingEventCount_++];
        direct = { event, event.timeSeconds + eventLatencySeconds_, false };
        if (direct.scheduledTimeSeconds + 1.0 / sampleRate_ < audioTimeSeconds_)
            lateEvents_.fetch_add(1, std::memory_order_relaxed);
        if (hasExhaustPulse) {
            auto& exhaust = pendingEvents_[pendingEventCount_++];
            exhaust = { event, event.timeSeconds + eventLatencySeconds_ + event.exhaustDelaySeconds, true };
            if (exhaust.scheduledTimeSeconds + 1.0 / sampleRate_ < audioTimeSeconds_)
                lateEvents_.fetch_add(1, std::memory_order_relaxed);
        }
    }
    const auto targetRpm = realtimeState_.rpm.load(std::memory_order_relaxed);
    const auto throttle = realtimeState_.throttle.load(std::memory_order_relaxed);
    const auto load = realtimeState_.load.load(std::memory_order_relaxed);
    const auto stress = realtimeState_.mechanicalStress.load(std::memory_order_relaxed);
    const auto starter = realtimeState_.starter.load(std::memory_order_relaxed);
    const auto timeScale = realtimeState_.timeScale.load(std::memory_order_relaxed);
    for (int sample = 0; sample < sampleCount; ++sample) {
        for (std::size_t index = 0; index < pendingEventCount_;) {
            if (pendingEvents_[index].scheduledTimeSeconds <= audioTimeSeconds_ + 0.5 / sampleRate_) {
                trigger(pendingEvents_[index].event, pendingEvents_[index].exhaust);
                pendingEvents_[index] = pendingEvents_[--pendingEventCount_];
            } else {
                ++index;
            }
        }
        float left = 0.0F;
        float right = 0.0F;
        const auto audibleRpm = targetRpm * timeScale;
        smoothedRpm_ += static_cast<float>(1.0 - std::exp(-18.0 / sampleRate_)) * (audibleRpm - smoothedRpm_);
        const auto rotationHz = std::max(0.0F, smoothedRpm_) / 60.0F;
        mechanicalPhase_ += 2.0 * std::numbers::pi * static_cast<double>(rotationHz) / sampleRate_;
        valvetrainPhase_ += 2.0 * std::numbers::pi * static_cast<double>(rotationHz * 2.0F) / sampleRate_;
        starterPhase_ += 2.0 * std::numbers::pi * 92.0 / sampleRate_;
        if (mechanicalPhase_ >= 2.0 * std::numbers::pi) mechanicalPhase_ -= 2.0 * std::numbers::pi;
        if (valvetrainPhase_ >= 2.0 * std::numbers::pi) valvetrainPhase_ -= 2.0 * std::numbers::pi;
        if (starterPhase_ >= 2.0 * std::numbers::pi) starterPhase_ -= 2.0 * std::numbers::pi;
        const auto speedGain = std::clamp(smoothedRpm_ / 7'000.0F, 0.0F, 1.0F);
        const auto mechanical = (static_cast<float>(std::sin(mechanicalPhase_)) * 0.018F
            + static_cast<float>(std::sin(valvetrainPhase_ * 3.03)) * (0.007F + stress * 0.012F)) * speedGain;
        intakeFilter_ += 0.08F * (noise() * throttle * speedGain - intakeFilter_);
        const auto intake = intakeFilter_ * (0.025F + load * 0.035F);
        const auto starterSound = starter * (static_cast<float>(std::sin(starterPhase_)) * 0.028F + noise() * 0.006F);
        left += mechanical + intake * 0.92F + starterSound;
        right += mechanical * 0.96F + intake + starterSound * 0.94F;
        for (auto& voice : voices_) {
            if (!voice.active) continue;
            const auto body = static_cast<float>(std::sin(voice.bodyPhase));
            const auto harmonic = static_cast<float>(std::sin(voice.bodyPhase * 2.01)) * 0.32F
                                + static_cast<float>(std::sin(voice.bodyPhase * 3.97)) * 0.13F;
            const auto crack = static_cast<float>(std::sin(voice.crackPhase)) * 0.18F + noise() * 0.12F;
            const auto raw = (body * 0.62F + harmonic + crack) * voice.amplitude;
            voice.filterState += 0.24F * (raw - voice.filterState);
            const auto leftGain = std::sqrt((1.0F - voice.pan) * 0.5F);
            const auto rightGain = std::sqrt((1.0F + voice.pan) * 0.5F);
            left += voice.filterState * leftGain;
            right += voice.filterState * rightGain;
            voice.bodyPhase += 2.0 * std::numbers::pi * static_cast<double>(voice.bodyFrequency) / sampleRate_;
            voice.crackPhase += 2.0 * std::numbers::pi * static_cast<double>(voice.crackFrequency) / sampleRate_;
            if (voice.bodyPhase >= 2.0 * std::numbers::pi) voice.bodyPhase -= 2.0 * std::numbers::pi;
            if (voice.crackPhase >= 2.0 * std::numbers::pi) voice.crackPhase -= 2.0 * std::numbers::pi;
            voice.amplitude *= voice.decay;
            if (voice.amplitude < 0.0001F) voice.active = false;
        }
        const auto reflectionIndex = (delayWrite_ + exhaustDelay_.size() - reflectionDelaySamples_) % exhaustDelay_.size();
        const auto reflected = exhaustDelay_[reflectionIndex] * 0.24F;
        exhaustDelay_[delayWrite_] = (left + right) * 0.5F + reflected * 0.18F;
        delayWrite_ = (delayWrite_ + 1) % exhaustDelay_.size();
        left += reflected; right += reflected * 0.82F;
        lowPassLeft_ += lowPassCoefficient_ * (left - lowPassLeft_);
        lowPassRight_ += lowPassCoefficient_ * (right - lowPassRight_);
        const auto outLeft = std::tanh(lowPassLeft_ * 1.65F) * 0.42F;
        const auto outRight = std::tanh(lowPassRight_ * 1.65F) * 0.42F;
        if (output.getNumChannels() > 0) output.setSample(0, startSample + sample, outLeft);
        if (output.getNumChannels() > 1) output.setSample(1, startSample + sample, outRight);
        for (int channel = 2; channel < output.getNumChannels(); ++channel)
            output.setSample(channel, startSample + sample, (outLeft + outRight) * 0.5F);
        audioTimeSeconds_ += 1.0 / sampleRate_;
    }
}

void RealtimeEngineAudio::trigger(const FiringEvent& event, bool exhaust) noexcept {
    auto voice = std::find_if(voices_.begin(), voices_.end(), [](const Voice& item) { return !item.active; });
    if (voice == voices_.end()) {
        voice = std::min_element(voices_.begin(), voices_.end(),
            [](const Voice& left, const Voice& right) { return left.amplitude < right.amplitude; });
        stolenVoices_.fetch_add(1, std::memory_order_relaxed);
    }
    const auto durationSeconds = std::max(0.004F, event.combustionDurationMs * 0.001F);
    voice->bodyPhase = 0.0;
    voice->crackPhase = 0.0;
    voice->filterState = 0.0F;
    const auto cylinders = std::max(1.0F, realtimeState_.cylinderCount.load(std::memory_order_relaxed));
    const auto cylinderNormalization = std::clamp(std::sqrt(4.0F / cylinders), 0.55F, 1.4F);
    const auto layerGain = exhaust ? 0.31F : 0.17F;
    voice->amplitude = std::max(0.001F, event.intensity * (event.misfire ? 0.07F : layerGain)
        * cylinderNormalization);
    voice->decay = std::exp(std::log(0.0001F) / static_cast<float>(sampleRate_ * durationSeconds));
    voice->bodyFrequency = exhaust
        ? 38.0F + event.pressureEstimateBar * 1.10F + event.exhaustResonanceHz * 0.32F
        : 105.0F + event.pressureEstimateBar * 2.10F;
    voice->crackFrequency = (exhaust ? 620.0F : 1'050.0F) + event.intensity * (exhaust ? 1'250.0F : 1'700.0F)
        + static_cast<float>(event.cylinderId % 32U) * 13.0F;
    voice->pan = std::clamp(event.stereoPosition, -0.82F, 0.82F);
    voice->active = true;
}
float RealtimeEngineAudio::noise() noexcept {
    noiseState_ ^= noiseState_ << 13U; noiseState_ ^= noiseState_ >> 17U; noiseState_ ^= noiseState_ << 5U;
    return static_cast<float>(noiseState_) / static_cast<float>(0xffffffffU) * 2.0F - 1.0F;
}
} // namespace enginelab
