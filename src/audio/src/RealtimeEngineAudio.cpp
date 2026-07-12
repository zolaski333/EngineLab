#include <enginelab/audio/RealtimeEngineAudio.hpp>
#include <algorithm>
#include <cmath>
#include <numbers>

namespace enginelab {
void RealtimeEngineAudio::prepare(double sampleRate, int maximumBlockSize) noexcept {
    sampleRate_ = sampleRate > 0.0 ? sampleRate : 48'000.0;
    const auto reflectionSeconds = std::clamp(realtimeState_.exhaustReflectionSeconds.load(std::memory_order_relaxed),
                                               0.001F, 0.080F);
    reflectionDelaySamples_ = std::clamp(static_cast<std::size_t>(sampleRate_ * reflectionSeconds),
                                         std::size_t { 1 }, forwardWave_.size() - 1);
    lowPassCoefficient_ = static_cast<float>(1.0 - std::exp(-2.0 * std::numbers::pi * 11'500.0 / sampleRate_));
    voiceFilterCoefficient_ = static_cast<float>(1.0 - std::exp(-2.0 * std::numbers::pi * 2'100.0 / sampleRate_));
    intakeFilterCoefficient_ = static_cast<float>(1.0 - std::exp(-2.0 * std::numbers::pi * 637.0 / sampleRate_));
    rpmFilterCoefficient_ = static_cast<float>(1.0 - std::exp(-18.0 / sampleRate_));
    reflectionFilterCoefficient_ = static_cast<float>(1.0 - std::exp(-2.0 * std::numbers::pi * 980.0 / sampleRate_));
    antiAliasCoefficient_ = static_cast<float>(1.0 - std::exp(-2.0 * std::numbers::pi
        * std::min(18'000.0, sampleRate_ * 0.42) / sampleRate_));
    if (!convolutionBank_.hasAnyImpulseResponse()) {
        std::array<float, 384> physicalIr {};
        physicalIr[0] = 0.72F;
        for (std::size_t index = 1; index < physicalIr.size(); ++index) {
            const auto time = static_cast<float>(index) / static_cast<float>(sampleRate_);
            physicalIr[index] = std::exp(-time * 4'800.0F)
                * (std::sin(static_cast<float>(2.0 * std::numbers::pi * 1'150.0) * time) * 0.16F
                   + std::sin(static_cast<float>(2.0 * std::numbers::pi * 310.0) * time) * 0.09F);
        }
        setImpulseResponse(physicalIr, sampleRate_, 0);
    }
    convolutionBank_.prepare(sampleRate_, maximumBlockSize, 2);
    release();
    updateExhaustPreset(realtimeState_.exhaustPreset.load(std::memory_order_relaxed));
}
void RealtimeEngineAudio::release() noexcept {
    voices_.fill({}); pendingEvents_.fill({}); pendingEventCount_ = 0; audioTimeSeconds_ = 0.0;
    forwardWave_.fill(0.0F); reverseWave_.fill(0.0F); fdnA_.fill(0.0F); fdnB_.fill(0.0F); fdnC_.fill(0.0F); fdnD_.fill(0.0F);
    waveWrite_ = 0; fdnWriteA_ = 0; fdnWriteB_ = 0; fdnWriteC_ = 0; fdnWriteD_ = 0; lowPassLeft_ = 0.0F; lowPassRight_ = 0.0F;
    pressureTailLeft_ = 0.0F; pressureTailRight_ = 0.0F; exhaustBodyLeft_ = 0.0F; exhaustBodyRight_ = 0.0F;
    exhaustAirLeft_ = 0.0F; exhaustAirRight_ = 0.0F; mechanicalPhase_ = 0.0; valvetrainPhase_ = 0.0;
    starterPhase_ = 0.0; smoothedRpm_ = 0.0F; intakeFilter_ = 0.0F; reflectedLowPass_ = 0.0F; collectorState_ = 0.0F;
    previousCollectorInput_ = 0.0F; jitterDelaySamples_ = 0.0F; jitterHistory_.fill(0.0F); jitterWrite_ = 0;
    levelEnvelope_ = 0.0F; levelGain_ = 1.0F;
    antiAliasLeftA_ = antiAliasLeftB_ = antiAliasRightA_ = antiAliasRightB_ = 0.0F;
    convolutionBank_.reset();
}

void RealtimeEngineAudio::setImpulseResponse(std::span<const float> samples,
                                             double sourceSampleRate,
                                             std::size_t pathIndex) {
    convolutionBank_.load(pathIndex, samples, sourceSampleRate);
}

void RealtimeEngineAudio::setImpulseResponse(juce::AudioBuffer<float>&& samples,
                                             double sourceSampleRate,
                                             std::size_t pathIndex) {
    convolutionBank_.load(pathIndex, std::move(samples), sourceSampleRate);
}

void RealtimeEngineAudio::render(juce::AudioBuffer<float>& output, int startSample, int sampleCount) noexcept {
    output.clear(startSample, sampleCount);
    convolutionBank_.beginBlock(std::min(2, output.getNumChannels()), sampleCount);
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
    const auto volume = realtimeState_.volume.load(std::memory_order_relaxed);
    const auto convolution = realtimeState_.convolution.load(std::memory_order_relaxed);
    const auto highGain = realtimeState_.highFrequencyGain.load(std::memory_order_relaxed);
    const auto lowNoise = realtimeState_.lowFrequencyNoise.load(std::memory_order_relaxed);
    const auto highNoise = realtimeState_.highFrequencyNoise.load(std::memory_order_relaxed);
    const auto combustionGain = realtimeState_.combustionGain.load(std::memory_order_relaxed);
    const auto exhaustGain = realtimeState_.exhaustGain.load(std::memory_order_relaxed);
    const auto intakeGain = realtimeState_.intakeGain.load(std::memory_order_relaxed);
    const auto mechanicalGain = realtimeState_.mechanicalGain.load(std::memory_order_relaxed);
    const auto redline = std::max(500.0F, realtimeState_.redlineRpm.load(std::memory_order_relaxed));
    const auto bankSeparation = std::clamp(realtimeState_.bankSeparation.load(std::memory_order_relaxed), 0.0F, 1.0F);
    const auto exhaustOpenness = std::clamp(realtimeState_.exhaustOpenness.load(std::memory_order_relaxed), 0.15F, 1.45F);
    const auto manifoldPressureKpa = realtimeState_.manifoldPressureKpa.load(std::memory_order_relaxed);
    const auto exhaustPressureKpa = realtimeState_.exhaustPressureKpa.load(std::memory_order_relaxed);
    const auto exhaustFlowGramsPerSecond = realtimeState_.exhaustFlowGramsPerSecond.load(std::memory_order_relaxed);
    const auto intakeDepression = std::clamp((101.325F - manifoldPressureKpa) / 75.0F, 0.0F, 1.2F);
    const auto physicalExhaustPressure = std::clamp((exhaustPressureKpa - 101.325F) / 120.0F, 0.0F, 1.5F);
    const auto physicalExhaustFlow = std::clamp(exhaustFlowGramsPerSecond / 150.0F, 0.0F, 1.8F);
    const auto boostRatio = std::clamp(realtimeState_.boostPressureRatio.load(std::memory_order_relaxed), 1.0F, 3.5F);
    for (int sample = 0; sample < sampleCount; ++sample) {
        const auto preset = realtimeState_.exhaustPreset.load(std::memory_order_relaxed);
        if (preset != activeExhaustPreset_) updateExhaustPreset(preset);
        for (std::size_t index = 0; index < pendingEventCount_;) {
            if (pendingEvents_[index].scheduledTimeSeconds <= audioTimeSeconds_ + 0.5 / sampleRate_) {
                trigger(pendingEvents_[index].event, pendingEvents_[index].exhaust);
                pendingEvents_[index] = pendingEvents_[--pendingEventCount_];
            } else {
                ++index;
            }
        }
        float combustionLeft = 0.0F;
        float combustionRight = 0.0F;
        float exhaustLeft = 0.0F;
        float exhaustRight = 0.0F;
        float intakeLeft = 0.0F;
        float intakeRight = 0.0F;
        float mechanicalLeft = 0.0F;
        float mechanicalRight = 0.0F;
        const auto audibleRpm = targetRpm * timeScale;
        smoothedRpm_ += rpmFilterCoefficient_ * (audibleRpm - smoothedRpm_);
        const auto rotationHz = std::max(0.0F, smoothedRpm_) / 60.0F;
        mechanicalPhase_ += 2.0 * std::numbers::pi * static_cast<double>(rotationHz) / sampleRate_;
        valvetrainPhase_ += 2.0 * std::numbers::pi * static_cast<double>(rotationHz * 2.0F) / sampleRate_;
        starterPhase_ += 2.0 * std::numbers::pi * 92.0 / sampleRate_;
        if (mechanicalPhase_ >= 2.0 * std::numbers::pi) mechanicalPhase_ -= 2.0 * std::numbers::pi;
        if (valvetrainPhase_ >= 2.0 * std::numbers::pi) valvetrainPhase_ -= 2.0 * std::numbers::pi;
        if (starterPhase_ >= 2.0 * std::numbers::pi) starterPhase_ -= 2.0 * std::numbers::pi;
        const auto speedGain = std::clamp(smoothedRpm_ / redline, 0.0F, 1.25F);
        const auto mechanical = (static_cast<float>(std::sin(mechanicalPhase_)) * 0.018F
            + static_cast<float>(std::sin(valvetrainPhase_ * 3.03)) * (0.007F + stress * 0.012F)) * speedGain;
        intakeFilter_ += intakeFilterCoefficient_ * (noise() * throttle * speedGain * lowNoise - intakeFilter_);
        const auto intake = intakeFilter_ * (0.018F + load * 0.028F + intakeDepression * 0.030F);
        const auto starterSound = starter * (static_cast<float>(std::sin(starterPhase_)) * 0.028F + noise() * 0.006F);
        mechanicalLeft += mechanical + starterSound;
        mechanicalRight += mechanical * 0.96F + starterSound * 0.94F;
        intakeLeft += intake * 0.92F;
        intakeRight += intake;
        for (auto& voice : voices_) {
            if (!voice.active) continue;
            const auto attack = std::max(0.00008F, voice.attackSeconds);
            const auto blowdown = std::max(0.0012F, voice.blowdownSeconds);
            const auto shockEnvelope = (1.0F - std::exp(-voice.ageSeconds / attack))
                * std::exp(-voice.ageSeconds / blowdown);
            const auto flowEnvelope = std::exp(-voice.ageSeconds / (blowdown * 1.85F));
            const auto flow = std::clamp(voice.massFlow / 75.0F, 0.0F, 1.8F);
            const auto pressure = std::clamp((voice.runnerPressure - 101.325F) / 120.0F, 0.0F, 1.4F);
            const auto body = static_cast<float>(std::sin(voice.bodyPhase));
            const auto harmonic = static_cast<float>(std::sin(voice.bodyPhase * 2.01)) * 0.32F
                                + static_cast<float>(std::sin(voice.bodyPhase * 3.97)) * 0.13F;
            const auto pipe = static_cast<float>(std::sin(voice.pipePhase)) * 0.48F
                            + static_cast<float>(std::sin(voice.pipePhase * 0.503)) * 0.19F;
            const auto jetCenter = std::clamp(520.0F + flow * 2'700.0F + pressure * 1'300.0F, 260.0F, 6'800.0F);
            const auto jetLowCoeff = static_cast<float>(1.0 - std::exp(-2.0 * std::numbers::pi * (jetCenter * 0.45F) / sampleRate_));
            const auto jetHighCoeff = static_cast<float>(1.0 - std::exp(-2.0 * std::numbers::pi * jetCenter / sampleRate_));
            const auto turbulentNoise = noise() * (0.35F + flow * 0.70F) * flowEnvelope;
            voice.jetLowState += std::clamp(jetLowCoeff, 0.001F, 0.82F) * (turbulentNoise - voice.jetLowState);
            voice.jetHighState += std::clamp(jetHighCoeff, 0.001F, 0.92F) * (voice.jetLowState - voice.jetHighState);
            voice.jetBandState = std::clamp((voice.jetLowState - voice.jetHighState) * (0.42F + flow * 0.48F), -2.0F, 2.0F);
            const auto crack = static_cast<float>(std::sin(voice.crackPhase)) * (0.10F + voice.knock * 0.30F + pressure * 0.11F);
            const auto shock = (body * 0.35F + harmonic * 0.42F + pipe * 0.52F + crack) * shockEnvelope;
            const auto jet = voice.exhaust ? voice.jetBandState * voice.turbulence : noise() * voice.turbulence * 0.20F;
            const auto raw = (shock + jet) * voice.amplitude;
            voice.filterState += voiceFilterCoefficient_ * (raw - voice.filterState);
            if (voice.exhaust) {
                const auto pathLeft = voice.filterState * voice.leftGain;
                const auto pathRight = voice.filterState * voice.rightGain;
                exhaustLeft += pathLeft;
                exhaustRight += pathRight;
                convolutionBank_.addInput(voice.exhaustPathIndex, 0, sample, pathLeft);
                convolutionBank_.addInput(voice.exhaustPathIndex, 1, sample, pathRight);
            } else {
                combustionLeft += voice.filterState * voice.leftGain;
                combustionRight += voice.filterState * voice.rightGain;
            }
            voice.bodyPhase += 2.0 * std::numbers::pi * static_cast<double>(voice.bodyFrequency) / sampleRate_;
            voice.crackPhase += 2.0 * std::numbers::pi * static_cast<double>(voice.crackFrequency) / sampleRate_;
            voice.pipePhase += 2.0 * std::numbers::pi * static_cast<double>(voice.pipeFrequency) / sampleRate_;
            if (voice.bodyPhase >= 2.0 * std::numbers::pi) voice.bodyPhase -= 2.0 * std::numbers::pi;
            if (voice.crackPhase >= 2.0 * std::numbers::pi) voice.crackPhase -= 2.0 * std::numbers::pi;
            if (voice.pipePhase >= 2.0 * std::numbers::pi) voice.pipePhase -= 2.0 * std::numbers::pi;
            voice.amplitude *= voice.decay;
            voice.ageSeconds += static_cast<float>(1.0 / sampleRate_);
            if (voice.amplitude < 0.0001F || voice.ageSeconds > blowdown * 8.0F) voice.active = false;
        }
        const auto collectorDrive = presetDrive_ * (0.86F + exhaustOpenness * 0.20F + load * 0.55F
            + speedGain * 0.42F + physicalExhaustPressure * 0.72F + physicalExhaustFlow * 0.38F
            + (boostRatio - 1.0F) * 0.22F);
        const auto continuousJet = noise() * physicalExhaustFlow
            * (0.0018F + physicalExhaustPressure * 0.0045F) * timeScale;
        const auto collectorInput = (exhaustLeft + exhaustRight) * 0.5F + continuousJet;
        const auto pressureDerivative = collectorInput - previousCollectorInput_;
        previousCollectorInput_ = collectorInput;
        const auto conditionedCollector = collectorInput + pressureDerivative
            * std::clamp(0.08F + physicalExhaustPressure * 0.06F, 0.04F, 0.20F);
        jitterHistory_[jitterWrite_] = conditionedCollector;
        const auto targetJitter = std::clamp(std::abs(noise())
            * (0.35F + physicalExhaustFlow * 1.4F), 0.0F, 5.5F);
        jitterDelaySamples_ += 0.015F * (targetJitter - jitterDelaySamples_);
        const auto delay0 = static_cast<std::size_t>(jitterDelaySamples_);
        const auto delay1 = std::min(delay0 + 1U, jitterHistory_.size() - 1U);
        const auto fraction = jitterDelaySamples_ - static_cast<float>(delay0);
        const auto read0 = (jitterWrite_ + jitterHistory_.size() - delay0) % jitterHistory_.size();
        const auto read1 = (jitterWrite_ + jitterHistory_.size() - delay1) % jitterHistory_.size();
        const auto jitteredCollector = jitterHistory_[read0] * (1.0F - fraction)
            + jitterHistory_[read1] * fraction;
        jitterWrite_ = (jitterWrite_ + 1) % jitterHistory_.size();
        const auto exhaustMono = saturateCollector(jitteredCollector, collectorDrive);
        collectorState_ += 0.18F * (exhaustMono - collectorState_);
        const auto waveRead = (waveWrite_ + forwardWave_.size() - reflectionDelaySamples_) % forwardWave_.size();
        const auto outletPressure = forwardWave_[waveRead];
        reflectedLowPass_ += reflectionFilterCoefficient_ * (outletPressure - reflectedLowPass_);
        const auto highReflection = outletPressure - reflectedLowPass_;
        const auto radiated = reflectedLowPass_ * (0.58F + (1.0F - presetReflection_) * 0.20F)
            + highReflection * (0.16F + highGain * 0.05F);
        const auto returned = reverseWave_[waveRead];
        reverseWave_[waveWrite_] = std::clamp(highReflection * presetReflection_ - returned * 0.10F, -1.8F, 1.8F);
        forwardWave_[waveWrite_] = std::clamp(exhaustMono + returned * 0.24F + collectorState_ * 0.06F, -1.8F, 1.8F);
        waveWrite_ = (waveWrite_ + 1) % forwardWave_.size();
        const auto fdnWet = processMufflerFdn(radiated + exhaustMono * 0.28F) * convolution * presetWet_;
        exhaustBodyLeft_ = exhaustBodyLeft_ * presetDamping_ + (radiated + fdnWet) * (0.005F + convolution * 0.030F);
        exhaustBodyRight_ = exhaustBodyRight_ * (presetDamping_ - 0.001F) + (radiated * 0.76F + fdnWet * 0.91F) * (0.004F + convolution * 0.026F);
        exhaustAirLeft_ += static_cast<float>(1.0 - std::exp(-2.0 * std::numbers::pi * presetToneHz_ / sampleRate_))
            * (exhaustLeft - exhaustAirLeft_);
        exhaustAirRight_ += static_cast<float>(1.0 - std::exp(-2.0 * std::numbers::pi * presetToneHz_ / sampleRate_))
            * (exhaustRight - exhaustAirRight_);
        pressureTailLeft_ = pressureTailLeft_ * 0.992F + (combustionLeft + exhaustLeft) * 0.010F;
        pressureTailRight_ = pressureTailRight_ * 0.992F + (combustionRight + exhaustRight) * 0.010F;
        const auto bankWidth = 1.0F + bankSeparation * 0.18F;
        exhaustLeft += (radiated + fdnWet + exhaustBodyLeft_) * bankWidth + exhaustAirLeft_ * (0.13F + exhaustOpenness * 0.05F)
            + pressureTailLeft_ * (0.20F + convolution * 0.46F) + noise() * highNoise * speedGain * 0.002F;
        exhaustRight += (radiated * 0.82F + fdnWet * 0.88F + exhaustBodyRight_) * bankWidth + exhaustAirRight_ * (0.13F + exhaustOpenness * 0.05F)
            + pressureTailRight_ * (0.20F + convolution * 0.46F) + noise() * highNoise * speedGain * 0.002F;
        auto left = combustionLeft * combustionGain + exhaustLeft * exhaustGain + intakeLeft * intakeGain
            + mechanicalLeft * mechanicalGain;
        auto right = combustionRight * combustionGain + exhaustRight * exhaustGain + intakeRight * intakeGain
            + mechanicalRight * mechanicalGain;
        lowPassLeft_ += lowPassCoefficient_ * (left - lowPassLeft_);
        lowPassRight_ += lowPassCoefficient_ * (right - lowPassRight_);
        if (output.getNumChannels() > 0) output.setSample(0, startSample + sample, lowPassLeft_);
        if (output.getNumChannels() > 1) output.setSample(1, startSample + sample, lowPassRight_);
        audioTimeSeconds_ += 1.0 / sampleRate_;
    }

    convolutionBank_.process(sampleCount);
    const auto irMix = std::clamp(convolution * 0.76F, 0.0F, 0.90F);
    for (int sample = 0; sample < sampleCount; ++sample) {
        const auto dryLeft = output.getNumChannels() > 0
            ? output.getSample(0, startSample + sample) : 0.0F;
        const auto dryRight = output.getNumChannels() > 1
            ? output.getSample(1, startSample + sample) : dryLeft;
        auto left = dryLeft + convolutionBank_.wetSample(0, sample) * irMix * exhaustGain;
        auto right = dryRight + convolutionBank_.wetSample(1, sample) * irMix * exhaustGain;

        const auto magnitude = 0.5F * (std::abs(left) + std::abs(right));
        const auto envelopeCoefficient = magnitude > levelEnvelope_ ? 0.0025F : 0.00012F;
        levelEnvelope_ += envelopeCoefficient * (magnitude - levelEnvelope_);
        const auto targetGain = std::clamp(0.16F / std::max(0.025F, levelEnvelope_), 0.45F, 2.2F);
        levelGain_ += 0.00035F * (targetGain - levelGain_);
        left *= levelGain_;
        right *= levelGain_;

        antiAliasLeftA_ += antiAliasCoefficient_ * (left - antiAliasLeftA_);
        antiAliasLeftB_ += antiAliasCoefficient_ * (antiAliasLeftA_ - antiAliasLeftB_);
        antiAliasRightA_ += antiAliasCoefficient_ * (right - antiAliasRightA_);
        antiAliasRightB_ += antiAliasCoefficient_ * (antiAliasRightA_ - antiAliasRightB_);
        const auto outLeft = std::tanh(antiAliasLeftB_ * (1.35F + highGain * 0.30F)) * 0.42F * volume;
        const auto outRight = std::tanh(antiAliasRightB_ * (1.35F + highGain * 0.30F)) * 0.42F * volume;
        if (output.getNumChannels() > 0) output.setSample(0, startSample + sample, outLeft);
        if (output.getNumChannels() > 1) output.setSample(1, startSample + sample, outRight);
        for (int channel = 2; channel < output.getNumChannels(); ++channel)
            output.setSample(channel, startSample + sample, (outLeft + outRight) * 0.5F);
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
    voice->pipePhase = 0.0;
    voice->filterState = 0.0F;
    voice->ageSeconds = 0.0F;
    voice->jetBandState = 0.0F;
    voice->jetLowState = 0.0F;
    voice->jetHighState = 0.0F;
    const auto cylinders = std::max(1.0F, realtimeState_.cylinderCount.load(std::memory_order_relaxed));
    const auto redline = std::max(500.0F, realtimeState_.redlineRpm.load(std::memory_order_relaxed));
    const auto cylinderDisplacement = std::max(0.03F, realtimeState_.cylinderDisplacementLitres.load(std::memory_order_relaxed));
    const auto boreStroke = std::clamp(realtimeState_.boreStrokeRatio.load(std::memory_order_relaxed), 0.55F, 1.65F);
    const auto exhaustOpenness = std::clamp(realtimeState_.exhaustOpenness.load(std::memory_order_relaxed), 0.15F, 1.45F);
    const auto boostRatio = std::clamp(realtimeState_.boostPressureRatio.load(std::memory_order_relaxed), 1.0F, 3.5F);
    const auto cylinderNormalization = std::clamp(std::pow(4.0F / cylinders, 0.32F), 0.68F, 1.32F);
    const auto chamberScale = std::clamp(std::sqrt(cylinderDisplacement / 0.5F), 0.45F, 2.25F);
    const auto layerGain = exhaust ? 0.31F : 0.17F;
    voice->amplitude = std::max(0.001F, event.intensity * (event.misfire ? 0.07F : layerGain)
        * cylinderNormalization);
    voice->attackSeconds = exhaust ? 0.00018F : 0.00042F;
    voice->blowdownSeconds = std::clamp(durationSeconds * (exhaust ? 0.42F : 0.70F), 0.0015F, 0.014F);
    voice->decay = std::exp(std::log(0.0001F) / static_cast<float>(sampleRate_ * (durationSeconds * 1.35F)));
    voice->bodyFrequency = exhaust
        ? (30.0F + event.pressureEstimateBar * 0.55F + event.exhaustResonanceHz * 0.16F)
            / std::clamp(chamberScale, 0.7F, 1.8F)
        : (88.0F + event.pressureEstimateBar * 1.35F) * std::clamp(boreStroke, 0.75F, 1.28F);
    voice->crackFrequency = (exhaust ? 620.0F : 1'050.0F) + event.intensity * (exhaust ? 1'250.0F : 1'700.0F)
        + static_cast<float>(event.cylinderId % 32U) * 13.0F + (redline / 7'000.0F - 1.0F) * 360.0F;
    voice->pipeFrequency = std::clamp(event.exhaustResonanceHz > 1.0F ? event.exhaustResonanceHz
        : 110.0F + event.pressureEstimateBar * 1.35F, 45.0F, 1'400.0F) * std::clamp(1.18F - cylinderDisplacement * 0.18F, 0.64F, 1.25F);
    const auto leanCrackle = std::clamp(std::abs(event.airFuelRatio - 13.2F) / 7.5F, 0.0F, 1.0F);
    voice->massFlow = std::max(0.0F, event.exhaustFlowMgPerCycle);
    voice->runnerPressure = std::max(80.0F, event.exhaustRunnerPressureKpa);
    voice->exhaustPathIndex = event.exhaustPathIndex;
    const auto flowTone = std::clamp(voice->massFlow / 58.0F, 0.0F, 1.7F);
    voice->turbulence = (exhaust ? 0.11F + exhaustOpenness * 0.07F : 0.07F)
        + leanCrackle * 0.08F + event.knockAmount * 0.12F + flowTone * 0.16F + (boostRatio - 1.0F) * 0.06F;
    voice->knock = std::clamp(event.knockAmount, 0.0F, 1.0F);
    const auto panValue = std::clamp(event.stereoPosition, -0.82F, 0.82F);
    voice->leftGain = std::sqrt((1.0F - panValue) * 0.5F);
    voice->rightGain = std::sqrt((1.0F + panValue) * 0.5F);
    voice->exhaust = exhaust;
    voice->active = true;
}

void RealtimeEngineAudio::updateExhaustPreset(int preset) noexcept {
    activeExhaustPreset_ = std::clamp(preset, 0, 4);
    struct PresetValues { float drive; float damping; float wet; float tone; float delay; float reflection; float fdn; };
    constexpr std::array<PresetValues, 5> presets {{
        { 1.02F, 0.986F, 0.42F, 1'450.0F, 0.006F, 0.36F, 0.56F },
        { 1.42F, 0.974F, 0.24F, 2'650.0F, 0.003F, 0.24F, 0.42F },
        { 0.76F, 0.992F, 0.72F, 920.0F, 0.012F, 0.48F, 0.68F },
        { 1.14F, 0.989F, 0.58F, 1'700.0F, 0.010F, 0.41F, 0.62F },
        { 1.30F, 0.982F, 0.31F, 2'250.0F, 0.004F, 0.30F, 0.48F }
    }};
    const auto values = presets[static_cast<std::size_t>(activeExhaustPreset_)];
    presetDrive_ = values.drive;
    presetDamping_ = values.damping;
    presetWet_ = values.wet;
    presetToneHz_ = values.tone;
    presetReflection_ = std::clamp(values.reflection, 0.08F, 0.72F);
    presetFdngain_ = std::clamp(values.fdn, 0.20F, 0.76F);
    const auto baseReflection = realtimeState_.exhaustReflectionSeconds.load(std::memory_order_relaxed);
    reflectionDelaySamples_ = std::clamp(static_cast<std::size_t>(sampleRate_ * std::max(baseReflection, values.delay)),
                                         std::size_t { 1 }, forwardWave_.size() - 1);
}

float RealtimeEngineAudio::processMufflerFdn(float sample) noexcept {
    const auto a = fdnA_[fdnWriteA_];
    const auto b = fdnB_[fdnWriteB_];
    const auto c = fdnC_[fdnWriteC_];
    const auto d = fdnD_[fdnWriteD_];
    const auto feedback = std::clamp(presetFdngain_ * presetDamping_, 0.12F, 0.74F);
    fdnA_[fdnWriteA_] = std::clamp(sample + (a + b + c + d) * 0.25F * feedback, -1.6F, 1.6F);
    fdnB_[fdnWriteB_] = std::clamp(sample * 0.71F + (a - b + c - d) * 0.25F * feedback, -1.6F, 1.6F);
    fdnC_[fdnWriteC_] = std::clamp(sample * 0.53F + (a + b - c - d) * 0.25F * feedback, -1.6F, 1.6F);
    fdnD_[fdnWriteD_] = std::clamp(sample * 0.39F + (a - b - c + d) * 0.25F * feedback, -1.6F, 1.6F);
    fdnWriteA_ = (fdnWriteA_ + 1) % fdnA_.size();
    fdnWriteB_ = (fdnWriteB_ + 1) % fdnB_.size();
    fdnWriteC_ = (fdnWriteC_ + 1) % fdnC_.size();
    fdnWriteD_ = (fdnWriteD_ + 1) % fdnD_.size();
    return (a * 0.42F + b * 0.31F + c * 0.23F + d * 0.17F) * 0.34F;
}

float RealtimeEngineAudio::saturateCollector(float sample, float drive) const noexcept {
    const auto pushed = sample * std::clamp(drive, 0.35F, 3.4F);
    const auto asymmetric = pushed >= 0.0F ? pushed * 1.08F : pushed * 0.78F;
    const auto folded = std::tanh(asymmetric);
    return std::clamp(folded + 0.08F * std::tanh(pushed * pushed * pushed), -1.0F, 1.0F);
}

float RealtimeEngineAudio::noise() noexcept {
    noiseState_ ^= noiseState_ << 13U; noiseState_ ^= noiseState_ >> 17U; noiseState_ ^= noiseState_ << 5U;
    return static_cast<float>(noiseState_) / static_cast<float>(0xffffffffU) * 2.0F - 1.0F;
}
} // namespace enginelab
