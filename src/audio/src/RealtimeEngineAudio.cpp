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
    pressureHighPassPole_ = static_cast<float>(std::exp(-2.0 * std::numbers::pi * 18.0 / sampleRate_));
    pressureBandCoefficient_ = static_cast<float>(1.0 - std::exp(-2.0 * std::numbers::pi
        * std::min(9'500.0, sampleRate_ * 0.38) / sampleRate_));
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
    // 2x oversampling for the master soft-clip so its harmonics do not alias
    // back down at high RPM (where fundamentals are already high).
    oversampler_ = std::make_unique<juce::dsp::Oversampling<float>>(
        2, 1, juce::dsp::Oversampling<float>::filterHalfBandPolyphaseIIR, true);
    oversampler_->initProcessing(static_cast<std::size_t>(std::max(1, maximumBlockSize)));
    oversampler_->reset();
    oversamplerLatency_ = static_cast<int>(std::lround(oversampler_->getLatencyInSamples()));
    release();
    updateExhaustPreset(realtimeState_.exhaustPreset.load(std::memory_order_relaxed));
}
void RealtimeEngineAudio::release() noexcept {
    voices_.fill({}); pendingEvents_.fill({}); pendingEventCount_ = 0; audioTimeSeconds_ = 0.0; producerClock_ = 0.0;
    forwardWave_.fill(0.0F); reverseWave_.fill(0.0F); fdnA_.fill(0.0F); fdnB_.fill(0.0F); fdnC_.fill(0.0F); fdnD_.fill(0.0F);
    for (auto& line : runnerForward_) line.fill(0.0F);
    for (auto& line : runnerBackward_) line.fill(0.0F);
    runnerWrite_.fill(0); collectorReturn_ = 0.0F;
    waveWrite_ = 0; fdnWriteA_ = 0; fdnWriteB_ = 0; fdnWriteC_ = 0; fdnWriteD_ = 0; lowPassLeft_ = 0.0F; lowPassRight_ = 0.0F;
    pressureTailLeft_ = 0.0F; pressureTailRight_ = 0.0F; exhaustBodyLeft_ = 0.0F; exhaustBodyRight_ = 0.0F;
    exhaustAirLeft_ = 0.0F; exhaustAirRight_ = 0.0F; mechanicalPhase_ = 0.0; valvetrainPhase_ = 0.0;
    starterPhase_ = 0.0; smoothedRpm_ = 0.0F; intakeFilter_ = 0.0F; reflectedLowPass_ = 0.0F; collectorState_ = 0.0F;
    intakeSvfLow_ = 0.0F; intakeSvfBand_ = 0.0F; bovEnvelope_ = 0.0F; bovNoiseState_ = 0.0F;
    previousThrottleForBov_ = 0.0F; fiWhistlePhase_ = 0.0;
    previousCollectorInput_ = 0.0F; jitterDelaySamples_ = 0.0F; jitterHistory_.fill(0.0F); jitterWrite_ = 0;
    levelEnvelope_ = 0.0F; levelGain_ = 1.0F;
    antiAliasLeftA_ = antiAliasLeftB_ = antiAliasRightA_ = antiAliasRightB_ = 0.0F;
    previousPressureSample_ = {}; currentPressureSample_ = {}; nextPressureSample_ = {};
    hasPreviousPressureSample_ = hasCurrentPressureSample_ = hasNextPressureSample_ = false;
    pressureRawPrevious_ = pressureHighPass_ = pressureHighPassPrevious_ = pressureBandLimited_ = 0.0F;
    cylinderPressureRawPrevious_.fill(0.0F);
    cylinderPressureHighPass_.fill(0.0F);
    cylinderPressureHighPassPrevious_.fill(0.0F);
    cylinderPressureBandLimited_.fill(0.0F);
    exhaustPressureRawPrevious_.fill(0.0F);
    exhaustPressureHighPass_.fill(0.0F);
    exhaustPressureHighPassPrevious_.fill(0.0F);
    exhaustPressureBandLimited_.fill(0.0F);
    convolutionBank_.reset();
    if (oversampler_) oversampler_->reset();
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
    // Flush denormals to zero for the whole callback. The exhaust body, muffler
    // FDN, waveguide and dozens of pressure/one-pole states decay toward zero
    // whenever the engine quietens; without this, denormal arithmetic on x86
    // costs 10-100x and eventually overruns the callback (audible dropouts).
    const juce::ScopedNoDenormals noDenormals;
    output.clear(startSample, sampleCount);
    convolutionBank_.beginBlock(std::min(2, output.getNumChannels()), sampleCount);
    FiringEvent event;
    std::size_t drained = 0;
    while (drained++ < 512 && queue_.tryPop(event)) {
        producerClock_ = std::max(producerClock_, event.timeSeconds);
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
    if (pressureQueue_) {
        if (!hasCurrentPressureSample_)
            hasCurrentPressureSample_ = pressureQueue_->tryPop(currentPressureSample_);
        if (hasCurrentPressureSample_ && !hasNextPressureSample_)
            hasNextPressureSample_ = pressureQueue_->tryPop(nextPressureSample_);
        if (hasCurrentPressureSample_)
            producerClock_ = std::max(producerClock_, currentPressureSample_.timeSeconds);
        if (hasNextPressureSample_)
            producerClock_ = std::max(producerClock_, nextPressureSample_.timeSeconds);
    }
    // Fractional-rate clock recovery (software PLL). The simulation timestamps
    // events/pressure on its steady_clock; the audio callback advances on the
    // sound-card crystal. Those clocks drift by tens of ppm, so over minutes a
    // free-running audio clock would either starve the event schedule or pile
    // events up past the pending buffer (dropouts). We slew audioTimeSeconds_
    // toward the producer clock every sample instead of only when the pending
    // queue happens to empty.
    const auto producerTarget = producerClock_ - eventLatencySeconds_;
    double clockDrift = producerTarget - audioTimeSeconds_;
    // Large gaps (device restart, time-scale jump, long stall) snap immediately;
    // everything else is a slow crystal drift that we trim out gradually. The
    // per-sample correction is capped to ~0.25% of the sample period, which
    // comfortably exceeds real sound-card drift (<0.1%) while staying far too
    // small to disturb sample-accurate event placement within a block.
    if (std::abs(clockDrift) > 0.25) { audioTimeSeconds_ = producerTarget; clockDrift = 0.0; }
    const auto maxClockCorrection = 0.0025 / sampleRate_;
    const auto clockCorrectionPerSample = std::clamp(clockDrift / (0.1 * sampleRate_),
                                                     -maxClockCorrection, maxClockCorrection);
    const auto audioTimeStep = 1.0 / sampleRate_ + clockCorrectionPerSample;
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
    const auto peakPistonAccelG = realtimeState_.peakPistonAccelerationG.load(std::memory_order_relaxed);
    const auto intakeRunnerResonanceHz = realtimeState_.intakeRunnerResonanceHz.load(std::memory_order_relaxed);
    const auto intakeRunnerAmplitudeKpa = realtimeState_.intakeRunnerAmplitudeKpa.load(std::memory_order_relaxed);
    const auto fiKind = realtimeState_.forcedInductionKind.load(std::memory_order_relaxed);
    const auto fiShaftRpm = realtimeState_.forcedInductionShaftRpm.load(std::memory_order_relaxed);
    const auto wastegateOpening = std::clamp(realtimeState_.wastegateOpening.load(std::memory_order_relaxed), 0.0F, 1.0F);
    for (std::size_t runner = 0; runner < maxRunners; ++runner) {
        const auto seconds = realtimeState_.runnerDelaySeconds[runner].load(std::memory_order_relaxed);
        runnerDelaySamples_[runner] = std::clamp<std::size_t>(
            static_cast<std::size_t>(static_cast<double>(seconds) * sampleRate_), 1, runnerLineLength - 1);
    }
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
        float physicalCylinderPressureLeft = 0.0F;
        float physicalCylinderPressureRight = 0.0F;
        float physicalBlowdownLeft = 0.0F;
        float physicalBlowdownRight = 0.0F;
        std::array<float, 32> cylinderExhaustPulse {};
        std::size_t activeCylinderCount = 0;
        if (pressureQueue_ && hasCurrentPressureSample_) {
            const auto pressureTime = audioTimeSeconds_ - eventLatencySeconds_;
            while (hasNextPressureSample_ && nextPressureSample_.timeSeconds <= pressureTime) {
                previousPressureSample_ = currentPressureSample_;
                hasPreviousPressureSample_ = true;
                currentPressureSample_ = nextPressureSample_;
                hasNextPressureSample_ = pressureQueue_->tryPop(nextPressureSample_);
            }
            if (!hasNextPressureSample_)
                hasNextPressureSample_ = pressureQueue_->tryPop(nextPressureSample_);
            const auto count = std::min<std::size_t>(currentPressureSample_.cylinderCount,
                currentPressureSample_.pressureBar.size());
            if (count > 0) {
                activeCylinderCount = count;
                const auto denominator = hasNextPressureSample_
                    ? nextPressureSample_.timeSeconds - currentPressureSample_.timeSeconds : 0.0;
                const auto fraction = denominator > 1.0e-9
                    ? std::clamp((pressureTime - currentPressureSample_.timeSeconds) / denominator, 0.0, 1.0)
                    : 0.0;
                // 3-point quadratic (Lagrange) interpolation using the previous,
                // current and next substep. It reconstructs the curvature of the
                // pressure pulse instead of the straight-line kink linear interp
                // leaves, so blow-down edges stay crisp between sparse substeps.
                const auto f = static_cast<float>(fraction);
                const auto quadPrevWeight = 0.5F * f * (f - 1.0F);
                const auto quadCurrentWeight = 1.0F - f * f;
                const auto quadNextWeight = 0.5F * f * (f + 1.0F);
                for (std::size_t index = 0; index < count; ++index) {
                    const auto prevPressure = hasPreviousPressureSample_
                        && index < previousPressureSample_.cylinderCount
                        ? previousPressureSample_.pressureBar[index]
                        : currentPressureSample_.pressureBar[index];
                    const auto nextPressure = hasNextPressureSample_
                        && index < nextPressureSample_.cylinderCount
                        ? nextPressureSample_.pressureBar[index]
                        : currentPressureSample_.pressureBar[index];
                    const auto pressureBar = quadPrevWeight * prevPressure
                        + quadCurrentWeight * currentPressureSample_.pressureBar[index]
                        + quadNextWeight * nextPressure;
                    const auto rawGaugePressure = pressureBar - 1.01325F;
                    cylinderPressureHighPass_[index] = pressureHighPassPole_
                        * (cylinderPressureHighPass_[index] + rawGaugePressure
                            - cylinderPressureRawPrevious_[index]);
                    cylinderPressureRawPrevious_[index] = rawGaugePressure;
                    const auto derivative = cylinderPressureHighPass_[index]
                        - cylinderPressureHighPassPrevious_[index];
                    cylinderPressureHighPassPrevious_[index] = cylinderPressureHighPass_[index];
                    const auto pressureTarget = cylinderPressureHighPass_[index] * 0.0085F
                        + derivative * 0.085F;
                    cylinderPressureBandLimited_[index] += pressureBandCoefficient_
                        * (pressureTarget - cylinderPressureBandLimited_[index]);
                    const auto physicalPressure = std::clamp(cylinderPressureBandLimited_[index], -0.65F, 0.65F)
                        / std::sqrt(static_cast<float>(count));
                    const auto pan = std::clamp(realtimeState_.cylinderPan[index].load(std::memory_order_relaxed),
                                                -0.82F, 0.82F);
                    physicalCylinderPressureLeft += physicalPressure * std::sqrt((1.0F - pan) * 0.5F);
                    physicalCylinderPressureRight += physicalPressure * std::sqrt((1.0F + pan) * 0.5F);
                    const auto prevExhaustPressure = hasPreviousPressureSample_
                        && index < previousPressureSample_.cylinderCount
                        ? previousPressureSample_.exhaustRunnerPressureKpa[index]
                        : currentPressureSample_.exhaustRunnerPressureKpa[index];
                    const auto nextExhaustPressure = hasNextPressureSample_
                        && index < nextPressureSample_.cylinderCount
                        ? nextPressureSample_.exhaustRunnerPressureKpa[index]
                        : currentPressureSample_.exhaustRunnerPressureKpa[index];
                    const auto runnerExhaustPressureKpa = quadPrevWeight * prevExhaustPressure
                        + quadCurrentWeight * currentPressureSample_.exhaustRunnerPressureKpa[index]
                        + quadNextWeight * nextExhaustPressure;
                    const auto exhaustGaugePressure = runnerExhaustPressureKpa - 101.325F;
                    exhaustPressureHighPass_[index] = pressureHighPassPole_
                        * (exhaustPressureHighPass_[index] + exhaustGaugePressure
                            - exhaustPressureRawPrevious_[index]);
                    exhaustPressureRawPrevious_[index] = exhaustGaugePressure;
                    const auto exhaustDerivative = exhaustPressureHighPass_[index]
                        - exhaustPressureHighPassPrevious_[index];
                    exhaustPressureHighPassPrevious_[index] = exhaustPressureHighPass_[index];
                    const auto exhaustFlow = std::lerp(currentPressureSample_.exhaustFlowMgPerCycle[index],
                        hasNextPressureSample_ && index < nextPressureSample_.cylinderCount
                            ? nextPressureSample_.exhaustFlowMgPerCycle[index]
                            : currentPressureSample_.exhaustFlowMgPerCycle[index],
                        static_cast<float>(fraction));
                    const auto flowGain = std::clamp(exhaustFlow / 75.0F, 0.08F, 1.8F);
                    // Runner gauge pressure (AC component) is the real acoustic
                    // exhaust pulse. It is the dominant exhaust source now, with a
                    // light derivative blend for edge (cf. es2d dF_F_mix).
                    const auto exhaustTarget = (exhaustPressureHighPass_[index] * 0.044F
                        + exhaustDerivative * 0.022F) * flowGain;
                    exhaustPressureBandLimited_[index] += pressureBandCoefficient_
                        * (exhaustTarget - exhaustPressureBandLimited_[index]);
                    // Per-cylinder runner pulse (undivided) feeds the waveguide,
                    // where the junction performs the physical averaging.
                    cylinderExhaustPulse[index] = std::clamp(exhaustPressureBandLimited_[index], -1.35F, 1.35F);
                    const auto physicalBlowdown = cylinderExhaustPulse[index] / std::sqrt(static_cast<float>(count));
                    physicalBlowdownLeft += physicalBlowdown * std::sqrt((1.0F - pan) * 0.5F);
                    physicalBlowdownRight += physicalBlowdown * std::sqrt((1.0F + pan) * 0.5F);
                }
            }
        }
        combustionLeft += physicalCylinderPressureLeft;
        combustionRight += physicalCylinderPressureRight;
        // Runner waveguide + collector scattering junction: the collector output
        // carries cross-talk between cylinders and runner-length tuning. A small
        // direct (pre-collector) component adds stereo width from cylinder pan.
        const auto collectorOut = processExhaustWaveguide(cylinderExhaustPulse, activeCylinderCount);
        constexpr auto directWidth = 0.22F;
        const auto exhaustExciteLeft = collectorOut * 0.72F + physicalBlowdownLeft * directWidth;
        const auto exhaustExciteRight = collectorOut * 0.72F + physicalBlowdownRight * directWidth;
        exhaustLeft += exhaustExciteLeft;
        exhaustRight += exhaustExciteRight;
        // Excite the recorded exhaust impulse response with the engine's own
        // collector pressure. This is what makes a V8 and an inline-four differ:
        // the exhaust timbre comes from the physical waveform + runner geometry
        // run through a real IR, not from shared synthetic oscillators.
        convolutionBank_.addInput(0, 0, sample, exhaustExciteLeft);
        convolutionBank_.addInput(0, 1, sample, exhaustExciteRight);
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
        // Mechanical: crank rumble scaled by real piston slap (peak piston
        // acceleration) plus valvetrain clatter (tonal + noisy) that grows with
        // mechanical stress, instead of two fixed sines.
        const auto pistonSlap = std::clamp(peakPistonAccelG / 6'000.0F, 0.0F, 1.4F);
        const auto valvetrainClatter = (static_cast<float>(std::sin(valvetrainPhase_ * 3.03)) * 0.5F
            + noise() * 0.5F) * (0.006F + stress * 0.011F);
        const auto mechanical = (static_cast<float>(std::sin(mechanicalPhase_)) * 0.016F * (0.6F + pistonSlap * 0.9F)
            + valvetrainClatter) * speedGain;
        // Induction: broadband throttle-body whoosh plus a resonant runner "honk"
        // at the simulated intake runner resonance frequency (state-variable
        // band excited by induction turbulence).
        intakeFilter_ += intakeFilterCoefficient_ * (noise() * throttle * speedGain * lowNoise - intakeFilter_);
        const auto whoosh = intakeFilter_ * (0.018F + load * 0.028F + intakeDepression * 0.030F);
        const auto intakeResonanceHz = std::clamp(intakeRunnerResonanceHz, 30.0F, 1'400.0F);
        const auto intakeResonanceAmp = std::clamp(intakeRunnerAmplitudeKpa / 12.0F, 0.0F, 1.5F);
        const auto intakeF = std::clamp(2.0F * std::sin(static_cast<float>(std::numbers::pi)
            * intakeResonanceHz / static_cast<float>(sampleRate_)), 0.0002F, 1.35F);
        const auto intakeExcite = (noise() * 0.7F + intakeFilter_) * (0.4F + throttle * 0.6F) * speedGain;
        intakeSvfLow_ += intakeF * intakeSvfBand_;
        const auto intakeHigh = intakeExcite - intakeSvfLow_ - intakeSvfBand_ * 0.32F;
        intakeSvfBand_ += intakeF * intakeHigh;
        const auto intakeHonk = std::clamp(intakeSvfBand_, -2.0F, 2.0F)
            * intakeResonanceAmp * (0.05F + intakeDepression * 0.05F);
        const auto intake = whoosh + intakeHonk;
        // Forced induction: spool whistle / blower whine (blade/rotor pass tone
        // from shaft speed), wastegate flutter, and a blow-off burst on lift-off.
        float induced = 0.0F;
        if (fiKind != 0) {
            const auto passMultiplier = fiKind == 2 ? 3.0F : 9.0F; // rotor lobes vs turbine blades
            const auto whistleHz = std::clamp(fiShaftRpm / 60.0F * passMultiplier, 200.0F, 12'000.0F);
            fiWhistlePhase_ += 2.0 * std::numbers::pi * static_cast<double>(whistleHz) / sampleRate_;
            if (fiWhistlePhase_ >= 2.0 * std::numbers::pi) fiWhistlePhase_ -= 2.0 * std::numbers::pi;
            const auto boostExcess = std::clamp(boostRatio - 1.0F, 0.0F, 2.5F);
            const auto whistleLevel = boostExcess * (fiKind == 2 ? 0.055F : 0.038F) * (0.35F + speedGain * 0.65F);
            const auto whistle = (static_cast<float>(std::sin(fiWhistlePhase_))
                + 0.3F * static_cast<float>(std::sin(fiWhistlePhase_ * 2.0))) * whistleLevel;
            const auto wastegate = fiKind == 1 ? noise() * wastegateOpening * boostExcess * 0.055F : 0.0F;
            // Trigger the blow-off envelope on a sharp closing throttle while boosted.
            const auto throttleDrop = std::max(0.0F, previousThrottleForBov_ - throttle);
            if (fiKind == 1 && throttleDrop > 0.05F && boostExcess > 0.15F)
                bovEnvelope_ = std::min(1.0F, bovEnvelope_ + throttleDrop * boostExcess * 4.0F);
            bovEnvelope_ *= 0.9994F;
            bovNoiseState_ += 0.08F * (noise() - bovNoiseState_);
            const auto bov = bovNoiseState_ * bovEnvelope_ * 0.22F;
            induced = whistle + wastegate + bov;
        }
        previousThrottleForBov_ = throttle;
        const auto starterSound = starter * (static_cast<float>(std::sin(starterPhase_)) * 0.028F + noise() * 0.006F);
        mechanicalLeft += mechanical + starterSound;
        mechanicalRight += mechanical * 0.96F + starterSound * 0.94F;
        intakeLeft += intake * 0.92F + induced * 0.94F;
        intakeRight += intake + induced;
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
            // Bore-dependent knock resonance (Draper first circumferential mode).
            const auto knockTone = static_cast<float>(std::sin(voice.knockPhase)) * voice.knock * 0.55F;
            const auto shock = (body * 0.35F + harmonic * 0.42F + pipe * 0.52F + crack + knockTone) * shockEnvelope;
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
            voice.knockPhase += 2.0 * std::numbers::pi * static_cast<double>(voice.knockFrequency) / sampleRate_;
            if (voice.bodyPhase >= 2.0 * std::numbers::pi) voice.bodyPhase -= 2.0 * std::numbers::pi;
            if (voice.crackPhase >= 2.0 * std::numbers::pi) voice.crackPhase -= 2.0 * std::numbers::pi;
            if (voice.pipePhase >= 2.0 * std::numbers::pi) voice.pipePhase -= 2.0 * std::numbers::pi;
            if (voice.knockPhase >= 2.0 * std::numbers::pi) voice.knockPhase -= 2.0 * std::numbers::pi;
            voice.amplitude *= voice.decay;
            voice.ageSeconds += static_cast<float>(1.0 / sampleRate_);
            if (voice.amplitude < 0.0001F || voice.ageSeconds > blowdown * 8.0F) voice.active = false;
        }
        // Tamed so the collector only reaches its non-linear region under real
        // load/flow instead of living permanently in the tanh flat zone.
        const auto collectorDrive = presetDrive_ * (0.42F + exhaustOpenness * 0.10F + load * 0.26F
            + speedGain * 0.20F + physicalExhaustPressure * 0.30F + physicalExhaustFlow * 0.18F
            + (boostRatio - 1.0F) * 0.12F);
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
        audioTimeSeconds_ += audioTimeStep;
    }

    convolutionBank_.process(sampleCount);
    const auto irMix = std::clamp(convolution * 0.76F, 0.0F, 0.90F);
    const auto brightness = 1.0F + highGain * 0.22F;
    // Pass 1: mix dry + convolution wet and apply the leveler, still linear.
    for (int sample = 0; sample < sampleCount; ++sample) {
        const auto dryLeft = output.getNumChannels() > 0
            ? output.getSample(0, startSample + sample) : 0.0F;
        const auto dryRight = output.getNumChannels() > 1
            ? output.getSample(1, startSample + sample) : dryLeft;
        auto left = (dryLeft + convolutionBank_.wetSample(0, sample) * irMix * exhaustGain) * brightness;
        auto right = (dryRight + convolutionBank_.wetSample(1, sample) * irMix * exhaustGain) * brightness;

        const auto magnitude = 0.5F * (std::abs(left) + std::abs(right));
        const auto envelopeCoefficient = magnitude > levelEnvelope_ ? 0.0025F : 0.00012F;
        levelEnvelope_ += envelopeCoefficient * (magnitude - levelEnvelope_);
        // Safety limiting must not normalise every engine to the same envelope.
        // Preserve real pressure/flow level differences and attenuate only when
        // the mixed signal approaches the nonlinear output ceiling.
        const auto targetGain = levelEnvelope_ > 0.32F
            ? std::clamp(0.32F / levelEnvelope_, 0.45F, 1.0F) : 1.0F;
        const auto levelRate = targetGain < levelGain_ ? 0.0020F : 0.00012F;
        levelGain_ += levelRate * (targetGain - levelGain_);
        if (output.getNumChannels() > 0) output.setSample(0, startSample + sample, left * levelGain_);
        if (output.getNumChannels() > 1) output.setSample(1, startSample + sample, right * levelGain_);
    }
    // Pass 2: single transparent soft-limiter (identity below the knee), run at
    // 2x oversampling so the peak-shaping harmonics do not alias back down.
    if (oversampler_ && output.getNumChannels() >= 2) {
        auto block = juce::dsp::AudioBlock<float>(output)
            .getSubsetChannelBlock(0, 2)
            .getSubBlock(static_cast<std::size_t>(startSample), static_cast<std::size_t>(sampleCount));
        auto oversampled = oversampler_->processSamplesUp(block);
        for (std::size_t channel = 0; channel < oversampled.getNumChannels(); ++channel) {
            auto* data = oversampled.getChannelPointer(channel);
            for (std::size_t index = 0; index < oversampled.getNumSamples(); ++index)
                data[index] = softLimit(data[index]);
        }
        oversampler_->processSamplesDown(block);
    } else {
        for (int sample = 0; sample < sampleCount; ++sample) {
            if (output.getNumChannels() > 0)
                output.setSample(0, startSample + sample, softLimit(output.getSample(0, startSample + sample)));
            if (output.getNumChannels() > 1)
                output.setSample(1, startSample + sample, softLimit(output.getSample(1, startSample + sample)));
        }
    }
    // Pass 3: gentle anti-alias tone shaping, master volume, downmix extras.
    for (int sample = 0; sample < sampleCount; ++sample) {
        const auto left = output.getNumChannels() > 0 ? output.getSample(0, startSample + sample) : 0.0F;
        const auto right = output.getNumChannels() > 1 ? output.getSample(1, startSample + sample) : left;
        antiAliasLeftA_ += antiAliasCoefficient_ * (left - antiAliasLeftA_);
        antiAliasLeftB_ += antiAliasCoefficient_ * (antiAliasLeftA_ - antiAliasLeftB_);
        antiAliasRightA_ += antiAliasCoefficient_ * (right - antiAliasRightA_);
        antiAliasRightB_ += antiAliasCoefficient_ * (antiAliasRightA_ - antiAliasRightB_);
        const auto outLeft = antiAliasLeftB_ * 0.72F * volume;
        const auto outRight = antiAliasRightB_ * 0.72F * volume;
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
    voice->knockPhase = 0.0;
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
    // Exhaust voices are now only crack/transient accents on top of the
    // pressure-driven exhaust body, so they sit well below the physical source.
    const auto layerGain = exhaust ? 0.12F : 0.13F;
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
    // Draper first circumferential knock mode: f ~= 1.841 c / (pi B), hot burned
    // gas sound speed ~= 900 m/s. Larger bores knock lower, smaller bores higher.
    const auto boreMm = std::clamp(realtimeState_.meanBoreMm.load(std::memory_order_relaxed), 40.0F, 160.0F);
    voice->knockFrequency = std::clamp(1.841F * 900'000.0F
        / (static_cast<float>(std::numbers::pi) * boreMm), 2'000.0F, 9'000.0F);
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

float RealtimeEngineAudio::processExhaustWaveguide(const std::array<float, 32>& pulse, std::size_t count) noexcept {
    // Bidirectional digital waveguide: each cylinder's blow-down pulse travels
    // its runner (a delay line) to a shared collector. The N-port scattering
    // junction reflects part of every runner's wave back into all the others,
    // which is the physical origin of collector cross-talk and header tuning
    // (e.g. flat-plane vs cross-plane V8 character emerges from firing order +
    // runner lengths, not from a preset).
    const auto n = std::min(count, maxRunners);
    if (n == 0) { collectorReturn_ *= 0.5F; return 0.0F; }
    std::array<float, maxRunners> arrived {};
    float junctionSum = 0.0F;
    for (std::size_t i = 0; i < n; ++i) {
        const auto delay = std::clamp<std::size_t>(runnerDelaySamples_[i], 1, runnerLineLength - 1);
        const auto readIdx = (runnerWrite_[i] + runnerLineLength - delay) % runnerLineLength;
        arrived[i] = runnerForward_[i][readIdx];
        junctionSum += arrived[i];
    }
    // Equal-impedance pressure junction across the N runners plus the collector
    // outlet: p_J = 2*(sum of incident waves)/(N+1).
    const auto junctionPressure = 2.0F * (junctionSum + collectorReturn_) / static_cast<float>(n + 1);
    for (std::size_t i = 0; i < n; ++i) {
        const auto delay = std::clamp<std::size_t>(runnerDelaySamples_[i], 1, runnerLineLength - 1);
        const auto readIdx = (runnerWrite_[i] + runnerLineLength - delay) % runnerLineLength;
        // Wave that left the junction 'delay' samples ago now reaches the port
        // and reflects off the (mostly closed during exhaust) valve/port end.
        const auto portReturn = runnerBackward_[i][readIdx] * 0.85F;
        runnerForward_[i][runnerWrite_[i]] = std::clamp(pulse[i] + portReturn, -2.0F, 2.0F);
        runnerBackward_[i][runnerWrite_[i]] = std::clamp(junctionPressure - arrived[i], -2.0F, 2.0F);
        runnerWrite_[i] = (runnerWrite_[i] + 1) % runnerLineLength;
    }
    const auto collectorOut = std::clamp(junctionPressure - collectorReturn_, -2.0F, 2.0F);
    // Partial reflection back from the downstream muffler/outlet discontinuity.
    collectorReturn_ = collectorOut * 0.2F;
    return collectorOut;
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

float RealtimeEngineAudio::softLimit(float sample) noexcept {
    // Identity below the knee, smooth compression above, hard-bounded to 1.0.
    constexpr float knee = 0.72F;
    const auto magnitude = std::abs(sample);
    if (magnitude <= knee) return sample;
    const auto over = (magnitude - knee) / (1.0F - knee);
    const auto shaped = knee + (1.0F - knee) * std::tanh(over);
    return std::copysign(shaped, sample);
}

float RealtimeEngineAudio::noise() noexcept {
    noiseState_ ^= noiseState_ << 13U; noiseState_ ^= noiseState_ >> 17U; noiseState_ ^= noiseState_ << 5U;
    return static_cast<float>(noiseState_) / static_cast<float>(0xffffffffU) * 2.0F - 1.0F;
}
} // namespace enginelab
