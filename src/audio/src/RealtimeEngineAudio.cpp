#include <enginelab/audio/RealtimeEngineAudio.hpp>
#include <enginelab/audio/AcousticMonitorCalibration.hpp>
#include <enginelab/audio/NonlinearDuctAcoustics.hpp>
#include <enginelab/foundation/ExhaustGasAcoustics.hpp>
#include <algorithm>
#include <cmath>
#include <numbers>

namespace enginelab {
namespace {
constexpr double referenceSampleRate = 48'000.0;
// Historical non-SI layers predate the thermoacoustic path and retain their
// established monitor mapping. The physical exhaust bypasses this factor: its
// pressure is calibrated independently below, so changing engine displacement
// or a legacy voicing constant cannot silently change a pascal at the observer.
constexpr float legacyReferenceLevel = 3.50F;
constexpr double observerDistanceM = 1.0;
constexpr double ambientSoundSpeedMps = 343.0;

[[nodiscard]] float exhaustSoundSpeedMps(float temperatureC) noexcept {
    // Shared with the exhaust topology compiler so the audio delay lines and the
    // graph's resonance modes never assume different wave speeds.
    return static_cast<float>(exhaustSpeedOfSoundMps(static_cast<double>(temperatureC)));
}

[[nodiscard]] float finiteState(float value, float absoluteLimit = 12.0F) noexcept {
    return std::isfinite(value) ? std::clamp(value, -absoluteLimit, absoluteLimit) : 0.0F;
}

[[nodiscard]] float rateInvariantPole(float referencePole, double sampleRate) noexcept {
    return static_cast<float>(std::pow(std::clamp(referencePole, 0.0F, 0.999999F),
                                       referenceSampleRate / sampleRate));
}

[[nodiscard]] float rateInvariantCoefficient(float referenceCoefficient,
                                             double sampleRate) noexcept {
    return 1.0F - rateInvariantPole(1.0F - referenceCoefficient, sampleRate);
}

/** Interpolated read at a fractional delay behind a masked ring's write cursor. */
[[nodiscard]] float readWaveDelayed(const std::vector<float>& line, std::size_t write,
                                    std::size_t mask, float delaySamples) noexcept {
    const auto delay0 = static_cast<std::size_t>(delaySamples);
    const auto fraction = delaySamples - static_cast<float>(delay0);
    const auto read0 = (write + line.size() - delay0) & mask;
    const auto read1 = (write + line.size() - delay0 - 1U) & mask;
    return std::lerp(line[read0], line[read1], fraction);
}

/** Finite-amplitude read: probe at the nominal delay, re-read at the
 *  amplitude-corrected arrival time. See NonlinearDuctAcoustics. */
[[nodiscard]] float readWaveSteepened(const std::vector<float>& line, std::size_t write,
                                      std::size_t mask, float delaySamples,
                                      float delayLimit, float stiffnessRhoC2) noexcept {
    return NonlinearDuctAcoustics::steepenedRead(
        [&line, write, mask](float delay) {
            return readWaveDelayed(line, write, mask, delay);
        },
        delaySamples, delayLimit, stiffnessRhoC2);
}

/** Smallest power of two >= value, floored at a usable minimum. */
[[nodiscard]] std::size_t delayLineLength(double requiredSamples, std::size_t minimum) noexcept {
    const auto required = std::isfinite(requiredSamples)
        ? static_cast<std::size_t>(std::ceil(std::max(0.0, requiredSamples))) : minimum;
    std::size_t length = minimum;
    while (length < required && length < (std::size_t { 1 } << 24U)) length <<= 1U;
    return length;
}
}

RealtimeEngineAudio::RealtimeEngineAudio(FiringEventQueue& queue,
                                         RealtimeAudioState& state,
                                         CylinderPressureQueue* pressureQueue,
                                         const ExhaustGraph* exhaustGraph,
                                         const EngineConfig* engineConfig)
    : queue_(queue), realtimeState_(state), pressureQueue_(pressureQueue),
      runners_(std::make_unique<RunnerWaveguides>()) {
    if (exhaustGraph != nullptr) {
        std::array<std::uint32_t, maxRunners> cylinderIds {};
        const auto count = std::clamp<std::size_t>(static_cast<std::size_t>(
            std::lround(realtimeState_.cylinderCount.load(std::memory_order_relaxed))),
            1U, maxRunners);
        for (std::size_t index = 0; index < count; ++index)
            cylinderIds[index] = realtimeState_.cylinderId[index].load(
                std::memory_order_relaxed);
        auto compiled = std::make_unique<AcousticExhaustNetwork>(
            *exhaustGraph,
            std::span<const std::uint32_t>(cylinderIds.data(), count));
        if (compiled->valid()) acousticExhaustNetwork_ = std::move(compiled);
    }
    if (engineConfig != nullptr) {
        const auto microphoneDistance = [](const AcousticPoint3M& point) {
            return std::sqrt(point.x * point.x + point.y * point.y
                + point.z * point.z);
        };
        const auto observerDistance = 0.5 * (
            microphoneDistance(engineConfig->acousticObserver.leftMicrophoneM)
            + microphoneDistance(engineConfig->acousticObserver.rightMicrophoneM));
        auto structural = std::make_unique<StructuralModalRadiator>(*engineConfig);
        if (structural->valid()) structuralModalRadiator_ = std::move(structural);
        auto intake = std::make_unique<AcousticIntakeNetwork>(*engineConfig);
        if (intake->valid()) acousticIntakeNetwork_ = std::move(intake);
        auto forcedInduction = std::make_unique<ForcedInductionAcoustics>(
            engineConfig->forcedInduction, observerDistance);
        if (forcedInduction->valid())
            forcedInductionAcoustics_ = std::move(forcedInduction);
    }
}

std::size_t RealtimeEngineAudio::runnerDelaySamples(double delaySeconds,
                                                    double sampleRate) noexcept {
    const auto safeRate = std::isfinite(sampleRate) && sampleRate > 0.0 ? sampleRate : referenceSampleRate;
    const auto safeDelay = std::isfinite(delaySeconds)
        ? std::clamp(delaySeconds, 0.0, maximumPublishedDelaySeconds) : 0.0;
    return std::max<std::size_t>(1, static_cast<std::size_t>(std::llround(safeDelay * safeRate)));
}

void RealtimeEngineAudio::allocateDelayLines() {
    // The runtime publishes its static geometry in its constructor, which always
    // precedes ours, so the counts and delays below describe the engine actually
    // loaded. A structural engine change rebuilds both objects, so this never
    // observes a stale engine.
    allocatedRunners_ = std::clamp<std::size_t>(static_cast<std::size_t>(std::lround(
        realtimeState_.cylinderCount.load(std::memory_order_relaxed))), 1, maxRunners);
    allocatedPaths_ = std::clamp<std::size_t>(
        realtimeState_.exhaustPathCount.load(std::memory_order_relaxed), 1, maximumPaths);

    const auto publishedSeconds = [](const std::atomic<float>& value) {
        const auto seconds = value.load(std::memory_order_relaxed);
        return std::isfinite(seconds)
            ? std::clamp(static_cast<double>(seconds), 0.0, maximumPublishedDelaySeconds) : 0.0;
    };
    auto longestRunnerSeconds = 0.0;
    for (std::size_t runner = 0; runner < allocatedRunners_; ++runner)
        longestRunnerSeconds = std::max(longestRunnerSeconds,
            publishedSeconds(realtimeState_.runnerDelaySeconds[runner]));
    auto longestReflectionSeconds = 0.0;
    for (std::size_t path = 0; path < allocatedPaths_; ++path)
        longestReflectionSeconds = std::max(longestReflectionSeconds,
            publishedSeconds(realtimeState_.exhaustPathReflectionSeconds[path]));
    auto longestMufflerSeconds = 0.0;
    for (std::size_t path = 0; path < allocatedPaths_; ++path)
        longestMufflerSeconds = std::max(longestMufflerSeconds,
            publishedSeconds(realtimeState_.exhaustPathMufflerTraversalSeconds[path]));

    // Every line carries its physical delay stretched by the worst acoustic
    // scale, plus two samples for the fractional read's second tap.
    const auto scaled = [this](double seconds) {
        return seconds * maximumAcousticDelayScale * sampleRate_ + 4.0;
    };
    const auto runnerLength = delayLineLength(scaled(longestRunnerSeconds), minimumLineLength);
    const auto waveLength = delayLineLength(scaled(longestReflectionSeconds), minimumLineLength);
    const auto mufflerLength = delayLineLength(scaled(longestMufflerSeconds), minimumLineLength);
    const auto observerDelaySamples = observerDistanceM / ambientSoundSpeedMps * sampleRate_;
    const auto observerLineLength = delayLineLength(
        observerDelaySamples + 4.0, minimumLineLength);

    runners_->forward.allocate(allocatedRunners_, runnerLength);
    runners_->backward.allocate(allocatedRunners_, runnerLength);

    exhaustPaths_.assign(allocatedPaths_, ExhaustPathState {});
    for (std::size_t pathIndex = 0; pathIndex < exhaustPaths_.size(); ++pathIndex) {
        auto& path = exhaustPaths_[pathIndex];
        path.forwardWave.assign(waveLength, 0.0F);
        path.reverseWave.assign(waveLength, 0.0F);
        path.waveMask = waveLength - 1;
        path.muffler.prepare(mufflerLength);
        path.observerPressure.assign(observerLineLength, 0.0F);
        path.observerMask = observerLineLength - 1;
        path.observerDelaySamples = static_cast<float>(observerDelaySamples);
        for (std::size_t line = 0; line < referenceFdnSamples.size(); ++line) {
            const auto fdnLength = delayLineLength(
                referenceFdnSamples[line] * sampleRate_ / referenceSampleRate
                    * maximumAcousticDelayScale + 4.0,
                minimumLineLength);
            path.fdn[line].assign(fdnLength, 0.0F);
        }
        const auto publishedArea = realtimeState_.exhaustPathOutletAreaM2[pathIndex]
            .load(std::memory_order_relaxed);
        const auto areaM2 = std::isfinite(publishedArea) && publishedArea > 1.0e-6F
            ? static_cast<double>(publishedArea) : 0.0020;
        const auto radiusM = std::sqrt(areaM2 / std::numbers::pi);
        (void) path.radiation.prepare(sampleRate_, radiusM, observerDistanceM);
        path.outletAcousticAdmittance = static_cast<float>(
            areaM2 / (1.2 * 343.0));
    }
    for (std::size_t line = 0; line < referenceFdnSamples.size(); ++line)
        fdnDelaySamples_[line] = std::clamp<std::size_t>(static_cast<std::size_t>(std::llround(
            referenceFdnSamples[line] * sampleRate_ / referenceSampleRate)),
            1, exhaustPaths_.front().fdn[line].size());
}

void RealtimeEngineAudio::prepare(double sampleRate, int maximumBlockSize) noexcept {
    sampleRate_ = sampleRate > 0.0 ? sampleRate : 48'000.0;
    maximumBlockSize_ = std::max(1, maximumBlockSize);
    // Keep the audio cursor at least one complete host callback plus a small
    // scheduling margin behind the producer. A fixed 20 ms look-ahead starves
    // the end of 1024/2048-sample blocks at lower sample rates.
    eventLatencySeconds_ = std::max(0.020,
        static_cast<double>(maximumBlockSize_) / sampleRate_ + 0.005);
    lowPassCoefficient_ = static_cast<float>(1.0 - std::exp(-2.0 * std::numbers::pi * 11'500.0 / sampleRate_));
    voiceFilterCoefficient_ = static_cast<float>(1.0 - std::exp(-2.0 * std::numbers::pi * 2'100.0 / sampleRate_));
    intakeFilterCoefficient_ = static_cast<float>(1.0 - std::exp(-2.0 * std::numbers::pi * 637.0 / sampleRate_));
    rpmFilterCoefficient_ = static_cast<float>(1.0 - std::exp(-18.0 / sampleRate_));
    reflectionFilterCoefficient_ = static_cast<float>(1.0 - std::exp(-2.0 * std::numbers::pi * 980.0 / sampleRate_));
    antiAliasCoefficient_ = static_cast<float>(1.0 - std::exp(-2.0 * std::numbers::pi
        * std::min(18'000.0, sampleRate_ * 0.42) / sampleRate_));
    dcBlockPole_ = static_cast<float>(std::exp(-2.0 * std::numbers::pi * 12.0 / sampleRate_));
    toneCoefficient_ = static_cast<float>(1.0 - std::exp(-2.0 * std::numbers::pi
        * std::min(1'800.0, sampleRate_ * 0.10) / sampleRate_));
    radiationLowCoefficient_ = static_cast<float>(1.0 - std::exp(
        -2.0 * std::numbers::pi * 250.0 / sampleRate_));
    pressureHighPassPole_ = static_cast<float>(std::exp(-2.0 * std::numbers::pi * 18.0 / sampleRate_));
    pressureBandCoefficient_ = static_cast<float>(1.0 - std::exp(-2.0 * std::numbers::pi
        * std::min(9'500.0, sampleRate_ * 0.38) / sampleRate_));
    thermoacousticMeanCoefficient_ = static_cast<float>(1.0 - std::exp(
        -2.0 * std::numbers::pi * 8.0 / sampleRate_));
    // 10 Hz: fast enough to track real temperature drift and rpm sweeps, slow
    // enough to attenuate frame-rate telemetry steps by about 28 dB at 240 Hz.
    controlRampCoefficient_ = static_cast<float>(1.0 - std::exp(
        -2.0 * std::numbers::pi * 10.0 / sampleRate_));
    pressureTailDecay_ = rateInvariantPole(0.992F, sampleRate_);
    pressureTailInputCoefficient_ = 0.010F * (1.0F - pressureTailDecay_) / (1.0F - 0.992F);
    jitterCoefficient_ = rateInvariantCoefficient(0.015F, sampleRate_);
    collectorCoefficient_ = rateInvariantCoefficient(0.18F, sampleRate_);
    levelAttackCoefficient_ = rateInvariantCoefficient(0.0025F, sampleRate_);
    levelReleaseCoefficient_ = rateInvariantCoefficient(0.00012F, sampleRate_);
    gainAttackCoefficient_ = rateInvariantCoefficient(0.0020F, sampleRate_);
    gainReleaseCoefficient_ = levelReleaseCoefficient_;
    allocateDelayLines();
    if (acousticExhaustNetwork_
        && !acousticExhaustNetwork_->prepare(
            sampleRate_, maximumAcousticDelayScale, observerDistanceM))
        acousticExhaustNetwork_.reset();
    if (structuralModalRadiator_
        && !structuralModalRadiator_->prepare(sampleRate_))
        structuralModalRadiator_.reset();
    if (acousticIntakeNetwork_
        && !acousticIntakeNetwork_->prepare(sampleRate_, observerDistanceM))
        acousticIntakeNetwork_.reset();
    if (forcedInductionAcoustics_
        && !forcedInductionAcoustics_->prepare(sampleRate_))
        forcedInductionAcoustics_.reset();
    // No fabricated default IR. A user-supplied measured room/cabin response is
    // legitimate downstream propagation; inventing one here would make the
    // exhaust sound larger without improving the simulated source.
    convolutionBank_.prepare(sampleRate_, maximumBlockSize_, 2);
    // 2x oversampling for the master soft-clip so its harmonics do not alias
    // back down at high RPM (where fundamentals are already high).
    oversampler_ = std::make_unique<juce::dsp::Oversampling<float>>(
        2, 1, juce::dsp::Oversampling<float>::filterHalfBandPolyphaseIIR, true);
    oversampler_->initProcessing(static_cast<std::size_t>(maximumBlockSize_));
    oversampler_->reset();
    release();
    updateExhaustPreset(realtimeState_.exhaustPreset.load(std::memory_order_relaxed));
}
void RealtimeEngineAudio::release() noexcept {
    voices_.fill({}); pendingEvents_.fill({}); pendingEventCount_ = 0;
    observedExhaustPathCount_ = 1;
    audioTimeSeconds_ = -eventLatencySeconds_; producerClock_ = 0.0;
    runners_->forward.clear();
    runners_->backward.clear();
    runners_->write.fill(0);
    for (auto& path : exhaustPaths_) {
        path.collectorReturn = 0.0F;
        std::fill(path.forwardWave.begin(), path.forwardWave.end(), 0.0F);
        std::fill(path.reverseWave.begin(), path.reverseWave.end(), 0.0F);
        std::fill(path.observerPressure.begin(), path.observerPressure.end(), 0.0F);
        for (auto& line : path.fdn) std::fill(line.begin(), line.end(), 0.0F);
        path.jitterHistory.fill(0.0F);
        path.waveWrite = 0;
        path.observerWrite = 0;
        path.fdnWrite.fill(0);
        path.jitterWrite = 0; path.reflectionDelaySamples = 1.0F;
        path.reflectedLowPass = path.collectorState = path.previousCollectorInput = 0.0F;
        path.jitterDelaySamples = path.exhaustBodyLeft = path.exhaustBodyRight = 0.0F;
        path.exhaustAirLeft = path.exhaustAirRight = 0.0F;
        path.wallLoss = {};
        path.wallLossOutbound.reset();
        path.wallLossReturn.reset();
        path.muffler.reset();
        path.mediumDensityKgPerM3 = 1.2F;
        path.mediumSoundSpeedMps = 343.0F;
        path.radiation.reset();
    }
    lowPassLeft_ = 0.0F; lowPassRight_ = 0.0F;
    pressureTailLeft_ = 0.0F; pressureTailRight_ = 0.0F;
    mechanicalPhase_ = 0.0; valvetrainPhase_ = 0.0;
    starterPhase_ = 0.0; smoothedRpm_ = 0.0F; intakeFilter_ = 0.0F;
    intakeSvfLow_ = 0.0F; intakeSvfBand_ = 0.0F;
    levelEnvelope_ = 0.0F; levelGain_ = 1.0F;
    antiAliasLeftA_ = antiAliasLeftB_ = antiAliasRightA_ = antiAliasRightB_ = 0.0F;
    dcInputLeft_ = dcInputRight_ = dcOutputLeft_ = dcOutputRight_ = 0.0F;
    toneLowLeft_ = toneLowRight_ = 0.0F;
    exhaustRadiationLowLeft_ = exhaustRadiationLowRight_ = 0.0F;
    wetExhaustRadiationLowLeft_ = wetExhaustRadiationLowRight_ = 0.0F;
    currentPressureSample_ = {}; nextPressureSample_ = {};
    hasCurrentPressureSample_ = hasNextPressureSample_ = false;
    cylinderPressureRawPrevious_.fill(0.0F);
    cylinderPressureHighPass_.fill(0.0F);
    cylinderPressureHighPassPrevious_.fill(0.0F);
    cylinderPressureBandLimited_.fill(0.0F);
    exhaustPressureRawPrevious_.fill(0.0F);
    exhaustPressureHighPass_.fill(0.0F);
    exhaustPressureHighPassPrevious_.fill(0.0F);
    exhaustPressureBandLimited_.fill(0.0F);
    exhaustMeanPressurePa_.fill(0.0F);
    exhaustMeanMassFlowKgPerSecond_.fill(0.0F);
    thermoacousticRunnerAdmittance_.fill(0.0F);
    thermoacousticPortReflection_.fill(0.999F);
    runnerStiffnessRhoC2_.fill(0.0F);
    thermoacousticMeanInitialised_.fill(false);
    // A stale boundary would let a new engine's runner reflect off the previous
    // engine's valve until the first pressure sample arrives.
    portBoundary_.fill({});
    for (auto& state : portReflectionState_) state.reset();
    for (auto& state : portSourceState_) state.reset();
    runnerWallLoss_.fill({});
    for (auto& state : runnerWallLossToJunction_) state.reset();
    for (auto& state : runnerWallLossToPort_) state.reset();
    boundaryReconstruction_ = {};
    boundaryReconstructionCouplingHz_ = 0.0;
    for (auto& state : boundaryReconstructionPressure_) state.reset();
    for (auto& state : boundaryReconstructionFlow_) state.reset();
    valveFlowAcousticSource_ = {};
    for (auto& state : valveFlowAcousticSourceState_) state.reset();
    if (acousticExhaustNetwork_) acousticExhaustNetwork_->reset();
    if (structuralModalRadiator_) structuralModalRadiator_->reset();
    if (acousticIntakeNetwork_) acousticIntakeNetwork_->reset();
    if (forcedInductionAcoustics_) forcedInductionAcoustics_->reset();
    physicalExhaustActive_ = false;
    pressureSampleIntervalSeconds_ = 0.0;
    levelLimitedSamples_.store(0, std::memory_order_relaxed);
    minObservedLevelGain_.store(1.0F, std::memory_order_relaxed);
    maxObservedExhaustPressurePa_.store(0.0F, std::memory_order_relaxed);
    maxObservedExhaustJetNoisePressurePa_.store(
        0.0F, std::memory_order_relaxed);
    maxObservedIntakePressurePa_.store(0.0F, std::memory_order_relaxed);
    maxIntakeSourcePressurePa_.store(0.0F, std::memory_order_relaxed);
    maxIntakeRunnerPressurePa_.store(0.0F, std::memory_order_relaxed);
    maxIntakePlenumPressurePa_.store(0.0F, std::memory_order_relaxed);
    maxIntakeAirboxPressurePa_.store(0.0F, std::memory_order_relaxed);
    maxIntakeMouthPressurePa_.store(0.0F, std::memory_order_relaxed);
    maxIntakeRadiatedPressurePa_.store(0.0F, std::memory_order_relaxed);
    maxObservedStructuralPressurePa_.store(0.0F, std::memory_order_relaxed);
    maxPreLimiterMagnitude_.store(0.0F, std::memory_order_relaxed);
    legacyPathSamples_.store(0, std::memory_order_relaxed);
    invalidBoundarySamples_.store(0, std::memory_order_relaxed);
    convolutionBank_.reset();
    if (oversampler_) oversampler_->reset();
}

void RealtimeEngineAudio::activatePhysicalExhaust() noexcept {
    if (physicalExhaustActive_) return;
    physicalExhaustActive_ = true;

    // Event voices are banks of sine oscillators (body, crack, pipe and knock
    // partials) triggered per firing event. They are a synthesis of what an
    // engine sounds like, not a model of anything, so once SI boundary
    // characteristics exist they have no physical owner and are retired --
    // combustion voices as well as exhaust voices.
    //
    // Retiring the combustion voices matters more than it looks. Measured on
    // the render harness, they carried roughly 17 dB more energy than the
    // physical exhaust radiation did, so while they were summed in, the
    // delivered voice was overwhelmingly oscillator output with the physical
    // path buried underneath it. Any improvement to the exhaust model was
    // inaudible against them, and the "physical" pipeline was physical only in
    // the part nobody could hear.
    for (auto& voice : voices_) voice.active = false;
    pendingEventCount_ = 0;
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
    renderWithStems(output, startSample, sampleCount, {});
}

void RealtimeEngineAudio::renderWithStems(
    juce::AudioBuffer<float>& output, int startSample, int sampleCount,
    const RealtimeAudioStemBuffers& stems) noexcept {
    if (sampleCount <= 0 || startSample < 0 || startSample >= output.getNumSamples()) return;
    sampleCount = std::min(sampleCount, output.getNumSamples() - startSample);
    if (sampleCount > maximumBlockSize_) {
        for (int offset = 0; offset < sampleCount; offset += maximumBlockSize_)
            renderWithStems(output, startSample + offset,
                std::min(maximumBlockSize_, sampleCount - offset), stems);
        return;
    }
    // Flush denormals to zero for the whole callback. The exhaust body, muffler
    // FDN, waveguide and dozens of pressure/one-pole states decay toward zero
    // whenever the engine quietens; without this, denormal arithmetic on x86
    // costs 10-100x and eventually overruns the callback (audible dropouts).
    const juce::ScopedNoDenormals noDenormals;
    output.clear(startSample, sampleCount);
    const auto writableStem = [&](juce::AudioBuffer<float>* candidate) {
        return candidate != nullptr && candidate != &output
            && candidate->getNumChannels() > 0
            && startSample + sampleCount <= candidate->getNumSamples();
    };
    const RealtimeAudioStemBuffers writable {
        writableStem(stems.combustion) ? stems.combustion : nullptr,
        writableStem(stems.exhaustDry) ? stems.exhaustDry : nullptr,
        writableStem(stems.exhaustIr) ? stems.exhaustIr : nullptr,
        writableStem(stems.intake) ? stems.intake : nullptr,
        writableStem(stems.forcedInduction) ? stems.forcedInduction : nullptr,
        writableStem(stems.mechanical) ? stems.mechanical : nullptr
    };
    const std::array stemBuffers {
        writable.combustion, writable.exhaustDry, writable.exhaustIr,
        writable.intake, writable.forcedInduction, writable.mechanical
    };
    const auto captureStems = std::any_of(
        stemBuffers.begin(), stemBuffers.end(), [](const auto* buffer) { return buffer != nullptr; });
    if (captureStems) {
        for (auto* buffer : stemBuffers) {
            if (buffer == nullptr) continue;
            for (int channel = 0; channel < std::min(2, buffer->getNumChannels()); ++channel)
                buffer->clear(channel, startSample, sampleCount);
        }
    }
    const auto writeStem = [&](juce::AudioBuffer<float>* buffer, int sample,
                               float left, float right) {
        if (buffer == nullptr) return;
        buffer->setSample(0, startSample + sample, finiteState(left, 24.0F));
        if (buffer->getNumChannels() > 1)
            buffer->setSample(1, startSample + sample, finiteState(right, 24.0F));
    };
    convolutionBank_.beginBlock(std::min(2, output.getNumChannels()), sampleCount);
    const auto publishedTimeScale = realtimeState_.timeScale.load(std::memory_order_relaxed);
    const auto acousticTimeScale = std::clamp(
        publishedTimeScale > 0.01F ? publishedTimeScale : 1.0F, 0.25F, 4.0F);
    const auto exhaustTemperatureC = realtimeState_.exhaustTemperatureC.load(std::memory_order_relaxed);
    const auto exhaustSoundSpeed = exhaustSoundSpeedMps(exhaustTemperatureC);
    const auto referenceExhaustSoundSpeed = std::clamp(
        realtimeState_.exhaustReferenceSoundSpeedMps.load(std::memory_order_relaxed),
        300.0F, 900.0F);
    // Simulation timestamps are already compressed/expanded by timeScale. Pipe
    // propagation must follow the same clock, while temperature changes its
    // physical wave speed relative to the graph's design-temperature reference.
    const auto acousticDelayScale = referenceExhaustSoundSpeed
        / exhaustSoundSpeed / acousticTimeScale;
    FiringEvent event;
    std::size_t drained = 0;
    double latestPayloadTime = 0.0;
    bool receivedTimestampedPayload = false;
    constexpr std::size_t maximumDrainedPerBlock = 512;
    while (drained < maximumDrainedPerBlock && queue_.tryPop(event)) {
        ++drained;
        latestPayloadTime = std::max(latestPayloadTime, event.timeSeconds);
        receivedTimestampedPayload = true;
        observedExhaustPathCount_ = std::max(observedExhaustPathCount_,
            std::min<std::size_t>(static_cast<std::size_t>(event.exhaustPathIndex) + 1U,
                allocatedPaths_));
        const auto componentCount = std::min<std::size_t>(event.exhaustComponentCount,
            maximumExhaustEventComponents);
        const auto hasLegacyExhaustPulse = componentCount == 0
            && event.exhaustDelaySeconds > 0.00005F;
        const auto proceduralExhaustCount = componentCount > 0 ? componentCount
            : (hasLegacyExhaustPulse ? std::size_t { 1 } : std::size_t { 0 });
        // Firing events still drive the non-exhaust combustion/structure layer.
        // Their exhaust copies are obsolete once an SI boundary is active.
        const auto exhaustCount = physicalExhaustActive_
            ? std::size_t { 0 } : proceduralExhaustCount;
        const auto required = std::size_t { 1 } + exhaustCount;
        if (pendingEventCount_ + required > pendingEvents_.size()) {
            droppedPendingEvents_.fetch_add(required, std::memory_order_relaxed);
            continue;
        }
        auto& direct = pendingEvents_[pendingEventCount_++];
        direct = { event, event.timeSeconds, false };
        if (direct.scheduledTimeSeconds + 1.0 / sampleRate_ < audioTimeSeconds_)
            lateEvents_.fetch_add(1, std::memory_order_relaxed);
        for (std::size_t index = 0; index < exhaustCount; ++index) {
            auto componentEvent = event;
            const auto delaySeconds = componentCount > 0
                ? event.exhaustComponentDelaySeconds[index] : event.exhaustDelaySeconds;
            if (componentCount > 0) {
                componentEvent.exhaustTransmissionGain = event.exhaustComponentGain[index];
                componentEvent.exhaustResonanceHz = event.exhaustComponentResonanceHz[index];
                componentEvent.exhaustPathIndex = event.exhaustComponentPathIndex[index];
                observedExhaustPathCount_ = std::max(observedExhaustPathCount_,
                    std::min<std::size_t>(static_cast<std::size_t>(componentEvent.exhaustPathIndex) + 1U,
                        allocatedPaths_));
            }
            auto& exhaust = pendingEvents_[pendingEventCount_++];
            exhaust = { componentEvent, event.timeSeconds
                + static_cast<double>(delaySeconds * acousticDelayScale), true };
            if (exhaust.scheduledTimeSeconds + 1.0 / sampleRate_ < audioTimeSeconds_)
                lateEvents_.fetch_add(1, std::memory_order_relaxed);
        }
    }
    // Stopping the drain leaves events in the queue that will never be rendered
    // at their scheduled time. Counting them here keeps droppedPendingEventCount()
    // honest: it used to report only the pending-buffer overflow above, so a
    // producer burst past this bound looked like a clean run.
    if (drained >= maximumDrainedPerBlock) {
        std::size_t abandoned = 0;
        while (queue_.tryPop(event)) ++abandoned;
        if (abandoned > 0) droppedPendingEvents_.fetch_add(abandoned, std::memory_order_relaxed);
    }
    if (pressureQueue_) {
        if (!hasCurrentPressureSample_)
            hasCurrentPressureSample_ = pressureQueue_->tryPop(currentPressureSample_);
        if (hasCurrentPressureSample_ && !hasNextPressureSample_)
            hasNextPressureSample_ = pressureQueue_->tryPop(nextPressureSample_);
        if (hasCurrentPressureSample_)
            latestPayloadTime = std::max(latestPayloadTime, currentPressureSample_.timeSeconds);
        if (hasNextPressureSample_)
            latestPayloadTime = std::max(latestPayloadTime, nextPressureSample_.timeSeconds);
        receivedTimestampedPayload = receivedTimestampedPayload
            || hasCurrentPressureSample_ || hasNextPressureSample_;
    }
    const auto publishedProducerNanoseconds = realtimeState_.producerTimeNanoseconds.load(std::memory_order_acquire);
    if (publishedProducerNanoseconds > 0) {
        // The runtime's steady-clock epoch is authoritative. Payload timestamps
        // can extend a few milliseconds into the frame being produced and must
        // not shorten the configured look-ahead latency.
        producerClock_ = static_cast<double>(publishedProducerNanoseconds) * 1.0e-9;
    } else if (receivedTimestampedPayload) {
        producerClock_ = std::max(producerClock_, latestPayloadTime);
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
    // Never jump backwards: on pause/device teardown a stale producer epoch must
    // not replay already-consumed pressure or events. Forward discontinuities
    // (restart/overrun) are safe to catch up immediately.
    if (clockDrift > 0.040) { audioTimeSeconds_ = producerTarget; clockDrift = 0.0; }
    const auto maxClockCorrection = 0.0025 / sampleRate_;
    const auto clockCorrectionPerSample = std::clamp(clockDrift / (0.1 * sampleRate_),
                                                     -maxClockCorrection, maxClockCorrection);
    const auto audioTimeStep = 1.0 / sampleRate_ + clockCorrectionPerSample;
    const auto targetRpm = realtimeState_.rpm.load(std::memory_order_relaxed);
    const auto throttle = realtimeState_.throttle.load(std::memory_order_relaxed);
    const auto load = realtimeState_.load.load(std::memory_order_relaxed);
    const auto stress = realtimeState_.mechanicalStress.load(std::memory_order_relaxed);
    const auto starter = realtimeState_.starter.load(std::memory_order_relaxed);
    const auto timeScale = std::clamp(publishedTimeScale, 0.0F, 4.0F);
    const auto volume = std::clamp(realtimeState_.volume.load(std::memory_order_relaxed), 0.0F, 2.0F);
    const auto convolution = std::clamp(
        realtimeState_.convolution.load(std::memory_order_relaxed), 0.0F, 1.0F);
    const auto highGain = std::clamp(
        realtimeState_.highFrequencyGain.load(std::memory_order_relaxed), 0.2F, 2.5F);
    const auto lowNoise = realtimeState_.lowFrequencyNoise.load(std::memory_order_relaxed);
    const auto highNoise = realtimeState_.highFrequencyNoise.load(std::memory_order_relaxed);
    const auto combustionGain = realtimeState_.combustionGain.load(std::memory_order_relaxed);
    const auto exhaustGain = realtimeState_.exhaustGain.load(std::memory_order_relaxed);
    const auto intakeGain = realtimeState_.intakeGain.load(std::memory_order_relaxed);
    const auto mechanicalGain = realtimeState_.mechanicalGain.load(std::memory_order_relaxed);
    const auto acousticFullScaleSplDb = std::clamp(
        realtimeState_.acousticFullScaleSplDb.load(std::memory_order_relaxed),
        100.0F, 180.0F);
    const auto redline = std::max(500.0F, realtimeState_.redlineRpm.load(std::memory_order_relaxed));
    // Equal pressure does not imply equal acoustic power: the radiating volume
    // velocity grows with cylinder displacement. Square-root scaling preserves
    // energy-normalised summing across cylinders without making large-bore,
    // low-speed engines artificially quiet.
    const auto cylinderDisplacementLitres = std::max(0.03F,
        realtimeState_.cylinderDisplacementLitres.load(std::memory_order_relaxed));
    const auto acousticDisplacementScale = std::clamp(
        std::sqrt(cylinderDisplacementLitres / 0.50F), 0.70F, 1.80F);
    const auto bankSeparation = std::clamp(realtimeState_.bankSeparation.load(std::memory_order_relaxed), 0.0F, 1.0F);
    const auto ambientPressureKpa = std::clamp(
        realtimeState_.ambientPressureKpa.load(std::memory_order_relaxed), 50.0F, 120.0F);
    const auto manifoldPressureKpa = realtimeState_.manifoldPressureKpa.load(std::memory_order_relaxed);
    const auto exhaustPressureKpa = realtimeState_.exhaustPressureKpa.load(std::memory_order_relaxed);
    const auto exhaustFlowGramsPerSecond = realtimeState_.exhaustFlowGramsPerSecond.load(std::memory_order_relaxed);
    const auto intakeDepression = std::clamp((ambientPressureKpa - manifoldPressureKpa) / 75.0F, 0.0F, 1.2F);
    const auto physicalExhaustPressure = std::clamp((exhaustPressureKpa - ambientPressureKpa) / 120.0F, 0.0F, 1.5F);
    const auto physicalExhaustFlow = std::clamp(exhaustFlowGramsPerSecond / 150.0F, 0.0F, 1.8F);
    const auto boostRatio = std::clamp(realtimeState_.boostPressureRatio.load(std::memory_order_relaxed), 1.0F, 3.5F);
    const auto peakPistonAccelG = realtimeState_.peakPistonAccelerationG.load(std::memory_order_relaxed);
    const auto intakeRunnerResonanceHz = realtimeState_.intakeRunnerResonanceHz.load(std::memory_order_relaxed);
    const auto intakeRunnerAmplitudeKpa = realtimeState_.intakeRunnerAmplitudeKpa.load(std::memory_order_relaxed);
    const auto fiShaftRpm = realtimeState_.forcedInductionShaftRpm.load(std::memory_order_relaxed);
    const auto wastegateOpening = std::clamp(realtimeState_.wastegateOpening.load(std::memory_order_relaxed), 0.0F, 1.0F);
    const auto correctedAirFlow = std::max(0.0F, realtimeState_
        .correctedAirFlowKgPerSecond.load(std::memory_order_relaxed));
    const auto compressorPowerWatts = std::max(0.0F, realtimeState_
        .compressorPowerWatts.load(std::memory_order_relaxed));
    const auto turbinePowerWatts = std::max(0.0F, realtimeState_
        .turbinePowerWatts.load(std::memory_order_relaxed));
    const auto blowOffMassFlow = std::max(0.0F, realtimeState_
        .blowOffMassFlowKgPerSecond.load(std::memory_order_relaxed));
    // Only paths that were allocated in prepare() have state to drive. The
    // runtime publishes the same count this was sized from, so clamping here is
    // a guard against a mismatched producer, not a routine narrowing.
    const auto exhaustPathCount = std::clamp<std::size_t>(
        std::max(observedExhaustPathCount_,
            static_cast<std::size_t>(realtimeState_.exhaustPathCount.load(
                std::memory_order_relaxed))),
        1, allocatedPaths_);
    const auto pathCountGain = 1.0F / std::sqrt(static_cast<float>(exhaustPathCount));
    std::array<float, maximumPaths> pathOpenness {};
    std::array<float, maximumPaths> pathGain {};
    std::array<float, maximumPaths> outletAdmittance {};
    std::array<float, maximumPaths> outletAreaM2 {};
    std::array<float, maximumPaths> collectorReflection {};
    for (std::size_t path = 0; path < exhaustPathCount; ++path) {
        pathOpenness[path] = std::clamp(
            realtimeState_.exhaustPathOpenness[path].load(std::memory_order_relaxed), 0.15F, 1.45F);
        pathGain[path] = std::clamp(
            realtimeState_.exhaustPathGain[path].load(std::memory_order_relaxed), 0.0F, 4.0F) * pathCountGain;
        const auto configuredReflection = std::clamp(
            realtimeState_.exhaustPathReflectionSeconds[path].load(std::memory_order_relaxed), 0.001F, 0.080F);
        const auto requestedReflection = static_cast<float>(
            sampleRate_ * configuredReflection * acousticDelayScale);
        const auto reflectionLimit = static_cast<float>(exhaustPaths_[path].forwardWave.size() - 2);
        if (requestedReflection > reflectionLimit)
            delayTruncations_.fetch_add(1, std::memory_order_relaxed);
        // Target only; ramped per sample. See RunnerWaveguides::delayTargetSamples.
        exhaustPaths_[path].reflectionDelayTargetSamples = std::clamp(
            requestedReflection, 1.0F, reflectionLimit);
        if (exhaustPaths_[path].reflectionDelaySamples <= 1.0F)
            exhaustPaths_[path].reflectionDelaySamples =
                exhaustPaths_[path].reflectionDelayTargetSamples;
        const auto publishedArea = realtimeState_.exhaustPathOutletAreaM2[path]
            .load(std::memory_order_relaxed);
        outletAreaM2[path] = std::isfinite(publishedArea) && publishedArea > 1.0e-6F
            ? std::clamp(publishedArea, 1.0e-5F, 0.080F) : 0.0020F;
        outletAdmittance[path] = physicalExhaustActive_
            ? exhaustPaths_[path].outletAcousticAdmittance
            : outletAreaM2[path];
        collectorReflection[path] = std::clamp(
            presetReflection_ * (1.12F - pathOpenness[path] * 0.30F), 0.04F, 0.78F);
        // Expansion-chamber silencer, refit per block from published geometry.
        // The chamber's own traversal is stretched by the same acoustic scale
        // as every other physical delay, so a cold exhaust or a slowed-down
        // audition moves its comb spacing with the rest of the network.
        {
            auto& muffler = exhaustPaths_[path].mufflerCoefficients;
            const auto expansionRatio = realtimeState_
                .exhaustPathMufflerExpansionRatio[path].load(std::memory_order_relaxed);
            const auto traversalSeconds = realtimeState_
                .exhaustPathMufflerTraversalSeconds[path].load(std::memory_order_relaxed);
            const auto requested = static_cast<float>(
                sampleRate_ * static_cast<double>(traversalSeconds) * acousticDelayScale);
            const auto limit = static_cast<float>(exhaustPaths_[path].muffler
                .towardOutlet.size() - 2U);
            muffler.enabled = std::isfinite(expansionRatio) && expansionRatio > 0.05F
                && std::isfinite(traversalSeconds) && requested >= 1.0F && limit >= 1.0F;
            if (muffler.enabled) {
                if (requested > limit) delayTruncations_.fetch_add(1, std::memory_order_relaxed);
                // Areas enter the junction only as their ratio, so the duct is
                // taken as unit area and the chamber as the published ratio.
                muffler.reflection = ExpansionChamberMuffler::reflectionCoefficient(
                    1.0, static_cast<double>(expansionRatio));
                // Target only; ramped per sample. Stepping this at the block
                // boundary phase-jumps the chamber lines every block.
                exhaustPaths_[path].mufflerDelayTargetSamples =
                    std::clamp(requested, 1.0F, limit);
                if (muffler.delaySamples < 1.0F)
                    muffler.delaySamples = exhaustPaths_[path].mufflerDelayTargetSamples;
            } else {
                muffler = ExpansionChamberMuffler::Coefficients {};
                exhaustPaths_[path].mufflerDelayTargetSamples = 0.0F;
            }
        }
        // Refit the collector-to-outlet wall loss once per block. The gas state
        // moves on a far slower timescale than a block, and the fit needs
        // transcendentals that have no place in the per-sample loop.
        if (physicalExhaustActive_) {
            auto& state = exhaustPaths_[path];
            state.wallLoss = DuctWallLoss::fit(
                static_cast<double>(state.reflectionDelaySamples) / sampleRate_,
                std::sqrt(static_cast<double>(outletAreaM2[path]) / std::numbers::pi),
                static_cast<double>(state.mediumDensityKgPerM3),
                static_cast<double>(state.mediumSoundSpeedMps),
                sampleRate_);
        }
    }
    const auto fdnScale = sampleRate_ / referenceSampleRate * acousticDelayScale;
    for (std::size_t line = 0; line < referenceFdnSamples.size(); ++line) {
        const auto requested = static_cast<std::size_t>(
            std::llround(referenceFdnSamples[line] * fdnScale));
        const auto limit = exhaustPaths_.front().fdn[line].size();
        if (requested > limit) delayTruncations_.fetch_add(1, std::memory_order_relaxed);
        fdnDelaySamples_[line] = std::clamp<std::size_t>(requested, 1, limit);
    }
    std::array<float, maxRunners> runnerAdmittance {};
    std::array<float, maxRunners> runnerAreaM2 {};
    std::array<float, maxRunners> portReflection {};
    portReflection.fill(0.92F);
    const auto runnerDelayLimit = static_cast<float>(runners_->forward.stride - 2);
    for (std::size_t runner = 0; runner < allocatedRunners_; ++runner) {
        const auto seconds = realtimeState_.runnerDelaySeconds[runner].load(std::memory_order_relaxed);
        const auto exactDelaySamples = std::isfinite(seconds)
            ? seconds * acousticDelayScale * static_cast<float>(sampleRate_) : 1.0F;
        if (exactDelaySamples > runnerDelayLimit)
            delayTruncations_.fetch_add(1, std::memory_order_relaxed);
        // Target only; the per-sample ramp lives in processExhaustWaveguides.
        // See RunnerWaveguides::delayTargetSamples. A line that has not yet
        // carried a wave snaps, so startup incurs no sweep from zero.
        runners_->delayTargetSamples[runner] = std::clamp(
            exactDelaySamples, 1.0F, runnerDelayLimit);
        if (runners_->delaySamples[runner] < 1.0F)
            runners_->delaySamples[runner] = runners_->delayTargetSamples[runner];
        const auto publishedArea = realtimeState_.cylinderExhaustAreaM2[runner]
            .load(std::memory_order_relaxed);
        runnerAreaM2[runner] = std::isfinite(publishedArea) && publishedArea > 1.0e-6F
            ? std::clamp(publishedArea, 1.0e-5F, 0.040F) : 0.00125F;
        if (physicalExhaustActive_) {
            const auto cachedAdmittance = thermoacousticRunnerAdmittance_[runner];
            runnerAdmittance[runner] = cachedAdmittance > 0.0F
                ? cachedAdmittance
                : runnerAreaM2[runner] / static_cast<float>(1.2 * ambientSoundSpeedMps);
            portReflection[runner] = thermoacousticPortReflection_[runner];
            // Refit the runner wall loss once per block, from the port's own gas
            // state. A runner is narrow and hot, so its boundary-layer loss is
            // the strongest damping any high mode in the network sees.
            const auto& boundary = portBoundary_[runner];
            if (boundary.physical)
                runnerWallLoss_[runner] = DuctWallLoss::fit(
                    static_cast<double>(runners_->delaySamples[runner]) / sampleRate_,
                    std::sqrt(static_cast<double>(runnerAreaM2[runner]) / std::numbers::pi),
                    static_cast<double>(boundary.densityKgPerM3),
                    boundary.characteristicImpedancePaSPerM3 > 0.0F
                        && boundary.densityKgPerM3 > 0.0F
                        ? static_cast<double>(boundary.characteristicImpedancePaSPerM3)
                            * static_cast<double>(runnerAreaM2[runner])
                            / static_cast<double>(boundary.densityKgPerM3)
                        : 343.0,
                    sampleRate_);
        } else {
            // Legacy scattering used area-proportional weights because it did
            // not carry a medium state. It remains isolated to that path.
            runnerAdmittance[runner] = runnerAreaM2[runner];
        }
    }
    std::array<float, maxRunners> cylinderExhaustGain {};
    for (std::size_t runner = 0; runner < allocatedRunners_; ++runner) {
        cylinderExhaustGain[runner] = std::clamp(
            realtimeState_.cylinderExhaustGain[runner].load(std::memory_order_relaxed), 0.0F, 8.0F);
    }
    if (acousticExhaustNetwork_) {
        std::array<AcousticExhaustNetwork::Medium, maximumPaths> media {};
        std::array<float, maximumPaths> meanMassFlow {};
        for (std::size_t path = 0; path < exhaustPathCount; ++path) {
            media[path] = {
                exhaustPaths_[path].mediumDensityKgPerM3,
                exhaustPaths_[path].mediumSoundSpeedMps,
            };
            meanMassFlow[path] = exhaustPaths_[path].meanExhaustMassFlowKgPerSecond;
        }
        // Per-duct gas state where the solver published it. The path medium
        // above is sampled at the valve, the hottest point in the system;
        // applying it to a tailpipe several hundred kelvin cooler ran that duct
        // 20-25% fast and moved its resonances with it. Held between coupling
        // flushes and slewed per sample inside the network, so the step at each
        // flush never reaches a scattering coefficient.
        std::array<AcousticExhaustNetwork::Medium,
                   CylinderPressureSample::maximumExhaustDucts> ductMedia {};
        auto ductMediumCount = std::size_t { 0 };
        if (hasCurrentPressureSample_) {
            ductMediumCount = std::min(currentPressureSample_.exhaustDuctCount,
                                       ductMedia.size());
            for (std::size_t duct = 0; duct < ductMediumCount; ++duct)
                ductMedia[duct] = {
                    currentPressureSample_.exhaustDuctDensityKgPerM3[duct],
                    currentPressureSample_.exhaustDuctSpeedOfSoundMps[duct],
                };
        }
        acousticExhaustNetwork_->beginBlock(
            std::span<const AcousticExhaustNetwork::Medium>(
                media.data(), exhaustPathCount),
            acousticTimeScale,
            std::span<const float>(meanMassFlow.data(), exhaustPathCount),
            std::span<const AcousticExhaustNetwork::Medium>(
                ductMedia.data(), ductMediumCount));
    }
    if (acousticIntakeNetwork_) {
        std::array<AcousticIntakeNetwork::PathBoundary, maximumPaths> paths {};
        std::array<float, maximumPaths> densitySum {};
        std::array<float, maximumPaths> soundSpeedSum {};
        std::array<float, maximumPaths> weights {};
        if (hasCurrentPressureSample_) {
            const auto count = std::min<std::size_t>(
                currentPressureSample_.cylinderCount,
                currentPressureSample_.intakePathIndex.size());
            for (std::size_t cylinder = 0; cylinder < count; ++cylinder) {
                const auto path = std::min<std::size_t>(
                    currentPressureSample_.intakePathIndex[cylinder], maximumPaths - 1U);
                const auto density = currentPressureSample_
                    .intakeRunnerDensityKgPerM3[cylinder];
                const auto soundSpeed = currentPressureSample_
                    .intakeRunnerSpeedOfSoundMps[cylinder];
                if (std::isfinite(density) && density > 0.0F
                    && std::isfinite(soundSpeed) && soundSpeed > 0.0F) {
                    densitySum[path] += density;
                    soundSpeedSum[path] += soundSpeed;
                    weights[path] += 1.0F;
                }
            }
            for (std::size_t path = 0; path < maximumPaths; ++path) {
                if (weights[path] > 0.0F) {
                    paths[path].densityKgPerM3 = densitySum[path] / weights[path];
                    paths[path].soundSpeedMps = soundSpeedSum[path] / weights[path];
                }
                if (path < currentPressureSample_.intakePathCount)
                    paths[path].throttleConductanceAreaM2 = currentPressureSample_
                        .intakeThrottleConductanceAreaM2[path];
            }
        }
        acousticIntakeNetwork_->beginBlock(paths, acousticTimeScale);
    }
    float blockPeakObservedExhaustPressurePa = 0.0F;
    float blockPeakObservedExhaustJetNoisePressurePa = 0.0F;
    float blockPeakObservedIntakePressurePa = 0.0F;
    float blockPeakObservedStructuralPressurePa = 0.0F;
    std::uint64_t legacySamplesThisBlock = 0;
    std::uint64_t invalidBoundarySamplesThisBlock = 0;
    for (int sample = 0; sample < sampleCount; ++sample) {
        const auto preset = realtimeState_.exhaustPreset.load(std::memory_order_relaxed);
        if (preset != activeExhaustPreset_) updateExhaustPreset(preset);
        for (std::size_t index = 0; index < pendingEventCount_;) {
            if (pendingEvents_[index].scheduledTimeSeconds <= audioTimeSeconds_ + 0.5 / sampleRate_) {
                // Once the physical path owns the voice, no oscillator voice of
                // either kind is started. See activatePhysicalExhaust().
                if (!acousticExhaustNetwork_)
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
        float physicalStructural = 0.0F;
        float physicalIntakeLeft = 0.0F;
        float physicalIntakeRight = 0.0F;
        float physicalForcedInduction = 0.0F;
        float physicalCylinderPressureLeft = 0.0F;
        float physicalCylinderPressureRight = 0.0F;
        std::array<float, maximumPaths> pathCollectorLeft {};
        std::array<float, maximumPaths> pathCollectorRight {};
        std::array<float, maximumPaths> pathExhaustLeft {};
        std::array<float, maximumPaths> pathExhaustRight {};
        std::array<float, maximumPaths> physicalBlowdownLeft {};
        std::array<float, maximumPaths> physicalBlowdownRight {};
        std::array<float, 32> cylinderExhaustPulse {};
        std::array<std::uint8_t, 32> cylinderExhaustPath {};
        std::array<float, maximumPaths> pathDensitySum {};
        std::array<float, maximumPaths> pathSoundSpeedSum {};
        std::array<float, maximumPaths> pathMediumWeight {};
        std::array<float, maximumPaths> pathMeanMassFlowSum {};
        StructuralExcitationSample structuralExcitation;
        std::array<AcousticIntakeNetwork::CylinderBoundary, 32> intakeBoundaries {};
        // Once established, the SI path is latched. A missing producer sample
        // lets the passive network ring down; it must never resurrect noise and
        // oscillators for a callback and hide the telemetry dropout.
        // A compiled SI topology owns production output from the first sample.
        // Before its first boundary arrives it propagates silence; it must not
        // briefly expose the procedural voice and then switch timbre at startup.
        auto sampleUsesPhysicalExhaust = acousticExhaustNetwork_ != nullptr
            || physicalExhaustActive_.load(std::memory_order_relaxed);
        std::size_t activeCylinderCount = 0;
        if (pressureQueue_ && hasCurrentPressureSample_) {
            const auto pressureTime = audioTimeSeconds_;
            while (hasNextPressureSample_ && nextPressureSample_.timeSeconds <= pressureTime) {
                currentPressureSample_ = nextPressureSample_;
                hasNextPressureSample_ = pressureQueue_->tryPop(nextPressureSample_);
            }
            if (!hasNextPressureSample_)
                hasNextPressureSample_ = pressureQueue_->tryPop(nextPressureSample_);
            const auto count = std::min<std::size_t>(currentPressureSample_.cylinderCount,
                currentPressureSample_.pressureBar.size());
            if (count > 0) {
                activeCylinderCount = count;
                structuralExcitation.cylinderCount = count;
                const auto denominator = hasNextPressureSample_
                    ? nextPressureSample_.timeSeconds - currentPressureSample_.timeSeconds : 0.0;
                auto valveFlowSourceRateChanged = false;
                if (denominator > 1.0e-7
                    && std::abs(denominator - pressureSampleIntervalSeconds_) > 1.0e-9) {
                    pressureSampleIntervalSeconds_ = denominator;
                    // The solver publishes an adaptive cadence.  Restrict the
                    // reconstructed pressure bandwidth to its actual Nyquist
                    // region instead of applying a fixed 9.5 kHz cutoff to a
                    // potentially 2-4 kHz source stream.
                    const auto telemetryRate = 1.0 / denominator;
                    valveFlowSourceRateChanged = std::abs(telemetryRate
                        - valveFlowSourceSamplingHz_) > 0.01 * std::max(
                            telemetryRate, valveFlowSourceSamplingHz_);
                    const auto cutoffHz = std::clamp(telemetryRate * 0.42, 180.0,
                        std::min(9'500.0, sampleRate_ * 0.38));
                    pressureBandCoefficient_ = static_cast<float>(1.0 - std::exp(
                        -2.0 * std::numbers::pi * cutoffHz / sampleRate_));
                    valveFlowSourceSamplingHz_ = telemetryRate;
                }
                // Track the producer's boundary sampling rate and refit the
                // anti-imaging reconstruction when it moves by more than 1%.
                // The rate follows engine speed, so refits are rare relative to
                // the audio rate and the transcendental cost is amortised away.
                const auto couplingHz = currentPressureSample_.exhaustCouplingFrequencyHz;
                if (std::abs(couplingHz - boundaryReconstructionCouplingHz_)
                        > 0.01 * std::max(couplingHz, boundaryReconstructionCouplingHz_)
                    || valveFlowSourceRateChanged) {
                    boundaryReconstruction_ = BoundaryReconstructionFilter::compute(
                        couplingHz, sampleRate_);
                    valveFlowAcousticSource_ = ValveFlowAcousticSource::compute(
                        couplingHz, sampleRate_, valveFlowSourceSamplingHz_);
                    boundaryReconstructionCouplingHz_ = couplingHz;
                }
                const auto fraction = denominator > 1.0e-9
                    ? std::clamp((pressureTime - currentPressureSample_.timeSeconds) / denominator, 0.0, 1.0)
                    : 0.0;
                // Adaptive solver samples are not equally spaced. Linear
                // interpolation is monotonic and cannot manufacture pressure
                // overshoot when a substep cadence changes or a queue sample was
                // deliberately rate-limited by the runtime.
                const auto f = static_cast<float>(fraction);
                for (std::size_t index = 0; index < count; ++index) {
                    const auto nextPressure = hasNextPressureSample_
                        && index < nextPressureSample_.cylinderCount
                        ? nextPressureSample_.pressureBar[index]
                        : currentPressureSample_.pressureBar[index];
                    const auto pressureBar = std::lerp(currentPressureSample_.pressureBar[index],
                                                       nextPressure, f);
                    const auto interpolateStructural = [&, index](const auto& member) {
                        const auto current = member(currentPressureSample_.structural)[index];
                        const auto next = hasNextPressureSample_
                            && index < nextPressureSample_.structural.cylinderCount
                            ? member(nextPressureSample_.structural)[index] : current;
                        return std::lerp(current, next, f);
                    };
                    structuralExcitation.gasForceN[index] = interpolateStructural(
                        [](const auto& value) -> const auto& { return value.gasForceN; });
                    structuralExcitation.inertiaForceN[index] = interpolateStructural(
                        [](const auto& value) -> const auto& { return value.inertiaForceN; });
                    structuralExcitation.bearingReactionForceN[index] = interpolateStructural(
                        [](const auto& value) -> const auto& { return value.bearingReactionForceN; });
                    structuralExcitation.sideThrustForceN[index] = interpolateStructural(
                        [](const auto& value) -> const auto& { return value.sideThrustForceN; });
                    structuralExcitation.crankReactionTorqueNm[index] = interpolateStructural(
                        [](const auto& value) -> const auto& { return value.crankReactionTorqueNm; });
                    const auto interpolatePressureField = [&, index](const auto& member) {
                        const auto current = member(currentPressureSample_)[index];
                        const auto next = hasNextPressureSample_
                            && index < nextPressureSample_.cylinderCount
                            ? member(nextPressureSample_)[index] : current;
                        return std::lerp(current, next, f);
                    };
                    auto& intakeBoundary = intakeBoundaries[index];
                    intakeBoundary.massFlowKgPerSecond = interpolatePressureField(
                        [](const auto& value) -> const auto& {
                            return value.intakeMassFlowKgPerSecond;
                        });
                    intakeBoundary.conductanceAreaM2 = interpolatePressureField(
                        [](const auto& value) -> const auto& {
                            return value.intakeValveConductanceAreaM2;
                        });
                    intakeBoundary.densityKgPerM3 = interpolatePressureField(
                        [](const auto& value) -> const auto& {
                            return value.intakeRunnerDensityKgPerM3;
                        });
                    intakeBoundary.soundSpeedMps = interpolatePressureField(
                        [](const auto& value) -> const auto& {
                            return value.intakeRunnerSpeedOfSoundMps;
                        });
                    intakeBoundary.physical = std::isfinite(
                        intakeBoundary.massFlowKgPerSecond)
                        && intakeBoundary.densityKgPerM3 > 0.0F
                        && intakeBoundary.soundSpeedMps > 0.0F;
                    const auto rawGaugePressure = pressureBar - ambientPressureKpa * 0.01F;
                    cylinderPressureHighPass_[index] = pressureHighPassPole_
                        * (cylinderPressureHighPass_[index] + rawGaugePressure
                            - cylinderPressureRawPrevious_[index]);
                    cylinderPressureRawPrevious_[index] = rawGaugePressure;
                    const auto derivative = cylinderPressureHighPass_[index]
                        - cylinderPressureHighPassPrevious_[index];
                    cylinderPressureHighPassPrevious_[index] = cylinderPressureHighPass_[index];
                    const auto pressureTarget = cylinderPressureHighPass_[index] * 0.0065F
                        + derivative * static_cast<float>(sampleRate_ / referenceSampleRate) * 0.075F;
                    cylinderPressureBandLimited_[index] += pressureBandCoefficient_
                        * (pressureTarget - cylinderPressureBandLimited_[index]);
                    // Structure-borne combustion noise is deliberately absent
                    // from the physical path.
                    //
                    // The quantity below is a weighted blend of chamber gauge
                    // pressure and its derivative, with coefficients that carry
                    // no units and no derivation. There is no transfer path in
                    // it from chamber pressure to observer pressure: no bore
                    // force, no radiating area, no block or head compliance, no
                    // modal response. It sounded like combustion because it was
                    // shaped to, which is exactly the kind of perceptual
                    // rendering this path is not allowed to present as physics.
                    //
                    // Reproducing that content honestly needs a reduced modal
                    // model of the block and head driven by piston and bearing
                    // forces, which is a separate milestone and is not
                    // implemented. Until it exists, the observer hears the
                    // exhaust radiation the network actually computes and
                    // nothing standing in for the structure.
                    const auto pan = std::clamp(
                        realtimeState_.cylinderPan[index].load(std::memory_order_relaxed),
                        -0.82F, 0.82F);
                    if (!acousticExhaustNetwork_) {
                        const auto legacyPressure = finiteState(
                            cylinderPressureBandLimited_[index], 4.0F)
                            / std::sqrt(static_cast<float>(count));
                        physicalCylinderPressureLeft += legacyPressure
                            * std::sqrt((1.0F - pan) * 0.5F);
                        physicalCylinderPressureRight += legacyPressure
                            * std::sqrt((1.0F + pan) * 0.5F);
                    }
                    const auto configuredPath = realtimeState_.cylinderExhaustPathIndex[index]
                        .load(std::memory_order_relaxed);
                    const auto samplePath = static_cast<std::size_t>(
                        currentPressureSample_.exhaustPathIndex[index]);
                    const auto resolvedPath = samplePath < exhaustPathCount
                        ? samplePath : std::min<std::size_t>(configuredPath, exhaustPathCount - 1U);
                    cylinderExhaustPath[index] = static_cast<std::uint8_t>(resolvedPath);
                    const auto nextExhaustPressure = hasNextPressureSample_
                        && index < nextPressureSample_.cylinderCount
                        ? nextPressureSample_.exhaustRunnerPressureKpa[index]
                        : currentPressureSample_.exhaustRunnerPressureKpa[index];
                    const auto runnerExhaustPressureKpa = std::lerp(
                        currentPressureSample_.exhaustRunnerPressureKpa[index], nextExhaustPressure, f);
                    const auto nextBoundaryIsValid = !hasNextPressureSample_
                        || index >= nextPressureSample_.cylinderCount
                        || nextPressureSample_.thermoacousticBoundaryValid[index] != 0;
                    auto physicalBoundaryIsValid =
                        currentPressureSample_.thermoacousticBoundaryValid[index] != 0
                        && nextBoundaryIsValid;
                    const auto nextMassFlow = hasNextPressureSample_
                        && index < nextPressureSample_.cylinderCount
                        ? nextPressureSample_.exhaustMassFlowKgPerSecond[index]
                        : currentPressureSample_.exhaustMassFlowKgPerSecond[index];
                    const auto nextDensity = hasNextPressureSample_
                        && index < nextPressureSample_.cylinderCount
                        ? nextPressureSample_.exhaustPortDensityKgPerM3[index]
                        : currentPressureSample_.exhaustPortDensityKgPerM3[index];
                    const auto nextSoundSpeed = hasNextPressureSample_
                        && index < nextPressureSample_.cylinderCount
                        ? nextPressureSample_.exhaustPortSpeedOfSoundMps[index]
                        : currentPressureSample_.exhaustPortSpeedOfSoundMps[index];
                    const auto nextValveArea = hasNextPressureSample_
                        && index < nextPressureSample_.cylinderCount
                        ? nextPressureSample_.exhaustValveConductanceAreaM2[index]
                        : currentPressureSample_.exhaustValveConductanceAreaM2[index];
                    const auto massFlowKgPerSecond = std::lerp(
                        currentPressureSample_.exhaustMassFlowKgPerSecond[index], nextMassFlow, f);
                    const auto nextAcousticMassFlow = hasNextPressureSample_
                        && index < nextPressureSample_.cylinderCount
                        ? nextPressureSample_.exhaustAcousticMassFlowKgPerSecond[index]
                        : currentPressureSample_.exhaustAcousticMassFlowKgPerSecond[index];
                    // Partner of the runner pressure below. Both are reconstructed
                    // from the same exhaust network knots at the same phase, which
                    // is what makes the characteristic split valid.
                    const auto acousticMassFlowKgPerSecond = std::lerp(
                        currentPressureSample_.exhaustAcousticMassFlowKgPerSecond[index],
                        nextAcousticMassFlow, f);
                    const auto densityKgPerM3 = std::lerp(
                        currentPressureSample_.exhaustPortDensityKgPerM3[index], nextDensity, f);
                    const auto soundSpeedMps = std::lerp(
                        currentPressureSample_.exhaustPortSpeedOfSoundMps[index], nextSoundSpeed, f);
                    const auto valveConductanceAreaM2 = std::lerp(
                        currentPressureSample_.exhaustValveConductanceAreaM2[index], nextValveArea, f);
                    physicalBoundaryIsValid = physicalBoundaryIsValid
                        && std::isfinite(runnerExhaustPressureKpa)
                        && std::isfinite(massFlowKgPerSecond)
                        && std::isfinite(acousticMassFlowKgPerSecond)
                        && std::isfinite(densityKgPerM3) && densityKgPerM3 > 0.0F
                        && std::isfinite(soundSpeedMps) && soundSpeedMps > 0.0F;
                    if (physicalBoundaryIsValid) {
                        if (!physicalExhaustActive_) {
                            activatePhysicalExhaust();
                            cylinderExhaustPulse.fill(0.0F);
                            physicalBlowdownLeft.fill(0.0F);
                            physicalBlowdownRight.fill(0.0F);
                            for (std::size_t runner = 0; runner < allocatedRunners_; ++runner) {
                                runnerAdmittance[runner] = runnerAreaM2[runner]
                                    / static_cast<float>(1.2 * ambientSoundSpeedMps);
                                portReflection[runner] = 0.999F;
                            }
                            for (std::size_t path = 0; path < exhaustPathCount; ++path)
                                outletAdmittance[path] = exhaustPaths_[path]
                                    .outletAcousticAdmittance;
                        }
                        const auto pressurePa = runnerExhaustPressureKpa * 1'000.0F;
                        // The mean is subtracted from each partner to leave the
                        // acoustic perturbation, so it has to track the same
                        // signal the perturbation is taken from.
                        if (!thermoacousticMeanInitialised_[index]) {
                            exhaustMeanPressurePa_[index] = pressurePa;
                            exhaustMeanMassFlowKgPerSecond_[index] = acousticMassFlowKgPerSecond;
                            thermoacousticMeanInitialised_[index] = true;
                        } else {
                            exhaustMeanPressurePa_[index] += thermoacousticMeanCoefficient_
                                * (pressurePa - exhaustMeanPressurePa_[index]);
                            exhaustMeanMassFlowKgPerSecond_[index] +=
                                thermoacousticMeanCoefficient_ * (acousticMassFlowKgPerSecond
                                    - exhaustMeanMassFlowKgPerSecond_[index]);
                        }
                        // Both characteristic partners pass through the same
                        // reconstruction low-pass, so their ratio -- and with
                        // it the characteristic split -- is untouched inside
                        // the physical band. Everything removed lies above the
                        // producer's sampling Nyquist and cannot be signal.
                        const auto pressurePerturbationPa = static_cast<float>(
                            BoundaryReconstructionFilter::process(
                                boundaryReconstruction_,
                                boundaryReconstructionPressure_[index],
                                pressurePa - exhaustMeanPressurePa_[index]));
                        const auto massFlowPerturbationKgPerSecond = static_cast<float>(
                            BoundaryReconstructionFilter::process(
                                boundaryReconstruction_,
                                boundaryReconstructionFlow_[index],
                                acousticMassFlowKgPerSecond
                                    - exhaustMeanMassFlowKgPerSecond_[index]));
                        const auto areaM2 = runnerAreaM2[index];
                        const auto characteristicImpedance = densityKgPerM3
                            * soundSpeedMps / areaM2;
                        const auto volumeVelocityPerturbation =
                            massFlowPerturbationKgPerSecond / densityKgPerM3;
                        const auto highBandSource = ValveFlowAcousticSource::process(
                            valveFlowAcousticSource_,
                            valveFlowAcousticSourceState_[index],
                            massFlowKgPerSecond, densityKgPerM3,
                            characteristicImpedance);
                        const auto outgoingMeasured = 0.5F * (pressurePerturbationPa
                            + characteristicImpedance * volumeVelocityPerturbation)
                            + static_cast<float>(highBandSource.outgoingPressurePa);
                        const auto incomingMeasured = 0.5F * (pressurePerturbationPa
                            - characteristicImpedance * volumeVelocityPerturbation)
                            + static_cast<float>(highBandSource.incomingPressurePa);

                        // Publish the valve state; the termination itself is
                        // evaluated inside the waveguide, where the acoustic
                        // velocity through the orifice that sets its resistance
                        // is known. See ValvePortTermination.
                        auto& boundary = portBoundary_[index];
                        boundary.conductanceAreaM2 = valveConductanceAreaM2;
                        boundary.meanMassFlowKgPerSecond = massFlowKgPerSecond;
                        boundary.densityKgPerM3 = densityKgPerM3;
                        boundary.characteristicImpedancePaSPerM3 = characteristicImpedance;
                        boundary.physical = true;

                        // Separate the measured boundary into the part the
                        // cylinder is driving and the part that is merely the
                        // runner's own returning wave reflecting off the port,
                        // so the waveguide is not fed its own reflection twice.
                        // The mean-flow term alone is used here: this filter sees
                        // the telemetry stream, not the in-runner wave.
                        const auto sourceCoefficients = ValvePortTermination::compute(
                            valveConductanceAreaM2, massFlowKgPerSecond, 0.0,
                            densityKgPerM3, characteristicImpedance, sampleRate_);
                        const auto reflectedIncoming = ValvePortTermination::process(
                            sourceCoefficients, portSourceState_[index], incomingMeasured);
                        thermoacousticPortReflection_[index] = static_cast<float>(
                            sourceCoefficients.b0);
                        cylinderExhaustPulse[index] = std::isfinite(outgoingMeasured)
                            ? outgoingMeasured - reflectedIncoming : 0.0F;
                        runnerAdmittance[index] = areaM2
                            / (densityKgPerM3 * soundSpeedMps);
                        thermoacousticRunnerAdmittance_[index] = runnerAdmittance[index];
                        runnerStiffnessRhoC2_[index] = densityKgPerM3
                            * soundSpeedMps * soundSpeedMps;
                        pathDensitySum[resolvedPath] += densityKgPerM3 * areaM2;
                        pathSoundSpeedSum[resolvedPath] += soundSpeedMps * areaM2;
                        pathMediumWeight[resolvedPath] += areaM2;
                        // Total mean outflow leaving this path (all its ports),
                        // for the outlet mean-flow convective loss.
                        pathMeanMassFlowSum[resolvedPath] += std::abs(
                            exhaustMeanMassFlowKgPerSecond_[index]);
                        sampleUsesPhysicalExhaust = true;
                    } else if (!acousticExhaustNetwork_) {
                        const auto exhaustGaugePressure = runnerExhaustPressureKpa
                            - ambientPressureKpa;
                        exhaustPressureHighPass_[index] = pressureHighPassPole_
                            * (exhaustPressureHighPass_[index] + exhaustGaugePressure
                                - exhaustPressureRawPrevious_[index]);
                        exhaustPressureRawPrevious_[index] = exhaustGaugePressure;
                        const auto exhaustDerivative = exhaustPressureHighPass_[index]
                            - exhaustPressureHighPassPrevious_[index];
                        exhaustPressureHighPassPrevious_[index] = exhaustPressureHighPass_[index];
                        const auto exhaustFlow = std::lerp(
                            currentPressureSample_.exhaustFlowMgPerCycle[index],
                            hasNextPressureSample_ && index < nextPressureSample_.cylinderCount
                                ? nextPressureSample_.exhaustFlowMgPerCycle[index]
                                : currentPressureSample_.exhaustFlowMgPerCycle[index], f);
                        const auto flowGain = std::clamp(exhaustFlow / 75.0F, 0.08F, 1.8F);
                        const auto exhaustTarget = (exhaustPressureHighPass_[index] * 0.0038F
                            + exhaustDerivative
                                * static_cast<float>(sampleRate_ / referenceSampleRate) * 0.028F)
                            * flowGain;
                        exhaustPressureBandLimited_[index] += pressureBandCoefficient_
                            * (exhaustTarget - exhaustPressureBandLimited_[index]);
                        cylinderExhaustPulse[index] = finiteState(
                            exhaustPressureBandLimited_[index] * cylinderExhaustGain[index], 6.0F);
                        const auto nextValveOpening = hasNextPressureSample_
                            && index < nextPressureSample_.cylinderCount
                            ? nextPressureSample_.exhaustValveOpening[index]
                            : currentPressureSample_.exhaustValveOpening[index];
                        const auto valveOpening = std::clamp(std::lerp(
                            currentPressureSample_.exhaustValveOpening[index],
                            nextValveOpening, f), 0.0F, 1.0F);
                        portReflection[index] = 0.92F - 0.70F * std::sqrt(valveOpening);
                        const auto physicalBlowdown = cylinderExhaustPulse[index]
                            / std::sqrt(static_cast<float>(count));
                        physicalBlowdownLeft[resolvedPath] += physicalBlowdown
                            * std::sqrt((1.0F - pan) * 0.5F);
                        physicalBlowdownRight[resolvedPath] += physicalBlowdown
                            * std::sqrt((1.0F + pan) * 0.5F);
                    }
                }
            }
        }
        if (structuralModalRadiator_ && structuralExcitation.cylinderCount > 0) {
            const auto pressurePa = structuralModalRadiator_->process(
                structuralExcitation);
            blockPeakObservedStructuralPressurePa = std::max(
                blockPeakObservedStructuralPressurePa, std::abs(pressurePa));
            physicalStructural = static_cast<float>(
                AcousticMonitorCalibration::normalisePeakPressure(
                    pressurePa, acousticFullScaleSplDb));
        }
        if (acousticIntakeNetwork_ && activeCylinderCount > 0) {
            const auto inletPressure = acousticIntakeNetwork_->process(
                std::span<const AcousticIntakeNetwork::CylinderBoundary>(
                    intakeBoundaries.data(), activeCylinderCount),
                controlRampCoefficient_);
            for (const auto pressure : inletPressure) {
                blockPeakObservedIntakePressurePa = std::max(
                    blockPeakObservedIntakePressurePa,
                    std::max(std::abs(pressure.leftPa),
                        std::abs(pressure.rightPa)));
                physicalIntakeLeft += static_cast<float>(
                    AcousticMonitorCalibration::normalisePeakPressure(
                        pressure.leftPa, acousticFullScaleSplDb));
                physicalIntakeRight += static_cast<float>(
                    AcousticMonitorCalibration::normalisePeakPressure(
                        pressure.rightPa, acousticFullScaleSplDb));
            }
        }
        if (forcedInductionAcoustics_) {
            auto density = 1.2F;
            auto soundSpeed = static_cast<float>(ambientSoundSpeedMps);
            auto mediumCount = 0.0F;
            auto densitySum = 0.0F;
            auto soundSpeedSum = 0.0F;
            for (std::size_t cylinder = 0;
                 cylinder < activeCylinderCount; ++cylinder) {
                const auto& boundary = intakeBoundaries[cylinder];
                if (boundary.physical) {
                    densitySum += boundary.densityKgPerM3;
                    soundSpeedSum += boundary.soundSpeedMps;
                    mediumCount += 1.0F;
                }
            }
            if (mediumCount > 0.0F) {
                density = densitySum / mediumCount;
                soundSpeed = soundSpeedSum / mediumCount;
            }
            const ForcedInductionAcoustics::Input input {
                .shaftSpeedRpm = fiShaftRpm,
                .correctedAirFlowKgPerSecond = correctedAirFlow,
                .pressureRatio = boostRatio,
                .compressorPowerWatts = compressorPowerWatts,
                .turbinePowerWatts = turbinePowerWatts,
                .exhaustMassFlowKgPerSecond = std::max(
                    0.0F, exhaustFlowGramsPerSecond * 0.001F),
                .wastegateOpening = wastegateOpening,
                .blowOffMassFlowKgPerSecond = blowOffMassFlow,
                .densityKgPerM3 = density,
                .soundSpeedMps = soundSpeed,
                .acousticTimeScale = acousticTimeScale
            };
            physicalForcedInduction = static_cast<float>(
                AcousticMonitorCalibration::normalisePeakPressure(
                    forcedInductionAcoustics_->process(input),
                    acousticFullScaleSplDb));
        }
        combustionLeft += physicalCylinderPressureLeft;
        combustionRight += physicalCylinderPressureRight;
        // Runner waveguide + collector scattering junction: the collector output
        // carries cross-talk between cylinders and runner-length tuning. A small
        // direct (pre-collector) component adds stereo width from cylinder pan.
        if (sampleUsesPhysicalExhaust) {
            for (std::size_t path = 0; path < exhaustPathCount; ++path) {
                if (pathMediumWeight[path] <= 0.0F) continue;
                const auto density = pathDensitySum[path] / pathMediumWeight[path];
                const auto soundSpeed = pathSoundSpeedSum[path] / pathMediumWeight[path];
                outletAdmittance[path] = outletAreaM2[path] / (density * soundSpeed);
                exhaustPaths_[path].outletAcousticAdmittance = outletAdmittance[path];
                // Cached for the next block's wall-loss fit.
                exhaustPaths_[path].mediumDensityKgPerM3 = density;
                exhaustPaths_[path].mediumSoundSpeedMps = soundSpeed;
                exhaustPaths_[path].meanExhaustMassFlowKgPerSecond =
                    pathMeanMassFlowSum[path];
                (void) exhaustPaths_[path].radiation.setMedium(density, soundSpeed);
            }
        }
        const auto useCompiledTopology = sampleUsesPhysicalExhaust
            && acousticExhaustNetwork_ != nullptr;
        if (useCompiledTopology) {
            const auto observerPressure = acousticExhaustNetwork_->process(
                std::span<const float>(cylinderExhaustPulse.data(), activeCylinderCount),
                std::span<const PortBoundary>(portBoundary_.data(), activeCylinderCount),
                controlRampCoefficient_);
            const auto jetNoisePressure =
                acousticExhaustNetwork_->lastOutletJetNoisePressure();
            for (std::size_t path = 0; path < exhaustPathCount; ++path) {
                blockPeakObservedExhaustPressurePa = std::max(
                    blockPeakObservedExhaustPressurePa,
                    std::max(std::abs(observerPressure[path].leftPa),
                        std::abs(observerPressure[path].rightPa)));
                blockPeakObservedExhaustJetNoisePressurePa = std::max(
                    blockPeakObservedExhaustJetNoisePressurePa,
                    std::max(std::abs(jetNoisePressure[path].leftPa),
                        std::abs(jetNoisePressure[path].rightPa)));
                const auto calibratedLeft = static_cast<float>(
                    AcousticMonitorCalibration::normalisePeakPressure(
                        observerPressure[path].leftPa, acousticFullScaleSplDb));
                const auto calibratedRight = static_cast<float>(
                    AcousticMonitorCalibration::normalisePeakPressure(
                        observerPressure[path].rightPa, acousticFullScaleSplDb));
                exhaustLeft += calibratedLeft;
                exhaustRight += calibratedRight;
                convolutionBank_.addInput(path, 0, sample, calibratedLeft);
                convolutionBank_.addInput(path, 1, sample, calibratedRight);
            }
        } else {
            const auto collectorOut = processExhaustWaveguides(
                cylinderExhaustPulse, cylinderExhaustPath, runnerAdmittance, portReflection,
                portBoundary_, outletAdmittance, activeCylinderCount, exhaustPathCount);
            constexpr auto directWidth = 0.34F;
            for (std::size_t path = 0; path < exhaustPathCount; ++path) {
                const auto exhaustExciteLeft = sampleUsesPhysicalExhaust
                    ? collectorOut[path]
                    : collectorOut[path] * 0.64F + physicalBlowdownLeft[path] * directWidth;
                const auto exhaustExciteRight = sampleUsesPhysicalExhaust
                    ? collectorOut[path]
                    : collectorOut[path] * 0.64F + physicalBlowdownRight[path] * directWidth;
                pathCollectorLeft[path] += exhaustExciteLeft;
                pathCollectorRight[path] += exhaustExciteRight;
                // Every physical collector drives only its own measured IR. This
                // preserves independent banks/tailpipes instead of folding all
                // pressure into path zero before convolution.
                if (!sampleUsesPhysicalExhaust) {
                    convolutionBank_.addInput(path, 0, sample, exhaustExciteLeft * pathGain[path]);
                    convolutionBank_.addInput(path, 1, sample, exhaustExciteRight * pathGain[path]);
                }
            }
        }
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
        // Production engines use solver-resolved gas, inertia, bearing and
        // torque excitation through StructuralModalRadiator. The old shaped
        // oscillators remain only for graph-less compatibility producers.
        auto mechanical = 0.0F;
        if (!structuralModalRadiator_) {
            const auto pistonSlap = std::clamp(peakPistonAccelG / 6'000.0F, 0.0F, 1.4F);
            const auto valvetrainClatter = (
                static_cast<float>(std::sin(valvetrainPhase_ * 3.03)) * 0.5F
                + noise() * 0.5F) * (0.006F + stress * 0.011F);
            mechanical = (static_cast<float>(std::sin(mechanicalPhase_)) * 0.016F
                * (0.6F + pistonSlap * 0.9F) + valvetrainClatter) * speedGain;
        }
        auto intake = 0.0F;
        if (!acousticIntakeNetwork_) {
            intakeFilter_ += intakeFilterCoefficient_
                * (noise() * throttle * speedGain * lowNoise - intakeFilter_);
            const auto whoosh = intakeFilter_
                * (0.018F + load * 0.028F + intakeDepression * 0.030F);
            const auto intakeResonanceHz = std::clamp(
                intakeRunnerResonanceHz * acousticTimeScale, 20.0F, 5'600.0F);
            const auto intakeResonanceAmp = std::clamp(
                intakeRunnerAmplitudeKpa / 12.0F, -1.5F, 1.5F);
            const auto intakeF = std::clamp(2.0F * std::sin(
                static_cast<float>(std::numbers::pi) * intakeResonanceHz
                    / static_cast<float>(sampleRate_)), 0.0002F, 1.35F);
            const auto intakeExcite = (noise() * 0.7F + intakeFilter_)
                * (0.4F + throttle * 0.6F) * speedGain;
            intakeSvfLow_ += intakeF * intakeSvfBand_;
            const auto intakeHigh = intakeExcite - intakeSvfLow_
                - intakeSvfBand_ * 0.32F;
            intakeSvfBand_ += intakeF * intakeHigh;
            const auto intakeHonk = std::clamp(intakeSvfBand_, -2.0F, 2.0F)
                * intakeResonanceAmp * (0.05F + intakeDepression * 0.05F);
            intake = whoosh + intakeHonk;
        }
        const auto starterSound = starter * (static_cast<float>(std::sin(starterPhase_)) * 0.028F + noise() * 0.006F);
        mechanicalLeft += mechanical + starterSound;
        mechanicalRight += mechanical * 0.96F + starterSound * 0.94F;
        intakeLeft += intake * 0.92F;
        intakeRight += intake;
        for (auto& voice : voices_) {
            if (!voice.active) continue;
            if (voice.exhaust && physicalExhaustActive_) {
                voice.active = false;
                continue;
            }
            const auto attack = std::max(0.00008F, voice.attackSeconds);
            const auto blowdown = std::max(0.0012F, voice.blowdownSeconds);
            const auto shockEnvelope = (1.0F - std::exp(-voice.ageSeconds / attack))
                * std::exp(-voice.ageSeconds / blowdown);
            const auto flowEnvelope = std::exp(-voice.ageSeconds / (blowdown * 1.85F));
            const auto flow = std::clamp(voice.massFlow / 75.0F, 0.0F, 1.8F);
            const auto pressure = std::clamp((voice.runnerPressure - ambientPressureKpa) / 120.0F, 0.0F, 1.4F);
            const auto body = static_cast<float>(std::sin(voice.bodyPhase));
            const auto harmonic = static_cast<float>(std::sin(voice.bodyPhase * 2.01)) * 0.32F
                                + static_cast<float>(std::sin(voice.bodyPhase * 3.97)) * 0.13F;
            const auto pipe = static_cast<float>(std::sin(voice.pipePhase)) * 0.48F
                            + static_cast<float>(std::sin(voice.pipePhase * 0.503)) * 0.19F;
            // Compression ignition is not rendered as arbitrary "diesel
            // noise". A short, steep heat release excites more of the head
            // and block's upper modes than a slower premixed flame. The
            // simulator publishes that pressure-rise proxy as sharpness, so
            // the fallback voice uses it only to shorten the attack and drive
            // this deterministic bending-mode component. The full acoustic
            // path instead receives the resolved cylinder-pressure trace.
            const auto compressionRise = voice.compressionIgnition
                ? std::clamp(voice.combustionSharpness, 0.0F, 1.0F) : 0.0F;
            const auto jetCenter = std::clamp(520.0F + flow * 2'700.0F + pressure * 1'300.0F, 260.0F, 6'800.0F);
            const auto jetLowCoeff = static_cast<float>(1.0 - std::exp(-2.0 * std::numbers::pi * (jetCenter * 0.45F) / sampleRate_));
            const auto jetHighCoeff = static_cast<float>(1.0 - std::exp(-2.0 * std::numbers::pi * jetCenter / sampleRate_));
            const auto turbulentNoise = noise() * (0.35F + flow * 0.70F) * flowEnvelope;
            voice.jetLowState += std::clamp(jetLowCoeff, 0.001F, 0.82F) * (turbulentNoise - voice.jetLowState);
            voice.jetHighState += std::clamp(jetHighCoeff, 0.001F, 0.92F) * (voice.jetLowState - voice.jetHighState);
            voice.jetBandState = std::clamp((voice.jetLowState - voice.jetHighState) * (0.42F + flow * 0.48F), -2.0F, 2.0F);
            const auto crack = static_cast<float>(std::sin(voice.crackPhase))
                * (0.10F + voice.knock * 0.30F + pressure * 0.11F
                    + compressionRise * 0.48F);
            // Bore-dependent knock resonance (Draper first circumferential mode).
            const auto knockTone = static_cast<float>(std::sin(voice.knockPhase)) * voice.knock * 0.55F;
            const auto shock = (body * 0.35F + harmonic * 0.42F + pipe * 0.52F + crack + knockTone) * shockEnvelope;
            const auto jet = voice.exhaust ? voice.jetBandState * voice.turbulence : noise() * voice.turbulence * 0.20F;
            const auto raw = (shock + jet) * voice.amplitude;
            voice.filterState += voiceFilterCoefficient_ * (raw - voice.filterState);
            if (voice.exhaust) {
                const auto path = std::min<std::size_t>(voice.exhaustPathIndex, exhaustPathCount - 1U);
                const auto pathLeft = voice.filterState * voice.leftGain;
                const auto pathRight = voice.filterState * voice.rightGain;
                pathExhaustLeft[path] += pathLeft;
                pathExhaustRight[path] += pathRight;
                convolutionBank_.addInput(path, 0, sample, pathLeft * pathGain[path]);
                convolutionBank_.addInput(path, 1, sample, pathRight * pathGain[path]);
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
        const auto bankWidth = 1.0F + bankSeparation * 0.18F;
        const auto airCoefficient = static_cast<float>(1.0 - std::exp(
            -2.0 * std::numbers::pi * presetToneHz_ * acousticTimeScale / sampleRate_));
        for (std::size_t pathIndex = 0; pathIndex < exhaustPathCount; ++pathIndex) {
            if (useCompiledTopology) continue;
            auto& path = exhaustPaths_[pathIndex];
            // Same control-rate ramp as the runner delays; see
            // RunnerWaveguides::delayTargetSamples.
            path.reflectionDelaySamples += controlRampCoefficient_
                * (path.reflectionDelayTargetSamples - path.reflectionDelaySamples);
            path.mufflerCoefficients.delaySamples += controlRampCoefficient_
                * (path.mufflerDelayTargetSamples - path.mufflerCoefficients.delaySamples);
            const auto collectorInput = (pathCollectorLeft[pathIndex]
                + pathCollectorRight[pathIndex]) * 0.5F;

            if (sampleUsesPhysicalExhaust) {
                // Collector -> outlet characteristic, passive unflanged load,
                // then the reflected characteristic returns to the collector.
                // Every value on these two delay lines remains pressure in Pa.
                // Finite-amplitude propagation over the collector-to-outlet
                // duct: strong compressions arrive early, so wavefronts steepen
                // and repopulate the harmonics above the telemetry band. See
                // NonlinearDuctAcoustics for the model and its clamps.
                const auto stiffnessRhoC2 = path.mediumDensityKgPerM3
                    * path.mediumSoundSpeedMps * path.mediumSoundSpeedMps;
                const auto waveDelayLimit = static_cast<float>(
                    path.forwardWave.size() - 2);
                // Both legs have travelled the collector-to-outlet length, so
                // both are attenuated by the duct wall. Without this the only
                // loss in the whole path was the radiation load, which reflects
                // almost perfectly at low frequency -- the network's modes then
                // rang far above the firing harmonics.
                const auto incidentAtMouth = DuctWallLoss::process(
                    path.wallLoss, path.wallLossOutbound,
                    readWaveSteepened(path.forwardWave, path.waveWrite, path.waveMask,
                        path.reflectionDelaySamples, waveDelayLimit, stiffnessRhoC2));
                const auto returnedAtCollector = DuctWallLoss::process(
                    path.wallLoss, path.wallLossReturn,
                    readWaveSteepened(path.reverseWave, path.waveWrite, path.waveMask,
                        path.reflectionDelaySamples, waveDelayLimit, stiffnessRhoC2));
                const auto radiation = path.radiation.process(incidentAtMouth);
                path.reverseWave[path.waveWrite] = finiteState(
                    static_cast<float>(radiation.reflectedPressurePa), 5.0e6F);
                // The silencer sits at the collector end of the duct, so both
                // of its ports are available on the same sample: what the
                // collector sends out, and what the duct has just returned. A
                // chamber at the mouth end would need the radiation load's
                // reflection one sample early. Disabled, this is an exact
                // through-connection, so an engine with no chamber renders
                // bit-identically to before the element existed.
                const auto scattered = ExpansionChamberMuffler::process(
                    path.muffler, path.mufflerCoefficients,
                    collectorInput, returnedAtCollector);
                path.forwardWave[path.waveWrite] = finiteState(
                    scattered.towardOutlet, 5.0e6F);
                path.collectorReturn = finiteState(scattered.towardCollector, 5.0e6F);
                path.waveWrite = (path.waveWrite + 1U) & path.waveMask;

                // The monopole expression returns the correct far-field
                // amplitude but is evaluated at source time. Apply r/c in air
                // explicitly so its phase relative to the mechanical source is
                // also physical at the configured observer.
                const auto mouthPressure = std::isfinite(radiation.farFieldPressurePa)
                    ? static_cast<float>(radiation.farFieldPressurePa)
                    : 0.0F;
                const auto observerDelay0 = static_cast<std::size_t>(
                    path.observerDelaySamples);
                const auto observerFraction = path.observerDelaySamples
                    - static_cast<float>(observerDelay0);
                const auto observerRead0 = (path.observerWrite
                    + path.observerPressure.size() - observerDelay0)
                    & path.observerMask;
                const auto observerRead1 = (path.observerWrite
                    + path.observerPressure.size() - observerDelay0 - 1U)
                    & path.observerMask;
                const auto observerPressurePa = std::lerp(
                    path.observerPressure[observerRead0],
                    path.observerPressure[observerRead1], observerFraction);
                blockPeakObservedExhaustPressurePa = std::max(
                    blockPeakObservedExhaustPressurePa, std::abs(observerPressurePa));
                path.observerPressure[path.observerWrite] = mouthPressure;
                path.observerWrite = (path.observerWrite + 1U)
                    & path.observerMask;

                const auto calibrated = static_cast<float>(
                    AcousticMonitorCalibration::normalisePeakPressure(
                        observerPressurePa, acousticFullScaleSplDb));
                // Outlet positions are not yet published, so free-field paths
                // are summed at one co-located mono observer. Inventing a stereo
                // pan would be less honest; a measured stereo IR can spatialise
                // the already-radiated signal downstream.
                exhaustLeft += calibrated;
                exhaustRight += calibrated;
                convolutionBank_.addInput(pathIndex, 0, sample, calibrated);
                convolutionBank_.addInput(pathIndex, 1, sample, calibrated);
                continue;
            }

            // Nonlinear and reflective state is deliberately private to this
            // tailpipe. Only cylinders assigned to the path can excite it.
            const auto collectorDrive = presetDrive_ * (0.42F + pathOpenness[pathIndex] * 0.10F
                + load * 0.26F + speedGain * 0.20F + physicalExhaustPressure * 0.30F
                + physicalExhaustFlow * 0.18F + (boostRatio - 1.0F) * 0.12F);
            const auto continuousJet = noise() * physicalExhaustFlow
                * (0.0018F + physicalExhaustPressure * 0.0045F)
                * (timeScale > 0.01F ? 1.0F : 0.0F) * pathCountGain;
            const auto proceduralCollectorInput = collectorInput + continuousJet;
            const auto pressureDerivative = proceduralCollectorInput
                - path.previousCollectorInput;
            path.previousCollectorInput = proceduralCollectorInput;
            const auto conditionedCollector = proceduralCollectorInput + pressureDerivative
                * static_cast<float>(sampleRate_ / referenceSampleRate)
                * std::clamp(0.025F + physicalExhaustPressure * 0.025F, 0.02F, 0.08F);
            path.jitterHistory[path.jitterWrite] = finiteState(conditionedCollector);
            const auto maximumJitterSamples = std::min(5.5F
                    * static_cast<float>(sampleRate_ / referenceSampleRate * acousticDelayScale),
                static_cast<float>(path.jitterHistory.size() - 2U));
            const auto targetJitter = std::clamp(std::abs(noise())
                * (0.35F + physicalExhaustFlow * 1.4F)
                * static_cast<float>(sampleRate_ / referenceSampleRate * acousticDelayScale),
                0.0F, maximumJitterSamples);
            path.jitterDelaySamples += jitterCoefficient_ * (targetJitter - path.jitterDelaySamples);
            const auto delay0 = static_cast<std::size_t>(path.jitterDelaySamples);
            const auto delay1 = std::min(delay0 + 1U, path.jitterHistory.size() - 1U);
            const auto jitterFraction = path.jitterDelaySamples - static_cast<float>(delay0);
            const auto read0 = (path.jitterWrite + path.jitterHistory.size() - delay0)
                % path.jitterHistory.size();
            const auto read1 = (path.jitterWrite + path.jitterHistory.size() - delay1)
                % path.jitterHistory.size();
            const auto jitteredCollector = path.jitterHistory[read0] * (1.0F - jitterFraction)
                + path.jitterHistory[read1] * jitterFraction;
            path.jitterWrite = (path.jitterWrite + 1) % path.jitterHistory.size();
            const auto exhaustMono = saturateCollector(jitteredCollector, collectorDrive);
            path.collectorState += collectorCoefficient_ * (exhaustMono - path.collectorState);
            const auto reflectionDelay0 = static_cast<std::size_t>(path.reflectionDelaySamples);
            const auto reflectionFraction = path.reflectionDelaySamples
                - static_cast<float>(reflectionDelay0);
            const auto waveRead0 = (path.waveWrite + path.forwardWave.size()
                - reflectionDelay0) & path.waveMask;
            const auto waveRead1 = (path.waveWrite + path.forwardWave.size()
                - reflectionDelay0 - 1U) & path.waveMask;
            const auto outletPressure = std::lerp(path.forwardWave[waveRead0],
                path.forwardWave[waveRead1], reflectionFraction);
            path.reflectedLowPass += reflectionFilterCoefficient_
                * (outletPressure - path.reflectedLowPass);
            const auto highReflection = outletPressure - path.reflectedLowPass;
            const auto radiated = path.reflectedLowPass * (0.58F + (1.0F - presetReflection_) * 0.20F)
                + highReflection * (0.16F + highGain * 0.05F);
            const auto returned = std::lerp(path.reverseWave[waveRead0],
                path.reverseWave[waveRead1], reflectionFraction);
            path.reverseWave[path.waveWrite] = finiteState(
                highReflection * collectorReflection[pathIndex] - returned * 0.10F);
            path.forwardWave[path.waveWrite] = finiteState(
                exhaustMono + path.collectorState * 0.06F);
            // The reverse line has now completed outlet -> collector travel;
            // expose it to the primary junction on the following sample.
            path.collectorReturn = finiteState(returned);
            path.waveWrite = (path.waveWrite + 1) & path.waveMask;
            // The muffler FDN and the resonant body are the exhaust model, not
            // the impulse response. They no longer ride the `convolution` knob,
            // which is purely the IR dry/wet mix: turning the IR down used to
            // mute the muffler simulation as a side effect. presetWet_ already
            // folds in that knob's default so the shipped voicing is unchanged.
            const auto fdnWet = processMufflerFdn(path, radiated + exhaustMono * 0.28F)
                * presetWet_;
            path.exhaustBodyLeft = path.exhaustBodyLeft * presetDamping_
                + (radiated + fdnWet) * 0.0185F * bodyExcitationScale_;
            path.exhaustBodyRight = path.exhaustBodyRight * presetDamping_
                + (radiated * 0.76F + fdnWet * 0.91F) * 0.0157F * bodyExcitationScale_;
            path.exhaustAirLeft += airCoefficient
                * (pathExhaustLeft[pathIndex] + radiated - path.exhaustAirLeft);
            path.exhaustAirRight += airCoefficient
                * (pathExhaustRight[pathIndex] + radiated * 0.82F - path.exhaustAirRight);
            // Event voices have already traversed the complete graph delay.
            // Continuous pressure instead travels primary -> collector ->
            // outlet here. Keeping those paths distinct avoids propagating and
            // summing the same exhaust pulse twice.
            exhaustLeft += (pathExhaustLeft[pathIndex]
                + (radiated + fdnWet + path.exhaustBodyLeft) * bankWidth
                + path.exhaustAirLeft * (0.13F + pathOpenness[pathIndex] * 0.05F)) * pathGain[pathIndex];
            exhaustRight += (pathExhaustRight[pathIndex]
                + (radiated * 0.82F + fdnWet * 0.88F + path.exhaustBodyRight) * bankWidth
                + path.exhaustAirRight * (0.13F + pathOpenness[pathIndex] * 0.05F)) * pathGain[pathIndex];
        }
        // Record which path actually produced this sample. A physical path that
        // never activates, or that drops back after activating, is otherwise
        // indistinguishable from a physical path that simply sounds unchanged.
        if (!sampleUsesPhysicalExhaust) {
            ++legacySamplesThisBlock;
            if (physicalExhaustActive_.load(std::memory_order_relaxed))
                ++invalidBoundarySamplesThisBlock;
        }

        // A short chamber-pressure tail restores body to the combustion layer.
        // Exhaust already has runner, outlet, FDN and IR memory; feeding it back
        // here duplicated the same pulse and produced the former low-frequency
        // pile-up.
        pressureTailLeft_ = pressureTailLeft_ * pressureTailDecay_
            + combustionLeft * pressureTailInputCoefficient_;
        pressureTailRight_ = pressureTailRight_ * pressureTailDecay_
            + combustionRight * pressureTailInputCoefficient_;
        combustionLeft += pressureTailLeft_ * 0.08F;
        combustionRight += pressureTailRight_ * 0.08F;
        if (!sampleUsesPhysicalExhaust) {
            exhaustLeft += noise() * highNoise * speedGain * 0.002F;
            exhaustRight += noise() * highNoise * speedGain * 0.002F;
        }

        // Only pressure/flow emerging from the exhaust termination receives
        // the open-pipe radiation shelf.  Applying it after the master mix used
        // to attenuate combustion, intake and mechanical fundamentals by about
        // 11 dB at idle even though those sources do not radiate from the
        // tailpipe.  Dry and IR-wet exhaust are filtered independently; the
        // shelf is linear, so their sum is equivalent to filtering one exhaust
        // bus without touching the other layers.
        auto radiatedExhaustLeft = exhaustLeft;
        auto radiatedExhaustRight = exhaustRight;
        if (!sampleUsesPhysicalExhaust) {
            exhaustRadiationLowLeft_ += radiationLowCoefficient_
                * (exhaustLeft - exhaustRadiationLowLeft_);
            exhaustRadiationLowRight_ += radiationLowCoefficient_
                * (exhaustRight - exhaustRadiationLowRight_);
            radiatedExhaustLeft -= exhaustRadiationLowLeft_ * 0.78F;
            radiatedExhaustRight -= exhaustRadiationLowRight_ * 0.78F;
        }
        const auto legacyMonitorScale = acousticDisplacementScale
            * legacyReferenceLevel;
        const auto exhaustMonitorScale = sampleUsesPhysicalExhaust
            ? 1.0F : legacyMonitorScale;
        if (captureStems) {
            writeStem(writable.combustion, sample,
                combustionLeft * combustionGain * legacyMonitorScale,
                combustionRight * combustionGain * legacyMonitorScale);
            writeStem(writable.exhaustDry, sample,
                radiatedExhaustLeft * exhaustGain * exhaustMonitorScale,
                radiatedExhaustRight * exhaustGain * exhaustMonitorScale);
            writeStem(writable.intake, sample,
                intakeLeft * intakeGain * legacyMonitorScale
                    + physicalIntakeLeft * intakeGain,
                intakeRight * intakeGain * legacyMonitorScale
                    + physicalIntakeRight * intakeGain);
            writeStem(writable.forcedInduction, sample,
                physicalForcedInduction * intakeGain,
                physicalForcedInduction * intakeGain);
            writeStem(writable.mechanical, sample,
                mechanicalLeft * mechanicalGain * legacyMonitorScale
                    + physicalStructural * mechanicalGain,
                mechanicalRight * mechanicalGain * legacyMonitorScale
                    + physicalStructural * mechanicalGain);
        }
        auto left = (combustionLeft * combustionGain + intakeLeft * intakeGain
            + mechanicalLeft * mechanicalGain) * legacyMonitorScale
            + physicalStructural * mechanicalGain
            + physicalIntakeLeft * intakeGain
            + physicalForcedInduction * intakeGain
            + radiatedExhaustLeft * exhaustGain * exhaustMonitorScale;
        auto right = (combustionRight * combustionGain + intakeRight * intakeGain
            + mechanicalRight * mechanicalGain) * legacyMonitorScale
            + physicalStructural * mechanicalGain
            + physicalIntakeRight * intakeGain
            + physicalForcedInduction * intakeGain
            + radiatedExhaustRight * exhaustGain * exhaustMonitorScale;
        lowPassLeft_ += lowPassCoefficient_ * (left - lowPassLeft_);
        lowPassRight_ += lowPassCoefficient_ * (right - lowPassRight_);
        if (output.getNumChannels() > 0) output.setSample(0, startSample + sample, lowPassLeft_);
        if (output.getNumChannels() > 1) output.setSample(1, startSample + sample, lowPassRight_);
        audioTimeSeconds_ += audioTimeStep;
    }
    if (blockPeakObservedExhaustPressurePa
            > maxObservedExhaustPressurePa_.load(std::memory_order_relaxed)) {
        maxObservedExhaustPressurePa_.store(
            blockPeakObservedExhaustPressurePa, std::memory_order_relaxed);
    }
    if (blockPeakObservedExhaustJetNoisePressurePa
            > maxObservedExhaustJetNoisePressurePa_.load(
                std::memory_order_relaxed)) {
        maxObservedExhaustJetNoisePressurePa_.store(
            blockPeakObservedExhaustJetNoisePressurePa,
            std::memory_order_relaxed);
    }
    if (blockPeakObservedIntakePressurePa
            > maxObservedIntakePressurePa_.load(std::memory_order_relaxed))
        maxObservedIntakePressurePa_.store(
            blockPeakObservedIntakePressurePa, std::memory_order_relaxed);
    if (acousticIntakeNetwork_) {
        const auto diagnostics = acousticIntakeNetwork_->diagnostics();
        maxIntakeSourcePressurePa_.store(
            diagnostics.sourcePressurePa, std::memory_order_relaxed);
        maxIntakeRunnerPressurePa_.store(
            diagnostics.runnerPressurePa, std::memory_order_relaxed);
        maxIntakePlenumPressurePa_.store(
            diagnostics.plenumPressurePa, std::memory_order_relaxed);
        maxIntakeAirboxPressurePa_.store(
            diagnostics.airboxPressurePa, std::memory_order_relaxed);
        maxIntakeMouthPressurePa_.store(
            diagnostics.mouthPressurePa, std::memory_order_relaxed);
        maxIntakeRadiatedPressurePa_.store(
            diagnostics.radiatedPressurePa, std::memory_order_relaxed);
    }
    if (blockPeakObservedStructuralPressurePa
            > maxObservedStructuralPressurePa_.load(std::memory_order_relaxed))
        maxObservedStructuralPressurePa_.store(
            blockPeakObservedStructuralPressurePa, std::memory_order_relaxed);
    if (legacySamplesThisBlock != 0)
        legacyPathSamples_.fetch_add(legacySamplesThisBlock, std::memory_order_relaxed);
    if (invalidBoundarySamplesThisBlock != 0)
        invalidBoundarySamples_.fetch_add(
            invalidBoundarySamplesThisBlock, std::memory_order_relaxed);

    convolutionBank_.process(sampleCount);
    const auto irMix = std::clamp(convolution * 0.50F, 0.0F, 0.78F);
    // Pass 1: dry/wet sum, a genuine high shelf, DC removal, reconstruction
    // filtering and calibrated gain staging.  Master volume is deliberately
    // before the only output limiter, so volume=2 can never create >0 dBFS.
    std::uint64_t levelLimitedBlockSamples = 0;
    float levelGainBlockMin = 1.0F;
    float preLimiterBlockPeak = 0.0F;
    for (int sample = 0; sample < sampleCount; ++sample) {
        const auto dryLeft = output.getNumChannels() > 0
            ? output.getSample(0, startSample + sample) : 0.0F;
        const auto dryRight = output.getNumChannels() > 1
            ? output.getSample(1, startSample + sample) : dryLeft;
        const auto wetExhaustLeft = convolutionBank_.wetSample(0, sample);
        const auto wetExhaustRight = convolutionBank_.wetSample(1, sample);
        wetExhaustRadiationLowLeft_ += radiationLowCoefficient_
            * (wetExhaustLeft - wetExhaustRadiationLowLeft_);
        wetExhaustRadiationLowRight_ += radiationLowCoefficient_
            * (wetExhaustRight - wetExhaustRadiationLowRight_);
        const auto radiatedWetExhaustLeft = physicalExhaustActive_
            ? wetExhaustLeft
            : wetExhaustLeft - wetExhaustRadiationLowLeft_ * 0.78F;
        const auto radiatedWetExhaustRight = physicalExhaustActive_
            ? wetExhaustRight
            : wetExhaustRight - wetExhaustRadiationLowRight_ * 0.78F;
        const auto wetMonitorScale = physicalExhaustActive_
            ? 1.0F : acousticDisplacementScale * legacyReferenceLevel;
        if (captureStems) {
            writeStem(writable.exhaustIr, sample,
                radiatedWetExhaustLeft * irMix * exhaustGain * wetMonitorScale,
                radiatedWetExhaustRight * irMix * exhaustGain * wetMonitorScale);
        }
        auto left = finiteState(dryLeft
            + radiatedWetExhaustLeft * irMix * exhaustGain * wetMonitorScale, 24.0F);
        auto right = finiteState(dryRight
            + radiatedWetExhaustRight * irMix * exhaustGain * wetMonitorScale, 24.0F);

        toneLowLeft_ += toneCoefficient_ * (left - toneLowLeft_);
        toneLowRight_ += toneCoefficient_ * (right - toneLowRight_);
        left = toneLowLeft_ + (left - toneLowLeft_) * highGain;
        right = toneLowRight_ + (right - toneLowRight_) * highGain;

        const auto dcLeft = left - dcInputLeft_ + dcBlockPole_ * dcOutputLeft_;
        const auto dcRight = right - dcInputRight_ + dcBlockPole_ * dcOutputRight_;
        dcInputLeft_ = left;
        dcInputRight_ = right;
        dcOutputLeft_ = finiteState(dcLeft, 24.0F);
        dcOutputRight_ = finiteState(dcRight, 24.0F);

        antiAliasLeftA_ += antiAliasCoefficient_ * (dcOutputLeft_ - antiAliasLeftA_);
        antiAliasLeftB_ += antiAliasCoefficient_ * (antiAliasLeftA_ - antiAliasLeftB_);
        antiAliasRightA_ += antiAliasCoefficient_ * (dcOutputRight_ - antiAliasRightA_);
        antiAliasRightB_ += antiAliasCoefficient_ * (antiAliasRightA_ - antiAliasRightB_);
        left = antiAliasLeftB_ * volume;
        right = antiAliasRightB_ * volume;

        const auto magnitude = std::max(std::abs(left), std::abs(right));
        preLimiterBlockPeak = std::max(preLimiterBlockPeak, magnitude);
        const auto envelopeCoefficient = magnitude > levelEnvelope_
            ? levelAttackCoefficient_ : levelReleaseCoefficient_;
        levelEnvelope_ += envelopeCoefficient * (magnitude - levelEnvelope_);
        // Slow safety gain only: normal engine dynamics remain untouched.  The
        // oversampled limiter below catches isolated peaks without pumping.
        const auto targetGain = levelEnvelope_ > 0.78F
            ? std::clamp(0.78F / levelEnvelope_, 0.20F, 1.0F) : 1.0F;
        const auto levelRate = targetGain < levelGain_
            ? gainAttackCoefficient_ : gainReleaseCoefficient_;
        levelGain_ += levelRate * (targetGain - levelGain_);
        if (levelGain_ < 0.99999F) ++levelLimitedBlockSamples;
        levelGainBlockMin = std::min(levelGainBlockMin, levelGain_);
        if (output.getNumChannels() > 0) output.setSample(0, startSample + sample, left * levelGain_);
        if (output.getNumChannels() > 1) output.setSample(1, startSample + sample, right * levelGain_);
    }
    // Publish the leveler observers once per block (measurement only).
    if (levelLimitedBlockSamples != 0)
        levelLimitedSamples_.fetch_add(levelLimitedBlockSamples, std::memory_order_relaxed);
    if (levelGainBlockMin < minObservedLevelGain_.load(std::memory_order_relaxed))
        minObservedLevelGain_.store(levelGainBlockMin, std::memory_order_relaxed);
    if (preLimiterBlockPeak > maxPreLimiterMagnitude_.load(std::memory_order_relaxed))
        maxPreLimiterMagnitude_.store(preLimiterBlockPeak, std::memory_order_relaxed);
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
    // Pass 3: leave inter-sample headroom after downsampling and downmix extras.
    for (int sample = 0; sample < sampleCount; ++sample) {
        const auto left = output.getNumChannels() > 0 ? output.getSample(0, startSample + sample) : 0.0F;
        const auto right = output.getNumChannels() > 1 ? output.getSample(1, startSample + sample) : left;
        const auto outLeft = std::clamp(finiteState(left, 1.0F), -0.999F, 0.999F);
        const auto outRight = std::clamp(finiteState(right, 1.0F), -0.999F, 0.999F);
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
    const auto publishedScale = realtimeState_.timeScale.load(std::memory_order_relaxed);
    const auto audibleScale = std::clamp(publishedScale > 0.01F ? publishedScale : 1.0F,
                                         0.25F, 4.0F);
    const auto minimumDurationSeconds = event.compressionIgnition ? 0.0012F : 0.004F;
    const auto durationSeconds = std::max(
        minimumDurationSeconds, event.combustionDurationMs * 0.001F)
        / audibleScale;
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
    const auto eventPath = std::min<std::size_t>(event.exhaustPathIndex,
        std::max<std::size_t>(1, allocatedPaths_) - 1U);
    const auto exhaustOpenness = std::clamp(
        realtimeState_.exhaustPathOpenness[eventPath].load(std::memory_order_relaxed), 0.15F, 1.45F);
    const auto boostRatio = std::clamp(realtimeState_.boostPressureRatio.load(std::memory_order_relaxed), 1.0F, 3.5F);
    const auto cylinderNormalization = std::clamp(std::pow(4.0F / cylinders, 0.32F), 0.68F, 1.32F);
    const auto chamberScale = std::clamp(std::sqrt(cylinderDisplacement / 0.5F), 0.45F, 2.25F);
    // Exhaust voices are now only crack/transient accents on top of the
    // pressure-driven exhaust body, so they sit well below the physical source.
    const auto layerGain = exhaust ? 0.20F : 0.16F;
    const auto exhaustTransmission = exhaust
        ? std::clamp(event.exhaustTransmissionGain, 0.0F, 8.0F) : 1.0F;
    voice->amplitude = std::max(0.001F, event.intensity
        * (event.misfire ? 0.07F : layerGain) * cylinderNormalization)
        * exhaustTransmission;
    const auto combustionSharpness = std::clamp(event.combustionSharpness, 0.0F, 1.0F);
    voice->attackSeconds = exhaust ? 0.00018F
        : 0.00042F / (1.0F + (event.compressionIgnition
            ? combustionSharpness * 3.5F : 0.0F));
    voice->blowdownSeconds = std::clamp(durationSeconds * (exhaust ? 0.42F : 0.70F), 0.0015F, 0.014F);
    voice->decay = std::exp(std::log(0.0001F) / static_cast<float>(sampleRate_ * (durationSeconds * 1.35F)));
    voice->bodyFrequency = exhaust
        ? (30.0F + event.pressureEstimateBar * 0.55F + event.exhaustResonanceHz * 0.16F)
            / std::clamp(chamberScale, 0.7F, 1.8F)
        : (88.0F + event.pressureEstimateBar * 1.35F) * std::clamp(boreStroke, 0.75F, 1.28F);
    voice->crackFrequency = (exhaust ? 620.0F
        : (event.compressionIgnition
            ? 1'650.0F + combustionSharpness * 1'350.0F
            : 1'050.0F))
        + event.intensity * (exhaust ? 1'250.0F : 1'700.0F)
        + static_cast<float>(event.cylinderId % 32U) * 13.0F
        + (redline / 7'000.0F - 1.0F) * 360.0F;
    voice->pipeFrequency = std::clamp(event.exhaustResonanceHz > 1.0F ? event.exhaustResonanceHz
        : 110.0F + event.pressureEstimateBar * 1.35F, 45.0F, 1'400.0F) * std::clamp(1.18F - cylinderDisplacement * 0.18F, 0.64F, 1.25F);
    voice->bodyFrequency *= audibleScale;
    voice->crackFrequency *= audibleScale;
    voice->pipeFrequency *= audibleScale;
    const auto leanCrackle = std::clamp(std::abs(event.airFuelRatio - 13.2F) / 7.5F, 0.0F, 1.0F);
    voice->massFlow = std::max(0.0F, event.exhaustFlowMgPerCycle);
    voice->runnerPressure = std::max(80.0F, event.exhaustRunnerPressureKpa);
    voice->exhaustPathIndex = static_cast<std::uint32_t>(eventPath);
    const auto flowTone = std::clamp(voice->massFlow / 58.0F, 0.0F, 1.7F);
    voice->turbulence = (exhaust ? 0.11F + exhaustOpenness * 0.07F : 0.07F)
        + leanCrackle * 0.08F + event.knockAmount * 0.12F + flowTone * 0.16F + (boostRatio - 1.0F) * 0.06F;
    voice->knock = std::clamp(event.knockAmount, 0.0F, 1.0F);
    voice->combustionSharpness = combustionSharpness;
    voice->compressionIgnition = event.compressionIgnition;
    // Draper first circumferential knock mode: f ~= 1.841 c / (pi B), hot burned
    // gas sound speed ~= 900 m/s. Larger bores knock lower, smaller bores higher.
    const auto boreMm = std::clamp(realtimeState_.meanBoreMm.load(std::memory_order_relaxed), 40.0F, 160.0F);
    voice->knockFrequency = std::clamp(1.841F * 900'000.0F
        / (static_cast<float>(std::numbers::pi) * boreMm), 2'000.0F, 9'000.0F) * audibleScale;
    const auto maximumVoiceHz = static_cast<float>(std::min(18'000.0, sampleRate_ * 0.42));
    voice->bodyFrequency = std::clamp(voice->bodyFrequency, 12.0F, maximumVoiceHz);
    voice->crackFrequency = std::clamp(voice->crackFrequency, 40.0F, maximumVoiceHz);
    voice->pipeFrequency = std::clamp(voice->pipeFrequency, 12.0F, maximumVoiceHz);
    voice->knockFrequency = std::clamp(voice->knockFrequency, 200.0F, maximumVoiceHz);
    const auto panValue = std::clamp(event.stereoPosition, -0.82F, 0.82F);
    voice->leftGain = std::sqrt((1.0F - panValue) * 0.5F);
    voice->rightGain = std::sqrt((1.0F + panValue) * 0.5F);
    voice->exhaust = exhaust;
    voice->active = true;
}

void RealtimeEngineAudio::updateExhaustPreset(int preset) noexcept {
    activeExhaustPreset_ = std::clamp(preset, 0, 4);
    struct PresetValues { float drive; float damping; float wet; float tone; float reflection; float fdn; };
    // wet and fdn were rescaled in one pass when two coupled corrections landed:
    //  - The muffler FDN wet and body excitation no longer ride the `convolution`
    //    (IR-mix) knob, so wet folds in that knob's 0.45 default to hold the
    //    shipped voicing while the muffler model stays present at any IR mix.
    //  - The Hadamard matrix was corrected from a 0.25 (energy-halving) to a 0.5
    //    (lossless) normalisation, doubling the loop gain, so fdn was halved to
    //    keep the same decay. presetFdngain_ is now the true loop gain.
    constexpr std::array<PresetValues, 5> presets {{
        { 1.02F, 0.986F, 0.189F, 1'450.0F, 0.36F, 0.280F },
        { 1.42F, 0.974F, 0.108F, 2'650.0F, 0.24F, 0.209F },
        { 0.76F, 0.992F, 0.324F, 920.0F, 0.48F, 0.340F },
        { 1.14F, 0.989F, 0.261F, 1'700.0F, 0.41F, 0.310F },
        { 1.30F, 0.982F, 0.140F, 2'250.0F, 0.30F, 0.240F }
    }};
    const auto values = presets[static_cast<std::size_t>(activeExhaustPreset_)];
    presetDrive_ = values.drive;
    presetDamping_ = rateInvariantPole(values.damping, sampleRate_);
    bodyExcitationScale_ = (1.0F - presetDamping_) / std::max(1.0e-6F, 1.0F - values.damping);
    presetWet_ = values.wet;
    presetToneHz_ = values.tone;
    presetReflection_ = std::clamp(values.reflection, 0.08F, 0.72F);
    // This is a per-delay-loop feedback value. The FDN delays themselves are
    // scaled with sample rate, so its decay in seconds remains invariant.
    presetFdngain_ = std::clamp(values.fdn * values.damping, 0.20F, 0.76F);
}

std::array<float, RealtimeEngineAudio::maximumPaths> RealtimeEngineAudio::processExhaustWaveguides(
    const std::array<float, maxRunners>& pulse,
    const std::array<std::uint8_t, maxRunners>& pathIndex,
    const std::array<float, maxRunners>& runnerAdmittance,
    const std::array<float, maxRunners>& portReflection,
    const std::array<PortBoundary, maxRunners>& portBoundary,
    const std::array<float, maximumPaths>& outletAdmittance,
    std::size_t count, std::size_t pathCount) noexcept {
    // Bidirectional digital waveguide: each cylinder's blow-down pulse travels
    // its runner (a delay line) to a shared collector. The N-port scattering
    // junction reflects part of every runner's wave back into all the others,
    // which is the physical origin of collector cross-talk and header tuning
    // (e.g. flat-plane vs cross-plane V8 character emerges from firing order +
    // runner lengths, not from a preset).
    const auto n = std::min({ count, maxRunners, allocatedRunners_ });
    pathCount = std::clamp<std::size_t>(pathCount, 1, allocatedPaths_);
    const auto stride = runners_->forward.stride;
    const auto mask = runners_->forward.mask;
    std::array<float, maximumPaths> output {};
    if (n == 0) {
        for (std::size_t path = 0; path < pathCount; ++path)
            exhaustPaths_[path].collectorReturn = 0.0F;
        return output;
    }
    std::array<float, maxRunners> arrived {};
    std::array<float, maximumPaths> weightedIncidentSum {};
    std::array<float, maximumPaths> junctionAdmittance {};
    std::array<std::size_t, maximumPaths> runnerCount {};
    // Interpolated read at a fractional delay behind a line's write cursor.
    const auto readDelayed = [stride, mask](const float* line, std::size_t write,
                                            float delaySamples) {
        const auto delay0 = static_cast<std::size_t>(delaySamples);
        const auto fraction = delaySamples - static_cast<float>(delay0);
        const auto read0 = (write + stride - delay0) & mask;
        const auto read1 = (write + stride - delay0 - 1U) & mask;
        return std::lerp(line[read0], line[read1], fraction);
    };
    // Finite-amplitude propagation: legacy runners publish rho*c^2 = 0 and
    // therefore read at exactly the nominal delay. See NonlinearDuctAcoustics.
    const auto readSteepened = [&readDelayed](const float* line, std::size_t write,
                                              float delaySamples, float delayLimit,
                                              float stiffnessRhoC2) {
        return NonlinearDuctAcoustics::steepenedRead(
            [&readDelayed, line, write](float delay) {
                return readDelayed(line, write, delay);
            },
            delaySamples, delayLimit, stiffnessRhoC2);
    };
    const auto delayLimit = static_cast<float>(stride - 2);
    for (std::size_t i = 0; i < n; ++i) {
        // Ramp the delay toward its block-rate target; applying the target
        // directly phase-jumps the line at every telemetry frame. See
        // RunnerWaveguides::delayTargetSamples. The ramped value is shared by
        // this loop and the port loop below within the same sample.
        runners_->delaySamples[i] += controlRampCoefficient_
            * (runners_->delayTargetSamples[i] - runners_->delaySamples[i]);
        const auto exactDelay = std::clamp(runners_->delaySamples[i], 1.0F, delayLimit);
        const auto* forward = runners_->forward.line(i);
        // The wave has just travelled the runner's full length to reach the
        // junction, so it arrives attenuated by the duct's boundary layer.
        arrived[i] = DuctWallLoss::process(runnerWallLoss_[i],
            runnerWallLossToJunction_[i],
            readSteepened(forward, runners_->write[i], exactDelay, delayLimit,
                          runnerStiffnessRhoC2_[i]));
        const auto path = std::min<std::size_t>(pathIndex[i], pathCount - 1U);
        const auto admittance = std::clamp(runnerAdmittance[i], 1.0e-10F, 0.10F);
        weightedIncidentSum[path] += admittance * arrived[i];
        junctionAdmittance[path] += admittance;
        ++runnerCount[path];
    }
    // Lossless pressure scattering junction.  Characteristic acoustic
    // admittance is proportional to tube area (rho*c is common here), hence
    // unequal primaries and collectors no longer behave as identical pipes.
    std::array<float, maximumPaths> junctionPressure {};
    for (std::size_t path = 0; path < pathCount; ++path) {
        auto& collectorReturn = exhaustPaths_[path].collectorReturn;
        if (runnerCount[path] == 0) {
            collectorReturn = 0.0F;
            continue;
        }
        const auto outletY = std::clamp(outletAdmittance[path], 1.0e-10F, 0.10F);
        const auto totalY = std::max(1.0e-12F, junctionAdmittance[path] + outletY);
        junctionPressure[path] = 2.0F
            * (weightedIncidentSum[path] + outletY * collectorReturn) / totalY;
        output[path] = finiteState(junctionPressure[path] - collectorReturn, 5.0e6F);
    }
    for (std::size_t i = 0; i < n; ++i) {
        const auto path = std::min<std::size_t>(pathIndex[i], pathCount - 1U);
        const auto exactDelay = std::clamp(runners_->delaySamples[i], 1.0F, delayLimit);
        auto* forward = runners_->forward.line(i);
        auto* backward = runners_->backward.line(i);
        // Wave that left the junction 'delay' samples ago now reaches the port
        // and reflects off the valve.
        // Likewise for the return leg, junction back down to the port.
        const auto portIncident = DuctWallLoss::process(runnerWallLoss_[i],
            runnerWallLossToPort_[i],
            readSteepened(backward, runners_->write[i], exactDelay, delayLimit,
                          runnerStiffnessRhoC2_[i]));
        float portReturn = 0.0F;
        if (portBoundary[i].physical) {
            // The orifice resistance is set by the total velocity through the
            // opening, so the acoustic volume velocity has to be evaluated here,
            // from the wave state, rather than precomputed from telemetry. The
            // previous sample's reflection is used, which is the standard
            // explicit treatment of a nonlinear termination and keeps the
            // waveguide causal.
            const auto& boundary = portBoundary[i];
            const auto impedance = static_cast<double>(
                boundary.characteristicImpedancePaSPerM3);
            const auto acousticVolumeVelocity = impedance > 0.0
                ? (static_cast<double>(portIncident)
                    - portReflectionState_[i].previousOutput) / impedance
                : 0.0;
            const auto coefficients = ValvePortTermination::compute(
                boundary.conductanceAreaM2, boundary.meanMassFlowKgPerSecond,
                acousticVolumeVelocity, boundary.densityKgPerM3, impedance,
                sampleRate_);
            portReturn = ValvePortTermination::process(
                coefficients, portReflectionState_[i], portIncident);
        } else {
            portReturn = portIncident * std::clamp(portReflection[i], -0.999F, 0.999F);
        }
        forward[runners_->write[i]] = finiteState(pulse[i] + portReturn, 5.0e6F);
        backward[runners_->write[i]] = finiteState(
            junctionPressure[path] - arrived[i], 5.0e6F);
        runners_->write[i] = (runners_->write[i] + 1) & mask;
    }
    return output;
}

float RealtimeEngineAudio::processMufflerFdn(ExhaustPathState& path, float sample) noexcept {
    const auto a = path.fdn[0][path.fdnWrite[0]];
    const auto b = path.fdn[1][path.fdnWrite[1]];
    const auto c = path.fdn[2][path.fdnWrite[2]];
    const auto d = path.fdn[3][path.fdnWrite[3]];
    const auto feedback = std::clamp(presetFdngain_, 0.12F, 0.74F);
    // The 4x4 Hadamard matrix has rows of norm 2, so the energy-preserving
    // (orthonormal) normalisation is H/2, i.e. a 0.5 scale. The previous 0.25
    // applied H/4, whose singular values are 0.5: it silently halved the loop
    // energy every pass, so presetFdngain_ was only ever half the real feedback.
    // With the correct 0.5 the matrix is lossless and presetFdngain_ is the
    // actual loop gain that sets the decay. The preset feedback values were
    // halved to match, keeping the decay while making the parameter honest.
    path.fdn[0][path.fdnWrite[0]] = finiteState(sample + (a + b + c + d) * 0.5F * feedback);
    path.fdn[1][path.fdnWrite[1]] = finiteState(sample * 0.71F + (a - b + c - d) * 0.5F * feedback);
    path.fdn[2][path.fdnWrite[2]] = finiteState(sample * 0.53F + (a + b - c - d) * 0.5F * feedback);
    path.fdn[3][path.fdnWrite[3]] = finiteState(sample * 0.39F + (a - b - c + d) * 0.5F * feedback);
    for (std::size_t line = 0; line < path.fdn.size(); ++line)
        path.fdnWrite[line] = (path.fdnWrite[line] + 1) % fdnDelaySamples_[line];
    return (a * 0.42F + b * 0.31F + c * 0.23F + d * 0.17F) * 0.34F;
}

float RealtimeEngineAudio::saturateCollector(float sample, float drive) const noexcept {
    // Symmetric, identity-through-normal-range collector compression.  Presets
    // vary the acoustic drive slightly but cannot introduce a DC component.
    const auto driveGain = std::lerp(0.92F, 1.18F,
        std::clamp((drive - 0.35F) / (3.4F - 0.35F), 0.0F, 1.0F));
    const auto pushed = finiteState(sample * driveGain);
    constexpr float knee = 1.40F;
    constexpr float ceiling = 3.20F;
    const auto magnitude = std::abs(pushed);
    if (magnitude <= knee) return pushed;
    const auto shaped = knee + (ceiling - knee)
        * std::tanh((magnitude - knee) / (ceiling - knee));
    return std::copysign(shaped, pushed);
}

float RealtimeEngineAudio::softLimit(float sample) noexcept {
    // Identity below the knee, smooth compression above, with headroom for the
    // oversampling down-filter's small inter-sample overshoot.
    constexpr float knee = 0.82F;
    constexpr float ceiling = 0.985F;
    const auto magnitude = std::abs(sample);
    if (magnitude <= knee) return sample;
    const auto over = (magnitude - knee) / (ceiling - knee);
    const auto shaped = knee + (ceiling - knee) * std::tanh(over);
    return std::copysign(shaped, sample);
}

float RealtimeEngineAudio::noise() noexcept {
    noiseState_ ^= noiseState_ << 13U; noiseState_ ^= noiseState_ >> 17U; noiseState_ ^= noiseState_ << 5U;
    return static_cast<float>(noiseState_) / static_cast<float>(0xffffffffU) * 2.0F - 1.0F;
}
} // namespace enginelab
