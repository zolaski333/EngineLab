#include "OfflineRenderChain.hpp"

#include <enginelab/audio/ImpulseResponseLoader.hpp>

#include <juce_audio_basics/juce_audio_basics.h>

#include <algorithm>
#include <utility>

namespace enginelab::offline {
namespace {

[[nodiscard]] std::string impulseResponseErrorName(
    ImpulseResponseLoadError error) {
    switch (error) {
    case ImpulseResponseLoadError::none:
        return "unspecified failure";
    case ImpulseResponseLoadError::missingFile:
        return "file not found";
    case ImpulseResponseLoadError::unsupportedOrCorrupt:
        return "unsupported or corrupt";
    case ImpulseResponseLoadError::invalidMetadata:
        return "invalid metadata";
    case ImpulseResponseLoadError::empty:
        return "empty";
    case ImpulseResponseLoadError::readFailure:
        return "read failure";
    }
    return "unknown failure";
}

} // namespace

RenderChain::RenderChain(
    const EngineConfig& config, const OfflineAudioMix& mix)
    : exhaust_(ExhaustGraph::makeForEngine(config)) {
    simulator_ = std::make_unique<EngineSimulator>(
        config, ecu_, physics_, events_, exhaust_);
    simulator_->setPressureSamplingEnabled(true);

    eventQueue_ = std::make_unique<FiringEventQueue>();
    pressureQueue_ = std::make_unique<CylinderPressureQueue>();
    exhaustAcousticQueue_ = std::make_unique<ExhaustAcousticQueue>();
    audioConfiguration_ = std::make_unique<EngineRuntime>(config);
    audioConfiguration_->setAudioVolume(mix.volume);
    audioConfiguration_->setAudioConvolution(mix.convolution);
    audioConfiguration_->setHighFrequencyGain(mix.highFrequencyGain);
    audioConfiguration_->setLowFrequencyGain(mix.lowFrequencyGain);
    audioConfiguration_->setLowFrequencyNoise(mix.lowFrequencyNoise);
    audioConfiguration_->setHighFrequencyNoise(mix.highFrequencyNoise);
    audioConfiguration_->setCombustionGain(mix.combustionGain);
    audioConfiguration_->setExhaustGain(mix.exhaustGain);
    audioConfiguration_->setIntakeGain(mix.intakeGain);
    audioConfiguration_->setMechanicalGain(mix.mechanicalGain);
    audioConfiguration_->setStereoWidth(mix.stereoWidth);
    audioConfiguration_->setOutletJetGain(mix.outletJetGain);
    audioConfiguration_->setSaturationDrive(mix.saturationDrive);
    audioConfiguration_->setSaturationPlacement(mix.saturationPlacement);
    audioConfiguration_->applyAudioVoicing(mix);

    renderer_ = std::make_unique<RealtimeEngineAudio>(
        *eventQueue_, audioConfiguration_->audioState(),
        pressureQueue_.get(), &audioConfiguration_->exhaustGraph(),
        &audioConfiguration_->engineConfig(),
        exhaustAcousticQueue_.get());
}

void RenderChain::loadAuthoredImpulseResponses(
    const EngineConfig& config,
    const std::filesystem::path& assetRoot,
    std::vector<std::string>& warnings,
    std::size_t& authored, std::size_t& loaded) {
    const auto pathCount = config.exhaustPaths.size();
    for (std::size_t pathIndex = 0; pathIndex < pathCount; ++pathIndex) {
        const auto& configured =
            config.exhaustPaths[pathIndex].impulseResponsePath;
        if (configured.empty()) continue;
        ++authored;
        if (pathIndex >= RealtimeConvolutionBank::maximumPaths) {
            warnings.push_back(
                "Impulse response path " + std::to_string(pathIndex + 1)
                + " exceeds the eight-path renderer limit.");
            continue;
        }
        auto fullPath = std::filesystem::path(configured);
        if (fullPath.is_relative())
            fullPath = assetRoot / fullPath;
        auto decoded = loadImpulseResponseFile(
            juce::File(fullPath.string()), 262'144);
        if (!decoded.ok()) {
            warnings.push_back(
                "Impulse response not loaded ("
                + impulseResponseErrorName(decoded.error)
                + "): " + fullPath.string());
            continue;
        }
        if (decoded.truncated) {
            warnings.push_back(
                "Impulse response truncated to 262144 samples: "
                + fullPath.string());
        }
        renderer_->setImpulseResponse(
            std::move(decoded.samples), decoded.sampleRateHz, pathIndex);
        ++loaded;
    }
}

void RenderChain::forward(
    const SimulationFrame& frame, bool starterEngaged,
    double drivelineLoad, double realtimeSeconds, double dt,
    DroppedTelemetry& dropped) {
    const auto simulationStart = frame.state.simulationTimeSeconds - dt;
    for (std::size_t eventIndex = 0;
         eventIndex < frame.firingEventCount; ++eventIndex) {
        auto event = frame.firingEvents[eventIndex];
        const auto fraction = std::clamp(
            (event.timeSeconds - simulationStart) / dt, 0.0, 1.0);
        event.timeSeconds = realtimeSeconds + fraction * dt;
        if (!eventQueue_->tryPush(event))
            ++dropped.firingEvents;
    }
    CylinderPressureSample pressureSample;
    while (simulator_->tryPopCylinderPressureSample(pressureSample)) {
        const auto fraction = std::clamp(
            (pressureSample.timeSeconds - simulationStart) / dt, 0.0, 1.0);
        pressureSample.timeSeconds = realtimeSeconds + fraction * dt;
        if (!pressureQueue_->tryPush(pressureSample))
            ++dropped.pressureSamples;
    }
    ExhaustAcousticSample acousticSample;
    while (simulator_->tryPopExhaustAcousticSample(acousticSample)) {
        const auto fraction = std::clamp(
            (acousticSample.timeSeconds - simulationStart) / dt, 0.0, 1.0);
        acousticSample.timeSeconds = realtimeSeconds + fraction * dt;
        for (std::size_t eventIndex = 0;
             eventIndex < acousticSample.reactionEventCount; ++eventIndex) {
            const auto eventFraction = std::clamp(
                (acousticSample.reactionEvents[eventIndex].timeSeconds
                    - simulationStart) / dt,
                0.0, 1.0);
            acousticSample.reactionEvents[eventIndex].timeSeconds =
                realtimeSeconds + eventFraction * dt;
        }
        if (!exhaustAcousticQueue_->tryPush(acousticSample))
            ++dropped.exhaustAcousticSamples;
    }

    publishAudioFrame(
        audioConfiguration_->audioState(), frame.state,
        { false, starterEngaged, drivelineLoad, 1.0 });
    audioConfiguration_->audioState().producerTimeNanoseconds.store(
        static_cast<std::uint64_t>(
            std::max(0.0, realtimeSeconds + dt) * 1.0e9),
        std::memory_order_release);
}

} // namespace enginelab::offline
