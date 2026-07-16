#include <enginelab/audio/RealtimeEngineAudio.hpp>

#include <juce_audio_basics/juce_audio_basics.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <span>
#include <string>
#include <vector>

namespace {

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

enginelab::FiringEvent eventFixture() {
    enginelab::FiringEvent event;
    event.timeSeconds = 0.0;
    event.intensity = 0.9F;
    event.pressureEstimateBar = 62.0F;
    event.combustionDurationMs = 5.0F;
    event.airFuelRatio = 13.1F;
    return event;
}

std::vector<float> renderEvent(double sampleRate, int preparedBlockSize,
                               int sampleCount, bool renderAsOneCall) {
    enginelab::FiringEventQueue queue;
    enginelab::RealtimeAudioState state;
    state.exhaustGain.store(0.0F);
    state.intakeGain.store(0.0F);
    state.mechanicalGain.store(0.0F);
    state.convolution.store(0.0F);
    auto renderer = std::make_unique<enginelab::RealtimeEngineAudio>(queue, state);
    renderer->prepare(sampleRate, preparedBlockSize);
    require(queue.tryPush(eventFixture()), "event fixture must enter the realtime queue");
    juce::AudioBuffer<float> output(2, sampleCount);
    if (renderAsOneCall) {
        renderer->render(output, 0, sampleCount);
    } else {
        for (int offset = 0; offset < sampleCount; offset += preparedBlockSize)
            renderer->render(output, offset, std::min(preparedBlockSize, sampleCount - offset));
    }
    std::vector<float> result(static_cast<std::size_t>(sampleCount));
    std::copy_n(output.getReadPointer(0), sampleCount, result.begin());
    return result;
}

std::size_t firstAudibleSample(const std::vector<float>& samples) {
    const auto found = std::find_if(samples.begin(), samples.end(), [](float value) {
        return std::abs(value) > 1.0e-6F;
    });
    return static_cast<std::size_t>(std::distance(samples.begin(), found));
}

std::vector<float> renderContinuousPressure(std::uint32_t cylinderPath,
                                            std::span<const float> pathOneIr,
                                            float ambientPressureKpa = 101.325F,
                                            float cylinderTransmissionGain = 1.0F) {
    enginelab::FiringEventQueue eventQueue;
    enginelab::RealtimeAudioState state;
    state.ambientPressureKpa.store(ambientPressureKpa);
    state.exhaustPathCount.store(2);
    state.cylinderExhaustPathIndex[0].store(cylinderPath);
    state.cylinderExhaustGain[0].store(cylinderTransmissionGain);
    state.combustionGain.store(0.0F);
    state.exhaustGain.store(1.0F);
    state.intakeGain.store(0.0F);
    state.mechanicalGain.store(0.0F);
    state.convolution.store(1.0F);
    auto pressureQueue = std::make_unique<enginelab::CylinderPressureQueue>();
    enginelab::CylinderPressureSample first;
    first.timeSeconds = 0.0;
    first.cylinderCount = 1;
    first.pressureBar[0] = ambientPressureKpa * 0.01F;
    first.exhaustRunnerPressureKpa[0] = ambientPressureKpa + 78.0F;
    first.exhaustFlowMgPerCycle[0] = 75.0F;
    first.exhaustPathIndex[0] = static_cast<std::uint8_t>(cylinderPath);
    auto second = first;
    second.timeSeconds = 0.004;
    second.exhaustRunnerPressureKpa[0] = ambientPressureKpa;
    require(pressureQueue->tryPush(first) && pressureQueue->tryPush(second),
            "pressure fixtures must enter the realtime queue");
    auto renderer = std::make_unique<enginelab::RealtimeEngineAudio>(
        eventQueue, state, pressureQueue.get());
    const std::array<float, 1> directIr { 1.0F };
    renderer->setImpulseResponse(directIr, 48'000.0, 0);
    renderer->setImpulseResponse(pathOneIr, 48'000.0, 1);
    renderer->prepare(48'000.0, 256);
    juce::AudioBuffer<float> output(2, 2'400);
    renderer->render(output, 0, output.getNumSamples());
    std::vector<float> result(static_cast<std::size_t>(output.getNumSamples()));
    std::copy_n(output.getReadPointer(0), output.getNumSamples(), result.begin());
    return result;
}

enginelab::EngineConfig customRuntimeExhaustFixture() {
    auto config = enginelab::makeDefaultInlineFour();
    auto& path = config.exhaustPaths.front();
    path.audioVolume = 0.62;
    enginelab::ExhaustNetworkConfig network;
    for (std::size_t index = 0; index < config.cylinders.size(); ++index) {
        enginelab::ExhaustComponentConfig primary;
        primary.id = static_cast<std::uint32_t>(100 + index);
        primary.type = enginelab::ExhaustComponentType::pipe;
        primary.lengthMm = 390.0 + static_cast<double>(index) * 20.0;
        primary.diameterMm = 41.0;
        network.components.push_back(primary);
        network.cylinderConnections.push_back({ config.cylinders[index].id, primary.id });
        network.connections.push_back({ primary.id, 200 });
    }
    enginelab::ExhaustComponentConfig merge;
    merge.id = 200;
    merge.type = enginelab::ExhaustComponentType::merge;
    merge.diameterMm = 57.0;
    network.components.push_back(merge);
    enginelab::ExhaustComponentConfig muffler;
    muffler.id = 300;
    muffler.type = enginelab::ExhaustComponentType::muffler;
    muffler.lengthMm = 520.0;
    muffler.diameterMm = 60.0;
    muffler.restriction = 0.42;
    muffler.acousticGain = 0.54;
    network.components.push_back(muffler);
    enginelab::ExhaustComponentConfig outlet;
    outlet.id = 400;
    outlet.type = enginelab::ExhaustComponentType::outlet;
    outlet.lengthMm = 210.0;
    outlet.diameterMm = 67.0;
    network.components.push_back(outlet);
    network.connections.push_back({ 200, 300 });
    network.connections.push_back({ 300, 400 });
    path.network = std::move(network);
    enginelab::normaliseEngineConfig(config);
    return config;
}

double absoluteDifference(const std::vector<float>& left, const std::vector<float>& right) {
    double difference = 0.0;
    for (std::size_t index = 0; index < std::min(left.size(), right.size()); ++index)
        difference += std::abs(static_cast<double>(left[index] - right[index]));
    return difference;
}

struct WaveformMetrics final {
    double mean {};
    double rms {};
    double peak {};
    std::size_t longestFlatTop {};
    bool finite { true };
};

WaveformMetrics analyseWaveform(std::span<const float> samples, std::size_t begin = 0) {
    WaveformMetrics metrics;
    begin = std::min(begin, samples.size());
    const auto count = samples.size() - begin;
    if (count == 0) return metrics;

    double sum = 0.0;
    double squareSum = 0.0;
    for (std::size_t index = begin; index < samples.size(); ++index) {
        const auto value = static_cast<double>(samples[index]);
        metrics.finite = metrics.finite && std::isfinite(value);
        sum += value;
        squareSum += value * value;
        metrics.peak = std::max(metrics.peak, std::abs(value));
    }
    metrics.mean = sum / static_cast<double>(count);
    metrics.rms = std::sqrt(squareSum / static_cast<double>(count));

    // A legitimate rounded peak can have a couple of low-slope samples. A
    // hard-clipped signal instead repeats essentially the same near-full-scale
    // value for a longer run. Use both an absolute and a signal-relative
    // tolerance so this remains meaningful for float output on every platform.
    const auto flatTolerance = std::max(2.0e-6, metrics.peak * 2.0e-5);
    const auto highLevel = std::max(0.90, metrics.peak * 0.995);
    std::size_t flatRun = 0;
    for (std::size_t index = begin + 1; index < samples.size(); ++index) {
        const auto current = static_cast<double>(samples[index]);
        const auto previous = static_cast<double>(samples[index - 1]);
        const auto isFlatTop = std::abs(current) >= highLevel
            && std::signbit(current) == std::signbit(previous)
            && std::abs(current - previous) <= flatTolerance;
        flatRun = isFlatTop ? flatRun + 1 : 0;
        metrics.longestFlatTop = std::max(metrics.longestFlatTop, flatRun);
    }
    return metrics;
}

std::array<std::vector<float>, 2> renderMaximumMix() {
    constexpr double sampleRate = 48'000.0;
    constexpr int blockSize = 256;
    constexpr int sampleCount = 48'000;

    enginelab::FiringEventQueue eventQueue;
    enginelab::RealtimeAudioState state;
    state.volume.store(2.0F);
    state.convolution.store(1.0F);
    state.highFrequencyGain.store(2.5F);
    state.lowFrequencyNoise.store(1.5F);
    state.highFrequencyNoise.store(1.5F);
    state.combustionGain.store(2.0F);
    state.exhaustGain.store(2.0F);
    state.intakeGain.store(2.0F);
    state.mechanicalGain.store(2.0F);
    state.rpm.store(7'200.0F);
    state.redlineRpm.store(7'200.0F);
    state.throttle.store(1.0F);
    state.load.store(1.0F);
    state.mechanicalStress.store(1.0F);
    state.starter.store(1.0F);
    state.manifoldPressureKpa.store(45.0F);
    state.exhaustPressureKpa.store(260.0F);
    state.exhaustFlowGramsPerSecond.store(260.0F);
    state.intakeRunnerResonanceHz.store(780.0F);
    state.intakeRunnerAmplitudeKpa.store(10.0F);
    state.forcedInductionKind.store(1);
    state.forcedInductionShaftRpm.store(125'000.0F);
    state.boostPressureRatio.store(2.5F);
    state.wastegateOpening.store(1.0F);

    for (std::uint32_t index = 0; index < 32; ++index) {
        auto event = eventFixture();
        event.timeSeconds = 0.050;
        event.cylinderId = index;
        event.stereoPosition = -0.8F + 1.6F * static_cast<float>(index) / 31.0F;
        event.exhaustDelaySeconds = 0.0005F;
        event.exhaustFlowMgPerCycle = 135.0F;
        event.exhaustRunnerPressureKpa = 245.0F;
        event.exhaustResonanceHz = 950.0F;
        require(eventQueue.tryPush(event), "stress events must enter the realtime queue");
    }

    auto pressureQueue = std::make_unique<enginelab::CylinderPressureQueue>();
    enginelab::CylinderPressureSample pressure;
    pressure.timeSeconds = 0.040;
    pressure.cylinderCount = 4;
    for (std::size_t index = 0; index < pressure.cylinderCount; ++index) {
        pressure.pressureBar[index] = 70.0F;
        pressure.exhaustRunnerPressureKpa[index] = 245.0F;
        pressure.exhaustFlowMgPerCycle[index] = 135.0F;
    }
    auto ambient = pressure;
    ambient.timeSeconds = 0.080;
    for (std::size_t index = 0; index < ambient.cylinderCount; ++index) {
        ambient.pressureBar[index] = 1.01325F;
        ambient.exhaustRunnerPressureKpa[index] = 101.325F;
        ambient.exhaustFlowMgPerCycle[index] = 0.0F;
    }
    require(pressureQueue->tryPush(pressure) && pressureQueue->tryPush(ambient),
            "stress pressure samples must enter the realtime queue");

    auto renderer = std::make_unique<enginelab::RealtimeEngineAudio>(
        eventQueue, state, pressureQueue.get());
    const std::array<float, 1> directIr { 1.0F };
    renderer->setImpulseResponse(directIr);
    renderer->prepare(sampleRate, blockSize);

    juce::AudioBuffer<float> output(2, sampleCount);
    for (int offset = 0; offset < sampleCount; offset += blockSize)
        renderer->render(output, offset, std::min(blockSize, sampleCount - offset));

    std::array<std::vector<float>, 2> channels;
    for (std::size_t channel = 0; channel < channels.size(); ++channel) {
        channels[channel].resize(sampleCount);
        std::copy_n(output.getReadPointer(static_cast<int>(channel)), sampleCount,
                    channels[channel].begin());
    }
    return channels;
}

void outputQualityRegression() {
    const auto channels = renderMaximumMix();
    for (const auto& channel : channels) {
        const auto complete = analyseWaveform(channel);
        require(complete.finite, "maximum mixer settings must produce only finite samples");
        require(complete.peak <= 1.00001,
                "the final output stage must remain within full scale at maximum mixer settings");
        require(complete.longestFlatTop <= 8,
                "the final output must not contain a sustained flat clipping plateau");

        // Ignore the deliberately dense initial excitation. Once it has passed,
        // nonlinear stages and feedback tails must settle around acoustic zero.
        const auto settled = analyseWaveform(channel, 6'000);
        const auto allowedDc = std::max(0.0025, settled.rms * 0.08);
        require(std::abs(settled.mean) <= allowedDc,
                "the post-transient output must not retain a measurable DC offset");
        require(settled.rms > 1.0e-5,
                "the stress fixture must remain audible while quality metrics are evaluated");
    }
}

void latencyAndBlockSizeRegression() {
    const auto at48k = renderEvent(48'000.0, 256, 2'400, true);
    const auto at96k = renderEvent(96'000.0, 256, 4'800, true);
    const auto onset48 = firstAudibleSample(at48k);
    const auto onset96 = firstAudibleSample(at96k);
    require(onset48 < at48k.size() && onset96 < at96k.size(),
            "a scheduled firing event must become audible");
    require(std::abs(static_cast<double>(onset48) / 48'000.0 - 0.020) < 0.001,
            "event latency must be applied exactly once at 48 kHz");
    require(std::abs(static_cast<double>(onset96) / 96'000.0 - 0.020) < 0.001,
            "event latency must be invariant at 96 kHz");

    const auto oversized = renderEvent(48'000.0, 256, 2'400, true);
    const auto chunked = renderEvent(48'000.0, 256, 2'400, false);
    require(absoluteDifference(oversized, chunked) < 1.0e-5,
            "an oversized host block must be rendered as lossless prepared-size chunks");

    for (const auto sampleRate : { 48'000.0, 96'000.0, 192'000.0 }) {
        for (const auto blockSize : { 1'024, 2'048 }) {
            enginelab::FiringEventQueue queue;
            enginelab::RealtimeAudioState state;
            auto renderer = std::make_unique<enginelab::RealtimeEngineAudio>(queue, state);
            renderer->prepare(sampleRate, blockSize);
            const auto requiredLookahead = std::max(0.020,
                static_cast<double>(blockSize) / sampleRate + 0.005);
            require(renderer->eventLatencySeconds() + 1.0e-12 >= requiredLookahead,
                "look-ahead must cover a complete 1024/2048-sample callback plus margin");
            require(static_cast<double>(enginelab::CylinderPressureQueue::usableCapacity) / 96'000.0
                    > renderer->eventLatencySeconds() + 0.005,
                "pressure queue retention must exceed the adaptive look-ahead");
        }
    }
}

void runnerDelaySampleRateRegression() {
    constexpr double requestedDelaySeconds = 0.080;
    for (const auto sampleRate : { 48'000.0, 96'000.0, 192'000.0 }) {
        const auto samples = enginelab::RealtimeEngineAudio::runnerDelaySamples(
            requestedDelaySeconds, sampleRate);
        const auto realisedDelaySeconds = static_cast<double>(samples) / sampleRate;
        require(std::abs(realisedDelaySeconds - requestedDelaySeconds) <= 0.5 / sampleRate,
                "80 ms runner delay must remain invariant at 48/96/192 kHz");
    }
    require(enginelab::RealtimeEngineAudio::runnerDelaySamples(0.080, 192'000.0) == 15'360,
            "192 kHz runner delay must not be truncated by circular-buffer capacity");
}

void exhaustPathIsolationRegression() {
    std::array<float, 128> delayedIr {};
    delayedIr[96] = 1.0F;
    std::array<float, 128> unrelatedIr {};
    unrelatedIr[0] = -0.7F;
    unrelatedIr[31] = 1.0F;

    const auto pathZeroWithDelayedPathOne = renderContinuousPressure(0, delayedIr);
    const auto pathZeroWithDifferentPathOne = renderContinuousPressure(0, unrelatedIr);
    require(absoluteDifference(pathZeroWithDelayedPathOne, pathZeroWithDifferentPathOne) < 1.0e-5,
            "an unused exhaust path IR must not colour another collector's continuous pressure");

    const auto pathOne = renderContinuousPressure(1, delayedIr);
    require(absoluteDifference(pathZeroWithDelayedPathOne, pathOne) > 0.01,
            "continuous runner pressure must route through its own exhaust path and IR");

    const auto transmitted = renderContinuousPressure(0, delayedIr, 101.325F, 1.0F);
    const auto mutedByGraph = renderContinuousPressure(0, delayedIr, 101.325F, 0.0F);
    require(absoluteDifference(transmitted, mutedByGraph) > 0.01,
            "per-cylinder graph transmission must affect continuous runner pressure");
    require(std::all_of(mutedByGraph.begin(), mutedByGraph.end(), [](float value) {
        return std::abs(value) < 1.0e-6F;
    }), "a zero graph transmission must fully mute the continuous pressure impulse");
}

void customGraphRuntimeTelemetryRegression() {
    const auto config = customRuntimeExhaustFixture();
    require(!enginelab::validateEngineConfig(config).has_value(),
            "custom runtime exhaust fixture must validate");
    const auto graph = enginelab::ExhaustGraph::makeForEngine(config);
    enginelab::EngineRuntime runtime(config);
    auto& state = runtime.audioState();
    double downstreamLengthSum = 0.0;
    double restrictionSum = 0.0;
    for (std::size_t index = 0; index < config.cylinders.size(); ++index) {
        const auto metrics = graph.acousticsForCylinder(config.cylinders[index].id);
        const auto flow = graph.cylinderFlowProperties(config.cylinders[index].id);
        require(metrics.routeCount == 1, "custom runtime cylinder route missing");
        require(std::abs(state.cylinderExhaustGain[index].load() - metrics.transmissionGain) < 1.0e-6,
                "runtime did not publish the graph's cylinder transmission");
        const auto expectedRunnerDelay = flow.runnerLengthMm
            / (graph.referenceWaveSpeedMps() * 1'000.0);
        require(std::abs(state.runnerDelaySeconds[index].load() - expectedRunnerDelay) < 1.0e-7,
                "runtime primary delay was not derived from the custom DAG inlet");
        require(state.cylinderExhaustPathIndex[index].load() == metrics.pathIndex,
                "runtime path routing differs from the compiled custom DAG");
        require(state.cylinderExhaustAreaM2[index].load() > 0.0F,
                "runtime did not publish the compiled primary acoustic admittance");
        downstreamLengthSum += std::max(0.0, metrics.meanLengthMm - flow.runnerLengthMm);
        restrictionSum += metrics.equivalentRestriction;
    }
    const auto cylinderCount = static_cast<double>(config.cylinders.size());
    const auto expectedReflection = (downstreamLengthSum / cylinderCount)
        / (graph.referenceWaveSpeedMps() * 1'000.0);
    const auto expectedOpenness = 1.0 / std::sqrt(
        1.0 + 1.35 * (restrictionSum / cylinderCount));
    require(std::abs(state.exhaustPathReflectionSeconds[0].load() - expectedReflection) < 1.0e-7,
            "path reflection was not derived from custom route lengths");
    require(std::abs(state.exhaustPathOpenness[0].load() - expectedOpenness) < 1.0e-6,
            "path openness was not derived from custom equivalent restriction");
    require(std::abs(state.exhaustPathGain[0].load() - 1.0F) < 1.0e-7F,
            "configured exhaust gain must not be multiplied again at path output");
    require(state.exhaustPathOutletAreaM2[0].load() > 0.0F,
            "runtime did not publish the compiled outlet acoustic admittance");
}

void ambientPressureRegression() {
    const std::array<float, 1> directIr { 1.0F };
    enginelab::FiringEventQueue eventQueue;
    enginelab::RealtimeAudioState state;
    state.ambientPressureKpa.store(80.0F);
    state.intakeGain.store(0.0F);
    state.mechanicalGain.store(0.0F);
    auto pressureQueue = std::make_unique<enginelab::CylinderPressureQueue>();
    enginelab::CylinderPressureSample sample;
    sample.timeSeconds = 0.0;
    sample.cylinderCount = 1;
    sample.pressureBar[0] = 0.8F;
    sample.exhaustRunnerPressureKpa[0] = 80.0F;
    auto next = sample;
    next.timeSeconds = 0.004;
    require(pressureQueue->tryPush(sample) && pressureQueue->tryPush(next),
            "ambient pressure fixtures must enter the realtime queue");
    auto renderer = std::make_unique<enginelab::RealtimeEngineAudio>(
        eventQueue, state, pressureQueue.get());
    renderer->setImpulseResponse(directIr);
    renderer->prepare(48'000.0, 256);
    juce::AudioBuffer<float> output(2, 512);
    renderer->render(output, 0, output.getNumSamples());
    require(output.getMagnitude(0, 0, output.getNumSamples()) < 1.0e-6F,
            "configured ambient pressure must be acoustic zero, including at altitude");
}

} // namespace

int main() {
    try {
        latencyAndBlockSizeRegression();
        runnerDelaySampleRateRegression();
        exhaustPathIsolationRegression();
        customGraphRuntimeTelemetryRegression();
        ambientPressureRegression();
        outputQualityRegression();
        std::cout << "Realtime audio/runtime regression tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Realtime regression failure: " << error.what() << '\n';
        return 1;
    }
}
