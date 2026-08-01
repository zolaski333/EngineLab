#include <enginelab/audio/AcousticMonitorCalibration.hpp>
#include <enginelab/audio/AcousticExhaustNetwork.hpp>
#include <enginelab/audio/AcousticIntakeNetwork.hpp>
#include <enginelab/audio/BoundaryReconstructionFilter.hpp>
#include <enginelab/audio/ExhaustJetNoise.hpp>
#include <enginelab/audio/ForcedInductionAcoustics.hpp>
#include <enginelab/audio/FreeFieldObserver.hpp>
#include <enginelab/audio/NonlinearDuctAcoustics.hpp>
#include <enginelab/audio/RealtimeEngineAudio.hpp>
#include <enginelab/audio/StructuralModalRadiator.hpp>
#include <enginelab/audio/DuctModeCutoff.hpp>
#include <enginelab/audio/DuctWallLoss.hpp>
#include <enginelab/audio/ExpansionChamberMuffler.hpp>
#include <enginelab/audio/PipeRadiationModel.hpp>
#include <enginelab/audio/ValvePortTermination.hpp>
#include <enginelab/audio/ValveFlowAcousticSource.hpp>
#include <enginelab/foundation/ExhaustGasAcoustics.hpp>

#include <juce_audio_basics/juce_audio_basics.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <complex>
#include <iostream>
#include <limits>
#include <memory>
#include <numbers>
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
                               int sampleCount, bool renderAsOneCall,
                               const enginelab::FiringEvent& event = eventFixture()) {
    enginelab::FiringEventQueue queue;
    enginelab::RealtimeAudioState state;
    state.exhaustGain.store(0.0F);
    state.intakeGain.store(0.0F);
    state.mechanicalGain.store(0.0F);
    state.convolution.store(0.0F);
    auto renderer = std::make_unique<enginelab::RealtimeEngineAudio>(queue, state);
    renderer->prepare(sampleRate, preparedBlockSize);
    require(queue.tryPush(event), "event fixture must enter the realtime queue");
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

struct PhysicalExhaustFixture final {
    double sampleRateHz { 48'000.0 };
    float tailpipeDelaySeconds { 0.001F };
    float transientMassFlowKgPerSecond { 0.080F };
    float cylinderTransmissionGain { 1.0F };
    float pathGain { 1.0F };
    float highFrequencyNoise { 0.0F };
    int exhaustPreset { 0 };
    bool transient { true };
    bool proceduralExhaustEvent { false };
};

std::vector<float> renderPhysicalExhaust(const PhysicalExhaustFixture& fixture) {
    constexpr int blockSize = 256;
    const auto sampleCount = static_cast<int>(std::lround(
        fixture.sampleRateHz * 0.050));
    enginelab::FiringEventQueue eventQueue;
    enginelab::RealtimeAudioState state;
    state.cylinderCount.store(1.0F);
    state.exhaustPathCount.store(1);
    state.cylinderExhaustAreaM2[0].store(0.00150F);
    state.cylinderExhaustGain[0].store(fixture.cylinderTransmissionGain);
    state.cylinderExhaustPathIndex[0].store(0);
    state.runnerDelaySeconds[0].store(0.00050F);
    state.exhaustPathOutletAreaM2[0].store(0.00320F);
    state.exhaustPathReflectionSeconds[0].store(fixture.tailpipeDelaySeconds);
    state.exhaustPathGain[0].store(fixture.pathGain);
    constexpr float exhaustTemperatureC = 600.0F;
    state.exhaustTemperatureC.store(exhaustTemperatureC);
    state.exhaustReferenceSoundSpeedMps.store(static_cast<float>(
        enginelab::exhaustSpeedOfSoundMps(exhaustTemperatureC)));
    state.exhaustPreset.store(fixture.exhaustPreset);
    state.highFrequencyNoise.store(fixture.highFrequencyNoise);
    state.combustionGain.store(0.0F);
    state.intakeGain.store(0.0F);
    state.mechanicalGain.store(0.0F);
    state.exhaustGain.store(1.0F);
    state.convolution.store(0.0F);

    if (fixture.proceduralExhaustEvent) {
        auto event = eventFixture();
        event.timeSeconds = -0.008;
        event.exhaustDelaySeconds = 0.0001F;
        event.exhaustFlowMgPerCycle = 400.0F;
        event.exhaustRunnerPressureKpa = 500.0F;
        event.exhaustResonanceHz = 1'900.0F;
        require(eventQueue.tryPush(event),
            "procedural exhaust proof event must enter its queue");
    }

    auto pressureQueue = std::make_unique<enginelab::CylinderPressureQueue>();
    const auto boundary = [](double timeSeconds, float pressureKpa,
                             float massFlowKgPerSecond, float conductanceAreaM2) {
        enginelab::CylinderPressureSample sample;
        sample.timeSeconds = timeSeconds;
        sample.cylinderCount = 1;
        sample.pressureBar[0] = 1.01325F;
        sample.exhaustRunnerPressureKpa[0] = pressureKpa;
        // This fixture authors one self-consistent boundary instant directly,
        // with no multirate reconstruction between network knots to make the two
        // published flows differ. The valve-plane flow and its acoustic partner
        // therefore coincide here, and both are set so the characteristic source
        // sees the signed flow this case exists to exercise.
        sample.exhaustMassFlowKgPerSecond[0] = massFlowKgPerSecond;
        sample.exhaustAcousticMassFlowKgPerSecond[0] = massFlowKgPerSecond;
        sample.exhaustPortDensityKgPerM3[0] = 0.65F;
        sample.exhaustPortSpeedOfSoundMps[0] = 540.0F;
        sample.exhaustValveConductanceAreaM2[0] = conductanceAreaM2;
        sample.exhaustPathIndex[0] = 0;
        sample.thermoacousticBoundaryValid[0] = 1;
        // Deliberately extreme legacy observables: a correct physical render
        // never consumes them after seeing the SI validity flag.
        sample.exhaustFlowMgPerCycle[0] = 900.0F;
        sample.exhaustValveOpening[0] = 1.0F;
        return sample;
    };
    const auto ambient = boundary(-0.015, 101.325F, 0.0F, 0.0F);
    require(pressureQueue->tryPush(ambient),
        "physical ambient boundary must enter its queue");
    if (fixture.transient) {
        require(pressureQueue->tryPush(boundary(-0.008, 151.325F,
                    fixture.transientMassFlowKgPerSecond, 0.00045F))
                && pressureQueue->tryPush(boundary(-0.005, 124.0F,
                    fixture.transientMassFlowKgPerSecond * 0.35F, 0.00030F))
                && pressureQueue->tryPush(boundary(0.000, 101.325F, 0.0F, 0.0F)),
            "physical transient boundaries must enter their queue");
    } else {
        require(pressureQueue->tryPush(boundary(0.010, 101.325F, 0.0F, 0.0F)),
            "physical steady boundary must enter its queue");
    }

    auto renderer = std::make_unique<enginelab::RealtimeEngineAudio>(
        eventQueue, state, pressureQueue.get());
    renderer->prepare(fixture.sampleRateHz, blockSize);
    juce::AudioBuffer<float> output(2, sampleCount);
    for (int offset = 0; offset < sampleCount; offset += blockSize)
        renderer->render(output, offset,
            std::min(blockSize, sampleCount - offset));
    std::vector<float> result(static_cast<std::size_t>(sampleCount));
    std::copy_n(output.getReadPointer(0), sampleCount, result.begin());
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

void branchedAcousticTopologyRegression() {
    auto config = enginelab::makeDefaultInlineFour();
    auto& path = config.exhaustPaths.front();
    enginelab::ExhaustNetworkConfig network;
    for (std::size_t index = 0; index < config.cylinders.size(); ++index) {
        enginelab::ExhaustComponentConfig primary;
        primary.id = static_cast<std::uint32_t>(100 + index);
        primary.type = enginelab::ExhaustComponentType::pipe;
        primary.lengthMm = 360.0 + 20.0 * static_cast<double>(index);
        primary.diameterMm = 40.0;
        network.components.push_back(primary);
        network.cylinderConnections.push_back({
            config.cylinders[index].id, primary.id });
        network.connections.push_back({ primary.id, 200 });
    }
    // Both branches carry an authored trunk: the collector's common pipe runs
    // downstream of the merge, the split's common pipe upstream of the splitter.
    enginelab::ExhaustComponentConfig merge;
    merge.id = 200;
    merge.type = enginelab::ExhaustComponentType::merge;
    merge.diameterMm = 55.0;
    merge.lengthMm = 130.0;
    network.components.push_back(merge);
    enginelab::ExhaustComponentConfig splitter;
    splitter.id = 210;
    splitter.type = enginelab::ExhaustComponentType::splitter;
    splitter.diameterMm = 55.0;
    splitter.lengthMm = 90.0;
    network.components.push_back(splitter);
    for (const auto [id, length] : std::array {
             std::pair { 300U, 240.0 }, std::pair { 301U, 510.0 } }) {
        enginelab::ExhaustComponentConfig outlet;
        outlet.id = id;
        outlet.type = enginelab::ExhaustComponentType::outlet;
        outlet.lengthMm = length;
        outlet.diameterMm = id == 300U ? 52.0 : 46.0;
        network.components.push_back(outlet);
        network.connections.push_back({ 210, id });
    }
    network.connections.push_back({ 200, 210 });
    path.network = std::move(network);
    enginelab::normaliseEngineConfig(config);

    const auto graph = enginelab::ExhaustGraph::makeForEngine(config);
    std::array<std::uint32_t, 4> cylinderIds {};
    for (std::size_t index = 0; index < cylinderIds.size(); ++index)
        cylinderIds[index] = config.cylinders[index].id;
    enginelab::AcousticExhaustNetwork acoustics(graph, cylinderIds);
    require(acoustics.valid(),
        "a valid branched exhaust DAG must compile for audio");
    // Four primaries and two outlets, plus one trunk for each authored branch
    // length. A branch is a scattering point *and* a pipe; dropping the pipe
    // deleted the collector from the waveguide and let the primaries scatter
    // straight into whatever followed the merge.
    require(acoustics.ductCount() == 8 && acoustics.outletCount() == 2
            && acoustics.junctionCount() >= 1,
        "the acoustic compiler must retain every primary, branch trunk and outlet");
    {
        // Same graph with the trunks unauthored: the two ducts must disappear,
        // so the count above is carried by the lengths and not by the topology.
        auto pointBranches = config;
        for (auto& component : pointBranches.exhaustPaths.front().network->components)
            if (component.type == enginelab::ExhaustComponentType::merge
                || component.type == enginelab::ExhaustComponentType::splitter)
                component.lengthMm = 0.0;
        const auto pointGraph = enginelab::ExhaustGraph::makeForEngine(pointBranches);
        enginelab::AcousticExhaustNetwork pointAcoustics(pointGraph, cylinderIds);
        require(pointAcoustics.valid() && pointAcoustics.ductCount() == 6,
            "a branch with no authored length must stay a point");
    }
    require(acoustics.prepare(48'000.0),
        "the compiled exhaust must allocate its realtime lines");
    const std::array<enginelab::AcousticExhaustNetwork::Medium, 1> medium {{
        { 0.55F, 535.0F }
    }};
    acoustics.beginBlock(medium, 1.0);

    std::array<float, 4> sources {};
    std::array<enginelab::AcousticExhaustNetwork::CylinderBoundary, 4> boundaries {};
    auto radiatedEnergy = 0.0;
    for (std::size_t sample = 0; sample < 4'096; ++sample) {
        sources[0] = sample < 128
            ? 5'000.0F * static_cast<float>(std::sin(
                std::numbers::pi * static_cast<double>(sample + 1U) / 129.0))
            : 0.0F;
        const auto output = acoustics.process(sources, boundaries, 1.0F);
        require(std::isfinite(output[0].leftPa)
                && std::isfinite(output[0].rightPa),
            "a branched acoustic graph must remain finite");
        radiatedEnergy += static_cast<double>(output[0].leftPa)
            * output[0].leftPa;
    }
    require(radiatedEnergy > 1.0e-10,
        "a source must reach the independently retained outlets");

    acoustics.reset();
    sources.fill(0.0F);
    for (std::size_t sample = 0; sample < 512; ++sample) {
        const auto output = acoustics.process(sources, boundaries, 1.0F);
        require(output[0].leftPa == 0.0F && output[0].rightPa == 0.0F,
            "a reset source-free network must be exactly silent");
    }
}

void exhaustJetNoiseRegression() {
    using Jet = enginelab::ExhaustJetNoise;
    constexpr double sampleRate = 48'000.0;
    constexpr double massFlowKgPerSecond = 0.12;
    constexpr double areaM2 = 0.0024;
    constexpr double densityKgPerM3 = 0.60;
    constexpr double soundSpeedMps = 510.0;
    const auto velocity = massFlowKgPerSecond / (densityKgPerM3 * areaM2);
    const auto diameter = 2.0 * std::sqrt(areaM2 / std::numbers::pi);
    require(std::abs(Jet::limitedJetVelocityMps(
                massFlowKgPerSecond, areaM2, densityKgPerM3, soundSpeedMps)
            - velocity) < 1.0e-12,
        "subsonic outlet velocity must come from resolved mass flow and area");
    require(std::abs(Jet::centreFrequencyHz(
                massFlowKgPerSecond, areaM2, densityKgPerM3, soundSpeedMps)
            - Jet::peakStrouhalNumber * velocity / diameter) < 1.0e-9,
        "outlet-noise centre frequency must retain Strouhal scaling");
    const auto lowPower = Jet::acousticPowerWatts(
        massFlowKgPerSecond, areaM2, densityKgPerM3, soundSpeedMps);
    const auto doubledPower = Jet::acousticPowerWatts(
        2.0 * massFlowKgPerSecond, areaM2, densityKgPerM3, soundSpeedMps);
    require(lowPower > 0.0
            && std::abs(doubledPower / lowPower - 256.0) < 1.0e-8,
        "subsonic outlet acoustic power must follow the documented U^8 law");

    Jet first(0x12345678U);
    Jet second(0x12345678U);
    require(first.prepare(sampleRate) && second.prepare(sampleRate),
        "valid outlet-noise renderers must prepare");
    first.configure(massFlowKgPerSecond, areaM2, densityKgPerM3,
        soundSpeedMps);
    second.configure(massFlowKgPerSecond, areaM2, densityKgPerM3,
        soundSpeedMps);
    first.snapToTarget();
    second.snapToTarget();
    auto energy = 0.0;
    constexpr std::size_t measurementSamples = 48'000;
    for (std::size_t sample = 0; sample < measurementSamples; ++sample) {
        const auto a = first.process(1.0F);
        const auto b = second.process(1.0F);
        require(a == b && std::isfinite(a),
            "equal outlet-noise seeds must render sample-exactly");
        if (sample >= 4'000)
            energy += static_cast<double>(a) * a;
    }
    const auto measuredRms = std::sqrt(
        energy / static_cast<double>(measurementSamples - 4'000));
    require(measuredRms > first.targetPressureRmsPa() * 0.80
            && measuredRms < first.targetPressureRmsPa() * 1.20,
        "normalised jet spectrum must retain the acoustic-power RMS");

    first.reset();
    first.configure(0.0, areaM2, densityKgPerM3, soundSpeedMps);
    first.snapToTarget();
    for (std::size_t sample = 0; sample < 4'096; ++sample)
        require(first.process(1.0F) == 0.0F,
            "zero resolved outlet flow must be exactly silent");

    // Integration gate: turbulence radiates through the same physical outlet
    // and microphone geometry, yet the diagnostic switch proves that it never
    // feeds a fabricated pressure wave back into the passive pipe network.
    auto config = enginelab::makeDefaultInlineFour();
    std::array<std::uint32_t, 4> cylinderIds {};
    for (std::size_t index = 0; index < cylinderIds.size(); ++index)
        cylinderIds[index] = config.cylinders[index].id;
    const auto graph = enginelab::ExhaustGraph::makeForEngine(config);
    enginelab::AcousticExhaustNetwork enabled(graph, cylinderIds);
    enginelab::AcousticExhaustNetwork disabled(graph, cylinderIds);
    require(enabled.valid() && disabled.valid()
            && enabled.prepare(sampleRate) && disabled.prepare(sampleRate),
        "outlet-noise integration fixtures must compile and prepare");
    disabled.setOutletJetNoiseEnabled(false);
    const std::array<enginelab::AcousticExhaustNetwork::Medium, 1> medium {{
        { static_cast<float>(densityKgPerM3),
          static_cast<float>(soundSpeedMps) }
    }};
    const std::array<float, 1> pathFlow {{ 0.18F }};
    enabled.beginBlock(medium, 1.0, pathFlow);
    disabled.beginBlock(medium, 1.0, pathFlow);
    std::array<float, 4> sources {};
    std::array<enginelab::AcousticExhaustNetwork::CylinderBoundary, 4>
        boundaries {};
    auto enabledEnergy = 0.0;
    for (std::size_t sample = 0; sample < 12'000; ++sample) {
        const auto withJet = enabled.process(sources, boundaries, 1.0F);
        const auto withoutJet = disabled.process(sources, boundaries, 1.0F);
        require(withoutJet[0].leftPa == 0.0F
                && withoutJet[0].rightPa == 0.0F,
            "disabling outlet turbulence must leave a source-free network silent");
        enabledEnergy += static_cast<double>(withJet[0].leftPa)
            * withJet[0].leftPa;
    }
    require(enabledEnergy > 1.0e-10,
        "resolved mean flow must radiate deterministic outlet turbulence");
}

// A merge is a scattering point *and* a pipe. The audio network used to keep
// only the point, so a 4-into-1 collector contributed no delay and the primaries
// scattered straight into whatever followed the merge. Measure the thing that
// was missing: the trunk's propagation time must appear at the outlet.
void branchTrunkDelayRegression() {
    constexpr double sampleRate = 48'000.0;
    constexpr float soundSpeedMps = 535.0F;
    constexpr double trunkLengthMm = 400.0;

    const auto firstArrivalSample = [&](double mergeLengthMm) {
        auto config = enginelab::makeDefaultInlineFour();
        auto& path = config.exhaustPaths.front();
        enginelab::ExhaustNetworkConfig network;
        for (std::size_t index = 0; index < config.cylinders.size(); ++index) {
            enginelab::ExhaustComponentConfig primary;
            primary.id = static_cast<std::uint32_t>(100 + index);
            primary.type = enginelab::ExhaustComponentType::pipe;
            primary.lengthMm = 420.0;
            primary.diameterMm = 42.0;
            network.components.push_back(primary);
            network.cylinderConnections.push_back({
                config.cylinders[index].id, primary.id });
            network.connections.push_back({ primary.id, 200 });
        }
        enginelab::ExhaustComponentConfig merge;
        merge.id = 200;
        merge.type = enginelab::ExhaustComponentType::merge;
        merge.diameterMm = 60.0;
        merge.lengthMm = mergeLengthMm;
        network.components.push_back(merge);
        enginelab::ExhaustComponentConfig outlet;
        outlet.id = 300;
        outlet.type = enginelab::ExhaustComponentType::outlet;
        outlet.lengthMm = 200.0;
        outlet.diameterMm = 60.0;
        network.components.push_back(outlet);
        network.connections.push_back({ 200, 300 });
        path.network = std::move(network);
        enginelab::normaliseEngineConfig(config);

        const auto graph = enginelab::ExhaustGraph::makeForEngine(config);
        std::array<std::uint32_t, 4> cylinderIds {};
        for (std::size_t index = 0; index < cylinderIds.size(); ++index)
            cylinderIds[index] = config.cylinders[index].id;
        enginelab::AcousticExhaustNetwork acoustics(graph, cylinderIds);
        require(acoustics.valid() && acoustics.prepare(sampleRate),
            "the trunk fixture must compile and allocate");
        const std::array<enginelab::AcousticExhaustNetwork::Medium, 1> medium {{
            { 0.55F, soundSpeedMps } }};
        acoustics.beginBlock(medium, 1.0);

        std::array<float, 4> sources {};
        std::array<enginelab::AcousticExhaustNetwork::CylinderBoundary, 4> boundaries {};
        std::vector<float> response(4'096, 0.0F);
        for (std::size_t sample = 0; sample < response.size(); ++sample) {
            sources[0] = sample < 16
                ? 5'000.0F * static_cast<float>(std::sin(
                    std::numbers::pi * static_cast<double>(sample + 1U) / 17.0))
                : 0.0F;
            const auto output = acoustics.process(sources, boundaries, 1.0F);
            require(std::isfinite(output[0].leftPa),
                "the trunk fixture must stay finite");
            response[sample] = std::abs(output[0].leftPa);
        }
        require(*std::max_element(response.begin(), response.end()) > 0.0F,
            "the source must reach the outlet");
        // Causal onset, not the loudest sample. The loudest sample sits inside
        // the resonant build-up, which a longer trunk also lengthens, so it
        // would conflate the one-way delay with the round trip. Every element
        // here starts from zeroed state and is causal, so the output is exactly
        // zero until the direct path arrives, and the first non-zero sample is
        // that arrival with no threshold to choose.
        const auto onset = std::find_if(response.begin(), response.end(),
            [](float magnitude) { return magnitude > 0.0F; });
        require(onset != response.end(), "the direct arrival must be detectable");
        return static_cast<std::size_t>(std::distance(response.begin(), onset));
    };

    const auto withoutTrunk = firstArrivalSample(0.0);
    const auto withTrunk = firstArrivalSample(trunkLengthMm);
    const auto expected = trunkLengthMm * 0.001 / static_cast<double>(soundSpeedMps)
        * sampleRate;
    const auto measured = static_cast<double>(withTrunk)
        - static_cast<double>(withoutTrunk);
    require(std::abs(measured - expected) < 3.0,
        "an authored branch length must add its own propagation time to the path");
}

// The renderer used to take one gas state per path, sampled at the exhaust port
// -- the hottest point in the system -- and apply it to primary, chamber and
// tailpipe alike. Every duct now carries its own. Verify that a duct's delay
// follows the gas in that duct and not the gas at the valve.
void ductMediumRegression() {
    using Medium = enginelab::AcousticExhaustNetwork::Medium;
    constexpr double sampleRate = 48'000.0;
    constexpr float hotSoundSpeedMps = 620.0F;   // at the valve
    constexpr float coolSoundSpeedMps = 430.0F;  // at the tailpipe

    auto config = enginelab::makeDefaultInlineFour();
    std::array<std::uint32_t, 4> cylinderIds {};
    for (std::size_t index = 0; index < cylinderIds.size(); ++index)
        cylinderIds[index] = config.cylinders[index].id;
    const auto graph = enginelab::ExhaustGraph::makeForEngine(config);

    const auto onsetSample = [&](float pathSoundSpeedMps, float ductSoundSpeedMps,
                                 bool supplyDuctMedia) {
        enginelab::AcousticExhaustNetwork acoustics(graph, cylinderIds);
        require(acoustics.valid() && acoustics.prepare(sampleRate),
            "the medium fixture must compile and allocate");
        const std::array<Medium, 1> pathMedia {{ { 0.45F, pathSoundSpeedMps } }};
        std::vector<Medium> ductMedia(acoustics.ductCount(),
                                      Medium { 0.45F, ductSoundSpeedMps });
        acoustics.beginBlock(pathMedia, 1.0, {},
            supplyDuctMedia ? std::span<const Medium>(ductMedia)
                            : std::span<const Medium> {});

        std::array<float, 4> sources {};
        std::array<enginelab::AcousticExhaustNetwork::CylinderBoundary, 4> boundaries {};
        for (std::size_t sample = 0; sample < 8'192; ++sample) {
            sources[0] = sample < 16
                ? 5'000.0F * static_cast<float>(std::sin(
                    std::numbers::pi * static_cast<double>(sample + 1U) / 17.0))
                : 0.0F;
            const auto output = acoustics.process(sources, boundaries, 1.0F);
            require(std::isfinite(output[0].leftPa),
                "the medium fixture must stay finite");
            if (std::abs(output[0].leftPa) > 0.0F) return sample;
        }
        require(false, "the source must reach the outlet");
        return std::size_t { 0 };
    };

    // Supplying every duct the cool state must reproduce, exactly, the network
    // that was told the whole path is cool. The duct state governs the delay.
    const auto coolPath = onsetSample(coolSoundSpeedMps, coolSoundSpeedMps, false);
    const auto coolDucts = onsetSample(hotSoundSpeedMps, coolSoundSpeedMps, true);
    require(coolPath == coolDucts,
        "a duct's propagation must follow its own gas state, not its path's");

    // And it must be a real dependence, not a no-op: hot gas is faster, so the
    // same geometry has to arrive earlier.
    const auto hotPath = onsetSample(hotSoundSpeedMps, hotSoundSpeedMps, false);
    require(hotPath < coolPath,
        "hotter gas must carry the wave through the same geometry sooner");

    // Ducts left unresolved by the solver fall back to the path rather than to
    // silence or to a zero medium.
    {
        enginelab::AcousticExhaustNetwork acoustics(graph, cylinderIds);
        require(acoustics.valid() && acoustics.prepare(sampleRate),
            "the fallback fixture must compile and allocate");
        const std::array<Medium, 1> pathMedia {{ { 0.45F, hotSoundSpeedMps } }};
        const std::array<Medium, 3> partial {{
            { 0.0F, 0.0F }, { 0.45F, coolSoundSpeedMps }, { -1.0F, 900.0F } }};
        acoustics.beginBlock(pathMedia, 1.0, {}, partial);
        std::array<float, 4> sources {};
        std::array<enginelab::AcousticExhaustNetwork::CylinderBoundary, 4> boundaries {};
        for (std::size_t sample = 0; sample < 512; ++sample) {
            sources[0] = sample == 0 ? 5'000.0F : 0.0F;
            const auto output = acoustics.process(sources, boundaries, 1.0F);
            require(std::isfinite(output[0].leftPa) && std::isfinite(output[0].rightPa),
                "an unusable duct medium must fall back, not poison the network");
        }
    }
}

// Nothing asserted what a junction does with an area step, which is the whole
// mechanism by which a collector tunes and an expansion chamber silences. Pin
// it against the closed form rather than against a render.
//
// A lossless admittance junction between two ducts transmits 2*Y1/(Y1+Y2). With
// one medium throughout, Y is proportional to area, so a pipe of area A feeding
// a chamber of area m*A and then an equal pipe transmits, on the direct path
// before any reflection returns,
//
//     T(m) = 2/(1+m) * 2m/(1+m) = 4m/(1+m)^2,
//
// which is the low-frequency limit of Munjal's expansion-chamber transmission
// loss. The excitation is well below every element's plane-mode cutoff and the
// ducts are long enough that the direct arrival is complete before the first
// reflection returns, so nothing else is in the measurement.
void areaStepScatteringRegression() {
    constexpr double sampleRate = 48'000.0;
    constexpr float soundSpeedMps = 550.0F;
    constexpr double pipeDiameterMm = 40.0;
    constexpr double excitationHz = 300.0;

    const auto directArrivalPeak = [&](double expansionRatio) {
        auto config = enginelab::makeDefaultInlineFour();
        config.cylinders.resize(1);
        auto& path = config.exhaustPaths.front();
        path.cylinderIds = { config.cylinders.front().id };
        enginelab::ExhaustNetworkConfig network;
        enginelab::ExhaustComponentConfig primary;
        primary.id = 100;
        primary.type = enginelab::ExhaustComponentType::pipe;
        primary.lengthMm = 2'000.0;
        primary.diameterMm = pipeDiameterMm;
        network.components.push_back(primary);
        network.cylinderConnections.push_back({ config.cylinders.front().id, 100 });

        enginelab::ExhaustComponentConfig chamber;
        chamber.id = 200;
        chamber.type = enginelab::ExhaustComponentType::pipe;
        chamber.lengthMm = 500.0;
        chamber.diameterMm = pipeDiameterMm * std::sqrt(expansionRatio);
        network.components.push_back(chamber);
        network.connections.push_back({ 100, 200 });

        enginelab::ExhaustComponentConfig outlet;
        outlet.id = 300;
        outlet.type = enginelab::ExhaustComponentType::outlet;
        outlet.lengthMm = 2'000.0;
        outlet.diameterMm = pipeDiameterMm;
        network.components.push_back(outlet);
        network.connections.push_back({ 200, 300 });
        path.network = std::move(network);
        enginelab::normaliseEngineConfig(config);

        const auto graph = enginelab::ExhaustGraph::makeForEngine(config);
        const std::array<std::uint32_t, 1> cylinderIds {
            config.cylinders.front().id };
        enginelab::AcousticExhaustNetwork acoustics(graph, cylinderIds);
        require(acoustics.valid() && acoustics.prepare(sampleRate),
            "the area-step fixture must compile and allocate");
        const std::array<enginelab::AcousticExhaustNetwork::Medium, 1> medium {{
            { 0.50F, soundSpeedMps } }};
        acoustics.beginBlock(medium, 1.0);

        // One cycle at 300 Hz: low enough that the widest chamber here stays
        // two octaves inside its plane-mode band, short enough to finish before
        // the reflection off the chamber inlet returns to the outlet.
        const auto burstSamples = static_cast<std::size_t>(sampleRate / excitationHz);
        std::array<float, 1> sources {};
        std::array<enginelab::AcousticExhaustNetwork::CylinderBoundary, 1> boundaries {};
        std::vector<float> response(3'000, 0.0F);
        for (std::size_t sample = 0; sample < response.size(); ++sample) {
            sources[0] = sample < burstSamples
                ? 5'000.0F * static_cast<float>(std::sin(
                    2.0 * std::numbers::pi * static_cast<double>(sample)
                        / static_cast<double>(burstSamples)))
                : 0.0F;
            const auto output = acoustics.process(sources, boundaries, 1.0F);
            require(std::isfinite(output[0].leftPa),
                "the area-step fixture must stay finite");
            response[sample] = output[0].leftPa;
        }
        const auto onset = std::find_if(response.begin(), response.end(),
            [](float value) { return value != 0.0F; });
        require(onset != response.end(), "the burst must reach the outlet");
        // The direct arrival occupies one burst length from the onset. The
        // shortest return path adds two traversals of the outlet pipe, which is
        // far longer than that, so this window holds the direct wave alone.
        const auto begin = static_cast<std::size_t>(
            std::distance(response.begin(), onset));
        auto peak = 0.0F;
        for (std::size_t sample = begin;
             sample < std::min(begin + burstSamples, response.size()); ++sample)
            peak = std::max(peak, std::abs(response[sample]));
        require(peak > 0.0F, "the direct arrival must carry energy");
        return static_cast<double>(peak);
    };

    const auto reference = directArrivalPeak(1.0);
    for (const auto expansionRatio : { 2.0, 4.0, 9.0 }) {
        const auto expected = 4.0 * expansionRatio
            / ((1.0 + expansionRatio) * (1.0 + expansionRatio));
        const auto measured = directArrivalPeak(expansionRatio) / reference;
        require(std::abs(measured - expected) < 0.05,
            "an area step must scatter with the lossless admittance ratio");
    }
}

void structuralModalRadiatorRegression() {
    const auto config = enginelab::makeDefaultInlineFour();
    enginelab::StructuralModalRadiator radiator(config);
    require(radiator.valid(),
        "a normal engine geometry must produce a structural mode set");
    require(radiator.modeCount() >= 8 && radiator.modeCount() <= 24,
        "the reduced structural model must retain 8 to 24 modes");
    require(radiator.provenance()
            == enginelab::StructuralModalRadiator::Provenance::estimatedFamily,
        "schema-v1 engines must identify their block modes as family estimates");
    for (std::size_t index = 0; index < radiator.modeCount(); ++index) {
        const auto mode = radiator.mode(index);
        require(std::isfinite(mode.frequencyHz) && mode.frequencyHz > 0.0
                && mode.dampingRatio > 0.0 && mode.dampingRatio < 1.0
                && mode.modalMassKg > 0.0 && mode.radiatingAreaM2 > 0.0
                && mode.radiationEfficiency >= 0.0
                && mode.radiationEfficiency <= 1.0,
            "every estimated mode must expose finite physical parameters");
    }
    require(radiator.prepare(48'000.0),
        "a valid modal set must prepare at audio rate");

    enginelab::StructuralExcitationSample excitation;
    excitation.cylinderCount = config.cylinders.size();
    for (std::size_t sample = 0; sample < 512; ++sample)
        require(radiator.process(excitation) == 0.0F,
            "a reset unforced structure must be exactly silent");

    const auto resonanceHz = radiator.mode(0).frequencyHz;
    const auto measure = [&](double frequencyHz) {
        radiator.reset();
        auto energy = 0.0;
        for (std::size_t sample = 0; sample < 48'000; ++sample) {
            const auto force = 1'000.0F * static_cast<float>(std::sin(
                2.0 * std::numbers::pi * frequencyHz
                    * static_cast<double>(sample) / 48'000.0));
            excitation.bearingReactionForceN[0] = force;
            const auto pressure = radiator.process(excitation);
            require(std::isfinite(pressure),
                "modal integration must remain finite under resonant forcing");
            if (sample >= 24'000)
                energy += static_cast<double>(pressure) * pressure;
        }
        return std::sqrt(energy / 24'000.0);
    };
    const auto resonantRms = measure(resonanceHz);
    const auto offResonantRms = measure(resonanceHz * 0.63);
    require(resonantRms > offResonantRms * 1.5,
        "solver-resolved bearing force must excite the calculated block mode");

    excitation = {};
    excitation.cylinderCount = config.cylinders.size();
    auto earlyDecayEnergy = 0.0;
    auto lateDecayEnergy = 0.0;
    for (std::size_t sample = 0; sample < 48'000; ++sample) {
        const auto pressure = radiator.process(excitation);
        if (sample < 2'000)
            earlyDecayEnergy += static_cast<double>(pressure) * pressure;
        if (sample >= 24'000)
            lateDecayEnergy += static_cast<double>(pressure) * pressure;
    }
    require(earlyDecayEnergy > 0.0 && lateDecayEnergy < earlyDecayEnergy * 1.0e-4,
        "positive modal damping must dissipate stored structural energy");

    const auto renderCylinderImpulse = [](
        enginelab::StructuralModalRadiator& target,
        std::size_t cylinderIndex, std::size_t cylinderCount,
        bool bankTopology) {
        target.setBankTopologyParticipationEnabled(bankTopology);
        target.reset();
        std::array<float, 2'048> response {};
        enginelab::StructuralExcitationSample impulse;
        impulse.cylinderCount = cylinderCount;
        impulse.gasForceN[cylinderIndex] = 8'000.0F;
        impulse.bearingReactionForceN[cylinderIndex] = 6'000.0F;
        impulse.sideThrustForceN[cylinderIndex] = 1'500.0F;
        impulse.crankReactionTorqueNm[cylinderIndex] = 120.0F;
        response[0] = target.process(impulse);
        impulse = {};
        impulse.cylinderCount = cylinderCount;
        for (std::size_t sample = 1; sample < response.size(); ++sample)
            response[sample] = target.process(impulse);
        return response;
    };

    // makeDefaultV8 stores cylinders 1,2,3,4... but its explicit banks are
    // 1,3,5,7 and 2,4,6,8. Cylinders 1 and 2 therefore occupy the same
    // longitudinal station. Their response must be identical in a symmetric
    // family estimate; the removed flat-index modulo placed them at different
    // antinodes.
    const auto v8Config = enginelab::makeDefaultV8();
    enginelab::StructuralModalRadiator v8Radiator(v8Config);
    require(v8Radiator.prepare(48'000.0),
        "V8 structural topology fixture must prepare");
    const auto topologyLeft = renderCylinderImpulse(v8Radiator, 0, 8, true);
    const auto topologyRight = renderCylinderImpulse(v8Radiator, 1, 8, true);
    const auto legacyLeft = renderCylinderImpulse(v8Radiator, 0, 8, false);
    const auto legacyRight = renderCylinderImpulse(v8Radiator, 1, 8, false);
    auto topologyDifference = 0.0;
    auto legacyDifference = 0.0;
    for (std::size_t sample = 0; sample < topologyLeft.size(); ++sample) {
        topologyDifference += std::abs(static_cast<double>(
            topologyLeft[sample] - topologyRight[sample]));
        legacyDifference += std::abs(static_cast<double>(
            legacyLeft[sample] - legacyRight[sample]));
    }
    require(topologyDifference == 0.0,
        "paired V-bank cylinders at one station must share one modal coordinate");
    require(legacyDifference > 1.0e-6,
        "the same-binary null control must retain the disproven flat index map");

    enginelab::StructuralModalRadiator inlineRadiator(config);
    require(inlineRadiator.prepare(48'000.0),
        "inline structural null fixture must prepare");
    const auto inlineTopology = renderCylinderImpulse(
        inlineRadiator, 2, 4, true);
    const auto inlineLegacy = renderCylinderImpulse(
        inlineRadiator, 2, 4, false);
    require(inlineTopology == inlineLegacy,
        "one-bank engines must be sample-identical across the bank-map fix");
}

void acousticIntakeNetworkRegression() {
    auto config = enginelab::makeDefaultInlineFour();
    config.intake.airboxVolumeLitres = 4.0;
    config.intake.inletDuctLengthMm = 280.0;
    config.intake.inletDuctDiameterMm = 72.0;
    config.intake.bellmouthDiameterMm = 96.0;
    config.intake.runnerPlenumDiameterMm = 52.0;
    for (auto& path : config.intakePaths) path.geometry = config.intake;
    enginelab::normaliseEngineConfig(config);

    enginelab::AcousticIntakeNetwork intake(config);
    require(intake.valid() && intake.runnerCount() == config.cylinders.size()
            && intake.pathCount() >= 1,
        "the intake compiler must retain every runner and intake path");
    require(intake.prepare(48'000.0),
        "a valid intake topology must allocate outside the callback");
    std::array<enginelab::AcousticIntakeNetwork::PathBoundary, 1> paths {{
        { 0.0025F, 1.15F, 350.0F }
    }};
    intake.beginBlock(paths, 1.0);
    std::array<enginelab::AcousticIntakeNetwork::CylinderBoundary, 4> cylinders {};
    for (auto& cylinder : cylinders) {
        cylinder.conductanceAreaM2 = 0.00045F;
        cylinder.densityKgPerM3 = 1.15F;
        cylinder.soundSpeedMps = 350.0F;
        cylinder.physical = true;
    }

    auto energy = 0.0;
    for (std::size_t sample = 0; sample < 12'000; ++sample) {
        cylinders[0].massFlowKgPerSecond = sample < 192
            ? 0.075F * static_cast<float>(std::sin(
                std::numbers::pi * static_cast<double>(sample + 1U) / 193.0))
            : 0.0F;
        const auto output = intake.process(cylinders, 1.0F);
        require(std::isfinite(output[0].leftPa)
                && std::isfinite(output[0].rightPa),
            "the complete intake network must remain finite");
        energy += static_cast<double>(output[0].leftPa) * output[0].leftPa;
    }
    require(energy > 1.0e-10,
        "an intake-valve flow pulse must reach the inlet radiation load");

    intake.reset();
    for (auto& cylinder : cylinders) cylinder.massFlowKgPerSecond = 0.0F;
    for (std::size_t sample = 0; sample < 1'024; ++sample)
        require(intake.process(cylinders, 1.0F)[0].leftPa == 0.0F,
            "a reset intake with no flow perturbation must be exactly silent");

    intake.reset();
    auto earlyEnergy = 0.0;
    auto lateEnergy = 0.0;
    for (std::size_t sample = 0; sample < 96'000; ++sample) {
        cylinders[0].massFlowKgPerSecond = 0.030F;
        const auto pressure = intake.process(cylinders, 1.0F)[0].leftPa;
        if (sample < 12'000)
            earlyEnergy += static_cast<double>(pressure) * pressure;
        if (sample >= 84'000)
            lateEnergy += static_cast<double>(pressure) * pressure;
    }
    require(earlyEnergy == 0.0 && lateEnergy == 0.0,
        "a flow that is stationary from reset must create no acoustic source");
}

void forcedInductionAcousticsRegression() {
    using Acoustics = enginelab::ForcedInductionAcoustics;
    constexpr double sampleRate = 48'000.0;
    enginelab::ForcedInductionConfig toneConfig;
    toneConfig.enabled = true;
    toneConfig.type = enginelab::ForcedInductionType::turbocharger;
    toneConfig.compressorBladeCount = 6;
    toneConfig.turbineBladeCount = 0;
    toneConfig.compressorInducerDiameterMm = 0.0;
    toneConfig.turbineExducerDiameterMm = 0.0;
    toneConfig.wastegateFlowAreaMm2 = 0.0;

    Acoustics tone(toneConfig);
    require(tone.valid() && tone.semiEmpirical() && tone.prepare(sampleRate),
        "configured rotor geometry must compile a semi-empirical FI source");
    Acoustics::Input input;
    for (std::size_t sample = 0; sample < 512; ++sample)
        require(tone.process(input) == 0.0F,
            "zero shaft power and flow must be exactly silent");

    // 6 blades at 6,000 rpm is an exact 600 Hz blade-passing order. Correlate
    // against the physical order rather than accepting any convenient whistle.
    input.shaftSpeedRpm = 6'000.0F;
    input.pressureRatio = 1.8F;
    input.compressorPowerWatts = 10'000.0F;
    auto atBladeOrder = std::complex<double> {};
    auto offBladeOrder = std::complex<double> {};
    auto energy = 0.0;
    constexpr std::size_t sampleCount = 48'000;
    for (std::size_t sample = 0; sample < sampleCount; ++sample) {
        const auto pressure = tone.process(input);
        require(std::isfinite(pressure),
            "forced-induction pressure must remain finite");
        const auto time = static_cast<double>(sample) / sampleRate;
        atBladeOrder += static_cast<double>(pressure) * std::exp(
            std::complex<double>(0.0, -2.0 * std::numbers::pi * 600.0 * time));
        offBladeOrder += static_cast<double>(pressure) * std::exp(
            std::complex<double>(0.0, -2.0 * std::numbers::pi * 500.0 * time));
        energy += static_cast<double>(pressure) * pressure;
    }
    require(std::abs(atBladeOrder) > std::abs(offBladeOrder) * 100.0,
        "FI tone frequency must be shaft speed times authored blade count");
    const auto lowPowerRms = std::sqrt(energy / sampleCount);

    Acoustics highPower(toneConfig);
    require(highPower.prepare(sampleRate),
        "a second FI source must prepare independently");
    input.compressorPowerWatts = 40'000.0F;
    energy = 0.0;
    for (std::size_t sample = 0; sample < sampleCount; ++sample) {
        const auto pressure = highPower.process(input);
        energy += static_cast<double>(pressure) * pressure;
    }
    const auto highPowerRms = std::sqrt(energy / sampleCount);
    require(std::abs(highPowerRms / lowPowerRms - 2.0) < 0.01,
        "radiated pressure must scale with the square root of shaft power");

    // Broadband compressor and wastegate radiation remain flow-driven even at
    // zero shaft speed. This prevents a telemetry dropout from muting real jet
    // flow, while a closed wastegate remains exactly absent.
    enginelab::ForcedInductionConfig jetConfig;
    jetConfig.enabled = true;
    jetConfig.type = enginelab::ForcedInductionType::turbocharger;
    jetConfig.compressorInducerDiameterMm = 50.0;
    jetConfig.wastegateFlowAreaMm2 = 400.0;
    Acoustics jets(jetConfig);
    require(jets.valid() && jets.prepare(sampleRate),
        "authored flow geometry must compile broadband FI radiation");
    Acoustics::Input jetInput;
    jetInput.correctedAirFlowKgPerSecond = 0.20F;
    jetInput.exhaustMassFlowKgPerSecond = 0.18F;
    jetInput.wastegateOpening = 0.0F;
    auto compressorJetEnergy = 0.0;
    for (std::size_t sample = 0; sample < 8'192; ++sample) {
        const auto pressure = jets.process(jetInput);
        compressorJetEnergy += static_cast<double>(pressure) * pressure;
    }
    require(compressorJetEnergy > 0.0,
        "corrected compressor flow must radiate broadband noise at zero shaft speed");

    // pressurePeakFromPower() is appropriate for a sinusoid, whose RMS is
    // peak/sqrt(2). Broadband instead needs both a unit-RMS bandpass and the RMS
    // pressure implied by W=4*pi*r^2*p_rms^2/(rho*c). Measure the complete
    // deterministic filter rather than accepting a positive-but-under-levelled
    // source.
    Acoustics powerNormalisedJet(jetConfig);
    Acoustics legacyUnnormalisedJet(jetConfig);
    legacyUnnormalisedJet.setBroadbandPowerNormalisationEnabled(false);
    require(powerNormalisedJet.prepare(sampleRate)
            && legacyUnnormalisedJet.prepare(sampleRate),
        "broadband power A/B fixtures must prepare");
    auto normalisedEnergy = 0.0;
    auto legacyEnergy = 0.0;
    constexpr std::size_t broadbandSamples = 96'000;
    constexpr std::size_t broadbandWarmup = 48'000;
    for (std::size_t sample = 0; sample < broadbandSamples; ++sample) {
        const auto normalised = powerNormalisedJet.process(jetInput);
        const auto legacy = legacyUnnormalisedJet.process(jetInput);
        require(std::isfinite(normalised) && std::isfinite(legacy),
            "broadband power fixtures must remain finite");
        if (sample >= broadbandWarmup) {
            normalisedEnergy += static_cast<double>(normalised) * normalised;
            legacyEnergy += static_cast<double>(legacy) * legacy;
        }
    }
    const auto measuredBroadbandRms = std::sqrt(
        normalisedEnergy
        / static_cast<double>(broadbandSamples - broadbandWarmup));
    const auto legacyBroadbandRms = std::sqrt(
        legacyEnergy
        / static_cast<double>(broadbandSamples - broadbandWarmup));
    const auto compressorRadiusM =
        jetConfig.compressorInducerDiameterMm * 0.0005;
    const auto compressorAreaM2 =
        std::numbers::pi * compressorRadiusM * compressorRadiusM;
    const auto jetVelocityMps = static_cast<double>(
        jetInput.correctedAirFlowKgPerSecond)
        / (static_cast<double>(jetInput.densityKgPerM3)
            * compressorAreaM2);
    const auto acousticPowerWatts =
        jetConfig.turbulentJetNoiseCoefficient
        * static_cast<double>(jetInput.densityKgPerM3)
        * compressorAreaM2 * std::pow(jetVelocityMps, 8.0)
        / std::pow(static_cast<double>(jetInput.soundSpeedMps), 5.0);
    const auto expectedBroadbandRms = std::sqrt(
        acousticPowerWatts * static_cast<double>(jetInput.densityKgPerM3)
        * static_cast<double>(jetInput.soundSpeedMps)
        / (4.0 * std::numbers::pi));
    require(std::abs(measuredBroadbandRms / expectedBroadbandRms - 1.0) < 0.12,
        "broadband FI pressure must conserve the configured acoustic power");
    require(measuredBroadbandRms > legacyBroadbandRms * 2.0,
        "the null control must expose the former bandpass RMS loss");

    jetConfig.compressorInducerDiameterMm = 0.0;
    Acoustics wastegate(jetConfig);
    require(wastegate.valid() && wastegate.prepare(sampleRate),
        "wastegate geometry alone must compile a flow source");
    jetInput.correctedAirFlowKgPerSecond = 0.0F;
    jetInput.wastegateOpening = 0.0F;
    for (std::size_t sample = 0; sample < 512; ++sample)
        require(wastegate.process(jetInput) == 0.0F,
            "a physically closed wastegate must be exactly silent");
    jetInput.wastegateOpening = 1.0F;
    auto wastegateEnergy = 0.0;
    for (std::size_t sample = 0; sample < 8'192; ++sample) {
        const auto pressure = wastegate.process(jetInput);
        wastegateEnergy += static_cast<double>(pressure) * pressure;
    }
    require(wastegateEnergy > 0.0,
        "resolved wastegate mass flow must radiate through its authored area");

    jetConfig.wastegateFlowAreaMm2 = 0.0;
    jetConfig.blowOffValveFlowAreaMm2 = 350.0;
    Acoustics blowOff(jetConfig);
    require(blowOff.valid() && blowOff.prepare(sampleRate),
        "authored blow-off geometry must compile a flow source");
    jetInput.exhaustMassFlowKgPerSecond = 0.0F;
    jetInput.blowOffMassFlowKgPerSecond = 0.0F;
    for (std::size_t sample = 0; sample < 512; ++sample)
        require(blowOff.process(jetInput) == 0.0F,
            "a blow-off valve without resolved mass flow must be exactly silent");
    jetInput.blowOffMassFlowKgPerSecond = 0.08F;
    auto blowOffEnergy = 0.0;
    for (std::size_t sample = 0; sample < 8'192; ++sample) {
        const auto pressure = blowOff.process(jetInput);
        blowOffEnergy += static_cast<double>(pressure) * pressure;
    }
    require(blowOffEnergy > 0.0,
        "resolved blow-off mass flow must radiate through its authored area");
}

double absoluteDifference(const std::vector<float>& left, const std::vector<float>& right) {
    double difference = 0.0;
    for (std::size_t index = 0; index < std::min(left.size(), right.size()); ++index)
        difference += std::abs(static_cast<double>(left[index] - right[index]));
    return difference;
}

double spectralBandEnergy(std::span<const float> samples, double sampleRate,
                          double lowHz, double highHz, std::size_t begin) {
    begin = std::min(begin, samples.size());
    const auto count = std::min<std::size_t>(2'048, samples.size() - begin);
    if (count < 2) return 0.0;
    const auto firstBin = static_cast<std::size_t>(std::ceil(
        lowHz * static_cast<double>(count) / sampleRate));
    const auto lastBin = std::min<std::size_t>(count / 2,
        static_cast<std::size_t>(std::floor(
            highHz * static_cast<double>(count) / sampleRate)));
    auto energy = 0.0;
    for (auto bin = firstBin; bin <= lastBin; ++bin) {
        std::complex<double> spectrum {};
        for (std::size_t sample = 0; sample < count; ++sample) {
            const auto window = 0.5 - 0.5 * std::cos(
                2.0 * std::numbers::pi * static_cast<double>(sample)
                / static_cast<double>(count - 1));
            const auto phase = -2.0 * std::numbers::pi
                * static_cast<double>(bin * sample) / static_cast<double>(count);
            spectrum += static_cast<double>(samples[begin + sample]) * window
                * std::exp(std::complex<double>(0.0, phase));
        }
        energy += std::norm(spectrum);
    }
    return energy;
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
    const auto isolatedCombustion = analyseWaveform(at48k, onset48);
    require(isolatedCombustion.rms > 0.008 && isolatedCombustion.peak > 0.05,
            "tailpipe radiation filtering must not attenuate the isolated combustion bus");

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

void compressionIgnitionTimbreRegression() {
    constexpr double sampleRate = 48'000.0;
    auto spark = eventFixture();
    spark.compressionIgnition = false;
    spark.combustionSharpness = 0.0F;
    auto diesel = spark;
    diesel.compressionIgnition = true;
    diesel.combustionSharpness = 0.85F;

    const auto sparkWave = renderEvent(sampleRate, 256, 3'600, true, spark);
    const auto dieselWave = renderEvent(sampleRate, 256, 3'600, true, diesel);
    const auto sparkOnset = firstAudibleSample(sparkWave);
    const auto dieselOnset = firstAudibleSample(dieselWave);
    require(sparkOnset < sparkWave.size() && dieselOnset < dieselWave.size(),
        "spark and compression-ignition events must both remain audible");

    const auto sparkHigh = spectralBandEnergy(
        sparkWave, sampleRate, 1'800.0, 5'200.0, sparkOnset);
    const auto sparkLow = spectralBandEnergy(
        sparkWave, sampleRate, 120.0, 1'200.0, sparkOnset);
    const auto dieselHigh = spectralBandEnergy(
        dieselWave, sampleRate, 1'800.0, 5'200.0, dieselOnset);
    const auto dieselLow = spectralBandEnergy(
        dieselWave, sampleRate, 120.0, 1'200.0, dieselOnset);
    require(sparkLow > 0.0 && dieselLow > 0.0,
        "combustion timbre fixture must contain a resolved low-frequency body");
    require(dieselHigh / dieselLow > (sparkHigh / sparkLow) * 1.35,
        "resolved compression-ignition sharpness must increase upper-mode energy");
    const auto dieselMetrics = analyseWaveform(dieselWave);
    require(dieselMetrics.finite && dieselMetrics.peak < 0.999,
        "diesel pressure-rise timbre must remain finite and below the safety limiter");
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

void diagnosticStemRegression() {
    constexpr int startSample = 17;
    constexpr int sampleCount = 4'096;
    constexpr int totalSamples = startSample + sampleCount + 19;
    constexpr int blockSize = 256;

    enginelab::FiringEventQueue normalQueue;
    enginelab::FiringEventQueue tappedQueue;
    enginelab::RealtimeAudioState normalState;
    enginelab::RealtimeAudioState tappedState;
    const auto configure = [](enginelab::RealtimeAudioState& state) {
        state.rpm.store(3'500.0F);
        state.throttle.store(0.72F);
        state.load.store(0.55F);
        state.mechanicalStress.store(0.30F);
        state.manifoldPressureKpa.store(72.0F);
        state.exhaustPressureKpa.store(158.0F);
        state.exhaustFlowGramsPerSecond.store(62.0F);
        state.exhaustTemperatureC.store(640.0F);
        state.intakeRunnerResonanceHz.store(310.0F);
        state.intakeRunnerAmplitudeKpa.store(4.0F);
    };
    configure(normalState);
    configure(tappedState);
    const auto event = eventFixture();
    require(normalQueue.tryPush(event) && tappedQueue.tryPush(event),
        "stem fixture events must enter both realtime queues");

    enginelab::RealtimeEngineAudio normal(normalQueue, normalState);
    enginelab::RealtimeEngineAudio tapped(tappedQueue, tappedState);
    normal.prepare(48'000.0, blockSize);
    tapped.prepare(48'000.0, blockSize);

    juce::AudioBuffer<float> normalMaster(2, totalSamples);
    juce::AudioBuffer<float> tappedMaster(2, totalSamples);
    normalMaster.clear();
    tappedMaster.clear();
    std::array<juce::AudioBuffer<float>, 8> stems {
        juce::AudioBuffer<float>(2, totalSamples),
        juce::AudioBuffer<float>(2, totalSamples),
        juce::AudioBuffer<float>(2, totalSamples),
        juce::AudioBuffer<float>(2, totalSamples),
        juce::AudioBuffer<float>(2, totalSamples),
        juce::AudioBuffer<float>(2, totalSamples),
        juce::AudioBuffer<float>(2, totalSamples),
        juce::AudioBuffer<float>(2, totalSamples)
    };
    for (auto& stem : stems) {
        for (int channel = 0; channel < stem.getNumChannels(); ++channel)
            for (int sample = 0; sample < stem.getNumSamples(); ++sample)
                stem.setSample(channel, sample, 0.25F);
    }
    const enginelab::RealtimeAudioStemBuffers stemBuffers {
        &stems[0], &stems[1], &stems[2], &stems[3], &stems[4], &stems[5],
        &stems[6], &stems[7]
    };

    normal.render(normalMaster, startSample, sampleCount);
    tapped.renderWithStems(tappedMaster, startSample, sampleCount, stemBuffers);

    for (int channel = 0; channel < normalMaster.getNumChannels(); ++channel) {
        for (int sample = 0; sample < totalSamples; ++sample) {
            require(normalMaster.getSample(channel, sample)
                    == tappedMaster.getSample(channel, sample),
                "enabling diagnostic stems changed a master sample");
        }
    }
    for (const auto& stem : stems) {
        require(stem.getSample(0, 0) == 0.25F
                && stem.getSample(1, totalSamples - 1) == 0.25F,
            "stem capture wrote outside the requested sample range");
    }
    require(stems[0].getMagnitude(0, startSample, sampleCount) > 1.0e-6F,
        "combustion stem did not expose its active source");
    require(stems[1].getMagnitude(0, startSample, sampleCount) > 1.0e-6F,
        "dry exhaust stem did not expose its active source");
    require(stems[3].getMagnitude(0, startSample, sampleCount) > 1.0e-6F,
        "intake stem did not expose its active source");
    require(stems[5].getMagnitude(0, startSample, sampleCount) > 1.0e-6F,
        "mechanical stem did not expose its active source");
    require(stems[2].getMagnitude(0, startSample, sampleCount) == 0.0F,
        "IR stem must stay silent when no impulse response is loaded");
    require(stems[4].getMagnitude(0, startSample, sampleCount) == 0.0F,
        "forced-induction stem must stay silent when no device is configured");
    require(stems[6].getMagnitude(0, startSample, sampleCount) > 1.0e-6F,
        "pressure-wave diagnostic must expose the legacy exhaust source");
    require(stems[7].getMagnitude(0, startSample, sampleCount) == 0.0F,
        "outlet-jet diagnostic must stay silent on the legacy fallback");
}

void physicalThermoacousticPathRegression() {
    const PhysicalExhaustFixture referenceFixture;
    const auto reference = renderPhysicalExhaust(referenceFixture);
    const auto referenceMetrics = analyseWaveform(reference);
    require(referenceMetrics.finite && referenceMetrics.rms > 1.0e-4
            && referenceMetrics.peak > 1.0e-3,
        "an SI pressure/flow transient must radiate a finite audible waveform");

    auto forbiddenVoicing = referenceFixture;
    forbiddenVoicing.cylinderTransmissionGain = 0.0F;
    forbiddenVoicing.pathGain = 0.0F;
    forbiddenVoicing.highFrequencyNoise = 100.0F;
    forbiddenVoicing.exhaustPreset = 4;
    forbiddenVoicing.proceduralExhaustEvent = true;
    const auto uncoloured = renderPhysicalExhaust(forbiddenVoicing);
    require(absoluteDifference(reference, uncoloured) < 1.0e-7,
        "SI exhaust must be invariant to procedural presets, noise, events and legacy gains");

    const auto repeated = renderPhysicalExhaust(referenceFixture);
    require(absoluteDifference(reference, repeated) == 0.0,
        "physical thermoacoustic rendering must be deterministic");

    auto reverseFlowFixture = referenceFixture;
    reverseFlowFixture.transientMassFlowKgPerSecond *= -1.0F;
    const auto reverseFlow = renderPhysicalExhaust(reverseFlowFixture);
    require(absoluteDifference(reference, reverseFlow) > 0.01,
        "signed reverse flow must change the characteristic source instead of being clamped away");

    auto steadyFixture = forbiddenVoicing;
    steadyFixture.transient = false;
    const auto steady = renderPhysicalExhaust(steadyFixture);
    require(std::all_of(steady.begin(), steady.end(), [](float value) {
        return std::abs(value) < 1.0e-7F;
    }), "steady SI pressure/flow must not manufacture exhaust sound");

    auto longTailFixture = referenceFixture;
    longTailFixture.tailpipeDelaySeconds = 0.003F;
    const auto longTail = renderPhysicalExhaust(longTailFixture);
    const auto shortOnset = firstAudibleSample(reference);
    const auto longOnset = firstAudibleSample(longTail);
    require(shortOnset < reference.size() && longOnset < longTail.size(),
        "both physical tailpipe geometries must radiate their transient");
    const auto measuredExtraDelay = static_cast<double>(longOnset - shortOnset)
        / referenceFixture.sampleRateHz;
    require(std::abs(measuredExtraDelay - 0.002) <= 3.0 / referenceFixture.sampleRateHz,
        "collector-to-mouth onset must follow the authored one-way tailpipe delay");

    auto highRateFixture = referenceFixture;
    highRateFixture.sampleRateHz = 96'000.0;
    const auto highRate = renderPhysicalExhaust(highRateFixture);
    const auto onset48Seconds = static_cast<double>(shortOnset)
        / referenceFixture.sampleRateHz;
    const auto onset96Seconds = static_cast<double>(firstAudibleSample(highRate))
        / highRateFixture.sampleRateHz;
    require(std::abs(onset48Seconds - onset96Seconds) < 0.00015,
        "physical propagation time must remain invariant across sample rates");
}

void pipeRadiationRegression() {
    constexpr double sampleRateHz = 48'000.0;
    constexpr double radiusM = 0.032;
    constexpr double densityKgPerM3 = 0.62;
    constexpr double soundSpeedMps = 548.0;
    enginelab::UnflangedPipeRadiation radiation;
    require(!radiation.prepare(0.0, radiusM, 1.0)
            && radiation.prepare(sampleRateHz, radiusM, 1.0)
            && radiation.setMedium(densityKgPerM3, soundSpeedMps),
            "unflanged radiation model must validate geometry and medium");

    const auto expectedImpedance = densityKgPerM3 * soundSpeedMps
        / (std::numbers::pi * radiusM * radiusM);
    require(std::abs(radiation.characteristicImpedancePaSPerM3()
                     / expectedImpedance - 1.0) < 1.0e-12,
            "pipe characteristic impedance must be rho*c/area");
    require(std::abs(radiation.reflectionCoefficient(0.0).real() + 1.0) < 1.0e-12,
            "an unflanged mouth must approach a pressure-release reflection at DC");

    constexpr double n1 = 0.167;
    constexpr double d1 = 1.393;
    constexpr double d2 = 0.457;
    for (int index = 1; index <= 200; ++index) {
        const auto frequencyHz = sampleRateHz * 0.49
            * static_cast<double>(index) / 200.0;
        const auto digital = radiation.reflectionCoefficient(frequencyHz);
        require(std::abs(digital) <= 1.0 + 1.0e-12,
                "causal unflanged radiation filter must remain passive");
        const auto warpedAngularFrequency = 2.0 * sampleRateHz
            * std::tan(std::numbers::pi * frequencyHz / sampleRateHz);
        const std::complex<double> jKa {
            0.0, warpedAngularFrequency * radiusM / soundSpeedMps
        };
        const auto continuous = -(1.0 + n1 * jKa)
            / (1.0 + d1 * jKa + d2 * jKa * jKa);
        require(std::abs(digital - continuous) < 2.0e-12,
                "bilinear radiation filter must match the published Pade load");
    }

    enginelab::PipeRadiationSample settled;
    for (int index = 0; index < 4'096; ++index)
        settled = radiation.process(2'000.0);
    require(std::abs(settled.reflectedPressurePa + 2'000.0) < 1.0e-7
            && std::abs(settled.farFieldPressurePa) < 1.0e-7,
            "steady pressure must reflect at the open end without radiating DC energy");
    require(radiation.planeModeCutoffHz() > 9'000.0,
            "fixture must remain inside the plane-mode validity band over audible midrange");

    // High-amplitude open-end flow separates and sheds vortices. Verify the
    // optional quasi-steady resistance as an energy sink, not an output clamp:
    // the same incident sine still crosses the radiation load, but less of its
    // energy is returned to the duct. The default above remains bit-for-bit
    // linear when the coefficient is zero.
    enginelab::UnflangedPipeRadiation linearHighLevel;
    enginelab::UnflangedPipeRadiation lossyHighLevel;
    require(linearHighLevel.prepare(sampleRateHz, radiusM, 1.0)
            && lossyHighLevel.prepare(sampleRateHz, radiusM, 1.0)
            && linearHighLevel.setMedium(densityKgPerM3, soundSpeedMps)
            && lossyHighLevel.setMedium(densityKgPerM3, soundSpeedMps)
            && !lossyHighLevel.setNonlinearLossCoefficient(-1.0)
            && lossyHighLevel.setNonlinearLossCoefficient(
                4.0 / (3.0 * std::numbers::pi)),
            "nonlinear mouth resistance must accept only finite passive coefficients");
    double incidentEnergy = 0.0;
    double linearReflectedEnergy = 0.0;
    double lossyReflectedEnergy = 0.0;
    for (int index = 0; index < 48'000; ++index) {
        const auto incident = 40'000.0 * std::sin(
            2.0 * std::numbers::pi * 500.0 * index / sampleRateHz);
        const auto linearSample = linearHighLevel.process(incident);
        const auto lossySample = lossyHighLevel.process(incident);
        require(std::isfinite(lossySample.reflectedPressurePa)
                && std::isfinite(lossySample.farFieldPressurePa),
                "nonlinear mouth loss must remain finite at extreme acoustic level");
        if (index >= 4'800) {
            incidentEnergy += incident * incident;
            linearReflectedEnergy += linearSample.reflectedPressurePa
                * linearSample.reflectedPressurePa;
            lossyReflectedEnergy += lossySample.reflectedPressurePa
                * lossySample.reflectedPressurePa;
        }
    }
    require(lossyReflectedEnergy < linearReflectedEnergy * 0.90,
            "vortex shedding must dissipate returned wave energy at high level");
    require(lossyReflectedEnergy <= incidentEnergy * (1.0 + 1.0e-9),
            "positive nonlinear resistance must never reflect more energy than arrives");
}

void acousticMonitorCalibrationRegression() {
    constexpr auto fullScaleSplDb = 144.0;
    enginelab::RealtimeAudioState defaultState;
    require(std::abs(defaultState.acousticFullScaleSplDb.load()
                     - enginelab::AcousticMonitorCalibration::defaultFullScaleSplDb) < 1.0e-12,
        "runtime and SI monitor calibration defaults must not drift apart");
    const auto fullScalePeakPressurePa =
        enginelab::AcousticMonitorCalibration::sinePeakPressurePa(fullScaleSplDb);
    require(std::abs(enginelab::AcousticMonitorCalibration::normalisePeakPressure(
                         fullScalePeakPressurePa, fullScaleSplDb) - 1.0) < 1.0e-12,
        "configured full-scale acoustic pressure must map exactly to 0 dBFS");
    require(std::abs(enginelab::AcousticMonitorCalibration::rmsPressurePa(120.0)
                     - 20.0) < 1.0e-12,
        "SPL calibration must retain the 20 micropascal reference");
    require(enginelab::AcousticMonitorCalibration::normalisePeakPressure(
                1.0, std::numeric_limits<double>::quiet_NaN()) == 0.0,
        "invalid monitor calibration must fail silent rather than poison audio");
}

} // namespace

// ---------------------------------------------------------------------------
// Valve port termination
// ---------------------------------------------------------------------------

// Magnitude of the port reflection filter at one frequency, from its
// coefficients, so the physical invariants below are stated in the frequency
// domain where they are meaningful.
double portReflectionMagnitude(const enginelab::ValvePortTermination::Coefficients& c,
                               double frequencyHz, double sampleRateHz) {
    const auto z = std::polar(1.0, -2.0 * std::numbers::pi * frequencyHz / sampleRateHz);
    const auto numerator = c.b0 + c.b1 * z;
    const auto denominator = 1.0 + c.a1 * z;
    return std::abs(denominator) > 1.0e-15 ? std::abs(numerator / denominator) : 0.0;
}

void valvePortTerminationRegression() {
    using Termination = enginelab::ValvePortTermination;
    constexpr double sampleRate = 48'000.0;
    // Representative hot-exhaust port: 40 mm runner, 800 K gas.
    constexpr double density = 0.45;
    constexpr double soundSpeed = 550.0;
    constexpr double runnerArea = 1.257e-3;
    constexpr double characteristicImpedance = density * soundSpeed / runnerArea;

    // A shut valve is a rigid end: R = +1 at every frequency, so the runner
    // reflects in phase. The previous mean-flow-only model returned -1 here.
    const auto shut = Termination::compute(0.0, 0.0, 0.0, density,
                                           characteristicImpedance, sampleRate);
    for (const auto frequency : { 100.0, 1'000.0, 4'000.0, 12'000.0 })
        require(std::abs(portReflectionMagnitude(shut, frequency, sampleRate) - 1.0) < 1.0e-4,
                "a shut valve must reflect with unit magnitude");
    require(shut.b0 > 0.0, "a shut valve must reflect in phase (rigid, not pressure-release)");

    // A nearly shut valve must approach the rigid end rather than invert. This
    // is the limit the linearised mean-flow resistance had backwards.
    const auto nearlyShut = Termination::compute(2.0e-6, 0.0, 0.0, density,
                                                 characteristicImpedance, sampleRate);
    require(portReflectionMagnitude(nearlyShut, 1'000.0, sampleRate) > 0.95,
            "a nearly shut valve must still be strongly reflective");
    require(nearlyShut.b0 > 0.0, "a nearly shut valve must not behave as an open end");

    // The physical defect that made the exhaust ring: with an open valve and no
    // mean flow, the port must still absorb, because the acoustic velocity
    // through the orifice sheds vortices. Mean-flow-only resistance gave |R| = 1
    // and therefore an undamped runner at idle.
    //
    // Amplitudes are stated as in-runner acoustic pressure and converted through
    // the runner's own characteristic impedance, so they are physically legible:
    // a blowdown pulse in a primary reaches several kPa.
    constexpr double openArea = 5.0e-4;
    const auto volumeVelocityFor = [&](double pressurePa) {
        return pressurePa / characteristicImpedance;
    };
    const auto quiescent = Termination::compute(openArea, 0.0, 0.0, density,
                                                characteristicImpedance, sampleRate);
    const auto excited = Termination::compute(openArea, 0.0, volumeVelocityFor(5'000.0),
                                              density, characteristicImpedance, sampleRate);
    const auto quiescentMagnitude = portReflectionMagnitude(quiescent, 2'000.0, sampleRate);
    const auto excitedMagnitude = portReflectionMagnitude(excited, 2'000.0, sampleRate);
    require(excitedMagnitude < quiescentMagnitude,
            "acoustic velocity through an open port must dissipate");
    // A 5 kPa pulse against a 0.5e-3 m^2 opening loses of order 15% of its
    // amplitude per reflection. The bound is deliberately loose -- the point is
    // that the loss is a significant fraction, not that it hits a tuned number.
    require(excitedMagnitude < 0.90,
            "an open port excited at realistic runner amplitude must absorb strongly");

    // Louder excitation must damp harder: this is what bounds a growing mode.
    const auto louder = Termination::compute(openArea, 0.0, volumeVelocityFor(20'000.0),
                                             density, characteristicImpedance, sampleRate);
    require(portReflectionMagnitude(louder, 2'000.0, sampleRate) < excitedMagnitude,
            "orifice resistance must rise with acoustic amplitude");

    // Mean flow damps as well, and the two contributions are the same mechanism.
    const auto flowing = Termination::compute(openArea, 0.05, 0.0, density,
                                              characteristicImpedance, sampleRate);
    require(portReflectionMagnitude(flowing, 2'000.0, sampleRate) < quiescentMagnitude,
            "mean flow through an open port must dissipate");

    // Passivity: a termination may never return more energy than it receives, at
    // any frequency, for any state. A filter that did would make the waveguide
    // grow without bound.
    for (const auto area : { 1.0e-6, 1.0e-4, 5.0e-4, 3.0e-3 })
        for (const auto flow : { 0.0, 0.005, 0.05, 0.4 })
            for (const auto acoustic : { 0.0, 1.0e-3, 1.0e-2 }) {
                const auto c = Termination::compute(area, flow, acoustic, density,
                                                    characteristicImpedance, sampleRate);
                for (int step = 0; step <= 48; ++step) {
                    const auto frequency = static_cast<double>(step) / 48.0
                        * sampleRate * 0.5;
                    require(portReflectionMagnitude(c, frequency, sampleRate) <= 1.0 + 1.0e-6,
                            "port reflection must be passive at every frequency");
                }
            }

    // Non-finite inputs must fall back to the rigid end, never to a divergent
    // filter, and the filter itself must stay finite under a non-finite sample.
    Termination::State state;
    const auto guarded = Termination::compute(
        std::nan(""), 0.0, 0.0, density, characteristicImpedance, sampleRate);
    require(std::isfinite(Termination::process(guarded, state, 1.0F)),
            "a non-finite boundary must not produce a non-finite reflection");
    (void) Termination::process(shut, state, std::numeric_limits<float>::infinity());
    require(std::isfinite(state.previousOutput),
            "filter state must remain finite after a non-finite sample");
}

void ductWallLossRegression() {
    using Loss = enginelab::DuctWallLoss;
    constexpr double sampleRate = 48'000.0;
    // Hot exhaust primary: 20 mm radius, 800 K gas, 3 ms of duct (about 1.6 m).
    constexpr double density = 0.45;
    constexpr double soundSpeed = 550.0;
    constexpr double radius = 0.020;
    constexpr double traversal = 0.003;

    // Attenuation must rise with frequency. This is the property that damps a
    // metallic high mode harder than the firing fundamental.
    const auto atLow = Loss::traversalGain(100.0, traversal, radius, density, soundSpeed);
    const auto atMid = Loss::traversalGain(1'000.0, traversal, radius, density, soundSpeed);
    const auto atHigh = Loss::traversalGain(8'000.0, traversal, radius, density, soundSpeed);
    require(atLow > atMid && atMid > atHigh,
            "wall attenuation must increase with frequency");
    require(atHigh > 0.0 && atLow < 1.0, "attenuation must be positive and lossy");

    // The sqrt(f) law: an eightfold frequency increase must raise the exponent
    // by sqrt(8), so ln(gain) scales accordingly.
    const auto exponentAt1k = -std::log(atMid);
    const auto exponentAt8k = -std::log(atHigh);
    require(std::abs(exponentAt8k / exponentAt1k - std::sqrt(8.0)) < 0.02,
            "attenuation must follow the sqrt(f) boundary-layer law");

    // Narrower ducts and longer ducts attenuate more.
    require(Loss::traversalGain(1'000.0, traversal, radius * 0.5, density, soundSpeed) < atMid,
            "a narrower duct must attenuate more");
    require(Loss::traversalGain(1'000.0, traversal * 2.0, radius, density, soundSpeed) < atMid,
            "a longer duct must attenuate more");

    // Passive at every frequency, and exact at DC (a static pressure difference
    // is not attenuated by a boundary layer).
    require(Loss::traversalGain(0.0, traversal, radius, density, soundSpeed) == 1.0,
            "there is no boundary-layer loss at zero frequency");

    const auto coefficients = Loss::fit(traversal, radius, density, soundSpeed, sampleRate);
    require(coefficients.pole >= 0.0F && coefficients.pole < 1.0F,
            "fitted pole must be inside the unit circle");
    require(coefficients.zero >= 0.0F && coefficients.zero <= coefficients.pole,
            "the zero must sit inside the pole, which is what makes the shelf passive");

    // The difference equation must actually realise the magnitude the fit
    // claims. Everything below reasons about Loss::magnitude, so measure the
    // impulse response once and hold the analytic form to it.
    {
        constexpr std::size_t impulseLength = 16'384;
        std::vector<double> impulse(impulseLength, 0.0);
        Loss::State state;
        impulse[0] = Loss::process(coefficients, state, 1.0F);
        for (std::size_t n = 1; n < impulseLength; ++n)
            impulse[n] = Loss::process(coefficients, state, 0.0F);

        auto directCurrentGain = 0.0;
        for (const auto sample : impulse) directCurrentGain += sample;
        require(std::abs(directCurrentGain - 1.0) < 1.0e-6,
                "a boundary layer must not attenuate a static pressure difference");

        for (const auto frequency : { 120.0, 900.0, 2'500.0, 7'000.0, 15'000.0, 21'000.0 }) {
            const auto omega = 2.0 * std::numbers::pi * frequency / sampleRate;
            auto response = std::complex<double> {};
            for (std::size_t n = 0; n < impulseLength; ++n)
                response += impulse[n] * std::polar(1.0, -omega * static_cast<double>(n));
            require(std::abs(std::abs(response)
                        - Loss::magnitude(coefficients, frequency, sampleRate)) < 1.0e-6,
                    "the measured response must match the analytic shelf magnitude");
        }
    }

    // The fit must be exact where it is designed to be exact. The solve is
    // closed-form and exact in double; the residual here is the cost of storing
    // the two coefficients as float, about 3e-8.
    for (const auto frequency : { Loss::lowerDesignFrequencyHz, Loss::upperDesignFrequencyHz }) {
        const auto exact = Loss::traversalGain(frequency, traversal, radius,
                                               density, soundSpeed);
        require(std::abs(Loss::magnitude(coefficients, frequency, sampleRate) - exact) < 1.0e-6,
                "the shelf must match the exact attenuation at both design points");
    }

    // The property the fit exists for: track the Kirchhoff law across the whole
    // band, not only where it is pinned. The bound is the accuracy claimed in
    // DuctWallLoss's own documentation, derived from the sqrt(f) law rather than
    // from any rendered output, so it cannot drift onto the simulator. A pole
    // alone cannot hold it -- the one-pole this replaced was 11.9 dB out.
    {
        // Geometry and gas state spanning the catalogue: primaries from a
        // 34 mm motorcycle header to a 142 mm expansion chamber, cold ambient
        // air through to 1200 K exhaust.
        struct DuctCase final {
            double traversalSeconds;
            double radiusM;
            double densityKgPerM3;
            double soundSpeedMps;
        };
        constexpr std::array<DuctCase, 8> cases { {
            { 0.690 / 600.0, 0.024, 0.38, 600.0 },   // LS3 primary
            { 0.400 / 550.0, 0.071, 0.45, 550.0 },   // LS3 expansion chamber
            { 0.180 / 520.0, 0.041, 0.50, 520.0 },   // LS3 tailpipe
            { 0.760 / 600.0, 0.0225, 0.38, 600.0 },  // K20 primary
            { 0.610 / 620.0, 0.017, 0.36, 620.0 },   // Hayabusa primary
            { 0.150 / 640.0, 0.026, 0.35, 640.0 },   // Merlin stack
            { 0.120 / 580.0, 0.038, 0.42, 580.0 },   // collector trunk
            { 0.500 / 340.0, 0.020, 1.20, 340.0 },   // cold, dense
        } };
        constexpr double toleranceDb = 0.40;
        for (const auto& duct : cases) {
            const auto fitted = Loss::fit(duct.traversalSeconds, duct.radiusM,
                                          duct.densityKgPerM3, duct.soundSpeedMps,
                                          sampleRate);
            for (int step = 0; step <= 240; ++step) {
                const auto frequency = 80.0
                    * std::pow(sampleRate * 0.47 / 80.0, static_cast<double>(step) / 240.0);
                const auto exact = Loss::traversalGain(frequency, duct.traversalSeconds,
                                                       duct.radiusM, duct.densityKgPerM3,
                                                       duct.soundSpeedMps);
                const auto fittedGain = Loss::magnitude(fitted, frequency, sampleRate);
                const auto errorDb = 20.0 * std::log10(fittedGain / exact);
                require(std::abs(errorDb) < toleranceDb,
                        "the shelf must track the Kirchhoff law across the whole band");
            }
        }
    }

    // Monotone and bounded by unity across the band: the network runs this
    // filter inside a feedback loop, so amplification anywhere is fatal.
    {
        auto previous = 1.1;
        for (int step = 0; step <= 256; ++step) {
            const auto frequency = static_cast<double>(step) / 256.0 * sampleRate * 0.5;
            const auto magnitude = Loss::magnitude(coefficients, frequency, sampleRate);
            require(magnitude <= 1.0 + 1.0e-9, "wall loss must never amplify");
            require(magnitude <= previous + 1.0e-9, "wall loss must be monotone in frequency");
            previous = magnitude;
        }
    }

    // A duct too lossy for any passive first-order shelf must degrade to the
    // documented fallback rather than place the zero outside the pole, which
    // would amplify. Past the envelope the sqrt(f) law is steeper than a
    // first-order section can be, so the fallback's error is not sign-definite;
    // what must survive is passivity, monotonicity, and exactness where the
    // audible content of such a duct still is.
    {
        constexpr double lossyTraversal = 4.0 / 750.0;
        constexpr double lossyRadius = 0.005;
        const auto pathological = Loss::fit(lossyTraversal, lossyRadius, 0.25, 750.0,
                                            sampleRate);
        require(pathological.zero == 0.0F,
                "the fallback must drop the zero rather than place it outside the pole");
        require(std::abs(Loss::magnitude(pathological, Loss::lowerDesignFrequencyHz, sampleRate)
                    - Loss::traversalGain(Loss::lowerDesignFrequencyHz, lossyTraversal,
                                          lossyRadius, 0.25, 750.0)) < 1.0e-6,
                "the fallback must stay exact at the lower design frequency");
        auto previous = 1.1;
        for (int step = 0; step <= 128; ++step) {
            const auto frequency = static_cast<double>(step) / 128.0 * sampleRate * 0.5;
            const auto magnitude = Loss::magnitude(pathological, frequency, sampleRate);
            require(magnitude <= 1.0 + 1.0e-9, "the fallback must never amplify");
            require(magnitude <= previous + 1.0e-9, "the fallback must stay monotone");
            previous = magnitude;
        }
    }

    // The fallback must stay a guard on the extremes rather than quietly
    // swallowing real geometry: every duct the catalogue actually builds has to
    // be inside the envelope where the two-point shelf exists.
    {
        constexpr std::array<std::pair<double, double>, 4> catalogueExtremes { {
            { 0.760 / 600.0, 0.0225 },  // longest primary
            { 0.610 / 620.0, 0.017 },   // narrowest primary
            { 2.000 / 700.0, 0.0125 },  // beyond anything the catalogue authors
            { 0.400 / 550.0, 0.071 },   // widest chamber
        } };
        for (const auto& [traversalSeconds, radiusM] : catalogueExtremes)
            require(Loss::fit(traversalSeconds, radiusM, 0.30, 700.0, sampleRate).zero > 0.0F,
                    "catalogue geometry must be inside the shelf's fit envelope");
    }

    // A lossless or degenerate configuration must pass through untouched rather
    // than produce a silent or divergent line.
    Loss::State state;
    const auto degenerate = Loss::fit(0.0, radius, density, soundSpeed, sampleRate);
    require(std::abs(Loss::process(degenerate, state, 1.0F) - 1.0F) < 1.0e-6,
            "a zero-length duct must not attenuate");
    state.reset();
    (void) Loss::process(coefficients, state, std::numeric_limits<float>::infinity());
    require(std::isfinite(state.previousOutput),
            "wall-loss state must stay finite after a non-finite sample");

    // Hotter gas is more viscous, so a hotter duct attenuates more for the same
    // geometry. Sound speed carries the temperature.
    require(Loss::traversalGain(1'000.0, traversal, radius, density, 700.0)
                < Loss::traversalGain(1'000.0, traversal, radius, density, 400.0),
            "hotter gas must attenuate more at equal density and geometry");
}

// ---------------------------------------------------------------------------
// Duct plane-mode band limit
// ---------------------------------------------------------------------------

void ductModeCutoffRegression() {
    using Cutoff = enginelab::DuctModeCutoff;
    constexpr double sampleRate = 48'000.0;

    // The cutoff is the (1,0) mode of a rigid circular duct. Anchor it on the
    // textbook value rather than on anything this project produces: a 50 mm
    // duct in air at 343 m/s cuts on just above 4 kHz.
    {
        const auto textbook = Cutoff::cutoffFrequencyHz(0.025, 343.0);
        require(std::abs(textbook - 4'019.0) < 5.0,
                "a 50 mm duct in air must cut on at about 4 kHz");
    }
    // f_c scales with c and inversely with radius; those two dependences are
    // what make this a geometric differentiator rather than a tone control.
    require(std::abs(Cutoff::cutoffFrequencyHz(0.0125, 343.0)
                - 2.0 * Cutoff::cutoffFrequencyHz(0.025, 343.0)) < 1.0e-6,
            "halving the radius must double the cutoff");
    require(std::abs(Cutoff::cutoffFrequencyHz(0.025, 686.0)
                - 2.0 * Cutoff::cutoffFrequencyHz(0.025, 343.0)) < 1.0e-6,
            "doubling the sound speed must double the cutoff");
    // Degenerate geometry must not invent a band.
    require(Cutoff::cutoffFrequencyHz(0.0, 550.0) == 0.0
                && Cutoff::cutoffFrequencyHz(0.024, 0.0) == 0.0,
            "degenerate geometry must report no cutoff");

    // A 142 mm expansion chamber and a 34 mm header primary must land more than
    // two octaves apart. This is the separation the delivered network was
    // missing when the band limit came from a fitted filter corner instead.
    {
        const auto chamber = Cutoff::cutoffFrequencyHz(0.071, 550.0);
        const auto primary = Cutoff::cutoffFrequencyHz(0.017, 620.0);
        require(primary / chamber > 4.0,
                "chamber and primary cutoffs must differ by more than two octaves");
    }

    // Response of a representative expansion chamber.
    const auto chamberCutoffHz = Cutoff::cutoffFrequencyHz(0.071, 550.0);
    const auto coefficients = Cutoff::fit(0.071, 550.0, sampleRate);

    // The difference equation must realise the magnitude the class claims.
    {
        constexpr std::size_t impulseLength = 8'192;
        std::vector<double> impulse(impulseLength, 0.0);
        Cutoff::State state;
        impulse[0] = Cutoff::process(coefficients, state, 1.0F);
        for (std::size_t n = 1; n < impulseLength; ++n)
            impulse[n] = Cutoff::process(coefficients, state, 0.0F);
        for (const auto frequency : { 120.0, 600.0, 1'135.0, 2'270.0, 6'000.0, 12'000.0 }) {
            const auto omega = 2.0 * std::numbers::pi * frequency / sampleRate;
            auto response = std::complex<double> {};
            for (std::size_t n = 0; n < impulseLength; ++n)
                response += impulse[n] * std::polar(1.0, -omega * static_cast<double>(n));
            require(std::abs(std::abs(response)
                        - Cutoff::magnitude(coefficients, frequency, sampleRate)) < 1.0e-5,
                    "the measured response must match the analytic Butterworth magnitude");
        }
    }

    // Exactly -3 dB at the cutoff: the corner is the physical cutoff, not a
    // corner placed near it.
    {
        const auto atCutoff = Cutoff::magnitude(coefficients, chamberCutoffHz, sampleRate);
        require(std::abs(20.0 * std::log10(atCutoff) + 3.0103) < 1.0e-3,
                "the section must be 3 dB down exactly at the plane-mode cutoff");
    }

    // Transparent below cutoff. Evanescent modes store energy but dissipate
    // none, so the passband must not attenuate. This is the property that lets
    // the section run inside the collector-to-outlet feedback loop, where any
    // per-traversal loss compounds, and it is why the cascade is fourth order:
    // a second-order section would be 0.264 dB down an octave below cutoff.
    require(-20.0 * std::log10(Cutoff::magnitude(coefficients, chamberCutoffHz * 0.5,
                                                 sampleRate)) < 0.02,
            "one octave below cutoff must be transparent to 0.02 dB");
    for (const auto divisor : { 4.0, 8.0 }) {
        const auto lossDb = -20.0 * std::log10(
            Cutoff::magnitude(coefficients, chamberCutoffHz / divisor, sampleRate));
        require(lossDb < 0.001, "two or more octaves below cutoff must be lossless");
    }

    // Butterworth asymptote: 24 dB per octave. Measured on a deliberately low
    // corner, where both probe octaves sit far enough below Nyquist that the
    // bilinear frequency warping has not yet bent the slope.
    {
        const auto lowCorner = Cutoff::fit(0.3224, 550.0, sampleRate);
        const auto cornerHz = Cutoff::cutoffFrequencyHz(0.3224, 550.0);
        const auto atTwo = 20.0 * std::log10(
            Cutoff::magnitude(lowCorner, cornerHz * 2.0, sampleRate));
        const auto atFour = 20.0 * std::log10(
            Cutoff::magnitude(lowCorner, cornerHz * 4.0, sampleRate));
        require(std::abs((atTwo - atFour) - 24.0) < 1.0,
                "the stopband must fall at the fourth-order rate");
    }
    // Nearer Nyquist the bilinear map compresses the frequency axis, so the
    // realised slope is steeper than the analog prototype, never shallower. The
    // band limit is therefore at least as sharp as it claims everywhere.
    {
        const auto atTwo = 20.0 * std::log10(
            Cutoff::magnitude(coefficients, chamberCutoffHz * 2.0, sampleRate));
        const auto atFour = 20.0 * std::log10(
            Cutoff::magnitude(coefficients, chamberCutoffHz * 4.0, sampleRate));
        require(atTwo - atFour >= 24.0,
                "warping must only steepen the realised stopband");
    }

    // Passive and monotone: this runs in a feedback loop.
    {
        auto previous = 1.1;
        for (int step = 0; step <= 256; ++step) {
            const auto frequency = static_cast<double>(step) / 256.0 * sampleRate * 0.5;
            const auto magnitude = Cutoff::magnitude(coefficients, frequency, sampleRate);
            require(magnitude <= 1.0 + 1.0e-9, "the band limit must never amplify");
            require(magnitude <= previous + 1.0e-9, "the band limit must be monotone");
            previous = magnitude;
        }
    }

    // A duct narrow enough that its physical cutoff runs past Nyquist must clamp
    // to a transparent section rather than switch off discontinuously.
    {
        const auto narrow = Cutoff::fit(0.002, 620.0, sampleRate);
        require(narrow.g > 0.0F, "the clamped section must stay well formed");
        for (const auto frequency : { 500.0, 4'000.0, 12'000.0 }) {
            const auto lossDb = -20.0 * std::log10(
                Cutoff::magnitude(narrow, frequency, sampleRate));
            require(lossDb < 0.01,
                    "a duct whose cutoff exceeds Nyquist must pass the band untouched");
        }
    }

    // Numerical hygiene: a non-finite sample must not poison the integrators.
    {
        Cutoff::State state;
        (void) Cutoff::process(coefficients, state,
                               std::numeric_limits<float>::infinity());
        for (std::size_t section = 0; section < Cutoff::sectionCount; ++section)
            require(std::isfinite(state.integrator1[section])
                        && std::isfinite(state.integrator2[section]),
                    "band-limit state must stay finite after a non-finite sample");
        require(std::isfinite(Cutoff::process(coefficients, state, 1.0F)),
                "the section must recover after a non-finite sample");
    }
}

// ---------------------------------------------------------------------------
// Boundary reconstruction filter
// ---------------------------------------------------------------------------

// Cascade magnitude at one frequency from the stage coefficients, so the
// invariants below are stated in the frequency domain where they belong. An
// eighth-order Linkwitz-Riley is a fourth-order Butterworth run twice, so this
// is the product of both stages, squared.
double reconstructionMagnitude(
    const enginelab::BoundaryReconstructionFilter::Coefficients& c,
    double frequencyHz, double sampleRateHz) {
    const auto z1 = std::polar(1.0, -2.0 * std::numbers::pi * frequencyHz / sampleRateHz);
    const auto z2 = z1 * z1;
    auto butterworth = 1.0;
    for (const auto& stage : c.stage)
        butterworth *= std::abs(
            (stage.b0 + stage.b1 * z1 + stage.b2 * z2)
                / (1.0 + stage.a1 * z1 + stage.a2 * z2));
    return butterworth * butterworth;
}

// Invariants come from sampled-data theory, not from the simulator's output:
// a boundary sampled at rate fc carries nothing above fc/2, so the filter must
// preserve the band below and establish its stopband at the image lines, which
// sit at multiples of fc. Thresholds are the analytic response of an
// eighth-order Linkwitz-Riley cut at 0.47 fc, with margin.
void boundaryReconstructionRegression() {
    using Filter = enginelab::BoundaryReconstructionFilter;
    constexpr double sampleRate = 48'000.0;

    // A zero coupling rate means the boundary was never sampled -- authored
    // directly, as this suite's own fixtures do. Passthrough must be exact,
    // not merely close: nothing may be "compensated" that never happened.
    {
        Filter::State state;
        const auto inactive = Filter::compute(0.0, sampleRate);
        require(!inactive.active, "an unsampled boundary must disable the filter");
        require(Filter::process(inactive, state, 0.37) == 0.37,
                "an inactive reconstruction must be an exact passthrough");
    }

    // Representative coupling rates measured across the catalogue.
    for (const auto couplingHz : { 2'000.0, 2'900.0, 3'920.0, 5'800.0 }) {
        const auto c = Filter::compute(couplingHz, sampleRate);
        require(c.active, "a sampled boundary must enable the filter");
        require(std::abs(reconstructionMagnitude(c, 0.0, sampleRate) - 1.0) < 1.0e-9,
                "mean back-pressure must pass the reconstruction at unity");
        require(reconstructionMagnitude(c, couplingHz * 0.25, sampleRate) > 0.98,
                "content well inside the physical band must pass within 0.2 dB");
        // The first image has to clear the harness's metallic threshold with
        // room to spare, or it is heard as a whine that tracks rpm. A
        // fourth-order crossover left it at 28 dB and did not.
        require(reconstructionMagnitude(c, couplingHz, sampleRate) < 0.01,
                "the first image line must be attenuated by at least 40 dB");
        if (couplingHz * 2.0 < sampleRate * 0.5)
            require(reconstructionMagnitude(c, couplingHz * 2.0, sampleRate) < 1.0e-4,
                    "the second image line must be attenuated by at least 80 dB");

        // Stability: the impulse response must decay. Prime at zero first so
        // the steady-state initialisation does not swallow the impulse.
        Filter::State state;
        (void) Filter::process(c, state, 0.0);
        (void) Filter::process(c, state, 1.0);
        auto tail = 0.0;
        for (int n = 0; n < 4'000; ++n) {
            const auto y = Filter::process(c, state, 0.0);
            if (n >= 3'900) tail = std::max(tail, std::abs(y));
        }
        require(tail < 1.0e-9, "the reconstruction filter must be stable");
    }

    // The cutoff clamp must keep the bilinear design finite and stable even for
    // absurd coupling rates, and a non-finite rate must disable rather than
    // produce a divergent filter.
    {
        const auto clamped = Filter::compute(1.0e6, sampleRate);
        require(clamped.active
                    && std::all_of(clamped.stage.begin(), clamped.stage.end(),
                        [](const Filter::Biquad& stage) {
                            return std::isfinite(stage.b0) && std::isfinite(stage.a2);
                        }),
                "an extreme coupling rate must clamp to a finite design");
        require(!Filter::compute(std::nan(""), sampleRate).active,
                "a non-finite coupling rate must disable the filter");
    }

    // Enabling the filter mid-stream must settle to the running input instead
    // of ringing in from zero: the first active sample primes the state.
    {
        Filter::State state;
        const auto c = Filter::compute(2'900.0, sampleRate);
        require(std::abs(Filter::process(c, state, 42.0) - 42.0) < 1.0e-9,
                "the first filtered sample must reproduce its input exactly");
    }
}

// ---------------------------------------------------------------------------
// Complementary high-band valve-flow source
// ---------------------------------------------------------------------------

void valveFlowAcousticSourceRegression() {
    using Source = enginelab::ValveFlowAcousticSource;
    using LowPass = enginelab::BoundaryReconstructionFilter;
    constexpr double sampleRate = 48'000.0;
    constexpr double couplingRate = 4'000.0;
    const auto high = Source::compute(couplingRate, sampleRate);
    const auto low = LowPass::compute(couplingRate, sampleRate);
    require(high.active && low.active,
        "a sampled boundary must enable both halves of the crossover");

    const auto bandLimited = Source::compute(
        couplingRate, sampleRate, 10'000.0);
    require(bandLimited.upperBandLimited,
        "a mechanically sampled valve source must publish a finite upper band");
    // Butterworth cascade response at one frequency, from a stage pair.
    const auto butterworthResponse = [](const auto& stages, double frequencyHz) {
        const auto z1 = std::polar(
            1.0, -2.0 * std::numbers::pi * frequencyHz / sampleRate);
        const auto z2 = z1 * z1;
        auto response = std::complex<double> { 1.0, 0.0 };
        for (const auto& stage : stages)
            response *= (stage.b0 + stage.b1 * z1 + stage.b2 * z2)
                / (1.0 + stage.a1 * z1 + stage.a2 * z2);
        return response;
    };
    {
        // Above the mechanical solver's Nyquist the source stream carries only
        // first-order-hold images, and the derivative in the radiation path
        // turns them into clicks. Eighth order puts the first one 80 dB down.
        constexpr double imageFrequencyHz = 12'000.0;
        const auto section = butterworthResponse(
            bandLimited.upperStage, imageFrequencyHz);
        require(std::norm(section * section) < 1.0e-4,
            "the source reconstruction filter must reject images above mechanical Nyquist");
    }

    // An eighth-order Linkwitz-Riley is a fourth-order Butterworth run twice.
    // Its low and high outputs have a flat coherent sum, so the transition
    // neither duplicates nor removes a band when both inputs agree. This is the
    // property that lets the crossover order be raised without retuning either
    // physical band.
    for (int step = 0; step <= 96; ++step) {
        const auto frequency = static_cast<double>(step) / 96.0
            * sampleRate * 0.5;
        const auto lowSection = butterworthResponse(low.stage, frequency);
        const auto highSection = butterworthResponse(high.highStage, frequency);
        const auto coherentSum = lowSection * lowSection
            + highSection * highSection;
        require(std::abs(std::abs(coherentSum) - 1.0) < 1.0e-9,
            "the low/high crossover sum must remain all-pass");
    }

    // An authored full-band boundary has no sampled-data gap, so the supplement
    // must be exactly absent. A steady mean flow must likewise produce no
    // acoustic source when the crossover is active.
    Source::State state;
    const auto disabled = Source::process(Source::compute(0.0, sampleRate), state,
        0.05, 0.5, 200'000.0);
    require(disabled.outgoingPressurePa == 0.0
            && disabled.incomingPressurePa == 0.0
            && disabled.highBandMassFlowKgPerSecond == 0.0,
        "an unsampled full-band boundary must not receive a duplicate source");
    state.reset();
    const auto steady = Source::process(high, state, 0.05, 0.5, 200'000.0);
    require(std::abs(steady.highBandMassFlowKgPerSecond) < 1.0e-15,
        "steady valve flow must not create a high-band source");

    // The source is a pure volume-velocity contribution: its characteristic
    // pressure components are antisymmetric and recover the filtered signed
    // mass flow exactly through U=(p+ - p-)/Zc and mdot=rho*U.
    const auto transient = Source::process(high, state, -0.025, 0.5, 200'000.0);
    require(std::abs(transient.outgoingPressurePa
                     + transient.incomingPressurePa) < 1.0e-12,
        "a valve-flow source must add no fabricated pressure boundary");
    const auto recoveredMassFlow = 0.5
        * (transient.outgoingPressurePa - transient.incomingPressurePa)
        / 200'000.0;
    require(std::abs(recoveredMassFlow
                     - transient.highBandMassFlowKgPerSecond) < 1.0e-12,
        "characteristic source waves must preserve signed volume flow");

    const auto guarded = Source::process(high, state, std::nan(""), 0.0, 0.0);
    require(guarded.outgoingPressurePa == 0.0
            && guarded.incomingPressurePa == 0.0,
        "invalid valve medium data must fail silent without non-finite waves");
}

// ---------------------------------------------------------------------------
// Finite-amplitude duct propagation (steepening)
// ---------------------------------------------------------------------------

// References are analytic, not simulator output. A finite-amplitude simple wave
// steepens as it propagates, and to leading order in the acoustic Mach number
// its second harmonic grows as
//     B_2 / B_1 -> sigma / 2,   sigma = beta * (p / rho c^2) * omega * T,
// which is the O(sigma) term of the exact pre-shock Fubini series
//     B_n / B_1 = [J_n(n sigma) / (n sigma)] / [J_1(sigma) / sigma].
// The single-probe steepened read is a FIRST-ORDER scheme: it reproduces that
// O(sigma) second-harmonic law but deliberately under-generates the O(sigma^2)
// third harmonic (an exact match there needs a shock-capturing characteristics
// solver, which is out of scope for an audio-band effect). The invariants below
// therefore assert exactly what first-order theory guarantees -- the leading
// second-harmonic law, a present and monotonically decreasing cascade, growth
// with drive, and passivity -- and nothing the scheme is not built to deliver.
void nonlinearDuctAcousticsRegression() {
    namespace Duct = enginelab::NonlinearDuctAcoustics;

    // Transparency invariants: zero amplitude, an invalid medium, or a
    // non-finite sample must reproduce the linear read exactly.
    require(Duct::delayScale(0.0F, 150'000.0F) == 1.0F,
            "zero amplitude must propagate at exactly the linear speed");
    require(Duct::delayScale(5'000.0F, 0.0F) == 1.0F,
            "an unpublished medium must disable the correction");
    require(Duct::delayScale(std::numeric_limits<float>::quiet_NaN(), 150'000.0F) == 1.0F,
            "a non-finite sample must not corrupt the read position");
    // Sign: compressions travel faster than c, rarefactions slower.
    require(Duct::delayScale(10'000.0F, 150'000.0F) < 1.0F,
            "a compression must arrive earlier than the linear wave");
    require(Duct::delayScale(-10'000.0F, 150'000.0F) > 1.0F,
            "a rarefaction must arrive later than the linear wave");
    // The Mach clamp bounds the correction for absurd amplitudes.
    require(std::abs(Duct::delayScale(1.0e9F, 150'000.0F) - 1.0F / 1.3F) < 1.0e-4F,
            "the compression clamp must bound the correction at +30% Mach");

    // Hot-exhaust medium, 400 Hz tone, 2 ms transit. Drive a pure sine through
    // the steepened read and project the output onto its first harmonics
    // (integer FFT bins, so no window is needed). Returns B1..B4 magnitudes and
    // the input/output power, for a requested normalised distance sigma.
    constexpr double sampleRate = 48'000.0;
    constexpr double toneHz = 400.0;
    constexpr float rhoC2 = 0.52F * 550.0F * 550.0F; // 157.3 kPa
    constexpr float nominalDelay = 96.0F;             // 2 ms transit
    constexpr int warmupSamples = 2 * static_cast<int>(nominalDelay);
    constexpr int analysisSamples = 4'800;            // exact tone periods
    struct Cascade { std::array<double, 4> b; double inPow, outPow; };
    const auto runCascade = [&](double sigma, bool steepen) {
        const auto amplitudePa = static_cast<float>(sigma * rhoC2
            / (Duct::coefficientOfNonlinearity * 2.0 * std::numbers::pi * toneHz
               * (nominalDelay / sampleRate)));
        std::vector<float> line(256, 0.0F);
        const std::size_t mask = line.size() - 1;
        std::size_t write = 0;
        const auto readDelayed = [&line, &write, mask](float delaySamples) {
            const auto delay0 = static_cast<std::size_t>(delaySamples);
            const auto fraction = delaySamples - static_cast<float>(delay0);
            const auto read0 = (write + line.size() - delay0) & mask;
            const auto read1 = (write + line.size() - delay0 - 1U) & mask;
            return std::lerp(line[read0], line[read1], fraction);
        };
        std::array<std::complex<double>, 4> bins {};
        double inPow = 0.0, outPow = 0.0;
        for (int n = 0; n < warmupSamples + analysisSamples; ++n) {
            const auto phase = 2.0 * std::numbers::pi * toneHz
                * static_cast<double>(n) / sampleRate;
            const auto input = amplitudePa * static_cast<float>(std::sin(phase));
            line[write] = input;
            write = (write + 1U) & mask;
            const auto output = Duct::steepenedRead(readDelayed, nominalDelay,
                static_cast<float>(line.size() - 2), steepen ? rhoC2 : 0.0F);
            if (n < warmupSamples) continue;
            inPow += static_cast<double>(input) * static_cast<double>(input);
            outPow += static_cast<double>(output) * static_cast<double>(output);
            for (std::size_t h = 0; h < bins.size(); ++h)
                bins[h] += static_cast<double>(output)
                    * std::polar(1.0, -phase * static_cast<double>(h + 1));
        }
        Cascade result { {}, inPow, outPow };
        for (std::size_t h = 0; h < bins.size(); ++h) result.b[h] = std::abs(bins[h]);
        return result;
    };

    // Leading-order law: at a small normalised distance where first-order
    // theory is accurate, B2/B1 must approach sigma/2. sigma = 0.3 -> 0.15.
    {
        const auto c = runCascade(0.3, true);
        require(c.b[0] > 0.0, "the fundamental must survive the read");
        const auto second = c.b[1] / c.b[0];
        require(std::abs(second - 0.15) < 0.15 * 0.30,
                "second-harmonic growth must follow the leading-order sigma/2 law");
        // A real steepening cascade: harmonics present and monotonically
        // decreasing, not a single spurious tone.
        require(c.b[1] > c.b[2] && c.b[2] > c.b[3] && c.b[3] > 0.0,
                "the harmonic cascade must decrease monotonically and be fully populated");
    }

    // Amplitude dependence is the signature of nonlinearity: doubling the drive
    // must roughly double B2/B1 (a linear delay line would leave it at zero).
    {
        const auto low = runCascade(0.25, true);
        const auto high = runCascade(0.50, true);
        const auto ratioLow = low.b[1] / low.b[0];
        const auto ratioHigh = high.b[1] / high.b[0];
        require(ratioHigh > ratioLow * 1.6,
                "second-harmonic content must grow with drive level");
        // Passivity: the read is a pure time warp and cannot add energy.
        require(high.outPow <= high.inPow * 1.02,
                "steepening must not amplify the wave");
    }

    // Control: with the medium disabled the identical drive must leave the tone
    // undistorted, proving the cascade came from the model, not the fractional
    // interpolation in the read.
    {
        const auto linear = runCascade(0.50, false);
        require(linear.b[1] / linear.b[0] < 1.0e-4,
                "the linear read must not distort the tone");
    }
}

/**
 * The expansion-chamber silencer against Munjal's transmission loss.
 *
 * The reference is the closed-form single-expansion-chamber result
 *     TL = 10 log10 [ 1 + 1/4 (m - 1/m)^2 sin^2(k L) ]
 * (Munjal, *Acoustics of Ducts and Mufflers*, ch. 2) -- literature, not the
 * simulator's own output, so tightening this gate later cannot recalibrate it
 * onto the behaviour it exists to catch.
 *
 * Unlike the first-order duct steepening, this element is an *exact* structural
 * realisation of the theory: two Kelly-Lochbaum junctions and an integer delay
 * reproduce the transfer function term for term. It is therefore held to a
 * tight tolerance, and a loose one here would mean a wiring mistake.
 */
void expansionChamberMufflerRegression() {
    using Muffler = enginelab::ExpansionChamberMuffler;
    constexpr double sampleRate = 48'000.0;
    constexpr std::size_t traversalSamples = 40;
    constexpr double traversalSeconds = traversalSamples / sampleRate;

    // Steady-state pressure transmission at one frequency, with both ports
    // anechoic: nothing is fed back in from the outlet side, and the wave
    // returning to the collector is discarded.
    const auto measureTransmissionDb = [&](double expansionRatio, double frequencyHz) {
        Muffler::State state;
        state.prepare(256);
        Muffler::Coefficients c;
        c.enabled = true;
        c.reflection = Muffler::reflectionCoefficient(1.0, expansionRatio);
        c.delaySamples = static_cast<float>(traversalSamples);

        constexpr std::size_t settle = 20'000;
        constexpr std::size_t window = 4'800; // 0.1 s: every test tone is a whole
                                              // number of cycles in this window.
        const auto omega = 2.0 * std::numbers::pi * frequencyHz / sampleRate;
        std::complex<double> transmitted { 0.0, 0.0 };
        for (std::size_t n = 0; n < settle + window; ++n) {
            const auto phase = omega * static_cast<double>(n);
            const auto out = Muffler::process(state, c,
                static_cast<float>(std::sin(phase)), 0.0F);
            if (n >= settle)
                transmitted += std::complex<double>(out.towardOutlet, 0.0)
                    * std::exp(std::complex<double>(0.0, -phase));
        }
        // The window holds an integer number of cycles, so the analysis bin
        // carries half the amplitude of a unit sine.
        const auto amplitude = 2.0 * std::abs(transmitted)
            / static_cast<double>(window);
        return -20.0 * std::log10(std::max(amplitude, 1.0e-12));
    };

    // A 2.5:1 diameter step, the order of a real road silencer.
    constexpr double expansionRatio = 6.25;
    // Stop-band peak (kL = pi/2), a partial band, and the pass-band (kL = pi).
    for (const auto frequencyHz : { 150.0, 300.0, 450.0, 600.0 }) {
        const auto expected = Muffler::transmissionLossDb(
            expansionRatio, frequencyHz, traversalSeconds);
        const auto measured = measureTransmissionDb(expansionRatio, frequencyHz);
        require(std::abs(measured - expected) < 0.25,
            "a lossless chamber must reproduce Munjal's transmission loss");
    }

    // The pass-band is the signature of a reactive chamber: at kL = pi the
    // chamber is acoustically transparent however large the expansion.
    require(measureTransmissionDb(expansionRatio, 600.0) < 0.25,
        "a lossless chamber must be transparent in its pass-band");
    require(measureTransmissionDb(16.0, 600.0) < 0.25,
        "the pass-band must not depend on the expansion ratio");

    // Deeper stop-band with a larger expansion ratio, which is the whole reason
    // geometry differentiates one engine's silencer from another's.
    require(measureTransmissionDb(16.0, 300.0)
            > measureTransmissionDb(6.25, 300.0) + 3.0,
        "a larger expansion ratio must deepen the stop-band");

    // Passivity. A silencer may never return more energy than it is given, in
    // either direction, or the waveguide it sits in can run away.
    {
        Muffler::State state;
        state.prepare(256);
        Muffler::Coefficients c;
        c.enabled = true;
        c.reflection = Muffler::reflectionCoefficient(1.0, expansionRatio);
        c.delaySamples = static_cast<float>(traversalSamples);
        auto inPower = 0.0;
        auto outPower = 0.0;
        std::uint32_t noise = 12'345U;
        for (std::size_t n = 0; n < 200'000; ++n) {
            noise ^= noise << 13U; noise ^= noise >> 17U; noise ^= noise << 5U;
            const auto drive = static_cast<float>(noise) / 2'147'483'648.0F - 1.0F;
            const auto fromOutlet = n < 100'000 ? 0.0F : drive * 0.5F;
            const auto out = Muffler::process(state, c, drive, fromOutlet);
            inPower += static_cast<double>(drive) * drive
                + static_cast<double>(fromOutlet) * fromOutlet;
            outPower += static_cast<double>(out.towardOutlet) * out.towardOutlet
                + static_cast<double>(out.towardCollector) * out.towardCollector;
        }
        require(outPower <= inPower * 1.02,
            "the chamber must be passive in both directions");
    }

    // Transparency. An engine with no chamber must render exactly as it did
    // before this element existed -- not approximately.
    {
        Muffler::State state;
        state.prepare(256);
        const Muffler::Coefficients disabled {};
        for (std::size_t n = 0; n < 64; ++n) {
            const auto a = static_cast<float>(n) * 0.37F - 5.0F;
            const auto b = static_cast<float>(n) * -0.11F + 2.0F;
            const auto out = Muffler::process(state, disabled, a, b);
            require(out.towardOutlet == a && out.towardCollector == b,
                "a disabled chamber must be an exact through-connection");
        }
    }

    // A unit expansion ratio is a straight pipe, so it must not attenuate.
    require(measureTransmissionDb(1.0, 300.0) < 1.0e-6,
        "a chamber with no expansion must be acoustically absent");
}

// The runtime's published calibration default is duplicated from the audio
// module's documented constant (audio depends on runtime, so it cannot be
// included there). If they drift apart the delivered level stops matching the
// calibration the documentation derives, silently.
void monitorCalibrationDefaultRegression() {
    enginelab::RealtimeAudioState state;
    require(std::abs(static_cast<double>(
                state.acousticFullScaleSplDb.load())
            - enginelab::AcousticMonitorCalibration::defaultFullScaleSplDb) < 1.0e-6,
        "runtime full-scale SPL default must match the documented calibration");
}

void freeFieldObserverRegression() {
    constexpr double sampleRate = 48'000.0;
    enginelab::AcousticObserverConfig geometry;
    geometry.leftMicrophoneM = { 0.0, 1.0, 0.0 };
    geometry.rightMicrophoneM = { 0.0, 2.0, 0.0 };
    geometry.soundSpeedMps = 343.0;
    enginelab::FreeFieldObserver observer;
    require(observer.prepare(sampleRate, 0.04, {}, { 0.0, 1.0, 0.0 },
            enginelab::AcousticTerminationType::unflanged, geometry),
        "valid SI observer geometry must prepare");
    auto leftEnergy = 0.0;
    auto rightEnergy = 0.0;
    for (std::size_t sample = 0; sample < 8'192; ++sample) {
        const auto drive = static_cast<float>(std::sin(
            2.0 * std::numbers::pi * 500.0
                * static_cast<double>(sample) / sampleRate));
        const auto output = observer.process(drive);
        if (sample > 1'024) {
            leftEnergy += static_cast<double>(output.leftPa) * output.leftPa;
            rightEnergy += static_cast<double>(output.rightPa) * output.rightPa;
        }
    }
    require(std::abs(std::sqrt(leftEnergy / rightEnergy) - 2.0) < 0.02,
        "free-field pressure must decay exactly as inverse distance");

    geometry.leftMicrophoneM = { 0.0, 1.0, 0.0 };
    geometry.rightMicrophoneM = { 0.0, -1.0, 0.0 };
    require(observer.prepare(sampleRate, 0.04, {}, { 0.0, 1.0, 0.0 },
            enginelab::AcousticTerminationType::flanged, geometry),
        "flanged observer geometry must prepare");
    leftEnergy = 0.0;
    rightEnergy = 0.0;
    for (std::size_t sample = 0; sample < 8'192; ++sample) {
        const auto drive = static_cast<float>(std::sin(
            2.0 * std::numbers::pi * 8'000.0
                * static_cast<double>(sample) / sampleRate));
        const auto output = observer.process(drive);
        if (sample > 1'024) {
            leftEnergy += static_cast<double>(output.leftPa) * output.leftPa;
            rightEnergy += static_cast<double>(output.rightPa) * output.rightPa;
        }
    }
    require(rightEnergy < leftEnergy * 0.15,
        "a rigid flange must suppress the rear high-frequency hemisphere");
}

int main() {
    try {
        monitorCalibrationDefaultRegression();
        freeFieldObserverRegression();
        ductWallLossRegression();
        ductModeCutoffRegression();
        valvePortTerminationRegression();
        boundaryReconstructionRegression();
        valveFlowAcousticSourceRegression();
        nonlinearDuctAcousticsRegression();
        expansionChamberMufflerRegression();
        latencyAndBlockSizeRegression();
        compressionIgnitionTimbreRegression();
        runnerDelaySampleRateRegression();
        exhaustPathIsolationRegression();
        exhaustJetNoiseRegression();
        branchedAcousticTopologyRegression();
        branchTrunkDelayRegression();
        ductMediumRegression();
        areaStepScatteringRegression();
        structuralModalRadiatorRegression();
        acousticIntakeNetworkRegression();
        forcedInductionAcousticsRegression();
        customGraphRuntimeTelemetryRegression();
        ambientPressureRegression();
        diagnosticStemRegression();
        physicalThermoacousticPathRegression();
        pipeRadiationRegression();
        acousticMonitorCalibrationRegression();
        outputQualityRegression();
        std::cout << "Realtime audio/runtime regression tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Realtime regression failure: " << error.what() << '\n';
        return 1;
    }
}
