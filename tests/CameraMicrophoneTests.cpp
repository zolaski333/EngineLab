// A listener that moves: the camera microphone of the 3-D view.
// FreeFieldObserver::moveMicrophones glides each microphone's delay, 1/r and
// directivity; RealtimeEngineAudio::setMicrophones moves every spatialised
// layer. Off, or at the authored positions, the sound is the default one.

#include <enginelab/audio/FreeFieldObserver.hpp>
#include <enginelab/audio/RealtimeEngineAudio.hpp>
#include <enginelab/audio/StructuralModalRadiator.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/runtime/EngineRuntime.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>
#include <juce_audio_basics/juce_audio_basics.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace enginelab::tests {
namespace {

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

constexpr double sampleRate = 48'000.0;

[[nodiscard]] double decibels(double ratio) { return 20.0 * std::log10(std::max(ratio, 1.0e-30)); }

[[nodiscard]] double rms(const std::vector<float>& values, std::size_t begin, std::size_t end) {
    double sum = 0.0;
    end = std::min(end, values.size());
    for (auto index = begin; index < end; ++index) sum += double(values[index]) * values[index];
    return std::sqrt(sum / static_cast<double>(std::max<std::size_t>(1, end - begin)));
}

[[nodiscard]] double largestStep(const std::vector<float>& values, std::size_t begin, std::size_t end) {
    double largest = 0.0;
    end = std::min(end, values.size());
    for (auto index = std::max<std::size_t>(begin, 1); index < end; ++index)
        largest = std::max(largest, std::abs(double(values[index]) - values[index - 1]));
    return largest;
}

[[nodiscard]] std::size_t at(double seconds) { return static_cast<std::size_t>(seconds * sampleRate); }

[[nodiscard]] AcousticPoint3M scaled(const AcousticPoint3M& point, double scale) {
    return { point.x * scale, point.y * scale, point.z * scale };
}

/** A source at the origin pointing at the listener, heard by the default
 * microphones (4 m behind the engine). */
[[nodiscard]] FreeFieldObserver preparedObserver() {
    FreeFieldObserver observer;
    require(observer.prepare(sampleRate, 0.025, {}, { 0.0, 1.0, 0.0 }, AcousticTerminationType::unflanged,
                             AcousticObserverConfig {}),
            "the observer prepares");
    return observer;
}

/** Moving a microphone to where it already is changes nothing, bit for bit;
 * a microphone moved twice as far glides to half the pressure with its pitch
 * within 2 % meanwhile; `immediately` jumps; a microphone at the source is
 * kept at the closest distance. */
void observerGlides() {
    const AcousticObserverConfig authored;
    const std::array positions { effectiveMicrophonePosition(authored, false),
                                 effectiveMicrophonePosition(authored, true) };
    auto reference = preparedObserver();
    auto moved = preparedObserver();
    moved.moveMicrophones(positions[0], positions[1]);
    std::uint32_t noise = 0x1234567U;
    for (auto sample = 0; sample < 48'000; ++sample) {
        noise = noise * 1'664'525U + 1'013'904'223U;
        const auto input = static_cast<float>(noise >> 8U) / 16'777'216.0F - 0.5F;
        const auto a = reference.process(input);
        const auto b = moved.process(input);
        require(a.leftPa == b.leftPa && a.rightPa == b.rightPa,
                "a microphone moved to where it is changes nothing");
    }

    // A 1 kHz tone; the microphones go twice as far after 1 s (the delay lines full).
    constexpr double toneHz = 1'000.0;
    auto glider = preparedObserver();
    std::vector<float> left;
    const auto total = static_cast<int>(5.0 * sampleRate);
    const auto moveAt = static_cast<std::size_t>(1.0 * sampleRate);
    for (auto sample = 0; sample < total; ++sample) {
        if (static_cast<std::size_t>(sample) == moveAt)
            glider.moveMicrophones(scaled(positions[0], 2.0), scaled(positions[1], 2.0));
        left.push_back(glider.process(static_cast<float>(
            std::sin(2.0 * std::numbers::pi * toneHz * sample / sampleRate))).leftPa);
    }
    // Pitch from the rising zero crossings, over the whole glide.
    double lowestHz = 1.0e9;
    double highestHz = 0.0;
    std::optional<double> lastCrossing;
    for (auto index = moveAt; index + 1 < left.size(); ++index) {
        if (!(left[index] <= 0.0F && left[index + 1] > 0.0F)) continue;
        const auto crossing = static_cast<double>(index)
            + left[index] / static_cast<double>(left[index] - left[index + 1]);
        if (lastCrossing) {
            const auto hz = sampleRate / (crossing - *lastCrossing);
            lowestHz = std::min(lowestHz, hz);
            highestHz = std::max(highestHz, hz);
        }
        lastCrossing = crossing;
    }
    const auto before = rms(left, moveAt - 24'000, moveAt);
    const auto after = rms(left, left.size() - 24'000, left.size());
    std::cout << "  observer 2x farther: " << decibels(after / before) << " dB, pitch " << lowestHz
              << " to " << highestHz << " Hz during the glide\n";
    require(std::abs(decibels(after / before) + 6.02) < 0.1, "twice as far is 6 dB quieter");
    require(lowestHz >= 0.98 * toneHz - 0.5 && highestHz <= 1.02 * toneHz + 0.5,
            "the glide shifts the pitch by 2 % at most");
    require(lowestHz < 0.995 * toneHz, "the glide is heard as a glide (else this proves nothing)");

    // Immediately: the next samples come from the new distance's delay.
    auto jumper = preparedObserver();
    auto far = preparedObserver();
    far.moveMicrophones(scaled(positions[0], 2.0), scaled(positions[1], 2.0), true);
    jumper.moveMicrophones(scaled(positions[0], 2.0), scaled(positions[1], 2.0), true);
    for (auto sample = 0; sample < 48'000; ++sample) {
        const auto input = static_cast<float>(std::sin(2.0 * std::numbers::pi * toneHz * sample / sampleRate));
        const auto a = jumper.process(input);
        const auto b = far.process(input);
        require(a.leftPa == b.leftPa, "an immediate move is a jump");
    }
    std::vector<float> jumped;
    auto quick = preparedObserver();
    for (auto sample = 0; sample < 48'000; ++sample) {
        if (sample == 24'000) quick.moveMicrophones(scaled(positions[0], 2.0), scaled(positions[1], 2.0), true);
        jumped.push_back(quick.process(static_cast<float>(
            std::sin(2.0 * std::numbers::pi * toneHz * sample / sampleRate))).leftPa);
    }
    require(std::abs(decibels(rms(jumped, 40'000, 48'000) / rms(jumped, 12'000, 24'000)) + 6.02) < 0.1,
            "an immediate move is at its distance within the delay");

    // At the source itself: the closest distance, finite.
    auto close = preparedObserver();
    close.moveMicrophones({}, {}, true);
    require(std::abs(close.distanceM(0) - FreeFieldObserver::minimumMicrophoneDistanceM) < 1.0e-9,
            "a microphone at the source is kept at the closest distance");
    for (auto sample = 0; sample < 4'800; ++sample)
        require(std::isfinite(close.process(1.0F).leftPa), "a close microphone stays finite");
    // 5 cm in front of the opening: kept at the closest distance, on that side.
    auto nearly = preparedObserver();
    nearly.moveMicrophones({ 0.0, 0.05, 0.0 }, { 0.03, 0.04, 0.0 }, true);
    require(std::abs(nearly.distanceM(0) - FreeFieldObserver::minimumMicrophoneDistanceM) < 1.0e-9
                && std::abs(nearly.distanceM(1) - FreeFieldObserver::minimumMicrophoneDistanceM) < 1.0e-9,
            "a microphone inside the closest distance is kept there");
}

/** Moving the opening to where it is changes nothing, bit for bit; moved
 * halfway to the microphones, a 1 kHz tone is louder as 1/r says; moved back,
 * the glide lands on the unmoved sound exactly. */
void observerMovesItsSource() {
    const AcousticObserverConfig authored;
    const auto left = effectiveMicrophonePosition(authored, false);
    const auto right = effectiveMicrophonePosition(authored, true);
    auto reference = preparedObserver();
    auto same = preparedObserver();
    same.moveSource({}, { 0.0, 2.0, 0.0 });
    constexpr double toneHz = 1'000.0;
    const AcousticPoint3M halfway { 0.25 * (left.x + right.x), 0.25 * (left.y + right.y), 0.25 * (left.z + right.z) };
    auto mover = preparedObserver();
    std::vector<float> referenceLeft;
    std::vector<float> movedLeft;
    for (auto sample = 0; sample < static_cast<int>(8.0 * sampleRate); ++sample) {
        if (sample == static_cast<int>(1.0 * sampleRate)) mover.moveSource(halfway, { 0.0, 1.0, 0.0 });
        if (sample == static_cast<int>(3.0 * sampleRate)) mover.moveSource({}, { 0.0, 1.0, 0.0 });
        const auto input = static_cast<float>(std::sin(2.0 * std::numbers::pi * toneHz * sample / sampleRate));
        const auto a = reference.process(input);
        const auto b = same.process(input);
        require(a.leftPa == b.leftPa && a.rightPa == b.rightPa, "an opening moved to where it is changes nothing");
        referenceLeft.push_back(a.leftPa);
        movedLeft.push_back(mover.process(input).leftPa);
    }
    const auto distance = [](const AcousticPoint3M& a, const AcousticPoint3M& b) {
        return std::hypot(a.x - b.x, a.y - b.y, a.z - b.z);
    };
    const auto expectedDb = decibels(distance(left, {}) / distance(left, halfway));
    const auto db = decibels(rms(movedLeft, at(2.5), at(3.0)) / rms(referenceLeft, at(2.5), at(3.0)));
    std::size_t lastDifferent = 0;
    for (auto index = at(3.0); index < movedLeft.size(); ++index)
        if (movedLeft[index] != referenceLeft[index]) lastDifferent = index;
    const auto backAtSeconds = static_cast<double>(lastDifferent) / sampleRate;
    std::cout << "  opening moved halfway to the listener: " << db << " dB (1/r: " << expectedDb
              << " dB); moved back at 3 s, the unmoved sound bit for bit from " << backAtSeconds << " s\n";
    require(std::abs(db - expectedDb) < 0.2, "an opening halfway to the listener is louder as 1/r says");
    require(lastDifferent > at(3.0) && backAtSeconds < 5.0, "moved back, the opening sounds as before, bit for bit");
}

/** One simulator heard by two renderers; the second one is `listen`ed to by
 * the test at each frame (its microphones moved). */
struct Listening final {
    std::vector<float> referenceMaster;
    std::vector<float> movedMaster;
    std::vector<float> referenceExhaust;
    std::vector<float> movedExhaust;
    std::vector<float> referenceTurbo;
    std::vector<float> movedTurbo;
    std::vector<float> referenceIntake;
    std::vector<float> movedIntake;
};

struct Rig final {
    explicit Rig(EngineConfig engine)
        : config(std::move(engine)), exhaust(ExhaustGraph::makeForEngine(config)),
          simulator(std::make_unique<EngineSimulator>(config, ecu, physics, events, exhaust)) {
        simulator->setPressureSamplingEnabled(true);
    }
    EngineConfig config;
    SimpleEcuModel ecu;
    SimplifiedGasolinePhysics physics;
    FourStrokeEventGenerator events;
    ExhaustGraph exhaust;
    std::unique_ptr<EngineSimulator> simulator;
};

[[nodiscard]] EngineConfig catalogueEngine(std::string_view selector) {
    static const auto catalogue = loadEngineCatalog(ENGINELAB_CATALOG_ROOT);
    const auto selected = selectSingleEngineCatalogEntry(catalogue.entries, selector);
    require(static_cast<bool>(selected), "catalogue engine " + std::string(selector));
    auto config = selected.entry->config;
    normaliseEngineConfig(config);
    return config;
}

[[nodiscard]] Listening listen(const EngineConfig& config, double durationSeconds,
                               const std::function<void(RealtimeEngineAudio&, double)>& action) {
    constexpr double frameSeconds = 1.0 / 240.0;
    constexpr int samplesPerFrame = 200;
    Rig rig(config);
    struct Listener final {
        std::unique_ptr<FiringEventQueue> events = std::make_unique<FiringEventQueue>();
        std::unique_ptr<CylinderPressureQueue> pressure = std::make_unique<CylinderPressureQueue>();
        std::unique_ptr<EngineRuntime> runtime;
        std::unique_ptr<RealtimeEngineAudio> renderer;
    };
    std::array<Listener, 2> listeners;
    for (auto& listener : listeners) {
        listener.runtime = std::make_unique<EngineRuntime>(config);
        listener.renderer = std::make_unique<RealtimeEngineAudio>(
            *listener.events, listener.runtime->audioState(), listener.pressure.get(),
            &listener.runtime->exhaustGraph(), &listener.runtime->engineConfig(),
            &listener.runtime->exhaustAcousticSamples());
        listener.renderer->prepare(sampleRate, samplesPerFrame);
        // Outlet turbulence is seeded noise: the two would differ anyway.
        listener.renderer->setOutletJetNoiseEnabled(false);
    }
    Listening result;
    juce::AudioBuffer<float> block(2, samplesPerFrame);
    juce::AudioBuffer<float> exhaust(2, samplesPerFrame);
    juce::AudioBuffer<float> turbo(2, samplesPerFrame);
    juce::AudioBuffer<float> intake(2, samplesPerFrame);
    double rpm = 0.0;
    const auto frames = static_cast<int>(durationSeconds / frameSeconds);
    for (auto frame = 0; frame < frames; ++frame) {
        const auto t = frame * frameSeconds;
        EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = t < 1.5;
        controls.throttle = t < 2.0 ? 0.0 : 0.6;
        auto step = rig.simulator->step(frameSeconds, controls);
        rpm = step.state.rpm;
        for (std::size_t index = 0; index < step.firingEventCount; ++index)
            for (auto& listener : listeners) (void)listener.events->tryPush(step.firingEvents[index]);
        CylinderPressureSample pressure;
        while (rig.simulator->tryPopCylinderPressureSample(pressure))
            for (auto& listener : listeners) (void)listener.pressure->tryPush(pressure);
        ExhaustAcousticSample acoustic;
        while (rig.simulator->tryPopExhaustAcousticSample(acoustic))
            for (auto& listener : listeners) (void)listener.runtime->exhaustAcousticSamples().tryPush(acoustic);
        action(*listeners[1].renderer, t);
        for (std::size_t index = 0; index < listeners.size(); ++index) {
            auto& listener = listeners[index];
            publishAudioFrame(listener.runtime->audioState(), step.state,
                              { false, controls.starterEngaged, 0.0, 1.0 });
            listener.runtime->audioState().producerTimeNanoseconds.store(
                static_cast<std::uint64_t>((t + frameSeconds) * 1.0e9), std::memory_order_release);
            block.clear();
            RealtimeAudioStemBuffers stems;
            stems.exhaustPressureWave = &exhaust;
            stems.forcedInduction = &turbo;
            stems.intake = &intake;
            listener.renderer->renderWithStems(block, 0, samplesPerFrame, stems);
            listener.renderer->collectRetiredExhaustNetworks();
            const auto first = index == 0;
            for (int sample = 0; sample < samplesPerFrame; ++sample) {
                (first ? result.referenceMaster : result.movedMaster).push_back(block.getSample(0, sample));
                (first ? result.referenceExhaust : result.movedExhaust).push_back(exhaust.getSample(0, sample));
                (first ? result.referenceTurbo : result.movedTurbo).push_back(turbo.getSample(0, sample));
                (first ? result.referenceIntake : result.movedIntake).push_back(intake.getSample(0, sample));
            }
        }
    }
    (void)rpm;
    return result;
}


[[nodiscard]] std::optional<std::array<AcousticPoint3M, 2>> authoredMicrophones(const EngineConfig& config,
                                                                                double scale = 1.0) {
    return std::array { scaled(effectiveMicrophonePosition(config.acousticObserver, false), scale),
                        scaled(effectiveMicrophonePosition(config.acousticObserver, true), scale) };
}

/** The camera microphone, heard through the whole renderer. */
void rendererFollowsTheListener() {
    const auto cp2 = catalogueEngine("Yamaha CP2");
    // At the authored positions, or switched off: the default sound exactly.
    const auto same = listen(cp2, 4.0, [&](RealtimeEngineAudio& audio, double t) {
        if (t == 0.0) audio.setMicrophones(authoredMicrophones(cp2));
        if (std::abs(t - 2.5) < 1.0e-9) audio.setMicrophones(std::nullopt);
    });
    require(same.referenceMaster == same.movedMaster,
            "microphones at the authored positions leave the sound bit-identical");

    // Twice as far from 3 s, back from 6 s.
    const auto farther = listen(cp2, 10.0, [&](RealtimeEngineAudio& audio, double t) {
        if (std::abs(t - 3.0) < 1.0e-9) audio.setMicrophones(authoredMicrophones(cp2, 2.0));
        if (std::abs(t - 6.0) < 1.0e-9) audio.setMicrophones(std::nullopt);
    });
    const auto exhaustDb = decibels(rms(farther.movedExhaust, at(4.5), at(6.0))
                                    / rms(farther.referenceExhaust, at(4.5), at(6.0)));
    const auto intakeDb = decibels(rms(farther.movedIntake, at(4.5), at(6.0))
                                   / rms(farther.referenceIntake, at(4.5), at(6.0)));
    const auto stepBefore = largestStep(farther.movedExhaust, at(2.5), at(3.0));
    const auto stepDuring = largestStep(farther.movedExhaust, at(3.0), at(4.0));
    // The glide lands on the authored place exactly: the last sample that
    // still differs from the default sound.
    std::size_t lastDifferent = 0;
    for (auto index = at(6.0); index < at(10.0); ++index)
        if (farther.movedMaster[index] != farther.referenceMaster[index]) lastDifferent = index;
    const auto backAtSeconds = static_cast<double>(lastDifferent) / sampleRate;
    std::cout << "  CP2 heard from twice as far: exhaust " << exhaustDb << " dB, intake " << intakeDb
              << " dB; exhaust sample step " << stepDuring << " in the first second, " << stepBefore
              << " before; switched off at 6 s, the default sound bit for bit from " << backAtSeconds << " s\n";
    require(exhaustDb < -4.5 && exhaustDb > -7.5, "the exhaust is about 6 dB quieter twice as far");
    require(intakeDb < -1.0, "the intake is quieter twice as far");
    require(stepDuring <= 1.25 * stepBefore, "moving the microphones does not click");
    require(lastDifferent > at(6.0) && backAtSeconds < 8.5,
            "switched off, the default sound comes back bit for bit within 2.5 s");

    // A live exhaust or intake change while the microphone is away: the
    // incoming networks are heard from where the microphone is.
    const auto graph = ExhaustGraph::makeForEngine(cp2);
    bool exhaustSwapped = false;
    bool intakeSwapped = false;
    std::array<std::uint64_t, 2> swaps {};
    const auto swapped = listen(cp2, 7.0, [&](RealtimeEngineAudio& audio, double t) {
        if (std::abs(t - 3.0) < 1.0e-9) audio.setMicrophones(authoredMicrophones(cp2, 2.0));
        if (std::abs(t - 4.5) < 1.0e-9) {
            exhaustSwapped = audio.replaceExhaustGraph(graph);
            intakeSwapped = audio.replaceIntake(cp2);
        }
        swaps = { audio.exhaustSwapCount(), audio.intakeSwapCount() };
    });
    require(exhaustSwapped && intakeSwapped, "the renderer takes the live exhaust and intake");
    require(swaps == std::array<std::uint64_t, 2> { 1U, 1U }, "both networks were swapped");
    const auto swappedExhaustDb = decibels(rms(swapped.movedExhaust, at(5.5), at(7.0))
                                           / rms(swapped.referenceExhaust, at(5.5), at(7.0)));
    const auto swappedIntakeDb = decibels(rms(swapped.movedIntake, at(5.5), at(7.0))
                                          / rms(swapped.referenceIntake, at(5.5), at(7.0)));
    std::cout << "  CP2 exhaust and intake swapped with the microphones twice as far: exhaust " << swappedExhaustDb
              << " dB, intake " << swappedIntakeDb << " dB\n";
    require(swappedExhaustDb < -4.5 && swappedExhaustDb > -7.5, "a swapped-in exhaust is heard from the moved microphones");
    require(swappedIntakeDb < -4.5 && swappedIntakeDb > -7.5, "a swapped-in intake is heard from the moved microphones");

    // The turbo's distance follows the microphones' mean distance.
    const auto turbo = catalogueEngine("2JZ");
    const auto boost = listen(turbo, 7.0, [&](RealtimeEngineAudio& audio, double t) {
        if (std::abs(t - 3.0) < 1.0e-9) audio.setMicrophones(authoredMicrophones(turbo, 2.0));
    });
    const auto turboDb = decibels(rms(boost.movedTurbo, at(5.5), at(7.0)) / rms(boost.referenceTurbo, at(5.5), at(7.0)));
    std::cout << "  2JZ turbo heard from twice as far: " << turboDb << " dB\n";
    require(rms(boost.referenceTurbo, at(5.5), at(7.0)) > 0.0, "the 2JZ's turbo is heard (else this proves nothing)");
    require(std::abs(turboDb + 6.02) < 0.3, "the turbo is 6 dB quieter twice as far");
}

/** Every opening of `config` placed `metres` from the authored microphones'
 * centre, towards the engine and facing them; the turbo at `turboScale` of
 * that centre. */
[[nodiscard]] AcousticSourcePlacements placedBeforeTheListener(const EngineConfig& config, double metres,
                                                                double turboScale) {
    const auto left = effectiveMicrophonePosition(config.acousticObserver, false);
    const auto right = effectiveMicrophonePosition(config.acousticObserver, true);
    const AcousticPoint3M centre { 0.5 * (left.x + right.x), 0.5 * (left.y + right.y), 0.5 * (left.z + right.z) };
    const auto distance = std::hypot(centre.x, centre.y, centre.z);
    AcousticSourcePlacement placement;
    placement.positionM = scaled(centre, 1.0 - metres / distance);
    placement.axis = scaled(centre, 1.0 / distance);
    AcousticSourcePlacements result;
    const auto graph = ExhaustGraph::makeForEngine(config);
    for (const auto& node : graph.nodes()) {
        if (node.type != ExhaustNodeType::outlet) continue;
        placement.pathIndex = node.pathIndex;
        placement.componentId = node.sourceComponentId;
        result.exhaustOutlets.push_back(placement);
    }
    placement.componentId = 0;
    for (std::uint32_t path = 0; path < std::max<std::size_t>(1U, config.intakePaths.size()); ++path) {
        placement.pathIndex = path;
        result.intakeMouths.push_back(placement);
    }
    result.forcedInductionM = scaled(centre, turboScale);
    return result;
}

/** The openings radiate from where the picture draws them, while the
 * microphones are moved; at the authored microphones, the default sound. */
void rendererHearsTheDrawnOpenings() {
    const auto cp2 = catalogueEngine("Yamaha CP2");
    const auto close = placedBeforeTheListener(cp2, 1.0, 0.5);
    // Placements alone, microphones not moved: the default sound exactly.
    const auto unmoved = listen(cp2, 4.0, [&](RealtimeEngineAudio& audio, double t) {
        if (t == 0.0) audio.setSoundSources(close);
    });
    require(unmoved.referenceMaster == unmoved.movedMaster,
            "placements leave the sound bit-identical while the microphones are not moved");

    // Microphones moved at 3 s, the openings placed 1 m before them a little
    // later (the engine edited while heard from the camera), switched off
    // at 6 s.
    const auto placed = listen(cp2, 10.0, [&](RealtimeEngineAudio& audio, double t) {
        if (std::abs(t - 3.0) < 1.0e-9) audio.setMicrophones(authoredMicrophones(cp2));
        if (std::abs(t - 3.2) < 1.0e-9) audio.setSoundSources(close);
        if (std::abs(t - 6.0) < 1.0e-9) audio.setMicrophones(std::nullopt);
    });
    const auto exhaustDb = decibels(rms(placed.movedExhaust, at(4.5), at(6.0))
                                    / rms(placed.referenceExhaust, at(4.5), at(6.0)));
    const auto intakeDb = decibels(rms(placed.movedIntake, at(4.5), at(6.0))
                                   / rms(placed.referenceIntake, at(4.5), at(6.0)));
    std::size_t lastDifferent = 0;
    for (auto index = at(6.0); index < at(10.0); ++index)
        if (placed.movedMaster[index] != placed.referenceMaster[index]) lastDifferent = index;
    const auto backAtSeconds = static_cast<double>(lastDifferent) / sampleRate;
    std::cout << "  CP2 openings 1 m before the microphones: exhaust " << exhaustDb << " dB, intake " << intakeDb
              << " dB; switched off at 6 s, the default sound bit for bit from " << backAtSeconds << " s\n";
    require(exhaustDb > 8.0, "an exhaust outlet before the microphones is louder");
    require(intakeDb > 8.0, "an intake mouth before the microphones is louder");
    require(lastDifferent > at(6.0) && backAtSeconds < 8.5,
            "switched off, the openings go back and the default sound returns bit for bit");

    // A live exhaust and intake change takes the placements too.
    const auto graph = ExhaustGraph::makeForEngine(cp2);
    std::array<std::uint64_t, 2> swaps {};
    const auto swapped = listen(cp2, 7.0, [&](RealtimeEngineAudio& audio, double t) {
        if (t == 0.0) audio.setSoundSources(close);
        if (std::abs(t - 3.0) < 1.0e-9) audio.setMicrophones(authoredMicrophones(cp2));
        if (std::abs(t - 4.5) < 1.0e-9) {
            require(audio.replaceExhaustGraph(graph) && audio.replaceIntake(cp2), "the renderer takes the live change");
        }
        swaps = { audio.exhaustSwapCount(), audio.intakeSwapCount() };
    });
    require(swaps == std::array<std::uint64_t, 2> { 1U, 1U }, "both networks were swapped");
    const auto swappedExhaustDb = decibels(rms(swapped.movedExhaust, at(5.5), at(7.0))
                                           / rms(swapped.referenceExhaust, at(5.5), at(7.0)));
    const auto swappedIntakeDb = decibels(rms(swapped.movedIntake, at(5.5), at(7.0))
                                          / rms(swapped.referenceIntake, at(5.5), at(7.0)));
    std::cout << "  CP2 swapped with its openings placed: exhaust " << swappedExhaustDb << " dB, intake "
              << swappedIntakeDb << " dB\n";
    require(std::abs(swappedExhaustDb - exhaustDb) < 1.5, "a swapped-in exhaust sounds from its placed outlets");
    require(std::abs(swappedIntakeDb - intakeDb) < 1.5, "a swapped-in intake sounds from its placed mouths");

    // The turbo is heard at the microphones' mean distance from its place.
    const auto turbo = catalogueEngine("2JZ");
    const auto turboPlaced = placedBeforeTheListener(turbo, 1.0, 0.5);
    const auto boost = listen(turbo, 7.0, [&](RealtimeEngineAudio& audio, double t) {
        if (t == 0.0) audio.setSoundSources(turboPlaced);
        if (std::abs(t - 3.0) < 1.0e-9) audio.setMicrophones(authoredMicrophones(turbo));
    });
    const auto left = effectiveMicrophonePosition(turbo.acousticObserver, false);
    const auto right = effectiveMicrophonePosition(turbo.acousticObserver, true);
    const auto& at2jz = *turboPlaced.forcedInductionM;
    const auto from = [](const AcousticPoint3M& a, const AcousticPoint3M& b) {
        return std::hypot(a.x - b.x, a.y - b.y, a.z - b.z);
    };
    const auto expectedDb = decibels(effectiveObserverDistanceM(turbo.acousticObserver)
                                     / (0.5 * (from(left, at2jz) + from(right, at2jz))));
    const auto turboDb = decibels(rms(boost.movedTurbo, at(5.5), at(7.0)) / rms(boost.referenceTurbo, at(5.5), at(7.0)));
    std::cout << "  2JZ turbo placed halfway: " << turboDb << " dB (1/r: " << expectedDb << " dB)\n";
    require(rms(boost.referenceTurbo, at(5.5), at(7.0)) > 0.0, "the 2JZ's turbo is heard (else this proves nothing)");
    require(expectedDb > 3.0 && std::abs(turboDb - expectedDb) < 0.3, "the turbo is heard from its place");
}

/** The block's radiation follows the listener's distance as 1/r. */
void structureFollowsTheListener() {
    const auto cp2 = catalogueEngine("Yamaha CP2");
    StructuralModalRadiator near(cp2);
    StructuralModalRadiator far(cp2);
    require(near.valid() && near.prepare(sampleRate) && far.prepare(sampleRate), "the CP2's block radiates");
    far.moveObserver(2.0 * effectiveObserverDistanceM(cp2.acousticObserver));
    StructuralExcitationSample excitation;
    excitation.cylinderCount = cp2.cylinders.size();
    std::vector<float> nearPressure;
    std::vector<float> farPressure;
    for (auto sample = 0; sample < 96'000; ++sample) {
        excitation.gasForceN[0] = sample % 960 < 24 ? 20'000.0F : 0.0F;
        nearPressure.push_back(near.process(excitation));
        farPressure.push_back(far.process(excitation));
    }
    const auto db = decibels(rms(farPressure, 72'000, 96'000) / rms(nearPressure, 72'000, 96'000));
    std::cout << "  CP2 block heard from twice as far: " << db << " dB\n";
    require(rms(nearPressure, 72'000, 96'000) > 0.0, "the block is heard (else this proves nothing)");
    require(std::abs(db + 6.02) < 0.1, "the block is 6 dB quieter twice as far");
}

} // namespace

void cameraMicrophoneRegression() {
    observerGlides();
    observerMovesItsSource();
    structureFollowsTheListener();
    rendererFollowsTheListener();
    rendererHearsTheDrawnOpenings();
}

} // namespace enginelab::tests
