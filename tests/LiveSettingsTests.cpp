// A live settings change: the running engine takes new injectors, boost and
// turbine settings between two steps, and its turbo sound follows them, as
// long as the machine stays the same (injection mode, forced induction on or
// off, type, blade and lobe counts). These tests drive the real simulator and
// the real renderer the way EngineRuntime does, deterministically.
#include <enginelab/audio/RealtimeEngineAudio.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/runtime/EngineRuntime.hpp>
#include <enginelab/serialization/EngineEditScope.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>
#include <juce_audio_basics/juce_audio_basics.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace enginelab::tests {
namespace {

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

constexpr double frameSeconds = 1.0 / 240.0;

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

/** Cranked to 1.5 s, then full throttle from 2 s held at 60 % of redline by
 * a proportional brake: the settled brake torque is what a setting changes. */
[[nodiscard]] EngineControls heldAt(double timeSeconds, double rpm, const EngineConfig& config) {
    EngineControls controls;
    controls.ignitionEnabled = true;
    controls.starterEngaged = timeSeconds < 1.5;
    controls.throttle = timeSeconds < 2.0 ? 0.0 : 1.0;
    if (timeSeconds >= 2.0)
        controls.dynamometerTorqueNm = std::max(0.0, 2.0 * (rpm - 0.6 * config.redlineRpm));
    return controls;
}

[[nodiscard]] EngineConfig catalogueEngine(std::string_view selector) {
    static const auto catalogue = loadEngineCatalog(ENGINELAB_CATALOG_ROOT);
    const auto selected = selectSingleEngineCatalogEntry(catalogue.entries, selector);
    require(static_cast<bool>(selected), "catalogue engine " + std::string(selector));
    auto config = selected.entry->config;
    normaliseEngineConfig(config);
    return config;
}

/** The 2JZ turned down from ~0.9 bar to ~0.45 bar of boost. */
[[nodiscard]] EngineConfig lowerBoost(EngineConfig config) {
    config.forcedInduction.pressureRatio = 1.45;
    config.forcedInduction.wastegatePressureRatio = 1.42;
    return config;
}

/** The CP2 with injectors too small for full load: 120 g/min each
 * (measured: 46 Nm instead of 60 at 5,900 rpm). Its own 20 g/s are far from
 * the limit, so halving them changes nothing. */
[[nodiscard]] EngineConfig smallInjectors(EngineConfig config) {
    config.injection.injectorFlowMgPerSecond = 2'000.0;
    return config;
}

void swapSettings(Rig& rig, const EngineConfig& config) {
    auto incoming = config;
    require(rig.simulator->replaceSettings(incoming), "the simulator takes the settings between two steps");
}

/** Taking the settings it already runs with changes nothing, bit for bit. */
void identicalSettingsAreInvisible() {
    for (const auto* name : { "2JZ", "Yamaha CP2" }) {
        const auto config = catalogueEngine(name);
        Rig reference(config);
        Rig swapped(config);
        double rpm = 0.0;
        for (auto frame = 0; frame < 5 * 240; ++frame) {
            const auto t = frame * frameSeconds;
            if (frame == 3 * 240) swapSettings(swapped, config);
            const auto controls = heldAt(t, rpm, config);
            const auto a = reference.simulator->step(frameSeconds, controls);
            const auto b = swapped.simulator->step(frameSeconds, controls);
            rpm = a.state.rpm;
            require(a.state.rpm == b.state.rpm
                    && a.state.crankAngleDegrees == b.state.crankAngleDegrees
                    && a.state.boostPressureRatio == b.state.boostPressureRatio,
                std::string(name) + ": the same settings must leave the run bit-identical");
        }
    }
}

/** Another machine is refused, and the refusal changes nothing. */
void anotherMachineIsRefused() {
    const auto turbo = catalogueEngine("2JZ");
    std::vector<std::pair<std::string, EngineConfig>> refused;
    auto edited = turbo;
    edited.injection.mode = InjectionMode::direct;
    refused.emplace_back("another injection mode", edited);
    edited = turbo;
    edited.forcedInduction.enabled = false;
    refused.emplace_back("forced induction removed", edited);
    edited = turbo;
    edited.forcedInduction.type = ForcedInductionType::supercharger;
    refused.emplace_back("a supercharger instead", edited);
    edited = turbo;
    edited.forcedInduction.compressorBladeCount += 1;
    refused.emplace_back("another compressor blade count", edited);
    edited = turbo;
    edited.forcedInduction.turbineBladeCount += 1;
    refused.emplace_back("another turbine blade count", edited);
    edited = turbo;
    edited.forcedInduction.superchargerLobeCount += 1;
    refused.emplace_back("another lobe count", edited);
    for (auto& [what, config] : refused) {
        require(!EngineSimulator::settingsReplaceable(turbo, config), what + ": refused");
        Rig reference(turbo);
        Rig touched(turbo);
        double rpm = 0.0;
        for (auto frame = 0; frame < 3 * 240; ++frame) {
            const auto t = frame * frameSeconds;
            if (frame == 2 * 240) {
                auto incoming = config;
                require(!touched.simulator->replaceSettings(incoming), what + ": the simulator refuses it");
            }
            const auto controls = heldAt(t, rpm, turbo);
            const auto a = reference.simulator->step(frameSeconds, controls);
            const auto b = touched.simulator->step(frameSeconds, controls);
            rpm = a.state.rpm;
            require(a.state.rpm == b.state.rpm && a.state.boostPressureRatio == b.state.boostPressureRatio,
                    what + ": a refused change leaves the run as it was");
        }
    }
}

/** New settings taken while the engine runs: no stall, and the engine
 * settles where an engine started with them runs. The boost is what a
 * wastegate setting moves: the 2JZ's torque hardly follows it (measured,
 * 1.69 against 1.94 boost ratio for 1 % of torque). */
void changedSettingsSettleLikeARestart() {
    struct Case final {
        const char* name;
        EngineConfig (*edit)(EngineConfig);
        const char* what;
        bool boost;
    };
    for (const auto& [name, edit, what, boost] : { Case { "2JZ", lowerBoost, "lower boost", true },
                                                    Case { "Yamaha CP2", smallInjectors, "small injectors", false } }) {
        const auto original = catalogueEngine(name);
        const auto edited = edit(original);
        require(EngineSimulator::settingsReplaceable(original, edited), std::string(what) + ": replaceable");
        Rig built(edited);
        Rig swapped(original);
        Rig untouched(original);
        constexpr auto swapFrame = 6 * 240;
        constexpr auto frames = 14 * 240;
        std::array<double, 3> rpm {};
        std::array<std::vector<double>, 3> observed;
        double minimumRpm = 1.0e9;
        for (auto frame = 0; frame < frames; ++frame) {
            const auto t = frame * frameSeconds;
            if (frame == swapFrame) swapSettings(swapped, edited);
            std::size_t index = 0;
            for (auto* rig : { &built, &swapped, &untouched }) {
                const auto controls = heldAt(t, rpm[index], original);
                const auto& state = rig->simulator->step(frameSeconds, controls).state;
                rpm[index] = state.rpm;
                observed[index].push_back(boost ? state.boostPressureRatio : controls.dynamometerTorqueNm);
                ++index;
            }
            if (frame >= swapFrame) minimumRpm = std::min(minimumRpm, rpm[1]);
        }
        const auto meanOfLast = [](const std::vector<double>& values) {
            const auto count = static_cast<std::ptrdiff_t>(4 * 240);
            double sum = 0.0;
            for (auto it = values.end() - count; it != values.end(); ++it) sum += *it;
            return sum / static_cast<double>(count);
        };
        const auto builtMean = meanOfLast(observed[0]);
        const auto swappedMean = meanOfLast(observed[1]);
        const auto untouchedMean = meanOfLast(observed[2]);
        std::cout << "  " << name << ", " << what << (boost ? ": boost ratio" : ": settled brake torque, Nm")
                  << ": built " << builtMean
                  << ", swapped " << swappedMean << ", untouched " << untouchedMean
                  << "; lowest rpm after the change " << minimumRpm << "\n";
        require(std::abs(untouchedMean - builtMean) > 0.05 * std::abs(untouchedMean),
                std::string(name) + ": the setting changes what is observed (else this proves nothing)");
        require(minimumRpm > 0.5 * original.idleRpm,
                std::string(name) + ": the engine keeps running through the change");
        require(std::abs(swappedMean - builtMean) <= 0.10 * std::abs(untouchedMean - builtMean),
                std::string(name) + ": the changed engine settles where an engine built with the setting runs");
    }
}

/** The runtime posts what the simulator can take and refuses the rest. */
void runtimeTakesSettings() {
    const auto turbo = catalogueEngine("2JZ");
    auto runtime = std::make_unique<EngineRuntime>(turbo);
    auto edited = lowerBoost(turbo);
    require(runtime->applyLiveSettings(edited), "the runtime takes a lower boost");
    require(runtime->engineConfig().forcedInduction.pressureRatio == 1.45,
            "the runtime's configuration carries the new boost");
    edited.forcedInduction.compressorBladeCount += 1;
    require(!runtime->applyLiveSettings(edited), "the runtime refuses another compressor");
    edited = turbo;
    edited.injection.injectorFlowMgPerSecond = -1.0;
    require(!runtime->applyLiveSettings(edited), "the runtime refuses an invalid injector");
    require(runtime->engineConfig().forcedInduction.compressorBladeCount
                == turbo.forcedInduction.compressorBladeCount
            && runtime->engineConfig().injection.injectorFlowMgPerSecond
                == turbo.injection.injectorFlowMgPerSecond,
            "a refused change leaves the runtime's configuration as it was");
}

/** engineEditScope sorts an edit into what a running engine can take. */
void editScopeSortsEdits() {
    const auto turbo = catalogueEngine("2JZ");
    require(!engineEditScope(turbo, turbo).any(), "an unchanged engine is no edit");
    const auto settings = engineEditScope(turbo, lowerBoost(turbo));
    require(settings.settings && !settings.exhaust && !settings.cylinders && !settings.other,
            "a boost edit is a settings edit only");
    auto injector = turbo;
    injector.injection.injectorFlowMgPerSecond *= 1.2;
    const auto injectorScope = engineEditScope(turbo, injector);
    require(injectorScope.settings && !injectorScope.other, "an injector edit is a settings edit");
    auto exhaust = turbo;
    exhaust.exhaust.primaryLengthMm += 40.0;
    const auto exhaustScope = engineEditScope(turbo, exhaust);
    require(exhaustScope.exhaust && !exhaustScope.settings && !exhaustScope.cylinders
                && !exhaustScope.other,
            "an exhaust edit is an exhaust edit only");
    auto bored = turbo;
    for (auto& cylinder : bored.cylinders) cylinder.boreMm -= 1.0;
    const auto boredScope = engineEditScope(turbo, bored);
    require(boredScope.cylinders && !boredScope.settings && !boredScope.exhaust && !boredScope.other,
            "a bore edit is a cylinder edit");
    auto mixed = lowerBoost(bored);
    const auto mixedScope = engineEditScope(turbo, mixed);
    require(mixedScope.cylinders && mixedScope.settings && !mixedScope.other,
            "a bore and boost edit is both, nothing else");
    auto other = turbo;
    other.redlineRpm -= 200.0;
    require(engineEditScope(turbo, other).other, "a redline edit needs a restart");
    auto fewer = turbo;
    fewer.cylinders.pop_back();
    require(engineEditScope(turbo, fewer).other, "a cylinder removed needs a restart");
}

/** Two renderers fed the same telemetry from one simulator; the second one
 * takes `fi` at `swapSeconds`. */
struct AudioPair final {
    std::vector<float> reference;
    std::vector<float> updated;
    std::uint64_t updates { 0 };
};

[[nodiscard]] AudioPair renderPair(const EngineConfig& config, const ForcedInductionConfig& fi,
                                   double swapSeconds, double durationSeconds) {
    Rig rig(config);
    constexpr double sampleRate = 48'000.0;
    constexpr int samplesPerFrame = 200;
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
        require(listener.renderer->forcedInductionAcousticsActive(), "the engine plays its turbo");
    }
    double heldRpm = 0.0;
    AudioPair result;
    juce::AudioBuffer<float> block(2, samplesPerFrame);
    const auto frames = static_cast<int>(durationSeconds / frameSeconds);
    for (auto frame = 0; frame < frames; ++frame) {
        const auto t = frame * frameSeconds;
        if (frame == static_cast<int>(swapSeconds / frameSeconds))
            require(listeners[1].renderer->replaceForcedInduction(fi),
                    "the renderer accepts settings of the same turbo");
        const auto controls = heldAt(t, heldRpm, config);
        auto step = rig.simulator->step(frameSeconds, controls);
        heldRpm = step.state.rpm;
        for (std::size_t index = 0; index < step.firingEventCount; ++index)
            for (auto& listener : listeners) (void)listener.events->tryPush(step.firingEvents[index]);
        CylinderPressureSample pressure;
        while (rig.simulator->tryPopCylinderPressureSample(pressure))
            for (auto& listener : listeners) (void)listener.pressure->tryPush(pressure);
        ExhaustAcousticSample acoustic;
        while (rig.simulator->tryPopExhaustAcousticSample(acoustic))
            for (auto& listener : listeners)
                (void)listener.runtime->exhaustAcousticSamples().tryPush(acoustic);
        for (std::size_t index = 0; index < listeners.size(); ++index) {
            auto& listener = listeners[index];
            publishAudioFrame(listener.runtime->audioState(), step.state,
                              { false, controls.starterEngaged, 0.0, 1.0 });
            listener.runtime->audioState().producerTimeNanoseconds.store(
                static_cast<std::uint64_t>((t + frameSeconds) * 1.0e9),
                std::memory_order_release);
            block.clear();
            listener.renderer->render(block, 0, samplesPerFrame);
            listener.renderer->collectRetiredExhaustNetworks();
            auto& out = index == 0 ? result.reference : result.updated;
            for (int sample = 0; sample < samplesPerFrame; ++sample)
                out.push_back(block.getSample(0, sample));
        }
    }
    result.updates = listeners[1].renderer->forcedInductionUpdateCount();
    return result;
}

/** The turbo sound alone: rendered once with the default settings and once
 * with the edited ones, both through the same layer. */
void turboSoundFollowsSettings() {
    const auto config = catalogueEngine("2JZ");
    constexpr double swapSeconds = 3.0;
    constexpr double duration = 4.0;
    const auto at = [](double seconds) { return static_cast<std::size_t>(seconds * 48'000.0); };
    const auto difference = [](const AudioPair& pair, std::size_t begin, std::size_t end) {
        double sum = 0.0;
        double signal = 0.0;
        for (auto index = begin; index < end; ++index) {
            const double d = pair.updated[index] - pair.reference[index];
            sum += d * d;
            signal += double(pair.reference[index]) * pair.reference[index];
        }
        return std::pair { std::sqrt(sum / double(end - begin)), std::sqrt(signal / double(end - begin)) };
    };
    const auto same = renderPair(config, config.forcedInduction, swapSeconds, duration);
    require(same.updates == 1, "the renderer takes the same settings");
    require(difference(same, 0, same.reference.size()).first == 0.0,
            "the same turbo settings play the same samples");

    auto louder = config.forcedInduction;
    louder.compressorInducerDiameterMm *= 1.5;
    louder.tonalAcousticEfficiency *= 10.0;
    const auto edited = renderPair(config, louder, swapSeconds, duration);
    require(edited.updates == 1, "the renderer takes the edited settings");
    const auto [before, signalBefore] = difference(edited, 0, at(swapSeconds));
    const auto [after, signal] = difference(edited, at(swapSeconds + 0.05), at(duration));
    std::cout << "  2JZ turbo settings taken live: difference before " << before
              << ", after " << 20.0 * std::log10(std::max(after, 1.0e-12) / signal) << " dB of the engine\n";
    (void)signalBefore;
    require(before == 0.0, "nothing changes before the settings are taken");
    require(after > 1.0e-3 * signal, "the edited turbo is heard once taken");

    // Another machine, or a sound that would start or stop, needs a new renderer.
    auto events = std::make_unique<FiringEventQueue>();
    auto pressure = std::make_unique<CylinderPressureQueue>();
    auto runtime = std::make_unique<EngineRuntime>(config);
    auto renderer = std::make_unique<RealtimeEngineAudio>(*events, runtime->audioState(), pressure.get(),
        &runtime->exhaustGraph(), &runtime->engineConfig(), &runtime->exhaustAcousticSamples());
    renderer->prepare(48'000.0, 200);
    auto other = config.forcedInduction;
    other.turbineBladeCount += 1;
    require(!renderer->replaceForcedInduction(other), "another turbine needs a new renderer");
    other = config.forcedInduction;
    other.enabled = false;
    require(!renderer->replaceForcedInduction(other), "a turbo removed needs a new renderer");
    // The layer itself refuses too, whoever asks it.
    ForcedInductionAcoustics layer(config.forcedInduction, 4.0);
    other = config.forcedInduction;
    other.compressorBladeCount += 1;
    require(!layer.updateConfig(other), "the turbo layer refuses another compressor");
    other = config.forcedInduction;
    other.wastegatePressureRatio -= 0.1;
    require(layer.updateConfig(other), "the turbo layer takes a new wastegate");

    const auto quiet = catalogueEngine("Yamaha CP2");
    auto quietRuntime = std::make_unique<EngineRuntime>(quiet);
    auto quietRenderer = std::make_unique<RealtimeEngineAudio>(*events, quietRuntime->audioState(),
        pressure.get(), &quietRuntime->exhaustGraph(), &quietRuntime->engineConfig(),
        &quietRuntime->exhaustAcousticSamples());
    quietRenderer->prepare(48'000.0, 200);
    require(quietRenderer->replaceForcedInduction(quiet.forcedInduction),
            "an engine without a turbo takes settings that make no turbo sound");
    require(!quietRenderer->replaceForcedInduction(config.forcedInduction),
            "a turbo added needs a new renderer");
}

} // namespace

void liveSettingsRegression() {
    editScopeSortsEdits();
    identicalSettingsAreInvisible();
    anotherMachineIsRefused();
    changedSettingsSettleLikeARestart();
    runtimeTakesSettings();
    turboSoundFollowsSettings();
}

} // namespace enginelab::tests
