// A live exhaust change: the engine keeps running while its exhaust is
// resized. The simulator swaps its gas network between two steps and the new
// one takes over the gas state; the audio feeds the incoming acoustic network
// silently, then fades to it. These tests drive the real simulator and the
// real renderer the way EngineRuntime does, deterministically.
#include <enginelab/audio/RealtimeEngineAudio.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustComponentResize.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/runtime/EngineRuntime.hpp>
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

/** One simulated engine, owned on the heap like the harnesses do. */
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

[[nodiscard]] EngineControls controlsAt(double timeSeconds, double throttle) {
    EngineControls controls;
    controls.ignitionEnabled = true;
    controls.starterEngaged = timeSeconds < 1.5;
    controls.throttle = timeSeconds < 2.0 ? 0.0 : throttle;
    return controls;
}

/** Full throttle held at 60 % of redline by a proportional brake, so the
 * settled brake torque is what the exhaust changes. */
[[nodiscard]] EngineControls heldAt(double timeSeconds, double rpm, const EngineConfig& config) {
    auto controls = controlsAt(timeSeconds, 1.0);
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

/** The engine with its first resizable exhaust pipe longer by `lengthMm` and
 * its diameter scaled by `diameterScale`, through the same function the 3-D
 * view's inspector uses. */
[[nodiscard]] EngineConfig resizedExhaust(EngineConfig config, double lengthMm,
                                          double diameterScale) {
    for (const auto& path : config.exhaustPaths) {
        if (!path.network) continue;
        for (const auto& component : path.network->components) {
            const auto size = exhaustComponentSize(config, path.id, component.id);
            if (!size || !size->lengthEditable
                || component.type != ExhaustComponentType::pipe) continue;
            const auto error = resizeExhaustComponent(config, path.id, component.id,
                size->lengthMm + lengthMm, size->diameterMm * diameterScale);
            require(error.empty(), "resize the first exhaust pipe: " + error);
            normaliseEngineConfig(config);
            return config;
        }
    }
    throw std::runtime_error("the engine has no resizable exhaust pipe");
}

void swapExhaust(Rig& rig, const EngineConfig& config) {
    auto network = rig.simulator->buildLiveExhaustNetwork(config);
    require(network != nullptr, "a resized exhaust builds a live network");
    require(rig.simulator->replaceExhaustNetwork(network, config),
            "the simulator takes the resized exhaust between two steps");
    require(network != nullptr, "the previous network comes back to the caller");
}

/** Swapping in the same exhaust changes nothing: the run continues bit for bit. */
void identicalSwapIsInvisible() {
    const auto config = catalogueEngine("Yamaha CP2");
    Rig reference(config);
    Rig swapped(config);
    for (auto frame = 0; frame < 4 * 240; ++frame) {
        const auto t = frame * frameSeconds;
        if (frame == 3 * 240) swapExhaust(swapped, config);
        const auto a = reference.simulator->step(frameSeconds, controlsAt(t, 0.25));
        const auto b = swapped.simulator->step(frameSeconds, controlsAt(t, 0.25));
        require(a.state.rpm == b.state.rpm
                && a.state.crankAngleDegrees == b.state.crankAngleDegrees
                && a.state.exhaustBackPressureKpa == b.state.exhaustBackPressureKpa,
            "swapping in the same exhaust must leave the run bit-identical");
    }
}

/** A resized exhaust swapped in while the engine runs: no stall, no step,
 * and the engine settles where an engine built with that exhaust runs. */
void resizedSwapSettlesLikeARestart() {
    for (const auto* name : { "Yamaha CP2", "K20A", "LS3" }) {
        const auto original = catalogueEngine(name);
        const auto resized = resizedExhaust(original, 500.0, 0.70);
        Rig built(resized);
        Rig swapped(original);
        Rig untouched(original);
        constexpr auto swapFrame = 6 * 240;
        constexpr auto frames = 14 * 240;
        double largestStepBefore = 0.0;
        double stepAtSwap = 0.0;
        double minimumRpm = 1.0e9;
        double previousRpm = 0.0;
        std::array<double, 3> rpm {};
        std::vector<double> builtTorque;
        std::vector<double> swappedTorque;
        std::vector<double> untouchedTorque;
        // Mean exhaust wall temperature: the slowest state the network carries
        // (tens of seconds), the one a restart would lose.
        const auto meanWallTemperatureK = [](Rig& rig) {
            rig.simulator->captureGasFieldNow();
            double sum = 0.0;
            std::size_t count = 0;
            for (const auto& element : rig.simulator->gasField().elements) {
                if (element.kind != GasFieldElementKind::exhaustDuct) continue;
                for (std::size_t sample = 0; sample < element.sampleCount; ++sample) {
                    sum += element.wallTemperatureK[sample];
                    ++count;
                }
            }
            return sum / static_cast<double>(std::max<std::size_t>(1, count));
        };
        double wallBeforeK = 0.0;
        double wallAfterK = 0.0;
        for (auto frame = 0; frame < frames; ++frame) {
            const auto t = frame * frameSeconds;
            if (frame == swapFrame) {
                wallBeforeK = meanWallTemperatureK(swapped);
                swapExhaust(swapped, resized);
                wallAfterK = meanWallTemperatureK(swapped);
            }
            const auto run = [&](Rig& rig, double& speed, std::vector<double>& torque) {
                const auto controls = heldAt(t, speed, original);
                speed = rig.simulator->step(frameSeconds, controls).state.rpm;
                torque.push_back(controls.dynamometerTorqueNm);
            };
            run(built, rpm[0], builtTorque);
            run(swapped, rpm[1], swappedTorque);
            run(untouched, rpm[2], untouchedTorque);
            if (frame >= swapFrame - 240 && frame < swapFrame)
                largestStepBefore = std::max(largestStepBefore, std::abs(rpm[1] - previousRpm));
            if (frame >= swapFrame && frame < swapFrame + 24)
                stepAtSwap = std::max(stepAtSwap, std::abs(rpm[1] - previousRpm));
            if (frame >= swapFrame) minimumRpm = std::min(minimumRpm, rpm[1]);
            previousRpm = rpm[1];
        }
        const auto meanOfLast = [](const std::vector<double>& values) {
            const auto count = static_cast<std::ptrdiff_t>(4 * 240);
            double sum = 0.0;
            for (auto it = values.end() - count; it != values.end(); ++it) sum += *it;
            return sum / static_cast<double>(count);
        };
        const auto builtMean = meanOfLast(builtTorque);
        const auto swappedMean = meanOfLast(swappedTorque);
        const auto untouchedMean = meanOfLast(untouchedTorque);
        std::cout << "  " << name << ": settled brake torque, Nm: built " << builtMean
                  << ", swapped " << swappedMean << ", untouched " << untouchedMean
                  << "; largest frame step before " << largestStepBefore
                  << " rpm, in the 0.1 s after the swap " << stepAtSwap
                  << " rpm; lowest rpm after " << minimumRpm
                  << "; mean exhaust wall " << wallBeforeK << " K before the swap, "
                  << wallAfterK << " K after\n";
        require(wallBeforeK > original.ambientTemperatureC + 273.15 + 20.0,
                std::string(name) + ": the exhaust walls are hot when the swap happens");
        require(std::abs(wallAfterK - wallBeforeK) < 0.03 * wallBeforeK,
                std::string(name) + ": the resized exhaust keeps the wall temperatures");
        require(minimumRpm > 0.5 * original.idleRpm,
                std::string(name) + ": the engine keeps running through the swap");
        require(stepAtSwap <= 1.5 * largestStepBefore,
                std::string(name) + ": the swap stays within the engine's own frame-to-frame steps");
        require(std::abs(swappedMean - builtMean) <= 0.25 * std::abs(untouchedMean - builtMean),
                std::string(name) + ": the swapped engine settles where the resized engine runs");
    }
}

/** Two renderers fed the same telemetry from one simulator; the second one
 * changes its exhaust at `swapSeconds` to `graph`. */
struct AudioPair final {
    std::vector<float> reference;
    std::vector<float> swapped;
    std::uint64_t swaps { 0 };
};

[[nodiscard]] AudioPair renderPair(const EngineConfig& config, const EngineConfig& swapTo,
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
        // Outlet turbulence is seeded noise: a new network starts its own
        // sequence, which no fade can make sample-identical.
        listener.renderer->setOutletJetNoiseEnabled(false);
        require(listener.renderer->compiledExhaustTopologyActive(),
                "the engine plays its compiled exhaust");
    }
    const auto swapGraph = ExhaustGraph::makeForEngine(swapTo);
    double heldRpm = 0.0;
    AudioPair result;
    juce::AudioBuffer<float> block(2, samplesPerFrame);
    const auto frames = static_cast<int>(durationSeconds / frameSeconds);
    for (auto frame = 0; frame < frames; ++frame) {
        const auto t = frame * frameSeconds;
        if (frame == static_cast<int>(swapSeconds / frameSeconds))
            require(listeners[1].renderer->replaceExhaustGraph(swapGraph),
                    "the renderer accepts the resized exhaust");
        const auto controls = heldAt(t, heldRpm, config);
        auto step = rig.simulator->step(frameSeconds, controls);
        heldRpm = step.state.rpm;
        const auto start = step.state.simulationTimeSeconds - frameSeconds;
        for (std::size_t index = 0; index < step.firingEventCount; ++index)
            for (auto& listener : listeners) (void)listener.events->tryPush(step.firingEvents[index]);
        CylinderPressureSample pressure;
        while (rig.simulator->tryPopCylinderPressureSample(pressure))
            for (auto& listener : listeners) (void)listener.pressure->tryPush(pressure);
        ExhaustAcousticSample acoustic;
        while (rig.simulator->tryPopExhaustAcousticSample(acoustic))
            for (auto& listener : listeners)
                (void)listener.runtime->exhaustAcousticSamples().tryPush(acoustic);
        (void)start;
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
            auto& out = index == 0 ? result.reference : result.swapped;
            for (int sample = 0; sample < samplesPerFrame; ++sample)
                out.push_back(block.getSample(0, sample));
        }
    }
    result.swaps = listeners[1].renderer->exhaustSwapCount();
    return result;
}

[[nodiscard]] double rms(const std::vector<float>& values, std::size_t begin, std::size_t end) {
    double sum = 0.0;
    for (auto index = begin; index < end; ++index) sum += double(values[index]) * values[index];
    return std::sqrt(sum / static_cast<double>(std::max<std::size_t>(1, end - begin)));
}

[[nodiscard]] double rmsDifference(const AudioPair& pair, std::size_t begin, std::size_t end) {
    double sum = 0.0;
    for (auto index = begin; index < end; ++index) {
        const double difference = pair.swapped[index] - pair.reference[index];
        sum += difference * difference;
    }
    return std::sqrt(sum / static_cast<double>(std::max<std::size_t>(1, end - begin)));
}

/** The same exhaust swapped into the renderer: the fade must not be heard. */
void identicalAudioSwapIsInaudible() {
    const auto config = catalogueEngine("Yamaha CP2");
    constexpr double swapSeconds = 4.0;
    const auto pair = renderPair(config, config, swapSeconds, 5.0);
    require(pair.swaps == 1, "the renderer completes the exhaust swap");
    const auto at = [](double seconds) { return static_cast<std::size_t>(seconds * 48'000.0); };
    require(rmsDifference(pair, 0, at(swapSeconds)) == 0.0,
            "two renderers fed the same telemetry play the same samples");
    const auto swapEnd = swapSeconds + RealtimeEngineAudio::exhaustSwapWarmupSeconds
        + RealtimeEngineAudio::exhaustSwapFadeSeconds;
    const auto signal = rms(pair.reference, at(swapSeconds), at(swapEnd + 0.1));
    const auto duringSwap = rmsDifference(pair, at(swapSeconds), at(swapEnd + 0.1));
    const auto after = rmsDifference(pair, at(swapEnd + 0.3), at(5.0));
    const auto decibels = [signal](double value) {
        return 20.0 * std::log10(std::max(value, 1.0e-12) / std::max(signal, 1.0e-12));
    };
    std::cout << "  identical exhaust swap: signal rms " << signal
              << ", difference over the swap " << decibels(duringSwap)
              << " dB, after it " << decibels(after) << " dB\n";
    require(signal > 1.0e-4, "the engine is audible when the swap happens");
    require(decibels(duringSwap) < -60.0,
            "the swap of an identical exhaust stays 60 dB below the engine");
    require(decibels(after) < -90.0,
            "after the swap the identical exhaust plays as before");
}

/** A resized exhaust swapped into the renderer: no click. */
void resizedAudioSwapHasNoClick() {
    const auto config = catalogueEngine("Yamaha CP2");
    constexpr double swapSeconds = 4.0;
    const auto pair = renderPair(config, resizedExhaust(config, 150.0, 1.07), swapSeconds, 5.0);
    require(pair.swaps == 1, "the renderer completes the resized exhaust swap");
    const auto at = [](double seconds) { return static_cast<std::size_t>(seconds * 48'000.0); };
    const auto largestJump = [&pair](std::size_t begin, std::size_t end) {
        double jump = 0.0;
        for (auto index = begin + 1; index < end; ++index)
            jump = std::max(jump, double(std::abs(pair.swapped[index] - pair.swapped[index - 1])));
        return jump;
    };
    const auto swapEnd = swapSeconds + RealtimeEngineAudio::exhaustSwapWarmupSeconds
        + RealtimeEngineAudio::exhaustSwapFadeSeconds;
    const auto before = largestJump(at(swapSeconds - 1.0), at(swapSeconds));
    const auto during = largestJump(at(swapSeconds), at(swapEnd + 0.05));
    const auto signal = rms(pair.reference, at(swapEnd + 0.1), at(5.0));
    const auto heard = rmsDifference(pair, at(swapEnd + 0.1), at(5.0));
    std::cout << "  resized exhaust swap: largest sample step before " << before
              << ", during " << during << "; the resized exhaust differs by "
              << 20.0 * std::log10(heard / signal) << " dB\n";
    require(during <= 1.25 * before, "the resized exhaust fades in without a click");
    require(heard > 0.03 * signal, "the resized exhaust is heard once the swap is over");
}

} // namespace

void liveChangeRegression() {
    identicalSwapIsInvisible();
    resizedSwapSettlesLikeARestart();
    identicalAudioSwapIsInaudible();
    resizedAudioSwapHasNoClick();
}

} // namespace enginelab::tests
