// A live exhaust change: the engine keeps running while its exhaust is
// resized. The simulator swaps its gas network between two steps and the new
// one takes over the gas state; the audio feeds the incoming acoustic network
// silently, then fades to it. A live cylinder resize: each crank journal takes
// the new bore and stroke at its gas-exchange TDC. These tests drive the real
// simulator and the real renderer the way EngineRuntime does,
// deterministically.
#include <enginelab/audio/RealtimeEngineAudio.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustComponentResize.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/foundation/CylinderResize.hpp>
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

/** The engine with every cylinder `boreMm` and `strokeMm` larger, through the
 * same function the 3-D view's inspector uses. */
[[nodiscard]] EngineConfig resizedCylinders(EngineConfig config, double boreMm, double strokeMm) {
    const auto size = cylinderSize(config);
    require(size.has_value(), config.name + ": the cylinders share one size");
    const auto error = resizeCylinders(config, size->boreMm + boreMm, size->strokeMm + strokeMm);
    require(error.empty(), config.name + ": " + error);
    normaliseEngineConfig(config);
    const auto invalid = validateEngineConfig(config);
    require(!invalid, config.name + ": " + invalid.value_or(""));
    return config;
}

/** Whether the phase passed `target` going from `previous` to `current`. */
[[nodiscard]] bool crossed(double previous, double current, double target) {
    const auto travel = std::fmod(current - previous + 720.0, 720.0);
    return std::fmod(target - previous + 720.0, 720.0) <= travel;
}

/** Resizing to the running size changes nothing, at once or over a ramp. */
void identicalResizeIsInvisible() {
    const auto config = catalogueEngine("Yamaha CP2");
    for (const auto rampSeconds : { 0.0, 0.5 }) {
        Rig reference(config);
        Rig resized(config);
        for (auto frame = 0; frame < 4 * 240; ++frame) {
            const auto t = frame * frameSeconds;
            if (frame == 3 * 240)
                require(resized.simulator->beginCylinderResize(config, rampSeconds),
                        "the simulator takes a resize to its own size");
            const auto a = reference.simulator->step(frameSeconds, controlsAt(t, 0.25));
            const auto b = resized.simulator->step(frameSeconds, controlsAt(t, 0.25));
            require(a.state.rpm == b.state.rpm
                    && a.state.crankAngleDegrees == b.state.crankAngleDegrees
                    && a.state.cycleAveragedTorqueNm == b.state.cycleAveragedTorqueNm
                    && a.state.exhaustBackPressureKpa == b.state.exhaustBackPressureKpa,
                "resizing to the same size must leave the run bit-identical");
        }
        // Otherwise the comparison above proves nothing.
        require(resized.simulator->cylinderGeometryRevision() > 0 && !resized.simulator->cylinderResizeActive(),
                "the identical resize was applied to every journal");
    }
}

/** Each crank journal changes at the gas-exchange TDC of its cylinder, one
 * after the other, never all at once. */
void resizeHappensAtGasExchangeTdc() {
    const auto original = catalogueEngine("K20A");
    const auto target = resizedCylinders(original, 3.0, 0.0);
    Rig rig(original);
    const auto count = original.cylinders.size();
    std::vector<std::array<double, 2>> phases(count, { 0.0, 0.0 });
    std::vector<int> changedAtFrame(count, -1);
    for (auto frame = 0; frame < 4 * 240; ++frame) {
        const auto t = frame * frameSeconds;
        if (frame == 3 * 240) require(rig.simulator->beginCylinderResize(target, 0.0), "the K20A takes a bore change");
        const auto result = rig.simulator->step(frameSeconds, controlsAt(t, 0.25));
        for (std::size_t index = 0; index < count; ++index) {
            const auto phase = result.state.cylinderStates[index].cyclePhaseDegrees;
            if (changedAtFrame[index] < 0
                && rig.simulator->config().cylinders[index].boreMm == target.cylinders[index].boreMm) {
                changedAtFrame[index] = frame;
                // The resize applies at the start of the sub-step after the
                // crossing, which may fall in the next frame.
                require(crossed(phases[index][0], phase, 360.0),
                        "cylinder " + std::to_string(index + 1) + " changes at its gas-exchange TDC (phase "
                            + std::to_string(phases[index][0]) + " to " + std::to_string(phase) + ")");
            }
            phases[index] = { phases[index][1], phase };
        }
    }
    std::vector<int> frames(changedAtFrame);
    std::sort(frames.begin(), frames.end());
    require(frames.front() >= 3 * 240, "every cylinder changed after the request");
    require(std::unique(frames.begin(), frames.end()) - frames.begin() >= 3,
            "the cylinders change one after the other, at their own TDC");
}

/** A bore and stroke change while the engine runs, at once or over 2 s:
 * no stall, no overshoot beyond the cycle-to-cycle variability, the brake
 * torque of an engine built that way, and the original engine's again once
 * resized back. */
void resizeSettlesLikeARestart() {
    for (const auto* name : { "Yamaha CP2", "K20A", "LS3" }) {
        const auto original = catalogueEngine(name);
        const auto resized = resizedCylinders(original, 3.0, 3.0);
        Rig built(resized);
        Rig instant(original);
        Rig ramped(original);
        Rig back(original);
        Rig untouched(original);
        constexpr auto swapFrame = 5 * 240;
        constexpr auto backFrame = 8 * 240;
        constexpr auto frames = 12 * 240;
        constexpr double rampSeconds = 2.0;
        std::array<Rig*, 5> rigs { &built, &instant, &ramped, &back, &untouched };
        std::array<double, 5> rpm {};
        std::array<std::vector<double>, 5> brake;
        std::vector<double> instantTorque;
        std::vector<double> rampedTorque;
        double minimumRpm = 1.0e9;
        auto boreMidRamp = 0.0;
        for (auto frame = 0; frame < frames; ++frame) {
            const auto t = frame * frameSeconds;
            if (frame == swapFrame) {
                require(instant.simulator->beginCylinderResize(resized, 0.0), std::string(name) + ": instant resize");
                require(ramped.simulator->beginCylinderResize(resized, rampSeconds), std::string(name) + ": ramped resize");
                require(back.simulator->beginCylinderResize(resized, 0.0), std::string(name) + ": resize");
            }
            if (frame == backFrame)
                require(back.simulator->beginCylinderResize(original, 0.0), std::string(name) + ": resize back");
            if (frame == swapFrame + static_cast<int>(rampSeconds * 120.0))
                boreMidRamp = ramped.simulator->config().cylinders.front().boreMm;
            for (std::size_t index = 0; index < rigs.size(); ++index) {
                const auto controls = heldAt(t, rpm[index], original);
                const auto result = rigs[index]->simulator->step(frameSeconds, controls);
                rpm[index] = result.state.rpm;
                brake[index].push_back(controls.dynamometerTorqueNm);
                if (index == 1) instantTorque.push_back(result.state.cycleAveragedTorqueNm);
                if (index == 2) rampedTorque.push_back(result.state.cycleAveragedTorqueNm);
                if (frame >= swapFrame && index >= 1 && index <= 3) minimumRpm = std::min(minimumRpm, rpm[index]);
            }
        }
        const auto mean = [](const std::vector<double>& values, int from, int to) {
            double sum = 0.0;
            for (auto frame = from; frame < to; ++frame) sum += values[static_cast<std::size_t>(frame)];
            return sum / static_cast<double>(to - from);
        };
        const auto deviation = [&mean](const std::vector<double>& values, int from, int to) {
            const auto average = mean(values, from, to);
            double sum = 0.0;
            for (auto frame = from; frame < to; ++frame)
                sum += std::pow(values[static_cast<std::size_t>(frame)] - average, 2.0);
            return std::sqrt(sum / static_cast<double>(to - from));
        };
        constexpr auto settled = frames - 3 * 240;
        const auto builtMean = mean(brake[0], settled, frames);
        const auto instantMean = mean(brake[1], settled, frames);
        const auto rampedMean = mean(brake[2], settled, frames);
        const auto backMean = mean(brake[3], settled, frames);
        const auto untouchedMean = mean(brake[4], settled, frames);
        const auto beforeTorque = mean(instantTorque, swapFrame - 2 * 240, swapFrame);
        const auto afterTorque = mean(instantTorque, settled, frames);
        const auto spread = std::max(deviation(instantTorque, swapFrame - 2 * 240, swapFrame),
                                     deviation(instantTorque, settled, frames));
        // The cycle torque, one value per cycle, against the band its
        // cycle-to-cycle variability allows between the level before and the
        // levels after (the one just after the change, walls still at the old
        // temperature, and the settled one). Counts the cycles outside it.
        const auto rampFrames = static_cast<int>(rampSeconds * 240.0);
        const auto cyclesOutside = [&](const std::vector<double>& torque, int settleFrom, int to) {
            const auto early = mean(torque, settleFrom + 24, settleFrom + 120);
            const auto low = std::min({ beforeTorque, afterTorque, early }) - 4.0 * spread;
            const auto high = std::max({ beforeTorque, afterTorque, early }) + 4.0 * spread;
            auto outside = 0;
            for (auto frame = swapFrame; frame < to; ++frame) {
                const auto value = torque[static_cast<std::size_t>(frame)];
                if (value == torque[static_cast<std::size_t>(frame - 1)]) continue;
                if (value < low || value > high) ++outside;
            }
            return outside;
        };
        const auto instantOutside = cyclesOutside(instantTorque, swapFrame, swapFrame + 240);
        const auto rampedOutside = cyclesOutside(rampedTorque, swapFrame + rampFrames, swapFrame + rampFrames + 240);
        const auto from = original.cylinders.front().boreMm;
        const auto to = resized.cylinders.front().boreMm;
        std::cout << "  " << name << ": settled brake torque, Nm: built " << builtMean << ", instant "
                  << instantMean << ", ramped " << rampedMean << ", resized and back " << backMean
                  << ", untouched " << untouchedMean << "; cycle torque " << beforeTorque << " -> "
                  << afterTorque << " (spread " << spread << "), cycles outside it: " << instantOutside
                  << " at once, " << rampedOutside << " over the ramp; bore " << boreMidRamp << " mm half-way through the ramp ("
                  << from << " -> " << to << "); lowest rpm " << minimumRpm << "\n";
        require(minimumRpm > 0.5 * original.idleRpm, std::string(name) + ": the resized engine keeps running");
        const auto effect = std::abs(untouchedMean - builtMean);
        require(effect > 0.01 * builtMean, std::string(name) + ": the resize changes the brake torque");
        require(std::abs(instantMean - builtMean) <= 0.25 * effect,
                std::string(name) + ": resized at once, the engine settles like one built that way");
        require(std::abs(rampedMean - builtMean) <= 0.25 * effect,
                std::string(name) + ": resized over a ramp, the engine settles like one built that way");
        require(std::abs(backMean - untouchedMean) <= 0.25 * effect,
                std::string(name) + ": resized and back, the engine settles where it was");
        // At once, the first cycle burns fuel metered for the old cylinder.
        require(instantOutside <= 1,
                std::string(name) + ": resized at once, the cycle torque steps to its new level");
        require(rampedOutside == 0,
                std::string(name) + ": resized over a ramp, the cycle torque stays within its variability");
        require(boreMidRamp > from + 0.2 * (to - from) && boreMidRamp < from + 0.8 * (to - from),
                std::string(name) + ": half-way through the ramp the bore is half-way");
    }
}

/** Every catalogue engine takes a bore and stroke change at idle and keeps
 * idling. */
void resizeAtIdleKeepsEveryEngineRunning() {
    static const auto catalogue = loadEngineCatalog(ENGINELAB_CATALOG_ROOT);
    std::size_t engines = 0;
    for (const auto& entry : catalogue.entries) {
        auto original = entry.config;
        normaliseEngineConfig(original);
        // Larger, unless that takes the engine past what the configuration
        // allows (the Merlin is at the 20 litre ceiling): then smaller.
        auto larger = original;
        const auto size = cylinderSize(original);
        require(size.has_value(), original.name + ": the cylinders share one size");
        const auto grows = resizeCylinders(larger, size->boreMm + 2.0, size->strokeMm + 2.0).empty()
            && !validateEngineConfig(larger);
        const auto step = grows ? 2.0 : -2.0;
        const auto resized = resizedCylinders(original, step, step);
        Rig rig(original);
        double lowestAfter = 1.0e9;
        double rpm = 0.0;
        for (auto frame = 0; frame < 6 * 240; ++frame) {
            const auto t = frame * frameSeconds;
            if (frame == 3 * 240)
                require(rig.simulator->beginCylinderResize(resized, 0.0), original.name + ": resize");
            rpm = rig.simulator->step(frameSeconds, controlsAt(t, 0.0)).state.rpm;
            if (frame >= 3 * 240) lowestAfter = std::min(lowestAfter, rpm);
        }
        std::cout << "  " << original.name << ": " << (step > 0.0 ? "+" : "") << step
                  << " mm, lowest rpm after the resize " << lowestAfter
                  << " (idle " << original.idleRpm << "), " << rpm << " at 6 s\n";
        require(lowestAfter > 0.5 * original.idleRpm, original.name + ": the resized engine keeps idling");
        require(!rig.simulator->cylinderResizeActive(), original.name + ": every journal was resized");
        ++engines;
    }
    require(engines >= 16, "the whole catalogue was resized");
}

void liveChangeRegression() {
    identicalSwapIsInvisible();
    resizedSwapSettlesLikeARestart();
    identicalAudioSwapIsInaudible();
    resizedAudioSwapHasNoClick();
}

void liveCylinderResizeRegression() {
    identicalResizeIsInvisible();
    resizeHappensAtGasExchangeTdc();
    resizeSettlesLikeARestart();
    resizeAtIdleKeepsEveryEngineRunning();
}

} // namespace enginelab::tests
