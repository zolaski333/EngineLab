// A live intake change: the engine keeps running while its runners, plenums
// and throttles are resized. Each new runner takes over its predecessor's gas
// state, each plenum keeps its gas state at its new volume; the audio feeds
// the incoming acoustic intake silently, then fades to it. These tests drive
// the real simulator and the real renderer the way EngineRuntime does,
// deterministically.
#include <enginelab/audio/RealtimeEngineAudio.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/ConservativeGasSystem.hpp>
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

/** Cranked to 1.5 s, then `throttle` from 2 s; at full throttle held at 60 %
 * of redline by a proportional brake. */
[[nodiscard]] EngineControls heldAt(double timeSeconds, double rpm, const EngineConfig& config,
                                    double throttle = 1.0) {
    EngineControls controls;
    controls.ignitionEnabled = true;
    controls.starterEngaged = timeSeconds < 1.5;
    controls.throttle = timeSeconds < 2.0 ? 0.0 : throttle;
    if (timeSeconds >= 2.0 && throttle >= 1.0)
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

/** Every runner `runnerMm` longer, every plenum `plenumScale` times larger and
 * every throttle `throttleMm` wider, on each intake path and on the cylinders
 * that carry their own runner size. */
[[nodiscard]] EngineConfig resizedIntake(EngineConfig config, double runnerMm,
                                         double plenumScale, double throttleMm) {
    const auto resize = [&](IntakeConfig& intake) {
        intake.runnerLengthMm += runnerMm;
        intake.plenumVolumeLitres *= plenumScale;
        intake.throttleDiameterMm += throttleMm;
    };
    resize(config.intake);
    for (auto& path : config.intakePaths) resize(path.geometry);
    for (auto& cylinder : config.cylinders)
        if (cylinder.intakeRunnerLengthMm > 0.0) cylinder.intakeRunnerLengthMm += runnerMm;
    normaliseEngineConfig(config);
    const auto invalid = validateEngineConfig(config);
    require(!invalid, config.name + ": " + invalid.value_or(""));
    return config;
}

void swapIntake(Rig& rig, const EngineConfig& config) {
    require(EngineSimulator::intakeReplaceable(rig.simulator->config(), config),
            "the intake is replaceable");
    auto intake = rig.simulator->buildLiveIntake(config);
    require(intake != nullptr, "the edited intake builds");
    auto incoming = config;
    require(rig.simulator->replaceIntake(*intake, incoming),
            "the simulator takes the intake between two steps");
}

/** A plenum resized keeping its state: pressure, temperature and velocity
 * stay, the gas follows the volume; the same volume changes nothing. */
void plenumKeepsItsState() {
    GasCell cell;
    cell.initialise(87.5, 2.0, 310.0);
    cell.setBulkVelocityMps(3.0, -1.0);
    const auto same = cell;
    cell.resizeKeepingState(2.0);
    require(cell.pressureKpa() == same.pressureKpa() && cell.massKg() == same.massKg()
                && cell.internalEnergyJoules() == same.internalEnergyJoules(),
            "the same volume changes nothing");
    cell.resizeKeepingState(3.5);
    const auto close = [](double a, double b) { return std::abs(a - b) <= 1.0e-9 * std::abs(b); };
    require(close(cell.pressureKpa(), same.pressureKpa()), "the pressure stays");
    require(close(cell.temperatureK(), same.temperatureK()), "the temperature stays");
    require(close(cell.velocityXMps(), same.velocityXMps()) && close(cell.velocityYMps(), same.velocityYMps()),
            "the velocity stays");
    require(close(cell.massKg(), same.massKg() * 1.75), "the gas follows the volume");
}

/** Swapping in the same intake changes nothing: the run continues bit for bit. */
void identicalIntakeIsInvisible() {
    for (const auto* name : { "Yamaha CP2", "LS3" }) {
        const auto config = catalogueEngine(name);
        Rig reference(config);
        Rig swapped(config);
        double rpm = 0.0;
        for (auto frame = 0; frame < 4 * 240; ++frame) {
            const auto t = frame * frameSeconds;
            if (frame == 3 * 240) swapIntake(swapped, config);
            const auto controls = heldAt(t, rpm, config, 0.25);
            const auto a = reference.simulator->step(frameSeconds, controls);
            const auto b = swapped.simulator->step(frameSeconds, controls);
            rpm = a.state.rpm;
            require(a.state.rpm == b.state.rpm
                    && a.state.crankAngleDegrees == b.state.crankAngleDegrees
                    && a.state.manifoldPressureKpa == b.state.manifoldPressureKpa,
                std::string(name) + ": swapping in the same intake must leave the run bit-identical");
        }
    }
}

/** What adds or removes a part is refused, and the refusal changes nothing. */
void anotherIntakeIsRefused() {
    const auto config = catalogueEngine("Yamaha CP2");
    std::vector<std::pair<std::string, EngineConfig>> refused;
    auto edited = config;
    const auto withAirbox = config.intake.airboxVolumeLitres > 0.0;
    edited.intake.airboxVolumeLitres = withAirbox ? 0.0 : 6.0;
    for (auto& path : edited.intakePaths) path.geometry.airboxVolumeLitres = edited.intake.airboxVolumeLitres;
    refused.emplace_back(withAirbox ? "the airbox removed" : "an airbox added", edited);
    edited = config;
    edited.intake.throttleCount += 1;
    for (auto& path : edited.intakePaths) path.geometry.throttleCount += 1;
    refused.emplace_back("another throttle count", edited);
    edited = config;
    edited.intakePaths.push_back(edited.intakePaths.back());
    edited.intakePaths.back().id += 100;
    refused.emplace_back("another intake path", edited);
    for (auto& [what, intake] : refused) {
        require(!EngineSimulator::intakeReplaceable(config, intake), what + ": refused");
        Rig reference(config);
        Rig touched(config);
        double rpm = 0.0;
        for (auto frame = 0; frame < 3 * 240; ++frame) {
            const auto t = frame * frameSeconds;
            if (frame == 2 * 240) {
                auto built = touched.simulator->buildLiveIntake(config);
                require(built != nullptr, what + ": the running intake builds");
                auto incoming = intake;
                require(!touched.simulator->replaceIntake(*built, incoming), what + ": the simulator refuses it");
            }
            const auto controls = heldAt(t, rpm, config, 0.25);
            const auto a = reference.simulator->step(frameSeconds, controls);
            const auto b = touched.simulator->step(frameSeconds, controls);
            rpm = a.state.rpm;
            require(a.state.rpm == b.state.rpm && a.state.manifoldPressureKpa == b.state.manifoldPressureKpa,
                    what + ": a refused change leaves the run as it was");
        }
    }
}

/** A resized intake swapped in while the engine runs: no stall, no step, and
 * the engine settles where an engine built with that intake runs. */
void resizedIntakeSettlesLikeARestart() {
    for (const auto* name : { "Yamaha CP2", "K20A", "LS3" }) {
        const auto original = catalogueEngine(name);
        const auto resized = resizedIntake(original, 120.0, 1.6, 6.0);
        Rig built(resized);
        Rig swapped(original);
        Rig untouched(original);
        constexpr auto swapFrame = 6 * 240;
        constexpr auto frames = 14 * 240;
        std::array<double, 3> rpm {};
        std::array<std::vector<double>, 3> torque;
        std::array<std::vector<double>, 3> speed;
        double largestStepBefore = 0.0;
        double stepAtSwap = 0.0;
        double minimumRpm = 1.0e9;
        double previousRpm = 0.0;
        double manifoldBeforeKpa = 0.0;
        double manifoldAfterKpa = 0.0;
        for (auto frame = 0; frame < frames; ++frame) {
            const auto t = frame * frameSeconds;
            if (frame == swapFrame) swapIntake(swapped, resized);
            std::size_t index = 0;
            for (auto* rig : { &built, &swapped, &untouched }) {
                const auto controls = heldAt(t, rpm[index], original);
                const auto step = rig->simulator->step(frameSeconds, controls);
                rpm[index] = step.state.rpm;
                torque[index].push_back(controls.dynamometerTorqueNm);
                speed[index].push_back(step.state.rpm);
                if (index == 1 && frame == swapFrame - 1) manifoldBeforeKpa = step.state.manifoldPressureKpa;
                if (index == 1 && frame == swapFrame) manifoldAfterKpa = step.state.manifoldPressureKpa;
                ++index;
            }
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
        const auto builtMean = meanOfLast(torque[0]);
        const auto swappedMean = meanOfLast(torque[1]);
        const auto untouchedMean = meanOfLast(torque[2]);
        // The stiff dyno takes the new torque within a few frames: a perfect
        // swap may move the speed by the whole shift between the two
        // equilibria in one frame, on top of the engine's own steps.
        const auto settledShiftRpm = std::abs(meanOfLast(speed[0]) - meanOfLast(speed[2]));
        std::cout << "  " << name << ": settled brake torque, Nm: built " << builtMean
                  << ", swapped " << swappedMean << ", untouched " << untouchedMean
                  << "; largest frame step before " << largestStepBefore
                  << " rpm, in the 0.1 s after the swap " << stepAtSwap << " (settled shift "
                  << settledShiftRpm << ")"
                  << " rpm; lowest rpm after " << minimumRpm << "; manifold "
                  << manifoldBeforeKpa << " kPa the frame before, " << manifoldAfterKpa << " after\n";
        require(std::abs(untouchedMean - builtMean) > 0.005 * std::abs(untouchedMean),
                std::string(name) + ": the resized intake changes the settled torque (else this proves nothing)");
        require(minimumRpm > 0.5 * original.idleRpm,
                std::string(name) + ": the engine keeps running through the swap");
        require(stepAtSwap <= 1.5 * largestStepBefore + settledShiftRpm,
                std::string(name) + ": the swap stays within the engine's own frame-to-frame steps");
        require(std::abs(swappedMean - builtMean) <= 0.25 * std::abs(untouchedMean - builtMean),
                std::string(name) + ": the swapped engine settles where the resized engine runs");
    }
}

/** Every catalogue engine takes a resized intake at idle and keeps idling. */
void resizeAtIdleKeepsEveryEngineRunning() {
    static const auto catalogue = loadEngineCatalog(ENGINELAB_CATALOG_ROOT);
    std::size_t engines = 0;
    for (const auto& entry : catalogue.entries) {
        auto original = entry.config;
        normaliseEngineConfig(original);
        const auto resized = resizedIntake(original, 60.0, 1.3, 2.0);
        Rig rig(original);
        double lowestAfter = 1.0e9;
        double rpm = 0.0;
        for (auto frame = 0; frame < 6 * 240; ++frame) {
            const auto t = frame * frameSeconds;
            if (frame == 3 * 240) swapIntake(rig, resized);
            rpm = rig.simulator->step(frameSeconds, heldAt(t, rpm, original, 0.0)).state.rpm;
            if (frame >= 3 * 240) lowestAfter = std::min(lowestAfter, rpm);
        }
        std::cout << "  " << original.name << ": lowest rpm after the intake change " << lowestAfter
                  << " (idle " << original.idleRpm << "), " << rpm << " at 6 s\n";
        require(lowestAfter > 0.5 * original.idleRpm, original.name + ": the engine keeps idling");
        ++engines;
    }
    require(engines >= 16, "the whole catalogue was resized");
}

/** The runtime posts what the simulator can take and refuses the rest. */
void runtimeTakesIntake() {
    const auto config = catalogueEngine("Yamaha CP2");
    auto runtime = std::make_unique<EngineRuntime>(config);
    const auto resized = resizedIntake(config, 80.0, 1.2, 0.0);
    require(runtime->applyLiveIntake(resized), "the runtime takes a resized intake");
    require(runtime->engineConfig().intake.runnerLengthMm == resized.intake.runnerLengthMm,
            "the runtime's configuration carries the new runners");
    auto edited = resized;
    edited.intake.throttleCount += 1;
    for (auto& path : edited.intakePaths) path.geometry.throttleCount += 1;
    require(!runtime->applyLiveIntake(edited), "the runtime refuses another throttle count");
    edited = resized;
    edited.intake.runnerLengthMm = -5.0;
    for (auto& path : edited.intakePaths) path.geometry.runnerLengthMm = -5.0;
    require(!runtime->applyLiveIntake(edited), "the runtime refuses an invalid runner");
    require(runtime->engineConfig().intake.runnerLengthMm == resized.intake.runnerLengthMm
            && runtime->engineConfig().intake.throttleCount == config.intake.throttleCount,
            "a refused change leaves the runtime's configuration as it was");
}

/** engineEditScope tells an intake edit from the others. */
void editScopeSortsIntakeEdits() {
    const auto config = catalogueEngine("Yamaha CP2");
    const auto intake = engineEditScope(config, resizedIntake(config, 50.0, 1.1, 2.0));
    require(intake.intake && !intake.exhaust && !intake.settings && !intake.cylinders && !intake.other,
            "a resized intake is an intake edit only");
    auto runner = config;
    runner.cylinders.front().intakeRunnerLengthMm = config.intake.runnerLengthMm + 30.0;
    const auto runnerScope = engineEditScope(config, runner);
    require(runnerScope.intake && !runnerScope.cylinders && !runnerScope.other,
            "one cylinder's own runner is an intake edit");
    auto bored = resizedIntake(config, 50.0, 1.0, 0.0);
    for (auto& cylinder : bored.cylinders) cylinder.boreMm -= 1.0;
    const auto both = engineEditScope(config, bored);
    require(both.intake && both.cylinders && !both.other, "runners and bores are both, nothing else");
    // One path per cylinder, each its own: an edit of one path alone, which
    // the engine-wide intake does not mirror, is an intake edit too.
    auto split = config;
    const auto shared = split.intakePaths.front();
    split.intakePaths.clear();
    for (std::size_t index = 0; index < split.cylinders.size(); ++index) {
        auto path = shared;
        path.id = static_cast<std::uint32_t>(index + 1);
        path.cylinderIds = { split.cylinders[index].id };
        path.inheritsGlobalGeometry = false;
        split.intakePaths.push_back(path);
    }
    normaliseEngineConfig(split);
    require(!validateEngineConfig(split), "the CP2 with one intake path per cylinder is valid");
    auto onePath = split;
    onePath.intakePaths.back().geometry.runnerLengthMm += 40.0;
    const auto pathScope = engineEditScope(split, onePath);
    require(pathScope.intake && !pathScope.other, "one path's own runner is an intake edit");
}

/** Two renderers fed the same telemetry from one simulator; the second one
 * takes `intake` at `swapSeconds`. */
struct AudioPair final {
    std::vector<float> reference;
    std::vector<float> swapped;
    std::uint64_t swaps { 0 };
};

[[nodiscard]] AudioPair renderPair(const EngineConfig& config, const EngineConfig& intake,
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
        // Outlet turbulence is seeded noise; this compares intakes.
        listener.renderer->setOutletJetNoiseEnabled(false);
        require(listener.renderer->compiledIntakeTopologyActive(), "the engine plays its compiled intake");
    }
    double heldRpm = 0.0;
    AudioPair result;
    juce::AudioBuffer<float> block(2, samplesPerFrame);
    const auto frames = static_cast<int>(durationSeconds / frameSeconds);
    for (auto frame = 0; frame < frames; ++frame) {
        const auto t = frame * frameSeconds;
        if (frame == static_cast<int>(swapSeconds / frameSeconds))
            require(listeners[1].renderer->replaceIntake(intake), "the renderer accepts the intake");
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
            auto& out = index == 0 ? result.reference : result.swapped;
            for (int sample = 0; sample < samplesPerFrame; ++sample)
                out.push_back(block.getSample(0, sample));
        }
    }
    result.swaps = listeners[1].renderer->intakeSwapCount();
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

/** The same intake swapped into the renderer: the fade is not heard. A
 * resized one fades in without a click and is heard. */
void audioIntakeSwap() {
    const auto config = catalogueEngine("Yamaha CP2");
    constexpr double swapSeconds = 4.0;
    constexpr double duration = 5.0;
    const auto at = [](double seconds) { return static_cast<std::size_t>(seconds * 48'000.0); };
    const auto swapEnd = swapSeconds + RealtimeEngineAudio::exhaustSwapWarmupSeconds
        + RealtimeEngineAudio::exhaustSwapFadeSeconds;

    const auto same = renderPair(config, config, swapSeconds, duration);
    require(same.swaps == 1, "the renderer completes the intake swap");
    require(rmsDifference(same, 0, at(swapSeconds)) == 0.0,
            "two renderers fed the same telemetry play the same samples");
    const auto signal = rms(same.reference, at(swapSeconds), at(swapEnd + 0.1));
    const auto decibels = [](double value, double reference) {
        return 20.0 * std::log10(std::max(value, 1.0e-12) / std::max(reference, 1.0e-12));
    };
    const auto during = rmsDifference(same, at(swapSeconds), at(swapEnd + 0.1));
    const auto after = rmsDifference(same, at(swapEnd + 0.3), at(duration));
    std::cout << "  identical intake swap: signal rms " << signal << ", difference over the swap "
              << decibels(during, signal) << " dB, after it " << decibels(after, signal) << " dB\n";
    require(signal > 1.0e-4, "the engine is audible when the swap happens");
    require(decibels(during, signal) < -60.0, "the swap of an identical intake stays 60 dB below the engine");
    require(decibels(after, signal) < -90.0, "after the swap the identical intake plays as before");

    const auto resized = renderPair(config, resizedIntake(config, 120.0, 1.6, 6.0), swapSeconds, duration);
    require(resized.swaps == 1, "the renderer completes the resized intake swap");
    const auto largestJump = [&resized](std::size_t begin, std::size_t end) {
        double jump = 0.0;
        for (auto index = begin + 1; index < end; ++index)
            jump = std::max(jump, double(std::abs(resized.swapped[index] - resized.swapped[index - 1])));
        return jump;
    };
    const auto before = largestJump(at(swapSeconds - 1.0), at(swapSeconds));
    const auto fading = largestJump(at(swapSeconds), at(swapEnd + 0.05));
    const auto heard = rmsDifference(resized, at(swapEnd + 0.1), at(duration));
    const auto level = rms(resized.reference, at(swapEnd + 0.1), at(duration));
    std::cout << "  resized intake swap: largest sample step before " << before << ", during "
              << fading << "; the resized intake differs by " << decibels(heard, level) << " dB\n";
    require(fading <= 1.25 * before, "the resized intake fades in without a click");
    require(heard > 0.003 * level, "the resized intake is heard once the swap is over");
}

} // namespace

void liveIntakeRegression() {
    plenumKeepsItsState();
    editScopeSortsIntakeEdits();
    identicalIntakeIsInvisible();
    anotherIntakeIsRefused();
    runtimeTakesIntake();
    resizedIntakeSettlesLikeARestart();
    resizeAtIdleKeepsEveryEngineRunning();
    audioIntakeSwap();
}

} // namespace enginelab::tests
