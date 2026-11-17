// The active exhaust valve: a silencer bypass with a valve at its inlet.
// Open, the exhaust is the same exhaust without a valve, bit for bit, in the
// gas network and in the audio network; shut, the gas goes through the
// silencer as on the stock engine and the bypass, which ends in a tailpipe
// of its own, is a closed branch; the
// butterfly takes its travel time to move, automatic mode follows speed and
// throttle with a hysteresis, the sound has no click when it moves, and the
// engine file keeps the valve and its actuator.

#include <enginelab/audio/AcousticExhaustNetwork.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/exhaust/ExhaustPathTopologyEditor.hpp>
#include <enginelab/exhaust/LegacyExhaustNetwork.hpp>
#include <enginelab/gasdynamics/ExhaustNetworkLayout.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/serialization/JsonEngineSerializer.hpp>
#include <enginelab/serialization/YamlEngineSerializer.hpp>
#include <enginelab/simulation/DynoAbsorberController.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

namespace {

void require(bool condition, const std::string& message) {
    if (condition) return;
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
}

constexpr double frameSeconds = 1.0 / 240.0;

/** The catalogue's K20A, as it ships. */
enginelab::EngineConfig stockEngine() {
    const auto catalog = enginelab::loadEngineCatalog(ENGINELAB_CATALOG_ROOT);
    for (const auto& entry : catalog.entries)
        if (entry.config.name.find("K20") != std::string::npos) return entry.config;
    require(false, "the K20A is in the catalogue");
    return {};
}

/** The K20A with a valved bypass around its first path's silencer, and the
 *  bypass pipe's id. */
std::pair<enginelab::EngineConfig, std::uint32_t> makeBypassedEngine() {
    auto config = stockEngine();
    require(!config.exhaustPaths.empty(), "the K20A has an exhaust path");
    auto& path = config.exhaustPaths.front();
    if (!path.network) path.network = enginelab::makeEditableExhaustNetwork(path);
    std::uint32_t silencer = 0;
    for (const auto& component : path.network->components)
        if (component.type == enginelab::ExhaustComponentType::muffler) silencer = component.id;
    require(silencer != 0, "the K20A's exhaust has a silencer");
    const auto before = *path.network;
    std::uint32_t bypass = 0;
    const auto error = enginelab::ExhaustNetworkEditor::addValvedBypass(
        *path.network, silencer, &bypass);
    require(error.empty(), "the bypass template applies: " + error);
    require(bypass != 0, "the template names its bypass pipe");
    const auto invalid = enginelab::validateEngineConfig(config);
    require(!invalid, "the bypassed engine is valid: " + invalid.value_or(""));
    require(enginelab::exhaustValveFitted(config), "the engine reports its valve");
    // Each branch has its own tailpipe (owner 2026-11-14): the bypass pipe
    // ends in an outlet of its own and nothing merges the two again.
    const auto count = [&](enginelab::ExhaustComponentType type) {
        return std::count_if(path.network->components.begin(), path.network->components.end(),
            [type](const enginelab::ExhaustComponentConfig& component) { return component.type == type; });
    };
    const auto countBefore = [&](enginelab::ExhaustComponentType type) {
        return std::count_if(before.components.begin(), before.components.end(),
            [type](const enginelab::ExhaustComponentConfig& component) { return component.type == type; });
    };
    require(count(enginelab::ExhaustComponentType::outlet) == countBefore(enginelab::ExhaustComponentType::outlet) + 1,
            "the bypass ends in a tailpipe of its own");
    require(count(enginelab::ExhaustComponentType::merge) == countBefore(enginelab::ExhaustComponentType::merge),
            "nothing merges the bypass back into the silenced branch");
    bool toOutlet = false;
    for (const auto& connection : path.network->connections)
        if (connection.fromComponentId == bypass)
            for (const auto& component : path.network->components)
                if (component.id == connection.toComponentId)
                    toOutlet = component.type == enginelab::ExhaustComponentType::outlet;
    require(toOutlet, "the bypass pipe runs straight to its tailpipe");
    return { config, bypass };
}

void setValve(enginelab::EngineConfig& config, std::uint32_t pipe, bool fitted) {
    for (auto& component : config.exhaustPaths.front().network->components)
        if (component.id == pipe) component.inletValve = fitted;
}

struct Rig final {
    explicit Rig(const enginelab::EngineConfig& engine)
        : config(engine), exhaust(enginelab::ExhaustGraph::makeForEngine(engine)),
          simulator(std::make_unique<enginelab::EngineSimulator>(
              config, ecu, physics, events, exhaust)) {}
    enginelab::EngineConfig config;
    enginelab::SimpleEcuModel ecu;
    enginelab::SimplifiedGasolinePhysics physics;
    enginelab::FourStrokeEventGenerator events;
    enginelab::ExhaustGraph exhaust;
    std::unique_ptr<enginelab::EngineSimulator> simulator;
};

enginelab::EngineControls startControls(double time, double throttle,
                                        enginelab::ExhaustValveMode mode) {
    enginelab::EngineControls controls;
    controls.ignitionEnabled = true;
    controls.starterEngaged = time < 1.1;
    controls.throttle = time < 0.9 ? 0.2 : throttle;
    controls.exhaustValveMode = mode;
    return controls;
}

void checkOpenIsNoValve(const enginelab::EngineConfig& withValve,
                        const enginelab::EngineConfig& without) {
    auto valved = std::make_unique<Rig>(withValve);
    auto plain = std::make_unique<Rig>(without);
    for (int frame = 0; frame < static_cast<int>(3.0 / frameSeconds); ++frame) {
        const auto time = frame * frameSeconds;
        const auto throttle = time < 2.0 ? 0.0 : 0.6;
        const auto a = valved->simulator->step(frameSeconds,
            startControls(time, throttle, enginelab::ExhaustValveMode::open));
        const auto b = plain->simulator->step(frameSeconds,
            startControls(time, throttle, enginelab::ExhaustValveMode::open));
        require(a.state.exhaustValveCount == 1 && b.state.exhaustValveCount == 0,
            "one valve on the bypassed engine, none on its twin");
        require(a.state.exhaustValveOpening == 1.0, "an open valve starts open");
        require(a.state.rpm == b.state.rpm && a.state.torqueNm == b.state.torqueNm
                && a.state.exhaustBackPressureKpa == b.state.exhaustBackPressureKpa
                && a.state.exhaustPressureKpa == b.state.exhaustPressureKpa,
            "open, the valve is the exhaust without it, bit for bit (frame "
                + std::to_string(frame) + ")");
    }
    std::cout << "open valve == no valve over 720 frames (start, idle, 60 % throttle)\n";
}

double meanBackPressure(Rig& rig, enginelab::DynoAbsorberController& absorber,
                        enginelab::ExhaustValveMode mode, double seconds,
                        std::vector<double>* strokes = nullptr) {
    double sum = 0.0;
    int count = 0;
    const auto frames = static_cast<int>(seconds / frameSeconds);
    for (int frame = 0; frame < frames; ++frame) {
        enginelab::EngineControls controls;
        controls.ignitionEnabled = true;
        controls.throttle = 1.0;
        controls.exhaustValveMode = mode;
        controls.dynamometerTorqueNm = absorber.advance(
            frameSeconds, 4'000.0, rig.simulator->state()).brakeTorqueNm;
        const auto result = rig.simulator->step(frameSeconds, controls);
        if (strokes != nullptr) strokes->push_back(result.state.exhaustValveStroke);
        if (frame >= frames / 2) {
            sum += result.state.exhaustBackPressureKpa;
            ++count;
        }
    }
    return sum / std::max(1, count);
}

/** A started engine held at 4,000 rpm at full throttle by its absorber. */
struct HeldEngine final {
    std::unique_ptr<Rig> rig;
    std::unique_ptr<enginelab::DynoAbsorberController> absorber;
};

HeldEngine startHeld(const enginelab::EngineConfig& config) {
    HeldEngine held { std::make_unique<Rig>(config), nullptr };
    for (int frame = 0; frame < static_cast<int>(1.5 / frameSeconds); ++frame)
        static_cast<void>(held.rig->simulator->step(frameSeconds,
            startControls(frame * frameSeconds, 0.3, enginelab::ExhaustValveMode::closed)));
    require(held.rig->simulator->state().rpm > 600.0, "the engine runs");
    held.absorber = std::make_unique<enginelab::DynoAbsorberController>(config);
    held.absorber->reset(held.rig->simulator->state().rpm, held.rig->simulator->state().torqueNm);
    return held;
}

void checkShutAndTravel(const enginelab::EngineConfig& withValve, const enginelab::EngineConfig& stock) {
    auto plain = startHeld(stock);
    const auto original = meanBackPressure(*plain.rig, *plain.absorber, enginelab::ExhaustValveMode::closed, 3.0);
    auto held = startHeld(withValve);
    auto& rig = held.rig;
    const auto shut = meanBackPressure(*rig, *held.absorber, enginelab::ExhaustValveMode::closed, 3.0);
    require(rig->simulator->state().exhaustValveStroke == 0.0
            && rig->simulator->state().exhaustValveOpening == 0.0,
        "closed mode holds the valve shut");
    std::vector<double> strokes;
    const auto open = meanBackPressure(*rig, *held.absorber, enginelab::ExhaustValveMode::open, 3.0, &strokes);
    std::cout << "back pressure at 4,000 rpm full throttle: stock " << original << " kPa, shut " << shut
              << " kPa, open " << open << " kPa\n";
    // Shut, the gas takes the stock way through the silencer to the stock
    // tailpipe; the bypass is a dead branch. (The K20A's silencer restricts
    // little, 0.25: opening the bypass adds the split's loss and takes away
    // less, so open reads 1.3 kPa above shut. The 4.9 kPa the other way
    // before 2026-11-14 was the merge that rejoined the branches.)
    require(std::abs(shut - original) < 0.05,
        "shut, the exhaust is the stock exhaust: the silenced branch is untouched");
    require(std::abs(open - shut) > 0.5, "open, the bypass carries gas: the back pressure moves");
    // 0.3 s from shut to open at a constant rate: 72 frames.
    const auto travel = withValve.exhaustValve.travelSeconds;
    for (std::size_t frame = 0; frame < strokes.size(); ++frame) {
        const auto expected = std::min(1.0, static_cast<double>(frame + 1) * frameSeconds / travel);
        require(std::abs(strokes[frame] - expected) < 1.0e-9,
            "the butterfly turns at one stroke per travel time (frame " + std::to_string(frame) + ")");
    }
    require(strokes[70] < 1.0 && strokes[72] == 1.0, "a full stroke takes 0.3 s");
}

void checkAutomatic(const enginelab::EngineConfig& withValve) {
    auto config = withValve;
    config.exhaustValve.openAboveRpm = 5'000.0;
    config.exhaustValve.openAboveThrottle = 0.5;
    config.exhaustValve.travelSeconds = 0.0;
    auto rig = std::make_unique<Rig>(config);
    enginelab::DynoAbsorberController absorber(config);
    auto hold = [&](double throttle, double seconds) {
        for (int frame = 0; frame < static_cast<int>(seconds / frameSeconds); ++frame) {
            enginelab::EngineControls controls;
            controls.ignitionEnabled = true;
            controls.throttle = throttle;
            controls.dynamometerTorqueNm = absorber.advance(
                frameSeconds, 2'500.0, rig->simulator->state()).brakeTorqueNm;
            static_cast<void>(rig->simulator->step(frameSeconds, controls));
        }
        return rig->simulator->state().exhaustValveStroke;
    };
    for (int frame = 0; frame < static_cast<int>(1.5 / frameSeconds); ++frame)
        static_cast<void>(rig->simulator->step(frameSeconds,
            startControls(frame * frameSeconds, 0.3, enginelab::ExhaustValveMode::automatic)));
    absorber.reset(rig->simulator->state().rpm, rig->simulator->state().torqueNm);
    require(hold(0.3, 1.0) == 0.0, "automatic: under both thresholds, shut");
    require(hold(0.6, 1.0) == 1.0, "automatic: over the throttle threshold, open");
    require(hold(0.47, 0.5) == 1.0, "automatic: inside the 5 % hysteresis, still open");
    require(hold(0.40, 0.5) == 0.0, "automatic: under the hysteresis, shut again");
    std::cout << "automatic mode follows the throttle with its hysteresis\n";
}

struct AcousticRun final {
    std::vector<float> left;
};

/** The audio network fed a 50 Hz train of short pulses; `opening(t)` gives
 *  the valve's target each 5 ms block, `ramp` the per-sample glide. */
template <typename Opening>
AcousticRun renderAcoustic(const enginelab::EngineConfig& config, Opening opening,
                           float ramp, double seconds) {
    constexpr double rate = 48'000.0;
    const auto graph = enginelab::ExhaustGraph::makeForEngine(config);
    std::vector<std::uint32_t> ids;
    for (const auto& cylinder : config.cylinders) ids.push_back(cylinder.id);
    enginelab::AcousticExhaustNetwork network(graph, ids);
    require(network.valid() && network.prepare(rate), "the audio network compiles");
    network.setOutletJetNoiseEnabled(false);
    std::vector<enginelab::AcousticExhaustNetwork::Medium> media(
        config.exhaustPaths.size(), { 0.55F, 560.0F });
    std::vector<float> sources(ids.size(), 0.0F);
    std::vector<enginelab::AcousticExhaustNetwork::CylinderBoundary> boundaries(ids.size());
    AcousticRun run;
    const auto samples = static_cast<std::size_t>(seconds * rate);
    run.left.reserve(samples);
    constexpr std::size_t block = 240;
    for (std::size_t sample = 0; sample < samples; ++sample) {
        if (sample % block == 0) {
            network.setExhaustValveOpening(static_cast<float>(opening(static_cast<double>(sample) / rate)));
            network.beginBlock(media, 1.0);
        }
        const auto phase = std::fmod(static_cast<double>(sample) / rate * 50.0, 1.0);
        for (std::size_t cylinder = 0; cylinder < sources.size(); ++cylinder) {
            const auto local = std::fmod(phase + static_cast<double>(cylinder) / static_cast<double>(sources.size()), 1.0);
            sources[cylinder] = local < 0.02
                ? static_cast<float>(2'000.0 * std::sin(std::numbers::pi * local / 0.02)) : 0.0F;
        }
        const auto output = network.process(sources, boundaries, ramp);
        float sum = 0.0F;
        for (std::size_t path = 0; path < config.exhaustPaths.size(); ++path)
            sum += output[path].leftPa;
        run.left.push_back(sum);
    }
    return run;
}

/** The train repeats every 960 samples (50 Hz at 48 kHz), so a settled
 *  network's output does too. The largest change, over [first, last), of the
 *  second difference from one period to the next: zero once settled, small
 *  where the network changes smoothly, large where something steps (a click
 *  is a step of the waveform, whose second difference is a spike). */
constexpr std::size_t trainPeriod = 960;
double largestPeriodChange(const std::vector<float>& signal, std::size_t first, std::size_t last) {
    const auto curvature = [&signal](std::size_t index) {
        return static_cast<double>(signal[index]) - 2.0 * signal[index - 1] + signal[index - 2];
    };
    double largest = 0.0;
    for (auto index = std::max<std::size_t>(first, trainPeriod + 2); index < last; ++index)
        largest = std::max(largest, std::abs(curvature(index) - curvature(index - trainPeriod)));
    return largest;
}

/** The largest second difference over [first, last) samples. */
double largestCurvature(const std::vector<float>& signal, std::size_t first, std::size_t last) {
    double largest = 0.0;
    for (auto index = std::max<std::size_t>(first, 2); index < last; ++index)
        largest = std::max(largest, std::abs(static_cast<double>(signal[index])
            - 2.0 * signal[index - 1] + signal[index - 2]));
    return largest;
}

void checkAcoustic(const enginelab::EngineConfig& withValve,
                   const enginelab::EngineConfig& without) {
    constexpr double rate = 48'000.0;
    const auto ramp = static_cast<float>(1.0 - std::exp(-2.0 * std::numbers::pi * 10.0 / rate));
    {
        const auto graph = enginelab::ExhaustGraph::makeForEngine(withValve);
        const auto layout = enginelab::gasdynamics::ExhaustNetworkLayout::compile(graph);
        require(enginelab::gasdynamics::exhaustValveInterfaces(layout).size() == 1,
            "the layout carries one valve face");
        std::vector<std::uint32_t> ids;
        for (const auto& cylinder : withValve.cylinders) ids.push_back(cylinder.id);
        enginelab::AcousticExhaustNetwork network(graph, ids);
        require(network.exhaustValveCount() == 1, "the audio network carries the valve");
    }
    const auto open = renderAcoustic(withValve, [](double) { return 1.0; }, ramp, 0.5);
    const auto plain = renderAcoustic(without, [](double) { return 1.0; }, ramp, 0.5);
    require(open.left == plain.left, "open, the audio network is the one without a valve, bit for bit");
    const auto shut = renderAcoustic(withValve, [](double) { return 0.0; }, ramp, 0.5);
    double difference = 0.0;
    double level = 0.0;
    for (std::size_t index = 0; index < shut.left.size(); ++index) {
        difference += std::pow(static_cast<double>(shut.left[index]) - open.left[index], 2.0);
        level += std::pow(static_cast<double>(open.left[index]), 2.0);
    }
    require(difference > 0.01 * level, "shut, the bypass changes the sound");

    // Shut for 1 s, then the 0.3 s stroke (1 - cos of the angle), then open.
    const auto stroke = [](double time) {
        const auto travelled = std::clamp((time - 1.0) / 0.3, 0.0, 1.0);
        return travelled >= 1.0 ? 1.0 : 1.0 - std::cos(0.5 * std::numbers::pi * travelled);
    };
    const auto snap = [](double time) { return time < 1.0 ? 0.0 : 1.0; };
    const auto moving = renderAcoustic(withValve, stroke, ramp, 2.0);
    const auto snapped = renderAcoustic(withValve, snap, ramp, 2.0);
    const auto unsmoothed = renderAcoustic(withValve, snap, 1.0F, 2.0);
    const auto at = [rate](double seconds) { return static_cast<std::size_t>(seconds * rate); };
    const auto curvature = largestCurvature(moving.left, at(0.5), at(2.0));
    const auto settled = std::max(largestPeriodChange(moving.left, at(0.6), at(1.0)),
                                  largestPeriodChange(moving.left, at(1.7), at(2.0)));
    const auto during = largestPeriodChange(moving.left, at(1.0), at(1.6));
    const auto duringSnap = largestPeriodChange(snapped.left, at(1.0), at(1.6));
    const auto duringUnsmoothed = largestPeriodChange(unsmoothed.left, at(1.0), at(1.6));
    std::cout << "largest second difference " << curvature
              << " Pa; its change from one period to the next: settled " << settled
              << ", valve moving " << during << ", snapped in a frame " << duringSnap
              << ", snapped without the glide " << duringUnsmoothed << '\n';
    require(settled < 0.05 * curvature, "the network settles into the train's period");
    require(during < 0.25 * duringUnsmoothed, "no click while the valve moves");
    require(duringSnap < 0.5 * duringUnsmoothed, "the glide takes the step out of a snapping actuator");
}

void checkFiles(const enginelab::EngineConfig& withValve, std::uint32_t pipe) {
    auto config = withValve;
    config.exhaustValve.openAboveRpm = 4'100.0;
    config.exhaustValve.openAboveThrottle = 0.37;
    config.exhaustValve.travelSeconds = 0.45;
    const auto check = [&](const enginelab::IEngineSerializer& serializer, const char* name) {
        const auto text = serializer.encode(config);
        const auto decoded = serializer.decode(text);
        require(static_cast<bool>(decoded), std::string(name) + " reads the valved engine back: " + decoded.error);
        bool valve = false;
        for (const auto& component : decoded.config->exhaustPaths.front().network->components)
            if (component.id == pipe) valve = component.inletValve;
        require(valve, std::string(name) + " keeps the valve on its pipe");
        require(decoded.config->exhaustValve.openAboveRpm == 4'100.0
                && decoded.config->exhaustValve.openAboveThrottle == 0.37
                && decoded.config->exhaustValve.travelSeconds == 0.45,
            std::string(name) + " keeps the actuator's settings");
        auto plain = config;
        setValve(plain, pipe, false);
        const auto plainText = serializer.encode(plain);
        // "exhaust_valve_count" is the cylinder head's: look for the whole keys.
        require(plainText.find("inlet_valve") == std::string::npos
                && plainText.find("\"exhaust_valve\"") == std::string::npos
                && plainText.find("exhaust_valve:") == std::string::npos,
            std::string(name) + ": an engine without a valve writes nothing of it");
    };
    check(enginelab::JsonEngineSerializer {}, "JSON");
    check(enginelab::YamlEngineSerializer {}, "YAML");

    auto onMuffler = withValve;
    for (auto& component : onMuffler.exhaustPaths.front().network->components)
        if (component.type == enginelab::ExhaustComponentType::muffler) component.inletValve = true;
    require(enginelab::validateEngineConfig(onMuffler).has_value(), "a valve sits on a pipe only");
    auto onPrimary = withValve;
    const auto& network = *onPrimary.exhaustPaths.front().network;
    const auto primary = network.cylinderConnections.front().componentId;
    for (auto& component : onPrimary.exhaustPaths.front().network->components)
        if (component.id == primary) component.inletValve = true;
    require(enginelab::validateEngineConfig(onPrimary).has_value(),
        "a pipe fed by a cylinder takes no valve");
}

} // namespace

int main() {
    auto [withValve, pipe] = makeBypassedEngine();
    auto without = withValve;
    setValve(without, pipe, false);
    checkOpenIsNoValve(withValve, without);
    checkShutAndTravel(withValve, stockEngine());
    checkAutomatic(withValve);
    checkAcoustic(withValve, without);
    checkFiles(withValve, pipe);
    std::cout << "EngineLab.ExhaustValve PASS\n";
    return 0;
}
