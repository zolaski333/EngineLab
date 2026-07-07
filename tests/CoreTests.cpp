#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/foundation/SpscQueue.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/serialization/JsonEngineSerializer.hpp>
#include <enginelab/serialization/YamlEngineSerializer.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>
#include <enginelab/runtime/EngineRuntime.hpp>
#include <enginelab/audio/RealtimeEngineAudio.hpp>
#include <chrono>
#include <algorithm>
#include <array>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <numeric>
#include <set>
#include <cmath>
#include <stdexcept>
#include <thread>

namespace {
void require(bool condition, const char* message) {
    if (!condition) { std::cerr << "FAILED: " << message << '\n'; std::exit(EXIT_FAILURE); }
}
}

int main() {
    auto config = enginelab::makeDefaultInlineFour();

    {
        enginelab::SimpleEcuModel noStartEcu;
        enginelab::SimplifiedGasolinePhysics noStartPhysics;
        enginelab::FourStrokeEventGenerator noStartEvents;
        auto noStartExhaust = enginelab::ExhaustGraph::makeForEngine(config);
        enginelab::EngineSimulator noStart(config, noStartEcu, noStartPhysics, noStartEvents, noStartExhaust);
        for (int step = 0; step < 1'200; ++step) (void)noStart.step(1.0 / 240.0, { true, false, 0.65, 0.0 });
        require(noStart.state().rpm == 0.0, "ignition and throttle must not start a stationary engine without starter");
        require(noStart.state().volumetricEfficiency == 0.0, "stationary engine must report zero volumetric efficiency");
        require(std::abs(noStart.state().manifoldPressureKpa - config.ambientPressureKpa) < 0.01,
                "stationary manifold pressure must settle at ambient pressure");
    }
    enginelab::SimpleEcuModel ecu;
    enginelab::SimplifiedGasolinePhysics physics;
    enginelab::FourStrokeEventGenerator events;
    auto exhaust = enginelab::ExhaustGraph::makeForEngine(config);
    enginelab::EngineSimulator simulator(config, ecu, physics, events, exhaust);
    std::set<std::uint32_t> firedCylinders;
    for (int step = 0; step < 2'400; ++step) {
        const enginelab::EngineControls controls { true, step < 600, 0.45, 0.05 };
        const auto frame = simulator.step(1.0 / 240.0, controls);
        for (std::size_t index = 0; index < frame.firingEventCount; ++index)
            firedCylinders.insert(frame.firingEvents[index].cylinderId);
    }
    require(simulator.state().rpm > 500.0, "engine should start");
    require(firedCylinders == std::set<std::uint32_t>({ 1, 2, 3, 4 }), "all cylinders should fire");
    require(std::abs(simulator.state().torqueNm
            - (simulator.state().indicatedTorqueNm - simulator.state().frictionTorqueNm)) < 0.001,
            "brake torque must equal indicated torque minus engine losses");
    require(std::abs(simulator.state().netTorqueNm
            - (simulator.state().torqueNm + simulator.state().starterTorqueNm - simulator.state().loadTorqueNm
               + simulator.state().reciprocatingTorqueNm)) < 0.001,
            "net acceleration torque must have an explicit balance");
    require(std::isfinite(simulator.state().cycleAveragedTorqueNm)
            && std::isfinite(simulator.state().cycleAveragedPowerKw),
            "cycle-averaged output must remain finite for UI and dyno consumers");

    enginelab::SpscQueue<int, 8> queue;
    require(queue.tryPush(42), "queue push should succeed");
    int value = 0;
    require(queue.tryPop(value) && value == 42, "queue must preserve payload");

    const enginelab::JsonEngineSerializer json;
    const auto jsonRoundTrip = json.decode(json.encode(config));
    require(static_cast<bool>(jsonRoundTrip), "JSON round trip should decode");
    require(jsonRoundTrip.config->firingOrder == config.firingOrder, "JSON must preserve firing order");
    require(std::abs(jsonRoundTrip.config->exhaust.primaryLengthMm - config.exhaust.primaryLengthMm) < 0.001,
            "JSON must preserve exhaust geometry");
    require(std::abs(jsonRoundTrip.config->plenumVolumeLitres - config.plenumVolumeLitres) < 0.001,
            "JSON must preserve intake plenum geometry");
    const auto v8JsonRoundTrip = json.decode(json.encode(enginelab::makeDefaultV8()));
    require(v8JsonRoundTrip && v8JsonRoundTrip.config->layout == enginelab::EngineLayout::vLayout,
            "JSON must preserve V engine layout");
    require(!json.decode(R"({"schema_version":1,"engine":{"name":"bad","cycle":"steam","fuel":"gasoline"}})"),
            "JSON must reject unknown enum values");

    const enginelab::YamlEngineSerializer yaml;
    const auto yamlRoundTrip = yaml.decode(yaml.encode(config));
    require(static_cast<bool>(yamlRoundTrip), "YAML round trip should decode");
    require(yamlRoundTrip.config->cylinders.size() == 4, "YAML must preserve cylinders");
    const auto v8YamlRoundTrip = yaml.decode(yaml.encode(enginelab::makeDefaultV8()));
    require(v8YamlRoundTrip && v8YamlRoundTrip.config->layout == enginelab::EngineLayout::vLayout,
            "YAML must preserve V engine layout");
    require(v8YamlRoundTrip && std::abs(v8YamlRoundTrip.config->bankAngleDegrees - 90.0) < 0.001,
            "YAML must preserve V-engine bank angle");
    require(std::abs(jsonRoundTrip.config->camshafts.intakeLiftMm - config.camshafts.intakeLiftMm) < 0.001,
            "JSON must preserve camshaft lift");
    auto invalidPhysicalConfig = config;
    invalidPhysicalConfig.cylinders.front().boreMm = -1.0;
    require(!json.decode(json.encode(invalidPhysicalConfig)), "decoder must reject physically invalid engine files");
    require(!yaml.decode(yaml.encode(invalidPhysicalConfig)), "YAML decoder must reject physically invalid engine files");

    const auto presets = enginelab::makeBaseEnginePresets();
    require(presets.size() == 5, "I2, I4, I5, V6 and V8 presets must exist");
    require(presets[0].cylinders.size() == 2 && presets[2].cylinders.size() == 5
            && presets[3].cylinders.size() == 6 && presets[4].cylinders.size() == 8,
            "base presets must expose the requested cylinder counts");
    require(enginelab::engineDisplacementLitres(presets[4]) > enginelab::engineDisplacementLitres(presets[0]),
            "V8 displacement should exceed I2 displacement");

    {
        const auto simulate = [](const enginelab::EngineConfig& testConfig, double dt) {
            enginelab::SimpleEcuModel testEcu;
            enginelab::SimplifiedGasolinePhysics testPhysics;
            enginelab::FourStrokeEventGenerator testEvents;
            auto testExhaust = enginelab::ExhaustGraph::makeForEngine(testConfig);
            enginelab::EngineSimulator testSimulator(testConfig, testEcu, testPhysics, testEvents, testExhaust);
            const auto steps = static_cast<int>(6.0 / dt);
            for (int index = 0; index < steps; ++index)
                (void)testSimulator.step(dt, { true, static_cast<double>(index) * dt < 2.0, 0.52, 0.08 });
            return testSimulator.state();
        };
        const auto at120 = simulate(config, 1.0 / 120.0);
        const auto at240 = simulate(config, 1.0 / 240.0);
        const auto at480 = simulate(config, 1.0 / 480.0);
        const auto rpmSpread = std::max({ at120.rpm, at240.rpm, at480.rpm }) - std::min({ at120.rpm, at240.rpm, at480.rpm });
        if (rpmSpread / std::max(1.0, at240.rpm) >= 0.06)
            std::cerr << "step RPM: " << at120.rpm << ", " << at240.rpm << ", " << at480.rpm << '\n';
        require(rpmSpread / std::max(1.0, at240.rpm) < 0.06, "simulation result must remain stable across supported fixed steps");
        for (const auto& preset : presets) {
            const auto state = simulate(preset, 1.0 / 240.0);
            if (!(std::isfinite(state.rpm) && std::isfinite(state.torqueNm) && state.rpm > 300.0))
                std::cerr << "preset failed: " << preset.name << " rpm=" << state.rpm << " torque=" << state.torqueNm
                          << " damage=" << state.damage << " coolant=" << state.coolantTemperatureC << '\n';
            require(std::isfinite(state.rpm) && std::isfinite(state.torqueNm) && state.rpm > 300.0,
                    "every base preset must run to a finite self-sustaining state");
            require(state.rpm < preset.redlineRpm * 1.15, "preset must stay bounded by ECU limiter and mechanical losses");
        }
    }
    require(enginelab::valveLiftMm(470.0, 470.0, 248.0, 10.2) > 10.19,
            "valve must reach configured lift at cam centerline");
    require(enginelab::valveLiftMm(100.0, 470.0, 248.0, 10.2) == 0.0,
            "valve must be closed outside cam duration");
    require(enginelab::combustionPulse(76.0, 40.0, 0.0) > 0.999,
            "ignition advance must move the mechanical pressure peak earlier in the crank cycle");
    require(enginelab::combustionPulse(90.0, 0.0, 10.0) < 0.98,
            "per-cylinder ignition offset must shift the mechanical pressure trace");

    {
        enginelab::SimpleEcuModel misfireEcu;
        misfireEcu.setTargetAirFuelRatio(18.0);
        enginelab::SimplifiedGasolinePhysics misfirePhysics;
        enginelab::FourStrokeEventGenerator misfireEvents;
        auto misfireExhaust = enginelab::ExhaustGraph::makeForEngine(config);
        enginelab::EngineSimulator misfireSimulator(config, misfireEcu, misfirePhysics, misfireEvents, misfireExhaust);
        bool observedMechanicalMisfire = false;
        for (int step = 0; step < 4'000; ++step) {
            (void)misfireSimulator.step(1.0 / 240.0, { true, step < 600, 0.55, 0.04 });
            const auto& state = misfireSimulator.state();
            for (std::size_t index = 0; index < state.cylinderStateCount; ++index) {
                if (state.cylinderStates[index].misfiring) {
                    observedMechanicalMisfire = true;
                    require(state.cylinderStates[index].combustionPulse == 0.0,
                            "a misfiring cylinder must remove its mechanical pressure pulse");
                }
            }
        }
        require(observedMechanicalMisfire, "deterministic lean-mixture run must expose cylinder-level misfires");
    }

    {
        enginelab::FourStrokeEventGenerator timingEvents;
        enginelab::EngineState timingState;
        timingState.simulationTimeSeconds = 1.0;
        timingState.rpm = 3'000.0;
        timingState.throttle = 0.8;
        enginelab::EcuCommand command;
        command.ignitionAdvanceDegrees = 18.0;
        command.fuelEnabled = true;
        command.sparkEnabled = true;
        enginelab::CombustionResult combustion;
        combustion.combustionQuality = 1.0;
        combustion.pressureEstimateBar = 70.0;
        std::array<enginelab::FiringEvent, 64> generated {};
        const auto count = timingEvents.generate(config, timingState, command, combustion,
            0.95, 680.0, 1'500.0, 0.05, generated);
        require(count >= 8, "event generator must preserve firings across multiple crank cycles");
        for (std::size_t index = 1; index < count; ++index)
            require(generated[index - 1].timeSeconds <= generated[index].timeSeconds,
                    "firing events must be chronologically ordered and interpolated");
        require(std::any_of(generated.begin(), generated.begin() + static_cast<std::ptrdiff_t>(count), [](const auto& event) {
            return std::abs(event.crankAngleDegrees - 702.0) < 0.01;
        }), "ignition advance must shift firing angle before top dead centre");
    }

    {
        auto v8Exhaust = enginelab::ExhaustGraph::makeForEngine(enginelab::makeDefaultV8());
        require(v8Exhaust.nodes().size() == 11, "exhaust graph must contain one primary per V8 cylinder");
        enginelab::FiringEvent event;
        event.exhaustPortId = 8;
        event.intensity = 1.0F;
        v8Exhaust.process(event);
        require(event.exhaustDelaySeconds > 0.0F && event.exhaustResonanceHz > 0.0F,
                "exhaust graph must propagate path delay and resonance into firing events");
    }

    {
        enginelab::EngineState healthyState;
        healthyState.rpm = 3'500.0;
        healthyState.manifoldPressureKpa = 95.0;
        enginelab::EcuCommand command;
        command.effectiveThrottle = 0.9;
        const auto healthy = physics.evaluateCombustion(config, healthyState, { true, false, 0.9, 0.2 }, command, 104.0);
        auto damagedState = healthyState;
        damagedState.damage = 0.8;
        const auto damaged = physics.evaluateCombustion(config, damagedState, { true, false, 0.9, 0.2 }, command, 104.0);
        require(healthy.indicatedTorqueNm > damaged.indicatedTorqueNm * 2.0,
                "damage must materially reduce combustion torque");
    }

    {
        auto invalidCam = config;
        invalidCam.camshafts.intakeCenterlineDegrees = std::numeric_limits<double>::quiet_NaN();
        require(enginelab::validateEngineConfig(invalidCam).has_value(),
                "non-finite cam timing must be rejected before it can poison simulation state");
        auto invalidPlenum = config;
        invalidPlenum.plenumVolumeLitres = 0.0;
        require(enginelab::validateEngineConfig(invalidPlenum).has_value(),
                "zero plenum volume must be rejected");
        auto unsupportedSchema = config;
        unsupportedSchema.schemaVersion = 99;
        require(enginelab::validateEngineConfig(unsupportedSchema).has_value(),
                "runtime construction must reject unsupported schemas too");
    }

    {
        auto invalidIdConfig = config;
        invalidIdConfig.cylinders.front().id = 4294967293U;
        require(enginelab::validateEngineConfig(invalidIdConfig).has_value(),
                "configuration with cylinder ID equal to or greater than reserved mergeId must be rejected");
    }

    {
        auto zeroLiftConfig = config;
        zeroLiftConfig.camshafts.intakeLiftMm = 0.0;
        zeroLiftConfig.camshafts.exhaustLiftMm = 0.0;
        enginelab::SimpleEcuModel zeroLiftEcu;
        enginelab::SimplifiedGasolinePhysics zeroLiftPhysics;
        enginelab::FourStrokeEventGenerator zeroLiftEvents;
        auto zeroLiftExhaust = enginelab::ExhaustGraph::makeForEngine(zeroLiftConfig);
        enginelab::EngineSimulator zeroLiftSim(zeroLiftConfig, zeroLiftEcu, zeroLiftPhysics, zeroLiftEvents, zeroLiftExhaust);
        for (int step = 0; step < 100; ++step) {
            (void)zeroLiftSim.step(1.0 / 240.0, { true, step < 20, 0.5, 0.0 });
        }
        require(zeroLiftSim.state().volumetricEfficiency == 0.0,
                "zero valve lift must result in zero volumetric efficiency");
        require(zeroLiftSim.state().rpm < 300.0,
                "engine must not start and run with zero valve lift");
    }

    {
        auto customCrankConfig = config;
        customCrankConfig.cylinders[0].crankOffsetDegrees = 90.0;
        customCrankConfig.cylinders[1].crankOffsetDegrees = 270.0;
        customCrankConfig.cylinders[2].crankOffsetDegrees = 450.0;
        customCrankConfig.cylinders[3].crankOffsetDegrees = 630.0;
        const enginelab::JsonEngineSerializer jsonSer;
        const auto jsonRt = jsonSer.decode(jsonSer.encode(customCrankConfig));
        require(jsonRt && std::abs(jsonRt.config->cylinders[0].crankOffsetDegrees - 90.0) < 0.01,
                "JSON round trip must preserve custom crankOffsetDegrees");
        const enginelab::YamlEngineSerializer yamlSer;
        const auto yamlRt = yamlSer.decode(yamlSer.encode(customCrankConfig));
        require(yamlRt && std::abs(yamlRt.config->cylinders[1].crankOffsetDegrees - 270.0) < 0.01,
                "YAML round trip must preserve custom crankOffsetDegrees");
    }

    {
        enginelab::EngineState lowSpeed;
        lowSpeed.rpm = 2'000.0;
        lowSpeed.manifoldPressureKpa = 95.0;
        lowSpeed.coolantTemperatureC = 20.0;
        auto highSpeed = lowSpeed;
        highSpeed.rpm = 6'000.0;
        enginelab::EcuCommand enriched;
        enriched.targetAirFuelRatio = 14.2;
        enriched.fuelCorrection = 1.12;
        const auto lowHeat = physics.evaluateCombustion(config, lowSpeed, { true, false, 0.9, 0.8 }, enriched, 104.0);
        const auto highHeat = physics.evaluateCombustion(config, highSpeed, { true, false, 0.9, 0.8 }, enriched, 104.0);
        require(lowHeat.actualAirFuelRatio < enriched.targetAirFuelRatio - 1.0,
                "reported AFR must include warm-up fuel correction");
        require(highHeat.heatPowerKw > lowHeat.heatPowerKw * 2.3,
                "thermal power must scale with combustion cycles per second");
        enriched.fuelEnabled = false;
        const auto disabled = physics.evaluateCombustion(config, highSpeed, {}, enriched, 104.0);
        require(!disabled.combustionEnabled && disabled.pressureEstimateBar == 0.0,
                "disabled combustion must not report fake cylinder pressure");
    }

    {
        auto openConfig = config;
        openConfig.exhaust.collectorDiameterMm = 100.0;
        openConfig.exhaust.outletDiameterMm = 100.0;
        openConfig.exhaust.mufflerRestriction = 0.0;
        auto restrictedConfig = config;
        restrictedConfig.exhaust.collectorDiameterMm = 25.0;
        restrictedConfig.exhaust.outletDiameterMm = 25.0;
        restrictedConfig.exhaust.mufflerRestriction = 1.0;
        const auto openExhaust = enginelab::ExhaustGraph::makeForEngine(openConfig);
        const auto restrictedExhaust = enginelab::ExhaustGraph::makeForEngine(restrictedConfig);
        enginelab::EngineState flowState;
        flowState.rpm = 6'000.0;
        flowState.fuelFlowGramsPerSecond = 18.0;
        require(restrictedExhaust.backPressureKpa(flowState) > openExhaust.backPressureKpa(flowState) + 20.0,
                "collector and outlet diameters must materially change back pressure");

        auto arbitraryIds = config;
        arbitraryIds.cylinders[0].id = 100;
        arbitraryIds.cylinders[1].id = 200;
        arbitraryIds.cylinders[2].id = 300;
        arbitraryIds.cylinders[3].id = 400;
        arbitraryIds.firingOrder = { 100, 300, 400, 200 };
        auto arbitraryGraph = enginelab::ExhaustGraph::makeForEngine(arbitraryIds);
        enginelab::FiringEvent pathEvent;
        pathEvent.exhaustPortId = 100;
        pathEvent.intensity = 1.0F;
        arbitraryGraph.process(pathEvent);
        require(pathEvent.exhaustDelaySeconds > static_cast<float>(arbitraryIds.exhaust.primaryLengthMm / 520'000.0),
                "exhaust propagation must traverse the complete path for arbitrary cylinder IDs");
    }

    {
        auto manyCylinderConfig = config;
        manyCylinderConfig.name = "32 cylinder event stress";
        manyCylinderConfig.cylinders.clear();
        manyCylinderConfig.firingOrder.clear();
        manyCylinderConfig.rotatingInertiaKgM2 = 1.5;
        for (std::uint32_t index = 0; index < 32; ++index) {
            auto cylinder = config.cylinders.front();
            cylinder.id = 100U + index;
            manyCylinderConfig.cylinders.push_back(cylinder);
            manyCylinderConfig.firingOrder.push_back(cylinder.id);
        }
        require(!enginelab::validateEngineConfig(manyCylinderConfig), "32-cylinder stress configuration must be valid");
        enginelab::FourStrokeEventGenerator stressGenerator;
        enginelab::EngineState stressState;
        stressState.rpm = 20'000.0;
        stressState.throttle = 1.0;
        stressState.cylinderStateCount = 32;
        for (std::size_t index = 0; index < 32; ++index)
            stressState.cylinderStates[index].id = manyCylinderConfig.cylinders[index].id;
        enginelab::EcuCommand stressCommand;
        enginelab::CombustionResult stressCombustion;
        stressCombustion.combustionQuality = 1.0;
        stressCombustion.actualAirFuelRatio = 13.0;
        std::array<enginelab::FiringEvent, enginelab::maxEventsPerSimulationStep> stressEvents {};
        const auto stressCount = stressGenerator.generate(manyCylinderConfig, stressState, stressCommand,
            stressCombustion, 0.0, 0.0, 6'000.0, 0.05, stressEvents);
        std::set<std::uint32_t> stressIds;
        for (std::size_t index = 0; index < stressCount; ++index) stressIds.insert(stressEvents[index].cylinderId);
        require(stressCount < stressEvents.size() && stressIds.size() == 32,
                "validated worst-case event generation must fit and preserve every cylinder");
        std::array<enginelab::FiringEvent, 4> tinyEventBuffer {};
        const auto tinyCount = stressGenerator.generate(manyCylinderConfig, stressState, stressCommand,
            stressCombustion, 0.0, 0.0, 6'000.0, 0.05, tinyEventBuffer);
        require(tinyCount == tinyEventBuffer.size() && stressGenerator.droppedEventCountLastGenerate() > 0,
                "caller-provided event buffer overflow must be explicitly observable");
        for (std::size_t index = 1; index < tinyCount; ++index)
            require(tinyEventBuffer[index - 1].timeSeconds <= tinyEventBuffer[index].timeSeconds,
                    "truncated event output must retain the earliest chronological events");
    }

    {
        enginelab::FiringEventQueue audioQueue;
        enginelab::RealtimeAudioState audioState;
        enginelab::RealtimeEngineAudio renderer(audioQueue, audioState);
        renderer.prepare(48'000.0, 1'200);
        enginelab::FiringEvent event;
        event.timeSeconds = 0.0;
        event.intensity = 0.8F;
        event.pressureEstimateBar = 60.0F;
        event.combustionDurationMs = 5.0F;
        require(audioQueue.tryPush(event), "audio timing event must enter realtime queue");
        juce::AudioBuffer<float> buffer(2, 1'200);
        renderer.render(buffer, 0, buffer.getNumSamples());
        int firstAudible = -1;
        for (int sample = 0; sample < buffer.getNumSamples(); ++sample)
            if (std::abs(buffer.getSample(0, sample)) > 1.0e-7F) { firstAudible = sample; break; }
        require(firstAudible >= 955 && firstAudible <= 970,
                "audio event must be rendered at its sample-accurate scheduled offset");
    }

    {
        enginelab::FiringEventQueue scaledQueue;
        enginelab::RealtimeAudioState scaledState;
        scaledState.rpm.store(3'000.0F);
        scaledState.timeScale.store(1.0F);
        enginelab::RealtimeEngineAudio scaledRenderer(scaledQueue, scaledState);
        scaledRenderer.prepare(48'000.0, 4'800);
        juce::AudioBuffer<float> audible(2, 4'800);
        scaledRenderer.render(audible, 0, audible.getNumSamples());
        require(audible.getMagnitude(0, 0, audible.getNumSamples()) > 1.0e-5F,
                "mechanical audio layer must follow running RPM");

        enginelab::FiringEventQueue pausedQueue;
        enginelab::RealtimeAudioState pausedState;
        pausedState.rpm.store(3'000.0F);
        pausedState.timeScale.store(0.0F);
        enginelab::RealtimeEngineAudio pausedRenderer(pausedQueue, pausedState);
        pausedRenderer.prepare(48'000.0, 4'800);
        juce::AudioBuffer<float> silent(2, 4'800);
        pausedRenderer.render(silent, 0, silent.getNumSamples());
        require(silent.getMagnitude(0, 0, silent.getNumSamples()) < 1.0e-7F,
                "paused time scale must silence continuous mechanical layers");

        enginelab::FiringEvent overflowEvent;
        overflowEvent.intensity = 0.8F;
        overflowEvent.exhaustDelaySeconds = 0.01F;
        for (int index = 0; index < 600; ++index) {
            overflowEvent.timeSeconds = static_cast<double>(index) * 0.0001;
            require(scaledQueue.tryPush(overflowEvent), "audio overflow fixture must enter queue");
        }
        juce::AudioBuffer<float> shortBuffer(2, 64);
        scaledRenderer.render(shortBuffer, 0, shortBuffer.getNumSamples());
        require(scaledRenderer.droppedPendingEventCount() > 0,
                "pending audio saturation must be observable instead of blocking the producer queue");
    }

    {
        enginelab::SpscQueue<int, 8> saturationQueue;
        for (int index = 0; index < 7; ++index) require(saturationQueue.tryPush(index), "SPSC must accept usable capacity");
        require(!saturationQueue.tryPush(8), "SPSC must report saturation without overwriting unread data");
        for (int index = 0; index < 7; ++index) {
            int item = -1;
            require(saturationQueue.tryPop(item) && item == index, "SPSC saturation must preserve FIFO ordering");
        }
    }

    const auto finiteFrame = simulator.step(std::numeric_limits<double>::quiet_NaN(),
        { true, false, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN() });
    require(std::isfinite(finiteFrame.state.rpm) && std::isfinite(finiteFrame.state.throttle)
            && std::isfinite(finiteFrame.state.load), "non-finite controls must not poison simulation state");

    {
        auto invalidConfig = config;
        invalidConfig.firingOrder.back() = 99;
        bool rejected = false;
        try {
            enginelab::EngineSimulator invalidSimulator(invalidConfig, ecu, physics, events, exhaust);
        } catch (const std::invalid_argument&) { rejected = true; }
        require(rejected, "simulator must reject firing orders that reference unknown cylinders");
    }

    {
        enginelab::EngineRuntime runtime(enginelab::makeDefaultInlineTwo());
        runtime.setIgnitionEnabled(false);
        runtime.setThrottle(0.31);
        runtime.setLoad(0.27);
        runtime.start(); runtime.startDyno();
        std::this_thread::sleep_for(std::chrono::milliseconds(4'500));
        const auto liveRun = runtime.currentDynoRun();
        if (liveRun.points.size() < 3)
            std::cerr << "dyno diagnostic: points=" << liveRun.points.size() << " rpm=" << runtime.snapshot().rpm << '\n';
        runtime.stopDyno();
        std::this_thread::sleep_for(std::chrono::milliseconds(750));
        const auto restored = runtime.snapshot();
        runtime.setPaused(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        const auto pausedAt = runtime.snapshot().simulationTimeSeconds;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        require(std::abs(runtime.snapshot().simulationTimeSeconds - pausedAt) < 1.0e-9,
                "runtime pause must freeze deterministic simulation time");
        runtime.stop();
        const auto runs = runtime.dynoHistory();
        require(runs.size() == 1 && runs.front().points.size() >= 3, "brake dyno must record a usable curve");
        require(runs.front().peakTorqueNm > 0.0 && runs.front().peakPowerKw > 0.0,
                "dyno must calculate torque and power");
        require(runs.front().peakCorrectedTorqueNm > 0.0 && runs.front().peakCorrectedPowerKw > 0.0,
                "dyno must publish corrected peaks explicitly");
        require(runs.front().points.front().atmosphericCorrectionFactor > 0.0
                && runs.front().points.front().correctedPowerKw > 0.0,
                 "dyno must publish atmospheric correction and corrected output");
        require(runs.front().points.front().volumetricEfficiency > 0.0
                && runs.front().points.front().manifoldPressureKpa > 0.0
                && runs.front().points.front().oilPressureKpa > 0.0
                && runs.front().points.front().airFlowGramsPerSecond > 0.0
                && runs.front().points.front().lambda > 0.0,
                "dyno points must retain the physical telemetry needed to explain a result");
        require(std::abs(restored.throttle - 0.31) < 0.01 && std::abs(restored.load - 0.27) < 0.001,
                "dyno must restore the user's manual throttle and load");
    }
    std::cout << "EngineLab core tests passed\n";
    return EXIT_SUCCESS;
}
