// Live tuning: the running engine takes a new flywheel and new camshafts
// without a restart. These tests drive the real simulator the way
// EngineRuntime does, deterministically.
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/runtime/DrivelineModel.hpp>
#include <enginelab/runtime/EngineRuntime.hpp>
#include <enginelab/serialization/EngineEditScope.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>
#include <enginelab/simulation/EngineThermalModel.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>

namespace enginelab::tests {
namespace {

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

constexpr double frameSeconds = 1.0 / 240.0;

struct Rig final {
    explicit Rig(EngineConfig engine)
        : config(std::move(engine)), exhaust(ExhaustGraph::makeForEngine(config)),
          simulator(std::make_unique<EngineSimulator>(config, ecu, physics, events, exhaust)) {}
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

/** Every crankshaft, and so the flywheel, `factor` times heavier. */
[[nodiscard]] EngineConfig flywheelTimes(EngineConfig config, double factor) {
    for (auto& crankshaft : config.crankshafts) crankshaft.momentOfInertiaKgM2 *= factor;
    config.rotatingInertiaKgM2 *= factor;
    normaliseEngineConfig(config);
    return config;
}

/** Cranked to 1.5 s, idling to 4 s, then full throttle with no load. */
[[nodiscard]] EngineControls freeRev(double timeSeconds) {
    EngineControls controls;
    controls.ignitionEnabled = true;
    controls.starterEngaged = timeSeconds < 1.5;
    controls.throttle = timeSeconds < 4.0 ? 0.0 : 1.0;
    return controls;
}

/** A flywheel taken at 3 s, at idle: the free rev from 4 s climbs like an
 * engine built with that flywheel. */
void flywheelRevsLikeARestart() {
    for (const auto* name : { "Yamaha CP2", "2JZ" }) {
        const auto original = catalogueEngine(name);
        const auto edited = flywheelTimes(original, 2.0);
        require(EngineSimulator::settingsReplaceable(original, edited), std::string(name) + ": replaceable");
        Rig built(edited);
        Rig swapped(original);
        Rig untouched(original);
        const auto target = 0.75 * original.redlineRpm;
        std::array<double, 3> reached { -1.0, -1.0, -1.0 };
        double minimumRpm = 1.0e9;
        for (auto frame = 0; frame < 8 * 240; ++frame) {
            const auto t = frame * frameSeconds;
            if (frame == 3 * 240) {
                auto incoming = edited;
                require(swapped.simulator->replaceSettings(incoming), std::string(name) + ": the flywheel is taken");
            }
            const auto controls = freeRev(t);
            std::size_t index = 0;
            for (auto* rig : { &built, &swapped, &untouched }) {
                const auto& state = rig->simulator->step(frameSeconds, controls).state;
                if (t >= 4.0 && reached[index] < 0.0 && state.rpm >= target) reached[index] = t - 4.0;
                if (index == 1 && t >= 3.0 && t < 4.0) minimumRpm = std::min(minimumRpm, state.rpm);
                ++index;
            }
        }
        std::cout << "  " << name << ", flywheel x2: free rev to " << target << " rpm in built "
                  << reached[0] << " s, swapped " << reached[1] << " s, untouched " << reached[2]
                  << " s; lowest idle after the change " << minimumRpm << " rpm\n";
        require(reached[0] > 0.0 && reached[1] > 0.0 && reached[2] > 0.0,
                std::string(name) + ": every engine reaches the target");
        require(reached[0] > 1.3 * reached[2],
                std::string(name) + ": the heavier flywheel slows the rev (else this proves nothing)");
        require(minimumRpm > 0.5 * original.idleRpm, std::string(name) + ": the engine keeps idling");
        require(std::abs(reached[1] - reached[0]) <= 0.15 * (reached[0] - reached[2]),
                std::string(name) + ": the changed engine revs like one built with the flywheel");
    }
}

/** Another crankshaft layout is refused; the runtime and the edit scope
 * treat a flywheel as a setting. */
void flywheelIsASetting() {
    const auto engine = catalogueEngine("2JZ");
    const auto heavier = flywheelTimes(engine, 1.5);
    const auto scope = engineEditScope(engine, heavier);
    require(scope.settings && !scope.other && !scope.cylinders && !scope.exhaust && !scope.intake,
            "a flywheel edit is a settings edit only");
    auto turned = engine;
    turned.crankshafts.front().rotationRatio = 2.0;
    require(!EngineSimulator::settingsReplaceable(engine, turned), "another crankshaft ratio is refused");
    auto runtime = std::make_unique<EngineRuntime>(engine);
    require(runtime->applyLiveSettings(heavier), "the runtime takes a heavier flywheel");
    require(std::abs(runtime->engineConfig().rotatingInertiaKgM2 - heavier.rotatingInertiaKgM2) < 1.0e-12,
            "the runtime's configuration carries the new flywheel");
    require(!runtime->applyLiveSettings(turned), "the runtime refuses another crankshaft");
}

/** Cranked to 1.5 s, then full throttle from 2 s held at 60 % of redline by
 * a proportional brake: the settled brake torque is what a cam changes. */
[[nodiscard]] EngineControls heldAt(double timeSeconds, double rpm, const EngineConfig& config) {
    EngineControls controls;
    controls.ignitionEnabled = true;
    controls.starterEngaged = timeSeconds < 1.5;
    controls.throttle = timeSeconds < 2.0 ? 0.0 : 1.0;
    if (timeSeconds >= 2.0)
        controls.dynamometerTorqueNm = std::max(0.0, 2.0 * (rpm - 0.6 * config.redlineRpm));
    return controls;
}

/** Every camshaft, high profiles too, milder: 40 degrees shorter and 30 %
 * less lift on both valves. */
[[nodiscard]] EngineConfig milderCams(EngineConfig config) {
    const auto soften = [](CamshaftConfig& cams) {
        cams.intakeDurationDegrees -= 40.0;
        cams.exhaustDurationDegrees -= 40.0;
        cams.highIntakeDurationDegrees -= 40.0;
        cams.highExhaustDurationDegrees -= 40.0;
        cams.intakeLiftMm *= 0.7;
        cams.exhaustLiftMm *= 0.7;
        cams.highIntakeLiftMm *= 0.7;
        cams.highExhaustLiftMm *= 0.7;
    };
    soften(config.camshafts);
    for (auto& bank : config.banks) soften(bank.camshafts);
    normaliseEngineConfig(config);
    return config;
}

[[nodiscard]] const CamshaftConfig& firstCamshafts(const EngineConfig& config) {
    return config.banks.empty() ? config.camshafts : config.banks.front().camshafts;
}

/** Taking the camshafts it already runs with changes nothing, bit for bit,
 * at once or over a ramp; other banks or a profile switch added are
 * refused; a cam edit is a camshaft edit only. */
void sameCamshaftsAreInvisible() {
    for (const auto* name : { "Yamaha CP2", "K20A" }) {
        const auto config = catalogueEngine(name);
        for (const auto ramp : { 0.0, 1.0 }) {
            Rig reference(config);
            Rig swapped(config);
            double rpm = 0.0;
            for (auto frame = 0; frame < 5 * 240; ++frame) {
                const auto t = frame * frameSeconds;
                if (frame == 3 * 240) {
                    auto incoming = config;
                    require(swapped.simulator->beginCamshaftChange(incoming, ramp),
                            std::string(name) + ": the simulator takes its own camshafts");
                }
                const auto controls = heldAt(t, rpm, config);
                const auto a = reference.simulator->step(frameSeconds, controls);
                const auto b = swapped.simulator->step(frameSeconds, controls);
                rpm = a.state.rpm;
                require(a.state.rpm == b.state.rpm && a.state.crankAngleDegrees == b.state.crankAngleDegrees,
                        std::string(name) + ": the same camshafts must leave the run bit-identical");
            }
        }
    }
    const auto k20 = catalogueEngine("K20A");
    const auto scope = engineEditScope(k20, milderCams(k20));
    require(scope.camshafts && !scope.settings && !scope.cylinders && !scope.other,
            "a cam edit is a camshaft edit only");
    auto switched = k20;
    switched.camshafts.variableProfileEnabled = !switched.camshafts.variableProfileEnabled;
    for (auto& bank : switched.banks) bank.camshafts.variableProfileEnabled = switched.camshafts.variableProfileEnabled;
    require(!EngineSimulator::camshaftsReplaceable(k20, switched), "a profile switch added or removed is refused");
    auto runtime = std::make_unique<EngineRuntime>(k20);
    require(runtime->applyLiveCamshafts(milderCams(k20), 2.0), "the runtime takes milder cams");
    require(firstCamshafts(runtime->engineConfig()).intakeDurationDegrees
                == firstCamshafts(k20).intakeDurationDegrees - 40.0,
            "the runtime's configuration carries the new cams");
    require(!runtime->applyLiveCamshafts(switched, 0.0), "the runtime refuses a profile switch added");
}

/** Milder cams taken at 6 s, at once or over 2 s: no stall, the duration
 * half-way at half the ramp, and the engine settles where an engine built
 * with them runs. */
void camshaftsSettleLikeARestart() {
    for (const auto* name : { "Yamaha CP2", "K20A" }) {
        const auto original = catalogueEngine(name);
        const auto edited = milderCams(original);
        require(EngineSimulator::camshaftsReplaceable(original, edited), std::string(name) + ": replaceable");
        for (const auto ramp : { 0.0, 2.0 }) {
            Rig built(edited);
            Rig swapped(original);
            Rig untouched(original);
            constexpr auto swapFrame = 6 * 240;
            constexpr auto frames = 14 * 240;
            std::array<double, 3> rpm {};
            std::array<double, 3> torque {};
            double minimumRpm = 1.0e9;
            double halfWay = -1.0;
            for (auto frame = 0; frame < frames; ++frame) {
                const auto t = frame * frameSeconds;
                if (frame == swapFrame) {
                    auto incoming = edited;
                    require(swapped.simulator->beginCamshaftChange(incoming, ramp),
                            std::string(name) + ": the cams are taken");
                }
                std::size_t index = 0;
                for (auto* rig : { &built, &swapped, &untouched }) {
                    const auto controls = heldAt(t, rpm[index], original);
                    rpm[index] = rig->simulator->step(frameSeconds, controls).state.rpm;
                    if (frame >= frames - 4 * 240) torque[index] += controls.dynamometerTorqueNm / (4.0 * 240.0);
                    ++index;
                }
                if (frame >= swapFrame) minimumRpm = std::min(minimumRpm, rpm[1]);
                if (ramp > 0.0 && frame == swapFrame + static_cast<int>(0.5 * ramp * 240.0))
                    halfWay = firstCamshafts(swapped.simulator->config()).intakeDurationDegrees
                        - firstCamshafts(original).intakeDurationDegrees;
            }
            std::cout << "  " << name << ", cams -40 deg, -30 % lift " << (ramp > 0.0 ? "over 2 s" : "at once")
                      << ": settled brake torque, Nm: built " << torque[0] << ", swapped " << torque[1]
                      << ", untouched " << torque[2] << "; lowest rpm after the change " << minimumRpm
                      << (ramp > 0.0 ? "; duration at half the ramp " + std::to_string(halfWay) : std::string())
                      << "\n";
            require(std::abs(torque[2] - torque[0]) > 0.03 * std::abs(torque[2]),
                    std::string(name) + ": the cams change the torque (else this proves nothing)");
            require(minimumRpm > 0.5 * original.idleRpm, std::string(name) + ": the engine keeps running");
            require(std::abs(torque[1] - torque[0]) <= 0.15 * std::abs(torque[2] - torque[0]),
                    std::string(name) + ": the changed engine settles where an engine built with the cams runs");
            if (ramp > 0.0)
                require(halfWay < -17.0 && halfWay > -23.0, std::string(name) + ": half-way at half the ramp");
        }
    }
}

/** The air at 2,000 m (79.5 kPa) and 35 degC, taken at 6 s while held at 60 %
 * of redline: no stall, and the engine settles where an engine built there
 * runs. An ambient edit is a settings edit. */
void ambientSettlesLikeARestart() {
    for (const auto* name : { "Yamaha CP2", "2JZ" }) {
        const auto original = catalogueEngine(name);
        auto edited = original;
        edited.ambientPressureKpa = 79.5;
        edited.ambientTemperatureC = 35.0;
        const auto scope = engineEditScope(original, edited);
        require(scope.settings && !scope.other, std::string(name) + ": an ambient edit is a settings edit only");
        Rig built(edited);
        Rig swapped(original);
        Rig untouched(original);
        constexpr auto swapFrame = 6 * 240;
        constexpr auto frames = 14 * 240;
        std::array<double, 3> rpm {};
        std::array<double, 3> torque {};
        double minimumRpm = 1.0e9;
        for (auto frame = 0; frame < frames; ++frame) {
            const auto t = frame * frameSeconds;
            if (frame == swapFrame) {
                auto incoming = edited;
                require(swapped.simulator->replaceSettings(incoming), std::string(name) + ": the air is taken");
            }
            std::size_t index = 0;
            for (auto* rig : { &built, &swapped, &untouched }) {
                const auto controls = heldAt(t, rpm[index], original);
                rpm[index] = rig->simulator->step(frameSeconds, controls).state.rpm;
                if (frame >= frames - 4 * 240) torque[index] += controls.dynamometerTorqueNm / (4.0 * 240.0);
                ++index;
            }
            if (frame >= swapFrame) minimumRpm = std::min(minimumRpm, rpm[1]);
        }
        std::cout << "  " << name << ", 2,000 m and 35 degC: settled brake torque, Nm: built " << torque[0]
                  << ", swapped " << torque[1] << ", untouched " << torque[2] << "; lowest rpm after the change "
                  << minimumRpm << "\n";
        require(std::abs(torque[2] - torque[0]) > 0.03 * std::abs(torque[2]),
                std::string(name) + ": the air changes the torque (else this proves nothing)");
        require(minimumRpm > 0.5 * original.idleRpm, std::string(name) + ": the engine keeps running");
        require(std::abs(torque[1] - torque[0]) <= 0.15 * std::abs(torque[2] - torque[0]),
                std::string(name) + ": the changed engine settles where an engine built in that air runs");
    }
}

/** A wastegate spring works against the atmosphere: up high the turbo holds
 * the same gauge boost, so the manifold loses only the ambient drop. A boost
 * held as a ratio to ambient lost 21 % of it, like an unblown engine. Only
 * the pressure changes; the 2JZ is held at 60 % of redline, above full boost. */
void wastegateHoldsGaugeBoostUpHigh() {
    const auto seaLevel = catalogueEngine("2JZ");
    auto upHigh = seaLevel;
    upHigh.ambientPressureKpa = 79.5;
    Rig low(seaLevel);
    Rig high(upHigh);
    constexpr auto frames = 10 * 240;
    std::array<double, 2> rpm {};
    std::array<double, 2> gaugeKpa {};
    std::array<double, 2> torque {};
    for (auto frame = 0; frame < frames; ++frame) {
        const auto t = frame * frameSeconds;
        std::size_t index = 0;
        for (auto* rig : { &low, &high }) {
            const auto controls = heldAt(t, rpm[index], seaLevel);
            const auto state = rig->simulator->step(frameSeconds, controls).state;
            rpm[index] = state.rpm;
            if (frame >= frames - 3 * 240) {
                gaugeKpa[index] += (state.boostPressureRatio - 1.0) * rig->config.ambientPressureKpa / (3.0 * 240.0);
                torque[index] += controls.dynamometerTorqueNm / (3.0 * 240.0);
            }
            ++index;
        }
    }
    std::cout << "  2JZ held at 60 % of redline: gauge boost at sea level " << gaugeKpa[0] << " kPa, at 79.5 kPa "
              << gaugeKpa[1] << " kPa; brake torque " << torque[0] << " -> " << torque[1] << " Nm\n";
    require(gaugeKpa[0] > 50.0, "the 2JZ is on its wastegate (else this proves nothing)");
    require(std::abs(gaugeKpa[1] - gaugeKpa[0]) <= 0.08 * gaugeKpa[0],
            "up high the wastegate holds the same gauge boost");
}

/** A regulated engine runs at its set temperatures from the start and through
 * a full-load run, as on a dyno cell; one let free starts at the ambient
 * temperature and warms up. Regulating a running cold engine is a settings
 * edit that sets the temperatures at once. */
void regulatedTemperaturesHold() {
    const auto regulated = catalogueEngine("Yamaha CP2");
    require(regulated.thermal.regulated, "the thermal model is regulated by default");
    auto unregulated = regulated;
    unregulated.thermal.regulated = false;
    const auto scope = engineEditScope(unregulated, regulated);
    require(scope.settings && !scope.other, "regulating the temperatures is a settings edit only");
    Rig held(regulated);
    Rig warming(unregulated);
    Rig switched(unregulated);
    constexpr auto frames = 12 * 240;
    std::array<double, 3> rpm {};
    std::array<double, 3> lowestCoolant { 1.0e9, 1.0e9, 1.0e9 };
    std::array<double, 3> highestCoolant {};
    for (auto frame = 0; frame < frames; ++frame) {
        const auto t = frame * frameSeconds;
        if (frame == 4 * 240) {
            auto incoming = regulated;
            require(switched.simulator->replaceSettings(incoming), "the regulation is taken");
            require(switched.simulator->state().coolantTemperatureC == regulated.thermal.regulatedCoolantTemperatureC
                        && switched.simulator->state().oilTemperatureC == regulated.thermal.regulatedOilTemperatureC,
                    "a regulated engine is at its set temperatures at once");
        }
        std::size_t index = 0;
        for (auto* rig : { &held, &warming, &switched }) {
            const auto state = rig->simulator->step(frameSeconds, heldAt(t, rpm[index], regulated)).state;
            rpm[index] = state.rpm;
            if (index != 2 || frame >= 4 * 240) {
                lowestCoolant[index] = std::min(lowestCoolant[index], state.coolantTemperatureC);
                highestCoolant[index] = std::max(highestCoolant[index], state.coolantTemperatureC);
            }
            ++index;
        }
    }
    std::cout << "  Yamaha CP2 at full load for 10 s: coolant regulated " << lowestCoolant[0] << " to "
              << highestCoolant[0] << " degC, free " << lowestCoolant[1] << " to " << highestCoolant[1]
              << " degC, regulated at 4 s " << lowestCoolant[2] << " to " << highestCoolant[2] << " degC\n";
    require(lowestCoolant[0] == 90.0 && highestCoolant[0] == 90.0, "a regulated engine stays at 90 degC");
    require(lowestCoolant[1] < 30.0 && highestCoolant[1] > lowestCoolant[1] + 1.0,
            "a free engine starts cold and warms up (else this proves nothing)");
    require(lowestCoolant[2] == 90.0 && highestCoolant[2] == 90.0, "a regulated running engine stays at 90 degC");
}

/** The realistic temperatures, on the thermal model alone (EngineThermalModel).
 * A CP2 at an idle like the simulated one (2.8 kW released, about 1.2 kW of
 * friction) warms past 80 degC behind
 * its closed thermostat, then its fan holds it; the former model leaked through
 * the closed thermostat and stalled near 37 degC after five dyno passes. At its
 * rated heat it overheats standing, fan on, but holds below 100 degC in the
 * air of 30 m/s; the fan stops at 95 degC; an air-cooled engine has none. */
void realisticTemperaturesFollowHeatAndAir() {
    auto config = catalogueEngine("Yamaha CP2");
    config.thermal.regulated = false;
    const EngineThermalModel model(config);
    const auto ratedHeatKw = model.estimatedRatedPowerKw() / 0.30;
    const auto ratedFrictionKw = 0.15 * model.estimatedRatedPowerKw();
    constexpr double dt = 0.05;
    const auto run = [&](const EngineThermalModel& thermal, EngineThermalModel::State& state, double heatKw, double frictionKw,
                         double airMps, double seconds, auto&& observe) {
        for (auto t = 0.0; t < seconds; t += dt) {
            thermal.advance(state, { heatKw, frictionKw, airMps, 1'300.0, 22.0 }, dt);
            observe(t, state);
        }
    };
    EngineThermalModel::State idling;
    double secondsTo80 = -1.0;
    double hottestIdle = 0.0;
    run(model, idling, 2.8, 1.2, 0.0, 40.0 * 60.0, [&](double t, const EngineThermalModel::State& state) {
        if (secondsTo80 < 0.0 && state.coolantTemperatureC >= 80.0) secondsTo80 = t;
        hottestIdle = std::max(hottestIdle, state.coolantTemperatureC);
    });
    EngineThermalModel::State standing { 90.0, 100.0, false };
    double fanOnAfter = -1.0;
    run(model, standing, ratedHeatKw, ratedFrictionKw, 0.0, 120.0, [&](double t, const EngineThermalModel::State& state) {
        if (fanOnAfter < 0.0 && state.fanOn) fanOnAfter = t;
    });
    const auto standingAfter2Min = standing.coolantTemperatureC;
    double fanOffAt = -1.0;
    run(model, standing, 0.0, 0.0, 0.0, 600.0, [&](double, const EngineThermalModel::State& state) {
        if (fanOffAt < 0.0 && !state.fanOn) fanOffAt = state.coolantTemperatureC;
    });
    EngineThermalModel::State riding { 90.0, 100.0, false };
    double hottestRide = 0.0;
    run(model, riding, ratedHeatKw, ratedFrictionKw, 30.0, 600.0, [&](double, const EngineThermalModel::State& state) {
        hottestRide = std::max(hottestRide, state.coolantTemperatureC);
    });
    auto aircooledConfig = catalogueEngine("Aircooled");
    aircooledConfig.thermal.regulated = false;
    const EngineThermalModel aircooled(aircooledConfig);
    EngineThermalModel::State fins { 90.0, 100.0, false };
    auto aircooledFan = false;
    run(aircooled, fins, aircooled.estimatedRatedPowerKw() / 0.30, 0.15 * aircooled.estimatedRatedPowerKw(), 0.0, 120.0,
        [&](double, const EngineThermalModel::State& state) { aircooledFan = aircooledFan || state.fanOn; });
    std::cout << "  Yamaha CP2 thermal model (" << model.coolantCapacityKjPerC() << " kJ/K, radiator "
              << model.radiatorKwPerC() << " kW/K, rated " << model.estimatedRatedPowerKw() << " kW): idle to 80 degC in "
              << secondsTo80 / 60.0 << " min, hottest idle " << hottestIdle << " degC; rated heat standing: fan on after "
              << fanOnAfter << " s, " << standingAfter2Min << " degC at 2 min, fan off at " << fanOffAt
              << " degC; at 30 m/s hottest " << hottestRide << " degC\n";
    require(secondsTo80 > 0.0 && secondsTo80 < 30.0 * 60.0, "an idling engine warms past 80 degC within 30 min");
    require(hottestIdle <= EngineThermalModel::fanOnC + 0.5, "an idling engine in still air is held by its fan");
    require(fanOnAfter > 0.0 && standingAfter2Min > 105.0, "a standing engine at full power overheats, fan on");
    require(fanOffAt > 0.0 && fanOffAt <= EngineThermalModel::fanOffC + 0.01, "the fan stops at 95 degC");
    require(hottestRide < 100.0, "at 30 m/s the radiator holds full power below 100 degC");
    require(!aircooledFan, "an air-cooled engine has no radiator fan");
}

/** Light knock does not wear the engine: the 2JZ held at full load knocks
 * lightly the whole time, as every catalogue engine does at WOT, and ends with
 * no damage. Counted linearly, it lost 0.3-0.5 % of power for good per dyno
 * pass. */
void lightKnockDoesNotWear() {
    const auto config = catalogueEngine("2JZ");
    Rig rig(config);
    constexpr auto frames = 10 * 240;
    double rpm = 0.0;
    double meanKnock = 0.0;
    double peakKnock = 0.0;
    for (auto frame = 0; frame < frames; ++frame) {
        const auto state = rig.simulator->step(frameSeconds, heldAt(frame * frameSeconds, rpm, config)).state;
        rpm = state.rpm;
        if (frame >= 3 * 240) {
            meanKnock += state.knockLevel / (7.0 * 240.0);
            peakKnock = std::max(peakKnock, state.knockLevel);
        }
    }
    const auto damage = rig.simulator->state().damage;
    std::cout << "  2JZ at full load for 8 s: knock mean " << meanKnock << ", peak " << peakKnock << ", damage "
              << damage << '\n';
    require(meanKnock > 0.1 && peakKnock < 0.45, "the 2JZ knocks lightly at full load (else this proves nothing)");
    require(damage < 1.0e-9, "light knock does not damage the engine");
}

/** A final drive 30 % shorter taken at 1 s, in first gear with the clutch
 * engaged and the engine held at 2,000 rpm: the car settles at the road speed
 * of one built with it. Another gear count is refused; a gearbox edit is a
 * settings edit. */
void gearboxSettlesLikeARestart() {
    const auto original = catalogueEngine("K20A");
    auto edited = original;
    edited.transmission.finalDriveRatio *= 1.3;
    edited.vehicle.massKg += 200.0;
    const auto scope = engineEditScope(original, edited);
    require(scope.settings && !scope.other, "a gearbox edit is a settings edit only");
    EngineState engine;
    engine.rpm = 2'000.0;
    engine.angularVelocityRadPerSecond = engine.rpm * 2.0 * std::numbers::pi / 60.0;
    engine.throttle = 0.5;
    DrivelineModel built(edited);
    DrivelineModel swapped(original);
    DrivelineModel untouched(original);
    std::array<DrivelineOutput, 3> output {};
    std::size_t index = 0;
    for (auto* driveline : { &built, &swapped, &untouched }) {
        driveline->requestGear(0);
        for (int step = 0; step < 4'000; ++step) {
            if (driveline == &swapped && step == 1'000)
                require(swapped.replaceVehicle(edited.transmission, edited.vehicle), "the driveline takes it");
            engine.simulationTimeSeconds = step * 0.001;
            output[index] = driveline->advance(0.001, engine, 0.0, 1.0, 0.0);
        }
        ++index;
    }
    std::cout << "  K20A, final drive x1.3 and +200 kg at 2,000 rpm in first: road speed built "
              << output[0].vehicleSpeedMps << ", swapped " << output[1].vehicleSpeedMps << ", untouched "
              << output[2].vehicleSpeedMps << " m/s\n";
    require(output[2].vehicleSpeedMps > 1.2 * output[0].vehicleSpeedMps,
            "the shorter final drive slows the car (else this proves nothing)");
    require(std::abs(output[1].vehicleSpeedMps - output[0].vehicleSpeedMps)
                <= 0.02 * (output[2].vehicleSpeedMps - output[0].vehicleSpeedMps),
            "the changed driveline settles where one built with it runs");
    auto fewer = original.transmission;
    fewer.gearRatios.pop_back();
    require(!swapped.replaceVehicle(fewer, original.vehicle), "another gear count is refused");
    auto fewerGears = original;
    fewerGears.transmission = fewer;
    require(!EngineSimulator::settingsReplaceable(original, fewerGears), "the simulator refuses it too");
}

} // namespace

void liveTuningRegression() {
    gearboxSettlesLikeARestart();
    ambientSettlesLikeARestart();
    wastegateHoldsGaugeBoostUpHigh();
    regulatedTemperaturesHold();
    realisticTemperaturesFollowHeatAndAir();
    lightKnockDoesNotWear();
    flywheelIsASetting();
    flywheelRevsLikeARestart();
    sameCamshaftsAreInvisible();
    camshaftsSettleLikeARestart();
}

} // namespace enginelab::tests
