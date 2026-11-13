// A pass-by run's scripted driver (PassByDriver) on a real engine and
// driveline (a K20, its car): from a standstill at the start it stages the
// car in first, launches at full throttle through the gears (on launch
// control when the tables carry it), passes the microphone and brakes to a
// stop past the end, the sound fading out; its place along the lane is the
// distance the car really covered. The constant-speed program holds its
// speed until its full-throttle point; a held gear is never left; the lift
// program lifts at the microphone.

#include <enginelab/audio/RealtimeEngineAudio.hpp>
#include <enginelab/calibration/CalibrationTypes.hpp>
#include <enginelab/calibration/EcuCalibration.hpp>
#include <enginelab/calibration/EcuCalibrationKeys.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/runtime/DriverCommandArbiter.hpp>
#include <enginelab/runtime/DrivelineModel.hpp>
#include <enginelab/runtime/EngineRuntime.hpp>
#include <enginelab/runtime/PassByRun.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <chrono>
#include <thread>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <variant>

namespace {

using namespace enginelab;

void require(bool condition, const std::string& message) {
    if (condition) return;
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
}

constexpr double frameSeconds = 1.0 / 240.0;

EngineConfig catalogueEngine(const std::string& name) {
    const auto catalog = loadEngineCatalog(ENGINELAB_CATALOG_ROOT);
    for (const auto& entry : catalog.entries)
        if (entry.config.name.find(name) != std::string::npos) return entry.config;
    require(false, "the " + name + " is in the catalogue");
    return {};
}

struct Car final {
    explicit Car(EngineConfig engine)
        : config(std::move(engine)), exhaust(ExhaustGraph::makeForEngine(config)),
          simulator(std::make_unique<EngineSimulator>(config, ecu, physics, events, exhaust)),
          driveline(std::make_unique<DrivelineModel>(config)) {
        config = simulator->config();
        ecu.initialiseCalibration(config);
    }
    EngineConfig config;
    SimpleEcuModel ecu;
    SimplifiedGasolinePhysics physics;
    FourStrokeEventGenerator events;
    ExhaustGraph exhaust;
    std::unique_ptr<EngineSimulator> simulator;
    std::unique_ptr<DrivelineModel> driveline;
    DrivelineOutput last {};

    void setScalar(std::string_view key, double value) {
        auto draft = calibration::makeDraft(*ecu.calibrationStore()->snapshot());
        const auto* found = draft.find(key);
        require(found != nullptr, "the tables carry " + std::string(key));
        auto entry = *found;
        std::get<calibration::ScalarCalibration>(entry).value = value;
        draft.set(std::move(entry));
        require(ecu.calibrationStore()->publish(draft).published, std::string(key) + " publishes");
    }

    PassByVehicle vehicle(bool launchControl) const {
        PassByVehicle out;
        out.idleRpm = config.idleRpm;
        out.ceilingRpm = std::min(config.redlineRpm, config.ignition.revLimitRpm);
        out.gearCount = static_cast<int>(config.transmission.gearRatios.size());
        out.automaticGearbox = false;
        out.launchControlFitted = launchControl;
        return out;
    }

    /** One frame driven by the pass-by driver. */
    PassByCommand step(PassByDriver& driver) {
        const auto& previous = simulator->state();
        PassByObservation observation;
        observation.speedMps = last.vehicleSpeedMps;
        observation.rpm = previous.rpm;
        observation.requestedGear = driveline->requestedGear();
        observation.engagedGear = last.engagedGear;
        observation.shiftInProgress = last.shiftInProgress;
        const auto command = driver.step(frameSeconds, observation);
        if (command.requestGear) driveline->requestGear(*command.requestGear);
        if (command.shiftUp) driveline->shiftUp();
        const auto vehicleStep = advanceVehicle(*driveline, frameSeconds, previous, 0.0, command.inputs);
        last = vehicleStep.drive;
        EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = command.starter;
        applyVehicleStep(controls, vehicleStep);
        static_cast<void>(simulator->step(frameSeconds, controls));
        return command;
    }
};

struct Trace final {
    double distanceAtStartM { -1.0 };
    double distanceAtEndM { 0.0 };
    double positionAtEndM { 0.0 };
    double speedAtMicrophoneKmh { -1.0 };
    double secondsToMicrophone { -1.0 };
    double peakRpm { 0.0 };
    double stagingPeakRpm { 0.0 };
    double stagingLateLowRpm { 1.0e9 };
    int highestGear { -1 };
    double soundGainAtEnd { 1.0 };
    bool sawEachPhase[7] {};
    double cruiseSpeedErrorMaxKmh { 0.0 };
    double throttleBeforeFullMax { 0.0 };
    double throttleAfterFullMin { 1.0 };
    double throttleAfterMicrophoneMax { 0.0 };
    double seconds { 0.0 };
};

Trace drive(Car& car, PassByDriver& driver, double limitSeconds) {
    Trace trace;
    const auto& settings = driver.settings();
    for (double t = 0.0; t < limitSeconds && driver.driving(); t += frameSeconds) {
        const auto phaseBefore = driver.phase();
        const auto command = car.step(driver);
        const auto phase = driver.phase();
        trace.sawEachPhase[static_cast<int>(phase)] = true;
        const auto rpm = car.simulator->state().rpm;
        if (phaseBefore == PassByPhase::staging && phase != PassByPhase::staging)
            trace.distanceAtStartM = car.last.vehicleDistanceM;
        if (phaseBefore == PassByPhase::runUp && phase == PassByPhase::measuring)
            trace.distanceAtStartM = car.last.vehicleDistanceM;
        if (phase == PassByPhase::staging) {
            trace.stagingPeakRpm = std::max(trace.stagingPeakRpm, rpm);
            // The last half second of the rev-up: held, not climbing past.
            // (staging lasts PassByDriver::stagingSeconds)
        }
        if (phase == PassByPhase::measuring || phase == PassByPhase::finishing) {
            trace.peakRpm = std::max(trace.peakRpm, rpm);
            trace.highestGear = std::max(trace.highestGear, car.last.engagedGear);
            if (trace.speedAtMicrophoneKmh < 0.0 && driver.positionM() >= 0.0) {
                trace.speedAtMicrophoneKmh = car.last.vehicleSpeedMps * 3.6;
                trace.secondsToMicrophone = driver.runSeconds();
            }
        }
        // A frame the driver spent all in the program (the first one's
        // command is still the run-up's).
        if (phase == PassByPhase::measuring && phaseBefore == PassByPhase::measuring) {
            const auto position = driver.positionM();
            if (settings.program == PassByProgram::acceleration) {
                // Settled cruise, well before the full-throttle point.
                if (position < -settings.fullThrottleBeforeM - 5.0) {
                    trace.throttleBeforeFullMax = std::max(trace.throttleBeforeFullMax, command.inputs.throttle);
                    trace.cruiseSpeedErrorMaxKmh = std::max(trace.cruiseSpeedErrorMaxKmh,
                        std::abs(car.last.vehicleSpeedMps * 3.6 - settings.approachSpeedKmh));
                } else if (position > -settings.fullThrottleBeforeM + 0.5) {
                    trace.throttleAfterFullMin = std::min(trace.throttleAfterFullMin, command.inputs.throttle);
                }
            }
            if (settings.program == PassByProgram::liftAtMicrophone) {
                if (position < -1.0) trace.throttleAfterFullMin = std::min(trace.throttleAfterFullMin, command.inputs.throttle);
                if (position > 1.0)
                    trace.throttleAfterMicrophoneMax = std::max(trace.throttleAfterMicrophoneMax, command.inputs.throttle);
            }
        }
        if (phaseBefore == PassByPhase::measuring) {
            trace.distanceAtEndM = car.last.vehicleDistanceM;
            trace.positionAtEndM = driver.positionM();
        }
        trace.soundGainAtEnd = driver.soundGain();
        trace.seconds = t;
    }
    return trace;
}

void standingStart() {
    Car car(catalogueEngine("K20"));
    PassByRunSettings settings;
    settings.startDistanceM = 150.0;
    settings.endDistanceM = 60.0;
    PassByDriver driver;
    driver.start(settings, car.vehicle(false));
    require(driver.positionM() == -150.0, "the car stands at the start, 150 m before the microphone");
    const auto trace = drive(car, driver, 90.0);
    std::cout << "standing start, no launch control: microphone at " << trace.speedAtMicrophoneKmh << " km/h after "
              << trace.secondsToMicrophone << " s, peak " << trace.peakRpm << " rpm, gear " << trace.highestGear + 1
              << ", staging peak " << trace.stagingPeakRpm << " rpm, finished after " << trace.seconds << " s"
              << std::endl;
    require(driver.phase() == PassByPhase::finished, "the run finishes: stopped past the end");
    for (const auto phase : { PassByPhase::stopping, PassByPhase::staging, PassByPhase::measuring,
                              PassByPhase::finishing, PassByPhase::finished })
        require(trace.sawEachPhase[static_cast<int>(phase)], "a standing start goes through every phase but the run-up");
    require(!trace.sawEachPhase[static_cast<int>(PassByPhase::runUp)], "a standing start has no run-up");
    require(trace.speedAtMicrophoneKmh > 60.0, "it passes the microphone accelerating hard (over 60 km/h)");
    require(trace.highestGear >= 1, "it shifts up through the gears");
    const auto ceiling = std::min(car.config.redlineRpm, car.config.ignition.revLimitRpm);
    require(trace.peakRpm < ceiling + 300.0, "it shifts before the limiter");
    require(trace.stagingPeakRpm > 0.3 * ceiling && trace.stagingPeakRpm < 0.55 * ceiling,
            "without launch control the driver holds the revs near 40 % of the ceiling on the line");
    const auto covered = trace.distanceAtEndM - trace.distanceAtStartM;
    require(std::abs(trace.positionAtEndM + settings.startDistanceM - covered) < 0.5,
            "its place along the lane is the distance the car covered");
    require(trace.positionAtEndM >= settings.endDistanceM, "it drives to the end");
    require(driver.positionM() > settings.endDistanceM, "it stops past the end");
    require(trace.soundGainAtEnd == 0.0, "the sound has faded out");
    require(car.driveline->requestedGear() == -1, "stopped, it is left in neutral");
    require(std::abs(car.last.vehicleSpeedMps) < 0.3, "and standing");
}

void launchControl() {
    Car car(catalogueEngine("K20"));
    car.setScalar(calibration::keys::launchLimit, 4'000.0);
    PassByRunSettings settings;
    settings.startDistanceM = 40.0;
    settings.endDistanceM = 20.0;
    PassByDriver driver;
    driver.start(settings, car.vehicle(true));
    const auto trace = drive(car, driver, 60.0);
    std::cout << "standing start on launch control at 4,000 rpm: staging peak " << trace.stagingPeakRpm
              << " rpm, microphone at " << trace.speedAtMicrophoneKmh << " km/h" << std::endl;
    require(driver.phase() == PassByPhase::finished, "a launch-control run finishes");
    require(trace.stagingPeakRpm > 3'500.0 && trace.stagingPeakRpm < 4'500.0,
            "foot down on the line, launch control holds the engine at its speed");
}

void accelerationProgram() {
    Car car(catalogueEngine("K20"));
    PassByRunSettings settings;
    settings.program = PassByProgram::acceleration;
    settings.startDistanceM = 80.0;
    settings.endDistanceM = 30.0;
    settings.approachSpeedKmh = 50.0;
    settings.fullThrottleBeforeM = 10.0;
    settings.gear = 2;
    PassByDriver driver;
    driver.start(settings, car.vehicle(false));
    const auto trace = drive(car, driver, 120.0);
    std::cout << "50 km/h in second, full throttle 10 m before: cruise error up to " << trace.cruiseSpeedErrorMaxKmh
              << " km/h, throttle before " << trace.throttleBeforeFullMax << ", microphone at "
              << trace.speedAtMicrophoneKmh << " km/h, highest gear " << trace.highestGear + 1 << std::endl;
    require(driver.phase() == PassByPhase::finished, "an acceleration run finishes");
    require(trace.sawEachPhase[static_cast<int>(PassByPhase::runUp)], "it gets to its speed before the start");
    require(trace.cruiseSpeedErrorMaxKmh < 2.0, "it arrives at 50 km/h within 2 km/h");
    require(trace.throttleBeforeFullMax < 0.6, "at part throttle before the full-throttle point");
    require(trace.throttleAfterFullMin == 1.0, "and at full throttle from it");
    require(trace.speedAtMicrophoneKmh > 52.0, "it is accelerating at the microphone");
    require(trace.highestGear == 1, "a held second gear is never left");
}

void liftProgram() {
    Car car(catalogueEngine("K20"));
    PassByRunSettings settings;
    settings.program = PassByProgram::liftAtMicrophone;
    settings.startDistanceM = 60.0;
    settings.endDistanceM = 40.0;
    settings.approachSpeedKmh = 40.0;
    PassByDriver driver;
    driver.start(settings, car.vehicle(false));
    const auto trace = drive(car, driver, 120.0);
    std::cout << "lift at the microphone from 40 km/h: microphone at " << trace.speedAtMicrophoneKmh << " km/h"
              << std::endl;
    require(driver.phase() == PassByPhase::finished, "a lift run finishes");
    require(trace.throttleAfterFullMin == 1.0, "full throttle up to the microphone");
    require(trace.throttleAfterMicrophoneMax == 0.0, "lifted past it");
}

void settingsHeld() {
    PassByRunSettings wild;
    wild.startDistanceM = 1.0e9;
    wild.endDistanceM = -5.0;
    wild.approachSpeedKmh = std::nan("");
    wild.fullThrottleBeforeM = 1.0e6;
    wild.gear = 40;
    const auto held = sanitised(wild);
    require(held.startDistanceM == PassByRunSettings::maximumDistanceM, "the start is held to 2 km");
    require(held.endDistanceM == PassByRunSettings::minimumDistanceM, "the end to 20 m at least");
    require(held.approachSpeedKmh == 50.0, "a non-finite speed is its default");
    require(held.fullThrottleBeforeM == held.startDistanceM, "the full-throttle point lies within the run");
    require(held.gear == 12, "a gear past the gearbox is held");
}

/** The running application's path: EngineRuntime's own thread drives the
 *  run, and what the audio reads (RealtimeAudioState, the trackside
 *  microphone's only input) says a run is on and where its car is. */
void runtimePublishesTheRun() {
    auto runtime = std::make_unique<EngineRuntime>(catalogueEngine("K20"));
    runtime->start();
    PassByRunSettings settings;
    settings.startDistanceM = 300.0;
    runtime->startPassByRun(settings);
    auto& audio = runtime->audioState();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
    auto phase = 0;
    auto position = 0.0F;
    while (std::chrono::steady_clock::now() < deadline) {
        phase = audio.passByPhase.load(std::memory_order_relaxed);
        position = audio.passByPositionM.load(std::memory_order_relaxed);
        if (phase != 0 && position == -300.0F) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    runtime->stopPassByRun();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    const auto after = audio.passByPhase.load(std::memory_order_relaxed);
    runtime->stop();
    std::cout << "runtime: the audio saw phase " << phase << " at " << position << " m, then " << after
              << " once stopped" << std::endl;
    require(phase != 0, "the audio is told a run is on");
    require(position == -300.0F, "and that its car stands 300 m before the microphone");
    require(after == 0, "stopped, the audio is told the run is off");
}

/** The renderer takes a published run: its microphone hears the car where
 *  the run puts it, and lets it loop again once the run is off. */
void rendererFollowsTheRun() {
    auto runtime = std::make_unique<EngineRuntime>(catalogueEngine("K20"));
    FiringEventQueue events;
    auto renderer = std::make_unique<RealtimeEngineAudio>(events, runtime->audioState(), nullptr,
        &runtime->exhaustGraph(), &runtime->engineConfig(), &runtime->exhaustAcousticSamples());
    renderer->prepare(48'000.0, 256);
    juce::AudioBuffer<float> block(2, 256);
    auto& audio = runtime->audioState();
    audio.passByPhase.store(static_cast<int>(PassByPhase::staging), std::memory_order_relaxed);
    audio.passByPositionM.store(-300.0F, std::memory_order_relaxed);
    audio.passBySoundGain.store(1.0F, std::memory_order_relaxed);
    for (auto index = 0; index < 8; ++index) {
        block.clear();
        renderer->render(block, 0, 256);
    }
    std::cout << "renderer: the microphone's car at " << renderer->passByPositionM() << " m" << std::endl;
    require(std::abs(renderer->passByPositionM() + 300.0) < 1.0, "the renderer's car is the run's, 300 m away");
    require(!renderer->passByEnabled(), "without the menu's microphone: the run alone switched it on");
}

} // namespace

int main() {
    runtimePublishesTheRun();
    rendererFollowsTheRun();
    settingsHeld();
    standingStart();
    launchControl();
    accelerationProgram();
    liftProgram();
    std::cout << "PASS: the pass-by driver stages, launches, holds its speed or its gear, lifts at the microphone, "
                 "covers the lane to its end and stops there"
              << std::endl;
    return 0;
}
