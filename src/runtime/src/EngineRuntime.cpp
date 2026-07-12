#include <enginelab/runtime/EngineRuntime.hpp>
#include <algorithm>
#include <chrono>
#include <numbers>
#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <Windows.h>
#endif

namespace enginelab {
EngineRuntime::EngineRuntime(EngineConfig config)
    : config_(std::move(config)), exhaust_(ExhaustGraph::makeForEngine(config_)),
      simulator_(config_, ecu_, physics_, eventGenerator_, exhaust_) {
    simulator_.setPressureSamplingEnabled(true);
    audioState_.cylinderCount.store(static_cast<float>(config_.cylinders.size()), std::memory_order_relaxed);
    const auto displacement = engineDisplacementLitres(config_);
    double boreSum = 0.0;
    double strokeSum = 0.0;
    for (const auto& cylinder : config_.cylinders) {
        boreSum += cylinder.boreMm;
        strokeSum += cylinder.strokeMm;
    }
    const auto cylinderCount = std::max<std::size_t>(1, config_.cylinders.size());
    const auto meanBore = boreSum / static_cast<double>(cylinderCount);
    const auto meanStroke = strokeSum / static_cast<double>(cylinderCount);
    audioState_.redlineRpm.store(static_cast<float>(config_.redlineRpm), std::memory_order_relaxed);
    audioState_.displacementLitres.store(static_cast<float>(displacement), std::memory_order_relaxed);
    audioState_.cylinderDisplacementLitres.store(static_cast<float>(displacement / static_cast<double>(cylinderCount)), std::memory_order_relaxed);
    audioState_.boreStrokeRatio.store(static_cast<float>(meanBore / std::max(1.0, meanStroke)), std::memory_order_relaxed);
    const auto bankSeparation = config_.layout == EngineLayout::vLayout ? 1.0F
        : (config_.layout == EngineLayout::flat ? 0.92F : (config_.layout == EngineLayout::radial ? 0.74F : 0.0F));
    audioState_.bankSeparation.store(bankSeparation, std::memory_order_relaxed);
    audioState_.exhaustOpenness.store(static_cast<float>(std::clamp(
        (config_.exhaust.outletDiameterMm / std::max(20.0, config_.exhaust.collectorDiameterMm))
            * (1.0 - config_.exhaust.mufflerRestriction * 0.72), 0.15, 1.45)), std::memory_order_relaxed);
    audioState_.boostPressureRatio.store(static_cast<float>(config_.forcedInduction.enabled
        ? config_.forcedInduction.pressureRatio : 1.0), std::memory_order_relaxed);
    const auto pathLengthMm = config_.exhaust.primaryLengthMm + 120.0 + 450.0 + 180.0;
    audioState_.exhaustReflectionSeconds.store(static_cast<float>(2.0 * pathLengthMm / 520'000.0),
                                               std::memory_order_relaxed);
}
EngineRuntime::~EngineRuntime() { stop(); }

void EngineRuntime::start() {
    if (!thread_.joinable()) thread_ = std::jthread([this](std::stop_token token) { run(token); });
}
void EngineRuntime::stop() {
    if (thread_.joinable()) { thread_.request_stop(); thread_.join(); }
    if (dynoRunning_.load()) stopDyno();
}
EngineState EngineRuntime::snapshot() const {
    const std::scoped_lock lock(snapshotMutex_);
    return snapshot_;
}

void EngineRuntime::setGear(int gear) noexcept {
    const auto maxGear = static_cast<int>(config_.transmission.gearRatios.size()) - 1;
    gear_.store(std::clamp(gear, -1, maxGear), std::memory_order_relaxed);
}

void EngineRuntime::shiftUp() noexcept {
    setGear(gear_.load(std::memory_order_relaxed) + 1);
}

void EngineRuntime::shiftDown() noexcept {
    setGear(gear_.load(std::memory_order_relaxed) - 1);
}

void EngineRuntime::adjustDynoHoldRpm(double delta) noexcept {
    const auto value = dynoHoldRpm_.load(std::memory_order_relaxed) + delta;
    dynoHoldRpm_.store(std::clamp(value, std::max(500.0, config_.idleRpm * 0.6), config_.redlineRpm),
                       std::memory_order_relaxed);
}

double EngineRuntime::updateDriveline(double dtSeconds, const EngineState& engineState, double requestedLoad) noexcept {
    auto requestedGear = gear_.load(std::memory_order_relaxed);
    const auto pedalClutch = clutchPressure_.load(std::memory_order_relaxed);
    const auto vehicle = config_.vehicle;
    const auto& transmission = config_.transmission;
    const auto maximumGear = static_cast<int>(transmission.gearRatios.size()) - 1;

    if (transmission.automaticShifting && !shiftInProgress_ && engagedGear_ >= 0) {
        if (engineState.rpm >= transmission.automaticUpshiftRpm && engagedGear_ < maximumGear
                && engineState.throttle > 0.12)
            gear_.store(requestedGear = engagedGear_ + 1, std::memory_order_relaxed);
        else if (engineState.rpm <= transmission.automaticDownshiftRpm && engagedGear_ > 0)
            gear_.store(requestedGear = engagedGear_ - 1, std::memory_order_relaxed);
    }
    if (!shiftInProgress_ && requestedGear != engagedGear_) {
        shiftInProgress_ = true;
        shiftFromGear_ = engagedGear_;
        shiftTargetGear_ = requestedGear;
        shiftElapsedSeconds_ = 0.0;
    }
    effectiveClutchPressure_ = pedalClutch;
    if (shiftInProgress_) {
        shiftElapsedSeconds_ += std::max(0.0, dtSeconds);
        shiftProgress_ = std::clamp(shiftElapsedSeconds_
            / std::max(0.01, transmission.shiftDurationSeconds), 0.0, 1.0);
        // Torque interruption follows a real disengage/synchronise/re-engage
        // envelope. The ratio changes only while the clutch is open.
        if (shiftProgress_ < 0.38)
            effectiveClutchPressure_ *= 1.0 - shiftProgress_ / 0.38;
        else if (shiftProgress_ < 0.58)
            effectiveClutchPressure_ = 0.0;
        else
            effectiveClutchPressure_ *= (shiftProgress_ - 0.58) / 0.42;
        if (shiftProgress_ >= 0.48) engagedGear_ = shiftTargetGear_;
        if (shiftProgress_ >= 1.0) {
            engagedGear_ = shiftTargetGear_;
            shiftInProgress_ = false;
            shiftProgress_ = 0.0;
        }
    } else {
        shiftProgress_ = 0.0;
    }

    const auto aeroForce = 0.5 * 1.225 * vehicle.dragCoefficient * vehicle.frontalAreaM2 * vehicleSpeedMps_ * vehicleSpeedMps_;
    const auto rollingForce = vehicleSpeedMps_ > 0.01
        ? vehicle.massKg * 9.80665 * vehicle.rollingResistanceCoefficient : 0.0;
    double driveForce = 0.0;
    drivelineLoadTorqueNm_ = 0.0;
    engineClutchTorqueNm_ = 0.0;
    wheelTorqueNm_ = 0.0;
    clutchSlipRpm_ = 0.0;
    if (engagedGear_ >= 0 && engagedGear_ <= maximumGear && effectiveClutchPressure_ > 0.001) {
        const auto totalRatio = transmission.gearRatios[static_cast<std::size_t>(engagedGear_)] * transmission.finalDriveRatio;
        const auto wheelAngularVelocity = vehicleSpeedMps_ / vehicle.tireRadiusM;
        const auto expectedEngineRpm = wheelAngularVelocity * totalRatio * 60.0 / (2.0 * std::numbers::pi);
        clutchSlipRpm_ = engineState.rpm - expectedEngineRpm;
        const auto slipOmega = clutchSlipRpm_ * 2.0 * std::numbers::pi / 60.0;
        const auto capacity = transmission.maxClutchTorqueNm * effectiveClutchPressure_;
        const auto equivalentWheelInertia = std::max(0.01,
            vehicle.massKg * vehicle.tireRadiusM * vehicle.tireRadiusM
                + transmission.drivenWheelInertiaKgM2);
        const auto synchronisingDenominator = std::max(1.0e-8, dtSeconds
            * (1.0 / config_.rotatingInertiaKgM2
                + totalRatio * totalRatio * transmission.drivelineEfficiency / equivalentWheelInertia));
        double clutchTorque = 0.0;
        if (std::abs(clutchSlipRpm_) <= transmission.clutchLockSpeedRpm) {
            clutchTorque = std::clamp(slipOmega / synchronisingDenominator, -capacity, capacity);
        } else {
            const auto slidingMagnitude = std::min(capacity,
                capacity * 0.15 + std::abs(clutchSlipRpm_)
                    * transmission.clutchSlipStiffnessNmPerRpm * effectiveClutchPressure_);
            clutchTorque = std::copysign(slidingMagnitude, slipOmega);
        }
        // clutchTorque is applied to the vehicle. The equal and opposite
        // reaction is injected into the crankshaft, preserving overrun and
        // engine-braking behaviour instead of clipping it away.
        engineClutchTorqueNm_ = -clutchTorque;
        drivelineLoadTorqueNm_ = clutchTorque;
        wheelTorqueNm_ = clutchTorque * totalRatio * transmission.drivelineEfficiency;
        driveForce = wheelTorqueNm_ / vehicle.tireRadiusM;
    }
    const auto effectiveVehicleMass = vehicle.massKg
        + transmission.drivenWheelInertiaKgM2 / (vehicle.tireRadiusM * vehicle.tireRadiusM);
    const auto acceleration = (driveForce - aeroForce - rollingForce) / effectiveVehicleMass;
    vehicleSpeedMps_ = std::max(0.0, vehicleSpeedMps_ + acceleration * dtSeconds);
    vehicleDistanceM_ += vehicleSpeedMps_ * dtSeconds;
    // The clutch reaction is already injected as a signed external crankshaft
    // torque. Converting it into the generic brake-load channel as well would
    // apply the same driveline demand twice under acceleration.
    return std::clamp(requestedLoad, 0.0, 1.0);
}

void EngineRuntime::startDyno() {
    const std::scoped_lock lock(dynoMutex_);
    if (dynoRunning_.load()) return;
    currentRun_ = {};
    currentRun_.id = nextDynoId_++;
    currentRun_.engineName = config_.name;
    dynoElapsed_ = 0.0;
    nextSampleRpm_ = std::max(1'000.0, config_.idleRpm);
    dynoTargetRpm_ = nextSampleRpm_;
    dynoStableElapsed_ = 0.0;
    dynoLoadCommand_ = 0.34;
    dynoFilteredRpm_ = simulator_.state().rpm;
    dynoTorqueAccumulator_ = 0.0;
    dynoPowerAccumulator_ = 0.0;
    dynoSampleCount_ = 0;
    savedIgnition_ = ignition_.load();
    savedStarter_ = starter_.load();
    savedThrottle_ = throttle_.load();
    savedLoad_ = load_.load();
    ignition_.store(true);
    starter_.store(true);
    throttle_.store(0.18);
    dynoSweeping_ = false;
    dynoCompleted_ = false;
    dynoRunning_.store(true);
}

void EngineRuntime::stopDyno() {
    const std::scoped_lock lock(dynoMutex_);
    if (!dynoRunning_.exchange(false)) return;
    if (currentRun_.points.size() >= 3) dynoHistory_.push_back(currentRun_);
    currentRun_ = {};
    ignition_.store(savedIgnition_);
    starter_.store(savedStarter_);
    throttle_.store(savedThrottle_);
    load_.store(savedLoad_);
}

DynoRun EngineRuntime::currentDynoRun() const {
    const std::scoped_lock lock(dynoMutex_);
    return currentRun_;
}

std::vector<DynoRun> EngineRuntime::dynoHistory() const {
    const std::scoped_lock lock(dynoMutex_);
    return dynoHistory_;
}

void EngineRuntime::deleteDynoRun(std::uint64_t id) {
    const std::scoped_lock lock(dynoMutex_);
    std::erase_if(dynoHistory_, [id](const DynoRun& run) { return run.id == id; });
}

void EngineRuntime::run(std::stop_token stopToken) {
    using Clock = std::chrono::steady_clock;
    constexpr auto baseStep = std::chrono::duration<double>(1.0 / 240.0);
#if defined(_WIN32)
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_ABOVE_NORMAL);
#endif
    auto deadline = Clock::now();
    double realtimeSeconds = 0.0;
    while (!stopToken.stop_requested()) {
        deadline += std::chrono::duration_cast<Clock::duration>(baseStep);
        auto requestedLoad = std::clamp(load_.load(), 0.0, 1.0);
        double dynoElapsed = 0.0;
        if (dynoRunning_.load()) {
            if (!dynoSweeping_ && simulator_.state().rpm >= std::max(650.0, config_.idleRpm * 0.82)) {
                dynoSweeping_ = true;
                dynoElapsed_ = 0.0;
                nextSampleRpm_ = dynoHoldEnabled_.load(std::memory_order_relaxed)
                    ? dynoHoldRpm_.load(std::memory_order_relaxed) : std::max(1'000.0, config_.idleRpm);
                dynoTargetRpm_ = nextSampleRpm_;
                dynoStableElapsed_ = 0.0;
                dynoLoadCommand_ = 0.34;
                dynoFilteredRpm_ = simulator_.state().rpm;
                dynoTorqueAccumulator_ = 0.0;
                dynoPowerAccumulator_ = 0.0;
                dynoSampleCount_ = 0;
                starter_.store(false);
                throttle_.store(1.0);
            }
            if (dynoSweeping_) {
                dynoElapsed_ += baseStep.count();
                dynoElapsed = dynoElapsed_;
                dynoFilteredRpm_ += (simulator_.state().rpm - dynoFilteredRpm_)
                    * (1.0 - std::exp(-baseStep.count() * 12.0));
                if (dynoHoldEnabled_.load(std::memory_order_relaxed))
                    dynoTargetRpm_ = dynoHoldRpm_.load(std::memory_order_relaxed);
                const auto error = dynoFilteredRpm_ - dynoTargetRpm_;
                dynoLoadCommand_ = std::clamp(dynoLoadCommand_ + error * 0.0022 * baseStep.count(), 0.02, 0.98);
                requestedLoad = std::clamp(dynoLoadCommand_ + error / 900.0, 0.02, 0.98);
            } else {
                requestedLoad = 0.0;
            }
        }
        if (!dynoRunning_.load(std::memory_order_relaxed))
            requestedLoad = updateDriveline(baseStep.count(), simulator_.state(), requestedLoad);
        const EngineControls controls { ignition_.load(), starter_.load(),
            dynoRunning_.load() ? (dynoSweeping_ ? 1.0 : 0.18) : std::clamp(throttle_.load(), 0.0, 1.0),
            requestedLoad, dynoRunning_.load() ? 0.0 : engineClutchTorqueNm_ };
        const auto isPaused = paused_.load(std::memory_order_relaxed) && !dynoRunning_.load(std::memory_order_relaxed);
        const auto simulationDt = isPaused ? 0.0 : baseStep.count()
            * (dynoRunning_.load(std::memory_order_relaxed) ? 1.0 : timeScale_.load(std::memory_order_relaxed));
        auto frame = simulationDt > 0.0 ? simulator_.step(simulationDt, controls) : SimulationFrame { simulator_.state() };
        if (simulationDt > 0.0) {
            const auto simulationStart = frame.state.simulationTimeSeconds - simulationDt;
            for (std::size_t index = 0; index < frame.firingEventCount; ++index) {
                const auto fraction = std::clamp((frame.firingEvents[index].timeSeconds - simulationStart) / simulationDt, 0.0, 1.0);
                frame.firingEvents[index].timeSeconds = realtimeSeconds + fraction * baseStep.count();
            }
            CylinderPressureSample pressureSample;
            while (simulator_.tryPopCylinderPressureSample(pressureSample)) {
                const auto fraction = std::clamp((pressureSample.timeSeconds
                    - simulationStart) / simulationDt, 0.0, 1.0);
                pressureSample.timeSeconds = realtimeSeconds + fraction * baseStep.count();
                if (!pressureQueue_.tryPush(pressureSample))
                    droppedPressureSamples_.fetch_add(1, std::memory_order_relaxed);
            }
            if (frame.droppedCylinderPressureSampleCount > 0)
                droppedPressureSamples_.fetch_add(frame.droppedCylinderPressureSampleCount,
                                                  std::memory_order_relaxed);
        }
        audioState_.rpm.store(isPaused ? 0.0F : static_cast<float>(frame.state.rpm), std::memory_order_relaxed);
        audioState_.throttle.store(isPaused ? 0.0F : static_cast<float>(frame.state.throttle), std::memory_order_relaxed);
        const auto drivelineAudioLoad = std::clamp(std::abs(engineClutchTorqueNm_)
            / std::max(20.0, config_.transmission.maxClutchTorqueNm), 0.0, 1.0);
        audioState_.load.store(static_cast<float>(std::max(frame.state.load, drivelineAudioLoad)),
                               std::memory_order_relaxed);
        audioState_.manifoldPressureKpa.store(static_cast<float>(frame.state.manifoldPressureKpa),
                                              std::memory_order_relaxed);
        audioState_.exhaustPressureKpa.store(static_cast<float>(frame.state.exhaustPressureKpa),
                                             std::memory_order_relaxed);
        audioState_.exhaustFlowGramsPerSecond.store(static_cast<float>(frame.state.exhaustFlowGramsPerSecond),
                                                    std::memory_order_relaxed);
        audioState_.boostPressureRatio.store(static_cast<float>(frame.state.boostPressureRatio), std::memory_order_relaxed);
        audioState_.mechanicalStress.store(static_cast<float>(std::clamp(frame.state.peakPistonAccelerationG / 7'000.0, 0.0, 1.0)),
                                           std::memory_order_relaxed);
        audioState_.starter.store(controls.starterEngaged && !isPaused ? 1.0F : 0.0F, std::memory_order_relaxed);
        audioState_.timeScale.store(isPaused ? 0.0F : static_cast<float>(dynoRunning_.load(std::memory_order_relaxed)
            ? 1.0 : timeScale_.load(std::memory_order_relaxed)), std::memory_order_relaxed);
        if (dynoRunning_.load() && dynoSweeping_) {
            if (std::abs(dynoFilteredRpm_ - dynoTargetRpm_) <= 80.0) {
                dynoTorqueAccumulator_ += frame.state.cycleAveragedTorqueNm;
                dynoPowerAccumulator_ += frame.state.cycleAveragedPowerKw;
                ++dynoSampleCount_;
                dynoStableElapsed_ += baseStep.count();
            } else {
                dynoTorqueAccumulator_ = 0.0;
                dynoPowerAccumulator_ = 0.0;
                dynoSampleCount_ = 0;
                dynoStableElapsed_ = 0.0;
            }
            if (dynoStableElapsed_ >= 0.10 && dynoSampleCount_ > 0) {
                const auto divisor = static_cast<double>(dynoSampleCount_);
                const auto atmosphericCorrection = std::clamp((99.0 / config_.ambientPressureKpa)
                    * std::sqrt((config_.ambientTemperatureC + 273.15) / 298.15), 0.80, 1.20);
                DynoPoint point { dynoTargetRpm_, dynoTorqueAccumulator_ / divisor,
                    dynoPowerAccumulator_ / divisor, frame.state.airFuelRatio, frame.state.coolantTemperatureC,
                    frame.state.exhaustTemperatureC, frame.state.ignitionAdvanceDegrees, atmosphericCorrection,
                    dynoTorqueAccumulator_ / divisor * atmosphericCorrection,
                    dynoPowerAccumulator_ / divisor * atmosphericCorrection };
                point.targetAirFuelRatio = frame.state.targetAirFuelRatio;
                point.volumetricEfficiency = frame.state.volumetricEfficiency;
                point.fuelFlowGramsPerSecond = frame.state.fuelFlowGramsPerSecond;
                point.manifoldPressureKpa = frame.state.manifoldPressureKpa;
                point.exhaustPressureKpa = frame.state.exhaustPressureKpa;
                point.oilTemperatureC = frame.state.oilTemperatureC;
                point.oilPressureKpa = frame.state.oilPressureKpa;
                point.airFlowGramsPerSecond = frame.state.airFlowGramsPerSecond;
                point.lambda = frame.state.lambda;
                point.brakeSpecificFuelConsumptionGPerKwh = frame.state.brakeSpecificFuelConsumptionGPerKwh;
                {
                    const std::scoped_lock lock(dynoMutex_);
                    currentRun_.points.push_back(point);
                    currentRun_.peakTorqueNm = std::max(currentRun_.peakTorqueNm, point.torqueNm);
                    currentRun_.peakPowerKw = std::max(currentRun_.peakPowerKw, point.powerKw);
                    currentRun_.peakCorrectedTorqueNm = std::max(currentRun_.peakCorrectedTorqueNm, point.correctedTorqueNm);
                    currentRun_.peakCorrectedPowerKw = std::max(currentRun_.peakCorrectedPowerKw, point.correctedPowerKw);
                }
                if (!dynoHoldEnabled_.load(std::memory_order_relaxed) && point.rpm >= config_.redlineRpm)
                    dynoCompleted_.store(true, std::memory_order_relaxed);
                if (!dynoHoldEnabled_.load(std::memory_order_relaxed))
                    dynoTargetRpm_ = std::min(config_.redlineRpm, dynoTargetRpm_ + 100.0);
                nextSampleRpm_ = dynoTargetRpm_;
                dynoTorqueAccumulator_ = 0.0;
                dynoPowerAccumulator_ = 0.0;
                dynoSampleCount_ = 0;
                dynoStableElapsed_ = 0.0;
            }
        }
        for (std::size_t index = 0; index < frame.firingEventCount; ++index)
            if (!eventQueue_.tryPush(frame.firingEvents[index])) droppedEvents_.fetch_add(1, std::memory_order_relaxed);
        if (frame.droppedFiringEventCount > 0)
            droppedEvents_.fetch_add(frame.droppedFiringEventCount, std::memory_order_relaxed);
        {
            frame.state.gear = engagedGear_;
            frame.state.gearCount = static_cast<int>(config_.transmission.gearRatios.size());
            frame.state.clutchPressure = clutchPressure_.load(std::memory_order_relaxed);
            frame.state.vehicleSpeedMps = vehicleSpeedMps_;
            frame.state.vehicleDistanceM = vehicleDistanceM_;
            frame.state.fuelEconomyLitresPer100Km = vehicleDistanceM_ > 10.0
                ? frame.state.fuelConsumedLitres * 100'000.0 / vehicleDistanceM_ : 0.0;
            frame.state.wheelTorqueNm = wheelTorqueNm_;
            frame.state.drivelineLoadTorqueNm = drivelineLoadTorqueNm_;
            frame.state.clutchTorqueNm = -engineClutchTorqueNm_;
            frame.state.clutchSlipRpm = clutchSlipRpm_;
            frame.state.shiftProgress = shiftProgress_;
            frame.state.shiftInProgress = shiftInProgress_;
            frame.state.dynoHoldRpm = dynoHoldRpm_.load(std::memory_order_relaxed);
            frame.state.dynoHoldEnabled = dynoHoldEnabled_.load(std::memory_order_relaxed);
            const std::scoped_lock lock(snapshotMutex_);
            snapshot_ = frame.state;
        }
        if (dynoRunning_.load() && (dynoCompleted_.load(std::memory_order_relaxed) || dynoElapsed >= 30.0))
            stopDyno();
        realtimeSeconds += baseStep.count();
        const auto now = Clock::now();
        if (now > deadline + std::chrono::duration_cast<Clock::duration>(baseStep * 4.0)) {
            timingOverruns_.fetch_add(1, std::memory_order_relaxed);
            deadline = now;
        }
        std::this_thread::sleep_until(deadline);
    }
}
} // namespace enginelab
