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
namespace {
[[nodiscard]] EngineConfig normalised(EngineConfig config) {
    normaliseEngineConfig(config);
    return config;
}
}
EngineRuntime::EngineRuntime(EngineConfig config)
    : config_(normalised(std::move(config))), exhaust_(ExhaustGraph::makeForEngine(config_)),
      simulator_(config_, ecu_, physics_, eventGenerator_, exhaust_), driveline_(config_) {
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
    for (std::size_t index = 0; index < config_.cylinders.size(); ++index) {
        const auto& cylinder = config_.cylinders[index];
        float pan = 0.0F;
        const auto bank = std::find_if(config_.banks.begin(), config_.banks.end(), [&cylinder](const auto& item) {
            return item.id == cylinder.bankId
                || std::find(item.cylinderIds.begin(), item.cylinderIds.end(), cylinder.id) != item.cylinderIds.end();
        });
        if (config_.layout == EngineLayout::radial)
            pan = static_cast<float>(std::sin(cylinder.bankOffsetDegrees * std::numbers::pi / 180.0) * 0.74);
        else if (bank != config_.banks.end() && std::abs(bank->angleDegrees) > 0.1)
            pan = static_cast<float>(std::sin(bank->angleDegrees * std::numbers::pi / 180.0) * 0.78);
        else if (config_.cylinders.size() > 1)
            pan = static_cast<float>(-0.72 + 1.44 * static_cast<double>(index)
                / static_cast<double>(config_.cylinders.size() - 1));
        audioState_.cylinderPan[index].store(std::clamp(pan, -0.82F, 0.82F), std::memory_order_relaxed);
    }
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
    const auto selected = std::clamp(gear, -2, maxGear);
    gear_.store(selected, std::memory_order_relaxed);
    driveline_.requestGear(selected);
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
    drivelineOutput_ = driveline_.advance(dtSeconds, engineState, requestedLoad,
        clutchPressure_.load(std::memory_order_relaxed), brakePressure_.load(std::memory_order_relaxed));
    engagedGear_ = drivelineOutput_.engagedGear;
    gear_.store(driveline_.requestedGear(), std::memory_order_relaxed);
    effectiveClutchPressure_ = drivelineOutput_.clutchPressure;
    engineClutchTorqueNm_ = drivelineOutput_.engineReactionTorqueNm;
    drivelineLoadTorqueNm_ = drivelineOutput_.clutchTorqueNm;
    wheelTorqueNm_ = drivelineOutput_.wheelTorqueNm;
    clutchSlipRpm_ = drivelineOutput_.clutchSlipRpm;
    vehicleSpeedMps_ = drivelineOutput_.vehicleSpeedMps;
    vehicleDistanceM_ = drivelineOutput_.vehicleDistanceM;
    shiftProgress_ = drivelineOutput_.shiftProgress;
    shiftInProgress_ = drivelineOutput_.shiftInProgress;
    return drivelineOutput_.requestedLoad;
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
        const auto isPaused = paused_.load(std::memory_order_relaxed) && !dynoRunning_.load(std::memory_order_relaxed);
        const auto simulationScale = dynoRunning_.load(std::memory_order_relaxed)
            ? 1.0 : std::clamp(timeScale_.load(std::memory_order_relaxed), 0.0, 8.0);
        const auto simulationDt = isPaused ? 0.0 : baseStep.count() * simulationScale;
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
        if (!dynoRunning_.load(std::memory_order_relaxed) && simulationDt > 0.0)
            requestedLoad = updateDriveline(simulationDt, simulator_.state(), requestedLoad);
        const EngineControls controls { ignition_.load(), starter_.load(),
            (dynoRunning_.load() ? (dynoSweeping_ ? 1.0 : 0.18) : std::clamp(throttle_.load(), 0.0, 1.0))
                * drivelineOutput_.torqueCutMultiplier,
            requestedLoad, dynoRunning_.load() ? 0.0 : engineClutchTorqueNm_,
            dynoRunning_.load() ? 0.0 : brakePressure_.load(std::memory_order_relaxed) };
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
            const auto minimumCycleWindowSeconds = std::clamp(120.0
                / std::max(250.0, dynoTargetRpm_) * 2.0, 0.12, 0.60);
            if (dynoStableElapsed_ >= minimumCycleWindowSeconds && dynoSampleCount_ > 0) {
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
            frame.state.clutchPressure = effectiveClutchPressure_;
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
            frame.state.brakePressure = drivelineOutput_.brakePressure;
            frame.state.brakeForceN = drivelineOutput_.brakeForceN;
            frame.state.tireLongitudinalForceN = drivelineOutput_.tireForceN;
            frame.state.tractionLimited = drivelineOutput_.tractionLimited;
            frame.state.clutchTemperatureC = drivelineOutput_.clutchTemperatureC;
            frame.state.clutchDissipatedEnergyJoules = drivelineOutput_.clutchDissipatedEnergyJoules;
            frame.state.clutchPowerLossKw = drivelineOutput_.clutchPowerLossKw;
            frame.state.drivelineStoredEnergyJoules = drivelineOutput_.storedEnergyJoules
                + 0.5 * effectiveRotatingInertiaKgM2(config_)
                    * frame.state.angularVelocityRadPerSecond * frame.state.angularVelocityRadPerSecond;
            frame.state.drivelineEnergyResidualJoules = drivelineOutput_.energyResidualJoules;
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
