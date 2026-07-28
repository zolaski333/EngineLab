#include <enginelab/runtime/EngineRuntime.hpp>
#include <enginelab/runtime/MonotonicPublicationTimeline.hpp>
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
constexpr std::size_t maximumAudioExhaustPaths = 8;

[[nodiscard]] EngineConfig normalised(EngineConfig config) {
    normaliseEngineConfig(config);
    return config;
}

[[nodiscard]] std::size_t exhaustPathIndexFor(const EngineConfig& config,
                                              const CylinderConfig& cylinder) noexcept {
    const auto path = std::find_if(config.exhaustPaths.begin(), config.exhaustPaths.end(),
        [&cylinder](const ExhaustPathConfig& item) {
            return std::find(item.cylinderIds.begin(), item.cylinderIds.end(), cylinder.id)
                != item.cylinderIds.end();
        });
    return path != config.exhaustPaths.end()
        ? static_cast<std::size_t>(std::distance(config.exhaustPaths.begin(), path)) : 0U;
}

[[nodiscard]] const ExhaustConfig& exhaustGeometryAt(const EngineConfig& config,
                                                     std::size_t pathIndex) noexcept {
    return pathIndex < config.exhaustPaths.size()
        ? config.exhaustPaths[pathIndex].geometry : config.exhaust;
}

[[nodiscard]] double circularAreaM2(double diameterMm) noexcept {
    const auto radiusM = std::max(1.0, diameterMm) * 0.0005;
    return std::numbers::pi * radiusM * radiusM;
}
}

void publishAudioFrame(RealtimeAudioState& state, const EngineState& engineState,
                       const AudioFramePublication& context) noexcept {
    const auto paused = context.paused;
    state.rpm.store(paused ? 0.0F : static_cast<float>(engineState.rpm), std::memory_order_relaxed);
    state.throttle.store(paused ? 0.0F : static_cast<float>(engineState.throttle),
                         std::memory_order_relaxed);
    state.load.store(static_cast<float>(std::max(engineState.load,
        std::clamp(context.drivelineLoad, 0.0, 1.0))), std::memory_order_relaxed);
    state.manifoldPressureKpa.store(static_cast<float>(engineState.manifoldPressureKpa),
                                    std::memory_order_relaxed);
    state.exhaustPressureKpa.store(static_cast<float>(engineState.exhaustPressureKpa),
                                   std::memory_order_relaxed);
    state.exhaustFlowGramsPerSecond.store(static_cast<float>(engineState.exhaustFlowGramsPerSecond),
                                          std::memory_order_relaxed);
    state.exhaustTemperatureC.store(static_cast<float>(std::clamp(
        engineState.exhaustTemperatureC, -50.0, 1'800.0)), std::memory_order_relaxed);
    state.boostPressureRatio.store(static_cast<float>(engineState.boostPressureRatio),
                                   std::memory_order_relaxed);
    state.mechanicalStress.store(static_cast<float>(std::clamp(
        engineState.peakPistonAccelerationG / 7'000.0, 0.0, 1.0)), std::memory_order_relaxed);
    state.peakPistonAccelerationG.store(static_cast<float>(std::max(0.0,
        engineState.peakPistonAccelerationG)), std::memory_order_relaxed);
    state.forcedInductionShaftRpm.store(static_cast<float>(std::max(0.0,
        engineState.forcedInductionShaftSpeedRpm)), std::memory_order_relaxed);
    state.wastegateOpening.store(static_cast<float>(std::clamp(
        engineState.wastegateOpening, 0.0, 1.0)), std::memory_order_relaxed);
    constexpr double referenceTemperatureK = 288.15;
    constexpr double referencePressureKpa = 101.325;
    // SAE-style compressor corrected flow is referenced at the compressor
    // inlet, not at the boosted manifold. A venting bypass still traverses the
    // compressor and is therefore part of its actual throughput.
    const auto inletTemperatureK = std::max(150.0,
        static_cast<double>(state.ambientTemperatureC.load(
            std::memory_order_relaxed)) + 273.15);
    const auto inletPressureKpa = std::max(1.0,
        static_cast<double>(state.ambientPressureKpa.load(
            std::memory_order_relaxed)));
    const auto correctedFlowKgPerSecond = (std::max(0.0,
        engineState.airFlowGramsPerSecond * 0.001)
        + std::max(0.0, engineState.blowOffMassFlowKgPerSecond))
        * std::sqrt(inletTemperatureK / referenceTemperatureK)
        / (inletPressureKpa / referencePressureKpa);
    state.correctedAirFlowKgPerSecond.store(
        static_cast<float>(correctedFlowKgPerSecond), std::memory_order_relaxed);
    state.compressorPowerWatts.store(static_cast<float>(std::max(
        0.0, engineState.compressorPowerKw * 1'000.0)), std::memory_order_relaxed);
    state.turbinePowerWatts.store(static_cast<float>(std::max(
        0.0, engineState.turbinePowerKw * 1'000.0)), std::memory_order_relaxed);
    state.blowOffMassFlowKgPerSecond.store(static_cast<float>(std::max(
        0.0, engineState.blowOffMassFlowKgPerSecond)), std::memory_order_relaxed);
    // Aggregate the dominant absolute intake-runner resonance while preserving
    // its sign/phase for the induction audio layer.
    double dominantAmplitude = 0.0;
    double dominantFrequency = 0.0;
    for (std::size_t index = 0; index < engineState.cylinderStateCount; ++index) {
        const auto& cylinder = engineState.cylinderStates[index];
        if (std::abs(cylinder.intakeResonancePressureKpa) > std::abs(dominantAmplitude)) {
            dominantAmplitude = cylinder.intakeResonancePressureKpa;
            dominantFrequency = cylinder.intakeResonanceFrequencyHz;
        }
    }
    state.intakeRunnerResonanceHz.store(static_cast<float>(dominantFrequency),
                                        std::memory_order_relaxed);
    state.intakeRunnerAmplitudeKpa.store(static_cast<float>(paused ? 0.0 : dominantAmplitude),
                                         std::memory_order_relaxed);
    state.starter.store(context.starterEngaged && !paused ? 1.0F : 0.0F, std::memory_order_relaxed);
    state.timeScale.store(paused ? 0.0F : static_cast<float>(context.timeScale),
                          std::memory_order_relaxed);
}

EngineRuntime::EngineRuntime(EngineConfig config,
                             std::shared_ptr<calibration::CalibrationStore> calibrations,
                             EngineSimulatorOptions simulatorOptions)
    : config_(normalised(std::move(config))), ecu_(std::move(calibrations)),
      exhaust_(ExhaustGraph::makeForEngine(config_)),
      simulator_(config_, ecu_, physics_, eventGenerator_, exhaust_,
                 std::move(simulatorOptions)), driveline_(config_),
      pressureQueue_(std::make_unique<CylinderPressureQueue>()) {
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
    for (std::size_t index = 0;
         index < config_.cylinders.size() && index < audioState_.cylinderPan.size(); ++index) {
        const auto& cylinder = config_.cylinders[index];
        audioState_.cylinderId[index].store(cylinder.id, std::memory_order_relaxed);
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
    audioState_.ambientPressureKpa.store(static_cast<float>(config_.ambientPressureKpa),
                                         std::memory_order_relaxed);
    audioState_.ambientTemperatureC.store(static_cast<float>(config_.ambientTemperatureC),
                                          std::memory_order_relaxed);
    const auto pathCount = std::clamp<std::size_t>(config_.exhaustPaths.empty()
        ? 1U : config_.exhaustPaths.size(), 1U, maximumAudioExhaustPaths);
    audioState_.exhaustPathCount.store(static_cast<std::uint32_t>(pathCount), std::memory_order_relaxed);
    const auto exhaustSoundSpeedMps = std::clamp(exhaust_.referenceWaveSpeedMps(), 300.0, 900.0);
    const auto exhaustSoundSpeedMmPerSecond = exhaustSoundSpeedMps * 1'000.0;
    audioState_.exhaustReferenceSoundSpeedMps.store(
        static_cast<float>(exhaustSoundSpeedMps), std::memory_order_relaxed);
    audioState_.exhaustTemperatureC.store(
        static_cast<float>(config_.ambientTemperatureC), std::memory_order_relaxed);
    // These route averages remain part of RealtimeAudioState for compatibility
    // with graph-less producers. Production audio receives exhaust_ directly
    // and compiles every duct, junction and outlet instead of consuming this
    // reduced path geometry.
    std::array<double, maximumAudioExhaustPaths> pathLengthSum {};
    std::array<double, maximumAudioExhaustPaths> pathRestrictionSum {};
    std::array<std::size_t, maximumAudioExhaustPaths> pathCylinderCount {};

    // Publish the exact energy-combined graph transmission used by firing
    // events. Continuous runner pressure consumes this value before entering
    // the waveguide, while the path output remains unity-gain.
    for (std::size_t index = 0;
         index < config_.cylinders.size() && index < audioState_.cylinderExhaustGain.size(); ++index) {
        const auto& cylinder = config_.cylinders[index];
        const auto acoustics = exhaust_.acousticsForCylinder(cylinder.id);
        const auto fallbackPathIndex = exhaustPathIndexFor(config_, cylinder);
        const auto compiledPathIndex = acoustics.routeCount > 0
            ? static_cast<std::size_t>(acoustics.pathIndex) : fallbackPathIndex;
        const auto pathIndex = std::min(compiledPathIndex, pathCount - 1U);
        audioState_.cylinderExhaustPathIndex[index].store(static_cast<std::uint32_t>(pathIndex),
                                                          std::memory_order_relaxed);
        audioState_.cylinderExhaustGain[index].store(static_cast<float>(std::clamp(
            acoustics.transmissionGain, 0.0, 8.0)), std::memory_order_relaxed);

        const auto& fallbackGeometry = exhaustGeometryAt(config_, pathIndex);
        const auto flowProperties = exhaust_.cylinderFlowProperties(cylinder.id);
        const auto runnerAreaM2 = flowProperties.inletAreaM2 > 1.0e-8
            ? flowProperties.inletAreaM2
            : circularAreaM2(fallbackGeometry.primaryDiameterMm);
        audioState_.cylinderExhaustAreaM2[index].store(
            static_cast<float>(std::clamp(runnerAreaM2, 1.0e-5, 0.040)),
            std::memory_order_relaxed);
        const auto fallbackRunnerLengthMm = cylinder.exhaustPrimaryLengthMm > 0.0
            ? cylinder.exhaustPrimaryLengthMm : fallbackGeometry.primaryLengthMm;
        const auto runnerLengthMm = flowProperties.runnerLengthMm > 0.0
            ? flowProperties.runnerLengthMm : fallbackRunnerLengthMm;
        const auto runnerDelaySeconds = std::max(0.0, runnerLengthMm)
            / exhaustSoundSpeedMmPerSecond;
        audioState_.runnerDelaySeconds[index].store(
            static_cast<float>(std::clamp(runnerDelaySeconds, 0.0, 0.080)),
            std::memory_order_relaxed);

        if (acoustics.routeCount > 0) {
            // The per-cylinder line above already propagates through the
            // primary.  The shared outlet line starts at the collector, so its
            // delay must contain only the remaining route length.
            pathLengthSum[pathIndex] += std::max(0.0,
                acoustics.meanLengthMm - runnerLengthMm);
            pathRestrictionSum[pathIndex] += acoustics.equivalentRestriction;
            ++pathCylinderCount[pathIndex];
        }
    }

    for (std::size_t pathIndex = 0; pathIndex < maximumAudioExhaustPaths; ++pathIndex) {
        const auto& geometry = exhaustGeometryAt(config_, pathIndex);
        const auto flowProperties = exhaust_.pathFlowProperties(pathIndex);
        const auto outletAreaM2 = flowProperties.effectiveOutletAreaM2 > 1.0e-8
            ? flowProperties.effectiveOutletAreaM2
            : circularAreaM2(geometry.outletDiameterMm)
                * std::clamp(geometry.outletDischargeCoefficient, 0.05, 1.5);
        audioState_.exhaustPathOutletAreaM2[pathIndex].store(
            static_cast<float>(std::clamp(outletAreaM2, 1.0e-5, 0.080)),
            std::memory_order_relaxed);
        auto openness = std::clamp(
            (geometry.outletDiameterMm / std::max(20.0, geometry.collectorDiameterMm))
                * (1.0 - geometry.mufflerRestriction * 0.72), 0.15, 1.45);
        auto pathLengthMm = 120.0 + 450.0 + 180.0;
        if (pathCylinderCount[pathIndex] > 0) {
            const auto count = static_cast<double>(pathCylinderCount[pathIndex]);
            pathLengthMm = pathLengthSum[pathIndex] / count;
            const auto equivalentRestriction = pathRestrictionSum[pathIndex] / count;
            // K is dimensionless: acoustic conductance is proportional to
            // 1/sqrt(1 + K). The factor keeps legacy presets close to their old
            // diameter heuristic while custom catalysts/mufflers close the path.
            openness = std::clamp(1.0 / std::sqrt(1.0 + 1.35 * equivalentRestriction),
                                  0.15, 1.45);
        }
        // One-way collector-to-outlet propagation. The bidirectional delay
        // line naturally makes a reflected wave's full trip twice this value.
        const auto reflectionSeconds = std::clamp(
            pathLengthMm / exhaustSoundSpeedMmPerSecond, 0.001, 0.080);
        audioState_.exhaustPathOpenness[pathIndex].store(
            static_cast<float>(openness), std::memory_order_relaxed);
        audioState_.exhaustPathReflectionSeconds[pathIndex].store(
            static_cast<float>(reflectionSeconds), std::memory_order_relaxed);
        // The chamber sits on the collector-to-outlet duct, so the duct area it
        // steps up from is the collector's. Geometry is published as the bare
        // expansion ratio and traversal time; turning those into scattering
        // coefficients is the audio layer's job, and Runtime cannot include an
        // audio header anyway -- Audio links Runtime, not the reverse. Both
        // fields must be present: a chamber with no length is not a chamber,
        // and neither is one with no expansion.
        const auto ductAreaM2 = circularAreaM2(geometry.collectorDiameterMm);
        const auto chamberAreaM2 = circularAreaM2(geometry.mufflerChamberDiameterMm);
        const auto chamberTraversalSeconds = geometry.mufflerChamberLengthMm
            / exhaustSoundSpeedMmPerSecond;
        const auto chamberConfigured = geometry.mufflerChamberDiameterMm > 1.0
            && geometry.mufflerChamberLengthMm > 1.0 && ductAreaM2 > 1.0e-9;
        audioState_.exhaustPathMufflerExpansionRatio[pathIndex].store(
            chamberConfigured
                ? static_cast<float>(std::clamp(chamberAreaM2 / ductAreaM2, 0.05, 100.0))
                : 0.0F,
            std::memory_order_relaxed);
        audioState_.exhaustPathMufflerTraversalSeconds[pathIndex].store(
            chamberConfigured
                ? static_cast<float>(std::clamp(chamberTraversalSeconds, 0.0, 0.020))
                : 0.0F,
            std::memory_order_relaxed);
        // All configured gain is already present per cylinder (and in each
        // firing event). Leaving the path at unity removes the historical
        // audioVolume double multiplication.
        audioState_.exhaustPathGain[pathIndex].store(1.0F, std::memory_order_relaxed);
    }
    audioState_.exhaustOpenness.store(audioState_.exhaustPathOpenness[0].load(std::memory_order_relaxed),
                                      std::memory_order_relaxed);
    audioState_.boostPressureRatio.store(static_cast<float>(config_.forcedInduction.enabled
        ? config_.forcedInduction.pressureRatio : 1.0), std::memory_order_relaxed);
    audioState_.exhaustReflectionSeconds.store(
        audioState_.exhaustPathReflectionSeconds[0].load(std::memory_order_relaxed),
        std::memory_order_relaxed);
    audioState_.meanBoreMm.store(static_cast<float>(std::max(20.0, meanBore)), std::memory_order_relaxed);
    audioState_.forcedInductionKind.store(config_.forcedInduction.enabled
        ? (config_.forcedInduction.type == ForcedInductionType::supercharger ? 2 : 1) : 0,
        std::memory_order_relaxed);
}
EngineRuntime::~EngineRuntime() { stop(); }

void EngineRuntime::start() {
    if (!thread_.joinable()) thread_ = std::jthread([this](std::stop_token token) { run(token); });
}
void EngineRuntime::stop() {
    dynoRequestedRunning_.store(false, std::memory_order_release);
    if (thread_.joinable()) { thread_.request_stop(); thread_.join(); }
    if (dynoActive_) finishDynoSession();
}
EngineState EngineRuntime::snapshot() const {
    const std::scoped_lock lock(snapshotMutex_);
    return snapshot_;
}

void EngineRuntime::setGear(int gear) noexcept {
    const auto maxGear = static_cast<int>(config_.transmission.gearRatios.size()) - 1;
    const auto selected = std::clamp(gear, -2, maxGear);
    gear_.store(selected, std::memory_order_relaxed);
    gearCommandGeneration_.fetch_add(1, std::memory_order_release);
}

void EngineRuntime::shiftUp() noexcept {
    setGear(gear_.load(std::memory_order_relaxed) + 1);
}

void EngineRuntime::shiftDown() noexcept {
    setGear(gear_.load(std::memory_order_relaxed) - 1);
}

void EngineRuntime::adjustDynoHoldRpm(double delta) noexcept {
    setDynoHoldRpm(dynoHoldRpm_.load(std::memory_order_relaxed) + delta);
}

void EngineRuntime::setDynoHoldRpm(double value) noexcept {
    if (!std::isfinite(value)) return;
    const auto maximumHoldRpm = std::min(
        config_.redlineRpm, config_.ignition.revLimitRpm);
    dynoHoldRpm_.store(
        std::clamp(value, std::max(500.0, config_.idleRpm * 0.6), maximumHoldRpm),
        std::memory_order_relaxed);
}

void EngineRuntime::updateDriveline(double dtSeconds, const EngineState& engineState,
                                    double requestedLoad) noexcept {
    drivelineOutput_ = driveline_.advance(dtSeconds, engineState, requestedLoad,
        clutchPressure_.load(std::memory_order_relaxed), brakePressure_.load(std::memory_order_relaxed));
    engagedGear_ = drivelineOutput_.engagedGear;
    effectiveClutchPressure_ = drivelineOutput_.clutchPressure;
    engineClutchTorqueNm_ = drivelineOutput_.engineReactionTorqueNm;
    drivelineLoadTorqueNm_ = drivelineOutput_.clutchTorqueNm;
    wheelTorqueNm_ = drivelineOutput_.wheelTorqueNm;
    clutchSlipRpm_ = drivelineOutput_.clutchSlipRpm;
    vehicleSpeedMps_ = drivelineOutput_.vehicleSpeedMps;
    vehicleDistanceM_ = drivelineOutput_.vehicleDistanceM;
    shiftProgress_ = drivelineOutput_.shiftProgress;
    shiftInProgress_ = drivelineOutput_.shiftInProgress;
}

void EngineRuntime::startDyno() {
    dynoRequestedRunning_.store(true, std::memory_order_release);
}

void EngineRuntime::stopDyno() {
    dynoRequestedRunning_.store(false, std::memory_order_release);
}

void EngineRuntime::beginDynoSession() {
    if (dynoActive_) return;
    {
        const std::scoped_lock lock(dynoMutex_);
        currentRun_ = {};
        currentRun_.id = nextDynoId_++;
        currentRun_.engineName = config_.name;
    }
    dynoElapsed_ = 0.0;
    dynoStartupElapsed_ = 0.0;
    nextSampleRpm_ = std::max(1'000.0, config_.idleRpm);
    dynoTargetRpm_ = nextSampleRpm_;
    dynoStableElapsed_ = 0.0;
    dynoBrakeTorqueNm_ = 0.0;
    dynoControllerIntegralNm_ = 0.0;
    dynoFeedForwardTorqueNm_ = 0.0;
    dynoFilteredRpm_ = simulator_.state().rpm;
    dynoFilteredAccelerationRpmPerSecond_ = 0.0;
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
    dynoActive_ = true;
}

void EngineRuntime::finishDynoSession() {
    if (!dynoActive_) return;
    {
        const std::scoped_lock lock(dynoMutex_);
        if (currentRun_.points.size() >= 3) dynoHistory_.push_back(currentRun_);
        currentRun_ = {};
    }
    ignition_.store(savedIgnition_);
    starter_.store(savedStarter_);
    throttle_.store(savedThrottle_);
    load_.store(savedLoad_);
    dynoActive_ = false;
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
    const auto clockEpoch = Clock::now();
    auto deadline = clockEpoch;
    auto consumedGearGeneration = std::uint64_t { 0 };
    auto consumedGearCommand = gear_.load(std::memory_order_relaxed);
    double nextPressurePublishTime = 0.0;
    MonotonicPublicationTimeline publicationTimeline;
    RealtimeLoadGovernor loadGovernor;
    const auto normalIntakeWallHeatIntervalSeconds =
        simulator_.intakeWallHeatUpdateIntervalSeconds();
    const auto protectedIntakeWallHeatIntervalSeconds =
        std::max(normalIntakeWallHeatIntervalSeconds, 600.0e-6);
    simulator_.setIntakeWallHeatUpdateIntervalSeconds(
        normalIntakeWallHeatIntervalSeconds);
    realtimeLoadProtectionActive_.store(false, std::memory_order_relaxed);
    constexpr double maximumPressurePublishRateHz = 96'000.0;
    constexpr double minimumPressurePublishInterval = 1.0 / maximumPressurePublishRateHz;
    while (!stopToken.stop_requested()) {
        const auto iterationStart = Clock::now();
        const auto realtimeSeconds = std::chrono::duration<double>(iterationStart - clockEpoch).count();
        audioState_.producerTimeNanoseconds.store(static_cast<std::uint64_t>(std::max(0.0, realtimeSeconds) * 1.0e9),
                                                  std::memory_order_release);
        deadline += std::chrono::duration_cast<Clock::duration>(baseStep);
        const auto dynoDesired = dynoRequestedRunning_.load(std::memory_order_acquire);
        if (dynoDesired && !dynoActive_) beginDynoSession();
        else if (!dynoDesired && dynoActive_) finishDynoSession();
        const auto isPaused = paused_.load(std::memory_order_relaxed) && !dynoActive_;
        const auto simulationScale = dynoActive_
            ? 1.0 : std::clamp(timeScale_.load(std::memory_order_relaxed), 0.0, 8.0);
        const auto simulationDt = isPaused ? 0.0 : baseStep.count() * simulationScale;
        auto requestedLoad = std::clamp(load_.load(), 0.0, 1.0);
        double dynoElapsed = 0.0;
        double dynoStartupElapsed = 0.0;
        if (dynoActive_) {
            dynoStartupElapsed_ += baseStep.count();
            dynoStartupElapsed = dynoStartupElapsed_;
            if (!dynoSweeping_ && simulator_.state().rpm >= std::max(650.0, config_.idleRpm * 0.82)) {
                dynoSweeping_ = true;
                dynoElapsed_ = 0.0;
                nextSampleRpm_ = dynoHoldEnabled_.load(std::memory_order_relaxed)
                    ? dynoHoldRpm_.load(std::memory_order_relaxed) : std::max(1'000.0, config_.idleRpm);
                dynoTargetRpm_ = nextSampleRpm_;
                dynoStableElapsed_ = 0.0;
                dynoBrakeTorqueNm_ = 0.0;
                dynoControllerIntegralNm_ = 0.0;
                dynoFeedForwardTorqueNm_ = std::max(0.0, simulator_.state().torqueNm);
                dynoFilteredRpm_ = simulator_.state().rpm;
                dynoFilteredAccelerationRpmPerSecond_ = 0.0;
                dynoTorqueAccumulator_ = 0.0;
                dynoPowerAccumulator_ = 0.0;
                dynoSampleCount_ = 0;
                starter_.store(false);
                throttle_.store(1.0);
            }
            if (dynoSweeping_) {
                dynoElapsed_ += baseStep.count();
                dynoElapsed = dynoElapsed_;
                const auto previousFilteredRpm = dynoFilteredRpm_;
                // Control against cycle-scale speed rather than individual firing
                // pulses.  A two-cylinder engine can otherwise look permanently
                // unstable even when its mean speed is stationary.
                dynoFilteredRpm_ += (simulator_.state().rpm - dynoFilteredRpm_)
                    * (1.0 - std::exp(-baseStep.count() * 3.0));
                const auto rawAcceleration = (dynoFilteredRpm_ - previousFilteredRpm)
                    / baseStep.count();
                dynoFilteredAccelerationRpmPerSecond_ +=
                    (rawAcceleration - dynoFilteredAccelerationRpmPerSecond_)
                    * (1.0 - std::exp(-baseStep.count() * 2.0));
                if (dynoHoldEnabled_.load(std::memory_order_relaxed))
                    dynoTargetRpm_ = dynoHoldRpm_.load(std::memory_order_relaxed);
                const auto error = dynoFilteredRpm_ - dynoTargetRpm_;
                const auto displacementM3 = engineDisplacementLitres(config_) * 0.001;
                // Controller scaling follows a strong naturally aspirated engine,
                // while the absorber itself has four times that capacity. A dyno
                // must be able to pull an engine below its torque peak; its limit
                // is an equipment rating, not a prediction of engine output.
                constexpr double controllerBrakeMeanEffectivePressurePa = 2'500'000.0;
                constexpr double absorberBrakeMeanEffectivePressurePa = 10'000'000.0;
                const auto controllerTorqueScaleNm = controllerBrakeMeanEffectivePressurePa
                    * displacementM3 / (4.0 * std::numbers::pi);
                const auto maximumBrakeTorqueNm = absorberBrakeMeanEffectivePressurePa
                    * displacementM3 / (4.0 * std::numbers::pi);
                dynoFeedForwardTorqueNm_ += (std::max(0.0, simulator_.state().torqueNm)
                    - dynoFeedForwardTorqueNm_) * (1.0 - std::exp(-baseStep.count() * 5.0));
                const auto proportionalGain = controllerTorqueScaleNm / 500.0;
                const auto integralGain = controllerTorqueScaleNm / 1'200.0;
                const auto accelerationGain = controllerTorqueScaleNm / 6'000.0;
                // An absorption dyno is unidirectional, but enabling the whole
                // firing-pulse feed-forward at one hard threshold makes a
                // relaxation oscillator: the EA288 accelerated unloaded to
                // target-120 rpm, received the complete brake torque in one
                // tick, fell below the threshold and repeated forever. Ramp
                // the absorber into contact across the final 120 rpm instead.
                // Farther below the setpoint it remains completely unloaded,
                // preserving the low-inertia run-up protection.
                constexpr double dynoContactBandRpm = 120.0;
                const auto contactPhase = std::clamp(
                    (error + dynoContactBandRpm) / dynoContactBandRpm,
                    0.0, 1.0);
                const auto contactScale =
                    contactPhase * contactPhase * (3.0 - 2.0 * contactPhase);
                const auto contactedFeedForwardTorqueNm =
                    dynoFeedForwardTorqueNm_ * contactScale;
                if (error < -dynoContactBandRpm) {
                    dynoControllerIntegralNm_ = 0.0;
                    dynoBrakeTorqueNm_ = 0.0;
                } else {
                    const auto integralCandidate = dynoControllerIntegralNm_
                        + error * integralGain * baseStep.count();
                    const auto unsaturated = contactedFeedForwardTorqueNm
                        + integralCandidate
                        + error * proportionalGain
                        + dynoFilteredAccelerationRpmPerSecond_ * accelerationGain;
                    const auto saturated = std::clamp(unsaturated, 0.0, maximumBrakeTorqueNm);
                    if (unsaturated == saturated
                        || (unsaturated < 0.0 && error > 0.0)
                        || (unsaturated > maximumBrakeTorqueNm && error < 0.0))
                        dynoControllerIntegralNm_ = integralCandidate;
                    dynoControllerIntegralNm_ = std::clamp(
                        dynoControllerIntegralNm_, -maximumBrakeTorqueNm, maximumBrakeTorqueNm);
                    dynoBrakeTorqueNm_ = std::clamp(
                        contactedFeedForwardTorqueNm
                        + dynoControllerIntegralNm_ + error * proportionalGain
                        + dynoFilteredAccelerationRpmPerSecond_ * accelerationGain,
                        0.0, maximumBrakeTorqueNm);
                }
                requestedLoad = 0.0;
            } else {
                requestedLoad = 0.0;
                dynoBrakeTorqueNm_ = 0.0;
            }
        }
        const auto gearGeneration = gearCommandGeneration_.load(std::memory_order_acquire);
        if (gearGeneration != consumedGearGeneration) {
            consumedGearCommand = gear_.load(std::memory_order_relaxed);
            driveline_.requestGear(consumedGearCommand);
            consumedGearGeneration = gearGeneration;
        }
        if (!dynoActive_ && simulationDt > 0.0) {
            updateDriveline(simulationDt, simulator_.state(), requestedLoad);
            // Reflect automatic shifts without ever overwriting a concurrent UI
            // command. A new generation is consumed on the next simulation tick.
            if (gearCommandGeneration_.load(std::memory_order_acquire) == consumedGearGeneration) {
                const auto modelRequest = driveline_.requestedGear();
                auto expected = consumedGearCommand;
                if (gear_.compare_exchange_strong(expected, modelRequest,
                                                  std::memory_order_relaxed,
                                                  std::memory_order_relaxed))
                    consumedGearCommand = modelRequest;
            }
        }
        const auto torqueCutMultiplier = dynoActive_
            ? 1.0 : drivelineOutput_.torqueCutMultiplier;
        const EngineControls controls { ignition_.load(), starter_.load(),
            (dynoActive_ ? (dynoSweeping_ ? 1.0 : 0.18) : std::clamp(throttle_.load(), 0.0, 1.0))
                * torqueCutMultiplier,
            dynoActive_ ? requestedLoad : 0.0, dynoActive_ ? 0.0 : engineClutchTorqueNm_,
            dynoActive_ ? 0.0 : brakePressure_.load(std::memory_order_relaxed),
            dynoActive_ ? dynoBrakeTorqueNm_ : 0.0 };
        auto frame = simulationDt > 0.0 ? simulator_.step(simulationDt, controls) : SimulationFrame { simulator_.state() };
        if (simulationDt > 0.0) {
            const auto simulationStart = frame.state.simulationTimeSeconds - simulationDt;
            const auto publicationWindow = publicationTimeline.beginWindow(
                realtimeSeconds, baseStep.count());
            for (std::size_t index = 0; index < frame.firingEventCount; ++index) {
                frame.firingEvents[index].timeSeconds = publicationWindow.mapSimulationTime(
                    frame.firingEvents[index].timeSeconds, simulationStart, simulationDt);
            }
            CylinderPressureSample pressureSample;
            while (simulator_.tryPopCylinderPressureSample(pressureSample)) {
                pressureSample.timeSeconds = publicationWindow.mapSimulationTime(
                    pressureSample.timeSeconds, simulationStart, simulationDt);
                if (pressureSample.timeSeconds + 1.0e-12 >= nextPressurePublishTime) {
                    if (!pressureQueue_->tryPush(pressureSample))
                        droppedPressureSamples_.fetch_add(1, std::memory_order_relaxed);
                    nextPressurePublishTime = pressureSample.timeSeconds + minimumPressurePublishInterval;
                }
            }
            if (frame.droppedCylinderPressureSampleCount > 0)
                droppedPressureSamples_.fetch_add(frame.droppedCylinderPressureSampleCount,
                                                  std::memory_order_relaxed);
        }
        publishAudioFrame(audioState_, frame.state, {
            isPaused, controls.starterEngaged,
            std::abs(engineClutchTorqueNm_)
                / std::max(20.0, config_.transmission.maxClutchTorqueNm),
            dynoActive_ ? 1.0 : timeScale_.load(std::memory_order_relaxed) });
        if (dynoActive_ && dynoSweeping_) {
            if (std::abs(dynoFilteredRpm_ - dynoTargetRpm_) <= 60.0
                    && std::abs(dynoFilteredAccelerationRpmPerSecond_) <= 120.0) {
                dynoTorqueAccumulator_ += frame.state.loadTorqueNm;
                dynoPowerAccumulator_ += frame.state.loadTorqueNm
                    * frame.state.angularVelocityRadPerSecond / 1'000.0;
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
                    dynoTargetRpm_ = std::min(config_.redlineRpm, dynoTargetRpm_ + 250.0);
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
            frame.state.requestedRoadLoad = drivelineOutput_.requestedLoad;
            frame.state.roadLoadForceN = drivelineOutput_.roadLoadForceN;
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
        const auto maximumDynoDurationSeconds =
            dynoMaximumDurationSeconds_.load(std::memory_order_relaxed);
        const auto dynoStartupTimedOut = !dynoSweeping_.load(std::memory_order_relaxed)
            && dynoStartupElapsed >= maximumDynoDurationSeconds;
        if (dynoActive_ && (dynoCompleted_.load(std::memory_order_relaxed)
            || dynoElapsed >= maximumDynoDurationSeconds || dynoStartupTimedOut)) {
            dynoRequestedRunning_.store(false, std::memory_order_release);
            finishDynoSession();
        }
        const auto now = Clock::now();
        const auto workFraction =
            std::chrono::duration<double>(now - iterationStart).count()
            / baseStep.count();
        const auto producerTime = std::chrono::duration<double>(now - clockEpoch).count();
        audioState_.producerTimeNanoseconds.store(static_cast<std::uint64_t>(std::max(0.0, producerTime) * 1.0e9),
                                                  std::memory_order_release);
        // Instrumentation escape hatch (see setRealtimeThrottleEnabled): with
        // the throttle off the loop free-runs, so the realtime factor stops
        // saturating at 1.0 and reads as capacity instead. The deadline is
        // carried forward to `now` so the overrun counter does not fill with
        // self-inflicted lateness that means nothing in this mode.
        if (!realtimeThrottleEnabled_.load(std::memory_order_relaxed)) {
            if (loadGovernor.reset()) {
                simulator_.setIntakeWallHeatUpdateIntervalSeconds(
                    normalIntakeWallHeatIntervalSeconds);
                realtimeLoadProtectionActive_.store(
                    false, std::memory_order_relaxed);
            }
            deadline = now;
            continue;
        }
        const auto missedDeadline = now > deadline;
        const auto protectionAllowed =
            realtimeLoadProtectionEnabled_.load(std::memory_order_relaxed)
            && !dynoActive_ && !isPaused;
        if (!protectionAllowed) {
            if (loadGovernor.reset()) {
                simulator_.setIntakeWallHeatUpdateIntervalSeconds(
                    normalIntakeWallHeatIntervalSeconds);
                realtimeLoadProtectionActive_.store(
                    false, std::memory_order_relaxed);
            }
        } else if (loadGovernor.observe(missedDeadline, workFraction)) {
            simulator_.setIntakeWallHeatUpdateIntervalSeconds(
                loadGovernor.active()
                    ? protectedIntakeWallHeatIntervalSeconds
                    : normalIntakeWallHeatIntervalSeconds);
            realtimeLoadProtectionActive_.store(
                loadGovernor.active(), std::memory_order_relaxed);
            if (loadGovernor.active())
                realtimeLoadProtectionActivations_.fetch_add(
                    1, std::memory_order_relaxed);
        }
        if (missedDeadline) {
            timingOverruns_.fetch_add(1, std::memory_order_relaxed);
            const auto lateness = std::chrono::duration<double>(now - deadline).count();
            auto previousMaximum = maximumTimingLatenessSeconds_.load(std::memory_order_relaxed);
            while (lateness > previousMaximum
                && !maximumTimingLatenessSeconds_.compare_exchange_weak(
                    previousMaximum, lateness, std::memory_order_relaxed)) {}
        }
        if (now > deadline + std::chrono::duration_cast<Clock::duration>(baseStep * 4.0)) {
            deadline = now;
        }
        std::this_thread::sleep_until(deadline);
    }
}
} // namespace enginelab
