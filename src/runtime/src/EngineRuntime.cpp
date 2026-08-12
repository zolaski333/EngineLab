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

/** A DynoPoint with every field at zero, for use as a running sum.
 *
 * `DynoPoint {}` is NOT this: several of its members default to physical
 * neutral values (14.7 AFR, 22 degC, lambda 1.0) which would be added into the
 * first sample and then divided by the sample count. */
[[nodiscard]] DynoPoint zeroedDynoAccumulator() noexcept {
    DynoPoint zero;
    zero.rpm = 0.0;
    zero.torqueNm = 0.0;
    zero.powerKw = 0.0;
    zero.airFuelRatio = 0.0;
    zero.coolantTemperatureC = 0.0;
    zero.exhaustTemperatureC = 0.0;
    zero.ignitionAdvanceDegrees = 0.0;
    zero.atmosphericCorrectionFactor = 0.0;
    zero.correctedTorqueNm = 0.0;
    zero.correctedPowerKw = 0.0;
    zero.targetAirFuelRatio = 0.0;
    zero.volumetricEfficiency = 0.0;
    zero.fuelFlowGramsPerSecond = 0.0;
    zero.manifoldPressureKpa = 0.0;
    zero.exhaustPressureKpa = 0.0;
    zero.oilTemperatureC = 0.0;
    zero.oilPressureKpa = 0.0;
    zero.airFlowGramsPerSecond = 0.0;
    zero.lambda = 0.0;
    zero.brakeSpecificFuelConsumptionGPerKwh = 0.0;
    return zero;
}

/** Highest speed a swept bench run is allowed to sample.
 *
 * The sweep used to step to `redlineRpm` inclusive, so its last point or two
 * sat on the latched rev limiter -- where a cut spark zeroes the published
 * combustion efficiency and the torque reading becomes an artefact rather than
 * a measurement (docs/physics-audit.md records a Merlin row of 351 Nm one step
 * after 2521 Nm). Both offline WOT instruments already stop at 0.95 of the
 * lower of the two limits; the bench the application drives did not, and its
 * curves collapsed at the top for that reason alone. */
[[nodiscard]] double sweepCeilingRpm(const EngineConfig& config) noexcept {
    return 0.95 * std::min(config.redlineRpm, config.ignition.revLimitRpm);
}

[[nodiscard]] double sweepEntryRpm(const EngineConfig& config) noexcept {
    const auto lowCylinderEntry = config.cylinders.size() <= 2U
        ? 1'800.0 : 0.0;
    return std::min(sweepCeilingRpm(config),
        std::max({ 1'000.0, config.idleRpm + 400.0,
                   lowCylinderEntry }));
}

[[nodiscard]] double moveTowards(
    double current, double target, double maximumDelta) noexcept {
    if (current < target)
        return std::min(target, current + maximumDelta);
    return std::max(target, current - maximumDelta);
}

/** Averaging window for one bench point, in seconds.
 *
 * Torque is periodic at the firing frequency, so a window that is not a whole
 * number of firing periods leaves a residual set by where the window happened
 * to start and stop. The window is driven at the 1/240 s runtime step and the
 * firing period is comparable to that step at high speed, so it cannot be
 * aligned exactly; what is available is length, since the residual falls as
 * 1/N in the number of periods covered. The old floor of 0.12 s was chosen as
 * two engine CYCLES, which on a slow twin is only a handful of firings. */
[[nodiscard]] double dynoAveragingWindowSeconds(
    const EngineConfig& config, double targetRpm) noexcept {
    const auto cylinders = std::max<std::size_t>(1U, config.cylinders.size());
    const auto firingPeriodSeconds = 120.0
        / (std::max(250.0, targetRpm) * static_cast<double>(cylinders));
    constexpr double minimumFirings = 48.0;
    return std::clamp(minimumFirings * firingPeriodSeconds, 0.25, 0.80);
}

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
                 std::move(simulatorOptions)), dynoAbsorber_(config_),
      driveline_(config_),
      pressureQueue_(std::make_unique<CylinderPressureQueue>()),
      exhaustAcousticQueue_(std::make_unique<ExhaustAcousticQueue>()) {
    simulator_.setPressureSamplingEnabled(true);
    applyAudioVoicing(config_.audioVoicing);
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

void EngineRuntime::applyAudioPhysicsCalibration(
    const AudioPhysicsCalibration& calibration) noexcept {
    {
        const std::scoped_lock lock(audioPhysicsCalibrationMutex_);
        pendingAudioPhysicsCalibration_ = calibration;
    }
    audioPhysicsCalibrationRevision_.fetch_add(1, std::memory_order_release);
}

void EngineRuntime::applyAudioVoicing(const AudioVoicingConfig& voicing) noexcept {
    setAudioVolume(voicing.volume);
    setAudioConvolution(voicing.convolution);
    setHighFrequencyGain(voicing.highFrequencyGain);
    setLowFrequencyGain(voicing.lowFrequencyGain);
    setLowFrequencyNoise(voicing.lowFrequencyNoise);
    setHighFrequencyNoise(voicing.highFrequencyNoise);
    setCombustionGain(voicing.combustionGain);
    setExhaustGain(voicing.exhaustGain);
    setIntakeGain(voicing.intakeGain);
    setMechanicalGain(voicing.mechanicalGain);
    setStereoWidth(voicing.stereoWidth);
    setOutletJetGain(voicing.outletJetGain);
    setSaturationDrive(voicing.saturationDrive);
    setSaturationPlacement(voicing.saturationPlacement);
    audioState_.monitorMode.store(static_cast<int>(voicing.monitorMode),
                                  std::memory_order_relaxed);
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
    engineCouplingTorqueNm_ = drivelineOutput_.engineCouplingTorqueNm;
    drivelineReflectedInertiaKgM2_ =
        drivelineOutput_.reflectedRotatingInertiaKgM2;
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
    nextSampleRpm_ = sweepEntryRpm(config_);
    dynoTargetRpm_ = nextSampleRpm_;
    dynoPreparationDestinationRpm_ =
        dynoHoldEnabled_.load(std::memory_order_relaxed)
        ? dynoHoldRpm_.load(std::memory_order_relaxed)
        : nextSampleRpm_;
    const auto runningThreshold =
        std::max(650.0, config_.idleRpm * 0.82);
    const auto currentRpm = simulator_.state().rpm;
    dynoPreparationTargetRpm_ = currentRpm >= runningThreshold
        ? currentRpm : dynoPreparationDestinationRpm_;
    dynoPullDownRequired_ = currentRpm
        > dynoPreparationDestinationRpm_ + 150.0;
    dynoThrottleCommand_ = 0.18;
    dynoRecoveryCount_ = 0;
    dynoStableElapsed_ = 0.0;
    dynoBrakeTorqueNm_ = 0.0;
    dynoAbsorber_.reset(
        simulator_.state().rpm, simulator_.state().torqueNm);
    dynoAbsorberOutput_ = {};
    dynoChannelAccumulator_ = zeroedDynoAccumulator();
    dynoSampleCount_ = 0;
    savedIgnition_ = ignition_.load();
    savedStarter_ = starter_.load();
    savedThrottle_ = throttle_.load();
    savedLoad_ = load_.load();
    ignition_.store(true);
    starter_.store(currentRpm < runningThreshold);
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
    // Delivered pace of this thread, published for the UI. Counting ITERATIONS
    // rather than simulated seconds is deliberate: `simulationDt` carries
    // `timeScale_` and goes to zero on pause, so a user who asked for slow
    // motion, or who paused, would otherwise be shown the same reading as an
    // engine the machine cannot keep up with. One iteration always owes one
    // `baseStep` of wall time, whatever it chose to simulate inside it.
    auto paceWindowStart = clockEpoch;
    auto paceWindowIterations = std::uint64_t { 0 };
    constexpr auto paceWindowSeconds = 0.25;
    auto consumedGearGeneration = std::uint64_t { 0 };
    auto consumedGearCommand = gear_.load(std::memory_order_relaxed);
    auto consumedAudioPhysicsRevision = std::uint64_t { 0 };
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
            const auto& dynoState = simulator_.state();
            const auto runningThreshold =
                std::max(650.0, config_.idleRpm * 0.82);
            if (!dynoSweeping_) {
                if (dynoHoldEnabled_.load(std::memory_order_relaxed))
                    dynoPreparationDestinationRpm_ =
                        dynoHoldRpm_.load(std::memory_order_relaxed);

                if (dynoState.rpm < runningThreshold) {
                    // A stopped engine is started without an absorber load.
                    // Re-resetting here also discards any brake integral left
                    // by a failed approach before the starter tries again.
                    starter_.store(true);
                    dynoThrottleCommand_ = 0.18;
                    dynoBrakeTorqueNm_ = 0.0;
                    dynoAbsorber_.reset(dynoState.rpm, 0.0);
                    dynoAbsorberOutput_ = {};
                } else {
                    starter_.store(false);
                    // Never connect an already-running engine directly to the
                    // 1,000 rpm point. Slew the absorber command down from the
                    // actual shaft speed, just like an operator progressively
                    // loads a water/eddy-current brake before beginning a run.
                    constexpr double preparationSlewRpmPerSecond = 700.0;
                    dynoPreparationTargetRpm_ = moveTowards(
                        dynoPreparationTargetRpm_,
                        dynoPreparationDestinationRpm_,
                        preparationSlewRpmPerSecond * baseStep.count());
                    // Once the engine has caught, keep the run at the same
                    // WOT operating condition used for every measured point.
                    // Trying to stage at part throttle fights the ECU's idle
                    // bypass on small engines and makes the one-way absorber
                    // alternately contact and release. The progressively
                    // slewed brake target already provides the gentle entry.
                    dynoThrottleCommand_ = 1.0;
                    dynoAbsorberOutput_ = dynoAbsorber_.advance(
                        baseStep.count(), dynoPreparationTargetRpm_,
                        dynoState, 1'000.0);
                    dynoBrakeTorqueNm_ =
                        dynoAbsorberOutput_.brakeTorqueNm;
                    // Starting below the first point follows the already
                    // validated WOT acquisition path immediately. Preparation
                    // exists specifically for the dangerous case: connecting
                    // a high-revving shaft to a 1,000 rpm target. In that case
                    // wait until the slewed controller has brought the filtered
                    // shaft close, then hand over to the precise 60 rpm hold.
                    const auto pullDownComplete =
                        std::abs(dynoPreparationTargetRpm_
                            - dynoPreparationDestinationRpm_) <= 1.0
                        && dynoAbsorberOutput_.filteredRpm
                            <= dynoPreparationDestinationRpm_ + 100.0;
                    if (!dynoPullDownRequired_ || pullDownComplete) {
                        dynoSweeping_ = true;
                        nextSampleRpm_ =
                            dynoPreparationDestinationRpm_;
                        dynoTargetRpm_ = nextSampleRpm_;
                        dynoStableElapsed_ = 0.0;
                        dynoBrakeTorqueNm_ = 0.0;
                        dynoAbsorber_.reset(
                            dynoState.rpm,
                            dynoState.cycleAveragedTorqueNm);
                        dynoAbsorberOutput_ = {};
                        dynoChannelAccumulator_ =
                            zeroedDynoAccumulator();
                        dynoSampleCount_ = 0;
                        starter_.store(false);
                        dynoThrottleCommand_ = 1.0;
                    }
                }
                requestedLoad = 0.0;
            }
            if (dynoSweeping_) {
                dynoElapsed_ += baseStep.count();
                dynoElapsed = dynoElapsed_;
                if (dynoHoldEnabled_.load(std::memory_order_relaxed)) {
                    // Wheel changes in hold mode are rate limited too; a large
                    // setpoint change cannot become another brake step.
                    constexpr double holdSlewRpmPerSecond = 700.0;
                    dynoTargetRpm_ = moveTowards(
                        dynoTargetRpm_,
                        dynoHoldRpm_.load(std::memory_order_relaxed),
                        holdSlewRpmPerSecond * baseStep.count());
                } else if (dynoRampEnabled_.load(std::memory_order_relaxed)) {
                    // The continuous ramp, and the single rule that makes it
                    // safe on any engine without per-engine tuning: it advances
                    // ONLY while the engine is genuinely producing torque, and
                    // decays otherwise. The target therefore cannot run away
                    // from an engine that has stopped following it. The stepped
                    // sweep has no such property -- every 250 rpm step is a
                    // setpoint edge the engine may fail to climb, which is what
                    // the recovery path below exists to catch.
                    //
                    // Threshold is ES2D's 1 ft-lb, i.e. "still pushing at all",
                    // not a torque figure to tune.
                    constexpr double rampTorqueThresholdNm = 1.4;
                    const auto rampCeilingRpm = sweepCeilingRpm(config_);
                    if (dynoState.cycleAveragedTorqueNm > rampTorqueThresholdNm)
                        dynoTargetRpm_ = std::min(rampCeilingRpm,
                            dynoTargetRpm_
                                + dynoRampRpmPerSecond_.load(std::memory_order_relaxed)
                                    * baseStep.count());
                    else
                        dynoTargetRpm_ /= (1.0 + baseStep.count());
                }
                const auto recoveryThreshold =
                    std::max(350.0, config_.idleRpm * 0.55);
                if (dynoState.rpm < recoveryThreshold) {
                    // An unexpected weak point must not turn into a permanent
                    // stall. Release the brake, restart if required, and
                    // approach the same target progressively before retrying.
                    dynoSweeping_ = false;
                    ++dynoRecoveryCount_;
                    dynoStartupElapsed_ = 0.0;
                    dynoStartupElapsed = 0.0;
                    dynoPreparationDestinationRpm_ = dynoTargetRpm_;
                    dynoPreparationTargetRpm_ = dynoState.rpm;
                    dynoPullDownRequired_ = false;
                    dynoBrakeTorqueNm_ = 0.0;
                    dynoAbsorber_.reset(dynoState.rpm, 0.0);
                    dynoAbsorberOutput_ = {};
                    dynoChannelAccumulator_ = zeroedDynoAccumulator();
                    dynoSampleCount_ = 0;
                    dynoStableElapsed_ = 0.0;
                    dynoThrottleCommand_ = 0.18;
                    starter_.store(true);
                } else {
                    dynoThrottleCommand_ = 1.0;
                    dynoAbsorberOutput_ = dynoAbsorber_.advance(
                        baseStep.count(), dynoTargetRpm_, dynoState);
                    dynoBrakeTorqueNm_ =
                        dynoAbsorberOutput_.brakeTorqueNm;
                }
                requestedLoad = 0.0;
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
        const auto requestedAudioPhysicsRevision =
            audioPhysicsCalibrationRevision_.load(std::memory_order_acquire);
        if (requestedAudioPhysicsRevision != consumedAudioPhysicsRevision) {
            AudioPhysicsCalibration calibration;
            {
                const std::scoped_lock lock(audioPhysicsCalibrationMutex_);
                calibration = pendingAudioPhysicsCalibration_;
            }
            simulator_.applyAudioPhysicsCalibration(calibration);
            consumedAudioPhysicsRevision = requestedAudioPhysicsRevision;
        }
        const EngineControls controls { ignition_.load(), starter_.load(),
            (dynoActive_ ? dynoThrottleCommand_
                         : std::clamp(throttle_.load(), 0.0, 1.0))
                * torqueCutMultiplier,
            dynoActive_ ? requestedLoad : 0.0, dynoActive_ ? 0.0 : engineCouplingTorqueNm_,
            dynoActive_ ? 0.0 : brakePressure_.load(std::memory_order_relaxed),
            dynoActive_ ? dynoBrakeTorqueNm_ : 0.0,
            dynoActive_ ? 0.0 : drivelineReflectedInertiaKgM2_ };
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
            ExhaustAcousticSample acousticSample;
            while (simulator_.tryPopExhaustAcousticSample(acousticSample)) {
                acousticSample.timeSeconds = publicationWindow.mapSimulationTime(
                    acousticSample.timeSeconds, simulationStart, simulationDt);
                for (std::size_t eventIndex = 0;
                     eventIndex < acousticSample.reactionEventCount; ++eventIndex) {
                    acousticSample.reactionEvents[eventIndex].timeSeconds =
                        publicationWindow.mapSimulationTime(
                            acousticSample.reactionEvents[eventIndex].timeSeconds,
                            simulationStart, simulationDt);
                }
                if (!exhaustAcousticQueue_->tryPush(acousticSample))
                    droppedExhaustAcousticSamples_.fetch_add(
                        1, std::memory_order_relaxed);
            }
            if (frame.droppedCylinderPressureSampleCount > 0)
                droppedPressureSamples_.fetch_add(frame.droppedCylinderPressureSampleCount,
                                                  std::memory_order_relaxed);
            if (frame.droppedExhaustAcousticSampleCount > 0)
                droppedExhaustAcousticSamples_.fetch_add(
                    frame.droppedExhaustAcousticSampleCount,
                    std::memory_order_relaxed);
        }
        publishAudioFrame(audioState_, frame.state, {
            isPaused, controls.starterEngaged,
            std::abs(engineClutchTorqueNm_)
                / std::max(20.0, config_.transmission.maxClutchTorqueNm),
            dynoActive_ ? 1.0 : timeScale_.load(std::memory_order_relaxed) });
        if (dynoActive_ && dynoSweeping_) {
            // A ramp is a TRANSIENT by construction, so the stepped sweep's
            // settling gate would reject every sample: it demands under
            // 120 rpm/s of acceleration while the ramp commands 500 by design.
            // What still has to hold is that the engine is TRACKING the target
            // rather than being dragged behind it, so the speed-error gate
            // stays and only the acceleration bound follows the commanded rate.
            const auto rampingSweep =
                dynoRampEnabled_.load(std::memory_order_relaxed)
                && !dynoHoldEnabled_.load(std::memory_order_relaxed);
            const auto acceptedSpeedErrorRpm = rampingSweep ? 150.0 : 60.0;
            const auto acceptedAccelerationRpmPerSecond = rampingSweep
                ? std::max(120.0, 3.0 * dynoRampRpmPerSecond_.load(
                    std::memory_order_relaxed))
                : 120.0;
            if (std::abs(dynoAbsorberOutput_.filteredRpm
                    - dynoTargetRpm_) <= acceptedSpeedErrorRpm
                    && std::abs(
                        dynoAbsorberOutput_
                            .filteredAccelerationRpmPerSecond)
                        <= acceptedAccelerationRpmPerSecond) {
                // The ENGINE's brake torque, not the absorber's brake command.
                //
                // `loadTorqueNm` is the dyno controller's output, and the
                // controller deliberately ramps its feed-forward term in over
                // the final 60 rpm of approach (DynoAbsorberController's
                // `contactScale`, a smoothstep) -- across exactly the +-60 rpm
                // this gate accepts a sample within. A point that settled 40 rpm
                // low therefore published about a quarter of the feed-forward
                // and a point that settled dead on published all of it, so the
                // curve rose and fell with where the engine happened to land in
                // the acceptance band rather than with what it produced. That is
                // the "rollercoaster". `state_.torqueNm` is the engine's own
                // brake torque and carries none of the controller's shape; it is
                // also what `DynoSweepHarness` averages, so the bench in the
                // application and the instrument the catalogue is validated
                // against now measure the same quantity.
                dynoChannelAccumulator_.rpm += frame.state.rpm;
                dynoChannelAccumulator_.torqueNm += frame.state.torqueNm;
                dynoChannelAccumulator_.powerKw += frame.state.torqueNm
                    * frame.state.angularVelocityRadPerSecond / 1'000.0;
                dynoChannelAccumulator_.airFuelRatio += frame.state.airFuelRatio;
                dynoChannelAccumulator_.coolantTemperatureC += frame.state.coolantTemperatureC;
                dynoChannelAccumulator_.exhaustTemperatureC += frame.state.exhaustTemperatureC;
                dynoChannelAccumulator_.ignitionAdvanceDegrees += frame.state.ignitionAdvanceDegrees;
                dynoChannelAccumulator_.targetAirFuelRatio += frame.state.targetAirFuelRatio;
                dynoChannelAccumulator_.volumetricEfficiency += frame.state.volumetricEfficiency;
                dynoChannelAccumulator_.fuelFlowGramsPerSecond += frame.state.fuelFlowGramsPerSecond;
                dynoChannelAccumulator_.manifoldPressureKpa += frame.state.manifoldPressureKpa;
                dynoChannelAccumulator_.exhaustPressureKpa += frame.state.exhaustBackPressureKpa;
                dynoChannelAccumulator_.oilTemperatureC += frame.state.oilTemperatureC;
                dynoChannelAccumulator_.oilPressureKpa += frame.state.oilPressureKpa;
                dynoChannelAccumulator_.airFlowGramsPerSecond += frame.state.airFlowGramsPerSecond;
                dynoChannelAccumulator_.lambda += frame.state.lambda;
                dynoChannelAccumulator_.brakeSpecificFuelConsumptionGPerKwh +=
                    frame.state.brakeSpecificFuelConsumptionGPerKwh;
                ++dynoSampleCount_;
                dynoStableElapsed_ += baseStep.count();
            } else {
                dynoChannelAccumulator_ = zeroedDynoAccumulator();
                dynoSampleCount_ = 0;
                dynoStableElapsed_ = 0.0;
            }
            const auto minimumCycleWindowSeconds =
                dynoAveragingWindowSeconds(config_, dynoTargetRpm_);
            if (dynoStableElapsed_ >= minimumCycleWindowSeconds && dynoSampleCount_ > 0) {
                const auto divisor = static_cast<double>(dynoSampleCount_);
                const auto atmosphericCorrection = std::clamp((99.0 / config_.ambientPressureKpa)
                    * std::sqrt((config_.ambientTemperatureC + 273.15) / 298.15), 0.80, 1.20);
                const auto meanTorqueNm = dynoChannelAccumulator_.torqueNm / divisor;
                const auto meanPowerKw = dynoChannelAccumulator_.powerKw / divisor;
                // The speed actually held, not the speed asked for. The gate
                // tolerates 60 rpm of placement error, so labelling the point
                // with its target put a real measurement at a false abscissa --
                // on a steep part of the curve that alone is several Nm of
                // apparent scatter.
                DynoPoint point { dynoChannelAccumulator_.rpm / divisor, meanTorqueNm,
                    meanPowerKw, dynoChannelAccumulator_.airFuelRatio / divisor,
                    dynoChannelAccumulator_.coolantTemperatureC / divisor,
                    dynoChannelAccumulator_.exhaustTemperatureC / divisor,
                    dynoChannelAccumulator_.ignitionAdvanceDegrees / divisor, atmosphericCorrection,
                    meanTorqueNm * atmosphericCorrection,
                    meanPowerKw * atmosphericCorrection };
                point.targetAirFuelRatio = dynoChannelAccumulator_.targetAirFuelRatio / divisor;
                point.volumetricEfficiency = dynoChannelAccumulator_.volumetricEfficiency / divisor;
                point.fuelFlowGramsPerSecond = dynoChannelAccumulator_.fuelFlowGramsPerSecond / divisor;
                point.manifoldPressureKpa = dynoChannelAccumulator_.manifoldPressureKpa / divisor;
                point.exhaustPressureKpa = dynoChannelAccumulator_.exhaustPressureKpa / divisor;
                point.oilTemperatureC = dynoChannelAccumulator_.oilTemperatureC / divisor;
                point.oilPressureKpa = dynoChannelAccumulator_.oilPressureKpa / divisor;
                point.airFlowGramsPerSecond = dynoChannelAccumulator_.airFlowGramsPerSecond / divisor;
                point.lambda = dynoChannelAccumulator_.lambda / divisor;
                point.brakeSpecificFuelConsumptionGPerKwh =
                    dynoChannelAccumulator_.brakeSpecificFuelConsumptionGPerKwh / divisor;
                {
                    const std::scoped_lock lock(dynoMutex_);
                    currentRun_.points.push_back(point);
                    currentRun_.peakTorqueNm = std::max(currentRun_.peakTorqueNm, point.torqueNm);
                    currentRun_.peakPowerKw = std::max(currentRun_.peakPowerKw, point.powerKw);
                    currentRun_.peakCorrectedTorqueNm = std::max(currentRun_.peakCorrectedTorqueNm, point.correctedTorqueNm);
                    currentRun_.peakCorrectedPowerKw = std::max(currentRun_.peakCorrectedPowerKw, point.correctedPowerKw);
                }
                // Completion is tested on the TARGET, never on the published
                // rpm: the published value is now the mean speed actually held
                // and can sit just under the ceiling forever, which would leave
                // the sweep running with nowhere left to step.
                const auto ceilingRpm = sweepCeilingRpm(config_);
                if (!dynoHoldEnabled_.load(std::memory_order_relaxed)
                        && dynoTargetRpm_ >= ceilingRpm - 1.0e-6)
                    dynoCompleted_.store(true, std::memory_order_relaxed);
                // The ramp advances every iteration up in the sweep block, so
                // publishing a point must NOT also step it -- doing both would
                // make the bench jump 250 rpm every window on top of the ramp.
                if (!dynoHoldEnabled_.load(std::memory_order_relaxed)
                        && !rampingSweep)
                    dynoTargetRpm_ = std::min(ceilingRpm, dynoTargetRpm_ + 250.0);
                nextSampleRpm_ = dynoTargetRpm_;
                dynoChannelAccumulator_ = zeroedDynoAccumulator();
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
            frame.state.drivelineReflectedInertiaKgM2 =
                drivelineReflectedInertiaKgM2_;
            frame.state.shiftProgress = shiftProgress_;
            frame.state.shiftInProgress = shiftInProgress_;
            frame.state.brakePressure = drivelineOutput_.brakePressure;
            frame.state.brakeForceN = drivelineOutput_.brakeForceN;
            frame.state.requestedRoadLoad = drivelineOutput_.requestedLoad;
            frame.state.roadLoadForceN = drivelineOutput_.roadLoadForceN;
            frame.state.tireLongitudinalForceN = drivelineOutput_.tireForceN;
            frame.state.drivenAxleNormalForceN =
                drivelineOutput_.drivenAxleNormalForceN;
            frame.state.longitudinalAccelerationMps2 =
                drivelineOutput_.longitudinalAccelerationMps2;
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
            frame.state.dynoRampEnabled = dynoRampEnabled_.load(std::memory_order_relaxed);
            frame.state.dynoRampRpmPerSecond =
                dynoRampRpmPerSecond_.load(std::memory_order_relaxed);
            frame.state.dynoActive = dynoActive_;
            frame.state.dynoPreparing = dynoActive_
                && !dynoSweeping_.load(std::memory_order_relaxed);
            frame.state.dynoTargetRpm = frame.state.dynoPreparing
                ? dynoPreparationDestinationRpm_ : dynoTargetRpm_;
            frame.state.dynoControllerTargetRpm = frame.state.dynoPreparing
                ? dynoPreparationTargetRpm_ : dynoTargetRpm_;
            frame.state.dynoFilteredAccelerationRpmPerSecond =
                dynoAbsorberOutput_.filteredAccelerationRpmPerSecond;
            frame.state.dynoBrakeTorqueNm = dynoBrakeTorqueNm_;
            const auto sweepStartRpm = sweepEntryRpm(config_);
            const auto sweepRangeRpm = std::max(
                1.0, sweepCeilingRpm(config_) - sweepStartRpm);
            frame.state.dynoProgress = dynoHoldEnabled_.load(
                std::memory_order_relaxed)
                ? (frame.state.dynoPreparing ? 0.0 : 1.0)
                : std::clamp((dynoTargetRpm_ - sweepStartRpm)
                    / sweepRangeRpm, 0.0, 1.0);
            frame.state.dynoRecoveryCount = dynoRecoveryCount_;
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
        // Accounted before the free-run escape below, so the instrumentation
        // mode reports capacity by the same arithmetic the throttled mode
        // reports delivery -- the throttle is the only difference between them.
        ++paceWindowIterations;
        if (const auto paceElapsed =
                std::chrono::duration<double>(now - paceWindowStart).count();
            paceElapsed >= paceWindowSeconds) {
            realtimeFactor_.store(
                static_cast<double>(paceWindowIterations) * baseStep.count()
                    / paceElapsed,
                std::memory_order_relaxed);
            paceWindowStart = now;
            paceWindowIterations = 0;
        }
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
