#include <enginelab/runtime/EngineRuntime.hpp>
#include <enginelab/runtime/MonotonicPublicationTimeline.hpp>
#include <algorithm>
#include <chrono>
#include <limits>
#include <numbers>
#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <Windows.h>
#endif

namespace enginelab {
namespace {
constexpr std::size_t maximumAudioExhaustPaths = 8;

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

[[nodiscard]] double interpolate(double left, double right,
                                 double fraction) noexcept {
    return left + (right - left) * std::clamp(fraction, 0.0, 1.0);
}

[[nodiscard]] DynoCycleTelemetry dynoTelemetry(
    const EngineState& state) noexcept {
    return {
        state.airFuelRatio,
        state.coolantTemperatureC,
        state.exhaustTemperatureC,
        state.ignitionAdvanceDegrees,
        state.targetAirFuelRatio,
        state.volumetricEfficiency,
        state.fuelFlowGramsPerSecond,
        state.manifoldPressureKpa,
        state.exhaustBackPressureKpa,
        state.oilTemperatureC,
        state.oilPressureKpa,
        state.airFlowGramsPerSecond,
        state.lambda,
        state.brakeSpecificFuelConsumptionGPerKwh,
    };
}

[[nodiscard]] DynoCycleTelemetry interpolateTelemetry(
    const DynoCycleTelemetry& left, const DynoCycleTelemetry& right,
    double fraction) noexcept {
    return {
        interpolate(left.airFuelRatio, right.airFuelRatio, fraction),
        interpolate(left.coolantTemperatureC, right.coolantTemperatureC, fraction),
        interpolate(left.exhaustTemperatureC, right.exhaustTemperatureC, fraction),
        interpolate(left.ignitionAdvanceDegrees, right.ignitionAdvanceDegrees, fraction),
        interpolate(left.targetAirFuelRatio, right.targetAirFuelRatio, fraction),
        interpolate(left.volumetricEfficiency, right.volumetricEfficiency, fraction),
        interpolate(left.fuelFlowGramsPerSecond, right.fuelFlowGramsPerSecond, fraction),
        interpolate(left.manifoldPressureKpa, right.manifoldPressureKpa, fraction),
        interpolate(left.exhaustPressureKpa, right.exhaustPressureKpa, fraction),
        interpolate(left.oilTemperatureC, right.oilTemperatureC, fraction),
        interpolate(left.oilPressureKpa, right.oilPressureKpa, fraction),
        interpolate(left.airFlowGramsPerSecond, right.airFlowGramsPerSecond, fraction),
        interpolate(left.lambda, right.lambda, fraction),
        interpolate(left.brakeSpecificFuelConsumptionGPerKwh,
                    right.brakeSpecificFuelConsumptionGPerKwh, fraction),
    };
}

[[nodiscard]] DynoWindowEstimate interpolateEstimate(
    const DynoWindowEstimate& left, const DynoWindowEstimate& right,
    double fraction, double meanRpm) noexcept {
    DynoWindowEstimate result = right;
    result.firstCycleId = right.firstCycleId;
    result.lastCycleId = right.lastCycleId;
    result.cycleCount = right.cycleCount;
    result.durationSeconds = interpolate(
        left.durationSeconds, right.durationSeconds, fraction);
    result.brakeWorkJoules = interpolate(
        left.brakeWorkJoules, right.brakeWorkJoules, fraction);
    result.integratedCrankRadians = interpolate(
        left.integratedCrankRadians, right.integratedCrankRadians, fraction);
    result.meanRpm = meanRpm;
    result.minRpm = interpolate(left.minRpm, right.minRpm, fraction);
    result.maxRpm = interpolate(left.maxRpm, right.maxRpm, fraction);
    result.meanTorqueNm = interpolate(
        left.meanTorqueNm, right.meanTorqueNm, fraction);
    result.meanPowerKw = interpolate(
        left.meanPowerKw, right.meanPowerKw, fraction);
    result.torqueVarianceNm2 = interpolate(
        left.torqueVarianceNm2, right.torqueVarianceNm2, fraction);
    result.meanTelemetry = interpolateTelemetry(
        left.meanTelemetry, right.meanTelemetry, fraction);
    result.continuous = left.continuous && right.continuous;
    result.capacityLimited = left.capacityLimited || right.capacityLimited;
    return result;
}

[[nodiscard]] DynoPoint makeDynoPoint(
    const EngineConfig& config, const DynoWindowEstimate& estimate,
    double coordinateRpm, bool fixedBin) noexcept {
    DynoPoint point;
    point.rpm = coordinateRpm;
    point.torqueNm = estimate.meanTorqueNm;
    point.powerKw = estimate.meanPowerKw;
    point.airFuelRatio = estimate.meanTelemetry.airFuelRatio;
    point.coolantTemperatureC = estimate.meanTelemetry.coolantTemperatureC;
    point.exhaustTemperatureC = estimate.meanTelemetry.exhaustTemperatureC;
    point.ignitionAdvanceDegrees = estimate.meanTelemetry.ignitionAdvanceDegrees;
    point.atmosphericCorrectionFactor = std::clamp(
        (99.0 / config.ambientPressureKpa)
            * std::sqrt((config.ambientTemperatureC + 273.15) / 298.15),
        0.80, 1.20);
    point.correctedTorqueNm = point.torqueNm
        * point.atmosphericCorrectionFactor;
    point.correctedPowerKw = point.powerKw
        * point.atmosphericCorrectionFactor;
    point.targetAirFuelRatio = estimate.meanTelemetry.targetAirFuelRatio;
    point.volumetricEfficiency = estimate.meanTelemetry.volumetricEfficiency;
    point.fuelFlowGramsPerSecond = estimate.meanTelemetry.fuelFlowGramsPerSecond;
    point.manifoldPressureKpa = estimate.meanTelemetry.manifoldPressureKpa;
    point.exhaustPressureKpa = estimate.meanTelemetry.exhaustPressureKpa;
    point.oilTemperatureC = estimate.meanTelemetry.oilTemperatureC;
    point.oilPressureKpa = estimate.meanTelemetry.oilPressureKpa;
    point.airFlowGramsPerSecond = estimate.meanTelemetry.airFlowGramsPerSecond;
    point.lambda = estimate.meanTelemetry.lambda;
    point.brakeSpecificFuelConsumptionGPerKwh =
        estimate.meanTelemetry.brakeSpecificFuelConsumptionGPerKwh;
    point.binRpm = fixedBin ? coordinateRpm : 0.0;
    point.windowMeanRpm = estimate.meanRpm;
    point.windowMinimumRpm = estimate.minRpm;
    point.windowMaximumRpm = estimate.maxRpm;
    point.windowDurationSeconds = estimate.durationSeconds;
    point.torqueVarianceNm2 = estimate.torqueVarianceNm2;
    point.firstCycleId = estimate.firstCycleId;
    point.lastCycleId = estimate.lastCycleId;
    point.acceptedCycleCount = static_cast<std::uint32_t>(
        std::min<std::size_t>(estimate.cycleCount,
            std::numeric_limits<std::uint32_t>::max()));
    point.qualityReasons = 0U;
    point.valid = estimate.quality == DynoEstimateQuality::ready
        && estimate.continuous && !estimate.capacityLimited;
    return point;
}

[[nodiscard]] DynoPoint makeInvalidDynoPoint(
    double binRpm, DynoQualityReason reasons) noexcept {
    DynoPoint point;
    point.rpm = binRpm;
    point.binRpm = binRpm;
    point.qualityReasons = static_cast<std::uint32_t>(
        reasons == DynoQualityReason::none
            ? DynoQualityReason::discontinuousCycle : reasons);
    point.valid = false;
    return point;
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
                             EngineSimulatorOptions simulatorOptions,
                             std::shared_ptr<DynoRunArchive> dynoArchive)
    : config_(normalised(std::move(config))), ecu_(std::move(calibrations)),
      exhaust_(ExhaustGraph::makeForEngine(config_)),
      simulator_(config_, ecu_, physics_, eventGenerator_, exhaust_,
                 std::move(simulatorOptions)), dynoAbsorber_(config_),
      driveline_(config_),
      pressureQueue_(std::make_unique<CylinderPressureQueue>()),
      exhaustAcousticQueue_(std::make_unique<ExhaustAcousticQueue>()),
      dynoArchive_(dynoArchive ? std::move(dynoArchive)
                               : std::make_shared<DynoRunArchive>()) {
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
    if (dynoActive_)
        finishDynoSession(
            DynoRunStatus::cancelled,
            DynoStopReason::runtimeStopped);
    else if (dynoSessionActive_.exchange(false, std::memory_order_acq_rel)) {
        {
            const std::scoped_lock lock(dynoMutex_);
            pendingDynoCalibration_.reset();
        }
        dynoRunStatus_.store(
            DynoRunStatus::cancelled, std::memory_order_release);
    }
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

void EngineRuntime::setDynoMode(DynoMode value) noexcept {
    if (dynoSessionActive_.load(std::memory_order_acquire)) return;
    configuredDynoMode_.store(value, std::memory_order_relaxed);
}

void EngineRuntime::setDynoHoldEnabled(bool value) noexcept {
    if (value) setDynoMode(DynoMode::hold);
    else if (dynoMode() == DynoMode::hold)
        setDynoMode(DynoMode::steppedCalibration);
}

void EngineRuntime::setDynoRampEnabled(bool value) noexcept {
    if (value) setDynoMode(DynoMode::continuousRamp);
    else if (dynoMode() == DynoMode::continuousRamp)
        setDynoMode(DynoMode::steppedCalibration);
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
    DynoSessionConfig config;
    config.mode = dynoMode();
    config.holdRpm = dynoHoldRpm_.load(std::memory_order_relaxed);
    config.rampRateRpmPerSecond =
        dynoRampRpmPerSecond_.load(std::memory_order_relaxed);
    config.maximumDurationSeconds =
        dynoMaximumDurationSeconds_.load(std::memory_order_relaxed);
    startDyno(config);
}

void EngineRuntime::startDyno(DynoSessionConfig config) {
    auto expected = false;
    if (!dynoSessionActive_.compare_exchange_strong(
            expected, true, std::memory_order_acq_rel))
        return;
    const auto defaultEntryRpm = sweepEntryRpm(config_);
    const auto safeCeilingRpm = sweepCeilingRpm(config_);
    config.sweepEntryRpm = std::isfinite(config.sweepEntryRpm)
            && config.sweepEntryRpm > 0.0
        ? std::clamp(config.sweepEntryRpm, 500.0, safeCeilingRpm)
        : defaultEntryRpm;
    config.sweepCeilingRpm = std::isfinite(config.sweepCeilingRpm)
            && config.sweepCeilingRpm > 0.0
        ? std::clamp(config.sweepCeilingRpm,
            config.sweepEntryRpm, safeCeilingRpm)
        : safeCeilingRpm;
    config.holdRpm = std::isfinite(config.holdRpm)
        ? std::clamp(config.holdRpm,
            std::max(500.0, config_.idleRpm * 0.6), safeCeilingRpm)
        : dynoHoldRpm_.load(std::memory_order_relaxed);
    config.rampRateRpmPerSecond = std::isfinite(
            config.rampRateRpmPerSecond)
        ? std::clamp(config.rampRateRpmPerSecond, 50.0, 2'000.0)
        : 500.0;
    config.binWidthRpm = std::isfinite(config.binWidthRpm)
        ? std::clamp(config.binWidthRpm, 10.0, 250.0) : 50.0;
    config.rollingWindowSeconds = std::isfinite(
            config.rollingWindowSeconds)
        ? std::clamp(config.rollingWindowSeconds, 0.10, 1.0) : 0.25;
    config.maximumDurationSeconds = std::isfinite(
            config.maximumDurationSeconds)
        ? std::clamp(config.maximumDurationSeconds, 30.0, 300.0) : 60.0;
    {
        const std::scoped_lock lock(dynoMutex_);
        pendingDynoConfig_ = config;
        // Acceptance, not the next 240 Hz tick, defines the metrology
        // boundary. Keep this immutable object alive even if the tuner
        // publishes another revision before beginDynoSession() runs.
        pendingDynoCalibration_ = ecu_.calibrationStore()->snapshot();
    }
    dynoRunStatus_.store(DynoRunStatus::running,
                         std::memory_order_release);
    dynoRequestedRunning_.store(true, std::memory_order_release);
}

void EngineRuntime::stopDyno() {
    dynoRequestedRunning_.store(false, std::memory_order_release);
}

void EngineRuntime::beginDynoSession() {
    if (dynoActive_) return;
    {
        const std::scoped_lock lock(dynoMutex_);
        activeDynoConfig_ = pendingDynoConfig_;
        const auto calibrationRevision = ecu_.pinCalibrationSnapshot(
            std::move(pendingDynoCalibration_));
        currentRun_ = {};
        currentRun_.id = dynoArchive_->reserveRunId();
        currentRun_.engineName = config_.name;
        currentRun_.sessionConfig = activeDynoConfig_;
        currentRun_.status = DynoRunStatus::running;
        currentRun_.calibrationRevision = calibrationRevision;
        currentRun_.startedAtSimulationSeconds =
            simulator_.state().simulationTimeSeconds;
        const auto expectedPointCount = static_cast<std::size_t>(std::ceil(
            std::max(0.0, activeDynoConfig_.sweepCeilingRpm
                - activeDynoConfig_.sweepEntryRpm)
            / std::max(1.0, activeDynoConfig_.binWidthRpm))) + 2U;
        currentRun_.points.reserve(expectedPointCount);
    }
    dynoElapsed_ = 0.0;
    dynoStartupElapsed_ = 0.0;
    nextSampleRpm_ = activeDynoConfig_.sweepEntryRpm;
    dynoTargetRpm_ = nextSampleRpm_;
    dynoPreparationDestinationRpm_ =
        activeDynoConfig_.mode == DynoMode::hold
        ? activeDynoConfig_.holdRpm
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
    dynoBrakeTorqueNm_ = 0.0;
    dynoAbsorber_.reset(
        simulator_.state().rpm, simulator_.state().torqueNm);
    dynoAbsorberOutput_ = {};
    dynoEstimator_ = DynoEstimator(
        activeDynoConfig_.rollingWindowSeconds);
    previousRampEstimate_.reset();
    lastCompletedBrakeCycleTorqueNm_ = 0.0;
    latestDynoQualityReasons_ = DynoQualityReason::notPrepared;
    pendingDynoInvalidReasons_ = DynoQualityReason::none;
    hasCompletedBrakeCycleTorque_ = false;
    dynoCycleTainted_ = true;
    dynoGateAllowsProgress_ = false;
    dynoRampPrimed_ = false;
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
    dynoRunStatus_.store(DynoRunStatus::running,
                         std::memory_order_release);
}

void EngineRuntime::finishDynoSession(
    DynoRunStatus status, DynoStopReason reason) {
    if (!dynoActive_) return;
    {
        const std::scoped_lock lock(dynoMutex_);
        currentRun_.status = status;
        currentRun_.stopReason = reason;
        currentRun_.endedAtSimulationSeconds =
            simulator_.state().simulationTimeSeconds;
        (void)dynoArchive_->append(currentRun_);
        currentRun_ = {};
    }
    ignition_.store(savedIgnition_);
    starter_.store(savedStarter_);
    throttle_.store(savedThrottle_);
    load_.store(savedLoad_);
    ecu_.releasePinnedCalibrationSnapshot();
    dynoActive_ = false;
    dynoRequestedRunning_.store(false, std::memory_order_release);
    dynoRunStatus_.store(status, std::memory_order_release);
    dynoSessionActive_.store(false, std::memory_order_release);
}

DynoRun EngineRuntime::currentDynoRun() const {
    const std::scoped_lock lock(dynoMutex_);
    return currentRun_;
}

std::vector<DynoRun> EngineRuntime::dynoHistory() const {
    return dynoArchive_->snapshot();
}

void EngineRuntime::deleteDynoRun(std::uint64_t id) {
    (void)dynoArchive_->erase(id);
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
        else if (!dynoDesired && dynoActive_) {
            auto hasValidHoldPoint = false;
            if (activeDynoConfig_.mode == DynoMode::hold) {
                const std::scoped_lock lock(dynoMutex_);
                hasValidHoldPoint = std::ranges::any_of(
                    currentRun_.points,
                    [](const DynoPoint& point) { return point.valid; });
            }
            finishDynoSession(
                hasValidHoldPoint ? DynoRunStatus::completed
                                  : DynoRunStatus::cancelled,
                hasValidHoldPoint ? DynoStopReason::operatorFinished
                                  : DynoStopReason::operatorCancelled);
        } else if (!dynoDesired
                   && dynoSessionActive_.load(std::memory_order_acquire)) {
            // startDyno()/stopDyno() may both be called between two simulation
            // ticks. No session ever began, so there is no fabricated empty run
            // to archive, but the public state must still leave `running`.
            dynoRunStatus_.store(
                DynoRunStatus::cancelled, std::memory_order_release);
            {
                const std::scoped_lock lock(dynoMutex_);
                pendingDynoCalibration_.reset();
            }
            dynoSessionActive_.store(false, std::memory_order_release);
        }
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
                if (activeDynoConfig_.mode == DynoMode::hold)
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
                        dynoBrakeTorqueNm_ = 0.0;
                        dynoAbsorber_.reset(
                            dynoState.rpm,
                            dynoState.cycleAveragedTorqueNm);
                        dynoAbsorberOutput_ = {};
                        dynoEstimator_.reset();
                        previousRampEstimate_.reset();
                        hasCompletedBrakeCycleTorque_ = false;
                        dynoCycleTainted_ = true;
                        dynoGateAllowsProgress_ = false;
                        dynoRampPrimed_ = false;
                        pendingDynoInvalidReasons_ =
                            DynoQualityReason::none;
                        starter_.store(false);
                        dynoThrottleCommand_ = 1.0;
                    }
                }
                requestedLoad = 0.0;
            }
            if (dynoSweeping_) {
                dynoElapsed_ += baseStep.count();
                dynoElapsed = dynoElapsed_;
                if (activeDynoConfig_.mode == DynoMode::hold) {
                    // Wheel changes in hold mode are rate limited too; a large
                    // setpoint change cannot become another brake step. A
                    // moving setpoint also starts a fresh measurement window:
                    // torque from the old hold must never be relabelled at the
                    // new target.
                    constexpr double holdSlewRpmPerSecond = 700.0;
                    const auto previousTargetRpm = dynoTargetRpm_;
                    dynoTargetRpm_ = moveTowards(
                        dynoTargetRpm_,
                        dynoHoldRpm_.load(std::memory_order_relaxed),
                        holdSlewRpmPerSecond * baseStep.count());
                    if (std::abs(dynoTargetRpm_ - previousTargetRpm) > 1.0e-9) {
                        dynoEstimator_.breakContinuity();
                        previousRampEstimate_.reset();
                        dynoCycleTainted_ = true;
                        dynoGateAllowsProgress_ = false;
                    }
                } else if (activeDynoConfig_.mode
                           == DynoMode::continuousRamp) {
                    // The target is frozen exactly until one complete rolling
                    // window has primed the pull, and on every rejected frame.
                    // The former `target /= 1 + dt` path was not a pause: at
                    // normal speeds it could command a multi-thousand-rpm/s
                    // fall after one weak cycle and invalidate the next bins.
                    if (dynoRampPrimed_ && dynoGateAllowsProgress_) {
                        const auto rampCeilingRpm =
                            activeDynoConfig_.sweepCeilingRpm;
                        dynoTargetRpm_ = std::min(rampCeilingRpm,
                            dynoTargetRpm_
                                + activeDynoConfig_.rampRateRpmPerSecond
                                    * baseStep.count());
                    }
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
                    dynoEstimator_.breakContinuity();
                    previousRampEstimate_.reset();
                    hasCompletedBrakeCycleTorque_ = false;
                    dynoCycleTainted_ = true;
                    dynoGateAllowsProgress_ = false;
                    pendingDynoInvalidReasons_ |=
                        DynoQualityReason::recoveryActive;
                    dynoThrottleCommand_ = 0.18;
                    starter_.store(true);
                } else {
                    dynoThrottleCommand_ = 1.0;
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
            // Observe the POST-step shaft state. The old acquisition gate used
            // the pre-step absorber output with post-step engine telemetry;
            // moving this controller update here keeps contact, acceleration,
            // limiter state and completed-cycle events on one time boundary.
            dynoAbsorberOutput_ = dynoAbsorber_.advance(
                baseStep.count(), dynoTargetRpm_, frame.state);
            dynoBrakeTorqueNm_ = dynoAbsorberOutput_.brakeTorqueNm;

            const auto acquisitionMode = activeDynoConfig_.mode;
            const auto holding = acquisitionMode == DynoMode::hold;
            const auto rampingSweep =
                acquisitionMode == DynoMode::continuousRamp;
            const auto rampRate =
                activeDynoConfig_.rampRateRpmPerSecond;
            const auto gateInput = [&](double cycleTorqueNm,
                                       double measuredRpm,
                                       bool cycleContinuous) noexcept {
                DynoQualityGateInput input;
                input.mode = acquisitionMode;
                input.targetRpm = dynoTargetRpm_;
                input.measuredRpm = measuredRpm;
                input.rampRateRpmPerSecond = rampRate;
                input.measuredCycleTorqueNm = cycleTorqueNm;
                input.prepared = true;
                input.protocolReady = true;
                input.recoveryActive = false;
                input.cycleContinuous = cycleContinuous;
                return input;
            };
            const auto storePoint = [&](const DynoPoint& point,
                                        bool replaceHoldPoint) {
                const std::scoped_lock lock(dynoMutex_);
                if (replaceHoldPoint && !currentRun_.points.empty())
                    currentRun_.points.front() = point;
                else
                    currentRun_.points.push_back(point);
                if (!point.valid) return;
                if (replaceHoldPoint) {
                    currentRun_.points.resize(1);
                    currentRun_.peakTorqueNm = point.torqueNm;
                    currentRun_.peakPowerKw = point.powerKw;
                    currentRun_.peakCorrectedTorqueNm = point.correctedTorqueNm;
                    currentRun_.peakCorrectedPowerKw = point.correctedPowerKw;
                } else {
                    currentRun_.peakTorqueNm = std::max(
                        currentRun_.peakTorqueNm, point.torqueNm);
                    currentRun_.peakPowerKw = std::max(
                        currentRun_.peakPowerKw, point.powerKw);
                    currentRun_.peakCorrectedTorqueNm = std::max(
                        currentRun_.peakCorrectedTorqueNm,
                        point.correctedTorqueNm);
                    currentRun_.peakCorrectedPowerKw = std::max(
                        currentRun_.peakCorrectedPowerKw,
                        point.correctedPowerKw);
                }
            };

            if (frame.droppedCompletedBrakeCycleSampleCount > 0) {
                droppedBrakeCycleSamples_.fetch_add(
                    frame.droppedCompletedBrakeCycleSampleCount,
                    std::memory_order_relaxed);
                dynoEstimator_.breakContinuity();
                previousRampEstimate_.reset();
                dynoCycleTainted_ = true;
                dynoGateAllowsProgress_ = false;
                latestDynoQualityReasons_ |=
                    DynoQualityReason::discontinuousCycle;
                pendingDynoInvalidReasons_ |=
                    DynoQualityReason::discontinuousCycle;
            }

            // Frame-level faults taint the complete cycle which contains them;
            // they also freeze a ramp immediately instead of waiting for its
            // boundary event. Cycle continuity itself is evaluated at the
            // boundary below, hence `true` in this instantaneous observation.
            const auto liveGate = dynoQualityGate_.evaluate(
                gateInput(hasCompletedBrakeCycleTorque_
                        ? lastCompletedBrakeCycleTorqueNm_ : 0.0,
                    dynoAbsorberOutput_.filteredRpm,
                    true),
                frame.state, dynoAbsorberOutput_);
            latestDynoQualityReasons_ = liveGate.reasons;
            if (!liveGate.accepted()) {
                dynoGateAllowsProgress_ = false;
                const auto physicallyTaintsCycle =
                    hasDynoQualityReason(liveGate.reasons,
                        DynoQualityReason::nonFinite)
                    || hasDynoQualityReason(liveGate.reasons,
                        DynoQualityReason::noBrakeContact)
                    || hasDynoQualityReason(liveGate.reasons,
                        DynoQualityReason::absorberCapacityLimited)
                    || hasDynoQualityReason(liveGate.reasons,
                        DynoQualityReason::revLimiterActive)
                    || hasDynoQualityReason(liveGate.reasons,
                        DynoQualityReason::recoveryActive);
                if (physicallyTaintsCycle) {
                    dynoCycleTainted_ = true;
                    pendingDynoInvalidReasons_ |= liveGate.reasons;
                }
            } else {
                dynoGateAllowsProgress_ = !rampingSweep
                    || (dynoRampPrimed_
                        && dynoEstimator_.estimate().quality
                            == DynoEstimateQuality::ready);
            }

            for (std::size_t sampleIndex = 0;
                 sampleIndex < frame.completedBrakeCycleSampleCount;
                 ++sampleIndex) {
                const auto& cycle =
                    frame.completedBrakeCycleSamples[sampleIndex];
                const auto update = dynoEstimator_.push(
                    cycle, dynoTelemetry(frame.state));
                lastCompletedBrakeCycleTorqueNm_ = cycle.meanTorqueNm;
                hasCompletedBrakeCycleTorque_ = cycle.numericallyValid;
                const auto cycleSequenceContinuous =
                    update.acceptance == DynoCycleAcceptance::accepted
                    && !dynoCycleTainted_;
                const auto cycleGate = dynoQualityGate_.evaluate(
                    gateInput(cycle.meanTorqueNm,
                              update.estimate.meanRpm > 0.0
                                ? update.estimate.meanRpm
                                : cycle.meanRpm,
                              cycleSequenceContinuous),
                    frame.state, dynoAbsorberOutput_);
                latestDynoQualityReasons_ = cycleGate.reasons;
                dynoCycleTainted_ = false;
                if (update.acceptance != DynoCycleAcceptance::accepted
                    || !cycleGate.accepted()) {
                    pendingDynoInvalidReasons_ |= cycleGate.reasons;
                    dynoEstimator_.breakContinuity();
                    previousRampEstimate_.reset();
                    dynoGateAllowsProgress_ = false;
                    continue;
                }

                const auto estimate = update.estimate;
                if (estimate.quality != DynoEstimateQuality::ready
                    || estimate.capacityLimited) {
                    dynoGateAllowsProgress_ = !rampingSweep;
                    continue;
                }
                dynoGateAllowsProgress_ = true;
                const auto ceilingRpm =
                    activeDynoConfig_.sweepCeilingRpm;

                if (rampingSweep) {
                    const auto binWidthRpm =
                        activeDynoConfig_.binWidthRpm;
                    if (!dynoRampPrimed_) {
                        const auto entryRpm =
                            activeDynoConfig_.sweepEntryRpm;
                        // Priming is a steady hold, even though the following
                        // acquisition is transient. Do not label a window up
                        // to 150 rpm away as the entry point.
                        if (std::abs(estimate.meanRpm - entryRpm)
                            > 0.5 * binWidthRpm)
                            continue;
                        storePoint(makeDynoPoint(
                            config_, estimate, entryRpm, true), false);
                        dynoRampPrimed_ = true;
                        nextSampleRpm_ = entryRpm + binWidthRpm;
                        previousRampEstimate_ = estimate;
                        pendingDynoInvalidReasons_ =
                            DynoQualityReason::none;
                        continue;
                    }
                    if (!previousRampEstimate_) {
                        while (nextSampleRpm_ <= ceilingRpm + 1.0e-9
                               && nextSampleRpm_
                                    < estimate.meanRpm - 1.0e-9) {
                            storePoint(makeInvalidDynoPoint(
                                nextSampleRpm_,
                                pendingDynoInvalidReasons_), false);
                            nextSampleRpm_ += binWidthRpm;
                        }
                        previousRampEstimate_ = estimate;
                        continue;
                    }
                    const auto previous = *previousRampEstimate_;
                    const auto rpmDelta = estimate.meanRpm - previous.meanRpm;
                    if (rpmDelta > 1.0e-9) {
                        while (nextSampleRpm_ <= ceilingRpm + 1.0e-9
                               && nextSampleRpm_
                                    <= estimate.meanRpm + 1.0e-9) {
                            if (nextSampleRpm_
                                < previous.meanRpm - 1.0e-9) {
                                storePoint(makeInvalidDynoPoint(
                                    nextSampleRpm_,
                                    pendingDynoInvalidReasons_), false);
                            } else {
                                const auto fraction =
                                    (nextSampleRpm_ - previous.meanRpm)
                                    / rpmDelta;
                                const auto atBin = interpolateEstimate(
                                    previous, estimate, fraction,
                                    nextSampleRpm_);
                                storePoint(makeDynoPoint(
                                    config_, atBin,
                                    nextSampleRpm_, true), false);
                            }
                            nextSampleRpm_ += binWidthRpm;
                        }
                        previousRampEstimate_ = estimate;
                        pendingDynoInvalidReasons_ =
                            DynoQualityReason::none;
                    }
                    if (dynoTargetRpm_ >= ceilingRpm - 1.0e-9
                        && nextSampleRpm_ > ceilingRpm + 1.0e-9)
                        dynoCompleted_.store(
                            true, std::memory_order_relaxed);
                    continue;
                }

                const auto point = makeDynoPoint(
                    config_, estimate, estimate.meanRpm, false);
                if (holding) {
                    // Hold is one live aggregate, not an ever-growing curve at
                    // a repeated abscissa. The UI may later render its temporal
                    // stability separately without inventing torque/RPM data.
                    storePoint(point, true);
                    continue;
                }

                storePoint(point, false);
                if (dynoTargetRpm_ >= ceilingRpm - 1.0e-6) {
                    dynoCompleted_.store(true, std::memory_order_relaxed);
                } else {
                    dynoTargetRpm_ = std::min(
                        ceilingRpm, dynoTargetRpm_ + 250.0);
                    nextSampleRpm_ = dynoTargetRpm_;
                    dynoEstimator_.reset();
                    previousRampEstimate_.reset();
                    dynoCycleTainted_ = true;
                    dynoGateAllowsProgress_ = false;
                }
                // A high-speed diagnostic step can theoretically contain more
                // than one cycle; every remaining sample belongs to the old
                // setpoint and must not leak into the new stepped window.
                break;
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
            const auto displayedDynoMode = dynoActive_
                ? activeDynoConfig_.mode : dynoMode();
            frame.state.dynoMode = displayedDynoMode;
            frame.state.dynoRunStatus = dynoRunStatus_.load(
                std::memory_order_acquire);
            frame.state.dynoPhase = !dynoActive_
                ? (frame.state.dynoRunStatus == DynoRunStatus::idle
                    ? DynoPhase::idle : DynoPhase::terminal)
                : (!dynoSweeping_.load(std::memory_order_relaxed)
                    ? (dynoRecoveryCount_ > 0
                        ? DynoPhase::recovery : DynoPhase::preparing)
                    : DynoPhase::acquiring);
            frame.state.dynoHoldRpm = dynoHoldRpm_.load(
                std::memory_order_relaxed);
            frame.state.dynoHoldEnabled =
                displayedDynoMode == DynoMode::hold;
            frame.state.dynoRampEnabled =
                displayedDynoMode == DynoMode::continuousRamp;
            frame.state.dynoRampRpmPerSecond = dynoActive_
                ? activeDynoConfig_.rampRateRpmPerSecond
                : dynoRampRpmPerSecond_.load(std::memory_order_relaxed);
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
            frame.state.dynoBrakeContactFraction =
                dynoAbsorberOutput_.contactFraction;
            frame.state.dynoAbsorberSaturatedLow =
                dynoAbsorberOutput_.saturatedLow;
            frame.state.dynoAbsorberSaturatedHigh =
                dynoAbsorberOutput_.saturatedHigh;
            frame.state.dynoQualityReasons =
                static_cast<std::uint32_t>(latestDynoQualityReasons_);
            const auto& dynoEstimate = dynoEstimator_.estimate();
            frame.state.dynoMeasurementReady =
                dynoEstimate.quality == DynoEstimateQuality::ready
                && dynoEstimate.continuous
                && !dynoEstimate.capacityLimited;
            frame.state.dynoAcceptedCycleCount =
                static_cast<std::uint32_t>(std::min<std::size_t>(
                    dynoEstimate.cycleCount,
                    std::numeric_limits<std::uint32_t>::max()));
            frame.state.dynoWindowDurationSeconds =
                dynoEstimate.durationSeconds;
            frame.state.dynoWindowMeanRpm = dynoEstimate.meanRpm;
            frame.state.dynoWindowMinimumRpm = dynoEstimate.minRpm;
            frame.state.dynoWindowMaximumRpm = dynoEstimate.maxRpm;
            const auto sweepStartRpm = dynoActive_
                ? activeDynoConfig_.sweepEntryRpm : sweepEntryRpm(config_);
            const auto sweepCeiling = dynoActive_
                ? activeDynoConfig_.sweepCeilingRpm : sweepCeilingRpm(config_);
            const auto sweepRangeRpm = std::max(
                1.0, sweepCeiling - sweepStartRpm);
            frame.state.dynoProgress = displayedDynoMode == DynoMode::hold
                ? (frame.state.dynoPreparing ? 0.0 : 1.0)
                : std::clamp((dynoTargetRpm_ - sweepStartRpm)
                    / sweepRangeRpm, 0.0, 1.0);
            frame.state.dynoRecoveryCount = dynoRecoveryCount_;
            const std::scoped_lock lock(snapshotMutex_);
            snapshot_ = frame.state;
        }
        const auto maximumDynoDurationSeconds = dynoActive_
            ? activeDynoConfig_.maximumDurationSeconds
            : dynoMaximumDurationSeconds_.load(std::memory_order_relaxed);
        const auto dynoStartupTimedOut = !dynoSweeping_.load(std::memory_order_relaxed)
            && dynoStartupElapsed >= maximumDynoDurationSeconds;
        if (dynoActive_ && dynoCompleted_.load(std::memory_order_relaxed)) {
            dynoRequestedRunning_.store(false, std::memory_order_release);
            finishDynoSession(
                DynoRunStatus::completed,
                DynoStopReason::sweepCeilingReached);
        } else if (dynoActive_ && dynoStartupTimedOut) {
            dynoRequestedRunning_.store(false, std::memory_order_release);
            finishDynoSession(
                DynoRunStatus::timedOut,
                DynoStopReason::startupTimeout);
        } else if (dynoActive_
                   && dynoElapsed >= maximumDynoDurationSeconds) {
            dynoRequestedRunning_.store(false, std::memory_order_release);
            finishDynoSession(
                DynoRunStatus::timedOut,
                DynoStopReason::acquisitionTimeout);
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
