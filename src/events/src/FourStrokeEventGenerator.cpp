#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <algorithm>
#include <cmath>
#include <numbers>

namespace enginelab {
std::size_t FourStrokeEventGenerator::generate(
    const EngineConfig& config, const EngineState& state, const EcuCommand& ecu,
    const CombustionResult& combustion, double stepStartTime, double previousAngle,
    double travelledDegrees, double dtSeconds, std::span<FiringEvent> output) noexcept {
    lastDroppedEventCount_ = 0;
    if (!ecu.fuelEnabled
        || (config.fuel == FuelType::gasoline && !ecu.sparkEnabled)
        || config.firingOrder.empty()
        || travelledDegrees <= 0.0)
        return 0;

    std::size_t written = 0;
    const auto cycle = cycleDegrees();
    for (std::size_t orderIndex = 0; orderIndex < config.firingOrder.size(); ++orderIndex) {
        const auto cylinderId = config.firingOrder[orderIndex];
        const auto cylinder = std::find_if(config.cylinders.begin(), config.cylinders.end(),
            [cylinderId](const CylinderConfig& item) { return item.id == cylinderId; });
        if (cylinder == config.cylinders.end()) continue;
        const auto cylinderIndex = static_cast<std::size_t>(std::distance(config.cylinders.begin(), cylinder));
        float stereoPosition = 0.0F;
        const auto bank = std::find_if(config.banks.begin(), config.banks.end(), [&cylinder](const auto& item) {
            return item.id == cylinder->bankId
                || std::find(item.cylinderIds.begin(), item.cylinderIds.end(), cylinder->id) != item.cylinderIds.end();
        });
        if (config.layout == EngineLayout::radial)
            stereoPosition = static_cast<float>(std::sin(cylinder->bankOffsetDegrees
                * std::numbers::pi / 180.0) * 0.74);
        else if (bank != config.banks.end() && std::abs(bank->angleDegrees) > 0.1)
            stereoPosition = static_cast<float>(std::sin(bank->angleDegrees
                * std::numbers::pi / 180.0) * 0.78);
        else if (config.cylinders.size() > 1)
            stereoPosition = static_cast<float>(-0.72 + 1.44 * static_cast<double>(cylinderIndex)
                / static_cast<double>(config.cylinders.size() - 1));
        const auto cylinderState = std::find_if(state.cylinderStates.begin(),
            state.cylinderStates.begin()
                + static_cast<std::ptrdiff_t>(state.cylinderStateCount),
            [cylinderId](const CylinderState& item) {
                return item.id == cylinderId;
            });
        const auto hasCylinderState = cylinderState
            != state.cylinderStates.begin()
                + static_cast<std::ptrdiff_t>(state.cylinderStateCount);
        if (config.fuel == FuelType::diesel
                && (!hasCylinderState
                    || !cylinderState->compressionIgnition))
            continue;
        const auto combustionPhase = config.fuel == FuelType::diesel
            ? cylinderState->combustionStartPhaseDegrees
            : cylinder->ignitionOffsetDegrees
                - ecu.ignitionAdvanceDegrees;
        const auto target = std::fmod(cylinder->crankOffsetDegrees
            + combustionPhase + cycle * 2.0, cycle);
        auto distance = std::fmod(target - previousAngle + cycle, cycle);
        if (distance < 1.0e-9) distance = cycle;
        while (distance <= travelledDegrees + 1.0e-9) {
            const auto misfire = cylinderState != state.cylinderStates.begin()
                    + static_cast<std::ptrdiff_t>(state.cylinderStateCount)
                ? cylinderState->misfiring
                : randomUnit() < static_cast<float>(combustion.misfireProbability);
            const auto variation = 0.97F + randomUnit() * 0.06F + static_cast<float>(cylinder->efficiencyOffset);
            const auto eventTime = stepStartTime + dtSeconds * std::clamp(distance / travelledDegrees, 0.0, 1.0);
            const auto fuelDelivery = hasCylinderState
                ? static_cast<float>(std::clamp(cylinderState->fuelDeliveryRatio, 0.0, 1.0)) : 1.0F;
            const auto resolvedPulse = hasCylinderState
                ? static_cast<float>(std::clamp(cylinderState->combustionPulse / 1.4, 0.0, 1.35)) : 1.0F;
            const auto resolvedPressureBar = hasCylinderState && cylinderState->pressureEstimateBar > 1.0
                ? static_cast<float>(cylinderState->pressureEstimateBar)
                : static_cast<float>(combustion.pressureEstimateBar);
            const auto resolvedCombustionDurationMs =
                hasCylinderState && cylinderState->compressionIgnition
                ? static_cast<float>(std::clamp(
                    cylinderState->combustionDurationMs, 0.2, 45.0))
                : hasCylinderState && cylinderState->flameSpeedMps > 0.01
                    ? static_cast<float>(std::clamp(cylinder->boreMm * 0.5
                        / cylinderState->flameSpeedMps, 1.0, 45.0))
                    : static_cast<float>(1.4 + 10.0
                        / std::max(1.0, state.rpm / 1'000.0));
            const auto bankCam = bank != config.banks.end() ? &bank->camshafts : &config.camshafts;
            const auto highProfile = bankCam->variableProfileEnabled
                && state.rpm >= bankCam->switchRpm && state.throttle >= bankCam->switchThrottle;
            const auto exhaustDuration = highProfile
                ? bankCam->highExhaustDurationDegrees : bankCam->exhaustDurationDegrees;
            const auto exhaustOpenCylinderPhase = std::fmod(360.0 - bankCam->exhaustCenterlineDegrees
                - exhaustDuration * 0.5 + cycle, cycle);
            const auto exhaustOpenGlobalAngle = std::fmod(cylinder->crankOffsetDegrees
                + exhaustOpenCylinderPhase + cycle, cycle);
            const auto crankDegreesToExhaustOpen = std::fmod(exhaustOpenGlobalAngle - target + cycle, cycle);
            const auto valveEventDelaySeconds = state.rpm > 20.0
                ? static_cast<float>(crankDegreesToExhaustOpen / (state.rpm * 6.0)) : 0.0F;
            // Driver pedal position is not cylinder charge: at closed-throttle
            // idle the separate bypass supplies a real combustible load while
            // state.throttle is exactly zero.  Using thermodynamic load keeps
            // the transient accent proportional to the charge that fired.
            // Misfires retain that physical reference intensity and are
            // attenuated once by the renderer's misfire path; the former fixed
            // 0.04 value could make an idle misfire louder than a valid firing.
            const auto thermodynamicLoad = static_cast<float>(
                std::clamp(state.load, 0.0, 1.35));
            const auto eventIntensity = std::clamp(thermodynamicLoad
                * static_cast<float>(combustion.combustionQuality)
                * variation * fuelDelivery * (0.65F + resolvedPulse * 0.35F),
                0.001F, 1.0F);
            FiringEvent event { eventTime, cylinderId, target,
                eventIntensity,
                resolvedPressureBar,
                resolvedCombustionDurationMs,
                static_cast<float>(state.airFuelRatio), static_cast<float>(ecu.ignitionAdvanceDegrees),
                static_cast<float>(hasCylinderState ? cylinderState->endGasKnockLevel : state.knockLevel), stereoPosition,
                cylinderState != state.cylinderStates.begin() + static_cast<std::ptrdiff_t>(state.cylinderStateCount)
                    ? static_cast<float>(cylinderState->exhaustFlowMgPerCycle) : 0.0F,
                cylinderState != state.cylinderStates.begin() + static_cast<std::ptrdiff_t>(state.cylinderStateCount)
                    ? static_cast<float>(cylinderState->runnerPressureKpa) : static_cast<float>(state.exhaustRunnerPressureKpa),
                valveEventDelaySeconds, 0.0F,
                misfire, cylinderId, cylinderId };
            event.compressionIgnition = hasCylinderState
                && cylinderState->compressionIgnition;
            event.combustionSharpness = hasCylinderState
                ? static_cast<float>(std::clamp(
                    cylinderState->combustionSharpness, 0.0, 1.0))
                : 0.0F;
            if (written < output.size()) {
                auto insertion = written;
                while (insertion > 0 && output[insertion - 1].timeSeconds > event.timeSeconds) {
                    output[insertion] = output[insertion - 1];
                    --insertion;
                }
                output[insertion] = event;
                ++written;
            } else {
                ++lastDroppedEventCount_;
                if (!output.empty() && event.timeSeconds < output[written - 1].timeSeconds) {
                    auto insertion = written - 1;
                    while (insertion > 0 && output[insertion - 1].timeSeconds > event.timeSeconds) {
                        output[insertion] = output[insertion - 1];
                        --insertion;
                    }
                    output[insertion] = event;
                }
            }
            distance += cycle;
        }
    }
    return written;
}

float FourStrokeEventGenerator::randomUnit() noexcept {
    randomState_ = 1'664'525U * randomState_ + 1'013'904'223U;
    return static_cast<float>(randomState_) / static_cast<float>(0xffffffffU);
}
} // namespace enginelab
