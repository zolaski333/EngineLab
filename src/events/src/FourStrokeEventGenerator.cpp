#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <algorithm>
#include <cmath>

namespace enginelab {
std::size_t FourStrokeEventGenerator::generate(
    const EngineConfig& config, const EngineState& state, const EcuCommand& ecu,
    const CombustionResult& combustion, double stepStartTime, double previousAngle,
    double travelledDegrees, double dtSeconds, std::span<FiringEvent> output) noexcept {
    lastDroppedEventCount_ = 0;
    if (!ecu.fuelEnabled || !ecu.sparkEnabled || output.empty() || config.firingOrder.empty()
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
        if (config.layout == EngineLayout::vLayout)
            stereoPosition = (cylinderIndex % 2U == 0U ? -0.52F : 0.52F);
        else if (config.layout == EngineLayout::flat) {
            const auto bankColumns = std::max(1.0, static_cast<double>((config.cylinders.size() + 1U) / 2U));
            stereoPosition = (cylinderIndex % 2U == 0U ? -0.64F : 0.64F)
                + static_cast<float>((static_cast<double>(cylinderIndex / 2U) / std::max(1.0, bankColumns - 1.0)) - 0.5) * 0.18F;
        }
        else if (config.layout == EngineLayout::radial)
            stereoPosition = static_cast<float>(std::sin(static_cast<double>(cylinderIndex)
                / static_cast<double>(config.cylinders.size()) * 6.283185307179586) * 0.74);
        else if (config.cylinders.size() > 1)
            stereoPosition = static_cast<float>(-0.72 + 1.44 * static_cast<double>(cylinderIndex)
                / static_cast<double>(config.cylinders.size() - 1));
        const auto target = std::fmod(cylinder->crankOffsetDegrees
            + cylinder->ignitionOffsetDegrees - ecu.ignitionAdvanceDegrees + cycle * 2.0, cycle);
        auto distance = std::fmod(target - previousAngle + cycle, cycle);
        if (distance < 1.0e-9) distance = cycle;
        while (distance <= travelledDegrees + 1.0e-9) {
            const auto cylinderState = std::find_if(state.cylinderStates.begin(),
                state.cylinderStates.begin() + static_cast<std::ptrdiff_t>(state.cylinderStateCount),
                [cylinderId](const CylinderState& item) { return item.id == cylinderId; });
            const auto misfire = cylinderState != state.cylinderStates.begin()
                    + static_cast<std::ptrdiff_t>(state.cylinderStateCount)
                ? cylinderState->misfiring
                : randomUnit() < static_cast<float>(combustion.misfireProbability);
            const auto variation = 0.97F + randomUnit() * 0.06F + static_cast<float>(cylinder->efficiencyOffset);
            const auto eventTime = stepStartTime + dtSeconds * std::clamp(distance / travelledDegrees, 0.0, 1.0);
            const auto hasCylinderState = cylinderState != state.cylinderStates.begin()
                + static_cast<std::ptrdiff_t>(state.cylinderStateCount);
            const auto fuelDelivery = hasCylinderState
                ? static_cast<float>(std::clamp(cylinderState->fuelDeliveryRatio, 0.0, 1.0)) : 1.0F;
            const auto resolvedPulse = hasCylinderState
                ? static_cast<float>(std::clamp(cylinderState->combustionPulse / 1.4, 0.20, 1.35)) : 1.0F;
            const auto resolvedPressureBar = hasCylinderState && cylinderState->pressureEstimateBar > 1.0
                ? static_cast<float>(cylinderState->pressureEstimateBar)
                : static_cast<float>(combustion.pressureEstimateBar);
            const auto resolvedCombustionDurationMs = hasCylinderState
                && cylinderState->flameSpeedMps > 0.01
                ? static_cast<float>(std::clamp(cylinder->boreMm * 0.5
                    / cylinderState->flameSpeedMps, 1.0, 45.0))
                : static_cast<float>(1.4 + 10.0 / std::max(1.0, state.rpm / 1'000.0));
            FiringEvent event { eventTime, cylinderId, target,
                misfire ? 0.04F : std::clamp(static_cast<float>(state.throttle * combustion.combustionQuality)
                    * variation * fuelDelivery * (0.65F + resolvedPulse * 0.35F), 0.001F, 1.0F),
                resolvedPressureBar,
                resolvedCombustionDurationMs,
                static_cast<float>(combustion.actualAirFuelRatio), static_cast<float>(ecu.ignitionAdvanceDegrees),
                static_cast<float>(combustion.knockLevel), stereoPosition,
                cylinderState != state.cylinderStates.begin() + static_cast<std::ptrdiff_t>(state.cylinderStateCount)
                    ? static_cast<float>(cylinderState->exhaustFlowMgPerCycle) : 0.0F,
                cylinderState != state.cylinderStates.begin() + static_cast<std::ptrdiff_t>(state.cylinderStateCount)
                    ? static_cast<float>(cylinderState->runnerPressureKpa) : static_cast<float>(state.exhaustRunnerPressureKpa),
                0.0F, 0.0F,
                misfire, cylinderId, cylinderId };
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
                if (event.timeSeconds < output[written - 1].timeSeconds) {
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
