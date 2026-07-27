#include <enginelab/physics/CompressionIgnitionModel.hpp>

#include <algorithm>
#include <cmath>

namespace enginelab {
namespace {

[[nodiscard]] double equivalenceRatio(const GasCell& chamber,
                                      const FuelConfig& fuel) noexcept {
    const auto oxygen = chamber.mixture().oxygenMoles;
    if (oxygen <= 1.0e-15) return 4.0;
    return std::clamp(chamber.mixture().fuelMoles
        * fuel.oxygenMolesPerFuelMole / oxygen, 0.0, 4.0);
}

[[nodiscard]] double normalisedCyclePhase(double phaseDegrees) noexcept {
    const auto wrapped = std::fmod(phaseDegrees + 720.0, 720.0);
    return wrapped > 360.0 ? wrapped - 720.0 : wrapped;
}

} // namespace

void CompressionIgnitionModel::beginCycle(
    CompressionIgnitionState& state) noexcept {
    state = {};
}

double CompressionIgnitionModel::ignitionDelaySeconds(
    const CombustionCalibrationConfig& calibration,
    double cetaneNumber, double pressureBar, double temperatureK,
    double equivalenceRatioValue) noexcept {
    if (!std::isfinite(cetaneNumber) || !std::isfinite(pressureBar)
            || !std::isfinite(temperatureK)
            || !std::isfinite(equivalenceRatioValue)
            || cetaneNumber <= 0.0 || pressureBar <= 0.0
            || temperatureK <= 0.0 || equivalenceRatioValue <= 0.0)
        return 0.2;

    // Assanis et al.: tau[ms] = 2.4 phi^-0.2 p^-1.02 exp(2100/T),
    // with p in bar. Cetane quality is represented by an Arrhenius-like
    // multiplicative correction about CN 50; the bounded result keeps the
    // induction integral well posed outside normal cranking conditions.
    const auto phi = std::clamp(equivalenceRatioValue, 0.02, 3.0);
    const auto pressure = std::clamp(pressureBar, 1.0, 300.0);
    const auto temperature = std::clamp(temperatureK, 300.0, 2'500.0);
    const auto cetaneFactor = std::exp((50.0
        - std::clamp(cetaneNumber, 30.0, 80.0)) / 25.0);
    const auto delayMs = 2.4 * std::pow(phi, -0.2)
        * std::pow(pressure, -1.02) * std::exp(2'100.0 / temperature)
        * cetaneFactor * calibration.compressionIgnitionDelayScale;
    return std::clamp(delayMs * 0.001, 20.0e-6, 0.2);
}

CompressionIgnitionResult CompressionIgnitionModel::advance(
    CompressionIgnitionState& state, GasCell& chamber,
    const FuelConfig& fuel,
    const CombustionCalibrationConfig& calibration,
    const CompressionIgnitionConditions& conditions,
    double dtSeconds) noexcept {
    CompressionIgnitionResult result;
    if (!(dtSeconds > 0.0) || !std::isfinite(dtSeconds)) return result;

    const auto availableFuel = std::max(0.0, chamber.mixture().fuelMoles);
    const auto burnableFuel = std::min(availableFuel,
        chamber.mixture().oxygenMoles
            / std::max(1.0e-12, fuel.oxygenMolesPerFuelMole));
    state.cycleFuelReferenceMoles = std::max(state.cycleFuelReferenceMoles,
        state.cumulativeBurnedFuelMoles + availableFuel);

    const auto phi = equivalenceRatio(chamber, fuel);
    const auto pressureBar = chamber.pressureKpa() * 0.01;
    const auto temperatureK = chamber.temperatureK();
    const auto compressionOrPowerStroke =
        conditions.cyclePhaseDegrees >= 540.0
        || conditions.cyclePhaseDegrees < 180.0;
    const auto inductionEligible = conditions.enabled
        && compressionOrPowerStroke
        && burnableFuel > 1.0e-15
        && chamber.mixture().oxygenMoles > 1.0e-12
        && pressureBar >= 4.0 && temperatureK >= 400.0;

    if (!state.autoIgnited && inductionEligible) {
        state.ignitionDelaySeconds = ignitionDelaySeconds(calibration,
            fuel.cetaneNumber, pressureBar, temperatureK, phi);
        state.livengoodWuIntegral += dtSeconds
            / std::max(20.0e-6, state.ignitionDelaySeconds);
        if (state.livengoodWuIntegral >= 1.0) {
            state.autoIgnited = true;
            state.active = true;
            state.startPhaseDegrees =
                normalisedCyclePhase(conditions.cyclePhaseDegrees);
            state.premixedFuelMolesRemaining = burnableFuel
                * std::clamp(
                    calibration.compressionIgnitionPremixedFraction,
                    0.0, 0.8);
        }
    }

    auto efficiency = 0.0;
    if (state.autoIgnited && conditions.enabled && burnableFuel > 1.0e-15) {
        const auto turbulenceFactor = std::clamp(
            0.65 + std::max(0.0, conditions.meanPistonSpeedMps) * 0.075,
            0.65, 2.0);
        const auto mixingTime = std::max(50.0e-6,
            calibration.compressionIgnitionMixingTimeSeconds
                / turbulenceFactor);
        const auto premixedTime = std::max(30.0e-6, mixingTime * 0.14);

        const auto premixedInventory = std::min(
            state.premixedFuelMolesRemaining, burnableFuel);
        const auto requestedPremixed = premixedInventory
            * (1.0 - std::exp(-dtSeconds / premixedTime));
        const auto mixingInventory = std::max(0.0,
            burnableFuel - premixedInventory);
        const auto requestedMixing = mixingInventory
            * (1.0 - std::exp(-dtSeconds / mixingTime));

        // Lean diesel combustion remains chemically complete over a broad
        // range; only the ultra-lean stability edge and oxygen-starved rich
        // operation reduce completeness.
        const auto leanCompleteness = std::clamp(phi / 0.12, 0.45, 1.0);
        const auto richCompleteness = std::clamp(1.25 / std::max(1.0, phi),
                                                 0.35, 1.0);
        const auto thermalCompleteness = std::clamp(
            (temperatureK - 450.0) / 350.0, 0.72, 1.0);
        efficiency = 0.96 * leanCompleteness * richCompleteness
            * thermalCompleteness;
        const auto lowerHeatingValue =
            fuel.lowerHeatingValueMjPerKg * 1'000'000.0;
        const auto premixedReaction = ConservativeGasSystem::reactFuelMoles(
            chamber, requestedPremixed, efficiency, lowerHeatingValue);
        state.premixedFuelMolesRemaining = std::max(0.0,
            state.premixedFuelMolesRemaining
                - premixedReaction.burnedFuelMoles);
        const auto mixingReaction = ConservativeGasSystem::reactFuelMoles(
            chamber, requestedMixing, efficiency, lowerHeatingValue);

        result.reaction.burnedFuelMoles =
            premixedReaction.burnedFuelMoles
            + mixingReaction.burnedFuelMoles;
        result.reaction.releasedEnergyJoules =
            premixedReaction.releasedEnergyJoules
            + mixingReaction.releasedEnergyJoules;
        result.reaction.completeness = availableFuel > 1.0e-15
            ? result.reaction.burnedFuelMoles / availableFuel : 0.0;
        state.cumulativeBurnedFuelMoles +=
            result.reaction.burnedFuelMoles;
        // Report the useful heat-release duration, not the exponentially
        // decaying numerical tail. With the mixing ODE, a 1e-8 threshold kept
        // adding substeps long after >99 % of the charge had reacted and made
        // an idle diesel appear to burn for ~80 ms. One ten-thousandth of the
        // cycle inventory per substep corresponds to the final ~1 % tail at
        // the delivered substep size and leaves mass/energy conservation
        // untouched; it changes telemetry only.
        const auto significantBurn = result.reaction.burnedFuelMoles
            > std::max(1.0e-15,
                state.cycleFuelReferenceMoles * 1.0e-4);
        if (significantBurn) state.elapsedBurnSeconds += dtSeconds;

        const auto requestedTotal = requestedPremixed + requestedMixing;
        const auto instantaneousSharpness = requestedTotal > 1.0e-15
            ? requestedPremixed / requestedTotal : 0.0;
        state.sharpness = std::max(state.sharpness,
            std::clamp(instantaneousSharpness, 0.0, 1.0));
        state.active = significantBurn
            || chamber.mixture().fuelMoles
                > std::max(1.0e-15,
                    state.cycleFuelReferenceMoles * 1.0e-4);
    } else if (state.autoIgnited) {
        state.active = false;
    }

    result.ignitionDelaySeconds = state.ignitionDelaySeconds;
    result.burnedFraction = state.cycleFuelReferenceMoles > 1.0e-15
        ? std::clamp(state.cumulativeBurnedFuelMoles
            / state.cycleFuelReferenceMoles, 0.0, 1.0)
        : 0.0;
    result.efficiency = efficiency;
    result.burnRateFuelMolesPerSecond =
        result.reaction.burnedFuelMoles / dtSeconds;
    result.startPhaseDegrees = state.startPhaseDegrees;
    result.durationSeconds = state.elapsedBurnSeconds;
    result.sharpness = state.sharpness;
    result.autoIgnited = state.autoIgnited;
    result.active = state.active;
    return result;
}

} // namespace enginelab
