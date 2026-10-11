// A closed runner and a finite external plenum form a passive mass/energy
// system. This fixture preserves the current characteristic mouth boundary,
// RK2, mesh and CFL policy while varying only the requested hold interval.
// Default: an identical 12.5 us microstep updates true plenum inventory every
// call; only the copy supplied to the boundary is held. --macro-advance keeps
// the original experiment, whose internal CFL step distribution can differ.
// --finite-reservoir uses current external inventory and gives its physical
// volume to the network for a coupled reservoir evolution within each call.
// It diagnoses external-reservoir lag; no engine torque is calibrated here.
#include <enginelab/gasdynamics/ExhaustGasNetwork.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <string_view>

namespace {
using namespace enginelab::gasdynamics;

constexpr double runnerLengthM = 0.285;
constexpr double runnerDiameterM = 0.038;
constexpr double referencePressurePa = 101'325.0;
constexpr double referenceTemperatureK = 300.0;
constexpr double pressureAmplitudePa = 100.0;
constexpr double traversalCount = 500.0;
constexpr double productionCouplingSeconds = 400.0e-6;
constexpr double fixedMicrostepSeconds = 12.5e-6;
constexpr double maximumPassiveAmplitudeGain = 1.05;

void require(bool condition, std::string_view message) {
    if (condition) return;
    std::cerr << "FAILED: " << message << '\n';
    std::exit(EXIT_FAILURE);
}

[[nodiscard]] ExhaustNetworkLayout runnerLayout(std::size_t cellCount, bool tapered = false) {
    const auto areaM2 = std::numbers::pi * runnerDiameterM * runnerDiameterM * 0.25;
    const auto outletAreaM2 = tapered ? 2.25 * areaM2 : areaM2;
    const auto meanAreaM2 = tapered
        ? (areaM2 + std::sqrt(areaM2 * outletAreaM2) + outletAreaM2) / 3.0 : areaM2;
    CompiledExhaustDuct duct;
    duct.nodeId = 100;
    duct.lengthM = runnerLengthM;
    duct.flowAreaM2 = meanAreaM2;
    duct.inletFlowAreaM2 = areaM2;
    duct.outletFlowAreaM2 = outletAreaM2;
    duct.connectionAreaM2 = tapered ? 0.5 * (areaM2 + outletAreaM2) : areaM2;
    duct.inletConnectionAreaM2 = areaM2;
    duct.outletConnectionAreaM2 = outletAreaM2;
    duct.hydraulicDiameterM = tapered
        ? std::sqrt(4.0 * meanAreaM2 / std::numbers::pi) : runnerDiameterM;
    duct.volumeM3 = meanAreaM2 * runnerLengthM;
    duct.cellCount = cellCount;
    CompiledCylinderPort valve;
    valve.cylinderId = 1;
    valve.networkEndpoint = { ExhaustEndpointType::ductInlet, 0, 100 };
    valve.runnerConnectionAreaM2 = areaM2;
    CompiledExhaustOutlet mouth;
    mouth.outletNodeId = 200;
    mouth.networkEndpoint = { ExhaustEndpointType::ductOutlet, 0, 100 };
    mouth.openingAreaM2 = outletAreaM2;
    return ExhaustNetworkLayout::assemble({ duct }, {}, {}, { valve }, { mouth });
}

[[nodiscard]] ExhaustNetworkInventory combinedInventory(
    const ExhaustGasNetwork& network, const ConservativeState& plenum, double plenumVolumeM3) {
    auto inventory = network.inventory();
    for (std::size_t species = 0; species < gasSpeciesCount; ++species)
        inventory.speciesMassKg[species] +=
            plenum.speciesMassDensityKgPerM3[species] * plenumVolumeM3;
    inventory.totalEnergyJ += plenum.totalEnergyDensityJPerM3 * plenumVolumeM3;
    return inventory;
}

// Remove one GLOBAL volume-weighted pressure mean. Removing the runner's
// mean separately would hide the acoustic energy stored in the finite plenum.
[[nodiscard]] double acousticEnergyJ(const ExhaustGasNetwork& network,
    const ConservativeState& plenum, double plenumVolumeM3,
    double referenceDensityKgPerM3, double referenceSoundSpeedMps) {
    const auto plenumPrimitive = network.mixtureModel().primitiveFromConservative(plenum);
    require(plenumPrimitive.has_value(), "a measured finite plenum must remain physical");
    const auto& duct = network.ducts().front();
    const auto cells = duct.cells();
    const auto cellVolumeM3 = network.layout().ducts().front().volumeM3
        / static_cast<double>(cells.size());
    auto pressureVolumePaM3 = plenumPrimitive->pressurePa * plenumVolumeM3;
    for (const auto& cell : cells) {
        const auto primitive = network.mixtureModel().primitiveFromConservative(cell);
        require(primitive.has_value(), "a measured passive runner must remain physical");
        pressureVolumePaM3 += primitive->pressurePa * cellVolumeM3;
    }
    const auto globalPressureMeanPa = pressureVolumePaM3
        / (plenumVolumeM3 + cellVolumeM3 * static_cast<double>(cells.size()));
    const auto inverseAcousticStiffness = 1.0
        / (referenceDensityKgPerM3 * referenceSoundSpeedMps * referenceSoundSpeedMps);
    const auto plenumPressurePerturbationPa = plenumPrimitive->pressurePa - globalPressureMeanPa;
    auto energyJ = 0.5 * plenumVolumeM3 * plenumPressurePerturbationPa
        * plenumPressurePerturbationPa * inverseAcousticStiffness;
    for (const auto& cell : cells) {
        const auto primitive = network.mixtureModel().primitiveFromConservative(cell);
        require(primitive.has_value(), "a measured passive runner must recover");
        const auto pressurePerturbationPa = primitive->pressurePa - globalPressureMeanPa;
        energyJ += 0.5 * cellVolumeM3
            * (pressurePerturbationPa * pressurePerturbationPa * inverseAcousticStiffness
               + referenceDensityKgPerM3 * primitive->velocityMps * primitive->velocityMps);
    }
    require(std::isfinite(energyJ), "the joint acoustic perturbation must remain finite");
    return energyJ;
}

struct Result final {
    double initialAcousticEnergyJ { 0.0 };
    double maximumEnergyRatio { 1.0 };
    double finalEnergyRatio { 0.0 };
    double maximumRelativeMassError { 0.0 };
    double maximumRelativeEnergyError { 0.0 };
    double absoluteMouthTransferredMassKg { 0.0 };
    double advancedSeconds { 0.0 };
    double wallHeatRejectedJ { 0.0 };
    double minimumAcceptedStepSeconds { 0.0 };
    double maximumAcceptedStepSeconds { 0.0 };
    double maximumAcceptedStepRatio { 0.0 };
    double maximumCourantCurrent { 0.0 };
    double maximumCourantPredictor { 0.0 };
    std::size_t acceptedSubsteps { 0 };
    std::size_t rejectedSubsteps { 0 };
    std::size_t macroSteps { 0 };
    std::size_t boundaryRefreshes { 0 };
    std::size_t minimumAcceptedSubstepsPerCall { 0 };
    std::size_t maximumAcceptedSubstepsPerCall { 0 };
    bool fixedMicrostepsMatched { true };
    bool completed { true };
    bool physical { true };
    bool conserved { true };
};

[[nodiscard]] Result measure(std::size_t cellCount, double plenumVolumeM3,
                            double intervalSeconds, bool fixedMicrosteps, bool finiteReservoir) {
    require(!finiteReservoir || !fixedMicrosteps,
        "the finite-reservoir control must use the current plenum on every macro advance");
    ExhaustGasNetworkConfig config;
    config.initialPressurePa = referencePressurePa;
    config.initialTemperatureK = referenceTemperatureK;
    config.absoluteRoughnessM = 1.5e-6;
    config.maximumCourantNumber = 0.80;
    config.firstOrderTimeIntegration = false;
    config.wallHeatTransferWPerM2K = 0.0;
    config.dynamicWallHeatTransferEnabled = false;
    config.externalWallHeatTransferWPerM2K = 0.0;
    ExhaustGasNetwork network;
    require(network.configure(runnerLayout(cellCount), config), "the passive runner must configure");
    const auto base = network.mixtureModel().conservativeFromPressureTemperature(
        referencePressurePa, referenceTemperatureK);
    require(base.has_value(), "the reference finite reservoir must be physical");
    const auto basePrimitive = network.mixtureModel().primitiveFromConservative(*base);
    require(basePrimitive.has_value(), "the reference state must recover");
    auto plenum = *base;
    auto cells = network.ducts().front().cells();
    for (std::size_t index = 0; index < cells.size(); ++index) {
        const auto positionFraction = (static_cast<double>(index) + 0.5)
            / static_cast<double>(cells.size());
        const auto pressurePa = referencePressurePa + pressureAmplitudePa
            * std::cos(std::numbers::pi * positionFraction);
        const auto density = basePrimitive->densityKgPerM3
            * std::pow(pressurePa / referencePressurePa, 1.0 / basePrimitive->heatCapacityRatio);
        const auto state = network.mixtureModel().conservativeFromPrimitive(
            density, 0.0, pressurePa, GasComposition::dryAir());
        require(state.has_value(), "the nonzero isentropic perturbation must be physical");
        cells[index] = *state;
    }
    const auto before = combinedInventory(network, plenum, plenumVolumeM3);
    auto initialMassKg = 0.0;
    for (const auto mass : before.speciesMassKg) initialMassKg += mass;
    Result result;
    result.initialAcousticEnergyJ = acousticEnergyJ(network, plenum, plenumVolumeM3,
        basePrimitive->densityKgPerM3, basePrimitive->speedOfSoundMps);
    require(result.initialAcousticEnergyJ > 0.0, "the joint fixture must contain a nonzero mode");
    const auto requestedSeconds = traversalCount * runnerLengthM / basePrimitive->speedOfSoundMps;
    const auto stepSeconds = fixedMicrosteps ? fixedMicrostepSeconds : intervalSeconds;
    const auto stepCount = static_cast<std::size_t>(std::ceil(requestedSeconds / stepSeconds));
    const auto holdMicrosteps = static_cast<std::size_t>(std::llround(
        intervalSeconds / fixedMicrostepSeconds));
    require(holdMicrosteps > 0 && std::abs(intervalSeconds
                - static_cast<double>(holdMicrosteps) * fixedMicrostepSeconds) <= 1.0e-15,
        "boundary refresh intervals must be exact multiples of the fixed microstep");
    auto suppliedPlenum = plenum;
    for (std::size_t step = 0; step < stepCount; ++step) {
        // Complete microsteps give every held-boundary case exactly the same
        // integration grid. The last macro advance retains its original clip.
        const auto durationSeconds = fixedMicrosteps ? fixedMicrostepSeconds
            : std::min(intervalSeconds,
                requestedSeconds - static_cast<double>(step) * intervalSeconds);
        if (!fixedMicrosteps || step % holdMicrosteps == 0) {
            suppliedPlenum = plenum;
            ++result.boundaryRefreshes;
        }
        // The valve is shut. The caller owns the finite plenum inventory and
        // applies the integrated mouth transfer once after EVERY call. The
        // historical held-boundary controls explicitly keep network volume 0;
        // the finite-reservoir control supplies current inventory AND volume.
        const auto suppliedReservoirVolumeM3 = finiteReservoir ? plenumVolumeM3 : 0.0;
        const auto advance = network.advance(durationSeconds, {},
            { suppliedPlenum, 1.0, suppliedReservoirVolumeM3 });
        ++result.macroSteps;
        result.minimumAcceptedSubstepsPerCall = result.macroSteps == 1
            ? advance.acceptedSubsteps
            : std::min(result.minimumAcceptedSubstepsPerCall, advance.acceptedSubsteps);
        result.maximumAcceptedSubstepsPerCall = std::max(
            result.maximumAcceptedSubstepsPerCall, advance.acceptedSubsteps);
        if (fixedMicrosteps)
            result.fixedMicrostepsMatched = result.fixedMicrostepsMatched
                && advance.acceptedSubsteps == 1 && advance.rejectedSubsteps == 0
                && std::abs(advance.minimumAcceptedTimeStepSeconds - fixedMicrostepSeconds) <= 1.0e-15
                && std::abs(advance.maximumAcceptedTimeStepSeconds - fixedMicrostepSeconds) <= 1.0e-15;
        result.acceptedSubsteps += advance.acceptedSubsteps;
        result.rejectedSubsteps += advance.rejectedSubsteps;
        result.advancedSeconds += advance.advancedTimeSeconds;
        result.wallHeatRejectedJ += advance.wallHeatRejectedJ;
        if (advance.minimumAcceptedTimeStepSeconds > 0.0)
            result.minimumAcceptedStepSeconds = result.minimumAcceptedStepSeconds == 0.0
                ? advance.minimumAcceptedTimeStepSeconds
                : std::min(result.minimumAcceptedStepSeconds, advance.minimumAcceptedTimeStepSeconds);
        result.maximumAcceptedStepSeconds = std::max(result.maximumAcceptedStepSeconds,
            advance.maximumAcceptedTimeStepSeconds);
        result.maximumAcceptedStepRatio = std::max(result.maximumAcceptedStepRatio,
            advance.maximumAcceptedStableStepRatio);
        result.maximumCourantCurrent = std::max(result.maximumCourantCurrent,
            advance.maximumAcceptedDuctCourantCurrent);
        result.maximumCourantPredictor = std::max(result.maximumCourantPredictor,
            advance.maximumAcceptedDuctCourantPredictor);
        // Positive outlet transfer leaves the runner and ENTERS the plenum.
        // Apply its exact integrated inventory once, with no damping or reset.
        const auto& mouth = network.outletSamples().front();
        for (std::size_t species = 0; species < gasSpeciesCount; ++species) {
            plenum.speciesMassDensityKgPerM3[species] += mouth.speciesMassKg[species] / plenumVolumeM3;
            result.absoluteMouthTransferredMassKg += std::abs(mouth.speciesMassKg[species]);
        }
        plenum.totalEnergyDensityJPerM3 += mouth.transferredEnergyJ / plenumVolumeM3;
        const auto& observedNetwork = network;
        const auto observedCells = observedNetwork.ducts().front().cells();
        result.physical = network.mixtureModel().isPhysical(plenum)
            && std::all_of(observedCells.begin(), observedCells.end(),
                [&](const ConservativeState& state) { return network.mixtureModel().isPhysical(state); });
        const auto after = combinedInventory(network, plenum, plenumVolumeM3);
        result.maximumRelativeEnergyError = std::max(result.maximumRelativeEnergyError,
            std::abs(after.totalEnergyJ - before.totalEnergyJ) / before.totalEnergyJ);
        for (std::size_t species = 0; species < gasSpeciesCount; ++species) {
            const auto errorKg = std::abs(after.speciesMassKg[species] - before.speciesMassKg[species]);
            result.maximumRelativeMassError = std::max(result.maximumRelativeMassError,
                errorKg / initialMassKg);
            result.conserved = result.conserved && errorKg
                <= 1.0e-14 + std::abs(before.speciesMassKg[species]) * 1.0e-10;
        }
        result.conserved = result.conserved && result.maximumRelativeEnergyError <= 1.0e-10;
        result.completed = advance.completed && std::abs(advance.advancedTimeSeconds - durationSeconds)
            <= std::max(1.0e-15, durationSeconds * 1.0e-12);
        if (!result.physical || !result.completed) break;
        const auto energyRatio = acousticEnergyJ(network, plenum, plenumVolumeM3,
            basePrimitive->densityKgPerM3, basePrimitive->speedOfSoundMps)
            / result.initialAcousticEnergyJ;
        result.maximumEnergyRatio = std::max(result.maximumEnergyRatio, energyRatio);
        result.finalEnergyRatio = energyRatio;
    }
    std::cout << "mode=" << (finiteReservoir ? "finite_reservoir"
        : fixedMicrosteps ? "fixed_microstep" : "macro_advance")
        << " cells=" << cellCount << " plenum_L=" << plenumVolumeM3 * 1'000.0
        << " supplied_reservoir_volume_m3=" << (finiteReservoir ? plenumVolumeM3 : 0.0)
        << " hold_us=" << intervalSeconds * 1.0e6
        << " requested_step_us=" << stepSeconds * 1.0e6
        << " initial_acoustic_J=" << result.initialAcousticEnergyJ
        << " max_energy_gain=" << result.maximumEnergyRatio
        << " max_amplitude_gain=" << std::sqrt(result.maximumEnergyRatio)
        << " final_energy_gain=" << result.finalEnergyRatio
        << " mass_relative_error=" << result.maximumRelativeMassError
        << " total_energy_relative_error=" << result.maximumRelativeEnergyError
        << " absolute_mouth_transfer_kg=" << result.absoluteMouthTransferredMassKg
        << " advanced_s=" << result.advancedSeconds
        << " macro_steps=" << result.macroSteps << " accepted=" << result.acceptedSubsteps
        << " boundary_refreshes=" << result.boundaryRefreshes
        << " accepted_per_call=[" << result.minimumAcceptedSubstepsPerCall << ','
        << result.maximumAcceptedSubstepsPerCall << ']'
        << " rejected=" << result.rejectedSubsteps
        << " accepted_dt_us=[" << result.minimumAcceptedStepSeconds * 1.0e6 << ','
        << result.maximumAcceptedStepSeconds * 1.0e6 << ']'
        << " accepted_step_ratio=" << result.maximumAcceptedStepRatio
        << " courant_current=" << result.maximumCourantCurrent
        << " courant_predictor=" << result.maximumCourantPredictor
        << " completed=" << result.completed << " physical=" << result.physical
        << " conserved=" << result.conserved
        << " fixed_microsteps_matched=" << result.fixedMicrostepsMatched << '\n';
    return result;
}

void boundaryAdmittance(bool enforceLimit) {
    ExhaustGasNetworkConfig config;
    config.initialPressurePa = referencePressurePa;
    config.initialTemperatureK = referenceTemperatureK;
    config.maximumCourantNumber = 0.80;
    config.firstOrderTimeIntegration = false;
    config.wallHeatTransferWPerM2K = 0.0;
    config.dynamicWallHeatTransferEnabled = false;
    ExhaustGasNetwork network;
    require(network.configure(runnerLayout(12), config), "the admittance runner must configure");
    const auto base = network.mixtureModel().conservativeFromPressureTemperature(
        referencePressurePa, referenceTemperatureK);
    require(base.has_value(), "the reference admittance reservoir must be physical");
    const auto primitive = network.mixtureModel().primitiveFromConservative(*base);
    require(primitive.has_value(), "the reference admittance reservoir must recover");
    const auto areaM2 = network.layout().outlets().front().openingAreaM2;
    // predictOutletTransfer includes a local terminal-cell Heun extrapolation.
    // Take its dt -> 0 limit so the control observes the current mouth law,
    // without a temporal predictor, plenum update, wall source or FV advance.
    constexpr double predictionSeconds = 1.0e-15;
    constexpr double smallSignalTolerance = 0.10;
    auto linear = true;
    auto symmetric = true;
    auto validSigns = true;
    std::cout << "boundary_admittance prediction_s=" << predictionSeconds
        << " cells=12 p0_Pa=" << referencePressurePa << " T0_K=" << referenceTemperatureK
        << " mouth_area_m2=" << areaM2 << " c0_mps=" << primitive->speedOfSoundMps << '\n';
    for (const auto pressureOffsetPa : std::array<double, 5> { 0.01, 0.1, 1.0, 10.0, 100.0 }) {
        std::array<double, 2> signedMassFlowKgPerS {};
        for (std::size_t index = 0; index < signedMassFlowKgPerS.size(); ++index) {
            const auto pressurePa = referencePressurePa
                + (index == 0 ? pressureOffsetPa : -pressureOffsetPa);
            const auto densityKgPerM3 = primitive->densityKgPerM3
                * std::pow(pressurePa / referencePressurePa, 1.0 / primitive->heatCapacityRatio);
            const auto state = network.mixtureModel().conservativeFromPrimitive(
                densityKgPerM3, 0.0, pressurePa, GasComposition::dryAir());
            require(state.has_value(), "uniform isentropic admittance probes must be physical");
            auto cells = network.ducts().front().cells();
            std::fill(cells.begin(), cells.end(), *state);
            const auto before = network.inventory();
            const auto prediction = network.predictOutletTransfer(0, { *base, 1.0 }, predictionSeconds);
            require(prediction.has_value(), "the mouth admittance prediction must exist");
            const auto after = network.inventory();
            require(before.speciesMassKg == after.speciesMassKg
                        && before.totalEnergyJ == after.totalEnergyJ,
                "the admittance control must not advance or change gas inventory");
            signedMassFlowKgPerS[index] = prediction->massFlowKgPerS;
            require(std::isfinite(signedMassFlowKgPerS[index]), "the predicted raw flow must be finite");
        }
        // For cold dry air near rest, characteristic compatibility gives
        // m_dot = A * dp / c0 to first order. 10% allows the finite probe,
        // HLLC recovery and the negligible positive-duration predictor. This
        // limit is deliberately not asserted for hot mixed-entropy contacts.
        const auto scale = primitive->speedOfSoundMps / (areaM2 * pressureOffsetPa);
        const auto normalisedOutflow = signedMassFlowKgPerS[0] * scale;
        const auto normalisedInflow = -signedMassFlowKgPerS[1] * scale;
        const auto asymmetry = std::abs(normalisedOutflow - normalisedInflow)
            / std::max({ 1.0e-30, std::abs(normalisedOutflow), std::abs(normalisedInflow) });
        validSigns = validSigns && signedMassFlowKgPerS[0] > 0.0 && signedMassFlowKgPerS[1] < 0.0;
        if (pressureOffsetPa < 1.0) {
            linear = linear && std::abs(normalisedOutflow - 1.0) <= smallSignalTolerance
                && std::abs(normalisedInflow - 1.0) <= smallSignalTolerance;
            symmetric = symmetric && asymmetry <= smallSignalTolerance;
        }
        std::cout << "delta_p_Pa=" << pressureOffsetPa
            << " raw_outflow_kgps=" << signedMassFlowKgPerS[0]
            << " raw_inflow_kgps=" << signedMassFlowKgPerS[1]
            << " normalised_outflow=" << normalisedOutflow
            << " normalised_inflow=" << normalisedInflow
            << " relative_asymmetry=" << asymmetry << '\n';
    }
    require(validSigns, "the uniform probes must have correctly signed nonzero mouth flows");
    if (enforceLimit) {
        require(linear, "the cold dry-air mouth must approach finite linear acoustic admittance");
        require(symmetric, "the cold dry-air small-signal mouth response must be symmetric within 10 percent");
    }
}

void outletPredictionEquilibrium(bool enforceEquilibrium) {
    constexpr double massToleranceKg = 1.0e-14;
    constexpr double energyToleranceJ = 1.0e-8;
    auto predictionsAtEquilibrium = true;
    auto actualAdvancesAtEquilibrium = true;
    struct GeometryProbe final {
        bool tapered;
        double openingFraction;
    };
    for (const auto cellCount : std::array<std::size_t, 3> { 3, 6, 12 }) {
        for (const auto probe : std::array<GeometryProbe, 3> {
                   GeometryProbe { false, 1.0 }, { true, 0.35 }, { true, 1.0 } }) {
            ExhaustGasNetworkConfig config;
            config.initialPressurePa = referencePressurePa;
            config.initialTemperatureK = referenceTemperatureK;
            config.maximumCourantNumber = 0.80;
            config.firstOrderTimeIntegration = false;
            config.wallHeatTransferWPerM2K = 0.0;
            config.dynamicWallHeatTransferEnabled = false;
            ExhaustGasNetwork network;
            require(network.configure(runnerLayout(cellCount, probe.tapered), config),
                "the equilibrium runner must configure");
            const auto& geometry = network.ducts().front().geometry();
            auto actualVolumeM3 = 0.0;
            for (std::size_t cell = 0; cell < cellCount; ++cell)
                actualVolumeM3 += geometry.cellVolumeM3(cell);
            require(geometry.hasVariableArea() == probe.tapered,
                "the accepted equilibrium geometry must retain the requested taper");
            require(std::abs(actualVolumeM3 - network.layout().ducts().front().volumeM3) <= 1.0e-15,
                "the accepted cell volumes must sum to the authored frustum volume");
            require(geometry.inletAreaM2() > 0.0 && geometry.outletAreaM2() > 0.0
                        && (!probe.tapered || geometry.outletAreaM2() > 2.0 * geometry.inletAreaM2()),
                "the conical equilibrium probe must have distinct positive end-face areas");
            const auto base = network.mixtureModel().conservativeFromPressureTemperature(
                referencePressurePa, referenceTemperatureK);
            require(base.has_value(), "the equilibrium reservoir must be physical");
            const auto reference = network.mixtureModel().primitiveFromConservative(*base);
            require(reference.has_value(), "the equilibrium reference must recover");
            for (const auto durationSeconds : std::array<double, 6> {
                     5.0e-6, 25.0e-6, 50.0e-6, 125.0e-6, 200.0e-6, 400.0e-6 }) {
                // Keep all 18 original constant-area cases. The extra 12 cases
                // isolate quasi-1D wall-pressure balance and a partially closed
                // mouth; neither changes the exact uniform equilibrium.
                if (probe.tapered && durationSeconds != 25.0e-6 && durationSeconds != 200.0e-6)
                    continue;
                require(network.reset(referencePressurePa, referenceTemperatureK),
                    "each equilibrium duration must start from the same uniform state");
                const auto before = network.inventory();
                const auto prediction = network.predictOutletTransfer(
                    0, { *base, probe.openingFraction }, durationSeconds);
                require(prediction.has_value(), "the equilibrium outlet prediction must exist");
                const auto afterPrediction = network.inventory();
                const auto predictionDidNotAdvance = before.speciesMassKg == afterPrediction.speciesMassKg
                    && before.totalEnergyJ == afterPrediction.totalEnergyJ;
                auto predictionMaximumSpeciesMassKg = 0.0;
                for (const auto massKg : prediction->speciesMassKg)
                    predictionMaximumSpeciesMassKg = std::max(predictionMaximumSpeciesMassKg, std::abs(massKg));
                const auto predictedZero = predictionDidNotAdvance
                    && predictionMaximumSpeciesMassKg <= massToleranceKg
                    && std::abs(prediction->transferredEnergyJ) <= energyToleranceJ;
                predictionsAtEquilibrium = predictionsAtEquilibrium && predictedZero;

                // Same initial state, geometry, reservoir and requested duration;
                // advance() balances all duct-face stresses and is an independent
                // check against the local outlet predictor's extrapolation.
                const auto advance = network.advance(durationSeconds, {}, { *base, probe.openingFraction });
                const auto& outlet = network.outletSamples().front();
                auto actualMaximumSpeciesMassKg = 0.0;
                for (const auto massKg : outlet.speciesMassKg)
                    actualMaximumSpeciesMassKg = std::max(actualMaximumSpeciesMassKg, std::abs(massKg));
                const auto after = network.inventory();
                auto maximumInventoryMassErrorKg = 0.0;
                for (std::size_t species = 0; species < gasSpeciesCount; ++species)
                    maximumInventoryMassErrorKg = std::max(maximumInventoryMassErrorKg,
                        std::abs(after.speciesMassKg[species] - before.speciesMassKg[species]));
                const auto inventoryEnergyErrorJ = std::abs(after.totalEnergyJ - before.totalEnergyJ);
                auto maximumPressureErrorPa = 0.0;
                auto maximumVelocityMps = 0.0;
                auto maximumDensityErrorKgPerM3 = 0.0;
                auto physical = true;
                const auto& observedNetwork = network;
                for (const auto& state : observedNetwork.ducts().front().cells()) {
                    const auto primitive = network.mixtureModel().primitiveFromConservative(state);
                    if (!primitive) {
                        physical = false;
                        continue;
                    }
                    maximumPressureErrorPa = std::max(maximumPressureErrorPa,
                        std::abs(primitive->pressurePa - reference->pressurePa));
                    maximumVelocityMps = std::max(maximumVelocityMps, std::abs(primitive->velocityMps));
                    maximumDensityErrorKgPerM3 = std::max(maximumDensityErrorKgPerM3,
                        std::abs(primitive->densityKgPerM3 - reference->densityKgPerM3));
                }
                const auto actualZero = advance.completed && physical
                    && std::abs(advance.advancedTimeSeconds - durationSeconds) <= 1.0e-15
                    && actualMaximumSpeciesMassKg <= massToleranceKg
                    && std::abs(outlet.transferredEnergyJ) <= energyToleranceJ
                    && maximumInventoryMassErrorKg <= massToleranceKg
                    && inventoryEnergyErrorJ <= energyToleranceJ
                    && maximumPressureErrorPa <= 1.0e-7
                    && maximumVelocityMps <= 1.0e-9
                    && maximumDensityErrorKgPerM3 <= 1.0e-12;
                actualAdvancesAtEquilibrium = actualAdvancesAtEquilibrium && actualZero;
                std::cout << "outlet_prediction_equilibrium cells=" << cellCount
                    << " geometry=" << (probe.tapered ? "conical" : "constant")
                    << " opening_fraction=" << probe.openingFraction
                    << " accepted_inlet_area_m2=" << geometry.inletAreaM2()
                    << " accepted_outlet_area_m2=" << geometry.outletAreaM2()
                    << " accepted_volume_m3=" << actualVolumeM3
                    << " duration_us=" << durationSeconds * 1.0e6
                    << " predicted_mass_flow_kgps=" << prediction->massFlowKgPerS
                    << " predicted_max_species_kg=" << predictionMaximumSpeciesMassKg
                    << " predicted_energy_J=" << prediction->transferredEnergyJ
                    << " prediction_inventory_unchanged=" << predictionDidNotAdvance
                    << " actual_mass_flow_kgps=" << outlet.massFlowKgPerS
                    << " actual_max_species_kg=" << actualMaximumSpeciesMassKg
                    << " actual_energy_J=" << outlet.transferredEnergyJ
                    << " actual_inventory_mass_error_kg=" << maximumInventoryMassErrorKg
                    << " actual_inventory_energy_error_J=" << inventoryEnergyErrorJ
                    << " actual_pressure_error_Pa=" << maximumPressureErrorPa
                    << " actual_max_velocity_mps=" << maximumVelocityMps
                    << " actual_density_error_kgpm3=" << maximumDensityErrorKgPerM3
                    << " actual_accepted=" << advance.acceptedSubsteps
                    << " actual_rejected=" << advance.rejectedSubsteps
                    << " predictor_zero=" << predictedZero << " actual_zero=" << actualZero << '\n';
            }
        }
    }
    require(actualAdvancesAtEquilibrium,
        "the independent complete advances must preserve exact uniform equilibrium");
    if (enforceEquilibrium)
        require(predictionsAtEquilibrium,
            "the outlet predictor must return zero species and energy transfer at equilibrium");
}
} // namespace

int main(int argc, char** argv) {
    auto diagnosticOnly = false;
    auto fixedMicrosteps = false;
    auto admittanceOnly = false;
    auto predictionEquilibriumOnly = false;
    auto finiteReservoir = false;
    auto historicalHoldRequested = false;
    for (int index = 1; index < argc; ++index) {
        const auto argument = std::string_view(argv[index]);
        if (argument == "--diagnostic-only") diagnosticOnly = true;
        else if (argument == "--macro-advance") {
            fixedMicrosteps = false;
            historicalHoldRequested = true;
        } else if (argument == "--fixed-microstep") {
            fixedMicrosteps = true;
            historicalHoldRequested = true;
        }
        else if (argument == "--finite-reservoir") finiteReservoir = true;
        else if (argument == "--boundary-admittance") admittanceOnly = true;
        else if (argument == "--outlet-prediction-equilibrium") predictionEquilibriumOnly = true;
        else require(false,
            "supported options: --diagnostic-only, --macro-advance, --fixed-microstep, "
            "--finite-reservoir, --boundary-admittance, --outlet-prediction-equilibrium");
    }
    std::cout << std::setprecision(12);
    require(!(admittanceOnly && predictionEquilibriumOnly),
        "select one independent boundary control per run");
    if (predictionEquilibriumOnly) {
        outletPredictionEquilibrium(!diagnosticOnly);
        return EXIT_SUCCESS;
    }
    if (admittanceOnly) {
        boundaryAdmittance(!diagnosticOnly);
        return EXIT_SUCCESS;
    }
    // The regression default follows production's finite plenum. Explicit
    // historical modes retain the held-boundary experiments for diagnosis.
    finiteReservoir = finiteReservoir || !historicalHoldRequested;
    // A finite reservoir is never supplied a deliberately stale copy, even
    // if an earlier or later CLI option requested the held-microstep protocol.
    if (finiteReservoir) fixedMicrosteps = false;
    auto fixturesHealthy = true;
    auto selectedIntervalsPassive = true;
    for (const auto cellCount : std::array<std::size_t, 3> { 3, 6, 12 }) {
        for (const auto plenumVolumeM3 : std::array<double, 3> { 1.0e-3, 2.5e-3, 7.0e-3 }) {
            for (const auto intervalSeconds : std::array<double, 4> {
                     productionCouplingSeconds, 125.0e-6, 50.0e-6, fixedMicrostepSeconds }) {
                if (!fixedMicrosteps && !finiteReservoir && intervalSeconds == fixedMicrostepSeconds)
                    continue;
                const auto result = measure(
                    cellCount, plenumVolumeM3, intervalSeconds, fixedMicrosteps, finiteReservoir);
                fixturesHealthy = fixturesHealthy && result.completed && result.physical && result.conserved
                    && result.absoluteMouthTransferredMassKg > 1.0e-12
                    && std::abs(result.wallHeatRejectedJ) < 1.0e-14
                    && result.fixedMicrostepsMatched;
                if (finiteReservoir || intervalSeconds == productionCouplingSeconds)
                    selectedIntervalsPassive = selectedIntervalsPassive && std::sqrt(result.maximumEnergyRatio)
                        <= maximumPassiveAmplitudeGain;
            }
        }
    }
    require(fixturesHealthy,
        "all passive fixtures must complete physically, conserve inventory and exchange nonzero mass");
    if (!diagnosticOnly)
        require(selectedIntervalsPassive, finiteReservoir
            ? "every finite-reservoir macro interval must preserve the joint passive mode"
            : "the production hold interval must not amplify the joint passive mode");
    return EXIT_SUCCESS;
}
