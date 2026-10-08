// A motored, adiabatic cylinder and a finite plenum isolate the history
// reduction used by the multirate intake from combustion, ECU and thermal drift.
// Every coupling case uses the same mechanical and FV microstep grids. The
// delayed virtual cylinder has the production independently averaged U/V, V
// and valve aperture; its accepted exchanges are applied once at the physical
// cylinder's macro endpoint. This is a diagnostic, not a calibrated engine.
#include <enginelab/gasdynamics/ExhaustGasNetwork.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <string_view>
#include <utility>
#include <vector>

namespace {
using namespace enginelab::gasdynamics;
constexpr double mechanicalStepSeconds = 12.5e-6;
constexpr double plenumVolumeM3 = 2.5e-3;
constexpr double sweptVolumeM3 = 250.0e-6;
constexpr double clearanceVolumeM3 = sweptVolumeM3 / 9.0;
constexpr double maximumValveAreaM2 = 300.0e-6;
constexpr double pressurePa = 101'325.0;
constexpr double temperatureK = 300.0;

void require(bool condition, std::string_view message) {
    if (condition) return;
    std::cerr << "FAILED: " << message << '\n';
    std::exit(EXIT_FAILURE);
}

[[nodiscard]] ExhaustNetworkLayout runnerLayout(std::size_t runnerCount = 1) {
    constexpr double lengthM = 0.285;
    constexpr double diameterM = 0.038;
    constexpr double areaM2 = std::numbers::pi * diameterM * diameterM * 0.25;
    CompiledExhaustDuct duct;
    duct.nodeId = 100;
    duct.lengthM = lengthM;
    duct.flowAreaM2 = areaM2;
    duct.inletFlowAreaM2 = areaM2;
    duct.outletFlowAreaM2 = areaM2;
    duct.connectionAreaM2 = areaM2;
    duct.inletConnectionAreaM2 = areaM2;
    duct.outletConnectionAreaM2 = areaM2;
    duct.hydraulicDiameterM = diameterM;
    duct.volumeM3 = lengthM * areaM2;
    duct.cellCount = 3;
    CompiledCylinderPort valve;
    valve.cylinderId = 1;
    valve.networkEndpoint = { ExhaustEndpointType::ductInlet, 0, 100 };
    valve.runnerConnectionAreaM2 = areaM2;
    CompiledExhaustOutlet mouth;
    mouth.outletNodeId = 200;
    mouth.networkEndpoint = { ExhaustEndpointType::ductOutlet, 0, 100 };
    mouth.openingAreaM2 = areaM2;
    std::vector<CompiledExhaustDuct> ducts;
    std::vector<CompiledCylinderPort> valves;
    std::vector<CompiledExhaustOutlet> mouths;
    for (std::size_t index = 0; index < runnerCount; ++index) {
        const auto id = static_cast<std::uint32_t>(index + 1);
        auto copyDuct = duct;
        copyDuct.nodeId = 100 + id;
        auto copyValve = valve;
        copyValve.cylinderId = id;
        copyValve.networkEndpoint = { ExhaustEndpointType::ductInlet, index, 100 + id };
        auto copyMouth = mouth;
        copyMouth.outletNodeId = 200 + id;
        copyMouth.networkEndpoint = { ExhaustEndpointType::ductOutlet, index, 100 + id };
        ducts.push_back(copyDuct);
        valves.push_back(copyValve);
        mouths.push_back(copyMouth);
    }
    return ExhaustNetworkLayout::assemble(std::move(ducts), {}, {}, std::move(valves), std::move(mouths));
}

void configure(ExhaustGasNetwork& network, std::size_t runnerCount = 1) {
    ExhaustGasNetworkConfig config;
    config.initialPressurePa = pressurePa;
    config.initialTemperatureK = temperatureK;
    config.maximumCourantNumber = 0.8;
    config.wallHeatTransferWPerM2K = 0.0;
    config.dynamicWallHeatTransferEnabled = false;
    config.externalWallHeatTransferWPerM2K = 0.0;
    require(network.configure(runnerLayout(runnerCount), config), "the fixed runner must configure");
}

[[nodiscard]] double volume(double angleRadians) {
    return clearanceVolumeM3 + 0.5 * sweptVolumeM3 * (1.0 - std::cos(angleRadians));
}

[[nodiscard]] double valveArea(double angleRadians) {
    constexpr double closingRadians = 240.0 * std::numbers::pi / 180.0;
    const auto phase = std::fmod(angleRadians, 4.0 * std::numbers::pi);
    if (phase >= closingRadians) return 0.0;
    const auto lift = std::sin(std::numbers::pi * phase / closingRadians);
    return maximumValveAreaM2 * lift * lift;
}

[[nodiscard]] ConservativeState scaleState(const ConservativeState& state, double scale) {
    auto result = state;
    for (auto& mass : result.speciesMassDensityKgPerM3) mass *= scale;
    result.totalEnergyDensityJPerM3 *= scale;
    result.momentumDensityKgPerM2S *= scale;
    return result;
}

void addState(ConservativeState& sum, const ConservativeState& state, double scale) {
    for (std::size_t species = 0; species < gasSpeciesCount; ++species)
        sum.speciesMassDensityKgPerM3[species] += state.speciesMassDensityKgPerM3[species] * scale;
    sum.totalEnergyDensityJPerM3 += state.totalEnergyDensityJPerM3 * scale;
    sum.momentumDensityKgPerM2S += state.momentumDensityKgPerM2S * scale;
}

[[nodiscard]] double pistonAdvance(ConservativeState& cylinder, double& cylinderVolume,
                                   double newVolume, const EulerMixtureModel& model) {
    const auto primitive = model.primitiveFromConservative(cylinder);
    require(primitive.has_value(), "the pre-work chamber must be physical");
    const auto oldEnergy = cylinder.totalEnergyDensityJPerM3 * cylinderVolume;
    const auto ratio = cylinderVolume / newVolume;
    for (auto& mass : cylinder.speciesMassDensityKgPerM3) mass *= ratio;
    cylinder.totalEnergyDensityJPerM3 *= std::pow(ratio, primitive->heatCapacityRatio);
    cylinderVolume = newVolume;
    // Exact calorically-perfect adiabatic work for this closed mechanical
    // increment. Positive means piston work delivered to the gas.
    return cylinder.totalEnergyDensityJPerM3 * cylinderVolume - oldEnergy;
}

void applyCylinderExchange(ConservativeState& cylinder, double cylinderVolume,
                            const CylinderGasExchange& exchange) {
    for (std::size_t species = 0; species < gasSpeciesCount; ++species)
        cylinder.speciesMassDensityKgPerM3[species] -= exchange.speciesMassKg[species] / cylinderVolume;
    cylinder.totalEnergyDensityJPerM3 -= exchange.totalEnergyJ / cylinderVolume;
}

void applyMouthExchange(ConservativeState& plenum, const ExhaustOutletFlowSample& exchange) {
    for (std::size_t species = 0; species < gasSpeciesCount; ++species)
        plenum.speciesMassDensityKgPerM3[species] += exchange.speciesMassKg[species] / plenumVolumeM3;
    plenum.totalEnergyDensityJPerM3 += exchange.transferredEnergyJ / plenumVolumeM3;
}

[[nodiscard]] ExhaustNetworkInventory totalInventory(const ExhaustGasNetwork& network,
    const ConservativeState& cylinder, double cylinderVolume, const ConservativeState& plenum) {
    auto result = network.inventory();
    for (std::size_t species = 0; species < gasSpeciesCount; ++species)
        result.speciesMassKg[species] += cylinder.speciesMassDensityKgPerM3[species] * cylinderVolume
            + plenum.speciesMassDensityKgPerM3[species] * plenumVolumeM3;
    result.totalEnergyJ += cylinder.totalEnergyDensityJPerM3 * cylinderVolume
        + plenum.totalEnergyDensityJPerM3 * plenumVolumeM3;
    return result;
}

struct History final {
    ConservativeState densities;
    ConservativeState inventories;
    double volumes { 0.0 };
    double valveAreas { 0.0 };
    std::size_t count { 0 };
    void add(const ConservativeState& state, double chamberVolume, double area) {
        addState(densities, state, 1.0);
        addState(inventories, state, chamberVolume);
        volumes += chamberVolume;
        valveAreas += area;
        ++count;
    }
};

// The fixed runner is never advanced. The exact same prescribed chamber
// history is sampled with its real aperture and with separate means. This
// removes FV time, gas feedback and energy bookkeeping as alternative causes.
void frozenHistory(double rpm, double couplingSeconds) {
    ExhaustGasNetwork network;
    configure(network);
    const auto& model = network.mixtureModel();
    const auto base = model.conservativeFromPressureTemperature(pressurePa, temperatureK);
    require(base.has_value(), "the reference state must recover");
    auto cylinder = *base;
    auto chamberVolume = volume(std::numbers::pi);
    const auto angularVelocity = rpm * 2.0 * std::numbers::pi / 60.0;
    const auto macroSteps = static_cast<std::size_t>(std::llround(couplingSeconds / mechanicalStepSeconds));
    auto history = History {};
    auto exactMass = 0.0;
    auto reducedMass = 0.0;
    auto absoluteExactMass = 0.0;
    auto maximumRelativeInventoryBias = 0.0;
    auto maximumWindowAbsoluteErrorKg = 0.0;
    auto exactWindow = 0.0;
    constexpr double intakeDurationRadians = 240.0 * std::numbers::pi / 180.0;
    // 160 mechanical steps partition every selected coupling interval, so
    // each rpm uses exactly the same history coverage in all four reductions.
    constexpr double commonBlockSeconds = 160.0 * mechanicalStepSeconds;
    const auto count = static_cast<std::size_t>(std::floor(
        intakeDurationRadians / angularVelocity / commonBlockSeconds)) * 160;
    std::array<CylinderBoundaryFlowSample, 1> sample;
    for (std::size_t step = 0; step < count; ++step) {
        const auto angle = angularVelocity * (static_cast<double>(step) + 0.5) * mechanicalStepSeconds;
        static_cast<void>(pistonAdvance(cylinder, chamberVolume, volume(angle), model));
        const CylinderValveBoundary boundary { 1, cylinder, chamberVolume, valveArea(angle), 1.0 };
        require(network.sampleCylinderBoundaries(std::span(&boundary, 1), sample) && sample[0].valid,
            "each real moving boundary sample must recover");
        const auto transfer = sample[0].massFlowKgPerSecond * mechanicalStepSeconds;
        exactMass += transfer;
        exactWindow += transfer;
        absoluteExactMass += std::abs(transfer);
        history.add(cylinder, chamberVolume, boundary.effectiveValveAreaM2);
        if (history.count != macroSteps) continue;
        const auto inverseCount = 1.0 / static_cast<double>(history.count);
        const auto reducedState = scaleState(history.densities, inverseCount);
        const auto reducedVolume = history.volumes * inverseCount;
        const CylinderValveBoundary reduced { 1, reducedState, reducedVolume,
            history.valveAreas * inverseCount, 1.0 };
        require(network.sampleCylinderBoundaries(std::span(&reduced, 1), sample) && sample[0].valid,
            "the averaged boundary sample must recover");
        const auto macroTransfer = sample[0].massFlowKgPerSecond * couplingSeconds;
        reducedMass += macroTransfer;
        maximumWindowAbsoluteErrorKg = std::max(maximumWindowAbsoluteErrorKg,
            std::abs(macroTransfer - exactWindow));
        const auto meanMass = history.inventories.densityKgPerM3() * inverseCount;
        maximumRelativeInventoryBias = std::max(maximumRelativeInventoryBias,
            std::abs(reducedState.densityKgPerM3() * reducedVolume - meanMass) / meanMass);
        history = {};
        exactWindow = 0.0;
    }
    require(count > 0 && absoluteExactMass > 1.0e-7, "the frozen history must exercise nonzero flow");
    std::cout << "frozen rpm=" << rpm << " coupling_us=" << couplingSeconds * 1.0e6
        << " exact_net_kg=" << exactMass << " averaged_net_kg=" << reducedMass
        << " exact_absolute_kg=" << absoluteExactMass
        << " net_error_over_absolute=" << (reducedMass - exactMass) / absoluteExactMass
        << " maximum_window_error_kg=" << maximumWindowAbsoluteErrorKg
        << " maximum_relative_inventory_bias=" << maximumRelativeInventoryBias << '\n';
}

struct MovingResult final {
    double finalMassKg { 0.0 };
    double finalPressurePa { 0.0 };
    double finalEnergyJ { 0.0 };
    double pistonWorkJ { 0.0 };
    double intakeTransferredKg { 0.0 };
    double absoluteValveTransferredKg { 0.0 };
    double maximumRelativeMassError { 0.0 };
    double maximumRelativeEnergyError { 0.0 };
    double maximumInventoryBias { 0.0 };
    std::size_t acceptedSteps { 0 };
    std::size_t rejectedSteps { 0 };
};

// The predictor and actual advance start from the exact same duct. Each
// duration uses one fixed FV grid for both finite and infinite reservoirs.
// The predictor currently holds the plenum thermodynamics even when its
// positive volume is supplied. The four-runner production prefix uses it
// with volume zero, whereas the ensuing real advance uses physical volume.
void finitePlenumPrediction(double durationSeconds) {
    constexpr double excessPressurePa = 5'000.0;
    const auto stepCount = static_cast<std::size_t>(std::llround(durationSeconds / mechanicalStepSeconds));
    ExhaustGasNetwork predictedNetwork;
    configure(predictedNetwork);
    const auto& model = predictedNetwork.mixtureModel();
    const auto plenum = model.conservativeFromPressureTemperature(pressurePa, temperatureK);
    const auto chargedRunner = model.conservativeFromPressureTemperature(
        pressurePa + excessPressurePa, temperatureK);
    require(plenum.has_value() && chargedRunner.has_value(), "the nonstationary mouth must recover");
    for (auto& cell : predictedNetwork.ducts().front().cells()) cell = *chargedRunner;
    const auto infinitePrediction = predictedNetwork.predictOutletTransfer(
        0, { *plenum, 1.0, 0.0 }, durationSeconds);
    const auto finitePrediction = predictedNetwork.predictOutletTransfer(
        0, { *plenum, 1.0, plenumVolumeM3 }, durationSeconds);
    require(infinitePrediction.has_value() && finitePrediction.has_value(),
        "both supplied-reservoir predictions must exist");
    const auto predictionIgnoresVolume = infinitePrediction->speciesMassKg == finitePrediction->speciesMassKg
        && infinitePrediction->transferredEnergyJ == finitePrediction->transferredEnergyJ;
    std::array<double, 2> actualMassKg {};
    std::array<double, 2> actualEnergyJ {};
    for (std::size_t caseIndex = 0; caseIndex < 2; ++caseIndex) {
        ExhaustGasNetwork actualNetwork;
        configure(actualNetwork);
        for (auto& cell : actualNetwork.ducts().front().cells()) cell = *chargedRunner;
        auto actualPlenum = *plenum;
        const auto suppliedVolume = caseIndex == 0 ? 0.0 : plenumVolumeM3;
        for (std::size_t step = 0; step < stepCount; ++step) {
            const auto advance = actualNetwork.advance(mechanicalStepSeconds, {},
                { actualPlenum, 1.0, suppliedVolume });
            require(advance.completed && advance.acceptedSubsteps == 1 && advance.rejectedSubsteps == 0,
                "the actual mouth must use the identical prescribed FV grid");
            const auto& mouth = actualNetwork.outletSamples().front();
            for (const auto mass : mouth.speciesMassKg) actualMassKg[caseIndex] += mass;
            actualEnergyJ[caseIndex] += mouth.transferredEnergyJ;
            if (suppliedVolume > 0.0) applyMouthExchange(actualPlenum, mouth);
        }
    }
    auto predictedMassKg = 0.0;
    for (const auto mass : finitePrediction->speciesMassKg) predictedMassKg += mass;
    std::cout << "prediction duration_us=" << durationSeconds * 1.0e6
        << " plenum_l=" << plenumVolumeM3 * 1.0e3
        << " initial_excess_pa=" << excessPressurePa
        << " predicted_mass_kg=" << predictedMassKg
        << " actual_infinite_mass_kg=" << actualMassKg[0]
        << " actual_finite_mass_kg=" << actualMassKg[1]
        << " predicted_energy_j=" << finitePrediction->transferredEnergyJ
        << " actual_infinite_energy_j=" << actualEnergyJ[0]
        << " actual_finite_energy_j=" << actualEnergyJ[1]
        << " relative_prediction_finite_error=" << (predictedMassKg - actualMassKg[1]) / actualMassKg[1]
        << " relative_finite_vs_infinite=" << (actualMassKg[1] - actualMassKg[0]) / actualMassKg[0]
        << " prediction_ignores_volume=" << predictionIgnoresVolume << '\n';
}

[[nodiscard]] MovingResult moving(double rpm, double couplingSeconds,
    double finiteVolumeStepSeconds, bool averageExtensiveInventory) {
    const auto macroSteps = static_cast<std::size_t>(std::llround(couplingSeconds / mechanicalStepSeconds));
    const auto fvSteps = static_cast<std::size_t>(std::llround(couplingSeconds / finiteVolumeStepSeconds));
    require(std::abs(static_cast<double>(macroSteps) * mechanicalStepSeconds - couplingSeconds) < 1.0e-15
        && std::abs(static_cast<double>(fvSteps) * finiteVolumeStepSeconds - couplingSeconds) < 1.0e-15,
        "both independent grids must partition each macro interval exactly");
    ExhaustGasNetwork network;
    configure(network);
    const auto& model = network.mixtureModel();
    const auto base = model.conservativeFromPressureTemperature(pressurePa, temperatureK);
    require(base.has_value(), "the reference state must recover");
    auto cylinder = *base;
    auto chamberVolume = volume(std::numbers::pi);
    auto plenum = *base;
    const auto initial = totalInventory(network, cylinder, chamberVolume, plenum);
    auto initialMass = 0.0;
    for (const auto species : initial.speciesMassKg) initialMass += species;
    const auto angularVelocity = rpm * 2.0 * std::numbers::pi / 60.0;
    // Start at BDC, finish after exactly ten crank revolutions. Five intakes
    // exercise the same closed system; no pressure/temperature reset is used.
    const auto requestedDuration = 10.0 * 2.0 * std::numbers::pi / angularVelocity;
    const auto macroCount = static_cast<std::size_t>(std::floor(requestedDuration / couplingSeconds));
    MovingResult result;
    for (std::size_t macro = 0; macro < macroCount; ++macro) {
        History history;
        for (std::size_t step = 0; step < macroSteps; ++step) {
            const auto time = static_cast<double>(macro) * couplingSeconds
                + static_cast<double>(step + 1) * mechanicalStepSeconds;
            const auto angle = std::numbers::pi + angularVelocity * time;
            result.pistonWorkJ += pistonAdvance(cylinder, chamberVolume, volume(angle), model);
            history.add(cylinder, chamberVolume, valveArea(angle));
        }
        const auto inverseCount = 1.0 / static_cast<double>(history.count);
        const auto reducedVolume = history.volumes * inverseCount;
        auto virtualCylinder = averageExtensiveInventory
            ? scaleState(history.inventories, inverseCount / reducedVolume)
            : scaleState(history.densities, inverseCount);
        const auto meanMass = history.inventories.densityKgPerM3() * inverseCount;
        result.maximumInventoryBias = std::max(result.maximumInventoryBias,
            std::abs(virtualCylinder.densityKgPerM3() * reducedVolume - meanMass) / meanMass);
        CylinderGasExchange macroExchange;
        for (std::size_t fv = 0; fv < fvSteps; ++fv) {
            const CylinderValveBoundary boundary { 1, virtualCylinder, reducedVolume,
                history.valveAreas * inverseCount, 1.0 };
            const auto advance = network.advance(finiteVolumeStepSeconds,
                std::span(&boundary, 1), { plenum, 1.0, plenumVolumeM3 });
            require(advance.completed && std::abs(advance.advancedTimeSeconds - finiteVolumeStepSeconds) < 1.0e-15,
                "every fixed FV microstep must complete");
            result.acceptedSteps += advance.acceptedSubsteps;
            result.rejectedSteps += advance.rejectedSubsteps;
            require(advance.acceptedSubsteps == 1 && advance.rejectedSubsteps == 0,
                "the prescribed FV grid must be the actual accepted grid");
            const auto& exchange = network.cylinderExchanges().front();
            applyCylinderExchange(virtualCylinder, reducedVolume, exchange);
            applyMouthExchange(plenum, network.outletSamples().front());
            for (std::size_t species = 0; species < gasSpeciesCount; ++species)
                macroExchange.speciesMassKg[species] += exchange.speciesMassKg[species];
            macroExchange.totalEnergyJ += exchange.totalEnergyJ;
            result.intakeTransferredKg -= exchange.totalMassKg();
            result.absoluteValveTransferredKg += std::abs(exchange.totalMassKg());
        }
        applyCylinderExchange(cylinder, chamberVolume, macroExchange);
        require(model.isPhysical(cylinder) && model.isPhysical(plenum),
            "both real reservoirs must accept the conservative exchange");
        const auto inventory = totalInventory(network, cylinder, chamberVolume, plenum);
        for (std::size_t species = 0; species < gasSpeciesCount; ++species)
            result.maximumRelativeMassError = std::max(result.maximumRelativeMassError,
                std::abs(inventory.speciesMassKg[species] - initial.speciesMassKg[species]) / initialMass);
        result.maximumRelativeEnergyError = std::max(result.maximumRelativeEnergyError,
            std::abs(inventory.totalEnergyJ - initial.totalEnergyJ - result.pistonWorkJ) / initial.totalEnergyJ);
    }
    const auto final = model.primitiveFromConservative(cylinder);
    require(final.has_value(), "the final chamber must recover");
    result.finalMassKg = cylinder.densityKgPerM3() * chamberVolume;
    result.finalPressurePa = final->pressurePa;
    result.finalEnergyJ = cylinder.totalEnergyDensityJPerM3 * chamberVolume;
    std::cout << "moving rpm=" << rpm << " coupling_us=" << couplingSeconds * 1.0e6
        << " fv_us=" << finiteVolumeStepSeconds * 1.0e6
        << " reduction=" << (averageExtensiveInventory ? "mean_inventory" : "mean_density")
        << " advanced_s=" << static_cast<double>(macroCount) * couplingSeconds
        << " final_cylinder_mass_kg=" << result.finalMassKg
        << " final_cylinder_pressure_pa=" << result.finalPressurePa
        << " final_cylinder_energy_j=" << result.finalEnergyJ
        << " piston_work_j=" << result.pistonWorkJ
        << " net_intake_kg=" << result.intakeTransferredKg
        << " absolute_valve_kg=" << result.absoluteValveTransferredKg
        << " maximum_relative_inventory_bias=" << result.maximumInventoryBias
        << " mass_relative_error=" << result.maximumRelativeMassError
        << " energy_plus_work_relative_error=" << result.maximumRelativeEnergyError
        << " accepted_steps=" << result.acceptedSteps << " rejected_steps=" << result.rejectedSteps << '\n';
    require(result.absoluteValveTransferredKg > 1.0e-5, "the moving fixture must exercise substantial real exchange");
    require(result.maximumRelativeMassError < 1.0e-10 && result.maximumRelativeEnergyError < 1.0e-10,
        "species and energy including exact piston work must balance");
    return result;
}

// Four 250 cc cylinders (one litre total) with 180-degree intake offsets
// compare the current independent-runner prefix against one existing
// multi-duct network sharing ONE finite plenum across its RK stages. Geometry,
// gas, piston trajectories, histories and the accepted 12.5 us FV grid match.
// A 400 us joint control keeps the lagged history, so topology is not confused
// with timely valve coupling. No CPU-capacity claim is made by this fixture.
void sharedMoving(double couplingSeconds, bool jointManifold, bool macroPrefix = false) {
    constexpr std::size_t cylinderCount = 4;
    constexpr double rpm = 5'000.0;
    const auto macroSteps = static_cast<std::size_t>(std::llround(couplingSeconds / mechanicalStepSeconds));
    ExhaustGasNetwork joint;
    std::array<ExhaustGasNetwork, cylinderCount> independent;
    configure(joint, cylinderCount);
    for (auto& network : independent) configure(network);
    const auto& model = joint.mixtureModel();
    const auto base = model.conservativeFromPressureTemperature(pressurePa, temperatureK);
    require(base.has_value(), "the four-cylinder reference state must recover");
    auto plenum = *base;
    std::array<ConservativeState, cylinderCount> cylinders;
    cylinders.fill(*base);
    std::array<double, cylinderCount> chamberVolumes;
    std::array<double, cylinderCount> phaseOffsets;
    for (std::size_t index = 0; index < cylinderCount; ++index) {
        phaseOffsets[index] = static_cast<double>(index) * std::numbers::pi;
        chamberVolumes[index] = volume(phaseOffsets[index]);
    }
    const auto inventory = [&]() {
        auto result = jointManifold ? joint.inventory() : ExhaustNetworkInventory {};
        if (!jointManifold) {
            for (const auto& network : independent) {
                const auto runner = network.inventory();
                result.totalEnergyJ += runner.totalEnergyJ;
                for (std::size_t species = 0; species < gasSpeciesCount; ++species)
                    result.speciesMassKg[species] += runner.speciesMassKg[species];
            }
        }
        for (std::size_t index = 0; index < cylinderCount; ++index) {
            result.totalEnergyJ += cylinders[index].totalEnergyDensityJPerM3 * chamberVolumes[index];
            for (std::size_t species = 0; species < gasSpeciesCount; ++species)
                result.speciesMassKg[species] += cylinders[index].speciesMassDensityKgPerM3[species]
                    * chamberVolumes[index];
        }
        result.totalEnergyJ += plenum.totalEnergyDensityJPerM3 * plenumVolumeM3;
        for (std::size_t species = 0; species < gasSpeciesCount; ++species)
            result.speciesMassKg[species] += plenum.speciesMassDensityKgPerM3[species] * plenumVolumeM3;
        return result;
    };
    const auto initial = inventory();
    auto initialMass = 0.0;
    for (const auto mass : initial.speciesMassKg) initialMass += mass;
    constexpr auto angularVelocity = rpm * 2.0 * std::numbers::pi / 60.0;
    constexpr auto durationSeconds = 0.12;
    const auto macroCount = static_cast<std::size_t>(std::llround(durationSeconds / couplingSeconds));
    auto pistonWork = 0.0;
    auto absoluteValveTransfer = 0.0;
    auto maximumRelativeMassError = 0.0;
    auto maximumRelativeEnergyError = 0.0;
    std::size_t acceptedSteps = 0;
    for (std::size_t macro = 0; macro < macroCount; ++macro) {
        std::array<History, cylinderCount> histories;
        for (std::size_t step = 0; step < macroSteps; ++step) {
            const auto time = static_cast<double>(macro) * couplingSeconds
                + static_cast<double>(step + 1) * mechanicalStepSeconds;
            for (std::size_t index = 0; index < cylinderCount; ++index) {
                const auto angle = phaseOffsets[index] + angularVelocity * time;
                pistonWork += pistonAdvance(cylinders[index], chamberVolumes[index], volume(angle), model);
                histories[index].add(cylinders[index], chamberVolumes[index], valveArea(angle));
            }
        }
        const auto inverseCount = 1.0 / static_cast<double>(macroSteps);
        std::array<CylinderValveBoundary, cylinderCount> boundaries;
        std::array<CylinderGasExchange, cylinderCount> macroExchanges;
        std::array<ConservativeState, cylinderCount> predictedPlenums;
        for (std::size_t index = 0; index < cylinderCount; ++index)
            boundaries[index] = { static_cast<std::uint32_t>(jointManifold ? index + 1 : 1),
                scaleState(histories[index].densities, inverseCount),
                histories[index].volumes * inverseCount,
                histories[index].valveAreas * inverseCount, 1.0 };
        for (std::size_t fv = 0; fv < macroSteps; ++fv) {
            std::array<CylinderGasExchange, cylinderCount> exchanges;
            std::array<ExhaustOutletFlowSample, cylinderCount> mouths;
            const auto requireGrid = [&](const ExhaustNetworkAdvanceResult& advance) {
                require(advance.completed && advance.acceptedSubsteps == 1 && advance.rejectedSubsteps == 0,
                    "every manifold case must accept the identical FV grid");
                acceptedSteps += advance.acceptedSubsteps;
            };
            if (jointManifold) {
                requireGrid(joint.advance(mechanicalStepSeconds, boundaries,
                    { plenum, 1.0, plenumVolumeM3 }));
                std::copy(joint.cylinderExchanges().begin(), joint.cylinderExchanges().end(), exchanges.begin());
                std::copy(joint.outletSamples().begin(), joint.outletSamples().end(), mouths.begin());
            } else {
                if (fv == 0 || !macroPrefix) {
                    predictedPlenums.fill(plenum);
                    for (std::size_t round = 0; round < 2; ++round) {
                        std::array<ExhaustOutletFlowSample, cylinderCount> predictions;
                        for (std::size_t index = 0; index < cylinderCount; ++index) {
                            const auto predicted = independent[index].predictOutletTransfer(0,
                                { predictedPlenums[index], 1.0 },
                                macroPrefix ? couplingSeconds : mechanicalStepSeconds);
                            require(predicted.has_value(), "the current two-round prefix must predict each mouth");
                            predictions[index] = *predicted;
                        }
                        auto scratchPlenum = plenum;
                        for (std::size_t index = 0; index < cylinderCount; ++index) {
                            predictedPlenums[index] = scratchPlenum;
                            applyMouthExchange(scratchPlenum, predictions[index]);
                            require(model.isPhysical(scratchPlenum), "the ordered predictor scratch must remain physical");
                        }
                    }
                }
                for (std::size_t index = 0; index < cylinderCount; ++index) {
                    requireGrid(independent[index].advance(mechanicalStepSeconds,
                        std::span(&boundaries[index], 1),
                        { predictedPlenums[index], 1.0, plenumVolumeM3 }));
                    exchanges[index] = independent[index].cylinderExchanges().front();
                    mouths[index] = independent[index].outletSamples().front();
                    if (macroPrefix) applyMouthExchange(predictedPlenums[index], mouths[index]);
                }
            }
            for (std::size_t index = 0; index < cylinderCount; ++index) {
                applyCylinderExchange(boundaries[index].cylinderState, boundaries[index].cylinderVolumeM3,
                    exchanges[index]);
                applyMouthExchange(plenum, mouths[index]);
                for (std::size_t species = 0; species < gasSpeciesCount; ++species)
                    macroExchanges[index].speciesMassKg[species] += exchanges[index].speciesMassKg[species];
                macroExchanges[index].totalEnergyJ += exchanges[index].totalEnergyJ;
                absoluteValveTransfer += std::abs(exchanges[index].totalMassKg());
            }
        }
        for (std::size_t index = 0; index < cylinderCount; ++index) {
            applyCylinderExchange(cylinders[index], chamberVolumes[index], macroExchanges[index]);
            require(model.isPhysical(cylinders[index]), "each real manifold cylinder must accept its exchange");
        }
        require(model.isPhysical(plenum), "the one real plenum must accept the sum once");
        const auto after = inventory();
        for (std::size_t species = 0; species < gasSpeciesCount; ++species)
            maximumRelativeMassError = std::max(maximumRelativeMassError,
                std::abs(after.speciesMassKg[species] - initial.speciesMassKg[species]) / initialMass);
        maximumRelativeEnergyError = std::max(maximumRelativeEnergyError,
            std::abs(after.totalEnergyJ - initial.totalEnergyJ - pistonWork) / initial.totalEnergyJ);
    }
    require(absoluteValveTransfer > 1.0e-4 && maximumRelativeMassError < 1.0e-10
        && maximumRelativeEnergyError < 1.0e-10, "the nonzero manifold exchanges must conserve species and energy plus work");
    auto cylinderMass = 0.0;
    auto cylinderEnergy = 0.0;
    for (std::size_t index = 0; index < cylinderCount; ++index) {
        cylinderMass += cylinders[index].densityKgPerM3() * chamberVolumes[index];
        cylinderEnergy += cylinders[index].totalEnergyDensityJPerM3 * chamberVolumes[index];
    }
    const auto finalPlenum = model.primitiveFromConservative(plenum);
    require(finalPlenum.has_value(), "the final shared plenum must recover");
    std::cout << "shared cylinders=" << cylinderCount << " displacement_cc=1000 rpm=" << rpm
        << " coupling_us=" << couplingSeconds * 1.0e6 << " fv_us=" << mechanicalStepSeconds * 1.0e6
        << " mode=" << (jointManifold ? "joint_finite_RK"
            : macroPrefix ? "independent_macro_prefix_two_round" : "independent_micro_prefix_two_round")
        << " prefix_refresh_us=" << (jointManifold ? 0.0
            : macroPrefix ? couplingSeconds * 1.0e6 : mechanicalStepSeconds * 1.0e6)
        << " final_cylinder_mass_kg=" << cylinderMass << " final_cylinder_energy_j=" << cylinderEnergy
        << " piston_work_j=" << pistonWork << " plenum_pressure_pa=" << finalPlenum->pressurePa
        << " plenum_temperature_k=" << finalPlenum->temperatureK
        << " absolute_valve_kg=" << absoluteValveTransfer
        << " mass_relative_error=" << maximumRelativeMassError
        << " energy_plus_work_relative_error=" << maximumRelativeEnergyError
        << " accepted_network_steps=" << acceptedSteps << " accepted_duct_steps=" << cylinderCount * 9'600 << '\n';
}
} // namespace

int main() {
    std::cout << std::setprecision(15);
    for (const auto rpm : { 3'600.0, 4'200.0, 5'000.0 }) {
        for (const auto coupling : { 400.0e-6, 125.0e-6, 50.0e-6, mechanicalStepSeconds })
            frozenHistory(rpm, coupling);
    }
    for (const auto coupling : { 400.0e-6, 125.0e-6, 50.0e-6, mechanicalStepSeconds })
        static_cast<void>(moving(5'000.0, coupling, mechanicalStepSeconds, false));
    static_cast<void>(moving(5'000.0, 400.0e-6, mechanicalStepSeconds, true));
    for (const auto fvStep : { 6.25e-6, 3.125e-6 })
        static_cast<void>(moving(5'000.0, 400.0e-6, fvStep, false));
    for (const auto duration : { mechanicalStepSeconds, 50.0e-6, 125.0e-6, 200.0e-6, 400.0e-6 })
        finitePlenumPrediction(duration);
    for (const auto coupling : { 400.0e-6, mechanicalStepSeconds }) {
        sharedMoving(coupling, false);
        sharedMoving(coupling, true);
    }
    sharedMoving(400.0e-6, false, true);
    return EXIT_SUCCESS;
}
