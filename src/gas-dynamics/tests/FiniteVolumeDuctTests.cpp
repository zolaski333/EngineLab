#include <enginelab/gasdynamics/FiniteVolumeDuct.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <numbers>
#include <string_view>

namespace {

using enginelab::gasdynamics::ConservedInventory;
using enginelab::gasdynamics::ConservativeState;
using enginelab::gasdynamics::DuctBoundaryCondition;
using enginelab::gasdynamics::DuctGeometry;
using enginelab::gasdynamics::EulerMixtureModel;
using enginelab::gasdynamics::FiniteVolumeDuct;
using enginelab::gasdynamics::GasComposition;
using enginelab::gasdynamics::gasSpeciesCount;

void require(bool condition, std::string_view message) {
    if (condition) return;
    std::cerr << "FAILED: " << message << '\n';
    std::exit(EXIT_FAILURE);
}

[[nodiscard]] double relativeError(double actual, double expected) {
    return std::abs(actual - expected) / std::max(1.0e-15, std::abs(expected));
}

[[nodiscard]] ConservativeState makeState(const EulerMixtureModel& model,
                                          double densityKgPerM3,
                                          double velocityMps,
                                          double pressurePa,
                                          GasComposition composition = GasComposition::inertGas()) {
    const auto state = model.conservativeFromPrimitive(
        densityKgPerM3, velocityMps, pressurePa, composition);
    require(state.has_value(), "test fixture must create a physical conservative state");
    return *state;
}

[[nodiscard]] DuctGeometry losslessGeometry(double lengthM, std::size_t cells) {
    DuctGeometry geometry;
    geometry.lengthM = lengthM;
    geometry.diameterM = 0.08;
    geometry.cellCount = cells;
    geometry.wallFrictionEnabled = false;
    geometry.absoluteRoughnessM = 0.0;
    geometry.localLossCoefficient = 0.0;
    geometry.wallHeatTransferWPerM2K = 0.0;
    return geometry;
}

void requireInventoryNear(const ConservedInventory& actual,
                          const ConservedInventory& expected,
                          double tolerance,
                          std::string_view context) {
    for (std::size_t index = 0; index < gasSpeciesCount; ++index) {
        if (relativeError(actual.speciesMassKg[index], expected.speciesMassKg[index]) > tolerance) {
            std::cerr << "FAILED: " << context << " species " << index
                      << " expected " << expected.speciesMassKg[index]
                      << " got " << actual.speciesMassKg[index] << '\n';
            std::exit(EXIT_FAILURE);
        }
    }
    require(relativeError(actual.totalEnergyJ, expected.totalEnergyJ) <= tolerance,
            "total energy inventory must be conserved");
}

[[nodiscard]] std::size_t closestCell(const FiniteVolumeDuct& duct, double positionM) {
    const auto scaled = positionM / duct.geometry().cellLengthM() - 0.5;
    const auto rounded = static_cast<long long>(std::llround(scaled));
    const auto bounded = std::clamp<long long>(
        rounded, 0, static_cast<long long>(duct.cells().size() - 1));
    return static_cast<std::size_t>(bounded);
}

void testEquationOfStateRoundTrip() {
    EulerMixtureModel model;
    GasComposition composition;
    composition.massFractions = { 0.12, 0.43, 0.05, 0.40 };
    const auto state = makeState(model, 1.37, -83.0, 237'000.0, composition);
    const auto primitive = model.primitiveFromConservative(state);
    require(primitive.has_value(), "round-trip state must remain physical");
    require(relativeError(primitive->densityKgPerM3, 1.37) < 1.0e-13,
            "density must round-trip through conserved species masses");
    require(relativeError(primitive->velocityMps, -83.0) < 1.0e-13,
            "velocity must round-trip through momentum density");
    require(relativeError(primitive->pressurePa, 237'000.0) < 1.0e-13,
            "pressure must round-trip through total energy");
    require(primitive->heatCapacityRatio > 1.05 && primitive->heatCapacityRatio < 1.4,
            "burned gas and fuel vapour must lower mixture gamma physically");

    auto invalidComposition = GasComposition::dryAir();
    invalidComposition.massFractions[0] = -0.1;
    require(!model.conservativeFromPrimitive(1.0, 0.0, 101'325.0, invalidComposition),
            "negative species fractions must be rejected, not clamped");

    auto roundoffState = state;
    const auto densityBeforeRepair = roundoffState.densityKgPerM3();
    roundoffState.speciesMassDensityKgPerM3[2] = -1.0e-16;
    const auto targetDensity = roundoffState.densityKgPerM3();
    require(model.canonicaliseSpeciesRoundoff(roundoffState)
            && roundoffState.speciesMassDensityKgPerM3[2] == 0.0
            && relativeError(roundoffState.densityKgPerM3(), targetDensity) < 2.0e-16,
        "roundoff repair must remove ulp-scale negativity without changing total density");
    roundoffState = state;
    roundoffState.speciesMassDensityKgPerM3[2] = -1.0e-6;
    require(!model.canonicaliseSpeciesRoundoff(roundoffState)
            && roundoffState.speciesMassDensityKgPerM3[2] == -1.0e-6
            && densityBeforeRepair > 0.0,
        "roundoff repair must reject physically significant species negativity");
}

void testUniformStatePreservation() {
    EulerMixtureModel model;
    GasComposition composition;
    composition.massFractions = { 0.15, 0.50, 0.0, 0.35 };
    const auto uniform = makeState(model, 1.1, 47.0, 160'000.0, composition);
    FiniteVolumeDuct duct;
    require(duct.configure(losslessGeometry(1.3, 96), uniform),
            "uniform-state duct must configure");
    const auto before = duct.inventory();
    const auto result = duct.advance(0.002, DuctBoundaryCondition::periodic(),
                                     DuctBoundaryCondition::periodic());
    require(result.completed && result.rejectedSubsteps == 0,
            "uniform state must advance without a positivity retry");
    for (const auto& cell : duct.cells()) {
        for (std::size_t index = 0; index < gasSpeciesCount; ++index)
            require(relativeError(cell.speciesMassDensityKgPerM3[index],
                                  uniform.speciesMassDensityKgPerM3[index]) < 2.0e-13,
                    "a uniform species field must remain uniform");
        require(relativeError(cell.momentumDensityKgPerM2S,
                              uniform.momentumDensityKgPerM2S) < 2.0e-13,
                "uniform momentum must remain uniform");
        require(relativeError(cell.totalEnergyDensityJPerM3,
                              uniform.totalEnergyDensityJPerM3) < 2.0e-13,
                "uniform total energy must remain uniform");
    }
    requireInventoryNear(duct.inventory(), before, 2.0e-13,
                         "uniform periodic inventory");
}

void testSingleControlVolumePreservesUniformConservation() {
    EulerMixtureModel model;
    GasComposition composition;
    composition.massFractions = { 0.08, 0.57, 0.0, 0.35 };
    const auto uniform = makeState(model, 0.74, 31.0, 142'000.0, composition);
    FiniteVolumeDuct duct;
    require(duct.configure(losslessGeometry(0.12, 1), uniform),
        "a sub-wavelength component must support one finite control volume");
    const auto before = duct.inventory();
    const auto result = duct.advance(0.003,
        DuctBoundaryCondition::periodic(), DuctBoundaryCondition::periodic());
    require(result.completed && result.rejectedSubsteps == 0,
        "one-cell periodic control volume must advance without a positivity retry");
    requireInventoryNear(duct.inventory(), before, 2.0e-13,
        "one-cell periodic inventory");
    const auto& after = duct.cells().front();
    require(relativeError(after.momentumDensityKgPerM2S,
                          uniform.momentumDensityKgPerM2S) < 2.0e-13
            && relativeError(after.totalEnergyDensityJPerM3,
                             uniform.totalEnergyDensityJPerM3) < 2.0e-13,
        "one finite control volume must preserve uniform momentum and energy");
}

void testMutableInitialConditionRefreshesDerivedState() {
    EulerMixtureModel model;
    const auto cold = makeState(model, 1.20, 0.0, 101'325.0, GasComposition::dryAir());
    const auto hot = makeState(model, 0.30, 0.0, 101'325.0, GasComposition::dryAir());
    FiniteVolumeDuct duct;
    require(duct.configure(losslessGeometry(0.8, 40), cold),
            "cache-refresh duct must configure");
    const auto coldStableStep = duct.maximumStableTimeStep(0.5);
    for (auto& cell : duct.cells()) cell = hot;
    const auto hotStableStep = duct.maximumStableTimeStep(0.5);
    require(coldStableStep > 0.0 && hotStableStep > 0.0
            && hotStableStep < coldStableStep * 0.55,
            "mutable initial conditions must invalidate cached sound speed and CFL data");
    const auto result = duct.advance(hotStableStep,
        DuctBoundaryCondition::periodic(), DuctBoundaryCondition::periodic(), 0.5);
    require(result.completed && result.rejectedSubsteps == 0,
            "refreshed state cache must drive the following residual consistently");
}

void testPeriodicConservation() {
    EulerMixtureModel model;
    const auto geometry = losslessGeometry(1.0, 192);
    FiniteVolumeDuct duct;
    require(duct.configure(geometry, makeState(model, 1.0, 0.0, 100'000.0)),
            "periodic conservation duct must configure");
    for (std::size_t index = 0; index < duct.cells().size(); ++index) {
        const auto x = duct.cellCentreM(index);
        const auto phase = 2.0 * std::numbers::pi * x / geometry.lengthM;
        GasComposition composition;
        const auto burnedFraction = 0.15 + 0.08 * std::sin(phase - 0.4);
        composition.massFractions = {
            0.18,
            0.82 - burnedFraction,
            0.0,
            burnedFraction,
        };
        duct.cells()[index] = makeState(model,
            1.0 + 0.12 * std::sin(phase),
            32.0 * std::cos(phase + 0.2),
            110'000.0 + 9'000.0 * std::sin(phase - 0.7),
            composition);
    }
    const auto before = duct.inventory();
    const auto result = duct.advance(0.0011, DuctBoundaryCondition::periodic(),
                                     DuctBoundaryCondition::periodic());
    require(result.completed, "smooth periodic field must complete its requested time");
    requireInventoryNear(duct.inventory(), before, 2.0e-11,
                         "non-uniform periodic inventory");
    for (std::size_t index = 0; index < gasSpeciesCount; ++index)
        require(relativeError(result.leftBoundaryFlux.speciesMassKgPerM2[index],
                              result.rightBoundaryFlux.speciesMassKgPerM2[index]) < 1.0e-13,
                "periodic face flux must be identical at both mesh ends");
}

void testVariableAreaQuasiOneDimensionalConservation() {
    EulerMixtureModel model;
    auto geometry = losslessGeometry(0.9, 180);
    geometry.crossSectionAreaM2 = 0.0;
    geometry.inletCrossSectionAreaM2 = 0.0080;
    geometry.outletCrossSectionAreaM2 = 0.0020;
    const auto uniform = makeState(
        model, 1.12, 0.0, 128'000.0, GasComposition::dryAir());
    FiniteVolumeDuct duct;
    const auto exactFrustumMeanAreaM2 =
        (0.0080 + std::sqrt(0.0080 * 0.0020) + 0.0020) / 3.0;
    require(geometry.valid() && geometry.hasVariableArea()
            && relativeError(geometry.areaM2(), exactFrustumMeanAreaM2)
                < 1.0e-14,
        "a conical taper must expose valid end areas and exact mean area");
    require(duct.configure(geometry, uniform),
        "variable-area quasi-1D duct must configure");
    const auto expectedVolumeM3 =
        exactFrustumMeanAreaM2 * geometry.lengthM;
    const auto initialInventory = duct.inventory();
    require(relativeError(initialInventory.totalEnergyJ,
                uniform.totalEnergyDensityJPerM3 * expectedVolumeM3)
            < 2.0e-14,
        "taper inventory must use each control volume's physical area");

    const auto equilibrium = duct.advance(
        0.003, DuctBoundaryCondition::transmissive(),
        DuctBoundaryCondition::transmissive());
    require(equilibrium.completed && equilibrium.rejectedSubsteps == 0,
        "stationary gas in a taper must remain a stable equilibrium");
    for (const auto& cell : duct.cells()) {
        require(relativeError(cell.densityKgPerM3(),
                    uniform.densityKgPerM3()) < 3.0e-13
                && std::abs(cell.momentumDensityKgPerM2S) < 2.0e-10
                && relativeError(cell.totalEnergyDensityJPerM3,
                    uniform.totalEnergyDensityJPerM3) < 3.0e-13,
            "p*dA/dx must exactly balance pressure-flux divergence at rest");
    }

    const auto moving = makeState(
        model, 1.05, 42.0, 118'000.0, GasComposition::dryAir());
    require(duct.configure(geometry, moving),
        "moving taper conservation fixture must configure");
    const auto before = duct.inventory();
    const auto result = duct.advance(
        0.00035, DuctBoundaryCondition::transmissive(),
        DuctBoundaryCondition::transmissive());
    const auto after = duct.inventory();
    require(result.completed,
        "a moving flow must remain physical through a four-to-one contraction");
    for (std::size_t index = 0; index < gasSpeciesCount; ++index) {
        const auto expectedChange =
            geometry.inletAreaM2()
                * result.leftBoundaryFlux.speciesMassKgPerM2[index]
            - geometry.outletAreaM2()
                * result.rightBoundaryFlux.speciesMassKgPerM2[index];
        require(std::abs((after.speciesMassKg[index]
                    - before.speciesMassKg[index]) - expectedChange)
                < std::max(1.0e-13,
                    std::abs(before.speciesMassKg[index]) * 3.0e-10),
            "taper mass inventory must close against its unequal face areas");
    }
    const auto expectedEnergyChange =
        geometry.inletAreaM2()
            * result.leftBoundaryFlux.totalEnergyJPerM2
        - geometry.outletAreaM2()
            * result.rightBoundaryFlux.totalEnergyJPerM2;
    require(std::abs((after.totalEnergyJ - before.totalEnergyJ)
                - expectedEnergyChange)
            < std::max(1.0e-9, std::abs(before.totalEnergyJ) * 3.0e-10),
        "taper energy inventory must close against its unequal face areas");

    const auto rejectedPeriodic = duct.advance(
        1.0e-5, DuctBoundaryCondition::periodic(),
        DuctBoundaryCondition::periodic());
    require(!rejectedPeriodic.completed,
        "a non-periodic taper must reject a mathematically inconsistent periodic seam");
}

void testSodShockTube() {
    EulerMixtureModel model;
    constexpr double referencePressurePa = 100'000.0;
    constexpr double referenceDensity = 1.0;
    const auto referenceVelocity = std::sqrt(referencePressurePa / referenceDensity);
    const auto leftState = makeState(model, 1.0, 0.0, referencePressurePa);
    const auto rightState = makeState(model, 0.125, 0.0, 0.1 * referencePressurePa);
    FiniteVolumeDuct duct;
    require(duct.configure(losslessGeometry(1.0, 600), leftState),
            "Sod shock tube must configure");
    for (std::size_t index = 0; index < duct.cells().size(); ++index)
        if (duct.cellCentreM(index) >= 0.5) duct.cells()[index] = rightState;

    const auto before = duct.inventory();
    const auto result = duct.advance(0.2 / referenceVelocity,
        DuctBoundaryCondition::transmissive(), DuctBoundaryCondition::transmissive(), 0.42);
    require(result.completed, "Sod shock tube must reach the analytic comparison time");
    require(result.rejectedSubsteps < 4,
            "Sod shock must remain positive without repeated timestep collapse");

    const auto leftStar = model.primitiveFromConservative(duct.cells()[closestCell(duct, 0.58)]);
    const auto rightStar = model.primitiveFromConservative(duct.cells()[closestCell(duct, 0.75)]);
    const auto undisturbedRight = model.primitiveFromConservative(
        duct.cells()[closestCell(duct, 0.93)]);
    require(leftStar && rightStar && undisturbedRight,
            "all Sod reference regions must remain physical");
    require(std::abs(leftStar->densityKgPerM3 - 0.4263) < 0.025,
            "Sod left-star density must match the exact Riemann solution");
    require(std::abs(leftStar->pressurePa / referencePressurePa - 0.3031) < 0.025,
            "Sod star pressure must match the exact Riemann solution");
    require(std::abs(leftStar->velocityMps / referenceVelocity - 0.9275) < 0.04,
            "Sod contact velocity must match the exact Riemann solution");
    require(std::abs(rightStar->densityKgPerM3 - 0.2656) < 0.025,
            "Sod right-star density must capture the contact discontinuity");
    require(std::abs(undisturbedRight->densityKgPerM3 - 0.125) < 0.005,
            "Sod state ahead of the shock must remain undisturbed");

    const auto after = duct.inventory();
    const auto area = duct.geometry().areaM2();
    for (std::size_t index = 0; index < gasSpeciesCount; ++index) {
        const auto expectedChange = area
            * (result.leftBoundaryFlux.speciesMassKgPerM2[index]
               - result.rightBoundaryFlux.speciesMassKgPerM2[index]);
        require(std::abs((after.speciesMassKg[index] - before.speciesMassKg[index])
                         - expectedChange)
                    < std::max(1.0e-13, std::abs(before.speciesMassKg[index]) * 2.0e-10),
                "open-boundary species inventory must close against integrated flux");
    }
    const auto expectedEnergyChange = area
        * (result.leftBoundaryFlux.totalEnergyJPerM2
           - result.rightBoundaryFlux.totalEnergyJPerM2);
    require(std::abs((after.totalEnergyJ - before.totalEnergyJ) - expectedEnergyChange)
                < std::max(1.0e-9, std::abs(before.totalEnergyJ) * 2.0e-10),
            "open-boundary energy inventory must close against integrated flux");
}

void testAcousticTransitSpeed() {
    EulerMixtureModel model;
    constexpr double baseDensity = 1.20;
    constexpr double basePressurePa = 101'325.0;
    constexpr double pulsePressurePa = 120.0;
    constexpr double startPositionM = 0.35;
    constexpr double pulseWidthM = 0.014;
    const auto base = makeState(model, baseDensity, 0.0, basePressurePa, GasComposition::dryAir());
    const auto basePrimitive = model.primitiveFromConservative(base);
    require(basePrimitive.has_value(), "base acoustic state must be physical");

    FiniteVolumeDuct duct;
    require(duct.configure(losslessGeometry(1.5, 600), base),
            "acoustic transit duct must configure");
    for (std::size_t index = 0; index < duct.cells().size(); ++index) {
        const auto offset = (duct.cellCentreM(index) - startPositionM) / pulseWidthM;
        const auto perturbation = pulsePressurePa * std::exp(-0.5 * offset * offset);
        duct.cells()[index] = makeState(model, baseDensity, 0.0,
            basePressurePa + perturbation, GasComposition::dryAir());
    }

    constexpr double travelTimeSeconds = 0.00155;
    const auto result = duct.advance(travelTimeSeconds,
        DuctBoundaryCondition::transmissive(), DuctBoundaryCondition::transmissive());
    require(result.completed, "small-amplitude acoustic pulse must complete");
    auto peakPressure = -std::numeric_limits<double>::infinity();
    auto peakPosition = 0.0;
    for (std::size_t index = 0; index < duct.cells().size(); ++index) {
        if (duct.cellCentreM(index) < 0.55) continue;
        const auto primitive = model.primitiveFromConservative(duct.cells()[index]);
        require(primitive.has_value(), "acoustic pulse cells must remain physical");
        if (primitive->pressurePa > peakPressure) {
            peakPressure = primitive->pressurePa;
            peakPosition = duct.cellCentreM(index);
        }
    }
    const auto measuredSpeed = (peakPosition - startPositionM) / travelTimeSeconds;
    require(relativeError(measuredSpeed, basePrimitive->speedOfSoundMps) < 0.035,
            "small pressure waves must travel at the thermodynamic speed of sound");
}

void testRigidEndReflection() {
    EulerMixtureModel model;
    constexpr double baseDensity = 1.20;
    constexpr double basePressurePa = 101'325.0;
    constexpr double pulsePressurePa = 100.0;
    constexpr double startPositionM = 0.30;
    constexpr double expectedReflectedPositionM = 0.80;
    constexpr double pulseWidthM = 0.016;
    const auto base = makeState(model, baseDensity, 0.0, basePressurePa, GasComposition::dryAir());
    const auto primitive = model.primitiveFromConservative(base);
    require(primitive.has_value(), "reflection base state must be physical");

    FiniteVolumeDuct duct;
    require(duct.configure(losslessGeometry(1.2, 600), base),
            "reflection duct must configure");
    for (std::size_t index = 0; index < duct.cells().size(); ++index) {
        const auto offset = (duct.cellCentreM(index) - startPositionM) / pulseWidthM;
        const auto pressurePerturbation = pulsePressurePa * std::exp(-0.5 * offset * offset);
        const auto density = baseDensity
            + pressurePerturbation
                / (primitive->speedOfSoundMps * primitive->speedOfSoundMps);
        const auto velocity = pressurePerturbation
            / (baseDensity * primitive->speedOfSoundMps);
        duct.cells()[index] = makeState(model, density, velocity,
            basePressurePa + pressurePerturbation, GasComposition::dryAir());
    }
    const auto pathLength = (duct.geometry().lengthM - startPositionM)
        + (duct.geometry().lengthM - expectedReflectedPositionM);
    const auto result = duct.advance(pathLength / primitive->speedOfSoundMps,
        DuctBoundaryCondition::transmissive(), DuctBoundaryCondition::reflective());
    require(result.completed, "rigid-end reflection must complete");

    auto peakPressure = -std::numeric_limits<double>::infinity();
    auto peakPosition = 0.0;
    auto peakVelocity = 0.0;
    for (std::size_t index = 0; index < duct.cells().size(); ++index) {
        if (duct.cellCentreM(index) < 0.55) continue;
        const auto state = model.primitiveFromConservative(duct.cells()[index]);
        require(state.has_value(), "reflected acoustic pulse must remain physical");
        if (state->pressurePa > peakPressure) {
            peakPressure = state->pressurePa;
            peakPosition = duct.cellCentreM(index);
            peakVelocity = state->velocityMps;
        }
    }
    require(std::abs(peakPosition - expectedReflectedPositionM) < 0.035,
            "a rigid end must return the pressure pulse at the characteristic travel time");
    require(peakVelocity < 0.0,
            "a rigid end must invert particle velocity while preserving pressure polarity");
}

void testFrictionConvertsResolvedMotionToHeat() {
    EulerMixtureModel model;
    auto geometry = losslessGeometry(0.8, 64);
    geometry.wallFrictionEnabled = true;
    geometry.absoluteRoughnessM = 4.5e-5;
    geometry.localLossCoefficient = 0.8;
    FiniteVolumeDuct duct;
    const auto initial = makeState(model, 0.9, 120.0, 140'000.0, GasComposition::dryAir());
    require(duct.configure(geometry, initial), "friction duct must configure");
    const auto beforeInventory = duct.inventory();
    const auto beforePrimitive = model.primitiveFromConservative(duct.cells().front());
    const auto result = duct.advance(0.004, DuctBoundaryCondition::periodic(),
                                     DuctBoundaryCondition::periodic());
    const auto afterPrimitive = model.primitiveFromConservative(duct.cells().front());
    require(result.completed && beforePrimitive && afterPrimitive,
            "friction case must remain physical");
    require(afterPrimitive->velocityMps < beforePrimitive->velocityMps,
            "Darcy and local losses must reduce resolved bulk velocity");
    require(afterPrimitive->temperatureK > beforePrimitive->temperatureK,
            "adiabatic wall friction must return lost kinetic energy as gas heat");
    require(relativeError(duct.inventory().totalEnergyJ, beforeInventory.totalEnergyJ) < 2.0e-11,
            "adiabatic friction must conserve total gas energy");
}

void testDynamicWallConservesGasAndMetalEnergy() {
    EulerMixtureModel model;
    auto geometry = losslessGeometry(0.8, 32);
    geometry.dynamicWallHeatTransferEnabled = true;
    geometry.wallTemperatureK = 330.0;
    geometry.wallThicknessM = 0.0015;
    geometry.wallDensityKgPerM3 = 7'900.0;
    geometry.wallSpecificHeatJPerKgK = 500.0;
    geometry.externalWallHeatTransferWPerM2K = 0.0;
    geometry.externalTemperatureK = 300.0;
    FiniteVolumeDuct duct;
    const auto initial = makeState(
        model, 0.55, 65.0, 180'000.0, GasComposition::dryAir());
    require(duct.configure(geometry, initial),
            "finite-capacity wall duct must configure");
    const auto beforeGasEnergyJ = duct.inventory().totalEnergyJ;
    const auto beforeWallEnergyJ = duct.wallThermalEnergyJ();
    const auto beforeTemperature =
        model.primitiveFromConservative(duct.cells().front());

    const auto result = duct.advance(0.010,
        DuctBoundaryCondition::periodic(), DuctBoundaryCondition::periodic());
    const auto afterGasEnergyJ = duct.inventory().totalEnergyJ;
    const auto afterWallEnergyJ = duct.wallThermalEnergyJ();
    const auto afterTemperature =
        model.primitiveFromConservative(duct.cells().front());
    const auto gasEnergyLostJ = beforeGasEnergyJ - afterGasEnergyJ;
    const auto wallEnergyGainedJ = afterWallEnergyJ - beforeWallEnergyJ;
    require(result.completed && beforeTemperature && afterTemperature,
            "finite-capacity wall exchange must remain physical");
    require(afterTemperature->temperatureK < beforeTemperature->temperatureK
            && duct.wallStates().front().temperatureK > geometry.wallTemperatureK,
            "hot gas must warm the resolved duct wall instead of losing hidden energy");
    require(gasEnergyLostJ > 0.0
            && std::abs(gasEnergyLostJ - wallEnergyGainedJ)
                < std::max(1.0, gasEnergyLostJ) * 1.0e-9,
            "adiabatic gas-wall exchange must conserve the combined energy inventory");
    require(std::abs(result.wallHeatRejectedJ) < 1.0e-12,
            "an insulated dynamic wall must not reject energy to an implicit reservoir");

    auto cooledGeometry = geometry;
    cooledGeometry.wallTemperatureK = 650.0;
    cooledGeometry.externalWallHeatTransferWPerM2K = 22.0;
    FiniteVolumeDuct cooledDuct;
    require(cooledDuct.configure(cooledGeometry, initial),
            "externally cooled finite-capacity wall must configure");
    const auto combinedEnergyBeforeJ = cooledDuct.inventory().totalEnergyJ
        + cooledDuct.wallThermalEnergyJ();
    const auto cooledResult = cooledDuct.advance(0.010,
        DuctBoundaryCondition::periodic(), DuctBoundaryCondition::periodic());
    const auto combinedEnergyAfterJ = cooledDuct.inventory().totalEnergyJ
        + cooledDuct.wallThermalEnergyJ();
    require(cooledResult.completed && cooledResult.wallHeatRejectedJ > 0.0,
            "a hot duct wall must reject explicit heat to cooler surroundings");
    require(std::abs(combinedEnergyBeforeJ - combinedEnergyAfterJ
                - cooledResult.wallHeatRejectedJ)
            < std::max(1.0, cooledResult.wallHeatRejectedJ) * 1.0e-8,
            "gas plus wall energy loss must equal reported external heat rejection");
}

void testStrongExpansionRemainsPositive() {
    EulerMixtureModel model;
    const auto high = makeState(model, 4.0, 0.0, 1'200'000.0);
    const auto low = makeState(model, 0.08, 0.0, 1'200.0);
    FiniteVolumeDuct duct;
    require(duct.configure(losslessGeometry(1.0, 360), high),
            "strong-expansion duct must configure");
    for (std::size_t index = 0; index < duct.cells().size(); ++index)
        if (duct.cellCentreM(index) >= 0.5) duct.cells()[index] = low;
    const auto result = duct.advance(0.00012,
        DuctBoundaryCondition::transmissive(), DuctBoundaryCondition::transmissive(), 0.35);
    require(result.completed, "strong pressure ratio must not collapse the timestep");
    for (const auto& state : duct.cells())
        require(model.isPhysical(state),
                "strong expansion must preserve positive density and internal energy");
}

} // namespace

void runExhaustNetworkLayoutTests();
void runExhaustGasNetworkTests();
void runIntakeNetworkTests();

int main() {
    testEquationOfStateRoundTrip();
    testUniformStatePreservation();
    testSingleControlVolumePreservesUniformConservation();
    testMutableInitialConditionRefreshesDerivedState();
    testPeriodicConservation();
    testVariableAreaQuasiOneDimensionalConservation();
    testSodShockTube();
    testAcousticTransitSpeed();
    testRigidEndReflection();
    testFrictionConvertsResolvedMotionToHeat();
    testDynamicWallConservesGasAndMetalEnergy();
    testStrongExpansionRemainsPositive();
    runExhaustNetworkLayoutTests();
    runExhaustGasNetworkTests();
    runIntakeNetworkTests();
    std::cout << "EngineLab gas-dynamics tests passed\n";
    return EXIT_SUCCESS;
}
