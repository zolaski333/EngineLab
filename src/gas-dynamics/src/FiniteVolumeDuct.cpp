#include <enginelab/gasdynamics/FiniteVolumeDuct.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace enginelab::gasdynamics {
namespace {

constexpr double minimumDensityKgPerM3 = 1.0e-12;
constexpr double minimumInternalEnergyDensityJPerM3 = 1.0e-9;
constexpr double reconstructionBias = 1.5;

[[nodiscard]] constexpr std::size_t speciesIndex(GasSpecies species) noexcept {
    return static_cast<std::size_t>(species);
}

[[nodiscard]] ConservativeState addScaled(const ConservativeState& first,
                                           const ConservativeState& second,
                                           double scale) noexcept {
    auto result = first;
    for (std::size_t index = 0; index < gasSpeciesCount; ++index)
        result.speciesMassDensityKgPerM3[index] +=
            second.speciesMassDensityKgPerM3[index] * scale;
    result.momentumDensityKgPerM2S += second.momentumDensityKgPerM2S * scale;
    result.totalEnergyDensityJPerM3 += second.totalEnergyDensityJPerM3 * scale;
    return result;
}

[[nodiscard]] ConservativeState difference(const ConservativeState& first,
                                            const ConservativeState& second) noexcept {
    ConservativeState result;
    for (std::size_t index = 0; index < gasSpeciesCount; ++index)
        result.speciesMassDensityKgPerM3[index] =
            first.speciesMassDensityKgPerM3[index]
            - second.speciesMassDensityKgPerM3[index];
    result.momentumDensityKgPerM2S =
        first.momentumDensityKgPerM2S - second.momentumDensityKgPerM2S;
    result.totalEnergyDensityJPerM3 =
        first.totalEnergyDensityJPerM3 - second.totalEnergyDensityJPerM3;
    return result;
}

[[nodiscard]] double minmod(double first, double second, double third) noexcept {
    if (first > 0.0 && second > 0.0 && third > 0.0)
        return std::min({ first, second, third });
    if (first < 0.0 && second < 0.0 && third < 0.0)
        return std::max({ first, second, third });
    return 0.0;
}

[[nodiscard]] ConservativeState monotonisedCentralSlope(
    const ConservativeState& previous,
    const ConservativeState& current,
    const ConservativeState& next) noexcept {
    const auto backward = difference(current, previous);
    const auto centred = difference(next, previous);
    const auto forward = difference(next, current);
    ConservativeState result;
    for (std::size_t index = 0; index < gasSpeciesCount; ++index) {
        result.speciesMassDensityKgPerM3[index] = minmod(
            reconstructionBias * backward.speciesMassDensityKgPerM3[index],
            0.5 * centred.speciesMassDensityKgPerM3[index],
            reconstructionBias * forward.speciesMassDensityKgPerM3[index]);
    }
    result.momentumDensityKgPerM2S = minmod(
        reconstructionBias * backward.momentumDensityKgPerM2S,
        0.5 * centred.momentumDensityKgPerM2S,
        reconstructionBias * forward.momentumDensityKgPerM2S);
    result.totalEnergyDensityJPerM3 = minmod(
        reconstructionBias * backward.totalEnergyDensityJPerM3,
        0.5 * centred.totalEnergyDensityJPerM3,
        reconstructionBias * forward.totalEnergyDensityJPerM3);
    return result;
}

[[nodiscard]] ConservativeState mirrored(const ConservativeState& state) noexcept {
    auto result = state;
    result.momentumDensityKgPerM2S = -result.momentumDensityKgPerM2S;
    return result;
}

[[nodiscard]] EulerFlux fluxDifferenceStateScaled(
    const EulerFlux& flux,
    const ConservativeState& first,
    const ConservativeState& second,
    double waveSpeed) noexcept {
    auto result = flux;
    for (std::size_t index = 0; index < gasSpeciesCount; ++index)
        result.speciesMassFluxKgPerM2S[index] += waveSpeed
            * (first.speciesMassDensityKgPerM3[index]
               - second.speciesMassDensityKgPerM3[index]);
    result.momentumFluxPa += waveSpeed
        * (first.momentumDensityKgPerM2S - second.momentumDensityKgPerM2S);
    result.totalEnergyFluxWPerM2 += waveSpeed
        * (first.totalEnergyDensityJPerM3 - second.totalEnergyDensityJPerM3);
    return result;
}

[[nodiscard]] EulerFlux hlleFlux(const ConservativeState& left,
                                 const ConservativeState& right,
                                 const PrimitiveState& leftPrimitive,
                                 const PrimitiveState& rightPrimitive,
                                 const EulerFlux& leftFlux,
                                 const EulerFlux& rightFlux) noexcept {
    const auto leftWave = std::min(leftPrimitive.velocityMps - leftPrimitive.speedOfSoundMps,
                                   rightPrimitive.velocityMps - rightPrimitive.speedOfSoundMps);
    const auto rightWave = std::max(leftPrimitive.velocityMps + leftPrimitive.speedOfSoundMps,
                                    rightPrimitive.velocityMps + rightPrimitive.speedOfSoundMps);
    if (leftWave >= 0.0) return leftFlux;
    if (rightWave <= 0.0) return rightFlux;
    const auto denominator = rightWave - leftWave;
    if (!(denominator > 1.0e-12)) {
        EulerFlux average;
        for (std::size_t index = 0; index < gasSpeciesCount; ++index) {
            average.speciesMassFluxKgPerM2S[index] = 0.5
                * (leftFlux.speciesMassFluxKgPerM2S[index]
                   + rightFlux.speciesMassFluxKgPerM2S[index]);
        }
        average.momentumFluxPa = 0.5
            * (leftFlux.momentumFluxPa + rightFlux.momentumFluxPa);
        average.totalEnergyFluxWPerM2 = 0.5
            * (leftFlux.totalEnergyFluxWPerM2 + rightFlux.totalEnergyFluxWPerM2);
        return average;
    }

    EulerFlux result;
    for (std::size_t index = 0; index < gasSpeciesCount; ++index) {
        result.speciesMassFluxKgPerM2S[index] =
            (rightWave * leftFlux.speciesMassFluxKgPerM2S[index]
             - leftWave * rightFlux.speciesMassFluxKgPerM2S[index]
             + leftWave * rightWave
                 * (right.speciesMassDensityKgPerM3[index]
                    - left.speciesMassDensityKgPerM3[index]))
            / denominator;
    }
    result.momentumFluxPa =
        (rightWave * leftFlux.momentumFluxPa - leftWave * rightFlux.momentumFluxPa
         + leftWave * rightWave
             * (right.momentumDensityKgPerM2S - left.momentumDensityKgPerM2S))
        / denominator;
    result.totalEnergyFluxWPerM2 =
        (rightWave * leftFlux.totalEnergyFluxWPerM2
         - leftWave * rightFlux.totalEnergyFluxWPerM2
         + leftWave * rightWave
             * (right.totalEnergyDensityJPerM3 - left.totalEnergyDensityJPerM3))
        / denominator;
    return result;
}

void accumulateFluxIntegral(EulerFluxIntegral& integral,
                            const EulerFlux& firstStage,
                            const EulerFlux& secondStage,
                            double timeStepSeconds) noexcept {
    const auto weight = 0.5 * timeStepSeconds;
    for (std::size_t index = 0; index < gasSpeciesCount; ++index) {
        integral.speciesMassKgPerM2[index] += weight
            * (firstStage.speciesMassFluxKgPerM2S[index]
               + secondStage.speciesMassFluxKgPerM2S[index]);
    }
    integral.momentumImpulseNsPerM2 += weight
        * (firstStage.momentumFluxPa + secondStage.momentumFluxPa);
    integral.totalEnergyJPerM2 += weight
        * (firstStage.totalEnergyFluxWPerM2 + secondStage.totalEnergyFluxWPerM2);
}

[[nodiscard]] bool finite(double value) noexcept {
    return std::isfinite(value);
}

} // namespace

ThermodynamicModel ThermodynamicModel::standardCombustionGas() noexcept {
    constexpr double oxygenMolarMassKg = 0.0319988;
    constexpr double inertMolarMassKg = 0.0280134;
    constexpr double gasolineMolarMassKg = 0.11423;
    constexpr double oxygenMolesPerFuelMole = 12.5;
    constexpr double productMolesPerFuelMole = 17.0;
    constexpr double burnedGamma = 1.26;
    const auto burnedMolarMassKg =
        (gasolineMolarMassKg + oxygenMolesPerFuelMole * oxygenMolarMassKg)
        / productMolesPerFuelMole;
    return { {
        SpeciesThermodynamics { oxygenMolarMassKg, 2.5 * universalGasConstantJPerMolK },
        SpeciesThermodynamics { inertMolarMassKg, 2.5 * universalGasConstantJPerMolK },
        SpeciesThermodynamics { gasolineMolarMassKg, 145.0 },
        SpeciesThermodynamics { burnedMolarMassKg,
                                universalGasConstantJPerMolK / (burnedGamma - 1.0) },
    } };
}

bool ThermodynamicModel::valid() const noexcept {
    for (const auto& entry : species) {
        if (!finite(entry.molarMassKgPerMol) || !(entry.molarMassKgPerMol > 0.0)
            || !finite(entry.molarHeatCapacityCvJPerMolK)
            || !(entry.molarHeatCapacityCvJPerMolK > 0.0))
            return false;
    }
    return true;
}

GasComposition GasComposition::dryAir() noexcept {
    constexpr double oxygenMoleFraction = 0.21;
    constexpr double inertMoleFraction = 0.79;
    constexpr double oxygenMolarMassKg = 0.0319988;
    constexpr double inertMolarMassKg = 0.0280134;
    const auto oxygenMass = oxygenMoleFraction * oxygenMolarMassKg;
    const auto inertMass = inertMoleFraction * inertMolarMassKg;
    const auto totalMass = oxygenMass + inertMass;
    return { { oxygenMass / totalMass, inertMass / totalMass, 0.0, 0.0 } };
}

GasComposition GasComposition::inertGas() noexcept {
    return { { 0.0, 1.0, 0.0, 0.0 } };
}

double ConservativeState::densityKgPerM3() const noexcept {
    double density = 0.0;
    for (const auto speciesDensity : speciesMassDensityKgPerM3)
        density += speciesDensity;
    return density;
}

EulerMixtureModel::EulerMixtureModel(ThermodynamicModel model) noexcept
    : model_(model) {}

std::optional<ConservativeState> EulerMixtureModel::conservativeFromPrimitive(
    double density, double velocity, double pressure,
    GasComposition composition) const noexcept {
    if (!model_.valid() || !finite(density) || !(density > minimumDensityKgPerM3)
        || !finite(velocity) || !finite(pressure) || !(pressure > 0.0))
        return std::nullopt;

    auto fractionSum = 0.0;
    for (const auto fraction : composition.massFractions) {
        if (!finite(fraction) || fraction < 0.0) return std::nullopt;
        fractionSum += fraction;
    }
    if (!(fractionSum > 0.0) || !finite(fractionSum)) return std::nullopt;

    ConservativeState result;
    auto molarDensity = 0.0;
    auto heatCapacityDensity = 0.0;
    for (std::size_t index = 0; index < gasSpeciesCount; ++index) {
        const auto massFraction = composition.massFractions[index] / fractionSum;
        const auto speciesDensity = density * massFraction;
        result.speciesMassDensityKgPerM3[index] = speciesDensity;
        const auto speciesMolesPerM3 = speciesDensity / model_.species[index].molarMassKgPerMol;
        molarDensity += speciesMolesPerM3;
        heatCapacityDensity += speciesMolesPerM3
            * model_.species[index].molarHeatCapacityCvJPerMolK;
    }
    if (!(molarDensity > 0.0) || !(heatCapacityDensity > 0.0)) return std::nullopt;
    const auto temperature = pressure / (molarDensity * universalGasConstantJPerMolK);
    const auto internalEnergyDensity = heatCapacityDensity * temperature;
    result.momentumDensityKgPerM2S = density * velocity;
    result.totalEnergyDensityJPerM3 = internalEnergyDensity
        + 0.5 * density * velocity * velocity;
    if (!isPhysical(result)) return std::nullopt;
    return result;
}

std::optional<ConservativeState> EulerMixtureModel::conservativeFromPressureTemperature(
    double pressure, double temperature, double velocity,
    GasComposition composition) const noexcept {
    if (!model_.valid() || !finite(pressure) || !(pressure > 0.0)
        || !finite(temperature) || !(temperature > 0.0) || !finite(velocity))
        return std::nullopt;
    auto fractionSum = 0.0;
    auto molesPerKg = 0.0;
    for (std::size_t index = 0; index < gasSpeciesCount; ++index) {
        const auto fraction = composition.massFractions[index];
        if (!finite(fraction) || fraction < 0.0) return std::nullopt;
        fractionSum += fraction;
    }
    if (!(fractionSum > 0.0)) return std::nullopt;
    for (std::size_t index = 0; index < gasSpeciesCount; ++index) {
        const auto normalisedFraction = composition.massFractions[index] / fractionSum;
        molesPerKg += normalisedFraction / model_.species[index].molarMassKgPerMol;
    }
    if (!(molesPerKg > 0.0) || !finite(molesPerKg)) return std::nullopt;
    const auto density = pressure
        / (molesPerKg * universalGasConstantJPerMolK * temperature);
    return conservativeFromPrimitive(density, velocity, pressure, composition);
}

std::optional<PrimitiveState> EulerMixtureModel::primitiveFromConservative(
    const ConservativeState& state) const noexcept {
    if (!model_.valid()) return std::nullopt;
    auto density = 0.0;
    auto molarDensity = 0.0;
    auto heatCapacityDensity = 0.0;
    for (std::size_t index = 0; index < gasSpeciesCount; ++index) {
        const auto speciesDensity = state.speciesMassDensityKgPerM3[index];
        if (!finite(speciesDensity) || speciesDensity < 0.0) return std::nullopt;
        density += speciesDensity;
        const auto molesPerM3 = speciesDensity / model_.species[index].molarMassKgPerMol;
        molarDensity += molesPerM3;
        heatCapacityDensity += molesPerM3
            * model_.species[index].molarHeatCapacityCvJPerMolK;
    }
    if (!(density > minimumDensityKgPerM3) || !(molarDensity > 0.0)
        || !(heatCapacityDensity > 0.0) || !finite(state.momentumDensityKgPerM2S)
        || !finite(state.totalEnergyDensityJPerM3))
        return std::nullopt;

    const auto velocity = state.momentumDensityKgPerM2S / density;
    const auto kineticEnergyDensity = 0.5 * density * velocity * velocity;
    const auto internalEnergyDensity = state.totalEnergyDensityJPerM3 - kineticEnergyDensity;
    if (!(internalEnergyDensity > minimumInternalEnergyDensityJPerM3)
        || !finite(internalEnergyDensity))
        return std::nullopt;

    const auto temperature = internalEnergyDensity / heatCapacityDensity;
    const auto pressure = molarDensity * universalGasConstantJPerMolK * temperature;
    const auto gamma = 1.0
        + molarDensity * universalGasConstantJPerMolK / heatCapacityDensity;
    const auto soundSpeedSquared = gamma * pressure / density;
    if (!(temperature > 0.0) || !(pressure > 0.0) || !(gamma > 1.0)
        || !(soundSpeedSquared > 0.0) || !finite(temperature) || !finite(pressure)
        || !finite(gamma) || !finite(soundSpeedSquared))
        return std::nullopt;

    PrimitiveState result;
    result.densityKgPerM3 = density;
    result.velocityMps = velocity;
    result.pressurePa = pressure;
    result.temperatureK = temperature;
    result.heatCapacityRatio = gamma;
    result.speedOfSoundMps = std::sqrt(soundSpeedSquared);
    for (std::size_t index = 0; index < gasSpeciesCount; ++index)
        result.massFractions[index] = state.speciesMassDensityKgPerM3[index] / density;
    return result;
}

bool EulerMixtureModel::isPhysical(const ConservativeState& state) const noexcept {
    return primitiveFromConservative(state).has_value();
}

bool EulerMixtureModel::canonicaliseSpeciesRoundoff(ConservativeState& state) const noexcept {
    auto targetDensity = 0.0;
    auto positiveDensity = 0.0;
    auto needsRepair = false;
    for (const auto speciesDensity : state.speciesMassDensityKgPerM3) {
        if (!finite(speciesDensity)) return false;
        targetDensity += speciesDensity;
        if (speciesDensity >= 0.0) positiveDensity += speciesDensity;
        else needsRepair = true;
    }
    if (!needsRepair) return true;
    if (!(targetDensity > minimumDensityKgPerM3) || !(positiveDensity > 0.0)) return false;
    const auto roundoffTolerance = 128.0 * std::numeric_limits<double>::epsilon()
        * std::max(targetDensity, minimumDensityKgPerM3);
    for (const auto speciesDensity : state.speciesMassDensityKgPerM3)
        if (speciesDensity < -roundoffTolerance) return false;

    const auto retainedFraction = targetDensity / positiveDensity;
    for (auto& speciesDensity : state.speciesMassDensityKgPerM3) {
        speciesDensity = speciesDensity > 0.0
            ? speciesDensity * retainedFraction : 0.0;
    }
    return true;
}

EulerFlux EulerMixtureModel::physicalFlux(const ConservativeState& state) const noexcept {
    const auto primitive = primitiveFromConservative(state);
    if (!primitive) return {};
    EulerFlux result;
    for (std::size_t index = 0; index < gasSpeciesCount; ++index) {
        result.speciesMassFluxKgPerM2S[index] =
            state.speciesMassDensityKgPerM3[index] * primitive->velocityMps;
    }
    result.momentumFluxPa = state.momentumDensityKgPerM2S * primitive->velocityMps
        + primitive->pressurePa;
    result.totalEnergyFluxWPerM2 =
        (state.totalEnergyDensityJPerM3 + primitive->pressurePa) * primitive->velocityMps;
    return result;
}

EulerFlux EulerMixtureModel::riemannFlux(const ConservativeState& left,
                                         const ConservativeState& right) const noexcept {
    const auto leftPrimitive = primitiveFromConservative(left);
    const auto rightPrimitive = primitiveFromConservative(right);
    if (!leftPrimitive || !rightPrimitive) return {};
    const auto leftPhysicalFlux = physicalFlux(left);
    const auto rightPhysicalFlux = physicalFlux(right);
    const auto leftWave = std::min(leftPrimitive->velocityMps - leftPrimitive->speedOfSoundMps,
                                   rightPrimitive->velocityMps - rightPrimitive->speedOfSoundMps);
    const auto rightWave = std::max(leftPrimitive->velocityMps + leftPrimitive->speedOfSoundMps,
                                    rightPrimitive->velocityMps + rightPrimitive->speedOfSoundMps);
    if (leftWave >= 0.0) return leftPhysicalFlux;
    if (rightWave <= 0.0) return rightPhysicalFlux;

    const auto denominator = leftPrimitive->densityKgPerM3
            * (leftWave - leftPrimitive->velocityMps)
        - rightPrimitive->densityKgPerM3
            * (rightWave - rightPrimitive->velocityMps);
    if (std::abs(denominator) <= 1.0e-12 || !finite(denominator)) {
        return hlleFlux(left, right, *leftPrimitive, *rightPrimitive,
                        leftPhysicalFlux, rightPhysicalFlux);
    }
    const auto contactWave =
        (rightPrimitive->pressurePa - leftPrimitive->pressurePa
         + leftPrimitive->densityKgPerM3 * leftPrimitive->velocityMps
             * (leftWave - leftPrimitive->velocityMps)
         - rightPrimitive->densityKgPerM3 * rightPrimitive->velocityMps
             * (rightWave - rightPrimitive->velocityMps))
        / denominator;
    const auto leftStarPressure = leftPrimitive->pressurePa
        + leftPrimitive->densityKgPerM3
            * (leftWave - leftPrimitive->velocityMps)
            * (contactWave - leftPrimitive->velocityMps);
    const auto rightStarPressure = rightPrimitive->pressurePa
        + rightPrimitive->densityKgPerM3
            * (rightWave - rightPrimitive->velocityMps)
            * (contactWave - rightPrimitive->velocityMps);
    const auto starPressure = 0.5 * (leftStarPressure + rightStarPressure);
    if (!finite(contactWave) || !finite(starPressure) || !(starPressure > 0.0)) {
        return hlleFlux(left, right, *leftPrimitive, *rightPrimitive,
                        leftPhysicalFlux, rightPhysicalFlux);
    }

    const auto starState = [contactWave, starPressure](
        const ConservativeState& state, const PrimitiveState& primitive,
        double outerWave) noexcept -> std::optional<ConservativeState> {
        const auto starDenominator = outerWave - contactWave;
        if (std::abs(starDenominator) <= 1.0e-12) return std::nullopt;
        const auto scale = (outerWave - primitive.velocityMps) / starDenominator;
        if (!(scale > 0.0) || !finite(scale)) return std::nullopt;
        ConservativeState result;
        for (std::size_t index = 0; index < gasSpeciesCount; ++index)
            result.speciesMassDensityKgPerM3[index] =
                state.speciesMassDensityKgPerM3[index] * scale;
        result.momentumDensityKgPerM2S = primitive.densityKgPerM3 * scale * contactWave;
        result.totalEnergyDensityJPerM3 =
            ((outerWave - primitive.velocityMps) * state.totalEnergyDensityJPerM3
             - primitive.pressurePa * primitive.velocityMps
             + starPressure * contactWave)
            / starDenominator;
        return result;
    };

    if (contactWave >= 0.0) {
        const auto star = starState(left, *leftPrimitive, leftWave);
        if (star && isPhysical(*star))
            return fluxDifferenceStateScaled(leftPhysicalFlux, *star, left, leftWave);
    } else {
        const auto star = starState(right, *rightPrimitive, rightWave);
        if (star && isPhysical(*star))
            return fluxDifferenceStateScaled(rightPhysicalFlux, *star, right, rightWave);
    }
    return hlleFlux(left, right, *leftPrimitive, *rightPrimitive,
                    leftPhysicalFlux, rightPhysicalFlux);
}

double DuctGeometry::areaM2() const noexcept {
    if (crossSectionAreaM2 > 0.0) return crossSectionAreaM2;
    const auto radius = 0.5 * diameterM;
    return std::numbers::pi * radius * radius;
}

double DuctGeometry::cellLengthM() const noexcept {
    return cellCount > 0 ? lengthM / static_cast<double>(cellCount) : 0.0;
}

bool DuctGeometry::valid() const noexcept {
    return finite(lengthM) && lengthM > 0.0
        && finite(crossSectionAreaM2) && crossSectionAreaM2 >= 0.0
        && finite(diameterM) && diameterM > 0.0
        && cellCount >= 2 && cellCount <= 1'000'000
        && finite(absoluteRoughnessM) && absoluteRoughnessM >= 0.0
        && finite(localLossCoefficient) && localLossCoefficient >= 0.0
        && finite(wallHeatTransferWPerM2K) && wallHeatTransferWPerM2K >= 0.0
        && finite(wallTemperatureK) && wallTemperatureK > 0.0;
}

DuctBoundaryCondition DuctBoundaryCondition::transmissive() noexcept {
    return { DuctBoundaryType::transmissive, {} };
}

DuctBoundaryCondition DuctBoundaryCondition::reflective() noexcept {
    return { DuctBoundaryType::reflective, {} };
}

DuctBoundaryCondition DuctBoundaryCondition::prescribed(
    const ConservativeState& state) noexcept {
    return { DuctBoundaryType::prescribed, state };
}

DuctBoundaryCondition DuctBoundaryCondition::periodic() noexcept {
    return { DuctBoundaryType::periodic, {} };
}

FiniteVolumeDuct::FiniteVolumeDuct(ThermodynamicModel model) noexcept
    : mixtureModel_(model) {}

bool FiniteVolumeDuct::configure(const DuctGeometry& geometry,
                                 const ConservativeState& initialState) {
    if (!geometry.valid() || !mixtureModel_.isPhysical(initialState)) return false;
    geometry_ = geometry;
    cells_.assign(geometry.cellCount, initialState);
    stage_.resize(geometry.cellCount);
    candidate_.resize(geometry.cellCount);
    residual_.resize(geometry.cellCount);
    stageResidual_.resize(geometry.cellCount);
    slopes_.resize(geometry.cellCount);
    faceFluxes_.resize(geometry.cellCount + 1);
    stageFaceFluxes_.resize(geometry.cellCount + 1);
    return true;
}

double FiniteVolumeDuct::cellCentreM(std::size_t index) const noexcept {
    if (index >= cells_.size()) return geometry_.lengthM;
    return (static_cast<double>(index) + 0.5) * geometry_.cellLengthM();
}

ConservedInventory FiniteVolumeDuct::inventory() const noexcept {
    ConservedInventory result;
    const auto cellVolume = geometry_.areaM2() * geometry_.cellLengthM();
    for (const auto& cell : cells_) {
        for (std::size_t index = 0; index < gasSpeciesCount; ++index)
            result.speciesMassKg[index] +=
                cell.speciesMassDensityKgPerM3[index] * cellVolume;
        result.axialMomentumKgMps += cell.momentumDensityKgPerM2S * cellVolume;
        result.totalEnergyJ += cell.totalEnergyDensityJPerM3 * cellVolume;
    }
    return result;
}

double FiniteVolumeDuct::maximumStableTimeStep(double maximumCourantNumber) const noexcept {
    if (cells_.empty() || !finite(maximumCourantNumber)
        || !(maximumCourantNumber > 0.0) || !(maximumCourantNumber <= 1.0))
        return 0.0;
    auto maximumSignalSpeed = 0.0;
    auto sourceLimitedStep = std::numeric_limits<double>::infinity();
    for (const auto& cell : cells_) {
        const auto primitive = mixtureModel_.primitiveFromConservative(cell);
        if (!primitive) return 0.0;
        maximumSignalSpeed = std::max(maximumSignalSpeed,
            std::abs(primitive->velocityMps) + primitive->speedOfSoundMps);

        if (geometry_.wallFrictionEnabled
            && std::abs(primitive->velocityMps) > 1.0e-9) {
            constexpr double referenceViscosityPaS = 1.716e-5;
            constexpr double referenceTemperatureK = 273.15;
            constexpr double sutherlandTemperatureK = 110.4;
            const auto viscosity = referenceViscosityPaS
                * std::pow(primitive->temperatureK / referenceTemperatureK, 1.5)
                * (referenceTemperatureK + sutherlandTemperatureK)
                / (primitive->temperatureK + sutherlandTemperatureK);
            const auto reynolds = primitive->densityKgPerM3
                * std::abs(primitive->velocityMps) * geometry_.diameterM
                / std::max(1.0e-12, viscosity);
            auto frictionFactor = 0.0;
            if (reynolds > 1.0) {
                if (reynolds < 2'300.0) {
                    frictionFactor = 64.0 / reynolds;
                } else {
                    const auto inverseRoot = -1.8 * std::log10(
                        std::pow(geometry_.absoluteRoughnessM
                                     / geometry_.diameterM / 3.7,
                                 1.11)
                        + 6.9 / reynolds);
                    frictionFactor = 1.0 / (inverseRoot * inverseRoot);
                }
            }
            const auto lossGradient = frictionFactor / geometry_.diameterM
                + geometry_.localLossCoefficient / geometry_.lengthM;
            const auto momentumSource = -0.5 * lossGradient
                * primitive->densityKgPerM3 * primitive->velocityMps
                * std::abs(primitive->velocityMps);
            if (std::abs(momentumSource) > 1.0e-12) {
                sourceLimitedStep = std::min(sourceLimitedStep,
                    0.5 * std::abs(cell.momentumDensityKgPerM2S / momentumSource));
            }
        }

        if (geometry_.wallHeatTransferWPerM2K > 0.0
            && primitive->temperatureK > geometry_.wallTemperatureK) {
            const auto heatSource = geometry_.wallHeatTransferWPerM2K
                * (4.0 / geometry_.diameterM)
                * (geometry_.wallTemperatureK - primitive->temperatureK);
            const auto kineticEnergy = 0.5 * primitive->densityKgPerM3
                * primitive->velocityMps * primitive->velocityMps;
            const auto internalEnergy = cell.totalEnergyDensityJPerM3 - kineticEnergy;
            if (heatSource < -1.0e-12) {
                sourceLimitedStep = std::min(sourceLimitedStep,
                    0.5 * internalEnergy / -heatSource);
            }
        }
    }
    if (!(maximumSignalSpeed > 0.0) || !finite(maximumSignalSpeed)) return 0.0;
    return std::min(maximumCourantNumber * geometry_.cellLengthM() / maximumSignalSpeed,
                    sourceLimitedStep);
}

bool FiniteVolumeDuct::allStatesPhysical(
    std::span<const ConservativeState> states) const noexcept {
    for (const auto& state : states)
        if (!mixtureModel_.isPhysical(state)) return false;
    return true;
}

void FiniteVolumeDuct::computeResidual(
    std::span<const ConservativeState> states,
    const DuctBoundaryCondition& leftBoundary,
    const DuctBoundaryCondition& rightBoundary,
    std::span<ConservativeState> residual,
    std::span<EulerFlux> faceFluxes) noexcept {
    const auto count = states.size();
    const auto periodic = leftBoundary.type == DuctBoundaryType::periodic;
    std::fill(slopes_.begin(), slopes_.end(), ConservativeState {});
    if (periodic) {
        for (std::size_t index = 0; index < count; ++index) {
            const auto previous = index == 0 ? count - 1 : index - 1;
            const auto next = index + 1 == count ? 0 : index + 1;
            slopes_[index] = monotonisedCentralSlope(
                states[previous], states[index], states[next]);
        }
    } else {
        for (std::size_t index = 1; index + 1 < count; ++index)
            slopes_[index] = monotonisedCentralSlope(
                states[index - 1], states[index], states[index + 1]);
    }

    const auto reconstructed = [this](const ConservativeState& cell,
                                      const ConservativeState& slope,
                                      double sign) noexcept {
        const auto candidate = addScaled(cell, slope, 0.5 * sign);
        return mixtureModel_.isPhysical(candidate) ? candidate : cell;
    };

    if (periodic) {
        const auto faceLeft = reconstructed(states[count - 1], slopes_[count - 1], 1.0);
        const auto faceRight = reconstructed(states[0], slopes_[0], -1.0);
        faceFluxes[0] = mixtureModel_.riemannFlux(faceLeft, faceRight);
        faceFluxes[count] = faceFluxes[0];
    } else {
        const auto insideLeft = reconstructed(states[0], slopes_[0], -1.0);
        auto outsideLeft = insideLeft;
        if (leftBoundary.type == DuctBoundaryType::reflective)
            outsideLeft = mirrored(insideLeft);
        else if (leftBoundary.type == DuctBoundaryType::prescribed)
            outsideLeft = leftBoundary.prescribedState;
        faceFluxes[0] = mixtureModel_.riemannFlux(outsideLeft, insideLeft);

        const auto insideRight = reconstructed(states[count - 1], slopes_[count - 1], 1.0);
        auto outsideRight = insideRight;
        if (rightBoundary.type == DuctBoundaryType::reflective)
            outsideRight = mirrored(insideRight);
        else if (rightBoundary.type == DuctBoundaryType::prescribed)
            outsideRight = rightBoundary.prescribedState;
        faceFluxes[count] = mixtureModel_.riemannFlux(insideRight, outsideRight);
    }

    for (std::size_t face = 1; face < count; ++face) {
        const auto faceLeft = reconstructed(states[face - 1], slopes_[face - 1], 1.0);
        const auto faceRight = reconstructed(states[face], slopes_[face], -1.0);
        faceFluxes[face] = mixtureModel_.riemannFlux(faceLeft, faceRight);
    }

    const auto inverseCellLength = 1.0 / geometry_.cellLengthM();
    for (std::size_t index = 0; index < count; ++index) {
        auto& cellResidual = residual[index];
        for (std::size_t species = 0; species < gasSpeciesCount; ++species) {
            cellResidual.speciesMassDensityKgPerM3[species] = -inverseCellLength
                * (faceFluxes[index + 1].speciesMassFluxKgPerM2S[species]
                   - faceFluxes[index].speciesMassFluxKgPerM2S[species]);
        }
        cellResidual.momentumDensityKgPerM2S = -inverseCellLength
            * (faceFluxes[index + 1].momentumFluxPa
               - faceFluxes[index].momentumFluxPa);
        cellResidual.totalEnergyDensityJPerM3 = -inverseCellLength
            * (faceFluxes[index + 1].totalEnergyFluxWPerM2
               - faceFluxes[index].totalEnergyFluxWPerM2);

        const auto primitive = mixtureModel_.primitiveFromConservative(states[index]);
        if (!primitive) continue;
        if (geometry_.wallFrictionEnabled
            && std::abs(primitive->velocityMps) > 1.0e-9) {
            constexpr double referenceViscosityPaS = 1.716e-5;
            constexpr double referenceTemperatureK = 273.15;
            constexpr double sutherlandTemperatureK = 110.4;
            const auto viscosity = referenceViscosityPaS
                * std::pow(primitive->temperatureK / referenceTemperatureK, 1.5)
                * (referenceTemperatureK + sutherlandTemperatureK)
                / (primitive->temperatureK + sutherlandTemperatureK);
            const auto reynolds = primitive->densityKgPerM3
                * std::abs(primitive->velocityMps) * geometry_.diameterM
                / std::max(1.0e-12, viscosity);
            auto frictionFactor = 0.0;
            if (reynolds > 1.0) {
                if (reynolds < 2'300.0) {
                    frictionFactor = 64.0 / reynolds;
                } else {
                    const auto inverseRoot = -1.8 * std::log10(
                        std::pow(geometry_.absoluteRoughnessM
                                     / geometry_.diameterM / 3.7,
                                 1.11)
                        + 6.9 / reynolds);
                    frictionFactor = 1.0 / (inverseRoot * inverseRoot);
                }
            }
            const auto lossGradient = frictionFactor / geometry_.diameterM
                + geometry_.localLossCoefficient / geometry_.lengthM;
            cellResidual.momentumDensityKgPerM2S -= 0.5 * lossGradient
                * primitive->densityKgPerM3 * primitive->velocityMps
                * std::abs(primitive->velocityMps);
        }
        if (geometry_.wallHeatTransferWPerM2K > 0.0) {
            cellResidual.totalEnergyDensityJPerM3 +=
                geometry_.wallHeatTransferWPerM2K * (4.0 / geometry_.diameterM)
                * (geometry_.wallTemperatureK - primitive->temperatureK);
        }
    }
}

DuctAdvanceResult FiniteVolumeDuct::advance(
    double durationSeconds,
    DuctBoundaryCondition left,
    DuctBoundaryCondition right,
    double maximumCourantNumber,
    std::size_t maximumSubsteps) noexcept {
    DuctAdvanceResult result;
    if (cells_.empty() || !finite(durationSeconds) || durationSeconds < 0.0
        || !finite(maximumCourantNumber) || !(maximumCourantNumber > 0.0)
        || maximumCourantNumber > 1.0 || maximumSubsteps == 0) {
        result.completed = false;
        return result;
    }
    if (durationSeconds == 0.0) return result;
    const auto leftPeriodic = left.type == DuctBoundaryType::periodic;
    const auto rightPeriodic = right.type == DuctBoundaryType::periodic;
    if (leftPeriodic != rightPeriodic
        || (left.type == DuctBoundaryType::prescribed
            && !mixtureModel_.isPhysical(left.prescribedState))
        || (right.type == DuctBoundaryType::prescribed
            && !mixtureModel_.isPhysical(right.prescribedState))) {
        result.completed = false;
        return result;
    }

    auto remaining = durationSeconds;
    std::size_t attempts = 0;
    const auto completionTolerance = std::max(1.0e-15, durationSeconds * 1.0e-13);
    while (remaining > completionTolerance && attempts < maximumSubsteps) {
        const auto stableStep = maximumStableTimeStep(maximumCourantNumber);
        if (!(stableStep > 0.0) || !finite(stableStep)) {
            result.completed = false;
            break;
        }
        auto trialStep = std::min(remaining, stableStep);
        auto accepted = false;
        while (!accepted && attempts < maximumSubsteps) {
            ++attempts;
            computeResidual(cells_, left, right, residual_, faceFluxes_);
            for (std::size_t index = 0; index < cells_.size(); ++index)
                stage_[index] = addScaled(cells_[index], residual_[index], trialStep);
            const auto stageRoundoffIsValid = std::all_of(
                stage_.begin(), stage_.end(), [this](ConservativeState& state) {
                    return mixtureModel_.canonicaliseSpeciesRoundoff(state);
                });
            if (!stageRoundoffIsValid || !allStatesPhysical(stage_)) {
                ++result.rejectedSubsteps;
                trialStep *= 0.5;
                if (!(trialStep > std::numeric_limits<double>::epsilon()
                                  * std::max(1.0, durationSeconds)))
                    break;
                continue;
            }

            computeResidual(stage_, left, right, stageResidual_, stageFaceFluxes_);
            for (std::size_t index = 0; index < cells_.size(); ++index) {
                const auto forwardEuler = addScaled(stage_[index], stageResidual_[index], trialStep);
                candidate_[index] = addScaled(cells_[index],
                    difference(forwardEuler, cells_[index]), 0.5);
            }
            const auto candidateRoundoffIsValid = std::all_of(
                candidate_.begin(), candidate_.end(), [this](ConservativeState& state) {
                    return mixtureModel_.canonicaliseSpeciesRoundoff(state);
                });
            if (!candidateRoundoffIsValid || !allStatesPhysical(candidate_)) {
                ++result.rejectedSubsteps;
                trialStep *= 0.5;
                if (!(trialStep > std::numeric_limits<double>::epsilon()
                                  * std::max(1.0, durationSeconds)))
                    break;
                continue;
            }

            accumulateFluxIntegral(result.leftBoundaryFlux,
                faceFluxes_.front(), stageFaceFluxes_.front(), trialStep);
            accumulateFluxIntegral(result.rightBoundaryFlux,
                faceFluxes_.back(), stageFaceFluxes_.back(), trialStep);
            cells_.swap(candidate_);
            remaining -= trialStep;
            result.advancedTimeSeconds += trialStep;
            ++result.acceptedSubsteps;
            accepted = true;
        }
        if (!accepted) {
            result.completed = false;
            break;
        }
    }
    if (remaining > completionTolerance) result.completed = false;
    return result;
}

} // namespace enginelab::gasdynamics
