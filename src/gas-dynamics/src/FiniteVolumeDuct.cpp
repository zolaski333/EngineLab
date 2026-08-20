#include <enginelab/gasdynamics/FiniteVolumeDuct.hpp>

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>

namespace enginelab::gasdynamics {
namespace {

constexpr double minimumDensityKgPerM3 = 1.0e-12;
constexpr double minimumInternalEnergyDensityJPerM3 = 1.0e-9;
constexpr double reconstructionBias = 1.5;

[[nodiscard]] constexpr bool finite(double value) noexcept {
    static_assert(std::numeric_limits<double>::is_iec559
                  && sizeof(double) == sizeof(std::uint64_t),
        "the conservative gas solver requires IEEE-754 binary64 doubles");
    constexpr auto exponentMask = std::uint64_t { 0x7ff0000000000000ULL };
    return (std::bit_cast<std::uint64_t>(value) & exponentMask) != exponentMask;
}

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

[[nodiscard]] PrimitiveState mirrored(const PrimitiveState& state) noexcept {
    auto result = state;
    result.velocityMps = -result.velocityMps;
    return result;
}

[[nodiscard]] bool identical(const ConservativeState& first,
                             const ConservativeState& second) noexcept {
    return first.speciesMassDensityKgPerM3 == second.speciesMassDensityKgPerM3
        && first.momentumDensityKgPerM2S == second.momentumDensityKgPerM2S
        && first.totalEnergyDensityJPerM3 == second.totalEnergyDensityJPerM3;
}

[[nodiscard]] bool isZero(const ConservativeState& state) noexcept {
    return state.speciesMassDensityKgPerM3
            == std::array<double, gasSpeciesCount> {}
        && state.momentumDensityKgPerM2S == 0.0
        && state.totalEnergyDensityJPerM3 == 0.0;
}

[[nodiscard]] bool hasPositiveDensityAndInternalEnergy(
    const ConservativeState& state) noexcept {
    auto density = 0.0;
    for (const auto speciesDensity : state.speciesMassDensityKgPerM3) {
        if (!finite(speciesDensity) || speciesDensity < 0.0) return false;
        density += speciesDensity;
    }
    if (!(density > minimumDensityKgPerM3)
        || !finite(state.momentumDensityKgPerM2S)
        || !finite(state.totalEnergyDensityJPerM3))
        return false;
    const auto kineticEnergyDensity = 0.5 * state.momentumDensityKgPerM2S
        * state.momentumDensityKgPerM2S / density;
    return finite(kineticEnergyDensity)
        && state.totalEnergyDensityJPerM3 - kineticEnergyDensity
            > minimumInternalEnergyDensityJPerM3;
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
    : model_(model), modelIsValid_(model.valid()) {
    if (!modelIsValid_) return;
    for (std::size_t index = 0; index < gasSpeciesCount; ++index) {
        inverseMolarMassKg_[index] = 1.0 / model_.species[index].molarMassKgPerMol;
        specificHeatCapacityCvJPerKgK_[index] =
            model_.species[index].molarHeatCapacityCvJPerMolK
            * inverseMolarMassKg_[index];
    }
}

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
    PrimitiveState result;
    if (!recoverPrimitive(state, result)) return std::nullopt;
    return result;
}

bool EulerMixtureModel::recoverPrimitive(const ConservativeState& state,
                                         PrimitiveState& result) const noexcept {
    if (!modelIsValid_) return false;
    auto density = 0.0;
    auto molarDensity = 0.0;
    auto heatCapacityDensity = 0.0;
    for (std::size_t index = 0; index < gasSpeciesCount; ++index) {
        const auto speciesDensity = state.speciesMassDensityKgPerM3[index];
        if (!finite(speciesDensity) || speciesDensity < 0.0) return false;
        density += speciesDensity;
        const auto molesPerM3 = speciesDensity * inverseMolarMassKg_[index];
        molarDensity += molesPerM3;
        heatCapacityDensity += speciesDensity
            * specificHeatCapacityCvJPerKgK_[index];
    }
    if (!(density > minimumDensityKgPerM3) || !(molarDensity > 0.0)
        || !(heatCapacityDensity > 0.0) || !finite(state.momentumDensityKgPerM2S)
        || !finite(state.totalEnergyDensityJPerM3))
        return false;

    const auto velocity = state.momentumDensityKgPerM2S / density;
    const auto kineticEnergyDensity = 0.5 * density * velocity * velocity;
    const auto internalEnergyDensity = state.totalEnergyDensityJPerM3 - kineticEnergyDensity;
    if (!(internalEnergyDensity > minimumInternalEnergyDensityJPerM3)
        || !finite(internalEnergyDensity))
        return false;

    const auto temperature = internalEnergyDensity / heatCapacityDensity;
    const auto pressure = molarDensity * universalGasConstantJPerMolK * temperature;
    const auto gamma = 1.0
        + molarDensity * universalGasConstantJPerMolK / heatCapacityDensity;
    const auto soundSpeedSquared = gamma * pressure / density;
    if (!(temperature > 0.0) || !(pressure > 0.0) || !(gamma > 1.0)
        || !(soundSpeedSquared > 0.0) || !finite(temperature) || !finite(pressure)
        || !finite(gamma) || !finite(soundSpeedSquared))
        return false;

    result.densityKgPerM3 = density;
    result.velocityMps = velocity;
    result.pressurePa = pressure;
    result.temperatureK = temperature;
    result.heatCapacityRatio = gamma;
    result.speedOfSoundMps = std::sqrt(soundSpeedSquared);
    // Measured by ablation, and the result refutes an optimisation that looks
    // obvious: these four divisions are ~31% of every division the duct solver
    // performs, and removing them buys **1.4%** while leaving all six bench
    // checksums bit-identical -- so they are provably unread on this path, and
    // provably not where the time goes. They are independent of one another and
    // therefore pipeline at division THROUGHPUT; what costs is the dependent
    // chain above (density -> velocity -> internal energy -> temperature ->
    // pressure -> gamma -> sound speed -> sqrt), which cannot be shortened by
    // deleting independent work. Same reason `/arch:AVX2` measures slower.
    for (std::size_t index = 0; index < gasSpeciesCount; ++index)
        result.massFractions[index] = state.speciesMassDensityKgPerM3[index] / density;
    return true;
}

bool EulerMixtureModel::isPhysical(const ConservativeState& state) const noexcept {
    PrimitiveState primitive;
    return recoverPrimitive(state, primitive);
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
    PrimitiveState primitive;
    if (!recoverPrimitive(state, primitive)) return {};
    return physicalFluxPrepared(state, primitive);
}

EulerFlux EulerMixtureModel::physicalFluxPrepared(
    const ConservativeState& state,
    const PrimitiveState& primitive) const noexcept {
    EulerFlux result;
    for (std::size_t index = 0; index < gasSpeciesCount; ++index) {
        result.speciesMassFluxKgPerM2S[index] =
            state.speciesMassDensityKgPerM3[index] * primitive.velocityMps;
    }
    result.momentumFluxPa = state.momentumDensityKgPerM2S * primitive.velocityMps
        + primitive.pressurePa;
    result.totalEnergyFluxWPerM2 =
        (state.totalEnergyDensityJPerM3 + primitive.pressurePa) * primitive.velocityMps;
    return result;
}

EulerFlux EulerMixtureModel::riemannFlux(const ConservativeState& left,
                                         const ConservativeState& right) const noexcept {
    PrimitiveState leftPrimitive;
    PrimitiveState rightPrimitive;
    if (!recoverPrimitive(left, leftPrimitive)
        || !recoverPrimitive(right, rightPrimitive))
        return {};
    return riemannFluxPrepared(left, leftPrimitive, right, rightPrimitive);
}

EulerFlux EulerMixtureModel::riemannFluxPrepared(
    const ConservativeState& left,
    const PrimitiveState& leftPrimitiveValue,
    const ConservativeState& right,
    const PrimitiveState& rightPrimitiveValue) const noexcept {
    if (identical(left, right)) return physicalFluxPrepared(left, leftPrimitiveValue);
    const auto* leftPrimitive = &leftPrimitiveValue;
    const auto* rightPrimitive = &rightPrimitiveValue;
    const auto leftPhysicalFlux = physicalFluxPrepared(left, *leftPrimitive);
    const auto rightPhysicalFlux = physicalFluxPrepared(right, *rightPrimitive);
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
        if (star && hasPositiveDensityAndInternalEnergy(*star))
            return fluxDifferenceStateScaled(leftPhysicalFlux, *star, left, leftWave);
    } else {
        const auto star = starState(right, *rightPrimitive, rightWave);
        if (star && hasPositiveDensityAndInternalEnergy(*star))
            return fluxDifferenceStateScaled(rightPhysicalFlux, *star, right, rightWave);
    }
    return hlleFlux(left, right, *leftPrimitive, *rightPrimitive,
                    leftPhysicalFlux, rightPhysicalFlux);
}

double DuctGeometry::areaM2() const noexcept {
    const auto inlet = inletAreaM2();
    const auto outlet = outletAreaM2();
    if (inlet == outlet) return inlet;
    // Exact length-mean area of a conical frustum:
    // V/L = pi/3 * (r0^2 + r0*r1 + r1^2).
    return (inlet + std::sqrt(inlet * outlet) + outlet) / 3.0;
}

double DuctGeometry::inletAreaM2() const noexcept {
    if (inletCrossSectionAreaM2 > 0.0) return inletCrossSectionAreaM2;
    if (crossSectionAreaM2 > 0.0) return crossSectionAreaM2;
    const auto radius = 0.5 * diameterM;
    return std::numbers::pi * radius * radius;
}

double DuctGeometry::outletAreaM2() const noexcept {
    if (outletCrossSectionAreaM2 > 0.0) return outletCrossSectionAreaM2;
    if (crossSectionAreaM2 > 0.0) return crossSectionAreaM2;
    const auto radius = 0.5 * diameterM;
    return std::numbers::pi * radius * radius;
}

double DuctGeometry::faceAreaM2(std::size_t faceIndex) const noexcept {
    if (cellCount == 0) return 0.0;
    const auto inlet = inletAreaM2();
    const auto outlet = outletAreaM2();
    if (inlet == outlet) return inlet;
    const auto bounded = std::min(faceIndex, cellCount);
    const auto fraction =
        static_cast<double>(bounded) / static_cast<double>(cellCount);
    const auto inletRadius = std::sqrt(inlet / std::numbers::pi);
    const auto outletRadius = std::sqrt(outlet / std::numbers::pi);
    const auto radius = std::lerp(inletRadius, outletRadius, fraction);
    return std::numbers::pi * radius * radius;
}

double DuctGeometry::cellAreaM2(std::size_t cellIndex) const noexcept {
    if (cellCount == 0) return 0.0;
    const auto bounded = std::min(cellIndex, cellCount - 1);
    const auto left = faceAreaM2(bounded);
    const auto right = faceAreaM2(bounded + 1);
    if (left == right) return left;
    return (left + std::sqrt(left * right) + right) / 3.0;
}

double DuctGeometry::cellVolumeM3(std::size_t cellIndex) const noexcept {
    return cellAreaM2(cellIndex) * cellLengthM();
}

double DuctGeometry::cellHydraulicDiameterM(
    std::size_t cellIndex) const noexcept {
    const auto meanArea = areaM2();
    return meanArea > 0.0
        ? diameterM * std::sqrt(cellAreaM2(cellIndex) / meanArea)
        : 0.0;
}

bool DuctGeometry::hasVariableArea() const noexcept {
    const auto scale = std::max(inletAreaM2(), outletAreaM2());
    return std::abs(inletAreaM2() - outletAreaM2())
        > std::max(1.0e-15, scale * 1.0e-12);
}

double DuctGeometry::cellLengthM() const noexcept {
    return cellCount > 0 ? lengthM / static_cast<double>(cellCount) : 0.0;
}

bool DuctGeometry::valid() const noexcept {
    return finite(lengthM) && lengthM > 0.0
        && finite(crossSectionAreaM2) && crossSectionAreaM2 >= 0.0
        && finite(inletCrossSectionAreaM2) && inletCrossSectionAreaM2 >= 0.0
        && finite(outletCrossSectionAreaM2) && outletCrossSectionAreaM2 >= 0.0
        && finite(inletAreaM2()) && inletAreaM2() > 0.0
        && finite(outletAreaM2()) && outletAreaM2() > 0.0
        && finite(diameterM) && diameterM > 0.0
        && cellCount >= 1 && cellCount <= 1'000'000
        && finite(absoluteRoughnessM) && absoluteRoughnessM >= 0.0
        && finite(localLossCoefficient) && localLossCoefficient >= 0.0
        && finite(wallHeatTransferWPerM2K) && wallHeatTransferWPerM2K >= 0.0
        && finite(wallTemperatureK) && wallTemperatureK > 0.0
        && (!dynamicWallHeatTransferEnabled || wallHeatTransferWPerM2K == 0.0)
        && finite(wallThicknessM) && wallThicknessM > 0.0
        && finite(wallDensityKgPerM3) && wallDensityKgPerM3 > 0.0
        && finite(wallSpecificHeatJPerKgK) && wallSpecificHeatJPerKgK > 0.0
        && finite(externalWallHeatTransferWPerM2K)
        && externalWallHeatTransferWPerM2K >= 0.0
        && finite(externalTemperatureK) && externalTemperatureK > 0.0
        && finite(cellularSubstrateOpenAreaRatio)
        && cellularSubstrateOpenAreaRatio > 0.0
        && cellularSubstrateOpenAreaRatio <= 1.0
        && finite(cellularSubstrateVolumetricHeatCapacityJPerM3K)
        && cellularSubstrateVolumetricHeatCapacityJPerM3K >= 0.0
        && (!homogenisedCellularSubstrate
            || (cellularSubstrateOpenAreaRatio > 0.0
                && cellularSubstrateOpenAreaRatio < 1.0
                && cellularSubstrateVolumetricHeatCapacityJPerM3K > 0.0));
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
    cellPrimitives_.resize(geometry.cellCount);
    stagePrimitives_.resize(geometry.cellCount);
    candidatePrimitives_.resize(geometry.cellCount);
    cellSourceTerms_.resize(geometry.cellCount);
    stageSourceTerms_.resize(geometry.cellCount);
    candidateSourceTerms_.resize(geometry.cellCount);
    reconstructedLeft_.resize(geometry.cellCount);
    reconstructedRight_.resize(geometry.cellCount);
    reconstructedLeftPrimitives_.resize(geometry.cellCount);
    reconstructedRightPrimitives_.resize(geometry.cellCount);
    faceFluxes_.resize(geometry.cellCount + 1);
    stageFaceFluxes_.resize(geometry.cellCount + 1);
    wallStates_.assign(geometry.cellCount,
        DuctWallThermalState { geometry.wallTemperatureK });
    candidateWallStates_.resize(geometry.cellCount);
    wallHeatTransferGeometries_.resize(geometry.cellCount);
    faceAreasM2_.resize(geometry.cellCount + 1);
    cellVolumesM3_.resize(geometry.cellCount);
    inverseCellVolumesM3_.resize(geometry.cellCount);
    hydraulicDiametersM_.resize(geometry.cellCount);
    turbulentRoughnessTerms_.resize(geometry.cellCount);
    wallHeatConductancePerVolumes_.resize(geometry.cellCount);
    cellLengthM_ = geometry.cellLengthM();
    localLossGradientPerM_ =
        geometry.localLossCoefficient / geometry.lengthM;
    for (std::size_t face = 0; face <= geometry.cellCount; ++face)
        faceAreasM2_[face] = geometry.faceAreaM2(face);
    for (std::size_t index = 0; index < geometry.cellCount; ++index) {
        const auto volumeM3 = geometry.cellVolumeM3(index);
        const auto diameterM = geometry.cellHydraulicDiameterM(index);
        cellVolumesM3_[index] = volumeM3;
        inverseCellVolumesM3_[index] = 1.0 / volumeM3;
        hydraulicDiametersM_[index] = diameterM;
        turbulentRoughnessTerms_[index] = std::pow(
            geometry.absoluteRoughnessM / diameterM / 3.7, 1.11);
        wallHeatConductancePerVolumes_[index] =
            geometry.wallHeatTransferWPerM2K * (4.0 / diameterM);
        auto wallGeometry =
            DuctWallHeatTransferModel::prepareGeometry(
                diameterM,
                cellLengthM_,
                geometry.wallThicknessM,
                geometry.wallDensityKgPerM3,
                geometry.wallSpecificHeatJPerKgK,
                geometry.externalWallHeatTransferWPerM2K);
        if (geometry.homogenisedCellularSubstrate) {
            // One state represents all parallel channels in this axial cell.
            // Their aggregate wetted perimeter is 4*A_open/Dh. The solid
            // substrate capacity comes from the authored volumetric value;
            // the outer metal can retains the network's wall material.
            const auto openAreaM2 = geometry.cellAreaM2(index);
            const auto housingAreaM2 = openAreaM2
                / geometry.cellularSubstrateOpenAreaRatio;
            const auto housingRadiusM = std::sqrt(
                housingAreaM2 / std::numbers::pi);
            const auto canOuterRadiusM = housingRadiusM
                + geometry.wallThicknessM;
            const auto substrateSolidAreaM2 = std::max(
                0.0, housingAreaM2 - openAreaM2);
            wallGeometry.innerAreaM2 = 4.0 * openAreaM2
                / diameterM * cellLengthM_;
            wallGeometry.wallHeatCapacityJPerK = cellLengthM_ * (
                substrateSolidAreaM2
                    * geometry.cellularSubstrateVolumetricHeatCapacityJPerM3K
                + std::numbers::pi
                    * (canOuterRadiusM * canOuterRadiusM
                        - housingRadiusM * housingRadiusM)
                    * geometry.wallDensityKgPerM3
                    * geometry.wallSpecificHeatJPerKgK);
            wallGeometry.externalConductanceWPerK =
                geometry.externalWallHeatTransferWPerM2K
                * 2.0 * std::numbers::pi * canOuterRadiusM * cellLengthM_;
        }
        wallHeatTransferGeometries_[index] = wallGeometry;
        if (geometry.dynamicWallHeatTransferEnabled
            && !wallHeatTransferGeometries_[index].valid())
            return false;
    }
    cellStateCacheIsValid_ = prepareStateCache(cells_, cellPrimitives_, cellSourceTerms_,
        maximumCellSignalSpeedMps_, cellSourceLimitedTimeStepSeconds_);
    return cellStateCacheIsValid_;
}

double FiniteVolumeDuct::cellCentreM(std::size_t index) const noexcept {
    if (index >= cells_.size()) return geometry_.lengthM;
    return (static_cast<double>(index) + 0.5) * cellLengthM_;
}

bool FiniteVolumeDuct::meanAcousticMedium(double& densityKgPerM3,
                                          double& speedOfSoundMps) const noexcept {
    const auto primitives = cellPrimitives();
    if (primitives.empty()) return false;
    auto densitySum = 0.0;
    auto soundSpeedSum = 0.0;
    for (const auto& primitive : primitives) {
        if (!finite(primitive.densityKgPerM3) || !(primitive.densityKgPerM3 > 0.0)
            || !finite(primitive.speedOfSoundMps) || !(primitive.speedOfSoundMps > 0.0))
            return false;
        densitySum += primitive.densityKgPerM3;
        soundSpeedSum += primitive.speedOfSoundMps;
    }
    // Cells are uniform in length, so an unweighted mean is the length mean.
    const auto count = static_cast<double>(primitives.size());
    densityKgPerM3 = densitySum / count;
    speedOfSoundMps = soundSpeedSum / count;
    return true;
}

ConservedInventory FiniteVolumeDuct::inventory() const noexcept {
    ConservedInventory result;
    for (std::size_t cellIndex = 0; cellIndex < cells_.size(); ++cellIndex) {
        const auto& cell = cells_[cellIndex];
        const auto cellVolume = cellVolumesM3_[cellIndex];
        for (std::size_t index = 0; index < gasSpeciesCount; ++index)
            result.speciesMassKg[index] +=
                cell.speciesMassDensityKgPerM3[index] * cellVolume;
        result.axialMomentumKgMps += cell.momentumDensityKgPerM2S * cellVolume;
        result.totalEnergyJ += cell.totalEnergyDensityJPerM3 * cellVolume;
    }
    return result;
}

double FiniteVolumeDuct::wallThermalEnergyJ() const noexcept {
    if (!geometry_.dynamicWallHeatTransferEnabled) return 0.0;
    auto energyJ = 0.0;
    for (std::size_t index = 0; index < wallStates_.size(); ++index) {
        energyJ += wallHeatTransferGeometries_[index].wallHeatCapacityJPerK
            * wallStates_[index].temperatureK;
    }
    return energyJ;
}

void FiniteVolumeDuct::resetWallTemperature(double temperatureK) noexcept {
    const auto safeTemperatureK = finite(temperatureK) && temperatureK > 0.0
        ? temperatureK : geometry_.wallTemperatureK;
    for (auto& wall : wallStates_) wall.temperatureK = safeTemperatureK;
    wallHeatPendingSeconds_ = 0.0;
}

bool FiniteVolumeDuct::applyDynamicWallHeatTransfer(
    std::span<ConservativeState> states,
    std::span<const PrimitiveState> primitives,
    std::span<DuctWallThermalState> wallStates,
    double durationSeconds,
    double& heatRejectedJ) const noexcept {
    if (!geometry_.dynamicWallHeatTransferEnabled || durationSeconds == 0.0)
        return true;
    if (states.size() != cells_.size()
        || wallStates.size() != states.size()
        || wallHeatTransferGeometries_.size() != states.size()
        || primitives.size() != states.size()
        || !finite(durationSeconds) || !(durationSeconds > 0.0))
        return false;

    for (std::size_t index = 0; index < states.size(); ++index) {
        const auto& primitive = primitives[index];
        const auto cellVolumeM3 = cellVolumesM3_[index];
        const auto specificGasConstantJPerKgK = primitive.pressurePa
            / (primitive.densityKgPerM3 * primitive.temperatureK);
        const auto specificHeatCvJPerKgK = specificGasConstantJPerKgK
            / (primitive.heatCapacityRatio - 1.0);
        const auto result = DuctWallHeatTransferModel::advancePrepared(
            wallStates[index], wallHeatTransferGeometries_[index], {
                hydraulicDiametersM_[index],
                cellLengthM_,
                geometry_.wallThicknessM,
                geometry_.wallDensityKgPerM3,
                geometry_.wallSpecificHeatJPerKgK,
                geometry_.externalWallHeatTransferWPerM2K,
                geometry_.externalTemperatureK,
                primitive.densityKgPerM3,
                primitive.velocityMps,
                specificHeatCvJPerKgK + specificGasConstantJPerKgK,
                primitive.densityKgPerM3 * cellVolumeM3 * specificHeatCvJPerKgK,
                primitive.temperatureK,
                durationSeconds,
            });
        if (!(result.wallHeatCapacityJPerK > 0.0)
            || !finite(result.heatToGasJ) || !finite(result.heatRejectedJ))
            return false;
        states[index].totalEnergyDensityJPerM3 +=
            result.heatToGasJ / cellVolumeM3;
        heatRejectedJ += result.heatRejectedJ;
    }
    return true;
}

double FiniteVolumeDuct::maximumStableTimeStep(double maximumCourantNumber) const noexcept {
    if (cells_.empty() || !finite(maximumCourantNumber)
        || !(maximumCourantNumber > 0.0) || !(maximumCourantNumber <= 1.0))
        return 0.0;
    if (!refreshCellStateCache()) return 0.0;
    if (!(maximumCellSignalSpeedMps_ > 0.0)
        || !finite(maximumCellSignalSpeedMps_)
        || !(cellSourceLimitedTimeStepSeconds_ > 0.0))
        return 0.0;
    return std::min(maximumCourantNumber * cellLengthM_
                        / maximumCellSignalSpeedMps_,
                    cellSourceLimitedTimeStepSeconds_);
}

bool FiniteVolumeDuct::prepareStateCache(
    std::span<const ConservativeState> states,
    std::span<PrimitiveState> primitives,
    std::span<ConservativeState> sourceTerms,
    double& maximumSignalSpeed,
    double& sourceLimitedTimeStep) const noexcept {
    if (states.size() != primitives.size() || states.size() != sourceTerms.size())
        return false;
    maximumSignalSpeed = 0.0;
    sourceLimitedTimeStep = std::numeric_limits<double>::infinity();
    constexpr double referenceViscosityPaS = 1.716e-5;
    constexpr double referenceTemperatureK = 273.15;
    constexpr double sutherlandTemperatureK = 110.4;
    for (std::size_t index = 0; index < states.size(); ++index) {
        const auto& state = states[index];
        auto& primitive = primitives[index];
        auto& source = sourceTerms[index];
        const auto hydraulicDiameterM = hydraulicDiametersM_[index];
        const auto turbulentRoughnessTerm =
            turbulentRoughnessTerms_[index];
        const auto wallHeatConductancePerVolume =
            wallHeatConductancePerVolumes_[index];
        source = {};
        if (!mixtureModel_.recoverPrimitive(state, primitive)) return false;
        maximumSignalSpeed = std::max(maximumSignalSpeed,
            std::abs(primitive.velocityMps) + primitive.speedOfSoundMps);

        if (geometry_.wallFrictionEnabled
            && std::abs(primitive.velocityMps) > 1.0e-9) {
            const auto temperatureRatio = primitive.temperatureK
                / referenceTemperatureK;
            const auto viscosity = referenceViscosityPaS
                * temperatureRatio * std::sqrt(temperatureRatio)
                * (referenceTemperatureK + sutherlandTemperatureK)
                / (primitive.temperatureK + sutherlandTemperatureK);
            const auto reynolds = primitive.densityKgPerM3
                * std::abs(primitive.velocityMps) * hydraulicDiameterM
                / std::max(1.0e-12, viscosity);
            auto frictionFactor = 0.0;
            if (reynolds > 1.0) {
                if (reynolds < 2'300.0) {
                    frictionFactor = 64.0 / reynolds;
                } else {
                    const auto inverseRoot = -1.8 * std::log10(
                        turbulentRoughnessTerm + 6.9 / reynolds);
                    frictionFactor = 1.0 / (inverseRoot * inverseRoot);
                }
            }
            const auto lossGradient = frictionFactor / hydraulicDiameterM
                + localLossGradientPerM_;
            source.momentumDensityKgPerM2S = -0.5 * lossGradient
                * primitive.densityKgPerM3 * primitive.velocityMps
                * std::abs(primitive.velocityMps);
            if (std::abs(source.momentumDensityKgPerM2S) > 1.0e-12) {
                sourceLimitedTimeStep = std::min(sourceLimitedTimeStep,
                    0.5 * std::abs(state.momentumDensityKgPerM2S
                                   / source.momentumDensityKgPerM2S));
            }
        }

        if (wallHeatConductancePerVolume > 0.0) {
            source.totalEnergyDensityJPerM3 = wallHeatConductancePerVolume
                * (geometry_.wallTemperatureK - primitive.temperatureK);
            if (source.totalEnergyDensityJPerM3 < -1.0e-12) {
                const auto kineticEnergy = 0.5 * primitive.densityKgPerM3
                    * primitive.velocityMps * primitive.velocityMps;
                const auto internalEnergy = state.totalEnergyDensityJPerM3 - kineticEnergy;
                sourceLimitedTimeStep = std::min(sourceLimitedTimeStep,
                    0.5 * internalEnergy / -source.totalEnergyDensityJPerM3);
            }
        }
    }
    if (!(maximumSignalSpeed > 0.0) || !finite(maximumSignalSpeed)
        || !(sourceLimitedTimeStep > 0.0))
        return false;
    return true;
}

bool FiniteVolumeDuct::recoverPrimitiveStates(
    std::span<const ConservativeState> states,
    std::span<PrimitiveState> primitives) const noexcept {
    if (states.size() != primitives.size()) return false;
    for (std::size_t index = 0; index < states.size(); ++index)
        if (!mixtureModel_.recoverPrimitive(states[index], primitives[index]))
            return false;
    return true;
}

bool FiniteVolumeDuct::refreshCellStateCache() const noexcept {
    if (cellStateCacheIsValid_) return true;
    cellStateCacheIsValid_ = prepareStateCache(
        cells_, cellPrimitives_, cellSourceTerms_, maximumCellSignalSpeedMps_,
        cellSourceLimitedTimeStepSeconds_);
    return cellStateCacheIsValid_;
}

bool FiniteVolumeDuct::computeResidual(
    std::span<const ConservativeState> states,
    std::span<const PrimitiveState> primitives,
    std::span<const ConservativeState> sourceTerms,
    const DuctBoundaryCondition& leftBoundary,
    const DuctBoundaryCondition& rightBoundary,
    std::span<ConservativeState> residual,
    std::span<EulerFlux> faceFluxes,
    bool computeBoundaryFluxes) noexcept {
    const auto count = states.size();
    if (primitives.size() != count || sourceTerms.size() != count
        || residual.size() != count
        || faceFluxes.size() != count + 1)
        return false;
    const auto periodic = leftBoundary.type == DuctBoundaryType::periodic;
    if (periodic) {
        for (std::size_t index = 0; index < count; ++index) {
            const auto previous = index == 0 ? count - 1 : index - 1;
            const auto next = index + 1 == count ? 0 : index + 1;
            slopes_[index] = monotonisedCentralSlope(
                states[previous], states[index], states[next]);
        }
    } else {
        slopes_.front() = {};
        if (count > 1) slopes_.back() = {};
        for (std::size_t index = 1; index + 1 < count; ++index)
            slopes_[index] = monotonisedCentralSlope(
                states[index - 1], states[index], states[index + 1]);
    }

    // This block -- the limiter, the two reconstructions and the two primitive
    // recoveries they need -- is 16-29% of the duct solver depending on cell
    // count, measured by forcing the slopes to zero. That is the largest single
    // item left, and it is NOT available: zeroing the slopes is exactly
    // dropping the scheme to first order, and the runner mesh is already known
    // not to be converged (see the mesh note in CLAUDE.md).
    for (std::size_t index = 0; index < count; ++index) {
        if (isZero(slopes_[index])) {
            reconstructedLeft_[index] = states[index];
            reconstructedRight_[index] = states[index];
            reconstructedLeftPrimitives_[index] = primitives[index];
            reconstructedRightPrimitives_[index] = primitives[index];
            continue;
        }
        reconstructedLeft_[index] = addScaled(states[index], slopes_[index], -0.5);
        if (!mixtureModel_.recoverPrimitive(
                reconstructedLeft_[index], reconstructedLeftPrimitives_[index])) {
            reconstructedLeft_[index] = states[index];
            reconstructedLeftPrimitives_[index] = primitives[index];
        }
        reconstructedRight_[index] = addScaled(states[index], slopes_[index], 0.5);
        if (!mixtureModel_.recoverPrimitive(
                reconstructedRight_[index], reconstructedRightPrimitives_[index])) {
            reconstructedRight_[index] = states[index];
            reconstructedRightPrimitives_[index] = primitives[index];
        }
    }

    if (periodic) {
        faceFluxes[0] = mixtureModel_.riemannFluxPrepared(
            reconstructedRight_[count - 1], reconstructedRightPrimitives_[count - 1],
            reconstructedLeft_[0], reconstructedLeftPrimitives_[0]);
        faceFluxes[count] = faceFluxes[0];
    } else if (computeBoundaryFluxes) {
        const auto& insideLeft = reconstructedLeft_[0];
        const auto& insideLeftPrimitive = reconstructedLeftPrimitives_[0];
        auto outsideLeft = insideLeft;
        auto outsideLeftPrimitive = insideLeftPrimitive;
        if (leftBoundary.type == DuctBoundaryType::reflective) {
            outsideLeft = mirrored(insideLeft);
            outsideLeftPrimitive = mirrored(insideLeftPrimitive);
        } else if (leftBoundary.type == DuctBoundaryType::prescribed) {
            outsideLeft = leftBoundary.prescribedState;
            if (!mixtureModel_.recoverPrimitive(outsideLeft, outsideLeftPrimitive)) return false;
        }
        faceFluxes[0] = mixtureModel_.riemannFluxPrepared(
            outsideLeft, outsideLeftPrimitive, insideLeft, insideLeftPrimitive);

        const auto& insideRight = reconstructedRight_[count - 1];
        const auto& insideRightPrimitive = reconstructedRightPrimitives_[count - 1];
        auto outsideRight = insideRight;
        auto outsideRightPrimitive = insideRightPrimitive;
        if (rightBoundary.type == DuctBoundaryType::reflective) {
            outsideRight = mirrored(insideRight);
            outsideRightPrimitive = mirrored(insideRightPrimitive);
        } else if (rightBoundary.type == DuctBoundaryType::prescribed) {
            outsideRight = rightBoundary.prescribedState;
            if (!mixtureModel_.recoverPrimitive(outsideRight, outsideRightPrimitive)) return false;
        }
        faceFluxes[count] = mixtureModel_.riemannFluxPrepared(
            insideRight, insideRightPrimitive, outsideRight, outsideRightPrimitive);
    } else {
        // ExhaustGasNetwork owns every graph endpoint and replaces both end
        // fluxes immediately after this call. Starting from zero avoids two
        // transmissive Riemann solves per duct and makes assignDuctBoundary()
        // add the physical flux directly. The network's assigned-endpoint gate
        // still fails the stage if a graph boundary was omitted.
        faceFluxes[0] = {};
        faceFluxes[count] = {};
    }

    for (std::size_t face = 1; face < count; ++face) {
        faceFluxes[face] = mixtureModel_.riemannFluxPrepared(
            reconstructedRight_[face - 1], reconstructedRightPrimitives_[face - 1],
            reconstructedLeft_[face], reconstructedLeftPrimitives_[face]);
    }

    for (std::size_t index = 0; index < count; ++index) {
        auto& cellResidual = residual[index];
        const auto leftAreaM2 = faceAreasM2_[index];
        const auto rightAreaM2 = faceAreasM2_[index + 1];
        const auto inverseCellVolumeM3 =
            inverseCellVolumesM3_[index];
        for (std::size_t species = 0; species < gasSpeciesCount; ++species) {
            cellResidual.speciesMassDensityKgPerM3[species] =
                -inverseCellVolumeM3
                * (rightAreaM2
                        * faceFluxes[index + 1]
                              .speciesMassFluxKgPerM2S[species]
                   - leftAreaM2
                        * faceFluxes[index]
                              .speciesMassFluxKgPerM2S[species]);
        }
        cellResidual.momentumDensityKgPerM2S =
            -inverseCellVolumeM3
                * (rightAreaM2 * faceFluxes[index + 1].momentumFluxPa
                   - leftAreaM2 * faceFluxes[index].momentumFluxPa)
            // Quasi-1D wall pressure force. It cancels the pressure part of
            // the face-flux divergence exactly for a stationary uniform gas.
            + primitives[index].pressurePa
                * (rightAreaM2 - leftAreaM2) * inverseCellVolumeM3;
        cellResidual.totalEnergyDensityJPerM3 =
            -inverseCellVolumeM3
            * (rightAreaM2
                    * faceFluxes[index + 1].totalEnergyFluxWPerM2
               - leftAreaM2
                    * faceFluxes[index].totalEnergyFluxWPerM2);

        cellResidual.momentumDensityKgPerM2S +=
            sourceTerms[index].momentumDensityKgPerM2S;
        cellResidual.totalEnergyDensityJPerM3 +=
            sourceTerms[index].totalEnergyDensityJPerM3;
    }
    return true;
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
        || (leftPeriodic && geometry_.hasVariableArea())
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
            if (!computeResidual(cells_, cellPrimitives_, cellSourceTerms_, left, right,
                                 residual_, faceFluxes_)) {
                result.completed = false;
                break;
            }
            for (std::size_t index = 0; index < cells_.size(); ++index)
                stage_[index] = addScaled(cells_[index], residual_[index], trialStep);
            const auto stageRoundoffIsValid = std::all_of(
                stage_.begin(), stage_.end(), [this](ConservativeState& state) {
                    return mixtureModel_.canonicaliseSpeciesRoundoff(state);
                });
            if (!stageRoundoffIsValid
                || !prepareStateCache(stage_, stagePrimitives_, stageSourceTerms_,
                    maximumStageSignalSpeedMps_,
                    stageSourceLimitedTimeStepSeconds_)) {
                ++result.rejectedSubsteps;
                trialStep *= 0.5;
                if (!(trialStep > std::numeric_limits<double>::epsilon()
                                  * std::max(1.0, durationSeconds)))
                    break;
                continue;
            }

            if (!computeResidual(stage_, stagePrimitives_, stageSourceTerms_, left, right,
                                 stageResidual_, stageFaceFluxes_)) {
                result.completed = false;
                break;
            }
            for (std::size_t index = 0; index < cells_.size(); ++index) {
                const auto forwardEuler = addScaled(stage_[index], stageResidual_[index], trialStep);
                candidate_[index] = addScaled(cells_[index],
                    difference(forwardEuler, cells_[index]), 0.5);
            }
            const auto candidateRoundoffIsValid = std::all_of(
                candidate_.begin(), candidate_.end(), [this](ConservativeState& state) {
                    return mixtureModel_.canonicaliseSpeciesRoundoff(state);
                });
            auto candidateHeatRejectedJ = 0.0;
            auto candidateIsValid = candidateRoundoffIsValid;
            // Decided from `trialStep` before any work, so a halved retry
            // re-decides consistently within its own attempt and nothing is
            // committed until the sub-step is accepted.
            const auto wallPendingSeconds = wallHeatPendingSeconds_ + trialStep;
            const auto applyWallHeat = geometry_.dynamicWallHeatTransferEnabled
                && wallPendingSeconds >= geometry_.wallHeatUpdateIntervalSeconds;
            if (candidateIsValid && applyWallHeat) {
                candidateIsValid = recoverPrimitiveStates(
                    candidate_, candidatePrimitives_);
                if (candidateIsValid) {
                    candidateWallStates_ = wallStates_;
                    candidateIsValid = applyDynamicWallHeatTransfer(
                        candidate_, candidatePrimitives_, candidateWallStates_,
                        wallPendingSeconds, candidateHeatRejectedJ);
                }
            }
            if (candidateIsValid) {
                candidateIsValid = prepareStateCache(
                    candidate_, candidatePrimitives_, candidateSourceTerms_,
                    maximumCandidateSignalSpeedMps_,
                    candidateSourceLimitedTimeStepSeconds_);
            }
            if (!candidateIsValid) {
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
            cellPrimitives_.swap(candidatePrimitives_);
            cellSourceTerms_.swap(candidateSourceTerms_);
            std::swap(maximumCellSignalSpeedMps_, maximumCandidateSignalSpeedMps_);
            std::swap(cellSourceLimitedTimeStepSeconds_,
                      candidateSourceLimitedTimeStepSeconds_);
            cellStateCacheIsValid_ = true;
            if (applyWallHeat) {
                wallStates_.swap(candidateWallStates_);
                result.wallHeatRejectedJ += candidateHeatRejectedJ;
                wallHeatPendingSeconds_ = 0.0;
            } else {
                wallHeatPendingSeconds_ = wallPendingSeconds;
            }
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
