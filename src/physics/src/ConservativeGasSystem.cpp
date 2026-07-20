#include <enginelab/physics/ConservativeGasSystem.hpp>

#include <limits>
#include <numbers>

namespace enginelab {
namespace {
constexpr double minimumVolumeM3 = 1.0e-8;
constexpr double minimumTemperatureK = 80.0;
// Public initialisation/injection inputs are bounded, but the temperature
// derived from conserved energy is deliberately not capped.
constexpr double maximumInputTemperatureK = 5'000.0;
// Gamma for plain air (diatomic, 5 dof)
constexpr double gammaAir = 1.4;
// Gamma for fully burned combustion products (CO2/H2O mixture, ~4.25 dof effective)
constexpr double gammaBurned = 1.26;
constexpr double gasolineEnergyJPerKg = 44'000'000.0;

[[nodiscard]] GasMixture atmosphericComposition(double totalMoles) noexcept {
    return { totalMoles * 0.21, totalMoles * 0.79, 0.0, 0.0 };
}

/**
 * Isentropic mass-flow function Ψ(P_d/P_u, γ).
 *
 * Returns the dimensionless flow function  Ψ  defined by:
 *   ṁ = Cd · A · P_u / √(R_s · T_u) · Ψ
 *
 * where R_s = R / M̄ is the specific gas constant of the source.
 * Both choked and sub-critical (Fanno) regimes are handled.
 */
[[nodiscard]] double massFlowKgPerSecond(double upstreamPressurePa,
                                         double downstreamPressurePa,
                                         double temperatureK,
                                         double areaM2,
                                         double coefficient,
                                         bool& choked,
                                         double gamma,
                                         double specificGasConstant) noexcept {
    if (!(upstreamPressurePa > downstreamPressurePa) || areaM2 <= 0.0 || coefficient <= 0.0)
        return 0.0;
    const auto ratio = std::clamp(downstreamPressurePa / upstreamPressurePa, 0.0, 1.0);
    const auto critical = std::pow(2.0 / (gamma + 1.0), gamma / (gamma - 1.0));
    choked = ratio <= critical;
    double flowFunction = 0.0;
    if (choked) {
        flowFunction = std::sqrt(gamma)
            * std::pow(2.0 / (gamma + 1.0), (gamma + 1.0) / (2.0 * (gamma - 1.0)));
    } else {
        const auto term = 2.0 * gamma / (gamma - 1.0)
            * (std::pow(ratio, 2.0 / gamma) - std::pow(ratio, (gamma + 1.0) / gamma));
        flowFunction = std::sqrt(std::max(0.0, term));
    }
    return coefficient * areaM2 * upstreamPressurePa
        / std::sqrt(specificGasConstant * std::max(minimumTemperatureK, temperatureK)) * flowFunction;
}

/**
 * Inject jet momentum into a cell after mass transfer at a throat.
 *
 * When gas flows through a restriction at throat velocity v_jet, the pressure
 * gradient accelerates the resolved bulk gas on both sides in the direction
 * of flow.  The equal-and-opposite reaction is borne by the duct walls, not by
 * an artificial recoil of the upstream gas control volume.
 *
 * The jet velocity is capped at the speed of sound of the source cell so
 * that we never inject supersonic momentum from a single throat event.
 */
void injectJetMomentum(GasCell& source, GasCell& sink,
                       double movedMassKg,
                       double massFlowRate,
                       double throatAreaM2,
                       double sourceCharAreaM2,
                       double sinkCharAreaM2,
                       double dirX, double dirY) noexcept {
    if (movedMassKg <= 0.0 || throatAreaM2 <= 0.0) return;

    // Density at source to estimate jet velocity at throat
    const auto sourceDensity = std::max(0.01, source.massKg()
        / std::max(minimumVolumeM3, source.volumeM3()));
    const auto sourceC = source.speedOfSoundMps();
    const auto sinkDensity = std::max(0.01, sink.massKg()
        / std::max(minimumVolumeM3, sink.volumeM3()));
    const auto sinkC = sink.speedOfSoundMps();

    // Characteristic-volume velocities from continuity: v = mass flow / (rho * area).
    // The throat controls mass flow; each adjoining section controls how much
    // of that jet appears as resolved bulk momentum in its 0-D volume.
    const auto sourceVelocity = sourceCharAreaM2 > 0.0
        ? std::clamp(massFlowRate / std::max(1.0e-12, sourceDensity * sourceCharAreaM2),
                     0.0, sourceC) : 0.0;
    const auto sinkVelocity = sinkCharAreaM2 > 0.0
        ? std::clamp(massFlowRate / std::max(1.0e-12, sinkDensity * sinkCharAreaM2),
                     0.0, sinkC) : 0.0;
    const auto activeSections = static_cast<double>(sourceCharAreaM2 > 0.0)
        + static_cast<double>(sinkCharAreaM2 > 0.0);
    const auto jetMomentum = movedMassKg * (sourceVelocity + sinkVelocity)
        / std::max(1.0, activeSections);

    if (sourceCharAreaM2 <= 0.0 && sinkCharAreaM2 <= 0.0) return;
    const auto length = std::hypot(dirX, dirY);
    const auto nx = length > 1.0e-12 ? dirX / length : 1.0;
    const auto ny = length > 1.0e-12 ? dirY / length : 0.0;
    source.addMomentumKgMps(jetMomentum * nx, jetMomentum * ny);
    sink.addMomentumKgMps(jetMomentum * nx, jetMomentum * ny);
}
} // anonymous namespace

// ---------------------------------------------------------------------------
// GasCell methods
// ---------------------------------------------------------------------------

void GasCell::initialise(double pressureKpa, double volumeLitres, double temperatureK,
                         GasMixture composition) noexcept {
    volumeM3_ = std::max(minimumVolumeM3, volumeLitres * 0.001);
    const auto safePressurePa = std::max(10.0, pressureKpa * 1'000.0);
    const auto safeTemperature = std::clamp(
        temperatureK, minimumTemperatureK, maximumInputTemperatureK);
    const auto requestedMoles = safePressurePa * volumeM3_ / (universalGasConstant * safeTemperature);
    const auto specifiedMoles = composition.totalMoles();
    if (specifiedMoles <= 1.0e-12) mixture_ = atmosphericComposition(requestedMoles);
    else {
        const auto scale = requestedMoles / specifiedMoles;
        mixture_ = { composition.oxygenMoles * scale, composition.inertMoles * scale,
                     composition.fuelMoles * scale, composition.burnedMoles * scale };
    }
    internalEnergyJ_ = requestedMoles * molarHeatCapacityCvEffective() * safeTemperature;
    momentumXKgMps_ = 0.0;
    momentumYKgMps_ = 0.0;
}

void GasCell::reset(double pressureKpa, double temperatureK, GasMixture composition) noexcept {
    initialise(pressureKpa, volumeLitres(), temperatureK, composition);
}

void GasCell::setGeometry(double characteristicAreaM2, double dx, double dy) noexcept {
    characteristicAreaM2_ = std::max(0.0, characteristicAreaM2);
    const auto len = std::sqrt(dx * dx + dy * dy);
    if (len > 1.0e-9) { dx_ = dx / len; dy_ = dy / len; }
    else { dx_ = 1.0; dy_ = 0.0; }
}

void GasCell::setVolumeAdiabatic(double volumeLitres) noexcept {
    const auto newVolume = std::max(minimumVolumeM3, volumeLitres * 0.001);
    if (std::abs(newVolume - volumeM3_) <= std::numeric_limits<double>::epsilon()) return;
    const auto gamma = heatCapacityRatioEffective();
    const auto ratio = volumeM3_ / newVolume;
    // Exact finite change for a closed ideal-gas control mass with frozen
    // composition: U2/U1 = (V1/V2)^(gamma-1).  A trapezoidal P dV estimate
    // violates P*V^gamma for the large volume changes of an engine cylinder.
    internalEnergyJ_ = std::max(0.0,
        internalEnergyJ_ * std::pow(ratio, gamma - 1.0));
    volumeM3_ = newVolume;
}

void GasCell::addHeatJoules(double joules) noexcept {
    if (std::isfinite(joules)) internalEnergyJ_ = std::max(0.0, internalEnergyJ_ + joules);
}

bool GasCell::tryApplyInventoryDelta(const GasInventoryDelta& delta) noexcept {
    auto candidateMixture = GasMixture {
        mixture_.oxygenMoles + delta.mixture.oxygenMoles,
        mixture_.inertMoles + delta.mixture.inertMoles,
        mixture_.fuelMoles + delta.mixture.fuelMoles,
        mixture_.burnedMoles + delta.mixture.burnedMoles,
    };
    auto candidateEnergy = internalEnergyJ_ + delta.internalEnergyJ;
    const auto candidateMomentumX = momentumXKgMps_ + delta.momentumXKgMps;
    const auto candidateMomentumY = momentumYKgMps_ + delta.momentumYKgMps;
    if (!std::isfinite(candidateMixture.oxygenMoles)
        || !std::isfinite(candidateMixture.inertMoles)
        || !std::isfinite(candidateMixture.fuelMoles)
        || !std::isfinite(candidateMixture.burnedMoles)
        || !std::isfinite(candidateEnergy)
        || !std::isfinite(candidateMomentumX)
        || !std::isfinite(candidateMomentumY))
        return false;

    const auto totalMoles = candidateMixture.totalMoles();
    const auto moleTolerance = 128.0 * std::numeric_limits<double>::epsilon()
        * std::max(1.0e-15, totalMoles);
    const auto significantNegative = [moleTolerance](double moles) noexcept {
        return moles < -moleTolerance;
    };
    if (!(totalMoles > 1.0e-15)
        || significantNegative(candidateMixture.oxygenMoles)
        || significantNegative(candidateMixture.inertMoles)
        || significantNegative(candidateMixture.fuelMoles)
        || significantNegative(candidateMixture.burnedMoles))
        return false;
    const auto repairRoundoff = [](double value) noexcept {
        return value < 0.0 ? 0.0 : value;
    };
    candidateMixture.oxygenMoles = repairRoundoff(candidateMixture.oxygenMoles);
    candidateMixture.inertMoles = repairRoundoff(candidateMixture.inertMoles);
    candidateMixture.fuelMoles = repairRoundoff(candidateMixture.fuelMoles);
    candidateMixture.burnedMoles = repairRoundoff(candidateMixture.burnedMoles);
    const auto energyTolerance = 128.0 * std::numeric_limits<double>::epsilon()
        * std::max(1.0, internalEnergyJ_);
    if (candidateEnergy < -energyTolerance) return false;
    if (candidateEnergy < 0.0) candidateEnergy = 0.0;

    mixture_ = candidateMixture;
    internalEnergyJ_ = candidateEnergy;
    momentumXKgMps_ = candidateMomentumX;
    momentumYKgMps_ = candidateMomentumY;
    return true;
}

void GasCell::injectFuelMoles(double moles, double temperature) noexcept {
    const auto added = std::max(0.0, moles);
    mixture_.fuelMoles += added;
    constexpr double gasolineVapourCvJPerMolK = 145.0;
    internalEnergyJ_ += added * gasolineVapourCvJPerMolK
        * std::clamp(temperature, minimumTemperatureK, maximumInputTemperatureK);
}

void GasCell::configureFuelChemistry(double fuelMolarMassKg, double oxygenMolesPerFuelMole,
                                     double productMolesPerFuelMole) noexcept {
    fuelMolarMassKg_ = std::clamp(fuelMolarMassKg, 0.020, 0.300);
    oxygenMolesPerFuelMole_ = std::clamp(oxygenMolesPerFuelMole, 1.0, 40.0);
    productMolesPerFuelMole_ = std::clamp(productMolesPerFuelMole, 1.0, 60.0);
    burnedGasMolarMassKg_ = (fuelMolarMassKg_
        + oxygenMolesPerFuelMole_ * oxygenMolarMassKg) / productMolesPerFuelMole_;
}

void GasCell::applyFlowResistance(double lengthM, double hydraulicDiameterM,
                                  double absoluteRoughnessM, double localLossCoefficient,
                                  double dtSeconds) noexcept {
    const auto mass = massKg();
    const auto velocity = std::hypot(velocityXMps(), velocityYMps());
    if (!(mass > 1.0e-12) || !(velocity > 1.0e-9) || !(dtSeconds > 0.0)
            || !(lengthM > 0.0) || !(hydraulicDiameterM > 0.0))
        return;
    const auto density = mass / std::max(minimumVolumeM3, volumeM3_);
    const auto temperature = std::max(80.0, temperatureK());
    // Sutherland correlation for gas dynamic viscosity.
    constexpr double referenceViscosityPaS = 1.716e-5;
    constexpr double referenceTemperatureK = 273.15;
    constexpr double sutherlandTemperatureK = 110.4;
    const auto viscosity = referenceViscosityPaS
        * std::pow(temperature / referenceTemperatureK, 1.5)
        * (referenceTemperatureK + sutherlandTemperatureK)
        / (temperature + sutherlandTemperatureK);
    const auto reynolds = density * velocity * hydraulicDiameterM
        / std::max(1.0e-9, viscosity);
    double frictionFactor = 0.0;
    if (reynolds > 1.0) {
        if (reynolds < 2'300.0) {
            frictionFactor = 64.0 / reynolds;
        } else {
            // Haaland explicit approximation of Colebrook-White.
            const auto inverseRoot = -1.8 * std::log10(
                std::pow(std::max(0.0, absoluteRoughnessM) / hydraulicDiameterM / 3.7, 1.11)
                + 6.9 / reynolds);
            frictionFactor = 1.0 / (inverseRoot * inverseRoot);
        }
    }
    const auto lossCoefficient = std::max(0.0, localLossCoefficient)
        + frictionFactor * lengthM / hydraulicDiameterM;
    const auto area = characteristicAreaM2_ > 0.0
        ? characteristicAreaM2_ : volumeM3_ / lengthM;
    const auto pressureLossPa = lossCoefficient * 0.5 * density * velocity * velocity;
    const auto resistingImpulse = pressureLossPa * area * dtSeconds;
    const auto momentumMagnitude = mass * velocity;
    const auto retained = 1.0 - std::clamp(
        resistingImpulse / momentumMagnitude, 0.0, 1.0);
    const auto kineticBefore = bulkKineticEnergyJoules();
    momentumXKgMps_ *= retained;
    momentumYKgMps_ *= retained;
    internalEnergyJ_ += std::max(0.0, kineticBefore - bulkKineticEnergyJoules());
}

double GasCell::pressureKpa() const noexcept {
    const auto n = totalMoles();
    if (n <= 1.0e-15 || volumeM3_ <= minimumVolumeM3) return 0.0;
    return n * universalGasConstant * temperatureK() / volumeM3_ / 1'000.0;
}

double GasCell::temperatureK() const noexcept {
    // Use effective Cv based on current gas composition.
    // Burned products have higher heat capacity (CO2: Cv ≈ 28.8 J/mol/K,
    // H2O: Cv ≈ 25.2 J/mol/K) compared to diatomic air (Cv = 20.8 J/mol/K).
    // Species-weighted heat capacity keeps fuel vapour and products distinct.
    const auto n = totalMoles();
    if (n <= 1.0e-15) return 0.0;
    const auto heatCapacity = n * molarHeatCapacityCvEffective();
    // Do not cap the derived temperature: internalEnergyJ_ is the conserved
    // sensible energy. Capping only this view would hide energy above the cap
    // and make pressure inconsistent with the cell's thermodynamic state.
    return std::max(0.0, internalEnergyJ_ / heatCapacity);
}

double GasCell::molarHeatCapacityCvEffective() const noexcept {
    const auto n = totalMoles();
    if (n <= 1.0e-15) return molarHeatCapacityCv;
    constexpr double burnedProductsCvJPerMolK = universalGasConstant / (gammaBurned - 1.0);
    constexpr double gasolineVapourCvJPerMolK = 145.0;
    const auto heatCapacity = (mixture_.oxygenMoles + mixture_.inertMoles) * molarHeatCapacityCv
        + mixture_.fuelMoles * gasolineVapourCvJPerMolK
        + mixture_.burnedMoles * burnedProductsCvJPerMolK;
    return std::clamp(heatCapacity / n, molarHeatCapacityCv, gasolineVapourCvJPerMolK);
}

double GasCell::heatCapacityRatioEffective() const noexcept {
    const auto n = totalMoles();
    if (n <= 1.0e-15) return gammaAir;
    // Exact thermodynamic identity gamma = 1 + R/Cv, evaluated on the current
    // species-mole-weighted Cv (molarHeatCapacityCvEffective). This is NOT a
    // linear blend in burned fraction: as combustion products (higher Cv)
    // replace air, Cv rises, so gamma falls smoothly from 1.40 (air) toward
    // ~1.26 (fully burned). The mixture Cv carries the non-linearity.
    return std::clamp(1.0 + universalGasConstant / molarHeatCapacityCvEffective(), 1.05, gammaAir);
}

double GasCell::speedOfSoundMps() const noexcept {
    const auto m = massKg();
    if (m <= 1.0e-12 || volumeM3_ <= minimumVolumeM3) return 340.0;
    const auto gamma = heatCapacityRatioEffective();
    const auto pressurePa = pressureKpa() * 1'000.0;
    const auto density = m / volumeM3_;
    return std::sqrt(std::max(1.0, gamma * pressurePa / density));
}

double GasCell::dynamicPressureKpa(double nx, double ny) const noexcept {
    const auto m = massKg();
    if (m <= 1.0e-12 || volumeM3_ <= minimumVolumeM3) return 0.0;
    // Bulk velocity is along (dx_, dy_); project onto query direction (nx, ny)
    const auto directionLength = std::hypot(nx, ny);
    if (directionLength <= 1.0e-12) return 0.0;
    const auto vDotN = velocityXMps() * nx / directionLength
        + velocityYMps() * ny / directionLength;
    if (std::abs(vDotN) <= 1.0e-12) return 0.0;
    const auto density = m / volumeM3_;
    // q = ½·ρ·|v_n|·v_n  (signed, in Pa → kPa)
    const auto gamma = heatCapacityRatioEffective();
    // pressureKpa() is a pure function of the (unchanged) cell state; evaluate it
    // once. It was previously called three times per invocation, and this method
    // is one of the two effective-pressure evaluations inside the flow solver's
    // inner bisection — the dominant realtime cost of the simulator.
    const auto staticPressureKpa = pressureKpa();
    const auto speedOfSoundSquared = std::max(1.0, gamma * staticPressureKpa * 1'000.0 / density);
    const auto machSquared = vDotN * vDotN / speedOfSoundSquared;
    const auto stagnationRatio = std::pow(1.0 + (gamma - 1.0) * 0.5 * machSquared,
                                          gamma / (gamma - 1.0));
    return std::copysign(staticPressureKpa * (stagnationRatio - 1.0), vDotN);
}

// ---------------------------------------------------------------------------
// ConservativeGasSystem — private helpers
// ---------------------------------------------------------------------------

GasFlowResult ConservativeGasSystem::transfer(GasCell& source, GasCell& sink, double moles) noexcept {
    GasFlowResult result;
    const auto totalEnergyBefore = totalEnergy(source, sink);
    const auto sourceMoles = source.totalMoles();
    const auto specificEnthalpy = (source.molarHeatCapacityCvEffective()
        + GasCell::universalGasConstant) * source.temperatureK();
    const auto thermalLimit = source.internalEnergyJ_ / std::max(1.0, specificEnthalpy) * 0.98;
    const auto amount = std::clamp(moles, 0.0, std::min(sourceMoles * 0.999, thermalLimit));
    if (amount <= 1.0e-15 || sourceMoles <= 1.0e-15) return result;
    const auto fraction = amount / sourceMoles;
    const GasMixture moved { source.mixture_.oxygenMoles * fraction,
                             source.mixture_.inertMoles  * fraction,
                             source.mixture_.fuelMoles   * fraction,
                             source.mixture_.burnedMoles * fraction };
    // Finite-volume energy flux transports stagnation enthalpy, not only u.
    const auto movedEnergy = amount * specificEnthalpy;
    const auto movedMass     = source.massKg()           * fraction;
    const auto movedMomentumX = source.momentumXKgMps_ * fraction;
    const auto movedMomentumY = source.momentumYKgMps_ * fraction;
    source.mixture_.oxygenMoles -= moved.oxygenMoles;
    source.mixture_.inertMoles  -= moved.inertMoles;
    source.mixture_.fuelMoles   -= moved.fuelMoles;
    source.mixture_.burnedMoles -= moved.burnedMoles;
    source.internalEnergyJ_ = std::max(0.0, source.internalEnergyJ_ - movedEnergy);
    source.momentumXKgMps_ -= movedMomentumX;
    source.momentumYKgMps_ -= movedMomentumY;
    sink.mixture_.oxygenMoles += moved.oxygenMoles;
    sink.mixture_.inertMoles  += moved.inertMoles;
    sink.mixture_.fuelMoles   += moved.fuelMoles;
    sink.mixture_.burnedMoles += moved.burnedMoles;
    sink.internalEnergyJ_  += movedEnergy;
    sink.momentumXKgMps_ += movedMomentumX;
    sink.momentumYKgMps_ += movedMomentumY;
    restorePairEnergy(source, sink, totalEnergyBefore);
    result.transferredMoles  = amount;
    result.transferredMassKg = movedMass;
    return result;
}

void ConservativeGasSystem::restorePairEnergy(GasCell& first, GasCell& second,
                                               double targetEnergyJoules) noexcept {
    const auto currentEnergy = totalEnergy(first, second);
    if (!std::isfinite(targetEnergyJoules) || !std::isfinite(currentEnergy)) return;
    auto correction = targetEnergyJoules - currentEnergy;
    if (correction >= 0.0) {
        second.internalEnergyJ_ += correction;
        return;
    }

    auto excess = -correction;
    const auto removeInternalEnergy = [&excess](GasCell& cell) noexcept {
        const auto removed = std::min(excess, cell.internalEnergyJ_);
        cell.internalEnergyJ_ -= removed;
        excess -= removed;
    };
    removeInternalEnergy(second);
    removeInternalEnergy(first);
    if (excess > 1.0e-12) {
        const auto kineticEnergy = first.bulkKineticEnergyJoules()
            + second.bulkKineticEnergyJoules();
        if (kineticEnergy > 0.0) {
            const auto targetKineticEnergy = std::max(0.0, kineticEnergy - excess);
            const auto scale = std::sqrt(targetKineticEnergy / kineticEnergy);
            first.momentumXKgMps_ *= scale;
            first.momentumYKgMps_ *= scale;
            second.momentumXKgMps_ *= scale;
            second.momentumYKgMps_ *= scale;
        }
    }
    correction = targetEnergyJoules - totalEnergy(first, second);
    if (correction > 0.0) second.internalEnergyJ_ += correction;
}

double ConservativeGasSystem::pressureEquilibriumMoles(const GasCell& source,
                                                         const GasCell& sink,
                                                         double directionX,
                                                         double directionY,
                                                         double requestedMoles,
                                                         bool includeDynamicPressure) noexcept {
    const auto effectivePressure = [includeDynamicPressure](const GasCell& cell,
                                                             double dx, double dy) noexcept {
        return cell.pressureKpa() + (includeDynamicPressure
            ? cell.dynamicPressureKpa(dx, dy) : 0.0);
    };
    const auto sourceEffective = effectivePressure(source, directionX, directionY);
    const auto sinkEffective = effectivePressure(sink, directionX, directionY);
    if (sourceEffective <= sinkEffective || source.totalMoles() <= 1.0e-15) return 0.0;
    auto low = 0.0;
    auto high = std::min(std::max(0.0, requestedMoles), source.totalMoles() * 0.95);
    auto sourceAtRequested = source;
    auto sinkAtRequested = sink;
    (void)transfer(sourceAtRequested, sinkAtRequested, high);
    const auto requestedSourcePressure = effectivePressure(
        sourceAtRequested, directionX, directionY);
    const auto requestedSinkPressure = effectivePressure(
        sinkAtRequested, directionX, directionY);
    if (requestedSourcePressure >= requestedSinkPressure) return high;
    // Keep `low` on the non-crossed side: a committed transfer may approach
    // equilibrium but must never reverse the pressure gradient.
    //
    // The bracket is bisected until either the moles are resolved finely
    // relative to the transfer being committed, or the 16-iteration ceiling is
    // reached. The fixed count previously refined every call to 2^-16 of the
    // initial bracket even when that precision could not change the downstream
    // gas state; each iteration costs two GasCell copies plus two effective-
    // pressure evaluations, and this bisection is the dominant realtime cost of
    // the whole simulator. Terminating at ~2^-8 of the initial bracket leaves a
    // residual far below the substep integration's sensitivity: combustion
    // phasing shifts by at most ~1-2 deg CA and stays inside the MBT window
    // (verified against EngineLab.CombustionPhasing and EngineLab.PhysicsRegression).
    const auto convergenceMoles = std::max(1.0e-15, high * 3.91e-3); // 2^-8 of initial bracket
    for (int iteration = 0; iteration < 16; ++iteration) {
        if (high - low <= convergenceMoles) break;
        const auto middle = 0.5 * (low + high);
        auto sourceCopy = source;
        auto sinkCopy = sink;
        (void)transfer(sourceCopy, sinkCopy, middle);
        const auto sourcePressure = effectivePressure(sourceCopy, directionX, directionY);
        const auto sinkPressure = effectivePressure(sinkCopy, directionX, directionY);
        if (sourcePressure > sinkPressure) low = middle;
        else high = middle;
    }
    return low;
}

// ---------------------------------------------------------------------------
// ConservativeGasSystem — massFlowKgPerSecond (static, parameterised)
// ---------------------------------------------------------------------------

double ConservativeGasSystem::massFlowKgPerSecond(double upstreamPressurePa,
                                                  double downstreamPressurePa,
                                                  double temperatureK,
                                                  double areaM2,
                                                  double coefficient,
                                                  bool& choked,
                                                  double gamma,
                                                  double specificGasConstant) noexcept {
    return ::enginelab::massFlowKgPerSecond(upstreamPressurePa, downstreamPressurePa,
        temperatureK, areaM2, coefficient, choked, gamma, specificGasConstant);
}

// ---------------------------------------------------------------------------
// ConservativeGasSystem::flow (simple backward-compatible overload)
// ---------------------------------------------------------------------------

GasFlowResult ConservativeGasSystem::flow(GasCell& first, GasCell& second,
                                          double area, double coefficient, double dt) noexcept {
    if (dt <= 0.0 || !std::isfinite(dt)) return {};
    auto* source = &first;
    auto* sink   = &second;
    if (second.pressureKpa() > first.pressureKpa()) std::swap(source, sink);
    bool choked = false;
    const auto gamma = source->heatCapacityRatioEffective();
    const auto specificR = GasCell::universalGasConstant / source->meanMolarMassKg();
    const auto massRate = massFlowKgPerSecond(source->pressureKpa() * 1'000.0,
        sink->pressureKpa() * 1'000.0, source->temperatureK(),
        area, coefficient, choked, gamma, specificR);
    const auto requestedMoles = massRate * dt / source->meanMolarMassKg();
    const auto directionX = source == &first ? source->orientationDx() : -source->orientationDx();
    const auto directionY = source == &first ? source->orientationDy() : -source->orientationDy();
    const auto equilibriumMoles = pressureEquilibriumMoles(*source, *sink, directionX, directionY,
                                                            requestedMoles, false);
    auto result = transfer(*source, *sink, std::min(requestedMoles, equilibriumMoles));
    // Jet momentum injection using the cells' stored geometry.
    // Falls back to raw throat velocity capped at 600 m/s for cells without
    // geometry, preserving the original behaviour for un-migrated call sites.
    const auto sourceCharArea = source->characteristicAreaM2_ > 0.0
        ? source->characteristicAreaM2_ : area;
    const auto sinkCharArea   = sink->characteristicAreaM2_ > 0.0
        ? sink->characteristicAreaM2_ : area;
    const auto energyBeforeJet = totalEnergy(*source, *sink);
    injectJetMomentum(*source, *sink, std::abs(result.transferredMassKg),
                      std::abs(result.transferredMassKg) / dt,
                      area, sourceCharArea, sinkCharArea,
                       directionX, directionY);
    restorePairEnergy(*source, *sink, energyBeforeJet);
    result.choked = choked;
    if (source == &second) {
        result.transferredMoles  = -result.transferredMoles;
        result.transferredMassKg = -result.transferredMassKg;
    }
    return result;
}

// ---------------------------------------------------------------------------
// ConservativeGasSystem::flow (FlowParameters variant — full physics)
// ---------------------------------------------------------------------------

GasFlowResult ConservativeGasSystem::flow(const FlowParameters& params) noexcept {
    if (!params.system0 || !params.system1) return {};
    if (params.dtSeconds <= 0.0 || !std::isfinite(params.dtSeconds)) return {};
    if (params.effectiveAreaM2 <= 0.0 || params.dischargeCoefficient <= 0.0) return {};

    GasCell& first  = *params.system0;
    GasCell& second = *params.system1;
    const auto dirX = params.directionX;
    const auto dirY = params.directionY;

    // Effective pressures include dynamic pressure contribution.
    //  P_eff(first)  = P_static(first)  + q(first  in flow direction)
    //  P_eff(second) = P_static(second) + q(second on the same interface normal)
    // This models gas inertia: a runner with momentum toward the cylinder
    // effectively raises the inlet pressure seen by the valve.
    const auto pEff0 = first.pressureKpa()  + first.dynamicPressureKpa( dirX,  dirY);
    // Both cells must be projected on the same interface normal. Reversing the
    // second normal changes +rho*v^2 into -rho*v^2 and invents a pressure drop
    // even for two equal co-flowing cells.
    const auto pEff1 = second.pressureKpa() + second.dynamicPressureKpa(dirX, dirY);

    auto* source = &first;
    auto* sink   = &second;
    double flowDirX = dirX;
    double flowDirY = dirY;
    if (pEff1 > pEff0) {
        std::swap(source, sink);
        flowDirX = -dirX;
        flowDirY = -dirY;
    }

    const auto sourcePressureEff = (source == &first) ? pEff0 : pEff1;
    const auto sinkPressureEff   = (source == &first) ? pEff1 : pEff0;

    bool choked = false;
    const auto gamma    = source->heatCapacityRatioEffective();
    const auto specificR = GasCell::universalGasConstant / source->meanMolarMassKg();
    const auto massRate = massFlowKgPerSecond(
        std::max(0.0, sourcePressureEff) * 1'000.0,
        std::max(0.0, sinkPressureEff)   * 1'000.0,
        source->temperatureK(), params.effectiveAreaM2, params.dischargeCoefficient,
        choked, gamma, specificR);

    const auto requestedMoles = massRate * params.dtSeconds / source->meanMolarMassKg();
    // Pressure comparison remains on the canonical interface normal even when
    // the material flow reverses; flipping this normal a second time reverses
    // the signed momentum flux and can suppress a legitimate reverse flow.
    const auto equilibriumMoles = pressureEquilibriumMoles(*source, *sink, dirX, dirY,
                                                            requestedMoles, true);
    auto result = transfer(*source, *sink, std::min(requestedMoles, equilibriumMoles));

    // Jet momentum injection with explicit cross-section overrides.
    const auto csArea0 = params.crossSectionArea0 > 0.0
        ? params.crossSectionArea0 : params.system0->characteristicAreaM2_;
    const auto csArea1 = params.crossSectionArea1 > 0.0
        ? params.crossSectionArea1 : params.system1->characteristicAreaM2_;
    const auto sourceCS = (source == &first) ? csArea0 : csArea1;
    const auto sinkCS   = (source == &first) ? csArea1 : csArea0;
    const auto energyBeforeJet = totalEnergy(*source, *sink);
    injectJetMomentum(*source, *sink, std::abs(result.transferredMassKg),
                      std::abs(result.transferredMassKg) / params.dtSeconds,
                      params.effectiveAreaM2, sourceCS, sinkCS,
                      flowDirX, flowDirY);
    restorePairEnergy(*source, *sink, energyBeforeJet);

    result.choked = choked;
    if (source == &second) {
        result.transferredMoles  = -result.transferredMoles;
        result.transferredMassKg = -result.transferredMassKg;
    }
    return result;
}

SimultaneousGasFlowResult ConservativeGasSystem::flowSimultaneous(
    const FlowParameters& firstParams, const FlowParameters& secondParams) noexcept {
    if (!firstParams.system0 || !firstParams.system1 || !secondParams.system0
            || !secondParams.system1 || firstParams.system1 != secondParams.system0)
        return {};
    const auto firstOpen = firstParams.effectiveAreaM2 > 0.0
        && firstParams.dischargeCoefficient > 0.0 && firstParams.dtSeconds > 0.0;
    const auto secondOpen = secondParams.effectiveAreaM2 > 0.0
        && secondParams.dischargeCoefficient > 0.0 && secondParams.dtSeconds > 0.0;
    if (!firstOpen && !secondOpen) return {};
    if (!firstOpen) return { {}, flow(secondParams) };
    if (!secondOpen) return { flow(firstParams), {} };

    const auto leftBefore = *firstParams.system0;
    const auto middleBefore = *firstParams.system1;
    const auto rightBefore = *secondParams.system1;
    const auto threeCellEnergy = [](const GasCell& left, const GasCell& middle,
                                    const GasCell& right) noexcept {
        return left.internalEnergyJ_ + middle.internalEnergyJ_ + right.internalEnergyJ_
            + left.bulkKineticEnergyJoules() + middle.bulkKineticEnergyJoules()
            + right.bulkKineticEnergyJoules();
    };
    const auto targetEnergy = threeCellEnergy(leftBefore, middleBefore, rightBefore);

    // Evaluate each restriction independently from exactly the same middle-cell
    // pre-state.  No branch is allowed to observe inventory delivered or removed
    // by the other branch during this explicit time step.
    auto leftAfter = leftBefore;
    auto middleAfterFirst = middleBefore;
    auto first = firstParams;
    first.system0 = &leftAfter;
    first.system1 = &middleAfterFirst;
    auto firstResult = flow(first);

    auto middleAfterSecond = middleBefore;
    auto rightAfter = rightBefore;
    auto second = secondParams;
    second.system0 = &middleAfterSecond;
    second.system1 = &rightAfter;
    auto secondResult = flow(second);

    struct CellDelta final {
        GasMixture mixture;
        double internalEnergyJ { 0.0 };
        double momentumXKgMps { 0.0 };
        double momentumYKgMps { 0.0 };
    };
    const auto difference = [](const GasCell& after, const GasCell& before) noexcept {
        return CellDelta {
            { after.mixture_.oxygenMoles - before.mixture_.oxygenMoles,
              after.mixture_.inertMoles - before.mixture_.inertMoles,
              after.mixture_.fuelMoles - before.mixture_.fuelMoles,
              after.mixture_.burnedMoles - before.mixture_.burnedMoles },
            after.internalEnergyJ_ - before.internalEnergyJ_,
            after.momentumXKgMps_ - before.momentumXKgMps_,
            after.momentumYKgMps_ - before.momentumYKgMps_
        };
    };
    const auto leftDelta = difference(leftAfter, leftBefore);
    const auto middleFirstDelta = difference(middleAfterFirst, middleBefore);
    const auto middleSecondDelta = difference(middleAfterSecond, middleBefore);
    const auto rightDelta = difference(rightAfter, rightBefore);

    auto commonScale = 1.0;
    const auto firstRemovesMiddle = middleFirstDelta.mixture.totalMoles() < -1.0e-15;
    const auto secondRemovesMiddle = middleSecondDelta.mixture.totalMoles() < -1.0e-15;
    if (firstRemovesMiddle && secondRemovesMiddle) {
        const auto constrainOutgoing = [&commonScale](double inventory,
                                                       double firstDelta,
                                                       double secondDelta) noexcept {
            const auto outgoing = std::max(0.0, -firstDelta) + std::max(0.0, -secondDelta);
            if (outgoing > 1.0e-15)
                commonScale = std::min(commonScale, 0.95 * inventory / outgoing);
        };
        constrainOutgoing(middleBefore.mixture_.oxygenMoles,
            middleFirstDelta.mixture.oxygenMoles, middleSecondDelta.mixture.oxygenMoles);
        constrainOutgoing(middleBefore.mixture_.inertMoles,
            middleFirstDelta.mixture.inertMoles, middleSecondDelta.mixture.inertMoles);
        constrainOutgoing(middleBefore.mixture_.fuelMoles,
            middleFirstDelta.mixture.fuelMoles, middleSecondDelta.mixture.fuelMoles);
        constrainOutgoing(middleBefore.mixture_.burnedMoles,
            middleFirstDelta.mixture.burnedMoles, middleSecondDelta.mixture.burnedMoles);
        constrainOutgoing(middleBefore.internalEnergyJ_,
            middleFirstDelta.internalEnergyJ, middleSecondDelta.internalEnergyJ);
        commonScale = std::clamp(commonScale, 0.0, 1.0);
    }

    const auto applyDelta = [](GasCell& cell, const CellDelta& delta, double scale) noexcept {
        cell.mixture_.oxygenMoles += delta.mixture.oxygenMoles * scale;
        cell.mixture_.inertMoles += delta.mixture.inertMoles * scale;
        cell.mixture_.fuelMoles += delta.mixture.fuelMoles * scale;
        cell.mixture_.burnedMoles += delta.mixture.burnedMoles * scale;
        cell.internalEnergyJ_ += delta.internalEnergyJ * scale;
        cell.momentumXKgMps_ += delta.momentumXKgMps * scale;
        cell.momentumYKgMps_ += delta.momentumYKgMps * scale;
    };
    auto left = leftBefore;
    auto middle = middleBefore;
    auto right = rightBefore;
    applyDelta(left, leftDelta, commonScale);
    applyDelta(middle, middleFirstDelta, commonScale);
    applyDelta(middle, middleSecondDelta, commonScale);
    applyDelta(right, rightDelta, commonScale);

    // The linear branch deltas conserve transported internal energy.  Their
    // jointly committed momenta have a nonlinear kinetic-energy cross term;
    // resolve that term conservatively instead of averaging two execution orders.
    auto correction = targetEnergy - threeCellEnergy(left, middle, right);
    if (correction >= 0.0) {
        middle.internalEnergyJ_ += correction;
    } else {
        auto excess = -correction;
        const auto internalTotal = left.internalEnergyJ_ + middle.internalEnergyJ_
            + right.internalEnergyJ_;
        if (internalTotal > 0.0) {
            const auto removed = std::min(excess, internalTotal);
            const auto retainedFraction = (internalTotal - removed) / internalTotal;
            left.internalEnergyJ_ *= retainedFraction;
            middle.internalEnergyJ_ *= retainedFraction;
            right.internalEnergyJ_ *= retainedFraction;
            excess -= removed;
        }
        if (excess > 1.0e-12) {
            const auto kineticTotal = left.bulkKineticEnergyJoules()
                + middle.bulkKineticEnergyJoules() + right.bulkKineticEnergyJoules();
            if (kineticTotal > 0.0) {
                const auto scale = std::sqrt(std::max(0.0, kineticTotal - excess) / kineticTotal);
                left.momentumXKgMps_ *= scale;
                left.momentumYKgMps_ *= scale;
                middle.momentumXKgMps_ *= scale;
                middle.momentumYKgMps_ *= scale;
                right.momentumXKgMps_ *= scale;
                right.momentumYKgMps_ *= scale;
            }
        }
        correction = targetEnergy - threeCellEnergy(left, middle, right);
        if (correction > 0.0) middle.internalEnergyJ_ += correction;
    }

    *firstParams.system0 = left;
    *firstParams.system1 = middle;
    *secondParams.system1 = right;
    firstResult.transferredMoles *= commonScale;
    firstResult.transferredMassKg *= commonScale;
    secondResult.transferredMoles *= commonScale;
    secondResult.transferredMassKg *= commonScale;
    return { firstResult, secondResult };
}

GasInventoryDelta ConservativeGasSystem::inventoryDelta(
    const GasCell& after, const GasCell& before) noexcept {
    return {
        { after.mixture_.oxygenMoles - before.mixture_.oxygenMoles,
          after.mixture_.inertMoles - before.mixture_.inertMoles,
          after.mixture_.fuelMoles - before.mixture_.fuelMoles,
          after.mixture_.burnedMoles - before.mixture_.burnedMoles },
        after.internalEnergyJ_ - before.internalEnergyJ_,
        after.momentumXKgMps_ - before.momentumXKgMps_,
        after.momentumYKgMps_ - before.momentumYKgMps_
    };
}

void ConservativeGasSystem::commitJacobiFlows(
    GasCell& shared, std::span<const JacobiGasFlowBranch> branches) noexcept {
    if (branches.empty()) return;

    const auto cellEnergy = [](const GasCell& cell) noexcept {
        return cell.internalEnergyJ_ + cell.bulkKineticEnergyJoules();
    };
    auto energyBefore = cellEnergy(shared);
    auto desiredEnergyChange = 0.0;
    GasMixture outgoingMoles {};
    auto outgoingEnergy = 0.0;
    for (const auto& branch : branches) {
        if (!branch.counterpart) continue;
        energyBefore += cellEnergy(*branch.counterpart);
        desiredEnergyChange += branch.sharedTotalEnergyDeltaJ;
        if (branch.sharedDelta.mixture.totalMoles() < -1.0e-15) {
            outgoingMoles.oxygenMoles += std::max(0.0, -branch.sharedDelta.mixture.oxygenMoles);
            outgoingMoles.inertMoles += std::max(0.0, -branch.sharedDelta.mixture.inertMoles);
            outgoingMoles.fuelMoles += std::max(0.0, -branch.sharedDelta.mixture.fuelMoles);
            outgoingMoles.burnedMoles += std::max(0.0, -branch.sharedDelta.mixture.burnedMoles);
            outgoingEnergy += std::max(0.0, -branch.sharedDelta.internalEnergyJ);
        }
    }

    auto outgoingScale = 1.0;
    const auto constrainOutgoing = [&outgoingScale](double inventory, double outgoing) noexcept {
        if (outgoing > 1.0e-15)
            outgoingScale = std::min(outgoingScale, 0.95 * inventory / outgoing);
    };
    constrainOutgoing(shared.mixture_.oxygenMoles, outgoingMoles.oxygenMoles);
    constrainOutgoing(shared.mixture_.inertMoles, outgoingMoles.inertMoles);
    constrainOutgoing(shared.mixture_.fuelMoles, outgoingMoles.fuelMoles);
    constrainOutgoing(shared.mixture_.burnedMoles, outgoingMoles.burnedMoles);
    constrainOutgoing(shared.internalEnergyJ_, outgoingEnergy);
    outgoingScale = std::clamp(outgoingScale, 0.0, 1.0);

    const auto applyDelta = [](GasCell& cell, const GasInventoryDelta& delta,
                               double scale) noexcept {
        cell.mixture_.oxygenMoles += delta.mixture.oxygenMoles * scale;
        cell.mixture_.inertMoles += delta.mixture.inertMoles * scale;
        cell.mixture_.fuelMoles += delta.mixture.fuelMoles * scale;
        cell.mixture_.burnedMoles += delta.mixture.burnedMoles * scale;
        cell.internalEnergyJ_ += delta.internalEnergyJ * scale;
        cell.momentumXKgMps_ += delta.momentumXKgMps * scale;
        cell.momentumYKgMps_ += delta.momentumYKgMps * scale;
    };
    for (const auto& branch : branches) {
        if (!branch.counterpart) continue;
        const auto removesShared = branch.sharedDelta.mixture.totalMoles() < -1.0e-15;
        const auto scale = removesShared ? outgoingScale : 1.0;
        // The branch cell already contains the full candidate delta. If the
        // shared inventory constrains it, remove the same fraction from the
        // current branch state before committing the scaled shared delta.
        applyDelta(*branch.counterpart, branch.counterpartDelta, scale - 1.0);
        applyDelta(shared, branch.sharedDelta, scale);
    }

    const auto currentEnergy = [&]() noexcept {
        auto total = cellEnergy(shared);
        for (const auto& branch : branches)
            if (branch.counterpart) total += cellEnergy(*branch.counterpart);
        return total;
    };
    const auto targetEnergy = energyBefore + desiredEnergyChange;
    const auto energyAfterCommit = currentEnergy();
    if (!std::isfinite(targetEnergy) || !std::isfinite(energyAfterCommit)) return;
    auto correction = targetEnergy - energyAfterCommit;
    if (correction >= 0.0) {
        shared.internalEnergyJ_ += correction;
        return;
    }

    auto excess = -correction;
    const auto removeInternalEnergy = [&excess](GasCell& cell) noexcept {
        const auto removed = std::min(excess, cell.internalEnergyJ_);
        cell.internalEnergyJ_ -= removed;
        excess -= removed;
    };
    removeInternalEnergy(shared);
    for (const auto& branch : branches)
        if (branch.counterpart && excess > 0.0) removeInternalEnergy(*branch.counterpart);
    if (excess > 1.0e-12) {
        auto kineticEnergy = shared.bulkKineticEnergyJoules();
        for (const auto& branch : branches)
            if (branch.counterpart) kineticEnergy += branch.counterpart->bulkKineticEnergyJoules();
        if (kineticEnergy > 0.0) {
            const auto momentumScale = std::sqrt(std::max(0.0, kineticEnergy - excess) / kineticEnergy);
            shared.momentumXKgMps_ *= momentumScale;
            shared.momentumYKgMps_ *= momentumScale;
            for (const auto& branch : branches) {
                if (!branch.counterpart) continue;
                branch.counterpart->momentumXKgMps_ *= momentumScale;
                branch.counterpart->momentumYKgMps_ *= momentumScale;
            }
        }
    }
    const auto energyAfterRemoval = currentEnergy();
    if (!std::isfinite(energyAfterRemoval)) return;
    correction = targetEnergy - energyAfterRemoval;
    if (correction > 0.0) shared.internalEnergyJ_ += correction;
}

// ---------------------------------------------------------------------------
// ConservativeGasSystem::flowFromBoundary
// ---------------------------------------------------------------------------

GasFlowResult ConservativeGasSystem::flowFromBoundary(GasCell& target, double pressureKpa,
                                                       double temperatureK, double area,
                                                       double coefficient, double dt,
                                                       double targetToBoundaryDirectionX,
                                                       double targetToBoundaryDirectionY) noexcept {
    if (dt <= 0.0 || area <= 0.0) return {};
    GasCell boundary;
    boundary.initialise(pressureKpa, std::max(1'000.0, target.volumeLitres() * 1'000.0),
                        temperatureK);
    boundary.setGeometry(area, targetToBoundaryDirectionX, targetToBoundaryDirectionY);
    return flow({ &target, &boundary, area, coefficient, dt,
        targetToBoundaryDirectionX, targetToBoundaryDirectionY,
        target.characteristicAreaM2(), area });
}

// ---------------------------------------------------------------------------
// ConservativeGasSystem — combustion reactions
// ---------------------------------------------------------------------------

CombustionReaction ConservativeGasSystem::reactGasoline(GasCell& cell, double requestedFraction,
                                                          double efficiency) noexcept {
    return reactFuel(cell, requestedFraction, efficiency, gasolineEnergyJPerKg);
}

CombustionReaction ConservativeGasSystem::reactFuel(GasCell& cell, double requestedFraction,
                                                      double efficiency,
                                                      double lowerHeatingValueJPerKg) noexcept {
    const auto availableFuel = cell.mixture_.fuelMoles;
    const auto oxygenLimitedFuel = cell.mixture_.oxygenMoles / cell.oxygenMolesPerFuelMole_;
    const auto burnable = std::min(availableFuel, oxygenLimitedFuel);
    return reactFuelMoles(cell, burnable * std::clamp(requestedFraction, 0.0, 1.0),
                          efficiency, lowerHeatingValueJPerKg);
}

CombustionReaction ConservativeGasSystem::reactFuelMoles(GasCell& cell,
                                                           double requestedFuelMoles,
                                                           double efficiency,
                                                           double lowerHeatingValueJPerKg) noexcept {
    const auto availableFuel = cell.mixture_.fuelMoles;
    const auto oxygenLimitedFuel = cell.mixture_.oxygenMoles / cell.oxygenMolesPerFuelMole_;
    // Efficiency is chemical completeness: inefficient combustion leaves a
    // corresponding fraction of fuel unreacted.  Every mole which does react
    // releases the fuel's full lower heating value; scaling heat alone would
    // destroy chemical energy while incorrectly consuming all reactants.
    const auto chemicallyReactingFuel = std::max(0.0, requestedFuelMoles)
        * std::clamp(efficiency, 0.0, 1.0);
    const auto burned = std::min({ availableFuel, oxygenLimitedFuel,
                                   chemicallyReactingFuel });
    if (burned <= 1.0e-15) return {};
    cell.mixture_.fuelMoles   -= burned;
    cell.mixture_.oxygenMoles -= burned * cell.oxygenMolesPerFuelMole_;
    // Product pseudo-species: mass-conserving surrogate derived from chemistry config.
    cell.mixture_.burnedMoles += burned * cell.productMolesPerFuelMole_;
    const auto energy = burned * cell.fuelMolarMassKg_
        * std::clamp(lowerHeatingValueJPerKg, 10'000'000.0, 60'000'000.0);
    cell.internalEnergyJ_ += energy;
    return { burned, energy, availableFuel > 1.0e-15 ? burned / availableFuel : 0.0 };
}

} // namespace enginelab
