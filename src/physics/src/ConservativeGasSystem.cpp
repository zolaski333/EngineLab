#include <enginelab/physics/ConservativeGasSystem.hpp>

#include <limits>
#include <numbers>

namespace enginelab {
namespace {
constexpr double minimumVolumeM3 = 1.0e-8;
constexpr double minimumTemperatureK = 80.0;
constexpr double maximumTemperatureK = 5'000.0;
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
 * When gas flows through a restriction at throat velocity v_jet, the
 * momentum carried by the flowing mass is m·v_jet.  The source "kicks back"
 * (Newton's 3rd law) and the sink is accelerated.  Velocity is signed by
 * signedDirection: +1 source→sink, -1 sink→source.
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

    // Throat velocity (from continuity): v = ṁ / (ρ·A)
    const auto throatVelocity = std::clamp(massFlowRate / std::max(1.0e-12, sourceDensity * throatAreaM2),
                                            0.0, sourceC);
    const auto jetMomentum = movedMassKg * throatVelocity;

    if (sourceCharAreaM2 <= 0.0 && sinkCharAreaM2 <= 0.0) return;
    const auto length = std::hypot(dirX, dirY);
    const auto nx = length > 1.0e-12 ? dirX / length : 1.0;
    const auto ny = length > 1.0e-12 ? dirY / length : 0.0;
    const auto kineticBefore = source.bulkKineticEnergyJoules() + sink.bulkKineticEnergyJoules();
    source.addMomentumKgMps(-jetMomentum * nx, -jetMomentum * ny);
    sink.addMomentumKgMps(jetMomentum * nx, jetMomentum * ny);
    const auto kineticAfter = source.bulkKineticEnergyJoules() + sink.bulkKineticEnergyJoules();
    source.addHeatJoules(-(kineticAfter - kineticBefore));
}
} // anonymous namespace

// ---------------------------------------------------------------------------
// GasCell methods
// ---------------------------------------------------------------------------

void GasCell::initialise(double pressureKpa, double volumeLitres, double temperatureK,
                         GasMixture composition) noexcept {
    volumeM3_ = std::max(minimumVolumeM3, volumeLitres * 0.001);
    const auto safePressurePa = std::max(10.0, pressureKpa * 1'000.0);
    const auto safeTemperature = std::clamp(temperatureK, minimumTemperatureK, maximumTemperatureK);
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
    const auto oldPressurePa = pressureKpa() * 1'000.0;
    const auto ratio = volumeM3_ / newVolume;
    const auto predictedPressurePa = oldPressurePa * std::pow(ratio, gamma);
    const auto averagePressurePa = 0.5 * (oldPressurePa + predictedPressurePa);
    internalEnergyJ_ = std::max(0.0, internalEnergyJ_ - averagePressurePa * (newVolume - volumeM3_));
    volumeM3_ = newVolume;
}

void GasCell::addHeatJoules(double joules) noexcept {
    if (std::isfinite(joules)) internalEnergyJ_ = std::max(0.0, internalEnergyJ_ + joules);
}

void GasCell::injectFuelMoles(double moles, double temperature) noexcept {
    const auto added = std::max(0.0, moles);
    mixture_.fuelMoles += added;
    constexpr double gasolineVapourCvJPerMolK = 145.0;
    internalEnergyJ_ += added * gasolineVapourCvJPerMolK
        * std::clamp(temperature, minimumTemperatureK, maximumTemperatureK);
}

void GasCell::configureFuelChemistry(double fuelMolarMassKg, double oxygenMolesPerFuelMole,
                                     double productMolesPerFuelMole) noexcept {
    fuelMolarMassKg_ = std::clamp(fuelMolarMassKg, 0.020, 0.300);
    oxygenMolesPerFuelMole_ = std::clamp(oxygenMolesPerFuelMole, 1.0, 40.0);
    productMolesPerFuelMole_ = std::clamp(productMolesPerFuelMole, 1.0, 60.0);
    burnedGasMolarMassKg_ = (fuelMolarMassKg_
        + oxygenMolesPerFuelMole_ * oxygenMolarMassKg) / productMolesPerFuelMole_;
}

void GasCell::dissipateMomentum(double timeConstantSeconds, double dtSeconds) noexcept {
    const auto timeConstant = std::max(1.0e-6, timeConstantSeconds);
    const auto decay = std::exp(-std::max(0.0, dtSeconds) / timeConstant);
    const auto kineticBefore = bulkKineticEnergyJoules();
    momentumXKgMps_ *= decay;
    momentumYKgMps_ *= decay;
    internalEnergyJ_ += std::max(0.0, kineticBefore - bulkKineticEnergyJoules());
}

void GasCell::dissipateExcessVelocity() noexcept {
    const auto m = massKg();
    if (m <= 1.0e-12) { momentumXKgMps_ = momentumYKgMps_ = 0.0; return; }
    const auto velocitySquared = velocityXMps() * velocityXMps() + velocityYMps() * velocityYMps();
    const auto c = speedOfSoundMps();
    if (velocitySquared <= c * c || velocitySquared <= 0.0) return;
    const auto scale = c / std::sqrt(velocitySquared);
    const auto excessKineticJ = bulkKineticEnergyJoules() * (1.0 - scale * scale);
    internalEnergyJ_ = std::max(0.0, internalEnergyJ_ + excessKineticJ);
    momentumXKgMps_ *= scale;
    momentumYKgMps_ *= scale;
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
    if (n <= 1.0e-15) return minimumTemperatureK;
    const auto heatCapacity = n * molarHeatCapacityCvEffective();
    return std::clamp(internalEnergyJ_ / heatCapacity, minimumTemperatureK, maximumTemperatureK);
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
    // Linear blend: γ_air = 1.40 → γ_burned = 1.26 as burned fraction → 1
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
    if (vDotN <= 0.0) return 0.0;
    const auto density = m / volumeM3_;
    // q = ½·ρ·|v_n|·v_n  (signed, in Pa → kPa)
    const auto gamma = heatCapacityRatioEffective();
    const auto speedOfSoundSquared = std::max(1.0, gamma * pressureKpa() * 1'000.0 / density);
    const auto machSquared = vDotN * vDotN / speedOfSoundSquared;
    const auto stagnationRatio = std::pow(1.0 + (gamma - 1.0) * 0.5 * machSquared,
                                          gamma / (gamma - 1.0));
    return pressureKpa() * (stagnationRatio - 1.0);
}

// ---------------------------------------------------------------------------
// ConservativeGasSystem — private helpers
// ---------------------------------------------------------------------------

GasFlowResult ConservativeGasSystem::transfer(GasCell& source, GasCell& sink, double moles) noexcept {
    GasFlowResult result;
    const auto sourceMoles = source.totalMoles();
    const auto specificEnthalpy = (source.molarHeatCapacityCvEffective()
        + GasCell::universalGasConstant) * source.temperatureK();
    const auto thermalLimit = source.internalEnergyJ_ / std::max(1.0, specificEnthalpy) * 0.98;
    const auto amount = std::clamp(moles, 0.0, std::min(sourceMoles * 0.999, thermalLimit));
    if (amount <= 1.0e-15 || sourceMoles <= 1.0e-15) return result;
    const auto fraction = amount / sourceMoles;
    const auto kineticBefore = source.bulkKineticEnergyJoules() + sink.bulkKineticEnergyJoules();
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
    const auto kineticAfter = source.bulkKineticEnergyJoules() + sink.bulkKineticEnergyJoules();
    sink.internalEnergyJ_ += std::max(0.0, kineticBefore - kineticAfter);
    result.transferredMoles  = amount;
    result.transferredMassKg = movedMass;
    return result;
}

double ConservativeGasSystem::pressureEquilibriumMoles(const GasCell& source,
                                                        const GasCell& sink,
                                                        double directionX,
                                                        double directionY,
                                                        double requestedMoles) noexcept {
    const auto sourceEffective = source.pressureKpa()
        + source.dynamicPressureKpa(directionX, directionY);
    const auto sinkEffective = sink.pressureKpa()
        + sink.dynamicPressureKpa(-directionX, -directionY);
    if (sourceEffective <= sinkEffective || source.totalMoles() <= 1.0e-15) return 0.0;
    auto low = 0.0;
    auto high = std::min(std::max(0.0, requestedMoles), source.totalMoles() * 0.95);
    auto sourceAtRequested = source;
    auto sinkAtRequested = sink;
    (void)transfer(sourceAtRequested, sinkAtRequested, high);
    const auto requestedSourcePressure = sourceAtRequested.pressureKpa()
        + sourceAtRequested.dynamicPressureKpa(directionX, directionY);
    const auto requestedSinkPressure = sinkAtRequested.pressureKpa()
        + sinkAtRequested.dynamicPressureKpa(-directionX, -directionY);
    if (requestedSourcePressure >= requestedSinkPressure) return high;
    // Ten monotonic bisection iterations bound a transfer to better than
    // 0.1% of the source inventory without dominating the realtime solver.
    for (int iteration = 0; iteration < 10; ++iteration) {
        const auto middle = 0.5 * (low + high);
        auto sourceCopy = source;
        auto sinkCopy = sink;
        (void)transfer(sourceCopy, sinkCopy, middle);
        const auto sourcePressure = sourceCopy.pressureKpa()
            + sourceCopy.dynamicPressureKpa(directionX, directionY);
        const auto sinkPressure = sinkCopy.pressureKpa()
            + sinkCopy.dynamicPressureKpa(-directionX, -directionY);
        if (sourcePressure > sinkPressure) low = middle;
        else high = middle;
    }
    return high;
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
                                                            requestedMoles);
    auto result = transfer(*source, *sink, std::min(requestedMoles, equilibriumMoles));
    // Jet momentum injection using the cells' stored geometry.
    // Falls back to raw throat velocity capped at 600 m/s for cells without
    // geometry, preserving the original behaviour for un-migrated call sites.
    const auto sourceCharArea = source->characteristicAreaM2_ > 0.0
        ? source->characteristicAreaM2_ : area;
    const auto sinkCharArea   = sink->characteristicAreaM2_ > 0.0
        ? sink->characteristicAreaM2_ : area;
    injectJetMomentum(*source, *sink, std::abs(result.transferredMassKg),
                      massRate, area, sourceCharArea, sinkCharArea,
                      source->dx_, source->dy_);
    // Clamp velocities to speed of sound.
    source->dissipateExcessVelocity();
    sink->dissipateExcessVelocity();
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
    //  P_eff(second) = P_static(second) + q(second in reverse direction)
    // This models gas inertia: a runner with momentum toward the cylinder
    // effectively raises the inlet pressure seen by the valve.
    const auto pEff0 = first.pressureKpa()  + first.dynamicPressureKpa( dirX,  dirY);
    const auto pEff1 = second.pressureKpa() + second.dynamicPressureKpa(-dirX, -dirY);

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
    const auto equilibriumMoles = pressureEquilibriumMoles(*source, *sink, flowDirX, flowDirY,
                                                            requestedMoles);
    auto result = transfer(*source, *sink, std::min(requestedMoles, equilibriumMoles));

    // Jet momentum injection with explicit cross-section overrides.
    const auto csArea0 = params.crossSectionArea0 > 0.0
        ? params.crossSectionArea0 : params.system0->characteristicAreaM2_;
    const auto csArea1 = params.crossSectionArea1 > 0.0
        ? params.crossSectionArea1 : params.system1->characteristicAreaM2_;
    const auto sourceCS = (source == &first) ? csArea0 : csArea1;
    const auto sinkCS   = (source == &first) ? csArea1 : csArea0;
    injectJetMomentum(*source, *sink, std::abs(result.transferredMassKg),
                      massRate, params.effectiveAreaM2, sourceCS, sinkCS,
                      flowDirX, flowDirY);

    source->dissipateExcessVelocity();
    sink->dissipateExcessVelocity();

    result.choked = choked;
    if (source == &second) {
        result.transferredMoles  = -result.transferredMoles;
        result.transferredMassKg = -result.transferredMassKg;
    }
    return result;
}

// ---------------------------------------------------------------------------
// ConservativeGasSystem::flowFromBoundary
// ---------------------------------------------------------------------------

GasFlowResult ConservativeGasSystem::flowFromBoundary(GasCell& target, double pressureKpa,
                                                       double temperatureK, double area,
                                                       double coefficient, double dt) noexcept {
    if (dt <= 0.0 || area <= 0.0) return {};
    GasCell boundary;
    boundary.initialise(pressureKpa, std::max(1'000.0, target.volumeLitres() * 1'000.0),
                        temperatureK);
    boundary.setGeometry(area, target.orientationDx(), target.orientationDy());
    return flow(target, boundary, area, coefficient, dt);
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
    const auto burned = std::min({ availableFuel, oxygenLimitedFuel,
                                   std::max(0.0, requestedFuelMoles) });
    if (burned <= 1.0e-15) return {};
    cell.mixture_.fuelMoles   -= burned;
    cell.mixture_.oxygenMoles -= burned * cell.oxygenMolesPerFuelMole_;
    // Product pseudo-species: mass-conserving surrogate derived from chemistry config.
    cell.mixture_.burnedMoles += burned * cell.productMolesPerFuelMole_;
    const auto energy = burned * cell.fuelMolarMassKg_
        * std::clamp(lowerHeatingValueJPerKg, 10'000'000.0, 60'000'000.0)
        * std::clamp(efficiency, 0.0, 1.0);
    cell.internalEnergyJ_ += energy;
    return { burned, energy, availableFuel > 1.0e-15 ? burned / availableFuel : 0.0 };
}

} // namespace enginelab
