#include <enginelab/physics/ConservativeGasSystem.hpp>

#include <limits>

namespace enginelab {
namespace {
constexpr double minimumVolumeM3 = 1.0e-8;
constexpr double minimumTemperatureK = 80.0;
constexpr double maximumTemperatureK = 5'000.0;
constexpr double gammaAir = 1.4;
constexpr double gasolineEnergyJPerKg = 44'000'000.0;

[[nodiscard]] GasMixture atmosphericComposition(double totalMoles) noexcept {
    return { totalMoles * 0.21, totalMoles * 0.79, 0.0, 0.0 };
}

[[nodiscard]] double massFlowKgPerSecond(double upstreamPressurePa, double downstreamPressurePa,
                                         double temperatureK, double areaM2, double coefficient,
                                         bool& choked) noexcept {
    if (!(upstreamPressurePa > downstreamPressurePa) || areaM2 <= 0.0 || coefficient <= 0.0)
        return 0.0;
    constexpr double specificGasConstant = GasCell::universalGasConstant / GasCell::airMolarMassKg;
    const auto ratio = std::clamp(downstreamPressurePa / upstreamPressurePa, 0.0, 1.0);
    const auto critical = std::pow(2.0 / (gammaAir + 1.0), gammaAir / (gammaAir - 1.0));
    choked = ratio <= critical;
    double flowFunction = 0.0;
    if (choked) {
        flowFunction = std::sqrt(gammaAir)
            * std::pow(2.0 / (gammaAir + 1.0), (gammaAir + 1.0) / (2.0 * (gammaAir - 1.0)));
    } else {
        const auto term = 2.0 * gammaAir / (gammaAir - 1.0)
            * (std::pow(ratio, 2.0 / gammaAir) - std::pow(ratio, (gammaAir + 1.0) / gammaAir));
        flowFunction = std::sqrt(std::max(0.0, term));
    }
    return coefficient * areaM2 * upstreamPressurePa
        / std::sqrt(specificGasConstant * std::max(minimumTemperatureK, temperatureK)) * flowFunction;
}
}

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
    internalEnergyJ_ = requestedMoles * molarHeatCapacityCv * safeTemperature;
    momentumKgMps_ = 0.0;
}

void GasCell::reset(double pressureKpa, double temperatureK, GasMixture composition) noexcept {
    initialise(pressureKpa, volumeLitres(), temperatureK, composition);
}

void GasCell::setVolumeAdiabatic(double volumeLitres) noexcept {
    const auto newVolume = std::max(minimumVolumeM3, volumeLitres * 0.001);
    if (std::abs(newVolume - volumeM3_) <= std::numeric_limits<double>::epsilon()) return;
    const auto oldPressurePa = pressureKpa() * 1'000.0;
    const auto ratio = volumeM3_ / newVolume;
    const auto predictedPressurePa = oldPressurePa * std::pow(ratio, gammaAir);
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
    internalEnergyJ_ += added * molarHeatCapacityCv
        * std::clamp(temperature, minimumTemperatureK, maximumTemperatureK);
}

void GasCell::dissipateMomentum(double timeConstantSeconds, double dtSeconds) noexcept {
    const auto timeConstant = std::max(1.0e-6, timeConstantSeconds);
    const auto decay = std::exp(-std::max(0.0, dtSeconds) / timeConstant);
    momentumKgMps_ *= decay;
}

double GasCell::pressureKpa() const noexcept {
    const auto n = totalMoles();
    if (n <= 1.0e-15 || volumeM3_ <= minimumVolumeM3) return 0.0;
    return n * universalGasConstant * temperatureK() / volumeM3_ / 1'000.0;
}

double GasCell::temperatureK() const noexcept {
    const auto heatCapacity = totalMoles() * molarHeatCapacityCv;
    if (heatCapacity <= 1.0e-15) return minimumTemperatureK;
    return std::clamp(internalEnergyJ_ / heatCapacity, minimumTemperatureK, maximumTemperatureK);
}

GasFlowResult ConservativeGasSystem::transfer(GasCell& source, GasCell& sink, double moles) noexcept {
    GasFlowResult result;
    const auto sourceMoles = source.totalMoles();
    const auto amount = std::clamp(moles, 0.0, sourceMoles * 0.35);
    if (amount <= 1.0e-15 || sourceMoles <= 1.0e-15) return result;
    const auto fraction = amount / sourceMoles;
    const GasMixture moved { source.mixture_.oxygenMoles * fraction, source.mixture_.inertMoles * fraction,
                             source.mixture_.fuelMoles * fraction, source.mixture_.burnedMoles * fraction };
    const auto movedEnergy = source.internalEnergyJ_ * fraction;
    const auto movedMass = source.massKg() * fraction;
    const auto movedMomentum = source.momentumKgMps_ * fraction;
    source.mixture_.oxygenMoles -= moved.oxygenMoles;
    source.mixture_.inertMoles -= moved.inertMoles;
    source.mixture_.fuelMoles -= moved.fuelMoles;
    source.mixture_.burnedMoles -= moved.burnedMoles;
    source.internalEnergyJ_ -= movedEnergy;
    source.momentumKgMps_ -= movedMomentum;
    sink.mixture_.oxygenMoles += moved.oxygenMoles;
    sink.mixture_.inertMoles += moved.inertMoles;
    sink.mixture_.fuelMoles += moved.fuelMoles;
    sink.mixture_.burnedMoles += moved.burnedMoles;
    sink.internalEnergyJ_ += movedEnergy;
    sink.momentumKgMps_ += movedMomentum;
    result.transferredMoles = amount;
    result.transferredMassKg = movedMass;
    return result;
}

GasFlowResult ConservativeGasSystem::flow(GasCell& first, GasCell& second, double area,
                                          double coefficient, double dt) noexcept {
    if (dt <= 0.0 || !std::isfinite(dt)) return {};
    auto* source = &first;
    auto* sink = &second;
    if (second.pressureKpa() > first.pressureKpa()) std::swap(source, sink);
    bool choked = false;
    const auto massRate = massFlowKgPerSecond(source->pressureKpa() * 1'000.0,
        sink->pressureKpa() * 1'000.0, source->temperatureK(), area, coefficient, choked);
    auto result = transfer(*source, *sink, massRate * dt / source->meanMolarMassKg());
    const auto density = std::max(0.01, source->massKg() / std::max(minimumVolumeM3, source->volumeM3()));
    const auto jetVelocity = std::clamp(massRate / std::max(1.0e-10, density * area), 0.0, 600.0);
    const auto jetMomentum = std::abs(result.transferredMassKg) * jetVelocity;
    source->momentumKgMps_ -= jetMomentum;
    sink->momentumKgMps_ += jetMomentum;
    result.choked = choked;
    if (source == &second) {
        result.transferredMoles = -result.transferredMoles;
        result.transferredMassKg = -result.transferredMassKg;
    }
    return result;
}

GasFlowResult ConservativeGasSystem::flowFromBoundary(GasCell& target, double pressureKpa,
                                                       double temperatureK, double area,
                                                       double coefficient, double dt) noexcept {
    if (dt <= 0.0 || area <= 0.0) return {};
    const auto targetPressure = target.pressureKpa();
    if (pressureKpa > targetPressure) {
        bool choked = false;
        const auto massRate = massFlowKgPerSecond(pressureKpa * 1'000.0, targetPressure * 1'000.0,
            temperatureK, area, coefficient, choked);
        const auto moles = massRate * dt / GasCell::airMolarMassKg;
        const auto safeMoles = std::max(0.0, moles);
        const auto composition = atmosphericComposition(safeMoles);
        target.mixture_.oxygenMoles += composition.oxygenMoles;
        target.mixture_.inertMoles += composition.inertMoles;
        target.internalEnergyJ_ += safeMoles * GasCell::molarHeatCapacityCv * temperatureK;
        return { safeMoles, massRate * dt, choked };
    }
    GasCell boundary;
    boundary.initialise(pressureKpa, std::max(1.0, target.volumeLitres()), temperatureK);
    auto result = flow(target, boundary, area, coefficient, dt);
    return result.transferredMoles > 0.0 ? result : GasFlowResult {};
}

CombustionReaction ConservativeGasSystem::reactGasoline(GasCell& cell, double requestedFraction,
                                                          double efficiency) noexcept {
    const auto availableFuel = cell.mixture_.fuelMoles;
    const auto oxygenLimitedFuel = cell.mixture_.oxygenMoles / 12.5;
    const auto burnable = std::min(availableFuel, oxygenLimitedFuel);
    const auto burned = burnable * std::clamp(requestedFraction, 0.0, 1.0);
    if (burned <= 1.0e-15) return {};
    cell.mixture_.fuelMoles -= burned;
    cell.mixture_.oxygenMoles -= burned * 12.5;
    // C8H18 + 12.5 O2 -> 8 CO2 + 9 H2O. The product pseudo-species uses
    // the mean molar mass of those 17 product moles, preserving mass exactly.
    cell.mixture_.burnedMoles += burned * 17.0;
    const auto energy = burned * GasCell::gasolineMolarMassKg * gasolineEnergyJPerKg
        * std::clamp(efficiency, 0.0, 1.0);
    cell.internalEnergyJ_ += energy;
    return { burned, energy, availableFuel > 1.0e-15 ? burned / availableFuel : 0.0 };
}

} // namespace enginelab
