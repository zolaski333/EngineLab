#pragma once

#include <algorithm>
#include <cmath>

namespace enginelab {

struct GasMixture final {
    double oxygenMoles { 0.0 };
    double inertMoles { 0.0 };
    double fuelMoles { 0.0 };
    double burnedMoles { 0.0 };

    [[nodiscard]] double totalMoles() const noexcept {
        return oxygenMoles + inertMoles + fuelMoles + burnedMoles;
    }
};

/**
 * A zero-dimensional gas control volume. Mass and internal energy only move
 * through explicit transfer/reaction operations; pressure and temperature are
 * derived from the conserved state.
 */
class GasCell final {
public:
    static constexpr double universalGasConstant = 8.31446261815324;
    static constexpr double molarHeatCapacityCv = 20.7861565454;
    static constexpr double airMolarMassKg = 0.0289652;
    static constexpr double oxygenMolarMassKg = 0.0319988;
    static constexpr double inertMolarMassKg = 0.0280134;
    static constexpr double gasolineMolarMassKg = 0.11423;
    static constexpr double burnedGasMolarMassKg =
        (gasolineMolarMassKg + 12.5 * oxygenMolarMassKg) / 17.0;

    void initialise(double pressureKpa, double volumeLitres, double temperatureK,
                    GasMixture composition = {}) noexcept;
    void reset(double pressureKpa, double temperatureK, GasMixture composition = {}) noexcept;
    void setVolumeAdiabatic(double volumeLitres) noexcept;
    void addHeatJoules(double joules) noexcept;
    void injectFuelMoles(double moles, double temperatureK = 293.15) noexcept;
    void setBulkVelocityMps(double velocity) noexcept { momentumKgMps_ = massKg() * velocity; }
    void dissipateMomentum(double timeConstantSeconds, double dtSeconds) noexcept;

    [[nodiscard]] double volumeLitres() const noexcept { return volumeM3_ * 1'000.0; }
    [[nodiscard]] double volumeM3() const noexcept { return volumeM3_; }
    [[nodiscard]] double pressureKpa() const noexcept;
    [[nodiscard]] double temperatureK() const noexcept;
    [[nodiscard]] double internalEnergyJoules() const noexcept { return internalEnergyJ_; }
    [[nodiscard]] double totalMoles() const noexcept { return mixture_.totalMoles(); }
    [[nodiscard]] double massKg() const noexcept {
        return mixture_.oxygenMoles * oxygenMolarMassKg
            + mixture_.inertMoles * inertMolarMassKg
            + mixture_.fuelMoles * gasolineMolarMassKg
            + mixture_.burnedMoles * burnedGasMolarMassKg;
    }
    [[nodiscard]] double meanMolarMassKg() const noexcept {
        return totalMoles() > 1.0e-15 ? massKg() / totalMoles() : airMolarMassKg;
    }
    [[nodiscard]] double momentumKgMps() const noexcept { return momentumKgMps_; }
    [[nodiscard]] double bulkVelocityMps() const noexcept {
        return massKg() > 1.0e-12 ? momentumKgMps_ / massKg() : 0.0;
    }
    [[nodiscard]] const GasMixture& mixture() const noexcept { return mixture_; }

private:
    friend class ConservativeGasSystem;
    GasMixture mixture_ {};
    double internalEnergyJ_ { 0.0 };
    double volumeM3_ { 0.001 };
    double momentumKgMps_ { 0.0 };
};

struct GasFlowResult final {
    double transferredMoles { 0.0 };
    double transferredMassKg { 0.0 };
    bool choked { false };
};

struct CombustionReaction final {
    double burnedFuelMoles { 0.0 };
    double releasedEnergyJoules { 0.0 };
    double completeness { 0.0 };
};

class ConservativeGasSystem final {
public:
    [[nodiscard]] static GasFlowResult flow(GasCell& first, GasCell& second,
                                            double effectiveAreaM2, double dischargeCoefficient,
                                            double dtSeconds) noexcept;
    [[nodiscard]] static GasFlowResult flowFromBoundary(GasCell& target, double boundaryPressureKpa,
                                                        double boundaryTemperatureK,
                                                        double effectiveAreaM2,
                                                        double dischargeCoefficient,
                                                        double dtSeconds) noexcept;
    [[nodiscard]] static CombustionReaction reactGasoline(GasCell& cell, double requestedFraction,
                                                           double efficiency) noexcept;
    [[nodiscard]] static double totalMoles(const GasCell& first, const GasCell& second) noexcept {
        return first.totalMoles() + second.totalMoles();
    }
    [[nodiscard]] static double totalEnergy(const GasCell& first, const GasCell& second) noexcept {
        return first.internalEnergyJoules() + second.internalEnergyJoules();
    }
    [[nodiscard]] static double totalMomentum(const GasCell& first, const GasCell& second) noexcept {
        return first.momentumKgMps() + second.momentumKgMps();
    }

private:
    static GasFlowResult transfer(GasCell& source, GasCell& sink, double moles) noexcept;
};

} // namespace enginelab
