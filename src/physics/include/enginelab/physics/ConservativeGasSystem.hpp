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
 *
 * Physics extensions (v2):
 *  - Variable heat-capacity ratio γ based on burned-gas fraction.
 *    Air: γ = 1.40 (diatomic, 5 dof). Burned products: γ ≈ 1.26 (CO₂/H₂O).
 *  - Two-dimensional momentum in global coordinates. Tube orientation is
 *    geometry, not a replacement for the momentum vector.
 *  - Dynamic pressure contribution for driving isentropic flow: enables
 *    runner scavenging, exhaust backpressure pulses, and intake reversion.
 *  - Speed-of-sound velocity cap: prevents supersonic bulk velocities that
 *    would cause numerical divergence.
 *  - Geometry (characteristicAreaM2) set once and used automatically in
 *    FlowParameters-based flow calls for jet-momentum injection.
 */
class GasCell final {
public:
    static constexpr double universalGasConstant = 8.31446261815324;
    static constexpr double molarHeatCapacityCv = 20.7861565454; // 5/2 * R  (diatomic air)
    static constexpr double airMolarMassKg = 0.0289652;
    static constexpr double oxygenMolarMassKg = 0.0319988;
    static constexpr double inertMolarMassKg = 0.0280134;
    static constexpr double gasolineMolarMassKg = 0.11423;
    static constexpr double burnedGasMolarMassKg =
        (gasolineMolarMassKg + 12.5 * oxygenMolarMassKg) / 17.0;

    // --- Initialisation --------------------------------------------------
    void initialise(double pressureKpa, double volumeLitres, double temperatureK,
                    GasMixture composition = {}) noexcept;
    void reset(double pressureKpa, double temperatureK, GasMixture composition = {}) noexcept;

    /**
     * Set the tube geometry for this control volume.
     *  characteristicAreaM2 : cross-section of the tube this cell represents
     *                         (used for jet-momentum injection in flow calls).
     *  dx, dy               : unit-vector orientation of the tube in global
     *                         coordinates. Positive momentum → flow in (dx,dy).
     */
    void setGeometry(double characteristicAreaM2,
                     double dx = 1.0, double dy = 0.0) noexcept;

    // --- State mutations -------------------------------------------------
    void setVolumeAdiabatic(double volumeLitres) noexcept;
    void addHeatJoules(double joules) noexcept;
    void injectFuelMoles(double moles, double temperatureK = 293.15) noexcept;
    void configureFuelChemistry(double fuelMolarMassKg, double oxygenMolesPerFuelMole,
                                double productMolesPerFuelMole) noexcept;
    void setBulkVelocityMps(double velocity) noexcept {
        momentumXKgMps_ = massKg() * velocity * dx_;
        momentumYKgMps_ = massKg() * velocity * dy_;
    }
    void setBulkVelocityMps(double velocityX, double velocityY) noexcept {
        momentumXKgMps_ = massKg() * velocityX;
        momentumYKgMps_ = massKg() * velocityY;
    }
    void addMomentumKgMps(double deltaX, double deltaY) noexcept {
        momentumXKgMps_ += deltaX;
        momentumYKgMps_ += deltaY;
    }
    void dissipateMomentum(double timeConstantSeconds, double dtSeconds) noexcept;

    /**
     * If the bulk gas velocity exceeds the local speed of sound, clamp it to c
     * and deposit the excess kinetic energy back as heat.  Call after each
     * flow step to keep the 0-D model well-behaved.
     */
    void dissipateExcessVelocity() noexcept;

    // --- Derived quantities -----------------------------------------------
    [[nodiscard]] double volumeLitres() const noexcept { return volumeM3_ * 1'000.0; }
    [[nodiscard]] double volumeM3() const noexcept { return volumeM3_; }
    [[nodiscard]] double pressureKpa() const noexcept;
    [[nodiscard]] double temperatureK() const noexcept;
    [[nodiscard]] double internalEnergyJoules() const noexcept { return internalEnergyJ_; }
    [[nodiscard]] double totalMoles() const noexcept { return mixture_.totalMoles(); }
    [[nodiscard]] double massKg() const noexcept {
        return mixture_.oxygenMoles * oxygenMolarMassKg
            + mixture_.inertMoles * inertMolarMassKg
            + mixture_.fuelMoles * fuelMolarMassKg_
            + mixture_.burnedMoles * burnedGasMolarMassKg_;
    }
    [[nodiscard]] double meanMolarMassKg() const noexcept {
        return totalMoles() > 1.0e-15 ? massKg() / totalMoles() : airMolarMassKg;
    }
    [[nodiscard]] double momentumKgMps() const noexcept {
        return momentumXKgMps_ * dx_ + momentumYKgMps_ * dy_;
    }
    [[nodiscard]] double momentumXKgMps() const noexcept { return momentumXKgMps_; }
    [[nodiscard]] double momentumYKgMps() const noexcept { return momentumYKgMps_; }
    [[nodiscard]] double bulkVelocityMps() const noexcept {
        return massKg() > 1.0e-12 ? momentumKgMps() / massKg() : 0.0;
    }
    [[nodiscard]] double velocityXMps() const noexcept {
        return massKg() > 1.0e-12 ? momentumXKgMps_ / massKg() : 0.0;
    }
    [[nodiscard]] double velocityYMps() const noexcept {
        return massKg() > 1.0e-12 ? momentumYKgMps_ / massKg() : 0.0;
    }
    [[nodiscard]] double bulkKineticEnergyJoules() const noexcept {
        const auto mass = massKg();
        return mass > 1.0e-12
            ? (momentumXKgMps_ * momentumXKgMps_ + momentumYKgMps_ * momentumYKgMps_)
                / (2.0 * mass)
            : 0.0;
    }

    /**
     * Effective heat-capacity ratio γ = Cp/Cv.
     * Linearly interpolates between air (1.40) and burned combustion products
     * (1.26) based on the burned-gas mole fraction.  This corrects ~8-12 %
     * over-prediction of peak pressure that arises from using γ_air = 1.40
     * for hot post-combustion gases.
     */
    [[nodiscard]] double heatCapacityRatioEffective() const noexcept;
    [[nodiscard]] double molarHeatCapacityCvEffective() const noexcept;

    /**
     * Local isentropic speed of sound: c = √(γ·P/ρ).
     * Used to cap bulk velocity and compute the critical flow function.
     */
    [[nodiscard]] double speedOfSoundMps() const noexcept;

    /**
     * Dynamic pressure contribution in direction (nx, ny).
     *
     * Computes  q = ½·ρ·(v⃗ · n̂)·|v⃗ · n̂|  where v⃗ is the bulk velocity
     * vector along (dx_, dy_).  The sign is preserved so that gas flowing
     * toward a junction adds positive dynamic pressure (helping drive flow)
     * while gas flowing away subtracts it (opposing flow — models inertia).
     *
     * Returns the value in kPa.
     */
    [[nodiscard]] double dynamicPressureKpa(double nx, double ny) const noexcept;

    [[nodiscard]] const GasMixture& mixture() const noexcept { return mixture_; }
    [[nodiscard]] double characteristicAreaM2() const noexcept { return characteristicAreaM2_; }
    [[nodiscard]] double orientationDx() const noexcept { return dx_; }
    [[nodiscard]] double orientationDy() const noexcept { return dy_; }

private:
    friend class ConservativeGasSystem;
    GasMixture mixture_ {};
    double internalEnergyJ_ { 0.0 };
    double volumeM3_ { 0.001 };
    double momentumXKgMps_ { 0.0 };
    double momentumYKgMps_ { 0.0 };
    // Geometry (set via setGeometry)
    double characteristicAreaM2_ { 0.0 }; ///< tube cross-section used for jet momentum
    double dx_ { 1.0 };                   ///< orientation unit vector x
    double dy_ { 0.0 };                   ///< orientation unit vector y
    // Fuel chemistry
    double fuelMolarMassKg_ { gasolineMolarMassKg };
    double oxygenMolesPerFuelMole_ { 12.5 };
    double productMolesPerFuelMole_ { 17.0 };
    double burnedGasMolarMassKg_ { burnedGasMolarMassKg };
};

struct GasFlowResult final {
    double transferredMoles { 0.0 };
    double transferredMassKg { 0.0 };
    bool choked { false };
};

struct SimultaneousGasFlowResult final {
    GasFlowResult first {};
    GasFlowResult second {};
};

struct CombustionReaction final {
    double burnedFuelMoles { 0.0 };
    double releasedEnergyJoules { 0.0 };
    double completeness { 0.0 };
};

/**
 * Parameters for the full physics-aware flow call.
 *
 * Compared to the basic flow(first, second, area, cd, dt) overload this
 * variant also:
 *  - Adds each cell's dynamic pressure contribution to the effective pressure
 *    driving flow (models gas inertia / ram effect).
 *  - Injects jet momentum into both cells proportional to the throat velocity,
 *    capped at the local speed of sound.
 *  - Uses the source cell's effective γ throughout the isentropic relations.
 *
 * crossSectionArea0 / crossSectionArea1: characteristic tube cross-sections
 *    used for the jet velocity calculation.  Pass 0.0 to skip momentum
 *    injection for that side (e.g. large plenum with negligible bulk velocity).
 *    If left as 0.0 the cell's own characteristicAreaM2_ is used instead
 *    (set via GasCell::setGeometry).
 */
struct FlowParameters final {
    GasCell* system0 { nullptr };
    GasCell* system1 { nullptr };
    double effectiveAreaM2 { 0.0 };
    double dischargeCoefficient { 1.0 };
    double dtSeconds { 0.0 };
    // Flow direction unit vector from system0 toward system1.
    double directionX { 1.0 };
    double directionY { 0.0 };
    // Override for jet-momentum cross-section (0 = use cell's stored geometry)
    double crossSectionArea0 { 0.0 };
    double crossSectionArea1 { 0.0 };
};

class ConservativeGasSystem final {
public:
    /**
     * Simple flow: uses static pressure only (no dynamic pressure).
     * Maintains full backward compatibility with existing call sites.
     * Jet momentum is still injected using the cells' stored
     * characteristicAreaM2_, so calling setGeometry() on each cell
     * improves accuracy without changing the call site.
     */
    [[nodiscard]] static GasFlowResult flow(GasCell& first, GasCell& second,
                                            double effectiveAreaM2, double dischargeCoefficient,
                                            double dtSeconds) noexcept;

    /**
     * Full physics-aware flow: includes dynamic pressure contribution and
     * directional jet-momentum injection (see FlowParameters documentation).
     */
    [[nodiscard]] static GasFlowResult flow(const FlowParameters& params) noexcept;

    /**
     * Resolves two restrictions sharing the middle control volume from one
     * common pre-flow state, then commits their conservative deltas together.
     * This removes valve-order bias during intake/exhaust overlap.
     * Contract: first.system1 must be second.system0.
     */
    [[nodiscard]] static SimultaneousGasFlowResult flowSimultaneous(
        const FlowParameters& first, const FlowParameters& second) noexcept;

    [[nodiscard]] static GasFlowResult flowFromBoundary(GasCell& target, double boundaryPressureKpa,
                                                        double boundaryTemperatureK,
                                                        double effectiveAreaM2,
                                                        double dischargeCoefficient,
                                                        double dtSeconds,
                                                        double targetToBoundaryDirectionX = 1.0,
                                                        double targetToBoundaryDirectionY = 0.0) noexcept;
    [[nodiscard]] static CombustionReaction reactGasoline(GasCell& cell, double requestedFraction,
                                                           double efficiency) noexcept;
    [[nodiscard]] static CombustionReaction reactFuel(GasCell& cell, double requestedFraction,
                                                       double efficiency,
                                                       double lowerHeatingValueJPerKg) noexcept;
    [[nodiscard]] static CombustionReaction reactFuelMoles(GasCell& cell,
                                                            double requestedFuelMoles,
                                                            double efficiency,
                                                            double lowerHeatingValueJPerKg) noexcept;
    [[nodiscard]] static double totalMoles(const GasCell& first, const GasCell& second) noexcept {
        return first.totalMoles() + second.totalMoles();
    }
    [[nodiscard]] static double totalEnergy(const GasCell& first, const GasCell& second) noexcept {
        return first.internalEnergyJoules() + second.internalEnergyJoules()
            + first.bulkKineticEnergyJoules() + second.bulkKineticEnergyJoules();
    }
    [[nodiscard]] static double totalMomentum(const GasCell& first, const GasCell& second) noexcept {
        const auto x = first.momentumXKgMps() + second.momentumXKgMps();
        const auto y = first.momentumYKgMps() + second.momentumYKgMps();
        return std::hypot(x, y);
    }
    [[nodiscard]] static double totalMomentumX(const GasCell& first, const GasCell& second) noexcept {
        return first.momentumXKgMps() + second.momentumXKgMps();
    }
    [[nodiscard]] static double totalMomentumY(const GasCell& first, const GasCell& second) noexcept {
        return first.momentumYKgMps() + second.momentumYKgMps();
    }

private:
    static GasFlowResult transfer(GasCell& source, GasCell& sink, double moles) noexcept;
    static void restorePairEnergy(GasCell& first, GasCell& second,
                                  double targetEnergyJoules) noexcept;
    [[nodiscard]] static double pressureEquilibriumMoles(const GasCell& source,
                                                         const GasCell& sink,
                                                         double directionX,
                                                         double directionY,
                                                         double requestedMoles,
                                                         bool includeDynamicPressure) noexcept;

    /**
     * Core isentropic mass-flow equation.
     * gamma : effective heat-capacity ratio of the source gas.
     * specificGasConstant : R / M_bar for the source gas [J/(kg·K)].
     */
    [[nodiscard]] static double massFlowKgPerSecond(double upstreamPressurePa,
                                                    double downstreamPressurePa,
                                                    double temperatureK,
                                                    double areaM2,
                                                    double coefficient,
                                                    bool& choked,
                                                    double gamma,
                                                    double specificGasConstant) noexcept;
};

} // namespace enginelab
