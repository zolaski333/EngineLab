#pragma once

#include <algorithm>
#include <cmath>
#include <span>

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

struct GasInventoryDelta;

/**
 * A zero-dimensional gas control volume. Mass and internal energy only move
 * through explicit transfer/reaction operations; pressure and temperature are
 * derived from the conserved state.
 *
 * Physics extensions (v2):
 *  - Thermally perfect species, Cv linear in temperature, so γ follows both
 *    the composition and the temperature: air 1.40 at 300 K, a burnt
 *    stoichiometric charge ~1.27 at 2,000 K.
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
    /** Each species is thermally perfect: Cv(T) = intercept + slope T, in
     *  J/(mol K), least-squares fits to JANAF between 300 and 2,600 K (O2 and
     *  N2 held at 5/2 R at 300 K; the products are 8 CO2 to 9 H2O, within 2 %
     *  of a diesel's 12 to 11.5). A calorically perfect gas kept N2 at 5/2 R
     *  and the products at gamma 1.26 at every temperature, so a burnt charge
     *  expanded at gamma ~1.35 instead of ~1.28: the cycle extracted too much
     *  work and left a cold exhaust. The fuel vapour stays at its constant Cv.
     *  `gasdynamics::ThermodynamicModel::standardCombustionGas` carries the
     *  same laws, so energy crossing the valves means the same temperature. */
    static constexpr double oxygenCvSlopeJPerMolK2 = 0.005292;
    static constexpr double inertCvSlopeJPerMolK2 = 0.003980;
    static constexpr double burnedCvSlopeJPerMolK2 = 0.008630;
    static constexpr double oxygenCvInterceptJPerMolK = molarHeatCapacityCv - 300.0 * oxygenCvSlopeJPerMolK2;
    static constexpr double inertCvInterceptJPerMolK = molarHeatCapacityCv - 300.0 * inertCvSlopeJPerMolK2;
    static constexpr double burnedCvInterceptJPerMolK = 29.536;
    static constexpr double fuelVapourCvJPerMolK = 145.0;
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
    /** Give the cell another volume holding the same gas state: pressure,
     * temperature, composition and velocity stay, the amount of gas follows
     * the volume (a resized part, not a compression). The same volume changes
     * nothing. */
    void resizeKeepingState(double volumeLitres) noexcept;
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
    /** Atomically apply an externally solved conservative transfer.
     * Returns false and leaves the cell unchanged if species or sensible
     * energy would become physically negative.
     */
    [[nodiscard]] bool tryApplyInventoryDelta(const GasInventoryDelta& delta) noexcept;

    /** Apply Darcy-Weisbach distributed and local flow losses to bulk momentum.
     *  Lost kinetic energy is returned to the gas as heat.
     */
    void applyFlowResistance(double lengthM, double hydraulicDiameterM,
                             double absoluteRoughnessM, double localLossCoefficient,
                             double dtSeconds) noexcept;

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
     * Local heat-capacity ratio γ = Cp/Cv = 1 + R/Cv(T), evaluated on the
     * current species-mole-weighted Cv at the cell's temperature (see
     * molarHeatCapacityCvEffective): 1.40 for air at 300 K, ~1.27 for a
     * stoichiometric charge's products at 2,000 K.
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
    [[nodiscard]] double configuredFuelMolarMassKg() const noexcept { return fuelMolarMassKg_; }
    [[nodiscard]] double configuredBurnedGasMolarMassKg() const noexcept {
        return burnedGasMolarMassKg_;
    }

    /** The internal energy the cell's gas holds at `temperatureK`,
     *  integrated from 0 K: sum n (intercept T + slope T^2 / 2). */
    [[nodiscard]] double internalEnergyAtTemperatureJ(double temperatureK) const noexcept;

private:
    friend class ConservativeGasSystem;
    /** The extensive heat capacity's two coefficients, sum n intercept
     *  (J/K) and sum n slope (J/K^2). */
    struct HeatCapacityCoefficients final {
        double interceptJPerK {};
        double slopeJPerK2 {};
    };
    [[nodiscard]] HeatCapacityCoefficients heatCapacityCoefficients() const noexcept;
    /** The temperature at which the cell's gas holds `internalEnergyJ`: the
     *  positive root of intercept T + slope T^2 / 2 = U. */
    [[nodiscard]] static double temperatureForEnergy(
        HeatCapacityCoefficients coefficients, double internalEnergyJ) noexcept;
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

/** Change in all extensive state carried by one frozen-state flow branch. */
struct GasInventoryDelta final {
    GasMixture mixture {};
    double internalEnergyJ { 0.0 };
    double momentumXKgMps { 0.0 };
    double momentumYKgMps { 0.0 };
};

/** One branch of an N-way Jacobi flow around a shared control volume.
 *
 * `counterpartDelta` is captured immediately after evaluating the branch, while
 * `counterpart` points at that branch cell's current state when the shared deltas
 * are committed. `sharedTotalEnergyDeltaJ` includes sensible and bulk kinetic
 * energy and is used to remove the nonlinear kinetic-energy cross term created
 * when several momentum deltas are combined in the shared cell. Counterpart
 * pointers in one transaction must be non-null and unique. */
struct JacobiGasFlowBranch final {
    GasCell* counterpart { nullptr };
    GasInventoryDelta sharedDelta {};
    GasInventoryDelta counterpartDelta {};
    double sharedTotalEnergyDeltaJ { 0.0 };
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
    // Optional sub-grid ram/resonance pressure added to each cell's effective
    // pressure at the interface (kPa). A 0-D runner cell stores only its net
    // (static) state and cannot represent the inertial/acoustic pulse that keeps
    // charge flowing into the cylinder past the point of static equilibrium. This
    // bias raises the charging pressure the valve sees AND the equilibrium fill
    // point together, so the cylinder can be filled above static manifold density
    // at the tuned speed (inertia/resonance supercharging). It changes the driving
    // pressure and the equilibrium bound only; every mole transferred still comes
    // from the source cell's real inventory, so mass and energy stay conserved.
    // Default 0.0 leaves every existing call site bit-identical.
    //
    // Currently NO caller passes a non-zero bias, and that is a measured result, not
    // an oversight. Raising the equilibrium bound is a necessary half of intake ram
    // charging, but it is not sufficient and it cannot be the whole mechanism: four
    // candidate bias sources have been measured and refuted, including the textbook
    // -rho*L*du/dt (unstable here by construction -- an orifice already sets flow
    // from delta-P, so a bias built from that flow's derivative over-determines it
    // with a gain that grows as dt shrinks) and rho*u^2/2 (stable, but it peaks at
    // full lift and the surplus is expelled again before the valve seats). Ram is
    // irreversible in a real engine because the flow LAGS and closure traps it, so
    // the missing piece belongs in the flow law as a relaxation, with a bias like
    // this one only permitting the overshoot. Read physics-audit.md (archive/docs-2026-09 tag)
    // "L'inertance de runner" before using these fields.
    double biasKpa0 { 0.0 };
    double biasKpa1 { 0.0 };
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
     * directional bulk-momentum injection (see FlowParameters documentation).
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

    [[nodiscard]] static GasInventoryDelta inventoryDelta(
        const GasCell& after, const GasCell& before) noexcept;

    /** Commit N branches evaluated against the same frozen shared cell.
     *
     * Outgoing branches are scaled together if their aggregate request would
     * overdraw any species or sensible energy. The matching correction is
     * applied to each counterpart, so mass is never created by a zero floor.
     * The final shared/counterpart energy is corrected for the nonlinear kinetic
     * cross term while the fixed input order determines every reduction. */
    static void commitJacobiFlows(GasCell& shared,
                                  std::span<const JacobiGasFlowBranch> branches) noexcept;

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
                                                         bool includeDynamicPressure,
                                                         double biasSourceKpa = 0.0,
                                                         double biasSinkKpa = 0.0) noexcept;

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
