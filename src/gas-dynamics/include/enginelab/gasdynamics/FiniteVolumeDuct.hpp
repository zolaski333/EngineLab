#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <span>
#include <vector>

namespace enginelab::gasdynamics {

class ExhaustGasNetwork;

inline constexpr std::size_t gasSpeciesCount = 4;
inline constexpr double universalGasConstantJPerMolK = 8.31446261815324;

/** Species carried conservatively by the gas network.
 *
 * The order deliberately matches GasMixture. The 1D solver transports mass,
 * rather than mole fraction, so every finite-volume update conserves total
 * mass even when species have different molar masses.
 */
enum class GasSpecies : std::size_t {
    oxygen = 0,
    inert = 1,
    fuel = 2,
    burned = 3,
};

struct SpeciesThermodynamics final {
    double molarMassKgPerMol { 0.0289652 };
    double molarHeatCapacityCvJPerMolK { 20.7861565454 };
};

/** Calorically-perfect mixture model used during one network solve.
 *
 * Heat capacity is species-weighted on every cell. Therefore gamma is allowed
 * to vary spatially with burned-gas and fuel-vapour concentration while each
 * species remains calorically perfect. Fuel constants can be replaced when an
 * EngineConfig selects chemistry other than the default gasoline surrogate.
 */
struct ThermodynamicModel final {
    std::array<SpeciesThermodynamics, gasSpeciesCount> species {};

    [[nodiscard]] static ThermodynamicModel standardCombustionGas() noexcept;
    [[nodiscard]] bool valid() const noexcept;
};

struct GasComposition final {
    std::array<double, gasSpeciesCount> massFractions { 0.0, 1.0, 0.0, 0.0 };

    [[nodiscard]] static GasComposition dryAir() noexcept;
    [[nodiscard]] static GasComposition inertGas() noexcept;
};

/** Cell-average conservative variables per unit volume. */
struct ConservativeState final {
    std::array<double, gasSpeciesCount> speciesMassDensityKgPerM3 {};
    double momentumDensityKgPerM2S { 0.0 };
    double totalEnergyDensityJPerM3 { 0.0 };

    [[nodiscard]] double densityKgPerM3() const noexcept;
};

/** Thermodynamic quantities recovered from a conservative cell state. */
struct PrimitiveState final {
    double densityKgPerM3 { 0.0 };
    double velocityMps { 0.0 };
    double pressurePa { 0.0 };
    double temperatureK { 0.0 };
    double heatCapacityRatio { 1.4 };
    double speedOfSoundMps { 0.0 };
    std::array<double, gasSpeciesCount> massFractions {};
};

/** Instantaneous Euler flux through a unit face area, positive left-to-right. */
struct EulerFlux final {
    std::array<double, gasSpeciesCount> speciesMassFluxKgPerM2S {};
    double momentumFluxPa { 0.0 };
    double totalEnergyFluxWPerM2 { 0.0 };
};

/** Time-integrated boundary flux through a unit face area. */
struct EulerFluxIntegral final {
    std::array<double, gasSpeciesCount> speciesMassKgPerM2 {};
    double momentumImpulseNsPerM2 { 0.0 };
    double totalEnergyJPerM2 { 0.0 };
};

/** Extensive inventory over an entire duct. */
struct ConservedInventory final {
    std::array<double, gasSpeciesCount> speciesMassKg {};
    double axialMomentumKgMps { 0.0 };
    double totalEnergyJ { 0.0 };
};

/** Equation of state and Riemann flux for the transported gas mixture. */
class EulerMixtureModel final {
public:
    explicit EulerMixtureModel(
        ThermodynamicModel model = ThermodynamicModel::standardCombustionGas()) noexcept;

    [[nodiscard]] const ThermodynamicModel& thermodynamics() const noexcept { return model_; }

    /** Build a conservative state from density, pressure and mass fractions.
     * Returns nullopt instead of silently normalising invalid physical inputs.
     */
    [[nodiscard]] std::optional<ConservativeState> conservativeFromPrimitive(
        double densityKgPerM3, double velocityMps, double pressurePa,
        GasComposition composition = GasComposition::dryAir()) const noexcept;

    [[nodiscard]] std::optional<ConservativeState> conservativeFromPressureTemperature(
        double pressurePa, double temperatureK, double velocityMps = 0.0,
        GasComposition composition = GasComposition::dryAir()) const noexcept;

    /** Recover pressure and temperature without thermodynamic floors. */
    [[nodiscard]] std::optional<PrimitiveState> primitiveFromConservative(
        const ConservativeState& state) const noexcept;

    [[nodiscard]] bool isPhysical(const ConservativeState& state) const noexcept;
    /** Remove only signed species residue within floating-point roundoff.
     * Positive species are rescaled to preserve the cell's original total
     * density. Significant negativity is rejected and left untouched.
     */
    [[nodiscard]] bool canonicaliseSpeciesRoundoff(ConservativeState& state) const noexcept;
    [[nodiscard]] EulerFlux physicalFlux(const ConservativeState& state) const noexcept;

    /** HLLC contact-resolving flux with an HLLE safety fallback.
     * Both inputs must satisfy isPhysical().
     */
    [[nodiscard]] EulerFlux riemannFlux(const ConservativeState& left,
                                        const ConservativeState& right) const noexcept;

private:
    ThermodynamicModel model_;
};

struct DuctGeometry final {
    double lengthM { 1.0 };
    /** Zero derives a circular area from diameterM. */
    double crossSectionAreaM2 { 0.0 };
    /** Hydraulic diameter used by wall friction and heat transfer. */
    double diameterM { 0.05 };
    std::size_t cellCount { 32 };
    bool wallFrictionEnabled { true };
    double absoluteRoughnessM { 1.5e-6 };
    /** Concentrated loss K distributed over this component's length. */
    double localLossCoefficient { 0.0 };
    /** Zero disables wall heat exchange. */
    double wallHeatTransferWPerM2K { 0.0 };
    double wallTemperatureK { 300.0 };

    [[nodiscard]] double areaM2() const noexcept;
    [[nodiscard]] double cellLengthM() const noexcept;
    [[nodiscard]] bool valid() const noexcept;
};

enum class DuctBoundaryType {
    transmissive,
    reflective,
    prescribed,
    periodic,
};

struct DuctBoundaryCondition final {
    DuctBoundaryType type { DuctBoundaryType::transmissive };
    ConservativeState prescribedState {};

    [[nodiscard]] static DuctBoundaryCondition transmissive() noexcept;
    [[nodiscard]] static DuctBoundaryCondition reflective() noexcept;
    [[nodiscard]] static DuctBoundaryCondition prescribed(
        const ConservativeState& state) noexcept;
    [[nodiscard]] static DuctBoundaryCondition periodic() noexcept;
};

struct DuctAdvanceResult final {
    double advancedTimeSeconds { 0.0 };
    std::size_t acceptedSubsteps { 0 };
    std::size_t rejectedSubsteps { 0 };
    bool completed { true };
    EulerFluxIntegral leftBoundaryFlux {};
    EulerFluxIntegral rightBoundaryFlux {};
};

/** Preallocated second-order finite-volume solver for one constant-area duct.
 *
 * Spatial reconstruction is monotonised-central TVD. Time integration uses
 * SSP-RK2 and every accepted substep obeys a CFL limit. A non-physical trial
 * is rejected and retried at half step; no density, species or energy floor is
 * injected into the solution.
 */
class FiniteVolumeDuct final {
public:
    explicit FiniteVolumeDuct(
        ThermodynamicModel model = ThermodynamicModel::standardCombustionGas()) noexcept;

    /** Allocate the mesh and initialise every cell. Not realtime-safe. */
    [[nodiscard]] bool configure(const DuctGeometry& geometry,
                                 const ConservativeState& initialState);

    [[nodiscard]] const DuctGeometry& geometry() const noexcept { return geometry_; }
    [[nodiscard]] const EulerMixtureModel& mixtureModel() const noexcept { return mixtureModel_; }
    [[nodiscard]] std::span<const ConservativeState> cells() const noexcept { return cells_; }
    [[nodiscard]] std::span<ConservativeState> cells() noexcept { return cells_; }
    [[nodiscard]] double cellCentreM(std::size_t index) const noexcept;
    [[nodiscard]] ConservedInventory inventory() const noexcept;

    /** CFL/source-limited stable step for the current cell state. */
    [[nodiscard]] double maximumStableTimeStep(double maximumCourantNumber = 0.45) const noexcept;

    /** Advance by an exact requested duration using internal CFL substeps.
     *
     * Boundary integrals use the same RK quadrature as the state update. For a
     * source-free duct they therefore close the extensive conservation balance
     * to roundoff when multiplied by geometry().areaM2(). Periodic boundaries
     * must be selected on both ends.
     */
    [[nodiscard]] DuctAdvanceResult advance(
        double durationSeconds,
        DuctBoundaryCondition left = DuctBoundaryCondition::transmissive(),
        DuctBoundaryCondition right = DuctBoundaryCondition::transmissive(),
        double maximumCourantNumber = 0.45,
        std::size_t maximumSubsteps = 100'000) noexcept;

private:
    friend class ExhaustGasNetwork;
    [[nodiscard]] bool allStatesPhysical(std::span<const ConservativeState> states) const noexcept;
    void computeResidual(std::span<const ConservativeState> states,
                         const DuctBoundaryCondition& left,
                         const DuctBoundaryCondition& right,
                         std::span<ConservativeState> residual,
                         std::span<EulerFlux> faceFluxes) noexcept;

    EulerMixtureModel mixtureModel_;
    DuctGeometry geometry_ {};
    std::vector<ConservativeState> cells_;
    std::vector<ConservativeState> stage_;
    std::vector<ConservativeState> candidate_;
    std::vector<ConservativeState> residual_;
    std::vector<ConservativeState> stageResidual_;
    std::vector<ConservativeState> slopes_;
    std::vector<EulerFlux> faceFluxes_;
    std::vector<EulerFlux> stageFaceFluxes_;
};

} // namespace enginelab::gasdynamics
