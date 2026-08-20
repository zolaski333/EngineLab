#pragma once

#include <enginelab/physics/DuctWallHeatTransferModel.hpp>

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
    friend class FiniteVolumeDuct;
    friend class ExhaustGasNetwork;

    /** Recover a primitive state into caller-owned storage. This is the hot-path
     * counterpart of primitiveFromConservative(); both enforce exactly the
     * same physical admissibility conditions.
     */
    [[nodiscard]] bool recoverPrimitive(const ConservativeState& state,
                                        PrimitiveState& primitive) const noexcept;
    [[nodiscard]] EulerFlux physicalFluxPrepared(
        const ConservativeState& state,
        const PrimitiveState& primitive) const noexcept;
    [[nodiscard]] EulerFlux riemannFluxPrepared(
        const ConservativeState& left,
        const PrimitiveState& leftPrimitive,
        const ConservativeState& right,
        const PrimitiveState& rightPrimitive) const noexcept;

    ThermodynamicModel model_;
    std::array<double, gasSpeciesCount> inverseMolarMassKg_ {};
    std::array<double, gasSpeciesCount> specificHeatCapacityCvJPerKgK_ {};
    bool modelIsValid_ { false };
};

struct DuctGeometry final {
    double lengthM { 1.0 };
    /** Reference/constant area. Zero derives a circular area from diameterM. */
    double crossSectionAreaM2 { 0.0 };
    /** Optional end-face areas for a circular, linearly tapered quasi-1D duct.
     *
     * Zero inherits the reference area. Setting either value enables a real
     * finite-volume area variation. Radius varies linearly between the end
     * faces (a conical frustum), face fluxes are multiplied by local area and
     * the momentum equation receives the p*dA/dx wall-force source.
     */
    double inletCrossSectionAreaM2 { 0.0 };
    double outletCrossSectionAreaM2 { 0.0 };
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
    /** Enable a finite-capacity wall instead of the fixed-temperature source. */
    bool dynamicWallHeatTransferEnabled { false };
    double wallThicknessM { 0.0015 };
    double wallDensityKgPerM3 { 7'900.0 };
    double wallSpecificHeatJPerKgK { 500.0 };
    double externalWallHeatTransferWPerM2K { 0.0 };
    double externalTemperatureK { 300.0 };
    /**
     * Advance the wall exchange only once this much simulated time has
     * accumulated, in one lump carrying the accumulated duration. Zero keeps
     * it on every solver sub-step.
     *
     * The exchange is a slow process sampled absurdly finely: a runner cell
     * moves about 0.04% of the gas-wall equilibrium gap per sub-step, a time
     * constant near 69 ms integrated every ~26 us. Sub-rating it is worth far
     * more than the wall arithmetic itself, because skipping the exchange also
     * skips the extra `recoverPrimitiveStates` pass that only exists to feed
     * it -- measured at 6.2% (coefficient chain), 11.0% (exchange) and ~10.8%
     * (the extra recover) of the duct solver respectively.
     *
     * This is NOT the averaging trap documented in CLAUDE.md: no state is
     * averaged before entering a non-linear law. The heat-transfer
     * coefficient is *sampled* less often, and the exchange it then drives is
     * the same exact two-capacity solution over a longer interval.
     */
    double wallHeatUpdateIntervalSeconds { 0.0 };

    /** Optional cellular-bundle thermal geometry. The flow remains one duct,
     * but its wall state represents all channel walls plus the outer can. */
    bool homogenisedCellularSubstrate { false };
    double cellularSubstrateOpenAreaRatio { 1.0 };
    double cellularSubstrateVolumetricHeatCapacityJPerM3K { 0.0 };

    /** Length-mean area (and therefore volume / length). */
    [[nodiscard]] double areaM2() const noexcept;
    [[nodiscard]] double inletAreaM2() const noexcept;
    [[nodiscard]] double outletAreaM2() const noexcept;
    [[nodiscard]] double faceAreaM2(std::size_t faceIndex) const noexcept;
    [[nodiscard]] double cellAreaM2(std::size_t cellIndex) const noexcept;
    [[nodiscard]] double cellVolumeM3(std::size_t cellIndex) const noexcept;
    [[nodiscard]] double cellHydraulicDiameterM(
        std::size_t cellIndex) const noexcept;
    [[nodiscard]] bool hasVariableArea() const noexcept;
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
    double wallHeatRejectedJ { 0.0 };
};

/** Preallocated second-order quasi-1D finite-volume solver.
 *
 * Spatial reconstruction is monotonised-central TVD. Time integration uses
 * SSP-RK2 and every accepted substep obeys a CFL limit. A non-physical trial
 * is rejected and retried at half step; no density, species or energy floor is
 * injected into the solution. Constant-area ducts retain the exact legacy
 * equations; a configured taper uses local face areas and the conservative
 * geometric momentum source.
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
    /** Mutable access is intended for initial-condition and test setup only.
     * It invalidates derived thermodynamic/source caches; the next advance or
     * CFL query rebuilds them before using the modified states.
     */
    [[nodiscard]] std::span<ConservativeState> cells() noexcept {
        cellStateCacheIsValid_ = false;
        return cells_;
    }
    [[nodiscard]] double cellCentreM(std::size_t index) const noexcept;
    [[nodiscard]] ConservedInventory inventory() const noexcept;

    /** Primitive state of every cell, recovered from the conservative state.
     *
     * Returns an empty span if the recovery fails, which is the same condition
     * that stops an advance. The cache is refreshed on demand and shared with
     * the solver, so repeated queries between advances cost nothing.
     */
    [[nodiscard]] std::span<const PrimitiveState> cellPrimitives() const noexcept {
        return refreshCellStateCache() ? std::span<const PrimitiveState>(cellPrimitives_)
                                       : std::span<const PrimitiveState> {};
    }

    /** Length-mean density and speed of sound over the duct.
     *
     * This is the acoustic medium of the duct as a whole: what a wave travelling
     * its length actually propagates through. Returns false and leaves the
     * outputs untouched when the state cannot be recovered.
     */
    [[nodiscard]] bool meanAcousticMedium(double& densityKgPerM3,
                                          double& speedOfSoundMps) const noexcept;
    [[nodiscard]] std::span<const DuctWallThermalState> wallStates() const noexcept {
        return wallStates_;
    }
    [[nodiscard]] double wallThermalEnergyJ() const noexcept;

    /** CFL/source-limited stable step for the current cell state. */
    [[nodiscard]] double maximumStableTimeStep(double maximumCourantNumber = 0.45) const noexcept;

    /** Advance by an exact requested duration using internal CFL substeps.
     *
     * Boundary integrals use the same RK quadrature as the state update. For a
     * source-free duct they therefore close the extensive conservation balance
     * to roundoff when the left and right integrals are multiplied by their
     * respective end-face areas. Periodic boundaries must be selected on both
     * ends and require equal end-face areas.
     */
    [[nodiscard]] DuctAdvanceResult advance(
        double durationSeconds,
        DuctBoundaryCondition left = DuctBoundaryCondition::transmissive(),
        DuctBoundaryCondition right = DuctBoundaryCondition::transmissive(),
        double maximumCourantNumber = 0.45,
        std::size_t maximumSubsteps = 100'000) noexcept;

private:
    friend class ExhaustGasNetwork;
    [[nodiscard]] bool prepareStateCache(
        std::span<const ConservativeState> states,
        std::span<PrimitiveState> primitives,
        std::span<ConservativeState> sourceTerms,
        double& maximumSignalSpeed,
        double& sourceLimitedTimeStep) const noexcept;
    [[nodiscard]] bool recoverPrimitiveStates(
        std::span<const ConservativeState> states,
        std::span<PrimitiveState> primitives) const noexcept;
    [[nodiscard]] bool refreshCellStateCache() const noexcept;
    [[nodiscard]] bool applyDynamicWallHeatTransfer(
        std::span<ConservativeState> states,
        std::span<const PrimitiveState> primitives,
        std::span<DuctWallThermalState> wallStates,
        double durationSeconds,
        double& heatRejectedJ) const noexcept;
    void resetWallTemperature(double temperatureK) noexcept;
    [[nodiscard]] bool computeResidual(
        std::span<const ConservativeState> states,
        std::span<const PrimitiveState> primitives,
        std::span<const ConservativeState> sourceTerms,
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
    mutable std::vector<PrimitiveState> cellPrimitives_;
    std::vector<PrimitiveState> stagePrimitives_;
    std::vector<PrimitiveState> candidatePrimitives_;
    mutable std::vector<ConservativeState> cellSourceTerms_;
    std::vector<ConservativeState> stageSourceTerms_;
    std::vector<ConservativeState> candidateSourceTerms_;
    std::vector<ConservativeState> reconstructedLeft_;
    std::vector<ConservativeState> reconstructedRight_;
    std::vector<PrimitiveState> reconstructedLeftPrimitives_;
    std::vector<PrimitiveState> reconstructedRightPrimitives_;
    std::vector<EulerFlux> faceFluxes_;
    std::vector<EulerFlux> stageFaceFluxes_;
    std::vector<DuctWallThermalState> wallStates_;
    std::vector<DuctWallThermalState> candidateWallStates_;
    std::vector<DuctWallHeatTransferGeometry> wallHeatTransferGeometries_;
    // Geometry is immutable after configure(). Cache every per-cell term used
    // by the RK hot path so a conical duct does not repeat sqrt/pow/lerp work
    // for each cell, each stage and every acoustic substep.
    std::vector<double> faceAreasM2_;
    std::vector<double> cellVolumesM3_;
    std::vector<double> inverseCellVolumesM3_;
    std::vector<double> hydraulicDiametersM_;
    std::vector<double> turbulentRoughnessTerms_;
    std::vector<double> wallHeatConductancePerVolumes_;
    double cellLengthM_ { 0.0 };
    double localLossGradientPerM_ { 0.0 };
    mutable double maximumCellSignalSpeedMps_ { 0.0 };
    mutable double cellSourceLimitedTimeStepSeconds_ { 0.0 };
    double maximumStageSignalSpeedMps_ { 0.0 };
    double stageSourceLimitedTimeStepSeconds_ { 0.0 };
    double maximumCandidateSignalSpeedMps_ { 0.0 };
    double candidateSourceLimitedTimeStepSeconds_ { 0.0 };
    // Simulated time advanced since the wall exchange last ran. Only ever
    // committed on an accepted sub-step, so a rejected trial cannot leak into
    // it and the sub-rating stays deterministic.
    double wallHeatPendingSeconds_ { 0.0 };
    mutable bool cellStateCacheIsValid_ { false };
};

} // namespace enginelab::gasdynamics
