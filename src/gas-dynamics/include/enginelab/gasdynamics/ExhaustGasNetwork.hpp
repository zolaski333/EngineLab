#pragma once

#include <enginelab/gasdynamics/ExhaustNetworkLayout.hpp>
#include <enginelab/gasdynamics/FiniteVolumeDuct.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace enginelab::gasdynamics {

struct ExhaustGasNetworkConfig final {
    double initialPressurePa { 101'325.0 };
    double initialTemperatureK { 300.0 };
    GasComposition initialComposition { GasComposition::dryAir() };
    double absoluteRoughnessM { 1.5e-6 };
    double wallHeatTransferWPerM2K { 0.0 };
    double wallTemperatureK { 450.0 };
    bool dynamicWallHeatTransferEnabled { false };
    double wallThicknessM { 0.0015 };
    double wallDensityKgPerM3 { 7'900.0 };
    double wallSpecificHeatJPerKgK { 500.0 };
    double externalWallHeatTransferWPerM2K { 0.0 };
    double externalTemperatureK { 300.0 };
    /** See `FiniteVolumeDuctGeometry::wallHeatUpdateIntervalSeconds`. Zero
     *  keeps the wall exchange on every solver sub-step. */
    double wallHeatUpdateIntervalSeconds { 0.0 };
    /**
     * Ignore the interval above and run the wall exchange only when
     * `requestWallHeatUpdate()` says so.
     *
     * Set this for networks advanced CONCURRENTLY with siblings. A network
     * timing its own sub-rating from its own accepted sub-steps drifts out of
     * phase with its siblings within a few mechanical sub-steps, and a barrier
     * costs the maximum over participants rather than the mean: measured on
     * the LS3, self-timed sub-rating made the duct solver 24% cheaper, gained
     * 19% with the pool disabled, and still lost 13% with it enabled, because
     * the wall burst landed on a different dispatch for every cylinder.
     */
    bool wallHeatUpdateExternallyTriggered { false };
    /** Use one conservative forward-Euler stage per accepted substep instead
     * of SSP-RK2. Intended for deliberately reduced, multirate intake
     * networks; the exhaust and offline oracle keep RK2. */
    bool firstOrderTimeIntegration { false };
    /** Evolve graph-axis momentum through a lumped merge/splitter.
     *
     * False preserves a deliberately well-mixed plenum. True carries the
     * momentum left after the junction wall balances its static pressure, which
     * is the appropriate model for an exhaust collector with a directed trunk. */
    bool evolveJunctionAxialMomentum { false };
    double maximumCourantNumber { 0.42 };
    std::size_t maximumSubstepsPerAdvance { 100'000 };

    [[nodiscard]] bool valid() const noexcept;
};

/** Frozen cylinder state and valve conductance for one requested advance. */
struct CylinderValveBoundary final {
    std::uint32_t cylinderId { 0 };
    ConservativeState cylinderState {};
    /** Physical chamber volume used to evolve the reservoir during subcycling. */
    double cylinderVolumeM3 { 0.0 };
    /** Geometric curtain/seat area before the valve discharge coefficient. */
    double effectiveValveAreaM2 { 0.0 };
    double dischargeCoefficient { 1.0 };
};

/** Atmosphere seen by all configured outlets during one requested advance. */
struct ExhaustAmbientBoundary final {
    ConservativeState reservoirState {};
    /** 0 closes every outlet, 1 uses the authored opening conductance. */
    double openingScale { 1.0 };
};

/** Time-integrated transfer, positive from the cylinder into the network. */
struct CylinderGasExchange final {
    std::uint32_t cylinderId { 0 };
    std::uint32_t pathIndex { 0 };
    std::array<double, gasSpeciesCount> speciesMassKg {};
    double totalEnergyJ { 0.0 };
    double axialMomentumImpulseNs { 0.0 };
    double cylinderPressurePaAfter { 0.0 };
    double cylinderTemperatureKAfter { 0.0 };
    double networkPressurePa { 0.0 };
    double networkTemperatureK { 0.0 };
    double networkDensityKgPerM3 { 0.0 };
    double networkVelocityMps { 0.0 };
    double networkSpeedOfSoundMps { 0.0 };

    [[nodiscard]] double totalMassKg() const noexcept;
};

/** Instantaneous valve-plane flow evaluated against the current network state.
 *
 * Unlike CylinderGasExchange this is not a time-integrated transfer and does
 * not mutate either reservoir. It exists so a faster mechanical/audio cadence
 * can observe the same Riemann boundary used by a multirate network advance.
 * Samples are returned in compiled cylinder-port order.
 */
struct CylinderBoundaryFlowSample final {
    std::uint32_t cylinderId { 0 };
    std::uint32_t pathIndex { 0 };
    /** Positive from the cylinder into the exhaust network. */
    double massFlowKgPerSecond { 0.0 };
    double networkPressurePa { 0.0 };
    double networkTemperatureK { 0.0 };
    double networkDensityKgPerM3 { 0.0 };
    double networkVelocityMps { 0.0 };
    double networkSpeedOfSoundMps { 0.0 };
    bool valid { false };
};

/** Physical state and mean transfer at one terminal opening.
 * Transfer signs are positive from the network into the environment.
 */
struct ExhaustOutletFlowSample final {
    std::uint32_t outletNodeId { 0 };
    std::uint32_t pathIndex { 0 };
    double staticPressurePa { 0.0 };
    double temperatureK { 0.0 };
    double axialVelocityMps { 0.0 };
    double densityKgPerM3 { 0.0 };
    double speedOfSoundMps { 0.0 };
    double massFlowKgPerS { 0.0 };
    double volumeFlowM3PerS { 0.0 };
    double totalEnergyFlowW { 0.0 };
    double openingAreaM2 { 0.0 };
    std::array<double, gasSpeciesCount> speciesMassKg {};
    double transferredEnergyJ { 0.0 };
};

struct ExhaustNetworkInventory final {
    std::array<double, gasSpeciesCount> speciesMassKg {};
    double totalEnergyJ { 0.0 };
    double resolvedAxialMomentumKgMps { 0.0 };
    double wallThermalEnergyJ { 0.0 };
};

struct ExhaustNetworkAdvanceResult final {
    double advancedTimeSeconds { 0.0 };
    std::size_t acceptedSubsteps { 0 };
    std::size_t rejectedSubsteps { 0 };
    bool completed { true };
    double wallHeatRejectedJ { 0.0 };
};

struct ExhaustFuelReactionConfig final {
    /** Ignition threshold, compared against the hotter of the bulk gas and the
     * pipe wall the gas is touching -- see `reactUnburnedFuel`. */
    double ignitionTemperatureK { 900.0 };
    double reactionTimeConstantSeconds { 0.010 };
    double reactionEfficiency { 0.95 };
    double oxygenMolesPerFuelMole { 12.5 };
    double fuelMolarMassKg { 0.114 };
    double lowerHeatingValueJPerKg { 44'000'000.0 };
    /** Residence time required at or above ignitionTemperatureK. The threshold
     * is the reference condition: this value is not silently divided by an
     * additional temperature ramp. */
    double inductionTimeSeconds { 0.004 };
    double minimumEquivalenceRatio { 0.45 };
    double maximumEquivalenceRatio { 1.80 };
    double quenchTemperatureK { 520.0 };
};

/** One conservative, spatially resolved heat-release source. The finite-volume
 * state remains the sole owner of mass and low-band energy; this descriptor is
 * only a location/time/energy observation for the high-band acoustic solver. */
struct ExhaustFuelReactionSource final {
    std::uint32_t nodeId { 0 };
    std::uint32_t sourceComponentId { 0 };
    std::uint32_t pathIndex { 0 };
    double axialPosition { 0.5 };
    double releasedEnergyJoules { 0.0 };
    double burnedFuelMassKg { 0.0 };
    double durationSeconds { 0.0 };
    double densityKgPerM3 { 0.0 };
    double speedOfSoundMps { 0.0 };
    double flowAreaM2 { 0.0 };
};

struct ExhaustFuelReactionResult final {
    // Coalesced per physical node, not per cell. Keeping this aligned with the
    // pressure-queue transport bounds EngineSimulator::step's already large
    // fixed stack frame on Windows.
    static constexpr std::size_t maximumSources { 32 };
    double burnedFuelMassKg { 0.0 };
    double consumedOxygenMassKg { 0.0 };
    double releasedEnergyJoules { 0.0 };
    std::size_t reactingControlVolumes { 0 };
    /** Control volumes whose bulk gas was below the ignition threshold and
     * which reacted on the wall instead. Zero means every reaction in this
     * call was gas-ignited, which during a closed-throttle overrun means the
     * hot-surface path is not contributing and the model has fallen back to
     * the behaviour that cannot pop. */
    std::size_t wallIgnitedControlVolumes { 0 };
    std::array<ExhaustFuelReactionSource, maximumSources> sources {};
    std::size_t sourceCount { 0 };
    std::size_t droppedSourceCount { 0 };
};

/** Globally coupled finite-volume exhaust network.
 *
 * Every duct and junction participates in the same SSP-RK2 stages. Direct
 * two-port connections exchange one common mass/energy flux; merges and
 * splitters accumulate those same fluxes in finite control volumes. All work
 * buffers and output arrays are allocated by configure().
 */
class ExhaustGasNetwork final {
public:
    explicit ExhaustGasNetwork(
        ThermodynamicModel model = ThermodynamicModel::standardCombustionGas()) noexcept;

    /** Build all duct meshes and work arrays. Not realtime-safe. */
    [[nodiscard]] bool configure(const ExhaustNetworkLayout& layout,
                                 ExhaustGasNetworkConfig config = {});

    /** Restore a configured network to a uniform quiescent state.
     *
     * No topology or work buffers are rebuilt, so this operation is
     * allocation-free and safe for an EngineSimulator::reset() boundary.
     */
    [[nodiscard]] bool reset(double pressurePa,
                             double temperatureK,
                             GasComposition composition = GasComposition::dryAir()) noexcept;

    [[nodiscard]] bool configured() const noexcept { return configured_; }
    [[nodiscard]] const EulerMixtureModel& mixtureModel() const noexcept { return mixtureModel_; }
    [[nodiscard]] const ExhaustNetworkLayout& layout() const noexcept { return layout_; }
    [[nodiscard]] std::span<const FiniteVolumeDuct> ducts() const noexcept { return ducts_; }
    [[nodiscard]] std::span<FiniteVolumeDuct> ducts() noexcept { return ducts_; }
    /** Hottest pipe wall anywhere in the network.
     *
     * This is the afterfire ignition source (see `reactUnburnedFuel`) and it is
     * slow: 1.5 mm of steel is about 5.9 kJ/m2K against an exhaust-side film of
     * a few hundred W/m2K, i.e. a time constant of tens of seconds. An engine
     * that has only just started has a cold pipe and physically cannot pop, so
     * any afterfire measurement has to state how long the exhaust was heated
     * before the overrun. */
    [[nodiscard]] double peakWallTemperatureK() const noexcept;
    [[nodiscard]] std::span<const ConservativeState> junctionStates() const noexcept {
        return junctionStates_;
    }
    [[nodiscard]] ExhaustNetworkInventory inventory() const noexcept;

    /** Operator-split oxidation of real fuel and oxygen inventories already
     * present in hot exhaust control volumes. Species mass is conserved and
     * chemical heat is added to the same conservative state that drives the
     * pressure/acoustic boundary. No authored fuel or synthetic impulse is
     * introduced. */
    [[nodiscard]] ExhaustFuelReactionResult reactUnburnedFuel(
        double durationSeconds,
        const ExhaustFuelReactionConfig& reaction) noexcept;

    /** Advance the complete network by exactly durationSeconds when successful.
     * Missing cylinder IDs are treated as closed valves. Duplicate supplied IDs
     * are rejected because iteration order must never select the physics.
     */
    [[nodiscard]] ExhaustNetworkAdvanceResult advance(
        double durationSeconds,
        std::span<const CylinderValveBoundary> cylinderBoundaries,
        const ExhaustAmbientBoundary& ambient) noexcept;

    /** Evaluate current signed valve flow without advancing the network.
     *
     * The output span must have room for every compiled cylinder port. Missing
     * boundaries are reported as invalid/closed samples. Unknown or duplicate
     * supplied cylinder IDs reject the complete observation.
     */
    [[nodiscard]] bool sampleCylinderBoundaries(
        std::span<const CylinderValveBoundary> cylinderBoundaries,
        std::span<CylinderBoundaryFlowSample> samples) const noexcept;

    [[nodiscard]] std::span<const CylinderGasExchange> cylinderExchanges() const noexcept {
        return cylinderExchanges_;
    }
    [[nodiscard]] std::span<const ExhaustOutletFlowSample> outletSamples() const noexcept {
        return outletSamples_;
    }

    /**
     * Predicts what one terminal opening would push into `ambient` over
     * `durationSeconds`, from the network's CURRENT state, without advancing
     * anything and without mutating the network.
     *
     * This exists so several networks that share one reservoir can be advanced
     * CONCURRENTLY while still each seeing the reservoir drawn down by the ones
     * ordered before them. The caller walks a scratch copy of the reservoir
     * through these predictions in a fixed order, hands each network the state
     * it should see, advances them all in parallel, and then commits the real
     * transfers. Without it the only option is to freeze the reservoir, which
     * on the engine's intake plenum was measured to converge to the wrong
     * pressure -- see EngineSimulator's runner pass.
     *
     * It is deliberately the first-stage flux only, not the two-stage integral
     * `advance` reports: the point is a same-instant estimate of the ORDERING
     * correction, and the exact transfer replaces it afterwards. A lagged
     * estimate would not do -- the reservoir/duct coupling responds in tens of
     * microseconds and one sub-step of lag drives it into a limit cycle.
     *
     * Returns nullopt for an unknown outlet, a non-positive duration, or a
     * state the primitive recovery rejects.
     */
    [[nodiscard]] std::optional<ExhaustOutletFlowSample> predictOutletTransfer(
        std::size_t outletIndex,
        const ExhaustAmbientBoundary& ambient,
        double durationSeconds) const noexcept;

    /**
     * Run the sub-rated duct wall exchange on the next `advance`, carrying all
     * the simulated time accumulated since the last one. Only consulted when
     * `wallHeatUpdateExternallyTriggered` is set; the flag is cleared once the
     * exchange has actually been applied to an accepted sub-step, so a request
     * cannot be lost to a rejected trial.
     *
     * The caller owns this cadence because only the caller knows the phase.
     * See the config field for what happens when a network times it alone.
     */
    void requestWallHeatUpdate() noexcept { wallHeatUpdateRequested_ = true; }

    /** Add species mass as a source in the duct cell adjacent to a cylinder
     * port, together with its sensible internal energy at temperatureK and an
     * optional additional heat term (negative for evaporation charge cooling).
     *
     * This is the network-side half of a port fuel injector or secondary-air
     * source: the mass appears with zero axial momentum, exactly like a wall
     * film evaporating into the stream. The candidate state is committed only
     * if it stays physically admissible, so a pathological command degrades to
     * a refused injection instead of a poisoned solve. Ports attached to a
     * junction are not supported (no compiled layout produces one for a runner
     * and a junction's primitive cache is not owned by the duct).
     */
    [[nodiscard]] bool injectSpeciesAtPort(std::size_t portIndex,
                                           GasSpecies species,
                                           double massKg,
                                           double temperatureK,
                                           double additionalHeatJ) noexcept;

private:
    struct ConservedFlowRate final {
        std::array<double, gasSpeciesCount> speciesMassKgPerS {};
        double momentumN { 0.0 };
        double totalEnergyW { 0.0 };
    };

    [[nodiscard]] double maximumStableTimeStep(
        std::span<const CylinderValveBoundary> cylinderBoundaries,
        const ExhaustAmbientBoundary& ambient) const noexcept;
    [[nodiscard]] bool evaluateStage(
        bool useStageState,
        std::span<const CylinderValveBoundary> cylinderBoundaries,
        const ExhaustAmbientBoundary& ambient) noexcept;
    [[nodiscard]] bool prepareStageStates(
        bool candidateStage,
        bool deferDynamicWallSources = false) noexcept;
    void updateOutletSamples(double durationSeconds) noexcept;

    EulerMixtureModel mixtureModel_;
    ExhaustNetworkLayout layout_;
    ExhaustGasNetworkConfig config_ {};
    // The network drives every duct with one shared trial step, so the
    // sub-rating accumulator is network-wide rather than per duct: the ducts
    // would otherwise all hold the same value anyway.
    double wallHeatPendingSeconds_ { 0.0 };
    bool wallHeatUpdateRequested_ { false };
    std::vector<FiniteVolumeDuct> ducts_;
    std::vector<ConservativeState> junctionStates_;
    std::vector<ConservativeState> junctionStage_;
    std::vector<ConservativeState> junctionCandidate_;
    std::vector<ConservativeState> junctionResidual_;
    std::vector<ConservativeState> junctionStageResidual_;
    std::vector<PrimitiveState> junctionPrimitives_;
    std::vector<PrimitiveState> junctionStagePrimitives_;
    std::vector<PrimitiveState> junctionCandidatePrimitives_;
    std::vector<double> junctionPortAreaSums_;
    std::vector<ConservativeState> cylinderReservoirStates_;
    std::vector<ConservativeState> cylinderReservoirStage_;
    std::vector<ConservativeState> cylinderReservoirCandidate_;
    std::vector<ConservativeState> cylinderReservoirResidual_;
    std::vector<ConservativeState> cylinderReservoirStageResidual_;
    std::vector<PrimitiveState> cylinderReservoirPrimitives_;
    std::vector<PrimitiveState> cylinderReservoirStagePrimitives_;
    std::vector<PrimitiveState> cylinderReservoirCandidatePrimitives_;
    std::vector<double> cylinderReservoirVolumesM3_;
    std::vector<std::uint8_t> cylinderReservoirActive_;
    /** Port-order lookup into the caller's boundary span, rebuilt once/advance. */
    std::vector<std::size_t> cylinderBoundaryIndices_;
    std::vector<std::uint8_t> ductInletAssigned_;
    std::vector<std::uint8_t> ductOutletAssigned_;
    std::vector<ConservedFlowRate> cylinderFirstStageFlow_;
    std::vector<ConservedFlowRate> cylinderSecondStageFlow_;
    std::vector<ConservedFlowRate> outletFirstStageFlow_;
    std::vector<ConservedFlowRate> outletSecondStageFlow_;
    std::vector<CylinderGasExchange> cylinderExchanges_;
    std::vector<ExhaustOutletFlowSample> outletSamples_;
    struct ReactionSiteState final {
        double inductionSeconds { 0.0 };
        bool burning { false };
        /** Origin of the current flame kernel, latched at ignition rather than
         * re-inferred after the released heat has warmed the gas. */
        bool wallIgnited { false };
    };
    std::vector<std::vector<ReactionSiteState>> ductReactionStates_;
    std::vector<ReactionSiteState> junctionReactionStates_;
    PrimitiveState ambientPrimitive_ {};
    bool configured_ { false };
};

} // namespace enginelab::gasdynamics
