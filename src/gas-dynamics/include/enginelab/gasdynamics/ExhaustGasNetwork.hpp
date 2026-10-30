#pragma once

#include <enginelab/gasdynamics/ExhaustNetworkLayout.hpp>
#include <enginelab/gasdynamics/FiniteVolumeDuct.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
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
     * of SSP-RK2. Alone (no `hancockPredictor`) it amplifies a closed duct's
     * acoustic mode and stays a diagnostic. A fully closed graph face then
     * takes the reflected Riemann pressure instead of the cell's own. */
    bool firstOrderTimeIntegration { false };
    /** With `firstOrderTimeIntegration`: predict every duct's reconstructed
     * face states half a step ahead before the face solves (MUSCL-Hancock),
     * so the single stage is second order in time inside the ducts. Graph
     * boundaries still read the end cells' averages at the start of the step.
     * One flux pass and one state cache per substep instead of RK2's two.
     * Diagnostic: on the RD350LC-like at 75 mm it lost half the torque at
     * 4,000 rpm against RK2 on the same mesh (journal 2026-10-15). */
    bool hancockPredictor { false };
    /** Evolve graph-axis momentum through a lumped merge/splitter.
     *
     * False preserves a deliberately well-mixed plenum. True carries the
     * momentum left after the junction wall balances its static pressure, which
     * is the appropriate model for an exhaust collector with a directed trunk. */
    bool evolveJunctionAxialMomentum { false };
    /** Meter a valve at a duct inlet with the characteristic (Benson-type)
     * boundary: the face state lies on the interior's wave curve and the
     * valve nozzle closes the problem. False keeps the quasi-steady nozzle
     * against the first cell's average, whose stiffness near zero pressure
     * difference makes the result depend on the Courant number. */
    bool characteristicValveBoundary { false };
    /** Reconstruct across every open duct-to-duct interface: each end cell
     * takes a limited slope through its neighbour's end cell, and the
     * interface's Riemann problem joins the two reconstructed faces instead
     * of the two cell averages. False keeps both end cells first order, which
     * leaves a duct of one or two cells entirely first order. */
    bool reconstructAcrossInterfaces { false };
    /** Diagnostic, with reconstructAcrossInterfaces: a duct end with no
     * neighbour (a port, an outlet, a junction) takes the one-sided slope of
     * its two end cells. */
    bool extrapolateOpenEnds { false };
    /** Diagnostic: every duct reconstructs third-order (kappa = 1/3) face
     * values, limited to the MC slope's bounds. */
    bool thirdOrderReconstruction { false };
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
    /** For an advance that runs AHEAD of the chamber: the reservoir volume
     * moves linearly at its rate, the piston doing p dV work on the reservoir;
     * the valve is shut before `valveOpensAfterSeconds` and from
     * `valveClosesAfterSeconds` on, and in between its area moves linearly from
     * the value above, starting at the opening. The defaults keep a fixed
     * reservoir and a fixed valve. */
    double cylinderVolumeRateM3PerS { 0.0 };
    double effectiveValveAreaRateM2PerS { 0.0 };
    double valveOpensAfterSeconds { 0.0 };
    double valveClosesAfterSeconds { std::numeric_limits<double>::infinity() };
    /** For an advance AHEAD of the chamber: what leaves the reservoir through
     * its other openings over the advance (an exhaust valve during overlap),
     * as species and total-energy rates. Zero for a reservoir with only this
     * valve. A net outflow is applied as its mass rate relative to the
     * starting content, leaving with the reservoir's own composition and
     * enthalpy: the rates may describe gas that has already left. */
    std::array<double, gasSpeciesCount> otherOutflowSpeciesKgPerS {};
    double otherOutflowEnergyW { 0.0 };
};

/** Shared reservoir seen by all configured outlets during one requested advance. */
struct ExhaustAmbientBoundary final {
    ConservativeState reservoirState {};
    /** 0 closes every outlet, 1 uses the authored opening conductance. */
    double openingScale { 1.0 };
    /** Physical volume of a well-mixed reservoir, evolved with the network's
     * stages when positive. Zero preserves an infinite fixed atmosphere.
     * The caller owns its inventory and applies the returned outlet transfers
     * once after the advance; this volume does not add to network inventory.
     */
    double reservoirVolumeM3 { 0.0 };
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
    // Accepted-step observations only; zero when no step is accepted.
    double minimumAcceptedTimeStepSeconds { 0.0 };
    double maximumAcceptedTimeStepSeconds { 0.0 };
    // Ratio to the coded cap, which includes CFL and source/boundary limits.
    double maximumAcceptedStableStepRatio { 0.0 };
    // dt * max(|u| + c) / cell length, measured before the commit.
    double maximumAcceptedDuctCourantCurrent { 0.0 };
    // The validated Euler predictor used by the RK2 second stage.
    // It is also observed in first-order mode, where it is the candidate.
    double maximumAcceptedDuctCourantPredictor { 0.0 };
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
    /** Reference delay at ignitionTemperatureK, stoichiometry and
     * inductionReferencePressureKpa. */
    double inductionTimeSeconds { 0.004 };
    double inductionReferencePressureKpa { 101.325 };
    /** Ea/R in kelvin for the normalised Arrhenius delay. Zero makes
     * temperature neutral for backwards compatibility. */
    double inductionActivationTemperatureK { 0.0 };
    /** tau is proportional to pressure^-n and equivalenceRatio^m. */
    double inductionPressureExponent { 0.0 };
    double inductionEquivalenceRatioExponent { 0.0 };
    /** Full decay time of an unlit Livengood-Wu integral while the mixture
     * remains flammable; a non-flammable inventory resets immediately. */
    double inductionDecayTimeSeconds { 0.008 };
    double minimumEquivalenceRatio { 0.45 };
    double maximumEquivalenceRatio { 1.80 };
    double quenchTemperatureK { 520.0 };
    /** Mixing time of an unburned pocket (see `emitUnburnedPocket`): its
     * temperature relaxes to the gas around it at this rate, and after three
     * of them it has mixed into the cells. Zero: no pockets. */
    double pocketMixingTimeSeconds { 0.0 };
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
    /** Largest local Livengood-Wu integral observed during this call. */
    double maximumInductionIntegral { 0.0 };
    /** Smallest finite local correlation delay evaluated during this call.
     * Zero means that no unlit site was induction-eligible. */
    double minimumInductionDelaySeconds { 0.0 };
    /** Largest finite local correlation delay evaluated during this call.
     * Zero means that no unlit site was induction-eligible. */
    double maximumInductionDelaySeconds { 0.0 };
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

    /** Take over the gas and wall state of a network of the same topology
     * (ExhaustNetworkLayout::sameTopology) compiled at other sizes or another
     * mesh: an exhaust resized while the engine runs.
     *
     * Each duct is remapped along its normalised length: a new cell takes the
     * overlap-weighted mean of the old cells it covers, for the gas (per unit
     * volume, so pressure, temperature, velocity and composition stay where
     * they were along the pipe) and for the wall temperature; a reaction site
     * takes the old site at its centre. Junctions, cylinder reservoirs, the
     * valve solves' warm starts and the wall-exchange clocks are copied. A duct with the same cell count is
     * copied exactly, so adopting from an identical network changes nothing.
     * Mass is conserved only where the volume is: a longer or wider pipe holds
     * more gas at the same state.
     *
     * Allocation-free. Returns false, leaving this network unchanged, when a
     * network is unconfigured, the topologies differ or a remapped state is
     * not physical. */
    [[nodiscard]] bool adoptStateFrom(const ExhaustGasNetwork& previous) noexcept;

    /** Bring the gas to rest at one pressure: every cell and junction keeps
     * its temperature and composition, loses its velocity and takes
     * `pressurePa`. Walls and reaction sites are untouched.
     *
     * For a network whose boundaries hold still, such as an intake runner
     * behind a stopped crank: its explicit reservoir coupling amplifies
     * roundoff into a limit cycle that a quiescent duct cannot have.
     * Allocation-free. Returns false, leaving this network unchanged, when a
     * settled state is not physical. */
    [[nodiscard]] bool settleAtRest(double pressurePa) noexcept;
    [[nodiscard]] bool configured() const noexcept { return configured_; }
    [[nodiscard]] const EulerMixtureModel& mixtureModel() const noexcept { return mixtureModel_; }
    [[nodiscard]] const ExhaustNetworkLayout& layout() const noexcept { return layout_; }
    [[nodiscard]] std::span<const FiniteVolumeDuct> ducts() const noexcept { return ducts_; }
    [[nodiscard]] const ExhaustGasNetworkConfig& config() const noexcept { return config_; }
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

    /** A cylinder's unburned charge leaving at exhaust valve opening, as one
     * pocket with its own composition and temperature. A cell is far larger
     * than the pocket, so its mean mixture says nothing about it: the pocket
     * is carried with the local gas, warms towards it over the mixing time,
     * and lights from its own temperature or a burning cell. It burns fuel
     * and oxygen the cells actually hold, so it adds no mass. Returns false
     * when the cylinder has no port here, the pocket is not flammable or
     * every slot is taken. */
    bool emitUnburnedPocket(std::uint32_t cylinderId, double fuelKg,
                            double oxygenKg, double temperatureK,
                            const ExhaustFuelReactionConfig& reaction) noexcept;
    /** A fired cycle's exhaust lights every pocket waiting in its cylinder's
     * port element. Its burnt gas leaves at 2,000-2,200 K and sweeps the
     * primary at blowdown; a misfire's slug warms only by mixing otherwise,
     * and on an overrun never reached its ignition temperature whole
     * (journal 2026-10-25). */
    void igniteUnburnedPockets(std::uint32_t cylinderId) noexcept;
    [[nodiscard]] std::size_t activeUnburnedPocketCount() const noexcept;

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
     * It approximates the two-stage transfer using a local Heun prediction of
     * the terminal cell. Its complete current residual includes the adjoining
     * internal face, geometric pressure force and cached wall sources. It does
     * not advance the other cells or the reservoir; the exact integrated
     * transfer from `advance` replaces this ordering estimate afterwards.
     *
     * Returns nullopt for an unknown outlet, a non-positive duration, a state
     * the primitive recovery rejects, a junction endpoint, or a one-cell duct
     * whose opposite graph boundary cannot be inferred locally.
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

    /** Open area of a restriction face (CompiledExhaustInterface::restriction),
     *  discharge coefficient included, clamped to the smaller of its two
     *  faces. Applies from the next advance. Returns false for an interface
     *  that is not a restriction or an area that is not finite. */
    [[nodiscard]] bool setRestrictionOpeningAreaM2(std::size_t interfaceIndex,
                                                   double areaM2) noexcept;
    [[nodiscard]] double restrictionOpeningAreaM2(std::size_t interfaceIndex) const noexcept {
        return interfaceIndex < interfaceOpeningAreasM2_.size()
            ? interfaceOpeningAreasM2_[interfaceIndex] : 0.0;
    }
    /** Makes a restriction a turbine. Of the gas crossing it forward (upstream
     *  to downstream), `wheelShare` expands through a wheel of total-to-static
     *  isentropic efficiency `efficiency` down to the receiving side's static
     *  pressure; the rest bypasses it (a wastegate) without work. The work
     *  leaves the gas on the receiving side of the face. Zero share or
     *  efficiency: a plain restriction. Applies from the next advance. */
    [[nodiscard]] bool setRestrictionTurbine(std::size_t interfaceIndex,
                                             double wheelShare,
                                             double efficiency) noexcept;
    /** Over the last advance: the shaft work a restriction's turbine took, J,
     *  and the mass that crossed the face forward minus backward, kg. */
    [[nodiscard]] double restrictionWorkJ(std::size_t interfaceIndex) const noexcept {
        return interfaceIndex < interfaceTransfers_.size()
            ? interfaceTransfers_[interfaceIndex].workJ : 0.0;
    }
    [[nodiscard]] double restrictionMassKg(std::size_t interfaceIndex) const noexcept {
        return interfaceIndex < interfaceTransfers_.size()
            ? interfaceTransfers_[interfaceIndex].massKg : 0.0;
    }

    /** Runs `body(context, i)` once for every i in [0, count), possibly on
     *  several threads, and returns once all of them have completed. */
    struct ParallelFor final {
        using Body = void (*)(void*, std::size_t) noexcept;
        void (*run)(void* executor, std::size_t count, Body body,
                    void* context) noexcept { nullptr };
        void* executor { nullptr };
        /** Threads that take part in a dispatch, the caller's included. */
        std::size_t threads { 1 };
    };

    /** Spreads the per-duct part of every RK stage over `parallelFor`: the
     * stage update, the primitive and source cache, the wall heat exchange and
     * the interior residual, plus the face flux of any port or outlet at the
     * duct's ends. Each duct writes only its own buffers; applying those
     * fluxes and the coupling between ducts (interfaces, junctions, the shared
     * reservoir) stays serial in a fixed order, so the result is bit-identical
     * to the serial path whatever the schedule. The executor must outlive the
     * network, or be cleared with a default ParallelFor. */
    void setParallelFor(ParallelFor parallelFor) noexcept {
        parallelFor_ = parallelFor;
        ductChunkBegin_.clear();
    }

    /** Add species mass as a source in the duct cell adjacent to a cylinder
     * port, together with its sensible internal energy at temperatureK and an
     * optional additional heat term (negative for evaporation charge cooling).
     *
     * This is the network-side half of a port fuel injector or secondary-air
     * source: the mass appears with zero axial momentum, exactly like a wall
     * film evaporating into the stream. The candidate state is committed only
     * if it stays physically admissible, so a pathological command degrades to
     * a refused injection instead of a poisoned solve. Ports attached to a
     * junction are not supported (a junction's primitive cache is not owned
     * by the duct): injectSpeciesAtDuctInlet feeds the duct beyond it.
     */
    [[nodiscard]] bool injectSpeciesAtPort(std::size_t portIndex,
                                           GasSpecies species,
                                           double massKg,
                                           double temperatureK,
                                           double additionalHeatJ) noexcept;
    /** The same source in the cells at a duct's inlet: a rotary's injector,
     *  in its rotor's runner beyond the port passage the faces share. */
    [[nodiscard]] bool injectSpeciesAtDuctInlet(std::size_t ductIndex,
                                                GasSpecies species,
                                                double massKg,
                                                double temperatureK,
                                                double additionalHeatJ) noexcept;

private:
    [[nodiscard]] bool injectSpeciesAtDuctEnd(const ExhaustEndpoint& endpoint, GasSpecies species,
        double massKg, double temperatureK, double additionalHeatJ) noexcept;
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
    /** Which half of a stage evaluation to run: the per-duct pass (interior
     *  residuals and boundary face fluxes), the serial coupling that applies
     *  them, or both. */
    enum class StagePart : std::uint8_t { whole, ductPass, coupling };
    /** As above; `prelude(i)` runs on duct i just before its own pass, inside
     *  the same parallel dispatch, and a false return abandons the pass. */
    template <typename Prelude>
    [[nodiscard]] bool evaluateStage(
        bool useStageState,
        std::span<const CylinderValveBoundary> cylinderBoundaries,
        const ExhaustAmbientBoundary& ambient,
        StagePart part,
        const Prelude& prelude) noexcept;
    /** Junction, reservoir and shared-reservoir primitives of a stage; the
     *  ducts' own caches are prepared per duct inside advance(). */
    [[nodiscard]] bool prepareNonDuctStageStates(bool candidateStage) noexcept;
    /** `body(i)` for every duct, through parallelFor_ when one is set; true
     *  when every call returned true. */
    template <typename Body>
    [[nodiscard]] bool forEachDuct(const Body& body) noexcept;
    void updateOutletSamples(double durationSeconds) noexcept;

    /** A port's or outlet's face flux. Computed alongside its duct's interior
     *  residual (in parallel when an executor is set), then applied serially
     *  in port and outlet order, so the accumulation order never changes. */
    struct BoundaryFlux final {
        EulerFlux rawFlux {};
        /** The flux the duct's end face receives (unused at a junction). */
        EulerFlux ductFlux {};
        double openingAreaM2 { 0.0 };
        /** The area `rawFlux` is integrated over. */
        double flowAreaM2 { 0.0 };
        /** A port's valve-face warm start after this solve. */
        std::array<double, 2> faceSolve {};
    };

    ParallelFor parallelFor_ {};
    /** First duct of each contiguous chunk a dispatch hands out, then the
     *  duct count; built on first use, cleared when the ducts or the executor
     *  change. */
    std::vector<std::size_t> ductChunkBegin_;
    void buildDuctChunks() noexcept;
    /** Half the trial step while the first stage of a Hancock substep is
     * evaluated, zero otherwise. */
    double hancockHalfStepSeconds_ { 0.0 };
    std::vector<std::uint8_t> ductSucceeded_;
    /** Set by a duct whose Euler stage is inadmissible in this attempt. */
    std::vector<std::uint8_t> ductStageRejected_;
    std::vector<double> ductWallHeatRejectedJ_;
    std::vector<BoundaryFlux> portFluxes_;
    std::vector<BoundaryFlux> outletFluxes_;
    /** Ports and outlets attached to each duct's ends, in index order. */
    std::vector<std::vector<std::size_t>> ductPorts_;
    std::vector<std::vector<std::size_t>> ductOutlets_;
    /** An open interface's Riemann flux and what each duct end receives. */
    struct InterfaceFlux {
        EulerFlux rawFlux {};
        EulerFlux upstreamDuctFlux {};
        EulerFlux downstreamDuctFlux {};
        double effectiveAreaM2 { 0.0 };
    };
    static constexpr std::size_t noInterfaceOwner = static_cast<std::size_t>(-1);
    std::vector<InterfaceFlux> interfaceFluxes_;
    /** The duct whose pass computes each open interface (noInterfaceOwner for
     * a restriction or a junction-to-junction one). */
    std::vector<std::size_t> interfaceOwnerDuct_;
    std::vector<std::vector<std::size_t>> ductInterfaces_;
    /** With reconstructAcrossInterfaces: per interface, whether it joins two
     * duct ends openly; per duct end (2 x duct + 0 inlet, + 1 outlet), the
     * neighbouring duct end it reconstructs through, if any. */
    std::vector<std::uint8_t> interfaceReconstructed_;
    std::vector<std::optional<ExhaustEndpoint>> ductEndNeighbour_;
    std::vector<ConservativeState> ductEndGhost_;

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
    /** Per interface: the full face for a plain one, the plate's open area
     *  for a restriction. */
    std::vector<double> interfaceOpeningAreasM2_;
    struct TurbineSetting final { double wheelShare { 0.0 }; double efficiency { 0.0 }; };
    std::vector<TurbineSetting> interfaceTurbines_;
    /** Rates for one RK stage (kg/s, W), and their integral over an advance
     *  (kg, J). Only restrictions carry them. */
    struct InterfaceTransfer final { double massKg { 0.0 }; double workJ { 0.0 }; };
    std::vector<InterfaceTransfer> interfaceFirstStageRate_;
    std::vector<InterfaceTransfer> interfaceSecondStageRate_;
    std::vector<InterfaceTransfer> interfaceTransfers_;
    std::vector<ConservativeState> cylinderReservoirStates_;
    std::vector<ConservativeState> cylinderReservoirStage_;
    std::vector<ConservativeState> cylinderReservoirCandidate_;
    std::vector<ConservativeState> cylinderReservoirResidual_;
    std::vector<ConservativeState> cylinderReservoirStageResidual_;
    std::vector<PrimitiveState> cylinderReservoirPrimitives_;
    std::vector<PrimitiveState> cylinderReservoirStagePrimitives_;
    std::vector<PrimitiveState> cylinderReservoirCandidatePrimitives_;
    std::vector<double> cylinderReservoirVolumesM3_;
    /** Volumes at the start of the advance; the current ones follow the
     * supplied volume rate (CylinderValveBoundary::cylinderVolumeRateM3PerS). */
    std::vector<double> cylinderReservoirInitialVolumesM3_;
    /** Time into the current advance at which the stage being evaluated sits. */
    double boundaryElapsedSeconds_ { 0.0 };
    std::vector<std::uint8_t> cylinderReservoirActive_;
    // A shut supplied valve has no RK source; unchanged stage states reuse the
    // already validated primitive while retaining the normal state arithmetic.
    std::vector<std::uint8_t> cylinderReservoirClosed_;
    /** Port-order lookup into the caller's boundary span, rebuilt once/advance. */
    std::vector<std::size_t> cylinderBoundaryIndices_;
    std::vector<std::uint8_t> ductInletAssigned_;
    std::vector<std::uint8_t> ductOutletAssigned_;
    std::vector<ConservedFlowRate> cylinderFirstStageFlow_;
    std::vector<ConservedFlowRate> cylinderSecondStageFlow_;
    std::vector<ConservedFlowRate> outletFirstStageFlow_;
    std::vector<ConservedFlowRate> outletSecondStageFlow_;
    std::vector<CylinderGasExchange> cylinderExchanges_;
    /** Each valve's last characteristic face pressure and residual slope:
     * the next solve's starting point (see characteristicValveBoundary). */
    std::vector<std::array<double, 2>> valveFaceSolve_;
    std::vector<ExhaustOutletFlowSample> outletSamples_;
    struct ReactionSiteState final {
        /** Dimensionless Livengood-Wu integral; ignition occurs at one. */
        double inductionIntegral { 0.0 };
        bool burning { false };
        /** Origin of the current flame kernel, latched at ignition rather than
         * re-inferred after the released heat has warmed the gas. */
        bool wallIgnited { false };
    };
    std::vector<std::vector<ReactionSiteState>> ductReactionStates_;
    std::vector<ReactionSiteState> junctionReactionStates_;
    struct UnburnedPocket final {
        bool active { false };
        bool burning { false };
        bool inJunction { false };
        std::size_t elementIndex { 0 };
        /** Metres from the duct inlet; unused in a junction. */
        double positionM { 0.0 };
        double fuelKg { 0.0 };
        double oxygenKg { 0.0 };
        double temperatureK { 0.0 };
        double ageSeconds { 0.0 };
        double inductionIntegral { 0.0 };
    };
    static constexpr std::size_t maximumUnburnedPockets { 32 };
    std::array<UnburnedPocket, maximumUnburnedPockets> unburnedPockets_ {};
    [[nodiscard]] static std::size_t pocketCellIndex(
        const UnburnedPocket& pocket, double ductLengthM,
        std::size_t cellCount) noexcept;
    void carryPocket(UnburnedPocket& pocket, double durationSeconds) noexcept;
    // One optional well-mixed external reservoir, shared by all outlets. Its
    // scratch inventory is excluded from inventory(); the caller books the
    // returned outlet integrals once into its own reservoir after advance.
    ConservativeState ambientReservoirState_ {};
    ConservativeState ambientReservoirStage_ {};
    ConservativeState ambientReservoirCandidate_ {};
    ConservativeState ambientReservoirResidual_ {};
    ConservativeState ambientReservoirStageResidual_ {};
    double ambientReservoirVolumeM3_ { 0.0 };
    PrimitiveState ambientPrimitive_ {};
    PrimitiveState ambientStagePrimitive_ {};
    PrimitiveState ambientCandidatePrimitive_ {};
    bool configured_ { false };
};

} // namespace enginelab::gasdynamics
