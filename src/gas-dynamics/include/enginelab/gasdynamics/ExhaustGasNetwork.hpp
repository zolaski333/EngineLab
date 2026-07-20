#pragma once

#include <enginelab/gasdynamics/ExhaustNetworkLayout.hpp>
#include <enginelab/gasdynamics/FiniteVolumeDuct.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
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
    double maximumCourantNumber { 0.42 };
    std::size_t maximumSubstepsPerAdvance { 100'000 };

    [[nodiscard]] bool valid() const noexcept;
};

/** Frozen cylinder state and valve conductance for one requested advance. */
struct CylinderValveBoundary final {
    std::uint32_t cylinderId { 0 };
    ConservativeState cylinderState {};
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

    [[nodiscard]] double totalMassKg() const noexcept;
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
};

struct ExhaustNetworkAdvanceResult final {
    double advancedTimeSeconds { 0.0 };
    std::size_t acceptedSubsteps { 0 };
    std::size_t rejectedSubsteps { 0 };
    bool completed { true };
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

    [[nodiscard]] bool configured() const noexcept { return configured_; }
    [[nodiscard]] const EulerMixtureModel& mixtureModel() const noexcept { return mixtureModel_; }
    [[nodiscard]] const ExhaustNetworkLayout& layout() const noexcept { return layout_; }
    [[nodiscard]] std::span<const FiniteVolumeDuct> ducts() const noexcept { return ducts_; }
    [[nodiscard]] std::span<FiniteVolumeDuct> ducts() noexcept { return ducts_; }
    [[nodiscard]] std::span<const ConservativeState> junctionStates() const noexcept {
        return junctionStates_;
    }
    [[nodiscard]] ExhaustNetworkInventory inventory() const noexcept;

    /** Advance the complete network by exactly durationSeconds when successful.
     * Missing cylinder IDs are treated as closed valves. Duplicate supplied IDs
     * are rejected because iteration order must never select the physics.
     */
    [[nodiscard]] ExhaustNetworkAdvanceResult advance(
        double durationSeconds,
        std::span<const CylinderValveBoundary> cylinderBoundaries,
        const ExhaustAmbientBoundary& ambient) noexcept;

    [[nodiscard]] std::span<const CylinderGasExchange> cylinderExchanges() const noexcept {
        return cylinderExchanges_;
    }
    [[nodiscard]] std::span<const ExhaustOutletFlowSample> outletSamples() const noexcept {
        return outletSamples_;
    }

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
    [[nodiscard]] bool allStageStatesPhysical(bool candidateStage) const noexcept;
    void updateOutletSamples(double durationSeconds) noexcept;

    EulerMixtureModel mixtureModel_;
    ExhaustNetworkLayout layout_;
    ExhaustGasNetworkConfig config_ {};
    std::vector<FiniteVolumeDuct> ducts_;
    std::vector<ConservativeState> junctionStates_;
    std::vector<ConservativeState> junctionStage_;
    std::vector<ConservativeState> junctionCandidate_;
    std::vector<ConservativeState> junctionResidual_;
    std::vector<ConservativeState> junctionStageResidual_;
    std::vector<double> junctionPortAreaSums_;
    std::vector<std::uint8_t> ductInletAssigned_;
    std::vector<std::uint8_t> ductOutletAssigned_;
    std::vector<ConservedFlowRate> cylinderFirstStageFlow_;
    std::vector<ConservedFlowRate> cylinderSecondStageFlow_;
    std::vector<ConservedFlowRate> outletFirstStageFlow_;
    std::vector<ConservedFlowRate> outletSecondStageFlow_;
    std::vector<CylinderGasExchange> cylinderExchanges_;
    std::vector<ExhaustOutletFlowSample> outletSamples_;
    bool configured_ { false };
};

} // namespace enginelab::gasdynamics
