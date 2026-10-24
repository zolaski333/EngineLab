#pragma once

#include <enginelab/gasdynamics/ExhaustGasNetwork.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace enginelab::gasdynamics {

/** An exhaust network advanced as its unconnected parts.
 *
 * Elements that no interface, junction or shared cylinder joins exchange
 * nothing during an advance: a two-stroke's pipes (one expansion chamber and
 * tailpipe per cylinder, each to its own outlet) are such parts. Partitioned,
 * each part is its own ExhaustGasNetwork with its own CFL step, and one
 * dispatch advances them all on the executor's threads, where the whole
 * network takes one step for all of them and spreads each stage over its
 * ducts, a barrier per stage.
 *
 * Global indices (ducts, junctions, interfaces, cylinder ports, outlets) are
 * the compiled layout's; each part keeps its elements in that order. A
 * network of one part, or one configured unpartitioned, forwards every call to
 * a single ExhaustGasNetwork and is bit-identical to it.
 *
 * The parts share no reservoir: an advance with a finite ambient reservoir
 * (ExhaustAmbientBoundary::reservoirVolumeM3 > 0) is refused unless there is
 * one part. Port injection is not offered.
 */
class PartitionedExhaustNetwork final {
public:
    explicit PartitionedExhaustNetwork(
        ThermodynamicModel model = ThermodynamicModel::standardCombustionGas()) noexcept;

    /** Build the parts' meshes and work arrays. Not realtime-safe. With
     * `partition` false the layout stays one network whatever its parts. */
    [[nodiscard]] bool configure(const ExhaustNetworkLayout& layout,
                                 ExhaustGasNetworkConfig config,
                                 bool partition);

    [[nodiscard]] bool configured() const noexcept { return !parts_.empty(); }
    [[nodiscard]] std::size_t partCount() const noexcept { return parts_.size(); }
    [[nodiscard]] const ExhaustGasNetwork& part(std::size_t index) const noexcept {
        return *parts_[index];
    }
    [[nodiscard]] const ExhaustNetworkLayout& layout() const noexcept { return layout_; }
    [[nodiscard]] const EulerMixtureModel& mixtureModel() const noexcept {
        return parts_.front()->mixtureModel();
    }
    [[nodiscard]] std::size_t ductCount() const noexcept { return ductAt_.size(); }
    [[nodiscard]] const FiniteVolumeDuct& duct(std::size_t index) const noexcept {
        return parts_[ductAt_[index].part]->ducts()[ductAt_[index].local];
    }
    [[nodiscard]] const ConservativeState& junctionState(std::size_t index) const noexcept {
        return parts_[junctionAt_[index].part]->junctionStates()[junctionAt_[index].local];
    }

    /** One part: handed to it, every RK stage spread over its ducts. Several:
     * each advance hands the parts out, one per claim, and each runs serially
     * on the thread that claimed it. */
    void setParallelFor(ExhaustGasNetwork::ParallelFor parallelFor) noexcept;

    [[nodiscard]] bool reset(double pressurePa, double temperatureK,
                             GasComposition composition = GasComposition::dryAir()) noexcept;
    /** Part by part (ExhaustGasNetwork::adoptStateFrom); false, nothing
     * changed, unless both were partitioned into the same parts. */
    [[nodiscard]] bool adoptStateFrom(const PartitionedExhaustNetwork& previous) noexcept;
    [[nodiscard]] double peakWallTemperatureK() const noexcept;
    [[nodiscard]] ExhaustNetworkInventory inventory() const noexcept;
    /** Each part's reaction in turn; the sources follow in part order, and
     * those past ExhaustFuelReactionResult::maximumSources are counted dropped. */
    [[nodiscard]] ExhaustFuelReactionResult reactUnburnedFuel(
        double durationSeconds, const ExhaustFuelReactionConfig& reaction) noexcept;

    /** Every part by exactly durationSeconds. Completed only if each part
     * completed; substep counts are the busiest part's, the time steps the
     * extremes over the parts, the rejected steps and wall heat summed. */
    [[nodiscard]] ExhaustNetworkAdvanceResult advance(
        double durationSeconds,
        std::span<const CylinderValveBoundary> cylinderBoundaries,
        const ExhaustAmbientBoundary& ambient) noexcept;
    [[nodiscard]] bool sampleCylinderBoundaries(
        std::span<const CylinderValveBoundary> cylinderBoundaries,
        std::span<CylinderBoundaryFlowSample> samples) const noexcept;

    /** In compiled cylinder-port and outlet order, as of the last advance. */
    [[nodiscard]] std::span<const CylinderGasExchange> cylinderExchanges() const noexcept;
    [[nodiscard]] std::span<const ExhaustOutletFlowSample> outletSamples() const noexcept;

    [[nodiscard]] bool setRestrictionOpeningAreaM2(std::size_t interfaceIndex,
                                                   double areaM2) noexcept;
    [[nodiscard]] bool setRestrictionTurbine(std::size_t interfaceIndex,
                                             double wheelShare,
                                             double efficiency) noexcept;
    [[nodiscard]] double restrictionWorkJ(std::size_t interfaceIndex) const noexcept;

private:
    struct Place final {
        std::uint32_t part { 0 };
        std::uint32_t local { 0 };
    };
    struct PartJob final {
        double durationSeconds { 0.0 };
        const ExhaustAmbientBoundary* ambient { nullptr };
        ExhaustNetworkAdvanceResult result {};
    };

    void gatherTransfers() noexcept;

    ThermodynamicModel model_;
    ExhaustNetworkLayout layout_;
    std::vector<std::unique_ptr<ExhaustGasNetwork>> parts_;
    std::vector<Place> ductAt_;
    std::vector<Place> junctionAt_;
    std::vector<Place> interfaceAt_;
    std::vector<Place> portAt_;
    std::vector<Place> outletAt_;
    ExhaustGasNetwork::ParallelFor parallelFor_ {};
    // Per-advance work, sized by configure() so that nothing realtime
    // allocates. Part p's boundaries and samples occupy
    // [partBoundaryBegin_[p], partBoundaryBegin_[p + 1]), its port count.
    std::vector<std::size_t> partBoundaryBegin_;
    std::vector<std::size_t> partBoundaryCount_;
    std::vector<CylinderValveBoundary> boundaryScratch_;
    mutable std::vector<std::size_t> sampleBoundaryCount_;
    mutable std::vector<CylinderValveBoundary> sampleBoundaryScratch_;
    mutable std::vector<CylinderBoundaryFlowSample> partSamples_;
    std::vector<PartJob> jobs_;
    std::vector<CylinderGasExchange> exchanges_;
    std::vector<ExhaustOutletFlowSample> outletSamples_;
};

} // namespace enginelab::gasdynamics
