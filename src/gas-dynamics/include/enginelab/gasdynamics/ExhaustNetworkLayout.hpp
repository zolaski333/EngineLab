#pragma once

#include <enginelab/exhaust/ExhaustGraph.hpp>

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace enginelab::gasdynamics {

/** Discretisation policy applied once while compiling an exhaust graph. */
struct ExhaustNetworkDiscretisation final {
    double targetCellLengthM { 0.020 };
    double minimumResolvedLengthM { 0.005 };
    std::size_t minimumCellsPerDuct { 2 };
    std::size_t maximumCellsPerDuct { 160 };
    std::size_t maximumTotalCells { 4'096 };

    [[nodiscard]] bool valid() const noexcept;
};

/** Production policy for the nonlinear, realtime exhaust-feedback mesh.
 *
 * Keep the simulator, editor advisory and measurement harnesses on this one
 * source of truth. The characteristic audio network owns audio-band wave
 * propagation; this deliberately coarse finite-volume mesh owns mean flow,
 * back-pressure and low-band nonlinear feedback. */
[[nodiscard]] inline ExhaustNetworkDiscretisation
realtimeExhaustFeedbackDiscretisation() noexcept {
    ExhaustNetworkDiscretisation result;
    result.targetCellLengthM = 0.360;
    result.minimumCellsPerDuct = 1;
    result.maximumCellsPerDuct = 64;
    result.maximumTotalCells = 1'024;
    return result;
}

/** One component resolved as a finite-volume duct. */
struct CompiledExhaustDuct final {
    std::uint32_t nodeId { 0 };
    std::uint32_t sourceComponentId { 0 };
    std::uint32_t pathIndex { 0 };
    ExhaustNodeType sourceType { ExhaustNodeType::pipe };
    double lengthM { 0.0 };
    /** Cell volume divided by length; may exceed throat area for a chamber. */
    double flowAreaM2 { 0.0 };
    /** Length-mean area retained for compatibility and volume accounting. */
    /** Areas used by the first and last quasi-1D faces. */
    double inletFlowAreaM2 { 0.0 };
    double outletFlowAreaM2 { 0.0 };
    /** Mean area presented to adjacent elements, retained for old consumers. */
    double connectionAreaM2 { 0.0 };
    /** Physical connection apertures at each component port. */
    double inletConnectionAreaM2 { 0.0 };
    double outletConnectionAreaM2 { 0.0 };
    double hydraulicDiameterM { 0.0 };
    double volumeM3 { 0.0 };
    double lossCoefficient { 0.0 };
    double dischargeCoefficient { 1.0 };
    std::size_t cellCount { 0 };
    bool lengthWasDerived { false };
    bool areaWasDerivedFromVolume { false };
    /** Acoustic-only porous lining metadata; ignored by the gas solver. */
    double packingFlowResistivityPaSPerM2 { 0.0 };
    double packingThicknessM { 0.0 };
    double perforatedOpenAreaRatio { 0.0 };
    /** True when one equivalent duct represents an authored cellular
     * catalyst substrate. It never changes cell or delay-line count. */
    bool homogenisedCatalystMonolith { false };
    double catalystOpenAreaRatio { 1.0 };
    double catalystSubstrateVolumetricHeatCapacityJPerM3K { 0.0 };
    /** A packed muffler carries mean flow through its perforated centre tube.
     *  The outer can is acoustic storage, not extra quasi-1D flow area. */
    bool perforatedCoreMuffler { false };
    /** Gross authored can volume minus the swept core volume. Audio only. */
    double mufflerAnnularVolumeM3 { 0.0 };
};

/** Well-mixed physical junction used only where topology changes direction or
 * branch count. A two-port duct-to-duct edge remains a direct 1D interface.
 */
struct CompiledExhaustJunction final {
    std::uint32_t nodeId { 0 };
    std::uint32_t sourceComponentId { 0 };
    std::uint32_t pathIndex { 0 };
    ExhaustNodeType sourceType { ExhaustNodeType::merge };
    double volumeM3 { 0.0 };
    double characteristicDiameterM { 0.0 };
    double lossCoefficient { 0.0 };
    /** Authored length of the trunk this junction carries, metres; zero when the
     *  author drew a branch with no extent.
     *
     *  A merge is not only a scattering point. A 4-into-1 collector, a Y-piece
     *  or a tailpipe split all have a common trunk of real length on their
     *  single-port side, and the graph schema lets the author state it. The
     *  finite-volume solver models a junction as a well-mixed plenum and folds
     *  that extent into volumeM3, which is the right lumped choice at the low
     *  frequencies it resolves. A waveguide cannot: length is delay there, and
     *  dropping it deletes the collector from the acoustic model entirely.
     *  Published here so the two discretisations read the same geometry. */
    double trunkLengthM { 0.0 };
    bool volumeWasDerived { false };
    /** Passive power-wave coupling amplitude when sourceType==crossover. */
    double crossoverCoupling { 0.0 };
};

enum class ExhaustEndpointType : std::uint8_t {
    ductInlet,
    ductOutlet,
    junction,
};

/** Stable index reference into ducts() or junctions(). */
struct ExhaustEndpoint final {
    ExhaustEndpointType type { ExhaustEndpointType::ductInlet };
    std::size_t elementIndex { 0 };
    std::uint32_t nodeId { 0 };
};

/** Oriented component-to-component interface. */
struct CompiledExhaustInterface final {
    ExhaustEndpoint upstream {};
    ExhaustEndpoint downstream {};
    std::uint8_t upstreamPort { unspecifiedExhaustComponentPort };
    std::uint8_t downstreamPort { unspecifiedExhaustComponentPort };
};

struct CompiledCylinderPort final {
    std::uint32_t cylinderId { 0 };
    std::uint32_t pathIndex { 0 };
    ExhaustEndpoint networkEndpoint {};
    double runnerConnectionAreaM2 { 0.0 };
    double dischargeCoefficient { 1.0 };
};

struct CompiledExhaustOutlet final {
    std::uint32_t outletNodeId { 0 };
    std::uint32_t pathIndex { 0 };
    ExhaustEndpoint networkEndpoint {};
    double openingAreaM2 { 0.0 };
    double dischargeCoefficient { 1.0 };
    AcousticPoint3M acousticPositionM {};
    AcousticPoint3M acousticAxis { 0.0, 1.0, 0.0 };
    AcousticTerminationType acousticTermination {
        AcousticTerminationType::unflanged };
};

enum class ExhaustNetworkLayoutIssue : std::uint8_t {
    invalidDiscretisation,
    missingGraphNode,
    unresolvedEndpoint,
    zeroLengthComponentResolved,
    cellBudgetExceeded,
    missingCylinderRoute,
    missingOutlet,
};

struct ExhaustNetworkLayoutDiagnostic final {
    ExhaustNetworkLayoutIssue issue { ExhaustNetworkLayoutIssue::unresolvedEndpoint };
    std::uint32_t relatedNodeId { 0 };
};

/** Immutable, allocation-ready physical layout compiled from ExhaustGraph.
 *
 * This class deliberately ignores audioGain and resonanceHz. Those are outputs
 * of the legacy acoustic reduction and have no place in gas conservation.
 */
class ExhaustNetworkLayout final {
public:
    [[nodiscard]] static ExhaustNetworkLayout compile(
        const ExhaustGraph& graph,
        ExhaustNetworkDiscretisation discretisation = {});

    /** Assemble a layout from explicit elements, without an authored graph.
     *
     * This exists for networks whose topology is owned by the simulator rather
     * than by an exhaust author — the first user is the 1-D intake runner
     * network, which is one duct per cylinder with the plenum as the ambient
     * reservoir. The solver itself is duct-direction agnostic, but the
     * boundary flux conventions are not: `compressibleValveFlux` treats the
     * cylinder as the left state and the open-end characteristic treats the
     * interior as the left state, and `evaluateStage` applies both without an
     * endpoint-type sign inversion. Compiled exhaust layouts therefore only
     * ever attach cylinder ports at duct INLETS and outlets at duct OUTLETS,
     * and this factory enforces the same orientation (junctions are
     * orientation-free and accepted for either). Interfaces must run
     * ductOutlet/junction -> ductInlet/junction for the same reason.
     *
     * Every duct face must be covered exactly once by an interface, port or
     * outlet: `evaluateStage` refuses to advance a network with an uncovered
     * face, so a layout that under- or over-covers is rejected here with an
     * `unresolvedEndpoint` diagnostic instead of failing every advance later.
     */
    [[nodiscard]] static ExhaustNetworkLayout assemble(
        std::vector<CompiledExhaustDuct> ducts,
        std::vector<CompiledExhaustJunction> junctions,
        std::vector<CompiledExhaustInterface> interfaces,
        std::vector<CompiledCylinderPort> cylinderPorts,
        std::vector<CompiledExhaustOutlet> outlets,
        ExhaustNetworkDiscretisation discretisation = {});

    [[nodiscard]] bool valid() const noexcept { return valid_; }
    [[nodiscard]] std::span<const CompiledExhaustDuct> ducts() const noexcept { return ducts_; }
    [[nodiscard]] std::span<const CompiledExhaustJunction> junctions() const noexcept {
        return junctions_;
    }
    [[nodiscard]] std::span<const CompiledExhaustInterface> interfaces() const noexcept {
        return interfaces_;
    }
    [[nodiscard]] std::span<const CompiledCylinderPort> cylinderPorts() const noexcept {
        return cylinderPorts_;
    }
    [[nodiscard]] std::span<const CompiledExhaustOutlet> outlets() const noexcept {
        return outlets_;
    }
    [[nodiscard]] std::span<const ExhaustNetworkLayoutDiagnostic> diagnostics() const noexcept {
        return diagnostics_;
    }
    [[nodiscard]] std::size_t totalCellCount() const noexcept { return totalCellCount_; }
    /** Shortest DUCT cell anywhere in the network, metres. Zero if none.
     *
     * This remains useful for mesh inspection, but it is not the complete CFL
     * scale: a finite junction may impose a smaller V/sum(A_port) bound. Use
     * `minimumCflLengthM()` for editor warnings and cost comparisons.
     *
     * It is deliberately separate from `totalCellCount()`, which is what the
     * `cellBudgetExceeded` diagnostic guards. The two failures are different
     * and the coarsening loop in `compile` only answers the first: it raises
     * the target cell length while the TOTAL exceeds the budget, and never
     * reacts to one duct being far shorter than the target. A user who
     * authored an 80 x 10 mm silencer body hit exactly that -- a handful of
     * cells, one of them 10 mm, and the network's substep rate went from
     * 11,520 Hz to 92,160 Hz, which put the engine into permanent slow motion
     * and was reported as the engine having "gained inertia and lost its
     * liveliness". Nothing in validation could see it. */
    [[nodiscard]] double minimumCellLengthM() const noexcept;
    /** Smallest geometry-only length entering the production CFL bound.
     *
     * This includes both duct dx and each finite junction's V/sum(A_port),
     * matching ExhaustGasNetwork's static area accounting. It intentionally
     * excludes the live |u|+c signal speed: ratios between two layouts at the
     * same operating state are the intended editor/benchmark use. */
    [[nodiscard]] double minimumCflLengthM() const noexcept;
    [[nodiscard]] const ExhaustNetworkDiscretisation& discretisation() const noexcept {
        return discretisation_;
    }

private:
    ExhaustNetworkDiscretisation discretisation_ {};
    std::vector<CompiledExhaustDuct> ducts_;
    std::vector<CompiledExhaustJunction> junctions_;
    std::vector<CompiledExhaustInterface> interfaces_;
    std::vector<CompiledCylinderPort> cylinderPorts_;
    std::vector<CompiledExhaustOutlet> outlets_;
    std::vector<ExhaustNetworkLayoutDiagnostic> diagnostics_;
    std::size_t totalCellCount_ { 0 };
    bool valid_ { false };
};

} // namespace enginelab::gasdynamics
