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

/** One component resolved as a finite-volume duct. */
struct CompiledExhaustDuct final {
    std::uint32_t nodeId { 0 };
    std::uint32_t sourceComponentId { 0 };
    std::uint32_t pathIndex { 0 };
    ExhaustNodeType sourceType { ExhaustNodeType::pipe };
    double lengthM { 0.0 };
    /** Cell volume divided by length; may exceed throat area for a chamber. */
    double flowAreaM2 { 0.0 };
    /** Area presented to adjacent elements at both component ports. */
    double connectionAreaM2 { 0.0 };
    double hydraulicDiameterM { 0.0 };
    double volumeM3 { 0.0 };
    double lossCoefficient { 0.0 };
    double dischargeCoefficient { 1.0 };
    std::size_t cellCount { 0 };
    bool lengthWasDerived { false };
    bool areaWasDerivedFromVolume { false };
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
    bool volumeWasDerived { false };
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

