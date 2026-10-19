#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace enginelab {

/** One cylinder as the balance analysis sees it. */
struct BalanceCylinder final {
    /** Crank angle of this cylinder's top dead centre, degrees (its
     *  crank_offset_deg: the pin meets the bore axis there). */
    double tdcDegrees { 0.0 };
    /** Bore axis, degrees from vertical in the plane normal to the crank. */
    double bankDegrees { 0.0 };
    /** Position along the crank, in cylinder pitches. */
    double axialPitches { 0.0 };
    /** Reciprocating mass x crank radius, relative to the other cylinders'. */
    double massRadius { 1.0 };
    /** Crank radius over rod length: the second order's weight. */
    double rodRatio { 0.3 };
};

/**
 * Free inertia forces and couples of the reciprocating masses, without
 * counterweights (a rotating primary can be cancelled by them, an oscillating
 * one cannot). Amplitudes over a turn: forces in units of one cylinder's
 * m r w^2, couples in m r w^2 x cylinder pitch, measured about the cylinders'
 * axial centroid. Each order is a sum of sinusoids of one frequency, so its
 * peak is exact (the largest singular value of its two phasor vectors).
 */
struct CrankBalance final {
    double primaryForce { 0.0 };
    double secondaryForce { 0.0 };
    double primaryCouple { 0.0 };
    double secondaryCouple { 0.0 };
    /** Crank degrees between successive firings, in firing order. */
    std::vector<double> firingIntervalsDegrees;
};

/**
 * The cylinders of an engine as the analysis sees them. No axial position is
 * authored, so it is assumed: an inline engine in cylinder id order, one
 * pitch apart; other layouts one pitch per row, a row holding one cylinder of
 * each bank in id order. Reciprocating mass = piston + one third of the rod.
 */
[[nodiscard]] std::vector<BalanceCylinder> balanceCylinders(const EngineConfig& config);

[[nodiscard]] CrankBalance analyseCrankBalance(
    const std::vector<BalanceCylinder>& cylinders, double cycleDegrees);

[[nodiscard]] CrankBalance analyseCrankBalance(const EngineConfig& config);

struct FiringOrderCandidate final {
    /** Cylinder ids (1-based index into the cylinders given) in firing order,
     *  starting with 1. */
    std::vector<std::uint32_t> order;
    CrankBalance balance;
    /** Successive firings of axial neighbours on one bank (feeds and bearings
     *  are shared there); fewer is better at equal balance. */
    std::uint32_t adjacentFirings { 0 };
};

/**
 * Even-firing orders for cylinders whose bank and axial position are given
 * (their tdcDegrees are ignored): firing k comes k x cycle / N after the first.
 * Ranked by twice the free forces plus the couples, then by adjacent firings.
 * Every order is tried up to 8 cylinders; above, a deterministic local search
 * (pairwise swaps from several regular starting orders) keeps the best it
 * finds, which is not proven optimal. A crank and its mirror both appear.
 */
[[nodiscard]] std::vector<FiringOrderCandidate> evenFiringOrders(
    const std::vector<BalanceCylinder>& cylinders, double cycleDegrees, std::size_t maxCandidates);

/** A crank whose cylinders of one row share their pins. */
struct RowPinCrank final {
    /** Top dead centre of each cylinder (the order given), cylinder 1 at 0. */
    std::vector<double> tdcDegrees;
    /** Cylinder ids in firing order, starting with 1. */
    std::vector<std::uint32_t> order;
    CrankBalance balance;
    std::uint32_t adjacentFirings { 0 };
    /** Longest firing interval less the shortest: zero fires evenly. */
    double unevennessDegrees { 0.0 };
    /** How far each bank's pin leads the first bank's in a row (banks ranked
     *  by angle): zero when the row shares one pin. */
    double splitDegrees { 0.0 };
};

/**
 * The best crank whose cylinders of one row (equal axialPitches) share one pin
 * (`splitDegrees` zero), or whose pins are split, each bank's leading the
 * first bank's by that much times its rank. Rows' pins lie on a 360 / rows
 * grid, every set tried (none above 200,000 sets: empty). Ranked by
 * unevenness, then as evenFiringOrders(). `firingOrder`, when given, is the
 * only order kept. With `evenOnly`, or above 10 cylinders on a four-stroke,
 * only even-firing cranks are sought. Empty when there is none.
 */
[[nodiscard]] std::optional<RowPinCrank> rowPinCrank(const std::vector<BalanceCylinder>& cylinders,
    double cycleDegrees, double splitDegrees, const std::vector<std::uint32_t>& firingOrder, bool evenOnly);

/**
 * The even-firing crank with split pins (rowPinCrank()) whose split is the
 * least, shared pins excluded. Splits tried: those that bring the banks onto
 * the even-firing angles. Empty when there is none.
 */
[[nodiscard]] std::optional<RowPinCrank> splitPinCrank(const std::vector<BalanceCylinder>& cylinders,
    double cycleDegrees, const std::vector<std::uint32_t>& firingOrder);

/** evenFiringOrders() for `cylinderCount` inline cylinders one pitch apart. */
[[nodiscard]] std::vector<FiringOrderCandidate> inlineFiringOrders(
    std::uint32_t cylinderCount, EngineCycle cycle, double rodRatio, std::size_t maxCandidates);

} // namespace enginelab
