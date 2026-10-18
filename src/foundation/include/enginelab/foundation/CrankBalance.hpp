#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

#include <cstddef>
#include <cstdint>
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
 * axial centroid.
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
    /** Cylinder ids in firing order, starting with 1. */
    std::vector<std::uint32_t> order;
    CrankBalance balance;
    /** Successive firings of neighbouring cylinders (feeds and bearings
     *  are shared there); fewer is better at equal balance. */
    std::uint32_t adjacentFirings { 0 };
};

/**
 * Even-firing inline cranks for `cylinderCount` cylinders, best balanced first:
 * every firing order starting with cylinder 1 places the pins (firing k at
 * k x cycle / N), ranked by primary + secondary couple, then by adjacent
 * firings. A crank and its mirror both appear. Empty above 9 cylinders, where
 * the enumeration is too long (8! orders at 9).
 */
[[nodiscard]] std::vector<FiringOrderCandidate> inlineFiringOrders(
    std::uint32_t cylinderCount, EngineCycle cycle, double rodRatio, std::size_t maxCandidates);

} // namespace enginelab
