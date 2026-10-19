#include <enginelab/foundation/CrankBalance.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <numeric>
#include <set>

namespace enginelab {
namespace {

constexpr double degreesToRadians = std::numbers::pi / 180.0;

[[nodiscard]] double wrapped(double degrees, double period) {
    const auto value = std::fmod(degrees, period);
    return value < 0.0 ? value + period : value;
}

/** Peak over a turn of |sum of weight x cos(order x (theta - tdc)) x axis|,
 *  with axis the bore direction, times the axial lever when `couple`.
 *  The sum is X cos(order theta) + Y sin(order theta) with X, Y vectors in the
 *  plane normal to the crank: its peak is the largest singular value of [X Y]. */
[[nodiscard]] double peakAmplitude(
    const std::vector<BalanceCylinder>& cylinders, double order, bool couple, double centroid) {
    double xx = 0.0;
    double xy = 0.0;
    double yx = 0.0;
    double yy = 0.0;
    for (const auto& cylinder : cylinders) {
        const auto weight = cylinder.massRadius * (order > 1.5 ? cylinder.rodRatio : 1.0)
            * (couple ? cylinder.axialPitches - centroid : 1.0);
        const auto phase = order * cylinder.tdcDegrees * degreesToRadians;
        const auto axisX = std::sin(cylinder.bankDegrees * degreesToRadians);
        const auto axisY = std::cos(cylinder.bankDegrees * degreesToRadians);
        xx += weight * std::cos(phase) * axisX;
        xy += weight * std::cos(phase) * axisY;
        yx += weight * std::sin(phase) * axisX;
        yy += weight * std::sin(phase) * axisY;
    }
    const auto a = xx * xx + xy * xy;
    const auto d = yx * yx + yy * yy;
    const auto b = xx * yx + xy * yy;
    const auto half = 0.5 * (a - d);
    return std::sqrt(std::max(0.0, 0.5 * (a + d) + std::sqrt(half * half + b * b)));
}

/** Firing positions -> top dead centres, every firing `interval` apart. */
void placeFirings(std::vector<BalanceCylinder>& cylinders, const std::vector<std::uint32_t>& order, double interval) {
    for (std::size_t position = 0; position < order.size(); ++position)
        cylinders[order[position] - 1].tdcDegrees = static_cast<double>(position) * interval;
}

[[nodiscard]] std::uint32_t adjacentFirings(
    const std::vector<BalanceCylinder>& cylinders, const std::vector<std::uint32_t>& order) {
    if (order.size() <= 2) return 0;
    std::uint32_t count = 0;
    for (std::size_t position = 0; position < order.size(); ++position) {
        const auto& current = cylinders[order[position] - 1];
        const auto& next = cylinders[order[(position + 1) % order.size()] - 1];
        if (std::abs(current.bankDegrees - next.bankDegrees) < 1.0e-9
                && std::abs(std::abs(current.axialPitches - next.axialPitches) - 1.0) < 1.0e-9)
            ++count;
    }
    return count;
}

[[nodiscard]] double score(const CrankBalance& balance) {
    return 2.0 * (balance.primaryForce + balance.secondaryForce) + balance.primaryCouple + balance.secondaryCouple;
}

/** Couples within 0.1 % of a cylinder's m r w^2 x pitch rank as equal. */
[[nodiscard]] bool better(const FiringOrderCandidate& a, const FiringOrderCandidate& b) {
    const auto scoreA = std::round(score(a.balance) * 1000.0);
    const auto scoreB = std::round(score(b.balance) * 1000.0);
    if (scoreA != scoreB) return scoreA < scoreB;
    if (a.adjacentFirings != b.adjacentFirings) return a.adjacentFirings < b.adjacentFirings;
    return a.order < b.order;
}

} // namespace

std::vector<BalanceCylinder> balanceCylinders(const EngineConfig& config) {
    auto ordered = config.cylinders;
    std::sort(ordered.begin(), ordered.end(), [](const auto& a, const auto& b) { return a.id < b.id; });
    std::set<double> banks;
    for (const auto& cylinder : ordered) banks.insert(cylinder.bankOffsetDegrees);
    const auto perRow = config.layout == EngineLayout::inlineLayout ? 1.0
        : static_cast<double>(std::max<std::size_t>(1, banks.size()));
    const auto reference = ordered.empty() ? 1.0
        : (ordered.front().pistonMassGrams + ordered.front().connectingRodMassGrams / 3.0)
            * ordered.front().strokeMm;
    std::vector<BalanceCylinder> result;
    result.reserve(ordered.size());
    for (std::size_t index = 0; index < ordered.size(); ++index) {
        const auto& cylinder = ordered[index];
        BalanceCylinder entry;
        entry.tdcDegrees = cylinder.crankOffsetDegrees;
        entry.bankDegrees = cylinder.bankOffsetDegrees;
        entry.axialPitches = std::floor(static_cast<double>(index) / perRow);
        entry.massRadius = (cylinder.pistonMassGrams + cylinder.connectingRodMassGrams / 3.0)
            * cylinder.strokeMm / std::max(1.0e-9, reference);
        entry.rodRatio = cylinder.strokeMm * 0.5 / std::max(1.0e-9, cylinder.connectingRodMm);
        result.push_back(entry);
    }
    return result;
}

CrankBalance analyseCrankBalance(const std::vector<BalanceCylinder>& cylinders, double cycleDegrees) {
    CrankBalance balance;
    if (cylinders.empty()) return balance;
    auto centroid = 0.0;
    for (const auto& cylinder : cylinders) centroid += cylinder.axialPitches;
    centroid /= static_cast<double>(cylinders.size());
    balance.primaryForce = peakAmplitude(cylinders, 1.0, false, centroid);
    balance.secondaryForce = peakAmplitude(cylinders, 2.0, false, centroid);
    balance.primaryCouple = peakAmplitude(cylinders, 1.0, true, centroid);
    balance.secondaryCouple = peakAmplitude(cylinders, 2.0, true, centroid);
    std::vector<double> phases;
    phases.reserve(cylinders.size());
    for (const auto& cylinder : cylinders) phases.push_back(wrapped(cylinder.tdcDegrees, cycleDegrees));
    std::sort(phases.begin(), phases.end());
    for (std::size_t index = 0; index < phases.size(); ++index) {
        const auto next = index + 1 < phases.size() ? phases[index + 1] : phases.front() + cycleDegrees;
        balance.firingIntervalsDegrees.push_back(next - phases[index]);
    }
    return balance;
}

CrankBalance analyseCrankBalance(const EngineConfig& config) {
    return analyseCrankBalance(balanceCylinders(config), cycleDegreesOf(config.cycle));
}

std::vector<FiringOrderCandidate> evenFiringOrders(
    const std::vector<BalanceCylinder>& cylinders, double cycleDegrees, std::size_t maxCandidates) {
    std::vector<FiringOrderCandidate> candidates;
    const auto count = static_cast<std::uint32_t>(cylinders.size());
    if (count == 0 || maxCandidates == 0) return candidates;
    const auto interval = cycleDegrees / static_cast<double>(count);
    auto working = cylinders;
    const auto evaluate = [&](const std::vector<std::uint32_t>& order) {
        FiringOrderCandidate candidate;
        candidate.order = order;
        placeFirings(working, order, interval);
        candidate.balance = analyseCrankBalance(working, cycleDegrees);
        candidate.adjacentFirings = adjacentFirings(working, order);
        return candidate;
    };

    if (count <= 8) {
        std::vector<std::uint32_t> order(count);
        std::iota(order.begin(), order.end(), 1U);
        do {
            candidates.push_back(evaluate(order));
        } while (std::next_permutation(order.begin() + 1, order.end()));
    } else {
        // Regular starting orders: every stride coprime with the count, both
        // ways round, then the best pairwise swap until none improves.
        std::set<std::vector<std::uint32_t>> seen;
        for (std::uint32_t stride = 1; stride < count; ++stride) {
            if (std::gcd(stride, count) != 1) continue;
            std::vector<std::uint32_t> order(count);
            for (std::uint32_t position = 0; position < count; ++position)
                order[position] = (position * stride) % count + 1;
            auto current = evaluate(order);
            for (;;) {
                auto best = current;
                for (std::uint32_t i = 1; i + 1 < count; ++i) {
                    for (std::uint32_t j = i + 1; j < count; ++j) {
                        auto swapped = current.order;
                        std::swap(swapped[i], swapped[j]);
                        auto candidate = evaluate(swapped);
                        if (better(candidate, best)) best = std::move(candidate);
                    }
                }
                if (best.order == current.order) break;
                current = std::move(best);
            }
            if (seen.insert(current.order).second) candidates.push_back(std::move(current));
        }
    }
    std::sort(candidates.begin(), candidates.end(), better);
    if (candidates.size() > maxCandidates) candidates.resize(maxCandidates);
    return candidates;
}

std::vector<FiringOrderCandidate> inlineFiringOrders(
    std::uint32_t cylinderCount, EngineCycle cycle, double rodRatio, std::size_t maxCandidates) {
    std::vector<BalanceCylinder> cylinders(cylinderCount);
    for (std::uint32_t index = 0; index < cylinderCount; ++index) {
        cylinders[index].axialPitches = static_cast<double>(index);
        cylinders[index].rodRatio = rodRatio;
    }
    return evenFiringOrders(cylinders, cycleDegreesOf(cycle), maxCandidates);
}

} // namespace enginelab
