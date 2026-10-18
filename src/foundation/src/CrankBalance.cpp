#include <enginelab/foundation/CrankBalance.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <set>

namespace enginelab {
namespace {

constexpr double degreesToRadians = std::numbers::pi / 180.0;
/** Crank positions sampled over a turn; a couple's peak is found to < 0.1 %. */
constexpr int turnSamples = 720;

[[nodiscard]] double wrapped(double degrees, double period) {
    const auto value = std::fmod(degrees, period);
    return value < 0.0 ? value + period : value;
}

/** Peak over a turn of |sum of weight x cos(order x (theta - tdc)) x axis|,
 *  with axis the bore direction, times the axial lever when `couple`. */
[[nodiscard]] double peakAmplitude(
    const std::vector<BalanceCylinder>& cylinders, double order, bool couple, double centroid) {
    auto peak = 0.0;
    for (int sample = 0; sample < turnSamples; ++sample) {
        const auto theta = 360.0 * static_cast<double>(sample) / static_cast<double>(turnSamples);
        auto x = 0.0;
        auto y = 0.0;
        for (const auto& cylinder : cylinders) {
            const auto weight = cylinder.massRadius * (order > 1.5 ? cylinder.rodRatio : 1.0)
                * std::cos(order * (theta - cylinder.tdcDegrees) * degreesToRadians)
                * (couple ? cylinder.axialPitches - centroid : 1.0);
            x += weight * std::sin(cylinder.bankDegrees * degreesToRadians);
            y += weight * std::cos(cylinder.bankDegrees * degreesToRadians);
        }
        peak = std::max(peak, std::hypot(x, y));
    }
    return peak;
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

std::vector<FiringOrderCandidate> inlineFiringOrders(
    std::uint32_t cylinderCount, EngineCycle cycle, double rodRatio, std::size_t maxCandidates) {
    std::vector<FiringOrderCandidate> candidates;
    if (cylinderCount == 0 || cylinderCount > 9) return candidates;
    const auto cycleDegrees = cycleDegreesOf(cycle);
    const auto interval = cycleDegrees / static_cast<double>(cylinderCount);
    std::vector<std::uint32_t> rest;
    for (std::uint32_t id = 2; id <= cylinderCount; ++id) rest.push_back(id);
    do {
        FiringOrderCandidate candidate;
        candidate.order.push_back(1);
        candidate.order.insert(candidate.order.end(), rest.begin(), rest.end());
        std::vector<BalanceCylinder> cylinders(cylinderCount);
        for (std::size_t position = 0; position < candidate.order.size(); ++position) {
            auto& cylinder = cylinders[candidate.order[position] - 1];
            cylinder.tdcDegrees = static_cast<double>(position) * interval;
            cylinder.axialPitches = static_cast<double>(candidate.order[position] - 1);
            cylinder.rodRatio = rodRatio;
        }
        candidate.balance = analyseCrankBalance(cylinders, cycleDegrees);
        for (std::size_t position = 0; position < candidate.order.size(); ++position) {
            const auto next = candidate.order[(position + 1) % candidate.order.size()];
            const auto current = candidate.order[position];
            if (cylinderCount > 2 && (next == current + 1 || current == next + 1)) ++candidate.adjacentFirings;
        }
        candidates.push_back(std::move(candidate));
    } while (std::next_permutation(rest.begin(), rest.end()));
    // Couples within 0.1 % of a cylinder's m r w^2 x pitch rank as equal.
    const auto score = [](const FiringOrderCandidate& candidate) {
        return std::round((candidate.balance.primaryCouple + candidate.balance.secondaryCouple) * 1000.0);
    };
    std::stable_sort(candidates.begin(), candidates.end(), [&score](const auto& a, const auto& b) {
        const auto scoreA = score(a);
        const auto scoreB = score(b);
        if (scoreA != scoreB) return scoreA < scoreB;
        return a.adjacentFirings < b.adjacentFirings;
    });
    if (candidates.size() > maxCandidates) candidates.resize(maxCandidates);
    return candidates;
}

} // namespace enginelab
