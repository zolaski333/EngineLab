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

/** The evener crank first, then as better(). */
[[nodiscard]] bool betterCrank(const RowPinCrank& a, const RowPinCrank& b) {
    const auto unevenA = std::round(a.unevennessDegrees * 1000.0);
    const auto unevenB = std::round(b.unevennessDegrees * 1000.0);
    if (unevenA != unevenB) return unevenA < unevenB;
    const auto scoreA = std::round(score(a.balance) * 1000.0);
    const auto scoreB = std::round(score(b.balance) * 1000.0);
    if (scoreA != scoreB) return scoreA < scoreB;
    if (a.adjacentFirings != b.adjacentFirings) return a.adjacentFirings < b.adjacentFirings;
    return a.order < b.order;
}

/** Distinct values, ascending. */
[[nodiscard]] std::vector<double> distinct(std::vector<double> values) {
    std::sort(values.begin(), values.end());
    values.erase(std::unique(values.begin(), values.end(),
        [](double a, double b) { return std::abs(a - b) < 1.0e-9; }), values.end());
    return values;
}

[[nodiscard]] std::size_t rankOf(const std::vector<double>& values, double value) {
    std::size_t index = 0;
    while (index + 1 < values.size() && values[index] < value - 1.0e-9) ++index;
    return index;
}

/** An even-firing crank's top dead centres fall, modulo a turn, on multiples
 *  of `step`, `perSlot` of them on each (two on a four-stroke with an even
 *  count: one a turn after the other). */
struct EvenSlots final {
    std::size_t perSlot { 1 };
    double step { 360.0 };
};

[[nodiscard]] EvenSlots evenSlots(std::size_t count, double cycleDegrees) {
    const auto perSlot = cycleDegrees > 360.5 && count % 2 == 0 ? std::size_t { 2 } : std::size_t { 1 };
    return { perSlot, 360.0 * static_cast<double>(perSlot) / static_cast<double>(count) };
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

std::optional<RowPinCrank> rowPinCrank(const std::vector<BalanceCylinder>& cylinders, double cycleDegrees,
    double splitDegrees, const std::vector<std::uint32_t>& firingOrder, bool evenOnly) {
    const auto count = cylinders.size();
    if (count == 0) return std::nullopt;
    std::vector<double> positions;
    std::vector<double> banks;
    for (const auto& cylinder : cylinders) {
        positions.push_back(cylinder.axialPitches);
        banks.push_back(cylinder.bankDegrees);
    }
    positions = distinct(std::move(positions));
    banks = distinct(std::move(banks));
    const auto rows = positions.size();
    if (std::pow(static_cast<double>(rows), static_cast<double>(rows - 1)) > 200'000.0) return std::nullopt;
    // Where each cylinder's top dead centre falls, modulo a turn, with its
    // row's pin at zero: its bore, and its bank's lead on a split pin.
    std::vector<std::size_t> row(count);
    std::vector<double> lead(count);
    for (std::size_t index = 0; index < count; ++index) {
        row[index] = rankOf(positions, cylinders[index].axialPitches);
        lead[index] = cylinders[index].bankDegrees
            + splitDegrees * static_cast<double>(rankOf(banks, cylinders[index].bankDegrees));
    }
    const auto grid = 360.0 / static_cast<double>(rows);
    const auto fourStroke = cycleDegrees > 360.5;
    const auto interval = cycleDegrees / static_cast<double>(count);

    std::optional<RowPinCrank> best;
    auto working = cylinders;
    std::vector<double> sorted(count);
    const auto consider = [&](std::vector<double> tdc) {
        std::vector<std::uint32_t> order(count);
        std::iota(order.begin(), order.end(), 1U);
        std::stable_sort(order.begin(), order.end(), [&tdc](auto a, auto b) { return tdc[a - 1] < tdc[b - 1]; });
        if (!firingOrder.empty() && order != firingOrder) return;
        auto low = cycleDegrees;
        auto high = 0.0;
        for (std::size_t position = 0; position < count; ++position) {
            const auto next = position + 1 < count ? tdc[order[position + 1] - 1] : cycleDegrees;
            const auto gap = next - tdc[order[position] - 1];
            low = std::min(low, gap);
            high = std::max(high, gap);
        }
        if (best && std::round((high - low) * 1000.0) > std::round(best->unevennessDegrees * 1000.0)) return;
        RowPinCrank candidate;
        for (std::size_t index = 0; index < count; ++index) working[index].tdcDegrees = tdc[index];
        candidate.balance = analyseCrankBalance(working, cycleDegrees);
        candidate.adjacentFirings = adjacentFirings(working, order);
        candidate.unevennessDegrees = high - low;
        candidate.splitDegrees = splitDegrees;
        candidate.tdcDegrees = std::move(tdc);
        candidate.order = std::move(order);
        if (!best || betterCrank(candidate, *best)) best = std::move(candidate);
    };
    // Every set of pins, the first row's at zero.
    std::vector<double> phase(count);
    const auto forEachPinSet = [&](const auto& visit) {
        std::vector<std::size_t> digits(rows, 0);
        for (;;) {
            const auto first = static_cast<double>(digits[row[0]]) * grid + lead[0];
            for (std::size_t index = 0; index < count; ++index) {
                phase[index] = wrapped(static_cast<double>(digits[row[index]]) * grid + lead[index] - first, 360.0);
                if (phase[index] > 360.0 - 1.0e-6) phase[index] = 0.0;
            }
            visit();
            std::size_t place = 1;
            while (place < rows && ++digits[place] == rows) digits[place++] = 0;
            if (place >= rows) break;
        }
    };

    const auto slots = evenSlots(count, cycleDegrees);
    const auto slotCount = count / slots.perSlot;
    forEachPinSet([&] {
        std::vector<std::vector<std::size_t>> slot(slotCount);
        for (std::size_t index = 0; index < count; ++index) {
            const auto at = phase[index] / slots.step;
            if (std::abs(at - std::round(at)) > 1.0e-6) return;
            auto& members = slot[static_cast<std::size_t>(std::lround(at)) % slotCount];
            members.push_back(index);
            if (members.size() > slots.perSlot) return;
        }
        if (slots.perSlot == 1) {
            // Odd count: each top dead centre is the one of its two turns
            // that falls on an even-firing angle.
            auto tdc = phase;
            for (auto& angle : tdc) {
                const auto at = angle / interval;
                if (fourStroke && std::abs(at - std::round(at)) > 1.0e-6) angle += 360.0;
            }
            consider(std::move(tdc));
            return;
        }
        for (std::uint32_t mask = 0; mask < (1U << slotCount); ++mask) {
            std::vector<double> tdc(count);
            for (std::size_t index = 0; index < slotCount; ++index) {
                const auto later = (mask >> index) & 1U;
                tdc[slot[index][0]] = phase[slot[index][0]] + (later != 0U ? 360.0 : 0.0);
                tdc[slot[index][1]] = phase[slot[index][1]] + (later != 0U ? 0.0 : 360.0);
            }
            if (tdc[0] < 1.0) consider(std::move(tdc));
        }
    });
    if (best || evenOnly || (fourStroke && count > 10)) return best;

    forEachPinSet([&] {
        const auto variants = fourStroke ? 1U << (count - 1) : 1U;
        for (std::uint32_t mask = 0; mask < variants; ++mask) {
            auto tdc = phase;
            for (std::size_t index = 1; index < count; ++index)
                if (((mask >> (index - 1)) & 1U) != 0U) tdc[index] += 360.0;
            consider(std::move(tdc));
        }
    });
    return best;
}

std::optional<RowPinCrank> splitPinCrank(
    const std::vector<BalanceCylinder>& cylinders, double cycleDegrees, const std::vector<std::uint32_t>& firingOrder) {
    std::vector<double> banks;
    for (const auto& cylinder : cylinders) banks.push_back(cylinder.bankDegrees);
    banks = distinct(std::move(banks));
    if (banks.size() < 2) return std::nullopt;
    // The splits that bring the second bank onto the first's even-firing
    // angles, the least first (a 90 deg V6: +-30, then +-90...).
    const auto step = evenSlots(cylinders.size(), cycleDegrees).step;
    std::vector<double> splits;
    for (int multiple = 0; static_cast<double>(multiple) * step < 360.0 - 1.0e-9; ++multiple) {
        const auto split = std::remainder(static_cast<double>(multiple) * step - (banks[1] - banks[0]), 360.0);
        if (std::abs(split) > 1.0e-6) splits.push_back(split);
    }
    std::sort(splits.begin(), splits.end(), [](double a, double b) {
        return std::abs(std::abs(a) - std::abs(b)) > 1.0e-9 ? std::abs(a) < std::abs(b) : a > b;
    });
    std::optional<RowPinCrank> best;
    for (const auto split : splits) {
        if (best && std::abs(split) > std::abs(best->splitDegrees) + 1.0e-9) break;
        auto crank = rowPinCrank(cylinders, cycleDegrees, split, firingOrder, true);
        if (crank && (!best || betterCrank(*crank, *best))) best = std::move(crank);
    }
    return best;
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
