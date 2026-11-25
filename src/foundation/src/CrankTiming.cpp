#include <enginelab/foundation/CrankTiming.hpp>

#include <algorithm>
#include <cmath>
#include <numeric>

namespace enginelab {
namespace {

[[nodiscard]] double wrapped(double degrees, double period) noexcept {
    const auto value = std::fmod(std::fmod(degrees, period) + period, period);
    return value < period - 1.0e-9 ? value : 0.0;
}

[[nodiscard]] bool radialCrank(const EngineConfig& config) noexcept {
    if (config.layout == EngineLayout::radial) return true;
    return std::any_of(config.cylinders.begin(), config.cylinders.end(), [](const CylinderConfig& cylinder) {
        return cylinder.connectingRodType != ConnectingRodType::conventional;
    });
}

} // namespace

std::vector<std::uint32_t> firingOrderFor(const EngineConfig& config, const std::vector<double>& tdcDegrees) {
    const auto cycle = cycleDegreesOf(config.cycle);
    std::vector<std::size_t> index(std::min(config.cylinders.size(), tdcDegrees.size()));
    std::iota(index.begin(), index.end(), std::size_t { 0 });
    std::stable_sort(index.begin(), index.end(), [&](std::size_t a, std::size_t b) {
        return wrapped(tdcDegrees[a], cycle) < wrapped(tdcDegrees[b], cycle);
    });
    std::vector<std::uint32_t> order;
    order.reserve(index.size());
    for (const auto position : index) order.push_back(config.cylinders[position].id);
    return order;
}

std::string retimeCrank(EngineConfig& config, const std::vector<double>& tdcDegrees) {
    if (config.cycle == EngineCycle::rotary)
        return "A rotary fires as its rotors are spaced round the eccentric shaft.";
    if (radialCrank(config)) return "A radial's cylinders fire as they stand round its one crank throw.";
    if (tdcDegrees.size() != config.cylinders.size()) return "Give every cylinder its top dead centre.";
    const auto cycle = cycleDegreesOf(config.cycle);
    for (const auto tdc : tdcDegrees)
        if (!std::isfinite(tdc) || tdc < 0.0 || tdc >= cycle)
            return "A top dead centre is 0 to " + std::to_string(static_cast<int>(cycle)) + " degrees of the cycle.";
    // The pins as they will point, shared by the cylinders that shared one
    // and still point the same way.
    std::vector<CrankJournalConfig> journals;
    std::vector<std::uint32_t> journalOf(config.cylinders.size(), 0U);
    std::vector<std::uint32_t> formerOf;
    for (std::size_t index = 0; index < config.cylinders.size(); ++index) {
        const auto& cylinder = config.cylinders[index];
        const auto pin = wrapped(tdcDegrees[index] - cylinder.bankOffsetDegrees, 360.0);
        const auto former = std::find_if(config.crankJournals.begin(), config.crankJournals.end(),
            [&cylinder](const CrankJournalConfig& journal) { return journal.id == cylinder.crankJournalId; });
        const auto formerId = former != config.crankJournals.end() ? former->id : 0U;
        std::size_t found = 0;
        while (found < journals.size()
               && !(formerId != 0U && formerOf[found] == formerId
                    && std::abs(std::remainder(journals[found].angleDegrees - pin, 360.0)) < 1.0e-6))
            ++found;
        if (found == journals.size()) {
            CrankJournalConfig journal;
            journal.id = static_cast<std::uint32_t>(found + 1);
            journal.angleDegrees = pin;
            journal.throwMm = former != config.crankJournals.end() ? former->throwMm : 0.5 * cylinder.strokeMm;
            journal.crankshaftId = former != config.crankJournals.end() ? former->crankshaftId
                : config.crankshafts.empty() ? 1U : config.crankshafts.front().id;
            journals.push_back(journal);
            formerOf.push_back(formerId);
        }
        journalOf[index] = journals[found].id;
    }
    for (std::size_t index = 0; index < config.cylinders.size(); ++index) {
        config.cylinders[index].crankOffsetDegrees = tdcDegrees[index];
        config.cylinders[index].crankJournalId = journalOf[index];
    }
    config.crankJournals = std::move(journals);
    config.firingOrder = firingOrderFor(config, tdcDegrees);
    return {};
}

std::string setVBankAngle(EngineConfig& config, double degrees) {
    if (config.layout != EngineLayout::vLayout || config.banks.size() != 2U)
        return "Only a V's two banks open and close.";
    if (!std::isfinite(degrees) || degrees <= 0.0 || degrees > 180.0) return "A V's banks stand 1 to 180 degrees apart.";
    auto& first = config.banks[0];
    auto& second = config.banks[1];
    const auto centre = 0.5 * (first.angleDegrees + second.angleDegrees);
    const auto sign = second.angleDegrees >= first.angleDegrees ? 1.0 : -1.0;
    const auto firstShift = centre - sign * 0.5 * degrees - first.angleDegrees;
    const auto secondShift = centre + sign * 0.5 * degrees - second.angleDegrees;
    const auto cycle = cycleDegreesOf(config.cycle);
    std::vector<double> tdc;
    for (auto& cylinder : config.cylinders) {
        const auto shift = cylinder.bankId == first.id ? firstShift : cylinder.bankId == second.id ? secondShift : 0.0;
        cylinder.bankOffsetDegrees += shift;
        // The pin stays: the TDC follows the bank.
        cylinder.crankOffsetDegrees = wrapped(cylinder.crankOffsetDegrees + shift, cycle);
        tdc.push_back(cylinder.crankOffsetDegrees);
    }
    first.angleDegrees += firstShift;
    second.angleDegrees += secondShift;
    config.bankAngleDegrees = degrees;
    config.firingOrder = firingOrderFor(config, tdc);
    return {};
}

} // namespace enginelab
