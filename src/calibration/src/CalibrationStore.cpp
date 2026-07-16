#include <enginelab/calibration/CalibrationStore.hpp>

#include <algorithm>
#include <limits>
#include <type_traits>

namespace enginelab::calibration {

CalibrationSnapshot::CalibrationSnapshot(std::uint64_t revision, std::string name,
                                         std::string description, std::string source, EntryMap entries)
    : revision_(revision), name_(std::move(name)), description_(std::move(description)),
      source_(std::move(source)), entries_(std::move(entries)) {}

const CalibrationEntry* CalibrationSnapshot::find(std::string_view id) const noexcept {
    const auto found = entries_.find(id);
    return found == entries_.end() ? nullptr : &found->second;
}

std::optional<double> CalibrationSnapshot::scalar(std::string_view id) const noexcept {
    const auto* entry = find(id);
    if (entry == nullptr) return std::nullopt;
    const auto* scalarEntry = std::get_if<ScalarCalibration>(entry);
    return scalarEntry == nullptr ? std::nullopt : std::optional<double> { scalarEntry->value };
}

std::optional<double> CalibrationSnapshot::sampleCurve(std::string_view id,
                                                       const AxisCoordinate& coordinate) const noexcept {
    const auto* entry = find(id);
    if (entry == nullptr) return std::nullopt;
    const auto* curve = std::get_if<CalibrationCurve1D>(entry);
    return curve == nullptr ? std::nullopt : curve->sample(coordinate);
}

std::optional<double> CalibrationSnapshot::sampleTable(std::string_view id,
                                                       const AxisCoordinate& xCoordinate,
                                                       const AxisCoordinate& yCoordinate) const noexcept {
    const auto* entry = find(id);
    if (entry == nullptr) return std::nullopt;
    const auto* table = std::get_if<CalibrationTable2D>(entry);
    return table == nullptr ? std::nullopt : table->sample(xCoordinate, yCoordinate);
}

double CalibrationSnapshot::scalarOr(std::string_view id, double fallback) const noexcept {
    const auto sampled = scalar(id);
    return sampled.value_or(fallback);
}

double CalibrationSnapshot::sampleCurveOr(std::string_view id, const AxisCoordinate& coordinate,
                                          double fallback) const noexcept {
    const auto sampled = sampleCurve(id, coordinate);
    return sampled.value_or(fallback);
}

double CalibrationSnapshot::sampleTableOr(std::string_view id, const AxisCoordinate& xCoordinate,
                                          const AxisCoordinate& yCoordinate, double fallback) const noexcept {
    const auto sampled = sampleTable(id, xCoordinate, yCoordinate);
    return sampled.value_or(fallback);
}

CalibrationStore::CalibrationStore()
    : active_(std::shared_ptr<const CalibrationSnapshot>(new CalibrationSnapshot(
          0, {}, {}, "initial", CalibrationSnapshot::EntryMap {}))) {}

std::shared_ptr<const CalibrationSnapshot> CalibrationStore::snapshot() const noexcept {
    return active_.load(std::memory_order_acquire);
}

std::shared_ptr<CalibrationReaderEpoch> CalibrationStore::registerReader() {
    const std::lock_guard lock(publishMutex_);
    auto reader = std::shared_ptr<CalibrationReaderEpoch>(
        new CalibrationReaderEpoch(active_.load(std::memory_order_acquire)->revision()));
    readers_.push_back(reader);
    return reader;
}

void CalibrationStore::collectRetiredSnapshots(std::uint64_t activeRevision) {
    auto minimumAcknowledgedRevision = activeRevision + 1;
    auto output = readers_.begin();
    for (auto input = readers_.begin(); input != readers_.end(); ++input) {
        if (const auto reader = input->lock()) {
            minimumAcknowledgedRevision = std::min(minimumAcknowledgedRevision, reader->revision());
            *output++ = *input;
        }
    }
    readers_.erase(output, readers_.end());
    std::erase_if(retired_, [minimumAcknowledgedRevision](const auto& snapshot) {
        return snapshot->revision() < minimumAcknowledgedRevision;
    });
}

PublishResult CalibrationStore::publish(const CalibrationDraft& draft, const PublishOptions& options) {
    PublishResult result;
    result.issues = validate(draft);
    const auto beforeValidation = snapshot();
    result.previousRevision = beforeValidation->revision();
    result.activeRevision = beforeValidation->revision();
    if (!result.issues.empty()) return result;

    // Serialise writers so the revision and optimistic-concurrency check form one transaction.
    const std::lock_guard lock(publishMutex_);
    const auto current = snapshot();
    result.previousRevision = current->revision();
    result.activeRevision = current->revision();
    if (options.expectedRevision && *options.expectedRevision != current->revision()) {
        result.issues.push_back({ CalibrationErrorCode::revisionConflict, "revision",
            "The active calibration changed after this edit transaction was started." });
        return result;
    }
    if (current->revision() == std::numeric_limits<std::uint64_t>::max()) {
        result.issues.push_back({ CalibrationErrorCode::revisionOverflow, "revision",
            "The calibration revision counter cannot be incremented." });
        return result;
    }

    auto next = std::shared_ptr<const CalibrationSnapshot>(new CalibrationSnapshot(
        current->revision() + 1, draft.name, draft.description, options.source, draft.entries()));
    auto retired = active_.exchange(std::move(next), std::memory_order_acq_rel);
    // Retain the old object until every registered reader has acknowledged a
    // newer revision. Erasing here keeps its eventual destructor/allocation
    // cleanup on this writer thread, never on the simulation thread.
    retired_.push_back(std::move(retired));
    collectRetiredSnapshots(current->revision() + 1);
    result.published = true;
    result.activeRevision = current->revision() + 1;
    return result;
}

} // namespace enginelab::calibration
