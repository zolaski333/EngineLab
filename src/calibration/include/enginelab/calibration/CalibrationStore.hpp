#pragma once

#include <enginelab/calibration/CalibrationTypes.hpp>

#include <atomic>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace enginelab::calibration {

/** Immutable, internally consistent set consumed by the ECU and realtime UI readers. */
class CalibrationSnapshot final {
public:
    using EntryMap = CalibrationDraft::EntryMap;

    [[nodiscard]] std::uint64_t revision() const noexcept { return revision_; }
    [[nodiscard]] const std::string& name() const noexcept { return name_; }
    [[nodiscard]] const std::string& description() const noexcept { return description_; }
    [[nodiscard]] const std::string& source() const noexcept { return source_; }
    [[nodiscard]] const EntryMap& entries() const noexcept { return entries_; }
    [[nodiscard]] const CalibrationEntry* find(std::string_view id) const noexcept;

    [[nodiscard]] std::optional<double> scalar(std::string_view id) const noexcept;
    [[nodiscard]] std::optional<double> sampleCurve(std::string_view id,
                                                    const AxisCoordinate& coordinate) const noexcept;
    [[nodiscard]] std::optional<double> sampleTable(std::string_view id,
                                                    const AxisCoordinate& xCoordinate,
                                                    const AxisCoordinate& yCoordinate) const noexcept;

    [[nodiscard]] double scalarOr(std::string_view id, double fallback) const noexcept;
    [[nodiscard]] double sampleCurveOr(std::string_view id, const AxisCoordinate& coordinate,
                                       double fallback) const noexcept;
    [[nodiscard]] double sampleTableOr(std::string_view id, const AxisCoordinate& xCoordinate,
                                       const AxisCoordinate& yCoordinate, double fallback) const noexcept;

private:
    friend class CalibrationStore;
    CalibrationSnapshot(std::uint64_t revision, std::string name, std::string description,
                        std::string source, EntryMap entries);

    std::uint64_t revision_ { 0 };
    std::string name_;
    std::string description_;
    std::string source_;
    EntryMap entries_;
};

struct PublishOptions {
    /** Optimistic concurrency guard used by editors to prevent overwriting a newer revision. */
    std::optional<std::uint64_t> expectedRevision;
    std::string source;
};

struct PublishResult {
    bool published { false };
    std::uint64_t previousRevision { 0 };
    std::uint64_t activeRevision { 0 };
    std::vector<CalibrationIssue> issues;
};

/**
 * Per-consumer epoch used to reclaim superseded snapshots away from realtime
 * threads. Consumers acknowledge a revision only after they own that snapshot.
 */
class CalibrationReaderEpoch final {
public:
    CalibrationReaderEpoch(const CalibrationReaderEpoch&) = delete;
    CalibrationReaderEpoch& operator=(const CalibrationReaderEpoch&) = delete;

    void acknowledge(std::uint64_t revision) noexcept {
        revision_.store(revision, std::memory_order_release);
    }
    [[nodiscard]] std::uint64_t revision() const noexcept {
        return revision_.load(std::memory_order_acquire);
    }

private:
    friend class CalibrationStore;
    explicit CalibrationReaderEpoch(std::uint64_t revision) noexcept : revision_(revision) {}
    std::atomic<std::uint64_t> revision_ { 0 };
};

/**
 * Transactional snapshot exchange. Readers never lock or observe a partially edited calibration.
 * Validation and allocation finish before the atomic pointer is replaced.
 */
class CalibrationStore final {
public:
    CalibrationStore();

    [[nodiscard]] std::shared_ptr<const CalibrationSnapshot> snapshot() const noexcept;
    [[nodiscard]] std::shared_ptr<CalibrationReaderEpoch> registerReader();
    [[nodiscard]] PublishResult publish(const CalibrationDraft& draft,
                                        const PublishOptions& options = {});

private:
    void collectRetiredSnapshots(std::uint64_t activeRevision);

    mutable std::mutex publishMutex_;
    std::atomic<std::shared_ptr<const CalibrationSnapshot>> active_;
    std::vector<std::weak_ptr<CalibrationReaderEpoch>> readers_;
    std::vector<std::shared_ptr<const CalibrationSnapshot>> retired_;
};

} // namespace enginelab::calibration
