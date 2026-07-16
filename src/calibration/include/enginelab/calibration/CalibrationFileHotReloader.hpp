#pragma once

#include <enginelab/calibration/CalibrationJson.hpp>

#include <cstdint>
#include <filesystem>
#include <string>

namespace enginelab::calibration {

struct CalibrationReloadResult final {
    bool changed { false };
    PublishResult publication;
    std::string error;

    [[nodiscard]] bool published() const noexcept { return publication.published; }
};

/**
 * Polling file adapter for editor-style hot reload.
 *
 * Parsing and validation complete before CalibrationStore atomically swaps the
 * active snapshot. A malformed save is reported and the last valid revision
 * remains live.
 */
class CalibrationFileHotReloader final {
public:
    explicit CalibrationFileHotReloader(CalibrationStore& store) noexcept : store_(store) {}

    void watch(std::filesystem::path path);
    void stop() noexcept;
    [[nodiscard]] const std::filesystem::path& path() const noexcept { return path_; }
    [[nodiscard]] bool watching() const noexcept { return !path_.empty(); }
    [[nodiscard]] CalibrationReloadResult poll() noexcept;

private:
    CalibrationStore& store_;
    std::filesystem::path path_;
    std::filesystem::file_time_type lastWriteTime_ {};
    std::uintmax_t lastSize_ {};
    bool observed_ { false };
    std::string lastError_;
};

} // namespace enginelab::calibration
