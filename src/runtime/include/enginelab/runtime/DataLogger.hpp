#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

#include <filesystem>
#include <fstream>
#include <string>

namespace enginelab {

/**
 * A datalogger: the engine's main channels, one CSV row per pushed state, as
 * a tuner's logger records them during a pull or a drive. SI units (kPa,
 * degrees Celsius, km/h for the road speed), time from the start of the log.
 *
 * Rows closer than `minimumIntervalSeconds` of simulated time are skipped,
 * so a paused or slowed simulation does not repeat rows. The file is
 * flushed on `stop()` and when the logger is destroyed.
 */
class DataLogger final {
public:
    DataLogger() = default;
    ~DataLogger();
    DataLogger(const DataLogger&) = delete;
    DataLogger& operator=(const DataLogger&) = delete;

    /** Starts a log at `file` (its folder is created). False if it cannot be
     *  opened; a log already running is stopped first. */
    bool start(const std::filesystem::path& file, double minimumIntervalSeconds = 0.02);
    void push(const EngineState& state);
    void stop();

    [[nodiscard]] bool recording() const noexcept { return stream_.is_open(); }
    [[nodiscard]] const std::filesystem::path& file() const noexcept { return file_; }
    [[nodiscard]] std::size_t rows() const noexcept { return rows_; }

    /** The header line, without its newline. */
    [[nodiscard]] static std::string header();
    /** One row for `state`, `timeSeconds` into the log, without its newline. */
    [[nodiscard]] static std::string row(const EngineState& state, double timeSeconds);

private:
    std::ofstream stream_;
    std::filesystem::path file_;
    double startSeconds_ { 0.0 };
    double lastSeconds_ { 0.0 };
    double minimumIntervalSeconds_ { 0.02 };
    std::size_t rows_ { 0 };
};

} // namespace enginelab
