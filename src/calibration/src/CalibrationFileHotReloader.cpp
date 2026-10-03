#include <enginelab/calibration/CalibrationFileHotReloader.hpp>

#include <fstream>
#include <iterator>
#include <system_error>

namespace enginelab::calibration {

void CalibrationFileHotReloader::watch(std::filesystem::path path) {
    path_ = std::move(path);
    observed_ = false;
    lastError_.clear();
    lastSize_ = 0;
    lastWriteTime_ = {};
    std::error_code error;
    if (std::filesystem::is_regular_file(path_, error) && !error) {
        lastWriteTime_ = std::filesystem::last_write_time(path_, error);
        if (!error) lastSize_ = std::filesystem::file_size(path_, error);
        observed_ = !error;
    }
}

void CalibrationFileHotReloader::stop() noexcept {
    path_.clear();
    observed_ = false;
    lastError_.clear();
}

CalibrationReloadResult CalibrationFileHotReloader::poll() noexcept {
    CalibrationReloadResult result;
    if (path_.empty()) return result;
    try {
        const auto reportError = [this, &result](std::string message) {
            result.changed = message != lastError_;
            result.error = std::move(message);
            lastError_ = result.error;
            return result;
        };
        std::error_code error;
        const auto status = std::filesystem::status(path_, error);
        if (error || !std::filesystem::is_regular_file(status)) {
            observed_ = false;
            return reportError("The watched calibration file was not found.");
        }
        const auto writeTime = std::filesystem::last_write_time(path_, error);
        if (error) {
            observed_ = false;
            return reportError("The calibration modification time is not readable.");
        }
        const auto size = std::filesystem::file_size(path_, error);
        if (error || size > 2U * 1024U * 1024U) {
            observed_ = false;
            return reportError(error ? "The calibration file size is not readable."
                                     : "The calibration exceeds the 2 MiB limit.");
        }
        if (observed_ && writeTime == lastWriteTime_ && size == lastSize_) return result;
        lastError_.clear();
        observed_ = true;
        lastWriteTime_ = writeTime;
        lastSize_ = size;
        result.changed = true;

        std::ifstream input(path_, std::ios::binary);
        if (!input) {
            return reportError("The calibration cannot be opened.");
        }
        const std::string json((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
        result.publication = publishJson(store_, json, { {}, path_.string() });
        if (!result.publication.published && !result.publication.issues.empty()) {
            result.error = result.publication.issues.front().path + ": "
                         + result.publication.issues.front().message;
            lastError_ = result.error;
        }
        return result;
    } catch (const std::exception& exception) {
        result.error = exception.what();
        result.changed = result.error != lastError_;
        lastError_ = result.error;
        return result;
    }
}

} // namespace enginelab::calibration
