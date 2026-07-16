#pragma once

#include <enginelab/calibration/CalibrationStore.hpp>

#include <string>
#include <string_view>

namespace enginelab::calibration {

class CalibrationJson final {
public:
    static constexpr int schemaVersion = 1;

    [[nodiscard]] static CalibrationResult<CalibrationDraft> parse(std::string_view jsonText);
    [[nodiscard]] static CalibrationResult<std::string> serialize(const CalibrationDraft& draft,
                                                                  int indentation = 2);
    [[nodiscard]] static std::string serialize(const CalibrationSnapshot& snapshot, int indentation = 2);
};

/** Parse, validate and publish as one transaction. A failure leaves the active snapshot unchanged. */
[[nodiscard]] PublishResult publishJson(CalibrationStore& store, std::string_view jsonText,
                                        const PublishOptions& options = {});

} // namespace enginelab::calibration
