#pragma once

#include <enginelab/calibration/CalibrationStore.hpp>

#include <functional>
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

/** Fills in what a file older than the running engine's tables lacks
    (addMissingDefaultSettings), before it is published. */
using DraftCompletion = std::function<void(CalibrationDraft&)>;
/** As publishJson, the parsed draft completed first when `complete` is set. */
[[nodiscard]] PublishResult publishJson(CalibrationStore& store, std::string_view jsonText,
                                        const PublishOptions& options, const DraftCompletion& complete);

} // namespace enginelab::calibration
