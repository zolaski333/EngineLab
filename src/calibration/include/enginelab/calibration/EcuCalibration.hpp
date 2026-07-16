#pragma once

#include <enginelab/calibration/CalibrationStore.hpp>
#include <enginelab/foundation/EngineTypes.hpp>

namespace enginelab::calibration {

/** Build the live-editable baseline maps used by SimpleEcuModel. */
[[nodiscard]] CalibrationDraft makeDefaultEcuCalibration(const EngineConfig& config);

/** Create a mutable transaction from an immutable realtime snapshot. */
[[nodiscard]] CalibrationDraft makeDraft(const CalibrationSnapshot& snapshot);

} // namespace enginelab::calibration
