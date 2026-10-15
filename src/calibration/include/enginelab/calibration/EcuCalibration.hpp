#pragma once

#include <enginelab/calibration/CalibrationStore.hpp>
#include <enginelab/foundation/EngineTypes.hpp>

namespace enginelab::calibration {

/** Build the live-editable baseline maps used by SimpleEcuModel. */
[[nodiscard]] CalibrationDraft makeDefaultEcuCalibration(const EngineConfig& config);

/** Create a mutable transaction from an immutable realtime snapshot. */
[[nodiscard]] CalibrationDraft makeDraft(const CalibrationSnapshot& snapshot);

/** Moves the boost-target table, when the store has one, by a gauge step
 *  (within its limits). The table is the one owner of the boost target: an
 *  edit of the configuration's wastegate ratio moves it by the same step
 *  (101.325 kPa per unit of ratio, as the table was seeded). Returns whether
 *  a table was published. */
bool shiftBoostTargetTable(CalibrationStore& store, double gaugeStepKpa);

} // namespace enginelab::calibration
