#pragma once

#include <enginelab/calibration/CalibrationStore.hpp>
#include <enginelab/foundation/EngineTypes.hpp>

namespace enginelab::calibration {

/** Build the live-editable baseline maps used by SimpleEcuModel. */
[[nodiscard]] CalibrationDraft makeDefaultEcuCalibration(const EngineConfig& config);

/** Adds to tables saved before them the settings whose absence already meant
 *  their default, so the engine runs exactly as before and the tuner shows
 *  them: the overboost cut of a turbocharged engine. Widens a target-AFR
 *  limit saved narrower than today's (18.0 before 2026-11-14). */
void addMissingDefaultSettings(CalibrationDraft& draft, const EngineConfig& config);

/** Puts the tables' VTEC settings (`vtec.*`) in step with the engine: adds
 *  the missing ones, seeded from its first VTEC camshaft, when it has VTEC
 *  cams; removes them all when it has none. Whether the draft changed. */
bool syncVtecSettings(CalibrationDraft& draft, const EngineConfig& config);

/** Puts the tables' equipment settings in step with the engine, so a part
 *  fitted to the running engine brings its settings: VTEC (syncVtecSettings);
 *  a turbocharger's boost target, overboost cut and, on petrol, anti-lag,
 *  each added when missing, as its generated tables have them. A turbo fitted
 *  live kept the tables of the engine without one, so the Equipment window
 *  offered no anti-lag (owner report 2026-11-16, the flat-6 3.6). Anti-lag
 *  leaves the tables of an engine that cannot carry it. Whether the draft
 *  changed. */
bool syncEquipmentSettings(CalibrationDraft& draft, const EngineConfig& config);

/** Whether syncEquipmentSettings would leave these tables as they are: a
 *  lookup per setting, cheap enough to ask every frame. */
[[nodiscard]] bool equipmentSettingsInStep(const CalibrationSnapshot& snapshot, const EngineConfig& config);

/** Create a mutable transaction from an immutable realtime snapshot. */
[[nodiscard]] CalibrationDraft makeDraft(const CalibrationSnapshot& snapshot);

/** Moves the boost-target table, when the store has one, by a gauge step
 *  (within its limits). The table is the one owner of the boost target: an
 *  edit of the configuration's wastegate ratio moves it by the same step
 *  (101.325 kPa per unit of ratio, as the table was seeded). Returns whether
 *  a table was published. */
bool shiftBoostTargetTable(CalibrationStore& store, double gaugeStepKpa);

} // namespace enginelab::calibration
