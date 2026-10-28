#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

#include <atomic>
#include <string>
#include <vector>

namespace enginelab {

/**
 * The knock-limited spark at full load, as a calibrator finds it on the dyno:
 * the engine is held at wide-open throttle at six speeds from 0.95 of its
 * limiter down, its own knock control on, and the spark the control settles
 * at is the limit. Each point is warm-started from the one above it, because
 * the control pulls spark back in a fraction of a second and gives it back at
 * a degree a second. A point where the map ran clean reads the map itself.
 *
 * Deterministic: the simulator is stepped frame by frame, as
 * `EngineLabDynoSweepHarness` does, so the same engine always gives the same
 * curve whatever the computer. A spark-ignition engine only; for a diesel the
 * curve is empty.
 */
struct KnockLimitCalibration final {
    std::vector<KnockLimitSample> curve;
    /** The knock control's mean retard at each point, the gap the map had. */
    std::vector<double> retardDegrees;
    /** Cylinder 1's 50 % heat-release angle after firing TDC at each point,
     *  the phasing the literature quotes knock limits in. */
    std::vector<double> ca50Degrees;
    /** Empty when the curve was measured; otherwise why it was not. */
    std::string error;
};

/** `cancel` may be null; when it is set the calibration stops and reports
 *  "cancelled". Any knock limit already on `config` is ignored. */
[[nodiscard]] KnockLimitCalibration calibrateKnockLimit(const EngineConfig& config,
                                                        const std::atomic<bool>* cancel = nullptr);

/** The same sweep with `config` as it is, its knock limit kept: the spark and
 *  the retard the knock control still adds behind a calibrated map. */
[[nodiscard]] KnockLimitCalibration measureFullLoadSpark(const EngineConfig& config,
                                                         const std::atomic<bool>* cancel = nullptr);

} // namespace enginelab
