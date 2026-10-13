#pragma once

#include <cmath>

namespace enginelab {

/**
 * Explicit conversion between SI acoustic pressure and a digital monitor.
 *
 * Pascals do not have an intrinsic dBFS representation: the missing quantity
 * is the microphone/preamp full-scale SPL. Keeping that calibration explicit
 * prevents an arbitrary voice gain or the safety limiter from silently filling
 * the role of a capture chain.
 *
 * The default represents a high-SPL engine recording path with 140 dB SPL
 * (RMS sine convention) at digital full scale. This maps 200 Pa RMS, or
 * 282.8 Pa for a sine peak, to 0 dBFS.
 *
 * That figure is chosen from what the model actually radiates, not to taste.
 * Catalogue validation converts the signal back to pascals at each engine's
 * explicitly authored microphone pair and requires physical layer peaks to
 * remain below the safety limiter. The calibration therefore remains a capture
 * property; it is never adjusted per engine to normalise model output.
 *
 * A temporary 156 dB setting was derived from an obsolete Merlin startup peak.
 * The current 16-engine catalogue measurement instead observes at most about
 * 70 Pa from any SI pressure source. The remaining Merlin startup peak belongs
 * to the legacy synthetic starter layer, which is already digital and is not
 * affected by this conversion. Using that unrelated layer to attenuate every
 * physical pressure source was therefore dimensionally wrong and made normal
 * running roughly 22 dB too quiet. Catalogue output-chain gates retain the
 * required limiter and hard-clamp headroom at this default.
 *
 * 134 dB became 140 dB on 2026-10-13, when the outlets started radiating into
 * ambient air instead of the hot duct gas (+2.5 to +11.7 dB of exhaust). At
 * 0.95 x redline, wide open, the loudest clean steady peak is the CP3's, about
 * 210 Pa; keeping it under the 0.82 knee needs at least 139.1 dB. The
 * Aircooled's isolated spikes (crest ~24 dB) already reached the limiter at
 * 134 dB before that change and are not what sets this figure.
 */
class AcousticMonitorCalibration final {
public:
    static constexpr double referenceRmsPressurePa = 20.0e-6;
    static constexpr double defaultFullScaleSplDb = 140.0;

    [[nodiscard]] static double rmsPressurePa(double soundPressureLevelDb) noexcept {
        return std::isfinite(soundPressureLevelDb)
            ? referenceRmsPressurePa * std::pow(10.0, soundPressureLevelDb / 20.0)
            : 0.0;
    }

    [[nodiscard]] static double sinePeakPressurePa(
        double soundPressureLevelDb) noexcept {
        return std::sqrt(2.0) * rmsPressurePa(soundPressureLevelDb);
    }

    [[nodiscard]] static double normalisePeakPressure(
        double pressurePa, double fullScaleSoundPressureLevelDb) noexcept {
        const auto fullScalePeakPressurePa = sinePeakPressurePa(
            fullScaleSoundPressureLevelDb);
        return std::isfinite(pressurePa) && fullScalePeakPressurePa > 0.0
            ? pressurePa / fullScalePeakPressurePa : 0.0;
    }
};

} // namespace enginelab
