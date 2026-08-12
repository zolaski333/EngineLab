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
 * The default represents a high-SPL engine recording path with 156 dB SPL
 * (RMS sine convention) at digital full scale, which is a normal preamp setting
 * for close exhaust miking with a measurement microphone.
 *
 * That figure is chosen from what the model actually radiates, not to taste.
 * Catalogue validation converts the signal back to pascals at each engine's
 * explicitly authored microphone pair and requires physical layer peaks to
 * remain below the safety limiter. The calibration therefore remains a capture
 * property; it is never adjusted per engine to normalise model output.
 *
 * The former 142 dB setting still clipped the measured Merlin startup peak:
 * 3.588 FS, about 1.28 kPa, and left the slow safety gain below unity for an
 * entire ten-second measurement. 156 dB corresponds to about 1.78 kPa sine
 * peak, retaining headroom for that catalogue maximum without per-engine gain,
 * AGC, or compression. It remains a capture-chain calibration rather than a
 * loudness-normalisation control.
 */
class AcousticMonitorCalibration final {
public:
    static constexpr double referenceRmsPressurePa = 20.0e-6;
    static constexpr double defaultFullScaleSplDb = 156.0;

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
