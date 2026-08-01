#pragma once

#include <enginelab/audio/DuctWallLoss.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <numbers>

namespace enginelab {

/** Passive absorption of a perforated-core, porous-packed exhaust silencer.
 *
 * The material surface impedance follows the empirical Delany-Bazley model for
 * a rigid-backed porous layer. The perforated open-area ratio scales how much
 * of that surface is acoustically exposed to the core. Defaults are zero/off:
 * no packing property is ever inferred from a mean-flow restriction.
 */
class PorousLinerLoss final {
public:
    [[nodiscard]] static double absorptionCoefficient(
        double frequencyHz, double densityKgPerM3, double soundSpeedMps,
        double flowResistivityPaSPerM2, double thicknessM) noexcept {
        if (!(frequencyHz > 0.0) || !(densityKgPerM3 > 0.0)
            || !(soundSpeedMps > 0.0) || !(flowResistivityPaSPerM2 > 0.0)
            || !(thicknessM > 0.0)) return 0.0;
        const auto x = std::clamp(
            densityKgPerM3 * frequencyHz / flowResistivityPaSPerM2,
            1.0e-5, 1.0e3);
        using Complex = std::complex<double>;
        const auto impedance0 = densityKgPerM3 * soundSpeedMps;
        const auto characteristic = impedance0 * Complex {
            1.0 + 0.0571 * std::pow(x, -0.754),
            -0.0870 * std::pow(x, -0.732)
        };
        const auto waveNumber = 2.0 * std::numbers::pi * frequencyHz
            / soundSpeedMps * Complex {
                1.0 + 0.0978 * std::pow(x, -0.700),
                -0.1890 * std::pow(x, -0.595)
            };
        const Complex imaginary { 0.0, 1.0 };
        const auto tangent = std::tan(waveNumber * thicknessM);
        if (std::abs(tangent) < 1.0e-12) return 0.0;
        const auto surface = -imaginary * characteristic / tangent;
        const auto reflection = (surface - impedance0) / (surface + impedance0);
        return std::clamp(1.0 - std::norm(reflection), 0.0, 1.0);
    }

    [[nodiscard]] static double traversalGain(
        double frequencyHz, double lengthM, double radiusM,
        double densityKgPerM3, double soundSpeedMps,
        double flowResistivityPaSPerM2, double thicknessM,
        double perforatedOpenAreaRatio) noexcept {
        if (!(lengthM > 0.0) || !(radiusM > 0.0)
            || !(perforatedOpenAreaRatio > 0.0)) return 1.0;
        const auto absorption = absorptionCoefficient(
            frequencyHz, densityKgPerM3, soundSpeedMps,
            flowResistivityPaSPerM2, thicknessM);
        // Plane-wave energy samples the lined perimeter as it propagates. For a
        // circular core, perimeter/area=2/r; converting energy loss to pressure
        // amplitude gives the L/(4r) exponent below. It is monotone and passive.
        const auto exponent = absorption
            * std::clamp(perforatedOpenAreaRatio, 0.0, 1.0)
            * lengthM / (4.0 * radiusM);
        return std::exp(-std::max(0.0, exponent));
    }

    [[nodiscard]] static DuctWallLoss::Coefficients fit(
        double lengthM, double radiusM, double densityKgPerM3,
        double soundSpeedMps, double flowResistivityPaSPerM2,
        double thicknessM, double perforatedOpenAreaRatio,
        double sampleRateHz) noexcept {
        DuctWallLoss::Coefficients result;
        if (!(sampleRateHz > 0.0) || !(flowResistivityPaSPerM2 > 0.0)
            || !(thicknessM > 0.0) || !(perforatedOpenAreaRatio > 0.0))
            return result;
        const auto upperHz = std::min(
            DuctWallLoss::upperDesignFrequencyHz, sampleRateHz * 0.45);
        const auto lowerHz = std::min(
            DuctWallLoss::lowerDesignFrequencyHz, upperHz * 0.4);
        const auto lowerGain = traversalGain(lowerHz, lengthM, radiusM,
            densityKgPerM3, soundSpeedMps, flowResistivityPaSPerM2,
            thicknessM, perforatedOpenAreaRatio);
        const auto upperGain = traversalGain(upperHz, lengthM, radiusM,
            densityKgPerM3, soundSpeedMps, flowResistivityPaSPerM2,
            thicknessM, perforatedOpenAreaRatio);
        if (!(lowerGain > 0.0) || lowerGain >= 1.0) return result;
        const auto warpedFrequency = [sampleRateHz](double frequencyHz) {
            const auto sine = std::sin(
                std::numbers::pi * frequencyHz / sampleRateHz);
            return sine * sine;
        };
        const auto lowerS = warpedFrequency(lowerHz);
        const auto upperS = warpedFrequency(upperHz);
        const auto lowerG2 = lowerGain * lowerGain;
        const auto upperG2 = upperGain * upperGain;
        const auto determinant = lowerS * upperS * (lowerG2 - upperG2);
        if (determinant > 1.0e-30) {
            const auto zeroWarp = (lowerG2 * lowerS * (upperG2 - 1.0)
                - (lowerG2 - 1.0) * upperG2 * upperS) / determinant;
            const auto poleWarp = (lowerS * (upperG2 - 1.0)
                - upperS * (lowerG2 - 1.0)) / determinant;
            if (zeroWarp > 0.0 && poleWarp > zeroWarp
                && std::isfinite(zeroWarp) && std::isfinite(poleWarp)) {
                result.zero = static_cast<float>(
                    DuctWallLoss::coefficientFromWarped(zeroWarp));
                result.pole = static_cast<float>(
                    DuctWallLoss::coefficientFromWarped(poleWarp));
                if (result.zero <= result.pole) {
                    result.renormalise();
                    return result;
                }
            }
        }
        result.zero = 0.0F;
        result.pole = static_cast<float>(DuctWallLoss::coefficientFromWarped(
            (1.0 / lowerG2 - 1.0) / lowerS));
        result.renormalise();
        return result;
    }
};

} // namespace enginelab
