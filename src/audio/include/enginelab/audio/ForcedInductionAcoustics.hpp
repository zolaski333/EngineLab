#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

#include <cstdint>

namespace enginelab {

/** Semi-empirical aeroacoustics driven exclusively by physical rotor/flow data.
 *
 * Tonal frequencies are exact blade/lobe-passing orders. Tone power is a
 * configured fraction of resolved shaft power. Broadband jet power follows
 * the compact subsonic-jet U^8 law and is shaped around Strouhal 0.2. This is
 * deliberately identified as semi-empirical: CFD or measured compressor maps
 * would be required for absolute prediction.
 */
class ForcedInductionAcoustics final {
public:
    struct Input final {
        float shaftSpeedRpm {};
        float correctedAirFlowKgPerSecond {};
        float pressureRatio { 1.0F };
        float compressorPowerWatts {};
        float turbinePowerWatts {};
        float exhaustMassFlowKgPerSecond {};
        float wastegateOpening {};
        float blowOffMassFlowKgPerSecond {};
        float densityKgPerM3 { 1.2F };
        float soundSpeedMps { 343.0F };
        /** Scales the complete spectrum for slow-motion auditioning. */
        float acousticTimeScale { 1.0F };
    };

    explicit ForcedInductionAcoustics(const ForcedInductionConfig& config,
        double observerDistanceM = 1.0) noexcept;

    [[nodiscard]] bool prepare(double sampleRateHz,
                               double observerDistanceM = 0.0) noexcept;
    void reset() noexcept;
    [[nodiscard]] float process(const Input& input, float whiteNoise) noexcept;

    [[nodiscard]] bool valid() const noexcept { return valid_; }
    [[nodiscard]] bool semiEmpirical() const noexcept { return true; }

private:
    struct BandNoiseState final {
        float lowSlow {};
        float lowFast {};
    };

    [[nodiscard]] float bandNoise(float noise, double centreHz,
                                  BandNoiseState& state) const noexcept;
    [[nodiscard]] double pressurePeakFromPower(double powerWatts,
                                               double densityKgPerM3,
                                               double soundSpeedMps) const noexcept;
    [[nodiscard]] double jetPower(double massFlowKgPerSecond, double areaM2,
                                  double densityKgPerM3,
                                  double soundSpeedMps) const noexcept;

    ForcedInductionConfig config_;
    double sampleRateHz_ { 48'000.0 };
    double observerDistanceM_ { 1.0 };
    double configuredObserverDistanceM_ { 1.0 };
    double compressorPhase_ {};
    double turbinePhase_ {};
    BandNoiseState compressorNoise_ {};
    BandNoiseState turbineNoise_ {};
    BandNoiseState wastegateNoise_ {};
    BandNoiseState blowOffNoise_ {};
    bool valid_ { false };
};

} // namespace enginelab
