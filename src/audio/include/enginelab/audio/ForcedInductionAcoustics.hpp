#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

#include <array>
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

    struct ExhaustFlowSplit final {
        double turbineKgPerSecond {};
        double wastegateKgPerSecond {};
    };

    explicit ForcedInductionAcoustics(const ForcedInductionConfig& config,
        double observerDistanceM = 1.0) noexcept;

    [[nodiscard]] bool prepare(double sampleRateHz,
                               double observerDistanceM = 0.0) noexcept;
    void reset() noexcept;
    /** The listener glides to `distanceM` from the engine (kept within
     * FreeFieldObserver's microphone distances) with the microphones' time
     * constant (FreeFieldObserver::microphoneGlideSeconds); `immediately`
     * jumps there. Audio thread. */
    void moveObserver(double distanceM, bool immediately = false) noexcept;
    /** Same-binary diagnostic control for the pre-fix unnormalised bandpass. */
    void setBroadbandPowerNormalisationEnabled(bool enabled) noexcept {
        broadbandPowerNormalisationEnabled_ = enabled;
    }
    [[nodiscard]] bool broadbandPowerNormalisationEnabled() const noexcept {
        return broadbandPowerNormalisationEnabled_;
    }
    [[nodiscard]] float process(const Input& input) noexcept;

    /** Partitions the measured total exhaust flow through parallel effective
     * turbine and wastegate areas. The two results always conserve the input
     * flow; opening a wastegate never duplicates mass through both sources. */
    [[nodiscard]] ExhaustFlowSplit partitionExhaustFlow(
        double totalKgPerSecond, float wastegateOpening) const noexcept;

    [[nodiscard]] bool valid() const noexcept { return valid_; }
    /** Whether `a` and `b` describe the same machine, which only its sizes,
     * areas and efficiencies tell apart: on or off alike, the same type, the
     * same blade and lobe counts. */
    [[nodiscard]] static bool sameMachine(const ForcedInductionConfig& a,
                                          const ForcedInductionConfig& b) noexcept;
    /** Take another configuration of the same machine while it plays: nothing
     * is derived from it ahead of process(). False, changing nothing, when it
     * is another machine or would not be valid. Allocates nothing. */
    [[nodiscard]] bool updateConfig(const ForcedInductionConfig& config) noexcept;
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
    [[nodiscard]] float nextWhiteNoise(std::size_t source) noexcept;
    void smoothTelemetry(const Input& input) noexcept;

    ForcedInductionConfig config_;
    double sampleRateHz_ { 48'000.0 };
    double targetObserverDistanceM_ { 1.0 };
    double observerGlideCoefficient_ { 1.0 };
    double observerDistanceM_ { 1.0 };
    double configuredObserverDistanceM_ { 1.0 };
    double compressorPhase_ {};
    double turbinePhase_ {};
    BandNoiseState compressorNoise_ {};
    BandNoiseState turbineNoise_ {};
    BandNoiseState wastegateNoise_ {};
    BandNoiseState blowOffNoise_ {};
    Input smoothedInput_ {};
    std::array<std::uint32_t, 4> noiseStates_ {};
    float telemetrySmoothingCoefficient_ { 1.0F };
    float transientAttackCoefficient_ { 1.0F };
    float transientReleaseCoefficient_ { 1.0F };
    bool telemetryInitialised_ { false };
    bool broadbandPowerNormalisationEnabled_ { true };
    bool valid_ { false };
};

} // namespace enginelab
