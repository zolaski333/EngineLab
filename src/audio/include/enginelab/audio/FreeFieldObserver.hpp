#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

#include <array>
#include <cstddef>
#include <vector>

namespace enginelab {

struct StereoPressure final {
    float leftPa {};
    float rightPa {};
};

/** Causal free-field propagation from one circular opening to two microphones.
 *
 * Input pressure is the opening's monopole far field at one metre.  Each
 * microphone receives its own propagation delay and spherical 1/r decay.  The
 * ka transition is represented by the exact low-frequency monopole plus a
 * first-order high band carrying the termination directivity; this avoids a
 * non-causal, sample-by-sample frequency-domain approximation.
 */
class FreeFieldObserver final {
public:
    [[nodiscard]] bool prepare(double sampleRateHz, double apertureRadiusM,
        const AcousticPoint3M& sourcePositionM,
        const AcousticPoint3M& sourceAxis,
        AcousticTerminationType termination,
        const AcousticObserverConfig& observer) noexcept;
    void reset() noexcept;
    [[nodiscard]] StereoPressure process(float pressureAtOneMetrePa) noexcept;
    /** Move the microphones while it plays, to `left` and `right` as given
     * (no listening-distance scale), kept `minimumMicrophoneDistanceM` to
     * `maximumMicrophoneDistanceM` from the source: closer, a point source
     * radiating 1/r is no model of a pipe mouth or a jet. Each microphone glides to its new delay, 1/r and
     * directivity with a `microphoneGlideSeconds` time constant, its delay
     * changing by at most `maximumDelayRate` samples per sample: a Doppler
     * shift of at most 2 %, and no step to click; `immediately` jumps there,
     * for an observer not heard yet. Audio thread, after prepare(); allocates
     * nothing. */
    void moveMicrophones(const AcousticPoint3M& left, const AcousticPoint3M& right,
                         bool immediately = false) noexcept;
    static constexpr double minimumMicrophoneDistanceM = 0.25;
    static constexpr double maximumMicrophoneDistanceM = 40.0;
    static constexpr double microphoneGlideSeconds = 0.2;
    static constexpr float maximumDelayRate = 0.02F;
    /** The glide's tail: it lands on its target instead of stalling. */
    static constexpr float minimumDelayRate = 1.0e-3F;
    /** One sample of a listener distance gliding to 	arget: the same time
     * constant, then a linear tail of 1e-5 of the target per sample that
     * lands on it exactly. */
    [[nodiscard]] static double glideDistance(double current, double target, double coefficient) noexcept;

    [[nodiscard]] double distanceM(std::size_t microphone) const noexcept;
    [[nodiscard]] double directivity(std::size_t microphone) const noexcept;

private:
    struct Microphone final {
        std::vector<float> delay;
        std::size_t write {};
        std::size_t mask {};
        float delaySamples { 1.0F };
        float distanceScale { 1.0F };
        float highBandDirectivity { 1.0F };
        /** Where a moved microphone is gliding to. */
        float targetDelaySamples { 1.0F };
        float targetDistanceScale { 1.0F };
        float targetDirectivity { 1.0F };
        float lowState {};
        double distanceM { 1.0 };
    };
    /** The targets of a microphone at `position`; false when too close. */
    [[nodiscard]] bool aim(Microphone& microphone, const AcousticPoint3M& position) const noexcept;
    void glide(Microphone& microphone) const noexcept;
    [[nodiscard]] static float approach(float value, float target, float coefficient,
                                        float minimumStep, float maximumStep) noexcept;

    std::array<Microphone, 2> microphones_;
    float lowPassCoefficient_ {};
    float glideCoefficient_ {};
    double sampleRateHz_ { 48'000.0 };
    double soundSpeedMps_ { 343.0 };
    AcousticPoint3M sourcePositionM_ {};
    AcousticPoint3M axis_ { 0.0, 1.0, 0.0 };
    AcousticTerminationType termination_ { AcousticTerminationType::unflanged };
    bool prepared_ { false };
};

} // namespace enginelab
