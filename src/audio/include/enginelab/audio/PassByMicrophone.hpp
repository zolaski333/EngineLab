#pragma once

#include <enginelab/audio/FreeFieldObserver.hpp>

#include <array>
#include <cstddef>
#include <vector>

namespace enginelab {

/** Where the trackside microphone stands and what lies between it and the car. */
struct PassBySettings final {
    /** From the microphone to the lane's centre line (ISO 362: 7.5 m). */
    double laneDistanceM { 7.5 };
    /** ISO 362: 1.2 m above the ground. */
    double microphoneHeightM { 1.2 };
    /** The car's sound as one point this high (an estimate: engine and
     *  tailpipe are 0.3-0.7 m up on a road car). */
    double sourceHeightM { 0.5 };
    /** The car drives from -this to +this along the lane (0 in front of the
     *  microphone), then starts again; the last 30 m each way fade out. */
    double halfLengthM { 150.0 };
    /** The ground's pressure reflection, broadband: hard asphalt (ISO 10844
     *  test surfaces absorb under 10 %). 0 removes the reflected path. */
    double groundReflection { 0.9 };
    /** ISO 9613-1 air absorption (20 C, 70 % relative humidity). */
    bool airAbsorption { true };
    double soundSpeedMps { 343.0 };
    /** How far from the car the incoming signal was heard (the engine's own
     *  observer): the pass-by scales it by this over the distance. */
    double referenceDistanceM { 4.0 };
    /** The two microphones' spacing along the lane (left upstream). */
    double microphoneSpacingM { 0.18 };
    /** How far behind the engine the tailpipe is: the exhaust's share of the
     *  sound (process()) comes from there. */
    double exhaustBehindM { 0.0 };
};

/** The engine heard from the side of a lane while the car drives past: the
 *  incoming stereo (the engine at `referenceDistanceM`) is summed to two
 *  points of the car, the exhaust's share at the tailpipe and the rest at
 *  the engine, which move along the lane at the vehicle's speed. Each of the
 *  two microphones gets, from each point, the direct path and the ground's
 *  image path,
 *  each with its retarded-time delay (the moving source's exact Doppler shift,
 *  for a speed constant over the delay), its 1/r spreading and the air's
 *  absorption over its length.
 *
 *  Switched on (once its delay line holds the car's sound) or off it
 *  cross-fades over 50 ms; off and faded it passes the
 *  signal through untouched (active() is false, the caller can skip it). */
class PassByMicrophone final {
public:
    /** Allocates the delay line (2.5 s). False on a bad setting. */
    [[nodiscard]] bool prepare(double sampleRateHz, const PassBySettings& settings);
    /** The delay line cleared, the car at `positionM` along the lane. */
    void reset(double positionM = 0.0) noexcept;
    void setEnabled(bool enabled) noexcept;
    /** Along the lane, per second; held to 80 % of the speed of sound. */
    void setVehicleSpeed(double metresPerSecond) noexcept;
    void setReferenceDistance(double metres) noexcept;
    /** Moves the car at once (tests, a new pass). */
    void setPosition(double positionM) noexcept;
    [[nodiscard]] bool active() const noexcept { return enabled_ || mix_ > 0.0F; }
    [[nodiscard]] bool prepared() const noexcept { return !delay_.empty(); }
    [[nodiscard]] double positionM() const noexcept { return positionM_; }
    /** `left`/`right` is everything heard; `exhaustLeft`/`exhaustRight` the
     *  part of it the tailpipe radiates (none: the car is one point). */
    [[nodiscard]] StereoPressure process(float left, float right, float exhaustLeft = 0.0F,
                                         float exhaustRight = 0.0F) noexcept;

    /** The ISO 9613-1 absorption two one-pole low-passes in series follow:
     *  their loss, 20 log10(1 + (f/fc)^2), is 8.69 (f/fc)^2 dB below fc, the
     *  f^2 law of air; fc = 58 kHz / sqrt(r in m) matches ISO 9613-1 from
     *  10 to 100 m at 2 and 4 kHz (EngineLab.PassByMicrophone). Past fc it
     *  falls short: over the same 90 m, 8 kHz loses 7.7 dB for 10.5. */
    [[nodiscard]] static double absorptionCutoffHz(double distanceM) noexcept;
    static constexpr double fadeLengthM = 30.0;
    static constexpr double crossFadeSeconds = 0.050;
    static constexpr double delaySeconds = 2.5;

private:
    struct Path final {
        float first {};
        float second {};
    };
    /** The delay (samples) and the emission distance for one point, the car
     *  at `alongM` now, moving at `speed`. */
    void retarded(double alongM, double heightDifferenceM, double offsetM,
                  double& delaySamples, double& distanceM) const noexcept;
    [[nodiscard]] float read(const std::vector<float>& line, double delaySamples) const noexcept;
    [[nodiscard]] std::size_t longestStartDelay() const noexcept;
    [[nodiscard]] float window(double alongM) const noexcept;

    PassBySettings settings_ {};
    /** The engine's share and the exhaust's, one sample each. */
    std::vector<float> delay_;
    std::vector<float> exhaustDelay_;
    std::size_t mask_ {};
    std::size_t write_ {};
    std::array<Path, 8> paths_ {};
    double sampleRateHz_ { 48'000.0 };
    double positionM_ {};
    double speedMps_ {};
    double targetSpeedMps_ {};
    double speedCoefficient_ {};
    float mix_ {};
    float mixStep_ {};
    /** Switched on, the cross-fade waits until the delay line holds the
     *  signal of the path heard last: an empty line would step in. */
    std::size_t warmup_ {};
    bool enabled_ { false };
};

} // namespace enginelab
