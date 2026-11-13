#pragma once

#include <enginelab/audio/FreeFieldObserver.hpp>

#include <array>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace enginelab {

/** The ground between the lane and the microphone, by its effective flow
 *  resistivity (Embleton, Piercy & Daigle, JASA 74, 1983: asphalt sealed by
 *  dust 30,000 kPa s/m2, an old dirt or gravel road 2,000-4,000, grass
 *  150-300, new snow 10-30). */
enum class PassByGround : std::uint8_t {
    /** No reflected path: the microphone in free field. */
    none = 0,
    asphalt = 1,
    gravel = 2,
    grass = 3,
    snow = 4,
};

/** Where the trackside microphone stands and what lies between it and the car. */
struct PassBySettings final {
    /** From the microphone to the lane's centre line (ISO 362: 7.5 m). */
    double laneDistanceM { 7.5 };
    /** ISO 362: 1.2 m above the ground. */
    double microphoneHeightM { 1.2 };
    /** The car's sound as one point this high (an estimate: engine and
     *  tailpipe are 0.3-0.7 m up on a road car). */
    double sourceHeightM { 0.5 };
    /** Looping (no run): the car drives from -this to +this along the lane
     *  (0 in front of the microphone), then starts again; the last 30 m each
     *  way fade out. */
    double halfLengthM { 150.0 };
    PassByGround ground { PassByGround::asphalt };
    /** ISO 9613-1 air absorption (20 C, 70 % relative humidity, 101.325 kPa). */
    bool airAbsorption { true };
    /** The level the microphone really gets (false), or the spreading held at
     *  the lane's distance (true): the timbre a far car arrives with, as loud
     *  as at the closest approach. */
    bool normalisedLevel { false };
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
 *  image path, each with its retarded-time delay (the moving source's exact
 *  Doppler shift, for a speed constant over the delay) and its 1/r
 *  spreading; the image's comb is the ground's (a real reflection, the
 *  spherical wave's magnitude at the comb's first notch).
 *
 *  Then, on each microphone, what a delay and a real gain cannot carry:
 *  ISO 9613-1's air absorption (four one-pole low-passes set where the air
 *  takes 20 dB, and the rest) and the porous ground's spherical-wave
 *  reflection (Delany-Bazley impedance, Q = Rp + (1 - Rp) F(w)) as third-
 *  octave levels, by a 29-band graphic equaliser redesigned every 256
 *  samples from where the car is.
 *
 *  It loops along a 300 m lane at the vehicle's speed, or follows a run
 *  (setRun): the car's place is the run's, with no loop.
 *
 *  Switched on (once its delay line holds the car's sound) or off it
 *  cross-fades over 50 ms; off and faded it passes the
 *  signal through untouched (active() is false, the caller can skip it). */
class PassByMicrophone final {
public:
    /** Allocates the delay line (for a car up to `maximumRangeM` away). False
     *  on a bad setting. */
    [[nodiscard]] bool prepare(double sampleRateHz, const PassBySettings& settings);
    /** The delay line cleared, the car at `positionM` along the lane. */
    void reset(double positionM = 0.0) noexcept;
    void setEnabled(bool enabled) noexcept;
    /** Along the lane, per second; held to 80 % of the speed of sound. */
    void setVehicleSpeed(double metresPerSecond) noexcept;
    void setReferenceDistance(double metres) noexcept;
    /** Moves the car at once (tests, a new pass). */
    void setPosition(double positionM) noexcept;
    /** The microphone's place and the ground, live; no allocation. False (and
     *  nothing changed) on a bad figure. */
    bool setGeometry(double laneDistanceM, double microphoneHeightM, PassByGround ground,
                     bool normalisedLevel) noexcept;
    /** A run's car (once a block): `running` follows its place (a jump of
     *  more than 50 m restarts the delay line: the new place's sound has not
     *  arrived yet) and scales the sound by `gain`; not running, it loops. */
    void setRun(bool running, double positionM, double gain) noexcept;
    [[nodiscard]] bool active() const noexcept { return enabled_ || mix_ > 0.0F; }
    [[nodiscard]] bool prepared() const noexcept { return !delay_.empty(); }
    [[nodiscard]] bool running() const noexcept { return running_; }
    [[nodiscard]] double positionM() const noexcept { return positionM_; }
    [[nodiscard]] const PassBySettings& settings() const noexcept { return settings_; }
    /** `left`/`right` is everything heard; `exhaustLeft`/`exhaustRight` the
     *  part of it the tailpipe radiates (none: the car is one point). */
    [[nodiscard]] StereoPressure process(float left, float right, float exhaustLeft = 0.0F,
                                         float exhaustRight = 0.0F) noexcept;

    /** ISO 9613-1 pure-tone absorption, dB per metre, at 20 C, 70 % relative
     *  humidity and 101.325 kPa. */
    [[nodiscard]] static double airAbsorptionDbPerM(double hertz) noexcept;
    /** The Faddeeva function w(z) = exp(-z^2) erfc(-iz) (Weideman, SIAM J.
     *  Numer. Anal. 31, 1994, 32 terms). */
    [[nodiscard]] static std::complex<double> faddeeva(std::complex<double> z) noexcept;
    /** Delany-Bazley's normalised impedance of a ground of `flowResistivity`
     *  kPa s/m2 (exp(-i w t)). */
    [[nodiscard]] static std::complex<double> groundImpedance(double hertz, double flowResistivityKPa) noexcept;
    /** The spherical-wave reflection coefficient Q of the image path
     *  (exp(-i w t)), the source and microphone `horizontalM` apart. */
    [[nodiscard]] static std::complex<double> sphericalReflection(double hertz, double horizontalM,
        double sourceHeightM, double microphoneHeightM, double flowResistivityKPa,
        double soundSpeedMps) noexcept;
    [[nodiscard]] static double flowResistivityKPa(PassByGround ground) noexcept;
    /** The graphic equaliser's band centres: third octaves 25 Hz-16 kHz. */
    static constexpr std::size_t bandCount = 29;
    [[nodiscard]] static double bandCentreHz(std::size_t band) noexcept;

    static constexpr double fadeLengthM = 30.0;
    static constexpr double crossFadeSeconds = 0.050;
    /** The farthest a car is heard from: a run's 2 km and its lane. */
    static constexpr double maximumRangeM = 2'100.0;
    /** The equaliser and the air's low-passes are redesigned this often. */
    static constexpr std::size_t designSamples = 256;
    /** The equaliser's band levels are held to this range (dB). */
    static constexpr double lowestBandDb = -36.0;
    static constexpr double highestBandDb = 12.0;
    static constexpr std::size_t airStages = 4;

private:
    struct Biquad final {
        double b0 { 1.0 };
        double b1 {};
        double b2 {};
        double a1 {};
        double a2 {};
    };
    struct BiquadState final {
        double z1 {};
        double z2 {};
    };
    /** The delay (samples) and the emission distance for one point, the car
     *  at `alongM` now, moving at `speed`. */
    void retarded(double alongM, double heightDifferenceM, double offsetM,
                  double& delaySamples, double& distanceM) const noexcept;
    [[nodiscard]] float read(const std::vector<float>& line, double delaySamples) const noexcept;
    [[nodiscard]] std::size_t longestStartDelay() const noexcept;
    [[nodiscard]] float window(double alongM) const noexcept;
    void clearLines() noexcept;
    /** The equaliser and the air's low-passes for where the car is now. */
    void design() noexcept;
    [[nodiscard]] static Biquad peaking(double centreHz, double gainDb, double sampleRateHz) noexcept;
    [[nodiscard]] static double responseDb(const Biquad& biquad, double hertz, double sampleRateHz) noexcept;
    [[nodiscard]] double airStageLossDb(double hertz) const noexcept;

    PassBySettings settings_ {};
    /** The engine's share and the exhaust's, one sample each. */
    std::vector<float> delay_;
    std::vector<float> exhaustDelay_;
    std::size_t mask_ {};
    std::size_t write_ {};
    double longestDelaySamples_ {};
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

    // A run: its place (moved on at the car's speed between blocks), the
    // sound's share and its smoothed value.
    bool running_ { false };
    double runTargetM_ {};
    double runGain_ { 1.0 };
    double runGainSmoothed_ { 1.0 };
    double runCorrection_ {};
    double gainCoefficient_ {};

    // The image's real reflection, the air's low-pass coefficient (0: none)
    // and the equaliser, shared by the two microphones; their states.
    std::size_t bands_ {};
    std::vector<double> interactionInverse_;
    double imageReflection_ { 1.0 };
    double airCoefficient_ {};
    std::array<Biquad, bandCount> equaliser_ {};
    std::array<std::array<BiquadState, bandCount>, 2> equaliserState_ {};
    std::array<std::array<double, airStages>, 2> airState_ {};
    std::size_t designCountdown_ {};
};

} // namespace enginelab
