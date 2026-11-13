#pragma once

#include <enginelab/runtime/CruiseControl.hpp>
#include <enginelab/runtime/DriverCommandArbiter.hpp>

#include <cstdint>
#include <optional>

namespace enginelab {

/** What the scripted driver of a pass-by run does in front of the
 *  microphone. */
enum class PassByProgram : std::uint8_t {
    /** From a standstill at the start, full throttle (on launch control when
     *  the tables carry it) through the gears. */
    standingStart = 0,
    /** At a constant speed, then full throttle from a point before the
     *  microphone (ISO 362's line AA', 10 m before it). */
    acceleration = 1,
    /** At a constant speed all along. */
    constantSpeed = 2,
    /** At full throttle, lifted in front of the microphone, coasting in gear
     *  after it. */
    liftAtMicrophone = 3,
};

/** Where a pass-by run is. */
enum class PassByPhase : std::uint8_t {
    off = 0,
    /** Brakes the car to a stop and starts the engine. */
    stopping = 1,
    /** At the start, first gear in, clutch down, on the brake; revs up for a
     *  standing start. */
    staging = 2,
    /** The constant programs get to their speed before the start, where the
     *  run begins: the car's place is held at the start meanwhile. */
    runUp = 3,
    /** The program, from the start to the end. */
    measuring = 4,
    /** Past the end: lifted, braking, the sound fading out. */
    finishing = 5,
    /** Stopped past the end, in neutral. */
    finished = 6,
};

struct PassByRunSettings final {
    PassByProgram program { PassByProgram::standingStart };
    /** Where the run begins, metres before the microphone. */
    double startDistanceM { 500.0 };
    /** Where it ends, metres past the microphone. */
    double endDistanceM { 500.0 };
    /** The constant programs' speed (held to 30-250 km/h: the cruise control
     *  holds it). */
    double approachSpeedKmh { 50.0 };
    /** The acceleration program's full throttle, metres before the
     *  microphone. */
    double fullThrottleBeforeM { 10.0 };
    /** 0: up through the gears (an automatic gearbox shifts itself); n: up to
     *  the n-th gear only, then held there. */
    int gear { 0 };
    /** A standing start on launch control, when the tables carry it. */
    bool launchControl { true };

    static constexpr double minimumDistanceM = 20.0;
    static constexpr double maximumDistanceM = 2'000.0;
    static constexpr double minimumApproachKmh = 30.0;
    static constexpr double maximumApproachKmh = 250.0;
};

/** The settings held to their ranges; a non-finite figure is its default. */
[[nodiscard]] PassByRunSettings sanitised(PassByRunSettings settings) noexcept;

/** What the scripted driver knows of the car. */
struct PassByVehicle final {
    double idleRpm { 800.0 };
    /** The lower of the redline and the rev limit. */
    double ceilingRpm { 6'500.0 };
    int gearCount { 5 };
    /** The gearbox shifts itself (TransmissionConfig::automaticShifting). */
    bool automaticGearbox { false };
    /** The tables carry launch control (`limits.launch_rpm` above zero). */
    bool launchControlFitted { false };
};

/** What the driver sees each frame. */
struct PassByObservation final {
    double speedMps { 0.0 };
    double rpm { 0.0 };
    /** The driveline's gear indices: -1 neutral, 0 first. */
    int requestedGear { -1 };
    int engagedGear { -1 };
    bool shiftInProgress { false };
};

/** The driver's answer for one frame. */
struct PassByCommand final {
    DriverInputs inputs {};
    /** A gear to ask the driveline for this frame (its index), if any. */
    std::optional<int> requestGear;
    bool shiftUp { false };
    /** The engine is stalled: the driver turns the key. */
    bool starter { false };
};

/** The scripted driver of a pass-by run: it stops the car, stages it at the
 *  start, drives the program past the microphone to the end, then brakes to a
 *  stop. The car's place along the lane (`positionM`, 0 in front of the
 *  microphone, negative before it) is the road speed integrated from the
 *  start. One thread; no allocation. */
class PassByDriver final {
public:
    void start(const PassByRunSettings& settings, const PassByVehicle& vehicle) noexcept;
    /** The driver lets go at once: the run is off. */
    void stop() noexcept;
    /** The driver holds the pedals (every phase but off and finished). */
    [[nodiscard]] bool driving() const noexcept {
        return phase_ != PassByPhase::off && phase_ != PassByPhase::finished;
    }
    [[nodiscard]] PassByCommand step(double dtSeconds, const PassByObservation& observation) noexcept;

    [[nodiscard]] PassByPhase phase() const noexcept { return phase_; }
    [[nodiscard]] double positionM() const noexcept { return positionM_; }
    /** Seconds since the run began at the start (0 before). */
    [[nodiscard]] double runSeconds() const noexcept { return runSeconds_; }
    /** The sound's share, 1 until the end, falling to 0 over `fadeSeconds`
     *  once past it. */
    [[nodiscard]] double soundGain() const noexcept { return soundGain_; }
    [[nodiscard]] const PassByRunSettings& settings() const noexcept { return settings_; }

    static constexpr double stagingSeconds = 2.0;
    /** Of the staging, the last this long revs a standing start up. */
    static constexpr double revSeconds = 1.2;
    static constexpr double fadeSeconds = 2.0;
    /** Upshifts at full throttle, as a share of the ceiling. */
    static constexpr double fullThrottleShift = 0.92;
    /** Upshifts while getting to a constant speed. */
    static constexpr double gentleShift = 0.55;

private:
    [[nodiscard]] int topGear() const noexcept;
    void upshiftAt(double shareOfCeiling, const PassByObservation& observation, PassByCommand& command) const noexcept;
    [[nodiscard]] double clutchRamp(double seconds) const noexcept;
    [[nodiscard]] double approachSpeedMps() const noexcept { return settings_.approachSpeedKmh / 3.6; }

    PassByRunSettings settings_ {};
    PassByVehicle vehicle_ {};
    PassByPhase phase_ { PassByPhase::off };
    CruiseControl cruise_;
    double phaseSeconds_ { 0.0 };
    /** Since the clutch began to come up. */
    double clutchSeconds_ { 0.0 };
    double positionM_ { 0.0 };
    double runSeconds_ { 0.0 };
    double soundGain_ { 1.0 };
    /** RunUp: how long the speed has been held. */
    double heldSeconds_ { 0.0 };
    double lastThrottle_ { 0.0 };
    bool lifted_ { false };
};

} // namespace enginelab
