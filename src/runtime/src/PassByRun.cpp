#include <enginelab/runtime/PassByRun.hpp>

#include <algorithm>
#include <cmath>

namespace enginelab {
namespace {
// Standstill: below this road speed the car is stopped.
constexpr double stoppedMps = 0.3;
// The clutch comes up over this long on a standing start at full throttle,
// and over this long getting to a constant speed.
constexpr double launchClutchSeconds = 0.6;
constexpr double runUpClutchSeconds = 1.0;
constexpr double runUpThrottle = 0.45;
// The constant speed is reached once it has been held this close this long.
constexpr double heldKmh = 1.0;
constexpr double heldSeconds = 1.5;
// A run-up that never reaches its speed (a held gear too short) starts anyway.
constexpr double runUpTimeoutSeconds = 90.0;
constexpr double finishingBrake = 0.6;
// Braking past the end, the clutch goes down below this speed or this share
// of the idle speed, as a driver keeps the engine from stalling.
constexpr double declutchMps = 4.0;
constexpr double declutchIdleShare = 1.2;
// Without launch control the driver holds the revs near this share of the
// ceiling on the line.
constexpr double heldLaunchShare = 0.40;

[[nodiscard]] double finiteOr(double value, double fallback) noexcept {
    return std::isfinite(value) ? value : fallback;
}
} // namespace

PassByRunSettings sanitised(PassByRunSettings settings) noexcept {
    const PassByRunSettings defaults;
    if (static_cast<std::uint8_t>(settings.program) > static_cast<std::uint8_t>(PassByProgram::liftAtMicrophone))
        settings.program = defaults.program;
    settings.startDistanceM = std::clamp(finiteOr(settings.startDistanceM, defaults.startDistanceM),
                                         PassByRunSettings::minimumDistanceM, PassByRunSettings::maximumDistanceM);
    settings.endDistanceM = std::clamp(finiteOr(settings.endDistanceM, defaults.endDistanceM),
                                       PassByRunSettings::minimumDistanceM, PassByRunSettings::maximumDistanceM);
    settings.approachSpeedKmh = std::clamp(finiteOr(settings.approachSpeedKmh, defaults.approachSpeedKmh),
                                           PassByRunSettings::minimumApproachKmh,
                                           PassByRunSettings::maximumApproachKmh);
    settings.fullThrottleBeforeM = std::clamp(finiteOr(settings.fullThrottleBeforeM, defaults.fullThrottleBeforeM),
                                              0.0, settings.startDistanceM);
    // The gearbox's own count caps it (start()); no other limit.
    settings.gear = std::max(settings.gear, 0);
    return settings;
}

void PassByDriver::start(const PassByRunSettings& settings, const PassByVehicle& vehicle) noexcept {
    settings_ = sanitised(settings);
    vehicle_ = vehicle;
    vehicle_.gearCount = std::max(vehicle_.gearCount, 1);
    vehicle_.idleRpm = std::max(finiteOr(vehicle_.idleRpm, 800.0), 100.0);
    vehicle_.ceilingRpm = std::max(finiteOr(vehicle_.ceilingRpm, 6'500.0), 2.0 * vehicle_.idleRpm);
    phase_ = PassByPhase::stopping;
    cruise_.cancel();
    phaseSeconds_ = 0.0;
    clutchSeconds_ = 0.0;
    positionM_ = -settings_.startDistanceM;
    runSeconds_ = 0.0;
    soundGain_ = 1.0;
    heldSeconds_ = 0.0;
    lastThrottle_ = 0.0;
    lifted_ = false;
}

void PassByDriver::stop() noexcept {
    phase_ = PassByPhase::off;
    cruise_.cancel();
    soundGain_ = 1.0;
}

int PassByDriver::topGear() const noexcept {
    const auto top = settings_.gear == 0 ? vehicle_.gearCount : std::min(settings_.gear, vehicle_.gearCount);
    return top - 1;
}

void PassByDriver::upshiftAt(double shareOfCeiling, const PassByObservation& observation,
                             PassByCommand& command) const noexcept {
    if (vehicle_.automaticGearbox) return;
    if (observation.shiftInProgress || observation.requestedGear < 0
        || observation.engagedGear != observation.requestedGear || observation.requestedGear >= topGear())
        return;
    if (observation.rpm >= shareOfCeiling * vehicle_.ceilingRpm) command.shiftUp = true;
}

double PassByDriver::clutchRamp(double seconds) const noexcept {
    const auto t = std::clamp(clutchSeconds_ / seconds, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

PassByCommand PassByDriver::step(double dtSeconds, const PassByObservation& observation) noexcept {
    PassByCommand command;
    if (!driving()) return command;
    const auto dt = std::isfinite(dtSeconds) ? std::max(dtSeconds, 0.0) : 0.0;
    const auto speed = std::isfinite(observation.speedMps) ? std::abs(observation.speedMps) : 0.0;
    const auto stalled = !(observation.rpm >= 0.5 * vehicle_.idleRpm);
    phaseSeconds_ += dt;
    command.starter = stalled;
    const auto cruise = [&](PassByCommand& out) {
        if (!cruise_.engaged()) cruise_.engage(speed, lastThrottle_);
        out.inputs = cruise_.apply(dt, speed, out.inputs, observation.requestedGear >= 0);
    };
    const auto enter = [&](PassByPhase next) {
        phase_ = next;
        phaseSeconds_ = 0.0;
    };

    switch (phase_) {
    case PassByPhase::stopping:
        command.inputs.clutchPedal = 0.0;
        command.inputs.brake = 1.0;
        positionM_ = -settings_.startDistanceM;
        if (speed < stoppedMps && !stalled && phaseSeconds_ > 0.5) enter(PassByPhase::staging);
        break;
    case PassByPhase::staging: {
        command.inputs.clutchPedal = 0.0;
        command.inputs.brake = 1.0;
        if (observation.requestedGear != 0) command.requestGear = 0;
        const auto standing = settings_.program == PassByProgram::standingStart;
        if (standing && phaseSeconds_ > stagingSeconds - revSeconds) {
            if (settings_.launchControl && vehicle_.launchControlFitted) {
                command.inputs.throttle = 1.0;
            } else {
                const auto target = std::clamp(heldLaunchShare * vehicle_.ceilingRpm, 1.5 * vehicle_.idleRpm,
                                               vehicle_.ceilingRpm);
                command.inputs.throttle = std::clamp(1.2 * (target - observation.rpm) / target, 0.0, 1.0);
            }
        }
        const auto inFirst = observation.requestedGear == 0 && observation.engagedGear == 0
            && !observation.shiftInProgress;
        if (phaseSeconds_ >= stagingSeconds && !stalled && inFirst) {
            clutchSeconds_ = 0.0;
            heldSeconds_ = 0.0;
            if (standing) {
                runSeconds_ = 0.0;
                enter(PassByPhase::measuring);
            } else {
                enter(PassByPhase::runUp);
            }
        }
        break;
    }
    case PassByPhase::runUp: {
        clutchSeconds_ += dt;
        positionM_ = -settings_.startDistanceM;
        const auto target = approachSpeedMps();
        if (!cruise_.engaged() && speed < target - 0.3 / 3.6) {
            command.inputs.throttle = runUpThrottle;
            command.inputs.clutchPedal = clutchRamp(runUpClutchSeconds);
            upshiftAt(gentleShift, observation, command);
        } else {
            cruise(command);
            // An automatic choice keeps a sensible gear for the speed.
            if (settings_.gear == 0) upshiftAt(gentleShift, observation, command);
        }
        heldSeconds_ = std::abs(speed - target) < heldKmh / 3.6 ? heldSeconds_ + dt : 0.0;
        if (heldSeconds_ >= heldSeconds || phaseSeconds_ > runUpTimeoutSeconds) {
            runSeconds_ = 0.0;
            enter(PassByPhase::measuring);
        }
        break;
    }
    case PassByPhase::measuring: {
        clutchSeconds_ += dt;
        runSeconds_ += dt;
        positionM_ += speed * dt;
        const auto full = [&] {
            cruise_.cancel();
            command.inputs.throttle = 1.0;
            upshiftAt(fullThrottleShift, observation, command);
        };
        switch (settings_.program) {
        case PassByProgram::standingStart:
            full();
            command.inputs.clutchPedal = clutchRamp(launchClutchSeconds);
            break;
        case PassByProgram::acceleration:
            if (positionM_ < -settings_.fullThrottleBeforeM) cruise(command);
            else full();
            break;
        case PassByProgram::constantSpeed:
            cruise(command);
            break;
        case PassByProgram::liftAtMicrophone:
            if (positionM_ < 0.0 && !lifted_) {
                full();
            } else {
                lifted_ = true;
                cruise_.cancel();
            }
            break;
        }
        if (positionM_ >= settings_.endDistanceM) {
            cruise_.cancel();
            enter(PassByPhase::finishing);
        }
        break;
    }
    case PassByPhase::finishing:
        runSeconds_ += dt;
        positionM_ += speed * dt;
        command.inputs.brake = finishingBrake;
        command.inputs.clutchPedal = speed < declutchMps || observation.rpm < declutchIdleShare * vehicle_.idleRpm
            ? 0.0 : 1.0;
        command.starter = false;
        soundGain_ = std::max(0.0, 1.0 - phaseSeconds_ / fadeSeconds);
        if (speed < stoppedMps && soundGain_ <= 0.0) {
            command.inputs.brake = 1.0;
            command.inputs.clutchPedal = 0.0;
            command.requestGear = -1;
            enter(PassByPhase::finished);
        }
        break;
    case PassByPhase::off:
    case PassByPhase::finished:
        break;
    }
    lastThrottle_ = command.inputs.throttle;
    return command;
}

} // namespace enginelab
