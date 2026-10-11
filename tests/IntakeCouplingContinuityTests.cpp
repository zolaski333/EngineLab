// Coupling time must follow physical elapsed time when the mechanical grid
// changes. These motored tests use public simulator telemetry: no combustion,
// private-state access, manufacturer calibration or torque-curve reference.
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/foundation/EngineTypes.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <numbers>
#include <string_view>

namespace {
constexpr double frameSeconds = 1.0 / 240.0;
constexpr double requestedIntervalSeconds = 400.0e-6;
constexpr double elapsedTimeToleranceSeconds = 1.0e-9;

void require(bool condition, std::string_view message) {
    if (condition) return;
    std::cerr << "FAILED: " << message << '\n';
    std::exit(EXIT_FAILURE);
}

[[nodiscard]] enginelab::EngineConfig fixtureConfig() {
    auto config = enginelab::makeDefaultInlineTwo();
    config.solver.mechanicalFrequencyHz = 2'000.0;
    config.solver.gasSubsteps = 1;
    config.solver.maximumCrankDegreesPerStep = 2.0;
    enginelab::normaliseEngineConfig(config);
    return config;
}

struct MotoredFixture final {
    enginelab::EngineConfig config { fixtureConfig() };
    enginelab::SimpleEcuModel ecu;
    enginelab::SimplifiedGasolinePhysics physics;
    enginelab::FourStrokeEventGenerator events;
    enginelab::ExhaustGraph exhaust { enginelab::ExhaustGraph::makeForEngine(config) };
    std::unique_ptr<enginelab::EngineSimulator> simulator;
    double extraInertiaKgM2 { 0.0 };
    double previousExternalTorqueNm { 0.0 };
    double scheduledAdvanceSeconds { 0.0 };
    std::size_t frameCount { 0 };

    explicit MotoredFixture(double extraInertia) : extraInertiaKgM2(extraInertia) {
        enginelab::EngineSimulatorOptions options;
        options.intakeJointManifold = false;
        options.intakeCouplingIntervalSeconds = requestedIntervalSeconds;
        simulator = std::make_unique<enginelab::EngineSimulator>(
            config, ecu, physics, events, exhaust, options);
    }

    [[nodiscard]] enginelab::EngineState advance(
        double targetRpm, double dtSeconds = frameSeconds) {
        const auto& before = simulator->state();
        const auto targetOmega = targetRpm * 2.0 * std::numbers::pi / 60.0;
        const auto inertia = enginelab::effectiveRotatingInertiaKgM2(config)
            + extraInertiaKgM2;
        // Signed motor/brake authority, compensating the last observed shaft
        // disturbance. Extra inertia limits firing-free compression ripple.
        const auto disturbanceTorqueNm = before.netTorqueNm - previousExternalTorqueNm;
        const auto externalTorqueNm = std::clamp(
            inertia * (targetOmega - before.angularVelocityRadPerSecond) / dtSeconds
                - disturbanceTorqueNm,
            -5'000.0, 5'000.0);
        enginelab::EngineControls controls;
        controls.ignitionEnabled = false;
        controls.starterEngaged = false;
        controls.throttle = 1.0;
        controls.externalTorqueNm = externalTorqueNm;
        controls.externalRotatingInertiaKgM2 = extraInertiaKgM2;
        const auto frame = simulator->step(dtSeconds, controls);
        previousExternalTorqueNm = externalTorqueNm;
        scheduledAdvanceSeconds += frame.state.intakeCouplingAdvancedSeconds;
        ++frameCount;
        require(std::isfinite(frame.state.rpm) && std::isfinite(lagSeconds()),
            "motored coupling telemetry must remain finite");
        return frame.state;
    }

    [[nodiscard]] double lagSeconds() const {
        return simulator->state().simulationTimeSeconds - scheduledAdvanceSeconds;
    }
};

struct CadenceResult final {
    double minimumRpm { std::numeric_limits<double>::infinity() };
    double maximumRpm { 0.0 };
    double minimumIntervalSeconds { std::numeric_limits<double>::infinity() };
    double maximumIntervalSeconds { 0.0 };
    std::size_t measuredFrames { 0 };
    std::size_t matchingGridFrames { 0 };
    std::size_t bracketingFrames { 0 };
    bool passes { false };
};

[[nodiscard]] CadenceResult measureCadence() {
    MotoredFixture fixture(2.0);
    for (std::size_t step = 0; step < 180; ++step)
        (void)fixture.advance(5'000.0);
    CadenceResult result;
    for (std::size_t step = 0; step < 120; ++step) {
        const auto state = fixture.advance(5'000.0);
        result.minimumRpm = std::min(result.minimumRpm, state.rpm);
        result.maximumRpm = std::max(result.maximumRpm, state.rpm);
        ++result.measuredFrames;
        if (state.solverSubsteps == 0 || state.solverResolutionLimited
                || std::abs(state.solverFrequencyHz
                    - static_cast<double>(state.solverSubsteps) / frameSeconds)
                    > 1.0e-7) continue;
        ++result.matchingGridFrames;
        if (state.intakeCouplingFlushCount == 0) continue;
        result.minimumIntervalSeconds = std::min(result.minimumIntervalSeconds,
            state.intakeCouplingMinimumIntervalSeconds);
        result.maximumIntervalSeconds = std::max(result.maximumIntervalSeconds,
            state.intakeCouplingMaximumIntervalSeconds);
        if (state.intakeCouplingMinimumIntervalSeconds < requestedIntervalSeconds
                && state.intakeCouplingMaximumIntervalSeconds
                    + elapsedTimeToleranceSeconds >= requestedIntervalSeconds)
            ++result.bracketingFrames;
    }
    require(result.matchingGridFrames == result.measuredFrames,
        "the adaptive solver must publish its actual valid mechanical cadence");
    require(result.minimumRpm > 4'960.0 && result.maximumRpm < 5'039.0,
        "the cadence fixture must hold the intended physical operating point");
    // The requested period does not generally land on the changing mechanical
    // grid. A clock retaining phase must bracket it; repeatedly rounding the
    // interval in one direction biases the long-run cadence. Forced valve-close
    // flushes may shorten an interval, but cannot justify that bias.
    result.passes = result.minimumIntervalSeconds < requestedIntervalSeconds
        && result.maximumIntervalSeconds + elapsedTimeToleranceSeconds
            >= requestedIntervalSeconds
        // Repeatedly rounding up would meet the maximum check but bias the
        // clock the other way. Normal frames must contain intervals on both
        // sides of the requested period; rare valve-close flushes are allowed.
        && result.bracketingFrames * 4 >= result.matchingGridFrames * 3;
    std::cout << "cadence target_rpm=5000 rpm_band=[" << result.minimumRpm << ','
              << result.maximumRpm << "] grid_frames=" << result.matchingGridFrames
              << " bracketing_frames=" << result.bracketingFrames
              << " interval_us=[" << result.minimumIntervalSeconds * 1.0e6 << ','
              << result.maximumIntervalSeconds * 1.0e6 << "] pass=" << result.passes << '\n';
    return result;
}

struct TransitionResult final {
    double maximumPendingBeforeSeconds { 0.0 };
    double maximumLowSpeedLagSeconds { 0.0 };
    double minimumLowRpm { std::numeric_limits<double>::infinity() };
    double maximumLowRpm { 0.0 };
    std::size_t attemptedTransitions { 0 };
    std::size_t lowSpeedFrames { 0 };
    bool passes { true };
};

[[nodiscard]] TransitionResult measureModeTransitions() {
    MotoredFixture fixture(0.35);
    TransitionResult result;
    for (std::size_t step = 0; step < 120; ++step)
        (void)fixture.advance(600.0);
    // A speed ramp can flush by coincidence before crossing the mode threshold.
    // Change only the caller's time grid instead: at 600 rpm a 250 us frame has
    // one mechanical substep and uses multirate coupling, while a 1/240 s frame
    // has nine ~463 us substeps at the gas minimum and couples on every substep.
    // Staying below the angular-cadence crossover prevents an adaptive short
    // step from legitimately re-entering multirate near the end of that frame.
    // RPM and physical configuration remain identical. The trigger is public pending
    // elapsed time, not a private accumulator or a chosen crank phase.
    constexpr double fastFrameSeconds = 1.0 / 4'000.0;
    for (std::size_t attempt = 0; attempt < 8; ++attempt) {
        auto armed = false;
        for (std::size_t step = 0; step < 40; ++step) {
            const auto state = fixture.advance(600.0, fastFrameSeconds);
            require(state.rpm > 500.0 && state.rpm < 700.0
                        && state.solverSubsteps == 1,
                "the fast caller grid must use the multirate intake regime");
            if (fixture.lagSeconds() > 100.0e-6) {
                result.maximumPendingBeforeSeconds = std::max(
                    result.maximumPendingBeforeSeconds, fixture.lagSeconds());
                armed = true;
                break;
            }
        }
        require(armed, "the time-grid transition must start with observable pending coupling time");
        ++result.attemptedTransitions;
        // Switching the next caller frame directly to the coarse grid leaves
        // no intervening multirate frame which could accidentally flush first.
        // All earlier pending duration must be committed on this transition.
        for (std::size_t step = 0; step < 6; ++step) {
            const auto state = fixture.advance(600.0);
            require(state.rpm > 500.0 && state.rpm < 700.0 && state.solverSubsteps == 9,
                "every transition observation must stay in the every-substep regime");
            result.minimumLowRpm = std::min(result.minimumLowRpm, state.rpm);
            result.maximumLowRpm = std::max(result.maximumLowRpm, state.rpm);
            const auto lag = std::abs(fixture.lagSeconds());
            result.maximumLowSpeedLagSeconds = std::max(result.maximumLowSpeedLagSeconds, lag);
            result.passes = result.passes && lag <= elapsedTimeToleranceSeconds;
            ++result.lowSpeedFrames;
        }
    }
    require(fixture.simulator->state().simulationTimeSeconds < 5.0,
        "the transition fixture must stay within its bounded simulation duration");
    std::cout << "time_grid_transitions fast_dt_us=" << fastFrameSeconds * 1.0e6
              << " slow_dt_us=" << frameSeconds * 1.0e6
              << " attempts=" << result.attemptedTransitions
              << " pending_before_max_us=" << result.maximumPendingBeforeSeconds * 1.0e6
              << " low_rpm_band=[" << result.minimumLowRpm << ',' << result.maximumLowRpm
              << "] low_frames=" << result.lowSpeedFrames
              << " low_lag_max_us=" << result.maximumLowSpeedLagSeconds * 1.0e6
              << " pass=" << result.passes << '\n';
    return result;
}
} // namespace

int main(int argc, char** argv) {
    auto diagnosticOnly = false;
    for (int index = 1; index < argc; ++index) {
        require(std::string_view(argv[index]) == "--diagnostic-only",
            "the only supported option is --diagnostic-only");
        diagnosticOnly = true;
    }
    std::cout << std::setprecision(10);
    const auto cadence = measureCadence();
    const auto transitions = measureModeTransitions();
    if (!diagnosticOnly) {
        require(cadence.passes, "intake coupling must retain the requested periodic cadence");
        require(transitions.passes,
            "leaving multirate intake must commit all pending history without losing time");
    }
    return EXIT_SUCCESS;
}
