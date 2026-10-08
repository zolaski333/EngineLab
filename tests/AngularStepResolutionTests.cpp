// An outer caller frame must not freeze the mechanical time grid while the
// crank accelerates. Check actual angular travel, elapsed time and published
// cadence, with ordinary startup and a signed external motor.
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numbers>
#include <string_view>

namespace {
constexpr double frameSeconds = 1.0 / 240.0;

struct Result final {
    std::size_t angularViolationFrames { 0 };
    std::size_t faultFrames { 0 };
    std::size_t completedCycles { 0 };
    std::size_t validCycles { 0 };
    std::size_t changingSpeedFrames { 0 };
    double maximumCrankDegrees { 0.0 };
    double maximumCadenceErrorHz { 0.0 };
    double maximumTimeErrorSeconds { 0.0 };
    double maximumRpm { 0.0 };
    double maximumCycleIdentityError { 0.0 };
    bool finite { true };
};

Result measure(const enginelab::EngineConfig& config, std::size_t frames,
               bool motorDriven, bool rapidAcceleration) {
    enginelab::SimpleEcuModel ecu;
    enginelab::SimplifiedGasolinePhysics physics;
    enginelab::FourStrokeEventGenerator events;
    auto exhaust = enginelab::ExhaustGraph::makeForEngine(config);
    auto simulator = std::make_unique<enginelab::EngineSimulator>(
        config, ecu, physics, events, exhaust);
    constexpr double motorInertia = 0.10;
    const auto rotatingInertia = enginelab::effectiveRotatingInertiaKgM2(config)
        + motorInertia;
    auto previousMotorTorque = 0.0;
    Result result;
    for (std::size_t step = 0; step < frames; ++step) {
        const auto before = simulator->state();
        enginelab::EngineControls controls;
        controls.ignitionEnabled = !rapidAcceleration;
        controls.starterEngaged = !motorDriven && step < 600;
        controls.throttle = motorDriven ? 1.0 : 0.45;
        controls.load = motorDriven ? 0.0 : 0.05;
        if (motorDriven) {
            const auto targetRpm = rapidAcceleration
                ? (step % 60 < 30 ? 800.0 : 3'500.0) : 2'100.0;
            const auto disturbance = before.netTorqueNm - previousMotorTorque;
            controls.externalTorqueNm = std::clamp(rotatingInertia
                * (targetRpm * 2.0 * std::numbers::pi / 60.0
                    - before.angularVelocityRadPerSecond) / frameSeconds
                    - disturbance,
                -5'000.0, 5'000.0);
            controls.externalRotatingInertiaKgM2 = motorInertia;
        }
        const auto frame = simulator->step(frameSeconds, controls);
        previousMotorTorque = controls.externalTorqueNm;
        const auto& state = frame.state;
        result.finite = result.finite && std::isfinite(state.rpm)
            && std::isfinite(state.crankDegreesPerSolverStep)
            && std::isfinite(state.solverFrequencyHz);
        result.maximumCrankDegrees = std::max(result.maximumCrankDegrees,
            state.crankDegreesPerSolverStep);
        result.maximumRpm = std::max(result.maximumRpm, state.rpm);
        result.angularViolationFrames += state.crankDegreesPerSolverStep
            > config.solver.maximumCrankDegreesPerStep + 1.0e-9 ? 1 : 0;
        result.faultFrames += state.solverResolutionLimited ? 1 : 0;
        result.changingSpeedFrames += state.rpm
            > std::max(100.0, before.rpm) * 1.07 ? 1 : 0;
        result.maximumCadenceErrorHz = std::max(result.maximumCadenceErrorHz,
            std::abs(state.solverFrequencyHz
                - static_cast<double>(state.solverSubsteps) / frameSeconds));
        result.maximumTimeErrorSeconds = std::max(result.maximumTimeErrorSeconds,
            std::abs(state.simulationTimeSeconds
                - static_cast<double>(step + 1) * frameSeconds));
        for (std::size_t index = 0;
             index < frame.completedBrakeCycleSampleCount; ++index) {
            const auto& cycle = frame.completedBrakeCycleSamples[index];
            ++result.completedCycles;
            result.validCycles += cycle.numericallyValid ? 1 : 0;
            result.maximumCycleIdentityError = std::max({
                result.maximumCycleIdentityError,
                std::abs(cycle.integratedCrankRadians - 4.0 * std::numbers::pi),
                std::abs(cycle.meanTorqueNm * cycle.integratedCrankRadians
                    - cycle.brakeWorkJoules)
                    / std::max(1.0, std::abs(cycle.brakeWorkJoules)),
                std::abs(cycle.meanPowerKw * cycle.durationSeconds * 1'000.0
                    - cycle.brakeWorkJoules)
                    / std::max(1.0, std::abs(cycle.brakeWorkJoules)) });
        }
    }
    std::cout << "engine=" << config.name << " motor=" << motorDriven
        << " rapid_acceleration=" << rapidAcceleration << " frames=" << frames
        << " max_rpm=" << result.maximumRpm
        << " max_crank_degrees=" << result.maximumCrankDegrees
        << " configured_degrees=" << config.solver.maximumCrankDegreesPerStep
        << " angular_violation_frames=" << result.angularViolationFrames
        << " fault_frames=" << result.faultFrames
        << " changing_speed_frames=" << result.changingSpeedFrames
        << " completed_cycles=" << result.completedCycles
        << " valid_cycles=" << result.validCycles
        << " max_cadence_error_Hz=" << result.maximumCadenceErrorHz
        << " max_time_error_s=" << result.maximumTimeErrorSeconds
        << " max_cycle_identity_error=" << result.maximumCycleIdentityError
        << " finite=" << result.finite << '\n';
    return result;
}

bool passes(const Result& result) {
    return result.finite && result.angularViolationFrames == 0
        && result.completedCycles > 10 && result.validCycles > 10
        && result.maximumCadenceErrorHz < 1.0e-7
        && result.maximumTimeErrorSeconds < 1.0e-9
        && result.maximumCycleIdentityError < 1.0e-8;
}
} // namespace

int main(int argc, char** argv) {
    const auto diagnosticOnly = argc == 2
        && std::string_view(argv[1]) == "--diagnostic-only";
    if (argc > 1 && !diagnosticOnly) return EXIT_FAILURE;
    std::cout << std::setprecision(14);
    const auto catalog = enginelab::loadEngineCatalog(
        std::filesystem::path(ENGINELAB_CATALOG_ROOT));
    if (!catalog.errors.empty()) return EXIT_FAILURE;
    const auto rapid = measure(enginelab::makeDefaultInlineTwo(), 600, true, true);
    const auto startup = measure(enginelab::makeDefaultInlineFour(), 2'400, false, false);
    auto valid = passes(rapid) && passes(startup) && rapid.changingSpeedFrames > 10;
    for (const auto filename : { "11_yamaha_cp2_mt07_like.engine.yaml",
                                 "07_harley_v_twin_like.engine.yaml" }) {
        const auto entry = std::find_if(catalog.entries.begin(), catalog.entries.end(),
            [filename](const auto& candidate) {
                return candidate.sourcePath.filename().string() == filename;
            });
        if (entry == catalog.entries.end()) return EXIT_FAILURE;
        valid = passes(measure(entry->config, 14'400, true, false)) && valid;
    }
    if (!valid && !diagnosticOnly) {
        std::cerr << "FAILED: an in-budget accelerating solver must respect the "
            "declared crank-angle bound and publish its actual cadence\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
