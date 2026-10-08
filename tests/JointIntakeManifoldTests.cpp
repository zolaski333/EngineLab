// The production intake must exchange with moving cylinders on the actual
// mechanical cadence and book one finite plenum per path. Test real engine
// topology and an untuned, user-sized one-litre four-cylinder gas fixture;
// neither maximum torque nor a smoothed curve is an oracle.
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numbers>
#include <string_view>
#include <vector>

namespace {
void require(bool condition, std::string_view message) {
    if (condition) return;
    std::cerr << "FAILED: " << message << '\n';
    std::exit(EXIT_FAILURE);
}

[[nodiscard]] enginelab::EngineConfig catalogue(std::string_view selector) {
    const auto loaded = enginelab::loadEngineCatalog(ENGINELAB_CATALOG_ROOT);
    const auto selected = enginelab::selectSingleEngineCatalogEntry(loaded.entries, selector);
    require(loaded.errors.empty() && static_cast<bool>(selected), "each distinct catalogue engine must load");
    return selected.entry->config;
}

[[nodiscard]] enginelab::EngineConfig oneLitre() {
    auto config = enginelab::makeDefaultInlineFour();
    config.name = "Untuned one litre joint manifold fixture";
    constexpr auto boreM = 0.068;
    constexpr auto strokeM = 1.0e-3 / (4.0 * std::numbers::pi * boreM * boreM * 0.25);
    for (std::size_t index = 0; index < config.cylinders.size(); ++index) {
        auto& cylinder = config.cylinders[index];
        cylinder.boreMm = boreM * 1'000.0;
        cylinder.strokeMm = strokeM * 1'000.0;
        cylinder.intakeRunnerLengthMm = 270.0 + static_cast<double>(index) * 15.0;
    }
    for (auto& journal : config.crankJournals) journal.throwMm = strokeM * 500.0;
    enginelab::normaliseEngineConfig(config);
    const auto error = enginelab::validateEngineConfig(config);
    require(!error, error.value_or("the geometry-only user fixture must remain valid"));
    return config;
}

[[nodiscard]] enginelab::EngineConfig splitIntakes(enginelab::EngineConfig config,
    const std::vector<std::vector<std::uint32_t>>& membership) {
    config.intakePaths.clear();
    config.banks.clear();
    for (std::size_t pathIndex = 0; pathIndex < membership.size(); ++pathIndex) {
        enginelab::IntakePathConfig path;
        path.id = static_cast<std::uint32_t>(31 + pathIndex * 7);
        path.cylinderIds = membership[pathIndex];
        path.geometry = config.intake;
        path.geometry.plenumVolumeLitres = 1.1 + static_cast<double>(pathIndex) * 0.7;
        path.geometry.runnerLengthMm = 245.0 + static_cast<double>(pathIndex) * 65.0;
        config.intakePaths.push_back(path);
        enginelab::CylinderBankConfig bank;
        bank.id = static_cast<std::uint32_t>(pathIndex + 1);
        bank.cylinderIds = path.cylinderIds;
        bank.camshafts = config.camshafts;
        bank.intakeId = path.id;
        bank.exhaustPathId = config.exhaustPaths.front().id;
        for (auto& cylinder : config.cylinders) {
            if (std::find(path.cylinderIds.begin(), path.cylinderIds.end(), cylinder.id)
                == path.cylinderIds.end()) continue;
            cylinder.bankId = bank.id;
            cylinder.intakeRunnerLengthMm = path.geometry.runnerLengthMm
                + static_cast<double>(cylinder.id) * 3.0;
        }
        config.banks.push_back(bank);
    }
    enginelab::normaliseEngineConfig(config);
    const auto error = enginelab::validateEngineConfig(config);
    require(!error, error.value_or("the explicit intake topology must remain valid"));
    return config;
}

[[nodiscard]] enginelab::EngineConfig ninePaths() {
    auto config = oneLitre();
    config.name = "Nine physical intake paths with eight published audio paths";
    const auto originalCylinder = config.cylinders.front();
    config.cylinders.clear();
    config.firingOrder.clear();
    config.crankJournals.clear();
    config.banks.clear();
    config.intakePaths.clear();
    config.exhaustPaths.front().cylinderIds.clear();
    std::vector<std::vector<std::uint32_t>> membership;
    for (std::uint32_t id = 1; id <= 9; ++id) {
        auto cylinder = originalCylinder;
        cylinder.id = id;
        cylinder.crankOffsetDegrees = static_cast<double>(id - 1) * 80.0;
        cylinder.crankJournalId = 0;
        cylinder.bankId = 0;
        config.cylinders.push_back(cylinder);
        config.firingOrder.push_back(id);
        config.exhaustPaths.front().cylinderIds.push_back(id);
        membership.push_back({id});
    }
    return splitIntakes(std::move(config), membership);
}

[[nodiscard]] std::size_t cylinderPath(const enginelab::EngineConfig& config, std::uint32_t id) {
    const auto found = std::find_if(config.intakePaths.begin(), config.intakePaths.end(),
        [id](const auto& path) {
            return std::find(path.cylinderIds.begin(), path.cylinderIds.end(), id) != path.cylinderIds.end();
        });
    require(found != config.intakePaths.end(), "each cylinder must retain an authored intake path");
    return static_cast<std::size_t>(found - config.intakePaths.begin());
}

void verifyField(enginelab::EngineSimulator& simulator) {
    simulator.captureGasFieldNow();
    const auto& config = simulator.config();
    const auto& field = simulator.gasField();
    const auto& state = simulator.state();
    std::array<bool, 32> seenCylinder {};
    std::array<bool, 32> seenPath {};
    std::size_t runnerElements = 0;
    std::size_t plenumElements = 0;
    for (const auto& element : field.elements) {
        if (element.kind == enginelab::GasFieldElementKind::intakePlenum) {
            require(element.pathIndex < config.intakePaths.size() && !seenPath[element.pathIndex],
                "each physical path must have its own unique field plenum");
            seenPath[element.pathIndex] = true;
            require(element.sampleCount == 1 && std::isfinite(element.pressurePa[0])
                && element.pressurePa[0] > 0.0F, "each plenum must retain finite physical gas");
            ++plenumElements;
        }
        if (element.kind != enginelab::GasFieldElementKind::intakeRunner) continue;
        const auto found = std::find_if(config.cylinders.begin(), config.cylinders.end(),
            [&](const auto& cylinder) { return cylinder.id == element.cylinderId; });
        require(found != config.cylinders.end() && element.sampleCount >= 3,
            "every field element must sample its authored runner duct");
        const auto index = static_cast<std::size_t>(found - config.cylinders.begin());
        require(!seenCylinder[index] && element.pathIndex == cylinderPath(config, found->id),
            "noncontiguous cylinder membership must retain each runner's path and identity");
        seenCylinder[index] = true;
        const auto portPressurePa = state.cylinderStates[index].intakeRunnerChargePressureKpa * 1'000.0;
        require(std::abs(static_cast<double>(element.pressurePa[element.sampleCount - 1]) - portPressurePa)
            < 1.0 + std::abs(portPressurePa) * 1.0e-6,
            "reversed plenum-to-valve gas-field samples must match that cylinder's live valve-side state");
        ++runnerElements;
    }
    require(runnerElements == config.cylinders.size() && plenumElements == config.intakePaths.size(),
        "the field must retain every runner and every authored finite plenum");
}

void measure(enginelab::EngineConfig config, bool legacyControl = false, bool liveSwap = false) {
    constexpr auto frameSeconds = 1.0 / 240.0;
    constexpr auto targetRpm = 5'000.0;
    constexpr auto extraInertia = 2.0;
    enginelab::SimpleEcuModel ecu;
    enginelab::SimplifiedGasolinePhysics physics;
    enginelab::FourStrokeEventGenerator events;
    auto exhaust = enginelab::ExhaustGraph::makeForEngine(config);
    enginelab::EngineSimulatorOptions options;
    options.intakeJointManifold = !legacyControl;
    auto simulator = std::make_unique<enginelab::EngineSimulator>(config, ecu, physics, events, exhaust, options);
    simulator->setPressureSamplingEnabled(true);
    require(simulator->intakeWorkerCount() == 0, "a shared RK reservoir must not dispatch independent runner advances");
    const auto& normalized = simulator->config();
    const auto pathCount = std::max<std::size_t>(1, normalized.intakePaths.size());
    const auto inertia = enginelab::effectiveRotatingInertiaKgM2(config) + extraInertia;
    auto previousTorque = 0.0;
    auto totalObservedMassMg = 0.0;
    std::size_t observedFrames = 0;
    std::size_t finiteCycleSamples = 0;
    std::array<double, 32> absoluteTransferredMassKg {};
    std::size_t pressureSamples = 0;
    const auto stepMotor = [&] {
        const auto& before = simulator->state();
        const auto disturbance = before.netTorqueNm - previousTorque;
        enginelab::EngineControls controls;
        controls.ignitionEnabled = false;
        controls.throttle = 1.0;
        controls.externalTorqueNm = std::clamp(inertia
            * (targetRpm * 2.0 * std::numbers::pi / 60.0 - before.angularVelocityRadPerSecond)
                / frameSeconds - disturbance,
            -5'000.0, 5'000.0);
        controls.externalRotatingInertiaKgM2 = extraInertia;
        auto frame = simulator->step(frameSeconds, controls);
        previousTorque = controls.externalTorqueNm;
        enginelab::CylinderPressureSample sample;
        while (simulator->tryPopCylinderPressureSample(sample)) {
            ++pressureSamples;
            require(sample.intakePathCount == std::min(pathCount, sample.intakeThrottleConductanceAreaM2.size()),
                "physical topology must not overflow the bounded published intake conductance array");
            for (std::size_t path = 0; path < sample.intakePathCount; ++path)
                require(std::isfinite(sample.intakeThrottleConductanceAreaM2[path])
                    && sample.intakeThrottleConductanceAreaM2[path] > 0.0F,
                    "every published throttle must retain its positive authored conductance");
            for (std::size_t index = 0; index < config.cylinders.size(); ++index) {
                require(sample.intakePathIndex[index] == cylinderPath(config, config.cylinders[index].id),
                    "pressure sampling must preserve physical cylinder-to-path mapping beyond the audio cap");
                absoluteTransferredMassKg[index] += std::abs(static_cast<double>(sample.intakeMassFlowKgPerSecond[index]))
                    / static_cast<double>(sample.mechanicalSamplingFrequencyHz);
            }
        }
        return frame;
    };
    const auto checkTimely = [&](const enginelab::EngineState& state) {
        require(state.intakeCouplingFlushCount == state.solverSubsteps,
            "production must refresh the intake at every actual mechanical step");
        require(std::abs(state.intakeCouplingAdvancedSeconds - frameSeconds) < 1.0e-11,
            "production cannot hold or lose a physical intake interval");
        require(std::abs(state.intakeNetworkAdvancedSeconds - static_cast<double>(pathCount) * frameSeconds) < 1.0e-11,
            "FV time must be accounted once per shared manifold, not once per runner");
        require(state.intakeNetworkAcceptedSubsteps >= pathCount * 2 * state.solverSubsteps,
            "each shared manifold must actually complete both mechanical half-advances");
        require(state.intakeCylinderTransferFailures == 0 && state.intakePlenumTransferFailures == 0,
            "each real cylinder and finite plenum must accept its conservative transfer");
    };
    for (std::size_t step = 0; step < 300; ++step) {
        const auto frame = stepMotor();
        if (step < 180) continue;
        ++observedFrames;
        const auto& state = frame.state;
        require(std::isfinite(state.rpm) && state.rpm > targetRpm * 0.98 && state.rpm < targetRpm * 1.02,
            "the motor must hold the prescribed nonzero physical operating point");
        checkTimely(state);
        for (std::size_t index = 0; index < state.cylinderStateCount; ++index) {
            require(state.cylinderStates[index].id == config.cylinders[index].id,
                "published cylinders must retain their authored identities");
            totalObservedMassMg += state.cylinderStates[index].trappedFreshAirMassMg;
        }
        for (std::size_t index = 0; index < frame.completedBrakeCycleSampleCount; ++index) {
            const auto& cycle = frame.completedBrakeCycleSamples[index];
            require(std::isfinite(cycle.brakeWorkJoules) && std::isfinite(cycle.meanTorqueNm),
                "motored cycle work must remain finite");
            require(std::abs(cycle.meanTorqueNm * cycle.integratedCrankRadians - cycle.brakeWorkJoules)
                < 1.0e-8 * std::max(1.0, std::abs(cycle.brakeWorkJoules)),
                "reported cycle torque must retain the actual work identity");
            ++finiteCycleSamples;
        }
    }
    require(observedFrames == 120 && finiteCycleSamples > 10 && totalObservedMassMg > 1'000.0,
        "the checks must observe many real valve exchanges and completed cycles");
    require(pressureSamples > 1'000, "the pressure-stream capacity check must observe real mechanical samples");
    for (std::size_t index = 0; index < config.cylinders.size(); ++index)
        require(absoluteTransferredMassKg[index] > 1.0e-3,
            "every physical runner, including paths above eight, must carry nonzero actual valve exchange");
    verifyField(*simulator);
    const auto verifyProbes = [&] {
        for (std::size_t elementIndex = 0; elementIndex < simulator->gasField().elements.size(); ++elementIndex) {
            const auto element = simulator->gasField().elements[elementIndex];
            if (element.kind != enginelab::GasFieldElementKind::intakeRunner
                && element.kind != enginelab::GasFieldElementKind::intakePlenum) continue;
            const auto sample = static_cast<std::uint8_t>(element.sampleCount - 1);
            simulator->probeGasField(static_cast<std::int32_t>(elementIndex), sample);
            checkTimely(stepMotor().state);
            verifyField(*simulator);
            const auto& probe = simulator->gasProbe();
            require(probe.latestBin >= 0 && probe.sequence > 0 && probe.element == static_cast<std::int32_t>(elementIndex),
                "every authored runner and plenum must produce actual probe samples");
            require(probe.pressurePa[static_cast<std::size_t>(probe.latestBin)]
                == simulator->gasField().elements[elementIndex].pressurePa[sample],
                "the live probe must read the same mapped physical duct or plenum as the field");
        }
        simulator->stopGasProbe();
    };
    verifyProbes();
    if (liveSwap) {
        for (std::size_t path = 0; path < config.intakePaths.size(); ++path) {
            config.intakePaths[path].geometry.runnerLengthMm += 40.0 + static_cast<double>(path) * 15.0;
            config.intakePaths[path].geometry.plenumVolumeLitres *= 1.2 + static_cast<double>(path) * 0.1;
        }
        for (auto& cylinder : config.cylinders) cylinder.intakeRunnerLengthMm += 45.0;
        enginelab::normaliseEngineConfig(config);
        require(!enginelab::validateEngineConfig(config), "the unequal live geometry edit must remain valid");
        require(enginelab::EngineSimulator::intakeReplaceable(simulator->config(), config),
            "geometry on noncontiguous intake paths must remain live-replaceable");
        auto built = simulator->buildLiveIntake(config);
        require(built != nullptr, "all replacement joint manifolds must compile");
        auto incoming = config;
        require(simulator->replaceIntake(*built, incoming), "all shared manifolds must adopt and swap their gas state");
        for (std::size_t step = 0; step < 120; ++step) checkTimely(stepMotor().state);
        verifyField(*simulator);
        verifyProbes();
    }
    std::cout << "engine=" << config.name << " cylinders=" << config.cylinders.size()
        << " intake_paths=" << pathCount << " observed_frames=" << observedFrames
        << " trapped_mass_sum_mg=" << totalObservedMassMg << " completed_cycles=" << finiteCycleSamples
        << " workers=" << simulator->intakeWorkerCount() << " pressure_samples=" << pressureSamples
        << " probed_physical_paths=" << pathCount << " live_swap=" << liveSwap << " joint_timely=true" << '\n';
}
} // namespace

int main(int argc, char** argv) {
    std::cout << std::setprecision(12);
    const auto legacyControl = argc > 1 && std::string_view(argv[1]) == "--legacy-control";
    if (argc > 1 && std::string_view(argv[1]) == "--nine-path-only") {
        measure(ninePaths());
        return EXIT_SUCCESS;
    }
    measure(catalogue("11_yamaha_cp2_mt07_like"), legacyControl);
    measure(catalogue("01_honda_k20a_like"));
    measure(catalogue("03_gm_ls3_like"));
    measure(oneLitre());
    auto split = splitIntakes(oneLitre(), {{1, 3}, {2, 4}});
    split.name = "Two unequal intake paths with noncontiguous cylinders and live replacement";
    measure(std::move(split), false, true);
    measure(ninePaths());
    return EXIT_SUCCESS;
}
