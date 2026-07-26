#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <numbers>
#include <string>
#include <sstream>
#include <vector>

namespace {
struct Metrics final {
    double meanRpm {};
    double peakRpm {};
    double meanTorque {};
    double meanAfr {};
    double fuelGrams {};
    double peakPressureBar {};
    double audioRms {};
    double audioPeak {};
    double meanImepBar {};
    double peakRunnerResonanceHz {};
};

void writeU16(std::ofstream& stream, std::uint16_t value) {
    stream.put(static_cast<char>(value & 0xffU));
    stream.put(static_cast<char>((value >> 8U) & 0xffU));
}
void writeU32(std::ofstream& stream, std::uint32_t value) {
    writeU16(stream, static_cast<std::uint16_t>(value & 0xffffU));
    writeU16(stream, static_cast<std::uint16_t>(value >> 16U));
}
void writeWav(const std::filesystem::path& path, const std::vector<float>& samples, int sampleRate) {
    std::ofstream out(path, std::ios::binary);
    const auto dataBytes = static_cast<std::uint32_t>(samples.size() * sizeof(std::int16_t));
    out.write("RIFF", 4); writeU32(out, 36U + dataBytes); out.write("WAVEfmt ", 8);
    writeU32(out, 16U); writeU16(out, 1U); writeU16(out, 1U); writeU32(out, sampleRate);
    writeU32(out, static_cast<std::uint32_t>(sampleRate * 2)); writeU16(out, 2U); writeU16(out, 16U);
    out.write("data", 4); writeU32(out, dataBytes);
    for (const auto sample : samples) {
        const auto value = static_cast<std::int16_t>(std::lrint(std::clamp(sample, -1.0F, 1.0F) * 32767.0F));
        writeU16(out, static_cast<std::uint16_t>(value));
    }
}

enginelab::EngineConfig makeVTwin() {
    auto config = enginelab::makeDefaultInlineTwo();
    config.name = "EL comparison V-twin";
    config.layout = enginelab::EngineLayout::vLayout;
    config.bankAngleDegrees = 60.0;
    config.banks = { { 1, -30.0, { 1 }, config.camshafts, 1, 1 },
                     { 2, 30.0, { 2 }, config.camshafts, 1, 1 } };
    config.cylinders[0].bankId = 1; config.cylinders[0].bankOffsetDegrees = -30.0;
    config.cylinders[1].bankId = 2; config.cylinders[1].bankOffsetDegrees = 30.0;
    enginelab::normaliseEngineConfig(config);
    return config;
}

Metrics runScenario(enginelab::EngineConfig config, const std::filesystem::path& output) {
    constexpr double durationSeconds = 4.0;
    constexpr double dt = 1.0 / 240.0;
    constexpr int sampleRate = 48'000;
    enginelab::normaliseEngineConfig(config);
    enginelab::SimpleEcuModel ecu;
    enginelab::SimplifiedGasolinePhysics physics;
    enginelab::FourStrokeEventGenerator events;
    auto exhaust = enginelab::ExhaustGraph::makeForEngine(config);
    enginelab::EngineSimulator simulator(config, ecu, physics, events, exhaust);
    simulator.setPressureSamplingEnabled(true);
    std::ofstream trace(output / (config.name + ".csv"));
    trace << "time_s,rpm,crank_deg,torque_nm,power_kw,indicated_work_j,imep_bar,indicated_power_kw,manifold_kpa,exhaust_kpa,afr,lambda,fuel_g_s,heat_kw,egt_c,coolant_c,oil_c,gas_energy_j,cyl1_bar,cyl1_piston_mm,cyl1_runner_resonance_hz,cyl1_vvt_intake_deg,cyl1_lift_multiplier\n";
    trace << std::fixed << std::setprecision(6);
    std::vector<float> audio(static_cast<std::size_t>(durationSeconds * sampleRate), 0.0F);
    float previousChamberGauge = 0.0F;
    float previousExhaustGauge = 0.0F;
    Metrics metrics;
    std::size_t metricSamples = 0;
    for (std::size_t step = 0; step < static_cast<std::size_t>(durationSeconds / dt); ++step) {
        const auto time = static_cast<double>(step) * dt;
        enginelab::EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = time < 1.15;
        controls.throttle = time < 0.8 ? 0.18 : std::clamp(0.30 + (time - 0.8) * 0.16, 0.0, 0.82);
        controls.load = time < 1.4 ? 0.0 : 0.18;
        const auto frame = simulator.step(dt, controls);
        const auto& state = frame.state;
        const auto cylPressure = state.cylinderStateCount > 0 ? state.cylinderStates[0].pressureEstimateBar : 0.0;
        const auto cylPiston = state.cylinderStateCount > 0 ? state.cylinderStates[0].pistonTravelMm : 0.0;
        trace << state.simulationTimeSeconds << ',' << state.rpm << ',' << state.crankAngleDegrees << ','
              << state.cycleAveragedTorqueNm << ',' << state.cycleAveragedPowerKw << ','
              << state.indicatedWorkJoulesPerCycle << ',' << state.indicatedMeanEffectivePressureBar << ','
              << state.indicatedPowerKw << ','
              << state.manifoldPressureKpa << ',' << state.exhaustPressureKpa << ',' << state.airFuelRatio << ','
              << state.lambda << ',' << state.fuelFlowGramsPerSecond << ',' << state.resolvedHeatReleaseKw << ','
              << state.exhaustTemperatureC << ',' << state.coolantTemperatureC << ',' << state.oilTemperatureC << ','
              << state.gasInternalEnergyJoules << ',' << cylPressure << ',' << cylPiston << ','
              << (state.cylinderStateCount > 0 ? state.cylinderStates[0].intakeResonanceFrequencyHz : 0.0) << ','
              << (state.cylinderStateCount > 0 ? state.cylinderStates[0].intakeValveAdvanceDegrees : 0.0) << ','
              << (state.cylinderStateCount > 0 ? state.cylinderStates[0].valveLiftMultiplier : 1.0) << '\n';
        if (time >= 2.0) {
            metrics.meanRpm += state.rpm; metrics.meanTorque += state.cycleAveragedTorqueNm;
            metrics.meanAfr += state.airFuelRatio; metrics.peakRpm = std::max(metrics.peakRpm, state.rpm);
            metrics.meanImepBar += state.indicatedMeanEffectivePressureBar;
            if (state.cylinderStateCount > 0) metrics.peakRunnerResonanceHz = std::max(
                metrics.peakRunnerResonanceHz, state.cylinderStates[0].intakeResonanceFrequencyHz);
            metrics.peakPressureBar = std::max(metrics.peakPressureBar, cylPressure); ++metricSamples;
        }
        enginelab::CylinderPressureSample pressureSample;
        while (simulator.tryPopCylinderPressureSample(pressureSample)) {
            if (pressureSample.cylinderCount == 0) continue;
            float chamberGauge = 0.0F;
            float exhaustGauge = 0.0F;
            for (std::size_t index = 0; index < pressureSample.cylinderCount; ++index) {
                chamberGauge += pressureSample.pressureBar[index] - 1.01325F;
                exhaustGauge += (pressureSample.exhaustRunnerPressureKpa[index] - 101.325F) * 0.01F;
            }
            const auto normalisation = 1.0F / std::sqrt(static_cast<float>(pressureSample.cylinderCount));
            chamberGauge *= normalisation;
            exhaustGauge *= normalisation;
            const auto physicalSource = (chamberGauge - previousChamberGauge) * 0.018F
                + (exhaustGauge - previousExhaustGauge) * 0.060F + exhaustGauge * 0.002F;
            previousChamberGauge = chamberGauge;
            previousExhaustGauge = exhaustGauge;
            const auto target = static_cast<std::size_t>(std::clamp(
                pressureSample.timeSeconds * sampleRate, 0.0,
                static_cast<double>(audio.size() - 1)));
            audio[target] += std::clamp(physicalSource, -0.85F, 0.85F);
        }
    }
    const auto& finalState = simulator.state();
    metrics.fuelGrams = finalState.fuelConsumedGrams;
    if (metricSamples > 0) {
        metrics.meanRpm /= static_cast<double>(metricSamples);
        metrics.meanTorque /= static_cast<double>(metricSamples);
        metrics.meanAfr /= static_cast<double>(metricSamples);
        metrics.meanImepBar /= static_cast<double>(metricSamples);
    }
    double squareSum = 0.0;
    for (const auto sample : audio) { squareSum += sample * sample; metrics.audioPeak = std::max(metrics.audioPeak, std::abs(static_cast<double>(sample))); }
    metrics.audioRms = std::sqrt(squareSum / static_cast<double>(audio.size()));
    writeWav(output / (config.name + ".wav"), audio, sampleRate);
    return metrics;
}

std::map<std::string, Metrics> readReference(const std::filesystem::path& path) {
    std::map<std::string, Metrics> result;
    std::ifstream input(path);
    std::string line;
    std::getline(input, line);
    while (std::getline(input, line)) {
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream values(line);
        std::string name; Metrics metrics;
        values >> std::quoted(name) >> metrics.meanRpm >> metrics.peakRpm >> metrics.meanTorque >> metrics.meanAfr
               >> metrics.fuelGrams >> metrics.peakPressureBar >> metrics.audioRms >> metrics.audioPeak
               >> metrics.meanImepBar >> metrics.peakRunnerResonanceHz;
        if (values) result.emplace(name, metrics);
    }
    return result;
}

struct DynoSweepPoint final {
    double targetRpm {};
    double measuredRpm {};
    double torqueNm {};
    double powerKw {};
    bool stable {};
};

std::vector<DynoSweepPoint> runSteadyDyno(const enginelab::EngineConfig& config,
                                         const std::filesystem::path& output) {
    enginelab::SimpleEcuModel ecu;
    enginelab::SimplifiedGasolinePhysics physics;
    enginelab::FourStrokeEventGenerator events;
    auto exhaust = enginelab::ExhaustGraph::makeForEngine(config);
    enginelab::EngineSimulator simulator(config, ecu, physics, events, exhaust);
    constexpr double dt = 1.0 / 240.0;
    for (int step = 0; step < static_cast<int>(10.0 / dt); ++step) {
        enginelab::EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = step < static_cast<int>(1.50 / dt);
        controls.throttle = controls.starterEngaged ? 0.72 : 0.0;
        (void)simulator.step(dt, controls);
    }
    std::cout << "Steady dyno run-up: closed-throttle speed=" << simulator.state().rpm << " rpm\n";

    const auto displacementM3 = enginelab::engineDisplacementLitres(config) * 0.001;
    const auto controllerTorqueScaleNm = 2'500'000.0 * displacementM3
        / (4.0 * std::numbers::pi);
    const auto maximumBrakeTorqueNm = 10'000'000.0 * displacementM3
        / (4.0 * std::numbers::pi);
    const auto firstTargetRpm = std::max(2'000.0, config.idleRpm * 1.45);
    const auto finalTargetRpm = std::min(config.redlineRpm,
        config.ignition.revLimitRpm) * 0.95;
    std::vector<double> targets;
    for (auto target = firstTargetRpm; target < finalTargetRpm - 200.0; target += 1'000.0)
        targets.push_back(target);
    targets.push_back(finalTargetRpm);

    std::vector<DynoSweepPoint> points;
    double filteredRpm = simulator.state().rpm;
    double filteredAcceleration = 0.0;
    double feedForwardTorque = std::max(0.0, simulator.state().torqueNm);
    for (const auto targetRpm : targets) {
        double integralTorque = 0.0;
        double brakeTorque = 0.0;
        double rpmSum = 0.0;
        double torqueSum = 0.0;
        double powerSum = 0.0;
        std::size_t samples = 0;
        for (int step = 0; step < static_cast<int>(3.0 / dt); ++step) {
            const auto previousFilteredRpm = filteredRpm;
            filteredRpm += (simulator.state().rpm - filteredRpm)
                * (1.0 - std::exp(-dt * 3.0));
            const auto rawAcceleration = (filteredRpm - previousFilteredRpm) / dt;
            filteredAcceleration += (rawAcceleration - filteredAcceleration)
                * (1.0 - std::exp(-dt * 2.0));
            feedForwardTorque += (std::max(0.0, simulator.state().torqueNm) - feedForwardTorque)
                * (1.0 - std::exp(-dt * 5.0));
            const auto error = filteredRpm - targetRpm;
            if (simulator.state().rpm < targetRpm - 120.0 && error < 0.0) {
                integralTorque = 0.0;
                brakeTorque = 0.0;
            } else {
                const auto proportionalGain = controllerTorqueScaleNm / 500.0;
                const auto integralGain = controllerTorqueScaleNm / 1'200.0;
                const auto accelerationGain = controllerTorqueScaleNm / 6'000.0;
                const auto candidate = integralTorque + error * integralGain * dt;
                const auto requested = feedForwardTorque + candidate
                    + error * proportionalGain + filteredAcceleration * accelerationGain;
                const auto saturated = std::clamp(requested, 0.0, maximumBrakeTorqueNm);
                if (requested == saturated
                    || (requested < 0.0 && error > 0.0)
                    || (requested > maximumBrakeTorqueNm && error < 0.0))
                    integralTorque = candidate;
                integralTorque = std::clamp(integralTorque,
                    -maximumBrakeTorqueNm, maximumBrakeTorqueNm);
                brakeTorque = std::clamp(feedForwardTorque + integralTorque
                    + error * proportionalGain + filteredAcceleration * accelerationGain,
                    0.0, maximumBrakeTorqueNm);
            }
            enginelab::EngineControls controls;
            controls.ignitionEnabled = true;
            controls.starterEngaged = points.empty()
                && simulator.state().rpm < std::max(650.0, config.idleRpm * 0.82);
            controls.throttle = 1.0;
            controls.dynamometerTorqueNm = brakeTorque;
            const auto frame = simulator.step(dt, controls);
            if (step >= static_cast<int>(1.5 / dt)
                    && std::abs(filteredRpm - targetRpm) <= 100.0
                    && std::abs(filteredAcceleration) <= 180.0) {
                rpmSum += frame.state.rpm;
                torqueSum += frame.state.loadTorqueNm;
                powerSum += frame.state.loadTorqueNm
                    * frame.state.angularVelocityRadPerSecond / 1'000.0;
                ++samples;
            } else if (step >= static_cast<int>(1.5 / dt)) {
                rpmSum = 0.0;
                torqueSum = 0.0;
                powerSum = 0.0;
                samples = 0;
            }
        }
        DynoSweepPoint point;
        point.targetRpm = targetRpm;
        point.stable = samples >= static_cast<std::size_t>(0.25 / dt);
        if (samples > 0) {
            const auto divisor = static_cast<double>(samples);
            point.measuredRpm = rpmSum / divisor;
            point.torqueNm = torqueSum / divisor;
            point.powerKw = powerSum / divisor;
        } else {
            point.measuredRpm = simulator.state().rpm;
        }
        points.push_back(point);
    }

    std::ofstream curve(output / "catalog-dyno.csv");
    curve << "engine,target_rpm,measured_rpm,torque_nm,power_kw,stable\n";
    curve << std::setprecision(10);
    for (const auto& point : points)
        curve << std::quoted(config.name) << ',' << point.targetRpm << ',' << point.measuredRpm
              << ',' << point.torqueNm << ',' << point.powerKw << ',' << (point.stable ? 1 : 0) << '\n';
    return points;
}

/**
 * Result of starting an engine and then leaving it entirely alone.
 *
 * Everything else in this harness drives the engine: the dyno sweep holds
 * throttle at 1.0 against a brake, and the acceptance run holds 0.72 against a
 * load controller. Both keep the engine turning by construction, so neither can
 * observe whether an engine can hold its own idle. An engine that stalls the
 * moment it is released passes every one of those checks and is still useless.
 */
struct IdleStability final {
    double meanIdleRpm {};
    double minimumIdleRpm {};
    double maximumIdleRpm {};
    double finalRpm {};
    bool everStarted { false };
    bool stalled { false };
    // Engine state captured at the last moment the engine was still turning
    // above half its idle target. For an engine that dies this is the state on
    // the way down, which is what identifies the torque that killed it.
    double peakRpm {};
    double stallTimeSeconds {};
    double stallManifoldKpa {};
    double stallExhaustKpa {};
    double stallIndicatedTorqueNm {};
    double stallFrictionTorqueNm {};
    double stallPumpingTorqueNm {};
    double stallNetTorqueNm {};
    double stallAirFuelRatio {};
    double stallTrappedMassMg {};
    double stallRequestedFuelMg {};
    double stallDeliveredFuelMg {};
    double stallFuelDeliveryRatio {};
    double stallResidualFraction {};
};

/**
 * Start the engine on the starter, release it, and hold a closed throttle.
 *
 * No dynamometer torque, no clutch reaction, no brake: the only things acting on
 * the crank are the engine's own combustion, its friction and pumping work, and
 * whatever idle control the ECU applies. This is the weakest condition an engine
 * has to survive, and the one the application puts it in the moment it is
 * loaded, so it is the right place to assert idle stability.
 */
IdleStability runIdleStability(const enginelab::EngineConfig& config,
                               bool trace = false) {
    enginelab::SimpleEcuModel ecu;
    enginelab::SimplifiedGasolinePhysics physics;
    enginelab::FourStrokeEventGenerator events;
    auto exhaust = enginelab::ExhaustGraph::makeForEngine(config);
    auto simulator = std::make_unique<enginelab::EngineSimulator>(
        config, ecu, physics, events, exhaust);

    constexpr double dt = 1.0 / 240.0;
    constexpr double starterReleaseSeconds = 2.0;
    // The post-start air schedule decays over up to six seconds after catch, and
    // an engine running an elevated fast idle during that time is behaving
    // correctly, not failing to hold idle. Judge only after it has bled away, so
    // this measures governed idle rather than the start transient.
    constexpr double settleSeconds = 10.0;
    constexpr double totalSeconds = 16.0;
    // The engine is considered to have caught once it exceeds the speed a
    // starter alone can sustain.
    const auto startedRpm = std::max(400.0, config.idleRpm * 0.60);

    IdleStability result;
    result.minimumIdleRpm = std::numeric_limits<double>::max();
    double rpmSum = 0.0;
    std::size_t samples = 0;

    for (int step = 0; step < static_cast<int>(totalSeconds / dt); ++step) {
        const auto time = static_cast<double>(step) * dt;
        enginelab::EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = time < starterReleaseSeconds;
        controls.throttle = 0.0;
        const auto frame = simulator->step(dt, controls);
        const auto rpm = frame.state.rpm;
        if (rpm > startedRpm) result.everStarted = true;
        result.peakRpm = std::max(result.peakRpm, rpm);
        // Latch the state while the engine is still alive and unaided, so a
        // stall can be attributed rather than merely observed.
        if (!controls.starterEngaged && rpm > config.idleRpm * 0.50) {
            result.stallTimeSeconds = time;
            result.stallManifoldKpa = frame.state.manifoldPressureKpa;
            result.stallExhaustKpa = frame.state.exhaustPressureKpa;
            result.stallIndicatedTorqueNm = frame.state.indicatedTorqueNm;
            result.stallFrictionTorqueNm = frame.state.frictionTorqueNm;
            result.stallPumpingTorqueNm = frame.state.pdvTorqueNm;
            result.stallNetTorqueNm = frame.state.netTorqueNm;
            if (frame.state.cylinderStateCount > 0) {
                const auto& cylinder = frame.state.cylinderStates[0];
                result.stallAirFuelRatio = cylinder.airFuelRatio;
                result.stallTrappedMassMg = cylinder.trappedMassMg;
                result.stallRequestedFuelMg = cylinder.requestedFuelMgPerCycle;
                result.stallDeliveredFuelMg = cylinder.deliveredFuelMgPerCycle;
                result.stallFuelDeliveryRatio = cylinder.fuelDeliveryRatio;
                result.stallResidualFraction = cylinder.residualGasFraction;
            }
        }
        // Only judge the settled window, so the start transient and the
        // starter-release dip are not mistaken for an unstable idle.
        if (time >= settleSeconds) {
            rpmSum += rpm;
            ++samples;
            result.minimumIdleRpm = std::min(result.minimumIdleRpm, rpm);
            result.maximumIdleRpm = std::max(result.maximumIdleRpm, rpm);
        }
        result.finalRpm = rpm;
        // Time trace. Whether the mixture goes lean before the speed collapses
        // or after it is the difference between a fuelling cause and a fuelling
        // symptom, and a single sample at the moment of death cannot tell them
        // apart.
        if (trace && frame.state.cylinderStateCount > 0
                && step % static_cast<int>(0.10 / dt) == 0) {
            const auto& cylinder = frame.state.cylinderStates[0];
            std::cout << "    t=" << std::fixed << std::setprecision(2) << time
                      << " starter=" << (controls.starterEngaged ? 1 : 0)
                      << " rpm=" << std::setprecision(1) << rpm
                      << " MAP=" << frame.state.manifoldPressureKpa
                      << " AFR=" << std::setprecision(2) << cylinder.airFuelRatio
                      << " req/del=" << cylinder.requestedFuelMgPerCycle << '/'
                      << cylinder.deliveredFuelMgPerCycle
                      << " trapped=" << cylinder.trappedMassMg
                      << " cycTq=" << frame.state.cycleAveragedTorqueNm
                      << " fric=" << frame.state.frictionTorqueNm
                      << '\n';
        }
    }
    if (samples != 0) result.meanIdleRpm = rpmSum / static_cast<double>(samples);
    if (result.minimumIdleRpm == std::numeric_limits<double>::max())
        result.minimumIdleRpm = 0.0;
    // Stalled means it caught and then died, or never caught at all.
    result.stalled = !result.everStarted
        || result.finalRpm < config.idleRpm * 0.50;
    return result;
}

bool runCatalogAcceptance(const std::filesystem::path& root, const std::string& engineFilter,
                          const std::filesystem::path& output) {
    const auto catalog = enginelab::loadEngineCatalog(root);
    bool failed = !catalog.errors.empty() || catalog.entries.empty();
    bool matchedFilter = engineFilter.empty();
    for (const auto& error : catalog.errors) std::cerr << error << '\n';
    for (const auto& entry : catalog.entries) {
        auto config = entry.config;
        if (!engineFilter.empty() && config.name.find(engineFilter) == std::string::npos) continue;
        matchedFilter = true;
        enginelab::SimpleEcuModel ecu;
        enginelab::SimplifiedGasolinePhysics physics;
        enginelab::FourStrokeEventGenerator events;
        auto exhaust = enginelab::ExhaustGraph::makeForEngine(config);
        enginelab::EngineSimulator simulator(config, ecu, physics, events, exhaust);
        simulator.setPressureSamplingEnabled(!engineFilter.empty());
        constexpr double dt = 1.0 / 240.0;
        double peakRpm = 0.0;
        double peakBoost = 1.0;
        double peakShaftRpm = 0.0;
        double afrErrorSum = 0.0;
        double afrSum = 0.0;
        double targetAfrSum = 0.0;
        double deliveredFuelSum = 0.0;
        double injectedFuelSum = 0.0;
        double airMassSum = 0.0;
        double deliveryRatioSum = 0.0;
        double injectorCapacitySum = 0.0;
        double closedLoopTrimSum = 0.0;
        double fmepSum = 0.0;
        std::size_t settledSamples = 0;
        std::array<double, 32> sampledForwardMassKg {};
        std::array<double, 32> sampledReverseMassKg {};
        std::array<double, 32> sampledPeakForwardKgPerSecond {};
        std::array<double, 32> sampledPeakReverseKgPerSecond {};
        std::array<std::uint64_t, 32> sampledInvalidBoundaries {};
        double previousPressureSampleTime = 0.0;
        double dynoLoadIntegral = 0.0;
        const auto dynoTargetRpm = std::max(config.idleRpm * 1.55,
                                            config.ignition.revLimitRpm * 0.65);
        for (int step = 0; step < static_cast<int>(7.0 / dt); ++step) {
            const auto time = static_cast<double>(step) * dt;
            auto dynoLoad = 0.0;
            if (time >= 1.5) {
                const auto speedError = (simulator.state().rpm - dynoTargetRpm)
                    / std::max(1.0, dynoTargetRpm);
                dynoLoadIntegral = std::clamp(
                    dynoLoadIntegral + speedError * dt * 1.20, 0.0, 0.92);
                dynoLoad = std::clamp(dynoLoadIntegral + speedError * 0.70, 0.0, 1.0);
            }
            const auto frame = simulator.step(dt,
                { true, time < 1.5, 0.72, dynoLoad });
            const auto& state = frame.state;
            enginelab::CylinderPressureSample pressureSample;
            while (simulator.tryPopCylinderPressureSample(pressureSample)) {
                const auto sampleDt = previousPressureSampleTime > 0.0
                    ? std::clamp(pressureSample.timeSeconds - previousPressureSampleTime,
                                 0.0, 0.01)
                    : 0.0;
                previousPressureSampleTime = pressureSample.timeSeconds;
                if (pressureSample.timeSeconds < 4.0) continue;
                for (std::size_t cylinderIndex = 0;
                     cylinderIndex < pressureSample.cylinderCount; ++cylinderIndex) {
                    const auto massFlow = static_cast<double>(
                        pressureSample.exhaustMassFlowKgPerSecond[cylinderIndex]);
                    sampledForwardMassKg[cylinderIndex] += std::max(0.0, massFlow) * sampleDt;
                    sampledReverseMassKg[cylinderIndex] += std::min(0.0, massFlow) * sampleDt;
                    sampledPeakForwardKgPerSecond[cylinderIndex] = std::max(
                        sampledPeakForwardKgPerSecond[cylinderIndex], massFlow);
                    sampledPeakReverseKgPerSecond[cylinderIndex] = std::min(
                        sampledPeakReverseKgPerSecond[cylinderIndex], massFlow);
                    sampledInvalidBoundaries[cylinderIndex] += static_cast<std::uint64_t>(
                        pressureSample.thermoacousticBoundaryValid[cylinderIndex] == 0);
                }
            }
            peakRpm = std::max(peakRpm, state.rpm);
            peakBoost = std::max(peakBoost, state.boostPressureRatio);
            peakShaftRpm = std::max(peakShaftRpm, state.forcedInductionShaftSpeedRpm);
            const auto settledSpeedError = std::abs(state.rpm - dynoTargetRpm)
                / std::max(1.0, dynoTargetRpm);
            if (time >= 4.0 && settledSpeedError < 0.12) {
                // A compression-ignition engine is quality-governed: it has no
                // stoichiometric setpoint. What EngineSimulator publishes as its
                // `targetAirFuelRatio` is a smoke-limit FLOOR (stoichiometric *
                // 1.16), and its fuel is metered by the injected-quantity map
                // with the closed-loop trim explicitly disabled -- so whichever
                // of the two binds first, the delivered mixture is normally
                // LEANER than the floor. Measured on the 2.0 TDI at its rated
                // point: AFR 22.41 against a 16.99 floor, which is textbook
                // diesel and reproduces the manufacturer's 340 Nm / 110 kW.
                // A two-sided |actual - target| therefore measures nothing
                // physical on this engine and read as a 5.41 error.
                //
                // Assert the side that IS physical, and only that one: a diesel
                // must never run RICHER than its smoke limit. This is strictly
                // tighter than the old two-sided band on the rich side, where
                // the real failure (sooting past the smoke limit) lives.
                afrErrorSum += config.fuel == enginelab::FuelType::diesel
                    ? std::max(0.0, state.targetAirFuelRatio - state.airFuelRatio)
                    : std::abs(state.airFuelRatio - state.targetAirFuelRatio);
                afrSum += state.airFuelRatio;
                targetAfrSum += state.targetAirFuelRatio;
                deliveredFuelSum += state.deliveredFuelMgPerCycle;
                injectedFuelSum += state.injectedFuelMgPerCycle;
                airMassSum += state.airMassMgPerCycle;
                for (std::size_t cylinderIndex = 0; cylinderIndex < state.cylinderStateCount; ++cylinderIndex) {
                    deliveryRatioSum += state.cylinderStates[cylinderIndex].fuelDeliveryRatio;
                    injectorCapacitySum += state.cylinderStates[cylinderIndex].injectorCapacityRatio;
                    closedLoopTrimSum += state.cylinderStates[cylinderIndex].closedLoopFuelTrim;
                }
                fmepSum += state.frictionMeanEffectivePressureBar;
                ++settledSamples;
            }
        }
        const auto requiredRpm = config.redlineRpm > 3'500.0
            ? std::min(3'500.0, config.redlineRpm * 0.72)
            : config.idleRpm * 2.0;
        const auto meanAfrError = settledSamples > 0
            ? afrErrorSum / static_cast<double>(settledSamples) : 100.0;
        const auto meanFmep = settledSamples > 0
            ? fmepSum / static_cast<double>(settledSamples) : 100.0;
        bool engineFailed = !std::isfinite(peakRpm) || peakRpm < requiredRpm
            || settledSamples == 0 || meanAfrError > 2.5 || meanFmep > 4.0;
        if (config.forcedInduction.enabled
                && config.forcedInduction.type == enginelab::ForcedInductionType::turbocharger)
            engineFailed = engineFailed || peakBoost < 1.12 || peakShaftRpm < 25'000.0;
        std::cout << "Catalog gate: " << config.name << " peak=" << peakRpm
                  << " rpm, AFR=" << (settledSamples > 0 ? afrSum / static_cast<double>(settledSamples) : 0.0)
                  << "/" << (settledSamples > 0 ? targetAfrSum / static_cast<double>(settledSamples) : 0.0)
                  << ", delivered=" << (settledSamples > 0
                        ? deliveredFuelSum / static_cast<double>(settledSamples) : 0.0)
                  << " mg/cycle, injected=" << (settledSamples > 0
                        ? injectedFuelSum / static_cast<double>(settledSamples) : 0.0)
                  << " mg/cycle, air=" << (settledSamples > 0
                        ? airMassSum / static_cast<double>(settledSamples) : 0.0)
                  << " mg/cycle, delivery=" << (settledSamples > 0
                        ? deliveryRatioSum / (static_cast<double>(settledSamples)
                            * static_cast<double>(config.cylinders.size())) : 0.0)
                  << ", injectorCapacity=" << (settledSamples > 0
                        ? injectorCapacitySum / (static_cast<double>(settledSamples)
                            * static_cast<double>(config.cylinders.size())) : 0.0)
                  << ", trim=" << (settledSamples > 0
                        ? closedLoopTrimSum / (static_cast<double>(settledSamples)
                            * static_cast<double>(config.cylinders.size())) : 0.0)
                  << ", AFR error=" << meanAfrError << ", FMEP=" << meanFmep
                  << " bar, boost=" << peakBoost << '\n';
        if (!engineFilter.empty()) {
            const auto& state = simulator.state();
            std::cout << "  final state: rpm=" << state.rpm
                      << ", MAP=" << state.manifoldPressureKpa
                      << " kPa, exhaust=" << state.exhaustPressureKpa
                      << '/' << state.exhaustRunnerPressureKpa
                      << " kPa, exhaust flow=" << state.exhaustFlowGramsPerSecond
                      << " g/s, torque indicated/friction/net="
                      << state.indicatedTorqueNm << '/' << state.frictionTorqueNm
                      << '/' << state.netTorqueNm
                      << " Nm, cycle torque=" << state.cycleAveragedTorqueNm
                      << " Nm, IMEP=" << state.indicatedMeanEffectivePressureBar
                      << " bar, heat=" << state.resolvedHeatReleaseKw << " kW"
                      << ", solver=" << state.solverFrequencyHz
                      << " Hz" << (state.solverResolutionLimited ? " (limited)" : "")
                      << '\n';
            for (std::size_t cylinderIndex = 0; cylinderIndex < state.cylinderStateCount; ++cylinderIndex) {
                const auto& cylinder = state.cylinderStates[cylinderIndex];
                std::cout << "  cylinder " << cylinder.id << ": AFR=" << cylinder.airFuelRatio
                          << ", requested=" << cylinder.requestedFuelMgPerCycle
                           << " mg, delivered=" << cylinder.deliveredFuelMgPerCycle
                           << " mg, ratio=" << cylinder.fuelDeliveryRatio
                           << ", trim=" << cylinder.closedLoopFuelTrim
                           << ", gas=" << cylinder.trappedMassMg << " mg"
                           << ", residual=" << cylinder.residualGasFraction
                           << ", exhaust=" << cylinder.exhaustFlowMgPerCycle
                           << " mg/cycle @ " << cylinder.runnerPressureKpa << " kPa, "
                           << cylinder.exhaustVelocityMps << " m/s"
                           << ", sampled +/-="
                           << sampledForwardMassKg[cylinderIndex] * 1'000.0 << '/'
                           << sampledReverseMassKg[cylinderIndex] * 1'000.0 << " g"
                           << ", peak +/-="
                           << sampledPeakForwardKgPerSecond[cylinderIndex] * 1'000.0 << '/'
                           << sampledPeakReverseKgPerSecond[cylinderIndex] * 1'000.0 << " g/s"
                           << ", invalid=" << sampledInvalidBoundaries[cylinderIndex] << '\n';
            }
        }
        if (engineFailed) {
            std::cerr << "Catalog physics gate failed: " << config.name << '\n';
            failed = true;
        }

        // Idle stability, measured with the engine released rather than driven.
        const auto idle = runIdleStability(config, !engineFilter.empty());
        std::cout << "  idle " << config.name
                  << ": started=" << (idle.everStarted ? "yes" : "NO")
                  << " peak=" << idle.peakRpm
                  << " mean=" << idle.meanIdleRpm
                  << " min=" << idle.minimumIdleRpm
                  << " final=" << idle.finalRpm
                  << " target=" << config.idleRpm
                  << (idle.stalled ? "  [STALLED]" : "") << '\n';
        std::cout << "       last-alive t=" << idle.stallTimeSeconds
                  << "s MAP=" << idle.stallManifoldKpa
                  << " exh=" << idle.stallExhaustKpa
                  << " kPa torque ind/fric/pump/net="
                  << idle.stallIndicatedTorqueNm << '/'
                  << idle.stallFrictionTorqueNm << '/'
                  << idle.stallPumpingTorqueNm << '/'
                  << idle.stallNetTorqueNm
                  << " Nm AFR=" << idle.stallAirFuelRatio
                  << " trapped=" << idle.stallTrappedMassMg << " mg"
                  << " fuel req/del=" << idle.stallRequestedFuelMg << '/'
                  << idle.stallDeliveredFuelMg
                  << " ratio=" << idle.stallFuelDeliveryRatio
                  << " residual=" << idle.stallResidualFraction << '\n';
        if (!idle.everStarted) {
            std::cerr << "Idle gate failed: " << config.name
                      << " never started on the starter\n";
            failed = true;
        } else if (idle.stalled) {
            std::cerr << "Idle gate failed: " << config.name
                      << " stalled after the starter was released (final rpm "
                      << idle.finalRpm << ", idle target " << config.idleRpm << ")\n";
            failed = true;
        } else if (idle.meanIdleRpm < config.idleRpm * 0.70
                   || idle.meanIdleRpm > config.idleRpm * 1.45) {
            std::cerr << "Idle gate failed: " << config.name
                      << " does not hold its idle target (mean " << idle.meanIdleRpm
                      << " rpm, target " << config.idleRpm << ")\n";
            failed = true;
        }
        if (!engineFilter.empty()) {
            const auto curve = runSteadyDyno(config, output);
            const auto peakPower = std::max_element(curve.begin(), curve.end(),
                [](const DynoSweepPoint& left, const DynoSweepPoint& right) {
                    return left.powerKw < right.powerKw;
                });
            if (peakPower != curve.end())
                std::cout << "Steady dyno: peak power=" << peakPower->powerKw
                          << " kW at " << peakPower->measuredRpm
                          << " rpm (target " << peakPower->targetRpm << ")\n";
            if (curve.empty() || std::none_of(curve.begin(), curve.end(),
                    [](const DynoSweepPoint& point) { return point.stable; })) {
                std::cerr << "Steady dyno failed to acquire a stable point: " << config.name << '\n';
                failed = true;
            }
        }
    }
    if (!matchedFilter) {
        std::cerr << "No catalog engine matched filter: " << engineFilter << '\n';
        failed = true;
    }
    return !failed;
}
} // namespace

int main(int argc, char** argv) {
    std::filesystem::path output = "comparison-output";
    std::filesystem::path reference;
    std::filesystem::path catalogRoot = std::filesystem::current_path();
    std::string catalogEngineFilter;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--output" && index + 1 < argc) output = argv[++index];
        else if (argument == "--reference" && index + 1 < argc) reference = argv[++index];
        else if (argument == "--catalog-root" && index + 1 < argc) catalogRoot = argv[++index];
        else if (argument == "--catalog-engine" && index + 1 < argc) catalogEngineFilter = argv[++index];
        else { std::cerr << "usage: EngineLabComparisonHarness [--output dir] [--reference summary.csv] [--catalog-root dir] [--catalog-engine name-fragment]\n"; return 2; }
    }
    std::filesystem::create_directories(output);
    const std::array scenarios { enginelab::makeDefaultInlineFour(), enginelab::makeDefaultV8(),
                                 makeVTwin(), enginelab::makeDefaultRadialFive() };
    std::map<std::string, Metrics> results;
    bool deterministicFailed = false;
    for (const auto& scenario : scenarios) {
        std::cout << "Running deterministic scenario: " << scenario.name << '\n';
        const auto metrics = runScenario(scenario, output);
        if (!std::isfinite(metrics.audioRms) || !std::isfinite(metrics.audioPeak)
                || metrics.audioRms <= 1.0e-8 || metrics.audioPeak <= 1.0e-7) {
            std::cerr << "Pressure-derived WAV gate failed: " << scenario.name << '\n';
            deterministicFailed = true;
        }
        results.emplace(scenario.name, metrics);
    }
    std::ofstream summary(output / "summary.csv");
    summary << "scenario,mean_rpm,peak_rpm,mean_torque_nm,mean_afr,fuel_g,peak_pressure_bar,audio_rms,audio_peak,mean_imep_bar,peak_runner_resonance_hz\n";
    summary << std::setprecision(10);
    for (const auto& [name, metric] : results)
        summary << std::quoted(name) << ',' << metric.meanRpm << ',' << metric.peakRpm << ',' << metric.meanTorque << ','
                << metric.meanAfr << ',' << metric.fuelGrams << ',' << metric.peakPressureBar << ','
                << metric.audioRms << ',' << metric.audioPeak << ',' << metric.meanImepBar << ','
                << metric.peakRunnerResonanceHz << '\n';
    summary.close();
    if (!reference.empty()) {
        const auto baseline = readReference(reference);
        bool failed = false;
        for (const auto& [name, metric] : results) {
            const auto found = baseline.find(name);
            if (found == baseline.end()) { std::cerr << "Missing reference scenario: " << name << '\n'; failed = true; continue; }
            const auto exceeds = [](double value, double expected, double tolerance) {
                return std::abs(value - expected) > std::max(1.0e-6, std::abs(expected) * tolerance);
            };
            if (exceeds(metric.meanRpm, found->second.meanRpm, 0.05)
                || exceeds(metric.meanTorque, found->second.meanTorque, 0.10)
                || exceeds(metric.meanAfr, found->second.meanAfr, 0.05)
                || exceeds(metric.fuelGrams, found->second.fuelGrams, 0.10)
                || exceeds(metric.peakPressureBar, found->second.peakPressureBar, 0.10)
                || exceeds(metric.meanImepBar, found->second.meanImepBar, 0.10)
                || exceeds(metric.peakRunnerResonanceHz, found->second.peakRunnerResonanceHz, 0.02)) {
                std::cerr << "Regression threshold exceeded: " << name << '\n'; failed = true;
            }
        }
        if (failed) return 1;
    }
    const auto catalogAccepted = runCatalogAcceptance(catalogRoot, catalogEngineFilter, output);
    return !deterministicFailed && catalogAccepted ? 0 : 1;
}
