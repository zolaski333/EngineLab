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

bool runCatalogAcceptance(const std::filesystem::path& root, const std::string& engineFilter) {
    const auto catalog = enginelab::loadEngineCatalog(root);
    bool failed = !catalog.errors.empty() || catalog.entries.empty();
    for (const auto& error : catalog.errors) std::cerr << error << '\n';
    for (const auto& entry : catalog.entries) {
        auto config = entry.config;
        if (!engineFilter.empty() && config.name.find(engineFilter) == std::string::npos) continue;
        enginelab::SimpleEcuModel ecu;
        enginelab::SimplifiedGasolinePhysics physics;
        enginelab::FourStrokeEventGenerator events;
        auto exhaust = enginelab::ExhaustGraph::makeForEngine(config);
        enginelab::EngineSimulator simulator(config, ecu, physics, events, exhaust);
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
        double closedLoopTrimSum = 0.0;
        double fmepSum = 0.0;
        std::size_t settledSamples = 0;
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
            peakRpm = std::max(peakRpm, state.rpm);
            peakBoost = std::max(peakBoost, state.boostPressureRatio);
            peakShaftRpm = std::max(peakShaftRpm, state.forcedInductionShaftSpeedRpm);
            const auto settledSpeedError = std::abs(state.rpm - dynoTargetRpm)
                / std::max(1.0, dynoTargetRpm);
            if (time >= 4.0 && settledSpeedError < 0.12) {
                afrErrorSum += std::abs(state.airFuelRatio - state.targetAirFuelRatio);
                afrSum += state.airFuelRatio;
                targetAfrSum += state.targetAirFuelRatio;
                deliveredFuelSum += state.deliveredFuelMgPerCycle;
                injectedFuelSum += state.injectedFuelMgPerCycle;
                airMassSum += state.airMassMgPerCycle;
                for (std::size_t cylinderIndex = 0; cylinderIndex < state.cylinderStateCount; ++cylinderIndex) {
                    deliveryRatioSum += state.cylinderStates[cylinderIndex].fuelDeliveryRatio;
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
                  << ", trim=" << (settledSamples > 0
                        ? closedLoopTrimSum / (static_cast<double>(settledSamples)
                            * static_cast<double>(config.cylinders.size())) : 0.0)
                  << ", AFR error=" << meanAfrError << ", FMEP=" << meanFmep
                  << " bar, boost=" << peakBoost << '\n';
        if (!engineFilter.empty()) {
            const auto& state = simulator.state();
            for (std::size_t cylinderIndex = 0; cylinderIndex < state.cylinderStateCount; ++cylinderIndex) {
                const auto& cylinder = state.cylinderStates[cylinderIndex];
                std::cout << "  cylinder " << cylinder.id << ": AFR=" << cylinder.airFuelRatio
                          << ", requested=" << cylinder.requestedFuelMgPerCycle
                          << " mg, delivered=" << cylinder.deliveredFuelMgPerCycle
                          << " mg, ratio=" << cylinder.fuelDeliveryRatio
                          << ", trim=" << cylinder.closedLoopFuelTrim << '\n';
            }
        }
        if (engineFailed) {
            std::cerr << "Catalog physics gate failed: " << config.name << '\n';
            failed = true;
        }
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
    const auto catalogAccepted = runCatalogAcceptance(catalogRoot, catalogEngineFilter);
    return !deterministicFailed && catalogAccepted ? 0 : 1;
}
