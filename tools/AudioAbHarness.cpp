// Offline A/B measurement harness for the ten catalogue engines.
//
// This is a *measurement-only* instrument. It renders the real realtime audio
// path (RealtimeEngineAudio) offline, deterministically, at the default
// `convolution` voicing (0.45), for every engine in engines/*.yaml, driving a
// governed idle -> redline sweep. For each engine it reports, per channel and
// per RPM setpoint: RMS, crest factor, DC offset, spectral band balance
// (low/mid/high), a brightness proxy, and the L/R correlation; plus a Schroeder
// RT60 of the exhaust chain. No voicing parameter is changed here.
//
// The metric math (scanSignal / analyseWindow / measureDecay) is deliberately
// identical to tools/AudioRenderHarness.cpp so the two instruments are directly
// comparable; the only new machinery is the catalogue loader and the multi-
// setpoint governed sweep in renderSweep().
//
// Output: a human-readable table on stdout and two CSVs (per-setpoint metrics
// and per-engine RT60) under --output, for transcription into the report.

#include <enginelab/audio/RealtimeEngineAudio.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <juce_audio_basics/juce_audio_basics.h>
#include <juce_dsp/juce_dsp.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <string>
#include <thread>
#include <vector>

namespace {
using namespace enginelab;

struct WavData { std::vector<float> samples; double sampleRate { 44'100.0 }; };

std::uint32_t readU32(const unsigned char* p) {
    return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8)
        | (static_cast<std::uint32_t>(p[2]) << 16) | (static_cast<std::uint32_t>(p[3]) << 24);
}
std::uint16_t readU16(const unsigned char* p) {
    return static_cast<std::uint16_t>(static_cast<std::uint16_t>(p[0]) | (static_cast<std::uint16_t>(p[1]) << 8));
}

bool loadWav(const std::filesystem::path& path, WavData& out) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;
    std::vector<unsigned char> bytes((std::istreambuf_iterator<char>(in)), {});
    if (bytes.size() < 44 || readU32(bytes.data()) != 0x46464952U /*RIFF*/) return false;
    std::size_t offset = 12;
    std::uint16_t channels = 1, bits = 16;
    std::uint32_t rate = 44'100;
    while (offset + 8 <= bytes.size()) {
        const auto id = readU32(&bytes[offset]);
        const auto size = readU32(&bytes[offset + 4]);
        const auto body = offset + 8;
        if (id == 0x20746d66U /*fmt */ && body + 16 <= bytes.size()) {
            channels = std::max<std::uint16_t>(1, readU16(&bytes[body + 2]));
            rate = readU32(&bytes[body + 4]);
            bits = readU16(&bytes[body + 14]);
        } else if (id == 0x61746164U /*data*/) {
            if (bits != 16) return false;
            const auto count = std::min<std::size_t>(size, bytes.size() - body) / 2;
            out.samples.reserve(count / channels);
            for (std::size_t i = 0; i + channels <= count; i += channels) {
                const auto s = static_cast<std::int16_t>(readU16(&bytes[body + i * 2]));
                out.samples.push_back(static_cast<float>(s) / 32768.0F);
            }
            out.sampleRate = rate;
            return !out.samples.empty();
        }
        offset = body + size + (size & 1U);
    }
    return false;
}

// --- metric structures and math (identical to AudioRenderHarness) ------------

struct SafetyScan final {
    bool finite { true };
    double peak {};
    double nearFullScaleFraction {};
    std::size_t longestFlatTop {};
};

struct WindowAnalysis final {
    double mean {};
    double rms {};
    double peak {};
    double crest {};
    double brightness {};
    double lowBandFraction {};
    double midBandFraction {};
    double highBandFraction {};
    std::array<double, 32> spectrum {};
};

SafetyScan scanSignal(const std::vector<float>& x, std::size_t begin, std::size_t end) {
    SafetyScan s;
    for (std::size_t i = begin; i < end && i < x.size(); ++i) {
        const auto v = static_cast<double>(x[i]);
        if (!std::isfinite(v)) { s.finite = false; continue; }
        s.peak = std::max(s.peak, std::abs(v));
    }
    std::size_t nearFullScaleSamples = 0, total = 0;
    for (std::size_t i = begin; i < end && i < x.size(); ++i, ++total)
        if (std::isfinite(x[i]) && std::abs(static_cast<double>(x[i])) >= 0.98) ++nearFullScaleSamples;
    s.nearFullScaleFraction = static_cast<double>(nearFullScaleSamples)
        / static_cast<double>(std::max<std::size_t>(1, total));
    return s;
}

WindowAnalysis analyseWindow(const std::vector<float>& x, std::size_t begin, std::size_t end,
                             double sampleRate) {
    WindowAnalysis m;
    end = std::min(end, x.size());
    if (begin >= end) return m;
    const auto n = end - begin;
    double sum = 0.0, sumSq = 0.0, diffSq = 0.0, peak = 0.0;
    for (std::size_t i = begin; i < end; ++i) {
        const auto v = static_cast<double>(x[i]);
        if (!std::isfinite(v)) continue;
        sum += v;
        sumSq += v * v;
        peak = std::max(peak, std::abs(v));
        if (i > begin) { const auto d = v - static_cast<double>(x[i - 1]); diffSq += d * d; }
    }
    m.mean = sum / static_cast<double>(std::max<std::size_t>(1, n));
    m.rms = std::sqrt(sumSq / std::max<std::size_t>(1, n));
    m.peak = peak;
    m.crest = m.rms > 1e-12 ? m.peak / m.rms : 0.0;
    m.brightness = sumSq > 1e-12 ? std::sqrt(diffSq / sumSq) : 0.0;

    constexpr int fftOrder = 15;
    constexpr std::size_t fftSize = std::size_t { 1 } << fftOrder;
    std::vector<float> fftData(fftSize * 2, 0.0F);
    const auto available = std::min(n, fftSize);
    const auto sourceBegin = end - available;
    const auto destinationBegin = fftSize - available;
    for (std::size_t i = 0; i < available; ++i) {
        const auto fftIndex = destinationBegin + i;
        const auto window = 0.5 - 0.5 * std::cos(2.0 * std::numbers::pi
            * static_cast<double>(fftIndex) / static_cast<double>(fftSize - 1));
        const auto value = static_cast<double>(x[sourceBegin + i]);
        fftData[fftIndex] = static_cast<float>(
            (std::isfinite(value) ? value - m.mean : 0.0) * window);
    }
    juce::dsp::FFT fft(fftOrder);
    fft.performFrequencyOnlyForwardTransform(fftData.data());

    constexpr double minimumFingerprintHz = 40.0;
    constexpr double maximumFingerprintHz = 8'000.0;
    const auto fingerprintOctaves = std::log(maximumFingerprintHz / minimumFingerprintHz);
    double lowPower = 0.0, midPower = 0.0, highPower = 0.0;
    for (std::size_t bin = 1; bin <= fftSize / 2; ++bin) {
        const auto frequency = static_cast<double>(bin) * sampleRate / static_cast<double>(fftSize);
        const auto magnitude = static_cast<double>(fftData[bin]);
        const auto power = magnitude * magnitude;
        if (frequency >= 20.0 && frequency <= 12'000.0) {
            if (frequency < 250.0) lowPower += power;
            else if (frequency < 4'000.0) midPower += power;
            else highPower += power;
        }
        if (frequency >= minimumFingerprintHz && frequency <= maximumFingerprintHz) {
            const auto position = std::log(frequency / minimumFingerprintHz) / fingerprintOctaves;
            const auto band = std::min(m.spectrum.size() - 1,
                static_cast<std::size_t>(position * static_cast<double>(m.spectrum.size())));
            m.spectrum[band] += power;
        }
    }
    const auto audiblePower = std::max(lowPower + midPower + highPower, 1.0e-18);
    m.lowBandFraction = lowPower / audiblePower;
    m.midBandFraction = midPower / audiblePower;
    m.highBandFraction = highPower / audiblePower;

    double fingerprintNorm = 0.0;
    for (auto& value : m.spectrum) { value = std::sqrt(value); fingerprintNorm += value * value; }
    fingerprintNorm = std::sqrt(std::max(fingerprintNorm, 1.0e-18));
    for (auto& value : m.spectrum) value /= fingerprintNorm;
    return m;
}

double normalisedCorrelation(const std::vector<float>& a, const std::vector<float>& b,
                             std::size_t begin, std::size_t end) {
    double dot = 0.0, normA = 0.0, normB = 0.0;
    end = std::min({ end, a.size(), b.size() });
    for (std::size_t i = begin; i < end; ++i) {
        const auto x = static_cast<double>(a[i]);
        const auto y = static_cast<double>(b[i]);
        if (!std::isfinite(x) || !std::isfinite(y)) continue;
        dot += x * y; normA += x * x; normB += y * y;
    }
    const auto denominator = std::sqrt(normA * normB);
    return denominator > 1.0e-18 ? dot / denominator : 1.0;
}

// --- exhaust decay (identical to AudioRenderHarness) -------------------------

struct DecayMeasurement final {
    double rt60Seconds { 0.0 };
    double peak { 0.0 };
    double tailFloorDb { 0.0 };
    bool finite { true };
    bool valid { false };
};

DecayMeasurement measureDecay(const std::vector<float>& x, double sampleRate) {
    DecayMeasurement result;
    if (x.empty() || !(sampleRate > 0.0)) return result;
    std::size_t peakIndex = 0;
    for (std::size_t i = 0; i < x.size(); ++i) {
        const auto v = static_cast<double>(x[i]);
        if (!std::isfinite(v)) { result.finite = false; continue; }
        if (std::abs(v) > result.peak) { result.peak = std::abs(v); peakIndex = i; }
    }
    if (!result.finite || !(result.peak > 1.0e-9)) return result;
    const auto n = x.size() - peakIndex;
    std::vector<double> energy(n, 0.0);
    double running = 0.0;
    for (std::size_t i = n; i-- > 0;) {
        const auto v = static_cast<double>(x[peakIndex + i]);
        running += std::isfinite(v) ? v * v : 0.0;
        energy[i] = running;
    }
    if (!(energy[0] > 0.0)) return result;
    const auto levelDb = [&](std::size_t i) {
        return 10.0 * std::log10(std::max(energy[i], 1.0e-300) / energy[0]);
    };
    result.tailFloorDb = levelDb(n - 1);
    std::size_t start = 0, end = 0;
    bool haveStart = false, haveEnd = false;
    for (std::size_t i = 0; i < n; ++i) {
        const auto db = levelDb(i);
        if (!haveStart && db <= -5.0) { start = i; haveStart = true; }
        if (haveStart && db <= -25.0) { end = i; haveEnd = true; break; }
    }
    if (!haveStart || !haveEnd || end <= start) return result;
    result.rt60Seconds = static_cast<double>(end - start) / sampleRate * 3.0;
    result.valid = true;
    return result;
}

DecayMeasurement measureExhaustDecay(const EngineConfig& baseConfig, const WavData& ir,
                                     int preset, double seconds, float convolutionMix) {
    auto config = baseConfig;
    normaliseEngineConfig(config);
    auto audioConfiguration = std::make_unique<EngineRuntime>(config);
    auto& audioState = audioConfiguration->audioState();

    constexpr double audioRate = 48'000.0;
    constexpr int samplesPerStep = 200;
    constexpr double telemetryRate = 4'000.0;
    auto eventQueuePtr = std::make_unique<FiringEventQueue>();
    auto pressureQueuePtr = std::make_unique<CylinderPressureQueue>();
    auto& eventQueue = *eventQueuePtr;
    auto& pressureQueue = *pressureQueuePtr;
    auto rendererPtr = std::make_unique<RealtimeEngineAudio>(
        eventQueue, audioState, &pressureQueue, &audioConfiguration->exhaustGraph(),
        &audioConfiguration->engineConfig());
    auto& renderer = *rendererPtr;
    if (!ir.samples.empty()) renderer.setImpulseResponse(ir.samples, ir.sampleRate, 0);
    renderer.prepare(audioRate, samplesPerStep);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    const auto ambientKpa = static_cast<float>(config.ambientPressureKpa);
    audioState.exhaustPreset.store(preset, std::memory_order_relaxed);
    audioState.combustionGain.store(0.0F);
    audioState.intakeGain.store(0.0F);
    audioState.mechanicalGain.store(0.0F);
    audioState.exhaustGain.store(1.0F);
    audioState.convolution.store(std::clamp(convolutionMix, 0.0F, 1.0F));
    audioState.volume.store(1.0F);
    audioState.timeScale.store(1.0F);
    audioState.rpm.store(0.0F);
    audioState.throttle.store(0.0F);
    audioState.load.store(0.0F);
    audioState.starter.store(0.0F);
    audioState.lowFrequencyNoise.store(0.0F);
    audioState.highFrequencyNoise.store(0.0F);
    audioState.exhaustFlowGramsPerSecond.store(0.0F);

    constexpr double pulseCentreSeconds = 0.050;
    constexpr double pulseWidthSeconds = 0.001;
    constexpr float pulseAmplitudeKpa = 60.0F;
    const auto pulseAt = [&](double t) {
        const auto offset = t - pulseCentreSeconds;
        if (std::abs(offset) >= pulseWidthSeconds * 0.5) return 0.0F;
        return pulseAmplitudeKpa * static_cast<float>(
            0.5 * (1.0 + std::cos(2.0 * std::numbers::pi * offset / pulseWidthSeconds)));
    };

    std::vector<float> tail;
    const auto steps = static_cast<std::size_t>(seconds * audioRate / samplesPerStep);
    tail.reserve(steps * samplesPerStep);
    juce::AudioBuffer<float> block(2, samplesPerStep);
    double nextPressureTime = 0.0;
    for (std::size_t step = 0; step < steps; ++step) {
        const auto blockEnd = static_cast<double>(step + 1) * samplesPerStep / audioRate;
        while (nextPressureTime < blockEnd + 0.010) {
            CylinderPressureSample sample;
            sample.timeSeconds = nextPressureTime;
            sample.cylinderCount = 1;
            sample.pressureBar[0] = ambientKpa * 0.01F;
            sample.exhaustRunnerPressureKpa[0] = ambientKpa + pulseAt(nextPressureTime);
            sample.exhaustFlowMgPerCycle[0] = 60.0F;
            sample.exhaustValveOpening[0] = 0.0F;
            sample.exhaustPathIndex[0] = 0;
            if (!pressureQueue.tryPush(sample)) break;
            nextPressureTime += 1.0 / telemetryRate;
        }
        audioState.producerTimeNanoseconds.store(
            static_cast<std::uint64_t>(blockEnd * 1.0e9), std::memory_order_release);
        block.clear();
        renderer.render(block, 0, samplesPerStep);
        for (int s = 0; s < samplesPerStep; ++s) tail.push_back(block.getSample(0, s));
    }
    return measureDecay(tail, audioRate);
}

// --- governed idle -> redline sweep -----------------------------------------

struct Setpoint final {
    std::string label;
    double seconds { 2.5 };
    double throttle { 0.9 };
    double targetRpm { 0.0 };  // 0 => natural idle (no governor)
};

struct SetpointResult final {
    std::string label {};
    double meanRpm {};
    WindowAnalysis left {};
    WindowAnalysis right {};
    SafetyScan leftScan {};
    SafetyScan rightScan {};
    double lrCorrelation { 1.0 };
};

struct SweepResult final {
    std::vector<SetpointResult> setpoints;
    std::uint64_t droppedEvents {};
    std::uint64_t droppedPressureSamples {};
    std::uint64_t lateEvents {};
    std::uint64_t stolenVoices {};
    bool finite { true };
};

SweepResult renderSweep(const EngineConfig& baseConfig, const WavData& ir,
                        const std::vector<Setpoint>& setpoints) {
    auto config = baseConfig;
    normaliseEngineConfig(config);
    SimpleEcuModel ecu; SimplifiedGasolinePhysics physics; FourStrokeEventGenerator events;
    auto exhaust = ExhaustGraph::makeForEngine(config);
    auto simulatorPtr = std::make_unique<EngineSimulator>(config, ecu, physics, events, exhaust);
    auto& simulator = *simulatorPtr;
    simulator.setPressureSamplingEnabled(true);

    auto eventQueuePtr = std::make_unique<FiringEventQueue>();
    auto pressureQueuePtr = std::make_unique<CylinderPressureQueue>();
    auto& eventQueue = *eventQueuePtr;
    auto& pressureQueue = *pressureQueuePtr;
    auto audioConfiguration = std::make_unique<EngineRuntime>(config);
    auto& audioState = audioConfiguration->audioState();
    // convolution stays at the runtime default (0.45) => default voicing.

    constexpr double audioRate = 48'000.0;
    constexpr double dt = 1.0 / 240.0;
    constexpr int samplesPerStep = 200; // 48000 / 240
    auto rendererPtr = std::make_unique<RealtimeEngineAudio>(
        eventQueue, audioState, &pressureQueue, &audioConfiguration->exhaustGraph(),
        &audioConfiguration->engineConfig());
    auto& renderer = *rendererPtr;
    if (!ir.samples.empty()) renderer.setImpulseResponse(ir.samples, ir.sampleRate, 0);
    renderer.prepare(audioRate, samplesPerStep);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // Timeline: a 1.1 s crank/start prologue, then each setpoint dwell in order.
    constexpr double crankSeconds = 1.1;
    struct Phase { std::string label; double begin; double end; double throttle; double target; bool governor; };
    std::vector<Phase> phases;
    double cursor = crankSeconds;
    for (const auto& sp : setpoints) {
        phases.push_back({ sp.label, cursor, cursor + sp.seconds, sp.throttle,
                           sp.targetRpm, sp.targetRpm > 1.0 });
        cursor += sp.seconds;
    }
    const auto totalSeconds = cursor;

    std::vector<float> audioLeft, audioRight;
    std::vector<double> rpmTrace;
    audioLeft.reserve(static_cast<std::size_t>(totalSeconds * audioRate));
    audioRight.reserve(static_cast<std::size_t>(totalSeconds * audioRate));
    juce::AudioBuffer<float> block(2, samplesPerStep);

    SweepResult out;
    double realtimeSeconds = 0.0;
    double loadIntegral = 0.0;
    const auto steps = static_cast<std::size_t>(totalSeconds / dt);
    for (std::size_t step = 0; step < steps; ++step) {
        const auto t = static_cast<double>(step) * dt;
        EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = t < crankSeconds;
        controls.throttle = 0.2;
        controls.load = 0.0;
        // Which phase are we in?
        const Phase* active = nullptr;
        for (const auto& ph : phases) if (t >= ph.begin && t < ph.end) { active = &ph; break; }
        if (active != nullptr) {
            controls.throttle = active->throttle;
            if (active->governor) {
                const auto speedError = (simulator.state().rpm - active->target)
                    / std::max(1.0, active->target);
                loadIntegral = std::clamp(loadIntegral + speedError * dt * 1.20, 0.0, 0.95);
                controls.load = std::clamp(loadIntegral + speedError * 0.70, 0.0, 1.0);
            } else {
                loadIntegral = 0.0;  // idle: let the ECU idle governor hold it
            }
        }

        auto frame = simulator.step(dt, controls);
        out.droppedEvents += frame.droppedFiringEventCount;
        out.droppedPressureSamples += frame.droppedCylinderPressureSampleCount;
        const auto simStart = frame.state.simulationTimeSeconds - dt;
        for (std::size_t i = 0; i < frame.firingEventCount; ++i) {
            const auto fraction = std::clamp((frame.firingEvents[i].timeSeconds - simStart) / dt, 0.0, 1.0);
            frame.firingEvents[i].timeSeconds = realtimeSeconds + fraction * dt;
            if (!eventQueue.tryPush(frame.firingEvents[i])) ++out.droppedEvents;
        }
        CylinderPressureSample ps;
        while (simulator.tryPopCylinderPressureSample(ps)) {
            const auto fraction = std::clamp((ps.timeSeconds - simStart) / dt, 0.0, 1.0);
            ps.timeSeconds = realtimeSeconds + fraction * dt;
            if (!pressureQueue.tryPush(ps)) ++out.droppedPressureSamples;
        }
        publishAudioFrame(audioState, frame.state,
            { false, controls.starterEngaged, controls.load, 1.0 });
        audioState.producerTimeNanoseconds.store(
            static_cast<std::uint64_t>(std::max(0.0, realtimeSeconds + dt) * 1.0e9),
            std::memory_order_release);

        block.clear();
        renderer.render(block, 0, samplesPerStep);
        for (int s = 0; s < samplesPerStep; ++s) {
            audioLeft.push_back(block.getSample(0, s));
            audioRight.push_back(block.getSample(1, s));
            rpmTrace.push_back(frame.state.rpm);
        }
        realtimeSeconds += dt;
    }

    out.finite = scanSignal(audioLeft, 0, audioLeft.size()).finite
        && scanSignal(audioRight, 0, audioRight.size()).finite;
    out.lateEvents = renderer.lateEventCount();
    out.stolenVoices = renderer.stolenVoiceCount();
    out.droppedEvents += renderer.droppedPendingEventCount();

    // Fingerprint the final window of each phase (<=0.8 s, and <= half the dwell).
    for (const auto& ph : phases) {
        const auto phaseEndSample = std::min(static_cast<std::size_t>(ph.end * audioRate), audioLeft.size());
        const auto dwell = ph.end - ph.begin;
        const auto windowSeconds = std::min(0.8, dwell * 0.5);
        const auto windowSamples = static_cast<std::size_t>(windowSeconds * audioRate);
        const auto windowBegin = phaseEndSample > windowSamples ? phaseEndSample - windowSamples : 0;
        SetpointResult r;
        r.label = ph.label;
        double rpmSum = 0.0; std::size_t rpmCount = 0;
        for (std::size_t i = windowBegin; i < phaseEndSample && i < rpmTrace.size(); ++i) { rpmSum += rpmTrace[i]; ++rpmCount; }
        r.meanRpm = rpmCount ? rpmSum / static_cast<double>(rpmCount) : 0.0;
        r.left = analyseWindow(audioLeft, windowBegin, phaseEndSample, audioRate);
        r.right = analyseWindow(audioRight, windowBegin, phaseEndSample, audioRate);
        r.leftScan = scanSignal(audioLeft, windowBegin, phaseEndSample);
        r.rightScan = scanSignal(audioRight, windowBegin, phaseEndSample);
        r.lrCorrelation = normalisedCorrelation(audioLeft, audioRight, windowBegin, phaseEndSample);
        out.setpoints.push_back(std::move(r));
    }
    return out;
}
} // namespace

int main(int argc, char** argv) {
    std::cout << std::unitbuf;
    std::filesystem::path root = ENGINELAB_CATALOG_ROOT;
    std::filesystem::path outDir = "audio-ab-output";
    std::filesystem::path irPath = root / "assets" / "ir" / "exhaust_default.wav";
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--output" && i + 1 < argc) outDir = argv[++i];
        else if (a == "--ir" && i + 1 < argc) irPath = argv[++i];
        else if (a == "--root" && i + 1 < argc) root = argv[++i];
    }
    std::filesystem::create_directories(outDir);

    WavData ir;
    if (loadWav(irPath, ir))
        std::cout << "Loaded IR: " << irPath.string() << " (" << ir.samples.size()
                  << " samples @ " << ir.sampleRate << " Hz)\n";
    else
        std::cout << "WARNING: could not load IR at " << irPath.string() << " (renderer fallback)\n";

    auto catalog = loadEngineCatalog(root);
    for (const auto& e : catalog.errors) std::cerr << "catalog: " << e << '\n';
    if (catalog.entries.empty()) { std::cerr << "No catalogue engines loaded.\n"; return 1; }
    std::cout << "Loaded " << catalog.entries.size() << " catalogue engines from "
              << (root / "engines").string() << "\n";
    std::cout << "Voicing: convolution=0.45 (default), exhaustPreset=street (default)\n\n";

    std::ofstream metricsCsv(outDir / "audio-ab-metrics.csv");
    metricsCsv << "engine,setpoint,channel,mean_rpm,rms,crest,dc,brightness,"
                  "low_pct,mid_pct,high_pct,peak,lr_corr,near_fs_frac,finite\n";
    std::ofstream decayCsv(outDir / "audio-ab-rt60.csv");
    decayCsv << "engine,preset,rt60_dry_s,rt60_full_s,peak_dry,peak_full,tail_floor_db,valid\n";

    // Dyno-style sweep. The engine free-revs to the limiter on throttle alone and
    // stalls under a heavy brake at near-closed throttle (there is no self-idle
    // loop in this offline path), so each operating point is held by a brake
    // governor under *partial* throttle: enough torque that the brake sets the
    // RPM without stalling. Targets are fractions of the rev range from a low
    // cruise (~0.25 x redline, the lowest reliably controllable point) up to near
    // redline -- a controllable low->redline sweep, not a literal curb idle.
    const std::vector<Setpoint> setpointTemplate = {
        { "low",  3.0, 0.55, 0.0 },   // targets filled per engine below
        { "mid",  3.0, 0.72, 0.0 },
        { "high", 3.0, 0.86, 0.0 },
        { "peak", 3.0, 0.97, 0.0 },
    };

    for (const auto& entry : catalog.entries) {
        const auto& config = entry.config;
        const auto idle = config.idleRpm;
        const auto redline = config.redlineRpm;
        auto setpoints = setpointTemplate;
        setpoints[0].targetRpm = std::max(idle * 1.4, redline * 0.25); // low cruise
        setpoints[1].targetRpm = redline * 0.45;                       // mid
        setpoints[2].targetRpm = redline * 0.68;                       // high
        setpoints[3].targetRpm = redline * 0.90;                       // peak

        std::cout << "=== " << config.name << "  (idle " << std::fixed << std::setprecision(0)
                  << idle << ", redline " << redline << " rpm) ===\n";
        const auto sweep = renderSweep(config, ir, setpoints);
        for (const auto& sp : sweep.setpoints) {
            const auto emit = [&](const char* ch, const WindowAnalysis& w, const SafetyScan& s) {
                metricsCsv << config.name << ',' << sp.label << ',' << ch << ','
                           << std::fixed << std::setprecision(0) << sp.meanRpm << ','
                           << std::setprecision(4) << w.rms << ','
                           << std::setprecision(2) << w.crest << ','
                           << std::setprecision(5) << std::showpos << w.mean << std::noshowpos << ','
                           << std::setprecision(3) << w.brightness << ','
                           << std::setprecision(1) << w.lowBandFraction * 100.0 << ','
                           << w.midBandFraction * 100.0 << ','
                           << w.highBandFraction * 100.0 << ','
                           << std::setprecision(4) << s.peak << ','
                           << std::setprecision(3) << sp.lrCorrelation << ','
                           << std::setprecision(5) << s.nearFullScaleFraction << ','
                           << (s.finite ? "yes" : "NO") << '\n';
            };
            emit("L", sp.left, sp.leftScan);
            emit("R", sp.right, sp.rightScan);
            std::cout << "  " << std::left << std::setw(6) << sp.label << std::right
                      << " rpm=" << std::fixed << std::setprecision(0) << std::setw(5) << sp.meanRpm
                      << " rms=" << std::setprecision(4) << sp.left.rms << '/' << sp.right.rms
                      << " crest=" << std::setprecision(2) << sp.left.crest
                      << " dc=" << std::setprecision(4) << std::showpos << sp.left.mean << std::noshowpos
                      << " bands(L)=" << std::setprecision(1) << sp.left.lowBandFraction * 100.0
                      << '/' << sp.left.midBandFraction * 100.0 << '/' << sp.left.highBandFraction * 100.0 << '%'
                      << " bright=" << std::setprecision(3) << sp.left.brightness
                      << " LRcorr=" << sp.lrCorrelation
                      << " peak=" << std::setprecision(3) << sp.leftScan.peak << '\n';
        }
        std::cout << "  drops(evt/press)=" << sweep.droppedEvents << '/' << sweep.droppedPressureSamples
                  << " late=" << sweep.lateEvents << " stolen=" << sweep.stolenVoices
                  << " finite=" << (sweep.finite ? "yes" : "NO") << '\n';

        // Exhaust decay at the default street preset (default voicing).
        const auto dry = measureExhaustDecay(config, ir, 0, 6.0, 0.0F);
        const auto wet = measureExhaustDecay(config, ir, 0, 6.0, 1.0F);
        decayCsv << config.name << ",street," << std::fixed << std::setprecision(3)
                 << dry.rt60Seconds << ',' << wet.rt60Seconds << ','
                 << std::setprecision(4) << dry.peak << ',' << wet.peak << ','
                 << std::setprecision(1) << wet.tailFloorDb << ','
                 << ((dry.valid && wet.valid) ? "yes" : "no") << '\n';
        std::cout << "  RT60 street: dry(noIR/FDN)=" << std::setprecision(3) << dry.rt60Seconds
                  << "s full=" << wet.rt60Seconds << "s tailFloor=" << std::setprecision(1)
                  << wet.tailFloorDb << "dB"
                  << (dry.valid && wet.valid ? "" : "  [UNMEASURABLE]") << "\n\n";
    }

    std::cout << "Wrote " << (outDir / "audio-ab-metrics.csv").string() << " and "
              << (outDir / "audio-ab-rt60.csv").string() << '\n';
    return 0;
}
