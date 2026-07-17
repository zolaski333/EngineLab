// Offline driver for the real realtime audio path (RealtimeEngineAudio) so the
// exhaust/sound changes can be verified without a sound device: it runs the
// physics simulator, feeds the firing-event and cylinder-pressure queues exactly
// like EngineRuntime does, renders through the real convolution IR, and reports
// objective metrics (level, crest factor, spectral fingerprint, stability).
//
// Two kinds of measurement live here:
//  - renderEngine(): steady-state fingerprint and safety gates for a running
//    engine. Safety gates (finiteness, peak, clipping plateaus) scan the whole
//    rendered signal; the spectral/level fingerprint uses the final steady-state
//    window only. Both channels are analysed.
//  - measureExhaustDecay(): drives a single exhaust impulse through a quiet
//    renderer and reports the reverberation time of the tailpipe chain. This is
//    the instrument for calibrating the muffler FDN and its presets against a
//    measured RT60 instead of by ear.

#include <enginelab/audio/RealtimeEngineAudio.hpp>
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

// Minimal 16-bit PCM WAV reader (the es2d impulse responses are mono PCM16).
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

void writeWav(const std::filesystem::path& path, const std::vector<float>& left,
              const std::vector<float>& right, int sampleRate) {
    std::ofstream out(path, std::ios::binary);
    const auto frames = std::min(left.size(), right.size());
    const auto dataBytes = static_cast<std::uint32_t>(frames * 2 * sizeof(std::int16_t));
    const auto put32 = [&](std::uint32_t v) { for (int i = 0; i < 4; ++i) out.put(static_cast<char>((v >> (8 * i)) & 0xffU)); };
    const auto put16 = [&](std::uint16_t v) { for (int i = 0; i < 2; ++i) out.put(static_cast<char>((v >> (8 * i)) & 0xffU)); };
    out.write("RIFF", 4); put32(36U + dataBytes); out.write("WAVEfmt ", 8);
    put32(16U); put16(1U); put16(2U); put32(static_cast<std::uint32_t>(sampleRate));
    put32(static_cast<std::uint32_t>(sampleRate * 4)); put16(4U); put16(16U);
    out.write("data", 4); put32(dataBytes);
    const auto encode = [&](float s) {
        put16(static_cast<std::uint16_t>(static_cast<std::int16_t>(
            std::lrint(std::clamp(s, -1.0F, 1.0F) * 32767.0F))));
    };
    for (std::size_t i = 0; i < frames; ++i) { encode(left[i]); encode(right[i]); }
}

// Safety scan over the complete rendered signal. These are the properties that
// must hold at every instant, so restricting them to an analysis window would
// defeat their purpose: a divergence at t=12s of a 25s stability run has to fail.
struct SafetyScan final {
    bool finite { true };
    double peak {};
    double nearFullScaleFraction {};
    std::size_t longestFlatTop {};
};

// Steady-state fingerprint over the final analysis window.
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

struct ChannelMetrics final {
    SafetyScan scan {};
    WindowAnalysis window {};
};

struct Metrics {
    ChannelMetrics left {};
    ChannelMetrics right {};
    // 1.0 means the two channels are identical: a stereo renderer that silently
    // collapses to mono is a regression the old single-channel harness could not see.
    double channelCorrelation { 1.0 };
    double finalRpm {};
    std::uint64_t droppedEvents {};
    std::uint64_t droppedPressureSamples {};
    std::uint64_t lateEvents {};
    std::uint64_t stolenVoices {};
    std::uint64_t delayTruncations {};
};

SafetyScan scanSignal(const std::vector<float>& x) {
    SafetyScan s;
    for (const auto value : x) {
        const auto v = static_cast<double>(value);
        if (!std::isfinite(v)) { s.finite = false; continue; }
        s.peak = std::max(s.peak, std::abs(v));
    }
    std::size_t nearFullScaleSamples = 0;
    for (const auto value : x)
        if (std::isfinite(value) && std::abs(static_cast<double>(value)) >= 0.98) ++nearFullScaleSamples;
    s.nearFullScaleFraction = static_cast<double>(nearFullScaleSamples)
        / static_cast<double>(std::max<std::size_t>(1, x.size()));

    const auto flatTolerance = std::max(2.0e-6, s.peak * 2.0e-5);
    const auto highLevel = std::max(0.90, s.peak * 0.995);
    std::size_t flatRun = 0;
    for (std::size_t i = 1; i < x.size(); ++i) {
        const auto current = static_cast<double>(x[i]);
        const auto previous = static_cast<double>(x[i - 1]);
        const auto isFlatTop = std::isfinite(current) && std::isfinite(previous)
            && std::abs(current) >= highLevel
            && std::signbit(current) == std::signbit(previous)
            && std::abs(current - previous) <= flatTolerance;
        flatRun = isFlatTop ? flatRun + 1 : 0;
        s.longestFlatTop = std::max(s.longestFlatTop, flatRun);
    }
    return s;
}

// Analyse the final steady-state window. The FFT is used both for broad energy
// balance and for a log-band engine fingerprint; this is less phase-sensitive
// than probing a handful of individual DFT frequencies.
WindowAnalysis analyseWindow(const std::vector<float>& x, std::size_t begin, double sampleRate) {
    WindowAnalysis m;
    if (begin >= x.size()) return m;
    const auto n = x.size() - begin;
    double sum = 0.0, sumSq = 0.0, diffSq = 0.0, peak = 0.0;
    for (std::size_t i = begin; i < x.size(); ++i) {
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
    const auto sourceBegin = x.size() - available;
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
    for (auto& value : m.spectrum) {
        value = std::sqrt(value);
        fingerprintNorm += value * value;
    }
    fingerprintNorm = std::sqrt(std::max(fingerprintNorm, 1.0e-18));
    for (auto& value : m.spectrum) value /= fingerprintNorm;
    return m;
}

double normalisedCorrelation(const std::vector<float>& a, const std::vector<float>& b,
                             std::size_t begin) {
    double dot = 0.0, normA = 0.0, normB = 0.0;
    for (std::size_t i = begin; i < a.size() && i < b.size(); ++i) {
        const auto x = static_cast<double>(a[i]);
        const auto y = static_cast<double>(b[i]);
        if (!std::isfinite(x) || !std::isfinite(y)) continue;
        dot += x * y; normA += x * x; normB += y * y;
    }
    const auto denominator = std::sqrt(normA * normB);
    return denominator > 1.0e-18 ? dot / denominator : 1.0;
}

double cosineSimilarity(const Metrics& a, const Metrics& b) {
    double dot = 0.0;
    for (std::size_t i = 0; i < a.left.window.spectrum.size(); ++i)
        dot += a.left.window.spectrum[i] * b.left.window.spectrum[i];
    return dot;
}

Metrics renderEngine(const EngineConfig& baseConfig, const WavData& ir,
                     const std::filesystem::path& outDir, double seconds, bool writeOutput,
                     bool syntheticTurbo = false) {
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
    // Constructing the runtime publishes the production static geometry-to-audio
    // mapping; publishAudioFrame() below is the same per-frame mapping the
    // runtime thread uses. The harness steps the simulator itself so the run
    // stays deterministic, but neither mapping is duplicated here.
    auto audioConfiguration = std::make_unique<EngineRuntime>(config);
    auto& audioState = audioConfiguration->audioState();

    constexpr double audioRate = 48'000.0;
    constexpr double dt = 1.0 / 240.0;
    constexpr int samplesPerStep = 200; // 48000 / 240
    auto rendererPtr = std::make_unique<RealtimeEngineAudio>(eventQueue, audioState, &pressureQueue);
    auto& renderer = *rendererPtr;
    if (!ir.samples.empty()) renderer.setImpulseResponse(ir.samples, ir.sampleRate, 0);
    renderer.prepare(audioRate, samplesPerStep);
    // Let the convolver's background IR load settle before rendering.
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    std::vector<float> audioLeft, audioRight;
    audioLeft.reserve(static_cast<std::size_t>(seconds * audioRate));
    audioRight.reserve(static_cast<std::size_t>(seconds * audioRate));
    juce::AudioBuffer<float> block(2, samplesPerStep);
    double realtimeSeconds = 0.0;
    double dynoLoadIntegral = 0.0;
    std::uint64_t droppedEvents = 0;
    std::uint64_t droppedPressureSamples = 0;
    const auto dynoTargetRpm = std::max(config.idleRpm * 1.50, config.redlineRpm * 0.55);
    const auto steps = static_cast<std::size_t>(seconds / dt);
    for (std::size_t step = 0; step < steps; ++step) {
        const auto t = static_cast<double>(step) * dt;
        EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = t < 1.1;
        controls.throttle = t < 0.9 ? 0.2 : 0.72;
        if (t >= 1.2) {
            const auto speedError = (simulator.state().rpm - dynoTargetRpm)
                / std::max(1.0, dynoTargetRpm);
            dynoLoadIntegral = std::clamp(
                dynoLoadIntegral + speedError * dt * 1.20, 0.0, 0.92);
            controls.load = std::clamp(dynoLoadIntegral + speedError * 0.70, 0.0, 1.0);
        }
        auto frame = simulator.step(dt, controls);
        droppedEvents += frame.droppedFiringEventCount;
        droppedPressureSamples += frame.droppedCylinderPressureSampleCount;
        const auto simStart = frame.state.simulationTimeSeconds - dt;
        for (std::size_t i = 0; i < frame.firingEventCount; ++i) {
            const auto fraction = std::clamp((frame.firingEvents[i].timeSeconds - simStart) / dt, 0.0, 1.0);
            frame.firingEvents[i].timeSeconds = realtimeSeconds + fraction * dt;
            if (!eventQueue.tryPush(frame.firingEvents[i])) ++droppedEvents;
        }
        CylinderPressureSample ps;
        while (simulator.tryPopCylinderPressureSample(ps)) {
            const auto fraction = std::clamp((ps.timeSeconds - simStart) / dt, 0.0, 1.0);
            ps.timeSeconds = realtimeSeconds + fraction * dt;
            if (!pressureQueue.tryPush(ps)) ++droppedPressureSamples;
        }
        publishAudioFrame(audioState, frame.state,
            { false, controls.starterEngaged, 0.0, 1.0 });
        audioState.producerTimeNanoseconds.store(
            static_cast<std::uint64_t>(std::max(0.0, realtimeSeconds + dt) * 1.0e9),
            std::memory_order_release);
        if (syntheticTurbo) {
            // Synthetic spool profile: ramp shaft speed + boost, then a lift-off
            // near the end to exercise the blow-off transient.
            const auto spool = std::clamp((t - 0.5) / 1.6, 0.0, 1.0);
            const auto liftOff = t > 2.4 && t < 2.5;
            audioState.forcedInductionKind.store(1);
            audioState.forcedInductionShaftRpm.store(static_cast<float>(20'000.0 + spool * 95'000.0));
            audioState.boostPressureRatio.store(static_cast<float>(1.0 + spool * 0.8));
            audioState.wastegateOpening.store(static_cast<float>(spool * 0.25));
            audioState.throttle.store(liftOff ? 0.1F : static_cast<float>(std::clamp(0.4 + spool * 0.5, 0.0, 0.95)));
        }

        block.clear();
        renderer.render(block, 0, samplesPerStep);
        for (int s = 0; s < samplesPerStep; ++s) {
            audioLeft.push_back(block.getSample(0, s));
            audioRight.push_back(block.getSample(1, s));
        }
        realtimeSeconds += dt;
    }

    Metrics m;
    // Safety properties scan the complete signal; only the fingerprint uses the
    // final steady-state window.
    m.left.scan = scanSignal(audioLeft);
    m.right.scan = scanSignal(audioRight);
    const auto analysisBegin = audioLeft.size() > static_cast<std::size_t>(1.0 * audioRate)
        ? audioLeft.size() - static_cast<std::size_t>(1.0 * audioRate) : 0;
    m.left.window = analyseWindow(audioLeft, analysisBegin, audioRate);
    m.right.window = analyseWindow(audioRight, analysisBegin, audioRate);
    m.channelCorrelation = normalisedCorrelation(audioLeft, audioRight, analysisBegin);
    m.finalRpm = simulator.state().rpm;
    m.droppedEvents = droppedEvents + renderer.droppedPendingEventCount();
    m.droppedPressureSamples = droppedPressureSamples;
    m.lateEvents = renderer.lateEventCount();
    m.stolenVoices = renderer.stolenVoiceCount();
    m.delayTruncations = renderer.delayTruncationCount();
    if (writeOutput)
        writeWav(outDir / (config.name + ".wav"), audioLeft, audioRight, static_cast<int>(audioRate));
    std::cout << std::left << std::setw(26) << config.name
              << " rms="   << std::fixed << std::setprecision(4) << m.left.window.rms
              << '/' << m.right.window.rms
              << " peak="  << m.left.scan.peak << '/' << m.right.scan.peak
              << " crest=" << std::setprecision(2) << m.left.window.crest
              << " rpm=" << std::setprecision(0) << m.finalRpm
              << " brightness=" << std::setprecision(3) << m.left.window.brightness
              << " dc=" << std::showpos << m.left.window.mean << std::noshowpos
              << " bands=" << std::setprecision(1) << m.left.window.lowBandFraction * 100.0
              << '/' << m.left.window.midBandFraction * 100.0
              << '/' << m.left.window.highBandFraction * 100.0 << '%'
              << " LRcorr=" << std::setprecision(3) << m.channelCorrelation
              << " nearFS=" << std::setprecision(4) << m.left.scan.nearFullScaleFraction
              << " flat=" << m.left.scan.longestFlatTop
              << " dropped=" << m.droppedEvents
              << " pressureDrops=" << m.droppedPressureSamples
              << " late="   << m.lateEvents
              << " stolen=" << m.stolenVoices
              << " truncated=" << m.delayTruncations
              << " finite=" << (m.left.scan.finite && m.right.scan.finite ? "yes" : "NO")
              << '\n';
    return m;
}

// ---------------------------------------------------------------------------
// Exhaust decay instrument
// ---------------------------------------------------------------------------

struct DecayMeasurement final {
    double rt60Seconds { 0.0 };
    double peak { 0.0 };
    double tailFloorDb { 0.0 };
    bool finite { true };
    bool valid { false };
};

/**
 * Reverberation time from Schroeder backward integration of an impulse tail.
 *
 * The energy decay curve is integrated from the end of the capture back to the
 * impulse peak, then RT60 is extrapolated from the -5 dB to -25 dB slope (RT20).
 * Starting at -5 dB skips the direct impulse; extrapolating from RT20 keeps the
 * estimate above the capture's noise/truncation floor.
 */
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

/**
 * Drive one runner-pressure impulse through an otherwise silent renderer and
 * measure the exhaust chain's decay for a given preset.
 *
 * The impulse is injected as cylinder-pressure telemetry rather than as a firing
 * event on purpose. Firing-event voices are summed straight onto the exhaust bus
 * and into the per-path IR; they never enter the collector. Only the pressure
 * stream reaches the runner waveguide, collector junction, outlet reflection line
 * and muffler FDN, so only a pressure-driven impulse can measure them.
 *
 * Everything but the exhaust bus is muted and the engine is held at rest, so no
 * continuous source (jet, induction, mechanical, high-frequency noise) adds a
 * floor that would flatten the Schroeder integral. The impulse amplitude keeps
 * the safety leveler and the master soft-limiter at identity, so the measured
 * decay is the acoustic model's own and not a compressor's release.
 *
 * convolutionMix selects what is measured: at 0 the per-path IR is muted and the
 * decay is the model's own. Note that `convolution` also scales the muffler FDN
 * wet and the exhaust-body excitation, so 0 removes those too; the pair of
 * measurements brackets the model rather than isolating one stage.
 */
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
    auto rendererPtr = std::make_unique<RealtimeEngineAudio>(eventQueue, audioState, &pressureQueue);
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

    // A 1 ms raised-cosine blowdown pulse on one cylinder, well after the
    // renderer's look-ahead so the whole pulse is scheduled rather than clipped.
    constexpr double pulseCentreSeconds = 0.050;
    constexpr double pulseWidthSeconds = 0.001;
    // Sized to keep the rendered peak far below the safety leveler's 0.78
    // threshold and the limiter's 0.82 knee, so both stay at identity and the
    // measured decay stays amplitude-invariant.
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
        // Publish telemetry slightly ahead of the block being rendered, exactly
        // as the runtime's producer does.
        while (nextPressureTime < blockEnd + 0.010) {
            CylinderPressureSample sample;
            sample.timeSeconds = nextPressureTime;
            sample.cylinderCount = 1;
            sample.pressureBar[0] = ambientKpa * 0.01F;
            sample.exhaustRunnerPressureKpa[0] = ambientKpa + pulseAt(nextPressureTime);
            sample.exhaustFlowMgPerCycle[0] = 60.0F;
            // A closed port is the reflective termination the primary rings against.
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
} // namespace

int main(int argc, char** argv) {
    std::cout << std::unitbuf;
    std::filesystem::path outDir = "audio-render-output";
    std::filesystem::path irPath = std::filesystem::path(ENGINELAB_CATALOG_ROOT) / "assets" / "ir" / "exhaust_default.wav";
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--output" && i + 1 < argc) outDir = argv[++i];
        else if (a == "--ir" && i + 1 < argc) irPath = argv[++i];
    }
    std::filesystem::create_directories(outDir);

    WavData ir;
    if (loadWav(irPath, ir))
        std::cout << "Loaded IR: " << irPath.string() << " (" << ir.samples.size()
                  << " samples @ " << ir.sampleRate << " Hz)\n";
    else
        std::cout << "WARNING: could not load IR at " << irPath.string() << " (using renderer fallback)\n";

    struct Named { std::string label; EngineConfig config; };
    std::vector<Named> engines {
        { "inline4", makeDefaultInlineFour() },
        { "v8",      makeDefaultV8() },
        { "inline2", makeDefaultInlineTwo() },
        { "radial5", makeDefaultRadialFive() },
    };

    std::cout << "\n--- Per-engine render (3.0 s) ---\n";
    std::vector<Metrics> metrics;
    for (auto& e : engines) metrics.push_back(renderEngine(e.config, ir, outDir, 3.0, true));

    std::cout << "\n--- Spectral differentiation (cosine similarity, lower = more distinct) ---\n";
    double worstSimilarity = 0.0;
    for (std::size_t i = 0; i < engines.size(); ++i)
        for (std::size_t j = i + 1; j < engines.size(); ++j) {
            const auto sim = cosineSimilarity(metrics[i], metrics[j]);
            worstSimilarity = std::max(worstSimilarity, sim);
            std::cout << "  " << std::left << std::setw(10) << engines[i].label
                      << " vs " << std::setw(10) << engines[j].label
                      << " similarity=" << std::fixed << std::setprecision(3) << sim << '\n';
        }

    std::cout << "\n--- Forced-induction layer (inline4 + synthetic turbo, 3.0 s) ---\n";
    const auto turbo = renderEngine(makeDefaultInlineFour(), ir, outDir, 3.0, true, true);

    std::cout << "\n--- Long-run stability (inline4, 25 s) ---\n";
    const auto stability = renderEngine(makeDefaultInlineFour(), ir, outDir, 25.0, false);

    // Instrument, not a gate: the correct RT60 per preset is not yet established,
    // so this reports the measurement and only fails if the tail is unmeasurable.
    // Calibrating the muffler FDN and its presets against these numbers is the
    // next step; recording the values here is what makes that step falsifiable.
    std::cout << "\n--- Exhaust decay per preset (inline4, Schroeder RT60) ---\n";
    constexpr std::array<const char*, 5> presetNames {
        "street", "openHeaders", "turboMuffled", "longTube", "motorcycle" };
    bool decayMeasurable = true;
    for (int preset = 0; preset < 5; ++preset) {
        const auto dry = measureExhaustDecay(makeDefaultInlineFour(), ir, preset, 6.0, 0.0F);
        const auto wet = measureExhaustDecay(makeDefaultInlineFour(), ir, preset, 6.0, 1.0F);
        std::cout << "  " << std::left << std::setw(14) << presetNames[static_cast<std::size_t>(preset)]
                  << " rt60(noIR/FDN)=" << std::fixed << std::setprecision(3) << dry.rt60Seconds << " s"
                  << " rt60(full)=" << wet.rt60Seconds << " s"
                  << " peak=" << std::setprecision(4) << dry.peak << '/' << wet.peak
                  << " tailFloor=" << std::setprecision(1) << wet.tailFloorDb << " dB"
                  << " finite=" << (dry.finite && wet.finite ? "yes" : "NO")
                  << (dry.valid && wet.valid ? "" : "  [UNMEASURABLE]") << '\n';
        if (!dry.finite || !dry.valid || !wet.finite || !wet.valid) decayMeasurable = false;
    }

    bool ok = true;
    const auto validate = [&ok](const Metrics& m, const std::string& label,
                                bool requireSpectralBalance) {
        const auto fail = [&ok, &label](const std::string& reason) {
            std::cerr << "FAIL: " << label << ": " << reason << '\n';
            ok = false;
        };
        const auto channels = { std::pair { "left", &m.left }, std::pair { "right", &m.right } };
        for (const auto& [name, channel] : channels) {
            const std::string suffix = std::string(" (") + name + " channel)";
            // Safety properties hold over the whole render, not just the window.
            if (!channel->scan.finite) fail("non-finite audio produced" + suffix);
            if (channel->scan.peak > 1.00001) fail("output exceeds digital full scale" + suffix);
            if (channel->scan.nearFullScaleFraction > 0.002)
                fail("output spends too long near digital full scale" + suffix);
            if (channel->scan.longestFlatTop > 8)
                fail("waveform contains a sustained flat clipping plateau" + suffix);
            if (channel->window.rms < 0.045)
                fail("nominal output is below the calibrated -27 dBFS RMS floor" + suffix);
            if (channel->window.rms > 0.32)
                fail("nominal output exceeds the calibrated -10 dBFS RMS ceiling" + suffix);
            if (channel->window.crest < 1.50) fail("insufficient waveform dynamics" + suffix);
            if (channel->window.crest > 16.0) fail("isolated spikes dominate the waveform" + suffix);
            if (std::abs(channel->window.mean) > std::max(0.0025, channel->window.rms * 0.08))
                fail("post-transient DC offset exceeds 8% of RMS" + suffix);
            if (requireSpectralBalance && channel->window.lowBandFraction > 0.985)
                fail("more than 98.5% of analysed energy is below 250 Hz" + suffix);
            if (requireSpectralBalance && channel->window.highBandFraction > 0.70)
                fail("upper-band noise dominates the engine signal" + suffix);
        }
        if (m.droppedEvents != 0) fail("firing events were dropped");
        if (m.droppedPressureSamples != 0) fail("cylinder-pressure samples were dropped");
        if (m.lateEvents != 0) fail("audio events missed their scheduled render time");
        if (m.stolenVoices != 0) fail("polyphony exhaustion stole active combustion voices");
        // Delay lines are sized in prepare() from the published geometry, so a
        // clamp here means the rendered acoustic length is shorter than configured.
        if (m.delayTruncations != 0) fail("a delay line was too short and truncated");
        if (m.channelCorrelation > 0.999)
            fail("left and right are effectively identical: stereo image collapsed to mono");
    };
    for (std::size_t index = 0; index < metrics.size(); ++index)
        validate(metrics[index], engines[index].label, true);
    validate(turbo, "synthetic turbo", true);
    validate(stability, "long-run inline4", true);
    if (worstSimilarity > 0.985) {
        std::cerr << "FAIL: engines are not spectrally differentiated (max similarity "
                  << worstSimilarity << ")\n";
        ok = false;
    }
    if (!decayMeasurable) {
        std::cerr << "FAIL: exhaust decay could not be measured for at least one preset\n";
        ok = false;
    }
    std::cout << "\nResult: " << (ok ? "PASS" : "FAIL")
              << " (max spectral similarity " << std::setprecision(3) << worstSimilarity << ")\n";
    return ok ? 0 : 1;
}
