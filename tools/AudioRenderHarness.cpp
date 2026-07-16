// Offline driver for the real realtime audio path (RealtimeEngineAudio) so the
// exhaust/sound changes can be verified without a sound device: it runs the
// physics simulator, feeds the firing-event and cylinder-pressure queues exactly
// like EngineRuntime does, renders through the real convolution IR, and reports
// objective metrics (level, crest factor, spectral fingerprint, stability).

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

void writeWav(const std::filesystem::path& path, const std::vector<float>& samples, int sampleRate) {
    std::ofstream out(path, std::ios::binary);
    const auto dataBytes = static_cast<std::uint32_t>(samples.size() * sizeof(std::int16_t));
    const auto put32 = [&](std::uint32_t v) { for (int i = 0; i < 4; ++i) out.put(static_cast<char>((v >> (8 * i)) & 0xffU)); };
    const auto put16 = [&](std::uint16_t v) { for (int i = 0; i < 2; ++i) out.put(static_cast<char>((v >> (8 * i)) & 0xffU)); };
    out.write("RIFF", 4); put32(36U + dataBytes); out.write("WAVEfmt ", 8);
    put32(16U); put16(1U); put16(1U); put32(static_cast<std::uint32_t>(sampleRate));
    put32(static_cast<std::uint32_t>(sampleRate * 2)); put16(2U); put16(16U);
    out.write("data", 4); put32(dataBytes);
    for (const auto s : samples) {
        const auto v = static_cast<std::int16_t>(std::lrint(std::clamp(s, -1.0F, 1.0F) * 32767.0F));
        put16(static_cast<std::uint16_t>(v));
    }
}

struct Metrics {
    double mean {};
    double rms {};
    double peak {};
    double crest {};
    double brightness {};
    double nearFullScaleFraction {};
    double lowBandFraction {};
    double midBandFraction {};
    double highBandFraction {};
    double finalRpm {};
    std::size_t longestFlatTop {};
    std::array<double, 32> spectrum {};
    bool finite { true };
};

// Analyse the final steady-state window. The FFT is used both for broad energy
// balance and for a log-band engine fingerprint; this is less phase-sensitive
// than probing a handful of individual DFT frequencies.
void fingerprint(const std::vector<float>& x, std::size_t begin, double sampleRate, Metrics& m) {
    const auto n = x.size() - begin;
    double sum = 0.0, sumSq = 0.0, diffSq = 0.0, peak = 0.0;
    std::size_t nearFullScaleSamples = 0;
    for (std::size_t i = begin; i < x.size(); ++i) {
        const auto v = static_cast<double>(x[i]);
        if (!std::isfinite(v)) m.finite = false;
        sum += v;
        sumSq += v * v; peak = std::max(peak, std::abs(v));
        if (std::abs(v) >= 0.98) ++nearFullScaleSamples;
        if (i > begin) { const auto d = v - x[i - 1]; diffSq += d * d; }
    }
    m.mean = sum / static_cast<double>(std::max<std::size_t>(1, n));
    m.rms = std::sqrt(sumSq / std::max<std::size_t>(1, n));
    m.peak = peak;
    m.crest = m.rms > 1e-12 ? m.peak / m.rms : 0.0;
    m.brightness = sumSq > 1e-12 ? std::sqrt(diffSq / sumSq) : 0.0;
    m.nearFullScaleFraction = static_cast<double>(nearFullScaleSamples)
        / static_cast<double>(std::max<std::size_t>(1, n));

    const auto flatTolerance = std::max(2.0e-6, m.peak * 2.0e-5);
    const auto highLevel = std::max(0.90, m.peak * 0.995);
    std::size_t flatRun = 0;
    for (std::size_t i = begin + 1; i < x.size(); ++i) {
        const auto current = static_cast<double>(x[i]);
        const auto previous = static_cast<double>(x[i - 1]);
        const auto isFlatTop = std::abs(current) >= highLevel
            && std::signbit(current) == std::signbit(previous)
            && std::abs(current - previous) <= flatTolerance;
        flatRun = isFlatTop ? flatRun + 1 : 0;
        m.longestFlatTop = std::max(m.longestFlatTop, flatRun);
    }

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
        fftData[fftIndex] = static_cast<float>(
            (static_cast<double>(x[sourceBegin + i]) - m.mean) * window);
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
}

double cosineSimilarity(const Metrics& a, const Metrics& b) {
    double dot = 0.0;
    for (std::size_t i = 0; i < a.spectrum.size(); ++i) dot += a.spectrum[i] * b.spectrum[i];
    return dot;
}

Metrics renderEngine(const EngineConfig& baseConfig, const WavData& ir,
                     const std::filesystem::path& outDir, double seconds, bool stabilityRun,
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
    // Use the production runtime's exact graph/geometry-to-audio mapping. A
    // hand-maintained copy here previously drifted and made this regression
    // harness test a different renderer configuration than the application.
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

    std::vector<float> audio;
    audio.reserve(static_cast<std::size_t>(seconds * audioRate));
    juce::AudioBuffer<float> block(2, samplesPerStep);
    double realtimeSeconds = 0.0;
    double dynoLoadIntegral = 0.0;
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
        const auto simStart = frame.state.simulationTimeSeconds - dt;
        for (std::size_t i = 0; i < frame.firingEventCount; ++i) {
            const auto fraction = std::clamp((frame.firingEvents[i].timeSeconds - simStart) / dt, 0.0, 1.0);
            frame.firingEvents[i].timeSeconds = realtimeSeconds + fraction * dt;
            (void)eventQueue.tryPush(frame.firingEvents[i]);
        }
        CylinderPressureSample ps;
        while (simulator.tryPopCylinderPressureSample(ps)) {
            const auto fraction = std::clamp((ps.timeSeconds - simStart) / dt, 0.0, 1.0);
            ps.timeSeconds = realtimeSeconds + fraction * dt;
            (void)pressureQueue.tryPush(ps);
        }
        audioState.rpm.store(static_cast<float>(frame.state.rpm));
        audioState.throttle.store(static_cast<float>(frame.state.throttle));
        audioState.load.store(static_cast<float>(frame.state.load));
        audioState.manifoldPressureKpa.store(static_cast<float>(frame.state.manifoldPressureKpa));
        audioState.exhaustPressureKpa.store(static_cast<float>(frame.state.exhaustPressureKpa));
        audioState.exhaustFlowGramsPerSecond.store(static_cast<float>(frame.state.exhaustFlowGramsPerSecond));
        audioState.exhaustTemperatureC.store(static_cast<float>(frame.state.exhaustTemperatureC));
        audioState.boostPressureRatio.store(static_cast<float>(frame.state.boostPressureRatio));
        audioState.peakPistonAccelerationG.store(static_cast<float>(std::max(0.0, frame.state.peakPistonAccelerationG)));
        audioState.forcedInductionShaftRpm.store(static_cast<float>(std::max(0.0, frame.state.forcedInductionShaftSpeedRpm)));
        audioState.wastegateOpening.store(static_cast<float>(std::clamp(frame.state.wastegateOpening, 0.0, 1.0)));
        {
            double domAmp = 0.0, domFreq = 0.0;
            for (std::size_t i = 0; i < frame.state.cylinderStateCount; ++i) {
                const auto& c = frame.state.cylinderStates[i];
                if (std::abs(c.intakeResonancePressureKpa) > std::abs(domAmp)) {
                    domAmp = c.intakeResonancePressureKpa;
                    domFreq = c.intakeResonanceFrequencyHz;
                }
            }
            audioState.intakeRunnerResonanceHz.store(static_cast<float>(domFreq));
            audioState.intakeRunnerAmplitudeKpa.store(static_cast<float>(domAmp));
        }
        audioState.starter.store(controls.starterEngaged ? 1.0F : 0.0F);
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
        if (!stabilityRun)
            for (int s = 0; s < samplesPerStep; ++s) audio.push_back(block.getSample(0, s));
        else
            for (int s = 0; s < samplesPerStep; ++s) audio.push_back(block.getSample(0, s)); // keep for finite/peak scan
        realtimeSeconds += dt;
    }

    Metrics m;
    const auto analysisBegin = audio.size() > static_cast<std::size_t>(1.0 * audioRate)
        ? audio.size() - static_cast<std::size_t>(1.0 * audioRate) : 0;
    fingerprint(audio, analysisBegin, audioRate, m);
    m.finalRpm = simulator.state().rpm;
    if (!stabilityRun)
        writeWav(outDir / (config.name + ".wav"), audio, static_cast<int>(audioRate));
    std::cout << std::left << std::setw(26) << config.name
              << " rms="   << std::fixed << std::setprecision(4) << m.rms
              << " peak="  << m.peak
              << " crest=" << std::setprecision(2) << m.crest
              << " rpm=" << std::setprecision(0) << m.finalRpm
              << " brightness=" << std::setprecision(3) << m.brightness
              << " dc=" << std::showpos << m.mean << std::noshowpos
              << " bands=" << std::setprecision(1) << m.lowBandFraction * 100.0
              << '/' << m.midBandFraction * 100.0 << '/' << m.highBandFraction * 100.0 << '%'
              << " nearFS=" << std::setprecision(4) << m.nearFullScaleFraction
              << " flat=" << m.longestFlatTop
              << " dropped=" << renderer.droppedPendingEventCount()
              << " late="   << renderer.lateEventCount()
              << " finite=" << (m.finite ? "yes" : "NO")
              << '\n';
    return m;
}
} // namespace

int main(int argc, char** argv) {
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
    for (auto& e : engines) metrics.push_back(renderEngine(e.config, ir, outDir, 3.0, false));

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
    const auto turbo = renderEngine(makeDefaultInlineFour(), ir, outDir, 3.0, false, true);

    std::cout << "\n--- Long-run stability (inline4, 25 s) ---\n";
    const auto stability = renderEngine(makeDefaultInlineFour(), ir, outDir, 25.0, true);

    bool ok = true;
    const auto validate = [&ok](const Metrics& m, const std::string& label,
                                bool requireSpectralBalance) {
        const auto fail = [&ok, &label](const std::string& reason) {
            std::cerr << "FAIL: " << label << ": " << reason << '\n';
            ok = false;
        };
        if (!m.finite) fail("non-finite audio produced");
        if (m.rms <= 1.0e-5) fail("analysis window is unexpectedly silent");
        if (m.peak > 1.00001) fail("output exceeds digital full scale");
        if (m.crest < 1.50) fail("insufficient waveform dynamics");
        if (m.crest > 16.0) fail("isolated spikes dominate the waveform");
        if (std::abs(m.mean) > std::max(0.0025, m.rms * 0.08))
            fail("post-transient DC offset exceeds 8% of RMS");
        if (m.nearFullScaleFraction > 0.002)
            fail("output spends too long near digital full scale");
        if (m.longestFlatTop > 8)
            fail("waveform contains a sustained flat clipping plateau");
        if (requireSpectralBalance && m.lowBandFraction > 0.985)
            fail("more than 98.5% of analysed energy is below 250 Hz");
        if (requireSpectralBalance && m.highBandFraction > 0.70)
            fail("upper-band noise dominates the engine signal");
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
    std::cout << "\nResult: " << (ok ? "PASS" : "FAIL")
              << " (max spectral similarity " << std::setprecision(3) << worstSimilarity << ")\n";
    return ok ? 0 : 1;
}
