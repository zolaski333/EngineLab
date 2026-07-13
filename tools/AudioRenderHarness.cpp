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

// Replicates the config-derived audio telemetry that EngineRuntime sets up once.
void configureStaticAudioState(const EngineConfig& config, RealtimeAudioState& state) {
    const auto cylinders = std::max<std::size_t>(1, config.cylinders.size());
    const auto displacement = engineDisplacementLitres(config);
    state.cylinderCount.store(static_cast<float>(cylinders));
    state.redlineRpm.store(static_cast<float>(config.redlineRpm));
    state.displacementLitres.store(static_cast<float>(displacement));
    state.cylinderDisplacementLitres.store(static_cast<float>(displacement / static_cast<double>(cylinders)));
    const auto bankSeparation = config.layout == EngineLayout::vLayout ? 1.0F
        : (config.layout == EngineLayout::flat ? 0.92F : (config.layout == EngineLayout::radial ? 0.74F : 0.0F));
    state.bankSeparation.store(bankSeparation);
    for (std::size_t i = 0; i < cylinders; ++i) {
        const auto pan = cylinders > 1
            ? static_cast<float>(-0.72 + 1.44 * static_cast<double>(i) / static_cast<double>(cylinders - 1)) : 0.0F;
        state.cylinderPan[i].store(std::clamp(pan, -0.82F, 0.82F));
    }
    state.exhaustOpenness.store(static_cast<float>(std::clamp(
        (config.exhaust.outletDiameterMm / std::max(20.0, config.exhaust.collectorDiameterMm))
            * (1.0 - config.exhaust.mufflerRestriction * 0.72), 0.15, 1.45)));
    double boreSum = 0.0;
    for (const auto& c : config.cylinders) boreSum += c.boreMm;
    state.meanBoreMm.store(static_cast<float>(std::max(20.0, boreSum / static_cast<double>(cylinders))));
    state.forcedInductionKind.store(config.forcedInduction.enabled
        ? (config.forcedInduction.type == ForcedInductionType::supercharger ? 2 : 1) : 0);
    constexpr double exhaustSoundSpeedMmPerSecond = 520'000.0;
    for (std::size_t i = 0; i < cylinders && i < 32; ++i) {
        const auto len = config.cylinders[i].exhaustPrimaryLengthMm > 0.0
            ? config.cylinders[i].exhaustPrimaryLengthMm : config.exhaust.primaryLengthMm;
        state.runnerDelaySeconds[i].store(static_cast<float>(std::max(0.0, len) / exhaustSoundSpeedMmPerSecond));
    }
}

struct Metrics { double rms {}, peak {}, crest {}, brightness {}; std::array<double, 24> spectrum {}; bool finite { true }; };

// Log-spaced magnitude spectrum (naive DFT) over the analysis window, unit-normalised.
void fingerprint(const std::vector<float>& x, std::size_t begin, double sampleRate, Metrics& m) {
    const auto n = x.size() - begin;
    double sumSq = 0.0, diffSq = 0.0, peak = 0.0;
    for (std::size_t i = begin; i < x.size(); ++i) {
        const auto v = static_cast<double>(x[i]);
        if (!std::isfinite(v)) m.finite = false;
        sumSq += v * v; peak = std::max(peak, std::abs(v));
        if (i > begin) { const auto d = v - x[i - 1]; diffSq += d * d; }
    }
    m.rms = std::sqrt(sumSq / std::max<std::size_t>(1, n));
    m.peak = peak;
    m.crest = m.rms > 1e-12 ? m.peak / m.rms : 0.0;
    m.brightness = sumSq > 1e-12 ? std::sqrt(diffSq / sumSq) : 0.0;
    double norm = 0.0;
    for (std::size_t b = 0; b < m.spectrum.size(); ++b) {
        const auto freq = 40.0 * std::pow(8000.0 / 40.0, static_cast<double>(b) / (m.spectrum.size() - 1));
        const auto w = 2.0 * std::numbers::pi * freq / sampleRate;
        double re = 0.0, im = 0.0;
        const std::size_t stride = std::max<std::size_t>(1, n / 4000); // ~4k point DFT is plenty
        for (std::size_t i = begin; i < x.size(); i += stride) {
            const auto ph = w * static_cast<double>(i - begin);
            re += x[i] * std::cos(ph); im += x[i] * std::sin(ph);
        }
        m.spectrum[b] = std::sqrt(re * re + im * im);
        norm += m.spectrum[b] * m.spectrum[b];
    }
    norm = std::sqrt(std::max(norm, 1e-18));
    for (auto& v : m.spectrum) v /= norm;
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
    RealtimeAudioState audioState;
    configureStaticAudioState(config, audioState);

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
    const auto steps = static_cast<std::size_t>(seconds / dt);
    for (std::size_t step = 0; step < steps; ++step) {
        const auto t = static_cast<double>(step) * dt;
        EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = t < 1.1;
        controls.throttle = t < 0.9 ? 0.2 : std::clamp(0.35 + (t - 0.9) * 0.25, 0.0, 0.9);
        controls.load = t < 1.5 ? 0.0 : 0.35;
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
        audioState.boostPressureRatio.store(static_cast<float>(frame.state.boostPressureRatio));
        audioState.peakPistonAccelerationG.store(static_cast<float>(std::max(0.0, frame.state.peakPistonAccelerationG)));
        audioState.forcedInductionShaftRpm.store(static_cast<float>(std::max(0.0, frame.state.forcedInductionShaftSpeedRpm)));
        audioState.wastegateOpening.store(static_cast<float>(std::clamp(frame.state.wastegateOpening, 0.0, 1.0)));
        {
            double domAmp = 0.0, domFreq = 0.0;
            for (std::size_t i = 0; i < frame.state.cylinderStateCount; ++i) {
                const auto& c = frame.state.cylinderStates[i];
                if (c.intakeResonancePressureKpa > domAmp) { domAmp = c.intakeResonancePressureKpa; domFreq = c.intakeResonanceFrequencyHz; }
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
    if (!stabilityRun)
        writeWav(outDir / (config.name + ".wav"), audio, static_cast<int>(audioRate));
    std::cout << std::left << std::setw(26) << config.name
              << " rms="   << std::fixed << std::setprecision(4) << m.rms
              << " peak="  << m.peak
              << " crest=" << std::setprecision(2) << m.crest
              << " brightness=" << std::setprecision(3) << m.brightness
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
    for (const auto& m : metrics) {
        if (!m.finite) { std::cerr << "FAIL: non-finite audio produced\n"; ok = false; }
        if (m.crest < 2.0) { std::cerr << "FAIL: crest factor too low (over-saturated): " << m.crest << '\n'; ok = false; }
    }
    if (!stability.finite) { std::cerr << "FAIL: long run produced non-finite audio\n"; ok = false; }
    if (!turbo.finite) { std::cerr << "FAIL: forced-induction layer produced non-finite audio\n"; ok = false; }
    if (turbo.crest < 2.0) { std::cerr << "FAIL: forced-induction crest too low: " << turbo.crest << '\n'; ok = false; }
    if (worstSimilarity > 0.985) { std::cerr << "WARN: engines not well differentiated (max similarity "
                                             << worstSimilarity << ")\n"; }
    std::cout << "\nResult: " << (ok ? "PASS" : "FAIL")
              << " (max spectral similarity " << std::setprecision(3) << worstSimilarity << ")\n";
    return ok ? 0 : 1;
}
