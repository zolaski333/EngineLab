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

#include <enginelab/audio/AcousticMonitorCalibration.hpp>
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
#include <memory>
#include <numbers>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

namespace {
using namespace enginelab;

struct WavData { std::vector<float> samples; double sampleRate { 44'100.0 }; };

// Layer isolation switches (--mute-combustion / --mute-mechanical). The exhaust
// is only one of several layers summed into the output, so attributing a
// measured artefact to the exhaust model requires being able to silence the
// others. Diagnostic only: the shipped voice renders every layer.
bool muteCombustionLayer = false;
bool muteMechanicalLayer = false;
bool muteIntakeLayer = false;
// Offline oracle only. Large engines are not expected to meet realtime when the
// complete nonlinear network is advanced on every mechanical substep.
bool referenceCouplingEverySubstep = false;
// Render chunk size within each simulation step; see the invariance probe at
// the render call. 200 reproduces the historical single-call behaviour.
int audioChunkSamples = 200;

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
    // Worst isolated narrowband resonance between 300 Hz and 9 kHz, in dB above
    // the local spectral baseline. This is the objective measure of "metallic":
    // a real exhaust radiates a dense harmonic series whose envelope falls off
    // smoothly, so every partial sits close to its neighbours. An undamped
    // waveguide mode instead stands 20+ dB proud of the surrounding spectrum and
    // rings at a frequency unrelated to the firing series. Judging that by ear is
    // exactly what this harness exists to avoid.
    double maxResonanceProminenceDb {};
    double maxResonanceFrequencyHz {};
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
    // Solver cadence, reported so an audible artefact can be checked against the
    // rates the simulator actually ran at rather than against a guess.
    double solverFrequencyHz {};
    std::uint32_t solverSubsteps {};
    // Bandwidth accounting. The network resolves the acoustic field at
    // networkSubstepHz; the audio boundary only observes it at couplingHz.
    // Spectral content above couplingHz/2 cannot be physical, so any reported
    // resonance above that line is a reconstruction image, not a mode.
    double networkSubstepHz {};
    double couplingHz {};
    // Coherence of the two valve-flow contracts. The instantaneous flow owns
    // mass accounting; the network flow is the matched partner of runner
    // pressure used by the characteristic split. Their residual is the source
    // energy phase 2 must recover without pairing mismatched states.
    double instantaneousValveFlowRmsKgPerSecond {};
    double acousticValveFlowRmsKgPerSecond {};
    double valveFlowResidualRmsKgPerSecond {};
    double valveFlowSimilarity {};
    std::uint64_t droppedEvents {};
    std::uint64_t droppedPressureSamples {};
    std::uint64_t lateEvents {};
    std::uint64_t stolenVoices {};
    std::uint64_t delayTruncations {};
    // Safety-leveler engagement. The slow AGC is a safety net, not a level
    // control: in a shipped voice it must stay at identity (0 limited samples,
    // min gain 1.0). A non-zero count means the default level is hot enough that
    // the leveler is silently doing steady-state gain work -- exactly the kind of
    // downstream compensation this harness exists to make visible.
    std::uint64_t levelLimitedSamples {};
    float minLevelGain { 1.0F };
    // Which path produced the audio. A physical path that never activated would
    // otherwise be reported as an unchanged-sounding physical path.
    bool physicalActive { false };
    std::uint64_t legacyPathSamples {};
    std::uint64_t invalidBoundarySamples {};
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

    // Narrowband resonance prominence. The baseline is a wide running mean of the
    // log spectrum, so a broad tilt (which the ear reads as tone) contributes
    // nothing while a single high-Q mode (which the ear reads as metallic) stands
    // proud of it. Bins are compared against the mean of a surrounding window
    // that excludes the immediate neighbourhood of the bin itself, so a genuine
    // firing harmonic broadened by the analysis window is not counted as a mode.
    {
        const auto binHz = sampleRate / static_cast<double>(fftSize);
        const auto firstBin = static_cast<std::size_t>(std::ceil(300.0 / binHz));
        const auto lastBin = std::min(fftSize / 2,
            static_cast<std::size_t>(9'000.0 / binHz));
        // ~200 Hz of context either side, skipping +/-25 Hz around the bin.
        const auto contextBins = std::max<std::size_t>(8,
            static_cast<std::size_t>(200.0 / binHz));
        const auto guardBins = std::max<std::size_t>(2,
            static_cast<std::size_t>(25.0 / binHz));
        const auto powerAt = [&](std::size_t bin) {
            const auto magnitude = static_cast<double>(fftData[bin]);
            return magnitude * magnitude;
        };
        for (std::size_t bin = firstBin; bin < lastBin; ++bin) {
            const auto low = bin > contextBins ? bin - contextBins : std::size_t { 1 };
            const auto high = std::min(lastBin, bin + contextBins);
            double contextPower = 0.0;
            std::size_t contextCount = 0;
            for (std::size_t other = low; other < high; ++other) {
                if (other + guardBins >= bin && other <= bin + guardBins) continue;
                contextPower += powerAt(other);
                ++contextCount;
            }
            if (contextCount < 8) continue;
            const auto baseline = contextPower / static_cast<double>(contextCount);
            if (!(baseline > 1.0e-24)) continue;
            const auto prominenceDb = 10.0 * std::log10(
                std::max(powerAt(bin), 1.0e-30) / baseline);
            if (prominenceDb > m.maxResonanceProminenceDb) {
                m.maxResonanceProminenceDb = prominenceDb;
                m.maxResonanceFrequencyHz = static_cast<double>(bin) * binHz;
            }
        }
    }

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
    simulator.setExhaustCouplingEverySubstep(referenceCouplingEverySubstep);

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
    double dynoLoadApplied = 0.0;
    std::uint64_t droppedEvents = 0;
    std::uint64_t droppedPressureSamples = 0;
    double instantaneousValveFlowSquareSum = 0.0;
    double acousticValveFlowSquareSum = 0.0;
    double valveFlowResidualSquareSum = 0.0;
    double valveFlowCrossSum = 0.0;
    std::uint64_t valveFlowSampleCount = 0;
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
            // A water-brake or eddy-current dyno has a torque bandwidth of a
            // few hertz; commanding a raw step every frame applied a brake
            // torque square wave at the 240 Hz frame rate. The engine responds
            // to that excitation physically, so the renders carried a comb at
            // exact frame-rate multiples that no renderer fix could remove:
            // the measurement rig was exciting the artefact it measured.
            const auto targetLoad = std::clamp(
                dynoLoadIntegral + speedError * 0.70, 0.0, 1.0);
            dynoLoadApplied += (1.0 - std::exp(
                -2.0 * std::numbers::pi * 2.0 * dt)) * (targetLoad - dynoLoadApplied);
            controls.load = dynoLoadApplied;
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
            const auto count = std::min(ps.cylinderCount,
                ps.exhaustMassFlowKgPerSecond.size());
            for (std::size_t index = 0; index < count; ++index) {
                const auto instantaneous = static_cast<double>(
                    ps.exhaustMassFlowKgPerSecond[index]);
                const auto acoustic = static_cast<double>(
                    ps.exhaustAcousticMassFlowKgPerSecond[index]);
                if (!std::isfinite(instantaneous) || !std::isfinite(acoustic))
                    continue;
                const auto residual = instantaneous - acoustic;
                instantaneousValveFlowSquareSum += instantaneous * instantaneous;
                acousticValveFlowSquareSum += acoustic * acoustic;
                valveFlowResidualSquareSum += residual * residual;
                valveFlowCrossSum += instantaneous * acoustic;
                ++valveFlowSampleCount;
            }
            const auto fraction = std::clamp((ps.timeSeconds - simStart) / dt, 0.0, 1.0);
            ps.timeSeconds = realtimeSeconds + fraction * dt;
            if (!pressureQueue.tryPush(ps)) ++droppedPressureSamples;
        }
        publishAudioFrame(audioState, frame.state,
            { false, controls.starterEngaged, 0.0, 1.0 });
        if (muteCombustionLayer) audioState.combustionGain.store(0.0F);
        if (muteMechanicalLayer) audioState.mechanicalGain.store(0.0F);
        if (muteIntakeLayer) audioState.intakeGain.store(0.0F);
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
        // Block-size invariance probe: a correct realtime renderer must produce
        // the same signal whatever callback size the host chooses. Rendering
        // each simulation step in smaller chunks moves any per-block parameter
        // step to a different comb frequency (sampleRate / chunk), which
        // separates render-block artefacts from simulation-frame artefacts.
        for (int offset = 0; offset < samplesPerStep; offset += audioChunkSamples)
            renderer.render(block, offset,
                std::min(audioChunkSamples, samplesPerStep - offset));
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
    m.solverFrequencyHz = simulator.state().solverFrequencyHz;
    m.solverSubsteps = simulator.state().solverSubsteps;
    m.networkSubstepHz = simulator.state().exhaustNetworkSubstepFrequencyHz;
    m.couplingHz = simulator.state().exhaustCouplingFrequencyHz;
    const auto flowSampleDivisor = static_cast<double>(
        std::max<std::uint64_t>(1, valveFlowSampleCount));
    m.instantaneousValveFlowRmsKgPerSecond = std::sqrt(
        instantaneousValveFlowSquareSum / flowSampleDivisor);
    m.acousticValveFlowRmsKgPerSecond = std::sqrt(
        acousticValveFlowSquareSum / flowSampleDivisor);
    m.valveFlowResidualRmsKgPerSecond = std::sqrt(
        valveFlowResidualSquareSum / flowSampleDivisor);
    const auto flowSimilarityDenominator = std::sqrt(
        instantaneousValveFlowSquareSum * acousticValveFlowSquareSum);
    m.valveFlowSimilarity = flowSimilarityDenominator > 1.0e-18
        ? valveFlowCrossSum / flowSimilarityDenominator : 1.0;
    m.droppedEvents = droppedEvents + renderer.droppedPendingEventCount();
    m.droppedPressureSamples = droppedPressureSamples;
    m.lateEvents = renderer.lateEventCount();
    m.stolenVoices = renderer.stolenVoiceCount();
    m.delayTruncations = renderer.delayTruncationCount();
    m.levelLimitedSamples = renderer.levelLimitedSampleCount();
    m.minLevelGain = renderer.minObservedLevelGain();
    m.physicalActive = renderer.physicalExhaustActive();
    m.legacyPathSamples = renderer.legacyPathSampleCount();
    m.invalidBoundarySamples = renderer.invalidBoundarySampleCount();
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
              << " physical=" << (m.physicalActive ? "yes" : "NO")
              << " legacySamples=" << m.legacyPathSamples
              << " boundaryDropouts=" << m.invalidBoundarySamples
              << " solverHz=" << std::setprecision(0) << m.solverFrequencyHz
              << " substeps=" << m.solverSubsteps
              << " networkHz=" << std::setprecision(0) << m.networkSubstepHz
              << " couplingHz=" << std::setprecision(0) << m.couplingHz
              << " couplingNyq=" << std::setprecision(0) << m.couplingHz * 0.5
              << " mechanicalNyq=" << std::setprecision(0) << m.solverFrequencyHz * 0.5
              << " valveFlowRms(inst/macro/res)=" << std::setprecision(5)
              << m.instantaneousValveFlowRmsKgPerSecond << '/'
              << m.acousticValveFlowRmsKgPerSecond << '/'
              << m.valveFlowResidualRmsKgPerSecond
              << " flowSimilarity=" << std::setprecision(3) << m.valveFlowSimilarity
              << " resonance=" << std::setprecision(1) << m.left.window.maxResonanceProminenceDb
              << "dB@" << std::setprecision(0) << m.left.window.maxResonanceFrequencyHz << "Hz"
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
              << " levelLimited=" << m.levelLimitedSamples
              << " minLevelGain=" << std::setprecision(4) << m.minLevelGain
              << " finite=" << (m.left.scan.finite && m.right.scan.finite ? "yes" : "NO")
              << '\n';
    return m;
}

struct IdleCycleMetrics final {
    double initialIdleRpm {};
    double initialIdleRms {};
    double revPeakRpm {};
    double returnedIdleRpm {};
    double returnedIdleRms {};
    SafetyScan scan {};
    WindowAnalysis idleWindow {};
    std::uint64_t droppedEvents {};
    std::uint64_t droppedPressureSamples {};
    std::uint64_t lateEvents {};
    std::uint64_t levelLimitedSamples {};
    float minLevelGain { 1.0F };
};

IdleCycleMetrics renderIdleCycle(const EngineConfig& baseConfig, const WavData& ir,
                                 const std::filesystem::path& outDir) {
    auto config = baseConfig;
    normaliseEngineConfig(config);
    SimpleEcuModel ecu;
    SimplifiedGasolinePhysics physics;
    FourStrokeEventGenerator events;
    auto exhaust = ExhaustGraph::makeForEngine(config);
    // These owners deliberately live on the heap. EngineSimulator, EngineRuntime,
    // the lock-free telemetry queues, and RealtimeEngineAudio each retain sizeable
    // fixed-capacity realtime storage; putting all of them in this one stack frame
    // exceeds the default Windows thread stack before the function body starts.
    // Heap ownership preserves deterministic lifetimes without changing the code
    // under test or allocating from the realtime render loop.
    auto simulatorOwner = std::make_unique<EngineSimulator>(
        config, ecu, physics, events, exhaust);
    auto& simulator = *simulatorOwner;
    simulator.setPressureSamplingEnabled(true);

    auto eventQueueOwner = std::make_unique<FiringEventQueue>();
    auto pressureQueueOwner = std::make_unique<CylinderPressureQueue>();
    auto audioConfiguration = std::make_unique<EngineRuntime>(config);
    auto& eventQueue = *eventQueueOwner;
    auto& pressureQueue = *pressureQueueOwner;
    auto& audioState = audioConfiguration->audioState();
    auto rendererOwner = std::make_unique<RealtimeEngineAudio>(
        eventQueue, audioState, &pressureQueue);
    auto& renderer = *rendererOwner;
    if (!ir.samples.empty()) renderer.setImpulseResponse(ir.samples, ir.sampleRate, 0);
    constexpr double audioRate = 48'000.0;
    constexpr double dt = 1.0 / 240.0;
    constexpr int samplesPerStep = 200;
    constexpr double durationSeconds = 12.0;
    renderer.prepare(audioRate, samplesPerStep);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    std::vector<float> left;
    std::vector<float> right;
    left.reserve(static_cast<std::size_t>(durationSeconds * audioRate));
    right.reserve(left.capacity());
    juce::AudioBuffer<float> block(2, samplesPerStep);
    IdleCycleMetrics metrics;
    double initialRpmSum = 0.0;
    double returnedRpmSum = 0.0;
    double initialSquareSum = 0.0;
    double returnedSquareSum = 0.0;
    std::size_t initialStateSamples = 0;
    std::size_t returnedStateSamples = 0;
    std::size_t initialAudioSamples = 0;
    std::size_t returnedAudioSamples = 0;
    double realtimeSeconds = 0.0;

    const auto steps = static_cast<std::size_t>(durationSeconds / dt);
    for (std::size_t step = 0; step < steps; ++step) {
        const auto t = static_cast<double>(step) * dt;
        EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = t < 1.5;
        if (t >= 4.0 && t < 4.4) controls.throttle = (t - 4.0) * 0.55;
        else if (t >= 4.4 && t < 4.7) controls.throttle = 0.22;
        else if (t >= 4.7 && t < 5.0) controls.throttle = (5.0 - t) * (0.22 / 0.30);

        auto frame = simulator.step(dt, controls);
        metrics.droppedEvents += frame.droppedFiringEventCount;
        metrics.droppedPressureSamples += frame.droppedCylinderPressureSampleCount;
        const auto simulationStart = frame.state.simulationTimeSeconds - dt;
        for (std::size_t index = 0; index < frame.firingEventCount; ++index) {
            auto event = frame.firingEvents[index];
            const auto fraction = std::clamp(
                (event.timeSeconds - simulationStart) / dt, 0.0, 1.0);
            event.timeSeconds = realtimeSeconds + fraction * dt;
            if (!eventQueue.tryPush(event)) ++metrics.droppedEvents;
        }
        CylinderPressureSample pressureSample;
        while (simulator.tryPopCylinderPressureSample(pressureSample)) {
            const auto fraction = std::clamp(
                (pressureSample.timeSeconds - simulationStart) / dt, 0.0, 1.0);
            pressureSample.timeSeconds = realtimeSeconds + fraction * dt;
            if (!pressureQueue.tryPush(pressureSample)) ++metrics.droppedPressureSamples;
        }
        publishAudioFrame(audioState, frame.state,
            { false, controls.starterEngaged, 0.0, 1.0 });
        audioState.producerTimeNanoseconds.store(
            static_cast<std::uint64_t>((realtimeSeconds + dt) * 1.0e9),
            std::memory_order_release);
        block.clear();
        renderer.render(block, 0, samplesPerStep);

        if (t >= 2.5 && t < 4.0) {
            initialRpmSum += frame.state.rpm;
            ++initialStateSamples;
        }
        if (t >= 4.0 && t < 7.0)
            metrics.revPeakRpm = std::max(metrics.revPeakRpm, frame.state.rpm);
        if (t >= 10.0) {
            returnedRpmSum += frame.state.rpm;
            ++returnedStateSamples;
        }
        for (int sample = 0; sample < samplesPerStep; ++sample) {
            const auto leftSample = block.getSample(0, sample);
            const auto rightSample = block.getSample(1, sample);
            left.push_back(leftSample);
            right.push_back(rightSample);
            if (t >= 2.5 && t < 4.0) {
                initialSquareSum += static_cast<double>(leftSample) * leftSample;
                ++initialAudioSamples;
            }
            if (t >= 10.0) {
                returnedSquareSum += static_cast<double>(leftSample) * leftSample;
                ++returnedAudioSamples;
            }
        }
        realtimeSeconds += dt;
    }

    if (initialStateSamples != 0)
        metrics.initialIdleRpm = initialRpmSum / static_cast<double>(initialStateSamples);
    if (returnedStateSamples != 0)
        metrics.returnedIdleRpm = returnedRpmSum / static_cast<double>(returnedStateSamples);
    if (initialAudioSamples != 0)
        metrics.initialIdleRms = std::sqrt(initialSquareSum
            / static_cast<double>(initialAudioSamples));
    if (returnedAudioSamples != 0)
        metrics.returnedIdleRms = std::sqrt(returnedSquareSum
            / static_cast<double>(returnedAudioSamples));
    metrics.scan = scanSignal(left);
    // Analyse the settled idle plateau (the last second before the throttle
    // ramp), which is where an undamped runner mode is most audible: there is
    // no broadband combustion energy to mask it.
    metrics.idleWindow = analyseWindow(
        std::vector<float>(left.begin(),
            left.begin() + static_cast<std::ptrdiff_t>(4.0 * audioRate)),
        static_cast<std::size_t>(3.0 * audioRate), audioRate);
    metrics.droppedEvents += renderer.droppedPendingEventCount();
    metrics.lateEvents = renderer.lateEventCount();
    metrics.levelLimitedSamples = renderer.levelLimitedSampleCount();
    metrics.minLevelGain = renderer.minObservedLevelGain();
    writeWav(outDir / "idle-start-rev-return.wav", left, right,
             static_cast<int>(audioRate));
    std::cout << std::left << std::setw(26) << config.name
              << " initial=" << std::fixed << std::setprecision(0)
              << metrics.initialIdleRpm << " rpm/" << std::setprecision(4)
              << metrics.initialIdleRms << " RMS"
              << " revPeak=" << std::setprecision(0) << metrics.revPeakRpm
              << " returned=" << metrics.returnedIdleRpm << " rpm/"
              << std::setprecision(4) << metrics.returnedIdleRms << " RMS"
              << " peak=" << metrics.scan.peak
              << " idleResonance=" << std::setprecision(1)
              << metrics.idleWindow.maxResonanceProminenceDb << "dB@"
              << std::setprecision(0) << metrics.idleWindow.maxResonanceFrequencyHz << "Hz"
              << " dropped=" << metrics.droppedEvents << '/'
              << metrics.droppedPressureSamples
              << " late=" << metrics.lateEvents
              << " levelLimited=" << metrics.levelLimitedSamples
              << " minLevelGain=" << metrics.minLevelGain
              << " observerPeak=" << renderer.maxObservedExhaustPressurePa() << " Pa"
              << '\n';
    return metrics;
}

bool validateIdleCycle(const IdleCycleMetrics& metrics,
                       const EngineConfig& config) {
    auto ok = true;
    const auto fail = [&ok](const std::string& reason) {
        std::cerr << "FAIL: true idle cycle: " << reason << '\n';
        ok = false;
    };
    if (!metrics.scan.finite || metrics.scan.peak > 1.00001)
        fail("non-finite or out-of-range audio");
    if (metrics.scan.nearFullScaleFraction > 0.002
            || metrics.scan.longestFlatTop > 8)
        fail("rev transient clips or spends too long near digital full scale");
    if (metrics.initialIdleRpm < config.idleRpm * 0.65
            || metrics.returnedIdleRpm < config.idleRpm * 0.65)
        fail("engine did not sustain idle before and after the rev");
    if (metrics.returnedIdleRpm > config.idleRpm * 1.60)
        fail("engine did not return to the idle-speed region");
    if (metrics.revPeakRpm < metrics.initialIdleRpm * 1.40)
        fail("throttle phase did not produce a meaningful rev");
    // Stated in SI for the same reason as the running-engine gate above: what
    // matters is that an idling engine radiates an audible sound pressure at
    // the observer, not that it hits a dBFS number the monitor calibration
    // could move on its own. 85 dB SPL at one metre is a quiet idle.
    const auto idleSplDb = [](double rms) {
        const auto pressurePa = rms * AcousticMonitorCalibration::sinePeakPressurePa(
            AcousticMonitorCalibration::defaultFullScaleSplDb);
        return 20.0 * std::log10(std::max(pressurePa, 1.0e-12)
            / AcousticMonitorCalibration::referenceRmsPressurePa);
    };
    if (idleSplDb(metrics.initialIdleRms) < 85.0
            || idleSplDb(metrics.returnedIdleRms) < 85.0)
        fail("idle radiates below 85 dB SPL at the one-metre observer");
    if (metrics.droppedEvents != 0 || metrics.droppedPressureSamples != 0
            || metrics.lateEvents != 0)
        fail("realtime telemetry or events were dropped/late");
    if (metrics.levelLimitedSamples != 0 || metrics.minLevelGain < 0.99999F)
        fail("safety leveler engaged during the idle cycle");
    return ok;
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

/**
 * Drive a real EngineRuntime exactly as the application wires it, and report
 * which audio path ends up producing sound.
 *
 * Every other measurement in this harness steps the simulator by hand and feeds
 * the queues itself. That is deterministic and good for spectra, but it is not
 * the wiring the user hears: the application constructs an EngineRuntime, lets
 * its own thread publish telemetry, and renders from the queues that thread
 * fills. A defect confined to that wiring -- telemetry never published, rate
 * limited away, or a boundary the runtime marks invalid -- would leave the
 * offline measurements looking healthy while the delivered application quietly
 * ran the legacy procedural path instead. This closes that gap.
 */
bool runtimePathCheck(const EngineConfig& baseConfig, const WavData& ir) {
    auto config = baseConfig;
    normaliseEngineConfig(config);
    auto runtime = std::make_unique<EngineRuntime>(config);
    auto renderer = std::make_unique<RealtimeEngineAudio>(
        runtime->audioEvents(), runtime->audioState(),
        &runtime->cylinderPressureSamples());
    if (!ir.samples.empty()) renderer->setImpulseResponse(ir.samples, ir.sampleRate, 0);

    constexpr double audioRate = 48'000.0;
    constexpr int blockSize = 256;
    renderer->prepare(audioRate, blockSize);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    runtime->setIgnitionEnabled(true);
    runtime->setStarterEngaged(true);
    runtime->start();

    juce::AudioBuffer<float> block(2, blockSize);
    double rendered = 0.0;
    double peak = 0.0;
    double squareSum = 0.0;
    std::size_t sampleCount = 0;
    const auto blockSeconds = static_cast<double>(blockSize) / audioRate;
    // Cost of the audio callback itself, against the wall-clock time the block
    // represents. This is the only number that says whether the delivered
    // renderer fits a real device callback: the offline spectra all pass on a
    // renderer that is twice too slow to run. Measured here rather than in the
    // hand-stepped renders because only this path has the runtime thread
    // competing for cores, which is what a user's machine actually does.
    std::vector<double> renderMicroseconds;
    renderMicroseconds.reserve(static_cast<std::size_t>(6.0 / blockSeconds) + 1U);
    // Pace against an absolute deadline, not by sleeping a block's worth per
    // iteration. A 256-sample block at 48 kHz is 5.3 ms, which is below the
    // granularity of a Windows sleep: sleeping per block would run the render
    // clock at a fraction of real time, starve the renderer against the
    // runtime's wall-clock telemetry, and produce a starvation artefact
    // belonging to this harness rather than to the application.
    const auto startTime = std::chrono::steady_clock::now();
    while (rendered < 6.0) {
        if (rendered > 1.5) runtime->setStarterEngaged(false);
        if (rendered > 3.0) runtime->setThrottle(0.5);
        block.clear();
        const auto renderBegin = std::chrono::steady_clock::now();
        renderer->render(block, 0, blockSize);
        const auto renderEnd = std::chrono::steady_clock::now();
        if (rendered > 1.5)
            renderMicroseconds.push_back(
                std::chrono::duration<double, std::micro>(renderEnd - renderBegin).count());
        for (int s = 0; s < blockSize; ++s) {
            const auto value = static_cast<double>(block.getSample(0, s));
            if (!std::isfinite(value)) continue;
            peak = std::max(peak, std::abs(value));
            if (rendered > 3.5) { squareSum += value * value; ++sampleCount; }
        }
        rendered += blockSeconds;
        const auto deadline = startTime + std::chrono::microseconds(
            static_cast<long long>(rendered * 1.0e6));
        // Sleep the bulk of the wait, then spin the remainder, so the render
        // clock tracks the runtime's clock to well under one block.
        const auto coarse = deadline - std::chrono::milliseconds(2);
        if (coarse > std::chrono::steady_clock::now())
            std::this_thread::sleep_until(coarse);
        while (std::chrono::steady_clock::now() < deadline) std::this_thread::yield();
    }
    runtime->stop();

    const auto rms = sampleCount != 0
        ? std::sqrt(squareSum / static_cast<double>(sampleCount)) : 0.0;
    // A device callback must return within the block period, every time; the
    // p95 is therefore the number that matters, not the mean.
    std::sort(renderMicroseconds.begin(), renderMicroseconds.end());
    const auto renderMean = renderMicroseconds.empty() ? 0.0
        : std::accumulate(renderMicroseconds.begin(), renderMicroseconds.end(), 0.0)
            / static_cast<double>(renderMicroseconds.size());
    const auto renderP95 = renderMicroseconds.empty() ? 0.0
        : renderMicroseconds[static_cast<std::size_t>(
            0.95 * static_cast<double>(renderMicroseconds.size() - 1U))];
    const auto blockBudgetMicroseconds = blockSeconds * 1.0e6;
    const auto physical = renderer->physicalExhaustActive();
    // Engine speed at the end of the run. A silent render means nothing until
    // it is known whether the engine was still turning: a stalled engine is a
    // physics defect, not an audio one.
    const auto finalRpm = runtime->audioState().rpm.load(std::memory_order_relaxed);
    std::cout << "  " << std::left << std::setw(26) << config.name
              << " finalRpm=" << std::fixed << std::setprecision(0) << finalRpm
              << " physical=" << (physical ? "yes" : "NO")
              << " legacySamples=" << renderer->legacyPathSampleCount()
              << " boundaryDropouts=" << renderer->invalidBoundarySampleCount()
              << " rms=" << std::fixed << std::setprecision(4) << rms
              << " peak=" << peak
              << " observerPeak=" << std::setprecision(1)
              << renderer->maxObservedExhaustPressurePa() << " Pa"
              << " droppedPressure=" << runtime->droppedPressureSampleCount()
              << '\n'
              << "  " << std::setw(26) << " "
              << " callback mean=" << std::setprecision(1) << renderMean << "us"
              << " p95=" << renderP95 << "us"
              << " max=" << (renderMicroseconds.empty() ? 0.0 : renderMicroseconds.back()) << "us"
              << " budget=" << blockBudgetMicroseconds << "us"
              << " load(p95)=" << std::setprecision(1)
              << renderP95 / blockBudgetMicroseconds * 100.0 << "%"
              // The other realtime thread. A comfortable audio callback proves
              // nothing on its own: if the 240 Hz physics loop misses its
              // deadline the telemetry stream stalls, and the exhaust chain --
              // which is driven only by that telemetry -- is what degrades.
              << " physicsOverruns=" << runtime->timingOverrunCount()
              << "/" << static_cast<std::uint64_t>(rendered * 240.0)
              << " maxLate=" << std::setprecision(2)
              << runtime->maximumTimingLatenessSeconds() * 1.0e3 << "ms\n";
    auto ok = physical;
    if (!physical)
        std::cerr << "FAIL: runtime wiring: " << config.name
                  << " never activated the physical exhaust path\n";
    // An engine that has stopped turning cannot produce engine sound, and no
    // amount of audio work will change that. This is checked here rather than in
    // the offline renders because those drive throttle and a dyno load
    // themselves and so hold the engine up artificially; only this path runs the
    // engine the way the application does, on its idle governor alone.
    if (!(finalRpm > config.idleRpm * 0.5)) {
        std::cerr << "FAIL: runtime wiring: " << config.name
                  << " stalled (final rpm " << std::fixed << std::setprecision(0)
                  << finalRpm << ", idle target " << config.idleRpm << ")\n";
        ok = false;
    }
    return ok;
}

int main(int argc, char** argv) {
    std::cout << std::unitbuf;
    std::filesystem::path outDir = "audio-render-output";
    std::filesystem::path irPath = std::filesystem::path(ENGINELAB_CATALOG_ROOT) / "assets" / "ir" / "exhaust_default.wav";
    bool idleOnly = false;
    std::string referenceFilter;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--output" && i + 1 < argc) outDir = argv[++i];
        else if (a == "--ir" && i + 1 < argc) irPath = argv[++i];
        else if (a == "--idle-only") idleOnly = true;
        else if (a == "--reference-filter" && i + 1 < argc)
            referenceFilter = argv[++i];
        else if (a == "--mute-combustion") muteCombustionLayer = true;
        else if (a == "--mute-mechanical") muteMechanicalLayer = true;
        else if (a == "--mute-intake") muteIntakeLayer = true;
        else if (a == "--audio-chunk" && i + 1 < argc)
            audioChunkSamples = std::clamp(std::atoi(argv[++i]), 1, 200);
    }
    std::filesystem::create_directories(outDir);

    WavData ir;
    if (loadWav(irPath, ir))
        std::cout << "Loaded IR: " << irPath.string() << " (" << ir.samples.size()
                  << " samples @ " << ir.sampleRate << " Hz)\n";
    else
        std::cout << "WARNING: could not load IR at " << irPath.string() << " (using renderer fallback)\n";

    if (!referenceFilter.empty()) {
        const auto catalog = loadEngineCatalog(
            std::filesystem::path(ENGINELAB_CATALOG_ROOT));
        const auto selected = std::find_if(catalog.entries.begin(), catalog.entries.end(),
            [&referenceFilter](const auto& entry) {
                return entry.config.name.find(referenceFilter) != std::string::npos;
            });
        if (selected == catalog.entries.end()) {
            std::cerr << "FAIL: no catalogue engine matches reference filter '"
                      << referenceFilter << "'\n";
            return 2;
        }
        referenceCouplingEverySubstep = true;
        std::cout << "\n--- Full-coupling offline reference (not realtime) ---\n";
        const auto metrics = renderEngine(
            selected->config, ir, outDir, 3.0, true);
        const auto valid = metrics.physicalActive
            && metrics.left.scan.finite && metrics.right.scan.finite
            && metrics.droppedPressureSamples == 0
            && metrics.invalidBoundarySamples == 0;
        return valid ? 0 : 1;
    }

    if (idleOnly) {
        const auto catalog = loadEngineCatalog(std::filesystem::path(ENGINELAB_CATALOG_ROOT));
        const auto idleEngine = std::find_if(catalog.entries.begin(), catalog.entries.end(),
            [](const auto& entry) {
                return entry.config.name.find("Big Twin") != std::string::npos;
            });
        if (idleEngine == catalog.entries.end()) return 2;
        const auto idleCycle = renderIdleCycle(idleEngine->config, ir, outDir);
        return validateIdleCycle(idleCycle, idleEngine->config) ? 0 : 1;
    }

    struct Named { std::string label; EngineConfig config; };
    std::vector<Named> engines {
        { "inline4", makeDefaultInlineFour() },
        { "v8",      makeDefaultV8() },
        { "inline2", makeDefaultInlineTwo() },
        { "radial5", makeDefaultRadialFive() },
    };

    std::cout << "\n--- Application wiring: real EngineRuntime path check ---\n";
    auto runtimePathOk = true;
    {
        const auto runtimeCatalog = loadEngineCatalog(
            std::filesystem::path(ENGINELAB_CATALOG_ROOT));
        // The first entries in catalogue order, plus the engine with the most
        // cylinders. Cost scales with cylinder count, so a fixed "first four"
        // window measured the cheap end of the catalogue and never the worst
        // case -- the check reported a comfortable callback load for engines
        // nobody was worried about.
        std::vector<std::size_t> selected;
        for (std::size_t index = 0; index < runtimeCatalog.entries.size() && index < 3; ++index)
            selected.push_back(index);
        const auto widest = std::max_element(runtimeCatalog.entries.begin(),
            runtimeCatalog.entries.end(), [](const auto& left, const auto& right) {
                return left.config.cylinders.size() < right.config.cylinders.size();
            });
        if (widest != runtimeCatalog.entries.end()) {
            const auto widestIndex = static_cast<std::size_t>(
                std::distance(runtimeCatalog.entries.begin(), widest));
            if (std::find(selected.begin(), selected.end(), widestIndex) == selected.end())
                selected.push_back(widestIndex);
        }
        const auto checked = selected.size();
        for (const auto index : selected)
            if (!runtimePathCheck(runtimeCatalog.entries[index].config, ir)) runtimePathOk = false;
        if (checked == 0) {
            std::cerr << "FAIL: no catalogue engines available for the runtime check\n";
            runtimePathOk = false;
        }
    }

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

    const auto catalog = loadEngineCatalog(std::filesystem::path(ENGINELAB_CATALOG_ROOT));
    const auto idleEngine = std::find_if(catalog.entries.begin(), catalog.entries.end(),
        [](const auto& entry) {
            return entry.config.name.find("Big Twin") != std::string::npos;
        });
    std::cout << "\n--- True idle start/rev/return (catalog Big Twin, 12 s) ---\n";
    const auto idleCycle = idleEngine != catalog.entries.end()
        ? renderIdleCycle(idleEngine->config, ir, outDir) : IdleCycleMetrics {};

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
            // Level is gated in SI, not in dBFS.
            //
            // The former gate was a fixed dBFS window (-27 to -10 dBFS RMS) for
            // every engine. That was appropriate when per-voice gains normalised
            // every engine to roughly the same loudness, but the physical path
            // has no such gain: a small twin and a big V8 radiate genuinely
            // different sound power, and forcing them into a common dBFS window
            // would mean re-introducing exactly the normalisation this path
            // exists to remove.
            //
            // A dBFS window is also unfalsifiable here, because the monitor
            // calibration alone can move it. Converting back to pascals through
            // that same calibration states the invariant where it is physical:
            // a running engine one metre from its tailpipe must radiate a
            // plausible sound pressure level. The bounds are wide, and taken
            // from what exhaust systems measure at one metre -- roughly 90 dB
            // for a quiet engine idling through a muffler up to about 130 dB
            // for open headers at power. Anything outside that is a modelling
            // error, and no choice of preamp gain can hide it.
            const auto rmsPressurePa = channel->window.rms
                * AcousticMonitorCalibration::sinePeakPressurePa(
                    AcousticMonitorCalibration::defaultFullScaleSplDb);
            const auto soundPressureLevelDb = 20.0 * std::log10(
                std::max(rmsPressurePa, 1.0e-12)
                / AcousticMonitorCalibration::referenceRmsPressurePa);
            if (soundPressureLevelDb < 90.0)
                fail("radiated level is below 90 dB SPL at the one-metre observer ("
                     + std::to_string(static_cast<int>(soundPressureLevelDb)) + " dB)" + suffix);
            if (soundPressureLevelDb > 130.0)
                fail("radiated level exceeds 130 dB SPL at the one-metre observer ("
                     + std::to_string(static_cast<int>(soundPressureLevelDb)) + " dB)" + suffix);
            // Digital safety is separate and still absolute.
            if (channel->window.rms > 0.40)
                fail("monitor level leaves too little headroom for transients" + suffix);
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
        // Channel correlation is reported, not gated.
        //
        // The former gate required the two channels to differ. That was a
        // meaningful check when the output was built from per-cylinder voices
        // carrying authored stereo pan values: a collapse to mono then meant the
        // panning had stopped working. Those pan values were a mixing decision,
        // not a measurement, and the voices they belonged to are gone.
        //
        // The physical path radiates every exhaust outlet to a single documented
        // observer point, because outlet positions are not part of the published
        // geometry. A mono result is therefore the correct output of the model
        // as it currently stands, and a gate demanding stereo would only be
        // satisfiable by inventing a pan -- which is precisely the kind of
        // decoration this path is meant to exclude. Genuine stereo needs outlet
        // positions and a two-microphone observer; until then this is a known
        // and documented limitation, not a regression to catch here.
        // The safety leveler must be safety-only in a shipped voice: if it is
        // pulling gain below identity here, the default level is set too hot and
        // the AGC is silently masking that offset. Keep the level honest instead.
        if (m.levelLimitedSamples != 0 || m.minLevelGain < 0.99999F)
            fail("safety leveler engaged at the default voice (AGC masking a level offset)");
    };
    for (std::size_t index = 0; index < metrics.size(); ++index)
        validate(metrics[index], engines[index].label, true);
    validate(turbo, "synthetic turbo", true);
    validate(stability, "long-run inline4", true);
    if (idleEngine == catalog.entries.end()) {
        std::cerr << "FAIL: catalog Big Twin fixture is missing\n";
        ok = false;
    } else if (!validateIdleCycle(idleCycle, idleEngine->config)) ok = false;
    if (worstSimilarity > 0.985) {
        std::cerr << "FAIL: engines are not spectrally differentiated (max similarity "
                  << worstSimilarity << ")\n";
        ok = false;
    }
    if (!decayMeasurable) {
        std::cerr << "FAIL: exhaust decay could not be measured for at least one preset\n";
        ok = false;
    }
    // A physical path that never activates under the application's own wiring is
    // a silent downgrade to the legacy procedural voice, which is exactly the
    // fallback this architecture is not allowed to have.
    if (!runtimePathOk) ok = false;
    std::cout << "\nResult: " << (ok ? "PASS" : "FAIL")
              << " (max spectral similarity " << std::setprecision(3) << worstSimilarity << ")\n";
    return ok ? 0 : 1;
}
