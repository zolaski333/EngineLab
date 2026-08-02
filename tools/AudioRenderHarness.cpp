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
//  - measureExhaustDecay(): drives one SI valve-boundary impulse through a quiet
//    compiled graph and reports the reverberation time of the physical chain.

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
#include <limits>
#include <memory>
#include <numbers>
#include <numeric>
#include <optional>
#include <stdexcept>
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
// Focused offline export only. The application master remains stereo; this
// asks RealtimeEngineAudio to observe its pre-master diagnostic buses.
bool writeDiagnosticStems = false;
// Same-binary A/B only. Production keeps the physically driven outlet source
// enabled; the switch proves exactly what that one layer contributes.
bool enableExhaustJetNoise = true;
// Same-binary null for the V/flat structural coordinate correction. Production
// follows each bank's explicit cylinder_ids ordering.
bool enableStructuralBankTopologyParticipation = true;
// Same-binary null for the broadband FI filter-power correction.
bool enableForcedInductionBroadbandPowerNormalisation = true;
bool retainRenderedAudioForComparison = false;
// Offline oracle only. Large engines are not expected to meet realtime when the
// complete nonlinear network is advanced on every mechanical substep.
bool referenceCouplingEverySubstep = false;
// Harness-only simulator policy. Empty means the production defaults compiled
// into EngineSimulator; selected comparison modes override one dimension while
// leaving the rest of the delivered path unchanged.
EngineSimulatorOptions renderSimulatorOptions;
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
    if (!path.parent_path().empty())
        std::filesystem::create_directories(path.parent_path());
    std::ofstream out(path, std::ios::binary);
    if (!out)
        throw std::runtime_error("could not create WAV proof: " + path.string());
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
    out.flush();
    if (!out)
        throw std::runtime_error("could not finish WAV proof: " + path.string());
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

// A control transition is allowed to change the sound, but it must not create
// a one-sample discontinuity. Compare the largest adjacent-sample step in a
// narrow window around each commanded transition with the 99.9th percentile of
// the same signal away from those windows. This keeps genuine combustion
// impulses in the reference population while making a control-boundary click
// stand out as an isolated outlier.
struct TransitionStepScan final {
    double maximumTransitionStep {};
    double backgroundStepP999 {};
    double transitionToBackgroundRatio {};
    bool measurable { false };
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
    // The same measurement restricted to above the coupling Nyquist. Reported,
    // deliberately not gated -- neither this nor the figure above can carry a
    // pass/fail on its own, and it is worth being explicit about why, because
    // the obvious gate is wrong in both directions.
    //
    // Below the coupling Nyquist a tall narrow peak is usually just the engine.
    // A strong second or third firing order stands as proud as any mode; the
    // catalogue reads 25-40 dB there on content that is exactly what the engine
    // should sound like.
    //
    // Above it the tempting argument is that the boundary carried no
    // information, so nothing there can be a mode. That is true of the
    // *boundary* and false of the *network*: the waveguide's own modes do not
    // stop at the coupling Nyquist, and the complementary valve-flow source
    // excites them. Measured on the reference inline four, muting that source
    // took the 4107 Hz peak from 31.2 dB to 22.1 dB -- so it is neither purely
    // an image nor purely a mode. A default-geometry fixture with no muffler
    // chamber is also a straight-through open header, which genuinely rings.
    //
    // What can be gated is the filter that is supposed to suppress the image
    // component, and that is asserted analytically in the boundary
    // reconstruction regression rather than read back off a render.
    double maxOutOfBandResonanceProminenceDb {};
    double maxOutOfBandResonanceFrequencyHz {};
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
    float maxPreLimiterMagnitude {};
    float maxExhaustPressurePa {};
    float maxExhaustJetNoisePressurePa {};
    float maxIntakePressurePa {};
    float maxStructuralPressurePa {};
    float maxForcedInductionPressurePa {};
    AcousticIntakeNetwork::Diagnostics intakeDiagnostics {};
    double observerDistanceM { 1.0 };
    // Which path produced the audio. A physical path that never activated would
    // otherwise be reported as an unchanged-sounding physical path.
    bool physicalActive { false };
    bool compiledTopologyActive { false };
    bool structuralRadiationActive { false };
    bool intakeTopologyActive { false };
    bool forcedInductionAcousticsActive { false };
    std::uint64_t legacyPathSamples {};
    std::uint64_t invalidBoundarySamples {};
    double maximumRpm {};
    double limiterEntrySeconds { -1.0 };
    double minimumRpmAfterLimiter {};
    double maximumBoostPressureRatio { 1.0 };
    double preLiftBoostPressureRatio { 1.0 };
    double maximumBlowOffMassFlowKgPerSecond {};
    TransitionStepScan commandedTransitionSteps {};
    // Populated only by focused same-binary comparisons. Normal catalogue runs
    // retain scalar fingerprints and do not keep every rendered sample.
    std::vector<float> comparisonAudioLeft;
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

TransitionStepScan scanTransitionSteps(
    const std::vector<float>& x, double sampleRate,
    const std::vector<double>& transitionTimesSeconds) {
    TransitionStepScan result;
    if (x.size() < 2 || !(sampleRate > 0.0)
        || transitionTimesSeconds.empty()) {
        return result;
    }

    constexpr double transitionHalfWindowSeconds = 0.010;
    std::vector<double> backgroundSteps;
    backgroundSteps.reserve(x.size());
    for (std::size_t index = 1; index < x.size(); ++index) {
        const auto current = static_cast<double>(x[index]);
        const auto previous = static_cast<double>(x[index - 1]);
        if (!std::isfinite(current) || !std::isfinite(previous)) continue;
        const auto timeSeconds = static_cast<double>(index) / sampleRate;
        const auto nearTransition = std::any_of(
            transitionTimesSeconds.begin(), transitionTimesSeconds.end(),
            [timeSeconds](double transitionTimeSeconds) {
                return std::abs(timeSeconds - transitionTimeSeconds)
                    <= transitionHalfWindowSeconds;
            });
        const auto step = std::abs(current - previous);
        if (nearTransition)
            result.maximumTransitionStep =
                std::max(result.maximumTransitionStep, step);
        else
            backgroundSteps.push_back(step);
    }
    if (backgroundSteps.empty()) return result;

    const auto percentileIndex = std::min(
        backgroundSteps.size() - 1,
        static_cast<std::size_t>(std::floor(
            0.999 * static_cast<double>(backgroundSteps.size() - 1))));
    std::nth_element(
        backgroundSteps.begin(),
        backgroundSteps.begin() + static_cast<std::ptrdiff_t>(percentileIndex),
        backgroundSteps.end());
    result.backgroundStepP999 = backgroundSteps[percentileIndex];
    result.transitionToBackgroundRatio = result.maximumTransitionStep
        / std::max(1.0e-9, result.backgroundStepP999);
    result.measurable = result.maximumTransitionStep > 0.0
        && result.backgroundStepP999 > 0.0;
    return result;
}

// Analyse the final steady-state window. The FFT is used both for broad energy
// balance and for a log-band engine fingerprint; this is less phase-sensitive
// than probing a handful of individual DFT frequencies.
WindowAnalysis analyseWindow(const std::vector<float>& x, std::size_t begin,
                             double sampleRate, double outOfBandFloorHz = 0.0) {
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
            const auto frequencyHz = static_cast<double>(bin) * binHz;
            if (prominenceDb > m.maxResonanceProminenceDb) {
                m.maxResonanceProminenceDb = prominenceDb;
                m.maxResonanceFrequencyHz = frequencyHz;
            }
            if (outOfBandFloorHz > 0.0 && frequencyHz > outOfBandFloorHz
                && prominenceDb > m.maxOutOfBandResonanceProminenceDb) {
                m.maxOutOfBandResonanceProminenceDb = prominenceDb;
                m.maxOutOfBandResonanceFrequencyHz = frequencyHz;
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
    bool syntheticTurbo = false, bool limiterRun = false, bool liftRun = false) {
    auto config = baseConfig;
    if (syntheticTurbo) {
        config.name += " Synthetic Turbo";
        config.forcedInduction.enabled = true;
        config.forcedInduction.type = ForcedInductionType::turbocharger;
        config.forcedInduction.pressureRatio = 1.8;
        config.forcedInduction.compressorBladeCount = 6;
        config.forcedInduction.turbineBladeCount = 9;
        config.forcedInduction.compressorInducerDiameterMm = 48.0;
        config.forcedInduction.turbineExducerDiameterMm = 42.0;
        config.forcedInduction.blowOffValveFlowAreaMm2 = 350.0;
    }
    normaliseEngineConfig(config);
    SimpleEcuModel ecu; SimplifiedGasolinePhysics physics; FourStrokeEventGenerator events;
    auto exhaust = ExhaustGraph::makeForEngine(config);
    auto simulatorPtr = std::make_unique<EngineSimulator>(
        config, ecu, physics, events, exhaust, renderSimulatorOptions);
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
    auto rendererPtr = std::make_unique<RealtimeEngineAudio>(
        eventQueue, audioState, &pressureQueue, &audioConfiguration->exhaustGraph(),
        &audioConfiguration->engineConfig());
    auto& renderer = *rendererPtr;
    renderer.setOutletJetNoiseEnabled(enableExhaustJetNoise);
    renderer.setStructuralBankTopologyParticipationEnabled(
        enableStructuralBankTopologyParticipation);
    renderer.setForcedInductionBroadbandPowerNormalisationEnabled(
        enableForcedInductionBroadbandPowerNormalisation);
    if (!ir.samples.empty()) renderer.setImpulseResponse(ir.samples, ir.sampleRate, 0);
    renderer.prepare(audioRate, samplesPerStep);
    // Let the convolver's background IR load settle before rendering.
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    std::vector<float> audioLeft, audioRight;
    audioLeft.reserve(static_cast<std::size_t>(seconds * audioRate));
    audioRight.reserve(static_cast<std::size_t>(seconds * audioRate));
    juce::AudioBuffer<float> block(2, samplesPerStep);
    constexpr std::size_t diagnosticStemCount = 6;
    std::array<juce::AudioBuffer<float>, diagnosticStemCount> stemBlocks;
    std::array<std::array<std::vector<float>, 2>, diagnosticStemCount> stemAudio;
    if (writeDiagnosticStems) {
        for (auto& stem : stemBlocks)
            stem.setSize(2, samplesPerStep, false, true, false);
        for (auto& stem : stemAudio)
            for (auto& channel : stem)
                channel.reserve(static_cast<std::size_t>(seconds * audioRate));
    }
    const RealtimeAudioStemBuffers stemBuffers {
        &stemBlocks[0], &stemBlocks[1], &stemBlocks[2],
        &stemBlocks[3], &stemBlocks[4], &stemBlocks[5]
    };
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
    double maximumRpm = 0.0;
    double limiterEntrySeconds = -1.0;
    double minimumRpmAfterLimiter = std::numeric_limits<double>::infinity();
    double maximumBoostPressureRatio = 1.0;
    double preLiftBoostPressureRatio = 1.0;
    double maximumBlowOffMassFlowKgPerSecond = 0.0;
    const auto dynoTargetRpm = std::max(config.idleRpm * 1.50, config.redlineRpm * 0.55);
    const auto steps = static_cast<std::size_t>(seconds / dt);
    for (std::size_t step = 0; step < steps; ++step) {
        const auto t = static_cast<double>(step) * dt;
        EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = t < 1.1;
        if (limiterRun) {
            controls.throttle = t < 0.9 ? 0.18
                : std::clamp(
                    0.18 + (t - 0.9) / 0.65 * 0.82, 0.18, 1.0);
        } else {
            const auto performLift = syntheticTurbo || liftRun;
            controls.throttle = t < 0.9 ? 0.2
                : (performLift && t >= 2.4 && t < 2.65 ? 0.08 : 0.72);
        }
        if (t >= 1.2 && !limiterRun) {
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
        maximumRpm = std::max(maximumRpm, frame.state.rpm);
        maximumBoostPressureRatio = std::max(
            maximumBoostPressureRatio, frame.state.boostPressureRatio);
        maximumBlowOffMassFlowKgPerSecond = std::max(
            maximumBlowOffMassFlowKgPerSecond,
            frame.state.blowOffMassFlowKgPerSecond);
        if (t >= 2.0 && t < 2.4)
            preLiftBoostPressureRatio = std::max(
                preLiftBoostPressureRatio, frame.state.boostPressureRatio);
        if (limiterRun && limiterEntrySeconds < 0.0
            // The ECU begins its alternating soft cut 220 rpm below the hard
            // latch. Measuring that first cut is the audible limiter entry;
            // requiring the hard latch would reject a correctly effective
            // soft limiter precisely because it prevented the overshoot.
            && frame.state.rpm >= config.ignition.revLimitRpm - 210.0) {
            limiterEntrySeconds = t;
        }
        if (limiterEntrySeconds >= 0.0 && t > limiterEntrySeconds + 0.02)
            minimumRpmAfterLimiter = std::min(
                minimumRpmAfterLimiter, frame.state.rpm);
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
        block.clear();
        // Block-size invariance probe: a correct realtime renderer must produce
        // the same signal whatever callback size the host chooses. Rendering
        // each simulation step in smaller chunks moves any per-block parameter
        // step to a different comb frequency (sampleRate / chunk), which
        // separates render-block artefacts from simulation-frame artefacts.
        for (int offset = 0; offset < samplesPerStep; offset += audioChunkSamples) {
            const auto chunk = std::min(audioChunkSamples, samplesPerStep - offset);
            if (writeDiagnosticStems)
                renderer.renderWithStems(block, offset, chunk, stemBuffers);
            else
                renderer.render(block, offset, chunk);
        }
        for (int s = 0; s < samplesPerStep; ++s) {
            audioLeft.push_back(block.getSample(0, s));
            audioRight.push_back(block.getSample(1, s));
            if (writeDiagnosticStems) {
                for (std::size_t stem = 0; stem < diagnosticStemCount; ++stem) {
                    stemAudio[stem][0].push_back(stemBlocks[stem].getSample(0, s));
                    stemAudio[stem][1].push_back(stemBlocks[stem].getSample(1, s));
                }
            }
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
    const auto couplingNyquistHz = simulator.state().exhaustCouplingFrequencyHz * 0.5;
    m.left.window = analyseWindow(audioLeft, analysisBegin, audioRate, couplingNyquistHz);
    m.right.window = analyseWindow(audioRight, analysisBegin, audioRate, couplingNyquistHz);
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
    m.maxPreLimiterMagnitude = renderer.maxPreLimiterMagnitude();
    m.maxExhaustPressurePa = renderer.maxObservedExhaustPressurePa();
    m.maxExhaustJetNoisePressurePa =
        renderer.maxObservedExhaustJetNoisePressurePa();
    m.maxIntakePressurePa = renderer.maxObservedIntakePressurePa();
    m.maxStructuralPressurePa = renderer.maxObservedStructuralPressurePa();
    m.maxForcedInductionPressurePa =
        renderer.maxObservedForcedInductionPressurePa();
    m.intakeDiagnostics = renderer.intakeNetworkDiagnostics();
    const auto microphoneDistance = [](const AcousticPoint3M& point) {
        return std::sqrt(point.x * point.x + point.y * point.y
            + point.z * point.z);
    };
    // The SPL guard back-extrapolates the observed pressure to one metre by
    // 1/r, so it must use the radius the render used, not the authored one.
    (void)microphoneDistance;
    m.observerDistanceM =
        effectiveObserverDistanceM(config.acousticObserver);
    m.physicalActive = renderer.physicalExhaustActive();
    m.compiledTopologyActive = renderer.compiledExhaustTopologyActive();
    m.structuralRadiationActive = renderer.structuralRadiationActive();
    m.intakeTopologyActive = renderer.compiledIntakeTopologyActive();
    m.forcedInductionAcousticsActive = renderer.forcedInductionAcousticsActive();
    m.legacyPathSamples = renderer.legacyPathSampleCount();
    m.invalidBoundarySamples = renderer.invalidBoundarySampleCount();
    m.maximumRpm = maximumRpm;
    m.limiterEntrySeconds = limiterEntrySeconds;
    m.minimumRpmAfterLimiter = std::isfinite(minimumRpmAfterLimiter)
        ? minimumRpmAfterLimiter : maximumRpm;
    m.maximumBoostPressureRatio = maximumBoostPressureRatio;
    m.preLiftBoostPressureRatio = preLiftBoostPressureRatio;
    m.maximumBlowOffMassFlowKgPerSecond =
        maximumBlowOffMassFlowKgPerSecond;
    if (retainRenderedAudioForComparison)
        m.comparisonAudioLeft = audioLeft;
    if (limiterRun && limiterEntrySeconds >= 0.0) {
        m.commandedTransitionSteps = scanTransitionSteps(
            audioLeft, audioRate, { limiterEntrySeconds });
    } else if (syntheticTurbo || liftRun) {
        m.commandedTransitionSteps = scanTransitionSteps(
            audioLeft, audioRate, { 2.4, 2.65 });
    }
    if (writeOutput) {
        writeWav(outDir / (config.name + ".wav"), audioLeft, audioRight, static_cast<int>(audioRate));
        if (writeDiagnosticStems) {
            constexpr std::array<const char*, diagnosticStemCount> names {
                "01-combustion", "02-exhaust-dry", "03-exhaust-ir",
                "04-intake", "05-forced-induction", "06-mechanical"
            };
            const auto stemDirectory = outDir / (config.name + "-stems");
            for (std::size_t stem = 0; stem < diagnosticStemCount; ++stem) {
                writeWav(stemDirectory / (std::string(names[stem]) + ".wav"),
                    stemAudio[stem][0], stemAudio[stem][1],
                    static_cast<int>(audioRate));
                const auto safety = scanSignal(stemAudio[stem][0]);
                const auto squareSum = std::inner_product(
                    stemAudio[stem][0].begin(), stemAudio[stem][0].end(),
                    stemAudio[stem][0].begin(), 0.0);
                const auto rms = std::sqrt(squareSum / static_cast<double>(
                    std::max<std::size_t>(1, stemAudio[stem][0].size())));
                std::cout << "    " << std::left << std::setw(20) << names[stem]
                          << " rms=" << std::fixed << std::setprecision(6) << rms
                          << " peak=" << safety.peak << '\n';
            }
            std::cout << "  diagnostic stems: " << stemDirectory.string()
                      << " (pre-master source buses)\n";
        }
    }
    std::cout << std::left << std::setw(26) << config.name
              << " rms="   << std::fixed << std::setprecision(4) << m.left.window.rms
              << '/' << m.right.window.rms
              << " peak="  << m.left.scan.peak << '/' << m.right.scan.peak
              << " crest=" << std::setprecision(2) << m.left.window.crest
              << " rpm=" << std::setprecision(0) << m.finalRpm
              << " brightness=" << std::setprecision(3) << m.left.window.brightness
              << " dc=" << std::showpos << m.left.window.mean << std::noshowpos
              << " physical=" << (m.physicalActive ? "yes" : "NO")
              << " topology=" << (m.compiledTopologyActive ? "full" : "LEGACY")
              << " structure=" << (m.structuralRadiationActive ? "modal" : "LEGACY")
              << " intake=" << (m.intakeTopologyActive ? "wave" : "LEGACY")
              << " forced=" << (m.forcedInductionAcousticsActive ? "semi-empirical" : "none")
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
              << " outOfBandResonance=" << std::setprecision(1)
              << m.left.window.maxOutOfBandResonanceProminenceDb
              << "dB@" << std::setprecision(0)
              << m.left.window.maxOutOfBandResonanceFrequencyHz << "Hz"
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
              << " preLimiter=" << m.maxPreLimiterMagnitude
              << " maxRpm=" << std::setprecision(0) << m.maximumRpm
              << " boost(pre/max)=" << std::setprecision(3)
              << m.preLiftBoostPressureRatio << '/'
              << m.maximumBoostPressureRatio
              << " bovKgS=" << std::setprecision(5)
              << m.maximumBlowOffMassFlowKgPerSecond
              << " transitionStep=" << std::setprecision(5)
              << m.commandedTransitionSteps.maximumTransitionStep
              << "/p999=" << m.commandedTransitionSteps.backgroundStepP999
              << "/ratio=" << std::setprecision(2)
              << m.commandedTransitionSteps.transitionToBackgroundRatio
              << " layerPa=" << std::setprecision(1) << m.maxExhaustPressurePa
              << '/' << m.maxIntakePressurePa << '/' << m.maxStructuralPressurePa
              << " fiPa=" << std::setprecision(4)
              << m.maxForcedInductionPressurePa
              << " jetPa=" << std::setprecision(4)
              << m.maxExhaustJetNoisePressurePa
              << " intakeStagesPa=" << m.intakeDiagnostics.sourcePressurePa
              << '/' << m.intakeDiagnostics.runnerPressurePa
              << '/' << m.intakeDiagnostics.plenumPressurePa
              << '/' << m.intakeDiagnostics.airboxPressurePa
              << '/' << m.intakeDiagnostics.mouthPressurePa
              << '/' << m.intakeDiagnostics.radiatedPressurePa
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
    TransitionStepScan commandedTransitionSteps {};
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
        eventQueue, audioState, &pressureQueue, &audioConfiguration->exhaustGraph(),
        &audioConfiguration->engineConfig());
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
        static_cast<std::size_t>(3.0 * audioRate), audioRate,
        simulator.state().exhaustCouplingFrequencyHz * 0.5);
    metrics.droppedEvents += renderer.droppedPendingEventCount();
    metrics.lateEvents = renderer.lateEventCount();
    metrics.levelLimitedSamples = renderer.levelLimitedSampleCount();
    metrics.minLevelGain = renderer.minObservedLevelGain();
    metrics.commandedTransitionSteps = scanTransitionSteps(
        left, audioRate, { 1.5, 4.0, 4.4, 4.7, 5.0 });
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
              << " transitionStep=" << std::setprecision(5)
              << metrics.commandedTransitionSteps.maximumTransitionStep
              << "/p999="
              << metrics.commandedTransitionSteps.backgroundStepP999
              << "/ratio="
              << metrics.commandedTransitionSteps.transitionToBackgroundRatio
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
    // could move on its own. 85 dB SPL at the published microphones is a quiet
    // but still clearly measurable idle.
    const auto idleSplDb = [](double rms) {
        const auto pressurePa = rms * AcousticMonitorCalibration::sinePeakPressurePa(
            AcousticMonitorCalibration::defaultFullScaleSplDb);
        return 20.0 * std::log10(std::max(pressurePa, 1.0e-12)
            / AcousticMonitorCalibration::referenceRmsPressurePa);
    };
    if (idleSplDb(metrics.initialIdleRms) < 85.0
            || idleSplDb(metrics.returnedIdleRms) < 85.0)
        fail("idle radiates below 85 dB SPL at the published observer");
    if (metrics.droppedEvents != 0 || metrics.droppedPressureSamples != 0
            || metrics.lateEvents != 0)
        fail("realtime telemetry or events were dropped/late");
    if (metrics.levelLimitedSamples != 0 || metrics.minLevelGain < 0.99999F)
        fail("safety leveler engaged during the idle cycle");
    if (!metrics.commandedTransitionSteps.measurable)
        fail("control-boundary discontinuity scan was not measurable");
    else if (metrics.commandedTransitionSteps.maximumTransitionStep > 0.50
        || metrics.commandedTransitionSteps.transitionToBackgroundRatio > 8.0)
        fail("starter or throttle transition produced an isolated audio step");
    return ok;
}

bool validateBoostLiftTransient(
    const Metrics& metrics, const EngineConfig& config) {
    auto ok = true;
    const auto fail = [&ok, &config](const std::string& reason) {
        std::cerr << "FAIL: boost lift transient (" << config.name
                  << "): " << reason << '\n';
        ok = false;
    };
    if (!metrics.left.scan.finite || !metrics.right.scan.finite
        || metrics.left.scan.peak > 1.00001
        || metrics.right.scan.peak > 1.00001) {
        fail("non-finite or out-of-range audio");
    }
    if (metrics.left.scan.nearFullScaleFraction > 0.002
        || metrics.left.scan.longestFlatTop > 8) {
        fail("lift transient clips or forms a flat-top plateau");
    }
    if (!metrics.physicalActive || !metrics.compiledTopologyActive
        || !metrics.structuralRadiationActive
        || !metrics.intakeTopologyActive
        || !metrics.forcedInductionAcousticsActive) {
        fail("one or more production physical audio layers were inactive");
    }
    if (metrics.preLiftBoostPressureRatio <= 1.01)
        fail("throttle lift happened without measurable pre-lift boost");
    if (metrics.maximumBlowOffMassFlowKgPerSecond <= 1.0e-5)
        fail("the physical blow-off valve never flowed during the lift");
    if (!metrics.commandedTransitionSteps.measurable)
        fail("lift-boundary discontinuity scan was not measurable");
    else if (metrics.commandedTransitionSteps.maximumTransitionStep > 0.50
        || metrics.commandedTransitionSteps.transitionToBackgroundRatio > 8.0)
        fail("throttle lift or recovery produced an isolated audio step");
    if (metrics.droppedEvents != 0 || metrics.droppedPressureSamples != 0
        || metrics.lateEvents != 0 || metrics.stolenVoices != 0
        || metrics.invalidBoundarySamples != 0) {
        fail("realtime telemetry, voices, or physical boundary samples were lost");
    }
    if (metrics.levelLimitedSamples != 0
        || metrics.minLevelGain < 0.99999F
        || metrics.maxPreLimiterMagnitude >= 0.82F) {
        fail("a downstream safety processor masked the transient");
    }
    return ok;
}

bool validateLimiterTransient(
    const Metrics& metrics, const EngineConfig& config) {
    auto ok = true;
    const auto fail = [&ok, &config](const std::string& reason) {
        std::cerr << "FAIL: rev-limiter transient (" << config.name
                  << "): " << reason << '\n';
        ok = false;
    };
    if (!metrics.left.scan.finite || !metrics.right.scan.finite
        || metrics.left.scan.peak > 1.00001
        || metrics.right.scan.peak > 1.00001) {
        fail("non-finite or out-of-range audio");
    }
    if (metrics.limiterEntrySeconds < 0.0
        || metrics.maximumRpm < config.ignition.revLimitRpm - 220.0) {
        fail("free rev never reached the configured ECU limiter");
    }
    if (metrics.maximumRpm > config.ignition.revLimitRpm * 1.08)
        fail("engine overshot the configured limiter by more than 8%");
    if (metrics.maximumRpm - metrics.minimumRpmAfterLimiter < 60.0)
        fail("limiter did not produce a resolved cut-and-release cycle");
    if (!metrics.commandedTransitionSteps.measurable)
        fail("limiter-entry discontinuity scan was not measurable");
    else if (metrics.commandedTransitionSteps.maximumTransitionStep > 0.50
        || metrics.commandedTransitionSteps.transitionToBackgroundRatio > 8.0)
        fail("limiter entry produced an isolated audio step");
    if (metrics.droppedEvents != 0 || metrics.droppedPressureSamples != 0
        || metrics.lateEvents != 0 || metrics.stolenVoices != 0
        || metrics.invalidBoundarySamples != 0) {
        fail("realtime telemetry, voices, or physical boundary samples were lost");
    }
    if (metrics.levelLimitedSamples != 0
        || metrics.minLevelGain < 0.99999F
        || metrics.maxPreLimiterMagnitude >= 0.82F) {
        fail("a downstream safety processor masked the limiter transient");
    }
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
 * measure the compiled exhaust chain's decay.
 *
 * The impulse is injected as cylinder-pressure telemetry rather than as a firing
 * event on purpose. Firing-event voices are summed straight onto the exhaust bus
 * and into the per-path IR; they never enter the collector. Only the pressure
 * stream reaches the runner waveguide, collector junction and outlet reflection
 * line, so only a boundary-driven impulse can measure them.
 *
 * Everything but the exhaust bus is muted and the engine is held at rest, so no
 * continuous source (jet, induction, mechanical, high-frequency noise) adds a
 * floor that would flatten the Schroeder integral. The impulse amplitude keeps
 * the safety leveler and the master soft-limiter at identity, so the measured
 * decay is the acoustic model's own and not a compressor's release.
 *
 * convolutionMix selects what is measured: at 0 the explicit downstream IR is
 * muted and the decay is the compiled graph's own; at 1 the supplied measured
 * environment is included.
 */
DecayMeasurement measureExhaustDecay(const EngineConfig& baseConfig, const WavData& ir,
                                     double seconds, float convolutionMix) {
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
            const auto pulseKpa = pulseAt(nextPressureTime);
            sample.exhaustRunnerPressureKpa[0] = ambientKpa + pulseKpa;
            // This diagnostic predates the SI boundary contract. Supplying only
            // a legacy runner pressure now correctly produces silence because a
            // compiled graph never falls back to the procedural path. Drive a
            // finite, self-contained valve boundary instead: pressure and flow
            // are sampled together and the flow vanishes exactly outside the
            // raised-cosine pulse.
            const auto pulseFraction = pulseKpa / pulseAmplitudeKpa;
            const auto massFlowKgPerSecond = 0.12F * pulseFraction;
            sample.exhaustMassFlowKgPerSecond[0] = massFlowKgPerSecond;
            sample.exhaustAcousticMassFlowKgPerSecond[0] = massFlowKgPerSecond;
            sample.exhaustPortDensityKgPerM3[0] = 0.65F;
            sample.exhaustPortSpeedOfSoundMps[0] = 540.0F;
            sample.exhaustValveConductanceAreaM2[0] = pulseFraction > 0.0F
                ? 0.00045F : 0.0F;
            sample.thermoacousticBoundaryValid[0] = 1;
            sample.exhaustFlowMgPerCycle[0] = 60.0F;
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
bool runtimePathCheck(const EngineConfig& baseConfig, const WavData& ir,
                      std::optional<std::size_t> intakeWorkers = {},
                      std::optional<double> intakeWallHeatUpdateSeconds = {}) {
    auto config = baseConfig;
    normaliseEngineConfig(config);
    EngineSimulatorOptions simulatorOptions;
    simulatorOptions.intakeWorkerCount = intakeWorkers;
    simulatorOptions.intakeWallHeatUpdateIntervalSeconds =
        intakeWallHeatUpdateSeconds;
    auto runtime = std::make_unique<EngineRuntime>(
        config, nullptr, simulatorOptions);
    if (intakeWallHeatUpdateSeconds.has_value())
        runtime->setRealtimeLoadProtectionEnabled(false);
    auto renderer = std::make_unique<RealtimeEngineAudio>(
        runtime->audioEvents(), runtime->audioState(),
        &runtime->cylinderPressureSamples(), &runtime->exhaustGraph(),
        &runtime->engineConfig());
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
    const auto fullTopology = renderer->compiledExhaustTopologyActive();
    const auto modalStructure = renderer->structuralRadiationActive();
    const auto intakeTopology = renderer->compiledIntakeTopologyActive();
    // Engine speed at the end of the run. A silent render means nothing until
    // it is known whether the engine was still turning: a stalled engine is a
    // physics defect, not an audio one.
    const auto finalRpm = runtime->audioState().rpm.load(std::memory_order_relaxed);
    std::cout << "  " << std::left << std::setw(26) << config.name
              << " finalRpm=" << std::fixed << std::setprecision(0) << finalRpm
              << " intakeWorkers=" << runtime->intakeWorkerCount()
              << " physical=" << (physical ? "yes" : "NO")
              << " topology=" << (fullTopology ? "full" : "LEGACY")
              << " structure=" << (modalStructure ? "modal" : "LEGACY")
              << " intake=" << (intakeTopology ? "wave" : "LEGACY")
              << " legacySamples=" << renderer->legacyPathSampleCount()
              << " boundaryDropouts=" << renderer->invalidBoundarySampleCount()
              << " rms=" << std::fixed << std::setprecision(4) << rms
              << " peak=" << peak
              << " observerPeak=" << std::setprecision(1)
              << renderer->maxObservedExhaustPressurePa() << " Pa"
              << " intakePeak=" << renderer->maxObservedIntakePressurePa() << " Pa"
              << " structurePeak=" << renderer->maxObservedStructuralPressurePa() << " Pa"
              << " preLimiter=" << std::setprecision(3)
              << renderer->maxPreLimiterMagnitude()
              << " levelLimited=" << renderer->levelLimitedSampleCount()
              << " minLevelGain=" << renderer->minObservedLevelGain()
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
              << runtime->maximumTimingLatenessSeconds() * 1.0e3 << "ms"
              << " protection="
              << (runtime->realtimeLoadProtectionActive() ? "active" : "normal")
              << " protectionActivations="
              << runtime->realtimeLoadProtectionActivationCount() << '\n';
    auto ok = physical && fullTopology && modalStructure && intakeTopology;
    if (!physical)
        std::cerr << "FAIL: runtime wiring: " << config.name
                  << " never activated the physical exhaust path\n";
    if (!fullTopology)
        std::cerr << "FAIL: runtime wiring: " << config.name
                  << " did not compile the complete exhaust topology\n";
    if (!modalStructure)
        std::cerr << "FAIL: runtime wiring: " << config.name
                  << " did not compile modal structural radiation\n";
    if (!intakeTopology)
        std::cerr << "FAIL: runtime wiring: " << config.name
                  << " did not compile the intake wave network\n";
    if (renderer->legacyPathSampleCount() != 0) {
        std::cerr << "FAIL: runtime wiring: " << config.name
                  << " exposed the procedural compatibility path for "
                  << renderer->legacyPathSampleCount() << " samples\n";
        ok = false;
    }
    if (renderer->levelLimitedSampleCount() != 0
        || renderer->minObservedLevelGain() < 0.99999F
        || renderer->maxPreLimiterMagnitude() >= 0.82F) {
        std::cerr << "FAIL: runtime wiring: " << config.name
                  << " drives the safety limiter; physical layer calibration is invalid\n";
        ok = false;
    }
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
    bool transientOnly = false;
    std::string referenceFilter;
    std::string catalogueFilter;
    std::string runtimeFilter;
    std::string stemFilter;
    std::string exhaustJetComparisonFilter;
    std::string structuralBankComparisonFilter;
    std::string forcedInductionPowerComparisonFilter;
    std::string couplingComparisonFilter;
    std::string junctionComparisonFilter;
    std::optional<std::size_t> intakeWorkers;
    std::optional<double> intakeWallHeatUpdateSeconds;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--output" && i + 1 < argc) outDir = argv[++i];
        else if (a == "--ir" && i + 1 < argc) irPath = argv[++i];
        else if (a == "--idle-only") idleOnly = true;
        else if (a == "--transient-only") transientOnly = true;
        else if (a == "--reference-filter" && i + 1 < argc)
            referenceFilter = argv[++i];
        else if (a == "--catalogue-filter" && i + 1 < argc)
            catalogueFilter = argv[++i];
        else if (a == "--runtime-filter" && i + 1 < argc)
            runtimeFilter = argv[++i];
        else if (a == "--stems" && i + 1 < argc)
            stemFilter = argv[++i];
        else if (a == "--exhaust-jet-comparison" && i + 1 < argc)
            exhaustJetComparisonFilter = argv[++i];
        else if (a == "--structural-bank-comparison" && i + 1 < argc)
            structuralBankComparisonFilter = argv[++i];
        else if (a == "--forced-induction-power-comparison" && i + 1 < argc)
            forcedInductionPowerComparisonFilter = argv[++i];
        else if (a == "--coupling-comparison" && i + 1 < argc)
            couplingComparisonFilter = argv[++i];
        else if (a == "--junction-comparison" && i + 1 < argc)
            junctionComparisonFilter = argv[++i];
        else if (a == "--intake-workers" && i + 1 < argc)
            intakeWorkers = static_cast<std::size_t>(std::stoull(argv[++i]));
        else if (a == "--intake-wall-us" && i + 1 < argc)
            intakeWallHeatUpdateSeconds = std::stod(argv[++i]) * 1.0e-6;
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

    if (!stemFilter.empty()) {
        const auto catalog = loadEngineCatalog(
            std::filesystem::path(ENGINELAB_CATALOG_ROOT));
        if (!catalog.errors.empty()) {
            std::cerr << "FAIL: catalogue load: " << catalog.errors.front() << '\n';
            return 2;
        }
        const auto selected = std::find_if(
            catalog.entries.begin(), catalog.entries.end(),
            [&stemFilter](const auto& entry) {
                return entry.config.name.find(stemFilter) != std::string::npos;
            });
        if (selected == catalog.entries.end()) {
            std::cerr << "FAIL: no catalogue engine matches stem filter '"
                      << stemFilter << "'\n";
            return 2;
        }
        std::cout << "\n--- Diagnostic pre-master stem export ---\n";
        writeDiagnosticStems = true;
        const auto metrics = renderEngine(
            selected->config, ir, outDir, 4.0, true);
        const auto valid = metrics.physicalActive
            && metrics.compiledTopologyActive
            && metrics.structuralRadiationActive
            && metrics.intakeTopologyActive
            && metrics.left.scan.finite && metrics.right.scan.finite
            && metrics.droppedPressureSamples == 0
            && metrics.invalidBoundarySamples == 0
            && metrics.levelLimitedSamples == 0;
        std::cout << "Stem export result: " << (valid ? "PASS" : "FAIL") << '\n';
        return valid ? 0 : 1;
    }

    if (transientOnly) {
        const auto catalog = loadEngineCatalog(
            std::filesystem::path(ENGINELAB_CATALOG_ROOT));
        if (!catalog.errors.empty()) {
            std::cerr << "FAIL: catalogue load: "
                      << catalog.errors.front() << '\n';
            return 2;
        }
        const auto findEngine = [&catalog](const std::string& fragment) {
            return std::find_if(
                catalog.entries.begin(), catalog.entries.end(),
                [&fragment](const auto& entry) {
                    return entry.config.name.find(fragment)
                        != std::string::npos;
                });
        };
        const auto idleEngine = findEngine("Big Twin");
        const auto boostedEngine = findEngine("2JZ");
        const auto limiterEngine = findEngine("K20");
        if (idleEngine == catalog.entries.end()
            || boostedEngine == catalog.entries.end()
            || limiterEngine == catalog.entries.end()) {
            std::cerr << "FAIL: transient fixtures require Big Twin, 2JZ, and K20 catalogue entries\n";
            return 2;
        }

        std::cout << "\n--- Audio transient regression suite ---\n";
        const auto idle = renderIdleCycle(
            idleEngine->config, ir, outDir / "start-idle-rev");
        const auto lift = renderEngine(
            boostedEngine->config, ir, outDir / "boost-lift",
            3.6, true, false, false, true);
        const auto limiter = renderEngine(
            limiterEngine->config, ir, outDir / "rev-limiter",
            4.5, true, false, true, false);
        const auto ok = validateIdleCycle(idle, idleEngine->config)
            && validateBoostLiftTransient(lift, boostedEngine->config)
            && validateLimiterTransient(limiter, limiterEngine->config);
        std::cout << "Transient result: " << (ok ? "PASS" : "FAIL")
                  << '\n';
        return ok ? 0 : 1;
    }

    if (!exhaustJetComparisonFilter.empty()) {
        const auto catalog = loadEngineCatalog(
            std::filesystem::path(ENGINELAB_CATALOG_ROOT));
        const auto selected = std::find_if(
            catalog.entries.begin(), catalog.entries.end(),
            [&exhaustJetComparisonFilter](const auto& entry) {
                return entry.config.name.find(exhaustJetComparisonFilter)
                    != std::string::npos;
            });
        if (selected == catalog.entries.end()) {
            std::cerr << "FAIL: no catalogue engine matches exhaust-jet comparison '"
                      << exhaustJetComparisonFilter << "'\n";
            return 2;
        }
        std::cout << "\n--- Exhaust outlet turbulence A/B ---\n";
        retainRenderedAudioForComparison = true;
        enableExhaustJetNoise = false;
        const auto baseline = renderEngine(
            selected->config, ir, outDir / "jet-off", 4.0, true);
        enableExhaustJetNoise = true;
        const auto candidate = renderEngine(
            selected->config, ir, outDir / "jet-on", 4.0, true);
        retainRenderedAudioForComparison = false;
        const auto similarity = cosineSimilarity(baseline, candidate);
        const auto comparisonSamples = std::min(
            baseline.comparisonAudioLeft.size(),
            candidate.comparisonAudioLeft.size());
        auto differenceSquareSum = 0.0;
        auto baselineSquareSum = 0.0;
        auto differencePeak = 0.0;
        for (std::size_t sample = 0; sample < comparisonSamples; ++sample) {
            const auto reference = static_cast<double>(
                baseline.comparisonAudioLeft[sample]);
            const auto difference = static_cast<double>(
                candidate.comparisonAudioLeft[sample]) - reference;
            differenceSquareSum += difference * difference;
            baselineSquareSum += reference * reference;
            differencePeak = std::max(
                differencePeak, std::abs(difference));
        }
        const auto differenceRms = comparisonSamples > 0
            ? std::sqrt(differenceSquareSum
                / static_cast<double>(comparisonSamples)) : 0.0;
        const auto relativeDifference = baselineSquareSum > 0.0
            ? std::sqrt(differenceSquareSum / baselineSquareSum) : 0.0;
        std::cout << std::fixed << std::setprecision(6)
                  << "  spectral cosine off/on=" << similarity << '\n'
                  << "  RMS left off/on=" << baseline.left.window.rms << '/'
                  << candidate.left.window.rms << '\n'
                  << "  brightness off/on="
                  << baseline.left.window.brightness << '/'
                  << candidate.left.window.brightness << '\n'
                  << "  band fractions off low/mid/high="
                  << baseline.left.window.lowBandFraction << '/'
                  << baseline.left.window.midBandFraction << '/'
                  << baseline.left.window.highBandFraction << '\n'
                  << "  band fractions on  low/mid/high="
                  << candidate.left.window.lowBandFraction << '/'
                  << candidate.left.window.midBandFraction << '/'
                  << candidate.left.window.highBandFraction << '\n'
                  << "  observed jet peak Pa off/on="
                  << baseline.maxExhaustJetNoisePressurePa << '/'
                  << candidate.maxExhaustJetNoisePressurePa << '\n'
                  << "  waveform difference RMS/relative/peak="
                  << differenceRms << '/' << relativeDifference << '/'
                  << differencePeak << '\n';
        const auto valid = [](const Metrics& measurement) {
            return measurement.left.scan.finite
                && measurement.right.scan.finite
                && measurement.physicalActive
                && measurement.compiledTopologyActive
                && measurement.legacyPathSamples == 0
                && measurement.invalidBoundarySamples == 0
                && measurement.droppedPressureSamples == 0
                && measurement.levelLimitedSamples == 0
                && measurement.maxPreLimiterMagnitude < 0.82F;
        };
        const auto isolated = baseline.maxExhaustJetNoisePressurePa == 0.0F
            && candidate.maxExhaustJetNoisePressurePa > 1.0e-6F
            && relativeDifference >= 0.01
            && relativeDifference <= 0.15
            && differencePeak < 0.25;
        if (!isolated)
            std::cerr << "FAIL: outlet turbulence did not produce an isolated "
                         "resolved acoustic difference\n";
        return valid(baseline) && valid(candidate) && isolated ? 0 : 1;
    }

    if (!structuralBankComparisonFilter.empty()) {
        const auto catalog = loadEngineCatalog(
            std::filesystem::path(ENGINELAB_CATALOG_ROOT));
        const auto selected = std::find_if(
            catalog.entries.begin(), catalog.entries.end(),
            [&structuralBankComparisonFilter](const auto& entry) {
                return entry.config.name.find(structuralBankComparisonFilter)
                    != std::string::npos;
            });
        if (selected == catalog.entries.end()) {
            std::cerr << "FAIL: no catalogue engine matches structural-bank "
                         "comparison '" << structuralBankComparisonFilter << "'\n";
            return 2;
        }
        std::cout << "\n--- Structural bank topology A/B ---\n";
        retainRenderedAudioForComparison = true;
        enableStructuralBankTopologyParticipation = false;
        const auto baseline = renderEngine(
            selected->config, ir, outDir / "flat-index", 4.0, true);
        enableStructuralBankTopologyParticipation = true;
        const auto candidate = renderEngine(
            selected->config, ir, outDir / "bank-topology", 4.0, true);
        retainRenderedAudioForComparison = false;
        const auto similarity = cosineSimilarity(baseline, candidate);
        const auto comparisonSamples = std::min(
            baseline.comparisonAudioLeft.size(),
            candidate.comparisonAudioLeft.size());
        auto differenceSquareSum = 0.0;
        auto baselineSquareSum = 0.0;
        auto differencePeak = 0.0;
        for (std::size_t sample = 0; sample < comparisonSamples; ++sample) {
            const auto reference = static_cast<double>(
                baseline.comparisonAudioLeft[sample]);
            const auto difference = static_cast<double>(
                candidate.comparisonAudioLeft[sample]) - reference;
            differenceSquareSum += difference * difference;
            baselineSquareSum += reference * reference;
            differencePeak = std::max(
                differencePeak, std::abs(difference));
        }
        const auto differenceRms = comparisonSamples > 0
            ? std::sqrt(differenceSquareSum
                / static_cast<double>(comparisonSamples)) : 0.0;
        const auto relativeDifference = baselineSquareSum > 0.0
            ? std::sqrt(differenceSquareSum / baselineSquareSum) : 0.0;
        std::cout << std::fixed << std::setprecision(6)
                  << "  spectral cosine flat-index/bank-topology="
                  << similarity << '\n'
                  << "  RMS left flat-index/bank-topology="
                  << baseline.left.window.rms << '/'
                  << candidate.left.window.rms << '\n'
                  << "  observed structure peak Pa flat-index/bank-topology="
                  << baseline.maxStructuralPressurePa << '/'
                  << candidate.maxStructuralPressurePa << '\n'
                  << "  waveform difference RMS/relative/peak="
                  << differenceRms << '/' << relativeDifference << '/'
                  << differencePeak << '\n';
        const auto valid = [](const Metrics& measurement) {
            return measurement.left.scan.finite
                && measurement.right.scan.finite
                && measurement.physicalActive
                && measurement.compiledTopologyActive
                && measurement.structuralRadiationActive
                && measurement.legacyPathSamples == 0
                && measurement.invalidBoundarySamples == 0
                && measurement.droppedPressureSamples == 0
                && measurement.levelLimitedSamples == 0
                && measurement.maxPreLimiterMagnitude < 0.82F;
        };
        const auto isolated = baseline.maxStructuralPressurePa > 0.0F
            && candidate.maxStructuralPressurePa > 0.0F
            && relativeDifference >= 0.001
            && relativeDifference <= 0.75
            && differencePeak < 0.50;
        if (!isolated)
            std::cerr << "FAIL: explicit bank topology did not produce a "
                         "bounded structural difference\n";
        return valid(baseline) && valid(candidate) && isolated ? 0 : 1;
    }

    if (!forcedInductionPowerComparisonFilter.empty()) {
        const auto catalog = loadEngineCatalog(
            std::filesystem::path(ENGINELAB_CATALOG_ROOT));
        const auto selected = std::find_if(
            catalog.entries.begin(), catalog.entries.end(),
            [&forcedInductionPowerComparisonFilter](const auto& entry) {
                return entry.config.name.find(
                    forcedInductionPowerComparisonFilter)
                    != std::string::npos;
            });
        if (selected == catalog.entries.end()) {
            std::cerr << "FAIL: no catalogue engine matches FI-power "
                         "comparison '" << forcedInductionPowerComparisonFilter
                      << "'\n";
            return 2;
        }
        std::cout << "\n--- Forced-induction broadband power A/B ---\n";
        retainRenderedAudioForComparison = true;
        enableForcedInductionBroadbandPowerNormalisation = false;
        const auto baseline = renderEngine(
            selected->config, ir, outDir / "unnormalised-band", 4.0, true);
        enableForcedInductionBroadbandPowerNormalisation = true;
        const auto candidate = renderEngine(
            selected->config, ir, outDir / "power-normalised-band", 4.0, true);
        retainRenderedAudioForComparison = false;
        const auto similarity = cosineSimilarity(baseline, candidate);
        const auto comparisonSamples = std::min(
            baseline.comparisonAudioLeft.size(),
            candidate.comparisonAudioLeft.size());
        auto differenceSquareSum = 0.0;
        auto baselineSquareSum = 0.0;
        auto differencePeak = 0.0;
        for (std::size_t sample = 0; sample < comparisonSamples; ++sample) {
            const auto reference = static_cast<double>(
                baseline.comparisonAudioLeft[sample]);
            const auto difference = static_cast<double>(
                candidate.comparisonAudioLeft[sample]) - reference;
            differenceSquareSum += difference * difference;
            baselineSquareSum += reference * reference;
            differencePeak = std::max(
                differencePeak, std::abs(difference));
        }
        const auto differenceRms = comparisonSamples > 0
            ? std::sqrt(differenceSquareSum
                / static_cast<double>(comparisonSamples)) : 0.0;
        const auto relativeDifference = baselineSquareSum > 0.0
            ? std::sqrt(differenceSquareSum / baselineSquareSum) : 0.0;
        std::cout << std::fixed << std::setprecision(6)
                  << "  spectral cosine unnormalised/power-normalised="
                  << similarity << '\n'
                  << "  RMS left unnormalised/power-normalised="
                  << baseline.left.window.rms << '/'
                  << candidate.left.window.rms << '\n'
                  << "  observed FI peak Pa unnormalised/power-normalised="
                  << baseline.maxForcedInductionPressurePa << '/'
                  << candidate.maxForcedInductionPressurePa << '\n'
                  << "  waveform difference RMS/relative/peak="
                  << differenceRms << '/' << relativeDifference << '/'
                  << differencePeak << '\n';
        const auto valid = [](const Metrics& measurement) {
            return measurement.left.scan.finite
                && measurement.right.scan.finite
                && measurement.physicalActive
                && measurement.compiledTopologyActive
                && measurement.forcedInductionAcousticsActive
                && measurement.legacyPathSamples == 0
                && measurement.invalidBoundarySamples == 0
                && measurement.droppedPressureSamples == 0
                && measurement.levelLimitedSamples == 0
                && measurement.maxPreLimiterMagnitude < 0.82F;
        };
        const auto isolated = baseline.maxForcedInductionPressurePa > 0.0F
            && candidate.maxForcedInductionPressurePa
                > baseline.maxForcedInductionPressurePa
            && relativeDifference >= 0.0001
            && relativeDifference <= 0.50
            && differencePeak < 0.50;
        if (!isolated)
            std::cerr << "FAIL: broadband power normalisation did not produce "
                         "a bounded FI-only difference\n";
        return valid(baseline) && valid(candidate) && isolated ? 0 : 1;
    }

    if (!junctionComparisonFilter.empty()) {
        const auto catalog = loadEngineCatalog(
            std::filesystem::path(ENGINELAB_CATALOG_ROOT));
        const auto selected = std::find_if(catalog.entries.begin(), catalog.entries.end(),
            [&junctionComparisonFilter](const auto& entry) {
                return entry.config.name.find(junctionComparisonFilter)
                    != std::string::npos;
            });
        if (selected == catalog.entries.end()) {
            std::cerr << "FAIL: no catalogue engine matches junction comparison '"
                      << junctionComparisonFilter << "'\n";
            return 2;
        }
        std::cout << "\n--- Exhaust collector momentum A/B ---\n";
        referenceCouplingEverySubstep = false;
        renderSimulatorOptions = {};
        renderSimulatorOptions.evolveExhaustJunctionAxialMomentum = true;
        const auto directed = renderEngine(
            selected->config, ir, outDir / "junction-directed", 3.0, true);
        renderSimulatorOptions = {};
        renderSimulatorOptions.evolveExhaustJunctionAxialMomentum = false;
        const auto mixed = renderEngine(
            selected->config, ir, outDir / "junction-well-mixed", 3.0, true);
        renderSimulatorOptions = {};

        const auto similarity = cosineSimilarity(directed, mixed);
        std::cout << std::fixed << std::setprecision(6)
                  << "  spectral cosine directed/well-mixed=" << similarity << '\n'
                  << "  exhaust observer peak Pa directed/well-mixed="
                  << directed.maxExhaustPressurePa << '/'
                  << mixed.maxExhaustPressurePa << '\n'
                  << "  RMS left directed/well-mixed="
                  << directed.left.window.rms << '/'
                  << mixed.left.window.rms << '\n'
                  << "  high-band fraction directed/well-mixed="
                  << directed.left.window.highBandFraction << '/'
                  << mixed.left.window.highBandFraction << '\n';
        const auto valid = [](const Metrics& measurement) {
            return measurement.left.scan.finite
                && measurement.right.scan.finite
                && measurement.physicalActive
                && measurement.compiledTopologyActive
                && measurement.structuralRadiationActive
                && measurement.intakeTopologyActive
                && measurement.legacyPathSamples == 0
                && measurement.invalidBoundarySamples == 0
                && measurement.droppedPressureSamples == 0
                && measurement.levelLimitedSamples == 0
                && measurement.maxPreLimiterMagnitude < 0.82F;
        };
        // The A/B is useful only if the corrected cylinder-pressure excitation
        // reaches the renderer strongly enough to change its spectral result.
        const auto acousticallyResolved = similarity < 0.999;
        if (!acousticallyResolved)
            std::cerr << "FAIL: collector momentum change did not produce a "
                         "resolved acoustic difference\n";
        return valid(directed) && valid(mixed) && acousticallyResolved ? 0 : 1;
    }

    if (!couplingComparisonFilter.empty()) {
        const auto catalog = loadEngineCatalog(
            std::filesystem::path(ENGINELAB_CATALOG_ROOT));
        const auto selected = std::find_if(catalog.entries.begin(), catalog.entries.end(),
            [&couplingComparisonFilter](const auto& entry) {
                return entry.config.name.find(couplingComparisonFilter)
                    != std::string::npos;
            });
        if (selected == catalog.entries.end()) {
            std::cerr << "FAIL: no catalogue engine matches coupling comparison '"
                      << couplingComparisonFilter << "'\n";
            return 2;
        }
        std::cout << "\n--- Exhaust coupling bandwidth A/B/oracle ---\n";
        referenceCouplingEverySubstep = false;
        renderSimulatorOptions = {};
        const auto production = renderEngine(
            selected->config, ir, outDir / "coupling-125us", 3.0, true);
        renderSimulatorOptions.maximumLowSpeedExhaustCouplingSeconds = 250.0e-6;
        const auto historical = renderEngine(
            selected->config, ir, outDir / "coupling-250us", 3.0, true);
        renderSimulatorOptions = {};
        referenceCouplingEverySubstep = true;
        const auto oracle = renderEngine(
            selected->config, ir, outDir / "coupling-oracle", 3.0, true);
        referenceCouplingEverySubstep = false;

        const auto productionSimilarity = cosineSimilarity(production, oracle);
        const auto historicalSimilarity = cosineSimilarity(historical, oracle);
        std::cout << std::fixed << std::setprecision(6)
                  << "  couplingHz production/historical/oracle="
                  << production.couplingHz << '/' << historical.couplingHz
                  << '/' << oracle.couplingHz << '\n'
                  << "  physical Nyquist Hz production/historical="
                  << production.couplingHz * 0.5 << '/'
                  << historical.couplingHz * 0.5 << '\n'
                  << "  spectral cosine to full-substep oracle production="
                  << productionSimilarity << " historical="
                  << historicalSimilarity << '\n'
                  << "  high-band fraction production/historical/oracle="
                  << production.left.window.highBandFraction << '/'
                  << historical.left.window.highBandFraction << '/'
                  << oracle.left.window.highBandFraction << '\n';
        const auto structurallyValid = production.left.scan.finite
            && historical.left.scan.finite && oracle.left.scan.finite
            && production.invalidBoundarySamples == 0
            && historical.invalidBoundarySamples == 0
            && oracle.invalidBoundarySamples == 0
            && production.droppedPressureSamples == 0
            && historical.droppedPressureSamples == 0
            && oracle.droppedPressureSamples == 0
            && production.couplingHz >= historical.couplingHz * 1.80;
        return structurallyValid ? 0 : 1;
    }

    if (!runtimeFilter.empty()) {
        const auto catalog = loadEngineCatalog(
            std::filesystem::path(ENGINELAB_CATALOG_ROOT));
        const auto selected = std::find_if(catalog.entries.begin(), catalog.entries.end(),
            [&runtimeFilter](const auto& entry) {
                return entry.config.name.find(runtimeFilter) != std::string::npos;
            });
        if (selected == catalog.entries.end()) {
            std::cerr << "FAIL: no catalogue engine matches runtime filter '"
                      << runtimeFilter << "'\n";
            return 2;
        }
        std::cout << "\n--- Application wiring: selected EngineRuntime path check ---\n";
        return runtimePathCheck(
            selected->config, ir, intakeWorkers,
            intakeWallHeatUpdateSeconds) ? 0 : 1;
    }

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
            && metrics.compiledTopologyActive
            && metrics.structuralRadiationActive
            && metrics.intakeTopologyActive
            && metrics.left.scan.finite && metrics.right.scan.finite
            && metrics.droppedPressureSamples == 0
            && metrics.invalidBoundarySamples == 0;
        return valid ? 0 : 1;
    }

    if (!catalogueFilter.empty()) {
        const auto catalog = loadEngineCatalog(
            std::filesystem::path(ENGINELAB_CATALOG_ROOT));
        const auto selected = std::find_if(catalog.entries.begin(), catalog.entries.end(),
            [&catalogueFilter](const auto& entry) {
                return entry.config.name.find(catalogueFilter) != std::string::npos;
            });
        if (selected == catalog.entries.end()) return 2;
        const auto metrics = renderEngine(selected->config, ir, outDir, 3.0, true);
        return metrics.physicalActive && metrics.compiledTopologyActive
            && metrics.structuralRadiationActive && metrics.intakeTopologyActive
            && metrics.left.scan.finite && metrics.right.scan.finite
            && metrics.legacyPathSamples == 0
            && metrics.invalidBoundarySamples == 0
            && metrics.levelLimitedSamples == 0
            && metrics.maxPreLimiterMagnitude < 0.82F
            && metrics.left.window.crest < 16.0
            && metrics.right.window.crest < 16.0 ? 0 : 1;
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
    std::cout << "\n--- Complete catalogue render (3.0 s each) ---\n";
    std::filesystem::create_directories(outDir / "catalogue");
    std::vector<std::pair<std::string, Metrics>> catalogueMetrics;
    catalogueMetrics.reserve(catalog.entries.size());
    for (const auto& entry : catalog.entries)
        catalogueMetrics.emplace_back(entry.config.name,
            // Starter release ends at 1.1 s and the dyno begins at 1.2 s. A
            // two-second capture made the "final steady-state" window include
            // that load transition, so a legitimate Merlin firing pulse was
            // divided by a transitional RMS and misclassified as an isolated
            // click. Three seconds leaves a complete settled analysis window.
            renderEngine(entry.config, ir, outDir / "catalogue", 3.0, true));
    const auto idleEngine = std::find_if(catalog.entries.begin(), catalog.entries.end(),
        [](const auto& entry) {
            return entry.config.name.find("Big Twin") != std::string::npos;
        });
    std::cout << "\n--- True idle start/rev/return (catalog Big Twin, 12 s) ---\n";
    const auto idleCycle = idleEngine != catalog.entries.end()
        ? renderIdleCycle(idleEngine->config, ir, outDir) : IdleCycleMetrics {};

    // Instrument, not a gate: the correct RT60 per preset is not yet established,
    std::cout << "\n--- Physical exhaust decay (inline4, Schroeder RT60) ---\n";
    const auto dryDecay = measureExhaustDecay(
        makeDefaultInlineFour(), ir, 6.0, 0.0F);
    const auto wetDecay = measureExhaustDecay(
        makeDefaultInlineFour(), ir, 6.0, 1.0F);
    std::cout << "  rt60(freeField)=" << std::fixed << std::setprecision(3)
              << dryDecay.rt60Seconds << " s"
              << " rt60(withIR)=" << wetDecay.rt60Seconds << " s"
              << " peak=" << std::setprecision(4) << dryDecay.peak << '/'
              << wetDecay.peak
              << " tailFloor=" << std::setprecision(1) << wetDecay.tailFloorDb << " dB"
              << " finite=" << (dryDecay.finite && wetDecay.finite ? "yes" : "NO")
              << (dryDecay.valid && wetDecay.valid ? "" : "  [UNMEASURABLE]") << '\n';
    const auto decayMeasurable = dryDecay.finite && dryDecay.valid
        && wetDecay.finite && wetDecay.valid;

    bool ok = true;
    const auto validate = [&ok](const Metrics& m, const std::string& label,
                                bool requireSpectralBalance,
                                bool requireForcedInduction = false) {
        const auto fail = [&ok, &label](const std::string& reason) {
            std::cerr << "FAIL: " << label << ": " << reason << '\n';
            ok = false;
        };
        if (!m.physicalActive) fail("physical exhaust rendering was not active");
        if (!m.compiledTopologyActive)
            fail("the complete exhaust topology was not active");
        if (!m.structuralRadiationActive)
            fail("modal block/head radiation was not active");
        if (!m.intakeTopologyActive)
            fail("the intake wave network was not active");
        if (requireForcedInduction && !m.forcedInductionAcousticsActive)
            fail("the solver-driven forced-induction source was not active");
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
            // a running engine must radiate plausible sound power. Catalogue
            // microphone distances intentionally differ (a Merlin is observed
            // much farther away than a motorcycle), so compare the directional
            // pressure extrapolated to one metre using the same free-field 1/r
            // law as the observer. Testing raw SPL at arbitrary distances would
            // reject geometry rather than source physics.
            const auto rmsPressurePa = channel->window.rms
                * AcousticMonitorCalibration::sinePeakPressurePa(
                    AcousticMonitorCalibration::defaultFullScaleSplDb);
            const auto oneMetreEquivalentPressurePa = rmsPressurePa
                * m.observerDistanceM;
            const auto soundPressureLevelDb = 20.0 * std::log10(
                std::max(oneMetreEquivalentPressurePa, 1.0e-12)
                / AcousticMonitorCalibration::referenceRmsPressurePa);
            if (soundPressureLevelDb < 90.0)
                fail("radiated level is below 90 dB SPL at one-metre equivalent ("
                     + std::to_string(static_cast<int>(soundPressureLevelDb)) + " dB)" + suffix);
            if (soundPressureLevelDb > 130.0)
                fail("radiated level exceeds 130 dB SPL at one-metre equivalent ("
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
        if (m.legacyPathSamples != 0)
            fail("procedural compatibility audio leaked into the production render");
        if (m.invalidBoundarySamples != 0)
            fail("the physical source stream contained invalid boundary samples");
        // Delay lines are sized in prepare() from the published geometry, so a
        // clamp here means the rendered acoustic length is shorter than configured.
        if (m.delayTruncations != 0) fail("a delay line was too short and truncated");
        // Channel correlation is reported rather than forced: a centred single
        // outlet is legitimately almost mono, while authored separated outlets
        // acquire width only from their physical microphone delays.
        // The safety leveler must be safety-only in a shipped voice: if it is
        // pulling gain below identity here, the default level is set too hot and
        // the AGC is silently masking that offset. Keep the level honest instead.
        if (m.levelLimitedSamples != 0 || m.minLevelGain < 0.99999F)
            fail("safety leveler engaged at the default voice (AGC masking a level offset)");
        if (m.maxPreLimiterMagnitude >= 0.82F)
            fail("physical layers reach the output limiter knee before monitoring");
    };
    for (std::size_t index = 0; index < metrics.size(); ++index)
        validate(metrics[index], engines[index].label, true);
    validate(turbo, "synthetic turbo", true, true);
    validate(stability, "long-run inline4", true);
    for (const auto& [label, measurement] : catalogueMetrics)
        validate(measurement, "catalogue " + label, true,
            measurement.forcedInductionAcousticsActive);
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
