// How much does the exhaust GEOMETRY change the sound?
//
// This is the instrument the project never had, and its absence is why months
// of exhaust work could not be judged. Every existing harness measures one
// render against a threshold, or two renders of the same geometry against each
// other. None of them answers the question a user actually asks: I fitted a
// short open pipe, why does it sound the same?
//
// So this one renders the SAME engine through the real realtime audio path with
// N different exhausts and reports the distance between the results, per third
// octave. It reports three numbers per pair and they mean different things:
//
//   niveau     broadband level change, dB. "Everything got louder" lands here.
//   forme      RMS over third-octave bands of the level difference AFTER the
//              broadband offset is removed. This is the SHAPE change, and it is
//              the number that says whether the geometry was heard.
//   bande_max  the single most-changed third octave, same offset removed.
//
// The separation matters because a broadband gain is not a geometry effect --
// moving the observer does that -- while a real silencer removes 15-25 dB from
// specific bands and leaves others alone.
//
// It measures the full mix AND the exhaust stem alone. Those two answer
// different questions: a large stem change with a small mix change means the
// exhaust responds to geometry but is buried; both small means the exhaust
// model itself does not respond.
//
// This instrument compares simulator variants against one another. It does not
// turn those internal A/Bs into claims about a real exhaust or listening test.

#include <enginelab/audio/RealtimeEngineAudio.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/gasdynamics/ExhaustNetworkLayout.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/runtime/EngineRuntime.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <juce_audio_basics/juce_audio_basics.h>
#include <juce_dsp/juce_dsp.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <numbers>
#include <set>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace {
using namespace enginelab;

constexpr double audioRate = 48'000.0;
constexpr double frameDt = 1.0 / 240.0;
constexpr int samplesPerFrame = 200; // 48000 / 240

// ISO third-octave centres from 25 Hz to 12.5 kHz. The band edges are
// fc * 2^(+-1/6), so they tile without gaps.
constexpr std::array<double, 28> thirdOctaveCentresHz {
    25.0, 31.5, 40.0, 50.0, 63.0, 80.0, 100.0, 125.0, 160.0, 200.0,
    250.0, 315.0, 400.0, 500.0, 630.0, 800.0, 1'000.0, 1'250.0, 1'600.0,
    2'000.0, 2'500.0, 3'150.0, 4'000.0, 5'000.0, 6'300.0, 8'000.0,
    10'000.0, 12'500.0
};

/** Third-octave band levels in dB, plus the broadband level. A band with no
 *  measurable energy is marked absent rather than being floored at some
 *  arbitrary number, because a floored band would then report a huge and
 *  entirely fictitious difference against a band that does have energy. */
struct Spectrum final {
    std::array<double, thirdOctaveCentresHz.size()> bandDb {};
    std::array<bool, thirdOctaveCentresHz.size()> bandPresent {};
    double broadbandDb { -200.0 };
    double rms { 0.0 };
};

/** Distance between two spectra. `shapeRmsDb` is the headline: it is the RMS
 *  band difference with the broadband offset removed, so it answers "did the
 *  TONE change" rather than "did the level change". */
struct Distance final {
    double levelDb { 0.0 };
    double shapeRmsDb { 0.0 };
    double worstBandDb { 0.0 };
    double worstBandHz { 0.0 };
    std::size_t comparedBands { 0 };
};

[[nodiscard]] Spectrum analyse(const std::vector<float>& signal,
                               std::size_t begin) {
    Spectrum spectrum;
    if (begin >= signal.size()) return spectrum;

    double squareSum = 0.0;
    std::size_t counted = 0;
    for (std::size_t index = begin; index < signal.size(); ++index) {
        const auto value = static_cast<double>(signal[index]);
        if (!std::isfinite(value)) continue;
        squareSum += value * value;
        ++counted;
    }
    spectrum.rms = std::sqrt(squareSum / static_cast<double>(std::max<std::size_t>(1, counted)));
    spectrum.broadbandDb = 20.0 * std::log10(std::max(spectrum.rms, 1.0e-12));

    // One long window over the whole steady-state segment. The signal is
    // periodic at the firing frequency and the run is held at a fixed speed, so
    // averaging several shorter windows would only trade frequency resolution
    // for a variance reduction that a deterministic simulator does not need.
    constexpr int fftOrder = 16;
    constexpr std::size_t fftSize = std::size_t { 1 } << fftOrder;
    std::vector<float> fftData(fftSize * 2, 0.0F);
    const auto available = std::min(signal.size() - begin, fftSize);
    const auto sourceBegin = signal.size() - available;
    const auto destinationBegin = fftSize - available;
    double mean = 0.0;
    for (std::size_t index = 0; index < available; ++index) {
        const auto value = static_cast<double>(signal[sourceBegin + index]);
        if (std::isfinite(value)) mean += value;
    }
    mean /= static_cast<double>(std::max<std::size_t>(1, available));
    double windowPowerSum = 0.0;
    for (std::size_t index = 0; index < available; ++index) {
        const auto fftIndex = destinationBegin + index;
        const auto window = 0.5 - 0.5 * std::cos(2.0 * std::numbers::pi
            * static_cast<double>(fftIndex) / static_cast<double>(fftSize - 1));
        windowPowerSum += window * window;
        const auto value = static_cast<double>(signal[sourceBegin + index]);
        fftData[fftIndex] = static_cast<float>(
            (std::isfinite(value) ? value - mean : 0.0) * window);
    }
    juce::dsp::FFT fft(fftOrder);
    fft.performFrequencyOnlyForwardTransform(fftData.data());

    const auto binHz = audioRate / static_cast<double>(fftSize);
    // Normalise by the window's own power so the reported band levels are
    // absolute (dB relative to full scale) rather than relative to an FFT
    // length that a future edit might change.
    const auto powerScale = 1.0 / std::max(1.0, windowPowerSum);
    for (std::size_t band = 0; band < thirdOctaveCentresHz.size(); ++band) {
        const auto centre = thirdOctaveCentresHz[band];
        const auto lower = centre * std::pow(2.0, -1.0 / 6.0);
        const auto upper = centre * std::pow(2.0, 1.0 / 6.0);
        const auto firstBin = std::max<std::size_t>(1,
            static_cast<std::size_t>(std::ceil(lower / binHz)));
        const auto lastBin = std::min(fftSize / 2,
            static_cast<std::size_t>(std::floor(upper / binHz)));
        if (firstBin > lastBin) continue;
        double power = 0.0;
        for (auto bin = firstBin; bin <= lastBin; ++bin) {
            const auto magnitude = static_cast<double>(fftData[bin]);
            power += magnitude * magnitude;
        }
        power *= powerScale;
        // Below this the band is numerical noise, not signal, and comparing two
        // noise floors manufactures differences that mean nothing.
        constexpr double presenceFloor = 1.0e-20;
        if (!(power > presenceFloor)) continue;
        spectrum.bandDb[band] = 10.0 * std::log10(power);
        spectrum.bandPresent[band] = true;
    }
    return spectrum;
}

[[nodiscard]] Distance distance(const Spectrum& a, const Spectrum& b) {
    Distance result;
    result.levelDb = b.broadbandDb - a.broadbandDb;
    double sum = 0.0;
    std::size_t counted = 0;
    for (std::size_t band = 0; band < thirdOctaveCentresHz.size(); ++band) {
        if (!a.bandPresent[band] || !b.bandPresent[band]) continue;
        sum += b.bandDb[band] - a.bandDb[band];
        ++counted;
    }
    if (counted == 0) return result;
    // Remove the mean band offset, not the broadband one: a change that is
    // purely a gain must score zero shape, and the two offsets differ slightly
    // because the bands do not cover the full audio range with equal weight.
    const auto offset = sum / static_cast<double>(counted);
    double squareSum = 0.0;
    for (std::size_t band = 0; band < thirdOctaveCentresHz.size(); ++band) {
        if (!a.bandPresent[band] || !b.bandPresent[band]) continue;
        const auto deviation = (b.bandDb[band] - a.bandDb[band]) - offset;
        squareSum += deviation * deviation;
        if (std::abs(deviation) > std::abs(result.worstBandDb)) {
            result.worstBandDb = deviation;
            result.worstBandHz = thirdOctaveCentresHz[band];
        }
    }
    result.comparedBands = counted;
    result.shapeRmsDb = std::sqrt(squareSum / static_cast<double>(counted));
    return result;
}

/** The axes "it sounds better" decomposes into, none of which a spectrum shape
 *  can answer. Shape says whether the TONE differs; these say whether the sound
 *  is PUNCHY, BRIGHT or ALIVE, and each points at a different part of the code.
 *
 *  `impliedFiringHz` is a by-product worth as much as the rest: it recovers the
 *  operating point from the recording itself, so an external capture whose rpm
 *  nobody controlled can still be compared against a render at a known speed. */
struct Character final {
    /** Peak over RMS. The classic impulsiveness measure, but sensitive to a
     *  single sample, which is why the p99.9 figure sits beside it. */
    double crestDb { 0.0 };
    double p999Db { 0.0 };
    /** Spread of short-block level: how hard the signal pulses between firing
     *  events. A drone modulates little; a bark modulates a lot. */
    double modulationDb { 0.0 };
    /** Regression of band level against log2 of band centre. Positive is
     *  brighter. This is the axis a missing radiation derivative would move. */
    double spectralTiltDbPerOctave { 0.0 };
    double envelopePeriodSeconds { 0.0 };
    double impliedFiringHz { 0.0 };
    /** Dispersion of energy from one firing period to the next. This is
     *  "does the idle breathe" rather than "is it loud". */
    double periodEnergyCovPercent { 0.0 };
    std::size_t periodsCounted { 0 };
    double periodicityStrength { 0.0 };
};

[[nodiscard]] Character characterise(const std::vector<float>& signal,
                                     std::size_t begin,
                                     const Spectrum& spectrum) {
    Character character;
    if (begin >= signal.size() || !(spectrum.rms > 0.0)) return character;

    std::vector<double> magnitudes;
    magnitudes.reserve(signal.size() - begin);
    for (auto index = begin; index < signal.size(); ++index) {
        const auto value = static_cast<double>(signal[index]);
        if (std::isfinite(value)) magnitudes.push_back(std::abs(value));
    }
    if (magnitudes.size() < 1'024) return character;

    auto sorted = magnitudes;
    std::sort(sorted.begin(), sorted.end());
    const auto peak = sorted.back();
    const auto p999 = sorted[static_cast<std::size_t>(
        0.999 * static_cast<double>(sorted.size() - 1))];
    character.crestDb = 20.0 * std::log10(std::max(peak, 1.0e-12) / spectrum.rms);
    character.p999Db = 20.0 * std::log10(std::max(p999, 1.0e-12) / spectrum.rms);

    // 0.25 ms blocks. Fine enough that a 200 Hz firing rate still spans about
    // 20 blocks, which the period search below needs to resolve rpm usefully.
    constexpr double blockSeconds = 0.000'25;
    const auto blockSize = static_cast<std::size_t>(audioRate * blockSeconds);
    std::vector<double> blocks;
    blocks.reserve(magnitudes.size() / std::max<std::size_t>(1, blockSize));
    for (std::size_t index = 0; index + blockSize <= magnitudes.size(); index += blockSize) {
        double squareSum = 0.0;
        for (std::size_t k = 0; k < blockSize; ++k)
            squareSum += magnitudes[index + k] * magnitudes[index + k];
        blocks.push_back(std::sqrt(squareSum / static_cast<double>(blockSize)));
    }
    if (blocks.size() < 512) return character;

    auto sortedBlocks = blocks;
    std::sort(sortedBlocks.begin(), sortedBlocks.end());
    const auto low = sortedBlocks[static_cast<std::size_t>(
        0.10 * static_cast<double>(sortedBlocks.size() - 1))];
    const auto high = sortedBlocks[static_cast<std::size_t>(
        0.90 * static_cast<double>(sortedBlocks.size() - 1))];
    character.modulationDb = 20.0 * std::log10(
        std::max(high, 1.0e-12) / std::max(low, 1.0e-12));

    double sumN = 0.0, sumX = 0.0, sumY = 0.0, sumXX = 0.0, sumXY = 0.0;
    for (std::size_t band = 0; band < thirdOctaveCentresHz.size(); ++band) {
        if (!spectrum.bandPresent[band]) continue;
        const auto x = std::log2(thirdOctaveCentresHz[band]);
        const auto y = spectrum.bandDb[band];
        sumN += 1.0; sumX += x; sumY += y; sumXX += x * x; sumXY += x * y;
    }
    if (sumN >= 3.0) {
        const auto denominator = sumN * sumXX - sumX * sumX;
        if (std::abs(denominator) > 1.0e-12)
            character.spectralTiltDbPerOctave =
                (sumN * sumXY - sumX * sumY) / denominator;
    }

    // Firing period from the envelope's autocorrelation. The lag window spans
    // 17 Hz to 1 kHz, which covers every firing rate the catalogue can reach.
    double mean = 0.0;
    for (const auto block : blocks) mean += block;
    mean /= static_cast<double>(blocks.size());
    std::vector<double> centred(blocks.size());
    for (std::size_t index = 0; index < blocks.size(); ++index)
        centred[index] = blocks[index] - mean;
    double norm = 0.0;
    for (const auto value : centred) norm += value * value;
    if (!(norm > 0.0)) return character;

    const std::size_t minimumLag = 4;
    const auto maximumLag = std::min<std::size_t>(240, blocks.size() / 4);
    double best = 0.0;
    std::size_t bestLag = 0;
    std::vector<double> correlation(maximumLag + 1, 0.0);
    for (auto lag = minimumLag; lag <= maximumLag; ++lag) {
        double accumulator = 0.0;
        for (std::size_t index = 0; index + lag < centred.size(); ++index)
            accumulator += centred[index] * centred[index + lag];
        correlation[lag] = accumulator / norm;
        if (correlation[lag] > best) { best = correlation[lag]; bestLag = lag; }
    }
    character.periodicityStrength = best;
    // Below this the envelope has no repeating structure and any "period" would
    // be the largest noise peak, which would then be reported as an rpm.
    if (bestLag == 0 || best < 0.10) return character;

    // Parabolic interpolation on the correlation peak: without it the period is
    // quantised to the block size and the implied rpm is coarse at high speed.
    auto refinedLag = static_cast<double>(bestLag);
    if (bestLag > minimumLag && bestLag < maximumLag) {
        const auto before = correlation[bestLag - 1];
        const auto after = correlation[bestLag + 1];
        const auto denominator = before - 2.0 * best + after;
        if (std::abs(denominator) > 1.0e-12)
            refinedLag += 0.5 * (before - after) / denominator;
    }
    character.envelopePeriodSeconds = refinedLag * blockSeconds;
    character.impliedFiringHz = 1.0 / std::max(character.envelopePeriodSeconds, 1.0e-9);

    std::vector<double> energies;
    for (std::size_t index = 0; index + bestLag <= blocks.size(); index += bestLag) {
        double energy = 0.0;
        for (std::size_t k = 0; k < bestLag; ++k)
            energy += blocks[index + k] * blocks[index + k];
        energies.push_back(energy);
    }
    if (energies.size() >= 8) {
        double energyMean = 0.0;
        for (const auto energy : energies) energyMean += energy;
        energyMean /= static_cast<double>(energies.size());
        double variance = 0.0;
        for (const auto energy : energies)
            variance += (energy - energyMean) * (energy - energyMean);
        variance /= static_cast<double>(energies.size());
        if (energyMean > 0.0)
            character.periodEnergyCovPercent = 100.0 * std::sqrt(variance) / energyMean;
        character.periodsCounted = energies.size();
    }
    return character;
}

void printCharacter(const Character& character) {
    std::cout << std::fixed
              << "      crest " << std::setprecision(1) << std::setw(5)
              << character.crestDb << " dB   p99.9 " << std::setw(5)
              << character.p999Db << " dB   modulation " << std::setw(5)
              << character.modulationDb << " dB   pente "
              << std::showpos << std::setprecision(2) << std::setw(6)
              << character.spectralTiltDbPerOctave << " dB/oct" << std::noshowpos
              << '\n' << "      allumage ";
    if (character.impliedFiringHz > 0.0)
        std::cout << std::setprecision(1) << std::setw(6) << character.impliedFiringHz
                  << " Hz (periodicite " << std::setprecision(2)
                  << character.periodicityStrength << ")   COV cycle a cycle "
                  << std::setprecision(1) << character.periodEnergyCovPercent
                  << " % sur " << character.periodsCounted << " periodes\n";
    else
        std::cout << "non detecte (periodicite " << std::setprecision(2)
                  << character.periodicityStrength << ")\n";
}

struct RenderResult final {
    Spectrum mix;
    Spectrum exhaust;
    /** The rendered mix itself, kept so it can be written to WAV and then
     *  measured by the SAME path as an external capture. */
    std::vector<float> mixSignal;
    std::size_t steadyBegin { 0 };
    /** Everything in the mix that is NOT the exhaust chain: combustion,
     *  intake, forced induction, mechanical. Kept as its own sum because the
     *  question "is the exhaust audible" is per band and cannot be answered
     *  from broadband levels. */
    Spectrum others;
    /** The four masking layers separately, in the order below. A sum says the
     *  exhaust is covered; only the split says by WHAT, and the answer decides
     *  which layer to touch. */
    std::array<Spectrum, 4> layers {};
    double finalRpm { 0.0 };
    double limitingCflLengthMm { 0.0 };
    double networkSubstepHz { 0.0 };
    bool physicalActive { false };
    bool finite { true };
    /** Maximum path pressure seen over the complete run-up and hold. It is a
     *  stability diagnostic only: unlike the spectra below it is neither an
     *  aggregate stereo pressure nor restricted to the steady-state window. */
    double maximumRunExhaustPressurePa { 0.0 };
    /** The two ways a level change can be removed after the fact. A slow AGC
     *  sitting below 1 means the safety leveler is doing steady-state gain
     *  work, which would turn "I removed the silencer" into "it sounds
     *  different but no louder" -- exactly the reported symptom. */
    double minLevelGain { 1.0 };
    std::uint64_t levelLimitedSamples { 0 };
    std::uint64_t saturationProcessedSamples { 0 };
    std::uint64_t softLimitedSamples { 0 };
    std::uint64_t hardClampedSamples { 0 };
    double maxPreLimiter { 0.0 };
    double maxPostLimiterPeak { 0.0 };
};

/** Per-band margin of the exhaust over everything else, in dB. Negative means
 *  the exhaust is BELOW the rest of the mix in that third octave, so no amount
 *  of geometry response there can reach the listener. */
[[nodiscard]] std::array<double, thirdOctaveCentresHz.size()>
maskingMarginDb(const Spectrum& exhaust, const Spectrum& others) {
    std::array<double, thirdOctaveCentresHz.size()> margin {};
    for (std::size_t band = 0; band < margin.size(); ++band) {
        margin[band] = exhaust.bandPresent[band] && others.bandPresent[band]
            ? exhaust.bandDb[band] - others.bandDb[band]
            : std::numeric_limits<double>::quiet_NaN();
    }
    return margin;
}

/** Run the real realtime audio path offline, holding a commanded speed, and
 *  return the steady-state spectra of the full mix and of the exhaust stem.
 *
 *  This mirrors AudioRenderHarness::renderEngine deliberately: the point of the
 *  measurement is that it goes through the delivered path, so anything
 *  simplified here would measure a different renderer. */
[[nodiscard]] RenderResult render(const EngineConfig& baseConfig,
                                  double targetRpm, double seconds) {
    auto config = baseConfig;
    normaliseEngineConfig(config);
    RenderResult result;
    {
        const auto graph = ExhaustGraph::makeForEngine(config);
        const auto layout = gasdynamics::ExhaustNetworkLayout::compile(
            graph, gasdynamics::realtimeExhaustFeedbackDiscretisation());
        result.limitingCflLengthMm = layout.minimumCflLengthM() * 1'000.0;
    }

    SimpleEcuModel ecu;
    SimplifiedGasolinePhysics physics;
    FourStrokeEventGenerator events;
    auto exhaust = ExhaustGraph::makeForEngine(config);
    auto simulatorOwner = std::make_unique<EngineSimulator>(
        config, ecu, physics, events, exhaust);
    auto& simulator = *simulatorOwner;
    simulator.setPressureSamplingEnabled(true);

    auto eventQueueOwner = std::make_unique<FiringEventQueue>();
    auto pressureQueueOwner = std::make_unique<CylinderPressureQueue>();
    auto runtimeOwner = std::make_unique<EngineRuntime>(config);
    auto& eventQueue = *eventQueueOwner;
    auto& pressureQueue = *pressureQueueOwner;
    auto& audioState = runtimeOwner->audioState();
    auto rendererOwner = std::make_unique<RealtimeEngineAudio>(
        eventQueue, audioState, &pressureQueue, &runtimeOwner->exhaustGraph(),
        &runtimeOwner->engineConfig(), &runtimeOwner->exhaustAcousticSamples());
    auto& renderer = *rendererOwner;
    renderer.prepare(audioRate, samplesPerFrame);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std::vector<float> mix;
    std::vector<float> exhaustStem;
    std::vector<float> otherStems;
    std::array<std::vector<float>, 4> layerStems;
    mix.reserve(static_cast<std::size_t>(seconds * audioRate));
    exhaustStem.reserve(mix.capacity());
    otherStems.reserve(mix.capacity());
    for (auto& layer : layerStems) layer.reserve(mix.capacity());
    juce::AudioBuffer<float> block(2, samplesPerFrame);
    std::array<juce::AudioBuffer<float>, 8> stems;
    for (auto& stem : stems) stem.setSize(2, samplesPerFrame, false, true, false);
    const RealtimeAudioStemBuffers stemBuffers {
        &stems[0], &stems[1], &stems[2], &stems[3],
        &stems[4], &stems[5], &stems[6], &stems[7]
    };

    double realtimeSeconds = 0.0;
    double loadIntegral = 0.0;
    double loadApplied = 0.0;
    const auto steps = static_cast<std::size_t>(seconds / frameDt);
    for (std::size_t step = 0; step < steps; ++step) {
        const auto t = static_cast<double>(step) * frameDt;
        EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = t < 1.1;
        controls.throttle = t < 0.9 ? 0.2 : 0.85;
        if (t >= 1.2) {
            const auto speedError = (simulator.state().rpm - targetRpm)
                / std::max(1.0, targetRpm);
            loadIntegral = std::clamp(loadIntegral + speedError * frameDt * 1.20, 0.0, 0.92);
            const auto targetLoad = std::clamp(loadIntegral + speedError * 0.70, 0.0, 1.0);
            // A brake has a bandwidth of a few hertz; a raw per-frame step
            // excites the engine at the frame rate and puts a comb into the
            // render, which the measurement would then attribute to geometry.
            loadApplied += (1.0 - std::exp(-2.0 * std::numbers::pi * 2.0 * frameDt))
                * (targetLoad - loadApplied);
            controls.load = loadApplied;
        }

        auto frame = simulator.step(frameDt, controls);
        const auto simulationStart = frame.state.simulationTimeSeconds - frameDt;
        for (std::size_t index = 0; index < frame.firingEventCount; ++index) {
            auto event = frame.firingEvents[index];
            const auto fraction = std::clamp(
                (event.timeSeconds - simulationStart) / frameDt, 0.0, 1.0);
            event.timeSeconds = realtimeSeconds + fraction * frameDt;
            (void)eventQueue.tryPush(event);
        }
        CylinderPressureSample sample;
        while (simulator.tryPopCylinderPressureSample(sample)) {
            const auto fraction = std::clamp(
                (sample.timeSeconds - simulationStart) / frameDt, 0.0, 1.0);
            sample.timeSeconds = realtimeSeconds + fraction * frameDt;
            (void)pressureQueue.tryPush(sample);
        }
        ExhaustAcousticSample acousticSample;
        while (simulator.tryPopExhaustAcousticSample(acousticSample)) {
            const auto fraction = std::clamp(
                (acousticSample.timeSeconds - simulationStart) / frameDt,
                0.0, 1.0);
            acousticSample.timeSeconds = realtimeSeconds + fraction * frameDt;
            for (std::size_t eventIndex = 0;
                 eventIndex < acousticSample.reactionEventCount; ++eventIndex) {
                const auto eventFraction = std::clamp(
                    (acousticSample.reactionEvents[eventIndex].timeSeconds
                        - simulationStart) / frameDt, 0.0, 1.0);
                acousticSample.reactionEvents[eventIndex].timeSeconds =
                    realtimeSeconds + eventFraction * frameDt;
            }
            if (!runtimeOwner->exhaustAcousticSamples().tryPush(acousticSample))
                throw std::runtime_error(
                    "geometry harness exhausted its thermoacoustic queue");
        }
        publishAudioFrame(audioState, frame.state,
            { false, controls.starterEngaged, 0.0, 1.0 });
        audioState.producerTimeNanoseconds.store(
            static_cast<std::uint64_t>((realtimeSeconds + frameDt) * 1.0e9),
            std::memory_order_release);
        block.clear();
        renderer.renderWithStems(block, 0, samplesPerFrame, stemBuffers);
        for (int index = 0; index < samplesPerFrame; ++index) {
            mix.push_back(block.getSample(0, index));
            // The exhaust chain is the dry network plus its convolution, which
            // are separate buses only because the IR is applied downstream.
            exhaustStem.push_back(stems[1].getSample(0, index)
                + stems[2].getSample(0, index));
            // combustion + intake + forced induction + mechanical. The jet and
            // pressure-wave buses are diagnostic taps INSIDE the exhaust chain,
            // not additional sources, so summing them here would double-count.
            otherStems.push_back(stems[0].getSample(0, index)
                + stems[3].getSample(0, index)
                + stems[4].getSample(0, index)
                + stems[5].getSample(0, index));
            layerStems[0].push_back(stems[0].getSample(0, index)); // combustion
            layerStems[1].push_back(stems[3].getSample(0, index)); // admission
            layerStems[2].push_back(stems[4].getSample(0, index)); // suralim.
            layerStems[3].push_back(stems[5].getSample(0, index)); // mecanique
        }
        realtimeSeconds += frameDt;
    }

    // Analyse the last two seconds: the speed hold has settled by then, and a
    // window that spans the run-up would report the transient rather than the
    // steady state -- the same trap as every fixed-window gate in this project.
    const auto analysisSamples = std::min<std::size_t>(mix.size(),
        static_cast<std::size_t>(2.0 * audioRate));
    const auto begin = mix.size() - analysisSamples;
    result.mix = analyse(mix, begin);
    result.steadyBegin = begin;
    result.mixSignal = mix;
    result.exhaust = analyse(exhaustStem, begin);
    result.others = analyse(otherStems, begin);
    for (std::size_t layer = 0; layer < result.layers.size(); ++layer)
        result.layers[layer] = analyse(layerStems[layer], begin);
    result.finalRpm = simulator.state().rpm;
    result.networkSubstepHz = simulator.state().exhaustNetworkSubstepFrequencyHz;
    result.physicalActive = renderer.physicalExhaustActive();
    result.maximumRunExhaustPressurePa =
        renderer.maxObservedExhaustPressurePa();
    result.minLevelGain = renderer.minObservedLevelGain();
    result.levelLimitedSamples = renderer.levelLimitedSampleCount();
    result.saturationProcessedSamples =
        renderer.saturationProcessedSampleCount();
    result.softLimitedSamples = renderer.softLimitedSampleCount();
    result.hardClampedSamples = renderer.hardClampedSampleCount();
    result.maxPreLimiter = renderer.maxPreLimiterMagnitude();
    result.maxPostLimiterPeak =
        renderer.maximumPostLimiterSampleMagnitude();
    result.finite = std::all_of(mix.begin(), mix.end(),
        [](float value) { return std::isfinite(value); });
    return result;
}

/** One exhaust a user could plausibly fit. Each is a mutation of the engine's
 *  OWN shipped geometry so the variants stay comparable across the catalogue,
 *  rather than being one absolute pipe imposed on a 700 cc twin and a 6.2 L V8
 *  alike. */
struct Variant final {
    const char* name;
    std::function<void(ExhaustConfig&)> apply;
};

[[nodiscard]] std::vector<Variant> variants() {
    return {
        { "reference", [](ExhaustConfig&) {} },
        // V8 control applied only when the graph contains two four-into-one
        // bank collectors, two silencers and two outlets. The graph rewrite
        // below bypasses only the common X section; all bank primaries,
        // collector bodies, silencers and terminal geometry remain authored.
        { "double-4en1-sans-x", [](ExhaustConfig&) {} },
        // Topology control matching the original user report: preserve each
        // authored primary but terminate it independently, with no collector,
        // crossover or silencer. This is not a length/diameter proxy; the DAG
        // itself is replaced below so a 4-2-1 and four open tubes cannot pass
        // by rendering the same network.
        { "tubes-independants", [](ExhaustConfig&) {} },
        // THE decisive variant, and it changes exactly ONE thing: the silencer
        // body is removed and nothing else moves. That makes the difference
        // against the reference a clean internal single-factor measurement.
        //
        // An earlier version of this harness compared "short AND open" against
        // the reference and called the result silencer authority. It changed
        // two factors at once, which is the trap this project keeps paying for.
        { "sans-silencieux", [](ExhaustConfig& exhaust) {
            exhaust.mufflerChamberDiameterMm = 0.0;
            exhaust.mufflerChamberLengthMm = 0.0;
            exhaust.mufflerPackingFlowResistivityPaSPerM2 = 0.0;
            exhaust.mufflerPackingThicknessMm = 0.0;
            exhaust.mufflerPerforatedOpenAreaRatio = 0.0;
        } },
        // Single-factor length. The quarter-wave of a 250 mm primary is near
        // 340 Hz and of a 900 mm primary near 95 Hz, so these two must not
        // sound alike.
        { "primaire-court", [](ExhaustConfig& exhaust) {
            exhaust.primaryLengthMm = 250.0;
        } },
        { "primaire-long", [](ExhaustConfig& exhaust) {
            exhaust.primaryLengthMm = 900.0;
        } },
        { "petit-diametre", [](ExhaustConfig& exhaust) {
            exhaust.primaryDiameterMm *= 0.65;
            exhaust.collectorDiameterMm *= 0.65;
            exhaust.outletDiameterMm *= 0.65;
        } },
        { "gros-diametre", [](ExhaustConfig& exhaust) {
            exhaust.primaryDiameterMm *= 1.55;
            exhaust.collectorDiameterMm *= 1.55;
            exhaust.outletDiameterMm *= 1.55;
        } },
        // The absorptive half of a real silencer, which the catalogue's road
        // engines do not author at all: the Delany-Bazley porous model exists
        // and is exercised only by the `cp2_absorptive_lab` preset. A reactive
        // chamber redistributes energy between transmitted and reflected; only
        // packing removes it, which is why a purely reactive element cannot
        // reach a production silencer's insertion loss in a reflective system.
        //
        // Values are representative of automotive silencer packing (E-glass or
        // basalt roving behind a perforated core): flow resistivity
        // 10-30 kPa.s/m2, 25-50 mm thick, 20-35 % open area. They are estimates
        // of a material class, not manufacturer data.
        { "avec-garnissage", [](ExhaustConfig& exhaust) {
            exhaust.mufflerPackingFlowResistivityPaSPerM2 = 24'000.0;
            exhaust.mufflerPackingThicknessMm = 35.0;
            exhaust.mufflerPerforatedOpenAreaRatio = 0.28;
        } },
        // Two probes on the SILENCER's own authority, not on realism. Munjal's
        // peak transmission loss is 10 log10[1 + (m - 1/m)^2 / 4] with
        // m = chamber area / duct area, so multiplying the chamber diameter by
        // 1.5 and by 2.3 must raise the insertion loss along a known curve. If
        // it does not, the element is not reaching the output and there is no
        // point making it bigger.
        { "chambre-x2.25", [](ExhaustConfig& exhaust) {
            if (exhaust.mufflerChamberDiameterMm > 1.0)
                exhaust.mufflerChamberDiameterMm *= 1.5;
        } },
        { "chambre-x5.3", [](ExhaustConfig& exhaust) {
            if (exhaust.mufflerChamberDiameterMm > 1.0)
                exhaust.mufflerChamberDiameterMm *= 2.3;
        } },
        // The user's own test case, kept as a compound: "even a basic 125 with
        // a short pipe and no silencer makes an unbelievable racket". Read it
        // against the two single factors above rather than on its own.
        { "court-ouvert", [](ExhaustConfig& exhaust) {
            exhaust.primaryLengthMm = 250.0;
            exhaust.mufflerChamberDiameterMm = 0.0;
            exhaust.mufflerChamberLengthMm = 0.0;
            exhaust.mufflerRestriction = 0.02;
            exhaust.mufflerPackingFlowResistivityPaSPerM2 = 0.0;
            exhaust.mufflerPackingThicknessMm = 0.0;
            exhaust.mufflerPerforatedOpenAreaRatio = 0.0;
        } },
    };
}

[[nodiscard]] EngineConfig withVariant(const EngineConfig& source,
                                       const Variant& variant) {
    auto config = source;
    variant.apply(config.exhaust);
    // A configured path shadows `config.exhaust` entirely (ExhaustGraph selects
    // `legacySinglePath ? config.exhaust : path.geometry`), so a variant that
    // only edited the legacy field would silently measure the SAME exhaust six
    // times on any engine that authors paths. Apply it to every path too.
    for (auto& path : config.exhaustPaths) {
        variant.apply(path.geometry);
        if (!path.network) continue;
        auto& network = *path.network;
        const auto name = std::string_view { variant.name };
        const auto primaryIds = [&] {
            std::set<std::uint32_t> ids;
            for (const auto& connection : network.cylinderConnections)
                ids.insert(connection.componentId);
            return ids;
        }();
        const auto setPrimaryLength = [&](double lengthMm) {
            for (auto& component : network.components)
                if (primaryIds.contains(component.id))
                    component.lengthMm = lengthMm;
        };
        const auto bypassMufflers = [&] {
            for (auto& component : network.components) {
                if (component.type != ExhaustComponentType::muffler) continue;
                // A removed straight-through can leaves the same centreline
                // length occupied by plain pipe. It does not delete the route
                // or move the tailpipe, and it removes both reactive volume
                // and porous material rather than only hiding a scalar flag.
                component.type = ExhaustComponentType::pipe;
                component.volumeLitres = 0.0;
                component.restriction = 0.0;
                component.packingFlowResistivityPaSPerM2 = 0.0;
                component.packingThicknessMm = 0.0;
                component.perforatedOpenAreaRatio = 0.0;
            }
        };
        if (name == "double-4en1-sans-x") {
            std::vector<std::uint32_t> bankMergeIds;
            std::vector<std::uint32_t> mufflerIds;
            std::vector<std::uint32_t> outletIds;
            for (const auto& component : network.components) {
                if (component.type == ExhaustComponentType::muffler)
                    mufflerIds.push_back(component.id);
                else if (component.type == ExhaustComponentType::outlet)
                    outletIds.push_back(component.id);
            }
            for (const auto& component : network.components) {
                if (component.type != ExhaustComponentType::merge) continue;
                const auto primaryInputs = std::count_if(
                    network.connections.begin(), network.connections.end(),
                    [&](const auto& connection) {
                        return connection.toComponentId == component.id
                            && primaryIds.contains(
                                connection.fromComponentId);
                    });
                if (primaryInputs >= 2) bankMergeIds.push_back(component.id);
            }
            std::ranges::sort(bankMergeIds);
            std::ranges::sort(mufflerIds);
            std::ranges::sort(outletIds);
            if (bankMergeIds.size() == 2 && mufflerIds.size() == 2
                    && outletIds.size() == 2) {
                std::set<std::uint32_t> keptIds = primaryIds;
                keptIds.insert(bankMergeIds.begin(), bankMergeIds.end());
                keptIds.insert(mufflerIds.begin(), mufflerIds.end());
                keptIds.insert(outletIds.begin(), outletIds.end());
                std::erase_if(network.components, [&](const auto& component) {
                    return !keptIds.contains(component.id);
                });
                std::erase_if(network.connections, [&](const auto& connection) {
                    return !(primaryIds.contains(connection.fromComponentId)
                            && std::ranges::find(bankMergeIds,
                                connection.toComponentId)
                                != bankMergeIds.end())
                        && !(std::ranges::find(mufflerIds,
                                connection.fromComponentId)
                                != mufflerIds.end()
                            && std::ranges::find(outletIds,
                                connection.toComponentId)
                                != outletIds.end());
                });
                for (std::size_t bank = 0; bank < 2; ++bank)
                    network.connections.push_back(
                        { bankMergeIds[bank], mufflerIds[bank] });
            }
        } else if (name == "tubes-independants") {
            const auto originalComponents = network.components;
            std::vector<ExhaustComponentConfig> independentComponents;
            independentComponents.reserve(primaryIds.size() * 2U);
            for (const auto& component : originalComponents)
                if (primaryIds.contains(component.id))
                    independentComponents.push_back(component);
            auto nextId = std::uint32_t { 1 };
            for (const auto& component : originalComponents)
                nextId = std::max(nextId, component.id + 1U);
            network.connections.clear();
            for (std::size_t index = 0;
                 index < network.cylinderConnections.size(); ++index) {
                const auto primaryId =
                    network.cylinderConnections[index].componentId;
                const auto primary = std::find_if(
                    originalComponents.begin(), originalComponents.end(),
                    [primaryId](const auto& component) {
                        return component.id == primaryId;
                    });
                if (primary == originalComponents.end()) continue;
                ExhaustComponentConfig outlet;
                outlet.id = nextId++;
                outlet.type = ExhaustComponentType::outlet;
                outlet.lengthMm = 0.0;
                outlet.diameterMm = primary->diameterMm;
                outlet.outletDiameterMm = primary->diameterMm;
                outlet.dischargeCoefficient = 1.0;
                outlet.acousticPositionM = path.acousticPositionM;
                outlet.acousticPositionM.x += 0.055
                    * (static_cast<double>(index)
                        - 0.5 * static_cast<double>(
                            network.cylinderConnections.size() - 1U));
                outlet.acousticAxis = path.acousticAxis;
                outlet.acousticTermination = path.acousticTermination;
                independentComponents.push_back(outlet);
                network.connections.push_back({ primaryId, outlet.id });
            }
            network.components = std::move(independentComponents);
        } else if (name == "sans-silencieux") {
            bypassMufflers();
        } else if (name == "primaire-court") {
            setPrimaryLength(250.0);
        } else if (name == "primaire-long") {
            setPrimaryLength(900.0);
        } else if (name == "petit-diametre" || name == "gros-diametre") {
            const auto scale = name == "petit-diametre" ? 0.65 : 1.55;
            for (auto& component : network.components) {
                component.diameterMm *= scale;
                if (component.outletDiameterMm > 0.0)
                    component.outletDiameterMm *= scale;
            }
        } else if (name == "avec-garnissage") {
            for (auto& component : network.components) {
                if (component.type != ExhaustComponentType::muffler) continue;
                component.packingFlowResistivityPaSPerM2 = 24'000.0;
                component.packingThicknessMm = 35.0;
                component.perforatedOpenAreaRatio = 0.28;
            }
        } else if (name == "chambre-x2.25" || name == "chambre-x5.3") {
            const auto areaScale = name == "chambre-x2.25" ? 2.25 : 5.29;
            for (auto& component : network.components)
                if (component.type == ExhaustComponentType::muffler)
                    component.volumeLitres *= areaScale;
        } else if (name == "court-ouvert") {
            setPrimaryLength(250.0);
            bypassMufflers();
        }
    }
    return config;
}

/** A mono signal read from a WAV file, plus what the file said about itself.
 *
 *  This exists so an EXTERNAL recording can be measured by exactly the same
 *  `analyse` and `distance` code as our own renders. A reference simulator with
 *  no offline render and no WAV export can only be captured from its sound card
 *  output; re-implementing the band maths for that path would produce numbers
 *  that look comparable and are not, which would defeat the entire purpose of
 *  measuring it. */
struct WavSignal final {
    std::vector<float> mono;
    double sampleRate { 0.0 };
    std::size_t channels { 0 };
    std::size_t frames { 0 };
    /** Per-channel RMS before the downmix. A capture from a multi-channel
     *  endpoint usually carries signal in two channels and silence in the rest,
     *  and that must be visible rather than quietly averaged away. */
    std::vector<double> channelRms;
};

[[nodiscard]] std::uint32_t readLe32(const std::vector<std::uint8_t>& bytes,
                                     std::size_t offset) {
    return static_cast<std::uint32_t>(bytes[offset])
        | (static_cast<std::uint32_t>(bytes[offset + 1]) << 8)
        | (static_cast<std::uint32_t>(bytes[offset + 2]) << 16)
        | (static_cast<std::uint32_t>(bytes[offset + 3]) << 24);
}

[[nodiscard]] std::uint16_t readLe16(const std::vector<std::uint8_t>& bytes,
                                     std::size_t offset) {
    return static_cast<std::uint16_t>(
        static_cast<std::uint16_t>(bytes[offset])
        | (static_cast<std::uint16_t>(bytes[offset + 1]) << 8));
}

[[nodiscard]] bool readWavMono(const std::filesystem::path& path,
                               WavSignal& signal,
                               std::string& error) {
    std::ifstream stream(path, std::ios::binary);
    if (!stream) { error = "fichier illisible"; return false; }
    const std::vector<std::uint8_t> bytes { std::istreambuf_iterator<char>(stream),
                                            std::istreambuf_iterator<char>() };
    if (bytes.size() < 44
        || std::memcmp(bytes.data(), "RIFF", 4) != 0
        || std::memcmp(bytes.data() + 8, "WAVE", 4) != 0) {
        error = "en-tete RIFF/WAVE absent";
        return false;
    }

    std::uint16_t formatCode = 0;
    std::uint16_t channels = 0;
    std::uint16_t bitsPerSample = 0;
    std::uint32_t sampleRate = 0;
    std::size_t dataBegin = 0;
    std::size_t dataBytes = 0;

    for (std::size_t offset = 12; offset + 8 <= bytes.size();) {
        const auto chunkSize = readLe32(bytes, offset + 4);
        const auto bodyBegin = offset + 8;
        if (bodyBegin + chunkSize > bytes.size()) break;

        if (std::memcmp(bytes.data() + offset, "fmt ", 4) == 0 && chunkSize >= 16) {
            formatCode = readLe16(bytes, bodyBegin);
            channels = readLe16(bytes, bodyBegin + 2);
            sampleRate = readLe32(bytes, bodyBegin + 4);
            bitsPerSample = readLe16(bytes, bodyBegin + 14);
            // WAVE_FORMAT_EXTENSIBLE hides the real code in the sub-format GUID,
            // whose first four bytes are the tag it stands in for.
            if (formatCode == 0xFFFE && chunkSize >= 40)
                formatCode = readLe16(bytes, bodyBegin + 24);
        } else if (std::memcmp(bytes.data() + offset, "data", 4) == 0) {
            dataBegin = bodyBegin;
            dataBytes = chunkSize;
        }
        offset = bodyBegin + chunkSize + (chunkSize % 2);
    }

    if (channels == 0 || sampleRate == 0 || dataBytes == 0) {
        error = "chunk fmt ou data absent";
        return false;
    }
    // The band edges are mapped through the module-level `audioRate`, so a file
    // at another rate would be binned against the wrong frequencies and report a
    // confident, meaningless spectrum. Refuse rather than resample.
    if (std::abs(static_cast<double>(sampleRate) - audioRate) > 0.5) {
        error = "echantillonnage " + std::to_string(sampleRate)
            + " Hz, attendu " + std::to_string(static_cast<int>(audioRate));
        return false;
    }

    const std::size_t bytesPerSample = bitsPerSample / 8U;
    if (bytesPerSample == 0) { error = "bits par echantillon nul"; return false; }
    const std::size_t frames = dataBytes / (bytesPerSample * channels);
    if (frames == 0) { error = "aucune image"; return false; }

    const auto decode = [&](std::size_t index) -> double {
        const auto at = dataBegin + index * bytesPerSample;
        if (formatCode == 3 && bitsPerSample == 32) {
            float value = 0.0F;
            std::memcpy(&value, bytes.data() + at, sizeof(float));
            return static_cast<double>(value);
        }
        if (formatCode == 1 && bitsPerSample == 16)
            return static_cast<double>(static_cast<std::int16_t>(readLe16(bytes, at)))
                / 32'768.0;
        if (formatCode == 1 && bitsPerSample == 32)
            return static_cast<double>(static_cast<std::int32_t>(readLe32(bytes, at)))
                / 2'147'483'648.0;
        return std::numeric_limits<double>::quiet_NaN();
    };

    if (std::isnan(decode(0))) {
        error = "encodage non gere (format " + std::to_string(formatCode) + ", "
            + std::to_string(bitsPerSample) + " bits)";
        return false;
    }

    signal.sampleRate = static_cast<double>(sampleRate);
    signal.channels = channels;
    signal.frames = frames;
    signal.channelRms.assign(channels, 0.0);
    signal.mono.resize(frames);

    std::vector<double> squareSums(channels, 0.0);
    for (std::size_t frame = 0; frame < frames; ++frame) {
        double sum = 0.0;
        for (std::size_t channel = 0; channel < channels; ++channel) {
            const auto value = decode(frame * channels + channel);
            squareSums[channel] += value * value;
            sum += value;
        }
        signal.mono[frame] = static_cast<float>(sum / static_cast<double>(channels));
    }
    for (std::size_t channel = 0; channel < channels; ++channel)
        signal.channelRms[channel] =
            std::sqrt(squareSums[channel] / static_cast<double>(frames));
    return true;
}

/** Write a mono 32-bit float WAV. Exists so our OWN render can be dumped and
 *  then read back through `readWavMono`, which means both sides of an
 *  EngineLab-versus-external comparison traverse byte-for-byte the same
 *  analysis path instead of merely the same formulas. */
[[nodiscard]] bool writeWavMono(const std::filesystem::path& path,
                                const std::vector<float>& samples) {
    std::ofstream stream(path, std::ios::binary);
    if (!stream) return false;

    const auto dataBytes = static_cast<std::uint32_t>(samples.size() * sizeof(float));
    const auto rate = static_cast<std::uint32_t>(audioRate);
    const auto put32 = [&stream](std::uint32_t value) {
        const char bytes[4] { static_cast<char>(value & 0xFF),
                              static_cast<char>((value >> 8) & 0xFF),
                              static_cast<char>((value >> 16) & 0xFF),
                              static_cast<char>((value >> 24) & 0xFF) };
        stream.write(bytes, 4);
    };
    const auto put16 = [&stream](std::uint16_t value) {
        const char bytes[2] { static_cast<char>(value & 0xFF),
                              static_cast<char>((value >> 8) & 0xFF) };
        stream.write(bytes, 2);
    };

    stream.write("RIFF", 4);
    put32(36U + dataBytes);
    stream.write("WAVE", 4);
    stream.write("fmt ", 4);
    put32(16U);
    put16(3U);   // WAVE_FORMAT_IEEE_FLOAT
    put16(1U);
    put32(rate);
    put32(rate * 4U);
    put16(4U);
    put16(32U);
    stream.write("data", 4);
    put32(dataBytes);
    stream.write(reinterpret_cast<const char*>(samples.data()),
                 static_cast<std::streamsize>(dataBytes));
    return static_cast<bool>(stream);
}

/** Compare captured WAV files the same way the rendered variants are compared.
 *  The first file is the reference; every other file is a variant against it. */
[[nodiscard]] int compareWavFiles(const std::filesystem::path& reference,
                                  const std::vector<std::filesystem::path>& variants,
                                  double skipSeconds) {
    std::vector<std::filesystem::path> paths { reference };
    paths.insert(paths.end(), variants.begin(), variants.end());

    std::vector<Spectrum> spectra;
    spectra.reserve(paths.size());

    std::cout << "Sensibilite mesuree sur des ENREGISTREMENTS externes\n"
              << "  meme code d'analyse que les rendus internes, donc les "
                 "chiffres sont comparables.\n"
              << "  " << std::setprecision(2) << skipSeconds
              << " s ignorees en tete de chaque fichier.\n\n";

    for (const auto& path : paths) {
        WavSignal signal;
        std::string error;
        if (!readWavMono(path, signal, error)) {
            std::cerr << "  " << path.filename().string() << " : " << error << '\n';
            return 1;
        }
        const auto begin = std::min(signal.mono.size(),
            static_cast<std::size_t>(std::max(0.0, skipSeconds) * signal.sampleRate));
        // 2^16 samples of FFT is 1.365 s at 48 kHz; below that the window is
        // zero-padded and the low bands stop meaning anything.
        const auto usable = signal.mono.size() - begin;
        if (usable < (std::size_t { 1 } << 16)) {
            std::cerr << "  " << path.filename().string() << " : seulement "
                      << std::setprecision(3)
                      << static_cast<double>(usable) / signal.sampleRate
                      << " s exploitables, il en faut 1.37\n";
            return 1;
        }
        auto spectrum = analyse(signal.mono, begin);

        std::cout << "  " << std::left << std::setw(28) << path.filename().string()
                  << std::right << std::fixed
                  << std::setprecision(2) << std::setw(7)
                  << static_cast<double>(signal.frames) / signal.sampleRate << " s"
                  << "  " << std::setw(2) << signal.channels << " ch"
                  << "  rms " << std::setprecision(6) << std::setw(9) << spectrum.rms
                  << "  " << std::setprecision(1) << std::setw(7)
                  << spectrum.broadbandDb << " dBFS\n";
        printCharacter(characterise(signal.mono, begin, spectrum));
        spectra.push_back(std::move(spectrum));
    }

    if (spectra.size() < 2) {
        std::cerr << "\n  il faut au moins une variante (--wav-variant)\n";
        return 1;
    }

    std::cout << "\n  Ecarts contre " << reference.filename().string() << ":\n"
              << "  " << std::left << std::setw(28) << "variante"
              << std::right << std::setw(10) << "niveau" << std::setw(10) << "forme"
              << std::setw(12) << "bande_max" << "   a\n";
    auto worstShape = 0.0;
    for (std::size_t index = 1; index < spectra.size(); ++index) {
        const auto gap = distance(spectra[0], spectra[index]);
        worstShape = std::max(worstShape, gap.shapeRmsDb);
        std::cout << "  " << std::left << std::setw(28)
                  << paths[index].filename().string()
                  << std::right << std::fixed << std::setprecision(2)
                  << std::setw(9) << gap.levelDb << " dB"
                  << std::setw(9) << gap.shapeRmsDb << " dB"
                  << std::setw(11) << gap.worstBandDb << " dB"
                  << std::setw(8) << std::setprecision(0) << gap.worstBandHz << " Hz\n";
    }
    std::cout << "\n  SENSIBILITE     " << std::fixed << std::setprecision(2)
              << worstShape << " dB de FORME (pire variante)\n";
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    std::filesystem::path catalogRoot = ENGINELAB_CATALOG_ROOT;
    std::string filter = "CP2";
    double targetRpm = 0.0;
    double seconds = 6.0;
    bool listEngines = false;
    std::filesystem::path wavReference;
    std::vector<std::filesystem::path> wavVariants;
    std::filesystem::path wavOut;
    double wavSkipSeconds = 1.0;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--catalog-root" && index + 1 < argc) catalogRoot = argv[++index];
        else if (argument == "--filter" && index + 1 < argc) filter = argv[++index];
        else if (argument == "--rpm" && index + 1 < argc) targetRpm = std::stod(argv[++index]);
        else if (argument == "--seconds" && index + 1 < argc) seconds = std::stod(argv[++index]);
        else if (argument == "--list") listEngines = true;
        else if (argument == "--wav-reference" && index + 1 < argc)
            wavReference = argv[++index];
        else if (argument == "--wav-variant" && index + 1 < argc)
            wavVariants.emplace_back(argv[++index]);
        else if (argument == "--wav-skip" && index + 1 < argc)
            wavSkipSeconds = std::stod(argv[++index]);
        else if (argument == "--wav-out" && index + 1 < argc)
            wavOut = argv[++index];
        else {
            std::cout << "usage: " << argv[0]
                      << " [--catalog-root path] [--filter name-fragment]"
                         " [--rpm N] [--seconds S] [--list]\n"
                         "       " << argv[0]
                      << " --wav-reference ref.wav --wav-variant v.wav [...]"
                         " [--wav-skip S]\n";
            return argument == "--help" ? 0 : 2;
        }
    }

    // Measuring external recordings shares only the analysis, never the
    // catalogue: there is no engine to select and no render to perform.
    if (!wavReference.empty())
        return compareWavFiles(wavReference, wavVariants, wavSkipSeconds);

    const auto catalog = loadEngineCatalog(catalogRoot);
    if (catalog.entries.empty()) {
        std::cerr << "catalogue vide sous " << catalogRoot << '\n';
        return 1;
    }
    if (listEngines) {
        for (const auto& entry : catalog.entries)
            std::cout << "  " << entry.config.name << '\n';
        return 0;
    }

    const auto selected = selectSingleEngineCatalogEntry(catalog.entries, filter);
    if (selected.status == EngineCatalogSelectionStatus::notFound) {
        std::cerr << "aucun moteur ne correspond a \"" << filter << "\"\n";
        return 1;
    }
    if (selected.status == EngineCatalogSelectionStatus::ambiguous) {
        std::cerr << "\"" << filter << "\" correspond a "
                  << selected.matches.size()
                  << " moteurs, precisez:\n";
        for (const auto* match : selected.matches)
            std::cerr << "  " << match->config.audioVoicingKey
                      << "  " << match->config.name << '\n';
        return 1;
    }
    const auto& engine = selected.entry->config;
    if (!(targetRpm > 0.0))
        targetRpm = std::max(engine.idleRpm * 2.0, engine.redlineRpm * 0.55);

    std::cout << "Sensibilite du son a la geometrie d'echappement\n"
              << "  moteur      " << engine.name << '\n'
              << "  regime tenu " << std::fixed << std::setprecision(0)
              << targetRpm << " tr/min, " << std::setprecision(1) << seconds
              << " s par variante\n"
              << "  forme = ecart RMS par tiers d'octave, offset large bande "
                 "retire. C'est LE chiffre.\n\n";

    const auto catalogueVariants = variants();
    std::vector<RenderResult> results;
    results.reserve(catalogueVariants.size());
    for (const auto& variant : catalogueVariants) {
        const auto config = withVariant(engine, variant);
        RenderResult result;
        try {
            result = render(config, targetRpm, seconds);
        } catch (const std::exception& error) {
            std::cerr << "  ECHEC variante " << variant.name << ": "
                      << error.what() << '\n';
            return 1;
        }
        std::cout << "  " << std::left << std::setw(17) << variant.name
                  << std::right
                  << " rms " << std::setw(9) << std::fixed << std::setprecision(6)
                  << result.mix.rms
                  << "  echap " << std::setw(9) << result.exhaust.rms
                  << "  rpm " << std::setw(5) << std::setprecision(0) << result.finalRpm
                  << "  CFL " << std::setw(6) << std::setprecision(1)
                  << result.limitingCflLengthMm << " mm"
                  << "  runPeakPa " << std::setw(7) << std::setprecision(1)
                  << result.maximumRunExhaustPressurePa
                  << "  agc " << std::setw(5) << std::setprecision(3)
                  << result.minLevelGain
                  << "  AGC " << std::setw(8) << result.levelLimitedSamples
                  << "  sat/lim/clamp "
                  << result.saturationProcessedSamples << '/'
                  << result.softLimitedSamples << '/'
                  << result.hardClampedSamples
                  << "  postLimPeak " << std::setprecision(4)
                  << result.maxPostLimiterPeak
                  << (result.physicalActive ? "" : "  [CHEMIN NON PHYSIQUE]")
                  << (result.finite ? "" : "  [NON FINI]")
                  << '\n';
        results.push_back(std::move(result));
    }

    // A render whose engine never reached the commanded speed is not a quiet
    // variant, it is a DEAD ENGINE, and every number downstream of it -- shape,
    // level, insertion loss, character -- describes a stall. The absorber here
    // holds speed with the brake alone at a fixed 0.85 throttle, so a low
    // command saturates it and drags the engine to zero; without this gate the
    // harness reports that as measurement. Refuse instead.
    std::vector<std::size_t> stalled;
    for (std::size_t index = 0; index < results.size(); ++index)
        if (!(results[index].finalRpm > 0.80 * targetRpm)) stalled.push_back(index);
    if (!stalled.empty()) {
        std::cerr << "\n  MOTEUR NON TENU a " << std::fixed << std::setprecision(0)
                  << targetRpm << " tr/min sur " << stalled.size() << " variante(s):\n";
        for (const auto index : stalled)
            std::cerr << "    " << catalogueVariants[index].name << " -> "
                      << std::setprecision(0) << results[index].finalRpm << " tr/min\n";
        std::cerr << "  L'absorbeur tient la vitesse au frein seul, gaz fixes a 0.85,"
                     " donc une consigne\n  basse le sature et cale le moteur. Aucun"
                     " chiffre de ce rendu n'est exploitable.\n"
                     "  Note aussi qu'une consigne basse tenue serait du PLEIN GAZ EN"
                     " SOUS-REGIME,\n  jamais un ralenti: ce harness ne peut pas"
                     " mesurer un ralenti.\n";
        return 1;
    }

    // Geometry comparisons are invalid if a downstream non-linearity removes
    // their level or reshapes their spectrum. Keep the four mechanisms
    // separate so a failure identifies the stage that actually acted.
    const auto shapedOutput = std::ranges::find_if(results, [](const auto& result) {
        return result.minLevelGain < 0.99999
            || result.levelLimitedSamples != 0
            || result.saturationProcessedSamples != 0
            || result.softLimitedSamples != 0
            || result.hardClampedSamples != 0;
    });
    if (shapedOutput != results.end()) {
        const auto index = static_cast<std::size_t>(
            std::distance(results.begin(), shapedOutput));
        std::cerr << "\n  SORTIE NON NEUTRE sur "
                  << catalogueVariants[index].name
                  << ": AGC=" << shapedOutput->levelLimitedSamples
                  << " saturation=" << shapedOutput->saturationProcessedSamples
                  << " softLimit=" << shapedOutput->softLimitedSamples
                  << " hardClamp=" << shapedOutput->hardClampedSamples
                  << ". Comparaison rejetee.\n";
        return 1;
    }

    // The reference render, written out so it can be read back and measured by
    // the same path as an external capture.
    if (!wavOut.empty() && !results.empty()) {
        if (!writeWavMono(wavOut, results[0].mixSignal)) {
            std::cerr << "\n  ecriture impossible: " << wavOut.string() << '\n';
            return 1;
        }
        std::cout << "\n  rendu de reference ecrit: " << wavOut.string() << " ("
                  << std::fixed << std::setprecision(2)
                  << static_cast<double>(results[0].mixSignal.size()) / audioRate
                  << " s)\n";
    }

    std::cout << "\n  Caractere de la reference:\n";
    printCharacter(characterise(results[0].mixSignal, results[0].steadyBegin,
                                results[0].mix));

    std::cout << "\n  Ecarts contre la reference (variante - reference):\n"
              << "  " << std::left << std::setw(17) << "variante"
              << std::right << std::setw(10) << "niveau" << std::setw(10) << "forme"
              << std::setw(12) << "bande_max" << "   a\n";
    auto worstShapeMix = 0.0;
    auto worstShapeExhaust = 0.0;
    for (std::size_t index = 1; index < results.size(); ++index) {
        const auto mixDistance = distance(results[0].mix, results[index].mix);
        const auto exhaustDistance = distance(results[0].exhaust, results[index].exhaust);
        worstShapeMix = std::max(worstShapeMix, mixDistance.shapeRmsDb);
        worstShapeExhaust = std::max(worstShapeExhaust, exhaustDistance.shapeRmsDb);
        std::cout << "  " << std::left << std::setw(17) << catalogueVariants[index].name
                  << std::right << std::fixed << std::setprecision(2)
                  << std::setw(9) << mixDistance.levelDb << " dB"
                  << std::setw(9) << mixDistance.shapeRmsDb << " dB"
                  << std::setw(11) << mixDistance.worstBandDb << " dB"
                  << std::setw(8) << std::setprecision(0) << mixDistance.worstBandHz << " Hz"
                  << "   (echap seul: forme " << std::setprecision(2)
                  << exhaustDistance.shapeRmsDb << " dB, bande_max "
                  << exhaustDistance.worstBandDb << " dB)"
                  << '\n';
    }

    std::cout << "\n  SENSIBILITE     mix " << std::fixed << std::setprecision(2)
              << worstShapeMix << " dB     echappement seul "
              << worstShapeExhaust << " dB\n"
              << "  (pire ecart de FORME sur toutes les variantes)\n";

    // Compare like with like: both spans below are RMS levels over the exact
    // same settled two-second window. The former code compared a full-run,
    // per-path pressure peak against a settled master RMS and then attributed
    // the discrepancy to downstream processing. That conclusion was invalid
    // even when every output-shaper counter was zero.
    auto quietestDb = std::numeric_limits<double>::infinity();
    auto loudestDb = -std::numeric_limits<double>::infinity();
    auto quietestExhaustDb = std::numeric_limits<double>::infinity();
    auto loudestExhaustDb = -std::numeric_limits<double>::infinity();
    for (const auto& result : results) {
        quietestDb = std::min(quietestDb, result.mix.broadbandDb);
        loudestDb = std::max(loudestDb, result.mix.broadbandDb);
        quietestExhaustDb = std::min(
            quietestExhaustDb, result.exhaust.broadbandDb);
        loudestExhaustDb = std::max(
            loudestExhaustDb, result.exhaust.broadbandDb);
    }
    std::cout << "  DYNAMIQUE       rendu " << std::setprecision(2)
              << loudestDb - quietestDb << " dB"
              << "     bus echappement "
              << loudestExhaustDb - quietestExhaustDb << " dB\n"
              << "  (RMS sur la meme fenetre stabilisee pour chaque variante.)\n";

    // Single-factor comparison at this simulated operating point. This is an
    // internal authority check, not a claimed real-world insertion loss.
    const auto silencerVariant = std::ranges::find_if(
        catalogueVariants, [](const auto& variant) {
            return std::string_view { variant.name } == "sans-silencieux";
        });
    const auto silencerIndex = static_cast<std::size_t>(
        std::distance(catalogueVariants.begin(), silencerVariant));
    if (silencerVariant != catalogueVariants.end()
            && results.size() > silencerIndex) {
        const auto renderedDb = results[silencerIndex].mix.broadbandDb
            - results[0].mix.broadbandDb;
        const auto exhaustBusDb = results[silencerIndex].exhaust.broadbandDb
            - results[0].exhaust.broadbandDb;
        std::cout << "  SILENCIEUX      retirer le corps seul: rendu "
                  << std::showpos << std::setprecision(2) << renderedDb
                  << " dB, bus echappement " << exhaustBusDb << " dB"
                  << std::noshowpos
                  << "\n  (A/B interne, meme regime et meme fenetre RMS.)\n";
    }

    // A large stem sensitivity with a small mix sensitivity has exactly one
    // explanation and it is not the exhaust model: the rest of the mix is
    // louder than the exhaust in the bands where the geometry acts. Print the
    // margin per band so that claim is checkable rather than asserted.
    const auto margin = maskingMarginDb(results[0].exhaust, results[0].others);
    std::cout << "\n  Marge de l'echappement sur le RESTE du mix, par tiers "
                 "d'octave (reference):\n   ";
    auto maskedBands = 0;
    auto presentBands = 0;
    for (std::size_t band = 0; band < margin.size(); ++band) {
        if (std::isnan(margin[band])) continue;
        ++presentBands;
        if (margin[band] < 0.0) ++maskedBands;
        std::cout << ' ' << std::setprecision(0) << thirdOctaveCentresHz[band]
                  << "Hz:" << std::showpos << std::setprecision(1)
                  << std::setw(6) << margin[band] << std::noshowpos;
        if ((band + 1) % 6 == 0) std::cout << "\n   ";
    }
    std::cout << "\n  " << maskedBands << " bandes sur " << presentBands
              << " ou l'echappement est SOUS le reste du mix"
                 " -- la geometrie y est inaudible par construction.\n";

    // Which layer does the masking. A sum cannot be acted on; this can.
    constexpr std::array<const char*, 4> layerNames {
        "combustion", "admission", "suralimentation", "mecanique" };
    std::cout << "\n  Couche par couche (reference), niveau relatif a "
                 "l'echappement:\n";
    for (std::size_t layer = 0; layer < layerNames.size(); ++layer) {
        const auto& spectrum = results[0].layers[layer];
        auto dominatedBands = 0;
        for (std::size_t band = 0; band < thirdOctaveCentresHz.size(); ++band) {
            if (!spectrum.bandPresent[band]
                || !results[0].exhaust.bandPresent[band]) continue;
            if (spectrum.bandDb[band] > results[0].exhaust.bandDb[band])
                ++dominatedBands;
        }
        std::cout << "    " << std::left << std::setw(17) << layerNames[layer]
                  << std::right << std::showpos << std::fixed
                  << std::setprecision(1) << std::setw(7)
                  << spectrum.broadbandDb - results[0].exhaust.broadbandDb
                  << " dB" << std::noshowpos
                  << "   domine " << std::setw(2) << dominatedBands << " bandes sur "
                  << presentBands << '\n';
    }
    return 0;
}
