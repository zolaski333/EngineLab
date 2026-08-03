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
// Reference points for reading the output, from the acoustics literature and
// never from this simulator: a straight-through absorptive silencer gives
// 10-25 dB of insertion loss over its working band, and a reactive expansion
// chamber gives 15-30 dB at its tuned frequencies with near 0 dB at its pass
// frequencies. Changing a primary from 250 to 900 mm moves the tuned quarter-
// wave from about 340 Hz to about 95 Hz, which is two octaves.

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
#include <filesystem>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <numbers>
#include <string>
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

struct RenderResult final {
    Spectrum mix;
    Spectrum exhaust;
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
    double shortestCellMm { 0.0 };
    double networkSubstepHz { 0.0 };
    bool physicalActive { false };
    bool finite { true };
    /** Pressure the exhaust chain actually delivered to the observer, before
     *  any level control. This is the physical answer to "is it louder"; the
     *  rendered RMS is that answer after the safety leveler has had its say. */
    double exhaustPeakPa { 0.0 };
    /** The two ways a level change can be removed after the fact. A slow AGC
     *  sitting below 1 means the safety leveler is doing steady-state gain
     *  work, which would turn "I removed the silencer" into "it sounds
     *  different but no louder" -- exactly the reported symptom. */
    double minLevelGain { 1.0 };
    std::uint64_t levelLimitedSamples { 0 };
    double maxPreLimiter { 0.0 };
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
        gasdynamics::ExhaustNetworkDiscretisation mesh;
        mesh.targetCellLengthM = 0.300;
        mesh.minimumCellsPerDuct = 1;
        mesh.maximumCellsPerDuct = 64;
        mesh.maximumTotalCells = 1'024;
        const auto layout = gasdynamics::ExhaustNetworkLayout::compile(graph, mesh);
        result.shortestCellMm = layout.minimumCellLengthM() * 1'000.0;
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
        &runtimeOwner->engineConfig());
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
    result.exhaust = analyse(exhaustStem, begin);
    result.others = analyse(otherStems, begin);
    for (std::size_t layer = 0; layer < result.layers.size(); ++layer)
        result.layers[layer] = analyse(layerStems[layer], begin);
    result.finalRpm = simulator.state().rpm;
    result.networkSubstepHz = simulator.state().exhaustNetworkSubstepFrequencyHz;
    result.physicalActive = renderer.physicalExhaustActive();
    result.exhaustPeakPa = renderer.maxObservedExhaustPressurePa();
    result.minLevelGain = renderer.minObservedLevelGain();
    result.levelLimitedSamples = renderer.levelLimitedSampleCount();
    result.maxPreLimiter = renderer.maxPreLimiterMagnitude();
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
        // THE decisive variant, and it changes exactly ONE thing: the silencer
        // body is removed and nothing else moves. That makes the difference
        // against the reference a clean insertion loss, comparable to the
        // 20-30 dB a production silencer is worth.
        //
        // An earlier version of this harness compared "short AND open" against
        // the reference and called the result silencer authority. It changed
        // two factors at once, which is the trap this project keeps paying for.
        { "sans-silencieux", [](ExhaustConfig& exhaust) {
            exhaust.mufflerChamberDiameterMm = 0.0;
            exhaust.mufflerChamberLengthMm = 0.0;
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
    for (auto& path : config.exhaustPaths) variant.apply(path.geometry);
    return config;
}

} // namespace

int main(int argc, char** argv) {
    std::filesystem::path catalogRoot = ENGINELAB_CATALOG_ROOT;
    std::string filter = "CP2";
    double targetRpm = 0.0;
    double seconds = 6.0;
    bool listEngines = false;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--catalog-root" && index + 1 < argc) catalogRoot = argv[++index];
        else if (argument == "--filter" && index + 1 < argc) filter = argv[++index];
        else if (argument == "--rpm" && index + 1 < argc) targetRpm = std::stod(argv[++index]);
        else if (argument == "--seconds" && index + 1 < argc) seconds = std::stod(argv[++index]);
        else if (argument == "--list") listEngines = true;
        else {
            std::cout << "usage: " << argv[0]
                      << " [--catalog-root path] [--filter name-fragment]"
                         " [--rpm N] [--seconds S] [--list]\n";
            return argument == "--help" ? 0 : 2;
        }
    }

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

    // Every engine selector in this repo is an unanchored substring match and
    // the catalogue contains names that contain each other, so silently taking
    // the first hit is how a measurement ends up being about a different engine
    // than the one it names.
    std::vector<const EngineConfig*> matches;
    for (const auto& entry : catalog.entries)
        if (entry.config.name.find(filter) != std::string::npos)
            matches.push_back(&entry.config);
    if (matches.empty()) {
        std::cerr << "aucun moteur ne correspond a \"" << filter << "\"\n";
        return 1;
    }
    if (matches.size() > 1) {
        std::cerr << "\"" << filter << "\" correspond a " << matches.size()
                  << " moteurs, precisez:\n";
        for (const auto* match : matches) std::cerr << "  " << match->name << '\n';
        return 1;
    }
    const auto& engine = *matches.front();
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
        auto result = render(config, targetRpm, seconds);
        std::cout << "  " << std::left << std::setw(17) << variant.name
                  << std::right
                  << " rms " << std::setw(9) << std::fixed << std::setprecision(6)
                  << result.mix.rms
                  << "  echap " << std::setw(9) << result.exhaust.rms
                  << "  rpm " << std::setw(5) << std::setprecision(0) << result.finalRpm
                  << "  maille " << std::setw(6) << std::setprecision(1)
                  << result.shortestCellMm << " mm"
                  << "  Pa " << std::setw(7) << std::setprecision(1)
                  << result.exhaustPeakPa
                  << "  agc " << std::setw(5) << std::setprecision(3)
                  << result.minLevelGain
                  << "  limite " << std::setw(8) << result.levelLimitedSamples
                  << (result.physicalActive ? "" : "  [CHEMIN NON PHYSIQUE]")
                  << (result.finite ? "" : "  [NON FINI]")
                  << '\n';
        results.push_back(std::move(result));
    }

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

    // The number a listener means by "an open pipe makes an unbelievable
    // racket". Shape is tone; this is loudness, and the two are independent --
    // a chain that changes tone but not level sounds "different" and never
    // "louder", which is precisely the reported complaint.
    //
    // Reference, from acoustics and not from this simulator: fitting a
    // production silencer to an open exhaust is worth 20-30 dB of insertion
    // loss. Anything under about 10 dB here means the silencer does not
    // silence.
    auto quietestDb = std::numeric_limits<double>::infinity();
    auto loudestDb = -std::numeric_limits<double>::infinity();
    auto quietestPa = std::numeric_limits<double>::infinity();
    auto loudestPa = -std::numeric_limits<double>::infinity();
    for (const auto& result : results) {
        quietestDb = std::min(quietestDb, result.mix.broadbandDb);
        loudestDb = std::max(loudestDb, result.mix.broadbandDb);
        quietestPa = std::min(quietestPa, result.exhaustPeakPa);
        loudestPa = std::max(loudestPa, result.exhaustPeakPa);
    }
    const auto physicalSpanDb = 20.0 * std::log10(
        std::max(loudestPa, 1.0e-9) / std::max(quietestPa, 1.0e-9));
    std::cout << "  DYNAMIQUE       rendu " << std::setprecision(2)
              << loudestDb - quietestDb << " dB"
              << "     physique (Pa observateur) " << physicalSpanDb << " dB\n"
              << "  (du plus silencieux au plus bruyant. Un vrai silencieux "
                 "vaut 20-30 dB.\n"
              << "   Si physique >> rendu, la difference est retiree APRES la "
                 "physique.)\n";

    // The single-factor insertion loss, called out on its own line because it
    // is the one number with a literature value to check against and the one
    // the user's complaint is literally about.
    const auto silencerIndex = std::size_t { 1 }; // "sans-silencieux"
    if (results.size() > silencerIndex) {
        const auto renderedDb = results[silencerIndex].mix.broadbandDb
            - results[0].mix.broadbandDb;
        const auto physicalDb = 20.0 * std::log10(
            std::max(results[silencerIndex].exhaustPeakPa, 1.0e-9)
            / std::max(results[0].exhaustPeakPa, 1.0e-9));
        std::cout << "  SILENCIEUX      retirer le corps seul: rendu "
                  << std::showpos << std::setprecision(2) << renderedDb
                  << " dB, physique " << physicalDb << " dB" << std::noshowpos
                  << "\n  (litterature: +20 a +30 dB. C'est une perte "
                     "d'insertion a un seul facteur.)\n";
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
