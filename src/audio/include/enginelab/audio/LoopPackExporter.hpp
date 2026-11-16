#pragma once

#include <enginelab/audio/OfflineAudioExporter.hpp>

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace enginelab {

/** A pack of seamless loops for a game's sample-based engine sound (BeamNG's
 *  `soundConfig` and `soundConfigExhaust`, for instance).
 *
 * Each point is held at a fixed speed, on load (full throttle against the
 * dyno's absorber) and off load (throttle shut, the shaft motored at that
 * speed as a car on the overrun is), and recorded once it has settled. A loop
 * spans a whole number of the engine's cycles, several of them so the cycle
 * to cycle life is kept, and its end is cross-faded into its start over a
 * whole number of cycles. Every file of the pack takes the same gain.
 */
struct LoopPackRequest final {
    EngineConfig engine;
    /** The ECU tables the render runs, as a running engine's store holds
     *  them; absent, the tables the configuration implies. An Audio workshop
     *  export ran the default tables and lost the live tuning (audit UA-01). */
    std::shared_ptr<const calibration::CalibrationSnapshot> calibration;
    OfflineAudioMix mix;
    std::filesystem::path outputDirectory;
    /** Root used to resolve engine-authored relative impulse-response paths. */
    std::filesystem::path assetRoot;
    /** 48,000 or 96,000. */
    std::uint32_t sampleRateHz { 48'000 };
    /** The points: the idle, then every `stepRpm` from a step above it. */
    double stepRpm { 500.0 };
    /** Lowest point; 0 starts at the engine's idle. */
    double lowestRpm { 0.0 };
    /** Highest point; 0 ends just under the redline and the rev limit. */
    double highestRpm { 0.0 };
    /** Length a loop aims for; it is rounded to whole cycles (4 at least). */
    double loopSeconds { 2.0 };
    bool onLoad { true };
    bool offLoad { true };
    /** An Ogg Vorbis copy of every loop beside the WAV. */
    bool writeOgg { false };
    /** Peak of the pack's loudest file. */
    double packPeakDbfs { -1.0 };
    bool loadAuthoredImpulseResponses { true };
    bool overwriteExistingFiles { false };
};

/** One held point and its loops. */
struct LoopPackLoop final {
    bool onLoad { true };
    double requestedRpm { 0.0 };
    /** Mean, minimum and maximum shaft speed over the recorded loop. */
    double meanRpm { 0.0 };
    double minimumRpm { 0.0 };
    double maximumRpm { 0.0 };
    /** False when the point had not settled by the hold's time limit; the
     *  loop is recorded anyway and the result carries a warning. */
    bool settled { false };
    double settleSeconds { 0.0 };
    std::uint32_t cycles { 0 };
    std::uint64_t frames { 0 };
    /** Crank degrees the simulator travelled over the loop. */
    double crankDegrees { 0.0 };
    double meanManifoldPressureKpa { 0.0 };
    /** After the pack gain, before quantisation. */
    double enginePeak { 0.0 };
    double exhaustPeak { 0.0 };
    double engineRms { 0.0 };
    double exhaustRms { 0.0 };
    std::filesystem::path engineFile;
    std::filesystem::path exhaustFile;
};

struct LoopPackResult final {
    bool success { false };
    bool cancelled { false };
    std::string error;
    std::vector<std::string> warnings;
    std::vector<std::filesystem::path> files;
    std::vector<LoopPackLoop> loops;
    /** The one gain every file took, and the peak of the loudest. */
    double packGainDb { 0.0 };
    double packPeak { 0.0 };
    double cycleDegrees { 720.0 };
    /** Firings per 720 crank degrees: what a sample player that derives the
     *  firing frequency as rpm / 120 x cylinders must be told (twice the
     *  cylinders of a two-stroke, two per rotor of a rotary). */
    std::uint32_t equivalentFourStrokeCylinders { 0 };
    bool oggWritten { false };
};

/** The speeds a pack holds for `engine` with these settings. */
[[nodiscard]] std::vector<double> loopPackPoints(
    const EngineConfig& engine, double stepRpm,
    double lowestRpm = 0.0, double highestRpm = 0.0);

/** Firings per 720 crank degrees (see LoopPackResult). */
[[nodiscard]] std::uint32_t equivalentFourStrokeCylinders(
    const EngineConfig& engine) noexcept;

/** Render the pack. `progress` is called outside the render loop with the
 *  share of points done; returning false cancels the job and removes the
 *  files it created. */
[[nodiscard]] LoopPackResult exportLoopPack(
    const LoopPackRequest& request,
    const OfflineAudioProgress& progress = {});

} // namespace enginelab
