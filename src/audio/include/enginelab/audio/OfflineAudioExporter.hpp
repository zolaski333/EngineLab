#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

#include <cstdint>
#include <filesystem>
#include <functional>
#include <string>
#include <string_view>
#include <vector>

namespace enginelab {

/** File encodings supported by the deterministic offline renderer. */
enum class OfflineWaveFormat {
    pcm24,
    float32
};

/** One control segment in an offline render scenario.
 *
 * Throttle, load and target speed are linearly interpolated over the segment.
 * When `governed` is true, the load fields are ignored and a deterministic
 * absorber controller follows targetRpmStart/targetRpmEnd.
 */
struct OfflineAudioStage final {
    std::string name { "stage" };
    double durationSeconds { 1.0 };
    bool ignitionEnabled { true };
    bool starterEngaged { false };
    bool governed { true };
    double throttleStart { 0.2 };
    double throttleEnd { 0.2 };
    double loadStart { 0.0 };
    double loadEnd { 0.0 };
    double targetRpmStart { 1'200.0 };
    double targetRpmEnd { 1'200.0 };
};

/** Ordered, machine-readable drive cycle for repeatable sound exports. */
struct OfflineAudioScenario final {
    std::string name { "showcase" };
    std::vector<OfflineAudioStage> stages;
};

/** The same user-facing mix controls as the realtime application. */
struct OfflineAudioMix final {
    double volume { 1.0 };
    double convolution { 0.45 };
    double highFrequencyGain { 1.0 };
    double lowFrequencyNoise { 0.35 };
    double highFrequencyNoise { 0.35 };
    double combustionGain { 1.0 };
    double exhaustGain { 1.0 };
    double intakeGain { 0.85 };
    double mechanicalGain { 0.70 };
};

struct OfflineAudioExportRequest final {
    EngineConfig engine;
    OfflineAudioScenario scenario;
    OfflineAudioMix mix;
    std::filesystem::path outputDirectory;
    /** Root used to resolve engine-authored relative impulse-response paths. */
    std::filesystem::path assetRoot;
    std::uint32_t sampleRateHz { 96'000 };
    OfflineWaveFormat format { OfflineWaveFormat::pcm24 };
    bool writeStems { true };
    bool loadAuthoredImpulseResponses { true };
    bool overwriteExistingFiles { false };
};

/** Speed actually reached inside one scenario stage.
 *
 * A scenario declares a *requested* trajectory; the engine answers it. Nothing
 * used to publish the answer, so an "idle" stage could hold twice the engine's
 * real idle and the render still looked correct. Judging the speed from the
 * rendered audio does not substitute for this: the firing fundamental of a big
 * twin at idle is near 13 Hz, and a harmonic-product estimate picks the wrong
 * octave there. These are simulator telemetry, not an audio estimate.
 */
struct OfflineAudioStageSpeed final {
    std::string name;
    double requestedRpmStart { 0.0 };
    double requestedRpmEnd { 0.0 };
    double meanRpm { 0.0 };
    double minimumRpm { 0.0 };
    double maximumRpm { 0.0 };
    /** Mean over the last quarter of the stage: what the stage settled to. */
    double settledRpm { 0.0 };
    bool governed { false };
};

/** Auditable result returned by the shared CLI/application export path. */
struct OfflineAudioExportResult final {
    bool success { false };
    bool cancelled { false };
    std::string error;
    std::vector<std::string> warnings;
    std::vector<std::filesystem::path> files;
    std::uint64_t renderedFrames { 0 };
    double durationSeconds { 0.0 };
    double masterPeak { 0.0 };
    double masterRms { 0.0 };
    std::size_t loadedImpulseResponses { 0 };
    std::size_t authoredImpulseResponses { 0 };
    bool physicalExhaustActive { false };
    bool compiledExhaustTopologyActive { false };
    bool compiledIntakeTopologyActive { false };
    bool forcedInductionAcousticsActive { false };
    std::uint64_t delayTruncationCount { 0 };
    std::uint64_t invalidBoundarySampleCount { 0 };
    std::uint64_t legacyPathSampleCount { 0 };
    std::uint64_t droppedFiringEvents { 0 };
    std::uint64_t droppedPressureSamples { 0 };
    std::vector<OfflineAudioStageSpeed> stageSpeeds;
};

/** Return the engine-relative crank/idle/rev/limiter/overrun showcase. */
[[nodiscard]] OfflineAudioScenario makeDefaultOfflineAudioScenario(
    const EngineConfig& engine);

/** Parse and strictly validate a schema-version 1 scenario JSON file. */
[[nodiscard]] bool loadOfflineAudioScenario(
    const std::filesystem::path& file,
    OfflineAudioScenario& destination,
    std::string& error);

/** Serialize a scenario in the public schema used by the CLI and manifest. */
[[nodiscard]] bool saveOfflineAudioScenario(
    const std::filesystem::path& file,
    const OfflineAudioScenario& scenario,
    std::string& error);

/** Run the shipping simulation/audio path faster than wall clock and stream WAVs.
 *
 * The callback is invoked outside the audio render loop at a bounded cadence.
 * Returning false cancels the job and removes only the partial files created by
 * that invocation.
 */
using OfflineAudioProgress =
    std::function<bool(double fraction, std::string_view stageName)>;

[[nodiscard]] OfflineAudioExportResult exportOfflineAudio(
    const OfflineAudioExportRequest& request,
    const OfflineAudioProgress& progress = {});

[[nodiscard]] const char* offlineWaveFormatName(
    OfflineWaveFormat format) noexcept;

} // namespace enginelab
