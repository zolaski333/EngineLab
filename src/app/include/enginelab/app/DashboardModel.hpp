#pragma once

#include <enginelab/audio/OfflineAudioExporter.hpp>
#include <enginelab/audio/RealtimeEngineAudio.hpp>
#include <enginelab/diagnostics/EngineDiagnostics.hpp>
#include <enginelab/foundation/EngineTypes.hpp>
#include <enginelab/render/RenderSnapshot.hpp>
#include <juce_gui_basics/juce_gui_basics.h>
#include <array>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace enginelab {

/** The last ten seconds of engine state, sampled at the UI rate. */
class TelemetryRing final {
public:
    static constexpr std::size_t capacity = 300;
    void push(const EngineState& state) noexcept;
    void clear() noexcept { write_ = 0; count_ = 0; }
    [[nodiscard]] std::size_t size() const noexcept { return count_; }
    /** Index 0 is the oldest retained sample. */
    [[nodiscard]] const EngineState& at(std::size_t index) const noexcept;

private:
    std::array<EngineState, capacity> samples_ {};
    std::size_t write_ { 0 };
    std::size_t count_ { 0 };
};

/** User-chosen name, colour and visibility of a saved dyno run. */
struct DynoCurvePresentation final {
    juce::String name;
    std::uint32_t colour { 0xffffffffU };
    bool visible { true };
};

/** Health counters of the real-time chain, read once per UI frame. */
struct RuntimeHealth final {
    bool available { false };
    double realtimeFactor { 1.0 };
    double timeScale { 1.0 };
    bool paused { false };
    std::uint64_t droppedFirings {};
    std::uint64_t droppedPressureSamples {};
    std::uint64_t droppedExhaustAcousticSamples {};
    std::uint64_t droppedDynoCycles {};
    std::uint64_t timingOverruns {};
    bool loadProtectionActive { false };
    std::uint64_t loadProtectionActivations {};
    bool audioAvailable { false };
    std::uint64_t droppedReactionEvents {};
    std::uint64_t reactionPressureLimitedSamples {};
    std::uint64_t lateAudioEvents {};
    std::uint64_t droppedPendingAudioEvents {};
    float minimumLevelGain { 1.0F };
    std::uint64_t levelLimitedSamples {};
    std::uint64_t saturationProcessedSamples {};
    std::uint64_t softLimitedSamples {};
    std::uint64_t hardClampedSamples {};
    float maximumPostLimiterMagnitude { 0.0F };
};

/** What the audio panel shows: the mix and which of its controls apply. */
struct AudioView final {
    OfflineAudioMix mix;
    int exhaustPresetIndex { 0 };
    bool physicalExhaustTopology { false };
    bool physicalIntakeTopology { false };
    bool structuralRadiationActive { false };
    bool forcedInductionAcousticsActive { false };
    bool impulseResponseAvailable { false };
    bool impulseResponseLoadError { false };
    juce::String impulseResponseStatus;
};

/** Everything the panels of the main window draw. The main component owns it
    and refreshes it once per UI frame; panels only read it. */
struct DashboardModel final {
    explicit DashboardModel(const EngineConfig& engineConfig) : config(engineConfig) {}

    const EngineConfig& config;
    EngineState state;
    RenderSnapshot renderSnapshot;
    TelemetryRing telemetry;
    DynoRun currentRun;
    std::vector<DynoRun> archivedRuns;
    std::unordered_map<std::uint64_t, DynoCurvePresentation> presentations;
    int selectedRun { -1 };
    std::vector<Diagnostic> diagnostics;
    RuntimeHealth health;
    AudioView audio;
    bool dynoRunning { false };
    bool scriptLive { false };
    std::uint64_t scriptRevision {};
    std::uint64_t voicingReloadCount {};

    [[nodiscard]] const DynoCurvePresentation* presentationFor(std::uint64_t runId) const;
    [[nodiscard]] const DynoRun* selectedArchivedRun() const;
};

namespace ui {
[[nodiscard]] const char* runningStateName(RunningState) noexcept;
[[nodiscard]] const char* layoutName(EngineLayout) noexcept;
[[nodiscard]] juce::String gearName(int gear);
[[nodiscard]] const char* dynoModeToken(DynoMode) noexcept;
[[nodiscard]] const char* dynoStatusToken(DynoRunStatus) noexcept;
[[nodiscard]] const char* dynoStatusLabel(DynoRunStatus) noexcept;
[[nodiscard]] const char* dynoStopReasonToken(DynoStopReason) noexcept;

/** "Inline-4 · 1 998 cm³ · 86 × 86 mm · turbo" */
[[nodiscard]] juce::String engineSpecLine(const EngineConfig&);
/** Integer with a thin space every three digits ("10 000"). */
[[nodiscard]] juce::String groupedInteger(double value);
} // namespace ui

} // namespace enginelab
