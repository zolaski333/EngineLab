#pragma once

#include <juce_core/juce_core.h>

#include <optional>
#include <vector>

/** `EngineLab --ui-bench ...`: runs one scripted scene in the real window and
    writes what the interface costs, so two builds of the interface can be
    compared on the same machine (CLAUDE.md, Instruments: interleave the runs,
    discard the first, take the minimum of N). It only drives what a user
    could: an engine, the ignition, the starter, the throttle, the dyno, a side
    tab and a tool window. Physics and sound are never touched. */
namespace enginelab::ui {

struct UiBenchSettings final {
    /** A substring of the engine's name (catalogue or saved); empty keeps the default one. */
    juce::String engine;
    double throttle { 0.0 };
    double warmupSeconds { 12.0 };
    double measureSeconds { 20.0 };
    /** The main window's content size, in logical pixels. */
    int width { 1'440 };
    int height { 860 };
    /** juce::Desktop's global scale (1.25 and 1.5 emulate a scaled display). */
    float scale { 1.0F };
    /** dyno, telemetry, audio or diagnostics; empty leaves the panel as it opens. */
    juce::String tab;
    /** A tool window opened after the warm-up: ecu, exhaust, fuel, equipment,
        passby, vehicle, audio or designer. */
    juce::String open;
    /** A top-bar menu shown open for the capture: design, tune, test, listen, more. */
    juce::String menu;
    bool dyno { false };
    /** The per-second CSV; empty writes none. */
    juce::File output;
    /** Written ("x y w h", physical pixels of the main window's content)
        once the scene is ready, then the bench holds still for
        `captureHoldSeconds` so a screen capture can be taken. */
    juce::File captureMarker;
    double captureHoldSeconds { 2.0 };
};

/** The settings from the command line, or none without `--ui-bench`. An
    option it does not know is an error (returned in `error`). */
[[nodiscard]] std::optional<UiBenchSettings> parseUiBench(const juce::StringArray& arguments,
                                                          juce::String& error);

/** CPU seconds the calling thread has used so far. */
[[nodiscard]] double currentThreadCpuSeconds() noexcept;
/** CPU seconds the whole process has used so far (every thread). */
[[nodiscard]] double processCpuSeconds() noexcept;

/** One measured second. */
struct UiBenchSample final {
    double wallSeconds {};
    double rpm {};
    double realtimeFactor {};
    double viewFps {};
    double viewRenderCpuMs {};
    /** Message-thread CPU used during the second, in ms (the panels' timer,
        their painting, the overlays). */
    double messageThreadCpuMs {};
    /** Whole-process CPU during the second, in ms (all cores together). */
    double processCpuMs {};
};

/** The CSV: a header naming the scene, then one row per second and a
    summary row (median of each column). */
[[nodiscard]] juce::String uiBenchCsv(const UiBenchSettings&, const juce::String& engineName,
                                      const std::vector<UiBenchSample>&);

} // namespace enginelab::ui
