#pragma once

#include <juce_gui_basics/juce_gui_basics.h>
#include <array>

namespace enginelab {

enum class AppAction : std::size_t {
    ignition, starter, dyno, dynoHold, fullscreen, dynoStats, pause, nextScreen,
    shiftUp, shiftDown, brake, throttleIdle, throttleQuarter, throttleHalf, throttleFull,
    timeQuarter, timeHalf, timeNormal, timeDouble, timeQuadruple,
    layerUp, layerDown, exhaustPreset, clutchHold, clutchDecrease, clutchIncrease,
    wheelDynoRpm, wheelVolume, wheelConvolution, wheelHighGain, wheelLowNoise,
    wheelHighNoise, wheelCombustion, wheelExhaust, wheelIntake, wheelMechanical,
    wheelSimulationRate, wheelFineThrottle,
    // ActionMap::entries_ is initialised POSITIONALLY against this order, so new
    // actions go here, at the end, immediately before `count`. Inserting one in
    // the middle silently rebinds every action after it.
    dynoRamp,
    abSwitch,
    cruise,
    automaticGearbox,
    datalog,
    engineBrake,
    exhaustValve,
    nitrous,
    // The keyboard's way to every window and command: the top bar's buttons
    // take no focus (the main component keeps the keyboard), so the `...`
    // menu, which lists every window, and the engine picker have their own
    // chords (audit UA-07).
    moreMenu,
    engineMenu,
    count
};

class ActionMap final {
public:
    /** The defaults, then the user's `keybindings.json` over them. */
    ActionMap();
    /** The defaults only: no file is read or written (tests). */
    struct DefaultsOnly final {};
    explicit ActionMap(DefaultsOnly);
    [[nodiscard]] bool matches(AppAction action, const juce::KeyPress& key) const noexcept;
    [[nodiscard]] bool isDown(AppAction action) const noexcept;
    [[nodiscard]] juce::String toJson() const;
    [[nodiscard]] bool fromJson(const juce::String& json, juce::String& error);
    void load();
    void save() const;
    [[nodiscard]] juce::String shortcut(AppAction action) const;

private:
    struct Entry final {
        const char* id;
        juce::KeyPress key;
    };
    [[nodiscard]] static juce::File settingsFile();
    /** True when one key event can match both bindings: Shift is ignored by
     *  a binding that does not ask for it (`matches`). */
    [[nodiscard]] static bool sameEvent(const juce::KeyPress&, const juce::KeyPress&) noexcept;
    /** Keys the main component handles before any action: Escape, Return,
     *  F1-F12 and the save chords Ctrl+S / Ctrl+Shift+S. */
    [[nodiscard]] static bool reserved(const juce::KeyPress&) noexcept;
    std::array<Entry, static_cast<std::size_t>(AppAction::count)> entries_;
};

} // namespace enginelab
