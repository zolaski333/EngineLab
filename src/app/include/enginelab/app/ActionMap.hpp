#pragma once

#include <juce_gui_basics/juce_gui_basics.h>
#include <array>

namespace enginelab {

enum class AppAction : std::size_t {
    ignition, starter, dyno, dynoHold, fullscreen, dynoStats, pause, nextScreen,
    shiftUp, shiftDown, throttleIdle, throttleQuarter, throttleHalf, throttleFull,
    timeQuarter, timeHalf, timeNormal, timeDouble, timeQuadruple,
    layerUp, layerDown, exhaustPreset, clutchDecrease, clutchIncrease,
    wheelDynoRpm, wheelVolume, wheelConvolution, wheelHighGain, wheelLowNoise,
    wheelHighNoise, wheelCombustion, wheelExhaust, wheelIntake, wheelMechanical,
    wheelSimulationRate, wheelFineThrottle, count
};

class ActionMap final {
public:
    ActionMap();
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
    std::array<Entry, static_cast<std::size_t>(AppAction::count)> entries_;
};

} // namespace enginelab
