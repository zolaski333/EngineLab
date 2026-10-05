#pragma once

#include <enginelab/app/DashboardModel.hpp>
#include <enginelab/app/Widgets.hpp>
#include <functional>

namespace enginelab::ui {

/** Brand, engine picker, running-state pill and the tool buttons. */
class TopBar final : public juce::Component {
public:
    TopBar();
    void setPresetNames(const juce::StringArray& names);
    /** `presetIndex` is -1 for an engine that is not one of the presets. */
    void setEngine(const EngineConfig&, int presetIndex);
    void refresh(const DashboardModel&);

    std::function<void(int presetIndex)> onEngineSelected;
    ActionButton exhaustButton { "Exhaust", ActionButton::Style::tool };
    ActionButton ecuButton { "ECU", ActionButton::Style::tool };
    ActionButton audioButton { "Audio", ActionButton::Style::tool };
    ActionButton moreButton { {}, ActionButton::Style::tool };

    void paint(juce::Graphics&) override;
    void resized() override;
    static constexpr int height = 56;

private:
    class EnginePicker final : public juce::Button {
    public:
        EnginePicker();
        void setEngine(const juce::String& name, const juce::String& spec);
        void paintButton(juce::Graphics&, bool highlighted, bool down) override;
    private:
        juce::String name_;
        juce::String spec_;
    };

    void showEngineMenu();
    EnginePicker picker_;
    juce::StringArray presetNames_;
    int presetIndex_ { -1 };
    juce::String stateText_;
    juce::Colour stateColour_;
    juce::Colour stateFill_;
    juce::Rectangle<float> pillArea_;
};

} // namespace enginelab::ui
