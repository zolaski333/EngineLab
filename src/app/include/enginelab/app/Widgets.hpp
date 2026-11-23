#pragma once

#include <juce_gui_basics/juce_gui_basics.h>
#include <functional>

/** Small reusable controls of the main window. None of them takes keyboard
    focus there: the main component owns the keyboard, because every key is a
    simulator control (arrows shift gears, Enter reloads, Space is the fine
    throttle modifier). A tool window, which has no simulator keys, makes its
    own controls reachable with Tab (`enableKeyboardNavigation`). */
namespace enginelab::ui {

/** A hairline button with an optional key cap on its right and an optional
    icon. A button that carries a state (a toggle, or one given `setEngaged`)
    shows a lamp on its left: lit when on, dark when off. */
class ActionButton final : public juce::Button {
public:
    /** `toggleSwitch` draws a track and knob instead of a lamp (the
        ignition); `momentary` inverts while held or engaged (the starter). */
    enum class Style { standard, primary, tool, compact, toggleSwitch, momentary };

    explicit ActionButton(const juce::String& label = {}, Style = Style::standard);
    void setStyle(Style);
    void setKeyCap(const juce::String& keyCap);
    /** A stroked outline in a 24 x 24 box, drawn left of the text. */
    void setIcon(juce::Path icon);
    /** Shows the "engaged" state (a lit lamp) without being a toggle. */
    void setEngaged(bool);
    [[nodiscard]] bool engaged() const noexcept { return engaged_; }
    [[nodiscard]] int idealWidth() const;
    /** Takes Tab focus; Space or Return then clicks it. */
    void setKeyboardNavigable(bool);

protected:
    void paintButton(juce::Graphics&, bool highlighted, bool down) override;
    void focusGained(FocusChangeType) override { repaint(); }
    void focusLost(FocusChangeType) override { repaint(); }

private:
    Style style_;
    juce::String keyCap_;
    juce::Path icon_;
    bool engaged_ { false };
    bool stateful_ { false };
};

/** A square indicator lamp: dark when off, lit in `colour` when on. */
void drawLamp(juce::Graphics&, juce::Rectangle<float> area, bool lit, juce::Colour colour);

/** Row of mutually exclusive options. `pill` is a recessed strip whose chosen
    option is inverted (light, dark text), `tabs` the underlined style of the
    side panel. */
class SegmentedControl final : public juce::Component, public juce::SettableTooltipClient {
public:
    enum class Style { pill, tabs };

    SegmentedControl(juce::StringArray options, Style = Style::pill);
    /** -1 selects nothing (a value the options do not list). */
    void setSelected(int index, juce::NotificationType = juce::dontSendNotification);
    [[nodiscard]] int selected() const noexcept { return selected_; }
    [[nodiscard]] int idealWidth() const;
    void setOptionTooltip(int index, const juce::String&);
    /** Takes Tab focus; the left and right arrows, Home and End then pick. */
    void setKeyboardNavigable(bool);
    std::function<void(int)> onChange;

    void paint(juce::Graphics&) override;
    void mouseMove(const juce::MouseEvent&) override;
    void mouseExit(const juce::MouseEvent&) override;
    void mouseDown(const juce::MouseEvent&) override;
    bool keyPressed(const juce::KeyPress&) override;
    void focusGained(FocusChangeType) override { repaint(); }
    void focusLost(FocusChangeType) override { repaint(); }

private:
    [[nodiscard]] juce::Rectangle<float> optionBounds(int index) const;
    [[nodiscard]] int optionAt(juce::Point<float>) const;
    [[nodiscard]] float optionWidth(int index) const;
    juce::StringArray options_;
    juce::StringArray tooltips_;
    Style style_;
    int selected_ { 0 };
    int hovered_ { -1 };
};

/** Every `ActionButton` and `SegmentedControl` inside `root` takes Tab focus
    (a tool window's controls, audit UA-07). */
void enableKeyboardNavigation(juce::Component& root);

/** A labelled horizontal slider whose current value is printed in the header. */
class SliderRow final : public juce::Component, private juce::Slider::Listener {
public:
    explicit SliderRow(const juce::String& name);
    ~SliderRow() override;
    void setName(const juce::String&) override;
    std::function<juce::String(double)> format;
    juce::Slider slider;

    void paint(juce::Graphics&) override;
    void resized() override;
    static constexpr int preferredHeight = 44;

private:
    void sliderValueChanged(juce::Slider*) override { repaint(); }
    juce::String name_;
};

/** Vertical scrolling container. A wheel turned while `forwardWheel` is true
    goes to the parent instead of scrolling: the wheel modifiers (volume,
    gains, simulation rate) must work wherever the pointer is. */
class ScrollPane final : public juce::Viewport {
public:
    ScrollPane();
    std::function<bool()> forwardWheel;
    void mouseWheelMove(const juce::MouseEvent&, const juce::MouseWheelDetails&) override;
};

/** Configures a slider for the main window: no text box, no keyboard focus. */
void prepareSlider(juce::Slider&, double minimum, double maximum, double interval, double value);

} // namespace enginelab::ui
