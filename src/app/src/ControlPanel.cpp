#include <enginelab/app/ControlPanel.hpp>

#include <enginelab/app/Theme.hpp>
#include <array>
#include <cmath>
#include <utility>

namespace enginelab::ui {
namespace {
[[nodiscard]] juce::String signedValue(double value, int decimals, const juce::String& suffix) {
    if (std::abs(value) < 0.5 * std::pow(10.0, -decimals))
        return utf8("\xc2\xb1") + juce::String(0.0, decimals) + suffix;
    return (value > 0.0 ? "+" : utf8("\xe2\x88\x92")) + juce::String(std::abs(value), decimals) + suffix;
}
} // namespace

juce::String ControlPanel::percentLabel(int percent) { return juce::String(percent) + " %"; }

ControlPanel::ControlPanel() {
    ignition.setClickingTogglesState(true);
    ignition.setTooltip(utf8("Ignition and injection on or off"));
    starter.setTooltip(utf8("Hold to crank the engine"));
    dyno.setTooltip(utf8("Automatic dyno run (H holds the engine speed, 6 arms the continuous ramp)"));
    for (auto* button : { &ignition, &starter, &dyno }) addAndMakeVisible(*button);

    throttle.setSliderStyle(juce::Slider::LinearBarVertical);
    throttle.setTextBoxStyle(juce::Slider::NoTextBox, true, 0, 0);
    throttle.setRange(0.0, 100.0, 1.0);
    throttle.setWantsKeyboardFocus(false);
    throttle.setMouseClickGrabsKeyboardFocus(false);
    throttle.setScrollWheelEnabled(false);
    throttle.setSliderSnapsToMousePosition(true);
    throttle.setMouseCursor(juce::MouseCursor::UpDownResizeCursor);
    throttle.setTooltip(utf8("Throttle. Drag, or hold Q / W / E / R for a momentary opening"));
    throttle.addListener(this);
    addAndMakeVisible(throttle);

    const std::array<std::pair<ActionButton*, double>, 4> presets {{
        { &throttleIdle, 0.01 }, { &throttleLow, 0.10 }, { &throttleMid, 0.20 }, { &throttleFull, 1.0 } }};
    for (const auto& [button, fraction] : presets) {
        button->onClick = [this, value = fraction] { if (onThrottlePreset) onThrottlePreset(value); };
        button->setTooltip(utf8("Click to set the throttle; hold the key for a momentary opening"));
        addAndMakeVisible(*button);
    }

    prepareSlider(afrTrim.slider, -3.0, 3.0, 0.1, 0.0);
    afrTrim.slider.setDoubleClickReturnValue(true, 0.0);
    afrTrim.format = [](double value) { return signedValue(value, 1, {}); };
    prepareSlider(sparkTrim.slider, -15.0, 15.0, 0.1, 0.0);
    sparkTrim.slider.setDoubleClickReturnValue(true, 0.0);
    sparkTrim.format = [](double value) { return signedValue(value, 1, utf8("\xc2\xb0")); };
    addChildComponent(afrTrim);
    addChildComponent(sparkTrim);
}

ControlPanel::~ControlPanel() { throttle.removeListener(this); }

void ControlPanel::setKeyCaps(const ActionMap& actions) {
    const auto cap = [&actions](AppAction action) { return keyCapText(actions.shortcut(action)); };
    ignition.setKeyCap(cap(AppAction::ignition));
    starter.setKeyCap(cap(AppAction::starter));
    dyno.setKeyCap(cap(AppAction::dyno));
    throttleIdle.setKeyCap(cap(AppAction::throttleIdle));
    throttleLow.setKeyCap(cap(AppAction::throttleQuarter));
    throttleMid.setKeyCap(cap(AppAction::throttleHalf));
    throttleFull.setKeyCap(cap(AppAction::throttleFull));
}

void ControlPanel::setDiesel(bool diesel) {
    afrTrim.setName(diesel ? utf8("Smoke limit trim (+ = less fuel)") : utf8("AFR trim"));
    afrTrim.slider.setTooltip(diesel ? utf8("Offset of the smoke-limit AFR table")
                                     : utf8("Offset added to the target AFR table"));
}

void ControlPanel::refresh(const DashboardModel& model) {
    const auto running = model.dynoRunning;
    dyno.setButtonText(running
        ? (model.state.dynoPreparing ? utf8("Cancel prep")
            : (model.state.dynoMode == DynoMode::hold ? utf8("End hold") : utf8("Cancel dyno")))
        : utf8("Start dyno"));
    dyno.setToggleState(running, juce::dontSendNotification);
    for (juce::Component* control : { static_cast<juce::Component*>(&ignition),
                                       static_cast<juce::Component*>(&starter),
                                       static_cast<juce::Component*>(&throttle),
                                       static_cast<juce::Component*>(&throttleIdle),
                                       static_cast<juce::Component*>(&throttleLow),
                                       static_cast<juce::Component*>(&throttleMid),
                                       static_cast<juce::Component*>(&throttleFull),
                                       static_cast<juce::Component*>(&afrTrim),
                                       static_cast<juce::Component*>(&sparkTrim) })
        control->setEnabled(!running);
}

void ControlPanel::sliderValueChanged(juce::Slider*) {
    repaint(throttleValue_.getSmallestIntegerContainer());
}

void ControlPanel::paint(juce::Graphics& g) {
    g.fillAll(colours::background);
    g.setColour(colours::line);
    g.fillRect(getLocalBounds().removeFromRight(1));
    drawSectionLabel(g, engineLabel_, "Engine");
    drawSectionLabel(g, throttleLabel_, "Throttle");
    g.setColour(throttle.isEnabled() ? colours::text : colours::muted);
    g.setFont(monoFont(32.0F));
    g.drawText(fixed(throttle.getValue(), 0) + " %", throttleValue_,
               juce::Justification::topLeft, false);
    drawSectionLabel(g, trimsHeader_, juce::String("Calibration trims  ") + (trimsOpen_ ? utf8("\xe2\x80\x93") : "+"));
}

void ControlPanel::mouseUp(const juce::MouseEvent& event) {
    if (!trimsHeader_.contains(event.position)) return;
    trimsOpen_ = !trimsOpen_;
    afrTrim.setVisible(trimsOpen_);
    sparkTrim.setVisible(trimsOpen_);
    resized();
    repaint();
}

void ControlPanel::resized() {
    auto area = getLocalBounds().reduced(16);
    engineLabel_ = area.removeFromTop(22).toFloat();
    for (auto* button : { &ignition, &starter, &dyno }) {
        button->setBounds(area.removeFromTop(42));
        area.removeFromTop(8);
    }
    area.removeFromTop(14);
    throttleLabel_ = area.removeFromTop(22).toFloat();
    auto throttleRow = area.removeFromTop(170);
    throttle.setBounds(throttleRow.removeFromLeft(44));
    throttleRow.removeFromLeft(12);
    throttleValue_ = throttleRow.removeFromTop(40).toFloat();
    auto presets = throttleRow.removeFromBottom(66);
    auto topPresets = presets.removeFromTop(30);
    auto bottomPresets = presets.removeFromBottom(30);
    const auto half = (topPresets.getWidth() - 6) / 2;
    throttleIdle.setBounds(topPresets.removeFromLeft(half));
    throttleLow.setBounds(topPresets.removeFromRight(half));
    throttleMid.setBounds(bottomPresets.removeFromLeft(half));
    throttleFull.setBounds(bottomPresets.removeFromRight(half));
    area.removeFromTop(20);
    trimsHeader_ = area.removeFromTop(22).toFloat();
    area.removeFromTop(4);
    if (trimsOpen_) {
        afrTrim.setBounds(area.removeFromTop(SliderRow::preferredHeight));
        area.removeFromTop(10);
        sparkTrim.setBounds(area.removeFromTop(SliderRow::preferredHeight));
    }
}

} // namespace enginelab::ui
