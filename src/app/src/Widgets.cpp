#include <enginelab/app/Widgets.hpp>

#include <enginelab/app/Theme.hpp>
#include <algorithm>
#include <utility>

namespace enginelab::ui {
namespace {
[[nodiscard]] float textWidth(const juce::Font& font, const juce::String& text) {
    return juce::GlyphArrangement::getStringWidth(font, text);
}

[[nodiscard]] juce::Font buttonFont(ActionButton::Style style) {
    switch (style) {
    case ActionButton::Style::standard:
    case ActionButton::Style::primary: return uiFont(15.5F, true);
    case ActionButton::Style::tool: return uiFont(15.0F);
    case ActionButton::Style::compact: return uiFont(14.0F);
    }
    return uiFont(15.0F);
}

[[nodiscard]] juce::Font segmentFont(SegmentedControl::Style style) {
    return style == SegmentedControl::Style::tabs ? uiFont(15.0F, true) : uiFont(14.0F);
}
} // namespace

ActionButton::ActionButton(const juce::String& text, Style style)
    : juce::Button(text), style_(style) {
    setWantsKeyboardFocus(false);
    setMouseClickGrabsKeyboardFocus(false);
    setMouseCursor(juce::MouseCursor::PointingHandCursor);
}

void ActionButton::setStyle(Style style) {
    if (style_ == style) return;
    style_ = style;
    repaint();
}

void ActionButton::setKeyCap(const juce::String& keyCap) {
    if (keyCap_ == keyCap) return;
    keyCap_ = keyCap;
    repaint();
}

void ActionButton::setIcon(juce::Path icon) {
    icon_ = std::move(icon);
    repaint();
}

void ActionButton::setEngaged(bool engaged) {
    if (engaged_ == engaged) return;
    engaged_ = engaged;
    repaint();
}

int ActionButton::idealWidth() const {
    auto width = textWidth(buttonFont(style_), getButtonText()) + 24.0F;
    if (!icon_.isEmpty()) width += getButtonText().isEmpty() ? 15.0F : 22.0F;
    if (keyCap_.isNotEmpty()) width += keyCapWidth(keyCap_) + 10.0F;
    return static_cast<int>(std::ceil(width));
}

void ActionButton::paintButton(juce::Graphics& g, bool highlighted, bool down) {
    const auto bounds = getLocalBounds().toFloat().reduced(0.5F);
    const auto on = getToggleState() || engaged_;
    const auto radius = style_ == Style::compact ? 7.0F : (style_ == Style::tool ? 8.0F : 9.0F);
    const auto alpha = isEnabled() ? 1.0F : 0.42F;
    auto textColour = colours::text;
    auto onAccent = false;

    if (style_ == Style::primary && !on) {
        auto top = juce::Colour(0xfff07a49);
        auto bottom = colours::accentDeep;
        if (down) { top = top.darker(0.12F); bottom = bottom.darker(0.12F); }
        else if (highlighted) { top = top.brighter(0.06F); bottom = bottom.brighter(0.06F); }
        g.setGradientFill(juce::ColourGradient(top.withMultipliedAlpha(alpha), 0.0F, bounds.getY(),
            bottom.withMultipliedAlpha(alpha), 0.0F, bounds.getBottom(), false));
        g.fillRoundedRectangle(bounds, radius);
        g.setColour(juce::Colour(0xfff08a5e).withMultipliedAlpha(alpha));
        g.drawRoundedRectangle(bounds, radius, 1.0F);
        textColour = juce::Colours::white;
        onAccent = true;
    } else if (style_ == Style::primary) {
        // A running dyno: the same button now cancels, so it stops looking
        // like an invitation.
        g.setColour(juce::Colour(0xff2a1a12).withMultipliedAlpha(alpha));
        g.fillRoundedRectangle(bounds, radius);
        g.setColour(colours::accent.withMultipliedAlpha(highlighted ? alpha : alpha * 0.75F));
        g.drawRoundedRectangle(bounds, radius, 1.0F);
        textColour = colours::accentLight;
    } else if (on) {
        g.setColour(colours::okFill.withMultipliedAlpha(alpha));
        g.fillRoundedRectangle(bounds, radius);
        g.setColour(colours::okBorder.withMultipliedAlpha(alpha));
        g.drawRoundedRectangle(bounds, radius, 1.0F);
        textColour = juce::Colour(0xffbfe8d6);
    } else {
        auto fill = highlighted || down ? colours::panelRaised : colours::panel;
        if (down) fill = fill.brighter(0.05F);
        g.setColour(fill.withMultipliedAlpha(alpha));
        g.fillRoundedRectangle(bounds, radius);
        g.setColour((highlighted ? colours::lineHover : colours::lineStrong).withMultipliedAlpha(alpha));
        g.drawRoundedRectangle(bounds, radius, 1.0F);
    }

    auto content = bounds.reduced(style_ == Style::compact ? 8.0F : 12.0F, 0.0F);
    if (keyCap_.isNotEmpty()) {
        const auto capWidth = keyCapWidth(keyCap_);
        juce::Graphics::ScopedSaveState state(g);
        g.setOpacity(alpha);
        drawKeyCap(g, content.removeFromRight(capWidth), keyCap_, onAccent);
        content.removeFromRight(6.0F);
    }
    if (!icon_.isEmpty()) {
        const auto iconBox = content.removeFromLeft(15.0F).withSizeKeepingCentre(15.0F, 15.0F);
        // Icons are authored in a 24 x 24 box, like the SVG they come from.
        auto icon = icon_;
        icon.applyTransform(juce::RectanglePlacement(juce::RectanglePlacement::centred)
            .getTransformToFit({ 0.0F, 0.0F, 24.0F, 24.0F }, iconBox));
        g.setColour(colours::muted.withMultipliedAlpha(alpha));
        g.strokePath(icon, juce::PathStrokeType(1.6F, juce::PathStrokeType::curved,
                                                juce::PathStrokeType::rounded));
        content.removeFromLeft(7.0F);
    }
    g.setColour(textColour.withMultipliedAlpha(alpha));
    g.setFont(buttonFont(style_));
    const auto centred = keyCap_.isEmpty() && icon_.isEmpty() && style_ == Style::compact;
    g.drawFittedText(getButtonText(), content.toNearestInt(),
                     centred ? juce::Justification::centred : juce::Justification::centredLeft, 1);
}

SegmentedControl::SegmentedControl(juce::StringArray options, Style style)
    : options_(std::move(options)), style_(style) {
    for (int i = 0; i < options_.size(); ++i) tooltips_.add({});
    setWantsKeyboardFocus(false);
    setMouseClickGrabsKeyboardFocus(false);
}

void SegmentedControl::setSelected(int index, juce::NotificationType notification) {
    index = std::clamp(index, 0, std::max(0, options_.size() - 1));
    if (index == selected_) return;
    selected_ = index;
    repaint();
    if (notification != juce::dontSendNotification && onChange) onChange(selected_);
}

void SegmentedControl::setOptionTooltip(int index, const juce::String& tooltip) {
    if (index >= 0 && index < tooltips_.size()) tooltips_.set(index, tooltip);
}

float SegmentedControl::optionWidth(int index) const {
    return textWidth(segmentFont(style_), options_[index]) + (style_ == Style::tabs ? 24.0F : 20.0F);
}

int SegmentedControl::idealWidth() const {
    auto width = style_ == Style::tabs ? 0.0F : 6.0F;
    for (int index = 0; index < options_.size(); ++index) width += optionWidth(index) + 2.0F;
    return static_cast<int>(std::ceil(width));
}

juce::Rectangle<float> SegmentedControl::optionBounds(int index) const {
    auto area = getLocalBounds().toFloat();
    if (style_ == Style::pill) area = area.reduced(3.0F);
    auto x = area.getX();
    for (int previous = 0; previous < index; ++previous) x += optionWidth(previous) + 2.0F;
    return { x, area.getY(), optionWidth(index), area.getHeight() };
}

int SegmentedControl::optionAt(juce::Point<float> position) const {
    for (int index = 0; index < options_.size(); ++index)
        if (optionBounds(index).contains(position)) return index;
    return -1;
}

void SegmentedControl::paint(juce::Graphics& g) {
    if (style_ == Style::pill) {
        const auto bounds = getLocalBounds().toFloat().reduced(0.5F);
        g.setColour(juce::Colour(0xd10d1211));
        g.fillRoundedRectangle(bounds, 9.0F);
        g.setColour(colours::lineStrong);
        g.drawRoundedRectangle(bounds, 9.0F, 1.0F);
    }
    g.setFont(segmentFont(style_));
    for (int index = 0; index < options_.size(); ++index) {
        const auto option = optionBounds(index);
        const auto active = index == selected_;
        if (style_ == Style::pill) {
            if (active || index == hovered_) {
                g.setColour(active ? colours::selection : colours::panelRaised);
                g.fillRoundedRectangle(option, 6.0F);
            }
        } else if (active) {
            g.setColour(colours::accent);
            g.fillRect(option.withTop(option.getBottom() - 2.0F));
        }
        g.setColour(active ? colours::text : (index == hovered_ ? colours::text.withAlpha(0.8F)
                                                                : colours::muted));
        g.drawText(options_[index], option, juce::Justification::centred, false);
    }
}

void SegmentedControl::mouseMove(const juce::MouseEvent& event) {
    const auto hovered = optionAt(event.position);
    if (hovered == hovered_) return;
    hovered_ = hovered;
    setTooltip(hovered >= 0 ? tooltips_[hovered] : juce::String {});
    setMouseCursor(hovered >= 0 ? juce::MouseCursor::PointingHandCursor : juce::MouseCursor::NormalCursor);
    repaint();
}

void SegmentedControl::mouseExit(const juce::MouseEvent&) {
    hovered_ = -1;
    repaint();
}

void SegmentedControl::mouseDown(const juce::MouseEvent& event) {
    const auto index = optionAt(event.position);
    if (index >= 0) setSelected(index, juce::sendNotificationSync);
}

SliderRow::SliderRow(const juce::String& name) : name_(name) {
    slider.addListener(this);
    addAndMakeVisible(slider);
}

SliderRow::~SliderRow() { slider.removeListener(this); }

void SliderRow::setName(const juce::String& name) {
    juce::Component::setName(name);
    name_ = name;
    repaint();
}

void SliderRow::paint(juce::Graphics& g) {
    auto header = getLocalBounds().toFloat().removeFromTop(18.0F);
    g.setColour(isEnabled() ? colours::muted : colours::faint);
    g.setFont(uiFont(14.0F));
    g.drawText(name_, header, juce::Justification::centredLeft, true);
    g.setColour(isEnabled() ? colours::text : colours::faint);
    g.setFont(monoFont(14.0F));
    const auto value = format ? format(slider.getValue()) : juce::String(slider.getValue(), 1);
    g.drawText(value, header, juce::Justification::centredRight, false);
}

void SliderRow::resized() {
    slider.setBounds(getLocalBounds().withTrimmedTop(20).withTrimmedLeft(-2).withTrimmedRight(-2));
}

ScrollPane::ScrollPane() {
    setScrollBarsShown(true, false);
    setScrollBarThickness(9);
    setWantsKeyboardFocus(false);
}

void ScrollPane::mouseWheelMove(const juce::MouseEvent& event, const juce::MouseWheelDetails& wheel) {
    if (forwardWheel && forwardWheel()) {
        juce::Component::mouseWheelMove(event, wheel);
        return;
    }
    juce::Viewport::mouseWheelMove(event, wheel);
}

void prepareSlider(juce::Slider& slider, double minimum, double maximum, double interval, double value) {
    slider.setSliderStyle(juce::Slider::LinearHorizontal);
    slider.setTextBoxStyle(juce::Slider::NoTextBox, true, 0, 0);
    slider.setRange(minimum, maximum, interval);
    slider.setValue(value, juce::dontSendNotification);
    slider.setWantsKeyboardFocus(false);
    slider.setMouseClickGrabsKeyboardFocus(false);
    // A wheel over a slider would otherwise move it instead of reaching the
    // wheel modifiers handled by the main component.
    slider.setScrollWheelEnabled(false);
}

} // namespace enginelab::ui
