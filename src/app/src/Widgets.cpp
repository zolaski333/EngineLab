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
    case ActionButton::Style::primary: return uiMediumFont(15.5F);
    case ActionButton::Style::tool: return uiMediumFont(15.0F);
    case ActionButton::Style::compact: return uiMediumFont(14.5F);
    }
    return uiMediumFont(15.0F);
}

[[nodiscard]] juce::Font segmentFont(SegmentedControl::Style style) {
    return style == SegmentedControl::Style::tabs ? uiFont(15.5F, true) : uiMediumFont(14.5F);
}

constexpr float lampSize = 7.0F;
constexpr float lampGap = 8.0F;
} // namespace

void drawLamp(juce::Graphics& g, juce::Rectangle<float> area, bool lit, juce::Colour colour) {
    const auto lamp = area.withSizeKeepingCentre(lampSize, lampSize);
    if (lit) {
        // A lit lamp glows a little past its edge; an unlit one is a dark tile.
        g.setColour(colour.withAlpha(0.22F));
        g.fillRoundedRectangle(lamp.expanded(2.0F), cornerRadius + 1.0F);
        g.setColour(colour);
        g.fillRoundedRectangle(lamp, 1.0F);
    } else {
        g.setColour(colours::unlit);
        g.fillRoundedRectangle(lamp, 1.0F);
    }
}

ActionButton::ActionButton(const juce::String& label, Style style)
    : juce::Button(label), style_(style) {
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
    // A button told its state once carries one: its lamp stays from then on.
    if (engaged_ == engaged && stateful_) return;
    engaged_ = engaged;
    stateful_ = true;
    repaint();
}

int ActionButton::idealWidth() const {
    auto width = textWidth(buttonFont(style_), getButtonText()) + 24.0F;
    if ((stateful_ || isToggleable()) && style_ != Style::primary) width += lampSize + lampGap;
    if (!icon_.isEmpty()) width += getButtonText().isEmpty() ? 15.0F : 22.0F;
    if (keyCap_.isNotEmpty()) width += keyCapWidth(keyCap_) + 10.0F;
    return static_cast<int>(std::ceil(width));
}

void ActionButton::paintButton(juce::Graphics& g, bool highlighted, bool down) {
    const auto bounds = getLocalBounds().toFloat().reduced(0.5F);
    const auto on = getToggleState() || engaged_;
    const auto radius = cornerRadius;
    const auto alpha = isEnabled() ? 1.0F : 0.42F;
    auto textColour = colours::text;
    auto onAccent = false;

    if (style_ == Style::primary && !on) {
        // The one invitation on a panel: inverted, light with dark text.
        auto fill = colours::text;
        if (down) fill = fill.darker(0.15F);
        else if (highlighted) fill = juce::Colours::white;
        g.setColour(fill.withMultipliedAlpha(alpha));
        g.fillRoundedRectangle(bounds, radius);
        textColour = colours::ink;
        onAccent = true;
    } else if (style_ == Style::primary) {
        // A running dyno: the same button now cancels, so it stops looking
        // like an invitation and reads as a warning.
        g.setColour(colours::warnFill.withMultipliedAlpha(alpha));
        g.fillRoundedRectangle(bounds, radius);
        g.setColour(colours::warn.withMultipliedAlpha(highlighted ? alpha : alpha * 0.75F));
        g.drawRoundedRectangle(bounds, radius, 1.0F);
        textColour = colours::warn;
    } else {
        auto fill = on ? colours::panelRaised : colours::panel;
        if (highlighted || down) fill = colours::selection;
        if (down) fill = fill.brighter(0.05F);
        g.setColour(fill.withMultipliedAlpha(alpha));
        g.fillRoundedRectangle(bounds, radius);
        g.setColour((highlighted ? colours::lineHover : (on ? colours::lineHover : colours::lineStrong))
            .withMultipliedAlpha(alpha));
        g.drawRoundedRectangle(bounds, radius, 1.0F);
        if (!on) textColour = colours::muted;
    }

    auto content = bounds.reduced(style_ == Style::compact ? 8.0F : 12.0F, 0.0F);
    if ((stateful_ || isToggleable()) && style_ != Style::primary) {
        juce::Graphics::ScopedSaveState state(g);
        g.setOpacity(alpha);
        drawLamp(g, content.removeFromLeft(lampSize), on, colours::ok);
        content.removeFromLeft(lampGap);
    }
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
    if (hasKeyboardFocus(false)) {
        g.setColour(onAccent ? colours::ink : colours::text);
        g.drawRoundedRectangle(bounds.reduced(2.0F), radius, 1.0F);
    }
}

void ActionButton::setKeyboardNavigable(bool navigable) {
    setWantsKeyboardFocus(navigable);
}

SegmentedControl::SegmentedControl(juce::StringArray options, Style style)
    : options_(std::move(options)), style_(style) {
    for (int i = 0; i < options_.size(); ++i) tooltips_.add({});
    setWantsKeyboardFocus(false);
    setMouseClickGrabsKeyboardFocus(false);
}

void SegmentedControl::setSelected(int index, juce::NotificationType notification) {
    index = std::clamp(index, -1, options_.size() - 1);
    if (index == selected_) return;
    selected_ = index;
    repaint();
    if (notification != juce::dontSendNotification && onChange) onChange(selected_);
}

void SegmentedControl::setKeyboardNavigable(bool navigable) {
    setWantsKeyboardFocus(navigable);
}

bool SegmentedControl::keyPressed(const juce::KeyPress& key) {
    if (!getWantsKeyboardFocus() || options_.isEmpty()) return false;
    const auto last = options_.size() - 1;
    auto target = selected_;
    if (key.isKeyCode(juce::KeyPress::leftKey)) target = selected_ <= 0 ? 0 : selected_ - 1;
    else if (key.isKeyCode(juce::KeyPress::rightKey)) target = std::min(selected_ + 1, last);
    else if (key.isKeyCode(juce::KeyPress::homeKey)) target = 0;
    else if (key.isKeyCode(juce::KeyPress::endKey)) target = last;
    else return false;
    setSelected(target, juce::sendNotificationSync);
    return true;
}

void enableKeyboardNavigation(juce::Component& root) {
    for (auto* child : root.getChildren()) {
        if (auto* button = dynamic_cast<ActionButton*>(child)) button->setKeyboardNavigable(true);
        else if (auto* segments = dynamic_cast<SegmentedControl*>(child)) segments->setKeyboardNavigable(true);
        enableKeyboardNavigation(*child);
    }
}

void SegmentedControl::setOptionTooltip(int index, const juce::String& tooltip) {
    if (index >= 0 && index < tooltips_.size()) tooltips_.set(index, tooltip);
}

float SegmentedControl::optionWidth(int index) const {
    const auto natural = style_ == Style::tabs ? 24.0F : 20.0F;
    // Laid out narrower than its ideal (five tabs in the side column), the
    // padding around each label gives way first, down to 8 px.
    auto padding = natural;
    if (const auto width = static_cast<float>(getWidth()); width > 0.0F && options_.size() > 0) {
        auto text = style_ == Style::tabs ? 0.0F : 6.0F;
        for (int option = 0; option < options_.size(); ++option)
            text += textWidth(segmentFont(style_), options_[option]) + 2.0F;
        const auto fitting = (width - text) / static_cast<float>(options_.size());
        padding = std::clamp(fitting, 8.0F, natural);
    }
    return textWidth(segmentFont(style_), options_[index]) + padding;
}

int SegmentedControl::idealWidth() const {
    // The natural width, whatever width it was last given.
    auto width = style_ == Style::tabs ? 0.0F : 6.0F;
    for (int index = 0; index < options_.size(); ++index)
        width += textWidth(segmentFont(style_), options_[index]) + (style_ == Style::tabs ? 24.0F : 20.0F) + 2.0F;
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
        // Over the 3-D view too: nearly opaque, so the labels stay readable.
        const auto bounds = getLocalBounds().toFloat().reduced(0.5F);
        g.setColour(colours::well.withAlpha(0.92F));
        g.fillRoundedRectangle(bounds, cornerRadius);
        g.setColour(colours::lineStrong);
        g.drawRoundedRectangle(bounds, cornerRadius, 1.0F);
    }
    g.setFont(segmentFont(style_));
    for (int index = 0; index < options_.size(); ++index) {
        const auto option = optionBounds(index);
        const auto active = index == selected_;
        auto label = active ? colours::text : (index == hovered_ ? colours::text : colours::muted);
        if (style_ == Style::pill) {
            if (active) {
                g.setColour(colours::text);
                g.fillRoundedRectangle(option, 1.0F);
                label = colours::ink;
            } else if (index == hovered_) {
                g.setColour(colours::selection);
                g.fillRoundedRectangle(option, 1.0F);
            }
        } else if (active) {
            g.setColour(colours::text);
            g.fillRect(option.withTop(option.getBottom() - 2.0F));
        }
        g.setColour(label);
        g.drawText(options_[index], option, juce::Justification::centred, false);
    }
    if (hasKeyboardFocus(false)) {
        g.setColour(colours::text);
        g.drawRoundedRectangle(getLocalBounds().toFloat().reduced(1.0F), cornerRadius, 1.0F);
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
    g.setFont(uiFont(15.0F));
    g.drawText(name_, header, juce::Justification::centredLeft, true);
    g.setColour(isEnabled() ? colours::text : colours::faint);
    g.setFont(monoFont(13.5F));
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
