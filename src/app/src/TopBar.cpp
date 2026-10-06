#include <enginelab/app/TopBar.hpp>

#include <enginelab/app/Theme.hpp>
#include <algorithm>

namespace enginelab::ui {
namespace {
[[nodiscard]] juce::Path svgIcon(const char* path) {
    return juce::Drawable::parseSVGPath(path);
}

[[nodiscard]] juce::Path moreIcon() {
    juce::Path dots;
    for (const auto x : { 5.0F, 12.0F, 19.0F }) dots.addEllipse(x - 1.2F, 10.8F, 2.4F, 2.4F);
    return dots;
}

struct StateStyle final { juce::Colour text; juce::Colour fill; };

[[nodiscard]] StateStyle styleFor(RunningState state) {
    switch (state) {
    case RunningState::idling:
    case RunningState::running: return { colours::ok, juce::Colour(0xff13201b) };
    case RunningState::cranking:
    case RunningState::unstable:
    case RunningState::knocking:
    case RunningState::overheating: return { colours::warn, colours::warnFill };
    case RunningState::damaged:
    case RunningState::destroyed: return { colours::critical, colours::criticalFill };
    case RunningState::stopped: break;
    }
    return { colours::muted, colours::panelRaised };
}
} // namespace

TopBar::EnginePicker::EnginePicker() : juce::Button("Engine") {
    setWantsKeyboardFocus(false);
    setMouseClickGrabsKeyboardFocus(false);
    setMouseCursor(juce::MouseCursor::PointingHandCursor);
    setTooltip(utf8("Choose an engine (F1 to F12 select the first twelve directly)"));
}

void TopBar::EnginePicker::setEngine(const juce::String& name, const juce::String& spec) {
    name_ = name;
    spec_ = spec;
    repaint();
}

void TopBar::EnginePicker::setModified(bool modified) {
    if (modified == modified_) return;
    modified_ = modified;
    setTooltip(modified ? utf8("Edited since it was loaded: ⋯ › Save engine keeps the edits, "
                               "Return reloads the engine as it was")
                        : utf8("Choose an engine (F1 to F12 select the first twelve directly)"));
    repaint();
}

void TopBar::EnginePicker::paintButton(juce::Graphics& g, bool highlighted, bool down) {
    const auto bounds = getLocalBounds().toFloat().reduced(0.5F);
    const auto alpha = isEnabled() ? 1.0F : 0.5F;
    g.setColour((highlighted || down ? colours::panelRaised : colours::panel).withMultipliedAlpha(alpha));
    g.fillRoundedRectangle(bounds, 8.0F);
    g.setColour((highlighted ? colours::lineHover : colours::lineStrong).withMultipliedAlpha(alpha));
    g.drawRoundedRectangle(bounds, 8.0F, 1.0F);
    auto content = bounds.reduced(12.0F, 5.0F);
    auto chevron = content.removeFromRight(14.0F);
    juce::Path arrow;
    arrow.startNewSubPath(chevron.getCentreX() - 4.0F, chevron.getCentreY() - 2.0F);
    arrow.lineTo(chevron.getCentreX(), chevron.getCentreY() + 2.0F);
    arrow.lineTo(chevron.getCentreX() + 4.0F, chevron.getCentreY() - 2.0F);
    g.setColour(colours::muted.withMultipliedAlpha(alpha));
    g.strokePath(arrow, juce::PathStrokeType(1.6F, juce::PathStrokeType::curved,
                                             juce::PathStrokeType::rounded));
    content.removeFromRight(8.0F);
    if (modified_) {
        const auto dot = content.removeFromRight(10.0F).withSizeKeepingCentre(7.0F, 7.0F);
        g.setColour(colours::warn.withMultipliedAlpha(alpha));
        g.fillEllipse(dot);
        content.removeFromRight(6.0F);
    }
    g.setColour(colours::text.withMultipliedAlpha(alpha));
    g.setFont(uiFont(16.5F, true));
    g.drawFittedText(name_, content.removeFromTop(content.getHeight() * 0.54F).toNearestInt(),
                     juce::Justification::bottomLeft, 1, 1.0F);
    g.setColour(colours::muted.withMultipliedAlpha(alpha));
    g.setFont(uiFont(14.0F));
    g.drawFittedText(spec_, content.toNearestInt(), juce::Justification::topLeft, 1, 1.0F);
}

TopBar::TopBar() {
    picker_.onClick = [this] { showEngineMenu(); };
    addAndMakeVisible(picker_);
    exhaustButton.setIcon(svgIcon("M3 12h4l3-7 4 14 3-7h4"));
    ecuButton.setIcon(svgIcon("M6 4h12a2 2 0 0 1 2 2v12a2 2 0 0 1-2 2H6a2 2 0 0 1-2-2V6a2 2 0 0 1 2-2z"
                              "M4 10h16M10 4v16"));
    audioButton.setIcon(svgIcon("M4 10v4M8 7v10M12 4v16M16 8v8M20 11v2"));
    moreButton.setIcon(moreIcon());
    exhaustButton.setTooltip(utf8("EXHAUST PRO: design the exhaust graph"));
    ecuButton.setTooltip(utf8("ECU tuner: fuel, ignition and boost tables"));
    audioButton.setTooltip(utf8("AUDIO HQ: mix, voicing and physical calibration"));
    moreButton.setTooltip(utf8("Import, export, JSON editor, dyno CSV, key bindings"));
    for (auto* button : { &exhaustButton, &ecuButton, &audioButton, &moreButton })
        addAndMakeVisible(*button);
}

void TopBar::setEngineChoices(std::vector<EngineChoice> choices) { choices_ = std::move(choices); }

void TopBar::setEngine(const EngineConfig& config, int presetIndex) {
    presetIndex_ = presetIndex;
    picker_.setEngine(juce::String::fromUTF8(config.name.c_str()), engineSpecLine(config));
}

void TopBar::setModified(bool modified) { picker_.setModified(modified); }

void TopBar::showEngineMenu() {
    juce::PopupMenu menu;
    auto savedHeader = false;
    for (int index = 0; index < static_cast<int>(choices_.size()); ++index) {
        const auto& choice = choices_[static_cast<std::size_t>(index)];
        if (choice.saved && !savedHeader) {
            menu.addSectionHeader(utf8("My engines"));
            savedHeader = true;
        }
        juce::PopupMenu::Item item(choice.edited ? choice.name + utf8("  • edited") : choice.name);
        item.itemID = index + 1;
        item.isTicked = index == presetIndex_;
        if (index < 12) item.shortcutKeyDescription = "F" + juce::String(index + 1);
        menu.addItem(item);
    }
    auto safe = juce::Component::SafePointer<TopBar>(this);
    menu.showMenuAsync(juce::PopupMenu::Options {}
                           .withTargetComponent(&picker_)
                           .withMinimumWidth(picker_.getWidth()),
                       [safe](int result) {
                           if (safe && result > 0 && safe->onEngineSelected)
                               safe->onEngineSelected(result - 1);
                       });
}

void TopBar::refresh(const DashboardModel& model) {
    picker_.setEnabled(!model.dynoRunning);
    const auto style = model.health.paused
        ? StateStyle { colours::warn, colours::warnFill } : styleFor(model.state.runningState);
    const auto text = model.health.paused ? juce::String("PAUSED")
                                          : juce::String(runningStateName(model.state.runningState));
    if (text == stateText_ && style.text == stateColour_) return;
    stateText_ = text;
    stateColour_ = style.text;
    stateFill_ = style.fill;
    repaint(pillArea_.getSmallestIntegerContainer().expanded(4));
}

void TopBar::paint(juce::Graphics& g) {
    g.fillAll(colours::chrome);
    g.setColour(colours::line);
    g.fillRect(getLocalBounds().removeFromBottom(1));

    const auto brandIcon = juce::Rectangle<float>(16.0F, static_cast<float>(getHeight()) * 0.5F - 9.0F,
                                                  18.0F, 18.0F);
    juce::ColourGradient gradient(colours::accent, brandIcon.getX(), brandIcon.getBottom(),
                                  colours::accentLight, brandIcon.getRight(), brandIcon.getY(), false);
    g.setGradientFill(gradient);
    g.fillRoundedRectangle(brandIcon, 5.0F);
    g.setColour(colours::text);
    g.setFont(uiFont(17.0F, true).withExtraKerningFactor(0.02F));
    g.drawText("EngineLab", juce::Rectangle<float>(42.0F, 0.0F, 110.0F, static_cast<float>(getHeight())),
               juce::Justification::centredLeft, false);

    if (stateText_.isEmpty()) return;
    g.setColour(stateFill_);
    g.fillRoundedRectangle(pillArea_, pillArea_.getHeight() * 0.5F);
    const auto dot = juce::Rectangle<float>(7.0F, 7.0F).withCentre({ pillArea_.getX() + 15.0F,
                                                                     pillArea_.getCentreY() });
    g.setColour(stateColour_.withAlpha(0.35F));
    g.fillEllipse(dot.expanded(3.0F));
    g.setColour(stateColour_);
    g.fillEllipse(dot);
    g.setFont(uiFont(13.5F, true).withExtraKerningFactor(0.04F));
    g.drawText(stateText_, pillArea_.withTrimmedLeft(25.0F).withTrimmedRight(10.0F),
               juce::Justification::centredLeft, false);
}

void TopBar::resized() {
    auto area = getLocalBounds().reduced(16, 0).withTrimmedBottom(1);
    area.removeFromLeft(140);
    const auto pickerWidth = std::clamp(getWidth() / 4, 300, 440);
    picker_.setBounds(area.removeFromLeft(pickerWidth).withSizeKeepingCentre(pickerWidth, 42));
    area.removeFromLeft(16);
    pillArea_ = area.removeFromLeft(130).withSizeKeepingCentre(130, 26).toFloat();

    auto tools = area;
    moreButton.setBounds(tools.removeFromRight(36).withSizeKeepingCentre(36, 34));
    for (auto* button : { &audioButton, &ecuButton, &exhaustButton }) {
        tools.removeFromRight(6);
        const auto width = button->idealWidth();
        button->setBounds(tools.removeFromRight(width).withSizeKeepingCentre(width, 34));
    }
}

} // namespace enginelab::ui
