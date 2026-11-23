#include <enginelab/app/TopBar.hpp>

#include <enginelab/app/Theme.hpp>
#include <algorithm>
#include <array>

namespace enginelab::ui {
namespace {
enum LampIndex : std::size_t {
    ignLamp, crankLamp, runLamp, dynoLamp, boostLamp, limitLamp, knockLamp, faultLamp, logLamp, versionLamp,
    pauseLamp, lampCount
};

// The order lamps give way in when the bar is short: the least telling first.
constexpr std::array<std::size_t, lampCount> lampDropOrder {
    crankLamp, pauseLamp, boostLamp, versionLamp, limitLamp, knockLamp, logLamp, dynoLamp, faultLamp, runLamp,
    ignLamp };

constexpr std::array<TopMenu, 5> menuOrder {
    TopMenu::design, TopMenu::tune, TopMenu::test, TopMenu::listen, TopMenu::more };

[[nodiscard]] juce::Font lampFont() { return uiFont(11.5F, true); }
[[nodiscard]] juce::Font menuFont() { return uiMediumFont(15.0F); }

[[nodiscard]] float lampWidth(const char* text) {
    return std::max(30.0F, juce::GlyphArrangement::getStringWidth(lampFont(), text) + 14.0F);
}

[[nodiscard]] bool hasLimit(const EngineState& state, ActiveLimit limit) {
    return (state.activeLimits & static_cast<std::uint32_t>(limit)) != 0U;
}

/** The brand mark: a crank throw, the pin on its circle. */
void drawMark(juce::Graphics& g, juce::Rectangle<float> box) {
    const auto unit = box.getWidth() / 24.0F;
    const auto at = [&box, unit](float x, float y) { return juce::Point<float>(box.getX() + x * unit, box.getY() + y * unit); };
    juce::Path throwPath;
    throwPath.addEllipse(juce::Rectangle<float>(14.0F * unit, 14.0F * unit).withCentre(at(12.0F, 15.0F)));
    throwPath.startNewSubPath(at(15.5F, 12.0F));
    throwPath.lineTo(at(12.0F, 3.0F));
    g.setColour(colours::text);
    g.strokePath(throwPath, juce::PathStrokeType(1.5F * unit, juce::PathStrokeType::curved,
                                                 juce::PathStrokeType::rounded));
    g.fillEllipse(juce::Rectangle<float>(3.2F * unit, 3.2F * unit).withCentre(at(15.5F, 12.0F)));
}

void drawCaret(juce::Graphics& g, juce::Point<float> centre, juce::Colour colour) {
    juce::Path caret;
    caret.addTriangle(centre.x - 3.5F, centre.y - 1.75F, centre.x + 3.5F, centre.y - 1.75F, centre.x, centre.y + 1.75F);
    g.setColour(colour);
    g.fillPath(caret);
}
} // namespace

TopBar::EnginePlate::EnginePlate() : juce::Button("Engine") {
    setWantsKeyboardFocus(false);
    setMouseClickGrabsKeyboardFocus(false);
    setMouseCursor(juce::MouseCursor::PointingHandCursor);
    setTooltip(utf8("Choose an engine (F1 to F12 select the first twelve directly)"));
}

void TopBar::EnginePlate::setEngine(const juce::String& name, const juce::String& spec) {
    name_ = name;
    spec_ = spec;
    repaint();
}

void TopBar::EnginePlate::setModified(bool modified) {
    if (modified == modified_) return;
    modified_ = modified;
    setTooltip(modified ? utf8("Edited since it was loaded: Design › Save engine keeps the edits, "
                               "Return reloads the engine as it was")
                        : utf8("Choose an engine (F1 to F12 select the first twelve directly)"));
    repaint();
}

void TopBar::EnginePlate::paintButton(juce::Graphics& g, bool highlighted, bool down) {
    const auto bounds = getLocalBounds().toFloat();
    const auto alpha = isEnabled() ? 1.0F : 0.5F;
    if (highlighted || down) {
        g.setColour(colours::panelRaised);
        g.fillRect(bounds.withTrimmedRight(1.0F));
    }
    g.setColour(colours::line);
    g.fillRect(bounds.withLeft(bounds.getRight() - 1.0F));
    auto content = bounds.reduced(12.0F, 6.0F);
    const auto chevron = content.removeFromRight(12.0F);
    drawCaret(g, chevron.getCentre(), colours::label.withMultipliedAlpha(alpha));
    content.removeFromRight(8.0F);
    if (modified_) {
        // Edited and unsaved: an amber lamp, as on the desk.
        drawLamp(g, content.removeFromRight(9.0F), true, colours::warn);
        content.removeFromRight(8.0F);
    }
    g.setColour(colours::text.withMultipliedAlpha(alpha));
    g.setFont(uiFont(16.0F, true));
    g.drawFittedText(name_, content.removeFromTop(content.getHeight() * 0.54F).toNearestInt(),
                     juce::Justification::bottomLeft, 1, 1.0F);
    g.setColour(colours::label.withMultipliedAlpha(alpha));
    g.setFont(uiFont(13.5F));
    g.drawFittedText(spec_, content.toNearestInt(), juce::Justification::topLeft, 1, 1.0F);
}

TopBar::MenuButton::MenuButton(TopBar& owner, TopMenu menu)
    : juce::Button(menuTitle(menu)), owner_(owner), menu_(menu) {
    setWantsKeyboardFocus(false);
    setMouseClickGrabsKeyboardFocus(false);
    setTriggeredOnMouseDown(true);
    setMouseCursor(juce::MouseCursor::PointingHandCursor);
    if (menu == TopMenu::more) setTooltip(utf8("Key bindings, units, full screen (Ctrl+M opens the menus)"));
}

void TopBar::MenuButton::setOpen(bool open) {
    if (open == open_) return;
    open_ = open;
    repaint();
}

int TopBar::MenuButton::idealWidth() const {
    if (menu_ == TopMenu::more) return 34;
    return static_cast<int>(std::ceil(juce::GlyphArrangement::getStringWidth(menuFont(), getButtonText()) + 34.0F));
}

bool TopBar::MenuButton::keyPressed(const juce::KeyPress& key) {
    if (key.isKeyCode(juce::KeyPress::leftKey)) { owner_.stepMenu(menu_, -1); return true; }
    if (key.isKeyCode(juce::KeyPress::rightKey)) { owner_.stepMenu(menu_, 1); return true; }
    return false;
}

void TopBar::MenuButton::paintButton(juce::Graphics& g, bool highlighted, bool down) {
    const auto bounds = getLocalBounds().toFloat().reduced(0.0F, 8.0F);
    const auto lit = open_ || down;
    if (lit || highlighted) {
        g.setColour(lit ? colours::selection : colours::panelRaised);
        g.fillRoundedRectangle(bounds, cornerRadius);
    }
    const auto textColour = lit || highlighted ? colours::text : colours::muted;
    if (menu_ == TopMenu::more) {
        g.setColour(textColour);
        for (const auto dx : { -6.0F, 0.0F, 6.0F })
            g.fillEllipse(juce::Rectangle<float>(2.6F, 2.6F).withCentre(bounds.getCentre().translated(dx, 0.0F)));
        return;
    }
    auto content = bounds.reduced(10.0F, 0.0F);
    const auto caret = content.removeFromRight(8.0F);
    drawCaret(g, { caret.getCentreX(), caret.getCentreY() + 1.0F }, colours::label);
    g.setColour(textColour);
    g.setFont(menuFont());
    g.drawText(getButtonText(), content, juce::Justification::centredLeft, false);
}

TopBar::TopBar() {
    plate_.onClick = [this] { showEngineMenu(); };
    addAndMakeVisible(plate_);
    for (std::size_t index = 0; index < menuOrder.size(); ++index) {
        const auto menu = menuOrder[index];
        menus_[index] = std::make_unique<MenuButton>(*this, menu);
        menus_[index]->onClick = [this, menu] { openMenu(menu); };
        addAndMakeVisible(*menus_[index]);
    }
    lamps_ = {
        { "IGN", utf8("Ignition on (A)"), colours::text },
        { "CRANK", utf8("The starter turns the engine (S, held)"), colours::warn },
        { "RUN", utf8("The engine runs on its own"), colours::ok },
        { "DYNO", utf8("A dyno run is under way (D cancels it)"), colours::accent },
        { "BOOST", utf8("The manifold is above ambient pressure"), colours::accent },
        { "LIMIT", utf8("A limiter is cutting: rev limiter, launch control or overboost"), colours::critical },
        { "KNOCK", utf8("Knock: the ECU pulls spark"), colours::critical },
        { "FAULT", utf8("No fault detected"), colours::critical },
        { "LOG", utf8("Datalogger (key 9)"), colours::critical },
        { "B", utf8("A/B: version B is running (key 7 switches back to A)"), colours::text },
        { "PAUSE", utf8("The simulation is paused"), colours::warn },
    };
}

void TopBar::setEngineChoices(std::vector<EngineChoice> choices) { choices_ = std::move(choices); }

void TopBar::setUnloadedEngines(std::vector<UnloadedEngine> unloaded) { unloaded_ = std::move(unloaded); }

void TopBar::setEngine(const EngineConfig& config, int presetIndex) {
    presetIndex_ = presetIndex;
    plate_.setEngine(juce::String::fromUTF8(config.name.c_str()), engineSpecLine(config));
}

void TopBar::setModified(bool modified) { plate_.setModified(modified); }

TopBar::MenuButton& TopBar::button(TopMenu menu) {
    const auto position = std::find(menuOrder.begin(), menuOrder.end(), menu) - menuOrder.begin();
    return *menus_[static_cast<std::size_t>(position)];
}

juce::Component& TopBar::menuAnchor(TopMenu menu) { return button(menu); }

void TopBar::openMenu(TopMenu menu) {
    for (auto& entry : menus_) entry->setOpen(false);
    button(menu).setOpen(true);
    if (onMenu) onMenu(menu, button(menu));
}

void TopBar::menuClosed() {
    for (auto& entry : menus_) entry->setOpen(false);
}

void TopBar::stepMenu(TopMenu from, int direction) {
    const auto position = static_cast<int>(std::find(menuOrder.begin(), menuOrder.end(), from) - menuOrder.begin());
    const auto count = static_cast<int>(menuOrder.size());
    const auto next = menuOrder[static_cast<std::size_t>((position + direction + count) % count)];
    // The open menu is still inside its key handler: close it and open the
    // next one once that has returned.
    auto safe = juce::Component::SafePointer<TopBar>(this);
    juce::MessageManager::callAsync([safe, next] {
        juce::PopupMenu::dismissAllActiveMenus();
        if (safe) safe->openMenu(next);
    });
}

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
    // Ids above the presets': the files that did not load, picked to see why.
    const auto unloadedBase = static_cast<int>(choices_.size()) + 1;
    if (!unloaded_.empty()) menu.addSectionHeader(utf8("Not loaded (pick one to see why)"));
    for (int index = 0; index < static_cast<int>(unloaded_.size()); ++index) {
        juce::PopupMenu::Item item(unloaded_[static_cast<std::size_t>(index)].file);
        item.itemID = unloadedBase + index;
        item.colour = colours::warn;
        menu.addItem(item);
    }
    auto safe = juce::Component::SafePointer<TopBar>(this);
    menu.showMenuAsync(juce::PopupMenu::Options {}
                           .withTargetComponent(&plate_)
                           .withMinimumWidth(plate_.getWidth()),
                       [safe, unloadedBase](int result) {
                           if (!safe || result <= 0) return;
                           if (result < unloadedBase) {
                               if (safe->onEngineSelected) safe->onEngineSelected(result - 1);
                               return;
                           }
                           const auto index = static_cast<std::size_t>(result - unloadedBase);
                           if (index < safe->unloaded_.size() && safe->onUnloadedEngineSelected)
                               safe->onUnloadedEngineSelected(safe->unloaded_[index]);
                       });
}

void TopBar::refresh(const DashboardModel& model, const Switches& switches) {
    plate_.setEnabled(!model.dynoRunning);
    const auto& state = model.state;
    const auto runningState = state.runningState;
    const auto running = runningState != RunningState::stopped && runningState != RunningState::cranking
        && runningState != RunningState::destroyed;
    std::array<bool, lampCount> lit {};
    std::array<juce::Colour, lampCount> colour {};
    for (std::size_t index = 0; index < lampCount; ++index) colour[index] = lamps_[index].colour;
    lit[ignLamp] = switches.ignition;
    lit[crankLamp] = runningState == RunningState::cranking;
    lit[runLamp] = running;
    colour[runLamp] = runningState == RunningState::idling || runningState == RunningState::running
        ? colours::ok : colours::warn;
    lit[dynoLamp] = model.dynoRunning;
    lit[boostLamp] = running && state.boostPressureRatio > 1.03;
    lit[limitLamp] = hasLimit(state, ActiveLimit::revLimiter) || hasLimit(state, ActiveLimit::launchControl)
        || hasLimit(state, ActiveLimit::overboostCut);
    lit[knockLamp] = hasLimit(state, ActiveLimit::knockRetard) || runningState == RunningState::knocking;
    const auto& diagnostics = model.diagnostics;
    const auto critical = std::any_of(diagnostics.begin(), diagnostics.end(), [](const Diagnostic& item) {
        return item.severity == DiagnosticSeverity::critical;
    });
    const auto warning = std::any_of(diagnostics.begin(), diagnostics.end(), [](const Diagnostic& item) {
        return item.severity == DiagnosticSeverity::warning;
    });
    const auto broken = runningState == RunningState::damaged || runningState == RunningState::destroyed;
    lit[faultLamp] = critical || warning || broken;
    colour[faultLamp] = critical || broken ? colours::critical : colours::warn;
    lit[logLamp] = switches.datalogRecording;
    lit[versionLamp] = switches.abOnB;
    lit[pauseLamp] = model.health.paused;

    auto changed = false;
    for (std::size_t index = 0; index < lampCount; ++index) {
        auto& lamp = lamps_[index];
        if (lamp.lit != lit[index] || lamp.colour != colour[index]) {
            lamp.lit = lit[index];
            lamp.colour = colour[index];
            changed = true;
        }
    }
    lamps_[runLamp].tooltip = utf8("Engine state: ") + juce::String(runningStateName(runningState)).toLowerCase();
    lamps_[faultLamp].tooltip = diagnostics.empty()
        ? (broken ? utf8("The engine is ") + juce::String(runningStateName(runningState)).toLowerCase()
                  : utf8("No fault detected"))
        : juce::String::fromUTF8(diagnostics.front().message.c_str())
            + (diagnostics.size() > 1 ? "  (+" + juce::String(static_cast<int>(diagnostics.size() - 1)) + ")"
                                      : juce::String {});
    if (switches.datalogTooltip.isNotEmpty()) lamps_[logLamp].tooltip = switches.datalogTooltip;
    if (changed) repaint(lampsArea().expanded(4));
}

int TopBar::shownLampCount() const {
    return static_cast<int>(std::count_if(lamps_.begin(), lamps_.end(), [](const Lamp& lamp) { return lamp.shown; }));
}

juce::Rectangle<int> TopBar::lampsArea() const {
    juce::Rectangle<float> area;
    for (const auto& lamp : lamps_)
        if (lamp.shown) area = area.isEmpty() ? lamp.area : area.getUnion(lamp.area);
    return area.getSmallestIntegerContainer();
}

juce::String TopBar::getTooltip() {
    const auto position = getMouseXYRelative().toFloat();
    for (const auto& lamp : lamps_)
        if (lamp.shown && lamp.area.contains(position)) return lamp.tooltip;
    return {};
}

void TopBar::paint(juce::Graphics& g) {
    g.fillAll(colours::chrome);
    g.setColour(colours::line);
    g.fillRect(getLocalBounds().removeFromBottom(1));

    const auto mid = static_cast<float>(getHeight()) * 0.5F;
    drawMark(g, { 14.0F, mid - 10.0F, 20.0F, 20.0F });
    g.setColour(colours::text);
    g.setFont(uiFont(17.0F, true));
    g.drawText("EngineLab", juce::Rectangle<float>(42.0F, 0.0F, 90.0F, static_cast<float>(getHeight())),
               juce::Justification::centredLeft, false);

    g.setFont(lampFont());
    for (const auto& lamp : lamps_) {
        if (!lamp.shown) continue;
        if (lamp.lit) {
            g.setColour(lamp.colour);
            g.fillRoundedRectangle(lamp.area, cornerRadius);
            g.setColour(colours::ink);
        } else {
            g.setColour(colours::well);
            g.fillRoundedRectangle(lamp.area, cornerRadius);
            g.setColour(colours::line);
            g.drawRoundedRectangle(lamp.area.reduced(0.5F), cornerRadius, 1.0F);
            g.setColour(colours::faint);
        }
        g.drawText(lamp.text, lamp.area.translated(0.0F, 0.5F), juce::Justification::centred, false);
    }
}

void TopBar::resized() {
    auto area = getLocalBounds().withTrimmedBottom(1);
    area.removeFromLeft(140);
    area.removeFromRight(8);
    // The menus, right to left; they always stay.
    for (auto index = static_cast<int>(menuOrder.size()) - 1; index >= 0; --index) {
        auto& menu = *menus_[static_cast<std::size_t>(index)];
        menu.setBounds(area.removeFromRight(menu.idealWidth()));
        if (index == static_cast<int>(menuOrder.size()) - 1) area.removeFromRight(4);
    }
    area.removeFromRight(16);
    // The plate takes a fifth of the bar; the lamps what is left after it.
    const auto plateWidth = std::clamp(getWidth() / 5, 220, 380);
    constexpr float gap = 3.0F;
    auto lampsWidth = [this] {
        auto total = 0.0F;
        for (const auto& lamp : lamps_) if (lamp.shown) total += lampWidth(lamp.text) + gap;
        return total;
    };
    for (auto& lamp : lamps_) lamp.shown = true;
    for (const auto dropped : lampDropOrder) {
        if (lampsWidth() + 16.0F <= static_cast<float>(area.getWidth() - plateWidth)) break;
        lamps_[dropped].shown = false;
    }
    // A plate squeezed below its share before the last lamps go.
    const auto plate = std::clamp(area.getWidth() - static_cast<int>(lampsWidth()) - 16, 180, plateWidth);
    plate_.setBounds(area.removeFromLeft(plate));
    area.removeFromLeft(16);
    auto x = static_cast<float>(area.getX());
    const auto lampHeight = 22.0F;
    const auto y = static_cast<float>(getHeight() - 1) * 0.5F - lampHeight * 0.5F;
    for (auto& lamp : lamps_) {
        if (!lamp.shown) { lamp.area = {}; continue; }
        const auto width = lampWidth(lamp.text);
        lamp.area = { x, y, width, lampHeight };
        x += width + gap;
    }
}

} // namespace enginelab::ui
