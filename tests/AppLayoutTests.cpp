// The main window's own components, built for real (audit lot 7): the key
// map's conflicts and reserved chords, the top bar at the narrowest window,
// the telemetry tab's height with VTEC, the readout strip at its narrowest.
// ENGINELAB_SNAPSHOT_DIR, when set, receives a PNG of each layout checked.

#include <enginelab/app/ActionMap.hpp>
#include <enginelab/app/DashboardModel.hpp>
#include <enginelab/app/ReadoutStrip.hpp>
#include <enginelab/app/SidePanel.hpp>
#include <enginelab/app/Theme.hpp>
#include <enginelab/app/TopBar.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>

#include <juce_gui_basics/juce_gui_basics.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

void require(bool condition, std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

void snapshot(juce::Component& component, const char* name) {
    const auto directory = juce::SystemStats::getEnvironmentVariable("ENGINELAB_SNAPSHOT_DIR", {});
    if (directory.isEmpty()) return;
    const auto image = component.createComponentSnapshot(component.getLocalBounds(), true, 1.0F);
    juce::File file = juce::File(directory).getChildFile(name);
    (void)file.deleteFile();
    juce::FileOutputStream stream(file);
    juce::PNGImageFormat png;
    require(stream.openedOk() && png.writeImageToStream(image, stream), "a snapshot is written");
}

[[nodiscard]] juce::KeyPress parsed(const char* description) {
    return juce::KeyPress::createFromDescription(description);
}

void checkActionMap() {
    using enginelab::ActionMap;
    using enginelab::AppAction;
    ActionMap map { ActionMap::DefaultsOnly {} };
    juce::String error;
    // UA-03: an unmodified binding also answers its key with Shift held, so a
    // Shift binding of the same key is the same event.
    require(!map.fromJson(R"({ "pause": "shift + Q" })", error) && error.contains("Conflict"),
            "Shift+Q conflicts with the throttle's Q");
    require(map.fromJson(R"({ "pause": "ctrl + Q" })", error), "Ctrl+Q does not conflict with Q");
    // UA-04: the save chords are the main component's before any action.
    ActionMap fresh { ActionMap::DefaultsOnly {} };
    for (const auto* chord : { R"({ "pause": "ctrl + S" })", R"({ "pause": "ctrl + shift + S" })" })
        require(!fresh.fromJson(chord, error) && error.contains("Reserved"), "a save chord is reserved");
    require(fresh.fromJson(R"({ "pause": "alt + S" })", error), "Alt+S is free");
    // UA-07: the menu and the engine picker have chords of their own, which
    // no held control reads.
    ActionMap defaults { ActionMap::DefaultsOnly {} };
    const auto menu = juce::KeyPress('M', juce::ModifierKeys::commandModifier, 0);
    require(defaults.matches(AppAction::moreMenu, menu), "Ctrl+M opens the menu");
    require(!defaults.matches(AppAction::layerUp, menu), "Ctrl+M is not the layer step");
    require(defaults.matches(AppAction::layerUp, juce::KeyPress('M', {}, 'm')), "M still steps the layer");
    require(defaults.matches(AppAction::engineMenu, juce::KeyPress('P', juce::ModifierKeys::commandModifier, 0)),
            "Ctrl+P opens the engine picker");
    ActionMap roundTrip { ActionMap::DefaultsOnly {} };
    require(roundTrip.fromJson(defaults.toJson(), error), "the defaults read back: " + error.toStdString());
    require(parsed("ctrl + M").isValid(), "a chord's description parses");
}

// The keyboard of 0.13.0, the interface before "Test cell": a new interface
// may move a command, never its key. tests/data/ui/default-keybindings.json
// is that reference; ENGINELAB_SNAPSHOT_DIR receives today's map beside it.
void checkKeymapUnchanged() {
    const enginelab::ActionMap defaults { enginelab::ActionMap::DefaultsOnly {} };
    const auto json = defaults.toJson();
    const auto directory = juce::SystemStats::getEnvironmentVariable("ENGINELAB_SNAPSHOT_DIR", {});
    if (directory.isNotEmpty())
        (void)juce::File(directory).getChildFile("default-keybindings.json").replaceWithText(json);
    const auto reference = juce::File(ENGINELAB_CATALOG_ROOT).getChildFile("tests/data/ui/default-keybindings.json");
    require(reference.existsAsFile(), "the reference key map exists");
    const auto normalised = [](juce::String text) { return text.removeCharacters("\r").trim(); };
    require(normalised(reference.loadFileAsString()) == normalised(json),
            "every default key is the one of 0.13.0 (tests/data/ui/default-keybindings.json)");
}

// The "Test cell" theme: every text colour readable on every panel fill
// (WCAG 2.1 1.4.3, 4.5:1), and the characters the text fonts lack drawn by
// the embedded fallbacks, not by a system font or an empty box.
void checkTheme() {
    namespace colours = enginelab::ui::colours;
    using enginelab::ui::contrastRatio;
    require(std::abs(contrastRatio(juce::Colours::black, juce::Colours::white) - 21.0) < 1.0e-9,
            "the contrast ratio is WCAG's (black on white is 21:1)");
    const std::pair<const char*, juce::Colour> texts[] = {
        { "text", colours::text }, { "muted", colours::muted }, { "label", colours::label },
        { "accent", colours::accent }, { "ok", colours::ok }, { "critical", colours::critical },
        { "intake", colours::intake }, { "pressureLow", colours::pressureLow } };
    const std::pair<const char*, juce::Colour> fills[] = {
        { "background", colours::background }, { "chrome", colours::chrome }, { "panel", colours::panel },
        { "panelRaised", colours::panelRaised }, { "well", colours::well }, { "selection", colours::selection },
        { "okFill", colours::okFill }, { "warnFill", colours::warnFill }, { "criticalFill", colours::criticalFill } };
    for (const auto& [textName, textColour] : texts)
        for (const auto& [fillName, fillColour] : fills)
            require(contrastRatio(textColour, fillColour) >= 4.5,
                    std::string(textName) + " on " + fillName + " reaches 4.5:1");
    // Ink on a lit lamp or an inverted control.
    for (const auto lit : { colours::text, colours::accent, colours::ok, colours::critical })
        require(contrastRatio(colours::ink, lit) >= 4.5, "ink on a lit lamp reaches 4.5:1");

    const auto lambda = enginelab::ui::utf8("\xce\xbb");
    const auto sans = enginelab::ui::uiFont(14.0F).findSuitableFontForText(lambda);
    const auto mono = enginelab::ui::monoFont(14.0F).findSuitableFontForText(lambda);
    require(sans.getTypefacePtr() != nullptr && sans.getTypefacePtr()->getName().startsWith("Inter"),
            "a lambda in interface text is drawn by the embedded Inter");
    require(mono.getTypefacePtr() != nullptr && mono.getTypefacePtr()->getName().startsWith("JetBrains"),
            "a lambda in a figure is drawn by the embedded JetBrains Mono");
    const auto plain = enginelab::ui::uiFont(14.0F).findSuitableFontForText("Torque");
    require(plain.getTypefacePtr() != nullptr && plain.getTypefacePtr()->getName().startsWith("Barlow"),
            "Latin text stays in Barlow Semi Condensed");
}

// Every command the top bar of 0.13.0 offered (tests/data/ui/commands.txt,
// "top bar") is in one of the Test cell menus, and only once.
void checkMenusCoverCommands() {
    using enginelab::ui::TopMenu;
    const auto inventory = juce::File(ENGINELAB_CATALOG_ROOT).getChildFile("tests/data/ui/commands.txt");
    require(inventory.existsAsFile(), "the command inventory exists");
    juce::StringArray listed;
    for (const auto menu : { TopMenu::design, TopMenu::tune, TopMenu::test, TopMenu::listen, TopMenu::more })
        for (const auto& entry : enginelab::ui::menuEntries(menu, {})) {
            require(!listed.contains(entry.id), "a command is in one menu only: " + entry.id.toStdString());
            listed.add(entry.id);
        }
    juce::StringArray lines;
    lines.addLines(inventory.loadFileAsString());
    auto topBarCommands = 0;
    for (const auto& line : lines) {
        if (line.trim().isEmpty() || line.trimStart().startsWithChar('#')) continue;
        const auto id = line.upToFirstOccurrenceOf("|", false, false).trim();
        const auto where = line.fromFirstOccurrenceOf("|", false, false).trim();
        if (!where.startsWith("top bar") || id == "engine.pick") continue;
        ++topBarCommands;
        require(listed.contains(id), "the command " + id.toStdString() + " of the 0.13.0 top bar is in a menu");
    }
    require(topBarCommands >= 20, "the inventory lists the top bar's commands");
}

void checkTopBar() {
    enginelab::ui::TopBar bar;
    // 1024 is a 1280 px window at 125 %.
    for (const auto width : { 1024, 1280, 1440, 1920, 2560 }) {
        bar.setBounds(0, 0, width, enginelab::ui::TopBar::height);
        std::vector<juce::Component*> parts;
        for (auto* child : bar.getChildren())
            if (child->isVisible()) parts.push_back(child);
        require(parts.size() == 6, "the plate and the five menus are always shown");
        for (std::size_t a = 0; a < parts.size(); ++a) {
            require(bar.getLocalBounds().contains(parts[a]->getBounds()), "a menu stays in the bar");
            require(!parts[a]->getBounds().intersects(bar.lampsArea()),
                    "at " + std::to_string(width) + " px the lamps leave the menus and the plate alone");
            for (std::size_t b = a + 1; b < parts.size(); ++b)
                require(!parts[a]->getBounds().intersects(parts[b]->getBounds()),
                        "at " + std::to_string(width) + " px two menus do not overlap (audit APP-09)");
        }
        if (width >= 1440) require(bar.shownLampCount() == 11, "a wide bar shows every lamp");
        require(bar.shownLampCount() >= 6, "even a short bar keeps the essential lamps");
        if (width == 1280) snapshot(bar, "topbar-1280.png");
    }
}

void checkTelemetry(const enginelab::EngineCatalogLoadResult& catalog) {
    const auto ls3 = std::find_if(catalog.entries.begin(), catalog.entries.end(),
        [](const auto& entry) { return entry.config.name.find("LS3") != std::string::npos; });
    require(ls3 != catalog.entries.end(), "catalog has the LS3");
    auto plain = ls3->config;
    auto vtec = ls3->config;
    require(!enginelab::vtecCamsFitted(plain) && enginelab::fitVtecCams(vtec), "VTEC cams fit the LS3");
    // A model holds a whole EngineState and render snapshot: on the heap.
    const auto plainModel = std::make_unique<enginelab::DashboardModel>(plain);
    const auto vtecModel = std::make_unique<enginelab::DashboardModel>(vtec);
    enginelab::ui::TelemetryPanel plainPanel(*plainModel);
    enginelab::ui::TelemetryPanel vtecPanel(*vtecModel);
    // APP-07: the VTEC row is one 24 px row more, and the buttons beside the
    // fuel-injection heading move down with it.
    require(vtecPanel.preferredHeight() - plainPanel.preferredHeight() == 24,
            "the VTEC row adds its height to the telemetry tab");
    plainPanel.setSize(380, plainPanel.preferredHeight());
    vtecPanel.setSize(380, vtecPanel.preferredHeight());
    std::vector<int> plainY, vtecY;
    for (auto* child : plainPanel.getChildren()) plainY.push_back(child->getY());
    for (auto* child : vtecPanel.getChildren()) vtecY.push_back(child->getY());
    require(plainY.size() == vtecY.size() && !plainY.empty(), "both panels have their buttons");
    for (std::size_t k = 0; k < plainY.size(); ++k)
        require(vtecY[k] - plainY[k] == 24, "the fuel-injection buttons follow the VTEC row");
    snapshot(vtecPanel, "telemetry-vtec.png");
}

void checkReadouts(const enginelab::EngineCatalogLoadResult& catalog) {
    const auto model = std::make_unique<enginelab::DashboardModel>(catalog.entries.front().config);
    model->state.rpm = 3250.0;
    model->state.cycleAveragedTorqueNm = 180.0;
    model->state.cycleAveragedPowerKw = 62.0;
    model->state.exhaustTemperatureC = 722.0;
    enginelab::ui::ReadoutStrip strip(*model);
    strip.setSize(652, 92);
    snapshot(strip, "readouts-652.png");
    strip.setSize(1100, 92);
    snapshot(strip, "readouts-1100.png");
}

} // namespace

int main() {
    juce::ScopedJuceInitialiser_GUI initialiseGui;
    enginelab::ui::LookAndFeel lookAndFeel;
    juce::LookAndFeel::setDefaultLookAndFeel(&lookAndFeel);
    checkTheme();
    const auto catalog = enginelab::loadEngineCatalog(ENGINELAB_CATALOG_ROOT);
    require(!catalog.entries.empty(), "catalog loads");
    checkActionMap();
    checkKeymapUnchanged();
    checkMenusCoverCommands();
    checkTopBar();
    checkTelemetry(catalog);
    checkReadouts(catalog);
    std::cout << "App layout: theme, key map, top bar, telemetry and readouts PASS\n";
    juce::LookAndFeel::setDefaultLookAndFeel(nullptr);
    return 0;
}
