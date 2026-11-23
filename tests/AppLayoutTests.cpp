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
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <string_view>
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

void checkTopBar() {
    enginelab::ui::TopBar bar;
    for (const auto width : { 1280, 1440, 1920 }) {
        bar.setBounds(0, 0, width, enginelab::ui::TopBar::height);
        std::vector<juce::Component*> buttons;
        for (auto* child : bar.getChildren())
            if (child->isVisible() && dynamic_cast<enginelab::ui::ActionButton*>(child) != nullptr)
                buttons.push_back(child);
        require(buttons.size() >= 2, "the top bar shows its menu and tools");
        for (std::size_t a = 0; a < buttons.size(); ++a) {
            require(bar.getLocalBounds().contains(buttons[a]->getBounds()), "a tool button stays in the bar");
            for (std::size_t b = a + 1; b < buttons.size(); ++b)
                require(!buttons[a]->getBounds().intersects(buttons[b]->getBounds()),
                        "at " + std::to_string(width) + " px two tool buttons do not overlap (audit APP-09)");
        }
        if (width == 1920) require(buttons.size() == 10, "a wide bar shows every tool button");
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
    const auto catalog = enginelab::loadEngineCatalog(ENGINELAB_CATALOG_ROOT);
    require(!catalog.entries.empty(), "catalog loads");
    checkActionMap();
    checkKeymapUnchanged();
    checkTopBar();
    checkTelemetry(catalog);
    checkReadouts(catalog);
    std::cout << "App layout: key map, top bar, telemetry and readouts PASS\n";
    return 0;
}
