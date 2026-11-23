#include <enginelab/app/TopMenus.hpp>

#include <enginelab/app/Theme.hpp>

namespace enginelab::ui {

juce::String menuTitle(TopMenu menu) {
    switch (menu) {
    case TopMenu::design: return "Design";
    case TopMenu::tune: return "Tune";
    case TopMenu::test: return "Test";
    case TopMenu::listen: return "Listen";
    case TopMenu::more: return "More";
    }
    return {};
}

std::vector<MenuEntry> menuEntries(TopMenu menu, const MenuState& state) {
    const auto loaded = state.engineLoaded;
    const auto idle = !state.dynoRunning;
    std::vector<MenuEntry> entries;
    const auto add = [&entries](juce::String id, juce::String label) -> MenuEntry& {
        auto& entry = entries.emplace_back();
        entry.id = std::move(id);
        entry.label = std::move(label);
        return entry;
    };
    switch (menu) {
    case TopMenu::design: {
        // What the engine is: its build, its parts, its file.
        add("engine.new", utf8("New engine…")).enabled = loaded && idle;
        add("window.exhaust", utf8("Exhaust designer…")).enabled = idle;
        add("window.equipment", utf8("Equipment…"));
        add("window.vehicle", utf8("Vehicle…"));
        auto& save = add("engine.save", state.savedEngine ? utf8("Save engine") : utf8("Save engine…"));
        save.enabled = loaded;
        save.shortcut = "Ctrl+S";
        save.separatorBefore = true;
        auto& saveAs = add("engine.saveAs", utf8("Save engine as…"));
        saveAs.enabled = loaded;
        saveAs.shortcut = "Ctrl+Shift+S";
        add("engine.deleteSaved", utf8("Delete saved engine…")).enabled = state.savedEngine;
        add("engine.import", utf8("Import engine…")).enabled = idle;
        add("engine.export", utf8("Export engine…"));
        add("engine.editJson", utf8("Edit engine JSON…")).enabled = idle;
        auto& reload = add("engine.reload", utf8("Reload engine"));
        reload.enabled = idle;
        reload.shortcut = "Return";
        reload.separatorBefore = true;
        break;
    }
    case TopMenu::tune: {
        // How it is run: maps, fuel, and a second version to compare.
        add("window.ecu", utf8("ECU tuner…")).enabled = idle;
        add("window.fuel", utf8("Fuel…"));
        auto& ab = add("ab.switch", state.abOnB ? utf8("Back to version A") : utf8("Switch to version B"));
        ab.shortcut = state.abKey;
        ab.separatorBefore = true;
        break;
    }
    case TopMenu::test: {
        // Measuring it: the dyno, the datalog.
        auto& dyno = add("dyno.toggle", state.dynoRunning ? utf8("Cancel dyno run") : utf8("Start dyno run"));
        dyno.shortcut = state.dynoKey;
        dyno.enabled = loaded;
        add("dyno.exportCsv", utf8("Export dyno CSV…"));
        auto& log = add("datalog.toggle", state.datalogRecording ? utf8("Stop the datalog")
                                                                 : utf8("Start a datalog"));
        log.shortcut = state.datalogKey;
        log.ticked = state.datalogRecording;
        log.separatorBefore = true;
        break;
    }
    case TopMenu::listen: {
        // Hearing it: the mix and voicing, and the car driving past.
        add("window.audio", utf8("Audio workshop…"));
        add("window.passBy", utf8("Pass-by…"));
        break;
    }
    case TopMenu::more: {
        add("settings.keyBindings", utf8("Key bindings…"));
        add("settings.units", utf8("Units")).subMenu = true;
        auto& full = add("view.fullScreen", utf8("Full screen"));
        full.shortcut = state.fullScreenKey;
        full.ticked = state.fullScreen;
        break;
    }
    }
    return entries;
}

} // namespace enginelab::ui
