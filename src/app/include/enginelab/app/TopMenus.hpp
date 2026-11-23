#pragma once

#include <juce_gui_basics/juce_gui_basics.h>

#include <vector>

/** The top bar's menus ("Test cell"): what each one lists, as data. The main
    component turns an entry's id into its action; EngineLab.AppLayout checks
    the ids against tests/data/ui/commands.txt, so a command may move between
    menus but never vanish. */
namespace enginelab::ui {

enum class TopMenu { design, tune, test, listen, more };

/** The menus drawn as words in the bar, left to right (`more` is the ⋯). */
inline constexpr TopMenu barMenus[] { TopMenu::design, TopMenu::tune, TopMenu::test, TopMenu::listen };

[[nodiscard]] juce::String menuTitle(TopMenu);

/** What the menus show that depends on the moment. */
struct MenuState final {
    bool engineLoaded { true };
    bool dynoRunning { false };
    /** The running engine is a file of the user's: it saves in place. */
    bool savedEngine { false };
    bool datalogRecording { false };
    bool abOnB { false };
    bool fullScreen { false };
    juce::String datalogKey {};
    juce::String abKey {};
    juce::String dynoKey {};
    juce::String fullScreenKey {};
};

struct MenuEntry final {
    /** The command's id in tests/data/ui/commands.txt. */
    juce::String id {};
    juce::String label {};
    juce::String shortcut {};
    bool enabled { true };
    bool ticked { false };
    /** A rule above it, and a heading when `heading` is not empty. */
    bool separatorBefore { false };
    juce::String heading {};
    /** Its items are built by the main component (Units). */
    bool subMenu { false };
};

[[nodiscard]] std::vector<MenuEntry> menuEntries(TopMenu, const MenuState&);

} // namespace enginelab::ui
