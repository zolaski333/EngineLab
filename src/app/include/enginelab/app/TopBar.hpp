#pragma once

#include <enginelab/app/DashboardModel.hpp>
#include <enginelab/app/TopMenus.hpp>
#include <enginelab/app/Widgets.hpp>
#include <array>
#include <functional>
#include <vector>

namespace enginelab::ui {

/** Brand, engine plate, annunciator lamps and the menus Design, Tune, Test,
    Listen and the ⋯ ("Test cell"). A lamp is lit by the engine's state; the
    menus' contents are TopMenus' data, turned into actions by the main
    component (`onMenu`). */
class TopBar final : public juce::Component, public juce::TooltipClient {
public:
    /** One line of the engine menu. */
    struct EngineChoice final {
        juce::String name;
        /** Saved by the user, listed under "My engines". */
        bool saved { false };
        /** Edited this session and not saved: switching back restores the edits. */
        bool edited { false };
    };

    /** An engine file that did not load: listed at the bottom of the menu,
        so a hand-edited engine never just disappears. */
    struct UnloadedEngine final {
        juce::String file;
        juce::String reason;
    };

    /** What the lamps show that the dashboard model does not carry. */
    struct Switches final {
        bool ignition { false };
        bool datalogRecording { false };
        bool abOnB { false };
        juce::String datalogTooltip {};
    };

    TopBar();
    void setEngineChoices(std::vector<EngineChoice>);
    void setUnloadedEngines(std::vector<UnloadedEngine>);
    /** `presetIndex` is -1 for an engine that is not one of the presets. */
    void setEngine(const EngineConfig&, int presetIndex);
    /** The running engine differs from the one loaded: a dot on the plate. */
    void setModified(bool);
    void refresh(const DashboardModel&, const Switches&);

    std::function<void(int presetIndex)> onEngineSelected;
    /** An unloaded engine was picked: say why it did not load. */
    std::function<void(const UnloadedEngine&)> onUnloadedEngineSelected;
    /** A menu was opened: show its entries under `anchor`, and call
        `menuClosed` when it closes. */
    std::function<void(TopMenu, juce::Component& anchor)> onMenu;

    /** Opens the engine menu under the plate (also from the keyboard). */
    void showEngineMenu();
    /** Opens a menu as if clicked (Ctrl+M opens the first). */
    void openMenu(TopMenu);
    void menuClosed();
    [[nodiscard]] juce::Component& menuAnchor(TopMenu);

    void paint(juce::Graphics&) override;
    /** Lamps give way first when the bar is short, least telling first; the
        menus always stay (audit APP-09). */
    void resized() override;
    juce::String getTooltip() override;
    static constexpr int height = 48;

    /** The lamps in the order drawn, for the layout test. */
    [[nodiscard]] int shownLampCount() const;
    [[nodiscard]] juce::Rectangle<int> lampsArea() const;

private:
    class EnginePlate final : public juce::Button {
    public:
        EnginePlate();
        void setEngine(const juce::String& name, const juce::String& spec);
        void setModified(bool);
        void paintButton(juce::Graphics&, bool highlighted, bool down) override;
    private:
        juce::String name_;
        juce::String spec_;
        bool modified_ { false };
    };

    class MenuButton final : public juce::Button {
    public:
        MenuButton(TopBar& owner, TopMenu menu);
        void setOpen(bool);
        [[nodiscard]] int idealWidth() const;
        /** An open menu passes ← and → here: they walk to the next menu. */
        bool keyPressed(const juce::KeyPress&) override;
        void paintButton(juce::Graphics&, bool highlighted, bool down) override;
    private:
        TopBar& owner_;
        TopMenu menu_;
        bool open_ { false };
    };

    struct Lamp final {
        const char* text { nullptr };
        juce::String tooltip {};
        juce::Colour colour {};
        bool lit { false };
        bool shown { true };
        juce::Rectangle<float> area {};
    };

    void stepMenu(TopMenu from, int direction);
    [[nodiscard]] MenuButton& button(TopMenu);

    EnginePlate plate_;
    std::array<std::unique_ptr<MenuButton>, 5> menus_;
    std::vector<Lamp> lamps_;
    std::vector<EngineChoice> choices_;
    std::vector<UnloadedEngine> unloaded_;
    int presetIndex_ { -1 };
};

} // namespace enginelab::ui
