#pragma once

#include <enginelab/app/DashboardModel.hpp>
#include <enginelab/app/Widgets.hpp>
#include <functional>
#include <optional>

namespace enginelab::ui {

/** Torque (solid) and power (dashed) against engine speed for the current run
    and every visible saved run. Hovering reads the nearest point. */
class DynoChart final : public juce::Component {
public:
    explicit DynoChart(const DashboardModel&);
    void paint(juce::Graphics&) override;
    void mouseMove(const juce::MouseEvent&) override;
    void mouseExit(const juce::MouseEvent&) override;

private:
    const DashboardModel& model_;
    std::optional<juce::Point<float>> mouse_;
};

/** Saved runs, newest first. */
class RunList final : public juce::Component {
public:
    explicit RunList(const DashboardModel&);
    std::function<void(int runIndex)> onSelect;
    [[nodiscard]] int preferredHeight() const;
    void paint(juce::Graphics&) override;
    void mouseDown(const juce::MouseEvent&) override;
    static constexpr int rowHeight = 34;

private:
    const DashboardModel& model_;
};

/** The Dyno tab: status, peaks, chart, saved runs and their presentation. */
class DynoPanel final : public juce::Component {
public:
    DynoPanel(const DashboardModel&, std::function<bool()> wheelModifierActive);
    /** Per-frame repaint of the live parts. */
    void refresh();
    /** Call after the saved runs, the selection or a presentation changed. */
    void syncRuns();

    std::function<void(int runIndex)> onSelectRun;
    std::function<void(const juce::String&)> onRenameRun;
    /** Enter or Escape in the run name: the keyboard goes back to the engine. */
    std::function<void()> onDoneTyping;
    std::function<void()> onCycleColour;
    std::function<void()> onToggleVisibility;
    std::function<void()> onDeleteRun;
    std::function<void()> onExportCsv;

    void paint(juce::Graphics&) override;
    void resized() override;

private:
    const DashboardModel& model_;
    DynoChart chart_;
    RunList runs_;
    ScrollPane runsPane_;
    juce::TextEditor runName_;
    ActionButton colourButton_ { "Colour", ActionButton::Style::compact };
    ActionButton visibilityButton_ { "Hide", ActionButton::Style::compact };
    ActionButton deleteButton_ { "Delete", ActionButton::Style::compact };
    ActionButton csvButton_ { "Export CSV", ActionButton::Style::compact };
    juce::Rectangle<float> header_;
    juce::Rectangle<float> peaks_;
    juce::Rectangle<float> runsLabel_;
    bool updatingName_ { false };
};

} // namespace enginelab::ui
