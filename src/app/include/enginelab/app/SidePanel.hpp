#pragma once

#include <enginelab/app/ActionMap.hpp>
#include <enginelab/app/DashboardModel.hpp>
#include <enginelab/app/DynoPanel.hpp>
#include <enginelab/app/Widgets.hpp>
#include <functional>
#include <utility>
#include <vector>

namespace enginelab::ui {

/** Ten seconds of traces, then the engine and driveline figures. */
class TelemetryPanel final : public juce::Component {
public:
    explicit TelemetryPanel(const DashboardModel&);
    [[nodiscard]] int preferredHeight() const;
    void paint(juce::Graphics&) override;

private:
    const DashboardModel& model_;
};

/** Exhaust preset, impulse response and the mix levels the wheel adjusts. */
class AudioPanel final : public juce::Component {
public:
    explicit AudioPanel(const DashboardModel&);
    void setKeyCaps(const ActionMap&);
    void syncExhaustPreset();
    [[nodiscard]] int preferredHeight() const;
    std::function<void(int presetIndex)> onExhaustPreset;
    void paint(juce::Graphics&) override;
    void resized() override;

private:
    const DashboardModel& model_;
    juce::ComboBox exhaustPreset_;
    std::vector<juce::String> keyCaps_;
};

/** Active faults, then every physics and real-time counter. */
class DiagnosticsPanel final : public juce::Component {
public:
    explicit DiagnosticsPanel(const DashboardModel&);
    [[nodiscard]] int preferredHeight() const;
    void paint(juce::Graphics&) override;

private:
    [[nodiscard]] std::vector<std::pair<juce::String, juce::String>> debugRows() const;
    const DashboardModel& model_;
};

/** Right column: Dyno, Telemetry, Audio and Diagnostics tabs. */
class SidePanel final : public juce::Component {
public:
    enum Tab { dynoTab, telemetryTab, audioTab, diagnosticsTab, tabCount };

    SidePanel(const DashboardModel&, std::function<bool()> wheelModifierActive);
    void showTab(int tab);
    void nextTab();
    [[nodiscard]] int currentTab() const noexcept { return tabs_.selected(); }
    /** Per-frame refresh of the visible tab only. */
    void refresh();

    DynoPanel dyno;
    AudioPanel audio;

    void paint(juce::Graphics&) override;
    void resized() override;
    static constexpr int width = 380;

private:
    void fitContent(ScrollPane&, juce::Component& content, int height);
    const DashboardModel& model_;
    SegmentedControl tabs_ { { "Dyno", "Telemetry", "Audio", "Diagnostics" },
                             SegmentedControl::Style::tabs };
    TelemetryPanel telemetry_;
    DiagnosticsPanel diagnostics_;
    ScrollPane telemetryPane_;
    ScrollPane audioPane_;
    ScrollPane diagnosticsPane_;
};

} // namespace enginelab::ui
