#pragma once

#include <enginelab/app/ActionMap.hpp>
#include <enginelab/app/DashboardModel.hpp>
#include <enginelab/app/DynoPanel.hpp>
#include <enginelab/app/Widgets.hpp>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

namespace enginelab::ui {

/** Ten seconds of traces, then the engine and driveline figures. */
class TelemetryPanel final : public juce::Component {
public:
    explicit TelemetryPanel(const DashboardModel&);
    [[nodiscard]] int preferredHeight() const;
    /** Called by the fuel-injection section's button: show the injectors in
        the 3-D view, with their inspector. */
    std::function<void()> onShowInjectors;
    /** Called by the section's Fuel button: the combustion chamber's
        inspector, where the fuel is chosen and edited. */
    std::function<void()> onShowFuel;
    void paint(juce::Graphics&) override;
    void resized() override;

private:
    const DashboardModel& model_;
    ActionButton showInjectors_ { "Show in 3-D", ActionButton::Style::compact };
    ActionButton showFuel_ { "Fuel", ActionButton::Style::compact };
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

/** The vehicle the engine drives: its figures, each typed in and taken by
    the running engine (Enter or leaving the field), and what it does now.
    Shown in its own window, from the top bar's Vehicle button. */
class VehiclePanel final : public juce::Component {
public:
    explicit VehiclePanel(const DashboardModel&);
    ~VehiclePanel() override;
    [[nodiscard]] int preferredHeight() const;
    /** Shows the running vehicle's figures in every field not being typed in. */
    void refresh();
    /** Called with the vehicle as edited: empty when the engine took it, else
        why not. */
    std::function<juce::String(const VehicleConfig&)> onEdit;
    void paint(juce::Graphics&) override;
    void resized() override;

    struct Field final {
        const char* label;
        const char* unit;
        double VehicleConfig::*member;
        int decimals;
        const char* tooltip;
    };

private:
    void commit(std::size_t field);
    void send(const VehicleConfig&);
    const DashboardModel& model_;
    std::vector<std::unique_ptr<juce::TextEditor>> editors_;
    juce::ComboBox drivenAxles_;
    juce::ToggleButton tyreGrip_ { "Tyres can spin (grip limit)" };
    juce::String status_;
    VehicleConfig shown_;
    bool shownOnce_ { false };
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
    /** The Telemetry tab asks to show the injectors in the 3-D view. */
    std::function<void()> onShowInjectors;
    /** The Telemetry tab asks for the fuel's inspector. */
    std::function<void()> onShowFuel;

    void paint(juce::Graphics&) override;
    void resized() override;
    static constexpr int width = 380;

private:
    void fitContent(ScrollPane&, juce::Component& content, int height);
    // No `model_` here on purpose: every tab below holds its own reference,
    // and the panel itself never reads one.
    SegmentedControl tabs_ { { "Dyno", "Telemetry", "Audio", "Diagnostics" },
                             SegmentedControl::Style::tabs };
    TelemetryPanel telemetry_;
    DiagnosticsPanel diagnostics_;
    ScrollPane telemetryPane_;
    ScrollPane audioPane_;
    ScrollPane diagnosticsPane_;
};

} // namespace enginelab::ui
