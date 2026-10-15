#pragma once

#include <enginelab/foundation/Units.hpp>
#include <juce_gui_basics/juce_gui_basics.h>
#include <functional>

namespace enginelab::ui {

/** Formats base values in the viewer's units. One per main window; panels get
    it through the DashboardModel. */
class UnitFormat final {
public:
    units::UnitPreferences preferences;
    /** The engine's ambient pressure, the zero of a gauge pressure. */
    double ambientPressureKpa { 101.325 };

    /** The number alone, at the unit's own precision unless `decimals` >= 0.
        An infinite fuel economy (a fuel cut) is shown as a dash. */
    [[nodiscard]] juce::String value(units::Quantity, double baseValue, int decimals = -1) const;
    /** The unit symbol (with "(g)" for a gauge pressure). */
    [[nodiscard]] juce::String symbol(units::Quantity) const;
    /** "123 km/h". */
    [[nodiscard]] juce::String text(units::Quantity, double baseValue, int decimals = -1) const;
    /** A pressure that is already a difference (oil pressure, a pressure
        drop): converted to the pressure unit, never shifted by the ambient. */
    [[nodiscard]] juce::String pressureDifference(double kilopascals, int decimals = -1) const;
    [[nodiscard]] double display(units::Quantity, double baseValue) const noexcept;
    [[nodiscard]] double base(units::Quantity, double displayValue) const noexcept;

    /** Reads `units.json` in the settings folder; unknown entries keep their
        default. A missing or unreadable file leaves the metric default. */
    void load();
    void save() const;
    [[nodiscard]] juce::String toJson() const;
    void fromJson(const juce::String&);

    /** Adds the presets, one submenu per quantity and the pressure reference.
        `changed` runs after any choice (the preferences are already saved). */
    void addMenu(juce::PopupMenu&, std::function<void()> changed);

private:
    [[nodiscard]] static juce::File settingsFile();
};

} // namespace enginelab::ui
