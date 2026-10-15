#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

// Display units. The simulation keeps one internal unit per quantity (the
// "base" column below); only what is shown to the user is converted. The
// viewer picks a unit per quantity, or a preset that sets them all.
namespace enginelab::units {

/** A displayed quantity, with its internal (base) unit. */
enum class Quantity : std::uint8_t {
    speed,           // m/s
    distance,        // m
    fuelConsumption, // L/100 km
    fuelVolume,      // L
    pressure,        // kPa, absolute
    torque,          // N m
    power,           // kW
    temperature,     // degrees Celsius
    length,          // mm
    displacement,    // cm3
    mass,            // kg
    airFlow,         // g/s
    count
};

inline constexpr std::size_t quantityCount = static_cast<std::size_t>(Quantity::count);

/** Whether a pressure is shown as absolute or relative to the ambient air. */
enum class PressureReference : std::uint8_t { absolute, gauge };

/** One unit of a quantity. A linear unit shows `base * scale + offset`; a
    reciprocal one (km/L, MPG) shows `scale / base`. */
struct UnitDefinition final {
    std::string_view id;     // stable, written to settings files
    std::string_view symbol; // shown next to a value (UTF-8)
    double scale { 1.0 };
    double offset { 0.0 };
    bool reciprocal { false };
    int decimals { 0 };      // a sensible default precision
};

/** Every unit offered for a quantity; index 0 is the base unit. */
[[nodiscard]] std::span<const UnitDefinition> unitsOf(Quantity) noexcept;
[[nodiscard]] std::string_view quantityId(Quantity) noexcept;
[[nodiscard]] std::string_view quantityLabel(Quantity) noexcept;

/** The viewer's choice: a unit index per quantity and a pressure reference. */
struct UnitPreferences final {
    std::array<std::uint8_t, quantityCount> unit {};
    PressureReference pressureReference { PressureReference::absolute };

    [[nodiscard]] const UnitDefinition& of(Quantity) const noexcept;
    /** Selects a unit by index; an index out of range is ignored. */
    void select(Quantity, std::size_t index) noexcept;
    /** Selects a unit by id; returns false (and changes nothing) if unknown. */
    bool select(Quantity, std::string_view id) noexcept;

    friend bool operator==(const UnitPreferences&, const UnitPreferences&) = default;
};

struct UnitPreset final {
    std::string_view name;
    UnitPreferences preferences;
};

/** Metric (the default, the simulation's own units), Metric bar/PS, US, UK. */
[[nodiscard]] std::span<const UnitPreset> presets() noexcept;

/** Converts a base value to the chosen unit. A pressure is made relative to
    `ambientKpa` when the gauge reference is chosen. A reciprocal unit of a
    zero or negative base value (no consumption) is +infinity. */
[[nodiscard]] double toDisplay(const UnitPreferences&, Quantity, double baseValue,
                               double ambientKpa = 101.325) noexcept;
/** The inverse of toDisplay, for values typed by the user. */
[[nodiscard]] double fromDisplay(const UnitPreferences&, Quantity, double displayValue,
                                 double ambientKpa = 101.325) noexcept;
/** The symbol shown for a quantity, with " (g)" appended to a gauge pressure. */
[[nodiscard]] std::string_view symbol(const UnitPreferences&, Quantity) noexcept;

} // namespace enginelab::units
