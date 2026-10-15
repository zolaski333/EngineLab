#include <enginelab/foundation/Units.hpp>

#include <cmath>
#include <limits>

namespace enginelab::units {
namespace {
// Exact definitions: international mile and pound, US and imperial gallons,
// standard gravity, mechanical horsepower (550 ft lbf/s) and metric PS.
constexpr double metresPerMile = 1'609.344;
constexpr double litresPerUsGallon = 3.785'411'784;
constexpr double litresPerImperialGallon = 4.546'09;
constexpr double kilogramsPerPound = 0.453'592'37;
constexpr double standardGravity = 9.806'65;
constexpr double kilowattsPerHorsepower = 0.745'699'871'582'270'2;
constexpr double kilowattsPerPs = 0.735'498'75;
constexpr double kilopascalsPerPsi = 6.894'757'293'168'361;
constexpr double newtonMetresPerPoundFoot = 1.355'817'948'331'400'4;

constexpr std::array speed {
    UnitDefinition { "m_s", "m/s", 1.0, 0.0, false, 1 },
    UnitDefinition { "km_h", "km/h", 3.6, 0.0, false, 0 },
    UnitDefinition { "mph", "mph", 1.0 / (metresPerMile / 3'600.0), 0.0, false, 0 },
};
constexpr std::array distance {
    UnitDefinition { "m", "m", 1.0, 0.0, false, 0 },
    UnitDefinition { "km", "km", 0.001, 0.0, false, 2 },
    UnitDefinition { "mi", "mi", 1.0 / metresPerMile, 0.0, false, 2 },
};
// MPG = (100 km per 100 km) / (L/100 km) expressed in miles per gallon.
constexpr std::array fuelConsumption {
    UnitDefinition { "l_100km", "L/100 km", 1.0, 0.0, false, 1 },
    UnitDefinition { "km_l", "km/L", 100.0, 0.0, true, 1 },
    UnitDefinition { "mpg_us", "MPG (US)", 100'000.0 * litresPerUsGallon / metresPerMile, 0.0, true, 1 },
    UnitDefinition { "mpg_uk", "MPG (UK)", 100'000.0 * litresPerImperialGallon / metresPerMile, 0.0, true, 1 },
};
constexpr std::array fuelVolume {
    UnitDefinition { "l", "L", 1.0, 0.0, false, 2 },
    UnitDefinition { "gal_us", "gal (US)", 1.0 / litresPerUsGallon, 0.0, false, 2 },
    UnitDefinition { "gal_uk", "gal (UK)", 1.0 / litresPerImperialGallon, 0.0, false, 2 },
};
constexpr std::array pressure {
    UnitDefinition { "kpa", "kPa", 1.0, 0.0, false, 0 },
    UnitDefinition { "bar", "bar", 0.01, 0.0, false, 2 },
    UnitDefinition { "psi", "psi", 1.0 / kilopascalsPerPsi, 0.0, false, 1 },
};
constexpr std::array torque {
    UnitDefinition { "nm", "Nm", 1.0, 0.0, false, 0 },
    UnitDefinition { "lb_ft", "lb\xc2\xb7""ft", 1.0 / newtonMetresPerPoundFoot, 0.0, false, 0 },
    UnitDefinition { "kgf_m", "kgf\xc2\xb7m", 1.0 / standardGravity, 0.0, false, 1 },
};
constexpr std::array power {
    UnitDefinition { "kw", "kW", 1.0, 0.0, false, 1 },
    UnitDefinition { "hp", "hp", 1.0 / kilowattsPerHorsepower, 0.0, false, 1 },
    UnitDefinition { "ps", "PS", 1.0 / kilowattsPerPs, 0.0, false, 1 },
};
constexpr std::array temperature {
    UnitDefinition { "c", "\xc2\xb0""C", 1.0, 0.0, false, 0 },
    UnitDefinition { "f", "\xc2\xb0""F", 1.8, 32.0, false, 0 },
    UnitDefinition { "k", "K", 1.0, 273.15, false, 0 },
};
constexpr std::array length {
    UnitDefinition { "mm", "mm", 1.0, 0.0, false, 1 },
    UnitDefinition { "in", "in", 1.0 / 25.4, 0.0, false, 2 },
};
constexpr std::array displacement {
    UnitDefinition { "cm3", "cm\xc2\xb3", 1.0, 0.0, false, 0 },
    UnitDefinition { "l", "L", 0.001, 0.0, false, 2 },
    UnitDefinition { "cu_in", "cu in", 1.0 / 16.387'064, 0.0, false, 0 },
};
constexpr std::array mass {
    UnitDefinition { "kg", "kg", 1.0, 0.0, false, 1 },
    UnitDefinition { "lb", "lb", 1.0 / kilogramsPerPound, 0.0, false, 1 },
};
constexpr std::array airFlow {
    UnitDefinition { "g_s", "g/s", 1.0, 0.0, false, 1 },
    UnitDefinition { "kg_h", "kg/h", 3.6, 0.0, false, 0 },
    UnitDefinition { "lb_min", "lb/min", 60.0 / 453.592'37, 0.0, false, 1 },
};

constexpr std::array<std::span<const UnitDefinition>, quantityCount> table {
    std::span<const UnitDefinition>(speed), std::span<const UnitDefinition>(distance),
    std::span<const UnitDefinition>(fuelConsumption), std::span<const UnitDefinition>(fuelVolume),
    std::span<const UnitDefinition>(pressure), std::span<const UnitDefinition>(torque),
    std::span<const UnitDefinition>(power), std::span<const UnitDefinition>(temperature),
    std::span<const UnitDefinition>(length), std::span<const UnitDefinition>(displacement),
    std::span<const UnitDefinition>(mass), std::span<const UnitDefinition>(airFlow),
};

constexpr std::size_t index(Quantity quantity) noexcept { return static_cast<std::size_t>(quantity); }

UnitPreferences makePreferences(std::array<std::uint8_t, quantityCount> unit, PressureReference reference) {
    UnitPreferences preferences;
    preferences.unit = unit;
    preferences.pressureReference = reference;
    return preferences;
}

// Column order follows Quantity: speed, distance, consumption, fuel volume,
// pressure, torque, power, temperature, length, displacement, mass, air flow.
const std::array<UnitPreset, 4> presetTable {
    UnitPreset { "Metric", makePreferences({ 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, PressureReference::absolute) },
    UnitPreset { "Metric (bar, PS)", makePreferences({ 1, 1, 0, 0, 1, 0, 2, 0, 0, 0, 0, 1 }, PressureReference::gauge) },
    UnitPreset { "US", makePreferences({ 2, 2, 2, 1, 2, 1, 1, 1, 1, 2, 1, 2 }, PressureReference::gauge) },
    UnitPreset { "UK", makePreferences({ 2, 2, 3, 0, 2, 1, 1, 0, 0, 0, 0, 0 }, PressureReference::gauge) },
};
} // namespace

std::span<const UnitDefinition> unitsOf(Quantity quantity) noexcept {
    return index(quantity) < quantityCount ? table[index(quantity)] : table[0];
}

std::string_view quantityId(Quantity quantity) noexcept {
    switch (quantity) {
    case Quantity::speed: return "speed";
    case Quantity::distance: return "distance";
    case Quantity::fuelConsumption: return "fuel_consumption";
    case Quantity::fuelVolume: return "fuel_volume";
    case Quantity::pressure: return "pressure";
    case Quantity::torque: return "torque";
    case Quantity::power: return "power";
    case Quantity::temperature: return "temperature";
    case Quantity::length: return "length";
    case Quantity::displacement: return "displacement";
    case Quantity::mass: return "mass";
    case Quantity::airFlow: return "air_flow";
    case Quantity::count: break;
    }
    return "unknown";
}

std::string_view quantityLabel(Quantity quantity) noexcept {
    switch (quantity) {
    case Quantity::speed: return "Speed";
    case Quantity::distance: return "Distance";
    case Quantity::fuelConsumption: return "Fuel consumption";
    case Quantity::fuelVolume: return "Fuel volume";
    case Quantity::pressure: return "Pressure";
    case Quantity::torque: return "Torque";
    case Quantity::power: return "Power";
    case Quantity::temperature: return "Temperature";
    case Quantity::length: return "Length";
    case Quantity::displacement: return "Displacement";
    case Quantity::mass: return "Mass";
    case Quantity::airFlow: return "Air flow";
    case Quantity::count: break;
    }
    return "Unknown";
}

const UnitDefinition& UnitPreferences::of(Quantity quantity) const noexcept {
    const auto units = unitsOf(quantity);
    const auto chosen = index(quantity) < quantityCount ? unit[index(quantity)] : 0U;
    return units[chosen < units.size() ? chosen : 0U];
}

void UnitPreferences::select(Quantity quantity, std::size_t unitIndex) noexcept {
    if (index(quantity) < quantityCount && unitIndex < unitsOf(quantity).size())
        unit[index(quantity)] = static_cast<std::uint8_t>(unitIndex);
}

bool UnitPreferences::select(Quantity quantity, std::string_view id) noexcept {
    const auto units = unitsOf(quantity);
    for (std::size_t candidate = 0; candidate < units.size(); ++candidate) {
        if (units[candidate].id == id) {
            select(quantity, candidate);
            return true;
        }
    }
    return false;
}

std::span<const UnitPreset> presets() noexcept { return presetTable; }

double toDisplay(const UnitPreferences& preferences, Quantity quantity, double baseValue,
                 double ambientKpa) noexcept {
    const auto& unit = preferences.of(quantity);
    if (quantity == Quantity::pressure && preferences.pressureReference == PressureReference::gauge)
        baseValue -= ambientKpa;
    if (unit.reciprocal)
        return baseValue > 0.0 ? unit.scale / baseValue : std::numeric_limits<double>::infinity();
    return baseValue * unit.scale + unit.offset;
}

double fromDisplay(const UnitPreferences& preferences, Quantity quantity, double displayValue,
                   double ambientKpa) noexcept {
    const auto& unit = preferences.of(quantity);
    auto base = unit.reciprocal
        ? (displayValue > 0.0 ? unit.scale / displayValue : std::numeric_limits<double>::infinity())
        : (displayValue - unit.offset) / unit.scale;
    if (quantity == Quantity::pressure && preferences.pressureReference == PressureReference::gauge)
        base += ambientKpa;
    return base;
}

std::string_view symbol(const UnitPreferences& preferences, Quantity quantity) noexcept {
    if (quantity == Quantity::pressure && preferences.pressureReference == PressureReference::gauge) {
        switch (preferences.unit[index(Quantity::pressure)]) {
        case 1: return "bar (g)";
        case 2: return "psi (g)";
        default: return "kPa (g)";
        }
    }
    return preferences.of(quantity).symbol;
}

} // namespace enginelab::units
