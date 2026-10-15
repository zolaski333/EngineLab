// Display units: exact conversion factors, reciprocal fuel-economy units,
// gauge pressure, round trips, presets and stable ids for settings files.

#include <enginelab/foundation/Units.hpp>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <set>
#include <string>
#include <string_view>

namespace {
using enginelab::units::PressureReference;
using enginelab::units::Quantity;
using enginelab::units::UnitPreferences;

void require(bool condition, std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

bool near(double actual, double expected, double relative = 1.0e-6) {
    return std::abs(actual - expected) <= relative * std::max(1.0, std::abs(expected));
}

UnitPreferences with(Quantity quantity, std::string_view id) {
    UnitPreferences preferences;
    require(preferences.select(quantity, id), "a known unit id is selectable");
    return preferences;
}

double shown(Quantity quantity, std::string_view id, double base) {
    return enginelab::units::toDisplay(with(quantity, id), quantity, base);
}

void checkKnownValues() {
    // Reference values from the exact definitions (NIST SP 811).
    require(near(shown(Quantity::speed, "km_h", 27.777'777'8), 100.0), "100 km/h");
    require(near(shown(Quantity::speed, "mph", 26.8224), 60.0), "60 mph is 26.8224 m/s");
    require(near(shown(Quantity::distance, "mi", 1'609.344), 1.0), "one mile");
    require(near(shown(Quantity::pressure, "psi", 6.894'757), 1.0), "one psi");
    require(near(shown(Quantity::pressure, "bar", 101.325), 1.013'25), "one atmosphere in bar");
    require(near(shown(Quantity::torque, "lb_ft", 1.355'818), 1.0), "one pound-foot");
    require(near(shown(Quantity::torque, "kgf_m", 9.806'65), 1.0), "one kilogram-force metre");
    require(near(shown(Quantity::power, "hp", 0.745'699'9), 1.0), "one mechanical horsepower");
    require(near(shown(Quantity::power, "ps", 0.735'498'75), 1.0), "one metric horsepower");
    require(near(shown(Quantity::temperature, "f", 100.0), 212.0), "water boils at 212 F");
    require(near(shown(Quantity::temperature, "f", -40.0), -40.0), "-40 C is -40 F");
    require(near(shown(Quantity::temperature, "k", 0.0), 273.15), "0 C is 273.15 K");
    require(near(shown(Quantity::length, "in", 25.4), 1.0), "one inch");
    require(near(shown(Quantity::displacement, "cu_in", 5'735.0), 350.0, 1.0e-3), "a 350 cu in V8");
    require(near(shown(Quantity::displacement, "l", 1'998.0), 1.998), "cm3 to litres");
    require(near(shown(Quantity::mass, "lb", 0.453'592'37), 1.0), "one pound");
    require(near(shown(Quantity::airFlow, "lb_min", 453.592'37 / 60.0), 1.0), "one pound per minute");
    require(near(shown(Quantity::airFlow, "kg_h", 1.0), 3.6), "g/s to kg/h");
    require(near(shown(Quantity::fuelVolume, "gal_us", 3.785'411'784), 1.0), "one US gallon");
    require(near(shown(Quantity::fuelVolume, "gal_uk", 4.546'09), 1.0), "one imperial gallon");
}

void checkFuelEconomy() {
    // 235.215 / (L/100 km) = MPG (US); 282.481 / (L/100 km) = MPG (UK).
    require(near(shown(Quantity::fuelConsumption, "mpg_us", 10.0), 23.521'458, 1.0e-6), "10 L/100 km in MPG US");
    require(near(shown(Quantity::fuelConsumption, "mpg_uk", 10.0), 28.248'094, 1.0e-6), "10 L/100 km in MPG UK");
    require(near(shown(Quantity::fuelConsumption, "km_l", 5.0), 20.0), "5 L/100 km is 20 km/L");
    require(std::isinf(shown(Quantity::fuelConsumption, "mpg_us", 0.0)),
        "no consumption (a fuel cut) is an infinite economy, not a division by zero");
    require(shown(Quantity::fuelConsumption, "l_100km", 0.0) == 0.0, "no consumption is 0 L/100 km");
}

void checkGaugePressure() {
    auto preferences = with(Quantity::pressure, "bar");
    preferences.pressureReference = PressureReference::gauge;
    require(near(enginelab::units::toDisplay(preferences, Quantity::pressure, 201.325, 101.325), 1.0),
        "200 kPa absolute is 1 bar of boost");
    require(near(enginelab::units::toDisplay(preferences, Quantity::pressure, 101.325, 101.325), 0.0),
        "ambient is zero gauge");
    require(near(enginelab::units::toDisplay(preferences, Quantity::pressure, 90.0, 90.0), 0.0),
        "gauge is relative to the ambient passed in (altitude)");
    require(enginelab::units::symbol(preferences, Quantity::pressure) == "bar (g)", "a gauge symbol says so");
    preferences.pressureReference = PressureReference::absolute;
    require(enginelab::units::symbol(preferences, Quantity::pressure) == "bar", "an absolute symbol is plain");
    require(enginelab::units::symbol(preferences, Quantity::temperature) == "\xc2\xb0""C",
        "the gauge suffix is for pressure only");
}

void checkRoundTrips() {
    for (std::size_t q = 0; q < enginelab::units::quantityCount; ++q) {
        const auto quantity = static_cast<Quantity>(q);
        std::set<std::string> ids;
        const auto units = enginelab::units::unitsOf(quantity);
        require(!units.empty() && units[0].scale == 1.0 && units[0].offset == 0.0 && !units[0].reciprocal,
            "index 0 is the base unit");
        for (std::size_t u = 0; u < units.size(); ++u) {
            require(ids.insert(std::string(units[u].id)).second, "unit ids are unique per quantity");
            for (const auto reference : { PressureReference::absolute, PressureReference::gauge }) {
                UnitPreferences preferences;
                preferences.select(quantity, u);
                preferences.pressureReference = reference;
                for (const auto value : { 0.37, 12.5, 4'321.0 }) {
                    const auto display = enginelab::units::toDisplay(preferences, quantity, value, 97.0);
                    const auto back = enginelab::units::fromDisplay(preferences, quantity, display, 97.0);
                    require(near(back, value, 1.0e-12), "display then back returns the base value");
                }
            }
        }
    }
}

void checkPreferencesAndPresets() {
    UnitPreferences preferences;
    for (std::size_t q = 0; q < enginelab::units::quantityCount; ++q)
        require(preferences.of(static_cast<Quantity>(q)).scale == 1.0, "the default is the base unit everywhere");
    require(!preferences.select(Quantity::speed, "furlong_fortnight"), "an unknown id is refused");
    require(preferences == UnitPreferences {}, "a refused id changes nothing");
    preferences.select(Quantity::speed, std::size_t { 99 });
    require(preferences == UnitPreferences {}, "an out-of-range index changes nothing");

    const auto presets = enginelab::units::presets();
    require(presets.size() == 4, "four presets");
    require(presets[0].name == "Metric", "the first preset is metric");
    const auto metric = presets[0].preferences;
    require(metric.of(Quantity::speed).id == "km_h" && metric.of(Quantity::pressure).id == "kpa"
            && metric.of(Quantity::power).id == "kw" && metric.pressureReference == PressureReference::absolute,
        "metric keeps the simulator's own readouts (km/h, absolute kPa, kW)");
    const auto us = presets[2].preferences;
    require(us.of(Quantity::speed).id == "mph" && us.of(Quantity::fuelConsumption).id == "mpg_us"
            && us.of(Quantity::torque).id == "lb_ft" && us.of(Quantity::temperature).id == "f"
            && us.of(Quantity::displacement).id == "cu_in" && us.of(Quantity::airFlow).id == "lb_min",
        "the US preset");
    const auto uk = presets[3].preferences;
    require(uk.of(Quantity::speed).id == "mph" && uk.of(Quantity::fuelConsumption).id == "mpg_uk"
            && uk.of(Quantity::temperature).id == "c" && uk.of(Quantity::fuelVolume).id == "l",
        "the UK preset: miles and imperial MPG, but Celsius and litres at the pump");
}
} // namespace

int main() {
    checkKnownValues();
    checkFuelEconomy();
    checkGaugePressure();
    checkRoundTrips();
    checkPreferencesAndPresets();
    std::cout << "Units tests passed\n";
    return EXIT_SUCCESS;
}
