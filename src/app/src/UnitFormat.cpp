#include <enginelab/app/UnitFormat.hpp>

#include <cmath>

namespace enginelab::ui {
namespace {
juce::String fromView(std::string_view text) {
    return juce::String::fromUTF8(text.data(), static_cast<int>(text.size()));
}
} // namespace

double UnitFormat::display(units::Quantity quantity, double baseValue) const noexcept {
    return units::toDisplay(preferences, quantity, baseValue, ambientPressureKpa);
}

double UnitFormat::base(units::Quantity quantity, double displayValue) const noexcept {
    return units::fromDisplay(preferences, quantity, displayValue, ambientPressureKpa);
}

juce::String UnitFormat::value(units::Quantity quantity, double baseValue, int decimals) const {
    const auto shown = display(quantity, baseValue);
    if (!std::isfinite(shown)) return juce::String::fromUTF8("\xe2\x80\x94");
    const auto places = decimals >= 0 ? decimals : preferences.of(quantity).decimals;
    // "-0" reads as a fault on a gauge: a value that rounds to zero is zero.
    const auto scale = std::pow(10.0, places);
    const auto rounded = std::round(shown * scale) / scale;
    return juce::String(rounded == 0.0 ? 0.0 : rounded, places);
}

juce::String UnitFormat::symbol(units::Quantity quantity) const {
    return fromView(units::symbol(preferences, quantity));
}

juce::String UnitFormat::text(units::Quantity quantity, double baseValue, int decimals) const {
    return value(quantity, baseValue, decimals) + " " + symbol(quantity);
}

juce::String UnitFormat::pressureDifference(double kilopascals, int decimals) const {
    auto absolute = *this;
    absolute.preferences.pressureReference = units::PressureReference::absolute;
    return absolute.text(units::Quantity::pressure, kilopascals, decimals);
}

juce::File UnitFormat::settingsFile() {
    return juce::File::getSpecialLocation(juce::File::userApplicationDataDirectory)
        .getChildFile("EngineLab").getChildFile("units.json");
}

juce::String UnitFormat::toJson() const {
    auto* object = new juce::DynamicObject();
    for (std::size_t index = 0; index < units::quantityCount; ++index) {
        const auto quantity = static_cast<units::Quantity>(index);
        object->setProperty(juce::Identifier(fromView(units::quantityId(quantity))),
                            fromView(preferences.of(quantity).id));
    }
    object->setProperty("pressure_reference",
        preferences.pressureReference == units::PressureReference::gauge ? "gauge" : "absolute");
    return juce::JSON::toString(juce::var(object));
}

void UnitFormat::fromJson(const juce::String& json) {
    const auto parsed = juce::JSON::parse(json);
    auto* object = parsed.getDynamicObject();
    if (object == nullptr) return;
    for (std::size_t index = 0; index < units::quantityCount; ++index) {
        const auto quantity = static_cast<units::Quantity>(index);
        const auto key = juce::Identifier(fromView(units::quantityId(quantity)));
        if (object->hasProperty(key))
            (void)preferences.select(quantity, object->getProperty(key).toString().toStdString());
    }
    const auto reference = object->getProperty("pressure_reference").toString();
    if (reference == "gauge") preferences.pressureReference = units::PressureReference::gauge;
    else if (reference == "absolute") preferences.pressureReference = units::PressureReference::absolute;
}

void UnitFormat::load() {
    const auto file = settingsFile();
    if (file.existsAsFile()) fromJson(file.loadFileAsString());
}

void UnitFormat::save() const {
    const auto file = settingsFile();
    (void)file.getParentDirectory().createDirectory();
    (void)file.replaceWithText(toJson());
}

void UnitFormat::addMenu(juce::PopupMenu& menu, std::function<void()> changed) {
    const auto apply = [this, changed](auto edit) {
        return [this, changed, edit] {
            edit(preferences);
            save();
            if (changed) changed();
        };
    };
    for (const auto& preset : units::presets()) {
        juce::PopupMenu::Item item(fromView(preset.name));
        item.isTicked = preset.preferences == preferences;
        const auto chosen = preset.preferences;
        item.action = apply([chosen](units::UnitPreferences& target) { target = chosen; });
        menu.addItem(std::move(item));
    }
    menu.addSeparator();
    for (std::size_t index = 0; index < units::quantityCount; ++index) {
        const auto quantity = static_cast<units::Quantity>(index);
        juce::PopupMenu submenu;
        const auto choices = units::unitsOf(quantity);
        for (std::size_t unit = 0; unit < choices.size(); ++unit) {
            juce::PopupMenu::Item item(fromView(choices[unit].symbol));
            item.isTicked = &preferences.of(quantity) == &choices[unit];
            item.action = apply([quantity, unit](units::UnitPreferences& target) { target.select(quantity, unit); });
            submenu.addItem(std::move(item));
        }
        if (quantity == units::Quantity::pressure) {
            submenu.addSeparator();
            const auto gauge = preferences.pressureReference == units::PressureReference::gauge;
            juce::PopupMenu::Item absolute(juce::String("Absolute"));
            absolute.isTicked = !gauge;
            absolute.action = apply([](units::UnitPreferences& target) {
                target.pressureReference = units::PressureReference::absolute;
            });
            submenu.addItem(std::move(absolute));
            juce::PopupMenu::Item relative(juce::String("Gauge (relative to ambient air)"));
            relative.isTicked = gauge;
            relative.action = apply([](units::UnitPreferences& target) {
                target.pressureReference = units::PressureReference::gauge;
            });
            submenu.addItem(std::move(relative));
        }
        menu.addSubMenu(fromView(units::quantityLabel(quantity)) + ": " + symbol(quantity), submenu);
    }
}

} // namespace enginelab::ui
