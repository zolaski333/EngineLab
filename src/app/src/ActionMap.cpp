#include <enginelab/app/ActionMap.hpp>

#include <juce_core/juce_core.h>
#include <algorithm>

namespace enginelab {
namespace {
[[nodiscard]] juce::KeyPress letter(juce::juce_wchar value) {
    return { static_cast<int>(value), {}, value };
}
}

ActionMap::ActionMap()
    : entries_ {{
        { "ignition", letter('A') }, { "starter", letter('S') }, { "dyno", letter('D') },
        { "dyno_hold", letter('H') }, { "fullscreen", letter('F') }, { "dyno_stats", letter('I') },
        { "pause", letter('P') }, { "next_screen", juce::KeyPress(juce::KeyPress::tabKey) },
        { "shift_up", juce::KeyPress(juce::KeyPress::upKey) },
        { "shift_down", juce::KeyPress(juce::KeyPress::downKey) },
        { "throttle_idle", letter('Q') }, { "throttle_quarter", letter('W') },
        { "throttle_half", letter('E') }, { "throttle_full", letter('R') },
        { "time_quarter", letter('1') }, { "time_half", letter('2') },
        { "time_normal", letter('3') }, { "time_double", letter('4') },
        { "time_quadruple", letter('5') }, { "layer_up", letter('M') },
        { "layer_down", letter(',') }, { "exhaust_preset", letter('Y') },
        { "clutch_decrease", letter('T') }, { "clutch_increase", letter('U') },
        { "wheel_dyno_rpm", letter('G') }, { "wheel_volume", letter('Z') },
        { "wheel_convolution", letter('X') }, { "wheel_high_gain", letter('C') },
        { "wheel_low_noise", letter('V') }, { "wheel_high_noise", letter('B') },
        { "wheel_combustion", letter('J') }, { "wheel_exhaust", letter('K') },
        { "wheel_intake", letter('L') }, { "wheel_mechanical", letter('O') },
        { "wheel_simulation_rate", letter('N') },
        { "wheel_fine_throttle", juce::KeyPress(juce::KeyPress::spaceKey) }
    }} {
    load();
}

bool ActionMap::matches(AppAction action, const juce::KeyPress& key) const noexcept {
    const auto& binding = entries_[static_cast<std::size_t>(action)].key;
    return key.getKeyCode() == binding.getKeyCode()
        && key.getModifiers().withoutMouseButtons() == binding.getModifiers().withoutMouseButtons();
}

bool ActionMap::isDown(AppAction action) const noexcept {
    const auto& binding = entries_[static_cast<std::size_t>(action)].key;
    const auto required = binding.getModifiers();
    const auto current = juce::ModifierKeys::getCurrentModifiersRealtime();
    return juce::KeyPress::isKeyCurrentlyDown(binding.getKeyCode())
        && (!required.isShiftDown() || current.isShiftDown())
        && (!required.isCtrlDown() || current.isCtrlDown())
        && (!required.isAltDown() || current.isAltDown())
        && (!required.isCommandDown() || current.isCommandDown());
}

juce::String ActionMap::toJson() const {
    auto* root = new juce::DynamicObject;
    for (const auto& entry : entries_) root->setProperty(entry.id, entry.key.getTextDescription());
    return juce::JSON::toString(juce::var(root), true);
}

bool ActionMap::fromJson(const juce::String& json, juce::String& error) {
    const auto parsed = juce::JSON::parse(json);
    const auto* object = parsed.getDynamicObject();
    if (object == nullptr) { error = "Le document doit être un objet JSON."; return false; }
    const auto& properties = object->getProperties();
    for (int propertyIndex = 0; propertyIndex < properties.size(); ++propertyIndex) {
        const auto propertyName = properties.getName(propertyIndex).toString();
        const auto known = std::any_of(entries_.begin(), entries_.end(), [&propertyName](const Entry& entry) {
            return propertyName == entry.id;
        });
        if (!known) { error = "Action inconnue : " + propertyName; return false; }
    }
    auto candidate = entries_;
    for (auto& entry : candidate) {
        const auto value = object->getProperty(entry.id);
        if (value.isVoid()) continue;
        if (!value.isString()) { error = juce::String(entry.id) + " doit être une chaîne."; return false; }
        const auto key = juce::KeyPress::createFromDescription(value.toString());
        if (!key.isValid()) { error = "Raccourci invalide pour " + juce::String(entry.id); return false; }
        if (key.getKeyCode() == juce::KeyPress::escapeKey
            || key.getKeyCode() == juce::KeyPress::returnKey
            || (key.getKeyCode() >= juce::KeyPress::F1Key && key.getKeyCode() <= juce::KeyPress::F12Key)) {
            error = "Raccourci réservé pour " + juce::String(entry.id);
            return false;
        }
        entry.key = key;
    }
    for (std::size_t left = 0; left < candidate.size(); ++left) {
        for (std::size_t right = left + 1; right < candidate.size(); ++right) {
            if (candidate[left].key == candidate[right].key) {
                error = "Conflit entre " + juce::String(candidate[left].id) + " et " + juce::String(candidate[right].id);
                return false;
            }
        }
    }
    entries_ = std::move(candidate);
    error.clear();
    return true;
}

juce::File ActionMap::settingsFile() {
    return juce::File::getSpecialLocation(juce::File::userApplicationDataDirectory)
        .getChildFile("EngineLab").getChildFile("keybindings.json");
}

void ActionMap::load() {
    const auto file = settingsFile();
    if (!file.existsAsFile()) { save(); return; }
    juce::String error;
    (void)fromJson(file.loadFileAsString(), error);
}

void ActionMap::save() const {
    const auto file = settingsFile();
    (void)file.getParentDirectory().createDirectory();
    (void)file.replaceWithText(toJson());
}

juce::String ActionMap::shortcut(AppAction action) const {
    return entries_[static_cast<std::size_t>(action)].key.getTextDescription();
}

} // namespace enginelab
