#include <enginelab/app/EcuTunerWindow.hpp>
#include <enginelab/app/Theme.hpp>
#include <enginelab/calibration/EcuCalibrationKeys.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cctype>
#include <filesystem>
#include <limits>
#include <map>
#include <string>
#include <variant>

namespace enginelab {
namespace {

/** The calibration file as std::filesystem sees it: a narrow string is the
 *  ANSI code page on Windows, and an accented path was never watched (audit
 *  DR-05). */
[[nodiscard]] std::filesystem::path watchedPath(const juce::File& file) {
#if defined(_WIN32)
    return std::filesystem::path(file.getFullPathName().toWideCharPointer());
#else
    return std::filesystem::path(file.getFullPathName().toRawUTF8());
#endif
}

[[nodiscard]] juce::String utf8(std::string_view value) {
    return juce::String::fromUTF8(value.data(), static_cast<int>(value.size()));
}

[[nodiscard]] juce::String number(double value, int precision) {
    return juce::String(value, std::clamp(precision, 0, 9));
}

[[nodiscard]] juce::Colour cellColour(double value, double minimum, double maximum) {
    const auto span = std::max(1.0e-9, maximum - minimum);
    const auto amount = static_cast<float>(std::clamp((value - minimum) / span, 0.0, 1.0));
    return juce::Colour::fromHSV(0.58F - amount * 0.55F, 0.52F, 0.32F + amount * 0.23F, 1.0F);
}

[[nodiscard]] bool parseFinite(const juce::String& text, double& value) {
    try {
        std::size_t consumed = 0;
        const auto source = text.trim().toStdString();
        value = std::stod(source, &consumed);
        return consumed == source.size() && std::isfinite(value);
    } catch (...) {
        return false;
    }
}

} // namespace

EcuTunerComponent::EcuTunerComponent(
    std::shared_ptr<calibration::CalibrationStore> store)
    : store_(store ? std::move(store) : std::make_shared<calibration::CalibrationStore>()),
      hotReloader_(std::make_unique<calibration::CalibrationFileHotReloader>(*store_)) {
    setOpaque(true);
    mapSelector_.setTextWhenNothingSelected("Select a table");
    mapSelector_.onChange = [this] {
        selection_.clear();
        rebuildCells();
    };
    loadButton_.onClick = [this] { chooseCalibrationToLoad(); };
    saveButton_.onClick = [this] { chooseCalibrationToSave(); };
    refreshButton_.onClick = [this] { reloadFromSnapshot(); };
    vtecButton_.onClick = [this] { pressVtecButton(); };
    antiLagButton_.onClick = [this] { pressAntiLagButton(); };
    antiLagButton_.setTooltip("Anti-lag: with your foot off, the throttle held open, a late spark and a share of the "
                              "sparks cut, fuel kept: the manifold burns it and the turbo keeps spinning. All its "
                              "settings are on the selector's Anti-lag page.");
    vtecButton_.setTooltip("VTEC: the ECU's switch over the high cam. All its settings are on the selector's VTEC "
                           "page.");
    using calibration::CellOperation;
    const auto step = [this] {
        const auto* entry = draft_.find(toolEntryId());
        return entry != nullptr ? calibration::cellStep(calibration::metadataOf(*entry)) : 0.0;
    };
    stepDownButton_.onClick = [this, step] { applyTool(CellOperation::add, -step()); };
    stepUpButton_.onClick = [this, step] { applyTool(CellOperation::add, step()); };
    percentDownButton_.onClick = [this] { applyTool(CellOperation::scalePercent, -5.0); };
    percentUpButton_.onClick = [this] { applyTool(CellOperation::scalePercent, 5.0); };
    smoothButton_.onClick = [this] { applyTool(CellOperation::smooth, 0.0); };
    interpolateButton_.onClick = [this] { applyTool(CellOperation::interpolate, 0.0); };
    undoButton_.onClick = [this] { stepHistory(true); };
    redoButton_.onClick = [this] { stepHistory(false); };
    statusLabel_.setColour(juce::Label::textColourId, ui::colours::ok);
    descriptionLabel_.setColour(juce::Label::textColourId, ui::colours::muted);
    descriptionLabel_.setFont(juce::FontOptions(12.0F));
    selectionLabel_.setColour(juce::Label::textColourId, ui::colours::label);
    selectionLabel_.setFont(juce::FontOptions(12.0F));
    const std::array<juce::Component*, 15> components {
        &mapSelector_, &loadButton_, &saveButton_, &refreshButton_, &statusLabel_, &descriptionLabel_,
        &stepDownButton_, &stepUpButton_, &percentDownButton_, &percentUpButton_, &smoothButton_,
        &interpolateButton_, &undoButton_, &redoButton_, &selectionLabel_
    };
    for (auto* component : components)
        addAndMakeVisible(*component);
    addChildComponent(vtecButton_);
    addChildComponent(antiLagButton_);
    reloadFromSnapshot(false);
    startTimerHz(10);
}

EcuTunerComponent::~EcuTunerComponent() {
    stopTimer();
    rebuilding_ = true;
    for (auto& editor : valueEditors_) {
        editor->onReturnKey = nullptr;
        editor->onFocusLost = nullptr;
    }
    // Destroy editors while their callbacks' backing state still exists. JUCE
    // may emit focus-loss notifications during component teardown.
    valueEditors_.clear();
}

void EcuTunerComponent::setOperatingPoint(double rpm, double normalizedLoad) {
    const auto nextRpm = std::isfinite(rpm) ? std::max(0.0, rpm) : 0.0;
    const auto nextLoad = std::isfinite(normalizedLoad)
        ? std::clamp(normalizedLoad, calibration::ecuLimits::minimumNormalizedLoad,
                     calibration::ecuLimits::maximumNormalizedLoad)
        : 0.0;
    if (std::abs(nextRpm - operatingRpm_) < 1.0 && std::abs(nextLoad - operatingLoad_) < 0.002) return;
    operatingRpm_ = nextRpm;
    operatingLoad_ = nextLoad;
    updateActiveCellHighlight();
    repaint();
}

void EcuTunerComponent::setFuel(const std::string& name, double stoichiometricAirFuelRatio, bool diesel) {
    const auto scale = diesel || !(stoichiometricAirFuelRatio > 0.0)
        ? 1.0 : stoichiometricAirFuelRatio / calibration::ecuLimits::petrolStoichiometricAirFuelRatio;
    const auto fuelAfr = !diesel && scale > 0.0;
    if (scale == afrDisplayScale_ && name == fuelName_ && fuelAfr == fuelAfr_) return;
    afrDisplayScale_ = scale;
    fuelName_ = name;
    fuelAfr_ = fuelAfr;
    if (!rebuilding_) rebuildCells();
}

double EcuTunerComponent::displayScaleOf(const std::string& id) const noexcept {
    return fuelAfr_ && id == calibration::keys::targetAirFuelRatio ? afrDisplayScale_ : 1.0;
}

void EcuTunerComponent::setVtecFittable(bool fittable) {
    if (vtecFittable_ == fittable) return;
    vtecFittable_ = fittable;
    updateVtecButton();
    resized();
}

void EcuTunerComponent::updateVtecButton() {
    const auto* entry = draft_.find(calibration::keys::vtecEnabled);
    const auto* setting = entry != nullptr ? std::get_if<calibration::ScalarCalibration>(entry) : nullptr;
    const auto on = setting != nullptr && setting->value >= 0.5;
    vtecButton_.setButtonText(setting != nullptr ? (on ? "VTEC on" : "VTEC off") : "Fit VTEC cams");
    vtecButton_.setColour(juce::TextButton::buttonColourId,
                          on ? ui::colours::accentDeep : ui::colours::panelRaised);
    vtecButton_.setVisible(setting != nullptr || vtecFittable_);
    vtecButton_.setEnabled(!sessionLocked_);
}

void EcuTunerComponent::updateAntiLagButton() {
    const auto* entry = draft_.find(calibration::keys::antiLagEnabled);
    const auto* setting = entry != nullptr ? std::get_if<calibration::ScalarCalibration>(entry) : nullptr;
    const auto on = setting != nullptr && setting->value >= 0.5;
    antiLagButton_.setButtonText(on ? "Anti-lag on" : "Anti-lag off");
    antiLagButton_.setColour(juce::TextButton::buttonColourId,
                             on ? ui::colours::accentDeep : ui::colours::panelRaised);
    antiLagButton_.setEnabled(!sessionLocked_);
    if (antiLagButton_.isVisible() != (setting != nullptr)) {
        antiLagButton_.setVisible(setting != nullptr);
        resized();
    }
}

void EcuTunerComponent::pressAntiLagButton() {
    if (sessionLocked_) return;
    const auto* current = draft_.find(calibration::keys::antiLagEnabled);
    if (current == nullptr) return;
    auto edited = *current;
    auto* setting = std::get_if<calibration::ScalarCalibration>(&edited);
    if (setting == nullptr) return;
    setting->value = setting->value >= 0.5 ? 0.0 : 1.0;
    const auto previous = *current;
    if (publishEdit(std::move(edited), previous)) rebuildCells();
}

void EcuTunerComponent::pressVtecButton() {
    if (sessionLocked_) return;
    const auto* current = draft_.find(calibration::keys::vtecEnabled);
    if (current == nullptr) {
        if (vtecFittable_ && onFitVtecCams) onFitVtecCams();
        return;
    }
    auto edited = *current;
    auto* setting = std::get_if<calibration::ScalarCalibration>(&edited);
    if (setting == nullptr) return;
    setting->value = setting->value >= 0.5 ? 0.0 : 1.0;
    const auto previous = *current;
    if (publishEdit(std::move(edited), previous)) rebuildCells();
}

void EcuTunerComponent::setSessionLocked(bool locked) {
    if (sessionLocked_ == locked) return;
    sessionLocked_ = locked;
    loadButton_.setEnabled(!locked);
    refreshButton_.setEnabled(!locked);
    if (!locked) {
        reloadFromSnapshot();
        return;
    }
    for (auto& editor : valueEditors_)
        editor->setReadOnly(locked);
    updateToolState();
    statusLabel_.setText(locked
        ? utf8("Dyno session · revision ") + juce::String(baseRevision_)
            + utf8(" PINNED · EDITING LOCKED")
        : "Revision " + juce::String(baseRevision_) + utf8("  ·  live mode"),
        juce::dontSendNotification);
}

juce::String EcuTunerComponent::selectedCalibrationId() const {
    const auto index = mapSelector_.getSelectedItemIndex();
    return index >= 0 && static_cast<std::size_t>(index) < mapIds_.size()
        ? utf8(mapIds_[static_cast<std::size_t>(index)]) : juce::String {};
}

const EcuTunerComponent::Page* EcuTunerComponent::selectedPage() const {
    const auto index = mapSelector_.getSelectedItemIndex();
    return index >= 0 && static_cast<std::size_t>(index) < pages_.size()
        ? &pages_[static_cast<std::size_t>(index)] : nullptr;
}

std::string EcuTunerComponent::entryIdOfCell(std::size_t valueIndex) const {
    if (!cellIds_.empty()) return valueIndex < cellIds_.size() ? cellIds_[valueIndex] : std::string {};
    return selectedCalibrationId().toStdString();
}

std::string EcuTunerComponent::toolEntryId() const {
    if (cellIds_.empty()) return selectedCalibrationId().toStdString();
    return selection_.empty() ? std::string {} : entryIdOfCell(selection_.front());
}

void EcuTunerComponent::selectPageOf(const std::string& entryId) {
    for (std::size_t index = 0; index < pages_.size(); ++index) {
        const auto& members = pages_[index].members;
        if (std::find(members.begin(), members.end(), entryId) == members.end()) continue;
        if (mapSelector_.getSelectedItemIndex() != static_cast<int>(index)) {
            mapSelector_.setSelectedItemIndex(static_cast<int>(index), juce::dontSendNotification);
            selection_.clear();
        }
        return;
    }
}

std::vector<EcuTunerComponent::Page> EcuTunerComponent::pagesOf(const calibration::CalibrationSnapshot& snapshot) {
    // A single-value setting shares its page with every other one of its
    // prefix (the id before its first dot) when there are two or more.
    const auto prefixOf = [](const std::string& id) {
        const auto dot = id.find('.');
        return dot == std::string::npos ? std::string {} : id.substr(0, dot);
    };
    std::map<std::string, std::size_t> scalarsPerPrefix;
    for (const auto& [id, entry] : snapshot.entries())
        if (std::holds_alternative<calibration::ScalarCalibration>(entry) && !prefixOf(id).empty())
            ++scalarsPerPrefix[prefixOf(id)];
    const auto groupTitle = [](const std::string& prefix) -> std::string {
        if (prefix == "antilag") return "Anti-lag";
        if (prefix == "vtec") return "VTEC";
        if (prefix == "limits") return "Limits";
        auto title = prefix;
        if (!title.empty()) title[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(title[0])));
        return title;
    };
    std::vector<Page> pages;
    std::map<std::string, std::size_t> groupPage;
    for (const auto& [id, entry] : snapshot.entries()) {
        const auto prefix = prefixOf(id);
        const auto grouped = std::holds_alternative<calibration::ScalarCalibration>(entry) && !prefix.empty()
            && scalarsPerPrefix[prefix] >= 2;
        if (!grouped) {
            const auto& name = calibration::metadataOf(entry).displayName;
            pages.push_back({ id, name.empty() || name == id ? id : name, { id } });
            continue;
        }
        if (const auto found = groupPage.find(prefix); found != groupPage.end()) {
            pages[found->second].members.push_back(id);
            continue;
        }
        groupPage.emplace(prefix, pages.size());
        pages.push_back({ prefix + ".*", groupTitle(prefix) + " settings", { id } });
    }
    // A group's switch ("enabled") reads first.
    for (auto& page : pages)
        if (page.group())
            std::stable_partition(page.members.begin(), page.members.end(),
                                  [](const std::string& id) { return id.ends_with(".enabled"); });
    return pages;
}

void EcuTunerComponent::reloadFromSnapshot(bool preserveSelection) {
    const auto selected = preserveSelection ? selectedCalibrationId() : juce::String {};
    const auto snapshot = store_->snapshot();
    draft_ = calibration::makeDraft(*snapshot);
    baseRevision_ = snapshot->revision();
    mapSelector_.clear(juce::dontSendNotification);
    mapIds_.clear();
    pages_ = pagesOf(*snapshot);
    int itemId = 1;
    for (const auto& page : pages_) {
        mapSelector_.addItem(utf8(page.title == page.id ? page.id : page.title + "  (" + page.id + ")"), itemId++);
        mapIds_.push_back(page.id);
    }
    if (selected.isNotEmpty()) {
        for (std::size_t index = 0; index < mapIds_.size(); ++index) {
            if (utf8(mapIds_[index]) == selected) {
                mapSelector_.setSelectedItemIndex(static_cast<int>(index), juce::dontSendNotification);
                break;
            }
        }
    }
    if (mapSelector_.getSelectedItemIndex() < 0 && mapSelector_.getNumItems() > 0) {
        mapSelector_.setSelectedItemIndex(0, juce::dontSendNotification);
        selection_.clear();
    }
    statusLabel_.setText("Revision " + juce::String(baseRevision_) + utf8("  ·  ")
        + (hotReloader_->watching() ? "hot reload: " + juce::String(hotReloader_->path().string())
                                    : juce::String("live mode")),
        juce::dontSendNotification);
    rebuildCells();
}

void EcuTunerComponent::rebuildCells() {
    rebuilding_ = true;
    xLabels_.clear();
    yLabels_.clear();
    rowLabels_.clear();
    cellIds_.clear();
    valueEditors_.clear();
    committedTexts_.clear();
    xCount_ = 1;
    yCount_ = 1;

    if (const auto* page = selectedPage(); page != nullptr && page->group()) {
        rebuildGroupPage(*page);
        return;
    }
    const auto id = selectedCalibrationId().toStdString();
    const auto* entry = draft_.find(id);
    if (entry == nullptr) {
        descriptionLabel_.setText("No table available.", juce::dontSendNotification);
        rebuilding_ = false;
        resized();
        repaint();
        return;
    }
    const auto& metadata = calibration::metadataOf(*entry);
    const auto scale = displayScaleOf(id);
    // A spark engine's AFR map is stored in petrol AFR; it reads in the
    // fuel's own AFR, the number the dyno and the gauges show.
    const auto fuelAfr = fuelAfr_ && id == calibration::keys::targetAirFuelRatio;
    const auto unitText = fuelAfr
        ? "AFR of " + fuelName_ + ", lambda x " + juce::String(scale * calibration::ecuLimits::petrolStoichiometricAirFuelRatio, 2).toStdString()
        : std::string(calibration::toString(metadata.unit));
    const auto description = fuelAfr
        ? std::string("Commanded mixture by engine speed and load, in the fuel's own AFR (stored as petrol AFR: "
                      "another fuel is metered at the same lambda).")
        : metadata.description;
    descriptionLabel_.setText(utf8(metadata.displayName + "  ·  " + description
        + "  [" + unitText + "]"), juce::dontSendNotification);

    const std::vector<double>* values = nullptr;
    const calibration::CalibrationAxis* xAxis = nullptr;
    const calibration::CalibrationAxis* yAxis = nullptr;
    double scalarValue = 0.0;
    if (const auto* scalar = std::get_if<calibration::ScalarCalibration>(entry)) {
        scalarValue = scalar->value;
    } else if (const auto* curve = std::get_if<calibration::CalibrationCurve1D>(entry)) {
        values = &curve->values;
        xAxis = &curve->axis;
        xCount_ = curve->axis.breakpoints.size();
    } else if (const auto* table = std::get_if<calibration::CalibrationTable2D>(entry)) {
        values = &table->values;
        xAxis = &table->xAxis;
        yAxis = &table->yAxis;
        xCount_ = table->xAxis.breakpoints.size();
        yCount_ = table->yAxis.breakpoints.size();
    }

    if (xAxis != nullptr) {
        for (const auto breakpoint : xAxis->breakpoints) {
            auto label = std::make_unique<juce::Label>();
            label->setText(number(breakpoint, 0), juce::dontSendNotification);
            label->setJustificationType(juce::Justification::centred);
            label->setColour(juce::Label::textColourId, ui::colours::label);
            addAndMakeVisible(*label);
            xLabels_.push_back(std::move(label));
        }
    }
    if (yAxis != nullptr) {
        for (const auto breakpoint : yAxis->breakpoints) {
            auto label = std::make_unique<juce::Label>();
            label->setText(number(breakpoint, 2), juce::dontSendNotification);
            label->setJustificationType(juce::Justification::centredRight);
            label->setColour(juce::Label::textColourId, ui::colours::label);
            addAndMakeVisible(*label);
            yLabels_.push_back(std::move(label));
        }
    }

    const auto count = values != nullptr ? values->size() : 1U;
    const auto minimum = values != nullptr && !values->empty()
        ? *std::min_element(values->begin(), values->end()) : scalarValue;
    const auto maximum = values != nullptr && !values->empty()
        ? *std::max_element(values->begin(), values->end()) : scalarValue;
    valueEditors_.reserve(count);
    committedTexts_.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
        const auto value = (values != nullptr ? (*values)[index] : scalarValue) * scale;
        auto editor = std::make_unique<juce::TextEditor>();
        const auto text = number(value, metadata.displayPrecision);
        editor->setText(text, false);
        editor->setJustification(juce::Justification::centred);
        editor->setFont(juce::FontOptions(14.0F, juce::Font::bold));
        editor->setColour(juce::TextEditor::backgroundColourId, cellColour(value, minimum * scale, maximum * scale));
        editor->setColour(juce::TextEditor::textColourId, juce::Colours::white);
        editor->setColour(juce::TextEditor::highlightColourId, ui::colours::accentDeep);
        editor->onReturnKey = [this, index] { commitCell(index); };
        editor->onFocusLost = [this, index] { commitCell(index); };
        editor->addMouseListener(this, true);
        addAndMakeVisible(*editor);
        committedTexts_.push_back(text);
        valueEditors_.push_back(std::move(editor));
    }

    updateActiveCellHighlight();
    for (auto& editor : valueEditors_)
        editor->setReadOnly(sessionLocked_);
    std::erase_if(selection_, [count](std::size_t index) { return index >= count; });
    updateToolState();
    rebuilding_ = false;
    resized();
    repaint();
}

void EcuTunerComponent::rebuildGroupPage(const Page& page) {
    // One row per setting: its name and unit, its value, its description
    // as the row's tooltip. A switch reads 1 on, 0 off.
    descriptionLabel_.setText(utf8(page.title + "  ·  every single-value setting of " + page.id
        + " on one page; type a value and press Enter, it is taken at once."), juce::dontSendNotification);
    for (const auto& member : page.members) {
        const auto* entry = draft_.find(member);
        const auto* scalar = entry != nullptr ? std::get_if<calibration::ScalarCalibration>(entry) : nullptr;
        if (scalar == nullptr) continue;
        const auto& metadata = calibration::metadataOf(*entry);
        const auto unit = std::string(calibration::toString(metadata.unit));
        auto label = std::make_unique<juce::Label>();
        label->setText(utf8((metadata.displayName.empty() ? member : metadata.displayName)
            + (unit.empty() ? std::string {} : "  [" + unit + "]")), juce::dontSendNotification);
        label->setJustificationType(juce::Justification::centredLeft);
        label->setColour(juce::Label::textColourId, ui::colours::text);
        label->setTooltip(utf8(member + ": " + metadata.description));
        addAndMakeVisible(*label);
        rowLabels_.push_back(std::move(label));
        const auto text = number(scalar->value * displayScaleOf(member), metadata.displayPrecision);
        auto editor = std::make_unique<juce::TextEditor>();
        editor->setText(text, false);
        editor->setJustification(juce::Justification::centred);
        editor->setFont(juce::FontOptions(14.0F, juce::Font::bold));
        editor->setColour(juce::TextEditor::backgroundColourId, ui::colours::well);
        editor->setColour(juce::TextEditor::textColourId, juce::Colours::white);
        editor->setColour(juce::TextEditor::highlightColourId, ui::colours::accentDeep);
        editor->setColour(juce::TextEditor::outlineColourId, ui::colours::lineHover);
        editor->setTooltip(utf8(member + ": " + metadata.description));
        const auto index = valueEditors_.size();
        editor->onReturnKey = [this, index] { commitCell(index); };
        editor->onFocusLost = [this, index] { commitCell(index); };
        editor->addMouseListener(this, true);
        editor->setReadOnly(sessionLocked_);
        addAndMakeVisible(*editor);
        committedTexts_.push_back(text);
        valueEditors_.push_back(std::move(editor));
        cellIds_.push_back(member);
    }
    xCount_ = 1;
    yCount_ = valueEditors_.size();
    const auto count = valueEditors_.size();
    std::erase_if(selection_, [count](std::size_t index) { return index >= count; });
    updateToolState();
    rebuilding_ = false;
    resized();
    repaint();
}

void EcuTunerComponent::updateActiveCellHighlight() {
    for (auto& editor : valueEditors_)
        editor->setColour(juce::TextEditor::outlineColourId, ui::colours::lineHover);
    const auto* entry = draft_.find(selectedCalibrationId().toStdString());
    const auto* table = entry != nullptr
        ? std::get_if<calibration::CalibrationTable2D>(entry) : nullptr;
    if (table == nullptr || table->xAxis.breakpoints.empty() || table->yAxis.breakpoints.empty()) return;
    const auto nearest = [](const std::vector<double>& axis, double value) {
        auto best = std::size_t { 0 };
        auto distance = std::numeric_limits<double>::infinity();
        for (std::size_t index = 0; index < axis.size(); ++index) {
            const auto candidate = std::abs(axis[index] - value);
            if (candidate < distance) { distance = candidate; best = index; }
        }
        return best;
    };
    const auto x = nearest(table->xAxis.breakpoints, operatingRpm_);
    const auto y = nearest(table->yAxis.breakpoints, operatingLoad_);
    const auto active = y * xCount_ + x;
    if (active < valueEditors_.size())
        valueEditors_[active]->setColour(juce::TextEditor::outlineColourId, ui::colours::accent);
}

void EcuTunerComponent::commitCell(std::size_t valueIndex) {
    if (rebuilding_ || sessionLocked_
        || valueIndex >= valueEditors_.size()) return;
    const auto text = valueEditors_[valueIndex]->getText().trim();
    if (valueIndex < committedTexts_.size() && text == committedTexts_[valueIndex]) return;
    double value = 0.0;
    if (!parseFinite(text, value)) {
        valueEditors_[valueIndex]->setText(committedTexts_[valueIndex], false);
        ui::showMessage(this, juce::MessageBoxIconType::WarningIcon,
            "Invalid ECU value", "Enter a finite number.");
        return;
    }
    const auto id = entryIdOfCell(valueIndex);
    const auto* current = draft_.find(id);
    if (current == nullptr) return;
    // A group page's row is its setting's only value.
    const auto entryIndex = cellIds_.empty() ? valueIndex : std::size_t { 0 };
    // Typed in the fuel's own AFR, stored in petrol AFR.
    value /= displayScaleOf(id);
    auto edited = *current;
    auto changed = false;
    std::visit([&](auto& calibrationEntry) {
        using T = std::decay_t<decltype(calibrationEntry)>;
        if constexpr (std::is_same_v<T, calibration::ScalarCalibration>) {
            if (entryIndex == 0) { calibrationEntry.value = value; changed = true; }
        } else {
            if (entryIndex < calibrationEntry.values.size()) {
                calibrationEntry.values[entryIndex] = value;
                changed = true;
            }
        }
    }, edited);
    if (!changed) return;
    const auto previous = *current;
    if (publishEdit(std::move(edited), previous)) committedTexts_[valueIndex] = text;
}

bool EcuTunerComponent::publishEdit(calibration::CalibrationEntry edited,
                                    const calibration::CalibrationEntry& previous) {
    draft_.set(std::move(edited));
    const auto result = store_->publish(draft_, { baseRevision_, "ecu-tuner" });
    if (!result.published) {
        showIssues(result, "Calibration rejected");
        reloadFromSnapshot();
        return false;
    }
    baseRevision_ = result.activeRevision;
    undo_.push_back(previous);
    redo_.clear();
    if (undo_.size() > 200) undo_.erase(undo_.begin());
    statusLabel_.setText("Revision " + juce::String(baseRevision_) + utf8("  ·  applied live"),
                         juce::dontSendNotification);
    updateToolState();
    return true;
}

void EcuTunerComponent::applyTool(calibration::CellOperation operation, double amount) {
    if (sessionLocked_ || selection_.empty()) return;
    if (!cellIds_.empty()) {
        // A group page: the tool acts on each selected row's setting.
        auto any = false;
        for (const auto index : selection_) {
            const auto* row = draft_.find(entryIdOfCell(index));
            if (row == nullptr) continue;
            auto edited = *row;
            if (!calibration::applyCellOperation(edited, { 0 }, operation, amount)) continue;
            const auto previous = *row;
            if (!publishEdit(std::move(edited), previous)) return;
            any = true;
        }
        if (any) rebuildCells();
        return;
    }
    const auto* current = draft_.find(selectedCalibrationId().toStdString());
    if (current == nullptr) return;
    auto edited = *current;
    if (!calibration::applyCellOperation(edited, selection_, operation, amount)) return;
    const auto previous = *current;
    if (publishEdit(std::move(edited), previous)) rebuildCells();
}

void EcuTunerComponent::stepHistory(bool undo) {
    auto& from = undo ? undo_ : redo_;
    auto& to = undo ? redo_ : undo_;
    if (sessionLocked_ || from.empty()) return;
    auto restored = std::move(from.back());
    from.pop_back();
    const auto id = calibration::metadataOf(restored).id;
    const auto* current = draft_.find(id);
    if (current == nullptr) {
        // The engine changed under the history: it no longer applies.
        undo_.clear();
        redo_.clear();
        updateToolState();
        return;
    }
    to.push_back(*current);
    draft_.set(std::move(restored));
    const auto result = store_->publish(draft_, { baseRevision_, "ecu-tuner" });
    if (!result.published) {
        undo_.clear();
        redo_.clear();
        showIssues(result, "Calibration rejected");
        reloadFromSnapshot();
        return;
    }
    baseRevision_ = result.activeRevision;
    selectPageOf(id);
    statusLabel_.setText("Revision " + juce::String(baseRevision_)
        + utf8(undo ? "  ·  UNDONE" : "  ·  REDONE"), juce::dontSendNotification);
    rebuildCells();
}

void EcuTunerComponent::updateToolState() {
    const auto editable = !sessionLocked_ && !selection_.empty();
    const auto* entry = draft_.find(toolEntryId());
    const auto isGrid = entry != nullptr && !std::holds_alternative<calibration::ScalarCalibration>(*entry);
    for (auto* button : { &stepDownButton_, &stepUpButton_, &percentDownButton_, &percentUpButton_ })
        button->setEnabled(editable);
    smoothButton_.setEnabled(editable && isGrid);
    interpolateButton_.setEnabled(editable && isGrid && selection_.size() > 1);
    undoButton_.setEnabled(!sessionLocked_ && !undo_.empty());
    redoButton_.setEnabled(!sessionLocked_ && !redo_.empty());
    updateVtecButton();
    updateAntiLagButton();
    const auto step = entry != nullptr ? calibration::cellStep(calibration::metadataOf(*entry)) : 0.0;
    selectionLabel_.setText(selection_.empty() || entry == nullptr
            ? juce::String("Click a cell; shift-click selects a block, ctrl-click adds a cell")
            : juce::String(static_cast<int>(selection_.size()))
                + utf8(selection_.size() == 1 ? (cellIds_.empty() ? " cell" : " setting")
                                              : (cellIds_.empty() ? " cells" : " settings"))
                + utf8("  ·  step ") + juce::String(step * displayScaleOf(calibration::metadataOf(*entry).id),
                    std::max(0, calibration::metadataOf(*entry).displayPrecision - 1)),
        juce::dontSendNotification);
    repaint();
}

void EcuTunerComponent::mouseDown(const juce::MouseEvent& event) {
    // The click lands on the editor's inner text holder: walk up to the cell.
    const auto found = std::find_if(valueEditors_.begin(), valueEditors_.end(), [&](const auto& editor) {
        for (auto* component = event.eventComponent; component != nullptr && component != this;
             component = component->getParentComponent())
            if (component == editor.get()) return true;
        return false;
    });
    if (found == valueEditors_.end()) return;
    const auto index = static_cast<std::size_t>(std::distance(valueEditors_.begin(), found));
    if (event.mods.isShiftDown() && !selection_.empty()) {
        selection_ = calibration::cellRectangle(selectionAnchor_, index, std::max<std::size_t>(1, xCount_));
    } else if (event.mods.isCtrlDown() || event.mods.isCommandDown()) {
        const auto existing = std::find(selection_.begin(), selection_.end(), index);
        if (existing != selection_.end()) selection_.erase(existing);
        else selection_.push_back(index);
        selectionAnchor_ = index;
    } else {
        selection_ = { index };
        selectionAnchor_ = index;
    }
    updateToolState();
}

void EcuTunerComponent::paintOverChildren(juce::Graphics& graphics) {
    graphics.setColour(ui::colours::intake);
    for (const auto index : selection_)
        if (index < valueEditors_.size())
            graphics.drawRect(valueEditors_[index]->getBounds().expanded(2), 2);
}

void EcuTunerComponent::showIssues(const calibration::PublishResult& result,
                                   const juce::String& title) {
    juce::String message;
    for (const auto& issue : result.issues) {
        if (message.isNotEmpty()) message << "\n";
        message << utf8(issue.path) << ": " << utf8(issue.message);
    }
    if (message.isEmpty()) message = "The transaction was not published.";
    ui::showMessage(this, juce::MessageBoxIconType::WarningIcon, title, message);
}

void EcuTunerComponent::setTableCompletion(calibration::DraftCompletion complete) {
    completeTables_ = complete;
    hotReloader_->setCompletion(std::move(complete));
}

void EcuTunerComponent::chooseCalibrationToLoad() {
    if (sessionLocked_) return;
    fileChooser_ = std::make_unique<juce::FileChooser>("Load an ECU calibration",
                                                       juce::File {}, "*.ecu.json;*.json");
    auto safe = juce::Component::SafePointer<EcuTunerComponent>(this);
    fileChooser_->launchAsync(juce::FileBrowserComponent::openMode
        | juce::FileBrowserComponent::canSelectFiles, [safe](const juce::FileChooser& chooser) {
        if (!safe) return;
        const auto file = chooser.getResult();
        if (file.existsAsFile() && file.getSize() <= 2 * 1024 * 1024) {
            const auto text = file.loadFileAsString();
            const auto json = text.toStdString();
            const auto result = calibration::publishJson(*safe->store_,
                json,
                { {}, file.getFullPathName().toStdString() }, safe->completeTables_);
            if (result.published) {
                safe->hotReloader_->watch(watchedPath(file));
                safe->reloadFromSnapshot();
            } else safe->showIssues(result, "ECU load rejected");
        } else if (file != juce::File {}) {
            ui::showMessage(safe.getComponent(), juce::MessageBoxIconType::WarningIcon,
                "ECU load failed", "File missing or larger than 2 MiB.");
        }
        safe->fileChooser_.reset();
    });
}

void EcuTunerComponent::chooseCalibrationToSave() {
    fileChooser_ = std::make_unique<juce::FileChooser>("Save the ECU calibration",
        juce::File::getSpecialLocation(juce::File::userDocumentsDirectory)
            .getChildFile("EngineLab.ecu.json"), "*.ecu.json;*.json");
    auto safe = juce::Component::SafePointer<EcuTunerComponent>(this);
    fileChooser_->launchAsync(juce::FileBrowserComponent::saveMode
        | juce::FileBrowserComponent::warnAboutOverwriting, [safe](const juce::FileChooser& chooser) {
        if (!safe) return;
        auto file = chooser.getResult();
        if (file != juce::File {}) {
            if (!file.hasFileExtension("json")) file = file.withFileExtension("ecu.json");
            const auto encoded = calibration::CalibrationJson::serialize(*safe->store_->snapshot());
            if (!file.replaceWithText(juce::String::fromUTF8(encoded.data(), static_cast<int>(encoded.size()))))
                ui::showMessage(safe.getComponent(), juce::MessageBoxIconType::WarningIcon,
                    "ECU save failed", "The file could not be written.");
            else {
                safe->hotReloader_->watch(watchedPath(file));
                safe->reloadFromSnapshot();
            }
        }
        safe->fileChooser_.reset();
    });
}

void EcuTunerComponent::timerCallback() {
    if (sessionLocked_) return;
    const auto reload = hotReloader_->poll();
    if (reload.changed) {
        if (reload.published()) reloadFromSnapshot();
        else if (!reload.error.empty())
            ui::showMessage(this, juce::MessageBoxIconType::WarningIcon,
                "ECU hot reload rejected", utf8(reload.error));
    } else if (store_->snapshot()->revision() != baseRevision_
               && std::none_of(valueEditors_.begin(), valueEditors_.end(), [](const auto& editor) {
                   return editor->hasKeyboardFocus(false);
               })) {
        reloadFromSnapshot();
    }
}

void EcuTunerComponent::paint(juce::Graphics& graphics) {
    graphics.fillAll(ui::colours::panel);
    graphics.setColour(ui::colours::line);
    graphics.drawHorizontalLine(96, 12.0F, static_cast<float>(getWidth() - 12));
    graphics.setColour(ui::colours::label);
    graphics.setFont(11.0F);
    if (cellIds_.empty()) graphics.drawText("Load / rpm", 12, 76, 96, 18, juce::Justification::centredLeft);
    graphics.drawText("Operating point  " + juce::String(operatingRpm_, 0) + utf8(" rpm  ·  ")
        + juce::String(operatingLoad_ * 100.0, 1) + " %", getWidth() - 270, 76, 255, 18,
        juce::Justification::centredRight);
}

void EcuTunerComponent::resized() {
    auto top = getLocalBounds().reduced(12).removeFromTop(34);
    if (vtecButton_.isVisible()) {
        vtecButton_.setBounds(top.removeFromRight(124));
        top.removeFromRight(8);
    }
    if (antiLagButton_.isVisible()) {
        antiLagButton_.setBounds(top.removeFromRight(124));
        top.removeFromRight(8);
    }
    // The buttons after the selector take 329 px.
    mapSelector_.setBounds(top.removeFromLeft(std::clamp(top.getWidth() - 329, 150, 330)));
    top.removeFromLeft(8);
    loadButton_.setBounds(top.removeFromLeft(92)); top.removeFromLeft(6);
    saveButton_.setBounds(top.removeFromLeft(112)); top.removeFromLeft(6);
    refreshButton_.setBounds(top.removeFromLeft(105));
    descriptionLabel_.setBounds(12, 50, getWidth() - 24, 22);
    statusLabel_.setBounds(12, getHeight() - 34, getWidth() - 24, 22);
    auto tools = juce::Rectangle<int>(12, getHeight() - 68, getWidth() - 24, 28);
    for (auto* button : { &stepDownButton_, &stepUpButton_, &percentDownButton_, &percentUpButton_ }) {
        button->setBounds(tools.removeFromLeft(button == &stepDownButton_ || button == &stepUpButton_ ? 38 : 56));
        tools.removeFromLeft(4);
    }
    tools.removeFromLeft(8);
    smoothButton_.setBounds(tools.removeFromLeft(80)); tools.removeFromLeft(4);
    interpolateButton_.setBounds(tools.removeFromLeft(108)); tools.removeFromLeft(12);
    undoButton_.setBounds(tools.removeFromLeft(64)); tools.removeFromLeft(4);
    redoButton_.setBounds(tools.removeFromLeft(64)); tools.removeFromLeft(12);
    selectionLabel_.setBounds(tools);

    if (!cellIds_.empty()) {
        // A group page: one row per setting, its name then its value, between
        // the description and the tool row. Rows that do not fit at 30 px go
        // on in a second column, and only then get shorter, so no value runs
        // under the tools (audit UA-02).
        constexpr int pageTop = 108;
        const auto bottom = getHeight() - 76;
        const auto available = std::max(1, bottom - pageTop);
        const auto count = static_cast<int>(std::max<std::size_t>(1, valueEditors_.size()));
        const auto columns = count * 30 > available && getWidth() >= 680 ? 2 : 1;
        const auto perColumn = (count + columns - 1) / columns;
        const auto rowHeight = std::clamp(available / perColumn, 22, 40);
        const auto columnWidth = (getWidth() - 24 - (columns - 1) * 16) / columns;
        const auto labelWidth = std::clamp(columnWidth - 148, 120, 460);
        for (std::size_t row = 0; row < valueEditors_.size(); ++row) {
            const auto column = static_cast<int>(row) / perColumn;
            const auto x = 12 + column * (columnWidth + 16);
            const auto y = pageTop + (static_cast<int>(row) % perColumn) * rowHeight;
            if (row < rowLabels_.size()) rowLabels_[row]->setBounds(x, y, labelWidth, rowHeight);
            valueEditors_[row]->setBounds(x + labelWidth + 8, y + 2, 140, rowHeight - 4);
        }
        return;
    }
    const auto availableWidth = std::max(100, getWidth() - 126);
    const auto cellWidth = std::clamp(availableWidth / static_cast<int>(std::max<std::size_t>(1, xCount_)), 58, 112);
    // Between the axis labels and the tool row: the ten load rows fit the
    // window's minimum height.
    const auto cellHeight = std::clamp((getHeight() - 116 - 76)
        / static_cast<int>(std::max<std::size_t>(1, yCount_)), 28, 54);
    const auto gridWidth = cellWidth * static_cast<int>(xCount_);
    const auto originX = 108 + std::max(0, (availableWidth - gridWidth) / 2);
    const auto originY = 116;
    for (std::size_t x = 0; x < xLabels_.size(); ++x)
        xLabels_[x]->setBounds(originX + static_cast<int>(x) * cellWidth, 92, cellWidth, 22);
    for (std::size_t y = 0; y < yLabels_.size(); ++y)
        yLabels_[y]->setBounds(12, originY + static_cast<int>(y) * cellHeight, 88, cellHeight);
    for (std::size_t index = 0; index < valueEditors_.size(); ++index) {
        const auto x = index % std::max<std::size_t>(1, xCount_);
        const auto y = index / std::max<std::size_t>(1, xCount_);
        valueEditors_[index]->setBounds(originX + static_cast<int>(x) * cellWidth + 2,
            originY + static_cast<int>(y) * cellHeight + 2, cellWidth - 4, cellHeight - 4);
    }
}

EcuTunerWindow::EcuTunerWindow(std::shared_ptr<calibration::CalibrationStore> store)
    : juce::DocumentWindow(utf8("EngineLab · ECU tuner"), ui::colours::panel,
                           juce::DocumentWindow::closeButton, true) {
    tuner_ = new EcuTunerComponent(std::move(store));
    setContentOwned(tuner_, true);
    setUsingNativeTitleBar(true);
    setResizable(true, false);
    setResizeLimits(720, 520, 1'400, 900);
    centreWithSize(980, 620);
}

void EcuTunerWindow::closeButtonPressed() { setVisible(false); }

void EcuTunerWindow::setOperatingPoint(double rpm, double normalizedLoad) {
    if (tuner_ != nullptr) tuner_->setOperatingPoint(rpm, normalizedLoad);
}

void EcuTunerWindow::setSessionLocked(bool locked) {
    if (tuner_ != nullptr) tuner_->setSessionLocked(locked);
}

void EcuTunerWindow::setFuel(const std::string& name, double stoichiometricAirFuelRatio, bool diesel) {
    if (tuner_ != nullptr) tuner_->setFuel(name, stoichiometricAirFuelRatio, diesel);
}

void EcuTunerWindow::setVtecFittable(bool fittable) {
    if (tuner_ != nullptr) tuner_->setVtecFittable(fittable);
}

} // namespace enginelab
