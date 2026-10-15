#include <enginelab/app/EcuTunerWindow.hpp>
#include <enginelab/calibration/EcuCalibrationKeys.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <variant>

namespace enginelab {
namespace {

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
    using calibration::CellOperation;
    const auto step = [this] {
        const auto* entry = draft_.find(selectedCalibrationId().toStdString());
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
    statusLabel_.setColour(juce::Label::textColourId, juce::Colour(0xff8fb7a8));
    descriptionLabel_.setColour(juce::Label::textColourId, juce::Colour(0xffa8b0ad));
    descriptionLabel_.setFont(juce::FontOptions(12.0F));
    selectionLabel_.setColour(juce::Label::textColourId, juce::Colour(0xff9eaaa6));
    selectionLabel_.setFont(juce::FontOptions(12.0F));
    const std::array<juce::Component*, 15> components {
        &mapSelector_, &loadButton_, &saveButton_, &refreshButton_, &statusLabel_, &descriptionLabel_,
        &stepDownButton_, &stepUpButton_, &percentDownButton_, &percentUpButton_, &smoothButton_,
        &interpolateButton_, &undoButton_, &redoButton_, &selectionLabel_
    };
    for (auto* component : components)
        addAndMakeVisible(*component);
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
        ? utf8("DYNO SESSION · REVISION ") + juce::String(baseRevision_)
            + utf8(" PINNED · EDITING LOCKED")
        : "REVISION " + juce::String(baseRevision_) + utf8("  ·  LIVE MODE"),
        juce::dontSendNotification);
}

juce::String EcuTunerComponent::selectedCalibrationId() const {
    const auto index = mapSelector_.getSelectedItemIndex();
    return index >= 0 && static_cast<std::size_t>(index) < mapIds_.size()
        ? utf8(mapIds_[static_cast<std::size_t>(index)]) : juce::String {};
}

void EcuTunerComponent::reloadFromSnapshot(bool preserveSelection) {
    const auto selected = preserveSelection ? selectedCalibrationId() : juce::String {};
    const auto snapshot = store_->snapshot();
    draft_ = calibration::makeDraft(*snapshot);
    baseRevision_ = snapshot->revision();
    mapSelector_.clear(juce::dontSendNotification);
    mapIds_.clear();
    int itemId = 1;
    for (const auto& [id, entry] : snapshot->entries()) {
        const auto& name = calibration::metadataOf(entry).displayName;
        mapSelector_.addItem(utf8(name.empty() || name == id ? id : name + "  (" + id + ")"), itemId++);
        mapIds_.push_back(id);
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
    statusLabel_.setText("REVISION " + juce::String(baseRevision_) + utf8("  ·  ")
        + (hotReloader_->watching() ? "HOT RELOAD: " + juce::String(hotReloader_->path().string())
                                    : juce::String("LIVE MODE")),
        juce::dontSendNotification);
    rebuildCells();
}

void EcuTunerComponent::rebuildCells() {
    rebuilding_ = true;
    xLabels_.clear();
    yLabels_.clear();
    valueEditors_.clear();
    committedTexts_.clear();
    xCount_ = 1;
    yCount_ = 1;

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
    descriptionLabel_.setText(utf8(metadata.displayName + "  ·  " + metadata.description
        + "  [" + std::string(calibration::toString(metadata.unit)) + "]"), juce::dontSendNotification);

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
            label->setColour(juce::Label::textColourId, juce::Colour(0xff9eaaa6));
            addAndMakeVisible(*label);
            xLabels_.push_back(std::move(label));
        }
    }
    if (yAxis != nullptr) {
        for (const auto breakpoint : yAxis->breakpoints) {
            auto label = std::make_unique<juce::Label>();
            label->setText(number(breakpoint, 2), juce::dontSendNotification);
            label->setJustificationType(juce::Justification::centredRight);
            label->setColour(juce::Label::textColourId, juce::Colour(0xff9eaaa6));
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
        const auto value = values != nullptr ? (*values)[index] : scalarValue;
        auto editor = std::make_unique<juce::TextEditor>();
        const auto text = number(value, metadata.displayPrecision);
        editor->setText(text, false);
        editor->setJustification(juce::Justification::centred);
        editor->setFont(juce::FontOptions(14.0F, juce::Font::bold));
        editor->setColour(juce::TextEditor::backgroundColourId, cellColour(value, minimum, maximum));
        editor->setColour(juce::TextEditor::textColourId, juce::Colours::white);
        editor->setColour(juce::TextEditor::highlightColourId, juce::Colour(0xffe89945));
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

void EcuTunerComponent::updateActiveCellHighlight() {
    for (auto& editor : valueEditors_)
        editor->setColour(juce::TextEditor::outlineColourId, juce::Colour(0xff46524e));
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
        valueEditors_[active]->setColour(juce::TextEditor::outlineColourId, juce::Colour(0xffffb34d));
}

void EcuTunerComponent::commitCell(std::size_t valueIndex) {
    if (rebuilding_ || sessionLocked_
        || valueIndex >= valueEditors_.size()) return;
    const auto text = valueEditors_[valueIndex]->getText().trim();
    if (valueIndex < committedTexts_.size() && text == committedTexts_[valueIndex]) return;
    double value = 0.0;
    if (!parseFinite(text, value)) {
        valueEditors_[valueIndex]->setText(committedTexts_[valueIndex], false);
        juce::AlertWindow::showMessageBoxAsync(juce::MessageBoxIconType::WarningIcon,
            "Invalid ECU value", "Enter a finite number.");
        return;
    }
    const auto id = selectedCalibrationId().toStdString();
    const auto* current = draft_.find(id);
    if (current == nullptr) return;
    auto edited = *current;
    auto changed = false;
    std::visit([&](auto& calibrationEntry) {
        using T = std::decay_t<decltype(calibrationEntry)>;
        if constexpr (std::is_same_v<T, calibration::ScalarCalibration>) {
            if (valueIndex == 0) { calibrationEntry.value = value; changed = true; }
        } else {
            if (valueIndex < calibrationEntry.values.size()) {
                calibrationEntry.values[valueIndex] = value;
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
    statusLabel_.setText("REVISION " + juce::String(baseRevision_) + utf8("  ·  APPLIED LIVE"),
                         juce::dontSendNotification);
    updateToolState();
    return true;
}

void EcuTunerComponent::applyTool(calibration::CellOperation operation, double amount) {
    if (sessionLocked_ || selection_.empty()) return;
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
    for (std::size_t index = 0; index < mapIds_.size(); ++index)
        if (mapIds_[index] == id && mapSelector_.getSelectedItemIndex() != static_cast<int>(index)) {
            mapSelector_.setSelectedItemIndex(static_cast<int>(index), juce::dontSendNotification);
            selection_.clear();
        }
    statusLabel_.setText("REVISION " + juce::String(baseRevision_)
        + utf8(undo ? "  ·  UNDONE" : "  ·  REDONE"), juce::dontSendNotification);
    rebuildCells();
}

void EcuTunerComponent::updateToolState() {
    const auto editable = !sessionLocked_ && !selection_.empty();
    const auto* entry = draft_.find(selectedCalibrationId().toStdString());
    const auto isGrid = entry != nullptr && !std::holds_alternative<calibration::ScalarCalibration>(*entry);
    for (auto* button : { &stepDownButton_, &stepUpButton_, &percentDownButton_, &percentUpButton_ })
        button->setEnabled(editable);
    smoothButton_.setEnabled(editable && isGrid);
    interpolateButton_.setEnabled(editable && isGrid && selection_.size() > 1);
    undoButton_.setEnabled(!sessionLocked_ && !undo_.empty());
    redoButton_.setEnabled(!sessionLocked_ && !redo_.empty());
    const auto step = entry != nullptr ? calibration::cellStep(calibration::metadataOf(*entry)) : 0.0;
    selectionLabel_.setText(selection_.empty() || entry == nullptr
            ? juce::String("Click a cell; shift-click selects a block, ctrl-click adds a cell")
            : juce::String(static_cast<int>(selection_.size())) + utf8(selection_.size() == 1 ? " cell" : " cells")
                + utf8("  ·  step ") + juce::String(step, std::max(0, calibration::metadataOf(*entry).displayPrecision - 1)),
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
    graphics.setColour(juce::Colour(0xff6fd3ff));
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
    juce::AlertWindow::showMessageBoxAsync(juce::MessageBoxIconType::WarningIcon, title, message);
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
                { {}, file.getFullPathName().toStdString() });
            if (result.published) {
                safe->hotReloader_->watch(file.getFullPathName().toStdString());
                safe->reloadFromSnapshot();
            } else safe->showIssues(result, "ECU load rejected");
        } else if (file != juce::File {}) {
            juce::AlertWindow::showMessageBoxAsync(juce::MessageBoxIconType::WarningIcon,
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
                juce::AlertWindow::showMessageBoxAsync(juce::MessageBoxIconType::WarningIcon,
                    "ECU save failed", "The file could not be written.");
            else {
                safe->hotReloader_->watch(file.getFullPathName().toStdString());
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
            juce::AlertWindow::showMessageBoxAsync(juce::MessageBoxIconType::WarningIcon,
                "ECU hot reload rejected", utf8(reload.error));
    } else if (store_->snapshot()->revision() != baseRevision_
               && std::none_of(valueEditors_.begin(), valueEditors_.end(), [](const auto& editor) {
                   return editor->hasKeyboardFocus(false);
               })) {
        reloadFromSnapshot();
    }
}

void EcuTunerComponent::paint(juce::Graphics& graphics) {
    graphics.fillAll(juce::Colour(0xff111817));
    graphics.setColour(juce::Colour(0xff273330));
    graphics.drawHorizontalLine(96, 12.0F, static_cast<float>(getWidth() - 12));
    graphics.setColour(juce::Colour(0xff77847f));
    graphics.setFont(11.0F);
    graphics.drawText("LOAD / RPM", 12, 76, 96, 18, juce::Justification::centredLeft);
    graphics.drawText("Operating point  " + juce::String(operatingRpm_, 0) + utf8(" rpm  ·  ")
        + juce::String(operatingLoad_ * 100.0, 1) + " %", getWidth() - 270, 76, 255, 18,
        juce::Justification::centredRight);
}

void EcuTunerComponent::resized() {
    auto top = getLocalBounds().reduced(12).removeFromTop(34);
    mapSelector_.setBounds(top.removeFromLeft(std::min(330, top.getWidth() / 2)));
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

    const auto availableWidth = std::max(100, getWidth() - 126);
    const auto cellWidth = std::clamp(availableWidth / static_cast<int>(std::max<std::size_t>(1, xCount_)), 58, 112);
    const auto cellHeight = std::clamp((getHeight() - 190)
        / static_cast<int>(std::max<std::size_t>(1, yCount_)), 34, 54);
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
    : juce::DocumentWindow(utf8("EngineLab · ECU tuner"), juce::Colour(0xff111817),
                           juce::DocumentWindow::closeButton, true) {
    tuner_ = new EcuTunerComponent(std::move(store));
    setContentOwned(tuner_, true);
    setUsingNativeTitleBar(true);
    setResizable(true, false);
    setResizeLimits(720, 450, 1'400, 900);
    centreWithSize(980, 620);
}

void EcuTunerWindow::closeButtonPressed() { setVisible(false); }

void EcuTunerWindow::setOperatingPoint(double rpm, double normalizedLoad) {
    if (tuner_ != nullptr) tuner_->setOperatingPoint(rpm, normalizedLoad);
}

void EcuTunerWindow::setSessionLocked(bool locked) {
    if (tuner_ != nullptr) tuner_->setSessionLocked(locked);
}

} // namespace enginelab
