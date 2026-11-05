#include <enginelab/app/FuelWindow.hpp>

#include <enginelab/app/FuelEditing.hpp>
#include <enginelab/app/Theme.hpp>
#include <enginelab/app/Widgets.hpp>

#include <cstddef>
#include <memory>
#include <string>
#include <utility>

namespace enginelab {
namespace {

using ui::utf8;

constexpr int margin = 20;
constexpr int rowHeight = 30;
constexpr int rowGap = 8;
constexpr int labelWidth = 170;

/** What the window shows of an engine: another engine, another fuel kind or
    another value means the fields are rebuilt. */
[[nodiscard]] bool sameFuel(const EngineConfig& left, const EngineConfig& right) {
    return left.name == right.name && left.fuel == right.fuel
        && left.fuelProperties.name == right.fuelProperties.name
        && fuel_editing::values(left) == fuel_editing::values(right);
}

} // namespace

class FuelWindow::Content final : public juce::Component {
public:
    Content(const EngineConfig& config, std::vector<FuelPreset> presets, ApplyCallback applyCallback)
        : presets_(std::move(presets)), apply_(std::move(applyCallback)) {
        setOpaque(true);
        presetBox_.setTooltip(utf8("A listed fuel (parts/fuels.yaml) fills the values below; Apply puts it in "
                                   "the tank."));
        presetBox_.onChange = [this] { presetChosen(); };
        addAndMakeVisible(presetBox_);
        presetLabel_.setText(utf8("Fuel"), juce::dontSendNotification);
        addAndMakeVisible(presetLabel_);
        for (auto* label : { &note_, &status_ }) {
            label->setJustificationType(juce::Justification::topLeft);
            label->setMinimumHorizontalScale(1.0F);
            addAndMakeVisible(*label);
        }
        note_.setColour(juce::Label::textColourId, ui::colours::muted);
        status_.setColour(juce::Label::textColourId, ui::colours::warn);
        applyButton_.setTooltip(utf8("Puts this fuel in the tank. The engine restarts; a new octane measures the "
                                     "knock-limited spark again (some seconds)."));
        applyButton_.onClick = [this] { applyEdits(); };
        revertButton_.setTooltip(utf8("Back to the fuel in the tank."));
        revertButton_.onClick = [this] { show(shown_); };
        addAndMakeVisible(applyButton_);
        addAndMakeVisible(revertButton_);
        show(config);
    }

    void setConfig(const EngineConfig& config) {
        // Unapplied edits of this engine stay; anything else shows the engine.
        const auto sameEngine = config.name == shown_.name && config.fuel == shown_.fuel;
        if (pending_ && sameEngine) {
            // Other edits made meanwhile are kept: Apply starts from this one.
            shown_ = config;
            return;
        }
        if (!sameFuel(config, shown_)) show(config);
    }

    void setStatus(const juce::String& status) {
        status_.setText(status, juce::dontSendNotification);
    }

    void setLocked(bool locked) {
        if (locked_ == locked) return;
        locked_ = locked;
        presetBox_.setEnabled(!locked);
        for (auto& row : rows_) row->slider.setEnabled(!locked);
        updateButtons();
        repaint();
    }

    void paint(juce::Graphics& g) override {
        g.fillAll(ui::colours::background);
        auto area = getLocalBounds().reduced(margin, 0);
        area.removeFromTop(14);
        ui::drawSectionLabel(g, area.removeFromTop(18).toFloat(), utf8("FUEL IN THE TANK"));
        g.setColour(ui::colours::text);
        g.setFont(ui::uiFont(16.0F, true));
        g.drawFittedText(juce::String::fromUTF8(shown_.name.c_str()), area.removeFromTop(26),
                         juce::Justification::centredLeft, 1, 1.0F);
        if (locked_) {
            g.setColour(ui::colours::warn);
            g.setFont(ui::uiFont(13.0F));
            g.drawFittedText(utf8("Stop the dyno run to change the fuel."), area.removeFromTop(18),
                             juce::Justification::centredLeft, 1, 1.0F);
        }
    }

    void resized() override {
        auto area = getLocalBounds().reduced(margin, 0);
        area.removeFromTop(14 + 18 + 26 + 18 + 8);
        auto presetRow = area.removeFromTop(rowHeight);
        presetLabel_.setBounds(presetRow.removeFromLeft(labelWidth));
        presetBox_.setBounds(presetRow);
        area.removeFromTop(rowGap + 6);
        for (auto& row : rows_) {
            auto line = area.removeFromTop(rowHeight);
            row->label.setBounds(line.removeFromLeft(labelWidth));
            row->slider.setBounds(line);
            area.removeFromTop(rowGap);
        }
        area.removeFromTop(6);
        auto buttons = area.removeFromTop(34);
        const auto applyWidth = applyButton_.idealWidth();
        applyButton_.setBounds(buttons.removeFromRight(applyWidth));
        buttons.removeFromRight(8);
        const auto revertWidth = revertButton_.idealWidth();
        revertButton_.setBounds(buttons.removeFromRight(revertWidth));
        area.removeFromTop(12);
        status_.setBounds(area.removeFromTop(40));
        note_.setBounds(area.removeFromTop(90));
    }

    [[nodiscard]] static int preferredHeight(bool diesel) {
        const auto fieldCount = static_cast<int>(fuel_editing::fields(diesel).size());
        return 14 + 18 + 26 + 18 + 8 + rowHeight + rowGap + 6 + fieldCount * (rowHeight + rowGap) + 6 + 34 + 12
            + 40 + 90 + 14;
    }

private:
    struct Row final {
        juce::Label label;
        juce::Slider slider { juce::Slider::IncDecButtons, juce::Slider::TextBoxLeft };
        /** Edited by hand: an untouched field keeps the fuel's exact value,
            not the slider's rounding of it. */
        bool touched { false };
    };

    /** Shows the engine's fuel and drops any unapplied edit. */
    void show(const EngineConfig& config) {
        shown_ = config;
        chosenPreset_ = -1;
        pending_ = false;
        const auto diesel = config.fuel == FuelType::diesel;
        offered_ = fuel_editing::presetsFor(presets_, diesel);
        presetBox_.clear(juce::dontSendNotification);
        auto selected = 0;
        for (std::size_t index = 0; index < offered_.size(); ++index) {
            presetBox_.addItem(juce::String::fromUTF8(offered_[index].properties.name.c_str()),
                               static_cast<int>(index) + 1);
            if (offered_[index].properties.name == config.fuelProperties.name) selected = static_cast<int>(index) + 1;
        }
        // The engine's own fuel when it is none of the listed ones.
        ownFuelId_ = static_cast<int>(offered_.size()) + 1;
        if (selected == 0) {
            presetBox_.addItem(juce::String::fromUTF8(config.fuelProperties.name.c_str()), ownFuelId_);
            selected = ownFuelId_;
        }
        presetBox_.setSelectedId(selected, juce::dontSendNotification);

        const auto fields = fuel_editing::fields(diesel);
        if (rows_.size() != fields.size() || diesel != diesel_) {
            for (auto& row : rows_) {
                removeChildComponent(&row->label);
                removeChildComponent(&row->slider);
            }
            rows_.clear();
            for (const auto& field : fields) {
                auto row = std::make_unique<Row>();
                row->label.setText(utf8(field.label), juce::dontSendNotification);
                row->label.setColour(juce::Label::textColourId, ui::colours::text);
                row->slider.setRange(field.minimum, field.maximum, field.fineStep);
                row->slider.setNumDecimalPlacesToDisplay(field.decimals);
                row->slider.setTextValueSuffix(utf8(field.unit));
                row->slider.setTextBoxStyle(juce::Slider::TextBoxLeft, false, 140, rowHeight);
                row->slider.setIncDecButtonsMode(juce::Slider::incDecButtonsDraggable_AutoDirection);
                row->slider.onValueChange = [this, raw = row.get()] {
                    if (loading_) return;
                    raw->touched = true;
                    valuesEdited();
                };
                row->slider.setEnabled(!locked_);
                addAndMakeVisible(row->label);
                addAndMakeVisible(row->slider);
                rows_.push_back(std::move(row));
            }
            diesel_ = diesel;
            if (auto* window = findParentComponentOfClass<juce::DocumentWindow>())
                window->setSize(window->getWidth(), preferredHeight(diesel));
            resized();
        }
        showValues(config);
        note_.setText(utf8(fuel_editing::note(diesel)), juce::dontSendNotification);
        updateButtons();
        repaint();
    }

    void showValues(const EngineConfig& config) {
        const auto values = fuel_editing::values(config);
        loading_ = true;
        for (std::size_t index = 0; index < rows_.size() && index < values.size(); ++index) {
            rows_[index]->slider.setValue(values[index], juce::dontSendNotification);
            rows_[index]->touched = false;
        }
        loading_ = false;
    }

    /** The running engine with the chosen listed fuel, before the fields' values. */
    [[nodiscard]] EngineConfig draft() const {
        auto result = shown_;
        if (chosenPreset_ >= 0) fuel_editing::applyPreset(result, offered_[static_cast<std::size_t>(chosenPreset_)]);
        return result;
    }

    void presetChosen() {
        const auto id = presetBox_.getSelectedId();
        chosenPreset_ = id >= 1 && id < ownFuelId_ ? id - 1 : -1;
        const auto chosen = draft();
        showValues(chosen);
        pending_ = !sameFuel(chosen, shown_);
        updateButtons();
    }

    void valuesEdited() {
        if (loading_) return;
        pending_ = true;
        updateButtons();
    }

    /** The fields' values on `base`: a field not edited keeps base's value. */
    [[nodiscard]] std::vector<double> fieldValues(const EngineConfig& base) const {
        auto values = fuel_editing::values(base);
        for (std::size_t index = 0; index < rows_.size() && index < values.size(); ++index)
            if (rows_[index]->touched) values[index] = rows_[index]->slider.getValue();
        return values;
    }

    void applyEdits() {
        if (!pending_ || locked_ || !apply_) return;
        auto edited = draft();
        if (const auto error = fuel_editing::apply(edited, fieldValues(edited)); !error.empty()) {
            setStatus(utf8(error.c_str()));
            return;
        }
        if (!apply_(edited)) return;
        // MainComponent hands the running engine back through setConfig;
        // until then, the edit shown is the one applied.
        show(edited);
    }

    void updateButtons() {
        applyButton_.setEnabled(pending_ && !locked_);
        revertButton_.setEnabled(pending_);
    }

    std::vector<FuelPreset> presets_;
    std::vector<FuelPreset> offered_;
    ApplyCallback apply_;
    EngineConfig shown_;
    /** The listed fuel chosen in the box and not applied yet; -1 for none. */
    int chosenPreset_ { -1 };
    bool pending_ { false };
    bool loading_ { false };
    bool locked_ { false };
    bool diesel_ { false };
    int ownFuelId_ { 1 };
    juce::Label presetLabel_;
    juce::ComboBox presetBox_;
    std::vector<std::unique_ptr<Row>> rows_;
    juce::Label note_;
    juce::Label status_;
    ui::ActionButton applyButton_ { "Apply", ui::ActionButton::Style::primary };
    ui::ActionButton revertButton_ { "Revert", ui::ActionButton::Style::standard };
};

FuelWindow::FuelWindow(const EngineConfig& config, std::vector<FuelPreset> presets, ApplyCallback applyCallback)
    : juce::DocumentWindow("EngineLab - Fuel", ui::colours::background, juce::DocumentWindow::closeButton, true) {
    content_ = new Content(config, std::move(presets), std::move(applyCallback));
    setContentOwned(content_, false);
    setUsingNativeTitleBar(true);
    setResizable(false, false);
    centreWithSize(520, Content::preferredHeight(config.fuel == FuelType::diesel));
}

FuelWindow::~FuelWindow() = default;

void FuelWindow::closeButtonPressed() { setVisible(false); }

void FuelWindow::setConfig(const EngineConfig& config) {
    if (content_ != nullptr) content_->setConfig(config);
}

void FuelWindow::setStatus(const juce::String& status) {
    if (content_ != nullptr) content_->setStatus(status);
}

void FuelWindow::setLocked(bool locked) {
    if (content_ != nullptr) content_->setLocked(locked);
}

} // namespace enginelab
