#include <enginelab/app/EngineDesignerWindow.hpp>
#include <enginelab/app/Theme.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/foundation/CrankBalance.hpp>
#include <enginelab/runtime/RealtimeCapacityProbe.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <exception>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace enginelab {
namespace {

using ui::utf8;

constexpr std::array<EngineUse, 6> uses { EngineUse::roadCar, EngineUse::sportCar, EngineUse::roadMotorcycle,
    EngineUse::sportMotorcycle, EngineUse::cruiserMotorcycle, EngineUse::aircraft };
constexpr std::array<EngineLayout, 5> layouts { EngineLayout::inlineLayout, EngineLayout::vLayout,
    EngineLayout::flat, EngineLayout::radial, EngineLayout::custom };

/** The numbers of a text field, separated by spaces, commas or dashes. */
[[nodiscard]] std::optional<std::vector<double>> numbersOf(const juce::String& text) {
    std::vector<double> numbers;
    auto tokens = juce::StringArray::fromTokens(text.replaceCharacters(",;-", "   "), " ", "");
    tokens.removeEmptyStrings();
    for (const auto& token : tokens) {
        if (!token.containsOnly("0123456789.")) return std::nullopt;
        numbers.push_back(token.getDoubleValue());
    }
    return numbers;
}

/** A positive number, zero for an empty field, none for anything else. */
[[nodiscard]] std::optional<double> numberOf(const juce::String& text) {
    const auto trimmed = text.trim();
    if (trimmed.isEmpty()) return 0.0;
    if (!trimmed.containsOnly("0123456789.")) return std::nullopt;
    return trimmed.getDoubleValue();
}

[[nodiscard]] juce::String shown(double value) {
    const auto magnitude = std::abs(value);
    return ui::fixed(value, magnitude >= 100.0 ? 0 : magnitude >= 10.0 ? 1 : magnitude >= 1.0 ? 2 : 3);
}

} // namespace

class EngineDesignerWindow::DesignerContent final : public juce::Component,
                                                    private juce::TableListBoxModel,
                                                    private juce::Timer {
public:
    DesignerContent(std::filesystem::path catalogRoot, BuildCallback buildCallback, PauseCallback pauseLive,
                    std::function<void()> close)
        : catalogRoot_(std::move(catalogRoot)), buildCallback_(std::move(buildCallback)),
          pauseLive_(std::move(pauseLive)), close_(std::move(close)) {
        setOpaque(true);
        try {
            parts_ = loadPartsLibrary(catalogRoot_);
        } catch (const std::exception& error) {
            partsError_ = error.what();
        }

        addField(name_, "Name", "Inline Seven");
        addChoice(cycle_, "Cycle", { "Four-stroke", "Two-stroke" });
        addChoice(fuel_, "Fuel", { "Petrol", "Diesel" });
        addChoice(use_, "Built for",
            { "Road car", "Sport car", "Road motorcycle", "Sport motorcycle", "Cruiser motorcycle", "Aircraft" });
        addChoice(layout_, "Layout", { "Inline", "V", "Flat (boxer)", "Radial", "W / several banks" });
        addField(cylinders_, "Cylinders", "7");
        addField(bankAngle_, "Bank angle (deg)", "");
        addField(bankCount_, "Banks", "");
        addField(bore_, "Bore (mm)", "84");
        addField(stroke_, "Stroke (mm)", "72");
        addField(displacement_, "or displacement (L)", "");
        addChoice(induction_, "Induction", { "Natural", "Turbocharged", "Supercharged" });
        addField(boost_, "Boost (absolute ratio)", "");
        addChoice(turbos_, "Turbochargers", { "1", "2", "3", "4" });
        addField(firingOrder_, "Firing order", "");
        addChoice(crankPins_, "Crank pins", { "Automatic", "One per row (shared)", "One per cylinder" });
        addField(crankOffsets_, "Crank offsets (deg)", "");
        addField(bankAngles_, "W bank angles (deg)", "");
        firingOrder_.editor.setTooltip(utf8("Cylinder ids in firing order, e.g. 1 3 4 2. Empty: the best balanced."));
        crankOffsets_.editor.setTooltip(
            utf8("Each cylinder's top dead centre in crank degrees, in id order, e.g. 0 270 for a 270° twin."));
        crankPins_.combo.setTooltip(utf8("A V's two cylinders of a row share one pin, as on real engines (a 90° "
            "twin fires 270-450). Automatic splits the pins when sharing cannot fire six or more evenly."));
        bankAngles_.editor.setTooltip(utf8("Each bank's angle from vertical, e.g. -52.5 -37.5 37.5 52.5."));

        table_.setModel(this);
        table_.setRowHeight(24);
        table_.getHeader().addColumn("Value", 1, 210);
        table_.getHeader().addColumn("", 2, 90);
        table_.getHeader().addColumn("Set by", 3, 70);
        table_.getHeader().addColumn("Rule", 4, 420);
        table_.getHeader().setStretchToFitActive(true);
        table_.setColour(juce::ListBox::backgroundColourId, ui::colours::panel);
        addAndMakeVisible(table_);

        overrideLabel_.setColour(juce::Label::textColourId, ui::colours::muted);
        addChildComponent(overrideLabel_);
        addChildComponent(overrideEditor_);
        overrideSet_.onClick = [this] { setOverride(); };
        overrideReset_.onClick = [this] { resetOverride(); };
        overrideEditor_.onReturnKey = [this] { setOverride(); };
        addChildComponent(overrideSet_);
        addChildComponent(overrideReset_);

        status_.setJustificationType(juce::Justification::topLeft);
        status_.setColour(juce::Label::textColourId, ui::colours::text);
        addAndMakeVisible(status_);
        advanced_.setClickingTogglesState(true);
        advanced_.onClick = [this] { showMode(); };
        build_.onClick = [this] { build(); };
        cancel_.onClick = [this] { cancel(); };
        addAndMakeVisible(advanced_);
        addAndMakeVisible(build_);
        addAndMakeVisible(cancel_);
        showMode();
        redesign();
    }

    ~DesignerContent() override {
        stopTimer();
        stopProbe();
    }

    /** Closes the window; a real-time check under way is abandoned. */
    void cancel() {
        stopProbe();
        build_.setEnabled(designed_.config.has_value());
        close_();
    }

    void paint(juce::Graphics& graphics) override {
        graphics.fillAll(ui::colours::background);
        graphics.setColour(ui::colours::text);
        graphics.setFont(ui::uiFont(18.0F, true));
        graphics.drawText(utf8("NEW ENGINE"), 16, 10, 300, 28, juce::Justification::centredLeft);
        graphics.setColour(ui::colours::muted);
        graphics.setFont(ui::uiFont(13.0F));
        graphics.drawText(utf8("From nothing: say what the engine is, every other value follows from rules fitted to "
                               "real engines. Advanced sets any of them by hand."),
            16, 36, getWidth() - 32, 20, juce::Justification::centredLeft);
    }

    void resized() override {
        auto bounds = getLocalBounds().reduced(16);
        bounds.removeFromTop(52);
        auto buttons = bounds.removeFromBottom(34);
        cancel_.setBounds(buttons.removeFromRight(110));
        buttons.removeFromRight(8);
        build_.setBounds(buttons.removeFromRight(170));
        advanced_.setBounds(buttons.removeFromLeft(130));
        bounds.removeFromBottom(8);
        status_.setBounds(bounds.removeFromBottom(64));
        bounds.removeFromBottom(8);

        auto form = bounds.removeFromLeft(360);
        bounds.removeFromLeft(16);
        for (auto* field : visibleFields()) {
            auto row = form.removeFromTop(30);
            field->label.setBounds(row.removeFromLeft(160));
            field->control().setBounds(row.reduced(0, 2));
            form.removeFromTop(4);
        }
        if (advanced_.getToggleState()) {
            auto edit = bounds.removeFromBottom(30);
            overrideLabel_.setBounds(edit.removeFromLeft(260));
            overrideReset_.setBounds(edit.removeFromRight(90));
            edit.removeFromRight(6);
            overrideSet_.setBounds(edit.removeFromRight(90));
            edit.removeFromRight(6);
            overrideEditor_.setBounds(edit);
            bounds.removeFromBottom(6);
        }
        table_.setBounds(bounds);
    }

private:
    struct Field final {
        juce::Label label;
        juce::TextEditor editor;
        juce::ComboBox combo;
        bool isCombo { false };
        bool advancedOnly { false };
        juce::Component& control() { return isCombo ? static_cast<juce::Component&>(combo) : editor; }
    };

    void addField(Field& field, const char* label, const char* text) {
        field.label.setText(utf8(label), juce::dontSendNotification);
        field.label.setColour(juce::Label::textColourId, ui::colours::muted);
        field.editor.setText(utf8(text), false);
        field.editor.onTextChange = [this] { startTimer(250); };
        addAndMakeVisible(field.label);
        addAndMakeVisible(field.editor);
        fields_.push_back(&field);
    }

    void addChoice(Field& field, const char* label, std::initializer_list<const char*> options) {
        field.isCombo = true;
        field.label.setText(utf8(label), juce::dontSendNotification);
        field.label.setColour(juce::Label::textColourId, ui::colours::muted);
        auto id = 1;
        for (const auto* option : options) field.combo.addItem(utf8(option), id++);
        field.combo.setSelectedItemIndex(0, juce::dontSendNotification);
        field.combo.onChange = [this] {
            showMode();
            redesign();
        };
        addAndMakeVisible(field.label);
        addAndMakeVisible(field.combo);
        fields_.push_back(&field);
    }

    [[nodiscard]] std::vector<Field*> visibleFields() {
        std::vector<Field*> visible;
        for (auto* field : fields_)
            if (field->label.isVisible()) visible.push_back(field);
        return visible;
    }

    void showMode() {
        const auto advanced = advanced_.getToggleState();
        const auto layout = layouts[static_cast<std::size_t>(std::max(0, layout_.combo.getSelectedItemIndex()))];
        const auto show = [](Field& field, bool visible) {
            field.label.setVisible(visible);
            field.control().setVisible(visible);
        };
        for (auto* field : fields_) show(*field, true);
        show(bankAngle_, layout == EngineLayout::vLayout || layout == EngineLayout::custom);
        show(bankCount_, layout == EngineLayout::custom);
        const auto boosted = induction_.combo.getSelectedItemIndex() > 0;
        show(boost_, boosted);
        show(turbos_, induction_.combo.getSelectedItemIndex() == 1);
        show(firingOrder_, advanced && layout != EngineLayout::radial);
        show(crankPins_, advanced && (layout == EngineLayout::vLayout || layout == EngineLayout::custom));
        show(crankOffsets_, advanced);
        show(bankAngles_, advanced && layout == EngineLayout::custom);
        for (auto* component : std::initializer_list<juce::Component*> {
                 &overrideLabel_, &overrideEditor_, &overrideSet_, &overrideReset_ })
            component->setVisible(advanced);
        if (!advanced) overrides_.clear();
        resized();
        repaint();
        if (!advanced) redesign();
    }

    void timerCallback() override {
        stopTimer();
        redesign();
    }

    /** The spec the fields describe, or why they do not describe one. */
    [[nodiscard]] std::optional<EngineSpec> spec(juce::String& error) const {
        EngineSpec spec;
        spec.name = name_.editor.getText().trim().toStdString();
        spec.cycle = cycle_.combo.getSelectedItemIndex() == 1 ? EngineCycle::twoStroke : EngineCycle::fourStroke;
        spec.fuel = fuel_.combo.getSelectedItemIndex() == 1 ? FuelType::diesel : FuelType::gasoline;
        spec.use = uses[static_cast<std::size_t>(std::max(0, use_.combo.getSelectedItemIndex()))];
        spec.layout = layouts[static_cast<std::size_t>(std::max(0, layout_.combo.getSelectedItemIndex()))];
        spec.induction = static_cast<Induction>(std::max(0, induction_.combo.getSelectedItemIndex()));
        spec.turbochargerCount = static_cast<std::uint32_t>(std::max(0, turbos_.combo.getSelectedItemIndex()) + 1);
        const auto number = [&error](const Field& field, double& target) {
            if (!field.label.isVisible()) return true;
            const auto value = numberOf(field.editor.getText());
            if (!value) {
                error = field.label.getText() + utf8(" must be a positive number.");
                return false;
            }
            target = *value;
            return true;
        };
        auto cylinders = 0.0;
        auto banks = 0.0;
        if (!number(cylinders_, cylinders) || !number(bankAngle_, spec.bankAngleDegrees) || !number(bankCount_, banks)
            || !number(bore_, spec.boreMm) || !number(stroke_, spec.strokeMm)
            || !number(displacement_, spec.displacementLitres) || !number(boost_, spec.boostPressureRatio))
            return std::nullopt;
        if (cylinders != std::round(cylinders) || banks != std::round(banks)) {
            error = utf8("Cylinders and banks are whole numbers.");
            return std::nullopt;
        }
        spec.cylinderCount = static_cast<std::uint32_t>(cylinders);
        spec.bankCount = static_cast<std::uint32_t>(banks);
        if (spec.displacementLitres > 0.0 && spec.boreMm > 0.0 && spec.strokeMm > 0.0) {
            error = utf8("Give either bore and stroke or the displacement, not both.");
            return std::nullopt;
        }
        if (firingOrder_.label.isVisible()) {
            const auto order = numbersOf(firingOrder_.editor.getText());
            if (!order) {
                error = utf8("The firing order is a list of cylinder ids.");
                return std::nullopt;
            }
            for (const auto id : *order) spec.firingOrder.push_back(static_cast<std::uint32_t>(id));
        }
        if (crankPins_.label.isVisible())
            spec.crankPins = static_cast<CrankPins>(std::max(0, crankPins_.combo.getSelectedItemIndex()));
        if (crankOffsets_.label.isVisible()) {
            const auto offsets = numbersOf(crankOffsets_.editor.getText());
            if (!offsets) {
                error = utf8("The crank offsets are a list of degrees.");
                return std::nullopt;
            }
            spec.crankOffsetsDegrees = *offsets;
        }
        if (bankAngles_.label.isVisible()) {
            // Signed angles: read them as they are written.
            auto tokens = juce::StringArray::fromTokens(bankAngles_.editor.getText().replaceCharacters(",;", "  "),
                " ", "");
            tokens.removeEmptyStrings();
            for (const auto& token : tokens) {
                if (!token.containsOnly("-+0123456789.")) {
                    error = utf8("The bank angles are a list of degrees.");
                    return std::nullopt;
                }
                spec.bankAnglesDegrees.push_back(token.getDoubleValue());
            }
        }
        spec.overrides = overrides_;
        return spec;
    }

    void redesign() {
        designed_ = {};
        juce::String error;
        if (!partsError_.empty()) error = utf8("The parts library did not load: ") + utf8(partsError_.c_str());
        else if (const auto described = spec(error)) designed_ = designEngine(*described, parts_);
        if (error.isEmpty() && !designed_.config) error = utf8(designed_.error.c_str());
        build_.setEnabled(designed_.config.has_value() && !probing_);
        if (designed_.config) {
            const auto& config = *designed_.config;
            const auto balance = analyseCrankBalance(config);
            juce::String order;
            for (const auto id : config.firingOrder) order << (order.isEmpty() ? "" : "-") << static_cast<int>(id);
            juce::String intervals;
            auto even = true;
            for (const auto gap : balance.firingIntervalsDegrees) {
                even = even && std::abs(gap - balance.firingIntervalsDegrees.front()) < 0.5;
                intervals << (intervals.isEmpty() ? "" : "/") << juce::roundToInt(gap);
            }
            const auto power = std::find_if(designed_.values.begin(), designed_.values.end(),
                [](const DesignValue& value) { return value.key == "estimated_power_kw"; });
            status_.setColour(juce::Label::textColourId, ui::colours::text);
            status_.setText(ui::fixed(engineDisplacementLitres(config), 2) + utf8(" L, ")
                    + juce::String(config.cylinders.size()) + utf8(" cylinders, redline ")
                    + juce::String(juce::roundToInt(config.redlineRpm)) + utf8(" rpm, about ")
                    + (power != designed_.values.end() ? juce::String(juce::roundToInt(power->value)) : juce::String("?"))
                    + utf8(" kW (estimate).\nFires ") + order + utf8(even ? " evenly" : ", intervals ") + (even ? juce::String() : intervals)
                    + utf8(". Free primary force ") + ui::fixed(balance.primaryForce, 2) + utf8(", secondary ")
                    + ui::fixed(balance.secondaryForce, 2) + utf8("; couples ") + ui::fixed(balance.primaryCouple, 2)
                    + utf8(" / ") + ui::fixed(balance.secondaryCouple, 2)
                    + utf8(" (one cylinder's m r w², x pitch). The ignition is a template, not yet calibrated."),
                juce::dontSendNotification);
        } else {
            status_.setColour(juce::Label::textColourId, ui::colours::critical);
            status_.setText(error, juce::dontSendNotification);
        }
        table_.updateContent();
        table_.repaint();
        showSelected();
    }

    int getNumRows() override { return static_cast<int>(designed_.values.size()); }

    void paintRowBackground(juce::Graphics& graphics, int row, int, int, bool selected) override {
        graphics.fillAll(selected ? ui::colours::selection : row % 2 == 0 ? ui::colours::panel : ui::colours::panelRaised);
    }

    void paintCell(juce::Graphics& graphics, int row, int column, int width, int height, bool) override {
        if (row < 0 || row >= getNumRows()) return;
        const auto& value = designed_.values[static_cast<std::size_t>(row)];
        juce::String text;
        auto colour = ui::colours::text;
        switch (column) {
        case 1: text = utf8(value.label.c_str()); break;
        case 2:
            text = value.settable ? shown(value.value) + " " + utf8(value.unit.c_str()) : juce::String();
            colour = value.overridden ? ui::colours::accentLight : ui::colours::text;
            break;
        case 3:
            text = value.overridden ? utf8("you") : utf8("rule");
            colour = value.overridden ? ui::colours::accentLight : ui::colours::faint;
            break;
        default: text = utf8(value.rule.c_str()); colour = ui::colours::muted; break;
        }
        graphics.setColour(colour);
        graphics.setFont(column == 2 ? ui::monoFont(13.0F, false) : ui::uiFont(13.0F));
        graphics.drawText(text, 6, 0, width - 8, height, juce::Justification::centredLeft, true);
    }

    void selectedRowsChanged(int) override { showSelected(); }

    [[nodiscard]] const DesignValue* selectedValue() const {
        const auto row = table_.getSelectedRow();
        if (row < 0 || row >= static_cast<int>(designed_.values.size())) return nullptr;
        const auto& value = designed_.values[static_cast<std::size_t>(row)];
        return value.settable ? &value : nullptr;
    }

    void showSelected() {
        const auto* value = selectedValue();
        overrideLabel_.setText(value ? utf8(value->label.c_str()) + (value->unit.empty() ? juce::String()
                                           : utf8(" (") + utf8(value->unit.c_str()) + utf8(")"))
                                     : utf8("Select a value to set it by hand"),
            juce::dontSendNotification);
        overrideEditor_.setText(value ? shown(value->value) : juce::String(), false);
        overrideEditor_.setEnabled(value != nullptr);
        overrideSet_.setEnabled(value != nullptr);
        overrideReset_.setEnabled(value != nullptr && value->overridden);
    }

    void setOverride() {
        const auto* value = selectedValue();
        if (value == nullptr) return;
        const auto text = overrideEditor_.getText().trim();
        if (text.isEmpty() || !text.containsOnly("-0123456789.")) return;
        overrides_[value->key] = text.getDoubleValue();
        redesign();
    }

    void resetOverride() {
        if (const auto* value = selectedValue()) {
            overrides_.erase(value->key);
            redesign();
        }
    }

    void build() {
        if (!designed_.config) return;
        auto config = *designed_.config;
        const auto voicing = loadAudioVoicing(catalogRoot_, "", "");
        if (!voicing) {
            status_.setColour(juce::Label::textColourId, ui::colours::critical);
            status_.setText(utf8("The default voicing did not load: ") + utf8(voicing.error.c_str()),
                juce::dontSendNotification);
            return;
        }
        config.audioVoicing = voicing.voicing;
        checkRealtimeThenBuild(std::move(config));
    }

    /** Real time is a hard limit: a design is run off-screen before it is built
     * (journal 2026-10-19: a 12-cylinder sport car, a two-stroke four and a
     * diesel V8 all came out under real time, and a quad-turbo W16 at half). */
    void checkRealtimeThenBuild(EngineConfig config) {
        if (probing_) return;
        if (probe_.joinable()) probe_.join();
        probing_ = true;
        build_.setEnabled(false);
        livePausedBefore_ = pauseLive_ ? pauseLive_(true) : false;
        status_.setColour(juce::Label::textColourId, ui::colours::text);
        status_.setText(utf8("Checking that this engine runs in real time on this computer (a few seconds)…"),
            juce::dontSendNotification);
        cancelProbe_.store(false);
        auto safe = juce::Component::SafePointer<DesignerContent>(this);
        probe_ = std::thread([this, safe, config = std::move(config)]() mutable {
            auto capacity = measureRealtimeCapacity(config, 1.5, 30.0, &cancelProbe_);
            if (cancelProbe_.load()) return;
            juce::MessageManager::callAsync([safe, config = std::move(config), capacity]() mutable {
                if (safe) safe->probeFinished(std::move(config), capacity);
            });
        });
    }

    void stopProbe() {
        cancelProbe_.store(true);
        if (probe_.joinable()) probe_.join();
        if (probing_ && pauseLive_) pauseLive_(livePausedBefore_);
        probing_ = false;
    }

    void probeFinished(EngineConfig config, const RealtimeCapacity& capacity) {
        if (!probing_) return;
        probing_ = false;
        if (pauseLive_) pauseLive_(livePausedBefore_);
        build_.setEnabled(designed_.config.has_value());
        const auto rpm = juce::String(juce::roundToInt(capacity.measuredRpm > 0.0 ? capacity.measuredRpm
                                                                                     : capacity.holdRpm));
        if (capacity.error.empty() && capacity.factor >= realtimeCapacityMargin) {
            if (buildCallback_(config)) close_();
            return;
        }
        const auto message = capacity.error.empty()
            ? utf8("At ") + rpm + utf8(" rpm this computer runs the engine's physics at ")
                + ui::fixed(capacity.factor, 2) + utf8(utf8Times)
                + utf8(" real time; the sound and the 3-D view need about ")
                + ui::fixed(realtimeCapacityMargin, 2) + utf8(utf8Times)
                + utf8(". It will lag, then go dull and silent.\n\nEach cylinder costs physics time, and more so "
                       "at high speed: fewer cylinders or a lower redline bring it back.")
            : utf8("The real-time check did not finish (") + utf8(capacity.error.c_str())
                + utf8("), so whether this engine runs in real time is not known.");
        status_.setColour(juce::Label::textColourId, ui::colours::critical);
        status_.setText(message, juce::dontSendNotification);
        auto safe = juce::Component::SafePointer<DesignerContent>(this);
        juce::AlertWindow::showOkCancelBox(juce::MessageBoxIconType::WarningIcon,
            utf8(capacity.error.empty() ? "Not real time" : "Real time not checked"), message,
            utf8("Build anyway"), utf8("Back"), this,
            juce::ModalCallbackFunction::create([safe, config = std::move(config)](int result) {
                if (safe && result == 1 && safe->buildCallback_(config)) safe->close_();
            }));
    }

    static constexpr const char* utf8Times = "\xc3\x97";

    std::filesystem::path catalogRoot_;
    BuildCallback buildCallback_;
    PauseCallback pauseLive_;
    std::function<void()> close_;
    std::thread probe_;
    std::atomic<bool> cancelProbe_ { false };
    bool probing_ { false };
    bool livePausedBefore_ { false };
    PartsLibrary parts_;
    std::string partsError_;
    DesignedEngine designed_;
    std::map<std::string, double> overrides_;
    std::vector<Field*> fields_;
    Field name_, cycle_, fuel_, use_, layout_, cylinders_, bankAngle_, bankCount_, bore_, stroke_, displacement_,
        induction_, boost_, turbos_, firingOrder_, crankPins_, crankOffsets_, bankAngles_;
    juce::TableListBox table_;
    juce::Label overrideLabel_;
    juce::TextEditor overrideEditor_;
    juce::TextButton overrideSet_ { utf8("Set") };
    juce::TextButton overrideReset_ { utf8("Rule") };
    juce::Label status_;
    juce::TextButton advanced_ { utf8("Advanced") };
    juce::TextButton build_ { utf8("BUILD AND RUN") };
    juce::TextButton cancel_ { utf8("Cancel") };
};

EngineDesignerWindow::EngineDesignerWindow(std::filesystem::path catalogRoot, BuildCallback buildCallback,
                                           PauseCallback pauseLive)
    : juce::DocumentWindow("EngineLab - New engine", ui::colours::background, juce::DocumentWindow::closeButton,
          true) {
    content_ = new DesignerContent(std::move(catalogRoot), std::move(buildCallback), std::move(pauseLive),
        [this] { setVisible(false); });
    setContentOwned(content_, true);
    setUsingNativeTitleBar(true);
    setResizable(true, false);
    setResizeLimits(1'000, 720, 2'000, 1'300);
    centreWithSize(1'180, 800);
}

EngineDesignerWindow::~EngineDesignerWindow() = default;

void EngineDesignerWindow::closeButtonPressed() { content_->cancel(); }

} // namespace enginelab
