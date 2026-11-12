#include <enginelab/app/FuelWindow.hpp>

#include <enginelab/app/FuelEditing.hpp>
#include <enginelab/app/Theme.hpp>
#include <enginelab/app/Widgets.hpp>

#include <array>
#include <cstddef>
#include <initializer_list>
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
constexpr int energyHeight = 22;
/** The fields' rows: fuel_editing::fields' order. */
constexpr std::size_t heatingRow = 1;
constexpr std::size_t ratioRow = 2;

/** What the window shows of an engine: another engine, another fuel kind or
    another value means the fields are rebuilt. */
[[nodiscard]] bool sameFuel(const EngineConfig& left, const EngineConfig& right) {
    return left.name == right.name && left.fuel == right.fuel
        && left.fuelProperties.name == right.fuelProperties.name
        && fuel_editing::values(left) == fuel_editing::values(right);
}

constexpr double kilowattsPerHorsepower = 0.745699872;

/**
 * The nitrous kit: hidden behind "Fit a kit..." until one is fitted, then
 * every setting of it, taken by the running engine at once.
 */
class NitrousPanel final : public juce::Component {
public:
    explicit NitrousPanel(FuelWindow::NitrousCallbacks callbacks) : callbacks_(std::move(callbacks)) {
        fitButton_.setTooltip(utf8("Fits a nitrous kit: a 100 hp wet shot and a 10 lb bottle, disarmed. "
                                   "Arm it from Drive (Nitrous) and hold its key to spray."));
        fitButton_.onClick = [this] {
            auto kit = NitrousConfig {};
            kit.fitted = true;
            commit(kit);
        };
        removeButton_.setTooltip(utf8("Takes the kit off the engine."));
        removeButton_.onClick = [this] {
            auto kit = kit_;
            kit.fitted = false;
            commit(kit);
        };
        refillButton_.setTooltip(utf8("Fills the bottle again."));
        refillButton_.onClick = [this] { if (callbacks_.refill) callbacks_.refill(); };
        type_.setOptionTooltip(0, utf8("Wet: the kit sprays its own fuel with the nitrous, at its mixture."));
        type_.setOptionTooltip(1, utf8("Dry: nitrous alone; the injectors add the fuel, at the kit's mixture."));
        type_.onChange = [this](int index) {
            auto kit = kit_;
            kit.type = index == 1 ? NitrousKitType::dry : NitrousKitType::wet;
            commit(kit);
        };
        unlimited_.setTooltip(utf8("A bottle that never runs down."));
        unlimited_.onClick = [this] { auto kit = kit_; kit.unlimitedBottle = !kit.unlimitedBottle; commit(kit); };
        heater_.setTooltip(utf8("A bottle heater holds the bottle at its temperature, so its pressure; "
                                "without it the bottle cools as it empties and the flow falls."));
        heater_.onClick = [this] { auto kit = kit_; kit.bottleHeater = !kit.bottleHeater; commit(kit); };
        push_.setTooltip(utf8("Push button: armed, it sprays only while its key is held. Off, it sprays "
                              "whenever armed, past the throttle and in the speed window."));
        push_.onClick = [this] { auto kit = kit_; kit.pushButton = !kit.pushButton; commit(kit); };
        for (auto* child : std::initializer_list<juce::Component*> { &fitButton_, &removeButton_, &refillButton_,
                                                                     &type_, &unlimited_, &heater_, &push_,
                                                                     &readout_, &none_ })
            addChildComponent(child);
        none_.setText(utf8("No kit"), juce::dontSendNotification);
        none_.setColour(juce::Label::textColourId, ui::colours::muted);
        readout_.setColour(juce::Label::textColourId, ui::colours::muted);
        readout_.setJustificationType(juce::Justification::centredLeft);
        const std::array<FieldSpec, fieldCount> specs { {
            { "Shot", " hp", 0.0, 100000.0, 5.0, 0,
              "The power the kit's jets are sized to add: a 100 hp shot passes 2.27 kg/min of nitrous at "
              "900 psi into the atmosphere." },
            { "Bottle", " kg", 0.0, 100000.0, 0.1, 2, "Nitrous in a full bottle (a 10 lb bottle holds 4.54 kg)." },
            { "Bottle temperature", " C", -40.0, 150.0, 1.0, 0,
              "The bottle's temperature when full, which sets its pressure (30 C: 62 bar). The heater holds it." },
            { "Minimum throttle", " %", 0.0, 100.0, 1.0, 0, "It sprays only past this throttle opening." },
            { "Minimum speed", " rpm", 0.0, 100000.0, 100.0, 0, "It sprays only above this engine speed." },
            { "Maximum speed", " rpm", 0.0, 100000.0, 100.0, 0,
              "It stops above this engine speed; 0 is none (the limiter cuts it)." },
            { "Ramp", " s", 0.0, 1000.0, 0.1, 1,
              "A progressive controller: the shot builds up over this time; 0 is all at once." },
            { "Retard", " deg/50 hp", -90.0, 90.0, 0.5, 1,
              "Spark retard while spraying, per 50 hp of shot (2 is the common rule)." },
            { "Kit mixture", " lambda", 0.05, 20.0, 0.01, 2,
              "The mixture of the nitrous's oxygen with the kit's fuel: 0.80 is rich and safe; "
              "lean it out and the engine knocks." },
        } };
        for (std::size_t index = 0; index < fieldCount; ++index) {
            auto& field = fields_[index];
            const auto& spec = specs[index];
            field.label.setText(utf8(spec.label), juce::dontSendNotification);
            field.label.setColour(juce::Label::textColourId, ui::colours::text);
            field.label.setTooltip(utf8(spec.tooltip));
            field.slider.setRange(spec.minimum, spec.maximum, spec.step);
            field.slider.setNumDecimalPlacesToDisplay(spec.decimals);
            field.slider.setTextValueSuffix(utf8(spec.unit));
            field.slider.setTextBoxStyle(juce::Slider::TextBoxLeft, false, 130, rowHeight);
            field.slider.setIncDecButtonsMode(juce::Slider::incDecButtonsDraggable_AutoDirection);
            field.slider.setTooltip(utf8(spec.tooltip));
            field.slider.onValueChange = [this] { if (!loading_) fieldsEdited(); };
            addChildComponent(field.label);
            addChildComponent(field.slider);
        }
        show(kit_, true);
    }

    std::function<void()> onLayoutChanged;

    [[nodiscard]] static int preferredHeight(bool fitted) {
        if (!fitted) return 18 + 8 + 34;
        constexpr int fieldRows = static_cast<int>((fieldCount + 1) / 2);
        return 18 + 8 + 30 + rowGap + 30 + rowGap + fieldRows * (rowHeight + rowGap) + energyHeight;
    }

    [[nodiscard]] bool fitted() const noexcept { return kit_.fitted; }

    void setKit(const NitrousConfig& kit) {
        if (kit == kit_) return;
        show(kit, false);
    }

    void setLocked(bool locked) {
        locked_ = locked;
        for (auto* control : std::initializer_list<juce::Component*> { &fitButton_, &removeButton_, &type_,
                                                                       &unlimited_, &heater_, &push_ })
            control->setEnabled(!locked);
        for (auto& field : fields_) field.slider.setEnabled(!locked);
    }

    void setState(const EngineState& state) {
        if (!kit_.fitted) return;
        juce::String text;
        if (kit_.unlimitedBottle) text << utf8("Unlimited bottle");
        else text << utf8("Bottle ") << juce::String(state.nitrousBottleMassKg, 2) << utf8(" kg");
        text << utf8(", ") << juce::String(state.nitrousBottlePressureKpa / 100.0, 1) << utf8(" bar, ")
             << juce::String(state.nitrousBottleTemperatureC, 0) << utf8(" C");
        if (state.nitrousSpraying)
            text << utf8("  -  spraying ") << juce::String(state.nitrousMassFlowKgPerSecond * 60.0, 2)
                 << utf8(" kg/min, ") << juce::String(state.nitrousOxygenShare * 100.0, 0)
                 << utf8(" % of the oxygen");
        if (readout_.getText() != text) readout_.setText(text, juce::dontSendNotification);
    }

    void paint(juce::Graphics& g) override {
        ui::drawSectionLabel(g, getLocalBounds().removeFromTop(18).toFloat(), utf8("NITROUS"));
    }

    void resized() override {
        auto area = getLocalBounds();
        area.removeFromTop(18 + 8);
        if (!kit_.fitted) {
            auto row = area.removeFromTop(34);
            fitButton_.setBounds(row.removeFromRight(fitButton_.idealWidth()));
            none_.setBounds(row);
            return;
        }
        auto top = area.removeFromTop(30);
        type_.setBounds(top.removeFromLeft(type_.idealWidth()));
        top.removeFromLeft(12);
        const auto toggleWidth = top.getWidth() / 3;
        unlimited_.setBounds(top.removeFromLeft(toggleWidth).reduced(3, 0));
        heater_.setBounds(top.removeFromLeft(toggleWidth).reduced(3, 0));
        push_.setBounds(top.reduced(3, 0));
        area.removeFromTop(rowGap);
        auto buttons = area.removeFromTop(30);
        removeButton_.setBounds(buttons.removeFromRight(removeButton_.idealWidth()));
        buttons.removeFromRight(8);
        refillButton_.setBounds(buttons.removeFromRight(refillButton_.idealWidth()));
        area.removeFromTop(rowGap);
        const auto columnWidth = (area.getWidth() - 16) / 2;
        for (std::size_t index = 0; index < fieldCount; index += 2) {
            auto line = area.removeFromTop(rowHeight);
            for (std::size_t column = 0; column < 2 && index + column < fieldCount; ++column) {
                auto cell = column == 0 ? line.removeFromLeft(columnWidth) : line.removeFromRight(columnWidth);
                auto& field = fields_[index + column];
                field.label.setBounds(cell.removeFromLeft(130));
                field.slider.setBounds(cell);
            }
            area.removeFromTop(rowGap);
        }
        readout_.setBounds(area.removeFromTop(energyHeight));
    }

private:
    struct FieldSpec final {
        const char* label;
        const char* unit;
        double minimum;
        double maximum;
        double step;
        int decimals;
        const char* tooltip;
    };
    struct Field final {
        juce::Label label;
        juce::Slider slider { juce::Slider::IncDecButtons, juce::Slider::TextBoxLeft };
    };
    static constexpr std::size_t fieldCount = 9;

    [[nodiscard]] static std::array<double, fieldCount> valuesOf(const NitrousConfig& kit) {
        return { kit.shotKw / kilowattsPerHorsepower, kit.bottleCapacityKg, kit.bottleTemperatureC,
                 kit.minimumThrottle * 100.0, kit.minimumRpm, kit.maximumRpm, kit.rampSeconds,
                 kit.sparkRetardDegreesPer50Hp, kit.kitLambda };
    }

    void show(const NitrousConfig& kit, bool first) {
        const auto layoutChanged = first || kit.fitted != kit_.fitted;
        kit_ = kit;
        loading_ = true;
        const auto values = valuesOf(kit);
        for (std::size_t index = 0; index < fieldCount; ++index)
            fields_[index].slider.setValue(values[index], juce::dontSendNotification);
        loading_ = false;
        type_.setSelected(kit.type == NitrousKitType::dry ? 1 : 0);
        unlimited_.setEngaged(kit.unlimitedBottle);
        heater_.setEngaged(kit.bottleHeater);
        push_.setEngaged(kit.pushButton);
        none_.setVisible(!kit.fitted);
        fitButton_.setVisible(!kit.fitted);
        for (auto* child : std::initializer_list<juce::Component*> { &removeButton_, &refillButton_, &type_,
                                                                     &unlimited_, &heater_, &push_, &readout_ })
            child->setVisible(kit.fitted);
        for (auto& field : fields_) {
            field.label.setVisible(kit.fitted);
            field.slider.setVisible(kit.fitted);
        }
        if (!kit.fitted) readout_.setText({}, juce::dontSendNotification);
        resized();
        repaint();
        if (layoutChanged && onLayoutChanged) onLayoutChanged();
    }

    void fieldsEdited() {
        auto kit = kit_;
        const auto value = [this](std::size_t index) { return fields_[index].slider.getValue(); };
        kit.shotKw = value(0) * kilowattsPerHorsepower;
        kit.bottleCapacityKg = value(1);
        kit.bottleTemperatureC = value(2);
        kit.minimumThrottle = value(3) / 100.0;
        kit.minimumRpm = value(4);
        kit.maximumRpm = value(5);
        kit.rampSeconds = value(6);
        kit.sparkRetardDegreesPer50Hp = value(7);
        kit.kitLambda = value(8);
        commit(kit);
    }

    /** The engine takes the kit at once; one it refuses is shown back as it was. */
    void commit(const NitrousConfig& kit) {
        if (locked_ || !callbacks_.apply || !callbacks_.apply(kit)) {
            show(kit_, false);
            return;
        }
        show(kit, false);
    }

    FuelWindow::NitrousCallbacks callbacks_;
    NitrousConfig kit_ {};
    bool loading_ { false };
    bool locked_ { false };
    juce::Label none_;
    juce::Label readout_;
    ui::ActionButton fitButton_ { utf8("Fit a kit..."), ui::ActionButton::Style::standard };
    ui::ActionButton removeButton_ { utf8("Remove kit"), ui::ActionButton::Style::compact };
    ui::ActionButton refillButton_ { utf8("Refill bottle"), ui::ActionButton::Style::compact };
    ui::SegmentedControl type_ { juce::StringArray { "Wet", "Dry" } };
    ui::ActionButton unlimited_ { utf8("Unlimited bottle"), ui::ActionButton::Style::compact };
    ui::ActionButton heater_ { utf8("Bottle heater"), ui::ActionButton::Style::compact };
    ui::ActionButton push_ { utf8("Push button"), ui::ActionButton::Style::compact };
    std::array<Field, fieldCount> fields_;
};

} // namespace

class FuelWindow::Content final : public juce::Component {
public:
    Content(const EngineConfig& config, std::vector<FuelPreset> presets, ApplyCallback applyCallback,
            NitrousCallbacks nitrous)
        : presets_(std::move(presets)), apply_(std::move(applyCallback)), nitrous_(std::move(nitrous)) {
        setOpaque(true);
        nitrous_.onLayoutChanged = [this] { fitWindow(); };
        addAndMakeVisible(nitrous_);
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
        energy_.setJustificationType(juce::Justification::centredLeft);
        energy_.setTooltip(utf8("The heat the fuel releases per kilogram of the air's oxygen it burns. Every real "
                                "fuel that takes its oxygen from the air is near 13 MJ/kg (petrol 12.6, ethanol "
                                "12.8, methanol 13.3, diesel 12.7): above it the engine makes more power from the "
                                "same air than such a fuel can. The heating value follows the stoichiometric ratio "
                                "to keep it, unless you set the heating value yourself."));
        addAndMakeVisible(energy_);
        applyButton_.setTooltip(utf8("Puts this fuel in the tank. A new octane alone is taken at once; another "
                                     "fuel or value restarts the engine. Another stoichiometric ratio resizes the "
                                     "injectors and carburettor jets for the same mixture. No spark map is changed."));
        applyButton_.onClick = [this] { applyEdits(); };
        revertButton_.setTooltip(utf8("Back to the fuel in the tank."));
        revertButton_.onClick = [this] { show(shown_); };
        addAndMakeVisible(applyButton_);
        addAndMakeVisible(revertButton_);
        show(config);
    }

    void setNitrousState(const EngineState& state) { nitrous_.setState(state); }

    void setConfig(const EngineConfig& config) {
        nitrous_.setKit(config.nitrous);
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
        nitrous_.setLocked(locked);
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
        energy_.setBounds(area.removeFromTop(energyHeight));
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
        area.removeFromTop(10);
        nitrous_.setBounds(area.removeFromTop(NitrousPanel::preferredHeight(nitrous_.fitted())));
    }

    [[nodiscard]] static int preferredHeight(bool diesel, bool kitFitted) {
        const auto fieldCount = static_cast<int>(fuel_editing::fields(diesel).size());
        return 14 + 18 + 26 + 18 + 8 + rowHeight + rowGap + 6 + fieldCount * (rowHeight + rowGap) + energyHeight
            + 6 + 34 + 12 + 40 + 90 + 10 + NitrousPanel::preferredHeight(kitFitted) + 14;
    }
    /** A fitted kit's settings need the wider window. */
    [[nodiscard]] static int preferredWidth(bool kitFitted) { return kitFitted ? 760 : 520; }

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
                    if (rows_.size() > ratioRow && raw == rows_[ratioRow].get()) followHeatingValue();
                    valuesEdited();
                };
                row->slider.setEnabled(!locked_);
                addAndMakeVisible(row->label);
                addAndMakeVisible(row->slider);
                rows_.push_back(std::move(row));
            }
            diesel_ = diesel;
            fitWindow();
        }
        showValues(config);
        note_.setText(utf8(fuel_editing::note(diesel)), juce::dontSendNotification);
        updateButtons();
        repaint();
    }

    /** The window's size for the fuel's fields and the kit's section. */
    void fitWindow() {
        if (auto* window = findParentComponentOfClass<juce::DocumentWindow>())
            window->setSize(preferredWidth(nitrous_.fitted()), preferredHeight(diesel_, nitrous_.fitted()));
        resized();
    }

    void showValues(const EngineConfig& config) {
        const auto values = fuel_editing::values(config);
        loading_ = true;
        for (std::size_t index = 0; index < rows_.size() && index < values.size(); ++index) {
            rows_[index]->slider.setValue(values[index], juce::dontSendNotification);
            rows_[index]->touched = false;
        }
        loading_ = false;
        showEnergy();
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
        showEnergy();
    }

    /** A new stoichiometric ratio moves the heating value with it (the same
        energy per kilogram of oxygen) until the heating value is set by hand. */
    void followHeatingValue() {
        if (rows_.size() <= ratioRow || rows_[heatingRow]->touched) return;
        const auto base = draft();
        loading_ = true;
        rows_[heatingRow]->slider.setValue(fuel_editing::followingHeatingValue(base.fuelProperties,
            rows_[ratioRow]->slider.getValue()), juce::dontSendNotification);
        loading_ = false;
    }

    /** The energy per kilogram of oxygen of the fuel the fields describe. */
    void showEnergy() {
        auto preview = draft();
        if (!fuel_editing::apply(preview, fieldValues(preview)).empty()) return;
        const auto energy = fuel_editing::energyPerOxygenMjPerKg(preview.fuelProperties);
        // Real fuels: 12.5-13.5 MJ/kg.
        constexpr double realFuelCeilingMjPerKg = 14.0;
        energy_.setColour(juce::Label::textColourId,
                          energy > realFuelCeilingMjPerKg ? ui::colours::warn : ui::colours::muted);
        energy_.setText(utf8("Energy per kg of oxygen: ") + juce::String(energy, 1)
                            + utf8(energy > realFuelCeilingMjPerKg
                                       ? " MJ/kg, more than any real fuel (about 13)"
                                       : " MJ/kg (real fuels about 13)"),
                        juce::dontSendNotification);
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
    juce::Label energy_;
    ui::ActionButton applyButton_ { "Apply", ui::ActionButton::Style::primary };
    ui::ActionButton revertButton_ { "Revert", ui::ActionButton::Style::standard };
    NitrousPanel nitrous_;
};

FuelWindow::FuelWindow(const EngineConfig& config, std::vector<FuelPreset> presets, ApplyCallback applyCallback,
                       NitrousCallbacks nitrous)
    : juce::DocumentWindow("EngineLab - Fuel", ui::colours::background, juce::DocumentWindow::closeButton, true) {
    content_ = new Content(config, std::move(presets), std::move(applyCallback), std::move(nitrous));
    setContentOwned(content_, false);
    setUsingNativeTitleBar(true);
    setResizable(false, false);
    content_->setConfig(config);
    centreWithSize(Content::preferredWidth(config.nitrous.fitted),
                   Content::preferredHeight(config.fuel == FuelType::diesel, config.nitrous.fitted));
}

void FuelWindow::setNitrousState(const EngineState& state) {
    if (content_ != nullptr) content_->setNitrousState(state);
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
