#include <enginelab/app/EquipmentWindow.hpp>

#include <enginelab/app/Theme.hpp>
#include <enginelab/app/Widgets.hpp>
#include <enginelab/calibration/EcuCalibration.hpp>
#include <enginelab/calibration/EcuCalibrationKeys.hpp>
#include <enginelab/exhaust/ExhaustPathTopologyEditor.hpp>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <initializer_list>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace enginelab {
namespace {

using ui::utf8;

constexpr int rowHeight = 30;
constexpr int rowGap = 8;
constexpr int titleHeight = 18;
constexpr int statusHeight = 22;
constexpr int sectionGap = 22;
constexpr int labelWidth = 150;
constexpr double kilowattsPerHorsepower = 0.745699872;

struct FieldSpec final {
    const char* label;
    const char* unit;
    double minimum;
    double maximum;
    double step;
    int decimals;
    const char* tooltip;
};

/** Labelled number fields, two to a row; an edit calls onEdited. */
class FieldGrid final {
public:
    FieldGrid(juce::Component& owner, std::vector<FieldSpec> specs) : fields_(specs.size()) {
        for (std::size_t index = 0; index < specs.size(); ++index) {
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
            field.slider.onValueChange = [this, index] { if (!loading_ && onEdited) onEdited(index); };
            owner.addChildComponent(field.label);
            owner.addChildComponent(field.slider);
        }
    }

    std::function<void(std::size_t)> onEdited;

    [[nodiscard]] std::size_t size() const noexcept { return fields_.size(); }
    [[nodiscard]] double value(std::size_t index) const { return fields_[index].slider.getValue(); }
    [[nodiscard]] juce::Slider& slider(std::size_t index) { return fields_[index].slider; }

    void setValues(const std::vector<double>& values) {
        loading_ = true;
        for (std::size_t index = 0; index < fields_.size() && index < values.size(); ++index)
            fields_[index].slider.setValue(values[index], juce::dontSendNotification);
        loading_ = false;
    }
    void setVisible(bool visible) {
        for (auto& field : fields_) {
            field.label.setVisible(visible);
            field.slider.setVisible(visible);
        }
    }
    void setEnabled(bool enabled) {
        for (auto& field : fields_) field.slider.setEnabled(enabled);
    }
    [[nodiscard]] int height() const noexcept {
        return static_cast<int>((fields_.size() + 1) / 2) * (rowHeight + rowGap);
    }
    void layout(juce::Rectangle<int>& area) {
        const auto columnWidth = (area.getWidth() - 16) / 2;
        for (std::size_t index = 0; index < fields_.size(); index += 2) {
            auto line = area.removeFromTop(rowHeight);
            for (std::size_t column = 0; column < 2 && index + column < fields_.size(); ++column) {
                auto cell = column == 0 ? line.removeFromLeft(columnWidth) : line.removeFromRight(columnWidth);
                auto& field = fields_[index + column];
                field.label.setBounds(cell.removeFromLeft(labelWidth));
                field.slider.setBounds(cell);
            }
            area.removeFromTop(rowGap);
        }
    }

private:
    struct Field final {
        juce::Label label;
        juce::Slider slider { juce::Slider::IncDecButtons, juce::Slider::TextBoxLeft };
    };
    std::vector<Field> fields_;
    bool loading_ { false };
};

/** One part's page: a title, what it is doing, and its controls. */
class Section : public juce::Component {
public:
    explicit Section(juce::String title) : title_(std::move(title)) {
        status_.setColour(juce::Label::textColourId, ui::colours::muted);
        status_.setJustificationType(juce::Justification::centredLeft);
        addAndMakeVisible(status_);
    }
    [[nodiscard]] virtual int preferredHeight() const = 0;
    virtual void setEngine(const EngineConfig&, const std::shared_ptr<calibration::CalibrationStore>&) {}
    virtual void setState(const EngineState&) {}
    virtual void setLocked(bool locked) { locked_ = locked; }
    std::function<void()> onLayoutChanged;

    void paint(juce::Graphics& g) override {
        ui::drawSectionLabel(g, getLocalBounds().removeFromTop(titleHeight).toFloat(), title_);
    }

protected:
    void setStatus(const juce::String& text) {
        if (status_.getText() != text) status_.setText(text, juce::dontSendNotification);
    }
    /** The area under the title and the status line. */
    [[nodiscard]] juce::Rectangle<int> body() {
        auto area = getLocalBounds();
        area.removeFromTop(titleHeight + 4);
        status_.setBounds(area.removeFromTop(statusHeight));
        area.removeFromTop(rowGap);
        return area;
    }
    static constexpr int headerHeight = titleHeight + 4 + statusHeight + rowGap;
    void layoutChanged() {
        resized();
        repaint();
        if (onLayoutChanged) onLayoutChanged();
    }
    bool locked_ { false };
    juce::Label status_;

private:
    juce::String title_;
};

[[nodiscard]] juce::String rpmText(double rpm) { return juce::String(rpm, 0) + utf8(" rpm"); }

// --------------------------------------------------------------------------

class NitrousSection final : public Section {
public:
    explicit NitrousSection(EquipmentWindow::Callbacks& callbacks)
        : Section(utf8("NITROUS")), callbacks_(callbacks),
          fields_(*this, {
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
          }) {
        fitButton_.setTooltip(utf8("Fits a nitrous kit: a 100 hp wet shot and a 10 lb bottle, disarmed. "
                                   "Arm it from Drive (Nitrous): it sprays at full throttle."));
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
        refillButton_.onClick = [this] { if (callbacks_.refillNitrous) callbacks_.refillNitrous(); };
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
        push_.setTooltip(utf8("Push button: armed, it sprays only while its key (=) is held. Off (the "
                              "default), it sprays whenever armed, past the throttle and in the speed window."));
        push_.onClick = [this] { auto kit = kit_; kit.pushButton = !kit.pushButton; commit(kit); };
        for (auto* child : std::initializer_list<juce::Component*> { &fitButton_, &removeButton_, &refillButton_,
                                                                     &type_, &unlimited_, &heater_, &push_ })
            addChildComponent(child);
        fields_.onEdited = [this](std::size_t) { fieldsEdited(); };
        show(kit_, true);
    }

    [[nodiscard]] int preferredHeight() const override {
        if (!kit_.fitted) return headerHeight + 34;
        return headerHeight + 30 + rowGap + 30 + rowGap + fields_.height();
    }

    void setEngine(const EngineConfig& config, const std::shared_ptr<calibration::CalibrationStore>&) override {
        if (config.nitrous == kit_) return;
        show(config.nitrous, false);
    }

    void setLocked(bool locked) override {
        Section::setLocked(locked);
        for (auto* control : std::initializer_list<juce::Component*> { &fitButton_, &removeButton_, &type_,
                                                                       &unlimited_, &heater_, &push_ })
            control->setEnabled(!locked);
        fields_.setEnabled(!locked);
    }

    void setState(const EngineState& state) override {
        if (!kit_.fitted) {
            setStatus(utf8("No kit fitted."));
            return;
        }
        juce::String text;
        if (state.nitrousSpraying)
            text << utf8("Spraying ") << juce::String(state.nitrousMassFlowKgPerSecond * 60.0, 2)
                 << utf8(" kg/min, ") << juce::String(state.nitrousOxygenShare * 100.0, 0)
                 << utf8(" % of the oxygen.  ");
        else
            text << describeNitrousBlockers(state.nitrousBlockers, kit_) << utf8("  ");
        if (kit_.unlimitedBottle) text << utf8("Unlimited bottle");
        else text << utf8("Bottle ") << juce::String(state.nitrousBottleMassKg, 2) << utf8(" kg");
        text << utf8(", ") << juce::String(state.nitrousBottlePressureKpa / 100.0, 1) << utf8(" bar, ")
             << juce::String(state.nitrousBottleTemperatureC, 0) << utf8(" C");
        setStatus(text);
    }

    void resized() override {
        auto area = body();
        if (!kit_.fitted) {
            auto row = area.removeFromTop(34);
            fitButton_.setBounds(row.removeFromLeft(fitButton_.idealWidth()));
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
        fields_.layout(area);
    }

private:
    [[nodiscard]] static std::vector<double> valuesOf(const NitrousConfig& kit) {
        return { kit.shotKw / kilowattsPerHorsepower, kit.bottleCapacityKg, kit.bottleTemperatureC,
                 kit.minimumThrottle * 100.0, kit.minimumRpm, kit.maximumRpm, kit.rampSeconds,
                 kit.sparkRetardDegreesPer50Hp, kit.kitLambda };
    }

    void show(const NitrousConfig& kit, bool first) {
        const auto changed = first || kit.fitted != kit_.fitted;
        kit_ = kit;
        fields_.setValues(valuesOf(kit));
        type_.setSelected(kit.type == NitrousKitType::dry ? 1 : 0);
        unlimited_.setEngaged(kit.unlimitedBottle);
        heater_.setEngaged(kit.bottleHeater);
        push_.setEngaged(kit.pushButton);
        fitButton_.setVisible(!kit.fitted);
        for (auto* child : std::initializer_list<juce::Component*> { &removeButton_, &refillButton_, &type_,
                                                                     &unlimited_, &heater_, &push_ })
            child->setVisible(kit.fitted);
        fields_.setVisible(kit.fitted);
        if (!kit.fitted) setStatus(utf8("No kit fitted."));
        if (changed) layoutChanged();
        else resized();
    }

    void fieldsEdited() {
        auto kit = kit_;
        kit.shotKw = fields_.value(0) * kilowattsPerHorsepower;
        kit.bottleCapacityKg = fields_.value(1);
        kit.bottleTemperatureC = fields_.value(2);
        kit.minimumThrottle = fields_.value(3) / 100.0;
        kit.minimumRpm = fields_.value(4);
        kit.maximumRpm = fields_.value(5);
        kit.rampSeconds = fields_.value(6);
        kit.sparkRetardDegreesPer50Hp = fields_.value(7);
        kit.kitLambda = fields_.value(8);
        commit(kit);
    }

    /** The engine takes the kit at once; one it refuses is shown back as it was. */
    void commit(const NitrousConfig& kit) {
        if (locked_ || !callbacks_.applyNitrous || !callbacks_.applyNitrous(kit)) {
            show(kit_, false);
            return;
        }
        show(kit, false);
    }

    EquipmentWindow::Callbacks& callbacks_;
    NitrousConfig kit_ {};
    FieldGrid fields_;
    ui::ActionButton fitButton_ { utf8("Fit a kit..."), ui::ActionButton::Style::standard };
    ui::ActionButton removeButton_ { utf8("Remove kit"), ui::ActionButton::Style::compact };
    ui::ActionButton refillButton_ { utf8("Refill bottle"), ui::ActionButton::Style::compact };
    ui::SegmentedControl type_ { juce::StringArray { "Wet", "Dry" } };
    ui::ActionButton unlimited_ { utf8("Unlimited bottle"), ui::ActionButton::Style::compact };
    ui::ActionButton heater_ { utf8("Bottle heater"), ui::ActionButton::Style::compact };
    ui::ActionButton push_ { utf8("Push button"), ui::ActionButton::Style::compact };
};

// --------------------------------------------------------------------------

class ValveSection final : public Section {
public:
    explicit ValveSection(EquipmentWindow::Callbacks& callbacks)
        : Section(utf8("ACTIVE EXHAUST VALVE")), callbacks_(callbacks),
          fields_(*this, {
            { "Opens above", " rpm", 0.0, 100000.0, 100.0, 0,
              "Automatic: the valve opens at or above this engine speed..." },
            { "...or above", " % throttle", 0.0, 100.0, 1.0, 0, "...or at or above this throttle opening." },
            { "Travel", " s", 0.0, 60.0, 0.05, 2, "Closed to open: about 0.3 s on a production actuator." },
          }) {
        fitButton_.setTooltip(utf8("Fits a bypass around each exhaust path's last silencer, behind an "
                                   "active valve. Fully open it is the same exhaust without the valve."));
        fitButton_.onClick = [this] { fit(); };
        designerButton_.setTooltip(utf8("The exhaust designer: a valve on another part (right-click it), or "
                                        "taken off."));
        designerButton_.onClick = [this] { if (callbacks_.openExhaustDesigner) callbacks_.openExhaustDesigner(); };
        mode_.setOptionTooltip(0, utf8("Automatic: opens above its speed or throttle."));
        mode_.setOptionTooltip(1, utf8("Shut: the gas goes through the silencer."));
        mode_.setOptionTooltip(2, utf8("Open: most of the gas takes the bypass."));
        mode_.onChange = [this](int index) {
            if (callbacks_.setValveMode)
                callbacks_.setValveMode(index == 1 ? ExhaustValveMode::closed
                                        : index == 2 ? ExhaustValveMode::open : ExhaustValveMode::automatic);
        };
        fields_.onEdited = [this](std::size_t) { fieldsEdited(); };
        for (auto* child : std::initializer_list<juce::Component*> { &fitButton_, &designerButton_, &mode_ })
            addChildComponent(child);
        designerButton_.setVisible(true);
        mode_.setSelected(0);
    }

    [[nodiscard]] int preferredHeight() const override {
        return headerHeight + 34 + (fitted_ ? rowGap + fields_.height() : 0);
    }

    void setEngine(const EngineConfig& config, const std::shared_ptr<calibration::CalibrationStore>&) override {
        config_ = config;
        fields_.setValues({ config.exhaustValve.openAboveRpm, config.exhaustValve.openAboveThrottle * 100.0,
                            config.exhaustValve.travelSeconds });
        const auto fitted = exhaustValveFitted(config);
        if (fitted == fitted_ && shown_) return;
        shown_ = true;
        fitted_ = fitted;
        fitButton_.setVisible(!fitted);
        mode_.setVisible(fitted);
        fields_.setVisible(fitted);
        if (!fitted) setStatus(utf8("No valve fitted."));
        layoutChanged();
    }

    void setMode(ExhaustValveMode mode) {
        mode_.setSelected(mode == ExhaustValveMode::closed ? 1 : mode == ExhaustValveMode::open ? 2 : 0);
    }

    void setLocked(bool locked) override {
        Section::setLocked(locked);
        fitButton_.setEnabled(!locked);
        fields_.setEnabled(!locked);
    }

    void setState(const EngineState& state) override {
        if (!fitted_) return;
        setStatus(utf8("Valve ") + juce::String(state.exhaustValveOpening * 100.0, 0) + utf8(" % open. Drive "
                  "switches it too (key -)."));
    }

    void resized() override {
        auto area = body();
        auto row = area.removeFromTop(34);
        designerButton_.setBounds(row.removeFromRight(designerButton_.idealWidth()));
        row.removeFromRight(8);
        if (fitted_) mode_.setBounds(row.removeFromLeft(mode_.idealWidth()).withSizeKeepingCentre(mode_.idealWidth(), 30));
        else fitButton_.setBounds(row.removeFromLeft(fitButton_.idealWidth()));
        area.removeFromTop(rowGap);
        if (fitted_) fields_.layout(area);
    }

private:
    void fit() {
        if (locked_ || !callbacks_.applyEngine) return;
        auto edited = config_;
        if (const auto error = ExhaustNetworkEditor::fitSilencerBypass(edited); !error.empty()) {
            setStatus(utf8(error.c_str()));
            return;
        }
        if (!callbacks_.applyEngine(edited)) setStatus(utf8("The engine did not take the valve."));
    }

    void fieldsEdited() {
        if (locked_ || !callbacks_.applyEngine) return;
        auto edited = config_;
        edited.exhaustValve.openAboveRpm = fields_.value(0);
        edited.exhaustValve.openAboveThrottle = fields_.value(1) / 100.0;
        edited.exhaustValve.travelSeconds = fields_.value(2);
        if (!callbacks_.applyEngine(edited)) setEngine(config_, {});
    }

    EquipmentWindow::Callbacks& callbacks_;
    EngineConfig config_ {};
    bool fitted_ { false };
    bool shown_ { false };
    FieldGrid fields_;
    ui::ActionButton fitButton_ { utf8("Fit a bypass valve"), ui::ActionButton::Style::standard };
    ui::ActionButton designerButton_ { utf8("Exhaust designer..."), ui::ActionButton::Style::compact };
    ui::SegmentedControl mode_ { juce::StringArray { "Automatic", "Shut", "Open" } };
};

// --------------------------------------------------------------------------

/** A section over scalar settings of the ECU's tables (VTEC, launch
 *  control): it reads them from each new snapshot and publishes an edit. */
class TableSection : public Section {
public:
    TableSection(juce::String title, std::vector<FieldSpec> specs, std::vector<std::string_view> keys,
                 std::vector<double> displayScale)
        : Section(std::move(title)), fields_(*this, std::move(specs)), keys_(std::move(keys)),
          scale_(std::move(displayScale)) {
        fields_.onEdited = [this](std::size_t index) {
            if (locked_ || !store_) return;
            if (!EquipmentWindow::publishScalar(*store_, keys_[index], fields_.value(index) / scale_[index]))
                readTables(true);
        };
    }

    void setEngine(const EngineConfig& config, const std::shared_ptr<calibration::CalibrationStore>& store) override {
        config_ = config;
        const auto changed = store != store_;
        store_ = store;
        readTables(changed);
    }

    void setLocked(bool locked) override {
        Section::setLocked(locked);
        fields_.setEnabled(!locked);
    }

protected:
    /** Whether the tables carry `key`, and its value. */
    [[nodiscard]] std::optional<double> table(std::string_view key) const {
        if (!store_) return std::nullopt;
        return store_->snapshot()->scalar(key);
    }
    /** Shows the fields when the tables carry every key; true when that changed. */
    bool readTables(bool force) {
        if (!store_) return false;
        const auto snapshot = store_->snapshot();
        if (!force && snapshot->revision() == revision_) return false;
        revision_ = snapshot->revision();
        std::vector<double> values;
        auto carried = true;
        for (std::size_t index = 0; index < keys_.size(); ++index) {
            const auto value = snapshot->scalar(keys_[index]);
            carried = carried && value.has_value();
            values.push_back(value.value_or(0.0) * scale_[index]);
        }
        fields_.setValues(values);
        tablesChanged(*snapshot);
        if (carried == carried_ && shown_) return false;
        shown_ = true;
        carried_ = carried;
        fields_.setVisible(carried);
        layoutChanged();
        return true;
    }
    virtual void tablesChanged(const calibration::CalibrationSnapshot&) {}

    FieldGrid fields_;
    std::vector<std::string_view> keys_;
    std::vector<double> scale_;
    std::shared_ptr<calibration::CalibrationStore> store_;
    EngineConfig config_ {};
    std::uint64_t revision_ { 0 };
    bool carried_ { false };
    bool shown_ { false };
};

class VtecSection final : public TableSection {
public:
    explicit VtecSection(EquipmentWindow::Callbacks& callbacks)
        : TableSection(utf8("VTEC"),
                       {
                           { "Engages at", " rpm", -1.0, calibration::ecuLimits::maximumRevLimitRpm, 50.0, 0,
                             "The high cam above this speed (-1: each camshaft's own)." },
                           { "Hysteresis", " rpm", 0.0, calibration::ecuLimits::maximumVtecHysteresisRpm, 10.0, 0,
                             "It drops back to the low cam this far under it." },
                           { "Minimum throttle", " %", -1.0, 100.0, 1.0, 0,
                             "Never on the high cam under this throttle (-1: each camshaft's own)." },
                       },
                       { calibration::keys::vtecEngage, calibration::keys::vtecHysteresis,
                         calibration::keys::vtecMinimumThrottle },
                       { 1.0, 1.0, 1.0 }),
          callbacks_(callbacks) {
        fitButton_.setTooltip(utf8("Fits VTEC cams: a high cam over each camshaft, as the K20A's, and the "
                                   "ECU's switch."));
        fitButton_.onClick = [this] { if (!locked_ && callbacks_.fitVtecCams) callbacks_.fitVtecCams(); };
        switch_.setTooltip(utf8("The ECU's VTEC switch: off holds the low cam."));
        switch_.onClick = [this] {
            if (locked_ || !store_) return;
            const auto on = table(calibration::keys::vtecEnabled).value_or(0.0) >= 0.5;
            (void)EquipmentWindow::publishScalar(*store_, calibration::keys::vtecEnabled, on ? 0.0 : 1.0);
            readTables(true);
        };
        addChildComponent(fitButton_);
        addChildComponent(switch_);
    }

    [[nodiscard]] int preferredHeight() const override {
        return headerHeight + 34 + (carried_ ? rowGap + fields_.height() : 0);
    }

    void setState(const EngineState& state) override {
        if (!carried_) {
            setStatus(config_.cycle == EngineCycle::twoStroke ? utf8("A two-stroke has no cam to switch.")
                                                              : utf8("No VTEC cams fitted."));
            return;
        }
        const auto high = std::popcount(state.vtecHighCylinderMask);
        setStatus(!state.vtecEnabled ? utf8("Off: the low cam only.")
                  : high > 0 ? utf8("High cam on ") + juce::String(high) + utf8(" cylinders.")
                             : utf8("Low cam; the high cam above ") + rpmText(state.vtecEngageRpm) + utf8("."));
    }

    void setLocked(bool locked) override {
        TableSection::setLocked(locked);
        fitButton_.setEnabled(!locked);
        switch_.setEnabled(!locked);
    }

    void resized() override {
        auto area = body();
        auto row = area.removeFromTop(34);
        if (carried_) switch_.setBounds(row.removeFromLeft(switch_.idealWidth()));
        else fitButton_.setBounds(row.removeFromLeft(fitButton_.idealWidth()));
        area.removeFromTop(rowGap);
        if (carried_) fields_.layout(area);
    }

private:
    void tablesChanged(const calibration::CalibrationSnapshot& snapshot) override {
        const auto setting = snapshot.scalar(calibration::keys::vtecEnabled);
        const auto on = setting.value_or(0.0) >= 0.5;
        switch_.setButtonText(on ? utf8("VTEC on") : utf8("VTEC off"));
        switch_.setEngaged(on);
        switch_.setVisible(setting.has_value());
        fitButton_.setVisible(!setting.has_value() && config_.cycle != EngineCycle::twoStroke);
    }

    EquipmentWindow::Callbacks& callbacks_;
    ui::ActionButton fitButton_ { utf8("Fit VTEC cams"), ui::ActionButton::Style::standard };
    ui::ActionButton switch_ { utf8("VTEC off"), ui::ActionButton::Style::compact };
};

class AntiLagSection final : public TableSection {
public:
    AntiLagSection()
        : TableSection(utf8("ANTI-LAG"),
                       {
                           { "Engine braking kept", " %", 0.0, 100.0, 5.0, 0,
                             "The share of the engine's own braking it keeps when you lift off: 100 % brakes as "
                             "without anti-lag and holds little boost, 0 % holds the engine's speed and the most "
                             "boost. It never makes the engine gain speed." },
                           { "Aggressiveness", " %", 0.0, 100.0, 5.0, 0,
                             "How much of its air it may open to keep the turbo spinning." },
                       },
                       { calibration::keys::antiLagBrakingKept, calibration::keys::antiLagAggressiveness },
                       { 1.0, 1.0 }),
          advanced_(*this, {
              { "Works above", " rpm", 0.0, calibration::ecuLimits::maximumRevLimitRpm, 100.0, 0,
                "It works only above this engine speed." },
              { "Lets go above", " rpm", 0.0, calibration::ecuLimits::maximumRevLimitRpm, 100.0, 0,
                "Above this engine speed it lets go (0: the rev limiter only)." },
              { "Air", " %", 0.0, 100.0, 1.0, 0, "The most throttle it opens with your foot off." },
              { "Spark", " deg ATDC", 0.0, calibration::ecuLimits::maximumAntiLagSparkAtdcDegrees, 1.0, 0,
                "How late the spark fires, after top dead centre." },
              { "Mixture", " lambda", calibration::ecuLimits::minimumAntiLagLambda,
                calibration::ecuLimits::maximumAntiLagLambda, 0.01, 2, "Its mixture: under 1 is rich." },
              { "Spark cut", " %", 0.0, 100.0, 1.0, 0,
                "The share of firings whose spark is cut, fuel kept: that charge goes to the exhaust." },
              { "Maximum time", " s", 0.0, calibration::ecuLimits::maximumAntiLagSeconds, 0.5, 1,
                "It lets go after this long on one lift, until you press the throttle again (0: no limit)." },
              { "Exhaust limit", " C", 0.0, calibration::ecuLimits::maximumAntiLagExhaustLimitC,
                10.0, 0, "Above this exhaust temperature it lets go (0: no limit)." },
          }),
          mode_(juce::StringArray { "Rally", "Launch" }) {
        advancedKeys_ = { calibration::keys::antiLagMinimumRpm, calibration::keys::antiLagMaximumRpm,
                          calibration::keys::antiLagAir, calibration::keys::antiLagRetard,
                          calibration::keys::antiLagLambda, calibration::keys::antiLagCut,
                          calibration::keys::antiLagMaximumSeconds, calibration::keys::antiLagExhaustLimit };
        advanced_.onEdited = [this](std::size_t index) {
            if (locked_ || !store_) return;
            if (!EquipmentWindow::publishScalar(*store_, advancedKeys_[index], advanced_.value(index)))
                readTables(true);
        };
        switch_.setTooltip(utf8("Switches anti-lag on or off (also in Drive)."));
        switch_.onClick = [this] {
            if (locked_ || !store_) return;
            const auto on = table(calibration::keys::antiLagEnabled).value_or(0.0) >= 0.5;
            (void)EquipmentWindow::publishScalar(*store_, calibration::keys::antiLagEnabled, on ? 0.0 : 1.0);
            readTables(true);
        };
        mode_.setOptionTooltip(0, utf8("Rally: on every lift above its speed, between gears and into corners."));
        mode_.setOptionTooltip(1, utf8("Launch: only while launch control holds the engine on the line, foot "
                                       "down, to build boost before the start."));
        mode_.onChange = [this](int index) {
            if (locked_ || !store_) return;
            (void)EquipmentWindow::publishScalar(*store_, calibration::keys::antiLagMode, index == 1 ? 1.0 : 0.0);
            readTables(true);
        };
        advancedButton_.setTooltip(utf8("Its speeds, air, spark, mixture, cut and exits."));
        advancedButton_.onClick = [this] {
            advancedOpen_ = !advancedOpen_;
            advancedButton_.setButtonText(advancedOpen_ ? utf8("Hide advanced") : utf8("Advanced..."));
            advanced_.setVisible(carried_ && advancedOpen_);
            layoutChanged();
        };
        addChildComponent(switch_);
        addChildComponent(mode_);
        addChildComponent(advancedButton_);
    }

    [[nodiscard]] int preferredHeight() const override {
        return headerHeight + (carried_ ? 34 + rowGap + fields_.height() + 34 + rowGap
                                            + (advancedOpen_ ? advanced_.height() : 0) : 0);
    }

    void setState(const EngineState& state) override {
        if (!carried_) {
            setStatus(utf8("Anti-lag needs a turbocharged petrol engine."));
            return;
        }
        setStatus(describeAntiLag(state, table(calibration::keys::antiLagMinimumRpm).value_or(0.0),
                                  table(calibration::keys::antiLagMaximumRpm).value_or(0.0)));
    }

    void setLocked(bool locked) override {
        TableSection::setLocked(locked);
        advanced_.setEnabled(!locked);
        switch_.setEnabled(!locked);
        mode_.setEnabled(!locked);
    }

    void resized() override {
        auto area = body();
        if (!carried_) return;
        auto row = area.removeFromTop(34);
        switch_.setBounds(row.removeFromLeft(switch_.idealWidth()));
        row.removeFromLeft(12);
        mode_.setBounds(row.removeFromLeft(220));
        area.removeFromTop(rowGap);
        fields_.layout(area);
        auto line = area.removeFromTop(34);
        advancedButton_.setBounds(line.removeFromLeft(advancedButton_.idealWidth()));
        area.removeFromTop(rowGap);
        if (advancedOpen_) advanced_.layout(area);
    }

private:
    void tablesChanged(const calibration::CalibrationSnapshot& snapshot) override {
        const auto setting = snapshot.scalar(calibration::keys::antiLagEnabled);
        const auto on = setting.value_or(0.0) >= 0.5;
        switch_.setButtonText(on ? utf8("Anti-lag on") : utf8("Anti-lag off"));
        switch_.setEngaged(on);
        mode_.setSelected(snapshot.scalar(calibration::keys::antiLagMode).value_or(0.0) >= 0.5 ? 1 : 0);
        std::vector<double> values;
        for (const auto key : advancedKeys_) values.push_back(snapshot.scalar(key).value_or(0.0));
        advanced_.setValues(values);
        const auto carried = setting.has_value();
        switch_.setVisible(carried);
        mode_.setVisible(carried);
        advancedButton_.setVisible(carried);
        advanced_.setVisible(carried && advancedOpen_);
    }

    FieldGrid advanced_;
    std::vector<std::string_view> advancedKeys_;
    ui::ActionButton switch_ { utf8("Anti-lag off"), ui::ActionButton::Style::compact };
    ui::SegmentedControl mode_;
    ui::ActionButton advancedButton_ { utf8("Advanced..."), ui::ActionButton::Style::compact };
    bool advancedOpen_ { false };
};

class LaunchSection final : public TableSection {
public:
    LaunchSection()
        : TableSection(utf8("LAUNCH CONTROL"),
                       { { "Launch speed", " rpm", 0.0, calibration::ecuLimits::maximumRevLimitRpm, 100.0, 0,
                           "Standing in gear under throttle, the engine is held at this speed until the clutch "
                           "bites: 0 is off." } },
                       { calibration::keys::launchLimit }, { 1.0 }) {}

    [[nodiscard]] int preferredHeight() const override {
        return headerHeight + (carried_ ? fields_.height() : 0);
    }

    void setState(const EngineState& state) override {
        if (!carried_) {
            setStatus(utf8("This engine's tables have no launch control."));
            return;
        }
        const auto rpm = table(calibration::keys::launchLimit).value_or(0.0);
        const auto holding = (state.activeLimits & static_cast<std::uint32_t>(ActiveLimit::launchControl)) != 0U;
        setStatus(rpm <= 0.0 ? utf8("Off: set a launch speed to fit it.")
                  : holding  ? utf8("Holding ") + rpmText(rpm) + utf8(".")
                             : utf8("Holds ") + rpmText(rpm) + utf8(" standing in gear under throttle."));
    }

    void resized() override {
        auto area = body();
        if (carried_) fields_.layout(area);
    }
};

} // namespace

// --------------------------------------------------------------------------

juce::String describeAntiLag(const EngineState& state, double minimumRpm, double maximumRpm) {
    if (!state.antiLagArmed) return utf8("Off: switch it on here or in Drive.");
    if (state.antiLagActive)
        return state.antiLagTargetTorqueNm < 0.0 || state.antiLagAuthority < 1.0
            // The lower half of its authority opens the air, the upper half
            // gives cut firings back with all of it open.
            ? utf8("Working: ") + juce::String(std::min(1.0, 2.0 * state.antiLagAuthority) * 100.0, 0)
                + utf8(" % of its air, holding the engine to ") + juce::String(state.antiLagTargetTorqueNm, 0)
                + utf8(" N.m.")
            : utf8("Working on the line.");
    const auto has = [&state](AntiLagBlocker blocker) {
        return (state.antiLagBlockers & static_cast<std::uint32_t>(blocker)) != 0U;
    };
    if (has(AntiLagBlocker::ignition)) return utf8("On: waiting for the engine to run.");
    if (has(AntiLagBlocker::timeUp)) return utf8("Let go after its maximum time: press the throttle to re-arm it.");
    if (has(AntiLagBlocker::exhaustTooHot)) return utf8("Let go: the exhaust is over its temperature limit.");
    if (has(AntiLagBlocker::protection)) return utf8("Let go: the rev limiter or the overboost cut holds the engine.");
    if (has(AntiLagBlocker::decoupled)) return utf8("Let go: neutral, or the clutch held out.");
    if (has(AntiLagBlocker::notLaunching))
        return utf8("Launch mode: works while launch control holds the engine on the line, foot down.");
    if (has(AntiLagBlocker::aboveMaximumRpm)) return utf8("On: lets go above ") + rpmText(maximumRpm) + utf8(".");
    if (has(AntiLagBlocker::throttle))
        return utf8("On: works when you lift off, above ") + rpmText(minimumRpm) + utf8(".");
    if (has(AntiLagBlocker::belowMinimumRpm)) return utf8("On: works above ") + rpmText(minimumRpm) + utf8(".");
    return utf8("On.");
}

juce::String describeNitrousBlockers(std::uint32_t blockers, const NitrousConfig& kit) {
    if (blockers == 0U) return {};
    const auto has = [blockers](NitrousBlocker blocker) {
        return (blockers & static_cast<std::uint32_t>(blocker)) != 0U;
    };
    if (has(NitrousBlocker::notArmed)) return utf8("Disarmed: arm it in Drive (Nitrous).");
    if (has(NitrousBlocker::bottleEmpty)) return utf8("The bottle is empty: refill it.");
    juce::StringArray waits;
    if (has(NitrousBlocker::ignitionOff)) waits.add(utf8("the ignition"));
    if (has(NitrousBlocker::pushButton)) waits.add(utf8("its key (=) held"));
    if (has(NitrousBlocker::throttle))
        waits.add(juce::String(kit.minimumThrottle * 100.0, 0) + utf8(" % throttle"));
    if (has(NitrousBlocker::belowMinimumRpm)) waits.add(utf8("over ") + rpmText(kit.minimumRpm));
    if (has(NitrousBlocker::aboveMaximumRpm)) waits.add(utf8("under ") + rpmText(kit.maximumRpm));
    return utf8("Armed, waiting for ") + waits.joinIntoString(utf8(", ")) + utf8(".");
}

class EquipmentWindow::Content final : public juce::Component {
public:
    Content(const EngineConfig& config, std::shared_ptr<calibration::CalibrationStore> store, Callbacks callbacks)
        : callbacks_(std::move(callbacks)), nitrous_(callbacks_), valve_(callbacks_), vtec_(callbacks_) {
        setOpaque(true);
        for (auto* section : sections()) {
            section->onLayoutChanged = [this] { layoutSections(); };
            inner_.addAndMakeVisible(*section);
        }
        viewport_.setViewedComponent(&inner_, false);
        viewport_.setScrollBarsShown(true, false);
        addAndMakeVisible(viewport_);
        setEngine(config, std::move(store));
    }

    [[nodiscard]] std::array<Section*, 5> sections() noexcept {
        return { &nitrous_, &antiLag_, &valve_, &vtec_, &launch_ };
    }

    void setEngine(const EngineConfig& config, std::shared_ptr<calibration::CalibrationStore> store) {
        for (auto* section : sections()) section->setEngine(config, store);
        layoutSections();
    }
    void setState(const EngineState& state) {
        for (auto* section : sections()) section->setState(state);
    }
    void setValveMode(ExhaustValveMode mode) { valve_.setMode(mode); }
    void setLocked(bool locked) {
        for (auto* section : sections()) section->setLocked(locked);
    }

    void paint(juce::Graphics& g) override { g.fillAll(ui::colours::panel); }
    void resized() override {
        viewport_.setBounds(getLocalBounds());
        layoutSections();
    }

private:
    void layoutSections() {
        const auto width = std::max(200, viewport_.getMaximumVisibleWidth());
        auto y = 20;
        for (auto* section : sections()) {
            section->setBounds(20, y, width - 40, section->preferredHeight());
            y += section->preferredHeight() + sectionGap;
        }
        inner_.setSize(width, y);
    }

    Callbacks callbacks_;
    juce::Viewport viewport_;
    juce::Component inner_;
    NitrousSection nitrous_;
    ValveSection valve_;
    VtecSection vtec_;
    AntiLagSection antiLag_;
    LaunchSection launch_;
};

EquipmentWindow::EquipmentWindow(const EngineConfig& config, std::shared_ptr<calibration::CalibrationStore> store,
                                 Callbacks callbacks)
    : juce::DocumentWindow(utf8("Equipment"), ui::colours::panel, juce::DocumentWindow::closeButton) {
    content_ = new Content(config, std::move(store), std::move(callbacks));
    setContentOwned(content_, false);
    setUsingNativeTitleBar(true);
    setResizable(true, false);
    setResizeLimits(560, 300, 2000, 2000);
    centreWithSize(760, 720);
}

EquipmentWindow::~EquipmentWindow() = default;

void EquipmentWindow::closeButtonPressed() { setVisible(false); }

void EquipmentWindow::setEngine(const EngineConfig& config, std::shared_ptr<calibration::CalibrationStore> store) {
    if (content_ != nullptr) content_->setEngine(config, std::move(store));
}

void EquipmentWindow::setState(const EngineState& state) {
    if (content_ != nullptr) content_->setState(state);
}

void EquipmentWindow::setValveMode(ExhaustValveMode mode) {
    if (content_ != nullptr) content_->setValveMode(mode);
}

void EquipmentWindow::setLocked(bool locked) {
    if (content_ != nullptr) content_->setLocked(locked);
}

juce::Component* EquipmentWindow::content() const noexcept { return content_; }

bool EquipmentWindow::publishScalar(calibration::CalibrationStore& store, std::string_view key, double value) {
    const auto snapshot = store.snapshot();
    auto draft = calibration::makeDraft(*snapshot);
    const auto* found = draft.find(key);
    if (found == nullptr) return false;
    auto entry = *found;
    auto* scalar = std::get_if<calibration::ScalarCalibration>(&entry);
    if (scalar == nullptr) return false;
    if (scalar->value == value) return true;
    scalar->value = value;
    draft.set(std::move(entry));
    return store.publish(draft, { snapshot->revision(), "equipment" }).published;
}

} // namespace enginelab
