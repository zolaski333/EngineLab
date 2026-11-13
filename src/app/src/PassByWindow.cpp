#include <enginelab/app/PassByWindow.hpp>

#include <enginelab/app/Theme.hpp>
#include <enginelab/app/Widgets.hpp>

#include <algorithm>
#include <cmath>
#include <utility>

namespace enginelab {
namespace {

using ui::utf8;

constexpr int rowHeight = 30;
constexpr int rowGap = 8;
constexpr int labelWidth = 150;
constexpr int titleHeight = 18;
constexpr int stripHeight = 150;

[[nodiscard]] juce::String metres(double value) { return ui::fixed(value, 0) + " m"; }

[[nodiscard]] juce::String gearName(int engagedGear) {
    return engagedGear < 0 ? juce::String("N") : juce::String(engagedGear + 1);
}

/** The lane seen from above: the start, the microphone, the end and the car. */
class Strip final : public juce::Component {
public:
    void set(const PassByRunSettings& settings, const PassByMicrophoneChoice& microphone, const EngineState& state) {
        settings_ = settings;
        microphone_ = microphone;
        state_ = state;
        repaint();
    }

    void paint(juce::Graphics& g) override {
        auto area = getLocalBounds().toFloat();
        ui::fillCard(g, area, 9.0F, ui::colours::panelRaised, ui::colours::line);
        area = area.reduced(14.0F, 10.0F);
        auto readout = area.removeFromBottom(22.0F);
        const auto first = -settings_.startDistanceM;
        const auto last = settings_.endDistanceM;
        const auto span = std::max(1.0, last - first);
        const auto x = [&](double along) {
            return area.getX() + static_cast<float>((std::clamp(along, first, last) - first) / span) * area.getWidth();
        };
        // The lane, its start and end lines.
        const auto laneY = area.getY() + area.getHeight() * 0.38F;
        const auto laneHeight = 26.0F;
        g.setColour(ui::colours::grid);
        g.fillRoundedRectangle(juce::Rectangle<float>(area.getX(), laneY - laneHeight * 0.5F, area.getWidth(), laneHeight),
                               4.0F);
        g.setColour(ui::colours::lineStrong);
        const float dashes[] { 8.0F, 8.0F };
        g.drawDashedLine(juce::Line<float>(area.getX(), laneY, area.getRight(), laneY), dashes, 2, 1.0F);
        g.setFont(ui::uiFont(12.0F));
        const auto marker = [&](double along, const juce::String& text, juce::Colour colour) {
            const auto px = x(along);
            g.setColour(colour);
            g.drawLine(px, laneY - laneHeight * 0.5F - 6.0F, px, laneY + laneHeight * 0.5F + 6.0F, 2.0F);
            g.drawFittedText(text, juce::Rectangle<int>(static_cast<int>(px) - 60,
                static_cast<int>(laneY - laneHeight * 0.5F - 26.0F), 120, 16), juce::Justification::centred, 1);
        };
        marker(first, "Start  -" + metres(settings_.startDistanceM), ui::colours::muted);
        marker(last, "End  +" + metres(settings_.endDistanceM), ui::colours::muted);
        if (settings_.program == PassByProgram::acceleration && settings_.fullThrottleBeforeM > 0.0)
            marker(-settings_.fullThrottleBeforeM, {}, ui::colours::warn);
        // The microphone, off the lane.
        const auto micX = x(0.0);
        const auto micY = laneY + laneHeight * 0.5F + 26.0F;
        g.setColour(ui::colours::intake);
        g.drawLine(micX, laneY + laneHeight * 0.5F + 2.0F, micX, micY - 6.0F, 1.0F);
        g.fillEllipse(micX - 6.0F, micY - 6.0F, 12.0F, 12.0F);
        g.drawFittedText("Microphone, " + ui::fixed(microphone_.laneDistanceM, 1) + " m off the lane",
            juce::Rectangle<int>(static_cast<int>(micX) - 120, static_cast<int>(micY) + 8, 240, 16),
            juce::Justification::centred, 1);
        // The car.
        const auto phase = static_cast<PassByPhase>(state_.passByPhase);
        if (phase != PassByPhase::off) {
            const auto carX = x(state_.passByPositionM);
            g.setColour(ui::colours::accent);
            g.fillRoundedRectangle(juce::Rectangle<float>(carX - 9.0F, laneY - 7.0F, 18.0F, 14.0F), 3.0F);
        }
        // Where, how fast, in which gear, since when.
        g.setColour(ui::colours::text);
        g.setFont(ui::monoFont(13.0F, false));
        juce::String text;
        if (phase == PassByPhase::off) {
            text = "Car not on the lane";
        } else {
            const auto along = state_.passByPositionM;
            text = std::abs(along) < 0.5 ? juce::String("At the microphone")
                 : along < 0.0 ? metres(-along) + " before the microphone"
                               : metres(along) + " past the microphone";
            text << "   " << ui::fixed(state_.vehicleSpeedMps * 3.6, 0) << " km/h   gear "
                 << gearName(state_.gear) << "   " << ui::fixed(state_.rpm, 0) << " rpm   "
                 << ui::fixed(state_.passBySeconds, 1) << " s";
        }
        g.drawFittedText(text, readout.toNearestInt(), juce::Justification::centredLeft, 1);
    }

private:
    PassByRunSettings settings_ {};
    PassByMicrophoneChoice microphone_ {};
    EngineState state_ {};
};

} // namespace

juce::String describePassBy(const EngineState& state, const PassByRunSettings& settings) {
    switch (static_cast<PassByPhase>(state.passByPhase)) {
    case PassByPhase::off: return utf8("Ready: set the run, then Start.");
    case PassByPhase::stopping: return utf8("Stopping the car and starting the engine...");
    case PassByPhase::staging:
        return utf8("At the start, ") + metres(settings.startDistanceM) + utf8(" before the microphone: first gear, "
            "on the brake");
    case PassByPhase::runUp:
        return utf8("Getting to ") + ui::fixed(settings.approachSpeedKmh, 0) + utf8(" km/h before the start");
    case PassByPhase::measuring: return utf8("Driving past the microphone");
    case PassByPhase::finishing: return utf8("Past the end: braking, the sound fading out");
    case PassByPhase::finished:
        return utf8("Finished: ") + ui::fixed(state.passBySeconds, 1) + utf8(" s from the start to the end. "
            "Restart runs it again; Stop gives back the pedals and the sound.");
    }
    return {};
}

class PassByWindow::Content final : public juce::Component {
public:
    Content(const EngineConfig& config, Callbacks callbacks) : callbacks_(std::move(callbacks)) {
        const auto field = [this](juce::Label& label, juce::Slider& slider, const char* name, const char* unit,
                                  double minimum, double maximum, double step, double value, const char* tooltip) {
            label.setText(utf8(name), juce::dontSendNotification);
            label.setColour(juce::Label::textColourId, ui::colours::text);
            label.setTooltip(utf8(tooltip));
            slider.setSliderStyle(juce::Slider::IncDecButtons);
            slider.setTextBoxStyle(juce::Slider::TextBoxLeft, false, 110, rowHeight);
            slider.setIncDecButtonsMode(juce::Slider::incDecButtonsDraggable_AutoDirection);
            slider.setRange(minimum, maximum, step);
            slider.setTextValueSuffix(utf8(unit));
            slider.setValue(value, juce::dontSendNotification);
            slider.setTooltip(utf8(tooltip));
            addAndMakeVisible(label);
            addAndMakeVisible(slider);
        };
        const PassByRunSettings run;
        const PassByMicrophoneChoice microphone;
        field(startLabel_, start_, "Start before mic", " m", PassByRunSettings::minimumDistanceM,
              PassByRunSettings::maximumDistanceM, 10.0, run.startDistanceM,
              "Where the car waits, before the microphone");
        field(endLabel_, end_, "End past mic", " m", PassByRunSettings::minimumDistanceM,
              PassByRunSettings::maximumDistanceM, 10.0, run.endDistanceM,
              "Where the run ends past the microphone: the car lifts, brakes and its sound fades out");
        field(speedLabel_, speed_, "Speed", " km/h", PassByRunSettings::minimumApproachKmh,
              PassByRunSettings::maximumApproachKmh, 5.0, run.approachSpeedKmh,
              "The constant programs' speed, held by the cruise control from before the start");
        field(throttleLabel_, throttlePoint_, "Full throttle at", " m before", 0.0, 500.0, 1.0,
              run.fullThrottleBeforeM, "Acceleration: full throttle from this far before the microphone "
              "(ISO 362's line, 10 m)");
        field(gearLabel_, gear_, "Gears", "", 0.0, 6.0, 1.0, 0.0,
              "Auto: up through the gears; a number: up to that gear and held there");
        gear_.textFromValueFunction = [](double value) {
            return value < 0.5 ? juce::String("Auto") : "Up to " + juce::String(static_cast<int>(value));
        };
        gear_.valueFromTextFunction = [](const juce::String& text) {
            return text.trim().equalsIgnoreCase("auto") ? 0.0 : static_cast<double>(text.retainCharacters("0123456789").getIntValue());
        };
        gear_.updateText();
        field(laneLabel_, lane_, "Off the lane", " m", 1.0, 50.0, 0.5, microphone.laneDistanceM,
              "The microphone from the lane's centre line (ISO 362: 7.5 m)");
        field(heightLabel_, height_, "Microphone height", " m", 0.1, 10.0, 0.1, microphone.microphoneHeightM,
              "Above the ground (ISO 362: 1.2 m)");
        for (auto* slider : { &start_, &end_, &speed_, &throttlePoint_, &gear_ })
            slider->onValueChange = [this] { refreshStrip(); };
        for (auto* slider : { &lane_, &height_ }) slider->onValueChange = [this] { microphoneEdited(); };

        program_.setSelected(0);
        program_.setOptionTooltip(0, utf8("From a standstill at the start, full throttle through the gears "
                                          "(on launch control when the tables carry it)"));
        program_.setOptionTooltip(1, utf8("At the speed, then full throttle from the set point before the microphone"));
        program_.setOptionTooltip(2, utf8("At the speed all along"));
        program_.setOptionTooltip(3, utf8("Full throttle up to the microphone, lifted there, coasting in gear after it"));
        program_.onChange = [this](int) { programChanged(); };
        addAndMakeVisible(program_);
        ground_.setSelected(static_cast<int>(microphone.ground));
        ground_.setTooltip(utf8("The ground between the lane and the microphone: its reflection (Delany-Bazley, "
                                "a spherical wave) makes the dips and the boost a real pass-by has"));
        ground_.onChange = [this](int) { microphoneEdited(); };
        addAndMakeVisible(ground_);
        level_.setSelected(0);
        level_.setOptionTooltip(0, utf8("What the microphone really gets: a car 500 m away is ~37 dB quieter"));
        level_.setOptionTooltip(1, utf8("The far car's timbre (its delay, air and ground), as loud as at the "
                                        "closest approach"));
        level_.onChange = [this](int) { microphoneEdited(); };
        addAndMakeVisible(level_);
        launch_.setButtonText(utf8("Launch control"));
        launch_.setToggleState(run.launchControl, juce::dontSendNotification);
        launch_.setColour(juce::ToggleButton::textColourId, ui::colours::text);
        launch_.setTooltip(utf8("A standing start on launch control when the tables carry it (Equipment)"));
        addAndMakeVisible(launch_);
        for (auto* label : { &programLabel_, &groundLabel_, &levelLabel_ }) {
            label->setColour(juce::Label::textColourId, ui::colours::text);
            addAndMakeVisible(*label);
        }
        programLabel_.setText(utf8("Program"), juce::dontSendNotification);
        groundLabel_.setText(utf8("Ground"), juce::dontSendNotification);
        levelLabel_.setText(utf8("Level"), juce::dontSendNotification);

        startButton_.setTooltip(utf8("The driver stops the car, waits at the start, drives the program past the "
                                     "microphone to the end and brakes. Restart begins again."));
        startButton_.onClick = [this] {
            if (callbacks_.start) callbacks_.start(runSettings(), recordButton_.engaged());
        };
        stopButton_.setTooltip(utf8("The driver lets go: the pedals are yours again"));
        stopButton_.onClick = [this] { if (callbacks_.stop) callbacks_.stop(); };
        recordButton_.setTooltip(utf8("Saves the next run, as heard, to a WAV file in Documents/EngineLab recordings"));
        recordButton_.onClick = [this] { recordButton_.setEngaged(!recordButton_.engaged()); };
        for (auto* button : { &startButton_, &stopButton_, &recordButton_ }) addAndMakeVisible(*button);
        for (auto* label : { &status_, &recording_ }) {
            label->setColour(juce::Label::textColourId, ui::colours::muted);
            addAndMakeVisible(*label);
        }
        addAndMakeVisible(strip_);
        setEngine(config);
        programChanged();
        setState(EngineState {});
        microphoneEdited();
        setSize(760, 600);
    }

    void setEngine(const EngineConfig& config) {
        const auto gears = std::max<std::size_t>(1, config.transmission.gearRatios.size());
        if (static_cast<std::size_t>(gear_.getMaximum()) != gears) {
            gear_.setRange(0.0, static_cast<double>(gears), 1.0);
            gear_.updateText();
        }
    }

    void setState(const EngineState& state) {
        state_ = state;
        const auto phase = static_cast<PassByPhase>(state.passByPhase);
        const auto running = phase != PassByPhase::off && phase != PassByPhase::finished;
        startButton_.setButtonText(running ? utf8("Restart") : utf8("Start"));
        stopButton_.setEnabled(phase != PassByPhase::off);
        const auto text = locked_ ? utf8("The dyno is running: no pass-by meanwhile.")
                                  : describePassBy(state, runSettings());
        if (status_.getText() != text) status_.setText(text, juce::dontSendNotification);
        refreshStrip();
    }

    void setRecordingStatus(const juce::String& text) {
        if (recording_.getText() != text) recording_.setText(text, juce::dontSendNotification);
    }

    void setLocked(bool locked) {
        if (locked == locked_) return;
        locked_ = locked;
        startButton_.setEnabled(!locked);
        setState(state_);
    }

    [[nodiscard]] PassByRunSettings runSettings() const {
        PassByRunSettings settings;
        settings.program = static_cast<PassByProgram>(std::clamp(program_.selected(), 0, 3));
        settings.startDistanceM = start_.getValue();
        settings.endDistanceM = end_.getValue();
        settings.approachSpeedKmh = speed_.getValue();
        settings.fullThrottleBeforeM = throttlePoint_.getValue();
        settings.gear = static_cast<int>(std::lround(gear_.getValue()));
        settings.launchControl = launch_.getToggleState();
        return settings;
    }

    [[nodiscard]] PassByMicrophoneChoice microphone() const {
        PassByMicrophoneChoice choice;
        choice.laneDistanceM = lane_.getValue();
        choice.microphoneHeightM = height_.getValue();
        choice.ground = static_cast<PassByGround>(std::clamp(ground_.selected(), 0, 4));
        choice.normalisedLevel = level_.selected() == 1;
        return choice;
    }

    void paint(juce::Graphics& g) override {
        g.fillAll(ui::colours::panel);
        ui::drawSectionLabel(g, runTitle_.toFloat(), utf8("Run"));
        ui::drawSectionLabel(g, microphoneTitle_.toFloat(), utf8("Microphone"));
    }

    void resized() override {
        auto area = getLocalBounds().reduced(18, 14);
        strip_.setBounds(area.removeFromTop(stripHeight));
        area.removeFromTop(rowGap);
        status_.setBounds(area.removeFromTop(22));
        recording_.setBounds(area.removeFromTop(22));
        area.removeFromTop(rowGap);
        auto buttons = area.removeFromTop(34);
        for (auto* button : { &startButton_, &stopButton_, &recordButton_ }) {
            const auto width = std::max(110, button->idealWidth());
            button->setBounds(buttons.removeFromLeft(width));
            buttons.removeFromLeft(10);
        }
        area.removeFromTop(16);
        const auto line = [&](juce::Label& label, juce::Component& control) {
            auto row = area.removeFromTop(rowHeight);
            label.setBounds(row.removeFromLeft(labelWidth));
            control.setBounds(row);
            area.removeFromTop(rowGap);
        };
        const auto pair = [&](juce::Label& leftLabel, juce::Component& left, juce::Label& rightLabel,
                              juce::Component& right) {
            auto row = area.removeFromTop(rowHeight);
            const auto half = (row.getWidth() - 16) / 2;
            auto first = row.removeFromLeft(half);
            row.removeFromLeft(16);
            leftLabel.setBounds(first.removeFromLeft(labelWidth));
            left.setBounds(first);
            rightLabel.setBounds(row.removeFromLeft(labelWidth));
            right.setBounds(row);
            area.removeFromTop(rowGap);
        };
        runTitle_ = area.removeFromTop(titleHeight);
        area.removeFromTop(4);
        line(programLabel_, program_);
        pair(startLabel_, start_, endLabel_, end_);
        pair(speedLabel_, speed_, throttleLabel_, throttlePoint_);
        {
            auto row = area.removeFromTop(rowHeight);
            const auto half = (row.getWidth() - 16) / 2;
            auto first = row.removeFromLeft(half);
            row.removeFromLeft(16);
            gearLabel_.setBounds(first.removeFromLeft(labelWidth));
            gear_.setBounds(first);
            launch_.setBounds(row);
            area.removeFromTop(rowGap);
        }
        area.removeFromTop(10);
        microphoneTitle_ = area.removeFromTop(titleHeight);
        area.removeFromTop(4);
        pair(laneLabel_, lane_, heightLabel_, height_);
        line(groundLabel_, ground_);
        line(levelLabel_, level_);
    }

private:
    void programChanged() {
        const auto program = static_cast<PassByProgram>(std::clamp(program_.selected(), 0, 3));
        speed_.setEnabled(program == PassByProgram::acceleration || program == PassByProgram::constantSpeed);
        throttlePoint_.setEnabled(program == PassByProgram::acceleration);
        launch_.setEnabled(program == PassByProgram::standingStart);
        refreshStrip();
    }

    void microphoneEdited() {
        if (callbacks_.setMicrophone) callbacks_.setMicrophone(microphone());
        refreshStrip();
    }

    void refreshStrip() { strip_.set(runSettings(), microphone(), state_); }

    Callbacks callbacks_;
    EngineState state_ {};
    bool locked_ { false };
    Strip strip_;
    juce::Label status_;
    juce::Label recording_;
    ui::ActionButton startButton_ { "Start", ui::ActionButton::Style::primary };
    ui::ActionButton stopButton_ { "Stop" };
    ui::ActionButton recordButton_ { "Record WAV" };
    juce::Rectangle<int> runTitle_;
    juce::Rectangle<int> microphoneTitle_;
    juce::Label programLabel_;
    ui::SegmentedControl program_ { juce::StringArray { "Standing start", "Acceleration", "Constant speed",
                                                        "Lift at the mic" } };
    juce::Label startLabel_;
    juce::Slider start_;
    juce::Label endLabel_;
    juce::Slider end_;
    juce::Label speedLabel_;
    juce::Slider speed_;
    juce::Label throttleLabel_;
    juce::Slider throttlePoint_;
    juce::Label gearLabel_;
    juce::Slider gear_;
    juce::ToggleButton launch_;
    juce::Label laneLabel_;
    juce::Slider lane_;
    juce::Label heightLabel_;
    juce::Slider height_;
    juce::Label groundLabel_;
    ui::SegmentedControl ground_ { juce::StringArray { "None", "Asphalt", "Gravel", "Grass", "Snow" } };
    juce::Label levelLabel_;
    ui::SegmentedControl level_ { juce::StringArray { "Real", "Normalised" } };
};

PassByWindow::PassByWindow(const EngineConfig& config, Callbacks callbacks)
    : juce::DocumentWindow(utf8("Pass-by"), ui::colours::panel, juce::DocumentWindow::closeButton) {
    content_ = new Content(config, std::move(callbacks));
    setContentOwned(content_, true);
    setUsingNativeTitleBar(true);
    setResizable(true, false);
    setResizeLimits(640, 560, 2000, 2000);
    centreWithSize(getWidth(), getHeight());
}

PassByWindow::~PassByWindow() = default;

void PassByWindow::closeButtonPressed() { setVisible(false); }

void PassByWindow::setEngine(const EngineConfig& config) {
    if (content_ != nullptr) content_->setEngine(config);
}

void PassByWindow::setState(const EngineState& state) {
    if (content_ != nullptr) content_->setState(state);
}

void PassByWindow::setRecordingStatus(const juce::String& text) {
    if (content_ != nullptr) content_->setRecordingStatus(text);
}

void PassByWindow::setLocked(bool locked) {
    if (content_ != nullptr) content_->setLocked(locked);
}

PassByRunSettings PassByWindow::runSettings() const {
    return content_ != nullptr ? content_->runSettings() : PassByRunSettings {};
}

PassByMicrophoneChoice PassByWindow::microphone() const {
    return content_ != nullptr ? content_->microphone() : PassByMicrophoneChoice {};
}

juce::Component* PassByWindow::content() const noexcept { return content_; }

} // namespace enginelab
