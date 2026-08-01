#include <enginelab/app/AudioWorkshopWindow.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <cmath>
#include <memory>
#include <string>
#include <string_view>
#include <thread>
#include <utility>

namespace enginelab {
namespace {

[[nodiscard]] juce::String utf8(std::string_view value) {
    return juce::String::fromUTF8(
        value.data(), static_cast<int>(value.size()));
}

[[nodiscard]] std::string fileSlug(std::string_view source) {
    std::string result;
    result.reserve(source.size());
    bool lastWasSeparator = false;
    for (const auto character : source) {
        const auto byte = static_cast<unsigned char>(character);
        if (std::isalnum(byte) != 0) {
            result.push_back(static_cast<char>(std::tolower(byte)));
            lastWasSeparator = false;
        } else if (!result.empty() && !lastWasSeparator) {
            result.push_back('-');
            lastWasSeparator = true;
        }
    }
    while (!result.empty() && result.back() == '-')
        result.pop_back();
    return result.empty() ? "engine" : result;
}

[[nodiscard]] std::filesystem::path fileSystemPath(
    const juce::File& file) {
    const auto text = file.getFullPathName();
    const auto* encoded = text.toRawUTF8();
    return std::filesystem::path(std::u8string(
        reinterpret_cast<const char8_t*>(encoded)));
}

[[nodiscard]] juce::String displayPath(
    const std::filesystem::path& path) {
    const auto encoded = path.u8string();
    return juce::String::fromUTF8(
        reinterpret_cast<const char*>(encoded.data()),
        static_cast<int>(encoded.size()));
}

} // namespace

OfflineAudioMix effectiveAudioWorkshopMix(
    const OfflineAudioMix& baseMix,
    const AudioWorkshopAvailability& availability,
    const AudioWorkshopLayerSwitches& switches) noexcept {
    auto mix = baseMix;
    if (!availability.measuredImpulseResponseAvailable)
        mix.convolution = 0.0;
    if (availability.compiledIntakeTopology)
        mix.lowFrequencyNoise = 0.0;
    if (availability.compiledExhaustTopology) {
        mix.highFrequencyNoise = 0.0;
        mix.combustionGain = 0.0;
    }

    auto anySolo = false;
    for (std::size_t layer = 0;
         layer < switches.solo.size(); ++layer) {
        const auto available =
            layer != 0
            || !availability.compiledExhaustTopology;
        anySolo = anySolo
            || (available && switches.solo[layer]);
    }
    const auto audible = [&](std::size_t layer) {
        const auto available =
            layer != 0
            || !availability.compiledExhaustTopology;
        return available
            && !switches.muted[layer]
            && (!anySolo || switches.solo[layer]);
    };
    if (!audible(0)) mix.combustionGain = 0.0;
    if (!audible(1)) mix.exhaustGain = 0.0;
    if (!audible(2)) mix.intakeGain = 0.0;
    if (!audible(3)) mix.mechanicalGain = 0.0;
    return mix;
}

AudioPhysicsSettings audioPhysicsSettingsFor(
    const EngineConfig& engine) noexcept {
    return {
        engine.combustionCalibration.cycleVariationCoefficientOfVariation,
        engine.combustionCalibration.cycleVariationCorrelation,
        engine.exhaustAfterfire.enabled,
        engine.ignition.limiterKeepsFuel,
        engine.exhaustAfterfire.ignitionTemperatureK,
        engine.exhaustAfterfire.reactionTimeConstantSeconds,
        engine.exhaustAfterfire.reactionEfficiency,
    };
}

void applyAudioPhysicsSettings(
    EngineConfig& engine, const AudioPhysicsSettings& settings) noexcept {
    engine.combustionCalibration.cycleVariationCoefficientOfVariation =
        settings.cycleVariationCoefficientOfVariation;
    engine.combustionCalibration.cycleVariationCorrelation =
        settings.cycleVariationCorrelation;
    engine.exhaustAfterfire.enabled = settings.afterfireEnabled;
    engine.ignition.limiterKeepsFuel = settings.limiterKeepsFuel;
    engine.exhaustAfterfire.ignitionTemperatureK =
        settings.afterfireIgnitionTemperatureK;
    engine.exhaustAfterfire.reactionTimeConstantSeconds =
        settings.afterfireReactionTimeSeconds;
    engine.exhaustAfterfire.reactionEfficiency =
        settings.afterfireEfficiency;
}

AudioPhysicsTelemetry audioPhysicsTelemetryFor(
    const EngineConfig& engine, const EngineState& state) noexcept {
    AudioPhysicsTelemetry telemetry;
    const auto cylinderCount = std::min(
        engine.cylinders.size(), state.cylinderStates.size());
    if (cylinderCount != 0) {
        telemetry.minimumCycleMultiplier =
            state.cylinderStates[0].combustionCycleMultiplier;
        telemetry.maximumCycleMultiplier =
            telemetry.minimumCycleMultiplier;
        for (std::size_t cylinder = 1; cylinder < cylinderCount; ++cylinder) {
            const auto multiplier =
                state.cylinderStates[cylinder].combustionCycleMultiplier;
            telemetry.minimumCycleMultiplier = std::min(
                telemetry.minimumCycleMultiplier, multiplier);
            telemetry.maximumCycleMultiplier = std::max(
                telemetry.maximumCycleMultiplier, multiplier);
        }
    }
    telemetry.afterfireHeatReleaseKw =
        state.exhaustAfterfireHeatReleaseKw;
    telemetry.afterfireFuelBurnMgPerSecond =
        state.exhaustAfterfireFuelBurnMgPerSecond;
    for (const auto& path : engine.exhaustPaths) {
        if (path.network) {
            for (const auto& component : path.network->components) {
                if (component.type == ExhaustComponentType::muffler
                        && component.packingFlowResistivityPaSPerM2 > 0.0
                        && component.packingThicknessMm > 0.0
                        && component.perforatedOpenAreaRatio > 0.0)
                    ++telemetry.porousMufflerCount;
            }
        } else if (path.geometry.mufflerPackingFlowResistivityPaSPerM2 > 0.0
                && path.geometry.mufflerPackingThicknessMm > 0.0
                && path.geometry.mufflerPerforatedOpenAreaRatio > 0.0) {
            ++telemetry.porousMufflerCount;
        }
    }
    return telemetry;
}

class AudioWorkshopWindow::WorkshopContent final
    : public juce::Component,
      private juce::Timer {
public:
    WorkshopContent(
        const EngineConfig& engine,
        std::filesystem::path assetRoot,
        const OfflineAudioMix& mix,
        bool compiledExhaustTopology,
        bool compiledIntakeTopology,
        bool measuredImpulseResponseAvailable,
        MixChangedCallback mixChanged,
        PhysicsApplyCallback physicsApply,
        TelemetryProvider telemetryProvider)
        : engine_(engine),
          assetRoot_(std::move(assetRoot)),
          compiledExhaustTopology_(compiledExhaustTopology),
          compiledIntakeTopology_(compiledIntakeTopology),
          measuredImpulseResponseAvailable_(
              measuredImpulseResponseAvailable),
          mixChanged_(std::move(mixChanged)),
          physicsApply_(std::move(physicsApply)),
          telemetryProvider_(std::move(telemetryProvider)),
          progressBar_(progressValue_) {
        setOpaque(true);

        mixGroup_.setText("MIX TEMPS REEL");
        exportGroup_.setText("RENDU HQ HORS LIGNE");
        physicsGroup_.setText("PHYSIQUE AUDIO  /  PRESSION -> ECHAPPEMENT");
        for (auto* group : { &mixGroup_, &exportGroup_, &physicsGroup_ }) {
            group->setColour(
                juce::GroupComponent::outlineColourId,
                juce::Colour(0xff33423e));
            group->setColour(
                juce::GroupComponent::textColourId,
                juce::Colour(0xff9cc8b7));
            addAndMakeVisible(*group);
        }

        engineLabel_.setFont(
            juce::FontOptions(18.0F, juce::Font::bold));
        engineLabel_.setColour(
            juce::Label::textColourId,
            juce::Colour(0xffe4ece8));
        statusLabel_.setColour(
            juce::Label::textColourId,
            juce::Colour(0xff8da39a));
        statusLabel_.setFont(juce::FontOptions(12.0F));
        statusLabel_.setJustificationType(
            juce::Justification::centredRight);
        addAndMakeVisible(engineLabel_);
        addAndMakeVisible(statusLabel_);

        const std::array<std::string_view, 9> names {
            "Master",
            "Retour IR mesure (max +50 %)",
            "Aigus master",
            "Bruit admission legacy",
            "Bruit echappement legacy",
            "Combustion directe",
            "Echappement",
            "Admission + suralimentation",
            "Structure / mecanique",
        };
        for (std::size_t index = 0;
             index < controlLabels_.size(); ++index) {
            controlLabels_[index].setText(
                utf8(names[index]), juce::dontSendNotification);
            controlLabels_[index].setColour(
                juce::Label::textColourId,
                juce::Colour(0xffaebbb6));
            controlLabels_[index].setFont(
                juce::FontOptions(12.0F));
            addAndMakeVisible(controlLabels_[index]);
        }

        configureSlider(volumeSlider_, 0.0, 2.0, 0.01);
        configureSlider(irSlider_, 0.0, 1.0, 0.01);
        configureSlider(highShelfSlider_, 0.2, 2.5, 0.01);
        configureSlider(lowNoiseSlider_, 0.0, 1.5, 0.01);
        configureSlider(highNoiseSlider_, 0.0, 1.5, 0.01);
        configureSlider(combustionSlider_, 0.0, 2.0, 0.01);
        configureSlider(exhaustSlider_, 0.0, 2.0, 0.01);
        configureSlider(intakeSlider_, 0.0, 2.0, 0.01);
        configureSlider(mechanicalSlider_, 0.0, 2.0, 0.01);

        for (std::size_t index = 0;
             index < muteButtons_.size(); ++index) {
            muteButtons_[index] =
                std::make_unique<juce::TextButton>("MUTE");
            soloButtons_[index] =
                std::make_unique<juce::TextButton>("SOLO");
            muteButtons_[index]->setClickingTogglesState(true);
            soloButtons_[index]->setClickingTogglesState(true);
            muteButtons_[index]->setColour(
                juce::TextButton::buttonOnColourId,
                juce::Colour(0xff9d4736));
            soloButtons_[index]->setColour(
                juce::TextButton::buttonOnColourId,
                juce::Colour(0xff347d68));
            muteButtons_[index]->onClick =
                [this] { publishMix(); };
            soloButtons_[index]->onClick =
                [this] { publishMix(); };
            addAndMakeVisible(*muteButtons_[index]);
            addAndMakeVisible(*soloButtons_[index]);
        }

        resetButton_.onClick = [this] {
            setMixInternal(OfflineAudioMix {});
            for (auto& button : muteButtons_)
                button->setToggleState(
                    false, juce::dontSendNotification);
            for (auto& button : soloButtons_)
                button->setToggleState(
                    false, juce::dontSendNotification);
            publishMix();
        };
        addAndMakeVisible(resetButton_);

        sampleRateSelector_.addItem("48 kHz", 1);
        sampleRateSelector_.addItem("96 kHz", 2);
        sampleRateSelector_.addItem("192 kHz", 3);
        sampleRateSelector_.setSelectedId(
            2, juce::dontSendNotification);
        formatSelector_.addItem("PCM 24 bits", 1);
        formatSelector_.addItem("Float 32 bits", 2);
        formatSelector_.setSelectedId(
            1, juce::dontSendNotification);
        stemsToggle_.setToggleState(
            true, juce::dontSendNotification);
        stemsToggle_.setColour(
            juce::ToggleButton::textColourId,
            juce::Colour(0xffc4d0cb));
        addAndMakeVisible(sampleRateSelector_);
        addAndMakeVisible(formatSelector_);
        addAndMakeVisible(stemsToggle_);

        scenarioButton_.onClick =
            [this] { chooseScenario(); };
        defaultScenarioButton_.onClick = [this] {
            scenarioPath_.clear();
            updateScenarioLabel();
        };
        exportButton_.onClick =
            [this] { chooseExportDirectory(); };
        cancelButton_.onClick = [this] {
            if (exportThread_.joinable()) {
                exportThread_.request_stop();
                statusLabel_.setText(
                    "ANNULATION DEMANDEE...",
                    juce::dontSendNotification);
            }
        };
        revealButton_.onClick = [this] {
            if (!lastOutputDirectory_.empty())
                juce::File(displayPath(
                    lastOutputDirectory_))
                    .revealToUser();
        };
        cancelButton_.setEnabled(false);
        revealButton_.setEnabled(false);
        for (auto* button : {
                 &scenarioButton_, &defaultScenarioButton_,
                 &exportButton_, &cancelButton_,
                 &revealButton_ }) {
            addAndMakeVisible(*button);
        }

        scenarioLabel_.setColour(
            juce::Label::textColourId,
            juce::Colour(0xff91a09a));
        scenarioLabel_.setFont(juce::FontOptions(12.0F));
        scenarioLabel_.setJustificationType(
            juce::Justification::topLeft);
        scenarioLabel_.setMinimumHorizontalScale(0.7F);
        addAndMakeVisible(scenarioLabel_);

        proofLabel_.setColour(
            juce::Label::textColourId,
            juce::Colour(0xff8fb7a8));
        proofLabel_.setFont(juce::FontOptions(12.0F));
        proofLabel_.setJustificationType(
            juce::Justification::topLeft);
        proofLabel_.setText(
            "Le rendu utilise EngineSimulator -> publishAudioFrame -> "
            "RealtimeEngineAudio. Le manifeste enregistre les compteurs "
            "du graphe physique et le format WAV reel.",
            juce::dontSendNotification);
        addAndMakeVisible(proofLabel_);
        addAndMakeVisible(progressBar_);

        const std::array<std::string_view, 5> physicsNames {
            "Variation cycle (COV)", "Correlation cycles",
            "Allumage afterfire (K)", "Reaction afterfire (ms)",
            "Rendement afterfire",
        };
        for (std::size_t index = 0; index < physicsLabels_.size(); ++index) {
            physicsLabels_[index].setText(
                utf8(physicsNames[index]), juce::dontSendNotification);
            physicsLabels_[index].setColour(
                juce::Label::textColourId, juce::Colour(0xffaebbb6));
            physicsLabels_[index].setFont(juce::FontOptions(12.0F));
            addAndMakeVisible(physicsLabels_[index]);
        }
        configureSlider(cycleVariationSlider_, 0.0, 0.20, 0.005);
        configureSlider(cycleCorrelationSlider_, 0.0, 0.98, 0.01);
        configureSlider(afterfireTemperatureSlider_, 500.0, 1'800.0, 10.0);
        configureSlider(afterfireReactionSlider_, 2.0, 50.0, 1.0);
        configureSlider(afterfireEfficiencySlider_, 0.0, 1.0, 0.01);
        cycleVariationSlider_.setTextValueSuffix(" ratio");
        cycleCorrelationSlider_.setTextValueSuffix(" ratio");
        afterfireTemperatureSlider_.setTextValueSuffix(" K");
        afterfireReactionSlider_.setTextValueSuffix(" ms");
        afterfireEfficiencySlider_.setTextValueSuffix(" ratio");

        afterfireToggle_.setColour(
            juce::ToggleButton::textColourId, juce::Colour(0xffc4d0cb));
        wetLimiterToggle_.setColour(
            juce::ToggleButton::textColourId, juce::Colour(0xffc4d0cb));
        addAndMakeVisible(afterfireToggle_);
        addAndMakeVisible(wetLimiterToggle_);
        demoPhysicsButton_.onClick = [this] {
            setPhysicsInternal({ 0.06, 0.55, true, true,
                                 800.0, 0.008, 0.95 });
            applyPhysics();
        };
        bypassPhysicsButton_.onClick = [this] {
            auto settings = physicsSettings();
            settings.cycleVariationCoefficientOfVariation = 0.0;
            settings.afterfireEnabled = false;
            settings.limiterKeepsFuel = false;
            setPhysicsInternal(settings);
            applyPhysics();
        };
        applyPhysicsButton_.onClick = [this] { applyPhysics(); };
        for (auto* button : { &demoPhysicsButton_, &bypassPhysicsButton_,
                              &applyPhysicsButton_ })
            addAndMakeVisible(*button);
        physicsTelemetryLabel_.setColour(
            juce::Label::textColourId, juce::Colour(0xff8fb7a8));
        physicsTelemetryLabel_.setFont(juce::FontOptions(12.0F));
        physicsTelemetryLabel_.setJustificationType(juce::Justification::topLeft);
        addAndMakeVisible(physicsTelemetryLabel_);
        physicsStatusLabel_.setColour(
            juce::Label::textColourId, juce::Colour(0xffd49a72));
        physicsStatusLabel_.setFont(juce::FontOptions(11.5F));
        addAndMakeVisible(physicsStatusLabel_);

        setMixInternal(mix);
        setPhysicsInternal(audioPhysicsSettingsFor(engine_));
        updateEnginePresentation();
        setPhysicsInternal(audioPhysicsSettingsFor(engine_));
        updateScenarioLabel();
        installSliderCallbacks();
        updateAvailability();
        startTimerHz(20);
    }

    ~WorkshopContent() override {
        stopTimer();
        if (exportThread_.joinable()) {
            exportThread_.request_stop();
            exportThread_.join();
        }
        for (auto* slider : sliders())
            slider->onValueChange = nullptr;
        for (auto& button : muteButtons_)
            button->onClick = nullptr;
        for (auto& button : soloButtons_)
            button->onClick = nullptr;
    }

    void setEngine(
        const EngineConfig& engine,
        std::filesystem::path assetRoot,
        bool compiledExhaustTopology,
        bool compiledIntakeTopology,
        bool measuredImpulseResponseAvailable) {
        engine_ = engine;
        assetRoot_ = std::move(assetRoot);
        compiledExhaustTopology_ =
            compiledExhaustTopology;
        compiledIntakeTopology_ =
            compiledIntakeTopology;
        measuredImpulseResponseAvailable_ =
            measuredImpulseResponseAvailable;
        updateEnginePresentation();
        updateAvailability();
        publishMix();
    }

    void setMix(const OfflineAudioMix& mix) {
        setMixInternal(mix);
        publishMix();
    }

    void paint(juce::Graphics& graphics) override {
        graphics.fillAll(juce::Colour(0xff0d1312));
        juce::ColourGradient gradient(
            juce::Colour(0xff17221f), 0.0F, 0.0F,
            juce::Colour(0xff0a0f0e), 0.0F,
            static_cast<float>(getHeight()), false);
        graphics.setGradientFill(gradient);
        graphics.fillRect(getLocalBounds());
    }

    void resized() override {
        auto area = getLocalBounds().reduced(18);
        auto header = area.removeFromTop(44);
        engineLabel_.setBounds(
            header.removeFromLeft(
                static_cast<int>(header.getWidth() * 0.62)));
        statusLabel_.setBounds(header);
        area.removeFromTop(8);

        auto physicsArea = area.removeFromBottom(224);
        area.removeFromBottom(12);
        const auto leftWidth = std::max(
            500, static_cast<int>(
                     static_cast<double>(area.getWidth())
                     * 0.58));
        auto mixArea = area.removeFromLeft(leftWidth);
        area.removeFromLeft(12);
        auto exportArea = area;
        mixGroup_.setBounds(mixArea);
        exportGroup_.setBounds(exportArea);
        physicsGroup_.setBounds(physicsArea);

        auto mixBody = mixArea.reduced(16, 28);
        const auto generalRowHeight = 34;
        for (std::size_t index = 0; index < 5; ++index) {
            auto row = mixBody.removeFromTop(
                generalRowHeight);
            controlLabels_[index].setBounds(
                row.removeFromLeft(190));
            sliders()[index]->setBounds(row);
        }
        mixBody.removeFromTop(8);
        const auto layerRowHeight = 38;
        for (std::size_t layer = 0; layer < 4; ++layer) {
            auto row = mixBody.removeFromTop(layerRowHeight);
            controlLabels_[layer + 5].setBounds(
                row.removeFromLeft(155));
            muteButtons_[layer]->setBounds(
                row.removeFromRight(58).reduced(2, 7));
            soloButtons_[layer]->setBounds(
                row.removeFromRight(58).reduced(2, 7));
            layerSliders()[layer]->setBounds(row);
        }
        resetButton_.setBounds(
            mixBody.removeFromBottom(32)
                .removeFromRight(150));

        auto exportBody = exportArea.reduced(16, 30);
        auto selectors = exportBody.removeFromTop(34);
        sampleRateSelector_.setBounds(
            selectors.removeFromLeft(
                selectors.getWidth() / 2 - 4));
        selectors.removeFromLeft(8);
        formatSelector_.setBounds(selectors);
        exportBody.removeFromTop(8);
        stemsToggle_.setBounds(
            exportBody.removeFromTop(30));
        exportBody.removeFromTop(8);
        auto scenarioButtons =
            exportBody.removeFromTop(34);
        scenarioButton_.setBounds(
            scenarioButtons.removeFromLeft(
                scenarioButtons.getWidth() * 2 / 3 - 4));
        scenarioButtons.removeFromLeft(8);
        defaultScenarioButton_.setBounds(
            scenarioButtons);
        scenarioLabel_.setBounds(
            exportBody.removeFromTop(72));
        exportBody.removeFromTop(8);
        exportButton_.setBounds(
            exportBody.removeFromTop(42));
        exportBody.removeFromTop(6);
        auto jobButtons =
            exportBody.removeFromTop(34);
        cancelButton_.setBounds(
            jobButtons.removeFromLeft(
                jobButtons.getWidth() / 2 - 4));
        jobButtons.removeFromLeft(8);
        revealButton_.setBounds(jobButtons);
        exportBody.removeFromTop(12);
        progressBar_.setBounds(
            exportBody.removeFromTop(24));
        exportBody.removeFromTop(12);
        proofLabel_.setBounds(exportBody);

        auto physicsBody = physicsArea.reduced(16, 28);
        const auto firstWidth = physicsBody.getWidth() * 31 / 100;
        const auto secondWidth = physicsBody.getWidth() * 34 / 100;
        auto firstColumn = physicsBody.removeFromLeft(firstWidth);
        physicsBody.removeFromLeft(12);
        auto secondColumn = physicsBody.removeFromLeft(secondWidth);
        physicsBody.removeFromLeft(12);
        auto thirdColumn = physicsBody;
        const auto physicsRow = [](juce::Rectangle<int>& column,
                                   juce::Label& label, juce::Slider& slider) {
            auto row = column.removeFromTop(35);
            label.setBounds(row.removeFromLeft(145));
            slider.setBounds(row);
        };
        physicsRow(firstColumn, physicsLabels_[0], cycleVariationSlider_);
        physicsRow(firstColumn, physicsLabels_[1], cycleCorrelationSlider_);
        physicsStatusLabel_.setBounds(firstColumn.reduced(0, 5));
        physicsRow(secondColumn, physicsLabels_[2], afterfireTemperatureSlider_);
        physicsRow(secondColumn, physicsLabels_[3], afterfireReactionSlider_);
        physicsRow(secondColumn, physicsLabels_[4], afterfireEfficiencySlider_);
        afterfireToggle_.setBounds(thirdColumn.removeFromTop(27));
        wetLimiterToggle_.setBounds(thirdColumn.removeFromTop(27));
        auto presets = thirdColumn.removeFromTop(32);
        demoPhysicsButton_.setBounds(
            presets.removeFromLeft(presets.getWidth() / 2 - 4));
        presets.removeFromLeft(8);
        bypassPhysicsButton_.setBounds(presets);
        thirdColumn.removeFromTop(5);
        applyPhysicsButton_.setBounds(thirdColumn.removeFromTop(34));
        thirdColumn.removeFromTop(5);
        physicsTelemetryLabel_.setBounds(thirdColumn);
    }

private:
    [[nodiscard]] std::array<juce::Slider*, 9> sliders() {
        return {
            &volumeSlider_, &irSlider_,
            &highShelfSlider_, &lowNoiseSlider_,
            &highNoiseSlider_, &combustionSlider_,
            &exhaustSlider_, &intakeSlider_,
            &mechanicalSlider_,
        };
    }

    [[nodiscard]] std::array<const juce::Slider*, 9>
    sliders() const {
        return {
            &volumeSlider_, &irSlider_,
            &highShelfSlider_, &lowNoiseSlider_,
            &highNoiseSlider_, &combustionSlider_,
            &exhaustSlider_, &intakeSlider_,
            &mechanicalSlider_,
        };
    }

    [[nodiscard]] std::array<juce::Slider*, 4>
    layerSliders() {
        return {
            &combustionSlider_, &exhaustSlider_,
            &intakeSlider_, &mechanicalSlider_,
        };
    }

    void configureSlider(
        juce::Slider& slider, double minimum,
        double maximum, double interval) {
        slider.setRange(minimum, maximum, interval);
        slider.setSliderStyle(
            juce::Slider::LinearHorizontal);
        slider.setTextBoxStyle(
            juce::Slider::TextBoxRight, false, 58, 22);
        slider.setColour(
            juce::Slider::thumbColourId,
            juce::Colour(0xffef7040));
        slider.setColour(
            juce::Slider::trackColourId,
            juce::Colour(0xff9c4c32));
        addAndMakeVisible(slider);
    }

    void installSliderCallbacks() {
        for (auto* slider : sliders())
            slider->onValueChange =
                [this] { publishMix(); };
    }

    void setMixInternal(const OfflineAudioMix& mix) {
        updatingControls_ = true;
        volumeSlider_.setValue(
            mix.volume, juce::dontSendNotification);
        irSlider_.setValue(
            mix.convolution, juce::dontSendNotification);
        highShelfSlider_.setValue(
            mix.highFrequencyGain,
            juce::dontSendNotification);
        lowNoiseSlider_.setValue(
            mix.lowFrequencyNoise,
            juce::dontSendNotification);
        highNoiseSlider_.setValue(
            mix.highFrequencyNoise,
            juce::dontSendNotification);
        combustionSlider_.setValue(
            mix.combustionGain,
            juce::dontSendNotification);
        exhaustSlider_.setValue(
            mix.exhaustGain,
            juce::dontSendNotification);
        intakeSlider_.setValue(
            mix.intakeGain,
            juce::dontSendNotification);
        mechanicalSlider_.setValue(
            mix.mechanicalGain,
            juce::dontSendNotification);
        updatingControls_ = false;
    }

    [[nodiscard]] AudioPhysicsSettings physicsSettings() const noexcept {
        return {
            cycleVariationSlider_.getValue(),
            cycleCorrelationSlider_.getValue(),
            afterfireToggle_.getToggleState(),
            wetLimiterToggle_.getToggleState(),
            afterfireTemperatureSlider_.getValue(),
            afterfireReactionSlider_.getValue() * 0.001,
            afterfireEfficiencySlider_.getValue(),
        };
    }

    void setPhysicsInternal(const AudioPhysicsSettings& settings) {
        cycleVariationSlider_.setValue(
            settings.cycleVariationCoefficientOfVariation,
            juce::dontSendNotification);
        cycleCorrelationSlider_.setValue(
            settings.cycleVariationCorrelation, juce::dontSendNotification);
        afterfireToggle_.setToggleState(
            settings.afterfireEnabled, juce::dontSendNotification);
        wetLimiterToggle_.setToggleState(
            settings.limiterKeepsFuel, juce::dontSendNotification);
        afterfireTemperatureSlider_.setValue(
            settings.afterfireIgnitionTemperatureK,
            juce::dontSendNotification);
        afterfireReactionSlider_.setValue(
            settings.afterfireReactionTimeSeconds * 1'000.0,
            juce::dontSendNotification);
        afterfireEfficiencySlider_.setValue(
            settings.afterfireEfficiency, juce::dontSendNotification);
    }

    void applyPhysics() {
        if (!physicsApply_) {
            physicsStatusLabel_.setText(
                "Lecture seule: aucun moteur hote.", juce::dontSendNotification);
            return;
        }
        const auto settings = physicsSettings();
        if (physicsApply_(settings)) {
            applyAudioPhysicsSettings(engine_, settings);
            physicsStatusLabel_.setText(
                "Applique. Le moteur a redemarre avec cette physique.",
                juce::dontSendNotification);
        } else {
            physicsStatusLabel_.setText(
                "Refuse (banc actif ou configuration invalide).",
                juce::dontSendNotification);
        }
    }

    [[nodiscard]] OfflineAudioMix baseMix() const {
        OfflineAudioMix mix;
        mix.volume = volumeSlider_.getValue();
        mix.convolution = irSlider_.getValue();
        mix.highFrequencyGain =
            highShelfSlider_.getValue();
        mix.lowFrequencyNoise =
            lowNoiseSlider_.getValue();
        mix.highFrequencyNoise =
            highNoiseSlider_.getValue();
        mix.combustionGain =
            combustionSlider_.getValue();
        mix.exhaustGain = exhaustSlider_.getValue();
        mix.intakeGain = intakeSlider_.getValue();
        mix.mechanicalGain =
            mechanicalSlider_.getValue();
        return mix;
    }

    [[nodiscard]] OfflineAudioMix effectiveMix() const {
        AudioWorkshopLayerSwitches switches;
        for (std::size_t layer = 0;
             layer < switches.muted.size(); ++layer) {
            switches.muted[layer] =
                muteButtons_[layer]->getToggleState();
            switches.solo[layer] =
                soloButtons_[layer]->getToggleState();
        }
        return effectiveAudioWorkshopMix(
            baseMix(),
            {
                compiledExhaustTopology_,
                compiledIntakeTopology_,
                measuredImpulseResponseAvailable_,
            },
            switches);
    }

    void publishMix() {
        if (updatingControls_ || !mixChanged_) return;
        mixChanged_(baseMix(), effectiveMix());
    }

    void updateEnginePresentation() {
        engineLabel_.setText(
            "ATELIER AUDIO  /  "
                + utf8(engine_.name),
            juce::dontSendNotification);
        if (!exportRunning_)
            statusLabel_.setText(
                compiledExhaustTopology_
                    ? "GRAPHE PHYSIQUE ACTIF"
                    : "MODE COMPATIBILITE",
                juce::dontSendNotification);
    }

    void updateAvailability() {
        irSlider_.setEnabled(
            measuredImpulseResponseAvailable_);
        controlLabels_[1].setText(
            measuredImpulseResponseAvailable_
                ? "Retour IR mesure (max +50 %)"
                : "Retour IR (aucune mesure chargee)",
            juce::dontSendNotification);
        irSlider_.setTooltip(
            measuredImpulseResponseAvailable_
                ? "Ajoute le retour de la reponse impulsionnelle mesuree; "
                  "le signal sec reste intact."
                : "Aucune reponse impulsionnelle mesuree n'est chargee.");

        lowNoiseSlider_.setEnabled(
            !compiledIntakeTopology_);
        controlLabels_[3].setText(
            compiledIntakeTopology_
                ? "Bruit admission legacy (N/A)"
                : "Bruit admission legacy",
            juce::dontSendNotification);
        highNoiseSlider_.setEnabled(
            !compiledExhaustTopology_);
        controlLabels_[4].setText(
            compiledExhaustTopology_
                ? "Bruit echappement legacy (N/A)"
                : "Bruit echappement legacy",
            juce::dontSendNotification);

        combustionSlider_.setEnabled(
            !compiledExhaustTopology_);
        muteButtons_[0]->setEnabled(
            !compiledExhaustTopology_);
        soloButtons_[0]->setEnabled(
            !compiledExhaustTopology_);
        if (compiledExhaustTopology_) {
            muteButtons_[0]->setToggleState(
                false, juce::dontSendNotification);
            soloButtons_[0]->setToggleState(
                false, juce::dontSendNotification);
            controlLabels_[5].setText(
                "Combustion directe (N/A: pression -> echappement)",
                juce::dontSendNotification);
            combustionSlider_.setTooltip(
                "Le graphe thermoacoustique possede la pression cylindre. "
                "La voix synthetique directe est volontairement retiree.");
        } else {
            controlLabels_[5].setText(
                "Combustion directe",
                juce::dontSendNotification);
            combustionSlider_.setTooltip({});
        }
    }

    void chooseScenario() {
        if (exportRunning_) return;
        fileChooser_ = std::make_unique<juce::FileChooser>(
            "Choisir un scenario audio JSON",
            juce::File::getSpecialLocation(
                juce::File::userDocumentsDirectory),
            "*.json");
        auto safe =
            juce::Component::SafePointer<WorkshopContent>(
                this);
        fileChooser_->launchAsync(
            juce::FileBrowserComponent::openMode
                | juce::FileBrowserComponent::canSelectFiles,
            [safe](const juce::FileChooser& chooser) {
                if (!safe) return;
                const auto file = chooser.getResult();
                if (file.existsAsFile()) {
                    OfflineAudioScenario scenario;
                    std::string error;
                    if (loadOfflineAudioScenario(
                            fileSystemPath(file),
                            scenario, error)) {
                        safe->scenarioPath_ =
                            fileSystemPath(file);
                        safe->updateScenarioLabel();
                    } else {
                        juce::AlertWindow::
                            showMessageBoxAsync(
                                juce::MessageBoxIconType::
                                    WarningIcon,
                                "Scenario invalide",
                                utf8(error));
                    }
                }
                safe->fileChooser_.reset();
            });
    }

    void updateScenarioLabel() {
        scenarioLabel_.setText(
            scenarioPath_.empty()
                ? "Scenario: showcase integre\n"
                  "demarrage -> ralenti -> montee -> "
                  "limiteur -> deceleration"
                : "Scenario JSON:\n"
                    + displayPath(scenarioPath_),
            juce::dontSendNotification);
    }

    void chooseExportDirectory() {
        if (exportRunning_) return;
        fileChooser_ = std::make_unique<juce::FileChooser>(
            "Choisir le dossier parent de l'export HQ",
            juce::File::getSpecialLocation(
                juce::File::userMusicDirectory),
            juce::String {});
        auto safe =
            juce::Component::SafePointer<WorkshopContent>(
                this);
        fileChooser_->launchAsync(
            juce::FileBrowserComponent::openMode
                | juce::FileBrowserComponent::
                    canSelectDirectories,
            [safe](const juce::FileChooser& chooser) {
                if (!safe) return;
                const auto directory = chooser.getResult();
                if (directory.isDirectory())
                    safe->startExport(
                        fileSystemPath(directory));
                safe->fileChooser_.reset();
            });
    }

    void startExport(
        const std::filesystem::path& parentDirectory) {
        OfflineAudioScenario scenario =
            makeDefaultOfflineAudioScenario(engine_);
        if (!scenarioPath_.empty()) {
            std::string error;
            if (!loadOfflineAudioScenario(
                    scenarioPath_, scenario, error)) {
                juce::AlertWindow::showMessageBoxAsync(
                    juce::MessageBoxIconType::WarningIcon,
                    "Scenario invalide", utf8(error));
                return;
            }
        }

        const auto rate = sampleRateSelector_.getSelectedId()
                == 1
            ? 48'000U
            : (sampleRateSelector_.getSelectedId() == 3
                   ? 192'000U
                   : 96'000U);
        const auto format =
            formatSelector_.getSelectedId() == 2
            ? OfflineWaveFormat::float32
            : OfflineWaveFormat::pcm24;
        const auto stamp =
            juce::Time::getCurrentTime().formatted(
                "%Y%m%d-%H%M%S")
                .toStdString();
        const auto baseName =
            fileSlug(engine_.name) + "-" + stamp
            + "-" + std::to_string(rate / 1'000U)
            + "k-" + offlineWaveFormatName(format);
        auto outputDirectory =
            parentDirectory / baseName;
        for (int suffix = 2;
             std::filesystem::exists(outputDirectory);
             ++suffix) {
            outputDirectory = parentDirectory
                / (baseName + "-"
                   + std::to_string(suffix));
        }

        OfflineAudioExportRequest request;
        request.engine = engine_;
        request.scenario = std::move(scenario);
        request.mix = effectiveMix();
        request.outputDirectory = outputDirectory;
        request.assetRoot = assetRoot_;
        request.sampleRateHz = rate;
        request.format = format;
        request.writeStems =
            stemsToggle_.getToggleState();

        if (exportThread_.joinable())
            exportThread_.join();
        exportRunning_ = true;
        threadProgress_.store(
            0.0, std::memory_order_relaxed);
        progressValue_ = 0.0;
        exportButton_.setEnabled(false);
        scenarioButton_.setEnabled(false);
        defaultScenarioButton_.setEnabled(false);
        sampleRateSelector_.setEnabled(false);
        formatSelector_.setEnabled(false);
        stemsToggle_.setEnabled(false);
        cancelButton_.setEnabled(true);
        revealButton_.setEnabled(false);
        statusLabel_.setText(
            "PREPARATION DU RENDU...",
            juce::dontSendNotification);

        auto safe =
            juce::Component::SafePointer<WorkshopContent>(
                this);
        exportThread_ = std::jthread(
            [safe, request = std::move(request),
             outputDirectory](
                std::stop_token stopToken) mutable {
                std::string lastStage;
                auto result = exportOfflineAudio(
                    request,
                    [safe, stopToken, &lastStage](
                        double fraction,
                        std::string_view stage) {
                        if (stopToken.stop_requested())
                            return false;
                        if (safe)
                            safe->threadProgress_.store(
                                fraction,
                                std::memory_order_relaxed);
                        if (stage != lastStage) {
                            lastStage = stage;
                            const auto stageCopy =
                                std::string(stage);
                            juce::MessageManager::callAsync(
                                [safe, stageCopy] {
                                    if (safe)
                                        safe->statusLabel_.setText(
                                            "RENDU  "
                                                + utf8(stageCopy),
                                            juce::
                                                dontSendNotification);
                                });
                        }
                        return !stopToken.stop_requested();
                    });
                juce::MessageManager::callAsync(
                    [safe, result = std::move(result),
                     outputDirectory]() mutable {
                        if (safe)
                            safe->finishExport(
                                std::move(result),
                                outputDirectory);
                    });
            });
    }

    void finishExport(
        OfflineAudioExportResult result,
        const std::filesystem::path& outputDirectory) {
        exportRunning_ = false;
        exportButton_.setEnabled(true);
        scenarioButton_.setEnabled(true);
        defaultScenarioButton_.setEnabled(true);
        sampleRateSelector_.setEnabled(true);
        formatSelector_.setEnabled(true);
        stemsToggle_.setEnabled(true);
        cancelButton_.setEnabled(false);
        if (result.success) {
            threadProgress_.store(
                1.0, std::memory_order_relaxed);
            lastOutputDirectory_ = outputDirectory;
            revealButton_.setEnabled(true);
            statusLabel_.setText(
                "EXPORT TERMINE  /  "
                    + juce::String(
                        static_cast<juce::int64>(
                            result.renderedFrames))
                    + " FRAMES",
                juce::dontSendNotification);
            juce::String message;
            message << "Export termine.\n\n"
                    << displayPath(outputDirectory)
                    << "\n\nGraphe echappement physique: "
                    << (result.physicalExhaustActive
                            ? "actif" : "inactif")
                    << "\nTroncatures de delai: "
                    << static_cast<juce::int64>(
                           result.delayTruncationCount)
                    << "\nFrontieres invalides: "
                    << static_cast<juce::int64>(
                           result.invalidBoundarySampleCount)
                    << "\nTelemetrie perdue: "
                    << static_cast<juce::int64>(
                           result.droppedFiringEvents
                           + result.droppedPressureSamples)
                    << "\nVariation cycles: "
                    << juce::String(result.cycleMultiplierMinimum, 3)
                    << " .. " << juce::String(result.cycleMultiplierMaximum, 3)
                    << "\nAfterfire: "
                    << juce::String(result.afterfirePeakHeatReleaseKw, 2)
                    << " kW peak / "
                    << juce::String(result.afterfireFuelBurnedMg, 1)
                    << " mg brules"
                    << "\nSilencieux poreux: "
                    << static_cast<int>(result.porousMufflerCount);
            if (!result.warnings.empty())
                message << "\n\nWarnings: "
                        << static_cast<int>(
                               result.warnings.size());
            juce::AlertWindow::showMessageBoxAsync(
                juce::MessageBoxIconType::InfoIcon,
                "Rendu audio HQ", message);
        } else if (result.cancelled) {
            threadProgress_.store(
                0.0, std::memory_order_relaxed);
            statusLabel_.setText(
                "EXPORT ANNULE  /  AUCUN WAV PARTIEL",
                juce::dontSendNotification);
        } else {
            statusLabel_.setText(
                "ECHEC EXPORT",
                juce::dontSendNotification);
            juce::AlertWindow::showMessageBoxAsync(
                juce::MessageBoxIconType::WarningIcon,
                "Echec du rendu audio HQ",
                utf8(result.error));
        }
    }

    void timerCallback() override {
        progressValue_ = threadProgress_.load(
            std::memory_order_relaxed);
        progressBar_.repaint();
        const auto telemetry = telemetryProvider_
            ? telemetryProvider_()
            : audioPhysicsTelemetryFor(engine_, EngineState {});
        physicsTelemetryLabel_.setText(
            "LIVE  cycles " + juce::String(telemetry.minimumCycleMultiplier, 3)
                + " .. " + juce::String(telemetry.maximumCycleMultiplier, 3)
                + "  |  afterfire " + juce::String(telemetry.afterfireHeatReleaseKw, 2)
                + " kW / " + juce::String(telemetry.afterfireFuelBurnMgPerSecond, 1)
                + " mg/s  |  silencieux poreux "
                + juce::String(static_cast<int>(telemetry.porousMufflerCount)),
            juce::dontSendNotification);
    }

    EngineConfig engine_;
    std::filesystem::path assetRoot_;
    std::filesystem::path scenarioPath_;
    std::filesystem::path lastOutputDirectory_;
    bool compiledExhaustTopology_ { false };
    bool compiledIntakeTopology_ { false };
    bool measuredImpulseResponseAvailable_ { false };
    bool updatingControls_ { false };
    bool exportRunning_ { false };
    MixChangedCallback mixChanged_;
    PhysicsApplyCallback physicsApply_;
    TelemetryProvider telemetryProvider_;

    juce::GroupComponent mixGroup_;
    juce::GroupComponent exportGroup_;
    juce::GroupComponent physicsGroup_;
    juce::Label engineLabel_;
    juce::Label statusLabel_;
    std::array<juce::Label, 9> controlLabels_;
    juce::Slider volumeSlider_;
    juce::Slider irSlider_;
    juce::Slider highShelfSlider_;
    juce::Slider lowNoiseSlider_;
    juce::Slider highNoiseSlider_;
    juce::Slider combustionSlider_;
    juce::Slider exhaustSlider_;
    juce::Slider intakeSlider_;
    juce::Slider mechanicalSlider_;
    std::array<std::unique_ptr<juce::TextButton>, 4>
        muteButtons_;
    std::array<std::unique_ptr<juce::TextButton>, 4>
        soloButtons_;
    juce::TextButton resetButton_ { "RESET MIX" };

    juce::ComboBox sampleRateSelector_;
    juce::ComboBox formatSelector_;
    juce::ToggleButton stemsToggle_ {
        "Master + stems exacts + carte d'ordres"
    };
    juce::TextButton scenarioButton_ {
        "SCENARIO JSON..."
    };
    juce::TextButton defaultScenarioButton_ {
        "DEFAUT"
    };
    juce::Label scenarioLabel_;
    juce::TextButton exportButton_ {
        "CHOISIR DOSSIER ET EXPORTER"
    };
    juce::TextButton cancelButton_ { "ANNULER" };
    juce::TextButton revealButton_ {
        "OUVRIR DOSSIER"
    };
    juce::Label proofLabel_;
    double progressValue_ { 0.0 };
    std::atomic<double> threadProgress_ { 0.0 };
    juce::ProgressBar progressBar_;
    std::jthread exportThread_;
    std::unique_ptr<juce::FileChooser> fileChooser_;
    std::array<juce::Label, 5> physicsLabels_;
    juce::Slider cycleVariationSlider_;
    juce::Slider cycleCorrelationSlider_;
    juce::Slider afterfireTemperatureSlider_;
    juce::Slider afterfireReactionSlider_;
    juce::Slider afterfireEfficiencySlider_;
    juce::ToggleButton afterfireToggle_ { "AFTERFIRE PHYSIQUE ACTIF" };
    juce::ToggleButton wetLimiterToggle_ {
        "RUPTEUR SPARK-CUT / CARBURANT CONSERVE"
    };
    juce::TextButton demoPhysicsButton_ { "DEMO AUDIBLE" };
    juce::TextButton bypassPhysicsButton_ { "BYPASS" };
    juce::TextButton applyPhysicsButton_ { "APPLIQUER ET REDEMARRER" };
    juce::Label physicsTelemetryLabel_;
    juce::Label physicsStatusLabel_;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(WorkshopContent)
};

AudioWorkshopWindow::AudioWorkshopWindow(
    const EngineConfig& engine,
    std::filesystem::path assetRoot,
    const OfflineAudioMix& mix,
    bool compiledExhaustTopology,
    bool compiledIntakeTopology,
    bool measuredImpulseResponseAvailable,
    MixChangedCallback mixChanged,
    PhysicsApplyCallback physicsApply,
    TelemetryProvider telemetryProvider)
    : juce::DocumentWindow(
        "EngineLab - Atelier audio",
        juce::Colour(0xff0d1312),
        juce::DocumentWindow::closeButton,
        true) {
    content_ = new WorkshopContent(
        engine, std::move(assetRoot), mix,
        compiledExhaustTopology,
        compiledIntakeTopology,
        measuredImpulseResponseAvailable,
        std::move(mixChanged), std::move(physicsApply),
        std::move(telemetryProvider));
    setContentOwned(content_, true);
    setUsingNativeTitleBar(true);
    setResizable(true, true);
    setResizeLimits(1'100, 800, 1'700, 1'080);
    centreWithSize(1'280, 860);
}

AudioWorkshopWindow::~AudioWorkshopWindow() = default;

void AudioWorkshopWindow::closeButtonPressed() {
    setVisible(false);
}

void AudioWorkshopWindow::setEngine(
    const EngineConfig& engine,
    std::filesystem::path assetRoot,
    bool compiledExhaustTopology,
    bool compiledIntakeTopology,
    bool measuredImpulseResponseAvailable) {
    if (content_ != nullptr)
        content_->setEngine(
            engine, std::move(assetRoot),
            compiledExhaustTopology,
            compiledIntakeTopology,
            measuredImpulseResponseAvailable);
}

void AudioWorkshopWindow::setMix(
    const OfflineAudioMix& mix) {
    if (content_ != nullptr) content_->setMix(mix);
}

} // namespace enginelab
