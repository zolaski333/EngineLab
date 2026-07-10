#include <enginelab/app/MainComponent.hpp>
#include <algorithm>
#include <cmath>
#include <numbers>

namespace enginelab {
namespace {
constexpr std::array<std::uint32_t, 8> curveColours {
    0xfff26b38, 0xff40c4ff, 0xff9ccc65, 0xffffca28, 0xffab76ff, 0xffff6e9d, 0xff26d7ae, 0xffef5350
};
[[nodiscard]] juce::String utf8(const char* text) { return juce::String::fromUTF8(text); }
[[nodiscard]] const char* runningStateName(RunningState state) noexcept {
    switch (state) {
    case RunningState::stopped: return "ARRÊTÉ";
    case RunningState::cranking: return "DÉMARRAGE";
    case RunningState::idling: return "RALENTI";
    case RunningState::running: return "EN MARCHE";
    case RunningState::unstable: return "INSTABLE";
    case RunningState::knocking: return "CLIQUETIS";
    case RunningState::overheating: return "SURCHAUFFE";
    case RunningState::damaged: return "ENDOMMAGÉ";
    case RunningState::destroyed: return "DÉTRUIT";
    }
    return "INCONNU";
}

[[nodiscard]] const char* layoutName(EngineLayout layout) noexcept {
    switch (layout) {
    case EngineLayout::inlineLayout: return "INLINE";
    case EngineLayout::vLayout: return "V";
    case EngineLayout::flat: return "FLAT";
    case EngineLayout::radial: return "RADIAL";
    case EngineLayout::custom: return "CUSTOM";
    }
    return "CUSTOM";
}

[[nodiscard]] double crankThrowMmFor(const EngineConfig& config, const CylinderConfig& cylinder) noexcept {
    if (cylinder.crankJournalId != 0) {
        const auto journal = std::find_if(config.crankJournals.begin(), config.crankJournals.end(),
            [&cylinder](const CrankJournalConfig& item) { return item.id == cylinder.crankJournalId; });
        if (journal != config.crankJournals.end()) return journal->throwMm;
    }
    return cylinder.strokeMm * 0.5;
}

[[nodiscard]] double crankOffsetDegreesFor(const EngineConfig& config, const CylinderConfig& cylinder) noexcept {
    (void)config;
    return cylinder.crankOffsetDegrees;
}
}

MainComponent::MainComponent() {
    presets_ = makeCatalogOrBasePresets(std::filesystem::path(ENGINELAB_CATALOG_ROOT));
    config_ = presets_[std::min<std::size_t>(1, presets_.size() - 1)];
    setSize(1'440, 840);
    setWantsKeyboardFocus(true);
    title_.setFont(juce::FontOptions(18.0F, juce::Font::bold));
    title_.setColour(juce::Label::textColourId, juce::Colour(0xffe5ebe8));
    addAndMakeVisible(title_);

    for (int index = 0; index < static_cast<int>(presets_.size()); ++index)
        engineSelector_.addItem(presets_[static_cast<std::size_t>(index)].name, index + 1);
    engineSelector_.setSelectedId(2, juce::dontSendNotification);
    engineSelector_.onChange = [this] { selectEngine(engineSelector_.getSelectedItemIndex()); };
    addAndMakeVisible(engineSelector_);
    editButton_.onClick = [this] { showConfigEditor(); };
    importButton_.onClick = [this] { importEngine(); };
    exportButton_.onClick = [this] { exportEngine(); };
    csvButton_.onClick = [this] { exportDynoCsv(); };
    for (auto* button : { &editButton_, &importButton_, &exportButton_, &csvButton_ }) addAndMakeVisible(*button);
    const std::array<const char*, 5> exhaustPresets { "Street", "Open", "Turbo", "Long tube", "Moto" };
    for (int index = 0; index < static_cast<int>(exhaustPresets.size()); ++index)
        exhaustPresetSelector_.addItem(exhaustPresets[static_cast<std::size_t>(index)], index + 1);
    exhaustPresetSelector_.setSelectedItemIndex(exhaustPresetIndex_, juce::dontSendNotification);
    exhaustPresetSelector_.onChange = [this] { applyExhaustPreset(exhaustPresetSelector_.getSelectedItemIndex()); };
    addAndMakeVisible(exhaustPresetSelector_);

    ignitionButton_.setClickingTogglesState(true);
    starterButton_.setButtonText(utf8("S  DÉMARREUR"));
    ignitionButton_.setColour(juce::TextButton::buttonOnColourId, juce::Colour(0xffb94d2b));
    ignitionButton_.onClick = [this] { if (runtime_) runtime_->setIgnitionEnabled(ignitionButton_.getToggleState()); };
    starterButton_.onStateChange = [this] {
        if (runtime_) runtime_->setStarterEngaged(starterKeyDown_ || starterButton_.getState() == juce::Button::buttonDown);
    };
    dynoButton_.setColour(juce::TextButton::buttonOnColourId, juce::Colour(0xff297d67));
    dynoButton_.onClick = [this] { toggleDyno(); };
    addAndMakeVisible(ignitionButton_); addAndMakeVisible(starterButton_); addAndMakeVisible(dynoButton_);

    historySelector_.setTextWhenNothingSelected(utf8("Courbes archivées"));
    historySelector_.onChange = [this] { repaint(); };
    deleteRunButton_.onClick = [this] {
        const auto selected = historySelector_.getSelectedItemIndex();
        if (selected >= 0 && selected < static_cast<int>(archivedRuns_.size())) {
            archivedRuns_.erase(archivedRuns_.begin() + selected);
            updateHistorySelector();
            if (!archivedRuns_.empty())
                historySelector_.setSelectedItemIndex(std::min(selected, static_cast<int>(archivedRuns_.size()) - 1));
            repaint();
        }
    };
    addAndMakeVisible(historySelector_); addAndMakeVisible(deleteRunButton_);

    throttleLabel_.setText(utf8("ACCÉLÉRATEUR  [W 25% / E 50% / R 100%]"), juce::dontSendNotification);
    loadLabel_.setText("CHARGE MANUELLE", juce::dontSendNotification);
    afrLabel_.setText(utf8("INJECTION IDÉALISÉE — AFR"), juce::dontSendNotification);
    advanceLabel_.setText("AVANCE ALLUMAGE", juce::dontSendNotification);
    for (auto* label : { &throttleLabel_, &loadLabel_, &afrLabel_, &advanceLabel_ }) {
        label->setColour(juce::Label::textColourId, juce::Colour(0xff87948f));
        label->setFont(juce::FontOptions(12.0F));
        addAndMakeVisible(*label);
    }
    configureSlider(throttleSlider_, 0.0, 100.0, 12.0, " %");
    configureSlider(loadSlider_, 0.0, 100.0, 8.0, " %");
    configureSlider(afrSlider_, 10.0, 18.0, 14.7, "");
    configureSlider(advanceSlider_, -5.0, 40.0, 18.0, utf8("°"));
    throttleSlider_.onValueChange = [this] { if (runtime_) runtime_->setThrottle(throttleSlider_.getValue() / 100.0); };
    loadSlider_.onValueChange = [this] { if (runtime_) runtime_->setLoad(loadSlider_.getValue() / 100.0); };
    afrSlider_.onValueChange = [this] { if (runtime_) runtime_->setTargetAirFuelRatio(afrSlider_.getValue()); };
    advanceSlider_.onValueChange = [this] { if (runtime_) runtime_->setIgnitionAdvanceDegrees(advanceSlider_.getValue()); };

    selectEngine(std::min(1, static_cast<int>(presets_.size()) - 1));
    startTimerHz(30);
    juce::MessageManager::callAsync([safe = juce::Component::SafePointer<MainComponent>(this)] { if (safe) safe->grabKeyboardFocus(); });
}

MainComponent::~MainComponent() { stopTimer(); shutdownAudio(); if (runtime_) runtime_->stop(); }

void MainComponent::selectEngine(int presetIndex) {
    if (presetIndex < 0 || presetIndex >= static_cast<int>(presets_.size())) return;
    applyConfig(presets_[static_cast<std::size_t>(presetIndex)]);
}

void MainComponent::applyConfig(const EngineConfig& newConfig) {
    std::unique_ptr<EngineRuntime> replacement;
    try {
        replacement = std::make_unique<EngineRuntime>(newConfig);
    } catch (const std::exception& error) {
        showError(utf8("Configuration moteur invalide"), juce::String::fromUTF8(error.what()));
        return;
    }
    collectFinishedRuns();
    shutdownAudio();
    if (runtime_) runtime_->stop();
    audio_.reset(); runtime_.reset();
    config_ = newConfig;
    runtime_ = std::move(replacement);
    audio_ = std::make_unique<RealtimeEngineAudio>(runtime_->audioEvents(), runtime_->audioState());
    importedRunCount_ = 0;
    telemetryWrite_ = 0; telemetryCount_ = 0;
    runtime_->setThrottle(throttleSlider_.getValue() / 100.0);
    runtime_->setLoad(loadSlider_.getValue() / 100.0);
    runtime_->setTargetAirFuelRatio(afrSlider_.getValue());
    runtime_->setIgnitionAdvanceDegrees(advanceSlider_.getValue());
    runtime_->setIgnitionEnabled(ignitionButton_.getToggleState());
    runtime_->setAudioVolume(audioVolume_);
    runtime_->setAudioConvolution(audioConvolution_);
    runtime_->setHighFrequencyGain(highFrequencyGain_);
    runtime_->setLowFrequencyNoise(lowFrequencyNoise_);
    runtime_->setHighFrequencyNoise(highFrequencyNoise_);
    runtime_->setCombustionGain(combustionGain_);
    runtime_->setExhaustGain(exhaustGain_);
    runtime_->setIntakeGain(intakeGain_);
    runtime_->setMechanicalGain(mechanicalGain_);
    runtime_->setExhaustPreset(static_cast<AudioExhaustPreset>(exhaustPresetIndex_));
    runtime_->start();
    setAudioChannels(0, 2);
    title_.setText("EngineLab   /   " + juce::String(config_.name) + "   /   "
                   + juce::String(engineDisplacementLitres(config_), 2) + " L", juce::dontSendNotification);
    engineSelector_.setText(juce::String(config_.name), juce::dontSendNotification);
}

void MainComponent::showError(const juce::String& title, const juce::String& message) {
    juce::AlertWindow::showMessageBoxAsync(juce::MessageBoxIconType::WarningIcon, title, message);
}

void MainComponent::showConfigEditor() {
    const auto encoded = jsonSerializer_.encode(config_);
    showConfigEditor(juce::String::fromUTF8(encoded.data(), static_cast<int>(encoded.size())));
}

void MainComponent::showConfigEditor(const juce::String& initialText) {
    if (configEditor_) return;
    configEditor_ = std::make_unique<juce::AlertWindow>(utf8("Éditeur moteur JSON"),
        utf8("Tous les paramètres sont éditables. Appliquer redémarre la simulation."),
        juce::MessageBoxIconType::NoIcon);
    configEditor_->addTextEditor("json", initialText, {}, false);
    if (auto* editor = configEditor_->getTextEditor("json")) {
        editor->setMultiLine(true, true);
        editor->setReturnKeyStartsNewLine(true);
        editor->setSize(720, 470);
        editor->setFont(juce::FontOptions(14.0F));
    }
    configEditor_->addButton(utf8("APPLIQUER"), 1, juce::KeyPress(juce::KeyPress::returnKey));
    configEditor_->addButton("ANNULER", 0, juce::KeyPress(juce::KeyPress::escapeKey));
    auto safe = juce::Component::SafePointer<MainComponent>(this);
    configEditor_->enterModalState(true, juce::ModalCallbackFunction::create([safe](int result) {
        if (!safe) return;
        juce::String retryText;
        if (result == 1 && safe->configEditor_) {
            if (const auto* editor = safe->configEditor_->getTextEditor("json")) {
                retryText = editor->getText();
                const auto utf8Text = retryText.toRawUTF8();
                const auto decoded = safe->jsonSerializer_.decode(utf8Text);
                if (decoded) safe->applyConfig(*decoded.config);
                else safe->showError(utf8("JSON invalide"), juce::String::fromUTF8(decoded.error.c_str()));
            }
        }
        safe->configEditor_.reset();
        if (result == 1 && retryText.isNotEmpty()) {
            const auto decoded = safe->jsonSerializer_.decode(retryText.toRawUTF8());
            if (!decoded) juce::MessageManager::callAsync([safe, retryText] {
                if (safe) safe->showConfigEditor(retryText);
            });
        }
    }), false);
}

void MainComponent::importEngine() {
    fileChooser_ = std::make_unique<juce::FileChooser>(utf8("Importer un moteur"), juce::File {}, "*.json;*.yaml;*.yml");
    auto safe = juce::Component::SafePointer<MainComponent>(this);
    fileChooser_->launchAsync(juce::FileBrowserComponent::openMode | juce::FileBrowserComponent::canSelectFiles,
        [safe](const juce::FileChooser& chooser) {
            if (!safe) return;
            const auto file = chooser.getResult();
            if (file.existsAsFile()) {
                if (file.getSize() > 2 * 1024 * 1024) {
                    safe->showError(utf8("Import impossible"), utf8("Le fichier dépasse la limite de 2 Mio."));
                    safe->fileChooser_.reset();
                    return;
                }
                const auto text = file.loadFileAsString();
                const auto bytes = text.toRawUTF8();
                const auto decoded = file.hasFileExtension("yaml;yml")
                    ? safe->yamlSerializer_.decode(bytes) : safe->jsonSerializer_.decode(bytes);
                if (decoded) safe->applyConfig(*decoded.config);
                else safe->showError(utf8("Import impossible"), juce::String::fromUTF8(decoded.error.c_str()));
            }
            safe->fileChooser_.reset();
        });
}

void MainComponent::exportEngine() {
    fileChooser_ = std::make_unique<juce::FileChooser>(utf8("Exporter le moteur"),
        juce::File::getSpecialLocation(juce::File::userDocumentsDirectory)
            .getChildFile(juce::File::createLegalFileName(juce::String(config_.name)) + ".json"),
        "*.json;*.yaml");
    auto safe = juce::Component::SafePointer<MainComponent>(this);
    fileChooser_->launchAsync(juce::FileBrowserComponent::saveMode | juce::FileBrowserComponent::warnAboutOverwriting,
        [safe](const juce::FileChooser& chooser) {
            if (!safe) return;
            auto file = chooser.getResult();
            if (file != juce::File {}) {
                if (!file.hasFileExtension("json;yaml;yml")) file = file.withFileExtension("json");
                const auto encoded = file.hasFileExtension("yaml;yml")
                    ? safe->yamlSerializer_.encode(safe->config_) : safe->jsonSerializer_.encode(safe->config_);
                if (!file.replaceWithText(juce::String::fromUTF8(encoded.data(), static_cast<int>(encoded.size()))))
                    safe->showError(utf8("Export impossible"), utf8("Le fichier n'a pas pu être écrit."));
            }
            safe->fileChooser_.reset();
        });
}

void MainComponent::exportDynoCsv() {
    collectFinishedRuns();
    const DynoRun* run = nullptr;
    const auto selected = historySelector_.getSelectedItemIndex();
    if (selected >= 0 && selected < static_cast<int>(archivedRuns_.size()))
        run = &archivedRuns_[static_cast<std::size_t>(selected)];
    else if (!archivedRuns_.empty()) run = &archivedRuns_.back();
    else if (!visibleCurrentRun_.points.empty()) run = &visibleCurrentRun_;
    if (run == nullptr) { showError("CSV DYNO", utf8("Aucune courbe à exporter.")); return; }
    juce::String csv = "rpm;torque_nm;power_kw;actual_afr;target_afr;lambda;volumetric_efficiency;air_flow_g_s;fuel_flow_g_s;bsfc_g_kwh;map_kpa;"
        "exhaust_pressure_kpa;coolant_c;oil_c;oil_pressure_kpa;exhaust_c;ignition_advance_deg;correction_factor;"
        "corrected_torque_nm;corrected_power_kw\n";
    for (const auto& point : run->points)
        csv << juce::String(point.rpm, 1) << ';' << juce::String(point.torqueNm, 2) << ';'
            << juce::String(point.powerKw, 2) << ';' << juce::String(point.airFuelRatio, 2) << ';'
            << juce::String(point.targetAirFuelRatio, 2) << ';' << juce::String(point.lambda, 4) << ';'
            << juce::String(point.volumetricEfficiency, 4) << ';' << juce::String(point.airFlowGramsPerSecond, 4) << ';'
            << juce::String(point.fuelFlowGramsPerSecond, 4) << ';'
            << juce::String(point.brakeSpecificFuelConsumptionGPerKwh, 2) << ';' << juce::String(point.manifoldPressureKpa, 3) << ';'
            << juce::String(point.exhaustPressureKpa, 3) << ';' << juce::String(point.coolantTemperatureC, 2) << ';'
            << juce::String(point.oilTemperatureC, 2) << ';' << juce::String(point.oilPressureKpa, 2) << ';'
            << juce::String(point.exhaustTemperatureC, 2)
            << ';' << juce::String(point.ignitionAdvanceDegrees, 2) << ';'
            << juce::String(point.atmosphericCorrectionFactor, 4) << ';'
            << juce::String(point.correctedTorqueNm, 2) << ';' << juce::String(point.correctedPowerKw, 2) << '\n';
    fileChooser_ = std::make_unique<juce::FileChooser>("Exporter CSV",
        juce::File::getSpecialLocation(juce::File::userDocumentsDirectory).getChildFile(juce::String(run->engineName) + ".csv"), "*.csv");
    auto safe = juce::Component::SafePointer<MainComponent>(this);
    fileChooser_->launchAsync(juce::FileBrowserComponent::saveMode | juce::FileBrowserComponent::warnAboutOverwriting,
        [safe, csv](const juce::FileChooser& chooser) {
            if (!safe) return;
            auto file = chooser.getResult();
            if (file != juce::File {} && !file.withFileExtension("csv").replaceWithText(csv))
                safe->showError("CSV DYNO", utf8("Le fichier n'a pas pu être écrit."));
            safe->fileChooser_.reset();
        });
}

void MainComponent::reloadEngine() {
    const auto selectedIndex = engineSelector_.getSelectedItemIndex();
    if (selectedIndex >= 0) {
        auto reloadedPresets = makeCatalogOrBasePresets(std::filesystem::path(ENGINELAB_CATALOG_ROOT));
        if (!reloadedPresets.empty()) {
            presets_ = std::move(reloadedPresets);
            engineSelector_.clear(juce::dontSendNotification);
            for (int index = 0; index < static_cast<int>(presets_.size()); ++index)
                engineSelector_.addItem(presets_[static_cast<std::size_t>(index)].name, index + 1);
            const auto clampedIndex = std::clamp(selectedIndex, 0, static_cast<int>(presets_.size()) - 1);
            engineSelector_.setSelectedItemIndex(clampedIndex, juce::dontSendNotification);
            applyConfig(presets_[static_cast<std::size_t>(clampedIndex)]);
            return;
        }
    }
    applyConfig(config_);
}

void MainComponent::prepareToPlay(int blockSize, double sampleRate) { if (audio_) audio_->prepare(sampleRate, blockSize); }
void MainComponent::getNextAudioBlock(const juce::AudioSourceChannelInfo& info) {
    if (audio_ && info.buffer != nullptr) audio_->render(*info.buffer, info.startSample, info.numSamples);
    else if (info.buffer != nullptr) info.buffer->clear(info.startSample, info.numSamples);
}
void MainComponent::releaseResources() { if (audio_) audio_->release(); }

void MainComponent::configureSlider(juce::Slider& slider, double min, double max, double value, const juce::String& suffix) {
    slider.setRange(min, max, (max - min) > 20.0 ? 1.0 : 0.1); slider.setValue(value); slider.setTextValueSuffix(suffix);
    slider.setSliderStyle(juce::Slider::LinearHorizontal); slider.setTextBoxStyle(juce::Slider::TextBoxRight, false, 68, 24);
    slider.setColour(juce::Slider::thumbColourId, juce::Colour(0xffef6f3c));
    slider.setColour(juce::Slider::trackColourId, juce::Colour(0xffa94a2c)); addAndMakeVisible(slider);
}

void MainComponent::collectFinishedRuns() {
    if (!runtime_) return;
    const auto runs = runtime_->dynoHistory();
    bool changed = false;
    for (auto index = importedRunCount_; index < runs.size(); ++index) {
        auto run = runs[index]; run.id = nextUiRunId_++; archivedRuns_.push_back(std::move(run));
        changed = true;
    }
    importedRunCount_ = runs.size();
    if (changed) updateHistorySelector();
}

void MainComponent::updateHistorySelector() {
    const auto previousSelection = historySelector_.getSelectedItemIndex();
    historySelector_.clear(juce::dontSendNotification);
    for (int index = 0; index < static_cast<int>(archivedRuns_.size()); ++index) {
        const auto& run = archivedRuns_[static_cast<std::size_t>(index)];
        historySelector_.addItem(juce::String(run.engineName) + "  #" + juce::String(run.id)
            + "  " + juce::String(run.peakCorrectedPowerKw > 0.0 ? run.peakCorrectedPowerKw : run.peakPowerKw, 1)
            + " kW corr.", index + 1);
    }
    if (previousSelection >= 0 && previousSelection < static_cast<int>(archivedRuns_.size()))
        historySelector_.setSelectedItemIndex(previousSelection, juce::dontSendNotification);
}

void MainComponent::setThrottlePreset(double value) {
    throttleSlider_.setValue(value * 100.0, juce::sendNotificationSync);
}

void MainComponent::toggleDyno() {
    if (!runtime_) return;
    if (runtime_->dynoRunning()) runtime_->stopDyno(); else runtime_->startDyno();
}

void MainComponent::applyExhaustPreset(int presetIndex) {
    exhaustPresetIndex_ = std::clamp(presetIndex, 0, 4);
    if (runtime_) runtime_->setExhaustPreset(static_cast<AudioExhaustPreset>(exhaustPresetIndex_));
    repaint();
}

void MainComponent::adjustAudioOrSimulation(double wheelDelta) {
    if (!runtime_ || wheelDelta == 0.0) return;
    const auto step = juce::KeyPress::isKeyCurrentlyDown(juce::KeyPress::spaceKey) ? 0.01 : 0.05;
    if (juce::KeyPress::isKeyCurrentlyDown('G') && visibleState_.dynoHoldEnabled) {
        runtime_->adjustDynoHoldRpm(wheelDelta > 0.0 ? 100.0 : -100.0);
    } else if (juce::KeyPress::isKeyCurrentlyDown('Z')) {
        audioVolume_ = std::clamp(audioVolume_ + wheelDelta * step, 0.0, 2.0);
        runtime_->setAudioVolume(audioVolume_);
    } else if (juce::KeyPress::isKeyCurrentlyDown('X')) {
        audioConvolution_ = std::clamp(audioConvolution_ + wheelDelta * step, 0.0, 1.0);
        runtime_->setAudioConvolution(audioConvolution_);
    } else if (juce::KeyPress::isKeyCurrentlyDown('C')) {
        highFrequencyGain_ = std::clamp(highFrequencyGain_ + wheelDelta * step, 0.2, 2.5);
        runtime_->setHighFrequencyGain(highFrequencyGain_);
    } else if (juce::KeyPress::isKeyCurrentlyDown('V')) {
        lowFrequencyNoise_ = std::clamp(lowFrequencyNoise_ + wheelDelta * step, 0.0, 1.5);
        runtime_->setLowFrequencyNoise(lowFrequencyNoise_);
    } else if (juce::KeyPress::isKeyCurrentlyDown('B')) {
        highFrequencyNoise_ = std::clamp(highFrequencyNoise_ + wheelDelta * step, 0.0, 1.5);
        runtime_->setHighFrequencyNoise(highFrequencyNoise_);
    } else if (juce::KeyPress::isKeyCurrentlyDown('J')) {
        combustionGain_ = std::clamp(combustionGain_ + wheelDelta * step, 0.0, 2.0);
        runtime_->setCombustionGain(combustionGain_);
    } else if (juce::KeyPress::isKeyCurrentlyDown('K')) {
        exhaustGain_ = std::clamp(exhaustGain_ + wheelDelta * step, 0.0, 2.0);
        runtime_->setExhaustGain(exhaustGain_);
    } else if (juce::KeyPress::isKeyCurrentlyDown('L')) {
        intakeGain_ = std::clamp(intakeGain_ + wheelDelta * step, 0.0, 2.0);
        runtime_->setIntakeGain(intakeGain_);
    } else if (juce::KeyPress::isKeyCurrentlyDown('O')) {
        mechanicalGain_ = std::clamp(mechanicalGain_ + wheelDelta * step, 0.0, 2.0);
        runtime_->setMechanicalGain(mechanicalGain_);
    } else if (juce::KeyPress::isKeyCurrentlyDown('N')) {
        runtime_->setTimeScale(runtime_->timeScale() + wheelDelta * step);
    } else if (juce::KeyPress::isKeyCurrentlyDown(juce::KeyPress::spaceKey)) {
        throttleSlider_.setValue(std::clamp(throttleSlider_.getValue() + wheelDelta * 2.0, 0.0, 100.0),
                                 juce::sendNotificationSync);
    }
    repaint();
}

bool MainComponent::keyPressed(const juce::KeyPress& key) {
    const auto character = juce::CharacterFunctions::toUpperCase(key.getTextCharacter());
    if (key.getKeyCode() >= juce::KeyPress::F1Key && key.getKeyCode() <= juce::KeyPress::F12Key) {
        const auto presetIndex = key.getKeyCode() - juce::KeyPress::F1Key;
        if (runtime_ && !runtime_->dynoRunning() && presetIndex < static_cast<int>(presets_.size())) {
            engineSelector_.setSelectedItemIndex(presetIndex, juce::dontSendNotification);
            selectEngine(presetIndex);
        }
        return true;
    }
    if (key == juce::KeyPress::escapeKey) { juce::JUCEApplicationBase::quit(); return true; }
    if (key == juce::KeyPress::returnKey) { reloadEngine(); return true; }
    if (key == juce::KeyPress::tabKey) { screen_ = (screen_ + 1) % 5; repaint(); return true; }
    if (key == juce::KeyPress::upKey) { if (runtime_) runtime_->shiftUp(); return true; }
    if (key == juce::KeyPress::downKey) { if (runtime_) runtime_->shiftDown(); return true; }
    if (character == 'P') { if (runtime_) runtime_->setPaused(!runtime_->paused()); return true; }
    if (character == 'F') {
        if (auto* window = findParentComponentOfClass<juce::DocumentWindow>())
            window->setFullScreen(!window->isFullScreen());
        return true;
    }
    if (character == 'I') { showDynoStats_ = !showDynoStats_; repaint(); return true; }
    if (character == 'Y') {
        applyExhaustPreset((exhaustPresetIndex_ + 1) % 5);
        exhaustPresetSelector_.setSelectedItemIndex(exhaustPresetIndex_, juce::dontSendNotification);
        return true;
    }
    if (character == 'M') { viewLayer_ = std::min(viewLayer_ + 1, 3); repaint(); return true; }
    if (character == ',') { viewLayer_ = std::max(viewLayer_ - 1, 0); repaint(); return true; }
    if (character == '1') { if (runtime_) runtime_->setTimeScale(0.25); return true; }
    if (character == '2') { if (runtime_) runtime_->setTimeScale(0.5); return true; }
    if (character == '3') { if (runtime_) runtime_->setTimeScale(1.0); return true; }
    if (character == '4') { if (runtime_) runtime_->setTimeScale(2.0); return true; }
    if (character == '5') { if (runtime_) runtime_->setTimeScale(4.0); return true; }
    if (character == 'A') {
        ignitionButton_.setToggleState(!ignitionButton_.getToggleState(), juce::sendNotificationSync);
        return true;
    }
    if (character == 'Q') { setThrottlePreset(0.01); return true; }
    if (character == 'R') { setThrottlePreset(1.0); return true; }
    if (character == 'W') { setThrottlePreset(0.25); return true; }
    if (character == 'E') { setThrottlePreset(0.50); return true; }
    if (character == 'D') { toggleDyno(); return true; }
    if (character == 'H') { if (runtime_) runtime_->setDynoHoldEnabled(!runtime_->dynoHoldEnabled()); return true; }
    if (character == 'T') {
        targetClutchPressure_ = std::clamp(targetClutchPressure_ - 0.08, 0.0, 1.0);
        if (runtime_) runtime_->setClutchPressure(targetClutchPressure_);
        return true;
    }
    if (character == 'U') {
        targetClutchPressure_ = std::clamp(targetClutchPressure_ + 0.08, 0.0, 1.0);
        if (runtime_) runtime_->setClutchPressure(targetClutchPressure_);
        return true;
    }
    return character == 'S';
}

bool MainComponent::keyStateChanged(bool) {
    const auto down = juce::KeyPress::isKeyCurrentlyDown('S');
    if (down != starterKeyDown_) {
        starterKeyDown_ = down;
        if (runtime_) runtime_->setStarterEngaged(down || starterButton_.getState() == juce::Button::buttonDown);
        starterButton_.setToggleState(down, juce::dontSendNotification);
    }
    if (runtime_) {
        if (juce::ModifierKeys::getCurrentModifiersRealtime().isShiftDown()) {
            runtime_->setClutchPressure(0.0);
        } else {
            runtime_->setClutchPressure(targetClutchPressure_);
        }
    }
    return down || juce::ModifierKeys::getCurrentModifiersRealtime().isShiftDown();
}

void MainComponent::mouseWheelMove(const juce::MouseEvent&, const juce::MouseWheelDetails& wheel) {
    adjustAudioOrSimulation(wheel.deltaY);
}

void MainComponent::timerCallback() {
    if (!runtime_) return;
    visibleState_ = runtime_->snapshot();
    telemetryHistory_[telemetryWrite_] = visibleState_;
    telemetryWrite_ = (telemetryWrite_ + 1) % telemetryHistory_.size();
    telemetryCount_ = std::min(telemetryCount_ + 1, telemetryHistory_.size());
    visibleCurrentRun_ = runtime_->currentDynoRun();
    visibleDiagnostics_ = diagnostics_.evaluate(config_, visibleState_);
    collectFinishedRuns();
    const auto running = runtime_->dynoRunning();
    title_.setText("EngineLab   /   " + juce::String(config_.name) + "   /   "
        + juce::String(engineDisplacementLitres(config_), 2) + " L   /   "
        + (runtime_->paused() ? juce::String("PAUSE") : "x" + juce::String(runtime_->timeScale(), 1)),
        juce::dontSendNotification);
    dynoButton_.setButtonText(running ? utf8("D  ARRÊTER DYNO") : juce::String("D  LANCER DYNO"));
    dynoButton_.setToggleState(running, juce::dontSendNotification);
    ignitionButton_.setEnabled(!running);
    starterButton_.setEnabled(!running);
    engineSelector_.setEnabled(!running);
    exhaustPresetSelector_.setEnabled(true);
    editButton_.setEnabled(!running); importButton_.setEnabled(!running);
    for (auto* slider : { &throttleSlider_, &loadSlider_, &afrSlider_, &advanceSlider_ }) slider->setEnabled(!running);
    repaint();
}

void MainComponent::paint(juce::Graphics& g) {
    juce::ColourGradient background(juce::Colour(0xff101715), 0, 0, juce::Colour(0xff060908), 0, static_cast<float>(getHeight()), false);
    g.setGradientFill(background); g.fillAll();
    g.setColour(juce::Colour(0xff26312e)); g.drawHorizontalLine(65, 0.0F, static_cast<float>(getWidth()));
    g.drawVerticalLine(285, 66.0F, static_cast<float>(getHeight()));
    const auto chartWidth = std::clamp(getWidth() * 0.34F, 380.0F, 540.0F);
    const auto chartX = static_cast<float>(getWidth()) - chartWidth - 22.0F;
    g.drawVerticalLine(static_cast<int>(chartX - 18.0F), 66.0F, static_cast<float>(getHeight()));
    const auto mainArea = juce::Rectangle<float>(305.0F, 82.0F, chartX - 335.0F, static_cast<float>(getHeight()) - 400.0F);
    if (screen_ == 1) drawLoadSimulationPanel(g, mainArea);
    else if (screen_ == 2) drawMixerPanel(g, mainArea);
    else if (screen_ == 3) drawOscilloscopePanel(g, mainArea);
    else if (screen_ == 4) drawDebugPanel(g, mainArea);
    else drawEngine(g, mainArea);
    auto details = juce::Rectangle<float>(305.0F, static_cast<float>(getHeight()) - 282.0F, chartX - 335.0F, 72.0F);
    g.setColour(juce::Colour(0xff111817)); g.fillRoundedRectangle(details, 8.0F);
    g.setColour(juce::Colour(0xff2b3834)); g.drawRoundedRectangle(details, 8.0F, 1.0F);
    double peakCylinderPressure = 0.0;
    for (std::size_t index = 0; index < visibleState_.cylinderStateCount; ++index)
        peakCylinderPressure = std::max(peakCylinderPressure, visibleState_.cylinderStates[index].pressureEstimateBar);
    const std::array<juce::String, 9> detailValues {
        utf8(runningStateName(visibleState_.runningState)),
        juce::String(layoutName(config_.layout)) + "  " + juce::String(config_.cylinders.size()) + " cyl",
        "VE " + juce::String(visibleState_.volumetricEfficiency * 100.0, 1) + " %",
        utf8("Air/ess. ") + juce::String(visibleState_.airFlowGramsPerSecond, 1) + "/"
            + juce::String(visibleState_.fuelFlowGramsPerSecond, 2) + " g/s",
        "Pmax " + juce::String(peakCylinderPressure, 1) + " bar  EXH "
            + juce::String(visibleState_.exhaustRunnerPressureKpa, 0) + " kPa",
        utf8("Huile ") + juce::String(visibleState_.oilPressureKpa, 0) + " kPa · " + juce::String(visibleState_.oilTemperatureC, 0) + utf8("°C"),
        "EGT " + juce::String(visibleState_.exhaustTemperatureC, 0) + utf8("°C"),
        utf8("Piston ") + juce::String(visibleState_.meanPistonSpeedMps, 1) + " m/s · "
            + juce::String(visibleState_.peakPistonAccelerationG, 0) + " g",
        "V " + juce::String(visibleState_.vehicleSpeedMps * 3.6, 1) + " km/h · G "
            + (visibleState_.gear < 0 ? juce::String("N") : juce::String(visibleState_.gear + 1))
    };
    auto detailBody = details.reduced(12.0F, 8.0F);
    const auto detailWidth = detailBody.getWidth() / 3.0F;
    const auto detailRowHeight = detailBody.getHeight() / 3.0F;
    g.setFont(juce::FontOptions(12.0F, juce::Font::bold));
    for (std::size_t index = 0; index < detailValues.size(); ++index) {
        const auto row = static_cast<float>(index / 3U);
        const auto column = static_cast<float>(index % 3U);
        const auto cell = juce::Rectangle<float>(detailBody.getX() + column * detailWidth,
            detailBody.getY() + row * detailRowHeight, detailWidth, detailRowHeight);
        g.setColour(index == 0 ? juce::Colour(0xff79b89f) : juce::Colour(0xffc4cfca));
        g.drawFittedText(detailValues[index], cell.toNearestInt(), juce::Justification::centredLeft, 1);
    }
    drawGaugeCluster(g, details);
    drawTelemetryChart(g, { 305.0F, static_cast<float>(getHeight()) - 202.0F, chartX - 335.0F, 94.0F });
    drawDynoChart(g, { chartX, 92.0F, chartWidth, static_cast<float>(getHeight()) - 132.0F });

    auto telemetry = juce::Rectangle<float>(305.0F, static_cast<float>(getHeight()) - 100.0F, chartX - 335.0F, 72.0F);
    const auto telemetryColumnWidth = telemetry.getWidth() / 3.0F;
    const auto compactTelemetry = getWidth() < 1'450;
    g.setColour(juce::Colour(0xffe8eeeb)); g.setFont(juce::FontOptions(30.0F, juce::Font::bold));
    g.drawFittedText(juce::String(static_cast<int>(visibleState_.rpm)) + " tr/min", telemetry.removeFromLeft(telemetryColumnWidth).toNearestInt(), juce::Justification::centredLeft, 1);
    g.setFont(juce::FontOptions(17.0F));
    const auto torqueText = compactTelemetry
        ? juce::String(visibleState_.cycleAveragedTorqueNm, 0) + " Nm  ·  "
            + juce::String(visibleState_.cycleAveragedPowerKw, 1) + " kW"
        : juce::String(visibleState_.cycleAveragedTorqueNm, 0) + " Nm moy.  /  "
            + juce::String(visibleState_.torqueNm, 0) + " Nm instant.";
    g.drawFittedText(torqueText,
                     telemetry.removeFromLeft(telemetryColumnWidth).toNearestInt(), juce::Justification::centred, 1);
    const auto environmentText = compactTelemetry
        ? "MAP " + juce::String(visibleState_.manifoldPressureKpa, 0) + " kPa\nAFR "
            + juce::String(visibleState_.airFuelRatio, 1) + utf8(" · ")
            + juce::String(visibleState_.coolantTemperatureC, 0) + utf8("°C")
        : juce::String(visibleState_.manifoldPressureKpa, 0) + " kPa  / AFR "
            + juce::String(visibleState_.airFuelRatio, 1) + "/" + juce::String(visibleState_.targetAirFuelRatio, 1)
            + utf8("  ·  Eau ") + juce::String(visibleState_.coolantTemperatureC, 0) + utf8("°C");
    g.drawFittedText(environmentText, telemetry.toNearestInt(), juce::Justification::centredRight,
                     compactTelemetry ? 2 : 1);

    const auto diagnosticArea = juce::Rectangle<float>(20.0F, static_cast<float>(getHeight()) - 92.0F, 245.0F, 64.0F);
    const auto hasDiagnostic = !visibleDiagnostics_.empty();
    const auto critical = std::any_of(visibleDiagnostics_.begin(), visibleDiagnostics_.end(), [](const Diagnostic& diagnostic) {
        return diagnostic.severity == DiagnosticSeverity::critical;
    });
    g.setColour(hasDiagnostic ? (critical ? juce::Colour(0xff5c2723) : juce::Colour(0xff59491f)) : juce::Colour(0xff17231f));
    g.fillRoundedRectangle(diagnosticArea, 7.0F);
    g.setColour(hasDiagnostic ? (critical ? juce::Colour(0xffff9a8c) : juce::Colour(0xffffd46a)) : juce::Colour(0xff79b89f));
    g.setFont(juce::FontOptions(11.0F, juce::Font::bold));
    juce::String diagnosticText = utf8("DIAGNOSTIC  ·  Aucun défaut détecté");
    if (hasDiagnostic) {
        diagnosticText.clear();
        for (std::size_t index = 0; index < std::min<std::size_t>(3, visibleDiagnostics_.size()); ++index) {
            if (index != 0) diagnosticText << '\n';
            diagnosticText << juce::String::fromUTF8(visibleDiagnostics_[index].message.c_str());
        }
    }
    g.drawFittedText(diagnosticText, diagnosticArea.reduced(10.0F, 7.0F).toNearestInt(), juce::Justification::centredLeft, 3);
}

void MainComponent::drawLoadSimulationPanel(juce::Graphics& g, juce::Rectangle<float> area) const {
    g.setColour(juce::Colour(0xff111817)); g.fillRoundedRectangle(area, 8.0F);
    g.setColour(juce::Colour(0xff2b3834)); g.drawRoundedRectangle(area, 8.0F, 1.0F);
    auto body = area.reduced(18.0F);
    g.setColour(juce::Colour(0xffdce5e1)); g.setFont(juce::FontOptions(16.0F, juce::Font::bold));
    g.drawText("LOAD / TRANSMISSION", body.removeFromTop(34.0F), juce::Justification::centredLeft);
    const auto gearText = visibleState_.gear < 0 ? juce::String("N") : juce::String(visibleState_.gear + 1);
    const std::array<juce::String, 8> values {
        "GEAR  " + gearText + " / " + juce::String(visibleState_.gearCount),
        "CLUTCH  " + juce::String(visibleState_.clutchPressure * 100.0, 0) + " %",
        "SPEED  " + juce::String(visibleState_.vehicleSpeedMps * 3.6, 1) + " km/h",
        "DIST  " + juce::String(visibleState_.vehicleDistanceM / 1'000.0, 3) + " km",
        "WHEEL  " + juce::String(visibleState_.wheelTorqueNm, 0) + " Nm",
        "DRAG LOAD  " + juce::String(visibleState_.drivelineLoadTorqueNm, 0) + " Nm",
        "DYNO HOLD  " + juce::String(visibleState_.dynoHoldEnabled ? "ON" : "OFF"),
        "HOLD RPM  " + juce::String(visibleState_.dynoHoldRpm, 0)
    };
    const auto columns = 2;
    const auto cellHeight = std::min(58.0F, body.getHeight() / 4.0F);
    const auto cellWidth = body.getWidth() / static_cast<float>(columns);
    for (std::size_t index = 0; index < values.size(); ++index) {
        const auto row = static_cast<float>(index / columns);
        const auto column = static_cast<float>(index % columns);
        auto cell = juce::Rectangle<float>(body.getX() + column * cellWidth, body.getY() + row * cellHeight,
                                           cellWidth - 10.0F, cellHeight - 8.0F);
        g.setColour(juce::Colour(0xff17231f)); g.fillRoundedRectangle(cell, 7.0F);
        g.setColour(index == 6 && visibleState_.dynoHoldEnabled ? juce::Colour(0xff79b89f) : juce::Colour(0xffc4cfca));
        g.setFont(juce::FontOptions(14.0F, juce::Font::bold));
        g.drawFittedText(values[index], cell.reduced(10.0F, 5.0F).toNearestInt(), juce::Justification::centredLeft, 1);
    }
}

void MainComponent::drawMixerPanel(juce::Graphics& g, juce::Rectangle<float> area) const {
    g.setColour(juce::Colour(0xff111817)); g.fillRoundedRectangle(area, 8.0F);
    g.setColour(juce::Colour(0xff2b3834)); g.drawRoundedRectangle(area, 8.0F, 1.0F);
    auto body = area.reduced(18.0F);
    g.setColour(juce::Colour(0xffdce5e1)); g.setFont(juce::FontOptions(16.0F, juce::Font::bold));
    g.drawText("MIXER / AUDIO", body.removeFromTop(34.0F), juce::Justification::centredLeft);
    const std::array<const char*, 5> presetNames { "Street chamber", "Open headers", "Turbo muffled", "Long tube", "Motorcycle" };
    g.setColour(juce::Colour(0xff79b89f));
    g.setFont(juce::FontOptions(13.0F, juce::Font::bold));
    g.drawText("PRESET  " + juce::String(presetNames[static_cast<std::size_t>(std::clamp(exhaustPresetIndex_, 0, 4))]),
               body.removeFromTop(28.0F), juce::Justification::centredLeft);
    const std::array<std::pair<juce::String, double>, 9> values {{
        { "Z  Volume", audioVolume_ / 2.0 },
        { "X  Convolution", audioConvolution_ },
        { "C  High gain", highFrequencyGain_ / 2.5 },
        { "V  Low noise", lowFrequencyNoise_ / 1.5 },
        { "B  High noise", highFrequencyNoise_ / 1.5 },
        { "J  Combustion", combustionGain_ / 2.0 },
        { "K  Exhaust", exhaustGain_ / 2.0 },
        { "L  Intake", intakeGain_ / 2.0 },
        { "O  Mechanical", mechanicalGain_ / 2.0 }
    }};
    for (const auto& item : values) {
        auto row = body.removeFromTop(36.0F);
        row.removeFromBottom(6.0F);
        g.setColour(juce::Colour(0xff202b28)); g.fillRoundedRectangle(row, 7.0F);
        g.setColour(juce::Colour(0xff83918c)); g.setFont(juce::FontOptions(12.0F, juce::Font::bold));
        g.drawText(item.first, row.removeFromLeft(130.0F).reduced(10.0F, 0.0F), juce::Justification::centredLeft);
        auto meter = row.reduced(8.0F, 12.0F);
        g.setColour(juce::Colour(0xff30413c)); g.fillRoundedRectangle(meter, 4.0F);
        g.setColour(juce::Colour(0xffef6f3c));
        g.fillRoundedRectangle(meter.withWidth(meter.getWidth() * static_cast<float>(std::clamp(item.second, 0.0, 1.0))), 4.0F);
    }
}

void MainComponent::drawGaugeCluster(juce::Graphics& g, juce::Rectangle<float> area) const {
    g.setColour(juce::Colour(0xff111817)); g.fillRoundedRectangle(area, 8.0F);
    g.setColour(juce::Colour(0xff2b3834)); g.drawRoundedRectangle(area, 8.0F, 1.0F);
    auto body = area.reduced(10.0F, 8.0F);
    struct Gauge { juce::String label; juce::String value; double normalized; juce::Colour colour; };
    const auto gearText = visibleState_.gear < 0 ? juce::String("N") : juce::String(visibleState_.gear + 1);
    const std::array<Gauge, 8> gauges {{
        { "RPM", juce::String(visibleState_.rpm, 0), visibleState_.rpm / std::max(1.0, config_.redlineRpm), juce::Colour(0xffef6f3c) },
        { "MAP", juce::String(visibleState_.manifoldPressureKpa, 0) + " kPa", visibleState_.manifoldPressureKpa / std::max(1.0, config_.ambientPressureKpa), juce::Colour(0xff41b6d7) },
        { "AFR", juce::String(visibleState_.airFuelRatio, 1), 1.0 - std::abs(visibleState_.airFuelRatio - visibleState_.targetAirFuelRatio) / 6.0, juce::Colour(0xff79b89f) },
        { "VE", juce::String(visibleState_.volumetricEfficiency * 100.0, 0) + "%", visibleState_.volumetricEfficiency / 1.20, juce::Colour(0xff9ccc65) },
        { "OIL", juce::String(visibleState_.oilPressureKpa, 0), visibleState_.oilPressureKpa / 520.0, juce::Colour(0xffffca28) },
        { "EGT", juce::String(visibleState_.exhaustTemperatureC, 0), (visibleState_.exhaustTemperatureC - 100.0) / 850.0, juce::Colour(0xffff7a45) },
        { "TRQ", juce::String(visibleState_.cycleAveragedTorqueNm, 0), visibleState_.cycleAveragedTorqueNm / std::max(80.0, engineDisplacementLitres(config_) * 125.0), juce::Colour(0xffab76ff) },
        { "GEAR", gearText + "  " + juce::String(visibleState_.vehicleSpeedMps * 3.6, 0), visibleState_.vehicleSpeedMps * 3.6 / 280.0, juce::Colour(0xff26d7ae) }
    }};
    const auto cellWidth = body.getWidth() / static_cast<float>(gauges.size());
    for (std::size_t index = 0; index < gauges.size(); ++index) {
        auto cell = juce::Rectangle<float>(body.getX() + static_cast<float>(index) * cellWidth, body.getY(),
                                           cellWidth - 6.0F, body.getHeight());
        const auto fill = std::clamp(static_cast<float>(gauges[index].normalized), 0.0F, 1.0F);
        g.setColour(juce::Colour(0xff1b2522)); g.fillRoundedRectangle(cell, 6.0F);
        auto bar = cell.removeFromBottom(7.0F).reduced(5.0F, 0.0F);
        g.setColour(juce::Colour(0xff303d39)); g.fillRoundedRectangle(bar, 3.0F);
        g.setColour(gauges[index].colour); g.fillRoundedRectangle(bar.withWidth(bar.getWidth() * fill), 3.0F);
        auto text = cell.reduced(6.0F, 3.0F);
        g.setColour(juce::Colour(0xff83918c)); g.setFont(juce::FontOptions(10.0F, juce::Font::bold));
        g.drawFittedText(gauges[index].label, text.removeFromTop(15.0F).toNearestInt(), juce::Justification::centred, 1);
        g.setColour(juce::Colour(0xffe3ebe7)); g.setFont(juce::FontOptions(13.0F, juce::Font::bold));
        g.drawFittedText(gauges[index].value, text.toNearestInt(), juce::Justification::centred, 1);
    }
}

void MainComponent::drawOscilloscopePanel(juce::Graphics& g, juce::Rectangle<float> area) const {
    g.setColour(juce::Colour(0xff111817)); g.fillRoundedRectangle(area, 8.0F);
    g.setColour(juce::Colour(0xff2b3834)); g.drawRoundedRectangle(area, 8.0F, 1.0F);
    auto header = area.reduced(18.0F).removeFromTop(34.0F);
    g.setColour(juce::Colour(0xffdce5e1)); g.setFont(juce::FontOptions(16.0F, juce::Font::bold));
    g.drawText("OSCILLOSCOPE", header, juce::Justification::centredLeft);
    auto plot = area.reduced(18.0F, 52.0F);
    g.setColour(juce::Colour(0xff17231f)); g.fillRoundedRectangle(plot, 7.0F);
    g.setColour(juce::Colour(0xff283632));
    for (int grid = 1; grid < 5; ++grid) {
        const auto y = plot.getY() + plot.getHeight() * static_cast<float>(grid) / 5.0F;
        g.drawHorizontalLine(static_cast<int>(y), plot.getX(), plot.getRight());
    }
    const auto drawTrace = [&](auto getter, double minimum, double maximum, juce::Colour colour, float width) {
        if (telemetryCount_ < 2) return;
        juce::Path path;
        for (std::size_t i = 0; i < telemetryCount_; ++i) {
            const auto index = (telemetryWrite_ + telemetryHistory_.size() - telemetryCount_ + i) % telemetryHistory_.size();
            const auto normalized = std::clamp((getter(telemetryHistory_[index]) - minimum) / (maximum - minimum), 0.0, 1.0);
            const auto x = plot.getX() + plot.getWidth() * static_cast<float>(i) / static_cast<float>(telemetryCount_ - 1);
            const auto y = plot.getBottom() - plot.getHeight() * static_cast<float>(normalized);
            if (i == 0) path.startNewSubPath(x, y); else path.lineTo(x, y);
        }
        g.setColour(colour); g.strokePath(path, juce::PathStrokeType(width));
    };
    drawTrace([this](const EngineState& state) { return state.rpm; }, 0.0, config_.redlineRpm, juce::Colour(0xffef6f3c), 2.2F);
    drawTrace([](const EngineState& state) { return state.manifoldPressureKpa; }, 20.0, 110.0, juce::Colour(0xff41b6d7), 1.8F);
    drawTrace([](const EngineState& state) { return state.lambda; }, 0.55, 1.35, juce::Colour(0xff79b89f), 1.6F);
    drawTrace([](const EngineState& state) { return state.knockLevel; }, 0.0, 1.0, juce::Colour(0xffffca28), 1.5F);
    drawTrace([](const EngineState& state) { return state.exhaustRunnerPressureKpa; }, 80.0, 220.0, juce::Colour(0xffff7a45), 1.5F);
    auto legend = area.reduced(18.0F).removeFromBottom(32.0F);
    const std::array<std::pair<const char*, juce::Colour>, 5> labels {{
        { "RPM", juce::Colour(0xffef6f3c) }, { "MAP", juce::Colour(0xff41b6d7) },
        { "LAMBDA", juce::Colour(0xff79b89f) }, { "KNOCK", juce::Colour(0xffffca28) },
        { "EXH", juce::Colour(0xffff7a45) }
    }};
    for (const auto& label : labels) {
        auto item = legend.removeFromLeft(90.0F);
        g.setColour(label.second); g.fillRoundedRectangle(item.removeFromLeft(18.0F).reduced(2.0F, 10.0F), 3.0F);
        g.setColour(juce::Colour(0xffaab7b2)); g.setFont(juce::FontOptions(11.0F, juce::Font::bold));
        g.drawText(label.first, item, juce::Justification::centredLeft);
    }
}

void MainComponent::drawDebugPanel(juce::Graphics& g, juce::Rectangle<float> area) const {
    g.setColour(juce::Colour(0xff111817)); g.fillRoundedRectangle(area, 8.0F);
    g.setColour(juce::Colour(0xff2b3834)); g.drawRoundedRectangle(area, 8.0F, 1.0F);
    auto body = area.reduced(18.0F);
    g.setColour(juce::Colour(0xffdce5e1)); g.setFont(juce::FontOptions(16.0F, juce::Font::bold));
    g.drawText("PHYSICS DEBUG", body.removeFromTop(34.0F), juce::Justification::centredLeft);
    const auto cylCount = std::max<std::size_t>(1, visibleState_.cylinderStateCount);
    double maxPressure = 0.0;
    double intakeLift = 0.0;
    double exhaustLift = 0.0;
    for (std::size_t index = 0; index < cylCount; ++index) {
        maxPressure = std::max(maxPressure, visibleState_.cylinderStates[index].pressureEstimateBar);
        intakeLift = std::max(intakeLift, visibleState_.cylinderStates[index].intakeValveLiftMm);
        exhaustLift = std::max(exhaustLift, visibleState_.cylinderStates[index].exhaustValveLiftMm);
    }
    const std::array<juce::String, 18> values {
        "Net torque       " + juce::String(visibleState_.netTorqueNm, 2),
        "Indicated torque " + juce::String(visibleState_.indicatedTorqueNm, 2),
        "Friction torque  " + juce::String(visibleState_.frictionTorqueNm, 2),
        "Recip torque     " + juce::String(visibleState_.reciprocatingTorqueNm, 2),
        "Load torque      " + juce::String(visibleState_.loadTorqueNm, 2),
        "Starter torque   " + juce::String(visibleState_.starterTorqueNm, 2),
        "MAP              " + juce::String(visibleState_.manifoldPressureKpa, 3),
        "Intake runner    " + juce::String(visibleState_.intakeRunnerPressureKpa, 3),
        "Exhaust runner   " + juce::String(visibleState_.exhaustRunnerPressureKpa, 3),
        "Exhaust flow     " + juce::String(visibleState_.exhaustFlowGramsPerSecond, 4),
        "Air mass/cycle   " + juce::String(visibleState_.airMassMgPerCycle, 3),
        "Fuel mg/cycle    " + juce::String(visibleState_.injectedFuelMgPerCycle, 4),
        "Max cyl pressure " + juce::String(maxPressure, 3),
        "Intake lift max  " + juce::String(intakeLift, 3),
        "Exhaust lift max " + juce::String(exhaustLift, 3),
        "Damage/wear      " + juce::String(visibleState_.damage, 4) + " / " + juce::String(visibleState_.wear, 4),
        "SPSC drops       " + juce::String(runtime_ ? runtime_->droppedEventCount() : 0),
        "Audio late/file  " + juce::String(audio_ ? audio_->lateEventCount() : 0) + " / " + juce::String(audio_ ? audio_->droppedPendingEventCount() : 0)
    };
    const auto columns = area.getWidth() > 680.0F ? 3 : 2;
    const auto cellWidth = body.getWidth() / static_cast<float>(columns);
    const auto cellHeight = std::min(34.0F, body.getHeight() / 6.0F);
    for (std::size_t index = 0; index < values.size(); ++index) {
        const auto row = static_cast<float>(index / static_cast<std::size_t>(columns));
        const auto column = static_cast<float>(index % static_cast<std::size_t>(columns));
        auto cell = juce::Rectangle<float>(body.getX() + column * cellWidth, body.getY() + row * cellHeight,
                                           cellWidth - 8.0F, cellHeight - 6.0F);
        g.setColour(juce::Colour(0xff17231f)); g.fillRoundedRectangle(cell, 6.0F);
        g.setColour(juce::Colour(0xffc4cfca)); g.setFont(juce::FontOptions(11.5F, juce::Font::bold));
        g.drawFittedText(values[index], cell.reduced(8.0F, 2.0F).toNearestInt(), juce::Justification::centredLeft, 1);
    }
}

void MainComponent::drawTelemetryChart(juce::Graphics& g, juce::Rectangle<float> area) const {
    g.setColour(juce::Colour(0xff111817)); g.fillRoundedRectangle(area, 8.0F);
    g.setColour(juce::Colour(0xff2b3834)); g.drawRoundedRectangle(area, 8.0F, 1.0F);
    auto plot = area.reduced(10.0F, 8.0F); plot.removeFromTop(15.0F);
    if (telemetryCount_ > 1) {
        const auto drawMetric = [&](auto getter, double minimum, double maximum, juce::Colour colour) {
            juce::Path path;
            for (std::size_t i = 0; i < telemetryCount_; ++i) {
                const auto index = (telemetryWrite_ + telemetryHistory_.size() - telemetryCount_ + i) % telemetryHistory_.size();
                const auto normalized = std::clamp((getter(telemetryHistory_[index]) - minimum) / (maximum - minimum), 0.0, 1.0);
                const auto x = plot.getX() + plot.getWidth() * static_cast<float>(i) / static_cast<float>(telemetryCount_ - 1);
                const auto y = plot.getBottom() - plot.getHeight() * static_cast<float>(normalized);
                if (i == 0) path.startNewSubPath(x, y); else path.lineTo(x, y);
            }
            g.setColour(colour); g.strokePath(path, juce::PathStrokeType(1.5F));
        };
        drawMetric([this](const EngineState& state) { return state.rpm; }, 0.0, config_.redlineRpm, juce::Colour(0xffef6f3c));
        drawMetric([this](const EngineState& state) { return state.manifoldPressureKpa; }, 20.0,
                   std::max(101.0, config_.ambientPressureKpa), juce::Colour(0xff41b6d7));
        drawMetric([](const EngineState& state) { return state.coolantTemperatureC; }, 20.0, 130.0, juce::Colour(0xffffca28));
    }
    g.setFont(juce::FontOptions(11.0F, juce::Font::bold));
    g.setColour(juce::Colour(0xffef6f3c)); g.drawText("RPM", area.removeFromLeft(42.0F).removeFromTop(18.0F), juce::Justification::centredLeft);
    g.setColour(juce::Colour(0xff41b6d7)); g.drawText("MAP", area.removeFromLeft(42.0F).removeFromTop(18.0F), juce::Justification::centredLeft);
    g.setColour(juce::Colour(0xffffca28)); g.drawText("EAU", area.removeFromLeft(42.0F).removeFromTop(18.0F), juce::Justification::centredLeft);
    if (runtime_ && audio_) {
        g.setColour(juce::Colour(0xff70807a));
        g.drawText("RT drop " + juce::String(runtime_->droppedEventCount()) + "  retard "
            + juce::String(runtime_->timingOverrunCount()) + "  audio tardif "
            + juce::String(audio_->lateEventCount()) + "  file audio "
            + juce::String(audio_->droppedPendingEventCount()), area.removeFromTop(18.0F), juce::Justification::centredRight);
    }
}

void MainComponent::drawEngine(juce::Graphics& g, juce::Rectangle<float> area) const {
    const auto count = static_cast<int>(config_.cylinders.size());
    if (count <= 0) return;
    if (config_.layout == EngineLayout::radial) {
        g.setFont(juce::FontOptions(12.0F, juce::Font::bold));
        g.setColour(juce::Colour(0xff82918b));
        g.drawText("RADIAL  /  DISTRIBUTION  " + juce::String(config_.camshafts.intakeDurationDegrees, 0)
            + " deg / " + juce::String(config_.camshafts.intakeLiftMm, 1) + " mm",
            area.removeFromTop(28.0F), juce::Justification::centred);
        auto body = area.reduced(10.0F);
        const auto diameter = std::min(body.getWidth(), body.getHeight());
        const auto radius = diameter * 0.30F;
        const auto strokeTravel = diameter * 0.13F;
        const auto cylinderWidth = std::clamp(diameter * 0.115F, 42.0F, 72.0F);
        const auto cylinderHeight = std::clamp(diameter * 0.245F, 88.0F, 138.0F);
        const auto center = body.getCentre();
        g.setColour(juce::Colour(0xff18221f));
        g.fillEllipse(center.x - radius * 0.72F, center.y - radius * 0.72F, radius * 1.44F, radius * 1.44F);
        g.setColour(juce::Colour(0xff2b3834));
        g.drawEllipse(center.x - radius * 0.72F, center.y - radius * 0.72F, radius * 1.44F, radius * 1.44F, 1.4F);
        g.setColour(juce::Colour(0x3341b6d7));
        g.drawEllipse(center.x - radius * 1.13F, center.y - radius * 1.13F, radius * 2.26F, radius * 2.26F, 5.0F);
        g.setColour(juce::Colour(0x44ef6f3c));
        g.drawEllipse(center.x - radius * 1.34F, center.y - radius * 1.34F, radius * 2.68F, radius * 2.68F, 5.0F);

        for (int index = 0; index < count; ++index) {
            const auto& cylinder = config_.cylinders[static_cast<std::size_t>(index)];
            const auto phase = std::fmod(visibleState_.crankAngleDegrees - crankOffsetDegreesFor(config_, cylinder) + 720.0, 720.0);
            const auto pistonAngle = phase / 720.0 * std::numbers::pi * 4.0;
            const auto crankRadius = crankThrowMmFor(config_, cylinder);
            const auto rodLength = std::max(cylinder.connectingRodMm, crankRadius + 0.1);
            const auto sliderTravel = crankRadius * (1.0 - std::cos(pistonAngle)) + rodLength
                - std::sqrt(std::max(0.0, rodLength * rodLength
                    - crankRadius * crankRadius * std::sin(pistonAngle) * std::sin(pistonAngle)));
            const auto travel = static_cast<float>(std::clamp(sliderTravel / cylinder.strokeMm, 0.0, 1.0));
            const auto angle = -std::numbers::pi * 0.5 + static_cast<double>(index) / static_cast<double>(count)
                * std::numbers::pi * 2.0;
            const auto unitX = static_cast<float>(std::cos(angle));
            const auto unitY = static_cast<float>(std::sin(angle));
            const auto pistonCenterX = center.x + unitX * (radius + travel * strokeTravel);
            const auto pistonCenterY = center.y + unitY * (radius + travel * strokeTravel);
            const auto crankPinX = center.x + static_cast<float>(std::cos(pistonAngle)) * diameter * 0.060F;
            const auto crankPinY = center.y + static_cast<float>(std::sin(pistonAngle)) * diameter * 0.060F;
            const auto* liveCylinder = static_cast<std::size_t>(index) < visibleState_.cylinderStateCount
                ? &visibleState_.cylinderStates[static_cast<std::size_t>(index)] : nullptr;
            if (liveCylinder != nullptr) {
                const auto intakePulse = static_cast<float>(std::clamp(liveCylinder->intakeFlowMgPerCycle / 55.0, 0.0, 1.0));
                const auto exhaustPulse = static_cast<float>(std::clamp(liveCylinder->exhaustFlowMgPerCycle / 55.0, 0.0, 1.0));
                g.setColour(juce::Colour(0xff41b6d7).withAlpha(0.18F + intakePulse * 0.52F));
                g.drawLine(center.x + unitX * radius * 1.13F, center.y + unitY * radius * 1.13F,
                           pistonCenterX, pistonCenterY, 1.4F + intakePulse * 3.4F);
                g.setColour(juce::Colour(0xffef6f3c).withAlpha(0.18F + exhaustPulse * 0.52F));
                g.drawLine(center.x + unitX * radius * 1.34F, center.y + unitY * radius * 1.34F,
                           pistonCenterX, pistonCenterY, 1.4F + exhaustPulse * 3.4F);
            }
            g.setColour(juce::Colour(0xff9ba8a3));
            g.drawLine(pistonCenterX, pistonCenterY, crankPinX, crankPinY, 4.0F);
            g.saveState();
            g.addTransform(juce::AffineTransform::rotation(static_cast<float>(angle + std::numbers::pi * 0.5),
                                                           pistonCenterX, pistonCenterY));
            const auto chamber = juce::Rectangle<float>(pistonCenterX - cylinderWidth * 0.5F,
                                                        pistonCenterY - cylinderHeight * 0.5F,
                                                        cylinderWidth, cylinderHeight);
            g.setColour(juce::Colour(0xff111817));
            g.fillRoundedRectangle(chamber, 7.0F);
            g.setColour(juce::Colour(0xff3a4743));
            g.drawRoundedRectangle(chamber, 7.0F, 1.3F);
            if (liveCylinder != nullptr && liveCylinder->combustionActive && liveCylinder->combustionPulse > 0.08) {
                g.setColour(liveCylinder->misfiring ? juce::Colour(0x88ffca28) : juce::Colour(0x99ff7a3d));
                g.fillEllipse(chamber.reduced(8.0F, cylinderHeight * 0.35F));
            }
            g.setColour(juce::Colour(0xff41b6d7));
            g.drawLine(chamber.getX() + cylinderWidth * 0.30F, chamber.getY() + 5.0F,
                       chamber.getX() + cylinderWidth * 0.30F, chamber.getY() + 23.0F, 3.0F);
            g.setColour(juce::Colour(0xffef6f3c));
            g.drawLine(chamber.getX() + cylinderWidth * 0.70F, chamber.getY() + 5.0F,
                       chamber.getX() + cylinderWidth * 0.70F, chamber.getY() + 23.0F, 3.0F);
            g.setColour(juce::Colour(0xffb7c1bd));
            g.fillRoundedRectangle(chamber.getX() + 5.0F, chamber.getBottom() - 28.0F - travel * 18.0F,
                                   cylinderWidth - 10.0F, 20.0F, 3.0F);
            g.restoreState();
            if (liveCylinder != nullptr) {
                g.setColour(liveCylinder->misfireProbability > 0.2 ? juce::Colour(0xffffca28) : juce::Colour(0xffaab7b2));
                g.setFont(juce::FontOptions(10.5F, juce::Font::bold));
                g.drawFittedText("C" + juce::String(liveCylinder->id) + "  "
                    + juce::String(liveCylinder->pressureEstimateBar, 1) + " bar",
                    juce::Rectangle<float>(pistonCenterX - 38.0F, pistonCenterY - 8.0F, 76.0F, 16.0F).toNearestInt(),
                    juce::Justification::centred, 1);
            }
        }
        const auto crankNeedle = visibleState_.crankAngleDegrees * std::numbers::pi / 180.0;
        g.setColour(juce::Colour(0xffef6f3c));
        g.fillEllipse(center.x - 11.0F, center.y - 11.0F, 22.0F, 22.0F);
        g.drawLine(center.x, center.y,
                   center.x + static_cast<float>(std::cos(crankNeedle)) * diameter * 0.085F,
                   center.y + static_cast<float>(std::sin(crankNeedle)) * diameter * 0.085F, 4.0F);
        return;
    }
    const auto splitBanks = config_.layout == EngineLayout::vLayout || config_.layout == EngineLayout::flat;
    const auto columns = splitBanks ? (count + 1) / 2 : count;
    const auto spacing = 14.0F;
    const auto cylinderWidth = std::clamp((area.getWidth() - spacing * static_cast<float>(columns - 1)) / static_cast<float>(columns), 46.0F, 112.0F);
    const auto totalWidth = cylinderWidth * static_cast<float>(columns) + spacing * static_cast<float>(columns - 1);
    const auto startX = area.getCentreX() - totalWidth * 0.5F;
    const auto bankHeight = splitBanks ? area.getHeight() * 0.43F : area.getHeight() * 0.72F;
    g.setFont(juce::FontOptions(12.0F, juce::Font::bold)); g.setColour(juce::Colour(0xff82918b));
    g.drawText(juce::String("DISTRIBUTION  ") + juce::String(config_.camshafts.intakeDurationDegrees, 0) + utf8("° / ")
        + juce::String(config_.camshafts.intakeLiftMm, 1) + " mm", area.removeFromTop(28.0F), juce::Justification::centred);

    const auto runnerTop = area.getY() + 10.0F;
    g.setColour(juce::Colour(0x3341b6d7));
    g.fillRoundedRectangle(area.withY(runnerTop).withHeight(11.0F).reduced(12.0F, 0.0F), 5.0F);
    g.setColour(juce::Colour(0x44ef6f3c));
    g.fillRoundedRectangle(area.withY(area.getBottom() - 18.0F).withHeight(11.0F).reduced(12.0F, 0.0F), 5.0F);

    for (int index = 0; index < count; ++index) {
        const auto bank = splitBanks ? index % 2 : 0;
        const auto column = splitBanks ? index / 2 : index;
        const auto& cylinder = config_.cylinders[static_cast<std::size_t>(index)];
        const auto phase = std::fmod(visibleState_.crankAngleDegrees - crankOffsetDegreesFor(config_, cylinder) + 720.0, 720.0);
        const auto pistonAngle = phase / 720.0 * std::numbers::pi * 4.0;
        const auto crankRadius = crankThrowMmFor(config_, cylinder);
        const auto rodLength = std::max(cylinder.connectingRodMm, crankRadius + 0.1);
        const auto sliderTravel = crankRadius * (1.0 - std::cos(pistonAngle)) + rodLength
            - std::sqrt(std::max(0.0, rodLength * rodLength
                - crankRadius * crankRadius * std::sin(pistonAngle) * std::sin(pistonAngle)));
        const auto travel = static_cast<float>(std::clamp(sliderTravel / cylinder.strokeMm, 0.0, 1.0));
        const auto x = startX + static_cast<float>(column) * (cylinderWidth + spacing);
        const auto top = area.getY() + 20.0F + static_cast<float>(bank) * (bankHeight + 18.0F);
        const auto height = bankHeight - 18.0F;
        const auto pistonY = top + 43.0F + travel * (height - 92.0F);
        const auto centerX = x + cylinderWidth * 0.5F;
        const auto crankY = top + height - 18.0F;
        g.saveState();
        if (config_.layout == EngineLayout::vLayout) {
            const auto visualAngle = static_cast<float>(std::clamp(config_.bankAngleDegrees * 0.12, 4.0, 12.0)
                * std::numbers::pi / 180.0);
            g.addTransform(juce::AffineTransform::rotation(bank == 0 ? -visualAngle : visualAngle, centerX, crankY));
        }
        g.setColour(juce::Colour(0xff111817)); g.fillRoundedRectangle(x, top, cylinderWidth, height, 7.0F);
        g.setColour(juce::Colour(0xff3a4743)); g.drawRoundedRectangle(x, top, cylinderWidth, height, 7.0F, 1.5F);
        const auto bankedPhase = phase + cylinder.bankOffsetDegrees;
        const auto intakeLift = profiledValveLiftMm(bankedPhase, 360.0 + config_.camshafts.intakeCenterlineDegrees,
            config_.camshafts.intakeDurationDegrees, config_.camshafts.intakeLiftMm,
            config_.camshafts.intakeLiftProfile);
        const auto exhaustLift = profiledValveLiftMm(bankedPhase, 720.0 - config_.camshafts.exhaustCenterlineDegrees,
            config_.camshafts.exhaustDurationDegrees, config_.camshafts.exhaustLiftMm,
            config_.camshafts.exhaustLiftProfile);
        const auto intakeDrop = static_cast<float>(intakeLift / std::max(1.0, config_.camshafts.intakeLiftMm) * 17.0);
        const auto exhaustDrop = static_cast<float>(exhaustLift / std::max(1.0, config_.camshafts.exhaustLiftMm) * 17.0);
        g.setColour(juce::Colour(0xff41b6d7)); g.drawLine(x + cylinderWidth * 0.30F, top + 5.0F, x + cylinderWidth * 0.30F, top + 22.0F + intakeDrop, 3.0F);
        g.fillEllipse(x + cylinderWidth * 0.21F, top + 18.0F + intakeDrop, cylinderWidth * 0.18F, 4.0F);
        g.setColour(juce::Colour(0xffef6f3c)); g.drawLine(x + cylinderWidth * 0.70F, top + 5.0F, x + cylinderWidth * 0.70F, top + 22.0F + exhaustDrop, 3.0F);
        g.fillEllipse(x + cylinderWidth * 0.61F, top + 18.0F + exhaustDrop, cylinderWidth * 0.18F, 4.0F);
        const auto* liveCylinder = static_cast<std::size_t>(index) < visibleState_.cylinderStateCount
            ? &visibleState_.cylinderStates[static_cast<std::size_t>(index)] : nullptr;
        if (liveCylinder != nullptr && liveCylinder->combustionActive && liveCylinder->combustionPulse > 0.08) {
            g.setColour(liveCylinder->misfiring ? juce::Colour(0x88ffca28) : juce::Colour(0x99ff7a3d));
            g.fillEllipse(x + 9.0F, top + 30.0F, cylinderWidth - 18.0F, 22.0F);
        }
        if (liveCylinder != nullptr) {
            const auto intakePulse = static_cast<float>(std::clamp(liveCylinder->intakeFlowMgPerCycle / 45.0, 0.0, 1.0));
            const auto exhaustPulse = static_cast<float>(std::clamp(liveCylinder->exhaustFlowMgPerCycle / 45.0, 0.0, 1.0));
            g.setColour(juce::Colour(0xff41b6d7).withAlpha(0.20F + intakePulse * 0.58F));
            g.drawLine(centerX, runnerTop + 9.0F, centerX, top + 25.0F, 2.0F + intakePulse * 4.0F);
            g.setColour(juce::Colour(0xffef6f3c).withAlpha(0.20F + exhaustPulse * 0.58F));
            g.drawLine(centerX, top + height - 24.0F, centerX, area.getBottom() - 13.0F, 2.0F + exhaustPulse * 4.0F);
        }
        g.setColour(juce::Colour(0xffb7c1bd)); g.fillRoundedRectangle(x + 5.0F, pistonY, cylinderWidth - 10.0F, 22.0F, 3.0F);
        const auto crankX = centerX + static_cast<float>(std::sin(pistonAngle)) * 14.0F;
        g.setColour(juce::Colour(0xff9ba8a3)); g.drawLine(centerX, pistonY + 18.0F, crankX, crankY, 4.0F);
        g.setColour(juce::Colour(0xffef6f3c)); g.fillEllipse(crankX - 4.0F, crankY - 4.0F, 8.0F, 8.0F);
        if (liveCylinder != nullptr) {
            const auto& cylinderState = *liveCylinder;
            g.setColour(cylinderState.misfireProbability > 0.2 ? juce::Colour(0xffffca28) : juce::Colour(0xff82918b));
            g.setFont(juce::FontOptions(10.5F, juce::Font::bold));
            g.drawFittedText("C" + juce::String(cylinderState.id) + "  "
                + juce::String(cylinderState.pressureEstimateBar, 1) + " bar",
                juce::Rectangle<float>(x + 3.0F, top + height - 14.0F, cylinderWidth - 6.0F, 12.0F).toNearestInt(),
                juce::Justification::centred, 1);
        }
        g.restoreState();
    }
}

void MainComponent::drawDynoChart(juce::Graphics& g, juce::Rectangle<float> area) const {
    g.setColour(juce::Colour(0xff111817)); g.fillRoundedRectangle(area, 10.0F);
    g.setColour(juce::Colour(0xff2b3834)); g.drawRoundedRectangle(area, 10.0F, 1.0F);
    auto plot = area.reduced(48.0F, 48.0F); plot.removeFromTop(24.0F);
    g.setFont(juce::FontOptions(13.0F, juce::Font::bold)); g.setColour(juce::Colour(0xffdce5e1));
    g.drawText(utf8("BANC DE PUISSANCE FREINÉ"), area.removeFromTop(38.0F), juce::Justification::centred);
    const auto maxRpm = std::max(8'500.0, config_.redlineRpm);
    if (visibleState_.dynoHoldEnabled) {
        g.setColour(juce::Colour(0xff79b89f));
        g.drawText("HOLD " + juce::String(visibleState_.dynoHoldRpm, 0) + " RPM",
                   juce::Rectangle<float>(plot.getX(), plot.getY() - 20.0F, plot.getWidth(), 18.0F),
                   juce::Justification::centredRight);
    }
    double maxTorque = 100.0;
    double maxPower = 75.0;
    auto accumulateMax = [&maxTorque, &maxPower](const DynoRun& run) {
        for (const auto& point : run.points) {
            maxTorque = std::max(maxTorque, point.correctedTorqueNm > 0.0 ? point.correctedTorqueNm : point.torqueNm);
            maxPower = std::max(maxPower, point.correctedPowerKw > 0.0 ? point.correctedPowerKw : point.powerKw);
        }
    };
    for (const auto& run : archivedRuns_) accumulateMax(run);
    accumulateMax(visibleCurrentRun_);
    g.setFont(12.0F); g.setColour(juce::Colour(0xff71827b));
    for (int grid = 0; grid <= 5; ++grid) {
        const auto y = plot.getBottom() - plot.getHeight() * static_cast<float>(grid) / 5.0F;
        g.drawHorizontalLine(static_cast<int>(y), plot.getX(), plot.getRight());
        g.drawText(juce::String(maxTorque * static_cast<double>(grid) / 5.0, 0),
                   juce::Rectangle<float>(area.getX() + 5.0F, y - 7.0F, 38.0F, 14.0F), juce::Justification::centredRight);
        g.drawText(juce::String(maxPower * static_cast<double>(grid) / 5.0, 0),
                   juce::Rectangle<float>(area.getRight() - 43.0F, y - 7.0F, 38.0F, 14.0F), juce::Justification::centredLeft);
    }
    const auto drawRun = [&](const DynoRun& run, juce::Colour colour, bool current) {
        if (run.points.size() < 2) return;
        juce::Path torquePath, powerPath;
        for (std::size_t i = 0; i < run.points.size(); ++i) {
            const auto& point = run.points[i];
            const auto x = plot.getX() + static_cast<float>(point.rpm / maxRpm) * plot.getWidth();
            const auto torque = point.correctedTorqueNm > 0.0 ? point.correctedTorqueNm : point.torqueNm;
            const auto power = point.correctedPowerKw > 0.0 ? point.correctedPowerKw : point.powerKw;
            const auto torqueY = plot.getBottom() - static_cast<float>(torque / maxTorque) * plot.getHeight();
            const auto powerY = plot.getBottom() - static_cast<float>(power / maxPower) * plot.getHeight();
            if (i == 0) { torquePath.startNewSubPath(x, torqueY); powerPath.startNewSubPath(x, powerY); }
            else { torquePath.lineTo(x, torqueY); powerPath.lineTo(x, powerY); }
        }
        g.setColour(colour.withAlpha(current ? 1.0F : 0.72F)); g.strokePath(torquePath, juce::PathStrokeType(current ? 2.8F : 1.7F));
        g.setColour(colour.brighter(0.75F).withAlpha(current ? 0.88F : 0.55F));
        juce::Path dashedPower;
        const float dashLengths[] { 7.0F, 4.0F };
        juce::PathStrokeType(current ? 1.9F : 1.2F).createDashedStroke(
            dashedPower, powerPath, dashLengths, 2);
        g.fillPath(dashedPower);
    };
    const auto selectedRun = historySelector_.getSelectedItemIndex();
    for (std::size_t index = 0; index < archivedRuns_.size(); ++index)
        drawRun(archivedRuns_[index], juce::Colour(curveColours[index % curveColours.size()]),
                static_cast<int>(index) == selectedRun);
    drawRun(visibleCurrentRun_, juce::Colour(0xffffffff), true);
    g.setColour(juce::Colour(0xff80908a)); g.setFont(10.0F);
    for (int grid = 0; grid <= 4; ++grid) {
        const auto x = plot.getX() + plot.getWidth() * static_cast<float>(grid) / 4.0F;
        g.drawText(juce::String(maxRpm * static_cast<double>(grid) / 4.0, 0),
                   juce::Rectangle<float>(x - 25.0F, plot.getBottom() + 3.0F, 50.0F, 14.0F), juce::Justification::centred);
    }
    g.drawText("Corrigé · Nm axe G (foncé) · kW axe D (clair)",
               juce::Rectangle<float>(plot.getX(), plot.getBottom() + 18.0F, plot.getWidth(), 18.0F), juce::Justification::centred);
    if (visibleState_.dynoHoldEnabled) {
        const auto holdX = plot.getX() + static_cast<float>(visibleState_.dynoHoldRpm / maxRpm) * plot.getWidth();
        g.setColour(juce::Colour(0xff79b89f).withAlpha(0.75F));
        g.drawVerticalLine(static_cast<int>(holdX), plot.getY(), plot.getBottom());
    }
}

void MainComponent::resized() {
    title_.setBounds(22, 12, getWidth() - 770, 42);
    engineSelector_.setBounds(getWidth() - 260, 17, 230, 32);
    exhaustPresetSelector_.setBounds(getWidth() - 415, 17, 145, 32);
    auto toolbarX = getWidth() - 827;
    editButton_.setBounds(toolbarX, 17, 98, 32); toolbarX += 102;
    importButton_.setBounds(toolbarX, 17, 92, 32); toolbarX += 96;
    exportButton_.setBounds(toolbarX, 17, 92, 32); toolbarX += 96;
    csvButton_.setBounds(toolbarX, 17, 92, 32);
    const auto compact = getHeight() < 740;
    const auto buttonHeight = compact ? 38 : 42;
    const auto labelHeight = compact ? 20 : 23;
    const auto sliderHeight = compact ? 34 : 40;
    const auto fieldGap = compact ? 5 : 8;
    auto controls = juce::Rectangle<int>(20, compact ? 78 : 84, 245, getHeight() - 99);
    ignitionButton_.setBounds(controls.removeFromTop(buttonHeight)); controls.removeFromTop(compact ? 4 : 6);
    starterButton_.setBounds(controls.removeFromTop(buttonHeight)); controls.removeFromTop(compact ? 4 : 6);
    dynoButton_.setBounds(controls.removeFromTop(compact ? 42 : 46)); controls.removeFromTop(compact ? 12 : 20);
    for (auto pair : { std::pair { &throttleLabel_, &throttleSlider_ }, { &loadLabel_, &loadSlider_ },
                       { &afrLabel_, &afrSlider_ }, { &advanceLabel_, &advanceSlider_ } }) {
        pair.first->setBounds(controls.removeFromTop(labelHeight));
        pair.second->setBounds(controls.removeFromTop(sliderHeight));
        controls.removeFromTop(fieldGap);
    }
    controls.removeFromTop(compact ? 4 : 12);
    historySelector_.setBounds(controls.removeFromTop(compact ? 30 : 34)); controls.removeFromTop(compact ? 4 : 6);
    deleteRunButton_.setBounds(controls.removeFromTop(compact ? 30 : 34));
}
} // namespace enginelab
