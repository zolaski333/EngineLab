#include <enginelab/app/MainComponent.hpp>
#include <enginelab/app/Theme.hpp>
#include <enginelab/gasdynamics/ExhaustNetworkLayout.hpp>
#include <enginelab/audio/ImpulseResponseLoader.hpp>
#include <enginelab/calibration/EcuCalibration.hpp>
#include <enginelab/calibration/EcuCalibrationKeys.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace enginelab {
namespace {
constexpr std::array<std::uint32_t, 8> curveColours {
    0xfff26b38, 0xff40c4ff, 0xff9ccc65, 0xffffca28, 0xffab76ff, 0xffff6e9d, 0xff26d7ae, 0xffef5350
};
[[nodiscard]] juce::String utf8(const char* text) { return juce::String::fromUTF8(text); }
[[nodiscard]] std::filesystem::path resolveCatalogRoot() {
    const auto executable = juce::File::getSpecialLocation(juce::File::currentExecutableFile);
    const std::array<std::filesystem::path, 4> candidates {
        std::filesystem::path(executable.getParentDirectory().getFullPathName().toStdString()),
        std::filesystem::path(executable.getParentDirectory().getParentDirectory().getFullPathName().toStdString()),
        std::filesystem::path(ENGINELAB_CATALOG_ROOT),
        std::filesystem::current_path()
    };
    for (const auto& candidate : candidates)
        if (std::filesystem::is_directory(candidate / "engines")
            && std::filesystem::is_directory(candidate / "parts")) return candidate;
    return std::filesystem::path(ENGINELAB_CATALOG_ROOT);
}

[[nodiscard]] juce::String utf8(const std::string& text) { return juce::String::fromUTF8(text.c_str()); }

/** A path that keeps every character of the file name, on Windows too. */
[[nodiscard]] std::filesystem::path pathOf(const juce::File& file) {
    return std::filesystem::path(file.getFullPathName().toWideCharPointer());
}

[[nodiscard]] juce::File fileOf(const std::filesystem::path& path) {
    return juce::File(juce::String(path.wstring().c_str()));
}

/** Where "Save engine" writes. */
[[nodiscard]] juce::File savedEnginesFolder() {
    return juce::File::getSpecialLocation(juce::File::userApplicationDataDirectory)
        .getChildFile("EngineLab").getChildFile("engines");
}

[[nodiscard]] juce::String issuesText(const std::vector<calibration::CalibrationIssue>& issues) {
    juce::StringArray lines;
    for (const auto& issue : issues) lines.add(utf8(issue.path + ": " + issue.message));
    return lines.joinIntoString("\n");
}

[[nodiscard]] juce::String csvField(const std::string& source) {
    auto value = juce::String::fromUTF8(source.c_str());
    value = value.replace("\"", "\"\"");
    return "\"" + value + "\"";
}
}

MainComponent::MainComponent() {
    catalogRoot_ = resolveCatalogRoot();
    const auto savedEngineErrors = loadPresets();
    config_ = presets_[std::min<std::size_t>(1, presets_.size() - 1)].config;
    setSize(1'440, 860);
    setOpaque(true);
    setWantsKeyboardFocus(true);

    refreshEngineChoices();
    topBar_.onEngineSelected = [this](int presetIndex) { selectEngine(presetIndex); };
    topBar_.exhaustButton.onClick = [this] { showExhaustDesigner(); };
    topBar_.ecuButton.onClick = [this] { showEcuTuner(); };
    topBar_.audioButton.onClick = [this] { showAudioWorkshop(); };
    topBar_.moreButton.onClick = [this] { showMoreMenu(); };

    controls_.ignition.onClick = [this] {
        if (runtime_) runtime_->setIgnitionEnabled(controls_.ignition.getToggleState());
    };
    controls_.starter.onStateChange = [this] {
        if (runtime_) runtime_->setStarterEngaged(starterKeyDown_
            || controls_.starter.getState() == juce::Button::buttonDown);
    };
    controls_.dyno.onClick = [this] { toggleDyno(); };
    controls_.onThrottlePreset = [this](double fraction) { setThrottlePreset(fraction); };
    controls_.throttle.onValueChange = [this] {
        if (runtime_) runtime_->setThrottle(controls_.throttle.getValue() / 100.0);
    };
    controls_.load.slider.onValueChange = [this] {
        if (runtime_) runtime_->setLoad(controls_.load.slider.getValue() / 100.0);
    };
    controls_.afrTrim.slider.onValueChange = [this] {
        if (runtime_) runtime_->setAirFuelRatioTrim(controls_.afrTrim.slider.getValue());
    };
    controls_.sparkTrim.slider.onValueChange = [this] {
        if (runtime_) runtime_->setIgnitionTrimDegrees(controls_.sparkTrim.slider.getValue());
    };

    auto& dyno = side_.dyno;
    dyno.onSelectRun = [this](int runIndex) { selectRun(runIndex); };
    dyno.onDoneTyping = [this] { grabKeyboardFocus(); };
    dyno.onRenameRun = [this](const juce::String& name) {
        auto* presentation = selectedPresentation();
        if (presentation == nullptr || name.isEmpty()) return;
        presentation->name = name;
        side_.dyno.syncRuns();
    };
    dyno.onCycleColour = [this] {
        auto* presentation = selectedPresentation();
        if (presentation == nullptr) return;
        const auto found = std::find(curveColours.begin(), curveColours.end(), presentation->colour);
        const auto index = found == curveColours.end() ? 0U
            : (static_cast<std::size_t>(std::distance(curveColours.begin(), found)) + 1U) % curveColours.size();
        presentation->colour = curveColours[index];
        side_.dyno.syncRuns();
    };
    dyno.onToggleVisibility = [this] {
        auto* presentation = selectedPresentation();
        if (presentation == nullptr) return;
        presentation->visible = !presentation->visible;
        side_.dyno.syncRuns();
    };
    dyno.onDeleteRun = [this] {
        const auto* run = model_.selectedArchivedRun();
        if (run == nullptr) return;
        const auto selected = model_.selectedRun;
        const auto id = run->id;
        (void)dynoArchive_->erase(id);
        model_.presentations.erase(id);
        collectFinishedRuns();
        model_.selectedRun = model_.archivedRuns.empty() ? -1
            : std::min(selected, static_cast<int>(model_.archivedRuns.size()) - 1);
        side_.dyno.syncRuns();
    };
    dyno.onExportCsv = [this] { exportDynoCsv(); };
    side_.audio.onExhaustPreset = [this](int presetIndex) { applyExhaustPreset(presetIndex); };
    viewport_.setWheelModifierCheck([this] { return wheelModifierDown(); });
    viewport_.setGasFieldSource([this](double crankAngleDegrees, GasFieldSnapshot& field) {
        if (!runtime_) return false;
        runtime_->requestGasField(crankAngleDegrees);
        return runtime_->latestGasField(field);
    });
    viewport_.setConfigEditor([this](const EngineConfig& edited) { return applyEngineEdit(edited); });
    viewport_.setCylinderEditor([this](const EngineConfig& edited, double rampSeconds) {
        return applyCylinderResize(edited, rampSeconds);
    });
    viewport_.onExhaustResolutionChanged = [this] { applyExhaustResolution(); };
    viewport_.onClaimKeyboard = [this] {
        if (!hasKeyboardFocus(false)) grabKeyboardFocus();
    };
    viewport_.onTimeScaleRequested = [this](double timeScale) {
        if (runtime_) runtime_->setTimeScale(timeScale);
    };
    viewport_.setGasProbeSource([this](std::int32_t element, std::uint8_t sample, GasProbeTrace& trace) {
        if (!runtime_) return false;
        runtime_->requestGasProbe(element, sample);
        return runtime_->latestGasProbe(trace);
    });

    for (juce::Component* panel : { static_cast<juce::Component*>(&topBar_),
                                    static_cast<juce::Component*>(&controls_),
                                    static_cast<juce::Component*>(&viewport_),
                                    static_cast<juce::Component*>(&readouts_),
                                    static_cast<juce::Component*>(&side_),
                                    static_cast<juce::Component*>(&status_) })
        addAndMakeVisible(*panel);
    refreshKeyCaps();

    selectEngine(std::min(1, static_cast<int>(presets_.size()) - 1));
    if (!savedEngineErrors.isEmpty())
        showError(utf8("Saved engines skipped"), savedEngineErrors.joinIntoString("\n"));
    startTimerHz(30);
    juce::MessageManager::callAsync([safe = juce::Component::SafePointer<MainComponent>(this)] { if (safe) safe->grabKeyboardFocus(); });
}

MainComponent::~MainComponent() {
    stopTimer();
    stopEngineScriptWatcher();
    audioWorkshopWindow_.reset();
    exhaustDesignerWindow_.reset();
    ecuTunerWindow_.reset();
    shutdownAudio();
    if (runtime_) runtime_->stop();
}

juce::StringArray MainComponent::loadPresets() {
    juce::StringArray errors;
    presets_.clear();
    for (auto& config : makeCatalogOrBasePresets(catalogRoot_))
        presets_.push_back({ std::move(config), std::nullopt, {} });
    auto saved = loadSavedEngines(pathOf(savedEnginesFolder()), catalogRoot_);
    for (const auto& error : saved.errors) errors.add(utf8(error));
    for (auto& engine : saved.engines) {
        // The menu tells engines apart by name.
        if (presetIndexOf(engine.config.name) >= 0) {
            errors.add(fileOf(engine.file).getFileName() + utf8(": another engine is already named \"")
                       + utf8(engine.config.name) + "\"");
            continue;
        }
        presets_.push_back({ std::move(engine.config), std::move(engine.calibration), std::move(engine.file) });
    }
    return errors;
}

void MainComponent::refreshEngineChoices() {
    std::vector<ui::TopBar::EngineChoice> choices;
    choices.reserve(presets_.size());
    for (const auto& preset : presets_)
        choices.push_back({ utf8(preset.config.name), !preset.file.empty(),
                            sessionEdits_.contains(preset.config.name) });
    topBar_.setEngineChoices(std::move(choices));
}

int MainComponent::presetIndexOf(const std::string& name) const noexcept {
    const auto preset = std::find_if(presets_.begin(), presets_.end(),
        [&name](const Preset& candidate) { return candidate.config.name == name; });
    return preset != presets_.end() ? static_cast<int>(std::distance(presets_.begin(), preset)) : -1;
}

std::vector<std::string> MainComponent::catalogueNames() const {
    std::vector<std::string> names;
    for (const auto& preset : presets_)
        if (preset.file.empty()) names.push_back(preset.config.name);
    return names;
}

std::shared_ptr<calibration::CalibrationStore> MainComponent::storeFor(const Preset& preset) {
    if (!preset.calibration) return {};
    auto store = std::make_shared<calibration::CalibrationStore>();
    const auto published = store->publish(*preset.calibration, { 0, "saved-engine" });
    if (published.published) return store;
    showError(utf8("Saved ECU tables rejected"),
              issuesText(published.issues) + utf8("\n\nThe engine runs on the tables its configuration implies."));
    return {};
}

void MainComponent::setBaseline(const EngineConfig& config,
                                const std::optional<calibration::CalibrationDraft>& tables) {
    auto canonical = config;
    normaliseEngineConfig(canonical);
    baseline_ = EngineBaseline(canonical, tables ? *tables : calibration::makeDefaultEcuCalibration(canonical));
    configChanged_ = true;
    seenCalibration_.reset();
    updateModified();
}

void MainComponent::updateModified() {
    if (!runtime_) return;
    if (configChanged_) {
        configModified_ = baseline_.configDiffers(config_);
        configChanged_ = false;
    }
    // Only the ECU window publishes tables: compare them when it has.
    if (auto snapshot = runtime_->calibrationStore()->snapshot(); snapshot != seenCalibration_) {
        calibrationModified_ = baseline_.calibrationDiffers(*snapshot);
        seenCalibration_ = std::move(snapshot);
    }
    topBar_.setModified(configModified_ || calibrationModified_);
}

std::optional<std::pair<std::string, MainComponent::SessionEdit>> MainComponent::unsavedEdit() const {
    if (!runtime_ || selectedPresetIndex_ < 0) return std::nullopt;
    const auto store = runtime_->calibrationStore();
    if (!baseline_.configDiffers(config_) && !baseline_.calibrationDiffers(*store->snapshot())) return std::nullopt;
    return std::pair { presets_[static_cast<std::size_t>(selectedPresetIndex_)].config.name,
                       SessionEdit { config_, store } };
}

bool MainComponent::switchEngine(const EngineConfig& config, std::shared_ptr<calibration::CalibrationStore> store,
                                 const std::optional<calibration::CalibrationDraft>& loadedTables) {
    auto kept = unsavedEdit();
    if (!applyConfig(config, false, false, std::move(store))) return false;
    if (kept && kept->first != config_.name) sessionEdits_.insert_or_assign(kept->first, std::move(kept->second));
    if (selectedPresetIndex_ >= 0) {
        // The edits now run; they are kept again when another engine is chosen.
        const auto& preset = presets_[static_cast<std::size_t>(selectedPresetIndex_)];
        sessionEdits_.erase(preset.config.name);
        setBaseline(preset.config, preset.calibration);
    } else {
        setBaseline(config_, loadedTables);
    }
    refreshEngineChoices();
    return true;
}

void MainComponent::selectEngine(int presetIndex) {
    if (presetIndex < 0 || presetIndex >= static_cast<int>(presets_.size())) return;
    // Choosing the running engine again keeps it as it is; Return reloads it.
    if (presetIndex == selectedPresetIndex_ && runtime_) return;
    const auto& preset = presets_[static_cast<std::size_t>(presetIndex)];
    if (const auto edit = sessionEdits_.find(preset.config.name); edit != sessionEdits_.end()) {
        const auto restored = edit->second;
        (void)switchEngine(restored.config, restored.calibration, std::nullopt);
        return;
    }
    const auto loaded = preset;
    (void)switchEngine(loaded.config, storeFor(loaded), loaded.calibration);
}

bool MainComponent::applyConfig(const EngineConfig& newConfig, bool preserveScriptWatcher,
                                bool preserveCalibration,
                                std::shared_ptr<calibration::CalibrationStore> calibration) {
    if (runtime_ && runtime_->dynoRunning()) {
        showError(utf8("Change rejected"),
            utf8("Stop the dyno before replacing the engine configuration."));
        return false;
    }
    std::unique_ptr<EngineRuntime> replacement;
    EngineConfig canonicalConfig = newConfig;
    auto retainedCalibration = preserveCalibration && runtime_
        ? runtime_->calibrationStore() : std::move(calibration);
    try {
        normaliseEngineConfig(canonicalConfig);
        if (const auto error = validateEngineConfig(canonicalConfig))
            throw std::invalid_argument(*error);
        EngineSimulatorOptions options;
        options.exhaustTargetCellLengthM = viewport_.exhaustCellLengthM();
        replacement = std::make_unique<EngineRuntime>(
            canonicalConfig, retainedCalibration, options, dynoArchive_);
    } catch (const std::exception& error) {
        showError(utf8("Invalid engine configuration"), juce::String::fromUTF8(error.what()));
        return false;
    }
    if (!preserveScriptWatcher) stopEngineScriptWatcher();
    // The ECU window edits one store: another store, another window.
    if (!runtime_ || retainedCalibration != runtime_->calibrationStore()) ecuTunerWindow_.reset();
    shutdownAudio();
    if (runtime_) runtime_->stop();
    collectFinishedRuns();
    audio_.reset(); runtime_.reset();
    config_ = std::move(canonicalConfig);
    const auto diesel = config_.fuel == FuelType::diesel;
    controls_.afrTrim.slider.setRange(diesel ? 0.0 : -3.0, diesel ? 15.0 : 3.0, 0.1);
    if (!preserveCalibration)
        controls_.afrTrim.slider.setValue(0.0, juce::dontSendNotification);
    controls_.setDiesel(diesel);
    adoptAudioVoicing(config_.audioVoicing);
    voicingRevision_ = audioVoicingRevision(catalogRoot_);
    if (exhaustDesignerWindow_) exhaustDesignerWindow_->setConfig(config_);
    renderSnapshotBuilder_ = std::make_unique<RenderSnapshotBuilder>(config_);
    renderSnapshotInterpolator_.reset();
    model_.renderSnapshot = {};
    runtime_ = std::move(replacement);
    audio_ = std::make_unique<RealtimeEngineAudio>(runtime_->audioEvents(), runtime_->audioState(),
        &runtime_->cylinderPressureSamples(), &runtime_->exhaustGraph(),
        &runtime_->engineConfig(), &runtime_->exhaustAcousticSamples());
    model_.telemetry.clear();
    runtime_->setThrottle(controls_.throttle.getValue() / 100.0);
    runtime_->setLoad(controls_.load.slider.getValue() / 100.0);
    runtime_->setAirFuelRatioTrim(controls_.afrTrim.slider.getValue());
    runtime_->setIgnitionTrimDegrees(controls_.sparkTrim.slider.getValue());
    runtime_->setIgnitionEnabled(controls_.ignition.getToggleState());
    runtime_->applyAudioVoicing(currentAudioMix());
    runtime_->setExhaustPreset(static_cast<AudioExhaustPreset>(exhaustPresetIndex_));
    updateAudioControlAvailability();
    configureImpulseResponse();
    if (audioWorkshopWindow_) {
        audioWorkshopWindow_->setEngine(
            config_, catalogRoot_, physicalExhaustTopology_,
            physicalIntakeTopology_, impulseResponseAvailable_);
        audioWorkshopWindow_->setMix(currentAudioMix());
    }
    runtime_->start();
    setAudioChannels(0, 2);
    // Like the old selector's setText(), an engine whose name matches a preset
    // counts as that preset: Enter then reloads it from the catalogue.
    selectedPresetIndex_ = presetIndexOf(config_.name);
    topBar_.setEngine(config_, selectedPresetIndex_);
    viewport_.setEngine(config_);
    viewport_.refresh();
    configChanged_ = true;
    return true;
}

bool MainComponent::applyEngineEdit(const EngineConfig& edited) {
    auto scope = engineEditScope(config_, edited);
    if (!scope.any()) return true;
    if (scope.other || !runtime_ || !audio_ || runtime_->dynoRunning())
        return applyConfig(edited, false, true);
    // Each step either takes its group live or restarts with the whole edit;
    // what is left is measured again after it.
    if (scope.settings && !applySettingsEdit(edited)) return false;
    scope = engineEditScope(config_, edited);
    if (scope.intake && !applyIntakeEdit(edited)) return false;
    scope = engineEditScope(config_, edited);
    if (scope.exhaust && !applyExhaustEdit(edited)) return false;
    scope = engineEditScope(config_, edited);
    if (scope.cylinders) return applyCylinderResize(edited, ui::EngineViewport::cylinderRampSeconds);
    return true;
}

bool MainComponent::applySettingsEdit(const EngineConfig& edited) {
    if (!runtime_ || !audio_ || runtime_->dynoRunning() || !runtime_->applyLiveSettings(edited))
        return applyConfig(edited, false, true);
    config_.injection = runtime_->engineConfig().injection;
    config_.forcedInduction = runtime_->engineConfig().forcedInduction;
    // The simulation has the new settings: a sound that cannot follow them
    // needs the restart after all.
    if (!audio_->replaceForcedInduction(config_.forcedInduction))
        return applyConfig(config_, false, true);
    viewport_.setEngine(config_);
    viewport_.refresh();
    configChanged_ = true;
    return true;
}

bool MainComponent::applyIntakeEdit(const EngineConfig& edited) {
    if (!runtime_ || !audio_ || runtime_->dynoRunning() || !runtime_->applyLiveIntake(edited))
        return applyConfig(edited, false, true);
    const auto& running = runtime_->engineConfig();
    config_.intake = running.intake;
    config_.intakePaths = running.intakePaths;
    config_.plenumVolumeLitres = running.plenumVolumeLitres;
    config_.throttleDiameterMm = running.throttleDiameterMm;
    for (std::size_t index = 0; index < config_.cylinders.size() && index < running.cylinders.size(); ++index) {
        config_.cylinders[index].intakeRunnerLengthMm = running.cylinders[index].intakeRunnerLengthMm;
        config_.cylinders[index].intakeRunnerDiameterMm = running.cylinders[index].intakeRunnerDiameterMm;
    }
    // The simulation has the new intake: a sound that cannot follow it needs
    // the restart after all.
    if (!audio_->replaceIntake(config_)) return applyConfig(config_, false, true);
    viewport_.setEngine(config_);
    viewport_.refresh();
    configChanged_ = true;
    return true;
}

bool MainComponent::applyExhaustEdit(const EngineConfig& edited) {
    if (!runtime_ || !audio_ || runtime_->dynoRunning() || !runtime_->applyLiveExhaust(edited))
        return applyConfig(edited, false, true);
    // The simulation thread swaps the runtime's own graph: the audio gets one
    // built here from the same configuration.
    config_.exhaust = runtime_->engineConfig().exhaust;
    config_.exhaustPaths = runtime_->engineConfig().exhaustPaths;
    try {
        if (!audio_->replaceExhaustGraph(ExhaustGraph::makeForEngine(config_)) && physicalExhaustTopology_)
            return applyConfig(config_, false, true);
    } catch (const std::exception&) {
        return applyConfig(config_, false, true);
    }
    if (exhaustDesignerWindow_) exhaustDesignerWindow_->setConfig(config_);
    viewport_.setEngine(config_);
    viewport_.refresh();
    configChanged_ = true;
    return true;
}

bool MainComponent::applyCylinderResize(const EngineConfig& edited, double rampSeconds) {
    if (!runtime_ || runtime_->dynoRunning() || !runtime_->applyLiveCylinderResize(edited, rampSeconds))
        return applyConfig(edited, false, true);
    // The view shows the target size at once; a ramp reaches it on the
    // simulation thread.
    config_.cylinders = runtime_->engineConfig().cylinders;
    config_.crankJournals = runtime_->engineConfig().crankJournals;
    renderSnapshotBuilder_ = std::make_unique<RenderSnapshotBuilder>(config_);
    renderSnapshotInterpolator_.reset();
    if (exhaustDesignerWindow_) exhaustDesignerWindow_->setConfig(config_);
    topBar_.setEngine(config_, selectedPresetIndex_);
    viewport_.setEngine(config_);
    viewport_.refresh();
    configChanged_ = true;
    return true;
}

void MainComponent::applyExhaustResolution() {
    if (!runtime_) return;
    const auto cellLengthM = viewport_.exhaustCellLengthM().value_or(
        gasdynamics::realtimeExhaustFeedbackDiscretisation().targetCellLengthM);
    if (runtime_->dynoRunning() || !runtime_->applyLiveExhaust(config_, cellLengthM))
        (void) applyConfig(config_, false, true);
}

void MainComponent::updateAudioControlAvailability() {
    physicalExhaustTopology_ = audio_ != nullptr
        && audio_->compiledExhaustTopologyActive();
    physicalIntakeTopology_ = audio_ != nullptr
        && audio_->compiledIntakeTopologyActive();
    structuralRadiationActive_ = audio_ != nullptr
        && audio_->structuralRadiationActive();
    forcedInductionAcousticsActive_ = audio_ != nullptr
        && audio_->forcedInductionAcousticsActive();
    refreshAudioView();
    side_.audio.syncExhaustPreset();
}

void MainComponent::refreshAudioView() {
    auto& view = model_.audio;
    view.mix = currentAudioMix();
    view.exhaustPresetIndex = exhaustPresetIndex_;
    view.physicalExhaustTopology = physicalExhaustTopology_;
    view.physicalIntakeTopology = physicalIntakeTopology_;
    view.structuralRadiationActive = structuralRadiationActive_;
    view.forcedInductionAcousticsActive = forcedInductionAcousticsActive_;
    view.impulseResponseAvailable = impulseResponseAvailable_;
    view.impulseResponseLoadError = impulseResponseLoadError_;
    view.impulseResponseStatus = impulseResponseStatus_;
}

void MainComponent::refreshRuntimeHealth() {
    auto& health = model_.health;
    health.available = runtime_ != nullptr;
    if (runtime_) {
        health.realtimeFactor = runtime_->realtimeFactor();
        health.timeScale = runtime_->timeScale();
        health.paused = runtime_->paused();
        health.droppedFirings = runtime_->droppedEventCount();
        health.droppedPressureSamples = runtime_->droppedPressureSampleCount();
        health.droppedExhaustAcousticSamples = runtime_->droppedExhaustAcousticSampleCount();
        health.droppedDynoCycles = runtime_->droppedBrakeCycleSampleCount();
        health.timingOverruns = runtime_->timingOverrunCount();
        health.loadProtectionActive = runtime_->realtimeLoadProtectionActive();
        health.loadProtectionActivations = runtime_->realtimeLoadProtectionActivationCount();
    }
    health.audioAvailable = audio_ != nullptr;
    if (audio_) {
        health.droppedReactionEvents = audio_->droppedReactionEventCount();
        health.reactionPressureLimitedSamples = audio_->reactionPressureLimitedSampleCount();
        health.lateAudioEvents = audio_->lateEventCount();
        health.droppedPendingAudioEvents = audio_->droppedPendingEventCount();
        health.minimumLevelGain = audio_->minObservedLevelGain();
        health.levelLimitedSamples = audio_->levelLimitedSampleCount();
        health.saturationProcessedSamples = audio_->saturationProcessedSampleCount();
        health.softLimitedSamples = audio_->softLimitedSampleCount();
        health.hardClampedSamples = audio_->hardClampedSampleCount();
        health.maximumPostLimiterMagnitude = audio_->maximumPostLimiterSampleMagnitude();
    }
}

void MainComponent::refreshKeyCaps() {
    controls_.setKeyCaps(actionMap_);
    side_.audio.setKeyCaps(actionMap_);
}

void MainComponent::configureImpulseResponse() {
    if (!audio_) return;
    constexpr juce::int64 maximumIrSamples = 262'144;
    impulseResponseLoadError_ = false;
    impulseResponseStatus_ = "IR free field";
    juce::StringArray errors;
    std::size_t authoredCount = 0;
    std::size_t loadedCount = 0;

    const auto catalogRootFile = juce::File(juce::String(catalogRoot_.string()));
    const auto configuredPaths = config_.exhaustPaths.size();
    // The physical renderer already owns pipe propagation and radiation. An IR
    // is therefore a measured downstream environment/system response, never a
    // preset or geometry-shaped substitute for missing gas physics. Load only
    // paths explicitly authored by the user; absence means true free field.
    for (std::size_t pathIndex = 0; pathIndex < configuredPaths; ++pathIndex) {
        const auto& path = config_.exhaustPaths[pathIndex];
        if (path.impulseResponsePath.empty()) continue;
        ++authoredCount;
        if (pathIndex >= RealtimeConvolutionBank::maximumPaths) {
            errors.add("Path " + juce::String(static_cast<int>(pathIndex + 1))
                + utf8(": more than eight impulse responses."));
            continue;
        }
        const auto configuredPath = juce::String::fromUTF8(
            path.impulseResponsePath.c_str());
        const auto file = juce::File::isAbsolutePath(configuredPath)
            ? juce::File(configuredPath)
            : catalogRootFile.getChildFile(configuredPath);
        auto decoded = loadImpulseResponseFile(file, maximumIrSamples);
        if (decoded.ok()) {
            audio_->setImpulseResponse(
                std::move(decoded.samples), decoded.sampleRateHz, pathIndex);
            ++loadedCount;
        } else {
            juce::String reason;
            switch (decoded.error) {
                case ImpulseResponseLoadError::missingFile:
                    reason = utf8("file not found");
                    break;
                case ImpulseResponseLoadError::unsupportedOrCorrupt:
                    reason = utf8("unreadable format or corrupt file");
                    break;
                case ImpulseResponseLoadError::invalidMetadata:
                    reason = utf8("invalid audio metadata");
                    break;
                case ImpulseResponseLoadError::empty:
                    reason = utf8("empty impulse response");
                    break;
                case ImpulseResponseLoadError::readFailure:
                    reason = utf8("audio read failed");
                    break;
                case ImpulseResponseLoadError::none:
                    reason = utf8("unspecified load failure");
                    break;
            }
            errors.add("Path " + juce::String(static_cast<int>(pathIndex + 1))
                + ": " + reason + ": " + file.getFullPathName());
        }
    }

    if (authoredCount != 0) {
        impulseResponseStatus_ = "IR " + juce::String(static_cast<int>(loadedCount))
            + "/" + juce::String(static_cast<int>(authoredCount)) + utf8(" loaded");
    }
    if (!errors.isEmpty()) {
        impulseResponseLoadError_ = true;
        impulseResponseStatus_ += utf8("  Â·  load error");
        showError(utf8("Impulse response not loaded"),
            errors.joinIntoString("\n")
                + utf8("\n\nThe path stays in free field; no hidden fallback was applied."));
    }
    impulseResponseAvailable_ = loadedCount != 0;
    if (audioWorkshopWindow_) {
        audioWorkshopWindow_->setEngine(
            config_, catalogRoot_, physicalExhaustTopology_,
            physicalIntakeTopology_, impulseResponseAvailable_);
    }
    refreshAudioView();
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
    configEditor_ = std::make_unique<juce::AlertWindow>(utf8("JSON engine editor"),
        utf8("Exhaust, intake, bore and stroke, injector and turbo edits are taken by the running engine; any other edit restarts it. ECU tables are edited live from the ECU window."),
        juce::MessageBoxIconType::NoIcon);
    configEditor_->addTextEditor("json", initialText, {}, false);
    if (auto* editor = configEditor_->getTextEditor("json")) {
        editor->setMultiLine(true, true);
        editor->setReturnKeyStartsNewLine(true);
        editor->setSize(720, 470);
        editor->setFont(juce::FontOptions(14.0F));
    }
    configEditor_->addButton(utf8("APPLY"), 1, juce::KeyPress(juce::KeyPress::returnKey));
    configEditor_->addButton("CANCEL", 0, juce::KeyPress(juce::KeyPress::escapeKey));
    auto safe = juce::Component::SafePointer<MainComponent>(this);
    configEditor_->enterModalState(true, juce::ModalCallbackFunction::create([safe](int result) {
        if (!safe) return;
        juce::String retryText;
        if (result == 1 && safe->configEditor_) {
            if (const auto* editor = safe->configEditor_->getTextEditor("json")) {
                retryText = editor->getText();
                const auto utf8Text = retryText.toRawUTF8();
                auto decoded = safe->jsonSerializer_.decode(utf8Text);
                if (decoded) {
                    (void)restoreAudioVoicing(*decoded.config, safe->catalogRoot_);
                    safe->applyEngineEdit(*decoded.config);
                }
                else safe->showError(utf8("Invalid JSON"), juce::String::fromUTF8(decoded.error.c_str()));
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

void MainComponent::showKeyBindingsEditor() {
    if (keyBindingsEditor_) return;
    keyBindingsEditor_ = std::make_unique<juce::AlertWindow>(utf8("Key bindings"),
        utf8("Edit the JUCE key descriptions. Duplicates and invalid keys are rejected."),
        juce::MessageBoxIconType::NoIcon);
    keyBindingsEditor_->addTextEditor("bindings", actionMap_.toJson(), {}, false);
    if (auto* editor = keyBindingsEditor_->getTextEditor("bindings")) {
        editor->setMultiLine(true);
        editor->setReturnKeyStartsNewLine(true);
        editor->setSize(620, 520);
    }
    keyBindingsEditor_->addButton("APPLY", 1, juce::KeyPress(juce::KeyPress::returnKey,
        juce::ModifierKeys::ctrlModifier, 0));
    keyBindingsEditor_->addButton("CANCEL", 0, juce::KeyPress(juce::KeyPress::escapeKey));
    auto safe = juce::Component::SafePointer<MainComponent>(this);
    keyBindingsEditor_->enterModalState(true, juce::ModalCallbackFunction::create([safe](int result) {
        if (!safe) return;
        if (result == 1 && safe->keyBindingsEditor_) {
            juce::String error;
            if (const auto* editor = safe->keyBindingsEditor_->getTextEditor("bindings");
                !safe->actionMap_.fromJson(editor->getText(), error)) {
                safe->showError("Invalid key bindings", error);
            } else {
                safe->actionMap_.save();
                safe->refreshKeyCaps();
            }
        }
        safe->keyBindingsEditor_.reset();
        safe->grabKeyboardFocus();
    }), true);
}

void MainComponent::showEcuTuner() {
    if (!runtime_) return;
    if (!ecuTunerWindow_)
        ecuTunerWindow_ = std::make_unique<EcuTunerWindow>(runtime_->calibrationStore());
    ecuTunerWindow_->setSessionLocked(runtime_->dynoRunning());
    ecuTunerWindow_->setVisible(true);
    ecuTunerWindow_->toFront(true);
}

OfflineAudioMix MainComponent::currentAudioMix() const noexcept {
    OfflineAudioMix mix;
    mix.volume = audioVolume_;
    mix.convolution = audioConvolution_;
    mix.highFrequencyGain = highFrequencyGain_;
    mix.lowFrequencyGain = lowFrequencyGain_;
    mix.lowFrequencyNoise = lowFrequencyNoise_;
    mix.highFrequencyNoise = highFrequencyNoise_;
    mix.combustionGain = combustionGain_;
    mix.exhaustGain = exhaustGain_;
    mix.intakeGain = intakeGain_;
    mix.mechanicalGain = mechanicalGain_;
    mix.stereoWidth = stereoWidth_;
    mix.outletJetGain = outletJetGain_;
    mix.saturationDrive = saturationDrive_;
    mix.saturationPlacement = saturationPlacement_;
    mix.monitorMode = audioMonitorMode_;
    return mix;
}

void MainComponent::adoptAudioVoicing(const AudioVoicingConfig& voicing) {
    audioVolume_ = voicing.volume;
    audioConvolution_ = voicing.convolution;
    highFrequencyGain_ = voicing.highFrequencyGain;
    lowFrequencyGain_ = voicing.lowFrequencyGain;
    lowFrequencyNoise_ = voicing.lowFrequencyNoise;
    highFrequencyNoise_ = voicing.highFrequencyNoise;
    combustionGain_ = voicing.combustionGain;
    exhaustGain_ = voicing.exhaustGain;
    intakeGain_ = voicing.intakeGain;
    mechanicalGain_ = voicing.mechanicalGain;
    stereoWidth_ = voicing.stereoWidth;
    outletJetGain_ = voicing.outletJetGain;
    saturationDrive_ = voicing.saturationDrive;
    saturationPlacement_ = voicing.saturationPlacement;
    audioMonitorMode_ = voicing.monitorMode;
}

void MainComponent::applyAudioWorkshopMix(
    const OfflineAudioMix& baseMix,
    const OfflineAudioMix& effectiveMix) {
    audioVolume_ = baseMix.volume;
    audioConvolution_ = baseMix.convolution;
    highFrequencyGain_ = baseMix.highFrequencyGain;
    lowFrequencyGain_ = baseMix.lowFrequencyGain;
    lowFrequencyNoise_ = baseMix.lowFrequencyNoise;
    highFrequencyNoise_ = baseMix.highFrequencyNoise;
    combustionGain_ = baseMix.combustionGain;
    exhaustGain_ = baseMix.exhaustGain;
    intakeGain_ = baseMix.intakeGain;
    mechanicalGain_ = baseMix.mechanicalGain;
    stereoWidth_ = baseMix.stereoWidth;
    outletJetGain_ = baseMix.outletJetGain;
    saturationDrive_ = baseMix.saturationDrive;
    saturationPlacement_ = baseMix.saturationPlacement;
    audioMonitorMode_ = baseMix.monitorMode;
    if (runtime_) {
        runtime_->applyAudioVoicing(effectiveMix);
    }
    refreshAudioView();
}

void MainComponent::syncAudioWorkshopMix() {
    if (audioWorkshopWindow_)
        audioWorkshopWindow_->setMix(currentAudioMix());
}

void MainComponent::showAudioWorkshop() {
    if (!runtime_ || !audio_) return;
    if (!audioWorkshopWindow_) {
        auto safe = juce::Component::SafePointer<MainComponent>(this);
        audioWorkshopWindow_ =
            std::make_unique<AudioWorkshopWindow>(
                config_, catalogRoot_, currentAudioMix(),
                physicalExhaustTopology_,
                physicalIntakeTopology_,
                impulseResponseAvailable_,
                [safe](
                    const OfflineAudioMix& baseMix,
                    const OfflineAudioMix& effectiveMix) {
                    if (safe)
                        safe->applyAudioWorkshopMix(
                            baseMix, effectiveMix);
                },
                [safe](const AudioPhysicsSettings& settings) {
                    if (!safe) return false;
                    if (!safe->runtime_ || safe->runtime_->dynoRunning()) {
                        safe->showError(utf8("Change rejected"),
                            utf8("Stop the dyno before changing the physical calibration."));
                        return false;
                    }
                    auto editedConfig = safe->config_;
                    applyAudioPhysicsSettings(editedConfig, settings);
                    normaliseEngineConfig(editedConfig);
                    if (const auto error = validateEngineConfig(editedConfig)) {
                        safe->showError(utf8("Invalid physical calibration"),
                            juce::String::fromUTF8(error->c_str()));
                        return false;
                    }
                    AudioPhysicsCalibration calibration;
                    calibration.cycleVariationCoefficientOfVariation =
                        editedConfig.combustionCalibration
                            .cycleVariationCoefficientOfVariation;
                    calibration.cycleVariationCorrelation =
                        editedConfig.combustionCalibration
                            .cycleVariationCorrelation;
                    calibration.limiterKeepsFuel =
                        editedConfig.ignition.limiterKeepsFuel;
                    calibration.exhaustAfterfire =
                        editedConfig.exhaustAfterfire;
                    safe->config_ = std::move(editedConfig);
                    safe->runtime_->applyAudioPhysicsCalibration(calibration);
                    return true;
                },
                [safe] {
                    if (!safe) return AudioPhysicsTelemetry {};
                    return audioPhysicsTelemetryFor(
                        safe->config_, safe->model_.state);
                });
    } else {
        audioWorkshopWindow_->setEngine(
            config_, catalogRoot_, physicalExhaustTopology_,
            physicalIntakeTopology_,
            impulseResponseAvailable_);
        audioWorkshopWindow_->setMix(currentAudioMix());
    }
    audioWorkshopWindow_->setVisible(true);
    audioWorkshopWindow_->toFront(true);
}

void MainComponent::showExhaustDesigner() {
    if (!runtime_) return;
    if (!exhaustDesignerWindow_) {
        auto safe = juce::Component::SafePointer<MainComponent>(this);
        exhaustDesignerWindow_ = std::make_unique<ExhaustDesignerWindow>(config_,
            [safe](const EngineConfig& editedConfig) {
                if (!safe) return false;
                // applyConfig intentionally keeps the designer alive. Its content
                // invokes this callback synchronously and resumes afterwards.
                return safe->applyExhaustEdit(editedConfig);
            });
    }
    exhaustDesignerWindow_->setVisible(true);
    exhaustDesignerWindow_->toFront(true);
}

void MainComponent::showMoreMenu() {
    const auto running = runtime_ != nullptr && runtime_->dynoRunning();
    auto safe = juce::Component::SafePointer<MainComponent>(this);
    juce::PopupMenu menu;
    const auto add = [&menu, safe](const juce::String& text, bool enabled, const juce::String& shortcut,
                                   void (MainComponent::*action)()) {
        juce::PopupMenu::Item item(text);
        item.isEnabled = enabled;
        item.shortcutKeyDescription = shortcut;
        item.action = [safe, action] { if (safe) (safe.getComponent()->*action)(); };
        menu.addItem(std::move(item));
    };
    const auto savedEngine = selectedPresetIndex_ >= 0
        && !presets_[static_cast<std::size_t>(selectedPresetIndex_)].file.empty();
    add(savedEngine ? utf8("Save engine") : utf8("Save engineâ€¦"), runtime_ != nullptr, {}, &MainComponent::saveEngine);
    add(utf8("Save engine asâ€¦"), runtime_ != nullptr, {}, &MainComponent::saveEngineAs);
    add(utf8("Delete saved engineâ€¦"), savedEngine, {}, &MainComponent::deleteSavedEngine);
    menu.addSeparator();
    add(utf8("Edit engine JSONâ€¦"), !running, {},
        static_cast<void (MainComponent::*)()>(&MainComponent::showConfigEditor));
    add(utf8("Import engineâ€¦"), !running, {}, &MainComponent::importEngine);
    add(utf8("Export engineâ€¦"), true, {}, &MainComponent::exportEngine);
    add(utf8("Export dyno CSVâ€¦"), true, {}, &MainComponent::exportDynoCsv);
    menu.addSeparator();
    add(utf8("Reload engine"), !running, "Return", &MainComponent::reloadEngine);
    add(utf8("Key bindingsâ€¦"), true, {}, &MainComponent::showKeyBindingsEditor);
    juce::PopupMenu::Item fullScreen(utf8("Full screen"));
    fullScreen.shortcutKeyDescription = actionMap_.shortcut(AppAction::fullscreen);
    fullScreen.action = [safe] {
        if (!safe) return;
        if (auto* window = safe->findParentComponentOfClass<juce::DocumentWindow>())
            window->setFullScreen(!window->isFullScreen());
    };
    menu.addItem(std::move(fullScreen));
    menu.showMenuAsync(juce::PopupMenu::Options {}
        .withTargetComponent(&topBar_.moreButton)
        .withMinimumWidth(240));
}

void MainComponent::startEngineScriptWatcher(const std::filesystem::path& path) {
    stopEngineScriptWatcher();
    scriptReloader_ = std::make_unique<scripting::EngineScriptHotReloader>(path);
    scriptRevision_ = 0;
    scriptAttempt_ = 0;
    scriptReloader_->start();
}

void MainComponent::stopEngineScriptWatcher() noexcept {
    if (scriptReloader_) scriptReloader_->stop();
    scriptReloader_.reset();
    scriptRevision_ = 0;
    scriptAttempt_ = 0;
}

void MainComponent::pollEngineScript() {
    if (!scriptReloader_) return;
    const auto reload = scriptReloader_->poll();
    if (!reload || reload->attempt <= scriptAttempt_) return;
    if (reload->lastAttemptSucceeded && reload->config && reload->revision > scriptRevision_) {
        // A structural replacement is deliberately forbidden during a dyno
        // run. Do not consume the successful attempt: the timer will apply it
        // once the run has stopped, without requiring another file save.
        if (runtime_ && runtime_->dynoRunning()) return;
        scriptAttempt_ = reload->attempt;
        if (applyConfig(*reload->config, true, true)) scriptRevision_ = reload->revision;
        return;
    }
    scriptAttempt_ = reload->attempt;
    if (reload->lastAttemptSucceeded || reload->diagnostics.empty()) return;
    juce::String message;
    const auto count = std::min<std::size_t>(reload->diagnostics.size(), 8);
    for (std::size_t index = 0; index < count; ++index) {
        const auto& diagnostic = reload->diagnostics[index];
        if (message.isNotEmpty()) message << "\n";
        message << juce::String(diagnostic.location.source.string()) << ':'
                << juce::String(static_cast<int>(diagnostic.location.line)) << ':'
                << juce::String(static_cast<int>(diagnostic.location.column)) << "  "
                << juce::String(diagnostic.code) << "  " << juce::String(diagnostic.message);
    }
    message << "\n\nThe last valid configuration stays active.";
    showError("Engine script rejected", message);
}

void MainComponent::pollAudioVoicing() {
    // Timer runs at 30 Hz. Filesystem probing once per second stays entirely on
    // the message thread and never enters the realtime callback.
    if (++voicingPollTicks_ < 30U) return;
    voicingPollTicks_ = 0;
    const auto revision = audioVoicingRevision(catalogRoot_);
    if (revision == voicingRevision_) return;
    voicingRevision_ = revision;
    const auto loaded = loadAudioVoicing(catalogRoot_,
        config_.audioVoicingFamily, config_.audioVoicingKey);
    if (!loaded) {
        showError("Audio voicing rejected",
            juce::String::fromUTF8(loaded.error.c_str())
                + "\n\nThe last valid voicing stays active.");
        return;
    }
    config_.audioVoicing = loaded.voicing;
    adoptAudioVoicing(loaded.voicing);
    if (runtime_) runtime_->applyAudioVoicing(loaded.voicing);
    syncAudioWorkshopMix();
    ++voicingReloadCount_;
}

void MainComponent::importEngine() {
    fileChooser_ = std::make_unique<juce::FileChooser>(utf8("Import an engine"), juce::File {},
                                                       "*.json;*.yaml;*.yml;*.els;*.engine");
    auto safe = juce::Component::SafePointer<MainComponent>(this);
    fileChooser_->launchAsync(juce::FileBrowserComponent::openMode | juce::FileBrowserComponent::canSelectFiles,
        [safe](const juce::FileChooser& chooser) {
            if (!safe) return;
            const auto file = chooser.getResult();
            if (file.existsAsFile()) {
                if (file.getSize() > 2 * 1024 * 1024) {
                    safe->showError(utf8("Import failed"), utf8("The file exceeds the 2 MiB limit."));
                    safe->fileChooser_.reset();
                    return;
                }
                if (file.hasFileExtension("els;engine")) {
                    safe->startEngineScriptWatcher(file.getFullPathName().toStdString());
                    safe->fileChooser_.reset();
                    return;
                }
                std::optional<SavedEngine> engine;
                if (file.hasFileExtension("yaml;yml")) {
                    const auto text = file.loadFileAsString();
                    auto decoded = safe->yamlSerializer_.decode(text.toRawUTF8());
                    if (decoded) engine = SavedEngine { std::move(*decoded.config), std::nullopt, pathOf(file) };
                    else safe->showError(utf8("Import failed"), utf8(decoded.error));
                } else {
                    // An engine exported with its ECU tables brings them along.
                    auto read = readSavedEngine(pathOf(file));
                    if (read.engine) engine = std::move(read.engine);
                    else safe->showError(utf8("Import failed"), utf8(read.error));
                }
                if (engine) {
                    if (const auto error = restoreAudioVoicing(engine->config, safe->catalogRoot_))
                        safe->showError(utf8("Voicing not found"), utf8(*error));
                    auto store = safe->storeFor(Preset { engine->config, engine->calibration, engine->file });
                    if (!store) engine->calibration.reset();
                    (void)safe->switchEngine(engine->config, std::move(store), engine->calibration);
                }
            }
            safe->fileChooser_.reset();
        });
}

void MainComponent::exportEngine() {
    fileChooser_ = std::make_unique<juce::FileChooser>(utf8("Export the engine"),
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
                if (file.hasFileExtension("yaml;yml")) {
                    const auto encoded = safe->yamlSerializer_.encode(safe->config_);
                    if (!file.replaceWithText(juce::String::fromUTF8(encoded.data(), static_cast<int>(encoded.size()))))
                        safe->showError(utf8("Export failed"), utf8("The file could not be written."));
                } else if (safe->runtime_) {
                    // The ECU tables go beside it, as a saved engine's do.
                    const auto snapshot = safe->runtime_->calibrationStore()->snapshot();
                    if (const auto error = enginelab::saveEngine(pathOf(file), safe->config_, snapshot.get()))
                        safe->showError(utf8("Export failed"), utf8(*error));
                }
            }
            safe->fileChooser_.reset();
        });
}

void MainComponent::exportDynoCsv() {
    collectFinishedRuns();
    const DynoRun* run = nullptr;
    if (const auto* selected = model_.selectedArchivedRun()) run = selected;
    else if (!model_.archivedRuns.empty()) run = &model_.archivedRuns.back();
    else if (!model_.currentRun.points.empty()) run = &model_.currentRun;
    if (run == nullptr) { showError("CSV DYNO", utf8("No run to export.")); return; }
    juce::String csv;
    csv << "#schema;enginelab-dyno-v3\n"
        << "#run_id;" << juce::String(static_cast<juce::int64>(run->id)) << '\n'
        << "#engine;" << csvField(run->engineName) << '\n'
        << "#status;" << ui::dynoStatusToken(run->status) << '\n'
        << "#stop_reason;" << ui::dynoStopReasonToken(run->stopReason) << '\n'
        << "#mode;" << ui::dynoModeToken(run->sessionConfig.mode) << '\n'
        << "#calibration_revision;"
        << juce::String(static_cast<juce::int64>(run->calibrationRevision)) << '\n'
        << "#started_simulation_s;"
        << juce::String(run->startedAtSimulationSeconds, 6) << '\n'
        << "#ended_simulation_s;"
        << juce::String(run->endedAtSimulationSeconds, 6) << '\n'
        << "#sweep_entry_rpm;"
        << juce::String(run->sessionConfig.sweepEntryRpm, 3) << '\n'
        << "#sweep_ceiling_rpm;"
        << juce::String(run->sessionConfig.sweepCeilingRpm, 3) << '\n'
        << "#ramp_rate_rpm_s;"
        << juce::String(run->sessionConfig.rampRateRpmPerSecond, 3) << '\n'
        << "#bin_width_rpm;"
        << juce::String(run->sessionConfig.binWidthRpm, 3) << '\n'
        << "#rolling_window_s;"
        << juce::String(run->sessionConfig.rollingWindowSeconds, 6) << '\n'
        << "rpm;valid;quality_reasons;bin_rpm;window_mean_rpm;window_min_rpm;window_max_rpm;window_duration_s;torque_variance_nm2;first_cycle_id;last_cycle_id;accepted_cycle_count;"
        << "torque_nm;power_kw;actual_afr;actual_afr_valid;target_afr;lambda;volumetric_efficiency;air_flow_g_s;fuel_flow_g_s;bsfc_g_kwh;map_kpa;"
        << "exhaust_pressure_kpa;coolant_c;oil_c;oil_pressure_kpa;exhaust_c;ignition_advance_deg;correction_factor;"
        << "corrected_torque_nm;corrected_power_kw\n";
    for (const auto& point : run->points) {
        csv << juce::String(point.rpm, 1) << ';' << (point.valid ? "1" : "0") << ';'
            << juce::String(static_cast<juce::int64>(point.qualityReasons)) << ';'
            << juce::String(point.binRpm, 1) << ';' << juce::String(point.windowMeanRpm, 3) << ';'
            << juce::String(point.windowMinimumRpm, 3) << ';' << juce::String(point.windowMaximumRpm, 3) << ';'
            << juce::String(point.windowDurationSeconds, 6) << ';' << juce::String(point.torqueVarianceNm2, 6) << ';'
            << juce::String(static_cast<juce::int64>(point.firstCycleId)) << ';'
            << juce::String(static_cast<juce::int64>(point.lastCycleId)) << ';'
            << juce::String(static_cast<juce::int64>(point.acceptedCycleCount));
        if (!point.valid) {
            for (int column = 0; column < 20; ++column) csv << ';';
            csv << '\n';
            continue;
        }
        csv << ';' << juce::String(point.torqueNm, 2) << ';'
            << juce::String(point.powerKw, 2) << ';' << juce::String(point.airFuelRatio, 2) << ';'
            << (point.airFuelRatioValid ? "1" : "0") << ';'
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
    }
    fileChooser_ = std::make_unique<juce::FileChooser>("Export CSV",
        juce::File::getSpecialLocation(juce::File::userDocumentsDirectory).getChildFile(juce::String(run->engineName) + ".csv"), "*.csv");
    auto safe = juce::Component::SafePointer<MainComponent>(this);
    fileChooser_->launchAsync(juce::FileBrowserComponent::saveMode | juce::FileBrowserComponent::warnAboutOverwriting,
        [safe, csv](const juce::FileChooser& chooser) {
            if (!safe) return;
            auto file = chooser.getResult();
            if (file != juce::File {} && !file.withFileExtension("csv").replaceWithText(csv))
                safe->showError("CSV DYNO", utf8("The file could not be written."));
            safe->fileChooser_.reset();
        });
}

void MainComponent::reloadEngine() {
    if (runtime_ && runtime_->dynoRunning()) {
        showError(utf8("Change rejected"), utf8("Stop the dyno before replacing the engine configuration."));
        return;
    }
    if (selectedPresetIndex_ >= 0) {
        // The engine as it is on disk: its unsaved edits are dropped.
        const auto name = presets_[static_cast<std::size_t>(selectedPresetIndex_)].config.name;
        const auto previousIndex = selectedPresetIndex_;
        const auto errors = loadPresets();
        sessionEdits_.erase(name);
        if (!errors.isEmpty()) showError(utf8("Saved engines skipped"), errors.joinIntoString("\n"));
        auto index = presetIndexOf(name);
        if (index < 0) index = std::clamp(previousIndex, 0, static_cast<int>(presets_.size()) - 1);
        // Not an edit to keep: the selection forgets the running engine first.
        selectedPresetIndex_ = -1;
        const auto loaded = presets_[static_cast<std::size_t>(index)];
        if (!switchEngine(loaded.config, storeFor(loaded), loaded.calibration)) refreshEngineChoices();
        return;
    }
    applyConfig(config_, false, true);
}

void MainComponent::saveEngine() {
    if (!runtime_) return;
    if (selectedPresetIndex_ < 0 || presets_[static_cast<std::size_t>(selectedPresetIndex_)].file.empty()) {
        saveEngineAs();
        return;
    }
    const auto& preset = presets_[static_cast<std::size_t>(selectedPresetIndex_)];
    writeSavedEngine(preset.config.name, preset.file);
}

void MainComponent::saveEngineAs() {
    if (!runtime_ || saveDialog_) return;
    const auto savedEngine = selectedPresetIndex_ >= 0
        && !presets_[static_cast<std::size_t>(selectedPresetIndex_)].file.empty();
    const auto suggested = savedEngine ? utf8(config_.name) : utf8(config_.name) + utf8(" (mine)");
    saveDialog_ = std::make_unique<juce::AlertWindow>(utf8("Save engine"),
        utf8("The engine is saved as it runs now: exhaust, intake, cylinders, every setting and the ECU tables. "
             "It is then listed under My engines."),
        juce::MessageBoxIconType::NoIcon);
    saveDialog_->addTextEditor("name", suggested, utf8("Name"));
    saveDialog_->addButton(utf8("SAVE"), 1, juce::KeyPress(juce::KeyPress::returnKey));
    saveDialog_->addButton(utf8("CANCEL"), 0, juce::KeyPress(juce::KeyPress::escapeKey));
    auto safe = juce::Component::SafePointer<MainComponent>(this);
    saveDialog_->enterModalState(true, juce::ModalCallbackFunction::create([safe](int result) {
        if (!safe) return;
        juce::String name;
        if (const auto* editor = safe->saveDialog_ ? safe->saveDialog_->getTextEditor("name") : nullptr)
            name = editor->getText();
        safe->saveDialog_.reset();
        safe->grabKeyboardFocus();
        if (result == 1) safe->confirmSaveAs(name);
    }), false);
}

void MainComponent::confirmSaveAs(const juce::String& typedName) {
    const auto name = typedName.trim().toStdString();
    if (const auto error = savedEngineNameError(name, catalogueNames())) {
        showError(utf8("Engine not saved"), utf8(*error));
        return;
    }
    const auto file = savedEngineFile(pathOf(savedEnginesFolder()), name);
    // Another saved engine with this name, or this file, is replaced: ask.
    const auto replaced = std::find_if(presets_.begin(), presets_.end(), [&](const Preset& preset) {
        return !preset.file.empty() && (preset.config.name == name || preset.file == file);
    });
    const auto replacesOther = replaced != presets_.end()
        && static_cast<int>(std::distance(presets_.begin(), replaced)) != selectedPresetIndex_;
    if (!replacesOther && !fileOf(file).existsAsFile()) {
        writeSavedEngine(name, file);
        return;
    }
    auto safe = juce::Component::SafePointer<MainComponent>(this);
    juce::AlertWindow::showOkCancelBox(juce::MessageBoxIconType::QuestionIcon, utf8("Replace the saved engine?"),
        utf8("\"") + fileOf(file).getFileNameWithoutExtension()
            + utf8("\" is already saved. Replace it with the running engine?"),
        utf8("Replace"), utf8("Cancel"), nullptr,
        juce::ModalCallbackFunction::create([safe, name, file](int result) {
            if (safe && result == 1) safe->writeSavedEngine(name, file);
        }));
}

void MainComponent::writeSavedEngine(const std::string& name, const std::filesystem::path& file) {
    if (!runtime_) return;
    auto config = config_;
    config.name = name;
    const auto snapshot = runtime_->calibrationStore()->snapshot();
    if (const auto error = enginelab::saveEngine(file, config, snapshot.get())) {
        showError(utf8("Engine not saved"), utf8(*error));
        return;
    }
    // The edits are saved: the engine they were made on is as on disk again.
    if (selectedPresetIndex_ >= 0) sessionEdits_.erase(presets_[static_cast<std::size_t>(selectedPresetIndex_)].config.name);
    sessionEdits_.erase(name);
    std::erase_if(presets_, [&](const Preset& preset) {
        return !preset.file.empty() && (preset.file == file || preset.config.name == name);
    });
    // Saved engines follow the catalogue, by file name, as they load.
    const auto position = std::find_if(presets_.begin(), presets_.end(),
        [&file](const Preset& preset) { return !preset.file.empty() && file < preset.file; });
    auto tables = calibration::makeDraft(*snapshot);
    presets_.insert(position, Preset { config, tables, file });
    // The running engine is the saved one now, without a restart.
    config_.name = name;
    runtime_->renameEngine(name);
    selectedPresetIndex_ = presetIndexOf(name);
    topBar_.setEngine(config_, selectedPresetIndex_);
    setBaseline(config_, tables);
    refreshEngineChoices();
}

void MainComponent::deleteSavedEngine() {
    if (selectedPresetIndex_ < 0) return;
    const auto& preset = presets_[static_cast<std::size_t>(selectedPresetIndex_)];
    if (preset.file.empty()) return;
    auto safe = juce::Component::SafePointer<MainComponent>(this);
    juce::AlertWindow::showOkCancelBox(juce::MessageBoxIconType::QuestionIcon, utf8("Delete the saved engine?"),
        utf8("\"") + utf8(preset.config.name)
            + utf8("\" goes to the recycle bin with its ECU tables. The running engine keeps running: "
                   "Save engine asâ€¦ saves it again."),
        utf8("Delete"), utf8("Cancel"), nullptr,
        juce::ModalCallbackFunction::create([safe, file = preset.file](int result) {
            if (safe && result == 1) safe->removeSavedEngine(file);
        }));
}

void MainComponent::removeSavedEngine(const std::filesystem::path& file) {
    for (const auto& path : { file, savedCalibrationFile(file) }) {
        const auto target = fileOf(path);
        if (target.existsAsFile() && !target.moveToTrash()) {
            showError(utf8("Engine not deleted"), target.getFullPathName() + utf8(" could not be moved to the recycle bin."));
            return;
        }
    }
    const auto removed = std::find_if(presets_.begin(), presets_.end(),
        [&file](const Preset& preset) { return preset.file == file; });
    if (removed == presets_.end()) return;
    sessionEdits_.erase(removed->config.name);
    presets_.erase(removed);
    selectedPresetIndex_ = presetIndexOf(config_.name);
    topBar_.setEngine(config_, selectedPresetIndex_);
    refreshEngineChoices();
}

void MainComponent::prepareToPlay(int blockSize, double sampleRate) { if (audio_) audio_->prepare(sampleRate, blockSize); }
void MainComponent::getNextAudioBlock(const juce::AudioSourceChannelInfo& info) {
    if (audio_ && info.buffer != nullptr) audio_->render(*info.buffer, info.startSample, info.numSamples);
    else if (info.buffer != nullptr) info.buffer->clear(info.startSample, info.numSamples);
}
void MainComponent::releaseResources() { if (audio_) audio_->release(); }

void MainComponent::collectFinishedRuns() {
    const auto revision = dynoArchive_->revision();
    if (revision == visibleDynoArchiveRevision_) return;
    const auto previousCount = model_.archivedRuns.size();
    model_.archivedRuns = dynoArchive_->snapshot();
    for (std::size_t index = 0; index < model_.archivedRuns.size(); ++index)
        ensureDynoPresentation(model_.archivedRuns[index], index);
    visibleDynoArchiveRevision_ = revision;
    const auto count = static_cast<int>(model_.archivedRuns.size());
    // A run that just finished is selected, so its peaks are the ones shown;
    // otherwise the user's selection is kept.
    if (count == 0) model_.selectedRun = -1;
    else if (model_.selectedRun < 0 || model_.selectedRun >= count
             || model_.archivedRuns.size() > previousCount)
        model_.selectedRun = count - 1;
    side_.dyno.syncRuns();
}

void MainComponent::ensureDynoPresentation(
    const DynoRun& run, std::size_t paletteIndex) {
    if (model_.presentations.contains(run.id)) return;
    DynoCurvePresentation presentation;
    presentation.name = juce::String::fromUTF8(run.engineName.c_str())
        + "  #" + juce::String(run.id);
    presentation.colour = curveColours[
        paletteIndex % curveColours.size()];
    model_.presentations.emplace(run.id, std::move(presentation));
}

DynoCurvePresentation* MainComponent::selectedPresentation() {
    const auto* run = model_.selectedArchivedRun();
    if (run == nullptr) return nullptr;
    const auto found = model_.presentations.find(run->id);
    return found != model_.presentations.end() ? &found->second : nullptr;
}

void MainComponent::selectRun(int runIndex) {
    if (runIndex < 0 || runIndex >= static_cast<int>(model_.archivedRuns.size())) return;
    model_.selectedRun = runIndex;
    side_.dyno.syncRuns();
}

void MainComponent::setThrottlePreset(double value) {
    controls_.throttle.setValue(value * 100.0, juce::sendNotificationSync);
}

void MainComponent::updateMomentaryThrottle() {
    double target = 0.0;
    if (actionMap_.isDown(AppAction::throttleFull)) target = 1.0;
    else if (actionMap_.isDown(AppAction::throttleHalf)) target = 0.20;
    else if (actionMap_.isDown(AppAction::throttleQuarter)) target = 0.10;
    else if (actionMap_.isDown(AppAction::throttleIdle)) target = 0.01;
    throttleKeyActive_ = target > 0.0;
    setThrottlePreset(target);
}

void MainComponent::toggleDyno() {
    if (!runtime_) return;
    if (runtime_->dynoRunning()) runtime_->stopDyno(); else runtime_->startDyno();
}

void MainComponent::applyExhaustPreset(int presetIndex) {
    if (physicalExhaustTopology_) return;
    exhaustPresetIndex_ = std::clamp(presetIndex, 0, 4);
    if (runtime_) runtime_->setExhaustPreset(static_cast<AudioExhaustPreset>(exhaustPresetIndex_));
    refreshAudioView();
    side_.audio.syncExhaustPreset();
}

bool MainComponent::wheelModifierDown() const noexcept {
    return actionMap_.isDown(AppAction::wheelDynoRpm)
        || actionMap_.isDown(AppAction::wheelVolume) || actionMap_.isDown(AppAction::wheelConvolution)
        || actionMap_.isDown(AppAction::wheelHighGain) || actionMap_.isDown(AppAction::wheelLowNoise)
        || actionMap_.isDown(AppAction::wheelHighNoise) || actionMap_.isDown(AppAction::wheelCombustion)
        || actionMap_.isDown(AppAction::wheelExhaust) || actionMap_.isDown(AppAction::wheelIntake)
        || actionMap_.isDown(AppAction::wheelMechanical) || actionMap_.isDown(AppAction::wheelSimulationRate)
        || actionMap_.isDown(AppAction::wheelFineThrottle);
}

void MainComponent::adjustAudioOrSimulation(double wheelDelta) {
    if (!runtime_ || wheelDelta == 0.0) return;
    const auto step = actionMap_.isDown(AppAction::wheelFineThrottle) ? 0.01 : 0.05;
    bool audioMixChanged = false;
    bool authoredVoicingChanged = false;
    if (actionMap_.isDown(AppAction::wheelDynoRpm) && model_.state.dynoHoldEnabled) {
        runtime_->adjustDynoHoldRpm(wheelDelta > 0.0 ? 100.0 : -100.0);
    } else if (actionMap_.isDown(AppAction::wheelVolume)) {
        audioVolume_ = std::clamp(audioVolume_ + wheelDelta * step, 0.0, 2.0);
        runtime_->setAudioVolume(audioVolume_);
        audioMixChanged = true;
    } else if (actionMap_.isDown(AppAction::wheelConvolution)) {
        if (!impulseResponseAvailable_) return;
        audioConvolution_ = std::clamp(audioConvolution_ + wheelDelta * step, 0.0, 1.0);
        runtime_->setAudioConvolution(audioConvolution_);
        audioMixChanged = true;
    } else if (actionMap_.isDown(AppAction::wheelHighGain)) {
        highFrequencyGain_ = std::clamp(highFrequencyGain_ + wheelDelta * step, 0.2, 2.5);
        runtime_->setHighFrequencyGain(highFrequencyGain_);
        audioMixChanged = true;
        authoredVoicingChanged = true;
    } else if (actionMap_.isDown(AppAction::wheelLowNoise)) {
        if (physicalIntakeTopology_) return;
        lowFrequencyNoise_ = std::clamp(lowFrequencyNoise_ + wheelDelta * step, 0.0, 1.5);
        runtime_->setLowFrequencyNoise(lowFrequencyNoise_);
        audioMixChanged = true;
        authoredVoicingChanged = true;
    } else if (actionMap_.isDown(AppAction::wheelHighNoise)) {
        if (physicalExhaustTopology_) return;
        highFrequencyNoise_ = std::clamp(highFrequencyNoise_ + wheelDelta * step, 0.0, 1.5);
        runtime_->setHighFrequencyNoise(highFrequencyNoise_);
        audioMixChanged = true;
        authoredVoicingChanged = true;
    } else if (actionMap_.isDown(AppAction::wheelCombustion)) {
        if (physicalExhaustTopology_) return;
        combustionGain_ = std::clamp(combustionGain_ + wheelDelta * step, 0.0, 2.0);
        runtime_->setCombustionGain(combustionGain_);
        audioMixChanged = true;
        authoredVoicingChanged = true;
    } else if (actionMap_.isDown(AppAction::wheelExhaust)) {
        exhaustGain_ = std::clamp(exhaustGain_ + wheelDelta * step, 0.0, 2.0);
        runtime_->setExhaustGain(exhaustGain_);
        audioMixChanged = true;
        authoredVoicingChanged = true;
    } else if (actionMap_.isDown(AppAction::wheelIntake)) {
        intakeGain_ = std::clamp(intakeGain_ + wheelDelta * step, 0.0, 2.0);
        runtime_->setIntakeGain(intakeGain_);
        audioMixChanged = true;
        authoredVoicingChanged = true;
    } else if (actionMap_.isDown(AppAction::wheelMechanical)) {
        mechanicalGain_ = std::clamp(mechanicalGain_ + wheelDelta * step, 0.0, 2.0);
        runtime_->setMechanicalGain(mechanicalGain_);
        audioMixChanged = true;
        authoredVoicingChanged = true;
    } else if (actionMap_.isDown(AppAction::wheelSimulationRate)) {
        runtime_->setTimeScale(runtime_->timeScale() + wheelDelta * step);
    } else if (actionMap_.isDown(AppAction::wheelFineThrottle)) {
        controls_.throttle.setValue(std::clamp(controls_.throttle.getValue() + wheelDelta * 2.0, 0.0, 100.0),
                                    juce::sendNotificationSync);
    }
    if (authoredVoicingChanged) {
        // Physical-reference mode deliberately fixes authored layer/EQ gains
        // at unity. Moving one of those controls is therefore an explicit
        // request for the voiced monitor, exactly as in Audio Workshop.
        audioMonitorMode_ = AudioMonitorMode::captureVoiced;
        runtime_->applyAudioVoicing(currentAudioMix());
    }
    if (audioMixChanged) {
        syncAudioWorkshopMix();
        refreshAudioView();
        // The levels being turned are only visible on the Audio tab.
        side_.showTab(ui::SidePanel::audioTab);
    }
}

bool MainComponent::keyPressed(const juce::KeyPress& key) {
    if (key.getKeyCode() >= juce::KeyPress::F1Key && key.getKeyCode() <= juce::KeyPress::F12Key) {
        const auto presetIndex = key.getKeyCode() - juce::KeyPress::F1Key;
        if (runtime_ && !runtime_->dynoRunning() && presetIndex < static_cast<int>(presets_.size()))
            selectEngine(presetIndex);
        return true;
    }
    if (key == juce::KeyPress::escapeKey) { juce::JUCEApplicationBase::quit(); return true; }
    if (key == juce::KeyPress::returnKey) { reloadEngine(); return true; }
    if (actionMap_.matches(AppAction::nextScreen, key)) { side_.nextTab(); return true; }
    if (actionMap_.matches(AppAction::shiftUp, key)) { if (runtime_) runtime_->shiftUp(); return true; }
    if (actionMap_.matches(AppAction::shiftDown, key)) { if (runtime_) runtime_->shiftDown(); return true; }
    if (actionMap_.matches(AppAction::pause, key)) { if (runtime_) runtime_->setPaused(!runtime_->paused()); return true; }
    if (actionMap_.matches(AppAction::fullscreen, key)) {
        if (auto* window = findParentComponentOfClass<juce::DocumentWindow>())
            window->setFullScreen(!window->isFullScreen());
        return true;
    }
    if (actionMap_.matches(AppAction::dynoStats, key)) { showDynoStats_ = !showDynoStats_; return true; }
    if (actionMap_.matches(AppAction::exhaustPreset, key)) {
        if (!physicalExhaustTopology_) applyExhaustPreset((exhaustPresetIndex_ + 1) % 5);
        return true;
    }
    if (actionMap_.matches(AppAction::layerUp, key)) { viewport_.stepLayer(1); return true; }
    if (actionMap_.matches(AppAction::layerDown, key)) { viewport_.stepLayer(-1); return true; }
    if (actionMap_.matches(AppAction::timeQuarter, key)) { if (runtime_) runtime_->setTimeScale(0.25); return true; }
    if (actionMap_.matches(AppAction::timeHalf, key)) { if (runtime_) runtime_->setTimeScale(0.5); return true; }
    if (actionMap_.matches(AppAction::timeNormal, key)) { if (runtime_) runtime_->setTimeScale(1.0); return true; }
    if (actionMap_.matches(AppAction::timeDouble, key)) { if (runtime_) runtime_->setTimeScale(2.0); return true; }
    if (actionMap_.matches(AppAction::timeQuadruple, key)) { if (runtime_) runtime_->setTimeScale(4.0); return true; }
    if (actionMap_.matches(AppAction::ignition, key)) {
        controls_.ignition.setToggleState(!controls_.ignition.getToggleState(), juce::sendNotificationSync);
        return true;
    }
    if (actionMap_.matches(AppAction::throttleIdle, key)
        || actionMap_.matches(AppAction::throttleFull, key)
        || actionMap_.matches(AppAction::throttleQuarter, key)
        || actionMap_.matches(AppAction::throttleHalf, key)) {
        updateMomentaryThrottle();
        return true;
    }
    if (actionMap_.matches(AppAction::dyno, key)) { toggleDyno(); return true; }
    if (actionMap_.matches(AppAction::dynoHold, key)) { if (runtime_) runtime_->setDynoHoldEnabled(!runtime_->dynoHoldEnabled()); return true; }
    if (actionMap_.matches(AppAction::dynoRamp, key)) { if (runtime_) runtime_->setDynoRampEnabled(!runtime_->dynoRampEnabled()); return true; }
    if (actionMap_.matches(AppAction::clutchDecrease, key)) {
        targetClutchPressure_ = std::clamp(targetClutchPressure_ - 0.08, 0.0, 1.0);
        return true;
    }
    if (actionMap_.matches(AppAction::clutchIncrease, key)) {
        targetClutchPressure_ = std::clamp(targetClutchPressure_ + 0.08, 0.0, 1.0);
        return true;
    }
    return actionMap_.matches(AppAction::starter, key);
}

bool MainComponent::keyStateChanged(bool) {
    const auto anyThrottleDown = actionMap_.isDown(AppAction::throttleIdle)
        || actionMap_.isDown(AppAction::throttleQuarter)
        || actionMap_.isDown(AppAction::throttleHalf)
        || actionMap_.isDown(AppAction::throttleFull);
    if (anyThrottleDown || throttleKeyActive_) updateMomentaryThrottle();
    const auto down = actionMap_.isDown(AppAction::starter);
    if (down != starterKeyDown_) {
        starterKeyDown_ = down;
        if (runtime_) runtime_->setStarterEngaged(down || controls_.starter.getState() == juce::Button::buttonDown);
        controls_.starter.setEngaged(down);
    }
    const auto brakeDown = actionMap_.isDown(AppAction::brake);
    if (brakeDown != brakeKeyDown_) {
        brakeKeyDown_ = brakeDown;
        if (runtime_) runtime_->setBrakePressure(brakeDown ? 1.0 : 0.0);
    }
    return down || brakeDown || anyThrottleDown || actionMap_.isDown(AppAction::clutchHold)
        || juce::ModifierKeys::getCurrentModifiersRealtime().isShiftDown();
}

void MainComponent::focusLost(FocusChangeType) {
    throttleKeyActive_ = false;
    setThrottlePreset(0.0);
    if (runtime_) {
        runtime_->setStarterEngaged(false);
        runtime_->setBrakePressure(0.0);
    }
    starterKeyDown_ = false;
    brakeKeyDown_ = false;
    controls_.starter.setEngaged(false);
}

void MainComponent::mouseWheelMove(const juce::MouseEvent&, const juce::MouseWheelDetails& wheel) {
    // Panels that do not use the wheel themselves pass it up to here.
    adjustAudioOrSimulation(wheel.deltaY);
}

void MainComponent::timerCallback() {
    pollEngineScript();
    pollAudioVoicing();
    if (!runtime_) return;
    // A live exhaust change retires the audio's previous network here, off
    // the audio thread.
    if (audio_) audio_->collectRetiredExhaustNetworks();
    if (throttleKeyActive_) updateMomentaryThrottle();
    const auto clutchTarget = (actionMap_.isDown(AppAction::clutchHold)
        || juce::ModifierKeys::getCurrentModifiersRealtime().isShiftDown())
        ? 0.0 : targetClutchPressure_;
    const auto slowClutch = actionMap_.isDown(AppAction::wheelFineThrottle);
    const auto clutchRate = slowClutch ? 2.2 : 12.0;
    currentClutchPressure_ += (clutchTarget - currentClutchPressure_) * (1.0 - std::exp(-clutchRate / 30.0));
    runtime_->setClutchPressure(currentClutchPressure_);
    model_.state = runtime_->snapshot();
    if (ecuTunerWindow_) {
        const auto normalizedLoad = std::clamp(model_.state.manifoldPressureKpa
            / std::max(1.0, config_.ambientPressureKpa),
            calibration::ecuLimits::minimumNormalizedLoad,
            calibration::ecuLimits::maximumNormalizedLoad);
        ecuTunerWindow_->setOperatingPoint(model_.state.rpm, normalizedLoad);
    }
    if (renderSnapshotBuilder_) {
        renderSnapshotInterpolator_.push(renderSnapshotBuilder_->build(model_.state));
        model_.renderSnapshot = renderSnapshotInterpolator_.sample(model_.state.simulationTimeSeconds);
    }
    model_.telemetry.push(model_.state);
    model_.currentRun = runtime_->currentDynoRun();
    model_.diagnostics = diagnostics_.evaluate(config_, model_.state);
    model_.dynoRunning = runtime_->dynoRunning();
    model_.scriptLive = scriptReloader_ != nullptr;
    model_.scriptRevision = scriptRevision_;
    model_.voicingReloadCount = voicingReloadCount_;
    refreshRuntimeHealth();
    refreshAudioView();
    collectFinishedRuns();
    const auto running = model_.dynoRunning;
    if (ecuTunerWindow_) ecuTunerWindow_->setSessionLocked(running);
    topBar_.ecuButton.setEnabled(!running);
    topBar_.exhaustButton.setEnabled(!running);
    updateModified();

    // Each panel repaints only its own area; the static chrome does not
    // repaint at all unless its content changed.
    topBar_.refresh(model_);
    controls_.refresh(model_);
    viewport_.refresh();
    readouts_.repaint();
    side_.refresh();
    status_.refresh();
}

void MainComponent::paint(juce::Graphics& g) {
    g.fillAll(ui::colours::background);
}

void MainComponent::resized() {
    auto area = getLocalBounds();
    topBar_.setBounds(area.removeFromTop(ui::TopBar::height));
    status_.setBounds(area.removeFromBottom(ui::StatusBar::height));
    controls_.setBounds(area.removeFromLeft(ui::ControlPanel::width));
    side_.setBounds(area.removeFromRight(ui::SidePanel::width));
    readouts_.setBounds(area.removeFromBottom(ui::ReadoutStrip::height));
    viewport_.setBounds(area);
}
} // namespace enginelab
