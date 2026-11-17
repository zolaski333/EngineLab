#include <enginelab/app/AudioWorkshopWindow.hpp>
#include <enginelab/app/EcuTunerWindow.hpp>
#include <enginelab/app/EquipmentWindow.hpp>
#include <enginelab/calibration/CalibrationStore.hpp>
#include <enginelab/calibration/EcuCalibration.hpp>
#include <enginelab/calibration/EcuCalibrationKeys.hpp>
#include <enginelab/app/ExhaustDesignerWindow.hpp>
#include <enginelab/app/FuelEditing.hpp>
#include <enginelab/app/FuelWindow.hpp>
#include <enginelab/app/PassByWindow.hpp>
#include <enginelab/app/Widgets.hpp>
#include <enginelab/app/Theme.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/exhaust/ExhaustPathTopologyEditor.hpp>
#include <enginelab/exhaust/LegacyExhaustNetwork.hpp>

#include <juce_gui_extra/juce_gui_extra.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <optional>
#include <iostream>
#include <memory>
#include <string_view>
#include <vector>

namespace {

void require(bool condition, std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

[[nodiscard]] bool near(double left, double right) {
    return std::abs(left - right) <= 1.0e-12;
}

} // namespace

int main() {
    juce::ScopedJuceInitialiser_GUI initialiseGui;
    const auto catalog =
        enginelab::loadEngineCatalog(ENGINELAB_CATALOG_ROOT);
    require(!catalog.entries.empty(), "catalog loads");
    const auto& engine = catalog.entries.front().config;
    const auto audioLab = std::find_if(
        catalog.entries.begin(), catalog.entries.end(), [](const auto& entry) {
            return entry.config.name == "Audio Physics Lab 689 Twin";
        });
    require(audioLab != catalog.entries.end(),
            "catalog exposes the selectable audio physics lab engine");
    const auto labSettings =
        enginelab::audioPhysicsSettingsFor(audioLab->config);
    require(
        !labSettings.afterfireEnabled && !labSettings.limiterKeepsFuel
            && near(labSettings.afterfireReactionTimeSeconds, 0.0005)
            && near(labSettings.overrunSparkAdvanceDegrees, 10.0)
            && near(labSettings.pocketMixingTimeSeconds, 0.040)
            && near(labSettings.overrunSparkCutFraction, 0.70)
            && audioLab->config.exhaustAfterfire.strategy
                == enginelab::ExhaustAfterfireStrategy::cleanDfco
            && near(labSettings.cycleVariationCoefficientOfVariation, 0.06),
        "audio lab engine carries the measured backfire calibration, backfires off");
    require(
        enginelab::audioPhysicsTelemetryFor(
            audioLab->config, enginelab::EngineState {}).porousMufflerCount == 1,
        "audio lab engine contains one active absorptive silencer");

    auto physicsConfig = engine;
    enginelab::AudioPhysicsSettings authoredPhysics {
        0.06, 0.55, true, true, 800.0, 0.008, 0.95, -25.0, 0.012
    };
    enginelab::applyAudioPhysicsSettings(physicsConfig, authoredPhysics);
    const auto recoveredPhysics =
        enginelab::audioPhysicsSettingsFor(physicsConfig);
    require(
        near(recoveredPhysics.cycleVariationCoefficientOfVariation, 0.06)
            && near(recoveredPhysics.cycleVariationCorrelation, 0.55)
            && recoveredPhysics.afterfireEnabled
            && recoveredPhysics.limiterKeepsFuel
            && near(recoveredPhysics.afterfireIgnitionTemperatureK, 800.0)
            && near(recoveredPhysics.afterfireReactionTimeSeconds, 0.008)
            && near(recoveredPhysics.afterfireEfficiency, 0.95)
            && near(recoveredPhysics.overrunSparkAdvanceDegrees, -25.0)
            && near(recoveredPhysics.pocketMixingTimeSeconds, 0.012)
            && physicsConfig.exhaustAfterfire.strategy
                == enginelab::ExhaustAfterfireStrategy::lateSparkOverrun
            && near(physicsConfig.exhaustAfterfire
                        .inductionActivationTemperatureK, 13'340.0)
            && near(physicsConfig.exhaustAfterfire.inductionPressureExponent,
                    0.989)
            && near(physicsConfig.exhaustAfterfire
                        .inductionEquivalenceRatioExponent, -0.577),
        "audio physics controls round-trip through EngineConfig");
    enginelab::EngineState physicsState;
    physicsState.cylinderStates[0].combustionCycleMultiplier = 0.91;
    physicsState.cylinderStates[1].combustionCycleMultiplier = 1.08;
    physicsState.exhaustAfterfireHeatReleaseKw = 3.4;
    physicsState.exhaustAfterfireFuelBurnMgPerSecond = 22.0;
    physicsState.exhaustAfterfireOverrunActive = true;
    const auto physicsTelemetry =
        enginelab::audioPhysicsTelemetryFor(physicsConfig, physicsState);
    require(
        near(physicsTelemetry.minimumCycleMultiplier, 0.91)
            && near(physicsTelemetry.maximumCycleMultiplier, 1.08)
            && near(physicsTelemetry.afterfireHeatReleaseKw, 3.4)
            && near(physicsTelemetry.afterfireFuelBurnMgPerSecond, 22.0)
            && physicsTelemetry.overrunAfterfireActive,
        "audio physics telemetry reports real simulator state");

    enginelab::OfflineAudioMix base;
    base.volume = 1.25;
    base.convolution = 0.60;
    base.highFrequencyGain = 1.10;
    base.lowFrequencyGain = 1.23;
    base.lowFrequencyNoise = 0.45;
    base.highFrequencyNoise = 0.55;
    base.combustionGain = 1.20;
    base.exhaustGain = 1.30;
    base.intakeGain = 0.90;
    base.mechanicalGain = 0.80;
    base.stereoWidth = 1.31;
    base.outletJetGain = 0.77;
    base.saturationDrive = 0.42;
    base.saturationPlacement =
        enginelab::AudioSaturationPlacement::preShelf;

    enginelab::AudioWorkshopAvailability physical {
        true, true, false
    };
    enginelab::AudioWorkshopLayerSwitches switches;
    auto effective = enginelab::effectiveAudioWorkshopMix(
        base, physical, switches);
    require(
        near(effective.convolution, 0.0)
            && near(effective.lowFrequencyNoise, 0.0)
            && near(effective.highFrequencyNoise, 0.0)
            && near(effective.combustionGain, 0.0),
        "unavailable physical-path controls are neutralised");
    require(
        near(effective.exhaustGain, base.exhaustGain)
            && near(effective.intakeGain, base.intakeGain)
            && near(effective.mechanicalGain, base.mechanicalGain),
        "active physical source faders remain audible");

    physical.measuredImpulseResponseAvailable = true;
    switches.solo[1] = true;
    effective = enginelab::effectiveAudioWorkshopMix(
        base, physical, switches);
    require(
        near(effective.convolution, base.convolution)
            && near(effective.exhaustGain, base.exhaustGain)
            && near(effective.intakeGain, 0.0)
            && near(effective.mechanicalGain, 0.0),
        "exhaust solo preserves IR return and mutes other sources");
    switches.muted[1] = true;
    effective = enginelab::effectiveAudioWorkshopMix(
        base, physical, switches);
    require(
        near(effective.exhaustGain, 0.0)
            && near(effective.intakeGain, 0.0)
            && near(effective.mechanicalGain, 0.0),
        "mute wins over solo");

    int callbackCount = 0;
    enginelab::OfflineAudioMix callbackBase;
    enginelab::OfflineAudioMix callbackEffective;
    int physicsApplyCount = 0;
    bool refusePhysics = false;
    enginelab::AudioPhysicsSettings appliedPhysics;
    enginelab::AudioWorkshopWindow window(
        engine, ENGINELAB_CATALOG_ROOT, base,
        true, true, false,
        [&](const enginelab::OfflineAudioMix& nextBase,
            const enginelab::OfflineAudioMix& nextEffective) {
            ++callbackCount;
            callbackBase = nextBase;
            callbackEffective = nextEffective;
        },
        [&](const enginelab::AudioPhysicsSettings& settings) {
            ++physicsApplyCount;
            if (refusePhysics) return false;
            appliedPhysics = settings;
            return true;
        },
        [physicsTelemetry] { return physicsTelemetry; });
    auto* content = window.getContentComponent();
    require(content != nullptr, "workshop owns content");
    const auto verifyLayoutAt = [&](int width, int height) {
        window.setBounds(20, 20, width, height);
        window.resized();
        content->resized();
        require(
            content->getWidth() > 0
                && content->getHeight() > 0,
            "workshop content receives usable dimensions");
        for (int index = 0;
             index < content->getNumChildComponents();
             ++index) {
            const auto* child =
                content->getChildComponent(index);
            if (!child->isVisible()) continue;
            require(
                child->getWidth() > 0
                    && child->getHeight() > 0,
                "every visible workshop control has non-zero bounds");
            require(
                content->getLocalBounds().contains(
                    child->getBounds()),
                "every visible workshop control stays inside the window");
        }
    };
    verifyLayoutAt(1'100, 800);
    verifyLayoutAt(1'280, 860);
    verifyLayoutAt(1'600, 1'000);

    int visibleChildren = 0;
    int sliderCount = 0;
    int disabledSliderCount = 0;
    int muteCount = 0;
    int soloCount = 0;
    int disabledMuteCount = 0;
    int disabledSoloCount = 0;
    bool exportButtonFound = false;
    juce::TextButton* catalogueMixButton = nullptr;
    juce::TextButton* neutralMixButton = nullptr;
    juce::TextButton* demoPhysicsButton = nullptr;
    juce::TextButton* applyPhysicsButton = nullptr;
    juce::ToggleButton* afterfireToggle = nullptr;
    juce::ToggleButton* mixAfterfireToggle = nullptr;
    juce::ToggleButton* captureModeToggle = nullptr;
    juce::Slider* exhaustMixSlider = nullptr;
    juce::TextButton* loopPackButton = nullptr;
    for (int index = 0;
         index < content->getNumChildComponents(); ++index) {
        auto* child = content->getChildComponent(index);
        if (!child->isVisible()) continue;
        ++visibleChildren;
        if (const auto* slider =
                dynamic_cast<juce::Slider*>(child)) {
            ++sliderCount;
            if (!slider->isEnabled()) ++disabledSliderCount;
        }
        if (auto* slider = dynamic_cast<juce::Slider*>(child);
            slider != nullptr && slider->getComponentID() == "mix-exhaust")
            exhaustMixSlider = slider;
        if (auto* button =
                dynamic_cast<juce::TextButton*>(child)) {
            const auto text = button->getButtonText();
            if (text == "MUTE") {
                ++muteCount;
                if (!button->isEnabled()) ++disabledMuteCount;
            } else if (text == "SOLO") {
                ++soloCount;
                if (!button->isEnabled()) ++disabledSoloCount;
            } else if (text
                       == "CHOOSE FOLDER AND EXPORT") {
                exportButtonFound = true;
            } else if (text == "AUDIBLE DEMO") {
                demoPhysicsButton = button;
            } else if (text == "VOICING CATALOGUE") {
                catalogueMixButton = button;
            } else if (text == "NEUTRAL") {
                neutralMixButton = button;
            } else if (text == "APPLY LIVE") {
                applyPhysicsButton = button;
            } else if (text == "LOOP PACK FOR GAMES...") {
                loopPackButton = button;
            }
        }
        if (auto* toggle =
                dynamic_cast<juce::ToggleButton*>(child)) {
            if (toggle->getComponentID() == "afterfire-enabled")
                afterfireToggle = toggle;
            else if (toggle->getComponentID() == "mix-afterfire")
                mixAfterfireToggle = toggle;
            else if (toggle->getComponentID() == "monitor-capture")
                captureModeToggle = toggle;
        }
    }
    require(visibleChildren >= 45, "complete workshop control set is present");
    require(loopPackButton != nullptr && loopPackButton->isEnabled()
            && loopPackButton->getHeight() >= 24
            && content->getLocalBounds().contains(loopPackButton->getBounds()),
        "the loop pack export sits in the export panel, ready");
    require(
        sliderCount == 17 && disabledSliderCount == 4,
        "nine faders and eight physics controls exist; four no-op faders are disabled");
    require(
        muteCount == 4 && soloCount == 4
            && disabledMuteCount == 1
            && disabledSoloCount == 1,
        "four source mute/solo rows exist and direct combustion is unavailable");
    require(exportButtonFound, "HQ export action is visible");
    require(catalogueMixButton != nullptr && neutralMixButton != nullptr,
            "catalogue and neutral voicing A/B actions are visible");
    require(demoPhysicsButton != nullptr && applyPhysicsButton != nullptr
                && afterfireToggle != nullptr,
            "audible physics A/B and explicit apply actions are visible");
    require(mixAfterfireToggle != nullptr,
            "afterfire is switchable from the real-time mix group");
    require(!mixAfterfireToggle->getToggleState()
                && !engine.exhaustAfterfire.enabled,
            "the mix switch shows the catalogue default (afterfire off)");
    require(captureModeToggle != nullptr,
            "the effective monitor mode is visible to the user");
    require(exhaustMixSlider != nullptr,
            "the physical exhaust fader is addressable");

    exhaustMixSlider->setValue(base.exhaustGain + 0.01,
                               juce::sendNotificationSync);
    require(
        captureModeToggle->getToggleState()
            && callbackBase.monitorMode
                == enginelab::AudioMonitorMode::captureVoiced
            && near(callbackBase.exhaustGain, base.exhaustGain + 0.01),
        "editing an authored fader visibly enables the mode that honours it");

    afterfireToggle->setToggleState(true, juce::dontSendNotification);
    afterfireToggle->onClick();
    require(!mixAfterfireToggle->getToggleState() && physicsApplyCount == 0,
            "a pending physics edit does not show as live in the mix switch");
    applyPhysicsButton->onClick();
    require(
        physicsApplyCount == 1
            && appliedPhysics.afterfireEnabled
            && near(appliedPhysics.afterfireIgnitionTemperatureK, 800.0)
            && near(appliedPhysics.afterfireReactionTimeSeconds, 0.0005)
            && near(appliedPhysics.overrunSparkAdvanceDegrees, 10.0)
            && near(appliedPhysics.pocketMixingTimeSeconds, 0.040)
            && near(appliedPhysics.overrunSparkCutFraction, 0.70),
        "enabling a clean afterfire installs the measured backfire profile");
    require(mixAfterfireToggle->getToggleState(),
            "the mix switch mirrors an afterfire applied from the physics group");

    demoPhysicsButton->onClick();
    require(
        physicsApplyCount == 2
            && near(appliedPhysics.cycleVariationCoefficientOfVariation, 0.06)
            && appliedPhysics.afterfireEnabled
            && appliedPhysics.limiterKeepsFuel
            && near(appliedPhysics.afterfireReactionTimeSeconds, 0.0005)
            && near(appliedPhysics.overrunSparkAdvanceDegrees, 10.0)
            && near(appliedPhysics.pocketMixingTimeSeconds, 0.040)
            && near(appliedPhysics.overrunSparkCutFraction, 0.70),
        "demo action publishes an intentionally audible physical calibration");

    mixAfterfireToggle->setToggleState(false, juce::dontSendNotification);
    mixAfterfireToggle->onClick();
    require(
        physicsApplyCount == 3
            && !appliedPhysics.afterfireEnabled
            && !afterfireToggle->getToggleState(),
        "switching afterfire off in the mix applies at once and mirrors the physics group");
    mixAfterfireToggle->setToggleState(true, juce::dontSendNotification);
    mixAfterfireToggle->onClick();
    require(
        physicsApplyCount == 4
            && appliedPhysics.afterfireEnabled
            && afterfireToggle->getToggleState()
            && near(appliedPhysics.overrunSparkAdvanceDegrees, 10.0)
            && near(appliedPhysics.pocketMixingTimeSeconds, 0.040)
            && near(appliedPhysics.overrunSparkCutFraction, 0.70),
        "switching afterfire back on applies at once and keeps the calibration");
    refusePhysics = true;
    mixAfterfireToggle->setToggleState(false, juce::dontSendNotification);
    mixAfterfireToggle->onClick();
    require(
        physicsApplyCount == 5
            && mixAfterfireToggle->getToggleState()
            && afterfireToggle->getToggleState(),
        "a refused switch shows the afterfire state that is actually live");
    refusePhysics = false;

    window.setMix(base);
    require(callbackCount > 0, "mix synchronisation publishes to MainComponent");
    require(
        near(callbackBase.volume, base.volume)
            && near(callbackBase.exhaustGain, base.exhaustGain)
            && near(callbackBase.lowFrequencyGain, base.lowFrequencyGain)
            && near(callbackBase.stereoWidth, base.stereoWidth)
            && near(callbackBase.outletJetGain, base.outletJetGain)
            && near(callbackBase.saturationDrive, base.saturationDrive)
            && callbackBase.saturationPlacement
                == enginelab::AudioSaturationPlacement::preShelf,
        "visible and hidden catalogue voicing fields are preserved");
    require(
        near(callbackEffective.convolution, 0.0)
            && near(callbackEffective.combustionGain, 0.0)
            && near(callbackEffective.exhaustGain, base.exhaustGain),
        "window publishes topology-aware effective mix");

    neutralMixButton->onClick();
    require(
        near(callbackBase.lowFrequencyGain, 1.0)
            && near(callbackBase.stereoWidth, 1.0)
            && near(callbackBase.outletJetGain, 1.0)
            && near(callbackBase.saturationDrive, 0.0),
        "neutral A/B action bypasses every catalogue voicing field");
    catalogueMixButton->onClick();
    require(
        near(callbackBase.lowFrequencyGain, base.lowFrequencyGain)
            && near(callbackBase.stereoWidth, base.stereoWidth)
            && near(callbackBase.outletJetGain, base.outletJetGain)
            && near(callbackBase.saturationDrive, base.saturationDrive),
        "catalogue A/B action restores the complete authored voicing");

    window.setEngine(
        engine, ENGINELAB_CATALOG_ROOT,
        false, false, true);
    require(
        near(callbackEffective.convolution, base.convolution)
            && near(callbackEffective.lowFrequencyNoise,
                    base.lowFrequencyNoise)
            && near(callbackEffective.highFrequencyNoise,
                    base.highFrequencyNoise)
            && near(callbackEffective.combustionGain,
                    base.combustionGain),
        "compatibility topology re-enables its real controls");

    // Another engine brings its own physics fields: APPLY LIVE sent the
    // last engine's to it (audit APP-04).
    auto otherEngine = engine;
    otherEngine.combustionCalibration.cycleVariationCoefficientOfVariation = 0.03;
    window.setEngine(otherEngine, ENGINELAB_CATALOG_ROOT, false, false, true);
    refusePhysics = false;
    applyPhysicsButton->onClick();
    require(near(appliedPhysics.cycleVariationCoefficientOfVariation, 0.03),
        "the physics fields follow the engine the workshop is given");

    // The exhaust designer draws a rotor's port passage inside its source,
    // so the passage has no node: painting it as one threw out of paint()
    // and closed the app on a rotary's Exhaust button (owner report
    // 2026-10-31).
    const auto rotary = std::find_if(
        catalog.entries.begin(), catalog.entries.end(), [](const auto& entry) {
            return entry.config.cycle == enginelab::EngineCycle::rotary;
        });
    require(rotary != catalog.entries.end(), "catalog has a rotary");
    enginelab::ExhaustDesignerWindow exhaustDesigner(
        rotary->config, [](const enginelab::EngineConfig&) { return true; });
    auto* designerContent = exhaustDesigner.getContentComponent();
    require(designerContent != nullptr && !designerContent->getBounds().isEmpty(),
            "exhaust designer lays out");
    const auto snapshot = designerContent->createComponentSnapshot(
        designerContent->getLocalBounds());
    require(snapshot.isValid(), "exhaust designer paints a rotary");

    // Owner report 2026-11-11: with a cylinder's default pipe deleted,
    // nothing else could be put in its place. A cylinder is a source in From.
    {
        auto config = enginelab::makeDefaultInlineFour();
        auto& path = config.exhaustPaths.front();
        path.network = enginelab::makeEditableExhaustNetwork(path);
        // Shown as 763.3: selecting it then another part must not round it.
        const auto tail = std::find_if(path.network->components.begin(), path.network->components.end(),
            [](const auto& item) { return item.type == enginelab::ExhaustComponentType::outlet; });
        require(tail != path.network->components.end(), "the editable network has an outlet");
        tail->lengthMm = 763.27;
        const auto tailId = tail->id;
        const auto tailRow = static_cast<int>(tail - path.network->components.begin());
        const auto original = *path.network;
        const auto cylinderId = path.cylinderIds.front();
        const auto mapping = std::find_if(original.cylinderConnections.begin(), original.cylinderConnections.end(),
            [cylinderId](const auto& item) { return item.cylinderId == cylinderId; });
        require(mapping != original.cylinderConnections.end(), "the editable network maps the first cylinder");
        const auto primaryId = mapping->componentId;
        const auto out = std::find_if(original.connections.begin(), original.connections.end(),
            [primaryId](const auto& edge) { return edge.fromComponentId == primaryId; });
        require(out != original.connections.end(), "the first cylinder's primary leads on");
        const auto downstreamId = out->toComponentId;

        std::optional<enginelab::EngineConfig> applied;
        enginelab::ExhaustDesignerWindow designer(config, [&applied](const enginelab::EngineConfig& edited) {
            applied = edited;
            return true;
        });
        auto* designerView = designer.getContentComponent();
        require(designerView != nullptr, "the exhaust designer has designerView");
        const auto child = [designerView](juce::StringRef id) {
            for (auto* item : designerView->getChildren())
                if (item->getComponentID() == id) return item;
            return static_cast<juce::Component*>(nullptr);
        };
        auto* list = dynamic_cast<juce::ListBox*>(child("exhaust-components"));
        auto* addType = dynamic_cast<juce::ComboBox*>(child("exhaust-add-type"));
        auto* add = dynamic_cast<juce::Button*>(child("exhaust-add"));
        auto* remove = dynamic_cast<juce::Button*>(child("exhaust-delete-component"));
        auto* from = dynamic_cast<juce::ComboBox*>(child("exhaust-from"));
        auto* to = dynamic_cast<juce::ComboBox*>(child("exhaust-to"));
        auto* connect = dynamic_cast<juce::Button*>(child("exhaust-connect"));
        auto* undo = dynamic_cast<juce::Button*>(child("exhaust-undo"));
        auto* redo = dynamic_cast<juce::Button*>(child("exhaust-redo"));
        auto* applyDesign = dynamic_cast<juce::Button*>(child("exhaust-apply"));
        require(list && addType && add && remove && from && to && connect && undo && redo && applyDesign,
                "the exhaust designer shows its editing controls");
        require(!undo->isEnabled() && !redo->isEnabled(), "nothing to undo before an edit");

        // The working copy's component order: the deleted pipe goes, the
        // added one comes last.
        std::vector<std::uint32_t> order;
        for (const auto& component : original.components)
            if (component.id != primaryId) order.push_back(component.id);
        const auto rowOf = [&order](std::uint32_t id) {
            return static_cast<int>(std::find(order.begin(), order.end(), id) - order.begin());
        };
        const auto primaryRow = static_cast<int>(std::find_if(original.components.begin(), original.components.end(),
            [primaryId](const auto& item) { return item.id == primaryId; }) - original.components.begin());
        list->selectRow(tailRow);
        list->selectRow(primaryRow);
        remove->onClick();
        addType->setSelectedItemIndex(static_cast<int>(enginelab::ExhaustComponentType::resonator), juce::dontSendNotification);
        add->onClick();
        const auto newPart = enginelab::ExhaustNetworkEditor::nextComponentId([&] {
            auto shown = original;
            std::erase_if(shown.components, [primaryId](const auto& item) { return item.id == primaryId; });
            return shown;
        }());
        order.push_back(newPart);
        from->setSelectedItemIndex(rowOf(newPart), juce::dontSendNotification);
        to->setSelectedItemIndex(rowOf(downstreamId), juce::dontSendNotification);
        connect->onClick();
        auto cylinderItem = -1;
        for (int index = 0; index < from->getNumItems(); ++index)
            if (from->getItemText(index) == "Cylinder " + juce::String(cylinderId)) cylinderItem = index;
        require(cylinderItem >= 0, "From lists the path's cylinders");
        from->setSelectedItemIndex(cylinderItem, juce::dontSendNotification);
        to->setSelectedItemIndex(rowOf(newPart), juce::dontSendNotification);
        connect->onClick();
        applyDesign->onClick();
        require(applied.has_value() && !enginelab::validateEngineConfig(*applied).has_value(),
                "a cylinder feeding a resonator in its pipe's place applies");
        const auto& network = *applied->exhaustPaths.front().network;
        require(std::any_of(network.cylinderConnections.begin(), network.cylinderConnections.end(),
                    [cylinderId, newPart](const auto& item) { return item.cylinderId == cylinderId && item.componentId == newPart; })
                    && std::any_of(network.connections.begin(), network.connections.end(),
                        [newPart, downstreamId](const auto& edge) { return edge.fromComponentId == newPart && edge.toComponentId == downstreamId; }),
                "the cylinder feeds the resonator, and the resonator the old pipe's collector");
        require(std::any_of(network.components.begin(), network.components.end(),
                    [tailId](const auto& item) { return item.id == tailId && item.lengthMm == 763.27; }),
                "selecting a part, then another, kept its length to the hundredth");

        // What the cylinder feeds, by type.
        const auto feeds = [cylinderId](const enginelab::EngineConfig& design, enginelab::ExhaustComponentType type) {
            const auto& graph = *design.exhaustPaths.front().network;
            const auto root = std::find_if(graph.cylinderConnections.begin(), graph.cylinderConnections.end(),
                [cylinderId](const auto& item) { return item.cylinderId == cylinderId; });
            if (root == graph.cylinderConnections.end()) return false;
            return std::any_of(graph.components.begin(), graph.components.end(),
                [&root, type](const auto& item) { return item.id == root->componentId && item.type == type; });
        };
        // Ctrl+Z, Ctrl+Y, then every step undone gives the engine back.
        require(undo->isEnabled() && !redo->isEnabled(), "the edits can be undone");
        require(designerView->keyPressed(juce::KeyPress('z', juce::ModifierKeys::commandModifier, 0)) && redo->isEnabled(),
                "Ctrl+Z undoes an edit");
        require(designerView->keyPressed(juce::KeyPress('y', juce::ModifierKeys::commandModifier, 0)) && !redo->isEnabled(),
                "Ctrl+Y redoes it");
        for (auto guard = 0; undo->isEnabled() && guard < 50; ++guard) undo->onClick();
        applied.reset();
        applyDesign->onClick();
        require(applied.has_value() && applied->exhaustPaths.front().network
                    && applied->exhaustPaths.front().network->components.size() == original.components.size()
                    && feeds(*applied, enginelab::ExhaustComponentType::pipe),
                "undoing every edit gives the original exhaust back");
        for (auto guard = 0; redo->isEnabled() && guard < 50; ++guard) redo->onClick();
        applied.reset();
        applyDesign->onClick();
        require(applied.has_value() && feeds(*applied, enginelab::ExhaustComponentType::resonator),
                "redoing every edit gives the edited exhaust back");
        require(designerView->createComponentSnapshot(designerView->getLocalBounds()).isValid(), "the edited designer paints");
        const auto exhaustSnapshot = juce::SystemStats::getEnvironmentVariable("ENGINELAB_EXHAUST_SNAPSHOT", {});
        if (exhaustSnapshot.isNotEmpty()) {
            juce::FileOutputStream stream { juce::File(exhaustSnapshot) };
            stream.setPosition(0);
            stream.truncate();
            juce::PNGImageFormat().writeImageToStream(
                designerView->createComponentSnapshot(designerView->getLocalBounds()), stream);
        }
    }

    // The Equipment window: every part in one place. Nitrous hidden behind
    // "Fit a kit..." until fitted, then its settings, each taken at once,
    // and a status that says what an armed kit waits for; the active valve
    // fitted in one click; VTEC and launch control read from the tables.
    {
        const auto petrol = std::find_if(catalog.entries.begin(), catalog.entries.end(),
            [](const auto& entry) { return entry.config.name.find("2JZ") != std::string::npos; });
        require(petrol != catalog.entries.end(), "catalog has the 2JZ");
        namespace calibration = enginelab::calibration;
        auto store = std::make_shared<calibration::CalibrationStore>();
        require(store->publish(calibration::makeDefaultEcuCalibration(petrol->config), { 0, "engine-default" }).published,
                "the 2JZ's tables publish");
        std::vector<enginelab::NitrousConfig> kits;
        std::vector<enginelab::EngineConfig> engines;
        auto refills = 0;
        auto designer = 0;
        enginelab::EquipmentWindow::Callbacks callbacks;
        callbacks.applyNitrous = [&kits](const enginelab::NitrousConfig& kit) { kits.push_back(kit); return true; };
        callbacks.refillNitrous = [&refills] { ++refills; };
        callbacks.applyEngine = [&engines](const enginelab::EngineConfig& edited) {
            engines.push_back(edited);
            return true;
        };
        callbacks.openExhaustDesigner = [&designer] { ++designer; };
        enginelab::EquipmentWindow equipment(petrol->config, store, callbacks);
        auto& root = *equipment.content();
        const auto findAll = [](juce::Component& parent, const auto& predicate) {
            std::vector<juce::Component*> found;
            std::function<void(juce::Component&)> walk = [&](juce::Component& node) {
                for (auto* child : node.getChildren()) {
                    if (predicate(*child)) found.push_back(child);
                    walk(*child);
                }
            };
            walk(parent);
            return found;
        };
        const auto button = [&](const juce::String& text) -> juce::Button* {
            const auto found = findAll(root, [&text](juce::Component& child) {
                auto* asButton = dynamic_cast<juce::Button*>(&child);
                return asButton != nullptr && asButton->getButtonText() == text;
            });
            return found.empty() ? nullptr : dynamic_cast<juce::Button*>(found.front());
        };
        const auto visibleSliders = [&] {
            return findAll(root, [](juce::Component& child) {
                return dynamic_cast<juce::Slider*>(&child) != nullptr && child.isShowing();
            }).size();
        };
        const auto labelWith = [&](const juce::String& text) {
            return !findAll(root, [&text](juce::Component& child) {
                auto* label = dynamic_cast<juce::Label*>(&child);
                return label != nullptr && label->getText().contains(text);
            }).empty();
        };
        equipment.setVisible(true);
        equipment.setEngine(petrol->config, store);
        // Launch control (one field) only: no kit, no valve, no VTEC cams.
        const auto before = visibleSliders();
        auto* fit = button("Fit a kit...");
        require(fit != nullptr && fit->isShowing(), "an engine without a kit shows Fit a kit...");
        require(button("Fit a bypass valve") != nullptr && button("Fit VTEC cams") != nullptr,
                "the valve and VTEC cams can be fitted from it");
        fit->onClick();
        require(kits.size() == 1U && kits.back().fitted && !kits.back().pushButton,
                "Fit a kit... fits the default kit at once, sprayed without a push button");
        auto fitted = petrol->config;
        fitted.nitrous = kits.back();
        equipment.setEngine(fitted, store);
        require(visibleSliders() == before + 9U, "fitted, its nine settings show");
        auto* refill = button("Refill bottle");
        require(refill != nullptr, "a fitted kit has a refill button");
        refill->onClick();
        require(refills == 1, "Refill bottle fills the bottle");
        enginelab::EngineState state {};
        state.nitrousBlockers = static_cast<std::uint32_t>(enginelab::NitrousBlocker::throttle)
            | static_cast<std::uint32_t>(enginelab::NitrousBlocker::belowMinimumRpm);
        equipment.setState(state);
        require(labelWith("Armed, waiting for 95 % throttle, over 3000 rpm"), "an armed kit says what it waits for");
        state.nitrousBlockers = static_cast<std::uint32_t>(enginelab::NitrousBlocker::notArmed);
        equipment.setState(state);
        require(labelWith("arm it in Drive"), "a disarmed kit says where to arm it");
        state.nitrousBlockers = 0U;
        state.nitrousSpraying = true;
        state.nitrousMassFlowKgPerSecond = 0.05;
        equipment.setState(state);
        require(labelWith("Spraying 3.00 kg/min"), "the status shows the spray");
        button("Remove kit")->onClick();
        require(!kits.back().fitted, "Remove kit takes it off");
        // Anti-lag on one page: its switch, its mode, two sliders, the rest
        // folded under Advanced..., and what it is doing.
        {
            const auto antiLag = [&store] {
                return store->snapshot()->scalar(calibration::keys::antiLagEnabled).value_or(-1.0);
            };
            auto* antiLagSwitch = button("Anti-lag off");
            require(antiLagSwitch != nullptr && antiLagSwitch->isShowing() && antiLag() == 0.0,
                    "a turbocharged petrol engine shows anti-lag, off");
            antiLagSwitch->onClick();
            require(antiLag() == 1.0, "one click switches it on");
            equipment.setEngine(petrol->config, store);
            const auto folded = visibleSliders();
            button("Advanced...")->onClick();
            require(visibleSliders() == folded + 8U, "its eight advanced settings unfold");
            button("Hide advanced")->onClick();
            require(visibleSliders() == folded, "and fold again");
            enginelab::EngineState armedState {};
            armedState.antiLagArmed = true;
            armedState.antiLagBlockers = static_cast<std::uint32_t>(enginelab::AntiLagBlocker::throttle);
            equipment.setState(armedState);
            require(labelWith("works when you lift off"), "on, it says it works when you lift off");
            armedState.antiLagBlockers = static_cast<std::uint32_t>(enginelab::AntiLagBlocker::timeUp);
            equipment.setState(armedState);
            require(labelWith("press the throttle to re-arm"), "timed out, it says how to re-arm it");
            armedState.antiLagBlockers = 0U;
            armedState.antiLagActive = true;
            armedState.antiLagAuthority = 0.2;
            armedState.antiLagTargetTorqueNm = -24.0;
            equipment.setState(armedState);
            require(labelWith("Working: 40 % of its air, holding the engine to -24 N.m"), "working, it says how");
            button("Anti-lag on")->onClick();
            require(antiLag() == 0.0, "and one more click switches it off");
            equipment.setEngine(petrol->config, store);
        }
        // The active valve, in one click: a bypass around the last silencer.
        button("Fit a bypass valve")->onClick();
        require(engines.size() == 1U && enginelab::exhaustValveFitted(engines.back()),
                "Fit a bypass valve fits one");
        equipment.setEngine(engines.back(), store);
        require(button("Fit a bypass valve") == nullptr || !button("Fit a bypass valve")->isShowing(),
                "fitted, the valve shows its settings instead");
        button("Exhaust designer...")->onClick();
        require(designer == 1, "the exhaust designer is one click away");
        equipment.setVisible(false);
    }

    // The Fuel window (top bar): a listed fuel goes in the tank as listed,
    // an edited value renames it, and a diesel shows a cetane number.
    {
        const auto presets = enginelab::loadFuelPresets(ENGINELAB_CATALOG_ROOT);
        const auto petrol = std::find_if(catalog.entries.begin(), catalog.entries.end(),
            [](const auto& entry) { return entry.config.fuel == enginelab::FuelType::gasoline; });
        require(petrol != catalog.entries.end(), "catalog has a petrol engine");
        std::optional<enginelab::EngineConfig> applied;
        enginelab::FuelWindow fuelWindow(petrol->config, presets, [&applied](const enginelab::EngineConfig& edited) {
            applied = edited;
            return true;
        });
        auto* fuelContent = fuelWindow.getContentComponent();
        require(fuelContent != nullptr && !fuelContent->getBounds().isEmpty(), "fuel window lays out");
        juce::ComboBox* fuelBox = nullptr;
        juce::Button* applyButton = nullptr;
        std::vector<juce::Slider*> fuelFields;
        for (auto* child : fuelContent->getChildren()) {
            if (auto* box = dynamic_cast<juce::ComboBox*>(child)) fuelBox = box;
            if (auto* button = dynamic_cast<juce::Button*>(child); button != nullptr && button->getButtonText() == "Apply")
                applyButton = button;
            if (auto* slider = dynamic_cast<juce::Slider*>(child)) fuelFields.push_back(slider);
        }
        require(fuelBox != nullptr && applyButton != nullptr && fuelFields.size() == 5U,
                "fuel window shows a fuel list, five petrol fields and Apply");
        require(!applyButton->isEnabled(), "nothing to apply before an edit");
        const auto snapshotFile = juce::SystemStats::getEnvironmentVariable("ENGINELAB_FUEL_SNAPSHOT", {});
        if (snapshotFile.isNotEmpty()) {
            juce::FileOutputStream stream { juce::File(snapshotFile) };
            stream.setPosition(0);
            stream.truncate();
            juce::PNGImageFormat().writeImageToStream(
                fuelContent->createComponentSnapshot(fuelContent->getLocalBounds()), stream);
        }
        // Another listed petrol fuel than the engine's.
        auto other = presets.end();
        for (auto preset = presets.begin(); preset != presets.end(); ++preset)
            if (!preset->diesel() && preset->properties.name != petrol->config.fuelProperties.name
                && preset->octaneRating > 0.0 && preset->octaneRating != petrol->config.octaneRating) {
                other = preset;
                break;
            }
        require(other != presets.end(), "fuels.yaml lists another petrol fuel");
        auto otherId = 0;
        for (int index = 0; index < fuelBox->getNumItems(); ++index)
            if (fuelBox->getItemText(index) == juce::String::fromUTF8(other->properties.name.c_str()))
                otherId = fuelBox->getItemId(index);
        require(otherId != 0, "the fuel list offers it");
        fuelBox->setSelectedId(otherId, juce::sendNotificationSync);
        require(applyButton->isEnabled(), "a chosen fuel can be applied");
        require(near(fuelFields.front()->getValue(), other->octaneRating), "the fields show the chosen fuel");
        applyButton->onClick();
        require(applied.has_value() && near(applied->octaneRating, other->octaneRating)
                    && applied->fuelProperties.name == other->properties.name,
                "Apply puts the listed fuel in the tank, unrenamed");
        fuelWindow.setConfig(*applied);
        fuelFields.front()->setValue(other->octaneRating + 2.0, juce::sendNotificationSync);
        applyButton->onClick();
        require(near(applied->octaneRating, other->octaneRating + 2.0)
                    && applied->fuelProperties.name == other->properties.name,
                "a new octane is applied and keeps the fuel's name (the octane is the engine's)");
        fuelWindow.setConfig(*applied);
        fuelFields[1]->setValue(other->properties.lowerHeatingValueMjPerKg + 1.0, juce::sendNotificationSync);
        applyButton->onClick();
        require(near(applied->fuelProperties.lowerHeatingValueMjPerKg, other->properties.lowerHeatingValueMjPerKg + 1.0)
                    && applied->fuelProperties.name == other->properties.name + " (edited)",
                "an edited fuel value is applied and renames the fuel");

        // A new stoichiometric ratio takes the heating value with it, the
        // same energy per kilogram of oxygen, unless the heating value is set
        // too. Kept alone, a methanol ratio at a petrol heat ran a turbo to
        // ten bar (journal 2026-11-11).
        fuelWindow.setConfig(*applied);
        const auto heatBefore = applied->fuelProperties.lowerHeatingValueMjPerKg;
        const auto ratioBefore = applied->fuelProperties.stoichiometricAirFuelRatio;
        const auto energyBefore = enginelab::fuel_editing::energyPerOxygenMjPerKg(applied->fuelProperties);
        require(energyBefore > 12.0 && energyBefore < 14.0, "a listed petrol releases about 13 MJ per kg of oxygen");
        fuelFields[2]->setValue(9.81, juce::sendNotificationSync);
        require(std::abs(fuelFields[1]->getValue() - heatBefore * 9.81 / ratioBefore) <= 0.051,
                "the heating value field follows the ratio");
        applyButton->onClick();
        require(std::abs(applied->fuelProperties.lowerHeatingValueMjPerKg - heatBefore * 9.81 / ratioBefore) < 1.0e-9
                    && std::abs(enginelab::fuel_editing::energyPerOxygenMjPerKg(applied->fuelProperties)
                                - energyBefore) < 1.0e-9,
                "applied, the fuel keeps its energy per kilogram of oxygen");
        fuelWindow.setConfig(*applied);
        fuelFields[1]->setValue(43.0, juce::sendNotificationSync);
        fuelFields[2]->setValue(8.0, juce::sendNotificationSync);
        require(near(fuelFields[1]->getValue(), 43.0), "a heating value set by hand no longer follows");
        applyButton->onClick();
        require(near(applied->fuelProperties.lowerHeatingValueMjPerKg, 43.0)
                    && near(applied->fuelProperties.stoichiometricAirFuelRatio, 8.0),
                "and is applied as set, whatever the energy per kilogram of oxygen");

        // An alcohol in the tank refits the metering for the same mixture:
        // the injectors flow the stoichiometric ratio's share more mass, a
        // carburettor's jets also the root of the density's.
        const auto alcohol = std::find_if(presets.begin(), presets.end(), [](const auto& preset) {
            return !preset.diesel() && preset.properties.stoichiometricAirFuelRatio < 10.0; });
        require(alcohol != presets.end(), "fuels.yaml lists an alcohol");
        auto refitted = petrol->config;
        refitted.intake.carburettor.enabled = true;
        refitted.intake.carburettor.mainJetFlowNumber = 200.0;
        const auto petrolFuel = refitted.fuelProperties;
        const auto injectorBefore = refitted.injection.injectorFlowMgPerSecond;
        enginelab::fuel_editing::applyPreset(refitted, *alcohol);
        const auto massRatio = petrolFuel.stoichiometricAirFuelRatio
            / alcohol->properties.stoichiometricAirFuelRatio;
        require(near(refitted.injection.injectorFlowMgPerSecond, injectorBefore * massRatio),
                "an alcohol's injectors flow the stoichiometric ratio's share more fuel");
        require(near(refitted.intake.carburettor.mainJetFlowNumber, 200.0 * massRatio
                    * std::sqrt(petrolFuel.densityKgPerL / alcohol->properties.densityKgPerL)),
                "an alcohol's main jet passes the same mixture");
        auto unchanged = petrol->config;
        enginelab::fuel_editing::applyPreset(unchanged, *other);
        require(petrolFuel.stoichiometricAirFuelRatio != other->properties.stoichiometricAirFuelRatio
                    || unchanged.injection.injectorFlowMgPerSecond == petrol->config.injection.injectorFlowMgPerSecond,
                "a petrol of the same ratio keeps its injectors");

        const auto diesel = std::find_if(catalog.entries.begin(), catalog.entries.end(),
            [](const auto& entry) { return entry.config.fuel == enginelab::FuelType::diesel; });
        require(diesel != catalog.entries.end(), "catalog has a diesel");
        fuelWindow.setConfig(diesel->config);
        auto dieselFields = 0;
        for (auto* child : fuelContent->getChildren())
            if (dynamic_cast<juce::Slider*>(child) != nullptr) ++dieselFields;
        require(dieselFields == 4, "a diesel's fuel has four fields, a cetane number first");
    }

    {
        // The ECU's target AFR map is stored in petrol AFR (lambda x 14.7)
        // and reads, and is typed, in the fuel's own AFR.
        const auto spark = std::find_if(catalog.entries.begin(), catalog.entries.end(),
            [](const auto& entry) { return entry.config.fuel == enginelab::FuelType::gasoline; });
        require(spark != catalog.entries.end(), "catalog has a spark engine");
        auto store = std::make_shared<enginelab::calibration::CalibrationStore>();
        require(store->publish(enginelab::calibration::makeDefaultEcuCalibration(spark->config),
                               { 0, "engine-default" }).published, "the default calibration publishes");
        enginelab::EcuTunerComponent tuner(store);
        tuner.setSize(980, 620);
        constexpr double methanolStoichiometricAfr = 6.47;
        tuner.setFuel("Methanol M100", methanolStoichiometricAfr, false);
        juce::ComboBox* selector = nullptr;
        for (auto* child : tuner.getChildren())
            if (auto* box = dynamic_cast<juce::ComboBox*>(child)) selector = box;
        require(selector != nullptr, "the tuner lists its maps");
        auto afrItem = -1;
        for (int index = 0; index < selector->getNumItems(); ++index)
            if (selector->getItemText(index).contains(juce::String(std::string(
                    enginelab::calibration::keys::targetAirFuelRatio))))
                afrItem = index;
        require(afrItem >= 0, "the tuner lists the target AFR map");
        selector->setSelectedItemIndex(afrItem, juce::sendNotificationSync);
        std::vector<juce::TextEditor*> cells;
        for (auto* child : tuner.getChildren())
            if (auto* cell = dynamic_cast<juce::TextEditor*>(child)) cells.push_back(cell);
        const auto stored = [&store] {
            const auto published = store->snapshot();
            const auto* entry = published->find(enginelab::calibration::keys::targetAirFuelRatio);
            const auto* table = entry != nullptr
                ? std::get_if<enginelab::calibration::CalibrationTable2D>(entry) : nullptr;
            return table != nullptr && !table->values.empty() ? table->values.front() : -1.0;
        };
        const auto scale = methanolStoichiometricAfr / enginelab::calibration::ecuLimits::petrolStoichiometricAirFuelRatio;
        require(!cells.empty() && std::abs(cells.front()->getText().getDoubleValue() - stored() * scale) < 0.01,
                "a methanol engine's AFR map reads in methanol AFR");
        cells.front()->setText("5.80", false);
        cells.front()->onReturnKey();
        require(std::abs(stored() - 5.80 / scale) < 1.0e-9, "a methanol AFR typed is stored as petrol AFR");
        tuner.setFuel("Premium gasoline 98 RON", 14.7, false);
        require(std::abs(stored() * 1.0 - 5.80 / scale) < 1.0e-9 && std::abs(
                    [&tuner] {
                        for (auto* child : tuner.getChildren())
                            if (auto* cell = dynamic_cast<juce::TextEditor*>(child))
                                return cell->getText().getDoubleValue();
                        return -1.0;
                    }() - 5.80 / scale) < 0.01,
                "on petrol the same map reads in petrol AFR");
    }

    {
        // VTEC from the tuner: an engine without VTEC cams offers to fit
        // them, one with them switches its VTEC off and on; an engine that
        // cannot take them (a two-stroke) has no button.
        const auto ls3Entry = std::find_if(catalog.entries.begin(), catalog.entries.end(),
            [](const auto& entry) { return entry.config.name.find("LS3") != std::string::npos; });
        require(ls3Entry != catalog.entries.end(), "catalog has the LS3");
        auto ls3 = ls3Entry->config;
        namespace calibration = enginelab::calibration;
        auto store = std::make_shared<calibration::CalibrationStore>();
        require(store->publish(calibration::makeDefaultEcuCalibration(ls3), { 0, "engine-default" }).published,
                "the LS3's calibration publishes");
        {
            enginelab::EcuTunerComponent tuner(store);
            tuner.setSize(980, 620);
            tuner.setVtecFittable(false);
            require(!tuner.vtecButtonVisible(), "an engine that cannot take VTEC cams shows no VTEC button");
            tuner.setVtecFittable(true);
            require(tuner.vtecButtonVisible() && tuner.vtecButtonText() == "FIT VTEC CAMS",
                    "an engine without VTEC cams offers to fit them");
            auto asked = 0;
            tuner.onFitVtecCams = [&] {
                ++asked;
                require(enginelab::fitVtecCams(ls3), "the LS3 takes VTEC cams");
                auto draft = calibration::makeDraft(*store->snapshot());
                require(calibration::syncVtecSettings(draft, ls3) && store->publish(draft).published,
                        "the fitted cams' VTEC settings publish");
            };
            tuner.pressVtecButton();
            require(asked == 1, "FIT VTEC CAMS asks for the cams");
        }
        enginelab::EcuTunerComponent tuner(store);
        tuner.setSize(980, 620);
        tuner.setVtecFittable(true);
        const auto vtec = [&store] { return store->snapshot()->scalar(calibration::keys::vtecEnabled).value_or(-1.0); };
        require(tuner.vtecButtonVisible() && tuner.vtecButtonText() == "VTEC ON" && vtec() == 1.0,
                "with VTEC cams fitted the tuner shows VTEC on");
        tuner.pressVtecButton();
        require(tuner.vtecButtonText() == "VTEC OFF" && vtec() == 0.0, "VTEC OFF holds the low cam");
        tuner.pressVtecButton();
        require(tuner.vtecButtonText() == "VTEC ON" && vtec() == 1.0, "and VTEC ON gives the high cam back");
        tuner.setSessionLocked(true);
        tuner.pressVtecButton();
        require(vtec() == 1.0, "a locked tuner (a dyno run) leaves VTEC alone");
        require(!tuner.antiLagButtonVisible(), "an engine without a turbo shows no anti-lag switch");
    }
    {
        // Anti-lag's switch, on a turbocharged petrol engine's tables: off,
        // then armed and off again in one click each.
        const auto turbo = std::find_if(catalog.entries.begin(), catalog.entries.end(), [](const auto& entry) {
            return entry.config.name.find("2JZ") != std::string::npos;
        });
        require(turbo != catalog.entries.end(), "catalog has the 2JZ");
        namespace calibration = enginelab::calibration;
        auto store = std::make_shared<calibration::CalibrationStore>();
        require(store->publish(calibration::makeDefaultEcuCalibration(turbo->config), { 0, "engine-default" }).published,
                "the 2JZ's tables publish");
        enginelab::EcuTunerComponent tuner(store);
        tuner.setSize(980, 620);
        const auto antiLag = [&store] {
            return store->snapshot()->scalar(calibration::keys::antiLagEnabled).value_or(-1.0);
        };
        require(tuner.antiLagButtonVisible() && tuner.antiLagButtonText() == "ANTI-LAG OFF" && antiLag() == 0.0,
                "a turbocharged petrol engine's tuner shows anti-lag, off");
        tuner.pressAntiLagButton();
        require(tuner.antiLagButtonText() == "ANTI-LAG ON" && antiLag() == 1.0, "one click arms it");
        tuner.pressAntiLagButton();
        require(tuner.antiLagButtonText() == "ANTI-LAG OFF" && antiLag() == 0.0, "and one more turns it off");
        // Its single-value settings are one page, one row each, not one
        // selector item per value; so are the limits.
        juce::ComboBox* selector = nullptr;
        for (auto* child : tuner.getChildren())
            if (auto* box = dynamic_cast<juce::ComboBox*>(child)) selector = box;
        require(selector != nullptr, "the tuner lists its pages");
        auto antiLagPage = -1;
        auto limitsPage = -1;
        for (int index = 0; index < selector->getNumItems(); ++index) {
            const auto text = selector->getItemText(index);
            require(!text.contains("(antilag.min_rpm)") && !text.contains("(limits.overboost_kpa)"),
                    "a grouped setting has no selector item of its own");
            if (text.contains("(antilag.*)")) antiLagPage = index;
            if (text.contains("(limits.*)")) limitsPage = index;
        }
        require(antiLagPage >= 0 && limitsPage >= 0, "the selector has an Anti-lag page and a Limits page");
        const auto pages = enginelab::EcuTunerComponent::pagesOf(*store->snapshot());
        const auto group = std::find_if(pages.begin(), pages.end(), [](const auto& page) { return page.id == "antilag.*"; });
        require(group != pages.end() && group->members.size() == 12U
                    && group->members.front() == calibration::keys::antiLagEnabled,
                "the Anti-lag page holds its twelve settings, its switch first");
        selector->setSelectedItemIndex(antiLagPage, juce::sendNotificationSync);
        std::vector<juce::TextEditor*> rows;
        for (auto* child : tuner.getChildren())
            if (auto* cell = dynamic_cast<juce::TextEditor*>(child)) rows.push_back(cell);
        require(rows.size() == 12U, "the Anti-lag page shows twelve rows");
        const auto minimumRpm = std::find_if(rows.begin(), rows.end(), [](juce::TextEditor* row) {
            return row->getTooltip().startsWith("antilag.min_rpm:");
        });
        require(minimumRpm != rows.end() && (*minimumRpm)->getBounds().getY() > rows.front()->getBounds().getY(),
                "each row is its own setting, below the switch");
        (*minimumRpm)->setText("3200", false);
        (*minimumRpm)->onReturnKey();
        require(store->snapshot()->scalar(calibration::keys::antiLagMinimumRpm).value_or(-1.0) == 3200.0,
                "a value typed on the page is published at once");
    }

    {
        // What a window shows is inside it when it fits, so a screen
        // recorder's window capture films it whole: alert boxes, dialogs and
        // each tool window's tooltips. Menus and boxes that would not show
        // whole stay desktop windows.
        enginelab::ui::LookAndFeel lookAndFeel;
        juce::LookAndFeel::setDefaultLookAndFeel(&lookAndFeel);
        auto& theDesktop = juce::Desktop::getInstance();
        {
            juce::ComboBox menuBox;
            menuBox.addItem("one", 1);
            menuBox.addItem("two", 2);
            juce::DocumentWindow captureWindow("Capture", juce::Colours::black, juce::DocumentWindow::closeButton, true);
            auto* captureContent = new juce::Component();
            captureContent->setSize(900, 700);
            captureContent->addAndMakeVisible(menuBox);
            menuBox.setBounds(20, 20, 200, 28);
            captureWindow.setContentOwned(captureContent, true);
            require(captureWindow.isOnDesktop(), "the window is on the desktop");
            require(enginelab::ui::windowOf(&menuBox) == &captureWindow, "a component's window is its top-level window");
            // A menu inside its window was cut to that window (a small tool
            // window's list showed three entries) and drawn under the 3-D
            // view's OpenGL surface: it is a desktop window again.
            require(lookAndFeel.getParentComponentForMenuOptions(juce::PopupMenu::Options {}.withTargetComponent(&menuBox))
                        == nullptr,
                    "a menu (a combo box's list) opens as a desktop window, never clipped or hidden");
            const auto childOfWindow = [&captureWindow]<typename Child>(Child*) {
                for (auto* child : captureWindow.getChildren())
                    if (auto* found = dynamic_cast<Child*>(child)) return found;
                return static_cast<Child*>(nullptr);
            };
            const auto desktopCount = theDesktop.getNumComponents();
            enginelab::ui::showMessage(&menuBox, juce::MessageBoxIconType::WarningIcon, "Title", "Message");
            auto* alert = childOfWindow(static_cast<juce::AlertWindow*>(nullptr));
            require(theDesktop.getNumComponents() == desktopCount && alert != nullptr && !alert->isOnDesktop()
                        && captureWindow.getLocalBounds().contains(alert->getBounds()),
                    "an alert box is drawn inside the window, not as a desktop window");
            delete alert;
            enginelab::ui::showOkCancel(&menuBox, juce::MessageBoxIconType::QuestionIcon, "Title", "Message", "Yes",
                                        "No", [](bool) {});
            alert = childOfWindow(static_cast<juce::AlertWindow*>(nullptr));
            require(theDesktop.getNumComponents() == desktopCount && alert != nullptr && !alert->isOnDesktop(),
                    "an OK/Cancel box too");
            delete alert;
            juce::AlertWindow dialog("Save engine", "Name", juce::MessageBoxIconType::NoIcon);
            dialog.addTextEditor("name", "My engine");
            require(dialog.isOnDesktop(), "a dialog built by hand starts as a desktop window");
            const auto withDialog = theDesktop.getNumComponents();
            enginelab::ui::placeInWindow(dialog, menuBox);
            require(dialog.getParentComponent() == &captureWindow && !dialog.isOnDesktop()
                        && theDesktop.getNumComponents() == withDialog - 1
                        && captureWindow.getLocalBounds().contains(dialog.getBounds()),
                    "and is moved inside the window, centred");
            captureWindow.removeChildComponent(&dialog);
            dialog.addToDesktop(juce::ComponentPeer::windowHasDropShadow);

            // A native view (the 3-D view's OpenGL surface) hides whatever the
            // window draws over it: a box there stays a desktop window.
            juce::Component glView;
            captureContent->addAndMakeVisible(glView);
            glView.setBounds(captureContent->getLocalBounds().reduced(100));
            enginelab::ui::markNativeView(glView, true);
            require(!enginelab::ui::placeInWindow(dialog, menuBox) && dialog.isOnDesktop()
                        && dialog.getParentComponent() == nullptr,
                    "a dialog over the 3-D view stays a desktop window");
            const auto withNativeView = theDesktop.getNumComponents();
            enginelab::ui::showMessage(&menuBox, juce::MessageBoxIconType::WarningIcon, "Over3d", "Message");
            juce::AlertWindow* overView = nullptr;
            for (int index = 0; index < theDesktop.getNumComponents(); ++index)
                if (auto* found = dynamic_cast<juce::AlertWindow*>(theDesktop.getComponent(index));
                    found != nullptr && found->getName() == "Over3d")
                    overView = found;
            require(theDesktop.getNumComponents() == withNativeView + 1 && overView != nullptr
                        && childOfWindow(static_cast<juce::AlertWindow*>(nullptr)) == nullptr,
                    "so does an alert box");
            delete overView;
            enginelab::ui::markNativeView(glView, false);
            require(enginelab::ui::placeInWindow(dialog, menuBox), "without the mark it goes inside again");
            captureWindow.removeChildComponent(&dialog);
            dialog.addToDesktop(juce::ComponentPeer::windowHasDropShadow);

            // A window smaller than the box would shrink it: it stays whole,
            // a desktop window.
            captureWindow.setContentComponentSize(160, 90);
            const auto size = dialog.getBounds().getWidth();
            require(!enginelab::ui::placeInWindow(dialog, menuBox) && dialog.isOnDesktop()
                        && dialog.getWidth() == size,
                    "a dialog larger than its window stays a desktop window, unshrunk");
        }
        {
            // A component in no window on screen keeps a desktop box: one
            // centred on a component of no size would be modal and unseen.
            juce::Component loose;
            require(enginelab::ui::windowOf(&loose) == nullptr, "a component in no window has none");
            const auto desktopCount = theDesktop.getNumComponents();
            enginelab::ui::showMessage(&loose, juce::MessageBoxIconType::WarningIcon, "Loose", "Message");
            juce::AlertWindow* looseBox = nullptr;
            for (int index = 0; index < theDesktop.getNumComponents(); ++index)
                if (auto* found = dynamic_cast<juce::AlertWindow*>(theDesktop.getComponent(index));
                    found != nullptr && found->getName() == "Loose")
                    looseBox = found;
            require(theDesktop.getNumComponents() == desktopCount + 1 && looseBox != nullptr, "its alert box is a desktop window");
            delete looseBox;
        }
        {
            const auto hasTooltips = [](juce::Component& toolWindow) {
                return std::any_of(toolWindow.getChildren().begin(), toolWindow.getChildren().end(),
                                   [](juce::Component* child) { return dynamic_cast<juce::TooltipWindow*>(child) != nullptr; });
            };
            auto toolStore = std::make_shared<enginelab::calibration::CalibrationStore>();
            require(toolStore->publish(enginelab::calibration::makeDefaultEcuCalibration(engine)).published,
                    "the engine's default tables publish");
            enginelab::EcuTunerWindow toolTuner(toolStore);
            enginelab::FuelWindow toolFuel(engine, enginelab::loadFuelPresets(ENGINELAB_CATALOG_ROOT), {});
            enginelab::ExhaustDesignerWindow toolExhaust(engine, {});
            require(hasTooltips(toolTuner) && hasTooltips(toolFuel) && hasTooltips(toolExhaust),
                    "each tool window shows its own tooltips");
        }
        juce::LookAndFeel::setDefaultLookAndFeel(nullptr);
    }

    // The pass-by window: a run's settings, Start/Restart/Stop and Record
    // WAV handed to the app, the microphone's place and ground, and what the
    // run is doing in words.
    {
        std::vector<std::pair<enginelab::PassByRunSettings, bool>> starts;
        auto stops = 0;
        std::vector<enginelab::PassByMicrophoneChoice> microphones;
        enginelab::PassByWindow::Callbacks callbacks;
        callbacks.start = [&starts](const enginelab::PassByRunSettings& settings, bool record) {
            starts.emplace_back(settings, record);
        };
        callbacks.stop = [&stops] { ++stops; };
        callbacks.setMicrophone = [&microphones](const enginelab::PassByMicrophoneChoice& choice) {
            microphones.push_back(choice);
        };
        enginelab::PassByWindow passBy(engine, callbacks);
        passBy.setVisible(true);
        auto& root = *passBy.content();
        std::vector<juce::Component*> all;
        std::function<void(juce::Component&)> walk = [&](juce::Component& node) {
            for (auto* child : node.getChildren()) {
                all.push_back(child);
                walk(*child);
            }
        };
        walk(root);
        const auto button = [&](const juce::String& text) -> juce::Button* {
            for (auto* child : all)
                if (auto* asButton = dynamic_cast<juce::Button*>(child); asButton != nullptr && asButton->getButtonText() == text)
                    return asButton;
            return nullptr;
        };
        std::vector<enginelab::ui::SegmentedControl*> segments;
        std::vector<juce::Slider*> sliders;
        for (auto* child : all) {
            if (auto* segment = dynamic_cast<enginelab::ui::SegmentedControl*>(child)) segments.push_back(segment);
            if (auto* slider = dynamic_cast<juce::Slider*>(child)) sliders.push_back(slider);
        }
        require(segments.size() == 3U && sliders.size() == 7U,
                "the window has its program, ground and level choices and its seven figures");
        require(!microphones.empty() && microphones.back().ground == enginelab::PassByGround::asphalt
                    && near(microphones.back().laneDistanceM, 7.5) && !microphones.back().normalisedLevel,
                "it hands the app ISO 362's microphone over asphalt at the real level");
        auto* start = button("Start");
        require(start != nullptr && button("Record WAV") != nullptr && button("Stop") != nullptr,
                "it has Start, Stop and Record WAV");
        start->onClick();
        require(starts.size() == 1U && !starts.back().second
                    && starts.back().first.program == enginelab::PassByProgram::standingStart
                    && near(starts.back().first.startDistanceM, 500.0) && near(starts.back().first.endDistanceM, 500.0),
                "Start asks for a standing start 500 m before the microphone to 500 m past it, unrecorded");
        segments[0]->setSelected(1, juce::sendNotification);
        sliders[0]->setValue(300.0, juce::sendNotificationSync);
        button("Record WAV")->onClick();
        start->onClick();
        require(starts.size() == 2U && starts.back().second
                    && starts.back().first.program == enginelab::PassByProgram::acceleration
                    && near(starts.back().first.startDistanceM, 300.0),
                "the program, the distances and Record WAV reach the run");
        segments[1]->setSelected(3, juce::sendNotification);
        segments[2]->setSelected(1, juce::sendNotification);
        require(microphones.back().ground == enginelab::PassByGround::grass && microphones.back().normalisedLevel,
                "the ground and the level reach the microphone at once");
        enginelab::EngineState state {};
        state.passByPhase = static_cast<std::uint8_t>(enginelab::PassByPhase::measuring);
        state.passByPositionM = -120.0;
        passBy.setState(state);
        require(button("Restart") != nullptr && button("Restart")->isEnabled() && button("Stop")->isEnabled(),
                "while a run drives, Start reads Restart and Stop is on");
        require(enginelab::describePassBy(state, passBy.runSettings()).contains("Driving past"),
                "and it says the car is driving past");
        button("Stop")->onClick();
        require(stops == 1, "Stop lets the driver go");
        state.passByPhase = static_cast<std::uint8_t>(enginelab::PassByPhase::finished);
        state.passBySeconds = 17.8;
        require(enginelab::describePassBy(state, passBy.runSettings()).contains("17.8 s"),
                "finished, it gives the run's time");
        passBy.setLocked(true);
        require(button("Restart") != nullptr && !button("Restart")->isEnabled(), "the dyno locks it");
    }

    {
        // At its smallest accepted size the Pass-by window still lays out every
        // row: the microphone level had no height at 560 px (audit APP-11).
        // Its choices take Tab focus and answer the arrows (audit UA-07).
        enginelab::PassByWindow passBy(engine, {});
        const auto* limits = passBy.getConstrainer();
        require(limits != nullptr, "the Pass-by window has resize limits");
        passBy.setSize(limits->getMinimumWidth(), limits->getMinimumHeight());
        auto& root = *passBy.content();
        root.setSize(limits->getMinimumWidth(), limits->getMinimumHeight());
        std::vector<enginelab::ui::SegmentedControl*> choices;
        for (auto* child : root.getChildren())
            if (auto* choice = dynamic_cast<enginelab::ui::SegmentedControl*>(child)) choices.push_back(choice);
        require(choices.size() == 3, "program, ground and level are segmented choices");
        for (auto* choice : choices) {
            require(choice->getHeight() >= 20 && choice->getBottom() <= root.getHeight(),
                    "at the minimum size every choice keeps its row (" + std::to_string(choice->getBottom())
                        + " of " + std::to_string(root.getHeight()) + ")");
            require(choice->getWantsKeyboardFocus(), "a Pass-by choice takes Tab focus");
        }
        auto* level = choices.back();
        level->setSelected(0);
        require(level->keyPressed(juce::KeyPress(juce::KeyPress::rightKey)) && level->selected() == 1,
                "the right arrow picks the next option");
        require(level->keyPressed(juce::KeyPress(juce::KeyPress::homeKey)) && level->selected() == 0,
                "Home picks the first option");
        // A disabled button (Stop before a run) is skipped by Tab: JUCE reads
        // no focus wish from a disabled component.
        auto focusable = 0;
        for (auto* child : root.getChildren())
            if (auto* action = dynamic_cast<enginelab::ui::ActionButton*>(child); action != nullptr
                && action->isEnabled()) {
                require(action->getWantsKeyboardFocus(), "a Pass-by button takes Tab focus");
                ++focusable;
            }
        require(focusable >= 2, "Start and Record take Tab focus");
        enginelab::ui::SegmentedControl mainWindowChoice { juce::StringArray { "a", "b" } };
        require(!mainWindowChoice.getWantsKeyboardFocus() && !mainWindowChoice.keyPressed(
                    juce::KeyPress(juce::KeyPress::rightKey)),
                "in the main window a choice leaves the arrows to the gearbox");
    }

    {
        // The anti-lag settings page at the tuner's smallest window: every value
        // inside the page, none under the tool row (audit UA-02).
        const auto twoJz = std::find_if(catalog.entries.begin(), catalog.entries.end(),
            [](const auto& entry) { return entry.config.name.find("2JZ") != std::string::npos; });
        require(twoJz != catalog.entries.end(), "catalog has the 2JZ");
        auto store = std::make_shared<enginelab::calibration::CalibrationStore>();
        require(store->publish(enginelab::calibration::makeDefaultEcuCalibration(twoJz->config),
                               { 0, "engine-default" }).published, "the 2JZ calibration publishes");
        enginelab::EcuTunerWindow tunerWindow(store);
        const auto* limits = tunerWindow.getConstrainer();
        require(limits != nullptr && limits->getMinimumHeight() >= 520, "the tuner's minimum height fits its maps");
        enginelab::EcuTunerComponent tuner(store);
        tuner.setSize(limits->getMinimumWidth(), limits->getMinimumHeight() - 30);
        juce::ComboBox* selector = nullptr;
        for (auto* child : tuner.getChildren())
            if (auto* box = dynamic_cast<juce::ComboBox*>(child)) selector = box;
        require(selector != nullptr, "the tuner lists its pages");
        auto pageItem = -1;
        for (int index = 0; index < selector->getNumItems(); ++index)
            if (selector->getItemText(index).containsIgnoreCase("anti-lag settings")) pageItem = index;
        require(pageItem >= 0, "the tuner lists the anti-lag settings page");
        juce::Rectangle<int> tools;
        const auto checkInside = [&](const char* what) {
            auto editors = 0;
            for (auto* child : tuner.getChildren()) {
                auto* button = dynamic_cast<juce::TextButton*>(child);
                if (button != nullptr && button->getButtonText() == "UNDO") tools = button->getBounds();
            }
            for (auto* child : tuner.getChildren()) {
                auto* editor = dynamic_cast<juce::TextEditor*>(child);
                if (editor == nullptr || !editor->isVisible()) continue;
                ++editors;
                require(editor->getBottom() <= tools.getY() && editor->getRight() <= tuner.getWidth(),
                        std::string(what) + ": a value stays above the tool row ("
                            + std::to_string(editor->getBottom()) + " / " + std::to_string(tools.getY()) + ")");
            }
            return editors;
        };
        selector->setSelectedItemIndex(pageItem, juce::sendNotificationSync);
        require(checkInside("anti-lag page") >= 10, "the anti-lag page shows its settings");
        for (int index = 0; index < selector->getNumItems(); ++index) {
            if (selector->getItemText(index).contains("settings")) continue;
            selector->setSelectedItemIndex(index, juce::sendNotificationSync);
            (void)checkInside(selector->getItemText(index).toRawUTF8());
        }
    }

    std::cout
        << "Audio workshop: layout, truthful availability, physical A/B, "
           "live telemetry, mute/solo routing, HQ export controls and in-window boxes PASS\n";
    return 0;
}
