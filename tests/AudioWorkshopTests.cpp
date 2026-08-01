#include <enginelab/app/AudioWorkshopWindow.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>

#include <juce_gui_extra/juce_gui_extra.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
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
        labSettings.afterfireEnabled && labSettings.limiterKeepsFuel
            && near(labSettings.cycleVariationCoefficientOfVariation, 0.06),
        "audio lab engine opts into the complete physical demo path");
    require(
        enginelab::audioPhysicsTelemetryFor(
            audioLab->config, enginelab::EngineState {}).porousMufflerCount == 1,
        "audio lab engine contains one active absorptive silencer");

    auto physicsConfig = engine;
    enginelab::AudioPhysicsSettings authoredPhysics {
        0.06, 0.55, true, true, 800.0, 0.008, 0.95
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
            && near(recoveredPhysics.afterfireEfficiency, 0.95),
        "audio physics controls round-trip through EngineConfig");
    enginelab::EngineState physicsState;
    physicsState.cylinderStates[0].combustionCycleMultiplier = 0.91;
    physicsState.cylinderStates[1].combustionCycleMultiplier = 1.08;
    physicsState.exhaustAfterfireHeatReleaseKw = 3.4;
    physicsState.exhaustAfterfireFuelBurnMgPerSecond = 22.0;
    const auto physicsTelemetry =
        enginelab::audioPhysicsTelemetryFor(physicsConfig, physicsState);
    require(
        near(physicsTelemetry.minimumCycleMultiplier, 0.91)
            && near(physicsTelemetry.maximumCycleMultiplier, 1.08)
            && near(physicsTelemetry.afterfireHeatReleaseKw, 3.4)
            && near(physicsTelemetry.afterfireFuelBurnMgPerSecond, 22.0),
        "audio physics telemetry reports real simulator state");

    enginelab::OfflineAudioMix base;
    base.volume = 1.25;
    base.convolution = 0.60;
    base.highFrequencyGain = 1.10;
    base.lowFrequencyNoise = 0.45;
    base.highFrequencyNoise = 0.55;
    base.combustionGain = 1.20;
    base.exhaustGain = 1.30;
    base.intakeGain = 0.90;
    base.mechanicalGain = 0.80;

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
    juce::TextButton* demoPhysicsButton = nullptr;
    bool applyPhysicsButtonFound = false;
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
                       == "CHOISIR DOSSIER ET EXPORTER") {
                exportButtonFound = true;
            } else if (text == "DEMO AUDIBLE") {
                demoPhysicsButton = button;
            } else if (text == "APPLIQUER ET REDEMARRER") {
                applyPhysicsButtonFound = true;
            }
        }
    }
    require(visibleChildren >= 45, "complete workshop control set is present");
    require(
        sliderCount == 14 && disabledSliderCount == 4,
        "nine faders and five physics controls exist; four no-op faders are disabled");
    require(
        muteCount == 4 && soloCount == 4
            && disabledMuteCount == 1
            && disabledSoloCount == 1,
        "four source mute/solo rows exist and direct combustion is unavailable");
    require(exportButtonFound, "HQ export action is visible");
    require(demoPhysicsButton != nullptr && applyPhysicsButtonFound,
            "audible physics A/B and explicit apply actions are visible");
    demoPhysicsButton->onClick();
    require(
        physicsApplyCount == 1
            && near(appliedPhysics.cycleVariationCoefficientOfVariation, 0.06)
            && appliedPhysics.afterfireEnabled
            && appliedPhysics.limiterKeepsFuel,
        "demo action publishes an intentionally audible physical calibration");

    window.setMix(base);
    require(callbackCount > 0, "mix synchronisation publishes to MainComponent");
    require(
        near(callbackBase.volume, base.volume)
            && near(callbackBase.exhaustGain, base.exhaustGain),
        "visible fader intent is preserved");
    require(
        near(callbackEffective.convolution, 0.0)
            && near(callbackEffective.combustionGain, 0.0)
            && near(callbackEffective.exhaustGain, base.exhaustGain),
        "window publishes topology-aware effective mix");

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

    std::cout
        << "Audio workshop: layout, truthful availability, physical A/B, "
           "live telemetry, mute/solo routing and HQ export controls PASS\n";
    return 0;
}
