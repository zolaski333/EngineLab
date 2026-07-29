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
    enginelab::AudioWorkshopWindow window(
        engine, ENGINELAB_CATALOG_ROOT, base,
        true, true, false,
        [&](const enginelab::OfflineAudioMix& nextBase,
            const enginelab::OfflineAudioMix& nextEffective) {
            ++callbackCount;
            callbackBase = nextBase;
            callbackEffective = nextEffective;
        });
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
    verifyLayoutAt(980, 620);
    verifyLayoutAt(1'120, 700);
    verifyLayoutAt(1'600, 900);

    int visibleChildren = 0;
    int sliderCount = 0;
    int disabledSliderCount = 0;
    int muteCount = 0;
    int soloCount = 0;
    int disabledMuteCount = 0;
    int disabledSoloCount = 0;
    bool exportButtonFound = false;
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
        if (const auto* button =
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
            }
        }
    }
    require(visibleChildren >= 30, "complete workshop control set is present");
    require(
        sliderCount == 9 && disabledSliderCount == 4,
        "nine faders exist and four truthful no-op controls are disabled");
    require(
        muteCount == 4 && soloCount == 4
            && disabledMuteCount == 1
            && disabledSoloCount == 1,
        "four source mute/solo rows exist and direct combustion is unavailable");
    require(exportButtonFound, "HQ export action is visible");

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
        << "Audio workshop: layout, truthful availability, "
           "mute/solo routing and HQ export controls PASS\n";
    return 0;
}
