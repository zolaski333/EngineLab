#pragma once

#include <enginelab/audio/OfflineAudioExporter.hpp>

#include <juce_gui_extra/juce_gui_extra.h>

#include <filesystem>
#include <functional>
#include <array>

namespace enginelab {

struct AudioWorkshopAvailability final {
    bool compiledExhaustTopology { false };
    bool compiledIntakeTopology { false };
    bool measuredImpulseResponseAvailable { false };
};

struct AudioWorkshopLayerSwitches final {
    /** combustion, exhaust, intake+FI, structure/mechanical */
    std::array<bool, 4> muted {};
    std::array<bool, 4> solo {};
};

/** Apply availability plus source mute/solo without modifying visible faders. */
[[nodiscard]] OfflineAudioMix effectiveAudioWorkshopMix(
    const OfflineAudioMix& baseMix,
    const AudioWorkshopAvailability& availability,
    const AudioWorkshopLayerSwitches& switches) noexcept;

/** Standalone source mixer and deterministic HQ-export front end.
 *
 * `baseMix` retains the visible fader values. `effectiveMix` additionally
 * applies mute/solo and physical-path availability, so MainComponent can keep
 * user intent separate from the values currently published to the audio thread.
 */
class AudioWorkshopWindow final : public juce::DocumentWindow {
public:
    using MixChangedCallback = std::function<void(
        const OfflineAudioMix& baseMix,
        const OfflineAudioMix& effectiveMix)>;

    AudioWorkshopWindow(
        const EngineConfig& engine,
        std::filesystem::path assetRoot,
        const OfflineAudioMix& mix,
        bool compiledExhaustTopology,
        bool compiledIntakeTopology,
        bool measuredImpulseResponseAvailable,
        MixChangedCallback mixChanged);
    ~AudioWorkshopWindow() override;

    void closeButtonPressed() override;
    void setEngine(
        const EngineConfig& engine,
        std::filesystem::path assetRoot,
        bool compiledExhaustTopology,
        bool compiledIntakeTopology,
        bool measuredImpulseResponseAvailable);
    /** Synchronise external fader changes without discarding mute/solo state. */
    void setMix(const OfflineAudioMix& mix);

private:
    class WorkshopContent;
    WorkshopContent* content_ {};

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(AudioWorkshopWindow)
};

} // namespace enginelab
