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

/** Authored controls that make the pressure/exhaust audio physics audible. */
struct AudioPhysicsSettings final {
    double cycleVariationCoefficientOfVariation { 0.0 };
    double cycleVariationCorrelation { 0.45 };
    bool afterfireEnabled { false };
    bool limiterKeepsFuel { false };
    double afterfireIgnitionTemperatureK { 900.0 };
    double afterfireReactionTimeSeconds { 0.010 };
    double afterfireEfficiency { 0.95 };
    double overrunFuelFraction { 0.0 };
    /** 0 retains fuel on every cycle (anti-lag roar); a rate chops it into
     *  discrete slugs, which is what a pop-and-bang map does. Append new
     *  members below this comment: the aggregate is initialised by position. */
    double overrunPulseHz { 0.0 };
    double overrunPulseDutyCycle { 0.35 };
    double overrunPulseTimingVariation { 0.0 };
};

/** Live proof that the authored controls are active in the simulator. */
struct AudioPhysicsTelemetry final {
    double minimumCycleMultiplier { 1.0 };
    double maximumCycleMultiplier { 1.0 };
    double afterfireHeatReleaseKw { 0.0 };
    double afterfireFuelBurnMgPerSecond { 0.0 };
    bool overrunAfterfireActive { false };
    std::size_t porousMufflerCount { 0 };
};

[[nodiscard]] AudioPhysicsSettings audioPhysicsSettingsFor(
    const EngineConfig& engine) noexcept;
void applyAudioPhysicsSettings(
    EngineConfig& engine, const AudioPhysicsSettings& settings) noexcept;
[[nodiscard]] AudioPhysicsTelemetry audioPhysicsTelemetryFor(
    const EngineConfig& engine, const EngineState& state) noexcept;

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
    using PhysicsApplyCallback = std::function<bool(
        const AudioPhysicsSettings& settings)>;
    using TelemetryProvider = std::function<AudioPhysicsTelemetry()>;

    AudioWorkshopWindow(
        const EngineConfig& engine,
        std::filesystem::path assetRoot,
        const OfflineAudioMix& mix,
        bool compiledExhaustTopology,
        bool compiledIntakeTopology,
        bool measuredImpulseResponseAvailable,
        MixChangedCallback mixChanged,
        PhysicsApplyCallback physicsApply = {},
        TelemetryProvider telemetryProvider = {});
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
