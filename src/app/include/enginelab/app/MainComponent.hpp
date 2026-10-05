#pragma once
#include <enginelab/audio/RealtimeEngineAudio.hpp>
#include <enginelab/app/ActionMap.hpp>
#include <enginelab/app/AudioWorkshopWindow.hpp>
#include <enginelab/app/ControlPanel.hpp>
#include <enginelab/app/DashboardModel.hpp>
#include <enginelab/app/EcuTunerWindow.hpp>
#include <enginelab/app/EngineViewport.hpp>
#include <enginelab/app/ExhaustDesignerWindow.hpp>
#include <enginelab/app/ReadoutStrip.hpp>
#include <enginelab/app/SidePanel.hpp>
#include <enginelab/app/StatusBar.hpp>
#include <enginelab/app/TopBar.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/diagnostics/EngineDiagnostics.hpp>
#include <enginelab/foundation/EngineTypes.hpp>
#include <enginelab/render/RenderSnapshot.hpp>
#include <enginelab/runtime/EngineRuntime.hpp>
#include <enginelab/scripting/EngineScriptHotReloader.hpp>
#include <enginelab/serialization/JsonEngineSerializer.hpp>
#include <enginelab/serialization/YamlEngineSerializer.hpp>
#include <juce_audio_utils/juce_audio_utils.h>
#include <juce_gui_extra/juce_gui_extra.h>
#include <memory>
#include <array>
#include <filesystem>
#include <limits>

namespace enginelab {
/** Desktop presentation layer. It owns the runtime and the audio device, writes
    controls and refreshes the panels from one snapshot per UI frame. The panels
    (top bar, controls, engine view, readouts, side tabs, status bar) only read
    the shared DashboardModel. */
class MainComponent final : public juce::AudioAppComponent, private juce::Timer {
public:
    MainComponent();
    ~MainComponent() override;
    void prepareToPlay(int samplesPerBlockExpected, double sampleRate) override;
    void getNextAudioBlock(const juce::AudioSourceChannelInfo&) override;
    void releaseResources() override;
    void paint(juce::Graphics&) override;
    void resized() override;
    bool keyPressed(const juce::KeyPress&) override;
    bool keyStateChanged(bool isKeyDown) override;
    void focusLost(FocusChangeType cause) override;
    void mouseWheelMove(const juce::MouseEvent&, const juce::MouseWheelDetails&) override;
private:
    void timerCallback() override;
    void selectEngine(int presetIndex);
    bool applyConfig(const EngineConfig&, bool preserveScriptWatcher = false,
                     bool preserveCalibration = false);
    /** An edit of config.exhaust and config.exhaustPaths only: taken by the
        running engine when it keeps the network's topology, else a restart. */
    bool applyExhaustEdit(const EngineConfig&);
    void showConfigEditor();
    void showConfigEditor(const juce::String& initialText);
    void showKeyBindingsEditor();
    void showAudioWorkshop();
    void showEcuTuner();
    void showExhaustDesigner();
    void showMoreMenu();
    void startEngineScriptWatcher(const std::filesystem::path&);
    void stopEngineScriptWatcher() noexcept;
    void pollEngineScript();
    void pollAudioVoicing();
    void adoptAudioVoicing(const AudioVoicingConfig&);
    void importEngine();
    void exportEngine();
    void exportDynoCsv();
    void showError(const juce::String& title, const juce::String& message);
    void reloadEngine();
    void collectFinishedRuns();
    void selectRun(int runIndex);
    void ensureDynoPresentation(const DynoRun&, std::size_t paletteIndex);
    [[nodiscard]] DynoCurvePresentation* selectedPresentation();
    void setThrottlePreset(double value);
    void updateMomentaryThrottle();
    void toggleDyno();
    void applyExhaustPreset(int presetIndex);
    void updateAudioControlAvailability();
    void configureImpulseResponse();
    void refreshKeyCaps();
    void refreshAudioView();
    void refreshRuntimeHealth();
    [[nodiscard]] bool wheelModifierDown() const noexcept;
    [[nodiscard]] OfflineAudioMix currentAudioMix() const noexcept;
    void applyAudioWorkshopMix(
        const OfflineAudioMix& baseMix,
        const OfflineAudioMix& effectiveMix);
    void syncAudioWorkshopMix();
    void adjustAudioOrSimulation(double wheelDelta);

    std::vector<EngineConfig> presets_ { makeBaseEnginePresets() };
    std::filesystem::path catalogRoot_;
    EngineConfig config_ { presets_[1] };
    int selectedPresetIndex_ { -1 };
    std::unique_ptr<EngineRuntime> runtime_;
    std::unique_ptr<RealtimeEngineAudio> audio_;
    EngineDiagnostics diagnostics_;
    ActionMap actionMap_;
    JsonEngineSerializer jsonSerializer_;
    YamlEngineSerializer yamlSerializer_;
    DashboardModel model_ { config_ };
    std::unique_ptr<RenderSnapshotBuilder> renderSnapshotBuilder_;
    RenderSnapshotInterpolator renderSnapshotInterpolator_;
    std::shared_ptr<DynoRunArchive> dynoArchive_ {
        std::make_shared<DynoRunArchive>() };
    std::uint64_t visibleDynoArchiveRevision_ {
        std::numeric_limits<std::uint64_t>::max() };
    bool starterKeyDown_ { false };
    bool brakeKeyDown_ { false };
    bool throttleKeyActive_ { false };
    double targetClutchPressure_ { 1.0 };
    double currentClutchPressure_ { 1.0 };
    bool showDynoStats_ { true };
    double audioVolume_ { 1.0 };
    double audioConvolution_ { 0.45 };
    double highFrequencyGain_ { 1.0 };
    double lowFrequencyGain_ { 1.0 };
    double lowFrequencyNoise_ { 0.35 };
    double highFrequencyNoise_ { 0.35 };
    double combustionGain_ { 1.0 };
    double exhaustGain_ { 1.0 };
    double intakeGain_ { 0.85 };
    double mechanicalGain_ { 0.70 };
    double stereoWidth_ { 1.0 };
    double outletJetGain_ { 1.0 };
    double saturationDrive_ { 0.0 };
    AudioSaturationPlacement saturationPlacement_ {
        AudioSaturationPlacement::postShelf
    };
    AudioMonitorMode audioMonitorMode_ {
        AudioMonitorMode::physicalReference
    };
    int exhaustPresetIndex_ { 0 };
    bool physicalExhaustTopology_ { false };
    bool physicalIntakeTopology_ { false };
    bool structuralRadiationActive_ { false };
    bool forcedInductionAcousticsActive_ { false };
    bool impulseResponseAvailable_ { false };
    bool impulseResponseLoadError_ { false };
    juce::String impulseResponseStatus_ { "IR free field" };
    std::unique_ptr<juce::FileChooser> fileChooser_;
    std::unique_ptr<juce::AlertWindow> configEditor_;
    std::unique_ptr<juce::AlertWindow> keyBindingsEditor_;
    std::unique_ptr<EcuTunerWindow> ecuTunerWindow_;
    std::unique_ptr<ExhaustDesignerWindow> exhaustDesignerWindow_;
    std::unique_ptr<AudioWorkshopWindow> audioWorkshopWindow_;
    std::unique_ptr<scripting::EngineScriptHotReloader> scriptReloader_;
    std::uint64_t scriptRevision_ {};
    std::uint64_t scriptAttempt_ {};
    std::filesystem::file_time_type voicingRevision_ {
        std::filesystem::file_time_type::min()
    };
    std::uint32_t voicingPollTicks_ {};
    std::uint64_t voicingReloadCount_ {};

    ui::TopBar topBar_;
    ui::ControlPanel controls_;
    ui::EngineViewport viewport_ { model_ };
    ui::ReadoutStrip readouts_ { model_ };
    ui::SidePanel side_ { model_, [this] { return wheelModifierDown(); } };
    ui::StatusBar status_ { model_ };
    juce::TooltipWindow tooltipWindow_ { this, 650 };
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(MainComponent)
};
} // namespace enginelab
