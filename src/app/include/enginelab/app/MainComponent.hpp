#pragma once
#include <enginelab/audio/RealtimeEngineAudio.hpp>
#include <enginelab/app/ActionMap.hpp>
#include <enginelab/app/AudioWorkshopWindow.hpp>
#include <enginelab/app/EcuTunerWindow.hpp>
#include <enginelab/app/ExhaustDesignerWindow.hpp>
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

namespace enginelab {
/** Desktop presentation layer. It only writes controls and renders snapshots. */
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
    void mouseDown(const juce::MouseEvent&) override;
    void mouseDrag(const juce::MouseEvent&) override;
    void mouseDoubleClick(const juce::MouseEvent&) override;
private:
    void timerCallback() override;
    void configureSlider(juce::Slider&, double minimum, double maximum, double value, const juce::String& suffix);
    void selectEngine(int presetIndex);
    bool applyConfig(const EngineConfig&, bool preserveScriptWatcher = false,
                     bool preserveCalibration = false);
    void showConfigEditor();
    void showConfigEditor(const juce::String& initialText);
    void showKeyBindingsEditor();
    void showAudioWorkshop();
    void showEcuTuner();
    void showExhaustDesigner();
    void startEngineScriptWatcher(const std::filesystem::path&);
    void stopEngineScriptWatcher() noexcept;
    void pollEngineScript();
    void importEngine();
    void exportEngine();
    void exportDynoCsv();
    void showError(const juce::String& title, const juce::String& message);
    void reloadEngine();
    void collectFinishedRuns();
    void updateHistorySelector();
    void setThrottlePreset(double value);
    void updateMomentaryThrottle();
    void toggleDyno();
    void applyExhaustPreset(int presetIndex);
    void updateAudioControlAvailability();
    void configureImpulseResponse();
    [[nodiscard]] OfflineAudioMix currentAudioMix() const noexcept;
    void applyAudioWorkshopMix(
        const OfflineAudioMix& baseMix,
        const OfflineAudioMix& effectiveMix);
    void syncAudioWorkshopMix();
    void adjustAudioOrSimulation(double wheelDelta);
    void drawLoadSimulationPanel(juce::Graphics&, juce::Rectangle<float> area) const;
    void drawMixerPanel(juce::Graphics&, juce::Rectangle<float> area) const;
    void drawOscilloscopePanel(juce::Graphics&, juce::Rectangle<float> area) const;
    void drawDebugPanel(juce::Graphics&, juce::Rectangle<float> area) const;
    void drawGaugeCluster(juce::Graphics&, juce::Rectangle<float> area) const;
    void drawEngine(juce::Graphics&, juce::Rectangle<float> area) const;
    void drawDynoChart(juce::Graphics&, juce::Rectangle<float> area) const;
    void drawTelemetryChart(juce::Graphics&, juce::Rectangle<float> area) const;

    std::vector<EngineConfig> presets_ { makeBaseEnginePresets() };
    std::filesystem::path catalogRoot_;
    EngineConfig config_ { presets_[1] };
    std::unique_ptr<EngineRuntime> runtime_;
    std::unique_ptr<RealtimeEngineAudio> audio_;
    EngineDiagnostics diagnostics_;
    ActionMap actionMap_;
    JsonEngineSerializer jsonSerializer_;
    YamlEngineSerializer yamlSerializer_;
    EngineState visibleState_;
    std::unique_ptr<RenderSnapshotBuilder> renderSnapshotBuilder_;
    RenderSnapshotInterpolator renderSnapshotInterpolator_;
    RenderSnapshot visibleRenderSnapshot_;
    DynoRun visibleCurrentRun_;
    std::vector<DynoRun> archivedRuns_;
    std::vector<Diagnostic> visibleDiagnostics_;
    std::size_t importedRunCount_ { 0 };
    std::uint64_t nextUiRunId_ { 1 };
    bool starterKeyDown_ { false };
    bool brakeKeyDown_ { false };
    bool throttleKeyActive_ { false };
    double targetClutchPressure_ { 1.0 };
    double currentClutchPressure_ { 1.0 };
    int screen_ { 0 };
    int viewLayer_ { 0 };
    float engineViewZoom_ { 1.0F };
    juce::Point<float> engineViewPan_ {};
    juce::Point<float> dragStartPan_ {};
    juce::Rectangle<float> engineViewportArea_ {};
    bool showDynoStats_ { true };
    double audioVolume_ { 1.0 };
    double audioConvolution_ { 0.45 };
    double highFrequencyGain_ { 1.0 };
    double lowFrequencyNoise_ { 0.35 };
    double highFrequencyNoise_ { 0.35 };
    double combustionGain_ { 1.0 };
    double exhaustGain_ { 1.0 };
    double intakeGain_ { 0.85 };
    double mechanicalGain_ { 0.70 };
    int exhaustPresetIndex_ { 0 };
    bool physicalExhaustTopology_ { false };
    bool physicalIntakeTopology_ { false };
    bool structuralRadiationActive_ { false };
    bool forcedInductionAcousticsActive_ { false };
    bool impulseResponseAvailable_ { false };
    bool impulseResponseLoadError_ { false };
    juce::String impulseResponseStatus_ { "IR  CHAMP LIBRE" };
    std::array<EngineState, 300> telemetryHistory_ {};
    std::size_t telemetryWrite_ { 0 };
    std::size_t telemetryCount_ { 0 };
    std::unique_ptr<juce::FileChooser> fileChooser_;
    std::unique_ptr<juce::AlertWindow> configEditor_;
    std::unique_ptr<juce::AlertWindow> keyBindingsEditor_;
    std::unique_ptr<EcuTunerWindow> ecuTunerWindow_;
    std::unique_ptr<ExhaustDesignerWindow> exhaustDesignerWindow_;
    std::unique_ptr<AudioWorkshopWindow> audioWorkshopWindow_;
    std::unique_ptr<scripting::EngineScriptHotReloader> scriptReloader_;
    std::uint64_t scriptRevision_ {};
    std::uint64_t scriptAttempt_ {};

    juce::Label title_;
    juce::ComboBox engineSelector_;
    juce::TextButton editButton_ { "EDITER JSON" };
    juce::TextButton importButton_ { "IMPORTER" };
    juce::TextButton exportButton_ { "EXPORTER" };
    juce::TextButton csvButton_ { "CSV DYNO" };
    juce::TextButton keyBindingsButton_ { "TOUCHES" };
    juce::TextButton ecuTunerButton_ { "ECU" };
    juce::TextButton exhaustDesignerButton_ { "ECHAP. PRO" };
    juce::TextButton audioWorkshopButton_ { "AUDIO HQ" };
    juce::ComboBox exhaustPresetSelector_;
    juce::TextButton ignitionButton_ { "CONTACT" };
    juce::TextButton starterButton_;
    juce::TextButton dynoButton_ { "D  LANCER DYNO" };
    juce::ComboBox historySelector_;
    juce::TextButton deleteRunButton_ { "SUPPRIMER COURBE" };
    juce::Label throttleLabel_, loadLabel_, afrLabel_, advanceLabel_;
    juce::Slider throttleSlider_, loadSlider_, afrSlider_, advanceSlider_;
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(MainComponent)
};
} // namespace enginelab
