#pragma once
#include <enginelab/audio/RealtimeEngineAudio.hpp>
#include <enginelab/diagnostics/EngineDiagnostics.hpp>
#include <enginelab/foundation/EngineTypes.hpp>
#include <enginelab/runtime/EngineRuntime.hpp>
#include <enginelab/serialization/JsonEngineSerializer.hpp>
#include <enginelab/serialization/YamlEngineSerializer.hpp>
#include <juce_audio_utils/juce_audio_utils.h>
#include <juce_gui_extra/juce_gui_extra.h>
#include <memory>
#include <array>

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
private:
    void timerCallback() override;
    void configureSlider(juce::Slider&, double minimum, double maximum, double value, const juce::String& suffix);
    void selectEngine(int presetIndex);
    void applyConfig(const EngineConfig&);
    void showConfigEditor();
    void showConfigEditor(const juce::String& initialText);
    void importEngine();
    void exportEngine();
    void exportDynoCsv();
    void showError(const juce::String& title, const juce::String& message);
    void collectFinishedRuns();
    void updateHistorySelector();
    void setThrottlePreset(double value);
    void toggleDyno();
    void drawEngine(juce::Graphics&, juce::Rectangle<float> area) const;
    void drawDynoChart(juce::Graphics&, juce::Rectangle<float> area) const;
    void drawTelemetryChart(juce::Graphics&, juce::Rectangle<float> area) const;

    std::vector<EngineConfig> presets_ { makeBaseEnginePresets() };
    EngineConfig config_ { presets_[1] };
    std::unique_ptr<EngineRuntime> runtime_;
    std::unique_ptr<RealtimeEngineAudio> audio_;
    EngineDiagnostics diagnostics_;
    JsonEngineSerializer jsonSerializer_;
    YamlEngineSerializer yamlSerializer_;
    EngineState visibleState_;
    DynoRun visibleCurrentRun_;
    std::vector<DynoRun> archivedRuns_;
    std::vector<Diagnostic> visibleDiagnostics_;
    std::size_t importedRunCount_ { 0 };
    std::uint64_t nextUiRunId_ { 1 };
    bool starterKeyDown_ { false };
    std::array<EngineState, 300> telemetryHistory_ {};
    std::size_t telemetryWrite_ { 0 };
    std::size_t telemetryCount_ { 0 };
    std::unique_ptr<juce::FileChooser> fileChooser_;
    std::unique_ptr<juce::AlertWindow> configEditor_;

    juce::Label title_;
    juce::ComboBox engineSelector_;
    juce::TextButton editButton_ { "EDITER JSON" };
    juce::TextButton importButton_ { "IMPORTER" };
    juce::TextButton exportButton_ { "EXPORTER" };
    juce::TextButton csvButton_ { "CSV DYNO" };
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
