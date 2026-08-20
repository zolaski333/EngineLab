#pragma once

#include <enginelab/calibration/CalibrationFileHotReloader.hpp>
#include <enginelab/calibration/EcuCalibration.hpp>

#include <juce_gui_extra/juce_gui_extra.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace enginelab {

/** ECU-style map editor backed by transactional realtime calibration snapshots. */
class EcuTunerComponent final : public juce::Component, private juce::Timer {
public:
    explicit EcuTunerComponent(std::shared_ptr<calibration::CalibrationStore> store);
    ~EcuTunerComponent() override;

    void paint(juce::Graphics&) override;
    void resized() override;
    void setOperatingPoint(double rpm, double normalizedLoad);
    void setSessionLocked(bool locked);

private:
    void timerCallback() override;
    void reloadFromSnapshot(bool preserveSelection = true);
    void rebuildCells();
    void updateActiveCellHighlight();
    void commitCell(std::size_t valueIndex);
    void chooseCalibrationToLoad();
    void chooseCalibrationToSave();
    void showIssues(const calibration::PublishResult&, const juce::String& title);
    [[nodiscard]] juce::String selectedCalibrationId() const;

    std::shared_ptr<calibration::CalibrationStore> store_;
    calibration::CalibrationDraft draft_;
    std::unique_ptr<calibration::CalibrationFileHotReloader> hotReloader_;
    std::uint64_t baseRevision_ {};
    double operatingRpm_ {};
    double operatingLoad_ {};
    bool rebuilding_ { false };
    bool sessionLocked_ { false };

    juce::ComboBox mapSelector_;
    juce::TextButton loadButton_ { "CHARGER" };
    juce::TextButton saveButton_ { "ENREGISTRER" };
    juce::TextButton refreshButton_ { "RECHARGER" };
    juce::Label statusLabel_;
    juce::Label descriptionLabel_;
    std::vector<std::unique_ptr<juce::Label>> xLabels_;
    std::vector<std::unique_ptr<juce::Label>> yLabels_;
    std::vector<std::unique_ptr<juce::TextEditor>> valueEditors_;
    std::vector<juce::String> committedTexts_;
    std::size_t xCount_ { 1 };
    std::size_t yCount_ { 1 };
    std::unique_ptr<juce::FileChooser> fileChooser_;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(EcuTunerComponent)
};

class EcuTunerWindow final : public juce::DocumentWindow {
public:
    explicit EcuTunerWindow(std::shared_ptr<calibration::CalibrationStore> store);
    void closeButtonPressed() override;
    void setOperatingPoint(double rpm, double normalizedLoad);
    void setSessionLocked(bool locked);

private:
    EcuTunerComponent* tuner_ {};
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(EcuTunerWindow)
};

} // namespace enginelab
