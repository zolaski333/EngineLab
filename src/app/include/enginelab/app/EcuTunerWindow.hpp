#pragma once

#include <enginelab/calibration/CalibrationFileHotReloader.hpp>
#include <enginelab/calibration/CalibrationEditing.hpp>
#include <enginelab/calibration/EcuCalibration.hpp>

#include <juce_gui_extra/juce_gui_extra.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace enginelab {

/** ECU-style map editor backed by transactional realtime calibration snapshots. */
class EcuTunerComponent final : public juce::Component, private juce::Timer {
public:
    explicit EcuTunerComponent(std::shared_ptr<calibration::CalibrationStore> store);
    ~EcuTunerComponent() override;

    void paint(juce::Graphics&) override;
    void paintOverChildren(juce::Graphics&) override;
    void mouseDown(const juce::MouseEvent&) override;
    void resized() override;
    void setOperatingPoint(double rpm, double normalizedLoad);
    void setSessionLocked(bool locked);

private:
    void timerCallback() override;
    void reloadFromSnapshot(bool preserveSelection = true);
    void rebuildCells();
    void updateActiveCellHighlight();
    void commitCell(std::size_t valueIndex);
    void applyTool(calibration::CellOperation operation, double amount);
    void stepHistory(bool undo);
    /** Publishes `edited` live; on success `previous` goes on the undo stack. */
    bool publishEdit(calibration::CalibrationEntry edited, const calibration::CalibrationEntry& previous);
    void updateToolState();
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
    std::vector<std::string> mapIds_;
    juce::TextButton stepDownButton_ { "-" };
    juce::TextButton stepUpButton_ { "+" };
    juce::TextButton percentDownButton_ { "-5 %" };
    juce::TextButton percentUpButton_ { "+5 %" };
    juce::TextButton smoothButton_ { "SMOOTH" };
    juce::TextButton interpolateButton_ { "INTERPOLATE" };
    juce::TextButton undoButton_ { "UNDO" };
    juce::TextButton redoButton_ { "REDO" };
    juce::Label selectionLabel_;
    std::vector<std::size_t> selection_;
    std::size_t selectionAnchor_ {};
    std::vector<calibration::CalibrationEntry> undo_;
    std::vector<calibration::CalibrationEntry> redo_;
    juce::TextButton loadButton_ { "LOAD" };
    juce::TextButton saveButton_ { "SAVE" };
    juce::TextButton refreshButton_ { "RELOAD" };
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
