#pragma once

#include <enginelab/calibration/CalibrationFileHotReloader.hpp>
#include <enginelab/calibration/CalibrationEditing.hpp>
#include <enginelab/calibration/EcuCalibration.hpp>

#include <juce_gui_extra/juce_gui_extra.h>

#include <cstddef>
#include <cstdint>
#include <functional>
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
    /** The fuel in the tank. A spark engine's target AFR map is stored in
     *  petrol AFR (lambda x 14.7); it is shown and typed in this fuel's own
     *  AFR, so a methanol map reads 5.8 where petrol reads 13.2. */
    void setFuel(const std::string& name, double stoichiometricAirFuelRatio, bool diesel);
    /** What a stored value of `id` reads on screen (1 but for a spark
     *  engine's target AFR). */
    [[nodiscard]] double displayScaleOf(const std::string& id) const noexcept;
    /** Whether the engine could take VTEC cams (not a two-stroke). The
     *  VTEC button shows the ECU's switch when the tables carry VTEC
     *  settings, otherwise offers to fit the cams (onFitVtecCams). */
    void setVtecFittable(bool fittable);
    /** Asks the owner to fit VTEC cams to the running engine. */
    std::function<void()> onFitVtecCams;
    /** The VTEC button's text now (for tests). */
    [[nodiscard]] juce::String vtecButtonText() const { return vtecButton_.getButtonText(); }
    [[nodiscard]] bool vtecButtonVisible() const { return vtecButton_.isVisible(); }
    /** Presses the VTEC button, as a click does. */
    void pressVtecButton();
    /** Completes tables loaded from a file (LOAD and its hot reload) with
        the settings the file predates. */
    void setTableCompletion(calibration::DraftCompletion complete);

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
    void updateVtecButton();
    void chooseCalibrationToLoad();
    void chooseCalibrationToSave();
    void showIssues(const calibration::PublishResult&, const juce::String& title);
    [[nodiscard]] juce::String selectedCalibrationId() const;

    std::shared_ptr<calibration::CalibrationStore> store_;
    calibration::CalibrationDraft draft_;
    std::unique_ptr<calibration::CalibrationFileHotReloader> hotReloader_;
    calibration::DraftCompletion completeTables_;
    std::uint64_t baseRevision_ {};
    double operatingRpm_ {};
    double operatingLoad_ {};
    bool rebuilding_ { false };
    bool sessionLocked_ { false };
    std::string fuelName_;
    double afrDisplayScale_ { 1.0 };
    /** A spark engine whose fuel is known: its AFR map reads in that fuel. */
    bool fuelAfr_ { false };

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
    juce::TextButton vtecButton_ { "FIT VTEC CAMS" };
    bool vtecFittable_ { false };
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
    void setFuel(const std::string& name, double stoichiometricAirFuelRatio, bool diesel);
    void setVtecFittable(bool fittable);
    /** The tuner itself (its VTEC callback). */
    [[nodiscard]] EcuTunerComponent* tuner() const noexcept { return tuner_; }

private:
    EcuTunerComponent* tuner_ {};
    /** Its own tooltips: a tooltip window parented to another window shows
        none over this one. */
    juce::TooltipWindow tooltips_ { this, 650 };
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(EcuTunerWindow)
};

} // namespace enginelab
