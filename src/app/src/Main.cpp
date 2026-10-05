#include <enginelab/app/MainComponent.hpp>
#include <enginelab/app/Theme.hpp>
#include <JuceHeader.h>

namespace enginelab {
class EngineLabApplication final : public juce::JUCEApplication {
public:
    [[nodiscard]] const juce::String getApplicationName() override { return "EngineLab"; }
    [[nodiscard]] const juce::String getApplicationVersion() override { return ProjectInfo::versionString; }
    void initialise(const juce::String&) override {
        // Installed before any window exists so every window and alert box
        // shares the theme; it outlives them all (see shutdown()).
        juce::LookAndFeel::setDefaultLookAndFeel(&lookAndFeel_);
        window_ = std::make_unique<MainWindow>(getApplicationName());
    }
    void shutdown() override {
        window_.reset();
        juce::LookAndFeel::setDefaultLookAndFeel(nullptr);
    }
private:
    class MainWindow final : public juce::DocumentWindow {
    public:
        explicit MainWindow(const juce::String& name)
            : DocumentWindow(name, ui::colours::background, DocumentWindow::allButtons) {
            setUsingNativeTitleBar(true); setContentOwned(new MainComponent(), true);
            setResizable(true, true);
            // Left controls (248 px) + right panel (380 px) leave the engine
            // view and its seven readouts about 650 px at the minimum width.
            setResizeLimits(1'280, 720, 3'840, 2'160);
            centreWithSize(getWidth(), getHeight()); setVisible(true);
        }
        void closeButtonPressed() override { juce::JUCEApplication::getInstance()->systemRequestedQuit(); }
    };
    ui::LookAndFeel lookAndFeel_;
    std::unique_ptr<MainWindow> window_;
};
} // namespace enginelab

START_JUCE_APPLICATION(enginelab::EngineLabApplication)
