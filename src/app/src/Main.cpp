#include <enginelab/app/MainComponent.hpp>
#include <JuceHeader.h>

namespace enginelab {
class EngineLabApplication final : public juce::JUCEApplication {
public:
    [[nodiscard]] const juce::String getApplicationName() override { return "EngineLab"; }
    [[nodiscard]] const juce::String getApplicationVersion() override { return "0.1.0"; }
    void initialise(const juce::String&) override { window_ = std::make_unique<MainWindow>(getApplicationName()); }
    void shutdown() override { window_.reset(); }
private:
    class MainWindow final : public juce::DocumentWindow {
    public:
        explicit MainWindow(const juce::String& name)
            : DocumentWindow(name, juce::Colour(0xff090d0c), DocumentWindow::allButtons) {
            setUsingNativeTitleBar(true); setContentOwned(new MainComponent(), true);
            setResizable(true, true);
            setResizeLimits(1'180, 700, 2'560, 1'440);
            centreWithSize(getWidth(), getHeight()); setVisible(true);
        }
        void closeButtonPressed() override { juce::JUCEApplication::getInstance()->systemRequestedQuit(); }
    };
    std::unique_ptr<MainWindow> window_;
};
} // namespace enginelab

START_JUCE_APPLICATION(enginelab::EngineLabApplication)
