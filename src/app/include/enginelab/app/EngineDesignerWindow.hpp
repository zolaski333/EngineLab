#pragma once

#include <enginelab/catalog/EngineDesign.hpp>

#include <juce_gui_extra/juce_gui_extra.h>

#include <filesystem>
#include <functional>

namespace enginelab {

/**
 * "New engine...": designs an engine from nothing (designEngine()). The
 * simple mode asks for what an engine is (cycle, fuel, use, layout,
 * cylinders, size, induction); the advanced mode also takes the firing order,
 * crank offsets and W bank angles, and lets every derived value be set by
 * hand. The values and the rule behind each one are shown as they change.
 *
 * The callback receives the designed, validated engine and returns whether it
 * was taken (the name may be refused there); the window closes if it was.
 *
 * Before that, the design is run for a few seconds off-screen
 * (measureRealtimeCapacity()); below the margin the build asks first. The
 * running engine is paused meanwhile through `pauseLive`, which sets the pause
 * and returns the previous state.
 */
class EngineDesignerWindow final : public juce::DocumentWindow {
public:
    using BuildCallback = std::function<bool(const EngineConfig&)>;
    using PauseCallback = std::function<bool(bool)>;

    EngineDesignerWindow(std::filesystem::path catalogRoot, BuildCallback buildCallback, PauseCallback pauseLive);
    ~EngineDesignerWindow() override;

    void closeButtonPressed() override;

private:
    class DesignerContent;
    DesignerContent* content_ {};

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(EngineDesignerWindow)
};

} // namespace enginelab
