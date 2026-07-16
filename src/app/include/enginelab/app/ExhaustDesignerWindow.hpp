#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

#include <juce_gui_extra/juce_gui_extra.h>

#include <functional>

namespace enginelab {

/**
 * Non-destructive editor for the configurable exhaust network of an engine.
 *
 * The window owns a working copy. The callback is invoked only after the
 * complete EngineConfig has passed validateEngineConfig(). Closing the window
 * therefore never mutates the running engine accidentally.
 */
class ExhaustDesignerWindow final : public juce::DocumentWindow {
public:
    using ApplyCallback = std::function<bool(const EngineConfig&)>;

    ExhaustDesignerWindow(const EngineConfig& config, ApplyCallback applyCallback);
    ~ExhaustDesignerWindow() override;

    void closeButtonPressed() override;
    void setConfig(const EngineConfig& config);

private:
    class DesignerContent;
    DesignerContent* content_ {};

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(ExhaustDesignerWindow)
};

} // namespace enginelab
