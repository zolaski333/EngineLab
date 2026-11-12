#pragma once

#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/foundation/EngineTypes.hpp>

#include <juce_gui_extra/juce_gui_extra.h>

#include <functional>
#include <vector>

namespace enginelab {

/**
 * The fuel in the tank: a listed fuel, or its properties one at a time.
 *
 * Edits stay in the window until Apply, because the engine restarts to take
 * them. The 3-D view's combustion chamber edits the same properties.
 */
class FuelWindow final : public juce::DocumentWindow {
public:
    using ApplyCallback = std::function<bool(const EngineConfig&)>;
    /** The nitrous kit's section: `apply` takes an edited kit at once (live),
        false when it could not; `refill` fills its bottle. */
    struct NitrousCallbacks final {
        std::function<bool(const NitrousConfig&)> apply;
        std::function<void()> refill;
    };

    FuelWindow(const EngineConfig& config, std::vector<FuelPreset> presets, ApplyCallback applyCallback,
               NitrousCallbacks nitrous = {});
    ~FuelWindow() override;

    void closeButtonPressed() override;
    /** The running engine: shown again unless the window holds unapplied edits
        of the same engine. */
    void setConfig(const EngineConfig& config);
    /** A line under the fields, e.g. the knock limit being measured again. */
    void setStatus(const juce::String& status);
    /** No edit while the dyno runs. */
    void setLocked(bool locked);
    /** The kit's bottle and spray, read from the running engine. */
    void setNitrousState(const EngineState& state);

private:
    class Content;
    Content* content_ {};
    /** Its own tooltips: a tooltip window parented to another window shows
        none over this one. */
    juce::TooltipWindow tooltips_ { this, 650 };

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(FuelWindow)
};

} // namespace enginelab
