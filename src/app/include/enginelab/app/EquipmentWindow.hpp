#pragma once

#include <enginelab/calibration/CalibrationStore.hpp>
#include <enginelab/foundation/EngineTypes.hpp>

#include <juce_gui_extra/juce_gui_extra.h>

#include <functional>
#include <memory>
#include <string_view>

namespace enginelab {

/**
 * Every optional part in one place (top bar, Equipment): the nitrous kit,
 * anti-lag, the active exhaust valve, VTEC and launch control. Each shows
 * whether it is fitted, fits it in one click, shows all of its settings on
 * one page and says, live, what it is doing or what it waits for. Every edit
 * is taken by the running engine at once.
 */
class EquipmentWindow final : public juce::DocumentWindow {
public:
    struct Callbacks final {
        /** A kit fitted, edited or taken off; false when refused. */
        std::function<bool(const NitrousConfig&)> applyNitrous;
        std::function<void()> refillNitrous;
        /** An edited engine (the valve's settings, a fitted valve, VTEC
         *  cams); false when refused. */
        std::function<bool(const EngineConfig&)> applyEngine;
        /** The valve's mode, as Drive's switch. */
        std::function<void(ExhaustValveMode)> setValveMode;
        std::function<void()> openExhaustDesigner;
        /** Fits VTEC cams to the engine (and its tables their settings). */
        std::function<void()> fitVtecCams;
    };

    EquipmentWindow(const EngineConfig& config, std::shared_ptr<calibration::CalibrationStore> store,
                    Callbacks callbacks);
    ~EquipmentWindow() override;

    void closeButtonPressed() override;
    /** The running engine and its tables. */
    void setEngine(const EngineConfig& config, std::shared_ptr<calibration::CalibrationStore> store);
    /** What each part is doing now. */
    void setState(const EngineState& state);
    void setValveMode(ExhaustValveMode mode);
    /** No edit while the dyno runs. */
    void setLocked(bool locked);

    /** The content (tests find its buttons and fields there). */
    [[nodiscard]] juce::Component* content() const noexcept;

    /** A scalar setting of the tables, published at once: false when the
     *  tables lack it or refuse the value. */
    static bool publishScalar(calibration::CalibrationStore& store, std::string_view key, double value);

private:
    class Content;
    Content* content_ {};
    juce::TooltipWindow tooltips_ { this, 650 };

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(EquipmentWindow)
};

/** What a nitrous kit waits for, in words ("" when it sprays or none). */
[[nodiscard]] juce::String describeNitrousBlockers(std::uint32_t blockers, const NitrousConfig& kit);

} // namespace enginelab
