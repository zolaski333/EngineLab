#pragma once

#include <enginelab/audio/PassByMicrophone.hpp>
#include <enginelab/foundation/EngineTypes.hpp>
#include <enginelab/runtime/PassByRun.hpp>

#include <juce_gui_extra/juce_gui_extra.h>

#include <functional>

namespace enginelab {

/** What the microphone of a pass-by run is set to. */
struct PassByMicrophoneChoice final {
    double laneDistanceM { 7.5 };
    double microphoneHeightM { 1.2 };
    PassByGround ground { PassByGround::asphalt };
    bool normalisedLevel { false };
};

/**
 * A pass-by run (top bar, Pass-by): the car waits at a chosen distance before
 * a trackside microphone, a scripted driver drives a program past it to a
 * chosen distance beyond (EngineRuntime::startPassByRun), and the microphone
 * hears it there (PassByMicrophone): the delay, the Doppler shift, the air
 * and the ground. A strip seen from above shows where the car is, its speed,
 * its gear and the time; Record WAV saves what is heard.
 */
class PassByWindow final : public juce::DocumentWindow {
public:
    struct Callbacks final {
        /** Start (or restart) a run; `record` saves it to a WAV file. */
        std::function<void(const PassByRunSettings&, bool record)> start;
        std::function<void()> stop;
        /** The microphone's place, the ground and the level, at each edit. */
        std::function<void(const PassByMicrophoneChoice&)> setMicrophone;
    };

    PassByWindow(const EngineConfig& config, Callbacks callbacks);
    ~PassByWindow() override;

    void closeButtonPressed() override;
    /** The gear choice follows the running engine's gearbox. */
    void setEngine(const EngineConfig& config);
    /** Where the run is. */
    void setState(const EngineState& state);
    /** What the recorder is doing ("" for nothing). */
    void setRecordingStatus(const juce::String& text);
    /** No run while the dyno runs. */
    void setLocked(bool locked);

    [[nodiscard]] PassByRunSettings runSettings() const;
    [[nodiscard]] PassByMicrophoneChoice microphone() const;
    /** The content (tests find its buttons and fields there). */
    [[nodiscard]] juce::Component* content() const noexcept;

private:
    class Content;
    Content* content_ {};
    juce::TooltipWindow tooltips_ { this, 650 };

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(PassByWindow)
};

/** What a pass-by run is doing, in words. */
[[nodiscard]] juce::String describePassBy(const EngineState& state, const PassByRunSettings& settings);

} // namespace enginelab
