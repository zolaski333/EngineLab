#pragma once

#include <enginelab/app/ActionMap.hpp>
#include <enginelab/app/DashboardModel.hpp>
#include <enginelab/app/Widgets.hpp>
#include <functional>

namespace enginelab::ui {

/** Left column: ignition, starter, dyno, throttle, driving (speed, gears,
    brake, clutch, cruise control, trip) and the calibration trims. Every
    driving action shows its key. The main component wires the controls to
    the runtime. */
class ControlPanel final : public juce::Component, private juce::Slider::Listener {
public:
    ControlPanel();
    ~ControlPanel() override;
    void setKeyCaps(const ActionMap&);
    void setDiesel(bool diesel);
    /** Shows the engine brake's switch (stages > 0) or hides it (0), and sets
     * it off. `cylinders` names how many each stage brakes. */
    void setEngineBrake(std::uint32_t stages, std::uint32_t cylinders);
    /** The switch's next stage after a key press: up one, off after the last. */
    [[nodiscard]] std::uint32_t nextEngineBrakeStage() const noexcept;
    /** The stage the switch shows. */
    void setEngineBrakeStage(std::uint32_t stage);
    /** Shows the active exhaust valve's switch (an engine with one) or hides it. */
    void setExhaustValveFitted(bool fitted);
    /** The switch's next mode after a key press: automatic, shut, open. */
    [[nodiscard]] ExhaustValveMode nextExhaustValveMode() const noexcept;
    /** The mode the switch shows. */
    void setExhaustValveMode(ExhaustValveMode mode);
    /** Shows the nitrous kit's arming switch (an engine with a kit) or hides it. */
    void setNitrousFitted(bool fitted);
    /** The arming the switch shows. */
    void setNitrousArmed(bool armed);
    [[nodiscard]] bool nitrousArmed() const noexcept { return nitrousArmed_; }
    void refresh(const DashboardModel&);

    /** Called with a throttle fraction (0.01, 0.10, 0.20 or 1.0). */
    std::function<void(double)> onThrottlePreset;
    /** Called with the stage picked from the engine brake's menu. */
    std::function<void(std::uint32_t)> onEngineBrakeStage;
    /** Called with the mode picked from the exhaust valve's menu. */
    std::function<void(ExhaustValveMode)> onExhaustValveMode;
    /** Called when the nitrous arming switch is clicked. */
    std::function<void()> onNitrousArm;

    ActionButton ignition { "Ignition" };
    ActionButton starter { "Starter" };
    ActionButton dyno { "Start dyno", ActionButton::Style::primary };
    juce::Slider throttle;
    ActionButton throttleIdle { percentLabel(1), ActionButton::Style::compact };
    ActionButton throttleLow { percentLabel(10), ActionButton::Style::compact };
    ActionButton throttleMid { percentLabel(20), ActionButton::Style::compact };
    ActionButton throttleFull { percentLabel(100), ActionButton::Style::compact };
    ActionButton gearDown { "Gear down", ActionButton::Style::compact };
    ActionButton gearUp { "Gear up", ActionButton::Style::compact };
    /** Held with the mouse like its key: the brake builds up while held. */
    ActionButton brake { "Brake", ActionButton::Style::compact };
    /** Held: the clutch pedal pressed. */
    ActionButton clutch { "Clutch", ActionButton::Style::compact };
    ActionButton cruise { "Cruise", ActionButton::Style::compact };
    /** Lit when the gearbox shifts by itself; a gear key goes back to manual. */
    ActionButton automatic { "Auto", ActionButton::Style::compact };
    ActionButton resetTrip { "Reset trip", ActionButton::Style::compact };
    /** A truck's engine brake switch, shown only when the engine has one: a
     * click opens the list of its stages; its key steps through them. */
    ActionButton engineBrake { "Engine brake off", ActionButton::Style::compact };
    /** The active exhaust valve, shown only when the exhaust has one: a click
     * opens Automatic / Shut / Open; its key steps through them. Lit while
     * the valve is open. */
    ActionButton exhaustValve { "Exhaust valve auto", ActionButton::Style::compact };
    /** The nitrous kit's arming switch, shown only when a kit is fitted.
     * Lit while the kit sprays. */
    ActionButton nitrous { "Nitrous off", ActionButton::Style::compact };
    SliderRow afrTrim { "AFR trim" };
    SliderRow sparkTrim { "Spark trim" };

    void paint(juce::Graphics&) override;
    void resized() override;
    void mouseUp(const juce::MouseEvent&) override;
    static constexpr int width = 248;

private:
    [[nodiscard]] static juce::String percentLabel(int percent);
    void sliderValueChanged(juce::Slider*) override;
    juce::Rectangle<float> engineLabel_;
    juce::Rectangle<float> throttleLabel_;
    juce::Rectangle<float> throttleValue_;
    juce::Rectangle<float> driveLabel_;
    juce::Rectangle<float> speedValue_;
    juce::Rectangle<float> tripText_;
    juce::String speedText_;
    juce::String speedUnit_;
    juce::String driveDetail_;
    juce::String tripDetail_;
    juce::Rectangle<float> trimsHeader_;
    bool trimsOpen_ { false };
    std::uint32_t engineBrakeStages_ { 0 };
    std::uint32_t engineBrakeStage_ { 0 };
    std::uint32_t engineBrakeCylinders_ { 0 };
    ExhaustValveMode exhaustValveMode_ { ExhaustValveMode::automatic };
    bool nitrousArmed_ { false };
    void openEngineBrakeMenu();
    void openExhaustValveMenu();
};

} // namespace enginelab::ui
