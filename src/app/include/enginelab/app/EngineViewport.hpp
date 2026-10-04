#pragma once

#include <enginelab/app/DashboardModel.hpp>
#include <enginelab/app/Widgets.hpp>
#include <functional>

namespace enginelab::ui {

/** The 2-D cutaway of the cylinders and valvetrain. Wheel zooms, drag pans,
    double-click recentres. */
class EngineCutawayView final : public juce::Component {
public:
    explicit EngineCutawayView(const DashboardModel&);
    void setLayer(int layer);
    /** True while a wheel-modifier key is held: the wheel then belongs to the
        main component and must not zoom. */
    std::function<bool()> wheelModifierActive;

    void paint(juce::Graphics&) override;
    void mouseWheelMove(const juce::MouseEvent&, const juce::MouseWheelDetails&) override;
    void mouseDown(const juce::MouseEvent&) override;
    void mouseDrag(const juce::MouseEvent&) override;
    void mouseDoubleClick(const juce::MouseEvent&) override;

private:
    void drawEngine(juce::Graphics&, juce::Rectangle<float> area) const;
    const DashboardModel& model_;
    int layer_ { 0 };
    float zoom_ { 1.0F };
    juce::Point<float> pan_ {};
    juce::Point<float> dragStartPan_ {};
};

/** Centre of the window: the engine view with its floating overlays. */
class EngineViewport final : public juce::Component {
public:
    explicit EngineViewport(const DashboardModel&);
    void setWheelModifierCheck(std::function<bool()>);
    void stepLayer(int delta);
    void refresh();

    void paint(juce::Graphics&) override;
    void resized() override;

private:
    const DashboardModel& model_;
    EngineCutawayView cutaway_;
    SegmentedControl layers_ { { "All", "Combustion", "Mechanical", "Gas flow" } };
};

} // namespace enginelab::ui
