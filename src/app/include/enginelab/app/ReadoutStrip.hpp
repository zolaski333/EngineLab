#pragma once

#include <enginelab/app/DashboardModel.hpp>

namespace enginelab::ui {

/** Row of live figures under the engine view: tachometer, torque, power,
    manifold pressure, mixture, exhaust gas and coolant. */
class ReadoutStrip final : public juce::Component {
public:
    explicit ReadoutStrip(const DashboardModel&);
    void paint(juce::Graphics&) override;
    static constexpr int height = 92;

private:
    void drawTachometer(juce::Graphics&, juce::Rectangle<float> area) const;
    const DashboardModel& model_;
};

} // namespace enginelab::ui
