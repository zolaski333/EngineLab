#pragma once

#include <enginelab/app/DashboardModel.hpp>

namespace enginelab::ui {

/** Bottom line: fault summary, real-time factor, simulation speed, live
    reload state and the cumulative real-time drop counters. */
class StatusBar final : public juce::Component {
public:
    explicit StatusBar(const DashboardModel&);
    /** Repaints only when the text actually changed. */
    void refresh();
    void paint(juce::Graphics&) override;
    static constexpr int height = 28;

private:
    struct Content final {
        juce::String fault;
        juce::Colour faultColour;
        juce::StringArray items;
        juce::Colour realtimeColour;
        juce::String counters;
        bool operator==(const Content&) const = default;
    };
    [[nodiscard]] Content compose() const;
    const DashboardModel& model_;
    Content content_;
};

} // namespace enginelab::ui
