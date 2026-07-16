#pragma once

#include <algorithm>
#include <cmath>

namespace enginelab {

/** A wall-clock interval reserved for one simulation frame's publications. */
struct RealtimePublicationWindow final {
    double startSeconds { 0.0 };
    double durationSeconds { 0.0 };

    [[nodiscard]] double endSeconds() const noexcept {
        return startSeconds + durationSeconds;
    }

    [[nodiscard]] double mapSimulationTime(double simulationTimeSeconds,
                                           double simulationStartSeconds,
                                           double simulationDurationSeconds) const noexcept {
        if (!(simulationDurationSeconds > 0.0) || !std::isfinite(simulationDurationSeconds))
            return startSeconds;
        const auto fraction = std::isfinite(simulationTimeSeconds)
            && std::isfinite(simulationStartSeconds)
            ? std::clamp((simulationTimeSeconds - simulationStartSeconds)
                / simulationDurationSeconds, 0.0, 1.0)
            : 0.0;
        return std::fma(fraction, durationSeconds, startSeconds);
    }
};

/**
 * Reserves non-overlapping realtime windows for successive simulation frames.
 *
 * A late frame may begin after its nominal deadline. If the following frame
 * then catches up, mapping both frames from their observed wall-clock starts
 * would overlap and could publish decreasing timestamps. This timeline moves
 * the later window forward just enough to retain monotonic ordering.
 */
class MonotonicPublicationTimeline final {
public:
    [[nodiscard]] RealtimePublicationWindow beginWindow(double observedStartSeconds,
                                                        double durationSeconds) noexcept {
        const auto safeObservedStart = std::isfinite(observedStartSeconds)
            ? std::max(0.0, observedStartSeconds) : nextWindowStartSeconds_;
        const auto safeDuration = std::isfinite(durationSeconds)
            ? std::max(0.0, durationSeconds) : 0.0;
        const auto start = initialised_
            ? std::max(safeObservedStart, nextWindowStartSeconds_) : safeObservedStart;
        const RealtimePublicationWindow window { start, safeDuration };
        nextWindowStartSeconds_ = window.endSeconds();
        initialised_ = true;
        return window;
    }

private:
    double nextWindowStartSeconds_ { 0.0 };
    bool initialised_ { false };
};

} // namespace enginelab
