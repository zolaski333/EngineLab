#pragma once

#include <cmath>
#include <cstddef>

namespace enginelab {

/**
 * Hysteretic detector for a sustained realtime physics deficit.
 *
 * It deliberately reacts to consecutive missed deadlines, not to one Windows
 * scheduling spike. Recovery is much slower and additionally requires spare
 * execution time, so the protected and normal policies cannot chatter.
 */
class RealtimeLoadGovernor final {
public:
    static constexpr std::size_t missesToEngage = 6;
    static constexpr std::size_t comfortableFramesToRecover = 480;
    static constexpr double comfortableWorkFraction = 0.85;

    /** Returns true only when the active state changes. */
    [[nodiscard]] bool observe(bool missedDeadline,
                               double workFraction) noexcept {
        if (!active_) {
            recoveryFrames_ = 0;
            consecutiveMisses_ = missedDeadline
                ? consecutiveMisses_ + 1 : 0;
            if (consecutiveMisses_ < missesToEngage) return false;
            active_ = true;
            consecutiveMisses_ = 0;
            return true;
        }

        consecutiveMisses_ = 0;
        const auto comfortable = !missedDeadline
            && std::isfinite(workFraction)
            && workFraction <= comfortableWorkFraction;
        recoveryFrames_ = comfortable ? recoveryFrames_ + 1 : 0;
        if (recoveryFrames_ < comfortableFramesToRecover) return false;
        active_ = false;
        recoveryFrames_ = 0;
        return true;
    }

    /** Returns true when reset changed the active state. */
    [[nodiscard]] bool reset() noexcept {
        const auto changed = active_;
        active_ = false;
        consecutiveMisses_ = 0;
        recoveryFrames_ = 0;
        return changed;
    }

    [[nodiscard]] bool active() const noexcept { return active_; }

private:
    bool active_ { false };
    std::size_t consecutiveMisses_ { 0 };
    std::size_t recoveryFrames_ { 0 };
};

} // namespace enginelab
