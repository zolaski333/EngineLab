#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>

namespace enginelab {

/**
 * Level-normalising automatic gain followed by a tanh soft clipper.
 *
 * This is a MIXING stage, not acoustics. It exists to answer one measured
 * question: how much of the character another engine simulator is praised for
 * comes from its output stage rather than from its gas dynamics? The static
 * analysis of that simulator (journal, 2026-10-13) found this stage at the very
 * end of its chain, enabled by default, after the room convolution:
 *
 *     env   = 2 |LP(max_c |x_c|)|            LP: RBJ low-pass, 5 Hz, Q 0.5
 *     level follows env through a second, identical low-pass:
 *             rising   -> the low-pass is stepped 4 times per sample (attack),
 *                         and a 1 s hold is re-armed;
 *             falling  -> frozen while the hold runs, then stepped once;
 *     G     = min(1 / level, 10)
 *     y_c   = tanh(G x_c)
 *
 * The algorithm is re-expressed here from that description; nothing else is
 * taken from the other program. Two details are deliberate departures: the hold
 * counts real seconds at any sample rate, and a non-finite input sample counts
 * as silence.
 *
 * What it does to an engine: `env` sits near twice the mean absolute level, so
 * the steady-state gain maps that level to about 0.5 and a sine settles at a
 * peak of tanh(pi/4) = 0.656 whatever its amplitude. Anything whose crest
 * factor exceeds about 6 dB reaches the tanh well beyond its linear region: the
 * firing pulses are flattened towards a common ceiling. The output loudness is
 * therefore almost independent of the input, and the crest factor is capped.
 *
 * It is not on the default path of `RealtimeEngineAudio`. The measurement
 * harnesses apply it to a finished render to compare with and without it.
 */
class AutoGainSoftClipper final {
public:
    struct Settings final {
        double envelopeCutoffHz { 5.0 };
        double envelopeQ { 0.5 };
        /** env = scale x low-passed peak magnitude. */
        double envelopeScale { 2.0 };
        /** Low-pass steps per sample while the envelope rises. */
        int attackSteps { 4 };
        double holdSeconds { 1.0 };
        double maximumGain { 10.0 };
    };

    AutoGainSoftClipper() noexcept { prepare(48'000.0, Settings {}); }

    void prepare(double sampleRateHz, const Settings& settings) noexcept {
        settings_ = settings;
        sampleRateHz_ = sampleRateHz > 0.0 ? sampleRateHz : 48'000.0;
        // RBJ cookbook low-pass, normalised by a0.
        const auto omega = 2.0 * std::numbers::pi
            * std::clamp(settings_.envelopeCutoffHz, 0.01, 0.45 * sampleRateHz_)
            / sampleRateHz_;
        const auto alpha = std::sin(omega) / (2.0 * std::max(settings_.envelopeQ, 1.0e-3));
        const auto cosine = std::cos(omega);
        const auto a0 = 1.0 + alpha;
        coefficients_.b0 = (1.0 - cosine) * 0.5 / a0;
        coefficients_.b1 = (1.0 - cosine) / a0;
        coefficients_.b2 = coefficients_.b0;
        coefficients_.a1 = -2.0 * cosine / a0;
        coefficients_.a2 = (1.0 - alpha) / a0;
        reset();
    }

    void reset() noexcept {
        detector_ = {};
        follower_ = {};
        level_ = 0.0;
        holdRemainingSeconds_ = 0.0;
    }

    /** Current gain, i.e. the one the next sample would be driven with. */
    [[nodiscard]] double gain() const noexcept {
        return level_ > 1.0 / settings_.maximumGain ? 1.0 / level_ : settings_.maximumGain;
    }

    /** Process one stereo frame in place. The gain is linked: both channels
     *  are driven by the louder one, so the image does not wander. */
    void process(float& left, float& right) noexcept {
        const auto l = std::isfinite(left) ? static_cast<double>(left) : 0.0;
        const auto r = std::isfinite(right) ? static_cast<double>(right) : 0.0;
        advance(std::max(std::abs(l), std::abs(r)));
        const auto g = gain();
        left = static_cast<float>(std::tanh(g * l));
        right = static_cast<float>(std::tanh(g * r));
    }

    /** Process a mono sample. */
    [[nodiscard]] float process(float sample) noexcept {
        const auto x = std::isfinite(sample) ? static_cast<double>(sample) : 0.0;
        advance(std::abs(x));
        return static_cast<float>(std::tanh(gain() * x));
    }

private:
    struct Coefficients final {
        double b0 { 1.0 }, b1 { 0.0 }, b2 { 0.0 }, a1 { 0.0 }, a2 { 0.0 };
    };
    /** Transposed direct form II. */
    struct Section final {
        double z1 { 0.0 }, z2 { 0.0 };
        [[nodiscard]] double step(const Coefficients& c, double x) noexcept {
            const auto y = c.b0 * x + z1;
            z1 = c.b1 * x - c.a1 * y + z2;
            z2 = c.b2 * x - c.a2 * y;
            return y;
        }
    };

    void advance(double magnitude) noexcept {
        const auto envelope = settings_.envelopeScale
            * std::abs(detector_.step(coefficients_, magnitude));
        if (envelope > level_) {
            for (int step = 0; step < std::max(1, settings_.attackSteps); ++step)
                level_ = follower_.step(coefficients_, envelope);
            holdRemainingSeconds_ = settings_.holdSeconds;
        } else if (holdRemainingSeconds_ > 0.0) {
            holdRemainingSeconds_ -= 1.0 / sampleRateHz_;
        } else {
            level_ = follower_.step(coefficients_, envelope);
        }
        if (!std::isfinite(level_) || level_ < 0.0) {
            follower_ = {};
            level_ = 0.0;
        }
    }

    Settings settings_ {};
    double sampleRateHz_ { 48'000.0 };
    Coefficients coefficients_ {};
    Section detector_ {};
    Section follower_ {};
    double level_ { 0.0 };
    double holdRemainingSeconds_ { 0.0 };
};

} // namespace enginelab
