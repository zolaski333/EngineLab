#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <vector>

namespace enginelab {

/** Raises one channel's sample rate by an integer factor.
 *
 * A polyphase windowed-sinc interpolator (Kaiser, beta 7.86, about 80 dB of
 * image rejection) with its cutoff at the input's Nyquist frequency: each
 * input sample becomes `factor` output samples. The transition band spans
 * 0.45-0.55 of the input rate, so at a 48 kHz input it passes 21.6 kHz and
 * rejects every image above 26.4 kHz. Latency: tapsPerPhase / 2 input samples.
 * Allocates in prepare() only.
 */
class IntegerUpsampler final {
public:
    static constexpr int tapsPerPhase = 48;

    void prepare(int factor) {
        factor_ = std::max(1, factor);
        const auto length = static_cast<std::size_t>(tapsPerPhase * factor_);
        coefficients_.assign(length, 0.0F);
        constexpr double beta = 7.86;
        const auto centre = 0.5 * static_cast<double>(length - 1);
        for (std::size_t n = 0; n < length; ++n) {
            const auto offset = static_cast<double>(n) - centre;
            // Cutoff at the input Nyquist: 0.5 / factor of the output rate.
            const auto x = offset / static_cast<double>(factor_);
            const auto sinc = std::abs(x) < 1.0e-12
                ? 1.0 : std::sin(std::numbers::pi * x) / (std::numbers::pi * x);
            const auto ratio = offset / centre;
            const auto window = besselI0(beta * std::sqrt(std::max(0.0, 1.0 - ratio * ratio)))
                / besselI0(beta);
            // sinc(n / factor) puts each phase's taps on unit sum, so a
            // passband tone keeps its amplitude.
            // Stored phase by phase, so each output reads its taps in a row.
            const auto phase = n % static_cast<std::size_t>(factor_);
            const auto tap = n / static_cast<std::size_t>(factor_);
            coefficients_[phase * tapsPerPhase + tap] = static_cast<float>(sinc * window);
        }
        history_.assign(2U * tapsPerPhase, 0.0F);
        write_ = 0;
    }

    void reset() noexcept {
        std::fill(history_.begin(), history_.end(), 0.0F);
        write_ = 0;
    }

    [[nodiscard]] int factor() const noexcept { return factor_; }

    /** Writes inputCount * factor() samples to output. */
    void process(const float* input, int inputCount, float* output) noexcept {
        if (history_.empty()) return;
        for (int index = 0; index < inputCount; ++index) {
            // The history is stored twice, so the newest tapsPerPhase samples
            // are always contiguous, the newest at write_ + tapsPerPhase.
            write_ = (write_ + 1) % tapsPerPhase;
            history_[static_cast<std::size_t>(write_)] = input[index];
            history_[static_cast<std::size_t>(write_ + tapsPerPhase)] = input[index];
            const auto* newest = history_.data() + write_ + tapsPerPhase;
            for (int phase = 0; phase < factor_; ++phase) {
                auto sum = 0.0F;
                const auto* tap = coefficients_.data()
                    + static_cast<std::size_t>(phase * tapsPerPhase);
                for (int j = 0; j < tapsPerPhase; ++j)
                    sum += tap[j] * newest[-j];
                *output++ = sum;
            }
        }
    }

private:
    [[nodiscard]] static double besselI0(double x) noexcept {
        auto sum = 1.0;
        auto term = 1.0;
        for (int k = 1; k < 40; ++k) {
            term *= (x / (2.0 * k)) * (x / (2.0 * k));
            sum += term;
            if (term < 1.0e-12 * sum) break;
        }
        return sum;
    }

    int factor_ { 1 };
    int write_ { 0 };
    std::vector<float> coefficients_;
    std::vector<float> history_;
};

} // namespace enginelab
