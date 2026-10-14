#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>

namespace enginelab {

/**
 * Turbulent fluctuation of a valve's effective flow area, as a mass-flow
 * source.
 *
 * Re-expressed from a static analysis of another engine simulator (journal,
 * 2026-10-13), which modulates the effective area of every valve and outlet
 * inside its gas solver:
 *
 *     psi      open area / pipe area
 *     v        |u_pipe| / max(psi, 0.001), the velocity through the opening
 *     d        min(2 sqrt(psi A_pipe / pi), 19.05 mm)
 *     n        uniform noise in [0, 1), one draw per step
 *     f_c      clamp(0.35 v / d, 100, 3500) Hz; n goes through four identical
 *              RBJ low-pass sections, Q 0.707 (eighth order, unity DC gain)
 *     m        turbulence state: m <- (1 - k) m + k target - 50000 |dA_open|,
 *              clamped to [0, 1]; target = 1 while open, k = dt / (tau + dt),
 *              tau = 0.1 d / max(v, 0.001) / (2 pi)
 *     M        1 + 0.3 m (2 n_f - 1)
 *     x        clamp((psi - 0.25) / (1.1 - 0.25), 0, 1)
 *     psi_eff  psi [(1 - x) M + x]
 *
 * Here the gas network cannot carry that band (its cells are hundreds of
 * millimetres long and it runs at the coupling rate), so the same modulation
 * is applied where the audio meets the valve: an orifice's mass flow scales
 * with its area, so the fluctuation is a mass-flow source
 * dm = m_dot (1 - x)(M - 1), injected as a Norton volume velocity next to the
 * valve's high-band source. Departures: every valve draws its own noise
 * stream (the other program seeds all of them identically, which correlates
 * them), the state follows real seconds at any sample rate except the
 * 50000 per-step area term, which is kept per step as in the original.
 *
 * Off by default; RealtimeEngineAudio::setValveTurbulenceNoiseEnabled.
 */
class ValveTurbulenceNoise final {
public:
    static constexpr double modulationDepth = 0.3;
    static constexpr double fullWeightOpening = 0.25;
    static constexpr double noWeightOpening = 1.1;
    static constexpr double strouhalNumber = 0.35;
    static constexpr double minimumCutoffHz = 100.0;
    static constexpr double maximumCutoffHz = 3'500.0;
    static constexpr double maximumDiameterM = 0.01905;
    static constexpr double timeConstantDiameters = 0.1;
    static constexpr double collapsePerSquareMetre = 50'000.0;
    static constexpr double butterworthQ = 0.707;

    void reset(std::uint32_t seed) noexcept {
        sections_ = {};
        turbulence_ = 0.0;
        previousOpenAreaM2_ = 0.0;
        rng_ = seed != 0U ? seed : 0x9E3779B9U;
        lastModulation_ = 0.0;
    }

    /** One audio sample. Returns the mass-flow fluctuation in kg/s. */
    [[nodiscard]] double process(double openAreaM2, double pipeAreaM2,
                                 double massFlowKgPerSecond, double densityKgPerM3,
                                 double sampleRateHz) noexcept {
        if (!(pipeAreaM2 > 0.0) || !(sampleRateHz > 0.0)
            || !std::isfinite(openAreaM2) || !std::isfinite(massFlowKgPerSecond)
            || !(densityKgPerM3 > 0.0)) {
            lastModulation_ = 0.0;
            return 0.0;
        }
        const auto area = std::max(openAreaM2, 0.0);
        const auto opening = std::min(area / pipeAreaM2, 1.0);
        const auto areaChange = std::abs(area - previousOpenAreaM2_);
        previousOpenAreaM2_ = area;
        const auto noise = nextUniform();
        if (!(opening > 0.0)) {
            // Closed: the target is 0 and d = 0 gives tau = 0, so the state
            // drops to zero at once; the filter is not run, as in the original.
            turbulence_ = 0.0;
            lastModulation_ = 0.0;
            return 0.0;
        }
        const auto pipeVelocity = std::abs(massFlowKgPerSecond)
            / (densityKgPerM3 * pipeAreaM2);
        const auto openingVelocity = pipeVelocity / std::max(opening, 0.001);
        const auto diameter = std::min(
            2.0 * std::sqrt(opening * pipeAreaM2 / std::numbers::pi), maximumDiameterM);
        const auto dt = 1.0 / sampleRateHz;
        const auto tau = timeConstantDiameters * diameter
            / std::max(openingVelocity, 0.001) / (2.0 * std::numbers::pi);
        const auto k = dt / (tau + dt);
        turbulence_ = std::clamp((1.0 - k) * turbulence_ + k
            - collapsePerSquareMetre * areaChange, 0.0, 1.0);

        const auto cutoff = std::clamp(strouhalNumber * openingVelocity
            / std::max(diameter, 1.0e-6), minimumCutoffHz,
            std::min(maximumCutoffHz, 0.45 * sampleRateHz));
        const auto coefficients = lowPass(cutoff, sampleRateHz);
        auto filtered = noise;
        for (auto& section : sections_) filtered = section.step(coefficients, filtered);

        const auto modulation = modulationDepth * turbulence_ * (2.0 * filtered - 1.0);
        const auto weight = 1.0 - std::clamp((opening - fullWeightOpening)
            / (noWeightOpening - fullWeightOpening), 0.0, 1.0);
        lastModulation_ = weight * modulation;
        return massFlowKgPerSecond * lastModulation_;
    }

    /** psi_eff / psi - 1 of the last sample. */
    [[nodiscard]] double lastModulation() const noexcept { return lastModulation_; }
    [[nodiscard]] double turbulence() const noexcept { return turbulence_; }

private:
    struct Coefficients final {
        double b0 {}, b1 {}, b2 {}, a1 {}, a2 {};
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

    [[nodiscard]] static Coefficients lowPass(double cutoffHz, double sampleRateHz) noexcept {
        const auto omega = 2.0 * std::numbers::pi * cutoffHz / sampleRateHz;
        const auto alpha = std::sin(omega) / (2.0 * butterworthQ);
        const auto cosine = std::cos(omega);
        const auto a0 = 1.0 + alpha;
        Coefficients c;
        c.b0 = (1.0 - cosine) * 0.5 / a0;
        c.b1 = (1.0 - cosine) / a0;
        c.b2 = c.b0;
        c.a1 = -2.0 * cosine / a0;
        c.a2 = (1.0 - alpha) / a0;
        return c;
    }

    /** xorshift32, uniform in [0, 1) with 24 bits. */
    [[nodiscard]] double nextUniform() noexcept {
        rng_ ^= rng_ << 13U;
        rng_ ^= rng_ >> 17U;
        rng_ ^= rng_ << 5U;
        return static_cast<double>(rng_ >> 8U) / 16'777'216.0;
    }

    std::array<Section, 4> sections_ {};
    double turbulence_ { 0.0 };
    double previousOpenAreaM2_ { 0.0 };
    double lastModulation_ { 0.0 };
    std::uint32_t rng_ { 0x9E3779B9U };
};

} // namespace enginelab
