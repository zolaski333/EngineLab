// ValveTurbulenceNoise: the expected values come from the definition in the
// header, worked by hand, or from the standard deviations the static analysis
// measured on the original filter (1.0 / 3.0 / 5.7 % at 100 Hz / 1 kHz /
// 3.5 kHz, m = 1, 44.1 kHz). None comes from a render.

#include <enginelab/audio/ValveTurbulenceNoise.hpp>

#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>

namespace {

using enginelab::ValveTurbulenceNoise;

constexpr double rate = 44'100.0;
constexpr double density = 1.0;
// A pipe wide enough that the opening's diameter sits on the 19.05 mm cap for
// every opening used below, so the cut-off depends on the velocity alone.
constexpr double pipeArea = 0.01;
constexpr double cappedDiameter = ValveTurbulenceNoise::maximumDiameterM;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

/** Mass flow that puts `velocity` through an opening of `openArea`. */
[[nodiscard]] double flowFor(double velocity, double openArea) {
    return velocity * density * openArea;
}

[[nodiscard]] double modulationDeviation(double cutoffHz) {
    ValveTurbulenceNoise noise;
    noise.reset(12'345U);
    const auto openArea = 0.2 * pipeArea;
    const auto velocity = cutoffHz * cappedDiameter / ValveTurbulenceNoise::strouhalNumber;
    const auto flow = flowFor(velocity, openArea);
    double squares = 0.0, sum = 0.0;
    const auto settle = static_cast<int>(0.1 * rate);
    const auto count = static_cast<int>(20.0 * rate);
    for (int index = 0; index < settle + count; ++index) {
        (void) noise.process(openArea, pipeArea, flow, density, rate);
        if (index < settle) continue;
        const auto value = noise.lastModulation();
        sum += value;
        squares += value * value;
    }
    const auto mean = sum / count;
    return std::sqrt(squares / count - mean * mean);
}

// The analysis measured the deviation of M - 1 on the original filter.
void deviationMatchesTheMeasuredFilter() {
    const struct { double cutoffHz, expected, tolerance; } cases[] {
        { 100.0, 0.010, 0.0015 },
        { 1'000.0, 0.030, 0.003 },
        { 3'500.0, 0.057, 0.004 },
    };
    for (const auto& item : cases) {
        const auto deviation = modulationDeviation(item.cutoffHz);
        require(std::abs(deviation - item.expected) < item.tolerance,
            "the deviation of M - 1 at " + std::to_string(item.cutoffHz)
                + " Hz must be " + std::to_string(item.expected) + ", got "
                + std::to_string(deviation));
    }
}

// Same velocity and capped diameter, so the same cut-off, state and noise:
// a fully open outlet is weighted by 1 - (1 - 0.25) / 0.85 = 0.1176 against a
// valve open below 25 %.
void openOutletIsWeightedDown() {
    ValveTurbulenceNoise small, open;
    small.reset(7U);
    open.reset(7U);
    const auto velocity = 50.0;
    const auto expectedWeight = 1.0 - (1.0 - 0.25) / (1.1 - 0.25);
    for (int index = 0; index < 4'410; ++index) {
        (void) small.process(0.2 * pipeArea, pipeArea, flowFor(velocity, 0.2 * pipeArea),
                             density, rate);
        (void) open.process(pipeArea, pipeArea, flowFor(velocity, pipeArea), density, rate);
        require(std::abs(open.lastModulation() - expectedWeight * small.lastModulation())
                    < 1.0e-12,
            "a fully open outlet must carry 0.1176 of a small opening's modulation");
    }
}

// A ramp of 4 mm2 per step at 54 m/s through a capped 19.05 mm opening:
// tau = 0.1 d / v / 2 pi = 5.57 us, k = dt / (tau + dt) = 0.803, and the state
// settles where m = (1 - k) m + k - 50000 x 4e-6, i.e. m = 1 - 0.2 / k = 0.751.
void fastOpeningCollapsesTheState() {
    ValveTurbulenceNoise noise;
    noise.reset(3U);
    const auto velocity = 54.0;
    auto area = 0.05 * pipeArea;
    for (int index = 0; index < 200; ++index)
        (void) noise.process(area, pipeArea, flowFor(velocity, area), density, rate);
    require(noise.turbulence() > 0.999, "a steady opening must reach full turbulence");
    for (int index = 0; index < 50; ++index) {
        area += 4.0e-6;
        (void) noise.process(area, pipeArea, flowFor(velocity, area), density, rate);
    }
    const auto dt = 1.0 / rate;
    const auto tau = 0.1 * cappedDiameter / velocity / (2.0 * std::numbers::pi);
    const auto k = dt / (tau + dt);
    const auto expected = 1.0 - 50'000.0 * 4.0e-6 / k;
    require(std::abs(noise.turbulence() - expected) < 0.005,
        "a fast area ramp must hold the state at " + std::to_string(expected)
            + ", got " + std::to_string(noise.turbulence()));
}

void closedOrStillValvesAreSilent() {
    ValveTurbulenceNoise noise;
    noise.reset(1U);
    for (int index = 0; index < 1'000; ++index) {
        require(noise.process(0.0, pipeArea, 0.05, density, rate) == 0.0,
            "a closed valve must not fluctuate");
        require(noise.process(0.1 * pipeArea, pipeArea, 0.0, density, rate) == 0.0,
            "no flow, no fluctuation");
    }
    require(noise.process(std::numeric_limits<double>::quiet_NaN(), pipeArea, 0.05,
                          density, rate) == 0.0
            && noise.process(0.1 * pipeArea, pipeArea,
                             std::numeric_limits<double>::infinity(), density, rate) == 0.0,
        "non-finite inputs must come out as silence");
}

} // namespace

int main() {
    try {
        deviationMatchesTheMeasuredFilter();
        openOutletIsWeightedDown();
        fastOpeningCollapsesTheState();
        closedOrStillValvesAreSilent();
    } catch (const std::exception& error) {
        std::cerr << "ValveTurbulenceNoise FAIL: " << error.what() << '\n';
        return 1;
    }
    std::cout << "ValveTurbulenceNoise: measured deviation, open-outlet weight, "
                 "collapse on fast ramps, silence when closed PASS\n";
    return 0;
}
