// AutoGainSoftClipper: the expected values below come from the stage's own
// definition (see the header), worked out by hand, never from a render.

#include <enginelab/audio/AutoGainSoftClipper.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using enginelab::AutoGainSoftClipper;

constexpr double rate = 48'000.0;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

[[nodiscard]] double sine(double amplitude, std::size_t index) {
    return amplitude * std::sin(2.0 * std::numbers::pi * 100.0
        * static_cast<double>(index) / rate);
}

[[nodiscard]] double peakOf(const std::vector<float>& signal, std::size_t begin,
                            std::size_t end) {
    double peak = 0.0;
    for (auto index = begin; index < end; ++index)
        peak = std::max(peak, std::abs(static_cast<double>(signal[index])));
    return peak;
}

[[nodiscard]] double crestDb(const std::vector<float>& signal, std::size_t begin) {
    double squares = 0.0, peak = 0.0;
    for (auto index = begin; index < signal.size(); ++index) {
        const auto value = static_cast<double>(signal[index]);
        squares += value * value;
        peak = std::max(peak, std::abs(value));
    }
    const auto rms = std::sqrt(squares / static_cast<double>(signal.size() - begin));
    return 20.0 * std::log10(peak / rms);
}

// A sine of any amplitude settles where env = 2 x mean|x| = 4A/pi, so
// G x = pi/4 at the crest and the output peak is tanh(pi/4).
void sineSettlesIndependentlyOfLevel() {
    const auto expected = std::tanh(std::numbers::pi / 4.0);
    for (const auto amplitude : { 0.1, 1.0, 10.0 }) {
        AutoGainSoftClipper stage;
        stage.prepare(rate, {});
        std::vector<float> output(static_cast<std::size_t>(3.0 * rate));
        for (std::size_t index = 0; index < output.size(); ++index)
            output[index] = stage.process(static_cast<float>(sine(amplitude, index)));
        const auto peak = peakOf(output, output.size() - 24'000, output.size());
        require(std::abs(peak - expected) < 0.01,
            "a settled sine must peak at tanh(pi/4) = 0.656 at any level, got "
                + std::to_string(peak) + " for amplitude " + std::to_string(amplitude));
    }
}

// Below 1 / maximumGain the gain is capped, and the stage is a fixed x10 gain
// into a tanh that is still linear.
void quietSignalIsCappedAtMaximumGain() {
    AutoGainSoftClipper stage;
    stage.prepare(rate, {});
    std::vector<float> output(static_cast<std::size_t>(2.0 * rate));
    for (std::size_t index = 0; index < output.size(); ++index)
        output[index] = stage.process(static_cast<float>(sine(0.001, index)));
    const auto peak = peakOf(output, output.size() - 24'000, output.size());
    require(std::abs(peak - std::tanh(0.01)) < 1.0e-4,
        "a quiet signal must see the 10x gain cap, got " + std::to_string(peak));
}

// After a loud second the level is held for one second: a signal ten times
// quieter keeps the loud gain (peak 0.1 pi/4 through a near-linear tanh), then
// recovers once the hold and the 5 Hz release are over.
void holdKeepsTheLoudGainForOneSecond() {
    AutoGainSoftClipper stage;
    stage.prepare(rate, {});
    const auto loud = static_cast<std::size_t>(2.0 * rate);
    std::vector<float> output(static_cast<std::size_t>(4.0 * rate));
    for (std::size_t index = 0; index < output.size(); ++index)
        output[index] = stage.process(static_cast<float>(
            sine(index < loud ? 1.0 : 0.1, index)));
    const auto at = [&](double seconds) {
        return loud + static_cast<std::size_t>(seconds * rate);
    };
    const auto held = peakOf(output, at(0.40), at(0.50));
    require(std::abs(held - std::tanh(0.1 * std::numbers::pi / 4.0)) < 0.004,
        "the hold must keep the loud gain 0.5 s after the drop, got "
            + std::to_string(held));
    const auto recovered = peakOf(output, at(1.70), at(1.80));
    require(std::abs(recovered - std::tanh(std::numbers::pi / 4.0)) < 0.02,
        "the gain must recover after the hold and release, got "
            + std::to_string(recovered));
}

// A pulse train over a quieter tone: the stage drives the pulses deep into the
// tanh, which is what caps the crest factor.
void pulsesAreFlattened() {
    AutoGainSoftClipper stage;
    stage.prepare(rate, {});
    const auto period = static_cast<std::size_t>(0.025 * rate);
    const auto burst = static_cast<std::size_t>(0.0005 * rate);
    std::vector<float> input(static_cast<std::size_t>(4.0 * rate));
    for (std::size_t index = 0; index < input.size(); ++index) {
        const auto phase = index % period;
        const auto pulse = phase < burst
            ? std::sin(std::numbers::pi * static_cast<double>(phase)
                / static_cast<double>(burst))
            : 0.0;
        input[index] = static_cast<float>(sine(0.1, index) + pulse);
    }
    std::vector<float> output(input.size());
    for (std::size_t index = 0; index < input.size(); ++index)
        output[index] = stage.process(input[index]);
    const auto begin = static_cast<std::size_t>(2.0 * rate);
    const auto inputCrest = crestDb(input, begin);
    const auto outputCrest = crestDb(output, begin);
    require(inputCrest > 17.0, "the test signal must be impulsive");
    require(outputCrest < 10.0,
        "the stage must cap the crest factor, got " + std::to_string(outputCrest)
            + " dB from " + std::to_string(inputCrest) + " dB");
}

// One gain for both channels: the louder side drives it, a silent side stays
// exactly silent.
void stereoGainIsLinked() {
    AutoGainSoftClipper stage;
    stage.prepare(rate, {});
    double rightPeak = 0.0, leftPeak = 0.0;
    const auto count = static_cast<std::size_t>(3.0 * rate);
    for (std::size_t index = 0; index < count; ++index) {
        auto left = static_cast<float>(sine(1.0, index));
        auto right = 0.0F;
        stage.process(left, right);
        rightPeak = std::max(rightPeak, std::abs(static_cast<double>(right)));
        if (index > count - 24'000)
            leftPeak = std::max(leftPeak, std::abs(static_cast<double>(left)));
    }
    require(rightPeak == 0.0, "a silent channel must stay silent");
    require(std::abs(leftPeak - std::tanh(std::numbers::pi / 4.0)) < 0.01,
        "the loud channel must settle as a mono sine does");
}

void nonFiniteInputIsSilence() {
    AutoGainSoftClipper stage;
    stage.prepare(rate, {});
    for (int index = 0; index < 1'000; ++index)
        (void) stage.process(static_cast<float>(sine(0.5, static_cast<std::size_t>(index))));
    const auto poisoned = stage.process(std::numeric_limits<float>::quiet_NaN());
    require(poisoned == 0.0F, "a NaN sample must come out as silence");
    require(std::isfinite(stage.gain()), "a NaN sample must not poison the gain");
    const auto after = stage.process(0.25F);
    require(std::isfinite(after) && after > 0.0F, "the stage must keep working after a NaN");
}

} // namespace

int main() {
    try {
        sineSettlesIndependentlyOfLevel();
        quietSignalIsCappedAtMaximumGain();
        holdKeepsTheLoudGainForOneSecond();
        pulsesAreFlattened();
        stereoGainIsLinked();
        nonFiniteInputIsSilence();
    } catch (const std::exception& error) {
        std::cerr << "AutoGainSoftClipper FAIL: " << error.what() << '\n';
        return 1;
    }
    std::cout << "AutoGainSoftClipper: level-independent sine, gain cap, 1 s hold, "
                 "crest cap, linked stereo, NaN guard PASS\n";
    return 0;
}
