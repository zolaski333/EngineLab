#include <enginelab/audio/IntegerUpsampler.hpp>
#include <enginelab/audio/RealtimeEngineAudio.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <numbers>
#include <string_view>
#include <vector>

namespace {

void require(bool condition, std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

/** Amplitude of the component at `hz` (Goertzel over whole cycles). */
double amplitudeAt(const std::vector<float>& signal, std::size_t begin,
                   double hz, double sampleRate) {
    const auto omega = 2.0 * std::numbers::pi * hz / sampleRate;
    const auto coefficient = 2.0 * std::cos(omega);
    auto previous = 0.0;
    auto beforePrevious = 0.0;
    for (auto index = begin; index < signal.size(); ++index) {
        const auto current = signal[index] + coefficient * previous - beforePrevious;
        beforePrevious = previous;
        previous = current;
    }
    const auto power = previous * previous + beforePrevious * beforePrevious
        - coefficient * previous * beforePrevious;
    return 2.0 * std::sqrt(std::max(0.0, power))
        / static_cast<double>(signal.size() - begin);
}

double rms(const juce::AudioBuffer<float>& buffer) {
    auto sum = 0.0;
    for (int sample = 0; sample < buffer.getNumSamples(); ++sample)
        sum += static_cast<double>(buffer.getSample(0, sample)) * buffer.getSample(0, sample);
    return std::sqrt(sum / buffer.getNumSamples());
}

struct Rendered {
    double synthesisRate { 0.0 };
    int factor { 0 };
    double rms { 0.0 };
    bool finite { true };
};

/** One second of the mechanical layer at 3,000 rpm through the renderer, in
 *  device blocks of `block` samples (not a multiple of the factor). */
Rendered renderSecond(double deviceRate, double referenceRate, int block) {
    auto queue = std::make_unique<enginelab::FiringEventQueue>();
    enginelab::RealtimeAudioState state;
    state.rpm.store(3'000.0F);
    state.timeScale.store(1.0F);
    state.mechanicalGain.store(0.70F);
    auto renderer = std::make_unique<enginelab::RealtimeEngineAudio>(
        *queue, state, nullptr, nullptr, nullptr, nullptr);
    renderer->setReferenceSynthesisRate(referenceRate);
    renderer->prepare(deviceRate, block);
    const auto total = static_cast<int>(deviceRate);
    juce::AudioBuffer<float> output(2, total);
    for (int offset = 0; offset < total; offset += block)
        renderer->render(output, offset, std::min(block, total - offset));
    Rendered result { renderer->synthesisSampleRate(), renderer->upsamplingFactor(),
                      rms(output), true };
    for (int channel = 0; channel < 2; ++channel)
        for (int sample = 0; sample < total; ++sample)
            if (!std::isfinite(output.getSample(channel, sample))) result.finite = false;
    return result;
}

} // namespace

int main() {
    // The interpolator: a tone in band keeps its amplitude, its image is gone.
    for (const auto factor : { 2, 3, 4 }) {
        enginelab::IntegerUpsampler upsampler;
        upsampler.prepare(factor);
        constexpr double inputRate = 48'000.0;
        constexpr double toneHz = 1'000.0;
        std::vector<float> input(4'800);
        for (std::size_t index = 0; index < input.size(); ++index)
            input[index] = static_cast<float>(0.5 * std::sin(
                2.0 * std::numbers::pi * toneHz * static_cast<double>(index) / inputRate));
        std::vector<float> output(input.size() * static_cast<std::size_t>(factor));
        upsampler.process(input.data(), static_cast<int>(input.size()), output.data());
        const auto outputRate = inputRate * factor;
        // Past the filter's settling (its length), over whole periods.
        const auto begin = static_cast<std::size_t>(
            enginelab::IntegerUpsampler::tapsPerPhase * factor);
        const auto tone = amplitudeAt(output, begin, toneHz, outputRate);
        const auto image = amplitudeAt(output, begin, inputRate - toneHz, outputRate);
        std::cout << "x" << factor << ": tone " << tone << ", image at 47 kHz "
                  << 20.0 * std::log10(std::max(1.0e-12, image / tone)) << " dB\n";
        require(std::abs(tone - 0.5) < 0.005,
                "an upsampled tone in band must keep its amplitude within 0.1 dB");
        require(image < tone * 1.0e-3,
                "the upsampler must reject the first image by 60 dB");
    }

    // The renderer: the whole fraction of the device rate that stays at least
    // 48 kHz, and nothing changes at or below it.
    const struct { double device; double synthesis; int factor; } policy[] {
        { 44'100.0, 44'100.0, 1 }, { 48'000.0, 48'000.0, 1 },
        { 96'000.0, 48'000.0, 2 }, { 176'400.0, 58'800.0, 3 },
        { 192'000.0, 48'000.0, 4 },
    };
    for (const auto& expected : policy) {
        auto queue = std::make_unique<enginelab::FiringEventQueue>();
        enginelab::RealtimeAudioState state;
        auto renderer = std::make_unique<enginelab::RealtimeEngineAudio>(
            *queue, state, nullptr, nullptr, nullptr, nullptr);
        renderer->setReferenceSynthesisRate(enginelab::RealtimeEngineAudio::voicedSampleRateHz);
        renderer->prepare(expected.device, 512);
        require(renderer->upsamplingFactor() == expected.factor
                    && std::abs(renderer->synthesisSampleRate() - expected.synthesis) < 1.0e-6,
                "the synthesis rate must be the whole fraction of the device rate still >= 48 kHz");
    }

    // Same sound: 192 kHz upsampled from 48 kHz holds the 48 kHz level, in
    // device blocks that do not divide by the factor.
    const auto voiced = renderSecond(48'000.0, enginelab::RealtimeEngineAudio::voicedSampleRateHz, 480);
    const auto upsampled = renderSecond(192'000.0, enginelab::RealtimeEngineAudio::voicedSampleRateHz, 1'917);
    const auto native = renderSecond(192'000.0, 0.0, 1'917);
    const auto levelDb = 20.0 * std::log10(upsampled.rms / voiced.rms);
    std::cout << "rms 48 kHz " << voiced.rms << ", 192 kHz upsampled " << upsampled.rms
              << " (" << levelDb << " dB), 192 kHz native " << native.rms << '\n';
    require(voiced.factor == 1 && upsampled.factor == 4 && native.factor == 1,
            "only the reference-rate renderer may upsample");
    require(voiced.rms > 1.0e-5 && upsampled.finite && voiced.finite,
            "the mechanical layer must be audible and finite");
    require(std::abs(levelDb) < 0.5,
            "a 192 kHz device must hear the 48 kHz voicing at its level");
    std::cout << "audio upsampling: ok\n";
    return EXIT_SUCCESS;
}
