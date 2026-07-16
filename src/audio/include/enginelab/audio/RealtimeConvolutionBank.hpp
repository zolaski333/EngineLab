#pragma once

#include <juce_dsp/juce_dsp.h>
#include <array>
#include <atomic>
#include <cstddef>
#include <span>

namespace enginelab {

/** Fixed-path, partitioned FIR bank prepared off the realtime callback. */
class RealtimeConvolutionBank final {
public:
    static constexpr std::size_t maximumPaths = 8;

    void load(std::size_t pathIndex, std::span<const float> monoSamples,
              double sourceSampleRate);
    void load(std::size_t pathIndex, juce::AudioBuffer<float>&& samples,
              double sourceSampleRate);
    void prepare(double sampleRate, int maximumBlockSize, int channels);
    void reset() noexcept;
    void beginBlock(int channels, int sampleCount) noexcept;
    void addInput(std::size_t pathIndex, int channel, int sample, float value) noexcept;
    void process(int sampleCount) noexcept;
    [[nodiscard]] float wetSample(int channel, int sample) const noexcept;
    [[nodiscard]] bool hasImpulseResponse(std::size_t pathIndex) const noexcept;
    [[nodiscard]] bool hasAnyImpulseResponse() const noexcept;

private:
    std::array<juce::dsp::Convolution, maximumPaths> convolvers_;
    std::array<juce::AudioBuffer<float>, maximumPaths> buffers_;
    // JUCE's Convolution accepts asynchronous IR replacement. Keep the small
    // project-side publication flag atomic as it crosses UI/audio threads too.
    std::array<std::atomic<bool>, maximumPaths> loaded_ {};
    int preparedChannels_ { 2 };
    int maximumBlockSize_ { 0 };
};

} // namespace enginelab
