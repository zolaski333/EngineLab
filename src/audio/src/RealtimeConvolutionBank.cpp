#include <enginelab/audio/RealtimeConvolutionBank.hpp>

#include <algorithm>

namespace enginelab {

void RealtimeConvolutionBank::load(std::size_t pathIndex,
                                   std::span<const float> monoSamples,
                                   double sourceSampleRate) {
    if (pathIndex >= maximumPaths || monoSamples.empty()) return;
    juce::AudioBuffer<float> buffer(1, static_cast<int>(monoSamples.size()));
    std::copy(monoSamples.begin(), monoSamples.end(), buffer.getWritePointer(0));
    load(pathIndex, std::move(buffer), sourceSampleRate);
}

void RealtimeConvolutionBank::load(std::size_t pathIndex,
                                   juce::AudioBuffer<float>&& samples,
                                   double sourceSampleRate) {
    if (pathIndex >= maximumPaths || samples.getNumSamples() <= 0 || sourceSampleRate <= 0.0) return;
    const auto stereo = samples.getNumChannels() > 1
        ? juce::dsp::Convolution::Stereo::yes : juce::dsp::Convolution::Stereo::no;
    convolvers_[pathIndex].loadImpulseResponse(std::move(samples), sourceSampleRate, stereo,
        juce::dsp::Convolution::Trim::no, juce::dsp::Convolution::Normalise::yes);
    loaded_[pathIndex].store(true, std::memory_order_release);
}

void RealtimeConvolutionBank::prepare(double sampleRate, int maximumBlockSize, int channels) {
    preparedChannels_ = std::clamp(channels, 1, 2);
    maximumBlockSize_ = std::max(1, maximumBlockSize);
    const juce::dsp::ProcessSpec spec { sampleRate,
        static_cast<juce::uint32>(maximumBlockSize_),
        static_cast<juce::uint32>(preparedChannels_) };
    for (std::size_t path = 0; path < maximumPaths; ++path) {
        buffers_[path].setSize(preparedChannels_, maximumBlockSize_, false, true, false);
        buffers_[path].clear();
        convolvers_[path].prepare(spec);
    }
}

void RealtimeConvolutionBank::reset() noexcept {
    for (auto& convolver : convolvers_) convolver.reset();
    for (auto& buffer : buffers_) buffer.clear();
}

void RealtimeConvolutionBank::beginBlock(int channels, int sampleCount) noexcept {
    const auto activeChannels = std::min({ channels, preparedChannels_, 2 });
    const auto activeSamples = std::min(sampleCount, maximumBlockSize_);
    for (std::size_t path = 0; path < maximumPaths; ++path)
        if (loaded_[path].load(std::memory_order_acquire))
            for (int channel = 0; channel < activeChannels; ++channel)
                buffers_[path].clear(channel, 0, activeSamples);
}

void RealtimeConvolutionBank::addInput(std::size_t pathIndex, int channel,
                                       int sample, float value) noexcept {
    if (pathIndex >= maximumPaths || !loaded_[pathIndex].load(std::memory_order_acquire)
        || channel < 0 || channel >= preparedChannels_
        || sample < 0 || sample >= maximumBlockSize_) return;
    buffers_[pathIndex].addSample(channel, sample, value);
}

void RealtimeConvolutionBank::process(int sampleCount) noexcept {
    const auto activeSamples = std::min(sampleCount, maximumBlockSize_);
    for (std::size_t path = 0; path < maximumPaths; ++path) {
        if (!loaded_[path].load(std::memory_order_acquire)) continue;
        juce::dsp::AudioBlock<float> fullBlock(buffers_[path]);
        auto block = fullBlock.getSubBlock(0, static_cast<std::size_t>(activeSamples));
        juce::dsp::ProcessContextReplacing<float> context(block);
        convolvers_[path].process(context);
    }
}

float RealtimeConvolutionBank::wetSample(int channel, int sample) const noexcept {
    if (channel < 0 || channel >= preparedChannels_ || sample < 0 || sample >= maximumBlockSize_) return 0.0F;
    float result = 0.0F;
    for (std::size_t path = 0; path < maximumPaths; ++path)
        if (loaded_[path].load(std::memory_order_acquire))
            result += buffers_[path].getSample(channel, sample);
    return result;
}

bool RealtimeConvolutionBank::hasImpulseResponse(std::size_t pathIndex) const noexcept {
    return pathIndex < maximumPaths
        && loaded_[pathIndex].load(std::memory_order_acquire);
}

bool RealtimeConvolutionBank::hasAnyImpulseResponse() const noexcept {
    return std::any_of(loaded_.begin(), loaded_.end(), [](const std::atomic<bool>& value) {
        return value.load(std::memory_order_acquire);
    });
}

} // namespace enginelab
