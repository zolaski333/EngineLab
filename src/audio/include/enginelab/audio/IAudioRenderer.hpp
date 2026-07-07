#pragma once
#include <juce_audio_basics/juce_audio_basics.h>
namespace enginelab {
class IAudioRenderer {
public:
    virtual ~IAudioRenderer() = default;
    virtual void prepare(double sampleRate, int maximumBlockSize) noexcept = 0;
    virtual void release() noexcept = 0;
    virtual void render(juce::AudioBuffer<float>& output, int startSample, int sampleCount) noexcept = 0;
};
} // namespace enginelab
