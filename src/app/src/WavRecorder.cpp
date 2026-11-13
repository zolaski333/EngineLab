#include <enginelab/app/WavRecorder.hpp>

#include <algorithm>

namespace enginelab {

WavRecorder::WavRecorder() { thread_.startThread(); }

WavRecorder::~WavRecorder() {
    stop();
    thread_.stopThread(2'000);
}

bool WavRecorder::start(const juce::File& file, double sampleRateHz, int channels) {
    stop();
    if (!(sampleRateHz > 0.0) || channels < 1) return false;
    (void)file.getParentDirectory().createDirectory();
    (void)file.deleteFile();
    auto stream = std::make_unique<juce::FileOutputStream>(file);
    if (!stream->openedOk()) return false;
    std::unique_ptr<juce::OutputStream> output = std::move(stream);
    juce::WavAudioFormat format;
    auto writer = format.createWriterFor(output, juce::AudioFormatWriterOptions {}
        .withSampleRate(sampleRateHz)
        .withNumChannels(channels)
        .withBitsPerSample(24));
    if (writer == nullptr) return false;
    // Half a minute of audio queued: the disk may stall that long.
    writer_ = std::make_unique<juce::AudioFormatWriter::ThreadedWriter>(writer.release(), thread_,
        static_cast<int>(sampleRateHz * 30.0));
    file_ = file;
    channels_ = channels;
    samples_.store(0, std::memory_order_relaxed);
    {
        const juce::ScopedLock lock(writerLock_);
        active_ = writer_.get();
    }
    recording_.store(true, std::memory_order_relaxed);
    return true;
}

void WavRecorder::stop() {
    {
        const juce::ScopedLock lock(writerLock_);
        active_ = nullptr;
    }
    recording_.store(false, std::memory_order_relaxed);
    // Deleting it flushes what is queued and closes the file.
    writer_.reset();
}

void WavRecorder::write(const juce::AudioBuffer<float>& buffer, int startSample, int sampleCount) noexcept {
    if (sampleCount <= 0) return;
    const juce::ScopedLock lock(writerLock_);
    if (active_ == nullptr) return;
    const float* channels[8] {};
    const auto available = std::min(buffer.getNumChannels(), 8);
    if (available <= 0) return;
    for (int channel = 0; channel < channels_ && channel < 8; ++channel)
        channels[channel] = buffer.getReadPointer(std::min(channel, available - 1), startSample);
    if (active_->write(channels, sampleCount)) samples_.fetch_add(sampleCount, std::memory_order_relaxed);
}

} // namespace enginelab
