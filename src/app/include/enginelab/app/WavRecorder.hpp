#pragma once

#include <juce_audio_formats/juce_audio_formats.h>

#include <atomic>
#include <memory>

namespace enginelab {

/**
 * What the speakers play, to a 24-bit WAV file: the audio thread hands each
 * block over (write) and a background thread writes it. Starting and stopping
 * take a short lock the audio thread also takes, once a block, only to find
 * the writer.
 */
class WavRecorder final {
public:
    WavRecorder();
    ~WavRecorder();

    /** Starts a new file (a running one is closed first); false when it
     *  cannot be created. Message thread. */
    bool start(const juce::File& file, double sampleRateHz, int channels);
    /** Closes the file: what was handed over is written. Message thread. */
    void stop();
    [[nodiscard]] bool recording() const noexcept { return recording_.load(std::memory_order_relaxed); }
    [[nodiscard]] const juce::File& file() const noexcept { return file_; }
    /** Samples handed over to this file so far. */
    [[nodiscard]] juce::int64 samples() const noexcept { return samples_.load(std::memory_order_relaxed); }
    /** Audio thread: one block of the output. */
    void write(const juce::AudioBuffer<float>& buffer, int startSample, int sampleCount) noexcept;

private:
    juce::TimeSliceThread thread_ { "EngineLab WAV recorder" };
    std::unique_ptr<juce::AudioFormatWriter::ThreadedWriter> writer_;
    juce::CriticalSection writerLock_;
    juce::AudioFormatWriter::ThreadedWriter* active_ {};
    juce::File file_;
    int channels_ { 2 };
    std::atomic<bool> recording_ { false };
    std::atomic<juce::int64> samples_ { 0 };
};

} // namespace enginelab
