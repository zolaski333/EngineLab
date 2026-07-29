#pragma once

#include <juce_audio_formats/juce_audio_formats.h>
#include <cstdint>

namespace enginelab {

enum class ImpulseResponseLoadError {
    none,
    missingFile,
    unsupportedOrCorrupt,
    invalidMetadata,
    empty,
    readFailure
};

/** Result of decoding a user-authored downstream acoustic measurement.
 *
 * This helper is deliberately outside the realtime renderer: filesystem and
 * codec work must finish on the UI/tool thread before the decoded buffer is
 * handed to RealtimeConvolutionBank.
 */
struct ImpulseResponseLoadResult final {
    juce::AudioBuffer<float> samples;
    double sampleRateHz { 0.0 };
    ImpulseResponseLoadError error { ImpulseResponseLoadError::none };
    bool truncated { false };

    [[nodiscard]] bool ok() const noexcept {
        return error == ImpulseResponseLoadError::none
            && sampleRateHz > 0.0 && samples.getNumSamples() > 0;
    }
};

[[nodiscard]] ImpulseResponseLoadResult loadImpulseResponseFile(
    const juce::File& file, juce::int64 maximumSamples = 262'144);

} // namespace enginelab
