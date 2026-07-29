#include <enginelab/audio/ImpulseResponseLoader.hpp>

#include <algorithm>
#include <cmath>
#include <memory>

namespace enginelab {

ImpulseResponseLoadResult loadImpulseResponseFile(
    const juce::File& file, juce::int64 maximumSamples) {
    ImpulseResponseLoadResult result;
    if (!file.existsAsFile()) {
        result.error = ImpulseResponseLoadError::missingFile;
        return result;
    }

    juce::AudioFormatManager formats;
    formats.registerBasicFormats();
    std::unique_ptr<juce::AudioFormatReader> reader(
        formats.createReaderFor(file));
    if (!reader) {
        result.error = ImpulseResponseLoadError::unsupportedOrCorrupt;
        return result;
    }
    if (!(reader->sampleRate > 0.0) || !std::isfinite(reader->sampleRate)
        || reader->numChannels == 0) {
        result.error = ImpulseResponseLoadError::invalidMetadata;
        return result;
    }

    const auto safeMaximum = std::max<juce::int64>(1, maximumSamples);
    const auto count64 = std::min(reader->lengthInSamples, safeMaximum);
    if (count64 <= 0) {
        result.error = ImpulseResponseLoadError::empty;
        return result;
    }

    const auto count = static_cast<int>(count64);
    result.samples.setSize(
        std::clamp(static_cast<int>(reader->numChannels), 1, 2),
        count, false, true, false);
    if (!reader->read(&result.samples, 0, count, 0, true, true)) {
        result.samples.setSize(0, 0);
        result.error = ImpulseResponseLoadError::readFailure;
        return result;
    }
    result.sampleRateHz = reader->sampleRate;
    result.truncated = reader->lengthInSamples > count64;
    return result;
}

} // namespace enginelab
