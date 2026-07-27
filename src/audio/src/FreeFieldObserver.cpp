#include <enginelab/audio/FreeFieldObserver.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace enginelab {
namespace {

[[nodiscard]] std::size_t nextPowerOfTwo(std::size_t value) noexcept {
    auto result = std::size_t { 1 };
    while (result < value && result <= std::numeric_limits<std::size_t>::max() / 2U)
        result <<= 1U;
    return result;
}

[[nodiscard]] AcousticPoint3M subtract(
    const AcousticPoint3M& a, const AcousticPoint3M& b) noexcept {
    return { a.x - b.x, a.y - b.y, a.z - b.z };
}

[[nodiscard]] double length(const AcousticPoint3M& value) noexcept {
    return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

[[nodiscard]] AcousticPoint3M normalised(
    const AcousticPoint3M& value, AcousticPoint3M fallback) noexcept {
    const auto magnitude = length(value);
    return std::isfinite(magnitude) && magnitude > 1.0e-9
        ? AcousticPoint3M { value.x / magnitude, value.y / magnitude,
            value.z / magnitude }
        : fallback;
}

[[nodiscard]] double dot(
    const AcousticPoint3M& a, const AcousticPoint3M& b) noexcept {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

} // namespace

bool FreeFieldObserver::prepare(double sampleRateHz, double apertureRadiusM,
    const AcousticPoint3M& sourcePositionM,
    const AcousticPoint3M& sourceAxis,
    AcousticTerminationType termination,
    const AcousticObserverConfig& observer) noexcept {
    prepared_ = false;
    const auto soundSpeedMps = observer.soundSpeedMps > 0.0
        ? observer.soundSpeedMps : 343.0;
    if (!(sampleRateHz > 1'000.0) || !std::isfinite(sampleRateHz)
        || !(apertureRadiusM > 0.0) || !std::isfinite(apertureRadiusM)
        || !(soundSpeedMps > 100.0) || !std::isfinite(soundSpeedMps))
        return false;

    const auto axis = normalised(sourceAxis, { 0.0, 1.0, 0.0 });
    const std::array positions { observer.leftMicrophoneM,
        observer.rightMicrophoneM };
    for (std::size_t index = 0; index < microphones_.size(); ++index) {
        auto& microphone = microphones_[index];
        const auto offset = subtract(positions[index], sourcePositionM);
        microphone.distanceM = length(offset);
        if (!(microphone.distanceM >= 0.05)
            || !std::isfinite(microphone.distanceM))
            return false;
        const auto direction = normalised(offset, axis);
        const auto cosine = std::clamp(dot(axis, direction), -1.0, 1.0);
        // An unflanged opening remains partly visible from behind; a flanged
        // aperture is mounted in an ideal rigid baffle and has no rear high-
        // frequency hemisphere.  The low band remains the physical monopole.
        microphone.highBandDirectivity = static_cast<float>(
            termination == AcousticTerminationType::flanged
                ? std::max(0.0, cosine)
                : std::sqrt(std::max(0.0, 0.5 * (1.0 + cosine))));
        microphone.distanceScale = static_cast<float>(1.0 / microphone.distanceM);
        microphone.delaySamples = static_cast<float>(
            microphone.distanceM / soundSpeedMps * sampleRateHz);
        const auto required = static_cast<std::size_t>(
            std::ceil(microphone.delaySamples)) + 4U;
        const auto size = std::max<std::size_t>(64U, nextPowerOfTwo(required));
        microphone.delay.assign(size, 0.0F);
        microphone.mask = size - 1U;
    }
    // ka=1 marks the transition from an acoustically compact opening to a
    // directional aperture.  A causal one-pole split preserves DC and total
    // pressure on-axis exactly.
    const auto transitionHz = soundSpeedMps
        / (2.0 * std::numbers::pi * apertureRadiusM);
    lowPassCoefficient_ = static_cast<float>(1.0 - std::exp(
        -2.0 * std::numbers::pi * std::min(transitionHz, 0.45 * sampleRateHz)
            / sampleRateHz));
    prepared_ = true;
    reset();
    return true;
}

void FreeFieldObserver::reset() noexcept {
    for (auto& microphone : microphones_) {
        std::fill(microphone.delay.begin(), microphone.delay.end(), 0.0F);
        microphone.write = 0;
        microphone.lowState = 0.0F;
    }
}

StereoPressure FreeFieldObserver::process(float pressureAtOneMetrePa) noexcept {
    StereoPressure result;
    if (!prepared_) return result;
    const auto input = std::isfinite(pressureAtOneMetrePa)
        ? pressureAtOneMetrePa : 0.0F;
    std::array<float, 2> values {};
    for (std::size_t index = 0; index < microphones_.size(); ++index) {
        auto& microphone = microphones_[index];
        microphone.lowState += lowPassCoefficient_
            * (input - microphone.lowState);
        const auto high = input - microphone.lowState;
        const auto propagated = (microphone.lowState
            + microphone.highBandDirectivity * high)
            * microphone.distanceScale;
        const auto delay0 = static_cast<std::size_t>(microphone.delaySamples);
        const auto fraction = microphone.delaySamples - static_cast<float>(delay0);
        const auto read0 = (microphone.write + microphone.delay.size() - delay0)
            & microphone.mask;
        const auto read1 = (microphone.write + microphone.delay.size() - delay0 - 1U)
            & microphone.mask;
        values[index] = std::lerp(
            microphone.delay[read0], microphone.delay[read1], fraction);
        microphone.delay[microphone.write] = propagated;
        microphone.write = (microphone.write + 1U) & microphone.mask;
    }
    result.leftPa = values[0];
    result.rightPa = values[1];
    return result;
}

double FreeFieldObserver::distanceM(std::size_t microphone) const noexcept {
    return microphone < microphones_.size()
        ? microphones_[microphone].distanceM : 0.0;
}

double FreeFieldObserver::directivity(std::size_t microphone) const noexcept {
    return microphone < microphones_.size()
        ? microphones_[microphone].highBandDirectivity : 0.0;
}

} // namespace enginelab
