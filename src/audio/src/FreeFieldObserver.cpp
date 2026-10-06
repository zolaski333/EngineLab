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
    sampleRateHz_ = sampleRateHz;
    soundSpeedMps_ = soundSpeedMps;
    sourcePositionM_ = sourcePositionM;
    axis_ = axis;
    termination_ = termination;
    glideCoefficient_ = static_cast<float>(1.0 - std::exp(
        -1.0 / (microphoneGlideSeconds * sampleRateHz)));
    // Place the listener at the scene's listening distance.
    //
    // The scale is taken about the ENGINE ORIGIN and applied to the microphone
    // pair, never about each source. Normalising every outlet to the same
    // distance individually would collapse the geometry between banks: two
    // outlets a metre apart would both read exactly the target distance and the
    // engine would lose its width. Scaling the pair about the origin moves the
    // listener along one ray and leaves every source where the engine put it,
    // so a V12's two banks keep their real path-length difference -- and in fact
    // recover one, since at 24 m that difference was negligible.
    const std::array positions {
        effectiveMicrophonePosition(observer, false),
        effectiveMicrophonePosition(observer, true) };
    for (std::size_t index = 0; index < microphones_.size(); ++index) {
        auto& microphone = microphones_[index];
        if (!aim(microphone, positions[index])) return false;
        microphone.delaySamples = microphone.targetDelaySamples;
        microphone.distanceScale = microphone.targetDistanceScale;
        microphone.highBandDirectivity = microphone.targetDirectivity;
        // Room for a microphone moved as far as moveMicrophones() allows.
        const auto farthest = std::max(microphone.distanceM, maximumMicrophoneDistanceM);
        const auto required = static_cast<std::size_t>(
            std::ceil(farthest / soundSpeedMps * sampleRateHz)) + 4U;
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

bool FreeFieldObserver::aim(Microphone& microphone, const AcousticPoint3M& position) const noexcept {
    const auto offset = subtract(position, sourcePositionM_);
    const auto distanceM = length(offset);
    if (!(distanceM >= 0.05) || !std::isfinite(distanceM)) return false;
    microphone.distanceM = distanceM;
    const auto direction = normalised(offset, axis_);
    const auto cosine = std::clamp(dot(axis_, direction), -1.0, 1.0);
    // An unflanged opening remains partly visible from behind; a flanged
    // aperture is mounted in an ideal rigid baffle and has no rear high-
    // frequency hemisphere.  The low band remains the physical monopole.
    microphone.targetDirectivity = static_cast<float>(
        termination_ == AcousticTerminationType::flanged
            ? std::max(0.0, cosine)
            : std::sqrt(std::max(0.0, 0.5 * (1.0 + cosine))));
    microphone.targetDistanceScale = static_cast<float>(1.0 / distanceM);
    microphone.targetDelaySamples = static_cast<float>(
        distanceM / soundSpeedMps_ * sampleRateHz_);
    return true;
}

void FreeFieldObserver::moveMicrophones(const AcousticPoint3M& left,
                                        const AcousticPoint3M& right, bool immediately) noexcept {
    if (!prepared_) return;
    const std::array positions { left, right };
    for (std::size_t index = 0; index < microphones_.size(); ++index) {
        auto& microphone = microphones_[index];
        auto position = positions[index];
        // Not farther than the delay line reaches, nor inside the opening.
        const auto offset = subtract(position, sourcePositionM_);
        const auto distanceM = length(offset);
        if (!std::isfinite(distanceM)) continue;
        const auto limit = std::min(maximumMicrophoneDistanceM,
            (static_cast<double>(microphone.delay.size()) - 4.0) / sampleRateHz_ * soundSpeedMps_);
        const auto wanted = std::clamp(distanceM, minimumMicrophoneDistanceM, limit);
        if (distanceM > 1.0e-9 && wanted != distanceM) {
            const auto scale = wanted / distanceM;
            position = { sourcePositionM_.x + offset.x * scale, sourcePositionM_.y + offset.y * scale,
                         sourcePositionM_.z + offset.z * scale };
        } else if (!(distanceM > 1.0e-9)) {
            position = { sourcePositionM_.x + axis_.x * minimumMicrophoneDistanceM,
                         sourcePositionM_.y + axis_.y * minimumMicrophoneDistanceM,
                         sourcePositionM_.z + axis_.z * minimumMicrophoneDistanceM };
        }
        if (!aim(microphone, position) || !immediately) continue;
        microphone.delaySamples = microphone.targetDelaySamples;
        microphone.distanceScale = microphone.targetDistanceScale;
        microphone.highBandDirectivity = microphone.targetDirectivity;
    }
}

float FreeFieldObserver::approach(float value, float target, float coefficient,
                                  float minimumStep, float maximumStep) noexcept {
    // An exponential step alone stalls in float once it falls below half an
    // ulp of the value (a 560-sample delay stops 0.3 sample short): the tail
    // moves by at least minimumStep and lands on the target exactly.
    const auto difference = target - value;
    if (!(std::abs(difference) > minimumStep)) return target;
    return value + std::copysign(
        std::clamp(std::abs(coefficient * difference), minimumStep, maximumStep), difference);
}

double FreeFieldObserver::glideDistance(double current, double target, double coefficient) noexcept {
    const auto tail = 1.0e-5 * target;
    const auto difference = target - current;
    if (!(std::abs(difference) > tail)) return target;
    return current + std::copysign(std::max(std::abs(coefficient * difference), tail), difference);
}

void FreeFieldObserver::glide(Microphone& microphone) const noexcept {
    constexpr auto unlimited = std::numeric_limits<float>::max();
    microphone.delaySamples = approach(microphone.delaySamples, microphone.targetDelaySamples,
        glideCoefficient_, minimumDelayRate, maximumDelayRate);
    microphone.distanceScale = approach(microphone.distanceScale, microphone.targetDistanceScale,
        glideCoefficient_, 1.0e-5F * microphone.targetDistanceScale, unlimited);
    microphone.highBandDirectivity = approach(microphone.highBandDirectivity,
        microphone.targetDirectivity, glideCoefficient_, 1.0e-5F, unlimited);
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
        // Only a moved microphone glides: an unmoved one runs the exact
        // arithmetic it always has.
        if (microphone.delaySamples != microphone.targetDelaySamples
            || microphone.distanceScale != microphone.targetDistanceScale
            || microphone.highBandDirectivity != microphone.targetDirectivity)
            glide(microphone);
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
