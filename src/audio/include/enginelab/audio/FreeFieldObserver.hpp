#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

#include <array>
#include <cstddef>
#include <vector>

namespace enginelab {

struct StereoPressure final {
    float leftPa {};
    float rightPa {};
};

/** Causal free-field propagation from one circular opening to two microphones.
 *
 * Input pressure is the opening's monopole far field at one metre.  Each
 * microphone receives its own propagation delay and spherical 1/r decay.  The
 * ka transition is represented by the exact low-frequency monopole plus a
 * first-order high band carrying the termination directivity; this avoids a
 * non-causal, sample-by-sample frequency-domain approximation.
 */
class FreeFieldObserver final {
public:
    [[nodiscard]] bool prepare(double sampleRateHz, double apertureRadiusM,
        const AcousticPoint3M& sourcePositionM,
        const AcousticPoint3M& sourceAxis,
        AcousticTerminationType termination,
        const AcousticObserverConfig& observer) noexcept;
    void reset() noexcept;
    [[nodiscard]] StereoPressure process(float pressureAtOneMetrePa) noexcept;

    [[nodiscard]] double distanceM(std::size_t microphone) const noexcept;
    [[nodiscard]] double directivity(std::size_t microphone) const noexcept;

private:
    struct Microphone final {
        std::vector<float> delay;
        std::size_t write {};
        std::size_t mask {};
        float delaySamples { 1.0F };
        float distanceScale { 1.0F };
        float highBandDirectivity { 1.0F };
        float lowState {};
        double distanceM { 1.0 };
    };

    std::array<Microphone, 2> microphones_;
    float lowPassCoefficient_ {};
    bool prepared_ { false };
};

} // namespace enginelab
