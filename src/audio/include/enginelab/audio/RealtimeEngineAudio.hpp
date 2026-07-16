#pragma once
#include <enginelab/audio/IAudioRenderer.hpp>
#include <enginelab/audio/RealtimeConvolutionBank.hpp>
#include <enginelab/runtime/EngineRuntime.hpp>
#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <span>
namespace enginelab {
/** Allocation-free layered combustion/exhaust renderer with stereo spatialisation. */
class RealtimeEngineAudio final : public IAudioRenderer {
public:
    RealtimeEngineAudio(FiringEventQueue& queue, RealtimeAudioState& state,
                        CylinderPressureQueue* pressureQueue = nullptr);
    void prepare(double sampleRate, int maximumBlockSize) noexcept override;
    void release() noexcept override;
    void render(juce::AudioBuffer<float>& output, int startSample, int sampleCount) noexcept override;
    void setImpulseResponse(std::span<const float> samples,
                            double sourceSampleRate = 48'000.0,
                            std::size_t pathIndex = 0);
    void setImpulseResponse(juce::AudioBuffer<float>&& samples,
                            double sourceSampleRate,
                            std::size_t pathIndex);
    [[nodiscard]] std::uint64_t lateEventCount() const noexcept { return lateEvents_.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t stolenVoiceCount() const noexcept { return stolenVoices_.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t droppedPendingEventCount() const noexcept { return droppedPendingEvents_.load(std::memory_order_relaxed); }
    [[nodiscard]] double eventLatencySeconds() const noexcept { return eventLatencySeconds_; }
    /** Sample-rate invariant quantisation used by the physical runner lines. */
    [[nodiscard]] static std::size_t runnerDelaySamples(double delaySeconds,
                                                        double sampleRate) noexcept;
private:
    struct Voice {
        double bodyPhase {}, crackPhase {}, pipePhase {}, knockPhase {};
        float amplitude {}, decay {}, ageSeconds {}, attackSeconds {}, blowdownSeconds {};
        float bodyFrequency {}, crackFrequency {}, pipeFrequency {}, knockFrequency {};
        float leftGain {}, rightGain {}, filterState {}, turbulence {}, knock {};
        float massFlow {}, runnerPressure {}, jetBandState {}, jetLowState {}, jetHighState {};
        std::uint32_t exhaustPathIndex {};
        bool exhaust {};
        bool active {};
    };
    struct PendingEvent { FiringEvent event {}; double scheduledTimeSeconds {}; bool exhaust {}; };
    static constexpr std::size_t maximumPaths = RealtimeConvolutionBank::maximumPaths;
    static constexpr std::size_t maxRunners = 32;
    // 80 ms at 192 kHz becomes 320 ms when auditioned at quarter speed.
    // Power-of-two capacities retain the full physical delay across the
    // supported sample-rate and time-scale range.
    static constexpr std::size_t runnerLineLength = 131'072;
    static constexpr std::size_t outletLineLength = 131'072;
    static_assert(runnerLineLength > 192'000 * 80 / 1'000);
    struct ExhaustPathState final {
        float collectorReturn {};
        std::array<float, outletLineLength> forwardWave {};
        std::array<float, outletLineLength> reverseWave {};
        std::array<float, 49'152> fdnA {};
        std::array<float, 65'536> fdnB {};
        std::array<float, 81'920> fdnC {};
        std::array<float, 98'304> fdnD {};
        std::array<float, 64> jitterHistory {};
        std::size_t waveWrite {};
        std::size_t fdnWriteA {};
        std::size_t fdnWriteB {};
        std::size_t fdnWriteC {};
        std::size_t fdnWriteD {};
        std::size_t jitterWrite {};
        std::size_t reflectionDelaySamples { 1 };
        float reflectedLowPass {};
        float collectorState {};
        float previousCollectorInput {};
        float jitterDelaySamples {};
        float exhaustBodyLeft {};
        float exhaustBodyRight {};
        float exhaustAirLeft {};
        float exhaustAirRight {};
    };
    struct RunnerWaveguides final {
        std::array<std::array<float, runnerLineLength>, maxRunners> forward {};
        std::array<std::array<float, runnerLineLength>, maxRunners> backward {};
        std::array<std::size_t, maxRunners> write {};
        std::array<std::size_t, maxRunners> delaySamples {};
    };
    void trigger(const FiringEvent&, bool exhaust) noexcept;
    void updateExhaustPreset(int preset) noexcept;
    [[nodiscard]] std::array<float, maximumPaths> processExhaustWaveguides(
        const std::array<float, maxRunners>& pulse,
        const std::array<std::uint8_t, maxRunners>& pathIndex,
        const std::array<float, maxRunners>& runnerAdmittance,
        const std::array<float, maxRunners>& portReflection,
        const std::array<float, maximumPaths>& outletAdmittance,
        std::size_t count, std::size_t pathCount) noexcept;
    [[nodiscard]] float processMufflerFdn(ExhaustPathState&, float sample) noexcept;
    [[nodiscard]] float saturateCollector(float sample, float drive) const noexcept;
    [[nodiscard]] static float softLimit(float sample) noexcept;
    [[nodiscard]] float noise() noexcept;
    FiringEventQueue& queue_;
    RealtimeAudioState& realtimeState_;
    CylinderPressureQueue* pressureQueue_ { nullptr };
    CylinderPressureSample currentPressureSample_ {};
    CylinderPressureSample nextPressureSample_ {};
    bool hasCurrentPressureSample_ { false };
    bool hasNextPressureSample_ { false };
    std::array<Voice, 96> voices_ {};
    std::array<PendingEvent, 512> pendingEvents_ {};
    std::size_t pendingEventCount_ { 0 };
    std::size_t observedExhaustPathCount_ { 1 };
    double sampleRate_ { 48'000.0 };
    double audioTimeSeconds_ { -0.020 };
    double producerClock_ { 0.0 };
    double eventLatencySeconds_ { 0.020 };
    std::uint32_t noiseState_ { 0x92d68ca2U };
    // Per-runner bidirectional digital waveguide (port <-> collector) with an
    // N-port scattering junction that couples cylinders sharing a collector.
    std::unique_ptr<RunnerWaveguides> runners_;
    // Path delay networks are large by design (80 ms at 192 kHz). Keep them
    // off the UI/test stack while retaining fixed, allocation-free callback state.
    std::unique_ptr<ExhaustPathState[]> exhaustPaths_;
    std::array<std::size_t, 4> fdnDelaySamples_ { 1, 1, 1, 1 };
    RealtimeConvolutionBank convolutionBank_;
    std::unique_ptr<juce::dsp::Oversampling<float>> oversampler_;
    int maximumBlockSize_ { 1 };
    int activeExhaustPreset_ { -1 };
    float presetDrive_ { 1.0F };
    float presetDamping_ { 0.985F };
    float presetWet_ { 0.42F };
    float presetToneHz_ { 1'500.0F };
    float presetReflection_ { 0.34F };
    float presetFdngain_ { 0.58F };
    float lowPassCoefficient_ { 0.18F };
    float voiceFilterCoefficient_ { 0.24F };
    float intakeFilterCoefficient_ { 0.08F };
    float rpmFilterCoefficient_ { 0.00037F };
    float reflectionFilterCoefficient_ { 0.12F };
    float lowPassLeft_ { 0.0F };
    float lowPassRight_ { 0.0F };
    float pressureTailDecay_ { 0.992F };
    float pressureTailInputCoefficient_ { 0.010F };
    float bovDecay_ { 0.9994F };
    float bovNoiseCoefficient_ { 0.08F };
    float jitterCoefficient_ { 0.015F };
    float collectorCoefficient_ { 0.18F };
    float levelAttackCoefficient_ { 0.0025F };
    float levelReleaseCoefficient_ { 0.00012F };
    float gainAttackCoefficient_ { 0.0020F };
    float gainReleaseCoefficient_ { 0.00012F };
    float bodyExcitationScale_ { 1.0F };
    float levelEnvelope_ { 0.0F };
    float levelGain_ { 1.0F };
    float antiAliasLeftA_ { 0.0F };
    float antiAliasLeftB_ { 0.0F };
    float antiAliasRightA_ { 0.0F };
    float antiAliasRightB_ { 0.0F };
    float antiAliasCoefficient_ { 0.5F };
    float dcBlockPole_ { 0.998F };
    float toneCoefficient_ { 0.1F };
    float dcInputLeft_ { 0.0F };
    float dcInputRight_ { 0.0F };
    float dcOutputLeft_ { 0.0F };
    float dcOutputRight_ { 0.0F };
    float toneLowLeft_ { 0.0F };
    float toneLowRight_ { 0.0F };
    float pressureTailLeft_ { 0.0F };
    float pressureTailRight_ { 0.0F };
    std::array<float, 32> cylinderPressureRawPrevious_ {};
    std::array<float, 32> cylinderPressureHighPass_ {};
    std::array<float, 32> cylinderPressureHighPassPrevious_ {};
    std::array<float, 32> cylinderPressureBandLimited_ {};
    std::array<float, 32> exhaustPressureRawPrevious_ {};
    std::array<float, 32> exhaustPressureHighPass_ {};
    std::array<float, 32> exhaustPressureHighPassPrevious_ {};
    std::array<float, 32> exhaustPressureBandLimited_ {};
    float pressureHighPassPole_ { 0.997F };
    float pressureBandCoefficient_ { 0.5F };
    double pressureSampleIntervalSeconds_ { 0.0 };
    double mechanicalPhase_ { 0.0 };
    double valvetrainPhase_ { 0.0 };
    double starterPhase_ { 0.0 };
    double fiWhistlePhase_ { 0.0 };
    float smoothedRpm_ { 0.0F };
    float intakeFilter_ { 0.0F };
    float intakeSvfLow_ { 0.0F };
    float intakeSvfBand_ { 0.0F };
    float bovEnvelope_ { 0.0F };
    float bovNoiseState_ { 0.0F };
    float previousThrottleForBov_ { 0.0F };
    std::atomic<std::uint64_t> lateEvents_ { 0 };
    std::atomic<std::uint64_t> stolenVoices_ { 0 };
    std::atomic<std::uint64_t> droppedPendingEvents_ { 0 };
};
} // namespace enginelab
