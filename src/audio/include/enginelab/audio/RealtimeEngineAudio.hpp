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
                        CylinderPressureQueue* pressureQueue = nullptr)
        : queue_(queue), realtimeState_(state), pressureQueue_(pressureQueue) {}
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
    void trigger(const FiringEvent&, bool exhaust) noexcept;
    void updateExhaustPreset(int preset) noexcept;
    [[nodiscard]] float processExhaustWaveguide(const std::array<float, 32>& pulse, std::size_t count) noexcept;
    [[nodiscard]] float processMufflerFdn(float sample) noexcept;
    [[nodiscard]] float saturateCollector(float sample, float drive) const noexcept;
    [[nodiscard]] static float softLimit(float sample) noexcept;
    [[nodiscard]] float noise() noexcept;
    FiringEventQueue& queue_;
    RealtimeAudioState& realtimeState_;
    CylinderPressureQueue* pressureQueue_ { nullptr };
    CylinderPressureSample previousPressureSample_ {};
    CylinderPressureSample currentPressureSample_ {};
    CylinderPressureSample nextPressureSample_ {};
    bool hasPreviousPressureSample_ { false };
    bool hasCurrentPressureSample_ { false };
    bool hasNextPressureSample_ { false };
    std::array<Voice, 48> voices_ {};
    std::array<PendingEvent, 512> pendingEvents_ {};
    std::size_t pendingEventCount_ { 0 };
    double sampleRate_ { 48'000.0 };
    double audioTimeSeconds_ { 0.0 };
    double producerClock_ { 0.0 };
    double eventLatencySeconds_ { 0.020 };
    std::uint32_t noiseState_ { 0x92d68ca2U };
    static constexpr std::size_t maxRunners = 32;
    static constexpr std::size_t runnerLineLength = 256;
    // Per-runner bidirectional digital waveguide (port <-> collector) with an
    // N-port scattering junction that couples cylinders sharing a collector.
    std::array<std::array<float, runnerLineLength>, maxRunners> runnerForward_ {};
    std::array<std::array<float, runnerLineLength>, maxRunners> runnerBackward_ {};
    std::array<std::size_t, maxRunners> runnerWrite_ {};
    std::array<std::size_t, maxRunners> runnerDelaySamples_ {};
    float collectorReturn_ { 0.0F };
    std::array<float, 4'096> forwardWave_ {};
    std::array<float, 4'096> reverseWave_ {};
    std::array<float, 1'493> fdnA_ {};
    std::array<float, 2'111> fdnB_ {};
    std::array<float, 2'791> fdnC_ {};
    std::array<float, 3'557> fdnD_ {};
    RealtimeConvolutionBank convolutionBank_;
    std::unique_ptr<juce::dsp::Oversampling<float>> oversampler_;
    int oversamplerLatency_ { 0 };
    std::array<float, 64> jitterHistory_ {};
    std::size_t jitterWrite_ { 0 };
    std::size_t waveWrite_ { 0 };
    std::size_t fdnWriteA_ { 0 };
    std::size_t fdnWriteB_ { 0 };
    std::size_t fdnWriteC_ { 0 };
    std::size_t fdnWriteD_ { 0 };
    std::size_t reflectionDelaySamples_ { 1'344 };
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
    float reflectedLowPass_ { 0.0F };
    float collectorState_ { 0.0F };
    float previousCollectorInput_ { 0.0F };
    float jitterDelaySamples_ { 0.0F };
    float levelEnvelope_ { 0.0F };
    float levelGain_ { 1.0F };
    float antiAliasLeftA_ { 0.0F };
    float antiAliasLeftB_ { 0.0F };
    float antiAliasRightA_ { 0.0F };
    float antiAliasRightB_ { 0.0F };
    float antiAliasCoefficient_ { 0.5F };
    float pressureTailLeft_ { 0.0F };
    float pressureTailRight_ { 0.0F };
    float pressureRawPrevious_ { 0.0F };
    float pressureHighPass_ { 0.0F };
    float pressureHighPassPrevious_ { 0.0F };
    float pressureBandLimited_ { 0.0F };
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
    float exhaustBodyLeft_ { 0.0F };
    float exhaustBodyRight_ { 0.0F };
    float exhaustAirLeft_ { 0.0F };
    float exhaustAirRight_ { 0.0F };
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
