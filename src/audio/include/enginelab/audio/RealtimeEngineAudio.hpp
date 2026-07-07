#pragma once
#include <enginelab/audio/IAudioRenderer.hpp>
#include <enginelab/runtime/EngineRuntime.hpp>
#include <array>
#include <atomic>
#include <cstdint>
namespace enginelab {
/** Allocation-free layered combustion/exhaust renderer with stereo spatialisation. */
class RealtimeEngineAudio final : public IAudioRenderer {
public:
    RealtimeEngineAudio(FiringEventQueue& queue, RealtimeAudioState& state) : queue_(queue), realtimeState_(state) {}
    void prepare(double sampleRate, int maximumBlockSize) noexcept override;
    void release() noexcept override;
    void render(juce::AudioBuffer<float>& output, int startSample, int sampleCount) noexcept override;
    [[nodiscard]] std::uint64_t lateEventCount() const noexcept { return lateEvents_.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t stolenVoiceCount() const noexcept { return stolenVoices_.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t droppedPendingEventCount() const noexcept { return droppedPendingEvents_.load(std::memory_order_relaxed); }
private:
    struct Voice {
        double bodyPhase {}, crackPhase {};
        float amplitude {}, decay {}, bodyFrequency {}, crackFrequency {}, pan {}, filterState {};
        bool active {};
    };
    struct PendingEvent { FiringEvent event {}; double scheduledTimeSeconds {}; bool exhaust {}; };
    void trigger(const FiringEvent&, bool exhaust) noexcept;
    [[nodiscard]] float noise() noexcept;
    FiringEventQueue& queue_;
    RealtimeAudioState& realtimeState_;
    std::array<Voice, 48> voices_ {};
    std::array<PendingEvent, 512> pendingEvents_ {};
    std::size_t pendingEventCount_ { 0 };
    double sampleRate_ { 48'000.0 };
    double audioTimeSeconds_ { 0.0 };
    double eventLatencySeconds_ { 0.020 };
    std::uint32_t noiseState_ { 0x92d68ca2U };
    std::array<float, 4'096> exhaustDelay_ {};
    std::size_t delayWrite_ { 0 };
    std::size_t reflectionDelaySamples_ { 1'344 };
    float lowPassCoefficient_ { 0.18F };
    float lowPassLeft_ { 0.0F };
    float lowPassRight_ { 0.0F };
    double mechanicalPhase_ { 0.0 };
    double valvetrainPhase_ { 0.0 };
    double starterPhase_ { 0.0 };
    float smoothedRpm_ { 0.0F };
    float intakeFilter_ { 0.0F };
    std::atomic<std::uint64_t> lateEvents_ { 0 };
    std::atomic<std::uint64_t> stolenVoices_ { 0 };
    std::atomic<std::uint64_t> droppedPendingEvents_ { 0 };
};
} // namespace enginelab
