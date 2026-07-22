#pragma once
#include <enginelab/audio/AcousticExhaustNetwork.hpp>
#include <enginelab/audio/AcousticIntakeNetwork.hpp>
#include <enginelab/audio/ForcedInductionAcoustics.hpp>
#include <enginelab/audio/IAudioRenderer.hpp>
#include <enginelab/audio/PipeRadiationModel.hpp>
#include <enginelab/audio/BoundaryReconstructionFilter.hpp>
#include <enginelab/audio/DuctWallLoss.hpp>
#include <enginelab/audio/ExpansionChamberMuffler.hpp>
#include <enginelab/audio/RealtimeConvolutionBank.hpp>
#include <enginelab/audio/StructuralModalRadiator.hpp>
#include <enginelab/audio/ValveFlowAcousticSource.hpp>
#include <enginelab/audio/ValvePortTermination.hpp>
#include <enginelab/runtime/EngineRuntime.hpp>
#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>
namespace enginelab {
/** Allocation-free engine renderer with an SI-unit thermoacoustic exhaust path.
 *
 * When CylinderPressureSample publishes a valid thermoacoustic boundary, the
 * exhaust is a passive characteristic network terminated by a radiation load.
 * The historical procedural exhaust remains available only for old producers
 * that never publish that boundary; the two paths are never mixed.
 */
class RealtimeEngineAudio final : public IAudioRenderer {
public:
    RealtimeEngineAudio(FiringEventQueue& queue, RealtimeAudioState& state,
                        CylinderPressureQueue* pressureQueue = nullptr,
                        const ExhaustGraph* exhaustGraph = nullptr,
                        const EngineConfig* engineConfig = nullptr);
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
    /** Times a requested delay exceeded its allocated line and was clamped.
     *
     * Lines are sized in prepare() from the delays the runtime published, so a
     * non-zero count means a delay grew afterwards and the acoustic geometry is
     * silently shorter than configured. Non-zero is a defect, not a warning.
     */
    [[nodiscard]] std::uint64_t delayTruncationCount() const noexcept { return delayTruncations_.load(std::memory_order_relaxed); }
    [[nodiscard]] double eventLatencySeconds() const noexcept { return eventLatencySeconds_; }
    // Read-only observers of the safety leveler, so a harness can prove whether the
    // slow AGC actually engages in a given voice (gain < 1) or stays at identity
    // (safety-only). These do not affect the audio path.
    [[nodiscard]] std::uint64_t levelLimitedSampleCount() const noexcept { return levelLimitedSamples_.load(std::memory_order_relaxed); }
    [[nodiscard]] float minObservedLevelGain() const noexcept { return minObservedLevelGain_.load(std::memory_order_relaxed); }
    /** Whether the physical thermoacoustic path has taken ownership of the voice.
     *
     * False means every rendered sample is coming from the legacy procedural
     * exhaust instead. That is a silent downgrade in audible quality, so it must
     * be observable rather than inferred: a caller that cannot distinguish the
     * two paths cannot tell a physical-path improvement from a physical path
     * that never activated. */
    [[nodiscard]] bool physicalExhaustActive() const noexcept {
        return physicalExhaustActive_.load(std::memory_order_relaxed);
    }
    /** True only when the complete ExhaustGraph, rather than the compatibility
     * runner/path reduction, was compiled successfully. */
    [[nodiscard]] bool compiledExhaustTopologyActive() const noexcept {
        return acousticExhaustNetwork_ != nullptr;
    }
    [[nodiscard]] bool structuralRadiationActive() const noexcept {
        return structuralModalRadiator_ != nullptr;
    }
    [[nodiscard]] bool compiledIntakeTopologyActive() const noexcept {
        return acousticIntakeNetwork_ != nullptr;
    }
    [[nodiscard]] bool forcedInductionAcousticsActive() const noexcept {
        return forcedInductionAcoustics_ != nullptr;
    }
    /** Samples rendered on the legacy procedural path.
     *
     * Non-zero after start-up means the physical boundary was unavailable or
     * rejected and the renderer fell back. Start-up itself contributes a bounded
     * number before the first valid telemetry arrives. */
    [[nodiscard]] std::uint64_t legacyPathSampleCount() const noexcept {
        return legacyPathSamples_.load(std::memory_order_relaxed);
    }
    /** Telemetry samples whose thermoacoustic boundary was published invalid. */
    [[nodiscard]] std::uint64_t invalidBoundarySampleCount() const noexcept {
        return invalidBoundarySamples_.load(std::memory_order_relaxed);
    }
    /** Peak SI pressure delivered by the free-field exhaust observer. */
    [[nodiscard]] float maxObservedExhaustPressurePa() const noexcept {
        return maxObservedExhaustPressurePa_.load(std::memory_order_relaxed);
    }
    [[nodiscard]] float maxObservedIntakePressurePa() const noexcept {
        return maxObservedIntakePressurePa_.load(std::memory_order_relaxed);
    }
    [[nodiscard]] float maxObservedStructuralPressurePa() const noexcept {
        return maxObservedStructuralPressurePa_.load(std::memory_order_relaxed);
    }
    [[nodiscard]] float maxPreLimiterMagnitude() const noexcept {
        return maxPreLimiterMagnitude_.load(std::memory_order_relaxed);
    }
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
    // Compile-time upper bounds. These size only small per-block scratch arrays
    // and the published-telemetry views; the delay lines below are sized at
    // prepare() from the engine actually loaded.
    static constexpr std::size_t maximumPaths = RealtimeConvolutionBank::maximumPaths;
    static constexpr std::size_t maxRunners = 32;

    // Every physical delay is stretched at render time by
    //   acousticDelayScale = referenceSoundSpeed / exhaustSoundSpeed / timeScale
    // whose render-time clamps bound it at 900 / 289.8 / 0.25 = 12.42: a cold
    // exhaust auditioned at quarter speed. Lines are sized for this worst case,
    // so temperature and time-scale changes never reach the clamp.
    static constexpr double maximumAcousticDelayScale = 12.5;
    // Ceiling EngineRuntime applies to every published physical delay. It is a
    // safety clamp, not a physical bound: 80 ms of primary would be a 41 m pipe.
    // Sizing lines from it would waste ~87x, so prepare() sizes from the delays
    // actually published and render() reports any residual truncation.
    static constexpr double maximumPublishedDelaySeconds = 0.080;
    static constexpr std::size_t minimumLineLength = 64;
    // Reference delays of the muffler FDN's four lines, in samples at 48 kHz
    // before the acoustic scale is applied. Mutually prime so their modes do
    // not coincide.
    static constexpr std::array<double, 4> referenceFdnSamples {
        1'493.0, 2'111.0, 2'791.0, 3'557.0 };

    /** A bank of `count` power-of-two delay lines held contiguously.
     *
     * One allocation per bank keeps a runner's read and write cursors on the
     * same pages, and the power-of-two stride turns the per-sample wrap into a
     * mask instead of an integer division.
     */
    struct DelayLineBank final {
        std::vector<float> data;
        std::size_t stride { 0 };
        std::size_t mask { 0 };
        void allocate(std::size_t lines, std::size_t lengthPowerOfTwo) {
            stride = lengthPowerOfTwo;
            mask = lengthPowerOfTwo - 1;
            data.assign(lines * lengthPowerOfTwo, 0.0F);
        }
        void clear() noexcept { std::fill(data.begin(), data.end(), 0.0F); }
        [[nodiscard]] float* line(std::size_t index) noexcept {
            return data.data() + index * stride;
        }
    };

    struct ExhaustPathState final {
        UnflangedPipeRadiation radiation;
        /** Characteristic admittance A/(rho*c), in m^3/(Pa*s). */
        float outletAcousticAdmittance {};
        float collectorReturn {};
        std::vector<float> forwardWave;
        std::vector<float> reverseWave;
        /** Free-field propagation from the mouth to the calibrated observer. */
        std::vector<float> observerPressure;
        std::array<std::vector<float>, 4> fdn {};
        std::array<float, 64> jitterHistory {};
        std::size_t waveWrite {};
        std::size_t waveMask {};
        std::size_t observerWrite {};
        std::size_t observerMask {};
        float observerDelaySamples { 1.0F };
        std::array<std::size_t, 4> fdnWrite {};
        std::size_t jitterWrite {};
        float reflectionDelaySamples { 1.0F };
        /** Block-rate target reflectionDelaySamples ramps toward per sample. */
        float reflectionDelayTargetSamples { 1.0F };
        float reflectedLowPass {};
        float collectorState {};
        float previousCollectorInput {};
        float jitterDelaySamples {};
        float exhaustBodyLeft {};
        float exhaustBodyRight {};
        float exhaustAirLeft {};
        float exhaustAirRight {};
        /** Thermoviscous attenuation over one collector-to-outlet traversal,
         *  applied independently in each direction. Refitted per block from the
         *  path's own gas state; see DuctWallLoss. */
        DuctWallLoss::Coefficients wallLoss {};
        DuctWallLoss::State wallLossOutbound {};
        DuctWallLoss::State wallLossReturn {};
        /** Path-average gas state, cached from the last physical sample so the
         *  per-block wall-loss fit does not need per-sample transcendentals. */
        float mediumDensityKgPerM3 { 1.2F };
        float mediumSoundSpeedMps { 343.0F };
        /** Expansion-chamber silencer at the collector end of the duct.
         *  Disabled until the engine publishes a chamber, in which state it is
         *  an exact through-connection. See ExpansionChamberMuffler. */
        ExpansionChamberMuffler::State muffler {};
        ExpansionChamberMuffler::Coefficients mufflerCoefficients {};
        /** Block-rate target the chamber traversal ramps toward per sample.
         *
         * The chamber delay is stretched by the published exhaust sound speed
         * exactly like every other physical delay, so it inherits the same
         * frame-rate step; see RunnerWaveguides::delayTargetSamples. Applying
         * it directly was measured as broadband hash reaching 28 % of the
         * signal energy in the top octave -- block-boundary discontinuities,
         * not silencing. */
        float mufflerDelayTargetSamples { 0.0F };
    };
    struct RunnerWaveguides final {
        DelayLineBank forward;
        DelayLineBank backward;
        std::array<std::size_t, maxRunners> write {};
        std::array<float, maxRunners> delaySamples {};
        /** Block-rate targets delaySamples ramps toward per sample.
         *
         * Every delay in the network scales with the published exhaust sound
         * speed, which is frame-rate telemetry: it aliases the engine cycle's
         * temperature swing into a step at every frame. Applying such a step
         * directly phase-jumps all delay lines at once, which was measured as
         * an engine-independent comb at exact multiples of the frame rate with
         * firing sidebands. The physical quantity is continuous; the step is
         * a telemetry artefact, so control-rate values are ramped at audio
         * rate instead of applied instantaneously.
         */
        std::array<float, maxRunners> delayTargetSamples {};
    };
    /** One-pole coefficient for control-rate parameter ramps (about 10 Hz). */
    float controlRampCoefficient_ { 0.0F };
    /** Size and zero every delay line for the engine currently published.
     *  Allocates, so it is called only from prepare(). */
    void allocateDelayLines();
    /** Latch the physical path and retire already-scheduled procedural exhaust. */
    void activatePhysicalExhaust() noexcept;
    void trigger(const FiringEvent&, bool exhaust) noexcept;
    void updateExhaustPreset(int preset) noexcept;
    /** Instantaneous physical state of one exhaust port's valve.
     *
     * Carried per runner so the port's orifice termination can be recomputed
     * every sample: its resistance depends on the acoustic velocity through the
     * opening, which is only known inside the waveguide.
     */
    using PortBoundary = AcousticExhaustNetwork::CylinderBoundary;
    [[nodiscard]] std::array<float, maximumPaths> processExhaustWaveguides(
        const std::array<float, maxRunners>& pulse,
        const std::array<std::uint8_t, maxRunners>& pathIndex,
        const std::array<float, maxRunners>& runnerAdmittance,
        const std::array<float, maxRunners>& portReflection,
        const std::array<PortBoundary, maxRunners>& portBoundary,
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
    /** Complete compiled DAG used by production engines. Null only for legacy
     * producers/tests that did not supply topology. */
    std::unique_ptr<AcousticExhaustNetwork> acousticExhaustNetwork_;
    /** Block/head modes compiled from immutable engine geometry. */
    std::unique_ptr<StructuralModalRadiator> structuralModalRadiator_;
    /** Runners/plenums/throttles/inlets compiled from EngineConfig. */
    std::unique_ptr<AcousticIntakeNetwork> acousticIntakeNetwork_;
    /** Rotor-order tones and jet radiation, parameterised by physical geometry. */
    std::unique_ptr<ForcedInductionAcoustics> forcedInductionAcoustics_;
    // Sized in prepare() to the cylinders and paths the loaded engine actually
    // has. Allocation stays off the callback; only the capacity is no longer a
    // worst-case guess paid for by every engine.
    std::vector<ExhaustPathState> exhaustPaths_;
    std::size_t allocatedRunners_ { 0 };
    std::size_t allocatedPaths_ { 0 };
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
    float radiationLowCoefficient_ { 0.03F };
    float dcInputLeft_ { 0.0F };
    float dcInputRight_ { 0.0F };
    float dcOutputLeft_ { 0.0F };
    float dcOutputRight_ { 0.0F };
    float toneLowLeft_ { 0.0F };
    float toneLowRight_ { 0.0F };
    float exhaustRadiationLowLeft_ { 0.0F };
    float exhaustRadiationLowRight_ { 0.0F };
    float wetExhaustRadiationLowLeft_ { 0.0F };
    float wetExhaustRadiationLowRight_ { 0.0F };
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
    std::array<float, 32> exhaustMeanPressurePa_ {};
    std::array<float, 32> exhaustMeanMassFlowKgPerSecond_ {};
    std::array<float, 32> thermoacousticRunnerAdmittance_ {};
    std::array<float, 32> thermoacousticPortReflection_ {};
    /** rho*c^2 of each runner's gas, for the finite-amplitude (steepening)
     *  propagation correction. Zero until physical telemetry arrives, which
     *  NonlinearDuctAcoustics::delayScale treats as exactly linear. */
    std::array<float, 32> runnerStiffnessRhoC2_ {};
    /** Latest published valve state per runner, and the two filter memories the
     *  orifice termination needs: one for the wave reflecting inside the runner,
     *  one for separating the measured boundary into source and reflection. */
    std::array<PortBoundary, maxRunners> portBoundary_ {};
    std::array<ValvePortTermination::State, maxRunners> portReflectionState_ {};
    std::array<ValvePortTermination::State, maxRunners> portSourceState_ {};
    /** Thermoviscous attenuation over one runner traversal, per direction. */
    std::array<DuctWallLoss::Coefficients, maxRunners> runnerWallLoss_ {};
    std::array<DuctWallLoss::State, maxRunners> runnerWallLossToJunction_ {};
    std::array<DuctWallLoss::State, maxRunners> runnerWallLossToPort_ {};
    /** Anti-imaging reconstruction of the sampled boundary. One shared
     *  coefficient set (the coupling rate is global), per-cylinder state for
     *  each of the two characteristic partners. See
     *  BoundaryReconstructionFilter. */
    BoundaryReconstructionFilter::Coefficients boundaryReconstruction_ {};
    double boundaryReconstructionCouplingHz_ { 0.0 };
    double valveFlowSourceSamplingHz_ { 0.0 };
    std::array<BoundaryReconstructionFilter::State, 32> boundaryReconstructionPressure_ {};
    std::array<BoundaryReconstructionFilter::State, 32> boundaryReconstructionFlow_ {};
    ValveFlowAcousticSource::Coefficients valveFlowAcousticSource_ {};
    std::array<ValveFlowAcousticSource::State, 32> valveFlowAcousticSourceState_ {};
    std::array<bool, 32> thermoacousticMeanInitialised_ {};
    float thermoacousticMeanCoefficient_ { 0.0F };
    // Atomic so a monitoring thread can observe which path is producing audio
    // without racing the renderer.
    std::atomic<bool> physicalExhaustActive_ { false };
    float pressureHighPassPole_ { 0.997F };
    float pressureBandCoefficient_ { 0.5F };
    double pressureSampleIntervalSeconds_ { 0.0 };
    double mechanicalPhase_ { 0.0 };
    double valvetrainPhase_ { 0.0 };
    double starterPhase_ { 0.0 };
    float smoothedRpm_ { 0.0F };
    float intakeFilter_ { 0.0F };
    float intakeSvfLow_ { 0.0F };
    float intakeSvfBand_ { 0.0F };
    std::atomic<std::uint64_t> lateEvents_ { 0 };
    std::atomic<std::uint64_t> stolenVoices_ { 0 };
    std::atomic<std::uint64_t> droppedPendingEvents_ { 0 };
    std::atomic<std::uint64_t> delayTruncations_ { 0 };
    // Safety-leveler observers (measurement only; updated once per block).
    std::atomic<std::uint64_t> levelLimitedSamples_ { 0 };
    std::atomic<float> minObservedLevelGain_ { 1.0F };
    std::atomic<float> maxObservedExhaustPressurePa_ { 0.0F };
    std::atomic<float> maxObservedIntakePressurePa_ { 0.0F };
    std::atomic<float> maxObservedStructuralPressurePa_ { 0.0F };
    std::atomic<float> maxPreLimiterMagnitude_ { 0.0F };
    std::atomic<std::uint64_t> legacyPathSamples_ { 0 };
    std::atomic<std::uint64_t> invalidBoundarySamples_ { 0 };
};
} // namespace enginelab
