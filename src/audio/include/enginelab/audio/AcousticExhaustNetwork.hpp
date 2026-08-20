#pragma once

#include <enginelab/audio/ExhaustJetNoise.hpp>
#include <enginelab/audio/ValvePortTermination.hpp>
#include <enginelab/audio/FreeFieldObserver.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>

namespace enginelab {

/** Allocation-free audio-band wave network compiled from the complete exhaust DAG.
 *
 * Every constant-section component becomes a bidirectional delay line; a
 * finite taper is resolved by up to four bounded sections under a graph-wide
 * complexity budget. Direct interfaces, merges and splitters are lossless
 * admittance-scattering junctions, compact junction volumes and homogenised
 * perforated-core annuli are passive compliances, and every authored outlet
 * owns an independent radiation load. The
 * finite-volume solver remains authoritative for mean flow and low-frequency
 * feedback, while this network transports the complementary audible band.
 */
class AcousticExhaustNetwork final {
public:
    static constexpr std::size_t maximumCylinders = 32;
    static constexpr std::size_t maximumPaths = 8;
    /** Last-resort stability bound for a compact source in the linear network.
     * Any use must be reported by RealtimeEngineAudio; it is not a voicing
     * limiter and must never be silent instrumentation. */
    static constexpr float maximumReactionSourcePressurePa = 100'000.0F;

    struct CylinderBoundary final {
        float conductanceAreaM2 {};
        float meanMassFlowKgPerSecond {};
        float densityKgPerM3 {};
        float characteristicImpedancePaSPerM3 {};
        bool physical { false };
    };

    struct Medium final {
        float densityKgPerM3 { 1.2F };
        float soundSpeedMps { 343.0F };
    };

    struct OutletBoundary final {
        std::uint32_t nodeId { 0 };
        float signedMassFlowKgPerSecond { 0.0F };
        float densityKgPerM3 { 0.0F };
        float soundSpeedMps { 0.0F };
        float openingAreaM2 { 0.0F };
    };

    AcousticExhaustNetwork(const ExhaustGraph& graph,
                           std::span<const std::uint32_t> cylinderIds);
    ~AcousticExhaustNetwork();
    AcousticExhaustNetwork(const AcousticExhaustNetwork&) = delete;
    AcousticExhaustNetwork& operator=(const AcousticExhaustNetwork&) = delete;

    /** Allocate delay/radiation storage outside the audio callback. */
    [[nodiscard]] bool prepare(double sampleRateHz,
                               double maximumDelayScale = 12.5,
                               double observerDistanceM = 1.0);
    void reset() noexcept;

    /** Refit delays and wall losses once per callback block.
     *
     *  The optional per-path mean exhaust mass flow drives the outlet mean-flow
     *  convective loss; an empty span leaves every outlet quiescent.
     *
     *  `ductMedia` carries the gas state of each compiled duct, indexed as
     *  ExhaustNetworkLayout::ducts(). Where it is supplied every duct resolves
     *  its own delay and wall loss from the gas actually in it, which differs
     *  by hundreds of kelvin between a header primary and a tailpipe. Where it
     *  is absent, or an entry is not usable, the duct falls back to its path
     *  medium. */
    void beginBlock(std::span<const Medium> pathMedia,
                    double acousticTimeScale,
                    std::span<const float> pathMeanMassFlowKgPerSecond = {},
                    std::span<const Medium> ductMedia = {},
                    std::span<const OutletBoundary> outletBoundaries = {}) noexcept;

    /** Propagate one sample and return each path at the two microphones, Pa. */
    [[nodiscard]] std::array<StereoPressure, maximumPaths> process(
        std::span<const float> cylinderSourcePressurePa,
        std::span<const CylinderBoundary> cylinderBoundaries,
        float delayRampCoefficient) noexcept;

    /** Add a symmetric high-band pressure source at the finite-volume node
     * that released the energy. No outlet or cylinder fallback is used when a
     * node cannot be resolved. */
    [[nodiscard]] bool injectReactionPressure(
        std::uint32_t nodeId, float axialPosition,
        float sourcePressurePa) noexcept;

    /** Diagnostic A/B switch. Production leaves the source enabled. */
    void setOutletJetNoiseEnabled(bool enabled) noexcept;
    [[nodiscard]] bool outletJetNoiseEnabled() const noexcept;
    /** Last sample contributed by outlet turbulence, after both microphones. */
    [[nodiscard]] std::array<StereoPressure, maximumPaths>
    lastOutletJetNoisePressure() const noexcept;

    [[nodiscard]] bool valid() const noexcept;
    [[nodiscard]] std::size_t ductCount() const noexcept;
    [[nodiscard]] std::size_t junctionCount() const noexcept;
    [[nodiscard]] std::size_t outletCount() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace enginelab
