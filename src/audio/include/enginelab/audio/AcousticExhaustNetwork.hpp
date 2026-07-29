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
 * Every finite-length component becomes a bidirectional delay line. Direct
 * interfaces, merges and splitters are lossless admittance-scattering
 * junctions; every authored outlet owns an independent radiation load. The
 * finite-volume solver remains authoritative for mean flow and low-frequency
 * feedback, while this network transports the complementary audible band.
 */
class AcousticExhaustNetwork final {
public:
    static constexpr std::size_t maximumCylinders = 32;
    static constexpr std::size_t maximumPaths = 8;

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
     *  its own delay, wall loss and plane-mode cutoff from the gas actually in
     *  it, which differs by hundreds of kelvin between a header primary and a
     *  tailpipe. Where it is absent, or an entry is not usable, the duct falls
     *  back to its path medium. */
    void beginBlock(std::span<const Medium> pathMedia,
                    double acousticTimeScale,
                    std::span<const float> pathMeanMassFlowKgPerSecond = {},
                    std::span<const Medium> ductMedia = {}) noexcept;

    /** Propagate one sample and return each path at the two microphones, Pa. */
    [[nodiscard]] std::array<StereoPressure, maximumPaths> process(
        std::span<const float> cylinderSourcePressurePa,
        std::span<const CylinderBoundary> cylinderBoundaries,
        float delayRampCoefficient) noexcept;

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
