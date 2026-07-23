#pragma once

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

    /** Refit delays and wall losses once per callback block. The optional
     *  per-path mean exhaust mass flow drives the outlet mean-flow convective
     *  loss; an empty span leaves every outlet quiescent. */
    void beginBlock(std::span<const Medium> pathMedia,
                    double acousticTimeScale,
                    std::span<const float> pathMeanMassFlowKgPerSecond = {}) noexcept;

    /** Propagate one sample and return each path at the two microphones, Pa. */
    [[nodiscard]] std::array<StereoPressure, maximumPaths> process(
        std::span<const float> cylinderSourcePressurePa,
        std::span<const CylinderBoundary> cylinderBoundaries,
        float delayRampCoefficient) noexcept;

    [[nodiscard]] bool valid() const noexcept;
    [[nodiscard]] std::size_t ductCount() const noexcept;
    [[nodiscard]] std::size_t junctionCount() const noexcept;
    [[nodiscard]] std::size_t outletCount() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace enginelab
