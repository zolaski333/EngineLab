#pragma once

#include <enginelab/foundation/AcousticSourcePlacement.hpp>
#include <enginelab/foundation/EngineTypes.hpp>
#include <enginelab/audio/FreeFieldObserver.hpp>

#include <array>
#include <cstddef>
#include <memory>
#include <span>

namespace enginelab {

/** Audio-rate intake network coupled one-way to the conservative 0-D charge model.
 *
 * Individual runners, plenum compliance, throttle conductance, optional airbox
 * compliance/inlet duct and inlet radiation all come from EngineConfig. The
 * network transports only zero-mean acoustic perturbations; mass and mean
 * pressure remain owned by EngineSimulator.
 */
class AcousticIntakeNetwork final {
public:
    static constexpr std::size_t maximumCylinders = 32;
    static constexpr std::size_t maximumPaths = 8;

    struct CylinderBoundary final {
        float massFlowKgPerSecond {};
        float conductanceAreaM2 {};
        float densityKgPerM3 { 1.2F };
        float soundSpeedMps { 343.0F };
        bool physical { false };
    };

    struct PathBoundary final {
        float throttleConductanceAreaM2 {};
        float densityKgPerM3 { 1.2F };
        float soundSpeedMps { 343.0F };
    };

    /** Cumulative absolute pressure peaks since reset(), in pascals.
     *
     * These taps are measurement-only and make it possible to distinguish an
     * over-driven valve source from gain created by a resonant network stage.
     */
    struct Diagnostics final {
        float sourcePressurePa {};
        float runnerPressurePa {};
        float plenumPressurePa {};
        float airboxPressurePa {};
        float mouthPressurePa {};
        float radiatedPressurePa {};
    };

    explicit AcousticIntakeNetwork(const EngineConfig& config);
    ~AcousticIntakeNetwork();
    AcousticIntakeNetwork(const AcousticIntakeNetwork&) = delete;
    AcousticIntakeNetwork& operator=(const AcousticIntakeNetwork&) = delete;

    [[nodiscard]] bool prepare(double sampleRateHz,
                               double observerDistanceM = 1.0);
    void reset() noexcept;
    /** Every inlet's microphones glide to `left` and `right`
     * (FreeFieldObserver::moveMicrophones). Audio thread. */
    void moveMicrophones(const AcousticPoint3M& left, const AcousticPoint3M& right,
                         bool immediately = false) noexcept;
    /** Each path's mouth radiates from its placement in `mouths` (matched by
     * path index), or from the engine's origin, as prepared, when none
     * matches (FreeFieldObserver::moveSource). Audio thread. */
    void placeMouths(std::span<const AcousticSourcePlacement> mouths,
                     bool immediately = false) noexcept;
    void beginBlock(std::span<const PathBoundary> paths,
                    double acousticTimeScale) noexcept;
    /** Update gas/throttle targets at the producer interpolation cadence.
     *
     * Duct delay and wall-loss fits remain block-rate work in beginBlock();
     * these lightweight boundary targets may be refreshed for every sample so
     * the physical result cannot depend on the host callback size.
     */
    void updateBoundaryTargets(std::span<const PathBoundary> paths) noexcept;

    /** Return inlet pressure at both microphones per intake path, Pa. */
    [[nodiscard]] std::array<StereoPressure, maximumPaths> process(
        std::span<const CylinderBoundary> cylinders,
        float delayRampCoefficient) noexcept;

    [[nodiscard]] bool valid() const noexcept;
    [[nodiscard]] std::size_t runnerCount() const noexcept;
    [[nodiscard]] std::size_t pathCount() const noexcept;
    [[nodiscard]] Diagnostics diagnostics() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace enginelab
