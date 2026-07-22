#pragma once

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

    explicit AcousticIntakeNetwork(const EngineConfig& config);
    ~AcousticIntakeNetwork();
    AcousticIntakeNetwork(const AcousticIntakeNetwork&) = delete;
    AcousticIntakeNetwork& operator=(const AcousticIntakeNetwork&) = delete;

    [[nodiscard]] bool prepare(double sampleRateHz,
                               double observerDistanceM = 1.0);
    void reset() noexcept;
    void beginBlock(std::span<const PathBoundary> paths,
                    double acousticTimeScale) noexcept;

    /** Return inlet pressure at both microphones per intake path, Pa. */
    [[nodiscard]] std::array<StereoPressure, maximumPaths> process(
        std::span<const CylinderBoundary> cylinders,
        float delayRampCoefficient) noexcept;

    [[nodiscard]] bool valid() const noexcept;
    [[nodiscard]] std::size_t runnerCount() const noexcept;
    [[nodiscard]] std::size_t pathCount() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace enginelab
