#pragma once

#include <enginelab/events/StructuralExcitationSample.hpp>
#include <enginelab/foundation/EngineTypes.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace enginelab {

/** Reduced block/head structural dynamics driven by solver-resolved forces.
 *
 * The default mode set is calculated from an explicitly documented hollow-box
 * block and thin-plate head estimate. It is therefore identified as a family
 * estimate, not presented as measured NVH data. The realtime step integrates
 * the damped modal equation exactly for a zero-order-held generalized force and
 * converts modal surface velocity to far-field pressure from radiated power.
 */
class StructuralModalRadiator final {
public:
    using Provenance = StructuralNvhProvenance;

    struct ModeInfo final {
        double frequencyHz {};
        double dampingRatio {};
        double modalMassKg {};
        double radiatingAreaM2 {};
        double radiationEfficiency {};
    };

    explicit StructuralModalRadiator(const EngineConfig& config);

    [[nodiscard]] bool prepare(double sampleRateHz,
                               double observerDistanceM = 0.0) noexcept;
    void reset() noexcept;

    /** Same-binary diagnostic control for the pre-fix flat cylinder-index map.
     *
     * Production keeps the explicit bank topology enabled. Authored/measured
     * mode shapes are identical in both positions because their participation
     * is already authoritative.
     */
    void setBankTopologyParticipationEnabled(bool enabled) noexcept {
        bankTopologyParticipationEnabled_ = enabled;
    }
    [[nodiscard]] bool bankTopologyParticipationEnabled() const noexcept {
        return bankTopologyParticipationEnabled_;
    }

    /** Return signed far-field structure pressure at the observer, Pa. */
    [[nodiscard]] float process(const StructuralExcitationSample& excitation) noexcept;

    [[nodiscard]] bool valid() const noexcept { return !modes_.empty(); }
    [[nodiscard]] std::size_t modeCount() const noexcept { return modes_.size(); }
    [[nodiscard]] Provenance provenance() const noexcept { return provenance_; }
    [[nodiscard]] std::string_view source() const noexcept { return source_; }
    [[nodiscard]] ModeInfo mode(std::size_t index) const noexcept;

private:
    struct Mode final {
        ModeInfo info;
        StructuralModeDrive drive { StructuralModeDrive::headGas };
        std::array<float, 32> participation {};
        std::array<float, 32> legacyFlatIndexParticipation {};
        double torqueRadiusM { 0.05 };
        double angularFrequencyRadPerSecond {};
        double dampedFrequencyRadPerSecond {};
        double decay {};
        double cosine {};
        double sine {};
        double displacementM {};
        double velocityMps {};
        // RMS surface velocity divided by the modal antinode velocity.  Modal
        // coordinates use unit antinode displacement, whereas radiated power
        // depends on the area integral of velocity squared.
        double surfaceVelocityRmsScale { 1.0 };
    };

    std::vector<Mode> modes_;
    Provenance provenance_ { Provenance::estimatedFamily };
    std::string source_;
    double sampleRateHz_ { 48'000.0 };
    double observerDistanceM_ { 1.0 };
    double configuredObserverDistanceM_ { 1.0 };
    bool bankTopologyParticipationEnabled_ { true };
};

} // namespace enginelab
