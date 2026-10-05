#pragma once

#include <enginelab/foundation/GasFieldSnapshot.hpp>
#include <enginelab/render/EngineModel3D.hpp>

#include <array>
#include <cstdint>
#include <vector>

namespace enginelab::render {

/** For each scene duct (in EngineModel3D::ducts() order), the index of the
    GasFieldSnapshot element that carries its gas, or -1 when the solver has
    none (an airbox, a throttle body, an acoustic side branch). */
[[nodiscard]] std::vector<int> bindGasField(const EngineModel3D&, const GasFieldSnapshot&);

/** The prototype's wave colours, linear RGB: violet below the mean, grey at
    the mean, orange then pale yellow above. `normalised` is the departure
    over the scale, -1..1. */
[[nodiscard]] Vec3 pressureColour(float normalised) noexcept;

/**
 * What the renderer draws along the exhaust and the intake runners and
 * plenums: per duct part, the wave colour and the wall temperature at up to
 * GasFieldElement::maximumSamples stations.
 *
 * The colour is the wave: each cell's departure from its own running mean
 * (averaged over simulated time, so a slowed or frozen view keeps it). Against
 * ambient, a turbocharged exhaust, whose whole 1-D network sits upstream of
 * the turbine, would read as one saturated colour. The scale follows the
 * strongest departure anywhere, decaying slowly, so an idle shows its waves
 * as clearly as full load; the legend states the scale. Intake and exhaust
 * share it, so their relative strength reads true: the intake's peak is 0.6
 * to 5 times the exhaust's over the catalogue.
 */
class GasFieldView final {
public:
    struct PartField final {
        std::uint16_t part {};
        std::uint8_t count {};
        /** Per station: wave colour (linear RGB), wall temperature (K). */
        std::array<float, 4 * GasFieldElement::maximumSamples> samples {};
    };

    /** Takes a new snapshot. Call again with the same model to rebind only
        when the engine changes. */
    void update(const EngineModel3D&, const GasFieldSnapshot&);
    void clear();

    [[nodiscard]] const std::vector<PartField>& parts() const noexcept { return parts_; }
    /** Departure from the mean shown at either end of the colour scale. */
    [[nodiscard]] float pressureScalePa() const noexcept { return scalePa_; }
    /** Strength of the flash at the outlets, 0..1. */
    [[nodiscard]] float afterfire() const noexcept { return afterfire_; }
    [[nodiscard]] bool valid() const noexcept { return !parts_.empty(); }

private:
    const EngineModel3D* model_ {};
    std::size_t boundElements_ {};
    std::vector<int> binding_;
    std::vector<PartField> parts_;
    /** Per snapshot element and sample, the running mean pressure (Pa). */
    std::vector<std::array<float, GasFieldElement::maximumSamples>> meanPa_;
    double meanSeconds_ {};
    float scalePa_ {};
    float afterfire_ {};
};

} // namespace enginelab::render
