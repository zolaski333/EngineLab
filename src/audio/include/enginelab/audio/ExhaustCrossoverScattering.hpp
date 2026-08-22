#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace enginelab {

/** Allocation-free matched four-port used by an exhaust X-pipe.
 *
 * Endpoint order is upstream 0, upstream 1, downstream 0, downstream 1.
 * Waves are converted to power-normalised pressure coordinates
 * q=sqrt(Y)*p before applying a real orthogonal, reciprocal matrix. The
 * returned pressure waves therefore conserve sum(Y*p^2) even when connected
 * duct admittances differ. There are no self-reflections: adjacent authored
 * pipes own delay, wall loss and area transitions, while this compact element
 * owns only straight/cross directionality.
 */
class ExhaustCrossoverScattering final {
public:
    using Waves = std::array<float, 4>;
    using Admittances = std::array<double, 4>;
    using RootAdmittances = std::array<float, 4>;

    [[nodiscard]] static Waves scatterFromRootAdmittanceAndCoupling(
        const Waves& incidentPressurePa,
        const RootAdmittances& rootAdmittance,
        float straightCoupling,
        float crossCoupling) noexcept {
        Waves normalisedIncident {};
        Waves result {};
        for (std::size_t index = 0; index < rootAdmittance.size(); ++index) {
            if (!std::isfinite(rootAdmittance[index])
                || !(rootAdmittance[index] > 0.0F)
                || !std::isfinite(incidentPressurePa[index]))
                return {};
            normalisedIncident[index] =
                rootAdmittance[index] * incidentPressurePa[index];
        }

        const std::array<float, 4> normalisedOutgoing {
            straightCoupling * normalisedIncident[2]
                + crossCoupling * normalisedIncident[3],
            -crossCoupling * normalisedIncident[2]
                + straightCoupling * normalisedIncident[3],
            straightCoupling * normalisedIncident[0]
                - crossCoupling * normalisedIncident[1],
            crossCoupling * normalisedIncident[0]
                + straightCoupling * normalisedIncident[1],
        };
        for (std::size_t index = 0; index < result.size(); ++index) {
            const auto pressure = normalisedOutgoing[index]
                / rootAdmittance[index];
            result[index] = std::isfinite(pressure) ? pressure : 0.0F;
        }
        return result;
    }

    /** Scatter with precomputed sqrt(Y). This is the realtime entry point:
     * medium targets only change once per block, so recomputing four square
     * roots at every audio sample would spend CPU without changing the model. */
    [[nodiscard]] static Waves scatterFromRootAdmittance(
        const Waves& incidentPressurePa,
        const RootAdmittances& rootAdmittance,
        double crossCoupling) noexcept {
        const auto cross = static_cast<float>(
            std::clamp(crossCoupling, 0.0, 1.0));
        const auto straight = static_cast<float>(
            std::sqrt(std::max(0.0, 1.0 - cross * cross)));
        return scatterFromRootAdmittanceAndCoupling(
            incidentPressurePa, rootAdmittance, straight, cross);
    }

    [[nodiscard]] static Waves scatter(
        const Waves& incidentPressurePa,
        const Admittances& admittanceM3PerPaSecond,
        double crossCoupling) noexcept {
        RootAdmittances rootAdmittance {};
        for (std::size_t index = 0; index < rootAdmittance.size(); ++index) {
            const auto admittance = admittanceM3PerPaSecond[index];
            if (!std::isfinite(admittance) || !(admittance > 0.0)
                || !std::isfinite(incidentPressurePa[index]))
                return {};
            rootAdmittance[index] = static_cast<float>(std::sqrt(admittance));
        }
        return scatterFromRootAdmittance(
            incidentPressurePa, rootAdmittance, crossCoupling);
    }
};

} // namespace enginelab
