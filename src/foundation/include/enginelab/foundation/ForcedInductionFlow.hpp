#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

#include <algorithm>

namespace enginelab {

struct TurboExhaustFlowSplit final {
    double turbineKgPerSecond {};
    double wastegateKgPerSecond {};
};

/** Conservatively partitions the measured exhaust flow through the parallel
 *  effective turbine and wastegate throats. The same split drives shaft power
 *  and aeroacoustics so neither subsystem can create or discard bypass flow. */
[[nodiscard]] inline TurboExhaustFlowSplit partitionTurboExhaustFlow(
    const ForcedInductionConfig& config, double totalKgPerSecond,
    double wastegateOpening) noexcept {
    const auto total = std::max(0.0, totalKgPerSecond);
    const auto turbineArea = std::max(0.0, config.turbineFlowAreaMm2);
    const auto wastegateArea = std::max(0.0, config.wastegateFlowAreaMm2)
        * std::clamp(wastegateOpening, 0.0, 1.0);
    const auto effectiveArea = turbineArea + wastegateArea;
    if (!(effectiveArea > 0.0)) return { total, 0.0 };
    const auto bypass = total * wastegateArea / effectiveArea;
    return { total - bypass, bypass };
}

} // namespace enginelab
