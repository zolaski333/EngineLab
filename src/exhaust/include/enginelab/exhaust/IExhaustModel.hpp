#pragma once
#include <enginelab/events/FiringEvent.hpp>
#include <enginelab/foundation/EngineTypes.hpp>
#include <cstddef>
#include <cstdint>
namespace enginelab {

/** Lumped physical properties compiled from one authored exhaust DAG. */
struct ExhaustPathFlowProperties final {
    bool authoredNetwork { false };
    double equivalentRestriction { 0.0 };
    double collectorVolumeLitres { 0.0 };
    // Conductance is represented as an equivalent area with this coefficient.
    double effectiveOutletAreaM2 { 0.0 };
    double outletDischargeCoefficient { 1.0 };
    double meanFlowLengthMm { 0.0 };
};

/** Physical inlet throat compiled for a cylinder's first DAG component. */
struct ExhaustCylinderFlowProperties final {
    bool authoredNetwork { false };
    std::uint32_t cylinderId { 0 };
    std::uint32_t pathIndex { 0 };
    double inletAreaM2 { 0.0 };
    double inletDischargeCoefficient { 1.0 };
    double runnerVolumeLitres { 0.0 };
    double runnerLengthMm { 0.0 };
};

class IExhaustModel {
public:
    virtual ~IExhaustModel() = default;
    [[nodiscard]] virtual double backPressureKpa(const EngineState&) const noexcept = 0;
    virtual void process(FiringEvent&) const noexcept = 0;
    [[nodiscard]] virtual ExhaustPathFlowProperties pathFlowProperties(
        std::size_t) const noexcept { return {}; }
    [[nodiscard]] virtual ExhaustCylinderFlowProperties cylinderFlowProperties(
        std::uint32_t) const noexcept { return {}; }
};
} // namespace enginelab
