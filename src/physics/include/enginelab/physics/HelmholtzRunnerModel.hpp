#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

namespace enginelab {

struct HelmholtzRunnerState final {
    double pressureAmplitudeKpa { 0.0 };
    double pressureVelocityKpaPerSecond { 0.0 };
};

struct HelmholtzRunnerResult final {
    double pressureAmplitudeKpa { 0.0 };
    double resonanceFrequencyHz { 0.0 };
    double flowAdmittance { 1.0 };
};

/** Lumped runner/plenum acoustic compliance coupled to the conservative mass-flow restriction. */
class HelmholtzRunnerModel final {
public:
    [[nodiscard]] static HelmholtzRunnerResult advance(const RunnerAcousticsConfig&,
        HelmholtzRunnerState&, const CylinderConfig&, const IntakeConfig&,
        double gasTemperatureK, double plenumPressureKpa, double runnerPressureKpa,
        double dtSeconds) noexcept;
};

} // namespace enginelab
