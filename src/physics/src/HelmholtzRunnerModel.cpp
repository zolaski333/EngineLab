#include <enginelab/physics/HelmholtzRunnerModel.hpp>
#include <algorithm>
#include <cmath>
#include <numbers>

namespace enginelab {

HelmholtzRunnerResult HelmholtzRunnerModel::advance(const RunnerAcousticsConfig& config,
    HelmholtzRunnerState& state, const CylinderConfig& cylinder, const IntakeConfig& intake,
    double temperatureK, double plenumPressureKpa, double runnerPressureKpa,
    double dtSeconds) noexcept {
    const auto runnerDiameterMm = cylinder.intakeRunnerDiameterMm > 0.0
        ? cylinder.intakeRunnerDiameterMm : intake.runnerDiameterMm;
    const auto runnerLengthMm = cylinder.intakeRunnerLengthMm > 0.0
        ? cylinder.intakeRunnerLengthMm : intake.runnerLengthMm;
    const auto diameterM = std::max(0.001, runnerDiameterMm * 0.001);
    const auto areaM2 = std::numbers::pi * diameterM * diameterM * 0.25;
    const auto effectiveLengthM = std::max(0.02, runnerLengthMm * 0.001 + diameterM * 0.85);
    const auto plenumVolumeM3 = std::max(0.00005, intake.plenumVolumeLitres * 0.001);
    const auto speedOfSound = std::sqrt(1.4 * 287.05 * std::max(200.0, temperatureK));
    const auto angularFrequency = speedOfSound * std::sqrt(areaM2 / (plenumVolumeM3 * effectiveLengthM));
    const auto frequencyHz = angularFrequency / (2.0 * std::numbers::pi);
    if (!config.enabled || dtSeconds <= 0.0) return { 0.0, frequencyHz, 1.0 };

    const auto forcingKpa = (plenumPressureKpa - runnerPressureKpa) * config.couplingGain;
    const auto acceleration = angularFrequency * angularFrequency
        * (forcingKpa - state.pressureAmplitudeKpa)
        - 2.0 * config.dampingRatio * angularFrequency * state.pressureVelocityKpaPerSecond;
    state.pressureVelocityKpaPerSecond += acceleration * dtSeconds;
    state.pressureAmplitudeKpa += state.pressureVelocityKpaPerSecond * dtSeconds;
    if (std::abs(state.pressureAmplitudeKpa) > config.maximumPressureAmplitudeKpa) {
        state.pressureAmplitudeKpa = std::copysign(config.maximumPressureAmplitudeKpa,
                                                   state.pressureAmplitudeKpa);
        state.pressureVelocityKpaPerSecond *= 0.25;
    }
    const auto pressureScale = std::max(5.0, std::abs(plenumPressureKpa - runnerPressureKpa));
    const auto admittance = std::clamp(1.0 + state.pressureAmplitudeKpa / pressureScale, 0.20, 1.80);
    return { state.pressureAmplitudeKpa, frequencyHz, admittance };
}

} // namespace enginelab
