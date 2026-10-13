#include <enginelab/foundation/TurboSizing.hpp>

#include <algorithm>
#include <cmath>
#include <utility>

namespace enginelab {

ForcedInductionConfig sizedTurbocharger(const EngineConfig& config) {
    const auto litres = std::max(0.05, engineDisplacementLitres(config));
    const auto redline = std::max(1'000.0, config.redlineRpm);
    double compression = 0.0;
    for (const auto& cylinder : config.cylinders) compression = std::max(compression, cylinder.compressionRatio);
    const auto diesel = config.fuel == FuelType::diesel;

    ForcedInductionConfig turbo;
    turbo.enabled = true;
    turbo.type = ForcedInductionType::turbocharger;
    turbo.pressureRatio = diesel ? 2.3 : compression >= 10.5 ? 1.5 : compression >= 9.5 ? 1.7 : 1.9;
    turbo.wastegatePressureRatio = turbo.pressureRatio - 0.04;
    turbo.fullBoostRpm = 0.45 * redline;
    turbo.compressorEfficiency = 0.72;
    turbo.turbineEfficiency = 0.70;
    turbo.chargeTemperatureRiseC = diesel ? 28.0 : 42.0;
    turbo.turbineFlowAreaMm2 = 0.028 * litres * redline;
    turbo.wastegateFlowAreaMm2 = 1.5 * turbo.turbineFlowAreaMm2;
    turbo.compressorInducerDiameterMm = 7.8 * std::sqrt(litres * redline / 1'000.0 * turbo.pressureRatio);
    turbo.turbineExducerDiameterMm = 0.9 * turbo.compressorInducerDiameterMm;
    turbo.shaftInertiaKgM2 = 4.75e-13 * std::pow(turbo.compressorInducerDiameterMm, 5.0);
    turbo.compressorBladeCount = 6;
    turbo.turbineBladeCount = 11;
    // A diesel has no throttle to close against the compressor.
    turbo.blowOffValveFlowAreaMm2 = diesel ? 0.0 : 0.65 * turbo.turbineFlowAreaMm2;
    return turbo;
}

EngineConfig withTurbocharger(EngineConfig config) {
    config.forcedInduction = sizedTurbocharger(config);
    return config;
}

EngineConfig withoutForcedInduction(EngineConfig config) {
    config.forcedInduction = ForcedInductionConfig {};
    return config;
}

} // namespace enginelab
