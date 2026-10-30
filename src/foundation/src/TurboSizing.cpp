#include <enginelab/foundation/TurboSizing.hpp>
#include <enginelab/foundation/ForcedInductionFlow.hpp>

#include <algorithm>
#include <cmath>
#include <utility>

namespace enginelab {

double turbochargerDesignShaftSpeedRpm(double pressureRatio, double inducerDiameterMm) {
    // The tip speed the head needs (Euler: the head goes as the tip speed
    // squared), anchored on the TDI-like's authored wheel: 39 mm at 165,000
    // rpm for 2.3, 337 m/s. The default 130,000 rpm on every size put a 193 L
    // marine six's 162 mm wheel at 1,100 m/s: 5 MJ in its shaft, still
    // spooling after 8 s at 1.34 of its 3.0 (journal 2026-10-30).
    constexpr double anchorTipSpeedMps = 3.14159265 * 0.039 * 165'000.0 / 60.0;
    const auto anchorHead = std::pow(2.3, turboIsentropicExponent) - 1.0;
    const auto head = std::pow(std::max(1.01, pressureRatio), turboIsentropicExponent) - 1.0;
    const auto tipSpeedMps = anchorTipSpeedMps * std::sqrt(head / anchorHead);
    return tipSpeedMps * 60.0 / (3.14159265 * 0.001 * std::max(1.0, inducerDiameterMm));
}

ForcedInductionConfig sizedTurbocharger(const EngineConfig& config) {
    const auto litres = std::max(0.05, breathingDisplacementLitres(config));
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
    turbo.designShaftSpeedRpm = turbochargerDesignShaftSpeedRpm(turbo.pressureRatio, turbo.compressorInducerDiameterMm);
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

EngineConfig withTurbochargerCount(EngineConfig config, std::uint32_t count) {
    auto& turbo = config.forcedInduction;
    const auto was = std::clamp<std::uint32_t>(turbo.turbochargerCount, 1U,
        static_cast<std::uint32_t>(maximumTurbochargers));
    const auto now = std::clamp<std::uint32_t>(count, 1U, static_cast<std::uint32_t>(maximumTurbochargers));
    if (!turbo.enabled || turbo.type != ForcedInductionType::turbocharger || was == now) return config;
    const auto share = static_cast<double>(was) / static_cast<double>(now);
    // Each unit is the old one scaled to its share of the air at the same
    // pressure ratio, so at the same tip speed: its wheels shrink as the root
    // of the flow (the areas go as the flow), spin faster by as much, and a
    // wheel's inertia goes as its mass times its radius squared, the fifth
    // power of its size. Two units hold half the shaft energy of one at the
    // design point between them and get half its turbine power each: they
    // spool sqrt(2) faster, which is why engines carry two. Scaling the
    // inertia with the flow alone kept the shaft speed and made a pair spool
    // exactly as the single did.
    const auto size = std::sqrt(share);
    // Resolved before the count changes: the default reads it.
    turbo.compressorDesignMassFlowKgPerSecond = compressorDesignMassFlowKgPerSecond(config) * share;
    turbo.turbineFlowAreaMm2 *= share;
    turbo.wastegateFlowAreaMm2 *= share;
    turbo.compressorInducerDiameterMm *= size;
    turbo.turbineExducerDiameterMm *= size;
    turbo.designShaftSpeedRpm /= size;
    turbo.shaftInertiaKgM2 *= share * share * size;
    // Journal bearings: friction torque as the speed and the cube of the
    // shaft's size, so the loss at the design speed goes as the root of the
    // share.
    turbo.bearingFrictionPowerWatts *= size;
    turbo.turbochargerCount = now;
    for (auto& path : config.exhaustPaths)
        if (path.turbocharger >= static_cast<std::int32_t>(now)) path.turbocharger = -1;
    return config;
}

} // namespace enginelab
