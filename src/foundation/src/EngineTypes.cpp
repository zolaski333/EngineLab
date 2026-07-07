#include <enginelab/foundation/EngineTypes.hpp>
#include <algorithm>
#include <cmath>
#include <numbers>
#include <unordered_set>
#include <limits>

namespace enginelab {

namespace {
EngineConfig makeEngine(std::string name, EngineLayout layout, std::uint32_t count,
                        std::vector<std::uint32_t> order, double bore, double stroke,
                        double idle, double redline, double inertia, CamshaftConfig cams) {
    EngineConfig config;
    config.name = std::move(name);
    config.layout = layout;
    config.idleRpm = idle;
    config.redlineRpm = redline;
    config.rotatingInertiaKgM2 = inertia;
    config.plenumVolumeLitres = std::max(0.6, static_cast<double>(count) * 0.70);
    config.throttleDiameterMm = std::clamp(36.0 + static_cast<double>(count) * 4.5, 42.0, 82.0);
    config.bankAngleDegrees = layout == EngineLayout::vLayout ? 60.0 : 0.0;
    config.camshafts = cams;
    config.cylinders.reserve(count);
    const auto spacing = 720.0 / static_cast<double>(count);
    for (std::uint32_t id = 1; id <= count; ++id) {
        CylinderConfig cylinder;
        cylinder.id = id;
        cylinder.boreMm = bore;
        cylinder.strokeMm = stroke;
        cylinder.efficiencyOffset = (static_cast<double>(id) - (static_cast<double>(count) + 1.0) * 0.5) * 0.003;
        const auto it = std::find(order.begin(), order.end(), id);
        if (it != order.end()) {
            const auto orderIndex = static_cast<double>(std::distance(order.begin(), it));
            cylinder.crankOffsetDegrees = orderIndex * spacing;
        } else {
            cylinder.crankOffsetDegrees = 0.0;
        }
        config.cylinders.push_back(cylinder);
    }
    config.firingOrder = std::move(order);
    return config;
}
}

EngineConfig makeDefaultInlineFour() {
    return makeEngine("EL-20 I4", EngineLayout::inlineLayout, 4, { 1, 3, 4, 2 },
                      86.0, 86.0, 850.0, 7'200.0, 0.24, {});
}

EngineConfig makeDefaultInlineTwo() {
    return makeEngine("EL-09 I2", EngineLayout::inlineLayout, 2, { 1, 2 },
                      84.0, 81.0, 1'050.0, 8'200.0, 0.13, { 262.0, 256.0, 10.8, 10.3, 108.0, 110.0 });
}

EngineConfig makeDefaultInlineFive() {
    return makeEngine("EL-25 I5", EngineLayout::inlineLayout, 5, { 1, 2, 4, 5, 3 },
                      82.5, 92.8, 780.0, 6'800.0, 0.29, { 252.0, 248.0, 10.5, 10.0, 110.0, 112.0 });
}

EngineConfig makeDefaultV6() {
    return makeEngine("EL-30 V6", EngineLayout::vLayout, 6, { 1, 4, 2, 5, 3, 6 },
                      86.0, 86.0, 750.0, 7'000.0, 0.32, { 256.0, 252.0, 11.0, 10.5, 109.0, 111.0 });
}

EngineConfig makeDefaultV8() {
    auto config = makeEngine("EL-50 V8", EngineLayout::vLayout, 8, { 1, 5, 4, 8, 6, 3, 7, 2 },
                             94.0, 90.0, 720.0, 6'600.0, 0.46, { 268.0, 264.0, 12.0, 11.5, 108.0, 110.0 });
    config.frictionCoefficient = 0.15;
    config.bankAngleDegrees = 90.0;
    return config;
}

std::vector<EngineConfig> makeBaseEnginePresets() {
    return { makeDefaultInlineTwo(), makeDefaultInlineFour(), makeDefaultInlineFive(), makeDefaultV6(), makeDefaultV8() };
}

double engineDisplacementLitres(const EngineConfig& config) noexcept {
    double cubicMillimetres = 0.0;
    for (const auto& cylinder : config.cylinders)
        cubicMillimetres += std::numbers::pi * cylinder.boreMm * cylinder.boreMm * 0.25 * cylinder.strokeMm;
    return cubicMillimetres / 1'000'000.0;
}

double valveLiftMm(double crankAngleDegrees, double centerlineDegrees,
                   double durationDegrees, double maximumLiftMm) noexcept {
    const auto wrapped = std::remainder(crankAngleDegrees - centerlineDegrees, 720.0);
    const auto halfDuration = std::max(1.0, durationDegrees * 0.5);
    if (std::abs(wrapped) >= halfDuration) return 0.0;
    const auto phase = (wrapped + halfDuration) / durationDegrees;
    return std::max(0.0, maximumLiftMm * 0.5 * (1.0 - std::cos(phase * 2.0 * std::numbers::pi)));
}

double combustionPulse(double cylinderPhaseDegrees, double ignitionAdvanceDegrees,
                       double ignitionOffsetDegrees) noexcept {
    const auto pressurePhase = std::fmod(cylinderPhaseDegrees + ignitionAdvanceDegrees * 0.35
        - ignitionOffsetDegrees + 720.0, 720.0);
    return pressurePhase < 180.0
        ? std::pow(std::sin(pressurePhase * std::numbers::pi / 180.0), 2.0) : 0.0;
}

std::optional<std::string> validateEngineConfig(const EngineConfig& config) {
    const auto inRange = [](double value, double minimum, double maximum) {
        return std::isfinite(value) && value >= minimum && value <= maximum;
    };
    if (config.schemaVersion != 1) return "Unsupported engine schema version";
    if (config.name.empty() || config.name.size() > 128) return "Engine name must contain between 1 and 128 bytes";
    if (config.cycle != EngineCycle::fourStroke || config.fuel != FuelType::gasoline)
        return "Only four-stroke gasoline engines are currently supported";
    if (config.cylinders.empty() || config.cylinders.size() > 32 || config.firingOrder.size() != config.cylinders.size())
        return "Firing order must reference between 1 and 32 cylinders";
    if (!inRange(config.idleRpm, 200.0, 3'000.0)
        || !inRange(config.redlineRpm, config.idleRpm + 100.0, 20'000.0)
        || !inRange(config.rotatingInertiaKgM2, 0.01, 5.0)
        || !inRange(config.frictionCoefficient, 0.0, 1.0)
        || !inRange(config.octaneRating, 70.0, 130.0)
        || !inRange(config.ambientPressureKpa, 50.0, 120.0)
        || !inRange(config.ambientTemperatureC, -50.0, 60.0)
        || !inRange(config.coolingEfficiency, 0.1, 5.0)
        || !inRange(config.plenumVolumeLitres, 0.1, 50.0)
        || !inRange(config.throttleDiameterMm, 15.0, 150.0)
        || !inRange(config.bankAngleDegrees, 0.0, 180.0)
        || !inRange(config.camshafts.intakeDurationDegrees, 1.0, 720.0)
        || !inRange(config.camshafts.exhaustDurationDegrees, 1.0, 720.0)
        || !inRange(config.camshafts.intakeLiftMm, 0.0, 30.0)
        || !inRange(config.camshafts.exhaustLiftMm, 0.0, 30.0)
        || !inRange(config.camshafts.intakeCenterlineDegrees, 0.0, 720.0)
        || !inRange(config.camshafts.exhaustCenterlineDegrees, 0.0, 720.0)
        || !inRange(config.exhaust.primaryLengthMm, 100.0, 3'000.0)
        || !inRange(config.exhaust.primaryDiameterMm, 15.0, 200.0)
        || !inRange(config.exhaust.collectorDiameterMm, 15.0, 300.0)
        || !inRange(config.exhaust.outletDiameterMm, 15.0, 300.0)
        || !inRange(config.exhaust.mufflerRestriction, 0.0, 1.0))
        return "Engine configuration contains non-finite or physically invalid values";
    std::unordered_set<std::uint32_t> cylinderIds;
    constexpr std::uint32_t mergeId = std::numeric_limits<std::uint32_t>::max() - 2U;
    for (const auto& cylinder : config.cylinders) {
        if (cylinder.id == 0 || cylinder.id >= mergeId || !inRange(cylinder.boreMm, 20.0, 200.0)
            || !inRange(cylinder.strokeMm, 20.0, 200.0)
            || !inRange(cylinder.connectingRodMm, cylinder.strokeMm * 0.5 + 0.01, 500.0)
            || !inRange(cylinder.pistonMassGrams, 50.0, 2'500.0)
            || !inRange(cylinder.compressionRatio, 5.0, 20.0)
            || !inRange(cylinder.ignitionOffsetDegrees, -30.0, 30.0)
            || !inRange(cylinder.efficiencyOffset, -0.5, 0.5)
            || !inRange(cylinder.crankOffsetDegrees, -360.0, 720.0))
            return "Cylinder IDs and dimensions must be finite and physically valid";
        cylinderIds.insert(cylinder.id);
    }
    const std::unordered_set<std::uint32_t> firingIds(config.firingOrder.begin(), config.firingOrder.end());
    if (cylinderIds.size() != config.cylinders.size()) return "Cylinder IDs must be unique";
    if (firingIds != cylinderIds) return "Firing order must contain every cylinder ID exactly once";
    const auto displacement = engineDisplacementLitres(config);
    if (!inRange(displacement, 0.05, 20.0)) return "Total displacement must be between 0.05 and 20 litres";
    return std::nullopt;
}

} // namespace enginelab
