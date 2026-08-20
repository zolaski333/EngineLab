#include <enginelab/foundation/EngineTypes.hpp>
#include <enginelab/foundation/CatalystMonolithGeometry.hpp>
#include <algorithm>
#include <cmath>
#include <numbers>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <limits>

namespace enginelab {

namespace {
[[nodiscard]] bool validMufflerPacking(const ExhaustConfig& exhaust) noexcept {
    const auto none = exhaust.mufflerPackingFlowResistivityPaSPerM2 == 0.0
        && exhaust.mufflerPackingThicknessMm == 0.0
        && exhaust.mufflerPerforatedOpenAreaRatio == 0.0;
    const auto complete = exhaust.mufflerPackingFlowResistivityPaSPerM2 > 0.0
        && exhaust.mufflerPackingThicknessMm > 0.0
        && exhaust.mufflerPerforatedOpenAreaRatio > 0.0
        && exhaust.mufflerChamberDiameterMm > 1.0
        && exhaust.mufflerChamberLengthMm > 1.0;
    return none || complete;
}

[[nodiscard]] IntakePathConfig makeDefaultIntakePath(const EngineConfig& config) {
    IntakePathConfig path;
    path.id = 1;
    path.geometry = config.intake;
    path.inheritsGlobalGeometry = true;
    for (const auto& cylinder : config.cylinders) path.cylinderIds.push_back(cylinder.id);
    return path;
}

[[nodiscard]] ExhaustPathConfig makeDefaultExhaustPath(const EngineConfig& config) {
    ExhaustPathConfig path;
    path.id = 1;
    path.geometry = config.exhaust;
    path.inheritsGlobalGeometry = true;
    for (const auto& cylinder : config.cylinders) path.cylinderIds.push_back(cylinder.id);
    return path;
}

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
    config.bankAngleDegrees = layout == EngineLayout::vLayout ? 60.0
        : (layout == EngineLayout::flat ? 180.0 : 0.0);
    config.camshafts = cams;
    config.intake.plenumVolumeLitres = config.plenumVolumeLitres;
    config.intake.throttleDiameterMm = config.throttleDiameterMm;
    config.ignition.revLimitRpm = redline;
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
        cylinder.crankJournalId = id;
        if (layout == EngineLayout::vLayout) {
            cylinder.bankOffsetDegrees = id % 2U == 0U ? config.bankAngleDegrees * 0.5 : -config.bankAngleDegrees * 0.5;
        } else if (layout == EngineLayout::flat) {
            cylinder.bankOffsetDegrees = id % 2U == 0U ? 90.0 : -90.0;
        }
        config.crankJournals.push_back({ id,
            cylinder.crankOffsetDegrees - cylinder.bankOffsetDegrees, stroke * 0.5, 1 });
        config.cylinders.push_back(cylinder);
    }
    config.firingOrder = std::move(order);
    if (layout == EngineLayout::vLayout || layout == EngineLayout::flat) {
        CylinderBankConfig left { 1, -config.bankAngleDegrees * 0.5, {}, cams, 1, 1 };
        CylinderBankConfig right { 2, config.bankAngleDegrees * 0.5, {}, cams, 1, 1 };
        for (auto& cylinder : config.cylinders) {
            cylinder.bankId = cylinder.id % 2U == 0U ? right.id : left.id;
            (cylinder.id % 2U == 0U ? right.cylinderIds : left.cylinderIds).push_back(cylinder.id);
        }
        config.banks = { std::move(left), std::move(right) };
    } else {
        CylinderBankConfig bank { 1, 0.0, {}, cams, 1, 1 };
        for (auto& cylinder : config.cylinders) {
            cylinder.bankId = bank.id;
            bank.cylinderIds.push_back(cylinder.id);
        }
        config.banks.push_back(std::move(bank));
    }
    ExhaustPathConfig path;
    path.id = 1;
    path.geometry = config.exhaust;
    path.inheritsGlobalGeometry = true;
    for (const auto& cylinder : config.cylinders) path.cylinderIds.push_back(cylinder.id);
    config.exhaustPaths.push_back(std::move(path));
    normaliseEngineConfig(config);
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
    for (auto& cylinder : config.cylinders) {
        cylinder.bankOffsetDegrees = cylinder.id % 2U == 0U ? 45.0 : -45.0;
        cylinder.intakeValveCount = 1;
        cylinder.exhaustValveCount = 1;
    }
    for (auto& journal : config.crankJournals) {
        const auto cylinder = std::find_if(config.cylinders.begin(), config.cylinders.end(),
            [&journal](const CylinderConfig& item) { return item.crankJournalId == journal.id; });
        if (cylinder != config.cylinders.end())
            journal.angleDegrees = cylinder->crankOffsetDegrees - cylinder->bankOffsetDegrees;
    }
    if (config.banks.size() == 2) {
        config.banks[0].angleDegrees = -45.0;
        config.banks[1].angleDegrees = 45.0;
    }
    normaliseEngineConfig(config);
    return config;
}

EngineConfig makeDefaultFlatSix() {
    auto config = makeEngine("EL-36 F6", EngineLayout::flat, 6, { 1, 6, 2, 4, 3, 5 },
                             97.0, 81.5, 780.0, 7'600.0, 0.33,
                             { 270.0, 266.0, 12.3, 11.9, 106.0, 109.0 });
    config.frictionCoefficient = 0.13;
    config.intake.plenumVolumeLitres = 4.2;
    config.intake.throttleDiameterMm = 74.0;
    if (config.intakePaths.size() == 1) config.intakePaths.front().geometry = config.intake;
    config.exhaust = { 720.0, 42.0, 66.0, 0.22, 74.0 };
    if (config.exhaustPaths.size() == 1) config.exhaustPaths.front().geometry = config.exhaust;
    config.transmission = { { 3.82, 2.20, 1.52, 1.22, 1.02, 0.84 }, 3.44, 980.0 };
    config.vehicle = { 1'430.0, 0.30, 2.04, 0.325, 0.013 };
    normaliseEngineConfig(config);
    return config;
}

EngineConfig makeDefaultRadialFive() {
    EngineConfig config;
    config.name = "EL-R5 Radial";
    config.layout = EngineLayout::radial;
    config.firingOrder = { 1, 3, 5, 2, 4 };
    config.idleRpm = 640.0;
    config.redlineRpm = 3'200.0;
    config.ignition.revLimitRpm = 3'200.0;
    config.rotatingInertiaKgM2 = 0.86;
    config.frictionCoefficient = 0.19;
    config.octaneRating = 92.0;
    config.coolingEfficiency = 2.6;
    config.plenumVolumeLitres = 6.4;
    config.throttleDiameterMm = 82.0;
    config.bankAngleDegrees = 72.0;
    config.camshafts = { 246.0, 250.0, 11.6, 11.2, 106.0, 110.0 };
    config.exhaust = { 920.0, 54.0, 96.0, 0.08, 110.0 };
    config.transmission = { { 1.0 }, 1.0, 6'800.0 };
    config.vehicle = { 1'000.0, 0.30, 2.0, 0.320, 0.014 };
    config.thermal = { 820.0, 430.0, 0.24, 0.16, 0.92, 0.46 };
    config.crankJournals.push_back({ 1, 0.0, 63.5, 1 });
    constexpr std::uint32_t count = 5;
    for (std::uint32_t id = 1; id <= count; ++id) {
        CylinderConfig cylinder;
        cylinder.id = id;
        cylinder.boreMm = 114.0;
        cylinder.strokeMm = 127.0;
        cylinder.connectingRodMm = 228.0;
        cylinder.pistonMassGrams = 1'050.0;
        cylinder.compressionRatio = 7.0;
        cylinder.intakeValveCount = 1;
        cylinder.exhaustValveCount = 1;
        cylinder.intakeValveDiameterMm = 49.0;
        cylinder.exhaustValveDiameterMm = 42.0;
        cylinder.efficiencyOffset = (static_cast<double>(id) - 3.0) * 0.004;
        const auto order = std::find(config.firingOrder.begin(), config.firingOrder.end(), id);
        cylinder.crankOffsetDegrees = order == config.firingOrder.end() ? 0.0
            : static_cast<double>(std::distance(config.firingOrder.begin(), order)) * (720.0 / static_cast<double>(count));
        cylinder.crankJournalId = 1;
        cylinder.connectingRodType = id == 1 ? ConnectingRodType::master : ConnectingRodType::articulated;
        cylinder.masterCylinderId = id == 1 ? 0 : 1;
        cylinder.articulatedJournalRadiusMm = id == 1 ? 0.0 : 34.0;
        cylinder.articulatedJournalAngleDegrees = static_cast<double>(id - 1U) * 72.0;
        cylinder.bankOffsetDegrees = static_cast<double>(id - 1U) * (360.0 / static_cast<double>(count));
        config.cylinders.push_back(cylinder);
    }
    normaliseEngineConfig(config);
    return config;
}

std::vector<EngineConfig> makeBaseEnginePresets() {
    return { makeDefaultInlineTwo(), makeDefaultInlineFour(), makeDefaultInlineFive(), makeDefaultV6(),
             makeDefaultV8(), makeDefaultFlatSix(), makeDefaultRadialFive() };
}

double engineDisplacementLitres(const EngineConfig& config) noexcept {
    double cubicMillimetres = 0.0;
    for (const auto& cylinder : config.cylinders)
        cubicMillimetres += std::numbers::pi * cylinder.boreMm * cylinder.boreMm * 0.25 * cylinder.strokeMm;
    return cubicMillimetres / 1'000'000.0;
}

double intakeRunnerVolumeLitres(const CylinderConfig& cylinder, const IntakeConfig& fallback) noexcept {
    const auto inletDiameterMm = cylinder.intakeRunnerDiameterMm > 0.0
        ? cylinder.intakeRunnerDiameterMm : fallback.runnerDiameterMm;
    const auto outletDiameterMm = fallback.runnerPlenumDiameterMm > 0.0
        ? fallback.runnerPlenumDiameterMm : inletDiameterMm;
    const auto lengthMm = cylinder.intakeRunnerLengthMm > 0.0
        ? cylinder.intakeRunnerLengthMm : fallback.runnerLengthMm;
    const auto inletRadiusMm = std::max(0.0, inletDiameterMm * 0.5);
    const auto outletRadiusMm = std::max(0.0, outletDiameterMm * 0.5);
    // Exact circular-frustum volume with a radius-linear runner.
    const auto volumeMm3 = std::numbers::pi * std::max(0.0, lengthMm) / 3.0
        * (inletRadiusMm * inletRadiusMm
            + inletRadiusMm * outletRadiusMm
            + outletRadiusMm * outletRadiusMm);
    return std::max(0.001, volumeMm3 / 1'000'000.0);
}

double effectiveRotatingInertiaKgM2(const EngineConfig& config) noexcept {
    if (config.crankshafts.empty()) return std::max(0.001, config.rotatingInertiaKgM2);
    double inertia = 0.0;
    for (const auto& crankshaft : config.crankshafts)
        inertia += std::max(0.0, crankshaft.momentOfInertiaKgM2)
            * crankshaft.rotationRatio * crankshaft.rotationRatio;
    for (const auto& cylinder : config.cylinders) {
        if (cylinder.connectingRodMomentOfInertiaKgM2 > 0.0) {
            inertia += cylinder.connectingRodMomentOfInertiaKgM2;
        } else {
            const auto radiusM = cylinder.strokeMm * 0.0005;
            inertia += cylinder.connectingRodMassGrams * 0.001 * (2.0 / 3.0) * radiusM * radiusM;
        }
    }
    return inertia > 0.0 ? inertia : std::max(0.001, config.rotatingInertiaKgM2);
}

double valveFlowCoefficient(double liftMm, double fallback,
                            const std::vector<ValveFlowSample>& samples) noexcept {
    if (samples.empty()) return std::max(0.0, fallback);
    if (liftMm <= samples.front().liftMm) return std::max(0.0, samples.front().dischargeCoefficient);
    if (liftMm >= samples.back().liftMm) return std::max(0.0, samples.back().dischargeCoefficient);
    for (std::size_t index = 1; index < samples.size(); ++index) {
        if (liftMm <= samples[index].liftMm) {
            const auto span = std::max(1.0e-12, samples[index].liftMm - samples[index - 1].liftMm);
            const auto amount = std::clamp((liftMm - samples[index - 1].liftMm) / span, 0.0, 1.0);
            return std::max(0.0, std::lerp(samples[index - 1].dischargeCoefficient,
                                           samples[index].dischargeCoefficient, amount));
        }
    }
    return std::max(0.0, fallback);
}

ValveControlSample interpolateValveControl(const ValveControlConfig& control, double rpm,
                                           double load) noexcept {
    ValveControlSample result;
    result.rpm = rpm;
    result.load = load;
    if (!control.enabled || control.samples.empty()) return result;
    const auto [minimumRpm, maximumRpm] = std::minmax_element(control.samples.begin(), control.samples.end(),
        [](const auto& left, const auto& right) { return left.rpm < right.rpm; });
    const auto [minimumLoad, maximumLoad] = std::minmax_element(control.samples.begin(), control.samples.end(),
        [](const auto& left, const auto& right) { return left.load < right.load; });
    const auto clampedRpm = std::clamp(rpm, minimumRpm->rpm, maximumRpm->rpm);
    const auto clampedLoad = std::clamp(load, minimumLoad->load, maximumLoad->load);
    auto lowerRpm = minimumRpm->rpm;
    auto upperRpm = maximumRpm->rpm;
    auto lowerLoad = minimumLoad->load;
    auto upperLoad = maximumLoad->load;
    for (const auto& sample : control.samples) {
        if (sample.rpm <= clampedRpm) lowerRpm = std::max(lowerRpm, sample.rpm);
        if (sample.rpm >= clampedRpm) upperRpm = std::min(upperRpm, sample.rpm);
        if (sample.load <= clampedLoad) lowerLoad = std::max(lowerLoad, sample.load);
        if (sample.load >= clampedLoad) upperLoad = std::min(upperLoad, sample.load);
    }
    const auto sampleAt = [&control](double sampleRpm, double sampleLoad) noexcept
        -> const ValveControlSample* {
        const auto item = std::find_if(control.samples.begin(), control.samples.end(),
            [sampleRpm, sampleLoad](const ValveControlSample& sample) {
                return std::abs(sample.rpm - sampleRpm) <= 1.0e-9
                    && std::abs(sample.load - sampleLoad) <= 1.0e-12;
            });
        return item == control.samples.end() ? nullptr : &*item;
    };
    const auto* lowerLower = sampleAt(lowerRpm, lowerLoad);
    const auto* upperLower = sampleAt(upperRpm, lowerLoad);
    const auto* lowerUpper = sampleAt(lowerRpm, upperLoad);
    const auto* upperUpper = sampleAt(upperRpm, upperLoad);
    if (lowerLower != nullptr && upperLower != nullptr
            && lowerUpper != nullptr && upperUpper != nullptr) {
        const auto rpmAmount = upperRpm > lowerRpm
            ? (clampedRpm - lowerRpm) / (upperRpm - lowerRpm) : 0.0;
        const auto loadAmount = upperLoad > lowerLoad
            ? (clampedLoad - lowerLoad) / (upperLoad - lowerLoad) : 0.0;
        const auto bilinear = [rpmAmount, loadAmount](double ll, double ul,
                                                       double lu, double uu) noexcept {
            return std::lerp(std::lerp(ll, ul, rpmAmount),
                             std::lerp(lu, uu, rpmAmount), loadAmount);
        };
        result.intakeAdvanceDegrees = bilinear(lowerLower->intakeAdvanceDegrees,
            upperLower->intakeAdvanceDegrees, lowerUpper->intakeAdvanceDegrees,
            upperUpper->intakeAdvanceDegrees);
        result.exhaustAdvanceDegrees = bilinear(lowerLower->exhaustAdvanceDegrees,
            upperLower->exhaustAdvanceDegrees, lowerUpper->exhaustAdvanceDegrees,
            upperUpper->exhaustAdvanceDegrees);
        result.liftMultiplier = bilinear(lowerLower->liftMultiplier,
            upperLower->liftMultiplier, lowerUpper->liftMultiplier,
            upperUpper->liftMultiplier);
        return result;
    }

    // Schema-v1 accepted sparse point clouds. Retain a deterministic fallback
    // for those files, but clamp at the calibrated axes so extrapolation can
    // never drift beyond the edge cells of a tuning table.
    // A sparse table is a SCHEDULE, so interpolate along it in rpm rather than
    // averaging it isotropically.
    //
    // This used to be inverse-distance (Shepard) weighting over all samples in a
    // normalised rpm/load plane. That is wrong for a cam-phaser map in a way
    // that is invisible until it is measured: Shepard weighting is not monotone
    // between neighbours, so every sample pulls on every query, and a schedule
    // whose author wrote "advance hard in the midrange, then fall back at the
    // top" instead gets the midrange value smeared across the whole upper range.
    // Measured on the K20A, whose sport_na table asks for 28 deg of intake
    // advance at 6000 rpm and 12 deg at 8500: the old fallback still delivered
    // 26 deg at 5000 and never came back, holding intake valve closing ~30 deg
    // early through the entire power band. Disabling the phaser outright was
    // worth +16 % peak power, which is the size of the error being corrected
    // here. See docs/physics-audit.md.
    //
    // Piecewise-linear in rpm is monotone between authored points, reproduces
    // the authored value exactly at each of them, and cannot extrapolate past
    // the ends (rpm is clamped above). Samples sharing an rpm are blended by
    // load first, so a table that does vary load at fixed rpm still works.
    auto lowerIndex = control.samples.size();
    auto upperIndex = control.samples.size();
    for (std::size_t index = 0; index < control.samples.size(); ++index) {
        const auto& sample = control.samples[index];
        if (sample.rpm <= clampedRpm
            && (lowerIndex == control.samples.size()
                || sample.rpm > control.samples[lowerIndex].rpm
                || (sample.rpm == control.samples[lowerIndex].rpm
                    && std::abs(sample.load - clampedLoad)
                        < std::abs(control.samples[lowerIndex].load - clampedLoad))))
            lowerIndex = index;
        if (sample.rpm >= clampedRpm
            && (upperIndex == control.samples.size()
                || sample.rpm < control.samples[upperIndex].rpm
                || (sample.rpm == control.samples[upperIndex].rpm
                    && std::abs(sample.load - clampedLoad)
                        < std::abs(control.samples[upperIndex].load - clampedLoad))))
            upperIndex = index;
    }
    if (lowerIndex == control.samples.size()) lowerIndex = upperIndex;
    if (upperIndex == control.samples.size()) upperIndex = lowerIndex;
    if (lowerIndex == control.samples.size()) return result;
    const auto& lower = control.samples[lowerIndex];
    const auto& upper = control.samples[upperIndex];
    const auto amount = upper.rpm > lower.rpm
        ? (clampedRpm - lower.rpm) / (upper.rpm - lower.rpm) : 0.0;
    result.intakeAdvanceDegrees = std::lerp(lower.intakeAdvanceDegrees,
        upper.intakeAdvanceDegrees, amount);
    result.exhaustAdvanceDegrees = std::lerp(lower.exhaustAdvanceDegrees,
        upper.exhaustAdvanceDegrees, amount);
    result.liftMultiplier = std::lerp(lower.liftMultiplier, upper.liftMultiplier, amount);
    return result;
}

void normaliseEngineConfig(EngineConfig& config) {
    // Schema 2 added authored exhaust DAGs/topology provenance; schema 3 adds
    // explicit valve geometry; schema 4 adds SI outlet/observer coordinates;
    // schema 5 adds authored vehicle layout and longitudinal load-transfer
    // geometry; schema 6 makes the overrun strategy explicit and adds local
    // induction/flammability/quench calibration; schema 7 adds explicit
    // cellular catalyst-substrate geometry. Older documents migrate to the
    // documented defaults below.
    const auto sourceSchemaVersion = config.schemaVersion;
    if (config.schemaVersion < currentEngineSchemaVersion)
        config.schemaVersion = currentEngineSchemaVersion;
    // Schema <=5 only had `enabled`, fuel fraction and an optional pulse rate.
    // Programmatic callers also commonly still set those three fields, so the
    // same deterministic migration is intentionally accepted there.  Once a
    // non-clean strategy is present it is authoritative.
    if (config.exhaustAfterfire.strategy
            == ExhaustAfterfireStrategy::cleanDfco
        && config.exhaustAfterfire.enabled) {
        config.exhaustAfterfire.strategy =
            config.exhaustAfterfire.overrunPulseHz > 0.0
            ? ExhaustAfterfireStrategy::discreteAfterfire
            : ExhaustAfterfireStrategy::continuousAntiLag;
    }
    if (sourceSchemaVersion < 6
        && !config.exhaustAfterfire.enabled) {
        config.exhaustAfterfire.strategy =
            ExhaustAfterfireStrategy::cleanDfco;
    }
    config.exhaustAfterfire.enabled = afterfireRetainsFuel(
        config.exhaustAfterfire.strategy);
    // `intake` is the canonical representation. Legacy scalar fields remain
    // mirrored so schema-v1 files and old catalog overrides remain compatible.
    if (config.intake.plenumVolumeLitres == IntakeConfig {}.plenumVolumeLitres
        && config.plenumVolumeLitres != EngineConfig {}.plenumVolumeLitres)
        config.intake.plenumVolumeLitres = config.plenumVolumeLitres;
    if (config.intake.throttleDiameterMm == IntakeConfig {}.throttleDiameterMm
        && config.throttleDiameterMm != EngineConfig {}.throttleDiameterMm)
        config.intake.throttleDiameterMm = config.throttleDiameterMm;
    config.plenumVolumeLitres = config.intake.plenumVolumeLitres;
    config.throttleDiameterMm = config.intake.throttleDiameterMm;
    if (!(config.acousticObserver.soundSpeedMps > 0.0)
        || !std::isfinite(config.acousticObserver.soundSpeedMps)) {
        constexpr double dryAirGamma = 1.4;
        constexpr double dryAirGasConstantJPerKgK = 287.05;
        config.acousticObserver.soundSpeedMps = std::sqrt(dryAirGamma
            * dryAirGasConstantJPerKgK * (config.ambientTemperatureC + 273.15));
    }

    if (config.crankshafts.empty()) {
        CrankshaftConfig crankshaft;
        crankshaft.momentOfInertiaKgM2 = config.rotatingInertiaKgM2;
        crankshaft.inheritsLegacyInertia = true;
        config.crankshafts.push_back(crankshaft);
    } else {
        // Explicit topology is authoritative. The scalar is a schema-v1
        // migration field and must never erase a deliberately configured
        // crankshaft inertia.
        if (config.crankshafts.size() == 1
                && config.crankshafts.front().inheritsLegacyInertia)
            config.crankshafts.front().momentOfInertiaKgM2 = config.rotatingInertiaKgM2;
        double configuredInertia = 0.0;
        double missingRatioSquared = 0.0;
        for (const auto& crankshaft : config.crankshafts) {
            const auto ratioSquared = crankshaft.rotationRatio * crankshaft.rotationRatio;
            if (crankshaft.momentOfInertiaKgM2 > 0.0)
                configuredInertia += crankshaft.momentOfInertiaKgM2 * ratioSquared;
            else
                missingRatioSquared += ratioSquared;
        }
        const auto inertiaPerMissingRatio = missingRatioSquared > 0.0
            ? std::max(0.001, (config.rotatingInertiaKgM2 - configuredInertia)
                / missingRatioSquared)
            : 0.0;
        for (auto& crankshaft : config.crankshafts)
            if (!(crankshaft.momentOfInertiaKgM2 > 0.0))
                crankshaft.momentOfInertiaKgM2 = inertiaPerMissingRatio;
        config.rotatingInertiaKgM2 = 0.0;
        for (const auto& crankshaft : config.crankshafts)
            config.rotatingInertiaKgM2 += crankshaft.momentOfInertiaKgM2
                * crankshaft.rotationRatio * crankshaft.rotationRatio;
    }
    if (config.crankJournals.empty()) {
        for (auto& cylinder : config.cylinders) {
            cylinder.crankJournalId = cylinder.id;
            config.crankJournals.push_back({ cylinder.id,
                cylinder.crankOffsetDegrees - cylinder.bankOffsetDegrees,
                cylinder.strokeMm * 0.5, config.crankshafts.front().id });
        }
    }
    for (auto& journal : config.crankJournals) {
        if (journal.crankshaftId == 0) journal.crankshaftId = config.crankshafts.front().id;
    }
    if (config.intakePaths.empty()) {
        config.intakePaths.push_back(makeDefaultIntakePath(config));
    } else if (config.intakePaths.size() == 1) {
        // As with crankshafts, a serialised path is an explicit topology. Keep
        // the legacy/global view mirrored from it instead of silently
        // overwriting the path during every validation or hot reload.
        if (config.intakePaths.front().inheritsGlobalGeometry)
            config.intakePaths.front().geometry = config.intake;
        else
            config.intake = config.intakePaths.front().geometry;
        config.plenumVolumeLitres = config.intake.plenumVolumeLitres;
        config.throttleDiameterMm = config.intake.throttleDiameterMm;
    }
    for (auto& bank : config.banks) {
        if (bank.intakeId == 0 && !config.intakePaths.empty()) bank.intakeId = config.intakePaths.front().id;
    }
    // A zero bank ID is legacy/unspecified and can be inferred without
    // ambiguity from the authoritative membership lists. Non-zero IDs remain
    // untouched so validation can expose contradictory authored topology.
    for (auto& cylinder : config.cylinders) {
        if (cylinder.bankId != 0) continue;
        const auto bank = std::find_if(config.banks.begin(), config.banks.end(),
            [&cylinder](const auto& candidate) {
                return std::find(candidate.cylinderIds.begin(), candidate.cylinderIds.end(), cylinder.id)
                    != candidate.cylinderIds.end();
            });
        if (bank != config.banks.end()) cylinder.bankId = bank->id;
    }
    if (config.exhaustPaths.empty()) {
        config.exhaustPaths.push_back(makeDefaultExhaustPath(config));
    } else if (config.exhaustPaths.size() == 1) {
        // Generated preset paths continue to mirror the legacy/global
        // geometry. A decoded or authored path is explicit and authoritative.
        if (config.exhaustPaths.front().inheritsGlobalGeometry)
            config.exhaustPaths.front().geometry = config.exhaust;
        else
            config.exhaust = config.exhaustPaths.front().geometry;
    }
}

double valveLiftMm(double crankAngleDegrees, double centerlineDegrees,
                   double durationDegrees, double maximumLiftMm) noexcept {
    const auto wrapped = std::remainder(crankAngleDegrees - centerlineDegrees, 720.0);
    const auto halfDuration = std::max(1.0, durationDegrees * 0.5);
    if (std::abs(wrapped) >= halfDuration) return 0.0;
    const auto phase = (wrapped + halfDuration) / durationDegrees;
    return std::max(0.0, maximumLiftMm * 0.5 * (1.0 - std::cos(phase * 2.0 * std::numbers::pi)));
}

double profiledValveLiftMm(double crankAngleDegrees, double centerlineDegrees,
                           double durationDegrees, double maximumLiftMm,
                           const std::vector<ValveLiftSample>& samples) noexcept {
    if (samples.size() < 2) return valveLiftMm(crankAngleDegrees, centerlineDegrees, durationDegrees, maximumLiftMm);
    const auto relativeAngle = std::remainder(crankAngleDegrees - centerlineDegrees, 720.0);
    const auto halfDuration = std::max(1.0, durationDegrees * 0.5);
    if (std::abs(relativeAngle) >= halfDuration) return 0.0;
    const auto firstAngle = samples.front().angleDegrees;
    const auto lastAngle = samples.back().angleDegrees;
    if (relativeAngle <= firstAngle) return std::clamp(samples.front().liftMm, 0.0, maximumLiftMm);
    if (relativeAngle >= lastAngle) return std::clamp(samples.back().liftMm, 0.0, maximumLiftMm);
    for (std::size_t index = 1; index < samples.size(); ++index) {
        const auto& left = samples[index - 1];
        const auto& right = samples[index];
        if (relativeAngle <= right.angleDegrees) {
            const auto span = std::max(1.0e-9, right.angleDegrees - left.angleDegrees);
            const auto t = std::clamp((relativeAngle - left.angleDegrees) / span, 0.0, 1.0);
            return std::clamp(std::lerp(left.liftMm, right.liftMm, t), 0.0, maximumLiftMm);
        }
    }
    return 0.0;
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
    const auto validPoint = [&inRange](const AcousticPoint3M& point) {
        return inRange(point.x, -100.0, 100.0)
            && inRange(point.y, -100.0, 100.0)
            && inRange(point.z, -100.0, 100.0);
    };
    if (config.schemaVersion < minimumSupportedEngineSchemaVersion
            || config.schemaVersion > currentEngineSchemaVersion)
        return "Unsupported engine schema version";
    if (config.name.empty() || config.name.size() > 128) return "Engine name must contain between 1 and 128 bytes";
    if (config.cycle != EngineCycle::fourStroke)
        return "Only four-stroke engines are currently supported";
    if (config.fuel == FuelType::diesel
            && config.injection.mode != InjectionMode::direct)
        return "Compression-ignition diesel engines require direct injection";
    if (config.fuelProperties.name.empty() || config.fuelProperties.name.size() > 128
        || !inRange(config.fuelProperties.lowerHeatingValueMjPerKg, 10.0, 60.0)
        || !inRange(config.fuelProperties.densityKgPerL, 0.30, 1.50)
        || !inRange(config.fuelProperties.stoichiometricAirFuelRatio, 5.0, 25.0)
        || !inRange(config.fuelProperties.molarMassGramsPerMole, 20.0, 300.0)
        || !inRange(config.fuelProperties.oxygenMolesPerFuelMole, 1.0, 40.0)
        || !inRange(config.fuelProperties.productMolesPerFuelMole, 1.0, 60.0)
        || !inRange(config.fuelProperties.laminarFlameSpeedMps,
            config.fuel == FuelType::diesel ? 0.0 : 0.05, 2.0)
        || !inRange(config.fuelProperties.turbulenceFlameSpeedGain, 0.0, 10.0)
        || (config.fuel == FuelType::gasoline
            && config.fuelProperties.cetaneNumber != 0.0)
        || (config.fuel == FuelType::diesel
            && !inRange(config.fuelProperties.cetaneNumber, 30.0, 80.0)))
        return "Fuel properties must define finite chemistry, ignition quality and thermodynamic values";
    const auto chemistryStoichiometricAfr = config.fuelProperties.oxygenMolesPerFuelMole * 31.9988
        / config.fuelProperties.molarMassGramsPerMole / 0.232;
    if (std::abs(config.fuelProperties.stoichiometricAirFuelRatio / chemistryStoichiometricAfr - 1.0) > 0.25)
        return "Fuel stoichiometric AFR is inconsistent with its molar oxygen requirement";
    if (config.cylinders.empty() || config.cylinders.size() > 32 || config.firingOrder.size() != config.cylinders.size())
        return "Firing order must reference between 1 and 32 cylinders";
    if (!inRange(config.idleRpm, 200.0, 3'000.0)
        || !inRange(config.redlineRpm, config.idleRpm + 100.0, 20'000.0)
        || !inRange(config.rotatingInertiaKgM2, 0.01, 5.0)
        || !inRange(config.frictionCoefficient, 0.0, 1.0)
        || (config.fuel == FuelType::gasoline
            && !inRange(config.octaneRating, 70.0, 130.0))
        || !inRange(config.ambientPressureKpa, 50.0, 120.0)
        || !inRange(config.ambientTemperatureC, -50.0, 60.0)
        || !inRange(config.coolingEfficiency, 0.1, 5.0)
        || !inRange(config.plenumVolumeLitres, 0.1, 50.0)
        || !inRange(config.throttleDiameterMm, 15.0, 150.0)
        || !inRange(config.bankAngleDegrees, 0.0, 180.0)
        || !validPoint(config.acousticObserver.leftMicrophoneM)
        || !validPoint(config.acousticObserver.rightMicrophoneM)
        || !(config.acousticObserver.soundSpeedMps == 0.0
            || inRange(config.acousticObserver.soundSpeedMps, 250.0, 450.0))
        // Zero keeps the authored microphone positions; anything else places
        // the listener at a real, human distance from the engine.
        || !(config.acousticObserver.listeningDistanceM == 0.0
            || inRange(config.acousticObserver.listeningDistanceM, 0.5, 200.0))
        || !inRange(config.forcedInduction.pressureRatio, 1.0, 3.5)
        || !inRange(config.forcedInduction.fullBoostRpm, 200.0, 20'000.0)
        || !inRange(config.forcedInduction.compressorEfficiency, 0.35, 0.95)
        || !inRange(config.forcedInduction.chargeTemperatureRiseC, 0.0, 160.0)
        || !inRange(config.forcedInduction.turbineEfficiency, 0.20, 0.95)
        || !inRange(config.forcedInduction.shaftInertiaKgM2, 0.000001, 0.1)
        || !inRange(config.forcedInduction.wastegatePressureRatio, 1.0, 3.5)
        || !inRange(config.forcedInduction.designShaftSpeedRpm, 10'000.0, 400'000.0)
        || !inRange(config.forcedInduction.bearingFrictionPowerWatts, 0.0, 20'000.0)
        || !inRange(config.forcedInduction.turbineFlowAreaMm2, 10.0, 20'000.0)
        || !inRange(config.forcedInduction.wastegateFlowAreaMm2, 0.0, 20'000.0)
        || !inRange(config.thermal.coolantMassKjPerC, 20.0, 5'000.0)
        || !inRange(config.thermal.oilMassKjPerC, 10.0, 2'500.0)
        || !inRange(config.thermal.coolantHeatShare, 0.0, 1.0)
        || !inRange(config.thermal.oilHeatShare, 0.0, 1.0)
        || !inRange(config.thermal.coolingPowerKwPerC, 0.02, 20.0)
        || !inRange(config.thermal.oilCoolingPowerKwPerC, 0.01, 10.0)
        || !inRange(config.camshafts.intakeDurationDegrees, 1.0, 720.0)
        || !inRange(config.camshafts.exhaustDurationDegrees, 1.0, 720.0)
        || !inRange(config.camshafts.intakeLiftMm, 0.0, 30.0)
        || !inRange(config.camshafts.exhaustLiftMm, 0.0, 30.0)
        || !inRange(config.camshafts.intakeCenterlineDegrees, 0.0, 720.0)
        || !inRange(config.camshafts.exhaustCenterlineDegrees, 0.0, 720.0)
        || !inRange(config.camshafts.intakeFlowCoefficient, 0.05, 1.50)
        || !inRange(config.camshafts.exhaustFlowCoefficient, 0.05, 1.50)
        || !inRange(config.exhaust.primaryLengthMm, 100.0, 3'000.0)
        || !inRange(config.exhaust.primaryDiameterMm, 15.0, 200.0)
        || !inRange(config.exhaust.collectorDiameterMm, 15.0, 300.0)
        || !inRange(config.exhaust.outletDiameterMm, 15.0, 300.0)
        || !inRange(config.exhaust.mufflerRestriction, 0.0, 1.0)
        || !inRange(config.exhaust.collectorVolumeLitres, 0.05, 200.0)
        || !inRange(config.exhaust.outletDischargeCoefficient, 0.02, 1.5)
        || !inRange(config.exhaust.mufflerChamberDiameterMm, 0.0, 600.0)
        || !inRange(config.exhaust.mufflerChamberLengthMm, 0.0, 3'000.0)
        || !inRange(config.exhaust.mufflerPackingFlowResistivityPaSPerM2, 0.0, 200'000.0)
        || !inRange(config.exhaust.mufflerPackingThicknessMm, 0.0, 300.0)
        || !inRange(config.exhaust.mufflerPerforatedOpenAreaRatio, 0.0, 1.0)
        || !validMufflerPacking(config.exhaust)
        || !inRange(config.transmission.finalDriveRatio, 1.0, 8.0)
        || !inRange(config.transmission.maxClutchTorqueNm, 10.0, 10'000.0)
        || !inRange(config.transmission.drivelineEfficiency, 0.2, 1.0)
        || !inRange(config.transmission.drivenWheelInertiaKgM2, 0.0, 200.0)
        || !inRange(config.transmission.clutchLockSpeedRpm, 0.1, 500.0)
        || !inRange(config.transmission.shiftDurationSeconds, 0.01, 3.0)
        || !inRange(config.transmission.automaticUpshiftRpm, 250.0, 50'000.0)
        || !inRange(config.transmission.automaticDownshiftRpm, 100.0, 30'000.0)
        || !inRange(config.transmission.reverseRatio, 0.1, 10.0)
        || !inRange(config.transmission.gearboxInputInertiaKgM2, 0.0, 20.0)
        || !inRange(config.transmission.differentialInertiaKgM2, 0.0, 50.0)
        || !inRange(config.transmission.clutchThermalCapacityJPerC, 100.0, 2'000'000.0)
        || !inRange(config.transmission.clutchCoolingWPerC, 0.0, 2'000.0)
        || !inRange(config.transmission.clutchFadeStartTemperatureC, 50.0, 800.0)
        || !inRange(config.transmission.clutchFailureTemperatureC,
                    config.transmission.clutchFadeStartTemperatureC + 1.0, 1'200.0)
        || !inRange(config.transmission.shiftTorqueCutFraction, 0.0, 1.0)
        || !inRange(config.vehicle.massKg, 50.0, 20'000.0)
        || !inRange(config.vehicle.dragCoefficient, 0.05, 2.0)
        || !inRange(config.vehicle.frontalAreaM2, 0.1, 20.0)
        || !inRange(config.vehicle.tireRadiusM, 0.05, 2.0)
        || !inRange(config.vehicle.rollingResistanceCoefficient, 0.0, 0.20)
        || !inRange(config.vehicle.tireFrictionCoefficient, 0.05, 3.0)
        || !inRange(config.vehicle.drivenAxleWeightFraction, 0.05, 1.0)
        || !inRange(config.vehicle.wheelbaseM, 0.5, 10.0)
        || !inRange(config.vehicle.centerOfGravityHeightM, 0.05, 3.0)
        || config.vehicle.centerOfGravityHeightM >= config.vehicle.wheelbaseM
        || !inRange(config.vehicle.maximumBrakeForceN, 0.0, 200'000.0)
        || !inRange(config.combustionCalibration.baseIgnitionDelaySeconds, 0.0, 0.02)
        || !inRange(config.combustionCalibration.ignitionDelayTemperatureExponent, 0.0, 5.0)
        || !inRange(config.combustionCalibration.ignitionDelayPressureExponent, 0.0, 3.0)
        || !inRange(config.combustionCalibration.wallHeatTransferCoefficientWPerK, 0.0, 5'000.0)
        || !inRange(config.combustionCalibration.residualDilutionSensitivity, 0.0, 3.0)
        || !inRange(config.combustionCalibration.chamberTurbulenceIntensityRatio, 0.1, 4.0)
        || config.combustionCalibration.ignitionSiteCount < 1
        || config.combustionCalibration.ignitionSiteCount > 4
        || !inRange(config.combustionCalibration.compressionIgnitionDelayScale, 0.2, 5.0)
        || !inRange(config.combustionCalibration.compressionIgnitionMixingTimeSeconds,
                    0.00005, 0.02)
        || !inRange(config.combustionCalibration.compressionIgnitionPremixedFraction,
                    0.0, 0.8)
        || !inRange(config.combustionCalibration
                        .cycleVariationCoefficientOfVariation,
                    0.0, 0.20)
        || !inRange(config.combustionCalibration.cycleVariationCorrelation,
                    0.0, 0.98)
        || !inRange(config.exhaustAfterfire.ignitionTemperatureK,
                    500.0, 2'000.0)
        || !inRange(config.exhaustAfterfire.reactionTimeConstantSeconds,
                    0.0001, 1.0)
        || !inRange(config.exhaustAfterfire.reactionEfficiency, 0.0, 1.0)
        || !inRange(config.exhaustAfterfire.overrunFuelFraction, 0.0, 0.25)
        || !inRange(config.exhaustAfterfire.overrunMinimumRpm, 500.0, 20'000.0)
        || !inRange(config.exhaustAfterfire.overrunMaximumThrottle, 0.0, 0.20)
        // Zero means "every cycle", the historical continuous strategy. The
        // upper bound is where chopping stops being audible as separate events
        // and merges back into the steady burn it exists to break up.
        || !inRange(config.exhaustAfterfire.overrunPulseHz, 0.0, 60.0)
        || !inRange(config.exhaustAfterfire.overrunPulseDutyCycle, 0.02, 1.0)
        || !inRange(config.exhaustAfterfire.overrunPulseTimingVariation,
                    0.0, 0.45)
        || !inRange(config.exhaustAfterfire.inductionTimeSeconds,
                    0.0001, 0.100)
        || !inRange(config.exhaustAfterfire.minimumEquivalenceRatio,
                    0.05, 1.0)
        || !inRange(config.exhaustAfterfire.maximumEquivalenceRatio,
                    1.0, 5.0)
        || config.exhaustAfterfire.minimumEquivalenceRatio
            >= config.exhaustAfterfire.maximumEquivalenceRatio
        || !inRange(config.exhaustAfterfire.quenchTemperatureK,
                    250.0, 1'500.0)
        || (afterfireRetainsFuel(config.exhaustAfterfire.strategy)
            && !(config.exhaustAfterfire.overrunFuelFraction > 0.0))
        || (config.exhaustAfterfire.strategy
                == ExhaustAfterfireStrategy::discreteAfterfire
            && !(config.exhaustAfterfire.overrunPulseHz > 0.0))
        || (config.exhaustAfterfire.strategy
                == ExhaustAfterfireStrategy::discreteAfterfire
            && config.exhaustAfterfire.overrunFuelFraction
                > config.exhaustAfterfire.overrunPulseDutyCycle)
        || !inRange(config.runnerAcoustics.dampingRatio, 0.01, 2.0)
        || !inRange(config.runnerAcoustics.couplingGain, 0.0, 2.0)
        || !inRange(config.runnerAcoustics.maximumPressureAmplitudeKpa, 0.1, 200.0)
        || config.forcedInduction.compressorBladeCount > 100
        || config.forcedInduction.turbineBladeCount > 100
        || config.forcedInduction.superchargerLobeCount > 20
        || !inRange(config.forcedInduction.superchargerDriveRatio, 0.1, 30.0)
        || !inRange(config.forcedInduction.compressorInducerDiameterMm, 0.0, 500.0)
        || !inRange(config.forcedInduction.turbineExducerDiameterMm, 0.0, 500.0)
        || !inRange(config.forcedInduction.blowOffValveFlowAreaMm2, 0.0, 5'000.0)
        || !inRange(config.forcedInduction.blowOffValveOpeningPressureRatio, 1.001, 3.0)
        || !inRange(config.forcedInduction.blowOffValveDischargeCoefficient, 0.05, 1.5)
        || !inRange(config.forcedInduction.tonalAcousticEfficiency, 0.0, 0.01)
        || !inRange(config.forcedInduction.turbulentJetNoiseCoefficient, 0.0, 0.1)
        || !inRange(config.intake.plenumVolumeLitres, 0.1, 50.0)
        || !inRange(config.intake.throttleDiameterMm, 15.0, 150.0)
        || config.intake.throttleCount < 1 || config.intake.throttleCount > 16
        || !inRange(config.intake.throttleDischargeCoefficient, 0.05, 1.5)
        || !inRange(config.intake.runnerLengthMm, 20.0, 2'000.0)
        || !inRange(config.intake.runnerDiameterMm, 10.0, 150.0)
        || !(config.intake.runnerPlenumDiameterMm == 0.0
            || inRange(config.intake.runnerPlenumDiameterMm, 10.0, 150.0))
        || !inRange(config.intake.airboxVolumeLitres, 0.0, 100.0)
        || !inRange(config.intake.inletDuctLengthMm, 0.0, 5'000.0)
        || !inRange(config.intake.inletDuctDiameterMm, 0.0, 500.0)
        || !inRange(config.intake.bellmouthDiameterMm, 0.0, 1'000.0)
        || !inRange(config.intake.idleBypassAreaMm2, 0.0, 1'000.0)
        || !inRange(config.intake.throttleGamma, 0.2, 5.0)
        || !inRange(config.ignition.revLimitRpm, config.idleRpm + 100.0, 25'000.0)
        || !inRange(config.ignition.limiterDurationSeconds, 0.005, 2.0)
        || !inRange(config.injection.startAngleDegrees, 0.0, 720.0)
        || !inRange(config.injection.endAngleDegrees, 0.0, 720.0)
        || std::abs(config.injection.startAngleDegrees - config.injection.endAngleDegrees) < 1.0
        || !inRange(config.injection.injectorFlowMgPerSecond, 10.0, 500'000.0)
        || !inRange(config.injection.fuelTemperatureC, -50.0, 180.0)
        || !inRange(config.injection.railPressureBar, 1.2, 3'000.0)
        || !inRange(config.injection.referencePressureBar, 1.2, 3'000.0)
        || !inRange(config.injection.wallFilmFraction, 0.0, 0.98)
        || !inRange(config.injection.vaporisationTimeConstantSeconds, 0.001, 2.0)
        || !inRange(config.injection.latentHeatKjPerKg, 10.0, 1'000.0)
        || !inRange(config.injection.directChargeCoolingEfficiency, 0.0, 1.0)
        || !inRange(config.injection.portChargeCoolingEfficiency, 0.0, 1.0)
        || !inRange(config.injection.directSprayVaporisationTimeConstantSeconds,
                    0.0, 0.02)
        || !inRange(config.injection.directSprayEntrainmentTimeConstantSeconds,
                    0.0, 0.02)
        || !inRange(config.solver.mechanicalFrequencyHz, 240.0, 50'000.0)
        || !inRange(config.solver.maximumMechanicalFrequencyHz, config.solver.mechanicalFrequencyHz, 100'000.0)
        || !inRange(config.solver.maximumCrankDegreesPerStep, 0.1, 30.0)
        || config.solver.gasSubsteps < 1 || config.solver.gasSubsteps > 32)
        return "Engine configuration contains non-finite or physically invalid values";
    if (config.injection.fullLoadFuelLimit.size() > 64
            || (config.fuel != FuelType::diesel
                && !config.injection.fullLoadFuelLimit.empty()))
        return "Full-load fuel quantity limits are only valid for compression-ignition engines";
    if (config.structuralNvh.modes.empty()) {
        if (config.structuralNvh.provenance
                != StructuralNvhProvenance::estimatedFamily
            || !config.structuralNvh.source.empty())
            return "Empty structural NVH data must retain estimated-family provenance";
    } else {
        if ((config.structuralNvh.provenance
                != StructuralNvhProvenance::estimatedFamily
             && config.structuralNvh.provenance
                != StructuralNvhProvenance::calculatedGeometry
             && config.structuralNvh.provenance
                != StructuralNvhProvenance::measured)
            || config.structuralNvh.modes.size() > 64
            || config.structuralNvh.source.empty()
            || config.structuralNvh.source.size() > 1'024)
            return "Authored structural NVH modes require a bounded auditable source";
        for (const auto& mode : config.structuralNvh.modes) {
            if ((mode.drive != StructuralModeDrive::headGas
                    && mode.drive
                        != StructuralModeDrive::bearingAxial
                    && mode.drive
                        != StructuralModeDrive::bearingLateral
                    && mode.drive != StructuralModeDrive::torsion)
                || mode.name.empty() || mode.name.size() > 128
                || !inRange(mode.frequencyHz, 10.0, 20'000.0)
                || !inRange(mode.dampingRatio, 0.001, 0.50)
                || !inRange(mode.modalMassKg, 0.01, 10'000.0)
                || !inRange(mode.radiatingAreaM2, 0.0001, 100.0)
                || !inRange(mode.radiationEfficiency, 0.0, 1.0)
                || !inRange(mode.surfaceVelocityRmsScale, 0.001, 1.0)
                || !inRange(mode.torqueRadiusM, 0.001, 2.0)
                || mode.cylinderParticipation.size()
                    != config.cylinders.size())
                return "Structural NVH modes must define finite SI modal data and one participation per cylinder";
            auto maximumParticipation = 0.0;
            for (const auto participation :
                 mode.cylinderParticipation) {
                if (!inRange(participation, -1.0, 1.0))
                    return "Structural NVH participation must be finite and antinode-normalised";
                maximumParticipation = std::max(
                    maximumParticipation, std::abs(participation));
            }
            if (maximumParticipation < 1.0e-6)
                return "Structural NVH modes must couple to at least one cylinder";
        }
    }
    auto previousFuelLimitRpm = -1.0;
    for (const auto& sample : config.injection.fullLoadFuelLimit) {
        if (!inRange(sample.rpm, 0.0, 25'000.0)
                || !inRange(sample.milligramsPerCycle, 0.0, 1'000.0)
                || sample.rpm <= previousFuelLimitRpm)
            return "Full-load fuel quantity limits must be finite and ordered by RPM";
        previousFuelLimitRpm = sample.rpm;
    }
    const auto maximumCalibratedRpm = std::max(config.redlineRpm, config.ignition.revLimitRpm);
    const auto requiredAngleFrequency = maximumCalibratedRpm * 6.0
        / config.solver.maximumCrankDegreesPerStep;
    const auto requiredSolverFrequency = std::max(requiredAngleFrequency,
        config.solver.mechanicalFrequencyHz * static_cast<double>(config.solver.gasSubsteps));
    if (config.solver.maximumMechanicalFrequencyHz + 1.0e-9 < requiredSolverFrequency)
        return "Solver maximum frequency cannot satisfy the configured crank-angle resolution at the rev limit";
    if (config.transmission.gearRatios.empty() || config.transmission.gearRatios.size() > 12)
        return "Transmission must contain between 1 and 12 forward gears";
    for (const auto gearRatio : config.transmission.gearRatios)
        if (!inRange(gearRatio, 0.05, 10.0)) return "Transmission gear ratios must be finite and positive";
    if (config.transmission.automaticDownshiftRpm >= config.transmission.automaticUpshiftRpm)
        return "Automatic transmission downshift RPM must be lower than upshift RPM";
    if (std::abs(config.plenumVolumeLitres - config.intake.plenumVolumeLitres) > 1.0e-9
        || std::abs(config.throttleDiameterMm - config.intake.throttleDiameterMm) > 1.0e-9)
        return "Legacy plenum and throttle fields must match the canonical intake configuration";
    if (config.crankshafts.empty() || config.crankshafts.size() > 8)
        return "Mechanical topology must define between 1 and 8 crankshafts";
    std::unordered_set<std::uint32_t> crankshaftIds;
    for (const auto& crankshaft : config.crankshafts) {
        if (crankshaft.id == 0 || !crankshaftIds.insert(crankshaft.id).second
            || !inRange(crankshaft.positionXMm, -10'000.0, 10'000.0)
            || !inRange(crankshaft.positionYMm, -10'000.0, 10'000.0)
            || !inRange(crankshaft.phaseOffsetDegrees, -720.0, 720.0)
            || !inRange(crankshaft.rotationRatio, -8.0, 8.0) || std::abs(crankshaft.rotationRatio) < 0.01
            || !inRange(crankshaft.massKg, 0.01, 2'000.0)
            || !inRange(crankshaft.flywheelMassKg, 0.0, 2'000.0)
            || !inRange(crankshaft.momentOfInertiaKgM2, 0.001, 50.0)
            || !inRange(crankshaft.frictionTorqueNm, 0.0, 10'000.0))
            return "Crankshaft IDs, ratios, positions, masses and inertia must be finite and valid";
    }
    if (config.crankJournals.empty() || config.crankJournals.size() > 64)
        return "Crank journal table must contain between 1 and 64 entries";
    std::unordered_set<std::uint32_t> journalIds;
    for (const auto& journal : config.crankJournals) {
        if (journal.id == 0 || !inRange(journal.angleDegrees, -360.0, 720.0)
            || !inRange(journal.throwMm, 1.0, 120.0) || !crankshaftIds.contains(journal.crankshaftId))
            return "Crank journals must have finite IDs, angles and throws";
        journalIds.insert(journal.id);
    }
    if (journalIds.size() != config.crankJournals.size()) return "Crank journal IDs must be unique";
    const auto validateLiftProfile = [&inRange](const std::vector<ValveLiftSample>& profile, double maxLift) {
        if (profile.size() > 64) return false;
        double previous = -std::numeric_limits<double>::infinity();
        for (const auto& sample : profile) {
            if (!inRange(sample.angleDegrees, -360.0, 360.0) || !inRange(sample.liftMm, 0.0, maxLift)) return false;
            if (sample.angleDegrees <= previous) return false;
            previous = sample.angleDegrees;
        }
        return true;
    };
    const auto validateFlowCurve = [&inRange](const std::vector<ValveFlowSample>& curve) {
        if (curve.size() > 64) return false;
        double previousLift = -1.0;
        for (const auto& sample : curve) {
            if (!inRange(sample.liftMm, 0.0, 40.0)
                || !inRange(sample.dischargeCoefficient, 0.0, 1.5)
                || sample.liftMm <= previousLift) return false;
            previousLift = sample.liftMm;
        }
        return true;
    };
    const auto validateCamshaft = [&inRange, &validateLiftProfile, &validateFlowCurve](const CamshaftConfig& cam) {
        if (!inRange(cam.continuousControl.responseFrequencyHz, 0.1, 100.0)
            || (cam.continuousControl.enabled && cam.continuousControl.samples.empty())
            || cam.continuousControl.samples.size() > 128
            || !validateFlowCurve(cam.intakeFlowCurve) || !validateFlowCurve(cam.exhaustFlowCurve)) return false;
        for (std::size_t index = 0; index < cam.continuousControl.samples.size(); ++index) {
            const auto& sample = cam.continuousControl.samples[index];
            if (!inRange(sample.rpm, 0.0, 30'000.0) || !inRange(sample.load, 0.0, 1.5)
                || !inRange(sample.intakeAdvanceDegrees, -90.0, 90.0)
                || !inRange(sample.exhaustAdvanceDegrees, -90.0, 90.0)
                || !inRange(sample.liftMultiplier, 0.1, 2.0)) return false;
            for (std::size_t other = index + 1; other < cam.continuousControl.samples.size(); ++other)
                if (std::abs(sample.rpm - cam.continuousControl.samples[other].rpm) <= 1.0e-9
                    && std::abs(sample.load - cam.continuousControl.samples[other].load) <= 1.0e-12)
                    return false;
        }
        return inRange(cam.intakeDurationDegrees, 1.0, 720.0)
            && inRange(cam.exhaustDurationDegrees, 1.0, 720.0)
            && inRange(cam.intakeLiftMm, 0.0, 30.0)
            && inRange(cam.exhaustLiftMm, 0.0, 30.0)
            && inRange(cam.intakeCenterlineDegrees, 0.0, 720.0)
            && inRange(cam.exhaustCenterlineDegrees, 0.0, 720.0)
            && inRange(cam.intakeFlowCoefficient, 0.05, 1.50)
            && inRange(cam.exhaustFlowCoefficient, 0.05, 1.50)
            && inRange(cam.switchRpm, 200.0, 25'000.0)
            && inRange(cam.switchThrottle, 0.0, 1.0)
            && inRange(cam.highIntakeDurationDegrees, 1.0, 720.0)
            && inRange(cam.highExhaustDurationDegrees, 1.0, 720.0)
            && inRange(cam.highIntakeLiftMm, 0.0, 30.0)
            && inRange(cam.highExhaustLiftMm, 0.0, 30.0)
            && validateLiftProfile(cam.intakeLiftProfile, cam.intakeLiftMm)
            && validateLiftProfile(cam.exhaustLiftProfile, cam.exhaustLiftMm)
            && validateLiftProfile(cam.highIntakeLiftProfile, cam.highIntakeLiftMm)
            && validateLiftProfile(cam.highExhaustLiftProfile, cam.highExhaustLiftMm);
    };
    if (!validateCamshaft(config.camshafts))
        return "Valve lift profiles must be sorted finite tables within configured max lift";
    if (config.ignition.timingCurve.empty() || config.ignition.timingCurve.size() > 128)
        return "Ignition timing curve must contain between 1 and 128 samples";
    double previousIgnitionRpm = -1.0;
    for (const auto& sample : config.ignition.timingCurve) {
        if (!inRange(sample.rpm, 0.0, 25'000.0) || !inRange(sample.advanceDegrees, -30.0, 80.0)
            || sample.rpm <= previousIgnitionRpm)
            return "Ignition timing curve must be finite and strictly ordered by RPM";
        previousIgnitionRpm = sample.rpm;
    }
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
            || !inRange(cylinder.crankOffsetDegrees, -360.0, 720.0)
            || !inRange(cylinder.bankOffsetDegrees, -360.0, 360.0)
            || !(cylinder.intakeRunnerLengthMm == 0.0
                || inRange(cylinder.intakeRunnerLengthMm, 20.0, 2'000.0))
            || !(cylinder.intakeRunnerDiameterMm == 0.0
                || inRange(cylinder.intakeRunnerDiameterMm, 10.0, 150.0))
            || !inRange(cylinder.exhaustPrimaryLengthMm, 0.0, 3'000.0)
            || !inRange(cylinder.soundAttenuation, 0.0, 4.0)
            || !inRange(cylinder.blowByCoefficient, 0.0, 0.1)
            || !inRange(cylinder.connectingRodMassGrams, 20.0, 5'000.0)
            || !inRange(cylinder.pistonFrictionCoefficient, 0.0, 0.5)
            || !inRange(cylinder.pistonBreakawayForceN, 0.0, 5'000.0)
            || !inRange(cylinder.pistonBreakawayVelocityMps, 0.001, 5.0)
            || !inRange(cylinder.pistonViscousFrictionNsPerM, 0.0, 2'000.0)
            || !inRange(cylinder.articulatedJournalRadiusMm, 0.0, 250.0)
            || !inRange(cylinder.articulatedJournalAngleDegrees, -720.0, 720.0)
            || !inRange(cylinder.deckHeightMm, 0.0, 1'000.0)
            || !inRange(cylinder.compressionHeightMm, 0.0, 250.0)
            || !inRange(cylinder.wristPinOffsetMm, -20.0, 20.0)
            || !inRange(cylinder.pistonCrownVolumeCc, -250.0, 250.0)
            || !inRange(cylinder.headChamberVolumeCc, 0.0, 500.0)
            || !inRange(cylinder.headGasketThicknessMm, 0.0, 10.0)
            || !inRange(cylinder.connectingRodMomentOfInertiaKgM2, 0.0, 1.0)
            || cylinder.intakeValveCount < 1 || cylinder.intakeValveCount > 4
            || cylinder.exhaustValveCount < 1 || cylinder.exhaustValveCount > 4
            || !(cylinder.intakeValveDiameterMm == 0.0
                || inRange(cylinder.intakeValveDiameterMm, 10.0, 80.0))
            || !(cylinder.exhaustValveDiameterMm == 0.0
                || inRange(cylinder.exhaustValveDiameterMm, 10.0, 80.0)))
            return "Cylinder IDs and dimensions must be finite and physically valid";
        if (cylinder.crankJournalId == 0)
            return "Every cylinder must explicitly reference a configured crank journal";
        if (!journalIds.contains(cylinder.crankJournalId))
            return "Cylinder crankJournalId must reference a configured crank journal";
        const auto journal = std::find_if(config.crankJournals.begin(), config.crankJournals.end(),
            [&cylinder](const auto& candidate) { return candidate.id == cylinder.crankJournalId; });
        if (journal != config.crankJournals.end()
            && std::abs(journal->throwMm * 2.0 - cylinder.strokeMm) > 0.05)
            return "Crank journal throw must equal half of the referenced cylinder stroke";
        const auto explicitDeckGeometry = cylinder.deckHeightMm > 0.0 || cylinder.compressionHeightMm > 0.0
            || cylinder.headChamberVolumeCc > 0.0 || cylinder.headGasketThicknessMm > 0.0
            || cylinder.pistonCrownVolumeCc != 0.0;
        if (explicitDeckGeometry && (cylinder.deckHeightMm <= 0.0 || cylinder.compressionHeightMm <= 0.0
            || cylinder.headChamberVolumeCc <= 0.0))
            return "Explicit chamber geometry requires positive deck height, compression height and head chamber volume";
        if (explicitDeckGeometry) {
            const auto areaMm2 = std::numbers::pi * cylinder.boreMm * cylinder.boreMm * 0.25;
            const auto deckClearanceMm = cylinder.deckHeightMm - cylinder.strokeMm * 0.5
                - cylinder.connectingRodMm - cylinder.compressionHeightMm;
            const auto clearanceCc = cylinder.headChamberVolumeCc - cylinder.pistonCrownVolumeCc
                + areaMm2 * (cylinder.headGasketThicknessMm + deckClearanceMm) / 1'000.0;
            const auto sweptCc = areaMm2 * cylinder.strokeMm / 1'000.0;
            if (clearanceCc <= 0.1 || std::abs((1.0 + sweptCc / clearanceCc) - cylinder.compressionRatio) > 0.25)
                return "Explicit chamber geometry must yield positive clearance and match compressionRatio";
        }
        cylinderIds.insert(cylinder.id);
    }
    const std::unordered_set<std::uint32_t> firingIds(config.firingOrder.begin(), config.firingOrder.end());
    if (cylinderIds.size() != config.cylinders.size()) return "Cylinder IDs must be unique";
    if (firingIds != cylinderIds) return "Firing order must contain every cylinder ID exactly once";
    for (const auto& cylinder : config.cylinders) {
        if (cylinder.connectingRodType == ConnectingRodType::articulated) {
            if (cylinder.masterCylinderId == 0 || cylinder.masterCylinderId == cylinder.id
                || !cylinderIds.contains(cylinder.masterCylinderId) || cylinder.articulatedJournalRadiusMm <= 0.0)
                return "Articulated rods must reference a distinct master cylinder and a positive articulation radius";
            const auto master = std::find_if(config.cylinders.begin(), config.cylinders.end(),
                [&cylinder](const auto& candidate) { return candidate.id == cylinder.masterCylinderId; });
            if (master == config.cylinders.end() || master->connectingRodType == ConnectingRodType::articulated)
                return "An articulated rod master must be a conventional or master rod";
        } else if (cylinder.masterCylinderId != 0 || cylinder.articulatedJournalRadiusMm != 0.0) {
            return "Only articulated rods may define a master cylinder or articulation radius";
        }
    }
    if (config.intakePaths.empty() || config.intakePaths.size() > 32)
        return "Intake topology must contain between 1 and 32 paths";
    std::unordered_set<std::uint32_t> intakePathIds;
    std::unordered_set<std::uint32_t> assignedIntakeCylinders;
    for (const auto& path : config.intakePaths) {
        const auto& intake = path.geometry;
        if (path.id == 0 || path.cylinderIds.empty() || !intakePathIds.insert(path.id).second
            || !inRange(intake.plenumVolumeLitres, 0.05, 100.0)
            || !inRange(intake.throttleDiameterMm, 5.0, 250.0)
            || intake.throttleCount < 1 || intake.throttleCount > 16
            || !inRange(intake.throttleDischargeCoefficient, 0.05, 1.5)
            || !inRange(intake.runnerLengthMm, 20.0, 2'000.0)
            || !inRange(intake.runnerDiameterMm, 10.0, 150.0)
            || !(intake.runnerPlenumDiameterMm == 0.0
                || inRange(intake.runnerPlenumDiameterMm, 10.0, 150.0))
            || !inRange(intake.airboxVolumeLitres, 0.0, 100.0)
            || !inRange(intake.inletDuctLengthMm, 0.0, 5'000.0)
            || !inRange(intake.inletDuctDiameterMm, 0.0, 500.0)
            || !inRange(intake.bellmouthDiameterMm, 0.0, 1'000.0)
            || !inRange(intake.idleBypassAreaMm2, 0.0, 1'000.0)
            || !inRange(intake.throttleGamma, 0.2, 5.0))
            return "Intake path IDs and geometry must be valid";
        for (const auto cylinderId : path.cylinderIds)
            if (!cylinderIds.contains(cylinderId) || !assignedIntakeCylinders.insert(cylinderId).second)
                return "Intake paths must reference every cylinder exactly once";
    }
    if (assignedIntakeCylinders != cylinderIds)
        return "Configured intake paths must assign every cylinder exactly once";
    if (config.banks.size() > 32) return "Bank topology may contain at most 32 banks";
    std::unordered_set<std::uint32_t> assignedBankCylinders;
    std::unordered_set<std::uint32_t> bankIds;
    for (const auto& bank : config.banks) {
        // Radial layouts conventionally enumerate cylinder axes over
        // [0, 360), whereas V/flat layouts commonly use signed angles.
        // Both describe the same physical circle and must remain valid.
        if (bank.id == 0 || bank.cylinderIds.empty() || !bankIds.insert(bank.id).second || !inRange(bank.angleDegrees, -360.0, 360.0)
            || !validateCamshaft(bank.camshafts)) return "Bank IDs, angles and camshafts must be valid";
        if (!intakePathIds.contains(bank.intakeId)) return "Bank intakeId must reference a configured intake path";
        for (const auto cylinderId : bank.cylinderIds) {
            if (!cylinderIds.contains(cylinderId) || !assignedBankCylinders.insert(cylinderId).second)
                return "Bank cylinder references must be unique and valid";
            const auto cylinder = std::find_if(config.cylinders.begin(), config.cylinders.end(),
                [cylinderId](const auto& candidate) { return candidate.id == cylinderId; });
            if (cylinder == config.cylinders.end() || cylinder->bankId != bank.id)
                return "Cylinder bankId must match the bank that contains the cylinder";
        }
    }
    if (!config.banks.empty() && assignedBankCylinders != cylinderIds)
        return "Configured banks must assign every cylinder exactly once";
    if (config.exhaustPaths.size() > 8) return "Exhaust topology may contain at most 8 paths supported by audio";
    std::unordered_set<std::uint32_t> exhaustPathIds;
    std::unordered_set<std::uint32_t> assignedExhaustCylinders;
    std::size_t exhaustAcousticSideBranchCount = 0;
    for (const auto& path : config.exhaustPaths) {
        const auto& exhaust = path.geometry;
        if (path.id == 0 || path.cylinderIds.empty() || !exhaustPathIds.insert(path.id).second || !inRange(path.audioVolume, 0.0, 8.0)
            || path.impulseResponsePath.size() > 1'024
            || !inRange(exhaust.primaryLengthMm, 100.0, 3'000.0)
            || !inRange(exhaust.primaryDiameterMm, 15.0, 200.0)
            || !inRange(exhaust.collectorDiameterMm, 15.0, 300.0)
            || !inRange(exhaust.outletDiameterMm, 15.0, 300.0)
            || !inRange(exhaust.mufflerRestriction, 0.0, 1.0)
            || !inRange(exhaust.collectorVolumeLitres, 0.05, 200.0)
            || !inRange(exhaust.outletDischargeCoefficient, 0.02, 1.5)
            || !inRange(exhaust.mufflerChamberDiameterMm, 0.0, 600.0)
            || !inRange(exhaust.mufflerChamberLengthMm, 0.0, 3'000.0)
            || !inRange(exhaust.mufflerPackingFlowResistivityPaSPerM2, 0.0, 200'000.0)
            || !inRange(exhaust.mufflerPackingThicknessMm, 0.0, 300.0)
            || !inRange(exhaust.mufflerPerforatedOpenAreaRatio, 0.0, 1.0)
            || !validMufflerPacking(exhaust)
            || !validPoint(path.acousticPositionM)
            || !validPoint(path.acousticAxis)
            || path.acousticAxis.x * path.acousticAxis.x
                + path.acousticAxis.y * path.acousticAxis.y
                + path.acousticAxis.z * path.acousticAxis.z < 1.0e-12)
            return "Exhaust path IDs and audio volumes must be valid";
        for (const auto cylinderId : path.cylinderIds)
            if (!cylinderIds.contains(cylinderId) || !assignedExhaustCylinders.insert(cylinderId).second)
                return "Exhaust paths must reference every cylinder at most once";

        if (path.network) {
            const auto& network = *path.network;
            if (network.components.empty() || network.components.size() > 256)
                return "Custom exhaust graphs must contain between 1 and 256 components";
            if (network.connections.size() > 1'024)
                return "Custom exhaust graphs may contain at most 1024 component connections";

            std::unordered_map<std::uint32_t, const ExhaustComponentConfig*> components;
            std::unordered_map<std::uint32_t, std::vector<std::uint32_t>> outgoing;
            std::unordered_map<std::uint32_t, std::size_t> componentIncoming;
            std::unordered_map<std::uint32_t, std::size_t> cylinderIncoming;
            components.reserve(network.components.size());
            outgoing.reserve(network.components.size());
            componentIncoming.reserve(network.components.size());
            cylinderIncoming.reserve(network.components.size());
            for (const auto& component : network.components) {
                const auto requiresLength = component.type == ExhaustComponentType::pipe
                    || component.type == ExhaustComponentType::resonator
                    || component.type == ExhaustComponentType::muffler
                    || component.type == ExhaustComponentType::catalyst;
                if (static_cast<std::uint8_t>(component.type)
                        > static_cast<std::uint8_t>(ExhaustComponentType::outlet)
                    || component.id == 0 || !components.emplace(component.id, &component).second
                    || !inRange(component.lengthMm, requiresLength ? 1.0 : 0.0, 10'000.0)
                    || !inRange(component.diameterMm, 5.0, 500.0)
                    || !(component.outletDiameterMm == 0.0
                        || inRange(component.outletDiameterMm, 5.0, 500.0))
                    || !inRange(component.volumeLitres, 0.0, 1'000.0)
                    || !inRange(component.restriction, 0.0, 20.0)
                    || !inRange(component.resonanceHz, 0.0, 20'000.0)
                    || !inRange(component.acousticGain, 0.0, 8.0)
                    || !inRange(component.dischargeCoefficient, 0.02, 1.5)
                    || !inRange(component.packingFlowResistivityPaSPerM2, 0.0, 200'000.0)
                    || !inRange(component.packingThicknessMm, 0.0, 300.0)
                    || !inRange(component.perforatedOpenAreaRatio, 0.0, 1.0)
                    || !inRange(component.catalystCellDensityCpsi, 0.0, 5'000.0)
                    || !inRange(component.catalystOpenAreaRatio, 0.0, 0.99)
                    || !inRange(component.catalystSubstrateVolumetricHeatCapacityJPerM3K,
                        0.0, 10'000'000.0)
                    || !validPoint(component.acousticPositionM)
                    || !validPoint(component.acousticAxis)
                    || (component.type == ExhaustComponentType::outlet
                        && component.acousticAxis.x * component.acousticAxis.x
                            + component.acousticAxis.y * component.acousticAxis.y
                            + component.acousticAxis.z * component.acousticAxis.z
                            < 1.0e-12))
                    return "Custom exhaust component IDs and dimensions must be finite, unique and valid";
                const auto hasPacking = component.packingFlowResistivityPaSPerM2 > 0.0
                    || component.packingThicknessMm > 0.0
                    || component.perforatedOpenAreaRatio > 0.0;
                if (hasPacking
                    && (component.type != ExhaustComponentType::muffler
                        || component.packingFlowResistivityPaSPerM2 <= 0.0
                        || component.packingThicknessMm <= 0.0
                        || component.perforatedOpenAreaRatio <= 0.0))
                    return "Porous packing requires a muffler with resistivity, thickness and perforated open area";
                if (hasPacking) {
                    const auto inletRadiusM = component.diameterMm * 0.0005;
                    const auto outletDiameterMm = component.outletDiameterMm > 0.0
                        ? component.outletDiameterMm : component.diameterMm;
                    const auto outletRadiusM = outletDiameterMm * 0.0005;
                    const auto inletAreaM2 = std::numbers::pi
                        * inletRadiusM * inletRadiusM;
                    const auto outletAreaM2 = std::numbers::pi
                        * outletRadiusM * outletRadiusM;
                    const auto coreMeanAreaM2 = (inletAreaM2
                        + std::sqrt(inletAreaM2 * outletAreaM2)
                        + outletAreaM2) / 3.0;
                    // m2 * mm has the same numerical value as litres. A packed
                    // straight-through component needs volume outside that
                    // swept core; otherwise its material fields describe a can
                    // which has no physical annulus to occupy.
                    const auto coreVolumeLitres = coreMeanAreaM2
                        * component.lengthMm;
                    if (!(component.volumeLitres
                            > coreVolumeLitres * (1.0 + 1.0e-9)))
                        return "Porous packing requires outer-can volume greater than the perforated core volume";
                }
                const auto hasCatalystMonolith =
                    component.catalystCellDensityCpsi > 0.0
                    || component.catalystOpenAreaRatio > 0.0
                    || component.catalystSubstrateVolumetricHeatCapacityJPerM3K > 0.0;
                if (hasCatalystMonolith) {
                    const auto monolith = catalystMonolithGeometry(
                        component.catalystCellDensityCpsi,
                        component.catalystOpenAreaRatio);
                    if (component.type != ExhaustComponentType::catalyst
                        || component.catalystCellDensityCpsi < 25.0
                        || component.catalystOpenAreaRatio < 0.05
                        || component.catalystSubstrateVolumetricHeatCapacityJPerM3K
                            < 100'000.0
                        || !monolith.active
                        || monolith.cellPitchM > component.diameterMm * 0.001)
                        return "Catalyst monolith requires a catalyst, 25..5000 cpsi, 0.05..0.99 open area, 0.1..10 MJ/m3/K substrate capacity and at least one cell across its diameter";
                }
                outgoing.try_emplace(component.id);
                componentIncoming.try_emplace(component.id, 0U);
                cylinderIncoming.try_emplace(component.id, 0U);
            }

            std::unordered_set<std::uint64_t> uniqueConnections;
            for (const auto& connection : network.connections) {
                if (!components.contains(connection.fromComponentId)
                    || !components.contains(connection.toComponentId)
                    || connection.fromComponentId == connection.toComponentId)
                    return "Custom exhaust connections must reference two distinct configured components";
                const auto key = (static_cast<std::uint64_t>(connection.fromComponentId) << 32U)
                    | connection.toComponentId;
                if (!uniqueConnections.insert(key).second)
                    return "Custom exhaust component connections must be unique";
                outgoing[connection.fromComponentId].push_back(connection.toComponentId);
                ++componentIncoming[connection.toComponentId];
            }

            std::unordered_set<std::uint32_t> pathCylinderIds(path.cylinderIds.begin(), path.cylinderIds.end());
            std::unordered_set<std::uint32_t> connectedCylinderIds;
            for (const auto& connection : network.cylinderConnections) {
                if (!pathCylinderIds.contains(connection.cylinderId)
                    || !components.contains(connection.componentId)
                    || !connectedCylinderIds.insert(connection.cylinderId).second)
                    return "Custom exhaust graphs must map each path cylinder to one configured component";
                ++cylinderIncoming[connection.componentId];
            }
            if (connectedCylinderIds != pathCylinderIds)
                return "Custom exhaust graphs must map every path cylinder exactly once";

            const auto isTerminalSideBranch = [&components, &outgoing](
                std::uint32_t componentId) {
                const auto component = components.find(componentId);
                const auto outputs = outgoing.find(componentId);
                return component != components.end()
                    && outputs != outgoing.end()
                    && component->second->type == ExhaustComponentType::resonator
                    && outputs->second.empty();
            };
            const auto pathAcousticSideBranchCount = static_cast<std::size_t>(
                std::count_if(network.components.begin(), network.components.end(),
                    [&isTerminalSideBranch](const auto& component) {
                        return isTerminalSideBranch(component.id);
                    }));
            if (pathAcousticSideBranchCount
                    > maximumExhaustAcousticSideBranches
                        - exhaustAcousticSideBranchCount)
                return "An engine may contain at most 8 terminal exhaust resonators";
            exhaustAcousticSideBranchCount += pathAcousticSideBranchCount;
            std::unordered_map<std::uint32_t, std::vector<std::uint32_t>>
                flowOutgoing;
            flowOutgoing.reserve(outgoing.size());
            for (const auto& [componentId, outputs] : outgoing) {
                auto& flowOutputs = flowOutgoing[componentId];
                for (const auto nextId : outputs)
                    if (!isTerminalSideBranch(nextId))
                        flowOutputs.push_back(nextId);
            }

            for (const auto& component : network.components) {
                const auto incoming = componentIncoming[component.id] + cylinderIncoming[component.id];
                if (isTerminalSideBranch(component.id)) {
                    if (componentIncoming[component.id] != 1U
                        || cylinderIncoming[component.id] != 0U)
                        return "A terminal exhaust resonator requires exactly one component attachment and no cylinder mapping";
                    continue;
                }
                const auto outputCount = flowOutgoing[component.id].size();
                switch (component.type) {
                case ExhaustComponentType::merge:
                    if (incoming < 2 || outputCount != 1)
                        return "An exhaust merge requires at least two inputs and exactly one output";
                    break;
                case ExhaustComponentType::splitter:
                    if (incoming != 1 || outputCount < 2)
                        return "An exhaust splitter requires exactly one input and at least two outputs";
                    break;
                case ExhaustComponentType::outlet:
                    if (incoming < 1 || outputCount != 0
                        || !outgoing[component.id].empty())
                        return "An exhaust outlet requires at least one input and cannot have outputs";
                    break;
                default:
                    if (incoming != 1 || outputCount != 1)
                        return "Pipe, resonator, muffler and catalyst components require one input and one output";
                    break;
                }
            }

            // Kahn's algorithm rejects cycles independently of cylinder roots.
            auto remainingIncoming = componentIncoming;
            std::vector<std::uint32_t> ready;
            ready.reserve(network.components.size());
            for (const auto& component : network.components)
                if (remainingIncoming[component.id] == 0) ready.push_back(component.id);
            std::size_t visitedCount = 0;
            for (std::size_t index = 0; index < ready.size(); ++index) {
                const auto componentId = ready[index];
                ++visitedCount;
                for (const auto nextId : outgoing[componentId])
                    if (--remainingIncoming[nextId] == 0) ready.push_back(nextId);
            }
            if (visitedCount != network.components.size())
                return "Custom exhaust component connections must form an acyclic graph";

            std::unordered_set<std::uint32_t> reachable;
            std::vector<std::uint32_t> frontier;
            for (const auto& connection : network.cylinderConnections)
                if (reachable.insert(connection.componentId).second) frontier.push_back(connection.componentId);
            for (std::size_t index = 0; index < frontier.size(); ++index)
                for (const auto nextId : outgoing[frontier[index]])
                    if (reachable.insert(nextId).second) frontier.push_back(nextId);
            if (reachable.size() != network.components.size())
                return "Every custom exhaust component must be reachable from a mapped cylinder";

            std::unordered_map<std::uint32_t, bool> reachesOutletMemo;
            std::function<bool(std::uint32_t)> everyRouteReachesOutlet = [&](std::uint32_t componentId) {
                if (const auto found = reachesOutletMemo.find(componentId); found != reachesOutletMemo.end())
                    return found->second;
                const auto component = components.at(componentId);
                if (component->type == ExhaustComponentType::outlet)
                    return reachesOutletMemo.emplace(componentId, true).first->second;
                const auto& outputs = flowOutgoing[componentId];
                const auto valid = !outputs.empty() && std::all_of(outputs.begin(), outputs.end(),
                    [&everyRouteReachesOutlet](std::uint32_t nextId) { return everyRouteReachesOutlet(nextId); });
                reachesOutletMemo.emplace(componentId, valid);
                return valid;
            };
            for (const auto& connection : network.cylinderConnections)
                if (!everyRouteReachesOutlet(connection.componentId))
                    return "Every route from a cylinder in a custom exhaust graph must reach an outlet";

            // Bound path expansion so a valid-looking splitter DAG cannot
            // cause exponential work or memory use in the runtime compiler.
            constexpr std::size_t maximumRoutes = 4'096;
            std::unordered_map<std::uint32_t, std::size_t> routeCounts;
            routeCounts.reserve(network.components.size());
            for (auto iterator = ready.rbegin(); iterator != ready.rend(); ++iterator) {
                const auto componentId = *iterator;
                if (components.at(componentId)->type == ExhaustComponentType::outlet) {
                    routeCounts[componentId] = 1;
                    continue;
                }
                std::size_t count = 0;
                for (const auto nextId : flowOutgoing[componentId])
                    count = std::min(maximumRoutes + 1, count + routeCounts[nextId]);
                routeCounts[componentId] = count;
            }
            std::size_t totalRoutes = 0;
            for (const auto& connection : network.cylinderConnections)
                totalRoutes = std::min(maximumRoutes + 1,
                    totalRoutes + routeCounts[connection.componentId]);
            if (totalRoutes > maximumRoutes)
                return "Custom exhaust graph expands to more than 4096 cylinder-to-outlet routes";
        }
    }
    if (!config.exhaustPaths.empty() && assignedExhaustCylinders != cylinderIds)
        return "Configured exhaust paths must assign every cylinder exactly once";
    for (const auto& bank : config.banks)
        if (!config.exhaustPaths.empty() && bank.exhaustPathId != 0 && !exhaustPathIds.contains(bank.exhaustPathId))
            return "Bank exhaustPathId must reference a configured exhaust path";
    const auto displacement = engineDisplacementLitres(config);
    if (!inRange(displacement, 0.05, 20.0)) return "Total displacement must be between 0.05 and 20 litres";
    return std::nullopt;
}

const char* afterfireBlockerName(std::uint32_t mask) noexcept {
    // Ordered by how much the reading tells someone who hears no pop. An
    // unauthored feature explains everything else, so it leads; the duty-cycle
    // chop is the mechanism of a WORKING pop map and therefore comes last.
    if (afterfireBlocked(mask, AfterfireBlocker::notAuthored)) return "non autorise";
    if (afterfireBlocked(mask, AfterfireBlocker::ignitionOff)) return "allumage coupe";
    if (afterfireBlocked(mask, AfterfireBlocker::cranking)) return "demarreur";
    if (afterfireBlocked(mask, AfterfireBlocker::notArmed)) return "jamais arme (monter en regime a fond)";
    if (afterfireBlocked(mask, AfterfireBlocker::belowMinimumRpm)) return "regime trop bas";
    if (afterfireBlocked(mask, AfterfireBlocker::throttleOpen)) return "papillon ouvert";
    if (afterfireBlocked(mask, AfterfireBlocker::revLimiterActive)) return "rupteur";
    if (afterfireBlocked(mask, AfterfireBlocker::fuelCutInactive)) return "pas en coupure decel";
    if (afterfireBlocked(mask, AfterfireBlocker::pulseChopClosed)) return "hachage ferme";
    return "";
}

} // namespace enginelab
