#include <enginelab/physics/MechanicalKinematics.hpp>
#include <algorithm>
#include <cmath>
#include <numbers>

namespace enginelab {
namespace {
struct Point final { double x {}; double y {}; };
struct Position final { Point crank; Point wrist; double axisPositionMm {}; double rodAngleRadians {}; };
struct CylinderReference final {
    double topAxisPositionMm {};
    double bottomAxisPositionMm {};
    double topDeadCentreAngleDegrees {};
    double bottomDeadCentreAngleDegrees {};
};

[[nodiscard]] const CrankJournalConfig* journalFor(const EngineConfig& config,
                                                    const CylinderConfig& cylinder) noexcept {
    const auto item = std::find_if(config.crankJournals.begin(), config.crankJournals.end(),
        [&cylinder](const auto& value) { return value.id == cylinder.crankJournalId; });
    return item == config.crankJournals.end() ? nullptr : &*item;
}

[[nodiscard]] const CrankshaftConfig* crankshaftFor(const EngineConfig& config,
                                                     const CrankJournalConfig* journal) noexcept {
    const auto id = journal == nullptr ? std::uint32_t { 1 } : journal->crankshaftId;
    const auto item = std::find_if(config.crankshafts.begin(), config.crankshafts.end(),
        [id](const auto& value) { return value.id == id; });
    return item == config.crankshafts.end() ? nullptr : &*item;
}

[[nodiscard]] double bankAngleFor(const EngineConfig& config, const CylinderConfig& cylinder) noexcept {
    if (cylinder.bankId != 0) {
        const auto bank = std::find_if(config.banks.begin(), config.banks.end(),
            [&cylinder](const auto& value) { return value.id == cylinder.bankId; });
        if (bank != config.banks.end()) return bank->angleDegrees;
    }
    return cylinder.bankOffsetDegrees;
}

[[nodiscard]] Position conventionalPosition(const EngineConfig& config, const CylinderConfig& cylinder,
                                              double primaryAngleDegrees) noexcept {
    const auto* journal = journalFor(config, cylinder);
    const auto* crankshaft = crankshaftFor(config, journal);
    const auto origin = Point { crankshaft == nullptr ? 0.0 : crankshaft->positionXMm,
                                crankshaft == nullptr ? 0.0 : crankshaft->positionYMm };
    const auto ratio = crankshaft == nullptr ? 1.0 : crankshaft->rotationRatio;
    const auto phase = crankshaft == nullptr ? 0.0 : crankshaft->phaseOffsetDegrees;
    const auto journalAngle = journal == nullptr ? cylinder.crankOffsetDegrees : journal->angleDegrees;
    const auto theta = (primaryAngleDegrees * ratio + phase - journalAngle) * std::numbers::pi / 180.0;
    const auto bank = bankAngleFor(config, cylinder) * std::numbers::pi / 180.0;
    const Point axis { std::sin(bank), -std::cos(bank) };
    const Point normal { std::cos(bank), std::sin(bank) };
    const auto radius = journal == nullptr ? cylinder.strokeMm * 0.5 : journal->throwMm;
    // A journal is one point in the crankshaft's global frame. Rotating it
    // into each cylinder-bank frame creates a different physical crankpin for
    // every bank and disconnects chamber volume from firing phase.
    const Point crank { origin.x + radius * std::sin(theta),
                        origin.y - radius * std::cos(theta) };
    const auto projected = (crank.x - origin.x) * axis.x + (crank.y - origin.y) * axis.y;
    const auto lateral = (crank.x - origin.x) * normal.x + (crank.y - origin.y) * normal.y
        - cylinder.wristPinOffsetMm;
    const auto along = std::sqrt(std::max(0.0,
        cylinder.connectingRodMm * cylinder.connectingRodMm - lateral * lateral));
    const auto piston = projected + along;
    const Point wrist { origin.x + axis.x * piston + normal.x * cylinder.wristPinOffsetMm,
                        origin.y + axis.y * piston + normal.y * cylinder.wristPinOffsetMm };
    return { crank, wrist, piston, std::atan2(wrist.y - crank.y, wrist.x - crank.x) };
}

[[nodiscard]] Position positionFor(const EngineConfig& config, std::size_t index,
                                    double primaryAngleDegrees) noexcept {
    const auto& cylinder = config.cylinders[index];
    if (cylinder.connectingRodType != ConnectingRodType::articulated)
        return conventionalPosition(config, cylinder, primaryAngleDegrees);
    const auto masterIndex = std::find_if(config.cylinders.begin(), config.cylinders.end(),
        [&cylinder](const auto& value) { return value.id == cylinder.masterCylinderId; });
    if (masterIndex == config.cylinders.end()) return conventionalPosition(config, cylinder, primaryAngleDegrees);
    const auto master = conventionalPosition(config, *masterIndex, primaryAngleDegrees);
    const auto rodDx = master.wrist.x - master.crank.x;
    const auto rodDy = master.wrist.y - master.crank.y;
    const auto rodLength = std::max(1.0e-9, std::hypot(rodDx, rodDy));
    const auto rodAngle = std::atan2(rodDy, rodDx)
        + cylinder.articulatedJournalAngleDegrees * std::numbers::pi / 180.0;
    const Point articulation { master.crank.x + rodDx / rodLength * cylinder.articulatedJournalRadiusMm * std::cos(
                                    cylinder.articulatedJournalAngleDegrees * std::numbers::pi / 180.0)
                                - rodDy / rodLength * cylinder.articulatedJournalRadiusMm * std::sin(
                                    cylinder.articulatedJournalAngleDegrees * std::numbers::pi / 180.0),
                               master.crank.y + rodDy / rodLength * cylinder.articulatedJournalRadiusMm * std::cos(
                                    cylinder.articulatedJournalAngleDegrees * std::numbers::pi / 180.0)
                                + rodDx / rodLength * cylinder.articulatedJournalRadiusMm * std::sin(
                                    cylinder.articulatedJournalAngleDegrees * std::numbers::pi / 180.0) };
    const auto* journal = journalFor(config, cylinder);
    const auto* crankshaft = crankshaftFor(config, journal);
    const Point origin { crankshaft == nullptr ? 0.0 : crankshaft->positionXMm,
                         crankshaft == nullptr ? 0.0 : crankshaft->positionYMm };
    const auto bank = bankAngleFor(config, cylinder) * std::numbers::pi / 180.0;
    const Point axis { std::sin(bank), -std::cos(bank) };
    const Point normal { std::cos(bank), std::sin(bank) };
    const auto projected = (articulation.x - origin.x) * axis.x + (articulation.y - origin.y) * axis.y;
    const auto lateral = (articulation.x - origin.x) * normal.x + (articulation.y - origin.y) * normal.y
        - cylinder.wristPinOffsetMm;
    const auto piston = projected + std::sqrt(std::max(0.0,
        cylinder.connectingRodMm * cylinder.connectingRodMm - lateral * lateral));
    const Point wrist { origin.x + axis.x * piston + normal.x * cylinder.wristPinOffsetMm,
                        origin.y + axis.y * piston + normal.y * cylinder.wristPinOffsetMm };
    (void)rodAngle;
    return { articulation, wrist, piston, std::atan2(wrist.y - articulation.y, wrist.x - articulation.x) };
}

[[nodiscard]] double nominalTopDeadCentreAngle(const EngineConfig& config,
                                                const CylinderConfig& cylinder) noexcept {
    const auto* journal = journalFor(config, cylinder);
    const auto* crankshaft = crankshaftFor(config, journal);
    const auto ratio = crankshaft == nullptr ? 1.0 : crankshaft->rotationRatio;
    const auto phase = crankshaft == nullptr ? 0.0 : crankshaft->phaseOffsetDegrees;
    const auto journalAngle = journal == nullptr ? cylinder.crankOffsetDegrees : journal->angleDegrees;
    return (journalAngle + bankAngleFor(config, cylinder) - phase) / ratio;
}

[[nodiscard]] CylinderReference resolveCylinderReference(const EngineConfig& config,
                                                          std::size_t cylinderIndex) noexcept {
    if (cylinderIndex >= config.cylinders.size()) return {};
    const auto& cylinder = config.cylinders[cylinderIndex];
    const auto* journal = journalFor(config, cylinder);
    const auto* crankshaft = crankshaftFor(config, journal);
    const auto ratio = crankshaft == nullptr ? 1.0 : crankshaft->rotationRatio;
    const auto periodDegrees = 360.0 / std::max(0.01, std::abs(ratio));
    const auto nominal = nominalTopDeadCentreAngle(config, cylinder);

    constexpr int coarseIntervals = 72;
    auto topAngle = nominal;
    auto bottomAngle = nominal;
    auto topPosition = positionFor(config, cylinderIndex, topAngle).axisPositionMm;
    auto bottomPosition = topPosition;
    for (int sample = 0; sample < coarseIntervals; ++sample) {
        const auto angle = nominal - periodDegrees * 0.5
            + periodDegrees * static_cast<double>(sample) / static_cast<double>(coarseIntervals);
        const auto position = positionFor(config, cylinderIndex, angle).axisPositionMm;
        if (position > topPosition) { topPosition = position; topAngle = angle; }
        if (position < bottomPosition) { bottomPosition = position; bottomAngle = angle; }
    }

    const auto refine = [&config, cylinderIndex, periodDegrees](double center,
                                                                 bool maximum) noexcept {
        auto left = center - periodDegrees / static_cast<double>(coarseIntervals);
        auto right = center + periodDegrees / static_cast<double>(coarseIntervals);
        for (int iteration = 0; iteration < 28; ++iteration) {
            const auto first = (2.0 * left + right) / 3.0;
            const auto second = (left + 2.0 * right) / 3.0;
            const auto firstPosition = positionFor(config, cylinderIndex, first).axisPositionMm;
            const auto secondPosition = positionFor(config, cylinderIndex, second).axisPositionMm;
            if ((firstPosition < secondPosition) == maximum) left = first;
            else right = second;
        }
        return 0.5 * (left + right);
    };
    topAngle = refine(topAngle, true);
    bottomAngle = refine(bottomAngle, false);
    return { positionFor(config, cylinderIndex, topAngle).axisPositionMm,
             positionFor(config, cylinderIndex, bottomAngle).axisPositionMm,
             topAngle, bottomAngle };
}

[[nodiscard]] double clearanceVolumeLitres(const CylinderConfig& cylinder,
                                            double resolvedStrokeMm) noexcept {
    const auto areaMm2 = std::numbers::pi * cylinder.boreMm * cylinder.boreMm * 0.25;
    if (cylinder.headChamberVolumeCc > 0.0) {
        const auto crankRadius = cylinder.strokeMm * 0.5;
        const auto deckClearance = cylinder.deckHeightMm
            - crankRadius - cylinder.connectingRodMm - cylinder.compressionHeightMm;
        const auto cc = cylinder.headChamberVolumeCc - cylinder.pistonCrownVolumeCc
            + areaMm2 * (cylinder.headGasketThicknessMm + deckClearance) / 1'000.0;
        return std::max(0.001, cc / 1'000.0);
    }
    const auto sweptLitres = areaMm2 * std::max(0.001, resolvedStrokeMm) / 1'000'000.0;
    return sweptLitres / std::max(1.0, cylinder.compressionRatio - 1.0);
}
} // namespace

EngineKinematicsReference buildEngineKinematicsReference(const EngineConfig& config) noexcept {
    EngineKinematicsReference result;
    result.cylinderCount = std::min(config.cylinders.size(), result.topAxisPositionMm.size());
    for (std::size_t index = 0; index < result.cylinderCount; ++index) {
        const auto reference = resolveCylinderReference(config, index);
        result.topAxisPositionMm[index] = reference.topAxisPositionMm;
        result.bottomAxisPositionMm[index] = reference.bottomAxisPositionMm;
        result.topDeadCentreAngleDegrees[index] = reference.topDeadCentreAngleDegrees;
        result.bottomDeadCentreAngleDegrees[index] = reference.bottomDeadCentreAngleDegrees;
    }
    return result;
}

CylinderKinematics evaluateCylinderKinematics(const EngineConfig& config,
                                               const EngineKinematicsReference& reference,
                                               std::size_t cylinderIndex,
                                               double crankAngleDegrees, double omega,
                                               double angularAcceleration) noexcept {
    CylinderKinematics result;
    if (cylinderIndex >= config.cylinders.size()) return result;
    const auto& cylinder = config.cylinders[cylinderIndex];
    constexpr double deltaRadians = 1.0e-4;
    constexpr double radiansToDegrees = 180.0 / std::numbers::pi;
    const auto center = positionFor(config, cylinderIndex, crankAngleDegrees);
    const auto before = positionFor(config, cylinderIndex, crankAngleDegrees - deltaRadians * radiansToDegrees);
    const auto after = positionFor(config, cylinderIndex, crankAngleDegrees + deltaRadians * radiansToDegrees);
    const auto* journal = journalFor(config, cylinder);
    const auto resolved = cylinderIndex < reference.cylinderCount
        ? CylinderReference { reference.topAxisPositionMm[cylinderIndex],
                              reference.bottomAxisPositionMm[cylinderIndex],
                              reference.topDeadCentreAngleDegrees[cylinderIndex],
                              reference.bottomDeadCentreAngleDegrees[cylinderIndex] }
        : resolveCylinderReference(config, cylinderIndex);
    const auto firstDerivative = (after.axisPositionMm - before.axisPositionMm) / (2.0 * deltaRadians);
    const auto secondDerivative = (after.axisPositionMm - 2.0 * center.axisPositionMm + before.axisPositionMm)
        / (deltaRadians * deltaRadians);
    result.pistonPositionMm = center.axisPositionMm;
    result.pistonTravelMm = std::max(0.0, resolved.topAxisPositionMm - center.axisPositionMm);
    result.displacementDerivativeMPerRadian = -firstDerivative * 0.001;
    result.pistonVelocityMps = firstDerivative * 0.001 * omega;
    result.pistonAccelerationMps2 = (secondDerivative * omega * omega
        + firstDerivative * angularAcceleration) * 0.001;
    result.connectingRodAngleDegrees = center.rodAngleRadians * radiansToDegrees;
    const auto cylinderAxisAngleDegrees = bankAngleFor(config, cylinder) - 90.0;
    result.connectingRodObliquityDegrees = std::abs(std::remainder(
        result.connectingRodAngleDegrees - cylinderAxisAngleDegrees, 180.0));
    result.crankPinXMm = center.crank.x;
    result.crankPinYMm = center.crank.y;
    result.wristPinXMm = center.wrist.x;
    result.wristPinYMm = center.wrist.y;
    const auto areaMm2 = std::numbers::pi * cylinder.boreMm * cylinder.boreMm * 0.25;
    result.chamberVolumeLitres = clearanceVolumeLitres(cylinder,
        resolved.topAxisPositionMm - resolved.bottomAxisPositionMm)
        + areaMm2 * result.pistonTravelMm / 1'000'000.0;
    result.crankJournalId = cylinder.crankJournalId;
    result.crankshaftId = journal == nullptr ? 1 : journal->crankshaftId;
    return result;
}

CylinderKinematics evaluateCylinderKinematics(const EngineConfig& config, std::size_t cylinderIndex,
                                               double crankAngleDegrees, double omega,
                                               double angularAcceleration) noexcept {
    EngineKinematicsReference reference;
    if (cylinderIndex < reference.topAxisPositionMm.size()) {
        const auto resolved = resolveCylinderReference(config, cylinderIndex);
        reference.cylinderCount = cylinderIndex + 1;
        reference.topAxisPositionMm[cylinderIndex] = resolved.topAxisPositionMm;
        reference.bottomAxisPositionMm[cylinderIndex] = resolved.bottomAxisPositionMm;
        reference.topDeadCentreAngleDegrees[cylinderIndex] = resolved.topDeadCentreAngleDegrees;
        reference.bottomDeadCentreAngleDegrees[cylinderIndex] = resolved.bottomDeadCentreAngleDegrees;
    }
    return evaluateCylinderKinematics(config, reference, cylinderIndex, crankAngleDegrees,
                                      omega, angularAcceleration);
}

} // namespace enginelab
