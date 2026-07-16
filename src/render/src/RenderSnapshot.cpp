#include <enginelab/render/RenderSnapshot.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <utility>

namespace enginelab {
namespace {

[[nodiscard]] double bankAngleFor(const EngineConfig& config,
                                  const CylinderConfig& cylinder) noexcept {
    const auto bank = std::find_if(config.banks.begin(), config.banks.end(),
        [&cylinder](const CylinderBankConfig& item) {
            return item.id == cylinder.bankId
                || std::find(item.cylinderIds.begin(), item.cylinderIds.end(), cylinder.id)
                    != item.cylinderIds.end();
        });
    return bank != config.banks.end() ? bank->angleDegrees : cylinder.bankOffsetDegrees;
}

[[nodiscard]] double lerpAngle(double from, double to, double amount) noexcept {
    return from + std::remainder(to - from, 360.0) * amount;
}

[[nodiscard]] RenderVector3 lerp(const RenderVector3& from,
                                 const RenderVector3& to,
                                 double amount) noexcept {
    return { std::lerp(from.x, to.x, amount), std::lerp(from.y, to.y, amount),
             std::lerp(from.z, to.z, amount) };
}

void includeBounds(RenderSnapshot& snapshot, const RenderVector3& point) noexcept {
    snapshot.boundsMinimumMm.x = std::min(snapshot.boundsMinimumMm.x, point.x);
    snapshot.boundsMinimumMm.y = std::min(snapshot.boundsMinimumMm.y, point.y);
    snapshot.boundsMinimumMm.z = std::min(snapshot.boundsMinimumMm.z, point.z);
    snapshot.boundsMaximumMm.x = std::max(snapshot.boundsMaximumMm.x, point.x);
    snapshot.boundsMaximumMm.y = std::max(snapshot.boundsMaximumMm.y, point.y);
    snapshot.boundsMaximumMm.z = std::max(snapshot.boundsMaximumMm.z, point.z);
}

void append(RenderSnapshot& snapshot, RenderPartState part) noexcept {
    if (snapshot.partCount >= snapshot.parts.size()) return;
    const auto radius = 0.5 * std::max({ std::abs(part.transform.scale.x),
                                         std::abs(part.transform.scale.y),
                                         std::abs(part.transform.scale.z) });
    includeBounds(snapshot, { part.transform.positionMm.x - radius,
                              part.transform.positionMm.y - radius,
                              part.transform.positionMm.z - radius });
    includeBounds(snapshot, { part.transform.positionMm.x + radius,
                              part.transform.positionMm.y + radius,
                              part.transform.positionMm.z + radius });
    snapshot.parts[snapshot.partCount++] = part;
}

} // namespace

RenderSnapshotBuilder::RenderSnapshotBuilder(EngineConfig config, RenderLayout layout)
    : config_(std::move(config)), layout_(layout) {
    normaliseEngineConfig(config_);
    if (const auto error = validateEngineConfig(config_)) throw std::invalid_argument(*error);
    layout_.cylinderSpacingMm = std::max(1.0, layout_.cylinderSpacingMm);
    layout_.nominalCylinderLengthMm = std::max(1.0, layout_.nominalCylinderLengthMm);
    layout_.nominalValveLengthMm = std::max(1.0, layout_.nominalValveLengthMm);
    cylinderStationsZ_.resize(config_.cylinders.size(), 0.0);

    // Cylinders in the same bank occupy successive longitudinal stations.
    // Equal-sized opposing banks consequently share stations, as a V/flat
    // engine normally does, instead of being interleaved arbitrarily by ID.
    for (std::size_t cylinderIndex = 0; cylinderIndex < config_.cylinders.size(); ++cylinderIndex) {
        const auto& cylinder = config_.cylinders[cylinderIndex];
        const auto bank = std::find_if(config_.banks.begin(), config_.banks.end(),
            [&cylinder](const CylinderBankConfig& item) {
                return item.id == cylinder.bankId
                    || std::find(item.cylinderIds.begin(), item.cylinderIds.end(), cylinder.id)
                        != item.cylinderIds.end();
            });
        if (bank == config_.banks.end() || bank->cylinderIds.empty()) {
            cylinderStationsZ_[cylinderIndex] =
                (static_cast<double>(cylinderIndex)
                    - 0.5 * static_cast<double>(config_.cylinders.size() - 1U))
                * layout_.cylinderSpacingMm;
            continue;
        }
        const auto station = std::find(bank->cylinderIds.begin(), bank->cylinderIds.end(), cylinder.id);
        const auto position = station == bank->cylinderIds.end()
            ? 0.0 : static_cast<double>(std::distance(bank->cylinderIds.begin(), station));
        cylinderStationsZ_[cylinderIndex] =
            (position - 0.5 * static_cast<double>(bank->cylinderIds.size() - 1U))
            * layout_.cylinderSpacingMm;
    }
}

double RenderSnapshotBuilder::cylinderStationZ(std::size_t cylinderIndex) const noexcept {
    return cylinderIndex < cylinderStationsZ_.size() ? cylinderStationsZ_[cylinderIndex] : 0.0;
}

RenderSnapshot RenderSnapshotBuilder::build(const EngineState& state) const noexcept {
    RenderSnapshot snapshot;
    snapshot.simulationTimeSeconds = state.simulationTimeSeconds;
    snapshot.crankAngleDegrees = state.crankAngleDegrees;
    snapshot.rpm = state.rpm;
    const auto infinity = std::numeric_limits<double>::infinity();
    snapshot.boundsMinimumMm = { infinity, infinity, infinity };
    snapshot.boundsMaximumMm = { -infinity, -infinity, -infinity };

    const auto stationBounds = std::minmax_element(cylinderStationsZ_.begin(), cylinderStationsZ_.end());
    const auto crankshaftCentreZ = cylinderStationsZ_.empty()
        ? 0.0 : 0.5 * (*stationBounds.first + *stationBounds.second);
    const auto crankshaftLength = cylinderStationsZ_.empty() ? layout_.cylinderSpacingMm
        : (*stationBounds.second - *stationBounds.first) + layout_.cylinderSpacingMm;
    for (const auto& crankshaft : config_.crankshafts) {
        append(snapshot, { RenderPartKind::crankshaft, crankshaft.id, 0U, 0U,
            { { crankshaft.positionXMm, crankshaft.positionYMm, crankshaftCentreZ },
              { 0.0, 0.0, state.crankAngleDegrees * crankshaft.rotationRatio
                    + crankshaft.phaseOffsetDegrees },
              { 1.0, 1.0, std::max(1.0, crankshaftLength) } },
            0.0, state.oilTemperatureC });
    }

    const auto count = std::min(config_.cylinders.size(), state.cylinderStateCount);
    for (std::size_t index = 0; index < count; ++index) {
        const auto& cylinder = config_.cylinders[index];
        const auto& live = state.cylinderStates[index];
        const auto z = cylinderStationZ(index);
        const auto bankAngle = bankAngleFor(config_, cylinder);
        const auto axisRadians = (bankAngle - 90.0) * std::numbers::pi / 180.0;
        const RenderVector3 crank { live.crankPinXMm, live.crankPinYMm, z };
        const RenderVector3 wrist { live.wristPinXMm, live.wristPinYMm, z };
        const auto rodDx = wrist.x - crank.x;
        const auto rodDy = wrist.y - crank.y;
        const auto rodLength = std::max(1.0, std::hypot(rodDx, rodDy));
        const auto rodAngle = std::atan2(rodDy, rodDx) * 180.0 / std::numbers::pi - 90.0;
        const RenderVector3 axis { std::cos(axisRadians), std::sin(axisRadians), 0.0 };
        // pistonTravelMm is measured from the resolved, configuration-specific
        // TDC. Reconstructing that fixed reference keeps the block and head
        // stationary while only the crank train moves.
        const RenderVector3 topWrist {
            wrist.x + axis.x * live.pistonTravelMm,
            wrist.y + axis.y * live.pistonTravelMm,
            z
        };
        const auto cylinderOrigin = RenderVector3 {
            topWrist.x - axis.x * 0.5 * layout_.nominalCylinderLengthMm,
            topWrist.y - axis.y * 0.5 * layout_.nominalCylinderLengthMm,
            z
        };
        const auto partRotation = bankAngle + 180.0;

        append(snapshot, { RenderPartKind::cylinder, cylinder.id, cylinder.id, cylinder.bankId,
            { cylinderOrigin, { 0.0, 0.0, partRotation },
              { cylinder.boreMm, layout_.nominalCylinderLengthMm, cylinder.boreMm } },
            live.combustionPulse, live.gasTemperatureC });
        append(snapshot, { RenderPartKind::crankJournal, live.crankJournalId, cylinder.id,
            live.crankshaftId, { crank, {}, { 1.0, 1.0, 1.0 } },
            0.0, state.oilTemperatureC });
        append(snapshot, { RenderPartKind::connectingRod, cylinder.id, cylinder.id,
            live.crankJournalId,
            { { 0.5 * (crank.x + wrist.x), 0.5 * (crank.y + wrist.y), z },
              { 0.0, 0.0, rodAngle },
              { std::max(1.0, cylinder.connectingRodMassGrams / 180.0), rodLength,
                std::max(1.0, cylinder.connectingRodMassGrams / 180.0) } },
            0.0, state.oilTemperatureC });
        append(snapshot, { RenderPartKind::piston, cylinder.id, cylinder.id, cylinder.bankId,
            { { wrist.x + axis.x * 0.5 * cylinder.compressionHeightMm,
                wrist.y + axis.y * 0.5 * cylinder.compressionHeightMm, z },
              { 0.0, 0.0, partRotation },
              { cylinder.boreMm * 0.94, std::max(12.0, cylinder.compressionHeightMm),
                cylinder.boreMm * 0.94 } },
            live.combustionPulse, live.gasTemperatureC });

        const auto valveBaseX = topWrist.x + axis.x * 0.08 * layout_.nominalCylinderLengthMm;
        const auto valveBaseY = topWrist.y + axis.y * 0.08 * layout_.nominalCylinderLengthMm;
        const auto valveOffset = cylinder.boreMm * 0.18;
        append(snapshot, { RenderPartKind::intakeValve, cylinder.id, cylinder.id, cylinder.bankId,
            { { valveBaseX - axis.y * valveOffset - axis.x * live.intakeValveLiftMm,
                valveBaseY + axis.x * valveOffset - axis.y * live.intakeValveLiftMm, z },
              { 0.0, 0.0, partRotation },
              { 1.0, layout_.nominalValveLengthMm, 1.0 } },
            live.intakeValveLiftMm, live.gasTemperatureC });
        append(snapshot, { RenderPartKind::exhaustValve, cylinder.id, cylinder.id, cylinder.bankId,
            { { valveBaseX + axis.y * valveOffset - axis.x * live.exhaustValveLiftMm,
                valveBaseY - axis.x * valveOffset - axis.y * live.exhaustValveLiftMm, z },
              { 0.0, 0.0, partRotation },
              { 1.0, layout_.nominalValveLengthMm, 1.0 } },
            live.exhaustValveLiftMm, live.exhaustTemperatureC });
    }

    if (snapshot.partCount == 0) {
        snapshot.boundsMinimumMm = {};
        snapshot.boundsMaximumMm = {};
    }
    return snapshot;
}

void RenderSnapshotInterpolator::reset() noexcept {
    previous_.reset();
    current_.reset();
}

void RenderSnapshotInterpolator::push(RenderSnapshot snapshot) noexcept {
    if (current_ && snapshot.simulationTimeSeconds < current_->simulationTimeSeconds) reset();
    previous_ = std::move(current_);
    current_ = std::move(snapshot);
}

RenderSnapshot RenderSnapshotInterpolator::sample(double simulationTimeSeconds) const noexcept {
    if (!current_) return {};
    if (!previous_ || current_->simulationTimeSeconds <= previous_->simulationTimeSeconds)
        return *current_;
    const auto amount = std::clamp((simulationTimeSeconds - previous_->simulationTimeSeconds)
        / (current_->simulationTimeSeconds - previous_->simulationTimeSeconds), 0.0, 1.0);
    auto result = *current_;
    result.simulationTimeSeconds = std::lerp(previous_->simulationTimeSeconds,
                                             current_->simulationTimeSeconds, amount);
    result.crankAngleDegrees = lerpAngle(previous_->crankAngleDegrees,
                                         current_->crankAngleDegrees, amount);
    result.rpm = std::lerp(previous_->rpm, current_->rpm, amount);
    if (previous_->partCount != current_->partCount) return result;
    for (std::size_t index = 0; index < result.partCount; ++index) {
        const auto& from = previous_->parts[index];
        const auto& to = current_->parts[index];
        if (from.kind != to.kind || from.id != to.id || from.cylinderId != to.cylinderId)
            continue;
        auto& part = result.parts[index];
        part.transform.positionMm = lerp(from.transform.positionMm, to.transform.positionMm, amount);
        part.transform.scale = lerp(from.transform.scale, to.transform.scale, amount);
        part.transform.rotation.xDegrees = lerpAngle(from.transform.rotation.xDegrees,
                                                      to.transform.rotation.xDegrees, amount);
        part.transform.rotation.yDegrees = lerpAngle(from.transform.rotation.yDegrees,
                                                      to.transform.rotation.yDegrees, amount);
        part.transform.rotation.zDegrees = lerpAngle(from.transform.rotation.zDegrees,
                                                      to.transform.rotation.zDegrees, amount);
        part.activity = std::lerp(from.activity, to.activity, amount);
        part.temperatureC = std::lerp(from.temperatureC, to.temperatureC, amount);
    }
    return result;
}

} // namespace enginelab
