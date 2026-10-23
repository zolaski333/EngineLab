#include <enginelab/foundation/IntakeResize.hpp>

#include <algorithm>
#include <cmath>
#include <string>

namespace enginelab {
namespace {

template <typename Config>
[[nodiscard]] auto* pathOf(Config& config, std::uint32_t pathId) {
    const auto path = std::find_if(config.intakePaths.begin(), config.intakePaths.end(),
                                   [pathId](const IntakePathConfig& item) { return item.id == pathId; });
    return path != config.intakePaths.end() ? &*path : nullptr;
}

[[nodiscard]] bool within(double value, double low, double high) {
    return std::isfinite(value) && value >= low && value <= high;
}

/** One path mirrors the engine-wide intake (normaliseEngineConfig), from
    whichever side it inherits. */
void mirrorSinglePath(EngineConfig& config) {
    if (config.intakePaths.size() != 1) return;
    const auto& geometry = config.intakePaths.front().geometry;
    config.intake = geometry;
    config.plenumVolumeLitres = geometry.plenumVolumeLitres;
    config.throttleDiameterMm = geometry.throttleDiameterMm;
}

} // namespace

std::optional<IntakePartSize> intakePartSize(const EngineConfig& config, std::uint32_t pathId,
                                             IntakePart part, std::size_t cylinderIndex) {
    const auto* path = pathOf(config, pathId);
    if (path == nullptr) return std::nullopt;
    const auto& geometry = path->geometry;
    IntakePartSize size;
    switch (part) {
    case IntakePart::runner: {
        const auto* cylinder = cylinderIndex < config.cylinders.size() ? &config.cylinders[cylinderIndex] : nullptr;
        const auto ownLength = cylinder != nullptr && cylinder->intakeRunnerLengthMm > 0.0;
        const auto ownDiameter = cylinder != nullptr && cylinder->intakeRunnerDiameterMm > 0.0;
        size.first = ownLength ? cylinder->intakeRunnerLengthMm : geometry.runnerLengthMm;
        size.second = ownDiameter ? cylinder->intakeRunnerDiameterMm : geometry.runnerDiameterMm;
        size.sharedByRunners = !ownLength && !ownDiameter && path->cylinderIds.size() > 1;
        break;
    }
    case IntakePart::plenum: size.first = geometry.plenumVolumeLitres; break;
    case IntakePart::throttle: size.first = geometry.throttleDiameterMm; break;
    }
    return size;
}

std::string resizeIntakePart(EngineConfig& config, std::uint32_t pathId, IntakePart part,
                             std::size_t cylinderIndex, double first, double second) {
    auto* path = pathOf(config, pathId);
    if (path == nullptr) return "The engine has no intake path " + std::to_string(pathId) + ".";
    auto& geometry = path->geometry;
    switch (part) {
    case IntakePart::runner: {
        if (!within(first, 30.0, 2'000.0)) return "A runner is 30 to 2,000 mm long.";
        if (!within(second, 10.0, 200.0)) return "A runner is 10 to 200 mm across.";
        auto* cylinder = cylinderIndex < config.cylinders.size() ? &config.cylinders[cylinderIndex] : nullptr;
        // The cylinder's own size where it has one; the path's, which every
        // runner without its own shares, otherwise.
        if (cylinder != nullptr && cylinder->intakeRunnerLengthMm > 0.0) cylinder->intakeRunnerLengthMm = first;
        else geometry.runnerLengthMm = first;
        if (cylinder != nullptr && cylinder->intakeRunnerDiameterMm > 0.0) cylinder->intakeRunnerDiameterMm = second;
        else geometry.runnerDiameterMm = second;
        break;
    }
    case IntakePart::plenum:
        if (!within(first, 0.05, 40.0)) return "A plenum holds 0.05 to 40 litres.";
        geometry.plenumVolumeLitres = first;
        break;
    case IntakePart::throttle:
        if (!within(first, 10.0, 150.0)) return "A throttle bore is 10 to 150 mm.";
        geometry.throttleDiameterMm = first;
        break;
    }
    mirrorSinglePath(config);
    return {};
}

std::optional<CarburettorTune> carburettorTune(const EngineConfig& config, std::uint32_t pathId) {
    const auto* path = pathOf(config, pathId);
    if (path == nullptr || !path->geometry.carburettor.enabled) return std::nullopt;
    const auto& carburettor = path->geometry.carburettor;
    return CarburettorTune { carburettor.mainJetFlowNumber, carburettor.pilotJetFlowNumber,
                             carburettor.needleClipPosition, carburettor.airScrewTurns };
}

std::string tuneCarburettor(EngineConfig& config, std::uint32_t pathId, const CarburettorTune& tune) {
    auto* path = pathOf(config, pathId);
    if (path == nullptr) return "The engine has no intake path " + std::to_string(pathId) + ".";
    auto& carburettor = path->geometry.carburettor;
    if (!carburettor.enabled) return "Intake path " + std::to_string(pathId) + " has no carburettor.";
    if (!within(tune.mainJetFlowNumber, 50.0, 1'000.0)) return "A main jet is a 50 to a 1,000.";
    if (!within(tune.pilotJetFlowNumber, 10.0, 150.0)) return "A pilot jet is a 10 to a 150.";
    if (!within(tune.needleClipPosition, 1.0, 5.0)) return "The needle clip is in position 1 to 5.";
    if (!within(tune.airScrewTurns, 0.0, 5.0)) return "The air screw is 0 to 5 turns out.";
    carburettor.mainJetFlowNumber = tune.mainJetFlowNumber;
    carburettor.pilotJetFlowNumber = tune.pilotJetFlowNumber;
    carburettor.needleClipPosition = tune.needleClipPosition;
    carburettor.airScrewTurns = tune.airScrewTurns;
    mirrorSinglePath(config);
    return {};
}

} // namespace enginelab
