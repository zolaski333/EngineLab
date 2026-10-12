#include <enginelab/serialization/EngineEditScope.hpp>
#include <enginelab/serialization/JsonEngineSerializer.hpp>

#include <exception>
#include <string>

namespace enginelab {
namespace {

void takeExhaust(EngineConfig& probe, const EngineConfig& edited) {
    probe.exhaust = edited.exhaust;
    probe.exhaustPaths = edited.exhaustPaths;
}

void takeSettings(EngineConfig& probe, const EngineConfig& edited) {
    probe.injection = edited.injection;
    probe.forcedInduction = edited.forcedInduction;
    probe.ignition = edited.ignition;
    probe.exhaustAfterfire = edited.exhaustAfterfire;
    probe.crankshafts = edited.crankshafts;
    probe.rotatingInertiaKgM2 = edited.rotatingInertiaKgM2;
    probe.ambientPressureKpa = edited.ambientPressureKpa;
    probe.ambientTemperatureC = edited.ambientTemperatureC;
}

/** False when the banks are not the same ones: that edit is not a camshaft
 *  change. */
bool takeCamshafts(EngineConfig& probe, const EngineConfig& edited) {
    if (probe.banks.size() != edited.banks.size()) return false;
    for (std::size_t index = 0; index < probe.banks.size(); ++index) {
        if (probe.banks[index].id != edited.banks[index].id) return false;
        probe.banks[index].camshafts = edited.banks[index].camshafts;
    }
    probe.camshafts = edited.camshafts;
    return true;
}

/** False when the cylinders are not the same ones: that edit is not an
 *  intake change. */
bool takeIntake(EngineConfig& probe, const EngineConfig& edited) {
    if (probe.cylinders.size() != edited.cylinders.size()) return false;
    for (std::size_t index = 0; index < probe.cylinders.size(); ++index) {
        const auto& source = edited.cylinders[index];
        auto& cylinder = probe.cylinders[index];
        if (source.id != cylinder.id) return false;
        cylinder.intakeRunnerLengthMm = source.intakeRunnerLengthMm;
        cylinder.intakeRunnerDiameterMm = source.intakeRunnerDiameterMm;
    }
    probe.intake = edited.intake;
    probe.intakePaths = edited.intakePaths;
    probe.plenumVolumeLitres = edited.plenumVolumeLitres;
    probe.throttleDiameterMm = edited.throttleDiameterMm;
    return true;
}

/** False when the cylinders or journals are not the same ones: that edit is
 *  not a resize. */
bool takeCylinderSizes(EngineConfig& probe, const EngineConfig& edited) {
    if (probe.cylinders.size() != edited.cylinders.size()
        || probe.crankJournals.size() != edited.crankJournals.size())
        return false;
    for (std::size_t index = 0; index < probe.cylinders.size(); ++index) {
        const auto& source = edited.cylinders[index];
        auto& cylinder = probe.cylinders[index];
        if (source.id != cylinder.id || source.crankJournalId != cylinder.crankJournalId)
            return false;
        cylinder.boreMm = source.boreMm;
        cylinder.strokeMm = source.strokeMm;
        cylinder.deckHeightMm = source.deckHeightMm;
        cylinder.connectingRodMm = source.connectingRodMm;
        cylinder.compressionRatio = source.compressionRatio;
    }
    for (std::size_t index = 0; index < probe.crankJournals.size(); ++index) {
        if (edited.crankJournals[index].id != probe.crankJournals[index].id) return false;
        probe.crankJournals[index].throwMm = edited.crankJournals[index].throwMm;
    }
    return true;
}

} // namespace

EngineEditScope engineEditScope(const EngineConfig& running, const EngineConfig& edited) {
    EngineEditScope scope;
    const JsonEngineSerializer json;
    try {
        auto was = running;
        auto now = edited;
        normaliseEngineConfig(was);
        normaliseEngineConfig(now);
        const auto target = json.encode(now);
        const auto unchanged = json.encode(was);
        if (target == unchanged) return scope;
        // One group at a time onto the running engine, normalised again, since
        // normalisation may derive fields from the group.
        const auto differs = [&](auto&& take) {
            auto probe = was;
            if (!take(probe)) return true;
            normaliseEngineConfig(probe);
            return json.encode(probe) != unchanged;
        };
        scope.exhaust = differs([&](EngineConfig& probe) { takeExhaust(probe, now); return true; });
        scope.settings = differs([&](EngineConfig& probe) { takeSettings(probe, now); return true; });
        scope.intake = differs([&](EngineConfig& probe) { return takeIntake(probe, now); });
        scope.camshafts = differs([&](EngineConfig& probe) { return takeCamshafts(probe, now); });
        auto probe = was;
        takeExhaust(probe, now);
        takeSettings(probe, now);
        // Banks of other cylinders stay in the difference: `other`.
        (void)takeCamshafts(probe, now);
        const auto resizable = takeCylinderSizes(probe, now) && takeIntake(probe, now);
        scope.cylinders = resizable && differs([&](EngineConfig& sized) { return takeCylinderSizes(sized, now); });
        normaliseEngineConfig(probe);
        scope.other = !resizable || json.encode(probe) != target;
    } catch (const std::exception&) {
        scope.other = true;
    }
    return scope;
}

} // namespace enginelab
