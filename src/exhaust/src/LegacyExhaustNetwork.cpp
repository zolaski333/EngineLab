#include <enginelab/exhaust/LegacyExhaustNetwork.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace enginelab {
namespace {

[[nodiscard]] ExhaustComponentConfig component(
    ExhaustComponentType type, std::uint32_t id) noexcept {
    ExhaustComponentConfig result;
    result.id = id;
    result.type = type;
    result.acousticGain = 1.0;
    result.restriction = 0.0;
    return result;
}

/** The scalar geometry's straight body length where no chamber scatters:
 * the solver's compiler meshes 450 mm there, or a configured but narrow
 * chamber's own length floored at its plane-wave limit (ExhaustGraph). */
[[nodiscard]] double straightBodyLengthMm(const ExhaustConfig& geometry) noexcept {
    const auto chamberConfigured = geometry.mufflerChamberDiameterMm > 1.0
        && geometry.mufflerChamberLengthMm > 1.0;
    if (!chamberConfigured) return 450.0;
    constexpr double planeWaveResonantLengthRatio = 0.853;
    const auto flowDiameterMm = std::clamp(geometry.mufflerChamberDiameterMm,
        geometry.collectorDiameterMm, 3'000.0);
    return std::max(std::min(geometry.mufflerChamberLengthMm, 15'000.0),
                    flowDiameterMm * planeWaveResonantLengthRatio);
}

[[nodiscard]] ExhaustNetworkConfig build(const ExhaustPathConfig& path,
                                         std::span<const CylinderConfig> cylinders,
                                         std::uint32_t* midpipeIdOut) {
    ExhaustNetworkConfig network;
    network.components.reserve(path.cylinderIds.size() + 5U);
    network.cylinderConnections.reserve(path.cylinderIds.size());
    network.connections.reserve(path.cylinderIds.size() + 4U);
    if (midpipeIdOut != nullptr) *midpipeIdOut = 0;

    std::uint32_t nextId = 1;
    std::vector<std::uint32_t> primaryIds;
    primaryIds.reserve(path.cylinderIds.size());
    for (const auto cylinderId : path.cylinderIds) {
        auto primary = component(ExhaustComponentType::pipe, nextId++);
        // A cylinder's own primary length wins, as in the solver's compiler.
        const auto cylinder = std::find_if(cylinders.begin(), cylinders.end(),
            [cylinderId](const CylinderConfig& item) { return item.id == cylinderId; });
        primary.lengthMm = cylinder != cylinders.end() && cylinder->exhaustPrimaryLengthMm > 0.0
            ? cylinder->exhaustPrimaryLengthMm : path.geometry.primaryLengthMm;
        primary.diameterMm = path.geometry.primaryDiameterMm;
        primaryIds.push_back(primary.id);
        network.cylinderConnections.push_back({ cylinderId, primary.id });
        network.components.push_back(primary);
    }

    std::uint32_t previousId = 0;
    if (primaryIds.size() > 1U) {
        auto collector = component(ExhaustComponentType::merge, nextId++);
        // The collector is its trunk, as the solver's own geometry has it:
        // without an authored body it holds what the trunk sweeps (zero
        // volume); an authored body is the trunk's volume.
        collector.lengthMm = legacyCollectorTrunkLengthMm;
        collector.diameterMm = path.geometry.collectorDiameterMm;
        collector.volumeLitres = path.geometry.collectorVolumeLitres;
        previousId = collector.id;
        network.components.push_back(collector);
        for (const auto primaryId : primaryIds)
            network.connections.push_back({ primaryId, collector.id });
    } else if (!primaryIds.empty()) {
        // One primary has nothing to merge, but the solver still runs it
        // through the collector's trunk.
        auto trunk = component(ExhaustComponentType::pipe, nextId++);
        trunk.lengthMm = legacyCollectorTrunkLengthMm;
        trunk.diameterMm = path.geometry.collectorDiameterMm;
        network.components.push_back(trunk);
        network.connections.push_back({ primaryIds.front(), trunk.id });
        previousId = trunk.id;
    }

    if (path.geometry.midpipeLengthMm > 0.0) {
        auto midpipe = component(ExhaustComponentType::pipe, nextId++);
        midpipe.lengthMm = path.geometry.midpipeLengthMm;
        midpipe.diameterMm = path.geometry.midpipeDiameterMm > 0.0
            ? path.geometry.midpipeDiameterMm
            : primaryIds.size() > 1U ? path.geometry.collectorDiameterMm
                                     : path.geometry.primaryDiameterMm;
        network.components.push_back(midpipe);
        if (previousId != 0)
            network.connections.push_back({ previousId, midpipe.id });
        previousId = midpipe.id;
        if (midpipeIdOut != nullptr) *midpipeIdOut = midpipe.id;
    }

    // An authored expansion chamber is geometry, not the editor's generic
    // muffler preset. The old UI silently replaced it with a 480 x collector-
    // diameter body, a 28% broadband loss and a live graph restriction. That
    // changed both the sound and the gas flow as soon as the user saved.
    //
    // A body no wider than the pipes it sits between is not a chamber. An
    // expansion chamber silences by SCATTERING at its two area steps, and its
    // transmission loss is a function of the area ratio alone -- exactly 0 dB at
    // ratio 1. So showing the user a silencer component there would be showing
    // something that provably does nothing, and a user who authors it (a common
    // shape when every diameter is set to the same number) is entitled to see
    // that the exhaust has no silencer rather than a silencer with no effect.
    // Not an approximation: there is nothing there to model. It is still a
    // length of pipe the solver meshes, so it is drawn as one.
    //
    // A body that is merely SHORT is a separate matter and is deliberately not
    // filtered here. `ExhaustGraph` floors its meshed length at the body's own
    // plane-wave resolution limit instead, which keeps the authored component
    // visible in the editor while stopping it from setting the CFL limit for the
    // whole gas network.
    const auto upstreamDiameterMm = primaryIds.size() > 1U
        ? path.geometry.collectorDiameterMm : path.geometry.primaryDiameterMm;
    const auto scatteringDiameterMm = std::max(upstreamDiameterMm,
                                               path.geometry.outletDiameterMm);
    const auto chamberConfigured = path.geometry.mufflerChamberDiameterMm > 1.0
        && path.geometry.mufflerChamberLengthMm > 1.0
        && path.geometry.mufflerChamberDiameterMm > scatteringDiameterMm;
    if (chamberConfigured) {
        auto muffler = component(ExhaustComponentType::muffler, nextId++);
        muffler.lengthMm = path.geometry.mufflerChamberLengthMm;
        muffler.packingFlowResistivityPaSPerM2 =
            path.geometry.mufflerPackingFlowResistivityPaSPerM2;
        muffler.packingThicknessMm = path.geometry.mufflerPackingThicknessMm;
        muffler.perforatedOpenAreaRatio =
            path.geometry.mufflerPerforatedOpenAreaRatio;
        const auto packedCore = muffler.packingFlowResistivityPaSPerM2 > 0.0
            && muffler.packingThicknessMm > 0.0
            && muffler.perforatedOpenAreaRatio > 0.0;
        // `diameterMm` is the gas passage. A dry reactive chamber passes gas
        // through its expanded body; a packed straight-through can passes it
        // through the perforated core and keeps the larger body in volumeLitres
        // as acoustic annular storage. This is the same representation emitted
        // by ExhaustGraph's direct legacy compiler, so merely opening/applying
        // the editor cannot change either flow area or sound.
        muffler.diameterMm = packedCore
            ? upstreamDiameterMm : path.geometry.mufflerChamberDiameterMm;
        const auto bodyRadiusM = path.geometry.mufflerChamberDiameterMm * 0.0005;
        muffler.volumeLitres = std::numbers::pi * bodyRadiusM * bodyRadiusM
            * path.geometry.mufflerChamberLengthMm;
        muffler.restriction = path.geometry.mufflerRestriction;
        network.components.push_back(muffler);
        if (previousId != 0)
            network.connections.push_back({ previousId, muffler.id });
        previousId = muffler.id;
    } else {
        auto body = component(ExhaustComponentType::pipe, nextId++);
        body.lengthMm = straightBodyLengthMm(path.geometry);
        body.diameterMm = path.geometry.collectorDiameterMm;
        body.restriction = path.geometry.mufflerRestriction;
        network.components.push_back(body);
        if (previousId != 0)
            network.connections.push_back({ previousId, body.id });
        previousId = body.id;
    }

    auto outlet = component(ExhaustComponentType::outlet, nextId);
    outlet.lengthMm = legacyTailpipeLengthMm;
    outlet.diameterMm = path.geometry.outletDiameterMm;
    outlet.dischargeCoefficient = path.geometry.outletDischargeCoefficient;
    outlet.acousticPositionM = path.acousticPositionM;
    outlet.acousticAxis = path.acousticAxis;
    outlet.acousticTermination = path.acousticTermination;
    network.components.push_back(outlet);
    if (previousId != 0)
        network.connections.push_back({ previousId, outlet.id });
    return network;
}

} // namespace

ExhaustNetworkConfig makeEditableExhaustNetwork(const ExhaustPathConfig& path,
                                                std::span<const CylinderConfig> cylinders) {
    return build(path, cylinders, nullptr);
}

std::uint32_t editableMidpipeId(const ExhaustPathConfig& path) {
    std::uint32_t midpipeId = 0;
    static_cast<void>(build(path, {}, &midpipeId));
    return midpipeId;
}

} // namespace enginelab
