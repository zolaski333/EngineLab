#include <enginelab/exhaust/LegacyExhaustNetwork.hpp>

#include <algorithm>

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

} // namespace

ExhaustNetworkConfig makeEditableExhaustNetwork(const ExhaustPathConfig& path) {
    ExhaustNetworkConfig network;
    network.components.reserve(path.cylinderIds.size() + 3U);
    network.cylinderConnections.reserve(path.cylinderIds.size());
    network.connections.reserve(path.cylinderIds.size() + 2U);

    std::uint32_t nextId = 1;
    std::vector<std::uint32_t> primaryIds;
    primaryIds.reserve(path.cylinderIds.size());
    for (const auto cylinderId : path.cylinderIds) {
        auto primary = component(ExhaustComponentType::pipe, nextId++);
        primary.lengthMm = path.geometry.primaryLengthMm;
        primary.diameterMm = path.geometry.primaryDiameterMm;
        primaryIds.push_back(primary.id);
        network.cylinderConnections.push_back({ cylinderId, primary.id });
        network.components.push_back(primary);
    }

    std::uint32_t previousId = 0;
    if (primaryIds.size() > 1U) {
        auto collector = component(ExhaustComponentType::merge, nextId++);
        collector.lengthMm = 0.0;
        collector.diameterMm = path.geometry.collectorDiameterMm;
        collector.volumeLitres = path.geometry.collectorVolumeLitres;
        previousId = collector.id;
        network.components.push_back(collector);
        for (const auto primaryId : primaryIds)
            network.connections.push_back({ primaryId, collector.id });
    } else if (!primaryIds.empty()) {
        previousId = primaryIds.front();
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
    // Not an approximation: there is nothing there to model.
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
        muffler.diameterMm = path.geometry.mufflerChamberDiameterMm;
        muffler.lengthMm = path.geometry.mufflerChamberLengthMm;
        muffler.volumeLitres = std::max(0.05, path.geometry.collectorVolumeLitres);
        muffler.packingFlowResistivityPaSPerM2 =
            path.geometry.mufflerPackingFlowResistivityPaSPerM2;
        muffler.packingThicknessMm = path.geometry.mufflerPackingThicknessMm;
        muffler.perforatedOpenAreaRatio =
            path.geometry.mufflerPerforatedOpenAreaRatio;
        network.components.push_back(muffler);
        if (previousId != 0)
            network.connections.push_back({ previousId, muffler.id });
        previousId = muffler.id;
    }

    auto outlet = component(ExhaustComponentType::outlet, nextId);
    outlet.lengthMm = 0.0;
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

} // namespace enginelab
