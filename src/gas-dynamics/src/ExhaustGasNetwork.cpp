#include <enginelab/gasdynamics/ExhaustGasNetwork.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace enginelab::gasdynamics {
namespace {

[[nodiscard]] bool finite(double value) noexcept {
    return std::isfinite(value);
}

[[nodiscard]] ConservativeState addScaled(const ConservativeState& first,
                                           const ConservativeState& second,
                                           double scale) noexcept {
    auto result = first;
    for (std::size_t index = 0; index < gasSpeciesCount; ++index) {
        result.speciesMassDensityKgPerM3[index] +=
            second.speciesMassDensityKgPerM3[index] * scale;
    }
    result.momentumDensityKgPerM2S += second.momentumDensityKgPerM2S * scale;
    result.totalEnergyDensityJPerM3 += second.totalEnergyDensityJPerM3 * scale;
    return result;
}

[[nodiscard]] ConservativeState rk2Combination(
    const ConservativeState& initial,
    const ConservativeState& firstStage,
    const ConservativeState& secondResidual,
    double timeStepSeconds) noexcept {
    ConservativeState result;
    for (std::size_t index = 0; index < gasSpeciesCount; ++index) {
        result.speciesMassDensityKgPerM3[index] = 0.5
            * (initial.speciesMassDensityKgPerM3[index]
               + std::fma(timeStepSeconds,
                          secondResidual.speciesMassDensityKgPerM3[index],
                          firstStage.speciesMassDensityKgPerM3[index]));
    }
    result.momentumDensityKgPerM2S = 0.5
        * (initial.momentumDensityKgPerM2S + firstStage.momentumDensityKgPerM2S
           + timeStepSeconds * secondResidual.momentumDensityKgPerM2S);
    result.totalEnergyDensityJPerM3 = 0.5
        * (initial.totalEnergyDensityJPerM3 + firstStage.totalEnergyDensityJPerM3
           + timeStepSeconds * secondResidual.totalEnergyDensityJPerM3);
    return result;
}

[[nodiscard]] EulerFlux areaAveragedBoundaryFlux(
    const EulerFlux& apertureFlux,
    double apertureAreaM2,
    double cellAreaM2,
    double cellPressurePa) noexcept {
    const auto openArea = std::clamp(apertureAreaM2, 0.0, cellAreaM2);
    const auto openFraction = cellAreaM2 > 0.0 ? openArea / cellAreaM2 : 0.0;
    EulerFlux result;
    for (std::size_t index = 0; index < gasSpeciesCount; ++index) {
        result.speciesMassFluxKgPerM2S[index] =
            apertureFlux.speciesMassFluxKgPerM2S[index] * openFraction;
    }
    // The closed part of a partially open face remains a pressure-loaded wall.
    result.momentumFluxPa = apertureFlux.momentumFluxPa * openFraction
        + cellPressurePa * (1.0 - openFraction);
    result.totalEnergyFluxWPerM2 = apertureFlux.totalEnergyFluxWPerM2 * openFraction;
    return result;
}

[[nodiscard]] double sumSpecies(
    const std::array<double, gasSpeciesCount>& species) noexcept {
    double result = 0.0;
    for (const auto value : species) result += value;
    return result;
}

} // namespace

bool ExhaustGasNetworkConfig::valid() const noexcept {
    return finite(initialPressurePa) && initialPressurePa > 0.0
        && finite(initialTemperatureK) && initialTemperatureK > 0.0
        && finite(absoluteRoughnessM) && absoluteRoughnessM >= 0.0
        && finite(wallHeatTransferWPerM2K) && wallHeatTransferWPerM2K >= 0.0
        && finite(wallTemperatureK) && wallTemperatureK > 0.0
        && finite(maximumCourantNumber) && maximumCourantNumber > 0.0
        && maximumCourantNumber <= 1.0 && maximumSubstepsPerAdvance > 0;
}

double CylinderGasExchange::totalMassKg() const noexcept {
    return sumSpecies(speciesMassKg);
}

ExhaustGasNetwork::ExhaustGasNetwork(ThermodynamicModel model) noexcept
    : mixtureModel_(model) {}

bool ExhaustGasNetwork::configure(const ExhaustNetworkLayout& layout,
                                  ExhaustGasNetworkConfig config) {
    configured_ = false;
    if (!layout.valid() || !config.valid()) return false;
    const auto initialState = mixtureModel_.conservativeFromPressureTemperature(
        config.initialPressurePa, config.initialTemperatureK, 0.0,
        config.initialComposition);
    if (!initialState) return false;

    layout_ = layout;
    config_ = config;
    ducts_.clear();
    ducts_.reserve(layout_.ducts().size());
    for (const auto& descriptor : layout_.ducts()) {
        DuctGeometry geometry;
        geometry.lengthM = descriptor.lengthM;
        geometry.crossSectionAreaM2 = descriptor.flowAreaM2;
        geometry.diameterM = descriptor.hydraulicDiameterM;
        geometry.cellCount = descriptor.cellCount;
        geometry.wallFrictionEnabled = true;
        geometry.absoluteRoughnessM = config.absoluteRoughnessM;
        geometry.localLossCoefficient = descriptor.lossCoefficient;
        geometry.wallHeatTransferWPerM2K = config.wallHeatTransferWPerM2K;
        geometry.wallTemperatureK = config.wallTemperatureK;
        ducts_.emplace_back(mixtureModel_.thermodynamics());
        if (!ducts_.back().configure(geometry, *initialState)) {
            ducts_.clear();
            return false;
        }
    }

    junctionStates_.assign(layout_.junctions().size(), *initialState);
    for (auto& state : junctionStates_) state.momentumDensityKgPerM2S = 0.0;
    junctionStage_.resize(junctionStates_.size());
    junctionCandidate_.resize(junctionStates_.size());
    junctionResidual_.resize(junctionStates_.size());
    junctionStageResidual_.resize(junctionStates_.size());
    junctionPortAreaSums_.assign(junctionStates_.size(), 0.0);
    cylinderReservoirStates_.resize(layout_.cylinderPorts().size());
    cylinderReservoirStage_.resize(layout_.cylinderPorts().size());
    cylinderReservoirCandidate_.resize(layout_.cylinderPorts().size());
    cylinderReservoirResidual_.resize(layout_.cylinderPorts().size());
    cylinderReservoirStageResidual_.resize(layout_.cylinderPorts().size());
    cylinderReservoirVolumesM3_.resize(layout_.cylinderPorts().size());
    cylinderReservoirActive_.resize(layout_.cylinderPorts().size());

    const auto endpointArea = [this](const ExhaustEndpoint& endpoint) noexcept {
        if (endpoint.type == ExhaustEndpointType::junction) {
            const auto diameter = layout_.junctions()[endpoint.elementIndex]
                .characteristicDiameterM;
            return 0.25 * std::acos(-1.0) * diameter * diameter;
        }
        return layout_.ducts()[endpoint.elementIndex].connectionAreaM2;
    };
    for (const auto& connection : layout_.interfaces()) {
        const auto area = std::min(endpointArea(connection.upstream),
                                   endpointArea(connection.downstream));
        if (connection.upstream.type == ExhaustEndpointType::junction)
            junctionPortAreaSums_[connection.upstream.elementIndex] += area;
        if (connection.downstream.type == ExhaustEndpointType::junction)
            junctionPortAreaSums_[connection.downstream.elementIndex] += area;
    }
    for (const auto& port : layout_.cylinderPorts()) {
        if (port.networkEndpoint.type == ExhaustEndpointType::junction) {
            junctionPortAreaSums_[port.networkEndpoint.elementIndex] +=
                std::min(port.runnerConnectionAreaM2, endpointArea(port.networkEndpoint));
        }
    }
    for (const auto& outlet : layout_.outlets()) {
        if (outlet.networkEndpoint.type == ExhaustEndpointType::junction) {
            junctionPortAreaSums_[outlet.networkEndpoint.elementIndex] +=
                std::min(outlet.openingAreaM2, endpointArea(outlet.networkEndpoint));
        }
    }

    // Every duct face must have exactly one physical owner. This is checked
    // once here so the realtime stage evaluator never has to resolve topology.
    std::vector<std::size_t> inletOwners(ducts_.size(), 0);
    std::vector<std::size_t> outletOwners(ducts_.size(), 0);
    const auto countEndpoint = [&inletOwners, &outletOwners](
        const ExhaustEndpoint& endpoint) noexcept {
        if (endpoint.type == ExhaustEndpointType::ductInlet)
            ++inletOwners[endpoint.elementIndex];
        else if (endpoint.type == ExhaustEndpointType::ductOutlet)
            ++outletOwners[endpoint.elementIndex];
    };
    for (const auto& connection : layout_.interfaces()) {
        countEndpoint(connection.upstream);
        countEndpoint(connection.downstream);
    }
    for (const auto& port : layout_.cylinderPorts()) countEndpoint(port.networkEndpoint);
    for (const auto& outlet : layout_.outlets()) countEndpoint(outlet.networkEndpoint);
    if (!std::all_of(inletOwners.begin(), inletOwners.end(),
            [](std::size_t count) { return count == 1; })
        || !std::all_of(outletOwners.begin(), outletOwners.end(),
            [](std::size_t count) { return count == 1; })) {
        ducts_.clear();
        return false;
    }

    ductInletAssigned_.resize(ducts_.size());
    ductOutletAssigned_.resize(ducts_.size());
    cylinderFirstStageFlow_.resize(layout_.cylinderPorts().size());
    cylinderSecondStageFlow_.resize(layout_.cylinderPorts().size());
    outletFirstStageFlow_.resize(layout_.outlets().size());
    outletSecondStageFlow_.resize(layout_.outlets().size());
    cylinderExchanges_.resize(layout_.cylinderPorts().size());
    for (std::size_t index = 0; index < cylinderExchanges_.size(); ++index) {
        cylinderExchanges_[index].cylinderId = layout_.cylinderPorts()[index].cylinderId;
        cylinderExchanges_[index].pathIndex = layout_.cylinderPorts()[index].pathIndex;
    }
    outletSamples_.resize(layout_.outlets().size());
    for (std::size_t index = 0; index < outletSamples_.size(); ++index) {
        outletSamples_[index].outletNodeId = layout_.outlets()[index].outletNodeId;
        outletSamples_[index].pathIndex = layout_.outlets()[index].pathIndex;
        outletSamples_[index].openingAreaM2 = layout_.outlets()[index].openingAreaM2;
    }
    configured_ = true;
    return true;
}

ExhaustNetworkInventory ExhaustGasNetwork::inventory() const noexcept {
    ExhaustNetworkInventory result;
    for (const auto& duct : ducts_) {
        const auto ductInventory = duct.inventory();
        for (std::size_t index = 0; index < gasSpeciesCount; ++index)
            result.speciesMassKg[index] += ductInventory.speciesMassKg[index];
        result.totalEnergyJ += ductInventory.totalEnergyJ;
        result.resolvedAxialMomentumKgMps += ductInventory.axialMomentumKgMps;
    }
    for (std::size_t index = 0; index < junctionStates_.size(); ++index) {
        const auto volume = layout_.junctions()[index].volumeM3;
        for (std::size_t species = 0; species < gasSpeciesCount; ++species) {
            result.speciesMassKg[species] +=
                junctionStates_[index].speciesMassDensityKgPerM3[species] * volume;
        }
        result.totalEnergyJ += junctionStates_[index].totalEnergyDensityJPerM3 * volume;
    }
    return result;
}

double ExhaustGasNetwork::maximumStableTimeStep(
    std::span<const CylinderValveBoundary> cylinderBoundaries,
    const ExhaustAmbientBoundary& ambient) const noexcept {
    auto stableStep = std::numeric_limits<double>::infinity();
    for (const auto& duct : ducts_) {
        stableStep = std::min(stableStep,
            duct.maximumStableTimeStep(config_.maximumCourantNumber));
    }
    for (std::size_t index = 0; index < junctionStates_.size(); ++index) {
        const auto primitive = mixtureModel_.primitiveFromConservative(junctionStates_[index]);
        if (!primitive) return 0.0;
        const auto areaSum = junctionPortAreaSums_[index];
        if (areaSum > 0.0) {
            stableStep = std::min(stableStep,
                config_.maximumCourantNumber * layout_.junctions()[index].volumeM3
                    / (areaSum * (std::abs(primitive->velocityMps)
                                  + primitive->speedOfSoundMps)));
        }
    }

    const auto endpointState = [this](const ExhaustEndpoint& endpoint)
        -> const ConservativeState& {
        if (endpoint.type == ExhaustEndpointType::junction)
            return junctionStates_[endpoint.elementIndex];
        const auto& cells = ducts_[endpoint.elementIndex].cells();
        return endpoint.type == ExhaustEndpointType::ductInlet
            ? cells.front() : cells.back();
    };
    const auto endpointVolume = [this](const ExhaustEndpoint& endpoint) noexcept {
        if (endpoint.type == ExhaustEndpointType::junction)
            return layout_.junctions()[endpoint.elementIndex].volumeM3;
        const auto& duct = ducts_[endpoint.elementIndex];
        return duct.geometry().areaM2() * duct.geometry().cellLengthM();
    };
    const auto constrainBoundary = [this, &stableStep, &endpointState, &endpointVolume](
        const ConservativeState& reservoir,
        const ExhaustEndpoint& endpoint,
        double openingAreaM2) noexcept {
        if (!(openingAreaM2 > 0.0)) return;
        const auto reservoirPrimitive = mixtureModel_.primitiveFromConservative(reservoir);
        const auto endpointPrimitive = mixtureModel_.primitiveFromConservative(endpointState(endpoint));
        if (!reservoirPrimitive || !endpointPrimitive) {
            stableStep = 0.0;
            return;
        }
        const auto signalSpeed = std::max(
            std::abs(reservoirPrimitive->velocityMps) + reservoirPrimitive->speedOfSoundMps,
            std::abs(endpointPrimitive->velocityMps) + endpointPrimitive->speedOfSoundMps);
        stableStep = std::min(stableStep,
            config_.maximumCourantNumber * endpointVolume(endpoint)
                / (openingAreaM2 * signalSpeed));
    };
    for (std::size_t index = 0; index < layout_.cylinderPorts().size(); ++index) {
        if (cylinderReservoirActive_[index] == 0) continue;
        const auto& port = layout_.cylinderPorts()[index];
        const auto boundary = std::find_if(cylinderBoundaries.begin(), cylinderBoundaries.end(),
            [&port](const CylinderValveBoundary& candidate) {
                return candidate.cylinderId == port.cylinderId;
            });
        if (boundary == cylinderBoundaries.end()) continue;
        const auto opening = std::min(
            std::max(0.0, boundary->effectiveValveAreaM2)
                * std::clamp(boundary->dischargeCoefficient, 0.0, 1.5),
            port.runnerConnectionAreaM2 * port.dischargeCoefficient);
        constrainBoundary(cylinderReservoirStates_[index], port.networkEndpoint, opening);
        const auto reservoirPrimitive = mixtureModel_.primitiveFromConservative(
            cylinderReservoirStates_[index]);
        const auto endpointPrimitive = mixtureModel_.primitiveFromConservative(
            endpointState(port.networkEndpoint));
        if (!reservoirPrimitive || !endpointPrimitive) return 0.0;
        const auto signalSpeed = std::max(
            reservoirPrimitive->speedOfSoundMps,
            std::abs(endpointPrimitive->velocityMps) + endpointPrimitive->speedOfSoundMps);
        if (opening > 0.0) {
            stableStep = std::min(stableStep,
                config_.maximumCourantNumber * cylinderReservoirVolumesM3_[index]
                    / (opening * signalSpeed));
        }
    }
    for (const auto& outlet : layout_.outlets()) {
        const auto opening = outlet.openingAreaM2 * outlet.dischargeCoefficient
            * std::clamp(ambient.openingScale, 0.0, 1.0);
        constrainBoundary(ambient.reservoirState, outlet.networkEndpoint, opening);
    }
    return stableStep;
}

bool ExhaustGasNetwork::evaluateStage(
    bool useStageState,
    std::span<const CylinderValveBoundary> cylinderBoundaries,
    const ExhaustAmbientBoundary& ambient) noexcept {
    std::fill(ductInletAssigned_.begin(), ductInletAssigned_.end(), std::uint8_t { 0 });
    std::fill(ductOutletAssigned_.begin(), ductOutletAssigned_.end(), std::uint8_t { 0 });
    auto& cylinderFlows = useStageState ? cylinderSecondStageFlow_ : cylinderFirstStageFlow_;
    auto& outletFlows = useStageState ? outletSecondStageFlow_ : outletFirstStageFlow_;
    std::fill(cylinderFlows.begin(), cylinderFlows.end(), ConservedFlowRate {});
    std::fill(outletFlows.begin(), outletFlows.end(), ConservedFlowRate {});
    auto& junctionResiduals = useStageState ? junctionStageResidual_ : junctionResidual_;
    std::fill(junctionResiduals.begin(), junctionResiduals.end(), ConservativeState {});
    auto& cylinderResiduals = useStageState
        ? cylinderReservoirStageResidual_ : cylinderReservoirResidual_;
    std::fill(cylinderResiduals.begin(), cylinderResiduals.end(), ConservativeState {});

    const auto transmissive = DuctBoundaryCondition::transmissive();
    for (auto& duct : ducts_) {
        const auto states = useStageState
            ? std::span<const ConservativeState>(duct.stage_)
            : std::span<const ConservativeState>(duct.cells_);
        const auto primitives = useStageState
            ? std::span<const PrimitiveState>(duct.stagePrimitives_)
            : std::span<const PrimitiveState>(duct.cellPrimitives_);
        const auto sourceTerms = useStageState
            ? std::span<const ConservativeState>(duct.stageSourceTerms_)
            : std::span<const ConservativeState>(duct.cellSourceTerms_);
        auto residual = useStageState
            ? std::span<ConservativeState>(duct.stageResidual_)
            : std::span<ConservativeState>(duct.residual_);
        auto faceFluxes = useStageState
            ? std::span<EulerFlux>(duct.stageFaceFluxes_)
            : std::span<EulerFlux>(duct.faceFluxes_);
        if (!duct.computeResidual(states, primitives, sourceTerms,
                                  transmissive, transmissive, residual, faceFluxes))
            return false;
    }

    const auto& junctionStates = useStageState ? junctionStage_ : junctionStates_;
    const auto endpointState = [this, useStageState, &junctionStates](
        const ExhaustEndpoint& endpoint) -> const ConservativeState& {
        if (endpoint.type == ExhaustEndpointType::junction)
            return junctionStates[endpoint.elementIndex];
        const auto& duct = ducts_[endpoint.elementIndex];
        const auto& states = useStageState ? duct.stage_ : duct.cells_;
        return endpoint.type == ExhaustEndpointType::ductInlet
            ? states.front() : states.back();
    };
    const auto endpointArea = [this](const ExhaustEndpoint& endpoint) noexcept {
        if (endpoint.type == ExhaustEndpointType::junction) {
            const auto diameter = layout_.junctions()[endpoint.elementIndex]
                .characteristicDiameterM;
            return 0.25 * std::acos(-1.0) * diameter * diameter;
        }
        return layout_.ducts()[endpoint.elementIndex].connectionAreaM2;
    };
    const auto makeFlowRate = [](const EulerFlux& flux, double areaM2) noexcept {
        ConservedFlowRate result;
        for (std::size_t index = 0; index < gasSpeciesCount; ++index)
            result.speciesMassKgPerS[index] = flux.speciesMassFluxKgPerM2S[index] * areaM2;
        result.momentumN = flux.momentumFluxPa * areaM2;
        result.totalEnergyW = flux.totalEnergyFluxWPerM2 * areaM2;
        return result;
    };
    const auto addJunctionFlow = [&junctionResiduals, this](
        std::size_t junctionIndex, const ConservedFlowRate& flow, double sign) noexcept {
        const auto inverseVolume = 1.0 / layout_.junctions()[junctionIndex].volumeM3;
        for (std::size_t species = 0; species < gasSpeciesCount; ++species) {
            junctionResiduals[junctionIndex].speciesMassDensityKgPerM3[species] +=
                sign * flow.speciesMassKgPerS[species] * inverseVolume;
        }
        // A zero-dimensional junction mixes incoming axial momenta. Their
        // kinetic energy remains in totalEnergyW and thermalises; vector wall
        // reactions carry momentum and are deliberately not stored as one
        // arbitrary scalar direction.
        junctionResiduals[junctionIndex].totalEnergyDensityJPerM3 +=
            sign * flow.totalEnergyW * inverseVolume;
    };
    const auto assignDuctBoundary = [this, useStageState](
        const ExhaustEndpoint& endpoint, const EulerFlux& newFlux) noexcept {
        if (endpoint.type == ExhaustEndpointType::junction) return true;
        const auto ductIndex = endpoint.elementIndex;
        auto& duct = ducts_[ductIndex];
        auto& assigned = endpoint.type == ExhaustEndpointType::ductInlet
            ? ductInletAssigned_[ductIndex] : ductOutletAssigned_[ductIndex];
        if (assigned != 0) return false;
        assigned = 1;
        auto& residual = useStageState ? duct.stageResidual_ : duct.residual_;
        auto& faceFluxes = useStageState ? duct.stageFaceFluxes_ : duct.faceFluxes_;
        const auto inverseDx = 1.0 / duct.geometry_.cellLengthM();
        const auto faceIndex = endpoint.type == ExhaustEndpointType::ductInlet
            ? std::size_t { 0 } : faceFluxes.size() - 1;
        const auto residualIndex = endpoint.type == ExhaustEndpointType::ductInlet
            ? std::size_t { 0 } : residual.size() - 1;
        const auto sign = endpoint.type == ExhaustEndpointType::ductInlet ? 1.0 : -1.0;
        const auto& oldFlux = faceFluxes[faceIndex];
        for (std::size_t species = 0; species < gasSpeciesCount; ++species) {
            residual[residualIndex].speciesMassDensityKgPerM3[species] += sign * inverseDx
                * (newFlux.speciesMassFluxKgPerM2S[species]
                   - oldFlux.speciesMassFluxKgPerM2S[species]);
        }
        residual[residualIndex].momentumDensityKgPerM2S += sign * inverseDx
            * (newFlux.momentumFluxPa - oldFlux.momentumFluxPa);
        residual[residualIndex].totalEnergyDensityJPerM3 += sign * inverseDx
            * (newFlux.totalEnergyFluxWPerM2 - oldFlux.totalEnergyFluxWPerM2);
        faceFluxes[faceIndex] = newFlux;
        return true;
    };
    const auto ductBoundaryFlux = [this, &endpointState](
        const ExhaustEndpoint& endpoint,
        const EulerFlux& apertureFlux,
        double apertureAreaM2) noexcept {
        const auto primitive = mixtureModel_.primitiveFromConservative(endpointState(endpoint));
        if (!primitive) return EulerFlux {};
        return areaAveragedBoundaryFlux(apertureFlux, apertureAreaM2,
            ducts_[endpoint.elementIndex].geometry().areaM2(), primitive->pressurePa);
    };
    const auto connect = [&](const ExhaustEndpoint& upstream,
                             const ExhaustEndpoint& downstream,
                             double openingAreaM2,
                             ConservedFlowRate* observedFlow) noexcept {
        auto effectiveArea = std::max(0.0, openingAreaM2);
        if (upstream.type == ExhaustEndpointType::junction) {
            effectiveArea /= std::sqrt(1.0
                + layout_.junctions()[upstream.elementIndex].lossCoefficient);
        }
        const auto rawFlux = effectiveArea > 0.0
            ? mixtureModel_.riemannFlux(endpointState(upstream), endpointState(downstream))
            : EulerFlux {};
        const auto flow = makeFlowRate(rawFlux, effectiveArea);
        if (observedFlow) *observedFlow = flow;
        if (upstream.type == ExhaustEndpointType::junction) {
            addJunctionFlow(upstream.elementIndex, flow, -1.0);
        } else if (!assignDuctBoundary(upstream,
                       ductBoundaryFlux(upstream, rawFlux, effectiveArea))) {
            return false;
        }
        if (downstream.type == ExhaustEndpointType::junction) {
            addJunctionFlow(downstream.elementIndex, flow, 1.0);
        } else if (!assignDuctBoundary(downstream,
                       ductBoundaryFlux(downstream, rawFlux, effectiveArea))) {
            return false;
        }
        return true;
    };

    for (const auto& connection : layout_.interfaces()) {
        if (!connect(connection.upstream, connection.downstream,
                     std::min(endpointArea(connection.upstream),
                              endpointArea(connection.downstream)),
                     nullptr))
            return false;
    }

    for (std::size_t index = 0; index < layout_.cylinderPorts().size(); ++index) {
        const auto& port = layout_.cylinderPorts()[index];
        const auto supplied = std::find_if(cylinderBoundaries.begin(), cylinderBoundaries.end(),
            [&port](const CylinderValveBoundary& boundary) {
                return boundary.cylinderId == port.cylinderId;
            });
        auto openingArea = 0.0;
        const ConservativeState* cylinderState = nullptr;
        if (supplied != cylinderBoundaries.end()) {
            openingArea = std::min(
                std::max(0.0, supplied->effectiveValveAreaM2)
                    * std::clamp(supplied->dischargeCoefficient, 0.0, 1.5),
                port.runnerConnectionAreaM2 * port.dischargeCoefficient);
            cylinderState = &supplied->cylinderState;
        }

        const auto& activeCylinderState = useStageState
            ? cylinderReservoirStage_[index] : cylinderReservoirStates_[index];
        const auto networkState = endpointState(port.networkEndpoint);
        const auto rawFlux = openingArea > 0.0 && cylinderState
            ? mixtureModel_.riemannFlux(activeCylinderState, networkState)
            : EulerFlux {};
        cylinderFlows[index] = makeFlowRate(rawFlux, openingArea);
        if (openingArea > 0.0 && cylinderReservoirActive_[index] != 0) {
            const auto inverseVolume = 1.0 / cylinderReservoirVolumesM3_[index];
            for (std::size_t species = 0; species < gasSpeciesCount; ++species) {
                cylinderResiduals[index].speciesMassDensityKgPerM3[species] -=
                    cylinderFlows[index].speciesMassKgPerS[species] * inverseVolume;
            }
            cylinderResiduals[index].totalEnergyDensityJPerM3 -=
                cylinderFlows[index].totalEnergyW * inverseVolume;
        }
        if (port.networkEndpoint.type == ExhaustEndpointType::junction) {
            addJunctionFlow(port.networkEndpoint.elementIndex, cylinderFlows[index], 1.0);
        } else if (!assignDuctBoundary(port.networkEndpoint,
                       ductBoundaryFlux(port.networkEndpoint, rawFlux, openingArea))) {
            return false;
        }
    }

    for (std::size_t index = 0; index < layout_.outlets().size(); ++index) {
        const auto& outlet = layout_.outlets()[index];
        auto openingArea = std::min(endpointArea(outlet.networkEndpoint),
            outlet.openingAreaM2 * outlet.dischargeCoefficient
                * std::clamp(ambient.openingScale, 0.0, 1.0));
        if (outlet.networkEndpoint.type == ExhaustEndpointType::junction) {
            openingArea /= std::sqrt(1.0
                + layout_.junctions()[outlet.networkEndpoint.elementIndex].lossCoefficient);
        }
        const auto rawFlux = openingArea > 0.0
            ? mixtureModel_.riemannFlux(endpointState(outlet.networkEndpoint),
                                        ambient.reservoirState)
            : EulerFlux {};
        outletFlows[index] = makeFlowRate(rawFlux, openingArea);
        if (outlet.networkEndpoint.type == ExhaustEndpointType::junction) {
            addJunctionFlow(outlet.networkEndpoint.elementIndex, outletFlows[index], -1.0);
        } else if (!assignDuctBoundary(outlet.networkEndpoint,
                       ductBoundaryFlux(outlet.networkEndpoint, rawFlux, openingArea))) {
            return false;
        }
    }

    return std::all_of(ductInletAssigned_.begin(), ductInletAssigned_.end(),
               [](std::uint8_t assigned) { return assigned != 0; })
        && std::all_of(ductOutletAssigned_.begin(), ductOutletAssigned_.end(),
               [](std::uint8_t assigned) { return assigned != 0; });
}

bool ExhaustGasNetwork::prepareStageStates(bool candidateStage) noexcept {
    for (auto& duct : ducts_) {
        const auto& states = candidateStage ? duct.candidate_ : duct.stage_;
        auto& primitives = candidateStage
            ? duct.candidatePrimitives_ : duct.stagePrimitives_;
        auto& sourceTerms = candidateStage
            ? duct.candidateSourceTerms_ : duct.stageSourceTerms_;
        auto& maximumSignalSpeed = candidateStage
            ? duct.maximumCandidateSignalSpeedMps_ : duct.maximumStageSignalSpeedMps_;
        auto& sourceLimitedTimeStep = candidateStage
            ? duct.candidateSourceLimitedTimeStepSeconds_
            : duct.stageSourceLimitedTimeStepSeconds_;
        if (!duct.prepareStateCache(states, primitives, sourceTerms,
                maximumSignalSpeed, sourceLimitedTimeStep))
            return false;
    }
    const auto& states = candidateStage ? junctionCandidate_ : junctionStage_;
    for (const auto& state : states)
        if (!mixtureModel_.isPhysical(state)) return false;
    const auto& reservoirs = candidateStage
        ? cylinderReservoirCandidate_ : cylinderReservoirStage_;
    for (std::size_t index = 0; index < reservoirs.size(); ++index) {
        if (cylinderReservoirActive_[index] != 0
            && !mixtureModel_.isPhysical(reservoirs[index]))
            return false;
    }
    return true;
}

ExhaustNetworkAdvanceResult ExhaustGasNetwork::advance(
    double durationSeconds,
    std::span<const CylinderValveBoundary> cylinderBoundaries,
    const ExhaustAmbientBoundary& ambient) noexcept {
    ExhaustNetworkAdvanceResult result;
    if (!configured_ || !finite(durationSeconds) || durationSeconds < 0.0
        || !finite(ambient.openingScale) || ambient.openingScale < 0.0
        || ambient.openingScale > 1.0
        || !mixtureModel_.isPhysical(ambient.reservoirState)) {
        result.completed = false;
        return result;
    }
    for (std::size_t index = 0; index < cylinderBoundaries.size(); ++index) {
        const auto& boundary = cylinderBoundaries[index];
        if (!finite(boundary.cylinderVolumeM3) || !(boundary.cylinderVolumeM3 > 0.0)
            || !finite(boundary.effectiveValveAreaM2) || boundary.effectiveValveAreaM2 < 0.0
            || !finite(boundary.dischargeCoefficient) || boundary.dischargeCoefficient < 0.0
            || !mixtureModel_.isPhysical(boundary.cylinderState)) {
            result.completed = false;
            return result;
        }
        for (std::size_t other = index + 1; other < cylinderBoundaries.size(); ++other) {
            if (boundary.cylinderId == cylinderBoundaries[other].cylinderId) {
                result.completed = false;
                return result;
            }
        }
    }

    for (std::size_t index = 0; index < cylinderExchanges_.size(); ++index) {
        const auto cylinderId = cylinderExchanges_[index].cylinderId;
        const auto pathIndex = cylinderExchanges_[index].pathIndex;
        cylinderExchanges_[index] = {};
        cylinderExchanges_[index].cylinderId = cylinderId;
        cylinderExchanges_[index].pathIndex = pathIndex;
        const auto boundary = std::find_if(cylinderBoundaries.begin(), cylinderBoundaries.end(),
            [cylinderId](const CylinderValveBoundary& candidate) {
                return candidate.cylinderId == cylinderId;
            });
        if (boundary != cylinderBoundaries.end()) {
            cylinderReservoirStates_[index] = boundary->cylinderState;
            cylinderReservoirVolumesM3_[index] = boundary->cylinderVolumeM3;
            cylinderReservoirActive_[index] = 1;
        } else {
            cylinderReservoirStates_[index] = {};
            cylinderReservoirVolumesM3_[index] = 0.0;
            cylinderReservoirActive_[index] = 0;
        }
    }
    for (std::size_t index = 0; index < outletSamples_.size(); ++index) {
        const auto nodeId = outletSamples_[index].outletNodeId;
        const auto pathIndex = outletSamples_[index].pathIndex;
        const auto openingArea = outletSamples_[index].openingAreaM2;
        outletSamples_[index] = {};
        outletSamples_[index].outletNodeId = nodeId;
        outletSamples_[index].pathIndex = pathIndex;
        outletSamples_[index].openingAreaM2 = openingArea;
    }
    if (durationSeconds == 0.0) {
        updateOutletSamples(0.0);
        return result;
    }

    auto remaining = durationSeconds;
    auto attempts = std::size_t { 0 };
    const auto completionTolerance = std::max(1.0e-15, durationSeconds * 1.0e-13);
    while (remaining > completionTolerance
           && attempts < config_.maximumSubstepsPerAdvance) {
        const auto stableStep = maximumStableTimeStep(cylinderBoundaries, ambient);
        if (!(stableStep > 0.0) || !finite(stableStep)) {
            result.completed = false;
            break;
        }
        auto trialStep = std::min(remaining, stableStep);
        auto accepted = false;
        while (!accepted && attempts < config_.maximumSubstepsPerAdvance) {
            ++attempts;
            if (!evaluateStage(false, cylinderBoundaries, ambient)) {
                result.completed = false;
                break;
            }
            for (auto& duct : ducts_) {
                for (std::size_t index = 0; index < duct.cells_.size(); ++index)
                    duct.stage_[index] = addScaled(
                        duct.cells_[index], duct.residual_[index], trialStep);
            }
            for (std::size_t index = 0; index < junctionStates_.size(); ++index) {
                junctionStage_[index] = addScaled(
                    junctionStates_[index], junctionResidual_[index], trialStep);
                junctionStage_[index].momentumDensityKgPerM2S = 0.0;
            }
            for (std::size_t index = 0; index < cylinderReservoirStates_.size(); ++index) {
                if (cylinderReservoirActive_[index] == 0) continue;
                cylinderReservoirStage_[index] = addScaled(
                    cylinderReservoirStates_[index],
                    cylinderReservoirResidual_[index], trialStep);
                cylinderReservoirStage_[index].momentumDensityKgPerM2S = 0.0;
            }
            for (auto& duct : ducts_)
                for (auto& state : duct.stage_)
                    (void) mixtureModel_.canonicaliseSpeciesRoundoff(state);
            for (auto& state : junctionStage_)
                (void) mixtureModel_.canonicaliseSpeciesRoundoff(state);
            for (std::size_t index = 0; index < cylinderReservoirStage_.size(); ++index)
                if (cylinderReservoirActive_[index] != 0)
                    (void) mixtureModel_.canonicaliseSpeciesRoundoff(
                        cylinderReservoirStage_[index]);
            if (!prepareStageStates(false)) {
                ++result.rejectedSubsteps;
                trialStep *= 0.5;
                if (!(trialStep > std::numeric_limits<double>::epsilon()
                                  * std::max(1.0, durationSeconds)))
                    break;
                continue;
            }

            if (!evaluateStage(true, cylinderBoundaries, ambient)) {
                result.completed = false;
                break;
            }
            for (auto& duct : ducts_) {
                for (std::size_t index = 0; index < duct.cells_.size(); ++index) {
                    duct.candidate_[index] = rk2Combination(
                        duct.cells_[index], duct.stage_[index],
                        duct.stageResidual_[index], trialStep);
                }
            }
            for (std::size_t index = 0; index < junctionStates_.size(); ++index) {
                junctionCandidate_[index] = rk2Combination(
                    junctionStates_[index], junctionStage_[index],
                    junctionStageResidual_[index], trialStep);
                junctionCandidate_[index].momentumDensityKgPerM2S = 0.0;
            }
            for (std::size_t index = 0; index < cylinderReservoirStates_.size(); ++index) {
                if (cylinderReservoirActive_[index] == 0) continue;
                cylinderReservoirCandidate_[index] = rk2Combination(
                    cylinderReservoirStates_[index], cylinderReservoirStage_[index],
                    cylinderReservoirStageResidual_[index], trialStep);
                cylinderReservoirCandidate_[index].momentumDensityKgPerM2S = 0.0;
            }
            for (auto& duct : ducts_)
                for (auto& state : duct.candidate_)
                    (void) mixtureModel_.canonicaliseSpeciesRoundoff(state);
            for (auto& state : junctionCandidate_)
                (void) mixtureModel_.canonicaliseSpeciesRoundoff(state);
            for (std::size_t index = 0; index < cylinderReservoirCandidate_.size(); ++index)
                if (cylinderReservoirActive_[index] != 0)
                    (void) mixtureModel_.canonicaliseSpeciesRoundoff(
                        cylinderReservoirCandidate_[index]);
            if (!prepareStageStates(true)) {
                ++result.rejectedSubsteps;
                trialStep *= 0.5;
                if (!(trialStep > std::numeric_limits<double>::epsilon()
                                  * std::max(1.0, durationSeconds)))
                    break;
                continue;
            }

            const auto integrationWeight = 0.5 * trialStep;
            for (std::size_t index = 0; index < cylinderExchanges_.size(); ++index) {
                for (std::size_t species = 0; species < gasSpeciesCount; ++species) {
                    cylinderExchanges_[index].speciesMassKg[species] += integrationWeight
                        * (cylinderFirstStageFlow_[index].speciesMassKgPerS[species]
                           + cylinderSecondStageFlow_[index].speciesMassKgPerS[species]);
                }
                cylinderExchanges_[index].totalEnergyJ += integrationWeight
                    * (cylinderFirstStageFlow_[index].totalEnergyW
                       + cylinderSecondStageFlow_[index].totalEnergyW);
                cylinderExchanges_[index].axialMomentumImpulseNs += integrationWeight
                    * (cylinderFirstStageFlow_[index].momentumN
                       + cylinderSecondStageFlow_[index].momentumN);
            }
            for (std::size_t index = 0; index < outletSamples_.size(); ++index) {
                for (std::size_t species = 0; species < gasSpeciesCount; ++species) {
                    outletSamples_[index].speciesMassKg[species] += integrationWeight
                        * (outletFirstStageFlow_[index].speciesMassKgPerS[species]
                           + outletSecondStageFlow_[index].speciesMassKgPerS[species]);
                }
                outletSamples_[index].transferredEnergyJ += integrationWeight
                    * (outletFirstStageFlow_[index].totalEnergyW
                       + outletSecondStageFlow_[index].totalEnergyW);
            }

            for (auto& duct : ducts_) {
                duct.cells_.swap(duct.candidate_);
                duct.cellPrimitives_.swap(duct.candidatePrimitives_);
                duct.cellSourceTerms_.swap(duct.candidateSourceTerms_);
                std::swap(duct.maximumCellSignalSpeedMps_,
                          duct.maximumCandidateSignalSpeedMps_);
                std::swap(duct.cellSourceLimitedTimeStepSeconds_,
                          duct.candidateSourceLimitedTimeStepSeconds_);
                duct.cellStateCacheIsValid_ = true;
            }
            junctionStates_.swap(junctionCandidate_);
            cylinderReservoirStates_.swap(cylinderReservoirCandidate_);
            remaining -= trialStep;
            result.advancedTimeSeconds += trialStep;
            ++result.acceptedSubsteps;
            accepted = true;
        }
        if (!accepted) {
            result.completed = false;
            break;
        }
    }
    if (remaining > completionTolerance) result.completed = false;
    updateOutletSamples(result.advancedTimeSeconds);
    return result;
}

void ExhaustGasNetwork::updateOutletSamples(double durationSeconds) noexcept {
    for (std::size_t index = 0; index < cylinderExchanges_.size(); ++index) {
        if (cylinderReservoirActive_[index] != 0) {
            const auto cylinderPrimitive = mixtureModel_.primitiveFromConservative(
                cylinderReservoirStates_[index]);
            if (cylinderPrimitive) {
                cylinderExchanges_[index].cylinderPressurePaAfter =
                    cylinderPrimitive->pressurePa;
                cylinderExchanges_[index].cylinderTemperatureKAfter =
                    cylinderPrimitive->temperatureK;
            }
        }
        const auto& endpoint = layout_.cylinderPorts()[index].networkEndpoint;
        const ConservativeState* state = nullptr;
        if (endpoint.type == ExhaustEndpointType::junction) {
            state = &junctionStates_[endpoint.elementIndex];
        } else {
            const auto& cells = ducts_[endpoint.elementIndex].cells();
            state = endpoint.type == ExhaustEndpointType::ductInlet
                ? &cells.front() : &cells.back();
        }
        const auto primitive = mixtureModel_.primitiveFromConservative(*state);
        if (primitive) {
            cylinderExchanges_[index].networkPressurePa = primitive->pressurePa;
            cylinderExchanges_[index].networkTemperatureK = primitive->temperatureK;
            cylinderExchanges_[index].networkVelocityMps = primitive->velocityMps;
        }
    }
    for (std::size_t index = 0; index < outletSamples_.size(); ++index) {
        auto& sample = outletSamples_[index];
        const auto& endpoint = layout_.outlets()[index].networkEndpoint;
        const ConservativeState* state = nullptr;
        if (endpoint.type == ExhaustEndpointType::junction) {
            state = &junctionStates_[endpoint.elementIndex];
        } else {
            const auto& cells = ducts_[endpoint.elementIndex].cells();
            state = endpoint.type == ExhaustEndpointType::ductInlet
                ? &cells.front() : &cells.back();
        }
        const auto primitive = mixtureModel_.primitiveFromConservative(*state);
        if (primitive) {
            sample.staticPressurePa = primitive->pressurePa;
            sample.temperatureK = primitive->temperatureK;
            sample.axialVelocityMps = primitive->velocityMps;
            sample.densityKgPerM3 = primitive->densityKgPerM3;
        }
        if (durationSeconds > 0.0) {
            sample.massFlowKgPerS = sumSpecies(sample.speciesMassKg) / durationSeconds;
            sample.totalEnergyFlowW = sample.transferredEnergyJ / durationSeconds;
            sample.volumeFlowM3PerS = sample.densityKgPerM3 > 0.0
                ? sample.massFlowKgPerS / sample.densityKgPerM3 : 0.0;
        }
    }
}

} // namespace enginelab::gasdynamics
