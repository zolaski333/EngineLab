#include <enginelab/gasdynamics/ExhaustGasNetwork.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

namespace enginelab::gasdynamics {
namespace {

[[nodiscard]] bool finite(double value) noexcept {
    return std::isfinite(value);
}

/**
 * Boundary state at a terminal opening discharging into open atmosphere.
 *
 * The atmosphere is a reservoir, not a neighbouring cell. Solving a Riemann
 * problem between the last duct state and a cell of ambient air makes the
 * exhaust push a semi-infinite column of cold dense gas out of the way: the
 * flux is then limited by the *ambient* acoustic impedance rho_a*c_a, which for
 * 22 degC air is larger than the hot exhaust's own. Measured on the LS3 at
 * 5990 rpm, the tailpipe sat 79 kPa above ambient while discharging at only
 * 68 m/s where a free expansion gives 699, and lightening the ambient reservoir
 * by 17x dropped that to 16 kPa -- the back pressure was the boundary, not the
 * pipe. It is wrong acoustically for the same reason: an open end must reflect
 * a compression as a rarefaction, and against a denser reservoir it reflected
 * with the sign of a closed one.
 *
 * The standard characteristic treatment instead imposes the reservoir's static
 * pressure at the exit plane and takes everything else from the outgoing
 * invariant u + 2c/(gamma-1), which is the only information the interior sends
 * to the boundary while the flow there is subsonic. Once the exit is sonic no
 * information travels upstream at all and the interior state stands unchanged.
 * Entropy and composition come from whichever side the gas actually arrives
 * from, so backflow draws air rather than re-inhaling its own exhaust.
 *
 * When the invariant says the end is DRAWING from the reservoir, the ghost
 * must not be the reservoir at rest: a Riemann flux against a RESTING
 * reservoir cell meters the inflow by the acoustic impedance rho*c, a
 * linearised small-signal law. Measured on a 250 mm intake runner at steady
 * draw, the duct had to sit 10.5 kPa below a 101.3 kPa reservoir to pull
 * 26 m/s where the isentropic (Bernoulli) entry needs 0.4 kPa — a 13 % steady
 * mass-flow deficit through the downstream valve, which is what capped every
 * engine's high-rpm breathing once the intake runners became resolved ducts.
 * The inflow ghost is therefore the reservoir gas isentropically accelerated
 * from rest to the INTERIOR's static pressure (sonic-capped). Pressure is the
 * one interior quantity that is continuous across the inflow contact, so
 * reading it does not repeat the invariant-across-the-contact catastrophe
 * recorded below; entropy, gamma and composition all stay the reservoir's
 * own, and the species/entropy jump is left to the Riemann solver, which is
 * built for exactly that. A quasi-steady full-face nozzle flux was tried here
 * instead and rejected: its free-jet momentum stress (P_exit + rho*u_exit^2,
 * u_exit sized by the whole pressure drop) rammed the interior 10 kPa ABOVE
 * the reservoir because the duct entry is not a compact aperture.
 */
[[nodiscard]] std::optional<PrimitiveState> openEndBoundaryPrimitive(
    const PrimitiveState& interior, const PrimitiveState& ambient) noexcept {
    const auto gamma = interior.heatCapacityRatio;
    if (!(gamma > 1.0) || !(interior.pressurePa > 0.0)
        || !(interior.densityKgPerM3 > 0.0) || !(interior.speedOfSoundMps > 0.0)
        || !(ambient.pressurePa > 0.0) || !(ambient.densityKgPerM3 > 0.0)
        || !(ambient.speedOfSoundMps > 0.0))
        return std::nullopt;
    // Sonic or supersonic outflow: the exit plane is determined entirely from
    // inside, so the interior state is already the boundary state.
    if (interior.velocityMps >= interior.speedOfSoundMps) return interior;

    PrimitiveState boundary = interior;
    boundary.pressurePa = ambient.pressurePa;
    // Isentropic along the interior's own entropy, which is what an outflowing
    // particle carries with it to the exit plane.
    boundary.densityKgPerM3 = interior.densityKgPerM3
        * std::pow(ambient.pressurePa / interior.pressurePa, 1.0 / gamma);
    if (!(boundary.densityKgPerM3 > 0.0)) return std::nullopt;
    boundary.speedOfSoundMps = std::sqrt(gamma * boundary.pressurePa
        / boundary.densityKgPerM3);
    boundary.velocityMps = interior.velocityMps
        + 2.0 * (interior.speedOfSoundMps - boundary.speedOfSoundMps) / (gamma - 1.0);
    // Backflow: the duct is drawing from the atmosphere, which is a reservoir at
    // rest. The invariant above cannot be continued across that contact -- it
    // holds only within one gamma and one entropy, and evaluating it on the hot
    // gas's sound speed while assigning it the atmosphere's produced a 2 km/s
    // outflow in the branch meant to model an inflow. Every catalogue engine but
    // one stalled on that. Build the inflow ghost from RESERVOIR quantities
    // alone, sized by the interior's static pressure (see the header comment).
    if (boundary.velocityMps < 0.0) {
        const auto reservoirGamma = std::clamp(ambient.heatCapacityRatio, 1.01, 2.0);
        // Isentropic acceleration from rest cannot expand past the sonic
        // (critical) pressure ratio; below it the entry is choked.
        const auto criticalPressureRatio = std::pow(2.0 / (reservoirGamma + 1.0),
            reservoirGamma / (reservoirGamma - 1.0));
        const auto pressureRatio = std::clamp(
            interior.pressurePa / ambient.pressurePa, criticalPressureRatio, 1.0);
        PrimitiveState inflow = ambient;
        inflow.pressurePa = ambient.pressurePa * pressureRatio;
        inflow.densityKgPerM3 = ambient.densityKgPerM3
            * std::pow(pressureRatio, 1.0 / reservoirGamma);
        if (!(inflow.densityKgPerM3 > 0.0)) return std::nullopt;
        inflow.temperatureK = ambient.temperatureK
            * pressureRatio / std::pow(pressureRatio, 1.0 / reservoirGamma);
        inflow.speedOfSoundMps = std::sqrt(reservoirGamma * inflow.pressurePa
            / inflow.densityKgPerM3);
        // Energy: c0^2 = c^2 + (gamma-1)/2 u^2 along the reservoir isentrope.
        const auto acceleratedSquared = 2.0
            * (ambient.speedOfSoundMps * ambient.speedOfSoundMps
               - inflow.speedOfSoundMps * inflow.speedOfSoundMps)
            / (reservoirGamma - 1.0);
        inflow.velocityMps = -std::sqrt(std::max(0.0, acceleratedSquared));
        if (!finite(inflow.velocityMps) || !finite(inflow.speedOfSoundMps))
            return std::nullopt;
        return inflow;
    }
    if (!finite(boundary.velocityMps) || !finite(boundary.speedOfSoundMps))
        return std::nullopt;
    return boundary;
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

/**
 * Quasi-steady flux through a compact valve aperture.
 *
 * A cylinder valve is a converging nozzle, not a zero-length continuation of
 * the runner. A plain shock-tube Riemann flux is appropriate between adjacent
 * duct cells but substantially under-predicts reservoir blowdown once the
 * pressure ratio is choked. This boundary uses the isentropic nozzle solution;
 * the separately supplied effective area already contains the measured valve
 * discharge coefficient. Species and stagnation enthalpy are transported from
 * the upstream state, so the finite cylinder reservoir and network remain one
 * conservative mass/energy system. Valve-wall reaction carries the unresolved
 * momentum, as it must for a zero-dimensional cylinder reservoir.
 */
[[nodiscard]] EulerFlux compressibleValveFlux(
    const ConservativeState& leftState,
    const PrimitiveState& left,
    const ConservativeState& rightState,
    const PrimitiveState& right) noexcept {
    EulerFlux result;
    const auto meanPressure = 0.5 * (left.pressurePa + right.pressurePa);
    result.momentumFluxPa = meanPressure;

    const auto pressureScale = std::max({ left.pressurePa, right.pressurePa, 1.0 });
    if (std::abs(left.pressurePa - right.pressurePa)
            <= pressureScale * 32.0 * std::numeric_limits<double>::epsilon())
        return result;

    const auto forward = left.pressurePa > right.pressurePa;
    const auto& upstreamState = forward ? leftState : rightState;
    const auto& upstream = forward ? left : right;
    const auto& downstream = forward ? right : left;
    const auto direction = forward ? 1.0 : -1.0;
    const auto gamma = std::clamp(upstream.heatCapacityRatio, 1.01, 2.0);
    const auto gasConstant = upstream.pressurePa
        / (upstream.densityKgPerM3 * upstream.temperatureK);
    if (!(gasConstant > 0.0) || !finite(gasConstant)) return result;

    // The selected upstream cell supplies the quasi-steady reservoir pressure
    // and temperature for this compact aperture. Cylinder momentum is zero by
    // construction; for reverse flow the transported total enthalpy below also
    // retains the runner's resolved kinetic energy.
    const auto upstreamPressurePa = upstream.pressurePa;
    const auto upstreamTemperatureK = upstream.temperatureK;
    const auto pressureRatio = std::clamp(
        downstream.pressurePa / upstreamPressurePa, 0.0, 1.0);
    const auto criticalPressureRatio = std::pow(
        2.0 / (gamma + 1.0), gamma / (gamma - 1.0));

    double massFluxMagnitude = 0.0;
    double exitPressurePa = downstream.pressurePa;
    double exitTemperatureK = upstreamTemperatureK;
    if (pressureRatio <= criticalPressureRatio) {
        const auto criticalTemperatureRatio = 2.0 / (gamma + 1.0);
        exitPressurePa = upstreamPressurePa * criticalPressureRatio;
        exitTemperatureK = upstreamTemperatureK * criticalTemperatureRatio;
        // Algebraically reuse the critical pressure ratio:
        // t^((gamma+1)/(2(gamma-1))) = t^(gamma/(gamma-1))/sqrt(t).
        // This is the exact nozzle relation with one transcendental evaluation,
        // which matters because every open valve is sampled at solver cadence.
        massFluxMagnitude = upstreamPressurePa
            / std::sqrt(gasConstant * upstreamTemperatureK)
            * std::sqrt(gamma)
            * criticalPressureRatio / std::sqrt(criticalTemperatureRatio);
    } else {
        // Let q=r^(1/gamma). Then r^(2/gamma)=q^2,
        // r^((gamma+1)/gamma)=r*q and r^((gamma-1)/gamma)=r/q.
        // One pow therefore supplies all three exact isentropic terms.
        const auto pressureRoot = std::pow(pressureRatio, 1.0 / gamma);
        const auto firstPower = pressureRoot * pressureRoot;
        const auto secondPower = pressureRatio * pressureRoot;
        massFluxMagnitude = upstreamPressurePa
            / std::sqrt(gasConstant * upstreamTemperatureK)
            * std::sqrt(std::max(0.0,
                2.0 * gamma / (gamma - 1.0) * (firstPower - secondPower)));
        exitTemperatureK = upstreamTemperatureK
            * pressureRatio / pressureRoot;
    }
    if (!(massFluxMagnitude >= 0.0) || !finite(massFluxMagnitude)
        || !(exitTemperatureK > 0.0) || !finite(exitTemperatureK))
        return result;

    const auto signedMassFlux = direction * massFluxMagnitude;
    const auto upstreamDensity = upstream.densityKgPerM3;
    for (std::size_t species = 0; species < gasSpeciesCount; ++species) {
        result.speciesMassFluxKgPerM2S[species] = signedMassFlux
            * upstreamState.speciesMassDensityKgPerM3[species]
            / upstreamDensity;
    }
    const auto totalSpecificEnthalpy =
        (upstreamState.totalEnergyDensityJPerM3 + upstream.pressurePa)
        / upstreamDensity;
    result.totalEnergyFluxWPerM2 = signedMassFlux * totalSpecificEnthalpy;

    const auto exitDensity = exitPressurePa / (gasConstant * exitTemperatureK);
    const auto exitVelocityMagnitude = exitDensity > 0.0
        ? massFluxMagnitude / exitDensity : 0.0;
    // rho*u^2 is positive in either direction. Pressure is evaluated at the
    // nozzle exit (critical section if choked, downstream static pressure if not).
    result.momentumFluxPa = exitPressurePa
        + massFluxMagnitude * exitVelocityMagnitude;
    return result;
}

} // namespace

bool ExhaustGasNetworkConfig::valid() const noexcept {
    return finite(initialPressurePa) && initialPressurePa > 0.0
        && finite(initialTemperatureK) && initialTemperatureK > 0.0
        && finite(absoluteRoughnessM) && absoluteRoughnessM >= 0.0
        && finite(wallHeatTransferWPerM2K) && wallHeatTransferWPerM2K >= 0.0
        && finite(wallTemperatureK) && wallTemperatureK > 0.0
        && (!dynamicWallHeatTransferEnabled || wallHeatTransferWPerM2K == 0.0)
        && finite(wallThicknessM) && wallThicknessM > 0.0
        && finite(wallDensityKgPerM3) && wallDensityKgPerM3 > 0.0
        && finite(wallSpecificHeatJPerKgK) && wallSpecificHeatJPerKgK > 0.0
        && finite(externalWallHeatTransferWPerM2K)
        && externalWallHeatTransferWPerM2K >= 0.0
        && finite(externalTemperatureK) && externalTemperatureK > 0.0
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
        geometry.inletCrossSectionAreaM2 = descriptor.inletFlowAreaM2;
        geometry.outletCrossSectionAreaM2 = descriptor.outletFlowAreaM2;
        geometry.diameterM = descriptor.hydraulicDiameterM;
        geometry.cellCount = descriptor.cellCount;
        geometry.wallFrictionEnabled = true;
        geometry.absoluteRoughnessM = config.absoluteRoughnessM;
        geometry.localLossCoefficient = descriptor.lossCoefficient;
        geometry.wallHeatTransferWPerM2K = config.wallHeatTransferWPerM2K;
        geometry.wallTemperatureK = config.wallTemperatureK;
        geometry.dynamicWallHeatTransferEnabled =
            config.dynamicWallHeatTransferEnabled;
        geometry.wallThicknessM = config.wallThicknessM;
        geometry.wallDensityKgPerM3 = config.wallDensityKgPerM3;
        geometry.wallSpecificHeatJPerKgK = config.wallSpecificHeatJPerKgK;
        geometry.externalWallHeatTransferWPerM2K =
            config.externalWallHeatTransferWPerM2K;
        geometry.externalTemperatureK = config.externalTemperatureK;
        geometry.wallHeatUpdateIntervalSeconds =
            config.wallHeatUpdateIntervalSeconds;
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
    const auto initialPrimitive = mixtureModel_.primitiveFromConservative(*initialState);
    if (!initialPrimitive) return false;
    junctionPrimitives_.assign(junctionStates_.size(), *initialPrimitive);
    junctionStagePrimitives_.resize(junctionStates_.size());
    junctionCandidatePrimitives_.resize(junctionStates_.size());
    junctionPortAreaSums_.assign(junctionStates_.size(), 0.0);
    cylinderReservoirStates_.resize(layout_.cylinderPorts().size());
    cylinderReservoirStage_.resize(layout_.cylinderPorts().size());
    cylinderReservoirCandidate_.resize(layout_.cylinderPorts().size());
    cylinderReservoirResidual_.resize(layout_.cylinderPorts().size());
    cylinderReservoirStageResidual_.resize(layout_.cylinderPorts().size());
    cylinderReservoirPrimitives_.resize(layout_.cylinderPorts().size());
    cylinderReservoirStagePrimitives_.resize(layout_.cylinderPorts().size());
    cylinderReservoirCandidatePrimitives_.resize(layout_.cylinderPorts().size());
    cylinderReservoirVolumesM3_.resize(layout_.cylinderPorts().size());
    cylinderReservoirActive_.resize(layout_.cylinderPorts().size());
    cylinderBoundaryIndices_.resize(layout_.cylinderPorts().size());

    const auto endpointArea = [this](const ExhaustEndpoint& endpoint) noexcept {
        if (endpoint.type == ExhaustEndpointType::junction) {
            const auto diameter = layout_.junctions()[endpoint.elementIndex]
                .characteristicDiameterM;
            return 0.25 * std::acos(-1.0) * diameter * diameter;
        }
        const auto& duct = layout_.ducts()[endpoint.elementIndex];
        return endpoint.type == ExhaustEndpointType::ductInlet
            ? duct.inletConnectionAreaM2
            : duct.outletConnectionAreaM2;
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

bool ExhaustGasNetwork::reset(double pressurePa,
                              double temperatureK,
                              GasComposition composition) noexcept {
    if (!configured_) return false;
    const auto initialState = mixtureModel_.conservativeFromPressureTemperature(
        pressurePa, temperatureK, 0.0, composition);
    if (!initialState) return false;
    const auto initialPrimitive = mixtureModel_.primitiveFromConservative(*initialState);
    if (!initialPrimitive) return false;

    config_.initialPressurePa = pressurePa;
    config_.initialTemperatureK = temperatureK;
    config_.initialComposition = composition;
    for (auto& duct : ducts_) {
        auto cells = duct.cells();
        std::fill(cells.begin(), cells.end(), *initialState);
        duct.resetWallTemperature(temperatureK);
        if (!duct.refreshCellStateCache()) return false;
    }
    wallHeatPendingSeconds_ = 0.0;
    std::fill(junctionStates_.begin(), junctionStates_.end(), *initialState);
    for (auto& state : junctionStates_) state.momentumDensityKgPerM2S = 0.0;
    std::fill(junctionPrimitives_.begin(), junctionPrimitives_.end(), *initialPrimitive);
    std::fill(cylinderReservoirStates_.begin(), cylinderReservoirStates_.end(),
              ConservativeState {});
    std::fill(cylinderReservoirVolumesM3_.begin(), cylinderReservoirVolumesM3_.end(), 0.0);
    std::fill(cylinderReservoirActive_.begin(), cylinderReservoirActive_.end(),
              std::uint8_t { 0 });
    for (std::size_t index = 0; index < cylinderExchanges_.size(); ++index) {
        cylinderExchanges_[index] = {};
        cylinderExchanges_[index].cylinderId = layout_.cylinderPorts()[index].cylinderId;
        cylinderExchanges_[index].pathIndex = layout_.cylinderPorts()[index].pathIndex;
    }
    for (std::size_t index = 0; index < outletSamples_.size(); ++index) {
        outletSamples_[index] = {};
        outletSamples_[index].outletNodeId = layout_.outlets()[index].outletNodeId;
        outletSamples_[index].pathIndex = layout_.outlets()[index].pathIndex;
        outletSamples_[index].openingAreaM2 = layout_.outlets()[index].openingAreaM2;
    }
    updateOutletSamples(0.0);
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
        result.wallThermalEnergyJ += duct.wallThermalEnergyJ();
    }
    for (std::size_t index = 0; index < junctionStates_.size(); ++index) {
        const auto volume = layout_.junctions()[index].volumeM3;
        for (std::size_t species = 0; species < gasSpeciesCount; ++species) {
            result.speciesMassKg[species] +=
                junctionStates_[index].speciesMassDensityKgPerM3[species] * volume;
        }
        result.totalEnergyJ += junctionStates_[index].totalEnergyDensityJPerM3 * volume;
        result.resolvedAxialMomentumKgMps +=
            junctionStates_[index].momentumDensityKgPerM2S * volume;
    }
    return result;
}

bool ExhaustGasNetwork::sampleCylinderBoundaries(
    std::span<const CylinderValveBoundary> cylinderBoundaries,
    std::span<CylinderBoundaryFlowSample> samples) const noexcept {
    const auto ports = layout_.cylinderPorts();
    if (!configured_ || samples.size() < ports.size()) return false;

    for (std::size_t index = 0; index < ports.size(); ++index) {
        samples[index] = {};
        samples[index].cylinderId = ports[index].cylinderId;
        samples[index].pathIndex = ports[index].pathIndex;
    }
    // Mutable access to a duct's initial condition deliberately invalidates its
    // derived cache. Rebuild it here as advance() would before observing it.
    for (const auto& duct : ducts_)
        if (!duct.refreshCellStateCache()) return false;

    const auto suppliedInPortOrder = cylinderBoundaries.size() == ports.size()
        && std::equal(cylinderBoundaries.begin(), cylinderBoundaries.end(),
            ports.begin(), [](const CylinderValveBoundary& boundary,
                              const CompiledCylinderPort& port) {
                return boundary.cylinderId == port.cylinderId;
            });

    for (std::size_t index = 0; index < cylinderBoundaries.size(); ++index) {
        const auto& boundary = cylinderBoundaries[index];
        if (!finite(boundary.cylinderVolumeM3) || !(boundary.cylinderVolumeM3 > 0.0)
            || !finite(boundary.effectiveValveAreaM2) || boundary.effectiveValveAreaM2 < 0.0
            || !finite(boundary.dischargeCoefficient) || boundary.dischargeCoefficient < 0.0)
            return false;
        if (!suppliedInPortOrder) {
            const auto known = std::find_if(ports.begin(), ports.end(),
                [&boundary](const CompiledCylinderPort& port) {
                    return port.cylinderId == boundary.cylinderId;
                });
            if (known == ports.end()) return false;
            for (std::size_t other = index + 1; other < cylinderBoundaries.size(); ++other)
                if (boundary.cylinderId == cylinderBoundaries[other].cylinderId) return false;
        }
    }

    const auto endpointState = [this](const ExhaustEndpoint& endpoint)
        -> const ConservativeState& {
        if (endpoint.type == ExhaustEndpointType::junction)
            return junctionStates_[endpoint.elementIndex];
        const auto& cells = ducts_[endpoint.elementIndex].cells_;
        return endpoint.type == ExhaustEndpointType::ductInlet
            ? cells.front() : cells.back();
    };
    const auto endpointPrimitive = [this](const ExhaustEndpoint& endpoint)
        -> const PrimitiveState& {
        if (endpoint.type == ExhaustEndpointType::junction)
            return junctionPrimitives_[endpoint.elementIndex];
        const auto& primitives = ducts_[endpoint.elementIndex].cellPrimitives_;
        return endpoint.type == ExhaustEndpointType::ductInlet
            ? primitives.front() : primitives.back();
    };

    for (std::size_t portIndex = 0; portIndex < ports.size(); ++portIndex) {
        const auto& port = ports[portIndex];
        auto& sample = samples[portIndex];
        const auto& networkPrimitive = endpointPrimitive(port.networkEndpoint);
        sample.networkPressurePa = networkPrimitive.pressurePa;
        sample.networkTemperatureK = networkPrimitive.temperatureK;
        sample.networkDensityKgPerM3 = networkPrimitive.densityKgPerM3;
        sample.networkVelocityMps = networkPrimitive.velocityMps;
        sample.networkSpeedOfSoundMps = networkPrimitive.speedOfSoundMps;

        const auto supplied = suppliedInPortOrder
            ? cylinderBoundaries.begin() + static_cast<std::ptrdiff_t>(portIndex)
            : std::find_if(cylinderBoundaries.begin(), cylinderBoundaries.end(),
                [&port](const CylinderValveBoundary& boundary) {
                    return boundary.cylinderId == port.cylinderId;
                });
        if (supplied == cylinderBoundaries.end()) continue;
        const auto cylinderPrimitive = mixtureModel_.primitiveFromConservative(
            supplied->cylinderState);
        if (!cylinderPrimitive) return false;
        const auto openingAreaM2 = std::min(
            supplied->effectiveValveAreaM2
                * std::clamp(supplied->dischargeCoefficient, 0.0, 1.5),
            port.runnerConnectionAreaM2 * port.dischargeCoefficient);
        if (openingAreaM2 > 0.0) {
            const auto flux = compressibleValveFlux(
                supplied->cylinderState, *cylinderPrimitive,
                endpointState(port.networkEndpoint), networkPrimitive);
            sample.massFlowKgPerSecond = openingAreaM2
                * sumSpecies(flux.speciesMassFluxKgPerM2S);
        }
        sample.valid = finite(sample.massFlowKgPerSecond)
            && sample.networkPressurePa > 0.0
            && sample.networkTemperatureK > 0.0
            && sample.networkDensityKgPerM3 > 0.0
            && sample.networkSpeedOfSoundMps > 0.0;
    }
    return true;
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
        const auto& primitive = junctionPrimitives_[index];
        const auto areaSum = junctionPortAreaSums_[index];
        if (areaSum > 0.0) {
            stableStep = std::min(stableStep,
                config_.maximumCourantNumber * layout_.junctions()[index].volumeM3
                    / (areaSum * (std::abs(primitive.velocityMps)
                                  + primitive.speedOfSoundMps)));
        }
    }

    const auto endpointPrimitive = [this](const ExhaustEndpoint& endpoint)
        -> const PrimitiveState& {
        if (endpoint.type == ExhaustEndpointType::junction)
            return junctionPrimitives_[endpoint.elementIndex];
        const auto& primitives = ducts_[endpoint.elementIndex].cellPrimitives_;
        return endpoint.type == ExhaustEndpointType::ductInlet
            ? primitives.front() : primitives.back();
    };
    const auto endpointVolume = [this](const ExhaustEndpoint& endpoint) noexcept {
        if (endpoint.type == ExhaustEndpointType::junction)
            return layout_.junctions()[endpoint.elementIndex].volumeM3;
        const auto& duct = ducts_[endpoint.elementIndex];
        const auto cellIndex = endpoint.type == ExhaustEndpointType::ductInlet
            ? std::size_t { 0 } : duct.cells().size() - 1;
        return duct.cellVolumesM3_[cellIndex];
    };
    const auto constrainBoundary = [this, &stableStep, &endpointPrimitive, &endpointVolume](
        const PrimitiveState& reservoir,
        const ExhaustEndpoint& endpoint,
        double openingAreaM2) noexcept {
        if (!(openingAreaM2 > 0.0)) return;
        const auto& endpointState = endpointPrimitive(endpoint);
        const auto signalSpeed = std::max(
            std::abs(reservoir.velocityMps) + reservoir.speedOfSoundMps,
            std::abs(endpointState.velocityMps) + endpointState.speedOfSoundMps);
        stableStep = std::min(stableStep,
            config_.maximumCourantNumber * endpointVolume(endpoint)
                / (openingAreaM2 * signalSpeed));
    };
    for (std::size_t index = 0; index < layout_.cylinderPorts().size(); ++index) {
        if (cylinderReservoirActive_[index] == 0) continue;
        const auto& port = layout_.cylinderPorts()[index];
        const auto boundaryIndex = cylinderBoundaryIndices_[index];
        if (boundaryIndex >= cylinderBoundaries.size()) continue;
        const auto& boundary = cylinderBoundaries[boundaryIndex];
        const auto opening = std::min(
            std::max(0.0, boundary.effectiveValveAreaM2)
                * std::clamp(boundary.dischargeCoefficient, 0.0, 1.5),
            port.runnerConnectionAreaM2 * port.dischargeCoefficient);
        const auto& reservoirPrimitive = cylinderReservoirPrimitives_[index];
        const auto& networkPrimitive = endpointPrimitive(port.networkEndpoint);
        constrainBoundary(reservoirPrimitive, port.networkEndpoint, opening);
        const auto signalSpeed = std::max(
            reservoirPrimitive.speedOfSoundMps,
            std::abs(networkPrimitive.velocityMps) + networkPrimitive.speedOfSoundMps);
        if (opening > 0.0) {
            stableStep = std::min(stableStep,
                config_.maximumCourantNumber * cylinderReservoirVolumesM3_[index]
                    / (opening * signalSpeed));
        }
    }
    for (const auto& outlet : layout_.outlets()) {
        const auto opening = outlet.openingAreaM2 * outlet.dischargeCoefficient
            * std::clamp(ambient.openingScale, 0.0, 1.0);
        constrainBoundary(ambientPrimitive_, outlet.networkEndpoint, opening);
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
    const auto& junctionPrimitives = useStageState
        ? junctionStagePrimitives_ : junctionPrimitives_;
    const auto& cylinderPrimitives = useStageState
        ? cylinderReservoirStagePrimitives_ : cylinderReservoirPrimitives_;
    const auto endpointState = [this, useStageState, &junctionStates](
        const ExhaustEndpoint& endpoint) -> const ConservativeState& {
        if (endpoint.type == ExhaustEndpointType::junction)
            return junctionStates[endpoint.elementIndex];
        const auto& duct = ducts_[endpoint.elementIndex];
        const auto& states = useStageState ? duct.stage_ : duct.cells_;
        return endpoint.type == ExhaustEndpointType::ductInlet
            ? states.front() : states.back();
    };
    const auto endpointPrimitive = [this, useStageState, &junctionPrimitives](
        const ExhaustEndpoint& endpoint) -> const PrimitiveState& {
        if (endpoint.type == ExhaustEndpointType::junction)
            return junctionPrimitives[endpoint.elementIndex];
        const auto& duct = ducts_[endpoint.elementIndex];
        const auto& primitives = useStageState
            ? duct.stagePrimitives_ : duct.cellPrimitives_;
        return endpoint.type == ExhaustEndpointType::ductInlet
            ? primitives.front() : primitives.back();
    };
    const auto endpointArea = [this](const ExhaustEndpoint& endpoint) noexcept {
        if (endpoint.type == ExhaustEndpointType::junction) {
            const auto diameter = layout_.junctions()[endpoint.elementIndex]
                .characteristicDiameterM;
            return 0.25 * std::acos(-1.0) * diameter * diameter;
        }
        const auto& duct = layout_.ducts()[endpoint.elementIndex];
        return endpoint.type == ExhaustEndpointType::ductInlet
            ? duct.inletConnectionAreaM2
            : duct.outletConnectionAreaM2;
    };
    const auto makeFlowRate = [](const EulerFlux& flux, double areaM2) noexcept {
        ConservedFlowRate result;
        for (std::size_t index = 0; index < gasSpeciesCount; ++index)
            result.speciesMassKgPerS[index] = flux.speciesMassFluxKgPerM2S[index] * areaM2;
        result.momentumN = flux.momentumFluxPa * areaM2;
        result.totalEnergyW = flux.totalEnergyFluxWPerM2 * areaM2;
        return result;
    };
    const auto addJunctionFlow = [&junctionResiduals, &junctionPrimitives, this](
        std::size_t junctionIndex, const ConservedFlowRate& flow,
        double openingAreaM2, double sign) noexcept {
        const auto inverseVolume = 1.0 / layout_.junctions()[junctionIndex].volumeM3;
        for (std::size_t species = 0; species < gasSpeciesCount; ++species) {
            junctionResiduals[junctionIndex].speciesMassDensityKgPerM3[species] +=
                sign * flow.speciesMassKgPerS[species] * inverseVolume;
        }
        // The unresolved wall balances the junction's own static pressure over
        // each face. What remains is the graph-axis pressure difference plus
        // advective momentum. With momentum evolution disabled the junction is
        // the historical well-mixed plenum; when enabled, a collector carries
        // momentum toward its trunk without inventing a net force at rest.
        const auto resolvedAxialForceN = flow.momentumN
            - junctionPrimitives[junctionIndex].pressurePa * openingAreaM2;
        if (config_.evolveJunctionAxialMomentum) {
            junctionResiduals[junctionIndex].momentumDensityKgPerM2S +=
                sign * resolvedAxialForceN * inverseVolume;
        }
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
        const auto faceIndex = endpoint.type == ExhaustEndpointType::ductInlet
            ? std::size_t { 0 } : faceFluxes.size() - 1;
        const auto residualIndex = endpoint.type == ExhaustEndpointType::ductInlet
            ? std::size_t { 0 } : residual.size() - 1;
        const auto sign = endpoint.type == ExhaustEndpointType::ductInlet ? 1.0 : -1.0;
        const auto faceAreaM2 = duct.faceAreasM2_[faceIndex];
        const auto inverseCellVolumeM3 =
            duct.inverseCellVolumesM3_[residualIndex];
        const auto& oldFlux = faceFluxes[faceIndex];
        for (std::size_t species = 0; species < gasSpeciesCount; ++species) {
            residual[residualIndex].speciesMassDensityKgPerM3[species] +=
                sign * faceAreaM2 * inverseCellVolumeM3
                * (newFlux.speciesMassFluxKgPerM2S[species]
                   - oldFlux.speciesMassFluxKgPerM2S[species]);
        }
        residual[residualIndex].momentumDensityKgPerM2S +=
            sign * faceAreaM2 * inverseCellVolumeM3
            * (newFlux.momentumFluxPa - oldFlux.momentumFluxPa);
        residual[residualIndex].totalEnergyDensityJPerM3 +=
            sign * faceAreaM2 * inverseCellVolumeM3
            * (newFlux.totalEnergyFluxWPerM2 - oldFlux.totalEnergyFluxWPerM2);
        faceFluxes[faceIndex] = newFlux;
        return true;
    };
    const auto ductBoundaryFlux = [this, &endpointPrimitive](
        const ExhaustEndpoint& endpoint,
        const EulerFlux& apertureFlux,
        double apertureAreaM2) noexcept {
        const auto& duct = ducts_[endpoint.elementIndex];
        const auto& geometry = duct.geometry();
        const auto faceIndex = endpoint.type == ExhaustEndpointType::ductInlet
            ? std::size_t { 0 } : geometry.cellCount;
        return areaAveragedBoundaryFlux(apertureFlux, apertureAreaM2,
            duct.faceAreasM2_[faceIndex],
            endpointPrimitive(endpoint).pressurePa);
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
            ? mixtureModel_.riemannFluxPrepared(
                endpointState(upstream), endpointPrimitive(upstream),
                endpointState(downstream), endpointPrimitive(downstream))
            : EulerFlux {};
        const auto flow = makeFlowRate(rawFlux, effectiveArea);
        if (observedFlow) *observedFlow = flow;
        if (upstream.type == ExhaustEndpointType::junction) {
            addJunctionFlow(upstream.elementIndex, flow, effectiveArea, -1.0);
        } else if (!assignDuctBoundary(upstream,
                       ductBoundaryFlux(upstream, rawFlux, effectiveArea))) {
            return false;
        }
        if (downstream.type == ExhaustEndpointType::junction) {
            addJunctionFlow(downstream.elementIndex, flow, effectiveArea, 1.0);
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
        const auto suppliedIndex = cylinderBoundaryIndices_[index];
        auto openingArea = 0.0;
        if (suppliedIndex < cylinderBoundaries.size()) {
            const auto& supplied = cylinderBoundaries[suppliedIndex];
            openingArea = std::min(
                std::max(0.0, supplied.effectiveValveAreaM2)
                    * std::clamp(supplied.dischargeCoefficient, 0.0, 1.5),
                port.runnerConnectionAreaM2 * port.dischargeCoefficient);
        }

        const auto& activeCylinderState = useStageState
            ? cylinderReservoirStage_[index] : cylinderReservoirStates_[index];
        const auto networkState = endpointState(port.networkEndpoint);
        const auto rawFlux = openingArea > 0.0
            ? compressibleValveFlux(
                activeCylinderState, cylinderPrimitives[index],
                networkState, endpointPrimitive(port.networkEndpoint))
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
            addJunctionFlow(port.networkEndpoint.elementIndex, cylinderFlows[index],
                openingArea, 1.0);
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
        // Terminal openings use the characteristic open-end boundary rather
        // than a Riemann problem against a cell of ambient air; see
        // openEndBoundaryPrimitive. The Riemann form is kept as the fallback
        // for a state the characteristic reconstruction cannot represent, so an
        // unphysical corner degrades to the previous behaviour instead of
        // producing no flux at all.
        auto rawFlux = EulerFlux {};
        if (openingArea > 0.0) {
            const auto& interiorPrimitive = endpointPrimitive(outlet.networkEndpoint);
            const auto boundary = openEndBoundaryPrimitive(
                interiorPrimitive, ambientPrimitive_);
            const auto boundaryState = boundary
                ? mixtureModel_.conservativeFromPrimitive(
                    boundary->densityKgPerM3, boundary->velocityMps, boundary->pressurePa,
                    GasComposition { boundary->massFractions })
                : std::nullopt;
            // Ghost-cell form deliberately, not the boundary state's physical
            // flux: the terminal cell is not uniformly at the exit state, and
            // taking the raw flux there let a blowdown drain the whole cell in
            // one substep. Going back through the Riemann solver keeps its wave
            // speeds and positivity safeguards while the ghost still carries the
            // atmosphere's pressure with the exhaust's own density (or, on
            // inflow, the reservoir's own gas already moving at its isentropic
            // entry speed), so the duct neither shoves a column of cold dense
            // air aside nor meters its intake by the acoustic impedance.
            rawFlux = boundaryState && mixtureModel_.isPhysical(*boundaryState)
                ? mixtureModel_.riemannFluxPrepared(
                    endpointState(outlet.networkEndpoint), interiorPrimitive,
                    *boundaryState, *boundary)
                : mixtureModel_.riemannFluxPrepared(
                    endpointState(outlet.networkEndpoint), interiorPrimitive,
                    ambient.reservoirState, ambientPrimitive_);
        }
        outletFlows[index] = makeFlowRate(rawFlux, openingArea);
        if (outlet.networkEndpoint.type == ExhaustEndpointType::junction) {
            addJunctionFlow(outlet.networkEndpoint.elementIndex, outletFlows[index],
                openingArea, -1.0);
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

bool ExhaustGasNetwork::prepareStageStates(
    bool candidateStage,
    bool deferDynamicWallSources) noexcept {
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
        if (candidateStage && deferDynamicWallSources
            && duct.geometry_.dynamicWallHeatTransferEnabled) {
            if (!duct.recoverPrimitiveStates(states, primitives)) return false;
        } else {
            if (!duct.prepareStateCache(states, primitives, sourceTerms,
                    maximumSignalSpeed, sourceLimitedTimeStep))
                return false;
        }
    }
    const auto& states = candidateStage ? junctionCandidate_ : junctionStage_;
    auto& junctionPrimitives = candidateStage
        ? junctionCandidatePrimitives_ : junctionStagePrimitives_;
    for (std::size_t index = 0; index < states.size(); ++index) {
        const auto primitive = mixtureModel_.primitiveFromConservative(states[index]);
        if (!primitive) return false;
        junctionPrimitives[index] = *primitive;
    }
    const auto& reservoirs = candidateStage
        ? cylinderReservoirCandidate_ : cylinderReservoirStage_;
    auto& reservoirPrimitives = candidateStage
        ? cylinderReservoirCandidatePrimitives_ : cylinderReservoirStagePrimitives_;
    for (std::size_t index = 0; index < reservoirs.size(); ++index) {
        if (cylinderReservoirActive_[index] == 0) continue;
        const auto primitive = mixtureModel_.primitiveFromConservative(reservoirs[index]);
        if (!primitive) return false;
        reservoirPrimitives[index] = *primitive;
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
        || ambient.openingScale > 1.0) {
        result.completed = false;
        return result;
    }
    const auto ambientPrimitive = mixtureModel_.primitiveFromConservative(
        ambient.reservoirState);
    if (!ambientPrimitive) {
        result.completed = false;
        return result;
    }
    ambientPrimitive_ = *ambientPrimitive;
    const auto ports = layout_.cylinderPorts();
    const auto suppliedInPortOrder = cylinderBoundaries.size() == ports.size()
        && std::equal(cylinderBoundaries.begin(), cylinderBoundaries.end(),
            ports.begin(), [](const CylinderValveBoundary& boundary,
                              const CompiledCylinderPort& port) {
                return boundary.cylinderId == port.cylinderId;
            });
    std::fill(cylinderBoundaryIndices_.begin(), cylinderBoundaryIndices_.end(),
              cylinderBoundaries.size());
    for (std::size_t index = 0; index < cylinderBoundaries.size(); ++index) {
        const auto& boundary = cylinderBoundaries[index];
        const auto primitive = mixtureModel_.primitiveFromConservative(
            boundary.cylinderState);
        if (!finite(boundary.cylinderVolumeM3) || !(boundary.cylinderVolumeM3 > 0.0)
            || !finite(boundary.effectiveValveAreaM2) || boundary.effectiveValveAreaM2 < 0.0
            || !finite(boundary.dischargeCoefficient) || boundary.dischargeCoefficient < 0.0
            || !primitive) {
            result.completed = false;
            return result;
        }
        const auto port = suppliedInPortOrder ? ports.begin()
                + static_cast<std::ptrdiff_t>(index)
            : std::find_if(ports.begin(), ports.end(),
                [&boundary](const CompiledCylinderPort& candidate) {
                    return candidate.cylinderId == boundary.cylinderId;
                });
        if (port == ports.end()) {
            result.completed = false;
            return result;
        }
        const auto portIndex = suppliedInPortOrder ? index
            : static_cast<std::size_t>(std::distance(ports.begin(), port));
        if (cylinderBoundaryIndices_[portIndex] < cylinderBoundaries.size()) {
            result.completed = false;
            return result;
        }
        cylinderBoundaryIndices_[portIndex] = index;
        cylinderReservoirPrimitives_[portIndex] = *primitive;
    }

    for (std::size_t index = 0; index < cylinderExchanges_.size(); ++index) {
        const auto cylinderId = cylinderExchanges_[index].cylinderId;
        const auto pathIndex = cylinderExchanges_[index].pathIndex;
        cylinderExchanges_[index] = {};
        cylinderExchanges_[index].cylinderId = cylinderId;
        cylinderExchanges_[index].pathIndex = pathIndex;
        const auto boundaryIndex = cylinderBoundaryIndices_[index];
        if (boundaryIndex < cylinderBoundaries.size()) {
            const auto& boundary = cylinderBoundaries[boundaryIndex];
            cylinderReservoirStates_[index] = boundary.cylinderState;
            cylinderReservoirVolumesM3_[index] = boundary.cylinderVolumeM3;
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
                if (!config_.evolveJunctionAxialMomentum)
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

            if (config_.firstOrderTimeIntegration) {
                // The validated stage is the conservative Euler candidate.
                // Reuse the normal candidate/commit path so positivity, wall
                // heat and conservation accounting remain common with RK2.
                for (auto& duct : ducts_) duct.candidate_ = duct.stage_;
                junctionCandidate_ = junctionStage_;
                cylinderReservoirCandidate_ = cylinderReservoirStage_;
                // The trapezoidal transfer accounting below becomes exactly
                // dt*f(U_n) when both slots carry the first-stage flow.
                cylinderSecondStageFlow_ = cylinderFirstStageFlow_;
                outletSecondStageFlow_ = outletFirstStageFlow_;
            } else {
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
                    if (!config_.evolveJunctionAxialMomentum)
                        junctionCandidate_[index].momentumDensityKgPerM2S = 0.0;
                }
                for (std::size_t index = 0;
                     index < cylinderReservoirStates_.size(); ++index) {
                    if (cylinderReservoirActive_[index] == 0) continue;
                    cylinderReservoirCandidate_[index] = rk2Combination(
                        cylinderReservoirStates_[index],
                        cylinderReservoirStage_[index],
                        cylinderReservoirStageResidual_[index], trialStep);
                    cylinderReservoirCandidate_[index].momentumDensityKgPerM2S = 0.0;
                }
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
            // Decided before any work from this attempt's own trial step, so a
            // halved retry re-decides consistently and nothing is committed
            // until the sub-step is accepted. `prepareStageStates` only defers
            // the candidate cache when the walls are actually going to run;
            // skipping the exchange therefore also skips the extra
            // `recoverPrimitiveStates` pass that exists solely to feed it,
            // which is the larger half of what sub-rating buys.
            const auto wallPendingSeconds = wallHeatPendingSeconds_ + trialStep;
            const auto applyWallHeat = config_.wallHeatUpdateExternallyTriggered
                ? wallHeatUpdateRequested_
                : wallPendingSeconds >= config_.wallHeatUpdateIntervalSeconds;
            if (!prepareStageStates(true, applyWallHeat)) {
                ++result.rejectedSubsteps;
                trialStep *= 0.5;
                if (!(trialStep > std::numeric_limits<double>::epsilon()
                                  * std::max(1.0, durationSeconds)))
                    break;
                continue;
            }

            auto candidateWallHeatRejectedJ = 0.0;
            auto candidateWallsAreValid = true;
            for (auto& duct : ducts_) {
                if (!applyWallHeat) break;
                if (!duct.geometry_.dynamicWallHeatTransferEnabled) continue;
                duct.candidateWallStates_ = duct.wallStates_;
                if (!duct.applyDynamicWallHeatTransfer(
                        duct.candidate_, duct.candidatePrimitives_,
                        duct.candidateWallStates_, wallPendingSeconds,
                        candidateWallHeatRejectedJ)
                    || !duct.prepareStateCache(
                        duct.candidate_, duct.candidatePrimitives_,
                        duct.candidateSourceTerms_,
                        duct.maximumCandidateSignalSpeedMps_,
                        duct.candidateSourceLimitedTimeStepSeconds_)) {
                    candidateWallsAreValid = false;
                    break;
                }
            }
            if (!candidateWallsAreValid) {
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
                if (applyWallHeat
                    && duct.geometry_.dynamicWallHeatTransferEnabled)
                    duct.wallStates_.swap(duct.candidateWallStates_);
            }
            if (applyWallHeat) {
                wallHeatPendingSeconds_ = 0.0;
                wallHeatUpdateRequested_ = false;
            } else {
                wallHeatPendingSeconds_ = wallPendingSeconds;
            }
            result.wallHeatRejectedJ += candidateWallHeatRejectedJ;
            if (!result.completed) break;
            junctionStates_.swap(junctionCandidate_);
            junctionPrimitives_.swap(junctionCandidatePrimitives_);
            cylinderReservoirStates_.swap(cylinderReservoirCandidate_);
            cylinderReservoirPrimitives_.swap(
                cylinderReservoirCandidatePrimitives_);
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

std::optional<ExhaustOutletFlowSample> ExhaustGasNetwork::predictOutletTransfer(
    std::size_t outletIndex,
    const ExhaustAmbientBoundary& ambient,
    double durationSeconds) const noexcept {
    if (!configured_ || outletIndex >= layout_.outlets().size()) return std::nullopt;
    if (!finite(durationSeconds) || !(durationSeconds > 0.0)) return std::nullopt;
    const auto& outlet = layout_.outlets()[outletIndex];
    // Only a plain duct end is supported. A junction's primitive cache is not
    // owned by a duct and no compiled runner layout produces one here.
    if (outlet.networkEndpoint.type == ExhaustEndpointType::junction)
        return std::nullopt;
    const auto& duct = ducts_[outlet.networkEndpoint.elementIndex];
    if (!duct.refreshCellStateCache()) return std::nullopt;
    const auto& compiled = layout_.ducts()[outlet.networkEndpoint.elementIndex];
    const auto isInlet = outlet.networkEndpoint.type == ExhaustEndpointType::ductInlet;
    const auto& interiorState = isInlet ? duct.cells_.front() : duct.cells_.back();
    const auto& interiorPrimitive =
        isInlet ? duct.cellPrimitives_.front() : duct.cellPrimitives_.back();
    const auto connectionAreaM2 = isInlet
        ? compiled.inletConnectionAreaM2 : compiled.outletConnectionAreaM2;
    const auto openingArea = std::min(connectionAreaM2,
        outlet.openingAreaM2 * outlet.dischargeCoefficient
            * std::clamp(ambient.openingScale, 0.0, 1.0));

    ExhaustOutletFlowSample sample;
    sample.outletNodeId = outlet.outletNodeId;
    sample.pathIndex = outlet.pathIndex;
    sample.openingAreaM2 = outlet.openingAreaM2;
    if (!(openingArea > 0.0)) return sample;

    PrimitiveState reservoirPrimitive;
    if (!mixtureModel_.recoverPrimitive(ambient.reservoirState, reservoirPrimitive))
        return std::nullopt;
    // Same characteristic open-end treatment `evaluateStage` uses, with the
    // same Riemann-ghost fallback, so the prediction is the flux the advance
    // would start from rather than a second, differently-behaved boundary.
    const auto mouthFlux = [&](const ConservativeState& state,
                               const PrimitiveState& primitive) noexcept {
        const auto ghost = openEndBoundaryPrimitive(primitive, reservoirPrimitive);
        const auto ghostState = ghost
            ? mixtureModel_.conservativeFromPrimitive(
                ghost->densityKgPerM3, ghost->velocityMps, ghost->pressurePa,
                GasComposition { ghost->massFractions })
            : std::nullopt;
        return ghostState && mixtureModel_.isPhysical(*ghostState)
            ? mixtureModel_.riemannFluxPrepared(state, primitive, *ghostState, *ghost)
            : mixtureModel_.riemannFluxPrepared(
                state, primitive, ambient.reservoirState, reservoirPrimitive);
    };
    const auto firstFlux = mouthFlux(interiorState, interiorPrimitive);

    // Heun on the terminal cell. What `advance` books is the two-stage average
    // 0.5*dt*(F1+F2), not dt*F1, and for a prediction whose only job is to
    // order several networks around one shared reservoir that difference IS the
    // residual error: with dt*F1 alone the stationary manifold settles 0.011
    // kPa off ambient, against 0.0004 for the serial reference it stands in
    // for. Advancing a LOCAL copy of the terminal cell by the mouth flux -- the
    // term that makes F2 differ from F1 to leading order -- recovers most of
    // it, and costs one more Riemann evaluation on a ~5 us advance.
    auto secondFlux = firstFlux;
    const auto terminalVolumeM3 = isInlet
        ? duct.cellVolumesM3_.front() : duct.cellVolumesM3_.back();
    if (terminalVolumeM3 > 0.0) {
        const auto weight = -openingArea * durationSeconds / terminalVolumeM3;
        auto predictedState = interiorState;
        for (std::size_t species = 0; species < gasSpeciesCount; ++species) {
            predictedState.speciesMassDensityKgPerM3[species] +=
                firstFlux.speciesMassFluxKgPerM2S[species] * weight;
        }
        predictedState.momentumDensityKgPerM2S += firstFlux.momentumFluxPa * weight;
        predictedState.totalEnergyDensityJPerM3 += firstFlux.totalEnergyFluxWPerM2 * weight;
        PrimitiveState predictedPrimitive;
        // An inadmissible extrapolation just falls back to the one-stage
        // estimate rather than poisoning the staircase.
        if (mixtureModel_.recoverPrimitive(predictedState, predictedPrimitive))
            secondFlux = mouthFlux(predictedState, predictedPrimitive);
    }

    // No orientation flip: `outletSamples_` accumulates makeFlowRate(rawFlux,
    // area) unsigned by endpoint type, so a prediction that flipped it would
    // not be comparable with the transfer it is standing in for.
    const auto scale = 0.5 * openingArea * durationSeconds;
    for (std::size_t species = 0; species < gasSpeciesCount; ++species) {
        sample.speciesMassKg[species] = scale
            * (firstFlux.speciesMassFluxKgPerM2S[species]
               + secondFlux.speciesMassFluxKgPerM2S[species]);
    }
    sample.transferredEnergyJ = scale
        * (firstFlux.totalEnergyFluxWPerM2 + secondFlux.totalEnergyFluxWPerM2);
    sample.staticPressurePa = interiorPrimitive.pressurePa;
    sample.temperatureK = interiorPrimitive.temperatureK;
    sample.densityKgPerM3 = interiorPrimitive.densityKgPerM3;
    sample.axialVelocityMps = interiorPrimitive.velocityMps;
    if (durationSeconds > 0.0) {
        auto massKg = 0.0;
        for (const auto species : sample.speciesMassKg) massKg += species;
        sample.massFlowKgPerS = massKg / durationSeconds;
        sample.totalEnergyFlowW = sample.transferredEnergyJ / durationSeconds;
        sample.volumeFlowM3PerS = sample.densityKgPerM3 > 0.0
            ? sample.massFlowKgPerS / sample.densityKgPerM3 : 0.0;
    }
    return sample;
}

bool ExhaustGasNetwork::injectSpeciesAtPort(std::size_t portIndex,
                                            GasSpecies species,
                                            double massKg,
                                            double temperatureK,
                                            double additionalHeatJ) noexcept {
    if (!configured_ || portIndex >= layout_.cylinderPorts().size()) return false;
    if (!finite(massKg) || massKg < 0.0 || !finite(temperatureK)
        || temperatureK <= 0.0 || !finite(additionalHeatJ))
        return false;
    if (massKg == 0.0 && additionalHeatJ == 0.0) return true;
    const auto& endpoint = layout_.cylinderPorts()[portIndex].networkEndpoint;
    if (endpoint.type == ExhaustEndpointType::junction) return false;
    auto& duct = ducts_[endpoint.elementIndex];
    // The source is spread over the spray-and-film footprint next to the
    // valve, not concentrated in the single adjacent cell. An injector cone
    // and its wall film physically wet several centimetres of port, and with
    // the valve shut the adjacent cell is a near-stagnant sliver of gas: on a
    // large cylinder one cycle's evaporation cooling dumped there chills it
    // by hundreds of kelvin (measured on the 2.25 L/cyl V12 at idle: the port
    // reading fell 62 -> 27 degC within one valve event and kept falling,
    // which starved vaporisation and stalled the engine).
    const auto spreadCells = std::min<std::size_t>(3, duct.cells_.size());
    if (spreadCells == 0) return false;
    const auto speciesIndex = static_cast<std::size_t>(species);
    const auto specificHeatCv =
        mixtureModel_.specificHeatCapacityCvJPerKgK_[speciesIndex];
    const auto share = 1.0 / static_cast<double>(spreadCells);
    const auto energyShareJ =
        (massKg * specificHeatCv * temperatureK + additionalHeatJ) * share;
    // All-or-nothing: every touched cell must stay admissible before any is
    // committed, so a pathological command cannot half-apply.
    std::array<ConservativeState, 3> candidates {};
    for (std::size_t offset = 0; offset < spreadCells; ++offset) {
        const auto cellIndex = endpoint.type == ExhaustEndpointType::ductInlet
            ? offset : duct.cells_.size() - 1 - offset;
        const auto cellVolumeM3 = duct.geometry_.cellVolumeM3(cellIndex);
        if (!(cellVolumeM3 > 0.0)) return false;
        auto candidate = duct.cells_[cellIndex];
        candidate.speciesMassDensityKgPerM3[speciesIndex] +=
            massKg * share / cellVolumeM3;
        candidate.totalEnergyDensityJPerM3 += energyShareJ / cellVolumeM3;
        if (!mixtureModel_.isPhysical(candidate)) return false;
        candidates[offset] = candidate;
    }
    for (std::size_t offset = 0; offset < spreadCells; ++offset) {
        const auto cellIndex = endpoint.type == ExhaustEndpointType::ductInlet
            ? offset : duct.cells_.size() - 1 - offset;
        duct.cells_[cellIndex] = candidates[offset];
    }
    duct.cellStateCacheIsValid_ = false;
    return true;
}

void ExhaustGasNetwork::updateOutletSamples(double durationSeconds) noexcept {
    const auto endpointPrimitive = [this](const ExhaustEndpoint& endpoint)
        -> const PrimitiveState& {
        if (endpoint.type == ExhaustEndpointType::junction)
            return junctionPrimitives_[endpoint.elementIndex];
        const auto& primitives = ducts_[endpoint.elementIndex].cellPrimitives_;
        return endpoint.type == ExhaustEndpointType::ductInlet
            ? primitives.front() : primitives.back();
    };
    for (std::size_t index = 0; index < cylinderExchanges_.size(); ++index) {
        if (cylinderReservoirActive_[index] != 0) {
            const auto& cylinderPrimitive = cylinderReservoirPrimitives_[index];
            cylinderExchanges_[index].cylinderPressurePaAfter =
                cylinderPrimitive.pressurePa;
            cylinderExchanges_[index].cylinderTemperatureKAfter =
                cylinderPrimitive.temperatureK;
        }
        const auto& endpoint = layout_.cylinderPorts()[index].networkEndpoint;
        const auto& primitive = endpointPrimitive(endpoint);
        cylinderExchanges_[index].networkPressurePa = primitive.pressurePa;
        cylinderExchanges_[index].networkTemperatureK = primitive.temperatureK;
        cylinderExchanges_[index].networkDensityKgPerM3 = primitive.densityKgPerM3;
        cylinderExchanges_[index].networkVelocityMps = primitive.velocityMps;
        cylinderExchanges_[index].networkSpeedOfSoundMps = primitive.speedOfSoundMps;
    }
    for (std::size_t index = 0; index < outletSamples_.size(); ++index) {
        auto& sample = outletSamples_[index];
        const auto& endpoint = layout_.outlets()[index].networkEndpoint;
        const auto& primitive = endpointPrimitive(endpoint);
        sample.staticPressurePa = primitive.pressurePa;
        sample.temperatureK = primitive.temperatureK;
        sample.axialVelocityMps = primitive.velocityMps;
        sample.densityKgPerM3 = primitive.densityKgPerM3;
        if (durationSeconds > 0.0) {
            sample.massFlowKgPerS = sumSpecies(sample.speciesMassKg) / durationSeconds;
            sample.totalEnergyFlowW = sample.transferredEnergyJ / durationSeconds;
            sample.volumeFlowM3PerS = sample.densityKgPerM3 > 0.0
                ? sample.massFlowKgPerS / sample.densityKgPerM3 : 0.0;
        }
    }
}

} // namespace enginelab::gasdynamics
