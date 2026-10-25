#include <enginelab/gasdynamics/PartitionedExhaustNetwork.hpp>

#include <algorithm>
#include <numeric>
#include <utility>

namespace enginelab::gasdynamics {

namespace {

[[nodiscard]] std::size_t findRoot(std::vector<std::size_t>& parent, std::size_t element) {
    while (parent[element] != element) {
        parent[element] = parent[parent[element]];
        element = parent[element];
    }
    return element;
}

void join(std::vector<std::size_t>& parent, std::size_t first, std::size_t second) {
    const auto a = findRoot(parent, first);
    const auto b = findRoot(parent, second);
    // The lower index wins, so a part's label does not depend on the order
    // in which its connections are visited.
    if (a != b) parent[std::max(a, b)] = std::min(a, b);
}

} // namespace

PartitionedExhaustNetwork::PartitionedExhaustNetwork(ThermodynamicModel model) noexcept
    : model_(std::move(model)) {}

bool PartitionedExhaustNetwork::configure(const ExhaustNetworkLayout& layout,
                                          ExhaustGasNetworkConfig config,
                                          bool partition) {
    parts_.clear();
    layout_ = layout;
    const auto ducts = layout.ducts();
    const auto junctions = layout.junctions();
    const auto interfaces = layout.interfaces();
    const auto ports = layout.cylinderPorts();
    const auto outlets = layout.outlets();
    // Ducts first, then junctions, in one index space.
    const auto element = [&ducts](const ExhaustEndpoint& endpoint) {
        return endpoint.type == ExhaustEndpointType::junction
            ? ducts.size() + endpoint.elementIndex : endpoint.elementIndex;
    };
    const auto elementCount = ducts.size() + junctions.size();
    std::vector<std::size_t> parent(elementCount);
    std::iota(parent.begin(), parent.end(), std::size_t { 0 });
    const auto inRange = [&](const ExhaustEndpoint& endpoint) { return element(endpoint) < elementCount; };
    auto ranged = layout.valid();
    for (const auto& face : interfaces) {
        ranged = ranged && inRange(face.upstream) && inRange(face.downstream);
        if (ranged) join(parent, element(face.upstream), element(face.downstream));
    }
    // A cylinder's ports share its reservoir, which an advance evolves.
    for (std::size_t first = 0; first < ports.size() && ranged; ++first) {
        ranged = inRange(ports[first].networkEndpoint);
        for (std::size_t second = first + 1; second < ports.size() && ranged; ++second)
            if (ports[second].cylinderId == ports[first].cylinderId && inRange(ports[second].networkEndpoint))
                join(parent, element(ports[first].networkEndpoint), element(ports[second].networkEndpoint));
    }
    for (const auto& outlet : outlets) ranged = ranged && inRange(outlet.networkEndpoint);

    std::vector<std::size_t> label(elementCount, elementCount);
    std::size_t count = 0;
    if (ranged) {
        for (std::size_t index = 0; index < elementCount; ++index) {
            const auto root = findRoot(parent, index);
            if (label[root] == elementCount) label[root] = count++;
            label[index] = label[root];
        }
    }
    if (!partition || count < 2) {
        // One network, exactly as compiled.
        auto network = std::make_unique<ExhaustGasNetwork>(model_);
        if (!network->configure(layout, config)) return false;
        parts_.push_back(std::move(network));
        ductAt_.resize(ducts.size());
        junctionAt_.resize(junctions.size());
        interfaceAt_.resize(interfaces.size());
        portAt_.resize(ports.size());
        outletAt_.resize(outlets.size());
        for (auto* places : { &ductAt_, &junctionAt_, &interfaceAt_, &portAt_, &outletAt_ })
            for (std::size_t index = 0; index < places->size(); ++index)
                (*places)[index] = { 0, static_cast<std::uint32_t>(index) };
    } else {
        ductAt_.assign(ducts.size(), {});
        junctionAt_.assign(junctions.size(), {});
        interfaceAt_.assign(interfaces.size(), {});
        portAt_.assign(ports.size(), {});
        outletAt_.assign(outlets.size(), {});
        // Each element's position inside its part, in compiled order.
        std::vector<std::uint32_t> localDucts(count, 0);
        std::vector<std::uint32_t> localJunctions(count, 0);
        for (std::size_t index = 0; index < ducts.size(); ++index)
            ductAt_[index] = { static_cast<std::uint32_t>(label[index]), localDucts[label[index]]++ };
        for (std::size_t index = 0; index < junctions.size(); ++index) {
            const auto part = label[ducts.size() + index];
            junctionAt_[index] = { static_cast<std::uint32_t>(part), localJunctions[part]++ };
        }
        const auto localEndpoint = [&](ExhaustEndpoint endpoint) {
            endpoint.elementIndex = endpoint.type == ExhaustEndpointType::junction
                ? junctionAt_[endpoint.elementIndex].local : ductAt_[endpoint.elementIndex].local;
            return endpoint;
        };
        for (std::size_t part = 0; part < count; ++part) {
            std::vector<CompiledExhaustDuct> partDucts;
            std::vector<CompiledExhaustJunction> partJunctions;
            std::vector<CompiledExhaustInterface> partInterfaces;
            std::vector<CompiledCylinderPort> partPorts;
            std::vector<CompiledExhaustOutlet> partOutlets;
            for (std::size_t index = 0; index < ducts.size(); ++index)
                if (label[index] == part) partDucts.push_back(ducts[index]);
            for (std::size_t index = 0; index < junctions.size(); ++index)
                if (label[ducts.size() + index] == part) partJunctions.push_back(junctions[index]);
            for (std::size_t index = 0; index < interfaces.size(); ++index) {
                if (label[element(interfaces[index].upstream)] != part) continue;
                interfaceAt_[index] = { static_cast<std::uint32_t>(part),
                                        static_cast<std::uint32_t>(partInterfaces.size()) };
                auto face = interfaces[index];
                face.upstream = localEndpoint(face.upstream);
                face.downstream = localEndpoint(face.downstream);
                partInterfaces.push_back(face);
            }
            for (std::size_t index = 0; index < ports.size(); ++index) {
                if (label[element(ports[index].networkEndpoint)] != part) continue;
                portAt_[index] = { static_cast<std::uint32_t>(part),
                                   static_cast<std::uint32_t>(partPorts.size()) };
                auto port = ports[index];
                port.networkEndpoint = localEndpoint(port.networkEndpoint);
                partPorts.push_back(port);
            }
            for (std::size_t index = 0; index < outlets.size(); ++index) {
                if (label[element(outlets[index].networkEndpoint)] != part) continue;
                outletAt_[index] = { static_cast<std::uint32_t>(part),
                                     static_cast<std::uint32_t>(partOutlets.size()) };
                auto outlet = outlets[index];
                outlet.networkEndpoint = localEndpoint(outlet.networkEndpoint);
                partOutlets.push_back(outlet);
            }
            const auto partLayout = ExhaustNetworkLayout::assemble(
                std::move(partDucts), std::move(partJunctions), std::move(partInterfaces),
                std::move(partPorts), std::move(partOutlets), layout.discretisation());
            auto network = std::make_unique<ExhaustGasNetwork>(model_);
            if (!partLayout.valid() || !network->configure(partLayout, config)) {
                parts_.clear();
                return false;
            }
            parts_.push_back(std::move(network));
        }
    }
    partBoundaryBegin_.assign(parts_.size() + 1, 0);
    for (const auto place : portAt_) ++partBoundaryBegin_[place.part + 1];
    for (std::size_t part = 0; part < parts_.size(); ++part)
        partBoundaryBegin_[part + 1] += partBoundaryBegin_[part];
    partBoundaryCount_.assign(parts_.size(), 0);
    boundaryScratch_.assign(ports.size(), {});
    sampleBoundaryCount_.assign(parts_.size(), 0);
    sampleBoundaryScratch_.assign(ports.size(), {});
    partSamples_.assign(ports.size(), {});
    jobs_.assign(parts_.size(), {});
    exchanges_.assign(ports.size(), {});
    outletSamples_.assign(outlets.size(), {});
    gatherTransfers();
    return true;
}

void PartitionedExhaustNetwork::setParallelFor(ExhaustGasNetwork::ParallelFor parallelFor) noexcept {
    parallelFor_ = parallelFor;
    if (parts_.size() == 1) parts_.front()->setParallelFor(parallelFor);
}

bool PartitionedExhaustNetwork::reset(double pressurePa, double temperatureK,
                                      GasComposition composition) noexcept {
    auto succeeded = !parts_.empty();
    for (auto& part : parts_) succeeded = part->reset(pressurePa, temperatureK, composition) && succeeded;
    gatherTransfers();
    return succeeded;
}

bool PartitionedExhaustNetwork::adoptStateFrom(const PartitionedExhaustNetwork& previous) noexcept {
    if (parts_.empty() || parts_.size() != previous.parts_.size()) return false;
    for (std::size_t index = 0; index < parts_.size(); ++index)
        if (!parts_[index]->layout().sameTopology(previous.parts_[index]->layout())) return false;
    // A part refuses only before touching itself, but one refusing after
    // another adopted would leave a mix: the layouts are checked above, so
    // what is left is a remapped state that is not physical.
    auto succeeded = true;
    for (std::size_t index = 0; index < parts_.size(); ++index)
        succeeded = parts_[index]->adoptStateFrom(*previous.parts_[index]) && succeeded;
    gatherTransfers();
    return succeeded;
}

double PartitionedExhaustNetwork::peakWallTemperatureK() const noexcept {
    auto peak = 0.0;
    for (const auto& part : parts_) peak = std::max(peak, part->peakWallTemperatureK());
    return peak;
}

ExhaustNetworkInventory PartitionedExhaustNetwork::inventory() const noexcept {
    if (parts_.size() == 1) return parts_.front()->inventory();
    ExhaustNetworkInventory total;
    for (const auto& part : parts_) {
        const auto inventory = part->inventory();
        for (std::size_t species = 0; species < gasSpeciesCount; ++species)
            total.speciesMassKg[species] += inventory.speciesMassKg[species];
        total.totalEnergyJ += inventory.totalEnergyJ;
        total.resolvedAxialMomentumKgMps += inventory.resolvedAxialMomentumKgMps;
        total.wallThermalEnergyJ += inventory.wallThermalEnergyJ;
    }
    return total;
}

ExhaustFuelReactionResult PartitionedExhaustNetwork::reactUnburnedFuel(
    double durationSeconds, const ExhaustFuelReactionConfig& reaction) noexcept {
    if (parts_.size() == 1) return parts_.front()->reactUnburnedFuel(durationSeconds, reaction);
    ExhaustFuelReactionResult total;
    for (auto& part : parts_) {
        const auto result = part->reactUnburnedFuel(durationSeconds, reaction);
        total.burnedFuelMassKg += result.burnedFuelMassKg;
        total.consumedOxygenMassKg += result.consumedOxygenMassKg;
        total.releasedEnergyJoules += result.releasedEnergyJoules;
        total.reactingControlVolumes += result.reactingControlVolumes;
        total.wallIgnitedControlVolumes += result.wallIgnitedControlVolumes;
        total.maximumInductionIntegral = std::max(total.maximumInductionIntegral,
                                                  result.maximumInductionIntegral);
        if (result.minimumInductionDelaySeconds > 0.0)
            total.minimumInductionDelaySeconds = total.minimumInductionDelaySeconds > 0.0
                ? std::min(total.minimumInductionDelaySeconds, result.minimumInductionDelaySeconds)
                : result.minimumInductionDelaySeconds;
        total.maximumInductionDelaySeconds = std::max(total.maximumInductionDelaySeconds,
                                                      result.maximumInductionDelaySeconds);
        total.droppedSourceCount += result.droppedSourceCount;
        for (std::size_t source = 0; source < result.sourceCount; ++source) {
            if (total.sourceCount < ExhaustFuelReactionResult::maximumSources)
                total.sources[total.sourceCount++] = result.sources[source];
            else
                ++total.droppedSourceCount;
        }
    }
    return total;
}

bool PartitionedExhaustNetwork::emitUnburnedPocket(
    std::uint32_t cylinderId, double fuelKg, double oxygenKg,
    double temperatureK, const ExhaustFuelReactionConfig& reaction) noexcept {
    if (parts_.size() == 1)
        return parts_.front()->emitUnburnedPocket(cylinderId, fuelKg,
            oxygenKg, temperatureK, reaction);
    const auto ports = layout_.cylinderPorts();
    const auto port = std::find_if(ports.begin(), ports.end(),
        [cylinderId](const CompiledCylinderPort& candidate) {
            return candidate.cylinderId == cylinderId;
        });
    if (port == ports.end()) return false;
    return parts_[portAt_[static_cast<std::size_t>(port - ports.begin())].part]
        ->emitUnburnedPocket(cylinderId, fuelKg, oxygenKg, temperatureK,
            reaction);
}

void PartitionedExhaustNetwork::igniteUnburnedPockets(
    std::uint32_t cylinderId) noexcept {
    if (parts_.size() == 1) {
        parts_.front()->igniteUnburnedPockets(cylinderId);
        return;
    }
    const auto ports = layout_.cylinderPorts();
    const auto port = std::find_if(ports.begin(), ports.end(),
        [cylinderId](const CompiledCylinderPort& candidate) {
            return candidate.cylinderId == cylinderId;
        });
    if (port == ports.end()) return;
    parts_[portAt_[static_cast<std::size_t>(port - ports.begin())].part]
        ->igniteUnburnedPockets(cylinderId);
}

std::size_t PartitionedExhaustNetwork::activeUnburnedPocketCount()
    const noexcept {
    std::size_t total = 0;
    for (const auto& part : parts_) total += part->activeUnburnedPocketCount();
    return total;
}

ExhaustNetworkAdvanceResult PartitionedExhaustNetwork::advance(
    double durationSeconds,
    std::span<const CylinderValveBoundary> cylinderBoundaries,
    const ExhaustAmbientBoundary& ambient) noexcept {
    if (parts_.size() == 1) {
        const auto result = parts_.front()->advance(durationSeconds, cylinderBoundaries, ambient);
        return result;
    }
    ExhaustNetworkAdvanceResult failed;
    failed.completed = false;
    if (parts_.empty() || ambient.reservoirVolumeM3 > 0.0) return failed;
    // Each boundary goes to the part holding its cylinder's ports, in the
    // order supplied; one no part holds is unknown, as a single network
    // would find it.
    std::fill(partBoundaryCount_.begin(), partBoundaryCount_.end(), std::size_t { 0 });
    const auto ports = layout_.cylinderPorts();
    for (const auto& boundary : cylinderBoundaries) {
        const auto port = std::find_if(ports.begin(), ports.end(),
            [&boundary](const CompiledCylinderPort& candidate) {
                return candidate.cylinderId == boundary.cylinderId;
            });
        if (port == ports.end()) return failed;
        const auto part = portAt_[static_cast<std::size_t>(port - ports.begin())].part;
        auto& filled = partBoundaryCount_[part];
        // More boundaries than the part has ports repeats a cylinder.
        if (partBoundaryBegin_[part] + filled >= partBoundaryBegin_[part + 1]) return failed;
        boundaryScratch_[partBoundaryBegin_[part] + filled++] = boundary;
    }
    for (auto& job : jobs_) {
        job.durationSeconds = durationSeconds;
        job.ambient = &ambient;
    }
    const auto advancePart = [](void* raw, std::size_t part) noexcept {
        auto& self = *static_cast<PartitionedExhaustNetwork*>(raw);
        auto& job = self.jobs_[part];
        job.result = self.parts_[part]->advance(job.durationSeconds,
            std::span<const CylinderValveBoundary>(
                self.boundaryScratch_.data() + self.partBoundaryBegin_[part],
                self.partBoundaryCount_[part]),
            *job.ambient);
    };
    if (parallelFor_.run && parallelFor_.threads > 1)
        parallelFor_.run(parallelFor_.executor, parts_.size(), advancePart, this);
    else
        for (std::size_t part = 0; part < parts_.size(); ++part) advancePart(this, part);

    ExhaustNetworkAdvanceResult total;
    total.advancedTimeSeconds = durationSeconds;
    for (const auto& job : jobs_) {
        const auto& result = job.result;
        total.advancedTimeSeconds = std::min(total.advancedTimeSeconds, result.advancedTimeSeconds);
        total.acceptedSubsteps = std::max(total.acceptedSubsteps, result.acceptedSubsteps);
        total.rejectedSubsteps += result.rejectedSubsteps;
        total.completed = total.completed && result.completed;
        total.wallHeatRejectedJ += result.wallHeatRejectedJ;
        if (result.acceptedSubsteps > 0) {
            total.minimumAcceptedTimeStepSeconds = total.minimumAcceptedTimeStepSeconds > 0.0
                ? std::min(total.minimumAcceptedTimeStepSeconds, result.minimumAcceptedTimeStepSeconds)
                : result.minimumAcceptedTimeStepSeconds;
        }
        total.maximumAcceptedTimeStepSeconds = std::max(total.maximumAcceptedTimeStepSeconds,
                                                        result.maximumAcceptedTimeStepSeconds);
        total.maximumAcceptedStableStepRatio = std::max(total.maximumAcceptedStableStepRatio,
                                                        result.maximumAcceptedStableStepRatio);
        total.maximumAcceptedDuctCourantCurrent = std::max(total.maximumAcceptedDuctCourantCurrent,
                                                           result.maximumAcceptedDuctCourantCurrent);
        total.maximumAcceptedDuctCourantPredictor = std::max(total.maximumAcceptedDuctCourantPredictor,
                                                             result.maximumAcceptedDuctCourantPredictor);
    }
    gatherTransfers();
    return total;
}

bool PartitionedExhaustNetwork::sampleCylinderBoundaries(
    std::span<const CylinderValveBoundary> cylinderBoundaries,
    std::span<CylinderBoundaryFlowSample> samples) const noexcept {
    if (parts_.size() == 1) return parts_.front()->sampleCylinderBoundaries(cylinderBoundaries, samples);
    const auto ports = layout_.cylinderPorts();
    if (parts_.empty() || samples.size() < ports.size()) return false;
    // The same routing as advance, into a scratch of its own: a const call
    // must not disturb the boundaries of the last advance.
    auto& scratch = sampleBoundaryScratch_;
    auto& counts = sampleBoundaryCount_;
    std::fill(counts.begin(), counts.end(), std::size_t { 0 });
    for (const auto& boundary : cylinderBoundaries) {
        const auto port = std::find_if(ports.begin(), ports.end(),
            [&boundary](const CompiledCylinderPort& candidate) {
                return candidate.cylinderId == boundary.cylinderId;
            });
        if (port == ports.end()) return false;
        const auto part = portAt_[static_cast<std::size_t>(port - ports.begin())].part;
        if (partBoundaryBegin_[part] + counts[part] >= partBoundaryBegin_[part + 1]) return false;
        scratch[partBoundaryBegin_[part] + counts[part]++] = boundary;
    }
    for (std::size_t part = 0; part < parts_.size(); ++part) {
        const auto begin = partBoundaryBegin_[part];
        const auto portCount = partBoundaryBegin_[part + 1] - begin;
        if (!parts_[part]->sampleCylinderBoundaries(
                std::span<const CylinderValveBoundary>(scratch.data() + begin, counts[part]),
                std::span<CylinderBoundaryFlowSample>(partSamples_.data() + begin, portCount)))
            return false;
    }
    for (std::size_t index = 0; index < ports.size(); ++index) {
        const auto place = portAt_[index];
        samples[index] = partSamples_[partBoundaryBegin_[place.part] + place.local];
    }
    return true;
}

std::span<const CylinderGasExchange> PartitionedExhaustNetwork::cylinderExchanges() const noexcept {
    if (parts_.size() == 1) return parts_.front()->cylinderExchanges();
    return exchanges_;
}

std::span<const ExhaustOutletFlowSample> PartitionedExhaustNetwork::outletSamples() const noexcept {
    if (parts_.size() == 1) return parts_.front()->outletSamples();
    return outletSamples_;
}

bool PartitionedExhaustNetwork::setRestrictionOpeningAreaM2(std::size_t interfaceIndex,
                                                            double areaM2) noexcept {
    if (interfaceIndex >= interfaceAt_.size()) return false;
    const auto place = interfaceAt_[interfaceIndex];
    return parts_[place.part]->setRestrictionOpeningAreaM2(place.local, areaM2);
}

bool PartitionedExhaustNetwork::setRestrictionTurbine(std::size_t interfaceIndex,
                                                      double wheelShare,
                                                      double efficiency) noexcept {
    if (interfaceIndex >= interfaceAt_.size()) return false;
    const auto place = interfaceAt_[interfaceIndex];
    return parts_[place.part]->setRestrictionTurbine(place.local, wheelShare, efficiency);
}

double PartitionedExhaustNetwork::restrictionWorkJ(std::size_t interfaceIndex) const noexcept {
    if (interfaceIndex >= interfaceAt_.size()) return 0.0;
    const auto place = interfaceAt_[interfaceIndex];
    return parts_[place.part]->restrictionWorkJ(place.local);
}

void PartitionedExhaustNetwork::gatherTransfers() noexcept {
    if (parts_.size() < 2) return;
    for (std::size_t index = 0; index < portAt_.size(); ++index)
        exchanges_[index] = parts_[portAt_[index].part]->cylinderExchanges()[portAt_[index].local];
    for (std::size_t index = 0; index < outletAt_.size(); ++index)
        outletSamples_[index] = parts_[outletAt_[index].part]->outletSamples()[outletAt_[index].local];
}

} // namespace enginelab::gasdynamics
