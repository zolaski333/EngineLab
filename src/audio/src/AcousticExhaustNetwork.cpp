#include <enginelab/audio/AcousticExhaustNetwork.hpp>

#include <enginelab/audio/DuctWallLoss.hpp>
#include <enginelab/audio/NonlinearDuctAcoustics.hpp>
#include <enginelab/audio/PipeRadiationModel.hpp>
#include <enginelab/audio/PorousLinerLoss.hpp>
#include <enginelab/gasdynamics/ExhaustNetworkLayout.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

namespace enginelab {
namespace {

[[nodiscard]] std::size_t nextPowerOfTwo(std::size_t value) noexcept {
    auto result = std::size_t { 1 };
    while (result < value && result <= std::numeric_limits<std::size_t>::max() / 2)
        result <<= 1U;
    return result;
}

[[nodiscard]] bool isDuctEndpoint(
    gasdynamics::ExhaustEndpointType type) noexcept {
    return type != gasdynamics::ExhaustEndpointType::junction;
}

[[nodiscard]] double circularArea(double diameterM) noexcept {
    const auto radiusM = 0.5 * diameterM;
    return std::numbers::pi * radiusM * radiusM;
}

} // namespace

struct AcousticExhaustNetwork::Impl final {
    enum class OwnerType : std::uint8_t {
        none, junction, cylinder, outlet, sideBranchTerminal
    };

    struct EndpointOwner final {
        OwnerType type { OwnerType::none };
        std::size_t index {};
    };

    struct Duct final {
        std::uint32_t nodeId {};
        std::uint32_t pathIndex {};
        double lengthM {};
        /** Length-mean area for wall/mode properties. */
        double areaM2 {};
        /** Characteristic areas at the two scattering endpoints. */
        double inletAreaM2 {};
        double outletAreaM2 {};
        double radiusM {};
        /** Compiled ducts whose gas state describes this one, as indices into
         *  the layout's duct list. A duct of the layout resolves to itself; a
         *  branch trunk, which the layout carries as a junction, resolves to the
         *  ducts on either side of it. Empty means fall back to the path. */
        std::vector<std::size_t> mediumSources;
        /** Live and target gas state of this duct. Held per duct rather than per
         *  path because delay and wall/liner losses are all local. */
        Medium medium {};
        Medium mediumTarget {};
        std::vector<float> forward;
        std::vector<float> reverse;
        std::size_t write {};
        std::size_t mask {};
        float delaySamples { 1.0F };
        float delayTargetSamples { 1.0F };
        DuctWallLoss::Coefficients wallLoss {};
        DuctWallLoss::Coefficients wallLossTarget {};
        DuctWallLoss::State forwardLoss {};
        DuctWallLoss::State reverseLoss {};
        double packingFlowResistivityPaSPerM2 { 0.0 };
        double packingThicknessM { 0.0 };
        double perforatedOpenAreaRatio { 0.0 };
        DuctWallLoss::Coefficients linerLoss {};
        DuctWallLoss::Coefficients linerLossTarget {};
        DuctWallLoss::State forwardLinerLoss {};
        DuctWallLoss::State reverseLinerLoss {};
    };

    struct Junction final {
        std::vector<std::size_t> ductEndpoints;
        std::vector<std::size_t> cylinderTerminals;
        std::vector<std::size_t> outletTerminals;
        std::uint32_t pathIndex {};
        /** Compact gas volume which is not already represented by a trunk
         *  delay line. It is a shunt acoustic compliance at the scattering
         *  node, C = V/(rho*c^2). */
        double compactVolumeM3 {};
        float complianceAdmittanceM3PerPaSecond {};
        float complianceAdmittanceTargetM3PerPaSecond {};
        /** Incident wave retained by the trapezoidal/WDF compliance port. */
        float complianceIncidentPressurePa {};
        float pendingSourcePressurePa {};
    };

    struct CylinderPort final {
        std::size_t audioCylinderIndex {};
        std::uint32_t pathIndex {};
        double areaM2 {};
        bool virtualTerminal { false };
        std::size_t endpointOrJunction {};
        float incidentToJunction {};
        ValvePortTermination::State reflectionState {};
    };

    struct Outlet final {
        std::uint32_t nodeId {};
        std::uint32_t pathIndex {};
        double areaM2 {};
        bool virtualTerminal { false };
        std::size_t endpointOrJunction {};
        float incidentToJunction {};
        UnflangedPipeRadiation radiation;
        FreeFieldObserver observer;
        FreeFieldObserver jetNoiseObserver;
        ExhaustJetNoise jetNoise;
        double radiationDensityKgPerM3 { 1.2 };
        double radiationSoundSpeedMps { 343.0 };
        double radiationTargetDensityKgPerM3 { 1.2 };
        double radiationTargetSoundSpeedMps { 343.0 };
        // Mean-flow convective loss at the open termination: |R| -> (1-M)/(1+M).
        double outletMach { 0.0 };
        double outletMachTarget { 0.0 };
        double convectiveReflection { 1.0 };
        AcousticPoint3M acousticPositionM {};
        AcousticPoint3M acousticAxis { 0.0, 1.0, 0.0 };
        AcousticTerminationType acousticTermination {
            AcousticTerminationType::unflanged };
    };

    struct SideBranchTerminal final {
        std::size_t ductIndex {};
        std::uint32_t pathIndex {};
        double cavityVolumeM3 {};
        float complianceAdmittanceM3PerPaSecond {};
        float complianceAdmittanceTargetM3PerPaSecond {};
        float complianceIncidentPressurePa {};
    };

    struct ReactionTarget final {
        std::uint32_t nodeId {};
        bool duct { true };
        std::size_t index {};
        std::size_t ductCount { 1 };
        bool useEventAxialPosition { true };
    };

    gasdynamics::ExhaustNetworkLayout layout;
    std::vector<Duct> ducts;
    std::vector<Junction> junctions;
    std::vector<CylinderPort> cylinderPorts;
    std::vector<Outlet> outlets;
    std::vector<SideBranchTerminal> sideBranchTerminals;
    std::vector<ReactionTarget> reactionTargets;
    std::vector<EndpointOwner> owners;
    std::vector<float> incident;
    std::vector<float> outgoing;
    std::array<Medium, maximumPaths> media {};
    std::array<Medium, maximumPaths> mediaTarget {};
    std::array<StereoPressure, maximumPaths> lastJetNoise {};
    bool jetNoiseEnabled { true };
    bool mediaInitialised { false };
    double sampleRateHz { 48'000.0 };
    double maximumDelayScale { 12.5 };
    // Note: the observer distance is validated as an API contract in prepare()
    // but not stored here. Radiation spreading to the observer is owned by each
    // path's PipeRadiationModel (prepared separately in RealtimeEngineAudio),
    // never by this network, so a stored copy here would be dead state.
    AcousticObserverConfig observerConfig;
    bool configured { false };
    bool prepared { false };

    explicit Impl(const ExhaustGraph& graph,
                  std::span<const std::uint32_t> cylinderIds)
        : layout(gasdynamics::ExhaustNetworkLayout::compile(graph)),
          observerConfig(graph.acousticObserver()) {
        if (!layout.valid()) return;
        struct DuctPlan final {
            std::size_t firstDuctIndex {};
            std::size_t sectionCount { 1 };
        };
        std::vector<DuctPlan> ductPlans(layout.ducts().size());
        std::vector<std::size_t> requestedTaperSections(
            layout.ducts().size(), 1U);
        for (std::size_t layoutIndex = 0;
             layoutIndex < layout.ducts().size(); ++layoutIndex) {
            const auto& descriptor = layout.ducts()[layoutIndex];
            const auto inletAreaM2 = std::max(
                1.0e-10, descriptor.inletFlowAreaM2);
            const auto outletAreaM2 = std::max(
                1.0e-10, descriptor.outletFlowAreaM2);
            const auto areaRatio = std::max(inletAreaM2, outletAreaM2)
                / std::min(inletAreaM2, outletAreaM2);
            if (areaRatio > 1.05) {
                // Keep each step modest, but cap the per-taper cost. Four
                // pressure-wave sections are enough to distinguish a finite
                // cone from one abrupt discontinuity without turning an
                // authored flare into a fine audio mesh.
                constexpr double maximumAreaRatioPerSection = 1.45;
                const auto requested = static_cast<std::size_t>(std::ceil(
                    std::log(areaRatio) / std::log(maximumAreaRatioPerSection)));
                constexpr double minimumSectionLengthM = 0.025;
                const auto resolvableByLength = static_cast<std::size_t>(
                    std::floor(descriptor.lengthM / minimumSectionLengthM));
                requestedTaperSections[layoutIndex] = std::clamp<std::size_t>(
                    std::min(requested, resolvableByLength), 1U, 4U);
            }
        }

        // Complexity is budgeted for the complete graph, not independently
        // per component. Twelve parallel Merlin ejector stacks requesting four
        // sections each measured 61.4% mean DSP and missed the 5.33 ms block
        // deadline; the previous one-line network was 41.9%. Allocate only
        // complete refinement rounds so equivalent parallel branches remain
        // symmetric. A lone taper can still receive all four sections, while
        // the whole network adds at most sixteen delay lines/junctions.
        constexpr std::size_t maximumAdditionalTaperSections = 16U;
        auto remainingTaperSections = maximumAdditionalTaperSections;
        std::vector<std::size_t> taperSections(layout.ducts().size(), 1U);
        for (std::size_t refinement = 2U; refinement <= 4U; ++refinement) {
            const auto candidates = static_cast<std::size_t>(std::count_if(
                requestedTaperSections.begin(), requestedTaperSections.end(),
                [refinement](std::size_t requested) {
                    return requested >= refinement;
                }));
            if (candidates == 0U) continue;
            if (candidates > remainingTaperSections) break;
            for (std::size_t index = 0; index < taperSections.size(); ++index)
                if (requestedTaperSections[index] >= refinement)
                    taperSections[index] = refinement;
            remainingTaperSections -= candidates;
        }

        ducts.reserve(layout.ducts().size()
            + maximumAdditionalTaperSections + layout.junctions().size()
            + graph.acousticSideBranches().size());
        for (std::size_t layoutIndex = 0;
             layoutIndex < layout.ducts().size(); ++layoutIndex) {
            const auto& descriptor = layout.ducts()[layoutIndex];
            const auto inletAreaM2 = std::max(
                1.0e-10, descriptor.inletFlowAreaM2);
            const auto outletAreaM2 = std::max(
                1.0e-10, descriptor.outletFlowAreaM2);
            const auto sectionCount = taperSections[layoutIndex];
            auto& plan = ductPlans[layoutIndex];
            plan.firstDuctIndex = ducts.size();
            plan.sectionCount = sectionCount;

            const auto inletRadiusM = std::sqrt(inletAreaM2 / std::numbers::pi);
            const auto outletRadiusM = std::sqrt(outletAreaM2 / std::numbers::pi);
            for (std::size_t section = 0; section < sectionCount; ++section) {
                const auto t0 = static_cast<double>(section)
                    / static_cast<double>(sectionCount);
                const auto t1 = static_cast<double>(section + 1U)
                    / static_cast<double>(sectionCount);
                const auto sectionInletRadiusM = std::lerp(
                    inletRadiusM, outletRadiusM, t0);
                const auto sectionOutletRadiusM = std::lerp(
                    inletRadiusM, outletRadiusM, t1);
                const auto sectionInletAreaM2 = std::numbers::pi
                    * sectionInletRadiusM * sectionInletRadiusM;
                const auto sectionOutletAreaM2 = std::numbers::pi
                    * sectionOutletRadiusM * sectionOutletRadiusM;
                const auto sectionAreaM2 = (sectionInletAreaM2
                    + std::sqrt(sectionInletAreaM2 * sectionOutletAreaM2)
                    + sectionOutletAreaM2) / 3.0;

                Duct duct;
                duct.nodeId = descriptor.nodeId;
                duct.pathIndex = descriptor.pathIndex;
                duct.lengthM = descriptor.lengthM
                    / static_cast<double>(sectionCount);
                duct.areaM2 = std::max(1.0e-10, sectionAreaM2);
                duct.inletAreaM2 = std::max(1.0e-10, sectionInletAreaM2);
                duct.outletAreaM2 = std::max(1.0e-10, sectionOutletAreaM2);
                duct.radiusM = descriptor.homogenisedCatalystMonolith
                    ? 0.5 * descriptor.hydraulicDiameterM
                    : std::sqrt(duct.areaM2 / std::numbers::pi);
                duct.packingFlowResistivityPaSPerM2 =
                    descriptor.packingFlowResistivityPaSPerM2;
                duct.packingThicknessM = descriptor.packingThicknessM;
                duct.perforatedOpenAreaRatio =
                    descriptor.perforatedOpenAreaRatio;
                duct.mediumSources.push_back(layoutIndex);
                ducts.push_back(std::move(duct));
            }
        }
        struct SideBranchPlan final {
            std::size_t ductIndex {};
            std::uint32_t attachmentNodeId {};
            std::uint32_t pathIndex {};
            double cavityVolumeM3 {};
        };
        std::vector<SideBranchPlan> sideBranchPlans;
        sideBranchPlans.reserve(graph.acousticSideBranches().size());
        for (const auto& descriptor : graph.acousticSideBranches()) {
            const auto effectiveLengthM = descriptor.referenceTuningHz > 0.0
                ? graph.referenceWaveSpeedMps()
                    / (4.0 * descriptor.referenceTuningHz)
                : descriptor.lengthM;
            if (!(effectiveLengthM > 0.0) || !(descriptor.diameterM > 0.0))
                return;
            const auto areaM2 = std::max(
                1.0e-10, circularArea(descriptor.diameterM));
            Duct duct;
            duct.nodeId = descriptor.sourceComponentId;
            duct.pathIndex = descriptor.pathIndex;
            duct.lengthM = std::clamp(effectiveLengthM, 0.005, 10.0);
            duct.areaM2 = areaM2;
            duct.inletAreaM2 = areaM2;
            duct.outletAreaM2 = areaM2;
            duct.radiusM = 0.5 * descriptor.diameterM;
            const auto host = std::find_if(
                layout.ducts().begin(), layout.ducts().end(),
                [&descriptor](const auto& candidate) {
                    return candidate.nodeId == descriptor.attachmentNodeId;
                });
            if (host != layout.ducts().end())
                duct.mediumSources.push_back(static_cast<std::size_t>(
                    std::distance(layout.ducts().begin(), host)));
            const auto ductIndex = ducts.size();
            ducts.push_back(std::move(duct));
            sideBranchPlans.push_back({
                ductIndex,
                descriptor.attachmentNodeId,
                descriptor.pathIndex,
                std::max(0.0, descriptor.terminalVolumeM3),
            });
        }
        const auto compiledAcousticDuctCount = ducts.size();

        const auto acousticEndpointKey = [&ductPlans](
            const gasdynamics::ExhaustEndpoint& endpoint) {
            const auto& plan = ductPlans[endpoint.elementIndex];
            const auto ductIndex = endpoint.type
                    == gasdynamics::ExhaustEndpointType::ductOutlet
                ? plan.firstDuctIndex + plan.sectionCount - 1U
                : plan.firstDuctIndex;
            return ductIndex * 2U
                + (endpoint.type == gasdynamics::ExhaustEndpointType::ductOutlet
                    ? 1U : 0U);
        };

        // Junction-to-junction edges connect two branches directly. When neither
        // carries an authored trunk they are coincident and form one physical
        // scattering node, so union them. When either does, the two branches are
        // separated by that pipe and have to stay distinct.
        std::vector<std::size_t> parent(layout.junctions().size());
        for (std::size_t index = 0; index < parent.size(); ++index) parent[index] = index;
        const auto rootOf = [&parent](std::size_t index) {
            auto root = index;
            while (parent[root] != root) root = parent[root];
            while (parent[index] != index) {
                const auto next = parent[index];
                parent[index] = root;
                index = next;
            }
            return root;
        };
        const auto junctionTrunkLengthM = [this](std::size_t index) {
            return layout.junctions()[index].trunkLengthM;
        };
        for (const auto& interface : layout.interfaces()) {
            if (interface.upstream.type != gasdynamics::ExhaustEndpointType::junction
                || interface.downstream.type != gasdynamics::ExhaustEndpointType::junction)
                continue;
            if (junctionTrunkLengthM(interface.upstream.elementIndex) > 0.0
                || junctionTrunkLengthM(interface.downstream.elementIndex) > 0.0)
                continue;
            const auto left = rootOf(interface.upstream.elementIndex);
            const auto right = rootOf(interface.downstream.elementIndex);
            if (left != right) parent[right] = left;
        }
        std::unordered_map<std::size_t, std::size_t> groupForRoot;
        std::vector<std::size_t> groupForJunction(layout.junctions().size());
        auto groupCount = std::size_t { 0 };
        for (std::size_t index = 0; index < layout.junctions().size(); ++index) {
            const auto [found, inserted] = groupForRoot.emplace(rootOf(index), groupCount);
            if (inserted) ++groupCount;
            groupForJunction[index] = found->second;
        }

        // Resolve each group's trunk before any storage is sized, because a
        // junction carrying an authored length becomes a duct of its own.
        struct GroupGeometry final {
            double lengthM { 0.0 };
            /** Sum of area * length over the group, so a chain of unioned
             *  junctions recovers a single length-weighted mean area. */
            double sweptVolumeM3 { 0.0 };
            /** Well-mixed volume left after the authored trunk has been
             *  represented explicitly as a waveguide. */
            double compactVolumeM3 { 0.0 };
            std::uint32_t pathIndex { 0 };
            std::size_t inflowCount { 0 };
            std::size_t outflowCount { 0 };
        };
        std::vector<GroupGeometry> groupGeometry(groupCount);
        for (std::size_t index = 0; index < layout.junctions().size(); ++index) {
            const auto& descriptor = layout.junctions()[index];
            auto& geometry = groupGeometry[groupForJunction[index]];
            geometry.lengthM += descriptor.trunkLengthM;
            const auto sweptVolumeM3 =
                circularArea(descriptor.characteristicDiameterM)
                * descriptor.trunkLengthM;
            geometry.sweptVolumeM3 += sweptVolumeM3;
            // The gas solver stores the complete junction control volume in
            // volumeM3. The acoustic network already turns an authored common
            // trunk into a distributed delay line, so only the residual may be
            // lumped here; adding the full value would count that pipe twice.
            geometry.compactVolumeM3 += std::max(
                0.0, descriptor.volumeM3 - sweptVolumeM3);
            geometry.pathIndex = descriptor.pathIndex;
        }
        // A trunk belongs on whichever side of the branch carries exactly one
        // connection: the common pipe of a collector is downstream of the merge,
        // the common pipe of a tailpipe split is upstream of the splitter. With
        // more than one connection on both sides the length cannot be attributed
        // to either, so the junction stays a point.
        //
        // Group of the junction on each side of an interface, if any. A merge
        // feeding a splitter counts on both, which is how the pipe between a
        // 4-into-1 and a tailpipe split is attributed.
        const auto groupOf = [&groupForJunction](const gasdynamics::ExhaustEndpoint& endpoint)
            -> std::optional<std::size_t> {
            if (endpoint.type != gasdynamics::ExhaustEndpointType::junction)
                return std::nullopt;
            return groupForJunction[endpoint.elementIndex];
        };
        for (const auto& interface : layout.interfaces()) {
            const auto upstream = groupOf(interface.upstream);
            const auto downstream = groupOf(interface.downstream);
            if (upstream && downstream) {
                if (*upstream == *downstream) continue; // Internal to one node.
                ++groupGeometry[*upstream].outflowCount;
                ++groupGeometry[*downstream].inflowCount;
            } else if (downstream) {
                ++groupGeometry[*downstream].inflowCount;
            } else if (upstream) {
                ++groupGeometry[*upstream].outflowCount;
            }
        }
        for (const auto& port : layout.cylinderPorts())
            if (port.networkEndpoint.type == gasdynamics::ExhaustEndpointType::junction)
                ++groupGeometry[groupForJunction[port.networkEndpoint.elementIndex]]
                     .inflowCount;
        for (const auto& outlet : layout.outlets())
            if (outlet.networkEndpoint.type == gasdynamics::ExhaustEndpointType::junction)
                ++groupGeometry[groupForJunction[outlet.networkEndpoint.elementIndex]]
                     .outflowCount;

        struct GroupPlan final {
            std::size_t manyNode { 0 };
            std::size_t singleNode { 0 };
            std::size_t trunkDuctIndex { 0 };
            bool trunkOnOutflow { false };
            bool hasTrunk { false };
        };
        // Allocate a scattering slot for each side of every group. A group with
        // a trunk needs two: the branch itself, and the far end of its common
        // pipe. They are only slots here -- adjacent branches merge some of them
        // below -- so the real Junction objects are built afterwards.
        std::vector<GroupPlan> groupPlans(groupCount);
        auto slotCount = std::size_t { 0 };
        for (std::size_t group = 0; group < groupCount; ++group) {
            const auto& geometry = groupGeometry[group];
            auto& plan = groupPlans[group];
            plan.manyNode = slotCount++;
            plan.singleNode = plan.manyNode;
            const auto trunkOnOutflow =
                geometry.outflowCount == 1 && geometry.inflowCount >= 1;
            const auto trunkOnInflow = !trunkOnOutflow
                && geometry.inflowCount == 1 && geometry.outflowCount >= 1;
            if (!(geometry.lengthM > 0.0) || !(trunkOnOutflow || trunkOnInflow))
                continue;
            const auto area = std::max(1.0e-10,
                geometry.sweptVolumeM3 / geometry.lengthM);
            plan.trunkDuctIndex = ducts.size();
            Duct trunk;
            trunk.pathIndex = geometry.pathIndex;
            trunk.lengthM = geometry.lengthM;
            trunk.areaM2 = area;
            trunk.inletAreaM2 = area;
            trunk.outletAreaM2 = area;
            trunk.radiusM = std::sqrt(area / std::numbers::pi);
            ducts.push_back(std::move(trunk));
            plan.singleNode = slotCount++;
            plan.trunkOnOutflow = trunkOnOutflow;
            plan.hasTrunk = true;
        }

        // A group with a trunk presents its many side to everything flowing
        // towards the branch and its single side to the common pipe beyond it.
        const auto slotFor = [&groupPlans](std::size_t group, bool inflowToGroup) {
            const auto& plan = groupPlans[group];
            if (!plan.hasTrunk) return plan.manyNode;
            return inflowToGroup == plan.trunkOnOutflow ? plan.manyNode
                                                        : plan.singleNode;
        };
        // Two branches wired directly to each other meet at one scattering node,
        // whether that is the branch point itself or the far end of a trunk.
        std::vector<std::size_t> slotParent(slotCount);
        for (std::size_t slot = 0; slot < slotCount; ++slot) slotParent[slot] = slot;
        const auto slotRootOf = [&slotParent](std::size_t slot) {
            auto root = slot;
            while (slotParent[root] != root) root = slotParent[root];
            while (slotParent[slot] != slot) {
                const auto next = slotParent[slot];
                slotParent[slot] = root;
                slot = next;
            }
            return root;
        };
        for (const auto& interface : layout.interfaces()) {
            const auto upstream = groupOf(interface.upstream);
            const auto downstream = groupOf(interface.downstream);
            if (!upstream || !downstream || *upstream == *downstream) continue;
            const auto left = slotRootOf(slotFor(*upstream, false));
            const auto right = slotRootOf(slotFor(*downstream, true));
            if (left != right) slotParent[right] = left;
        }
        std::unordered_map<std::size_t, std::size_t> junctionForSlotRoot;
        const auto nodeForSlot = [&](std::size_t slot) {
            const auto root = slotRootOf(slot);
            const auto [found, inserted] =
                junctionForSlotRoot.emplace(root, junctions.size());
            if (inserted) junctions.emplace_back();
            return found->second;
        };
        const auto nodeFor = [&](std::size_t group, bool inflowToGroup) {
            return nodeForSlot(slotFor(group, inflowToGroup));
        };

        owners.resize(ducts.size() * 2U);
        incident.resize(owners.size());
        outgoing.resize(owners.size());

        const auto assignEndpoint = [this](std::size_t key, OwnerType type,
                                            std::size_t index) {
            if (key >= owners.size() || owners[key].type != OwnerType::none)
                return false;
            owners[key] = { type, index };
            return true;
        };
        // Consecutive sections of a taper meet at ordinary lossless
        // admittance junctions. Constant-area ducts have one section and pay
        // none of this per-sample cost.
        for (const auto& plan : ductPlans) {
            for (std::size_t section = 0;
                 section + 1U < plan.sectionCount; ++section) {
                const auto junctionIndex = junctions.size();
                junctions.emplace_back();
                const auto upstreamKey =
                    (plan.firstDuctIndex + section) * 2U + 1U;
                const auto downstreamKey =
                    (plan.firstDuctIndex + section + 1U) * 2U;
                if (!assignEndpoint(upstreamKey, OwnerType::junction,
                        junctionIndex)
                    || !assignEndpoint(downstreamKey, OwnerType::junction,
                        junctionIndex))
                    return;
                junctions[junctionIndex].ductEndpoints.push_back(upstreamKey);
                junctions[junctionIndex].ductEndpoints.push_back(downstreamKey);
            }
        }
        // Attach each trunk between the two nodes its group resolved to,
        // oriented so the wave crosses it in the direction of mean flow.
        for (std::size_t group = 0; group < groupCount; ++group) {
            const auto& plan = groupPlans[group];
            if (!plan.hasTrunk) continue;
            // Whichever side the trunk sits on, its inlet faces the same node as
            // everything flowing towards the branch from that side, and its
            // outlet faces the node beyond it. slotFor resolves both.
            const auto inletKey = plan.trunkDuctIndex * 2U;
            const auto nearNode = nodeFor(group, true);
            const auto farNode = nodeFor(group, false);
            if (!assignEndpoint(inletKey, OwnerType::junction, nearNode)
                || !assignEndpoint(inletKey + 1U, OwnerType::junction, farNode))
                return;
            junctions[nearNode].ductEndpoints.push_back(inletKey);
            junctions[farNode].ductEndpoints.push_back(inletKey + 1U);
        }
        for (const auto& interface : layout.interfaces()) {
            const auto upstreamDuct = isDuctEndpoint(interface.upstream.type);
            const auto downstreamDuct = isDuctEndpoint(interface.downstream.type);
            if (!upstreamDuct && !downstreamDuct) continue;
            std::size_t groupIndex {};
            if (!upstreamDuct) {
                groupIndex = nodeFor(
                    groupForJunction[interface.upstream.elementIndex], false);
            } else if (!downstreamDuct) {
                groupIndex = nodeFor(
                    groupForJunction[interface.downstream.elementIndex], true);
            } else {
                groupIndex = junctions.size();
                junctions.emplace_back();
            }
            if (upstreamDuct) {
                const auto key = acousticEndpointKey(interface.upstream);
                if (!assignEndpoint(key, OwnerType::junction, groupIndex)) return;
                junctions[groupIndex].ductEndpoints.push_back(key);
            }
            if (downstreamDuct) {
                const auto key = acousticEndpointKey(interface.downstream);
                if (!assignEndpoint(key, OwnerType::junction, groupIndex)) return;
                junctions[groupIndex].ductEndpoints.push_back(key);
            }
        }

        cylinderPorts.reserve(layout.cylinderPorts().size());
        for (const auto& port : layout.cylinderPorts()) {
            const auto found = std::find(cylinderIds.begin(), cylinderIds.end(), port.cylinderId);
            if (found == cylinderIds.end()) return;
            CylinderPort compiled;
            compiled.audioCylinderIndex = static_cast<std::size_t>(
                std::distance(cylinderIds.begin(), found));
            compiled.pathIndex = port.pathIndex;
            compiled.areaM2 = port.runnerConnectionAreaM2;
            const auto portIndex = cylinderPorts.size();
            if (port.networkEndpoint.type == gasdynamics::ExhaustEndpointType::junction) {
                compiled.virtualTerminal = true;
                compiled.endpointOrJunction = nodeFor(
                    groupForJunction[port.networkEndpoint.elementIndex], true);
                junctions[compiled.endpointOrJunction].cylinderTerminals.push_back(portIndex);
            } else {
                compiled.endpointOrJunction = acousticEndpointKey(
                    port.networkEndpoint);
                if (!assignEndpoint(compiled.endpointOrJunction,
                        OwnerType::cylinder, portIndex)) return;
            }
            cylinderPorts.push_back(compiled);
        }

        outlets.reserve(layout.outlets().size());
        for (const auto& outlet : layout.outlets()) {
            Outlet compiled;
            compiled.nodeId = outlet.outletNodeId;
            compiled.pathIndex = outlet.pathIndex;
            compiled.areaM2 = outlet.openingAreaM2;
            compiled.acousticPositionM = outlet.acousticPositionM;
            compiled.acousticAxis = outlet.acousticAxis;
            compiled.acousticTermination = outlet.acousticTermination;
            const auto outletIndex = outlets.size();
            compiled.jetNoise.setSeed(static_cast<std::uint32_t>(
                0x9e3779b9U ^ (outletIndex + 1U) * 0x85ebca6bU
                ^ (static_cast<std::size_t>(outlet.pathIndex) + 1U)
                    * 0xc2b2ae35U));
            if (outlet.networkEndpoint.type == gasdynamics::ExhaustEndpointType::junction) {
                compiled.virtualTerminal = true;
                compiled.endpointOrJunction = nodeFor(
                    groupForJunction[outlet.networkEndpoint.elementIndex], false);
                junctions[compiled.endpointOrJunction].outletTerminals.push_back(outletIndex);
            } else {
                compiled.endpointOrJunction = acousticEndpointKey(
                    outlet.networkEndpoint);
                if (!assignEndpoint(compiled.endpointOrJunction,
                        OwnerType::outlet, outletIndex)) return;
            }
            outlets.push_back(std::move(compiled));
        }

        sideBranchTerminals.reserve(sideBranchPlans.size());
        for (const auto& branch : sideBranchPlans) {
            std::optional<std::size_t> attachmentJunction;
            const auto hostDuct = std::find_if(
                layout.ducts().begin(), layout.ducts().end(),
                [&branch](const auto& candidate) {
                    return candidate.nodeId == branch.attachmentNodeId;
                });
            if (hostDuct != layout.ducts().end()) {
                const auto layoutIndex = static_cast<std::size_t>(
                    std::distance(layout.ducts().begin(), hostDuct));
                const gasdynamics::ExhaustEndpoint hostOutlet {
                    gasdynamics::ExhaustEndpointType::ductOutlet,
                    layoutIndex,
                    hostDuct->nodeId,
                };
                const auto hostKey = acousticEndpointKey(hostOutlet);
                if (hostKey >= owners.size()
                    || owners[hostKey].type != OwnerType::junction)
                    return;
                attachmentJunction = owners[hostKey].index;
            } else {
                const auto hostJunction = std::find_if(
                    layout.junctions().begin(), layout.junctions().end(),
                    [&branch](const auto& candidate) {
                        return candidate.nodeId == branch.attachmentNodeId;
                    });
                if (hostJunction == layout.junctions().end()) return;
                const auto layoutIndex = static_cast<std::size_t>(
                    std::distance(layout.junctions().begin(), hostJunction));
                attachmentJunction = nodeFor(
                    groupForJunction[layoutIndex], false);
            }

            auto& terminalDuct = ducts[branch.ductIndex];
            if (terminalDuct.mediumSources.empty()) {
                for (const auto key : junctions[*attachmentJunction].ductEndpoints) {
                    for (const auto source : ducts[key / 2U].mediumSources)
                        if (std::find(terminalDuct.mediumSources.begin(),
                                terminalDuct.mediumSources.end(), source)
                            == terminalDuct.mediumSources.end())
                            terminalDuct.mediumSources.push_back(source);
                }
            }
            const auto nearKey = branch.ductIndex * 2U;
            const auto farKey = nearKey + 1U;
            if (!assignEndpoint(nearKey, OwnerType::junction,
                    *attachmentJunction))
                return;
            junctions[*attachmentJunction].ductEndpoints.push_back(nearKey);
            const auto terminalIndex = sideBranchTerminals.size();
            if (!assignEndpoint(farKey, OwnerType::sideBranchTerminal,
                    terminalIndex))
                return;
            sideBranchTerminals.push_back({
                branch.ductIndex,
                branch.pathIndex,
                branch.cavityVolumeM3,
            });
        }

        // Attach each compact volume to the branch itself (the many-port side),
        // never to the far end of its common trunk. Coincident authored branch
        // groups share one scattering node, so their residual volumes add.
        for (std::size_t group = 0; group < groupCount; ++group) {
            const auto& geometry = groupGeometry[group];
            if (!(geometry.compactVolumeM3 > 0.0)) continue;
            auto& junction = junctions[nodeForSlot(groupPlans[group].manyNode)];
            junction.pathIndex = geometry.pathIndex;
            junction.compactVolumeM3 += geometry.compactVolumeM3;
        }
        reactionTargets.reserve(layout.ducts().size()
            + layout.junctions().size());
        for (std::size_t index = 0; index < layout.ducts().size(); ++index) {
            const auto& plan = ductPlans[index];
            reactionTargets.push_back({
                layout.ducts()[index].nodeId, true, plan.firstDuctIndex,
                plan.sectionCount, true });
        }
        for (std::size_t index = 0; index < layout.junctions().size(); ++index) {
            const auto group = groupForJunction[index];
            const auto& plan = groupPlans[group];
            if (plan.hasTrunk) {
                reactionTargets.push_back({
                    layout.junctions()[index].nodeId, true,
                    plan.trunkDuctIndex, 1U, false });
            } else {
                reactionTargets.push_back({
                    layout.junctions()[index].nodeId, false,
                    nodeFor(group, true), 0U, false });
            }
        }
        // A trunk has no entry of its own in the layout, because the layout
        // carries the branch as a junction. Its gas is what passes between the
        // ducts on either side of it, so read their state: the mean is a linear
        // interpolation along the chain, which is the shape of the temperature
        // gradient through a real exhaust.
        for (std::size_t group = 0; group < groupCount; ++group) {
            const auto& plan = groupPlans[group];
            if (!plan.hasTrunk) continue;
            auto& trunk = ducts[plan.trunkDuctIndex];
            for (const auto inflowSide : { true, false }) {
                for (const auto key : junctions[nodeFor(group, inflowSide)].ductEndpoints) {
                    const auto ductIndex = key / 2U;
                    if (ductIndex == plan.trunkDuctIndex
                        || ductIndex >= compiledAcousticDuctCount)
                        continue;
                    for (const auto mediumSource : ducts[ductIndex].mediumSources)
                        if (std::find(trunk.mediumSources.begin(),
                                trunk.mediumSources.end(), mediumSource)
                            == trunk.mediumSources.end())
                            trunk.mediumSources.push_back(mediumSource);
                }
            }
        }

        configured = std::all_of(owners.begin(), owners.end(), [](const auto& owner) {
            return owner.type != OwnerType::none;
        }) && !ducts.empty() && !cylinderPorts.empty() && !outlets.empty();
    }

    [[nodiscard]] float readDelayed(const std::vector<float>& line,
                                    std::size_t write,
                                    float delaySamples,
                                    float stiffnessRhoC2) const noexcept {
        const auto linearRead = [&line, write](float delay) {
            const auto delay0 = static_cast<std::size_t>(delay);
            const auto fraction = delay - static_cast<float>(delay0);
            const auto mask = line.size() - 1U;
            const auto read0 = (write + line.size() - delay0) & mask;
            const auto read1 = (write + line.size() - delay0 - 1U) & mask;
            return std::lerp(line[read0], line[read1], fraction);
        };
        return NonlinearDuctAcoustics::steepenedRead(
            linearRead, delaySamples, static_cast<float>(line.size() - 2U),
            stiffnessRhoC2);
    }

    [[nodiscard]] float portReflection(std::size_t portIndex,
                                       float incidentPressurePa,
                                       std::span<const float> sources,
                                       std::span<const CylinderBoundary> boundaries) noexcept {
        auto& port = cylinderPorts[portIndex];
        const auto audioIndex = port.audioCylinderIndex;
        const auto source = audioIndex < sources.size()
            && std::isfinite(sources[audioIndex]) ? sources[audioIndex] : 0.0F;
        if (audioIndex >= boundaries.size() || !boundaries[audioIndex].physical)
            return source + incidentPressurePa;
        const auto& boundary = boundaries[audioIndex];
        const auto impedance = static_cast<double>(
            boundary.characteristicImpedancePaSPerM3);
        const auto acousticVolumeVelocity = impedance > 0.0
            ? (static_cast<double>(incidentPressurePa)
                - port.reflectionState.previousOutput) / impedance
            : 0.0;
        const auto coefficients = ValvePortTermination::compute(
            boundary.conductanceAreaM2, boundary.meanMassFlowKgPerSecond,
            acousticVolumeVelocity, boundary.densityKgPerM3, impedance,
            sampleRateHz);
        return source + ValvePortTermination::process(
            coefficients, port.reflectionState, incidentPressurePa);
    }

    [[nodiscard]] float endpointAdmittance(std::size_t key) const noexcept {
        const auto& duct = ducts[key / 2U];
        // The characteristic admittance A/(rho c) is a property of this duct and
        // the gas in it, so a hot primary and a cool tailpipe of equal bore
        // scatter differently -- which is the area-and-temperature step that
        // makes a collector and a chamber do their work.
        const auto endpointAreaM2 = key % 2U == 0U
            ? duct.inletAreaM2 : duct.outletAreaM2;
        return static_cast<float>(endpointAreaM2
            / (static_cast<double>(duct.medium.densityKgPerM3)
                * static_cast<double>(duct.medium.soundSpeedMps)));
    }

    [[nodiscard]] float portAdmittance(std::size_t portIndex,
                                       std::span<const CylinderBoundary> boundaries) const noexcept {
        const auto& port = cylinderPorts[portIndex];
        const auto audioIndex = port.audioCylinderIndex;
        if (audioIndex < boundaries.size()) {
            const auto impedance = boundaries[audioIndex]
                .characteristicImpedancePaSPerM3;
            if (std::isfinite(impedance) && impedance > 0.0F)
                return 1.0F / impedance;
        }
        const auto& medium = media[std::min<std::size_t>(
            port.pathIndex, media.size() - 1U)];
        return static_cast<float>(port.areaM2
            / (static_cast<double>(medium.densityKgPerM3)
                * static_cast<double>(medium.soundSpeedMps)));
    }

};

AcousticExhaustNetwork::AcousticExhaustNetwork(
    const ExhaustGraph& graph, std::span<const std::uint32_t> cylinderIds)
    : impl_(std::make_unique<Impl>(graph, cylinderIds)) {}

AcousticExhaustNetwork::~AcousticExhaustNetwork() = default;

bool AcousticExhaustNetwork::prepare(double sampleRateHz,
                                     double maximumDelayScale,
                                     double observerDistanceM) {
    if (!impl_->configured || !(sampleRateHz > 1'000.0)
        || !(maximumDelayScale >= 1.0) || !(observerDistanceM > 0.0)
        || !std::isfinite(sampleRateHz) || !std::isfinite(maximumDelayScale)
        || !std::isfinite(observerDistanceM))
        return false;
    impl_->sampleRateHz = sampleRateHz;
    impl_->maximumDelayScale = maximumDelayScale;
    constexpr double minimumExhaustSoundSpeedMps = 289.8;
    for (auto& duct : impl_->ducts) {
        const auto maximumSamples = static_cast<std::size_t>(std::ceil(
            duct.lengthM / minimumExhaustSoundSpeedMps
                * sampleRateHz * maximumDelayScale)) + 4U;
        const auto length = std::max<std::size_t>(64U,
            nextPowerOfTwo(maximumSamples));
        duct.forward.assign(length, 0.0F);
        duct.reverse.assign(length, 0.0F);
        duct.mask = length - 1U;
    }
    for (auto& outlet : impl_->outlets) {
        const auto radiusM = std::sqrt(outlet.areaM2 / std::numbers::pi);
        // Radiation produces its reference pressure at one metre. Geometric
        // propagation, directivity and arrival time belong to the observer.
        if (!outlet.radiation.prepare(sampleRateHz, radiusM, 1.0)
            || !outlet.observer.prepare(sampleRateHz, radiusM,
                outlet.acousticPositionM, outlet.acousticAxis,
                outlet.acousticTermination, impl_->observerConfig)
            || !outlet.jetNoiseObserver.prepare(sampleRateHz, radiusM,
                outlet.acousticPositionM, outlet.acousticAxis,
                outlet.acousticTermination, impl_->observerConfig)
            || !outlet.jetNoise.prepare(sampleRateHz))
            return false;
        // The same quasi-steady vorticity resistance already used by the
        // intake mouth belongs at a sharp exhaust edge too. It is passive and
        // driven by the local acoustic particle velocity inside the radiation
        // model; no arbitrary exhaust gain or clipping threshold is involved.
        constexpr double unflangedVortexLoss = 4.0
            / (3.0 * std::numbers::pi);
        const auto edgeCoefficient = outlet.acousticTermination
                == AcousticTerminationType::unflanged
            ? unflangedVortexLoss : unflangedVortexLoss * 0.5;
        if (!outlet.radiation.setNonlinearLossCoefficient(edgeCoefficient))
            return false;
    }
    impl_->prepared = true;
    reset();
    // Seed delay/loss targets from the default medium so the network is ready
    // before the first block, then clear the init flag so the first *real*
    // beginBlock snaps the live medium straight to the true gas state instead of
    // sweeping up from cold air.
    beginBlock(impl_->media, 1.0);
    impl_->mediaInitialised = false;
    return true;
}

void AcousticExhaustNetwork::reset() noexcept {
    if (!impl_) return;
    for (auto& duct : impl_->ducts) {
        std::fill(duct.forward.begin(), duct.forward.end(), 0.0F);
        std::fill(duct.reverse.begin(), duct.reverse.end(), 0.0F);
        duct.write = 0;
        duct.forwardLoss.reset();
        duct.reverseLoss.reset();
        duct.forwardLinerLoss.reset();
        duct.reverseLinerLoss.reset();
    }
    for (auto& port : impl_->cylinderPorts) {
        port.incidentToJunction = 0.0F;
        port.reflectionState.reset();
    }
    for (auto& outlet : impl_->outlets) {
        outlet.incidentToJunction = 0.0F;
        outlet.radiation.reset();
        outlet.observer.reset();
        outlet.jetNoiseObserver.reset();
        outlet.jetNoise.reset();
    }
    for (auto& junction : impl_->junctions) {
        junction.complianceAdmittanceM3PerPaSecond = 0.0F;
        junction.complianceAdmittanceTargetM3PerPaSecond = 0.0F;
        junction.complianceIncidentPressurePa = 0.0F;
        junction.pendingSourcePressurePa = 0.0F;
    }
    for (auto& terminal : impl_->sideBranchTerminals) {
        terminal.complianceAdmittanceM3PerPaSecond = 0.0F;
        terminal.complianceAdmittanceTargetM3PerPaSecond = 0.0F;
        terminal.complianceIncidentPressurePa = 0.0F;
    }
    std::fill(impl_->incident.begin(), impl_->incident.end(), 0.0F);
    std::fill(impl_->outgoing.begin(), impl_->outgoing.end(), 0.0F);
    impl_->lastJetNoise.fill(StereoPressure {});
    impl_->mediaInitialised = false;
}

void AcousticExhaustNetwork::beginBlock(
    std::span<const Medium> pathMedia, double acousticTimeScale,
    std::span<const float> pathMeanMassFlowKgPerSecond,
    std::span<const Medium> ductMedia,
    std::span<const OutletBoundary> outletBoundaries) noexcept {
    if (!impl_->prepared) return;
    for (std::size_t path = 0; path < impl_->mediaTarget.size(); ++path) {
        if (path < pathMedia.size()
            && std::isfinite(pathMedia[path].densityKgPerM3)
            && pathMedia[path].densityKgPerM3 > 0.0F
            && std::isfinite(pathMedia[path].soundSpeedMps)
            && pathMedia[path].soundSpeedMps > 0.0F)
            impl_->mediaTarget[path] = pathMedia[path];
    }
    // The gas state is sampled at the block boundary but drives per-sample
    // scattering coefficients: junction admittance, duct stiffness, the
    // wall-loss pole and the outlet radiation load all read the medium. Stepping
    // it here phase-jumps every one of them once per block, which aliases into a
    // comb at the block rate (fs/blockSize) and is the metallic fizz that scales
    // with the host buffer size. Only the *targets* are set here; process()
    // ramps the live medium and its derived coefficients toward them at the
    // control rate, exactly as the delay lengths are already ramped. The first
    // block snaps so startup carries no sweep.
    const auto snap = !impl_->mediaInitialised;
    if (snap) {
        impl_->media = impl_->mediaTarget;
        impl_->mediaInitialised = true;
    }
    const auto timeScale = std::clamp(
        std::isfinite(acousticTimeScale) ? acousticTimeScale : 1.0, 0.25, 4.0);
    const auto usable = [](const Medium& candidate) {
        return std::isfinite(candidate.densityKgPerM3) && candidate.densityKgPerM3 > 0.0F
            && std::isfinite(candidate.soundSpeedMps) && candidate.soundSpeedMps > 0.0F;
    };
    for (auto& duct : impl_->ducts) {
        // Resolve this duct's own gas state, falling back to its path wherever
        // the solver did not supply one. A trunk averages the ducts it joins.
        duct.mediumTarget = impl_->mediaTarget[std::min<std::size_t>(
            duct.pathIndex, impl_->mediaTarget.size() - 1U)];
        auto densitySum = 0.0F;
        auto soundSpeedSum = 0.0F;
        auto resolved = std::size_t { 0 };
        for (const auto source : duct.mediumSources) {
            if (source >= ductMedia.size() || !usable(ductMedia[source])) continue;
            densitySum += ductMedia[source].densityKgPerM3;
            soundSpeedSum += ductMedia[source].soundSpeedMps;
            ++resolved;
        }
        if (resolved > 0) {
            const auto count = static_cast<float>(resolved);
            duct.mediumTarget = { densitySum / count, soundSpeedSum / count };
        }
        if (snap) duct.medium = duct.mediumTarget;
        const auto& medium = duct.mediumTarget;
        const auto traversalSeconds = duct.lengthM
            / static_cast<double>(medium.soundSpeedMps) / timeScale;
        const auto limit = static_cast<float>(duct.forward.size() - 2U);
        duct.delayTargetSamples = std::clamp(static_cast<float>(
            traversalSeconds * impl_->sampleRateHz), 1.0F, limit);
        if (duct.delaySamples <= 1.0F)
            duct.delaySamples = duct.delayTargetSamples;
        duct.wallLossTarget = DuctWallLoss::fit(
            traversalSeconds, duct.radiusM, medium.densityKgPerM3,
            medium.soundSpeedMps, impl_->sampleRateHz);
        duct.linerLossTarget = PorousLinerLoss::fit(
            duct.lengthM, duct.radiusM, medium.densityKgPerM3,
            medium.soundSpeedMps, duct.packingFlowResistivityPaSPerM2,
            duct.packingThicknessM, duct.perforatedOpenAreaRatio,
            impl_->sampleRateHz);
        if (snap) {
            duct.wallLoss = duct.wallLossTarget;
            duct.linerLoss = duct.linerLossTarget;
        }
    }
    std::array<double, maximumPaths> totalOutletAreaM2 {};
    for (const auto& outlet : impl_->outlets) {
        const auto path = std::min<std::size_t>(
            outlet.pathIndex, totalOutletAreaM2.size() - 1U);
        totalOutletAreaM2[path] += outlet.areaM2;
    }
    for (auto& outlet : impl_->outlets) {
        auto medium = impl_->mediaTarget[std::min<std::size_t>(
            outlet.pathIndex, impl_->mediaTarget.size() - 1U)];
        const auto boundary = std::find_if(
            outletBoundaries.begin(), outletBoundaries.end(),
            [&outlet](const OutletBoundary& candidate) noexcept {
                return candidate.nodeId == outlet.nodeId;
            });
        if (boundary != outletBoundaries.end()
            && std::isfinite(boundary->densityKgPerM3)
            && boundary->densityKgPerM3 > 0.0F
            && std::isfinite(boundary->soundSpeedMps)
            && boundary->soundSpeedMps > 0.0F) {
            medium = { boundary->densityKgPerM3,
                       boundary->soundSpeedMps };
        }
        outlet.radiationTargetDensityKgPerM3 = medium.densityKgPerM3;
        outlet.radiationTargetSoundSpeedMps = medium.soundSpeedMps;
        // A quiescent open pipe end reflects almost fully below its radiation
        // cutoff, so a purely reactive network rings for seconds. The real
        // exhaust loses that low-frequency energy because the mean outflow
        // convects it downstream out of the standing wave. The low-frequency
        // plane-wave reflection at a subsonic outflow falls to (1-M)/(1+M), so
        // the outlet Mach number sets a real, physically scaled dissipation at
        // the termination -- not a broadband gain inside the collector loop.
        auto mach = 0.0;
        auto massFlow = 0.0;
        if (boundary != outletBoundaries.end()) {
            // Negative is ambient reversion. It neither convects acoustic
            // energy out of the pipe nor creates a downstream exhaust jet.
            massFlow = std::max(0.0, static_cast<double>(
                boundary->signedMassFlowKgPerSecond));
        } else if (outlet.pathIndex < pathMeanMassFlowKgPerSecond.size()) {
            massFlow = std::abs(static_cast<double>(
                pathMeanMassFlowKgPerSecond[outlet.pathIndex]));
        }
        {
            const auto density = static_cast<double>(medium.densityKgPerM3);
            const auto soundSpeed = static_cast<double>(medium.soundSpeedMps);
            if (outlet.areaM2 > 0.0 && std::isfinite(massFlow)
                && density > 0.0 && soundSpeed > 0.0)
                mach = massFlow / (density * outlet.areaM2 * soundSpeed);
        }
        outlet.outletMachTarget = std::clamp(mach, 0.0, 0.9);
        const auto path = std::min<std::size_t>(
            outlet.pathIndex, totalOutletAreaM2.size() - 1U);
        const auto assignedFlow = boundary != outletBoundaries.end()
            ? massFlow
            : (totalOutletAreaM2[path] > 0.0
                ? massFlow * outlet.areaM2 / totalOutletAreaM2[path] : 0.0);
        outlet.jetNoise.configure(
            assignedFlow, outlet.areaM2, medium.densityKgPerM3,
            medium.soundSpeedMps, timeScale);
        if (snap) {
            outlet.radiationDensityKgPerM3 = medium.densityKgPerM3;
            outlet.radiationSoundSpeedMps = medium.soundSpeedMps;
            outlet.outletMach = outlet.outletMachTarget;
            outlet.convectiveReflection =
                (1.0 - outlet.outletMach) / (1.0 + outlet.outletMach);
            (void) outlet.radiation.setMedium(
                outlet.radiationDensityKgPerM3, outlet.radiationSoundSpeedMps);
            outlet.jetNoise.snapToTarget();
        }
    }
    for (auto& junction : impl_->junctions) {
        const auto& medium = impl_->mediaTarget[std::min<std::size_t>(
            junction.pathIndex, impl_->mediaTarget.size() - 1U)];
        const auto density = static_cast<double>(medium.densityKgPerM3);
        const auto soundSpeed = static_cast<double>(medium.soundSpeedMps);
        // Bilinear/WDF port admittance of an acoustic compliance:
        // C = V/(rho*c^2), Yc = 2*C/T. It adds one scalar state to the compact
        // junction and no delay line, mesh cell or substep.
        junction.complianceAdmittanceTargetM3PerPaSecond =
            junction.compactVolumeM3 > 0.0 && density > 0.0
                && soundSpeed > 0.0
            ? static_cast<float>(2.0 * junction.compactVolumeM3
                * impl_->sampleRateHz / (density * soundSpeed * soundSpeed))
            : 0.0F;
        if (snap) junction.complianceAdmittanceM3PerPaSecond =
            junction.complianceAdmittanceTargetM3PerPaSecond;
        junction.pendingSourcePressurePa = 0.0F;
    }
    for (auto& terminal : impl_->sideBranchTerminals) {
        const auto& medium = impl_->mediaTarget[std::min<std::size_t>(
            terminal.pathIndex, impl_->mediaTarget.size() - 1U)];
        const auto density = static_cast<double>(medium.densityKgPerM3);
        const auto soundSpeed = static_cast<double>(medium.soundSpeedMps);
        terminal.complianceAdmittanceTargetM3PerPaSecond =
            terminal.cavityVolumeM3 > 0.0 && density > 0.0
                && soundSpeed > 0.0
            ? static_cast<float>(2.0 * terminal.cavityVolumeM3
                * impl_->sampleRateHz / (density * soundSpeed * soundSpeed))
            : 0.0F;
        if (snap) terminal.complianceAdmittanceM3PerPaSecond =
            terminal.complianceAdmittanceTargetM3PerPaSecond;
    }
}

std::array<StereoPressure, AcousticExhaustNetwork::maximumPaths>
AcousticExhaustNetwork::process(
    std::span<const float> cylinderSourcePressurePa,
    std::span<const CylinderBoundary> cylinderBoundaries,
    float delayRampCoefficient) noexcept {
    std::array<StereoPressure, maximumPaths> result {};
    if (!impl_->prepared) return result;
    impl_->lastJetNoise.fill(StereoPressure {});
    const auto ramp = std::clamp(delayRampCoefficient, 0.0F, 1.0F);
    // Slew the gas medium toward the block target so every derived scattering
    // coefficient moves continuously. Junction admittance (endpointAdmittance)
    // and duct stiffness read impl_->media directly and so become continuous for
    // free; the wall-loss pole and the outlet radiation medium are slewed
    // explicitly here and below. See beginBlock.
    for (std::size_t path = 0; path < impl_->media.size(); ++path) {
        impl_->media[path].densityKgPerM3 += ramp
            * (impl_->mediaTarget[path].densityKgPerM3
                - impl_->media[path].densityKgPerM3);
        impl_->media[path].soundSpeedMps += ramp
            * (impl_->mediaTarget[path].soundSpeedMps
                - impl_->media[path].soundSpeedMps);
    }
    for (auto& outlet : impl_->outlets) {
        outlet.radiationDensityKgPerM3 += ramp
            * (outlet.radiationTargetDensityKgPerM3
                - outlet.radiationDensityKgPerM3);
        outlet.radiationSoundSpeedMps += ramp
            * (outlet.radiationTargetSoundSpeedMps
                - outlet.radiationSoundSpeedMps);
        (void) outlet.radiation.setMedium(
            outlet.radiationDensityKgPerM3, outlet.radiationSoundSpeedMps);
        outlet.outletMach += ramp * (outlet.outletMachTarget - outlet.outletMach);
        outlet.convectiveReflection =
            (1.0 - outlet.outletMach) / (1.0 + outlet.outletMach);
    }
    for (std::size_t index = 0; index < impl_->ducts.size(); ++index) {
        auto& duct = impl_->ducts[index];
        duct.delaySamples += ramp
            * (duct.delayTargetSamples - duct.delaySamples);
        duct.wallLoss.pole += ramp
            * (duct.wallLossTarget.pole - duct.wallLoss.pole);
        duct.wallLoss.zero += ramp
            * (duct.wallLossTarget.zero - duct.wallLoss.zero);
        // The shelf's DC gain is a function of both coefficients, so it has to
        // be rebuilt after the slew moves them; interpolating it independently
        // would let the duct pass DC gain while the medium is changing.
        duct.wallLoss.renormalise();
        duct.linerLoss.pole += ramp
            * (duct.linerLossTarget.pole - duct.linerLoss.pole);
        duct.linerLoss.zero += ramp
            * (duct.linerLossTarget.zero - duct.linerLoss.zero);
        duct.linerLoss.renormalise();
        duct.medium.densityKgPerM3 += ramp
            * (duct.mediumTarget.densityKgPerM3 - duct.medium.densityKgPerM3);
        duct.medium.soundSpeedMps += ramp
            * (duct.mediumTarget.soundSpeedMps - duct.medium.soundSpeedMps);
        const auto stiffness = duct.medium.densityKgPerM3
            * duct.medium.soundSpeedMps * duct.medium.soundSpeedMps;
        // The plane mode does not cease to exist at the first transverse-mode
        // cutoff. The former fourth-order low-pass deleted it there, so every
        // large chamber erased the upper spectrum instead of merely acquiring
        // additional modes. Preserve the passive plane branch across the
        // audible band; unmodelled transverse modes are now an explicit
        // limitation rather than energy destruction disguised as physics.
        impl_->incident[index * 2U] = DuctWallLoss::process(
            duct.linerLoss, duct.reverseLinerLoss,
            DuctWallLoss::process(
                duct.wallLoss, duct.reverseLoss,
                impl_->readDelayed(duct.reverse, duct.write,
                    duct.delaySamples, stiffness)));
        impl_->incident[index * 2U + 1U] = DuctWallLoss::process(
            duct.linerLoss, duct.forwardLinerLoss,
            DuctWallLoss::process(
                duct.wallLoss, duct.forwardLoss,
                impl_->readDelayed(duct.forward, duct.write,
                    duct.delaySamples, stiffness)));
    }
    std::fill(impl_->outgoing.begin(), impl_->outgoing.end(), 0.0F);

    for (auto& junction : impl_->junctions) {
        junction.complianceAdmittanceM3PerPaSecond += ramp
            * (junction.complianceAdmittanceTargetM3PerPaSecond
                - junction.complianceAdmittanceM3PerPaSecond);
        double weightedIncident = 0.0;
        double totalAdmittance = 0.0;
        for (const auto key : junction.ductEndpoints) {
            const auto admittance = impl_->endpointAdmittance(key);
            weightedIncident += admittance * impl_->incident[key];
            totalAdmittance += admittance;
        }
        for (const auto portIndex : junction.cylinderTerminals) {
            const auto admittance = impl_->portAdmittance(
                portIndex, cylinderBoundaries);
            weightedIncident += admittance
                * impl_->cylinderPorts[portIndex].incidentToJunction;
            totalAdmittance += admittance;
        }
        for (const auto outletIndex : junction.outletTerminals) {
            const auto& outlet = impl_->outlets[outletIndex];
            const auto impedance = outlet.radiation
                .characteristicImpedancePaSPerM3();
            const auto admittance = impedance > 0.0 ? 1.0 / impedance : 0.0;
            weightedIncident += admittance * outlet.incidentToJunction;
            totalAdmittance += admittance;
        }
        const auto complianceAdmittance = static_cast<double>(
            junction.complianceAdmittanceM3PerPaSecond);
        if (complianceAdmittance > 0.0) {
            weightedIncident += complianceAdmittance
                * junction.complianceIncidentPressurePa;
            totalAdmittance += complianceAdmittance;
        }
        const auto junctionPressure = (totalAdmittance > 1.0e-15
            ? static_cast<float>(2.0 * weightedIncident / totalAdmittance)
            : 0.0F) + junction.pendingSourcePressurePa;
        junction.pendingSourcePressurePa = 0.0F;
        if (complianceAdmittance > 0.0) {
            // A lossless compliance is a one-port whose reflected wave becomes
            // its incident wave on the next sample. This is the same passive
            // trapezoidal adaptor used by AcousticIntakeNetwork's plenums.
            const auto nextIncident = junctionPressure
                - junction.complianceIncidentPressurePa;
            junction.complianceIncidentPressurePa = std::isfinite(nextIncident)
                ? nextIncident : 0.0F;
        }
        for (const auto key : junction.ductEndpoints)
            impl_->outgoing[key] = junctionPressure - impl_->incident[key];
        for (const auto portIndex : junction.cylinderTerminals) {
            auto& port = impl_->cylinderPorts[portIndex];
            const auto towardBoundary = junctionPressure - port.incidentToJunction;
            port.incidentToJunction = impl_->portReflection(
                portIndex, towardBoundary, cylinderSourcePressurePa,
                cylinderBoundaries);
        }
        for (const auto outletIndex : junction.outletTerminals) {
            auto& outlet = impl_->outlets[outletIndex];
            const auto towardMouth = junctionPressure - outlet.incidentToJunction;
            const auto radiation = outlet.radiation.process(towardMouth);
            // Convective loss removes energy from the wave returning into the
            // pipe; the radiated far field is the radiation model's own output.
            outlet.incidentToJunction = static_cast<float>(
                radiation.reflectedPressurePa * outlet.convectiveReflection);
            const auto path = std::min<std::size_t>(
                outlet.pathIndex, result.size() - 1U);
            const auto observed = outlet.observer.process(
                static_cast<float>(radiation.farFieldPressurePa));
            auto observedJet = StereoPressure {};
            if (impl_->jetNoiseEnabled) {
                const auto jetPressureAtOneMetre = outlet.jetNoise.process(ramp);
                observedJet = outlet.jetNoiseObserver.process(
                    jetPressureAtOneMetre);
            }
            impl_->lastJetNoise[path].leftPa += observedJet.leftPa;
            impl_->lastJetNoise[path].rightPa += observedJet.rightPa;
            result[path].leftPa += observed.leftPa + observedJet.leftPa;
            result[path].rightPa += observed.rightPa + observedJet.rightPa;
        }
    }

    for (std::size_t index = 0; index < impl_->cylinderPorts.size(); ++index) {
        auto& port = impl_->cylinderPorts[index];
        if (port.virtualTerminal) continue;
        const auto key = port.endpointOrJunction;
        impl_->outgoing[key] = impl_->portReflection(
            index, impl_->incident[key], cylinderSourcePressurePa,
            cylinderBoundaries);
    }
    for (auto& outlet : impl_->outlets) {
        if (outlet.virtualTerminal) continue;
        const auto key = outlet.endpointOrJunction;
        const auto radiation = outlet.radiation.process(impl_->incident[key]);
        impl_->outgoing[key] = static_cast<float>(
            radiation.reflectedPressurePa * outlet.convectiveReflection);
        const auto path = std::min<std::size_t>(
            outlet.pathIndex, result.size() - 1U);
        const auto observed = outlet.observer.process(
            static_cast<float>(radiation.farFieldPressurePa));
        auto observedJet = StereoPressure {};
        if (impl_->jetNoiseEnabled) {
            const auto jetPressureAtOneMetre = outlet.jetNoise.process(ramp);
            observedJet = outlet.jetNoiseObserver.process(
                jetPressureAtOneMetre);
        }
        impl_->lastJetNoise[path].leftPa += observedJet.leftPa;
        impl_->lastJetNoise[path].rightPa += observedJet.rightPa;
        result[path].leftPa += observed.leftPa + observedJet.leftPa;
        result[path].rightPa += observed.rightPa + observedJet.rightPa;
    }

    for (auto& terminal : impl_->sideBranchTerminals) {
        terminal.complianceAdmittanceM3PerPaSecond += ramp
            * (terminal.complianceAdmittanceTargetM3PerPaSecond
                - terminal.complianceAdmittanceM3PerPaSecond);
        const auto key = terminal.ductIndex * 2U + 1U;
        const auto incidentPressure = impl_->incident[key];
        const auto complianceAdmittance = static_cast<double>(
            terminal.complianceAdmittanceM3PerPaSecond);
        if (!(complianceAdmittance > 0.0)) {
            // Rigid closed end: pressure reflection coefficient +1.
            impl_->outgoing[key] = incidentPressure;
            continue;
        }
        const auto ductAdmittance = static_cast<double>(
            impl_->endpointAdmittance(key));
        const auto totalAdmittance = ductAdmittance + complianceAdmittance;
        const auto pressure = totalAdmittance > 1.0e-15
            ? static_cast<float>(2.0
                * (ductAdmittance * incidentPressure
                    + complianceAdmittance
                        * terminal.complianceIncidentPressurePa)
                / totalAdmittance)
            : 0.0F;
        impl_->outgoing[key] = pressure - incidentPressure;
        const auto nextComplianceIncident = pressure
            - terminal.complianceIncidentPressurePa;
        terminal.complianceIncidentPressurePa =
            std::isfinite(nextComplianceIncident)
            ? nextComplianceIncident : 0.0F;
    }

    for (std::size_t index = 0; index < impl_->ducts.size(); ++index) {
        auto& duct = impl_->ducts[index];
        duct.forward[duct.write] = std::isfinite(impl_->outgoing[index * 2U])
            ? impl_->outgoing[index * 2U] : 0.0F;
        duct.reverse[duct.write] = std::isfinite(impl_->outgoing[index * 2U + 1U])
            ? impl_->outgoing[index * 2U + 1U] : 0.0F;
        duct.write = (duct.write + 1U) & duct.mask;
    }
    return result;
}

bool AcousticExhaustNetwork::injectReactionPressure(
    std::uint32_t nodeId, float axialPosition,
    float sourcePressurePa) noexcept {
    if (!impl_ || !impl_->prepared || !std::isfinite(sourcePressurePa))
        return false;
    const auto found = std::find_if(
        impl_->reactionTargets.begin(), impl_->reactionTargets.end(),
        [nodeId](const Impl::ReactionTarget& target) noexcept {
            return target.nodeId == nodeId;
        });
    if (found == impl_->reactionTargets.end()) return false;
    const auto boundedPressure = std::clamp(
        sourcePressurePa, -100'000.0F, 100'000.0F);
    if (!found->duct) {
        if (found->index >= impl_->junctions.size()) return false;
        impl_->junctions[found->index].pendingSourcePressurePa +=
            boundedPressure;
        return true;
    }
    if (found->ductCount == 0U || found->index >= impl_->ducts.size()
        || found->ductCount > impl_->ducts.size() - found->index)
        return false;
    auto ductIndex = found->index;
    auto axial = 0.5F;
    if (found->useEventAxialPosition) {
        const auto wholeAxial = std::clamp(axialPosition, 0.0F, 1.0F);
        const auto scaledAxial = wholeAxial
            * static_cast<float>(found->ductCount);
        const auto section = std::min<std::size_t>(
            static_cast<std::size_t>(scaledAxial), found->ductCount - 1U);
        ductIndex += section;
        axial = wholeAxial >= 1.0F ? 1.0F
            : scaledAxial - static_cast<float>(section);
    }
    auto& duct = impl_->ducts[ductIndex];
    const auto addHistorical = [&](std::vector<float>& line,
                                   float offsetSamples,
                                   float pressurePa) noexcept {
        if (line.empty()) return;
        const auto offset = std::max(0.0F, offsetSamples);
        const auto whole = static_cast<std::size_t>(std::floor(offset));
        const auto fraction = offset - static_cast<float>(whole);
        const auto nearer = (duct.write - whole) & duct.mask;
        const auto farther = (nearer - 1U) & duct.mask;
        line[nearer] += pressurePa * (1.0F - fraction);
        line[farther] += pressurePa * fraction;
    };
    // Half the compact heat-addition pressure travels in each direction.
    const auto halfPressure = 0.5F * boundedPressure;
    addHistorical(duct.forward, axial * duct.delaySamples, halfPressure);
    addHistorical(duct.reverse,
        (1.0F - axial) * duct.delaySamples, halfPressure);
    return true;
}

void AcousticExhaustNetwork::setOutletJetNoiseEnabled(bool enabled) noexcept {
    if (impl_) impl_->jetNoiseEnabled = enabled;
}

bool AcousticExhaustNetwork::outletJetNoiseEnabled() const noexcept {
    return impl_ && impl_->jetNoiseEnabled;
}

std::array<StereoPressure, AcousticExhaustNetwork::maximumPaths>
AcousticExhaustNetwork::lastOutletJetNoisePressure() const noexcept {
    return impl_ ? impl_->lastJetNoise
                 : std::array<StereoPressure, maximumPaths> {};
}

bool AcousticExhaustNetwork::valid() const noexcept {
    return impl_ && impl_->configured;
}

std::size_t AcousticExhaustNetwork::ductCount() const noexcept {
    return impl_ ? impl_->ducts.size() : 0U;
}

std::size_t AcousticExhaustNetwork::junctionCount() const noexcept {
    return impl_ ? impl_->junctions.size() : 0U;
}

std::size_t AcousticExhaustNetwork::outletCount() const noexcept {
    return impl_ ? impl_->outlets.size() : 0U;
}

} // namespace enginelab
