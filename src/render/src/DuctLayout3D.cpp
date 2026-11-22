#include <enginelab/render/DuctLayout3D.hpp>

#include <enginelab/exhaust/LegacyExhaustNetwork.hpp>
#include <enginelab/render/FreeSpace.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <numbers>
#include <unordered_map>

namespace enginelab::render {
namespace {
constexpr float pi = std::numbers::pi_v<float>;
constexpr Vec3 zAxis { 0.0F, 0.0F, 1.0F };
constexpr Vec3 downAxis { 0.0F, -1.0F, 0.0F };
constexpr float litre = 1.0e6F; // mm^3

[[nodiscard]] float mm(double value) noexcept { return static_cast<float>(value); }

/** Centripetal Catmull-Rom through `controls`: unlike the uniform form, it
    neither loops nor overshoots when the control points are unevenly spaced. */
[[nodiscard]] std::vector<Vec3> smoothCurve(const std::vector<Vec3>& controls, int samplesPerSpan) {
    if (controls.size() < 2U) return controls;
    std::vector<Vec3> result;
    const auto count = controls.size();
    for (std::size_t span = 0; span + 1U < count; ++span) {
        const auto p1 = controls[span];
        const auto p2 = controls[span + 1U];
        const auto p0 = span == 0 ? p1 * 2.0F - p2 : controls[span - 1U];
        const auto p3 = span + 2U < count ? controls[span + 2U] : p2 * 2.0F - p1;
        const auto knot = [](Vec3 a, Vec3 b) { return std::max(1.0e-3F, std::sqrt(length(b - a))); };
        const auto t0 = 0.0F;
        const auto t1 = t0 + knot(p0, p1);
        const auto t2 = t1 + knot(p1, p2);
        const auto t3 = t2 + knot(p2, p3);
        for (int s = 0; s < samplesPerSpan; ++s) {
            const auto t = t1 + (t2 - t1) * static_cast<float>(s) / static_cast<float>(samplesPerSpan);
            const auto a1 = p0 * ((t1 - t) / (t1 - t0)) + p1 * ((t - t0) / (t1 - t0));
            const auto a2 = p1 * ((t2 - t) / (t2 - t1)) + p2 * ((t - t1) / (t2 - t1));
            const auto a3 = p2 * ((t3 - t) / (t3 - t2)) + p3 * ((t - t2) / (t3 - t2));
            const auto b1 = a1 * ((t2 - t) / (t2 - t0)) + a2 * ((t - t0) / (t2 - t0));
            const auto b2 = a2 * ((t3 - t) / (t3 - t1)) + a3 * ((t - t1) / (t3 - t1));
            result.push_back(b1 * ((t2 - t) / (t2 - t1)) + b2 * ((t - t1) / (t2 - t1)));
        }
    }
    result.push_back(controls.back());
    return result;
}

/** Same polyline with extra points so that no segment is longer than
    `step`. Every original point is kept, so the length does not change. */
[[nodiscard]] std::vector<Vec3> resample(const std::vector<Vec3>& points, float step) {
    if (points.size() < 2U) return points;
    step = std::max(step, polylineLength(points) / 400.0F);
    std::vector<Vec3> result { points.front() };
    for (std::size_t i = 1; i < points.size(); ++i) {
        const auto span = length(points[i] - points[i - 1U]);
        if (span < 1.0e-4F) continue;
        const auto pieces = std::max(1, static_cast<int>(std::ceil(span / step)));
        for (int k = 1; k <= pieces; ++k)
            result.push_back(lerp(points[i - 1U], points[i], static_cast<float>(k) / static_cast<float>(pieces)));
    }
    if (result.size() < 2U) result.push_back(points.back());
    return result;
}

/** Leaves `a` along `dirA`, arrives at `b` along `dirB`, and bulges towards
    `bulge` until the centreline has the requested length. Too short a length
    for the span gives the straightest curve. */
[[nodiscard]] std::vector<Vec3> routeWithLength(Vec3 a, Vec3 dirA, Vec3 b, Vec3 dirB, float lengthMm, Vec3 bulge) {
    const auto span = length(b - a);
    const auto lead = std::clamp(0.15F * lengthMm, 12.0F, std::max(12.0F, 0.3F * span));
    const auto make = [&](float sag) {
        const auto middle = lerp(a, b, 0.5F) + bulge * sag;
        return smoothCurve({ a, a + dirA * lead, middle, b - dirB * lead, b }, 10);
    };
    auto curve = make(0.0F);
    if (polylineLength(curve) >= lengthMm) return curve;
    float low = 0.0F, high = std::max(lengthMm, 1.0F);
    for (int iteration = 0; iteration < 30; ++iteration) {
        const auto middle = 0.5F * (low + high);
        if (polylineLength(make(middle)) < lengthMm) low = middle;
        else high = middle;
    }
    return make(0.5F * (low + high));
}

/** Radius along a centreline, from a profile over the normalised arc length. */
template <typename Profile>
[[nodiscard]] std::vector<float> radiiAlong(const std::vector<Vec3>& points, Profile profile) {
    std::vector<float> radii(points.size());
    const auto total = std::max(1.0e-3F, polylineLength(points));
    float walked = 0.0F;
    for (std::size_t i = 0; i < points.size(); ++i) {
        if (i > 0) walked += length(points[i] - points[i - 1U]);
        radii[i] = profile(walked / total);
    }
    return radii;
}

[[nodiscard]] int segmentsFor(float diameter) noexcept { return diameter > 90.0F ? 32 : diameter > 50.0F ? 22 : 16; }

[[nodiscard]] const PortAnchor* portOf(const std::vector<PortAnchor>& ports, std::uint32_t cylinderId) noexcept {
    const auto found = std::find_if(ports.begin(), ports.end(),
        [cylinderId](const PortAnchor& port) { return port.cylinderId == cylinderId; });
    return found == ports.end() ? nullptr : &*found;
}

/** Point in the plane across `axis` (the trunk unless given), `index` of
    `count`, around `centre`. */
[[nodiscard]] Vec3 slot(Vec3 centre, std::size_t index, std::size_t count, float radius, Vec3 spread,
                        Vec3 axis = zAxis) {
    if (count <= 1U) return centre;
    if (count == 2U) return centre + spread * (index == 0 ? -radius : radius);
    const auto angle = 2.0F * pi * static_cast<float>(index) / static_cast<float>(count);
    const auto other = normalise(cross(axis, spread));
    return centre + (spread * std::cos(angle) + other * std::sin(angle)) * radius;
}

// ------------------------------------------------------------------ exhaust

struct Node final {
    const ExhaustComponentConfig* component {};
    std::vector<std::size_t> ins;
    std::vector<std::size_t> outs;
    /** How many of `ins` and `outs`, at their ends, are side branches (an
        H's balance pipe): they meet the junction across its flank, and take
        no part in its lane, its place along the trunk or its slots. */
    std::size_t sideIns {};
    std::size_t sideOuts {};
    std::vector<std::uint32_t> cylinders;
    bool routed {};
    bool stack {};
    bool firstJunction {};
    /** A first collector whose mouth faces its ports rather than back along
        the trunk: its primaries come in along `mouthAxis` (across the
        crankshaft, away from the ports) into the mouth at `mouthCentre`, and
        the pipe after it bends onto the trunk. Fixed along the trunk, the
        mouth made the primaries of ports beyond it turn back and loop to
        reach it (an I5's, owner 2026-11-14). */
    bool turned {};
    Vec3 mouthCentre;
    Vec3 mouthAxis { zAxis };
    /** Position across the trunk (x, y); z unused. For a routed pipe, the
        lane it comes from. */
    Vec3 lane;
    Vec3 spread { 0.0F, 1.0F, 0.0F };
    float zIn {};
    float zOut {};
    float portZ {};
    float diameter {};
    float outletDiameter {};
    float body {};
    float drawLength {};
    /** A straight tail pipe drawn shorter than authored (a broken view):
        by how much. */
    float folded {};
    /** Radius of the circle the inputs plug into, and of the mouth that
        holds them: wide enough for every input side by side. */
    float slots {};
    float mouth {};
    /** Slot of each input (ins, then cylinders); empty: in order. */
    std::vector<std::size_t> slotOf;
    std::vector<Vec3> curve;
    /** A turbo is bolted to its outlet: the trunk resumes at `exitLane`,
        `exitZ`, past the turbine. */
    bool turbo {};
    /** With `turbo`, its placement in the caller's list. */
    std::size_t turboSlot {};
    /** With `turbo`, bolted ahead of this node instead: a path with no
        junction, its one primary ending on the turbine at `turboInlet`, and
        this node resuming past it. */
    bool turboAtInlet {};
    Vec3 turboInlet;
    Vec3 exitLane;
    float exitZ {};
};

/** A rotor's port passage, taken out of the laid-out graph: drawn as the
    port through the housing, its first face's anchor. */
struct RotorPassage final {
    ExhaustComponentConfig component;
    std::uint32_t face {};
};

struct PathLayout final {
    const ExhaustPathConfig* path {};
    ExhaustNetworkConfig network;
    std::vector<Node> nodes;
    std::vector<std::size_t> order;
    std::vector<RotorPassage> passages;
};

struct Base final {
    Vec3 lane;
    Vec3 spread;
    float z {};
};

/** Where the junction of a group of ports sits: out from the ports, past the
    head, and no higher than the crankshaft. */
[[nodiscard]] Base baseFor(const std::vector<const PortAnchor*>& ports, float clearance) {
    Vec3 centre {}, direction {};
    for (const auto* port : ports) {
        centre += port->position;
        direction += port->direction;
    }
    centre = centre * (1.0F / static_cast<float>(std::max<std::size_t>(1U, ports.size())));
    direction = normalise(direction);
    Base base;
    const Vec3 across { direction.x, direction.y, 0.0F };
    if (length(across) < 0.3F) {
        // Ports along the crankshaft (a radial): gather on the axis behind
        // them, as a collector ring does.
        base.lane = { centre.x, centre.y, 0.0F };
        base.spread = { 1.0F, 0.0F, 0.0F };
        base.z = centre.z + (direction.z < 0.0F ? -1.0F : 1.0F) * clearance;
        return base;
    }
    const auto out = normalise(across);
    float extent = 0.0F;
    for (const auto* port : ports) {
        const auto offset = port->position - centre;
        extent = std::max(extent, offset.x * out.x + offset.y * out.y);
    }
    base.lane = { centre.x + out.x * (extent + clearance), std::min(0.0F, centre.y + out.y * (extent + clearance)), 0.0F };
    base.spread = { -out.y, out.x, 0.0F };
    base.z = centre.z;
    return base;
}

/** A rotor's three faces open in turn onto one port in its housing, and the
    simulator sums them in a passage, a merge only they feed. The passage is
    inside the housing: drawn, it was a three-slot junction out in the air
    with one face's feeder in it, and the pipe after it looped back past the
    housing (owner report 2026-10-30). The pipe now starts at the rotor's
    port, which only its first face carries; the passage is the port. */
std::vector<RotorPassage> foldRotorPassages(ExhaustNetworkConfig& network, const std::vector<PortAnchor>& ports) {
    struct Fold final {
        std::uint32_t passage;
        std::uint32_t face;
        std::uint32_t next;
    };
    std::vector<Fold> folds;
    std::vector<RotorPassage> passages;
    for (const auto& component : network.components) {
        if (component.type != ExhaustComponentType::merge) continue;
        const auto id = component.id;
        if (std::any_of(network.connections.begin(), network.connections.end(),
                [id](const auto& connection) { return connection.toComponentId == id; }))
            continue;
        const auto outs = std::count_if(network.connections.begin(), network.connections.end(),
            [id](const auto& connection) { return connection.fromComponentId == id; });
        if (outs != 1) continue;
        std::vector<std::uint32_t> anchored;
        for (const auto& mapping : network.cylinderConnections)
            if (mapping.componentId == id && portOf(ports, mapping.cylinderId) != nullptr)
                anchored.push_back(mapping.cylinderId);
        if (anchored.size() != 1U) continue;
        const auto next = std::find_if(network.connections.begin(), network.connections.end(),
            [id](const auto& connection) { return connection.fromComponentId == id; });
        folds.push_back({ id, anchored.front(), next->toComponentId });
        passages.push_back({ component, anchored.front() });
    }
    for (const auto& fold : folds) {
        std::erase_if(network.cylinderConnections, [&fold](const auto& mapping) {
            return mapping.componentId == fold.passage;
        });
        network.cylinderConnections.push_back({ fold.face, fold.next });
        std::erase_if(network.connections, [&fold](const auto& connection) {
            return connection.fromComponentId == fold.passage;
        });
        std::erase_if(network.components, [&fold](const auto& component) { return component.id == fold.passage; });
    }
    return passages;
}

void buildNodes(PathLayout& layout) {
    const auto& network = layout.network;
    std::unordered_map<std::uint32_t, std::size_t> index;
    layout.nodes.resize(network.components.size());
    for (std::size_t i = 0; i < network.components.size(); ++i) {
        index[network.components[i].id] = i;
        layout.nodes[i].component = &network.components[i];
    }
    for (const auto& connection : network.cylinderConnections)
        if (const auto found = index.find(connection.componentId); found != index.end())
            layout.nodes[found->second].cylinders.push_back(connection.cylinderId);
    // Inputs ordered by crossover port, so an X sends each bank back to its side.
    std::vector<std::pair<std::uint8_t, std::pair<std::size_t, std::size_t>>> ins, outs;
    for (const auto& connection : network.connections) {
        const auto from = index.find(connection.fromComponentId);
        const auto to = index.find(connection.toComponentId);
        if (from == index.end() || to == index.end() || from->second == to->second) continue;
        ins.push_back({ connection.toPort, { to->second, from->second } });
        outs.push_back({ connection.fromPort, { from->second, to->second } });
    }
    // A side branch (an H's balance pipe) after the through pipe.
    const auto rank = [](std::uint8_t port) { return port == exhaustSideBranchPort ? 0x100 : static_cast<int>(port); };
    const auto byPort = [rank](const auto& a, const auto& b) { return rank(a.first) < rank(b.first); };
    std::stable_sort(ins.begin(), ins.end(), byPort);
    std::stable_sort(outs.begin(), outs.end(), byPort);
    for (const auto& [port, link] : ins) {
        layout.nodes[link.first].ins.push_back(link.second);
        if (port == exhaustSideBranchPort) ++layout.nodes[link.first].sideIns;
    }
    for (const auto& [port, link] : outs) {
        layout.nodes[link.first].outs.push_back(link.second);
        if (port == exhaustSideBranchPort) ++layout.nodes[link.first].sideOuts;
    }

    // Upstream first; a malformed cycle is broken rather than followed.
    std::vector<int> state(layout.nodes.size(), 0);
    const auto visit = [&](auto&& self, std::size_t node) -> void {
        if (state[node] != 0) return;
        state[node] = 1;
        for (const auto in : layout.nodes[node].ins) self(self, in);
        state[node] = 2;
        layout.order.push_back(node);
    };
    for (std::size_t i = 0; i < layout.nodes.size(); ++i) visit(visit, i);

    for (auto& node : layout.nodes) {
        const auto& c = *node.component;
        node.diameter = std::max(5.0F, mm(c.diameterMm));
        node.outletDiameter = c.outletDiameterMm > 1.0 ? mm(c.outletDiameterMm) : node.diameter;
        const auto volume = mm(c.volumeLitres) * litre;
        auto drawLength = mm(c.lengthMm);
        if (drawLength <= 1.0F) {
            switch (c.type) {
            case ExhaustComponentType::merge:
                drawLength = std::max(1.4F * node.diameter, volume > 0.0F ? 1.2F * std::cbrt(volume) : 0.0F);
                break;
            case ExhaustComponentType::crossover: drawLength = 2.0F * node.diameter; break;
            // An outlet with no length is the open end of the pipe before it:
            // only a collar, so the opening shows.
            case ExhaustComponentType::outlet: drawLength = 0.35F * node.diameter; break;
            default: drawLength = 0.6F * node.diameter; break;
            }
        }
        node.drawLength = drawLength;
        node.body = node.diameter;
        const auto hasBody = c.type == ExhaustComponentType::merge || c.type == ExhaustComponentType::muffler
            || c.type == ExhaustComponentType::catalyst || c.type == ExhaustComponentType::resonator;
        if (hasBody && volume > 0.0F)
            node.body = std::max(node.diameter, std::sqrt(4.0F * volume / (pi * drawLength)));
        if (c.type == ExhaustComponentType::crossover) node.body = 1.25F * node.diameter;
        node.stack = !node.cylinders.empty() && node.ins.empty() && node.outs.empty();
        // The pipe onto an H's near end too: left straight on its
        // collector's lane, the far end could not mirror it (a V8's trunks
        // 432 mm apart for a 250 mm crossover).
        node.routed = !node.stack && c.type == ExhaustComponentType::pipe
            && (!node.cylinders.empty()
                || (node.outs.size() == 1U
                    && (layout.nodes[node.outs.front()].ins.size() > 1U
                        || layout.nodes[node.outs.front()].sideOuts > 0U)));
    }
}

[[nodiscard]] std::size_t indexIn(const std::vector<std::size_t>& list, std::size_t value) noexcept {
    const auto found = std::find(list.begin(), list.end(), value);
    return found == list.end() ? 0U : static_cast<std::size_t>(std::distance(list.begin(), found));
}

[[nodiscard]] std::size_t throughIns(const Node& node) noexcept { return node.ins.size() - node.sideIns; }
[[nodiscard]] std::size_t throughOuts(const Node& node) noexcept { return node.outs.size() - node.sideOuts; }

/** Lane of the `k`-th branch leaving `node`. An X (as many outputs as
    inputs) sends each output back to the side its input came from; a side
    branch leaves from the junction itself. */
[[nodiscard]] Vec3 outLane(const PathLayout& layout, const Node& node, std::size_t k) {
    const auto n = throughOuts(node);
    if (k >= n) return node.lane;
    if (n <= 1U) return node.turbo ? node.exitLane : node.lane;
    if (throughIns(node) == n) return layout.nodes[node.ins[k]].lane;
    float widest = node.body;
    for (std::size_t j = 0; j < n; ++j) widest = std::max(widest, layout.nodes[node.outs[j]].body);
    return node.lane + node.spread * ((static_cast<float>(k) - 0.5F * static_cast<float>(n - 1U)) * 1.9F * widest);
}

[[nodiscard]] Vec3 at(Vec3 lane, float z) noexcept { return { lane.x, lane.y, z }; }

/** Where along the trunk what follows a node starts. */
[[nodiscard]] float exitZ(const Node& node) noexcept { return node.turbo ? node.exitZ : node.zOut; }

[[nodiscard]] float slotRadius(const Node& node) noexcept {
    return std::max(node.slots, 0.28F * std::max(node.body, 1.3F * node.diameter));
}

[[nodiscard]] Vec3 slotPosition(const Node& node, std::size_t index) {
    if (node.turned)
        return slot(node.mouthCentre, index, throughIns(node) + node.cylinders.size(), slotRadius(node), node.spread,
                    node.mouthAxis);
    return slot(at(node.lane, node.zIn), index, throughIns(node) + node.cylinders.size(), slotRadius(node), node.spread);
}

/** The way the inputs of `node` come into it. */
[[nodiscard]] Vec3 entryAxis(const Node& node) noexcept { return node.turned ? node.mouthAxis : zAxis; }

/** Radius of the bend that takes the pipe after a turned collector from the
    collector's axis onto the trunk: wide enough not to kink it. */
[[nodiscard]] float turnRadius(const Node& pipe) noexcept { return 1.6F * std::max(pipe.diameter, pipe.body); }

/** Where a side branch meets `node`: on its flank, halfway along it, facing
    `toward`; `direction` is the way out of the junction. */
[[nodiscard]] Vec3 flank(const Node& node, Vec3 toward, Vec3& direction) {
    const auto across = Vec3 { toward.x - node.lane.x, toward.y - node.lane.y, 0.0F };
    direction = length(across) > 1.0e-3F ? normalise(across) : node.spread;
    return at(node.lane, 0.5F * (node.zIn + node.zOut)) + direction * (0.5F * node.body);
}

[[nodiscard]] Vec3 inSlot(const Node& node, std::size_t k) {
    return slotPosition(node, k < node.slotOf.size() ? node.slotOf[k] : k);
}

/** Straight run a routed pipe keeps where it leaves a port or a junction. */
[[nodiscard]] float startLead(const Node& node, bool fromPort) noexcept {
    return (fromPort ? 0.8F : 0.5F) * node.diameter;
}

/** Straight run a routed pipe keeps into its junction: one diameter, so the
    inputs of a collector arrive side by side along it. */
[[nodiscard]] float endLead(const Node& node) noexcept { return std::max(node.diameter, node.outletDiameter); }

/** Sizes each junction's mouth so that its inputs fit side by side with the
    router's clearance between them. */
void sizeSlots(PathLayout& layout, const std::vector<PortAnchor>& ports, float gap) {
    for (auto& node : layout.nodes) {
        const auto n = throughIns(node) + node.cylinders.size();
        if (n < 2U) continue;
        float entry = 0.0F;
        for (std::size_t k = 0; k < throughIns(node); ++k) {
            const auto& in = layout.nodes[node.ins[k]];
            entry = std::max(entry, 0.5F * std::max(in.diameter, in.outletDiameter));
        }
        for (const auto cylinder : node.cylinders)
            if (const auto* port = portOf(ports, cylinder)) entry = std::max(entry, 0.5F * port->diameterMm);
        const auto half = entry + 0.5F * gap;
        node.slots = n == 2U ? half : half / std::sin(pi / static_cast<float>(n));
        node.mouth = node.slots + entry + 1.0F;
    }
}

/** Where a side branch from `node` heads: what it joins, else across. */
[[nodiscard]] Vec3 sideTarget(const PathLayout& layout, const Node& node, std::size_t k) {
    const auto& branch = layout.nodes[node.outs[k]];
    return branch.outs.empty() ? node.lane + node.spread : layout.nodes[branch.outs.front()].lane;
}

[[nodiscard]] Vec3 outSlot(const PathLayout& layout, const Node& node, std::size_t k) {
    const auto n = throughOuts(node);
    if (k >= n) {
        Vec3 direction;
        return flank(node, sideTarget(layout, node, k), direction);
    }
    if (node.turbo) return at(node.exitLane, node.exitZ);
    if (n > 1U && throughIns(node) == n) {
        // An X: each branch leaves on the side it is heading to.
        const auto target = outLane(layout, node, k);
        const auto offset = Vec3 { target.x - node.lane.x, target.y - node.lane.y, 0.0F };
        return at(node.lane, node.zOut) + normalise(offset) * (length(offset) > 1.0e-3F ? 0.3F * node.body : 0.0F);
    }
    return slot(at(node.lane, node.zOut), k, n, 0.3F * node.body, node.spread);
}

/** Sizes of the turbocharger, from the configuration where it gives them. */
struct TurboSpec final {
    bool wanted {};
    float turbineWheelRadius {};
    float compressorWheelRadius {};
};

[[nodiscard]] TurboSpec turboSpec(const EngineConfig& config) {
    TurboSpec spec;
    const auto& forced = config.forcedInduction;
    spec.wanted = forced.enabled && forced.type == ForcedInductionType::turbocharger;
    // A turbine exducer is about 85 % of its wheel, a compressor inducer about
    // 70 %; without them, a wheel the size of a small turbo.
    spec.turbineWheelRadius = forced.turbineExducerDiameterMm > 1.0 ? 0.5F * mm(forced.turbineExducerDiameterMm) / 0.85F : 26.0F;
    spec.compressorWheelRadius
        = forced.compressorInducerDiameterMm > 1.0 ? 0.5F * mm(forced.compressorInducerDiameterMm) / 0.7F : 32.0F;
    return spec;
}

/** Places the turbo on the outlet of `node`, a collector already in place:
    the gas bends outwards, enters the volute tangentially, and leaves the
    exducer along the trunk, where the rest of the path resumes. */
void placeTurbo(const PathLayout& layout, Node& node, const TurboSpec& spec, TurboPlacement& turbo) {
    const auto inlet = at(node.lane, node.zOut);
    const auto rc = 0.5F * node.outletDiameter;
    const auto rt = std::max(spec.turbineWheelRadius, 0.6F * rc);
    const auto rk = std::max(spec.compressorWheelRadius, 0.6F * rc);
    turbo.turbineWheelRadius = rt;
    turbo.compressorWheelRadius = rk;
    turbo.turbineWidth = 0.9F * node.outletDiameter;
    turbo.compressorWidth = 1.1F * rk;
    turbo.compressorOuterRadius = 2.15F * rk;
    const auto bearing = rt + 12.0F;

    // Out, away from the engine; the volute below the bend if it can be. A
    // turned collector already sends its gas out along its axis.
    Vec3 out = node.turned ? node.mouthAxis : Vec3 { node.spread.y, -node.spread.x, 0.0F };
    const auto lane = Vec3 { node.lane.x, node.lane.y, 0.0F };
    // After turned collectors the spread is the trunk's axis, so no way out:
    // an X12's compressor then sat round its own collector. Away from the
    // crank instead, or down.
    if (length(out) < 0.5F) out = length(lane) > 1.0F ? normalise(lane) : Vec3 { 0.0F, -1.0F, 0.0F };
    Vec3 across = normalise(cross(zAxis, out));
    const auto score = [&](Vec3 n) {
        return (length(lane) > 1.0F ? dot(n, normalise(lane)) : 0.0F) - 0.5F * n.y;
    };
    if (score(across * -1.0F) > score(across)) across = across * -1.0F;
    const auto bend = 1.25F * node.outletDiameter;
    const auto volute = rt + 0.9F * rc + 4.0F;
    // Far enough out that the compressor, beside the collector, clears it.
    const auto collectorRadius = std::max({ 0.5F * node.body, node.mouth, rc });
    const auto need = turbo.compressorOuterRadius + collectorRadius + 10.0F;
    const auto reach = std::max(bend, std::sqrt(std::max(0.0F, need * need - volute * volute)));

    std::vector<Vec3> path;
    Vec3 entry;
    if (node.turned) {
        // Straight on, until the compressor beside the volute is past the
        // collector's end.
        path.push_back(inlet);
        entry = inlet + out * (turbo.compressorOuterRadius + 10.0F);
        path.push_back(entry);
    } else {
        constexpr int bendSteps = 10;
        for (int k = 0; k <= bendSteps; ++k) {
            const auto a = 0.5F * pi * static_cast<float>(k) / bendSteps;
            path.push_back(inlet + out * (bend * (1.0F - std::cos(a))) + zAxis * (bend * std::sin(a)));
        }
        entry = inlet + out * reach + zAxis * bend;
        if (reach > bend + 1.0F) path.push_back(entry);
    }
    const auto centre = entry + across * volute;
    // Round the wheel, narrowing as the volute feeds it.
    const auto toEntry = across * -1.0F;
    const auto inner = rt + 0.35F * rc + 3.0F;
    constexpr int scrollSteps = 36;
    std::vector<float> radii(path.size(), rc);
    for (int k = 1; k <= scrollSteps; ++k) {
        const auto t = static_cast<float>(k) / scrollSteps;
        const auto a = 1.9F * pi * t;
        const auto r = volute + (inner - volute) * t;
        path.push_back(centre + (toEntry * std::cos(a) + out * std::sin(a)) * r);
        radii.push_back(rc * (1.0F - 0.65F * t));
    }
    turbo.placed = true;
    turbo.pathId = layout.path->id;
    turbo.collectorId = node.component->id;
    turbo.scroll = std::move(path);
    turbo.scrollRadii = std::move(radii);
    turbo.axis = zAxis;
    turbo.radial = toEntry;
    turbo.turbineCentre = centre;
    turbo.compressorCentre = centre - zAxis * (0.5F * turbo.turbineWidth + bearing + 0.5F * turbo.compressorWidth);
    // The exducer, then a cone down to the pipe that follows.
    turbo.outletRadius = rc;
    turbo.outlet = centre + zAxis * (0.5F * turbo.turbineWidth + node.outletDiameter);
    node.exitLane = { centre.x, centre.y, 0.0F };
    node.exitZ = turbo.outlet.z;
}

/** Places the path's junctions and trunk down from its first junctions;
    `settle` may move each later junction, once placed, before its turbo and
    what follows it. */
void placePath(PathLayout& layout, const std::vector<PortAnchor>& ports, float clearance, const TurboSpec& spec,
               std::vector<TurboPlacement>* turbos, const std::function<void(std::size_t)>& settle) {
    auto& nodes = layout.nodes;
    for (const auto i : layout.order) {
        auto& node = nodes[i];
        if (node.firstJunction && node.turbo && turbos != nullptr && node.turboAtInlet) {
            // The turbine where the primary ends, then this node past it:
            // the turbo bolted to a collector of nothing.
            auto mount = node;
            mount.zOut = node.zIn;
            mount.outletDiameter = node.ins.empty() ? node.diameter : nodes[node.ins.front()].outletDiameter;
            mount.body = mount.outletDiameter;
            mount.mouth = 0.0F;
            placeTurbo(layout, mount, spec, (*turbos)[node.turboSlot]);
            node.turboInlet = at(node.lane, node.zIn);
            node.lane = mount.exitLane;
            node.zIn = mount.exitZ;
            node.zOut = node.zIn + node.drawLength;
            node.exitLane = node.lane;
            node.exitZ = node.zOut;
        } else if (node.firstJunction && node.turbo && turbos != nullptr) {
            placeTurbo(layout, node, spec, (*turbos)[node.turboSlot]);
        }
        if (node.stack || node.firstJunction) continue;
        if (node.routed) {
            if (node.ins.empty()) {
                std::vector<const PortAnchor*> own;
                for (const auto cylinder : node.cylinders)
                    if (const auto* port = portOf(ports, cylinder)) own.push_back(port);
                if (!own.empty()) {
                    const auto base = baseFor(own, clearance);
                    node.lane = base.lane;
                    node.spread = base.spread;
                    node.portZ = base.z;
                }
            } else {
                const auto& parent = nodes[node.ins.front()];
                node.lane = outLane(layout, parent, indexIn(parent.outs, i));
                node.spread = parent.spread;
            }
            continue;
        }
        float z = -std::numeric_limits<float>::max();
        // Pipes routed in from junctions further up: close enough for the
        // earliest, clear of the latest.
        float routedEarliest = std::numeric_limits<float>::max(), routedLatest = z;
        // Where each would sit at three quarters of its length, and the
        // furthest it reaches (audit R01/R03: a 4-2-1's two secondaries
        // leave Y pieces 100 mm apart, and the earlier rule set the merge
        // three quarters along the earlier one, so the later one coiled
        // through it).
        float routedTargets = 0.0F, routedFurthest = std::numeric_limits<float>::max();
        int routedCount = 0;
        Vec3 laneSum {};
        int lanes = 0;
        bool spreadSet = false;
        // A side branch comes in across: it neither moves the junction along
        // the trunk nor pulls it towards the pipe it comes from (an H's merge
        // sat halfway between the two sides).
        for (std::size_t k = 0; k < throughIns(node); ++k) {
            const auto in = node.ins[k];
            const auto& upstream = nodes[in];
            float candidate {};
            if (upstream.routed && upstream.ins.empty()) {
                candidate = upstream.portZ + 0.5F * upstream.drawLength;
                laneSum += upstream.lane;
            } else if (upstream.routed) {
                const auto from = exitZ(nodes[upstream.ins.front()]);
                routedEarliest = std::min(routedEarliest, from + 0.75F * upstream.drawLength);
                routedLatest = std::max(routedLatest, from + 0.25F * upstream.drawLength);
                routedTargets += from + 0.75F * upstream.drawLength;
                routedFurthest = std::min(routedFurthest, from + 0.9F * upstream.drawLength);
                ++routedCount;
                candidate = -std::numeric_limits<float>::max();
                laneSum += upstream.lane;
            } else {
                candidate = exitZ(upstream);
                laneSum += outLane(layout, upstream, indexIn(upstream.outs, i));
            }
            if (!spreadSet) {
                node.spread = upstream.spread;
                spreadSet = true;
            }
            ++lanes;
            z = std::max(z, candidate);
        }
        if (!node.cylinders.empty()) {
            std::vector<const PortAnchor*> own;
            for (const auto cylinder : node.cylinders)
                if (const auto* port = portOf(ports, cylinder)) own.push_back(port);
            if (!own.empty()) {
                const auto base = baseFor(own, clearance);
                laneSum += base.lane;
                ++lanes;
                z = std::max(z, base.z);
                if (!spreadSet) node.spread = base.spread;
            }
        }
        if (routedLatest > -std::numeric_limits<float>::max()) {
            // Their mean target, clear of the latest and within the reach of
            // the earliest.
            const auto target = routedCount > 1
                ? std::clamp(routedTargets / static_cast<float>(routedCount), routedLatest,
                             std::max(routedLatest, routedFurthest))
                : std::max(routedEarliest, routedLatest);
            z = std::max(z, target);
        }
        if (lanes == 0) z = 0.0F;
        node.lane = lanes > 0 ? laneSum * (1.0F / static_cast<float>(lanes)) : Vec3 {};
        node.zIn = z;
        node.zOut = z + node.drawLength;
        if (throughIns(node) == 1U && nodes[node.ins.front()].turned && !nodes[node.ins.front()].turbo) {
            // After a turned collector: a quarter bend from its axis onto
            // the trunk, then straight on; the bend spends its share of the
            // length.
            const auto& parent = nodes[node.ins.front()];
            const auto radius = turnRadius(node);
            const auto bent = parent.lane + parent.mouthAxis * radius;
            node.lane = { bent.x, bent.y, 0.0F };
            node.zIn = parent.zOut + radius;
            node.zOut = node.zIn + std::max(0.2F * node.drawLength, node.drawLength - 0.5F * pi * radius);
        }
        settle(i);
        if (node.turbo && !node.turboAtInlet && turbos != nullptr)
            placeTurbo(layout, node, spec, (*turbos)[node.turboSlot]);
    }
}

/** How far back along the trunk a collector fed by primaries sits: behind
    the last port when every primary still reaches it with room to bend (long
    tubes), else no further back than the shortest allows, and never ahead of
    `centre`, the middle of its ports. Primaries enter along the trunk, so one
    whose port lies behind the mouth has to turn back to reach it. Where they
    all reach behind the last port, as far back as every primary still
    reaches: the place follows their length. Kept just behind the last port, a
    4 m primary spent its length in a hairpin a metre and a half out (owner
    2026-11-18), as a rotary's coiled beside its ports. */
[[nodiscard]] float collectorZ(const std::vector<Node>& nodes, const Node& node, const std::vector<PortAnchor>& ports,
                               float centre) {
    float last = centre;
    for (const auto in : node.ins)
        for (const auto cylinder : nodes[in].cylinders)
            if (const auto* port = portOf(ports, cylinder)) last = std::max(last, port->position.z);
    const auto reaches = [&](float z) {
        for (const auto in : node.ins) {
            const auto& primary = nodes[in];
            const auto* port = primary.cylinders.empty() ? nullptr : portOf(ports, primary.cylinders.front());
            if (port == nullptr) continue;
            const auto lead = startLead(primary, true);
            const auto entry = endLead(primary);
            const auto from = port->position + port->direction * lead;
            const auto to = Vec3 { node.lane.x, node.lane.y, z - entry };
            // Straight-line reach plus a fifth for the bends.
            if (lead + entry + 1.2F * length(to - from) > primary.drawLength) return false;
        }
        return true;
    };
    const auto back = last + node.diameter;
    constexpr int steps = 24;
    if (reaches(back)) {
        float longest = 0.0F;
        for (const auto in : node.ins) longest = std::max(longest, nodes[in].drawLength);
        for (int k = 0; k < steps; ++k) {
            const auto z = back + longest * static_cast<float>(steps - k) / static_cast<float>(steps);
            if (reaches(z)) return z;
        }
    }
    for (int k = 0; k <= steps; ++k) {
        const auto z = back + (centre - back) * static_cast<float>(k) / static_cast<float>(steps);
        if (reaches(z)) return z;
    }
    return centre;
}

/** Where a routed pipe leaves from: its port, or its parent's outlet slot. */
[[nodiscard]] bool routeStart(const PathLayout& layout, std::size_t i, const std::vector<PortAnchor>& ports, Vec3& at,
                              Vec3& direction, bool& fromPort) {
    const auto& node = layout.nodes[i];
    fromPort = node.ins.empty();
    if (fromPort) {
        const auto* port = node.cylinders.empty() ? nullptr : portOf(ports, node.cylinders.front());
        if (port == nullptr) return false;
        at = port->position;
        direction = port->direction;
        return true;
    }
    const auto& parent = layout.nodes[node.ins.front()];
    const auto k = indexIn(parent.outs, i);
    if (k >= throughOuts(parent)) {
        at = flank(parent, sideTarget(layout, parent, k), direction);
        return true;
    }
    at = outSlot(layout, parent, k);
    direction = entryAxis(parent);
    return true;
}

/** Gives the routed inputs of each junction the slots that make the straight
    joins from their starts shortest in total; in a plane such joins never
    cross, so the pipes do not have to pass round one another. */
void assignSlots(PathLayout& layout, const std::vector<PortAnchor>& ports) {
    for (auto& node : layout.nodes) {
        const auto n = throughIns(node) + node.cylinders.size();
        node.slotOf.resize(n);
        for (std::size_t k = 0; k < n; ++k) node.slotOf[k] = k;
        std::vector<std::size_t> routed;
        std::vector<Vec3> from;
        for (std::size_t k = 0; k < throughIns(node); ++k) {
            const auto upstream = node.ins[k];
            if (!layout.nodes[upstream].routed) continue;
            Vec3 start, direction;
            bool fromPort = false;
            if (!routeStart(layout, upstream, ports, start, direction, fromPort)) continue;
            routed.push_back(k);
            from.push_back(start + direction * startLead(layout.nodes[upstream], fromPort));
        }
        if (routed.size() < 2U || routed.size() > 8U) continue;
        auto order = routed;
        auto best = order;
        float shortest = std::numeric_limits<float>::max();
        do {
            float total = 0.0F;
            for (std::size_t j = 0; j < routed.size(); ++j) {
                const auto& upstream = layout.nodes[node.ins[routed[j]]];
                total += length(slotPosition(node, order[j]) - entryAxis(node) * endLead(upstream) - from[j]);
            }
            if (total < shortest - 1.0e-3F) {
                shortest = total;
                best = order;
            }
        } while (std::next_permutation(order.begin(), order.end()));
        for (std::size_t j = 0; j < routed.size(); ++j) node.slotOf[routed[j]] = best[j];
    }
}

/** Gives `twin`'s inputs the mirror images of `seat`'s slots: an input from
    the image of a port takes the slot at the image of that port's. Left to
    the shortest joins, the two banks of a V8 took different slots on ties,
    and their primaries could not be drawn as images. */
void mirrorSlots(const PathLayout& seatLayout, const Node& seat, const PathLayout& twinLayout, Node& twin, float dz,
                 const std::vector<PortAnchor>& ports) {
    const auto n = throughIns(twin) + twin.cylinders.size();
    if (twin.slotOf.size() != n || seat.slotOf.size() != throughIns(seat) + seat.cylinders.size()) return;
    const auto portFed = [&](const PathLayout& layout, std::size_t in) -> const PortAnchor* {
        const auto& node = layout.nodes[in];
        return node.routed && !node.cylinders.empty() ? portOf(ports, node.cylinders.front()) : nullptr;
    };
    auto slots = twin.slotOf;
    for (std::size_t k = 0; k < throughIns(twin); ++k) {
        const auto* q = portFed(twinLayout, twin.ins[k]);
        if (q == nullptr) return;
        std::size_t match = seat.ins.size();
        for (std::size_t m = 0; m < throughIns(seat) && match == seat.ins.size(); ++m) {
            const auto* p = portFed(seatLayout, seat.ins[m]);
            if (p != nullptr && length(Vec3 { -p->position.x, p->position.y, p->position.z + dz } - q->position) < 5.0F)
                match = m;
        }
        if (match == seat.ins.size()) return;
        const auto mine = slotPosition(seat, seat.slotOf[match]);
        const Vec3 image { -mine.x, mine.y, mine.z + dz };
        std::size_t found = n;
        for (std::size_t s = 0; s < n && found == n; ++s)
            if (length(slotPosition(twin, s) - image) < 1.0F) found = s;
        if (found == n) return;
        slots[k] = found;
    }
    twin.slotOf = slots;
}

void shapeComponent(DuctPiece& piece, const Node& node, std::vector<Vec3> points) {
    const auto& c = *node.component;
    points = resample(points, 12.0F);
    const auto start = 0.5F * node.diameter, end = 0.5F * node.outletDiameter, body = 0.5F * node.body;
    std::vector<float> radii;
    bool capStart = false;
    if (c.type == ExhaustComponentType::pipe || c.type == ExhaustComponentType::outlet || node.body <= node.diameter * 1.02F) {
        if (c.type == ExhaustComponentType::merge) {
            // A collector: a wide mouth over the feeders, narrowing to its pipe.
            const auto mouth = std::max({ body, 0.68F * node.diameter, node.mouth });
            radii = radiiAlong(points, [&](float t) { return t < 0.6F ? mouth + (end - mouth) * (t / 0.6F) : end; });
            capStart = true;
        } else {
            radii = radiiAlong(points, [&](float t) { return start + (end - start) * t; });
        }
    } else {
        const auto cone = std::clamp(0.8F * (body - std::min(start, end)) / std::max(1.0F, node.drawLength), 0.04F, 0.3F);
        const auto mouth = c.type == ExhaustComponentType::merge ? std::max(body, node.mouth) : start;
        capStart = c.type == ExhaustComponentType::merge;
        radii = radiiAlong(points, [&](float t) {
            if (t < cone) return mouth + (body - mouth) * (t / cone);
            if (t > 1.0F - cone) return body + (end - body) * ((t - (1.0F - cone)) / cone);
            return body;
        });
    }
    appendTube(piece.mesh, points, radii, segmentsFor(node.body), capStart, false);
    if (c.type == ExhaustComponentType::outlet && points.size() >= 2U) {
        // A rolled lip round the open end.
        const auto tip = points.back();
        const auto axis = normalise(tip - points[points.size() - 2U]);
        appendTorus(piece.mesh, Mat4::frameAlongY(tip, axis), end, std::max(0.8F, 0.05F * node.outletDiameter),
                    segmentsFor(node.outletDiameter), 8);
        piece.opensToAtmosphere = true;
        piece.openEnd = tip;
        piece.openAxis = axis;
    }
    piece.centreline = std::move(points);
    piece.radii = std::move(radii);
}

/** A graze is not a clash: RouteCheck's own allowance for a tube of radius
    `radius`. */
[[nodiscard]] float grazeMm(float radius) noexcept { return 0.5F + 0.15F * radius; }

/** Whether the tube of radius `radius` along `points` sits in a solid beyond a
    graze, leaving out what lies within two radii of an end in `ends` (a
    pipe's end is in the head it leaves, or the box it ends on). */
[[nodiscard]] bool tubeSunk(const std::vector<Vec3>& points, float radius, const std::vector<EngineSolid>& solids,
                            bool ends = false) {
    if (points.empty()) return false;
    for (const auto& point : points) {
        if (ends && (length(point - points.front()) < 2.0F * radius || length(point - points.back()) < 2.0F * radius))
            continue;
        for (const auto& solid : solids)
            if (radius - signedDistance(solid, point) > grazeMm(radius)) return true;
    }
    return false;
}

/** Whether `body` sits in a solid by more than 2 mm (RouteCheck's measure). */
[[nodiscard]] bool bodySunk(const EngineSolid& body, const std::vector<EngineSolid>& solids) {
    for (const auto& point : solidSamples(body))
        for (const auto& solid : solids)
            if (-signedDistance(solid, point) > 2.0F) return true;
    return false;
}

/** The box round `solids`, `ports` and `tubes`. */
void boundsOf(const std::vector<EngineSolid>& solids, const std::vector<RouteTube>& tubes, Vec3& low, Vec3& high) {
    const auto add = [&](Vec3 centre, Vec3 half) {
        low = { std::min(low.x, centre.x - half.x), std::min(low.y, centre.y - half.y), std::min(low.z, centre.z - half.z) };
        high = { std::max(high.x, centre.x + half.x), std::max(high.y, centre.y + half.y),
                 std::max(high.z, centre.z + half.z) };
    };
    for (const auto& solid : solids) add(solid.centre, solidHalfExtent(solid));
    for (const auto& tube : tubes)
        for (std::size_t i = 0; i < tube.points.size(); ++i) {
            const auto r = i < tube.radii.size() ? tube.radii[i] : 0.0F;
            add(tube.points[i], { r, r, r });
        }
}

/** A guess from `a` (leaving along `dirA`) to `b` (arriving along `dirB`)
    round everything `space` holds, through the straight leads; empty when
    there is no way. */
[[nodiscard]] std::vector<Vec3> freeGuess(const FreeSpace& space, Vec3 a, Vec3 dirA, float leadA, Vec3 b, Vec3 dirB,
                                          float leadB, float maximumRunMm) {
    const auto from = a + dirA * leadA, to = b - dirB * leadB;
    const auto field = space.reach(from, dirA, maximumRunMm);
    auto way = space.path(field, from, to, dirB, maximumRunMm);
    if (way.empty()) return way;
    std::vector<Vec3> controls { a };
    for (const auto& point : way)
        if (length(point - controls.back()) > 1.0F) controls.push_back(point);
    if (length(b - controls.back()) > 1.0F) controls.push_back(b);
    return controls.size() < 3U ? controls : smoothCurve(controls, 8);
}

/** Points every `step` of arc along `points`, both ends included. */
[[nodiscard]] std::vector<Vec3> evenly(const std::vector<Vec3>& points, float step) {
    std::vector<Vec3> result;
    if (points.empty()) return result;
    result.push_back(points.front());
    float carried = 0.0F;
    for (std::size_t i = 1; i < points.size(); ++i) {
        const auto span = length(points[i] - points[i - 1U]);
        if (span < 1.0e-4F) continue;
        auto at = step - carried;
        while (at <= span) {
            result.push_back(lerp(points[i - 1U], points[i], at / span));
            at += step;
        }
        carried = span - (at - step);
    }
    if (length(result.back() - points.back()) > 0.25F * step) result.push_back(points.back());
    else result.back() = points.back();
    return result;
}

/** `corners` with each inner corner rounded by an arc of radius `radius`,
    or as wide as the straights beside it leave room for; the first
    `keepStart` and last `keepEnd` of the line stay straight. */
[[nodiscard]] std::vector<Vec3> bent(const std::vector<Vec3>& corners, float radius, float keepStart, float keepEnd) {
    if (corners.size() < 3U) return corners;
    std::vector<Vec3> result { corners.front() };
    for (std::size_t i = 1; i + 1U < corners.size(); ++i) {
        const auto in = corners[i] - corners[i - 1U], out = corners[i + 1U] - corners[i];
        const auto lengthIn = length(in), lengthOut = length(out);
        if (lengthIn < 1.0e-3F || lengthOut < 1.0e-3F) continue;
        const auto dIn = in * (1.0F / lengthIn), dOut = out * (1.0F / lengthOut);
        const auto cosine = std::clamp(dot(dIn, dOut), -1.0F, 1.0F);
        const auto turn = std::acos(cosine);
        if (turn < 1.0e-3F) {
            result.push_back(corners[i]);
            continue;
        }
        // The first and last straights end in a lead each; the others are
        // shared by the arcs at their two ends.
        const auto roomIn = i == 1U ? std::max(0.0F, lengthIn - keepStart) : 0.5F * lengthIn;
        const auto roomOut = i + 2U == corners.size() ? std::max(0.0F, lengthOut - keepEnd) : 0.5F * lengthOut;
        if (std::min(roomIn, roomOut) < 1.0e-3F) {
            result.push_back(corners[i]);
            continue;
        }
        const auto half = std::tan(0.5F * std::min(turn, pi - 1.0e-3F));
        const auto reach = std::min({ radius * half, roomIn, roomOut });
        const auto r = reach / half;
        const auto start = corners[i] - dIn * reach;
        const auto axis = normalise(cross(dIn, dOut));
        const auto centre = start + normalise(cross(axis, dIn)) * r;
        const auto spoke = start - centre;
        const auto steps = std::max(2, static_cast<int>(std::ceil(turn / 0.15F)));
        for (int s = 0; s <= steps; ++s) {
            const auto angle = turn * static_cast<float>(s) / static_cast<float>(steps);
            // Rodrigues: the spoke turned about the axis (square to it).
            const auto turned = spoke * std::cos(angle) + cross(axis, spoke) * std::sin(angle);
            result.push_back(centre + turned);
        }
    }
    result.push_back(corners.back());
    return result;
}

/** A first guess bent as a tube is, straights and arcs of `bendRadius`:
    from `a` straight along `dirA` past its lead, out along `bulge` to one
    corner (`width` 0) or across a top `width` long, and back to arrive at
    `b` along `dirB` -- as far out as `lengthMm` asks. Empty when even the
    flattest is longer. */
[[nodiscard]] std::vector<Vec3> bentGuess(Vec3 a, Vec3 dirA, float leadA, Vec3 b, Vec3 dirB, float leadB,
                                          float lengthMm, float bendRadius, Vec3 bulge, float width) {
    const auto from = a + dirA * leadA, to = b - dirB * leadB;
    auto along = to - from;
    along = along - bulge * dot(along, bulge);
    along = length(along) > 1.0F ? normalise(along) : normalise(cross(bulge, std::abs(bulge.x) < 0.9F ? Vec3 { 1.0F, 0.0F, 0.0F } : Vec3 { 0.0F, 1.0F, 0.0F }));
    const auto middle = lerp(from, to, 0.5F);
    // Straight on past a lead far enough for the bend after it: a tube
    // bender's arc of `bendRadius` reaches R tan(turn / 2) along each side.
    const auto past = [&](Vec3 at, Vec3 direction, Vec3 next) {
        const auto toward = next - at;
        const auto turn = length(toward) > 1.0e-3F ? std::acos(std::clamp(dot(direction, normalise(toward)), -1.0F, 1.0F)) : 0.0F;
        return bendRadius * std::min(4.0F, std::tan(0.5F * std::min(turn, 2.8F)));
    };
    const auto make = [&](float height) {
        std::vector<Vec3> tops;
        if (width <= 0.0F) {
            tops.push_back(middle + bulge * height);
        } else {
            tops.push_back(middle + bulge * height - along * (0.5F * width));
            tops.push_back(middle + bulge * height + along * (0.5F * width));
        }
        std::vector<Vec3> corners { a };
        corners.push_back(from + dirA * past(from, dirA, tops.front()));
        corners.insert(corners.end(), tops.begin(), tops.end());
        corners.push_back(to - dirB * past(to, dirB * -1.0F, tops.back()));
        corners.push_back(b);
        return bent(corners, bendRadius, leadA, leadB);
    };
    if (polylineLength(make(0.0F)) >= lengthMm) return {};
    float low = 0.0F, high = std::max(lengthMm, 1.0F);
    for (int iteration = 0; iteration < 24; ++iteration) {
        const auto height = 0.5F * (low + high);
        if (polylineLength(make(height)) < lengthMm) low = height;
        else high = height;
    }
    return make(0.5F * (low + high));
}

/**
 * Chooses each route's first guess among shapes of its length -- which way
 * it rises off the straight line between its ends, and how wide its top is
 * -- for the one that keeps clearest of the others, of `obstacles` and of
 * `solids`, of itself, and of bends tighter than the minimum. The relaxation
 * only polishes a guess: from guesses through one another it kept each
 * pipe's length by coiling it (owner 2026-11-18: a 12-into-1 of metre-long
 * primaries, 18 clashes and 21 kinks). A route whose guess is already clear
 * keeps it.
 */
void spreadGuesses(std::vector<RoutedPipe>& routes, const std::vector<RouteTube>& obstacles,
                   const std::vector<EngineSolid>& solids, const RouteSettings& routing) {
    if (routes.empty()) return;
    struct Sampled final {
        std::vector<Vec3> points;
        float radius {};
        Vec3 low, high;
    };
    const auto sample = [](const std::vector<Vec3>& points, float radius) {
        Sampled s;
        s.radius = radius;
        s.points = evenly(points, std::clamp(radius, 4.0F, 20.0F));
        s.low = s.high = s.points.front();
        for (const auto& p : s.points) {
            s.low = { std::min(s.low.x, p.x), std::min(s.low.y, p.y), std::min(s.low.z, p.z) };
            s.high = { std::max(s.high.x, p.x), std::max(s.high.y, p.y), std::max(s.high.z, p.z) };
        }
        return s;
    };
    const auto apart = [](const Sampled& s, Vec3 low, Vec3 high, float reach) {
        return s.low.x > high.x + reach || s.high.x < low.x - reach || s.low.y > high.y + reach || s.high.y < low.y - reach
            || s.low.z > high.z + reach || s.high.z < low.z - reach;
    };
    struct Box final {
        Vec3 low, high;
    };
    std::vector<Box> obstacleBoxes;
    for (const auto& tube : obstacles) {
        Box box { tube.points.empty() ? Vec3 {} : tube.points.front(), tube.points.empty() ? Vec3 {} : tube.points.front() };
        for (std::size_t k = 0; k < tube.points.size(); ++k) {
            const auto r = k < tube.radii.size() ? tube.radii[k] : 0.0F;
            const auto& p = tube.points[k];
            box.low = { std::min(box.low.x, p.x - r), std::min(box.low.y, p.y - r), std::min(box.low.z, p.z - r) };
            box.high = { std::max(box.high.x, p.x + r), std::max(box.high.y, p.y + r), std::max(box.high.z, p.z + r) };
        }
        obstacleBoxes.push_back(box);
    }
    const auto gap = routing.gapMm;
    // How far `s`, route `self`'s candidate, is from clear of everything.
    std::vector<Sampled> current;
    for (const auto& route : routes) current.push_back(sample(route.points, route.radius));
    const auto cost = [&](const Sampled& s, std::size_t self) {
        const auto r = s.radius;
        const auto& p = s.points;
        float total = 0.0F;
        for (std::size_t j = 0; j < current.size(); ++j) {
            if (j == self) continue;
            const auto& other = current[j];
            const auto reach = r + other.radius + gap;
            if (apart(s, other.low, other.high, reach)) continue;
            for (const auto& point : p) {
                float worst = 0.0F;
                for (const auto& q : other.points) worst = std::max(worst, reach - length(point - q));
                total += worst;
            }
        }
        for (std::size_t o = 0; o < obstacles.size(); ++o) {
            if (obstacles[o].points.size() < 2U || obstacles[o].radii.size() != obstacles[o].points.size()
                || apart(s, obstacleBoxes[o].low, obstacleBoxes[o].high, r + gap)) continue;
            for (const auto& point : p) {
                const auto contact = tubeContact(point, r + gap, obstacles[o].points, &obstacles[o].radii);
                total += std::max(0.0F, contact.depth);
            }
        }
        // A pipe's ends are in the head it leaves or the box it ends on.
        for (const auto& point : p) {
            if (length(point - p.front()) < 2.0F * r || length(point - p.back()) < 2.0F * r) continue;
            for (const auto& solid : solids) total += 4.0F * std::max(0.0F, r + gap - signedDistance(solid, point));
        }
        const auto step = p.size() >= 2U ? length(p[1] - p[0]) : 1.0F;
        const auto window = static_cast<std::size_t>(std::ceil(4.0F * r / std::max(step, 1.0e-3F)));
        for (std::size_t k = 0; k + window < p.size(); ++k)
            for (std::size_t l = k + window; l < p.size(); ++l) total += std::max(0.0F, 2.0F * r + gap - length(p[k] - p[l]));
        // Bends: the circle through points a diameter apart, as RouteCheck
        // measures it, against the minimum radius.
        const auto diameter = 2.0F * r;
        const auto half = std::max<std::size_t>(1U, static_cast<std::size_t>(std::lround(0.5F * diameter / std::max(step, 1.0e-3F))));
        const auto tightest = routing.minimumBendDiameters * diameter;
        for (std::size_t k = half; k + half < p.size(); ++k) {
            const auto ab = p[k] - p[k - half], bc = p[k + half] - p[k], ca = p[k - half] - p[k + half];
            const auto area2 = length(cross(ab, bc));
            if (area2 < 1.0e-6F) continue;
            const auto radius = length(ab) * length(bc) * length(ca) / (2.0F * area2);
            total += std::max(0.0F, tightest - radius);
        }
        return total;
    };
    std::vector<float> costs(routes.size());
    for (std::size_t i = 0; i < routes.size(); ++i) costs[i] = cost(current[i], i);
    constexpr int angles = 12;
    for (int sweep = 0; sweep < 3; ++sweep) {
        bool changed = false;
        for (std::size_t i = 0; i < routes.size(); ++i) {
            auto& route = routes[i];
            costs[i] = cost(current[i], i);
            if (costs[i] <= 0.0F || route.points.size() < 2U) continue;
            const auto a = route.points.front(), b = route.points.back();
            const auto chord = (b - route.endDirection * route.endLead) - (a + route.startDirection * route.startLead);
            const auto along = length(chord) > 1.0F ? normalise(chord) : route.startDirection;
            auto first = downAxis - along * dot(downAxis, along);
            if (length(first) < 0.1F) first = Vec3 { 1.0F, 0.0F, 0.0F } - along * along.x;
            first = normalise(first);
            const auto second = normalise(cross(along, first));
            const auto diameter = 2.0F * route.radius;
            const auto bendRadius = 1.5F * diameter;
            for (const auto width : { 0.0F, 2.0F * bendRadius, 4.0F * bendRadius }) {
                for (int k = 0; k < angles; ++k) {
                    const auto angle = 2.0F * pi * static_cast<float>(k) / static_cast<float>(angles);
                    const auto bulge = first * std::cos(angle) + second * std::sin(angle);
                    auto points = bentGuess(a, route.startDirection, route.startLead, b, route.endDirection,
                                            route.endLead, route.lengthMm, bendRadius, bulge, width);
                    if (points.size() < 2U) continue;
                    auto candidate = sample(points, route.radius);
                    const auto value = cost(candidate, i);
                    if (value < costs[i] - 1.0e-3F) {
                        costs[i] = value;
                        current[i] = std::move(candidate);
                        route.points = std::move(points);
                        changed = true;
                    }
                }
            }
        }
        if (!changed) break;
    }
}

/**
 * Bends `routes` together; then every one still through a solid of `blocks`
 * (beyond a graze, its ends left out) starts again from a guess round them,
 * drawn longer than authored where the way round needs it, and is bent again
 * with the others held -- or, where they are in its way, with them all from
 * their first guesses. The relaxation only
 * moves a guess locally: one through the crankcase stays there (owner
 * 2026-11-18, an X12's primaries through its block). Returns the routes that
 * were stuck.
 */
std::vector<std::size_t> routeRound(std::vector<RoutedPipe>& routes, const std::vector<RouteTube>& obstacles,
                                    const std::vector<EngineSolid>& blocks, const RouteSettings& routing) {
    spreadGuesses(routes, obstacles, blocks, routing);
    const auto guesses = routes;
    relaxRoutes(routes, obstacles, blocks, routing);
    std::vector<std::size_t> stuck;
    float widest = 0.0F;
    for (std::size_t r = 0; r < routes.size(); ++r)
        if (tubeSunk(routes[r].points, routes[r].radius, blocks, true)) {
            stuck.push_back(r);
            widest = std::max(widest, routes[r].radius);
        }
    if (stuck.empty()) return stuck;
    auto held = obstacles;
    for (std::size_t r = 0; r < routes.size(); ++r)
        if (std::find(stuck.begin(), stuck.end(), r) == stuck.end())
            held.push_back({ routes[r].points, std::vector<float>(routes[r].points.size(), routes[r].radius) });
    Vec3 low { std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
    Vec3 high = low * -1.0F;
    boundsOf(blocks, obstacles, low, high);
    float longest = 0.0F;
    for (const auto r : stuck) {
        longest = std::max(longest, guesses[r].lengthMm);
        for (const auto& point : guesses[r].points) boundsOf({}, { { { point }, { guesses[r].radius } } }, low, high);
    }
    const auto margin = std::max(150.0F, 0.4F * longest);
    low = low - Vec3 { margin, margin, margin };
    high = high + Vec3 { margin, margin, margin };
    const auto clearance = widest + routing.gapMm + 5.0F;
    const FreeSpace space(low, high, std::max(20.0F, widest), blocks, obstacles, clearance);
    // How far an end may be from the free room: a port's mouth is in its head.
    const auto escape = std::max(80.0F, 3.0F * clearance);
    std::vector<RoutedPipe> retry;
    for (const auto r : stuck) {
        auto route = guesses[r];
        if (route.points.size() >= 2U) {
            const auto way = freeGuess(space, route.points.front(), route.startDirection, route.startLead,
                                       route.points.back(), route.endDirection, route.endLead, escape);
            if (way.size() >= 2U) {
                route.points = way;
                route.lengthMm = std::max(route.lengthMm, 1.03F * polylineLength(way));
            }
        }
        retry.push_back(std::move(route));
    }
    relaxRoutes(retry, held, blocks, routing);
    const auto still = std::any_of(retry.begin(), retry.end(), [&](const RoutedPipe& route) {
        return tubeSunk(route.points, route.radius, blocks, true);
    });
    if (!still) {
        for (std::size_t k = 0; k < stuck.size(); ++k) routes[stuck[k]] = std::move(retry[k]);
        return stuck;
    }
    // The others in the way: all bent together again, the others from their
    // first guesses.
    auto together = guesses;
    for (std::size_t k = 0; k < stuck.size(); ++k) {
        together[stuck[k]].points = retry[k].points;
        together[stuck[k]].lengthMm = retry[k].lengthMm;
    }
    relaxRoutes(together, obstacles, blocks, routing);
    routes = std::move(together);
    return stuck;
}

/** Total turning along `points`, a direction every `step` of arc. */
[[nodiscard]] float turningOf(const std::vector<Vec3>& points, float step) {
    const auto even = evenly(points, step);
    float total = 0.0F;
    for (std::size_t i = 2; i < even.size(); ++i) {
        const auto a = even[i - 1U] - even[i - 2U], b = even[i] - even[i - 1U];
        if (length(a) < 1.0e-3F || length(b) < 1.0e-3F) continue;
        total += std::acos(std::clamp(dot(normalise(a), normalise(b)), -1.0F, 1.0F));
    }
    return total;
}

/**
 * Draws the pipes of one bank as the mirror images of the other's, across
 * the crank's plane (x = 0), the bank staggered along the crank: a pipe
 * whose two ends are the images of another's, of the same length and width,
 * becomes that one's image. The source is the side whose pipes turn the
 * least in all; an image that would sink into a solid or touch a pipe kept
 * as it was, or another obstacle, is left as routed. Bent one after the
 * other, the two banks of a V8 came out 150 mm apart from mirror images
 * (owner engines, 2026-11-18).
 */
void mirrorRoutes(std::vector<RoutedPipe>& routes, const std::vector<RouteTube>& obstacles,
                  const std::vector<EngineSolid>& blocks) {
    struct Pair final {
        std::size_t left;
        std::size_t right;
        float dz;
        /** The stagger at the far ends: an H's two ends face each other
            across, unstaggered, at the end of staggered banks' trunks. */
        float dzEnd;
    };
    std::vector<Pair> pairs;
    std::vector<bool> paired(routes.size(), false);
    for (std::size_t l = 0; l < routes.size(); ++l) {
        const auto& left = routes[l];
        if (left.points.size() < 2U || left.points.front().x > -1.0F || paired[l]) continue;
        for (std::size_t r = 0; r < routes.size(); ++r) {
            const auto& right = routes[r];
            if (r == l || paired[r] || right.points.size() < 2U || std::abs(right.lengthMm - left.lengthMm) > 1.0F
                || std::abs(right.radius - left.radius) > 0.5F)
                continue;
            const auto a = left.points.front(), b = right.points.front();
            if (std::abs(a.x + b.x) > 1.0F || std::abs(a.y - b.y) > 1.0F) continue;
            const auto dz = b.z - a.z;
            const auto end = left.points.back(), otherEnd = right.points.back();
            const auto dzEnd = otherEnd.z - end.z;
            if (std::abs(end.x + otherEnd.x) > 1.0F || std::abs(end.y - otherEnd.y) > 1.0F
                || std::abs(dzEnd - dz) > std::max(1.0F, 2.0F * std::abs(dz)))
                continue;
            pairs.push_back({ l, r, dz, dzEnd });
            paired[l] = paired[r] = true;
            break;
        }
    }
    if (pairs.empty()) return;
    float leftTurning = 0.0F, rightTurning = 0.0F;
    for (const auto& pair : pairs) {
        leftTurning += turningOf(routes[pair.left].points, 2.0F * routes[pair.left].radius);
        rightTurning += turningOf(routes[pair.right].points, 2.0F * routes[pair.right].radius);
    }
    const auto fromLeft = leftTurning <= rightTurning;
    // The image of `pair`'s source side (the left one when `left`).
    const auto imageOf = [&](const Pair& pair, bool left) {
        const auto source = left ? pair.left : pair.right;
        const auto dz = left ? pair.dz : -pair.dz;
        const auto dzEnd = left ? pair.dzEnd : -pair.dzEnd;
        const auto& points = routes[source].points;
        const auto total = std::max(1.0e-3F, polylineLength(points));
        std::vector<Vec3> image;
        float walked = 0.0F;
        for (std::size_t i = 0; i < points.size(); ++i) {
            if (i > 0) walked += length(points[i] - points[i - 1U]);
            const auto t = walked / total;
            image.push_back({ -points[i].x, points[i].y, points[i].z + dz + (dzEnd - dz) * t });
        }
        return image;
    };
    std::vector<bool> replaced(routes.size(), false);
    for (const auto& pair : pairs) replaced[fromLeft ? pair.right : pair.left] = true;
    // Whether `image`, drawn in place of `target`, is clear; `skip` leaves
    // out the routes to be replaced by images as well.
    const auto clear = [&](const std::vector<Vec3>& image, std::size_t target, float radius, bool skip) {
        if (tubeSunk(image, radius, blocks, true)) return false;
        for (const auto& point : image) {
            if (length(point - image.front()) < 2.0F * radius || length(point - image.back()) < 2.0F * radius) continue;
            for (std::size_t o = 0; o < routes.size(); ++o)
                if (!(skip && replaced[o]) && o != target
                    && tubeContact(point, radius, routes[o].points, nullptr, routes[o].radius).depth > grazeMm(radius))
                    return false;
            for (const auto& obstacle : obstacles)
                if (tubeContact(point, radius, obstacle.points, &obstacle.radii).depth > grazeMm(radius)) return false;
        }
        return true;
    };
    // Points where the circle through neighbours half a diameter apart is
    // under a diameter, as RouteCheck counts a bend too tight.
    const auto tightBends = [](const std::vector<Vec3>& points, float radius) {
        const auto even = resample(points, std::max(1.0F, radius));
        std::size_t count = 0;
        for (std::size_t k = 1; k + 1U < even.size(); ++k) {
            const auto ab = even[k] - even[k - 1U], bc = even[k + 1U] - even[k], ca = even[k - 1U] - even[k + 1U];
            const auto area2 = length(cross(ab, bc));
            if (area2 < 1.0e-6F) continue;
            if (length(ab) * length(bc) * length(ca) / (2.0F * area2) < 2.0F * radius) ++count;
        }
        return count;
    };
    std::vector<std::vector<Vec3>> images(pairs.size());
    for (std::size_t p = 0; p < pairs.size(); ++p) images[p] = imageOf(pairs[p], fromLeft);
    for (std::size_t p = 0; p < pairs.size(); ++p) {
        const auto target = fromLeft ? pairs[p].right : pairs[p].left;
        if (clear(images[p], target, routes[target].radius, true)) {
            routes[target].points = images[p];
            continue;
        }
        // The other way round: a flat-12's right trunk, the source, crossed
        // the middle, so its image met it, while the left's bowed out clear.
        const auto other = fromLeft ? pairs[p].left : pairs[p].right;
        // Never with bends tighter than the route it replaces has (a V4
        // two-stroke's then had two more).
        auto image = imageOf(pairs[p], !fromLeft);
        const auto radius = routes[other].radius;
        if (tightBends(image, radius) <= tightBends(routes[other].points, radius) && clear(image, other, radius, false))
            routes[other].points = std::move(image);
    }
}

} // namespace

namespace {

/** The room a vehicle gives its exhaust, in the frame the layout works in
    (rearwards +Z): what RouteCheck's plausibility check allows a drawn
    centreline, a little inside it. Inactive: anywhere (a radial, a caller
    that gives no engine). */
struct Corridor final {
    bool active {};
    /** Whether the engine's bounds below are known (a radial's too, for
        its tail only). */
    bool bounded {};
    /** Bounds of a centreline: across, from under the engine up to over its
        highest port, from just ahead of it; no bound rearwards. */
    Vec3 low;
    Vec3 high;
    /** The engine and its gearbox, as one box. */
    Vec3 engineLow;
    Vec3 engineHigh;
    /** Solid walls round it, for the router to keep the pipes inside. */
    std::vector<EngineSolid> walls;

    [[nodiscard]] bool holds(Vec3 p, float margin) const noexcept {
        return !active
            || (p.x >= low.x + margin && p.x <= high.x - margin && p.y >= low.y + margin && p.y <= high.y - margin
                && p.z >= low.z + margin);
    }
};

/** A straight pipe all of whose downstream is single pieces and splits to
    open ends (no merge, no routed pipe, no turbo), where that runs on past
    1.5 engine-and-gearbox lengths behind the gearbox, is drawn short enough
    for it to end there, as a broken view: a V8's 1.4 m tailpipes ran 3 m
    out of the picture, each to its own end. */
void breakTails(PathLayout& layout, const Corridor& corridor) {
    const auto plane = corridor.engineHigh.z + 1.5F * (corridor.engineHigh.z - corridor.engineLow.z);
    auto& nodes = layout.nodes;
    for (const auto i : layout.order) {
        auto& node = nodes[i];
        // An even pipe only: a two-stroke chamber's cones keep their shape.
        if (node.routed || node.stack || node.turbo || !node.cylinders.empty() || node.ins.size() != 1U
            || node.component->type != ExhaustComponentType::pipe
            || std::abs(node.outletDiameter - node.diameter) > 0.5F)
            continue;
        std::vector<std::size_t> after;
        bool tree = true;
        for (std::size_t k = 0; tree && k <= after.size(); ++k) {
            const auto& from = k == 0 ? node : nodes[after[k - 1U]];
            for (const auto out : from.outs) {
                const auto& next = nodes[out];
                tree = tree && !next.routed && !next.turbo && next.ins.size() == 1U;
                after.push_back(out);
            }
        }
        if (!tree) continue;
        float end = node.zOut;
        for (const auto k : after) end = std::max(end, nodes[k].zOut);
        // A pipe moving over to its own lane (after an X) keeps the run its
        // S-bend needs: cut to three diameters, an LS3's bent 0.9 diameters.
        const auto& parent = nodes[node.ins.front()];
        float lateral = 0.0F;
        if (!parent.routed && !parent.turned)
            lateral = length(outSlot(layout, parent, indexIn(parent.outs, i)) - at(node.lane, node.zIn));
        const auto keep = std::max(3.0F * node.diameter, 1.7F * lateral + node.diameter);
        const auto cut = std::min(end - plane, node.drawLength - keep);
        if (cut <= 1.0F) continue;
        node.drawLength -= cut;
        node.zOut -= cut;
        node.folded += cut;
        for (const auto k : after) {
            nodes[k].zIn -= cut;
            nodes[k].zOut -= cut;
        }
    }
}

/** A pipe drawn shorter than authored says so: two collars half way along
    it, a diameter apart, as a drawing marks a broken view. */
void markFold(DuctPiece& piece) {
    const auto& line = piece.centreline;
    if (piece.foldedMm <= 0.0F || line.size() < 2U || piece.radii.size() != line.size()) return;
    const auto total = polylineLength(line);
    const auto radius = *std::max_element(piece.radii.begin(), piece.radii.end());
    const auto pointAt = [&line](float arc) {
        for (std::size_t i = 1; i < line.size(); ++i) {
            const auto span = length(line[i] - line[i - 1U]);
            if (arc <= span || i + 1U == line.size()) {
                const auto t = span > 1.0e-3F ? std::clamp(arc / span, 0.0F, 1.0F) : 0.0F;
                return line[i - 1U] + (line[i] - line[i - 1U]) * t;
            }
            arc -= span;
        }
        return line.back();
    };
    const auto width = 0.3F * radius;
    if (total < 4.0F * radius + 2.0F * width) return;
    for (const auto side : { -1.0F, 1.0F }) {
        const auto middle = 0.5F * total + side * radius;
        appendTube(piece.mesh, { pointAt(middle - width), pointAt(middle + width) }, { 1.2F * radius, 1.2F * radius },
                   segmentsFor(2.0F * radius), true, true);
    }
}

/** The exhaust laid out once; the junctions of the paths in `movable` move
    off their places where the engine blocks them. With `stuck`, a path whose
    junction is blocked and a routed pipe of which starts out deep in the
    engine is added there, and the layout stops short: it is to be laid out
    again. */
std::vector<DuctPiece> layoutExhaustOnce(const EngineConfig& config, const std::vector<PortAnchor>& ports, float boreMm,
                                         const std::vector<RouteTube>& avoid, const std::vector<EngineSolid>& solids,
                                         std::vector<TurboPlacement>* turbos, const std::vector<std::uint32_t>& movable,
                                         std::vector<std::uint32_t>* stuck, const Corridor& corridor) {
    const auto clearance = std::max(60.0F, 0.8F * boreMm);
    const RouteSettings routing;
    std::vector<PathLayout> layouts(config.exhaustPaths.size());
    for (std::size_t p = 0; p < layouts.size(); ++p) {
        auto& layout = layouts[p];
        layout.path = &config.exhaustPaths[p];
        layout.network = layout.path->network ? *layout.path->network
                                               : makeEditableExhaustNetwork(*layout.path, config.cylinders);
        if (config.cycle == EngineCycle::rotary) layout.passages = foldRotorPassages(layout.network, ports);
        buildNodes(layout);
        sizeSlots(layout, ports, routing.gapMm);
    }

    // First junctions: fed only by ports, placed out from those ports.
    struct Junction final {
        std::size_t path;
        std::size_t node;
    };
    std::vector<Junction> junctions;
    for (std::size_t p = 0; p < layouts.size(); ++p) {
        auto& nodes = layouts[p].nodes;
        for (const auto i : layouts[p].order) {
            auto& node = nodes[i];
            if (node.routed || node.stack) continue;
            std::vector<const PortAnchor*> feeding;
            bool other = false;
            for (const auto cylinder : node.cylinders)
                if (const auto* port = portOf(ports, cylinder)) feeding.push_back(port);
            for (const auto in : node.ins) {
                const auto& upstream = nodes[in];
                if (upstream.routed && upstream.ins.empty()) {
                    for (const auto cylinder : upstream.cylinders)
                        if (const auto* port = portOf(ports, cylinder)) feeding.push_back(port);
                } else {
                    other = true;
                }
            }
            if (feeding.empty() || other) continue;
            const auto base = baseFor(feeding, clearance + 0.5F * node.body);
            node.firstJunction = true;
            node.lane = base.lane;
            node.spread = base.spread;
            node.zIn = base.z;
            if (node.cylinders.empty())
                node.zIn = collectorZ(nodes, node, ports, base.z);
            junctions.push_back({ p, i });
        }
    }
    // Junctions of one path that would overlap sit side by side across the
    // trunk, each level with its own ports (the two Y pieces of a 4-2-1), so
    // that the pipe leaving one does not have to pass the other; another
    // path's junction moves across instead.
    std::vector<bool> done(junctions.size(), false);
    for (std::size_t a = 0; a < junctions.size(); ++a) {
        if (done[a]) continue;
        std::vector<std::size_t> cluster { a };
        const auto& first = layouts[junctions[a].path].nodes[junctions[a].node];
        for (std::size_t b = a + 1U; b < junctions.size(); ++b) {
            if (done[b] || junctions[b].path != junctions[a].path) continue;
            const auto& other = layouts[junctions[b].path].nodes[junctions[b].node];
            if (length(other.lane - first.lane) < 1.5F * std::max(first.body, other.body)) cluster.push_back(b);
        }
        Vec3 centre {};
        float width = 0.0F;
        const auto extent = [](const Node& node) { return std::max(0.5F * node.body, node.mouth); };
        for (const auto k : cluster) {
            const auto& node = layouts[junctions[k].path].nodes[junctions[k].node];
            centre += node.lane;
            width += 2.0F * extent(node) + 10.0F;
            done[k] = true;
        }
        centre = centre * (1.0F / static_cast<float>(cluster.size()));
        auto across = -0.5F * (width - 10.0F);
        for (const auto k : cluster) {
            auto& node = layouts[junctions[k].path].nodes[junctions[k].node];
            if (cluster.size() > 1U) node.lane = centre + node.spread * (across + extent(node));
            across += 2.0F * extent(node) + 10.0F;
            node.zOut = node.zIn + node.drawLength;
        }
    }

    // Each turbocharger goes on the first junction where every primary of a
    // path has met, in the first path it serves: the simulator gives path p
    // its own unit, or unit p modulo the count.
    const auto spec = turboSpec(config);
    if (turbos != nullptr) turbos->clear();
    const auto units = std::clamp<std::size_t>(config.forcedInduction.turbochargerCount, 1U, maximumTurbochargers);
    std::array<bool, maximumTurbochargers> unitPlaced {};
    for (std::size_t p = 0; turbos != nullptr && spec.wanted && p < layouts.size(); ++p) {
        const auto authored = layouts[p].path->turbocharger;
        const auto unit = authored >= 0 ? std::min(static_cast<std::size_t>(authored), units - 1U) : p % units;
        if (unitPlaced[unit]) continue;
        auto& nodes = layouts[p].nodes;
        std::size_t total = 0;
        for (const auto& node : nodes) total += node.cylinders.size();
        std::vector<std::size_t> fed(nodes.size(), 0);
        bool found = false;
        for (const auto i : layouts[p].order) {
            auto& node = nodes[i];
            fed[i] = node.cylinders.size();
            for (const auto in : node.ins) fed[i] += fed[in];
            if (node.routed || node.stack || node.outs.size() != 1U || node.ins.size() + node.cylinders.size() < 2U
                || fed[i] < total)
                continue;
            node.turbo = true;
            node.turboSlot = turbos->size();
            TurboPlacement placement;
            placement.unit = static_cast<std::uint32_t>(unit);
            turbos->push_back(std::move(placement));
            found = true;
            break;
        }
        // No junction (a single, or a path a cylinder has to itself): the
        // simulator's turbine is where the one primary ends, so the turbo is
        // drawn there, ahead of the first component past the primary.
        for (const auto i : layouts[p].order) {
            if (found) break;
            auto& node = nodes[i];
            if (!node.firstJunction || node.routed || node.stack || !node.cylinders.empty() || node.ins.size() != 1U
                || fed[i] < total)
                continue;
            const auto& primary = nodes[node.ins.front()];
            if (!primary.routed || !primary.ins.empty() || primary.outs.size() != 1U
                || primary.cylinders.size() != total)
                continue;
            node.turbo = true;
            node.turboAtInlet = true;
            node.turboSlot = turbos->size();
            TurboPlacement placement;
            placement.unit = static_cast<std::uint32_t>(unit);
            turbos->push_back(std::move(placement));
            found = true;
        }
        if (found) unitPlaced[unit] = true;
    }
    // The straight run that carries on from a junction along its lane, to
    // the next junction: its widest part and where it ends. A two-stroke's
    // chamber is three times as wide as the header cone that starts it, and
    // a V4's two chambers on one side ran through each other along their
    // whole length (owner 2026-11-14).
    struct Run final {
        float widest {};
        float zEnd {};
    };
    const auto runFrom = [&](const Junction& junction) {
        const auto& nodes = layouts[junction.path].nodes;
        const auto* node = &nodes[junction.node];
        Run run { node->body, node->zIn };
        for (std::size_t guard = 0; guard <= nodes.size(); ++guard) {
            run.widest = std::max({ run.widest, node->body, node->diameter });
            run.zEnd += node->drawLength;
            if (node->outs.size() != 1U) break;
            const auto& next = nodes[node->outs.front()];
            if (next.routed || next.stack || next.ins.size() != 1U || next.turbo) break;
            node = &next;
        }
        return run;
    };
    // In a vehicle's corridor every collector is placed below instead.
    for (std::size_t a = 0; !corridor.active && a < junctions.size(); ++a) {
        auto& node = layouts[junctions[a].path].nodes[junctions[a].node];
        const auto own = runFrom(junctions[a]);
        float step = 1.6F * node.body;
        for (int attempt = 0; attempt < 8; ++attempt) {
            bool clash = false;
            for (std::size_t b = 0; b < a; ++b) {
                if (junctions[b].path == junctions[a].path) continue;
                const auto& other = layouts[junctions[b].path].nodes[junctions[b].node];
                const auto theirs = runFrom(junctions[b]);
                const auto apart = 0.5F * (own.widest + theirs.widest) + 10.0F;
                const auto reach = std::max(1.5F * std::max(node.body, other.body), apart);
                if (length(other.lane - node.lane) < reach && node.zIn < theirs.zEnd + reach
                    && other.zIn < own.zEnd + reach) {
                    clash = true;
                    step = std::max(step, apart);
                }
            }
            if (!clash) break;
            // Away from the crankshaft, so both sides of a V move out alike.
            const auto away = dot(node.spread, Vec3 { node.lane.x, node.lane.y, 0.0F }) < 0.0F ? -1.0F : 1.0F;
            node.lane += node.spread * (away * step);
        }
    }
    // How much of its length the slackest (or, with `slackest` false, the
    // tightest) primary of `node` spends getting to a mouth at `to` facing
    // along `axis`: straight-line reach plus a fifth for the bends, as
    // `collectorZ`. Above 1, one falls short.
    const auto spentBy = [&](const std::vector<Node>& nodes, const Node& node, Vec3 to, Vec3 axis, bool slackest) {
        float result = slackest ? std::numeric_limits<float>::max() : 0.0F;
        for (const auto in : node.ins) {
            const auto& primary = nodes[in];
            const auto* port = primary.cylinders.empty() ? nullptr : portOf(ports, primary.cylinders.front());
            if (port == nullptr) continue;
            const auto lead = startLead(primary, true);
            const auto entry = endLead(primary);
            const auto from = port->position + port->direction * lead;
            const auto need = lead + entry + 1.2F * length(to - axis * entry - from);
            const auto share = need / std::max(1.0F, primary.drawLength);
            result = slackest ? std::min(result, share) : std::max(result, share);
        }
        return result;
    };
    struct Turn final {
        std::size_t junction {};
        Vec3 base;
        Vec3 outward;
        float step {};
        int steps {};
        bool forced {};
        float trunkScore {};
    };
    std::vector<Turn> turns;
    for (std::size_t j = 0; !corridor.active && j < junctions.size(); ++j) {
        auto& nodes = layouts[junctions[j].path].nodes;
        auto& node = nodes[junctions[j].node];
        if (config.cycle == EngineCycle::rotary || node.turboAtInlet || !node.cylinders.empty()
            || throughIns(node) < 3U || node.sideIns != 0U || throughOuts(node) != 1U)
            continue;
        auto& after = nodes[node.outs.front()];
        if (!node.turbo
            && (after.routed || after.component->type != ExhaustComponentType::pipe || throughIns(after) != 1U
                || after.drawLength < 1.6F * turnRadius(after)))
            continue;
        Vec3 centroid {}, facing {};
        std::size_t count = 0;
        bool behind = false;
        float longest = 0.0F;
        for (const auto in : node.ins) {
            const auto& primary = nodes[in];
            longest = std::max(longest, primary.drawLength);
            for (const auto cylinder : primary.cylinders)
                if (const auto* port = portOf(ports, cylinder)) {
                    centroid += port->position;
                    facing += port->direction;
                    ++count;
                    behind = behind || port->position.z > node.zIn - endLead(primary);
                }
        }
        // Ports facing along the crankshaft (a radial) gather on its axis
        // behind them, as a collector ring does: turned out to the side, a
        // radial's collector sat in its own cylinders (2026-11-18).
        facing = normalise(facing);
        if (count < 3U || (!behind && std::hypot(facing.x, facing.y) < 0.3F)) continue;
        centroid = centroid * (1.0F / static_cast<float>(count));
        Turn turn;
        turn.junction = j;
        turn.base = at(node.lane, centroid.z);
        const auto across = Vec3 { turn.base.x - centroid.x, turn.base.y - centroid.y, 0.0F };
        if (length(across) < 1.0F) continue;
        turn.outward = normalise(across);
        turn.forced = behind;
        turn.trunkScore = spentBy(nodes, node, at(node.lane, node.zIn), zAxis, true);
        // Out from the ports as far as every primary still reaches it with
        // room to bend, as a 6-into-1 hangs below its engine: kept beside the
        // head, a 700 mm primary curled to spend its length (owner
        // 2026-11-18).
        constexpr int steps = 24;
        turn.step = longest / static_cast<float>(steps);
        if (spentBy(nodes, node, turn.base, turn.outward, false) <= 1.0F)
            for (int k = steps; k > 0; --k)
                if (spentBy(nodes, node, turn.base + turn.outward * (turn.step * static_cast<float>(k)), turn.outward,
                            false)
                    <= 1.0F) {
                    turn.steps = k;
                    break;
                }
        turns.push_back(turn);
    }
    // Where a turned trunk comes down onto another junction's, both draw
    // back towards their heads a step at a time, alike: the two banks of a
    // flat-6 turned towards each other and met under the crank (2026-11-18).
    const auto turnedNode = [&](const Turn& turn) -> Node& {
        return layouts[junctions[turn.junction].path].nodes[junctions[turn.junction].node];
    };
    const auto widthOf = [](const Node& node) { return std::max(node.body, node.diameter); };
    const auto trunkOf = [&](const Turn& turn, int k) {
        const auto& nodes = layouts[junctions[turn.junction].path].nodes;
        const auto& node = turnedNode(turn);
        const auto& after = nodes[node.outs.front()];
        const auto reach = static_cast<float>(k) * turn.step + node.drawLength + (node.turbo ? 0.0F : turnRadius(after));
        const auto lane = turn.base + turn.outward * reach;
        return Vec3 { lane.x, lane.y, 0.0F };
    };
    const auto trunkWidth = [&](const Turn& turn) {
        const auto& nodes = layouts[junctions[turn.junction].path].nodes;
        const auto& node = turnedNode(turn);
        const auto& after = nodes[node.outs.front()];
        return std::max(widthOf(node), std::max(after.body, after.diameter));
    };
    std::vector<int> wanted(turns.size(), -1);
    for (std::size_t t = 0; t < turns.size(); ++t) wanted[t] = turns[t].steps;
    for (int pass = 0; pass <= 24; ++pass) {
        std::vector<bool> crowded(turns.size(), false);
        for (std::size_t t = 0; t < turns.size(); ++t) {
            const auto mine = trunkOf(turns[t], wanted[t]);
            const auto width = trunkWidth(turns[t]);
            for (std::size_t j = 0; j < junctions.size() && !crowded[t]; ++j) {
                if (j == turns[t].junction) continue;
                Vec3 theirs;
                float theirWidth = 0.0F;
                const auto other = std::find_if(turns.begin(), turns.end(),
                                                [j](const Turn& turn) { return turn.junction == j; });
                if (other != turns.end()) {
                    theirs = trunkOf(*other, wanted[static_cast<std::size_t>(other - turns.begin())]);
                    theirWidth = trunkWidth(*other);
                } else {
                    const auto& node = layouts[junctions[j].path].nodes[junctions[j].node];
                    theirs = Vec3 { node.lane.x, node.lane.y, 0.0F };
                    theirWidth = widthOf(node);
                }
                crowded[t] = length(mine - theirs) < 0.5F * (width + theirWidth) + 10.0F;
            }
        }
        bool moved = false;
        for (std::size_t t = 0; t < turns.size(); ++t)
            if (crowded[t] && wanted[t] > 0) {
                --wanted[t];
                moved = true;
            } else if (crowded[t] && !turns[t].forced) {
                wanted[t] = -1;
            }
        if (!moved) break;
    }
    // A collector of three or more primaries whose mouth, facing back along
    // the trunk, has ports beyond it turns its mouth to face them: the
    // primaries then come straight across into it instead of looping back,
    // and the pipe after it bends onto the trunk -- or, on a collector that
    // carries its turbo, the gas goes straight on into the volute, whose
    // exducer already faces along the trunk. Only where that pipe is long
    // enough to bend without a kink; a rotary's collectors keep their
    // places. Where none comes from behind, it turns only if that leaves its
    // slackest primary clearly less to spend than the place back along the
    // trunk: there, the ports nearest the mouth had a metre to lose in a
    // hairpin on 1.5 m primaries (owner 2026-11-18); and only where the pipe
    // after it makes its quarter bend within its own length (a Fireblade's
    // 4-1 was drawn 11 % longer).
    for (std::size_t t = 0; t < turns.size(); ++t) {
        const auto& turn = turns[t];
        auto& nodes = layouts[junctions[turn.junction].path].nodes;
        auto& node = nodes[junctions[turn.junction].node];
        const auto& after = nodes[node.outs.front()];
        if (wanted[t] < 0) continue;
        const auto mouth = turn.base + turn.outward * (turn.step * static_cast<float>(wanted[t]));
        if (!turn.forced
            && (spentBy(nodes, node, mouth, turn.outward, true) < turn.trunkScore + 0.05F
                || (!node.turbo && 0.8F * after.drawLength < 0.5F * pi * turnRadius(after))))
            continue;
        // Even where a primary is too short to reach the turned mouth with
        // room to bend: along the trunk it had to turn back on itself as
        // well, and 25 of the 26 primaries that did were knotted or kinked
        // (owner engines, 2026-11-18); too short, it is drawn longer and
        // flagged as stretched either way.
        node.turned = true;
        node.mouthCentre = mouth;
        node.mouthAxis = turn.outward;
        node.spread = zAxis;
        const auto end = mouth + node.mouthAxis * node.drawLength;
        node.lane = { end.x, end.y, 0.0F };
        node.zIn = node.zOut = mouth.z;
    }

    // A collector whose trunk -- its body, the run along its lane, its turbo
    // -- would sit in the engine, its gearbox or an intake box moves to the
    // free space where its primaries reach it with the least of their length
    // left over (owner 2026-11-18: an X12's 12-into-1 sat in its gearbox, its
    // primaries through the block). Every other keeps the place the rules
    // above gave it.
    struct Footprint final {
        std::vector<RouteTube> tubes;
        std::vector<EngineSolid> bodies;
        /** Of `tubes`, the turbo's volute: a housing beside the head, not a
            pipe, so free of the corridor (its outlet's pipe is not). */
        std::size_t volute { std::numeric_limits<std::size_t>::max() };
    };
    // As drawn, each piece at its own width; `padded`, with the router's gap
    // (a place to move to keeps it).
    const auto footprintOf = [&](const Junction& junction, const Node& node, bool padded) {
        const auto& layout = layouts[junction.path];
        const auto& nodes = layout.nodes;
        const auto gap = padded ? routing.gapMm : 0.0F;
        Footprint footprint;
        const auto tube = [&](Vec3 a, Vec3 b, float radius) {
            auto points = resample({ a, b }, 10.0F);
            footprint.tubes.push_back({ points, std::vector<float>(points.size(), radius + gap) });
        };
        const auto inlet = node.turned ? node.mouthCentre : at(node.lane, node.zIn);
        tube(inlet, at(node.lane, node.zOut), 0.5F * std::max(node.body, node.diameter));
        auto lane = node.lane;
        auto z = node.zOut;
        if (node.turbo && !node.turboAtInlet && turbos != nullptr) {
            auto copy = node;
            TurboPlacement turbo;
            placeTurbo(layout, copy, spec, turbo);
            footprint.volute = footprint.tubes.size();
            footprint.tubes.push_back({ turbo.scroll, turbo.scrollRadii });
            for (auto& radius : footprint.tubes.back().radii) radius += gap;
            footprint.bodies = turboSolids(turbo);
            lane = copy.exitLane;
            z = copy.exitZ;
        } else if (node.turned && throughOuts(node) == 1U) {
            // The quarter bend of the pipe after it onto the trunk (placePath):
            // left out, a V12's trunks under turned collectors ran 100 mm
            // under the corridor's floor.
            const auto radius = turnRadius(nodes[node.outs.front()]);
            const auto bent = lane + node.mouthAxis * radius;
            tube(at(lane, z), at(bent, z + radius), 0.5F * std::max(node.body, node.diameter));
            lane = { bent.x, bent.y, 0.0F };
            z += radius;
        }
        // The straight run on along its lane, to where a pipe is routed away.
        for (const auto* walk = &node; walk->outs.size() == 1U;) {
            const auto& next = nodes[walk->outs.front()];
            if (next.routed || next.stack || next.ins.size() != 1U) break;
            tube(at(lane, z), at(lane, z + next.drawLength), 0.5F * std::max(next.body, next.diameter));
            z += next.drawLength;
            walk = &next;
        }
        // The straight runs its routed inputs keep into its mouth: a
        // radial's ring sat just behind its gearbox, so its primaries came
        // in through it.
        float entryRun = 0.0F, widestIn = 0.0F;
        for (std::size_t k = 0; k < throughIns(node); ++k) {
            const auto& upstream = nodes[node.ins[k]];
            if (!upstream.routed) continue;
            entryRun = std::max(entryRun, endLead(upstream));
            widestIn = std::max(widestIn, 0.5F * std::max(upstream.diameter, upstream.outletDiameter));
        }
        if (entryRun > 0.0F) {
            const auto inputs = throughIns(node) + node.cylinders.size();
            // A pipe's graze, not the wide mouth's.
            auto mouthRadius = (inputs >= 2U ? slotRadius(node) : 0.0F) + widestIn;
            mouthRadius += grazeMm(mouthRadius) - grazeMm(widestIn) + gap;
            auto points = resample({ inlet - entryAxis(node) * entryRun, inlet }, 10.0F);
            footprint.tubes.push_back({ points, std::vector<float>(points.size(), mouthRadius) });
        }
        return footprint;
    };
    const auto inEngine = [&](const Footprint& footprint) {
        for (const auto& tube : footprint.tubes)
            for (std::size_t i = 0; i < tube.points.size(); ++i)
                for (const auto& solid : solids)
                    if (tube.radii[i] - signedDistance(solid, tube.points[i]) > grazeMm(tube.radii[i])) return true;
        return std::any_of(footprint.bodies.begin(), footprint.bodies.end(),
                           [&](const EngineSolid& body) { return bodySunk(body, solids); });
    };
    std::vector<Footprint> footprints;
    for (const auto& junction : junctions)
        footprints.push_back(footprintOf(junction, layouts[junction.path].nodes[junction.node], true));
    // Clear of the solids, the intake's tubes and every other junction's trunk.
    const auto clearOf = [&](const Footprint& footprint, std::size_t self) {
        for (std::size_t t = 0; t < footprint.tubes.size(); ++t)
            for (const auto& point : footprint.tubes[t].points)
                if (t != footprint.volute && !corridor.holds(point, 5.0F)) return false;
        if (inEngine(footprint)) return false;
        const auto touches = [&](Vec3 q, float r, float allowance) {
            const auto hit = [&](const RouteTube& t) {
                return tubeContact(q, r, t.points, &t.radii).depth > allowance;
            };
            if (std::any_of(avoid.begin(), avoid.end(), hit)) return true;
            for (std::size_t o = 0; o < footprints.size(); ++o) {
                if (o == self) continue;
                if (std::any_of(footprints[o].tubes.begin(), footprints[o].tubes.end(), hit)) return true;
                for (const auto& body : footprints[o].bodies)
                    if (r - signedDistance(body, q) > allowance) return true;
            }
            return false;
        };
        for (const auto& tube : footprint.tubes)
            for (std::size_t i = 0; i < tube.points.size(); ++i)
                if (touches(tube.points[i], tube.radii[i], grazeMm(tube.radii[i]))) return false;
        for (const auto& body : footprint.bodies)
            for (const auto& point : solidSamples(body))
                if (touches(point, 0.0F, 2.0F)) return false;
        // An intake tube through a turbo's middle misses its corners: an
        // X12's lower plenum ran through a turbo hung under the engine.
        if (!footprint.bodies.empty())
            for (const auto& tube : avoid) {
                const auto radius = *std::max_element(tube.radii.begin(), tube.radii.end());
                for (const auto& point : resample(tube.points, 10.0F))
                    for (const auto& body : footprint.bodies)
                        if (radius - signedDistance(body, point) > 2.0F) return false;
            }
        return true;
    };
    // What feeds a junction through routed pipes: where each pipe starts its
    // way (past its straight lead), the way it leaves, and its length.
    struct Primary final {
        Vec3 from;
        Vec3 direction;
        float lead {};
        float entry {};
        float length {};
        float radius {};
    };
    const auto isMovable = [&](const PathLayout& layout) {
        return std::find(movable.begin(), movable.end(), layout.path->id) != movable.end();
    };
    // Moves `node` (of `self`; `index` is its footprint, or past the last) to
    // the free place its routed inputs reach with the least of their length
    // left over, else to the one that asks them for the least extra length
    // (drawn that much longer and flagged as stretched, never through the
    // engine). False where no place is free at all: it keeps its own.
    const auto relocate = [&](const Junction& self, Node& node, const std::vector<Primary>& primaries,
                              std::size_t index, Footprint& placed) {
        if (primaries.empty()) return false;
        float centreZ = 0.0F, lastZ = -std::numeric_limits<float>::max(), longest = 0.0F, widestPort = 0.0F;
        for (const auto& primary : primaries) {
            centreZ += primary.from.z;
            lastZ = std::max(lastZ, primary.from.z);
            longest = std::max(longest, primary.length);
            widestPort = std::max(widestPort, primary.radius);
        }
        centreZ /= static_cast<float>(primaries.size());
        // The room: the engine, the intake and the other trunks, out to where
        // the longest input could reach.
        Vec3 low { std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                   std::numeric_limits<float>::max() };
        Vec3 high = low * -1.0F;
        boundsOf(solids, {}, low, high);
        for (const auto& primary : primaries) boundsOf({}, { { { primary.from }, { 0.0F } } }, low, high);
        const auto envelope = std::max({ std::abs(low.x), std::abs(high.x), std::abs(low.y), std::abs(high.y) });
        const auto margin = std::max(150.0F, 0.6F * longest);
        low = low - Vec3 { margin, margin, margin };
        high = high + Vec3 { margin, margin, margin };
        std::vector<RouteTube> others = avoid;
        auto bodies = solids;
        for (std::size_t o = 0; o < footprints.size(); ++o) {
            if (o == index) continue;
            others.insert(others.end(), footprints[o].tubes.begin(), footprints[o].tubes.end());
            bodies.insert(bodies.end(), footprints[o].bodies.begin(), footprints[o].bodies.end());
        }
        const auto room = widestPort + routing.gapMm + 5.0F;
        const FreeSpace space(low, high, std::max(20.0F, 2.0F * widestPort), bodies, others, room);
        const auto escape = std::max(80.0F, 3.0F * room);
        std::vector<FreeSpace::Field> fields;
        for (const auto& primary : primaries) fields.push_back(space.reach(primary.from, primary.direction, escape));

        const auto original = node;
        const auto widest = std::max(node.body, node.diameter);
        const auto step = std::max(20.0F, 0.5F * widest);
        const std::array<float, 4> zIns { centreZ, lastZ + node.diameter, lastZ + node.diameter + 0.25F * longest,
                                          lastZ + node.diameter + 0.5F * longest };
        float best = std::numeric_limits<float>::max();
        Node chosen = original;
        Footprint chosenFootprint;
        float leastExcess = std::numeric_limits<float>::max();
        Node stretched = original;
        Footprint stretchedFootprint;
        constexpr int angles = 24;
        for (const auto zIn : zIns) {
            for (int a = -1; a < angles; ++a) {
                // a = -1: the lane the rules gave it, only further back.
                const auto angle = 2.0F * pi * static_cast<float>(a) / static_cast<float>(angles);
                const Vec3 out { std::cos(angle), std::sin(angle), 0.0F };
                for (float r = 0.0F; r <= envelope + 2.0F * widest + margin; r += step) {
                    if (a < 0 && r > 0.0F) break;
                    auto candidate = original;
                    candidate.turned = false;
                    candidate.mouthAxis = zAxis;
                    candidate.lane = a < 0 ? original.lane : out * r;
                    if (a >= 0) candidate.spread = { -out.y, out.x, 0.0F };
                    candidate.zIn = zIn;
                    candidate.zOut = zIn + candidate.drawLength;
                    auto footprint = footprintOf(self, candidate, true);
                    if (!clearOf(footprint, index)) continue;
                    // The first clear place out along this way: every input
                    // must reach it with room to bend.
                    float slack = 0.0F, excess = 0.0F;
                    bool reached = true;
                    for (std::size_t k = 0; k < primaries.size(); ++k) {
                        const auto& primary = primaries[k];
                        const auto entry = at(candidate.lane, zIn) - zAxis * primary.entry;
                        const auto need = primary.lead + primary.entry
                            + space.distance(fields[k], entry, zAxis, escape);
                        reached = reached && need <= 0.97F * primary.length;
                        slack += primary.length - need;
                        excess += std::max(0.0F, need - 0.97F * primary.length);
                    }
                    if (reached && slack < best) {
                        best = slack;
                        chosen = candidate;
                        chosenFootprint = footprint;
                    }
                    if (!reached && std::isfinite(excess) && excess < leastExcess) {
                        leastExcess = excess;
                        stretched = candidate;
                        stretchedFootprint = std::move(footprint);
                    }
                    break;
                }
            }
        }
        const auto inReach = best < std::numeric_limits<float>::max();
        if (!inReach && leastExcess == std::numeric_limits<float>::max()) return false;
        node = inReach ? chosen : stretched;
        placed = std::move(inReach ? chosenFootprint : stretchedFootprint);
        return true;
    };
    // In a vehicle, each collector its primaries reach goes down and back:
    // into the corridor, on its own side of the crank's plane, where its
    // primaries spend the most of their length getting to it -- back along
    // the gearbox, no further than its end, and low -- clear of the engine,
    // the intake and every collector already placed. Out from the head as
    // far as its primaries reached, a truck's collectors and turbos hung a
    // metre beside the block and a 1.5 m primary curled to spend the rest
    // (owner 2026-11-18). What it cannot spend there is drawn folded.
    const auto primariesOf = [&](const PathLayout& layout, const Node& node) {
        std::vector<Primary> primaries;
        for (const auto in : node.ins) {
            const auto& primary = layout.nodes[in];
            const auto* port = primary.cylinders.empty() || !primary.routed ? nullptr
                                                                             : portOf(ports, primary.cylinders.front());
            if (port == nullptr) continue;
            const auto lead = startLead(primary, true);
            primaries.push_back({ port->position + port->direction * lead, port->direction, lead, endLead(primary),
                                  primary.drawLength, 0.5F * std::max(primary.diameter, primary.outletDiameter) });
        }
        return primaries;
    };
    // A collector to place, with what feeds it; `dz` is how far its twin's
    // ports sit along the crank from the mirror images of its own.
    // A sibling (`mirror` false) sits at one of `offsets` from it instead:
    // the two 2-into-1 pieces of a 4-2-1, side by side, one behind or one
    // over the other, ahead of their merge.
    struct Seat final {
        std::size_t index {};
        std::vector<Primary> primaries;
        float dz {};
        bool mirror { true };
        std::vector<Vec3> offsets;
    };
    const auto nodeOf = [&](std::size_t j) -> Node& { return layouts[junctions[j].path].nodes[junctions[j].node]; };
    // Whether every primary reaches a mouth at `mouth` facing along `axis`,
    // the length they would have left (`slack`) and what they lack
    // (`excess`): the straight reach and a fifth for the bends, and a port
    // behind the mouth's entry goes past it and comes back round a bend.
    const auto reach = [](const std::vector<Primary>& primaries, Vec3 mouth, Vec3 axis, float& slack, float& excess) {
        bool reached = true;
        for (const auto& primary : primaries) {
            const auto entry = mouth - axis * primary.entry;
            const auto behind = std::max(0.0F, dot(primary.from - entry, axis));
            const auto gap = length(entry - primary.from);
            auto need = primary.lead + primary.entry + 1.2F * gap;
            // The way back is length lost, never spent: counted as spent, a
            // 4 m I6's collector sat under its middle, facing back, and its
            // rear primaries looped past it (chord a third of their length).
            const auto detour = behind > 0.0F ? 2.0F * behind + 3.0F * pi * primary.radius : 0.0F;
            need += detour;
            // Turning from the port's way to the mouth's, on a bend of three
            // radii: a twin's forward ports turned about to a collector
            // behind, drawn 4 % longer than they are.
            if (gap > 1.0F) {
                const auto turn = std::acos(std::clamp(dot(primary.direction, (entry - primary.from) * (1.0F / gap)),
                                                       -1.0F, 1.0F));
                need += 3.0F * primary.radius * (turn - std::sin(turn));
            }
            reached = reached && need <= 0.97F * primary.length;
            slack += primary.length - need + 2.0F * detour;
            excess += std::max(0.0F, need - 0.97F * primary.length);
        }
        return reached;
    };
    // Turned, the mouth faces its ports from under them and the pipe after
    // it bends back onto the trunk: an I6's collector under its middle
    // ports, where facing rearwards three of its six primaries turned back
    // to reach it. Only where that pipe has the length to bend, or a turbo
    // takes the gas on.
    const auto turnable = [&](const PathLayout& layout, const Node& node) {
        if (node.turboAtInlet || !node.cylinders.empty() || throughIns(node) < 2U || node.sideIns != 0U
            || throughOuts(node) != 1U)
            return false;
        if (node.turbo) return true;
        const auto& after = layout.nodes[node.outs.front()];
        // Routed on, the pipe after it makes its own bend.
        if (after.routed) return true;
        return after.component->type == ExhaustComponentType::pipe && throughIns(after) == 1U
            && 0.8F * after.drawLength >= 0.5F * pi * turnRadius(after);
    };
    const auto seatedAt = [](const Node& original, Vec3 mouth, Vec3 axis) {
        auto candidate = original;
        candidate.turned = axis.z < 0.5F;
        candidate.mouthAxis = axis;
        if (candidate.turned) {
            candidate.mouthCentre = mouth;
            candidate.spread = zAxis;
            const auto end = mouth + axis * candidate.drawLength;
            candidate.lane = { end.x, end.y, 0.0F };
            candidate.zIn = candidate.zOut = mouth.z;
        } else {
            candidate.lane = { mouth.x, mouth.y, 0.0F };
            candidate.zIn = mouth.z;
            candidate.zOut = mouth.z + candidate.drawLength;
        }
        return candidate;
    };
    // Places `seat`'s collector, and with `twin` its mirror image across the
    // crank's plane at once: placed one after the other, the first of a
    // flat-6's two took the room under the crank and the second went out
    // beyond its head (342 mm apart from mirror images).
    const auto placeInCorridor = [&](const Seat& seat, const Seat* twin) {
        const auto& primaries = seat.primaries;
        if (primaries.empty()) return false;
        auto& node = nodeOf(seat.index);
        float centreZ = 0.0F, lastZ = -std::numeric_limits<float>::max(), meanX = 0.0F;
        for (const auto& primary : primaries) {
            centreZ += primary.from.z;
            lastZ = std::max(lastZ, primary.from.z);
            meanX += primary.from.x;
        }
        centreZ /= static_cast<float>(primaries.size());
        meanX /= static_cast<float>(primaries.size());
        const auto side = std::abs(meanX) > 30.0F ? (meanX > 0.0F ? 1.0F : -1.0F) : 0.0F;
        const auto half = 0.5F * std::max({ node.body, node.diameter, 2.0F * node.mouth });
        const auto step = std::max(25.0F, half);
        // Ports facing forwards (a motorcycle's) drop their primaries down
        // the front of the engine: the collectors may sit ahead of them,
        // under the crankcase's front (a Hayabusa's, kept behind, went over
        // its gearbox at head height).
        float forwards = 0.0F;
        for (const auto& primary : primaries) forwards += primary.direction.z;
        const auto zLow = forwards < -0.5F * static_cast<float>(primaries.size()) ? corridor.low.z + half : centreZ;
        const auto zHigh = std::max(corridor.engineHigh.z, lastZ + node.diameter);
        const auto zStep = std::max(25.0F, (zHigh - zLow) / 24.0F);
        // No higher than its ports: as low as its primaries reach, so up
        // beside the head on short ones (a 2JZ's turbo on 430 mm primaries;
        // kept under the crank, it went back past the engine and its
        // primaries were drawn twice their length).
        float top = std::numeric_limits<float>::max();
        for (const auto& primary : primaries) top = std::min(top, primary.from.y);
        top = std::min(top, corridor.high.y) - half;
        const auto original = node;
        const auto twinOriginal = twin != nullptr ? nodeOf(twin->index) : original;
        struct Choice final {
            Node node, twinNode;
            Footprint footprint, twinFootprint;
        };
        float best = std::numeric_limits<float>::max(), leastExcess = best;
        Choice chosen, stretched;
        bool found = false, anyStretched = false;
        // A collector both banks feed sits on the crank's plane where it
        // can: off it, the primaries of mirror ports differ.
        std::vector<float> across;
        for (auto x = corridor.low.x + half; x <= corridor.high.x - half + 1.0F; x += step)
            if (side * x >= -1.0F) across.push_back(x);
        float firstZ = std::numeric_limits<float>::max();
        for (const auto& primary : primaries) firstZ = std::min(firstZ, primary.from.z);
        const auto& layout = layouts[junctions[seat.index].path];
        const auto canTurn = turnable(layout, node)
            && (twin == nullptr || turnable(layouts[junctions[twin->index].path], nodeOf(twin->index)));
        const auto sibling = twin != nullptr && !twin->mirror;
        const auto offsets = sibling ? twin->offsets : std::vector<Vec3> { Vec3 {} };
        // Tries the mouth at `mouth` (its twin's at its image), keeping the
        // reached place with the least left over, else the least lacking.
        const auto tryAt = [&](Vec3 mouth, Vec3 axis, Vec3 offset) {
            float slack = 0.0F, excess = 0.0F;
            auto reached = reach(primaries, mouth, axis, slack, excess);
            const auto image = sibling ? mouth + offset
                                       : Vec3 { -mouth.x, mouth.y, mouth.z + (twin != nullptr ? twin->dz : 0.0F) };
            if (twin != nullptr) reached = reach(twin->primaries, image, axis, slack, excess) && reached;
            // Down before back: a millimetre over the crank counts as one of
            // length left over (a Hayabusa's Y pieces sat behind its head,
            // over the gearbox, rather than under the crankcase's front).
            slack += std::max(0.0F, mouth.y) + (twin != nullptr ? std::max(0.0F, image.y) : 0.0F);
            if (reached ? slack >= best : (found || excess >= leastExcess)) return false;
            Choice choice { seatedAt(original, mouth, axis), seatedAt(twinOriginal, image, axis), {}, {} };
            // A mirror twin's slots the mirror images of these.
            const auto spread = choice.node.spread;
            if (!sibling) choice.twinNode.spread = { -spread.x, spread.y, spread.z };
            choice.footprint = footprintOf(junctions[seat.index], choice.node, true);
            if (!clearOf(choice.footprint, seat.index)) return false;
            if (twin != nullptr) {
                choice.twinFootprint = footprintOf(junctions[twin->index], choice.twinNode, true);
                auto held = std::move(footprints[seat.index]);
                footprints[seat.index] = choice.footprint;
                const auto clear = clearOf(choice.twinFootprint, twin->index);
                footprints[seat.index] = std::move(held);
                if (!clear) return false;
            }
            if (reached) {
                best = slack;
                chosen = std::move(choice);
                found = true;
            } else {
                leastExcess = excess;
                stretched = std::move(choice);
                anyStretched = true;
            }
            return reached;
        };
        for (const auto offset : offsets) {
            // Siblings side by side straddle the crank's plane.
            const auto centreX = -0.5F * offset.x;
            const auto centred = side == 0.0F && corridor.low.x + half < centreX
                && corridor.high.x - half > centreX + offset.x;
            bool reachedHere = false;
            for (int pass = centred ? 0 : 1; pass < 2 && !reachedHere; ++pass) {
                for (const auto axis : { zAxis, downAxis }) {
                    const auto turned = axis.z < 0.5F;
                    if (turned && !canTurn) continue;
                    // Turned, under its ports.
                    const auto from = turned ? lastZ : zHigh;
                    const auto to = turned ? firstZ : zLow;
                    const auto stride = turned ? std::max(25.0F, (lastZ - firstZ) / 12.0F) : zStep;
                    for (auto z = from; z >= to - 1.0F; z -= stride)
                        for (auto y = corridor.low.y + half; y <= top + 1.0F; y += step)
                            for (const auto x : pass == 0 ? std::vector<float> { centreX } : across)
                                reachedHere = tryAt({ x, y, z }, axis, offset) || reachedHere;
                }
            }
        }
        if (!found && !anyStretched) return false;
        auto& choice = found ? chosen : stretched;
        node = choice.node;
        footprints[seat.index] = std::move(choice.footprint);
        if (twin != nullptr) {
            nodeOf(twin->index) = choice.twinNode;
            footprints[twin->index] = std::move(choice.twinFootprint);
        }
        return true;
    };
    // A merge further down (a 4-2-1's 2-into-1), fed by pipes routed from
    // collectors already in the corridor, goes where they reach it with the
    // least left over: left where the rules put it, a Hayabusa's secondaries
    // ran 340 mm on 256 (owner 2026-11-18).
    //
    // A merge a side branch also feeds (an H's far end) stays on the side of
    // its through pipes, as near as it can to `target` (the mirror image of
    // the junction the branch leaves), with the branch reaching it too: by
    // least slack, a V8's left trunk crossed to the right and its 250 mm
    // crossover ran 512 (owner 2026-11-18).
    // Banks on either side of the crank's plane (a V, a flat, an X), each
    // bank's ports at one distance out, not ports spread along a crank lying
    // across a motorcycle's frame (a CP4's 2-into-1 then sat on the middle).
    bool banked = false;
    {
        bool onLeft = false, onRight = false;
        float nearest = std::numeric_limits<float>::max(), furthest = 0.0F;
        for (const auto& port : ports) {
            const auto out = std::abs(port.position.x);
            if (out < 25.0F) continue;
            onLeft = onLeft || port.position.x < 0.0F;
            onRight = onRight || port.position.x > 0.0F;
            nearest = std::min(nearest, out);
            furthest = std::max(furthest, out);
        }
        banked = onLeft && onRight && furthest - nearest < 0.25F * furthest;
    }
    const auto mergeInCorridor = [&](const Junction& self, Node& node, const std::vector<Primary>& primaries,
                                     const std::vector<Primary>& branches, const Vec3* target, Footprint& placed) {
        if (primaries.empty()) return false;
        float meanX = 0.0F;
        for (const auto& primary : primaries) meanX += primary.from.x;
        meanX /= static_cast<float>(primaries.size());
        const auto side = target == nullptr || std::abs(meanX) < 25.0F ? 0.0F : (meanX > 0.0F ? 1.0F : -1.0F);
        if (target != nullptr) {
            // The image itself where every pipe in is at least as long as
            // the straight line to it: a V8's two 770 mm trunks then run
            // side by side, the crossover straight across.
            bool fits = true;
            for (const auto& pipe : primaries)
                fits = fits && pipe.lead + length(*target - zAxis * pipe.entry - pipe.from) + pipe.entry <= pipe.length;
            for (const auto& pipe : branches)
                fits = fits && pipe.lead + length(*target - pipe.from) <= pipe.length;
            if (fits) {
                auto candidate = seatedAt(node, *target, zAxis);
                auto footprint = footprintOf(self, candidate, true);
                if (clearOf(footprint, footprints.size())) {
                    node = candidate;
                    placed = std::move(footprint);
                    return true;
                }
            }
        }
        float lastZ = -std::numeric_limits<float>::max(), firstZ = std::numeric_limits<float>::max();
        for (const auto& primary : primaries) {
            lastZ = std::max(lastZ, primary.from.z);
            firstZ = std::min(firstZ, primary.from.z);
        }
        const auto half = 0.5F * std::max({ node.body, node.diameter, 2.0F * node.mouth });
        const auto step = std::max(25.0F, half);
        // Up to the corridor's top: kept as low as its inputs, an X12's
        // turbo hung through the corridor's floor wherever it went.
        const auto top = corridor.high.y - half;
        float longest = 0.0F;
        for (const auto& primary : primaries) longest = std::max(longest, primary.length);
        // As far back as they reach, past the gearbox if they must (an X12's
        // turbo found no room under it beside its lower plenum).
        const auto zHigh = std::max(lastZ, corridor.engineHigh.z) + longest;
        const auto original = node;
        float best = std::numeric_limits<float>::max(), leastExcess = best;
        Node chosen = original, stretched = original;
        Footprint chosenFootprint, stretchedFootprint;
        bool found = false, anyStretched = false;
        // Fed from both sides, on the crank's plane where it can be: off it
        // by a step of the grid, the two inputs' slots were no images.
        bool left = false, right = false;
        for (const auto& primary : primaries) {
            left = left || primary.from.x < -25.0F;
            right = right || primary.from.x > 25.0F;
        }
        std::vector<float> across;
        for (auto x = corridor.low.x + half; x <= corridor.high.x - half + 1.0F; x += step)
            if (side * x >= -1.0F) across.push_back(x);
        const auto centred = banked && left && right && side == 0.0F && corridor.low.x + half < 0.0F
            && corridor.high.x - half > 0.0F;
        for (int pass = centred ? 0 : 1; pass < 2 && !found && !anyStretched; ++pass) {
            for (auto z = firstZ; z <= zHigh + 1.0F; z += step) {
                for (auto y = corridor.low.y + half; y <= top + 1.0F; y += step) {
                    for (const auto x : pass == 0 ? std::vector<float> { 0.0F } : across) {
                        const Vec3 mouth { x, y, z };
                        float slack = 0.0F, excess = 0.0F;
                        auto reached = reach(primaries, mouth, zAxis, slack, excess);
                        for (const auto& branch : branches) {
                            // A side branch comes in across, straight from where it leaves.
                            const auto way = mouth - branch.from;
                            if (length(way) > 1.0F)
                                reached = reach({ branch }, mouth, way * (1.0F / length(way)), slack, excess) && reached;
                        }
                        // Down before back, as a collector; an H's end as near
                        // its mirror image as it can.
                        slack = target != nullptr ? length(mouth - *target) : slack + std::max(0.0F, mouth.y);
                        if (reached ? slack >= best : (found || excess >= leastExcess)) continue;
                        auto candidate = seatedAt(original, mouth, zAxis);
                        auto footprint = footprintOf(self, candidate, true);
                        if (!clearOf(footprint, footprints.size())) continue;
                        if (reached) {
                            best = slack;
                            chosen = candidate;
                            chosenFootprint = std::move(footprint);
                            found = true;
                        } else {
                            leastExcess = excess;
                            stretched = candidate;
                            stretchedFootprint = std::move(footprint);
                            anyStretched = true;
                        }
                    }
                }
            }
        }
        if (!found && !anyStretched) return false;
        node = found ? chosen : stretched;
        placed = std::move(found ? chosenFootprint : stretchedFootprint);
        return true;
    };
    // A junction one pipe from each bank feeds (an X, a Y under the middle)
    // takes them side by side, each on its own side: slots one above the
    // other, a V8's X came in from two heights and its left trunk climbed
    // over itself (asymmetry 68 mm).
    const auto sideBySide = [&](Node& node, const std::vector<Primary>& primaries) {
        if (!banked || node.turbo || primaries.size() != 2U) return;
        const auto a = primaries[0].from.x, b = primaries[1].from.x;
        if (a * b >= 0.0F || std::abs(a) < 25.0F || std::abs(b) < 25.0F) return;
        // The first slot is at minus the spread.
        node.spread = { a > b ? -1.0F : 1.0F, 0.0F, 0.0F };
    };
    // Collectors placed as each other's mirror images: their slots are
    // given as images too, once the slots are assigned.
    struct TwinPair final {
        std::size_t seat {};
        std::size_t twin {};
        float dz {};
    };
    std::vector<TwinPair> twinPairs;
    if (corridor.active) {
        std::vector<Seat> seats(junctions.size());
        std::vector<bool> inCorridor(junctions.size(), false);
        for (std::size_t j = 0; j < junctions.size(); ++j) {
            const auto& layout = layouts[junctions[j].path];
            const auto& node = layout.nodes[junctions[j].node];
            seats[j].index = j;
            seats[j].primaries = primariesOf(layout, node);
            inCorridor[j] = node.cylinders.empty() && throughIns(node) >= 1U && !seats[j].primaries.empty();
            if (inCorridor[j]) footprints[j] = {};
        }
        // Mirror images: the same collector fed by the same primaries from
        // ports that are each other's images across the crank's plane, one
        // bank staggered along the crank from the other.
        const auto mirrors = [&](std::size_t j, std::size_t k, float& dz) {
            const auto& a = seats[j].primaries;
            const auto& b = seats[k].primaries;
            const auto& first = nodeOf(j);
            const auto& second = nodeOf(k);
            // One may carry the path's turbo: the pair still sits as mirror
            // images, the turbo hung off one of them.
            if (a.size() != b.size()
                || first.component->type != second.component->type
                || std::abs(first.drawLength - second.drawLength) > 1.0F || std::abs(first.body - second.body) > 1.0F)
                return false;
            float meanA = 0.0F, meanB = 0.0F, meanX = 0.0F;
            for (std::size_t p = 0; p < a.size(); ++p) {
                meanA += a[p].from.z;
                meanB += b[p].from.z;
                meanX += a[p].from.x;
            }
            if (std::abs(meanX) < 30.0F * static_cast<float>(a.size())) return false;
            dz = (meanB - meanA) / static_cast<float>(a.size());
            return std::all_of(a.begin(), a.end(), [&](const Primary& p) {
                const Vec3 image { -p.from.x, p.from.y, p.from.z + dz };
                return std::any_of(b.begin(), b.end(), [&](const Primary& q) {
                    return length(q.from - image) < 5.0F && std::abs(q.length - p.length) < 1.0F;
                });
            });
        };
        // Siblings: two collectors of one path whose pipes, routed on, meet
        // at one merge (a 4-2-1's two Y pieces).
        const auto mergeOf = [&](std::size_t j) -> std::size_t {
            const auto& layout = layouts[junctions[j].path];
            const auto& node = layout.nodes[junctions[j].node];
            if (throughOuts(node) != 1U) return layout.nodes.size();
            const auto& pipe = layout.nodes[node.outs.front()];
            if (!pipe.routed || pipe.outs.empty()) return layout.nodes.size();
            return pipe.outs.front();
        };
        const auto siblings = [&](std::size_t j, std::size_t k) {
            if (junctions[j].path != junctions[k].path) return false;
            const auto merge = mergeOf(j);
            const auto& first = nodeOf(j);
            const auto& second = nodeOf(k);
            return merge < layouts[junctions[j].path].nodes.size() && merge == mergeOf(k)
                && first.component->type == second.component->type
                && seats[j].primaries.size() == seats[k].primaries.size();
        };
        std::vector<bool> placed(junctions.size(), false);
        for (std::size_t j = 0; j < junctions.size(); ++j) {
            if (!inCorridor[j] || placed[j]) continue;
            placed[j] = true;
            const Seat* twin = nullptr;
            for (std::size_t k = j + 1U; k < junctions.size() && twin == nullptr; ++k) {
                float dz = 0.0F;
                if (inCorridor[k] && !placed[k] && mirrors(j, k, dz)) {
                    seats[k].dz = dz;
                    twin = &seats[k];
                    placed[k] = true;
                }
            }
            for (std::size_t k = j + 1U; k < junctions.size() && twin == nullptr; ++k) {
                if (inCorridor[k] && !placed[k] && siblings(j, k)) {
                    const auto& first = nodeOf(j);
                    const auto& second = nodeOf(k);
                    const auto width = 0.5F
                        * (std::max({ first.body, first.diameter, 2.0F * first.mouth })
                           + std::max({ second.body, second.diameter, 2.0F * second.mouth }));
                    // Side by side, the pair a 4-2-1 pairs (its outer
                    // cylinders, its inner ones) crossed the engine to reach
                    // one Y piece: one behind or under the other too.
                    const auto apart = width + routing.gapMm;
                    seats[k].mirror = false;
                    seats[k].offsets = { Vec3 { apart, 0.0F, 0.0F },
                                         Vec3 { 0.0F, 0.0F, first.drawLength + routing.gapMm },
                                         Vec3 { 0.0F, 0.0F, -second.drawLength - routing.gapMm },
                                         Vec3 { 0.0F, -apart, 0.0F }, Vec3 { 0.0F, apart, 0.0F } };
                    twin = &seats[k];
                    placed[k] = true;
                }
            }
            if (placeInCorridor(seats[j], twin)) {
                if (twin != nullptr && twin->mirror) twinPairs.push_back({ j, twin->index, twin->dz });
                continue;
            }
            // No room for both: each on its own.
            const std::array<const Seat*, 2> each { &seats[j], twin };
            for (const auto* seat : each) {
                if (seat == nullptr) continue;
                auto& node = nodeOf(seat->index);
                if (!placeInCorridor(*seat, nullptr))
                    relocate(junctions[seat->index], node, seat->primaries, seat->index, footprints[seat->index]);
                if (footprints[seat->index].tubes.empty())
                    footprints[seat->index] = footprintOf(junctions[seat->index], node, true);
            }
        }
    }
    // Paths with a junction whose place is blocked, left where it is.
    std::vector<std::uint32_t> unsettled;
    const auto unsettle = [&](const PathLayout& layout) {
        if (std::find(unsettled.begin(), unsettled.end(), layout.path->id) == unsettled.end())
            unsettled.push_back(layout.path->id);
    };
    for (std::size_t j = 0; j < junctions.size(); ++j) {
        auto& layout = layouts[junctions[j].path];
        auto& node = layout.nodes[junctions[j].node];
        if (!node.cylinders.empty() || node.turboAtInlet || throughIns(node) < 1U || clearOf(footprints[j], j)) continue;
        if (!isMovable(layout)) {
            unsettle(layout);
            continue;
        }
        std::vector<Primary> primaries;
        for (const auto in : node.ins) {
            const auto& primary = layout.nodes[in];
            const auto* port = primary.cylinders.empty() || !primary.routed ? nullptr
                                                                             : portOf(ports, primary.cylinders.front());
            if (port == nullptr) continue;
            const auto lead = startLead(primary, true);
            primaries.push_back({ port->position + port->direction * lead, port->direction, lead, endLead(primary),
                                  primary.drawLength, 0.5F * std::max(primary.diameter, primary.outletDiameter) });
        }
        relocate(junctions[j], node, primaries, j, footprints[j]);
    }
    // A junction further down, fed by pipes routed from the collectors (a V's
    // Y behind the engine, on the crank's axis: in its gearbox), moves the same
    // way as its path is placed, before what follows it.
    const auto settle = [&](std::size_t l, std::size_t i) {
        auto& layout = layouts[l];
        if (!isMovable(layout) && stuck == nullptr && !corridor.active) return;
        auto& node = layout.nodes[i];
        if (node.routed || node.stack || node.firstJunction || !node.cylinders.empty() || node.turboAtInlet) return;
        std::vector<Primary> primaries;
        for (std::size_t k = 0; k < throughIns(node); ++k) {
            const auto in = node.ins[k];
            const auto& upstream = layout.nodes[in];
            if (!upstream.routed || upstream.ins.empty()) return;
            Vec3 start, direction;
            bool fromPort = false;
            if (!routeStart(layout, in, ports, start, direction, fromPort)) return;
            const auto lead = startLead(upstream, fromPort);
            primaries.push_back({ start + direction * lead, direction, lead, endLead(upstream), upstream.drawLength,
                                  0.5F * std::max(upstream.diameter, upstream.outletDiameter) });
        }
        const Junction self { l, i };
        // An H's far end: the side branch, from the junction it leaves.
        std::vector<Primary> branches;
        Vec3 image {};
        bool mirrored = false;
        if (node.sideOuts > 0U && !primaries.empty()) {
            // An H's near end: on its own side, as far in as leaves its
            // crossover room to run straight across to the far end's
            // image, and as far back as its pipe runs straight.
            const auto& crossing = layout.nodes[node.outs.back()];
            Vec3 mean {};
            float run = std::numeric_limits<float>::max();
            for (const auto& primary : primaries) {
                mean += primary.from;
                run = std::min(run, primary.length - primary.lead);
            }
            mean = mean * (1.0F / static_cast<float>(primaries.size()));
            const auto half = 0.4F * crossing.drawLength;
            const auto x = std::abs(mean.x) > half ? (mean.x > 0.0F ? half : -half) : mean.x;
            const auto across = x - mean.x;
            image = { x, mean.y, mean.z + 0.95F * std::sqrt(std::max(0.0F, run * run - across * across)) };
            mirrored = true;
        }
        for (std::size_t k = throughIns(node); k < node.ins.size(); ++k) {
            const auto in = node.ins[k];
            const auto& branch = layout.nodes[in];
            if (!branch.routed || branch.ins.empty()) continue;
            Vec3 start, direction;
            bool fromPort = false;
            if (!routeStart(layout, in, ports, start, direction, fromPort)) continue;
            const auto lead = startLead(branch, fromPort);
            branches.push_back({ start + direction * lead, direction, lead, endLead(branch), branch.drawLength,
                                 0.5F * std::max(branch.diameter, branch.outletDiameter) });
            const auto& source = layout.nodes[branch.ins.front()];
            image = { -source.lane.x, source.lane.y, source.zIn };
            mirrored = true;
        }
        if (corridor.active) {
            Footprint footprint;
            sideBySide(node, primaries);
            if (mergeInCorridor(self, node, primaries, branches, mirrored ? &image : nullptr, footprint)) {
                footprints.push_back(std::move(footprint));
                return;
            }
        }
        if (!isMovable(layout) && stuck == nullptr) return;
        auto footprint = footprintOf(self, node, true);
        if (clearOf(footprint, footprints.size())) return;
        if (!isMovable(layout)) {
            unsettle(layout);
            return;
        }
        if (relocate(self, node, primaries, footprints.size(), footprint)) footprints.push_back(std::move(footprint));
    };

    std::vector<DuctPiece> pieces;
    // Routed pipes are bent together once everything else is in place.
    struct Pending final {
        std::size_t layout;
        std::size_t node;
        DuctPiece piece;
    };
    std::vector<Pending> pending;
    std::vector<RoutedPipe> routes;
    for (std::size_t l = 0; l < layouts.size(); ++l) {
        auto& layout = layouts[l];
        const auto settleHere = [&](std::size_t i) { settle(l, i); };
        placePath(layout, ports, clearance, spec, turbos, settleHere);
        assignSlots(layout, ports);
        for (const auto& pair : twinPairs) {
            if (junctions[pair.twin].path != l) continue;
            const auto& seatLayout = layouts[junctions[pair.seat].path];
            mirrorSlots(seatLayout, seatLayout.nodes[junctions[pair.seat].node], layout,
                        layout.nodes[junctions[pair.twin].node], pair.dz, ports);
        }
        if (corridor.bounded) breakTails(layout, corridor);
        auto& nodes = layout.nodes;
        const auto pathId = layout.path->id;
        const auto pieceFor = [&](const Node& node, DuctKind kind) {
            DuctPiece piece;
            piece.kind = kind;
            piece.pathId = pathId;
            piece.elementId = node.component->id;
            piece.componentType = node.component->type;
            piece.authoredLengthMm = mm(node.component->lengthMm);
            return piece;
        };
        // Trunk components and stacks first: routed pipes end on them.
        for (const auto i : layout.order) {
            auto& node = nodes[i];
            if (node.routed) continue;
            auto piece = pieceFor(node, DuctKind::exhaustComponent);
            if (node.stack) {
                const auto* port = node.cylinders.empty() ? nullptr : portOf(ports, node.cylinders.front());
                if (port == nullptr) continue;
                shapeComponent(piece, node, { port->position, port->position + port->direction * node.drawLength });
                piece.nearEngine = true;
                pieces.push_back(std::move(piece));
                continue;
            }
            const auto inlet = at(node.lane, node.zIn);
            const auto outlet = at(node.lane, node.zOut);
            std::vector<Vec3> points { inlet, outlet };
            if (node.turned) {
                points = { node.mouthCentre, outlet };
            } else if (node.ins.size() == 1U && node.cylinders.empty() && nodes[node.ins.front()].turned
                       && !nodes[node.ins.front()].turbo) {
                // The quarter bend from the collector's axis onto the trunk.
                const auto& parent = nodes[node.ins.front()];
                const auto start = outSlot(layout, parent, indexIn(parent.outs, i));
                const auto radius = turnRadius(node);
                points.clear();
                constexpr int steps = 12;
                for (int s = 0; s <= steps; ++s) {
                    const auto angle = 0.5F * pi * static_cast<float>(s) / static_cast<float>(steps);
                    points.push_back(start + parent.mouthAxis * (radius * std::sin(angle))
                                     + zAxis * (radius * (1.0F - std::cos(angle))));
                }
                if (node.zOut > node.zIn + 1.0F) points.push_back(outlet);
            } else if (node.ins.size() == 1U && node.cylinders.empty() && !nodes[node.ins.front()].routed) {
                const auto& parent = nodes[node.ins.front()];
                const auto start = outSlot(layout, parent, indexIn(parent.outs, i));
                if (length(start - inlet) > 1.0F) {
                    // A branch moving to its own lane bends on the way, and
                    // turns away at once: the branches of an X start side by
                    // side and would otherwise run through each other.
                    const auto lead = 0.3F * node.drawLength;
                    const auto leave = std::min(lead, 0.25F * node.diameter);
                    points = smoothCurve({ start, start + zAxis * leave, outlet - zAxis * lead, outlet }, 12);
                }
            }
            shapeComponent(piece, node, std::move(points));
            // The outlet still sounds from its drawn tip: a moved microphone
            // hears the tailpipe where the picture shows it.
            piece.foldedMm = node.folded;
            pieces.push_back(std::move(piece));

            // Joins with no authored duct: ports and side branches straight
            // onto a junction.
            const auto slots = throughIns(node) + node.cylinders.size();
            for (std::size_t k = 0; slots > 1U && k < throughIns(node); ++k) {
                const auto& upstream = nodes[node.ins[k]];
                if (upstream.routed) continue;
                const auto a = outSlot(layout, upstream, indexIn(upstream.outs, i));
                const auto b = inSlot(node, k);
                if (length(b - a) < 1.0F) continue;
                auto joint = pieceFor(node, DuctKind::exhaustFeeder);
                joint.authoredLengthMm = 0.0F;
                const auto lead = 0.3F * length(b - a);
                joint.centreline = resample(smoothCurve({ a, a + zAxis * lead, b - zAxis * lead, b }, 10), 12.0F);
                joint.radii.assign(joint.centreline.size(), 0.5F * upstream.outletDiameter);
                appendTube(joint.mesh, joint.centreline, joint.radii, segmentsFor(upstream.outletDiameter));
                pieces.push_back(std::move(joint));
            }
            for (std::size_t k = 0; k < node.cylinders.size(); ++k) {
                const auto* port = portOf(ports, node.cylinders[k]);
                if (port == nullptr) continue;
                auto joint = pieceFor(node, DuctKind::exhaustFeeder);
                joint.authoredLengthMm = 0.0F;
                const auto b = inSlot(node, throughIns(node) + k);
                joint.centreline = resample(routeWithLength(port->position, port->direction, b, zAxis, 0.0F, downAxis), 12.0F);
                joint.radii.assign(joint.centreline.size(), 0.5F * port->diameterMm);
                appendTube(joint.mesh, joint.centreline, joint.radii, 16);
                joint.nearEngine = true;
                pieces.push_back(std::move(joint));
            }
        }
        for (const auto i : layout.order) {
            auto& node = nodes[i];
            if (!node.routed) continue;
            Vec3 a, dirA;
            bool fromPort = false;
            if (!routeStart(layout, i, ports, a, dirA, fromPort)) continue;
            const auto joins = !node.outs.empty();
            Vec3 b = a + dirA * node.drawLength;
            auto dirB = joins ? zAxis : dirA;
            bool crossover = false;
            if (joins) {
                const auto& downstream = nodes[node.outs.front()];
                const auto k = indexIn(downstream.ins, i);
                crossover = k >= throughIns(downstream);
                dirB = entryAxis(downstream);
                if (downstream.turboAtInlet) {
                    // Onto the turbine, along the trunk.
                    b = downstream.turboInlet;
                    dirB = zAxis;
                } else if (k >= throughIns(downstream)) {
                    // Into the flank, from the side it comes from.
                    Vec3 outward;
                    b = flank(downstream, a, outward);
                    dirB = outward * -1.0F;
                } else {
                    b = inSlot(downstream, k);
                }
            }
            auto piece = pieceFor(node, DuctKind::exhaustComponent);
            piece.nearEngine = fromPort;
            // A port facing along the crankshaft (a radial) bends outwards,
            // away from the axis; everything else sags.
            auto bulge = downAxis;
            if (fromPort && std::abs(dirA.z) > 0.9F && std::abs(a.x) + std::abs(a.y) > 1.0F)
                bulge = normalise(Vec3 { a.x, a.y, 0.0F });
            // On the corridor's floor a sag has nowhere to go: outwards
            // instead, so the two banks' pipes bow as mirror images (a V8's
            // trunks both bowed to its right).
            const auto side = a.x + b.x;
            if (corridor.active && !fromPort && std::abs(side) > 1.0F
                && std::min(a.y, b.y) - corridor.low.y < 3.0F * node.diameter)
                bulge = { side > 0.0F ? 1.0F : -1.0F, 0.0F, 0.0F };
            RoutedPipe route;
            route.radius = 0.5F * std::max(node.diameter, node.outletDiameter);
            route.startLead = startLead(node, fromPort);
            route.endLead = joins ? endLead(node) : 0.0F;
            route.lengthMm = node.drawLength;
            if (corridor.active) {
                // More than the room between its ends holds: drawn as a
                // broken view, its length kept by the simulator. Spent in
                // full, a 1.5 m primary to a collector 600 mm off coiled.
                // Its straight reach, past the end and back where it starts
                // behind it, as the collector was placed.
                const auto from = a + dirA * route.startLead, to = b - dirB * route.endLead;
                const auto behind = std::max(0.0F, dot(from - to, dirB));
                auto straight = route.startLead + route.endLead + length(to - from);
                if (behind > 0.0F) straight += 2.0F * behind + 3.0F * pi * route.radius;
                // An H's crossover runs straight across: given the room of
                // any other pipe, a V8's 250 mm one looped on its 186 mm span.
                const auto room = crossover ? straight : std::max(1.3F * straight, straight + 3.0F * node.diameter);
                if (node.drawLength > room) {
                    piece.foldedMm = node.drawLength - room;
                    route.lengthMm = room;
                }
            }
            route.points = routeWithLength(a, dirA, b, dirB, route.lengthMm, bulge);
            route.startDirection = dirA;
            route.endDirection = dirB;
            routes.push_back(std::move(route));
            pending.push_back({ l, i, std::move(piece) });
        }
    }

    // Everything already laid out, and what the caller asks to avoid, is fixed.
    auto obstacles = avoid;
    for (const auto& piece : pieces)
        if (piece.radii.size() == piece.centreline.size() && piece.centreline.size() >= 2U)
            obstacles.push_back({ piece.centreline, piece.radii });
    auto blocks = solids;
    if (turbos != nullptr)
        for (const auto& turbo : *turbos) {
            if (!turbo.placed) continue;
            obstacles.push_back({ turbo.scroll, turbo.scrollRadii });
            for (const auto& solid : turboSolids(turbo)) blocks.push_back(solid);
        }
    // A path whose junction is blocked and a pipe of which starts out with
    // its axis more than a diameter deep in the engine (beyond what bending
    // moves it: a 12-into-1's primaries through the block) is laid out again
    // with its junctions free to move: nothing more is drawn here. Shallower,
    // the bending may clear it, and the drawn pipes decide (below).
    if (stuck != nullptr) {
        for (std::size_t r = 0; r < routes.size(); ++r) {
            const auto pathId = layouts[pending[r].layout].path->id;
            if (std::find(unsettled.begin(), unsettled.end(), pathId) == unsettled.end()
                || std::find(stuck->begin(), stuck->end(), pathId) != stuck->end())
                continue;
            const auto& route = routes[r];
            const auto deep = std::any_of(route.points.begin(), route.points.end(), [&](Vec3 point) {
                if (length(point - route.points.front()) < 2.0F * route.radius
                    || length(point - route.points.back()) < 2.0F * route.radius)
                    return false;
                return std::any_of(blocks.begin(), blocks.end(), [&](const EngineSolid& solid) {
                    return -signedDistance(solid, point) > 2.0F * route.radius;
                });
            });
            if (deep) stuck->push_back(pathId);
        }
        if (!stuck->empty()) return pieces;
    }
    // The corridor's walls keep the pipes in it.
    blocks.insert(blocks.end(), corridor.walls.begin(), corridor.walls.end());
    routeRound(routes, obstacles, blocks, routing);
    if (corridor.active) mirrorRoutes(routes, obstacles, blocks);
    for (std::size_t r = 0; r < pending.size(); ++r) {
        auto& item = pending[r];
        shapeComponent(item.piece, layouts[item.layout].nodes[item.node], std::move(routes[r].points));
        pieces.push_back(std::move(item.piece));
    }
    // A rotor's passage: its port through the housing, from the face to the
    // mouth the pipe leaves from (after the routing, which starts there).
    // Inside the housing by construction, so drawn as a body with no
    // centreline, as a plenum is: not a pipe to route round the engine.
    for (const auto& layout : layouts) {
        for (const auto& passage : layout.passages) {
            const auto* port = portOf(ports, passage.face);
            if (port == nullptr) continue;
            DuctPiece piece;
            piece.pathId = layout.path->id;
            piece.elementId = passage.component.id;
            piece.componentType = passage.component.type;
            const auto radius = 0.5F * port->diameterMm;
            appendTube(piece.mesh, { port->inner, port->position }, { radius, radius }, segmentsFor(port->diameterMm));
            piece.nearEngine = true;
            pieces.push_back(std::move(piece));
        }
    }
    return pieces;
}

std::vector<DuctPiece> layoutLocal(const EngineConfig& config, const std::vector<PortAnchor>& ports, float boreMm,
                                   const std::vector<RouteTube>& avoid, const std::vector<EngineSolid>& solids,
                                   std::vector<TurboPlacement>* turbos, const Corridor& corridor) {
    // A path whose junction sits in the way, a pipe of which starts out deep
    // in the engine, goes straight to the second pass.
    std::vector<std::uint32_t> blocked;
    auto pieces = layoutExhaustOnce(config, ports, boreMm, avoid, solids, turbos, {}, &blocked, corridor);
    if (!blocked.empty())
        return layoutExhaustOnce(config, ports, boreMm, avoid, solids, turbos, blocked, nullptr, corridor);
    // A path any drawn pipe of which sits in the engine, its gearbox, an
    // intake box or another path's turbo (beyond a graze, two radii at each
    // end left out, as RouteCheck judges it), or whose turbo sits in the
    // engine, is laid out again with its junctions free to move; every other
    // keeps its layout exactly.
    const auto block = [&](std::uint32_t pathId) {
        if (std::find(blocked.begin(), blocked.end(), pathId) == blocked.end()) blocked.push_back(pathId);
    };
    std::vector<std::pair<std::uint32_t, EngineSolid>> turboBodies;
    if (turbos != nullptr)
        for (const auto& turbo : *turbos) {
            if (!turbo.placed) continue;
            for (const auto& body : turboSolids(turbo)) {
                turboBodies.emplace_back(turbo.pathId, body);
                if (bodySunk(body, solids)) block(turbo.pathId);
            }
        }
    for (const auto& piece : pieces) {
        if (piece.radii.size() != piece.centreline.size() || piece.centreline.size() < 2U) continue;
        if (std::find(blocked.begin(), blocked.end(), piece.pathId) != blocked.end()) continue;
        const auto& points = piece.centreline;
        for (std::size_t i = 0; i < points.size(); ++i) {
            const auto r = piece.radii[i];
            if (length(points[i] - points.front()) < 2.0F * r || length(points[i] - points.back()) < 2.0F * r) continue;
            const auto sunk = [&](const EngineSolid& solid) { return r - signedDistance(solid, points[i]) > grazeMm(r); };
            if (std::any_of(solids.begin(), solids.end(), sunk)
                || std::any_of(turboBodies.begin(), turboBodies.end(), [&](const auto& body) {
                       return body.first != piece.pathId && sunk(body.second);
                   })) {
                block(piece.pathId);
                break;
            }
        }
    }
    if (blocked.empty()) return pieces;
    return layoutExhaustOnce(config, ports, boreMm, avoid, solids, turbos, blocked, nullptr, corridor);
}

/** The corridor round the engine (the first `engineSolids` of `solids`) and
    its ports, in the layout's frame. */
Corridor corridorFor(const std::vector<PortAnchor>& ports, const std::vector<EngineSolid>& solids,
                     std::size_t engineSolids) {
    Corridor corridor;
    if (ports.empty() || engineSolids == 0U) return corridor;
    Vec3 low { std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
    Vec3 high = low * -1.0F;
    for (std::size_t s = 0; s < engineSolids && s < solids.size(); ++s) {
        const auto half = solidHalfExtent(solids[s]);
        low = { std::min(low.x, solids[s].centre.x - half.x), std::min(low.y, solids[s].centre.y - half.y),
                std::min(low.z, solids[s].centre.z - half.z) };
        high = { std::max(high.x, solids[s].centre.x + half.x), std::max(high.y, solids[s].centre.y + half.y),
                 std::max(high.z, solids[s].centre.z + half.z) };
    }
    if (high.x < low.x) return corridor;
    float topPort = -std::numeric_limits<float>::max(), frontPort = std::numeric_limits<float>::max();
    for (const auto& port : ports) {
        topPort = std::max(topPort, port.position.y + 2.0F * port.diameterMm);
        frontPort = std::min(frontPort, port.position.z - 3.0F * port.diameterMm);
    }
    // RouteCheck's plausibility bounds.
    corridor.active = true;
    corridor.bounded = true;
    corridor.engineLow = low;
    corridor.engineHigh = high;
    corridor.low = { low.x - 150.0F, low.y - 250.0F, std::min(low.z, frontPort) - 100.0F };
    corridor.high = { high.x + 150.0F, topPort, std::numeric_limits<float>::max() };
    // Walls a hand thick, from ahead of the corridor to well past the
    // gearbox: beyond it the trunk runs straight along its lane.
    constexpr float wall = 100.0F;
    const auto front = corridor.low.z, back = high.z + 800.0F;
    const auto& c = corridor;
    corridor.walls = {
        boxBetween({ c.low.x - wall, c.low.y - wall, front - wall }, { c.low.x, c.high.y + wall, back }),
        boxBetween({ c.high.x, c.low.y - wall, front - wall }, { c.high.x + wall, c.high.y + wall, back }),
        boxBetween({ c.low.x, c.low.y - wall, front - wall }, { c.high.x, c.low.y, back }),
        boxBetween({ c.low.x, c.high.y, front - wall }, { c.high.x, c.high.y + wall, back }),
        boxBetween({ c.low.x, c.low.y, front - wall }, { c.high.x, c.high.y, front }),
    };
    return corridor;
}

} // namespace

std::vector<DuctPiece> layoutExhaust(const EngineConfig& config, const std::vector<PortAnchor>& ports, float boreMm,
                                     const std::vector<RouteTube>& avoid, const std::vector<EngineSolid>& solids,
                                     std::vector<TurboPlacement>* turbos, const ExhaustFrame& frame) {
    // Laid out with rearwards along +Z: a motorcycle's (rearwards across its
    // crank) is turned into that frame and back.
    constexpr Vec3 up { 0.0F, 1.0F, 0.0F };
    auto rear = Vec3 { frame.rear.x, 0.0F, frame.rear.z };
    rear = length(rear) > 1.0e-3F ? normalise(rear) : zAxis;
    const auto across = normalise(cross(up, rear));
    const auto turned = std::abs(rear.z - 1.0F) > 1.0e-5F;
    const auto toLocal = [&](Vec3 v) { return Vec3 { dot(v, across), v.y, dot(v, rear) }; };
    const auto toWorld = [&](Vec3 v) { return across * v.x + up * v.y + rear * v.z; };
    auto localPorts = ports;
    auto localAvoid = avoid;
    auto localSolids = solids;
    if (turned) {
        for (auto& port : localPorts) {
            port.position = toLocal(port.position);
            port.direction = toLocal(port.direction);
            port.inner = toLocal(port.inner);
            for (auto& point : port.transferDuct) point = toLocal(point);
            port.carburettorUp = toLocal(port.carburettorUp);
            port.transferMirrorPoint = toLocal(port.transferMirrorPoint);
            port.transferMirrorNormal = toLocal(port.transferMirrorNormal);
        }
        for (auto& tube : localAvoid)
            for (auto& point : tube.points) point = toLocal(point);
        for (auto& solid : localSolids) {
            solid.centre = toLocal(solid.centre);
            for (auto& axis : solid.axes) axis = toLocal(axis);
        }
    }
    // A radial's pipes stand round it, wherever its cylinders are: only its
    // tail is broken off past the same plane.
    auto corridor = corridorFor(localPorts, localSolids, frame.engineSolids);
    if (frame.radial) {
        corridor.active = false;
        corridor.walls.clear();
    }
    auto pieces = layoutLocal(config, localPorts, boreMm, localAvoid, localSolids, turbos, corridor);
    for (auto& piece : pieces) markFold(piece);
    if (!turned) return pieces;
    for (auto& piece : pieces) {
        for (auto& vertex : piece.mesh.vertices) {
            vertex.position = toWorld(vertex.position);
            vertex.normal = toWorld(vertex.normal);
        }
        for (auto& point : piece.centreline) point = toWorld(point);
        piece.openEnd = toWorld(piece.openEnd);
        piece.openAxis = toWorld(piece.openAxis);
    }
    if (turbos != nullptr)
        for (auto& turbo : *turbos) {
            for (auto& point : turbo.scroll) point = toWorld(point);
            turbo.axis = toWorld(turbo.axis);
            turbo.radial = toWorld(turbo.radial);
            turbo.turbineCentre = toWorld(turbo.turbineCentre);
            turbo.compressorCentre = toWorld(turbo.compressorCentre);
            turbo.outlet = toWorld(turbo.outlet);
        }
    return pieces;
}

std::vector<EngineSolid> superchargerSolids(const SuperchargerPlacement& supercharger) {
    std::vector<EngineSolid> result;
    if (!supercharger.placed) return result;
    EngineSolid solid;
    solid.shape = EngineSolid::Shape::cylinder;
    solid.centre = supercharger.centre;
    solid.axes[1] = supercharger.axis;
    solid.axes[0] = supercharger.radial;
    solid.axes[2] = normalise(cross(supercharger.radial, supercharger.axis));
    solid.half = { supercharger.outerRadius, 0.5F * supercharger.width + 0.5F * (supercharger.outerRadius - 1.45F * supercharger.wheelRadius), 0.0F };
    result.push_back(solid);
    return result;
}

std::vector<EngineSolid> turboSolids(const TurboPlacement& turbo) {
    std::vector<EngineSolid> result;
    if (!turbo.placed) return result;
    const auto ring = [&](Vec3 centre, float radius, float halfLength) {
        EngineSolid solid;
        solid.shape = EngineSolid::Shape::cylinder;
        solid.centre = centre;
        solid.axes[1] = turbo.axis;
        solid.axes[0] = turbo.radial;
        solid.axes[2] = normalise(cross(turbo.radial, turbo.axis));
        solid.half = { radius, halfLength, 0.0F };
        result.push_back(solid);
    };
    // The turbine housing's centre round the wheel (its volute is a tube),
    // the bearing housing, and the compressor with its volute.
    ring(turbo.turbineCentre, turbo.turbineWheelRadius + 5.0F, 0.5F * turbo.turbineWidth);
    const auto bearingStart = turbo.turbineCentre - turbo.axis * (0.5F * turbo.turbineWidth);
    const auto bearingEnd = turbo.compressorCentre + turbo.axis * (0.5F * turbo.compressorWidth);
    ring((bearingStart + bearingEnd) * 0.5F, 0.6F * turbo.turbineWheelRadius + 6.0F,
         0.5F * length(bearingStart - bearingEnd));
    ring(turbo.compressorCentre, turbo.compressorOuterRadius, 0.5F * turbo.compressorWidth);
    return result;
}

// ------------------------------------------------------------------- intake

namespace {

struct IntakeRunner final {
    const PortAnchor* port {};
    float lengthMm {};
    float diameter {};
    float plenumDiameter {};
};

/** Box `size` (local x, y, z) placed by an orthonormal frame. */
void addBox(DuctPiece& piece, Vec3 x, Vec3 y, Vec3 z, Vec3 centre, Vec3 size) {
    appendBox(piece.mesh, Mat4::basis(x, y, z, centre), size);
}

/** A supercharger upstream of a single throttle whose mouth is `mouth`,
    offset `away` from the engine. */
SuperchargerPlacement placeSupercharger(const ForcedInductionConfig& forced, std::uint32_t pathId, float throttle,
                                        Vec3 mouth, Vec3 direction, Vec3 away) {
    SuperchargerPlacement result;
    // Without an inducer diameter, an eye a little wider than the throttle;
    // an inducer is about 70 % of its wheel.
    const auto eye = forced.compressorInducerDiameterMm > 1.0 ? 0.5F * mm(forced.compressorInducerDiameterMm)
                                                              : 0.6F * throttle;
    const auto wheel = eye / 0.7F;
    auto side = away - direction * dot(away, direction);
    if (length(side) < 1.0e-3F) side = normalise(cross(direction, length(cross(direction, zAxis)) > 0.1F ? zAxis : downAxis));
    side = normalise(side);
    const auto ring = 1.45F * wheel;
    const auto gap = 1.2F * throttle;
    result.placed = true;
    result.pathId = pathId;
    result.axis = direction;
    result.radial = -side;
    result.wheelRadius = wheel;
    result.width = 0.8F * wheel;
    result.outerRadius = 1.85F * wheel;
    result.centre = mouth + direction * (gap + 0.5F * result.width) + side * ring;
    result.chargeOutlet = mouth + direction * gap;
    result.throttleInlet = mouth;
    result.chargeRadius = 0.55F * throttle;
    result.eye = result.centre + direction * (0.5F * result.width + 0.6F * wheel);
    result.eyeRadius = eye + 3.0F;
    return result;
}

/** One throttle bore of `throttle` diameter from `face` along `direction`,
    1.1 bores long; a slide carburettor's slide tower towards `up` and its
    float bowl opposite. */
void appendThrottleBore(Mesh& mesh, Vec3 face, Vec3 direction, float throttle, bool carburettor, Vec3 up) {
    const auto bodyLength = 1.1F * throttle;
    const auto frame = Mat4::frameAlongY(face, direction);
    appendCylinder(mesh, frame, 0.62F * throttle, 0.62F * throttle, bodyLength, 28);
    appendTorus(mesh, frame * Mat4::translation({ 0.0F, 0.15F * throttle, 0.0F }), 0.66F * throttle,
                0.06F * throttle, 28, 8);
    if (!carburettor) return;
    const auto middle = face + direction * (0.5F * bodyLength);
    appendCylinder(mesh, Mat4::frameAlongY(middle + up * (0.45F * throttle), up), 0.42F * throttle, 0.42F * throttle,
                   0.75F * throttle, 20);
    appendCylinder(mesh, Mat4::frameAlongY(middle - up * (0.45F * throttle), -up), 0.58F * throttle,
                   0.58F * throttle, 0.70F * throttle, 24);
}

/** Throttle bores, airbox and inlet duct upstream of a plenum, in the order
    the air crosses them from the atmosphere; a supercharger between the
    throttle and the airbox when `supercharger` is given. */
void appendUpstream(std::vector<DuctPiece>& pieces, std::uint32_t pathId, const IntakeConfig& geometry,
                    const std::vector<Vec3>& faces, Vec3 direction, const ForcedInductionConfig& forced,
                    Vec3 away, SuperchargerPlacement* supercharger) {
    const auto throttle = std::max(5.0F, mm(geometry.throttleDiameterMm));
    const auto bodyLength = 1.1F * throttle;
    DuctPiece bodies;
    bodies.kind = DuctKind::intakeThrottle;
    bodies.pathId = pathId;
    bodies.nearEngine = true;
    std::vector<Vec3> mouths;
    Vec3 mouthCentre {};
    for (const auto face : faces) {
        // A slide carburettor: the slide's tower above the bore, the float
        // bowl under it.
        auto up = -downAxis - direction * dot(-downAxis, direction);
        up = length(up) > 1.0e-3F ? normalise(up) : normalise(cross(direction, zAxis));
        // With individual throttles the plates are in the runners (and the
        // carburettors with them): the plenum's mouths are bare necks.
        if (geometry.individualThrottles)
            appendCylinder(bodies.mesh, Mat4::frameAlongY(face, direction), 0.55F * throttle, 0.55F * throttle,
                           bodyLength, 28);
        else
            appendThrottleBore(bodies.mesh, face, direction, throttle, geometry.carburettor.enabled, up);
        mouths.push_back(face + direction * bodyLength);
        mouthCentre += mouths.back();
    }
    mouthCentre = mouthCentre * (1.0F / static_cast<float>(std::max<std::size_t>(1U, mouths.size())));
    const auto charged = supercharger != nullptr && mouths.size() == 1U;
    if (charged) {
        *supercharger = placeSupercharger(forced, pathId, throttle, mouthCentre, direction, away);
        // The air reaches the throttle through the supercharger: the airbox
        // and the inlet duct feed its eye.
        mouths = { supercharger->eye };
        mouthCentre = supercharger->eye;
    }

    const auto airbox = mm(geometry.airboxVolumeLitres) * litre;
    const auto ductLength = mm(geometry.inletDuctLengthMm);
    const auto ductDiameter = geometry.inletDuctDiameterMm > 1.0 ? mm(geometry.inletDuctDiameterMm) : throttle;
    const auto bellmouth = geometry.bellmouthDiameterMm > 1.0 ? mm(geometry.bellmouthDiameterMm) : ductDiameter;
    // Several trumpets are authored as one lip of their combined area: each
    // bore's own lip is that over the root of their count.
    const auto lip = bellmouth / std::sqrt(static_cast<float>(std::max<std::size_t>(1U, mouths.size())));
    if (!charged && airbox <= 0.0F && ductLength <= 1.0F && geometry.podFilter) {
        // A pod filter on each mouth: a clamped collar, then the element, a
        // cone wider than the bore closed by its end cap.
        for (const auto mouth : mouths) {
            appendCylinder(bodies.mesh, Mat4::frameAlongY(mouth, direction), 0.56F * throttle, 0.56F * throttle,
                           0.25F * throttle, 28, false, false);
            appendCylinder(bodies.mesh, Mat4::frameAlongY(mouth + direction * (0.25F * throttle), direction),
                           0.85F * throttle, 0.62F * throttle, 1.4F * throttle, 28, true, true);
        }
    } else if (!charged && airbox <= 0.0F && ductLength <= 1.0F && lip > 1.05F * throttle) {
        // Open throttle mouths: a velocity stack on each.
        for (const auto mouth : mouths)
            appendCylinder(bodies.mesh, Mat4::frameAlongY(mouth, direction), 0.5F * throttle, 0.5F * lip,
                           0.45F * throttle, 28, false, false);
    }
    pieces.push_back(std::move(bodies));

    auto inlet = mouthCentre;
    if (airbox > 0.0F) {
        DuctPiece box;
        box.kind = DuctKind::intakeAirbox;
        box.pathId = pathId;
        box.authoredLengthMm = 0.0F;
        const auto frame = Mat4::frameAlongY({}, direction);
        const auto x = frame.transformDirection({ 1.0F, 0.0F, 0.0F });
        const auto z = frame.transformDirection({ 0.0F, 0.0F, 1.0F });
        float spanZ = 0.0F;
        for (const auto mouth : mouths) spanZ = std::max(spanZ, std::abs(dot(mouth - mouthCentre, z)) * 2.0F);
        const auto depth = std::max(spanZ + 2.0F * throttle, std::cbrt(airbox));
        const auto width = std::max(1.6F * throttle, std::sqrt(airbox / depth));
        const auto height = airbox / (depth * width);
        addBox(box, x, direction, z, mouthCentre + direction * (0.5F * height), { width, height, depth });
        box.centreline = { mouthCentre, mouthCentre + direction * height };
        inlet = mouthCentre + direction * height;
        pieces.push_back(std::move(box));
    }
    if (ductLength > 1.0F) {
        DuctPiece duct;
        duct.kind = DuctKind::intakeInletDuct;
        duct.pathId = pathId;
        duct.authoredLengthMm = ductLength;
        // Gas-flow order: from the open mouth towards the engine.
        const auto mouth = inlet + direction * ductLength;
        duct.centreline = resample({ mouth, inlet }, 12.0F);
        duct.radii.assign(duct.centreline.size(), 0.5F * ductDiameter);
        appendTube(duct.mesh, duct.centreline, duct.radii, segmentsFor(ductDiameter));
        if (bellmouth > 1.05F * ductDiameter)
            appendCylinder(duct.mesh, Mat4::frameAlongY(mouth, direction), 0.5F * ductDiameter, 0.5F * bellmouth,
                           0.35F * ductDiameter, segmentsFor(bellmouth), false, false);
        pieces.push_back(std::move(duct));
    }
    // The outermost piece is where the air comes in, and the path's mouth
    // radiates from.
    auto& outermost = pieces.back();
    outermost.opensToAtmosphere = true;
    outermost.openEnd = ductLength > 1.0F ? inlet + direction * ductLength : inlet;
    outermost.openAxis = normalise(direction);
}

/** How deep what appendUpstream would draw from `face` along `direction` (one
    throttle bore, its airbox and its inlet duct, at their sizes) sits in the
    solids of `solids` (mm; 0 when clear). */
[[nodiscard]] float upstreamDepth(const IntakeConfig& geometry, Vec3 face, Vec3 direction,
                                  const std::vector<EngineSolid>& solids) {
    const auto throttle = std::max(5.0F, mm(geometry.throttleDiameterMm));
    // Square across, as RouteCheck judges a body by its bounding box.
    const auto along = [&](Vec3 start, float radius, float run) {
        EngineSolid solid;
        solid.centre = start + direction * (0.5F * run);
        solid.axes[1] = direction;
        solid.axes[0] = normalise(cross(direction, std::abs(direction.y) < 0.9F ? Vec3 { 0.0F, 1.0F, 0.0F } : zAxis));
        solid.axes[2] = cross(solid.axes[0], direction);
        solid.half = { radius, 0.5F * run, radius };
        return solid;
    };
    // A slide carburettor's tower and float bowl stand out of its bore.
    std::vector<EngineSolid> drawn { along(face, (geometry.carburettor.enabled ? 1.2F : 0.62F) * throttle,
                                           1.1F * throttle) };
    auto inlet = face + direction * (1.1F * throttle);
    const auto airbox = mm(geometry.airboxVolumeLitres) * litre;
    if (airbox > 0.0F) {
        const auto depth = std::max(2.0F * throttle, std::cbrt(airbox));
        const auto width = std::max(1.6F * throttle, std::sqrt(airbox / depth));
        const auto height = airbox / (depth * width);
        // Its widest half, as a drum round the way out.
        drawn.push_back(along(inlet, 0.5F * std::max(depth, width), height));
        inlet = inlet + direction * height;
    }
    const auto ductLength = mm(geometry.inletDuctLengthMm);
    if (ductLength > 1.0F) {
        const auto ductDiameter = geometry.inletDuctDiameterMm > 1.0 ? mm(geometry.inletDuctDiameterMm) : throttle;
        drawn.push_back(along(inlet, 0.5F * ductDiameter, ductLength));
    }
    float deepest = 0.0F;
    for (const auto& body : drawn)
        for (const auto& point : solidSamples(body))
            for (const auto& solid : solids) deepest = std::max(deepest, -signedDistance(solid, point));
    return deepest;
}

/** The first of `options` (a face and the way out of it) whose throttle,
    airbox and inlet duct clear `solids` (RouteCheck's 2 mm); else the one
    that sits in them least. */
void chooseUpstream(const IntakeConfig& geometry, const std::vector<std::pair<Vec3, Vec3>>& options,
                    const std::vector<EngineSolid>& solids, std::vector<Vec3>& faces, Vec3& upstream) {
    float least = std::numeric_limits<float>::max();
    for (const auto& [face, direction] : options) {
        const auto depth = upstreamDepth(geometry, face, direction, solids);
        if (depth < least) {
            least = depth;
            faces = { face };
            upstream = direction;
        }
        if (depth <= 2.0F) return;
    }
}

/** A two-stroke's intake, cylinder by cylinder from the crankcase out, then
    the airbox every carburettor shares (layoutIntake). */
std::vector<DuctPiece> layoutTwoStrokeIntake(const EngineConfig& config, const std::vector<PortAnchor>& ports) {
    std::vector<DuctPiece> pieces;
    // Measured off real reed engines: a reed block about 20 mm deep, the
    // carburettor on a 25 mm rubber manifold behind it (3-4 cm from the
    // petals), a boot of at least 25 mm into the airbox.
    constexpr float reedDepth = 20.0F;
    constexpr float manifoldLength = 25.0F;
    constexpr float bootClearance = 25.0F;
    struct Mouth final {
        Vec3 point;
        Vec3 direction;
        float throttle {};
        std::uint32_t pathId {};
    };
    std::vector<Mouth> mouths;
    float airboxLitres = 0.0F;
    for (const auto& path : config.intakePaths) {
        if (path.cylinderIds.empty()) continue;
        const auto* port = portOf(ports, path.cylinderIds.front());
        if (port == nullptr || port->transferDuct.size() < 2U) continue;
        const auto& geometry = path.geometry;
        const auto throttle = std::max(5.0F, mm(geometry.throttleDiameterMm));
        airboxLitres = std::max(airboxLitres, mm(geometry.airboxVolumeLitres));

        // The transfer duct is the path's runner in the solver.
        const auto& cylinder = *std::find_if(config.cylinders.begin(), config.cylinders.end(),
            [&path](const CylinderConfig& c) { return c.id == path.cylinderIds.front(); });
        DuctPiece runner;
        runner.kind = DuctKind::intakeRunner;
        runner.pathId = path.id;
        runner.elementId = cylinder.id;
        runner.nearEngine = true;
        runner.authoredLengthMm = std::max(1.0F, mm(cylinder.intakeRunnerLengthMm > 1.0 ? cylinder.intakeRunnerLengthMm
                                                                                       : geometry.runnerLengthMm));
        const auto runnerDiameter = cylinder.intakeRunnerDiameterMm > 1.0 ? mm(cylinder.intakeRunnerDiameterMm)
                                                                          : mm(geometry.runnerDiameterMm);
        // Cast in the barrel's wall: no wider than the wall holds.
        const auto runnerRadius = std::min(0.5F * std::max(5.0F, runnerDiameter), 0.18F * mm(cylinder.boreMm));
        // Its authored length, taken or given at the crankcase end.
        auto duct = smoothCurve(port->transferDuct, 8);
        auto excess = polylineLength(duct) - runner.authoredLengthMm;
        if (excess < 0.0F) {
            duct.insert(duct.begin(), duct.front() + normalise(duct.front() - duct[1]) * -excess);
        } else {
            while (duct.size() > 2U && length(duct[1] - duct[0]) <= excess) {
                excess -= length(duct[1] - duct[0]);
                duct.erase(duct.begin());
            }
            duct.front() += normalise(duct[1] - duct[0]) * std::min(excess, 0.9F * length(duct[1] - duct[0]));
        }
        runner.centreline = resample(duct, 8.0F);
        runner.radii.assign(runner.centreline.size(), runnerRadius);
        appendTube(runner.mesh, runner.centreline, runner.radii, 16, true, true);
        // Its twin on the other flank carries the same flow: one duct in the
        // solver, two passages in the casting.
        if (const auto normal = port->transferMirrorNormal; length(normal) > 0.5F) {
            auto twin = runner.centreline;
            for (auto& point : twin) point = point - normal * (2.0F * dot(point - port->transferMirrorPoint, normal));
            appendTube(runner.mesh, twin, runner.radii, 16, true, true);
        }
        pieces.push_back(std::move(runner));

        // The reed block (on the crankcase, or in a cylinder port that has
        // one), the manifold, the carburettor.
        DuctPiece bodies;
        bodies.kind = DuctKind::intakeThrottle;
        bodies.pathId = path.id;
        bodies.nearEngine = true;
        const auto direction = normalise(port->direction);
        const auto up = normalise(port->carburettorUp - direction * dot(port->carburettorUp, direction));
        const auto across = normalise(cross(direction, up));
        const auto reed = config.twoStroke.intake != TwoStrokeIntake::cylinderPort || config.twoStroke.intakePortReed;
        if (reed)
            addBox(bodies, across, direction, up, port->position + direction * (0.5F * reedDepth),
                   { 1.35F * throttle, reedDepth, 1.15F * throttle });
        const auto manifoldStart = port->position + direction * (reed ? reedDepth : 0.0F);
        appendCylinder(bodies.mesh, Mat4::frameAlongY(manifoldStart, direction), 0.6F * throttle, 0.58F * throttle,
                       manifoldLength, 24);
        const auto face = manifoldStart + direction * manifoldLength;
        appendThrottleBore(bodies.mesh, face, direction, throttle, geometry.carburettor.enabled, up);
        bodies.centreline = { port->position, face + direction * (1.1F * throttle) };
        pieces.push_back(std::move(bodies));
        mouths.push_back({ face + direction * (1.1F * throttle), direction, throttle, path.id });
    }
    if (mouths.empty()) return pieces;

    // Without an airbox each carburettor breathes through its own mouth.
    const auto airbox = airboxLitres * litre;
    if (airbox <= 0.0F) {
        for (auto& piece : pieces) {
            if (piece.kind != DuctKind::intakeThrottle) continue;
            piece.opensToAtmosphere = true;
            piece.openEnd = piece.centreline.back();
            piece.openAxis = normalise(piece.centreline.back() - piece.centreline.front());
        }
        return pieces;
    }

    // One airbox for them all, out where the carburettors point (above a
    // V's valley, behind an inline engine's barrels).
    Vec3 out {};
    float widestThrottle = 0.0F;
    for (const auto& mouth : mouths) {
        out += mouth.direction;
        widestThrottle = std::max(widestThrottle, mouth.throttle);
    }
    out = length(out) > 0.3F * static_cast<float>(mouths.size()) ? normalise(out) : -downAxis;
    const auto acrossAxis = std::abs(dot(out, zAxis)) > 0.9F ? normalise(cross(out, Vec3 { 1.0F, 0.0F, 0.0F }))
                                                             : zAxis;
    const auto sideAxis = normalise(cross(acrossAxis, out));
    float floor = -1.0e9F;
    for (const auto& mouth : mouths) floor = std::max(floor, dot(mouth.point, out));
    floor += bootClearance + 0.6F * widestThrottle;
    float lowA = 1.0e9F, highA = -1.0e9F, lowS = 1.0e9F, highS = -1.0e9F;
    for (const auto& mouth : mouths) {
        lowA = std::min(lowA, dot(mouth.point, acrossAxis));
        highA = std::max(highA, dot(mouth.point, acrossAxis));
        lowS = std::min(lowS, dot(mouth.point, sideAxis));
        highS = std::max(highS, dot(mouth.point, sideAxis));
    }
    auto spanA = highA - lowA + 2.4F * widestThrottle;
    auto spanS = highS - lowS + 2.4F * widestThrottle;
    auto height = airbox / (spanA * spanS);
    // A deep narrow box is drawn squatter and wider, at the same volume.
    if (const auto squat = std::max(spanA, spanS); height > squat) {
        const auto grow = std::sqrt(height / squat);
        spanA *= grow;
        spanS *= grow;
        height = airbox / (spanA * spanS);
    }
    // A box too flat for its boots is drawn narrower, at the same volume.
    if (const auto lowest = 1.2F * widestThrottle; height < lowest) {
        const auto shrink = std::sqrt(height / lowest);
        spanA *= shrink;
        spanS *= shrink;
        height = airbox / (spanA * spanS);
    }
    const auto centre = acrossAxis * (0.5F * (lowA + highA)) + sideAxis * (0.5F * (lowS + highS))
        + out * (floor + 0.5F * height);
    for (const auto& mouth : mouths) {
        DuctPiece boot;
        boot.kind = DuctKind::intakeInletDuct;
        boot.pathId = mouth.pathId;
        boot.nearEngine = true;
        const auto entry = mouth.point + out * (floor - dot(mouth.point, out));
        const auto lead = 0.35F * (floor - dot(mouth.point, out));
        boot.centreline = resample(smoothCurve({ entry, entry - out * lead, mouth.point + mouth.direction * lead,
                                                 mouth.point }, 10), 8.0F);
        boot.radii.assign(boot.centreline.size(), 0.62F * mouth.throttle);
        appendTube(boot.mesh, boot.centreline, boot.radii, segmentsFor(1.24F * mouth.throttle));
        pieces.push_back(std::move(boot));
    }
    DuctPiece box;
    box.kind = DuctKind::intakeAirbox;
    box.pathId = mouths.front().pathId;
    addBox(box, acrossAxis, out, sideAxis, centre, { spanA, height, spanS });
    box.opensToAtmosphere = true;
    box.openEnd = centre + out * (0.5F * height);
    box.openAxis = out;
    pieces.push_back(std::move(box));
    return pieces;
}

} // namespace

std::vector<DuctPiece> layoutIntake(const EngineConfig& config, const std::vector<PortAnchor>& ports, float boreMm,
                                    bool radial, const std::vector<EngineSolid>& solids,
                                    SuperchargerPlacement* supercharger) {
    if (supercharger != nullptr) *supercharger = {};
    if (config.cycle == EngineCycle::twoStroke && !radial
        && std::all_of(ports.begin(), ports.end(), [](const PortAnchor& port) { return port.transferDuct.size() >= 2U; }))
        return layoutTwoStrokeIntake(config, ports);
    const auto& forced = config.forcedInduction;
    const auto superchargerWanted = forced.enabled && forced.type == ForcedInductionType::supercharger;
    std::vector<DuctPiece> pieces;
    // Runners are bent together once every plenum is in place.
    struct Pending final {
        std::size_t piece;
        IntakeRunner runner;
        /** Individual throttles: the plate's bore, drawn on the runner. */
        float plateBore { 0.0F };
        bool carburettor { false };
    };
    std::vector<Pending> pending;
    std::vector<RoutedPipe> routes;
    // Balance pipes between the boxes of a path: the runners bend round them.
    std::vector<RouteTube> balances;
    auto avoid = solids;
    const auto pathCount = std::max<std::size_t>(1U, config.intakePaths.size());
    for (std::size_t p = 0; p < pathCount; ++p) {
        const auto& geometry = p < config.intakePaths.size() ? config.intakePaths[p].geometry : config.intake;
        const auto pathId = p < config.intakePaths.size() ? config.intakePaths[p].id : 1U;
        std::vector<IntakeRunner> runners;
        for (const auto& cylinder : config.cylinders) {
            std::size_t owner = 0;
            for (std::size_t q = 0; q < config.intakePaths.size(); ++q) {
                const auto& ids = config.intakePaths[q].cylinderIds;
                if (std::find(ids.begin(), ids.end(), cylinder.id) != ids.end()) {
                    owner = q;
                    break;
                }
            }
            if (owner != p) continue;
            const auto* port = portOf(ports, cylinder.id);
            if (port == nullptr) continue;
            IntakeRunner runner;
            runner.port = port;
            runner.lengthMm = std::max(1.0F, mm(cylinder.intakeRunnerLengthMm > 1.0 ? cylinder.intakeRunnerLengthMm
                                                                                   : geometry.runnerLengthMm));
            runner.diameter = std::max(5.0F, mm(cylinder.intakeRunnerDiameterMm > 1.0 ? cylinder.intakeRunnerDiameterMm
                                                                                     : geometry.runnerDiameterMm));
            runner.plenumDiameter = geometry.runnerPlenumDiameterMm > 1.0 ? mm(geometry.runnerPlenumDiameterMm)
                                                                          : runner.diameter;
            runners.push_back(runner);
        }
        if (runners.empty()) continue;

        Vec3 centre {}, direction {};
        float shortest = std::numeric_limits<float>::max(), widest = 0.0F;
        for (const auto& runner : runners) {
            centre += runner.port->position;
            direction += runner.port->direction;
            shortest = std::min(shortest, runner.lengthMm);
            widest = std::max(widest, std::max(runner.diameter, runner.plenumDiameter));
        }
        centre = centre * (1.0F / static_cast<float>(runners.size()));
        const auto volume = std::max(1.0F, mm(geometry.plenumVolumeLitres) * litre);

        std::vector<Vec3> ends(runners.size());
        std::vector<Vec3> arrivals(runners.size());
        std::vector<Vec3> bulges(runners.size());
        std::vector<DuctPiece> plenums;
        DuctPiece plenum;
        plenum.kind = DuctKind::intakePlenum;
        plenum.pathId = pathId;
        plenum.nearEngine = true;
        std::vector<Vec3> faces;
        Vec3 upstream;
        // Away from the engine, for a supercharger beside the throttle.
        Vec3 away;

        // A radial two-stroke's path is one cylinder's crankcase circuit: its
        // carburettor stands at the end of that cylinder's own runner, out
        // along its port. One drum a path, all on the axis, drew five drums
        // and five carburettors in one place, as if one fed them all (owner
        // 2026-11-21, R5 2S).
        const auto ownCarburettor = radial && config.cycle == EngineCycle::twoStroke && runners.size() == 1U;
        IntakeConfig shared;
        const auto* drawnGeometry = &geometry;
        if (ownCarburettor) {
            const auto& runner = runners.front();
            const auto out = normalise(runner.port->direction);
            const Vec3 flat { runner.port->position.x, runner.port->position.y, 0.0F };
            ends[0] = runner.port->position + out * (0.9F * runner.lengthMm);
            arrivals[0] = out;
            bulges[0] = length(flat) > 1.0e-3F ? normalise(flat) : normalise(cross(out, zAxis));
            faces.push_back(ends[0]);
            upstream = out;
            away = out;
            // Each carburettor draws its share of the airbox the paths copy.
            shared = geometry;
            shared.airboxVolumeLitres /= static_cast<double>(pathCount);
            drawnGeometry = &shared;
        } else if (radial) {
            // A drum around the crank axis, in front of the cylinders.
            float radius = 0.0F;
            for (const auto& runner : runners)
                radius += length(Vec3 { runner.port->position.x, runner.port->position.y, 0.0F });
            radius /= static_cast<float>(runners.size());
            auto drum = std::max(0.35F * radius, 1.5F * widest);
            auto drumLength = volume / (pi * drum * drum);
            if (drumLength < 0.8F * widest) drumLength = 0.8F * widest;
            if (drumLength > drum) {
                drumLength = std::cbrt(volume / pi);
                drum = std::sqrt(volume / (pi * drumLength));
            }
            // Far enough ahead that the straight run is most of the runner.
            const auto inward = std::max(0.0F, radius - drum);
            const auto ahead = std::sqrt(std::max(0.0F, 0.45F * shortest * shortest - inward * inward));
            const Vec3 hub { 0.0F, 0.0F, centre.z - ahead - 0.5F * drumLength };
            for (std::size_t i = 0; i < runners.size(); ++i) {
                const auto outward = normalise(Vec3 { runners[i].port->position.x, runners[i].port->position.y, 0.0F });
                ends[i] = hub + outward * drum;
                arrivals[i] = -outward;
                bulges[i] = outward;
            }
            appendCylinder(plenum.mesh, Mat4::translation(hub - zAxis * (0.5F * drumLength)) * Mat4::rotationX(0.5F * pi),
                           drum, drum, drumLength, 48);
            EngineSolid solid;
            solid.shape = EngineSolid::Shape::cylinder;
            solid.centre = hub;
            solid.axes[0] = { 1.0F, 0.0F, 0.0F };
            solid.axes[1] = zAxis;
            solid.axes[2] = { 0.0F, 1.0F, 0.0F };
            solid.half = { drum, 0.5F * drumLength, 0.0F };
            avoid.push_back(solid);
            plenum.centreline = { hub - zAxis * (0.5F * drumLength), hub + zAxis * (0.5F * drumLength) };
            // Below the drum, else wherever the throttle and its airbox
            // clear the engine and its gearbox.
            const Vec3 xAxis { 1.0F, 0.0F, 0.0F };
            chooseUpstream(geometry,
                           { { hub + downAxis * drum, downAxis }, { hub - zAxis * (0.5F * drumLength), -zAxis },
                             { hub + xAxis * drum, xAxis }, { hub - xAxis * drum, -xAxis } },
                           solids, faces, upstream);
            away = -zAxis;
            plenums.push_back(std::move(plenum));
        } else {
            // Runners whose ports open into two opposite valleys (an X's
            // top and bottom ones) feed a box in each, the volume shared by
            // their count, joined by a balance pipe; the throttle feeds the
            // upper one. One box for both ran half the runners through the
            // block (owner 2026-11-18). The simulator keeps one plenum.
            std::vector<std::vector<std::size_t>> groups(1);
            for (std::size_t i = 0; i < runners.size(); ++i) groups.front().push_back(i);
            const auto across = [&](std::size_t i) {
                const auto d = runners[i].port->direction;
                const Vec3 flat { d.x, d.y, 0.0F };
                return length(flat) > 1.0e-3F ? normalise(flat) : Vec3 {};
            };
            {
                float most = 2.0F;
                std::size_t seedA = 0, seedB = 0;
                for (std::size_t i = 0; i < runners.size(); ++i)
                    for (std::size_t k = i + 1U; k < runners.size(); ++k)
                        if (dot(across(i), across(k)) < most) {
                            most = dot(across(i), across(k));
                            seedA = i;
                            seedB = k;
                        }
                auto meanA = across(seedA), meanB = across(seedB);
                std::vector<std::size_t> a, b;
                for (int pass = 0; pass < 6 && most < -0.5F; ++pass) {
                    a.clear();
                    b.clear();
                    Vec3 sumA {}, sumB {};
                    for (std::size_t i = 0; i < runners.size(); ++i) {
                        const auto d = across(i);
                        if (dot(d, meanA) >= dot(d, meanB)) {
                            a.push_back(i);
                            sumA += d;
                        } else {
                            b.push_back(i);
                            sumB += d;
                        }
                    }
                    if (a.empty() || b.empty()) break;
                    meanA = normalise(sumA);
                    meanB = normalise(sumB);
                }
                if (!a.empty() && !b.empty() && dot(meanA, meanB) < -0.5F) {
                    // The upper valley first: it takes the throttle.
                    if (meanB.y > meanA.y) std::swap(a, b);
                    groups = { a, b };
                }
            }
            std::vector<EngineSolid> boxes;
            for (std::size_t g = 0; g < groups.size(); ++g) {
                const auto& members = groups[g];
                DuctPiece box;
                box.kind = DuctKind::intakePlenum;
                box.pathId = pathId;
                box.nearEngine = true;
                centre = {};
                direction = {};
                shortest = std::numeric_limits<float>::max();
                widest = 0.0F;
                for (const auto i : members) {
                    const auto& runner = runners[i];
                    centre += runner.port->position;
                    direction += runner.port->direction;
                    shortest = std::min(shortest, runner.lengthMm);
                    widest = std::max(widest, std::max(runner.diameter, runner.plenumDiameter));
                }
                centre = centre * (1.0F / static_cast<float>(members.size()));
                const auto boxVolume = volume * static_cast<float>(members.size()) / static_cast<float>(runners.size());
                // A box whose face holds every runner end, as far out as the
                // shortest runner allows.
                auto out = Vec3 { direction.x, direction.y, 0.0F };
                out = length(out) < 1.0e-3F ? Vec3 { 0.0F, 1.0F, 0.0F } : normalise(out);
                auto lateral = normalise(cross(out, zAxis));
                if (lateral.y < 0.0F) lateral = -lateral;
                // Runner ends close in towards the middle by about half the
                // distance between the banks; the face stands off as far as the
                // runner furthest across still allows.
                constexpr float keepAcross = 0.45F;
                float widestAcross = 0.0F;
                for (const auto i : members)
                    widestAcross = std::max(widestAcross, std::abs(dot(runners[i].port->position - centre, lateral)));
                const auto residual = (1.0F - keepAcross) * widestAcross;
                // The straight line spends 0.9 of the runner; one that also has
                // to come across turns twice on the way (an S), so its offset
                // counts a little over its length: a flat-6's runners came out
                // 8 % long once its heads stood off by their real combustion
                // chamber.
                const auto sideways = 1.1F * residual;
                const auto straightReach = std::sqrt(std::max(0.0625F * shortest * shortest,
                                                              0.81F * shortest * shortest - sideways * sideways));
                // A short, wide runner cannot spend a tenth of its length in a
                // bend: the face stands out at least as far as one arc of two
                // diameters' radius leaves room for, past the straight leads
                // (audit R03: a CP2's 120 mm by 35 mm runners bent at 0.6 of a
                // diameter).
                const auto bendRadius = 3.0F * widest;
                const auto curved = std::max(0.0F, shortest - 1.8F * widest);
                const auto halfAngle = std::min(0.5F * pi, curved / (2.0F * bendRadius));
                const auto chord = std::min(0.97F * shortest, 1.8F * widest + 2.0F * bendRadius * std::sin(halfAngle));
                const auto reach = std::max(straightReach, std::sqrt(std::max(0.0F, chord * chord - sideways * sideways)));
                float zMin = std::numeric_limits<float>::max(), zMax = -zMin, latMin = zMin, latMax = -zMin;
                for (const auto i : members) {
                    const auto offset = runners[i].port->position - centre;
                    const auto offsetAcross = dot(offset, lateral);
                    ends[i] = centre + out * reach + zAxis * offset.z + lateral * (keepAcross * offsetAcross);
                    arrivals[i] = out;
                    bulges[i] = std::abs(offsetAcross) > 0.1F * boreMm ? lateral * (offsetAcross > 0.0F ? 1.0F : -1.0F)
                                                                       : lateral;
                    zMin = std::min(zMin, ends[i].z);
                    zMax = std::max(zMax, ends[i].z);
                    latMin = std::min(latMin, dot(ends[i] - centre, lateral));
                    latMax = std::max(latMax, dot(ends[i] - centre, lateral));
                }
                const auto depth = zMax - zMin + 1.4F * widest;
                auto width = latMax - latMin + 1.4F * widest;
                width = std::max(width, std::sqrt(boxVolume / depth));
                const auto height = std::max(0.8F * widest, boxVolume / (depth * width));
                const auto boxCentre = Vec3 { centre.x, centre.y, 0.5F * (zMin + zMax) } + out * (reach + 0.5F * height)
                    + lateral * (0.5F * (latMin + latMax));
                addBox(box, lateral, out, zAxis, boxCentre, { width, height, depth });
                EngineSolid solid;
                solid.centre = boxCentre;
                solid.axes[0] = lateral;
                solid.axes[1] = out;
                solid.axes[2] = zAxis;
                solid.half = Vec3 { width, height, depth } * 0.5F;
                avoid.push_back(solid);
                boxes.push_back(solid);
                box.centreline = { boxCentre - out * (0.5F * height), boxCentre + out * (0.5F * height) };
                plenums.push_back(std::move(box));
                if (g != 0U) continue;
                away = out;
                const auto throttles = std::max<std::uint32_t>(1U, geometry.throttleCount);
                if (throttles == 1U) {
                    // Ahead of the box, else wherever the throttle and its
                    // airbox clear the engine and its gearbox.
                    chooseUpstream(geometry,
                                   { { boxCentre - zAxis * (0.5F * depth), -zAxis },
                                     { boxCentre + zAxis * (0.5F * depth), zAxis },
                                     { boxCentre + out * (0.5F * height), out },
                                     { boxCentre + lateral * (0.5F * width), lateral },
                                     { boxCentre - lateral * (0.5F * width), -lateral } },
                                   solids, faces, upstream);
                } else {
                    const auto spacing = depth / static_cast<float>(throttles);
                    for (std::uint32_t k = 0; k < throttles; ++k)
                        faces.push_back(boxCentre + out * (0.5F * height)
                                        + zAxis * ((static_cast<float>(k) - 0.5F * static_cast<float>(throttles - 1U)) * spacing));
                    upstream = out;
                }
            }
            if (boxes.size() == 2U) {
                // The balance pipe, round the end the throttle leaves free,
                // as wide as the widest runner.
                float widestRunner = 0.0F;
                for (const auto& runner : runners)
                    widestRunner = std::max(widestRunner, std::max(runner.diameter, runner.plenumDiameter));
                const auto radius = 0.5F * widestRunner;
                const auto leave = upstream.z < -0.5F ? zAxis : -zAxis;
                const auto a = boxes[0].centre + leave * boxes[0].half.z;
                const auto b = boxes[1].centre + leave * boxes[1].half.z;
                Vec3 low { std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                           std::numeric_limits<float>::max() };
                Vec3 high = low * -1.0F;
                boundsOf(avoid, balances, low, high);
                const auto margin = 6.0F * widestRunner + 150.0F;
                low = low - Vec3 { margin, margin, margin };
                high = high + Vec3 { margin, margin, margin };
                const FreeSpace space(low, high, std::max(20.0F, widestRunner), avoid, balances, radius + 8.0F);
                auto way = freeGuess(space, a, leave, widestRunner, b, -leave, widestRunner, std::max(80.0F, 3.0F * (radius + 8.0F)));
                if (way.size() >= 2U) {
                    DuctPiece pipe;
                    pipe.kind = DuctKind::intakePlenum;
                    pipe.pathId = pathId;
                    pipe.nearEngine = true;
                    pipe.centreline = resample(way, 12.0F);
                    pipe.radii.assign(pipe.centreline.size(), radius);
                    appendTube(pipe.mesh, pipe.centreline, pipe.radii, segmentsFor(widestRunner));
                    balances.push_back({ pipe.centreline, pipe.radii });
                    plenums.push_back(std::move(pipe));
                }
            }
        }

        for (std::size_t i = 0; i < runners.size(); ++i) {
            const auto& runner = runners[i];
            DuctPiece piece;
            piece.kind = DuctKind::intakeRunner;
            piece.pathId = pathId;
            piece.elementId = runner.port->cylinderId;
            piece.authoredLengthMm = runner.lengthMm;
            piece.nearEngine = true;
            RoutedPipe route;
            route.points = routeWithLength(runner.port->position, runner.port->direction, ends[i], arrivals[i],
                                           runner.lengthMm, bulges[i]);
            route.radius = 0.5F * std::max(runner.diameter, runner.plenumDiameter);
            route.lengthMm = runner.lengthMm;
            route.startDirection = runner.port->direction;
            route.endDirection = arrivals[i];
            route.startLead = 0.8F * runner.diameter;
            route.endLead = 2.0F * route.radius;
            routes.push_back(std::move(route));
            pending.push_back({ pieces.size(), runner,
                                geometry.individualThrottles ? std::max(5.0F, mm(geometry.throttleDiameterMm)) : 0.0F,
                                geometry.carburettor.enabled });
            pieces.push_back(std::move(piece));
        }
        for (auto& box : plenums) pieces.push_back(std::move(box));
        const auto wantsSupercharger = superchargerWanted && supercharger != nullptr && !supercharger->placed;
        appendUpstream(pieces, pathId, *drawnGeometry, faces, upstream, forced, away,
                       wantsSupercharger ? supercharger : nullptr);
        if (wantsSupercharger && supercharger->placed) {
            // The runners and the exhaust stay clear of it.
            for (const auto& solid : superchargerSolids(*supercharger)) avoid.push_back(solid);
        }
    }

    routeRound(routes, balances, avoid, RouteSettings {});
    for (std::size_t r = 0; r < pending.size(); ++r) {
        const auto& runner = pending[r].runner;
        auto& piece = pieces[pending[r].piece];
        // Gas-flow order: from the plenum to the valves.
        auto route = std::move(routes[r].points);
        std::reverse(route.begin(), route.end());
        auto radii = radiiAlong(route, [&](float t) {
            return 0.5F * (runner.plenumDiameter + (runner.diameter - runner.plenumDiameter) * t);
        });
        // The port inside the head, behind the valves, closes the runner.
        route.push_back(runner.port->inner);
        radii.push_back(0.5F * runner.diameter);
        appendTube(piece.mesh, route, radii, 18, false, true);
        route.pop_back();
        radii.pop_back();
        if (const auto bore = pending[r].plateBore; bore > 0.0F && route.size() >= 2U) {
            // Individual throttles: the body sits in the runner, the plate
            // half way along it, where the solver puts it.
            const auto total = polylineLength(route);
            auto walked = 0.0F;
            for (std::size_t k = 1; k < route.size(); ++k) {
                const auto step = length(route[k] - route[k - 1U]);
                if (walked + step < 0.5F * (total - 1.1F * bore) && k + 1U < route.size()) {
                    walked += step;
                    continue;
                }
                const auto direction = normalise(route[k] - route[k - 1U]);
                const auto start = route[k - 1U]
                    + direction * std::clamp(0.5F * (total - 1.1F * bore) - walked, 0.0F, step);
                auto up = -downAxis - direction * dot(-downAxis, direction);
                up = length(up) > 1.0e-3F ? normalise(up) : normalise(cross(direction, zAxis));
                DuctPiece body;
                body.kind = DuctKind::intakeThrottle;
                body.pathId = piece.pathId;
                body.elementId = piece.elementId;
                body.nearEngine = true;
                appendThrottleBore(body.mesh, start, direction, bore, pending[r].carburettor, up);
                body.centreline = { start, start + direction * (1.1F * bore) };
                pieces.push_back(std::move(body));
                break;
            }
        }
        piece.centreline = std::move(route);
        piece.radii = std::move(radii);
    }
    return pieces;
}

} // namespace enginelab::render
