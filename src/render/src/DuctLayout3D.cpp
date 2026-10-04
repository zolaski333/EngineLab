#include <enginelab/render/DuctLayout3D.hpp>

#include <enginelab/exhaust/LegacyExhaustNetwork.hpp>

#include <algorithm>
#include <cmath>
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

/** Point in the plane across the trunk, `index` of `count`, around `centre`. */
[[nodiscard]] Vec3 slot(Vec3 centre, std::size_t index, std::size_t count, float radius, Vec3 spread) {
    if (count <= 1U) return centre;
    if (count == 2U) return centre + spread * (index == 0 ? -radius : radius);
    const auto angle = 2.0F * pi * static_cast<float>(index) / static_cast<float>(count);
    const auto other = normalise(cross(zAxis, spread));
    return centre + (spread * std::cos(angle) + other * std::sin(angle)) * radius;
}

// ------------------------------------------------------------------ exhaust

struct Node final {
    const ExhaustComponentConfig* component {};
    std::vector<std::size_t> ins;
    std::vector<std::size_t> outs;
    std::vector<std::uint32_t> cylinders;
    bool routed {};
    bool stack {};
    bool firstJunction {};
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
    std::vector<Vec3> curve;
};

struct PathLayout final {
    const ExhaustPathConfig* path {};
    ExhaustNetworkConfig network;
    std::vector<Node> nodes;
    std::vector<std::size_t> order;
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
    const auto byPort = [](const auto& a, const auto& b) { return a.first < b.first; };
    std::stable_sort(ins.begin(), ins.end(), byPort);
    std::stable_sort(outs.begin(), outs.end(), byPort);
    for (const auto& [port, link] : ins) layout.nodes[link.first].ins.push_back(link.second);
    for (const auto& [port, link] : outs) layout.nodes[link.first].outs.push_back(link.second);

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
            case ExhaustComponentType::outlet: drawLength = node.diameter; break;
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
        node.routed = !node.stack && c.type == ExhaustComponentType::pipe
            && (!node.cylinders.empty()
                || (node.outs.size() == 1U && layout.nodes[node.outs.front()].ins.size() > 1U));
    }
}

[[nodiscard]] std::size_t indexIn(const std::vector<std::size_t>& list, std::size_t value) noexcept {
    const auto found = std::find(list.begin(), list.end(), value);
    return found == list.end() ? 0U : static_cast<std::size_t>(std::distance(list.begin(), found));
}

/** Lane of the `k`-th branch leaving `node`. An X (as many outputs as
    inputs) sends each output back to the side its input came from. */
[[nodiscard]] Vec3 outLane(const PathLayout& layout, const Node& node, std::size_t k) {
    const auto n = node.outs.size();
    if (n <= 1U) return node.lane;
    if (node.ins.size() == n) return layout.nodes[node.ins[k]].lane;
    float widest = node.body;
    for (const auto out : node.outs) widest = std::max(widest, layout.nodes[out].body);
    return node.lane + node.spread * ((static_cast<float>(k) - 0.5F * static_cast<float>(n - 1U)) * 1.9F * widest);
}

[[nodiscard]] Vec3 at(Vec3 lane, float z) noexcept { return { lane.x, lane.y, z }; }

[[nodiscard]] float slotRadius(const Node& node) noexcept { return 0.28F * std::max(node.body, 1.3F * node.diameter); }

[[nodiscard]] Vec3 inSlot(const Node& node, std::size_t k) {
    return slot(at(node.lane, node.zIn), k, node.ins.size() + node.cylinders.size(), slotRadius(node), node.spread);
}

[[nodiscard]] Vec3 outSlot(const PathLayout& layout, const Node& node, std::size_t k) {
    const auto n = node.outs.size();
    if (n > 1U && node.ins.size() == n) {
        // An X: each branch leaves on the side it is heading to.
        const auto target = outLane(layout, node, k);
        const auto offset = Vec3 { target.x - node.lane.x, target.y - node.lane.y, 0.0F };
        return at(node.lane, node.zOut) + normalise(offset) * (length(offset) > 1.0e-3F ? 0.3F * node.body : 0.0F);
    }
    return slot(at(node.lane, node.zOut), k, n, 0.3F * node.body, node.spread);
}

void placePath(PathLayout& layout, const std::vector<PortAnchor>& ports, float clearance) {
    auto& nodes = layout.nodes;
    for (const auto i : layout.order) {
        auto& node = nodes[i];
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
        Vec3 laneSum {};
        int lanes = 0;
        bool spreadSet = false;
        for (const auto in : node.ins) {
            const auto& upstream = nodes[in];
            float candidate {};
            if (upstream.routed && upstream.ins.empty()) {
                candidate = upstream.portZ + 0.5F * upstream.drawLength;
                laneSum += upstream.lane;
            } else if (upstream.routed) {
                const auto from = nodes[upstream.ins.front()].zOut;
                routedEarliest = std::min(routedEarliest, from + 0.75F * upstream.drawLength);
                routedLatest = std::max(routedLatest, from + 0.25F * upstream.drawLength);
                candidate = -std::numeric_limits<float>::max();
                laneSum += upstream.lane;
            } else {
                candidate = upstream.zOut;
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
        if (routedLatest > -std::numeric_limits<float>::max()) z = std::max(z, std::max(routedEarliest, routedLatest));
        if (lanes == 0) z = 0.0F;
        node.lane = lanes > 0 ? laneSum * (1.0F / static_cast<float>(lanes)) : Vec3 {};
        node.zIn = z;
        node.zOut = z + node.drawLength;
    }
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
            const auto mouth = std::max(body, 0.68F * node.diameter);
            radii = radiiAlong(points, [&](float t) { return t < 0.6F ? mouth + (end - mouth) * (t / 0.6F) : end; });
            capStart = true;
        } else {
            radii = radiiAlong(points, [&](float t) { return start + (end - start) * t; });
        }
    } else {
        const auto cone = std::clamp(0.8F * (body - std::min(start, end)) / std::max(1.0F, node.drawLength), 0.04F, 0.3F);
        const auto mouth = c.type == ExhaustComponentType::merge ? body : start;
        capStart = c.type == ExhaustComponentType::merge;
        radii = radiiAlong(points, [&](float t) {
            if (t < cone) return mouth + (body - mouth) * (t / cone);
            if (t > 1.0F - cone) return body + (end - body) * ((t - (1.0F - cone)) / cone);
            return body;
        });
    }
    appendTube(piece.mesh, points, radii, segmentsFor(node.body), capStart, false);
    piece.centreline = std::move(points);
}

} // namespace

std::vector<DuctPiece> layoutExhaust(const EngineConfig& config, const std::vector<PortAnchor>& ports, float boreMm) {
    const auto clearance = std::max(60.0F, 0.8F * boreMm);
    std::vector<PathLayout> layouts(config.exhaustPaths.size());
    for (std::size_t p = 0; p < layouts.size(); ++p) {
        auto& layout = layouts[p];
        layout.path = &config.exhaustPaths[p];
        layout.network = layout.path->network ? *layout.path->network : makeEditableExhaustNetwork(*layout.path);
        buildNodes(layout);
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
            junctions.push_back({ p, i });
        }
    }
    // Junctions of one path that would overlap queue up along the trunk,
    // centred on their ports (the two Y pieces of a 4-2-1); another path's
    // junction moves across instead.
    std::vector<bool> done(junctions.size(), false);
    for (std::size_t a = 0; a < junctions.size(); ++a) {
        if (done[a]) continue;
        std::vector<std::size_t> cluster { a };
        auto& first = layouts[junctions[a].path].nodes[junctions[a].node];
        for (std::size_t b = a + 1U; b < junctions.size(); ++b) {
            if (done[b] || junctions[b].path != junctions[a].path) continue;
            const auto& other = layouts[junctions[b].path].nodes[junctions[b].node];
            if (length(other.lane - first.lane) < 1.5F * std::max(first.body, other.body)) cluster.push_back(b);
        }
        float total = 0.0F, centre = 0.0F, gap = 0.0F;
        for (const auto k : cluster) {
            const auto& node = layouts[junctions[k].path].nodes[junctions[k].node];
            total += node.drawLength;
            centre += node.zIn;
            gap = std::max(gap, 0.8F * node.body);
            done[k] = true;
        }
        centre /= static_cast<float>(cluster.size());
        total += gap * static_cast<float>(cluster.size() - 1U);
        auto z = cluster.size() > 1U ? centre - 0.5F * total : centre;
        for (const auto k : cluster) {
            auto& node = layouts[junctions[k].path].nodes[junctions[k].node];
            node.zIn = z;
            node.zOut = z + node.drawLength;
            z = node.zOut + gap;
        }
    }
    for (std::size_t a = 0; a < junctions.size(); ++a) {
        auto& node = layouts[junctions[a].path].nodes[junctions[a].node];
        for (int attempt = 0; attempt < 8; ++attempt) {
            bool clash = false;
            for (std::size_t b = 0; b < a; ++b) {
                if (junctions[b].path == junctions[a].path) continue;
                const auto& other = layouts[junctions[b].path].nodes[junctions[b].node];
                const auto reach = 1.5F * std::max(node.body, other.body);
                if (length(other.lane - node.lane) < reach && node.zIn < other.zOut + reach && other.zIn < node.zOut + reach)
                    clash = true;
            }
            if (!clash) break;
            node.lane += node.spread * (1.6F * node.body);
        }
    }

    std::vector<DuctPiece> pieces;
    for (auto& layout : layouts) {
        placePath(layout, ports, clearance);
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
            if (node.ins.size() == 1U && node.cylinders.empty() && !nodes[node.ins.front()].routed) {
                const auto& parent = nodes[node.ins.front()];
                const auto start = outSlot(layout, parent, indexIn(parent.outs, i));
                if (length(start - inlet) > 1.0F) {
                    // A branch moving to its own lane bends on the way.
                    const auto lead = 0.3F * node.drawLength;
                    points = smoothCurve({ start, start + zAxis * lead, outlet - zAxis * lead, outlet }, 12);
                }
            }
            shapeComponent(piece, node, std::move(points));
            pieces.push_back(std::move(piece));

            // Joins with no authored duct: ports and side branches straight
            // onto a junction.
            const auto slots = node.ins.size() + node.cylinders.size();
            for (std::size_t k = 0; slots > 1U && k < node.ins.size(); ++k) {
                const auto& upstream = nodes[node.ins[k]];
                if (upstream.routed) continue;
                const auto a = outSlot(layout, upstream, indexIn(upstream.outs, i));
                const auto b = inSlot(node, k);
                if (length(b - a) < 1.0F) continue;
                auto joint = pieceFor(node, DuctKind::exhaustFeeder);
                joint.authoredLengthMm = 0.0F;
                const auto lead = 0.3F * length(b - a);
                joint.centreline = resample(smoothCurve({ a, a + zAxis * lead, b - zAxis * lead, b }, 10), 12.0F);
                appendTube(joint.mesh, joint.centreline, { 0.5F * upstream.outletDiameter }, segmentsFor(upstream.outletDiameter));
                pieces.push_back(std::move(joint));
            }
            for (std::size_t k = 0; k < node.cylinders.size(); ++k) {
                const auto* port = portOf(ports, node.cylinders[k]);
                if (port == nullptr) continue;
                auto joint = pieceFor(node, DuctKind::exhaustFeeder);
                joint.authoredLengthMm = 0.0F;
                const auto b = inSlot(node, node.ins.size() + k);
                joint.centreline = resample(routeWithLength(port->position, port->direction, b, zAxis, 0.0F, downAxis), 12.0F);
                appendTube(joint.mesh, joint.centreline, { 0.5F * port->diameterMm }, 16);
                joint.nearEngine = true;
                pieces.push_back(std::move(joint));
            }
        }
        for (const auto i : layout.order) {
            auto& node = nodes[i];
            if (!node.routed) continue;
            Vec3 a, dirA;
            const PortAnchor* port = nullptr;
            if (node.ins.empty()) {
                port = node.cylinders.empty() ? nullptr : portOf(ports, node.cylinders.front());
                if (port == nullptr) continue;
                a = port->position;
                dirA = port->direction;
            } else {
                const auto& parent = nodes[node.ins.front()];
                a = outSlot(layout, parent, indexIn(parent.outs, i));
                dirA = zAxis;
            }
            Vec3 b;
            if (node.outs.empty()) {
                b = a + dirA * node.drawLength;
            } else {
                const auto& downstream = nodes[node.outs.front()];
                b = inSlot(downstream, indexIn(downstream.ins, i));
            }
            auto piece = pieceFor(node, DuctKind::exhaustComponent);
            piece.nearEngine = port != nullptr;
            // Primaries drop into their collector instead of turning round
            // to arrive along the trunk.
            const auto arrival = port != nullptr ? normalise(Vec3 { 0.0F, -1.0F, 0.6F }) : zAxis;
            // A port facing along the crankshaft (a radial) bends outwards,
            // away from the axis; everything else sags.
            auto bulge = downAxis;
            if (port != nullptr && std::abs(port->direction.z) > 0.9F
                && std::abs(port->position.x) + std::abs(port->position.y) > 1.0F)
                bulge = normalise(Vec3 { port->position.x, port->position.y, 0.0F });
            shapeComponent(piece, node, routeWithLength(a, dirA, b, arrival, node.drawLength, bulge));
            pieces.push_back(std::move(piece));
        }
    }
    return pieces;
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

/** Throttle bores, airbox and inlet duct upstream of a plenum, in the order
    the air crosses them from the atmosphere. */
void appendUpstream(std::vector<DuctPiece>& pieces, std::uint32_t pathId, const IntakeConfig& geometry,
                    const std::vector<Vec3>& faces, Vec3 direction) {
    const auto throttle = std::max(5.0F, mm(geometry.throttleDiameterMm));
    const auto bodyLength = 1.1F * throttle;
    DuctPiece bodies;
    bodies.kind = DuctKind::intakeThrottle;
    bodies.pathId = pathId;
    bodies.nearEngine = true;
    std::vector<Vec3> mouths;
    Vec3 mouthCentre {};
    for (const auto face : faces) {
        const auto frame = Mat4::frameAlongY(face, direction);
        appendCylinder(bodies.mesh, frame, 0.62F * throttle, 0.62F * throttle, bodyLength, 28);
        appendTorus(bodies.mesh, frame * Mat4::translation({ 0.0F, 0.15F * throttle, 0.0F }), 0.66F * throttle,
                    0.06F * throttle, 28, 8);
        mouths.push_back(face + direction * bodyLength);
        mouthCentre += mouths.back();
    }
    mouthCentre = mouthCentre * (1.0F / static_cast<float>(std::max<std::size_t>(1U, mouths.size())));

    const auto airbox = mm(geometry.airboxVolumeLitres) * litre;
    const auto ductLength = mm(geometry.inletDuctLengthMm);
    const auto ductDiameter = geometry.inletDuctDiameterMm > 1.0 ? mm(geometry.inletDuctDiameterMm) : throttle;
    const auto bellmouth = geometry.bellmouthDiameterMm > 1.0 ? mm(geometry.bellmouthDiameterMm) : ductDiameter;
    if (airbox <= 0.0F && ductLength <= 1.0F && bellmouth > 1.05F * throttle) {
        // Open throttle mouths: a velocity stack on each.
        for (const auto mouth : mouths)
            appendCylinder(bodies.mesh, Mat4::frameAlongY(mouth, direction), 0.5F * throttle, 0.5F * bellmouth,
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
        appendTube(duct.mesh, duct.centreline, { 0.5F * ductDiameter }, segmentsFor(ductDiameter));
        if (bellmouth > 1.05F * ductDiameter)
            appendCylinder(duct.mesh, Mat4::frameAlongY(mouth, direction), 0.5F * ductDiameter, 0.5F * bellmouth,
                           0.35F * ductDiameter, segmentsFor(bellmouth), false, false);
        pieces.push_back(std::move(duct));
    }
}

} // namespace

std::vector<DuctPiece> layoutIntake(const EngineConfig& config, const std::vector<PortAnchor>& ports, float boreMm,
                                    bool radial) {
    std::vector<DuctPiece> pieces;
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
        DuctPiece plenum;
        plenum.kind = DuctKind::intakePlenum;
        plenum.pathId = pathId;
        plenum.nearEngine = true;
        std::vector<Vec3> faces;
        Vec3 upstream;

        if (radial) {
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
            plenum.centreline = { hub - zAxis * (0.5F * drumLength), hub + zAxis * (0.5F * drumLength) };
            faces.push_back(hub + downAxis * drum);
            upstream = downAxis;
        } else {
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
            for (const auto& runner : runners)
                widestAcross = std::max(widestAcross, std::abs(dot(runner.port->position - centre, lateral)));
            const auto residual = (1.0F - keepAcross) * widestAcross;
            const auto reach = std::sqrt(std::max(0.0625F * shortest * shortest,
                                                  0.81F * shortest * shortest - residual * residual));
            float zMin = std::numeric_limits<float>::max(), zMax = -zMin, latMin = zMin, latMax = -zMin;
            for (std::size_t i = 0; i < runners.size(); ++i) {
                const auto offset = runners[i].port->position - centre;
                const auto across = dot(offset, lateral);
                ends[i] = centre + out * reach + zAxis * offset.z + lateral * (keepAcross * across);
                arrivals[i] = out;
                bulges[i] = std::abs(across) > 0.1F * boreMm ? lateral * (across > 0.0F ? 1.0F : -1.0F) : lateral;
                zMin = std::min(zMin, ends[i].z);
                zMax = std::max(zMax, ends[i].z);
                latMin = std::min(latMin, dot(ends[i] - centre, lateral));
                latMax = std::max(latMax, dot(ends[i] - centre, lateral));
            }
            const auto depth = zMax - zMin + 1.4F * widest;
            auto width = latMax - latMin + 1.4F * widest;
            width = std::max(width, std::sqrt(volume / depth));
            const auto height = std::max(0.8F * widest, volume / (depth * width));
            const auto boxCentre = Vec3 { centre.x, centre.y, 0.5F * (zMin + zMax) } + out * (reach + 0.5F * height)
                + lateral * (0.5F * (latMin + latMax));
            addBox(plenum, lateral, out, zAxis, boxCentre, { width, height, depth });
            plenum.centreline = { boxCentre - out * (0.5F * height), boxCentre + out * (0.5F * height) };
            const auto throttles = std::max<std::uint32_t>(1U, geometry.throttleCount);
            if (throttles == 1U) {
                faces.push_back(boxCentre - zAxis * (0.5F * depth));
                upstream = -zAxis;
            } else {
                const auto spacing = depth / static_cast<float>(throttles);
                for (std::uint32_t k = 0; k < throttles; ++k)
                    faces.push_back(boxCentre + out * (0.5F * height)
                                    + zAxis * ((static_cast<float>(k) - 0.5F * static_cast<float>(throttles - 1U)) * spacing));
                upstream = out;
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
            // Gas-flow order: from the plenum to the valves.
            auto route = resample(routeWithLength(runner.port->position, runner.port->direction, ends[i], arrivals[i],
                                                  runner.lengthMm, bulges[i]), 10.0F);
            std::reverse(route.begin(), route.end());
            auto radii = radiiAlong(route, [&](float t) {
                return 0.5F * (runner.plenumDiameter + (runner.diameter - runner.plenumDiameter) * t);
            });
            // The port inside the head, behind the valves, closes the runner.
            route.push_back(runner.port->inner);
            radii.push_back(0.5F * runner.diameter);
            appendTube(piece.mesh, route, radii, 18, false, true);
            route.pop_back();
            piece.centreline = std::move(route);
            pieces.push_back(std::move(piece));
        }
        pieces.push_back(std::move(plenum));
        appendUpstream(pieces, pathId, geometry, faces, upstream);
    }
    return pieces;
}

} // namespace enginelab::render
