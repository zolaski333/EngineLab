#include <enginelab/render/RouteCheck.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace enginelab::render {
namespace {

struct Box final {
    Vec3 minimum { std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                   std::numeric_limits<float>::max() };
    Vec3 maximum { -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(),
                   -std::numeric_limits<float>::max() };

    void add(Vec3 p, float margin) noexcept {
        minimum = { std::min(minimum.x, p.x - margin), std::min(minimum.y, p.y - margin), std::min(minimum.z, p.z - margin) };
        maximum = { std::max(maximum.x, p.x + margin), std::max(maximum.y, p.y + margin), std::max(maximum.z, p.z + margin) };
    }
    [[nodiscard]] bool overlaps(const Box& other) const noexcept {
        return minimum.x <= other.maximum.x && other.minimum.x <= maximum.x && minimum.y <= other.maximum.y
            && other.minimum.y <= maximum.y && minimum.z <= other.maximum.z && other.minimum.z <= maximum.z;
    }
    [[nodiscard]] bool contains(Vec3 p, float margin) const noexcept {
        return p.x >= minimum.x - margin && p.x <= maximum.x + margin && p.y >= minimum.y - margin
            && p.y <= maximum.y + margin && p.z >= minimum.z - margin && p.z <= maximum.z + margin;
    }
};

struct Tube final {
    int duct {};
    const std::vector<Vec3>* points {};
    const std::vector<float>* radii {};
    std::vector<float> arc;
    Box bounds;
    /** Ends that plug into something: overlap within two radii is a joint. */
    bool attached[2] {};
};

/** Overlap of a sphere (centre `q`, radius `rq`) with tube `t`: positive
    when they overlap. Segments [skipFrom, skipTo) are left out. */
[[nodiscard]] float overlap(Vec3 q, float rq, const Tube& t, std::size_t skipFrom = 0, std::size_t skipTo = 0) {
    return tubeContact(q, rq, *t.points, t.radii, 0.0F, skipFrom, skipTo).depth;
}

/** Arc index range of `t` within `distance` of arc position `s`. */
void arcWindow(const Tube& t, float s, float distance, std::size_t& from, std::size_t& to) {
    from = 0;
    while (from < t.arc.size() && t.arc[from] < s - distance) ++from;
    to = from;
    while (to < t.arc.size() && t.arc[to] <= s + distance) ++to;
}

[[nodiscard]] bool nearAttachedEnd(const Tube& t, std::size_t i) {
    const auto& p = *t.points;
    const auto& r = *t.radii;
    if (t.attached[0] && length(p[i] - p.front()) < 2.0F * r.front()) return true;
    if (t.attached[1] && length(p[i] - p.back()) < 2.0F * r.back()) return true;
    return false;
}

[[nodiscard]] Vec3 meanPoint(const Vec3& a, const Vec3& b) noexcept { return (a + b) * 0.5F; }

} // namespace

std::size_t RouteReport::count(RouteIssueKind kind) const noexcept {
    return static_cast<std::size_t>(
        std::count_if(issues.begin(), issues.end(), [kind](const RouteIssue& issue) { return issue.kind == kind; }));
}

RouteReport checkRoutes(const EngineModel3D& model) {
    RouteReport report;
    const auto& ducts = model.ducts();

    // Boxes (plenum, airbox, throttle bodies) and ports take joints too.
    std::vector<Box> boxes;
    std::vector<Tube> tubes;
    for (std::size_t d = 0; d < ducts.size(); ++d) {
        const auto& duct = ducts[d];
        if (duct.centreline.size() >= 2U && duct.radii.size() == duct.centreline.size()) {
            Tube tube;
            tube.duct = static_cast<int>(d);
            tube.points = &duct.centreline;
            tube.radii = &duct.radii;
            tube.arc.assign(duct.centreline.size(), 0.0F);
            for (std::size_t i = 0; i < duct.centreline.size(); ++i) {
                if (i > 0) tube.arc[i] = tube.arc[i - 1U] + length(duct.centreline[i] - duct.centreline[i - 1U]);
                tube.bounds.add(duct.centreline[i], duct.radii[i]);
            }
            tubes.push_back(std::move(tube));
        } else {
            Box box;
            for (const auto& vertex : model.parts()[duct.part].mesh.vertices) box.add(vertex.position, 0.0F);
            boxes.push_back(box);
        }
    }
    std::vector<Vec3> ports;
    for (const auto& port : model.exhaustPorts()) ports.push_back(port.position);
    for (const auto& port : model.intakePorts()) ports.push_back(port.position);

    for (auto& tube : tubes) {
        for (int end = 0; end < 2; ++end) {
            const auto e = end == 0 ? tube.points->front() : tube.points->back();
            const auto re = end == 0 ? tube.radii->front() : tube.radii->back();
            bool attached = std::any_of(ports.begin(), ports.end(), [&](Vec3 port) { return length(port - e) < 2.0F * re; })
                || std::any_of(boxes.begin(), boxes.end(), [&](const Box& box) { return box.contains(e, re); });
            for (const auto& other : tubes) {
                if (attached) break;
                if (other.duct == tube.duct || !other.bounds.contains(e, re)) continue;
                attached = overlap(e, re, other) > -0.25F * re;
            }
            tube.attached[end] = attached;
        }
    }

    // Duct against duct, both ways, deepest overlap per pair.
    for (std::size_t a = 0; a < tubes.size(); ++a) {
        for (std::size_t b = a + 1U; b < tubes.size(); ++b) {
            const auto& ta = tubes[a];
            const auto& tb = tubes[b];
            if (!ta.bounds.overlaps(tb.bounds)) continue;
            RouteIssue worst { RouteIssueKind::ductClash, ta.duct, tb.duct, {}, 0.0F };
            const auto test = [&](const Tube& from, const Tube& against) {
                for (std::size_t i = 0; i < from.points->size(); ++i) {
                    if (nearAttachedEnd(from, i)) continue;
                    const auto q = (*from.points)[i];
                    const auto rq = (*from.radii)[i];
                    if (!against.bounds.contains(q, rq)) continue;
                    const auto depth = overlap(q, rq, against);
                    // A graze is not a clash.
                    if (depth > 0.5F + 0.15F * rq && depth > worst.value) {
                        worst.value = depth;
                        worst.where = q;
                    }
                }
            };
            test(ta, tb);
            test(tb, ta);
            if (worst.value > 0.0F) report.issues.push_back(worst);
        }
    }

    // A duct against itself, beyond four radii of arc.
    for (const auto& tube : tubes) {
        RouteIssue worst { RouteIssueKind::selfClash, tube.duct, -1, {}, 0.0F };
        for (std::size_t i = 0; i < tube.points->size(); ++i) {
            const auto rq = (*tube.radii)[i];
            std::size_t from = 0, to = 0;
            arcWindow(tube, tube.arc[i], 4.0F * rq, from, to);
            // Segments touching the points of the window.
            const auto depth = overlap((*tube.points)[i], rq, tube, from > 0 ? from - 1U : 0U, to);
            if (depth > 0.5F + 0.15F * rq && depth > worst.value) {
                worst.value = depth;
                worst.where = (*tube.points)[i];
            }
        }
        if (worst.value > 0.0F) report.issues.push_back(worst);
    }

    // Ducts against the engine's solids.
    const auto& solids = model.solids();
    for (const auto& tube : tubes) {
        for (std::size_t s = 0; s < solids.size(); ++s) {
            RouteIssue worst { RouteIssueKind::engineClash, tube.duct, static_cast<int>(s), {}, 0.0F };
            for (std::size_t i = 0; i < tube.points->size(); ++i) {
                if (nearAttachedEnd(tube, i)) continue;
                const auto rq = (*tube.radii)[i];
                const auto depth = rq - signedDistance(solids[s], (*tube.points)[i]);
                if (depth > 0.5F + 0.15F * rq && depth > worst.value) {
                    worst.value = depth;
                    worst.where = (*tube.points)[i];
                }
            }
            if (worst.value > 0.0F) report.issues.push_back(worst);
        }
    }

    // Bends: the circle through three points half a diameter apart along the
    // centreline must have a radius of at least one diameter.
    for (const auto& tube : tubes) {
        const auto& p = *tube.points;
        RouteIssue run { RouteIssueKind::tightBend, tube.duct, -1, {}, std::numeric_limits<float>::max() };
        bool open = false;
        for (std::size_t i = 1; i + 1U < p.size(); ++i) {
            const auto diameter = 2.0F * (*tube.radii)[i];
            std::size_t before = i, after = i;
            while (before > 0 && tube.arc[i] - tube.arc[before] < 0.5F * diameter) --before;
            while (after + 1U < p.size() && tube.arc[after] - tube.arc[i] < 0.5F * diameter) ++after;
            const auto ab = p[i] - p[before], bc = p[after] - p[i], ca = p[before] - p[after];
            const auto area2 = length(cross(ab, bc));
            const auto ratio = area2 < 1.0e-6F ? std::numeric_limits<float>::max()
                : length(ab) * length(bc) * length(ca) / (2.0F * area2) / diameter;
            const auto tight = ratio < 1.0F && tube.arc[i] - tube.arc[before] >= 0.45F * diameter
                && tube.arc[after] - tube.arc[i] >= 0.45F * diameter;
            if (tight) {
                open = true;
                if (ratio < run.value) {
                    run.value = ratio;
                    run.where = p[i];
                }
            } else if (open) {
                report.issues.push_back(run);
                run.value = std::numeric_limits<float>::max();
                open = false;
            }
        }
        if (open) report.issues.push_back(run);
    }

    // Pipes drawn longer than authored.
    for (std::size_t d = 0; d < ducts.size(); ++d) {
        const auto& duct = ducts[d];
        const auto pipe = (duct.kind == DuctKind::exhaustComponent && duct.componentType == ExhaustComponentType::pipe)
            || duct.kind == DuctKind::intakeRunner;
        if (!pipe || duct.authoredLengthMm <= 1.0F) continue;
        const auto drawn = polylineLength(duct.centreline);
        if (drawn > 1.03F * duct.authoredLengthMm)
            report.issues.push_back({ RouteIssueKind::stretched, static_cast<int>(d), -1,
                                      meanPoint(duct.centreline.front(), duct.centreline.back()),
                                      drawn / duct.authoredLengthMm });
    }
    return report;
}

} // namespace enginelab::render
