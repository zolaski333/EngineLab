#include <enginelab/render/RouteSolver.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace enginelab::render {

float signedDistance(const EngineSolid& solid, Vec3 point) noexcept {
    const auto offset = point - solid.centre;
    float qx = 0.0F, qy = 0.0F, qz = -1.0e9F;
    if (solid.shape == EngineSolid::Shape::box) {
        qx = std::abs(dot(offset, solid.axes[0])) - solid.half.x;
        qy = std::abs(dot(offset, solid.axes[1])) - solid.half.y;
        qz = std::abs(dot(offset, solid.axes[2])) - solid.half.z;
    } else {
        const auto along = dot(offset, solid.axes[1]);
        qx = length(offset - solid.axes[1] * along) - solid.half.x;
        qy = std::abs(along) - solid.half.y;
    }
    const auto outside = std::sqrt(std::max(qx, 0.0F) * std::max(qx, 0.0F) + std::max(qy, 0.0F) * std::max(qy, 0.0F)
                                   + std::max(qz, 0.0F) * std::max(qz, 0.0F));
    return outside + std::min(std::max({ qx, qy, qz }), 0.0F);
}

EngineSolid boxBetween(Vec3 minimum, Vec3 maximum) noexcept {
    EngineSolid solid;
    solid.centre = (minimum + maximum) * 0.5F;
    solid.half = (maximum - minimum) * 0.5F;
    return solid;
}

namespace {

/** Any unit vector square to `d`. */
[[nodiscard]] Vec3 across(Vec3 d) noexcept {
    const auto c = cross(d, Vec3 { 0.0F, 1.0F, 0.0F });
    return length(c) > 1.0e-3F * length(d) ? normalise(c) : normalise(cross(d, Vec3 { 1.0F, 0.0F, 0.0F }));
}

/** Overlap with the flat disc closing a tube end at `centre`, facing out
    along `outward`, for a sphere on its outer side. */
[[nodiscard]] TubeContact disc(Vec3 q, float reach, Vec3 centre, Vec3 outward, float radius) noexcept {
    const auto v = q - centre;
    const auto along = dot(v, outward);
    const auto radial = v - outward * along;
    const auto byAxis = reach - along;
    const auto byRadius = radius + reach - length(radial);
    if (byAxis < byRadius) return { byAxis, outward };
    return { byRadius, length(radial) > 1.0e-4F ? normalise(radial) : across(outward) };
}

} // namespace

TubeContact tubeContact(Vec3 q, float reach, const std::vector<Vec3>& p, const std::vector<float>* radii, float radius,
                        std::size_t skipFrom, std::size_t skipTo) {
    TubeContact deepest { -std::numeric_limits<float>::max(), {} };
    const auto n = p.size();
    if (n < 2U) return deepest;
    const auto radiusAt = [&](std::size_t k) { return radii == nullptr ? radius : (*radii)[k]; };
    const auto skipped = [&](std::size_t k) { return k >= skipFrom && k < skipTo; };
    // Behind an open end only the end disc is tube: the swept segments next
    // to it would otherwise bulge out past the end plane by their radius.
    const auto startAxis = normalise(p[1] - p[0]);
    const auto endAxis = normalise(p[n - 1U] - p[n - 2U]);
    const auto behindStart = !skipped(0) && dot(q - p[0], startAxis) < 0.0F;
    const auto pastEnd = !skipped(n - 2U) && dot(q - p[n - 1U], endAxis) > 0.0F;
    if (behindStart) deepest = disc(q, reach, p[0], -startAxis, radiusAt(0));
    if (pastEnd) {
        const auto end = disc(q, reach, p[n - 1U], endAxis, radiusAt(n - 1U));
        if (end.depth > deepest.depth) deepest = end;
    }
    const auto total = (behindStart || pastEnd) ? polylineLength(p) : 0.0F;
    float arc = 0.0F;
    for (std::size_t k = 0; k + 1U < n; ++k) {
        const auto d = p[k + 1U] - p[k];
        const auto span = length(d);
        const auto from = arc;
        arc += span;
        if (skipped(k) || span < 1.0e-4F) continue;
        const auto clear = 2.0F * (std::max(radiusAt(k), radiusAt(k + 1U)) + reach);
        if ((behindStart && from < clear) || (pastEnd && total - arc < clear)) continue;
        const auto s = std::clamp(dot(q - p[k], d) / (span * span), 0.0F, 1.0F);
        const auto c = p[k] + d * s;
        const auto rb = radiusAt(k) + (radiusAt(k + 1U) - radiusAt(k)) * s;
        const auto v = q - c;
        const auto distance = length(v);
        const TubeContact found { reach + rb - distance, distance > 1.0e-4F ? v * (1.0F / distance) : across(d) };
        if (found.depth > deepest.depth) deepest = found;
    }
    return deepest;
}

namespace {

struct Bounds final {
    Vec3 minimum { std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                   std::numeric_limits<float>::max() };
    Vec3 maximum { -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(),
                   -std::numeric_limits<float>::max() };

    void add(Vec3 p, float margin) noexcept {
        minimum = { std::min(minimum.x, p.x - margin), std::min(minimum.y, p.y - margin), std::min(minimum.z, p.z - margin) };
        maximum = { std::max(maximum.x, p.x + margin), std::max(maximum.y, p.y + margin), std::max(maximum.z, p.z + margin) };
    }
    [[nodiscard]] bool contains(Vec3 p, float margin) const noexcept {
        return p.x >= minimum.x - margin && p.x <= maximum.x + margin && p.y >= minimum.y - margin
            && p.y <= maximum.y + margin && p.z >= minimum.z - margin && p.z <= maximum.z + margin;
    }
};

[[nodiscard]] Vec3 gradient(const EngineSolid& solid, Vec3 q) noexcept {
    constexpr float h = 0.5F;
    const Vec3 g { signedDistance(solid, q + Vec3 { h, 0, 0 }) - signedDistance(solid, q - Vec3 { h, 0, 0 }),
                   signedDistance(solid, q + Vec3 { 0, h, 0 }) - signedDistance(solid, q - Vec3 { 0, h, 0 }),
                   signedDistance(solid, q + Vec3 { 0, 0, h }) - signedDistance(solid, q - Vec3 { 0, 0, h }) };
    return length(g) > 1.0e-6F ? normalise(g) : normalise(q - solid.centre);
}

/** Point at arc fraction `t` of a polyline. */
[[nodiscard]] Vec3 alongPolyline(const std::vector<Vec3>& p, float total, float t) {
    auto remaining = std::clamp(t, 0.0F, 1.0F) * total;
    for (std::size_t k = 0; k + 1U < p.size(); ++k) {
        const auto span = length(p[k + 1U] - p[k]);
        if (remaining <= span && span > 0.0F) return lerp(p[k], p[k + 1U], remaining / span);
        remaining -= span;
    }
    return p.back();
}

struct Chain final {
    RoutedPipe* pipe {};
    std::vector<Vec3> p;
    /** First and last free points. */
    std::size_t firstFree {};
    std::size_t lastFree {};
    float segment {};
    Bounds bounds;
};

[[nodiscard]] float inverseMass(const Chain& c, std::size_t i) noexcept {
    return i >= c.firstFree && i <= c.lastFree ? 1.0F : 0.0F;
}

void keepLengths(Chain& c, int sweeps) {
    const auto n = c.p.size();
    const auto solve = [&c](std::size_t k) {
        const auto wa = inverseMass(c, k), wb = inverseMass(c, k + 1U);
        if (wa + wb <= 0.0F) return;
        const auto d = c.p[k + 1U] - c.p[k];
        const auto distance = length(d);
        if (distance < 1.0e-6F) return;
        const auto correction = d * ((distance - c.segment) / (distance * (wa + wb)));
        c.p[k] += correction * wa;
        c.p[k + 1U] += correction * -wb;
    };
    for (int sweep = 0; sweep < sweeps; ++sweep) {
        for (std::size_t k = 0; k + 1U < n; ++k) solve(k);
        for (std::size_t k = n - 1U; k-- > 0;) solve(k);
    }
}

/** A vertex may sit at most `sag` off the midpoint of its neighbours: the
    sagitta of the tightest allowed arc over two segments. */
void keepBends(Chain& c, float sag) {
    for (std::size_t i = 1; i + 1U < c.p.size(); ++i) {
        const auto wa = inverseMass(c, i - 1U), wi = inverseMass(c, i), wb = inverseMass(c, i + 1U);
        const auto weight = wi + 0.25F * (wa + wb);
        if (weight <= 0.0F) continue;
        const auto offset = c.p[i] - (c.p[i - 1U] + c.p[i + 1U]) * 0.5F;
        const auto distance = length(offset);
        if (distance <= sag) continue;
        const auto normal = offset * (1.0F / distance);
        const auto lambda = (distance - sag) / weight;
        c.p[i] += normal * (-wi * lambda);
        c.p[i - 1U] += normal * (0.5F * wa * lambda);
        c.p[i + 1U] += normal * (0.5F * wb * lambda);
    }
}

[[nodiscard]] float sagFor(const Chain& c, const RouteSettings& settings) noexcept {
    const auto bend = settings.minimumBendDiameters * 2.0F * c.pipe->radius;
    return c.segment * c.segment / (2.0F * bend);
}

/** Bend and length passes between two contact passes: cheap, and they carry
    a correction further along the pipe than one pass would. */
void shape(Chain& c, const RouteSettings& settings) {
    const auto sag = sagFor(c, settings);
    for (int pass = 0; pass < settings.shapePasses; ++pass) {
        keepBends(c, sag);
        keepLengths(c, 1);
    }
}

} // namespace

void relaxRoutes(std::vector<RoutedPipe>& pipes, const std::vector<RouteTube>& obstacles,
                 const std::vector<EngineSolid>& solids, const RouteSettings& settings) {
    const auto gap = settings.gapMm;
    // Fixed tubes, plus pipes too short to bend, which stay as obstacles.
    std::vector<RouteTube> fixed = obstacles;
    std::vector<Chain> chains;
    for (auto& pipe : pipes) {
        if (pipe.points.size() < 2U || pipe.radius <= 0.0F) continue;
        const auto start = pipe.points.front(), end = pipe.points.back();
        // A pipe too short for its span is bent at the shortest length that
        // leaves room for its turns; it shows as drawn longer than authored.
        const auto reach = pipe.startLead + pipe.endLead
            + length(end - pipe.endDirection * pipe.endLead - (start + pipe.startDirection * pipe.startLead));
        if (pipe.lengthMm < 1.02F * reach) pipe.lengthMm = 1.1F * reach;
        const auto step = std::clamp(0.5F * pipe.radius, 6.0F, 14.0F);
        const auto segments = std::clamp(static_cast<int>(std::ceil(pipe.lengthMm / step)), 8, 240);
        Chain c;
        c.pipe = &pipe;
        c.segment = pipe.lengthMm / static_cast<float>(segments);
        const auto n = static_cast<std::size_t>(segments) + 1U;
        const auto head = std::max<std::size_t>(2U, 1U + static_cast<std::size_t>(pipe.startLead / c.segment));
        const auto tail = std::max<std::size_t>(2U, 1U + static_cast<std::size_t>(pipe.endLead / c.segment));
        const auto freeSpan = start + pipe.startDirection * (static_cast<float>(head - 1U) * c.segment)
            - (end - pipe.endDirection * (static_cast<float>(tail - 1U) * c.segment));
        const auto freeSegments = static_cast<float>(n - head - tail + 1U);
        if (head + tail + 2U > n || length(freeSpan) > 0.98F * freeSegments * c.segment) {
            RouteTube tube;
            tube.points = pipe.points;
            tube.radii.assign(pipe.points.size(), pipe.radius);
            fixed.push_back(std::move(tube));
            continue;
        }
        c.firstFree = head;
        c.lastFree = n - 1U - tail;
        const auto guess = polylineLength(pipe.points);
        c.p.resize(n);
        for (std::size_t i = 0; i < n; ++i) {
            if (i < head) c.p[i] = start + pipe.startDirection * (static_cast<float>(i) * c.segment);
            else if (i > c.lastFree) c.p[i] = end - pipe.endDirection * (static_cast<float>(n - 1U - i) * c.segment);
            else c.p[i] = alongPolyline(pipe.points, guess, static_cast<float>(i) / static_cast<float>(n - 1U));
        }
        chains.push_back(std::move(c));
    }
    std::vector<Bounds> fixedBounds(fixed.size());
    for (std::size_t o = 0; o < fixed.size(); ++o)
        for (std::size_t i = 0; i < fixed[o].points.size(); ++i) fixedBounds[o].add(fixed[o].points[i], fixed[o].radii[i]);

    std::vector<Vec3> before;
    for (int iteration = 0; iteration < settings.iterations; ++iteration) {
        before.clear();
        for (const auto& c : chains) before.insert(before.end(), c.p.begin(), c.p.end());
        for (auto& c : chains) {
            c.bounds = {};
            for (const auto& point : c.p) c.bounds.add(point, c.pipe->radius);
        }
        for (std::size_t a = 0; a < chains.size(); ++a) {
            auto& c = chains[a];
            const auto reach = c.pipe->radius + gap;
            const auto window = static_cast<std::size_t>(std::ceil(4.0F * c.pipe->radius / c.segment));
            for (auto i = c.firstFree; i <= c.lastFree; ++i) {
                auto& q = c.p[i];
                for (std::size_t o = 0; o < fixed.size(); ++o) {
                    if (!fixedBounds[o].contains(q, reach)) continue;
                    const auto hit = tubeContact(q, reach, fixed[o].points, &fixed[o].radii);
                    if (hit.depth > 0.0F) q += hit.normal * hit.depth;
                }
                for (std::size_t b = 0; b < chains.size(); ++b) {
                    const auto& other = chains[b];
                    if (b == a) {
                        const auto from = i > window ? i - window : 0U;
                        const auto hit = tubeContact(q, reach, other.p, nullptr, other.pipe->radius, from, i + window);
                        if (hit.depth > 0.0F) q += hit.normal * (0.5F * hit.depth);
                        continue;
                    }
                    if (!other.bounds.contains(q, reach)) continue;
                    const auto hit = tubeContact(q, reach, other.p, nullptr, other.pipe->radius);
                    if (hit.depth > 0.0F) q += hit.normal * (0.5F * hit.depth);
                }
                for (const auto& solid : solids) {
                    const auto distance = signedDistance(solid, q);
                    if (distance < reach) q += gradient(solid, q) * (reach - distance);
                }
            }
            shape(c, settings);
        }
        // Settled: nothing moved by more than a hundredth of a millimetre.
        float moved = 0.0F;
        std::size_t k = 0;
        for (const auto& c : chains)
            for (const auto& point : c.p) moved = std::max(moved, length(point - before[k++]));
        if (moved < 0.01F) break;
    }
    for (auto& c : chains) {
        keepLengths(c, 3);
        c.pipe->points = std::move(c.p);
    }
}

} // namespace enginelab::render
