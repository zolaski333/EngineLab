#include <enginelab/render/Mesh.hpp>

#include <algorithm>
#include <numbers>

namespace enginelab::render {
namespace {
constexpr float twoPi = 2.0F * std::numbers::pi_v<float>;

[[nodiscard]] std::uint32_t addVertex(Mesh& mesh, const Mat4& frame, Vec3 position, Vec3 normal) {
    mesh.vertices.push_back({ frame.transformPoint(position), normalise(frame.transformDirection(normal)) });
    return static_cast<std::uint32_t>(mesh.vertices.size() - 1U);
}

/** Adds a triangle wound counter-clockwise when seen from the side its
    vertex normals point to, so no builder can produce an inside-out face. */
void addTriangle(Mesh& mesh, std::uint32_t a, std::uint32_t b, std::uint32_t c) {
    const auto& pa = mesh.vertices[a];
    const auto& pb = mesh.vertices[b];
    const auto& pc = mesh.vertices[c];
    const auto faceNormal = cross(pb.position - pa.position, pc.position - pa.position);
    const auto averageNormal = pa.normal + pb.normal + pc.normal;
    if (dot(faceNormal, averageNormal) < 0.0F) std::swap(b, c);
    mesh.indices.insert(mesh.indices.end(), { a, b, c });
}

void addQuad(Mesh& mesh, std::uint32_t a, std::uint32_t b, std::uint32_t c, std::uint32_t d) {
    addTriangle(mesh, a, b, c);
    addTriangle(mesh, a, c, d);
}

[[nodiscard]] float angleOf(int step, int segments) noexcept {
    return twoPi * static_cast<float>(step) / static_cast<float>(segments);
}

void appendDisc(Mesh& mesh, const Mat4& frame, float y, float radius, int segments, float normalY) {
    const auto centre = addVertex(mesh, frame, { 0.0F, y, 0.0F }, { 0.0F, normalY, 0.0F });
    const auto first = static_cast<std::uint32_t>(mesh.vertices.size());
    for (int i = 0; i < segments; ++i) {
        const auto a = angleOf(i, segments);
        (void)addVertex(mesh, frame, { radius * std::cos(a), y, radius * std::sin(a) }, { 0.0F, normalY, 0.0F });
    }
    const auto count = static_cast<std::uint32_t>(segments);
    for (std::uint32_t i = 0; i < count; ++i)
        addTriangle(mesh, centre, first + i, first + (i + 1U) % count);
}

[[nodiscard]] Vec3 anyPerpendicular(Vec3 axis) noexcept {
    const auto helper = std::abs(axis.y) < 0.9F ? Vec3 { 0.0F, 1.0F, 0.0F } : Vec3 { 1.0F, 0.0F, 0.0F };
    return normalise(cross(axis, helper));
}
} // namespace

void appendCylinder(Mesh& mesh, const Mat4& frame, float bottomRadius, float topRadius, float height,
                    int segments, bool capBottom, bool capTop) {
    segments = std::max(3, segments);
    const auto slope = (bottomRadius - topRadius) / std::max(1.0e-6F, height);
    const auto first = static_cast<std::uint32_t>(mesh.vertices.size());
    for (int i = 0; i <= segments; ++i) {
        const auto a = angleOf(i, segments);
        const auto c = std::cos(a), s = std::sin(a);
        const Vec3 normal { c, slope, s };
        (void)addVertex(mesh, frame, { bottomRadius * c, 0.0F, bottomRadius * s }, normal);
        (void)addVertex(mesh, frame, { topRadius * c, height, topRadius * s }, normal);
    }
    for (std::uint32_t i = 0; i < static_cast<std::uint32_t>(segments); ++i) {
        const auto b0 = first + i * 2U, t0 = b0 + 1U, b1 = b0 + 2U, t1 = b0 + 3U;
        addQuad(mesh, b0, t0, t1, b1);
    }
    if (capBottom && bottomRadius > 0.0F) appendDisc(mesh, frame, 0.0F, bottomRadius, segments, -1.0F);
    if (capTop && topRadius > 0.0F) appendDisc(mesh, frame, height, topRadius, segments, 1.0F);
}

void appendBox(Mesh& mesh, const Mat4& frame, Vec3 size) {
    const auto h = size * 0.5F;
    const std::array<Vec3, 6> normals {{ { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 } }};
    for (const auto n : normals) {
        // Two in-plane axes of the face.
        const auto u = std::abs(n.x) > 0.5F ? Vec3 { 0, 1, 0 } : Vec3 { 1, 0, 0 };
        const auto v = cross(n, u);
        const auto centre = Vec3 { n.x * h.x, n.y * h.y, n.z * h.z };
        const auto du = Vec3 { u.x * h.x, u.y * h.y, u.z * h.z };
        const auto dv = Vec3 { v.x * h.x, v.y * h.y, v.z * h.z };
        const auto a = addVertex(mesh, frame, centre - du - dv, n);
        const auto b = addVertex(mesh, frame, centre + du - dv, n);
        const auto c = addVertex(mesh, frame, centre + du + dv, n);
        const auto d = addVertex(mesh, frame, centre - du + dv, n);
        addQuad(mesh, a, b, c, d);
    }
}

void appendTorus(Mesh& mesh, const Mat4& frame, float ringRadius, float tubeRadius, int segments, int tubeSegments) {
    segments = std::max(3, segments);
    tubeSegments = std::max(3, tubeSegments);
    const auto first = static_cast<std::uint32_t>(mesh.vertices.size());
    for (int i = 0; i <= segments; ++i) {
        const auto u = angleOf(i, segments);
        for (int j = 0; j <= tubeSegments; ++j) {
            const auto v = angleOf(j, tubeSegments);
            const Vec3 normal { std::cos(v) * std::cos(u), std::sin(v), std::cos(v) * std::sin(u) };
            const auto r = ringRadius + tubeRadius * std::cos(v);
            (void)addVertex(mesh, frame, { r * std::cos(u), tubeRadius * std::sin(v), r * std::sin(u) }, normal);
        }
    }
    const auto stride = static_cast<std::uint32_t>(tubeSegments + 1);
    for (std::uint32_t i = 0; i < static_cast<std::uint32_t>(segments); ++i)
        for (std::uint32_t j = 0; j < static_cast<std::uint32_t>(tubeSegments); ++j) {
            const auto a = first + i * stride + j;
            addQuad(mesh, a, a + stride, a + stride + 1U, a + 1U);
        }
}

void appendSphere(Mesh& mesh, const Mat4& frame, float radius, int segments, int rings) {
    segments = std::max(3, segments);
    rings = std::max(2, rings);
    const auto first = static_cast<std::uint32_t>(mesh.vertices.size());
    for (int i = 0; i <= rings; ++i) {
        const auto phi = std::numbers::pi_v<float> * static_cast<float>(i) / static_cast<float>(rings);
        for (int j = 0; j <= segments; ++j) {
            const auto theta = angleOf(j, segments);
            const Vec3 normal { std::sin(phi) * std::cos(theta), std::cos(phi), std::sin(phi) * std::sin(theta) };
            (void)addVertex(mesh, frame, normal * radius, normal);
        }
    }
    const auto stride = static_cast<std::uint32_t>(segments + 1);
    for (std::uint32_t i = 0; i < static_cast<std::uint32_t>(rings); ++i)
        for (std::uint32_t j = 0; j < static_cast<std::uint32_t>(segments); ++j) {
            const auto a = first + i * stride + j;
            // The pole rows collapse to a point: skip their degenerate halves.
            if (i != 0) addTriangle(mesh, a, a + stride, a + 1U);
            if (i + 1U != static_cast<std::uint32_t>(rings)) addTriangle(mesh, a + 1U, a + stride, a + stride + 1U);
        }
}

void appendTube(Mesh& mesh, const std::vector<Vec3>& points, const std::vector<float>& radii,
                int radialSegments, bool capStart, bool capEnd) {
    if (points.size() < 2U || radii.empty()) return;
    radialSegments = std::max(3, radialSegments);
    const auto radiusAt = [&radii](std::size_t i) { return radii.size() == 1U ? radii.front() : radii[std::min(i, radii.size() - 1U)]; };
    const auto first = static_cast<std::uint32_t>(mesh.vertices.size());
    Vec3 normal {};
    std::vector<Vec3> tangents(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
        const auto& previous = points[i == 0 ? 0 : i - 1U];
        const auto& next = points[std::min(i + 1U, points.size() - 1U)];
        tangents[i] = normalise(next - previous);
    }
    const Mat4 identity;
    for (std::size_t i = 0; i < points.size(); ++i) {
        const auto t = tangents[i];
        // Parallel transport: carry the previous ring's normal along so the
        // tube does not twist where the path bends.
        normal = i == 0 ? anyPerpendicular(t) : normalise(normal - t * dot(normal, t));
        const auto binormal = cross(t, normal);
        for (int j = 0; j <= radialSegments; ++j) {
            const auto a = angleOf(j, radialSegments);
            const auto direction = normal * std::cos(a) + binormal * std::sin(a);
            (void)addVertex(mesh, identity, points[i] + direction * radiusAt(i), direction);
        }
    }
    const auto stride = static_cast<std::uint32_t>(radialSegments + 1);
    for (std::uint32_t i = 0; i + 1U < static_cast<std::uint32_t>(points.size()); ++i)
        for (std::uint32_t j = 0; j < static_cast<std::uint32_t>(radialSegments); ++j) {
            const auto a = first + i * stride + j;
            addQuad(mesh, a, a + stride, a + stride + 1U, a + 1U);
        }
    const auto cap = [&](std::size_t index, float sign) {
        const auto t = tangents[index] * sign;
        const auto centre = addVertex(mesh, identity, points[index], t);
        const auto ring = static_cast<std::uint32_t>(mesh.vertices.size());
        const auto ringStart = first + static_cast<std::uint32_t>(index) * stride;
        for (std::uint32_t j = 0; j < static_cast<std::uint32_t>(radialSegments); ++j)
            (void)addVertex(mesh, identity, mesh.vertices[ringStart + j].position, t);
        for (std::uint32_t j = 0; j < static_cast<std::uint32_t>(radialSegments); ++j)
            addTriangle(mesh, centre, ring + j, ring + (j + 1U) % static_cast<std::uint32_t>(radialSegments));
    };
    if (capStart) cap(0, -1.0F);
    if (capEnd) cap(points.size() - 1U, 1.0F);
}

void appendConvexExtrusion(Mesh& mesh, const Mat4& frame, std::vector<Vec2> outline, float depth) {
    // Andrew's monotone chain gives the hull counter-clockwise.
    std::sort(outline.begin(), outline.end(), [](Vec2 a, Vec2 b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });
    const auto turn = [](Vec2 o, Vec2 a, Vec2 b) { return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x); };
    std::vector<Vec2> hull;
    for (int pass = 0; pass < 2; ++pass) {
        const auto start = hull.size();
        for (std::size_t k = 0; k < outline.size(); ++k) {
            const auto p = pass == 0 ? outline[k] : outline[outline.size() - 1U - k];
            while (hull.size() >= start + 2U && turn(hull[hull.size() - 2U], hull.back(), p) <= 0.0F) hull.pop_back();
            hull.push_back(p);
        }
        hull.pop_back();
    }
    if (hull.size() < 3U) return;
    const auto half = depth * 0.5F;
    for (const auto side : { -1.0F, 1.0F }) {
        const Vec3 n { 0.0F, 0.0F, side };
        const auto first = static_cast<std::uint32_t>(mesh.vertices.size());
        for (const auto p : hull) (void)addVertex(mesh, frame, { p.x, p.y, side * half }, n);
        for (std::uint32_t i = 1; i + 1U < static_cast<std::uint32_t>(hull.size()); ++i)
            addTriangle(mesh, first, first + i, first + i + 1U);
    }
    for (std::size_t i = 0; i < hull.size(); ++i) {
        const auto a = hull[i];
        const auto b = hull[(i + 1U) % hull.size()];
        const auto n = normalise(Vec3 { b.y - a.y, a.x - b.x, 0.0F });
        const auto v0 = addVertex(mesh, frame, { a.x, a.y, -half }, n);
        const auto v1 = addVertex(mesh, frame, { b.x, b.y, -half }, n);
        const auto v2 = addVertex(mesh, frame, { b.x, b.y, half }, n);
        const auto v3 = addVertex(mesh, frame, { a.x, a.y, half }, n);
        addQuad(mesh, v0, v1, v2, v3);
    }
}

std::vector<Vec3> catmullRom(const std::vector<Vec3>& controls, int samplesPerSpan) {
    if (controls.size() < 2U) return controls;
    samplesPerSpan = std::max(1, samplesPerSpan);
    std::vector<Vec3> result;
    const auto at = [&controls](std::ptrdiff_t i) {
        return controls[static_cast<std::size_t>(std::clamp<std::ptrdiff_t>(i, 0, static_cast<std::ptrdiff_t>(controls.size()) - 1))];
    };
    for (std::ptrdiff_t span = 0; span + 1 < static_cast<std::ptrdiff_t>(controls.size()); ++span) {
        const auto p0 = at(span - 1), p1 = at(span), p2 = at(span + 1), p3 = at(span + 2);
        for (int s = 0; s < samplesPerSpan; ++s) {
            const auto t = static_cast<float>(s) / static_cast<float>(samplesPerSpan);
            const auto t2 = t * t, t3 = t2 * t;
            result.push_back((p1 * 2.0F + (p2 - p0) * t + (p0 * 2.0F - p1 * 5.0F + p2 * 4.0F - p3) * t2
                              + (p1 * 3.0F - p0 - p2 * 3.0F + p3) * t3) * 0.5F);
        }
    }
    result.push_back(controls.back());
    return result;
}

float polylineLength(const std::vector<Vec3>& points) noexcept {
    float total = 0.0F;
    for (std::size_t i = 1; i < points.size(); ++i) total += length(points[i] - points[i - 1U]);
    return total;
}

} // namespace enginelab::render
