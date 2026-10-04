#pragma once

#include <enginelab/render/Math3D.hpp>

#include <cstdint>
#include <vector>

namespace enginelab::render {

struct MeshVertex final {
    Vec3 position;
    Vec3 normal;
};

/** Indexed triangle list. Builders append, so one mesh can hold several primitives. */
struct Mesh final {
    std::vector<MeshVertex> vertices;
    std::vector<std::uint32_t> indices;

    [[nodiscard]] bool empty() const noexcept { return indices.empty(); }
};

struct Vec2 final {
    float x {};
    float y {};
};

/** Cylinder or cone along local +Y, from y = 0 to y = height, placed by `frame`. */
void appendCylinder(Mesh&, const Mat4& frame, float bottomRadius, float topRadius, float height,
                    int segments, bool capBottom = true, bool capTop = true);
/** Axis-aligned box centred on the frame origin. */
void appendBox(Mesh&, const Mat4& frame, Vec3 size);
/** Ring around local +Y, in the local XZ plane. */
void appendTorus(Mesh&, const Mat4& frame, float ringRadius, float tubeRadius, int segments, int tubeSegments);
/** Sphere centred on the frame origin. */
void appendSphere(Mesh&, const Mat4& frame, float radius, int segments, int rings);
/** Tube swept along a polyline with parallel-transported frames. `radii` is
    either one radius for the whole tube or one radius per point. */
void appendTube(Mesh&, const std::vector<Vec3>& points, const std::vector<float>& radii,
                int radialSegments, bool capStart = false, bool capEnd = false);
/** Prism from the convex hull of `outline` (local XY), extruded from z = -depth/2
    to z = +depth/2 and placed by `frame`. */
void appendConvexExtrusion(Mesh&, const Mat4& frame, std::vector<Vec2> outline, float depth);

/** Smooth Catmull-Rom curve through `controls`, sampled `samplesPerSpan` times per span. */
[[nodiscard]] std::vector<Vec3> catmullRom(const std::vector<Vec3>& controls, int samplesPerSpan);
/** Total length of a polyline. */
[[nodiscard]] float polylineLength(const std::vector<Vec3>& points) noexcept;

} // namespace enginelab::render
