#pragma once

#include <array>
#include <cmath>

namespace enginelab::render {

/** Single-precision vector for scene geometry, in millimetres. */
struct Vec3 final {
    float x {};
    float y {};
    float z {};

    friend constexpr Vec3 operator+(Vec3 a, Vec3 b) noexcept { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
    friend constexpr Vec3 operator-(Vec3 a, Vec3 b) noexcept { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
    friend constexpr Vec3 operator*(Vec3 a, float s) noexcept { return { a.x * s, a.y * s, a.z * s }; }
    friend constexpr Vec3 operator*(float s, Vec3 a) noexcept { return a * s; }
    friend constexpr Vec3 operator-(Vec3 a) noexcept { return { -a.x, -a.y, -a.z }; }
    constexpr Vec3& operator+=(Vec3 b) noexcept { x += b.x; y += b.y; z += b.z; return *this; }
};

[[nodiscard]] constexpr float dot(Vec3 a, Vec3 b) noexcept { return a.x * b.x + a.y * b.y + a.z * b.z; }
[[nodiscard]] constexpr Vec3 cross(Vec3 a, Vec3 b) noexcept {
    return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}
[[nodiscard]] inline float length(Vec3 a) noexcept { return std::sqrt(dot(a, a)); }
[[nodiscard]] inline Vec3 normalise(Vec3 a) noexcept {
    const auto l = length(a);
    return l > 1.0e-12F ? a * (1.0F / l) : Vec3 { 0.0F, 1.0F, 0.0F };
}
[[nodiscard]] constexpr Vec3 lerp(Vec3 a, Vec3 b, float t) noexcept { return a + (b - a) * t; }

/** Column-major 4x4 matrix, laid out as OpenGL expects it. */
struct Mat4 final {
    std::array<float, 16> m { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

    [[nodiscard]] constexpr float& at(int row, int column) noexcept { return m[static_cast<std::size_t>(column * 4 + row)]; }
    [[nodiscard]] constexpr float at(int row, int column) const noexcept { return m[static_cast<std::size_t>(column * 4 + row)]; }

    [[nodiscard]] static constexpr Mat4 identity() noexcept { return {}; }
    [[nodiscard]] static constexpr Mat4 translation(Vec3 t) noexcept {
        Mat4 r;
        r.at(0, 3) = t.x; r.at(1, 3) = t.y; r.at(2, 3) = t.z;
        return r;
    }
    [[nodiscard]] static Mat4 rotationZ(float radians) noexcept {
        Mat4 r;
        const auto c = std::cos(radians), s = std::sin(radians);
        r.at(0, 0) = c; r.at(0, 1) = -s; r.at(1, 0) = s; r.at(1, 1) = c;
        return r;
    }
    [[nodiscard]] static Mat4 rotationX(float radians) noexcept {
        Mat4 r;
        const auto c = std::cos(radians), s = std::sin(radians);
        r.at(1, 1) = c; r.at(1, 2) = -s; r.at(2, 1) = s; r.at(2, 2) = c;
        return r;
    }
    /** Rigid frame whose local +Y runs along `axis`, placed at `origin`. */
    [[nodiscard]] static Mat4 frameAlongY(Vec3 origin, Vec3 axis) noexcept {
        const auto y = normalise(axis);
        // Any vector not parallel to the axis gives a stable perpendicular.
        const auto helper = std::abs(y.z) < 0.9F ? Vec3 { 0, 0, 1 } : Vec3 { 1, 0, 0 };
        const auto x = normalise(cross(y, helper));
        const auto z = cross(x, y);
        return basis(x, y, z, origin);
    }
    /** Rigid frame from three orthonormal axes and an origin. */
    [[nodiscard]] static constexpr Mat4 basis(Vec3 x, Vec3 y, Vec3 z, Vec3 origin) noexcept {
        Mat4 r;
        r.at(0, 0) = x.x; r.at(1, 0) = x.y; r.at(2, 0) = x.z;
        r.at(0, 1) = y.x; r.at(1, 1) = y.y; r.at(2, 1) = y.z;
        r.at(0, 2) = z.x; r.at(1, 2) = z.y; r.at(2, 2) = z.z;
        r.at(0, 3) = origin.x; r.at(1, 3) = origin.y; r.at(2, 3) = origin.z;
        return r;
    }
    [[nodiscard]] static Mat4 perspective(float verticalFovRadians, float aspect, float near, float far) noexcept {
        Mat4 r;
        const auto f = 1.0F / std::tan(verticalFovRadians * 0.5F);
        r.at(0, 0) = f / aspect; r.at(1, 1) = f;
        r.at(2, 2) = (far + near) / (near - far); r.at(2, 3) = 2.0F * far * near / (near - far);
        r.at(3, 2) = -1.0F; r.at(3, 3) = 0.0F;
        return r;
    }
    [[nodiscard]] static Mat4 lookAt(Vec3 eye, Vec3 target, Vec3 up) noexcept {
        const auto f = normalise(target - eye);
        const auto s = normalise(cross(f, up));
        const auto u = cross(s, f);
        Mat4 r;
        r.at(0, 0) = s.x; r.at(0, 1) = s.y; r.at(0, 2) = s.z; r.at(0, 3) = -dot(s, eye);
        r.at(1, 0) = u.x; r.at(1, 1) = u.y; r.at(1, 2) = u.z; r.at(1, 3) = -dot(u, eye);
        r.at(2, 0) = -f.x; r.at(2, 1) = -f.y; r.at(2, 2) = -f.z; r.at(2, 3) = dot(f, eye);
        return r;
    }

    friend constexpr Mat4 operator*(const Mat4& a, const Mat4& b) noexcept {
        Mat4 r;
        for (int row = 0; row < 4; ++row)
            for (int column = 0; column < 4; ++column) {
                float sum = 0.0F;
                for (int k = 0; k < 4; ++k) sum += a.at(row, k) * b.at(k, column);
                r.at(row, column) = sum;
            }
        return r;
    }
    [[nodiscard]] constexpr Vec3 transformPoint(Vec3 p) const noexcept {
        return { at(0, 0) * p.x + at(0, 1) * p.y + at(0, 2) * p.z + at(0, 3),
                 at(1, 0) * p.x + at(1, 1) * p.y + at(1, 2) * p.z + at(1, 3),
                 at(2, 0) * p.x + at(2, 1) * p.y + at(2, 2) * p.z + at(2, 3) };
    }
    [[nodiscard]] constexpr Vec3 transformDirection(Vec3 d) const noexcept {
        return { at(0, 0) * d.x + at(0, 1) * d.y + at(0, 2) * d.z,
                 at(1, 0) * d.x + at(1, 1) * d.y + at(1, 2) * d.z,
                 at(2, 0) * d.x + at(2, 1) * d.y + at(2, 2) * d.z };
    }
};

} // namespace enginelab::render
