#include <enginelab/render/FreeSpace.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <utility>

namespace enginelab::render {
namespace {

constexpr float infinite = std::numeric_limits<float>::infinity();
/** Cells at most: a quarter of a million keeps one field under 2 MB and a
    search under a few milliseconds. */
constexpr float maximumCells = 250'000.0F;

/** Distance from `q` to the segment [a, b]. */
[[nodiscard]] float segmentDistance(Vec3 q, Vec3 a, Vec3 b) noexcept {
    const auto d = b - a;
    const auto span2 = dot(d, d);
    const auto s = span2 > 1.0e-9F ? std::clamp(dot(q - a, d) / span2, 0.0F, 1.0F) : 0.0F;
    return length(q - (a + d * s));
}

} // namespace

Vec3 solidHalfExtent(const EngineSolid& solid) noexcept {
    if (solid.shape == EngineSolid::Shape::box) {
        Vec3 h {};
        for (int k = 0; k < 3; ++k) {
            const auto a = solid.axes[k];
            const auto s = k == 0 ? solid.half.x : k == 1 ? solid.half.y : solid.half.z;
            h = h + Vec3 { std::abs(a.x), std::abs(a.y), std::abs(a.z) } * s;
        }
        return h;
    }
    const auto a = solid.axes[1];
    const auto r = solid.half.x, l = solid.half.y;
    const auto side = [&](float c) { return std::abs(c) * l + r * std::sqrt(std::max(0.0F, 1.0F - c * c)); };
    return { side(a.x), side(a.y), side(a.z) };
}

std::vector<Vec3> solidSamples(const EngineSolid& solid) {
    std::vector<Vec3> corners;
    if (solid.shape == EngineSolid::Shape::box) {
        for (int k = 0; k < 8; ++k)
            corners.push_back(solid.centre + solid.axes[0] * ((k & 1) != 0 ? solid.half.x : -solid.half.x)
                              + solid.axes[1] * ((k & 2) != 0 ? solid.half.y : -solid.half.y)
                              + solid.axes[2] * ((k & 4) != 0 ? solid.half.z : -solid.half.z));
    } else {
        const auto axis = solid.axes[1];
        const auto u = normalise(cross(axis, std::abs(axis.y) < 0.9F ? Vec3 { 0.0F, 1.0F, 0.0F } : Vec3 { 1.0F, 0.0F, 0.0F }));
        const auto v = cross(axis, u);
        for (int end = 0; end < 2; ++end)
            for (int k = 0; k < 4; ++k) {
                const auto rim = (k == 0 ? u : k == 1 ? v : k == 2 ? u * -1.0F : v * -1.0F) * solid.half.x;
                corners.push_back(solid.centre + axis * (end == 0 ? -solid.half.y : solid.half.y) + rim);
            }
    }
    std::vector<Vec3> samples { solid.centre };
    for (const auto& corner : corners) {
        samples.push_back(lerp(solid.centre, corner, 0.5F));
        samples.push_back(lerp(solid.centre, corner, 0.9F));
    }
    return samples;
}

FreeSpace::FreeSpace(Vec3 minimum, Vec3 maximum, float cellMm, const std::vector<EngineSolid>& solids,
                     const std::vector<RouteTube>& tubes, float clearanceMm) {
    const auto size = maximum - minimum;
    const auto volume = std::max(1.0F, size.x) * std::max(1.0F, size.y) * std::max(1.0F, size.z);
    cell_ = std::max(cellMm, std::cbrt(volume / maximumCells));
    minimum_ = minimum;
    nx_ = std::max(1, static_cast<int>(std::ceil(size.x / cell_)));
    ny_ = std::max(1, static_cast<int>(std::ceil(size.y / cell_)));
    nz_ = std::max(1, static_cast<int>(std::ceil(size.z / cell_)));
    blocked_.assign(static_cast<std::size_t>(nx_) * static_cast<std::size_t>(ny_) * static_cast<std::size_t>(nz_), 0U);

    // Marks the cells of a box of the grid whose centre `inside` says.
    const auto mark = [&](Vec3 low, Vec3 high, const std::function<bool(Vec3)>& inside) {
        const auto index = [&](float v, float origin, int n) {
            return std::clamp(static_cast<int>(std::floor((v - origin) / cell_)), 0, n - 1);
        };
        if (high.x < minimum_.x || high.y < minimum_.y || high.z < minimum_.z || low.x > maximum.x || low.y > maximum.y
            || low.z > maximum.z)
            return;
        const auto x0 = index(low.x, minimum_.x, nx_), x1 = index(high.x, minimum_.x, nx_);
        const auto y0 = index(low.y, minimum_.y, ny_), y1 = index(high.y, minimum_.y, ny_);
        const auto z0 = index(low.z, minimum_.z, nz_), z1 = index(high.z, minimum_.z, nz_);
        for (int z = z0; z <= z1; ++z)
            for (int y = y0; y <= y1; ++y)
                for (int x = x0; x <= x1; ++x) {
                    const auto cell = (static_cast<std::size_t>(z) * static_cast<std::size_t>(ny_)
                                       + static_cast<std::size_t>(y)) * static_cast<std::size_t>(nx_)
                        + static_cast<std::size_t>(x);
                    if (blocked_[cell] != 0U) continue;
                    if (inside(centreOf(static_cast<std::int32_t>(cell)))) blocked_[cell] = 1U;
                }
    };
    for (const auto& solid : solids) {
        const auto h = solidHalfExtent(solid) + Vec3 { clearanceMm, clearanceMm, clearanceMm };
        mark(solid.centre - h, solid.centre + h,
             [&](Vec3 q) { return signedDistance(solid, q) < clearanceMm; });
    }
    for (const auto& tube : tubes) {
        for (std::size_t k = 0; k + 1U < tube.points.size(); ++k) {
            const auto a = tube.points[k], b = tube.points[k + 1U];
            const auto r = std::max(k < tube.radii.size() ? tube.radii[k] : 0.0F,
                                    k + 1U < tube.radii.size() ? tube.radii[k + 1U] : 0.0F)
                + clearanceMm;
            const Vec3 low { std::min(a.x, b.x) - r, std::min(a.y, b.y) - r, std::min(a.z, b.z) - r };
            const Vec3 high { std::max(a.x, b.x) + r, std::max(a.y, b.y) + r, std::max(a.z, b.z) + r };
            mark(low, high, [&](Vec3 q) { return segmentDistance(q, a, b) < r; });
        }
    }
}

std::int32_t FreeSpace::cellOf(Vec3 point) const noexcept {
    const auto x = static_cast<int>(std::floor((point.x - minimum_.x) / cell_));
    const auto y = static_cast<int>(std::floor((point.y - minimum_.y) / cell_));
    const auto z = static_cast<int>(std::floor((point.z - minimum_.z) / cell_));
    if (x < 0 || y < 0 || z < 0 || x >= nx_ || y >= ny_ || z >= nz_) return -1;
    return (z * ny_ + y) * nx_ + x;
}

Vec3 FreeSpace::centreOf(std::int32_t cell) const noexcept {
    const auto x = cell % nx_;
    const auto y = (cell / nx_) % ny_;
    const auto z = cell / (nx_ * ny_);
    return minimum_ + Vec3 { (static_cast<float>(x) + 0.5F) * cell_, (static_cast<float>(y) + 0.5F) * cell_,
                             (static_cast<float>(z) + 0.5F) * cell_ };
}

bool FreeSpace::blocked(Vec3 point) const noexcept {
    const auto cell = cellOf(point);
    return cell >= 0 && blocked_[static_cast<std::size_t>(cell)] != 0U;
}

bool FreeSpace::clear(Vec3 a, Vec3 b) const noexcept {
    const auto span = length(b - a);
    const auto steps = std::max(1, static_cast<int>(std::ceil(span / (0.5F * cell_))));
    for (int k = 0; k <= steps; ++k)
        if (blocked(lerp(a, b, static_cast<float>(k) / static_cast<float>(steps)))) return false;
    return true;
}

bool FreeSpace::firstFree(Vec3 origin, Vec3 direction, float maximumRunMm, Vec3& point, float& run) const {
    const auto step = 0.5F * cell_;
    for (float s = 0.0F; s <= maximumRunMm; s += step) {
        const auto q = origin + direction * s;
        if (cellOf(q) < 0) break;
        if (!blocked(q)) {
            point = q;
            run = s;
            return true;
        }
    }
    // Not along the way out: the nearest free cell within the same reach, so
    // that a point hemmed in by clearance never escapes through the engine.
    const auto centre = cellOf(origin);
    if (centre < 0) return false;
    const auto cx = centre % nx_, cy = (centre / nx_) % ny_, cz = centre / (nx_ * ny_);
    const auto span = static_cast<int>(std::ceil(maximumRunMm / cell_));
    float nearest = std::numeric_limits<float>::max();
    for (int dz = -span; dz <= span; ++dz)
        for (int dy = -span; dy <= span; ++dy)
            for (int dx = -span; dx <= span; ++dx) {
                const auto x = cx + dx, y = cy + dy, z = cz + dz;
                if (x < 0 || y < 0 || z < 0 || x >= nx_ || y >= ny_ || z >= nz_) continue;
                const auto cell = (z * ny_ + y) * nx_ + x;
                if (blocked_[static_cast<std::size_t>(cell)] != 0U) continue;
                const auto q = centreOf(cell);
                const auto d = length(q - origin);
                if (d < nearest && d <= maximumRunMm) {
                    nearest = d;
                    point = q;
                }
            }
    if (nearest > maximumRunMm) return false;
    run = nearest;
    return true;
}

FreeSpace::Field FreeSpace::reach(Vec3 origin, Vec3 leave, float maximumRunMm) const {
    Field field;
    if (!firstFree(origin, leave, maximumRunMm, field.start, field.startRun)) return field;
    const auto start = cellOf(field.start);
    if (start < 0 || blocked_[static_cast<std::size_t>(start)] != 0U) return field;
    field.distance.assign(blocked_.size(), infinite);
    field.previous.assign(blocked_.size(), -1);
    struct Step final {
        int dx, dy, dz;
        float cost;
    };
    std::vector<Step> steps;
    for (int dz = -1; dz <= 1; ++dz)
        for (int dy = -1; dy <= 1; ++dy)
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0 && dz == 0) continue;
                steps.push_back({ dx, dy, dz, cell_ * std::sqrt(static_cast<float>(dx * dx + dy * dy + dz * dz)) });
            }
    using Entry = std::pair<float, std::int32_t>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<>> queue;
    field.distance[static_cast<std::size_t>(start)] = length(centreOf(start) - field.start);
    queue.push({ field.distance[static_cast<std::size_t>(start)], start });
    while (!queue.empty()) {
        const auto [d, cell] = queue.top();
        queue.pop();
        if (d > field.distance[static_cast<std::size_t>(cell)]) continue;
        const auto x = cell % nx_, y = (cell / nx_) % ny_, z = cell / (nx_ * ny_);
        for (const auto& s : steps) {
            const auto px = x + s.dx, py = y + s.dy, pz = z + s.dz;
            if (px < 0 || py < 0 || pz < 0 || px >= nx_ || py >= ny_ || pz >= nz_) continue;
            const auto next = (pz * ny_ + py) * nx_ + px;
            if (blocked_[static_cast<std::size_t>(next)] != 0U) continue;
            const auto candidate = d + s.cost;
            if (candidate < field.distance[static_cast<std::size_t>(next)]) {
                field.distance[static_cast<std::size_t>(next)] = candidate;
                field.previous[static_cast<std::size_t>(next)] = cell;
                queue.push({ candidate, next });
            }
        }
    }
    field.valid = true;
    return field;
}

float FreeSpace::distance(const Field& field, Vec3 target, Vec3 arrive, float maximumRunMm) const {
    if (!field.valid) return infinite;
    Vec3 q;
    float run = 0.0F;
    if (!firstFree(target, arrive * -1.0F, maximumRunMm, q, run)) return infinite;
    const auto cell = cellOf(q);
    if (cell < 0) return infinite;
    const auto d = field.distance[static_cast<std::size_t>(cell)];
    if (!std::isfinite(d)) return infinite;
    return field.startRun + d + length(q - centreOf(cell)) + run;
}

std::vector<Vec3> FreeSpace::path(const Field& field, Vec3 origin, Vec3 target, Vec3 arrive, float maximumRunMm) const {
    if (!field.valid) return {};
    Vec3 q;
    float run = 0.0F;
    if (!firstFree(target, arrive * -1.0F, maximumRunMm, q, run)) return {};
    auto cell = cellOf(q);
    if (cell < 0 || !std::isfinite(field.distance[static_cast<std::size_t>(cell)])) return {};
    // The cells back to the start, then in order from it.
    std::vector<Vec3> cells { q };
    for (std::size_t guard = 0; cell >= 0 && guard < blocked_.size(); ++guard) {
        cells.push_back(centreOf(cell));
        cell = field.previous[static_cast<std::size_t>(cell)];
    }
    cells.push_back(field.start);
    std::reverse(cells.begin(), cells.end());
    // The straightest free line: from each kept point, on to the furthest
    // point still in sight.
    std::vector<Vec3> kept { cells.front() };
    std::size_t at = 0;
    while (at + 1U < cells.size()) {
        auto furthest = at + 1U;
        for (auto k = cells.size() - 1U; k > at + 1U; --k)
            if (clear(cells[at], cells[k])) {
                furthest = k;
                break;
            }
        kept.push_back(cells[furthest]);
        at = furthest;
    }
    std::vector<Vec3> result { origin };
    for (const auto& point : kept)
        if (length(point - result.back()) > 1.0e-3F) result.push_back(point);
    if (length(target - result.back()) > 1.0e-3F) result.push_back(target);
    return result;
}

} // namespace enginelab::render
