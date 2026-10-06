#include <enginelab/render/EngineModel3D.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <utility>

namespace enginelab::render {
namespace {
constexpr float pi = std::numbers::pi_v<float>;
constexpr Vec3 xAxis { 1.0F, 0.0F, 0.0F };
constexpr Vec3 yAxis { 0.0F, 1.0F, 0.0F };
constexpr Vec3 zAxis { 0.0F, 0.0F, 1.0F };

/** Where each vertex of a duct sits along its centreline, 0..1: the arc length
    of its nearest centreline point over the whole length. */
[[nodiscard]] std::vector<float> stationsAlong(const Mesh& mesh, const std::vector<Vec3>& centreline) {
    if (centreline.size() < 2U) return {};
    std::vector<float> arc(centreline.size(), 0.0F);
    for (std::size_t i = 1; i < centreline.size(); ++i) arc[i] = arc[i - 1] + length(centreline[i] - centreline[i - 1]);
    const auto total = std::max(arc.back(), 1.0e-3F);
    std::vector<float> stations;
    stations.reserve(mesh.vertices.size());
    for (const auto& vertex : mesh.vertices) {
        auto best = 1.0e30F;
        auto station = 0.0F;
        for (std::size_t i = 1; i < centreline.size(); ++i) {
            const auto a = centreline[i - 1];
            const auto segment = centreline[i] - a;
            const auto span = dot(segment, segment);
            const auto t = span > 0.0F ? std::clamp(dot(vertex.position - a, segment) / span, 0.0F, 1.0F) : 0.0F;
            const auto offset = vertex.position - (a + segment * t);
            const auto distance = dot(offset, offset);
            if (distance < best) {
                best = distance;
                station = (arc[i - 1] + t * (arc[i] - arc[i - 1])) / total;
            }
        }
        stations.push_back(station);
    }
    return stations;
}

[[nodiscard]] const CylinderBankConfig* bankFor(const EngineConfig& config, const CylinderConfig& cylinder) noexcept {
    const auto bank = std::find_if(config.banks.begin(), config.banks.end(),
        [&cylinder](const CylinderBankConfig& item) {
            return item.id == cylinder.bankId
                || std::find(item.cylinderIds.begin(), item.cylinderIds.end(), cylinder.id) != item.cylinderIds.end();
        });
    return bank == config.banks.end() ? nullptr : &*bank;
}

/** Frame whose +Y runs along a cylinder axis (which always lies in the
    crank plane) and whose +Z is the crankshaft axis. */
[[nodiscard]] Mat4 cylinderFrame(Vec3 origin, Vec3 axis) noexcept {
    const auto x = normalise(cross(axis, zAxis));
    return Mat4::basis(x, axis, cross(x, axis), origin);
}

/** Cylinder along the crankshaft (+Z) from `zStart` over `length`. */
/** A box solid of full `size` along the orthonormal axes x, y, z. */
[[nodiscard]] EngineSolid boxSolid(Vec3 x, Vec3 y, Vec3 z, Vec3 centre, Vec3 size) noexcept {
    EngineSolid solid;
    solid.centre = centre;
    solid.axes[0] = x;
    solid.axes[1] = y;
    solid.axes[2] = z;
    solid.half = size * 0.5F;
    return solid;
}

/** A cylinder solid from `base` along the unit `axis`. */
[[nodiscard]] EngineSolid cylinderSolid(Vec3 base, Vec3 axis, float radius, float height) noexcept {
    EngineSolid solid;
    solid.shape = EngineSolid::Shape::cylinder;
    solid.centre = base + axis * (0.5F * height);
    solid.axes[1] = axis;
    solid.half = { radius, 0.5F * height, 0.0F };
    return solid;
}

void appendShaft(Mesh& mesh, Vec3 centreXY, float zStart, float length, float radius, int segments) {
    appendCylinder(mesh, Mat4::translation({ centreXY.x, centreXY.y, zStart }) * Mat4::rotationX(pi * 0.5F),
                   radius, radius, length, segments);
}

/** Visible flame strength over the cycle: a short kernel before TDC, then a
    fast rise and an exponential burn-out through the first 75 degrees. */
[[nodiscard]] float burnEnvelope(double phaseDegrees) noexcept {
    const auto p = static_cast<float>(phaseDegrees);
    if (p < 75.0F) return std::exp(-p / 24.0F) * std::min(1.0F, p / 5.0F + 0.25F);
    if (p > 705.0F) return (p - 705.0F) / 15.0F * 0.25F;
    return 0.0F;
}

[[nodiscard]] std::vector<float> zOffsets(std::uint32_t count, float bore) {
    switch (std::clamp<std::uint32_t>(count, 1U, 3U)) {
    case 1U: return { 0.0F };
    case 2U: return { -0.21F * bore, 0.21F * bore };
    default: return { -0.27F * bore, 0.0F, 0.27F * bore };
    }
}
} // namespace

EngineModel3D::EngineModel3D(EngineConfig config) : config_(std::move(config)) {
    normaliseEngineConfig(config_);
    if (const auto error = validateEngineConfig(config_)) throw std::invalid_argument(*error);
    if (config_.cylinders.empty() || config_.cylinders.size() > 32U)
        throw std::invalid_argument("the 3D view supports 1 to 32 cylinders");
    reference_ = buildEngineKinematicsReference(config_);
    radialLike_ = config_.layout == EngineLayout::radial || config_.banks.size() > 2U;
    float bore = 0.0F;
    for (const auto& cylinder : config_.cylinders) bore = std::max(bore, static_cast<float>(cylinder.boreMm));
    pitch_ = bore * 1.25F;
    buildCylinders();
    buildCrankshaft();
    buildStructure();
    buildDucts();
    computeBounds();
}

Vec3 EngineModel3D::world(double simX, double simY, float z) const noexcept {
    return { -static_cast<float>(simX), -static_cast<float>(simY), z };
}

std::uint16_t EngineModel3D::addPart(Mesh mesh, SceneMaterial material, std::uint8_t layers) {
    parts_.push_back({ std::move(mesh), material, layers });
    return static_cast<std::uint16_t>(parts_.size() - 1U);
}

double EngineModel3D::cyclePhaseDegrees(std::size_t cylinder, double crankAngleDegrees) const noexcept {
    if (cylinder >= cylinders_.size()) return 0.0;
    return std::fmod(std::fmod(crankAngleDegrees - cylinders_[cylinder].crankOffsetDegrees, 720.0) + 720.0, 720.0);
}

void EngineModel3D::buildCylinders() {
    const auto count = config_.cylinders.size();
    std::vector<float> station(count), shift(count);
    float lastStation = 0.0F;
    for (std::size_t i = 0; i < count; ++i) {
        const auto& cylinder = config_.cylinders[i];
        const auto* bank = bankFor(config_, cylinder);
        auto position = static_cast<float>(i);
        std::size_t bankIndex = 0;
        if (bank != nullptr && !bank->cylinderIds.empty()) {
            const auto found = std::find(bank->cylinderIds.begin(), bank->cylinderIds.end(), cylinder.id);
            position = found == bank->cylinderIds.end() ? 0.0F
                : static_cast<float>(std::distance(bank->cylinderIds.begin(), found));
            bankIndex = static_cast<std::size_t>(bank - config_.banks.data());
        }
        station[i] = position;
        lastStation = std::max(lastStation, position);
        // Two banks sharing crank throws sit side by side, one rod width apart,
        // as their rods do on a shared pin.
        if (config_.banks.size() == 2U && !radialLike_)
            shift[i] = (bankIndex == 0 ? -0.28F : 0.28F) * static_cast<float>(cylinder.boreMm);
    }

    for (std::size_t i = 0; i < count; ++i) {
        const auto& config = config_.cylinders[i];
        Cylinder cylinder;
        cylinder.bore = static_cast<float>(config.boreMm);
        const auto b = cylinder.bore;
        cylinder.z = (station[i] - lastStation * 0.5F) * pitch_ + shift[i];
        const auto top = evaluateCylinderKinematics(config_, reference_, i, reference_.topDeadCentreAngleDegrees[i], 0.0);
        const auto bottom = evaluateCylinderKinematics(config_, reference_, i, reference_.bottomDeadCentreAngleDegrees[i], 0.0);
        const auto topWrist = world(top.wristPinXMm, top.wristPinYMm, cylinder.z);
        const auto bottomWrist = world(bottom.wristPinXMm, bottom.wristPinYMm, cylinder.z);
        cylinder.axis = length(topWrist - bottomWrist) > 1.0e-3F ? normalise(topWrist - bottomWrist) : Vec3 { 0, 1, 0 };
        const auto axis = cylinder.axis;
        cylinder.compressionHeight = config.compressionHeightMm > 0.0
            ? static_cast<float>(config.compressionHeightMm) : 0.42F * b;
        const auto gap = std::max(1.2F, 0.015F * b);
        cylinder.deckCentre = topWrist + axis * (cylinder.compressionHeight + gap);
        cylinder.chamberCentre = cylinder.deckCentre - axis * (0.1F * b);
        cylinder.crankOffsetDegrees = config.crankOffsetDegrees;
        const auto* bank = bankFor(config_, config);
        cylinder.cams = bank != nullptr ? &bank->camshafts : &config_.camshafts;

        // Exhaust on the outside of a V or flat bank, on +X for an upright
        // inline engine; a radial breathes front to back.
        if (radialLike_) {
            cylinder.exhaustSide = zAxis;
        } else {
            auto side = normalise(cross(axis, zAxis));
            if (std::abs(axis.x) > 0.3F && std::abs(side.x) > 0.2F) {
                if (side.x * axis.x < 0.0F) side = -side;
            } else if (std::abs(axis.x) > 0.3F) {
                if (side.y > 0.0F) side = -side;
            } else if (side.x < 0.0F) {
                side = -side;
            }
            cylinder.exhaustSide = side;
        }
        const auto side = cylinder.exhaustSide;
        cylinder.intakeValveAxis = normalise(axis - side * 0.21F);
        cylinder.exhaustValveAxis = normalise(axis + side * 0.21F);
        const auto lateral = normalise(cross(axis, side));

        // Piston: crown, three rings and the wrist pin, in the cylinder frame
        // with the wrist pin at the origin.
        Mesh piston;
        const auto skirt = 0.32F * b;
        appendCylinder(piston, Mat4::translation({ 0, -skirt, 0 }), 0.49F * b, 0.49F * b,
                       skirt + cylinder.compressionHeight, 40);
        for (int ring = 0; ring < 3; ++ring)
            appendTorus(piston, Mat4::translation({ 0, cylinder.compressionHeight - (0.06F + 0.05F * static_cast<float>(ring)) * b, 0 }),
                        0.493F * b, 0.012F * b, 40, 6);
        appendCylinder(piston, Mat4::rotationX(pi * 0.5F) * Mat4::translation({ 0, -0.38F * b, 0 }),
                       0.12F * b, 0.12F * b, 0.76F * b, 16);
        cylinder.pistonPart = addPart(std::move(piston), SceneMaterial::piston, layerMechanism);

        // Connecting rod: big end at the origin, small end at +Y.
        const auto rodLength = static_cast<float>(config.connectingRodMm);
        const auto articulated = config.connectingRodType == ConnectingRodType::articulated;
        const auto bigEndRadius = (articulated ? 0.13F : 0.25F) * b;
        Mesh rod;
        appendCylinder(rod, Mat4::translation({ 0, bigEndRadius * 0.8F, 0 }), 0.075F * b, 0.06F * b,
                       std::max(1.0F, rodLength - bigEndRadius * 0.8F - 0.12F * b), 12);
        appendCylinder(rod, Mat4::rotationX(pi * 0.5F) * Mat4::translation({ 0, -0.11F * b, 0 }),
                       bigEndRadius, bigEndRadius, 0.22F * b, 28);
        appendCylinder(rod, Mat4::translation({ 0, rodLength, 0 }) * Mat4::rotationX(pi * 0.5F)
                                * Mat4::translation({ 0, -0.09F * b, 0 }),
                       0.14F * b, 0.14F * b, 0.18F * b, 20);
        cylinder.rodPart = addPart(std::move(rod), SceneMaterial::forged, layerMechanism);

        // Valves at rest, in world space: lifting only translates them.
        const auto valves = [&](std::uint32_t valveCount, double diameterMm, Vec3 valveAxis, float sideSign, bool intake) {
            const auto n = std::clamp<std::uint32_t>(valveCount, 1U, 3U);
            const auto diameter = diameterMm > 0.0 ? static_cast<float>(diameterMm)
                : b * (n == 1U ? (intake ? 0.46F : 0.40F) : n == 2U ? (intake ? 0.36F : 0.31F) : 0.30F);
            Mesh mesh;
            for (const auto offset : zOffsets(n, b)) {
                const auto seat = cylinder.deckCentre + side * (sideSign * 0.22F * b) + lateral * offset + axis * 0.6F;
                const auto frame = Mat4::frameAlongY(seat, valveAxis);
                appendCylinder(mesh, frame, diameter * 0.5F, 0.06F * b, 0.07F * b, 24);
                appendCylinder(mesh, frame * Mat4::translation({ 0, 0.06F * b, 0 }), 0.028F * b, 0.028F * b, 0.7F * b, 8);
                appendCylinder(mesh, frame * Mat4::translation({ 0, 0.62F * b, 0 }), diameter * 0.3F, diameter * 0.3F,
                               0.04F * b, 16);
            }
            return mesh;
        };
        cylinder.intakeValvePart = addPart(valves(config.intakeValveCount, config.intakeValveDiameterMm,
                                                  cylinder.intakeValveAxis, -1.0F, true),
                                           SceneMaterial::intakeValve, layerValves);
        cylinder.exhaustValvePart = addPart(valves(config.exhaustValveCount, config.exhaustValveDiameterMm,
                                                   cylinder.exhaustValveAxis, 1.0F, false),
                                            SceneMaterial::exhaustValve, layerValves);

        Mesh flame;
        appendSphere(flame, Mat4::identity(), 0.42F * b, 24, 12);
        cylinder.flamePart = addPart(std::move(flame), SceneMaterial::flame, layerCombustion);

        // Liner, open at both ends: the X-ray view looks straight through it.
        Mesh liner;
        const auto linerBottom = bottomWrist - axis * (skirt + 0.1F * b);
        appendCylinder(liner, cylinderFrame(linerBottom, axis), 0.56F * b, 0.56F * b,
                       length(cylinder.deckCentre - linerBottom), 40, false, false);
        solids_.push_back(cylinderSolid(linerBottom, axis, 0.56F * b, length(cylinder.deckCentre - linerBottom)));
        cylinder.linerPart = addPart(std::move(liner), SceneMaterial::blockShell, layerStructure);

        cylinders_.push_back(cylinder);
    }
}

void EngineModel3D::buildCrankshaft() {
    struct Group final {
        std::uint32_t crankshaftId {};
        Vec3 pin;
        Vec3 origin;
        float zMin {};
        float zMax {};
        float bore {};
        float rodLength {};
        std::size_t reference {};
    };
    std::vector<Group> groups;
    for (std::size_t i = 0; i < cylinders_.size(); ++i) {
        const auto& config = config_.cylinders[i];
        if (config.connectingRodType == ConnectingRodType::articulated) continue;
        const auto k = evaluateCylinderKinematics(config_, reference_, i, 0.0, 0.0);
        const auto pin = world(k.crankPinXMm, k.crankPinYMm, 0.0F);
        const auto z = cylinders_[i].z;
        const auto shared = std::find_if(groups.begin(), groups.end(), [&](const Group& group) {
            return group.crankshaftId == k.crankshaftId && length(group.pin - pin) < 0.5F
                && z - group.zMax < pitch_ * 0.75F && group.zMin - z < pitch_ * 0.75F;
        });
        if (shared != groups.end()) {
            shared->zMin = std::min(shared->zMin, z);
            shared->zMax = std::max(shared->zMax, z);
            continue;
        }
        const auto crankshaft = std::find_if(config_.crankshafts.begin(), config_.crankshafts.end(),
            [&k](const CrankshaftConfig& item) { return item.id == k.crankshaftId; });
        const auto origin = crankshaft == config_.crankshafts.end() ? Vec3 {}
            : world(crankshaft->positionXMm, crankshaft->positionYMm, 0.0F);
        groups.push_back({ k.crankshaftId, pin, origin, z, z, cylinders_[i].bore,
                           static_cast<float>(config.connectingRodMm), i });
    }

    std::vector<std::uint32_t> crankshaftIds;
    for (const auto& group : groups) {
        const auto b = group.bore;
        const auto radius = length(group.pin - group.origin);
        const auto zCentre = 0.5F * (group.zMin + group.zMax);
        const auto pinHalf = 0.5F * (group.zMax - group.zMin) + 0.14F * b;
        const auto webThickness = 0.10F * b;
        // Keep the counterweight clear of the piston skirt at bottom dead centre.
        const auto counterweight = std::min(0.62F * b, (group.rodLength - radius - 0.32F * b) * 0.9F);
        Mesh mesh;
        appendShaft(mesh, { 0.0F, radius, 0.0F }, zCentre - pinHalf, 2.0F * pinHalf, 0.2F * b, 28);
        std::vector<Vec2> outline;
        for (int step = 0; step <= 14; ++step) {
            const auto a = pi * (200.0F + 10.0F * static_cast<float>(step)) / 180.0F;
            outline.push_back({ counterweight * std::cos(a), counterweight * std::sin(a) });
        }
        for (int step = 0; step < 20; ++step) {
            const auto a = 2.0F * pi * static_cast<float>(step) / 20.0F;
            outline.push_back({ 0.27F * b * std::cos(a), radius + 0.27F * b * std::sin(a) });
            outline.push_back({ 0.27F * b * std::cos(a), 0.27F * b * std::sin(a) });
        }
        for (const auto sideSign : { -1.0F, 1.0F })
            appendConvexExtrusion(mesh, Mat4::translation({ 0, 0, zCentre + sideSign * (pinHalf + webThickness * 0.5F) }),
                                  outline, webThickness);
        // Mesh is in the crankshaft frame: translate by the origin when posed.
        throws_.push_back({ group.origin, group.reference, addPart(std::move(mesh), SceneMaterial::forged, layerMechanism) });
        if (std::find(crankshaftIds.begin(), crankshaftIds.end(), group.crankshaftId) == crankshaftIds.end())
            crankshaftIds.push_back(group.crankshaftId);
    }

    // Main shaft, nose pulley and flywheel of each crankshaft.
    for (const auto id : crankshaftIds) {
        float zMin = 1.0e9F, zMax = -1.0e9F, b = 0.0F, radius = 0.0F;
        std::size_t firstThrow = 0;
        bool found = false;
        for (std::size_t t = 0; t < throws_.size(); ++t) {
            const auto& group = groups[t];
            if (group.crankshaftId != id) continue;
            if (!found) { firstThrow = t; found = true; }
            zMin = std::min(zMin, group.zMin);
            zMax = std::max(zMax, group.zMax);
            b = std::max(b, group.bore);
            radius = std::max(radius, length(group.pin - group.origin));
        }
        Mesh shaft;
        const auto start = zMin - pitch_ * 0.5F, end = zMax + pitch_ * 0.5F;
        appendShaft(shaft, {}, start, end - start, 0.23F * b, 32);
        // Nose pulley with a timing mark, flywheel with a ring gear and bolts:
        // round parts, so the marks are what shows the rotation.
        const auto pulleyLength = 0.3F * b;
        appendShaft(shaft, {}, start - pulleyLength, pulleyLength, 0.6F * b, 40);
        const auto origin = throws_[firstThrow].origin;
        solids_.push_back(cylinderSolid(origin + zAxis * (start - pulleyLength), zAxis, 0.6F * b, pulleyLength));
        appendTorus(shaft, Mat4::translation({ 0, 0, start - pulleyLength * 0.5F }) * Mat4::rotationX(pi * 0.5F),
                    0.6F * b, 0.035F * b, 40, 6);
        appendBox(shaft, Mat4::translation({ 0, 0.5F * b, start - pulleyLength - 0.01F * b }),
                  { 0.06F * b, 0.2F * b, 0.03F * b });
        const auto flywheelRadius = std::max(radius + 0.8F * b, 1.15F * b);
        const auto flywheelThickness = 0.22F * b;
        appendShaft(shaft, {}, end, flywheelThickness, flywheelRadius, 64);
        solids_.push_back(cylinderSolid(origin + zAxis * end, zAxis, flywheelRadius, flywheelThickness));
        appendTorus(shaft, Mat4::translation({ 0, 0, end + flywheelThickness * 0.5F }) * Mat4::rotationX(pi * 0.5F),
                    flywheelRadius, 0.04F * b, 64, 6);
        for (int bolt = 0; bolt < 6; ++bolt) {
            const auto a = 2.0F * pi * static_cast<float>(bolt) / 6.0F;
            appendShaft(shaft, { 0.32F * flywheelRadius * std::cos(a), 0.32F * flywheelRadius * std::sin(a), 0 },
                        end + flywheelThickness, 0.05F * b, 0.06F * b, 12);
        }
        appendBox(shaft, Mat4::translation({ 0, flywheelRadius * 0.82F, end + flywheelThickness + 0.01F * b }),
                  { 0.08F * b, 0.22F * flywheelRadius, 0.03F * b });
        rotatingParts_.push_back(addPart(std::move(shaft), SceneMaterial::steel, layerMechanism));
        rotatingThrow_.push_back(firstThrow);
    }
}

void EngineModel3D::buildStructure() {
    // Group the cylinders of each bank; a radial gets one group per cylinder.
    std::vector<std::vector<std::size_t>> groups;
    for (std::size_t i = 0; i < cylinders_.size(); ++i) {
        const auto* bank = bankFor(config_, config_.cylinders[i]);
        const auto key = radialLike_ || bank == nullptr ? (radialLike_ ? i : 0U)
            : static_cast<std::size_t>(bank - config_.banks.data());
        if (groups.size() <= key) groups.resize(key + 1U);
        groups[key].push_back(i);
    }
    const auto crankOrigin = throws_.empty() ? Vec3 {} : throws_.front().origin;
    float zMin = 1.0e9F, zMax = -1.0e9F, b = 0.0F, radius = 0.0F;
    for (const auto& c : cylinders_) { zMin = std::min(zMin, c.z); zMax = std::max(zMax, c.z); b = std::max(b, c.bore); }
    for (std::size_t i = 0; i < cylinders_.size(); ++i) {
        const auto k = evaluateCylinderKinematics(config_, reference_, i, 0.0, 0.0);
        radius = std::max(radius, length(world(k.crankPinXMm, k.crankPinYMm, 0.0F) - crankOrigin));
    }

    Mesh heads, block;
    for (const auto& group : groups) {
        if (group.empty()) continue;
        const auto& first = cylinders_[group.front()];
        const auto axis = first.axis;
        const auto side = first.exhaustSide;
        const auto lateral = normalise(cross(side, axis));
        Vec3 deck {};
        float groupMin = 1.0e9F, groupMax = -1.0e9F;
        for (const auto index : group) {
            deck += cylinders_[index].deckCentre;
            groupMin = std::min(groupMin, cylinders_[index].z);
            groupMax = std::max(groupMax, cylinders_[index].z);
        }
        deck = deck * (1.0F / static_cast<float>(group.size()));
        const auto span = radialLike_ ? 1.2F * b : groupMax - groupMin + pitch_ * 0.92F;
        appendBox(heads, Mat4::basis(side, axis, lateral, deck + axis * (0.31F * b)), { 1.4F * b, 0.62F * b, span });
        solids_.push_back(boxSolid(side, axis, lateral, deck + axis * (0.31F * b), { 1.4F * b, 0.62F * b, span }));
        if (!radialLike_) {
            const auto deckDistance = dot(deck - crankOrigin, axis);
            const auto height = std::max(0.2F * b, deckDistance - 0.25F * b);
            const auto centre = Vec3 { crankOrigin.x, crankOrigin.y, deck.z } + axis * (0.25F * b + height * 0.5F);
            appendBox(block, Mat4::basis(side, axis, lateral, centre), { 1.25F * b, height, span });
            solids_.push_back(boxSolid(side, axis, lateral, centre, { 1.25F * b, height, span }));
        }
    }
    const auto length = zMax - zMin + pitch_;
    if (radialLike_) {
        appendShaft(block, crankOrigin, zMin - 0.75F * b, 1.5F * b + (zMax - zMin), radius + 0.7F * b, 48);
        solids_.push_back(cylinderSolid(Vec3 { crankOrigin.x, crankOrigin.y, zMin - 0.75F * b }, zAxis, radius + 0.7F * b,
                                        1.5F * b + (zMax - zMin)));
    } else {
        const auto halfWidth = radius + 0.7F * b;
        const auto top = crankOrigin.y + 0.3F * b;
        const auto bottom = crankOrigin.y - (radius + 0.68F * b);
        const auto zCentre = 0.5F * (zMin + zMax);
        appendBox(block, Mat4::translation({ crankOrigin.x, 0.5F * (top + bottom), zCentre }),
                  { 2.0F * halfWidth, top - bottom, length + pitch_ * 0.1F });
        appendBox(block, Mat4::translation({ crankOrigin.x, bottom - 0.28F * b, zCentre }),
                  { 1.7F * halfWidth, 0.56F * b, length * 0.92F });
        solids_.push_back(boxSolid(xAxis, yAxis, zAxis, { crankOrigin.x, 0.5F * (top + bottom), zCentre },
                                   { 2.0F * halfWidth, top - bottom, length + pitch_ * 0.1F }));
        solids_.push_back(boxSolid(xAxis, yAxis, zAxis, { crankOrigin.x, bottom - 0.28F * b, zCentre },
                                   { 1.7F * halfWidth, 0.56F * b, length * 0.92F }));
    }
    (void)addPart(std::move(block), SceneMaterial::blockShell, layerStructure);
    (void)addPart(std::move(heads), SceneMaterial::headShell, layerStructure);
}

void EngineModel3D::buildDucts() {
    Mesh exhaust;
    float bore = 0.0F;
    for (std::size_t i = 0; i < cylinders_.size(); ++i) {
        const auto& cylinder = cylinders_[i];
        const auto& config = config_.cylinders[i];
        const auto b = cylinder.bore;
        bore = std::max(bore, b);
        const auto axis = cylinder.axis;
        const auto side = cylinder.exhaustSide;

        // Intake port: behind the intake valves, out of the head; the runner
        // itself is laid out from the intake path.
        const auto intakeDiameter = config.intakeRunnerDiameterMm > 0.0
            ? static_cast<float>(config.intakeRunnerDiameterMm) : 0.36F * b;
        const auto intakePort = cylinder.deckCentre + axis * (0.42F * b) - side * (0.70F * b);
        const auto intakeOut = radialLike_ ? -side : normalise(axis * 0.35F - side);
        intakePorts_.push_back({ config.id, intakePort, intakeOut, intakeDiameter,
                                 cylinder.deckCentre + axis * (0.12F * b) - side * (0.22F * b) });

        // Exhaust port stub: out of the head on the exhaust side.
        const auto exhaustDiameter = 0.34F * b;
        const auto exhaustInner = cylinder.deckCentre + axis * (0.1F * b) + side * (0.22F * b);
        const auto exhaustPort = cylinder.deckCentre + axis * (0.3F * b) + side * (0.70F * b);
        const auto exhaustOut = normalise(side - axis * 0.25F);
        const auto exhaustEnd = exhaustPort + exhaustOut * (0.6F * b);
        appendTube(exhaust, catmullRom({ exhaustInner, exhaustPort, exhaustEnd }, 10), { exhaustDiameter * 0.5F }, 18);
        exhaustPorts_.push_back({ config.id, exhaustEnd, exhaustOut, exhaustDiameter, exhaustInner });
    }
    (void)addPart(std::move(exhaust), SceneMaterial::exhaustPipe, layerExhaust);

    const auto add = [this](std::vector<DuctPiece> pieces) {
        for (auto& piece : pieces) {
            if (piece.mesh.empty()) continue;
            const auto intake = piece.kind != DuctKind::exhaustComponent && piece.kind != DuctKind::exhaustFeeder;
            const auto material = !intake ? SceneMaterial::exhaustPipe
                : piece.kind == DuctKind::intakeThrottle ? SceneMaterial::forged : SceneMaterial::intakeFlow;
            const auto part = addPart(std::move(piece.mesh), material, intake ? layerIntake : layerExhaust);
            parts_[part].stations = stationsAlong(parts_[part].mesh, piece.centreline);
            if (farParts_.size() < parts_.size()) farParts_.resize(parts_.size(), false);
            farParts_[part] = !piece.nearEngine;
            ducts_.push_back({ piece.kind, piece.pathId, piece.elementId, piece.componentType, part,
                               std::move(piece.centreline), piece.authoredLengthMm, std::move(piece.radii),
                               piece.opensToAtmosphere, piece.openEnd, piece.openAxis });
            if (piece.kind == DuctKind::intakeRunner)
                for (std::size_t i = 0; i < cylinders_.size(); ++i)
                    if (config_.cylinders[i].id == piece.elementId) cylinders_[i].runnerPart = part;
        }
    };
    add(layoutIntake(config_, intakePorts_, bore, radialLike_, solids_, &supercharger_));
    buildSupercharger();
    // The exhaust is routed round the intake: its tubes, and its boxes
    // (plenum, throttle bodies, airbox) as solids.
    std::vector<RouteTube> intakeTubes;
    auto avoid = solids_;
    for (const auto& duct : ducts_) {
        if (duct.radii.size() == duct.centreline.size() && duct.centreline.size() >= 2U) {
            intakeTubes.push_back({ duct.centreline, duct.radii });
            continue;
        }
        const auto& mesh = parts_[duct.part].mesh;
        if (mesh.vertices.empty()) continue;
        auto low = mesh.vertices.front().position, high = low;
        for (const auto& vertex : mesh.vertices) {
            const auto p = vertex.position;
            low = { std::min(low.x, p.x), std::min(low.y, p.y), std::min(low.z, p.z) };
            high = { std::max(high.x, p.x), std::max(high.y, p.y), std::max(high.z, p.z) };
        }
        avoid.push_back(boxBetween(low, high));
    }
    add(layoutExhaust(config_, exhaustPorts_, bore, intakeTubes, avoid, &turbo_));
    buildTurbo();

    // A plume at every outlet, lit only by an afterfire.
    const auto ductCount = ducts_.size();
    for (std::size_t d = 0; d < ductCount; ++d) {
        const auto& duct = ducts_[d];
        if (duct.kind != DuctKind::exhaustComponent || duct.componentType != ExhaustComponentType::outlet
            || duct.centreline.size() < 2U)
            continue;
        const auto tip = duct.centreline.back();
        const auto axis = normalise(tip - duct.centreline[duct.centreline.size() - 2U]);
        float radius = 0.0F;
        const auto& part = parts_[duct.part];
        for (std::size_t v = 0; v < part.mesh.vertices.size(); ++v)
            if (part.stations[v] > 0.98F) {
                const auto offset = part.mesh.vertices[v].position - tip;
                radius = std::max(radius, length(offset - axis * dot(offset, axis)));
            }
        radius = std::max(radius, 0.15F * bore);
        Mesh plume;
        appendCylinder(plume, Mat4::frameAlongY(tip, axis), 0.85F * radius, 1.7F * radius, 5.0F * radius, 20, false, false);
        appendSphere(plume, Mat4::translation(tip + axis * (1.2F * radius)), 0.95F * radius, 16, 10);
        outletFlameParts_.push_back(addPart(std::move(plume), SceneMaterial::flame, layerExhaust));
    }
    farParts_.resize(parts_.size(), false);
}

void EngineModel3D::buildTurbo() {
    if (!turbo_.placed) return;
    const auto& forced = config_.forcedInduction;
    Mesh turbineHousing, compressorHousing, turbineWheel, compressorWheel;
    appendTurboHousings(turbineHousing, compressorHousing, turbo_);
    // Without a blade count, a typical wheel: it is only drawn.
    const auto turbineBlades = forced.turbineBladeCount > 0U ? static_cast<int>(forced.turbineBladeCount) : 10;
    const auto compressorBlades = forced.compressorBladeCount > 0U ? static_cast<int>(forced.compressorBladeCount) : 7;
    appendTurboWheel(turbineWheel, turbo_.turbineWheelRadius, 0.8F * turbo_.turbineWidth, turbineBlades, 1.0F);
    appendTurboWheel(compressorWheel, turbo_.compressorWheelRadius, 0.7F * turbo_.compressorWidth, compressorBlades, -1.0F);
    turboParts_[0] = addPart(std::move(turbineHousing), SceneMaterial::turboHousing, layerExhaust);
    turboParts_[1] = addPart(std::move(compressorHousing), SceneMaterial::turboHousing, layerIntake);
    turboParts_[2] = addPart(std::move(turbineWheel), SceneMaterial::forged, layerExhaust);
    turboParts_[3] = addPart(std::move(compressorWheel), SceneMaterial::steel, layerIntake);
    for (const auto& solid : turboSolids(turbo_)) solids_.push_back(solid);
}

void EngineModel3D::buildSupercharger() {
    if (!supercharger_.placed) return;
    Mesh housing, impeller;
    appendSuperchargerHousing(housing, supercharger_);
    // Without a blade count, a typical impeller: it is only drawn.
    const auto blades = config_.forcedInduction.compressorBladeCount > 0U
        ? static_cast<int>(config_.forcedInduction.compressorBladeCount) : 12;
    appendTurboWheel(impeller, supercharger_.wheelRadius, 0.7F * supercharger_.width, blades, 1.0F);
    superchargerParts_[0] = addPart(std::move(housing), SceneMaterial::turboHousing, layerIntake);
    superchargerParts_[1] = addPart(std::move(impeller), SceneMaterial::steel, layerIntake);
    for (const auto& solid : superchargerSolids(supercharger_)) solids_.push_back(solid);
}

double EngineModel3D::superchargerImpellerDegrees(double crankAngleDegrees) const noexcept {
    return std::fmod(crankAngleDegrees * config_.forcedInduction.superchargerDriveRatio, 360.0);
}

float EngineModel3D::throwRotation(const Throw& item, double crankAngleDegrees) const noexcept {
    const auto k = evaluateCylinderKinematics(config_, reference_, item.referenceCylinder, crankAngleDegrees, 0.0);
    const auto pin = world(k.crankPinXMm, k.crankPinYMm, 0.0F) - item.origin;
    // Throw meshes are built with their pin on +Y.
    return std::atan2(pin.y, pin.x) - pi * 0.5F;
}

EngineModel3D::CylinderProbe EngineModel3D::probe(std::size_t index, double crankAngleDegrees) const noexcept {
    if (index >= cylinders_.size()) return {};
    const auto& cylinder = cylinders_[index];
    const auto k = evaluateCylinderKinematics(config_, reference_, index, crankAngleDegrees, 0.0);
    const auto wrist = world(k.wristPinXMm, k.wristPinYMm, cylinder.z);
    return { wrist, world(k.crankPinXMm, k.crankPinYMm, cylinder.z), wrist + cylinder.axis * cylinder.compressionHeight,
             cylinder.deckCentre, cylinder.axis, static_cast<float>(config_.cylinders[index].connectingRodMm) };
}

void EngineModel3D::pose(const ScenePoseInput& input, std::vector<SceneInstance>& out) const {
    out.clear();
    for (std::size_t part = 0; part < parts_.size(); ++part)
        out.push_back({ static_cast<std::uint16_t>(part), Mat4::identity(), 1.0F });
    const auto angle = input.crankAngleDegrees;

    std::size_t throwIndex = 0;
    std::array<Mat4, 64> throwTransforms {};
    for (const auto& item : throws_) {
        const auto transform = Mat4::translation(item.origin) * Mat4::rotationZ(throwRotation(item, angle));
        if (throwIndex < throwTransforms.size()) throwTransforms[throwIndex] = transform;
        out[item.part].transform = transform;
        ++throwIndex;
    }
    for (std::size_t k = 0; k < rotatingParts_.size(); ++k)
        if (rotatingThrow_[k] < throwTransforms.size()) out[rotatingParts_[k]].transform = throwTransforms[rotatingThrow_[k]];

    for (std::size_t i = 0; i < cylinders_.size(); ++i) {
        const auto& cylinder = cylinders_[i];
        const auto k = evaluateCylinderKinematics(config_, reference_, i, angle, 0.0);
        const auto wrist = world(k.wristPinXMm, k.wristPinYMm, cylinder.z);
        const auto bigEnd = world(k.crankPinXMm, k.crankPinYMm, cylinder.z);
        out[cylinder.pistonPart].transform = cylinderFrame(wrist, cylinder.axis);
        const auto rodAxis = normalise(wrist - bigEnd);
        const auto rodX = normalise(cross(rodAxis, zAxis));
        out[cylinder.rodPart].transform = Mat4::basis(rodX, rodAxis, cross(rodX, rodAxis), bigEnd);

        const auto phase = cyclePhaseDegrees(i, angle);
        const auto& cams = *cylinder.cams;
        const auto high = cams.variableProfileEnabled && input.rpm >= cams.switchRpm && input.throttle >= cams.switchThrottle;
        const auto intakeLift = profiledValveLiftMm(phase, 360.0 + cams.intakeCenterlineDegrees,
            high ? cams.highIntakeDurationDegrees : cams.intakeDurationDegrees,
            high ? cams.highIntakeLiftMm : cams.intakeLiftMm,
            high && !cams.highIntakeLiftProfile.empty() ? cams.highIntakeLiftProfile : cams.intakeLiftProfile);
        const auto exhaustLift = profiledValveLiftMm(phase, 360.0 - cams.exhaustCenterlineDegrees,
            high ? cams.highExhaustDurationDegrees : cams.exhaustDurationDegrees,
            high ? cams.highExhaustLiftMm : cams.exhaustLiftMm,
            high && !cams.highExhaustLiftProfile.empty() ? cams.highExhaustLiftProfile : cams.exhaustLiftProfile);
        out[cylinder.intakeValvePart].transform = Mat4::translation(cylinder.intakeValveAxis * -static_cast<float>(intakeLift));
        out[cylinder.exhaustValvePart].transform = Mat4::translation(cylinder.exhaustValveAxis * -static_cast<float>(exhaustLift));

        const auto burn = burnEnvelope(phase) * std::clamp(input.combustion[i], 0.0F, 1.0F);
        const auto scale = 0.6F + 0.5F * burn;
        const auto x = normalise(cross(cylinder.axis, zAxis));
        out[cylinder.flamePart].transform = Mat4::basis(x * scale, cylinder.axis * (scale * 0.45F), zAxis * scale,
                                                         cylinder.chamberCentre);
        out[cylinder.flamePart].intensity = burn;
        const auto maximumLift = std::max(0.1, high ? cams.highIntakeLiftMm : cams.intakeLiftMm);
        if (cylinder.runnerPart < out.size())
            out[cylinder.runnerPart].intensity = static_cast<float>(std::clamp(intakeLift / maximumLift, 0.0, 1.0));
    }
    for (const auto part : outletFlameParts_) out[part].intensity = std::clamp(input.afterfire, 0.0F, 1.0F);
    if (turbo_.placed) {
        // Both wheels turn with the shaft, about its axis.
        const auto shaft = static_cast<float>(std::fmod(input.turboShaftDegrees, 360.0) * (pi / 180.0F));
        const auto axis = turbo_.axis;
        const auto x = turbo_.radial * std::cos(shaft) + cross(axis, turbo_.radial) * std::sin(shaft);
        const auto z = cross(x, axis);
        out[turboParts_[2]].transform = Mat4::basis(x, axis, z, turbo_.turbineCentre);
        out[turboParts_[3]].transform = Mat4::basis(x, axis, z, turbo_.compressorCentre);
    }
    if (supercharger_.placed) {
        // Geared to the crank.
        const auto impeller = static_cast<float>(superchargerImpellerDegrees(input.crankAngleDegrees) * (pi / 180.0F));
        const auto axis = supercharger_.axis;
        const auto x = supercharger_.radial * std::cos(impeller) + cross(axis, supercharger_.radial) * std::sin(impeller);
        out[superchargerParts_[1]].transform = Mat4::basis(x, axis, cross(x, axis), supercharger_.centre);
    }
    // Plenum, airbox and inlet duct glow with the mean draw of their runners.
    for (const auto& duct : ducts_) {
        if (duct.kind != DuctKind::intakePlenum && duct.kind != DuctKind::intakeAirbox
            && duct.kind != DuctKind::intakeInletDuct)
            continue;
        float draw = 0.0F;
        int runners = 0;
        for (const auto& other : ducts_)
            if (other.kind == DuctKind::intakeRunner && other.pathId == duct.pathId) {
                draw += out[other.part].intensity;
                ++runners;
            }
        const auto share = duct.kind == DuctKind::intakePlenum ? 0.5F : 0.3F;
        out[duct.part].intensity = runners > 0 ? share * draw / static_cast<float>(runners) : 0.0F;
    }
}

PartIdentity EngineModel3D::identify(std::uint16_t part) const noexcept {
    for (std::size_t i = 0; i < cylinders_.size(); ++i) {
        const auto& c = cylinders_[i];
        const auto cylinder = static_cast<int>(i);
        if (part == c.pistonPart) return { PartRole::piston, cylinder, -1 };
        if (part == c.rodPart) return { PartRole::rod, cylinder, -1 };
        if (part == c.intakeValvePart) return { PartRole::intakeValve, cylinder, -1 };
        if (part == c.exhaustValvePart) return { PartRole::exhaustValve, cylinder, -1 };
        if (part == c.flamePart) return { PartRole::combustion, cylinder, -1 };
        if (part == c.linerPart) return { PartRole::liner, cylinder, -1 };
    }
    for (std::size_t d = 0; d < ducts_.size(); ++d)
        if (ducts_[d].part == part) {
            int cylinder = -1;
            if (ducts_[d].kind == DuctKind::intakeRunner)
                for (std::size_t i = 0; i < config_.cylinders.size(); ++i)
                    if (config_.cylinders[i].id == ducts_[d].elementId) cylinder = static_cast<int>(i);
            return { PartRole::duct, cylinder, static_cast<int>(d) };
        }
    if (std::find(outletFlameParts_.begin(), outletFlameParts_.end(), part) != outletFlameParts_.end())
        return { PartRole::afterfire, -1, -1 };
    if (std::find(turboParts_.begin(), turboParts_.end(), part) != turboParts_.end()) return { PartRole::turbo, -1, -1 };
    if (std::find(superchargerParts_.begin(), superchargerParts_.end(), part) != superchargerParts_.end())
        return { PartRole::supercharger, -1, -1 };
    if (part >= parts_.size()) return {};
    switch (parts_[part].material) {
    case SceneMaterial::headShell: return { PartRole::head, -1, -1 };
    case SceneMaterial::exhaustPipe: return { PartRole::exhaustPorts, -1, -1 };
    case SceneMaterial::intakeFlow: return { PartRole::intakePorts, -1, -1 };
    case SceneMaterial::steel:
    case SceneMaterial::forged: return { PartRole::crankshaft, -1, -1 };
    default: return { PartRole::block, -1, -1 };
    }
}

std::optional<PartHit> pickPart(const EngineModel3D& model, const std::vector<SceneInstance>& instances, Vec3 origin,
                                Vec3 direction, const std::function<bool(std::uint16_t)>& pickable) {
    std::optional<PartHit> best;
    for (const auto& instance : instances) {
        if (instance.part >= model.parts().size() || !pickable(instance.part)) continue;
        const auto& mesh = model.parts()[instance.part].mesh;
        for (std::size_t i = 0; i + 2U < mesh.indices.size(); i += 3U) {
            // Moller-Trumbore, both faces.
            const auto a = instance.transform.transformPoint(mesh.vertices[mesh.indices[i]].position);
            const auto e1 = instance.transform.transformPoint(mesh.vertices[mesh.indices[i + 1U]].position) - a;
            const auto e2 = instance.transform.transformPoint(mesh.vertices[mesh.indices[i + 2U]].position) - a;
            const auto p = cross(direction, e2);
            const auto determinant = dot(e1, p);
            if (std::abs(determinant) < 1.0e-9F) continue;
            const auto inverse = 1.0F / determinant;
            const auto t = origin - a;
            const auto u = dot(t, p) * inverse;
            if (u < 0.0F || u > 1.0F) continue;
            const auto q = cross(t, e1);
            const auto v = dot(direction, q) * inverse;
            if (v < 0.0F || u + v > 1.0F) continue;
            const auto distance = dot(e2, q) * inverse;
            if (distance > 0.0F && (!best || distance < best->distance)) best = PartHit { instance.part, distance };
        }
    }
    return best;
}

void EngineModel3D::computeBounds() {
    std::vector<SceneInstance> instances;
    pose({}, instances);
    const SceneBounds empty { { 1.0e9F, 1.0e9F, 1.0e9F }, { -1.0e9F, -1.0e9F, -1.0e9F } };
    bounds_ = empty;
    systemBounds_ = empty;
    const auto grow = [](SceneBounds& bounds, Vec3 p) {
        bounds.minimum = { std::min(bounds.minimum.x, p.x), std::min(bounds.minimum.y, p.y), std::min(bounds.minimum.z, p.z) };
        bounds.maximum = { std::max(bounds.maximum.x, p.x), std::max(bounds.maximum.y, p.y), std::max(bounds.maximum.z, p.z) };
    };
    for (const auto& instance : instances) {
        if (parts_[instance.part].material == SceneMaterial::flame) continue;
        const auto near = instance.part >= farParts_.size() || !farParts_[instance.part];
        for (const auto& vertex : parts_[instance.part].mesh.vertices) {
            const auto p = instance.transform.transformPoint(vertex.position);
            grow(systemBounds_, p);
            if (near) grow(bounds_, p);
        }
    }
}

} // namespace enginelab::render
