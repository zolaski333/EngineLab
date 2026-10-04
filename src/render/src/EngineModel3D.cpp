#include <enginelab/render/EngineModel3D.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <utility>

namespace enginelab::render {
namespace {
constexpr float pi = std::numbers::pi_v<float>;
constexpr Vec3 zAxis { 0.0F, 0.0F, 1.0F };

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
        (void)addPart(std::move(liner), SceneMaterial::blockShell, layerStructure);

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
        appendTorus(shaft, Mat4::translation({ 0, 0, start - pulleyLength * 0.5F }) * Mat4::rotationX(pi * 0.5F),
                    0.6F * b, 0.035F * b, 40, 6);
        appendBox(shaft, Mat4::translation({ 0, 0.5F * b, start - pulleyLength - 0.01F * b }),
                  { 0.06F * b, 0.2F * b, 0.03F * b });
        const auto flywheelRadius = std::max(radius + 0.8F * b, 1.15F * b);
        const auto flywheelThickness = 0.22F * b;
        appendShaft(shaft, {}, end, flywheelThickness, flywheelRadius, 64);
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
        if (!radialLike_) {
            const auto deckDistance = dot(deck - crankOrigin, axis);
            const auto height = std::max(0.2F * b, deckDistance - 0.25F * b);
            const auto centre = Vec3 { crankOrigin.x, crankOrigin.y, deck.z } + axis * (0.25F * b + height * 0.5F);
            appendBox(block, Mat4::basis(side, axis, lateral, centre), { 1.25F * b, height, span });
        }
    }
    const auto length = zMax - zMin + pitch_;
    if (radialLike_) {
        appendShaft(block, crankOrigin, zMin - 0.75F * b, 1.5F * b + (zMax - zMin), radius + 0.7F * b, 48);
    } else {
        const auto halfWidth = radius + 0.7F * b;
        const auto top = crankOrigin.y + 0.3F * b;
        const auto bottom = crankOrigin.y - (radius + 0.68F * b);
        const auto zCentre = 0.5F * (zMin + zMax);
        appendBox(block, Mat4::translation({ crankOrigin.x, 0.5F * (top + bottom), zCentre }),
                  { 2.0F * halfWidth, top - bottom, length + pitch_ * 0.1F });
        appendBox(block, Mat4::translation({ crankOrigin.x, bottom - 0.28F * b, zCentre }),
                  { 1.7F * halfWidth, 0.56F * b, length * 0.92F });
    }
    (void)addPart(std::move(block), SceneMaterial::blockShell, layerStructure);
    (void)addPart(std::move(heads), SceneMaterial::headShell, layerStructure);
}

void EngineModel3D::buildDucts() {
    Mesh exhaust;
    std::vector<Vec3> runnerEnds;
    for (std::size_t i = 0; i < cylinders_.size(); ++i) {
        auto& cylinder = cylinders_[i];
        const auto& config = config_.cylinders[i];
        const auto b = cylinder.bore;
        const auto axis = cylinder.axis;
        const auto side = cylinder.exhaustSide;

        // Intake: from behind the intake valves, out of the head, then up and
        // away towards the plenum.
        const auto intakeDiameter = config.intakeRunnerDiameterMm > 0.0
            ? static_cast<float>(config.intakeRunnerDiameterMm) : 0.36F * b;
        const auto intakePort = cylinder.deckCentre + axis * (0.42F * b) - side * (0.70F * b);
        const auto intakeOut = radialLike_ ? -side : normalise(axis * 0.35F - side);
        const auto runnerEnd = intakePort + intakeOut * (0.9F * b) + axis * (radialLike_ ? 0.2F * b : 0.7F * b);
        runnerEnds.push_back(runnerEnd);
        Mesh runner;
        appendTube(runner,
            catmullRom({ cylinder.deckCentre + axis * (0.12F * b) - side * (0.22F * b), intakePort,
                         intakePort + intakeOut * (0.45F * b) + axis * (0.2F * b), runnerEnd }, 10),
            { intakeDiameter * 0.5F }, 18, false, true);
        cylinder.runnerPart = addPart(std::move(runner), SceneMaterial::intakeFlow, layerIntake);
        intakePorts_.push_back({ config.id, intakePort, intakeOut, intakeDiameter });

        // Exhaust port stub: out of the head on the exhaust side.
        const auto exhaustDiameter = 0.34F * b;
        const auto exhaustPort = cylinder.deckCentre + axis * (0.3F * b) + side * (0.70F * b);
        const auto exhaustOut = normalise(side - axis * 0.25F);
        const auto exhaustEnd = exhaustPort + exhaustOut * (0.6F * b);
        appendTube(exhaust,
            catmullRom({ cylinder.deckCentre + axis * (0.1F * b) + side * (0.22F * b), exhaustPort, exhaustEnd }, 10),
            { exhaustDiameter * 0.5F }, 18);
        exhaustPorts_.push_back({ config.id, exhaustEnd, exhaustOut, exhaustDiameter });
    }
    (void)addPart(std::move(exhaust), SceneMaterial::exhaustPipe, layerExhaust);

    if (!radialLike_ && !runnerEnds.empty()) {
        // One box over every runner end, both banks of a V included.
        Vec3 centre {};
        float xMin = 1.0e9F, xMax = -1.0e9F, zMin = 1.0e9F, zMax = -1.0e9F, b = 0.0F;
        for (const auto& end : runnerEnds) {
            centre += end;
            xMin = std::min(xMin, end.x);
            xMax = std::max(xMax, end.x);
            zMin = std::min(zMin, end.z);
            zMax = std::max(zMax, end.z);
        }
        for (const auto& c : cylinders_) b = std::max(b, c.bore);
        centre = centre * (1.0F / static_cast<float>(runnerEnds.size()));
        Mesh plenum;
        appendBox(plenum, Mat4::translation({ 0.5F * (xMin + xMax), centre.y + 0.25F * b, 0.5F * (zMin + zMax) }),
                  { xMax - xMin + 0.9F * b, 0.75F * b, zMax - zMin + pitch_ * 0.8F });
        plenumPart_ = addPart(std::move(plenum), SceneMaterial::intakeFlow, layerIntake);
    }
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
        out[cylinder.runnerPart].intensity = static_cast<float>(std::clamp(intakeLift / maximumLift, 0.0, 1.0));
    }
    // The plenum glows with the mean draw of its runners.
    if (plenumPart_ >= 0 && !cylinders_.empty()) {
        float draw = 0.0F;
        for (const auto& cylinder : cylinders_) draw += out[cylinder.runnerPart].intensity;
        out[static_cast<std::size_t>(plenumPart_)].intensity = 0.5F * draw / static_cast<float>(cylinders_.size());
    }
}

void EngineModel3D::computeBounds() {
    std::vector<SceneInstance> instances;
    pose({}, instances);
    bounds_ = { { 1.0e9F, 1.0e9F, 1.0e9F }, { -1.0e9F, -1.0e9F, -1.0e9F } };
    for (const auto& instance : instances) {
        if (parts_[instance.part].material == SceneMaterial::flame) continue;
        for (const auto& vertex : parts_[instance.part].mesh.vertices) {
            const auto p = instance.transform.transformPoint(vertex.position);
            bounds_.minimum = { std::min(bounds_.minimum.x, p.x), std::min(bounds_.minimum.y, p.y), std::min(bounds_.minimum.z, p.z) };
            bounds_.maximum = { std::max(bounds_.maximum.x, p.x), std::max(bounds_.maximum.y, p.y), std::max(bounds_.maximum.z, p.z) };
        }
    }
}

} // namespace enginelab::render
