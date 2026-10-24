#include <enginelab/render/EngineModel3D.hpp>
#include <enginelab/foundation/TwoStrokePorts.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <numeric>
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
[[nodiscard]] float burnEnvelope(double phaseDegrees, double cycleDegrees) noexcept {
    const auto p = static_cast<float>(phaseDegrees);
    const auto sparkRamp = static_cast<float>(cycleDegrees) - 15.0F;
    if (p < 75.0F) return std::exp(-p / 24.0F) * std::min(1.0F, p / 5.0F + 0.25F);
    if (p > sparkRamp) return (p - sparkRamp) / 15.0F * 0.25F;
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
    radialLike_ = config_.layout == EngineLayout::radial;
    float bore = 0.0F;
    for (const auto& cylinder : config_.cylinders) bore = std::max(bore, static_cast<float>(cylinder.boreMm));
    // A two-stroke's barrels stand apart: its transfers are cast in their flanks.
    pitch_ = bore * (config_.cycle == EngineCycle::twoStroke && !radialLike_ ? 1.5F : 1.25F);
    buildCylinders();
    buildCrankshaft();
    buildStructure();
    buildDucts();
    buildGearbox();
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
    const auto cycle = cycleDegreesOf(config_.cycle);
    return std::fmod(std::fmod(crankAngleDegrees - cylinders_[cylinder].crankOffsetDegrees, cycle) + cycle, cycle);
}

void EngineModel3D::buildCylinders() {
    const auto count = config_.cylinders.size();
    std::vector<float> station(count), shift(count);
    float lastStation = 0.0F;
    // More than two banks (a W): banks next to each other in angle are only
    // 15-30 deg apart, so their cylinders interleave along the crank, half a
    // pitch apart, as a VR's do.
    std::vector<float> bankShift(config_.banks.size(), 0.0F);
    if (config_.banks.size() > 2U && !radialLike_) {
        std::vector<std::size_t> byAngle(config_.banks.size());
        std::iota(byAngle.begin(), byAngle.end(), std::size_t { 0 });
        std::sort(byAngle.begin(), byAngle.end(), [this](std::size_t a, std::size_t b) {
            return config_.banks[a].angleDegrees < config_.banks[b].angleDegrees;
        });
        for (std::size_t rank = 0; rank < byAngle.size(); ++rank)
            bankShift[byAngle[rank]] = (rank % 2U == 0U ? -0.25F : 0.25F) * pitch_;
    }
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
        else if (bank != nullptr && bankIndex < bankShift.size())
            shift[i] = bankShift[bankIndex];
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
        // A two-stroke has no valves: its ports are windows in the cylinder
        // wall, each from where the piston uncovers it down to where it is
        // fully open. The exhaust faces the exhaust side; the transfers come
        // in a pair on the flanks, turned a little to the back, half their
        // width each; a cylinder intake port is on the back wall, uncovered by
        // the skirt's lower edge, its top where that edge stands at top dead
        // centre (TwoStrokePorts' intakePortOpenAreaMm2).
        const auto wristAt = [&](double fromTopDegrees) {
            const auto at = evaluateCylinderKinematics(config_, reference_, i,
                reference_.topDeadCentreAngleDegrees[i] + fromTopDegrees, 0.0);
            return world(at.wristPinXMm, at.wristPinYMm, cylinder.z);
        };
        const auto crownAt = [&](double fromTopDegrees) { return wristAt(fromTopDegrees) + axis * cylinder.compressionHeight; };
        const auto window = [&](Mesh& mesh, Vec3 facing, double widthMm, Vec3 upper, Vec3 lower) {
            const auto tangent = normalise(cross(axis, facing));
            const auto centre = (upper + lower) * 0.5F + facing * (0.52F * b);
            appendBox(mesh, Mat4::basis(tangent, facing, cross(tangent, facing), centre),
                      { std::clamp(static_cast<float>(widthMm), 1.0F, 0.9F * b), 0.05F * b,
                        std::max(1.0F, length(upper - lower)) });
            return (upper + lower) * 0.5F + facing * (0.5F * b);
        };
        const auto twoStrokeCycle = config_.cycle == EngineCycle::twoStroke;
        if (twoStrokeCycle) {
            const auto& ports = config_.twoStroke;
            const auto atBottom = crownAt(180.0);
            Mesh transfers, exhaust;
            // About 100 degrees round from the exhaust.
            cylinder.transferFacing = normalise(side * -0.17F + lateral * 0.98F);
            const auto transferWidth = 0.5 * transferPortWidthMm(ports, config);
            const auto transferTop = crownAt(ports.transferPortOpenDegrees);
            cylinder.transferWindowCentre = window(transfers, cylinder.transferFacing, transferWidth, transferTop, atBottom);
            (void)window(transfers, normalise(side * -0.17F - lateral * 0.98F), transferWidth, transferTop, atBottom);
            if (ports.intake == TwoStrokeIntake::cylinderPort)
                cylinder.intakeWindowCentre = window(transfers, -side, intakePortWidthMm(ports, config),
                    wristAt(0.0) - axis * skirt, wristAt(ports.intakePortOpenDegreesBtdc) - axis * skirt);
            const auto exhaustTop = crownAt(ports.exhaustPortOpenDegrees);
            const auto exhaustWidth = exhaustPortWidthMm(ports, config);
            cylinder.exhaustWindowCentre = window(exhaust, side, exhaustWidth, exhaustTop, atBottom);
            cylinder.intakeValvePart = addPart(std::move(transfers), SceneMaterial::intakeValve, layerValves);
            cylinder.exhaustValvePart = addPart(std::move(exhaust), SceneMaterial::exhaustValve, layerValves);
            if (hasPowerValve(ports)) {
                // A guillotine just outside the window, its lower edge on
                // the port's roof when open; shut, it drops to the roof the
                // valve gives, which opens the port later.
                const auto drop = crownAt(ports.exhaustPortOpenDegrees) - crownAt(ports.powerValveClosedExhaustPortOpenDegrees);
                const auto height = length(drop) + 0.16F * b;
                const auto tangent = normalise(cross(axis, side));
                Mesh blade;
                appendBox(blade, Mat4::basis(tangent, side, axis, exhaustTop + axis * (0.5F * height) + side * (0.58F * b)),
                          { std::clamp(static_cast<float>(exhaustWidth), 1.0F, 0.9F * b) * 1.04F, 0.04F * b, height });
                cylinder.powerValvePart = addPart(std::move(blade), SceneMaterial::steel, layerValves);
                cylinder.powerValveDrop = drop;
            }
            // The crank's centre level with this cylinder, and the crankcase
            // round it just clear of the webs (buildCrankshaft's counterweight).
            const auto k = evaluateCylinderKinematics(config_, reference_, i, 0.0, 0.0);
            const auto crankshaft = std::find_if(config_.crankshafts.begin(), config_.crankshafts.end(),
                [&k](const CrankshaftConfig& item) { return item.id == k.crankshaftId; });
            cylinder.crankCentre = crankshaft == config_.crankshafts.end() ? Vec3 { 0.0F, 0.0F, cylinder.z }
                : world(crankshaft->positionXMm, crankshaft->positionYMm, cylinder.z);
            const auto throwRadius = 0.5F * static_cast<float>(config.strokeMm);
            const auto counterweight = std::min(0.62F * b, (static_cast<float>(config.connectingRodMm) - throwRadius - 0.32F * b) * 0.9F);
            crankcaseRadius_ = std::max(crankcaseRadius_, std::max(counterweight, throwRadius + 0.27F * b) + 0.1F * b);
            cylinder.barrelRadius = 0.70F * b;
        } else {
            cylinder.intakeValvePart = addPart(valves(config.intakeValveCount, config.intakeValveDiameterMm,
                cylinder.intakeValveAxis, -1.0F, true), SceneMaterial::intakeValve, layerValves);
            cylinder.exhaustValvePart = addPart(valves(config.exhaustValveCount, config.exhaustValveDiameterMm,
                cylinder.exhaustValveAxis, 1.0F, false), SceneMaterial::exhaustValve, layerValves);
        }

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
        std::uint32_t journalId {};
    };
    std::vector<Group> groups;
    for (std::size_t i = 0; i < cylinders_.size(); ++i) {
        const auto& config = config_.cylinders[i];
        if (config.connectingRodType == ConnectingRodType::articulated) continue;
        const auto k = evaluateCylinderKinematics(config_, reference_, i, 0.0, 0.0);
        const auto pin = world(k.crankPinXMm, k.crankPinYMm, 0.0F);
        const auto z = cylinders_[i].z;
        // Rods on one journal share its pin. A journal per cylinder pointing
        // the same way is shared too within a row (a V's banks sit 0.45 pitch
        // apart), not across rows (0.55 pitch): two neighbouring rows whose
        // pins point alike, as in a V12's middle, are two throws.
        const auto shared = std::find_if(groups.begin(), groups.end(), [&](const Group& group) {
            if (group.crankshaftId != k.crankshaftId || length(group.pin - pin) >= 0.5F) return false;
            if (group.journalId == config.crankJournalId) return true;
            return z - group.zMax < pitch_ * 0.5F && group.zMin - z < pitch_ * 0.5F;
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
                           static_cast<float>(config.connectingRodMm), i, config.crankJournalId });
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
        flywheelEndZ_ = std::max(flywheelEndZ_, end + flywheelThickness);
        flywheelRadius_ = std::max(flywheelRadius_, flywheelRadius);
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
    if (config_.cycle == EngineCycle::twoStroke && !radialLike_) {
        buildTwoStrokeStructure();
        return;
    }
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

void EngineModel3D::buildTwoStrokeStructure() {
    // A two-stroke is a round crankcase with no sump (the crankcase is its
    // charge pump, the oil comes with the fuel), a separate barrel on it for
    // each cylinder with the transfers cast in its flanks, and a thin head
    // over each with only its plug in it.
    Mesh heads, block;
    std::vector<std::vector<std::size_t>> cranks;
    for (std::size_t i = 0; i < cylinders_.size(); ++i) {
        const auto centre = cylinders_[i].crankCentre;
        const auto same = std::find_if(cranks.begin(), cranks.end(), [&](const std::vector<std::size_t>& group) {
            const auto other = cylinders_[group.front()].crankCentre;
            return std::abs(other.x - centre.x) < 0.5F && std::abs(other.y - centre.y) < 0.5F;
        });
        if (same == cranks.end()) cranks.push_back({ i });
        else same->push_back(i);
    }
    const auto radius = crankcaseRadius_;
    for (const auto& group : cranks) {
        float zMin = 1.0e9F, zMax = -1.0e9F, b = 0.0F;
        for (const auto index : group) {
            zMin = std::min(zMin, cylinders_[index].z);
            zMax = std::max(zMax, cylinders_[index].z);
            b = std::max(b, cylinders_[index].bore);
        }
        // To just short of the main shaft's ends, where the pulley and the
        // flywheel stand outside it.
        const auto start = zMin - 0.45F * pitch_;
        const auto span = zMax - zMin + 0.9F * pitch_;
        const auto centre = cylinders_[group.front()].crankCentre;
        const auto base = Vec3 { centre.x, centre.y, start };
        appendCylinder(block, cylinderFrame(base, zAxis), radius, radius, span, 48);
        solids_.push_back(cylinderSolid(base, zAxis, radius, span));
    }
    for (auto& c : cylinders_) {
        const auto b = c.bore;
        const auto axis = c.axis;
        const auto side = c.exhaustSide;
        const auto tangent = normalise(cross(axis, side));
        // Standing in the crankcase's top, so the two read as one casting.
        c.barrelBottom = 0.8F * radius;
        const auto bottom = c.crankCentre + axis * c.barrelBottom;
        const auto height = std::max(0.2F * b, dot(c.deckCentre - bottom, axis));
        appendCylinder(block, cylinderFrame(bottom, axis), c.barrelRadius, c.barrelRadius, height, 40, false, false);
        solids_.push_back(cylinderSolid(bottom, axis, c.barrelRadius, height));
        // The base flange it is bolted down by.
        const auto flange = bottom + axis * (0.2F * radius + 0.03F * b);
        appendBox(block, Mat4::basis(side, axis, tangent, flange), { 1.55F * b, 0.06F * b, 1.4F * b });
        if (c.powerValvePart != 0xFFFFU) {
            // The valve's housing and actuator over the exhaust spigot.
            const auto housing = c.exhaustWindowCentre + side * (0.78F * b) + axis * (0.36F * b);
            appendBox(block, Mat4::basis(tangent, side, axis, housing), { 0.9F * b, 0.32F * b, 0.26F * b });
            solids_.push_back(boxSolid(tangent, side, axis, housing, { 0.9F * b, 0.32F * b, 0.26F * b }));
        }
        constexpr float headHeight = 0.26F;
        appendCylinder(heads, cylinderFrame(c.deckCentre, axis), 0.72F * b, 0.64F * b, headHeight * b, 40);
        appendCylinder(heads, cylinderFrame(c.deckCentre + axis * (headHeight * b), axis), 0.09F * b, 0.09F * b,
                       0.16F * b, 12);
        solids_.push_back(cylinderSolid(c.deckCentre, axis, 0.72F * b, headHeight * b));
    }
    (void)addPart(std::move(block), SceneMaterial::blockShell, layerStructure);
    (void)addPart(std::move(heads), SceneMaterial::headShell, layerStructure);
}

void EngineModel3D::buildDucts() {
    Mesh exhaust;
    float bore = 0.0F;
    // A two-stroke V's carburettors stand in the valley between its banks,
    // pointing out of it, as on every V-twin and V4 two-stroke built.
    std::optional<Vec3> valley;
    if (config_.cycle == EngineCycle::twoStroke && !radialLike_) {
        std::vector<Vec3> bankAxes;
        for (const auto& c : cylinders_) {
            const auto axis = normalise(Vec3 { c.axis.x, c.axis.y, 0.0F });
            if (std::none_of(bankAxes.begin(), bankAxes.end(), [&](Vec3 seen) { return dot(seen, axis) > 0.999F; }))
                bankAxes.push_back(axis);
        }
        if (bankAxes.size() == 2U && dot(bankAxes[0], bankAxes[1]) > -0.95F)
            valley = normalise(bankAxes[0] + bankAxes[1]);
    }
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
        auto intakePort = cylinder.deckCentre + axis * (0.42F * b) - side * (0.70F * b);
        auto intakeOut = radialLike_ ? -side : normalise(axis * 0.35F - side);
        const auto twoStrokeBarrel = config_.cycle == EngineCycle::twoStroke && !radialLike_;
        if (twoStrokeBarrel) {
            // The transfer duct, in gas-flow order: out of the crankcase's
            // top into the barrel's flank, up its wall and in at the window;
            // its twin is the mirror image on the other flank.
            const auto facing = cylinder.transferFacing;
            const auto wall = 0.5F * (0.56F * b + cylinder.barrelRadius) + 0.04F * b;
            const auto base = cylinder.crankCentre + axis * (0.55F * crankcaseRadius_) + facing * wall;
            const auto window = cylinder.transferWindowCentre;
            const auto belowWindow = window + facing * (wall - 0.5F * b) - axis * (0.30F * b);
            const auto startOfRise = cylinder.crankCentre + axis * (cylinder.barrelBottom + 0.10F * b) + facing * wall;
            const auto lateral = normalise(cross(axis, side));
            if (config_.twoStroke.intake == TwoStrokeIntake::cylinderPort) {
                // Through the back wall of the barrel, under the transfers;
                // the carburettor on its spigot, a reed in it if fitted.
                intakeOut = valley ? normalise(*valley - side * 0.5F) : -side;
                intakePort = cylinder.intakeWindowCentre - side * (cylinder.barrelRadius - 0.5F * b + 0.04F * b);
            } else {
                // A reed block on the crankcase's back, under the barrel,
                // the carburettor behind it pointing back and up; in a V's
                // valley, out of the valley clear of both barrels.
                intakeOut = valley ? *valley : normalise(-side + axis * 0.5F);
                auto reach = crankcaseRadius_;
                if (valley) {
                    const auto sine = std::sqrt(std::max(0.05F, 1.0F - dot(*valley, axis) * dot(*valley, axis)));
                    reach = std::max(reach, (cylinder.barrelRadius + 0.6F * intakeDiameter) / sine);
                }
                intakePort = cylinder.crankCentre + intakeOut * reach;
            }
            PortAnchor anchor { config.id, intakePort, intakeOut, intakeDiameter, window };
            anchor.transferDuct = { base, startOfRise, belowWindow, window };
            anchor.transferMirrorPoint = cylinder.crankCentre;
            anchor.transferMirrorNormal = lateral;
            // Upright in a valley, a slide rises along the crank.
            anchor.carburettorUp = valley ? zAxis : yAxis;
            intakePorts_.push_back(std::move(anchor));
        } else {
            intakePorts_.push_back({ config.id, intakePort, intakeOut, intakeDiameter,
                                     cylinder.deckCentre + axis * (0.12F * b) - side * (0.22F * b) });
        }

        if (twoStrokeBarrel) {
            // Exhaust port: out of its window through the barrel's wall to the
            // spigot the pipe is clamped on, sloping a little down.
            const auto exhaustDiameter = 0.45F * b;
            const auto exhaustInner = cylinder.exhaustWindowCentre;
            const auto spigot = exhaustInner + side * (cylinder.barrelRadius - 0.5F * b + 0.06F * b);
            const auto exhaustOut = normalise(side - axis * 0.2F);
            const auto exhaustEnd = spigot + exhaustOut * (0.35F * b);
            appendTube(exhaust, catmullRom({ exhaustInner, spigot, exhaustEnd }, 10), { exhaustDiameter * 0.5F }, 18);
            exhaustPorts_.push_back({ config.id, exhaustEnd, exhaustOut, exhaustDiameter, exhaustInner });
        } else {
            // Exhaust port stub: out of the head on the exhaust side.
            const auto exhaustDiameter = 0.34F * b;
            const auto exhaustInner = cylinder.deckCentre + axis * (0.1F * b) + side * (0.22F * b);
            const auto exhaustPort = cylinder.deckCentre + axis * (0.3F * b) + side * (0.70F * b);
            const auto exhaustOut = normalise(side - axis * 0.25F);
            const auto exhaustEnd = exhaustPort + exhaustOut * (0.6F * b);
            appendTube(exhaust, catmullRom({ exhaustInner, exhaustPort, exhaustEnd }, 10), { exhaustDiameter * 0.5F }, 18);
            exhaustPorts_.push_back({ config.id, exhaustEnd, exhaustOut, exhaustDiameter, exhaustInner });
        }

        // The injector. A port injector sits in the runner just out of the
        // head, above it, spraying at the back of the intake valves; a direct
        // injector sits in the head on the intake side, under the valves,
        // spraying across the chamber. A cylinder fed by a carburettor has
        // none: its fuel comes with the air.
        const auto carburetted = std::any_of(config_.intakePaths.begin(), config_.intakePaths.end(),
            [&config](const IntakePathConfig& path) {
                return path.geometry.carburettor.enabled
                    && std::find(path.cylinderIds.begin(), path.cylinderIds.end(), config.id) != path.cylinderIds.end();
            });
        if (carburetted) continue;
        const auto direct = config_.injection.mode == InjectionMode::direct;
        // A ported two-stroke's sprays from its reed manifold through the reeds.
        const auto reeds = !direct && !intakePorts_.back().transferDuct.empty();
        auto& drawn = cylinders_[i];
        drawn.injectorTip = direct ? cylinder.deckCentre + axis * (0.10F * b) - side * (0.42F * b)
            : reeds ? intakePort + intakeOut * (0.45F * b) + intakePorts_.back().carburettorUp * (0.30F * b)
                    : intakePort + intakeOut * (0.30F * b) + axis * (0.22F * b);
        const auto target = direct ? cylinder.deckCentre - axis * (0.35F * b) + side * (0.10F * b)
            : reeds ? intakePort
                    : intakePorts_.back().inner;
        drawn.injectorAim = normalise(target - drawn.injectorTip);
        Mesh injector;
        const auto nozzle = 0.10F * b;
        const auto bodyLength = 0.34F * b;
        const auto back = drawn.injectorTip - drawn.injectorAim * (nozzle + bodyLength + 0.12F * b);
        const auto frame = Mat4::frameAlongY(back, drawn.injectorAim);
        // Connector, body, then the nozzle ending at the tip.
        appendCylinder(injector, frame, 0.07F * b, 0.07F * b, 0.12F * b, 12);
        appendCylinder(injector, frame * Mat4::translation({ 0.0F, 0.12F * b, 0.0F }), 0.085F * b, 0.07F * b,
                       bodyLength, 16);
        appendCylinder(injector, frame * Mat4::translation({ 0.0F, 0.12F * b + bodyLength, 0.0F }), 0.04F * b,
                       0.025F * b, nozzle, 12);
        drawn.injectorPart = addPart(std::move(injector), SceneMaterial::steel, direct ? layerCombustion : layerIntake);
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
    add(layoutExhaust(config_, exhaustPorts_, bore, intakeTubes, avoid, &turbos_));
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
    // A unit the layout found no collector for is not drawn.
    std::erase_if(turbos_, [](const TurboPlacement& turbo) { return !turbo.placed; });
    turboParts_.clear();
    const auto& forced = config_.forcedInduction;
    // Without a blade count, a typical wheel: it is only drawn.
    const auto turbineBlades = forced.turbineBladeCount > 0U ? static_cast<int>(forced.turbineBladeCount) : 10;
    const auto compressorBlades = forced.compressorBladeCount > 0U ? static_cast<int>(forced.compressorBladeCount) : 7;
    for (const auto& turbo : turbos_) {
        Mesh turbineHousing, compressorHousing, turbineWheel, compressorWheel;
        appendTurboHousings(turbineHousing, compressorHousing, turbo);
        appendTurboWheel(turbineWheel, turbo.turbineWheelRadius, 0.8F * turbo.turbineWidth, turbineBlades, 1.0F);
        appendTurboWheel(compressorWheel, turbo.compressorWheelRadius, 0.7F * turbo.compressorWidth,
            compressorBlades, -1.0F);
        turboParts_.push_back({ addPart(std::move(turbineHousing), SceneMaterial::turboHousing, layerExhaust),
            addPart(std::move(compressorHousing), SceneMaterial::turboHousing, layerIntake),
            addPart(std::move(turbineWheel), SceneMaterial::forged, layerExhaust),
            addPart(std::move(compressorWheel), SceneMaterial::steel, layerIntake) });
        for (const auto& solid : turboSolids(turbo)) solids_.push_back(solid);
    }
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

void EngineModel3D::buildGearbox() {
    const auto& ratios = config_.transmission.gearRatios;
    if (ratios.size() < 2U || throws_.empty() || flywheelRadius_ <= 0.0F) return;
    float b = 0.0F, zMin = 1.0e9F, zMax = -1.0e9F;
    for (const auto& c : cylinders_) {
        b = std::max(b, c.bore);
        zMin = std::min(zMin, c.z);
        zMax = std::max(zMax, c.z);
    }
    const auto origin = throws_.front().origin;
    const auto pairCount = ratios.size();
    // Shaft centre distance, gear face width and pair pitch, in bores: an
    // MT-07 box runs about 70 mm between its shafts for an 80 mm bore.
    const auto centre = 0.95F * b;
    const auto width = 0.15F * b;
    const auto spacing = 0.26F * b;
    const auto span = spacing * static_cast<float>(pairCount + 1U);
    const auto turn = std::remainder(throwRotation(throws_.front(), 10.0) - throwRotation(throws_.front(), 0.0), 2.0F * pi);
    const auto crankSense = turn >= 0.0F ? 1.0F : -1.0F;
    const auto motorcycle = config_.vehicle.massKg < 500.0;
    float zStart = 0.0F;
    if (motorcycle) {
        // Stacked behind the crank, away from the exhaust, driven by a
        // primary gear: the input shaft turns against the crank.
        auto back = cylinders_.front().exhaustSide * -1.0F;
        back = Vec3 { back.x, back.y, 0.0F };
        back = length(back) > 1.0e-3F ? normalise(back) : Vec3 { -1.0F, 0.0F, 0.0F };
        // Behind a V's front bank stands its rear bank: the box turns about
        // the crank, the least it can, until its case clears every barrel
        // (a V-twin's or a V4's sits under the rear cylinders).
        float smallestRatio = 1.0e9F, largestRatio = 0.0F, stroke = 0.0F;
        for (const auto ratio : ratios) {
            smallestRatio = std::min(smallestRatio, static_cast<float>(std::max(0.2, ratio)));
            largestRatio = std::max(largestRatio, static_cast<float>(std::max(0.2, ratio)));
        }
        for (const auto& cylinder : config_.cylinders) stroke = std::max(stroke, static_cast<float>(cylinder.strokeMm));
        const auto reachMargin = 0.12F * b / pi + 0.12F * b;
        const auto inputReachEstimate = centre / (1.0F + smallestRatio) + reachMargin;
        const auto outputReachEstimate = centre * largestRatio / (1.0F + largestRatio) + reachMargin;
        const auto axesFor = [&](Vec3 direction) {
            const auto input = Vec3 { origin.x, origin.y, 0.0F } + direction * (flywheelRadius_ + 0.45F * b)
                - yAxis * (0.1F * b);
            return std::pair { input, input + normalise(direction * 0.55F - yAxis * 0.85F) * centre };
        };
        // Least distance in the crank's plane from the case to a barrel
        // (from above the crank's sweep to the deck), less the barrel's radius.
        const auto clearance = [&](Vec3 direction) {
            const auto [input, output] = axesFor(direction);
            const auto along = normalise(output - input);
            const auto across = Vec3 { -along.y, along.x, 0.0F };
            const auto low = -inputReachEstimate, high = centre + outputReachEstimate;
            const auto middle = input + along * (0.5F * (low + high));
            const auto halfAlong = 0.5F * (high - low);
            const auto halfAcross = std::max(inputReachEstimate, outputReachEstimate);
            auto least = 1.0e9F;
            for (const auto& c : cylinders_) {
                const auto axis = normalise(Vec3 { c.axis.x, c.axis.y, 0.0F });
                const auto start = Vec3 { origin.x, origin.y, 0.0F } + axis * (0.5F * stroke + 0.25F * c.bore);
                const auto end = Vec3 { c.deckCentre.x, c.deckCentre.y, 0.0F };
                for (int k = 0; k <= 16; ++k) {
                    const auto point = start + (end - start) * (static_cast<float>(k) / 16.0F) - middle;
                    const auto u = std::max(0.0F, std::abs(dot(point, along)) - halfAlong);
                    const auto v = std::max(0.0F, std::abs(dot(point, across)) - halfAcross);
                    least = std::min(least, std::sqrt(u * u + v * v) - 0.6F * c.bore);
                }
            }
            return least;
        };
        const auto turned = [](Vec3 direction, float radians) {
            const auto cosine = std::cos(radians), sine = std::sin(radians);
            return Vec3 { cosine * direction.x - sine * direction.y, sine * direction.x + cosine * direction.y, 0.0F };
        };
        auto chosen = back;
        if (clearance(back) < 0.0F) {
            auto best = clearance(back);
            bool clear = false;
            for (int step = 1; step <= 36 && !clear; ++step) {
                const auto angle = static_cast<float>(step) * (5.0F * pi / 180.0F);
                // Either way round, downwards first at the same angle.
                auto first = turned(back, angle), second = turned(back, -angle);
                if (second.y < first.y) std::swap(first, second);
                for (const auto candidate : { first, second }) {
                    const auto room = clearance(candidate);
                    if (room >= 0.0F) {
                        chosen = candidate;
                        clear = true;
                        break;
                    }
                    if (room > best) {
                        best = room;
                        chosen = candidate;
                    }
                }
            }
        }
        std::tie(gearbox_.inputAxis, gearbox_.outputAxis) = axesFor(chosen);
        zStart = 0.5F * (zMin + zMax) - 0.5F * span;
        inputSense_ = -crankSense;
    } else {
        // In line behind the flywheel and clutch, the layshaft below.
        gearbox_.inputAxis = Vec3 { origin.x, origin.y, 0.0F };
        gearbox_.outputAxis = gearbox_.inputAxis - yAxis * centre;
        zStart = flywheelEndZ_ + 0.35F * b;
        inputSense_ = crankSense;
    }
    const auto toOutput = normalise(gearbox_.outputAxis - gearbox_.inputAxis);
    // Angle of the contact direction from +Y, counter-clockwise about +Z.
    const auto contact = std::atan2(-toOutput.x, toOutput.y);

    const auto appendGear = [](Mesh& mesh, float z, int teeth, float toothModule, float faceWidth) {
        const auto pitchRadius = 0.5F * toothModule * static_cast<float>(teeth);
        appendShaft(mesh, {}, z - 0.5F * faceWidth, faceWidth, pitchRadius - 1.1F * toothModule, std::max(24, teeth * 2));
        for (int t = 0; t < teeth; ++t) {
            const auto a = 2.0F * pi * static_cast<float>(t) / static_cast<float>(teeth);
            appendBox(mesh, Mat4::translation({ 0.0F, 0.0F, z }) * Mat4::rotationZ(a)
                          * Mat4::translation({ 0.0F, pitchRadius - 0.05F * toothModule, 0.0F }),
                      { 1.4F * toothModule, 2.1F * toothModule, faceWidth });
        }
    };

    Mesh inputMesh, outputShaft, collar, casing;
    float inputReach = 0.0F, outputReach = 0.0F;
    const auto targetToothModule = 0.12F * b / pi;
    for (std::size_t k = 0; k < pairCount; ++k) {
        const auto ratio = static_cast<float>(std::max(0.2, ratios[k]));
        const auto inputTarget = centre / (1.0F + ratio);
        const auto inputTeeth = std::max(9, static_cast<int>(std::lround(2.0F * inputTarget / targetToothModule)));
        const auto outputTeeth = std::max(9, static_cast<int>(std::lround(static_cast<float>(inputTeeth) * ratio)));
        const auto toothModule = 2.0F * centre / static_cast<float>(inputTeeth + outputTeeth);
        const auto z = zStart + spacing * static_cast<float>(k + 1U);
        gearbox_.inputTeeth.push_back(inputTeeth);
        gearbox_.outputTeeth.push_back(outputTeeth);
        gearbox_.pairZ.push_back(z);
        appendGear(inputMesh, z, inputTeeth, toothModule, width);
        Mesh outputGear;
        appendGear(outputGear, z, outputTeeth, toothModule, width);
        outputGearParts_.push_back(addPart(std::move(outputGear), SceneMaterial::forged, layerMechanism));
        // Input tooth 0 at the contact meets a gap of the output gear.
        const auto share = static_cast<float>(inputTeeth) / static_cast<float>(outputTeeth);
        outputGearPhase_.push_back(contact * (1.0F + share) + pi + pi / static_cast<float>(outputTeeth));
        inputReach = std::max(inputReach, 0.5F * toothModule * static_cast<float>(inputTeeth) + toothModule);
        outputReach = std::max(outputReach, 0.5F * toothModule * static_cast<float>(outputTeeth) + toothModule);
    }

    // Shafts: the input carries the clutch disc (car) or the clutch pack
    // (motorcycle), the output the sprocket of a motorcycle.
    const auto shaftRadius = 0.1F * b;
    const auto zEnd = zStart + span;
    if (motorcycle) {
        appendShaft(inputMesh, {}, zStart - 0.05F * b, span + 0.45F * b, shaftRadius, 20);
        appendShaft(inputMesh, {}, zEnd + 0.1F * b, 0.3F * b, 0.7F * b, 48);
        appendShaft(outputShaft, {}, zStart - 0.35F * b, span + 0.4F * b, 1.2F * shaftRadius, 20);
        appendGear(outputShaft, zStart - 0.25F * b, 16, 0.07F * b, 0.08F * b);
    } else {
        appendShaft(inputMesh, {}, flywheelEndZ_ + 0.02F * b, zEnd - flywheelEndZ_, shaftRadius, 20);
        appendShaft(inputMesh, {}, flywheelEndZ_ + 0.05F * b, 0.06F * b, 0.75F * flywheelRadius_, 48);
        appendShaft(outputShaft, {}, zStart + 0.05F * b, span + 0.5F * b, 1.2F * shaftRadius, 20);
    }
    // The selector collar: a ring with dogs, slid against the engaged gear.
    const auto collarWidth = 0.07F * b;
    appendShaft(collar, {}, -0.5F * collarWidth, collarWidth, 0.26F * b, 32);
    for (int dog = 0; dog < 6; ++dog)
        appendBox(collar, Mat4::rotationZ(2.0F * pi * static_cast<float>(dog) / 6.0F) * Mat4::translation({ 0.0F, 0.2F * b, 0.0F }),
                  { 0.06F * b, 0.08F * b, collarWidth + 0.03F * b });
    for (const auto z : gearbox_.pairZ) collarZ_.push_back(z - 0.5F * width - 0.5F * collarWidth - 0.005F * b);
    collarZ_.push_back(gearbox_.pairZ[0] + 0.5F * spacing);

    // The case: a box round both gear stacks.
    const auto side = cross(zAxis, toOutput);
    const auto margin = 0.12F * b;
    const auto widest = std::max(inputReach, outputReach) + margin;
    const auto low = -(inputReach + margin), high = centre + outputReach + margin;
    const auto middle = gearbox_.inputAxis + toOutput * (0.5F * (low + high)) + zAxis * (zStart + 0.5F * span);
    appendBox(casing, Mat4::basis(toOutput, side, zAxis, middle), { high - low, 2.0F * widest, span });

    gearboxParts_[0] = addPart(std::move(casing), SceneMaterial::blockShell, layerStructure);
    gearboxParts_[1] = addPart(std::move(inputMesh), SceneMaterial::forged, layerMechanism);
    gearboxParts_[2] = addPart(std::move(outputShaft), SceneMaterial::steel, layerMechanism);
    gearboxParts_[3] = addPart(std::move(collar), SceneMaterial::intakeValve, layerMechanism);
    // Out of the engine framing: the camera views keep framing the engine.
    farParts_.resize(parts_.size(), true);
    gearbox_.placed = true;
}

float EngineModel3D::outputGearRadians(std::size_t pair, double inputDegrees) const noexcept {
    if (pair >= outputGearPhase_.size()) return 0.0F;
    const auto share = static_cast<double>(gearbox_.inputTeeth[pair]) / static_cast<double>(gearbox_.outputTeeth[pair]);
    const auto degrees = std::fmod(static_cast<double>(inputSense_) * inputDegrees * share, 360.0);
    return -static_cast<float>(degrees * (std::numbers::pi / 180.0)) + outputGearPhase_[pair];
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
        // A two-stroke's ports are in the cylinder wall: its valves stay shut.
        const auto valves = config_.cycle != EngineCycle::twoStroke;
        const auto intakeLift = !valves ? 0.0 : profiledValveLiftMm(phase, 360.0 + cams.intakeCenterlineDegrees,
            high ? cams.highIntakeDurationDegrees : cams.intakeDurationDegrees,
            high ? cams.highIntakeLiftMm : cams.intakeLiftMm,
            high && !cams.highIntakeLiftProfile.empty() ? cams.highIntakeLiftProfile : cams.intakeLiftProfile);
        const auto exhaustLift = !valves ? 0.0 : profiledValveLiftMm(phase, 360.0 - cams.exhaustCenterlineDegrees,
            high ? cams.highExhaustDurationDegrees : cams.exhaustDurationDegrees,
            high ? cams.highExhaustLiftMm : cams.exhaustLiftMm,
            high && !cams.highExhaustLiftProfile.empty() ? cams.highExhaustLiftProfile : cams.exhaustLiftProfile);
        out[cylinder.intakeValvePart].transform = Mat4::translation(cylinder.intakeValveAxis * -static_cast<float>(intakeLift));
        out[cylinder.exhaustValvePart].transform = Mat4::translation(cylinder.exhaustValveAxis * -static_cast<float>(exhaustLift));

        const auto burn = burnEnvelope(phase, cycleDegreesOf(config_.cycle)) * std::clamp(input.combustion[i], 0.0F, 1.0F);
        const auto scale = 0.6F + 0.5F * burn;
        const auto x = normalise(cross(cylinder.axis, zAxis));
        out[cylinder.flamePart].transform = Mat4::basis(x * scale, cylinder.axis * (scale * 0.45F), zAxis * scale,
                                                         cylinder.chamberCentre);
        out[cylinder.flamePart].intensity = burn;
        const auto maximumLift = std::max(0.1, high ? cams.highIntakeLiftMm : cams.intakeLiftMm);
        auto opening = std::clamp(intakeLift / maximumLift, 0.0, 1.0);
        if (!valves) {
            // A two-stroke's transfer duct glows as far as its window is open.
            const auto& config = config_.cylinders[i];
            const auto open = config_.twoStroke.transferPortOpenDegrees;
            const auto fromTop = std::fmod(std::fmod(angle - reference_.topDeadCentreAngleDegrees[i], 360.0) + 360.0, 360.0);
            opening = std::clamp(portOpenAreaMm2(config, open, 1.0, 1.0, fromTop)
                / std::max(1.0e-6, portOpenAreaMm2(config, open, 1.0, 1.0, 180.0)), 0.0, 1.0);
        }
        if (cylinder.runnerPart < out.size()) out[cylinder.runnerPart].intensity = static_cast<float>(opening);
        if (cylinder.powerValvePart < out.size())
            out[cylinder.powerValvePart].transform = Mat4::translation(cylinder.powerValveDrop
                * -static_cast<float>(1.0 - std::clamp(input.powerValveOpening, 0.0, 1.0)));
    }
    for (const auto part : outletFlameParts_) out[part].intensity = std::clamp(input.afterfire, 0.0F, 1.0F);
    for (std::size_t index = 0; index < turbos_.size(); ++index) {
        // Both wheels turn with the shaft, about its axis; each unit a little
        // out of phase with the next, so a pair does not look geared together.
        const auto& turbo = turbos_[index];
        const auto shaft = static_cast<float>(std::fmod(input.turboShaftDegrees + 47.0 * static_cast<double>(turbo.unit),
            360.0) * (pi / 180.0F));
        const auto axis = turbo.axis;
        const auto x = turbo.radial * std::cos(shaft) + cross(axis, turbo.radial) * std::sin(shaft);
        const auto z = cross(x, axis);
        out[turboParts_[index][2]].transform = Mat4::basis(x, axis, z, turbo.turbineCentre);
        out[turboParts_[index][3]].transform = Mat4::basis(x, axis, z, turbo.compressorCentre);
    }
    if (supercharger_.placed) {
        // Geared to the crank.
        const auto impeller = static_cast<float>(superchargerImpellerDegrees(input.crankAngleDegrees) * (pi / 180.0F));
        const auto axis = supercharger_.axis;
        const auto x = supercharger_.radial * std::cos(impeller) + cross(axis, supercharger_.radial) * std::sin(impeller);
        out[superchargerParts_[1]].transform = Mat4::basis(x, axis, cross(x, axis), supercharger_.centre);
    }
    if (gearbox_.placed) {
        // Every input gear turns with its shaft, every output gear at its own
        // ratio on the output shaft; the engaged one carries the shaft and
        // the collar with it.
        const auto about = [](Vec3 axis, float radians, float z) {
            return Mat4::translation({ axis.x, axis.y, z }) * Mat4::rotationZ(radians);
        };
        const auto inputRadians = inputSense_
            * static_cast<float>(std::fmod(input.gearboxInputDegrees, 360.0) * (std::numbers::pi / 180.0));
        out[gearboxParts_[1]].transform = about(gearbox_.inputAxis, inputRadians, 0.0F);
        for (std::size_t k = 0; k < outputGearParts_.size(); ++k)
            out[outputGearParts_[k]].transform
                = about(gearbox_.outputAxis, outputGearRadians(k, input.gearboxInputDegrees), 0.0F);
        const auto engaged = input.engagedGear >= 0 && static_cast<std::size_t>(input.engagedGear) < outputGearParts_.size();
        const auto gear = engaged ? static_cast<std::size_t>(input.engagedGear) : outputGearParts_.size();
        const auto shaft = engaged ? outputGearRadians(gear, input.gearboxInputDegrees) : 0.0F;
        out[gearboxParts_[2]].transform = about(gearbox_.outputAxis, shaft, 0.0F);
        out[gearboxParts_[3]].transform = about(gearbox_.outputAxis, shaft, collarZ_[gear]);
    }
    // Plenum, airbox and inlet duct glow with the mean draw of their runners.
    for (const auto& duct : ducts_) {
        if (duct.kind != DuctKind::intakePlenum && duct.kind != DuctKind::intakeAirbox
            && duct.kind != DuctKind::intakeInletDuct)
            continue;
        float draw = 0.0F;
        int runners = 0;
        // A two-stroke's airbox is every crankcase's.
        const auto shared = duct.kind == DuctKind::intakeAirbox && config_.cycle == EngineCycle::twoStroke;
        for (const auto& other : ducts_)
            if (other.kind == DuctKind::intakeRunner && (shared || other.pathId == duct.pathId)) {
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
        if (part == c.injectorPart) return { PartRole::injector, cylinder, -1 };
        if (part == c.powerValvePart) return { PartRole::powerValve, cylinder, -1 };
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
    for (std::size_t index = 0; index < turboParts_.size(); ++index)
        if (std::find(turboParts_[index].begin(), turboParts_[index].end(), part) != turboParts_[index].end())
            return { PartRole::turbo, -1, -1, static_cast<int>(index) };
    if (std::find(superchargerParts_.begin(), superchargerParts_.end(), part) != superchargerParts_.end())
        return { PartRole::supercharger, -1, -1 };
    if (std::find(gearboxParts_.begin(), gearboxParts_.end(), part) != gearboxParts_.end()
        || std::find(outputGearParts_.begin(), outputGearParts_.end(), part) != outputGearParts_.end())
        return { PartRole::gearbox, -1, -1 };
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
