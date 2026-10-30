#include <enginelab/catalog/EngineDesign.hpp>
#include <enginelab/foundation/CrankBalance.hpp>
#include <enginelab/foundation/ForcedInductionFlow.hpp>
#include <enginelab/foundation/IntakeResize.hpp>
#include <enginelab/foundation/TurboSizing.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <numbers>
#include <numeric>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace enginelab {
namespace {

/**
 * The values a designer picks for one use. Fitted to the catalogue engines,
 * which were authored from their makers' figures (engines/<name>.engine.yaml):
 * mean piston speed at the redline (K20A 24.7, CP2 22.9, CP4 20.4, LS3 20.2,
 * 2JZ 20.6, Audi I5 21.6, Harley 21.3, Merlin 18.5 m/s; boosted petrol 1.5
 * lower, journal 2026-10-19), piston mass over bore cubed
 * (g/cm^3: cars 0.47-0.52, sport bikes 0.27-0.37, cruiser 0.55, aircraft
 * 0.71-0.76), rotating inertia per litre, cams, throttle area per litre and
 * rpm, and the family parts they use.
 */
struct UseProfile final {
    double redlinePistonSpeedMps;
    double idleRpm;
    double boreStrokeRatio;
    double intakeDurationDegrees;
    double exhaustDurationDegrees;
    double intakeCenterlineDegrees;
    double exhaustCenterlineDegrees;
    double liftOverValveDiameter;
    double pistonDensityGPerCm3;
    double inertiaKgM2PerLitre;
    double frictionCoefficient;
    double compressionBonus;
    std::uint32_t valvesPerSide;
    double throttleAreaMm2PerLitreRpm;
    bool throttlePerCylinder;
    bool motorcycle;
    double octane;
    double throttleGamma;
    double boostPressureRatio;
    const char* exhaustPart;
    const char* transmissionPart;
    const char* vehiclePart;
    const char* injectionPart;
};

[[nodiscard]] UseProfile profileFor(EngineUse use) {
    switch (use) {
    case EngineUse::roadCar:
        return { 20.5, 800.0, 1.0, 248.0, 244.0, 112.0, 116.0, 0.30, 0.50, 0.10, 0.14, 0.0, 2, 0.18, false, false,
            98.0, 1.45, 1.8, "compact_sport", "sport_6_speed", "compact_coupe", "sequential_port" };
    case EngineUse::sportCar:
        return { 22.0, 900.0, 1.05, 268.0, 264.0, 108.0, 110.0, 0.355, 0.50, 0.09, 0.11, 0.3, 2, 0.18, false, false,
            98.0, 1.45, 2.0, "equal_length_header", "jdm_6_speed", "jdm_gt", "sequential_port" };
    case EngineUse::roadMotorcycle:
        return { 22.5, 1350.0, 1.15, 256.0, 248.0, 106.0, 110.0, 0.27, 0.35, 0.065, 0.09, 0.3, 2, 0.42, true, true,
            95.0, 1.3, 1.6, "motorcycle_4_2_1", "motorcycle_6_speed", "motorcycle", "high_flow_port" };
    case EngineUse::sportMotorcycle:
        return { 23.5, 1300.0, 1.4, 278.0, 270.0, 104.0, 108.0, 0.30, 0.32, 0.045, 0.09, 0.8, 2, 0.42, true, true,
            98.0, 1.3, 1.6, "motorcycle_4_2_1", "motorcycle_6_speed_quickshifter", "motorcycle", "high_flow_port" };
    case EngineUse::cruiserMotorcycle:
        return { 21.0, 800.0, 0.9, 236.0, 242.0, 104.0, 108.0, 0.22, 0.55, 0.11, 0.18, -0.5, 1, 0.22, false, true,
            95.0, 1.3, 1.5, "short_v_twin", "cruiser_6_speed", "cruiser_bike", "sequential_port" };
    case EngineUse::aircraft:
        return { 15.0, 700.0, 0.85, 250.0, 256.0, 108.0, 110.0, 0.25, 0.74, 0.07, 0.14, 0.0, 2, 0.11, false, false,
            98.0, 1.45, 1.7, "aircraft_manifold", "aircraft_direct", "test_stand", "continuous_manifold" };
    }
    throw std::invalid_argument("Unknown engine use");
}

[[nodiscard]] std::string format(double value) {
    std::ostringstream stream;
    stream.precision(4);
    stream << value;
    return stream.str();
}

/** Settles values in order: an override wins over the derived value, and
 *  every value is recorded with its rule. */
class Derivation final {
public:
    Derivation(const std::map<std::string, double>& overrides, std::vector<DesignValue>& values)
        : overrides_(overrides), values_(values) {}

    double operator()(const char* key, const char* label, const char* unit, double derived, std::string rule) {
        DesignValue value { key, label, unit, derived, false, std::move(rule) };
        if (const auto found = overrides_.find(key); found != overrides_.end()) {
            value.value = found->second;
            value.overridden = true;
            used_.insert(key);
        }
        if (!std::isfinite(value.value)) throw std::invalid_argument(std::string(label) + " is not a number");
        values_.push_back(value);
        return value.value;
    }

    [[nodiscard]] std::string unknownOverride() const {
        for (const auto& [key, value] : overrides_)
            if (!used_.contains(key)) return key;
        return {};
    }

private:
    const std::map<std::string, double>& overrides_;
    std::vector<DesignValue>& values_;
    std::set<std::string> used_;
};

template <typename T>
[[nodiscard]] T part(const std::map<std::string, T>& parts, const char* key) {
    const auto found = parts.find(key);
    if (found == parts.end()) throw std::runtime_error(std::string("Missing part: ") + key);
    return found->second;
}

/** Firing intervals as "270-450", or "every 90 deg" when even. */
[[nodiscard]] std::string intervalsText(const CrankBalance& balance) {
    const auto& intervals = balance.firingIntervalsDegrees;
    if (intervals.empty()) return {};
    const auto [low, high] = std::minmax_element(intervals.begin(), intervals.end());
    if (*high - *low < 1.0e-6) return "every " + format(*low) + " deg";
    std::string text;
    for (const auto interval : intervals) text += (text.empty() ? "" : "-") + format(interval);
    return text + " deg";
}

/** Bank angle of each cylinder (id order) and its row along the crank. */
struct CylinderPlace final {
    double bankDegrees { 0.0 };
    double row { 0.0 };
    std::uint32_t bankIndex { 0 };
};

[[nodiscard]] std::vector<double> bankAngles(EngineLayout layout, std::uint32_t cylinders, std::uint32_t bankCount,
                                             double bankAngle, const std::vector<double>& given) {
    if (layout == EngineLayout::custom && !given.empty()) return given;
    switch (layout) {
    case EngineLayout::inlineLayout: return { 0.0 };
    case EngineLayout::vLayout: return { -0.5 * bankAngle, 0.5 * bankAngle };
    case EngineLayout::flat: return { -90.0, 90.0 };
    case EngineLayout::radial: {
        std::vector<double> angles;
        for (std::uint32_t index = 0; index < cylinders; ++index)
            angles.push_back(static_cast<double>(index) * 360.0 / static_cast<double>(cylinders));
        return angles;
    }
    case EngineLayout::custom: {
        std::vector<double> angles;
        const auto count = std::max<std::uint32_t>(1, bankCount);
        for (std::uint32_t index = 0; index < count; ++index)
            angles.push_back((static_cast<double>(index) - 0.5 * static_cast<double>(count - 1)) * bankAngle);
        return angles;
    }
    }
    return { 0.0 };
}

[[nodiscard]] double cylinderDisplacementLitres(double boreMm, double strokeMm) {
    return std::numbers::pi * 0.25 * boreMm * boreMm * strokeMm * 1.0e-6;
}

/** Ignition advance at fractions of the redline. */
[[nodiscard]] std::vector<IgnitionMapSample> stretchedTiming(
    std::initializer_list<std::pair<double, double>> points, double redline) {
    std::vector<IgnitionMapSample> curve;
    for (const auto& [fraction, advance] : points) curve.push_back({ fraction * redline, advance });
    return curve;
}

/** How the pipes of one designed exhaust path are joined. */
enum class PathJoin : std::uint8_t {
    /** Every bank into one pipe (a Y when there are several). */
    single,
    /** One bank's collector split into two pipes. */
    split,
    /** Two sides joined by a balance pipe. */
    hPipe,
    /** Two sides crossing in an X. */
    xPipe,
};

/** The sizes one designed exhaust path is built from. */
struct ExhaustPlan final {
    /** Primaries, collector, mid pipe, silencer and tip. */
    ExhaustConfig pipe;
    double tailpipeLengthMm { 0.0 };
    /** From the collector to the H's balance pipe or the X's centre (to the
     *  split of a split collector). */
    double crossoverPositionMm { 0.0 };
    double balancePipeLengthMm { 0.0 };
    double balancePipeDiameterMm { 0.0 };
    double crossoverCoupling { 0.0 };
};

/**
 * One designed four-stroke exhaust path: each bank's primaries into its
 * collector, the banks of a side meeting in a Y, then the mid pipe joined as
 * `join` says, a silencer, a tailpipe and a tip on each pipe. `banks` lists
 * each bank's cylinder ids. Every duct is a finite pipe the gas solver
 * resolves; the junctions are compact, so they add no trunk length the pipes
 * do not have.
 *
 * An H-pipe is a splitter on one side's pipe, the balance pipe, and a merge on
 * the other's, the balance pipe on each as a side branch (exhaustSideBranchPort):
 * it meets the pipes at right angles and takes none of their momentum, so the
 * names only orient the graph and it carries whichever side is higher. The
 * acoustic network's guides are bidirectional and its junctions scatter into
 * every port alike (journal 2026-10-29).
 */
[[nodiscard]] ExhaustNetworkConfig designedExhaustNetwork(const ExhaustPlan& plan, PathJoin join,
    const std::vector<std::vector<std::uint32_t>>& banks, const AcousticPoint3M& singleOutletPosition,
    std::size_t facesPerPort = 1) {
    ExhaustNetworkConfig network;
    const auto& pipe = plan.pipe;
    const auto primaryLength = std::max(20.0, pipe.primaryLengthMm);
    const auto primaryDiameter = std::max(10.0, pipe.primaryDiameterMm);
    const auto collectorDiameter = std::max(primaryDiameter, pipe.collectorDiameterMm);
    const auto midDiameter = pipe.midpipeDiameterMm > 0.0 ? pipe.midpipeDiameterMm : collectorDiameter;
    const auto midLength = std::max(0.0, pipe.midpipeLengthMm);
    const auto add = [&network](std::uint32_t id, ExhaustComponentType type, double lengthMm, double diameterMm) {
        ExhaustComponentConfig component;
        component.id = id;
        component.type = type;
        component.lengthMm = lengthMm;
        component.diameterMm = diameterMm;
        network.components.push_back(component);
    };
    const auto connect = [&network](std::uint32_t from, std::uint32_t to,
                                    std::uint8_t fromPort = unspecifiedExhaustComponentPort,
                                    std::uint8_t toPort = unspecifiedExhaustComponentPort) {
        network.connections.push_back({ from, to, fromPort, toPort });
    };
    // A pipe of `lengthMm` after `from`, or `from` itself when it has none.
    const auto chain = [&](std::uint32_t from, std::uint32_t id, double lengthMm, double diameterMm) {
        if (!(lengthMm >= 1.0)) return from;
        add(id, ExhaustComponentType::pipe, lengthMm, diameterMm);
        connect(from, id);
        return id;
    };

    // Each bank's primaries into its collector, as the catalogue's.
    std::vector<std::uint32_t> bankEnds;
    auto primaryId = std::uint32_t { 1000 };
    // Above the silencers (600s), tails (650s) and outlets (700s).
    auto passageId = std::uint32_t { 800 };
    for (std::size_t bank = 0; bank < banks.size(); ++bank) {
        std::vector<std::uint32_t> primaries;
        for (std::size_t first = 0; first < banks[bank].size(); first += facesPerPort) {
            add(primaryId, ExhaustComponentType::pipe, primaryLength, primaryDiameter);
            if (facesPerPort == 1) {
                network.cylinderConnections.push_back({ banks[bank][first], primaryId });
            } else {
                // A rotor's faces open in turn onto one port: its passage, a
                // merge of the primary's bore 2.5 bores long, as the
                // catalogue's rotary (ExhaustGraph).
                add(passageId, ExhaustComponentType::merge, 0.0, primaryDiameter);
                network.components.back().volumeLitres = std::numbers::pi * 0.25 * primaryDiameter
                    * primaryDiameter * 2.5 * primaryDiameter * 1.0e-6;
                for (auto face = first; face < std::min(banks[bank].size(), first + facesPerPort); ++face)
                    network.cylinderConnections.push_back({ banks[bank][face], passageId });
                connect(passageId++, primaryId);
            }
            primaries.push_back(primaryId++);
        }
        if (primaries.size() == 1) {
            bankEnds.push_back(primaries.front());
            continue;
        }
        const auto mergeId = static_cast<std::uint32_t>(200 + bank);
        add(mergeId, ExhaustComponentType::merge, std::max(120.0, primaryLength * 0.35), collectorDiameter);
        network.components.back().volumeLitres = pipe.collectorVolumeLitres > 0.0
            ? std::max(0.05, pipe.collectorVolumeLitres) : 0.0;
        for (const auto primary : primaries) connect(primary, mergeId);
        bankEnds.push_back(mergeId);
    }

    // The banks of one side meet in a Y: a leg from each collector into a
    // compact merge. Returns the Y (or the lone collector) and the length of
    // mid pipe the legs took.
    const auto legLength = std::clamp(0.2 * midLength, 150.0, 400.0);
    const auto meet = [&](std::size_t first, std::size_t last, std::uint32_t side) {
        if (last - first == 1) return std::pair { bankEnds[first], 0.0 };
        const auto yId = 300 + side;
        add(yId, ExhaustComponentType::merge, 0.0, midDiameter);
        for (auto bank = first; bank < last; ++bank) {
            const auto legId = static_cast<std::uint32_t>(250 + bank);
            add(legId, ExhaustComponentType::pipe, legLength, collectorDiameter);
            connect(bankEnds[bank], legId);
            connect(legId, yId);
        }
        return std::pair { yId, legLength };
    };

    // The pipes ending at the silencers, and their bores.
    std::vector<std::pair<std::uint32_t, double>> ends;
    const auto position = std::clamp(plan.crossoverPositionMm, 0.0, midLength);
    if (join == PathJoin::single) {
        const auto [start, used] = meet(0, bankEnds.size(), 0);
        ends.push_back({ chain(start, 500, midLength - used, midDiameter), midDiameter });
    } else if (join == PathJoin::split) {
        const auto [start, used] = meet(0, bankEnds.size(), 0);
        const auto front = chain(start, 400, position - used, midDiameter);
        add(450, ExhaustComponentType::splitter, 0.0, midDiameter);
        connect(front, 450);
        // Each branch carries half the flow; real twin pipes off one front
        // pipe run about 0.85 of its bore (2.5 in into 2 x 2.25 in).
        const auto branch = 0.85 * midDiameter;
        const auto rear = std::max(100.0, midLength - position);
        ends.push_back({ chain(450, 500, rear, branch), branch });
        ends.push_back({ chain(450, 501, rear, branch), branch });
    } else {
        const auto half = bankEnds.size() / 2;
        const auto [left, leftUsed] = meet(0, half, 0);
        const auto [right, rightUsed] = meet(half, bankEnds.size(), 1);
        // Every port of a crossover meets a finite duct, and the H's
        // junctions are kept off the collectors' alike.
        const auto leftFront = chain(left, 400, std::max(100.0, position - leftUsed), midDiameter);
        const auto rightFront = chain(right, 401, std::max(100.0, position - rightUsed), midDiameter);
        const auto rear = std::max(100.0, midLength - position);
        if (join == PathJoin::hPipe) {
            add(450, ExhaustComponentType::splitter, 0.0, midDiameter);
            add(451, ExhaustComponentType::merge, 0.0, midDiameter);
            add(460, ExhaustComponentType::pipe, std::max(50.0, plan.balancePipeLengthMm),
                plan.balancePipeDiameterMm > 0.0 ? plan.balancePipeDiameterMm : midDiameter);
            // The balance pipe meets each pipe at right angles.
            connect(leftFront, 450);
            connect(450, 460, exhaustSideBranchPort);
            connect(460, 451, unspecifiedExhaustComponentPort, exhaustSideBranchPort);
            connect(rightFront, 451);
            ends.push_back({ chain(450, 500, rear, midDiameter), midDiameter });
            ends.push_back({ chain(451, 501, rear, midDiameter), midDiameter });
        } else {
            add(455, ExhaustComponentType::crossover, 0.0, midDiameter);
            network.components.back().restriction = 0.06;
            network.components.back().crossoverCoupling = std::clamp(plan.crossoverCoupling, 0.0, 1.0);
            connect(leftFront, 455, unspecifiedExhaustComponentPort, 0);
            connect(rightFront, 455, unspecifiedExhaustComponentPort, 1);
            add(500, ExhaustComponentType::pipe, rear, midDiameter);
            add(501, ExhaustComponentType::pipe, rear, midDiameter);
            connect(455, 500, 0);
            connect(455, 501, 1);
            ends.push_back({ 500, midDiameter });
            ends.push_back({ 501, midDiameter });
        }
    }

    // A silencer, a tailpipe and a tip on each pipe.
    const auto silenced = pipe.mufflerChamberDiameterMm > 1.0 && pipe.mufflerChamberLengthMm > 1.0;
    for (std::size_t index = 0; index < ends.size(); ++index) {
        const auto side = static_cast<std::uint32_t>(index);
        auto [end, bore] = ends[index];
        if (silenced) {
            const auto mufflerId = 600 + side;
            add(mufflerId, ExhaustComponentType::muffler, pipe.mufflerChamberLengthMm, bore);
            auto& muffler = network.components.back();
            const auto radiusM = pipe.mufflerChamberDiameterMm * 0.0005;
            muffler.volumeLitres = std::numbers::pi * radiusM * radiusM * pipe.mufflerChamberLengthMm;
            muffler.restriction = pipe.mufflerRestriction;
            muffler.packingFlowResistivityPaSPerM2 = pipe.mufflerPackingFlowResistivityPaSPerM2;
            muffler.packingThicknessMm = pipe.mufflerPackingThicknessMm;
            muffler.perforatedOpenAreaRatio = pipe.mufflerPerforatedOpenAreaRatio;
            connect(end, mufflerId);
            end = mufflerId;
        }
        end = chain(end, 650 + side, plan.tailpipeLengthMm, bore);
        const auto outletId = 700 + side;
        add(outletId, ExhaustComponentType::outlet, 0.0, std::max(10.0, pipe.outletDiameterMm));
        auto& outlet = network.components.back();
        outlet.dischargeCoefficient = pipe.outletDischargeCoefficient;
        outlet.acousticPositionM = singleOutletPosition;
        if (ends.size() > 1)
            outlet.acousticPositionM.x = 0.9 * (static_cast<double>(index) / static_cast<double>(ends.size() - 1) - 0.5);
        connect(end, outletId);
    }
    return network;
}

/** Top speed where the wheel power meets drag and rolling resistance. */
[[nodiscard]] double topSpeedMps(double wheelPowerW, const VehicleConfig& vehicle) {
    const auto demand = [&vehicle](double speed) {
        return 0.5 * 1.204 * vehicle.dragCoefficient * vehicle.frontalAreaM2 * speed * speed * speed
            + vehicle.rollingResistanceCoefficient * vehicle.massKg * 9.81 * speed;
    };
    double low = 1.0;
    double high = 200.0;
    for (int iteration = 0; iteration < 80; ++iteration) {
        const auto middle = 0.5 * (low + high);
        (demand(middle) < wheelPowerW ? low : high) = middle;
    }
    return 0.5 * (low + high);
}

} // namespace

const char* engineUseName(EngineUse use) noexcept {
    switch (use) {
    case EngineUse::roadCar: return "road_car";
    case EngineUse::sportCar: return "sport_car";
    case EngineUse::roadMotorcycle: return "road_motorcycle";
    case EngineUse::sportMotorcycle: return "sport_motorcycle";
    case EngineUse::cruiserMotorcycle: return "cruiser_motorcycle";
    case EngineUse::aircraft: return "aircraft";
    }
    return "road_car";
}

DesignedEngine designCatalogueEngine(const EngineSpec& spec, const std::filesystem::path& catalogRoot) {
    DesignedEngine designed;
    try {
        designed = designEngine(spec, loadPartsLibrary(catalogRoot));
    } catch (const std::exception& error) {
        designed.error = std::string("The parts library did not load: ") + error.what();
        return designed;
    }
    if (!designed.config) return designed;
    const auto voicing = loadAudioVoicing(catalogRoot, "", "");
    if (!voicing) {
        designed.error = "The default voicing did not load: " + voicing.error;
        designed.config.reset();
        return designed;
    }
    designed.config->audioVoicing = voicing.voicing;
    return designed;
}

std::optional<EngineUse> parseEngineUse(std::string_view name) noexcept {
    for (const auto use : { EngineUse::roadCar, EngineUse::sportCar, EngineUse::roadMotorcycle,
             EngineUse::sportMotorcycle, EngineUse::cruiserMotorcycle, EngineUse::aircraft })
        if (name == engineUseName(use)) return use;
    return std::nullopt;
}

const char* crankPinsName(CrankPins pins) noexcept {
    switch (pins) {
    case CrankPins::automatic: return "automatic";
    case CrankPins::shared: return "shared";
    case CrankPins::separate: return "separate";
    }
    return "automatic";
}

std::optional<CrankPins> parseCrankPins(std::string_view name) noexcept {
    for (const auto pins : { CrankPins::automatic, CrankPins::shared, CrankPins::separate })
        if (name == crankPinsName(pins)) return pins;
    return std::nullopt;
}

const char* exhaustSystemName(ExhaustSystem system) noexcept {
    switch (system) {
    case ExhaustSystem::automatic: return "automatic";
    case ExhaustSystem::single: return "single";
    case ExhaustSystem::dual: return "dual";
    case ExhaustSystem::dualHPipe: return "h_pipe";
    case ExhaustSystem::dualXPipe: return "x_pipe";
    }
    return "automatic";
}

std::optional<ExhaustSystem> parseExhaustSystem(std::string_view name) noexcept {
    for (const auto system : { ExhaustSystem::automatic, ExhaustSystem::single, ExhaustSystem::dual,
             ExhaustSystem::dualHPipe, ExhaustSystem::dualXPipe })
        if (name == exhaustSystemName(system)) return system;
    return std::nullopt;
}

const char* exhaustSilencerName(ExhaustSilencer silencer) noexcept {
    switch (silencer) {
    case ExhaustSilencer::automatic: return "automatic";
    case ExhaustSilencer::packedBox: return "packed_box";
    case ExhaustSilencer::straightThrough: return "straight_through";
    case ExhaustSilencer::none: return "none";
    }
    return "automatic";
}

std::optional<ExhaustSilencer> parseExhaustSilencer(std::string_view name) noexcept {
    for (const auto silencer : { ExhaustSilencer::automatic, ExhaustSilencer::packedBox,
             ExhaustSilencer::straightThrough, ExhaustSilencer::none })
        if (name == exhaustSilencerName(silencer)) return silencer;
    return std::nullopt;
}

const char* fuelSystemName(FuelSystem system) noexcept {
    switch (system) {
    case FuelSystem::automatic: return "automatic";
    case FuelSystem::portInjection: return "port_injection";
    case FuelSystem::directInjection: return "direct_injection";
    case FuelSystem::carburettor: return "carburettor";
    }
    return "automatic";
}

std::optional<FuelSystem> parseFuelSystem(std::string_view name) noexcept {
    for (const auto system : { FuelSystem::automatic, FuelSystem::portInjection, FuelSystem::directInjection,
             FuelSystem::carburettor })
        if (name == fuelSystemName(system)) return system;
    return std::nullopt;
}

const char* intakeStyleName(IntakeStyle style) noexcept {
    switch (style) {
    case IntakeStyle::automatic: return "automatic";
    case IntakeStyle::airbox: return "airbox";
    case IntakeStyle::podFilter: return "pod_filter";
    case IntakeStyle::velocityStacks: return "velocity_stacks";
    }
    return "automatic";
}

std::optional<IntakeStyle> parseIntakeStyle(std::string_view name) noexcept {
    for (const auto style : { IntakeStyle::automatic, IntakeStyle::airbox, IntakeStyle::podFilter,
             IntakeStyle::velocityStacks })
        if (name == intakeStyleName(style)) return style;
    return std::nullopt;
}

DesignedEngine designEngine(const EngineSpec& spec, const PartsLibrary& parts) {
    DesignedEngine designed;
    if (spec.name.empty()) {
        designed.error = "The engine needs a name";
        return designed;
    }
    if (spec.cylinderCount < 1 || spec.cylinderCount > 32) {
        designed.error = "An engine has 1 to 32 cylinders";
        return designed;
    }
    // A rotary's count is its rotors, each of three faces the simulator runs
    // as cylinders, 32 at most.
    const auto rotary = spec.cycle == EngineCycle::rotary;
    if (rotary && spec.cylinderCount > 10) {
        designed.error = "A rotary has 1 to 10 rotors";
        return designed;
    }
    try {
        const auto profile = profileFor(spec.use);
        const auto twoStroke = spec.cycle == EngineCycle::twoStroke;
        const auto diesel = spec.fuel == FuelType::diesel;
        const auto boosted = spec.induction != Induction::natural;
        // The fuel system each kind of engine has in the world, unless one is
        // chosen: a diesel injects into the cylinder, a two-stroke breathes
        // through a carburettor, a four-stroke injects into its ports.
        const auto fuelSystem = spec.fuelSystem != FuelSystem::automatic ? spec.fuelSystem
            : diesel ? FuelSystem::directInjection
            : twoStroke ? FuelSystem::carburettor : FuelSystem::portInjection;
        if (diesel && fuelSystem != FuelSystem::directInjection)
            throw std::invalid_argument("A diesel's fuel is injected into the cylinder: choose direct injection");
        const auto carburetted = fuelSystem == FuelSystem::carburettor;
        if (rotary) {
            if (diesel)
                throw std::invalid_argument("A rotary burns petrol: its long chamber cannot hold a diesel's "
                                            "compression");
            if (spec.layout != EngineLayout::inlineLayout)
                throw std::invalid_argument("A rotary's rotors stand one behind another on the shaft: choose inline");
            if (!spec.firingOrder.empty() || !spec.crankOffsetsDegrees.empty())
                throw std::invalid_argument("A rotary fires as its rotors are spaced: evenly round the eccentric "
                                            "shaft");
            if (spec.boreMm > 0.0 || spec.strokeMm > 0.0)
                throw std::invalid_argument("A rotary is sized by its displacement or its rotor (generating "
                                            "radius, eccentricity, width), not a bore and a stroke");
        }
        const auto rotors = spec.cylinderCount;
        const auto count = rotary ? 3U * rotors : spec.cylinderCount;
        const auto countD = static_cast<double>(count);
        Derivation derive(spec.overrides, designed.values);
        EngineConfig config;
        config.name = spec.name;
        config.cycle = spec.cycle;
        config.fuel = spec.fuel;
        config.layout = spec.layout;

        // Size. Bore and stroke as given, or from the displacement and the
        // use's bore/stroke ratio.
        // A rotary's rotor instead: from its displacement (Mazda's, one face
        // per rotor: a 13B's 1.3 L) at the 13B's proportions, R / e 7 and
        // b / R 0.76 (105, 15 and 80 mm), each face sweeping 3 sqrt(3) R e b.
        RotaryConfig rotor;
        if (rotary) {
            constexpr double eccentricityShare = 15.0 / 105.0;
            constexpr double widthShare = 80.0 / 105.0;
            const auto perRotorMm3 = spec.displacementLitres / static_cast<double>(rotors) * 1.0e6;
            rotor.generatingRadiusMm = derive("generating_radius_mm", "Rotor generating radius", "mm",
                std::cbrt(std::max(0.0, perRotorMm3) / (3.0 * std::sqrt(3.0) * eccentricityShare * widthShare)),
                "from the displacement, at the 13B's proportions (R 105, e 15, b 80 mm)");
            rotor.eccentricityMm = derive("eccentricity_mm", "Eccentricity", "mm",
                eccentricityShare * rotor.generatingRadiusMm, "R / 7, the 13B's");
            rotor.rotorWidthMm = derive("rotor_width_mm", "Rotor width", "mm", widthShare * rotor.generatingRadiusMm,
                "0.76 x R, the 13B's");
            rotor.rotorCount = rotors;
            if (!(rotor.generatingRadiusMm > 0.0 && rotor.eccentricityMm > 0.0 && rotor.rotorWidthMm > 0.0))
                throw std::invalid_argument("Give a rotary's displacement, or its rotor");
            if (rotor.generatingRadiusMm < 4.0 * rotor.eccentricityMm)
                throw std::invalid_argument("A rotor's generating radius is more than 4 times its eccentricity, "
                                            "or the apexes cut the housing (Mazda's are near 7)");
        }
        const auto ratio = rotary ? 1.0 : derive("bore_stroke_ratio", "Bore / stroke", "",
            diesel ? 0.85 : twoStroke ? 1.18 : profile.boreStrokeRatio,
            "the use's (cars ~1.0, sport bikes 1.4, cruisers 0.9, diesels 0.85)");
        auto boreFromVolume = 0.0;
        if (spec.displacementLitres > 0.0)
            boreFromVolume = std::cbrt(spec.displacementLitres / countD * 1.0e6 * 4.0 * ratio / std::numbers::pi);
        // A face's bore and stroke are the equivalents its flames and its
        // sweep run with (rotaryEquivalentBoreMm).
        const auto bore = rotary ? rotaryEquivalentBoreMm(rotor)
            : derive("bore_mm", "Bore", "mm", spec.boreMm > 0.0 ? spec.boreMm : boreFromVolume,
                spec.boreMm > 0.0 ? "given" : "from the displacement and bore / stroke");
        const auto stroke = rotary ? rotaryEquivalentStrokeMm(rotor)
            : derive("stroke_mm", "Stroke", "mm",
                spec.strokeMm > 0.0 ? spec.strokeMm : bore / std::max(0.1, ratio),
                spec.strokeMm > 0.0 ? "given" : "bore / (bore / stroke)");
        // No size is refused for being large or small: a model engine and a
        // ship's are both engines. The bounds only keep the numbers physical.
        if (!(bore >= minimumBoreStrokeMm && bore <= maximumBoreStrokeMm
                && stroke >= minimumBoreStrokeMm && stroke <= maximumBoreStrokeMm))
            throw std::invalid_argument("Give a bore and a stroke of 5-1,000 mm, or a displacement");
        // The sizing rules below take a rotary's faces' sweep in 720 deg of
        // the shaft (breathingDisplacementLitres): a 13B's 2.6 L.
        const auto litres = cylinderDisplacementLitres(bore, stroke) * countD * (rotary ? 720.0 / 1'080.0 : 1.0);
        // Below 10 cc the carburettor and the simulator's parts no longer
        // scale: a 2.5 cc single stays rich (lambda 0.33) and never fires.
        if (litres < 0.01 - 1.0e-9) throw std::invalid_argument("The simulator runs engines of 10 cc at least");

        // Speeds: the redline from the mean piston speed the use reaches.
        const auto pistonSpeed = diesel ? std::min(16.0, profile.redlinePistonSpeedMps)
            : twoStroke ? 17.5 : profile.redlinePistonSpeedMps - (boosted ? 1.5 : 0.0);
        auto redlineDerived = pistonSpeed * 60.0 / (2.0 * stroke * 1.0e-3);
        if (diesel) redlineDerived = std::min(redlineDerived, 5'000.0);
        // A rotary's is its apexes' speed: the 13B's 9,000 rpm (the RX-8's
        // high-power one) or 7,500 (its standard one) at R 105 mm, 1,000 less
        // boosted (the 13B-REW's 8,000).
        const auto sportUse = spec.use == EngineUse::sportCar || spec.use == EngineUse::sportMotorcycle;
        if (rotary)
            redlineDerived = ((sportUse ? 9'000.0 : 7'500.0) - (boosted ? 1'000.0 : 0.0)) * 105.0
                / rotor.generatingRadiusMm;
        redlineDerived = std::min(redlineDerived, 16'000.0);
        config.redlineRpm = derive("redline_rpm", "Redline", "rpm", std::round(redlineDerived / 100.0) * 100.0,
            rotary ? std::string("the 13B's apex speed: ") + (sportUse ? "9,000" : "7,500")
                    + " rpm at R 105 mm" + (boosted ? ", 1,000 less boosted" : "")
                : "mean piston speed " + format(pistonSpeed) + " m/s at the redline");
        const auto peakPowerRpm = derive("peak_power_rpm", "Peak power speed", "rpm",
            (diesel ? 0.8 : 0.9) * config.redlineRpm, diesel ? "0.8 x redline" : "0.9 x redline");
        config.idleRpm = derive("idle_rpm", "Idle", "rpm", twoStroke ? 1'300.0 : rotary ? 800.0 : profile.idleRpm,
            rotary ? "the 13B's" : "the use's");
        config.ignition.revLimitRpm = derive("rev_limit_rpm", "Rev limiter", "rpm",
            diesel ? config.redlineRpm : config.redlineRpm * 1.03, diesel ? "the redline" : "3 % over the redline");

        // Boost first: the compression ratio follows it.
        const auto boost = !boosted ? 1.0
            : derive("boost_pressure_ratio", "Boost pressure ratio", "",
                spec.boostPressureRatio > 0.0 ? spec.boostPressureRatio : diesel ? 2.3 : profile.boostPressureRatio,
                spec.boostPressureRatio > 0.0 ? "given" : "the use's");
        auto compressionDerived = diesel ? 16.5
            : spec.use == EngineUse::aircraft ? 7.0
            : std::clamp(14.0 - 0.033 * bore + profile.compressionBonus, 8.5, 13.5) - 2.0 * (boost - 1.0);
        if (twoStroke && !diesel) compressionDerived = 11.0;
        if (rotary) compressionDerived = 10.0 - 1.25 * (boost - 1.0);
        const auto compression = derive("compression_ratio", "Compression ratio", "",
            std::max(diesel ? 14.0 : 7.0, compressionDerived),
            diesel ? "a direct-injection diesel's" : spec.use == EngineUse::aircraft ? "an aircraft engine on avgas"
            : rotary ? "the Renesis' 10, less 1.25 per bar of boost (the 13B-REW's 9.0)"
            : "14 - 0.033 x bore + the use's, less 2 per bar of boost");

        // Moving parts.
        // A rotor reciprocates nothing: a face has no rod and no piston to
        // list (applyRotaryFaceGeometry keeps the unread fields valid).
        const auto rod = rotary ? stroke : derive("connecting_rod_mm", "Connecting rod", "mm",
            twoStroke ? std::max(2.0 * stroke, 1.7 * bore) : std::max(1.6 * stroke, 1.25 * bore),
            twoStroke ? "max(2.0 x stroke, 1.7 x bore)" : "max(1.6 x stroke, 1.25 x bore)");
        const auto density = diesel ? 0.96 : twoStroke ? 0.88
            : profile.pistonDensityGPerCm3 * (boosted ? 1.25 : 1.0);
        const auto pistonByBore = density * std::pow(bore / 10.0, 3.0);
        const auto piston = rotary ? minimumPistonMassGrams : derive("piston_mass_g", "Piston mass", "g",
            std::max(pistonByBore, minimumPistonMassGrams), pistonByBore < minimumPistonMassGrams
                ? format(density) + " g/cm3 x bore^3 gives " + format(pistonByBore) + ", raised to the simulator's floor"
                : format(density) + " g/cm3 x bore^3");
        const auto rodMass = rotary ? 1.3 * piston : derive("connecting_rod_mass_g", "Connecting rod mass", "g",
            1.3 * piston, "1.3 x the piston");
        // A two-stroke has no valves, no headers and no injector: its ports,
        // its chambers and its carburettor carry them, so those rows are not
        // listed for it and setting one is an unknown value.
        const auto valvesPerSide = twoStroke || rotary ? 2U : static_cast<std::uint32_t>(std::lround(derive("valves_per_side",
            "Valves per side", "", diesel ? 2.0 : static_cast<double>(profile.valvesPerSide), "the use's")));
        // Below 0.22 L the per-litre rule falls under the simulator's 0.01
        // floor (a 125 cc sport single: 0.0056): it is raised to the floor.
        // A diesel's 0.135 is a car's on 0.5 L cylinders; a geometrically
        // similar crank and flywheel go as the bore to the fifth, so per litre
        // as the cylinder to the two-thirds above that. A 193 L marine six
        // on 26 kg m2 gained 1,700 rpm inside the brake's 0.13 s lag, hit its
        // limiter and lost its boost (journal 2026-10-30).
        const auto similarCrank = diesel ? std::max(1.0, std::pow(litres / countD / 0.5, 2.0 / 3.0)) : 1.0;
        const auto inertiaPerLitre = (diesel ? 0.135 * similarCrank : twoStroke ? 0.087 : rotary ? 0.042
            : profile.inertiaKgM2PerLitre) * litres;
        config.rotatingInertiaKgM2 = derive("rotating_inertia_kg_m2", "Rotating inertia", "kg m2",
            std::max(minimumRotatingInertiaKgM2, inertiaPerLitre),
            inertiaPerLitre < minimumRotatingInertiaKgM2
                ? "per litre gives " + format(inertiaPerLitre) + ", raised to the simulator's floor"
                : rotary ? std::string("the catalogue 13B's, per litre swept in 720 deg (0.11 on 2.6 L)")
                : diesel && similarCrank > 1.0 ? std::string("a car diesel's 0.135 per litre, x (cylinder / 0.5 L)^(2/3): a similar crank")
                : std::string("per litre, the use's (cars 0.09-0.10, sport bikes 0.045, cruisers 0.11)"));
        config.frictionCoefficient = derive("friction_coefficient", "Friction coefficient", "",
            diesel ? 0.16 : twoStroke ? 0.07 : rotary ? 0.11 : profile.frictionCoefficient,
            rotary ? "the catalogue 13B's: apex and side seals" : "the use's");
        config.octaneRating = derive("octane_rating", "Octane", "RON", profile.octane, "the use's");

        // Crank: banks, rows, firing order.
        // A W with no angles given spreads its banks over 120 deg (a broad-arrow
        // three at 60 deg); without this every bank stood at 0 deg, one on
        // another (journal 2026-10-19).
        const auto evenW = spec.layout == EngineLayout::custom && spec.bankAnglesDegrees.empty();
        const auto wSpacing = std::min(90.0, 120.0 / static_cast<double>(spec.bankCount > 1 ? spec.bankCount - 1 : 1));
        const auto banks = bankAngles(spec.layout,
            count, spec.bankCount, spec.layout == EngineLayout::vLayout
                ? derive("bank_angle_deg", "Bank angle", "deg", spec.bankAngleDegrees > 0.0 ? spec.bankAngleDegrees
                    : 90.0, spec.bankAngleDegrees > 0.0 ? "given" : "90 deg")
                : evenW ? derive("bank_angle_deg", "Angle between banks", "deg",
                    spec.bankAngleDegrees > 0.0 ? spec.bankAngleDegrees : wSpacing,
                    spec.bankAngleDegrees > 0.0 ? "given" : "120 deg shared between the banks, at most 90")
                : spec.bankAngleDegrees, spec.bankAnglesDegrees);
        for (std::size_t a = 0; a < banks.size(); ++a)
            for (std::size_t b = a + 1; b < banks.size(); ++b)
                if (spec.layout != EngineLayout::radial && std::abs(banks[a] - banks[b]) < 1.0)
                    throw std::invalid_argument("Two banks stand at the same angle");
        config.bankAngleDegrees = spec.layout == EngineLayout::vLayout || (evenW && banks.size() > 1) ? banks[1] - banks[0]
            : spec.layout == EngineLayout::flat ? 180.0 : spec.bankAngleDegrees;
        const auto radial = spec.layout == EngineLayout::radial;
        const auto bankCount = static_cast<std::uint32_t>(banks.size());
        if (spec.layout == EngineLayout::custom && bankCount < 2)
            throw std::invalid_argument("A custom (W) layout needs 2 or more banks and their angle");
        if (!radial && count % bankCount != 0)
            throw std::invalid_argument("The cylinders do not share out evenly over the banks");
        std::vector<CylinderPlace> places(count);
        std::vector<BalanceCylinder> geometry(count);
        for (std::uint32_t index = 0; index < count; ++index) {
            places[index].bankIndex = radial ? index : index % bankCount;
            places[index].bankDegrees = banks[places[index].bankIndex];
            places[index].row = radial ? 0.0 : std::floor(static_cast<double>(index) / static_cast<double>(bankCount));
            geometry[index].bankDegrees = places[index].bankDegrees;
            geometry[index].axialPitches = places[index].row;
            geometry[index].rodRatio = 0.5 * stroke / rod;
        }
        const auto cycleDegrees = cycleDegreesOf(spec.cycle);
        std::vector<double> tdc(count, 0.0);
        std::vector<std::uint32_t> order = spec.firingOrder;
        const auto rowed = (spec.layout == EngineLayout::vLayout || spec.layout == EngineLayout::custom)
            && spec.crankPins != CrankPins::separate;
        std::optional<RowPinCrank> crank;
        std::string pinRule = radial ? "one throw, a master rod and articulated rods"
            : !spec.crankOffsetsDegrees.empty() ? "the cylinders whose offsets put them on one pin share it"
            : spec.layout == EngineLayout::vLayout || spec.layout == EngineLayout::custom
            ? "one per cylinder: as asked" : "one per cylinder";
        if (rotary) {
            // A rotor's faces fire 360 deg of the shaft apart, and the rotors
            // are spaced evenly round it: a 13B's two at 180 deg, a 20B's
            // three at 120, so a face fires every 360 / rotors.
            for (std::uint32_t index = 0; index < count; ++index)
                tdc[index] = static_cast<double>(index / 3) * 360.0 / static_cast<double>(rotors)
                    + static_cast<double>(index % 3) * 360.0;
            order.resize(count);
            std::iota(order.begin(), order.end(), 1U);
            std::stable_sort(order.begin(), order.end(), [&tdc](auto a, auto b) { return tdc[a - 1] < tdc[b - 1]; });
            pinRule = "a face fires every " + format(360.0 / static_cast<double>(rotors))
                + " deg of the eccentric shaft";
        } else if (!spec.crankOffsetsDegrees.empty()) {
            if (spec.crankOffsetsDegrees.size() != count)
                throw std::invalid_argument("Give one crank offset per cylinder");
            tdc = spec.crankOffsetsDegrees;
            order.resize(count);
            std::iota(order.begin(), order.end(), 1U);
            std::stable_sort(order.begin(), order.end(), [&tdc, cycleDegrees](auto a, auto b) {
                return std::fmod(tdc[a - 1], cycleDegrees) < std::fmod(tdc[b - 1], cycleDegrees);
            });
        } else {
            if (radial) {
                // One crank throw: a cylinder's top dead centre is where the
                // pin points at it, the even ids one turn later on a
                // four-stroke. An odd count fires evenly, an even one cannot.
                if (!order.empty())
                    throw std::invalid_argument("A radial's firing order follows from its cylinders' places");
                for (std::uint32_t index = 0; index < count; ++index)
                    tdc[index] = places[index].bankDegrees + ((index + 1) % 2 == 0 ? cycleDegrees - 360.0 : 0.0);
                order.resize(count);
                std::iota(order.begin(), order.end(), 1U);
                std::stable_sort(order.begin(), order.end(),
                    [&tdc](auto a, auto b) { return tdc[a - 1] < tdc[b - 1]; });
            } else {
                // A V's or a W's cylinders of a row share a pin, as on the
                // real engines (the owner's report, 2026-10-19): each had its
                // own, placed for even firing.
                if (rowed) {
                    const auto shared = rowPinCrank(geometry, cycleDegrees, 0.0, order, false);
                    if (shared && (shared->unevennessDegrees < 1.0e-6 || spec.crankPins == CrankPins::shared
                            || count <= 4)) {
                        crank = shared;
                        pinRule = "one per row, shared by the banks";
                    } else if (spec.crankPins == CrankPins::automatic) {
                        crank = splitPinCrank(geometry, cycleDegrees, order);
                        if (crank)
                            pinRule = "split by " + format(std::abs(crank->splitDegrees)) + " deg in each row: "
                                + (shared ? "one shared pin per row fires " + intervalsText(shared->balance)
                                          : std::string(order.empty() ? "no shared pin fires evenly"
                                                                      : "no shared pin gives this firing order"));
                    }
                    if (!crank)
                        pinRule = std::string("one per cylinder: ") + (spec.crankPins == CrankPins::separate
                            ? "as asked" : !order.empty() ? "no shared or split pin gives this firing order"
                            : "no shared or split pin fires evenly");
                }
                if (crank) {
                    tdc = crank->tdcDegrees;
                    order = crank->order;
                }
                if (order.empty()) order = evenFiringOrders(geometry, cycleDegrees, 1).front().order;
                auto sorted = order;
                std::sort(sorted.begin(), sorted.end());
                for (std::size_t index = 0; index < sorted.size(); ++index)
                    if (sorted[index] != index + 1)
                        throw std::invalid_argument("The firing order must name every cylinder once");
                if (order.size() != count)
                    throw std::invalid_argument("The firing order must name every cylinder once");
                if (!crank)
                    for (std::size_t position = 0; position < order.size(); ++position)
                        tdc[order[position] - 1] = static_cast<double>(position) * cycleDegrees / countD;
            }
        }
        config.firingOrder = order;

        // Head and cams.
        const auto intakeValveDiameter = bore * (valvesPerSide > 1 ? 0.40 : 0.50);
        CamshaftConfig cams;
        // A rotary's ports as valves of its face's 1,080 deg cycle: a
        // side-port Renesis-class face's (the catalogue 13B's estimate, no
        // overlap), the rotor's flank sweeping each open over a quarter of
        // its duration. Their sizes go with the rotor.
        const auto portScale = rotary ? std::cbrt(rotaryFaceDisplacementLitres(rotor) / 0.6547) : 1.0;
        if (rotary) {
            cams.intakeDurationDegrees = derive("intake_duration_deg", "Intake port open", "deg", 332.0,
                "a side port's, the catalogue 13B's");
            cams.exhaustDurationDegrees = derive("exhaust_duration_deg", "Exhaust port open", "deg", 317.0,
                "a side port's, the catalogue 13B's");
            cams.intakeCenterlineDegrees = derive("intake_centerline_deg", "Intake port centre", "deg", 169.0,
                "after the gas-exchange top dead centre, the catalogue 13B's");
            cams.exhaustCenterlineDegrees = derive("exhaust_centerline_deg", "Exhaust port centre", "deg", 161.5,
                "before the gas-exchange top dead centre, the catalogue 13B's");
            cams.intakeLiftMm = derive("intake_lift_mm", "Intake port height", "mm", 8.0 * portScale,
                "the 13B's 8 mm, with the rotor's size");
            cams.exhaustLiftMm = derive("exhaust_lift_mm", "Exhaust port height", "mm", 7.5 * portScale,
                "the 13B's 7.5 mm, with the rotor's size");
            const auto trapezoid = [](double duration, double lift) {
                return std::vector<ValveLiftSample> { { -0.5 * duration, 0.0 }, { -0.25 * duration, lift },
                    { 0.25 * duration, lift }, { 0.5 * duration, 0.0 } };
            };
            cams.intakeLiftProfile = trapezoid(cams.intakeDurationDegrees, cams.intakeLiftMm);
            cams.exhaustLiftProfile = trapezoid(cams.exhaustDurationDegrees, cams.exhaustLiftMm);
            const auto lift = [](double fraction, double height) { return fraction * height; };
            cams.intakeFlowCurve = { { 0.0, 0.0 }, { lift(0.25, cams.intakeLiftMm), 0.50 },
                { lift(0.5, cams.intakeLiftMm), 0.68 }, { cams.intakeLiftMm, 0.75 } };
            cams.exhaustFlowCurve = { { 0.0, 0.0 }, { lift(0.267, cams.exhaustLiftMm), 0.48 },
                { lift(0.533, cams.exhaustLiftMm), 0.66 }, { cams.exhaustLiftMm, 0.72 } };
        } else if (!twoStroke) {
            const auto dieselCams = diesel;
            cams.intakeDurationDegrees = derive("intake_duration_deg", "Intake duration", "deg",
                dieselCams ? 224.0 : profile.intakeDurationDegrees, dieselCams ? "a diesel's" : "the use's");
            cams.exhaustDurationDegrees = derive("exhaust_duration_deg", "Exhaust duration", "deg",
                dieselCams ? 228.0 : profile.exhaustDurationDegrees, dieselCams ? "a diesel's" : "the use's");
            cams.intakeCenterlineDegrees = derive("intake_centerline_deg", "Intake centreline", "deg",
                dieselCams ? 114.0 : profile.intakeCenterlineDegrees, dieselCams ? "a diesel's" : "the use's");
            cams.exhaustCenterlineDegrees = derive("exhaust_centerline_deg", "Exhaust centreline", "deg",
                dieselCams ? 116.0 : profile.exhaustCenterlineDegrees, dieselCams ? "a diesel's" : "the use's");
            const auto liftRatio = dieselCams ? 0.30 : profile.liftOverValveDiameter;
            cams.intakeLiftMm = derive("intake_lift_mm", "Intake lift", "mm", liftRatio * intakeValveDiameter,
                format(liftRatio) + " x the intake valve diameter");
            cams.exhaustLiftMm = derive("exhaust_lift_mm", "Exhaust lift", "mm", 0.94 * cams.intakeLiftMm,
                "0.94 x the intake lift");
        }
        config.camshafts = cams;

        // Cylinders.
        for (std::uint32_t index = 0; index < count; ++index) {
            CylinderConfig cylinder;
            cylinder.id = index + 1;
            cylinder.boreMm = bore;
            cylinder.strokeMm = stroke;
            cylinder.connectingRodMm = rod;
            cylinder.pistonMassGrams = piston;
            cylinder.connectingRodMassGrams = rodMass;
            cylinder.compressionRatio = compression;
            cylinder.crankOffsetDegrees = tdc[index];
            cylinder.bankOffsetDegrees = places[index].bankDegrees;
            cylinder.bankId = places[index].bankIndex + 1;
            cylinder.intakeValveCount = rotary ? 3U : valvesPerSide;
            cylinder.exhaustValveCount = valvesPerSide;
            if (rotary) {
                // The 13B's three intake ports a face and two exhaust.
                cylinder.intakeValveDiameterMm = 32.0 * portScale;
                cylinder.exhaustValveDiameterMm = 30.0 * portScale;
            }
            if (radial) {
                cylinder.crankJournalId = 1;
                cylinder.connectingRodType = index == 0 ? ConnectingRodType::master : ConnectingRodType::articulated;
                if (index > 0) {
                    cylinder.masterCylinderId = 1;
                    cylinder.articulatedJournalRadiusMm = 0.27 * stroke;
                    cylinder.articulatedJournalAngleDegrees = places[index].bankDegrees;
                }
            }
            config.cylinders.push_back(cylinder);
        }
        if (rotary) {
            // The faces ride their rotor's eccentric: no journals to share.
        } else if (radial) {
            config.crankJournals = { { 1, 0.0, 0.5 * stroke, 1 } };
        } else {
            // Cylinders of one row whose pins point the same way share it.
            std::vector<CrankJournalConfig> journals;
            std::vector<double> journalRows;
            for (std::uint32_t index = 0; index < count; ++index) {
                const auto pin = std::fmod(std::fmod(tdc[index] - places[index].bankDegrees, 360.0) + 360.0, 360.0);
                std::size_t found = 0;
                while (found < journals.size()
                        && !(journalRows[found] == places[index].row
                            && std::abs(std::remainder(journals[found].angleDegrees - pin, 360.0)) < 1.0e-6))
                    ++found;
                if (found == journals.size()) {
                    journals.push_back({ static_cast<std::uint32_t>(found + 1), pin < 360.0 - 1.0e-9 ? pin : 0.0,
                        0.5 * stroke, 0 });
                    journalRows.push_back(places[index].row);
                }
                config.cylinders[index].crankJournalId = journals[found].id;
            }
            if (journals.size() < count) {
                config.crankJournals = std::move(journals);
            } else {
                for (auto& cylinder : config.cylinders) cylinder.crankJournalId = 0;
            }
        }
        if (rotary)
            designed.values.push_back({ "rotors", "Rotors", "", static_cast<double>(rotors), false,
                "three faces each; " + pinRule, false });
        else
            designed.values.push_back({ "crank_pins", "Crank pins", "",
                radial ? 1.0 : static_cast<double>(config.crankJournals.empty() ? count : config.crankJournals.size()),
                false, pinRule + "; fires " + intervalsText(analyseCrankBalance(config)), false });

        // Intake.
        auto& intake = config.intake;
        // A rotary's runner feeds a rotor, its port open 332 deg of a face's
        // cycle against a piston's ~250: tuned 4/3 as long.
        intake.runnerLengthMm = derive("intake_runner_length_mm", "Intake runner length", "mm",
            std::clamp((twoStroke ? 1.0e6 : rotary ? 2.5e6 : 1.9e6) / peakPowerRpm, 80.0, 600.0),
            twoStroke ? "1.0e6 / peak power rpm" : rotary ? "2.5e6 / peak power rpm: the port's longer opening"
                : "1.9e6 / peak power rpm (catalogue 1.6-2.3e6)");
        intake.runnerDiameterMm = derive("intake_runner_diameter_mm", "Intake runner diameter", "mm",
            (twoStroke ? 0.56 : rotary ? 0.41 : valvesPerSide > 1 ? 0.47 : 0.44) * bore,
            rotary ? "0.41 x the face's bore: the catalogue 13B's 50 mm a rotor"
                : "a share of the bore (catalogue 0.42-0.52)");
        // A two-stroke's throttle is the carburettor bore feeding the crankcase
        // circuit, as in the catalogue's RD350: one. Four bodies there idled a
        // two-stroke four at 2,070 rpm for 1,300, misfiring (journal 2026-10-19).
        const auto perCylinderThrottles = profile.throttlePerCylinder && !diesel && !twoStroke && count <= 16;
        intake.throttleCount = static_cast<std::uint32_t>(std::lround(derive("throttle_count", "Throttle bodies", "",
            perCylinderThrottles ? (rotary ? static_cast<double>(rotors) : countD) : 1.0,
            perCylinderThrottles ? (rotary ? "one per rotor" : "one per cylinder")
                : twoStroke ? "one, feeding the crankcase circuit" : "one")));
        // A two-stroke's bore feeds one cylinder, because its intake path is
        // that cylinder's crankcase, so it is sized from that cylinder and not
        // from the engine. Sized from the engine it was wrong by the cylinder
        // count: a 125 cc single got a 16.6 mm carburettor and a 500 cc four
        // got 33.2 mm for the same 125 cc cylinder, four times the area for
        // identical hardware. The factor doubles with the change so that the
        // twin the figure was calibrated on -- the RD350's 26 mm -- does not
        // move; every other cylinder count does.
        const auto throttleAreaFactor = diesel ? 0.27 : twoStroke ? 0.32 : profile.throttleAreaMm2PerLitreRpm;
        const auto throttleDisplacementLitres = twoStroke
            ? litres / std::max(1.0, countD) : litres;
        const auto throttleArea = throttleAreaFactor * throttleDisplacementLitres
            * config.redlineRpm;
        const auto throttleDiameter = std::sqrt(4.0 * throttleArea
            / (std::numbers::pi * static_cast<double>(std::max(1U, intake.throttleCount))));
        const auto throttleRule = format(throttleAreaFactor) + " mm2 per litre and redline rpm, shared by the bodies";
        // The floors are a 50 cc engine's; below that they shrink with it (a
        // 25 cc engine on a 15 mm bore and a 0.1 L crankcase did not fire).
        const auto smallShare = std::min(1.0, litres / 0.05);
        const auto throttleFloor = minimumThrottleDiameterMm * std::sqrt(smallShare);
        const auto plenumFloor = minimumPlenumVolumeLitres * smallShare;
        intake.throttleDiameterMm = derive("throttle_diameter_mm", "Throttle diameter", "mm",
            std::max(throttleFloor, throttleDiameter), throttleDiameter < throttleFloor
                ? throttleRule + " gives " + format(throttleDiameter) + ", raised to the simulator's floor"
                : throttleRule);
        const auto plenum = (spec.use == EngineUse::aircraft ? 0.7 : twoStroke ? 1.44 : 1.3) * litres;
        intake.plenumVolumeLitres = derive("plenum_volume_l", "Plenum", "L",
            std::max(plenumFloor, plenum), plenum < plenumFloor
                ? "per litre gives " + format(plenum) + ", raised to the simulator's floor"
                : std::string("per litre of displacement"));
        // A two-stroke's carburettor feeds one cylinder, so its idle air goes
        // with that cylinder: the RD350 runs 3 mm2 per carburettor on 175 cc.
        // Per litre of the whole engine, a four's carburettors each had a
        // twin's idle air and their pilots were sized to match (#63).
        intake.idleBypassAreaMm2 = derive("idle_bypass_area_mm2", "Idle bypass", "mm2",
            diesel ? 0.0 : twoStroke ? 18.0 * litres / std::max(1.0, countD) : 12.0 * litres,
            diesel ? "none: a diesel is unthrottled"
                : twoStroke ? "per litre of the cylinder its carburettor feeds" : "per litre");
        // The plate-in-bore gap leaks in proportion to the bore's rim. Held at
        // the simulator's 0.6 mm2 on a small bore, the leak alone outran a
        // 50 cc engine's idle air: 2,042 rpm for 1,350; 0.1 mm2 gave 1,329
        // (journal 2026-10-19).
        intake.closedThrottleLeakageAreaMm2 = derive("closed_throttle_leak_mm2", "Closed throttle leak", "mm2",
            intake.closedThrottleLeakageAreaMm2 * std::min(1.0, intake.throttleDiameterMm / 60.0),
            "per bore: 0.6 mm2 at 60 mm and over, in proportion below");
        // What the throttle breathes from. An airbox is the filter's housing;
        // a pod filter is the element alone on the mouth; an open trumpet is
        // the bare bellmouth, no element at all, which is the loud one.
        switch (spec.intakeStyle) {
        case IntakeStyle::automatic:
        case IntakeStyle::airbox:
            intake.airboxVolumeLitres = derive("airbox_volume_l", "Airbox", "L",
                spec.use == EngineUse::aircraft && spec.intakeStyle == IntakeStyle::automatic ? 0.0
                    : profile.motorcycle ? std::clamp(2.5 * litres, 3.0, 10.0) : 4.0 + 0.5 * litres,
                profile.motorcycle ? "2.5 x litres, 3-10 L" : "4 L + 0.5 x litres");
            // A car's box breathes through a snorkel: the catalogue's run
            // 200-400 mm at 1.35 times the throttles' area (1.1-1.56), with a
            // lip 1.17 times their bore. Without one the box breathed through
            // a hole of the throttle's bore. A motorcycle's box opens straight
            // under the tank: its mouth is the bores' combined area.
            if (intake.airboxVolumeLitres > 0.0) {
                const auto snorkel = !profile.motorcycle;
                const auto boresMm = intake.throttleDiameterMm
                    * std::sqrt(static_cast<double>(std::max(1U, intake.throttleCount)));
                intake.inletDuctLengthMm = derive("inlet_duct_length_mm", "Airbox snorkel length", "mm",
                    snorkel ? 250.0 : 0.0, snorkel ? "the catalogue cars' (200-400 mm)"
                        : "none: a motorcycle's box opens under the tank");
                intake.inletDuctDiameterMm = derive("inlet_duct_diameter_mm", "Airbox snorkel bore", "mm",
                    snorkel ? 1.16 * boresMm : 0.0, snorkel ? "1.35 x the throttles' area, the catalogue cars'"
                        : "none: the bores' combined area");
                intake.bellmouthDiameterMm = derive("bellmouth_diameter_mm", "Mouth lip", "mm",
                    snorkel ? 1.17 * intake.inletDuctDiameterMm : 0.0,
                    snorkel ? "1.17 x the snorkel's bore, the catalogue cars'" : "none: the bores' combined area");
            }
            break;
        case IntakeStyle::podFilter:
            intake.airboxVolumeLitres = 0.0;
            intake.podFilter = true;
            designed.values.push_back({ "intake_style", "Intake", "", 0.0, false,
                "a pod filter on the throttle: the element with no housing", false });
            break;
        case IntakeStyle::velocityStacks:
            intake.airboxVolumeLitres = 0.0;
            // A trumpet's lip flares to about 1.5 times its bore, which is
            // what keeps the inflow attached (Blair's bellmouth). The intake
            // radiates through one mouth, so several trumpets are one lip of
            // their combined area.
            intake.bellmouthDiameterMm = derive("bellmouth_diameter_mm", "Bellmouth lip", "mm",
                1.5 * intake.throttleDiameterMm * std::sqrt(static_cast<double>(std::max(1U, intake.throttleCount))),
                "1.5 x the throttle bore, over all the bores' area");
            designed.values.push_back({ "intake_style", "Intake", "", 0.0, false,
                "an open trumpet on the throttle: no filter", false });
            break;
        }
        intake.throttleGamma = profile.throttleGamma;
        if (carburetted) {
            // A carburettor's jets are sized the way a tuner sizes them and
            // not copied from one engine: from the fuel the engine needs at
            // each end of its range, and the head available to push it through
            // at that point. A jet's stamped number is its flow under one
            // metre of fuel, and flow goes as the square root of the head, so
            // a number is the fuel wanted divided by the square root of the
            // head there.
            //
            // On an RD350-sized design the arithmetic returns a 287 main and a
            // 41 pilot. The bike runs 260-290 and 30-40, which is the only
            // check that matters: neither number was fitted to it.
            auto& carburettor = intake.carburettor;
            carburettor.enabled = true;
            const auto bores = std::max(1.0, static_cast<double>(intake.throttleCount));
            // One crankcase circuit per cylinder, so a two-stroke's bores
            // swallow one cylinder, however they were sized; a four-stroke's
            // share the plenum and the whole engine's air.
            const auto displacementPerBoreM3 = twoStroke ? litres * 0.001 / std::max(1.0, countD)
                : litres * 0.001;
            const auto fuelDensityKgPerM3 = std::max(1.0,
                config.fuelProperties.densityKgPerL * 1'000.0);
            constexpr double ambientDensityKgPerM3 = 1.2;
            constexpr double gravityMPerS2 = 9.80665;
            // A two-stroke delivers about 0.85 of its swept volume at peak
            // power, a four-stroke fills about 0.90 every other turn.
            const auto wotAirKgPerSecond = twoStroke
                ? displacementPerBoreM3 * 0.85 * (peakPowerRpm / 60.0) * ambientDensityKgPerM3
                : displacementPerBoreM3 * 0.90 * (peakPowerRpm / 120.0) * ambientDensityKgPerM3;
            // A slide carburettor's venturi is about 0.8 of its nominal bore.
            // A car's is sized for its signal instead: Holley rates a
            // carburettor's flow at 1.5 inHg (5.1 kPa) of venturi depression,
            // and one sized to reach that at peak power still meters at low
            // speed, where a throttle-body-sized throat (2 kPa at peak) left
            // the main jet under its own lift up to 900 rpm, and the engine
            // stalled on opening.
            auto throatM = 0.80 * intake.throttleDiameterMm * 0.001;
            if (!twoStroke) {
                constexpr double ratedDepressionPa = 5'080.0;
                const auto ratedVelocityMps = std::sqrt(2.0 * ratedDepressionPa / ambientDensityKgPerM3);
                const auto venturiAreaM2 = wotAirKgPerSecond / (ambientDensityKgPerM3 * ratedVelocityMps) / bores;
                carburettor.venturiDiameterMm = derive("carb_venturi_mm", "Venturi", "mm",
                    std::sqrt(4.0 * venturiAreaM2 / std::numbers::pi) * 1'000.0,
                    "5.1 kPa (Holley's 1.5 inHg) of depression at peak power, per bore");
                throatM = carburettor.venturiDiameterMm * 0.001;
                // A venturi sized for its signal is a car's fixed one, with
                // a butterfly and no needle: a slide's taper, which holds the
                // main jet to a quarter of its area at a fifth of the travel,
                // left a 2-litre four at lambda 1.7-3.6 after a part-throttle
                // opening from idle, and it stalled.
                carburettor.fixedVenturi = true;
                // Weber's 32/36 DGV pumps about 0.5 cc a stroke on a 2-litre.
                carburettor.acceleratorPumpStrokeCc = derive("carb_pump_stroke_cc", "Accelerator pump", "cc",
                    0.25 * litres / bores, "0.25 cc per litre a full stroke, per bore");
            }
            const auto throatAreaM2 = 0.25 * std::numbers::pi * throatM * throatM * bores;
            const auto jetNumberFor = [&](double fuelKgPerSecond, double headM) {
                if (!(fuelKgPerSecond > 0.0) || !(headM > 0.0)) return 0.0;
                const auto ccPerMinute = fuelKgPerSecond / fuelDensityKgPerM3
                    * 60.0 * 1.0e6;
                return ccPerMinute / std::sqrt(headM);
            };

            // Main circuit, at peak power: the depression is the venturi's own,
            // from the velocity the air has to have to get through the throat.
            const auto throatVelocityMps = throatAreaM2 > 0.0
                ? wotAirKgPerSecond / (ambientDensityKgPerM3 * throatAreaM2) : 0.0;
            const auto wotDepressionPa = 0.5 * ambientDensityKgPerM3
                * throatVelocityMps * throatVelocityMps;
            const auto wotHeadM = wotDepressionPa
                / (fuelDensityKgPerM3 * gravityMPerS2);
            // A two-stroke runs its full load rich: the surplus fuel is what
            // cools the piston and carries the oil. A four-stroke's best power
            // is as rich, 12.5-13.
            constexpr double fullLoadAirFuelRatio = 12.8;
            // The hole a Dellorto or Keihin jet of that flow is stamped with.
            const auto holeOf = [&carburettor](double flowNumber) {
                return " (a " + std::to_string(static_cast<int>(std::lround(
                    jetHoleHundredthsMm(flowNumber, carburettor.jetDischargeCoefficient)))) + " hole)";
            };
            // Each bore has its own jets and meters its share of the air.
            const auto mainJetFlow = std::clamp(
                jetNumberFor(wotAirKgPerSecond / fullLoadAirFuelRatio / bores, wotHeadM), 20.0, 20'000.0);
            carburettor.mainJetFlowNumber = derive("carb_main_jet_flow", "Main jet", "cc/min", mainJetFlow,
                "the fuel peak power needs at AFR 12.8, over the root of the venturi head there"
                    + holeOf(mainJetFlow));

            // Pilot circuit, at idle: the depression is the whole drop from the
            // float bowl to a crankcase pumping about 20 kPa below ambient.
            // The air it meters for is what the idle air path passes at that
            // drop, the closed-slide leak and the bypass at the governor's
            // neutral opening; the pilot then follows that path as it opens.
            // Sized instead for a delivery ratio of 0.35 at idle, about three
            // times what a two-stroke idles on, it fed a 350 cc twin AFR ~7
            // and the twin flooded (journal 2026-10-23).
            // A four-stroke's manifold idles near 35 kPa, 65 below ambient,
            // and the air across the plates is then choked (35 / 101 is under
            // the critical 0.53): its flow is the sonic one, not the
            // incompressible one the crankcase's mild drop allows.
            const auto idleCrankcaseDropPa = twoStroke ? 20'000.0 : 65'000.0;
            const auto idleAirAreaM2 = (intake.closedThrottleLeakageAreaMm2 * bores
                + intake.idleBypassAreaMm2 * carburettorPilotSizingIdleOpening) * 1.0e-6;
            constexpr double ambientPressurePa = 101'325.0;
            constexpr double airGamma = 1.4;
            const auto chokedAirKgPerSecondPerM2 = ambientDensityKgPerM3
                * std::sqrt(airGamma * ambientPressurePa / ambientDensityKgPerM3)
                * std::pow(2.0 / (airGamma + 1.0), (airGamma + 1.0) / (2.0 * (airGamma - 1.0)));
            const auto idleAirKgPerSecond = intake.throttleDischargeCoefficient * idleAirAreaM2
                * (twoStroke ? std::sqrt(2.0 * ambientDensityKgPerM3 * idleCrankcaseDropPa)
                             : chokedAirKgPerSecondPerM2);
            const auto idleHeadM = idleCrankcaseDropPa
                / (fuelDensityKgPerM3 * gravityMPerS2);
            constexpr double idleAirFuelRatio = 13.5;
            // What reaches the engine is the jet throttled by everything after
            // it, so the jet itself has to be the larger by that factor.
            const auto pilotCircuitFactor = std::max(0.01,
                carburettor.pilotCircuitFactor);
            const auto pilotJetFlow = std::clamp(jetNumberFor(idleAirKgPerSecond / idleAirFuelRatio / bores,
                idleHeadM) / pilotCircuitFactor, 5.0, 5'000.0);
            carburettor.pilotJetFlowNumber = derive("carb_pilot_jet_flow", "Pilot jet", "cc/min", pilotJetFlow,
                std::string("the fuel the idle needs at AFR 13.5, over the root of the ")
                    + (twoStroke ? "crankcase" : "manifold") + " head, divided by the circuit's own restriction"
                    + holeOf(pilotJetFlow));
            if (!twoStroke) {
                // A four-stroke cranks with its start aid's extra air in, and
                // that air into a small manifold leaves little depression for
                // the pilot: a 750 cc four on four bores cranked at 75 kPa,
                // lambda 1.6-7, and never fired. Its enrichener is what a
                // real one starts on: a jet that runs the start aid's own air
                // at AFR 8 on the pilot's head. A choke's AFR 4 flooded a
                // warm 2-litre at AFR 6.8 once it ran with the aid still open.
                constexpr double starterAirFuelRatio = 8.0;
                const auto starterAirKgPerSecond = intake.throttleDischargeCoefficient
                    * intake.idleBypassAreaMm2 * 1.0e-6 * carburettorPilotSizingIdleOpening
                    * chokedAirKgPerSecondPerM2;
                const auto starterJetFlow = std::clamp(jetNumberFor(
                    starterAirKgPerSecond / starterAirFuelRatio / bores, idleHeadM), 0.5, 5'000.0);
                carburettor.starterJetFlowNumber = derive("carb_starter_jet_flow", "Starter jet", "cc/min",
                    starterJetFlow, "the start aid's air at AFR 8, over the root of the manifold head"
                        + holeOf(starterJetFlow));
            }
        }
        if (diesel) intake.throttleDischargeCoefficient = 0.86;
        config.plenumVolumeLitres = intake.plenumVolumeLitres;
        config.throttleDiameterMm = intake.throttleDiameterMm;

        // Exhaust: the family's silencer and mid pipe, pipes sized here.
        const auto exhaustKey = boosted && spec.induction == Induction::turbocharged && !profile.motorcycle
            ? "turbo_downpipe" : spec.layout == EngineLayout::radial ? "radial_collector" : profile.exhaustPart;
        // Independent ejector stacks: the outlet is each stack's flared lip.
        const auto stacks = std::string_view(exhaustKey) == "aircraft_manifold";
        auto exhaust = part(parts.exhausts, exhaustKey);
        if (spec.use != EngineUse::aircraft && !twoStroke) {
            exhaust.primaryLengthMm = derive("exhaust_primary_length_mm", "Exhaust primary length", "mm",
                spec.induction == Induction::turbocharged ? 430.0 : std::clamp(5.0e6 / peakPowerRpm, 250.0, 1'200.0),
                spec.induction == Induction::turbocharged ? "to the turbine, the catalogue turbos'"
                    : "5.0e6 / peak power rpm (catalogue 4.1-5.9e6)");
        }
        exhaust.primaryDiameterMm = twoStroke ? 0.45 * bore
            : derive("exhaust_primary_diameter_mm", "Exhaust primary diameter", "mm",
                (rotary ? 0.41 : 0.45) * bore, rotary ? "0.41 x the face's bore: the catalogue 13B's 50 mm a rotor"
                    : "0.45 x bore (catalogue 0.42-0.52)");
        const auto paths = twoStroke ? count
            : (spec.layout == EngineLayout::vLayout || spec.layout == EngineLayout::flat) && count >= 4 ? 2U
            : spec.layout == EngineLayout::custom ? bankCount : 1U;
        // A rotary's primaries are its rotors'.
        const auto merged = std::max(1U, (rotary ? rotors : count) / paths);
        const auto collectorDiameter = exhaust.primaryDiameterMm
            * std::max(1.0, std::sqrt(0.55 * static_cast<double>(merged)));
        exhaust.collectorDiameterMm = twoStroke ? collectorDiameter
            : derive("exhaust_collector_diameter_mm", "Exhaust collector diameter", "mm",
                collectorDiameter, "0.55 x the merged primaries' area");
        const auto outletDiameter = stacks ? 2.7 * exhaust.primaryDiameterMm : 1.1 * exhaust.collectorDiameterMm;
        exhaust.outletDiameterMm = twoStroke ? outletDiameter
            : derive("exhaust_outlet_diameter_mm", "Exhaust outlet diameter", "mm", outletDiameter,
                stacks ? "each stack's lip, 2.7 x the stack (the part's 52 -> 140 mm; narrower jets roar)"
                       : "1.1 x the collector");
        // A four-stroke's pipes after the collector, its silencer and its
        // tip: each the family's unless set. A two-stroke has its expansion
        // chambers below, an aircraft engine its stacks, a radial its ring.
        const auto pipedExhaust = !twoStroke && !stacks && !radial;
        ExhaustPlan plan;
        auto system = spec.exhaustSystem;
        if (pipedExhaust) {
            exhaust.midpipeLengthMm = derive("exhaust_midpipe_length_mm", "Mid pipe length", "mm",
                exhaust.midpipeLengthMm, "the family's (" + std::string(exhaustKey) + "): the downpipe, cat and "
                    "pipe to the silencer, a car's system 3.5-3.7 m from port to tip, a motorcycle's 1.2-1.5 m");
            // 0 already means "the collector's" downstream, so a design that
            // sets nothing keeps writing 0 and builds as it did.
            const auto pipeBore = derive("exhaust_midpipe_diameter_mm", "Mid pipe diameter", "mm",
                exhaust.midpipeDiameterMm > 0.0 ? exhaust.midpipeDiameterMm : exhaust.collectorDiameterMm,
                "the collector's");
            if (spec.overrides.contains("exhaust_midpipe_diameter_mm")) exhaust.midpipeDiameterMm = pipeBore;
            // A silencer's body goes with the pipe it ends: the catalogue's
            // boxes run 1.8-2.1 x their pipe across and 5-7.5 x it long.
            auto silencerRule = std::string("the family's");
            switch (spec.exhaustSilencer) {
            case ExhaustSilencer::automatic:
                // The family's box on a pipe it was not drawn for (a tank's
                // V12 on a car's family) grows with the pipe, in proportion.
                if (exhaust.mufflerChamberDiameterMm > 1.0 && exhaust.mufflerChamberDiameterMm < 1.5 * pipeBore) {
                    const auto scale = 1.8 * pipeBore / exhaust.mufflerChamberDiameterMm;
                    exhaust.mufflerChamberDiameterMm *= scale;
                    exhaust.mufflerChamberLengthMm *= scale;
                    exhaust.mufflerPackingThicknessMm *= scale;
                    silencerRule = "the family's, grown with its pipe to 1.8 x across";
                }
                break;
            case ExhaustSilencer::packedBox:
                exhaust.mufflerChamberDiameterMm = 2.1 * pipeBore;
                exhaust.mufflerChamberLengthMm = 7.5 * pipeBore;
                exhaust.mufflerRestriction = 0.46;
                exhaust.mufflerPackingFlowResistivityPaSPerM2 = 30'000.0;
                exhaust.mufflerPackingThicknessMm = 0.55 * pipeBore;
                exhaust.mufflerPerforatedOpenAreaRatio = 0.25;
                silencerRule = "a packed road box: 2.1 x the pipe across, 7.5 x long (turbo_downpipe's)";
                break;
            case ExhaustSilencer::straightThrough:
                exhaust.mufflerChamberDiameterMm = 1.8 * pipeBore;
                exhaust.mufflerChamberLengthMm = 5.0 * pipeBore;
                exhaust.mufflerRestriction = 0.20;
                exhaust.mufflerPackingFlowResistivityPaSPerM2 = 16'000.0;
                exhaust.mufflerPackingThicknessMm = 0.32 * pipeBore;
                exhaust.mufflerPerforatedOpenAreaRatio = 0.32;
                silencerRule = "a straight-through can: 1.8 x the pipe across, 5 x long";
                break;
            case ExhaustSilencer::none:
                exhaust.mufflerChamberDiameterMm = 0.0;
                exhaust.mufflerChamberLengthMm = 0.0;
                silencerRule = "none: an open pipe";
                break;
            }
            exhaust.mufflerChamberDiameterMm = derive("exhaust_silencer_diameter_mm", "Silencer body diameter", "mm",
                exhaust.mufflerChamberDiameterMm, silencerRule + " (0: no silencer)");
            exhaust.mufflerChamberLengthMm = derive("exhaust_silencer_length_mm", "Silencer body length", "mm",
                exhaust.mufflerChamberLengthMm, silencerRule + " (0: no silencer)");
            exhaust.mufflerRestriction = derive("exhaust_silencer_restriction", "Silencer flow loss", "",
                exhaust.mufflerRestriction, "its mean-flow loss coefficient: " + silencerRule);
            if (exhaust.mufflerChamberDiameterMm > 1.0 && exhaust.mufflerChamberDiameterMm <= pipeBore)
                throw std::invalid_argument("The silencer's body must be wider than its "
                    + format(pipeBore) + " mm pipe");
            // The packing fills at most the annulus between the core and the
            // shell. The family's boxes keep theirs as authored, so a design
            // that sets nothing builds as it did.
            if (spec.exhaustSilencer != ExhaustSilencer::automatic
                || spec.overrides.contains("exhaust_silencer_diameter_mm")
                || spec.overrides.contains("exhaust_midpipe_diameter_mm"))
                exhaust.mufflerPackingThicknessMm = std::min(exhaust.mufflerPackingThicknessMm,
                    std::max(0.0, 0.5 * (exhaust.mufflerChamberDiameterMm - pipeBore)));
            if (!(exhaust.mufflerPackingThicknessMm > 0.0)) {
                exhaust.mufflerPackingThicknessMm = 0.0;
                exhaust.mufflerPackingFlowResistivityPaSPerM2 = 0.0;
                exhaust.mufflerPerforatedOpenAreaRatio = 0.0;
            }
            plan.tailpipeLengthMm = derive("exhaust_tailpipe_length_mm", "Tailpipe length", "mm", 0.0,
                "none: the silencer's outlet is the tip");

            if (system == ExhaustSystem::dualHPipe || system == ExhaustSystem::dualXPipe) {
                if (bankCount < 2)
                    throw std::invalid_argument("An H- or an X-pipe joins two banks' pipes: this engine has one "
                                                "bank (choose a dual system to split its collector)");
                if (spec.induction == Induction::turbocharged)
                    throw std::invalid_argument("An H- or an X-pipe would join the banks ahead of the turbines, "
                                                "which the simulator places on each collector: choose single or "
                                                "dual on a turbocharged engine");
                plan.crossoverPositionMm = derive("exhaust_crossover_position_mm",
                    system == ExhaustSystem::dualHPipe ? "Balance pipe from the collectors" : "X from the collectors",
                    "mm", 0.35 * exhaust.midpipeLengthMm,
                    "a third of the mid pipe: under the gearbox, behind the cats");
                if (system == ExhaustSystem::dualHPipe) {
                    plan.balancePipeLengthMm = derive("exhaust_balance_pipe_length_mm", "Balance pipe length", "mm",
                        250.0, "the gap between two pipes under a car, 250 mm");
                    plan.balancePipeDiameterMm = derive("exhaust_balance_pipe_diameter_mm",
                        "Balance pipe diameter", "mm", pipeBore, "the mid pipe's");
                } else {
                    plan.crossoverCoupling = derive("exhaust_x_pipe_coupling", "X-pipe coupling", "", 0.30,
                        "the share of each pipe's pulse the X turns into the other (0-1): the LS3's estimate");
                    if (!(plan.crossoverCoupling >= 0.0 && plan.crossoverCoupling <= 1.0))
                        throw std::invalid_argument("The X-pipe coupling is between 0 and 1");
                }
            } else if (system == ExhaustSystem::dual && bankCount == 1) {
                plan.crossoverPositionMm = derive("exhaust_split_position_mm", "Split from the collector", "mm",
                    0.35 * exhaust.midpipeLengthMm, "a third of the mid pipe, then two pipes of 0.85 x its bore");
            }
        }
        config.exhaust = exhaust;
        plan.pipe = exhaust;
        // The family's layout, or the one asked for; built here when the
        // catalogue's compiler cannot express it.
        if (system == ExhaustSystem::automatic && plan.tailpipeLengthMm >= 1.0)
            system = paths == 1 ? ExhaustSystem::single : ExhaustSystem::dual;
        const auto designedNetwork = pipedExhaust && system != ExhaustSystem::automatic;
        // Which path each bank's exhaust goes into.
        std::vector<std::uint32_t> bankPath(radial ? count : bankCount, 0);
        auto pathCount = paths;
        if (designedNetwork) {
            pathCount = system == ExhaustSystem::dual && bankCount >= 2 ? 2U : 1U;
            for (std::uint32_t bank = 0; bank < bankCount; ++bank)
                bankPath[bank] = pathCount == 2 && bank >= bankCount / 2 ? 1U : 0U;
        } else {
            for (std::uint32_t bank = 0; bank < bankPath.size(); ++bank)
                bankPath[bank] = paths > 1 && !twoStroke ? bank % paths : 0U;
        }
        for (std::uint32_t path = 0; path < pathCount; ++path) {
            ExhaustPathConfig pathConfig;
            pathConfig.id = path + 1;
            pathConfig.geometry = exhaust;
            std::vector<std::vector<std::uint32_t>> pathBanks;
            for (std::uint32_t index = 0; index < count; ++index) {
                const auto owner = twoStroke ? index : pathCount == 1 ? 0U : bankPath[places[index].bankIndex];
                if (owner == path) pathConfig.cylinderIds.push_back(index + 1);
            }
            for (std::uint32_t bank = 0; bank < bankCount && designedNetwork; ++bank) {
                if (bankPath[bank] != path) continue;
                pathBanks.emplace_back();
                for (std::uint32_t index = 0; index < count; ++index)
                    if (places[index].bankIndex == bank) pathBanks.back().push_back(index + 1);
            }
            if (pathCount > 1)
                pathConfig.acousticPositionM.x = 0.9 * (static_cast<double>(path) / static_cast<double>(pathCount - 1) - 0.5);
            if (designedNetwork) {
                const auto join = system == ExhaustSystem::dualHPipe ? PathJoin::hPipe
                    : system == ExhaustSystem::dualXPipe ? PathJoin::xPipe
                    : system == ExhaustSystem::dual && pathCount == 1 ? PathJoin::split : PathJoin::single;
                pathConfig.network = designedExhaustNetwork(plan, join, pathBanks, pathConfig.acousticPositionM,
                    rotary ? 3 : 1);
            }
            config.exhaustPaths.push_back(std::move(pathConfig));
        }
        if (!designedNetwork)
            for (auto& path : config.exhaustPaths) compileLegacyExhaustPath(path, exhaustKey);
        if (pipedExhaust)
            designed.values.push_back({ "exhaust_system", "Exhaust system", "", 0.0, false,
                system == ExhaustSystem::single ? "one pipe and one silencer"
                    : system == ExhaustSystem::dual ? (bankCount >= 2 ? "one pipe and silencer per side"
                                                                      : "the collector split into two pipes")
                    : system == ExhaustSystem::dualHPipe ? "a pipe per side, joined by a balance pipe (H)"
                    : system == ExhaustSystem::dualXPipe ? "a pipe per side, crossing in an X"
                    : paths == 1 ? "the family's: one pipe" : "the family's: one pipe per bank", false });

        // Banks, for the cams and the exhaust each one feeds.
        for (std::uint32_t bank = 0; bank < (radial ? count : bankCount); ++bank) {
            CylinderBankConfig bankConfig;
            bankConfig.id = bank + 1;
            bankConfig.angleDegrees = banks[bank];
            bankConfig.camshafts = config.camshafts;
            for (std::uint32_t index = 0; index < count; ++index)
                if (places[index].bankIndex == bank) bankConfig.cylinderIds.push_back(index + 1);
            bankConfig.exhaustPathId = bankPath[bank] + 1;
            config.banks.push_back(std::move(bankConfig));
        }

        // Fuel, injection, ignition.
        if (diesel) config.fuelProperties = part(parts.fuels, "road_diesel_en590");
        if (twoStroke) {
            auto& injection = config.injection;
            injection.mode = InjectionMode::port;
            injection.startAngleDegrees = 250.0;
            injection.endAngleDegrees = 355.0;
            injection.injectorFlowMgPerSecond = 9'000.0;
            injection.railPressureBar = 3.0;
            injection.referencePressureBar = 3.0;
            injection.wallFilmFraction = 0.10;
            injection.vaporisationTimeConstantSeconds = 0.020;
            injection.portChargeCoolingEfficiency = 0.30;
        } else if (diesel) {
            config.injection = part(parts.injections, "common_rail_diesel");
        } else if (fuelSystem == FuelSystem::directInjection) {
            // Petrol injected into the cylinder for a homogeneous charge: on
            // the intake stroke, from about 300 deg before firing TDC, so the
            // spray has the intake's turbulence and the compression to mix
            // (Zhao, Lai and Harrington 1999). The part's own window, late in
            // compression, is the stratified lean-burn one, which needs a
            // spray-guided cloud the zero-dimensional charge does not hold.
            // A rotary's strokes are 270 deg: the same window, half as long
            // again (the 16X's direct injection, into the intake stroke).
            config.injection = part(parts.injections, "direct_injection");
            config.injection.startAngleDegrees = derive("injection_start_deg", "Injection starts", "deg ATDC",
                rotary ? 630.0 : 420.0, rotary ? "on the intake stroke, 450 deg before firing TDC: a homogeneous charge"
                    : "on the intake stroke, 300 deg before firing TDC: a homogeneous charge");
            config.injection.endAngleDegrees = derive("injection_end_deg", "Injection window ends", "deg ATDC",
                rotary ? 900.0 : 600.0, rotary ? "180 deg before firing TDC, early in compression"
                    : "120 deg before firing TDC, early in compression");
        } else {
            config.injection = part(parts.injections, profile.injectionPart);
            if (rotary) {
                // Onto the closed port ahead of its opening near 543 deg,
                // ending before it shuts: the catalogue 13B's window.
                config.injection.startAngleDegrees = 400.0;
                config.injection.endAngleDegrees = 860.0;
            }
        }
        if (twoStroke && fuelSystem == FuelSystem::directInjection) {
            // Into the cylinder once the piston has shut the exhaust port, so
            // none of it short-circuits into the pipe: the reason a two-stroke
            // is injected directly at all. The window follows the port.
            config.injection = part(parts.injections, "direct_injection");
            config.injection.mode = InjectionMode::direct;
        }
        // Rich full-load air flow at 1.1 x displacement and the boost, at the
        // redline; each injector at most 80 % open. A direct injector meters
        // only inside its window, which is a share of the cycle: a diesel's
        // 94 deg sized as if open the whole cycle held a 193 L marine six to
        // 1.7 g a stroke, lambda 3.0 at 3.0 bar (journal 2026-10-30).
        const auto airKgPerSecond = 1.18 * litres * 1.0e-3 * config.redlineRpm / (twoStroke ? 60.0 : 120.0) * 1.1 * boost;
        const auto fuelMgPerSecond = airKgPerSecond / (diesel ? 18.0 : 12.0) * 1.0e6;
        const auto directWindow = diesel || fuelSystem == FuelSystem::directInjection
            ? (twoStroke && !diesel ? 70.0 : std::fmod(config.injection.endAngleDegrees
                - config.injection.startAngleDegrees + cycleDegrees, cycleDegrees)) / cycleDegrees
            : 1.0;
        if (!(directWindow > 0.0))
            throw std::invalid_argument("The injection window ends where it starts");
        const auto injectorFlow = std::max(config.injection.injectorFlowMgPerSecond,
            fuelMgPerSecond / countD / 0.8 / directWindow);
        config.injection.injectorFlowMgPerSecond = twoStroke || carburetted ? injectorFlow
            : derive("injector_flow_mg_s", "Injector flow", "mg/s", injectorFlow,
                "the part's, or the full-load fuel at 80 % duty if more");
        if (diesel) {
            const auto scale = litres / countD / 0.492 * boost / 2.3;
            for (auto& sample : config.injection.fullLoadFuelLimit) {
                sample.milligramsPerCycle *= scale;
                sample.rpm *= config.redlineRpm / 5'000.0;
            }
            auto& calibration = config.combustionCalibration;
            calibration.wallHeatTransferCoefficientWPerK = 0.58;
            calibration.compressionIgnitionMixingTimeSeconds = 0.0016;
            calibration.compressionIgnitionPremixedFraction = 0.08;
            config.solver.maximumMechanicalFrequencyHz = 30'000.0;
            config.solver.maximumCrankDegreesPerStep = 1.5;
            config.solver.gasSubsteps = 3;
            config.ignition.timingCurve = stretchedTiming({ { 0.0, 2.0 }, { 0.2, 4.0 }, { 0.4, 7.0 }, { 0.7, 9.0 },
                { 1.0, 8.0 } }, config.redlineRpm);
        } else if (twoStroke) {
            config.ignition.timingCurve = stretchedTiming({ { 0.0, 12.0 }, { 0.2, 20.0 }, { 1.05, 20.0 } },
                config.redlineRpm);
        } else if (rotary) {
            // Eccentric-shaft degrees, the catalogue 13B's curve (estimated);
            // boosted, 10 deg less at load, the knock limit measured on the
            // dyno holding it below that.
            config.ignition.timingCurve = boosted
                ? stretchedTiming({ { 0.0, 5.0 }, { 0.13, 10.0 }, { 0.33, 14.0 }, { 0.61, 18.0 }, { 0.83, 20.0 },
                    { 1.0, 18.0 } }, config.redlineRpm)
                : stretchedTiming({ { 0.0, 5.0 }, { 0.13, 10.0 }, { 0.33, 20.0 }, { 0.61, 28.0 }, { 0.83, 30.0 },
                    { 1.0, 28.0 } }, config.redlineRpm);
            // A leading and a trailing plug in each housing.
            config.combustionCalibration.ignitionSiteCount = 2;
        } else if (spec.use == EngineUse::aircraft) {
            config.ignition.timingCurve = stretchedTiming({ { 0.0, 4.0 }, { 0.24, 9.0 }, { 0.42, 14.0 },
                { 0.63, 18.0 }, { 0.84, 16.0 } }, config.redlineRpm);
        } else if (boosted) {
            config.ignition.timingCurve = stretchedTiming({ { 0.0, 7.0 }, { 0.17, 12.0 }, { 0.34, 17.0 },
                { 0.5, 16.0 }, { 0.75, 19.0 }, { 1.03, 17.0 } }, config.redlineRpm);
        } else {
            config.ignition.timingCurve = stretchedTiming({ { 0.0, 8.0 }, { 0.12, 13.0 }, { 0.25, 22.0 },
                { 0.45, 30.0 }, { 0.7, 33.0 }, { 1.03, 30.0 } }, config.redlineRpm);
        }
        designed.values.push_back({ "ignition_curve", "Ignition curve", "", 0.0, false,
            diesel ? "the diesel template stretched to the redline"
                   : "the family template stretched to the redline, held under the knock limit "
                     "measured on the dyno when it is built", false });

        // Two-stroke ports and chamber.
        if (twoStroke) {
            const auto sport = spec.use == EngineUse::sportMotorcycle || spec.use == EngineUse::sportCar;
            config.twoStroke.exhaustPortOpenDegrees = derive("exhaust_port_open_deg", "Exhaust port opens", "deg ATDC",
                sport ? 82.0 : 92.0, sport ? "a sport port (RD350LC: 82)" : "a road port");
            config.twoStroke.transferPortOpenDegrees = derive("transfer_port_open_deg", "Transfer ports open",
                "deg ATDC", sport ? 116.0 : 118.0, "the use's");
            config.twoStroke.intake = spec.twoStrokeIntake;
            if (fuelSystem == FuelSystem::directInjection) {
                const auto exhaustCloses = 360.0 - config.twoStroke.exhaustPortOpenDegrees;
                config.injection.startAngleDegrees = derive("injection_start_deg", "Injection starts", "deg ATDC",
                    exhaustCloses, "as the piston shuts the exhaust port: nothing short-circuits");
                config.injection.endAngleDegrees = derive("injection_end_deg", "Injection window ends", "deg ATDC",
                    std::min(350.0, config.injection.startAngleDegrees + 70.0), "70 deg later");
            }
            if (spec.twoStrokeIntake == TwoStrokeIntake::cylinderPort) {
                config.twoStroke.intakePortReed = spec.twoStrokeIntakeReed;
                designed.values.push_back({ "two_stroke_intake", "Intake", "", 0.0, false,
                    spec.twoStrokeIntakeReed ? "a port in the cylinder wall with a reed in it"
                                             : "a port in the cylinder wall (piston port)", false });
                // The skirt uncovers it this far before top dead centre and
                // covers it as far after: Jennings gives a road engine about
                // 140-150 deg of inlet duration and a racing one about 190.
                config.twoStroke.intakePortOpenDegreesBtdc = derive("intake_port_open_deg_btdc",
                    "Intake port opens", "deg BTDC", sport ? 95.0 : 72.0,
                    sport ? "a racing port, about 190 deg open" : "a road port, about 145 deg open");
                config.twoStroke.intakePortWidthMm = derive("intake_port_width_mm", "Intake port width", "mm",
                    0.70 * bore, "0.70 x bore");
            } else {
                designed.values.push_back({ "two_stroke_intake", "Intake", "", 0.0, false,
                    "a reed block on the crankcase", false });
            }
            config.twoStroke.crankcaseCompressionRatio = derive("crankcase_compression_ratio",
                "Crankcase compression", "", 1.35, "1.25-1.5 on road and race engines");
            config.twoStroke.scavengeLoopLengthRatio = derive("scavenge_loop_length_ratio",
                "Scavenging loop length", "x bore", 1.8,
                "a loop-scavenged cylinder: up the far wall, over the head, back to the port");
            // A sport port's long exhaust window throws the charge out at low
            // speed: on a designed 500 cc twin, 30 N.m at 4,000-4,750 rpm
            // against 46-50 from 7,000. A power valve lowers the port there and
            // lifts as the pipe comes in; 12 deg later shut (82 -> 94, 196 deg
            // of window -> 172) gave +20-39 % from 3,250 to 5,750 rpm and the
            // same top end (journal 2026-10-23).
            if (sport) {
                config.twoStroke.powerValveClosedExhaustPortOpenDegrees = derive(
                    "power_valve_closed_exhaust_port_open_deg", "Power valve shut: exhaust opens", "deg ATDC",
                    config.twoStroke.exhaustPortOpenDegrees + 12.0, "12 deg later than the open port");
                config.twoStroke.powerValveOpeningStartRpm = derive("power_valve_opening_start_rpm",
                    "Power valve starts to open", "rpm", 0.65 * peakPowerRpm, "65 % of peak power speed");
                config.twoStroke.powerValveFullyOpenRpm = derive("power_valve_fully_open_rpm",
                    "Power valve fully open", "rpm", 0.90 * peakPowerRpm, "90 % of peak power speed");
            }
            config.combustionCalibration.chamberTurbulenceIntensityRatio = derive("chamber_turbulence_ratio",
                "Chamber turbulence", "", 2.0, "a squish head, as the RD350LC's");

            // One expansion chamber per cylinder. Its tuned length (port to
            // mid-baffle) returns the blowdown's reflected wave to the port
            // as it closes at peak power: L = a x exhaust duration / (12 x
            // rpm), a = 500 m/s (Blair). Its shape is the RD350LC's
            // (engines/17: 940 mm tuned, 64 mm bore), scaled by both.
            const auto exhaustDuration = 360.0 - 2.0 * config.twoStroke.exhaustPortOpenDegrees;
            const auto tuned = derive("expansion_chamber_tuned_length_mm", "Expansion chamber tuned length", "mm",
                std::clamp(500.0 * exhaustDuration / (12.0 * peakPowerRpm) * 1'000.0, 500.0, 1'800.0),
                "500 m/s x exhaust duration / (12 x peak power rpm)");
            const auto length = [tuned](double fraction) { return std::max(100.0, fraction * tuned); };
            const auto cylinderLitres = litres / countD;
            for (auto& path : config.exhaustPaths) {
                const auto base = path.id * 100;
                const auto component = [base](std::uint32_t index, ExhaustComponentType type, double lengthMm,
                                              double diameterMm, double outletMm) {
                    ExhaustComponentConfig part;
                    part.id = base + index;
                    part.type = type;
                    part.lengthMm = lengthMm;
                    part.diameterMm = diameterMm;
                    part.outletDiameterMm = outletMm;
                    return part;
                };
                ExhaustNetworkConfig chamber;
                chamber.components = {
                    component(1, ExhaustComponentType::pipe, length(0.277), 0.625 * bore, 0.0),
                    component(2, ExhaustComponentType::pipe, length(0.234), 0.625 * bore, 1.094 * bore),
                    component(3, ExhaustComponentType::pipe, length(0.213), 1.094 * bore, 1.719 * bore),
                    component(4, ExhaustComponentType::pipe, length(0.138), 1.719 * bore, 0.0),
                    component(5, ExhaustComponentType::pipe, length(0.277), 1.719 * bore, 0.406 * bore),
                    component(6, ExhaustComponentType::pipe, length(0.234), 0.406 * bore, 0.0),
                    component(7, ExhaustComponentType::muffler, length(0.319), 1.094 * bore, 0.0),
                    component(8, ExhaustComponentType::outlet, 150.0, 0.4375 * bore, 0.0),
                };
                const auto core = std::numbers::pi * 0.25 * std::pow(1.094 * bore, 2.0) * length(0.319) * 1.0e-6;
                chamber.components[6].volumeLitres = std::max(1.2 * cylinderLitres / 0.1737, 1.1 * core);
                chamber.components[6].restriction = 0.10;
                auto& outlet = chamber.components[7];
                outlet.dischargeCoefficient = 0.78;
                outlet.acousticPositionM = path.acousticPositionM;
                outlet.acousticAxis = path.acousticAxis;
                outlet.acousticTermination = path.acousticTermination;
                chamber.cylinderConnections.push_back({ path.cylinderIds.front(), base + 1 });
                for (std::uint32_t index = 1; index < 8; ++index)
                    chamber.connections.push_back({ base + index, base + index + 1 });
                path.network = std::move(chamber);
                path.inheritsGlobalGeometry = false;
            }
        }

        // Forced induction.
        if (spec.induction == Induction::turbocharged) {
            config.forcedInduction = sizedTurbocharger(config);
            auto& turbo = config.forcedInduction;
            turbo.pressureRatio = boost;
            turbo.wastegatePressureRatio = boost - 0.04;
            turbo.compressorInducerDiameterMm = 7.8 * std::sqrt(litres * config.redlineRpm / 1'000.0 * boost);
            turbo.turbineExducerDiameterMm = 0.9 * turbo.compressorInducerDiameterMm;
            turbo.shaftInertiaKgM2 = 4.75e-13 * std::pow(turbo.compressorInducerDiameterMm, 5.0);
            turbo.designShaftSpeedRpm = turbochargerDesignShaftSpeedRpm(boost, turbo.compressorInducerDiameterMm);
            config = withTurbochargerCount(std::move(config), spec.turbochargerCount);
        } else if (spec.induction == Induction::supercharged
                   || spec.induction == Induction::centrifugalSupercharged) {
            auto& charger = config.forcedInduction;
            charger.enabled = true;
            charger.type = ForcedInductionType::supercharger;
            charger.pressureRatio = boost;
            charger.compressorEfficiency = 0.70;
            charger.chargeTemperatureRiseC = 48.0;
            const auto aircraft = spec.use == EngineUse::aircraft;
            if (aircraft || spec.induction == Induction::centrifugalSupercharged) {
                // An aircraft's wheel is regulated from 60 % of its speed up;
                // a road one (a ProCharger's) reaches its boost at the
                // redline, its pressure rising with the square of the speed.
                charger.fullBoostRpm = (aircraft ? 0.6 : 1.0) * config.redlineRpm;
                charger.compressorBladeCount = 12;
                charger.superchargerDriveRatio = 8.1;
            } else {
                // A Roots geared to 14,000 rotor rpm at the redline (an Eaton
                // M-series' limit), its displacement from the boost there.
                charger.fullBoostRpm = config.redlineRpm;
                charger.superchargerLobeCount = 4;
                charger.superchargerDriveRatio = std::clamp(14'000.0 / config.redlineRpm, 0.5, 4.0);
                charger.superchargerDisplacementLitres = rootsDisplacementLitres(config);
            }
        }

        // Drivetrain: the family's gearbox and vehicle, the final drive set
        // for the top speed the estimated power reaches in top gear.
        config.transmission = part(parts.transmissions, profile.transmissionPart);
        config.vehicle = part(parts.vehicles, diesel && !profile.motorcycle && spec.use != EngineUse::aircraft
            ? "diesel_hatch" : profile.vehiclePart);
        const auto bmepAtPeakPowerPa = 1.0e5 * (diesel ? 7.8 * boost : twoStroke ? 7.0
            : spec.use == EngineUse::aircraft ? 9.5 * boost : 11.5 * (boosted ? 0.8 * boost : 1.0));
        const auto estimatedPowerW = bmepAtPeakPowerPa * litres * 1.0e-3 * peakPowerRpm / 60.0
            / (twoStroke ? 1.0 : 2.0);
        const auto estimatedTorqueNm = bmepAtPeakPowerPa * (diesel ? 1.3 : 1.15) * litres * 1.0e-3
            / (twoStroke ? 2.0 * std::numbers::pi : 4.0 * std::numbers::pi);
        designed.values.push_back({ "estimated_power_kw", "Estimated power", "kW", estimatedPowerW * 1.0e-3, false,
            "a family BMEP at peak power, for the gearing (not a result)", false });
        config.transmission.maxClutchTorqueNm = derive("clutch_capacity_nm", "Clutch capacity", "N m",
            std::max(60.0, 1.6 * estimatedTorqueNm), "1.6 x the estimated peak torque");
        if (spec.use != EngineUse::aircraft && !config.transmission.gearRatios.empty()) {
            const auto speed = topSpeedMps(estimatedPowerW * config.transmission.drivelineEfficiency, config.vehicle);
            const auto engineRadPerS = 1.05 * peakPowerRpm * 2.0 * std::numbers::pi / 60.0;
            const auto wheelRadPerS = speed / config.vehicle.tireRadiusM;
            // A very small engine in a car's gearing would need more than the
            // simulator takes (a 50 cc single: 20): it is held there.
            const auto finalDrive = engineRadPerS / (wheelRadPerS * config.transmission.gearRatios.back());
            const auto rule = "top gear at 1.05 x peak power speed at the top speed (" + format(speed * 3.6) + " km/h)";
            config.transmission.finalDriveRatio = derive("final_drive_ratio", "Final drive", "",
                std::clamp(finalDrive, 1.0, maximumFinalDriveRatio),
                finalDrive > maximumFinalDriveRatio || finalDrive < 1.0
                    ? rule + " gives " + format(finalDrive) + ", held at the simulator's limit" : rule);
        }

        config.thermal.liquidCooled = derive("liquid_cooled", "Liquid cooled", "", radial ? 0.0 : 1.0,
            radial ? "a radial is air-cooled" : "liquid") > 0.5;
        if (!config.thermal.liquidCooled) config.thermal.engineDrivenFan = !radial;

        if (rotary) {
            config.rotary = rotor;
            applyRotaryFaceGeometry(config);
        }
        if (const auto unknown = derive.unknownOverride(); !unknown.empty())
            throw std::invalid_argument("Unknown value: " + unknown);
        normaliseEngineConfig(config);
        if (auto error = validateEngineConfig(config)) throw std::invalid_argument(*error);
        designed.config = std::move(config);
    } catch (const std::exception& error) {
        designed.error = error.what();
        designed.config.reset();
    }
    return designed;
}

} // namespace enginelab
