#include <enginelab/catalog/EngineDesign.hpp>
#include <enginelab/foundation/CrankBalance.hpp>
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
    try {
        const auto profile = profileFor(spec.use);
        const auto twoStroke = spec.cycle == EngineCycle::twoStroke;
        const auto diesel = spec.fuel == FuelType::diesel;
        const auto boosted = spec.induction != Induction::natural;
        const auto count = spec.cylinderCount;
        const auto countD = static_cast<double>(count);
        Derivation derive(spec.overrides, designed.values);
        EngineConfig config;
        config.name = spec.name;
        config.cycle = spec.cycle;
        config.fuel = spec.fuel;
        config.layout = spec.layout;

        // Size. Bore and stroke as given, or from the displacement and the
        // use's bore/stroke ratio.
        const auto ratio = derive("bore_stroke_ratio", "Bore / stroke", "",
            diesel ? 0.85 : twoStroke ? 1.18 : profile.boreStrokeRatio,
            "the use's (cars ~1.0, sport bikes 1.4, cruisers 0.9, diesels 0.85)");
        auto boreFromVolume = 0.0;
        if (spec.displacementLitres > 0.0)
            boreFromVolume = std::cbrt(spec.displacementLitres / countD * 1.0e6 * 4.0 * ratio / std::numbers::pi);
        const auto bore = derive("bore_mm", "Bore", "mm", spec.boreMm > 0.0 ? spec.boreMm : boreFromVolume,
            spec.boreMm > 0.0 ? "given" : "from the displacement and bore / stroke");
        const auto stroke = derive("stroke_mm", "Stroke", "mm",
            spec.strokeMm > 0.0 ? spec.strokeMm : bore / std::max(0.1, ratio),
            spec.strokeMm > 0.0 ? "given" : "bore / (bore / stroke)");
        if (!(bore >= 20.0 && bore <= 200.0 && stroke >= 20.0 && stroke <= 200.0))
            throw std::invalid_argument("Give a bore and a stroke of 20-200 mm, or a displacement");
        const auto litres = cylinderDisplacementLitres(bore, stroke) * countD;
        if (litres > 20.0) throw std::invalid_argument("The simulator runs engines of 20 litres at most");
        if (litres < 0.05 - 1.0e-9) throw std::invalid_argument("The simulator runs engines of 50 cc at least");

        // Speeds: the redline from the mean piston speed the use reaches.
        const auto pistonSpeed = diesel ? std::min(16.0, profile.redlinePistonSpeedMps)
            : twoStroke ? 17.5 : profile.redlinePistonSpeedMps - (boosted ? 1.5 : 0.0);
        auto redlineDerived = pistonSpeed * 60.0 / (2.0 * stroke * 1.0e-3);
        if (diesel) redlineDerived = std::min(redlineDerived, 5'000.0);
        redlineDerived = std::min(redlineDerived, 16'000.0);
        config.redlineRpm = derive("redline_rpm", "Redline", "rpm", std::round(redlineDerived / 100.0) * 100.0,
            "mean piston speed " + format(pistonSpeed) + " m/s at the redline");
        const auto peakPowerRpm = derive("peak_power_rpm", "Peak power speed", "rpm",
            (diesel ? 0.8 : 0.9) * config.redlineRpm, diesel ? "0.8 x redline" : "0.9 x redline");
        config.idleRpm = derive("idle_rpm", "Idle", "rpm", twoStroke ? 1'300.0 : profile.idleRpm, "the use's");
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
        const auto compression = derive("compression_ratio", "Compression ratio", "",
            std::max(diesel ? 14.0 : 7.0, compressionDerived),
            diesel ? "a direct-injection diesel's" : spec.use == EngineUse::aircraft ? "an aircraft engine on avgas"
            : "14 - 0.033 x bore + the use's, less 2 per bar of boost");

        // Moving parts.
        const auto rod = derive("connecting_rod_mm", "Connecting rod", "mm",
            twoStroke ? std::max(2.0 * stroke, 1.7 * bore) : std::max(1.6 * stroke, 1.25 * bore),
            twoStroke ? "max(2.0 x stroke, 1.7 x bore)" : "max(1.6 x stroke, 1.25 x bore)");
        const auto density = diesel ? 0.96 : twoStroke ? 0.88
            : profile.pistonDensityGPerCm3 * (boosted ? 1.25 : 1.0);
        const auto pistonByBore = density * std::pow(bore / 10.0, 3.0);
        const auto piston = derive("piston_mass_g", "Piston mass", "g",
            std::clamp(pistonByBore, minimumPistonMassGrams, 2'400.0), pistonByBore < minimumPistonMassGrams
                ? format(density) + " g/cm3 x bore^3 gives " + format(pistonByBore) + ", raised to the simulator's floor"
                : format(density) + " g/cm3 x bore^3");
        const auto rodMass = derive("connecting_rod_mass_g", "Connecting rod mass", "g", 1.3 * piston,
            "1.3 x the piston");
        const auto valvesPerSide = static_cast<std::uint32_t>(std::lround(derive("valves_per_side",
            "Valves per side", "", diesel ? 2.0 : static_cast<double>(profile.valvesPerSide), "the use's")));
        // Below 0.22 L the per-litre rule falls under the simulator's 0.01
        // floor (a 125 cc sport single: 0.0056): it is raised to the floor.
        const auto inertiaPerLitre = (diesel ? 0.135 : twoStroke ? 0.087 : profile.inertiaKgM2PerLitre) * litres;
        config.rotatingInertiaKgM2 = derive("rotating_inertia_kg_m2", "Rotating inertia", "kg m2",
            std::max(minimumRotatingInertiaKgM2, inertiaPerLitre),
            inertiaPerLitre < minimumRotatingInertiaKgM2
                ? "per litre gives " + format(inertiaPerLitre) + ", raised to the simulator's floor"
                : std::string("per litre, the use's (cars 0.09-0.10, sport bikes 0.045, cruisers 0.11)"));
        config.frictionCoefficient = derive("friction_coefficient", "Friction coefficient", "",
            diesel ? 0.16 : twoStroke ? 0.07 : profile.frictionCoefficient, "the use's");
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
        if (!spec.crankOffsetsDegrees.empty()) {
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
        if (!twoStroke) {
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
            cylinder.intakeValveCount = valvesPerSide;
            cylinder.exhaustValveCount = valvesPerSide;
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
        if (radial) {
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
        designed.values.push_back({ "crank_pins", "Crank pins", "",
            radial ? 1.0 : static_cast<double>(config.crankJournals.empty() ? count : config.crankJournals.size()),
            false, pinRule + "; fires " + intervalsText(analyseCrankBalance(config)), false });

        // Intake.
        auto& intake = config.intake;
        intake.runnerLengthMm = derive("intake_runner_length_mm", "Intake runner length", "mm",
            std::clamp((twoStroke ? 1.0e6 : 1.9e6) / peakPowerRpm, 80.0, 600.0),
            twoStroke ? "1.0e6 / peak power rpm" : "1.9e6 / peak power rpm (catalogue 1.6-2.3e6)");
        intake.runnerDiameterMm = derive("intake_runner_diameter_mm", "Intake runner diameter", "mm",
            (twoStroke ? 0.56 : valvesPerSide > 1 ? 0.47 : 0.44) * bore, "a share of the bore (catalogue 0.42-0.52)");
        // A two-stroke's throttle is the carburettor bore feeding the crankcase
        // circuit, as in the catalogue's RD350: one. Four bodies there idled a
        // two-stroke four at 2,070 rpm for 1,300, misfiring (journal 2026-10-19).
        const auto perCylinderThrottles = profile.throttlePerCylinder && !diesel && !twoStroke && count <= 16;
        intake.throttleCount = static_cast<std::uint32_t>(std::lround(derive("throttle_count", "Throttle bodies", "",
            perCylinderThrottles ? countD : 1.0,
            perCylinderThrottles ? "one per cylinder" : twoStroke ? "one, feeding the crankcase circuit" : "one")));
        const auto throttleAreaFactor = diesel ? 0.27 : twoStroke ? 0.16 : profile.throttleAreaMm2PerLitreRpm;
        const auto throttleArea = throttleAreaFactor * litres * config.redlineRpm;
        const auto throttleDiameter = std::sqrt(4.0 * throttleArea
            / (std::numbers::pi * static_cast<double>(std::max(1U, intake.throttleCount))));
        const auto throttleRule = format(throttleAreaFactor) + " mm2 per litre and redline rpm, shared by the bodies";
        intake.throttleDiameterMm = derive("throttle_diameter_mm", "Throttle diameter", "mm",
            std::max(minimumThrottleDiameterMm, throttleDiameter), throttleDiameter < minimumThrottleDiameterMm
                ? throttleRule + " gives " + format(throttleDiameter) + ", raised to the simulator's floor"
                : throttleRule);
        const auto plenum = (spec.use == EngineUse::aircraft ? 0.7 : twoStroke ? 1.44 : 1.3) * litres;
        intake.plenumVolumeLitres = derive("plenum_volume_l", "Plenum", "L",
            std::max(minimumPlenumVolumeLitres, plenum), plenum < minimumPlenumVolumeLitres
                ? "per litre gives " + format(plenum) + ", raised to the simulator's floor"
                : std::string("per litre of displacement"));
        intake.idleBypassAreaMm2 = derive("idle_bypass_area_mm2", "Idle bypass", "mm2",
            diesel ? 0.0 : (twoStroke ? 9.0 : 12.0) * litres, diesel ? "none: a diesel is unthrottled" : "per litre");
        // The plate-in-bore gap leaks in proportion to the bore's rim. Held at
        // the simulator's 0.6 mm2 on a small bore, the leak alone outran a
        // 50 cc engine's idle air: 2,042 rpm for 1,350; 0.1 mm2 gave 1,329
        // (journal 2026-10-19).
        intake.closedThrottleLeakageAreaMm2 = derive("closed_throttle_leak_mm2", "Closed throttle leak", "mm2",
            intake.closedThrottleLeakageAreaMm2 * std::min(1.0, intake.throttleDiameterMm / 60.0),
            "per bore: 0.6 mm2 at 60 mm and over, in proportion below");
        intake.airboxVolumeLitres = derive("airbox_volume_l", "Airbox", "L",
            spec.use == EngineUse::aircraft ? 0.0 : profile.motorcycle ? std::clamp(2.5 * litres, 3.0, 10.0)
                : 4.0 + 0.5 * litres, profile.motorcycle ? "2.5 x litres, 3-10 L" : "4 L + 0.5 x litres");
        intake.throttleGamma = profile.throttleGamma;
        if (diesel) intake.throttleDischargeCoefficient = 0.86;
        config.plenumVolumeLitres = intake.plenumVolumeLitres;
        config.throttleDiameterMm = intake.throttleDiameterMm;

        // Exhaust: the family's silencer and mid pipe, pipes sized here.
        const auto exhaustKey = boosted && spec.induction == Induction::turbocharged && !profile.motorcycle
            ? "turbo_downpipe" : spec.layout == EngineLayout::radial ? "radial_collector" : profile.exhaustPart;
        // Independent ejector stacks: the outlet is each stack's flared lip.
        const auto stacks = std::string_view(exhaustKey) == "aircraft_manifold";
        auto exhaust = part(parts.exhausts, exhaustKey);
        if (spec.use != EngineUse::aircraft) {
            exhaust.primaryLengthMm = derive("exhaust_primary_length_mm", "Exhaust primary length", "mm",
                spec.induction == Induction::turbocharged ? 430.0 : std::clamp(5.0e6 / peakPowerRpm, 250.0, 1'200.0),
                spec.induction == Induction::turbocharged ? "to the turbine, the catalogue turbos'"
                    : "5.0e6 / peak power rpm (catalogue 4.1-5.9e6)");
        }
        exhaust.primaryDiameterMm = derive("exhaust_primary_diameter_mm", "Exhaust primary diameter", "mm",
            0.45 * bore, "0.45 x bore (catalogue 0.42-0.52)");
        const auto paths = twoStroke ? count
            : (spec.layout == EngineLayout::vLayout || spec.layout == EngineLayout::flat) && count >= 4 ? 2U
            : spec.layout == EngineLayout::custom ? bankCount : 1U;
        const auto merged = std::max(1U, count / paths);
        exhaust.collectorDiameterMm = derive("exhaust_collector_diameter_mm", "Exhaust collector diameter", "mm",
            exhaust.primaryDiameterMm * std::max(1.0, std::sqrt(0.55 * static_cast<double>(merged))),
            "0.55 x the merged primaries' area");
        exhaust.outletDiameterMm = derive("exhaust_outlet_diameter_mm", "Exhaust outlet diameter", "mm",
            stacks ? 2.7 * exhaust.primaryDiameterMm : 1.1 * exhaust.collectorDiameterMm,
            stacks ? "each stack's lip, 2.7 x the stack (the part's 52 -> 140 mm; narrower jets roar)"
                   : "1.1 x the collector");
        config.exhaust = exhaust;
        for (std::uint32_t path = 0; path < paths; ++path) {
            ExhaustPathConfig pathConfig;
            pathConfig.id = path + 1;
            pathConfig.geometry = exhaust;
            for (std::uint32_t index = 0; index < count; ++index) {
                const auto owner = twoStroke ? index : paths == 1 ? 0U : places[index].bankIndex % paths;
                if (owner == path) pathConfig.cylinderIds.push_back(index + 1);
            }
            if (paths > 1)
                pathConfig.acousticPositionM.x = 0.9 * (static_cast<double>(path) / static_cast<double>(paths - 1) - 0.5);
            config.exhaustPaths.push_back(std::move(pathConfig));
        }
        for (auto& path : config.exhaustPaths) compileLegacyExhaustPath(path, exhaustKey);

        // Banks, for the cams and the exhaust each one feeds.
        for (std::uint32_t bank = 0; bank < (radial ? count : bankCount); ++bank) {
            CylinderBankConfig bankConfig;
            bankConfig.id = bank + 1;
            bankConfig.angleDegrees = banks[bank];
            bankConfig.camshafts = config.camshafts;
            for (std::uint32_t index = 0; index < count; ++index)
                if (places[index].bankIndex == bank) bankConfig.cylinderIds.push_back(index + 1);
            bankConfig.exhaustPathId = paths > 1 && !twoStroke ? bank % paths + 1 : 1;
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
        } else {
            config.injection = part(parts.injections, diesel ? "common_rail_diesel" : profile.injectionPart);
        }
        // Rich full-load air flow at 1.1 x displacement and the boost, at the
        // redline; each injector at most 80 % open.
        const auto airKgPerSecond = 1.18 * litres * 1.0e-3 * config.redlineRpm / (twoStroke ? 60.0 : 120.0) * 1.1 * boost;
        const auto fuelMgPerSecond = airKgPerSecond / (diesel ? 18.0 : 12.0) * 1.0e6;
        config.injection.injectorFlowMgPerSecond = derive("injector_flow_mg_s", "Injector flow", "mg/s",
            std::max(config.injection.injectorFlowMgPerSecond, fuelMgPerSecond / countD / 0.8),
            "the part's, or the full-load fuel at 80 % duty if more");
        if (diesel) {
            const auto scale = litres / countD / 0.492 * boost / 2.3;
            for (auto& sample : config.injection.fullLoadFuelLimit) {
                sample.milligramsPerCycle *= scale;
                sample.rpm *= config.redlineRpm / 5'000.0;
            }
            auto& calibration = config.combustionCalibration;
            calibration.wallHeatTransferCoefficientWPerK = 0.58;
            calibration.compressionIgnitionDelayScale = 3.0;
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
                   : "the family template stretched to the redline (not calibrated)", false });

        // Two-stroke ports and chamber.
        if (twoStroke) {
            const auto sport = spec.use == EngineUse::sportMotorcycle || spec.use == EngineUse::sportCar;
            config.twoStroke.exhaustPortOpenDegrees = derive("exhaust_port_open_deg", "Exhaust port opens", "deg ATDC",
                sport ? 82.0 : 92.0, sport ? "a sport port (RD350LC: 82)" : "a road port");
            config.twoStroke.transferPortOpenDegrees = derive("transfer_port_open_deg", "Transfer ports open",
                "deg ATDC", sport ? 116.0 : 118.0, "the use's");
            config.twoStroke.crankcaseCompressionRatio = derive("crankcase_compression_ratio",
                "Crankcase compression", "", 1.35, "1.25-1.5 on road and race engines");
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
            config = withTurbochargerCount(std::move(config), spec.turbochargerCount);
        } else if (spec.induction == Induction::supercharged) {
            auto& charger = config.forcedInduction;
            charger.enabled = true;
            charger.type = ForcedInductionType::supercharger;
            charger.pressureRatio = boost;
            charger.compressorEfficiency = 0.70;
            charger.chargeTemperatureRiseC = 48.0;
            const auto centrifugal = spec.use == EngineUse::aircraft;
            charger.fullBoostRpm = (centrifugal ? 0.6 : 0.3) * config.redlineRpm;
            if (centrifugal) {
                charger.compressorBladeCount = 12;
                charger.superchargerDriveRatio = 8.1;
            } else {
                charger.superchargerLobeCount = 4;
                charger.superchargerDriveRatio = 2.2;
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
