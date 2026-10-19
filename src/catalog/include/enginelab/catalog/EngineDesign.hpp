#pragma once

#include <enginelab/catalog/EngineCatalog.hpp>

#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace enginelab {

/** What an engine is built for. It sets the values a designer picks for that
 *  family: piston speed at the redline, cams, idle, silencer, gearbox and
 *  vehicle. */
enum class EngineUse : std::uint8_t {
    roadCar, sportCar, roadMotorcycle, sportMotorcycle, cruiserMotorcycle, aircraft
};

enum class Induction : std::uint8_t { natural, turbocharged, supercharged };

/** How the cylinders of one row of a V or a W meet the crank. Inline, flat
 *  and radial engines are not concerned. */
enum class CrankPins : std::uint8_t {
    /** Shared when that fires evenly, or on four cylinders or fewer (a 90 deg
     *  V-twin's 270-450, as the real ones); otherwise split pins, offset the
     *  least that fires evenly (a 90 deg V6's 30 deg); otherwise one pin per
     *  cylinder. */
    automatic,
    /** One pin per row, firing as evenly as that allows. */
    shared,
    /** One pin per cylinder: the best balanced even-firing crank. */
    separate
};

/**
 * The short description an engine is designed from, with no base engine.
 * A zero or empty field is derived. Bore and stroke, or the displacement, are
 * required.
 */
struct EngineSpec final {
    std::string name;
    EngineCycle cycle { EngineCycle::fourStroke };
    FuelType fuel { FuelType::gasoline };
    EngineUse use { EngineUse::roadCar };
    EngineLayout layout { EngineLayout::inlineLayout };
    std::uint32_t cylinderCount { 4 };
    /** Banks of a custom layout (a W: 3 or 4). Inline 1, V and flat 2, radial
     *  one per cylinder, whatever this says. */
    std::uint32_t bankCount { 0 };
    /** Angle between adjacent banks: a V's or a W's. Flat is 180, radial
     *  360 / N, whatever this says. */
    double bankAngleDegrees { 0.0 };
    /** A custom layout's banks, each one's angle from vertical (a W16:
     *  -52.5, -37.5, 37.5, 52.5); when given, bankCount and bankAngleDegrees
     *  are not used. */
    std::vector<double> bankAnglesDegrees;
    double boreMm { 0.0 };
    double strokeMm { 0.0 };
    /** Used when bore and stroke are not given, with the bore/stroke ratio
     *  (derived from the use, or the `bore_stroke_ratio` override). */
    double displacementLitres { 0.0 };
    Induction induction { Induction::natural };
    /** Absolute boost pressure ratio; zero derives it from the use. */
    double boostPressureRatio { 0.0 };
    std::uint32_t turbochargerCount { 1 };
    /** Cylinder ids in firing order; empty picks the best balanced even-firing
     *  order (evenFiringOrders()). A radial's follows from its single throw. */
    std::vector<std::uint32_t> firingOrder;
    /** Top dead centre of each cylinder (id order), crank degrees, for an
     *  uneven-firing crank such as a 270 deg twin; empty fires evenly. */
    std::vector<double> crankOffsetsDegrees;
    /** A V's or a W's crank pins, when crankOffsetsDegrees is empty. */
    CrankPins crankPins { CrankPins::automatic };
    /** Values set by hand, by DesignValue key. They win over the derived ones,
     *  and what is derived from them follows. An unknown key is an error. */
    std::map<std::string, double> overrides;
};

/** One value the design settled, how, and whether it was set by hand. */
struct DesignValue final {
    std::string key;
    std::string label;
    std::string unit;
    double value { 0.0 };
    bool overridden { false };
    /** The rule that derived it (kept when overridden: what it would be). */
    std::string rule;
    /** False for what is shown but cannot be set by key (an estimate, a
     *  curve). */
    bool settable { true };
};

struct DesignedEngine final {
    std::optional<EngineConfig> config;
    /** Every derived or overridden value, in the order they were settled. */
    std::vector<DesignValue> values;
    /** Why the spec was refused, when `config` is empty. */
    std::string error;
};

/**
 * Designs a complete engine from `spec`. Every value is either set by hand or
 * derived by a rule fitted to the real engines the catalogue was authored from
 * (their bore, stroke, rods, pistons, intake and exhaust sizes, cams and
 * speeds), the silencer, gearbox and vehicle coming from the family parts of
 * `parts`. The result is normalised and validated; it is not calibrated by
 * running it (ignition is a template stretched to the redline).
 */
[[nodiscard]] DesignedEngine designEngine(const EngineSpec& spec, const PartsLibrary& parts);

/**
 * designEngine() with the parts of the catalogue at `catalogRoot`, and the
 * catalogue's default voicing (`voicing/default.yaml`): a designed engine has
 * no voicing of its own. Never throws; a failure is in `error`.
 */
[[nodiscard]] DesignedEngine designCatalogueEngine(const EngineSpec& spec, const std::filesystem::path& catalogRoot);

[[nodiscard]] const char* engineUseName(EngineUse) noexcept;
[[nodiscard]] std::optional<EngineUse> parseEngineUse(std::string_view) noexcept;
[[nodiscard]] const char* crankPinsName(CrankPins) noexcept;
[[nodiscard]] std::optional<CrankPins> parseCrankPins(std::string_view) noexcept;

} // namespace enginelab
