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
 *  vehicle. A truck's four-stroke also gets a compression-release engine
 *  brake (EngineBrakeConfig), the only use that does. */
enum class EngineUse : std::uint8_t {
    roadCar, sportCar, roadMotorcycle, sportMotorcycle, cruiserMotorcycle, aircraft, truck
};

/** `supercharged` fits the use's usual machine: a Roots, a centrifugal one on
 *  an aircraft. `centrifugalSupercharged` fits a centrifugal one on any use. */
enum class Induction : std::uint8_t { natural, turbocharged, supercharged, centrifugalSupercharged };

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

/** How a four-stroke's exhaust leaves the engine. A two-stroke has one
 *  expansion chamber per cylinder and an aircraft engine open stacks, whatever
 *  this says. */
enum class ExhaustSystem : std::uint8_t {
    /** The family's: one pipe per bank on a V or a flat of four cylinders or
     *  more, else one pipe. */
    automatic,
    /** Every cylinder into one pipe and one silencer (a V's banks meet in a
     *  Y). */
    single,
    /** Two pipes and two silencers: one per side of a V, a flat or a W; an
     *  inline engine's collector splits in two. */
    dual,
    /** Dual, the banks' two pipes joined by a balance pipe (an H-pipe). */
    dualHPipe,
    /** Dual, the banks' two pipes crossing in an X. */
    dualXPipe,
};

/** The silencer at the end of each pipe. */
enum class ExhaustSilencer : std::uint8_t {
    /** The family's box. */
    automatic,
    /** A large, long, heavily packed road box. */
    packedBox,
    /** A short straight-through sports can, lightly packed. */
    straightThrough,
    /** None: an open pipe. */
    none,
};

/** What a four-stroke petrol engine's fuel comes from. A diesel injects
 *  directly into the cylinder whatever this says. */
enum class FuelSystem : std::uint8_t {
    /** The family's: port injection (continuous on an aircraft engine), a
     *  carburettor on a two-stroke. */
    automatic,
    /** An injector in each intake port. */
    portInjection,
    /** An injector in each cylinder. */
    directInjection,
    /** A carburettor on each throttle bore, metering from the air: a slide
     *  one on a two-stroke, a fixed venturi on a four-stroke. */
    carburettor,
};

/** What the air enters through. */
enum class IntakeStyle : std::uint8_t {
    /** The family's: an airbox on a car or a motorcycle, the bare throttle on
     *  an aircraft engine. */
    automatic,
    /** A closed airbox ahead of the throttle, its filter inside. */
    airbox,
    /** A filter on the throttle's mouth with no box around it. */
    podFilter,
    /** Open velocity stacks (trumpets): the bellmouths draw from the air. */
    velocityStacks,
};

/** How many throttles, and where. */
enum class ThrottleLayout : std::uint8_t {
    /** The family's: one body on a car, one per cylinder ahead of the
     *  plenum on a sport motorcycle; a two-stroke's carburettor feeds its
     *  own crankcase. */
    automatic,
    /** One throttle body for the whole engine, ahead of the plenum. */
    single,
    /** One plate in each cylinder's runner (individual throttle bodies), a
     *  four-stroke spark engine's. */
    individual,
    /** One throttle body per cylinder (per rotor) side by side ahead of the
     *  plenum, as a sport motorcycle's: the plates' area of individual
     *  throttles, the plenum's response. */
    plenumBodies,
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
    /** A two-stroke's: where its carburettor delivers, and whether a
     *  cylinder port has a reed (a crankcase intake always has one). */
    TwoStrokeIntake twoStrokeIntake { TwoStrokeIntake::crankcaseReed };
    bool twoStrokeIntakeReed { true };
    ExhaustSystem exhaustSystem { ExhaustSystem::automatic };
    ExhaustSilencer exhaustSilencer { ExhaustSilencer::automatic };
    FuelSystem fuelSystem { FuelSystem::automatic };
    IntakeStyle intakeStyle { IntakeStyle::automatic };
    ThrottleLayout throttles { ThrottleLayout::automatic };
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

/** The fuel system `config` has: a carburettor on any intake path, else its
 *  injection's mode. */
[[nodiscard]] FuelSystem fuelSystemOf(const EngineConfig& config) noexcept;
/** What its throttle breathes from: an airbox, a pod filter, or an open
 *  mouth (trumpets). */
[[nodiscard]] IntakeStyle intakeStyleOf(const EngineConfig& config) noexcept;

/**
 * `config` with another fuel system, sized as the designer sizes it for this
 * engine's displacement, redline and boost: the injection part and window,
 * the injectors' flow, or a carburettor's venturi and jets on every intake
 * path, for the fuel in the tank. Everything else is kept, the ignition map
 * and its knock limit included. A diesel takes direct injection only. Never
 * throws; a refusal is in `error`, the settled values in `values`.
 */
[[nodiscard]] DesignedEngine withFuelSystem(const EngineConfig& config, FuelSystem system,
                                            const PartsLibrary& parts);
[[nodiscard]] DesignedEngine withFuelSystem(const EngineConfig& config, FuelSystem system,
                                            const std::filesystem::path& catalogRoot);

/**
 * `config` breathing from another intake: an airbox sized as the designer
 * sizes it (a motorcycle's, under the tank, when the vehicle weighs under
 * 600 kg; a car's, with its snorkel, otherwise), a pod filter or open
 * trumpets, on every intake path. Never throws.
 */
[[nodiscard]] DesignedEngine withIntakeStyle(const EngineConfig& config, IntakeStyle style);

/** single, plenumBodies or individual: what `config` has (a single body, or
 *  several ahead of the plenum). */
[[nodiscard]] ThrottleLayout throttleLayoutOf(const EngineConfig& config) noexcept;
/**
 * `config` with one throttle body for the engine, one body a cylinder ahead
 * of the plenum, or one plate a cylinder in each runner, on every intake path: the plates' total area kept, a
 * carburettor resized for its bores. A two-stroke, a diesel or a rotary is
 * refused individual throttles. Never throws; a refusal is in `error`.
 */
[[nodiscard]] DesignedEngine withThrottleLayout(const EngineConfig& config, ThrottleLayout layout);

[[nodiscard]] const char* engineUseName(EngineUse) noexcept;
[[nodiscard]] std::optional<EngineUse> parseEngineUse(std::string_view) noexcept;
[[nodiscard]] const char* crankPinsName(CrankPins) noexcept;
[[nodiscard]] std::optional<CrankPins> parseCrankPins(std::string_view) noexcept;
[[nodiscard]] const char* exhaustSystemName(ExhaustSystem) noexcept;
[[nodiscard]] std::optional<ExhaustSystem> parseExhaustSystem(std::string_view) noexcept;
[[nodiscard]] const char* exhaustSilencerName(ExhaustSilencer) noexcept;
[[nodiscard]] std::optional<ExhaustSilencer> parseExhaustSilencer(std::string_view) noexcept;
[[nodiscard]] const char* fuelSystemName(FuelSystem) noexcept;
[[nodiscard]] std::optional<FuelSystem> parseFuelSystem(std::string_view) noexcept;
[[nodiscard]] const char* intakeStyleName(IntakeStyle) noexcept;
[[nodiscard]] std::optional<IntakeStyle> parseIntakeStyle(std::string_view) noexcept;
[[nodiscard]] const char* throttleLayoutName(ThrottleLayout) noexcept;
[[nodiscard]] std::optional<ThrottleLayout> parseThrottleLayout(std::string_view) noexcept;

} // namespace enginelab
