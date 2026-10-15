#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

#include <cstdint>
#include <optional>
#include <string>

namespace enginelab {

/** What the new engine's turbocharging becomes. */
enum class Aspiration : std::uint8_t {
    /** The base engine's own: none, its supercharger or its turbochargers. */
    keep,
    /** No forced induction. */
    natural,
    /** A turbocharger sized for the new engine (sizedTurbocharger()), shared
     *  over `EngineRecipe::turbochargerCount` identical units. */
    turbocharged,
};

/**
 * A new engine described from a base one. Zero keeps the base's value.
 * The layout, cylinder count, firing order, heads, camshafts, exhaust,
 * gearbox and vehicle are the base's: the builder starts from an engine that
 * runs and changes what the recipe names, with what has to follow it.
 */
struct EngineRecipe final {
    std::string name;
    double boreMm { 0.0 };
    double strokeMm { 0.0 };
    double compressionRatio { 0.0 };
    double redlineRpm { 0.0 };
    Aspiration aspiration { Aspiration::keep };
    std::uint32_t turbochargerCount { 1 };
};

struct BuiltEngine final {
    std::optional<EngineConfig> config;
    /** Why the recipe was refused, when `config` is empty. */
    std::string error;
};

/**
 * The base engine with the recipe applied, normalised and validated.
 *
 * Bore and stroke go through resizeCylinders() (crank throws follow; an
 * engine authored by chamber geometry gets its head chamber resized to keep
 * the compression ratio asked for). Pistons scale with the cube of the bore
 * and rods with its square, as geometrically similar parts.
 *
 * The redline moves the rev limiter by as much, keeping the base's margin,
 * and stretches the ignition curve's engine speeds (and a diesel's
 * full-load fuel limit's) by the same ratio.
 *
 * What feeds the air keeps the base's proportions to the new air flow
 * (displacement x redline x boost ratio): injector flow by the ratio, the
 * throttle bores by its square root (same air speed), plenums by the
 * displacement ratio, a diesel's full-load fuel per cycle and the clutch
 * capacity by the torque ratio (cylinder displacement x boost). The exhaust
 * is left as the base's: its sizes are the sound, edited on purpose.
 */
[[nodiscard]] BuiltEngine buildEngine(const EngineConfig& base, const EngineRecipe& recipe);

} // namespace enginelab
