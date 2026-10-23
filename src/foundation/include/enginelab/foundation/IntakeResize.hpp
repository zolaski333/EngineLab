#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace enginelab {

/** The intake parts the 3-D view's inspector resizes. */
enum class IntakePart : std::uint8_t {
    /** A runner: its length and its diameter at the valve, in millimetres. */
    runner,
    /** A plenum: its volume, in litres. */
    plenum,
    /** A throttle body: its bore, in millimetres. */
    throttle,
};

struct IntakePartSize final {
    /** Runner length (mm), plenum volume (L) or throttle bore (mm). */
    double first { 0.0 };
    /** Runner diameter (mm); zero for the other parts. */
    double second { 0.0 };
    /** The runner size belongs to the path: every runner on it changes. */
    bool sharedByRunners { false };
};

/**
 * The size of a part of intake path `pathId`, or nothing when the engine has
 * no such path. `cylinderIndex` picks the runner, which is the cylinder's own
 * when it carries one (CylinderConfig::intakeRunnerLengthMm), else the path's.
 */
[[nodiscard]] std::optional<IntakePartSize> intakePartSize(const EngineConfig& config, std::uint32_t pathId,
                                                           IntakePart part, std::size_t cylinderIndex = 0);

/**
 * Gives that part `first` and `second` (see IntakePartSize) in place, where
 * intakePartSize() reads them, so a single path keeps the engine-wide intake
 * mirrored. Returns an empty string on success, or why the edit was refused
 * (the configuration is then unchanged): no such path, or a size out of
 * range. The caller still validates the whole configuration.
 */
[[nodiscard]] std::string resizeIntakePart(EngineConfig& config, std::uint32_t pathId, IntakePart part,
                                           std::size_t cylinderIndex, double first, double second = 0.0);

/** What the 3-D view's inspector tunes on a carburettor: its jets as stamped
 *  flow numbers, the needle clip (1 richest to 5) and the pilot air screw's
 *  turns out (CarburettorConfig). */
struct CarburettorTune final {
    double mainJetFlowNumber { 0.0 };
    double pilotJetFlowNumber { 0.0 };
    double needleClipPosition { 0.0 };
    double airScrewTurns { 0.0 };
};

/** The tune of intake path `pathId`'s carburettor, or nothing when the path
 *  does not exist or meters with an injector. */
[[nodiscard]] std::optional<CarburettorTune> carburettorTune(const EngineConfig& config, std::uint32_t pathId);

/**
 * Gives intake path `pathId`'s carburettor `tune` in place, mirrored like
 * resizeIntakePart(). Returns an empty string on success, or why the edit was
 * refused (the configuration is then unchanged): no such path, no
 * carburettor on it, or a value out of range (main jet 50-1,000, pilot jet
 * 10-150, clip 1-5, air screw 0-5 turns).
 */
[[nodiscard]] std::string tuneCarburettor(EngineConfig& config, std::uint32_t pathId, const CarburettorTune& tune);

} // namespace enginelab
