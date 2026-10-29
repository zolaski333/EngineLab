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
 * mirrored. A carburettor whose venturi is its bore's share (no authored
 * venturiDiameterMm) keeps its mixture when its bore changes: the main jet
 * meters at the venturi's depression, so its flow number scales with the
 * throat's area, by the square of the bore. Returns an empty string on success, or why the edit was refused
 * (the configuration is then unchanged): no such path, or a size out of
 * range. The caller still validates the whole configuration.
 */
[[nodiscard]] std::string resizeIntakePart(EngineConfig& config, std::uint32_t pathId, IntakePart part,
                                           std::size_t cylinderIndex, double first, double second = 0.0);

/**
 * The hole of a jet stamped with flow number `flowNumber` (a Mikuni main's:
 * cc/min at one metre of head), in hundredths of a millimetre -- the size
 * Dellorto and Keihin stamp -- through an orifice of discharge coefficient
 * `dischargeCoefficient` (CarburettorConfig::jetDischargeCoefficient). At
 * 0.80, a 500 is a 173 hole. Zero for a jet that flows nothing.
 */
[[nodiscard]] double jetHoleHundredthsMm(double flowNumber, double dischargeCoefficient) noexcept;
/** The flow number of a jet whose hole is `holeHundredthsMm`: the inverse of
 *  jetHoleHundredthsMm(). */
[[nodiscard]] double jetFlowNumberForHole(double holeHundredthsMm, double dischargeCoefficient) noexcept;

/** What the 3-D view's inspector tunes on a carburettor: its jets as stamped
 *  flow numbers, the needle clip (1 richest to 5), the pilot air screw's
 *  turns out and the idle stop screw's turns in (CarburettorConfig). */
struct CarburettorTune final {
    double mainJetFlowNumber { 0.0 };
    double pilotJetFlowNumber { 0.0 };
    double needleClipPosition { 0.0 };
    double airScrewTurns { 0.0 };
    /** The idle stop screw's turns in (CarburettorConfig::idleScrewTurns). */
    double idleScrewTurns { carburettorNominalIdleScrewTurns };
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
