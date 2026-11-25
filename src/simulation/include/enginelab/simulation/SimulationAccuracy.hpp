#pragma once

#include <enginelab/foundation/EngineTypes.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <cstddef>
#include <optional>
#include <string>
#include <string_view>

namespace enginelab {

/** How finely one run resolves the gas dynamics and the crank.
 *
 * `realtime` is production and changes nothing. Each finer profile halves
 * every resolution of the one before it -- exhaust and intake cells, exhaust
 * and intake coupling intervals, crank step -- so that a profile is a level
 * of one refinement ladder, not a set of values tuned to any result. The
 * finest also couples both networks every substep and halves the Courant
 * numbers. Finer profiles may run slower than real time. Each knob stays
 * overridable on its own after a profile is applied (one factor at a
 * time). */
enum class AccuracyProfile : unsigned char {
    realtime,
    balanced,
    engineering,
    highPrecision,
};

[[nodiscard]] std::string_view accuracyProfileName(AccuracyProfile) noexcept;
/** "realtime", "balanced", "engineering", "high-precision". */
[[nodiscard]] std::optional<AccuracyProfile> accuracyProfileFromName(std::string_view) noexcept;

/** What a profile asks for `config`. */
struct AccuracyResolution final {
    AccuracyProfile profile { AccuracyProfile::realtime };
    /** 1, 2, 4, 8: each resolution below is production's divided by it. */
    double refinement { 1.0 };
    double exhaustCellLengthM { 0.0 };
    /** The exhaust coupling's low-speed cap; ignored when every substep. */
    double exhaustCouplingSeconds { 0.0 };
    bool exhaustCouplingEverySubstep { false };
    double exhaustCourantNumber { 0.8 };
    std::size_t exhaustMaximumCellsPerDuct { 64 };
    std::size_t exhaustMaximumTotalCells { 1'024 };
    double crankDegreesPerStep { 0.0 };
    /** The crank step the ladder asked was finer than the solver's frequency
     * ceiling allows at the rev limit, and is held at that ceiling. */
    bool crankStepHeldBySolverCeiling { false };
    double solverMaximumFrequencyHz { 0.0 };
    double intakeCellLengthM { 0.0 };
    /** Zero: every substep. */
    double intakeCouplingSeconds { 0.0 };
    double intakeCourantNumber { 0.8 };
    std::size_t intakeMaximumCellsPerRunner { 12 };
};

/** The production values the ladder starts from. */
inline constexpr double productionExhaustCouplingSeconds = 125.0e-6;
inline constexpr double productionIntakeCellLengthM = 0.095;
inline constexpr double productionIntakeCouplingSeconds = 330.0e-6;
inline constexpr double productionCourantNumber = 0.8;
/** validateEngineConfig's ceiling on the mechanical substep rate. */
inline constexpr double solverFrequencyCeilingHz = 100'000.0;

[[nodiscard]] AccuracyResolution accuracyResolution(AccuracyProfile, const EngineConfig&) noexcept;

/** Writes `profile` into a run's options and its solver settings.
 * `realtime` writes nothing, so a realtime run is production bit for bit.
 * Call it before any single-knob override. */
AccuracyResolution applyAccuracyProfile(AccuracyProfile profile, EngineConfig& config,
                                        EngineSimulatorOptions& options);

/** One line on what a run resolves: the profile, the exhaust mesh it got
 * against the one it asked (and whether a ceiling coarsened it), the
 * shortest cell, the coupling, the crank step and the intake cell. The
 * resolutions are read back from the run's `layout` (EngineSimulator::
 * exhaustLayout), `options` and `config`, so a single-knob override applied
 * after the profile shows. */
[[nodiscard]] std::string accuracyReport(const AccuracyResolution& asked,
                                         const EngineSimulatorOptions& options,
                                         const EngineConfig& config,
                                         const gasdynamics::ExhaustNetworkLayout& layout);

} // namespace enginelab
