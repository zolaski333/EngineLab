#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

#include <cstdint>
#include <optional>
#include <vector>

namespace enginelab {

/** Where one opening radiates from, in the acoustic frame of
 * AcousticObserverConfig (metres, +Z up), and the unit direction out of it. */
struct AcousticSourcePlacement final {
    /** Index in EngineConfig::exhaustPaths or intakePaths (0 for an engine
     * without paths). */
    std::uint32_t pathIndex {};
    /** An exhaust outlet's component id in its path's authored network; 0 for
     * the outlet of a path without one. Unused for an intake mouth. */
    std::uint32_t componentId {};
    AcousticPoint3M positionM {};
    AcousticPoint3M axis { 0.0, 1.0, 0.0 };
};

/** Where the drawn engine puts its sound sources (render::ListenerFrame::
 * sources): each exhaust outlet's tip, each intake path's mouth, and the
 * turbocharger or supercharger. An opening missing here keeps its authored
 * place. */
struct AcousticSourcePlacements final {
    std::vector<AcousticSourcePlacement> exhaustOutlets;
    std::vector<AcousticSourcePlacement> intakeMouths;
    std::optional<AcousticPoint3M> forcedInductionM;
};

} // namespace enginelab
