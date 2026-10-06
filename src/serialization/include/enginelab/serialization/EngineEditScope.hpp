#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

namespace enginelab {

/**
 * What an edit of a running engine changes, by the groups a running engine
 * can take without a restart. Each flag says the group differs; `other` says
 * something outside every group differs, which needs a restart.
 */
struct EngineEditScope final {
    /** config.exhaust and config.exhaustPaths (EngineRuntime::applyLiveExhaust). */
    bool exhaust { false };
    /** Bore, stroke, deck height and compression ratio of the same cylinders,
     *  the throws of the same journals (applyLiveCylinderResize). */
    bool cylinders { false };
    /** config.injection and config.forcedInduction (applyLiveSettings). */
    bool settings { false };
    bool other { false };

    [[nodiscard]] bool any() const noexcept { return exhaust || cylinders || settings || other; }
};

/**
 * Compares `edited` with `running` through their JSON encodings, both
 * normalised: a field the JSON does not carry is not an edit. A configuration
 * that does not normalise counts as `other`.
 */
[[nodiscard]] EngineEditScope engineEditScope(const EngineConfig& running,
                                              const EngineConfig& edited);

} // namespace enginelab
