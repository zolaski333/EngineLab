#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

#include <span>

namespace enginelab {

/** Build the neutral editable graph shown when a scalar exhaust path is opened.
 *
 * The conversion preserves authored geometry but deliberately does not invent
 * the editor's component defaults. In particular, a scalar expansion chamber
 * has unity acoustic gain and no additional graph restriction: the scalar
 * `mufflerRestriction` field is not an active authored-network component loss.
 *
 * Its routes are the solver's own (audit EXHAUST-02): each primary at its
 * cylinder's own length when `cylinders` carries one, the collector's
 * 120 mm trunk, the straight body the solver puts where no chamber is
 * configured (a pipe: no wider than its pipes, it scatters nothing) and
 * the 180 mm tailpipe. Without `cylinders` every primary takes the path's.
 */
[[nodiscard]] ExhaustNetworkConfig makeEditableExhaustNetwork(
    const ExhaustPathConfig& path, std::span<const CylinderConfig> cylinders = {});

/** The id of the mid pipe in that graph, or 0 when the path has none: the one
 *  non-primary pipe a scalar field sizes. */
[[nodiscard]] std::uint32_t editableMidpipeId(const ExhaustPathConfig& path);

} // namespace enginelab
