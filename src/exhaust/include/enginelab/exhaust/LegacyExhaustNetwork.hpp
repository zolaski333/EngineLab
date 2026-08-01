#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

namespace enginelab {

/** Build the neutral editable graph shown when a scalar exhaust path is opened.
 *
 * The conversion preserves authored geometry but deliberately does not invent
 * the editor's component defaults. In particular, a scalar expansion chamber
 * has unity acoustic gain and no additional graph restriction: the scalar
 * `mufflerRestriction` field is not an active authored-network component loss.
 */
[[nodiscard]] ExhaustNetworkConfig makeEditableExhaustNetwork(
    const ExhaustPathConfig& path);

} // namespace enginelab
