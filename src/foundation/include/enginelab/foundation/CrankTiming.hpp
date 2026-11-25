#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

#include <string>
#include <vector>

namespace enginelab {

/**
 * Gives each cylinder (in `config.cylinders` order) its top dead centre at
 * `tdcDegrees` of the cycle, in place: the crank the edit describes. Each
 * crank pin points at its cylinder's TDC less its bank's angle; cylinders
 * that shared a pin keep sharing it while their pins still point the same
 * way, and a cylinder whose pin moved gets a pin of its own. The firing order
 * follows the new TDCs. Returns an empty string on success, or why the edit
 * was refused (the configuration is then unchanged): a rotary or a radial
 * (their timing follows their rotors and their one throw), a count that is
 * not the cylinders', or an angle outside the cycle. The caller still
 * validates the whole configuration.
 */
[[nodiscard]] std::string retimeCrank(EngineConfig& config, const std::vector<double>& tdcDegrees);

/**
 * Opens or closes a V's two banks to `degrees` between them, about the same
 * centre line, in place. The crank stays the one it was: each cylinder's TDC
 * moves with its bank, so the firing intervals change as on a real block of
 * another angle (retimeCrank then sets them). Returns an empty string on
 * success, or why the edit was refused (unchanged): not a V of two banks, or
 * an angle outside (0, 180] degrees.
 */
[[nodiscard]] std::string setVBankAngle(EngineConfig& config, double degrees);

/** The cylinder ids in firing order for these TDCs (`config.cylinders`
 *  order), earliest in the cycle first; a tie keeps the cylinders' order. */
[[nodiscard]] std::vector<std::uint32_t> firingOrderFor(const EngineConfig& config,
                                                        const std::vector<double>& tdcDegrees);

} // namespace enginelab
