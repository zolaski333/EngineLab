#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

namespace enginelab {

/**
 * A turbocharger sized for an engine that has none, from its displacement,
 * redline, compression ratio and fuel.
 *
 * The scaling laws are fitted to the four catalogue turbochargers (2JZ, EJ25,
 * I5, TDI), whose sizes were set against their makers' torque and power:
 *
 * - turbine flow area 0.028 mm^2 per litre and per redline rpm (the four lie
 *   within 10 % of it): the exhaust mass flow scales with both;
 * - full boost at 45 % of the redline (38-50 %);
 * - compressor inducer 7.8 mm x sqrt(litres x krpm x pressure ratio), the
 *   square root of the air mass flow (within 5 %); turbine exducer 0.9 x it;
 * - rotor inertia 4.75e-13 kg m^2 x inducer(mm)^5, a wheel's mass times its
 *   radius squared (within 40 %: a family estimate);
 * - wastegate port 1.5 x and blow-off valve 0.65 x the turbine area (2JZ).
 *
 * The pressure ratio follows the compression ratio, as on a street
 * conversion: a high-compression engine runs a low boost on pump fuel
 * (about 0.5 bar from 10.5:1), a low-compression one more. Diesels 2.3.
 * The ignition map is the engine's own: at boost a high-compression engine
 * may knock, and the knock counter shows it.
 */
[[nodiscard]] ForcedInductionConfig sizedTurbocharger(const EngineConfig&);

/** The engine with sizedTurbocharger() fitted, everything else unchanged.
 *  The simulator puts the turbine on each exhaust path's collector (or at
 *  the outlets when a path has none); it takes a restart. */
[[nodiscard]] EngineConfig withTurbocharger(EngineConfig);

/** The engine without forced induction. */
[[nodiscard]] EngineConfig withoutForcedInduction(EngineConfig);

/** The turbo engine's air shared over `count` identical turbochargers
 *  (clamped to 1..maximumTurbochargers): each unit's design flow, turbine and
 *  wastegate areas, rotor inertia and bearing loss scale by old / new count,
 *  so the set has the old one's total size and shaft speed. The blow-off
 *  valve stays, on the common charge. A path naming a turbocharger that no
 *  longer exists goes back to the default (its index modulo the count). It
 *  takes a restart. */
[[nodiscard]] EngineConfig withTurbochargerCount(EngineConfig, std::uint32_t count);

} // namespace enginelab
