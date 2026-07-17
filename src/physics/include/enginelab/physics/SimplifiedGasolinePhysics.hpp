#pragma once
#include <enginelab/physics/IPhysicsModel.hpp>
namespace enginelab {
/**
 * Perceptually calibrated mean-value gasoline model; explicitly not an
 * engineering solver.
 *
 * IMPORTANT for maintainers (this file misleads on a first read):
 * `indicatedTorqueNm` and the other torque/pressure fields it returns do NOT
 * drive the crankshaft. The engine's motion is integrated from the
 * per-cylinder, per-substep chamber pressure resolved by the 0-D gas solver
 * (`ConservativeGasSystem`) inside `EngineSimulator` (`gasIndicatedTorque`).
 * The values here are telemetry/legacy. What this model *does* still feed:
 *   - `combustionQuality` (afr x timing efficiency) -> exhaust audio amplitude;
 *   - `heatOutput` -> cylinder wall temperature -> wall heat transfer;
 *   - `airMassMgPerCycle`, `misfireProbability` -> display / event-gen fallback.
 * So retuning the thermodynamics here changes audio and thermal feel, never the
 * dyno torque. See docs/physics-audit.md ("Constat d'architecture").
 */
class SimplifiedGasolinePhysics final : public IPhysicsModel {
public:
    [[nodiscard]] CombustionResult evaluateCombustion(
        const EngineConfig&, const EngineState&, const EngineControls&,
        const EcuCommand&, double exhaustBackPressureKpa) const noexcept override;
};
} // namespace enginelab
