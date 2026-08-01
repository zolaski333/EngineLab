#pragma once

#include <enginelab/foundation/EngineTypes.hpp>
#include <enginelab/physics/ConservativeGasSystem.hpp>

namespace enginelab {

struct FuelInjectionState final {
    double liquidFilmMoles { 0.0 };
    double directLiquidSprayMoles { 0.0 };
    double directDispersingVapourMoles { 0.0 };
};

struct FuelInjectionResult final {
    double meteredMoles { 0.0 };
    /** Fraction of this call for which the injector was physically open.
     *
     * A pulse can consume only part of a solver sub-step when its remaining
     * command is below the flow capacity. Counting every non-zero pulse as a
     * full sub-step overstates duty cycle; this value preserves that final
     * fractional opening without exposing the model's internal capacity.
     */
    double openFraction { 0.0 };
    double vaporisedMoles { 0.0 };
    double entrainedMoles { 0.0 };
    double chargeCoolingJoules { 0.0 };
    double liquidSprayMoles { 0.0 };
    double dispersingVapourMoles { 0.0 };
};

/** Pressure-aware delivery with port wall film or a two-stage direct spray.
 *
 * Direct-injected fuel remains in explicit liquid and vapor-cloud inventories
 * until vaporisation and turbulent entrainment make it available to the gas
 * chemistry. This keeps fuel mass conservative without pretending that a rail
 * pulse is an instantly homogeneous chamber mixture.
 */
class FuelInjectionModel final {
public:
    [[nodiscard]] static FuelInjectionResult deliver(const InjectionConfig& injection,
                                                       const FuelConfig& fuel,
                                                       FuelInjectionState& state,
                                                       GasCell& target,
                                                       double commandedMoles,
                                                       double dtSeconds) noexcept;

    /**
     * Advance the per-cylinder closed-loop fuel trim once per engine cycle.
     * The controller bandwidth follows both cycle time and port-film delay so
     * slow manifold injection cannot oscillate like a delay-free DI system.
     */
    [[nodiscard]] static double updateClosedLoopTrim(const InjectionConfig& injection,
                                                     double rpm,
                                                     double measuredAirFuelRatio,
                                                     double targetAirFuelRatio,
                                                     double currentTrim) noexcept;
};

} // namespace enginelab
