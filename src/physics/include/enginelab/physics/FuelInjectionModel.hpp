#pragma once

#include <enginelab/foundation/EngineTypes.hpp>
#include <enginelab/physics/ConservativeGasSystem.hpp>

namespace enginelab {

struct FuelInjectionState final {
    double liquidFilmMoles { 0.0 };
};

struct FuelInjectionResult final {
    double meteredMoles { 0.0 };
    double vaporisedMoles { 0.0 };
    double chargeCoolingJoules { 0.0 };
};

/** Pressure-aware injector delivery with port wall-film and DI charge cooling. */
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
