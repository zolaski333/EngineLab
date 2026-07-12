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
};

} // namespace enginelab
