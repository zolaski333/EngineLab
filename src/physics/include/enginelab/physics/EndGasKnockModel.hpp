#pragma once

namespace enginelab {

struct EndGasKnockState final {
    double inductionIntegral { 0.0 };
    double filteredLevel { 0.0 };
    bool autoIgnitedThisCycle { false };
};

struct EndGasConditions final {
    double pressureBar { 1.0 };
    double temperatureK { 293.15 };
    double equivalenceRatio { 1.0 };
    double burnedFraction { 0.0 };
    double octaneRating { 95.0 };
    bool combustionActive { false };
};

struct EndGasKnockResult final {
    double level { 0.0 };
    double autoIgnitedFuelFraction { 0.0 };
    double autoIgnitionDelaySeconds { 1.0 };
    bool autoIgnited { false };
};

/** Livengood-Wu end-gas auto-ignition integral driven by actual chamber P/T. */
class EndGasKnockModel final {
public:
    [[nodiscard]] static EndGasKnockResult advance(EndGasKnockState& state,
                                                    const EndGasConditions& conditions,
                                                    double dtSeconds) noexcept;
    [[nodiscard]] static double autoIgnitionDelaySeconds(const EndGasConditions& conditions) noexcept;
};

} // namespace enginelab
