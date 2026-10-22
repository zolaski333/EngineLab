#pragma once

namespace enginelab {

/**
 * The end gas of one cylinder: the unburned charge, followed from the moment
 * the cylinder seals until it opens again.
 *
 * The chamber is one well-mixed cell, and its mean temperature is not the
 * end gas's: once the flame has started it mixes burned gas at 2,000-2,800 K
 * with an unburned charge near 800-1,000 K. So the unburned zone is carried
 * here as its own temperature, compressed isentropically by the chamber
 * pressure from the seal on, with the heat capacities of the unburned
 * mixture at its own temperature (Heywood, *Internal Combustion Engine
 * Fundamentals*, 9.2: the unburned zone of a two-zone model).
 */
struct EndGasKnockState final {
    double inductionIntegral { 0.0 };
    double filteredLevel { 0.0 };
    bool autoIgnitedThisCycle { false };
    /** True from the seal to the next opening; false resets the zone. */
    bool sealed { false };
    double unburnedTemperatureK { 0.0 };
    double lastPressureBar { 0.0 };
    /** Mole fractions of the unburned zone, frozen when the flame starts:
     * from then on the cell's own composition is the burned zone's too. */
    double fuelMoleFraction { 0.0 };
    double burnedMoleFraction { 0.0 };
    bool compositionFrozen { false };
    /** This cycle's hottest end gas, and the last completed cycle's. */
    double peakUnburnedTemperatureK { 0.0 };
    double lastCyclePeakUnburnedTemperatureK { 0.0 };
    /** The pressure the last knock event's end gas released, bar. */
    double lastKnockPressureRiseBar { 0.0 };
};

struct EndGasConditions final {
    double pressureBar { 1.0 };
    /** The chamber's mean temperature: the end gas's only before the flame. */
    double temperatureK { 293.15 };
    double equivalenceRatio { 1.0 };
    double burnedFraction { 0.0 };
    double octaneRating { 95.0 };
    /** Both valves (a two-stroke's ports) shut. */
    bool sealed { false };
    /** A flame is burning in this cylinder this cycle. */
    bool combustionActive { false };
    /** Live mole fractions of the chamber gas. */
    double fuelMoleFraction { 0.0 };
    double burnedMoleFraction { 0.0 };
    double chamberVolumeM3 { 0.0 };
    /** The heat the whole charge releases when burned, J. */
    double chargeFuelEnergyJoules { 0.0 };
    double heatCapacityRatio { 1.3 };
};

struct EndGasKnockResult final {
    double level { 0.0 };
    double autoIgnitedFuelFraction { 0.0 };
    double autoIgnitionDelaySeconds { 1.0 };
    double unburnedTemperatureK { 0.0 };
    /** This event's end-gas pressure rise, bar; zero without an event. */
    double knockPressureRiseBar { 0.0 };
    bool autoIgnited { false };
};

/**
 * Livengood-Wu end-gas auto-ignition integral, from ignition, over the
 * Douaud-Eyzat delay of the unburned zone. When it reaches one, the gas the
 * flame has not reached burns at once; the pressure that would add at
 * constant volume, (gamma - 1) x_u Q / V, is the knock's intensity.
 */
class EndGasKnockModel final {
public:
    [[nodiscard]] static EndGasKnockResult advance(EndGasKnockState& state,
                                                    const EndGasConditions& conditions,
                                                    double dtSeconds) noexcept;
    /** Douaud and Eyzat (SAE 780080), for the unburned zone's temperature. */
    [[nodiscard]] static double autoIgnitionDelaySeconds(double pressureBar,
                                                         double unburnedTemperatureK,
                                                         double octaneRating) noexcept;
    /** Molar cp of an unburned fuel-air-residual mixture, J/(mol K). */
    [[nodiscard]] static double unburnedMolarHeatCapacityCp(double temperatureK,
                                                            double fuelMoleFraction,
                                                            double burnedMoleFraction) noexcept;
    /** The end-gas pressure rise, bar, at which `level` reads 1: about the
     * heaviest conventional knock, whose oscillation reaches ~10 bar (a cycle
     * is called knocking from ~1 bar). The ECU hears knock from 3 bar of
     * rise, and the diagnostics call it severe from 13.5 (0.45). */
    static constexpr double fullScaleKnockPressureRiseBar = 30.0;
};

} // namespace enginelab
