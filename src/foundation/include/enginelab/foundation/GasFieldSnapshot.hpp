#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace enginelab {

/** What one GasFieldElement stands for in the solver. */
enum class GasFieldElementKind : std::uint8_t {
    /** A finite-volume exhaust duct (one authored component). */
    exhaustDuct,
    /** A well-mixed exhaust junction (a merge, split or crossover). */
    exhaustJunction,
    /** The 1-D intake runner of one cylinder. */
    intakeRunner,
    /** The lumped intake plenum of one intake path. */
    intakePlenum,
};

/**
 * Gas state along one solver element, at most `maximumSamples` cells, in
 * gas-flow order (port to outlet for the exhaust, plenum to port for an intake
 * runner). Sample s sits at (s + 0.5) / sampleCount of the element's length.
 */
struct GasFieldElement final {
    static constexpr std::size_t maximumSamples = 16;

    GasFieldElementKind kind { GasFieldElementKind::exhaustDuct };
    /** Index of the exhaust or intake path in the configuration. */
    std::uint32_t pathIndex {};
    /** Authored exhaust component id; 0 for a path compiled from the
        geometry-only (schema v1) fields. */
    std::uint32_t componentId {};
    /** Solver node id: the cylinder id for a geometry-only primary. */
    std::uint32_t nodeId {};
    /** ExhaustNodeType of the solver node, as its integer value. */
    std::uint8_t nodeType {};
    /** Cylinder id of an intake runner. */
    std::uint32_t cylinderId {};
    std::uint8_t sampleCount {};
    std::array<float, maximumSamples> pressurePa {};
    std::array<float, maximumSamples> gasTemperatureK {};
    /** Duct wall temperature; 0 where the element has no wall state. */
    std::array<float, maximumSamples> wallTemperatureK {};
};

/**
 * The simulator's gas field at one crank angle, for the 3-D view.
 *
 * Drawing the field at the crank angle the view shows needs a snapshot taken
 * at that angle, not the latest one: the physics thread advances a whole
 * frame (1/240 s, 75 degrees at 3,000 rpm) at a time. The view asks for an
 * angle, the simulator captures when its crank crosses it between two
 * sub-steps, so in slow motion the picture is the most recent cycle at the
 * displayed angle. In real time the view takes the latest field instead.
 */
struct GasFieldSnapshot final {
    std::uint64_t sequence {};
    double crankAngleDegrees {};
    double simulationTimeSeconds {};
    double ambientPressurePa {};
    double ambientTemperatureK {};
    /** Afterfire heat release in the exhaust at the captured instant. */
    double afterfireHeatReleaseKw {};
    std::vector<GasFieldElement> elements;
};

/**
 * Pressure at one place of the gas field over the cycle, recorded at every
 * solver sub-step: the part inspector's oscilloscope. A snapshot holds one
 * instant; this holds the history, so the waves in a duct read as a trace.
 */
struct GasProbeTrace final {
    /** Crank bins over the cycle (2 degrees each over a four-stroke cycle). */
    static constexpr std::size_t bins = 360;

    std::uint64_t sequence {};
    /** Index in GasFieldSnapshot::elements, and the sample along it. */
    std::int32_t element { -1 };
    std::uint8_t sample {};
    double cycleDegrees { 720.0 };
    double ambientPressurePa {};
    /** Pressure in each crank bin, the latest pass of the crank (Pa); 0 where
        the crank has not passed since the probe was set. */
    std::array<float, bins> pressurePa {};
    /** Bin the crank is in. */
    std::int32_t latestBin { -1 };
};

} // namespace enginelab
