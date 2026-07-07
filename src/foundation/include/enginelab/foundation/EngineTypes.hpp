#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <optional>
#include <array>

namespace enginelab {

enum class EngineCycle : std::uint8_t { fourStroke, twoStroke };
enum class FuelType : std::uint8_t { gasoline, diesel };
enum class EngineLayout : std::uint8_t { inlineLayout, vLayout, flat, custom };
enum class RunningState : std::uint8_t {
    stopped, cranking, idling, running, unstable, knocking, overheating, damaged, destroyed
};

struct CylinderConfig final {
    std::uint32_t id { 1 };
    double boreMm { 86.0 };
    double strokeMm { 86.0 };
    double connectingRodMm { 143.0 };
    double pistonMassGrams { 420.0 };
    double compressionRatio { 10.5 };
    double ignitionOffsetDegrees { 0.0 };
    double efficiencyOffset { 0.0 };
    double crankOffsetDegrees { 0.0 };
};

struct CamshaftConfig final {
    double intakeDurationDegrees { 248.0 };
    double exhaustDurationDegrees { 244.0 };
    double intakeLiftMm { 10.2 };
    double exhaustLiftMm { 9.8 };
    double intakeCenterlineDegrees { 110.0 };
    double exhaustCenterlineDegrees { 112.0 };
};

struct ExhaustConfig final {
    double primaryLengthMm { 480.0 };
    double primaryDiameterMm { 42.0 };
    double collectorDiameterMm { 58.0 };
    double mufflerRestriction { 0.28 };
    double outletDiameterMm { 65.0 };
};

struct EngineConfig final {
    std::uint32_t schemaVersion { 1 };
    std::string name { "Untitled engine" };
    EngineCycle cycle { EngineCycle::fourStroke };
    FuelType fuel { FuelType::gasoline };
    EngineLayout layout { EngineLayout::inlineLayout };
    std::vector<CylinderConfig> cylinders;
    std::vector<std::uint32_t> firingOrder;
    double idleRpm { 850.0 };
    double redlineRpm { 7'200.0 };
    double rotatingInertiaKgM2 { 0.24 };
    double frictionCoefficient { 0.12 };
    double octaneRating { 98.0 };
    double ambientPressureKpa { 101.325 };
    double ambientTemperatureC { 22.0 };
    double coolingEfficiency { 1.0 };
    double plenumVolumeLitres { 3.0 };
    double throttleDiameterMm { 60.0 };
    double bankAngleDegrees { 0.0 };
    CamshaftConfig camshafts;
    ExhaustConfig exhaust;
};

struct EngineControls final {
    bool ignitionEnabled { false };
    bool starterEngaged { false };
    double throttle { 0.0 };
    double load { 0.0 };
};

struct CylinderState final {
    std::uint32_t id { 0 };
    double cyclePhaseDegrees { 0.0 };
    double pressureEstimateBar { 0.0 };
    double combustionPulse { 0.0 };
    double misfireProbability { 0.0 };
    double exhaustTemperatureC { 22.0 };
    bool combustionActive { false };
    bool misfiring { false };
};

struct EngineState final {
    double simulationTimeSeconds { 0.0 };
    double rpm { 0.0 };
    double crankAngleDegrees { 719.9 };
    double throttle { 0.0 };
    double load { 0.0 };
    double angularVelocityRadPerSecond { 0.0 };
    double indicatedTorqueNm { 0.0 };
    double frictionTorqueNm { 0.0 };
    double reciprocatingTorqueNm { 0.0 };
    double meanPistonSpeedMps { 0.0 };
    double loadTorqueNm { 0.0 };
    double starterTorqueNm { 0.0 };
    double netTorqueNm { 0.0 };
    double torqueNm { 0.0 };
    double cycleAveragedTorqueNm { 0.0 };
    double coolantTemperatureC { 22.0 };
    double oilTemperatureC { 22.0 };
    double exhaustTemperatureC { 22.0 };
    double knockLevel { 0.0 };
    double misfireRate { 0.0 };
    double airFuelRatio { 14.7 };
    double targetAirFuelRatio { 14.7 };
    double ignitionAdvanceDegrees { 0.0 };
    double manifoldPressureKpa { 28.0 };
    double exhaustPressureKpa { 101.325 };
    double volumetricEfficiency { 0.0 };
    double airMassMgPerCycle { 0.0 };
    double injectedFuelMgPerCycle { 0.0 };
    double fuelFlowGramsPerSecond { 0.0 };
    double airFlowGramsPerSecond { 0.0 };
    double lambda { 1.0 };
    double brakeSpecificFuelConsumptionGPerKwh { 0.0 };
    double mechanicalEfficiency { 0.0 };
    double oilPressureKpa { 0.0 };
    double peakPistonAccelerationG { 0.0 };
    double powerKw { 0.0 };
    double cycleAveragedPowerKw { 0.0 };
    double wear { 0.0 };
    double damage { 0.0 };
    RunningState runningState { RunningState::stopped };
    std::array<CylinderState, 32> cylinderStates {};
    std::size_t cylinderStateCount { 0 };
};

struct EcuCommand final {
    double targetAirFuelRatio { 14.2 };
    double ignitionAdvanceDegrees { 18.0 };
    double effectiveThrottle { 0.0 };
    double fuelCorrection { 1.0 };
    bool fuelEnabled { true };
    bool sparkEnabled { true };
};

struct CombustionResult final {
    double indicatedTorqueNm { 0.0 };
    double pressureEstimateBar { 0.0 };
    double combustionQuality { 0.0 };
    double heatOutput { 0.0 };
    double knockLevel { 0.0 };
    double misfireProbability { 0.0 };
    double volumetricEfficiency { 0.0 };
    double airMassMgPerCycle { 0.0 };
    double fuelMassMgPerCycle { 0.0 };
    double thermalEfficiency { 0.0 };
    double actualAirFuelRatio { 14.7 };
    double heatPowerKw { 0.0 };
    bool combustionEnabled { false };
};

struct DynoPoint final {
    double rpm { 0.0 };
    double torqueNm { 0.0 };
    double powerKw { 0.0 };
    double airFuelRatio { 14.7 };
    double coolantTemperatureC { 22.0 };
    double exhaustTemperatureC { 22.0 };
    double ignitionAdvanceDegrees { 0.0 };
    double atmosphericCorrectionFactor { 1.0 };
    double correctedTorqueNm { 0.0 };
    double correctedPowerKw { 0.0 };
    double targetAirFuelRatio { 14.7 };
    double volumetricEfficiency { 0.0 };
    double fuelFlowGramsPerSecond { 0.0 };
    double manifoldPressureKpa { 0.0 };
    double exhaustPressureKpa { 0.0 };
    double oilTemperatureC { 22.0 };
    double oilPressureKpa { 0.0 };
    double airFlowGramsPerSecond { 0.0 };
    double lambda { 1.0 };
    double brakeSpecificFuelConsumptionGPerKwh { 0.0 };
};

struct DynoRun final {
    std::uint64_t id { 0 };
    std::string engineName;
    std::vector<DynoPoint> points;
    double peakTorqueNm { 0.0 };
    double peakPowerKw { 0.0 };
    double peakCorrectedTorqueNm { 0.0 };
    double peakCorrectedPowerKw { 0.0 };
};

[[nodiscard]] EngineConfig makeDefaultInlineFour();
[[nodiscard]] EngineConfig makeDefaultInlineTwo();
[[nodiscard]] EngineConfig makeDefaultInlineFive();
[[nodiscard]] EngineConfig makeDefaultV6();
[[nodiscard]] EngineConfig makeDefaultV8();
[[nodiscard]] std::vector<EngineConfig> makeBaseEnginePresets();
[[nodiscard]] double engineDisplacementLitres(const EngineConfig&) noexcept;
[[nodiscard]] double valveLiftMm(double crankAngleDegrees, double centerlineDegrees,
                                 double durationDegrees, double maximumLiftMm) noexcept;
[[nodiscard]] double combustionPulse(double cylinderPhaseDegrees, double ignitionAdvanceDegrees,
                                     double ignitionOffsetDegrees) noexcept;
[[nodiscard]] std::optional<std::string> validateEngineConfig(const EngineConfig&);

} // namespace enginelab
