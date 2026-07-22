#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <optional>
#include <array>

namespace enginelab {
inline constexpr std::uint32_t minimumSupportedEngineSchemaVersion { 1 };
inline constexpr std::uint32_t currentEngineSchemaVersion { 4 };


enum class EngineCycle : std::uint8_t { fourStroke, twoStroke };
enum class FuelType : std::uint8_t { gasoline, diesel };
enum class InjectionMode : std::uint8_t { port, direct };
enum class EngineLayout : std::uint8_t { inlineLayout, vLayout, flat, radial, custom };
enum class ConnectingRodType : std::uint8_t { conventional, master, articulated };
enum class ForcedInductionType : std::uint8_t { turbocharger, supercharger };
enum class ExhaustComponentType : std::uint8_t {
    pipe, merge, splitter, resonator, muffler, catalyst, outlet
};
enum class AcousticTerminationType : std::uint8_t { unflanged, flanged };
enum class RunningState : std::uint8_t {
    stopped, cranking, idling, running, unstable, knocking, overheating, damaged, destroyed
};

/** Calibrated properties of the gasoline surrogate used by this engine. */
struct FuelConfig final {
    std::string name { "Pump gasoline" };
    double lowerHeatingValueMjPerKg { 43.0 };
    double densityKgPerL { 0.745 };
    double stoichiometricAirFuelRatio { 14.7 };
    double molarMassGramsPerMole { 114.23 };
    double oxygenMolesPerFuelMole { 12.5 };
    double productMolesPerFuelMole { 17.0 };
    double laminarFlameSpeedMps { 0.38 };
    double turbulenceFlameSpeedGain { 1.55 };
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
    std::uint32_t crankJournalId { 0 };
    double bankOffsetDegrees { 0.0 };
    std::uint32_t bankId { 0 };
    // Zero delegates to the selected intake path.  A non-zero value is a
    // genuine per-cylinder override rather than an accidental global default.
    double intakeRunnerLengthMm { 0.0 };
    double intakeRunnerDiameterMm { 0.0 };
    double exhaustPrimaryLengthMm { 0.0 };
    double soundAttenuation { 1.0 };
    double blowByCoefficient { 0.00002 };
    double connectingRodMassGrams { 600.0 };
    double pistonFrictionCoefficient { 0.025 };
    double pistonBreakawayForceN { 45.0 };
    double pistonBreakawayVelocityMps { 0.10 };
    double pistonViscousFrictionNsPerM { 4.5 };
    ConnectingRodType connectingRodType { ConnectingRodType::conventional };
    std::uint32_t masterCylinderId { 0 };
    double articulatedJournalRadiusMm { 0.0 };
    double articulatedJournalAngleDegrees { 0.0 };
    double deckHeightMm { 0.0 };
    double compressionHeightMm { 0.0 };
    double wristPinOffsetMm { 0.0 };
    double pistonCrownVolumeCc { 0.0 };
    double headChamberVolumeCc { 0.0 };
    double headGasketThicknessMm { 0.0 };
    double connectingRodMomentOfInertiaKgM2 { 0.0 };
    std::uint32_t intakeValveCount { 2 };
    std::uint32_t exhaustValveCount { 2 };
    // Zero selects a physically proportioned diameter from bore and count.
    double intakeValveDiameterMm { 0.0 };
    double exhaustValveDiameterMm { 0.0 };
};

struct CrankJournalConfig final {
    std::uint32_t id { 0 };
    double angleDegrees { 0.0 };
    double throwMm { 0.0 };
    std::uint32_t crankshaftId { 1 };
};

struct CrankshaftConfig final {
    std::uint32_t id { 1 };
    double positionXMm { 0.0 };
    double positionYMm { 0.0 };
    double phaseOffsetDegrees { 0.0 };
    double rotationRatio { 1.0 };
    double massKg { 18.0 };
    double flywheelMassKg { 8.0 };
    double momentOfInertiaKgM2 { 0.0 };
    double frictionTorqueNm { 0.0 };
    // Runtime migration provenance; deliberately not part of schema v1.
    // Explicitly decoded crankshafts keep the default false value.
    bool inheritsLegacyInertia { false };
};

struct ValveLiftSample final {
    double angleDegrees { 0.0 };
    double liftMm { 0.0 };
};

struct ValveFlowSample final {
    double liftMm { 0.0 };
    double dischargeCoefficient { 0.0 };
};

struct ValveControlSample final {
    double rpm { 0.0 };
    double load { 0.0 };
    double intakeAdvanceDegrees { 0.0 };
    double exhaustAdvanceDegrees { 0.0 };
    double liftMultiplier { 1.0 };
};

struct ValveControlConfig final {
    bool enabled { false };
    double responseFrequencyHz { 8.0 };
    std::vector<ValveControlSample> samples;
};

struct CamshaftConfig final {
    double intakeDurationDegrees { 248.0 };
    double exhaustDurationDegrees { 244.0 };
    double intakeLiftMm { 10.2 };
    double exhaustLiftMm { 9.8 };
    double intakeCenterlineDegrees { 110.0 };
    double exhaustCenterlineDegrees { 112.0 };
    double intakeFlowCoefficient { 0.62 };
    double exhaustFlowCoefficient { 0.62 };
    std::vector<ValveLiftSample> intakeLiftProfile;
    std::vector<ValveLiftSample> exhaustLiftProfile;
    bool variableProfileEnabled { false };
    double switchRpm { 5'800.0 };
    double switchThrottle { 0.55 };
    double highIntakeDurationDegrees { 282.0 };
    double highExhaustDurationDegrees { 276.0 };
    double highIntakeLiftMm { 12.5 };
    double highExhaustLiftMm { 11.8 };
    std::vector<ValveLiftSample> highIntakeLiftProfile;
    std::vector<ValveLiftSample> highExhaustLiftProfile;
    std::vector<ValveFlowSample> intakeFlowCurve;
    std::vector<ValveFlowSample> exhaustFlowCurve;
    ValveControlConfig continuousControl;
};

struct ExhaustConfig final {
    double primaryLengthMm { 480.0 };
    double primaryDiameterMm { 42.0 };
    double collectorDiameterMm { 58.0 };
    double mufflerRestriction { 0.28 };
    double outletDiameterMm { 65.0 };
    double collectorVolumeLitres { 2.0 };
    double outletDischargeCoefficient { 0.72 };
    /** Single expansion chamber on the collector-to-outlet duct. Zero on either
     *  field means "no silencer": the audio element becomes an exact
     *  through-connection, which is the correct model for an open stack and
     *  keeps every engine that predates these fields rendering unchanged. See
     *  `enginelab/audio/ExpansionChamberMuffler.hpp`. */
    double mufflerChamberDiameterMm { 0.0 };
    double mufflerChamberLengthMm { 0.0 };
};

struct IntakeConfig final {
    double plenumVolumeLitres { 3.0 };
    double throttleDiameterMm { 60.0 };
    /** Number of equal throttle bores feeding this intake path. */
    std::uint32_t throttleCount { 1 };
    double throttleDischargeCoefficient { 0.72 };
    double runnerLengthMm { 300.0 };
    double runnerDiameterMm { 38.0 };
    /** Optional upstream acoustic hardware. Zero volume/length means the
     * throttle mouth is directly exposed rather than inventing an airbox. */
    double airboxVolumeLitres { 0.0 };
    double inletDuctLengthMm { 0.0 };
    /** Zero inherits the throttle diameter. */
    double inletDuctDiameterMm { 0.0 };
    /** Zero inherits the inlet-duct diameter; used by inlet radiation. */
    double bellmouthDiameterMm { 0.0 };
    double idleBypassAreaMm2 { 12.0 };
    double throttleGamma { 1.7 };
    /**
     * Flow area still open with the throttle plate fully closed, per bore.
     *
     * A throttle plate is a disc in a bore and cannot seal against it: it runs
     * with a working clearance, and production throttle bodies additionally
     * carry a deliberate minimum air path (a stop screw or a machined notch) so
     * the engine can breathe with the pedal released and the idle actuator shut.
     * The closed-plate leakage of a typical 60 mm body is a few square
     * millimetres, which is the order of this default.
     *
     * Without it the intake has literally zero flow area whenever the driver
     * throttle and the idle bypass command are both zero, and the engine pumps
     * against a sealed manifold. Measured on the catalogue before this existed,
     * manifold pressure fell to about 10 kPa during the post-start speed flare
     * -- a vacuum no throttled engine can produce -- torque went to about
     * -75 Nm, and every larger engine stalled within a second of the starter
     * releasing.
     *
     * This is intake hardware, not a control command. It deliberately does not
     * live in the ECU: the driver's throttle and the idle actuator must both
     * still be able to close completely, which is why a minimum *commanded*
     * throttle opening was previously removed. The floor belongs to the bore.
     *
     * The default is a genuine leak and not an idle air supply. It is small
     * enough that the idle actuator still governs idle: at 3 mm2 per bore a
     * four-throttle 1.3 litre engine idled at 1945 rpm against a 1250 rpm
     * target, because leakage alone exceeded its idle air demand. What keeps an
     * engine alive through the post-start flare is the ECU's post-start air
     * schedule, not this.
     */
    double closedThrottleLeakageAreaMm2 { 0.6 };
};

struct IntakePathConfig final {
    std::uint32_t id { 1 };
    std::vector<std::uint32_t> cylinderIds;
    IntakeConfig geometry;
    // True only for a path synthesized from the legacy/global intake fields.
    // Explicit paths are authoritative and retain the default false value.
    bool inheritsGlobalGeometry { false };
};

struct CylinderBankConfig final {
    std::uint32_t id { 0 };
    double angleDegrees { 0.0 };
    std::vector<std::uint32_t> cylinderIds;
    CamshaftConfig camshafts;
    std::uint32_t intakeId { 0 };
    std::uint32_t exhaustPathId { 0 };
};

/** Engine-fixed free-field acoustic coordinates, in metres.
 *
 * +X is the engine's right side, +Y points out of the nominal tailpipe and +Z
 * points upward. These are geometry and measurement-chain data, never pan or
 * gain controls.
 */
struct AcousticPoint3M final {
    double x { 0.0 };
    double y { 0.0 };
    double z { 0.0 };
};

struct AcousticObserverConfig final {
    AcousticPoint3M leftMicrophoneM { -0.18, 3.0, 0.8 };
    AcousticPoint3M rightMicrophoneM { 0.18, 3.0, 0.8 };
    /** Zero derives local sound speed from the acoustic medium. */
    double soundSpeedMps { 0.0 };
};

/** A user-authored component in one exhaust path's directed acyclic graph. */
struct ExhaustComponentConfig final {
    // Component IDs are local to the containing exhaust path.
    std::uint32_t id { 0 };
    ExhaustComponentType type { ExhaustComponentType::pipe };
    double lengthMm { 300.0 };
    double diameterMm { 42.0 };
    double volumeLitres { 0.0 };
    // Dimensionless pressure-loss coefficient added to the geometric loss.
    double restriction { 0.0 };
    // Zero lets the runtime derive the dominant quarter-wave resonance.
    double resonanceHz { 0.0 };
    double acousticGain { 1.0 };
    double dischargeCoefficient { 0.72 };
    /** Radiation geometry, used only when type==outlet. */
    AcousticPoint3M acousticPositionM {};
    AcousticPoint3M acousticAxis { 0.0, 1.0, 0.0 };
    AcousticTerminationType acousticTermination {
        AcousticTerminationType::unflanged };
};

struct ExhaustCylinderConnectionConfig final {
    std::uint32_t cylinderId { 0 };
    std::uint32_t componentId { 0 };
};

struct ExhaustComponentConnectionConfig final {
    std::uint32_t fromComponentId { 0 };
    std::uint32_t toComponentId { 0 };
};

/**
 * Optional custom topology for an exhaust path. When absent, the legacy
 * geometry fields are compiled to a primary/collector/muffler/outlet graph.
 */
struct ExhaustNetworkConfig final {
    std::vector<ExhaustComponentConfig> components;
    std::vector<ExhaustCylinderConnectionConfig> cylinderConnections;
    std::vector<ExhaustComponentConnectionConfig> connections;
};

struct ExhaustPathConfig final {
    std::uint32_t id { 0 };
    std::vector<std::uint32_t> cylinderIds;
    ExhaustConfig geometry;
    std::string impulseResponsePath;
    double audioVolume { 1.0 };
    std::optional<ExhaustNetworkConfig> network;
    // Runtime migration provenance; deliberately not part of schema v1.
    bool inheritsGlobalGeometry { false };
    /** Radiation geometry for the outlet synthesized from scalar path data. */
    AcousticPoint3M acousticPositionM {};
    AcousticPoint3M acousticAxis { 0.0, 1.0, 0.0 };
    AcousticTerminationType acousticTermination {
        AcousticTerminationType::unflanged };
};

struct IgnitionMapSample final {
    double rpm { 0.0 };
    double advanceDegrees { 10.0 };
};

struct IgnitionConfig final {
    std::vector<IgnitionMapSample> timingCurve {
        { 0.0, 10.0 }, { 1'000.0, 14.0 }, { 2'000.0, 22.0 },
        { 3'500.0, 30.0 }, { 5'500.0, 34.0 }, { 8'000.0, 32.0 }
    };
    double revLimitRpm { 7'200.0 };
    double limiterDurationSeconds { 0.08 };
};

struct InjectionConfig final {
    InjectionMode mode { InjectionMode::direct };
    double startAngleDegrees { 570.0 };
    double endAngleDegrees { 690.0 };
    // Per-injector mass flow. At 0.745 kg/L, 400 cc/min is about 4,970 mg/s;
    // the larger DI default reflects its high pressure and short open window.
    double injectorFlowMgPerSecond { 20'000.0 };
    double fuelTemperatureC { 25.0 };
    // Port injection: regulated differential pressure relative to the intake
    // manifold. Direct injection: absolute rail pressure.
    double railPressureBar { 200.0 };
    // Differential pressure at which injectorFlowMgPerSecond is specified.
    double referencePressureBar { 200.0 };
    double wallFilmFraction { 0.0 };
    double vaporisationTimeConstantSeconds { 0.035 };
    double latentHeatKjPerKg { 350.0 };
    double directChargeCoolingEfficiency { 0.82 };
    double portChargeCoolingEfficiency { 0.28 };
};

struct SolverConfig final {
    double mechanicalFrequencyHz { 2'000.0 };
    double maximumMechanicalFrequencyHz { 60'000.0 };
    double maximumCrankDegreesPerStep { 2.0 };
    std::uint32_t gasSubsteps { 1 };
};

struct TransmissionConfig final {
    std::vector<double> gearRatios { 2.97, 2.07, 1.43, 1.00, 0.84, 0.56 };
    double finalDriveRatio { 3.42 };
    double maxClutchTorqueNm { 1'356.0 };
    double drivelineEfficiency { 0.88 };
    double drivenWheelInertiaKgM2 { 3.6 };
    // Half-width of the clutch lock window in slip rpm: inside it the disc sticks
    // and carries torque up to capacity, outside it slips at capacity (kinetic).
    double clutchLockSpeedRpm { 35.0 };
    double shiftDurationSeconds { 0.18 };
    bool automaticShifting { false };
    double automaticUpshiftRpm { 6'400.0 };
    double automaticDownshiftRpm { 1'650.0 };
    double reverseRatio { 3.20 };
    double gearboxInputInertiaKgM2 { 0.08 };
    double differentialInertiaKgM2 { 0.16 };
    double clutchThermalCapacityJPerC { 18'000.0 };
    double clutchCoolingWPerC { 8.0 };
    double clutchFadeStartTemperatureC { 220.0 };
    double clutchFailureTemperatureC { 420.0 };
    double shiftTorqueCutFraction { 0.85 };
};

struct VehicleConfig final {
    double massKg { 1'420.0 };
    double dragCoefficient { 0.32 };
    double frontalAreaM2 { 2.20 };
    double tireRadiusM { 0.315 };
    double rollingResistanceCoefficient { 0.014 };
    double tireFrictionCoefficient { 1.0 };
    double drivenAxleWeightFraction { 0.55 };
    double maximumBrakeForceN { 14'000.0 };
};

struct ForcedInductionConfig final {
    bool enabled { false };
    ForcedInductionType type { ForcedInductionType::turbocharger };
    double pressureRatio { 1.0 };
    double fullBoostRpm { 3'500.0 };
    double compressorEfficiency { 0.68 };
    double chargeTemperatureRiseC { 35.0 };
    double turbineEfficiency { 0.68 };
    double shaftInertiaKgM2 { 0.00012 };
    double wastegatePressureRatio { 1.05 };
    double designShaftSpeedRpm { 130'000.0 };
    double bearingFrictionPowerWatts { 280.0 };
    double turbineFlowAreaMm2 { 700.0 };
    double wastegateFlowAreaMm2 { 520.0 };
    /** Rotor geometry used by aeroacoustics. Zero count disables the
     * corresponding blade/lobe-passing tone rather than inventing one. */
    std::uint32_t compressorBladeCount { 0 };
    std::uint32_t turbineBladeCount { 0 };
    std::uint32_t superchargerLobeCount { 0 };
    double superchargerDriveRatio { 1.0 };
    double compressorInducerDiameterMm { 0.0 };
    double turbineExducerDiameterMm { 0.0 };
    /** Vent-to-ambient compressor bypass. Zero area means absent. The valve
     * opens from upstream/downstream pressure ratio, never throttle gestures. */
    double blowOffValveFlowAreaMm2 { 0.0 };
    double blowOffValveOpeningPressureRatio { 1.12 };
    double blowOffValveDischargeCoefficient { 0.72 };
    /** Measurable/semi-empirical conversion parameters, not mix gains. */
    double tonalAcousticEfficiency { 1.0e-6 };
    double turbulentJetNoiseCoefficient { 1.0e-5 };
};

struct ThermalConfig final {
    double coolantMassKjPerC { 180.0 };
    double oilMassKjPerC { 95.0 };
    double coolantHeatShare { 0.34 };
    double oilHeatShare { 0.18 };
    double coolingPowerKwPerC { 0.42 };
    double oilCoolingPowerKwPerC { 0.22 };
};

struct CombustionCalibrationConfig final {
    double baseIgnitionDelaySeconds { 0.00035 };
    double ignitionDelayTemperatureExponent { 1.10 };
    double ignitionDelayPressureExponent { 0.35 };
    double wallHeatTransferCoefficientWPerK { 0.42 };
    double residualDilutionSensitivity { 0.78 };
};

struct RunnerAcousticsConfig final {
    bool enabled { true };
    double dampingRatio { 0.16 };
    double couplingGain { 0.45 };
    double maximumPressureAmplitudeKpa { 35.0 };
};

struct EngineConfig final {
    std::uint32_t schemaVersion { currentEngineSchemaVersion };
    std::string name { "Untitled engine" };
    EngineCycle cycle { EngineCycle::fourStroke };
    FuelType fuel { FuelType::gasoline };
    FuelConfig fuelProperties;
    EngineLayout layout { EngineLayout::inlineLayout };
    std::vector<CylinderConfig> cylinders;
    std::vector<std::uint32_t> firingOrder;
    std::vector<CrankJournalConfig> crankJournals;
    std::vector<CrankshaftConfig> crankshafts;
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
    IntakeConfig intake;
    std::vector<IntakePathConfig> intakePaths;
    std::vector<CylinderBankConfig> banks;
    std::vector<ExhaustPathConfig> exhaustPaths;
    AcousticObserverConfig acousticObserver;
    IgnitionConfig ignition;
    InjectionConfig injection;
    SolverConfig solver;
    ForcedInductionConfig forcedInduction;
    ThermalConfig thermal;
    CombustionCalibrationConfig combustionCalibration;
    RunnerAcousticsConfig runnerAcoustics;
    CamshaftConfig camshafts;
    ExhaustConfig exhaust;
    TransmissionConfig transmission;
    VehicleConfig vehicle;
};

struct EngineControls final {
    bool ignitionEnabled { false };
    bool starterEngaged { false };
    double throttle { 0.0 };
    // Legacy normalized absorber command retained for existing integrations.
    // New dynamometer code should command a calibrated reaction torque below.
    double load { 0.0 };
    double externalTorqueNm { 0.0 };
    double brake { 0.0 };
    double dynamometerTorqueNm { 0.0 };
};

struct CylinderState final {
    std::uint32_t id { 0 };
    double cyclePhaseDegrees { 0.0 };
    double pressureEstimateBar { 0.0 };
    double combustionPulse { 0.0 };
    double misfireProbability { 0.0 };
    double intakeValveLiftMm { 0.0 };
    double exhaustValveLiftMm { 0.0 };
    double intakeFlowMgPerCycle { 0.0 };
    double exhaustFlowMgPerCycle { 0.0 };
    double runnerPressureKpa { 101.325 };
    double exhaustTemperatureC { 22.0 };
    double gasTemperatureC { 22.0 };
    double trappedMassMg { 0.0 };
    double oxygenMoles { 0.0 };
    double fuelMoles { 0.0 };
    double burnedMoles { 0.0 };
    double intakeVelocityMps { 0.0 };
    double exhaustVelocityMps { 0.0 };
    double fuelDeliveryRatio { 0.0 };
    double flameSpeedMps { 0.0 };
    double burnedFraction { 0.0 };
    double combustionEfficiency { 0.0 };
    double endGasKnockLevel { 0.0 };
    bool combustionActive { false };
    bool misfiring { false };
    double pistonTravelMm { 0.0 };
    double pistonPositionMm { 0.0 };
    double pistonVelocityMps { 0.0 };
    double pistonAccelerationMps2 { 0.0 };
    double connectingRodAngleDegrees { 0.0 };
    double crankPinXMm { 0.0 };
    double crankPinYMm { 0.0 };
    double wristPinXMm { 0.0 };
    double wristPinYMm { 0.0 };
    double mechanicalReactionTorqueNm { 0.0 };
    std::uint32_t crankshaftId { 1 };
    std::uint32_t crankJournalId { 0 };
    double indicatedWorkJoulesPerCycle { 0.0 };
    double residualGasFraction { 0.0 };
    double airFuelRatio { 14.7 };
    double requestedFuelMgPerCycle { 0.0 };
    double deliveredFuelMgPerCycle { 0.0 };
    double closedLoopFuelTrim { 1.0 };
    double intakeValveAdvanceDegrees { 0.0 };
    double exhaustValveAdvanceDegrees { 0.0 };
    double valveLiftMultiplier { 1.0 };
    double intakeResonancePressureKpa { 0.0 };
    double intakeResonanceFrequencyHz { 0.0 };
    /** Charge state in this cylinder's own intake runner, at the valve.
     *
     * What an IAT sensor in the port would read, and the direct determinant of
     * charge density and therefore of volumetric efficiency. Reported per
     * cylinder rather than per plenum because reversion through the intake
     * valve heats one runner at a time. */
    double intakeRunnerTemperatureC { 22.0 };
    double intakeRunnerChargePressureKpa { 101.325 };
};

struct EngineState final {
    double simulationTimeSeconds { 0.0 };
    double rpm { 0.0 };
    double crankAngleDegrees { 719.9 };
    double throttle { 0.0 };
    double load { 0.0 };
    double angularVelocityRadPerSecond { 0.0 };
    double indicatedTorqueNm { 0.0 };
    double meanWorkTorqueNm { 0.0 };
    double cylinderPressureTorqueNm { 0.0 };
    double cylinderPressureTorqueBlend { 0.0 };
    double frictionTorqueNm { 0.0 };
    double frictionMeanEffectivePressureBar { 0.0 };
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
    double boostPressureRatio { 1.0 };
    double exhaustPressureKpa { 101.325 };
    double intakeRunnerPressureKpa { 101.325 };
    double exhaustRunnerPressureKpa { 101.325 };
    double exhaustFlowGramsPerSecond { 0.0 };
    double manifoldGasMassGrams { 0.0 };
    double cylinderGasMassGrams { 0.0 };
    double gasInternalEnergyJoules { 0.0 };
    double solverFrequencyHz { 0.0 };
    double crankDegreesPerSolverStep { 0.0 };
    std::uint32_t solverSubsteps { 0 };
    bool solverResolutionLimited { false };
    /** CFL substeps the exhaust network accepted internally over the last frame.
     *
     * The network resolves the acoustic field at this cadence, which is far
     * above the multirate coupling cadence that the audio boundary is sampled
     * at. Published so the ratio between the two is observable: bandwidth the
     * solver has already paid for but that nothing downstream reads is invisible
     * from any other measurement.
     */
    std::uint64_t exhaustNetworkAcceptedSubsteps { 0 };
    /** Wall-clock rate of those substeps, Hz. Compare against the coupling
     * cadence to see how much resolved bandwidth reaches the audio path.
     */
    double exhaustNetworkSubstepFrequencyHz { 0.0 };
    /** Cadence at which the audio boundary is actually sampled, Hz. Content
     * above half this frequency cannot be physical.
     */
    double exhaustCouplingFrequencyHz { 0.0 };
    double volumetricEfficiency { 0.0 };
    double airMassMgPerCycle { 0.0 };
    double injectedFuelMgPerCycle { 0.0 };
    double deliveredFuelMgPerCycle { 0.0 };
    double fuelFlowGramsPerSecond { 0.0 };
    double fuelConsumedGrams { 0.0 };
    double fuelConsumedLitres { 0.0 };
    double fuelEconomyLitresPer100Km { 0.0 };
    double airFlowGramsPerSecond { 0.0 };
    double lambda { 1.0 };
    double brakeSpecificFuelConsumptionGPerKwh { 0.0 };
    double mechanicalEfficiency { 0.0 };
    double oilPressureKpa { 0.0 };
    double peakPistonAccelerationG { 0.0 };
    double resolvedHeatReleaseKw { 0.0 };
    double compressorPowerKw { 0.0 };
    double turbinePowerKw { 0.0 };
    double forcedInductionShaftSpeedRpm { 0.0 };
    double wastegateOpening { 0.0 };
    double blowOffMassFlowKgPerSecond { 0.0 };
    double indicatedWorkJoulesPerCycle { 0.0 };
    double indicatedMeanEffectivePressureBar { 0.0 };
    double indicatedPowerKw { 0.0 };
    double pdvTorqueNm { 0.0 };
    double powerKw { 0.0 };
    double cycleAveragedPowerKw { 0.0 };
    double wear { 0.0 };
    double damage { 0.0 };
    int gear { -1 };
    int gearCount { 0 };
    double clutchPressure { 1.0 };
    double vehicleSpeedMps { 0.0 };
    double vehicleDistanceM { 0.0 };
    double wheelTorqueNm { 0.0 };
    double drivelineLoadTorqueNm { 0.0 };
    double clutchTorqueNm { 0.0 };
    double clutchSlipRpm { 0.0 };
    double shiftProgress { 0.0 };
    bool shiftInProgress { false };
    double brakePressure { 0.0 };
    double brakeForceN { 0.0 };
    double requestedRoadLoad { 0.0 };
    double roadLoadForceN { 0.0 };
    double tireLongitudinalForceN { 0.0 };
    bool tractionLimited { false };
    double clutchTemperatureC { 22.0 };
    double clutchDissipatedEnergyJoules { 0.0 };
    double clutchPowerLossKw { 0.0 };
    double drivelineStoredEnergyJoules { 0.0 };
    double drivelineEnergyResidualJoules { 0.0 };
    double dynoHoldRpm { 0.0 };
    bool dynoHoldEnabled { false };
    RunningState runningState { RunningState::stopped };
    std::array<CylinderState, 32> cylinderStates {};
    std::size_t cylinderStateCount { 0 };
};

struct EcuCommand final {
    double targetAirFuelRatio { 14.2 };
    double ignitionAdvanceDegrees { 18.0 };
    double effectiveThrottle { 0.0 };
    /** Normalised command for the configured idle-air bypass actuator. */
    double idleAirOpening { 0.0 };
    double fuelCorrection { 1.0 };
    bool fuelEnabled { true };
    bool sparkEnabled { true };
};

// Output of the mean-value model SimplifiedGasolinePhysics::evaluateCombustion.
// It is telemetry, NOT the crank-driving model: the crankshaft is integrated
// from the 0-D solver's chamber pressure (see that model's header and
// docs/physics-audit.md). This is the *measured* consumption map (Phase 6): the
// only production readers are EngineSimulator and FourStrokeEventGenerator, plus
// the EngineLab.Core characterization test. Field order is load-bearing (used by
// designated/positional init in the model); do not reorder to "tidy" the groups.
//
//   Drives live behaviour:
//     combustionEnabled   gates injection/combustion/event generation
//     combustionQuality   exhaust-audio firing amplitude (read unconditionally)
//     heatOutput          cylinder wall temperature -> wall heat transfer
//     pressureEstimateBar firing pressure, event-gen fallback when no resolved
//                         per-cylinder state exists
//     misfireProbability  misfire draw, event-gen fallback (same condition)
//     airMassMgPerCycle   breathingQuality display
//   Telemetry / test-only (no production reader):
//     indicatedTorqueNm   copied to state.meanWorkTorqueNm for display only
//     actualAirFuelRatio  read only by EngineLab.Core (characterization)
//     heatPowerKw         read only by EngineLab.Core (characterization)
//   Unread anywhere (kept as struct-stable telemetry; the value is still a
//   necessary local intermediate inside the model, so the field is cheap):
//     knockLevel (forced 0.0), volumetricEfficiency, fuelMassMgPerCycle,
//     thermalEfficiency
struct CombustionResult final {
    double indicatedTorqueNm { 0.0 };     // telemetry -> state.meanWorkTorqueNm (display)
    double pressureEstimateBar { 0.0 };   // event-gen firing-pressure fallback
    double combustionQuality { 0.0 };     // exhaust-audio amplitude (live, unconditional)
    double heatOutput { 0.0 };            // cylinder wall temperature (live)
    double knockLevel { 0.0 };            // unread; knock resolved per-cylinder in EngineSimulator
    double misfireProbability { 0.0 };    // event-gen misfire fallback
    double volumetricEfficiency { 0.0 };  // unread (intermediate exposed as telemetry)
    double airMassMgPerCycle { 0.0 };     // breathingQuality display (live)
    double fuelMassMgPerCycle { 0.0 };    // unread (intermediate exposed as telemetry)
    double thermalEfficiency { 0.0 };     // unread (intermediate exposed as telemetry)
    double actualAirFuelRatio { 14.7 };   // read only by EngineLab.Core characterization
    double heatPowerKw { 0.0 };           // read only by EngineLab.Core characterization
    bool combustionEnabled { false };     // gates injection/combustion/event-gen (live)
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
[[nodiscard]] EngineConfig makeDefaultFlatSix();
[[nodiscard]] EngineConfig makeDefaultRadialFive();
[[nodiscard]] std::vector<EngineConfig> makeBaseEnginePresets();
[[nodiscard]] double engineDisplacementLitres(const EngineConfig&) noexcept;
[[nodiscard]] double effectiveRotatingInertiaKgM2(const EngineConfig&) noexcept;
[[nodiscard]] double intakeRunnerVolumeLitres(const CylinderConfig&, const IntakeConfig&) noexcept;
void normaliseEngineConfig(EngineConfig&);
[[nodiscard]] double valveFlowCoefficient(double liftMm, double fallback,
                                          const std::vector<ValveFlowSample>&) noexcept;
[[nodiscard]] ValveControlSample interpolateValveControl(const ValveControlConfig&, double rpm,
                                                         double load) noexcept;
[[nodiscard]] double valveLiftMm(double crankAngleDegrees, double centerlineDegrees,
                                 double durationDegrees, double maximumLiftMm) noexcept;
[[nodiscard]] double profiledValveLiftMm(double crankAngleDegrees, double centerlineDegrees,
                                         double durationDegrees, double maximumLiftMm,
                                         const std::vector<ValveLiftSample>& samples) noexcept;
[[nodiscard]] double combustionPulse(double cylinderPhaseDegrees, double ignitionAdvanceDegrees,
                                     double ignitionOffsetDegrees) noexcept;
[[nodiscard]] std::optional<std::string> validateEngineConfig(const EngineConfig&);

} // namespace enginelab
