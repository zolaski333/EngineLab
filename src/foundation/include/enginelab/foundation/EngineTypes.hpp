#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <optional>
#include <array>

namespace enginelab {
inline constexpr std::uint32_t minimumSupportedEngineSchemaVersion { 1 };
inline constexpr std::uint32_t currentEngineSchemaVersion { 5 };


enum class EngineCycle : std::uint8_t { fourStroke, twoStroke };
enum class FuelType : std::uint8_t { gasoline, diesel };
enum class InjectionMode : std::uint8_t { port, direct };
enum class EngineLayout : std::uint8_t { inlineLayout, vLayout, flat, radial, custom };
enum class ConnectingRodType : std::uint8_t { conventional, master, articulated };
enum class ForcedInductionType : std::uint8_t { turbocharger, supercharger };
enum class DrivenAxleLayout : std::uint8_t { front, rear, all };
enum class StructuralNvhProvenance : std::uint8_t {
    estimatedFamily, calculatedGeometry, measured
};
enum class StructuralModeDrive : std::uint8_t {
    headGas, bearingAxial, bearingLateral, torsion
};
enum class ExhaustComponentType : std::uint8_t {
    pipe, merge, splitter, resonator, muffler, catalyst, outlet
};
enum class AcousticTerminationType : std::uint8_t { unflanged, flanged };
enum class RunningState : std::uint8_t {
    stopped, cranking, idling, running, unstable, knocking, overheating, damaged, destroyed
};

/** Calibrated thermochemical properties of the fuel surrogate. */
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
    /** Ignition quality for compression-ignition fuel. Zero explicitly means
     *  "not a compression-ignition fuel"; diesel configurations use 30-80.
     */
    double cetaneNumber { 0.0 };
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
    /** Runner diameter at the cylinder-head/valve end. */
    double runnerDiameterMm { 38.0 };
    /** Optional diameter at the plenum end. Zero keeps a constant runner. */
    double runnerPlenumDiameterMm { 0.0 };
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
    AcousticPoint3M leftMicrophoneM { -0.18, 4.0, 0.8 };
    AcousticPoint3M rightMicrophoneM { 0.18, 4.0, 0.8 };
    /** Zero derives local sound speed from the acoustic medium. */
    double soundSpeedMps { 0.0 };
};

/** A user-authored component in one exhaust path's directed acyclic graph. */
struct ExhaustComponentConfig final {
    // Component IDs are local to the containing exhaust path.
    std::uint32_t id { 0 };
    ExhaustComponentType type { ExhaustComponentType::pipe };
    double lengthMm { 300.0 };
    /** Diameter at the component inlet. */
    double diameterMm { 42.0 };
    /** Optional outlet diameter. Zero keeps a constant section. A non-zero
     * value forms a linear-area taper in the quasi-1D gas solver.
     */
    double outletDiameterMm { 0.0 };
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
    /** Optional perforated-core porous packing. All three must be positive;
     * zero leaves the acoustic network exactly unlined. */
    double packingFlowResistivityPaSPerM2 { 0.0 };
    double packingThicknessMm { 0.0 };
    double perforatedOpenAreaRatio { 0.0 };
};

/** Optional oxidation of unburned charge inside the physical exhaust network.
 * Disabled by default: enabling it never creates fuel, oxygen or an audio
 * event, it only lets inventories already present react when hot enough. */
struct ExhaustAfterfireConfig final {
    bool enabled { false };
    double ignitionTemperatureK { 900.0 };
    double reactionTimeConstantSeconds { 0.010 };
    double reactionEfficiency { 0.95 };
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
    // Fraction of a port-injection pulse that wets the port wall as liquid
    // film instead of vaporising in flight (the X of the X-tau wall-film
    // model; Aquino). Port-injection literature puts it at 0.3-0.6 warm and
    // higher cold; the catalogue loader applies 0.22 to port-injected
    // engines. Note, measured: the film does NOT by itself prevent the
    // flooded-misfire lock-in a big-cylinder engine can enter at idle catch
    // with a resolved 1-D runner (the V12 flooded identically at 0.22), so
    // do not reach for this knob to fix an idle. DI ignores this field.
    double wallFilmFraction { 0.35 };
    double vaporisationTimeConstantSeconds { 0.035 };
    double latentHeatKjPerKg { 350.0 };
    double directChargeCoolingEfficiency { 0.82 };
    double portChargeCoolingEfficiency { 0.28 };
    /** Direct-injection liquid-droplet vaporisation time at the reference
     *  chamber temperature. Zero preserves the legacy, fully-vaporised pulse.
     */
    double directSprayVaporisationTimeConstantSeconds { 0.0 };
    /** Time for vaporised spray parcels to entrain sufficient chamber gas to
     *  become part of the locally combustible mixture. Zero is instantaneous.
     */
    double directSprayEntrainmentTimeConstantSeconds { 0.0 };
    struct FuelQuantityLimitSample final {
        double rpm { 0.0 };
        /** Per-cylinder maximum metered fuel at full driver demand. */
        double milligramsPerCycle { 0.0 };
    };
    /** Compression-ignition full-load smoke/torque limiter. Empty means the
     *  oxygen-based target alone governs quantity. The driver-demand fraction
     *  scales this limit before it is applied.
     */
    std::vector<FuelQuantityLimitSample> fullLoadFuelLimit;
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
    DrivenAxleLayout drivenAxleLayout { DrivenAxleLayout::rear };
    /** Static fraction of vehicle weight carried by the driven axle. Ignored
     * for all-wheel drive, where every tyre contributes its normal load. */
    double drivenAxleWeightFraction { 0.55 };
    double wheelbaseM { 2.65 };
    double centerOfGravityHeightM { 0.52 };
    double maximumBrakeForceN { 14'000.0 };
    /** Whether the driven tyre may break traction and spin.
     *
     * Off by default, and deliberately so: the grip limit is a VEHICLE feature
     * layered on top of the engine, and until the engine side is trustworthy it
     * only adds a second explanation for every "it will not rev" observation --
     * which is exactly what happened, with a part-load fuelling defect being
     * read from the application as a traction limit because the panel was
     * announcing one. With this off the tyre transmits whatever the driveline
     * asks, the model behaves like a rolling road, and `tractionLimited` is
     * always false so the diagnostic cannot fire. Turning it on restores the
     * friction-circle clamp and the longitudinal load transfer that
     * `EngineLab.VehicleDynamics` exercises; that test sets it explicitly and
     * therefore still covers the model. */
    bool tyreGripLimitEnabled { false };
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
    /** Chamber-generated tumble/squish intensity relative to the legacy
     * mean-piston-speed closure. This belongs to the cylinder head, not to the
     * fuel chemistry: pent-roof/tumble ports and low-speed aircraft chambers
     * do not create the same u' at equal piston speed.
     */
    double chamberTurbulenceIntensityRatio { 1.0 };
    /** Independent flame kernels (normally spark plugs) per cylinder.
     *
     * Multiple kernels grow through disjoint chamber volume until their fronts
     * meet. This is physically distinct from increasing flame speed and is
     * important for large-bore dual-ignition aircraft engines.
     */
    std::uint32_t ignitionSiteCount { 1 };
    /** Multiplier on the pressure/temperature/cetane ignition-delay
     *  correlation used by the compression-ignition model.
     */
    double compressionIgnitionDelayScale { 1.0 };
    /** Mixing-controlled diffusion-burn time constant after autoignition. */
    double compressionIgnitionMixingTimeSeconds { 0.0014 };
    /** Fraction of fuel accumulated during ignition delay which burns in the
     *  initial premixed phase; the remainder is mixing controlled.
     */
    double compressionIgnitionPremixedFraction { 0.20 };
    /** Standard deviation of the physical burn-rate multiplier from one
     * cylinder cycle to the next. Zero is an exact deterministic bypass. */
    double cycleVariationCoefficientOfVariation { 0.0 };
    /** AR(1) correlation between consecutive combustion cycles. */
    double cycleVariationCorrelation { 0.45 };
};

/**
 * Intake-runner acoustics (see HelmholtzRunnerModel).
 *
 * `couplingGain` scales a forcing term -- `(plenum - runner)` -- that the 0-D
 * intake topology has already flattened to a few kPa, so it is an amplitude knob
 * and NOT a tuning parameter: raising it lifts the whole VE curve without moving
 * its peak, and on an engine whose low-speed torque is already at or above the
 * real one (the LS3: 596 Nm at 2000 rpm, real peak 575 at 4600) that is a
 * regression dressed as a calibration. Measured numbers in
 * HelmholtzRunnerModel's header and docs/physics-audit.md. Leave it at 0.45
 * until the runner has a real inertance to force against.
 */
struct RunnerAcousticsConfig final {
    bool enabled { true };
    double dampingRatio { 0.16 };
    double couplingGain { 0.45 };
    double maximumPressureAmplitudeKpa { 35.0 };
};

/** One authored structural mode. Every amplitude-bearing field is an SI
 * physical/modal parameter; there is deliberately no arbitrary mix gain. */
struct StructuralModeConfig final {
    std::string name;
    StructuralModeDrive drive { StructuralModeDrive::headGas };
    double frequencyHz { 1'000.0 };
    double dampingRatio { 0.04 };
    double modalMassKg { 5.0 };
    double radiatingAreaM2 { 0.10 };
    double radiationEfficiency { 0.50 };
    double surfaceVelocityRmsScale { 0.50 };
    double torqueRadiusM { 0.05 };
    /** Signed, antinode-normalised modal participation, one value per
     * cylinder in EngineConfig::cylinders order. */
    std::vector<double> cylinderParticipation;
};

/** Optional measured or geometry-calculated replacement for the family
 * estimate used by StructuralModalRadiator. Empty modes select the documented
 * estimated-family fallback and may not claim measured provenance. */
struct StructuralNvhConfig final {
    StructuralNvhProvenance provenance {
        StructuralNvhProvenance::estimatedFamily
    };
    /** Measurement report, dataset, model revision or other auditable source. */
    std::string source;
    std::vector<StructuralModeConfig> modes;
};

enum class AudioSaturationPlacement : std::uint8_t {
    preShelf,
    postShelf
};

/** Non-physical monitor voicing, kept separate from engine and exhaust physics.
 *
 * These values shape only the listening chain. Defaults exactly match the
 * historical hard-coded realtime mix, so introducing a voicing catalogue does
 * not silently retune existing engines.
 */
struct AudioVoicingConfig final {
    double volume { 1.0 };
    double convolution { 0.45 };
    double highFrequencyGain { 1.0 };
    double lowFrequencyGain { 1.0 };
    double lowFrequencyNoise { 0.35 };
    double highFrequencyNoise { 0.35 };
    double combustionGain { 1.0 };
    double exhaustGain { 1.0 };
    double intakeGain { 0.85 };
    double mechanicalGain { 0.70 };
    double stereoWidth { 1.0 };
    double outletJetGain { 1.0 };
    double saturationDrive { 0.0 };
    AudioSaturationPlacement saturationPlacement {
        AudioSaturationPlacement::postShelf
    };
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
    StructuralNvhConfig structuralNvh;
    CamshaftConfig camshafts;
    ExhaustConfig exhaust;
    ExhaustAfterfireConfig exhaustAfterfire;
    TransmissionConfig transmission;
    VehicleConfig vehicle;
    /** Resolved default -> family -> engine voicing. Runtime-only: the engine
     * serializers remain physics/configuration documents, while the catalogue
     * YAML files in voicing/ own listening-chain choices. */
    AudioVoicingConfig audioVoicing;
    std::string audioVoicingFamily;
    std::string audioVoicingKey;
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
    /** Fraction of the commanded injector pulse actually metered this cycle.
     *  1.0 means the injector kept up; a low value is real capacity saturation,
     *  independent of the closed-loop trim (unlike fuelDeliveryRatio). */
    double injectorCapacityRatio { 1.0 };
    double flameSpeedMps { 0.0 };
    double burnedFraction { 0.0 };
    double combustionEfficiency { 0.0 };
    double endGasKnockLevel { 0.0 };
    bool combustionActive { false };
    bool misfiring { false };
    // NOTE: `misfiring` is the last member reached by the positional aggregate
    // initialiser in EngineSimulator::publishCylinderStates. Add new members
    // BELOW this line and assign them by name, or every field after the
    // insertion point silently receives the wrong value.
    /** Instantaneous compact-valve conductance Cd*A used by the gas solver. */
    double exhaustValveConductanceAreaM2 { 0.0 };
    /** Signed instantaneous flow; positive from chamber into exhaust runner. */
    double exhaustMassFlowKgPerSecond { 0.0 };
    /** Oxygen-equivalent fresh air latched when this cylinder's intake valve
     * closed, on the same basis as EngineState::airMassMgPerCycle.
     */
    double trappedFreshAirMassMg { 0.0 };
    /** Net oxygen-equivalent fresh air delivered through this intake valve
     * during the most recently completed 720-degree cycle.
     */
    double deliveredFreshAirMassMgPerCycle { 0.0 };
    /**
     * Burned mole fraction in the chamber at the instant of ignition -- i.e. the
     * residual the gas exchange failed to expel, since nothing has burned yet.
     *
     * The flame model has always consumed this (it sets both the ignition-delay
     * residual penalty and the dilution factor of `combustionEfficiency`), but
     * it was only ever passed positionally into `FlameConditions` and never
     * published, so a residual failure could only be seen through its downstream
     * symptom: efficiency collapsing with no visible cause.
     *
     * Literature (Heywood ch. 6.4): 3-7 % at wide-open throttle, rising towards
     * 20 % at idle where the pressure ratio across the overlap is adverse. A WOT
     * figure above ~0.15 is a gas-exchange failure, not a calibration choice.
     *
     * Declared further down, next to the other named-assignment members.
     */
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
    /**
     * INSTANTANEOUS burned mole fraction, despite the name. Immediately after
     * combustion it is ~1.0, so a cycle average of it is not a residual figure
     * and cannot show a gas-exchange failure. Kept with this meaning because the
     * comparison harness's stall diagnostic and the perf harness CSV both read
     * it that way. For the residual, use `residualGasFractionAtSpark` below.
     */
    double residualGasFraction { 0.0 };
    /**
     * Burned mole fraction sampled at the instant of ignition -- i.e. the
     * residual the gas exchange failed to expel, since nothing has burned yet.
     *
     * The flame model has always consumed this quantity: it sets the
     * ignition-delay residual penalty and the dilution factor of
     * `combustionEfficiency`. But it was only ever passed positionally into
     * `FlameConditions` and never published, so a residual failure could only be
     * seen through its downstream symptom -- efficiency collapsing with no
     * visible cause -- while the field named `residualGasFraction` reported
     * something else entirely.
     *
     * Literature (Heywood ch. 6.4): 3-7 % at wide-open throttle, rising towards
     * 20 % at idle where the pressure ratio across the overlap is adverse. A WOT
     * figure above ~0.15 is a gas-exchange failure, not a calibration choice.
     */
    double residualGasFractionAtSpark { 0.0 };
    /**
     * Equivalence ratio in the chamber at the instant of ignition -- the mixture
     * the flame model actually evaluated, which is what sets the mixture factor
     * of `combustionEfficiency`. Distinct from `airFuelRatio`, a completed-cycle
     * figure: a fixed crank-angle injection window occupies more crank degrees
     * as speed rises, so at high rpm the charge can still be arriving when the
     * spark fires and the flame sees a leaner mixture than the cycle reports.
     */
    double equivalenceRatioAtSpark { 0.0 };
    double airFuelRatio { 14.7 };
    double requestedFuelMgPerCycle { 0.0 };
    double deliveredFuelMgPerCycle { 0.0 };
    double closedLoopFuelTrim { 1.0 };
    double intakeValveAdvanceDegrees { 0.0 };
    double exhaustValveAdvanceDegrees { 0.0 };
    double valveLiftMultiplier { 1.0 };
    double intakeResonancePressureKpa { 0.0 };
    double intakeResonanceFrequencyHz { 0.0 };
    /** Stagnation head of the arriving intake runner column, rho*u^2/2, in kPa.
     *
     * The charging pressure a column moving at `intakePortColumnVelocityMps` would
     * present at the valve, over and above the port's static pressure. DIAGNOSTIC
     * ONLY -- nothing reads it for flow. It peaks at full lift and is ~3 % of peak
     * by the time the valve seats, which is why biasing the fill with it loses
     * charge instead of trapping it; see the intake-valve comment in
     * EngineSimulator.cpp and docs/physics-audit.md. Published because it is the
     * quantity to watch when evaluating any future ram mechanism. */
    double intakePortRamPressureKpa { 0.0 };
    /** Runner column velocity at the valve plane, u = mdot_valve/(rho*A_runner).
     *
     * Referred to the runner cross-section, so it is the speed of the gas column
     * whose momentum `intakePortRamPressureKpa` differentiates. Distinct from
     * `intakeVelocityMps` (the valve-throat speed) and from the runner cell's own
     * bulk velocity, which does not collapse at IVC. */
    double intakePortColumnVelocityMps { 0.0 };
    /** Charge state in this cylinder's own intake runner, at the valve.
     *
     * What an IAT sensor in the port would read, and the direct determinant of
     * charge density and therefore of volumetric efficiency. Reported per
     * cylinder rather than per plenum because reversion through the intake
     * valve heats one runner at a time. */
    double intakeRunnerTemperatureC { 22.0 };
    double intakeRunnerChargePressureKpa { 101.325 };
    /** True when heat release is governed by compression ignition instead of
     *  a spark-initiated flame front.
     */
    bool compressionIgnition { false };
    /** Resolved start of combustion in cylinder-cycle coordinates, where
     *  firing TDC is zero. Negative means before firing TDC.
     */
    double combustionStartPhaseDegrees { 0.0 };
    double combustionDurationMs { 0.0 };
    /** Normalised initial pressure-rise propensity, used by diagnostics and
     *  the physically distinct diesel combustion-noise renderer.
     */
    double combustionSharpness { 0.0 };
    double directLiquidSprayFuelMg { 0.0 };
    double directDispersingFuelMg { 0.0 };
    /** Current per-cylinder physical burn-rate multiplier. Unlike the legacy
     * event-generator jitter, this acts before pressure and heat release. */
    double combustionCycleMultiplier { 1.0 };
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
    /**
     * NOT back pressure. This is the `max` over cylinder ports of the
     * INSTANTANEOUS exhaust-runner pressure -- a blowdown peak envelope. It
     * reads like a mean and is not one: on a healthy V8 at 5,940 rpm it sits at
     * 172 kPa while the gas is discharging freely. Audio and telemetry want the
     * peak, so it stays; any back-pressure, pumping or restriction argument
     * built on it is void. Use `exhaustBackPressureKpa` for that.
     */
    double exhaustPressureKpa { 101.325 };
    /**
     * Cycle-mean exhaust back pressure: the port pressures averaged over the
     * cylinders and then damped over several firing periods, which is what a
     * real back-pressure gauge on a manifold boss reads. This is the quantity a
     * restriction warning must be built on.
     */
    double exhaustBackPressureKpa { 101.325 };
    double intakeRunnerPressureKpa { 101.325 };
    double exhaustRunnerPressureKpa { 101.325 };
    double exhaustFlowGramsPerSecond { 0.0 };
    /** Real chemical heat release occurring in the physical exhaust network. */
    double exhaustAfterfireHeatReleaseKw { 0.0 };
    double exhaustAfterfireFuelBurnMgPerSecond { 0.0 };
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
    /**
     * Charge mass that actually crossed the intake valves last cycle.
     *
     * Distinct from `airMassMgPerCycle`, which is the oxygen-equivalent air
     * TRAPPED in the chambers -- the right quantity to meter fuel against,
     * because it is what can burn, but not a measure of breathing: a chamber
     * holding unburned oxygen (a misfire, a motored cylinder, or a cam with no
     * lift at all) reports a full charge it never inducted. Measured on a
     * zero-lift engine that the starter can now turn: `volumetricEfficiency`
     * reads 1.031 while nothing whatsoever passes the valve. Induction is the
     * quantity that is genuinely zero there.
     *
     * Note this is total charge mass, not air alone -- with port injection it
     * carries the fuel vapour picked up in the runner -- so it is NOT a drop-in
     * numerator for volumetric efficiency. See docs/physics-audit.md.
     */
    double inductedChargeMassMgPerCycle { 0.0 };
    /**
     * Fresh air DELIVERED past the intake valves last cycle, and the volumetric
     * efficiency built on it.
     *
     * `volumetricEfficiency` above is a *trapping* figure: oxygen-equivalent air
     * still in the chamber at IVC. Heywood's definition (ch. 2.10) is the mass
     * of fresh air *inducted* per cycle over the ambient-density displacement --
     * a delivery figure, and the one a dyno's air meter reads. The two differ by
     * exactly the fresh charge that entered and was then pushed back out before
     * the valve shut, so
     *
     *     trapping efficiency = volumetricEfficiency / deliveredVolumetricEfficiency
     *
     * and a ratio below 1 is charge the engine paid to accelerate and did not
     * keep. That is not a bookkeeping curiosity: it is the direct per-cylinder
     * read-out of the overlap backflow that the pumping loop
     * (`pumpingMeanEffectivePressureBar`) shows in aggregate.
     *
     * Both are oxygen-derived, so both are fresh air on the same basis and their
     * ratio is meaningful. Do NOT use `inductedChargeMassMgPerCycle` as the
     * numerator instead -- it is total charge and carries port fuel vapour.
     *
     * A reverting cylinder can drive the net delivery negative over a cycle; the
     * figure is published signed for that reason, and only the VE built on it is
     * clamped non-negative.
     */
    double deliveredAirMassMgPerCycle { 0.0 };
    double deliveredVolumetricEfficiency { 0.0 };
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
    /**
     * The two loops of the indicated diagram, published separately.
     *
     * `indicatedMeanEffectivePressureBar` above is the NET figure and already
     * contains the gas-exchange loop; these two split it, and
     * gross + pumping == net up to the difference between the engine-level
     * torque accumulation and the per-cylinder p-dV integrals they come from.
     *
     * `pumpingMeanEffectivePressureBar` is SIGNED, not a loss magnitude:
     * negative is the normal case (the piston spends work moving gas), and a
     * supercharged engine whose intake stroke is pressurised can legitimately
     * read positive. At wide-open throttle a healthy naturally aspirated engine
     * sits around -0.2 to -0.6 bar, the loss growing with speed; a throttled
     * one reaches -0.8 bar or worse, which is where the throttling loss lives.
     *
     * This is the number that exposes a gas-exchange failure. A cycle-averaged
     * IMEP cannot: an engine that cannot evacuate its cylinder reports a
     * perfectly healthy IMEP while paying twice, once in pumping work and once
     * in the residual it re-inducts on the next intake stroke.
     */
    double pumpingMeanEffectivePressureBar { 0.0 };
    /**
     * The exhaust-stroke half of `pumpingMeanEffectivePressureBar`; the intake
     * half is the remainder (`pumping - exhaustStroke`). Same sign convention.
     *
     * A single PMEP figure says an engine pumps too hard but not which side is
     * at fault, and the two have unrelated causes: the exhaust half is back
     * pressure (a duct, a collector, an exhaust-valve area, a terminal boundary
     * condition), the intake half is depression (a plate, a runner, an
     * intake-valve area). They are also of opposite physical sign at the pump --
     * the exhaust stroke pushes against a positive gauge pressure while the
     * intake stroke pulls against a negative one -- so a pair of errors on the
     * two sides can partly cancel in the total and hide from a PMEP gate.
     */
    double exhaustStrokeMeanEffectivePressureBar { 0.0 };
    double grossIndicatedMeanEffectivePressureBar { 0.0 };
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
    double drivenAxleNormalForceN { 0.0 };
    double longitudinalAccelerationMps2 { 0.0 };
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
