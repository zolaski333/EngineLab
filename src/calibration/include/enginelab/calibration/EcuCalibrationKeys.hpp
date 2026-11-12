#pragma once

#include <string_view>

namespace enginelab::calibration::keys {

inline constexpr std::string_view targetAirFuelRatio = "fuel.target_afr";
inline constexpr std::string_view dieselFuelQuantityMgPerCycle =
    "fuel.diesel_quantity_mg_per_cycle";
inline constexpr std::string_view ignitionAdvance = "ignition.advance_deg";
inline constexpr std::string_view volumetricEfficiency = "fuel.volumetric_efficiency";
inline constexpr std::string_view variableValveTiming = "valvetrain.vvt_deg";
inline constexpr std::string_view variableValveLift = "valvetrain.vvl_mm";
// Turbocharged engines: the boost the wastegate controller holds, gauge kPa
// by engine speed. Absent (supercharger, saved tables without it): the
// configuration's wastegate pressure ratio.
inline constexpr std::string_view boostTargetGauge = "boost.target_gauge_kpa";
inline constexpr std::string_view revLimit = "limits.rev_rpm";
// Launch control (two-step): the rev limit while the vehicle stands in gear
// under throttle. Zero: off.
inline constexpr std::string_view launchLimit = "limits.launch_rpm";
// Turbocharged engines: the fuel is cut while the manifold's gauge pressure
// runs this far past the boost target, and given back at half of it. Zero:
// off. Absent (saved tables from before it): the default margin.
inline constexpr std::string_view overboostCut = "limits.overboost_kpa";
inline constexpr std::string_view knockRetardGain = "knock.retard_deg_per_intensity";
// The speed the idle governor holds with the throttle shut. Absent (a
// carburettor, whose idle stop screw sets it, or saved tables without it):
// the configuration's idle speed.
inline constexpr std::string_view idleTarget = "idle.target_rpm";
// VTEC, on an engine whose camshafts carry a high cam (a rotary's staged
// intake ports): the ECU's switch, 1 on and 0 off; the speed the high cam
// engages above (whole shaft cycles above it); how far below that it drops
// back; and the throttle under which it never runs, in percent. Absent
// (tables saved before them): on, at each camshaft's own `switch_rpm` and
// `switch_throttle`, with no hysteresis.
inline constexpr std::string_view vtecEnabled = "vtec.enabled";
inline constexpr std::string_view vtecEngage = "vtec.engage_rpm";
inline constexpr std::string_view vtecHysteresis = "vtec.hysteresis_rpm";
inline constexpr std::string_view vtecMinimumThrottle = "vtec.min_throttle_percent";
// Anti-lag, on a turbocharged petrol engine: the switch, 1 armed and 0 off
// (the default); the speed it works above; the throttle it holds open, in
// percent; how late the spark fires, degrees after top dead centre; the
// mixture, as a lambda; and the share of firings whose spark is cut with the
// fuel kept, in percent. It works with the driver's foot off the throttle:
// the late and the unlit charges burn in the manifold, before the turbine.
inline constexpr std::string_view antiLagEnabled = "antilag.enabled";
inline constexpr std::string_view antiLagMinimumRpm = "antilag.min_rpm";
inline constexpr std::string_view antiLagAir = "antilag.air_percent";
inline constexpr std::string_view antiLagRetard = "antilag.spark_atdc_deg";
inline constexpr std::string_view antiLagLambda = "antilag.lambda";
inline constexpr std::string_view antiLagCut = "antilag.cut_percent";

} // namespace enginelab::calibration::keys

namespace enginelab::calibration::ecuLimits {

// These are hard runtime limits as well as editor/serialization limits. Keep
// one contract so a value accepted by the tuner is never silently clamped to a
// different range by the ECU.
// Petrol AFR (lambda x 14.7). Lambda 0.61, past the rich-misfire onset at
// 0.68: an alcohol map may run rich and a tuner may find that limit.
inline constexpr double minimumAirFuelRatio = 9.0;
// The richest cell a generated (default) map holds, unchanged since the
// limit above was 10.5: the shipped engines keep their maps.
inline constexpr double minimumDefaultMapAirFuelRatio = 10.5;
inline constexpr double maximumGasolineAirFuelRatio = 18.0;
inline constexpr double maximumDieselSmokeLimitAirFuelRatio = 40.0;
inline constexpr double maximumAirFuelRatio =
    maximumDieselSmokeLimitAirFuelRatio;
inline constexpr double minimumDieselFuelQuantityMgPerCycle = 0.0;
inline constexpr double maximumDieselFuelQuantityMgPerCycle = 125'000.0;
inline constexpr double minimumIgnitionAdvanceDegrees = -10.0;
inline constexpr double maximumIgnitionAdvanceDegrees = 55.0;
inline constexpr double minimumIdleTargetRpm = 50.0;
inline constexpr double maximumIdleTargetRpm = 3'000.0;
inline constexpr double minimumRevLimitRpm = 150.0;
inline constexpr double maximumRevLimitRpm = 25'000.0;
// Knock control: the most spark it may pull (0 = control off, the knock runs
// free). The key keeps its first name, from when the retard was proportional.
inline constexpr double minimumKnockRetardGainDegrees = 0.0;
inline constexpr double maximumKnockRetardGainDegrees = 40.0;
inline constexpr double defaultKnockRetardGainDegrees = 12.0;
// Gauge boost: the configuration's pressure ratio stops at 6 (507 kPa).
inline constexpr double minimumBoostTargetGaugeKpa = 0.0;
inline constexpr double maximumBoostTargetGaugeKpa = 510.0;
// The overboost cut's margin over the target, off by default: a held boost
// overshoots its target by at most 6 kPa (EngineLab.CatalogBoostHold), but the
// catalogue 2JZ's spool-up on the dyno runs more than 40 kPa past it for 1 s
// of a sweep (journal 2026-11-11), so a cut on by default would change it.
inline constexpr double defaultOverboostCutMarginKpa = 0.0;
inline constexpr double maximumOverboostCutMarginKpa = 1'000.0;
// VTEC: any speed a rev limiter can be set to, a hysteresis as large, and the
// whole pedal. No other bound: a high cam engaged below idle, or never, is the
// tuner's choice.
inline constexpr double maximumVtecHysteresisRpm = maximumRevLimitRpm;
inline constexpr double maximumVtecThrottlePercent = 100.0;
// Anti-lag: the spark may fire up to 90 deg after top dead centre (the
// spark schedule carries far more); its mixture is a lambda.
inline constexpr double maximumAntiLagSparkAtdcDegrees = 90.0;
inline constexpr double minimumAntiLagLambda = 0.5;
inline constexpr double maximumAntiLagLambda = 1.5;
// Above this driver's throttle anti-lag lets go: the driver drives.
inline constexpr double antiLagMaximumDriverThrottle = 0.05;
inline constexpr double minimumNormalizedLoad = 0.0;
inline constexpr double maximumNormalizedLoad = 4.0;
inline constexpr double minimumAirFuelRatioTrim = -3.0;
inline constexpr double maximumAirFuelRatioTrim = 15.0;
inline constexpr double referenceAirFuelRatio = 14.2;
// A spark engine's AFR map and limits are petrol AFR: lambda x this. The ECU
// meters another fuel at the same lambda (its stoichiometric ratio / this).
inline constexpr double petrolStoichiometricAirFuelRatio = 14.7;

} // namespace enginelab::calibration::ecuLimits
