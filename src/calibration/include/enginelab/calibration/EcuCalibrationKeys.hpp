#pragma once

#include <string_view>

namespace enginelab::calibration::keys {

inline constexpr std::string_view targetAirFuelRatio = "fuel.target_afr";
inline constexpr std::string_view ignitionAdvance = "ignition.advance_deg";
inline constexpr std::string_view volumetricEfficiency = "fuel.volumetric_efficiency";
inline constexpr std::string_view variableValveTiming = "valvetrain.vvt_deg";
inline constexpr std::string_view variableValveLift = "valvetrain.vvl_mm";
inline constexpr std::string_view boostTarget = "boost.target_kpa";
inline constexpr std::string_view wastegateDuty = "boost.wastegate_duty";
inline constexpr std::string_view revLimit = "limits.rev_rpm";

} // namespace enginelab::calibration::keys

namespace enginelab::calibration::ecuLimits {

// These are hard runtime limits as well as editor/serialization limits. Keep
// one contract so a value accepted by the tuner is never silently clamped to a
// different range by the ECU.
inline constexpr double minimumAirFuelRatio = 10.5;
inline constexpr double maximumAirFuelRatio = 18.0;
inline constexpr double minimumIgnitionAdvanceDegrees = -10.0;
inline constexpr double maximumIgnitionAdvanceDegrees = 55.0;
inline constexpr double minimumRevLimitRpm = 600.0;
inline constexpr double maximumRevLimitRpm = 25'000.0;
inline constexpr double minimumNormalizedLoad = 0.0;
inline constexpr double maximumNormalizedLoad = 4.0;
inline constexpr double minimumAirFuelRatioTrim = -3.0;
inline constexpr double maximumAirFuelRatioTrim = 3.0;
inline constexpr double referenceAirFuelRatio = 14.2;

} // namespace enginelab::calibration::ecuLimits
