#include <enginelab/calibration/CalibrationTypes.hpp>
#include <enginelab/calibration/EcuCalibrationKeys.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <type_traits>

namespace enginelab::calibration {
namespace {

bool isValidIdentifier(std::string_view id) noexcept {
    if (id.empty()) return false;
    const auto first = static_cast<unsigned char>(id.front());
    if (std::isalpha(first) == 0 && id.front() != '_') return false;
    return std::all_of(id.begin() + 1, id.end(), [](char character) {
        const auto value = static_cast<unsigned char>(character);
        return std::isalnum(value) != 0 || character == '_' || character == '-' || character == '.';
    });
}

void addIssue(std::vector<CalibrationIssue>& issues, CalibrationErrorCode code,
              std::string path, std::string message) {
    issues.push_back({ code, std::move(path), std::move(message) });
}

bool validateLimits(const CalibrationLimits& limits, std::string_view path,
                    std::vector<CalibrationIssue>& issues) {
    auto valid = true;
    if (limits.minimum && !std::isfinite(*limits.minimum)) {
        addIssue(issues, CalibrationErrorCode::invalidMetadata, std::string(path) + ".minimum",
                 "The lower hard limit must be finite.");
        valid = false;
    }
    if (limits.maximum && !std::isfinite(*limits.maximum)) {
        addIssue(issues, CalibrationErrorCode::invalidMetadata, std::string(path) + ".maximum",
                 "The upper hard limit must be finite.");
        valid = false;
    }
    if (limits.minimum && limits.maximum && *limits.minimum > *limits.maximum) {
        addIssue(issues, CalibrationErrorCode::invalidMetadata, std::string(path),
                 "The lower hard limit must not exceed the upper hard limit.");
        valid = false;
    }
    return valid;
}

bool validateMetadata(const CalibrationMetadata& metadata, std::string_view path,
                      std::vector<CalibrationIssue>& issues) {
    auto valid = true;
    if (!isValidIdentifier(metadata.id)) {
        addIssue(issues, CalibrationErrorCode::invalidIdentifier, std::string(path) + ".id",
                 "Calibration identifiers must start with a letter or underscore and contain only letters, digits, '.', '-' or '_'.");
        valid = false;
    }
    if (metadata.displayName.empty()) {
        addIssue(issues, CalibrationErrorCode::invalidMetadata, std::string(path) + ".display_name",
                 "A display name is required.");
        valid = false;
    }
    if (metadata.displayPrecision < 0 || metadata.displayPrecision > 9) {
        addIssue(issues, CalibrationErrorCode::invalidMetadata, std::string(path) + ".display_precision",
                 "Display precision must be in the range 0 to 9.");
        valid = false;
    }
    return validateLimits(metadata.hardLimits, std::string(path) + ".limits", issues) && valid;
}

bool validateAxis(const CalibrationAxis& axis, std::string_view path,
                  std::vector<CalibrationIssue>& issues) {
    auto valid = true;
    if (!isValidIdentifier(axis.metadata.id)) {
        addIssue(issues, CalibrationErrorCode::invalidIdentifier, std::string(path) + ".id",
                 "Axis identifiers follow the same rules as calibration identifiers.");
        valid = false;
    }
    if (axis.metadata.displayName.empty()) {
        addIssue(issues, CalibrationErrorCode::invalidAxis, std::string(path) + ".display_name",
                 "An axis display name is required.");
        valid = false;
    }
    if (!isUnitCompatible(axis.metadata.quantity, axis.metadata.unit)) {
        addIssue(issues, CalibrationErrorCode::incompatibleUnit, std::string(path) + ".unit",
                 "The selected unit is incompatible with the axis quantity.");
        valid = false;
    }
    if (axis.breakpoints.empty()) {
        addIssue(issues, CalibrationErrorCode::invalidAxis, std::string(path) + ".breakpoints",
                 "An axis needs at least one breakpoint.");
        return false;
    }
    for (std::size_t index = 0; index < axis.breakpoints.size(); ++index) {
        if (!std::isfinite(axis.breakpoints[index])) {
            addIssue(issues, CalibrationErrorCode::invalidAxis,
                     std::string(path) + ".breakpoints[" + std::to_string(index) + "]",
                     "Axis breakpoints must be finite.");
            valid = false;
        }
        if (index > 0 && !(axis.breakpoints[index] > axis.breakpoints[index - 1])) {
            addIssue(issues, CalibrationErrorCode::invalidAxis,
                     std::string(path) + ".breakpoints[" + std::to_string(index) + "]",
                     "Axis breakpoints must be strictly increasing and unique.");
            valid = false;
        }
    }
    return valid;
}

void validateValues(const CalibrationMetadata& metadata, const std::vector<double>& values,
                    std::string_view path, std::vector<CalibrationIssue>& issues) {
    for (std::size_t index = 0; index < values.size(); ++index) {
        const auto value = values[index];
        const auto valuePath = std::string(path) + "[" + std::to_string(index) + "]";
        if (!std::isfinite(value)) {
            addIssue(issues, CalibrationErrorCode::invalidValue, valuePath,
                     "Calibration values must be finite.");
        } else if (metadata.hardLimits.minimum && value < *metadata.hardLimits.minimum) {
            addIssue(issues, CalibrationErrorCode::invalidValue, valuePath,
                     "The value is below its hard lower limit.");
        } else if (metadata.hardLimits.maximum && value > *metadata.hardLimits.maximum) {
            addIssue(issues, CalibrationErrorCode::invalidValue, valuePath,
                     "The value is above its hard upper limit.");
        }
    }
}

void validateValue(const CalibrationMetadata& metadata, double value, std::string_view path,
                   std::vector<CalibrationIssue>& issues) {
    validateValues(metadata, std::vector<double> { value }, path, issues);
}

bool hasCanonicalLimits(const CalibrationMetadata& metadata, double minimum, double maximum) noexcept {
    return metadata.hardLimits.minimum && metadata.hardLimits.maximum
        && *metadata.hardLimits.minimum >= minimum
        && *metadata.hardLimits.maximum <= maximum;
}

void validateCanonicalLimits(const CalibrationMetadata& metadata, double minimum, double maximum,
                             std::string_view path, std::vector<CalibrationIssue>& issues) {
    if (!hasCanonicalLimits(metadata, minimum, maximum)) {
        addIssue(issues, CalibrationErrorCode::invalidMetadata, std::string(path) + ".limits",
                 "Known ECU calibrations require hard limits contained in the runtime range ["
                     + std::to_string(minimum) + ", " + std::to_string(maximum) + "].");
    }
}

void validateRpmAxisContract(const CalibrationAxis& axis, std::string_view path,
                             std::vector<CalibrationIssue>& issues) {
    if (axis.metadata.quantity != AxisQuantity::engineSpeed
            || axis.metadata.unit != CalibrationUnit::revolutionsPerMinute) {
        addIssue(issues, CalibrationErrorCode::invalidAxis, std::string(path),
                 "This ECU axis must be engine speed expressed in rpm.");
    }
    if (std::any_of(axis.breakpoints.begin(), axis.breakpoints.end(), [](double value) {
            return value < 0.0 || value > ecuLimits::maximumRevLimitRpm;
        })) {
        addIssue(issues, CalibrationErrorCode::invalidAxis, std::string(path) + ".breakpoints",
                 "ECU speed breakpoints must remain between 0 and 25000 rpm.");
    }
}

void validateLoadAxisContract(const CalibrationAxis& axis, std::string_view path,
                              std::vector<CalibrationIssue>& issues) {
    if (axis.metadata.quantity != AxisQuantity::normalizedLoad
            || axis.metadata.unit != CalibrationUnit::ratio) {
        addIssue(issues, CalibrationErrorCode::invalidAxis, std::string(path),
                 "This ECU axis must be normalized absolute load expressed as a ratio.");
    }
    if (std::any_of(axis.breakpoints.begin(), axis.breakpoints.end(), [](double value) {
            return value < ecuLimits::minimumNormalizedLoad
                || value > ecuLimits::maximumNormalizedLoad;
        })) {
        addIssue(issues, CalibrationErrorCode::invalidAxis, std::string(path) + ".breakpoints",
                 "Normalized-load breakpoints must remain between 0 and 4 (0 to 400%).");
    }
}

void validateMappedEcuEntry(const CalibrationEntry& entry, CalibrationUnit unit,
                            double minimum, double maximum, std::string_view path,
                            std::vector<CalibrationIssue>& issues) {
    const auto& metadata = metadataOf(entry);
    if (metadata.unit != unit) {
        addIssue(issues, CalibrationErrorCode::incompatibleUnit, std::string(path) + ".unit",
                 "The known ECU key has a fixed output unit.");
    }
    validateCanonicalLimits(metadata, minimum, maximum, path, issues);
    if (const auto* curve = std::get_if<CalibrationCurve1D>(&entry)) {
        validateRpmAxisContract(curve->axis, std::string(path) + ".axis", issues);
    } else if (const auto* table = std::get_if<CalibrationTable2D>(&entry)) {
        validateRpmAxisContract(table->xAxis, std::string(path) + ".x_axis", issues);
        validateLoadAxisContract(table->yAxis, std::string(path) + ".y_axis", issues);
    } else {
        addIssue(issues, CalibrationErrorCode::invalidDimensions, std::string(path),
                 "This ECU key must be a speed curve or a speed/load table.");
    }
}

void validateRpmCurveEcuEntry(const CalibrationEntry& entry,
                              CalibrationUnit unit, double minimum,
                              double maximum, std::string_view path,
                              std::vector<CalibrationIssue>& issues) {
    const auto& metadata = metadataOf(entry);
    if (metadata.unit != unit) {
        addIssue(issues, CalibrationErrorCode::incompatibleUnit,
                 std::string(path) + ".unit",
                 "The known ECU key has a fixed output unit.");
    }
    validateCanonicalLimits(metadata, minimum, maximum, path, issues);
    if (const auto* curve = std::get_if<CalibrationCurve1D>(&entry)) {
        validateRpmAxisContract(curve->axis, std::string(path) + ".axis", issues);
    } else {
        addIssue(issues, CalibrationErrorCode::invalidDimensions,
                 std::string(path),
                 "This ECU key must be an engine-speed curve.");
    }
}

void validateKnownEcuEntry(std::string_view id, const CalibrationEntry& entry,
                           std::string_view path, std::vector<CalibrationIssue>& issues) {
    if (id == keys::targetAirFuelRatio) {
        validateMappedEcuEntry(entry, CalibrationUnit::airFuelRatio,
            ecuLimits::minimumAirFuelRatio, ecuLimits::maximumAirFuelRatio, path, issues);
    } else if (id == keys::dieselFuelQuantityMgPerCycle) {
        validateRpmCurveEcuEntry(entry, CalibrationUnit::milligram,
            ecuLimits::minimumDieselFuelQuantityMgPerCycle,
            ecuLimits::maximumDieselFuelQuantityMgPerCycle, path, issues);
    } else if (id == keys::ignitionAdvance) {
        validateMappedEcuEntry(entry, CalibrationUnit::degreeCrankshaft,
            ecuLimits::minimumIgnitionAdvanceDegrees,
            ecuLimits::maximumIgnitionAdvanceDegrees, path, issues);
    } else if (id == keys::revLimit) {
        const auto& metadata = metadataOf(entry);
        if (metadata.unit != CalibrationUnit::revolutionsPerMinute) {
            addIssue(issues, CalibrationErrorCode::incompatibleUnit, std::string(path) + ".unit",
                     "The rev limiter must be expressed in rpm.");
        }
        validateCanonicalLimits(metadata, ecuLimits::minimumRevLimitRpm,
                                ecuLimits::maximumRevLimitRpm, path, issues);
        if (!std::holds_alternative<ScalarCalibration>(entry)) {
            addIssue(issues, CalibrationErrorCode::invalidDimensions, std::string(path),
                     "The rev limiter must be a scalar calibration.");
        }
    }
}

} // namespace

std::string_view toString(CalibrationErrorCode code) noexcept {
    switch (code) {
        case CalibrationErrorCode::invalidDocument: return "invalid_document";
        case CalibrationErrorCode::invalidIdentifier: return "invalid_identifier";
        case CalibrationErrorCode::invalidMetadata: return "invalid_metadata";
        case CalibrationErrorCode::invalidAxis: return "invalid_axis";
        case CalibrationErrorCode::incompatibleUnit: return "incompatible_unit";
        case CalibrationErrorCode::invalidValue: return "invalid_value";
        case CalibrationErrorCode::invalidDimensions: return "invalid_dimensions";
        case CalibrationErrorCode::duplicateCalibration: return "duplicate_calibration";
        case CalibrationErrorCode::unsupportedSchema: return "unsupported_schema";
        case CalibrationErrorCode::parseError: return "parse_error";
        case CalibrationErrorCode::revisionConflict: return "revision_conflict";
        case CalibrationErrorCode::revisionOverflow: return "revision_overflow";
    }
    return "invalid_document";
}

std::string_view toString(CalibrationUnit unit) noexcept {
    switch (unit) {
        case CalibrationUnit::dimensionless: return "dimensionless";
        case CalibrationUnit::ratio: return "ratio";
        case CalibrationUnit::percent: return "percent";
        case CalibrationUnit::revolutionsPerMinute: return "rpm";
        case CalibrationUnit::kilopascal: return "kPa";
        case CalibrationUnit::bar: return "bar";
        case CalibrationUnit::degreeCelsius: return "degC";
        case CalibrationUnit::degreeCrankshaft: return "deg_crank";
        case CalibrationUnit::millisecond: return "ms";
        case CalibrationUnit::second: return "s";
        case CalibrationUnit::lambda: return "lambda";
        case CalibrationUnit::airFuelRatio: return "afr";
        case CalibrationUnit::newtonMetre: return "Nm";
        case CalibrationUnit::gramPerSecond: return "g_s";
        case CalibrationUnit::milligram: return "mg";
        case CalibrationUnit::kilometrePerHour: return "km_h";
        case CalibrationUnit::hertz: return "Hz";
    }
    return "dimensionless";
}

std::string_view toString(AxisQuantity quantity) noexcept {
    switch (quantity) {
        case AxisQuantity::generic: return "generic";
        case AxisQuantity::engineSpeed: return "engine_speed";
        case AxisQuantity::manifoldAbsolutePressure: return "manifold_absolute_pressure";
        case AxisQuantity::normalizedLoad: return "normalized_load";
        case AxisQuantity::throttlePosition: return "throttle_position";
        case AxisQuantity::acceleratorPosition: return "accelerator_position";
        case AxisQuantity::coolantTemperature: return "coolant_temperature";
        case AxisQuantity::intakeAirTemperature: return "intake_air_temperature";
        case AxisQuantity::lambda: return "lambda";
        case AxisQuantity::vehicleSpeed: return "vehicle_speed";
        case AxisQuantity::elapsedTime: return "elapsed_time";
    }
    return "generic";
}

std::optional<CalibrationUnit> calibrationUnitFromString(std::string_view value) noexcept {
    constexpr CalibrationUnit units[] {
        CalibrationUnit::dimensionless, CalibrationUnit::ratio, CalibrationUnit::percent,
        CalibrationUnit::revolutionsPerMinute, CalibrationUnit::kilopascal, CalibrationUnit::bar,
        CalibrationUnit::degreeCelsius, CalibrationUnit::degreeCrankshaft, CalibrationUnit::millisecond,
        CalibrationUnit::second, CalibrationUnit::lambda, CalibrationUnit::airFuelRatio,
        CalibrationUnit::newtonMetre, CalibrationUnit::gramPerSecond, CalibrationUnit::milligram,
        CalibrationUnit::kilometrePerHour, CalibrationUnit::hertz
    };
    for (const auto unit : units) if (toString(unit) == value) return unit;
    return std::nullopt;
}

std::optional<AxisQuantity> axisQuantityFromString(std::string_view value) noexcept {
    constexpr AxisQuantity quantities[] {
        AxisQuantity::generic, AxisQuantity::engineSpeed, AxisQuantity::manifoldAbsolutePressure,
        AxisQuantity::normalizedLoad, AxisQuantity::throttlePosition, AxisQuantity::acceleratorPosition,
        AxisQuantity::coolantTemperature, AxisQuantity::intakeAirTemperature, AxisQuantity::lambda,
        AxisQuantity::vehicleSpeed, AxisQuantity::elapsedTime
    };
    for (const auto quantity : quantities) if (toString(quantity) == value) return quantity;
    return std::nullopt;
}

bool isUnitCompatible(AxisQuantity quantity, CalibrationUnit unit) noexcept {
    switch (quantity) {
        case AxisQuantity::generic: return true;
        case AxisQuantity::engineSpeed: return unit == CalibrationUnit::revolutionsPerMinute;
        case AxisQuantity::manifoldAbsolutePressure:
            return unit == CalibrationUnit::kilopascal || unit == CalibrationUnit::bar;
        case AxisQuantity::normalizedLoad:
        case AxisQuantity::throttlePosition:
        case AxisQuantity::acceleratorPosition:
            return unit == CalibrationUnit::ratio || unit == CalibrationUnit::percent;
        case AxisQuantity::coolantTemperature:
        case AxisQuantity::intakeAirTemperature:
            return unit == CalibrationUnit::degreeCelsius;
        case AxisQuantity::lambda:
            return unit == CalibrationUnit::lambda || unit == CalibrationUnit::ratio;
        case AxisQuantity::vehicleSpeed: return unit == CalibrationUnit::kilometrePerHour;
        case AxisQuantity::elapsedTime:
            return unit == CalibrationUnit::second || unit == CalibrationUnit::millisecond;
    }
    return false;
}

AxisInterval CalibrationAxis::locateClamped(double coordinate) const noexcept {
    if (breakpoints.size() <= 1 || !std::isfinite(coordinate) || coordinate <= breakpoints.front()) {
        return { 0, 0, 0.0 };
    }
    if (coordinate >= breakpoints.back()) {
        const auto last = breakpoints.size() - 1;
        return { last, last, 0.0 };
    }
    const auto upper = std::upper_bound(breakpoints.begin(), breakpoints.end(), coordinate);
    const auto upperIndex = static_cast<std::size_t>(upper - breakpoints.begin());
    const auto lowerIndex = upperIndex - 1;
    const auto span = breakpoints[upperIndex] - breakpoints[lowerIndex];
    return { lowerIndex, upperIndex,
             std::clamp((coordinate - breakpoints[lowerIndex]) / span, 0.0, 1.0) };
}

bool CalibrationAxis::accepts(const AxisCoordinate& coordinate) const noexcept {
    return coordinate.quantity == metadata.quantity && coordinate.unit == metadata.unit
        && std::isfinite(coordinate.value);
}

double CalibrationCurve1D::sampleClamped(double coordinate) const noexcept {
    if (values.empty()) return 0.0;
    const auto interval = axis.locateClamped(coordinate);
    if (interval.lowerIndex == interval.upperIndex) return values[interval.lowerIndex];
    return std::lerp(values[interval.lowerIndex], values[interval.upperIndex], interval.fraction);
}

std::optional<double> CalibrationCurve1D::sample(const AxisCoordinate& coordinate) const noexcept {
    if (!axis.accepts(coordinate)) return std::nullopt;
    return sampleClamped(coordinate.value);
}

double CalibrationTable2D::sampleClamped(double xCoordinate, double yCoordinate) const noexcept {
    if (values.empty() || xAxis.breakpoints.empty()) return 0.0;
    const auto x = xAxis.locateClamped(xCoordinate);
    const auto y = yAxis.locateClamped(yCoordinate);
    const auto width = xAxis.breakpoints.size();
    const auto at = [this, width](std::size_t xIndex, std::size_t yIndex) noexcept {
        return values[yIndex * width + xIndex];
    };
    const auto lower = std::lerp(at(x.lowerIndex, y.lowerIndex), at(x.upperIndex, y.lowerIndex), x.fraction);
    const auto upper = std::lerp(at(x.lowerIndex, y.upperIndex), at(x.upperIndex, y.upperIndex), x.fraction);
    return std::lerp(lower, upper, y.fraction);
}

std::optional<double> CalibrationTable2D::sample(const AxisCoordinate& xCoordinate,
                                                  const AxisCoordinate& yCoordinate) const noexcept {
    if (!xAxis.accepts(xCoordinate) || !yAxis.accepts(yCoordinate)) return std::nullopt;
    return sampleClamped(xCoordinate.value, yCoordinate.value);
}

const CalibrationMetadata& metadataOf(const CalibrationEntry& entry) noexcept {
    return std::visit([](const auto& calibration) -> const CalibrationMetadata& {
        return calibration.metadata;
    }, entry);
}

void CalibrationDraft::set(CalibrationEntry entry) {
    auto id = metadataOf(entry).id;
    entries_.insert_or_assign(std::move(id), std::move(entry));
}

bool CalibrationDraft::erase(std::string_view id) {
    const auto found = entries_.find(id);
    if (found == entries_.end()) return false;
    entries_.erase(found);
    return true;
}

const CalibrationEntry* CalibrationDraft::find(std::string_view id) const noexcept {
    const auto found = entries_.find(id);
    return found == entries_.end() ? nullptr : &found->second;
}

std::vector<CalibrationIssue> validate(const CalibrationDraft& draft) {
    std::vector<CalibrationIssue> issues;
    for (const auto& [id, entry] : draft.entries()) {
        const auto path = std::string("calibrations.") + (id.empty() ? "<empty>" : id);
        const auto& metadata = metadataOf(entry);
        validateMetadata(metadata, path, issues);
        if (id != metadata.id) {
            addIssue(issues, CalibrationErrorCode::invalidIdentifier, path + ".id",
                     "The calibration map key and metadata identifier do not match.");
        }
        std::visit([&](const auto& calibration) {
            using T = std::decay_t<decltype(calibration)>;
            if constexpr (std::is_same_v<T, ScalarCalibration>) {
                validateValue(calibration.metadata, calibration.value, path + ".value", issues);
            } else if constexpr (std::is_same_v<T, CalibrationCurve1D>) {
                validateAxis(calibration.axis, path + ".axis", issues);
                if (calibration.values.size() != calibration.axis.breakpoints.size()) {
                    addIssue(issues, CalibrationErrorCode::invalidDimensions, path + ".values",
                             "A 1D curve needs exactly one value per axis breakpoint.");
                }
                validateValues(calibration.metadata, calibration.values, path + ".values", issues);
            } else {
                validateAxis(calibration.xAxis, path + ".x_axis", issues);
                validateAxis(calibration.yAxis, path + ".y_axis", issues);
                const auto xSize = calibration.xAxis.breakpoints.size();
                const auto ySize = calibration.yAxis.breakpoints.size();
                const auto overflow = xSize != 0 && ySize > std::numeric_limits<std::size_t>::max() / xSize;
                if (overflow || calibration.values.size() != xSize * ySize) {
                    addIssue(issues, CalibrationErrorCode::invalidDimensions, path + ".values",
                             "A 2D table needs x-axis size multiplied by y-axis size values, stored row-major.");
                }
                validateValues(calibration.metadata, calibration.values, path + ".values", issues);
            }
        }, entry);
        validateKnownEcuEntry(id, entry, path, issues);
    }
    return issues;
}

} // namespace enginelab::calibration
