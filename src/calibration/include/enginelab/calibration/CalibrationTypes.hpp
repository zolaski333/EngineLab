#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

namespace enginelab::calibration {

enum class CalibrationErrorCode {
    invalidDocument,
    invalidIdentifier,
    invalidMetadata,
    invalidAxis,
    incompatibleUnit,
    invalidValue,
    invalidDimensions,
    duplicateCalibration,
    unsupportedSchema,
    parseError,
    revisionConflict,
    revisionOverflow
};

struct CalibrationIssue {
    CalibrationErrorCode code { CalibrationErrorCode::invalidDocument };
    std::string path;
    std::string message;
};

[[nodiscard]] std::string_view toString(CalibrationErrorCode code) noexcept;

template <typename T>
class CalibrationResult {
public:
    [[nodiscard]] static CalibrationResult success(T value) {
        CalibrationResult result;
        result.value_.emplace(std::move(value));
        return result;
    }

    [[nodiscard]] static CalibrationResult failure(std::vector<CalibrationIssue> issues) {
        CalibrationResult result;
        result.issues_ = std::move(issues);
        return result;
    }

    [[nodiscard]] bool hasValue() const noexcept { return value_.has_value(); }
    [[nodiscard]] explicit operator bool() const noexcept { return hasValue(); }
    [[nodiscard]] const T& value() const& { return value_.value(); }
    [[nodiscard]] T& value() & { return value_.value(); }
    [[nodiscard]] T takeValue() { return std::move(value_.value()); }
    [[nodiscard]] const std::vector<CalibrationIssue>& issues() const noexcept { return issues_; }

private:
    std::optional<T> value_;
    std::vector<CalibrationIssue> issues_;
};

enum class CalibrationUnit {
    dimensionless,
    ratio,
    percent,
    revolutionsPerMinute,
    kilopascal,
    bar,
    degreeCelsius,
    degreeCrankshaft,
    millisecond,
    second,
    lambda,
    airFuelRatio,
    newtonMetre,
    gramPerSecond,
    milligram,
    kilometrePerHour,
    hertz
};

enum class AxisQuantity {
    generic,
    engineSpeed,
    manifoldAbsolutePressure,
    normalizedLoad,
    throttlePosition,
    acceleratorPosition,
    coolantTemperature,
    intakeAirTemperature,
    lambda,
    vehicleSpeed,
    elapsedTime
};

[[nodiscard]] std::string_view toString(CalibrationUnit unit) noexcept;
[[nodiscard]] std::string_view toString(AxisQuantity quantity) noexcept;
[[nodiscard]] std::optional<CalibrationUnit> calibrationUnitFromString(std::string_view value) noexcept;
[[nodiscard]] std::optional<AxisQuantity> axisQuantityFromString(std::string_view value) noexcept;
[[nodiscard]] bool isUnitCompatible(AxisQuantity quantity, CalibrationUnit unit) noexcept;

struct CalibrationLimits {
    std::optional<double> minimum;
    std::optional<double> maximum;
};

struct CalibrationMetadata {
    std::string id;
    std::string displayName;
    std::string description;
    CalibrationUnit unit { CalibrationUnit::dimensionless };
    CalibrationLimits hardLimits;
    int displayPrecision { 2 };
    bool liveEditable { true };
};

struct AxisMetadata {
    std::string id;
    std::string displayName;
    AxisQuantity quantity { AxisQuantity::generic };
    CalibrationUnit unit { CalibrationUnit::dimensionless };
};

struct AxisCoordinate {
    AxisQuantity quantity { AxisQuantity::generic };
    CalibrationUnit unit { CalibrationUnit::dimensionless };
    double value { 0.0 };
};

struct AxisInterval {
    std::size_t lowerIndex { 0 };
    std::size_t upperIndex { 0 };
    double fraction { 0.0 };
};

struct CalibrationAxis {
    AxisMetadata metadata;
    std::vector<double> breakpoints;

    [[nodiscard]] AxisInterval locateClamped(double coordinate) const noexcept;
    [[nodiscard]] bool accepts(const AxisCoordinate& coordinate) const noexcept;
};

struct ScalarCalibration {
    CalibrationMetadata metadata;
    double value { 0.0 };
};

struct CalibrationCurve1D {
    CalibrationMetadata metadata;
    CalibrationAxis axis;
    std::vector<double> values;

    [[nodiscard]] double sampleClamped(double coordinate) const noexcept;
    [[nodiscard]] std::optional<double> sample(const AxisCoordinate& coordinate) const noexcept;
};

/** Values are stored row-major: one complete x-axis row for each y-axis breakpoint. */
struct CalibrationTable2D {
    CalibrationMetadata metadata;
    CalibrationAxis xAxis;
    CalibrationAxis yAxis;
    std::vector<double> values;

    [[nodiscard]] double sampleClamped(double xCoordinate, double yCoordinate) const noexcept;
    [[nodiscard]] std::optional<double> sample(const AxisCoordinate& xCoordinate,
                                               const AxisCoordinate& yCoordinate) const noexcept;
};

using CalibrationEntry = std::variant<ScalarCalibration, CalibrationCurve1D, CalibrationTable2D>;

[[nodiscard]] const CalibrationMetadata& metadataOf(const CalibrationEntry& entry) noexcept;

/** Mutable editing model. It may temporarily be invalid; only publish() creates a valid snapshot. */
class CalibrationDraft {
public:
    using EntryMap = std::map<std::string, CalibrationEntry, std::less<>>;

    std::string name;
    std::string description;

    void set(CalibrationEntry entry);
    [[nodiscard]] bool erase(std::string_view id);
    [[nodiscard]] const CalibrationEntry* find(std::string_view id) const noexcept;
    [[nodiscard]] const EntryMap& entries() const noexcept { return entries_; }

private:
    EntryMap entries_;
};

[[nodiscard]] std::vector<CalibrationIssue> validate(const CalibrationDraft& draft);

} // namespace enginelab::calibration
