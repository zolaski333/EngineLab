#include <enginelab/calibration/CalibrationJson.hpp>

#include <nlohmann/json.hpp>

#include <optional>
#include <type_traits>
#include <utility>

namespace enginelab::calibration {
namespace {

using Json = nlohmann::json;

void addParseIssue(std::vector<CalibrationIssue>& issues, std::string path, std::string message) {
    issues.push_back({ CalibrationErrorCode::parseError, std::move(path), std::move(message) });
}

std::optional<std::string> requiredString(const Json& object, std::string_view key,
                                          std::string_view path,
                                          std::vector<CalibrationIssue>& issues) {
    const auto found = object.find(key);
    if (found == object.end() || !found->is_string()) {
        addParseIssue(issues, std::string(path) + "." + std::string(key), "A string value is required.");
        return std::nullopt;
    }
    return found->get<std::string>();
}

std::string optionalString(const Json& object, std::string_view key, std::string fallback,
                           std::string_view path, std::vector<CalibrationIssue>& issues) {
    const auto found = object.find(key);
    if (found == object.end()) return fallback;
    if (!found->is_string()) {
        addParseIssue(issues, std::string(path) + "." + std::string(key), "This value must be a string.");
        return fallback;
    }
    return found->get<std::string>();
}

std::optional<double> optionalNumber(const Json& object, std::string_view key, std::string_view path,
                                     std::vector<CalibrationIssue>& issues) {
    const auto found = object.find(key);
    if (found == object.end()) return std::nullopt;
    if (!found->is_number()) {
        addParseIssue(issues, std::string(path) + "." + std::string(key), "This value must be numeric.");
        return std::nullopt;
    }
    return found->get<double>();
}

std::optional<std::vector<double>> numberArray(const Json& object, std::string_view key,
                                               std::string_view path,
                                               std::vector<CalibrationIssue>& issues) {
    const auto found = object.find(key);
    const auto arrayPath = std::string(path) + "." + std::string(key);
    if (found == object.end() || !found->is_array()) {
        addParseIssue(issues, arrayPath, "An array of numbers is required.");
        return std::nullopt;
    }
    std::vector<double> result;
    result.reserve(found->size());
    for (std::size_t index = 0; index < found->size(); ++index) {
        if (!(*found)[index].is_number()) {
            addParseIssue(issues, arrayPath + "[" + std::to_string(index) + "]",
                          "Array elements must be numeric.");
            return std::nullopt;
        }
        result.push_back((*found)[index].get<double>());
    }
    return result;
}

std::optional<CalibrationUnit> parseUnit(const Json& object, std::string_view key,
                                         std::string_view path,
                                         std::vector<CalibrationIssue>& issues) {
    const auto text = requiredString(object, key, path, issues);
    if (!text) return std::nullopt;
    const auto unit = calibrationUnitFromString(*text);
    if (!unit) {
        addParseIssue(issues, std::string(path) + "." + std::string(key),
                      "Unknown calibration unit '" + *text + "'.");
    }
    return unit;
}

std::optional<CalibrationMetadata> parseMetadata(const Json& object, std::string_view path,
                                                 std::vector<CalibrationIssue>& issues) {
    if (!object.is_object()) {
        addParseIssue(issues, std::string(path), "Calibration metadata must be an object.");
        return std::nullopt;
    }
    const auto id = requiredString(object, "id", path, issues);
    const auto displayName = requiredString(object, "display_name", path, issues);
    const auto unit = parseUnit(object, "unit", path, issues);
    if (!id || !displayName || !unit) return std::nullopt;

    CalibrationMetadata metadata;
    metadata.id = *id;
    metadata.displayName = *displayName;
    metadata.description = optionalString(object, "description", {}, path, issues);
    metadata.unit = *unit;
    if (const auto precision = object.find("display_precision"); precision != object.end()) {
        if (!precision->is_number_integer()) {
            addParseIssue(issues, std::string(path) + ".display_precision", "Display precision must be an integer.");
        } else {
            metadata.displayPrecision = precision->get<int>();
        }
    }
    if (const auto live = object.find("live_editable"); live != object.end()) {
        if (!live->is_boolean()) {
            addParseIssue(issues, std::string(path) + ".live_editable", "live_editable must be boolean.");
        } else {
            metadata.liveEditable = live->get<bool>();
        }
    }
    if (const auto limits = object.find("limits"); limits != object.end()) {
        if (!limits->is_object()) {
            addParseIssue(issues, std::string(path) + ".limits", "Limits must be an object.");
        } else {
            metadata.hardLimits.minimum = optionalNumber(*limits, "minimum", std::string(path) + ".limits", issues);
            metadata.hardLimits.maximum = optionalNumber(*limits, "maximum", std::string(path) + ".limits", issues);
        }
    }
    return metadata;
}

std::optional<CalibrationAxis> parseAxis(const Json& object, std::string_view path,
                                         std::vector<CalibrationIssue>& issues) {
    if (!object.is_object()) {
        addParseIssue(issues, std::string(path), "An axis must be an object.");
        return std::nullopt;
    }
    const auto id = requiredString(object, "id", path, issues);
    const auto displayName = requiredString(object, "display_name", path, issues);
    const auto quantityText = requiredString(object, "quantity", path, issues);
    const auto unit = parseUnit(object, "unit", path, issues);
    const auto breakpoints = numberArray(object, "breakpoints", path, issues);
    const auto quantity = quantityText ? axisQuantityFromString(*quantityText) : std::nullopt;
    if (quantityText && !quantity) {
        addParseIssue(issues, std::string(path) + ".quantity",
                      "Unknown axis quantity '" + *quantityText + "'.");
    }
    if (!id || !displayName || !quantity || !unit || !breakpoints) return std::nullopt;
    return CalibrationAxis { { *id, *displayName, *quantity, *unit }, *breakpoints };
}

Json metadataToJson(const CalibrationMetadata& metadata) {
    Json object {
        { "id", metadata.id },
        { "display_name", metadata.displayName },
        { "description", metadata.description },
        { "unit", toString(metadata.unit) },
        { "display_precision", metadata.displayPrecision },
        { "live_editable", metadata.liveEditable }
    };
    Json limits = Json::object();
    if (metadata.hardLimits.minimum) limits["minimum"] = *metadata.hardLimits.minimum;
    if (metadata.hardLimits.maximum) limits["maximum"] = *metadata.hardLimits.maximum;
    object["limits"] = std::move(limits);
    return object;
}

Json axisToJson(const CalibrationAxis& axis) {
    return {
        { "id", axis.metadata.id },
        { "display_name", axis.metadata.displayName },
        { "quantity", toString(axis.metadata.quantity) },
        { "unit", toString(axis.metadata.unit) },
        { "breakpoints", axis.breakpoints }
    };
}

template <typename EntryMap>
Json documentToJson(std::string_view name, std::string_view description, const EntryMap& entries) {
    Json calibrations = Json::array();
    for (const auto& [id, entry] : entries) {
        static_cast<void>(id);
        Json object;
        std::visit([&](const auto& calibration) {
            using T = std::decay_t<decltype(calibration)>;
            object["metadata"] = metadataToJson(calibration.metadata);
            if constexpr (std::is_same_v<T, ScalarCalibration>) {
                object["kind"] = "scalar";
                object["value"] = calibration.value;
            } else if constexpr (std::is_same_v<T, CalibrationCurve1D>) {
                object["kind"] = "curve_1d";
                object["axis"] = axisToJson(calibration.axis);
                object["values"] = calibration.values;
            } else {
                object["kind"] = "table_2d";
                object["x_axis"] = axisToJson(calibration.xAxis);
                object["y_axis"] = axisToJson(calibration.yAxis);
                object["values"] = calibration.values;
            }
        }, entry);
        calibrations.push_back(std::move(object));
    }
    return {
        { "schema_version", CalibrationJson::schemaVersion },
        { "name", name },
        { "description", description },
        { "calibrations", std::move(calibrations) }
    };
}

} // namespace

CalibrationResult<CalibrationDraft> CalibrationJson::parse(std::string_view jsonText) {
    std::vector<CalibrationIssue> issues;
    try {
        const auto root = Json::parse(jsonText.begin(), jsonText.end());
        if (!root.is_object()) {
            return CalibrationResult<CalibrationDraft>::failure({ {
                CalibrationErrorCode::parseError, "$", "The calibration document root must be an object." } });
        }
        const auto schema = root.find("schema_version");
        if (schema == root.end() || !schema->is_number_integer()) {
            addParseIssue(issues, "schema_version", "An integer schema version is required.");
        } else if (schema->get<int>() != schemaVersion) {
            issues.push_back({ CalibrationErrorCode::unsupportedSchema, "schema_version",
                "Only calibration schema version 1 is supported." });
        }

        CalibrationDraft draft;
        draft.name = optionalString(root, "name", {}, "$", issues);
        draft.description = optionalString(root, "description", {}, "$", issues);
        const auto calibrations = root.find("calibrations");
        if (calibrations == root.end() || !calibrations->is_array()) {
            addParseIssue(issues, "calibrations", "A calibrations array is required.");
        } else {
            for (std::size_t index = 0; index < calibrations->size(); ++index) {
                const auto& object = (*calibrations)[index];
                const auto path = std::string("calibrations[") + std::to_string(index) + "]";
                if (!object.is_object()) {
                    addParseIssue(issues, path, "Each calibration must be an object.");
                    continue;
                }
                const auto kind = requiredString(object, "kind", path, issues);
                const auto metadataNode = object.find("metadata");
                if (metadataNode == object.end()) {
                    addParseIssue(issues, path + ".metadata", "Calibration metadata is required.");
                    continue;
                }
                const auto metadata = parseMetadata(*metadataNode, path + ".metadata", issues);
                if (!kind || !metadata) continue;
                if (draft.find(metadata->id) != nullptr) {
                    issues.push_back({ CalibrationErrorCode::duplicateCalibration, path + ".metadata.id",
                        "Calibration identifier '" + metadata->id + "' occurs more than once." });
                    continue;
                }

                if (*kind == "scalar") {
                    const auto value = optionalNumber(object, "value", path, issues);
                    if (!value) {
                        if (object.find("value") == object.end())
                            addParseIssue(issues, path + ".value", "A scalar value is required.");
                        continue;
                    }
                    draft.set(ScalarCalibration { *metadata, *value });
                } else if (*kind == "curve_1d") {
                    const auto axisNode = object.find("axis");
                    if (axisNode == object.end()) {
                        addParseIssue(issues, path + ".axis", "A 1D curve axis is required.");
                        continue;
                    }
                    const auto axis = parseAxis(*axisNode, path + ".axis", issues);
                    const auto values = numberArray(object, "values", path, issues);
                    if (axis && values) draft.set(CalibrationCurve1D { *metadata, *axis, *values });
                } else if (*kind == "table_2d") {
                    const auto xNode = object.find("x_axis");
                    const auto yNode = object.find("y_axis");
                    if (xNode == object.end() || yNode == object.end()) {
                        addParseIssue(issues, path, "A 2D table requires x_axis and y_axis objects.");
                        continue;
                    }
                    const auto xAxis = parseAxis(*xNode, path + ".x_axis", issues);
                    const auto yAxis = parseAxis(*yNode, path + ".y_axis", issues);
                    const auto values = numberArray(object, "values", path, issues);
                    if (xAxis && yAxis && values)
                        draft.set(CalibrationTable2D { *metadata, *xAxis, *yAxis, *values });
                } else {
                    addParseIssue(issues, path + ".kind", "Unknown calibration kind '" + *kind + "'.");
                }
            }
        }
        const auto validationIssues = validate(draft);
        issues.insert(issues.end(), validationIssues.begin(), validationIssues.end());
        if (!issues.empty()) return CalibrationResult<CalibrationDraft>::failure(std::move(issues));
        return CalibrationResult<CalibrationDraft>::success(std::move(draft));
    } catch (const Json::exception& exception) {
        issues.push_back({ CalibrationErrorCode::parseError, "$", exception.what() });
        return CalibrationResult<CalibrationDraft>::failure(std::move(issues));
    }
}

CalibrationResult<std::string> CalibrationJson::serialize(const CalibrationDraft& draft, int indentation) {
    auto issues = validate(draft);
    if (!issues.empty()) return CalibrationResult<std::string>::failure(std::move(issues));
    return CalibrationResult<std::string>::success(
        documentToJson(draft.name, draft.description, draft.entries()).dump(indentation));
}

std::string CalibrationJson::serialize(const CalibrationSnapshot& snapshot, int indentation) {
    auto document = documentToJson(snapshot.name(), snapshot.description(), snapshot.entries());
    document["revision"] = snapshot.revision();
    document["source"] = snapshot.source();
    return document.dump(indentation);
}

PublishResult publishJson(CalibrationStore& store, std::string_view jsonText, const PublishOptions& options) {
    auto parsed = CalibrationJson::parse(jsonText);
    if (!parsed) {
        const auto active = store.snapshot();
        return { false, active->revision(), active->revision(), parsed.issues() };
    }
    return store.publish(parsed.value(), options);
}

} // namespace enginelab::calibration

