#include <enginelab/catalog/SavedEngines.hpp>

#include <enginelab/calibration/CalibrationJson.hpp>
#include <enginelab/calibration/EcuCalibration.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/serialization/JsonEngineSerializer.hpp>
#include <enginelab/serialization/YamlEngineSerializer.hpp>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cctype>
#include <fstream>
#include <iterator>
#include <sstream>
#include <system_error>

namespace enginelab {
namespace {
using Json = nlohmann::json;
constexpr std::size_t maximumNameLength = 80;
constexpr std::uintmax_t maximumFileBytes = 2U * 1024U * 1024U;
constexpr std::string_view calibrationSuffix = ".ecu.json";
/** The engine file's member that holds its ECU calibration. */
constexpr const char* calibrationKey = "ecu_calibration";

/** The calibration's JSON tree in YAML: strings quoted, numbers as JSON
    writes them (the shortest text that reads back to the same double), so
    jsonOf() gives the same tree back. */
void emitYaml(YAML::Emitter& out, const Json& value) {
    if (value.is_object()) {
        out << YAML::BeginMap;
        for (const auto& [key, item] : value.items()) {
            out << YAML::Key << key << YAML::Value;
            emitYaml(out, item);
        }
        out << YAML::EndMap;
    } else if (value.is_array()) {
        // A row of numbers on one line keeps a table readable.
        if (std::all_of(value.begin(), value.end(), [](const Json& item) { return item.is_primitive(); }))
            out << YAML::Flow;
        out << YAML::BeginSeq;
        for (const auto& item : value) emitYaml(out, item);
        out << YAML::EndSeq;
    } else if (value.is_string()) {
        out << YAML::DoubleQuoted << value.get<std::string>();
    } else if (value.is_null()) {
        out << YAML::Null;
    } else {
        out << value.dump();
    }
}

[[nodiscard]] Json jsonOf(const YAML::Node& node) {
    switch (node.Type()) {
    case YAML::NodeType::Map: {
        auto object = Json::object();
        for (const auto& item : node) object[item.first.as<std::string>()] = jsonOf(item.second);
        return object;
    }
    case YAML::NodeType::Sequence: {
        auto array = Json::array();
        for (const auto& item : node) array.push_back(jsonOf(item));
        return array;
    }
    case YAML::NodeType::Scalar: {
        const auto& text = node.Scalar();
        // A quoted scalar is a string whatever it holds.
        if (node.Tag() == "!") return text;
        if (text == "true") return true;
        if (text == "false") return false;
        if (text == "~" || text == "null") return nullptr;
        auto number = Json::parse(text, nullptr, false);
        if (!number.is_discarded() && number.is_number()) return number;
        return text;
    }
    default:
        return nullptr;
    }
}

[[nodiscard]] std::string lowercase(std::string text) {
    for (auto& character : text) character = static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
    return text;
}

[[nodiscard]] bool endsWith(std::string_view text, std::string_view suffix) noexcept {
    return text.size() >= suffix.size() && text.substr(text.size() - suffix.size()) == suffix;
}

[[nodiscard]] std::filesystem::path pathFromUtf8(std::string_view text) {
    return std::filesystem::path(std::u8string(reinterpret_cast<const char8_t*>(text.data()), text.size()));
}

/** A file name for a message, whatever characters it holds. */
[[nodiscard]] std::string nameOf(const std::filesystem::path& file) {
    const auto name = file.filename().u8string();
    return std::string(reinterpret_cast<const char*>(name.data()), name.size());
}

[[nodiscard]] std::string trimmed(std::string_view text) {
    const auto first = text.find_first_not_of(" \t");
    if (first == std::string_view::npos) return {};
    const auto last = text.find_last_not_of(" \t");
    return std::string(text.substr(first, last - first + 1));
}

/** Writes `text` beside `file`, then renames it over `file`. */
[[nodiscard]] std::optional<std::string> writeReplacing(const std::filesystem::path& file, std::string_view text) {
    auto temporary = file;
    temporary += ".tmp";
    {
        std::ofstream out(temporary, std::ios::binary | std::ios::trunc);
        if (!out) return "cannot write " + nameOf(temporary);
        out.write(text.data(), static_cast<std::streamsize>(text.size()));
        out.close();
        if (!out) return "cannot write " + nameOf(temporary);
    }
    std::error_code error;
    std::filesystem::rename(temporary, file, error);
    if (error) {
        std::filesystem::remove(temporary, error);
        return "cannot replace " + nameOf(file);
    }
    return std::nullopt;
}

[[nodiscard]] std::optional<std::string> readText(const std::filesystem::path& file, std::string& text) {
    std::error_code error;
    const auto size = std::filesystem::file_size(file, error);
    if (error) return "cannot read " + nameOf(file);
    if (size > maximumFileBytes) return nameOf(file) + " exceeds the 2 MiB limit";
    std::ifstream in(file, std::ios::binary);
    if (!in) return "cannot read " + nameOf(file);
    text.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    return std::nullopt;
}

[[nodiscard]] std::string issuesText(const std::vector<calibration::CalibrationIssue>& issues) {
    std::string text;
    for (const auto& issue : issues) {
        if (!text.empty()) text += "; ";
        text += issue.path.empty() ? issue.message : issue.path + ": " + issue.message;
    }
    return text;
}
} // namespace

std::filesystem::path savedEngineFile(const std::filesystem::path& folder, std::string_view name) {
    std::string stem;
    for (const auto character : trimmed(name)) {
        const auto code = static_cast<unsigned char>(character);
        const auto reserved = code < 32U || std::string_view("<>:\"/\\|?*").find(character) != std::string_view::npos;
        stem += reserved ? '_' : character;
    }
    // Windows drops trailing dots and spaces from a file name.
    while (!stem.empty() && (stem.back() == '.' || stem.back() == ' ')) stem.pop_back();
    if (stem.empty()) stem = "engine";
    // Not a calibration file by accident.
    if (endsWith(stem, ".ecu")) stem += '_';
    return folder / pathFromUtf8(stem + ".json");
}

std::filesystem::path savedCalibrationFile(const std::filesystem::path& engineFile) {
    auto file = engineFile;
    file.replace_extension(pathFromUtf8(calibrationSuffix));
    return file;
}

std::optional<std::string> savedEngineNameError(std::string_view name,
                                                const std::vector<std::string>& catalogueNames) {
    const auto clean = trimmed(name);
    if (clean.empty()) return "The name is empty.";
    if (clean.size() > maximumNameLength) return "The name is longer than 80 characters.";
    if (std::find(catalogueNames.begin(), catalogueNames.end(), clean) != catalogueNames.end())
        return "A catalogue engine already has this name.";
    return std::nullopt;
}

EngineFileFormat engineFileFormatOf(const std::filesystem::path& file) {
    const auto extension = lowercase(nameOf(file.extension()));
    return extension == ".yaml" || extension == ".yml" ? EngineFileFormat::yaml : EngineFileFormat::json;
}

DroppedFileKind droppedFileKind(const std::filesystem::path& file) {
    const auto name = lowercase(nameOf(file));
    if (endsWith(name, calibrationSuffix)) return DroppedFileKind::calibration;
    if (endsWith(name, ".json") || endsWith(name, ".yaml") || endsWith(name, ".yml")) return DroppedFileKind::engine;
    if (endsWith(name, ".els") || endsWith(name, ".engine")) return DroppedFileKind::engineScript;
    return DroppedFileKind::none;
}

std::string encodeEngineFile(const EngineConfig& config, const calibration::CalibrationSnapshot* calibration,
                             EngineFileFormat format) {
    if (format == EngineFileFormat::yaml) {
        auto text = YamlEngineSerializer {}.encode(config);
        if (calibration == nullptr) return text;
        YAML::Emitter out;
        out << YAML::BeginMap << YAML::Key << calibrationKey << YAML::Value;
        emitYaml(out, Json::parse(calibration::CalibrationJson::serialize(*calibration)));
        out << YAML::EndMap;
        if (!text.empty() && text.back() != '\n') text += '\n';
        return text + out.c_str() + '\n';
    }
    auto text = JsonEngineSerializer {}.encode(config);
    if (calibration == nullptr) return text;
    auto document = Json::parse(text);
    document[calibrationKey] = Json::parse(calibration::CalibrationJson::serialize(*calibration));
    return document.dump(2);
}

SavedEngineRead decodeEngineFile(std::string_view text, EngineFileFormat format, const std::filesystem::path& file) {
    SavedEngineRead result;
    // A hand-edited value outside its range is clamped, not a reason to
    // drop the engine from the menu.
    std::vector<std::string> repairs;
    EngineDecodeResult decoded;
    std::optional<std::string> calibrationText;
    try {
        if (format == EngineFileFormat::yaml) {
            decoded = YamlEngineSerializer {}.decode(text);
            if (decoded) {
                const auto document = YAML::Load(std::string(text));
                if (const auto node = document[calibrationKey]; node && !node.IsNull())
                    calibrationText = jsonOf(node).dump();
            }
        } else {
            decoded = JsonEngineSerializer {}.decode(text, &repairs);
            if (decoded) {
                const auto document = Json::parse(text, nullptr, false);
                if (document.is_object() && document.contains(calibrationKey))
                    calibrationText = document.at(calibrationKey).dump();
            }
        }
    } catch (const std::exception& error) {
        result.error = nameOf(file) + ": " + error.what();
        return result;
    }
    if (!decoded) {
        result.error = nameOf(file) + ": " + decoded.error;
        return result;
    }
    SavedEngine engine;
    engine.config = std::move(*decoded.config);
    disableBackfires(engine.config);
    engine.file = file;
    if (calibrationText) {
        auto parsed = calibration::CalibrationJson::parse(*calibrationText);
        if (!parsed) {
            result.error = nameOf(file) + " (ECU calibration): " + issuesText(parsed.issues());
            return result;
        }
        engine.calibration = parsed.takeValue();
        calibration::addMissingDefaultSettings(*engine.calibration, engine.config);
    }
    result.engine = std::move(engine);
    for (auto& repair : repairs) result.warnings.push_back(nameOf(file) + ": " + std::move(repair));
    return result;
}

std::optional<std::string> saveEngine(const std::filesystem::path& engineFile, const EngineConfig& config,
                                      const calibration::CalibrationSnapshot* calibration) {
    std::error_code error;
    std::filesystem::create_directories(engineFile.parent_path(), error);
    if (error) return "cannot create the folder of " + nameOf(engineFile);
    auto saved = config;
    disableBackfires(saved);
    if (auto failed = writeReplacing(engineFile, encodeEngineFile(saved, calibration, engineFileFormatOf(engineFile))))
        return failed;
    // An older save's tables beside it would outlive the ones now inside.
    std::filesystem::remove(savedCalibrationFile(engineFile), error);
    return std::nullopt;
}

SavedEngineRead readSavedEngine(const std::filesystem::path& engineFile) {
    std::string text;
    if (auto failed = readText(engineFile, text)) {
        SavedEngineRead result;
        result.error = *failed;
        return result;
    }
    auto result = decodeEngineFile(text, engineFileFormatOf(engineFile), engineFile);
    if (!result.engine || result.engine->calibration) return result;
    // A save made by 0.9.1 or earlier: its tables beside it.
    const auto calibrationFile = savedCalibrationFile(engineFile);
    std::error_code error;
    if (std::filesystem::exists(calibrationFile, error)) {
        if (auto failed = readText(calibrationFile, text)) {
            result.engine.reset();
            result.error = *failed;
            return result;
        }
        auto parsed = calibration::CalibrationJson::parse(text);
        if (!parsed) {
            result.engine.reset();
            result.error = nameOf(calibrationFile) + ": " + issuesText(parsed.issues());
            return result;
        }
        result.engine->calibration = parsed.takeValue();
        calibration::addMissingDefaultSettings(*result.engine->calibration, result.engine->config);
    }
    return result;
}

SavedEngineLoad loadSavedEngines(const std::filesystem::path& folder, const std::filesystem::path& catalogueRoot) {
    SavedEngineLoad result;
    std::error_code error;
    if (!std::filesystem::is_directory(folder, error)) return result;
    std::vector<std::filesystem::path> files;
    for (const auto& entry : std::filesystem::directory_iterator(folder, error)) {
        const auto& path = entry.path();
        if (entry.is_regular_file(error) && path.extension() == ".json" && path.stem().extension() != ".ecu")
            files.push_back(path);
    }
    std::sort(files.begin(), files.end());
    for (const auto& file : files) {
        auto read = readSavedEngine(file);
        if (!read.engine) {
            result.errors.push_back(std::move(read.error));
            continue;
        }
        for (auto& warning : read.warnings) result.warnings.push_back(std::move(warning));
        if (auto failed = restoreAudioVoicing(read.engine->config, catalogueRoot))
            result.errors.push_back(nameOf(file) + " voicing: " + *failed);
        result.engines.push_back(std::move(*read.engine));
    }
    return result;
}

std::optional<std::string> restoreAudioVoicing(EngineConfig& config, const std::filesystem::path& catalogueRoot) {
    if (config.audioVoicingKey.empty()) return std::nullopt;
    auto loaded = loadAudioVoicing(catalogueRoot, config.audioVoicingFamily, config.audioVoicingKey);
    if (!loaded) return std::move(loaded.error);
    config.audioVoicing = std::move(loaded.voicing);
    return std::nullopt;
}

namespace {

// Backfires are switched on the running engine, not edited into it.
std::string comparable(EngineConfig config) {
    disableBackfires(config);
    return JsonEngineSerializer {}.encode(config);
}

} // namespace

EngineBaseline::EngineBaseline(const EngineConfig& config, const calibration::CalibrationDraft& calibration)
    : config_(comparable(config)) {
    auto tables = calibration::CalibrationJson::serialize(calibration);
    if (tables) calibration_ = tables.takeValue();
}

bool EngineBaseline::configDiffers(const EngineConfig& config) const {
    return comparable(config) != config_;
}

bool EngineBaseline::calibrationDiffers(const calibration::CalibrationSnapshot& snapshot) const {
    auto tables = calibration::CalibrationJson::serialize(calibration::makeDraft(snapshot));
    return !tables || tables.takeValue() != calibration_;
}

} // namespace enginelab
