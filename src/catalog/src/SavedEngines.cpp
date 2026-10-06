#include <enginelab/catalog/SavedEngines.hpp>

#include <enginelab/calibration/CalibrationJson.hpp>
#include <enginelab/calibration/EcuCalibration.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/serialization/JsonEngineSerializer.hpp>
#include <algorithm>
#include <fstream>
#include <iterator>
#include <sstream>
#include <system_error>

namespace enginelab {
namespace {
constexpr std::size_t maximumNameLength = 80;
constexpr std::uintmax_t maximumFileBytes = 2U * 1024U * 1024U;
constexpr std::string_view calibrationSuffix = ".ecu.json";

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

std::optional<std::string> saveEngine(const std::filesystem::path& engineFile, const EngineConfig& config,
                                      const calibration::CalibrationSnapshot* calibration) {
    std::error_code error;
    std::filesystem::create_directories(engineFile.parent_path(), error);
    if (error) return "cannot create the folder of " + nameOf(engineFile);
    const auto calibrationFile = savedCalibrationFile(engineFile);
    // The calibration first: an engine file never points at tables from
    // another save.
    if (calibration != nullptr) {
        if (auto failed = writeReplacing(calibrationFile, calibration::CalibrationJson::serialize(*calibration)))
            return failed;
    } else {
        std::filesystem::remove(calibrationFile, error);
    }
    return writeReplacing(engineFile, JsonEngineSerializer {}.encode(config));
}

SavedEngineRead readSavedEngine(const std::filesystem::path& engineFile) {
    SavedEngineRead result;
    std::string text;
    if (auto failed = readText(engineFile, text)) {
        result.error = *failed;
        return result;
    }
    auto decoded = JsonEngineSerializer {}.decode(text);
    if (!decoded) {
        result.error = nameOf(engineFile) + ": " + decoded.error;
        return result;
    }
    SavedEngine engine;
    engine.config = std::move(*decoded.config);
    engine.file = engineFile;
    const auto calibrationFile = savedCalibrationFile(engineFile);
    std::error_code error;
    if (std::filesystem::exists(calibrationFile, error)) {
        if (auto failed = readText(calibrationFile, text)) {
            result.error = *failed;
            return result;
        }
        auto parsed = calibration::CalibrationJson::parse(text);
        if (!parsed) {
            result.error = nameOf(calibrationFile) + ": " + issuesText(parsed.issues());
            return result;
        }
        engine.calibration = parsed.takeValue();
    }
    result.engine = std::move(engine);
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

EngineBaseline::EngineBaseline(const EngineConfig& config, const calibration::CalibrationDraft& calibration)
    : config_(JsonEngineSerializer {}.encode(config)) {
    auto tables = calibration::CalibrationJson::serialize(calibration);
    if (tables) calibration_ = tables.takeValue();
}

bool EngineBaseline::configDiffers(const EngineConfig& config) const {
    return JsonEngineSerializer {}.encode(config) != config_;
}

bool EngineBaseline::calibrationDiffers(const calibration::CalibrationSnapshot& snapshot) const {
    auto tables = calibration::CalibrationJson::serialize(calibration::makeDraft(snapshot));
    return !tables || tables.takeValue() != calibration_;
}

} // namespace enginelab
