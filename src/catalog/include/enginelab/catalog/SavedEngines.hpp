#pragma once

#include <enginelab/calibration/CalibrationStore.hpp>
#include <enginelab/foundation/EngineTypes.hpp>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace enginelab {

/**
 * An engine the user saved as it was: its whole configuration (exhaust,
 * intake, cylinders, forced induction...) and its ECU calibration.
 *
 * On disk, one file: `<name>.json` (or a `.yaml` export), the engine as
 * "Export engine" writes it, its ECU calibration inside it under
 * `ecu_calibration`, so any copy imports anywhere with its maps. A save made
 * by 0.9.1 or earlier kept the calibration beside it in `<name>.ecu.json`: such a
 * file is still read, and the next save folds it in. Without either the
 * engine runs on the calibration its configuration implies.
 */
struct SavedEngine final {
    EngineConfig config;
    std::optional<calibration::CalibrationDraft> calibration;
    std::filesystem::path file;
};

struct SavedEngineRead final {
    std::optional<SavedEngine> engine;
    std::string error;
    /** Values clamped into their range to load the file, one line each. */
    std::vector<std::string> warnings;
};

struct SavedEngineLoad final {
    /** Sorted by file name. */
    std::vector<SavedEngine> engines;
    /** One line per file that could not be read. */
    std::vector<std::string> errors;
    /** One line per value clamped into its range ("file: what"). */
    std::vector<std::string> warnings;
};

/** `<folder>/<name>.json`, the name made safe for a file system. */
[[nodiscard]] std::filesystem::path savedEngineFile(const std::filesystem::path& folder, std::string_view name);
/** The calibration file a save made by 0.9.1 or earlier kept beside an engine file. */
[[nodiscard]] std::filesystem::path savedCalibrationFile(const std::filesystem::path& engineFile);

enum class EngineFileFormat { json, yaml };
/** YAML for `.yaml` and `.yml`, JSON otherwise. */
[[nodiscard]] EngineFileFormat engineFileFormatOf(const std::filesystem::path& file);
/** An engine file's text: the engine and, when given, its calibration under
    `ecu_calibration` (the ECU window's JSON, or the same tree in YAML). */
[[nodiscard]] std::string encodeEngineFile(const EngineConfig&, const calibration::CalibrationSnapshot* calibration,
                                           EngineFileFormat);
/** Reads an engine file's text; `file` names it in messages and in the result.
    A calibration inside it that does not parse fails the read. */
[[nodiscard]] SavedEngineRead decodeEngineFile(std::string_view text, EngineFileFormat,
                                               const std::filesystem::path& file);

/** What a file dropped on the window is. */
enum class DroppedFileKind { none, engine, calibration, engineScript };
/** By its name: `.ecu.json` maps, `.json`/`.yaml`/`.yml` engines, `.els`/`.engine` scripts. */
[[nodiscard]] DroppedFileKind droppedFileKind(const std::filesystem::path& file);

/** Why a name cannot be used for a saved engine, or none. A catalogue name
    is refused: the engine list tells engines apart by name. */
[[nodiscard]] std::optional<std::string> savedEngineNameError(std::string_view name,
                                                               const std::vector<std::string>& catalogueNames);

/**
 * Writes the engine with its calibration inside, in the format of the file's
 * extension, to a temporary file renamed over the old one, so a failure never
 * leaves half a file. A calibration file left beside it by an older save is
 * removed. Returns why it failed.
 */
[[nodiscard]] std::optional<std::string> saveEngine(const std::filesystem::path& engineFile, const EngineConfig&,
                                                    const calibration::CalibrationSnapshot* calibration);

/**
 * Puts an engine just saved into the engine menu's list, where catalogue
 * engines have no file: drops the saved entries it replaces (same file or
 * same name) and inserts it among the saved ones in file order. Returns its
 * index. `saved` is taken by value because a caller may pass one of the
 * list's own entries: held by reference, the erase slid the next engine into
 * its place, and each Save wrote over the next saved engine in the menu
 * (owner report 2026-11-16, Audi RC8 into three others).
 */
std::size_t placeSavedEngine(std::vector<SavedEngine>& engines, SavedEngine saved);

/** Reads one saved engine (JSON or YAML) and, if present, its calibration:
    the one inside it, or else an older save's file beside it. A calibration
    that does not parse makes the whole read fail: running the engine on other
    tables than the saved ones would not be the engine as it was saved. */
[[nodiscard]] SavedEngineRead readSavedEngine(const std::filesystem::path& engineFile);

/** Every saved engine in `folder` (none if it does not exist), each with the
    voicing it names in the catalogue at `catalogueRoot`. */
[[nodiscard]] SavedEngineLoad loadSavedEngines(const std::filesystem::path& folder,
                                               const std::filesystem::path& catalogueRoot);

/** Gives an engine read from a file the voicing it names (family and key) in
    the catalogue at `catalogueRoot`; an engine that names none keeps its own.
    Returns why the voicing could not be loaded. */
[[nodiscard]] std::optional<std::string> restoreAudioVoicing(EngineConfig&,
                                                             const std::filesystem::path& catalogueRoot);

/**
 * The engine as it was loaded, to tell whether it has been modified since.
 * The configuration is compared through its JSON encoding, which is what a
 * save writes; the calibration through the JSON of its tables, without the
 * store's revision, so tables edited back to their values are not "modified".
 */
class EngineBaseline final {
public:
    EngineBaseline() = default;
    /** `config` normalised, as the runtime runs it. */
    EngineBaseline(const EngineConfig& config, const calibration::CalibrationDraft& calibration);
    /** Encodes the whole configuration: call it after an edit, not per frame. */
    [[nodiscard]] bool configDiffers(const EngineConfig&) const;
    /** Serialises every table: call it when the snapshot changes. */
    [[nodiscard]] bool calibrationDiffers(const calibration::CalibrationSnapshot&) const;
private:
    std::string config_;
    std::string calibration_;
};

} // namespace enginelab
