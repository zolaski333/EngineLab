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
 * On disk, two files share a stem: `<name>.json`, the engine as "Export
 * engine" writes it (so any copy imports anywhere), and `<name>.ecu.json`, the
 * calibration as the ECU window saves it. Without the second file the engine
 * runs on the calibration its configuration implies.
 */
struct SavedEngine final {
    EngineConfig config;
    std::optional<calibration::CalibrationDraft> calibration;
    std::filesystem::path file;
};

struct SavedEngineRead final {
    std::optional<SavedEngine> engine;
    std::string error;
};

struct SavedEngineLoad final {
    /** Sorted by file name. */
    std::vector<SavedEngine> engines;
    /** One line per file that could not be read. */
    std::vector<std::string> errors;
};

/** `<folder>/<name>.json`, the name made safe for a file system. */
[[nodiscard]] std::filesystem::path savedEngineFile(const std::filesystem::path& folder, std::string_view name);
/** The calibration file that goes with an engine file. */
[[nodiscard]] std::filesystem::path savedCalibrationFile(const std::filesystem::path& engineFile);

/** Why a name cannot be used for a saved engine, or none. A catalogue name
    is refused: the engine list tells engines apart by name. */
[[nodiscard]] std::optional<std::string> savedEngineNameError(std::string_view name,
                                                               const std::vector<std::string>& catalogueNames);

/**
 * Writes the engine and its calibration, each to a temporary file renamed
 * over the old one, so a failure never leaves half a file. Without a
 * calibration, a stale calibration file is removed. Returns why it failed.
 */
[[nodiscard]] std::optional<std::string> saveEngine(const std::filesystem::path& engineFile, const EngineConfig&,
                                                    const calibration::CalibrationSnapshot* calibration);

/** Reads one saved engine and, if present, its calibration. A calibration
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
