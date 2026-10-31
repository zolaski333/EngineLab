#pragma once

#include <enginelab/foundation/EngineTypes.hpp>
#include <filesystem>
#include <map>
#include <string>
#include <string_view>
#include <vector>

namespace enginelab {

/** The shared parts under `parts/`: family estimates an engine file names
 *  under `uses:`. */
struct PartsLibrary final {
    std::map<std::string, FuelConfig> fuels;
    std::map<std::string, InjectionConfig> injections;
    std::map<std::string, CamshaftConfig> camshafts;
    std::map<std::string, ExhaustConfig> exhausts;
    std::map<std::string, TransmissionConfig> transmissions;
    std::map<std::string, VehicleConfig> vehicles;
};

/** Reads every `*.yaml` under `<root>/parts`; throws on a malformed file. */
[[nodiscard]] PartsLibrary loadPartsLibrary(const std::filesystem::path& rootDirectory);

/**
 * Gives a path with no graph the one its scalar geometry describes: one
 * primary per cylinder, a collector, the mid pipe and silencer, an outlet
 * (independent stacks for `aircraft_manifold`). `presetKey` is the exhaust
 * part the geometry came from, or empty. A rotary's faces open onto their
 * rotor's one port: `facesPerPort` 3 gives each rotor one passage and one
 * primary, as the catalogue's rotary.
 */
void compileLegacyExhaustPath(ExhaustPathConfig& path, std::string_view presetKey, std::size_t facesPerPort = 1);

struct EngineCatalogEntry final {
    EngineConfig config;
    std::filesystem::path sourcePath;
    std::string family;
};

struct EngineCatalogLoadResult final {
    std::vector<EngineCatalogEntry> entries;
    std::vector<std::string> errors;
};

enum class EngineCatalogSelectionStatus {
    notFound,
    unique,
    ambiguous,
};

/**
 * Result of resolving a selector that is required to identify exactly one
 * catalogue engine. Pointers remain valid only while the supplied entries
 * vector is alive and is not reallocated.
 */
struct EngineCatalogSelectionResult final {
    EngineCatalogSelectionStatus status { EngineCatalogSelectionStatus::notFound };
    const EngineCatalogEntry* entry {};
    std::vector<const EngineCatalogEntry*> matches;
    bool exactMatch {};

    [[nodiscard]] explicit operator bool() const noexcept {
        return status == EngineCatalogSelectionStatus::unique && entry != nullptr;
    }
};

struct AudioVoicingLoadResult final {
    AudioVoicingConfig voicing;
    std::vector<std::filesystem::path> sources;
    std::string error;
    [[nodiscard]] explicit operator bool() const noexcept {
        return error.empty();
    }
};

/** Resolve voicing/default.yaml, then the optional family and engine layers. */
[[nodiscard]] AudioVoicingLoadResult loadAudioVoicing(
    const std::filesystem::path& rootDirectory,
    std::string_view family,
    std::string_view engineKey);

/** Latest voicing tree timestamp, for cheap UI-thread hot-reload polling. */
[[nodiscard]] std::filesystem::file_time_type audioVoicingRevision(
    const std::filesystem::path& rootDirectory) noexcept;

[[nodiscard]] EngineCatalogLoadResult loadEngineCatalog(const std::filesystem::path& rootDirectory);
[[nodiscard]] std::vector<EngineConfig> makeCatalogOrBasePresets(const std::filesystem::path& rootDirectory);

/**
 * Resolve one engine deterministically. An exact case-insensitive match on the
 * stable catalogue key, source-file stem or complete display name takes
 * precedence over substring matching. A substring is accepted only when it is
 * unique; ambiguity is returned to the caller instead of silently selecting
 * the first catalogue entry.
 */
[[nodiscard]] EngineCatalogSelectionResult selectSingleEngineCatalogEntry(
    const std::vector<EngineCatalogEntry>& entries,
    std::string_view selector);

} // namespace enginelab
