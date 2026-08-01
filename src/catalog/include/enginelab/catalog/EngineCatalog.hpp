#pragma once

#include <enginelab/foundation/EngineTypes.hpp>
#include <filesystem>
#include <string>
#include <vector>

namespace enginelab {

struct EngineCatalogEntry final {
    EngineConfig config;
    std::filesystem::path sourcePath;
    std::string family;
};

struct EngineCatalogLoadResult final {
    std::vector<EngineCatalogEntry> entries;
    std::vector<std::string> errors;
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

} // namespace enginelab
