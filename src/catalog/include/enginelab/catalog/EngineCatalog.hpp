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

[[nodiscard]] EngineCatalogLoadResult loadEngineCatalog(const std::filesystem::path& rootDirectory);
[[nodiscard]] std::vector<EngineConfig> makeCatalogOrBasePresets(const std::filesystem::path& rootDirectory);

} // namespace enginelab
