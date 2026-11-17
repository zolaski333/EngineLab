#pragma once

// The default catalogue of a command-line tool that ships in the release
// archive (audit PKG-02). An installed tool sits in `<package>/tools/`, its
// catalogue (`engines/`, `parts/`) one directory up; a tool run from the
// build tree finds neither and keeps the source catalogue it was built
// against. `--catalog` (or the tool's own flag) still overrides it.

#include <filesystem>
#include <system_error>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace enginelab::tools {

/** This process's executable, or an empty path when the platform does not say. */
[[nodiscard]] inline std::filesystem::path currentExecutablePath() {
#if defined(_WIN32)
    wchar_t buffer[32768] {};
    const auto length = GetModuleFileNameW(nullptr, buffer, static_cast<DWORD>(std::size(buffer)));
    if (length == 0 || length >= std::size(buffer)) return {};
    return std::filesystem::path(buffer);
#else
    std::error_code error;
    auto path = std::filesystem::read_symlink("/proc/self/exe", error);
    if (error) return {};
    return path;
#endif
}

/** True when `root` holds a catalogue the tools can load. */
[[nodiscard]] inline bool holdsCatalog(const std::filesystem::path& root) {
    std::error_code error;
    return std::filesystem::is_directory(root / "engines", error)
        && std::filesystem::is_directory(root / "parts", error);
}

/** The catalogue beside the executable or one directory above it (the
 *  release layout), else `buildTimeRoot`. */
[[nodiscard]] inline std::filesystem::path defaultCatalogRoot(const std::filesystem::path& buildTimeRoot) {
    const auto executable = currentExecutablePath();
    if (!executable.empty()) {
        const auto directory = executable.parent_path();
        if (holdsCatalog(directory)) return directory;
        if (holdsCatalog(directory.parent_path())) return directory.parent_path();
    }
    return buildTimeRoot;
}

} // namespace enginelab::tools
