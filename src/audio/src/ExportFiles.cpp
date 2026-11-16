#include "ExportFiles.hpp"

#include <system_error>
#include <utility>

namespace enginelab::exportfiles {

std::filesystem::path partialPathFor(const std::filesystem::path& finalPath) {
    auto partial = finalPath;
    partial += ".partial";
    return partial;
}

std::filesystem::path replacedPathFor(const std::filesystem::path& finalPath) {
    auto replaced = finalPath;
    replaced += ".replaced";
    return replaced;
}

std::optional<std::string> checkFinalFile(const std::filesystem::path& finalPath, bool overwrite) {
    std::error_code error;
    if (std::filesystem::exists(partialPathFor(finalPath), error))
        return "Stale partial export exists: " + partialPathFor(finalPath).string();
    if (std::filesystem::exists(replacedPathFor(finalPath), error))
        return "A file an earlier export set aside is still there: " + replacedPathFor(finalPath).string();
    const auto status = std::filesystem::symlink_status(finalPath, error);
    if (!std::filesystem::exists(status)) return std::nullopt;
    if (!overwrite) return "Output already exists: " + finalPath.string();
    if (!std::filesystem::is_regular_file(status))
        return "Output exists and is not a file: " + finalPath.string();
    return std::nullopt;
}

std::optional<std::string> commitPartials(const std::vector<std::filesystem::path>& finalFiles, bool overwrite) {
    std::vector<std::pair<std::filesystem::path, std::filesystem::path>> setAside;
    std::vector<std::filesystem::path> placed;
    const auto rollBack = [&] {
        std::error_code ignored;
        for (const auto& file : placed) std::filesystem::remove(file, ignored);
        for (const auto& [file, aside] : setAside) std::filesystem::rename(aside, file, ignored);
    };
    for (const auto& file : finalFiles) {
        std::error_code error;
        if (std::filesystem::exists(file, error)) {
            if (!overwrite) {
                rollBack();
                return "Output already exists: " + file.string();
            }
            const auto aside = replacedPathFor(file);
            std::filesystem::rename(file, aside, error);
            if (error) {
                rollBack();
                return "Could not replace " + file.string() + ": " + error.message();
            }
            setAside.emplace_back(file, aside);
        }
        std::filesystem::rename(partialPathFor(file), file, error);
        if (error) {
            rollBack();
            return "Could not write " + file.string() + ": " + error.message();
        }
        placed.push_back(file);
    }
    std::error_code ignored;
    for (const auto& [file, aside] : setAside) std::filesystem::remove(aside, ignored);
    return std::nullopt;
}

} // namespace enginelab::exportfiles
