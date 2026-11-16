#pragma once

// How the offline and loop-pack exports put their files in place: each file
// is written beside its final name (".partial") and renamed over it once all
// of them are written. Shared by OfflineAudioExporter.cpp and
// LoopPackExporter.cpp; not a public header.

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace enginelab::exportfiles {

/** Where `finalPath` is written before it is complete. */
[[nodiscard]] std::filesystem::path partialPathFor(const std::filesystem::path& finalPath);

/** Where the file a replacement moves aside waits until every new file is in
 *  place. */
[[nodiscard]] std::filesystem::path replacedPathFor(const std::filesystem::path& finalPath);

/**
 * Before anything is written: a stale partial or set-aside file of an earlier
 * export, an existing file when `overwrite` is off, or something that is not
 * a file where the file goes. The message, or nothing when the file can go.
 */
[[nodiscard]] std::optional<std::string> checkFinalFile(const std::filesystem::path& finalPath, bool overwrite);

/**
 * Renames every partial over its final file (an existing one only when
 * `overwrite` is on). A file it replaces is moved
 * aside first and deleted only once all the new files are in place; if one
 * cannot be put in place, the files already put are removed and the
 * replaced ones come back, so a failed export leaves the folder as it was
 * (audit UA-05: a failure lost the earlier master). The message on failure.
 */
[[nodiscard]] std::optional<std::string> commitPartials(const std::vector<std::filesystem::path>& finalFiles,
                                                      bool overwrite);

} // namespace enginelab::exportfiles
