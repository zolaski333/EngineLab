#pragma once

#include <enginelab/scripting/EngineScriptCompiler.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace enginelab::scripting {

struct EngineScriptReloadState final {
    /** Last valid configuration. It remains unchanged after a failed attempt. */
    std::shared_ptr<const EngineConfig> config;
    std::uint64_t revision { 0 };
    std::uint64_t attempt { 0 };
    bool lastAttemptSucceeded { false };
    std::vector<EngineScriptDiagnostic> diagnostics;
    std::vector<std::filesystem::path> dependencies;
};

/**
 * Background polling reloader. Compilation and filesystem checks are performed only on its
 * worker thread; poll() is an atomic pointer load suitable for non-blocking consumers.
 */
class EngineScriptHotReloader final {
public:
    explicit EngineScriptHotReloader(
        std::filesystem::path rootScript,
        EngineScriptCompileOptions options = {},
        std::chrono::milliseconds pollingInterval = std::chrono::milliseconds { 250 });
    ~EngineScriptHotReloader();

    EngineScriptHotReloader(const EngineScriptHotReloader&) = delete;
    EngineScriptHotReloader& operator=(const EngineScriptHotReloader&) = delete;

    void start();
    void stop() noexcept;
    /** Schedules a background compile and returns immediately. */
    void requestReload() noexcept;

    [[nodiscard]] bool running() const noexcept;
    [[nodiscard]] std::shared_ptr<const EngineScriptReloadState> poll() const noexcept;
    [[nodiscard]] std::uint64_t revision() const noexcept;

private:
    struct FileStamp;
    [[nodiscard]] static FileStamp readFileStamp(const std::filesystem::path& path) noexcept;
    void workerLoop();
    [[nodiscard]] bool watchedFilesChanged() const;
    void compileAndPublish();
    void refreshWatchList(const std::vector<std::filesystem::path>& attemptedDependencies,
                          const std::vector<std::filesystem::path>& validDependencies);

    std::filesystem::path rootScript_;
    EngineScriptCompileOptions options_;
    std::chrono::milliseconds pollingInterval_;
    EngineScriptCompiler compiler_;
    std::atomic<std::shared_ptr<const EngineScriptReloadState>> state_;
    std::atomic<bool> running_ { false };
    std::atomic<bool> stopRequested_ { false };
    std::atomic<std::uint64_t> reloadRequest_ { 1 };
    mutable std::mutex lifecycleMutex_;
    std::condition_variable wakeCondition_;
    std::thread worker_;
    std::vector<std::filesystem::path> watchedFiles_;
    std::vector<FileStamp> watchedStamps_;
    std::vector<std::filesystem::path> validDependencies_;
};

} // namespace enginelab::scripting
