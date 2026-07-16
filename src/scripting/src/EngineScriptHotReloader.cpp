#include <enginelab/scripting/EngineScriptHotReloader.hpp>

#include <algorithm>
#include <limits>
#include <system_error>
#include <utility>

namespace enginelab::scripting {

struct EngineScriptHotReloader::FileStamp {
    bool exists { false };
    std::filesystem::file_time_type writeTime {};
    std::uintmax_t size { 0 };

    [[nodiscard]] bool operator==(const FileStamp&) const noexcept = default;
};

namespace {

std::filesystem::path normalisePath(const std::filesystem::path& path) {
    std::error_code error;
    auto absolute = path;
    if (!absolute.is_absolute()) absolute = std::filesystem::absolute(path, error);
    if (error) absolute = path;
    error.clear();
    const auto canonical = std::filesystem::weakly_canonical(absolute, error);
    return (error ? absolute.lexically_normal() : canonical).lexically_normal();
}

} // namespace

EngineScriptHotReloader::FileStamp EngineScriptHotReloader::readFileStamp(
    const std::filesystem::path& path) noexcept {
    std::error_code error;
    const auto exists = std::filesystem::is_regular_file(path, error);
    if (error || !exists) return {};
    error.clear();
    const auto writeTime = std::filesystem::last_write_time(path, error);
    if (error) return {};
    error.clear();
    const auto size = std::filesystem::file_size(path, error);
    if (error) return {};
    return { true, writeTime, size };
}

EngineScriptHotReloader::EngineScriptHotReloader(
    std::filesystem::path rootScript, EngineScriptCompileOptions options,
    std::chrono::milliseconds pollingInterval)
    : rootScript_(normalisePath(rootScript)), options_(std::move(options)),
      pollingInterval_(std::max(std::chrono::milliseconds { 10 }, pollingInterval)),
      state_(std::make_shared<const EngineScriptReloadState>()) {
    watchedFiles_.push_back(rootScript_);
    watchedStamps_.push_back(readFileStamp(rootScript_));
}

EngineScriptHotReloader::~EngineScriptHotReloader() {
    stop();
}

void EngineScriptHotReloader::start() {
    std::lock_guard lock(lifecycleMutex_);
    if (worker_.joinable()) return;
    stopRequested_.store(false, std::memory_order_release);
    running_.store(true, std::memory_order_release);
    worker_ = std::thread([this] { workerLoop(); });
}

void EngineScriptHotReloader::stop() noexcept {
    std::thread worker;
    {
        std::lock_guard lock(lifecycleMutex_);
        if (!worker_.joinable()) {
            running_.store(false, std::memory_order_release);
            return;
        }
        stopRequested_.store(true, std::memory_order_release);
        wakeCondition_.notify_all();
        worker = std::move(worker_);
    }
    worker.join();
    running_.store(false, std::memory_order_release);
}

void EngineScriptHotReloader::requestReload() noexcept {
    reloadRequest_.fetch_add(1, std::memory_order_release);
    wakeCondition_.notify_all();
}

bool EngineScriptHotReloader::running() const noexcept {
    return running_.load(std::memory_order_acquire);
}

std::shared_ptr<const EngineScriptReloadState> EngineScriptHotReloader::poll() const noexcept {
    return state_.load(std::memory_order_acquire);
}

std::uint64_t EngineScriptHotReloader::revision() const noexcept {
    return poll()->revision;
}

bool EngineScriptHotReloader::watchedFilesChanged() const {
    if (watchedFiles_.size() != watchedStamps_.size()) return true;
    for (std::size_t index = 0; index < watchedFiles_.size(); ++index)
        if (!(readFileStamp(watchedFiles_[index]) == watchedStamps_[index])) return true;
    return false;
}

void EngineScriptHotReloader::refreshWatchList(
    const std::vector<std::filesystem::path>& attemptedDependencies,
    const std::vector<std::filesystem::path>& validDependencies) {
    watchedFiles_.clear();
    watchedFiles_.push_back(rootScript_);
    for (const auto& dependency : attemptedDependencies)
        watchedFiles_.push_back(normalisePath(dependency));
    for (const auto& dependency : validDependencies)
        watchedFiles_.push_back(normalisePath(dependency));
    std::sort(watchedFiles_.begin(), watchedFiles_.end());
    watchedFiles_.erase(std::unique(watchedFiles_.begin(), watchedFiles_.end()), watchedFiles_.end());
    watchedStamps_.clear();
    watchedStamps_.reserve(watchedFiles_.size());
    for (const auto& path : watchedFiles_) watchedStamps_.push_back(readFileStamp(path));
}

void EngineScriptHotReloader::compileAndPublish() {
    std::vector<FileStamp> stampsBefore;
    const auto filesBefore = watchedFiles_;
    stampsBefore.reserve(watchedFiles_.size());
    for (const auto& path : watchedFiles_) stampsBefore.push_back(readFileStamp(path));

    auto compiled = compiler_.compileFile(rootScript_, options_);
    const auto previous = poll();
    auto next = std::make_shared<EngineScriptReloadState>();
    next->attempt = previous->attempt + 1;
    next->diagnostics = std::move(compiled.diagnostics);
    next->dependencies = compiled.dependencies;
    if (compiled) {
        if (previous->revision == std::numeric_limits<std::uint64_t>::max()) {
            next->config = previous->config;
            next->revision = previous->revision;
            next->lastAttemptSucceeded = false;
            next->diagnostics.push_back({ ScriptDiagnosticSeverity::error, "ES500",
                "The hot-reload revision counter cannot be incremented.", { rootScript_, 1, 1 } });
        } else {
            next->config = std::make_shared<const EngineConfig>(std::move(*compiled.config));
            next->revision = previous->revision + 1;
            next->lastAttemptSucceeded = true;
            validDependencies_ = next->dependencies;
        }
    } else {
        next->config = previous->config;
        next->revision = previous->revision;
        next->lastAttemptSucceeded = false;
    }
    refreshWatchList(next->dependencies, validDependencies_);
    state_.store(std::move(next), std::memory_order_release);

    // A file modified while compilation was in progress needs one more pass; recording only the
    // post-compile stamp could otherwise miss that race.
    for (std::size_t index = 0; index < stampsBefore.size(); ++index) {
        if (!(stampsBefore[index] == readFileStamp(filesBefore[index]))) {
            reloadRequest_.fetch_add(1, std::memory_order_release);
            break;
        }
    }
}

void EngineScriptHotReloader::workerLoop() {
    auto observedRequest = std::uint64_t { 0 };
    while (!stopRequested_.load(std::memory_order_acquire)) {
        const auto requested = reloadRequest_.load(std::memory_order_acquire);
        if (requested != observedRequest || watchedFilesChanged()) {
            compileAndPublish();
            observedRequest = requested;
            continue;
        }
        std::unique_lock lock(lifecycleMutex_);
        wakeCondition_.wait_for(lock, pollingInterval_, [this, observedRequest] {
            return stopRequested_.load(std::memory_order_acquire)
                || reloadRequest_.load(std::memory_order_acquire) != observedRequest;
        });
    }
    running_.store(false, std::memory_order_release);
}

} // namespace enginelab::scripting
