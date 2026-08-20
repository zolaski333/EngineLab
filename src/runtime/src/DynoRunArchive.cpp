#include <enginelab/runtime/DynoRunArchive.hpp>

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>

namespace enginelab {

std::uint64_t DynoRunArchive::reserveRunId() {
    const std::scoped_lock lock(mutex_);
    if (nextRunId_ == std::numeric_limits<std::uint64_t>::max())
        throw std::overflow_error("dyno run identifier space exhausted");
    return nextRunId_++;
}

bool DynoRunArchive::append(DynoRun run) {
    const std::scoped_lock lock(mutex_);
    // IDs reserved by this store are unique. Refuse accidental replay instead
    // of silently replacing historical evidence with a different run.
    if (run.id == 0
        || run.id == std::numeric_limits<std::uint64_t>::max()
        || std::ranges::any_of(
            runs_, [&run](const DynoRun& candidate) {
                return candidate.id == run.id;
            }))
        return false;
    nextRunId_ = std::max(nextRunId_, run.id + 1);
    runs_.push_back(std::move(run));
    revision_.fetch_add(1, std::memory_order_release);
    return true;
}

bool DynoRunArchive::erase(std::uint64_t id) {
    const std::scoped_lock lock(mutex_);
    const auto oldSize = runs_.size();
    std::erase_if(runs_, [id](const DynoRun& run) { return run.id == id; });
    if (runs_.size() == oldSize) return false;
    revision_.fetch_add(1, std::memory_order_release);
    return true;
}

std::vector<DynoRun> DynoRunArchive::snapshot() const {
    const std::scoped_lock lock(mutex_);
    return runs_;
}

} // namespace enginelab
