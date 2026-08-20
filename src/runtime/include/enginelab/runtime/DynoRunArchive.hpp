#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

#include <atomic>
#include <cstdint>
#include <mutex>
#include <vector>

namespace enginelab {

/**
 * Thread-safe, application-lifetime store for terminal dyno runs.
 *
 * EngineRuntime instances are deliberately replaceable when the user changes
 * engine configuration. The archive is not: it owns monotonically increasing
 * identifiers and completed/cancelled/failed records across those replacements.
 * Its revision lets a 30 Hz UI avoid copying every stored point unless the
 * collection actually changed.
 */
class DynoRunArchive final {
public:
    [[nodiscard]] std::uint64_t reserveRunId();
    [[nodiscard]] bool append(DynoRun run);
    [[nodiscard]] bool erase(std::uint64_t id);
    [[nodiscard]] std::vector<DynoRun> snapshot() const;
    [[nodiscard]] std::uint64_t revision() const noexcept {
        return revision_.load(std::memory_order_acquire);
    }

private:
    mutable std::mutex mutex_;
    std::vector<DynoRun> runs_;
    std::uint64_t nextRunId_ { 1 };
    std::atomic<std::uint64_t> revision_ { 0 };
};

} // namespace enginelab
