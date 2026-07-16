#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <type_traits>

namespace enginelab {

/** Fixed-capacity lock-free queue for one simulation producer and one audio consumer. */
template <typename T, std::size_t Capacity>
class SpscQueue final {
    static_assert(Capacity >= 2 && (Capacity & (Capacity - 1)) == 0,
                  "SPSC capacity must be a power of two");
    static_assert(std::is_trivially_copyable_v<T>,
                  "Realtime queue payloads must be trivially copyable");
public:
    static constexpr std::size_t usableCapacity = Capacity - 1;

    [[nodiscard]] bool tryPush(const T& value) noexcept {
        const auto write = writeIndex_.load(std::memory_order_relaxed);
        const auto next = (write + 1) & mask;
        if (next == readIndex_.load(std::memory_order_acquire)) return false;
        storage_[write] = value;
        writeIndex_.store(next, std::memory_order_release);
        return true;
    }

    [[nodiscard]] bool tryPop(T& value) noexcept {
        const auto read = readIndex_.load(std::memory_order_relaxed);
        if (read == writeIndex_.load(std::memory_order_acquire)) return false;
        value = storage_[read];
        readIndex_.store((read + 1) & mask, std::memory_order_release);
        return true;
    }

private:
    static constexpr std::size_t mask = Capacity - 1;
    alignas(64) std::array<T, Capacity> storage_ {};
    alignas(64) std::atomic<std::size_t> writeIndex_ { 0 };
    alignas(64) std::atomic<std::size_t> readIndex_ { 0 };
};

} // namespace enginelab
