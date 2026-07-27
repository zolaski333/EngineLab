#pragma once

/**
 * A persistent worker pool for the per-cylinder phase of a gas sub-step.
 *
 * READ THIS BEFORE CHANGING IT. A per-cylinder fork-join already existed in
 * this project and was REMOVED after measurement, in four variants, all of
 * which lost to running inline (CLAUDE.md and docs/physics-audit.md carry the
 * numbers). This pool is not a reversal of that decision; it is aimed at a
 * different body.
 *
 * What was removed dispatched `processCylinder`, which the profiler puts at
 * 5-10% of a mechanical sub-step -- roughly 0.6 us of work per cylinder. No
 * barrier amortises that: the synchronisation was most of the cost, and the
 * spin used to shorten it stole cycles from the very thread the barrier was
 * waiting on. What is dispatched here is the 1-D intake runner advance, which
 * the same profiler puts at 75-84%: about 5 us per cylinder on a nine-cell
 * runner, an order of magnitude larger. The barrier overhead per unit of work
 * is therefore roughly a tenth of what it was, which is the entire reason to
 * try again.
 *
 * Design choices, each of which is a lesson from that failure:
 *
 *  - The calling thread is a worker. A master that only waits is one core
 *    wasted and one more barrier edge to cross.
 *  - Work is claimed from a shared atomic counter, not statically partitioned.
 *    Cylinder cost is bimodal -- a cylinder with its intake valve open takes
 *    two half-advances, one with it shut takes one -- so a static split would
 *    leave half the workers idle at the barrier.
 *  - Workers spin briefly, then yield, then park on a condition variable. A
 *    pure spin burns cores that the audio callback needs; a pure condvar
 *    cannot keep up with ~40,000 dispatches a second. The parking threshold is
 *    set so that a running simulation never reaches it and a stopped one never
 *    stays out of it.
 *  - Results must not depend on the schedule. The body may write only state
 *    private to its index; everything shared is applied serially, in index
 *    order, by the caller after `runIndices` returns. That is what keeps the
 *    simulator bit-reproducible, which matters here more than usual because
 *    the catalogue's idles are ULP-sensitive attractors.
 *  - A dispatch is a full barrier: it returns only once EVERY worker has
 *    acknowledged this generation, not merely once every item is done. That
 *    is stronger than it looks necessary, and it is load-bearing -- see the
 *    note on `arrived_` below.
 *
 * There is one master. `runIndices` and `runIndexRange` are not reentrant and
 * must be called from the same thread for the pool's lifetime.
 */

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <new>
#include <thread>
#include <vector>

namespace enginelab {

class CylinderWorkerPool final {
public:
    /** Type-erased body so a dispatch costs no allocation and no indirect
     *  std::function call. */
    using Body = void (*)(void*, std::size_t) noexcept;

    explicit CylinderWorkerPool(std::size_t workerCount) {
        workers_.reserve(workerCount);
        for (std::size_t index = 0; index < workerCount; ++index)
            workers_.emplace_back([this] { workerLoop(); });
    }

    CylinderWorkerPool(const CylinderWorkerPool&) = delete;
    CylinderWorkerPool& operator=(const CylinderWorkerPool&) = delete;

    ~CylinderWorkerPool() {
        {
            const std::scoped_lock lock(parkMutex_);
            stopping_.store(true, std::memory_order_relaxed);
            generation_.fetch_add(1, std::memory_order_release);
        }
        parkSignal_.notify_all();
        for (auto& worker : workers_) if (worker.joinable()) worker.join();
    }

    [[nodiscard]] std::size_t workerCount() const noexcept { return workers_.size(); }

    /**
     * Calls `body(context, i)` exactly once for every i in [0, count), and
     * returns only when all of them have completed. The calling thread runs a
     * share of them itself.
     */
    void runIndices(std::size_t count, Body body, void* context) noexcept {
        runIndexRange(0, count, body, context);
    }

    /**
     * Calls `body(context, i)` exactly once for every i in [begin, end), and
     * returns only when all of them have completed. The calling thread runs a
     * share of them itself.
     */
    void runIndexRange(std::size_t begin, std::size_t end, Body body,
                       void* context) noexcept {
        if (end <= begin) return;
        if (workers_.empty()) {
            for (std::size_t index = begin; index < end; ++index) body(context, index);
            return;
        }
        body_ = body;
        context_ = context;
        count_.store(end, std::memory_order_relaxed);
        nextIndex_.store(begin, std::memory_order_relaxed);
        arrived_.store(0, std::memory_order_relaxed);
        // Release: everything above must be visible to a worker that observes
        // the new generation.
        generation_.fetch_add(1, std::memory_order_release);
        if (parked_.load(std::memory_order_acquire) > 0) {
            const std::scoped_lock lock(parkMutex_);
            parkSignal_.notify_all();
        }
        drainQueue();
        // Acquire: every worker's writes must be visible once it has arrived.
        const auto expected = workers_.size();
        while (arrived_.load(std::memory_order_acquire) != expected) spinPause();
    }

private:
    // Long enough to cover a barrier crossing on a loaded desktop, short
    // enough that an idle pool parks within a millisecond.
    static constexpr int spinsBeforeYield = 512;
    static constexpr int yieldsBeforePark = 64;

    static void spinPause() noexcept {
#if defined(_MSC_VER) && (defined(_M_X64) || defined(_M_IX86))
        _mm_pause();
#elif defined(__x86_64__) || defined(__i386__)
        __builtin_ia32_pause();
#elif defined(__aarch64__)
        __asm__ __volatile__("yield");
#endif
    }

    void drainQueue() noexcept {
        const auto count = count_.load(std::memory_order_relaxed);
        for (;;) {
            const auto index = nextIndex_.fetch_add(1, std::memory_order_relaxed);
            if (index >= count) return;
            body_(context_, index);
        }
    }

    void workerLoop() noexcept {
        // Deliberately NOT `generation_.load()`. A worker thread that has not
        // started yet when the first job is published would latch that job's
        // generation as already seen, never arrive, and hang the master
        // forever. Zero is the known value before any dispatch, so a late
        // starter observes a difference and arrives.
        std::uint64_t seen = 0;
        for (;;) {
            auto spins = 0;
            auto yields = 0;
            std::uint64_t current = seen;
            while ((current = generation_.load(std::memory_order_acquire)) == seen) {
                if (++spins < spinsBeforeYield) { spinPause(); continue; }
                spins = 0;
                if (++yields < yieldsBeforePark) { std::this_thread::yield(); continue; }
                std::unique_lock lock(parkMutex_);
                parked_.fetch_add(1, std::memory_order_release);
                parkSignal_.wait(lock, [this, seen] {
                    return generation_.load(std::memory_order_acquire) != seen;
                });
                parked_.fetch_sub(1, std::memory_order_release);
                yields = 0;
            }
            seen = current;
            if (stopping_.load(std::memory_order_relaxed)) return;
            drainQueue();
            arrived_.fetch_add(1, std::memory_order_release);
        }
    }

#if defined(__cpp_lib_hardware_interference_size)
    static constexpr std::size_t cacheLine = std::hardware_destructive_interference_size;
#else
    static constexpr std::size_t cacheLine = 64;
#endif

    // Each of these is written by every participant on every dispatch, so they
    // are kept on separate lines: false sharing between the claim counter and
    // the completion counter alone doubled the barrier cost in the variant
    // this replaces.
    alignas(cacheLine) std::atomic<std::uint64_t> generation_ { 0 };
    alignas(cacheLine) std::atomic<std::size_t> nextIndex_ { 0 };
    // Counts WORKERS that have finished this generation, not ITEMS. Counting
    // items instead deadlocks, and it took nine hours of a hung test to find:
    // a worker that has observed a generation but not yet entered the queue is
    // not observable by the master, so the master can finish the job, publish
    // the next one, and only then have that worker enter -- claiming an item of
    // the NEW job and decrementing an item counter the master is about to
    // overwrite. The count is then permanently one too high and the master
    // spins forever (measured: one core at 100%, every worker parked). No guard
    // on "is a worker about to enter" can close that, because the window sits
    // between two of the worker's own instructions. Requiring every worker to
    // acknowledge generation g before g+1 exists removes the concept of a
    // stale worker entirely, which is also what makes it safe to publish
    // `body_` and `context_` as plain non-atomic fields.
    alignas(cacheLine) std::atomic<std::size_t> arrived_ { 0 };
    alignas(cacheLine) std::atomic<std::size_t> count_ { 0 };
    alignas(cacheLine) std::atomic<std::size_t> parked_ { 0 };
    std::atomic<bool> stopping_ { false };
    Body body_ { nullptr };
    void* context_ { nullptr };
    std::mutex parkMutex_;
    std::condition_variable parkSignal_;
    std::vector<std::thread> workers_;
};

} // namespace enginelab
