#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <thread>
#include <type_traits>
#include <vector>

namespace enginelab {

/**
 * Fork-join executor for the per-cylinder gas sub-step loop.
 *
 * The simulator dispatches a small, embarrassingly-parallel body (one call per
 * cylinder) tens of thousands of times per second — once per gas sub-step. A
 * general thread pool that keeps its workers spinning (OpenMP's default, most
 * lock-free pools) would peg every core during the idle gap the realtime loop
 * sleeps through between frames. This executor instead has each worker spin only
 * briefly after a job — long enough to catch the next back-to-back sub-step with
 * low latency — then block on a condition variable, so between frames the workers
 * cost nothing.
 *
 * The body is passed by reference and invoked through a plain function pointer,
 * so dispatch allocates nothing. Work is split into contiguous static chunks
 * across the worker threads plus the calling thread; parallelFor() blocks until
 * every index has been processed.
 */
class SubstepParallelExecutor final {
public:
    static SubstepParallelExecutor& shared() {
        static SubstepParallelExecutor executor;
        return executor;
    }

    /** Worker threads plus the calling thread. 1 means "run everything inline". */
    [[nodiscard]] std::size_t participantCount() const noexcept { return workerCount_ + 1; }

    /** Run body(i) for every i in [0, count) and block until all have completed.
     *  body must be safe to invoke concurrently for distinct i. */
    template <class Body>
    void parallelFor(std::size_t count, Body&& body, bool expectAnotherJobSoon = false) {
        if (count == 0) return;
        // The executor is shared by every EngineSimulator. Serialising dispatch
        // prevents two simulation threads from overwriting the type-erased job
        // fields while either job is still in flight. The cylinder work itself
        // remains parallel; concurrent simulators take turns using the one pool
        // instead of oversubscribing the machine with competing worker sets.
        std::unique_lock<std::mutex> dispatchLock(dispatchMutex_);
        const std::size_t participants = count < workerCount_ + 1 ? count : workerCount_ + 1;
        if (participants <= 1) {
            for (std::size_t index = 0; index < count; ++index) body(index);
            return;
        }
        using BodyType = std::remove_reference_t<Body>;
        body_ = const_cast<void*>(static_cast<const void*>(&body));
        invoke_ = [](void* erased, std::size_t index) {
            (*static_cast<BodyType*>(erased))(index);
        };
        count_ = count;
        activeParticipants_ = participants;
        spinAfter_ = expectAnotherJobSoon;
        remaining_.store(participants - 1, std::memory_order_relaxed);
        {
            // Publish only to workers which participate in this job. A single
            // global generation lets a non-participant observe a job after the
            // caller has already seen remaining_ reach zero and started writing
            // the next job fields. Per-worker generations make the completion
            // count a complete lifetime barrier for every reader of this job.
            std::lock_guard<std::mutex> lock(mutex_);
            ++jobGeneration_;
            for (std::size_t worker = 0; worker + 1 < participants; ++worker)
                workerGenerations_[worker].store(jobGeneration_, std::memory_order_release);
        }
        condition_.notify_all();
        runChunk(0, participants, count);
        // Spin-wait for the workers; the chunks are balanced so this is short.
        while (remaining_.load(std::memory_order_acquire) != 0) {
            std::this_thread::yield();
        }
        body_ = nullptr;
        invoke_ = nullptr;
    }

    SubstepParallelExecutor(const SubstepParallelExecutor&) = delete;
    SubstepParallelExecutor& operator=(const SubstepParallelExecutor&) = delete;

    ~SubstepParallelExecutor() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_.store(true, std::memory_order_release);
        }
        condition_.notify_all();
        for (auto& worker : workers_) worker.join();
    }

private:
    SubstepParallelExecutor() {
        const auto hardware = std::thread::hardware_concurrency();
        const std::size_t available = hardware > 1 ? static_cast<std::size_t>(hardware) - 1 : 0;
        workerCount_ = available < maximumWorkers ? available : maximumWorkers;
        workers_.reserve(workerCount_);
        for (std::size_t worker = 0; worker < workerCount_; ++worker)
            workers_.emplace_back([this, worker] { workerLoop(worker, worker + 1); });
    }

    void runChunk(std::size_t participantIndex, std::size_t participants, std::size_t count) noexcept {
        const auto base = count / participants;
        const auto remainder = count % participants;
        const auto begin = participantIndex * base + (participantIndex < remainder ? participantIndex : remainder);
        const auto extra = participantIndex < remainder ? std::size_t { 1 } : std::size_t { 0 };
        const auto end = begin + base + extra;
        for (std::size_t index = begin; index < end; ++index) invoke_(body_, index);
    }

    void workerLoop(std::size_t workerIndex, std::size_t participantIndex) {
        // Start from the pre-job generation (0). A worker whose OS thread only
        // starts running after the first job has already been published must still
        // observe that job; seeding `seen` from the live counter here would let it
        // adopt the in-flight generation and never decrement its completion, so
        // the dispatcher would wait forever.
        std::uint64_t seen = 0;
        auto shouldSpin = false;
        while (true) {
            std::uint64_t current = workerGenerations_[workerIndex].load(std::memory_order_acquire);
            if (current == seen) {
                // Spin only when the dispatcher explicitly says another gas
                // sub-step in the same frame is imminent. The final sub-step
                // sends workers directly to the condition variable, so they do
                // not burn CPU while the realtime loop sleeps between frames.
                if (shouldSpin) {
                    for (int attempt = 0; attempt < spinAttempts && current == seen; ++attempt) {
                        std::this_thread::yield();
                        current = workerGenerations_[workerIndex].load(std::memory_order_acquire);
                    }
                }
                if (current == seen) {
                    std::unique_lock<std::mutex> lock(mutex_);
                    condition_.wait(lock, [this, workerIndex, seen] {
                        return workerGenerations_[workerIndex].load(std::memory_order_acquire) != seen
                            || stop_.load(std::memory_order_acquire);
                    });
                    current = workerGenerations_[workerIndex].load(std::memory_order_acquire);
                }
            }
            if (stop_.load(std::memory_order_acquire)) return;
            seen = current;
            runChunk(participantIndex, activeParticipants_, count_);
            shouldSpin = spinAfter_;
            remaining_.fetch_sub(1, std::memory_order_acq_rel);
        }
    }

    static constexpr std::size_t maximumWorkers = 15;
    static constexpr int spinAttempts = 2048;

    std::size_t workerCount_ { 0 };
    std::vector<std::thread> workers_;
    std::mutex dispatchMutex_;
    std::mutex mutex_;
    std::condition_variable condition_;
    std::array<std::atomic<std::uint64_t>, maximumWorkers> workerGenerations_ {};
    std::uint64_t jobGeneration_ { 0 };
    std::atomic<bool> stop_ { false };
    std::atomic<std::size_t> remaining_ { 0 };
    // Written by the dispatching thread before the generation bump that releases
    // them, then only read by workers for the current job — no concurrent write.
    std::size_t count_ { 0 };
    std::size_t activeParticipants_ { 1 };
    bool spinAfter_ { false };
    void* body_ { nullptr };
    void (*invoke_)(void*, std::size_t) { nullptr };
};

} // namespace enginelab
