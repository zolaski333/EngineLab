#pragma once

/**
 * Asks the operating system to treat the simulation's threads as latency
 * critical.
 *
 * Why this exists: the physics thread must finish a 4.17 ms step every
 * 4.17 ms, and from three cylinders up it crosses a full barrier with the
 * resident workers tens of thousands of times a second (CylinderWorkerPool),
 * so the step is as slow as the slowest of those threads. Windows 11 on a
 * hybrid processor (Intel 12th gen and later) decides per thread whether it
 * runs on a performance or an efficiency core, and under EcoQoS (power
 * throttling) at an efficiency clock; it decides from heuristics -- window
 * focus, visibility -- that know nothing of a real-time simulation. One worker
 * placed on an efficiency core holds every barrier. Nothing here changes what
 * is computed, only where and how fast.
 *
 * Every request is advisory: a call the OS refuses (an older Windows, another
 * platform) leaves the thread as it was.
 */

#include <cstdint>
#include <span>
#include <vector>

namespace enginelab::platform {

/** One logical processor as the OS describes it (Windows CPU sets). */
struct CpuSetDescription {
    std::uint32_t id { 0 };
    /** Higher is faster: 0 for efficiency cores, 1 for performance cores on
     *  an Intel hybrid part; one value for every core on a uniform one. */
    std::uint8_t efficiencyClass { 0 };
    /** The physical core it belongs to: two logical processors of one core
     *  (SMT, Hyper-Threading) share an index. */
    std::uint8_t coreIndex { 0 };
};

/**
 * The CPU sets a latency-critical thread should be held to: those of the
 * fastest class, and only when the machine HAS more than one class. On a
 * uniform processor the result is empty, which means "do not restrict" --
 * restricting there would only take cores away from the scheduler.
 */
[[nodiscard]] std::vector<std::uint32_t> fastestCpuSets(
    std::span<const CpuSetDescription> cpuSets);

/**
 * What requestLatencyCriticalThread() holds a thread to: the fastest class
 * (as above), then one logical processor per physical core, the first listed.
 * The simulation's master and its pool workers cross a barrier tens of
 * thousands of times a second, so two of them on the two halves of one core
 * run the slower half's pace: the V4-2S held to one processor a core ran
 * 1.28 x real time against 1.20 (journal 2026-11-21). Empty, "do not
 * restrict", when that would keep every processor (no SMT, uniform). Fewer
 * than four cores of the fastest class (the master and three workers spin)
 * keep both halves of each, as fastestCpuSets() does.
 */
[[nodiscard]] std::vector<std::uint32_t> latencyCriticalCpuSets(
    std::span<const CpuSetDescription> cpuSets);

/**
 * Opts the whole process out of power throttling: no EcoQoS clock, and the
 * timer resolution the process asked for (JUCE asks for 1 ms) is kept even
 * while its window is hidden. Call once, early; repeating it is harmless.
 */
void requestLatencyCriticalProcess() noexcept;

/** What requestLatencyCriticalThread() obtained, for diagnostics and tests. */
struct LatencyCriticalThreadResult {
    bool throttlingOptOut { false };
    /** Number of CPU sets the thread was held to; 0 when not restricted. */
    std::size_t heldToCpuSets { 0 };
};

/**
 * For the calling thread: opts it out of power throttling and holds it to
 * latencyCriticalCpuSets(): the performance cores of a hybrid processor, one
 * logical processor each when SMT gives two.
 */
LatencyCriticalThreadResult requestLatencyCriticalThread() noexcept;

/**
 * Measurement only (EngineLabRealtimeBudgetHarness --power-throttled): turns
 * every later request above into its opposite -- EcoQoS on, the timer
 * resolution released, no core preference -- which is what Windows may do to
 * a process it judges in the background. Lets the cost of what those requests
 * prevent be measured on any machine. Call before the runtime starts.
 */
void simulateBackgroundThrottlingForMeasurement() noexcept;

/**
 * Measurement only (EngineLabRealtimeBudgetHarness --smt-siblings): holds
 * latency-critical threads to the fastest class only, both halves of a core
 * allowed, as before 2026-11-21. Call before the runtime starts.
 */
void allowSmtSiblingsForMeasurement() noexcept;

/**
 * The logical processors this process may run on: its affinity mask's when
 * one restricts it (a container, `start /affinity`, taskset), otherwise
 * std::thread::hardware_concurrency(), which ignores the mask. A pool sized
 * from the latter on two allowed processors put four spinning threads on
 * them (journal 2026-10-25).
 */
[[nodiscard]] unsigned usableProcessorCount() noexcept;

} // namespace enginelab::platform
