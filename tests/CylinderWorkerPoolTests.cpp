#include <enginelab/simulation/CylinderWorkerPool.hpp>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <iostream>

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#elif defined(__linux__)
#include <sched.h>
#endif

namespace {
void require(bool condition, const char* message) {
    if (condition) return;
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
}

// Holds the whole process to one logical processor, before any pool thread
// exists (Linux affinity is per thread and inherited by the threads created
// after it is set). Returns false where the platform offers no such control.
bool holdToOneProcessor() {
#if defined(_WIN32)
    DWORD_PTR process = 0;
    DWORD_PTR system = 0;
    if (!GetProcessAffinityMask(GetCurrentProcess(), &process, &system) || process == 0)
        return false;
    return SetProcessAffinityMask(GetCurrentProcess(), process & (~process + 1)) != 0;
#elif defined(__linux__)
    cpu_set_t allowed;
    CPU_ZERO(&allowed);
    if (sched_getaffinity(0, sizeof(allowed), &allowed) != 0) return false;
    for (int cpu = 0; cpu < CPU_SETSIZE; ++cpu) {
        if (!CPU_ISSET(cpu, &allowed)) continue;
        cpu_set_t one;
        CPU_ZERO(&one);
        CPU_SET(cpu, &one);
        return sched_setaffinity(0, sizeof(one), &one) == 0;
    }
    return false;
#else
    return false;
#endif
}

struct Tally {
    std::atomic<std::size_t> calls { 0 };
    std::atomic<std::size_t> indexSum { 0 };
};

void count(void* context, std::size_t index) noexcept {
    auto& tally = *static_cast<Tally*>(context);
    tally.calls.fetch_add(1, std::memory_order_relaxed);
    tally.indexSum.fetch_add(index, std::memory_order_relaxed);
}
}

int main() {
    // A pool with more threads than free processors: a release runner sharing
    // its cores with a second test, or a two-core laptop. The pool sizes itself
    // from hardware_concurrency(), which an affinity mask or a busy host does
    // not lower. A master that only spins while it waits for its workers then
    // holds the one processor they need until the scheduler preempts it, so
    // every dispatch costs a time slice: the LS3 sweep took 7 s on twelve
    // processors, 61 s on two, and never finished on one.
    if (!holdToOneProcessor()) {
        std::cout << "CylinderWorkerPool tests skipped: no affinity control\n";
        return 0;
    }
    enginelab::CylinderWorkerPool pool(2);
    Tally tally;
    constexpr std::size_t dispatches = 2'000;
    constexpr std::size_t items = 3;
    const auto start = std::chrono::steady_clock::now();
    for (std::size_t dispatch = 0; dispatch < dispatches; ++dispatch)
        pool.runIndices(items, &count, &tally);
    const auto seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
    require(tally.calls.load() == dispatches * items,
        "every index must run exactly once per dispatch");
    require(tally.indexSum.load() == dispatches * (0 + 1 + 2),
        "each dispatch must run the indices it was given");
    std::cout << dispatches << " dispatches on one processor: " << seconds << " s\n";
    // The physics thread dispatches ~40,000 times per simulated second, so a
    // dispatch that cannot hand the processor over in well under a
    // millisecond is a simulator that never reaches real time.
    require(seconds < 2.0,
        "a pool sharing one processor must hand it to its workers, not spin it away");
    std::cout << "CylinderWorkerPool tests passed\n";
    return 0;
}
