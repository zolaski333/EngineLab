#include <enginelab/simulation/SubstepParallel.hpp>

#include <array>
#include <atomic>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <thread>

namespace {
void require(bool condition, const char* message) {
    if (condition) return;
    std::cerr << "FAILED: " << message << '\n';
    std::exit(EXIT_FAILURE);
}
}

int main() {
    auto& executor = enginelab::SubstepParallelExecutor::shared();
    constexpr std::array<std::size_t, 5> counts { 1, 7, 8, 12, 32 };
    for (const auto count : counts) {
        std::array<std::atomic<unsigned int>, 32> hits {};
        for (int pass = 0; pass < 100; ++pass)
            executor.parallelFor(count, [&hits](std::size_t index) {
                hits[index].fetch_add(1, std::memory_order_relaxed);
            });
        for (std::size_t index = 0; index < hits.size(); ++index)
            require(hits[index].load(std::memory_order_relaxed) == (index < count ? 100U : 0U),
                    "parallelFor must execute every requested index exactly once per job");
    }

    // EngineSimulator instances may be stepped on separate test/application
    // threads. Both callers share this singleton, so their type-erased jobs must
    // never overlap or overwrite one another.
    using HitArray = std::array<std::atomic<unsigned int>, 12>;
    HitArray firstHits {};
    HitArray secondHits {};
    std::atomic<unsigned int> ready { 0 };
    std::atomic<bool> start { false };
    const auto caller = [&](HitArray& hits, std::size_t count) {
        ready.fetch_add(1, std::memory_order_release);
        while (!start.load(std::memory_order_acquire)) std::this_thread::yield();
        for (int pass = 0; pass < 250; ++pass)
            executor.parallelFor(count, [&hits](std::size_t index) {
                hits[index].fetch_add(1, std::memory_order_relaxed);
            });
    };
    std::thread first(caller, std::ref(firstHits), std::size_t { 8 });
    std::thread second(caller, std::ref(secondHits), std::size_t { 12 });
    while (ready.load(std::memory_order_acquire) != 2U) std::this_thread::yield();
    start.store(true, std::memory_order_release);
    first.join();
    second.join();
    for (std::size_t index = 0; index < firstHits.size(); ++index) {
        require(firstHits[index].load(std::memory_order_relaxed) == (index < 8 ? 250U : 0U),
                "concurrent eight-cylinder dispatch must remain isolated");
        require(secondHits[index].load(std::memory_order_relaxed) == 250U,
                "concurrent twelve-cylinder dispatch must remain isolated");
    }

    std::cout << "Sub-step parallel executor tests passed\n";
    return EXIT_SUCCESS;
}
