#include <enginelab/foundation/ThreadScheduling.hpp>

#include <bit>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <vector>
#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <Windows.h>
#endif

namespace {
void require(bool condition, const char* message) {
    if (condition) return;
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
}
}

int main() {
    using enginelab::platform::CpuSetDescription;
    using enginelab::platform::fastestCpuSets;

    require(fastestCpuSets({}).empty(), "no CPU sets reported must restrict nothing");

    // A Ryzen 7 8840U: sixteen logical processors of one class.
    std::vector<CpuSetDescription> uniform;
    for (std::uint32_t id = 256; id < 272; ++id) uniform.push_back({ id, 0 });
    require(fastestCpuSets(uniform).empty(),
        "a uniform processor must not be restricted: it would only take cores "
        "away from the scheduler");

    // An i9-14900K: 8 performance cores with SMT (class 1, sixteen logical
    // processors) and 16 efficiency cores (class 0), interleaved as Windows
    // may enumerate them.
    std::vector<CpuSetDescription> hybrid;
    std::uint32_t id = 256;
    for (int index = 0; index < 16; ++index) hybrid.push_back({ id++, 1 });
    for (int index = 0; index < 16; ++index) hybrid.push_back({ id++, 0 });
    std::swap(hybrid[3], hybrid[20]);
    const auto fast = fastestCpuSets(hybrid);
    require(fast.size() == 16, "a hybrid part must be held to all its performance cores");
    for (const auto chosen : fast)
        for (const auto& cpuSet : hybrid)
            if (cpuSet.id == chosen)
                require(cpuSet.efficiencyClass == 1,
                    "only performance-class CPU sets may be chosen");

    // Three classes (some mobile parts add low-power efficiency cores): the
    // fastest wins, not "anything above the slowest".
    const std::vector<CpuSetDescription> threeClasses {
        { 1, 0 }, { 2, 1 }, { 3, 2 }, { 4, 2 }, { 5, 1 } };
    const auto top = fastestCpuSets(threeClasses);
    require(top.size() == 2 && top[0] == 3 && top[1] == 4,
        "with three classes only the fastest class may be chosen");

    // The live request must be harmless on any machine, and may only hold a
    // thread to processors the machine has.
    std::thread probe([] {
        enginelab::platform::requestLatencyCriticalProcess();
        const auto result = enginelab::platform::requestLatencyCriticalThread();
        const auto logical = std::thread::hardware_concurrency();
        require(logical == 0 || result.heldToCpuSets <= logical,
            "a thread cannot be held to more CPU sets than the machine has");
        std::cout << "this machine: throttling opt-out "
                  << (result.throttlingOptOut ? "granted" : "refused")
                  << ", held to " << result.heldToCpuSets << " CPU sets"
                  << " (0 = uniform processor, not restricted)\n";
    });
    probe.join();

    // A process held to two processors sizes its pools for two, whatever the
    // machine has.
    const auto unrestricted = enginelab::platform::usableProcessorCount();
    require(unrestricted >= 1, "a process may run on at least one processor");
#if defined(_WIN32)
    DWORD_PTR processMask = 0;
    DWORD_PTR systemMask = 0;
    if (GetProcessAffinityMask(GetCurrentProcess(), &processMask, &systemMask)
        && std::popcount(static_cast<std::uint64_t>(systemMask)) > 2) {
        const auto lowestBit = [](DWORD_PTR mask) { return mask & (~mask + 1); };
        const auto first = lowestBit(systemMask);
        const auto second = lowestBit(systemMask & ~first);
        require(SetProcessAffinityMask(GetCurrentProcess(), first | second) != 0,
            "the test can restrict its own affinity");
        const auto restricted = enginelab::platform::usableProcessorCount();
        SetProcessAffinityMask(GetCurrentProcess(), processMask);
        std::cout << "usable processors: " << unrestricted << ", held to two: " << restricted << '\n';
        require(restricted == 2, "a process held to two processors may use two");
    }
#endif
    std::cout << "ThreadScheduling tests passed\n";
    return 0;
}
