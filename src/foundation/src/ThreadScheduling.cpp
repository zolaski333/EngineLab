#include <enginelab/foundation/ThreadScheduling.hpp>

#include <algorithm>
#include <atomic>
#include <bit>
#include <cstddef>
#include <thread>
#if defined(__linux__)
#include <sched.h>
#endif
#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <Windows.h>
#endif

namespace enginelab::platform {

std::vector<std::uint32_t> fastestCpuSets(std::span<const CpuSetDescription> cpuSets) {
    if (cpuSets.empty()) return {};
    const auto [slowest, fastest] = std::ranges::minmax_element(
        cpuSets, {}, &CpuSetDescription::efficiencyClass);
    if (slowest->efficiencyClass == fastest->efficiencyClass) return {};
    std::vector<std::uint32_t> ids;
    for (const auto& cpuSet : cpuSets)
        if (cpuSet.efficiencyClass == fastest->efficiencyClass) ids.push_back(cpuSet.id);
    return ids;
}

namespace {
std::atomic<bool> simulateBackgroundThrottling { false };
}

void simulateBackgroundThrottlingForMeasurement() noexcept {
    simulateBackgroundThrottling.store(true, std::memory_order_relaxed);
}

#if defined(_WIN32)
namespace {

std::vector<CpuSetDescription> systemCpuSets() {
    ULONG length = 0;
    (void)GetSystemCpuSetInformation(nullptr, 0, &length, GetCurrentProcess(), 0);
    if (length == 0) return {};
    std::vector<std::byte> buffer(length);
    auto* first = reinterpret_cast<SYSTEM_CPU_SET_INFORMATION*>(buffer.data());
    if (!GetSystemCpuSetInformation(first, length, &length, GetCurrentProcess(), 0))
        return {};
    std::vector<CpuSetDescription> cpuSets;
    for (std::size_t offset = 0; offset < length;) {
        const auto* entry = reinterpret_cast<const SYSTEM_CPU_SET_INFORMATION*>(
            buffer.data() + offset);
        if (entry->Size == 0) break;
        if (entry->Type == CpuSetInformation)
            cpuSets.push_back({ entry->CpuSet.Id, entry->CpuSet.EfficiencyClass });
        offset += entry->Size;
    }
    return cpuSets;
}

// Read once: the topology does not change while the process runs.
const std::vector<std::uint32_t>& performanceCpuSets() noexcept {
    static const auto ids = []() noexcept -> std::vector<std::uint32_t> {
        try {
            return fastestCpuSets(systemCpuSets());
        } catch (...) {
            return {};
        }
    }();
    return ids;
}

} // namespace

void requestLatencyCriticalProcess() noexcept {
    PROCESS_POWER_THROTTLING_STATE state {};
    state.Version = PROCESS_POWER_THROTTLING_CURRENT_VERSION;
    // A bit in ControlMask with its StateMask bit clear means "never apply
    // this", as opposed to leaving the decision to the system's heuristics.
    state.ControlMask = PROCESS_POWER_THROTTLING_EXECUTION_SPEED
        | PROCESS_POWER_THROTTLING_IGNORE_TIMER_RESOLUTION;
    state.StateMask = simulateBackgroundThrottling.load(std::memory_order_relaxed)
        ? state.ControlMask : 0;
    (void)SetProcessInformation(GetCurrentProcess(), ProcessPowerThrottling,
                                &state, sizeof(state));
}

LatencyCriticalThreadResult requestLatencyCriticalThread() noexcept {
    LatencyCriticalThreadResult result;
    THREAD_POWER_THROTTLING_STATE state {};
    state.Version = THREAD_POWER_THROTTLING_CURRENT_VERSION;
    state.ControlMask = THREAD_POWER_THROTTLING_EXECUTION_SPEED;
    const auto throttled = simulateBackgroundThrottling.load(std::memory_order_relaxed);
    state.StateMask = throttled ? state.ControlMask : 0;
    const auto applied = SetThreadInformation(
        GetCurrentThread(), ThreadPowerThrottling, &state, sizeof(state)) != FALSE;
    if (throttled) return result;
    result.throttlingOptOut = applied;
    const auto& ids = performanceCpuSets();
    if (!ids.empty()
        && SetThreadSelectedCpuSets(GetCurrentThread(),
                                    reinterpret_cast<const ULONG*>(ids.data()),
                                    static_cast<ULONG>(ids.size())))
        result.heldToCpuSets = ids.size();
    return result;
}
#else
void requestLatencyCriticalProcess() noexcept {}

LatencyCriticalThreadResult requestLatencyCriticalThread() noexcept { return {}; }
#endif

unsigned usableProcessorCount() noexcept {
#if defined(_WIN32)
    // A mask equal to the system's is no restriction; reading it then would
    // count one processor group only on a machine with more than 64.
    DWORD_PTR processMask = 0;
    DWORD_PTR systemMask = 0;
    if (GetProcessAffinityMask(GetCurrentProcess(), &processMask, &systemMask)
        && processMask != 0 && processMask != systemMask)
        return static_cast<unsigned>(std::popcount(static_cast<std::uint64_t>(processMask)));
#elif defined(__linux__)
    cpu_set_t set;
    CPU_ZERO(&set);
    if (sched_getaffinity(0, sizeof(set), &set) == 0 && CPU_COUNT(&set) > 0)
        return static_cast<unsigned>(CPU_COUNT(&set));
#endif
    return std::thread::hardware_concurrency();
}

} // namespace enginelab::platform
