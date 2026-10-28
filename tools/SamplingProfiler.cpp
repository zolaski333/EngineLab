// Sampling profiler for Windows, without administrator rights.
//
//   EngineLabSamplingProfiler [--warmup s] [--seconds s] [--interval-ms ms]
//                             [--top n] -- <program> [arguments...]
//
// Starts the program, waits `--warmup`, then every `--interval-ms` suspends
// each of its threads, walks its stack and resumes it. A sample counts only
// for a thread that used CPU since the previous one, weighted by that CPU
// time, so a parked thread costs nothing and a spinning one costs what it
// burns. Prints the self and the inclusive share of CPU per function, and per
// thread. The program must carry a PDB: build it in a tree whose Release flags
// add /Zi and whose linker adds /DEBUG (docs/tests-and-validation.md).

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <dbghelp.h>
#include <tlhelp32.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace {

struct ThreadSampler final {
    HANDLE handle { nullptr };
    std::uint64_t lastCpu100ns { 0 };
    double cpuSeconds { 0.0 };
};

std::uint64_t threadCpu100ns(HANDLE thread) {
    FILETIME created {}, exited {}, kernel {}, user {};
    if (!GetThreadTimes(thread, &created, &exited, &kernel, &user)) return 0;
    const auto join = [](const FILETIME& time) {
        return (static_cast<std::uint64_t>(time.dwHighDateTime) << 32U) | time.dwLowDateTime;
    };
    return join(kernel) + join(user);
}

std::string symbolName(HANDLE process, DWORD64 address) {
    alignas(SYMBOL_INFO) char buffer[sizeof(SYMBOL_INFO) + 512] {};
    auto* symbol = reinterpret_cast<SYMBOL_INFO*>(buffer);
    symbol->SizeOfStruct = sizeof(SYMBOL_INFO);
    symbol->MaxNameLen = 511;
    DWORD64 displacement = 0;
    if (SymFromAddr(process, address, &displacement, symbol)) return std::string(symbol->Name);
    IMAGEHLP_MODULE64 module {};
    module.SizeOfStruct = sizeof(module);
    if (SymGetModuleInfo64(process, address, &module)) return std::string(module.ModuleName) + "!?";
    return "?";
}

std::string quoteArgument(const std::string& argument) {
    if (!argument.empty() && argument.find_first_of(" \t\"") == std::string::npos) return argument;
    std::string quoted = "\"";
    for (const auto character : argument) {
        if (character == '"') quoted += '\\';
        quoted += character;
    }
    return quoted + "\"";
}

}  // namespace

int main(int argc, char** argv) {
    double warmupSeconds = 3.0;
    double sampleSeconds = 10.0;
    int intervalMs = 2;
    std::size_t top = 40;
    int programIndex = -1;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--") { programIndex = index + 1; break; }
        if (index + 1 >= argc) break;
        if (argument == "--warmup") warmupSeconds = std::stod(argv[++index]);
        else if (argument == "--seconds") sampleSeconds = std::stod(argv[++index]);
        else if (argument == "--interval-ms") intervalMs = std::max(1, std::stoi(argv[++index]));
        else if (argument == "--top") top = static_cast<std::size_t>(std::stoul(argv[++index]));
    }
    if (programIndex < 0 || programIndex >= argc) {
        std::cerr << "usage: EngineLabSamplingProfiler [--warmup s] [--seconds s] [--interval-ms ms]"
                     " [--top n] -- <program> [arguments...]\n";
        return EXIT_FAILURE;
    }
    std::string commandLine;
    for (int index = programIndex; index < argc; ++index)
        commandLine += (commandLine.empty() ? "" : " ") + quoteArgument(argv[index]);

    STARTUPINFOA startup {};
    startup.cb = sizeof(startup);
    PROCESS_INFORMATION child {};
    std::vector<char> mutableCommand(commandLine.begin(), commandLine.end());
    mutableCommand.push_back('\0');
    if (!CreateProcessA(nullptr, mutableCommand.data(), nullptr, nullptr, TRUE, 0, nullptr, nullptr,
            &startup, &child)) {
        std::cerr << "cannot start: " << commandLine << " (error " << GetLastError() << ")\n";
        return EXIT_FAILURE;
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(warmupSeconds));
    if (WaitForSingleObject(child.hProcess, 0) == WAIT_OBJECT_0) {
        std::cerr << "the program ended during the warm-up\n";
        return EXIT_FAILURE;
    }
    SymSetOptions(SYMOPT_UNDNAME | SYMOPT_DEFERRED_LOADS | SYMOPT_LOAD_LINES);
    if (!SymInitialize(child.hProcess, nullptr, TRUE)) {
        std::cerr << "SymInitialize failed (error " << GetLastError() << ")\n";
        return EXIT_FAILURE;
    }

    std::map<DWORD, ThreadSampler> threads;
    std::unordered_map<DWORD64, double> selfSeconds;
    std::unordered_map<DWORD64, double> inclusiveSeconds;
    std::map<DWORD, std::map<DWORD64, double>> threadTopFrames;
    std::map<std::pair<DWORD64, DWORD64>, double> selfByCaller;
    double totalSeconds = 0.0;
    timeBeginPeriod(1);
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(sampleSeconds);
    std::size_t round = 0;
    while (std::chrono::steady_clock::now() < end
           && WaitForSingleObject(child.hProcess, 0) != WAIT_OBJECT_0) {
        if (round++ % 50 == 0) {
            const auto snapshot = CreateToolhelp32Snapshot(TH32CS_SNAPTHREAD, 0);
            THREADENTRY32 entry {};
            entry.dwSize = sizeof(entry);
            for (auto more = Thread32First(snapshot, &entry); more; more = Thread32Next(snapshot, &entry)) {
                if (entry.th32OwnerProcessID != child.dwProcessId || threads.contains(entry.th32ThreadID)) continue;
                const auto handle = OpenThread(THREAD_SUSPEND_RESUME | THREAD_GET_CONTEXT | THREAD_QUERY_INFORMATION,
                    FALSE, entry.th32ThreadID);
                if (handle) threads[entry.th32ThreadID] = { handle, threadCpu100ns(handle), 0.0 };
            }
            CloseHandle(snapshot);
        }
        for (auto& [id, thread] : threads) {
            const auto cpu = threadCpu100ns(thread.handle);
            const auto used = static_cast<double>(cpu - std::min(cpu, thread.lastCpu100ns)) * 1.0e-7;
            thread.lastCpu100ns = cpu;
            if (used <= 0.0) continue;
            if (SuspendThread(thread.handle) == static_cast<DWORD>(-1)) continue;
            CONTEXT context {};
            context.ContextFlags = CONTEXT_FULL;
            std::vector<DWORD64> frames;
            if (GetThreadContext(thread.handle, &context)) {
                STACKFRAME64 frame {};
                frame.AddrPC.Offset = context.Rip;
                frame.AddrPC.Mode = AddrModeFlat;
                frame.AddrFrame.Offset = context.Rbp;
                frame.AddrFrame.Mode = AddrModeFlat;
                frame.AddrStack.Offset = context.Rsp;
                frame.AddrStack.Mode = AddrModeFlat;
                while (frames.size() < 96
                       && StackWalk64(IMAGE_FILE_MACHINE_AMD64, child.hProcess, thread.handle, &frame, &context,
                              nullptr, SymFunctionTableAccess64, SymGetModuleBase64, nullptr)
                       && frame.AddrPC.Offset != 0)
                    frames.push_back(frame.AddrPC.Offset);
            }
            ResumeThread(thread.handle);
            if (frames.empty()) continue;
            thread.cpuSeconds += used;
            totalSeconds += used;
            // Functions, not addresses: an address maps to its symbol's start.
            std::vector<DWORD64> functions;
            for (const auto address : frames) {
                DWORD64 displacement = 0;
                alignas(SYMBOL_INFO) char buffer[sizeof(SYMBOL_INFO) + 8] {};
                auto* symbol = reinterpret_cast<SYMBOL_INFO*>(buffer);
                symbol->SizeOfStruct = sizeof(SYMBOL_INFO);
                symbol->MaxNameLen = 1;
                functions.push_back(SymFromAddr(child.hProcess, address, &displacement, symbol)
                    ? symbol->Address : address);
            }
            selfSeconds[functions.front()] += used;
            threadTopFrames[id][functions.front()] += used;
            const auto caller = std::find_if(functions.begin(), functions.end(),
                [&](DWORD64 function) { return function != functions.front(); });
            selfByCaller[{ functions.front(), caller == functions.end() ? 0 : *caller }] += used;
            std::set<DWORD64> seen;
            for (const auto function : functions)
                if (seen.insert(function).second) inclusiveSeconds[function] += used;
        }
        Sleep(static_cast<DWORD>(intervalMs));
    }
    timeEndPeriod(1);

    const auto print = [&](const char* title, const std::unordered_map<DWORD64, double>& table) {
        std::vector<std::pair<double, DWORD64>> rows;
        for (const auto& [address, seconds] : table) rows.emplace_back(seconds, address);
        std::sort(rows.rbegin(), rows.rend());
        std::printf("\n%s (%% of %.2f CPU-s sampled)\n", title, totalSeconds);
        for (std::size_t index = 0; index < std::min(top, rows.size()); ++index)
            std::printf("%6.2f %%  %s\n", 100.0 * rows[index].first / std::max(1.0e-12, totalSeconds),
                symbolName(child.hProcess, rows[index].second).c_str());
    };
    print("self", selfSeconds);
    print("inclusive", inclusiveSeconds);
    std::printf("\nthreads\n");
    for (const auto& [id, thread] : threads) {
        if (thread.cpuSeconds <= 0.0) continue;
        DWORD64 hottest = 0;
        double hottestSeconds = 0.0;
        for (const auto& [address, seconds] : threadTopFrames[id])
            if (seconds > hottestSeconds) { hottest = address; hottestSeconds = seconds; }
        std::printf("%6lu %6.2f %% CPU-s %.2f, hottest %s\n", id, 100.0 * thread.cpuSeconds / std::max(1.0e-12, totalSeconds),
            thread.cpuSeconds, symbolName(child.hProcess, hottest).c_str());
    }
    std::printf("\nself by caller\n");
    {
        std::vector<std::pair<double, std::pair<DWORD64, DWORD64>>> rows;
        for (const auto& [pair, seconds] : selfByCaller) rows.emplace_back(seconds, pair);
        std::sort(rows.rbegin(), rows.rend());
        for (std::size_t index = 0; index < std::min(top, rows.size()); ++index)
            std::printf("%6.2f %%  %s  <-  %s\n", 100.0 * rows[index].first / std::max(1.0e-12, totalSeconds),
                symbolName(child.hProcess, rows[index].second.first).c_str(),
                symbolName(child.hProcess, rows[index].second.second).c_str());
    }
    for (const auto& [id, frames] : threadTopFrames) {
        std::vector<std::pair<double, DWORD64>> rows;
        for (const auto& [address, seconds] : frames) rows.emplace_back(seconds, address);
        std::sort(rows.rbegin(), rows.rend());
        std::printf("\nthread %lu self\n", id);
        for (std::size_t index = 0; index < std::min<std::size_t>(15, rows.size()); ++index)
            std::printf("%6.2f %%  %s\n", 100.0 * rows[index].first / std::max(1.0e-12, threads[id].cpuSeconds),
                symbolName(child.hProcess, rows[index].second).c_str());
    }
    SymCleanup(child.hProcess);
    TerminateProcess(child.hProcess, 0);
    for (auto& [id, thread] : threads) CloseHandle(thread.handle);
    CloseHandle(child.hThread);
    CloseHandle(child.hProcess);
    return EXIT_SUCCESS;
}
