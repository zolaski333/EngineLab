# Platforms

Windows x64 is the reference platform. Every performance figure, every
`ctest` baseline and every release archive comes from the Visual Studio tree
(`out/build/windows-vs2022`). Linux is built and tested, but a number measured
there is not comparable with a Windows one. This page says what differs and
why, so a difference is read as a platform difference rather than a regression.

## Toolchains

| Platform | Compiler | Generator | Preset |
| --- | --- | --- | --- |
| Windows x64 (reference) | MSVC 14.44, Visual Studio 2022 BuildTools | Visual Studio 17 | `windows-vs2022`, `windows-ninja-release` |
| Linux x64 | GCC 13.3 (Ubuntu 24.04) | Ninja | `linux-gcc-release`, `linux-gcc-debug` |
| Linux x64 (second opinion) | Clang 18 with libstdc++ | Ninja | `linux-clang-release` |

Every preset carries a `condition` on `${hostSystemName}`, so
`cmake --list-presets` on either host shows only the presets that can run
there. `scripts/vsenv.ps1` is Windows-only and is not needed on Linux.

The Linux build dependencies are listed in
[CONTRIBUTING.md](../CONTRIBUTING.md#linux). `libasound2-dev` is required even
for a headless build: `juce_audio_devices` compiles its ALSA backend
unconditionally on Linux.

## What differs, and what was done about it

### Floating point

GCC and Clang default to `-ffp-contract=fast`, which fuses `a * b + c` into a
single FMA instruction; MSVC under `/fp:precise` does not contract. Two
consequences:

- the last bits of a physics result differ between the platforms, so the
  bit-exact checksum of `EngineLabIntakeDuctBench` is comparable **within** a
  platform ("unchanged = physics untouched") and not across them;
- `sizeof(long double)` is 16 on Linux x86-64 and 8 on MSVC, so a `long double`
  intermediate carries 64 bits of mantissa on one and 53 on the other.

glibc's `libm` also differs from MSVC's in the last bits of `sin`, `cos`, `exp`
and `pow`, so cross-platform bit-exactness is not attainable at any flag
setting. The flags only decide how close the two get.

### The physics thread's priority

The physics thread is the one that must not miss its deadline: the real-time
factor the diagnostics show, and with it every audible symptom of a late step,
is measured on it. Windows raises it with `SetThreadPriority`. POSIX grants a
real-time policy (`SCHED_RR`) only to a privileged process, so
`EngineRuntime::run` attempts it and ignores the failure. An unprivileged Linux
run therefore keeps `SCHED_OTHER` and can report a lower real-time factor than
Windows for the same engine. That is a scheduling difference, not a slower
solver.

### Hybrid processors (Intel 12th generation and later)

Priority does not decide which core a thread runs on. On a processor with
performance and efficiency cores, Windows 11 places threads by quality of
service, which it infers from window focus and visibility, and Intel's Thread
Director steers a thread that spins on `pause` -- as the resident workers do
between dispatches -- towards an efficiency core. Every dispatch of
`CylinderWorkerPool` waits for its slowest worker, so one worker on an
efficiency core sets the pace of the whole step. Under EcoQoS (a process judged
in the background) the clock drops as well.

`ThreadScheduling.hpp` therefore opts the process out of power throttling,
keeps the timer resolution it asked for while the window is hidden, and holds
the physics thread and the pool workers to the fastest class of CPU sets when
the machine has more than one class. On a uniform processor (the Ryzen 7
8840U reference) nothing is restricted. Nothing of this changes what is computed.
`EngineLabRealtimeBudgetHarness --power-throttled` measures the throttled case
on any machine; no hybrid machine has measured the fix itself.

### Stack size

An `EngineRuntime` embeds a `SpscQueue<CylinderPressureSample, 8192>` and
weighs about 7.4 MB. On Windows eleven test targets link with
`/STACK:8388608`; Linux has no link-time equivalent (the kernel ignores
`PT_GNU_STACK`'s size, and the main thread's stack comes from `ulimit -s`,
8 MB by default). Every `EngineRuntime` in the test suite is therefore
allocated with `std::make_unique`, which is the rule the rest of the project
already follows and which needs no per-platform link option.

### File-name encodings

`std::filesystem::path` reads a narrow string through the active code page on
Windows, which drops accents, and a wide one through the locale on Linux, which
can throw under `LC_ALL=C`. `pathOf` and `fileOf` in `MainComponent.cpp`
therefore use the wide form on Windows and UTF-8 on Linux: each platform gets
the encoding its own `path` is native in.

### Warnings

`cmake/CompilerWarnings.cmake` applies `-Wall -Wextra -Wpedantic -Wconversion
-Wshadow` (GCC and Clang) or `/W4` (MSVC), and `-Werror` / `/WX` when
`ENGINELAB_ENABLE_WARNINGS_AS_ERRORS` is on, as CI has it.

JUCE's module units are compiled **into** the targets that link them, because
they arrive as interface sources, so they would inherit that set. They are not
EngineLab's code and they do not build clean under `-Wconversion`: on the first
Linux build they produced about 400 diagnostics around EngineLab's own 125.
`enginelab_quiet_vendor_units` turns the categories they trip off for those
units only, leaving EngineLab's own sources under the full set.

A header is a different problem: it is compiled as part of whatever includes
it, so no per-source property can reach it. One `-Wconversion` line of JUCE's
`juce_UMPUtils.h` broke fourteen of our own translation units the first time
warnings became errors, and yaml-cpp's `impl.h`, `emitterstyle.h` and `type.h`
broke more the first time Clang ran. `enginelab_vendor_headers_are_system`
therefore re-publishes all three dependencies' include directories as SYSTEM
ones, which is what tells GCC and Clang to keep quiet about code we do not own.
`FetchContent_Declare(... SYSTEM)` says the same in one word but needs
CMake 3.25, and this project supports 3.24.

GCC's `-Warray-bounds` reads `std::sort` on a small `std::array` as a
possible overrun: it keeps the long-range branch of the sort, whose
insertion-sort threshold is 16 elements, and reports element 16 of a
4-element array. `ExhaustGraph::process` now clamps the sort's length against
`std::array::size()`, which restates an invariant the optimiser could not
carry through the surrounding loops and costs nothing at run time. MSVC does
not look that far and never reported it.

`std::hardware_destructive_interference_size` draws GCC's
`-Winterference-size`: the value is not stable across compilers, so a class
whose layout depends on it is not ABI-portable. EngineLab is one binary per
build and nothing crosses an ABI boundary, while the per-architecture value is
exactly what `CylinderWorkerPool`'s padding wants, so the warning is suppressed
at the declaration rather than the value replaced. Clang with libstdc++ does
not define `__cpp_lib_hardware_interference_size` at all; the existing
`#else` branch gives it 64.

**What Clang adds over GCC.** The `linux-clang-release` preset exists because
the two reject different things, so a green GCC build is not proof on its own.
Measured on the first Clang run (2026-10-21):

It found 26 diagnostics in our own code, plus 12 in yaml-cpp's headers:

- **22 were `-Wsign-conversion`**, which Clang's `-Wconversion` pulls in and
  GCC's does not in C++. Every one was an `int` loop counter or rpm bin index
  subscripting a `std::vector` or `std::array`. They are now `std::size_t`,
  converted once after the range check that rejects a negative rather than at
  every subscript, because the index is cast from a `double` and that guard is
  the only thing making it safe.
- **1 was not a warning at all** but a conformance error Clang is simply right
  about: a lambda with no capture-default may not reach a local
  `constexpr std::string_view`, because comparing against it binds a reference
  and so odr-uses it. GCC and MSVC both accept it.
- **3 were dead declarations** neither other compiler mentioned: an unused
  file-local function, an accumulator nothing read, and a `SidePanel` member
  the constructor stored and no method of that class ever used.

Clang also does not define `__cpp_lib_hardware_interference_size` with
libstdc++ (above), and its LTO flag handling differs from GCC's.

### Packaging

`--target package` produces `EngineLab-<version>-win64.zip` on Windows and
`EngineLab-<version>-Linux.zip` on Linux. Only the Windows archive is attached
to a GitHub release. The Windows archive ships the MSVC runtime beside the
executable; the Linux one cannot, because the application links the
distribution's own ALSA, X11, GL and freetype.

### The GUI under `ctest`

`EngineLab.AudioWorkshop` opens a real window, and on Linux a virtual display
alone is not enough for it. JUCE 8 looks the `_NET_WM_*` atoms up with
`XInternAtom(..., only_if_exists = True)` (`Atoms::getIfExists` in
`juce_XWindowSystem_linux.cpp`). On a bare Xvfb no window manager has ever
interned them, so the lookups return `None` and JUCE then issues an
`XChangeProperty` with atom 0: the test dies with `BadAtom` in 0.4 s, before
its first assertion. Measured on Ubuntu 24.04 with JUCE 8.0.13, and it passes
unchanged once a window manager is running.

`scripts/run-gui-test.sh` is therefore what `ctest` invokes: it starts Xvfb and
`matchbox-window-manager`, waits for `_NET_SUPPORTED` to appear on the root
window, then runs the test. When Xvfb or the window manager is missing it says
which. Wrapping the test rather than wrapping `ctest` in CI is what keeps a
contributor's bare `ctest` working. See
[CONTRIBUTING.md](../CONTRIBUTING.md#linux).

**It asks the display whether a window manager is there; it does not trust
`DISPLAY`.** The first version reused an already-set `DISPLAY` on the
assumption that a display someone had set up would be a managed desktop. The
hosted Linux runner exports one with no window manager behind it, so CI failed
on `EngineLab.AudioWorkshop` in 0.49 s -- the same `BadAtom`, from the same
cause -- while every local run passed, because WSLg does run a window manager
and hid it. The probe is `_NET_SUPPORTED` on the root window: absent on a bare
Xvfb, present as soon as a manager claims the screen.

The probe has to read xprop's **output**, not its exit status: `xprop` returns
0 for a property that does not exist and prints `no such atom on any window`.
Testing the status made the check answer yes on a bare server, and it is also
why the wait for `matchbox` never actually waited. Verified on Ubuntu 24.04,
four conditions, with a bare `Xvfb :77` standing in for the runner: `DISPLAY`
set to that bare server passes, no `DISPLAY` at all passes, a real managed
desktop passes and is used as-is (no nested server), and the test binary run
straight at the bare server still fails -- which is what makes the first case
a fix rather than a tautology.

## Not covered yet

- **macOS**: nothing is built or tested. The code is standard C++20 and JUCE,
  and the POSIX branches added for Linux compile there in principle, but no
  one has run it.
- **A loopback recorder on Linux**: `tools/WasapiLoopbackRecorder.cpp` is
  Windows-only and is not part of the build. The reference recordings for
  milestone 1 are made on Windows.
- **aarch64**: `CylinderWorkerPool::spinPause` has an aarch64 branch and the
  cache-line constant follows the architecture, but no aarch64 build has been
  run.
