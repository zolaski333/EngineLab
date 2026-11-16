# Linux build and bounded checks

## Outcome

The current tree does not complete the repository's standard GCC Release build. A fresh Ubuntu 24.04 / GCC 13.3 build with the `linux-gcc-release` preset stops in the exhaust library on PLAT-01 below. No production source or build configuration was changed and no warning was suppressed.

Four independent targets were then freshly built and passed their exact CTest entries: `EngineLab.Calibration`, `EngineLab.Scripting`, `EngineLab.ThreadScheduling` and `EngineLab.PhysicsRegression` (4/4). This is a bounded Linux smoke/component result, not a passing full Linux build or suite.

The original `enginelab-linux` container remains stopped, with exactly its previous StartedAt and FinishedAt timestamps. The audit-created container and filesystem snapshot image were removed after checks. All build/log artifacts remain in `out/audit2/linux`.

## Environment and isolation

The existing container `enginelab-linux` (`02b87e744edcc2b338f7d1474bbc0b73118644418cd5e0999a0503e32e316019`) was inspected while stopped. Its image is `ubuntu:24.04`, command is `["sh", "/work/run.sh"]`, and its sole bind mount points `/work` at an old Windows temporary directory. That directory still exists, but `/work/run.sh` no longer does. There was no retained CMake build cache.

Rather than change that command or the original bind contents, the audit took a temporary filesystem snapshot (`bd206b45d5b6df2240c28d1bbaf82d7d8a72c652b43790639b5c7c28405cb422`) and created `enginelab-audit2-check` (`706dc8710c0da61aaad4cf987ebd528c51b71bf5e809369c75139ad76dafd6f5`). `/src` bound the current repository read-only; only `/audit`, bound to this audit-output directory, was writable. No host software was installed.

Observed toolchain: Ubuntu 24.04.5, GCC/G++ 13.3.0, CMake 3.28.3, Ninja 1.11.1. Required queried ALSA, freetype, fontconfig, X11 and GL development packages were available. Clang, Xvfb, matchbox-window-manager and xprop were absent. curl/WebKit configure probes were absent, as expected for the disabled JUCE web/curl features; they did not prevent configuration. Configure emitted yaml-cpp's CMake compatibility deprecation warning, not a production-source compile failure.

The configure step built JUCE's cold `juceaide` bootstrap successfully using JUCE's default helper invocation. Subsequent EngineLab target builds were explicitly limited to `-j 2` with `CMAKE_BUILD_PARALLEL_LEVEL=2`. Compiler cache was OFF. The actual compile line also contains the repository's `-ffp-contract=off`; this is not a cross-platform bit-identity claim.

The native main-thread stack limit was 8192 KiB, recorded before tests. No EngineRuntime stack-exhaustion crash occurred in the four selected component checks. GUI execution, live audio-device behavior, real-time scheduling success and performance were not measured.

## Commands and results

The real preset was used with only the build-directory and already-present dependency-source locations redirected to audit storage:

```sh
cmake --preset linux-gcc-release -B /audit/build-gcc   -DENGINELAB_ENABLE_COMPILER_CACHE=OFF   -DFETCHCONTENT_SOURCE_DIR_JUCE=/src/out/build/windows-vs2022/_deps/juce-src   -DFETCHCONTENT_SOURCE_DIR_NLOHMANN_JSON=/src/out/build/windows-vs2022/_deps/nlohmann_json-src   -DFETCHCONTENT_SOURCE_DIR_YAML_CPP=/src/out/build/windows-vs2022/_deps/yaml_cpp-src
```

Configuration passed. `configure-gcc.log`, `build-gcc/CMakeCache.txt`, `build-gcc/build.ninja` and `build-gcc/compile_commands.json` retain the generated evidence.

```sh
CMAKE_BUILD_PARALLEL_LEVEL=2 cmake --build /audit/build-gcc -j 2 --target   EngineLabApp EngineLabPhysicsRegressionTests EngineLabGasDynamicsTests   EngineLabScriptingTests EngineLabCalibrationTests EngineLabDriverCommandTests   EngineLabThreadSchedulingTests EngineLabSavedEngineTests
```

This fresh build stopped at step 94/208, followed by one already-running compilation completing at 95/208, on PLAT-01. The App, GasDynamics, DriverCommand and SavedEngine outcomes are therefore blocked, not passed. Source units already completed include JUCE's Linux audio/device/GUI backend, foundation's POSIX thread scheduling, current calibration, physics and ECU.

```sh
CMAKE_BUILD_PARALLEL_LEVEL=2 cmake --build /audit/build-gcc -j 2 --target   EngineLabPhysicsRegressionTests EngineLabCalibrationTests   EngineLabThreadSchedulingTests EngineLabScriptingTests
ctest --test-dir /audit/build-gcc   -R '^EngineLab[.](PhysicsRegression|Scripting|Calibration|ThreadScheduling)$'   --output-on-failure -j 1 --timeout 180
```

The independent build completed all 15 requested remaining steps. CTest reported 4/4 passed: Calibration 0.01 s, Scripting 0.10 s, ThreadScheduling 0.00 s and PhysicsRegression 0.01 s, total 0.24 s. Those elapsed times are acquisition details only and are not benchmark figures. See `build-independent.log` and `ctest-independent.log`.

No full Linux suite, GUI test, Clang build, package build or real-time benchmark was run. No Linux success is inferred for unbuilt or unexecuted targets.

## Cleanup and provenance

`container-before.json` and `container-after.json` show the original container remained `exited`, `Running=false`, with StartedAt `2026-11-05T22:29:17.787035793Z` and FinishedAt `2026-11-05T22:39:33.015284933Z`. Audit container `706dc871...` and image `bd206b45...` were removed. Environment, selected source checksums, current source/header hashes, binary hashes and log hashes are recorded in `environment.log` and `verification.json`.

## Captured compiler failure

```text
 src/exhaust/CMakeFiles/EngineLabExhaust.dir/src/ExhaustPathTopologyEditor.cpp.o 
/usr/bin/c++  -I/src/src/exhaust/include -I/src/src/foundation/include -I/src/src/events/include -O3 -DNDEBUG -std=c++20 -ffp-contract=off -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror -MD -MT src/exhaust/CMakeFiles/EngineLabExhaust.dir/src/ExhaustPathTopologyEditor.cpp.o -MF src/exhaust/CMakeFiles/EngineLabExhaust.dir/src/ExhaustPathTopologyEditor.cpp.o.d -o src/exhaust/CMakeFiles/EngineLabExhaust.dir/src/ExhaustPathTopologyEditor.cpp.o -c /src/src/exhaust/src/ExhaustPathTopologyEditor.cpp
/src/src/exhaust/src/ExhaustPathTopologyEditor.cpp: In function 'enginelab::ExhaustPathEditResult enginelab::{anonymous}::reject(std::string)':
/src/src/exhaust/src/ExhaustPathTopologyEditor.cpp:400:34: error: missing initializer for member 'enginelab::ExhaustPathEditResult::note' [-Werror=missing-field-initializers]
  400 |     return { 0, std::move(error) };
      |                                  ^
cc1plus: all warnings being treated as errors
[95/208] Building CXX object src/gas-dynamics/CMakeFiles/EngineLabGasDynamics.dir/src/FiniteVolumeDuct.cpp.o
ninja: build stopped: subcommand failed.

```

## Captured independent test result

```text
Internal ctest changing into directory: /audit/build-gcc
Test project /audit/build-gcc
    Start  1: EngineLab.Calibration
1/4 Test  #1: EngineLab.Calibration ............   Passed    0.01 sec
    Start  7: EngineLab.Scripting
2/4 Test  #7: EngineLab.Scripting ..............   Passed    0.10 sec
    Start 19: EngineLab.ThreadScheduling
3/4 Test #19: EngineLab.ThreadScheduling .......   Passed    0.00 sec
    Start 22: EngineLab.PhysicsRegression
4/4 Test #22: EngineLab.PhysicsRegression ......   Passed    0.01 sec

100% tests passed, 0 tests failed out of 4

Total Test time (real) =   0.24 sec

```
