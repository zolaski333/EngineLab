cmake_minimum_required(VERSION 3.24)
# C is listed even though EngineLab itself is C++ only. JUCE declares
# `project(JUCE LANGUAGES C CXX)`, but that runs inside the FetchContent
# subdirectory scope, so the C rule variables (CMAKE_C_COMPILE_OBJECT and
# friends) never reach the top level. The Visual Studio generator tolerates
# that -- MSBuild picks the language from the file extension -- which is why the
# windows-vs2022 cache has CMAKE_C_FLAGS but no CMAKE_C_COMPILER at all. Ninja
# resolves the compile rule at generate time and fails outright.
project(EngineLab VERSION 0.1.0 LANGUAGES CXX C)

option(ENGINELAB_BUILD_TESTS "Build EngineLab unit tests" ON)
option(ENGINELAB_BUILD_COMPARISON_HARNESS "Build deterministic comparison and regression harness" ON)
option(ENGINELAB_ENABLE_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
option(ENGINELAB_ENABLE_SANITIZERS "Enable Address/UndefinedBehavior sanitizers" OFF)
option(ENGINELAB_ENABLE_CLANG_TIDY "Run clang-tidy while compiling" OFF)
option(ENGINELAB_ENABLE_COMPILER_CACHE "Route compilation through sccache/ccache when one is available" OFF)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# DO NOT SWITCH THIS ON. It is kept only so the finding is reproducible.
#
# A compiler cache looked like the way to make a bisect cheap, and it is not
# usable here. Two independent reasons:
#
# - The Visual Studio generator silently IGNORES
#   CMAKE_CXX_COMPILER_LAUNCHER, so the authoritative tree cannot use it at all.
# - With the Ninja generator it is worse than useless: it breaks header
#   dependency tracking. Ninja learns dependencies by parsing cl.exe's
#   /showIncludes output, and sccache does not pass it through, so every object
#   records `#deps 0` and the tree serves STALE objects after any header edit.
#   Measured on this project: with the launcher, `ninja -t deps` on
#   EngineTypes.cpp.obj reports `#deps 0`; without it, `#deps 1` naming
#   EngineTypes.hpp. SCCACHE_DIRECT=false does not help, and a forced recache
#   (SCCACHE_RECACHE=1) does not either -- it is not a cache-hit artefact.
#
# The cache was measured at 94.5 % hits and 56.5 s for a full rebuild that costs
# ~474 s cold, so the prize was real; it is simply not worth a build tree that
# can silently compile against a header it no longer matches.
if(ENGINELAB_ENABLE_COMPILER_CACHE)
    file(GLOB ENGINELAB_COMPILER_CACHE_HINTS
        "$ENV{LOCALAPPDATA}/Microsoft/WinGet/Packages/Mozilla.sccache*/*")
    find_program(ENGINELAB_COMPILER_CACHE NAMES sccache ccache
        HINTS ${ENGINELAB_COMPILER_CACHE_HINTS})
    if(ENGINELAB_COMPILER_CACHE)
        set(CMAKE_CXX_COMPILER_LAUNCHER "${ENGINELAB_COMPILER_CACHE}")
        if(MSVC)
            # sccache cannot cache a compile that writes into a shared PDB, and
            # silently falls back to an uncached invocation for every such
            # object -- a 0% hit rate that looks like the cache simply not
            # working. /Z7 puts the debug info in the object instead, where the
            # cache can carry it. CMake 3.24 predates
            # CMAKE_MSVC_DEBUG_INFORMATION_FORMAT (CMP0141 landed in 3.25), so
            # the per-configuration flags are rewritten directly. Release
            # carries no debug flag at all here and comes out unchanged.
            foreach(enginelab_config DEBUG RELWITHDEBINFO MINSIZEREL RELEASE)
                string(REPLACE "/ZI" "/Z7"
                    CMAKE_CXX_FLAGS_${enginelab_config} "${CMAKE_CXX_FLAGS_${enginelab_config}}")
                string(REPLACE "/Zi" "/Z7"
                    CMAKE_CXX_FLAGS_${enginelab_config} "${CMAKE_CXX_FLAGS_${enginelab_config}}")
            endforeach()
        endif()
        message(STATUS "EngineLab: compiler cache enabled (${ENGINELAB_COMPILER_CACHE})")
    else()
        message(WARNING
            "ENGINELAB_ENABLE_COMPILER_CACHE is ON but neither sccache nor ccache was found")
    endif()
endif()

# Sanitizer instrumentation must be consistent across EngineLab and fetched
# static dependencies (notably yaml-cpp), otherwise MSVC's annotated STL types
# produce LNK2038 mismatches at the final executable.
if(ENGINELAB_ENABLE_SANITIZERS)
    if(MSVC)
        add_compile_options(/fsanitize=address)
        add_link_options(/INCREMENTAL:NO)
    else()
        add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
        add_link_options(-fsanitize=address,undefined)
    endif()
endif()

if(ENGINELAB_ENABLE_CLANG_TIDY)
    find_program(ENGINELAB_CLANG_TIDY_EXECUTABLE NAMES clang-tidy REQUIRED)
    set(CMAKE_CXX_CLANG_TIDY "${ENGINELAB_CLANG_TIDY_EXECUTABLE}")
endif()

include(FetchContent)
include(cmake/CompilerWarnings.cmake)

# yaml-cpp 0.8.0 still declares compatibility with CMake 3.4. CMake 4.x
# requires consumers to opt into the 3.5 policy floor explicitly.
set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING "Minimum policy version for legacy dependencies")

# Pinned dependencies keep builds reproducible.
FetchContent_Declare(JUCE GIT_REPOSITORY https://github.com/juce-framework/JUCE.git GIT_TAG 8.0.13 GIT_SHALLOW TRUE)
FetchContent_Declare(nlohmann_json GIT_REPOSITORY https://github.com/nlohmann/json.git GIT_TAG v3.12.0 GIT_SHALLOW TRUE)
FetchContent_Declare(yaml_cpp GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git GIT_TAG 0.8.0 GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(JUCE nlohmann_json yaml_cpp)

add_subdirectory(src/foundation)
add_subdirectory(src/calibration)
add_subdirectory(src/events)
add_subdirectory(src/physics)
add_subdirectory(src/ecu)
add_subdirectory(src/exhaust)
add_subdirectory(src/gas-dynamics)
add_subdirectory(src/simulation)
add_subdirectory(src/diagnostics)
add_subdirectory(src/render)
add_subdirectory(src/serialization)
add_subdirectory(src/scripting)
add_subdirectory(src/catalog)
add_subdirectory(src/runtime)
add_subdirectory(src/audio)
add_subdirectory(src/app)
if(ENGINELAB_BUILD_COMPARISON_HARNESS)
    add_subdirectory(tools)
endif()

if(ENGINELAB_BUILD_TESTS)
    enable_testing()
    add_subdirectory(src/calibration/tests)
    add_subdirectory(src/exhaust/tests)
    add_subdirectory(src/gas-dynamics/tests)
    add_subdirectory(src/render/tests)
    add_subdirectory(src/scripting/tests)
    add_subdirectory(tests)
endif()

install(TARGETS EngineLabApp
    RUNTIME DESTINATION . COMPONENT EngineLab
    BUNDLE DESTINATION . COMPONENT EngineLab)
if(TARGET EngineLabOfflineAudioExporter AND TARGET EngineLabAbClipRenderer)
    install(TARGETS EngineLabOfflineAudioExporter EngineLabAbClipRenderer
        RUNTIME DESTINATION tools COMPONENT EngineLab)
endif()
install(DIRECTORY engines parts assets examples voicing DESTINATION . COMPONENT EngineLab)
install(DIRECTORY docs DESTINATION . COMPONENT EngineLab)
install(FILES README.md LICENSE.md DESTINATION . COMPONENT EngineLab)
set(CPACK_PACKAGE_NAME "EngineLab")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_GENERATOR "ZIP")
set(CPACK_INSTALL_CMAKE_PROJECTS "${CMAKE_BINARY_DIR};${PROJECT_NAME};EngineLab;/")
include(CPack)
