cmake_minimum_required(VERSION 3.24)
project(EngineLab VERSION 0.1.0 LANGUAGES CXX)

option(ENGINELAB_BUILD_TESTS "Build EngineLab unit tests" ON)
option(ENGINELAB_ENABLE_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
option(ENGINELAB_ENABLE_SANITIZERS "Enable Address/UndefinedBehavior sanitizers" OFF)
option(ENGINELAB_ENABLE_CLANG_TIDY "Run clang-tidy while compiling" OFF)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
add_subdirectory(src/events)
add_subdirectory(src/physics)
add_subdirectory(src/ecu)
add_subdirectory(src/exhaust)
add_subdirectory(src/simulation)
add_subdirectory(src/diagnostics)
add_subdirectory(src/serialization)
add_subdirectory(src/catalog)
add_subdirectory(src/runtime)
add_subdirectory(src/audio)
add_subdirectory(src/app)

if(ENGINELAB_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

install(TARGETS EngineLabApp
    RUNTIME DESTINATION . COMPONENT EngineLab
    BUNDLE DESTINATION . COMPONENT EngineLab)
install(DIRECTORY engines parts DESTINATION . COMPONENT EngineLab)
install(FILES README.md LICENSE.md DESTINATION . COMPONENT EngineLab)
set(CPACK_PACKAGE_NAME "EngineLab")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_GENERATOR "ZIP")
set(CPACK_INSTALL_CMAKE_PROJECTS "${CMAKE_BINARY_DIR};${PROJECT_NAME};EngineLab;/")
include(CPack)
