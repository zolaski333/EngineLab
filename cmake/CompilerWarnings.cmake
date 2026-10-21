# JUCE's module units are compiled INTO the targets that link them (they arrive
# as interface sources), so they would inherit the warning set below. They are
# not our code and they do not build clean under -Wconversion, which drowned
# EngineLab's own 125 diagnostics in ~400 of JUCE's on the first Linux build.
# The categories JUCE trips are turned off for those units only; our own
# sources keep the full set. JUCE's module units are one level deep inside
# `modules/` by construction, so a single glob finds them.
function(enginelab_quiet_vendor_units)
    if(MSVC OR NOT juce_SOURCE_DIR)
        return()
    endif()
    get_property(units GLOBAL PROPERTY ENGINELAB_VENDOR_UNITS)
    if(NOT units)
        file(GLOB units "${juce_SOURCE_DIR}/modules/*/*.cpp")
        set_property(GLOBAL PROPERTY ENGINELAB_VENDOR_UNITS "${units}")
    endif()
    if(units)
        # A source property resolves in the directory that defines the target,
        # which is this function's caller: a function is not a new scope.
        set_source_files_properties(${units} PROPERTIES COMPILE_OPTIONS
            "-Wno-conversion;-Wno-float-conversion;-Wno-missing-field-initializers;-Wno-shadow;-Wno-comment")
    endif()
endfunction()

# A vendored *header* is compiled as part of whatever includes it, so no source
# property can reach it. One line of JUCE's `juce_UMPUtils.h` broke fourteen of
# our translation units on the first Linux build with warnings as errors, and
# yaml-cpp's `impl.h`, `emitterstyle.h` and `type.h` broke more under Clang,
# whose `-Wconversion` pulls in `-Wsign-conversion` where GCC's does not.
# Re-publishing a dependency's include directories as SYSTEM ones is what tells
# GCC and Clang to keep quiet about code we do not own.
# `FetchContent_Declare(... SYSTEM)` says the same in one word, but it needs
# CMake 3.25 and this project supports 3.24.
function(enginelab_vendor_headers_are_system)
    if(MSVC)
        return()
    endif()
    foreach(target IN LISTS ARGN)
        if(NOT TARGET ${target})
            continue()
        endif()
        # A property cannot be set through an alias, and these dependencies
        # publish one each (`yaml-cpp::yaml-cpp`, `juce::juce_core`), so follow
        # it to the real target first.
        get_target_property(aliased ${target} ALIASED_TARGET)
        if(aliased)
            set(target ${aliased})
        endif()
        get_target_property(includes ${target} INTERFACE_INCLUDE_DIRECTORIES)
        if(includes)
            set_property(TARGET ${target} APPEND PROPERTY
                INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${includes}")
        endif()
    endforeach()
endfunction()

function(enginelab_set_warnings target)
    if(MSVC)
        # C4324 reports the intentional cache-line padding used by SpscQueue.
        target_compile_options(${target} PRIVATE /W4 /permissive- /Zc:__cplusplus /utf-8 /wd4324)
        if(ENGINELAB_ENABLE_WARNINGS_AS_ERRORS)
            target_compile_options(${target} PRIVATE /WX)
        endif()
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
        if(ENGINELAB_ENABLE_WARNINGS_AS_ERRORS)
            target_compile_options(${target} PRIVATE -Werror)
        endif()
        enginelab_quiet_vendor_units()
    endif()
endfunction()
