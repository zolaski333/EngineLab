# Fails when a source file holds UTF-8 text that was decoded as Windows-1252
# and saved again: an ellipsis becomes "â€¦", a middle dot "Â·". It has
# reached the menus and labels twice. Run as a script with -DROOT=<source dir>.
file(GLOB_RECURSE files
    "${ROOT}/src/*.cpp" "${ROOT}/src/*.hpp" "${ROOT}/src/*.h"
    "${ROOT}/tools/*.cpp" "${ROOT}/tools/*.hpp"
    "${ROOT}/tests/*.cpp" "${ROOT}/tests/*.hpp")
set(found "")
foreach(file IN LISTS files)
    file(READ "${file}" text)
    foreach(marker IN ITEMS "Â" "Ã" "â€")
        string(FIND "${text}" "${marker}" at)
        if(NOT at EQUAL -1)
            list(APPEND found "${file}")
            break()
        endif()
    endforeach()
endforeach()
list(LENGTH files count)
if(found)
    message(FATAL_ERROR "mis-encoded text (UTF-8 read as Windows-1252) in: ${found}")
endif()
message(STATUS "source encoding: ${count} files clean")
