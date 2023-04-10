cmake_minimum_required(VERSION 3.9)
project(rserial_test
        VERSION 0.1.0
        LANGUAGES C
        DESCRIPTION
        "Simple way to use serial port ")

add_library(rserial rserial.c rserial.h)

add_executable(rserial_test
        test/runit/src/runit.c
        rserial.h
        rserial.c
        test/test.c)

include(test/runit/compiler_flags.cmake)

target_include_directories(rserial_test PRIVATE ./ ./test/runit/src)

if (MSVC OR MSYS OR MINGW)
    target_compile_definitions(rserial PUBLIC -DRSERIAL_FOR_WINDOWS)
endif ()

if (APPLE)
    target_compile_definitions(rserial PUBLIC -DRSERIAL_FOR_APPLE)
endif ()

if (UNIX AND NOT APPLE)
    target_compile_definitions(rserial PUBLIC -DRSERIAL_FOR_UNIX)
endif ()

set(FILES_FOR_FORMATTING
        rserial.c
        rserial.h
        test/test.c
        )

# clang-format
find_program(clang_format NAMES clang-format)
if (NOT clang_format)
    message(STATUS "Could not locate clang-format")
else ()
    file(GLOB format_files ${FILES_FOR_FORMATTING})
    message(STATUS "Using clang-format: ${clang_format}; files: ${format_files}")
    add_custom_target(format_rserial COMMAND ${clang_format} -i -fallback-style=none -style=file --verbose ${format_files})
endif ()


enable_testing()
add_test(NAME rserial_test COMMAND rserial_test)
