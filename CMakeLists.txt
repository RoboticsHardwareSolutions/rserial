cmake_minimum_required(VERSION 3.9)
project(rserial_test
        VERSION 0.1.0
        LANGUAGES C
        DESCRIPTION
        "Simple way to use serial port ")

#################### MAIN SECTION #######################

add_library(rserial STATIC rserial.c)

target_include_directories(rserial PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

if (MSVC OR MSYS OR MINGW)
    target_compile_definitions(rserial PUBLIC -DRSERIAL_FOR_WINDOWS)
endif ()

if (APPLE)
    target_compile_definitions(rserial PUBLIC -DRSERIAL_FOR_APPLE)
endif ()

if (UNIX AND NOT APPLE)
    target_compile_options(rserial PRIVATE -std=gnu99)
    target_compile_definitions(rserial PUBLIC -DRSERIAL_FOR_UNIX)
endif ()

#################### TEST SECTION #######################

if (NOT ${CMAKE_SYSTEM_NAME} MATCHES Generic)
    add_executable(rserial_test
            test/runit/src/runit.c
            rserial.h
            rserial.c
            rserial_def.h
            test/test.c)

    target_include_directories(rserial_test PUBLIC ./ PUBLIC ./test/runit/src)

    if (MSVC OR MSYS OR MINGW)
        target_compile_definitions(rserial_test PUBLIC -DRSERIAL_FOR_WINDOWS)
    endif ()

    if (APPLE)
        target_compile_definitions(rserial_test PUBLIC -DRSERIAL_FOR_APPLE)
    endif ()

    if (UNIX AND NOT APPLE)
        target_compile_definitions(rserial_test PUBLIC -DRSERIAL_FOR_UNIX)
    endif ()
endif ()


#################### TERM SECTION #######################

if (NOT ${CMAKE_SYSTEM_NAME} MATCHES Generic)
    add_executable(rserial_term
            utils/rserial_term.c
            utils/screen.c
            utils/screen.h
            utils/tty.c
            utils/tty.h
            rserial.h
            rserial.c
            rserial_def.h
    )

    target_include_directories(rserial_term PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

    if (MSVC OR MSYS OR MINGW)
        target_compile_definitions(rserial_term PUBLIC -DRSERIAL_FOR_WINDOWS)
    endif ()

    if (APPLE)
        target_compile_definitions(rserial_term PUBLIC -DRSERIAL_FOR_APPLE)
    endif ()

    if (UNIX AND NOT APPLE)
        target_compile_definitions(rserial_term PUBLIC -DRSERIAL_FOR_UNIX)
    endif ()
endif ()


#################### FORMAT SECTION #######################

set(FILES_FOR_FORMATTING
        rserial.c
        rserial.h
        rserial_def.h
        test/test.c
)

# clang-format
find_program(clang_format NAMES clang-format)

if (NOT clang_format)
    message(STATUS "Could not locate clang-format")
else ()
    file(GLOB format_files ${FILES_FOR_FORMATTING})
    add_custom_target(rserial_format ALL COMMAND ${clang_format} -i -fallback-style=none -style=file --verbose ${format_files})
endif ()

enable_testing()
add_test(NAME rserial_test COMMAND rserial_test)
