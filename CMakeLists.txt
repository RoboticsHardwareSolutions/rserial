cmake_minimum_required(VERSION 3.9)
project(rserial_test
        VERSION 0.1.0
        LANGUAGES C
        DESCRIPTION
        "Simple way to use serial port ")

add_library(rserial rserial.c rserial.h)

add_executable(rserial_test
        rserial.h
        rserial.c
        test/test.c)


add_subdirectory(test/runit)

target_include_directories(rserial_test PRIVATE . )
if (NOT MSVC)
    target_link_libraries(rserial_test PRIVATE m)
endif ()


if (MSVC OR MSYS OR MINGW)
    target_compile_definitions(rserial PUBLIC -DRSERIAL_FOR_WINDOWS)
    target_link_libraries(${PROJECT_NAME} PUBLIC runit)
endif ()

if (APPLE)
    target_compile_definitions(rserial PUBLIC -DRSERIAL_FOR_APPLE)
    target_link_libraries(${PROJECT_NAME} PUBLIC runit)
endif ()

if (UNIX AND NOT APPLE)
    target_compile_definitions(rserial PUBLIC -DRSERIAL_FOR_UNIX)
    target_link_libraries(${PROJECT_NAME} PUBLIC runit)
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
    add_custom_target(format COMMAND ${clang_format} -i -fallback-style=none -style=file --verbose ${format_files})
endif ()


enable_testing()
add_test(NAME rserial_test COMMAND rserial_test)
