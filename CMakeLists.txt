cmake_minimum_required(VERSION 3.24)
project(rserial
        VERSION 0.1.0
        LANGUAGES C
        DESCRIPTION
        "base64 algorithm")

#################### MAIN SECTION #######################

add_library(${PROJECT_NAME} STATIC rserial.c rserial.h rserial_def.h)
target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

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
    add_executable(${PROJECT_NAME}_test test/test.c)
    if (NOT TARGET runit)
        include(FetchContent)
        FetchContent_Declare(
                runit
                GIT_REPOSITORY https://github.com/RoboticsHardwareSolutions/runit.git
                GIT_TAG main
        )
        FetchContent_MakeAvailable(runit)
    endif ()
    target_link_libraries(${PROJECT_NAME}_test rserial runit)
    enable_testing()
    add_test(NAME ${PROJECT_NAME}_test COMMAND ${PROJECT_NAME}_test)
endif ()


#################### FORMAT SECTION #######################
include(FetchContent)
FetchContent_Declare(
        rlibhelper
        GIT_REPOSITORY https://github.com/RoboticsHardwareSolutions/rlibhelper.git
        GIT_TAG main
)
FetchContent_MakeAvailable(rlibhelper)
set(FILES_FOR_FORMATTING
        rserial.c
        rserial.h
        rserial_def.h
        test/test.c
)
if (EXISTS "${rlibhelper_SOURCE_DIR}/format.cmake")
    include(${rlibhelper_SOURCE_DIR}/format.cmake)
else ()
    message(WARNING "format.cmake file not found")
endif ()
format_files(${PROJECT_NAME} SOURCES ${FILES_FOR_FORMATTING})






