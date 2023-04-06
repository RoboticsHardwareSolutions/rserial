add_library(rserial rserial.c rserial.h)

if (MSVC OR MSYS OR MINGW)
    target_compile_definitions(rserial PUBLIC -DRSERIAL_FOR_WINDOWS)
endif ()

if (APPLE)
    target_compile_definitions(rserial PUBLIC -DRSERIAL_FOR_APPLE)
endif ()

if (UNIX AND NOT APPLE)
    target_compile_definitions(rserial PUBLIC -DRSERIAL_FOR_UNIX)
endif ()
