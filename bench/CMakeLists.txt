# © Joseph Cameron - All Rights Reserved

find_package(Threads REQUIRED)

file(GLOB JFC_COLLISION_BENCH_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/../src/*.cpp")

add_library(gdkcollisions_bench_objects OBJECT ${JFC_COLLISION_BENCH_SOURCES})

target_include_directories(gdkcollisions_bench_objects PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/../include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../impl/include"
    ${gdkmath_INCLUDE_DIRECTORIES})

option(JFC_COLLISION_BENCH_PROFILE "Compile the benchmark's library copy with per-phase timers" ON)

if (JFC_COLLISION_BENCH_PROFILE)
    target_compile_definitions(gdkcollisions_bench_objects PUBLIC GDK_COLLISION_PROFILE)
endif()

set_target_properties(gdkcollisions_bench_objects PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF)

if (NOT MSVC)
    target_compile_options(gdkcollisions_bench_objects PRIVATE -O2)
else()
    target_compile_options(gdkcollisions_bench_objects PRIVATE /O2)
endif()

target_compile_options(gdkcollisions_bench_objects PRIVATE ${JFC_COLLISION_FP_FLAGS})

foreach(JFC_BENCH_NAME bench grid)
    set(JFC_BENCH_TARGET "gdkcollisions_${JFC_BENCH_NAME}")

    add_executable(${JFC_BENCH_TARGET} "${CMAKE_CURRENT_LIST_DIR}/${JFC_BENCH_NAME}.cpp")

    target_link_libraries(${JFC_BENCH_TARGET}
        gdkcollisions_bench_objects
        Threads::Threads)

    set_target_properties(${JFC_BENCH_TARGET} PROPERTIES
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS OFF)

    if (NOT MSVC)
        target_compile_options(${JFC_BENCH_TARGET} PRIVATE -O2)
    else()
        target_compile_options(${JFC_BENCH_TARGET} PRIVATE /O2)
    endif()

    target_compile_options(${JFC_BENCH_TARGET} PRIVATE ${JFC_COLLISION_FP_FLAGS})
endforeach()
